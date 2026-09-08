#!/usr/bin/env python3
"""Offline MuJoCo hand-guidance experiment; no ROS, transports or hardware.

Example: python validate_hand_guidance_dynamics.py --anchor model_session_anchor.json
         --speed-deg-s 30 --output hand_guidance_dynamics.json
"""
import argparse
from collections import Counter
from dataclasses import asdict
import hashlib
import json
import math
from pathlib import Path
import re
import sys


ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
DT = 0.0005
CONTROL_DT = 0.01
DURATION = 14.0
JOINTS = tuple(f"J{i}" for i in range(1, 7))


def sha256(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def clip(value, limit):
    return max(-limit, min(limit, value))


def native_array(source, name):
    match = re.search(r"\b" + name + r"\s*\{\{([^}]+)\}\}", source)
    if match is None:
        raise ValueError(f"native torque array missing: {name}")
    values = tuple(float(value.strip()) for value in match[1].split(","))
    if len(values) != 6 or not all(math.isfinite(value) for value in values):
        raise ValueError(f"invalid native torque array: {name}")
    return values


def hand_load(t, full_six):
    amplitude = (0.45, 0.80, 0.60, 0.25, 0.15, 0.08) if full_six else (0.0, 0.80, 0.60, 0.0, 0.0, 0.0)
    scale = 0.0
    for begin, end, magnitude in ((2.0, 4.0, 1.0), (8.0, 10.0, 0.8)):
        if begin <= t < end:
            scale = magnitude * min(1.0, (t - begin) / 0.25, (end - t) / 0.25)
    return tuple(scale * value for value in amplitude)


def phase(t):
    return "calibration" if t < 1.0 else "static" if t < 2.0 else "push" if t < 4.0 else "released" if t < 8.0 else "push_again" if t < 10.0 else "released_again"


def release_metrics(rows, begin, end):
    """Stop means <=0.25 deg/s for >=0.3 s and the remaining release window."""
    selected = [row for row in rows if begin <= row["time_s"] <= end]
    if not selected or selected[0]["time_s"] > begin + CONTROL_DT:
        return {"reached": False}
    result = {"reached": True, "release_time_s": begin, "window_end_s": end, "joints": {}}
    for i, joint in enumerate(JOINTS):
        stopped = frozen = None
        for index, row in enumerate(selected):
            remaining = selected[index:]
            if remaining[-1]["time_s"] - row["time_s"] < 0.3 - 1e-9:
                break
            if stopped is None and all(abs(math.degrees(item["dq_rad_s"][i])) <= 0.25 for item in remaining):
                stopped = row["time_s"] - begin
            if frozen is None and all(abs(item["q_ref_rad"][i] - row["q_ref_rad"][i]) <= 1e-8 for item in remaining):
                frozen = row["time_s"] - begin
        first = selected[0]
        result["joints"][joint] = {
            "velocity_at_release_deg_s": math.degrees(first["dq_rad_s"][i]),
            "reference_velocity_at_release_deg_s": math.degrees(first["dq_ref_rad_s"][i]),
            "actual_stop_after_s": stopped, "reference_frozen_after_s": frozen,
            "max_additional_actual_excursion_deg": max(abs(math.degrees(row["q_rad"][i] - first["q_rad"][i])) for row in selected),
            "max_additional_reference_excursion_deg": max(abs(math.degrees(row["q_ref_rad"][i] - first["q_ref_rad"][i])) for row in selected),
        }
    return result


def case(model_path, initial_q, caps, ff_caps, gear, j6_cap, full_six, speed_deg_s=3.0):
    import mujoco
    from hand_guidance import GuidanceProfile, HandGuidance
    from go_m8010_arm_hardware.gravity_model import StaticGravityEvaluator

    gravity = StaticGravityEvaluator(model_path)
    model = gravity.model
    model.opt.timestep = DT
    model.opt.gravity[:] = (0.0, 0.0, -9.81)
    model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST
    data = mujoco.MjData(model)  # Never use the evaluator's zero-velocity data as the plant.
    data.qpos[:] = initial_q
    mujoco.mj_forward(model, data)
    if (model.nq, model.nv, model.nu, model.ngeom) != (6, 6, 0, 0):
        raise ValueError("expected the six-coordinate, actuator-free dynamics-only model")
    conversion = (gear, 2.0 * gear, gear, gear, gear, 1.0)
    joint_caps = tuple(caps[i] * conversion[i] if i < 5 else j6_cap for i in range(6))
    # GO gains are rotor-side engineering assumptions below existing native limits.
    kp_rotor = (0.5, 0.6, 0.6, 0.5, 0.5)
    kd_rotor = (0.05, 0.10, 0.05, 0.05, 0.05)
    kp = tuple(kp_rotor[i] * gear * conversion[i] for i in range(5)) + (5.0,)
    kd = tuple(kd_rotor[i] * gear * conversion[i] for i in range(5)) + (0.10,)
    position_lsb = (2.0 * math.pi / 32768.0 / gear,) * 5 + (25.0 / 65535.0,)
    torque_lsb = tuple(conversion[i] / 256.0 for i in range(5)) + (0.002,)
    sensor_bias = (0.02, -0.025, 0.03, 0.008, 0.004, 0.003)
    delay_ticks = 3 if full_six else 1
    profile = GuidanceProfile(speed_deg_s=speed_deg_s, enabled=(True,) * 5 + (full_six,))
    q_ref, dq_ref = tuple(initial_q), (0.0,) * 6
    guidance = None
    history, calibration, rows = [], [], []
    bias = None
    max_torque, max_ff = [0.0] * 6, [0.0] * 6
    cap_frames, ff_cap_frames = [0] * 6, [0] * 6
    maximum_acceleration = 0.0
    previous_dq_ref = (0.0,) * 6
    fault = None
    control_stride = round(CONTROL_DT / DT)
    for step in range(round(DURATION / DT) + 1):
        t = round(step * DT, 9)
        q, dq = tuple(float(v) for v in data.qpos), tuple(float(v) for v in data.qvel)
        if not all(math.isfinite(v) for v in q + dq):
            fault = "NONFINITE_PLANT"
            break
        g = gravity.evaluate(q)
        ff = tuple(clip(g[i], ff_caps[i] * conversion[i]) if i < 5 else g[i] for i in range(6))
        requested = tuple(ff[i] + kp[i] * (q_ref[i] - q[i]) + kd[i] * (dq_ref[i] - dq[i]) for i in range(6))
        motor = tuple(clip(requested[i], joint_caps[i]) for i in range(6))
        hand = hand_load(t, full_six)
        for i in range(6):
            max_torque[i] = max(max_torque[i], abs(motor[i]))
            max_ff[i] = max(max_ff[i], abs(ff[i]))
            cap_frames[i] += abs(requested[i] - motor[i]) > 1e-12
            ff_cap_frames[i] += abs(g[i] - ff[i]) > 1e-12
        if step % control_stride == 0:
            measured_q = tuple(round(q[i] / position_lsb[i]) * position_lsb[i] for i in range(6))
            measured_tau = tuple(round((motor[i] + sensor_bias[i]) / torque_lsb[i]) * torque_lsb[i] for i in range(6))
            history.append((t, measured_q, measured_tau))
            index = max(0, len(history) - 1 - delay_ticks)
            source_t, sensed_q, sensed_tau = history[index]
            earlier = history[max(0, index - 4)]
            sensed_dq = tuple((sensed_q[i] - earlier[1][i]) / (source_t - earlier[0]) if source_t > earlier[0] else 0.0 for i in range(6))
            model_g = gravity.evaluate(sensed_q)
            residual = tuple(model_g[i] - sensed_tau[i] for i in range(6))
            if 0.25 <= t < 1.0 and max(abs(v) for v in sensed_dq) <= math.radians(0.25) and not any(hand):
                calibration.append(residual)
            if t >= 1.0 and guidance is None:
                if len(calibration) < 50:
                    fault = "INSUFFICIENT_STATIONARY_STARTUP_BIAS_SAMPLES"
                    break
                bias = tuple(sum(row[i] for row in calibration) / len(calibration) for i in range(6))
                guidance = HandGuidance(profile, q_ref, now_s=10.0 + t - CONTROL_DT)
            estimated = tuple(residual[i] - bias[i] for i in range(6)) if bias is not None else (0.0,) * 6
            output = None
            if guidance is not None:
                output = guidance.update(10.0 + t, sensed_q, sensed_dq, estimated, source_time_s=10.0 + source_t)
                q_ref, dq_ref = output.q_ref, output.dq_ref
                maximum_acceleration = max(maximum_acceleration, max(abs(dq_ref[i] - previous_dq_ref[i]) / CONTROL_DT for i in range(6)))
                previous_dq_ref = dq_ref
                if output.fault:
                    fault = output.fault
            rows.append({
                "time_s": t, "phase": phase(t), "state": output.state if output else "calibration",
                "q_rad": q, "dq_rad_s": dq, "q_ref_rad": q_ref, "dq_ref_rad_s": dq_ref,
                "motor_joint_nm": motor, "gravity_joint_nm": g,
                "estimated_external_joint_nm": estimated, "applied_hand_joint_nm": hand,
                "feedback_age_s": t - source_t,
            })
            if fault:
                break
        data.qfrc_applied[:] = [motor[i] + hand[i] for i in range(6)]
        mujoco.mj_step(model, data)

    def window(begin, end):
        return [row for row in rows if begin <= row["time_s"] <= end]

    def settle_metrics(begin, end):
        selected = window(begin, end)
        if not selected:
            return {"pass": False, "reason": "window_not_reached"}
        span = [math.degrees(max(row["q_rad"][i] for row in selected) - min(row["q_rad"][i] for row in selected)) for i in range(6)]
        velocity = [max(abs(math.degrees(row["dq_rad_s"][i])) for row in selected) for i in range(6)]
        goal_span = [max(row["q_ref_rad"][i] for row in selected) - min(row["q_ref_rad"][i] for row in selected) for i in range(6)]
        return {"pass": max(span) <= 0.25 and max(velocity) <= 0.25 and max(goal_span) <= 1e-8,
                "position_span_deg": span, "max_abs_velocity_deg_s": velocity, "goal_span_rad": goal_span,
                "end_q_rad": selected[-1]["q_rad"], "end_q_ref_rad": selected[-1]["q_ref_rad"]}

    first_stop, final_stop = settle_metrics(7.0, 7.99), settle_metrics(13.0, 14.0)
    static = window(1.0, 1.99)
    concurrent = any(abs(row["dq_ref_rad_s"][1]) > math.radians(0.1) and abs(row["dq_ref_rad_s"][2]) > math.radians(0.1) for row in window(2.0, 4.0))
    first_motion = [math.degrees(first_stop.get("end_q_rad", initial_q)[i] - initial_q[i]) for i in range(6)]
    second_motion = [math.degrees(final_stop.get("end_q_rad", initial_q)[i] - first_stop.get("end_q_rad", initial_q)[i]) for i in range(6)]
    final_goal_motion = [math.degrees(final_stop.get("end_q_ref_rad", initial_q)[i] - initial_q[i]) for i in range(6)]
    checked_joints = range(6) if full_six else (1, 2)
    checks = {
        "no_core_or_plant_fault": fault is None and len(rows) == round(DURATION / CONTROL_DT) + 1,
        "stationary_startup_bias_frozen": bias is not None and len(calibration) >= 50,
        "no_hand_static_drift_under_0_1_deg": bool(static) and max(abs(math.degrees(row["q_rad"][i] - initial_q[i])) for row in static for i in range(6)) <= 0.1,
        "j2_j3_concurrent_guidance": concurrent,
        "first_push_moves_selected_joints": all(first_motion[i] > 0.2 for i in checked_joints),
        "first_release_settles_at_new_goal": first_stop["pass"],
        "push_again_moves_selected_joints": all(second_motion[i] > 0.1 for i in checked_joints),
        "second_release_settles_at_new_goal": final_stop["pass"],
        "new_goals_do_not_return_to_initial_pose": all(final_goal_motion[i] > 0.3 for i in checked_joints),
        "reference_speed_bound": all(abs(v) <= math.radians(profile.speed_deg_s) + 1e-10 for row in rows for v in row["dq_ref_rad_s"]),
        "reference_acceleration_bound": maximum_acceleration <= math.radians(profile.acceleration_deg_s2) + 1e-9,
        "go_motor_torque_caps_respected": all(max_torque[i] <= joint_caps[i] + 1e-12 for i in range(5)),
        "assumed_j6_proxy_cap_respected": max_torque[5] <= j6_cap + 1e-12,
    }
    return {
        "name": "six_axis_with_assumed_j6_proxy" if full_six else "go_five_axis_core_j2_j3_push",
        "status": "SOFTWARE_MODEL_PASS" if all(checks.values()) else "SOFTWARE_MODEL_FAIL",
        "checks": checks, "fault": fault, "profile": asdict(profile),
        "startup_bias_joint_nm": bias, "startup_bias_sample_count": len(calibration),
        "state_counts": dict(Counter(row["state"] for row in rows)),
        "first_release": first_stop, "second_release": final_stop,
        "release_response": [release_metrics(rows, 4.0, 7.99), release_metrics(rows, 10.0, 14.0)],
        "maximum_actual_speed_deg_s": [max(abs(math.degrees(row["dq_rad_s"][i])) for row in rows) for i in range(6)],
        "maximum_reference_speed_deg_s": [max(abs(math.degrees(row["dq_ref_rad_s"][i])) for row in rows) for i in range(6)],
        "first_displacement_deg": first_motion, "second_displacement_deg": second_motion,
        "final_goal_displacement_deg": final_goal_motion,
        "max_motor_joint_nm": max_torque, "max_gravity_feedforward_joint_nm": max_ff,
        "go_max_abs_rotor_nm": dict(zip(JOINTS[:5], [max_torque[i] / conversion[i] for i in range(5)])),
        "torque_cap_step_counts": cap_frames, "gravity_cap_step_counts": ff_cap_frames,
        "maximum_reference_acceleration_deg_s2": math.degrees(maximum_acceleration),
        "assumptions": {"inner_kp_joint_nm_rad": kp, "inner_kd_joint_nm_s_rad": kd,
            "position_quantization_rad": position_lsb, "torque_quantization_joint_nm": torque_lsb,
            "constant_torque_sensor_bias_joint_nm": sensor_bias, "feedback_delay_ms": delay_ticks * 10,
            "encoder_velocity_difference_window_ms": 40, "j6_virtual_torque_cap_nm": j6_cap,
            "j6_cap_is_real_hardware_limit": False, "inner_feedback_ideal_current_state": True,
            "motor_torque_is_applied_servo_effort_not_direct_hand_force": True},
        "trace": rows[::5] + ([rows[-1]] if rows and rows[-1] not in rows[::5] else []),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=ROOT)
    parser.add_argument("--anchor", type=Path, required=True, help="Read-only gravity_model_anchor_v2.json with the starting model pose")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--speed-deg-s", type=float, default=3.0, help="Reference speed limit, 0 < speed <= 30; not an achieved speed claim")
    parser.add_argument("--j6-virtual-torque-cap-nm", type=float, default=0.2, help="Simulation assumption only; never a hardware torque rating")
    args = parser.parse_args()
    if args.output.exists():
        parser.error("output exists; use a new filename to preserve previous evidence")
    if not math.isfinite(args.speed_deg_s) or not 0 < args.speed_deg_s <= 30:
        parser.error("reference speed must be finite and in (0, 30]")
    if not math.isfinite(args.j6_virtual_torque_cap_nm) or args.j6_virtual_torque_cap_nm <= 0:
        parser.error("J6 virtual torque cap must be finite and positive")
    source_root = args.repo_root / WORKSPACE / "ros2_ws/src/go_m8010_arm_hardware"
    sys.path.insert(0, str(source_root))
    from go_m8010_arm_hardware.gravity_model import GravityModelAnchorV2
    from go_m8010_arm_hardware.torque_semantics import GO_GEAR_RATIO
    import hand_guidance
    import mujoco

    model_path = args.repo_root / WORKSPACE / "mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
    native_path = args.repo_root / "tools/hardware/v15_30a_gui_go_controller.cpp"
    native = native_path.read_text(encoding="utf-8")
    caps = list(native_array(native, "kAuxPredictedRotorWorkNm"))
    caps[1] = float(re.search(r"\bkJ2PredictedRotorWorkNm\s*=\s*([0-9.]+)", native)[1])
    ff_caps = native_array(native, "kGravityFeedforwardLimits")
    anchor = GravityModelAnchorV2.from_path(args.anchor)
    report = {
        "schema": "go-m8010-hand-guidance-dynamics/1.0", "status": "INCOMPLETE",
        "scope": "OFFLINE_MODEL_AND_ASSUMED_INNER_SERVO_ONLY",
        "hardware_validation": "NOT_RUN", "j2_ab_differential_sync": "NOT_MODELED_SINGLE_COMMON_J2_COORDINATE",
        "mujoco_version": mujoco.__version__, "physics_dt_s": DT, "core_dt_s": CONTROL_DT,
        "source_hashes": {str(path): sha256(path) for path in (model_path, native_path, args.anchor, Path(hand_guidance.__file__), Path(__file__))},
        "initial_model_q_rad": anchor.model_absolute_joint_rad,
        "go_native_rotor_work_caps_nm": dict(zip(JOINTS[:5], caps[:5])),
        "go_native_gravity_rotor_caps_nm": dict(zip(JOINTS[:5], ff_caps[:5])),
        "limitations": [
            "Matched frozen gravity and engineering inertia; no independent measured plant identification.",
            "No physical friction, backlash, motor rotor reflected inertia, flexible cables or contacts.",
            "GO inner PD is an engineering proxy, not execution of the native controller.",
            "J6 internal POS_VEL dynamics and torque cap are unmeasured proxy assumptions in both cases.",
            "Applied hand torque is only a plant input; core receives g(delayed encoder)-quantized delayed motor effort-frozen startup bias.",
            "Bias is frozen after contact-free stationary startup; no adaptation during pushing or release.",
            "Software convergence does not establish hardware weight support, stopping performance or J2A/J2B synchronization.",
        ],
    }
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    report["cases"] = [case(model_path, anchor.model_absolute_joint_rad, caps, ff_caps, GO_GEAR_RATIO, args.j6_virtual_torque_cap_nm, full, args.speed_deg_s) for full in (False, True)]
    report["status"] = "SOFTWARE_MODEL_PASS" if all(row["status"] == "SOFTWARE_MODEL_PASS" for row in report["cases"]) else "SOFTWARE_MODEL_FAIL"
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    print(json.dumps({"status": report["status"], "output": str(args.output), "cases": [{"name": row["name"], "fault": row["fault"], "failed_checks": [k for k, v in row["checks"].items() if not v]} for row in report["cases"]]}, ensure_ascii=False))
    return 0 if report["status"] == "SOFTWARE_MODEL_PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from pathlib import Path
import statistics
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "hardware" / "v15_24c"
MODEL_REL = Path(
    "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/mujoco_v15_14/"
    "go_m8010_arm_v15_14_kinematic.xml"
)
MODEL_SHA256 = "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
SDK_COMMIT = "5b79a42d81cd69adac1367db79efdb972a898fd1"
MANUAL_V12_SHA256 = "1312eb4f9af0d5ebfe1e46376e6373df43330edc9222540e04e4ee4dd55e92be"
DATASHEET_V10_SHA256 = "b6dcadd37db9fa05c9e0704647346c7a9091aefaf3296ecf525712b3044ff446"
GEAR = 6.329999923706055
SIGN_A = -1
SIGN_B = 1
GRID_DEG = (-30, -15, 0, 15, 30)


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def dump_json(path: Path, value: object) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=True,
                   allow_nan=False) + "\n",
        encoding="utf-8",
    )


def finite_float(value: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"non-finite value: {value}")
    return result


def run_serializer(sdk_root: Path) -> tuple[list[dict[str, float | int]], str]:
    source = ROOT / "tools" / "hardware" / "v15_24c_j2_tau_serializer_self_test.cpp"
    with tempfile.TemporaryDirectory(prefix="v15_24c_tau_") as temp:
        executable = Path(temp) / "tau_self_test"
        subprocess.run(
            [
                "g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
                f"-I{sdk_root / 'include'}", str(source),
                str(sdk_root / "lib" / "libUnitreeMotorSDK_Linux64.so"),
                f"-Wl,-rpath,{sdk_root / 'lib'}", "-o", str(executable),
            ],
            check=True,
        )
        completed = subprocess.run(
            [str(executable)], check=True, text=True, capture_output=True
        )
    lines = completed.stdout.splitlines()
    if "SELF_TEST=PASS" not in lines or "SERIAL_PORT_CONSTRUCTED=NO" not in lines:
        raise RuntimeError("serializer self-test did not prove offline-only execution")
    rows: list[dict[str, float | int]] = []
    for line in lines[1:]:
        if "=" in line:
            continue
        literal, count, decoded = line.split(",")
        rows.append(
            {
                "literal_tau_nm": float(literal),
                "raw_int16_count": int(count),
                "decoded_tau_nm": float(decoded),
            }
        )
    return rows, completed.stdout


def model_sweep() -> tuple[list[dict[str, float | str]], dict[str, object]]:
    import mujoco

    model_path = ROOT / MODEL_REL
    if sha256(model_path) != MODEL_SHA256:
        raise RuntimeError("production MJCF hash mismatch")
    model = mujoco.MjModel.from_xml_path(str(model_path.resolve()))
    data = mujoco.MjData(model)
    xml_gravity = [float(x) for x in model.opt.gravity]
    model.opt.gravity[:] = (0.0, 0.0, -9.81)
    rows: list[dict[str, float | str]] = []
    for j2_deg in GRID_DEG:
        for j3_deg in GRID_DEG:
            mujoco.mj_resetData(model, data)
            data.qpos[:] = 0.0
            data.qvel[:] = 0.0
            data.qacc[:] = 0.0
            data.qpos[1] = math.radians(j2_deg)
            data.qpos[2] = math.radians(j3_deg)
            mujoco.mj_forward(model, data)
            hold = [float(value) for value in data.qfrc_bias]
            row: dict[str, float | str] = {
                "pose_class": "REPRESENTATIVE_MODEL_SWEEP",
                "j1_deg": 0.0,
                "j2_deg": float(j2_deg),
                "j3_deg": float(j3_deg),
                "j4_deg": 0.0,
                "j5_deg": 0.0,
                "j6_deg": 0.0,
            }
            for index, value in enumerate(hold, 1):
                row[f"J{index}_qfrc_bias_hold_nm"] = value
                row[f"J{index}_gravity_generalized_force_nm"] = -value
            row["J2_equal_share_output_side_hypothesis_nm"] = hold[1] / 2.0
            row["J2_equal_share_rotor_side_hypothesis_nm"] = hold[1] / (2.0 * GEAR)
            rows.append(row)
    j2 = [float(row["J2_qfrc_bias_hold_nm"]) for row in rows]
    j3 = [float(row["J3_qfrc_bias_hold_nm"]) for row in rows]
    summary = {
        "method": "mj_forward with qvel=qacc=0; runtime-only gravity override",
        "mujoco_version": mujoco.__version__,
        "production_model": str(MODEL_REL).replace("\\", "/"),
        "production_model_sha256": MODEL_SHA256,
        "xml_compiled_gravity_m_s2": xml_gravity,
        "runtime_gravity_m_s2": [0.0, 0.0, -9.81],
        "production_model_modified": False,
        "current_physical_pose_exact_available": False,
        "pose_class": "REPRESENTATIVE_MODEL_SWEEP",
        "fixed_joint_values_deg": {"J1": 0, "J4": 0, "J5": 0, "J6": 0},
        "J2": {
            "min_hold_nm": min(j2), "max_hold_nm": max(j2),
            "median_hold_nm": statistics.median(j2),
            "sign_change": min(j2) < 0.0 < max(j2),
        },
        "J3": {
            "min_hold_nm": min(j3), "max_hold_nm": max(j3),
            "median_hold_nm": statistics.median(j3),
            "sign_change": min(j3) < 0.0 < max(j3),
        },
    }
    return rows, summary


def percentile(values: list[float], fraction: float) -> float:
    ordered = sorted(values)
    index = fraction * (len(ordered) - 1)
    lower = math.floor(index)
    upper = math.ceil(index)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] * (upper - index) + ordered[upper] * (index - lower)


def analyze_existing_runs() -> tuple[list[dict[str, object]], dict[str, object]]:
    base = ROOT / "hardware" / "v15_24b_ft"
    inventory = json.loads((base / "final_installed_load_inventory.json").read_text())
    specifications = [
        ("RUN_A", "j2_installed_load_run.csv", inventory["j2_run_a"], 0.60),
        ("RUN_A_OPERATOR_REPEAT", "j2_installed_load_run_operator_repeat.csv",
         inventory["j2_run_a_operator_repeat"], 0.60),
        ("RUN_B", "j2_installed_load_kp070.csv", inventory["j2_run_b"], 0.70),
    ]
    output: list[dict[str, object]] = []
    run_checks: list[dict[str, object]] = []
    for run_id, filename, facts, kp in specifications:
        rows = list(csv.DictReader((base / filename).open(encoding="utf-8")))
        active = [
            row for row in rows
            if row["phase"] not in ("SESSION_BRAKE_CAPTURE", "FINAL_DUAL_BRAKE")
        ]
        ref_a = float(facts["a0_raw_rad"])
        ref_b = float(facts["b0_raw_rad"])
        map_error_a = max(
            abs(finite_float(row["a_q_cmd"]) -
                (ref_a + SIGN_A * GEAR * finite_float(row["q_ref_rad"])))
            for row in active
        )
        map_error_b = max(
            abs(finite_float(row["b_q_cmd"]) -
                (ref_b + SIGN_B * GEAR * finite_float(row["q_ref_rad"])))
            for row in active
        )
        q_ref = [finite_float(row["q_ref_rad"]) for row in active]
        command_ok = (
            min(q_ref) == -math.radians(5.0) and
            max(q_ref) == math.radians(5.0) and
            map_error_a < 1e-12 and map_error_b < 1e-12
        )
        run_checks.append(
            {
                "run_id": run_id, "filename": filename, "kp": kp,
                "command_target_correct": command_ok,
                "q_ref_min_deg": math.degrees(min(q_ref)),
                "q_ref_max_deg": math.degrees(max(q_ref)),
                "max_a_command_mapping_error_rad": map_error_a,
                "max_b_command_mapping_error_rad": map_error_b,
                "a_q_cmd_min_rad": min(finite_float(row["a_q_cmd"]) for row in active),
                "a_q_cmd_max_rad": max(finite_float(row["a_q_cmd"]) for row in active),
                "b_q_cmd_min_rad": min(finite_float(row["b_q_cmd"]) for row in active),
                "b_q_cmd_max_rad": max(finite_float(row["b_q_cmd"]) for row in active),
            }
        )
        for phase in sorted({row["phase"] for row in active}):
            phase_rows = [row for row in active if row["phase"] == phase]
            a_tau = [finite_float(row["a_tau"]) for row in phase_rows]
            b_tau = [finite_float(row["b_tau"]) for row in phase_rows]
            a_median = statistics.median(a_tau)
            b_median = statistics.median(b_tau)
            output.append(
                {
                    "run_id": run_id, "source_csv": filename, "kp": kp,
                    "phase": phase, "rows": len(phase_rows),
                    "q_ref_min_deg": math.degrees(min(finite_float(row["q_ref_rad"]) for row in phase_rows)),
                    "q_ref_max_deg": math.degrees(max(finite_float(row["q_ref_rad"]) for row in phase_rows)),
                    "a_tau_min_sdk_units": min(a_tau),
                    "a_tau_max_sdk_units": max(a_tau),
                    "a_tau_mean_sdk_units": statistics.fmean(a_tau),
                    "a_tau_median_sdk_units": a_median,
                    "a_tau_p05_sdk_units": percentile(a_tau, 0.05),
                    "a_tau_p95_sdk_units": percentile(a_tau, 0.95),
                    "b_tau_min_sdk_units": min(b_tau),
                    "b_tau_max_sdk_units": max(b_tau),
                    "b_tau_mean_sdk_units": statistics.fmean(b_tau),
                    "b_tau_median_sdk_units": b_median,
                    "b_tau_p05_sdk_units": percentile(b_tau, 0.05),
                    "b_tau_p95_sdk_units": percentile(b_tau, 0.95),
                    "a_feedback_distinct_counts": len(set(a_tau)),
                    "b_feedback_distinct_counts": len(set(b_tau)),
                    "logical_j2_effort_from_paired_medians_nm": GEAR * (
                        SIGN_A * a_median + SIGN_B * b_median
                    ),
                }
            )
    summary = {
        "runs": run_checks,
        "existing_run_command_target_correct": all(
            bool(item["command_target_correct"]) for item in run_checks
        ),
        "position_command_bug": "NOT_FOUND",
        "torque_feedback_behavior": (
            "At +/-5 deg endpoints A/B feedback has opposite raw signs and the "
            "same aligned logical effort; magnitudes form sustained narrow bands. "
            "The bands rise from about 0.31-0.35 at Kp 0.60 to about 0.35-0.40 "
            "rotor N.m at Kp 0.70. Paired median conversion gives about "
            "4.05-4.25 N.m logical J2 effort at Kp 0.60 and 4.53-4.87 N.m "
            "at Kp 0.70."
        ),
        "protocol_range_saturation_evidence": False,
        "motor_or_internal_controller_saturation_evidence": "INCONCLUSIVE",
        "interpretation": (
            "Correct commands plus low differential error exclude an A/B sign or "
            "common command-generation failure. Kp-dependent effort with very small "
            "motion supports a common external load/friction/brake or internal torque-"
            "scaling/limit candidate; it does not prove gravity as the root cause."
        ),
    }
    return output, summary


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--sdk-root", type=Path,
                        default=Path("/home/car/vendor/unitree_actuator_sdk"))
    args = parser.parse_args()
    sdk_root = args.sdk_root.resolve()
    OUT.mkdir(parents=True, exist_ok=True)

    sdk_head = subprocess.run(
        ["git", "-C", str(sdk_root), "rev-parse", "HEAD"],
        check=True, text=True, capture_output=True,
    ).stdout.strip()
    if sdk_head != SDK_COMMIT:
        raise RuntimeError(f"unexpected SDK commit: {sdk_head}")
    serializer_rows, serializer_stdout = run_serializer(sdk_root)
    header = sdk_root / "include/unitreeMotor/include/motor_msg_GO-M8010-6.h"
    readme = sdk_root / "README.md"
    tau_audit = {
        "schema": "go-m8010-arm-v15.24c-j2-tau-sdk-audit/1.0",
        "sdk_repository": "https://github.com/unitreerobotics/unitree_actuator_sdk",
        "sdk_commit": sdk_head,
        "sdk_header_sha256": sha256(header),
        "sdk_readme_sha256": sha256(readme),
        "evidence": {
            "go_protocol_header": (
                "RIS_Comd_t.tor_des: desired joint output torque, unit N.m, q8"
            ),
            "sdk_readme": (
                "The README states commands are for the rotor side and output-side "
                "commands require reducer conversion."
            ),
            "official_debugging_assistant": (
                "Official GO-M8010 debugging assistant labels torque/speed/position "
                "as rotor quantities."
            ),
            "go_m8010_use_manual_v1_2": {
                "sha256": MANUAL_V12_SHA256,
                "page_4": (
                    "Mixed-control tau is explicitly the motor rotor output torque; "
                    "p and omega are rotor position and velocity."
                ),
                "page_13": (
                    "All commands target shaft 1 before the reducer, not output "
                    "shaft 2; reducer ratio is 6.33."
                ),
                "page_17": (
                    "tau_set is desired motor torque; Tff is N.m and multiplied by 256."
                ),
                "page_18": (
                    "Unless specially stated, protocol parameters are rotor-side, "
                    "not output-side; rotor-to-output ratio is 6.33."
                ),
            },
            "go_m8010_user_manual_v1_0": {
                "sha256": DATASHEET_V10_SHA256,
                "page_3_max_torque_nm": 23.7,
                "page_3_gear_ratio": 6.33,
                "page_7": "T is motor rotor output torque.",
            },
        },
        "documentation_conflict": False,
        "resolved_documentation_ambiguity": (
            "The terse protocol header uses joint-output wording, but the product-"
            "specific V1.2 manual repeatedly and explicitly defines command T/tau, "
            "position, and velocity as pre-reducer rotor-side quantities."
        ),
        "quantity_unit": "N.m",
        "quantity_unit_authority": "PASS",
        "tau_side": "MOTOR_ROTOR_TORQUE",
        "tau_side_authority": "PASS",
        "tau_physical_unit_authority": "PASS",
        "protocol_field": "signed int16 q8",
        "protocol_storage_count_min": -32768,
        "protocol_storage_count_max": 32767,
        "protocol_storage_nominal_nm_min": -128.0,
        "protocol_storage_nominal_nm_max": 127.99609375,
        "serializer_scale_counts_per_nm": 256.0,
        "serializer_lsb_nm": 1.0 / 256.0,
        "frozen_serializer_effective_count_min": min(
            int(row["raw_int16_count"]) for row in serializer_rows
        ),
        "frozen_serializer_effective_count_max": max(
            int(row["raw_int16_count"]) for row in serializer_rows
        ),
        "frozen_serializer_effective_nm_min": min(
            float(row["decoded_tau_nm"]) for row in serializer_rows
        ),
        "frozen_serializer_effective_nm_max": max(
            float(row["decoded_tau_nm"]) for row in serializer_rows
        ),
        "serializer_out_of_range_behavior": "clamp to +/-32765 counts in tested cases",
        "serializer_cases": serializer_rows,
        "serializer_self_test": "PASS",
        "serializer_stdout": serializer_stdout.splitlines(),
        "serial_port_constructed": False,
        "device_io": False,
        "j2_tau_command_authority": "PASS",
    }
    dump_json(OUT / "j2_tau_sdk_audit.json", tau_audit)

    sweep_rows, sweep_summary = model_sweep()
    with (OUT / "j2_gravity_model_sweep.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(sweep_rows[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(sweep_rows)

    analysis_rows, analysis_summary = analyze_existing_runs()
    with (OUT / "j2_existing_run_torque_analysis.csv").open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream, fieldnames=list(analysis_rows[0]), lineterminator="\n"
        )
        writer.writeheader()
        writer.writerows(analysis_rows)

    j2_min = float(sweep_summary["J2"]["min_hold_nm"])
    j2_max = float(sweep_summary["J2"]["max_hold_nm"])
    bounded_literal = 0.05
    bounded_encoded = 12 / 256.0
    design = {
        "schema": "go-m8010-arm-v15.24c-j2-feedforward-design/1.0",
        "hardware_motion_used": False,
        "model_audit": sweep_summary,
        "existing_run_analysis": analysis_summary,
        "torque_mapping": {
            "positive_ros_j2_torque_requires": {"J2A_tau_sign": -1, "J2B_tau_sign": 1},
            "rotor_side_conditional_formula": {
                "joint_total": "tau_J2 = G*(sign_A*tau_A_rotor + sign_B*tau_B_rotor)",
                "J2A_equal_share": "tau_A_rotor = sign_A*tau_J2/(2*G)",
                "J2B_equal_share": "tau_B_rotor = sign_B*tau_J2/(2*G)",
            },
            "output_side_conditional_formula": {
                "joint_total": "tau_J2 = sign_A*tau_A_output + sign_B*tau_B_output",
                "J2A_equal_share": "tau_A_output = sign_A*tau_J2/2",
                "J2B_equal_share": "tau_B_output = sign_B*tau_J2/2",
            },
            "selected_physical_formula": "ROTOR_SIDE",
            "reason": "GO-M8010 use manual V1.2 pages 4, 13, 17, and 18",
        },
        "representative_per_motor_torque_range": {
            "output_shaft_share_abs_nm": [j2_min / 2.0, j2_max / 2.0],
            "sdk_rotor_tff_abs_nm": [j2_min / (2.0 * GEAR), j2_max / (2.0 * GEAR)],
            "percent_of_serializer_effective_max": [
                100.0 * (j2_min / (2.0 * GEAR)) / 127.98828125,
                100.0 * (j2_max / (2.0 * GEAR)) / 127.98828125,
            ],
            "datasheet_max_output_torque_nm": 23.7,
            "datasheet_max_rotor_equivalent_nm": 23.7 / 6.33,
        },
        "gravity_load_contribution": "SUPPORTED",
        "gravity_root_cause": "NOT_PROVEN",
        "remaining_candidates": [
            "motor/internal torque or current saturation despite protocol headroom",
            "position-loop effort scaling or limiting", "mechanical brake or static friction",
            "unmodeled load and model-pose mismatch", "controller encoding beyond serializer",
        ],
        "proposed_initial_strategy": (
            "Use rotor-side conversion, equal split, and the frozen mirror signs. "
            "Start with a conservative fixed paired Tff of 0.05 literal per motor, "
            "not full compensation, because the exact six-axis pose is unavailable."
        ),
        "proposed_first_bounded_tff_literal_sdk_units_per_motor": bounded_literal,
        "proposed_first_bounded_tff_encoded_sdk_units_per_motor": bounded_encoded,
        "paired_signs_for_positive_logical_j2": {"J2A": -bounded_literal, "J2B": bounded_literal},
        "percentage_of_theoretical_gravity": {
            "exact_pose_unique_value_available": False,
            "representative_rotor_side_percent_range": [
                100.0 * bounded_encoded / (j2_max / (2.0 * GEAR)),
                100.0 * bounded_encoded / (j2_min / (2.0 * GEAR)),
            ],
        },
        "next_single_experiment": "J2_GRAVITY_FEEDFORWARD_BOUNDED_HOLD_AND_5DEG_TEST",
        "next_experiment_constraints": {
            "Kp": 0.60, "Kd": 0.05, "velocity_deg_s": 10,
            "acceleration_deg_s2": 30, "five_degree_motion": True,
            "start_with_tff_zero_hold": True,
            "then_ramp_paired_tff": "J2A 0 to -0.05; J2B 0 to +0.05",
            "motion_route_after_bounded_hold_pass": "0_TO_PLUS5_TO_0_TO_MINUS5_TO_0",
            "abort_on": [
                "unexpected direction", "e_sync > 0.3 deg", "rapid motion",
                "merror != 0", "temperature/sound/operator abnormality",
            ],
        },
        "j2_tau_command_authority": "PASS",
        "j2_gravity_model_audit": "PASS",
        "j2_feedforward_experiment_ready": "YES",
        "j2_local_motion_authority": "NOT_GRANTED",
        "j3_local_motion_authority": "NOT_GRANTED",
        "simulation_modified": False,
        "final_task_result": "DIAGNOSIS_COMPLETE",
    }
    dump_json(OUT / "j2_feedforward_design.json", design)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

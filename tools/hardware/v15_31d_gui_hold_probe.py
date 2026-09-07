#!/usr/bin/env python3
"""Observe 10 seconds of current-pose HOLD through the existing GUI controller.

Default is offline: print the contract without importing ROS or Qt. --execute
uses the operator's already-confirmed reliable support, then always requests
the existing GUI BRAKE/DISABLE and verifies fresh terminal feedback. Pass GUI
parameters after --ros-args, as for arm_gui (config_path, joint_limits_path,
initial_pose_path, embedded_model_path, embedded_session_pose_deg,
thermal_config_path and log_directory). No zero, target angle, gains, or gravity
scale is written. The bound empirical confirmation heartbeat stays at level 0.
Readiness/engagement, HOLD, and terminal limits are 20/10/3 s.
"""

from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import time


MOTORS = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
READINESS_S, HOLD_S, TERMINAL_S = 20.0, 10.0, 3.0
DEMO_HOLD_ERROR_DEG = 0.25
DEMO_HOLD_VELOCITY_DEG_S = 0.25


class HoldProbe:
    """Bounded observer; all motor commands remain owned by MainWindow."""

    def __init__(self, window, observe, *, now=time.monotonic):
        self.window, self.observe, self.now = window, observe, now
        self.started = now()
        self.stage = "readiness"
        self.deadline = self.started + READINESS_S
        self.requested = False
        self.target = None
        self.identity = None
        self.rejected_baseline = None
        self.hold_since = None
        self.first_hold_source_ns = None
        self.last_hold_source_ns = None
        self.hold_window_pass = False
        self.terminal_confirmed = False
        self.terminal_sequences = set()
        self.failure = None
        self.done = False
        self.stages = [{"stage": self.stage, "at_monotonic_s": self.started}]
        self.samples = []

    def stop(self, reason):
        if self.stage == "terminal" or self.done:
            return
        self.failure = reason
        self._terminal()

    def _terminal(self):
        self.stage = "terminal"
        self.deadline = self.now() + TERMINAL_S
        self.stages.append({"stage": self.stage, "at_monotonic_s": self.now()})
        try:
            self.window._emergency_brake(support_confirmed=True)
        except Exception as error:
            self.failure = f"{self.failure or ''}; GUI brake request failed: {error}"

    def tick(self):
        if self.done:
            return
        now = self.now()
        try:
            sample = self.observe()
            sample["stage"] = self.stage
            self.samples.append(sample)
            if self.stage == "terminal":
                if (sample["feedback_fresh"]
                        and all(sample["modes"].get(name) == "brake" for name in MOTORS)
                        and type(sample["j6_drive_state"]) is int and sample["j6_drive_state"] == 0
                        and type(sample["j6_raw_sequence"]) is int
                        and sample["j6_raw_sequence"] > 0
                        and (self.identity is None or sample["identity"][:2] == self.identity[:2])):
                    if self.terminal_sequences and sample["j6_raw_sequence"] < max(self.terminal_sequences):
                        self.terminal_sequences.clear()
                    self.terminal_sequences.add(sample["j6_raw_sequence"])
                    if len(self.terminal_sequences) >= 3:
                        self.terminal_confirmed = self.done = True
                else:
                    self.terminal_sequences.clear()
                if now >= self.deadline:
                    self.done = True
                    if not self.terminal_confirmed:
                        self.failure = f"{self.failure or ''}; terminal BRAKE/J6 DISABLED unconfirmed"
                return
            if self.stage == "readiness" and now >= self.deadline:
                self.stop("20-second readiness/HOLD engagement timeout")
                return

            if self.requested:
                if (not sample["healthy"] or not sample["zero_ff_authority"]
                        or sample["identity"] != self.identity):
                    self.stop("feedback/fault/zero-FF authority/session changed")
                    return
                if sample["router_rejected_commands"] != self.rejected_baseline:
                    self.stop("command router rejected a command during the probe")
                    return
                if tuple(self.window.command_targets) != self.target:
                    self.stop("frozen HOLD target changed")
                    return
                sample["target_rad"] = list(self.target)
                sample["error_deg"] = [math.degrees(actual - target)
                                       for actual, target in zip(sample["actual_rad"], self.target)]
                confirmed = (all(sample["modes"].get(name) == "hold" for name in MOTORS)
                             and type(sample["j6_drive_state"]) is int and sample["j6_drive_state"] == 1
                             and sample["router_hold_fresh"])
                in_window = (confirmed and sample["stationary_hold_ready"]
                             and max(map(abs, sample["error_deg"])) <= DEMO_HOLD_ERROR_DEG)
                if in_window and self.hold_since is None:
                    self.hold_since = now
                    self.first_hold_source_ns = sample["source_monotonic_ns"]
                    self.stage = "hold"
                    self.deadline = min(now + HOLD_S + 0.25, self.started + READINESS_S + HOLD_S)
                    self.stages.append({"stage": self.stage, "at_monotonic_s": now})
                elif self.hold_since is not None and not confirmed:
                    self.stop("seven-motor confirmed HOLD window interrupted")
                    return
                elif self.hold_since is not None and not sample["stationary_hold_ready"]:
                    self.stop("HOLD lost the 0.25-degree / 0.25-degree-per-second window")
                    return
                source_ns = sample["source_monotonic_ns"]
                if self.hold_since is not None:
                    sample["stage"] = "hold"
                    if max(map(abs, sample["error_deg"])) > DEMO_HOLD_ERROR_DEG:
                        self.stop("demo HOLD target error exceeded 0.25 degrees")
                        return
                    if (self.last_hold_source_ns is not None
                            and source_ns - self.last_hold_source_ns > 250_000_000):
                        self.stop("HOLD feedback sample gap exceeded 250 ms")
                        return
                    self.last_hold_source_ns = source_ns
                    if source_ns - self.first_hold_source_ns >= int(HOLD_S * 1e9):
                        self.hold_window_pass = True
                        self._terminal()
                        return
                    if now >= self.deadline:
                        self.stop("HOLD source feedback did not cover 10 seconds")
                        return
            elif sample["healthy"] and sample["zero_ff_authority"] and sample["router_ready"]:
                # The diagnostic callback pump may have newer data than the UI.
                # Run the original GUI path once so HOLD captures this frame.
                self.window._tick()
                sample = self.observe()
                if not (sample["healthy"] and sample["zero_ff_authority"] and sample["router_ready"]):
                    return
                self.identity = sample["identity"]
                self.rejected_baseline = sample["router_rejected_commands"]
                self.window._hold_current()
                if self.window.hardware_mode != "hold" or self.window.command_stream_suspended:
                    self.stop("GUI rejected current-pose HOLD")
                    return
                self.target = tuple(self.window.command_targets)
                self.requested = True
                self.stages.append({"stage": "hold_requested", "at_monotonic_s": now,
                                    "target_rad": list(self.target)})
        except Exception as error:
            if self.stage == "terminal":
                self.failure = f"terminal observation failed: {error}"
                self.done = now >= self.deadline
            else:
                self.stop(f"probe observation failed: {error}")

    def result(self):
        held = [sample for sample in self.samples if sample["stage"] == "hold" and "error_deg" in sample]
        temperatures = [sample.get("temperature_c", {}) for sample in self.samples]
        sync = [abs(sample["j2_sync_error_rad"]) for sample in self.samples
                if type(sample.get("j2_sync_error_rad")) in (int, float)]
        coverage_s = ((self.last_hold_source_ns - self.first_hold_source_ns) / 1e9
                      if self.last_hold_source_ns is not None else 0.0)
        times = sorted({sample["source_monotonic_ns"] for sample in held})
        names = [f"J{i + 1}" for i in range(6)]
        return {
            "schema": "go-m8010-current-pose-hold-probe/1.0",
            "status": "PASS" if self.hold_window_pass and self.terminal_confirmed and not self.failure else "FAIL",
            "scope": "CURRENT_POSE_10_SECOND_HOLD_ONLY_NOT_PRECISION_OR_LONG_TERM_LOAD_QUALIFICATION",
            "hold_window_pass": self.hold_window_pass,
            "terminal_brake_and_j6_disabled_confirmed": self.terminal_confirmed,
            "failure": self.failure,
            "target_rad": list(self.target) if self.target is not None else None,
            "demo_hold_error_limit_deg": DEMO_HOLD_ERROR_DEG,
            "demo_hold_velocity_limit_deg_s": DEMO_HOLD_VELOCITY_DEG_S,
            "strict_0_1_degree_qualification": "NOT_RUN_OR_MODIFIED",
            "hold_source_coverage_s": coverage_s,
            "hold_distinct_sample_count": len({sample["source_monotonic_ns"] for sample in held}),
            "maximum_hold_source_gap_ms": max(((right - left) / 1e6 for left, right in zip(times, times[1:])), default=None),
            "max_abs_error_deg_by_joint": {name: max(abs(sample["error_deg"][i]) for sample in held) for i, name in enumerate(names)} if held else None,
            "peak_to_peak_drift_deg_by_joint": {name: math.degrees(max(sample["actual_rad"][i] for sample in held)
                                                                  - min(sample["actual_rad"][i] for sample in held)) for i, name in enumerate(names)} if held else None,
            "start_to_end_drift_deg_by_joint": {name: math.degrees(held[-1]["actual_rad"][i] - held[0]["actual_rad"][i]) for i, name in enumerate(names)} if held else None,
            "maximum_velocity_deg_s_by_joint": {name: max(abs(math.degrees(sample["velocity_rad_s"][i])) for sample in held) for i, name in enumerate(names)} if held else None,
            "maximum_temperature_c": {name: max((values[name] for values in temperatures if type(values.get(name)) in (int, float)), default=None) for name in MOTORS},
            "maximum_j2_sync_error_deg": math.degrees(max(sync)) if sync else None,
            "elapsed_s": self.now() - self.started,
            "stages": self.stages, "samples": self.samples,
        }


def empirical_binding_matches(status, binding):
    empirical = status.get("empirical_validation", {})
    return bool(
        status.get("session_id") == binding.session_id
        and status.get("state_instance_id") == binding.state_instance_id
        and empirical.get("envelope_id") == binding.envelope_id
        and empirical.get("envelope_sha256") == binding.envelope_sha256
        and empirical.get("anchor_sha256") == binding.anchor_sha256
    )


def observe_gui(window, gui, raw_j6, binding):
    now, now_ns = time.monotonic(), time.monotonic_ns()
    node = window.node
    hardware = node.latest_hardware or {}
    gravity = node.latest_gravity_status or {}
    valid = gui.hardware_state_contract_valid(hardware)
    fresh = bool(valid and node.control_streams_fresh(now)
                 and gui.hardware_state_source_is_fresh(hardware, now_ns)
                 and all(item["fresh"] and item["communication_ok"]
                         for item in hardware["per_motor"].values()))
    healthy = fresh and all(gui.classify_logical_joint_observations(hardware)[0])
    empirical = gravity.get("empirical_validation", {})
    identity = (hardware.get("session_id"), hardware.get("state_instance_id"),
                gravity.get("source_instance_id"), empirical.get("anchor_sha256"),
                empirical.get("envelope_sha256"))
    zero_authority = bool(
        node.gravity_status_fresh(now)
        and empirical_binding_matches(gravity, binding)
        and gui.gravity_status_authorizes_hardware(
            gravity, session_id=hardware.get("session_id"),
            state_instance_id=hardware.get("state_instance_id"), now_monotonic_ns=now_ns,
        )
        and gravity.get("gravity_scale") == 0.0
        and gravity.get("gravity_scale_target") == 0.0
        and gravity.get("feedforward_nm") == [0.0] * 6
        and gravity.get("hardware_tff_enabled") is False
    )
    paired = hardware.get("j6_raw_feedback_identity")
    drive_state = None
    raw_sequence = None
    for raw in reversed(raw_j6):
        if (isinstance(paired, dict)
                and all(raw.get(key) == paired.get(key) for key in (
                    "source_instance_id", "sequence", "source_monotonic_ns", "session_id", "state_instance_id"))
                and raw.get("session_id") == hardware.get("session_id")
                and raw.get("state_instance_id") == hardware.get("state_instance_id")
                and type(raw.get("source_monotonic_ns")) is int
                and 0 <= now_ns - raw["source_monotonic_ns"] <= 250_000_000
                and raw["samples"][0].get("communication_ok") is True
                and raw["samples"][0].get("merror") == 0):
            drive_state = raw.get("drive_state")
            raw_sequence = raw.get("sequence")
            break
    router = node.latest_control_status or {}
    router_age = router.get("last_command_age_ms")
    router_text, severity = gui.command_router_status_text(
        router, node.control_status_fresh(now),
        float(window.config["控制"]["命令租约_秒"]) * 1000.0, window.hardware_mode,
    )
    return {
        "at_monotonic_s": now, "source_monotonic_ns": hardware.get("source_monotonic_ns"),
        "identity": identity, "feedback_fresh": fresh, "healthy": bool(healthy),
        "zero_ff_authority": zero_authority,
        "stationary_hold_ready": gui.collision_motion_state_ready(
            hardware, window.command_targets, window.requested_active_joint_mask,
            window.hardware_mode, now_ns=now_ns,
        ),
        "modes": hardware.get("controller_mode_by_motor", {}),
        "j6_drive_state": drive_state,
        "j6_raw_sequence": raw_sequence,
        "router_ready": bool(node.control_status_fresh(now) and type(router.get("rejected_commands")) is int and router["rejected_commands"] >= 0),
        "router_rejected_commands": router.get("rejected_commands"),
        "router_status": router_text,
        "router_hold_fresh": bool(node.control_status_fresh(now) and severity != "critical"
                                  and router.get("last_mode") == "hold"
                                  and router.get("last_activation_epoch") == window.activation_epoch
                                  and router.get("last_active_joint_mask") == [True] * 6
                                  and router.get("last_moving_joint_mask") == [False] * 6
                                  and type(router_age) in (int, float) and 0 <= router_age < 250),
        "actual_rad": hardware.get("position_rad"),
        "velocity_rad_s": hardware.get("velocity_rad_s"),
        "motor_tracking": {name: {key: item.get(key) for key in (
            "tau_cmd_rotor_nm", "tau_feedback_rotor_nm", "gravity_feedforward_rotor_nm", "dq_joint_rad_s",
            "trajectory_state", "trajectory_sample_index", "trajectory_interval_count")}
            for name, item in hardware.get("per_motor", {}).items()},
        "temperature_c": {name: item.get("temperature_c") for name, item in hardware.get("per_motor", {}).items()},
        "j2_sync_error_rad": hardware.get("j2_e_sync_rad"),
        "controller_faults": hardware.get("controller_fault_by_motor"),
        "source_age_ms": (now_ns - hardware["source_monotonic_ns"]) / 1e6 if valid else None,
        "gravity_blocker": gravity.get("empirical_validation", {}).get("blocker"),
    }


def run_live(ros_args, binding):
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
                          / "ros2_ws" / "src" / "go_m8010_arm_gui"))
    from go_m8010_arm_gui import main_window as gui
    from v15_31b_acceptance_signal import CONFIRMATION_TOPIC, SignalPayloadSequence

    gui.rclpy.init(args=ros_args)
    app = gui.QApplication([sys.argv[0]])
    app.setQuitOnLastWindowClosed(False)
    node = gui.ArmGuiNode()

    class ProbeWindow(gui.MainWindow):
        probe = None

        def closeEvent(self, event):
            if self.probe is not None and not self.probe.done:
                self.probe.stop("operator closed probe window")
                event.ignore()
            else:
                super().closeEvent(event)

    window = ProbeWindow(node)
    raw_j6 = deque(maxlen=64)

    def receive_raw(message):
        try:
            value = json.loads(message.data)
            samples = value.get("samples")
            if (value.get("schema") == "go-m8010-motor-feedback/1.0"
                    and isinstance(samples, list) and len(samples) == 1
                    and isinstance(samples[0], dict) and samples[0].get("motor") == "J6"):
                raw_j6.append(value)
        except (ValueError, AttributeError):
            pass

    subscription = node.create_subscription(gui.String, "/whole_arm/motor_feedback_raw", receive_raw, 100)
    probe = window.probe = HoldProbe(window, lambda: observe_gui(window, gui, raw_j6, binding))
    sequence = SignalPayloadSequence(binding)
    confirmation = node.create_publisher(gui.String, CONFIRMATION_TOPIC, 10)
    next_confirmation_s = 0.0
    confirmation_count = 0
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: probe.stop("operator/process stop requested"))
    timer = gui.QTimer(window)

    def tick():
        nonlocal next_confirmation_s, confirmation_count
        try:
            if gui.rclpy.ok(context=node.context):
                # Four domains add about 400 Hz of raw diagnostic feedback.
                # Drain at most 32 nonblocking callbacks; keep the GUI budget unchanged.
                for _ in range(4):
                    gui.pump_ros_callbacks(node, gui.rclpy.spin_once)
            if not probe.done and probe.stage != "terminal":
                binding.ensure_not_expired()
                hardware = node.latest_hardware
                if (gui.hardware_state_contract_valid(hardware)
                        and gui.hardware_state_source_is_fresh(hardware, time.monotonic_ns())):
                    if (hardware["session_id"], hardware["state_instance_id"]) != (binding.session_id, binding.state_instance_id):
                        raise RuntimeError("empirical envelope does not match current hardware session")
                    gravity_fresh = node.gravity_status_fresh()
                    if gravity_fresh and not empirical_binding_matches(node.latest_gravity_status or {}, binding):
                        raise RuntimeError("live gravity envelope/anchor does not match the supplied evidence binding")
                    if gravity_fresh and time.monotonic() >= next_confirmation_s:
                        confirmation.publish(gui.String(data=json.dumps(sequence.confirmation(0.0))))
                        confirmation_count += 1
                        next_confirmation_s = time.monotonic() + 1.0
        except Exception as error:
            probe.stop(f"confirmation/callback failure: {error}")
        probe.tick()
        if probe.done:
            timer.stop()
            app.quit()

    timer.timeout.connect(tick)
    timer.start(20)
    window.centralWidget().setEnabled(False)
    stop_action = window.addToolBar("验证停止").addAction("立即停止并制动（Esc）")
    stop_action.setToolTip("机械臂已独立可靠支撑；立即停止验证并制动。关闭窗口也执行停止。")
    stop_action.setShortcut("Esc")
    stop_action.triggered.connect(lambda *_: probe.stop("operator pressed immediate stop"))
    window.setWindowTitle("当前姿态 HOLD 10 秒验证：结束自动制动，保持可靠支撑")
    window.show()
    try:
        app.exec()
    except BaseException as error:
        probe.stop(f"GUI event loop exited: {error}")
    finally:
        if not probe.done:
            probe.stop("GUI event loop stopped before verification completed")
            while not probe.done:
                try:
                    window._tick()
                except Exception:
                    pass
                probe.tick()
                time.sleep(0.01)
    result = probe.result()
    result["gui_config_path"] = str(node.config_path)
    result["gui_config_sha256"] = hashlib.sha256(node.config_path.read_bytes()).hexdigest()
    result["gui_control_config"] = window.config["控制"]
    result["empirical_confirmation"] = {
        "topic": CONFIRMATION_TOPIC, "target_gravity_scale": 0.0,
        "sent_count": confirmation_count, "envelope_sha256": binding.envelope_sha256,
        "anchor_sha256": binding.anchor_sha256, "session_id": binding.session_id,
        "state_instance_id": binding.state_instance_id,
    }
    window.timer.stop()
    window.close()
    node.destroy_subscription(subscription)
    node.destroy_node()
    if gui.rclpy.ok():
        gui.rclpy.shutdown()
    return result


def main(argv=None):
    values = list(sys.argv[1:] if argv is None else argv)
    split = values.index("--ros-args") if "--ros-args" in values else len(values)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--envelope", type=Path)
    parser.add_argument("--anchor-validation", type=Path)
    parser.add_argument("--expected-envelope-sha256")
    args = parser.parse_args(values[:split])
    if not args.execute:
        print(json.dumps({"mode": "OFFLINE_DESCRIPTION_ONLY", "hardware_accessed": False,
                          "readiness_s": READINESS_S, "hold_s": HOLD_S, "terminal_s": TERMINAL_S,
                          "execution": "--execute --output <new-result.json> --envelope <current.json> --anchor-validation <current-validation.json> --expected-envelope-sha256 <sha256> --ros-args <existing GUI parameters>"}))
        return 0
    if args.output is None or args.output.exists():
        parser.error("--execute requires --output pointing to a new report file")
    if split == len(values):
        parser.error("--execute requires existing GUI configuration via --ros-args")
    if not all((args.envelope, args.anchor_validation, args.expected_envelope_sha256)):
        parser.error("--execute requires current envelope, anchor validation and expected envelope SHA256")
    from v15_31b_active_acceptance_runner import EvidenceBinding
    binding = EvidenceBinding.from_paths(args.envelope, args.anchor_validation, args.expected_envelope_sha256)
    binding.ensure_not_expired()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x+", encoding="utf-8") as stream:
        stream.write('{"status":"INCOMPLETE","reason":"probe is running"}\n')
        stream.flush()
        result = run_live(values[split:], binding)
        stream.seek(0)
        stream.write(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2) + "\n")
        stream.truncate()
    print(json.dumps({key: result[key] for key in (
        "status", "hold_window_pass", "terminal_brake_and_j6_disabled_confirmed", "failure")}, ensure_ascii=False))
    print(f"REPORT={args.output.resolve()}")
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

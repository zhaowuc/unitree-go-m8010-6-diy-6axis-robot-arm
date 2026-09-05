#!/usr/bin/env python3
"""Sequence empirical checks while the GUI remains the sole motion publisher.

No GUI command publisher and no motor transport. A failed check requests the
existing acceptance runner's all-domain stop. A fresh physical gate and bound
envelope are required before any ROS publisher is created.
"""
import argparse
import json
import os
import signal
import time
from pathlib import Path

from v15_31b_acceptance_signal import SignalPayloadSequence
from v15_31b_active_acceptance_runner import EvidenceBinding, validate_physical_startup_gate


def gui_hold_ready(hardware, control):
    modes = hardware.get("controller_mode_by_motor", {})
    return (
        set(modes) == {"J1", "J2A", "J2B", "J3", "J4", "J5", "J6"}
        and all(mode == "hold" for mode in modes.values())
        and control.get("last_mode") == "hold"
        and isinstance(control.get("last_command_age_ms"), (int, float))
        and 0 <= control["last_command_age_ms"] < 250
    )


def run(binding):
    import rclpy
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    from std_msgs.msg import String

    rclpy.init()
    node = rclpy.create_node("gui_gravity_session")
    seq = SignalPayloadSequence(binding)
    confirmation = node.create_publisher(String, "/whole_arm/empirical_stage_confirmation", 10)
    control = node.create_publisher(String, "/whole_arm/v15_31b/acceptance_control", 10)
    topics = {
        "hardware": "/whole_arm/hardware_state",
        "router": "/whole_arm/control_status",
        "gravity": "/whole_arm/gravity_status",
        "runner": "/whole_arm/v15_31b/acceptance_status",
    }
    latest, received = {}, {}

    def receive(key, message):
        latest[key] = json.loads(message.data)
        received[key] = time.monotonic()

    subscriptions = [
        node.create_subscription(String, topic, lambda msg, k=key: receive(k, msg), 10)
        for key, topic in topics.items()
    ]
    client = node.create_client(SetParameters, "/whole_arm_gravity_node/set_parameters")
    stopping = False
    target = 0.0
    next_confirmation = 0.0
    engaged = False

    def stop(*_):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    def send(action, **extras):
        control.publish(String(data=json.dumps(seq.control(action, extras))))

    def wait(predicate, timeout, label):
        nonlocal next_confirmation
        deadline = time.monotonic() + timeout
        while not stopping and time.monotonic() < deadline:
            binding.ensure_not_expired()
            now = time.monotonic()
            if now >= next_confirmation:
                confirmation.publish(String(data=json.dumps(seq.confirmation(target))))
                next_confirmation = now + 1.0
            rclpy.spin_once(node, timeout_sec=0.02)
            if latest.get("runner", {}).get("failure"):
                raise RuntimeError(str(latest["runner"]["failure"]))
            empirical = latest.get("gravity", {}).get("empirical_validation", {})
            if empirical.get("invalidated"):
                raise RuntimeError(str(empirical.get("blocker")))
            if len(received) == 4 and all(now - t < 0.5 for t in received.values()):
                if latest["hardware"].get("session_id") != binding.session_id:
                    raise RuntimeError("hardware session changed")
                if latest["hardware"].get("state_instance_id") != binding.state_instance_id:
                    raise RuntimeError("hardware state instance changed")
                if predicate():
                    return
        raise RuntimeError("stopped or timed out: " + label)

    def comparison(condition, target_j2_rad):
        send("START_J2_COMPARISON", condition=condition,
             target_j2_rad=target_j2_rad)
        start = time.monotonic()
        wait(lambda: time.monotonic() - start >= 1.2, 3, condition)
        send("STOP_J2_COMPARISON")
        wait(lambda: condition in latest["runner"].get("comparison", {}).get("complete_conditions", []),
             4, condition)

    try:
        print("GUI_OWNS_HOLD=YES; WAITING_FOR_GUI_HOLD", flush=True)
        wait(lambda: gui_hold_ready(latest["hardware"], latest["router"]),
             max(0.0, binding.expires_at_utc.timestamp() - time.time()), "GUI HOLD")
        engaged = True
        comparison_target_j2_rad = latest["hardware"]["position_rad"][1]
        comparison("WITHOUT_FF", comparison_target_j2_rad)
        send("START_GRAVITY_LADDER")
        for index, level in enumerate((0.0, 0.25, 0.5, 0.75, 1.0)):
            target = level
            next_confirmation = 0.0
            if index:
                start = time.monotonic()
                wait(lambda: time.monotonic() - start >= 0.2, 1, "confirmation delivery")
                if not client.wait_for_service(timeout_sec=2):
                    raise RuntimeError("gravity parameter service unavailable")
                request = SetParameters.Request(parameters=[Parameter(
                    name="gravity_scale_target",
                    value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=level))])
                future = client.call_async(request)
                wait(future.done, 3, "scale request")
                if not future.result().results or not all(r.successful for r in future.result().results):
                    raise RuntimeError("scale request rejected")
            wait(lambda: latest["gravity"]["empirical_validation"].get("stage_index") == index
                 and latest["gravity"]["empirical_validation"].get("stage_complete") is True,
                 12, "gravity rung")
            print(f"GRAVITY_LEVEL_{int(level * 100)}=PASS", flush=True)
        wait(lambda: latest["runner"]["gravity_ladder"]["complete"], 2, "runner ladder")
        comparison("WITH_FF", comparison_target_j2_rad)
        send("START_POSITION")
        wait(lambda: latest["runner"].get("position", {}).get("started") is True
             and latest["runner"]["position"].get("awaiting_gui_command") is True,
             4, "runner position ready")
        print("GUI_PREVIEW_READY=YES; J1_ONLY_FIRST", flush=True)
        while not stopping:
            wait(lambda: False, 3600, "operator session")
    finally:
        if engaged:
            send("STOP_AND_BRAKE")
            for _ in range(10):
                rclpy.spin_once(node, timeout_sec=0.02)
        node.destroy_node()
        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--envelope", type=Path, required=True)
    parser.add_argument("--anchor-validation", type=Path, required=True)
    parser.add_argument("--expected-envelope-sha256", required=True)
    args = parser.parse_args()
    validate_physical_startup_gate(os.environ)
    run(EvidenceBinding.from_paths(args.envelope, args.anchor_validation, args.expected_envelope_sha256))


if __name__ == "__main__":
    main()

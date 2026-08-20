#!/usr/bin/env python3
"""V15.21D operator-gated, hard-deadline POS_VEL enable/hold test."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from pathlib import Path

from dm_g6220_posvel_transport import DmG6220PosVelTransport
from j6_raw_can_diagnostic import (
    Blocked,
    add_evidence,
    capture_refresh_samples,
    exact_usb_identity,
    read_parameter,
    state_text,
    timestamp_token,
    write_json,
    write_position_csv,
    write_raw_csv,
)


CLEARANCE_GATE = "J6_POSVEL_MICRO_ENABLE_CLEARANCE_10DEG=YES"
STOP_GATE = "J6_POSVEL_EMERGENCY_STOP_READY=YES"
MAX_ENABLE_WINDOW_MS = 15.0
PLANNED_FIRST_FD_MS = 12.0
CTRL_MODE_POS_VEL = 2
FAULT_STATES = {8, 9, 0xA, 0xB, 0xC, 0xD, 0xE}


def busy_wait_until(target: float) -> None:
    while time.monotonic() < target:
        pass


def require_gates(args: argparse.Namespace) -> None:
    if args.operator_clearance != CLEARANCE_GATE:
        raise Blocked(f"require --operator-clearance '{CLEARANCE_GATE}'")
    if args.operator_stop != STOP_GATE:
        raise Blocked(f"require --operator-stop '{STOP_GATE}'")


def micro_enable(args: argparse.Namespace) -> int:
    require_gates(args)
    inventory = json.loads(args.inventory.read_text(encoding="utf-8"))
    first_attempt = inventory.get("status") == "POSVEL_BASELINE_PASS_AWAITING_MICRO_ENABLE_GATE"
    explicit_observation_repeat = (
        inventory.get("status") == "AWAITING_OPERATOR_PHYSICAL_OBSERVATION"
        and args.operator_repeat_authorized
        and not inventory.get("micro_enable_attempts")
    )
    if not (first_attempt or explicit_observation_repeat):
        raise Blocked(f"inventory status does not authorize active test: {inventory.get('status')}")
    if explicit_observation_repeat:
        previous = inventory.get("micro_enable")
        if previous:
            previous["status"] = "OPERATOR_DID_NOT_OBSERVE_CLEARLY"
            previous["operator_visible_jerk"] = "NOT_OBSERVED_CLEARLY"
            inventory.setdefault("micro_enable_attempts", []).append(previous)
    exact_usb_identity()
    master_id = int(inventory["motor"]["master_id_rid7"])
    transport = DmG6220PosVelTransport(master_id)
    first_fd_sent = False
    try:
        logger = transport._raw_logger_for_commissioning()
        time.sleep(0.1)
        mode = int(read_parameter(logger, 10))
        inventory["ctrl_mode_before_enable"] = mode
        if mode != CTRL_MODE_POS_VEL:
            raise Blocked(f"RID10 before enable is {mode}, expected 2")
        initial_count = 100 if explicit_observation_repeat else 5
        initial_values, _ = capture_refresh_samples(
            logger, initial_count, "PRE_ENABLE_DISABLED"
        )
        if any(item[1].state != 0 for item in initial_values):
            raise Blocked("motor was not DISABLED before preload")

        if explicit_observation_repeat:
            q_session = statistics.median(item[1].position for item in initial_values)
            inventory["j6_posvel_session_reference_rad"] = q_session
        else:
            q_session = float(inventory["j6_posvel_session_reference_rad"])
        preload_at, preload_payload = transport.send_pos_vel_command(
            q_session, 0.0, "POSVEL_PRELOAD_DISABLED"
        )
        fc_at = transport.send_enable()
        transport.send_pos_vel_command(q_session, 0.0, "POSVEL_HOLD_IMMEDIATE")
        busy_wait_until(fc_at + 0.010)
        transport.send_pos_vel_command(q_session, 0.0, "POSVEL_HOLD_10MS")
        busy_wait_until(fc_at + PLANNED_FIRST_FD_MS / 1000.0)
        first_fd_at = transport.send_disable("FD_DISABLE_HARD_DEADLINE")
        first_fd_sent = True
        enabled_window_ms = (first_fd_at - fc_at) * 1000.0
        time.sleep(0.020)
        transport.send_disable("FD_DISABLE_REPEAT_2")
        time.sleep(0.020)
        transport.send_disable("FD_DISABLE_REPEAT_3")
        remaining = fc_at + 0.315 - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)

        final_values, _ = capture_refresh_samples(logger, 5, "FINAL_DISABLED_VERIFY")
        final_disabled = len(final_values) == 5 and all(item[1].state == 0 for item in final_values)
        post_values, _ = capture_refresh_samples(logger, 50, "POST_DISABLE_POSITION")
        if any(item[1].state != 0 for item in post_values):
            final_disabled = False
        ctrl_mode_final = int(read_parameter(logger, 10))

        drained = transport.strict_feedback_drain()
        raw_events = list(drained.raw_events)
        normal = list(drained.normal_feedback)
        active = [
            sample for sample in normal
            if sample.event.event_monotonic_s >= fc_at and sample.decoded.state == 1
        ]
        first_active = active[0] if active else None
        signed_max_velocity = None
        if active:
            signed_max_velocity = max(
                (sample.decoded.velocity for sample in active), key=abs
            )
        fault_names = sorted({
            state_text(sample.decoded.state)
            for sample in normal
            if sample.decoded.state in FAULT_STATES
        })
        unexpected_rx = [
            event for event in raw_events
            if event.event_kind == "RX_CALLBACK"
            and event.event_monotonic_s >= fc_at
            and logger.classify(event) == "UNKNOWN"
        ]
        q_post = statistics.median(item[1].position for item in post_values)
        delta_deg = math.degrees(q_post - q_session)

        raw_path = args.output_dir / f"j6_posvel_micro_enable_raw_{timestamp_token()}.csv"
        post_path = args.output_dir / f"j6_posvel_post_disable_{timestamp_token()}.csv"
        write_raw_csv(raw_path, logger, raw_events)
        write_position_csv(post_path, post_values)
        inventory["micro_enable"] = {
            "status": "AWAITING_OPERATOR_PHYSICAL_OBSERVATION",
            "attempt_number": len(inventory.get("micro_enable_attempts", [])) + 1,
            "explicit_operator_repeat_after_missed_observation": explicit_observation_repeat,
            "operator_gate": "PASS",
            "pre_enable_disabled_valid_frames": len(initial_values),
            "preload": {
                "sent_while_disabled": True,
                "timestamp_monotonic_s": preload_at,
                "can_id": 0x101,
                "payload_hex": preload_payload.hex().upper(),
                "position_target_rad": q_session,
                "velocity_target_rad_s": 0.0,
            },
            "fc_sent": True,
            "actual_enabled_window_ms": enabled_window_ms,
            "maximum_allowed_window_ms": MAX_ENABLE_WINDOW_MS,
            "fd_sent": first_fd_sent,
            "final_five_frame_disabled": "PASS" if final_disabled else "FAIL",
            "first_active_feedback": None if first_active is None else {
                "can_id": first_active.event.can_id,
                "payload_hex": first_active.event.payload.hex().upper(),
                "classification": logger.classify(first_active.event),
                "state": state_text(first_active.decoded.state),
                "position_rad": first_active.decoded.position,
                "velocity_rad_s": first_active.decoded.velocity,
            },
            "active_normal_feedback_count": len(active),
            "max_active_velocity_rad_s": signed_max_velocity,
            "fault_states": fault_names,
            "unexpected_rx_count": len(unexpected_rx),
            "q_post_median_rad": q_post,
            "q_post_minus_q_session_rad": q_post - q_session,
            "q_post_minus_q_session_deg": delta_deg,
            "operator_visible_jerk": "PENDING",
            "raw_csv_path": str(raw_path),
            "post_csv_path": str(post_path),
        }
        inventory["ctrl_mode_final"] = ctrl_mode_final
        inventory["ctrl_mode_persistence"] = "NOT_TESTED"
        add_evidence(inventory, raw_path)
        add_evidence(inventory, post_path)
        hard_failure = any((
            enabled_window_ms > MAX_ENABLE_WINDOW_MS,
            not final_disabled,
            bool(fault_names),
            bool(unexpected_rx),
            ctrl_mode_final != CTRL_MODE_POS_VEL,
            abs(delta_deg) > 0.2,
        ))
        inventory["micro_enable"]["pre_operator_hard_acceptance"] = "FAIL" if hard_failure else "PASS"
        inventory["status"] = "AWAITING_OPERATOR_PHYSICAL_OBSERVATION"
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, indent=2))
        return 0
    finally:
        if not first_fd_sent:
            try:
                for index in range(3):
                    transport.send_disable(f"FD_DISABLE_FINALLY_{index + 1}")
                    if index < 2:
                        time.sleep(0.020)
            except Exception:
                pass
        transport.close()


def finalize(args: argparse.Namespace) -> int:
    inventory = json.loads(args.inventory.read_text(encoding="utf-8"))
    if inventory.get("status") != "AWAITING_OPERATOR_PHYSICAL_OBSERVATION":
        raise Blocked(f"inventory is not awaiting observation: {inventory.get('status')}")
    if args.visible_jerk not in ("YES", "NO"):
        raise Blocked("--visible-jerk must be YES or NO")
    micro = inventory["micro_enable"]
    micro["operator_visible_jerk"] = args.visible_jerk
    passed = micro["pre_operator_hard_acceptance"] == "PASS" and args.visible_jerk == "NO"
    micro["status"] = "PASS" if passed else "FAIL"
    inventory["j6_posvel_enable_hold"] = "PASS" if passed else "FAIL"
    inventory["final_task_result"] = "PASS" if passed else "FAIL"
    inventory["status"] = inventory["final_task_result"]
    inventory["completed_at_utc"] = __import__(
        "datetime"
    ).datetime.now(__import__("datetime").timezone.utc).isoformat()
    write_json(args.inventory, inventory)
    print(json.dumps(inventory, indent=2))
    return 0 if passed else 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("micro-enable", "finalize"))
    parser.add_argument("--operator-clearance", default="")
    parser.add_argument("--operator-stop", default="")
    parser.add_argument("--operator-repeat-authorized", action="store_true")
    parser.add_argument("--visible-jerk", choices=("YES", "NO"))
    parser.add_argument("--output-dir", type=Path, default=Path("hardware/v15_21d"))
    parser.add_argument(
        "--inventory",
        type=Path,
        default=Path("hardware/v15_21d/j6_posvel_enable_hold_inventory.json"),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        return micro_enable(args) if args.phase == "micro-enable" else finalize(args)
    except Blocked as exc:
        print(f"BLOCKED: {exc}")
        return 4


if __name__ == "__main__":
    raise SystemExit(main())

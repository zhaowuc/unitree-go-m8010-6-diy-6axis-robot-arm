#!/usr/bin/env python3
"""V15.21F read-only J6 power-cycle and READY reference capture."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from pathlib import Path

from j6_raw_can_diagnostic import (
    Blocked,
    RawCanLogger,
    add_evidence,
    capture_refresh_samples,
    exact_usb_identity,
    normal_feedback,
    read_parameter,
    refresh_request,
    state_text,
    timestamp_token,
    utc_now,
    write_json,
    write_position_csv,
    write_raw_csv,
)


SOURCE_HEAD = "b0c72afdb4b5bdf88aa8a87582beab0760719eca"
WORK_BRANCH = "agent/v15-21f-j6-startup-ready-reference"
MOTOR_ID = 1
EXPECTED_MASTER_ID = 0
EXPECTED_PMAX = 12.5
CTRL_MODE_POS_VEL = 2
SAMPLE_COUNT = 200

POWER_OFF_GATE = "J6_24V_POWER_OFF_CONFIRMED=YES"
POWER_ON_GATE = "J6_24V_POWER_ON_CONFIRMED=YES"
POSE_MOVED_YES = "PHYSICAL_POSE_MOVED_WHILE_OFF=YES"
POSE_MOVED_NO = "PHYSICAL_POSE_MOVED_WHILE_OFF=NO"
READY_GATE = "J6_READY_PHYSICAL_POSE_SET=YES"


def base_inventory() -> dict:
    return {
        "schema_version": "1.0",
        "task": "V15.21F J6 startup and READY reference authority",
        "source_head": SOURCE_HEAD,
        "work_branch": WORK_BRANCH,
        "created_at_utc": utc_now(),
        "status": "NEW",
        "frozen_authority": {
            "j6_posvel_local_motion": "PASS",
            "j6_bidirectional_5deg": "PASS",
            "j6_local_motion_authority": "PASS",
            "j6_protocol_to_ros_sign": -1,
            "mit_path": "FAILED_NOT_PREFERRED",
        },
        "safety": {
            "active_motor_motion_used": False,
            "fc_enable_used": False,
            "set_zero_used": False,
            "id_write_used": False,
            "parameter_write_used": False,
            "eeprom_flash_write_used": False,
            "original_ready_pose_v1_modified": False,
            "j1_j345_simulation_modified": False,
        },
        "authority": {
            "j6_startup_state_authority": "NOT_ESTABLISHED",
            "j6_ready_reference": "PENDING_OPERATOR_POSE",
            "j6_ready_recovery_motion": "NOT_TESTED",
            "j6_cad_zero": "PENDING",
            "j6_ros_zero": "PENDING",
            "j6_physical_limits": "PENDING",
            "j6_full_commissioning": "NOT_COMPLETE",
        },
        "evidence": [],
    }


def load_inventory(path: Path) -> dict:
    if not path.is_file():
        raise Blocked(f"inventory does not exist: {path}")
    return json.loads(path.read_text(encoding="utf-8"))


def readonly_motor(logger: RawCanLogger) -> dict:
    master_id = int(read_parameter(logger, 7))
    logger.master_id = master_id
    return {
        "master_id_rid7": master_id,
        "esc_id_rid8": int(read_parameter(logger, 8)),
        "ctrl_mode_rid10": int(read_parameter(logger, 10)),
        "pmax_rid21": read_parameter(logger, 21),
        "allowed_normal_feedback_can_ids": sorted(logger.allowed_feedback_ids()),
    }


def validate_identity(motor: dict) -> None:
    if motor["master_id_rid7"] != EXPECTED_MASTER_ID:
        raise Blocked(f"RID7 Master ID changed: {motor['master_id_rid7']}")
    if motor["esc_id_rid8"] != MOTOR_ID:
        raise Blocked(f"RID8 ESC ID changed: {motor['esc_id_rid8']}")
    if not math.isclose(motor["pmax_rid21"], EXPECTED_PMAX, rel_tol=0.0, abs_tol=1e-5):
        raise Blocked(f"RID21 PMAX changed: {motor['pmax_rid21']}")


def feedback_stats(values, semantics: str) -> dict:
    positions = [item[1].position for item in values]
    velocities = [item[1].velocity for item in values]
    mos = [item[1].mos_temp for item in values]
    coil = [item[1].coil_temp for item in values]
    states = sorted({item[1].state for item in values})
    return {
        "valid_frames": len(values),
        "total_required": SAMPLE_COUNT,
        "state_values": states,
        "state_names": [state_text(value) for value in states],
        "q_median_rad": statistics.median(positions),
        "q_mean_rad": statistics.fmean(positions),
        "q_std_rad": statistics.pstdev(positions),
        "q_min_rad": min(positions),
        "q_max_rad": max(positions),
        "velocity_min_rad_s": min(velocities),
        "velocity_max_rad_s": max(velocities),
        "mos_temperature_min_c": min(mos),
        "mos_temperature_max_c": max(mos),
        "coil_temperature_min_c": min(coil),
        "coil_temperature_max_c": max(coil),
        "semantics": semantics,
    }


def capture_disabled(logger: RawCanLogger, label: str, semantics: str):
    initial, _ = capture_refresh_samples(logger, 5, f"{label}_STATE_CHECK")
    if len(initial) != 5 or any(item[1].state != 0 for item in initial):
        states = sorted({state_text(item[1].state) for item in initial})
        raise Blocked(f"motor is not reliably DISABLED: {states}")
    values, start_index = capture_refresh_samples(logger, SAMPLE_COUNT, label)
    if len(values) != SAMPLE_COUNT or any(item[1].state != 0 for item in values):
        raise Blocked(f"{label} acceptance failed")
    return values, start_index, feedback_stats(values, semantics)


def save_capture(args, inventory: dict, logger: RawCanLogger, values, stem: str) -> dict:
    token = timestamp_token()
    decoded_path = args.output_dir / f"{stem}_{token}.csv"
    raw_path = args.output_dir / f"{stem}_raw_{token}.csv"
    write_position_csv(decoded_path, values)
    write_raw_csv(raw_path, logger, logger.snapshot())
    add_evidence(inventory, decoded_path)
    add_evidence(inventory, raw_path)
    return {"decoded_csv_path": str(decoded_path), "raw_csv_path": str(raw_path)}


def pre_power(args: argparse.Namespace) -> int:
    inventory = base_inventory()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    logger = None
    try:
        inventory["usb_pre_power"] = exact_usb_identity()
        logger = RawCanLogger()
        time.sleep(0.1)
        motor = readonly_motor(logger)
        validate_identity(motor)
        inventory["pre_power_motor"] = motor
        values, _, stats = capture_disabled(
            logger, "PRE_POWER_CYCLE_BASELINE", "POWER_CYCLE_LOCAL_COMPARISON_ONLY"
        )
        stats.update(save_capture(args, inventory, logger, values, "j6_pre_powercycle_baseline"))
        stats["status"] = "PASS"
        inventory["pre_power_baseline"] = stats
        inventory["status"] = "PRE_POWER_BASELINE_PASS_AWAITING_POWER_OFF_GATE"
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, indent=2))
        return 0
    except Blocked as exc:
        inventory["status"] = "BLOCKED"
        inventory["block_reason"] = str(exc)
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, indent=2))
        return 4
    finally:
        if logger is not None:
            logger.close()


def confirm_power_off(args: argparse.Namespace) -> int:
    if args.operator_gate != POWER_OFF_GATE:
        raise Blocked(f"require --operator-gate '{POWER_OFF_GATE}'")
    inventory = load_inventory(args.inventory)
    if inventory.get("status") != "PRE_POWER_BASELINE_PASS_AWAITING_POWER_OFF_GATE":
        raise Blocked(f"inventory status does not authorize power-off probe: {inventory.get('status')}")
    inventory["power_off_operator_confirmation"] = "YES"
    logger = None
    try:
        inventory["usb_while_motor_off"] = exact_usb_identity()
        logger = RawCanLogger()
        time.sleep(0.1)
        start_index = len(logger.snapshot())
        start = time.monotonic()
        # Read-only refresh probes over at least three seconds. No control frame is sent.
        for index in range(31):
            target = start + index * 0.1
            remaining = target - time.monotonic()
            if remaining > 0:
                time.sleep(remaining)
            logger.send(0x7FF, refresh_request(), f"MOTOR_POWER_OFF_REFRESH_{index:02d}")
        time.sleep(0.3)
        received = normal_feedback(logger, logger.snapshot()[start_index:])
        raw_path = args.output_dir / f"j6_power_off_feedback_probe_raw_{timestamp_token()}.csv"
        write_raw_csv(raw_path, logger, logger.snapshot())
        add_evidence(inventory, raw_path)
        inventory["power_off_probe"] = {
            "probe_duration_s": time.monotonic() - start,
            "refresh_requests": 31,
            "normal_feedback_frames": len(received),
            "feedback_disappeared_while_motor_off": len(received) == 0,
            "raw_csv_path": str(raw_path),
        }
        if received:
            raise Blocked("normal feedback remained present after operator-confirmed motor power off")
        inventory["status"] = "POWER_OFF_CONFIRMED_AWAITING_POWER_ON_GATE"
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, indent=2))
        return 0
    except Blocked as exc:
        inventory["status"] = "BLOCKED"
        inventory["block_reason"] = str(exc)
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, indent=2))
        return 4
    finally:
        if logger is not None:
            logger.close()


def post_power(args: argparse.Namespace) -> int:
    if args.operator_gate != POWER_ON_GATE:
        raise Blocked(f"require --operator-gate '{POWER_ON_GATE}'")
    if args.pose_moved not in (POSE_MOVED_YES, POSE_MOVED_NO):
        raise Blocked(f"require --pose-moved '{POSE_MOVED_YES}' or '{POSE_MOVED_NO}'")
    inventory = load_inventory(args.inventory)
    if inventory.get("status") != "POWER_OFF_CONFIRMED_AWAITING_POWER_ON_GATE":
        raise Blocked(f"inventory status does not authorize post-power capture: {inventory.get('status')}")
    inventory["power_on_operator_confirmation"] = "YES"
    physical_moved = args.pose_moved == POSE_MOVED_YES
    inventory["physical_pose_moved_while_off"] = physical_moved
    logger = None
    try:
        time.sleep(1.0)
        inventory["usb_post_power"] = exact_usb_identity()
        logger = RawCanLogger()
        time.sleep(0.1)
        motor = readonly_motor(logger)
        validate_identity(motor)
        inventory["post_power_motor"] = motor
        values, _, stats = capture_disabled(
            logger, "POST_POWER_CYCLE_BASELINE", "POWER_CYCLE_LOCAL_COMPARISON_ONLY"
        )
        stats.update(save_capture(args, inventory, logger, values, "j6_post_powercycle_baseline"))
        stats["status"] = "PASS"
        inventory["post_power_baseline"] = stats

        before_mode = int(inventory["pre_power_motor"]["ctrl_mode_rid10"])
        after_mode = int(motor["ctrl_mode_rid10"])
        if before_mode == CTRL_MODE_POS_VEL and after_mode == CTRL_MODE_POS_VEL:
            persistence = "PERSISTS_IN_THIS_TEST"
        elif before_mode == CTRL_MODE_POS_VEL and after_mode == 1:
            persistence = "DOES_NOT_PERSIST"
        else:
            persistence = "INCONCLUSIVE"
        inventory["ctrl_mode_power_cycle_persistence"] = persistence
        if persistence == "INCONCLUSIVE":
            raise Blocked(f"CTRL_MODE behavior is inconclusive: before={before_mode}, after={after_mode}")

        q_before = float(inventory["pre_power_baseline"]["q_median_rad"])
        q_after = float(stats["q_median_rad"])
        delta_rad = q_after - q_before
        delta_deg = math.degrees(delta_rad)
        if physical_moved:
            position_classification = "POSE_MOVED_OPERATOR_REPORTED_NO_REPEATABILITY_CLAIM"
        elif abs(delta_deg) <= 1.0:
            position_classification = "LOCAL_REPEATABILITY_OBSERVED_IN_THIS_TEST"
        else:
            position_classification = "NOT_REPEATABLE_IN_THIS_TEST"
        inventory["power_cycle_position_behavior"] = {
            "q_after_minus_q_before_rad": delta_rad,
            "q_after_minus_q_before_deg": delta_deg,
            "physical_pose_moved_while_off": physical_moved,
            "classification": position_classification,
            "claim_limit": "OBSERVED_POWER_CYCLE_POSITION_BEHAVIOR_ONLY",
        }
        inventory["runtime_mode_write_performed"] = False
        inventory["authority"]["j6_startup_state_authority"] = "PASS"
        inventory["status"] = "STARTUP_STATE_PASS_AWAITING_READY_OPERATOR_POSE"
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, indent=2))
        return 0
    except Blocked as exc:
        inventory["status"] = "BLOCKED"
        inventory["block_reason"] = str(exc)
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, indent=2))
        return 4
    finally:
        if logger is not None:
            logger.close()


def capture_ready(args: argparse.Namespace) -> int:
    if args.operator_gate != READY_GATE:
        raise Blocked(f"require --operator-gate '{READY_GATE}'")
    inventory = load_inventory(args.inventory)
    if inventory.get("status") != "STARTUP_STATE_PASS_AWAITING_READY_OPERATOR_POSE":
        raise Blocked(f"inventory status does not authorize READY capture: {inventory.get('status')}")
    logger = None
    try:
        inventory["ready_operator_pose_gate"] = "YES"
        exact_usb_identity()
        logger = RawCanLogger()
        time.sleep(0.1)
        motor = readonly_motor(logger)
        validate_identity(motor)
        if int(motor["ctrl_mode_rid10"]) != int(inventory["post_power_motor"]["ctrl_mode_rid10"]):
            raise Blocked("CTRL_MODE changed between post-power baseline and READY capture")
        values, _, stats = capture_disabled(
            logger, "READY_REFERENCE_V1_CAPTURE", "DM_G6220_PROTOCOL_PHYSICAL_POSE_REFERENCE"
        )
        stats.update(save_capture(args, inventory, logger, values, "j6_ready_reference_capture"))
        stats.update({
            "status": "PASS",
            "name": "J6_READY_REFERENCE_V1",
            "protocol_to_ros_sign": -1,
            "capture_state": "DISABLED",
            "cad_zero": False,
            "ros_zero": False,
            "motor_internal_zero": False,
            "blind_motion_target": False,
        })
        inventory["j6_ready_reference_v1"] = stats
        inventory["authority"]["j6_ready_reference"] = "FROZEN"
        inventory["status"] = "READY_CAPTURE_PASS_PENDING_OFFLINE_FINALIZATION"
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, indent=2))
        return 0
    except Blocked as exc:
        inventory["status"] = "BLOCKED"
        inventory["block_reason"] = str(exc)
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, indent=2))
        return 4
    finally:
        if logger is not None:
            logger.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("pre-power", "confirm-power-off", "post-power", "capture-ready"))
    parser.add_argument("--operator-gate", default="")
    parser.add_argument("--pose-moved", default="")
    parser.add_argument("--output-dir", type=Path, default=Path("hardware/v15_21f"))
    parser.add_argument(
        "--inventory",
        type=Path,
        default=Path("hardware/v15_21f/j6_startup_ready_inventory.json"),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.phase == "pre-power":
        return pre_power(args)
    if args.phase == "confirm-power-off":
        return confirm_power_off(args)
    if args.phase == "post-power":
        return post_power(args)
    return capture_ready(args)


if __name__ == "__main__":
    raise SystemExit(main())

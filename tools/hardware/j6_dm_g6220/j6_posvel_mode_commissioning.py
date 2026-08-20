#!/usr/bin/env python3
"""Operator-gated V15.21D runtime RID10 switch and DISABLED baseline."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import struct
import time
from pathlib import Path

from j6_raw_can_diagnostic import (
    Blocked,
    RawCanLogger,
    add_evidence,
    capture_refresh_samples,
    exact_usb_identity,
    read_parameter,
    state_text,
    timestamp_token,
    utc_now,
    write_json,
    write_raw_csv,
)


SOURCE_HEAD = "349fa982726e73cb799113f96cbb462e98fc85ca"
WORK_BRANCH = "agent/v15-21d-j6-posvel-enable-hold"
MODE_GATE = "J6_ALLOW_RUNTIME_CTRL_MODE_SWITCH_TO_POS_VEL=YES"
MOTOR_ID = 1
CTRL_MODE_MIT = 1
CTRL_MODE_POS_VEL = 2
RID_CTRL_MODE = 10


def base_inventory() -> dict:
    return {
        "schema_version": "1.0",
        "task": "V15.21D J6 POS_VEL safe enable/current-position hold diagnosis",
        "source_head": SOURCE_HEAD,
        "work_branch": WORK_BRANCH,
        "created_at_utc": utc_now(),
        "status": "NEW",
        "windows_reference": {
            "motion": "OPERATOR_VERIFIED_PASS",
            "default_mode": "POS_VEL",
            "switch_mode_default_enabled": True,
            "command_can_id": 0x101,
            "payload_format": "<ff position_rad velocity_rad_s",
        },
        "frozen_authority": {
            "v15_21a_communication": "PASS",
            "v15_21b_mit_hold": "FAIL",
            "v15_21c_root_cause": "REAL_ENABLE_TRANSIENT",
        },
        "safety": {
            "local_motion_authority": "NOT_GRANTED",
            "mit_tested": False,
            "kp_kd": "NOT_APPLICABLE_TO_POSVEL_TEST",
            "flash_eeprom_save_used": False,
            "set_zero_used": False,
            "id_write_used": False,
            "other_rid_write_used": False,
            "ready_pose_modified": False,
            "j1_j345_simulation_modified": False,
        },
        "evidence": [],
    }


def readonly_motor_state(logger: RawCanLogger) -> dict:
    master_id = int(read_parameter(logger, 7))
    logger.master_id = master_id
    return {
        "master_id_rid7": master_id,
        "esc_id_rid8": int(read_parameter(logger, 8)),
        "ctrl_mode_rid10": int(read_parameter(logger, 10)),
        "pmax_rid21": read_parameter(logger, 21),
        "allowed_normal_feedback_can_ids": sorted(logger.allowed_feedback_ids()),
    }


def preflight(args: argparse.Namespace) -> int:
    inventory = base_inventory()
    inventory["usb"] = exact_usb_identity()
    logger = RawCanLogger()
    try:
        time.sleep(0.1)
        motor = readonly_motor_state(logger)
        values, _ = capture_refresh_samples(logger, 5, "INITIAL_READ_ONLY")
        if motor["esc_id_rid8"] != MOTOR_ID:
            raise Blocked(f"RID8 changed: {motor['esc_id_rid8']}")
        if any(item[1].state != 0 for item in values):
            raise Blocked("initial feedback was not entirely DISABLED")
        inventory["motor"] = motor
        inventory["ctrl_mode_initial"] = motor["ctrl_mode_rid10"]
        inventory["initial_readonly"] = {
            "valid_frames": len(values),
            "states": [state_text(item[1].state) for item in values],
            "q_median_rad": statistics.median(item[1].position for item in values),
        }
        inventory["runtime_mode_write"] = {
            "operator_gate": "AWAITING",
            "rid10_write_count": 0,
            "flash_eeprom_save_used": False,
        }
        inventory["status"] = "AWAITING_RUNTIME_MODE_WRITE_GATE"
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
        logger.close()


def rid10_write_payload(value: int) -> bytes:
    return bytes((MOTOR_ID, 0, 0x55, RID_CTRL_MODE)) + struct.pack("<I", value)


def stats(values) -> dict:
    positions = [item[1].position for item in values]
    velocities = [item[1].velocity for item in values]
    return {
        "valid_frames": len(values),
        "total_required": len(values),
        "q_median_rad": statistics.median(positions),
        "q_mean_rad": statistics.fmean(positions),
        "q_std_rad": statistics.pstdev(positions),
        "q_min_rad": min(positions),
        "q_max_rad": max(positions),
        "velocity_min_rad_s": min(velocities),
        "velocity_max_rad_s": max(velocities),
        "mos_temperature_min_c": min(item[1].mos_temp for item in values),
        "mos_temperature_max_c": max(item[1].mos_temp for item in values),
        "coil_temperature_min_c": min(item[1].coil_temp for item in values),
        "coil_temperature_max_c": max(item[1].coil_temp for item in values),
        "states": sorted({state_text(item[1].state) for item in values}),
    }


def switch_and_baseline(args: argparse.Namespace) -> int:
    if args.operator_gate != MODE_GATE:
        raise Blocked(f"require --operator-gate '{MODE_GATE}'")
    inventory = json.loads(args.inventory.read_text(encoding="utf-8"))
    if inventory.get("status") != "AWAITING_RUNTIME_MODE_WRITE_GATE":
        raise Blocked(f"inventory status does not authorize RID10 write: {inventory.get('status')}")
    logger = RawCanLogger()
    write_sent = False
    try:
        exact_usb_identity()
        time.sleep(0.1)
        motor = readonly_motor_state(logger)
        if motor["esc_id_rid8"] != MOTOR_ID:
            raise Blocked(f"RID8 changed: {motor['esc_id_rid8']}")
        if motor["ctrl_mode_rid10"] != inventory["ctrl_mode_initial"]:
            raise Blocked("RID10 changed after preflight")
        if motor["ctrl_mode_rid10"] != CTRL_MODE_MIT:
            raise Blocked(f"expected initial MIT/1, observed {motor['ctrl_mode_rid10']}")
        before_values, _ = capture_refresh_samples(logger, 20, "PRE_SWITCH_DISABLED")
        if any(item[1].state != 0 for item in before_values):
            raise Blocked("motor was not DISABLED before RID10 write")
        q_before = statistics.median(item[1].position for item in before_values)

        logger.send(0x7FF, rid10_write_payload(CTRL_MODE_POS_VEL), "WRITE_RID10_POS_VEL_ONCE")
        write_sent = True
        inventory["runtime_mode_write"] = {
            "operator_gate": "PASS",
            "rid10_write_count": 1,
            "requested_value": CTRL_MODE_POS_VEL,
            "flash_eeprom_save_used": False,
        }
        readback = int(read_parameter(logger, RID_CTRL_MODE))
        inventory["ctrl_mode_after_write"] = readback
        if readback != CTRL_MODE_POS_VEL:
            raise Blocked(f"RID10 readback is {readback}, expected 2")

        baseline_values, _ = capture_refresh_samples(logger, 100, "POSVEL_DISABLED_BASELINE")
        baseline = stats(baseline_values)
        if len(baseline_values) != 100 or any(item[1].state != 0 for item in baseline_values):
            raise Blocked("POS_VEL DISABLED baseline acceptance failed")
        q_after = baseline["q_median_rad"]
        delta_deg = math.degrees(q_after - q_before)
        baseline.update({
            "status": "PASS" if abs(delta_deg) <= 0.2 else "FAIL",
            "q_before_switch_median_rad": q_before,
            "mode_switch_only_delta_rad": q_after - q_before,
            "mode_switch_only_delta_deg": delta_deg,
            "session_reference_semantics": "SESSION_LOCAL_ONLY_NOT_ZERO_OR_READY",
        })
        inventory["posvel_disabled_baseline"] = baseline
        inventory["j6_posvel_session_reference_rad"] = q_after
        raw_path = args.output_dir / f"j6_posvel_disabled_baseline_{timestamp_token()}.csv"
        write_raw_csv(raw_path, logger, logger.snapshot())
        add_evidence(inventory, raw_path)
        if abs(delta_deg) > 0.2:
            inventory["status"] = "FAIL"
            inventory["j6_posvel_enable_hold"] = "FAIL"
            inventory["fail_reason"] = "mode-switch-only displacement exceeded 0.2 deg"
            result = 2
        else:
            inventory["status"] = "POSVEL_BASELINE_PASS_AWAITING_MICRO_ENABLE_GATE"
            result = 0
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, indent=2))
        return result
    except Blocked as exc:
        inventory["runtime_mode_write"] = inventory.get("runtime_mode_write", {
            "operator_gate": "PASS",
            "rid10_write_count": int(write_sent),
            "flash_eeprom_save_used": False,
        })
        inventory["status"] = "BLOCKED"
        inventory["block_reason"] = str(exc)
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, indent=2))
        return 4
    finally:
        logger.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("preflight", "switch-and-baseline"))
    parser.add_argument("--operator-gate", default="")
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
        return preflight(args) if args.phase == "preflight" else switch_and_baseline(args)
    except Blocked as exc:
        print(f"BLOCKED: {exc}")
        return 4


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""Offline J4 direction migration; preserve raw encoder zeros and back up files."""
import argparse
import copy
import hashlib
import json
from contextlib import ExitStack
from datetime import datetime, timezone
from pathlib import Path

import v15_30a_rebase_whole_arm_software_zero as storage


def corrected_documents(zero, hints, initial, stamp):
    if zero["mapping"]["signs"]["J4"] != 1:
        raise ValueError("migration requires the old J4 +1 mapping")
    zero, hints, initial = map(copy.deepcopy, (zero, hints, initial))
    zero["mapping"]["signs"]["J4"] = -1
    zero["direction_correction"] = {
        "joint": "J4", "from_sign": 1, "to_sign": -1,
        "operator_evidence": "USER_REPORTED_J4_VIRTUAL_REAL_REVERSED_20260905",
        "recorded_at_utc": stamp, "raw_encoder_zeros_changed": False,
    }
    hints["motors"]["J4"]["logical_position_rad"] *= -1
    initial["关节位置_弧度"]["J4"] *= -1
    digest = hashlib.sha256(storage.json_bytes(zero)).hexdigest()
    initial["会话标识"] = "persistent:" + digest[:16]
    return zero, hints, initial, digest


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--state-dir", required=True, type=Path)
    parser.add_argument("--expected-zero-sha256", required=True)
    parser.add_argument("--apply", action="store_true")
    args = parser.parse_args()
    root = args.state_dir.resolve(strict=True)
    zero = root / "persistent_software_zero.json"
    sidecar = root / "persistent_software_zero.json.sha256"
    hints = root / "recovery_branch_hints.json"
    initial = root / "initial_pose.json"
    stamp = datetime.now(timezone.utc).isoformat()
    with ExitStack() as stack:
        if args.apply:
            storage.acquire_offline_locks(stack)
        paths = (zero, sidecar, hints, initial)
        for path in paths:
            storage.require_secure_production_file(path)
        originals = {path: path.read_bytes() for path in paths}
        old_sha = hashlib.sha256(originals[zero]).hexdigest()
        if old_sha != args.expected_zero_sha256 or originals[sidecar].decode().split()[0] != old_sha:
            raise ValueError("persistent zero hash mismatch")
        z, h, i, digest = corrected_documents(
            json.loads(originals[zero]), json.loads(originals[hints]),
            json.loads(originals[initial]), stamp,
        )
        backup = root / ("before_j4_direction_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ"))
        if args.apply:
            storage.apply_documents(
                zero_path=zero, sidecar_path=sidecar, hint_path=hints,
                initial_path=initial, backup_dir=backup,
                zero_data=storage.json_bytes(z), hint_data=storage.json_bytes(h),
                initial_data=storage.json_bytes(i),
                sidecar_data=(digest + "  " + zero.name + "\n").encode("ascii"),
                original_data=originals,
            )
        print(json.dumps({"applied": args.apply, "old_sha256": old_sha,
            "new_sha256": digest, "backup": str(backup) if args.apply else None,
            "hardware_accessed": False, "raw_encoder_zeros_changed": False}))


if __name__ == "__main__":
    main()

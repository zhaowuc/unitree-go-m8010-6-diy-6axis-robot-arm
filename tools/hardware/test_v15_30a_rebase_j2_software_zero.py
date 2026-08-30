#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import tempfile
import unittest
from pathlib import Path


MODULE_PATH = Path(__file__).with_name("v15_30a_rebase_j2_software_zero.py")
SPEC = importlib.util.spec_from_file_location("rebase_j2_zero", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def fixture_zero() -> dict:
    return {
        "schema": MODULE.ZERO_SCHEMA,
        "reference_name": MODULE.REFERENCE_NAME,
        "motors": {
            name: {
                "raw_position_rad": float(index),
                "sample_count": 100,
                "sample_span_rad": 0.001,
            }
            for index, name in enumerate(sorted(MODULE.EXPECTED_MOTORS))
        },
        "mapping": {
            "gear_ratio": MODULE.GEAR_RATIO,
            "signs": {
                "J1": 1,
                "J2A": -1,
                "J2B": 1,
                "J3": 1,
                "J4": 1,
                "J5": 1,
                "J6": -1,
            },
        },
        "writes": {
            "motor_internal_zero_modified": False,
            "rid_written": False,
            "flash_or_eeprom_written": False,
        },
    }


def fixture_hints() -> dict:
    return {
        "schema": MODULE.HINT_SCHEMA,
        "reference_name": MODULE.REFERENCE_NAME,
        "source": "old",
        "motors": {
            name: {"logical_position_rad": 0.1}
            for name in sorted(MODULE.EXPECTED_MOTORS - {"J6"})
        },
    }


def fixture_initial_pose(old_sha256: str) -> dict:
    return {
        "有效": True,
        "会话标识": f"persistent:{old_sha256[:16]}",
        "时间戳": "2026-08-25T07:40:28+00:00",
        "关节位置_弧度": {f"J{index}": 0.0 for index in range(1, 7)},
    }


class RebaseJ2SoftwareZeroTests(unittest.TestCase):
    def test_build_updates_only_j2_and_resets_hints(self):
        zero = fixture_zero()
        hints = fixture_hints()
        original = json.loads(json.dumps(zero))
        updated, updated_hints = MODULE.build_documents(
            zero,
            hints,
            original_sha256="a" * 64,
            j2a_raw=1.25,
            j2b_raw=2.50,
            j2a_span=0.001,
            j2b_span=0.002,
            sample_count=501,
            recorded_at_utc="2026-08-27T10:05:00+00:00",
            source="test capture",
        )
        self.assertEqual(updated["motors"]["J2A"]["raw_position_rad"], 1.25)
        self.assertEqual(updated["motors"]["J2B"]["raw_position_rad"], 2.50)
        for name in MODULE.EXPECTED_MOTORS - {"J2A", "J2B"}:
            self.assertEqual(updated["motors"][name], original["motors"][name])
        self.assertEqual(updated_hints["motors"]["J2A"]["logical_position_rad"], 0.0)
        self.assertEqual(updated_hints["motors"]["J2B"]["logical_position_rad"], 0.0)
        self.assertFalse(updated["last_software_zero_rebase"]["rid_written"])

    def test_rejects_unsafe_span_and_short_capture(self):
        with self.assertRaises(ValueError):
            MODULE.validate_measurement(1.0, math.radians(10.0), 501, "J2A")
        with self.assertRaises(ValueError):
            MODULE.validate_measurement(1.0, 0.001, 49, "J2A")

    def test_apply_is_backed_up_and_checksum_commits_last(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            zero_path = root / "persistent_software_zero.json"
            sidecar_path = root / "persistent_software_zero.json.sha256"
            hint_path = root / "recovery_branch_hints.json"
            initial_pose_path = root / "initial_pose.json"
            zero_data = MODULE.json_bytes(fixture_zero())
            hint_data = MODULE.json_bytes(fixture_hints())
            zero_path.write_bytes(zero_data)
            hint_path.write_bytes(hint_data)
            old_hash = hashlib.sha256(zero_data).hexdigest()
            initial_pose_path.write_bytes(MODULE.json_bytes(fixture_initial_pose(old_hash)))
            sidecar_path.write_text(
                f"{old_hash}  {zero_path.name}\n", encoding="utf-8"
            )
            updated, updated_hints = MODULE.build_documents(
                fixture_zero(),
                fixture_hints(),
                original_sha256=old_hash,
                j2a_raw=1.25,
                j2b_raw=2.50,
                j2a_span=0.001,
                j2b_span=0.002,
                sample_count=501,
                recorded_at_utc="2026-08-27T10:05:00+00:00",
                source="test capture",
            )
            new_zero_data = MODULE.json_bytes(updated)
            new_hash = hashlib.sha256(new_zero_data).hexdigest()
            new_initial_pose = MODULE.build_initial_pose(
                fixture_initial_pose(old_hash),
                new_sha256=new_hash,
                recorded_at_utc="2026-08-27T10:05:00+00:00",
            )
            backup = root / "backup"
            MODULE.apply_documents(
                zero_path=zero_path,
                sidecar_path=sidecar_path,
                hint_path=hint_path,
                initial_pose_path=initial_pose_path,
                backup_dir=backup,
                zero_data=new_zero_data,
                hint_data=MODULE.json_bytes(updated_hints),
                initial_pose_data=MODULE.json_bytes(new_initial_pose),
                sidecar_data=f"{new_hash}  {zero_path.name}\n".encode("ascii"),
            )
            self.assertEqual(hashlib.sha256(zero_path.read_bytes()).hexdigest(), new_hash)
            self.assertEqual(MODULE.parse_sidecar(sidecar_path, zero_path.name), new_hash)
            self.assertEqual(hashlib.sha256((backup / zero_path.name).read_bytes()).hexdigest(), old_hash)
            written_initial = json.loads(initial_pose_path.read_text(encoding="utf-8"))
            self.assertEqual(written_initial["会话标识"], f"persistent:{new_hash[:16]}")


if __name__ == "__main__":
    unittest.main()

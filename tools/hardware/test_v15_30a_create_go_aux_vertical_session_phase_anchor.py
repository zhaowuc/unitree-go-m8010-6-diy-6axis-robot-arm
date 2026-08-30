#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import importlib.util
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace


MODULE_PATH = Path(__file__).with_name(
    "v15_30a_create_go_aux_vertical_session_phase_anchor.py"
)
SPEC = importlib.util.spec_from_file_location("go_aux_vertical_phase_anchor", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

BOOT_ID = "12345678-1234-5678-9abc-1234567890ab"
POWER_SESSION_ID = "go-aux-power-session-test-01"
CAPTURE_BOOT_NS = 10_000_000_000
ISSUED_BOOT_NS = 20_000_000_000
WORKER_SHA = "f" * 64


def raw_record(mean: float, count: int = 500) -> dict:
    span = 0.001
    return {
        "sample_count": count,
        "unwrapped_raw_position_rad": {
            "mean": mean,
            "minimum": mean - span / 2.0,
            "maximum": mean + span / 2.0,
            "span": span,
            "standard_deviation": span / 6.0,
        },
    }


def capture(count: int = 500) -> dict:
    names_by_bus = {"j1": ("J1",), "j345": ("J3", "J4", "J5")}
    means = {"J1": 13.9655, "J3": -2.2, "J4": 2.4, "J5": -0.7}
    return {
        "schema": MODULE.CAPTURE_SCHEMA,
        "capture_id": "go-aux-brake-raw-test-01",
        "power_session_id": POWER_SESSION_ID,
        "recorded_at_utc": "2026-08-28T01:02:03Z",
        "host_boot_id": BOOT_ID,
        "recorded_boottime_ns": CAPTURE_BOOT_NS,
        "status": "PASS",
        "hardware_accessed": True,
        "physical_power_state_during_capture": "24V_ON",
        "worker": {"path": "/tmp/test-go-worker", "sha256": WORKER_SHA},
        "domains": {
            bus: {
                "packet_count": count,
                "source_coverage_s": 4.99,
                "motor_names": list(names),
            }
            for bus, names in names_by_bus.items()
        },
        "motors": {name: raw_record(mean, count) for name, mean in means.items()},
        "safety": {
            "execution_policy": "BRAKE_ONLY",
            "all_controller_modes": ["brake"],
            "command_rx_enabled": False,
            "foc_tx_attempt_count": 0,
            "foc_serial_send_call_count": 0,
            "other_mode_tx_attempt_count": 0,
            "active_or_hold_commands_sent": 0,
            "communication_failure_packets": 0,
            "merror_nonzero_packets": 0,
            "motor_internal_zero_modified": False,
            "rid_written": False,
            "flash_or_eeprom_written": False,
        },
    }


def write_json(path: Path, value: dict) -> bytes:
    data = MODULE.base.json_bytes(value)
    path.write_bytes(data)
    return data


class GoAuxVerticalSessionPhaseAnchorTest(unittest.TestCase):
    def test_initial_pose_wins_over_stale_hints_and_two_permits_are_distinct(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            zero = {
                "schema": MODULE.base.ZERO_SCHEMA,
                "reference_name": MODULE.base.REFERENCE_NAME,
                "motors": {
                    name: {"raw_position_rad": float(index)}
                    for index, name in enumerate(sorted(MODULE.base.EXPECTED_MOTORS))
                },
                "mapping": {
                    "gear_ratio": MODULE.base.GEAR_RATIO,
                    "signs": {
                        "J1": 1, "J2A": -1, "J2B": 1,
                        "J3": 1, "J4": 1, "J5": 1, "J6": -1,
                    },
                },
                "writes": {
                    "motor_internal_zero_modified": False,
                    "rid_written": False,
                    "flash_or_eeprom_written": False,
                },
            }
            zero_path = root / "persistent_software_zero.json"
            zero_data = write_json(zero_path, zero)
            zero_sha = hashlib.sha256(zero_data).hexdigest()
            sidecar_path = root / "persistent_software_zero.json.sha256"
            sidecar_path.write_text(f"{zero_sha}  {zero_path.name}\n", encoding="ascii")
            hints = {
                "schema": MODULE.base.HINT_SCHEMA,
                "reference_name": MODULE.base.REFERENCE_NAME,
                "motors": {
                    "J1": {"logical_position_rad": 0.0},
                    "J2A": {"logical_position_rad": 0.0},
                    "J2B": {"logical_position_rad": 0.0},
                    "J3": {"logical_position_rad": -0.2882325},
                    "J4": {"logical_position_rad": 0.3028612},
                    "J5": {"logical_position_rad": 0.0},
                },
            }
            hints_path = root / "recovery_branch_hints.json"
            hints_data = write_json(hints_path, hints)
            initial = {
                "有效": True,
                "会话标识": f"persistent:{zero_sha[:16]}",
                "关节位置_弧度": {f"J{index}": 0.0 for index in range(1, 7)},
            }
            initial_path = root / "initial_pose.json"
            initial_data = write_json(initial_path, initial)
            capture_path = root / "go_aux_raw.json"
            capture_data = write_json(capture_path, capture())
            protected_before = {
                path: path.read_bytes()
                for path in (zero_path, sidecar_path, hints_path, initial_path, capture_path)
            }
            args = SimpleNamespace(
                zero_file=zero_path,
                zero_sha256_file=sidecar_path,
                expected_parent_zero_sha256=zero_sha,
                recovery_hint_file=hints_path,
                expected_recovery_hint_sha256=hashlib.sha256(hints_data).hexdigest(),
                initial_pose_file=initial_path,
                expected_initial_pose_sha256=hashlib.sha256(initial_data).hexdigest(),
                capture_statistics_file=capture_path,
                expected_capture_sha256=hashlib.sha256(capture_data).hexdigest(),
                operator_evidence_id="operator-test-01",
                operator_confirmed_at_utc="2026-08-28T01:02:10Z",
                operator_power_session_id=POWER_SESSION_ID,
                physical_confirmation=MODULE.PHYSICAL_GATE,
                anchor_directory=root / "anchors",
                launch_permit_directory=root / "permits" / "pending",
                apply=False,
                confirm="",
            )
            result = MODULE.run(
                args,
                now_utc=datetime(2026, 8, 28, 1, 2, 12, tzinfo=timezone.utc),
                host_boot_id=BOOT_ID,
                issued_boottime_ns=ISSUED_BOOT_NS,
            )
            anchor = result["anchor"]
            permits = result["launch_permits"]
            self.assertTrue(result["protected_inputs_unchanged"])
            self.assertFalse(anchor["preserved_inputs"]["recovery_hints_used_for_go_aux"])
            for name in ("J1", "J3", "J4", "J5"):
                self.assertEqual(anchor["motors"][name]["logical_position_rad"], 0.0)
                self.assertEqual(
                    anchor["motors"][name]["logical_position_source"],
                    "HASH_BOUND_READ_ONLY_INITIAL_POSE",
                )
            self.assertEqual(set(permits), {"j1", "j345"})
            self.assertNotEqual(permits["j1"]["permit_id"], permits["j345"]["permit_id"])
            self.assertEqual(permits["j1"]["scope"], "j1")
            self.assertEqual(permits["j345"]["scope"], "j345")
            self.assertTrue(all(path.read_bytes() == data for path, data in protected_before.items()))

    def test_rejects_any_domain_below_500_brake_samples(self):
        with self.assertRaises(MODULE.base.AnchorValidationError):
            MODULE._validate_capture(capture(499))


if __name__ == "__main__":
    unittest.main()

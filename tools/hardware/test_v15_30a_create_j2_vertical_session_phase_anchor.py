#!/usr/bin/env python3

from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import os
import tempfile
import unittest
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace


MODULE_PATH = Path(__file__).with_name(
    "v15_30a_create_j2_vertical_session_phase_anchor.py"
)
SPEC = importlib.util.spec_from_file_location("j2_vertical_phase_anchor", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


TEST_HOST_BOOT_ID = "12345678-1234-5678-9abc-1234567890ab"
OTHER_HOST_BOOT_ID = "aaaaaaaa-bbbb-4ccc-8ddd-eeeeeeeeeeee"
TEST_RECORDED_BOOTTIME_NS = 10_000_000_000
TEST_ISSUED_BOOTTIME_NS = 20_000_000_000
TEST_ISSUED_AT_UTC = datetime(2026, 8, 28, 1, 2, 12, tzinfo=timezone.utc)
TEST_WORKER_SHA256 = "f" * 64
TEST_WORKER_PATH = str(
    (MODULE_PATH.parent / "fixtures" / "hash-pinned-worker").resolve()
)


def fixture_zero() -> dict:
    return {
        "schema": MODULE.ZERO_SCHEMA,
        "created_at_utc": "2026-08-25T07:30:21+00:00",
        "reference_name": MODULE.REFERENCE_NAME,
        "motors": {
            name: {
                "raw_position_rad": (
                    -4.596924286444928
                    if name == "J2A"
                    else 12.4254104489523
                    if name == "J2B"
                    else float(index)
                ),
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
        "source": "preserved vertical initialization",
        "motors": {
            name: {"logical_position_rad": 0.0}
            for name in sorted(MODULE.EXPECTED_MOTORS - {"J6"})
        },
    }


def fixture_initial_pose(parent_sha256: str) -> dict:
    return {
        "有效": True,
        "会话标识": f"persistent:{parent_sha256[:16]}",
        "时间戳": "2026-08-27T10:41:24+00:00",
        "语义": "持久软件零位；断电后重新加载可返回",
        "关节位置_弧度": {f"J{index}": 0.0 for index in range(1, 7)},
    }


def raw_record(mean: float, *, span: float = 0.001, count: int = 500) -> dict:
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


def fixture_capture() -> dict:
    return {
        "schema": MODULE.CAPTURE_SCHEMA,
        "capture_id": "j2-brake-20260828T010203Z",
        "power_session_id": "j2-power-session-20260828-01",
        "recorded_at_utc": "2026-08-28T01:02:03+00:00",
        "host_boot_id": TEST_HOST_BOOT_ID,
        "recorded_boottime_ns": TEST_RECORDED_BOOTTIME_NS,
        "status": "PASS",
        "physical_power_state_during_capture": "24V_ON",
        "packet_count": 500,
        "source_coverage_s": 4.99,
        "safety": {
            "execution_policy": "BRAKE_ONLY",
            "all_controller_modes": ["brake"],
            "command_rx_enabled": False,
            "foc_tx_attempt_count": 0,
            "active_or_hold_commands_sent": 0,
            "communication_failure_packets": 0,
            "merror_nonzero_packets": 0,
            "motor_internal_zero_modified": False,
            "rid_written": False,
            "flash_or_eeprom_written": False,
            "startup_brake_prime_passed": True,
            "startup_invalid_prefix_pairs": 3,
        },
        "motors": {
            "J2A": raw_record(5.04623),
            "J2B": raw_record(2.76539),
        },
        "raw_safety_summary": {
            "startup_brake_prime": {
                "state": "PASS",
                "maximum_invalid_prefix_pairs": 3,
                "invalid_prefix_pairs": 3,
                "required_healthy_pairs": 5,
                "healthy_pairs": 5,
                "attempted_pairs": 8,
                "tx_attempt_count": 16,
                "elapsed_ns": 180_000_000,
                "feedback_published_during_prime": False,
                "window_reopens_after_first_healthy_pair": False,
            },
            "phase_tx_accounting": {
                "completed_cycles": 500,
                "startup_prime_tx_attempt_count": 16,
                "control_loop_tx_attempt_count": 1000,
                "final_brake_tx_attempt_count": 40,
                "aggregate_tx_attempt_count": 1056,
            },
            "worker": {
                "path": TEST_WORKER_PATH,
                "sha256": TEST_WORKER_SHA256,
                "returncode": 0,
                "stop_escalation": "none",
                "forced_kill": False,
                "terminal_proof": {
                    "TERMINAL_PATH": "NORMAL",
                    "GUI_GO_CONTROLLER_BUS": "j2",
                    "EXECUTION_POLICY": "BRAKE_ONLY",
                    "COMMAND_RX_ENABLED": "NO",
                    "J2_SESSION_REFERENCE_CONFIGURED": "NO",
                    "FOC_TX_ATTEMPT_COUNT": "0",
                    "OTHER_MODE_TX_ATTEMPT_COUNT": "0",
                    "BRAKE_ONLY_GUARD_BLOCK_COUNT": "0",
                    "FOC_SERIAL_SEND_CALL_COUNT": "0",
                    "BRAKE_ONLY_AUDIT": "PASS",
                    "FINAL_MODE": "BRAKE",
                    "FINAL_BRAKE": "PASS",
                    "MOTOR_INTERNAL_ZERO_WRITE": "NO",
                    "J2_STARTUP_BRAKE_PRIME_STATE": "PASS",
                    "J2_STARTUP_BRAKE_PRIME_MAX_INVALID_PREFIX_PAIRS": "3",
                    "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS": "3",
                    "J2_STARTUP_BRAKE_PRIME_REQUIRED_HEALTHY_PAIRS": "5",
                    "J2_STARTUP_BRAKE_PRIME_HEALTHY_PAIRS": "5",
                    "J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS": "8",
                    "J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT": "16",
                    "J2_STARTUP_BRAKE_PRIME_ELAPSED_NS": "180000000",
                    "COMPLETED_CYCLES": "500",
                    "FINAL_BRAKE_TX_ATTEMPT_COUNT": "40",
                    "TX_ATTEMPT_TOTAL": "1056",
                    "BRAKE_TX_ATTEMPT_COUNT": "1056",
                    "SERIAL_SEND_CALL_COUNT": "1056",
                },
            }
        },
    }


def write_json(path: Path, document: dict) -> bytes:
    data = MODULE.json_bytes(document)
    path.write_bytes(data)
    return data


def make_fixture_tree(root: Path) -> tuple[SimpleNamespace, dict[str, bytes]]:
    zero_path = root / "persistent_software_zero.json"
    sidecar_path = root / "persistent_software_zero.json.sha256"
    hint_path = root / "recovery_branch_hints.json"
    initial_pose_path = root / "initial_pose.json"
    capture_path = root / "j2_raw_capture.json"
    zero_data = write_json(zero_path, fixture_zero())
    zero_sha = hashlib.sha256(zero_data).hexdigest()
    sidecar_data = f"{zero_sha}  {zero_path.name}\n".encode("ascii")
    sidecar_path.write_bytes(sidecar_data)
    hint_data = write_json(hint_path, fixture_hints())
    initial_data = write_json(initial_pose_path, fixture_initial_pose(zero_sha))
    capture_data = write_json(capture_path, fixture_capture())
    blobs = {
        "zero": zero_data,
        "sidecar": sidecar_data,
        "hints": hint_data,
        "initial_pose": initial_data,
        "capture": capture_data,
    }
    args = SimpleNamespace(
        zero_file=zero_path,
        zero_sha256_file=sidecar_path,
        expected_parent_zero_sha256=zero_sha,
        recovery_hint_file=hint_path,
        expected_recovery_hint_sha256=hashlib.sha256(hint_data).hexdigest(),
        initial_pose_file=initial_pose_path,
        expected_initial_pose_sha256=hashlib.sha256(initial_data).hexdigest(),
        capture_statistics_file=capture_path,
        expected_capture_sha256=hashlib.sha256(capture_data).hexdigest(),
        operator_evidence_id="onsite-user-confirmation-20260828-01",
        operator_confirmed_at_utc="2026-08-28T01:02:10+00:00",
        operator_power_session_id="j2-power-session-20260828-01",
        physical_confirmation=MODULE.PHYSICAL_GATE,
        anchor_directory=root / "j2_vertical_phase_anchors",
        launch_permit_directory=root / "launch_permits" / "pending",
        apply=False,
        confirm="",
    )
    return args, blobs


def run_tool(args: SimpleNamespace, **overrides):
    runtime = {
        "now_utc": TEST_ISSUED_AT_UTC,
        "host_boot_id": TEST_HOST_BOOT_ID,
        "issued_boottime_ns": TEST_ISSUED_BOOTTIME_NS,
    }
    runtime.update(overrides)
    return MODULE.run(args, **runtime)


class J2VerticalSessionPhaseAnchorTests(unittest.TestCase):
    def test_builds_phase_evidence_without_rebasing_parent(self):
        capture = MODULE.validate_capture(fixture_capture())
        confirmation = MODULE.validate_operator_confirmation(
            evidence_id="operator-evidence-1",
            confirmed_at_utc="2026-08-28T01:02:10Z",
            power_session_id="j2-power-session-20260828-01",
            physical_confirmation=MODULE.PHYSICAL_GATE,
            capture_power_session_id=capture["power_session_id"],
        )
        zero = fixture_zero()
        original = json.loads(json.dumps(zero))
        anchor = MODULE.build_anchor(
            zero=zero,
            parent_zero_sha256="a" * 64,
            parent_sidecar_sha256="b" * 64,
            recovery_hint_sha256="c" * 64,
            initial_pose_sha256="d" * 64,
            capture_source_path="/evidence/j2_capture.json",
            capture_file_sha256="e" * 64,
            capture=capture,
            operator_confirmation=confirmation,
        )
        self.assertEqual(zero, original)
        self.assertEqual(anchor["anchor_version"], 1)
        self.assertFalse(anchor["control_authority"]["is_software_zero"])
        self.assertFalse(anchor["control_authority"]["authorizes_active_control"])
        self.assertFalse(anchor["derivation"]["integer_turn_branch_inferred"])
        self.assertFalse(anchor["preserved_inputs"]["overwritten_or_deleted"])
        self.assertEqual(anchor["parent_persistent_zero_sha256"], "a" * 64)
        self.assertEqual(anchor["reference_name"], MODULE.REFERENCE_NAME)
        self.assertEqual(anchor["source_evidence"]["sha256"], "e" * 64)
        self.assertEqual(anchor["raw_capture"]["host_boot_id"], TEST_HOST_BOOT_ID)
        self.assertEqual(
            anchor["raw_capture"]["recorded_boottime_ns"],
            TEST_RECORDED_BOOTTIME_NS,
        )
        self.assertEqual(
            anchor["raw_capture"]["expected_worker_sha256"],
            TEST_WORKER_SHA256,
        )
        self.assertEqual(
            anchor["raw_capture"]["startup_brake_prime"][
                "invalid_prefix_pairs"
            ],
            3,
        )
        self.assertEqual(
            anchor["raw_capture"]["phase_tx_accounting"],
            capture["phase_tx_accounting"],
        )
        self.assertEqual(
            anchor["raw_capture"]["worker"]["terminal_proof"],
            capture["worker"]["terminal_proof"],
        )
        self.assertEqual(set(anchor["motors"]), {"J2A", "J2B"})
        self.assertEqual(anchor["motors"]["J2A"]["logical_position_rad"], 0.0)
        self.assertTrue(
            anchor["operator_confirmation"]["vertical_initialization_pose"]
        )
        self.assertEqual(
            anchor["writes"],
            {
                "motor_internal_zero_modified": False,
                "rid_written": False,
                "flash_or_eeprom_written": False,
            },
        )
        self.assertAlmostEqual(
            anchor["raw_capture"]["motors"]["J2A"]["single_turn_phase_rad"],
            (5.04623 + math.pi) % (2.0 * math.pi) - math.pi,
            places=12,
        )

    def test_rejects_unsafe_or_inconsistent_capture(self):
        capture = fixture_capture()
        capture["safety"]["foc_tx_attempt_count"] = 1
        with self.assertRaises(MODULE.AnchorValidationError):
            MODULE.validate_capture(capture)
        capture = fixture_capture()
        capture["motors"]["J2A"] = raw_record(
            5.0, span=MODULE.MAX_RAW_SPAN_RAD + 0.001
        )
        with self.assertRaises(MODULE.AnchorValidationError):
            MODULE.validate_capture(capture)
        capture = fixture_capture()
        capture["motors"]["J2B"]["sample_count"] = 499
        with self.assertRaises(MODULE.AnchorValidationError):
            MODULE.validate_capture(capture)

    def test_confirmation_must_bind_same_power_session_and_all_physical_gates(self):
        with self.assertRaises(MODULE.AnchorValidationError):
            MODULE.validate_operator_confirmation(
                evidence_id="operator-evidence-1",
                confirmed_at_utc="2026-08-28T01:02:10Z",
                power_session_id="different-session",
                physical_confirmation=MODULE.PHYSICAL_GATE,
                capture_power_session_id="capture-session",
            )
        with self.assertRaises(MODULE.AnchorValidationError):
            MODULE.validate_operator_confirmation(
                evidence_id="operator-evidence-1",
                confirmed_at_utc="2026-08-28T01:02:10Z",
                power_session_id="capture-session",
                physical_confirmation="J2_VERTICAL=YES",
                capture_power_session_id="capture-session",
            )

    def test_dry_run_is_default_and_writes_nothing(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args, blobs = make_fixture_tree(root)
            result = run_tool(args)
            self.assertFalse(result["applied"])
            self.assertFalse(args.anchor_directory.exists())
            self.assertFalse(args.launch_permit_directory.exists())
            self.assertEqual(
                result["launch_permit"]["schema"], MODULE.LAUNCH_PERMIT_SCHEMA
            )
            for key, path in (
                ("zero", args.zero_file),
                ("sidecar", args.zero_sha256_file),
                ("hints", args.recovery_hint_file),
                ("initial_pose", args.initial_pose_file),
                ("capture", args.capture_statistics_file),
            ):
                self.assertEqual(path.read_bytes(), blobs[key])

    def test_apply_creates_one_versioned_anchor_and_preserves_all_inputs(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args, blobs = make_fixture_tree(root)
            args.apply = True
            args.confirm = MODULE.APPLY_GATE
            result = run_tool(args)
            output = Path(result["output_path"])
            permit = Path(result["permit_path"])
            self.assertTrue(output.is_file())
            self.assertTrue(permit.is_file())
            self.assertTrue(output.name.startswith("j2_power_session_reference_v1_"))
            self.assertEqual(
                hashlib.sha256(output.read_bytes()).hexdigest(),
                result["anchor_sha256"],
            )
            self.assertEqual([output], list(args.anchor_directory.glob("*.json")))
            self.assertEqual(
                hashlib.sha256(permit.read_bytes()).hexdigest(),
                result["permit_sha256"],
            )
            self.assertEqual(
                [permit], list(args.launch_permit_directory.glob("*.json"))
            )
            inflight = Path(
                json.loads(permit.read_text(encoding="utf-8"))["inflight_path"]
            )
            spent = Path(
                json.loads(permit.read_text(encoding="utf-8"))["spent_path"]
            )
            self.assertTrue(inflight.parent.is_dir())
            self.assertTrue(spent.parent.is_dir())
            self.assertFalse(inflight.exists())
            self.assertFalse(spent.exists())
            if os.name != "nt":
                self.assertEqual(output.stat().st_mode & 0o777, 0o600)
                self.assertEqual(permit.stat().st_mode & 0o777, 0o600)
                for private_directory in (
                    args.anchor_directory,
                    args.launch_permit_directory,
                    inflight.parent,
                    spent.parent,
                ):
                    self.assertEqual(private_directory.stat().st_mode & 0o777, 0o700)
            for key, path in (
                ("zero", args.zero_file),
                ("sidecar", args.zero_sha256_file),
                ("hints", args.recovery_hint_file),
                ("initial_pose", args.initial_pose_file),
                ("capture", args.capture_statistics_file),
            ):
                self.assertEqual(path.read_bytes(), blobs[key])
            with self.assertRaises(MODULE.AnchorValidationError):
                run_tool(args)
            self.assertEqual(1, len(list(args.anchor_directory.glob("*.json"))))
            self.assertEqual(
                1, len(list(args.launch_permit_directory.glob("*.json")))
            )

    def test_deferred_permit_is_published_later_without_rewriting_anchor(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args, _ = make_fixture_tree(root)
            args.apply = True
            args.confirm = MODULE.APPLY_GATE
            args.defer_launch_permit = True
            args.publish_deferred_launch_permit = False
            deferred = run_tool(args)
            anchor_path = Path(deferred["anchor_path"])
            permit_path = Path(deferred["permit_path"])
            anchor_bytes = anchor_path.read_bytes()
            self.assertTrue(deferred["anchor_published"])
            self.assertFalse(deferred["launch_permit_published"])
            self.assertTrue(anchor_path.is_file())
            self.assertFalse(permit_path.exists())

            args.defer_launch_permit = False
            args.publish_deferred_launch_permit = True
            published = run_tool(
                args,
                now_utc=TEST_ISSUED_AT_UTC.replace(second=20),
                issued_boottime_ns=TEST_ISSUED_BOOTTIME_NS + 8_000_000_000,
            )
            self.assertFalse(published["anchor_published"])
            self.assertTrue(published["launch_permit_published"])
            self.assertEqual(anchor_path.read_bytes(), anchor_bytes)
            self.assertTrue(permit_path.is_file())
            with self.assertRaises(MODULE.AnchorValidationError):
                run_tool(
                    args,
                    now_utc=TEST_ISSUED_AT_UTC.replace(second=21),
                    issued_boottime_ns=TEST_ISSUED_BOOTTIME_NS + 9_000_000_000,
                )

    def test_deferred_permit_rejects_changed_or_missing_anchor(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            args, _ = make_fixture_tree(root)
            args.apply = True
            args.confirm = MODULE.APPLY_GATE
            args.defer_launch_permit = False
            args.publish_deferred_launch_permit = True
            with self.assertRaisesRegex(
                MODULE.AnchorValidationError, "exact immutable anchor"
            ):
                run_tool(args)

            args.defer_launch_permit = True
            args.publish_deferred_launch_permit = False
            deferred = run_tool(args)
            anchor_path = Path(deferred["anchor_path"])
            anchor_path.write_bytes(anchor_path.read_bytes() + b" ")
            args.defer_launch_permit = False
            args.publish_deferred_launch_permit = True
            with self.assertRaisesRegex(
                MODULE.AnchorValidationError, "exact immutable anchor"
            ):
                run_tool(
                    args,
                    now_utc=TEST_ISSUED_AT_UTC.replace(second=20),
                    issued_boottime_ns=TEST_ISSUED_BOOTTIME_NS + 8_000_000_000,
                )

    def test_apply_requires_explicit_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            args, _ = make_fixture_tree(Path(directory))
            args.apply = True
            with self.assertRaises(MODULE.AnchorValidationError):
                run_tool(args)
            self.assertFalse(args.anchor_directory.exists())

    def test_pinned_parent_and_preserved_hashes_are_mandatory(self):
        with tempfile.TemporaryDirectory() as directory:
            args, _ = make_fixture_tree(Path(directory))
            args.expected_parent_zero_sha256 = "0" * 64
            with self.assertRaises(MODULE.AnchorValidationError):
                run_tool(args)
        with tempfile.TemporaryDirectory() as directory:
            args, _ = make_fixture_tree(Path(directory))
            args.expected_initial_pose_sha256 = "0" * 64
            with self.assertRaises(MODULE.AnchorValidationError):
                run_tool(args)

    def test_launch_permit_contract_ttl_paths_and_pinned_worker(self):
        with tempfile.TemporaryDirectory() as directory:
            args, _ = make_fixture_tree(Path(directory))
            result = run_tool(args)
            permit = result["launch_permit"]
            self.assertEqual(permit["schema"], MODULE.LAUNCH_PERMIT_SCHEMA)
            self.assertEqual(permit["permit_version"], 1)
            self.assertEqual(permit["permit_id"], result["permit_id"])
            self.assertEqual(len(permit["permit_id"]), 64)
            self.assertTrue(
                all(character in "0123456789abcdef" for character in permit["permit_id"])
            )
            self.assertEqual(permit["scope"], "j2")
            self.assertIs(permit["single_use"], True)
            self.assertEqual(
                permit["power_session_id"], "j2-power-session-20260828-01"
            )
            self.assertEqual(permit["host_boot_id"], TEST_HOST_BOOT_ID)
            self.assertEqual(
                permit["source_capture"],
                {
                    "host_boot_id": TEST_HOST_BOOT_ID,
                    "recorded_boottime_ns": TEST_RECORDED_BOOTTIME_NS,
                },
            )
            self.assertEqual(permit["ttl_seconds"], 30)
            self.assertEqual(
                permit["expires_unix_s"] - permit["issued_unix_s"], 30
            )
            self.assertEqual(
                permit["expires_boottime_ns"] - permit["issued_boottime_ns"],
                30_000_000_000,
            )
            issued = datetime.fromisoformat(
                permit["issued_at_utc"].replace("Z", "+00:00")
            )
            expires = datetime.fromisoformat(
                permit["expires_at_utc"].replace("Z", "+00:00")
            )
            self.assertEqual((expires - issued).total_seconds(), 30.0)
            self.assertEqual(
                permit["parent_persistent_zero_sha256"],
                result["parent_persistent_zero_sha256"],
            )
            self.assertEqual(
                permit["session_reference"],
                {
                    "path": str(Path(result["anchor_path"]).resolve()),
                    "sha256": result["anchor_sha256"],
                },
            )
            self.assertEqual(
                permit["worker"],
                {"path": TEST_WORKER_PATH, "sha256": TEST_WORKER_SHA256},
            )
            lifecycle = [
                Path(permit[field])
                for field in ("pending_path", "inflight_path", "spent_path")
            ]
            self.assertTrue(all(path.is_absolute() for path in lifecycle))
            self.assertEqual({path.name for path in lifecycle}, {lifecycle[0].name})
            self.assertEqual(lifecycle[0], Path(result["permit_path"]).resolve())
            self.assertEqual(
                [path.parent.name for path in lifecycle],
                ["pending", "inflight", "spent"],
            )
            self.assertEqual(
                permit["serial"],
                {
                    "stable_by_id": MODULE.STABLE_J2_SERIAL_BY_ID,
                    "bus": "j2",
                    "motor_ids": [0, 1],
                    "gear_ratio": MODULE.GEAR_RATIO,
                    "signs": {"J2A": -1, "J2B": 1},
                },
            )
            self.assertEqual(
                permit["startup_recheck"]["minimum_brake_frames"], 50
            )
            self.assertAlmostEqual(
                permit["startup_recheck"]["max_raw_phase_delta_rad"],
                MODULE.GEAR_RATIO * math.radians(0.25),
            )
            self.assertAlmostEqual(
                permit["startup_recheck"]["max_raw_span_rad"],
                MODULE.GEAR_RATIO * math.radians(0.20),
            )
            self.assertFalse(result["active_control_authorized"])
            self.assertFalse(
                result["anchor"]["control_authority"]["authorizes_active_control"]
            )
            self.assertEqual(
                hashlib.sha256(MODULE.json_bytes(permit)).hexdigest(),
                result["permit_sha256"],
            )
            later = run_tool(
                args,
                now_utc=TEST_ISSUED_AT_UTC.replace(second=13),
                issued_boottime_ns=TEST_ISSUED_BOOTTIME_NS + 1_000_000_000,
            )
            self.assertEqual(later["permit_id"], result["permit_id"])
            self.assertEqual(later["permit_path"], result["permit_path"])

    def test_permit_requires_same_boot_and_fresh_monotonic_capture(self):
        with tempfile.TemporaryDirectory() as directory:
            args, _ = make_fixture_tree(Path(directory))
            with self.assertRaisesRegex(
                MODULE.AnchorValidationError, "current host boot id differs"
            ):
                run_tool(args, host_boot_id=OTHER_HOST_BOOT_ID)
            with self.assertRaisesRegex(
                MODULE.AnchorValidationError, "precedes raw capture"
            ):
                run_tool(
                    args,
                    issued_boottime_ns=TEST_RECORDED_BOOTTIME_NS - 1,
                )
            boundary = run_tool(
                args,
                issued_boottime_ns=(
                    TEST_RECORDED_BOOTTIME_NS + MODULE.MAX_CAPTURE_TO_PERMIT_NS
                ),
            )
            self.assertFalse(boundary["applied"])
            with self.assertRaisesRegex(
                MODULE.AnchorValidationError, "older than the 300-second"
            ):
                run_tool(
                    args,
                    issued_boottime_ns=(
                        TEST_RECORDED_BOOTTIME_NS
                        + MODULE.MAX_CAPTURE_TO_PERMIT_NS
                        + 1
                    ),
                )

    def test_launch_permit_directory_is_explicit_pending_lifecycle(self):
        with tempfile.TemporaryDirectory() as directory:
            args, _ = make_fixture_tree(Path(directory))
            args.launch_permit_directory = Path(directory) / "not-pending"
            with self.assertRaisesRegex(
                MODULE.AnchorValidationError, "must be the pending"
            ):
                run_tool(args)

    def test_capture_requires_boot_boottime_and_strict_worker_evidence(self):
        mutations = (
            lambda capture: capture.pop("host_boot_id"),
            lambda capture: capture.__setitem__("host_boot_id", "NOT-A-UUID"),
            lambda capture: capture.__setitem__("recorded_boottime_ns", True),
            lambda capture: capture["raw_safety_summary"].pop("worker"),
            lambda capture: capture["raw_safety_summary"]["worker"].__setitem__(
                "sha256", "g" * 64
            ),
            lambda capture: capture["raw_safety_summary"]["worker"].__setitem__(
                "sha256", "F" * 64
            ),
            lambda capture: capture["raw_safety_summary"]["worker"].__setitem__(
                "path", ""
            ),
        )
        for mutate in mutations:
            with self.subTest(mutate=mutate):
                capture = fixture_capture()
                mutate(capture)
                with self.assertRaises(MODULE.AnchorValidationError):
                    MODULE.validate_capture(capture)

    def test_startup_prime_bounds_are_inclusive(self):
        capture = fixture_capture()
        capture["raw_safety_summary"]["startup_brake_prime"][
            "elapsed_ns"
        ] = MODULE.STARTUP_PRIME_MAXIMUM_ELAPSED_NS
        capture["raw_safety_summary"]["worker"]["terminal_proof"][
            "J2_STARTUP_BRAKE_PRIME_ELAPSED_NS"
        ] = str(MODULE.STARTUP_PRIME_MAXIMUM_ELAPSED_NS)
        MODULE.validate_capture(capture)

        capture = fixture_capture()
        prime = capture["raw_safety_summary"]["startup_brake_prime"]
        phase = capture["raw_safety_summary"]["phase_tx_accounting"]
        proof = capture["raw_safety_summary"]["worker"]["terminal_proof"]
        capture["safety"]["startup_invalid_prefix_pairs"] = 0
        prime.update(
            {
                "invalid_prefix_pairs": 0,
                "attempted_pairs": 5,
                "tx_attempt_count": 10,
                "elapsed_ns": 1,
            }
        )
        phase.update(
            {
                "startup_prime_tx_attempt_count": 10,
                "aggregate_tx_attempt_count": 1050,
            }
        )
        proof.update(
            {
                "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS": "0",
                "J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS": "5",
                "J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT": "10",
                "J2_STARTUP_BRAKE_PRIME_ELAPSED_NS": "1",
                "TX_ATTEMPT_TOTAL": "1050",
                "BRAKE_TX_ATTEMPT_COUNT": "1050",
                "SERIAL_SEND_CALL_COUNT": "1050",
            }
        )
        MODULE.validate_capture(capture)

    def test_rejects_forged_startup_prime_or_phase_tx_accounting(self):
        mutations = (
            (
                "missing prime",
                lambda capture: capture["raw_safety_summary"].pop(
                    "startup_brake_prime"
                ),
            ),
            (
                "extra prime field",
                lambda capture: capture["raw_safety_summary"][
                    "startup_brake_prime"
                ].__setitem__("untrusted", 0),
            ),
            (
                "prime pass safety flag",
                lambda capture: capture["safety"].__setitem__(
                    "startup_brake_prime_passed", False
                ),
            ),
            (
                "integer substituted for prime pass boolean",
                lambda capture: capture["safety"].__setitem__(
                    "startup_brake_prime_passed", 1
                ),
            ),
            (
                "integer substituted for command-rx boolean",
                lambda capture: capture["safety"].__setitem__(
                    "command_rx_enabled", 0
                ),
            ),
            (
                "prime state",
                lambda capture: capture["raw_safety_summary"][
                    "startup_brake_prime"
                ].__setitem__("state", "FAIL"),
            ),
            (
                "prime published feedback",
                lambda capture: capture["raw_safety_summary"][
                    "startup_brake_prime"
                ].__setitem__("feedback_published_during_prime", True),
            ),
            (
                "prime window reopened",
                lambda capture: capture["raw_safety_summary"][
                    "startup_brake_prime"
                ].__setitem__("window_reopens_after_first_healthy_pair", True),
            ),
            (
                "maximum prefix contract",
                lambda capture: capture["raw_safety_summary"][
                    "startup_brake_prime"
                ].__setitem__("maximum_invalid_prefix_pairs", 4),
            ),
            (
                "prefix above maximum",
                lambda capture: capture["raw_safety_summary"][
                    "startup_brake_prime"
                ].__setitem__("invalid_prefix_pairs", 4),
            ),
            (
                "required healthy contract",
                lambda capture: capture["raw_safety_summary"][
                    "startup_brake_prime"
                ].__setitem__("required_healthy_pairs", 4),
            ),
            (
                "healthy qualification",
                lambda capture: capture["raw_safety_summary"][
                    "startup_brake_prime"
                ].__setitem__("healthy_pairs", 4),
            ),
            (
                "attempted pair sum",
                lambda capture: capture["raw_safety_summary"][
                    "startup_brake_prime"
                ].__setitem__("attempted_pairs", 9),
            ),
            (
                "prime transmit sum",
                lambda capture: capture["raw_safety_summary"][
                    "startup_brake_prime"
                ].__setitem__("tx_attempt_count", 15),
            ),
            (
                "elapsed lower bound",
                lambda capture: capture["raw_safety_summary"][
                    "startup_brake_prime"
                ].__setitem__("elapsed_ns", 0),
            ),
            (
                "elapsed upper bound",
                lambda capture: capture["raw_safety_summary"][
                    "startup_brake_prime"
                ].__setitem__(
                    "elapsed_ns", MODULE.STARTUP_PRIME_MAXIMUM_ELAPSED_NS + 1
                ),
            ),
            (
                "boolean prime integer",
                lambda capture: capture["raw_safety_summary"][
                    "startup_brake_prime"
                ].__setitem__("attempted_pairs", True),
            ),
            (
                "safety prefix binding",
                lambda capture: capture["safety"].__setitem__(
                    "startup_invalid_prefix_pairs", 2
                ),
            ),
            (
                "missing phase accounting",
                lambda capture: capture["raw_safety_summary"].pop(
                    "phase_tx_accounting"
                ),
            ),
            (
                "extra phase field",
                lambda capture: capture["raw_safety_summary"][
                    "phase_tx_accounting"
                ].__setitem__("untrusted", 0),
            ),
            (
                "completed cycle binding",
                lambda capture: capture["raw_safety_summary"][
                    "phase_tx_accounting"
                ].__setitem__("completed_cycles", 499),
            ),
            (
                "prime phase binding",
                lambda capture: capture["raw_safety_summary"][
                    "phase_tx_accounting"
                ].__setitem__("startup_prime_tx_attempt_count", 15),
            ),
            (
                "control transmit formula",
                lambda capture: capture["raw_safety_summary"][
                    "phase_tx_accounting"
                ].__setitem__("control_loop_tx_attempt_count", 999),
            ),
            (
                "final transmit contract",
                lambda capture: capture["raw_safety_summary"][
                    "phase_tx_accounting"
                ].__setitem__("final_brake_tx_attempt_count", 39),
            ),
            (
                "aggregate strict sum",
                lambda capture: capture["raw_safety_summary"][
                    "phase_tx_accounting"
                ].__setitem__("aggregate_tx_attempt_count", 1055),
            ),
            (
                "boolean phase integer",
                lambda capture: capture["raw_safety_summary"][
                    "phase_tx_accounting"
                ].__setitem__("completed_cycles", True),
            ),
        )
        for label, mutate in mutations:
            with self.subTest(label=label):
                capture = fixture_capture()
                mutate(capture)
                with self.assertRaises(MODULE.AnchorValidationError):
                    MODULE.validate_capture(capture)

    def test_rejects_forged_worker_terminal_proof_and_lifecycle(self):
        worker_mutations = (
            ("missing terminal proof", lambda worker: worker.pop("terminal_proof")),
            ("boolean return code", lambda worker: worker.__setitem__("returncode", False)),
            ("failed return code", lambda worker: worker.__setitem__("returncode", 1)),
            (
                "stop escalation",
                lambda worker: worker.__setitem__("stop_escalation", "terminate"),
            ),
            ("forced kill", lambda worker: worker.__setitem__("forced_kill", True)),
        )
        for label, mutate in worker_mutations:
            with self.subTest(label=label):
                capture = fixture_capture()
                mutate(capture["raw_safety_summary"]["worker"])
                with self.assertRaises(MODULE.AnchorValidationError):
                    MODULE.validate_capture(capture)

        proof_mutations = (
            ("missing proof field", lambda proof: proof.pop("COMPLETED_CYCLES")),
            ("extra proof field", lambda proof: proof.__setitem__("UNTRUSTED", "0")),
            (
                "wrong terminal path",
                lambda proof: proof.__setitem__("TERMINAL_PATH", "ABNORMAL"),
            ),
            (
                "wrong startup maximum",
                lambda proof: proof.__setitem__(
                    "J2_STARTUP_BRAKE_PRIME_MAX_INVALID_PREFIX_PAIRS", "4"
                ),
            ),
            (
                "wrong startup requirement",
                lambda proof: proof.__setitem__(
                    "J2_STARTUP_BRAKE_PRIME_REQUIRED_HEALTHY_PAIRS", "4"
                ),
            ),
            (
                "noncanonical count",
                lambda proof: proof.__setitem__(
                    "J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS", "08"
                ),
            ),
        )
        for label, mutate in proof_mutations:
            with self.subTest(label=label):
                capture = fixture_capture()
                proof = capture["raw_safety_summary"]["worker"]["terminal_proof"]
                mutate(proof)
                with self.assertRaises(MODULE.AnchorValidationError):
                    MODULE.validate_capture(capture)

        bound_count_fields = (
            "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS",
            "J2_STARTUP_BRAKE_PRIME_HEALTHY_PAIRS",
            "J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS",
            "J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT",
            "J2_STARTUP_BRAKE_PRIME_ELAPSED_NS",
            "COMPLETED_CYCLES",
            "FINAL_BRAKE_TX_ATTEMPT_COUNT",
            "TX_ATTEMPT_TOTAL",
            "BRAKE_TX_ATTEMPT_COUNT",
            "SERIAL_SEND_CALL_COUNT",
        )
        for field in bound_count_fields:
            with self.subTest(field=field):
                capture = fixture_capture()
                proof = capture["raw_safety_summary"]["worker"]["terminal_proof"]
                proof[field] = str(int(proof[field], 10) + 1)
                with self.assertRaises(MODULE.AnchorValidationError):
                    MODULE.validate_capture(capture)

        capture = fixture_capture()
        proof = capture["raw_safety_summary"]["worker"]["terminal_proof"]
        for field in (
            "TX_ATTEMPT_TOTAL",
            "BRAKE_TX_ATTEMPT_COUNT",
            "SERIAL_SEND_CALL_COUNT",
        ):
            proof[field] = "1057"
        with self.assertRaises(MODULE.AnchorValidationError):
            MODULE.validate_capture(capture)

    def test_existing_lifecycle_file_blocks_before_anchor_publication(self):
        for lifecycle_field in ("pending_path", "inflight_path", "spent_path"):
            with self.subTest(lifecycle_field=lifecycle_field):
                with tempfile.TemporaryDirectory() as directory:
                    args, blobs = make_fixture_tree(Path(directory))
                    dry_run = run_tool(args)
                    marker = Path(dry_run["launch_permit"][lifecycle_field])
                    marker.parent.mkdir(parents=True)
                    competitor = b"pre-existing lifecycle state\n"
                    marker.write_bytes(competitor)
                    args.apply = True
                    args.confirm = MODULE.APPLY_GATE
                    with self.assertRaises(MODULE.AnchorValidationError):
                        run_tool(args)
                    self.assertEqual(marker.read_bytes(), competitor)
                    self.assertFalse(Path(dry_run["anchor_path"]).exists())
                    for key, path in (
                        ("zero", args.zero_file),
                        ("sidecar", args.zero_sha256_file),
                        ("hints", args.recovery_hint_file),
                        ("initial_pose", args.initial_pose_file),
                        ("capture", args.capture_statistics_file),
                    ):
                        self.assertEqual(path.read_bytes(), blobs[key])

    def test_existing_anchor_alone_blocks_permit_reissuance(self):
        with tempfile.TemporaryDirectory() as directory:
            args, _ = make_fixture_tree(Path(directory))
            dry_run = run_tool(args)
            anchor_path = Path(dry_run["anchor_path"])
            MODULE.atomic_write_new(
                anchor_path, MODULE.json_bytes(dry_run["anchor"])
            )
            args.apply = True
            args.confirm = MODULE.APPLY_GATE
            with self.assertRaises(MODULE.AnchorValidationError):
                run_tool(args)
            self.assertFalse(Path(dry_run["permit_path"]).exists())

    def test_atomic_publication_never_replaces_existing_file(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "pending" / "permit.json"
            MODULE.ensure_private_directory(target.parent)
            target.write_bytes(b"competitor\n")
            with self.assertRaises(MODULE.AnchorValidationError):
                MODULE.atomic_write_new(target, b"replacement\n")
            self.assertEqual(target.read_bytes(), b"competitor\n")

    def test_source_has_no_hardware_or_process_access_modules(self):
        source = MODULE_PATH.read_text(encoding="utf-8")
        for forbidden in (
            "import socket",
            "import serial",
            "import subprocess",
            "/dev/tty",
            "sendRecv(",
        ):
            self.assertNotIn(forbidden, source)


if __name__ == "__main__":
    unittest.main()

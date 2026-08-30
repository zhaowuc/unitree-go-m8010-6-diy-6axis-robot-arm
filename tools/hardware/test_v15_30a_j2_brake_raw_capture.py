#!/usr/bin/env python3

from __future__ import annotations

import argparse
import io
import importlib.util
import json
import math
import tempfile
import unittest
from contextlib import redirect_stdout
from datetime import datetime, timezone
from pathlib import Path
from unittest import mock


SCRIPT = Path(__file__).with_name("v15_30a_j2_brake_raw_capture.py")
SPEC = importlib.util.spec_from_file_location("j2_brake_raw_capture", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

ANCHOR_SCRIPT = Path(__file__).with_name(
    "v15_30a_create_j2_vertical_session_phase_anchor.py"
)
ANCHOR_SPEC = importlib.util.spec_from_file_location(
    "j2_vertical_session_phase_anchor_contract", ANCHOR_SCRIPT
)
assert ANCHOR_SPEC is not None and ANCHOR_SPEC.loader is not None
ANCHOR = importlib.util.module_from_spec(ANCHOR_SPEC)
ANCHOR_SPEC.loader.exec_module(ANCHOR)

TEST_HOST_BOOT_ID = "01234567-89ab-4def-8123-456789abcdef"
TEST_RECORDED_BOOTTIME_NS = 123_456_789_012
TEST_POSE_BINDING_ID = "0123456789abcdef" * 4


def packet(index: int, *, raw_a: float = 5.04623, raw_b: float = 2.76539) -> dict:
    wobble = 1.0e-5 * math.sin(index / 17.0)
    samples = [
        {
            "motor": "J2A",
            "position_rad": raw_a + wobble,
            "velocity_rad_s": 0.0,
            "temperature_c": 32.0,
            "merror": 0,
            "communication_ok": True,
            "unwrapped_raw_position_rad": raw_a + wobble,
            # Deliberately omit software-zero and recovery-branch fields.  Raw
            # capture must not depend on either old coordinate contract.
        },
        {
            "motor": "J2B",
            "position_rad": raw_b - wobble,
            "velocity_rad_s": 0.0,
            "temperature_c": 33.0,
            "merror": 0,
            "communication_ok": True,
            "unwrapped_raw_position_rad": raw_b - wobble,
        },
    ]
    return {
        "schema": "go-m8010-motor-feedback/1.0",
        "source_monotonic_ns": 1_000_000_000 + index * 10_000_000,
        "samples": samples,
        "controller_mode": "brake",
        "controller_mode_by_motor": {"J2A": "brake", "J2B": "brake"},
        "j2_sync_fault": False,
        "domain_fault": False,
        "lease_safe_hold": False,
    }


def capture_metadata(count: int) -> tuple[list[int], list[tuple[str, int]]]:
    received = [1_001_000_000 + index * 10_000_000 for index in range(count)]
    sources = [("127.0.0.1", 49152)] * count
    return received, sources


def analyze(packets: list[dict]) -> dict:
    received, sources = capture_metadata(len(packets))
    return MODULE.analyze_raw_packets(packets, received, sources)


def terminal_proof(**overrides: str) -> str:
    fields = {
        "TERMINAL_PATH": "NORMAL",
        "GUI_GO_CONTROLLER_BUS": "j2",
        "EXECUTION_POLICY": "BRAKE_ONLY",
        "COMMAND_RX_ENABLED": "NO",
        "J2_SESSION_REFERENCE_CONFIGURED": "NO",
        "COMPLETED_CYCLES": "500",
        "TX_ATTEMPT_TOTAL": "1056",
        "BRAKE_TX_ATTEMPT_COUNT": "1056",
        "FOC_TX_ATTEMPT_COUNT": "0",
        "OTHER_MODE_TX_ATTEMPT_COUNT": "0",
        "BRAKE_ONLY_GUARD_BLOCK_COUNT": "0",
        "SERIAL_SEND_CALL_COUNT": "1056",
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
        "FINAL_BRAKE_TX_ATTEMPT_COUNT": "40",
    }
    fields.update(overrides)
    return "\n".join(f"{key}={value}" for key, value in fields.items()) + "\n"


def pass_document(**proof_overrides: str) -> dict:
    statistics = analyze([packet(index) for index in range(500)])
    identity = MODULE.new_capture_identity(
        now=datetime(2026, 8, 28, 1, 2, 3, tzinfo=timezone.utc),
        token="0123456789abcdef0123456789abcdef",
        host_boot_id=TEST_HOST_BOOT_ID,
        recorded_boottime_ns=TEST_RECORDED_BOOTTIME_NS,
    )
    proof_text = terminal_proof(**proof_overrides)
    proof = MODULE.brake_verify.parse_worker_terminal_proof(proof_text)
    return MODULE.build_capture_document(
        pose_binding_id=TEST_POSE_BINDING_ID,
        identity=identity,
        statistics=statistics,
        worker_path=Path("/tmp/hash-pinned-worker"),
        worker_sha256="a" * 64,
        worker_returncode=0,
        stop_escalation="none",
        terminal_proof=proof,
        stdout=proof_text,
        stderr="",
        feedback_port=15300,
    )


class J2BrakeRawCaptureTests(unittest.TestCase):
    def test_pass_document_is_accepted_by_phase_anchor_contract(self):
        document = pass_document()
        normalized = ANCHOR.validate_capture(document)
        self.assertEqual(document["schema"], MODULE.CAPTURE_SCHEMA)
        self.assertEqual(normalized["packet_count"], 500)
        self.assertEqual(set(normalized["motors"]), {"J2A", "J2B"})
        self.assertAlmostEqual(
            document["motors"]["J2A"]["unwrapped_raw_position_rad"]["mean"],
            5.04623,
            places=6,
        )
        self.assertEqual(document["safety"]["all_controller_modes"], ["brake"])
        self.assertEqual(document["safety"]["foc_tx_attempt_count"], 0)
        self.assertEqual(document["safety"]["foc_serial_send_call_count"], 0)
        self.assertEqual(document["safety"]["other_mode_tx_attempt_count"], 0)
        self.assertEqual(document["safety"]["active_or_hold_commands_sent"], 0)
        self.assertEqual(document["safety"]["active_commands_sent"], 0)
        self.assertEqual(document["safety"]["hold_commands_sent"], 0)
        self.assertEqual(document["safety"]["motor_internal_zero_write_count"], 0)
        self.assertEqual(document["safety"]["rid_write_count"], 0)
        self.assertEqual(document["safety"]["flash_or_eeprom_write_count"], 0)
        self.assertTrue(document["safety"]["startup_brake_prime_passed"])
        self.assertEqual(document["safety"]["startup_invalid_prefix_pairs"], 3)
        self.assertEqual(
            document["raw_safety_summary"]["startup_brake_prime"][
                "healthy_pairs"
            ],
            5,
        )
        self.assertFalse(
            document["raw_safety_summary"]["startup_brake_prime"][
                "feedback_published_during_prime"
            ]
        )
        self.assertEqual(document["host_boot_id"], TEST_HOST_BOOT_ID)
        self.assertEqual(
            document["recorded_boottime_ns"], TEST_RECORDED_BOOTTIME_NS
        )

    def test_boot_identity_validation_and_clock_injection_are_fail_closed(self):
        self.assertEqual(
            MODULE.validate_host_boot_id(TEST_HOST_BOOT_ID), TEST_HOST_BOOT_ID
        )
        for invalid in (
            TEST_HOST_BOOT_ID.upper(),
            TEST_HOST_BOOT_ID.replace("-", ""),
            "00000000-0000-0000-0000-000000000000",
            "not-a-uuid",
            None,
        ):
            with self.assertRaises(MODULE.CaptureError):
                MODULE.validate_host_boot_id(invalid)
        for invalid in (0, -1, True, 1.0, "1", None):
            with self.assertRaises(MODULE.CaptureError):
                MODULE.validate_recorded_boottime_ns(invalid)

        observed_clock_ids = []
        value = MODULE.read_recorded_boottime_ns(
            clock_gettime_ns=lambda clock_id: (
                observed_clock_ids.append(clock_id) or TEST_RECORDED_BOOTTIME_NS
            ),
            clock_id=777,
        )
        self.assertEqual(value, TEST_RECORDED_BOOTTIME_NS)
        self.assertEqual(observed_clock_ids, [777])

        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "boot_id"
            path.write_text(TEST_HOST_BOOT_ID + "\n", encoding="ascii")
            self.assertEqual(MODULE.read_host_boot_id(path), TEST_HOST_BOOT_ID)
            path.write_text(TEST_HOST_BOOT_ID.upper(), encoding="ascii")
            with self.assertRaises(MODULE.CaptureError):
                MODULE.read_host_boot_id(path)
        self.assertEqual(
            MODULE.HOST_BOOT_ID_PATH,
            Path("/proc/sys/kernel/random/boot_id"),
        )

    def test_pass_and_failure_documents_preserve_same_boot_identity(self):
        passed = pass_document()
        identity = MODULE.new_capture_identity(
            now=datetime(2026, 8, 28, 1, 2, 3, tzinfo=timezone.utc),
            token="fedcba9876543210fedcba9876543210",
            host_boot_id=TEST_HOST_BOOT_ID,
            recorded_boottime_ns=TEST_RECORDED_BOOTTIME_NS,
        )
        failed = MODULE.failure_document(
            pose_binding_id=TEST_POSE_BINDING_ID,
            identity=identity,
            worker_path=Path("/tmp/hash-pinned-worker"),
            worker_sha256="a" * 64,
            process=None,
            stop_escalation="none",
            stdout="",
            stderr="",
            feedback_port=15300,
            packet_count=0,
            failures=["synthetic failure"],
        )
        for document in (passed, failed):
            self.assertEqual(document["pose_binding_id"], TEST_POSE_BINDING_ID)
            self.assertEqual(document["host_boot_id"], TEST_HOST_BOOT_ID)
            self.assertEqual(
                document["recorded_boottime_ns"], TEST_RECORDED_BOOTTIME_NS
            )
        self.assertEqual(failed["status"], "FAIL")

    def test_raw_analysis_does_not_require_old_zero_or_branch_fields(self):
        result = analyze([packet(index) for index in range(500)])
        self.assertEqual(result["packet_count"], 500)
        self.assertGreaterEqual(result["source_coverage_s"], 4.0)
        self.assertLess(
            result["motors"]["J2A"]["unwrapped_raw_position_rad"]["span"],
            MODULE.MAXIMUM_RAW_SPAN_RAD,
        )
        self.assertEqual(result["communication_failure_packets"], 0)
        self.assertEqual(result["merror_nonzero_packets"], 0)

    def test_rejects_too_few_frames_or_short_coverage(self):
        with self.assertRaises(MODULE.CaptureError):
            analyze([packet(index) for index in range(499)])
        packets = [packet(index) for index in range(500)]
        received = [1_001_000_000 + index * 1_000_000 for index in range(500)]
        sources = [("127.0.0.1", 49152)] * 500
        with self.assertRaises(MODULE.CaptureError):
            MODULE.analyze_raw_packets(packets, received, sources)

    def test_rejects_non_brake_fault_communication_and_merror(self):
        packets = [packet(index) for index in range(500)]
        packets[10]["controller_mode"] = "hold"
        with self.assertRaises(MODULE.CaptureError):
            analyze(packets)
        packets = [packet(index) for index in range(500)]
        packets[10]["domain_fault"] = True
        with self.assertRaises(MODULE.CaptureError):
            analyze(packets)
        packets = [packet(index) for index in range(500)]
        packets[10]["samples"][0]["communication_ok"] = False
        with self.assertRaises(MODULE.CaptureError):
            analyze(packets)
        packets = [packet(index) for index in range(500)]
        packets[10]["samples"][1]["merror"] = 1
        with self.assertRaises(MODULE.CaptureError):
            analyze(packets)

    def test_rejects_motion_stale_time_and_changed_source(self):
        packets = [
            packet(index, raw_a=5.04623 + index * 0.0001)
            for index in range(500)
        ]
        with self.assertRaises(MODULE.CaptureError):
            analyze(packets)
        packets = [packet(index) for index in range(500)]
        received, sources = capture_metadata(500)
        received[100] = packets[100]["source_monotonic_ns"] + 300_000_000
        with self.assertRaises(MODULE.CaptureError):
            MODULE.analyze_raw_packets(packets, received, sources)

    def test_live_raw_protocol_range_matches_anchor_consumer(self):
        packets = [packet(index, raw_a=13.0) for index in range(500)]
        result = analyze(packets)
        self.assertAlmostEqual(
            result["motors"]["J2A"]["unwrapped_raw_position_rad"]["mean"],
            13.0,
            places=5,
        )
        packets = [
            packet(index, raw_a=MODULE.MAXIMUM_RAW_POSITION_RAD + 1.0)
            for index in range(500)
        ]
        with self.assertRaisesRegex(MODULE.CaptureError, "protocol range"):
            analyze(packets)
        received, sources = capture_metadata(500)
        sources[100] = ("127.0.0.1", 49153)
        with self.assertRaises(MODULE.CaptureError):
            MODULE.analyze_raw_packets(packets, received, sources)

    def test_worker_command_is_brake_only_and_has_no_reference_inputs(self):
        args = argparse.Namespace(
            worker=Path("/tmp/worker"),
            feedback_port=15300,
            thermal_config=Path("/tmp/thermal_limits.yaml"),
            expected_thermal_config_sha256="a" * 64,
        )
        command = MODULE.build_worker_command(args)
        self.assertIn("--brake-only", command)
        self.assertEqual(command[command.index("--bus") + 1], "j2")
        self.assertEqual(
            command[command.index("--thermal-config") + 1],
            str(Path("/tmp/thermal_limits.yaml")),
        )
        self.assertEqual(
            command[command.index("--expected-thermal-config-sha256") + 1],
            "a" * 64,
        )
        for forbidden in (
            "--zero-file",
            "--recovery-hint-file",
            "--command-port",
            "hold",
            "position",
        ):
            self.assertNotIn(forbidden, command)

    def test_complete_software_and_physical_gates_are_mandatory(self):
        MODULE.validate_gates(MODULE.CONFIRM_GATE, MODULE.PHYSICAL_GATE)
        self.assertIn("J2_VERTICAL_INITIALIZATION_POSE=YES", MODULE.PHYSICAL_GATE)
        self.assertIn("ARM_NOT_MOVED=YES", MODULE.PHYSICAL_GATE)
        with self.assertRaises(MODULE.CaptureError):
            MODULE.validate_gates("", MODULE.PHYSICAL_GATE)
        with self.assertRaises(MODULE.CaptureError):
            MODULE.validate_gates(MODULE.CONFIRM_GATE, "24V_ON=YES")

    def test_terminal_proof_and_raw_summary_preserve_zero_foc_evidence(self):
        document = pass_document()
        worker = document["raw_safety_summary"]["worker"]
        self.assertEqual(worker["terminal_proof"]["FOC_TX_ATTEMPT_COUNT"], "0")
        self.assertEqual(worker["terminal_proof"]["FINAL_BRAKE"], "PASS")
        self.assertEqual(
            worker["terminal_proof"]["J2_SESSION_REFERENCE_CONFIGURED"], "NO"
        )
        self.assertFalse(document["raw_safety_summary"]["branch_inputs_loaded"])
        self.assertEqual(document["raw_safety_summary"]["udp_send_calls"], 0)
        with self.assertRaises(MODULE.brake_verify.VerificationError):
            MODULE.brake_verify.parse_worker_terminal_proof(
                terminal_proof(FOC_TX_ATTEMPT_COUNT="1")
            )
        with self.assertRaises(MODULE.CaptureError):
            pass_document(J2_SESSION_REFERENCE_CONFIGURED="YES")
        with self.assertRaises(MODULE.brake_verify.VerificationError):
            pass_document(J2_STARTUP_BRAKE_PRIME_STATE="FAIL")
        with self.assertRaises(MODULE.brake_verify.VerificationError):
            pass_document(J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS="4")
        zero_prefix = pass_document(
            J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS="0",
            J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS="5",
            J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT="10",
            J2_STARTUP_BRAKE_PRIME_ELAPSED_NS="40000000",
            TX_ATTEMPT_TOTAL="1050",
            BRAKE_TX_ATTEMPT_COUNT="1050",
            SERIAL_SEND_CALL_COUNT="1050",
        )
        self.assertEqual(
            zero_prefix["safety"]["startup_invalid_prefix_pairs"], 0
        )
        with self.assertRaises(MODULE.CaptureError):
            pass_document(TX_ATTEMPT_TOTAL="1000", BRAKE_TX_ATTEMPT_COUNT="1000", SERIAL_SEND_CALL_COUNT="1000")

    def test_power_session_ids_are_unique_and_capture_output_never_overwrites(self):
        first = MODULE.new_capture_identity(
            host_boot_id=TEST_HOST_BOOT_ID,
            recorded_boottime_ns=TEST_RECORDED_BOOTTIME_NS,
        )
        second = MODULE.new_capture_identity(
            host_boot_id=TEST_HOST_BOOT_ID,
            recorded_boottime_ns=TEST_RECORDED_BOOTTIME_NS,
        )
        self.assertNotEqual(first["power_session_id"], second["power_session_id"])
        self.assertNotEqual(first["capture_id"], second["capture_id"])
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "capture.json"
            document = pass_document()
            MODULE.publish_json_no_overwrite(path, document)
            with self.assertRaises(FileExistsError):
                MODULE.publish_json_no_overwrite(path, pass_document())
            self.assertEqual(json.loads(path.read_text(encoding="utf-8")), document)

    def test_unhandled_post_start_error_reports_hardware_and_power_off(self):
        def fail_after_worker_start(_args):
            MODULE.RUNTIME_HARDWARE_ACCESSED = True
            raise OSError("synthetic post-start failure")

        output = io.StringIO()
        parsed = argparse.Namespace(pose_binding_id=TEST_POSE_BINDING_ID)
        with mock.patch.object(MODULE, "parse_args", return_value=parsed), mock.patch.object(
            MODULE, "run", side_effect=fail_after_worker_start
        ), redirect_stdout(output):
            returncode = MODULE.main()
        report = json.loads(output.getvalue())
        self.assertEqual(returncode, 4)
        self.assertTrue(report["hardware_accessed"])
        self.assertTrue(report["physical_power_off_required"])
        self.assertEqual(report["pose_binding_id"], TEST_POSE_BINDING_ID)

    def test_pose_binding_id_is_required_and_strictly_lowercase_hex(self):
        self.assertEqual(
            MODULE.validate_pose_binding_id(TEST_POSE_BINDING_ID),
            TEST_POSE_BINDING_ID,
        )
        for invalid in (
            TEST_POSE_BINDING_ID.upper(),
            f" {TEST_POSE_BINDING_ID}",
            f"{TEST_POSE_BINDING_ID} ",
            TEST_POSE_BINDING_ID[:-1],
            TEST_POSE_BINDING_ID + "0",
            "g" * 64,
            "",
            None,
        ):
            with self.assertRaises(ValueError):
                MODULE.validate_pose_binding_id(invalid)

        base = [
            "--worker", "/tmp/worker",
            "--expected-worker-sha256", "a" * 64,
            "--thermal-config", "/tmp/thermal_limits.yaml",
            "--expected-thermal-config-sha256", "b" * 64,
            "--output", "/tmp/out.json",
            "--confirm", MODULE.CONFIRM_GATE,
            "--physical-confirmation", MODULE.PHYSICAL_GATE,
        ]
        with self.assertRaises(SystemExit):
            MODULE.parse_args(base)
        for invalid in (
            TEST_POSE_BINDING_ID.upper(),
            f" {TEST_POSE_BINDING_ID}",
            TEST_POSE_BINDING_ID[:-1],
            "g" * 64,
        ):
            with self.assertRaises(SystemExit):
                MODULE.parse_args(base + ["--pose-binding-id", invalid])
        parsed = MODULE.parse_args(
            base + ["--pose-binding-id", TEST_POSE_BINDING_ID]
        )
        self.assertEqual(parsed.pose_binding_id, TEST_POSE_BINDING_ID)

    def test_cli_requires_at_least_500_target_frames(self):
        base = [
            "--worker",
            "/tmp/worker",
            "--expected-worker-sha256",
            "a" * 64,
            "--thermal-config",
            "/tmp/thermal_limits.yaml",
            "--expected-thermal-config-sha256",
            "b" * 64,
            "--target-packets",
            "499",
            "--output",
            "/tmp/out.json",
            "--confirm",
            MODULE.CONFIRM_GATE,
            "--physical-confirmation",
            MODULE.PHYSICAL_GATE,
            "--pose-binding-id",
            TEST_POSE_BINDING_ID,
        ]
        with self.assertRaises(SystemExit):
            MODULE.parse_args(base)


if __name__ == "__main__":
    unittest.main()

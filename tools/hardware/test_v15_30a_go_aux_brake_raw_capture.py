#!/usr/bin/env python3

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import io
import json
import tempfile
import unittest
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from unittest import mock


SCRIPT = Path(__file__).with_name("v15_30a_go_aux_brake_raw_capture.py")
SPEC = importlib.util.spec_from_file_location("go_aux_brake_raw_capture", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)

TEST_POSE_BINDING_ID = "fedcba9876543210" * 4
TEST_HOST_BOOT_ID = "01234567-89ab-4def-8123-456789abcdef"


def domain(bus: str) -> dict:
    names = MODULE.BUS_MOTORS[bus]
    return {
        "packet_count": 500,
        "source_coverage_s": 4.99,
        "local_receive_coverage_s": 4.99,
        "motor_names": list(names),
        "motors": {
            name: {
                "sample_count": 500,
                "unwrapped_raw_position_rad": {
                    "mean": 0.1,
                    "minimum": 0.09999,
                    "maximum": 0.10001,
                    "span": 0.00002,
                    "standard_deviation": 0.000001,
                },
            }
            for name in names
        },
        "temperature_min_c": {name: 30.0 for name in names},
        "temperature_max_c": {name: 31.0 for name in names},
        "feedback_source_endpoint": "127.0.0.1:49152",
        "timing": {
            "hard_gap_limit_ns": MODULE.MAX_GAP_NS,
            "hard_feedback_age_limit_ns": MODULE.MAX_FEEDBACK_AGE_NS,
        },
        "terminal_proof": {"EXECUTION_POLICY": "BRAKE_ONLY"},
        "stdout": ["EXECUTION_POLICY=BRAKE_ONLY"],
        "stderr": [],
    }


def packet_series(
    bus: str,
) -> tuple[list[dict], list[int], list[tuple[str, int]]]:
    names = MODULE.BUS_MOTORS[bus]
    source_stamps = [1_000_000_000 + index * 10_000_000 for index in range(500)]
    packets = [
        {
            "schema": "go-m8010-motor-feedback/1.0",
            "source_monotonic_ns": stamp,
            "controller_mode": "brake",
            "controller_mode_by_motor": {name: "brake" for name in names},
            "j2_sync_fault": False,
            "domain_fault": False,
            "lease_safe_hold": False,
            "samples": [
                {
                    "motor": name,
                    "communication_ok": True,
                    "merror": 0,
                    "temperature_c": 30.0,
                    "unwrapped_raw_position_rad": 0.1 + index * 1.0e-8,
                }
                for name in names
            ],
        }
        for index, stamp in enumerate(source_stamps)
    ]
    received = [stamp + 1_000_000 for stamp in source_stamps]
    endpoints = [("127.0.0.1", 49152)] * len(packets)
    return packets, received, endpoints


def shift_stamps_after(values: list[int], index: int, amount_ns: int) -> None:
    for position in range(index, len(values)):
        values[position] += amount_ns


def terminal_stdout(bus: str) -> str:
    return "\n".join(
        (
            "TERMINAL_PATH=NORMAL",
            f"GUI_GO_CONTROLLER_BUS={bus}",
            "EXECUTION_POLICY=BRAKE_ONLY",
            "COMMAND_RX_ENABLED=NO",
            "COMPLETED_CYCLES=500",
            "TX_ATTEMPT_TOTAL=1563",
            "BRAKE_TX_ATTEMPT_COUNT=1563",
            "FOC_TX_ATTEMPT_COUNT=0",
            "OTHER_MODE_TX_ATTEMPT_COUNT=0",
            "BRAKE_ONLY_GUARD_BLOCK_COUNT=0",
            "SERIAL_SEND_CALL_COUNT=1563",
            "FOC_SERIAL_SEND_CALL_COUNT=0",
            "BRAKE_ONLY_AUDIT=PASS",
            "FINAL_MODE=BRAKE",
            "FINAL_BRAKE=PASS",
            "MOTOR_INTERNAL_ZERO_WRITE=NO",
        )
    ) + "\n"


class FakeListener:
    def __init__(self, datagrams: list[tuple[bytes, tuple[str, int]]]) -> None:
        self.datagrams = list(datagrams)
        self.closed = False

    def setsockopt(self, *_args) -> None:
        pass

    def bind(self, *_args) -> None:
        pass

    def settimeout(self, *_args) -> None:
        pass

    def recvfrom(self, _maximum: int):
        return self.datagrams.pop(0)

    def close(self) -> None:
        self.closed = True


class FakeProcess:
    def __init__(self) -> None:
        self.returncode = 0

    def poll(self):
        return None


class GoAuxBrakeRawCaptureTests(unittest.TestCase):
    def cli_base(self) -> list[str]:
        return [
            "--worker", "/tmp/worker",
            "--expected-worker-sha256", "a" * 64,
            "--output", "/tmp/out.json",
            "--confirm", MODULE.CONFIRM_GATE,
            "--physical-confirmation", MODULE.PHYSICAL_GATE,
        ]

    def test_cli_requires_exact_lowercase_pose_binding_id(self):
        with self.assertRaises(SystemExit):
            MODULE.parse_args(self.cli_base())
        for invalid in (
            TEST_POSE_BINDING_ID.upper(),
            f" {TEST_POSE_BINDING_ID}",
            f"{TEST_POSE_BINDING_ID} ",
            TEST_POSE_BINDING_ID[:-1],
            TEST_POSE_BINDING_ID + "0",
            "g" * 64,
        ):
            with self.assertRaises(SystemExit):
                MODULE.parse_args(
                    self.cli_base() + ["--pose-binding-id", invalid]
                )
        args = MODULE.parse_args(
            self.cli_base() + ["--pose-binding-id", TEST_POSE_BINDING_ID]
        )
        self.assertEqual(args.pose_binding_id, TEST_POSE_BINDING_ID)

    def test_pass_evidence_preserves_pose_binding_id(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            worker = root / "worker"
            worker.write_bytes(b"offline fixture")
            output = root / "capture.json"
            args = argparse.Namespace(
                worker=worker,
                expected_worker_sha256=hashlib.sha256(worker.read_bytes()).hexdigest(),
                feedback_port=15300,
                target_packets=500,
                maximum_runtime_s=8.0,
                output=output,
                confirm=MODULE.CONFIRM_GATE,
                physical_confirmation=MODULE.PHYSICAL_GATE,
                pose_binding_id=TEST_POSE_BINDING_ID,
            )
            with mock.patch.object(
                MODULE.raw_base, "read_host_boot_id", return_value=TEST_HOST_BOOT_ID
            ), mock.patch.object(
                MODULE.raw_base,
                "read_recorded_boottime_ns",
                return_value=123_456_789,
            ), mock.patch.object(
                MODULE, "_capture_domain", side_effect=lambda **kw: domain(kw["bus"])
            ):
                self.assertEqual(MODULE.run(args), 0)
            document = json.loads(output.read_text(encoding="utf-8"))
            self.assertEqual(document["status"], "PASS")
            self.assertEqual(document["pose_binding_id"], TEST_POSE_BINDING_ID)
            self.assertEqual(
                document["domains"]["j345"]["timing"]["hard_gap_limit_ns"],
                MODULE.MAX_GAP_NS,
            )

    def test_go_q15_multi_turn_value_is_not_dm_pmax_limited(self):
        statistics = MODULE._stats([13.9655] * 500, "J1")
        self.assertAlmostEqual(statistics["mean"], 13.9655)
        with self.assertRaisesRegex(MODULE.CaptureError, "protocol range"):
            MODULE._stats(
                [MODULE.raw_base.MAXIMUM_RAW_POSITION_RAD + 1.0] * 500,
                "J1",
            )

    def test_blocked_evidence_preserves_pose_binding_id(self):
        args = argparse.Namespace(pose_binding_id=TEST_POSE_BINDING_ID)
        output = io.StringIO()
        with mock.patch.object(MODULE, "parse_args", return_value=args), mock.patch.object(
            MODULE, "run", side_effect=MODULE.CaptureError("synthetic block")
        ), redirect_stdout(output):
            returncode = MODULE.main()
        document = json.loads(output.getvalue())
        self.assertEqual(returncode, 4)
        self.assertEqual(document["status"], "BLOCKED")
        self.assertEqual(document["pose_binding_id"], TEST_POSE_BINDING_ID)

    def test_blocked_evidence_preserves_domain_worker_diagnostic(self):
        args = argparse.Namespace(pose_binding_id=TEST_POSE_BINDING_ID)
        diagnostic = {
            "bus": "j345",
            "packet_count_before_stop": 417,
            "target_packets": 500,
            "worker_returncode": 2,
            "stop_escalation": "none",
            "forced_kill": False,
            "stdout": ["FINAL_BRAKE=FAIL"],
            "stderr": ["synthetic terminal failure"],
        }
        output = io.StringIO()
        error = MODULE.DomainWorkerError("synthetic worker block", diagnostic)
        with mock.patch.object(MODULE, "parse_args", return_value=args), mock.patch.object(
            MODULE, "run", side_effect=error
        ), redirect_stdout(output):
            returncode = MODULE.main()
        document = json.loads(output.getvalue())
        self.assertEqual(returncode, 4)
        self.assertEqual(document["status"], "BLOCKED")
        self.assertEqual(document["domain_worker_diagnostic"], diagnostic)

    def analyze(self, bus: str, packets, received, endpoints):
        return MODULE.analyze_domain_packets(
            bus=bus,
            packets=packets,
            receive_ns=received,
            endpoints=endpoints,
            target_packets=500,
        )

    def test_nominal_timing_passes_for_both_go_domains(self):
        for bus in ("j1", "j345"):
            with self.subTest(bus=bus):
                packets, received, endpoints = packet_series(bus)
                result = self.analyze(bus, packets, received, endpoints)
                self.assertEqual(result["packet_count"], 500)
                self.assertEqual(result["timing"]["maximum_local_gap_ns"], 10_000_000)
                self.assertEqual(result["timing"]["maximum_source_gap_ns"], 10_000_000)
                self.assertEqual(result["timing"]["feedback_age_max_ns"], 1_000_000)
                self.assertFalse(
                    result["timing"]["collector_scheduler_jitter_candidate"]
                )

    def test_150ms_local_only_gap_is_diagnostic_not_hard_failure(self):
        packets, received, endpoints = packet_series("j345")
        shift_stamps_after(received, 100, 140_000_000)
        result = self.analyze("j345", packets, received, endpoints)
        timing = result["timing"]
        self.assertEqual(timing["maximum_local_gap_ns"], 150_000_000)
        self.assertEqual(timing["maximum_source_gap_ns"], 10_000_000)
        self.assertEqual(timing["local_gap_over_diagnostic_count"], 1)
        self.assertTrue(timing["collector_scheduler_jitter_candidate"])

    def test_150ms_source_and_local_gap_is_recorded_as_source_stall(self):
        packets, received, endpoints = packet_series("j345")
        for packet in packets[100:]:
            packet["source_monotonic_ns"] += 140_000_000
        shift_stamps_after(received, 100, 140_000_000)
        result = self.analyze("j345", packets, received, endpoints)
        timing = result["timing"]
        self.assertEqual(timing["maximum_local_gap_ns"], 150_000_000)
        self.assertEqual(timing["maximum_source_gap_ns"], 150_000_000)
        self.assertTrue(timing["source_stall_candidate"])

    def test_exact_250ms_gap_passes_and_one_nanosecond_more_blocks(self):
        for extra_ns, should_pass in ((240_000_000, True), (240_000_001, False)):
            with self.subTest(extra_ns=extra_ns):
                packets, received, endpoints = packet_series("j345")
                for packet in packets[100:]:
                    packet["source_monotonic_ns"] += extra_ns
                shift_stamps_after(received, 100, extra_ns)
                if should_pass:
                    result = self.analyze("j345", packets, received, endpoints)
                    self.assertEqual(
                        result["timing"]["maximum_source_gap_ns"],
                        MODULE.MAX_GAP_NS,
                    )
                else:
                    with self.assertRaisesRegex(
                        MODULE.CaptureError, "local feedback gap.*exceeds"
                    ):
                        self.analyze("j345", packets, received, endpoints)

    def test_feedback_age_over_250ms_blocks_before_gap_result(self):
        packets, received, endpoints = packet_series("j345")
        shift_stamps_after(received, 100, 249_000_001)
        with self.assertRaisesRegex(MODULE.CaptureError, "feedback age.*exceeds"):
            self.analyze("j345", packets, received, endpoints)

    def test_feedback_age_exact_250ms_passes_and_one_nanosecond_more_blocks(self):
        for age_ns, should_pass in (
            (MODULE.MAX_FEEDBACK_AGE_NS, True),
            (MODULE.MAX_FEEDBACK_AGE_NS + 1, False),
        ):
            with self.subTest(age_ns=age_ns):
                packets, received, endpoints = packet_series("j345")
                for index, packet in enumerate(packets):
                    received[index] = packet["source_monotonic_ns"] + age_ns
                if should_pass:
                    result = self.analyze("j345", packets, received, endpoints)
                    self.assertEqual(
                        result["timing"]["feedback_age_max_ns"], age_ns
                    )
                else:
                    with self.assertRaisesRegex(
                        MODULE.CaptureError, "feedback age.*exceeds"
                    ):
                        self.analyze("j345", packets, received, endpoints)

    def test_future_and_non_increasing_source_timestamps_block(self):
        packets, received, endpoints = packet_series("j345")
        received[100] = packets[100]["source_monotonic_ns"] - 1
        with self.assertRaisesRegex(MODULE.CaptureError, "future-dated"):
            self.analyze("j345", packets, received, endpoints)

        packets, received, endpoints = packet_series("j345")
        packets[100]["source_monotonic_ns"] = packets[99]["source_monotonic_ns"]
        with self.assertRaisesRegex(MODULE.CaptureError, "not increasing"):
            self.analyze("j345", packets, received, endpoints)

    def test_motor_communication_failure_precedes_timing_acceptance(self):
        packets, received, endpoints = packet_series("j345")
        for packet in packets[100:]:
            packet["source_monotonic_ns"] += 110_000_000
        shift_stamps_after(received, 100, 110_000_000)
        packets[100]["samples"][1]["communication_ok"] = False
        with self.assertRaisesRegex(
            MODULE.CaptureError, "j345 packet 100 J4 communication failed"
        ):
            self.analyze("j345", packets, received, endpoints)

    def test_domain_fault_precedes_timing_acceptance(self):
        packets, received, endpoints = packet_series("j345")
        for packet in packets[100:]:
            packet["source_monotonic_ns"] += 110_000_000
        shift_stamps_after(received, 100, 110_000_000)
        packets[100]["domain_fault"] = True
        with self.assertRaisesRegex(
            MODULE.CaptureError, "j345 packet 100 left safe BRAKE state"
        ):
            self.analyze("j345", packets, received, endpoints)

    def test_validation_diagnostic_preserves_timing_health_and_terminal_proof(self):
        packets, received, _endpoints = packet_series("j345")
        for packet in packets[100:]:
            packet["source_monotonic_ns"] += 110_000_000
        shift_stamps_after(received, 100, 110_000_000)
        packets[100]["samples"][2]["communication_ok"] = False
        proof = {"FINAL_BRAKE": "PASS", "COMPLETED_CYCLES": "500"}
        diagnostic = MODULE._domain_validation_diagnostic(
            bus="j345",
            packets=packets,
            receive_ns=received,
            target_packets=500,
            process=SimpleNamespace(returncode=0),
            escalation="none",
            stdout="TX_ATTEMPT_TOTAL=1563\nFINAL_BRAKE=PASS\n",
            stderr="synthetic stderr",
            terminal_proof=proof,
            reason="synthetic validation failure",
        )
        self.assertEqual(diagnostic["bus"], "j345")
        self.assertEqual(diagnostic["timing"]["maximum_local_gap_ns"], 120_000_000)
        self.assertEqual(diagnostic["timing"]["maximum_source_gap_ns"], 120_000_000)
        self.assertEqual(diagnostic["packet_health"]["unhealthy_packet_count"], 1)
        self.assertEqual(
            diagnostic["packet_health"]["first_unhealthy_packets"][0][
                "failed_motors"
            ],
            ["J5"],
        )
        self.assertEqual(diagnostic["terminal_proof"], proof)
        self.assertIn("TX_ATTEMPT_TOTAL=1563", diagnostic["stdout"])
        self.assertEqual(diagnostic["stderr"], ["synthetic stderr"])

    def test_malformed_packets_and_strict_schema_values_fail_closed(self):
        packets, received, endpoints = packet_series("j345")
        packets[10] = None
        with self.assertRaisesRegex(MODULE.CaptureError, "not an object"):
            self.analyze("j345", packets, received, endpoints)
        health = MODULE._health_diagnostic("j345", packets)
        self.assertEqual(health["unhealthy_packet_count"], 1)
        self.assertEqual(
            health["first_unhealthy_packets"][0]["issues"],
            ["PACKET_STRUCTURE"],
        )

        packets, received, endpoints = packet_series("j345")
        packets[10]["samples"][0]["merror"] = False
        with self.assertRaisesRegex(MODULE.CaptureError, "merror is nonzero"):
            self.analyze("j345", packets, received, endpoints)

        packets, received, endpoints = packet_series("j345")
        packets[10]["controller_mode_by_motor"]["J6"] = "brake"
        with self.assertRaisesRegex(MODULE.CaptureError, "motor mode mismatch"):
            self.analyze("j345", packets, received, endpoints)

    def test_collection_json_failure_is_wrapped_with_terminal_diagnostic(self):
        listener = FakeListener([(b"{invalid-json", ("127.0.0.1", 49152))])
        process = FakeProcess()
        stop_worker = mock.Mock(
            return_value=(terminal_stdout("j345"), "synthetic stderr", "none")
        )
        with mock.patch.object(MODULE.socket, "socket", return_value=listener), mock.patch.object(
            MODULE.subprocess, "Popen", return_value=process
        ), mock.patch.object(
            MODULE.raw_base.brake_verify, "stop_worker", stop_worker
        ), mock.patch.object(
            MODULE.time, "monotonic", return_value=0.0
        ), mock.patch.object(
            MODULE.time, "monotonic_ns", return_value=1_001_000_000
        ):
            with self.assertRaises(MODULE.DomainWorkerError) as caught:
                MODULE._capture_domain(
                    worker=Path("/tmp/worker"),
                    feedback_port=15300,
                    bus="j345",
                    target_packets=500,
                    maximum_runtime_s=8.0,
                )
        diagnostic = caught.exception.diagnostic
        self.assertIn("feedback collection failed", str(caught.exception))
        self.assertEqual(diagnostic["worker_returncode"], 0)
        self.assertEqual(diagnostic["terminal_proof"]["FINAL_BRAKE"], "PASS")
        self.assertEqual(diagnostic["stderr"], ["synthetic stderr"])
        self.assertTrue(listener.closed)
        stop_worker.assert_called_once_with(process)

    def test_hard_timing_failure_live_path_keeps_complete_diagnostic(self):
        packets, received, _endpoints = packet_series("j345")
        for packet in packets[100:]:
            packet["source_monotonic_ns"] += 240_000_001
        shift_stamps_after(received, 100, 240_000_001)
        datagrams = [
            (json.dumps(packet).encode("utf-8"), ("127.0.0.1", 49152))
            for packet in packets
        ]
        listener = FakeListener(datagrams)
        process = FakeProcess()
        stop_worker = mock.Mock(return_value=(terminal_stdout("j345"), "", "none"))
        with mock.patch.object(MODULE.socket, "socket", return_value=listener), mock.patch.object(
            MODULE.subprocess, "Popen", return_value=process
        ), mock.patch.object(
            MODULE.raw_base.brake_verify, "stop_worker", stop_worker
        ), mock.patch.object(
            MODULE.time, "monotonic", return_value=0.0
        ), mock.patch.object(
            MODULE.time, "monotonic_ns", side_effect=received
        ):
            with self.assertRaises(MODULE.DomainWorkerError) as caught:
                MODULE._capture_domain(
                    worker=Path("/tmp/worker"),
                    feedback_port=15300,
                    bus="j345",
                    target_packets=500,
                    maximum_runtime_s=8.0,
                )
        diagnostic = caught.exception.diagnostic
        self.assertIn("local feedback gap", str(caught.exception))
        self.assertEqual(
            diagnostic["timing"]["maximum_local_gap_ns"],
            MODULE.MAX_GAP_NS + 1,
        )
        self.assertEqual(diagnostic["packet_health"]["unhealthy_packet_count"], 0)
        self.assertEqual(diagnostic["terminal_proof"]["FINAL_BRAKE"], "PASS")
        self.assertTrue(listener.closed)
        stop_worker.assert_called_once_with(process)


if __name__ == "__main__":
    unittest.main()

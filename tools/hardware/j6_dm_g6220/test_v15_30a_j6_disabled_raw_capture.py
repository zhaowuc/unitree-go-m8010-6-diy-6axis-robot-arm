#!/usr/bin/env python3
"""Offline mock tests for the J6 DISABLED refresh-only raw capture."""

from __future__ import annotations

import argparse
import json
import math
import sys
import tempfile
import unittest
from contextlib import contextmanager, redirect_stderr, redirect_stdout
from datetime import datetime, timezone
from io import StringIO
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import v15_30a_j6_disabled_raw_capture as capture


IDENTITY = {
    "sysfs": "/sys/bus/usb/devices/3-1",
    "vid": capture.EXPECTED_USB_VID,
    "pid": capture.EXPECTED_USB_PID,
    "serial": capture.EXPECTED_USB_SERIAL,
    "manufacturer": "DaMiao-Tech",
    "product": "DM-USB2FDCAN",
    "busnum": "3",
    "devnum": "119",
}
BOOT_ID = "a5542fa1-f39c-42c3-b466-a47b0e50aac9"
POSE_BINDING_ID = "b" * 64


class FakeLogger:
    def __init__(self, harness: "Harness") -> None:
        self.harness = harness
        self.channel = capture.EXPECTED_CHANNEL
        self.motor_id = capture.EXPECTED_MOTOR_ID
        self.master_id = None
        self.events: list[SimpleNamespace] = []
        self.sent: list[tuple[int, bytes, str]] = []
        self.closed = False

    def send(self, can_id: int, payload: bytes, label: str) -> float:
        self.sent.append((can_id, payload, label))
        event = SimpleNamespace(
            event_monotonic_s=self.harness.monotonic_value,
            event_kind="TX_REQUEST",
            can_id=can_id,
            payload=payload,
            normal=False,
        )
        self.events.append(event)
        return self.harness.monotonic_value

    def snapshot(self) -> list[SimpleNamespace]:
        return list(self.events)

    @staticmethod
    def classify(event: SimpleNamespace) -> str:
        return "NORMAL_FEEDBACK" if getattr(event, "normal", False) else "UNKNOWN"

    def close(self) -> None:
        if self.harness.close_raises:
            raise RuntimeError("injected close failure")
        self.closed = True


class Harness:
    def __init__(self) -> None:
        self.monotonic_value = 100.0
        self.now = datetime(2026, 8, 28, 10, 0, tzinfo=timezone.utc)
        self.loggers: list[FakeLogger] = []
        self.lock_entries = 0
        self.lock_exits = 0
        self.identity = dict(IDENTITY)
        self.count_by_label: dict[str, int] = {}
        self.duration_by_label = {
            "J6_DISABLED_ZERO_CAPTURE": 4.99,
            "J6_FINAL_DISABLED": 0.04,
        }
        self.states_by_label: dict[str, list[int]] = {}
        self.raise_by_label: dict[str, BaseException] = {}
        self.temperature_by_label: dict[str, float] = {}
        self.close_raises = False

    def logger_factory(self) -> FakeLogger:
        logger = FakeLogger(self)
        self.loggers.append(logger)
        return logger

    @contextmanager
    def lock_factory(self):
        self.lock_entries += 1
        try:
            yield
        finally:
            self.lock_exits += 1

    def sample_capturer(
        self, logger: capture.RefreshOnlyLogger, count: int, label: str
    ) -> tuple[list[tuple[SimpleNamespace, SimpleNamespace]], int]:
        if label in self.raise_by_label:
            raise self.raise_by_label[label]
        actual_count = self.count_by_label.get(label, count)
        duration = self.duration_by_label.get(label, 0.0)
        start = self.monotonic_value
        states = self.states_by_label.get(label, [0] * actual_count)
        temperature = self.temperature_by_label.get(label, 34.0)
        values = []
        start_index = len(logger.snapshot())
        for index in range(actual_count):
            logger.send(
                0x7FF,
                capture.refresh_request(),
                f"{label}_{index:03d}",
            )
            fraction = 0.0 if actual_count <= 1 else index / (actual_count - 1)
            event_time = start + duration * fraction
            event = SimpleNamespace(
                event_monotonic_s=event_time,
                event_kind="RX_CALLBACK",
                can_id=capture.EXPECTED_MASTER_ID,
                payload=b"",
                normal=True,
            )
            logger._logger.events.append(event)
            feedback = SimpleNamespace(
                state=states[index] if index < len(states) else 0,
                position=-2.0 + index * 1.0e-7,
                velocity=(-1 if index % 2 else 1) * 0.02,
                torque=0.0,
                mos_temp=temperature,
                coil_temp=temperature - 1.0,
            )
            values.append((event, feedback))
        self.monotonic_value = start + duration
        return values, start_index

    def dependencies(self) -> capture.RuntimeDependencies:
        return capture.RuntimeDependencies(
            logger_factory=self.logger_factory,
            identity_reader=lambda: dict(self.identity),
            sample_capturer=self.sample_capturer,
            monotonic=lambda: self.monotonic_value,
            now_utc=lambda: self.now,
            host_boot_id_reader=lambda: BOOT_ID,
            boottime_ns_reader=lambda: 123456789,
            token_factory=lambda: "1" * 32,
            lock_factory=self.lock_factory,
        )


def args_for(output: Path) -> argparse.Namespace:
    return argparse.Namespace(
        output=output,
        pose_binding_id=POSE_BINDING_ID,
        confirm=capture.CONFIRM_GATE,
        physical_confirmation=capture.PHYSICAL_GATE,
    )


class DisabledRawCaptureTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.output = self.root / "j6_capture.json"
        self.harness = Harness()
        self.deps = self.harness.dependencies()

    def run_capture(self) -> tuple[int, dict]:
        with redirect_stdout(StringIO()):
            returncode = capture.run(args_for(self.output), self.deps)
        document = json.loads(self.output.read_text(encoding="utf-8"))
        return returncode, document

    def test_pass_is_500_frames_four_seconds_and_finally_disabled(self) -> None:
        returncode, document = self.run_capture()
        self.assertEqual(returncode, 0)
        self.assertEqual(document["schema"], capture.CAPTURE_SCHEMA)
        self.assertEqual(document["status"], "PASS")
        self.assertEqual(document["pose_binding_id"], POSE_BINDING_ID)
        self.assertFalse(document["physical_power_off_required"])
        self.assertEqual(document["host_boot_id"], BOOT_ID)
        self.assertEqual(document["packet_count"], 500)
        self.assertGreaterEqual(document["source_coverage_s"], 4.0)
        self.assertEqual(document["motors"]["J6"]["sample_count"], 500)
        stats = document["motors"]["J6"]["raw_position_rad"]
        self.assertEqual(
            set(stats),
            {"mean", "median", "minimum", "maximum", "span", "standard_deviation"},
        )
        self.assertLess(stats["span"], capture.MAXIMUM_RAW_SPAN_RAD)
        self.assertEqual(document["terminal"]["final_disabled_frames"], 5)
        self.assertEqual(document["terminal"]["states"], [0] * 5)
        self.assertTrue(document["terminal"]["confirmed"])
        self.assertTrue(document["terminal"]["channel_closed"])
        self.assertEqual(document["physical_confirmation"], {
            "confirmation_gate": capture.PHYSICAL_GATE,
            "j6_24v_on": True,
            "support_reliable": True,
            "whole_arm_vertical_initialization_pose": True,
            "arm_stationary": True,
            "not_at_mechanical_limit": True,
        })
        safety = document["safety"]
        self.assertEqual(safety["execution_policy"], "DISABLED_REFRESH_ONLY")
        self.assertEqual(safety["refresh_request_count"], 505)
        for field in (
            "rid_read_request_count",
            "rid_write_count",
            "flash_or_eeprom_write_count",
            "motor_internal_zero_write_count",
            "set_zero_command_count",
            "fc_enable_count",
            "fd_disable_count",
            "position_velocity_torque_command_count",
            "other_tx_attempt_count",
            "forbidden_tx_attempt_count",
        ):
            self.assertEqual(safety[field], 0, field)
        self.assertTrue(safety["only_disabled_refresh_requests"])
        self.assertEqual(safety["observed_tx_audit"], {
            key: safety[key]
            for key in capture.empty_command_audit()
        })
        logger = self.harness.loggers[0]
        self.assertTrue(logger.closed)
        self.assertEqual(len(logger.sent), 505)
        self.assertTrue(all(
            can_id == 0x7FF and payload == capture.refresh_request()
            for can_id, payload, _ in logger.sent
        ))
        self.assertEqual(self.harness.lock_entries, 1)
        self.assertEqual(self.harness.lock_exits, 1)
        self.assertEqual(list(self.root.glob(".*.tmp")), [])

    def test_enabled_frame_produces_failure_evidence_and_closes(self) -> None:
        states = [0] * 500
        states[250] = 1
        self.harness.states_by_label["J6_DISABLED_ZERO_CAPTURE"] = states
        returncode, document = self.run_capture()
        self.assertEqual(returncode, 2)
        self.assertEqual(document["status"], "FAIL")
        self.assertEqual(document["pose_binding_id"], POSE_BINDING_ID)
        self.assertTrue(any("continuously DISABLED" in item for item in document["failures"]))
        self.assertTrue(document["physical_power_off_required"])
        self.assertTrue(document["terminal"]["channel_closed"])
        self.assertTrue(self.harness.loggers[0].closed)

    def test_short_capture_duration_is_rejected_with_evidence(self) -> None:
        self.harness.duration_by_label["J6_DISABLED_ZERO_CAPTURE"] = 3.99
        returncode, document = self.run_capture()
        self.assertEqual(returncode, 2)
        self.assertEqual(document["status"], "FAIL")
        self.assertLess(document["source_coverage_s"], 4.0)
        self.assertTrue(any("coverage" in item for item in document["failures"]))

    def test_fewer_than_500_frames_is_rejected(self) -> None:
        self.harness.count_by_label["J6_DISABLED_ZERO_CAPTURE"] = 499
        returncode, document = self.run_capture()
        self.assertEqual(returncode, 2)
        self.assertEqual(document["packet_count"], 499)
        self.assertEqual(document["motors"]["J6"]["sample_count"], 499)
        self.assertTrue(any("499/500" in item for item in document["failures"]))

    def test_final_five_frames_must_all_be_disabled(self) -> None:
        self.harness.states_by_label["J6_FINAL_DISABLED"] = [0, 0, 1, 0, 0]
        returncode, document = self.run_capture()
        self.assertEqual(returncode, 2)
        self.assertFalse(document["terminal"]["confirmed"])
        self.assertEqual(document["terminal"]["final_disabled_frames"], 4)

    def test_unsafe_temperature_is_rejected(self) -> None:
        self.harness.temperature_by_label["J6_DISABLED_ZERO_CAPTURE"] = 60.0
        returncode, document = self.run_capture()
        self.assertEqual(returncode, 2)
        self.assertTrue(any("temperature" in item for item in document["failures"]))

    def test_capture_exception_still_closes_and_publishes_failure(self) -> None:
        self.harness.raise_by_label["J6_FINAL_DISABLED"] = RuntimeError("injected")
        returncode, document = self.run_capture()
        self.assertEqual(returncode, 2)
        self.assertEqual(document["status"], "FAIL")
        self.assertTrue(document["terminal"]["channel_closed"])
        self.assertTrue(any("injected" in item for item in document["failures"]))

    def test_close_failure_is_not_claimed_closed_and_requires_power_off(self) -> None:
        self.harness.close_raises = True
        returncode, document = self.run_capture()
        self.assertEqual(returncode, 2)
        self.assertEqual(document["status"], "FAIL")
        self.assertFalse(document["terminal"]["channel_closed"])
        self.assertTrue(document["physical_power_off_required"])
        self.assertTrue(any(
            "close failure" in item for item in document["failures"]
        ))

    def test_wrong_usb_identity_never_opens_logger_but_leaves_evidence(self) -> None:
        self.harness.identity["serial"] = "WRONG"
        returncode, document = self.run_capture()
        self.assertEqual(returncode, 2)
        self.assertFalse(document["hardware_accessed"])
        self.assertEqual(self.harness.loggers, [])
        self.assertTrue(any("USB" in item for item in document["failures"]))

    def test_existing_output_is_never_overwritten_or_hardware_opened(self) -> None:
        self.output.write_text("preserve", encoding="utf-8")
        with self.assertRaises(capture.CaptureError):
            capture.run(args_for(self.output), self.deps)
        self.assertEqual(self.output.read_text(encoding="utf-8"), "preserve")
        self.assertEqual(self.harness.loggers, [])
        self.assertEqual(self.harness.lock_entries, 0)

    def test_physical_confirmation_is_exact_and_blocks_before_hardware(self) -> None:
        arguments = args_for(self.output)
        arguments.physical_confirmation = "SUPPORT_RELIABLE=YES"
        with self.assertRaises(capture.CaptureError):
            capture.run(arguments, self.deps)
        self.assertFalse(self.output.exists())
        self.assertEqual(self.harness.loggers, [])
        self.assertEqual(self.harness.lock_entries, 0)

    def test_pose_binding_id_is_required_and_strictly_lowercase_hex(self) -> None:
        valid = capture.parse_args([
            "--output", str(self.output),
            "--pose-binding-id", POSE_BINDING_ID,
            "--confirm", capture.CONFIRM_GATE,
            "--physical-confirmation", capture.PHYSICAL_GATE,
        ])
        self.assertEqual(valid.pose_binding_id, POSE_BINDING_ID)

        invalid_values = (
            "A" * 64,
            "b" * 63,
            "b" * 65,
            "b" * 63 + "g",
            " " + "b" * 64,
            "b" * 64 + " ",
        )
        for value in invalid_values:
            with self.subTest(value=repr(value)):
                with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
                    with self.assertRaises(SystemExit) as raised:
                        capture.parse_args([
                            "--output", str(self.output),
                            "--pose-binding-id", value,
                            "--confirm", capture.CONFIRM_GATE,
                            "--physical-confirmation", capture.PHYSICAL_GATE,
                        ])
                self.assertEqual(raised.exception.code, 2)

        with redirect_stdout(StringIO()), redirect_stderr(StringIO()):
            with self.assertRaises(SystemExit) as raised:
                capture.parse_args([
                    "--output", str(self.output),
                    "--confirm", capture.CONFIRM_GATE,
                    "--physical-confirmation", capture.PHYSICAL_GATE,
                ])
        self.assertEqual(raised.exception.code, 2)

    def test_invalid_programmatic_pose_binding_blocks_before_hardware(self) -> None:
        arguments = args_for(self.output)
        arguments.pose_binding_id = "B" * 64
        with self.assertRaises(capture.CaptureError):
            capture.run(arguments, self.deps)
        self.assertFalse(self.output.exists())
        self.assertEqual(self.harness.loggers, [])
        self.assertEqual(self.harness.lock_entries, 0)

    def test_main_blocked_stdout_keeps_valid_pose_binding_id(self) -> None:
        arguments = args_for(self.output)
        arguments.physical_confirmation = "SUPPORT_RELIABLE=YES"
        stdout = StringIO()
        with patch.object(capture, "parse_args", return_value=arguments):
            with redirect_stdout(stdout):
                returncode = capture.main()
        self.assertEqual(returncode, 4)
        blocked = json.loads(stdout.getvalue())
        self.assertEqual(blocked["status"], "BLOCKED")
        self.assertEqual(blocked["pose_binding_id"], POSE_BINDING_ID)
        self.assertFalse(self.output.exists())
        self.assertEqual(self.harness.loggers, [])

    def test_refresh_only_capability_rejects_every_forbidden_family(self) -> None:
        logger = FakeLogger(self.harness)
        guarded = capture.RefreshOnlyLogger(logger)
        forbidden = (
            (0x7FF, bytes.fromhex("0100330A00000000"), "RID_READ"),
            (0x7FF, bytes.fromhex("0100550A02000000"), "RID_WRITE"),
            (1, b"\xFF" * 7 + b"\xFC", "FC"),
            (1, b"\xFF" * 7 + b"\xFD", "FD"),
            (1, b"\xFF" * 7 + b"\xFE", "SET_ZERO"),
            (1, b"\x00" * 8, "POSITION"),
        )
        for can_id, payload, label in forbidden:
            with self.assertRaises(capture.CaptureError):
                guarded.send(can_id, payload, label)
        self.assertEqual(logger.sent, [])
        self.assertEqual(guarded.command_audit["forbidden_tx_attempt_count"], 6)
        self.assertEqual(guarded.command_audit["rid_read_request_count"], 1)
        self.assertEqual(guarded.command_audit["rid_write_count"], 1)
        self.assertEqual(guarded.command_audit["fc_enable_count"], 1)
        self.assertEqual(guarded.command_audit["fd_disable_count"], 1)
        self.assertEqual(guarded.command_audit["set_zero_command_count"], 1)
        self.assertEqual(
            guarded.command_audit["motor_internal_zero_write_count"], 1
        )
        self.assertEqual(
            guarded.command_audit["position_velocity_torque_command_count"], 1
        )

    def test_production_capturer_stops_after_first_non_disabled_frame(self) -> None:
        harness = self.harness

        class ImmediateFeedbackLogger(FakeLogger):
            def send(self, can_id: int, payload: bytes, label: str) -> float:
                sent_at = super().send(can_id, payload, label)
                feedback = SimpleNamespace(
                    state=1,
                    position=0.0,
                    velocity=0.0,
                    torque=0.0,
                    mos_temp=30.0,
                    coil_temp=30.0,
                )
                self.events.append(SimpleNamespace(
                    event_monotonic_s=sent_at,
                    event_kind="RX_CALLBACK",
                    can_id=capture.EXPECTED_MASTER_ID,
                    payload=b"",
                    normal=True,
                    feedback=feedback,
                ))
                return sent_at

        raw = ImmediateFeedbackLogger(harness)
        guarded = capture.RefreshOnlyLogger(raw)

        def decode_fake_feedback(_logger, events):
            return [
                (event, event.feedback)
                for event in events
                if hasattr(event, "feedback")
            ]

        with patch.object(capture, "normal_feedback", decode_fake_feedback):
            with self.assertRaises(capture.CaptureError):
                capture.capture_disabled_refresh_samples(guarded, 10, "FAIL_FAST")
        self.assertEqual(len(raw.sent), 1)

    def test_partial_raw_logger_construction_invokes_cleanup(self) -> None:
        cleanup_calls = []

        def broken_constructor(instance):
            instance.open_started = True
            raise RuntimeError("injected constructor failure")

        def cleanup(instance):
            cleanup_calls.append(instance.open_started)

        with patch.object(capture.RawCanLogger, "__init__", broken_constructor):
            with patch.object(capture.RawCanLogger, "close", cleanup):
                with self.assertRaises(RuntimeError):
                    capture.FailSafeRawCanLogger()
        self.assertEqual(cleanup_calls, [True])

    def test_nonfinite_feedback_is_rejected(self) -> None:
        original = self.harness.sample_capturer

        def nonfinite(logger, count, label):
            values, start = original(logger, count, label)
            if label == "J6_DISABLED_ZERO_CAPTURE":
                values[10][1].position = math.nan
            return values, start

        self.deps = capture.RuntimeDependencies(
            **{
                **self.deps.__dict__,
                "sample_capturer": nonfinite,
            }
        )
        returncode, document = self.run_capture()
        self.assertEqual(returncode, 2)
        self.assertTrue(any("non-finite" in item for item in document["failures"]))


if __name__ == "__main__":
    unittest.main()

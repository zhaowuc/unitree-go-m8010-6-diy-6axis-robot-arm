#!/usr/bin/env python3
"""Offline-only tests for the volatile J6 RID10 commissioning state machine."""

from __future__ import annotations

import argparse
import io
import json
import secrets
import sys
import tempfile
import unittest
from contextlib import nullcontext, redirect_stdout
from datetime import datetime, timedelta, timezone
from pathlib import Path
from types import SimpleNamespace

SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

import j6_posvel_mode_commissioning as commissioning


IDENTITY = {
    "sysfs": "/sys/bus/usb/devices/1-1",
    "vid": "34b7",
    "pid": "6877",
    "serial": "EEE8D71AB573449FCAFE7B39BD222C75",
    "manufacturer": "DAMIAO",
    "product": "USB2FDCAN",
    "busnum": "1",
    "devnum": "2",
}


class FakeLogger:
    def __init__(self, harness: "Harness") -> None:
        self.harness = harness
        self.master_id = None
        self.sent: list[tuple[int, bytes, str]] = []
        self.closed = False
        self.drain_calls = 0

    def allowed_feedback_ids(self) -> set[int]:
        ids = {commissioning.MOTOR_ID}
        if self.master_id is not None:
            ids.add(self.master_id)
        return ids

    def send(self, can_id: int, payload: bytes, label: str) -> float:
        self.sent.append((can_id, payload, label))
        if payload == commissioning.rid10_write_payload(
            commissioning.CTRL_MODE_POS_VEL
        ):
            if self.harness.inventory_path is not None:
                on_disk = json.loads(
                    self.harness.inventory_path.read_text(encoding="utf-8")
                )
                self.harness.status_seen_at_write = on_disk["status"]
                self.harness.attempt_count_seen_at_write = on_disk[
                    "runtime_mode_write"
                ]["rid10_write_attempt_count"]
            self.harness.mode = commissioning.CTRL_MODE_POS_VEL
        return 1.0

    def snapshot(self) -> list:
        return []

    def drain_events(self) -> tuple:
        self.drain_calls += 1
        return ()

    def close(self) -> None:
        self.closed = True


class Harness:
    def __init__(self) -> None:
        self.now = datetime(2026, 8, 27, 12, 0, tzinfo=timezone.utc)
        self.mode = commissioning.CTRL_MODE_MIT
        self.identity = dict(IDENTITY)
        self.loggers: list[FakeLogger] = []
        self.inventory_path: Path | None = None
        self.status_seen_at_write: str | None = None
        self.attempt_count_seen_at_write: int | None = None
        self.readback_failure = False
        self.rid10_read_values: list[float] = []
        self.states_by_label: dict[str, list[int]] = {}
        self.positions_by_label: dict[str, list[float]] = {}
        self.mos_temperatures_by_label: dict[str, list[float]] = {}
        self.coil_temperatures_by_label: dict[str, list[float]] = {}
        self.advance_seconds_by_label: dict[str, float] = {}
        self.evidence_failure = False
        self.lock_entries = 0

    def logger_factory(self) -> FakeLogger:
        logger = FakeLogger(self)
        self.loggers.append(logger)
        return logger

    def parameter_reader(self, _logger: FakeLogger, rid: int) -> float:
        if rid == 7:
            return 0.0
        if rid == 8:
            return 1.0
        if rid == 10:
            if self.rid10_read_values:
                return float(self.rid10_read_values.pop(0))
            if self.readback_failure and self.mode == commissioning.CTRL_MODE_POS_VEL:
                raise commissioning.Blocked("injected RID10 readback timeout")
            return float(self.mode)
        if rid == 21:
            return 12.5
        raise AssertionError(f"unexpected RID {rid}")

    def sample_capturer(
        self, _logger: FakeLogger, count: int, label: str
    ) -> tuple[list, int]:
        states = self.states_by_label.get(label, [0] * count)
        positions = self.positions_by_label.get(
            label, [1.0 + index * 1.0e-7 for index in range(count)]
        )
        mos_temperatures = self.mos_temperatures_by_label.get(label, [35.0] * count)
        coil_temperatures = self.coil_temperatures_by_label.get(label, [33.0] * count)
        samples = []
        for index, state in enumerate(states[:count]):
            feedback = SimpleNamespace(
                state=state,
                position=positions[index],
                velocity=0.0,
                mos_temp=mos_temperatures[index],
                coil_temp=coil_temperatures[index],
            )
            samples.append((SimpleNamespace(), feedback))
        self.now += timedelta(seconds=self.advance_seconds_by_label.get(label, 0.0))
        return samples, 0

    def lock_factory(self):
        self.lock_entries += 1
        return nullcontext()

    def evidence_recorder(
        self, inventory: dict, path: Path, _logger: FakeLogger
    ) -> None:
        if self.evidence_failure:
            raise OSError("injected evidence save failure")
        inventory.setdefault("evidence", []).append({"path": str(path), "mock": True})

    def dependencies(self) -> commissioning.RuntimeDependencies:
        return commissioning.RuntimeDependencies(
            logger_factory=self.logger_factory,
            identity_reader=lambda: dict(self.identity),
            parameter_reader=self.parameter_reader,
            sample_capturer=self.sample_capturer,
            sleep=lambda _seconds: None,
            now_utc=lambda: self.now,
            token_factory=lambda: secrets.token_hex(
                commissioning.SESSION_TOKEN_BYTES
            ),
            lock_factory=self.lock_factory,
            evidence_recorder=self.evidence_recorder,
            enforce_canonical_inventory=False,
        )


def args_for(
    path: Path, token: str = "", power_cycle_attestation: str = ""
) -> argparse.Namespace:
    return argparse.Namespace(
        phase="preflight" if not token else "switch-and-baseline",
        operator_gate=commissioning.MODE_GATE if token else "",
        session_token=token,
        power_cycle_attestation=power_cycle_attestation,
        output_dir=path.parent / "evidence",
        inventory=path,
    )


class CommissioningTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.inventory = Path(self.temporary.name) / "session.json"
        self.harness = Harness()
        self.harness.inventory_path = self.inventory
        self.deps = self.harness.dependencies()

    def run_preflight(self) -> dict:
        result, document = self.run_preflight_result()
        self.assertEqual(result, 0)
        return document

    def run_preflight_result(
        self, power_cycle_attestation: str = ""
    ) -> tuple[int, dict]:
        with redirect_stdout(io.StringIO()):
            result = commissioning.preflight(
                args_for(
                    self.inventory,
                    power_cycle_attestation=power_cycle_attestation,
                ),
                self.deps,
            )
        return result, json.loads(self.inventory.read_text(encoding="utf-8"))

    def run_commit(self, token: str) -> tuple[int, dict]:
        with redirect_stdout(io.StringIO()):
            result = commissioning.switch_and_baseline(
                args_for(self.inventory, token), self.deps
            )
        return result, json.loads(self.inventory.read_text(encoding="utf-8"))

    def test_preflight_ready_is_atomic_tokenized_and_finally_disabled(self) -> None:
        document = self.run_preflight()
        self.assertEqual(document["status"], commissioning.PREFLIGHT_READY)
        token = document["commissioning_session"]["token"]
        self.assertEqual(len(token), commissioning.SESSION_TOKEN_BYTES * 2)
        self.assertEqual(document["device_identity"], IDENTITY)
        self.assertEqual(document["rid_snapshot"]["ctrl_mode_rid10"], 1)
        self.assertEqual(document["rid_snapshot"]["ctrl_mode_name"], "MIT")
        self.assertFalse(
            document["persistence_semantics"]["explicit_save_command_implemented"]
        )
        self.assertFalse(
            document["persistence_semantics"][
                "flash_or_eeprom_vendor_guarantee_claimed"
            ]
        )
        self.assertTrue(document["final_disable"]["confirmed"])
        logger = self.harness.loggers[-1]
        fd_frames = [frame for frame in logger.sent if frame[1] == b"\xFF" * 7 + b"\xFD"]
        self.assertEqual(len(fd_frames), commissioning.FINAL_DISABLE_SEND_COUNT)
        self.assertEqual(logger.drain_calls, 0)
        self.assertTrue(logger.closed)
        self.assertEqual(list(self.inventory.parent.glob(".*.tmp")), [])

    def test_already_posvel_completes_without_any_mode_write(self) -> None:
        self.harness.mode = commissioning.CTRL_MODE_POS_VEL
        document = self.run_preflight()

        self.assertEqual(document["status"], commissioning.FINAL_STATUS_COMPLETED)
        self.assertEqual(document["result"], commissioning.RESULT_ALREADY_POSVEL)
        self.assertEqual(document["rid_snapshot"]["ctrl_mode_rid10"], 2)
        self.assertEqual(document["rid_snapshot"]["ctrl_mode_name"], "POS_VEL")
        self.assertNotIn("ctrl_mode_after_write", document)
        self.assertEqual(
            document["ctrl_mode_readonly_verification"],
            {
                "rid": 10,
                "initial": 2,
                "after_disabled_baseline": 2,
                "write_performed": False,
            },
        )
        self.assertEqual(
            document["commissioning_session"]["token_authority"],
            "NONE_ALREADY_POS_VEL",
        )
        write = document["runtime_mode_write"]
        self.assertEqual(write["operator_gate"], "NOT_REQUIRED_ALREADY_POS_VEL")
        self.assertEqual(write["from"], 2)
        self.assertEqual(write["to"], 2)
        self.assertFalse(write["required"])
        self.assertEqual(write["rid10_write_attempt_count"], 0)
        self.assertEqual(write["rid10_write_count"], 0)
        baseline = document["posvel_disabled_baseline"]
        self.assertEqual(baseline["valid_frames"], 100)
        self.assertEqual(baseline["states"], ["DISABLED"])
        self.assertFalse(baseline["mode_switch_performed"])
        self.assertLessEqual(abs(baseline["readonly_observation_delta_deg"]), 0.2)
        self.assertTrue(document["final_disable"]["confirmed"])

        logger = self.harness.loggers[-1]
        writes = [
            frame
            for frame in logger.sent
            if frame[0] == 0x7FF and frame[1][2:4] == bytes((0x55, 10))
        ]
        self.assertEqual(writes, [])
        self.assertFalse(any("WRITE" in frame[2] for frame in logger.sent))
        fd_frames = [
            frame
            for frame in logger.sent
            if frame[1] == b"\xFF" * 7 + b"\xFD"
        ]
        self.assertEqual(len(fd_frames), commissioning.FINAL_DISABLE_SEND_COUNT)

    def test_already_posvel_non_disabled_baseline_blocks_without_write(self) -> None:
        self.harness.mode = commissioning.CTRL_MODE_POS_VEL
        self.harness.states_by_label["POSVEL_ALREADY_DISABLED_BASELINE"] = (
            [0] * 99 + [1]
        )
        result, document = self.run_preflight_result()
        self.assertEqual(result, 4)
        self.assertEqual(document["status"], commissioning.BLOCKED_NO_MODE_WRITE)
        self.assertEqual(document["runtime_mode_write"]["rid10_write_count"], 0)
        self.assertTrue(document["final_disable"]["confirmed"])
        self.assertFalse(any("WRITE" in frame[2] for frame in self.harness.loggers[-1].sent))

    def test_already_posvel_short_baseline_blocks_without_write(self) -> None:
        self.harness.mode = commissioning.CTRL_MODE_POS_VEL
        self.harness.states_by_label["POSVEL_ALREADY_DISABLED_BASELINE"] = [0] * 99
        result, document = self.run_preflight_result()
        self.assertEqual(result, 4)
        self.assertEqual(document["status"], commissioning.BLOCKED_NO_MODE_WRITE)
        self.assertEqual(document["runtime_mode_write"]["rid10_write_attempt_count"], 0)

    def test_already_posvel_displacement_blocks_without_write(self) -> None:
        self.harness.mode = commissioning.CTRL_MODE_POS_VEL
        self.harness.positions_by_label["POSVEL_ALREADY_DISABLED_BASELINE"] = [
            1.01
        ] * commissioning.ALREADY_POSVEL_BASELINE_COUNT
        result, document = self.run_preflight_result()
        self.assertEqual(result, 4)
        self.assertEqual(document["status"], commissioning.BLOCKED_NO_MODE_WRITE)
        self.assertGreater(
            abs(document["posvel_disabled_baseline"]["readonly_observation_delta_deg"]),
            0.2,
        )
        self.assertEqual(document["runtime_mode_write"]["rid10_write_count"], 0)

    def test_already_posvel_symmetric_motion_span_blocks_without_write(self) -> None:
        self.harness.mode = commissioning.CTRL_MODE_POS_VEL
        self.harness.positions_by_label["POSVEL_ALREADY_DISABLED_BASELINE"] = [
            value
            for _index in range(commissioning.ALREADY_POSVEL_BASELINE_COUNT // 2)
            for value in (0.9, 1.1)
        ]
        result, document = self.run_preflight_result()
        self.assertEqual(result, 4)
        self.assertEqual(document["status"], commissioning.BLOCKED_NO_MODE_WRITE)
        self.assertGreater(
            document["posvel_disabled_baseline"]["readonly_span_rad"],
            commissioning.MAXIMUM_RAW_SPAN_RAD,
        )
        self.assertEqual(document["runtime_mode_write"]["rid10_write_count"], 0)

    def test_already_posvel_nonfinite_feedback_blocks_without_write(self) -> None:
        self.harness.mode = commissioning.CTRL_MODE_POS_VEL
        self.harness.positions_by_label["POSVEL_ALREADY_DISABLED_BASELINE"] = [
            float("nan")
        ] * commissioning.ALREADY_POSVEL_BASELINE_COUNT
        result, document = self.run_preflight_result()
        self.assertEqual(result, 4)
        self.assertEqual(document["status"], commissioning.BLOCKED_NO_MODE_WRITE)
        self.assertIn("non-finite", document["blocker"]["reason"])

    def test_already_posvel_overtemperature_blocks_without_write(self) -> None:
        self.harness.mode = commissioning.CTRL_MODE_POS_VEL
        self.harness.mos_temperatures_by_label[
            "POSVEL_ALREADY_DISABLED_BASELINE"
        ] = [60.0] * commissioning.ALREADY_POSVEL_BASELINE_COUNT
        result, document = self.run_preflight_result()
        self.assertEqual(result, 4)
        self.assertEqual(document["status"], commissioning.BLOCKED_NO_MODE_WRITE)
        self.assertIn("temperature", document["blocker"]["reason"])

    def test_already_posvel_final_disabled_failure_requires_power_cycle(self) -> None:
        self.harness.mode = commissioning.CTRL_MODE_POS_VEL
        self.harness.states_by_label["PREFLIGHT_FINAL_DISABLED_CONFIRM"] = [
            0,
            0,
            1,
            0,
            0,
        ]
        result, document = self.run_preflight_result()
        self.assertEqual(result, 4)
        self.assertEqual(document["status"], commissioning.FINAL_STATUS_FAILED)
        self.assertEqual(document["runtime_mode_write"]["rid10_write_count"], 0)
        self.assertFalse(document["final_disable"]["confirmed"])
        self.assertEqual(
            document["physical_action_required"],
            commissioning.PHYSICAL_POWER_OFF_ACTION,
        )

    def test_already_posvel_final_overtemperature_requires_power_cycle(self) -> None:
        self.harness.mode = commissioning.CTRL_MODE_POS_VEL
        self.harness.mos_temperatures_by_label[
            "PREFLIGHT_FINAL_DISABLED_CONFIRM"
        ] = [60.0] * commissioning.FINAL_DISABLED_CONFIRM_COUNT
        result, document = self.run_preflight_result()
        self.assertEqual(result, 4)
        self.assertEqual(document["status"], commissioning.FINAL_STATUS_FAILED)
        self.assertFalse(document["final_disable"]["confirmed"])
        self.assertTrue(document["final_disable"]["errors"])
        self.assertEqual(document["runtime_mode_write"]["rid10_write_count"], 0)

    def test_already_posvel_final_mode_drift_blocks_without_write(self) -> None:
        self.harness.mode = commissioning.CTRL_MODE_POS_VEL
        self.harness.rid10_read_values = [
            commissioning.CTRL_MODE_POS_VEL,
            commissioning.CTRL_MODE_MIT,
        ]
        result, document = self.run_preflight_result()
        self.assertEqual(result, 4)
        self.assertEqual(document["status"], commissioning.BLOCKED_NO_MODE_WRITE)
        self.assertEqual(
            document["ctrl_mode_readonly_verification"]["after_disabled_baseline"],
            commissioning.CTRL_MODE_MIT,
        )
        self.assertEqual(document["runtime_mode_write"]["rid10_write_count"], 0)

    def test_already_posvel_evidence_failure_blocks_without_write(self) -> None:
        self.harness.mode = commissioning.CTRL_MODE_POS_VEL
        self.harness.evidence_failure = True
        result, document = self.run_preflight_result()
        self.assertEqual(result, 4)
        self.assertEqual(document["status"], commissioning.BLOCKED_NO_MODE_WRITE)
        self.assertEqual(document["runtime_mode_write"]["rid10_write_count"], 0)
        self.assertIn("raw evidence save failed", document["blocker"]["reason"])

    def test_unknown_mode_blocks_without_write(self) -> None:
        self.harness.mode = 3
        result, document = self.run_preflight_result()
        self.assertEqual(result, 4)
        self.assertEqual(document["status"], commissioning.BLOCKED_NO_MODE_WRITE)
        self.assertEqual(document["runtime_mode_write"]["rid10_write_count"], 0)

    def test_already_posvel_completed_token_cannot_enter_commit(self) -> None:
        self.harness.mode = commissioning.CTRL_MODE_POS_VEL
        document = self.run_preflight()
        logger_count = len(self.harness.loggers)
        with self.assertRaises(commissioning.Blocked):
            with redirect_stdout(io.StringIO()):
                commissioning.switch_and_baseline(
                    args_for(
                        self.inventory,
                        document["commissioning_session"]["token"],
                    ),
                    self.deps,
                )
        self.assertEqual(len(self.harness.loggers), logger_count)

    def test_already_posvel_completed_recovery_needs_no_power_cycle(self) -> None:
        self.harness.mode = commissioning.CTRL_MODE_POS_VEL
        self.run_preflight()
        second = self.run_preflight()
        self.assertEqual(
            second["prior_session_recovery"]["reason"],
            "PRIOR_COMPLETED_SESSION_PROVED_NO_MODE_WRITE",
        )

    def test_malformed_already_completed_recovery_requires_power_cycle(self) -> None:
        self.harness.mode = commissioning.CTRL_MODE_POS_VEL
        document = self.run_preflight()
        document["final_disable"]["errors"] = ["injected prior health failure"]
        self.inventory.write_text(json.dumps(document), encoding="utf-8")
        logger_count = len(self.harness.loggers)
        with self.assertRaises(commissioning.Blocked):
            with redirect_stdout(io.StringIO()):
                commissioning.preflight(args_for(self.inventory), self.deps)
        self.assertEqual(len(self.harness.loggers), logger_count)

    def test_malformed_blocked_no_write_recovery_requires_power_cycle(self) -> None:
        self.harness.mode = 3
        result, document = self.run_preflight_result()
        self.assertEqual(result, 4)
        document["schema_version"] = "1.0"
        self.inventory.write_text(json.dumps(document), encoding="utf-8")
        logger_count = len(self.harness.loggers)
        with self.assertRaises(commissioning.Blocked):
            with redirect_stdout(io.StringIO()):
                commissioning.preflight(args_for(self.inventory), self.deps)
        self.assertEqual(len(self.harness.loggers), logger_count)

    def test_written_completed_session_still_requires_power_cycle(self) -> None:
        token = self.run_preflight()["commissioning_session"]["token"]
        result, document = self.run_commit(token)
        self.assertEqual(result, 0)
        self.assertEqual(document["result"], commissioning.RESULT_SWITCHED_TO_POSVEL)
        logger_count = len(self.harness.loggers)
        with self.assertRaises(commissioning.Blocked):
            with redirect_stdout(io.StringIO()):
                commissioning.preflight(args_for(self.inventory), self.deps)
        self.assertEqual(len(self.harness.loggers), logger_count)

    def test_success_consumes_before_exactly_one_write_and_finishes_disabled(self) -> None:
        token = self.run_preflight()["commissioning_session"]["token"]
        result, document = self.run_commit(token)
        self.assertEqual(result, 0)
        self.assertEqual(document["status"], commissioning.FINAL_STATUS_COMPLETED)
        self.assertEqual(
            self.harness.status_seen_at_write, commissioning.COMMIT_IN_PROGRESS
        )
        self.assertEqual(self.harness.attempt_count_seen_at_write, 1)
        logger = self.harness.loggers[-1]
        writes = [
            frame
            for frame in logger.sent
            if frame[0] == 0x7FF and frame[1][2:4] == bytes((0x55, 10))
        ]
        self.assertEqual(
            writes,
            [
                (
                    0x7FF,
                    bytes.fromhex("0100550A02000000"),
                    "WRITE_RID10_POS_VEL_ONCE",
                )
            ],
        )
        fd_frames = [frame for frame in logger.sent if frame[1] == b"\xFF" * 7 + b"\xFD"]
        self.assertEqual(len(fd_frames), commissioning.FINAL_DISABLE_SEND_COUNT)
        self.assertTrue(document["final_disable"]["confirmed"])
        self.assertFalse(document["safety"]["flash_eeprom_save_used"])
        self.assertFalse(document["safety"]["automatic_mit_rollback_used"])

    def test_wrong_token_does_not_open_transport_or_consume_ready_inventory(self) -> None:
        self.run_preflight()
        logger_count = len(self.harness.loggers)
        with self.assertRaises(commissioning.Blocked):
            commissioning.switch_and_baseline(
                args_for(self.inventory, "0" * 64), self.deps
            )
        document = json.loads(self.inventory.read_text(encoding="utf-8"))
        self.assertEqual(document["status"], commissioning.PREFLIGHT_READY)
        self.assertFalse(document["commissioning_session"]["token_consumed"])
        self.assertEqual(len(self.harness.loggers), logger_count)

    def test_expired_token_is_blocked_without_opening_transport(self) -> None:
        token = self.run_preflight()["commissioning_session"]["token"]
        logger_count = len(self.harness.loggers)
        self.harness.now += timedelta(
            seconds=commissioning.PREFLIGHT_TTL_SECONDS + 1
        )
        result, document = self.run_commit(token)
        self.assertEqual(result, 4)
        self.assertEqual(document["status"], commissioning.BLOCKED_NO_MODE_WRITE)
        self.assertIn("expired", document["blocker"]["reason"])
        self.assertEqual(len(self.harness.loggers), logger_count)
        self.assertEqual(document["physical_action_required"], "NONE")

    def test_token_expiring_during_prechecks_is_blocked_at_write_boundary(self) -> None:
        token = self.run_preflight()["commissioning_session"]["token"]
        self.harness.advance_seconds_by_label["PRE_SWITCH_DISABLED"] = (
            commissioning.PREFLIGHT_TTL_SECONDS + 1
        )
        result, document = self.run_commit(token)
        self.assertEqual(result, 4)
        self.assertEqual(document["status"], commissioning.BLOCKED_NO_MODE_WRITE)
        self.assertEqual(document["runtime_mode_write"]["rid10_write_count"], 0)
        self.assertEqual(document["runtime_mode_write"]["rid10_write_attempt_count"], 0)
        self.assertTrue(document["final_disable"]["confirmed"])

    def test_readback_failure_never_retries_or_rolls_back_and_finishes_fd(self) -> None:
        token = self.run_preflight()["commissioning_session"]["token"]
        self.harness.readback_failure = True
        result, document = self.run_commit(token)
        self.assertEqual(result, 4)
        self.assertEqual(document["status"], commissioning.FINAL_STATUS_FAILED)
        logger = self.harness.loggers[-1]
        writes = [frame for frame in logger.sent if frame[1][2:4] == bytes((0x55, 10))]
        self.assertEqual(len(writes), 1)
        self.assertEqual(writes[0][1], bytes.fromhex("0100550A02000000"))
        self.assertNotIn(bytes.fromhex("0100550A01000000"), [item[1] for item in logger.sent])
        fd_frames = [frame for frame in logger.sent if frame[1] == b"\xFF" * 7 + b"\xFD"]
        self.assertEqual(len(fd_frames), commissioning.FINAL_DISABLE_SEND_COUNT)
        self.assertTrue(document["final_disable"]["confirmed"])
        self.assertFalse(document["safety"]["automatic_mit_rollback_used"])
        self.assertIn("commit", document["raw_evidence"])
        self.assertEqual(logger.drain_calls, 0)

    def test_failed_final_disabled_proof_forces_power_cycle_status(self) -> None:
        token = self.run_preflight()["commissioning_session"]["token"]
        self.harness.states_by_label["COMMIT_FINAL_DISABLED_CONFIRM"] = [1] * 5
        result, document = self.run_commit(token)
        self.assertEqual(result, 4)
        self.assertEqual(document["status"], commissioning.FINAL_STATUS_FAILED)
        self.assertFalse(document["final_disable"]["confirmed"])
        self.assertEqual(document["final_disable"]["states"], [1] * 5)

    def test_preflight_failure_after_open_still_sends_fd_and_records_failure(self) -> None:
        self.harness.states_by_label["INITIAL_READ_ONLY"] = [1] * 5
        with redirect_stdout(io.StringIO()):
            result = commissioning.preflight(args_for(self.inventory), self.deps)
        self.assertEqual(result, 4)
        document = json.loads(self.inventory.read_text(encoding="utf-8"))
        self.assertEqual(document["status"], commissioning.BLOCKED_NO_MODE_WRITE)
        logger = self.harness.loggers[-1]
        fd_frames = [frame for frame in logger.sent if frame[1] == b"\xFF" * 7 + b"\xFD"]
        self.assertEqual(len(fd_frames), commissioning.FINAL_DISABLE_SEND_COUNT)
        self.assertTrue(document["final_disable"]["confirmed"])
        self.assertEqual(document["physical_action_required"], "NONE")
        self.assertIn("preflight", document["raw_evidence"])

    def test_interrupted_commit_cannot_replay_write(self) -> None:
        document = self.run_preflight()
        token = document["commissioning_session"]["token"]
        document["status"] = commissioning.COMMIT_IN_PROGRESS
        document["commissioning_session"]["token_consumed"] = True
        commissioning.atomic_write_json(self.inventory, document)
        logger_count = len(self.harness.loggers)
        result, failed = self.run_commit(token)
        self.assertEqual(result, 4)
        self.assertEqual(failed["status"], commissioning.FINAL_STATUS_FAILED)
        self.assertIn("ambiguous", failed["failure"]["reason"])
        self.assertEqual(len(self.harness.loggers), logger_count)

    def test_lock_contract_matches_gui_worker(self) -> None:
        self.assertEqual(
            commissioning.DEVICE_LOCK_PATH, Path("/tmp/v15_30a_gui_j6.lock")
        )

    def test_production_rejects_noncanonical_ledger_path(self) -> None:
        production_policy = commissioning.RuntimeDependencies(
            enforce_canonical_inventory=True
        )
        with self.assertRaises(commissioning.Blocked):
            commissioning._require_canonical_inventory(
                args_for(self.inventory), production_policy
            )
        canonical_args = args_for(commissioning.CANONICAL_LEDGER_PATH)
        commissioning._require_canonical_inventory(canonical_args, production_policy)

    def test_identity_change_before_consumption_is_blocked_without_power_alarm(self) -> None:
        token = self.run_preflight()["commissioning_session"]["token"]
        logger_count = len(self.harness.loggers)
        self.harness.identity["devnum"] = "3"
        result, document = self.run_commit(token)
        self.assertEqual(result, 4)
        self.assertEqual(document["status"], commissioning.BLOCKED_NO_MODE_WRITE)
        self.assertEqual(document["physical_action_required"], "NONE")
        self.assertEqual(document["runtime_mode_write"]["rid10_write_count"], 0)
        self.assertEqual(len(self.harness.loggers), logger_count)
        self.assertEqual(
            document["final_disable"]["reason"], "TRANSPORT_WAS_NOT_OPENED"
        )

    def test_failed_session_requires_power_cycle_attestation_before_new_preflight(self) -> None:
        token = self.run_preflight()["commissioning_session"]["token"]
        self.harness.states_by_label["COMMIT_FINAL_DISABLED_CONFIRM"] = [1] * 5
        result, failed = self.run_commit(token)
        self.assertEqual(result, 4)
        self.assertEqual(failed["status"], commissioning.FINAL_STATUS_FAILED)
        logger_count = len(self.harness.loggers)

        with self.assertRaises(commissioning.Blocked):
            commissioning.preflight(args_for(self.inventory), self.deps)
        self.assertEqual(len(self.harness.loggers), logger_count)

        # This is a purely simulated power cycle: no real transport exists in tests.
        self.harness.mode = commissioning.CTRL_MODE_MIT
        del self.harness.states_by_label["COMMIT_FINAL_DISABLED_CONFIRM"]
        with redirect_stdout(io.StringIO()):
            replacement_result = commissioning.preflight(
                args_for(
                    self.inventory,
                    power_cycle_attestation=commissioning.POWER_CYCLE_ATTESTATION,
                ),
                self.deps,
            )
        self.assertEqual(replacement_result, 0)
        replacement = json.loads(self.inventory.read_text(encoding="utf-8"))
        self.assertEqual(replacement["status"], commissioning.PREFLIGHT_READY)
        self.assertEqual(
            replacement["prior_session_recovery"]["power_cycle_attestation"],
            "PASS",
        )

    def test_commit_in_progress_inventory_cannot_be_overwritten_by_preflight(self) -> None:
        document = self.run_preflight()
        document["status"] = commissioning.COMMIT_IN_PROGRESS
        document["commissioning_session"]["token_consumed"] = True
        document["runtime_mode_write"]["rid10_write_attempt_count"] = 1
        commissioning.atomic_write_json(self.inventory, document)
        logger_count = len(self.harness.loggers)
        with self.assertRaises(commissioning.Blocked):
            commissioning.preflight(args_for(self.inventory), self.deps)
        preserved = json.loads(self.inventory.read_text(encoding="utf-8"))
        self.assertEqual(preserved["status"], commissioning.COMMIT_IN_PROGRESS)
        self.assertEqual(len(self.harness.loggers), logger_count)

    def test_payload_builder_rejects_mit_rollback(self) -> None:
        with self.assertRaises(ValueError):
            commissioning.rid10_write_payload(commissioning.CTRL_MODE_MIT)


if __name__ == "__main__":
    unittest.main()

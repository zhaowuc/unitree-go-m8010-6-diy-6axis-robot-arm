#!/usr/bin/env python3
"""Create an immutable, offline J2 vertical-session phase anchor.

The tool only reads already-created JSON evidence.  It never opens a motor,
serial, CAN, or UDP endpoint.  Dry-run is the default.  Apply mode creates one
new, versioned anchor and one short-lived, single-use launch permit with
no-overwrite atomic publication; it never edits the persistent software zero,
its checksum, recovery hints, initial pose, or raw capture.

The long-lived anchor is evidence for a later, separately reviewed
boot-alignment step.  It is not a software zero and is not authorization to
enter an active mode.  A launch permit is issued only together with a new
anchor, is bound to the capture host boot and worker, expires after 30 seconds
of CLOCK_BOOTTIME, and still does not authorize active control by itself.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import tempfile
import time
import uuid
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
    "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws/src/go_m8010_arm_hardware"))
from go_m8010_arm_hardware.state_model import (
    PRESERVED_SESSION_REFERENCE_SCHEMA, session_reference_for_raw,
    validate_preserved_session_reference,
)


ANCHOR_SCHEMA = "go-m8010-j2-power-session-reference/1.0"
LAUNCH_PERMIT_SCHEMA = "go-m8010-j2-power-session-launch-permit/1.0"
CAPTURE_SCHEMA = "go-m8010-j2-brake-raw-capture-statistics/1.0"
ZERO_SCHEMA = "go-m8010-persistent-software-zero/1.0"
HINT_SCHEMA = "go-m8010-recovery-branch-hints/1.0"
REFERENCE_NAME = "PERSISTENT_SOFTWARE_ZERO_V1"
APPLY_GATE = "V15_30A_CREATE_J2_VERTICAL_SESSION_PHASE_ANCHOR=YES"
PHYSICAL_GATE = (
    "J2_VERTICAL=YES;SUPPORT_RELIABLE=YES;ARM_STATIONARY=YES;"
    "NOT_AT_MECHANICAL_LIMIT=YES"
)
EXPECTED_MOTORS = {"J1", "J2A", "J2B", "J3", "J4", "J5", "J6"}
J2_SIGNS = {"J2A": -1, "J2B": 1}
GEAR_RATIO = 6.329999923706055
TWO_PI = 2.0 * math.pi
# GO-M8010-6 position is signed int32 Q15 (approximately +/-65536 rad).
# The unrelated DM-G6220 PMAX value of 12.5 rad must not reject a valid GO
# multi-turn phase reference.
MAX_PROTOCOL_POSITION_RAD = float(1 << 16)
MIN_SAMPLE_COUNT = 450
MIN_SOURCE_COVERAGE_S = 4.0
MAX_RAW_SPAN_RAD = GEAR_RATIO * math.radians(0.2)
MAX_TEXT_LENGTH = 256
LAUNCH_PERMIT_TTL_SECONDS = 30
LAUNCH_PERMIT_TTL_NS = LAUNCH_PERMIT_TTL_SECONDS * 1_000_000_000
MAX_CAPTURE_TO_PERMIT_SECONDS = 300
MAX_CAPTURE_TO_PERMIT_NS = MAX_CAPTURE_TO_PERMIT_SECONDS * 1_000_000_000
HOST_BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")
STABLE_J2_SERIAL_BY_ID = (
    "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if01-port0"
)
STARTUP_RECHECK_MINIMUM_BRAKE_FRAMES = 50
STARTUP_RECHECK_MAX_RAW_PHASE_DELTA_RAD = GEAR_RATIO * math.radians(0.25)
STARTUP_RECHECK_MAX_RAW_SPAN_RAD = GEAR_RATIO * math.radians(0.20)
STARTUP_PRIME_MAX_INVALID_PREFIX_PAIRS = 3
STARTUP_PRIME_REQUIRED_HEALTHY_PAIRS = 5
STARTUP_PRIME_MAXIMUM_ELAPSED_NS = 500_000_000
FINAL_BRAKE_TX_ATTEMPT_COUNT = 40


class AnchorValidationError(ValueError):
    """Raised when pinned offline evidence is incomplete or inconsistent."""


def json_bytes(document: dict[str, Any]) -> bytes:
    return (
        json.dumps(document, ensure_ascii=False, indent=2, sort_keys=False) + "\n"
    ).encode("utf-8")


def canonical_bytes(document: dict[str, Any]) -> bytes:
    return json.dumps(
        document,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path) -> tuple[dict[str, Any], bytes]:
    data = path.read_bytes()
    try:
        document = json.loads(data)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise AnchorValidationError(f"{path.name} is not valid UTF-8 JSON") from exc
    if not isinstance(document, dict):
        raise AnchorValidationError(f"{path.name} must contain a JSON object")
    return document, data


def normalized_sha256(value: Any, label: str) -> str:
    if not isinstance(value, str):
        raise AnchorValidationError(f"{label} SHA-256 is malformed")
    normalized = value.lower()
    if len(normalized) != 64 or any(
        character not in "0123456789abcdef" for character in normalized
    ):
        raise AnchorValidationError(f"{label} SHA-256 is malformed")
    return normalized


def finite_number(value: Any, label: str) -> float:
    if type(value) not in {int, float} or not math.isfinite(float(value)):
        raise AnchorValidationError(f"{label} must be finite")
    return float(value)


def nonempty_text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise AnchorValidationError(f"{label} must be a non-empty string")
    normalized = value.strip()
    if len(normalized) > MAX_TEXT_LENGTH or any(
        ord(character) < 0x20 for character in normalized
    ):
        raise AnchorValidationError(f"{label} is invalid or too long")
    return normalized


def positive_integer(value: Any, label: str) -> int:
    if type(value) is not int or value <= 0:
        raise AnchorValidationError(f"{label} must be a positive integer")
    return value


def exact_integer(value: Any, label: str) -> int:
    if type(value) is not int:
        raise AnchorValidationError(f"{label} must be an integer")
    return value


def canonical_terminal_count(
    proof: dict[str, Any], field: str, *, positive: bool = False
) -> int:
    raw_value = proof.get(field)
    if not isinstance(raw_value, str):
        raise AnchorValidationError(
            f"raw capture worker terminal proof {field} must be canonical decimal"
        )
    try:
        value = int(raw_value, 10)
    except ValueError as exc:
        raise AnchorValidationError(
            f"raw capture worker terminal proof {field} must be canonical decimal"
        ) from exc
    if raw_value != str(value) or value < 0 or (positive and value <= 0):
        raise AnchorValidationError(
            f"raw capture worker terminal proof {field} must be "
            f"{'positive ' if positive else 'nonnegative '}canonical decimal"
        )
    return value


def normalized_host_boot_id(value: Any, label: str = "host boot id") -> str:
    if not isinstance(value, str):
        raise AnchorValidationError(f"{label} must be a lowercase canonical UUID")
    try:
        parsed = uuid.UUID(value)
    except (AttributeError, ValueError) as exc:
        raise AnchorValidationError(
            f"{label} must be a lowercase canonical UUID"
        ) from exc
    if str(parsed) != value or parsed.int == 0:
        raise AnchorValidationError(f"{label} must be a lowercase canonical UUID")
    return value


def parse_utc_timestamp(value: Any, label: str) -> datetime:
    text = nonempty_text(value, label)
    try:
        parsed = datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError as exc:
        raise AnchorValidationError(f"{label} is not an ISO-8601 timestamp") from exc
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise AnchorValidationError(f"{label} must include a UTC offset")
    if parsed.utcoffset().total_seconds() != 0:
        raise AnchorValidationError(f"{label} must be UTC")
    return parsed.astimezone(timezone.utc)


def parse_sidecar(data: bytes, expected_basename: str) -> str:
    try:
        fields = data.decode("utf-8").strip().split()
    except UnicodeDecodeError as exc:
        raise AnchorValidationError("software-zero checksum sidecar is not UTF-8") from exc
    if len(fields) != 2 or fields[1] != expected_basename:
        raise AnchorValidationError("software-zero checksum sidecar is malformed")
    return normalized_sha256(fields[0], "software-zero sidecar")


def validate_zero(document: dict[str, Any]) -> None:
    if document.get("schema") != ZERO_SCHEMA:
        raise AnchorValidationError("persistent software-zero schema mismatch")
    if document.get("reference_name") != REFERENCE_NAME:
        raise AnchorValidationError("persistent software-zero reference mismatch")
    motors = document.get("motors")
    if not isinstance(motors, dict) or set(motors) != EXPECTED_MOTORS:
        raise AnchorValidationError("persistent software-zero motor set mismatch")
    for name, record in motors.items():
        if not isinstance(record, dict):
            raise AnchorValidationError(f"persistent software-zero {name} record invalid")
        finite_number(record.get("raw_position_rad"), f"{name} persistent raw position")
    mapping = document.get("mapping")
    if not isinstance(mapping, dict):
        raise AnchorValidationError("persistent software-zero mapping is missing")
    ratio = finite_number(mapping.get("gear_ratio"), "persistent gear ratio")
    if not math.isclose(ratio, GEAR_RATIO, rel_tol=0.0, abs_tol=1e-9):
        raise AnchorValidationError("persistent software-zero gear ratio mismatch")
    signs = mapping.get("signs")
    if not isinstance(signs, dict) or any(
        signs.get(name) != sign for name, sign in J2_SIGNS.items()
    ):
        raise AnchorValidationError("persistent software-zero J2 signs mismatch")
    writes = document.get("writes")
    if not isinstance(writes, dict) or any(
        writes.get(field) is not False
        for field in (
            "motor_internal_zero_modified",
            "rid_written",
            "flash_or_eeprom_written",
        )
    ):
        raise AnchorValidationError("persistent software-zero records a forbidden write")


def validate_hints(document: dict[str, Any]) -> None:
    if document.get("schema") != HINT_SCHEMA:
        raise AnchorValidationError("recovery-hint schema mismatch")
    if document.get("reference_name") != REFERENCE_NAME:
        raise AnchorValidationError("recovery-hint reference mismatch")
    motors = document.get("motors")
    if not isinstance(motors, dict) or not set(J2_SIGNS).issubset(motors):
        raise AnchorValidationError("recovery hints are missing J2 motors")
    for name in J2_SIGNS:
        record = motors.get(name)
        if not isinstance(record, dict):
            raise AnchorValidationError(f"recovery-hint {name} record invalid")
        finite_number(record.get("logical_position_rad"), f"{name} recovery hint")


def validate_initial_pose(document: dict[str, Any], parent_zero_sha256: str) -> None:
    if document.get("有效") is not True:
        raise AnchorValidationError("initial pose is not marked valid")
    expected_session = f"persistent:{parent_zero_sha256[:16]}"
    if document.get("会话标识") != expected_session:
        raise AnchorValidationError("initial pose is bound to another software-zero hash")
    positions = document.get("关节位置_弧度")
    expected_joints = {f"J{index}" for index in range(1, 7)}
    if not isinstance(positions, dict) or set(positions) != expected_joints:
        raise AnchorValidationError("initial pose joint set mismatch")
    for name, value in positions.items():
        finite_number(value, f"initial pose {name}")


def normalized_single_turn_phase(raw_position_rad: float) -> float:
    phase = (raw_position_rad + math.pi) % TWO_PI - math.pi
    return 0.0 if phase == -0.0 else phase


def validate_capture(document: dict[str, Any]) -> dict[str, Any]:
    if document.get("schema") != CAPTURE_SCHEMA:
        raise AnchorValidationError("raw capture-statistics schema mismatch")
    capture_id = nonempty_text(document.get("capture_id"), "capture_id")
    power_session_id = nonempty_text(
        document.get("power_session_id"), "power_session_id"
    )
    recorded_at = parse_utc_timestamp(document.get("recorded_at_utc"), "recorded_at_utc")
    host_boot_id = normalized_host_boot_id(
        document.get("host_boot_id"), "raw capture host boot id"
    )
    recorded_boottime_ns = positive_integer(
        document.get("recorded_boottime_ns"), "raw capture recorded CLOCK_BOOTTIME"
    )
    if document.get("status") != "PASS":
        raise AnchorValidationError("raw capture statistics are not marked PASS")
    if document.get("physical_power_state_during_capture") != "24V_ON":
        raise AnchorValidationError("raw capture is not bound to a 24V-on session")
    packet_count = document.get("packet_count")
    if type(packet_count) is not int or packet_count < MIN_SAMPLE_COUNT:
        raise AnchorValidationError(
            f"raw capture requires at least {MIN_SAMPLE_COUNT} packets"
        )
    coverage = finite_number(document.get("source_coverage_s"), "source coverage")
    if coverage < MIN_SOURCE_COVERAGE_S:
        raise AnchorValidationError("raw capture source coverage is too short")

    safety = document.get("safety")
    if not isinstance(safety, dict):
        raise AnchorValidationError("raw capture safety evidence is missing")
    exact_safety = {
        "execution_policy": "BRAKE_ONLY",
        "command_rx_enabled": False,
        "foc_tx_attempt_count": 0,
        "active_or_hold_commands_sent": 0,
        "communication_failure_packets": 0,
        "merror_nonzero_packets": 0,
        "motor_internal_zero_modified": False,
        "rid_written": False,
        "flash_or_eeprom_written": False,
        "startup_brake_prime_passed": True,
    }
    for field, expected in exact_safety.items():
        actual = safety.get(field)
        if type(actual) is not type(expected) or actual != expected:
            raise AnchorValidationError(
                f"raw capture safety field {field} is not {expected!r}"
            )
    if safety.get("all_controller_modes") != ["brake"]:
        raise AnchorValidationError("raw capture contains a non-BRAKE mode")

    raw_safety_summary = document.get("raw_safety_summary")
    if not isinstance(raw_safety_summary, dict):
        raise AnchorValidationError("raw capture safety summary is missing")

    startup_prime = raw_safety_summary.get("startup_brake_prime")
    startup_prime_fields = {
        "state",
        "maximum_invalid_prefix_pairs",
        "invalid_prefix_pairs",
        "required_healthy_pairs",
        "healthy_pairs",
        "attempted_pairs",
        "tx_attempt_count",
        "elapsed_ns",
        "feedback_published_during_prime",
        "window_reopens_after_first_healthy_pair",
    }
    if not isinstance(startup_prime, dict) or set(startup_prime) != startup_prime_fields:
        raise AnchorValidationError(
            "raw capture startup BRAKE prime evidence has an invalid field set"
        )
    if startup_prime.get("state") != "PASS":
        raise AnchorValidationError("raw capture startup BRAKE prime is not PASS")
    if startup_prime.get("feedback_published_during_prime") is not False:
        raise AnchorValidationError(
            "raw capture published feedback during startup BRAKE prime"
        )
    if startup_prime.get("window_reopens_after_first_healthy_pair") is not False:
        raise AnchorValidationError(
            "raw capture startup BRAKE prime qualification window reopened"
        )
    maximum_invalid_prefix_pairs = exact_integer(
        startup_prime.get("maximum_invalid_prefix_pairs"),
        "startup BRAKE prime maximum invalid-prefix pairs",
    )
    invalid_prefix_pairs = exact_integer(
        startup_prime.get("invalid_prefix_pairs"),
        "startup BRAKE prime invalid-prefix pairs",
    )
    required_healthy_pairs = exact_integer(
        startup_prime.get("required_healthy_pairs"),
        "startup BRAKE prime required healthy pairs",
    )
    healthy_pairs = exact_integer(
        startup_prime.get("healthy_pairs"),
        "startup BRAKE prime healthy pairs",
    )
    attempted_pairs = exact_integer(
        startup_prime.get("attempted_pairs"),
        "startup BRAKE prime attempted pairs",
    )
    startup_prime_tx_attempt_count = exact_integer(
        startup_prime.get("tx_attempt_count"),
        "startup BRAKE prime transmit-attempt count",
    )
    startup_prime_elapsed_ns = exact_integer(
        startup_prime.get("elapsed_ns"),
        "startup BRAKE prime elapsed time",
    )
    if maximum_invalid_prefix_pairs != STARTUP_PRIME_MAX_INVALID_PREFIX_PAIRS:
        raise AnchorValidationError(
            "raw capture startup BRAKE prime invalid-prefix limit mismatch"
        )
    if not 0 <= invalid_prefix_pairs <= maximum_invalid_prefix_pairs:
        raise AnchorValidationError(
            "raw capture startup BRAKE prime invalid-prefix count is out of bounds"
        )
    if required_healthy_pairs != STARTUP_PRIME_REQUIRED_HEALTHY_PAIRS:
        raise AnchorValidationError(
            "raw capture startup BRAKE prime healthy-pair requirement mismatch"
        )
    if healthy_pairs != required_healthy_pairs:
        raise AnchorValidationError(
            "raw capture startup BRAKE prime healthy qualification is incomplete"
        )
    if attempted_pairs != invalid_prefix_pairs + healthy_pairs:
        raise AnchorValidationError(
            "raw capture startup BRAKE prime pair counters disagree"
        )
    if startup_prime_tx_attempt_count != 2 * attempted_pairs:
        raise AnchorValidationError(
            "raw capture startup BRAKE prime transmit count disagrees"
        )
    if not 1 <= startup_prime_elapsed_ns <= STARTUP_PRIME_MAXIMUM_ELAPSED_NS:
        raise AnchorValidationError(
            "raw capture startup BRAKE prime elapsed time is out of bounds"
        )
    safety_invalid_prefix_pairs = exact_integer(
        safety.get("startup_invalid_prefix_pairs"),
        "raw capture safety startup invalid-prefix pairs",
    )
    if safety_invalid_prefix_pairs != invalid_prefix_pairs:
        raise AnchorValidationError(
            "raw capture safety and startup BRAKE prime counts disagree"
        )

    phase_tx_accounting = raw_safety_summary.get("phase_tx_accounting")
    phase_tx_fields = {
        "completed_cycles",
        "startup_prime_tx_attempt_count",
        "control_loop_tx_attempt_count",
        "final_brake_tx_attempt_count",
        "aggregate_tx_attempt_count",
    }
    if (
        not isinstance(phase_tx_accounting, dict)
        or set(phase_tx_accounting) != phase_tx_fields
    ):
        raise AnchorValidationError(
            "raw capture phase transmit accounting has an invalid field set"
        )
    phase_tx = {
        field: exact_integer(
            phase_tx_accounting.get(field),
            f"raw capture phase transmit accounting {field}",
        )
        for field in phase_tx_fields
    }
    if phase_tx["completed_cycles"] != packet_count:
        raise AnchorValidationError(
            "raw capture completed cycles do not match packet count"
        )
    if (
        phase_tx["startup_prime_tx_attempt_count"]
        != startup_prime_tx_attempt_count
    ):
        raise AnchorValidationError(
            "raw capture prime transmit counts disagree across phases"
        )
    if phase_tx["control_loop_tx_attempt_count"] != 2 * packet_count:
        raise AnchorValidationError(
            "raw capture control-loop transmit count does not match packet count"
        )
    if phase_tx["final_brake_tx_attempt_count"] != FINAL_BRAKE_TX_ATTEMPT_COUNT:
        raise AnchorValidationError(
            "raw capture final BRAKE transmit count is not exactly 40"
        )
    expected_aggregate_tx_attempt_count = (
        phase_tx["startup_prime_tx_attempt_count"]
        + phase_tx["control_loop_tx_attempt_count"]
        + phase_tx["final_brake_tx_attempt_count"]
    )
    if (
        phase_tx["aggregate_tx_attempt_count"]
        != expected_aggregate_tx_attempt_count
    ):
        raise AnchorValidationError(
            "raw capture aggregate transmit count is not the strict phase sum"
        )

    worker = raw_safety_summary.get("worker")
    if not isinstance(worker, dict):
        raise AnchorValidationError("raw capture worker evidence is missing")
    worker_path = nonempty_text(worker.get("path"), "raw capture worker path")
    raw_worker_sha256 = worker.get("sha256")
    worker_sha256 = normalized_sha256(
        raw_worker_sha256, "raw capture worker"
    )
    if raw_worker_sha256 != worker_sha256:
        raise AnchorValidationError(
            "raw capture worker SHA-256 must be lowercase canonical hex"
        )
    if type(worker.get("returncode")) is not int or worker.get("returncode") != 0:
        raise AnchorValidationError("raw capture worker did not exit normally")
    if worker.get("stop_escalation") != "none":
        raise AnchorValidationError("raw capture worker required stop escalation")
    if worker.get("forced_kill") is not False:
        raise AnchorValidationError("raw capture worker was forcibly killed")

    terminal_proof = worker.get("terminal_proof")
    exact_terminal_text = {
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
        "J2_STARTUP_BRAKE_PRIME_MAX_INVALID_PREFIX_PAIRS": str(
            STARTUP_PRIME_MAX_INVALID_PREFIX_PAIRS
        ),
        "J2_STARTUP_BRAKE_PRIME_REQUIRED_HEALTHY_PAIRS": str(
            STARTUP_PRIME_REQUIRED_HEALTHY_PAIRS
        ),
    }
    terminal_count_fields = {
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
    }
    if (
        not isinstance(terminal_proof, dict)
        or set(terminal_proof) != set(exact_terminal_text) | terminal_count_fields
    ):
        raise AnchorValidationError(
            "raw capture worker terminal proof has an invalid field set"
        )
    for field, expected in exact_terminal_text.items():
        if terminal_proof.get(field) != expected:
            raise AnchorValidationError(
                f"raw capture worker terminal proof {field} is not {expected!r}"
            )
    terminal_counts = {
        field: canonical_terminal_count(
            terminal_proof,
            field,
            positive=(field != "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS"),
        )
        for field in terminal_count_fields
    }
    terminal_cross_bindings = {
        "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS": invalid_prefix_pairs,
        "J2_STARTUP_BRAKE_PRIME_HEALTHY_PAIRS": healthy_pairs,
        "J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS": attempted_pairs,
        "J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT": (
            startup_prime_tx_attempt_count
        ),
        "J2_STARTUP_BRAKE_PRIME_ELAPSED_NS": startup_prime_elapsed_ns,
        "COMPLETED_CYCLES": phase_tx["completed_cycles"],
        "FINAL_BRAKE_TX_ATTEMPT_COUNT": phase_tx[
            "final_brake_tx_attempt_count"
        ],
        "TX_ATTEMPT_TOTAL": phase_tx["aggregate_tx_attempt_count"],
        "BRAKE_TX_ATTEMPT_COUNT": phase_tx["aggregate_tx_attempt_count"],
        "SERIAL_SEND_CALL_COUNT": phase_tx["aggregate_tx_attempt_count"],
    }
    for field, expected in terminal_cross_bindings.items():
        if terminal_counts[field] != expected:
            raise AnchorValidationError(
                f"raw capture worker terminal proof {field} is not bound "
                "to the structured capture evidence"
            )

    motors = document.get("motors")
    if not isinstance(motors, dict) or set(motors) != set(J2_SIGNS):
        raise AnchorValidationError("raw capture must contain exactly J2A and J2B")
    normalized_motors: dict[str, Any] = {}
    for name in J2_SIGNS:
        record = motors.get(name)
        if not isinstance(record, dict):
            raise AnchorValidationError(f"raw capture {name} record invalid")
        sample_count = record.get("sample_count")
        if type(sample_count) is not int or sample_count != packet_count:
            raise AnchorValidationError(f"raw capture {name} sample count mismatch")
        raw = record.get("unwrapped_raw_position_rad")
        if not isinstance(raw, dict):
            raise AnchorValidationError(f"raw capture {name} statistics are missing")
        mean = finite_number(raw.get("mean"), f"{name} raw mean")
        minimum = finite_number(raw.get("minimum"), f"{name} raw minimum")
        maximum = finite_number(raw.get("maximum"), f"{name} raw maximum")
        span = finite_number(raw.get("span"), f"{name} raw span")
        standard_deviation = finite_number(
            raw.get("standard_deviation"), f"{name} raw standard deviation"
        )
        if any(
            abs(value) > MAX_PROTOCOL_POSITION_RAD
            for value in (mean, minimum, maximum)
        ):
            raise AnchorValidationError(f"raw capture {name} exceeds protocol range")
        if not minimum <= mean <= maximum:
            raise AnchorValidationError(f"raw capture {name} mean is outside its range")
        calculated_span = maximum - minimum
        if span < 0.0 or not math.isclose(
            span,
            calculated_span,
            rel_tol=1e-9,
            abs_tol=1e-12,
        ):
            raise AnchorValidationError(f"raw capture {name} span is inconsistent")
        if span > MAX_RAW_SPAN_RAD:
            raise AnchorValidationError(f"raw capture {name} is not stationary")
        if not 0.0 <= standard_deviation <= span + 1e-12:
            raise AnchorValidationError(
                f"raw capture {name} standard deviation is inconsistent"
            )
        normalized_motors[name] = {
            "sample_count": sample_count,
            "unwrapped_raw_position_rad": {
                "mean": mean,
                "minimum": minimum,
                "maximum": maximum,
                "span": span,
                "standard_deviation": standard_deviation,
            },
            "single_turn_phase_rad": normalized_single_turn_phase(mean),
        }
    return {
        "capture_id": capture_id,
        "power_session_id": power_session_id,
        "recorded_at_utc": recorded_at.isoformat().replace("+00:00", "Z"),
        "host_boot_id": host_boot_id,
        "recorded_boottime_ns": recorded_boottime_ns,
        "worker": {
            "path": worker_path,
            "sha256": worker_sha256,
            "terminal_proof": dict(terminal_proof),
        },
        "expected_worker_sha256": worker_sha256,
        "startup_brake_prime": {
            "state": "PASS",
            "maximum_invalid_prefix_pairs": maximum_invalid_prefix_pairs,
            "invalid_prefix_pairs": invalid_prefix_pairs,
            "required_healthy_pairs": required_healthy_pairs,
            "healthy_pairs": healthy_pairs,
            "attempted_pairs": attempted_pairs,
            "tx_attempt_count": startup_prime_tx_attempt_count,
            "elapsed_ns": startup_prime_elapsed_ns,
            "feedback_published_during_prime": False,
            "window_reopens_after_first_healthy_pair": False,
        },
        "phase_tx_accounting": {
            "completed_cycles": phase_tx["completed_cycles"],
            "startup_prime_tx_attempt_count": phase_tx[
                "startup_prime_tx_attempt_count"
            ],
            "control_loop_tx_attempt_count": phase_tx[
                "control_loop_tx_attempt_count"
            ],
            "final_brake_tx_attempt_count": phase_tx[
                "final_brake_tx_attempt_count"
            ],
            "aggregate_tx_attempt_count": phase_tx[
                "aggregate_tx_attempt_count"
            ],
        },
        "packet_count": packet_count,
        "source_coverage_s": coverage,
        "motors": normalized_motors,
    }


def validate_operator_confirmation(
    *,
    evidence_id: str,
    confirmed_at_utc: str,
    power_session_id: str,
    physical_confirmation: str,
    capture_power_session_id: str,
) -> dict[str, Any]:
    normalized_evidence_id = nonempty_text(evidence_id, "operator evidence id")
    normalized_session_id = nonempty_text(power_session_id, "operator power session id")
    confirmed_at = parse_utc_timestamp(confirmed_at_utc, "operator confirmed_at_utc")
    if normalized_session_id != capture_power_session_id:
        raise AnchorValidationError(
            "operator confirmation and raw capture power-session ids differ"
        )
    if physical_confirmation != PHYSICAL_GATE:
        raise AnchorValidationError("complete physical-state confirmation gate is missing")
    return {
        "evidence_id": normalized_evidence_id,
        "confirmed_at_utc": confirmed_at.isoformat().replace("+00:00", "Z"),
        "power_session_id": normalized_session_id,
        "vertical_initialization_pose": True,
        "support_reliable": True,
        "arm_not_moved": True,
        "not_at_mechanical_limit": True,
        "confirmation_gate": PHYSICAL_GATE,
    }


def build_anchor(
    *,
    zero: dict[str, Any],
    parent_zero_sha256: str,
    parent_sidecar_sha256: str,
    recovery_hint_sha256: str,
    initial_pose_sha256: str,
    capture_source_path: str,
    capture_file_sha256: str,
    capture: dict[str, Any],
    operator_confirmation: dict[str, Any],
) -> dict[str, Any]:
    identity_material = {
        "schema": ANCHOR_SCHEMA,
        "parent_zero_sha256": parent_zero_sha256,
        "capture_file_sha256": capture_file_sha256,
        "capture_id": capture["capture_id"],
        "power_session_id": capture["power_session_id"],
        "operator_evidence_id": operator_confirmation["evidence_id"],
    }
    identity_digest = sha256_bytes(canonical_bytes(identity_material))
    anchor_id = f"j2-vertical-session-phase-v1-{identity_digest[:16]}"
    parent_raw = {
        name: float(zero["motors"][name]["raw_position_rad"])
        for name in J2_SIGNS
    }
    motors = {
        name: {
            "session_reference_raw_rad": capture["motors"][name][
                "unwrapped_raw_position_rad"
            ]["mean"],
            "logical_position_rad": 0.0,
            "sample_count": capture["motors"][name]["sample_count"],
            "sample_span_raw_rad": capture["motors"][name][
                "unwrapped_raw_position_rad"
            ]["span"],
            "sign": J2_SIGNS[name],
            "gear_ratio": GEAR_RATIO,
            "single_turn_phase_rad": capture["motors"][name][
                "single_turn_phase_rad"
            ],
        }
        for name in J2_SIGNS
    }
    return {
        "schema": ANCHOR_SCHEMA,
        "anchor_version": 1,
        "anchor_id": anchor_id,
        "parent_persistent_zero_sha256": parent_zero_sha256,
        "reference_name": REFERENCE_NAME,
        "source_evidence": {
            "path": capture_source_path,
            "sha256": capture_file_sha256,
        },
        "operator_confirmation": operator_confirmation,
        "writes": {
            "motor_internal_zero_modified": False,
            "rid_written": False,
            "flash_or_eeprom_written": False,
        },
        "motors": motors,
        "purpose": (
            "Immutable J2 vertical-session raw phase evidence for separately "
            "reviewed boot alignment"
        ),
        "control_authority": {
            "is_software_zero": False,
            "authorizes_active_control": False,
            "authorizes_motor_internal_write": False,
        },
        "parent_persistent_software_zero": {
            "schema": ZERO_SCHEMA,
            "reference_name": REFERENCE_NAME,
            "json_sha256": parent_zero_sha256,
            "checksum_sidecar_sha256": parent_sidecar_sha256,
            "J2_raw_position_rad": parent_raw,
        },
        "preserved_inputs": {
            "recovery_branch_hints_sha256": recovery_hint_sha256,
            "initial_pose_sha256": initial_pose_sha256,
            "overwritten_or_deleted": False,
        },
        "raw_capture": {
            "source_schema": CAPTURE_SCHEMA,
            "source_file_sha256": capture_file_sha256,
            **capture,
        },
        "derivation": {
            "single_turn_phase_formula": "((raw_mean + pi) mod (2*pi)) - pi",
            "gear_ratio": GEAR_RATIO,
            "J2_signs": dict(J2_SIGNS),
            "integer_turn_branch_inferred": False,
            "persistent_zero_rebased": False,
            "recovery_hints_updated": False,
            "initial_pose_updated": False,
        },
        "offline_tool_safety": {
            "hardware_accessed_by_this_tool": False,
            "existing_calibration_file_modified": False,
        },
    }


def versioned_output_path(anchor_directory: Path, anchor: dict[str, Any]) -> Path:
    recorded_at = parse_utc_timestamp(
        anchor["raw_capture"]["recorded_at_utc"], "capture recorded_at_utc"
    )
    timestamp = recorded_at.strftime("%Y%m%dT%H%M%SZ")
    suffix = anchor["anchor_id"].rsplit("-", 1)[-1]
    filename = f"j2_power_session_reference_v1_{timestamp}_{suffix}.json"
    return anchor_directory.resolve() / filename


def preserve_reference_if_requested(anchor: dict[str, Any], args: argparse.Namespace,
                                    capture_validator=validate_capture) -> Path | None:
    source_arg = getattr(args, "preserve_reference_file", None)
    expected_sha = getattr(args, "expected_preserve_reference_sha256", None)
    if bool(source_arg) != bool(expected_sha):
        raise AnchorValidationError("preserve-reference requires both source file and SHA256")
    if source_arg is None:
        return None
    source_path = source_arg.resolve(strict=True)
    source, data = read_json(source_path)
    source_sha = normalized_sha256(expected_sha, "preserved reference")
    if sha256_bytes(data) != source_sha:
        raise AnchorValidationError("preserved reference SHA256 mismatch")
    evidence, evidence_data = read_json(Path(source["source_evidence"]["path"]))
    normalized = capture_validator(evidence)
    if source["raw_capture"] != {
        "source_schema": evidence["schema"],
        "source_file_sha256": sha256_bytes(evidence_data), **normalized,
    }:
        raise AnchorValidationError("preserved original capture does not match validated evidence")
    anchor["preserved_reference"] = {
        "schema": PRESERVED_SESSION_REFERENCE_SCHEMA,
        "source_path": str(source_path), "source_sha256": source_sha,
    }
    for name, record in anchor["motors"].items():
        original = source["motors"][name]
        for field in ("session_reference_raw_rad", "logical_position_rad", "sign", "gear_ratio"):
            record[field] = original[field]
        raw = anchor["raw_capture"]["motors"][name]["unwrapped_raw_position_rad"]["mean"]
        reference = session_reference_for_raw(
            raw, record["session_reference_raw_rad"], record["logical_position_rad"], record["sign"]
        )
        record["startup_logical_position_rad"] = record["sign"] * (raw - reference) / GEAR_RATIO
    validate_preserved_session_reference(anchor)
    anchor["anchor_id"] = anchor["anchor_id"].rsplit("-", 1)[0] + "-" + sha256_bytes(
        canonical_bytes({"anchor_id": anchor["anchor_id"], "preserved_source": source_sha})
    )[:16]
    anchor["derivation"]["startup_position_source"] = "FRESH_RAW_WITH_UNCHANGED_SESSION_GEOMETRY"
    return source_path


def current_host_boot_id(path: Path = HOST_BOOT_ID_PATH) -> str:
    try:
        value = path.read_text(encoding="ascii").strip()
    except (OSError, UnicodeError) as exc:
        raise AnchorValidationError(f"host boot id is unavailable: {exc}") from exc
    return normalized_host_boot_id(value)


def current_boottime_ns(*, clock_gettime_ns=None, clock_id=None) -> int:
    reader = (
        getattr(time, "clock_gettime_ns", None)
        if clock_gettime_ns is None
        else clock_gettime_ns
    )
    boot_clock = (
        getattr(time, "CLOCK_BOOTTIME", None) if clock_id is None else clock_id
    )
    if not callable(reader) or type(boot_clock) is not int:
        raise AnchorValidationError("CLOCK_BOOTTIME is unavailable on this host")
    try:
        value = reader(boot_clock)
    except (OSError, ValueError) as exc:
        raise AnchorValidationError(f"CLOCK_BOOTTIME read failed: {exc}") from exc
    return positive_integer(value, "issued CLOCK_BOOTTIME")


def normalized_issued_at_utc(value: datetime | None) -> datetime:
    issued_at = datetime.now(timezone.utc) if value is None else value
    if (
        not isinstance(issued_at, datetime)
        or issued_at.tzinfo is None
        or issued_at.utcoffset() is None
    ):
        raise AnchorValidationError("issued_at_utc must be timezone-aware")
    return issued_at.astimezone(timezone.utc)


def launch_permit_id(anchor: dict[str, Any], anchor_sha256: str) -> str:
    material = {
        "schema": LAUNCH_PERMIT_SCHEMA,
        "anchor_id": anchor["anchor_id"],
        "anchor_sha256": normalized_sha256(anchor_sha256, "session reference"),
        "parent_persistent_zero_sha256": anchor[
            "parent_persistent_zero_sha256"
        ],
        "power_session_id": anchor["raw_capture"]["power_session_id"],
        "host_boot_id": anchor["raw_capture"]["host_boot_id"],
        "worker_sha256": anchor["raw_capture"]["worker"]["sha256"],
    }
    digest = sha256_bytes(canonical_bytes(material))
    return digest


def launch_permit_paths(
    launch_permit_directory: Path, permit_id: str
) -> dict[str, Path]:
    pending_directory = launch_permit_directory.resolve()
    if pending_directory.name != "pending":
        raise AnchorValidationError(
            "launch-permit directory must be the pending lifecycle directory"
        )
    filename = f"{permit_id}.json"
    return {
        "pending": pending_directory / filename,
        "inflight": pending_directory.parent / "inflight" / filename,
        "spent": pending_directory.parent / "spent" / filename,
    }


def build_launch_permit(
    *,
    anchor: dict[str, Any],
    anchor_path: Path,
    anchor_sha256: str,
    permit_paths: dict[str, Path],
    issued_at_utc: datetime,
    host_boot_id: str,
    issued_boottime_ns: int,
) -> dict[str, Any]:
    normalized_boot_id = normalized_host_boot_id(host_boot_id)
    capture_boot_id = normalized_host_boot_id(
        anchor["raw_capture"]["host_boot_id"], "raw capture host boot id"
    )
    if normalized_boot_id != capture_boot_id:
        raise AnchorValidationError(
            "current host boot id differs from raw capture host boot id"
        )
    issued_boot_ns = positive_integer(
        issued_boottime_ns, "issued CLOCK_BOOTTIME"
    )
    recorded_boot_ns = positive_integer(
        anchor["raw_capture"]["recorded_boottime_ns"],
        "raw capture recorded CLOCK_BOOTTIME",
    )
    capture_age_ns = issued_boot_ns - recorded_boot_ns
    if capture_age_ns < 0:
        raise AnchorValidationError(
            "issued CLOCK_BOOTTIME precedes raw capture CLOCK_BOOTTIME"
        )
    if capture_age_ns > MAX_CAPTURE_TO_PERMIT_NS:
        raise AnchorValidationError(
            "raw capture is older than the 300-second launch-permit window"
        )

    issued_at = normalized_issued_at_utc(issued_at_utc)
    expires_at = issued_at + timedelta(seconds=LAUNCH_PERMIT_TTL_SECONDS)
    anchor_path_text = str(anchor_path.resolve())
    canonical_paths = {
        name: str(path.resolve()) for name, path in permit_paths.items()
    }
    anchor_digest = normalized_sha256(anchor_sha256, "session reference")
    worker = anchor["raw_capture"]["worker"]
    worker_path = nonempty_text(worker.get("path"), "worker path")
    worker_sha256 = normalized_sha256(worker.get("sha256"), "worker")
    permit_id = launch_permit_id(anchor, anchor_digest)
    return {
        "schema": LAUNCH_PERMIT_SCHEMA,
        "permit_version": 1,
        "permit_id": permit_id,
        "scope": "j2",
        "single_use": True,
        "power_session_id": anchor["raw_capture"]["power_session_id"],
        "host_boot_id": normalized_boot_id,
        "issued_at_utc": issued_at.isoformat().replace("+00:00", "Z"),
        "expires_at_utc": expires_at.isoformat().replace("+00:00", "Z"),
        "issued_unix_s": int(issued_at.timestamp()),
        "expires_unix_s": int(expires_at.timestamp()),
        "issued_boottime_ns": issued_boot_ns,
        "expires_boottime_ns": issued_boot_ns + LAUNCH_PERMIT_TTL_NS,
        "ttl_seconds": LAUNCH_PERMIT_TTL_SECONDS,
        "parent_persistent_zero_sha256": anchor[
            "parent_persistent_zero_sha256"
        ],
        "source_capture": {
            "host_boot_id": capture_boot_id,
            "recorded_boottime_ns": recorded_boot_ns,
        },
        "session_reference": {
            "path": anchor_path_text,
            "sha256": anchor_digest,
        },
        "worker": {
            "path": worker_path,
            "sha256": worker_sha256,
        },
        "pending_path": canonical_paths["pending"],
        "inflight_path": canonical_paths["inflight"],
        "spent_path": canonical_paths["spent"],
        "serial": {
            "stable_by_id": STABLE_J2_SERIAL_BY_ID,
            "bus": "j2",
            "motor_ids": [0, 1],
            "gear_ratio": GEAR_RATIO,
            "signs": dict(J2_SIGNS),
        },
        "startup_recheck": {
            "minimum_brake_frames": STARTUP_RECHECK_MINIMUM_BRAKE_FRAMES,
            "max_raw_phase_delta_rad": STARTUP_RECHECK_MAX_RAW_PHASE_DELTA_RAD,
            "max_raw_span_rad": STARTUP_RECHECK_MAX_RAW_SPAN_RAD,
        },
    }


def path_lexists(path: Path) -> bool:
    return os.path.lexists(os.fspath(path))


def ensure_private_directory(path: Path) -> None:
    """Create output-directory components privately without touching ancestors."""
    target = path.resolve()
    missing: list[Path] = []
    cursor = target
    while not path_lexists(cursor):
        missing.append(cursor)
        parent = cursor.parent
        if parent == cursor:
            break
        cursor = parent
    if path_lexists(cursor) and not cursor.is_dir():
        raise AnchorValidationError(f"output parent is not a directory: {cursor}")
    for directory in reversed(missing):
        try:
            directory.mkdir(mode=0o700)
        except FileExistsError:
            if not directory.is_dir():
                raise AnchorValidationError(
                    f"output parent is not a directory: {directory}"
                )
        os.chmod(directory, 0o700)
    if not target.is_dir():
        raise AnchorValidationError(f"output parent is not a directory: {target}")
    os.chmod(target, 0o700)


def preflight_new_outputs(paths: list[Path]) -> None:
    for path in paths:
        if path_lexists(path):
            raise AnchorValidationError(
                f"refuse to overwrite existing immutable output: {path}"
            )
    canonical = [path.resolve() for path in paths]
    if len(set(canonical)) != len(canonical):
        raise AnchorValidationError("immutable output paths must be distinct")


def fsync_directory(path: Path) -> None:
    if os.name == "nt":
        return
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def atomic_write_new(path: Path, data: bytes) -> None:
    """Atomically publish a complete new file without ever replacing a target."""
    ensure_private_directory(path.parent)
    if path_lexists(path):
        raise AnchorValidationError(
            f"refuse to overwrite existing immutable output: {path}"
        )
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    published = False
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, 0o600)
        # A same-directory hard link is an atomic no-replace publication:
        # os.link raises FileExistsError if another process won the race.
        try:
            os.link(temporary, path)
        except FileExistsError as exc:
            raise AnchorValidationError(
                f"refuse to overwrite existing immutable output: {path}"
            ) from exc
        published = True
        fsync_directory(path.parent)
    finally:
        temporary.unlink(missing_ok=True)
        if published:
            fsync_directory(path.parent)
    if not published or path.read_bytes() != data:
        raise AnchorValidationError("post-write immutable output verification failed")


def _resolve_distinct_inputs(paths: list[Path]) -> list[Path]:
    resolved = [path.resolve(strict=True) for path in paths]
    if len(set(resolved)) != len(resolved):
        raise AnchorValidationError("input evidence paths must be distinct")
    return resolved


def run(
    args: argparse.Namespace,
    *,
    now_utc: datetime | None = None,
    host_boot_id: str | None = None,
    issued_boottime_ns: int | None = None,
) -> dict[str, Any]:
    defer_launch_permit = bool(
        getattr(args, "defer_launch_permit", False)
    )
    publish_deferred_launch_permit = bool(
        getattr(args, "publish_deferred_launch_permit", False)
    )
    if defer_launch_permit and publish_deferred_launch_permit:
        raise AnchorValidationError(
            "defer and publish-deferred launch-permit modes are mutually exclusive"
        )
    if (defer_launch_permit or publish_deferred_launch_permit) and not args.apply:
        raise AnchorValidationError(
            "deferred launch-permit modes require --apply"
        )
    zero_path, sidecar_path, hint_path, initial_pose_path, capture_path = (
        _resolve_distinct_inputs(
            [
                args.zero_file,
                args.zero_sha256_file,
                args.recovery_hint_file,
                args.initial_pose_file,
                args.capture_statistics_file,
            ]
        )
    )
    zero, zero_data = read_json(zero_path)
    hints, hint_data = read_json(hint_path)
    initial_pose, initial_pose_data = read_json(initial_pose_path)
    capture_document, capture_data = read_json(capture_path)
    sidecar_data = sidecar_path.read_bytes()

    fingerprints = {
        "zero": sha256_bytes(zero_data),
        "sidecar": sha256_bytes(sidecar_data),
        "hints": sha256_bytes(hint_data),
        "initial_pose": sha256_bytes(initial_pose_data),
        "capture": sha256_bytes(capture_data),
    }
    expected = {
        "zero": normalized_sha256(
            args.expected_parent_zero_sha256, "expected parent zero"
        ),
        "hints": normalized_sha256(
            args.expected_recovery_hint_sha256, "expected recovery hints"
        ),
        "initial_pose": normalized_sha256(
            args.expected_initial_pose_sha256, "expected initial pose"
        ),
        "capture": normalized_sha256(
            args.expected_capture_sha256, "expected capture"
        ),
    }
    for name in expected:
        if fingerprints[name] != expected[name]:
            raise AnchorValidationError(f"pinned {name} SHA-256 does not match")
    if parse_sidecar(sidecar_data, zero_path.name) != fingerprints["zero"]:
        raise AnchorValidationError("software-zero checksum sidecar verification failed")

    validate_zero(zero)
    validate_hints(hints)
    validate_initial_pose(initial_pose, fingerprints["zero"])
    capture = validate_capture(capture_document)
    operator_confirmation = validate_operator_confirmation(
        evidence_id=args.operator_evidence_id,
        confirmed_at_utc=args.operator_confirmed_at_utc,
        power_session_id=args.operator_power_session_id,
        physical_confirmation=args.physical_confirmation,
        capture_power_session_id=capture["power_session_id"],
    )
    anchor = build_anchor(
        zero=zero,
        parent_zero_sha256=fingerprints["zero"],
        parent_sidecar_sha256=fingerprints["sidecar"],
        recovery_hint_sha256=fingerprints["hints"],
        initial_pose_sha256=fingerprints["initial_pose"],
        capture_source_path=str(capture_path),
        capture_file_sha256=fingerprints["capture"],
        capture=capture,
        operator_confirmation=operator_confirmation,
    )
    preserved_source_path = preserve_reference_if_requested(anchor, args)
    output_path = versioned_output_path(args.anchor_directory, anchor)
    anchor_data = json_bytes(anchor)
    anchor_sha256 = sha256_bytes(anchor_data)
    permit_id = launch_permit_id(anchor, anchor_sha256)
    permit_paths = launch_permit_paths(args.launch_permit_directory, permit_id)
    issued_at = normalized_issued_at_utc(now_utc)
    current_boot_id = (
        current_host_boot_id()
        if host_boot_id is None
        else normalized_host_boot_id(host_boot_id)
    )
    current_boot_ns = (
        current_boottime_ns()
        if issued_boottime_ns is None
        else positive_integer(issued_boottime_ns, "issued CLOCK_BOOTTIME")
    )
    launch_permit = build_launch_permit(
        anchor=anchor,
        anchor_path=output_path,
        anchor_sha256=anchor_sha256,
        permit_paths=permit_paths,
        issued_at_utc=issued_at,
        host_boot_id=current_boot_id,
        issued_boottime_ns=current_boot_ns,
    )
    if launch_permit["permit_id"] != permit_id:
        raise AnchorValidationError("launch-permit identity derivation is inconsistent")
    permit_data = json_bytes(launch_permit)
    protected_paths = {
        zero_path,
        sidecar_path,
        hint_path,
        initial_pose_path,
        capture_path,
    }
    if preserved_source_path is not None:
        protected_paths.add(preserved_source_path)
    immutable_output_paths = [output_path, *permit_paths.values()]
    if any(path.resolve() in protected_paths for path in immutable_output_paths):
        raise AnchorValidationError("immutable output aliases protected input evidence")
    if len({path.resolve() for path in immutable_output_paths}) != len(
        immutable_output_paths
    ):
        raise AnchorValidationError("immutable output paths must be distinct")

    if args.apply:
        validate_preserved_session_reference(anchor)
        if args.confirm != APPLY_GATE:
            raise AnchorValidationError(f"apply requires --confirm {APPLY_GATE}")
        if publish_deferred_launch_permit:
            if (
                not output_path.is_file()
                or output_path.is_symlink()
                or output_path.read_bytes() != anchor_data
            ):
                raise AnchorValidationError(
                    "deferred launch-permit publication requires the exact immutable anchor"
                )
            preflight_new_outputs(list(permit_paths.values()))
        else:
            preflight_new_outputs(immutable_output_paths)
        before_publish = {
            "zero": sha256_file(zero_path),
            "sidecar": sha256_file(sidecar_path),
            "hints": sha256_file(hint_path),
            "initial_pose": sha256_file(initial_pose_path),
            "capture": sha256_file(capture_path),
        }
        if before_publish != fingerprints:
            raise AnchorValidationError("input evidence changed before anchor publication")
        for directory in {
            output_path.parent,
            permit_paths["pending"].parent,
            permit_paths["inflight"].parent,
            permit_paths["spent"].parent,
        }:
            ensure_private_directory(directory)
        if publish_deferred_launch_permit:
            if output_path.read_bytes() != anchor_data:
                raise AnchorValidationError(
                    "immutable anchor changed before deferred launch-permit publication"
                )
        else:
            preflight_new_outputs(immutable_output_paths)
            atomic_write_new(output_path, anchor_data)
        after_anchor_publish = {
            "zero": sha256_file(zero_path),
            "sidecar": sha256_file(sidecar_path),
            "hints": sha256_file(hint_path),
            "initial_pose": sha256_file(initial_pose_path),
            "capture": sha256_file(capture_path),
        }
        if after_anchor_publish != fingerprints:
            raise AnchorValidationError("protected input changed during anchor publication")
        if not defer_launch_permit:
            preflight_new_outputs(list(permit_paths.values()))
            atomic_write_new(permit_paths["pending"], permit_data)
        after_permit_publish = {
            "zero": sha256_file(zero_path),
            "sidecar": sha256_file(sidecar_path),
            "hints": sha256_file(hint_path),
            "initial_pose": sha256_file(initial_pose_path),
            "capture": sha256_file(capture_path),
        }
        if after_permit_publish != fingerprints:
            raise AnchorValidationError(
                "protected input changed during launch-permit publication"
            )
        validate_preserved_session_reference(anchor)

    return {
        "schema": "go-m8010-j2-vertical-session-phase-anchor-result/1.0",
        "applied": bool(args.apply),
        "output_path": str(output_path),
        "anchor_path": str(output_path),
        "anchor_id": anchor["anchor_id"],
        "anchor_sha256": anchor_sha256,
        "permit_path": str(permit_paths["pending"]),
        "permit_id": permit_id,
        "permit_sha256": sha256_bytes(permit_data),
        "anchor_published": bool(args.apply) and not publish_deferred_launch_permit,
        "launch_permit_published": bool(args.apply) and not defer_launch_permit,
        "deferred_launch_permit": defer_launch_permit,
        "parent_persistent_zero_sha256": fingerprints["zero"],
        "protected_inputs_unchanged": True,
        "hardware_accessed": False,
        "active_control_authorized": False,
        "anchor": anchor if not args.apply else None,
        "launch_permit": launch_permit if not args.apply else None,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zero-file", type=Path, required=True)
    parser.add_argument("--zero-sha256-file", type=Path, required=True)
    parser.add_argument("--expected-parent-zero-sha256", required=True)
    parser.add_argument("--recovery-hint-file", type=Path, required=True)
    parser.add_argument("--expected-recovery-hint-sha256", required=True)
    parser.add_argument("--initial-pose-file", type=Path, required=True)
    parser.add_argument("--expected-initial-pose-sha256", required=True)
    parser.add_argument("--capture-statistics-file", type=Path, required=True)
    parser.add_argument("--expected-capture-sha256", required=True)
    parser.add_argument("--preserve-reference-file", type=Path)
    parser.add_argument("--expected-preserve-reference-sha256")
    parser.add_argument("--operator-evidence-id", required=True)
    parser.add_argument("--operator-confirmed-at-utc", required=True)
    parser.add_argument("--operator-power-session-id", required=True)
    parser.add_argument("--physical-confirmation", required=True)
    parser.add_argument("--anchor-directory", type=Path, required=True)
    parser.add_argument("--launch-permit-directory", type=Path, required=True)
    parser.add_argument("--apply", action="store_true")
    deferred_group = parser.add_mutually_exclusive_group()
    deferred_group.add_argument("--defer-launch-permit", action="store_true")
    deferred_group.add_argument(
        "--publish-deferred-launch-permit", action="store_true"
    )
    parser.add_argument("--confirm", default="")
    return parser.parse_args(argv)


def main() -> int:
    try:
        result = run(parse_args())
    except (ValueError, KeyError, TypeError, OSError) as exc:
        print(
            json.dumps(
                {
                    "schema": "go-m8010-j2-vertical-session-phase-anchor-result/1.0",
                    "status": "BLOCKED",
                    "reason": str(exc),
                    "hardware_accessed": False,
                },
                ensure_ascii=False,
                indent=2,
            )
        )
        return 4
    print(json.dumps(result, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

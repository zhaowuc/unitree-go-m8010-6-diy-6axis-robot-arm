#!/usr/bin/env python3
"""Atomically make one verified stationary pose the whole-arm software zero.

The program is deliberately offline-only: it never opens a motor, serial, CAN,
USB, ROS, or UDP endpoint.  It consumes immutable BRAKE/DISABLED captures, then
updates the host-side persistent zero, recovery hints, initial pose and checksum
as one fail-closed transaction.  The checksum is committed last and originals
are copied to a new backup directory before any production file is replaced.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import math
import os
import re
import stat
import tempfile
import time
import uuid
from contextlib import ExitStack
from datetime import datetime
from pathlib import Path
from typing import Any

try:
    import fcntl
except ImportError:  # pragma: no cover - production apply runs on Linux.
    fcntl = None


_J2_ANCHOR_PATH = Path(__file__).with_name(
    "v15_30a_create_j2_vertical_session_phase_anchor.py"
)
_J2_ANCHOR_SPEC = importlib.util.spec_from_file_location(
    "v15_30a_j2_anchor_contract", _J2_ANCHOR_PATH
)
if _J2_ANCHOR_SPEC is None or _J2_ANCHOR_SPEC.loader is None:
    raise ImportError(f"cannot load J2 anchor contract from {_J2_ANCHOR_PATH}")
j2_anchor_contract = importlib.util.module_from_spec(_J2_ANCHOR_SPEC)
_J2_ANCHOR_SPEC.loader.exec_module(j2_anchor_contract)


APPLY_GATE = "V15_30A_REBASE_WHOLE_ARM_SOFTWARE_ZERO=YES"
ZERO_SCHEMA = "go-m8010-persistent-software-zero/1.0"
HINT_SCHEMA = "go-m8010-recovery-branch-hints/1.0"
J2_CAPTURE_SCHEMA = "go-m8010-j2-brake-raw-capture-statistics/1.0"
GO_AUX_CAPTURE_SCHEMA = "go-m8010-go-aux-brake-raw-capture-statistics/1.0"
J6_CAPTURE_SCHEMA = "go-m8010-j6-disabled-raw-capture-statistics/1.0"
REFERENCE_NAME = "PERSISTENT_SOFTWARE_ZERO_V1"
J2_PHYSICAL_GATE = (
    "24V_ON=YES;SUPPORT_RELIABLE=YES;J2_VERTICAL_INITIALIZATION_POSE=YES;"
    "ARM_NOT_MOVED=YES;ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES"
)
GO_AUX_PHYSICAL_GATE = (
    "24V_ON=YES;SUPPORT_RELIABLE=YES;WHOLE_ARM_VERTICAL_INITIALIZATION_POSE=YES;"
    "ARM_NOT_MOVED=YES;ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES"
)
J6_PHYSICAL_GATE = (
    "J6_24V_ON=YES;SUPPORT_RELIABLE=YES;"
    "WHOLE_ARM_VERTICAL_INITIALIZATION_POSE=YES;ARM_STATIONARY=YES;"
    "NOT_AT_MECHANICAL_LIMIT=YES"
)
EXPECTED_MOTORS = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
GO_HINT_MOTORS = ("J1", "J2A", "J2B", "J3", "J4", "J5")
GO_SIGNS = {"J1": 1, "J2A": -1, "J2B": 1, "J3": 1, "J4": 1, "J5": 1}
ALL_SIGNS = {**GO_SIGNS, "J6": -1}
GEAR_RATIO = 6.329999923706055
MIN_SAMPLE_COUNT = 500
MIN_CAPTURE_COVERAGE_S = 4.0
MAX_GO_CAPTURE_SPAN_RAD = GEAR_RATIO * math.radians(0.20)
MAX_J6_CAPTURE_SPAN_RAD = math.radians(0.20)
MAX_PROTOCOL_POSITION_RAD = 12.5
MAX_CAPTURE_BUNDLE_SPAN_NS = 120_000_000_000
MAX_CAPTURE_AGE_NS = 300_000_000_000
J2_STARTUP_PRIME_MAX_INVALID_PREFIX_PAIRS = 3
J2_STARTUP_PRIME_REQUIRED_HEALTHY_PAIRS = 5
J2_STARTUP_PRIME_MAXIMUM_ELAPSED_NS = 500_000_000
J2_FINAL_BRAKE_TX_ATTEMPT_COUNT = 40
EXPECTED_J6_USB = {
    "vid": "34b7",
    "pid": "6877",
    "serial": "EEE8D71AB573449FCAFE7B39BD222C75",
}
EXPECTED_J6_CHANNEL = 0
EXPECTED_J6_MOTOR_ID = 1
EXPECTED_J6_MASTER_ID = 0
LOWER_SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
CAPTURE_ID_PATTERNS = {
    "j2": re.compile(r"^j2-brake-raw-\d{8}T\d{12}Z-[0-9a-f]{16}$"),
    "go_aux": re.compile(r"^go-aux-brake-raw-\d{8}T\d{12}Z-[0-9a-f]{16}$"),
    "j6": re.compile(r"^j6-disabled-raw-\d{8}T\d{12}Z-[0-9a-f]{16}$"),
}
POWER_SESSION_ID_PATTERNS = {
    "j2": re.compile(r"^j2-power-session-\d{8}T\d{12}Z-[0-9a-f]{32}$"),
    "go_aux": re.compile(r"^go-aux-power-session-\d{8}T\d{12}Z-[0-9a-f]{32}$"),
    "j6": re.compile(r"^j6-power-session-\d{8}T\d{12}Z-[0-9a-f]{32}$"),
}
FORBIDDEN_COMMAND_COUNT_FIELDS = (
    "active_or_hold_commands_sent",
    "active_commands_sent",
    "hold_commands_sent",
    "foc_tx_attempt_count",
    "foc_serial_send_call_count",
    "other_mode_tx_attempt_count",
    "brake_only_guard_block_count",
    "motor_internal_zero_write_count",
    "rid_read_request_count",
    "rid_write_count",
    "flash_or_eeprom_write_count",
    "set_zero_command_count",
    "fc_enable_count",
    "fd_disable_count",
    "position_velocity_torque_command_count",
    "motion_command_count",
    "other_tx_attempt_count",
    "forbidden_tx_attempt_count",
)
LOCK_PATHS = (
    Path("/tmp/v15_30a_gui_prebuild.lock"),
    Path("/tmp/v15_30a_gui_supervisor.lock"),
    Path("/tmp/v15_30a_whole_arm_go_feedback.lock"),
    Path("/tmp/v15_30a_gui_j1.lock"),
    Path("/tmp/go_m8010_ftasqa6f_channel3.lock"),
    Path("/tmp/v15_30a_gui_j2.lock"),
    Path("/tmp/v15_23d_ft_j2_channel1.lock"),
    Path("/tmp/v15_30a_gui_j345.lock"),
    Path("/tmp/v15_23c_j3_bus.lock"),
    Path("/tmp/v15_22b_j4_bus.lock"),
    Path("/tmp/v15_22a_j5_bus.lock"),
    Path("/tmp/v15_30a_gui_j6.lock"),
)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def json_bytes(document: dict[str, Any]) -> bytes:
    return (json.dumps(document, ensure_ascii=False, indent=2) + "\n").encode("utf-8")


def finite_number(value: Any, label: str) -> float:
    if type(value) not in {int, float}:
        raise ValueError(f"{label} must be a number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{label} must be finite")
    return converted


def exact_int(value: Any, label: str) -> int:
    if type(value) is not int:
        raise ValueError(f"{label} must be an integer")
    return value


def parse_utc_timestamp(value: Any, label: str = "recorded timestamp") -> datetime:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{label} must be a canonical UTC timestamp")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise ValueError(f"{label} is malformed") from exc
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ValueError(f"{label} must include an offset")
    if parsed.utcoffset().total_seconds() != 0:
        raise ValueError(f"{label} must be UTC")
    return parsed


def validate_utc_timestamp(value: str) -> None:
    parse_utc_timestamp(value)


def normalized_sha256(value: Any, label: str) -> str:
    if not isinstance(value, str) or LOWER_SHA256_RE.fullmatch(value) is None:
        raise ValueError(f"{label} must be 64 lowercase hexadecimal characters")
    return value


def normalized_boot_id(value: Any, label: str = "host boot ID") -> str:
    if not isinstance(value, str) or value != value.strip():
        raise ValueError(f"{label} is malformed")
    try:
        parsed = uuid.UUID(value)
    except (ValueError, AttributeError) as exc:
        raise ValueError(f"{label} is malformed") from exc
    canonical = str(parsed)
    if value != canonical:
        raise ValueError(f"{label} must be canonical lowercase UUID text")
    return canonical


def nonempty_text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or value != value.strip():
        raise ValueError(f"{label} must be non-empty canonical text")
    return value


def read_json(path: Path) -> tuple[dict[str, Any], bytes, str]:
    data = path.read_bytes()
    try:
        document = json.loads(data)
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"{path.name} is not valid UTF-8 JSON") from exc
    if not isinstance(document, dict):
        raise ValueError(f"{path.name} must contain a JSON object")
    return document, data, sha256_bytes(data)


def parse_sidecar_bytes(data: bytes, expected_basename: str) -> str:
    try:
        text = data.decode("ascii")
    except UnicodeDecodeError as exc:
        raise ValueError("persistent software-zero checksum sidecar is not ASCII") from exc
    fields = text.strip().split()
    if len(fields) != 2 or fields[1] != expected_basename:
        raise ValueError("persistent software-zero checksum sidecar is malformed")
    return normalized_sha256(fields[0], "persistent software-zero checksum")


def parse_sidecar(path: Path, expected_basename: str) -> str:
    return parse_sidecar_bytes(path.read_bytes(), expected_basename)


def require_exact(mapping: dict[str, Any], field: str, expected: Any, label: str) -> None:
    actual = mapping.get(field)
    if type(actual) is not type(expected) or actual != expected:
        raise ValueError(f"{label}.{field} must be exactly {expected!r}")


def require_zero_counts(mapping: dict[str, Any], fields: tuple[str, ...], label: str) -> None:
    for field in fields:
        if exact_int(mapping.get(field), f"{label}.{field}") != 0:
            raise ValueError(f"{label}.{field} records forbidden command activity")


def validate_zero(document: dict[str, Any]) -> None:
    if document.get("schema") != ZERO_SCHEMA or document.get("reference_name") != REFERENCE_NAME:
        raise ValueError("persistent software-zero schema/reference mismatch")
    motors = document.get("motors")
    if not isinstance(motors, dict) or set(motors) != set(EXPECTED_MOTORS):
        raise ValueError("persistent software-zero must contain exactly seven motors")
    for name in EXPECTED_MOTORS:
        record = motors[name]
        if not isinstance(record, dict):
            raise ValueError(f"invalid persistent-zero motor record: {name}")
        finite_number(record.get("raw_position_rad"), f"{name} raw reference")
        if "sample_count" in record and exact_int(record["sample_count"], f"{name} sample count") <= 0:
            raise ValueError(f"{name} persistent sample count must be positive")
        if "sample_span_rad" in record and finite_number(
            record["sample_span_rad"], f"{name} sample span"
        ) < 0.0:
            raise ValueError(f"{name} persistent sample span must be non-negative")
    mapping = document.get("mapping")
    if not isinstance(mapping, dict):
        raise ValueError("persistent software-zero mapping is missing")
    if not math.isclose(
        finite_number(mapping.get("gear_ratio"), "gear ratio"),
        GEAR_RATIO,
        rel_tol=0.0,
        abs_tol=1e-9,
    ):
        raise ValueError("persistent software-zero gear ratio mismatch")
    if mapping.get("signs") != ALL_SIGNS:
        raise ValueError("persistent software-zero signs mismatch")
    writes = document.get("writes")
    if not isinstance(writes, dict) or any(
        writes.get(field) is not False
        for field in (
            "motor_internal_zero_modified",
            "rid_written",
            "flash_or_eeprom_written",
        )
    ):
        raise ValueError("persistent software-zero records a forbidden motor write")


def validate_hints(document: dict[str, Any]) -> None:
    if document.get("schema") != HINT_SCHEMA or document.get("reference_name") != REFERENCE_NAME:
        raise ValueError("recovery-hint schema/reference mismatch")
    motors = document.get("motors")
    if not isinstance(motors, dict) or set(motors) != set(GO_HINT_MOTORS):
        raise ValueError("recovery hints must contain exactly the six GO motors")
    for name in GO_HINT_MOTORS:
        record = motors[name]
        if not isinstance(record, dict):
            raise ValueError(f"invalid recovery-hint record: {name}")
        finite_number(record.get("logical_position_rad"), f"{name} recovery hint")


def validate_initial_pose(document: dict[str, Any], current_sha256: str) -> None:
    if document.get("有效") is not True:
        raise ValueError("initial pose is not marked valid")
    if document.get("会话标识") != f"persistent:{current_sha256[:16]}":
        raise ValueError("initial pose is bound to a different persistent zero")
    positions = document.get("关节位置_弧度")
    joint_names = {f"J{index}" for index in range(1, 7)}
    if not isinstance(positions, dict) or set(positions) != joint_names:
        raise ValueError("initial pose must contain exactly six joints")
    for name, value in positions.items():
        finite_number(value, f"{name} initial position")


def validate_capture_identity(
    document: dict[str, Any],
    *,
    kind: str,
    schema: str,
    policy: str,
    host_boot_id: str,
    pose_binding_id: str,
    physical_gate: str,
    physical_fields: tuple[str, ...],
) -> dict[str, Any]:
    if document.get("schema") != schema or document.get("status") != "PASS":
        raise ValueError(f"{kind} capture schema/status mismatch")
    require_exact(document, "hardware_accessed", True, f"{kind} capture")
    require_exact(
        document,
        "physical_power_state_during_capture",
        "24V_ON",
        f"{kind} capture",
    )
    require_exact(document, "physical_power_off_required", False, f"{kind} capture")
    capture_id = nonempty_text(document.get("capture_id"), f"{kind} capture_id")
    if CAPTURE_ID_PATTERNS[kind].fullmatch(capture_id) is None:
        raise ValueError(f"{kind} capture_id format mismatch")
    power_session_id = nonempty_text(
        document.get("power_session_id"), f"{kind} power_session_id"
    )
    if POWER_SESSION_ID_PATTERNS[kind].fullmatch(power_session_id) is None:
        raise ValueError(f"{kind} power_session_id format mismatch")
    if document.get("host_boot_id") != host_boot_id:
        raise ValueError(f"{kind} capture host boot ID mismatch")
    normalized_boot_id(document.get("host_boot_id"), f"{kind} host boot ID")
    recorded_boottime_ns = exact_int(
        document.get("recorded_boottime_ns"), f"{kind} recorded CLOCK_BOOTTIME"
    )
    if recorded_boottime_ns <= 0:
        raise ValueError(f"{kind} recorded CLOCK_BOOTTIME must be positive")
    recorded_at = parse_utc_timestamp(
        document.get("recorded_at_utc"), f"{kind} recorded_at_utc"
    )
    if document.get("pose_binding_id") != pose_binding_id:
        raise ValueError(f"{kind} capture pose_binding_id mismatch")
    normalized_sha256(document.get("pose_binding_id"), f"{kind} pose_binding_id")
    safety = document.get("safety")
    if not isinstance(safety, dict):
        raise ValueError(f"{kind} capture safety evidence is missing")
    require_exact(safety, "execution_policy", policy, f"{kind} safety")
    for field in FORBIDDEN_COMMAND_COUNT_FIELDS:
        if field in safety and exact_int(safety[field], f"{kind} safety.{field}") != 0:
            raise ValueError(f"{kind} safety.{field} records forbidden command activity")
    for field in (
        "motor_internal_zero_modified",
        "rid_written",
        "flash_or_eeprom_written",
        "fc_enable_used",
        "motion_command_used",
    ):
        if field in safety and safety[field] is not False:
            raise ValueError(f"{kind} safety.{field} records forbidden command activity")
    physical = document.get("physical_confirmation")
    if not isinstance(physical, dict):
        raise ValueError(f"{kind} capture physical confirmation is missing")
    require_exact(physical, "confirmation_gate", physical_gate, f"{kind} physical")
    for field in physical_fields:
        require_exact(physical, field, True, f"{kind} physical")
    return {
        "capture_id": capture_id,
        "power_session_id": power_session_id,
        "recorded_at_utc": recorded_at,
        "recorded_boottime_ns": recorded_boottime_ns,
        "safety": safety,
    }


def validate_go_terminal_proof(
    proof: Any, *, bus: str, packet_count: int, require_j2_marker: bool = False
) -> None:
    if not isinstance(proof, dict):
        raise ValueError(f"{bus} terminal proof is missing")
    expected = {
        "TERMINAL_PATH": "NORMAL",
        "GUI_GO_CONTROLLER_BUS": bus,
        "EXECUTION_POLICY": "BRAKE_ONLY",
        "COMMAND_RX_ENABLED": "NO",
        "FOC_TX_ATTEMPT_COUNT": "0",
        "FOC_SERIAL_SEND_CALL_COUNT": "0",
        "OTHER_MODE_TX_ATTEMPT_COUNT": "0",
        "BRAKE_ONLY_GUARD_BLOCK_COUNT": "0",
        "BRAKE_ONLY_AUDIT": "PASS",
        "FINAL_MODE": "BRAKE",
        "FINAL_BRAKE": "PASS",
        "MOTOR_INTERNAL_ZERO_WRITE": "NO",
    }
    if require_j2_marker:
        expected["J2_SESSION_REFERENCE_CONFIGURED"] = "NO"
    for field, value in expected.items():
        require_exact(proof, field, value, f"{bus} terminal")
    numeric_fields = (
        "COMPLETED_CYCLES",
        "TX_ATTEMPT_TOTAL",
        "BRAKE_TX_ATTEMPT_COUNT",
        "SERIAL_SEND_CALL_COUNT",
    )
    parsed: dict[str, int] = {}
    for field in numeric_fields:
        value = proof.get(field)
        if not isinstance(value, str) or not value.isdigit():
            raise ValueError(f"{bus} terminal {field} must be a decimal count")
        parsed[field] = int(value, 10)
    if parsed["COMPLETED_CYCLES"] < packet_count:
        raise ValueError(f"{bus} terminal completed-cycle count is too short")
    if not (
        parsed["TX_ATTEMPT_TOTAL"]
        == parsed["BRAKE_TX_ATTEMPT_COUNT"]
        == parsed["SERIAL_SEND_CALL_COUNT"]
    ):
        raise ValueError(f"{bus} terminal BRAKE transmit counters disagree")
    if parsed["TX_ATTEMPT_TOTAL"] < packet_count:
        raise ValueError(f"{bus} terminal BRAKE transmit count is too short")


def canonical_terminal_count(proof: dict[str, Any], field: str) -> int:
    value = proof.get(field)
    if not isinstance(value, str) or not value.isdigit():
        raise ValueError(f"J2 terminal {field} must be a decimal count")
    parsed = int(value, 10)
    if value != str(parsed):
        raise ValueError(f"J2 terminal {field} must be canonical decimal")
    return parsed


def validate_j2_startup_and_tx_evidence(
    *,
    safety: dict[str, Any],
    raw_summary: dict[str, Any],
    proof: dict[str, Any],
    packet_count: int,
) -> None:
    require_exact(safety, "startup_brake_prime_passed", True, "J2 safety")

    startup_prime = raw_summary.get("startup_brake_prime")
    if not isinstance(startup_prime, dict):
        raise ValueError("J2 startup BRAKE prime evidence is missing")
    for field, expected in {
        "state": "PASS",
        "maximum_invalid_prefix_pairs": J2_STARTUP_PRIME_MAX_INVALID_PREFIX_PAIRS,
        "required_healthy_pairs": J2_STARTUP_PRIME_REQUIRED_HEALTHY_PAIRS,
        "feedback_published_during_prime": False,
        "window_reopens_after_first_healthy_pair": False,
    }.items():
        require_exact(startup_prime, field, expected, "J2 startup BRAKE prime")

    invalid_prefix_pairs = exact_int(
        startup_prime.get("invalid_prefix_pairs"),
        "J2 startup BRAKE prime invalid prefix pairs",
    )
    if not 0 <= invalid_prefix_pairs <= J2_STARTUP_PRIME_MAX_INVALID_PREFIX_PAIRS:
        raise ValueError("J2 startup BRAKE prime invalid prefix is outside 0..3")
    healthy_pairs = exact_int(
        startup_prime.get("healthy_pairs"),
        "J2 startup BRAKE prime healthy pairs",
    )
    if healthy_pairs != J2_STARTUP_PRIME_REQUIRED_HEALTHY_PAIRS:
        raise ValueError("J2 startup BRAKE prime must contain exactly 5 healthy pairs")
    attempted_pairs = exact_int(
        startup_prime.get("attempted_pairs"),
        "J2 startup BRAKE prime attempted pairs",
    )
    if attempted_pairs != invalid_prefix_pairs + healthy_pairs:
        raise ValueError("J2 startup BRAKE prime pair counters disagree")
    prime_tx_attempt_count = exact_int(
        startup_prime.get("tx_attempt_count"),
        "J2 startup BRAKE prime transmit count",
    )
    if prime_tx_attempt_count != 2 * attempted_pairs:
        raise ValueError("J2 startup BRAKE prime transmit count disagrees")
    elapsed_ns = exact_int(
        startup_prime.get("elapsed_ns"),
        "J2 startup BRAKE prime elapsed time",
    )
    if not 1 <= elapsed_ns <= J2_STARTUP_PRIME_MAXIMUM_ELAPSED_NS:
        raise ValueError(
            "J2 startup BRAKE prime elapsed time is outside 1..500000000 ns"
        )

    safety_invalid_prefix_pairs = exact_int(
        safety.get("startup_invalid_prefix_pairs"),
        "J2 safety startup invalid prefix pairs",
    )
    if safety_invalid_prefix_pairs != invalid_prefix_pairs:
        raise ValueError("J2 safety and raw startup invalid-prefix counts disagree")

    for field, expected in {
        "J2_STARTUP_BRAKE_PRIME_STATE": "PASS",
        "J2_STARTUP_BRAKE_PRIME_MAX_INVALID_PREFIX_PAIRS": str(
            J2_STARTUP_PRIME_MAX_INVALID_PREFIX_PAIRS
        ),
        "J2_STARTUP_BRAKE_PRIME_REQUIRED_HEALTHY_PAIRS": str(
            J2_STARTUP_PRIME_REQUIRED_HEALTHY_PAIRS
        ),
    }.items():
        require_exact(proof, field, expected, "J2 terminal")
    terminal_counts = {
        field: canonical_terminal_count(proof, field)
        for field in (
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
    }
    for field, expected in {
        "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS": invalid_prefix_pairs,
        "J2_STARTUP_BRAKE_PRIME_HEALTHY_PAIRS": healthy_pairs,
        "J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS": attempted_pairs,
        "J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT": prime_tx_attempt_count,
        "J2_STARTUP_BRAKE_PRIME_ELAPSED_NS": elapsed_ns,
    }.items():
        if terminal_counts[field] != expected:
            raise ValueError(f"J2 terminal {field} disagrees with raw startup evidence")
    if terminal_counts["COMPLETED_CYCLES"] != packet_count:
        raise ValueError("J2 terminal completed cycles do not equal packet count")
    if (
        terminal_counts["FINAL_BRAKE_TX_ATTEMPT_COUNT"]
        != J2_FINAL_BRAKE_TX_ATTEMPT_COUNT
    ):
        raise ValueError("J2 terminal final BRAKE transmit count is not exactly 40")

    phase_accounting = raw_summary.get("phase_tx_accounting")
    if not isinstance(phase_accounting, dict):
        raise ValueError("J2 phase transmit accounting is missing")
    phase_counts = {
        field: exact_int(
            phase_accounting.get(field), f"J2 phase transmit accounting {field}"
        )
        for field in (
            "completed_cycles",
            "startup_prime_tx_attempt_count",
            "control_loop_tx_attempt_count",
            "final_brake_tx_attempt_count",
            "aggregate_tx_attempt_count",
        )
    }
    expected_control_tx = 2 * packet_count
    if phase_counts["completed_cycles"] != packet_count:
        raise ValueError("J2 phase completed cycles do not equal packet count")
    if phase_counts["startup_prime_tx_attempt_count"] != prime_tx_attempt_count:
        raise ValueError("J2 phase and startup prime transmit counts disagree")
    if phase_counts["control_loop_tx_attempt_count"] != expected_control_tx:
        raise ValueError(
            "J2 phase control-loop transmit count is not twice packet count"
        )
    if (
        phase_counts["final_brake_tx_attempt_count"]
        != J2_FINAL_BRAKE_TX_ATTEMPT_COUNT
    ):
        raise ValueError("J2 phase final BRAKE transmit count is not exactly 40")
    expected_aggregate = (
        prime_tx_attempt_count
        + expected_control_tx
        + J2_FINAL_BRAKE_TX_ATTEMPT_COUNT
    )
    if phase_counts["aggregate_tx_attempt_count"] != expected_aggregate:
        raise ValueError("J2 phase aggregate is not prime + control + final BRAKE")
    if not (
        terminal_counts["TX_ATTEMPT_TOTAL"]
        == terminal_counts["BRAKE_TX_ATTEMPT_COUNT"]
        == terminal_counts["SERIAL_SEND_CALL_COUNT"]
        == phase_counts["aggregate_tx_attempt_count"]
    ):
        raise ValueError("J2 terminal and phase aggregate transmit counters disagree")


def validate_worker(worker: Any, label: str) -> dict[str, Any]:
    if not isinstance(worker, dict):
        raise ValueError(f"{label} worker evidence is missing")
    nonempty_text(worker.get("path"), f"{label} worker path")
    normalized_sha256(worker.get("sha256"), f"{label} worker sha256")
    return worker


def validate_temperature_maps(
    minimums: Any, maximums: Any, names: tuple[str, ...], label: str
) -> None:
    if not isinstance(minimums, dict) or set(minimums) != set(names):
        raise ValueError(f"{label} minimum-temperature map mismatch")
    if not isinstance(maximums, dict) or set(maximums) != set(names):
        raise ValueError(f"{label} maximum-temperature map mismatch")
    for name in names:
        minimum = finite_number(minimums[name], f"{label} {name} minimum temperature")
        maximum = finite_number(maximums[name], f"{label} {name} maximum temperature")
        if not 0.0 <= minimum <= maximum < 60.0:
            raise ValueError(f"{label} {name} temperature evidence is unsafe")


def capture_measurement(
    document: dict[str, Any],
    name: str,
    *,
    expected_count: int,
    j6: bool = False,
) -> dict[str, float | int]:
    motors = document.get("motors")
    if not isinstance(motors, dict) or not isinstance(motors.get(name), dict):
        raise ValueError(f"capture is missing motor {name}")
    record = motors[name]
    count = exact_int(record.get("sample_count"), f"{name} sample count")
    if count != expected_count or count < MIN_SAMPLE_COUNT:
        raise ValueError(f"{name} capture sample count mismatch")
    statistics_key = "raw_position_rad" if j6 else "unwrapped_raw_position_rad"
    statistics = record.get(statistics_key)
    if not isinstance(statistics, dict):
        raise ValueError(f"{name} capture statistics are missing")
    required = ["mean", "minimum", "maximum", "span", "standard_deviation"]
    if j6:
        required.append("median")
    values = {
        field: finite_number(statistics.get(field), f"{name} capture {field}")
        for field in required
    }
    mean = values["mean"]
    minimum = values["minimum"]
    maximum = values["maximum"]
    span = values["span"]
    deviation = values["standard_deviation"]
    if any(abs(value) > MAX_PROTOCOL_POSITION_RAD for value in (mean, minimum, maximum)):
        raise ValueError(f"{name} capture is outside the protocol envelope")
    if not minimum <= mean <= maximum:
        raise ValueError(f"{name} capture mean is outside its range")
    if j6 and not minimum <= values["median"] <= maximum:
        raise ValueError(f"{name} capture median is outside its range")
    if not math.isclose(span, maximum - minimum, rel_tol=1e-9, abs_tol=1e-12):
        raise ValueError(f"{name} capture span is inconsistent")
    maximum_span = MAX_J6_CAPTURE_SPAN_RAD if j6 else MAX_GO_CAPTURE_SPAN_RAD
    if not 0.0 <= span <= maximum_span:
        raise ValueError(f"{name} capture span is unsafe")
    if not 0.0 <= deviation <= span + 1e-12:
        raise ValueError(f"{name} capture standard deviation is inconsistent")
    return {"raw_position_rad": mean, "sample_count": count, "sample_span_rad": span}


def validate_j2_capture(
    document: dict[str, Any], *, host_boot_id: str, pose_binding_id: str
) -> tuple[dict[str, dict[str, float | int]], dict[str, Any]]:
    try:
        j2_anchor_contract.validate_capture(document)
    except j2_anchor_contract.AnchorValidationError as exc:
        raise ValueError(f"J2 shared anchor contract rejected capture: {exc}") from exc
    identity = validate_capture_identity(
        document,
        kind="j2",
        schema=J2_CAPTURE_SCHEMA,
        policy="BRAKE_ONLY",
        host_boot_id=host_boot_id,
        pose_binding_id=pose_binding_id,
        physical_gate=J2_PHYSICAL_GATE,
        physical_fields=(
            "support_reliable",
            "j2_vertical_initialization_pose",
            "arm_not_moved",
            "arm_stationary",
            "not_at_mechanical_limit",
        ),
    )
    safety = identity["safety"]
    for field, expected in {
        "all_controller_modes": ["brake"],
        "command_rx_enabled": False,
        "j2_session_reference_configured": False,
        "motor_internal_zero_modified": False,
        "rid_written": False,
        "flash_or_eeprom_written": False,
    }.items():
        require_exact(safety, field, expected, "J2 safety")
    require_zero_counts(
        safety,
        (
            "foc_tx_attempt_count",
            "foc_serial_send_call_count",
            "other_mode_tx_attempt_count",
            "brake_only_guard_block_count",
            "active_or_hold_commands_sent",
            "active_commands_sent",
            "hold_commands_sent",
            "communication_failure_packets",
            "merror_nonzero_packets",
            "motor_internal_zero_write_count",
            "rid_write_count",
            "flash_or_eeprom_write_count",
        ),
        "J2 safety",
    )
    packet_count = exact_int(document.get("packet_count"), "J2 packet count")
    if not MIN_SAMPLE_COUNT <= packet_count <= 1500:
        raise ValueError("J2 capture packet count is outside its contract")
    if finite_number(document.get("source_coverage_s"), "J2 capture coverage") < MIN_CAPTURE_COVERAGE_S:
        raise ValueError("J2 capture duration is insufficient")
    motors = document.get("motors")
    if not isinstance(motors, dict) or set(motors) != {"J2A", "J2B"}:
        raise ValueError("J2 capture motor set mismatch")
    raw_summary = document.get("raw_safety_summary")
    if not isinstance(raw_summary, dict):
        raise ValueError("J2 raw safety summary is missing")
    for field in (
        "branch_inputs_loaded",
        "persistent_zero_loaded",
        "recovery_hint_loaded",
        "integer_turn_branch_inferred",
    ):
        require_exact(raw_summary, field, False, "J2 raw safety summary")
    require_exact(raw_summary, "udp_send_calls", 0, "J2 raw safety summary")
    require_exact(raw_summary, "controller_fault_packets", 0, "J2 raw safety summary")
    validate_temperature_maps(
        raw_summary.get("temperature_min_c"),
        raw_summary.get("temperature_max_c"),
        ("J2A", "J2B"),
        "J2",
    )
    worker = validate_worker(raw_summary.get("worker"), "J2")
    require_exact(worker, "returncode", 0, "J2 worker")
    require_exact(worker, "stop_escalation", "none", "J2 worker")
    require_exact(worker, "forced_kill", False, "J2 worker")
    terminal_proof = worker.get("terminal_proof")
    validate_go_terminal_proof(
        terminal_proof,
        bus="j2",
        packet_count=packet_count,
        require_j2_marker=True,
    )
    validate_j2_startup_and_tx_evidence(
        safety=safety,
        raw_summary=raw_summary,
        proof=terminal_proof,
        packet_count=packet_count,
    )
    measurements = {
        name: capture_measurement(
            document, name, expected_count=packet_count
        )
        for name in ("J2A", "J2B")
    }
    return measurements, identity


def validate_go_aux_capture(
    document: dict[str, Any], *, host_boot_id: str, pose_binding_id: str
) -> tuple[dict[str, dict[str, float | int]], dict[str, Any]]:
    identity = validate_capture_identity(
        document,
        kind="go_aux",
        schema=GO_AUX_CAPTURE_SCHEMA,
        policy="BRAKE_ONLY",
        host_boot_id=host_boot_id,
        pose_binding_id=pose_binding_id,
        physical_gate=GO_AUX_PHYSICAL_GATE,
        physical_fields=(
            "support_reliable",
            "whole_arm_vertical_initialization_pose",
            "arm_not_moved",
            "arm_stationary",
            "not_at_mechanical_limit",
        ),
    )
    safety = identity["safety"]
    for field, expected in {
        "all_controller_modes": ["brake"],
        "command_rx_enabled": False,
        "motor_internal_zero_modified": False,
        "rid_written": False,
        "flash_or_eeprom_written": False,
    }.items():
        require_exact(safety, field, expected, "GO-AUX safety")
    require_zero_counts(
        safety,
        (
            "foc_tx_attempt_count",
            "foc_serial_send_call_count",
            "other_mode_tx_attempt_count",
            "active_or_hold_commands_sent",
            "communication_failure_packets",
            "merror_nonzero_packets",
        ),
        "GO-AUX safety",
    )
    validate_worker(document.get("worker"), "GO-AUX")
    domains = document.get("domains")
    motors = document.get("motors")
    bus_motors = {"j1": ("J1",), "j345": ("J3", "J4", "J5")}
    if not isinstance(domains, dict) or set(domains) != set(bus_motors):
        raise ValueError("GO-AUX capture domain set mismatch")
    if not isinstance(motors, dict) or set(motors) != {"J1", "J3", "J4", "J5"}:
        raise ValueError("GO-AUX capture motor set mismatch")
    counts: dict[str, int] = {}
    for bus, names in bus_motors.items():
        domain = domains[bus]
        if not isinstance(domain, dict):
            raise ValueError(f"GO-AUX domain {bus} is invalid")
        count = exact_int(domain.get("packet_count"), f"{bus} packet count")
        if not MIN_SAMPLE_COUNT <= count <= 1500:
            raise ValueError(f"GO-AUX domain {bus} packet count is outside its contract")
        if finite_number(domain.get("source_coverage_s"), f"{bus} coverage") < MIN_CAPTURE_COVERAGE_S:
            raise ValueError(f"GO-AUX domain {bus} duration is insufficient")
        if domain.get("motor_names") != list(names):
            raise ValueError(f"GO-AUX domain {bus} motor order mismatch")
        validate_temperature_maps(
            domain.get("temperature_min_c"),
            domain.get("temperature_max_c"),
            names,
            f"GO-AUX {bus}",
        )
        nonempty_text(domain.get("feedback_source_endpoint"), f"{bus} feedback endpoint")
        validate_go_terminal_proof(
            domain.get("terminal_proof"), bus=bus, packet_count=count
        )
        for name in names:
            counts[name] = count
    raw_summary = document.get("raw_safety_summary")
    if not isinstance(raw_summary, dict):
        raise ValueError("GO-AUX raw safety summary is missing")
    for field in (
        "persistent_zero_loaded",
        "recovery_hint_loaded",
        "integer_turn_branch_inferred",
    ):
        require_exact(raw_summary, field, False, "GO-AUX raw safety summary")
    require_exact(raw_summary, "udp_send_calls", 0, "GO-AUX raw safety summary")
    measurements = {
        name: capture_measurement(document, name, expected_count=counts[name])
        for name in ("J1", "J3", "J4", "J5")
    }
    return measurements, identity


def validate_j6_capture(
    document: dict[str, Any], *, host_boot_id: str, pose_binding_id: str
) -> tuple[dict[str, float | int], dict[str, Any]]:
    identity = validate_capture_identity(
        document,
        kind="j6",
        schema=J6_CAPTURE_SCHEMA,
        policy="DISABLED_REFRESH_ONLY",
        host_boot_id=host_boot_id,
        pose_binding_id=pose_binding_id,
        physical_gate=J6_PHYSICAL_GATE,
        physical_fields=(
            "j6_24v_on",
            "support_reliable",
            "whole_arm_vertical_initialization_pose",
            "arm_stationary",
            "not_at_mechanical_limit",
        ),
    )
    packet_count = exact_int(document.get("packet_count"), "J6 packet count")
    if packet_count != MIN_SAMPLE_COUNT:
        raise ValueError("J6 capture must contain exactly 500 main frames")
    if finite_number(document.get("source_coverage_s"), "J6 capture coverage") < MIN_CAPTURE_COVERAGE_S:
        raise ValueError("J6 capture duration is insufficient")
    safety = identity["safety"]
    attempted_fields = (
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
    )
    require_zero_counts(safety, attempted_fields, "J6 safety")
    require_zero_counts(
        safety,
        (
            "active_or_hold_commands_sent",
            "active_commands_sent",
            "hold_commands_sent",
            "motor_internal_zero_write_count",
            "rid_write_count",
            "flash_or_eeprom_write_count",
        ),
        "J6 safety",
    )
    for field in (
        "motor_internal_zero_modified",
        "rid_written",
        "flash_or_eeprom_written",
    ):
        require_exact(safety, field, False, "J6 safety")
    expected_refreshes = packet_count + 5
    require_exact(safety, "total_tx_attempt_count", expected_refreshes, "J6 safety")
    require_exact(safety, "refresh_request_count", expected_refreshes, "J6 safety")
    observed = safety.get("observed_tx_audit")
    if not isinstance(observed, dict):
        raise ValueError("J6 observed transmit audit is missing")
    require_exact(observed, "total_tx_attempt_count", expected_refreshes, "J6 observed TX")
    require_exact(observed, "refresh_request_count", expected_refreshes, "J6 observed TX")
    require_zero_counts(observed, attempted_fields, "J6 observed TX")
    terminal = document.get("terminal")
    if not isinstance(terminal, dict):
        raise ValueError("J6 terminal DISABLED proof is missing")
    require_exact(terminal, "final_disabled_frames", 5, "J6 terminal")
    require_exact(terminal, "required_final_disabled_frames", 5, "J6 terminal")
    require_exact(terminal, "confirmed", True, "J6 terminal")
    require_exact(terminal, "channel_closed", True, "J6 terminal")
    states = terminal.get("states")
    if not isinstance(states, list) or len(states) != 5 or any(
        type(value) is not int or value != 0 for value in states
    ):
        raise ValueError("J6 terminal states are not five DISABLED frames")
    motors = document.get("motors")
    if not isinstance(motors, dict) or set(motors) != {"J6"}:
        raise ValueError("J6 capture motor set mismatch")
    motor = motors["J6"]
    require_exact(motor, "state_values", [0], "J6 motor")
    usb_identity = document.get("usb_identity")
    if not isinstance(usb_identity, dict) or any(
        usb_identity.get(field) != expected for field, expected in EXPECTED_J6_USB.items()
    ):
        raise ValueError("J6 USB identity mismatch")
    raw_summary = document.get("raw_safety_summary")
    if not isinstance(raw_summary, dict):
        raise ValueError("J6 raw safety summary is missing")
    for field in (
        "branch_inputs_loaded",
        "persistent_zero_loaded",
        "recovery_hint_loaded",
        "integer_turn_branch_inferred",
    ):
        require_exact(raw_summary, field, False, "J6 raw safety summary")
    require_exact(raw_summary, "error_callback_count", 0, "J6 raw safety summary")
    require_exact(raw_summary, "unknown_rx_callback_count", 0, "J6 raw safety summary")
    require_exact(raw_summary, "expected_can_channel", EXPECTED_J6_CHANNEL, "J6 raw safety summary")
    require_exact(raw_summary, "expected_motor_id", EXPECTED_J6_MOTOR_ID, "J6 raw safety summary")
    require_exact(raw_summary, "expected_master_id", EXPECTED_J6_MASTER_ID, "J6 raw safety summary")
    require_exact(document, "failures", [], "J6 capture")
    measurement = capture_measurement(
        document, "J6", expected_count=packet_count, j6=True
    )
    return measurement, identity


def validate_capture_bundle(
    j2: dict[str, Any],
    go_aux: dict[str, Any],
    j6: dict[str, Any],
    *,
    host_boot_id: str,
    pose_binding_id: str,
    current_boottime_ns: int | None = None,
) -> tuple[dict[str, dict[str, float | int]], dict[str, Any]]:
    normalized_boot_id(host_boot_id)
    normalized_sha256(pose_binding_id, "expected pose_binding_id")
    measurements, j2_identity = validate_j2_capture(
        j2, host_boot_id=host_boot_id, pose_binding_id=pose_binding_id
    )
    aux_measurements, aux_identity = validate_go_aux_capture(
        go_aux, host_boot_id=host_boot_id, pose_binding_id=pose_binding_id
    )
    j6_measurement, j6_identity = validate_j6_capture(
        j6, host_boot_id=host_boot_id, pose_binding_id=pose_binding_id
    )
    measurements.update(aux_measurements)
    measurements["J6"] = j6_measurement
    identities = {"j2": j2_identity, "go_aux": aux_identity, "j6": j6_identity}
    capture_ids = [identity["capture_id"] for identity in identities.values()]
    if len(set(capture_ids)) != 3:
        raise ValueError("capture IDs must be unique")
    boottimes = [identity["recorded_boottime_ns"] for identity in identities.values()]
    if max(boottimes) - min(boottimes) > MAX_CAPTURE_BUNDLE_SPAN_NS:
        raise ValueError("capture bundle exceeds the 120-second same-pose window")
    if current_boottime_ns is not None:
        current = exact_int(current_boottime_ns, "current CLOCK_BOOTTIME")
        if current <= 0 or any(value > current for value in boottimes):
            raise ValueError("capture CLOCK_BOOTTIME is future-dated")
        if any(current - value > MAX_CAPTURE_AGE_NS for value in boottimes):
            raise ValueError("capture bundle is older than the 300-second apply window")
    return measurements, identities


def validate_go_captures(
    j2: dict[str, Any],
    go_aux: dict[str, Any],
    *,
    host_boot_id: str,
    pose_binding_id: str,
) -> dict[str, dict[str, float | int]]:
    measurements, _ = validate_j2_capture(
        j2, host_boot_id=host_boot_id, pose_binding_id=pose_binding_id
    )
    aux, _ = validate_go_aux_capture(
        go_aux, host_boot_id=host_boot_id, pose_binding_id=pose_binding_id
    )
    measurements.update(aux)
    return measurements


def build_documents(
    zero: dict[str, Any],
    hints: dict[str, Any],
    initial_pose: dict[str, Any],
    *,
    original_sha256: str,
    measurements: dict[str, dict[str, float | int]],
    capture_hashes: dict[str, str],
    capture_ids: dict[str, str],
    pose_binding_id: str,
    recorded_at_utc: str,
    host_boot_id: str,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any], str]:
    validate_utc_timestamp(recorded_at_utc)
    normalized_boot_id(host_boot_id)
    normalized_sha256(original_sha256, "original persistent zero sha256")
    normalized_sha256(pose_binding_id, "pose_binding_id")
    if set(capture_hashes) != {"j2", "go_aux", "j6"} or any(
        normalized_sha256(value, f"{name} capture sha256") != value
        for name, value in capture_hashes.items()
    ):
        raise ValueError("capture hash set is incomplete")
    if set(capture_ids) != {"j2", "go_aux", "j6"} or len(set(capture_ids.values())) != 3:
        raise ValueError("capture ID set is incomplete or non-unique")
    if set(measurements) != set(EXPECTED_MOTORS):
        raise ValueError("whole-arm rebase requires all seven motor measurements")
    updated_zero = copy.deepcopy(zero)
    updated_hints = copy.deepcopy(hints)
    updated_initial = copy.deepcopy(initial_pose)
    measurement_records: dict[str, dict[str, Any]] = {}
    for name in EXPECTED_MOTORS:
        measurement = measurements[name]
        old_reference = finite_number(
            zero["motors"][name]["raw_position_rad"], f"old {name} reference"
        )
        new_reference = finite_number(measurement["raw_position_rad"], f"new {name} reference")
        sample_count = exact_int(measurement["sample_count"], f"{name} sample count")
        sample_span = finite_number(measurement["sample_span_rad"], f"{name} sample span")
        updated_zero["motors"][name] = {
            "raw_position_rad": new_reference,
            "sample_count": sample_count,
            "sample_span_rad": sample_span,
        }
        measurement_records[name] = {
            "old_raw_position_rad": old_reference,
            "new_raw_position_rad": new_reference,
            "sample_count": sample_count,
            "sample_span_rad": sample_span,
        }
    previous_record = copy.deepcopy(updated_zero.get("last_software_zero_rebase"))
    history = updated_zero.get("software_zero_rebase_history", [])
    if not isinstance(history, list):
        raise ValueError("software-zero rebase history must be an array")
    history = copy.deepcopy(history)
    if previous_record is not None and previous_record not in history:
        history.append(previous_record)
    record = {
        "schema": "go-m8010-software-zero-rebase-record/1.0",
        "recorded_at_utc": recorded_at_utc,
        "source": "V15.30A_whole-arm_user-confirmed-vertical_stationary_BRAKE-DISABLED_captures",
        "scope": list(EXPECTED_MOTORS),
        "original_persistent_zero_sha256": original_sha256,
        "host_boot_id": host_boot_id,
        "pose_binding_id": pose_binding_id,
        "capture_id": copy.deepcopy(capture_ids),
        "capture_sha256": capture_hashes,
        "measurements": measurement_records,
        "motor_internal_zero_modified": False,
        "rid_written": False,
        "flash_or_eeprom_written": False,
    }
    history.append(copy.deepcopy(record))
    updated_zero["software_zero_rebase_history"] = history
    updated_zero["last_software_zero_rebase"] = record
    updated_zero["updated_at_utc"] = recorded_at_utc
    updated_zero["terminal_state"] = {
        "go_motors": "BRAKE_CONFIRMED",
        "J6": "DISABLED_CONFIRMED",
    }
    for name in GO_HINT_MOTORS:
        updated_hints["motors"][name]["logical_position_rad"] = 0.0
    updated_hints["source"] = record["source"]
    updated_hints["updated_at_utc"] = recorded_at_utc
    zero_data = json_bytes(updated_zero)
    new_sha256 = sha256_bytes(zero_data)
    updated_initial["会话标识"] = f"persistent:{new_sha256[:16]}"
    updated_initial["时间戳"] = recorded_at_utc
    updated_initial["语义"] = "用户确认的竖直初始化姿态；主机持久软件零位"
    updated_initial["电机内部零位"] = "未修改"
    updated_initial["CAD零位"] = "PENDING"
    updated_initial["ROS零位"] = "PERSISTENT_SOFTWARE_ZERO_V1"
    updated_initial["关节位置_弧度"] = {f"J{index}": 0.0 for index in range(1, 7)}
    validate_zero(updated_zero)
    validate_hints(updated_hints)
    validate_initial_pose(updated_initial, new_sha256)
    return updated_zero, updated_hints, updated_initial, new_sha256


def stage_file(path: Path, data: bytes, mode: int) -> Path:
    descriptor, name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, mode)
        return temporary
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def fsync_directory(path: Path) -> None:
    if os.name == "nt":
        return
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def require_secure_production_file(path: Path) -> None:
    information = path.lstat()
    if not stat.S_ISREG(information.st_mode) or path.is_symlink():
        raise ValueError(f"unsafe production file type: {path}")
    if information.st_nlink != 1:
        raise ValueError(f"production file has multiple hard links: {path}")
    if os.name != "nt" and information.st_uid != os.geteuid():
        raise ValueError(f"production file is not owned by the current user: {path}")
    if os.name != "nt" and stat.S_IMODE(information.st_mode) != 0o600:
        raise ValueError(f"production file mode must be 0600: {path}")


def acquire_offline_locks(stack: ExitStack) -> None:
    if os.name == "nt":
        return
    if fcntl is None:
        raise RuntimeError("fcntl is required for production apply")
    flags = (
        os.O_RDWR
        | os.O_CREAT
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    for path in LOCK_PATHS:
        try:
            descriptor = os.open(path, flags, 0o600)
        except OSError as exc:
            raise RuntimeError(f"cannot securely open hardware/build lock: {path}") from exc
        try:
            information = os.fstat(descriptor)
            if (
                not stat.S_ISREG(information.st_mode)
                or information.st_nlink != 1
                or information.st_uid != os.geteuid()
                or stat.S_IMODE(information.st_mode) & 0o022
            ):
                raise RuntimeError(f"unsafe hardware/build lock file: {path}")
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except Exception as exc:
            os.close(descriptor)
            raise RuntimeError(f"hardware/build lock is busy or unsafe: {path}") from exc
        stack.callback(os.close, descriptor)


def write_new_durable(path: Path, data: bytes, mode: int = 0o600) -> None:
    flags = (
        os.O_WRONLY
        | os.O_CREAT
        | os.O_EXCL
        | getattr(os, "O_CLOEXEC", 0)
        | getattr(os, "O_NOFOLLOW", 0)
    )
    descriptor = os.open(path, flags, mode)
    try:
        with os.fdopen(descriptor, "wb", closefd=False) as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        if os.name != "nt":
            os.fchmod(descriptor, mode)
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
    if path.read_bytes() != data:
        raise RuntimeError(f"durable write verification failed: {path}")


def verify_exact_files(expected: dict[Path, bytes], label: str) -> None:
    for path, data in expected.items():
        require_secure_production_file(path)
        if path.read_bytes() != data:
            raise RuntimeError(f"{label} byte verification failed: {path}")


def create_durable_backup(
    backup_dir: Path, originals: dict[Path, bytes]
) -> dict[Path, Path]:
    if backup_dir.exists() or backup_dir.is_symlink():
        raise ValueError("backup directory already exists")
    if not backup_dir.parent.is_dir() or backup_dir.parent.is_symlink():
        raise ValueError("backup parent must be an existing real directory")
    backup_dir.mkdir(exist_ok=False, mode=0o700)
    os.chmod(backup_dir, 0o700)
    fsync_directory(backup_dir.parent)
    backups: dict[Path, Path] = {}
    try:
        for source, data in originals.items():
            destination = backup_dir / source.name
            write_new_durable(destination, data, 0o600)
            backups[source] = destination
        fsync_directory(backup_dir)
        for source, backup in backups.items():
            if backup.read_bytes() != originals[source]:
                raise RuntimeError(f"backup byte verification failed: {backup}")
        return backups
    except BaseException:
        fsync_directory(backup_dir)
        raise


def replace_payloads(payloads: tuple[tuple[Path, bytes], ...]) -> None:
    staged: list[tuple[Path, Path]] = []
    try:
        for destination, data in payloads:
            staged.append((stage_file(destination, data, 0o600), destination))
        for temporary, destination in staged:
            os.replace(temporary, destination)
        fsync_directory(payloads[0][0].parent)
    finally:
        for temporary, _ in staged:
            temporary.unlink(missing_ok=True)


def restore_from_backup(
    *, backups: dict[Path, Path], originals: dict[Path, bytes], ordered_targets: tuple[Path, ...]
) -> None:
    restored_payloads: list[tuple[Path, bytes]] = []
    for target in ordered_targets:
        backup = backups[target]
        data = backup.read_bytes()
        if data != originals[target]:
            raise RuntimeError(f"rollback backup changed: {backup}")
        restored_payloads.append((target, data))
    replace_payloads(tuple(restored_payloads))
    verify_exact_files(originals, "rollback")


def apply_documents(
    *,
    zero_path: Path,
    sidecar_path: Path,
    hint_path: Path,
    initial_path: Path,
    backup_dir: Path,
    zero_data: bytes,
    hint_data: bytes,
    initial_data: bytes,
    sidecar_data: bytes,
    original_data: dict[Path, bytes],
) -> None:
    targets = (zero_path, sidecar_path, hint_path, initial_path)
    if len({path.parent for path in targets}) != 1:
        raise ValueError("all production reference files must share one directory")
    expected_names = {
        zero_path: "persistent_software_zero.json",
        sidecar_path: "persistent_software_zero.json.sha256",
        hint_path: "recovery_branch_hints.json",
        initial_path: "initial_pose.json",
    }
    if any(path.name != expected for path, expected in expected_names.items()):
        raise ValueError("production reference filenames are not canonical")
    if set(original_data) != set(targets):
        raise ValueError("original production byte snapshot is incomplete")
    for path in targets:
        require_secure_production_file(path)
    verify_exact_files(original_data, "pre-backup production snapshot")
    backups = create_durable_backup(backup_dir, original_data)
    ordered_targets = (hint_path, initial_path, zero_path, sidecar_path)
    new_payloads = {
        zero_path: zero_data,
        sidecar_path: sidecar_data,
        hint_path: hint_data,
        initial_path: initial_data,
    }
    replacement_started = False
    try:
        # Re-read under all locks immediately before the first replacement.
        verify_exact_files(original_data, "pre-commit production snapshot")
        replacement_started = True
        replace_payloads(tuple((path, new_payloads[path]) for path in ordered_targets))
        verify_exact_files(new_payloads, "committed production snapshot")
    except BaseException as apply_error:
        if replacement_started:
            try:
                restore_from_backup(
                    backups=backups,
                    originals=original_data,
                    ordered_targets=ordered_targets,
                )
            except BaseException as rollback_error:
                raise RuntimeError(
                    "production update failed and verified backup rollback also failed"
                ) from rollback_error
        else:
            verify_exact_files(original_data, "aborted production snapshot")
        raise apply_error


def current_boot_id() -> str:
    path = Path("/proc/sys/kernel/random/boot_id")
    if not path.is_file():
        raise RuntimeError("current host boot ID is unavailable")
    value = path.read_text(encoding="ascii").strip()
    return normalized_boot_id(value, "current host boot ID")


def current_boottime_ns() -> int:
    if not hasattr(time, "CLOCK_BOOTTIME"):
        raise RuntimeError("CLOCK_BOOTTIME is unavailable")
    value = time.clock_gettime_ns(time.CLOCK_BOOTTIME)
    if type(value) is not int or value <= 0:
        raise RuntimeError("current CLOCK_BOOTTIME is invalid")
    return value


def resolve_real_file(path: Path, label: str) -> Path:
    absolute = Path(os.path.abspath(path))
    if absolute.is_symlink():
        raise ValueError(f"{label} must not be a symbolic link")
    resolved = absolute.resolve(strict=True)
    if not resolved.is_file():
        raise ValueError(f"{label} must be a regular file")
    return resolved


def load_production_documents(
    *,
    zero_path: Path,
    sidecar_path: Path,
    hint_path: Path,
    initial_path: Path,
    expected_zero_sha256: str,
    secure: bool,
) -> tuple[
    dict[str, Any],
    dict[str, Any],
    dict[str, Any],
    dict[Path, bytes],
    str,
]:
    paths = (zero_path, sidecar_path, hint_path, initial_path)
    if secure:
        for path in paths:
            require_secure_production_file(path)
    zero, zero_data, zero_sha256 = read_json(zero_path)
    hints, hint_data, _ = read_json(hint_path)
    initial, initial_data, _ = read_json(initial_path)
    sidecar_data = sidecar_path.read_bytes()
    validate_zero(zero)
    validate_hints(hints)
    if zero_sha256 != expected_zero_sha256:
        raise ValueError("expected current persistent software-zero hash does not match")
    if parse_sidecar_bytes(sidecar_data, zero_path.name) != zero_sha256:
        raise ValueError("persistent software-zero checksum verification failed")
    validate_initial_pose(initial, zero_sha256)
    originals = {
        zero_path: zero_data,
        sidecar_path: sidecar_data,
        hint_path: hint_data,
        initial_path: initial_data,
    }
    return zero, hints, initial, originals, zero_sha256


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zero-file", type=Path, required=True)
    parser.add_argument("--zero-sha256-file", type=Path, required=True)
    parser.add_argument("--recovery-hint-file", type=Path, required=True)
    parser.add_argument("--initial-pose-file", type=Path, required=True)
    parser.add_argument("--j2-capture", type=Path, required=True)
    parser.add_argument("--go-aux-capture", type=Path, required=True)
    parser.add_argument("--j6-capture", type=Path, required=True)
    parser.add_argument("--expected-current-sha256", required=True)
    parser.add_argument("--expected-j2-capture-sha256", required=True)
    parser.add_argument("--expected-go-aux-capture-sha256", required=True)
    parser.add_argument("--expected-j6-capture-sha256", required=True)
    parser.add_argument("--expected-pose-binding-id", required=True)
    parser.add_argument("--expected-host-boot-id", required=True)
    parser.add_argument("--recorded-at-utc", required=True)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--backup-dir", type=Path)
    return parser.parse_args(argv)


def main() -> int:
    args = parse_args()
    if args.apply and args.confirm != APPLY_GATE:
        raise SystemExit("whole-arm rebase apply confirmation gate is missing")
    if args.apply and args.backup_dir is None:
        raise SystemExit("--backup-dir is required in apply mode")
    host_boot_id = normalized_boot_id(args.expected_host_boot_id, "expected host boot ID")
    pose_binding_id = normalized_sha256(
        args.expected_pose_binding_id, "expected pose_binding_id"
    )
    expected_current_sha256 = normalized_sha256(
        args.expected_current_sha256, "expected current persistent zero sha256"
    )
    zero_path = resolve_real_file(args.zero_file, "persistent zero")
    sidecar_path = resolve_real_file(args.zero_sha256_file, "persistent zero sidecar")
    hint_path = resolve_real_file(args.recovery_hint_file, "recovery hints")
    initial_path = resolve_real_file(args.initial_pose_file, "initial pose")
    capture_paths = {
        "j2": resolve_real_file(args.j2_capture, "J2 capture"),
        "go_aux": resolve_real_file(args.go_aux_capture, "GO-AUX capture"),
        "j6": resolve_real_file(args.j6_capture, "J6 capture"),
    }
    if args.apply:
        for path in capture_paths.values():
            require_secure_production_file(path)
    capture_inputs = {name: read_json(path) for name, path in capture_paths.items()}
    captures = {name: record[0] for name, record in capture_inputs.items()}
    capture_hashes = {name: record[2] for name, record in capture_inputs.items()}
    expected_capture_hashes = {
        "j2": normalized_sha256(args.expected_j2_capture_sha256, "expected J2 capture sha256"),
        "go_aux": normalized_sha256(
            args.expected_go_aux_capture_sha256, "expected GO-AUX capture sha256"
        ),
        "j6": normalized_sha256(args.expected_j6_capture_sha256, "expected J6 capture sha256"),
    }
    if capture_hashes != expected_capture_hashes:
        raise SystemExit("one or more capture SHA-256 values do not match")

    def prepare_and_optionally_apply(current_clock_ns: int | None) -> tuple[
        str,
        str,
        dict[str, dict[str, float | int]],
        dict[str, Any],
    ]:
        measurements, identities = validate_capture_bundle(
            captures["j2"],
            captures["go_aux"],
            captures["j6"],
            host_boot_id=host_boot_id,
            pose_binding_id=pose_binding_id,
            current_boottime_ns=current_clock_ns,
        )
        zero, hints, initial, originals, actual_sha256 = load_production_documents(
            zero_path=zero_path,
            sidecar_path=sidecar_path,
            hint_path=hint_path,
            initial_path=initial_path,
            expected_zero_sha256=expected_current_sha256,
            secure=args.apply,
        )
        capture_ids = {
            name: identity["capture_id"] for name, identity in identities.items()
        }
        updated_zero, updated_hints, updated_initial, new_sha256 = build_documents(
            zero,
            hints,
            initial,
            original_sha256=actual_sha256,
            measurements=measurements,
            capture_hashes=capture_hashes,
            capture_ids=capture_ids,
            pose_binding_id=pose_binding_id,
            recorded_at_utc=args.recorded_at_utc,
            host_boot_id=host_boot_id,
        )
        zero_data = json_bytes(updated_zero)
        hint_data = json_bytes(updated_hints)
        initial_data = json_bytes(updated_initial)
        sidecar_data = f"{new_sha256}  {zero_path.name}\n".encode("ascii")
        if args.apply:
            apply_documents(
                zero_path=zero_path,
                sidecar_path=sidecar_path,
                hint_path=hint_path,
                initial_path=initial_path,
                backup_dir=args.backup_dir.resolve(),
                zero_data=zero_data,
                hint_data=hint_data,
                initial_data=initial_data,
                sidecar_data=sidecar_data,
                original_data=originals,
            )
        return actual_sha256, new_sha256, measurements, capture_ids

    if args.apply:
        with ExitStack() as stack:
            acquire_offline_locks(stack)
            if current_boot_id() != host_boot_id:
                raise SystemExit("capture host boot ID is not the current host boot ID")
            actual_sha256, new_sha256, measurements, capture_ids = (
                prepare_and_optionally_apply(current_boottime_ns())
            )
    else:
        actual_sha256, new_sha256, measurements, capture_ids = (
            prepare_and_optionally_apply(None)
        )
    print(
        json.dumps(
            {
                "schema": "go-m8010-whole-arm-software-zero-rebase-result/1.0",
                "applied": bool(args.apply),
                "hardware_opened": False,
                "motor_internal_zero_modified": False,
                "rid_written": False,
                "flash_or_eeprom_written": False,
                "old_persistent_zero_sha256": actual_sha256,
                "new_persistent_zero_sha256": new_sha256,
                "host_boot_id": host_boot_id,
                "pose_binding_id": pose_binding_id,
                "capture_id": capture_ids,
                "capture_sha256": capture_hashes,
                "new_logical_initial_pose_rad": {f"J{index}": 0.0 for index in range(1, 7)},
                "measurements": measurements,
                "backup_dir": str(args.backup_dir.resolve()) if args.apply else None,
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

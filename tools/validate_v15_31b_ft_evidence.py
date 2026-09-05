#!/usr/bin/env python3
"""Offline, fail-closed validator for the V15.31B-FT evidence bundle.

The validator never opens CAN, serial, ROS, or a hardware worker.  It validates
only committed-style JSON/CSV evidence, its checksum manifest, and Git commit
objects already present in the supplied repository.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import subprocess
import sys
from collections import defaultdict
from datetime import datetime
from pathlib import Path
from typing import Any, Iterable, Mapping, Sequence


EXPECTED_BRANCH = "agent/v15-31b-ft-powered-position-thermal-validation"
SOURCE_COMMIT = "60f0272fc4b54b86a87eec1012af1c3c92ca7924"
IMPLEMENTATION_COMMIT_SUBJECT = (
    "fix: validate powered gravity-aware position control"
)
EVIDENCE_SEAL_COMMIT_SUBJECT = (
    "test: validate powered whole-arm position and thermal stability"
)
PRODUCTION_MODEL_SHA256 = (
    "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
)
GRAVITY_ANCHOR_SCHEMA = "go-m8010-gravity-model-anchor-v2/2.0"
POWER_ON_SCHEMA = "go-m8010-v15-31b-power-on-readonly/1.0"
HARDWARE_STATE_SCHEMA = "go-m8010-hardware-state/1.1"
GRAVITY_READONLY_SCHEMA = "V15.31B-gravity-readonly-v1"
GRAVITY_CONFIG_SHA256 = (
    "307469b8384fd35547327ba1d5f80aa440e6b9663406ab7c9d9469bea263335d"
)
GO_GEAR_RATIO = 6.329999923706055
MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
GO_MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5")
JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
POSITION_ORDER = ("J1", "J6", "J5", "J4", "J3", "J2")
MOTOR_BY_JOINT = {
    "J1": ("J1",),
    "J2": ("J2A", "J2B"),
    "J3": ("J3",),
    "J4": ("J4",),
    "J5": ("J5",),
    "J6": ("J6",),
}
DOMAIN_BY_JOINT = {
    "J1": "J1",
    "J2": "J2",
    "J3": "J345",
    "J4": "J345",
    "J5": "J345",
    "J6": "J6",
}
FROZEN_MOTOR_SIGNS = {
    "J1": +1,
    "J2A": -1,
    "J2B": +1,
    "J3": +1,
    "J4": -1,
    "J5": +1,
    "J6": -1,
}

HASHED_ARTIFACTS = tuple(sorted((
    "power_on_readonly.json",
    "model_session_anchor_validation.json",
    "gravity_readonly_validation.json",
    "gravity_scale_validation.csv",
    "position_validation.csv",
    "j2_control_comparison.csv",
    "multi_joint_validation.json",
    "thermal_5min.csv",
    "thermal_15min.csv",
    "thermal_30min.csv",
    "thermal_summary.json",
    "gui_powered_validation.json",
    "final_result.json",
)))
REQUIRED_ARTIFACTS = HASHED_ARTIFACTS + ("SHA256SUMS",)

FINAL_FIELD_NAMES = (
    "V15.31B RESULT",
    "branch",
    "implementation commit",
    "evidence commit",
    "Power-on read-only",
    "Anchor",
    "Gravity calculation",
    "Gravity powered result",
    "J1 error",
    "J2 error",
    "J2 max sync",
    "J3 error",
    "J4 error",
    "J5 error",
    "J6 error",
    "multi-joint",
    "J2A",
    "J2B",
    "J2 sustained rotor torque",
    "saturation ratio",
    "thermal classification",
    "counterbalance required",
    "GUI powered result",
    "restore initial",
    "final brake",
    "motor zero modified",
    "MuJoCo modified",
    "git clean",
    "PRIMARY BLOCKER",
    "FINAL TASK RESULT",
    "READY_FOR_NEXT_STAGE",
)

POSITION_PHASES = (
    "CENTER_START",
    "PLUS_5",
    "CENTER_AFTER_PLUS",
    "MINUS_5",
    "CENTER_FINAL",
)
REVISION_ORDER = {"BASELINE": 0, "REVISION_1": 1, "REVISION_2": 2}
GRAVITY_LEVELS = (0.0, 0.25, 0.50, 0.75, 1.0)
READ_ONLY_MINIMUM_DURATION_NS = 10_000_000_000
READ_ONLY_MINIMUM_SAMPLES = 100
READ_ONLY_MAXIMUM_SAMPLE_GAP_NS = 250_000_000
READ_ONLY_MAXIMUM_STATIONARY_VELOCITY_RAD_S = math.radians(0.25)
READ_ONLY_MAXIMUM_J2_SYNC_RAD = math.radians(0.25)
THERMAL_MAXIMUM_SAMPLE_GAP_NS = 2_000_000_000
THERMAL_ELAPSED_MONOTONIC_TOLERANCE_S = 0.010
MINIMUM_RELATIVE_PD_IMPROVEMENT = 0.10
MINIMUM_ABSOLUTE_PD_IMPROVEMENT_NM = 0.05
MAXIMUM_STABLE_THERMAL_SLOPE_C_PER_MIN = 0.5
MINIMUM_THERMAL_SLOPE_IMPROVEMENT_C_PER_MIN = 0.05
MINIMUM_EARLY_FUNDAMENTAL_SLOPE_C_PER_MIN = 0.25
MINIMUM_EARLY_FUNDAMENTAL_RISE_C = 1.0
MECHANICAL_UNLOAD_FRACTION = 0.20
STANDARD_GRAVITY_M_S2 = 9.80665
TRACE_MAXIMUM_SAMPLE_GAP_NS = 100_000_000
TRACE_MINIMUM_DWELL_NS = 500_000_000
TRACE_MINIMUM_DWELL_SAMPLES = 6
MAXIMUM_RECEIPT_AGE_NS = 250_000_000
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
COMMIT_RE = re.compile(r"^[0-9a-f]{40}$")
PLACEHOLDER_RE = re.compile(
    r"(?:THIS_FILE_IS_PART_OF_COMMIT|SAME_AS_COMMIT|"
    r"RESOLVABLE_SHA_REPORTED|VERIFIED_AFTER_COMMIT|"
    r"\bPLACEHOLDER\b|\bTBD\b|\bTODO\b|<[^>]*(?:sha|commit)[^>]*>)",
    re.IGNORECASE,
)
NONPASS_PREFIXES = (
    "NOT_RUN",
    "BLOCKED",
    "PENDING",
    "FAIL",
    "ABORTED",
    "UNDETERMINED",
    "PARTIAL_PASS",
    "HARDWARE_COUNTERBALANCE_REQUIRED",
)


class EvidenceValidationError(ValueError):
    """A bounded validation failure suitable for CLI reporting."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise EvidenceValidationError(message)


def _no_duplicate_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise EvidenceValidationError(f"JSON contains duplicate key: {key}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    raise EvidenceValidationError(f"JSON contains non-finite constant: {value}")


def _load_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(
            path.read_text(encoding="utf-8-sig"),
            object_pairs_hook=_no_duplicate_object,
            parse_constant=_reject_json_constant,
        )
    except (OSError, UnicodeError, json.JSONDecodeError) as exc:
        raise EvidenceValidationError(f"cannot parse JSON {path.name}: {exc}") from exc
    _require(isinstance(value, dict), f"{path.name}: JSON root must be an object")
    return value


def _read_csv(path: Path, required_fields: Iterable[str]) -> list[dict[str, str]]:
    try:
        with path.open("r", encoding="utf-8-sig", newline="") as stream:
            reader = csv.DictReader(stream, strict=True)
            fieldnames = reader.fieldnames
            _require(fieldnames is not None, f"{path.name}: CSV header is missing")
            _require(
                len(fieldnames) == len(set(fieldnames)),
                f"{path.name}: CSV header contains duplicate fields",
            )
            expected_fields = tuple(required_fields)
            _require(
                tuple(fieldnames) == expected_fields,
                f"{path.name}: CSV fields/order must exactly match the evidence schema",
            )
            rows = list(reader)
    except (OSError, UnicodeError, csv.Error) as exc:
        raise EvidenceValidationError(f"cannot parse CSV {path.name}: {exc}") from exc
    _require(rows, f"{path.name}: CSV contains no evidence rows")
    for index, row in enumerate(rows, start=2):
        _require(None not in row, f"{path.name}:{index}: row has surplus columns")
    return rows


def _number(value: Any, field: str) -> float:
    _require(
        type(value) in {int, float} and math.isfinite(float(value)),
        f"{field}: finite number required",
    )
    return float(value)


def _integer(value: Any, field: str, *, minimum: int | None = None) -> int:
    _require(type(value) is int, f"{field}: integer required")
    if minimum is not None:
        _require(value >= minimum, f"{field}: must be >= {minimum}")
    return value


def _row_number(row: Mapping[str, str], field: str, location: str) -> float:
    text = row.get(field, "").strip()
    try:
        value = float(text)
    except ValueError as exc:
        raise EvidenceValidationError(f"{location}.{field}: finite number required") from exc
    _require(math.isfinite(value), f"{location}.{field}: finite number required")
    return value


def _row_integer(row: Mapping[str, str], field: str, location: str) -> int:
    text = row.get(field, "").strip()
    _require(re.fullmatch(r"-?[0-9]+", text) is not None, f"{location}.{field}: integer required")
    return int(text)


def _row_bool(row: Mapping[str, str], field: str, location: str) -> bool:
    value = row.get(field, "").strip().upper()
    _require(value in {"TRUE", "FALSE", "YES", "NO", "1", "0"}, f"{location}.{field}: boolean required")
    return value in {"TRUE", "YES", "1"}


def _timestamp(value: Any, field: str) -> datetime:
    _require(isinstance(value, str) and value.strip(), f"{field}: timestamp required")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise EvidenceValidationError(f"{field}: invalid ISO-8601 timestamp") from exc
    _require(parsed.tzinfo is not None, f"{field}: timezone is required")
    return parsed


def _status_token(value: Any, field: str) -> str:
    if isinstance(value, str):
        token = value.strip().upper()
    elif isinstance(value, Mapping):
        candidate = value.get("status", value.get("result"))
        _require(isinstance(candidate, str), f"{field}: structured status is missing")
        token = candidate.strip().upper()
    else:
        raise EvidenceValidationError(f"{field}: status string or object required")
    _require(token, f"{field}: status must not be empty")
    return token


def _is_pass(value: Any, field: str = "status") -> bool:
    return _status_token(value, field) in {"PASS", "FULL_PASS"}


def _is_nonpass(value: Any, field: str = "status") -> bool:
    token = _status_token(value, field)
    return any(token.startswith(prefix) for prefix in NONPASS_PREFIXES)


def _document_status(document: Mapping[str, Any], filename: str) -> str:
    value = document.get("result", document.get("status"))
    _require(value is not None, f"{filename}: result/status is required")
    token = _status_token(value, f"{filename}.result")
    _require(
        token in {"PASS", "FULL_PASS"}
        or any(token.startswith(prefix) for prefix in NONPASS_PREFIXES),
        f"{filename}: unsupported result {token!r}",
    )
    if token not in {"PASS", "FULL_PASS"}:
        reason = document.get("reason", document.get("primary_blocker"))
        _require(isinstance(reason, str) and reason.strip(), f"{filename}: non-PASS evidence requires a reason")
    return token


def _exact_keys(value: Any, keys: Sequence[str], field: str) -> Mapping[str, Any]:
    _require(isinstance(value, Mapping), f"{field}: object required")
    _require(set(value) == set(keys), f"{field}: keys must be exactly {sorted(keys)}")
    return value


def _finite_vector(value: Any, length: int, field: str) -> tuple[float, ...]:
    _require(isinstance(value, list) and len(value) == length, f"{field}: {length} values required")
    return tuple(_number(item, f"{field}[{index}]") for index, item in enumerate(value))


def _parse_json_text(value: Any, field: str) -> Any:
    _require(isinstance(value, str) and value.strip(), f"{field}: JSON text required")
    try:
        return json.loads(
            value,
            object_pairs_hook=_no_duplicate_object,
            parse_constant=_reject_json_constant,
        )
    except (json.JSONDecodeError, TypeError) as exc:
        raise EvidenceValidationError(f"{field}: invalid JSON: {exc}") from exc


def _canonical_document_sha256(value: Any) -> str:
    try:
        payload = json.dumps(
            value,
            ensure_ascii=True,
            allow_nan=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("ascii")
    except (TypeError, ValueError, UnicodeEncodeError) as exc:
        raise EvidenceValidationError(f"cannot canonicalize evidence JSON: {exc}") from exc
    return hashlib.sha256(payload).hexdigest()


def _require_document_sha(value: Any, persisted: Any, field: str) -> str:
    _require(
        isinstance(persisted, str) and SHA256_RE.fullmatch(persisted) is not None,
        f"{field}: SHA-256 required",
    )
    actual = _canonical_document_sha256(value)
    _require(actual == persisted, f"{field}: canonical SHA-256 mismatch")
    return actual


def _expected_binding_tuple(value: Mapping[str, Any]) -> tuple[Any, ...]:
    return tuple(
        value.get(field)
        for field in (
            "envelope_id",
            "envelope_sha256",
            "session_id",
            "state_instance_id",
            "anchor_sha256",
        )
    )


def _validate_binding_columns(
    row: Mapping[str, str], expected: Mapping[str, Any], location: str
) -> None:
    actual = tuple(
        row.get(field, "").strip()
        for field in (
            "envelope_id",
            "envelope_sha256",
            "session_id",
            "state_instance_id",
            "anchor_sha256",
        )
    )
    _require(actual == _expected_binding_tuple(expected), f"{location}: evidence binding mismatch")
    _require(
        SHA256_RE.fullmatch(actual[1]) is not None
        and SHA256_RE.fullmatch(actual[4]) is not None,
        f"{location}: evidence binding SHA invalid",
    )


def _finite_summary(value: Any, field: str) -> None:
    if type(value) in {int, float}:
        _number(value, field)
        return
    _require(isinstance(value, Mapping) and value, f"{field}: number or non-empty numeric summary required")
    observed = 0
    for key, item in value.items():
        if item is None:
            continue
        _number(item, f"{field}.{key}")
        observed += 1
    _require(observed > 0, f"{field}: numeric summary is empty")


def _strictly_increasing(values: Sequence[int], field: str) -> None:
    _require(values, f"{field}: no timestamps")
    _require(all(current > previous for previous, current in zip(values, values[1:])), f"{field}: timestamps must be strictly increasing")


def _validate_no_placeholders(evidence_dir: Path) -> None:
    for filename in HASHED_ARTIFACTS:
        path = evidence_dir / filename
        try:
            text = path.read_text(encoding="utf-8-sig")
        except (OSError, UnicodeError) as exc:
            raise EvidenceValidationError(f"cannot scan {filename}: {exc}") from exc
        match = PLACEHOLDER_RE.search(text)
        _require(match is None, f"{filename}: prohibited placeholder text {match.group(0)!r}" if match else "")


def _validate_manifest(evidence_dir: Path) -> None:
    path = evidence_dir / "SHA256SUMS"
    try:
        lines = path.read_text(encoding="ascii").splitlines()
    except (OSError, UnicodeError) as exc:
        raise EvidenceValidationError(f"cannot read SHA256SUMS: {exc}") from exc
    _require(len(lines) == 13, "SHA256SUMS: exactly 13 entries required")
    pattern = re.compile(r"^([0-9a-f]{64})  ([A-Za-z0-9_.-]+)$")
    entries: list[tuple[str, str]] = []
    for index, line in enumerate(lines, start=1):
        match = pattern.fullmatch(line)
        _require(match is not None, f"SHA256SUMS:{index}: malformed entry")
        assert match is not None
        entries.append((match.group(1), match.group(2)))
    names = tuple(name for _digest, name in entries)
    _require(names == HASHED_ARTIFACTS, "SHA256SUMS: entries must be the exact 13 sorted evidence filenames")
    for expected, filename in entries:
        actual = hashlib.sha256((evidence_dir / filename).read_bytes()).hexdigest()
        _require(actual == expected, f"SHA256SUMS: digest mismatch for {filename}")


def _validate_power_on(document: Mapping[str, Any]) -> tuple[str, dict[str, Any]]:
    status = _document_status(document, "power_on_readonly.json")
    if status not in {"PASS", "FULL_PASS"}:
        return status, {}
    _require(document.get("schema") == POWER_ON_SCHEMA, "power_on_readonly schema mismatch")
    _require(
        document.get("model_sha256") == PRODUCTION_MODEL_SHA256,
        "power_on_readonly model SHA mismatch",
    )
    duration = _number(document.get("duration_s"), "power_on_readonly.duration_s")
    _require(duration >= 10.0, "power_on_readonly: continuous duration must be >= 10 s")
    frame_count = _integer(
        document.get("valid_frame_count"),
        "power_on_readonly.valid_frame_count",
        minimum=READ_ONLY_MINIMUM_SAMPLES,
    )
    session_id = document.get("session_id")
    state_instance_id = document.get("state_instance_id")
    _require(isinstance(session_id, str) and session_id, "power_on_readonly.session_id required")
    _require(
        isinstance(state_instance_id, str)
        and re.fullmatch(r"[0-9a-f]{32}", state_instance_id) is not None,
        "power_on_readonly.state_instance_id must be 32 lowercase hex characters",
    )
    samples = document.get("samples")
    _require(
        isinstance(samples, list) and len(samples) == frame_count,
        "power_on_readonly valid_frame_count must match retained samples",
    )
    sequences: list[int] = []
    source_stamps: list[int] = []
    sample_modes: dict[str, set[str]] = {name: set() for name in MOTOR_NAMES}
    maximum_age_by_motor = {name: 0.0 for name in MOTOR_NAMES}
    maximum_temperature_by_motor = {name: -math.inf for name in MOTOR_NAMES}
    maximum_velocity_by_motor = {name: 0.0 for name in MOTOR_NAMES}
    maximum_abs_tau_command_by_motor = {name: 0.0 for name in GO_MOTOR_NAMES}
    maximum_sample_sync_rad = 0.0
    for index, sample in enumerate(samples):
        field = f"power_on_readonly.samples[{index}]"
        _require(isinstance(sample, Mapping), f"{field}: object required")
        _require(sample.get("schema") == HARDWARE_STATE_SCHEMA, f"{field}: schema mismatch")
        _require(sample.get("session_id") == session_id, f"{field}: session changed")
        _require(
            sample.get("state_instance_id") == state_instance_id,
            f"{field}: state instance changed",
        )
        sequences.append(_integer(sample.get("sequence"), f"{field}.sequence", minimum=1))
        source_stamps.append(
            _integer(
                sample.get("source_monotonic_ns"),
                f"{field}.source_monotonic_ns",
                minimum=1,
            )
        )
        _finite_vector(sample.get("position_rad"), 6, f"{field}.position_rad")
        velocities = _finite_vector(
            sample.get("velocity_rad_s"), 6, f"{field}.velocity_rad_s"
        )
        _require(
            max(abs(value) for value in velocities)
            <= READ_ONLY_MAXIMUM_STATIONARY_VELOCITY_RAD_S,
            f"{field}: mechanism is not stationary",
        )
        sync_rad = abs(_number(sample.get("j2_e_sync_rad"), f"{field}.j2_e_sync_rad"))
        maximum_sample_sync_rad = max(maximum_sample_sync_rad, sync_rad)
        _require(
            sync_rad <= READ_ONLY_MAXIMUM_J2_SYNC_RAD,
            f"{field}: J2 synchronization warning threshold exceeded",
        )
        _require(sample.get("j2_sync_fault") is False, f"{field}: J2 sync fault")
        modes = _exact_keys(
            sample.get("controller_mode_by_motor"),
            MOTOR_NAMES,
            f"{field}.controller_mode_by_motor",
        )
        sample_motors = _exact_keys(
            sample.get("per_motor"), MOTOR_NAMES, f"{field}.per_motor"
        )
        for motor_index, name in enumerate(MOTOR_NAMES):
            allowed_modes = {"brake"} if name != "J6" else {"brake", "disabled"}
            mode = str(modes[name]).strip().lower()
            _require(mode in allowed_modes, f"{field}.{name}: active mode observed")
            sample_modes[name].add(mode)
            record = sample_motors[name]
            _require(isinstance(record, Mapping), f"{field}.{name}: object required")
            _require(
                record.get("communication_ok") is True and record.get("fresh") is True,
                f"{field}.{name}: feedback is not fresh",
            )
            _require(
                _integer(record.get("merror"), f"{field}.{name}.merror", minimum=0)
                == 0,
                f"{field}.{name}: nonzero merror",
            )
            age = _number(record.get("age_ms"), f"{field}.{name}.age_ms")
            _require(0.0 <= age <= 250.0, f"{field}.{name}: feedback age exceeds 250 ms")
            temperature = _number(
                record.get("temperature_c"), f"{field}.{name}.temperature_c"
            )
            _require(
                0.0 <= temperature < 60.0,
                f"{field}.{name}: temperature outside read-only bounds",
            )
            raw = _number(record.get("raw_position_rad"), f"{field}.{name}.raw_position_rad")
            del raw
            logical = _number(record.get("q_joint_rad"), f"{field}.{name}.q_joint_rad")
            velocity = _number(
                record.get("dq_joint_rad_s"), f"{field}.{name}.dq_joint_rad_s"
            )
            _require(
                math.isclose(velocity, velocities[motor_index if motor_index < 2 else min(motor_index - 1, 5)], abs_tol=1.0e-9)
                or name in {"J2A", "J2B"},
                f"{field}.{name}: per-motor velocity is inconsistent",
            )
            del logical
            maximum_age_by_motor[name] = max(maximum_age_by_motor[name], age)
            maximum_temperature_by_motor[name] = max(
                maximum_temperature_by_motor[name], temperature
            )
            maximum_velocity_by_motor[name] = max(
                maximum_velocity_by_motor[name], abs(velocity)
            )
            if name == "J6":
                for torque_field in (
                    "tau_cmd_rotor_nm",
                    "tau_feedback_rotor_nm",
                    "tau_joint_estimated_nm",
                ):
                    _require(
                        record.get(torque_field) is None,
                        f"{field}.J6.{torque_field}: must be null",
                    )
            else:
                tau_command = _number(
                    record.get("tau_cmd_rotor_nm"),
                    f"{field}.{name}.tau_cmd_rotor_nm",
                )
                _number(
                    record.get("tau_feedback_rotor_nm"),
                    f"{field}.{name}.tau_feedback_rotor_nm",
                )
                _number(
                    record.get("tau_joint_estimated_nm"),
                    f"{field}.{name}.tau_joint_estimated_nm",
                )
                maximum_abs_tau_command_by_motor[name] = max(
                    maximum_abs_tau_command_by_motor[name], abs(tau_command)
                )
                _require(
                    abs(tau_command) <= 1.0e-12,
                    f"{field}.{name}: nonzero active torque command",
                )
    _strictly_increasing(sequences, "power_on_readonly.samples.sequence")
    _strictly_increasing(source_stamps, "power_on_readonly.samples.source_monotonic_ns")
    _require(
        all(
            current - previous <= READ_ONLY_MAXIMUM_SAMPLE_GAP_NS
            for previous, current in zip(source_stamps, source_stamps[1:])
        ),
        "power_on_readonly: telemetry gap exceeds 250 ms",
    )
    observed_duration_ns = source_stamps[-1] - source_stamps[0]
    _require(
        observed_duration_ns >= READ_ONLY_MINIMUM_DURATION_NS,
        "power_on_readonly: retained samples cover less than 10 seconds",
    )
    _require(
        math.isclose(duration, observed_duration_ns * 1.0e-9, abs_tol=1.0e-6),
        "power_on_readonly: duration_s does not match retained samples",
    )
    motors = _exact_keys(document.get("per_motor"), MOTOR_NAMES, "power_on_readonly.per_motor")
    for name in MOTOR_NAMES:
        motor = motors[name]
        _require(isinstance(motor, Mapping), f"power_on_readonly.{name}: object required")
        _require(motor.get("online") is True, f"power_on_readonly.{name}: must remain online")
        age = _number(motor.get("max_feedback_age_ms"), f"power_on_readonly.{name}.max_feedback_age_ms")
        _require(0.0 <= age <= 250.0, f"power_on_readonly.{name}: feedback age exceeds 250 ms")
        _finite_summary(motor.get("raw_position_rad"), f"power_on_readonly.{name}.raw_position_rad")
        _finite_summary(motor.get("logical_position_rad"), f"power_on_readonly.{name}.logical_position_rad")
        _number(motor.get("max_abs_velocity_rad_s"), f"power_on_readonly.{name}.max_abs_velocity_rad_s")
        _number(motor.get("max_temperature_c"), f"power_on_readonly.{name}.max_temperature_c")
        _require(motor.get("abnormal_velocity_detected") is False, f"power_on_readonly.{name}: abnormal velocity detected")
        _require(_integer(motor.get("max_abs_merror"), f"power_on_readonly.{name}.max_abs_merror", minimum=0) == 0, f"power_on_readonly.{name}: merror must stay zero")
        _require(_integer(motor.get("communication_interruptions"), f"power_on_readonly.{name}.communication_interruptions", minimum=0) == 0, f"power_on_readonly.{name}: communication interruption")
        modes = motor.get("observed_modes")
        _require(isinstance(modes, list) and modes, f"power_on_readonly.{name}.observed_modes required")
        normalized_modes = {str(item).strip().lower() for item in modes}
        allowed_modes = {"brake"} if name != "J6" else {"brake", "disabled"}
        _require(normalized_modes <= allowed_modes, f"power_on_readonly.{name}: active mode observed")
        _require(
            normalized_modes == sample_modes[name],
            f"power_on_readonly.{name}: observed_modes disagrees with samples",
        )
        _require(
            math.isclose(age, maximum_age_by_motor[name], abs_tol=1.0e-9),
            f"power_on_readonly.{name}: max_feedback_age_ms disagrees with samples",
        )
        _require(
            math.isclose(
                _number(
                    motor.get("max_temperature_c"),
                    f"power_on_readonly.{name}.max_temperature_c",
                ),
                maximum_temperature_by_motor[name],
                abs_tol=1.0e-9,
            ),
            f"power_on_readonly.{name}: max_temperature_c disagrees with samples",
        )
        if name == "J6":
            for field in ("tau_cmd_rotor_nm", "tau_feedback_rotor_nm", "tau_joint_estimated_nm"):
                _require(motor.get(field) is None, f"power_on_readonly.J6.{field}: POS_VEL authority must be null")
        else:
            _finite_summary(motor.get("tau_feedback_rotor_nm"), f"power_on_readonly.{name}.tau_feedback_rotor_nm")
            _finite_summary(motor.get("tau_joint_estimated_nm"), f"power_on_readonly.{name}.tau_joint_estimated_nm")
            _require(_number(motor.get("max_abs_tau_cmd_rotor_nm"), f"power_on_readonly.{name}.max_abs_tau_cmd_rotor_nm") <= 1.0e-12, f"power_on_readonly.{name}: nonzero active torque command in read-only phase")
            _require(
                math.isclose(
                    _number(
                        motor.get("max_abs_tau_cmd_rotor_nm"),
                        f"power_on_readonly.{name}.max_abs_tau_cmd_rotor_nm",
                    ),
                    maximum_abs_tau_command_by_motor[name],
                    abs_tol=1.0e-12,
                ),
                f"power_on_readonly.{name}: torque-command summary disagrees with samples",
            )
    j2 = document.get("j2")
    _require(isinstance(j2, Mapping), "power_on_readonly.j2 object required")
    _finite_summary(j2.get("logical_position_rad"), "power_on_readonly.j2.logical_position_rad")
    sync = _number(j2.get("max_abs_e_sync_deg"), "power_on_readonly.j2.max_abs_e_sync_deg")
    _require(sync <= 0.25, "power_on_readonly: J2 sync warning threshold exceeded")
    _require(
        math.isclose(sync, math.degrees(maximum_sample_sync_rad), abs_tol=1.0e-9),
        "power_on_readonly: J2 sync summary disagrees with samples",
    )
    _finite_summary(j2.get("j2a_logical_torque_contribution_nm"), "power_on_readonly.j2.j2a_logical_torque_contribution_nm")
    _finite_summary(j2.get("j2b_logical_torque_contribution_nm"), "power_on_readonly.j2.j2b_logical_torque_contribution_nm")
    safety = document.get("safety")
    _require(isinstance(safety, Mapping), "power_on_readonly.safety object required")
    for field in ("position_enabled", "motor_internal_zero_modified", "flash_written", "eeprom_written"):
        _require(safety.get(field) is False, f"power_on_readonly.safety.{field} must be false")
    _require(_integer(safety.get("active_command_count"), "power_on_readonly.safety.active_command_count", minimum=0) == 0, "power_on_readonly: active command was sent")
    operator = document.get("operator_confirmation")
    _require(isinstance(operator, Mapping), "power_on_readonly.operator_confirmation required")
    _timestamp(operator.get("confirmed_at_utc"), "power_on_readonly.operator_confirmation.confirmed_at_utc")
    for field in (
        "mechanism_stationary",
        "model_pose_aligned",
        "support_reliable",
        "not_at_mechanical_limit",
        "operator_stop_ready",
    ):
        _require(operator.get(field) is True, f"power_on_readonly operator gate failed: {field}")
    writes = document.get("writes")
    _require(isinstance(writes, Mapping), "power_on_readonly.writes required")
    for field in (
        "motor_internal_zero_modified",
        "rid_written",
        "flash_or_eeprom_written",
    ):
        _require(writes.get(field) is False, f"power_on_readonly prohibited write: {field}")
    return status, {"session_id": session_id, "state_instance_id": state_instance_id, "j2_max_sync_deg": sync}


def _validate_anchor(
    document: Mapping[str, Any],
    power: Mapping[str, Any],
    power_file_sha256: str,
) -> tuple[str, dict[str, Any]]:
    status = _document_status(document, "model_session_anchor_validation.json")
    if status not in {"PASS", "FULL_PASS"}:
        return status, {}
    anchor = _exact_keys(
        document.get("anchor"),
        (
            "schema", "model_sha256", "session_id", "state_instance_id",
            "created_utc", "gear_ratio", "motor_direction_sign",
            "motor_raw_reference_rad", "motor_encoder_branch",
            "logical_joint_reference_rad", "model_absolute_joint_rad",
        ),
        "model_session_anchor_validation.anchor",
    )
    _require(anchor["schema"] == GRAVITY_ANCHOR_SCHEMA, "anchor schema mismatch")
    _require(anchor["model_sha256"] == PRODUCTION_MODEL_SHA256, "anchor model SHA mismatch")
    _require(anchor["session_id"] == power.get("session_id"), "anchor session_id does not match read-only session")
    _require(anchor["state_instance_id"] == power.get("state_instance_id"), "anchor state_instance_id does not match read-only session")
    _timestamp(anchor["created_utc"], "anchor.created_utc")
    ratio = _number(anchor["gear_ratio"], "anchor.gear_ratio")
    _require(abs(ratio - GO_GEAR_RATIO) <= 1.0e-12, "anchor gear ratio mismatch")
    signs = _exact_keys(anchor["motor_direction_sign"], MOTOR_NAMES, "anchor.motor_direction_sign")
    _require(dict(signs) == FROZEN_MOTOR_SIGNS, "anchor motor signs mismatch")
    raw = _exact_keys(anchor["motor_raw_reference_rad"], MOTOR_NAMES, "anchor.motor_raw_reference_rad")
    branches = _exact_keys(anchor["motor_encoder_branch"], MOTOR_NAMES, "anchor.motor_encoder_branch")
    for name in MOTOR_NAMES:
        _number(raw[name], f"anchor.motor_raw_reference_rad.{name}")
        _integer(branches[name], f"anchor.motor_encoder_branch.{name}")
    _finite_vector(anchor["logical_joint_reference_rad"], 6, "anchor.logical_joint_reference_rad")
    _finite_vector(anchor["model_absolute_joint_rad"], 6, "anchor.model_absolute_joint_rad")
    anchor_sha = document.get("runtime_anchor_sha256")
    _require(isinstance(anchor_sha, str) and SHA256_RE.fullmatch(anchor_sha), "runtime_anchor_sha256 must be lowercase SHA256")
    _require(
        document.get("anchor_sha256") == anchor_sha,
        "anchor_sha256 and runtime_anchor_sha256 disagree",
    )
    _require(
        document.get("source_power_on_readonly_sha256") == power_file_sha256,
        "anchor source power-on evidence SHA mismatch",
    )
    canonical_anchor_bytes = (
        json.dumps(anchor, ensure_ascii=False, indent=2) + "\n"
    ).encode("utf-8")
    _require(
        hashlib.sha256(canonical_anchor_bytes).hexdigest() == anchor_sha,
        "runtime_anchor_sha256 does not match the embedded runtime anchor",
    )
    _require(
        document.get("anchor_field_count") == len(anchor),
        "anchor_field_count does not match the runtime anchor",
    )
    checks = document.get("checks")
    _require(isinstance(checks, Mapping), "anchor checks object required")
    for field in ("readonly_sample_match", "session_match", "state_instance_match", "model_hash_match"):
        _require(checks.get(field) is True, f"anchor check failed: {field}")
    for field in ("motor_internal_zero_modified", "flash_written", "eeprom_written", "mujoco_model_modified"):
        _require(checks.get(field) is False, f"anchor prohibited modification: {field}")
    return status, {"anchor_sha256": anchor_sha, "anchor": anchor}


def _validate_gravity_readonly(document: Mapping[str, Any], power: Mapping[str, Any], anchor: Mapping[str, Any]) -> tuple[str, dict[str, Any]]:
    status = _document_status(document, "gravity_readonly_validation.json")
    if status not in {"PASS", "FULL_PASS"}:
        return status, {}
    _require(
        document.get("schema") == GRAVITY_READONLY_SCHEMA,
        "gravity read-only schema mismatch",
    )
    _require(
        document.get("model_sha256") == PRODUCTION_MODEL_SHA256,
        "gravity read-only model SHA mismatch",
    )
    _require(
        document.get("gravity_config_sha256") == GRAVITY_CONFIG_SHA256,
        "gravity read-only config SHA mismatch",
    )
    _require(document.get("session_id") == power.get("session_id"), "gravity read-only session mismatch")
    _require(document.get("state_instance_id") == power.get("state_instance_id"), "gravity read-only state instance mismatch")
    _require(document.get("anchor_sha256") == anchor.get("anchor_sha256"), "gravity read-only anchor hash mismatch")
    _require(document.get("hardware_tff_enabled") is False, "gravity read-only must keep hardware Tff disabled")
    _require(document.get("tff_transmitted") is False, "gravity read-only transmitted Tff")
    duration = _number(document.get("duration_s"), "gravity_readonly.duration_s")
    _require(duration >= 10.0, "gravity read-only duration must be at least 10 seconds")
    declared_count = _integer(
        document.get("valid_sample_count"),
        "gravity_readonly.valid_sample_count",
        minimum=READ_ONLY_MINIMUM_SAMPLES,
    )
    gravity_source = document.get("gravity_source_instance_id")
    _require(
        isinstance(gravity_source, str) and gravity_source,
        "gravity read-only source instance is required",
    )
    checks = document.get("checks")
    _require(isinstance(checks, Mapping), "gravity read-only checks object required")
    for field in (
        "finite", "continuous", "direction_reasonable", "direction_independent_of_motion",
        "j2_split_50_50", "j2a_sign_correct", "j2b_sign_correct",
    ):
        _require(checks.get(field) is True, f"gravity read-only check failed: {field}")
    samples = document.get("samples")
    _require(
        isinstance(samples, list) and len(samples) == declared_count,
        "gravity read-only sample count must match retained samples",
    )
    gravity_sequences: list[int] = []
    gravity_stamps: list[int] = []
    hardware_sequences: list[int] = []
    hardware_stamps: list[int] = []
    maximum_velocity = 0.0
    for index, sample in enumerate(samples):
        field = f"gravity_readonly.samples[{index}]"
        _require(isinstance(sample, Mapping), f"{field}: object required")
        gravity_sequences.append(
            _integer(
                sample.get("gravity_status_sequence"),
                f"{field}.gravity_status_sequence",
                minimum=1,
            )
        )
        gravity_stamps.append(
            _integer(
                sample.get("gravity_status_source_monotonic_ns"),
                f"{field}.gravity_status_source_monotonic_ns",
                minimum=1,
            )
        )
        hardware_sequences.append(
            _integer(
                sample.get("hardware_state_sequence"),
                f"{field}.hardware_state_sequence",
                minimum=1,
            )
        )
        hardware_stamps.append(
            _integer(
                sample.get("hardware_state_source_monotonic_ns"),
                f"{field}.hardware_state_source_monotonic_ns",
                minimum=1,
            )
        )
        _require(
            hardware_stamps[-1] <= gravity_stamps[-1]
            and gravity_stamps[-1] - hardware_stamps[-1]
            <= READ_ONLY_MAXIMUM_SAMPLE_GAP_NS,
            f"{field}: hardware pose is stale or from the future",
        )
        _finite_vector(sample.get("q_actual_rad"), 6, f"{field}.q_actual_rad")
        _finite_vector(sample.get("model_q_rad"), 6, f"{field}.model_q_rad")
        gravity = _finite_vector(sample.get("gravity_joint_nm"), 6, f"{field}.gravity_joint_nm")
        predicted = _exact_keys(sample.get("predicted_rotor_nm"), GO_MOTOR_NAMES, f"{field}.predicted_rotor_nm")
        expected = {
            "J1": gravity[0] / GO_GEAR_RATIO,
            "J2A": -gravity[1] / (2.0 * GO_GEAR_RATIO),
            "J2B": +gravity[1] / (2.0 * GO_GEAR_RATIO),
            "J3": gravity[2] / GO_GEAR_RATIO,
            "J4": -gravity[3] / GO_GEAR_RATIO,
            "J5": gravity[4] / GO_GEAR_RATIO,
        }
        for name in GO_MOTOR_NAMES:
            actual = _number(predicted[name], f"{field}.predicted_rotor_nm.{name}")
            _require(math.isclose(actual, expected[name], rel_tol=1.0e-8, abs_tol=1.0e-8), f"{field}: {name} rotor prediction violates mapping")
        _require(
            _number(sample.get("gravity_scale"), f"{field}.gravity_scale") == 0.0
            and _number(
                sample.get("gravity_scale_target"),
                f"{field}.gravity_scale_target",
            )
            == 0.0,
            f"{field}: gravity scale must stay zero",
        )
        feedforward = _finite_vector(
            sample.get("feedforward_nm"), 6, f"{field}.feedforward_nm"
        )
        _require(
            all(abs(value) <= 1.0e-12 for value in feedforward),
            f"{field}: nonzero feedforward observed in read-only phase",
        )
        sample_velocity = abs(
            _number(
                sample.get("max_abs_velocity_rad_s"),
                f"{field}.max_abs_velocity_rad_s",
            )
        )
        maximum_velocity = max(maximum_velocity, sample_velocity)
        _require(
            sample_velocity <= READ_ONLY_MAXIMUM_STATIONARY_VELOCITY_RAD_S,
            f"{field}: mechanism is not stationary",
        )
    for values, field in (
        (gravity_sequences, "gravity_readonly.gravity_status_sequence"),
        (gravity_stamps, "gravity_readonly.gravity_status_monotonic_ns"),
        (hardware_sequences, "gravity_readonly.hardware_state_sequence"),
        (hardware_stamps, "gravity_readonly.hardware_state_monotonic_ns"),
    ):
        _strictly_increasing(values, field)
    for values, field in (
        (gravity_stamps, "gravity status"),
        (hardware_stamps, "hardware state"),
    ):
        _require(
            all(
                current - previous <= READ_ONLY_MAXIMUM_SAMPLE_GAP_NS
                for previous, current in zip(values, values[1:])
            ),
            f"gravity read-only {field} gap exceeds 250 ms",
        )
    observed_duration_ns = gravity_stamps[-1] - gravity_stamps[0]
    _require(
        observed_duration_ns >= READ_ONLY_MINIMUM_DURATION_NS,
        "gravity read-only retained samples cover less than 10 seconds",
    )
    _require(
        math.isclose(duration, observed_duration_ns * 1.0e-9, abs_tol=1.0e-6),
        "gravity read-only duration_s disagrees with retained samples",
    )
    continuity = document.get("continuity")
    _require(isinstance(continuity, Mapping), "gravity read-only continuity required")
    _require(
        _number(
            continuity.get("maximum_gravity_status_gap_ms"),
            "gravity_readonly.continuity.maximum_gravity_status_gap_ms",
        )
        <= 250.0,
        "gravity read-only reported gravity status gap exceeds 250 ms",
    )
    _require(
        _number(
            continuity.get("maximum_hardware_state_gap_ms"),
            "gravity_readonly.continuity.maximum_hardware_state_gap_ms",
        )
        <= 250.0,
        "gravity read-only reported hardware state gap exceeds 250 ms",
    )
    _require(
        math.isclose(
            _number(
                continuity.get("maximum_abs_velocity_rad_s"),
                "gravity_readonly.continuity.maximum_abs_velocity_rad_s",
            ),
            maximum_velocity,
            abs_tol=1.0e-9,
        ),
        "gravity read-only velocity summary disagrees with samples",
    )
    probes = document.get("direction_probes")
    _require(
        isinstance(probes, list) and len(probes) == len(JOINT_NAMES),
        "gravity read-only requires six ordered direction probes",
    )
    center_gravity = tuple(
        float(value) for value in samples[0]["gravity_joint_nm"]
    )
    for index, (joint, probe) in enumerate(zip(JOINT_NAMES, probes)):
        field = f"gravity_readonly.direction_probes[{index}]"
        _require(isinstance(probe, Mapping), f"{field}: object required")
        _require(probe.get("joint") == joint, f"{field}: joint order mismatch")
        _require(
            math.isclose(
                _number(probe.get("delta_q_rad"), f"{field}.delta_q_rad"),
                math.radians(0.25),
                abs_tol=1.0e-12,
            ),
            f"{field}: probe displacement mismatch",
        )
        minus = _finite_vector(
            probe.get("minus_gravity_joint_nm"), 6, f"{field}.minus"
        )
        center = _finite_vector(
            probe.get("center_gravity_joint_nm"), 6, f"{field}.center"
        )
        plus = _finite_vector(
            probe.get("plus_gravity_joint_nm"), 6, f"{field}.plus"
        )
        _require(
            all(
                math.isclose(actual, expected, abs_tol=1.0e-8)
                for actual, expected in zip(center, center_gravity)
            ),
            f"{field}: center gravity disagrees with captured model pose",
        )
        second_difference = max(
            abs(plus_value - 2.0 * center_value + minus_value)
            for minus_value, center_value, plus_value in zip(minus, center, plus)
        )
        _require(
            second_difference <= 0.05
            and math.isclose(
                second_difference,
                _number(
                    probe.get("maximum_second_difference_nm"),
                    f"{field}.maximum_second_difference_nm",
                ),
                abs_tol=1.0e-9,
            ),
            f"{field}: discontinuous or inconsistent direction probe",
        )
        for name in ("qvel_rad_s", "qacc_rad_s2"):
            vector = _finite_vector(probe.get(name), 6, f"{field}.{name}")
            _require(
                all(abs(value) <= 1.0e-12 for value in vector),
                f"{field}: {name} must stay zero",
            )
    safety = document.get("safety")
    _require(isinstance(safety, Mapping), "gravity read-only safety object required")
    _require(
        _integer(
            safety.get("active_command_count_observed"),
            "gravity_readonly.safety.active_command_count_observed",
            minimum=0,
        )
        == 0,
        "gravity read-only observed an active command",
    )
    for field in (
        "motor_internal_zero_modified",
        "flash_written",
        "eeprom_written",
        "mujoco_model_modified",
    ):
        _require(safety.get(field) is False, f"gravity read-only prohibited change: {field}")
    return status, {}


GRAVITY_SCALE_FIELDS = (
    "timestamp_utc", "monotonic_ns", "stage", "phase",
    "observer_receipt_monotonic_ns",
    "gravity_scale_target", "gravity_scale_applied", "phase_elapsed_s",
    "position_error_deg", "tau_feedback_rotor_nm",
    "pd_contribution_rotor_nm", "gravity_ff_contribution_rotor_nm",
    "total_command_rotor_nm", "temperature_c",
    "temperature_slope_c_per_min", "j2_e_sync_deg",
    "saturation_observed", "merror", "communication_ok",
    "operator_stop_available", "status", "reason",
    "envelope_id", "envelope_sha256", "session_id", "state_instance_id",
    "anchor_sha256", "gravity_source_instance_id",
    "gravity_status_sequence", "gravity_status_source_monotonic_ns",
    "hardware_state_sequence", "hardware_state_source_monotonic_ns",
    "node_feedforward_nm_json", "worker_feedforward_rotor_nm_json",
    "matched_node_feedforward_nm_json",
    "matched_expected_worker_feedforward_rotor_nm_json",
    "worker_echo_source_monotonic_ns_json",
    "worker_echo_match_gravity_status_sequence",
    "worker_echo_match_gravity_status_source_monotonic_ns",
    "worker_echo_lag_ms",
    "gravity_status_sha256", "hardware_state_sha256",
)


def _csv_all_structured_nonpass(rows: Sequence[Mapping[str, str]], filename: str) -> str | None:
    statuses = [row.get("status", "").strip().upper() for row in rows]
    if statuses and all(any(value.startswith(prefix) for prefix in NONPASS_PREFIXES) for value in statuses):
        for index, row in enumerate(rows, start=2):
            _require(row.get("reason", "").strip(), f"{filename}:{index}: non-PASS row requires reason")
        return statuses[0]
    return None


def _validate_gravity_scale(
    path: Path,
    power: Mapping[str, Any],
    anchor: Mapping[str, Any],
) -> tuple[str, dict[str, Any]]:
    rows = _read_csv(path, GRAVITY_SCALE_FIELDS)
    nonpass = _csv_all_structured_nonpass(rows, path.name)
    if nonpass:
        return nonpass, {}
    stamps: list[int] = []
    gravity_sequences: list[int] = []
    gravity_stamps: list[int] = []
    hardware_sequences: list[int] = []
    hardware_stamps: list[int] = []
    by_level_phase: dict[tuple[float, str], list[dict[str, str]]] = defaultdict(list)
    first_level_order: list[float] = []
    binding: tuple[str, str, str, str, str, str] | None = None
    node_publication_by_identity: dict[
        tuple[int, int], tuple[float, ...]
    ] = {}
    confirmation_requirements: list[tuple[int, float | None]] = []
    for index, row in enumerate(rows, start=2):
        location = f"{path.name}:{index}"
        _require(row["status"].strip().upper() == "PASS", f"{location}: completed ladder rows must be PASS")
        _timestamp(row["timestamp_utc"], f"{location}.timestamp_utc")
        stamp = _row_integer(row, "monotonic_ns", location)
        _require(stamp > 0, f"{location}.monotonic_ns must be positive")
        observer_receipt_ns = _row_integer(
            row, "observer_receipt_monotonic_ns", location
        )
        _require(
            0 <= observer_receipt_ns - stamp
            <= READ_ONLY_MAXIMUM_SAMPLE_GAP_NS,
            f"{location}: observer receipt is before source or stale",
        )
        stamps.append(stamp)
        row_binding = (
            row["envelope_id"].strip(),
            row["envelope_sha256"].strip(),
            row["session_id"].strip(),
            row["state_instance_id"].strip(),
            row["anchor_sha256"].strip(),
            row["gravity_source_instance_id"].strip(),
        )
        _require(row_binding[0].startswith("v15-31b-empirical-"), f"{location}: envelope id invalid")
        _require(SHA256_RE.fullmatch(row_binding[1]) is not None, f"{location}: envelope SHA invalid")
        _require(row_binding[2] == power.get("session_id"), f"{location}: session mismatch")
        _require(row_binding[3] == power.get("state_instance_id"), f"{location}: state instance mismatch")
        _require(row_binding[4] == anchor.get("anchor_sha256"), f"{location}: anchor SHA mismatch")
        _require(re.fullmatch(r"[0-9a-f]{32}", row_binding[5]) is not None, f"{location}: gravity source invalid")
        if binding is None:
            binding = row_binding
        _require(row_binding == binding, f"{location}: ladder binding/source changed")
        gravity_sequences.append(_row_integer(row, "gravity_status_sequence", location))
        gravity_stamps.append(_row_integer(row, "gravity_status_source_monotonic_ns", location))
        hardware_sequences.append(_row_integer(row, "hardware_state_sequence", location))
        hardware_stamps.append(_row_integer(row, "hardware_state_source_monotonic_ns", location))
        _require(
            0 <= stamp - gravity_stamps[-1] <= READ_ONLY_MAXIMUM_SAMPLE_GAP_NS,
            f"{location}: gravity status is stale or from the future",
        )
        _require(
            0 <= stamp - hardware_stamps[-1] <= READ_ONLY_MAXIMUM_SAMPLE_GAP_NS,
            f"{location}: hardware state is stale or from the future",
        )
        _require(
            SHA256_RE.fullmatch(row["gravity_status_sha256"].strip()) is not None
            and SHA256_RE.fullmatch(row["hardware_state_sha256"].strip()) is not None,
            f"{location}: source document SHA invalid",
        )
        try:
            node_feedforward = _finite_vector(
                _parse_json_text(
                    row["node_feedforward_nm_json"],
                    f"{location}.node_feedforward_nm_json",
                ),
                6,
                f"{location}.node_feedforward_nm_json",
            )
            worker_feedforward = _finite_vector(
                _parse_json_text(
                    row["worker_feedforward_rotor_nm_json"],
                    f"{location}.worker_feedforward_rotor_nm_json",
                ),
                6,
                f"{location}.worker_feedforward_rotor_nm_json",
            )
            matched_node_feedforward = _finite_vector(
                _parse_json_text(
                    row["matched_node_feedforward_nm_json"],
                    f"{location}.matched_node_feedforward_nm_json",
                ),
                6,
                f"{location}.matched_node_feedforward_nm_json",
            )
            matched_expected = _finite_vector(
                _parse_json_text(
                    row[
                        "matched_expected_worker_feedforward_rotor_nm_json"
                    ],
                    f"{location}.matched_expected_worker_feedforward_rotor_nm_json",
                ),
                6,
                f"{location}.matched_expected_worker_feedforward_rotor_nm_json",
            )
            worker_echo_sources = _parse_json_text(
                row["worker_echo_source_monotonic_ns_json"],
                f"{location}.worker_echo_source_monotonic_ns_json",
            )
        except (json.JSONDecodeError, TypeError) as exc:
            raise EvidenceValidationError(
                f"{location}: worker feedforward echo JSON invalid"
            ) from exc
        _require(
            isinstance(worker_echo_sources, list)
            and len(worker_echo_sources) == 6
            and all(type(value) is int and value > 0 for value in worker_echo_sources),
            f"{location}: worker echo source timestamps invalid",
        )
        _require(
            abs(node_feedforward[5]) <= 1.0e-12,
            f"{location}: node J6 feedforward must be zero",
        )
        _require(
            abs(matched_node_feedforward[5]) <= 1.0e-12,
            f"{location}: matched node J6 feedforward must be zero",
        )
        recomputed_matched_expected = (
            matched_node_feedforward[0],
            -matched_node_feedforward[1],
            matched_node_feedforward[1],
            matched_node_feedforward[2],
            -matched_node_feedforward[3],
            matched_node_feedforward[4],
        )
        _require(
            all(
                abs(persisted - recomputed) <= 1.0e-9
                and abs(actual - recomputed) <= 1.0e-9
                for persisted, actual, recomputed in zip(
                    matched_expected,
                    worker_feedforward,
                    recomputed_matched_expected,
                )
            ),
            f"{location}: worker echo is not derived from matched node publication",
        )
        matched_sequence = _row_integer(
            row, "worker_echo_match_gravity_status_sequence", location
        )
        _require(
            0 < matched_sequence <= gravity_sequences[-1],
            f"{location}: worker echo matched sequence invalid",
        )
        matched_source_ns = _row_integer(
            row,
            "worker_echo_match_gravity_status_source_monotonic_ns",
            location,
        )
        _require(
            0 < matched_source_ns
            <= _row_integer(
                row, "gravity_status_source_monotonic_ns", location
            )
            and all(
                0 <= source_ns - matched_source_ns <= 300_000_000
                and source_ns <= hardware_stamps[-1]
                for source_ns in worker_echo_sources
            ),
            f"{location}: worker echo source timestamps are not causal within 300 ms",
        )
        echo_lag_ms = _row_number(row, "worker_echo_lag_ms", location)
        _require(
            0.0 <= echo_lag_ms <= 300.0,
            f"{location}: worker echo propagation exceeds 300 ms",
        )
        _require(
            abs(
                echo_lag_ms
                - (max(worker_echo_sources) - matched_source_ns) / 1.0e6
            )
            <= 1.0e-6,
            f"{location}: worker echo lag is not source-time derived",
        )
        matched_publication = node_publication_by_identity.get(
            (matched_sequence, matched_source_ns)
        )
        first_zero_bootstrap = bool(
            index == 2
            and row["stage"].strip() == "0%"
            and abs(_row_number(row, "gravity_scale_target", location))
            <= 1.0e-12
            and abs(_row_number(row, "gravity_scale_applied", location))
            <= 1.0e-12
            and matched_sequence < gravity_sequences[-1]
            and matched_source_ns < gravity_stamps[-1]
            and all(
                abs(value) <= 1.0e-12
                for vector in (
                    node_feedforward,
                    matched_node_feedforward,
                    matched_expected,
                    worker_feedforward,
                )
                for value in vector
            )
        )
        _require(
            first_zero_bootstrap
            or (
                matched_publication is not None
                and all(
                    abs(actual - persisted) <= 1.0e-12
                    for actual, persisted in zip(
                        matched_node_feedforward,
                        matched_publication,
                    )
                )
            ),
            f"{location}: matched node publication is not a unique prior CSV row",
        )
        publication_identity = (
            gravity_sequences[-1], gravity_stamps[-1]
        )
        _require(
            publication_identity not in node_publication_by_identity,
            f"{location}: duplicate gravity publication identity",
        )
        node_publication_by_identity[publication_identity] = node_feedforward
        target = _row_number(row, "gravity_scale_target", location)
        level = min(GRAVITY_LEVELS, key=lambda item: abs(item - target))
        _require(abs(target - level) <= 1.0e-12, f"{location}: invalid gravity scale target")
        confirmation_requirements.append(
            (observer_receipt_ns, None if level == 0.0 else level)
        )
        applied = _row_number(row, "gravity_scale_applied", location)
        _require(-1.0e-12 <= applied <= 1.0 + 1.0e-12, f"{location}: applied scale outside [0,1]")
        phase = row["phase"].strip().upper()
        _require(
            phase in {"RAMP", "HOLD", "AWAIT_INTERSTAGE_CONFIRMATION"},
            f"{location}: gravity phase invalid",
        )
        _row_number(row, "phase_elapsed_s", location)
        for field in (
            "position_error_deg", "tau_feedback_rotor_nm", "pd_contribution_rotor_nm",
            "gravity_ff_contribution_rotor_nm", "total_command_rotor_nm",
            "temperature_slope_c_per_min",
        ):
            _row_number(row, field, location)
        temperature = _row_number(row, "temperature_c", location)
        _require(temperature < 60.0, f"{location}: PASS row reaches hard thermal stop")
        sync = abs(_row_number(row, "j2_e_sync_deg", location))
        _require(sync <= 0.25, f"{location}: J2 sync warning threshold exceeded")
        _row_bool(row, "saturation_observed", location)
        _require(_row_integer(row, "merror", location) == 0, f"{location}: nonzero merror")
        _require(_row_bool(row, "communication_ok", location), f"{location}: communication failure")
        _require(_row_bool(row, "operator_stop_available", location), f"{location}: operator stop unavailable")
        key = (level, phase)
        by_level_phase[key].append(row)
        _require(
            row["stage"].strip() == f"{int(level * 100)}%",
            f"{location}: stage label mismatch",
        )
        if level not in first_level_order:
            first_level_order.append(level)
    _strictly_increasing(stamps, f"{path.name}.monotonic_ns")
    for values, field in (
        (gravity_sequences, f"{path.name}.gravity_status_sequence"),
        (gravity_stamps, f"{path.name}.gravity_status_source_monotonic_ns"),
        (hardware_sequences, f"{path.name}.hardware_state_sequence"),
        (hardware_stamps, f"{path.name}.hardware_state_source_monotonic_ns"),
    ):
        _strictly_increasing(values, field)
    _require(tuple(first_level_order) == GRAVITY_LEVELS, "gravity ladder must execute 0,25,50,75,100 percent in order")
    previous_stage_final_stamp: int | None = None
    previous_level = 0.0
    for level in GRAVITY_LEVELS:
        holds = by_level_phase.get((level, "HOLD"), [])
        _require(holds, f"gravity scale {level}: HOLD phase missing")
        stage_rows = [
            row
            for row in rows
            if abs(float(row["gravity_scale_target"]) - level) <= 1.0e-12
        ]
        stage_phases = [row["phase"].strip().upper() for row in stage_rows]
        expected_phase_order = (
            ["HOLD"] if level == 0.0 else ["RAMP", "HOLD"]
        )
        if level < GRAVITY_LEVELS[-1]:
            expected_phase_order.append("AWAIT_INTERSTAGE_CONFIRMATION")
        collapsed_phases = [
            phase
            for index, phase in enumerate(stage_phases)
            if index == 0 or phase != stage_phases[index - 1]
        ]
        _require(
            collapsed_phases == expected_phase_order,
            f"gravity scale {level}: phase order mismatch",
        )
        stage_stamps = [int(row["monotonic_ns"]) for row in stage_rows]
        _require(
            all(
                0 < current - previous <= 100_000_000
                for previous, current in zip(stage_stamps, stage_stamps[1:])
            ),
            f"gravity scale {level}: sample gap exceeds 100 ms",
        )
        if previous_stage_final_stamp is not None:
            _require(
                0 < stage_stamps[0] - previous_stage_final_stamp
                <= 30_000_000_000,
                f"gravity scale {level}: next-stage confirmation gap exceeds 30 s",
            )
        hold_elapsed = [_row_number(row, "phase_elapsed_s", f"gravity scale {level} HOLD") for row in holds]
        coverage = max(hold_elapsed) - min(hold_elapsed)
        _require(5.0 <= coverage <= 10.5, f"gravity scale {level}: HOLD coverage must be 5-10 seconds")
        hold_start_ns = int(holds[0]["monotonic_ns"])
        for row in holds:
            elapsed_value = float(row["phase_elapsed_s"])
            _require(
                math.isclose(
                    elapsed_value,
                    (int(row["monotonic_ns"]) - hold_start_ns) * 1.0e-9,
                    abs_tol=0.010,
                ),
                f"gravity scale {level}: HOLD elapsed time disagrees with monotonic clock",
            )
            _require(
                abs(float(row["gravity_scale_applied"]) - level) <= 1.0e-6,
                f"gravity scale {level}: HOLD applied scale changed",
            )
        final_applied = _row_number(holds[-1], "gravity_scale_applied", f"gravity scale {level} HOLD final")
        _require(abs(final_applied - level) <= 1.0e-6, f"gravity scale {level}: applied scale did not reach target")
        waits = by_level_phase.get(
            (level, "AWAIT_INTERSTAGE_CONFIRMATION"), []
        )
        if level < GRAVITY_LEVELS[-1]:
            _require(
                waits,
                f"gravity scale {level}: interstage confirmation wait missing",
            )
            wait_start_ns = int(waits[0]["monotonic_ns"])
            wait_end_ns = int(waits[-1]["monotonic_ns"])
            _require(
                0 <= wait_end_ns - wait_start_ns <= 30_000_000_000,
                f"gravity scale {level}: interstage confirmation wait exceeds 30 s",
            )
            for row in waits:
                row_stamp = int(row["monotonic_ns"])
                elapsed_value = float(row["phase_elapsed_s"])
                _require(
                    math.isclose(
                        elapsed_value,
                        (row_stamp - wait_start_ns) * 1.0e-9,
                        abs_tol=0.010,
                    ),
                    f"gravity scale {level}: confirmation wait elapsed disagrees with monotonic clock",
                )
                _require(
                    abs(float(row["gravity_scale_applied"]) - level)
                    <= 1.0e-6,
                    f"gravity scale {level}: applied scale changed while awaiting confirmation",
                )
        else:
            _require(
                not waits,
                "gravity scale 1.0: unexpected interstage confirmation wait",
            )
        if level > 0.0:
            ramps = by_level_phase.get((level, "RAMP"), [])
            _require(ramps, f"gravity scale {level}: RAMP phase missing")
            ramp_start_ns = int(ramps[0]["monotonic_ns"])
            transition_duration_s = (hold_start_ns - ramp_start_ns) * 1.0e-9
            _require(
                1.9 <= transition_duration_s <= 2.10,
                f"gravity scale {level}: transition must cover 2 seconds within one 100 ms sample",
            )
            _require(
                abs(float(ramps[0]["gravity_scale_applied"]) - previous_level)
                <= 0.01,
                f"gravity scale {level}: ramp did not begin at prior rung",
            )
            previous_applied = previous_level
            for row in ramps:
                row_stamp = int(row["monotonic_ns"])
                elapsed_value = float(row["phase_elapsed_s"])
                _require(
                    math.isclose(
                        elapsed_value,
                        (row_stamp - ramp_start_ns) * 1.0e-9,
                        abs_tol=0.010,
                    ),
                    f"gravity scale {level}: RAMP elapsed time disagrees with monotonic clock",
                )
                applied_value = float(row["gravity_scale_applied"])
                _require(
                    previous_applied - 1.0e-9 <= applied_value < level - 1.0e-6,
                    f"gravity scale {level}: ramp is non-monotonic or reached target early",
                )
                expected_applied = previous_level + (
                    level - previous_level
                ) * min(1.0, (row_stamp - ramp_start_ns) / 2_000_000_000)
                _require(
                    abs(applied_value - expected_applied) <= 0.02,
                    f"gravity scale {level}: applied ramp is not a smooth 2 s transition",
                )
                previous_applied = applied_value
        previous_stage_final_stamp = stage_stamps[-1]
        previous_level = level
    assert binding is not None
    return "PASS", {
        "maximum_j2_sync_deg": max(abs(float(row["j2_e_sync_deg"])) for row in rows),
        "envelope_id": binding[0],
        "envelope_sha256": binding[1],
        "session_id": binding[2],
        "state_instance_id": binding[3],
        "anchor_sha256": binding[4],
        "gravity_source_instance_id": binding[5],
        "first_hardware_state_sequence": hardware_sequences[0],
        "first_hardware_state_source_ns": hardware_stamps[0],
        "last_hardware_state_sequence": hardware_sequences[-1],
        "last_hardware_state_source_ns": hardware_stamps[-1],
        "confirmation_requirements": confirmation_requirements,
    }


MOTION_TRACE_SAMPLE_FIELDS = (
    "hardware_state_sequence",
    "hardware_state_source_monotonic_ns",
    "receipt_monotonic_ns",
    "hardware_state_sha256",
    "joint",
    "target_rad",
    "actual_rad",
    "error_deg",
    "velocity_deg_s",
    "position_rad",
    "controller_mode_by_motor",
    "moving_joint_mask",
    "trajectory_sha256",
)


def _validate_motion_trace(
    document: Any,
    persisted_sha: Any,
    *,
    field: str,
    schema: str,
    joint: str,
    target_rad: float,
    trajectory_sha256: str,
    require_dwell: bool,
    allow_empty: bool = False,
) -> list[Mapping[str, Any]]:
    _require(isinstance(document, Mapping), f"{field}: object required")
    _require(
        set(document) == {"schema", "samples"} and document.get("schema") == schema,
        f"{field}: trace schema/keys invalid",
    )
    _require_document_sha(document, persisted_sha, f"{field}.sha256")
    samples = document.get("samples")
    _require(isinstance(samples, list), f"{field}.samples: list required")
    if allow_empty and not samples:
        return []
    _require(samples, f"{field}.samples: trace must not be empty")
    sequences: list[int] = []
    sources: list[int] = []
    receipts: list[int] = []
    joint_index = JOINT_NAMES.index(joint)
    moving_motors = set(MOTOR_BY_JOINT[joint])
    expected_mask = [name == joint for name in JOINT_NAMES]
    center_observation = trajectory_sha256 == "CENTER_OBSERVATION"
    if center_observation:
        expected_mask = [False] * 6
    for index, sample in enumerate(samples):
        location = f"{field}.samples[{index}]"
        _require(
            isinstance(sample, Mapping) and set(sample) == set(MOTION_TRACE_SAMPLE_FIELDS),
            f"{location}: exact motion-trace sample keys required",
        )
        sequence = _integer(
            sample.get("hardware_state_sequence"),
            f"{location}.hardware_state_sequence",
            minimum=1,
        )
        source_ns = _integer(
            sample.get("hardware_state_source_monotonic_ns"),
            f"{location}.hardware_state_source_monotonic_ns",
            minimum=1,
        )
        receipt_ns = _integer(
            sample.get("receipt_monotonic_ns"),
            f"{location}.receipt_monotonic_ns",
            minimum=1,
        )
        _require(
            0 <= receipt_ns - source_ns <= MAXIMUM_RECEIPT_AGE_NS,
            f"{location}: hardware state is stale or received before its source time",
        )
        sequences.append(sequence)
        sources.append(source_ns)
        receipts.append(receipt_ns)
        _require(
            isinstance(sample.get("hardware_state_sha256"), str)
            and SHA256_RE.fullmatch(str(sample["hardware_state_sha256"])) is not None,
            f"{location}.hardware_state_sha256 invalid",
        )
        _require(sample.get("joint") == joint, f"{location}: joint mismatch")
        sample_target = _number(sample.get("target_rad"), f"{location}.target_rad")
        actual_rad = _number(sample.get("actual_rad"), f"{location}.actual_rad")
        error_deg = _number(sample.get("error_deg"), f"{location}.error_deg")
        velocity_deg_s = _number(
            sample.get("velocity_deg_s"), f"{location}.velocity_deg_s"
        )
        position = _finite_vector(sample.get("position_rad"), 6, f"{location}.position_rad")
        _require(
            math.isclose(sample_target, target_rad, abs_tol=1.0e-9)
            and math.isclose(actual_rad, position[joint_index], abs_tol=1.0e-9)
            and math.isclose(
                error_deg,
                math.degrees(target_rad - actual_rad),
                abs_tol=1.0e-6,
            ),
            f"{location}: target/actual/error is not self-consistent",
        )
        modes = _exact_keys(
            sample.get("controller_mode_by_motor"),
            MOTOR_NAMES,
            f"{location}.controller_mode_by_motor",
        )
        for motor in MOTOR_NAMES:
            expected_mode = (
                "hold"
                if center_observation or motor not in moving_motors
                else "position"
            )
            _require(
                str(modes[motor]).strip().lower() == expected_mode,
                f"{location}.{motor}: controller mode is not continuous HOLD/POSITION",
            )
        _require(
            sample.get("moving_joint_mask") == expected_mask,
            f"{location}: moving_joint_mask mismatch",
        )
        _require(
            sample.get("trajectory_sha256") == trajectory_sha256,
            f"{location}: trajectory SHA mismatch",
        )
        if require_dwell:
            _require(
                abs(error_deg) <= 0.5 + 1.0e-9
                and abs(velocity_deg_s) <= 0.25 + 1.0e-9,
                f"{location}: endpoint dwell is outside error/velocity limits",
            )
    _strictly_increasing(sequences, f"{field}.hardware_state_sequence")
    _strictly_increasing(sources, f"{field}.hardware_state_source_monotonic_ns")
    _strictly_increasing(receipts, f"{field}.receipt_monotonic_ns")
    maximum_gap_ns = (
        TRACE_MAXIMUM_SAMPLE_GAP_NS
        if require_dwell else THERMAL_MAXIMUM_SAMPLE_GAP_NS
    )
    _require(
        all(
            current - previous <= maximum_gap_ns
            for previous, current in zip(sources, sources[1:])
        ),
        f"{field}: source sample gap exceeds the allowed stream bound",
    )
    if require_dwell:
        _require(
            len(samples) >= TRACE_MINIMUM_DWELL_SAMPLES
            and sources[-1] - sources[0] >= TRACE_MINIMUM_DWELL_NS,
            f"{field}: endpoint dwell trace is shorter than 0.5 s/six samples",
        )
    return samples


def _validate_final_dwell_trace(
    samples: Any,
    persisted_sha: Any,
    *,
    field: str,
    target_deg: Sequence[float],
) -> list[Mapping[str, Any]]:
    document = {
        "schema": "V15.31B-final-dwell-trace-v1",
        "samples": samples,
    }
    _require_document_sha(document, persisted_sha, f"{field}.sha256")
    _require(isinstance(samples, list), f"{field}: list required")
    _require(
        len(samples) >= TRACE_MINIMUM_DWELL_SAMPLES,
        f"{field}: at least six samples required",
    )
    expected_keys = {
        "hardware_state_sequence",
        "hardware_state_source_monotonic_ns",
        "receipt_monotonic_ns",
        "hardware_state_sha256",
        "position_rad",
        "velocity_rad_s",
        "error_deg_by_joint",
        "controller_mode_by_motor",
    }
    sequences: list[int] = []
    sources: list[int] = []
    receipts: list[int] = []
    for index, sample in enumerate(samples):
        location = f"{field}[{index}]"
        _require(
            isinstance(sample, Mapping) and set(sample) == expected_keys,
            f"{location}: exact final-dwell sample keys required",
        )
        sequences.append(
            _integer(sample.get("hardware_state_sequence"), f"{location}.sequence", minimum=1)
        )
        source_ns = _integer(
            sample.get("hardware_state_source_monotonic_ns"),
            f"{location}.source_monotonic_ns",
            minimum=1,
        )
        receipt_ns = _integer(
            sample.get("receipt_monotonic_ns"),
            f"{location}.receipt_monotonic_ns",
            minimum=1,
        )
        _require(
            0 <= receipt_ns - source_ns <= MAXIMUM_RECEIPT_AGE_NS,
            f"{location}: stale/future hardware state",
        )
        sources.append(source_ns)
        receipts.append(receipt_ns)
        _require(
            isinstance(sample.get("hardware_state_sha256"), str)
            and SHA256_RE.fullmatch(str(sample["hardware_state_sha256"])) is not None,
            f"{location}.hardware_state_sha256 invalid",
        )
        position = _finite_vector(sample.get("position_rad"), 6, f"{location}.position_rad")
        velocity = _finite_vector(sample.get("velocity_rad_s"), 6, f"{location}.velocity_rad_s")
        errors = _exact_keys(
            sample.get("error_deg_by_joint"), JOINT_NAMES, f"{location}.error_deg_by_joint"
        )
        for joint_index, joint in enumerate(JOINT_NAMES):
            expected_error = abs(float(target_deg[joint_index]) - math.degrees(position[joint_index]))
            _require(
                math.isclose(
                    _number(errors[joint], f"{location}.error_deg_by_joint.{joint}"),
                    expected_error,
                    abs_tol=1.0e-6,
                )
                and expected_error <= 0.5 + 1.0e-9,
                f"{location}.{joint}: final dwell error invalid",
            )
        _require(
            max(abs(math.degrees(value)) for value in velocity) <= 0.25 + 1.0e-9,
            f"{location}: final dwell velocity exceeds 0.25 deg/s",
        )
        modes = _exact_keys(
            sample.get("controller_mode_by_motor"),
            MOTOR_NAMES,
            f"{location}.controller_mode_by_motor",
        )
        _require(
            all(str(modes[motor]).strip().lower() == "hold" for motor in MOTOR_NAMES),
            f"{location}: final dwell is not whole-arm HOLD",
        )
    for values, name in (
        (sequences, "sequence"),
        (sources, "source_monotonic_ns"),
        (receipts, "receipt_monotonic_ns"),
    ):
        _strictly_increasing(values, f"{field}.{name}")
    _require(
        sources[-1] - sources[0] >= TRACE_MINIMUM_DWELL_NS
        and all(
            current - previous <= TRACE_MAXIMUM_SAMPLE_GAP_NS
            for previous, current in zip(sources, sources[1:])
        ),
        f"{field}: final HOLD dwell is not continuous for 0.5 s",
    )
    return samples


POSITION_FIELDS = (
    "timestamp_utc", "monotonic_ns", "receipt_monotonic_ns",
    "test_id", "joint", "revision",
    "phase", "target_deg", "actual_deg", "error_deg",
    "endpoint_dwell_s", "j2_e_sync_deg", "temperature_c", "merror",
    "communication_ok", "controller_mode", "status", "reason",
    "envelope_id", "envelope_sha256", "session_id", "state_instance_id",
    "anchor_sha256", "trajectory_sha256",
    "trajectory_duration_ns", "trajectory_execute_at_monotonic_ns",
    "plan_token_id",
    "plan_recipe_sha256", "collision_proof_sha256",
    "hardware_state_sha256", "related_domain",
    "endpoint_dwell_trace_json", "endpoint_dwell_trace_sha256",
    "segment_execution_trace_json", "segment_execution_trace_sha256",
    "hardware_state_sequence", "hardware_state_source_monotonic_ns",
    "gui_command_sequence", "gui_command_source_instance_id",
    "gui_command_source_monotonic_ns",
    "actual_center_start_deg", "actual_displacement_from_center_deg",
    "minimum_required_actual_displacement_deg",
)


def _validate_position(
    path: Path, expected_binding: Mapping[str, Any]
) -> tuple[str, dict[str, Any]]:
    rows = _read_csv(path, POSITION_FIELDS)
    nonpass = _csv_all_structured_nonpass(rows, path.name)
    if nonpass:
        return nonpass, {}
    stamps: list[int] = []
    receipts: list[int] = []
    state_sequences: list[int] = []
    gui_sequences: list[int] = []
    gui_sources: list[int] = []
    grouped: dict[tuple[str, str], list[dict[str, str]]] = defaultdict(list)
    revisions_by_joint: dict[str, set[str]] = defaultdict(set)
    first_joint_order: list[str] = []
    trajectories: dict[str, tuple[int, int]] = {}
    trajectory_duration_total_ns = 0
    confirmation_requirements: list[tuple[int, float | None]] = []
    for index, row in enumerate(rows, start=2):
        location = f"{path.name}:{index}"
        _require(
            row["status"].strip().upper() == "PASS",
            f"{location}: completed position row must be PASS",
        )
        _timestamp(row["timestamp_utc"], f"{location}.timestamp_utc")
        stamp = _row_integer(row, "monotonic_ns", location)
        _require(stamp > 0, f"{location}.monotonic_ns must be positive")
        receipt = _row_integer(row, "receipt_monotonic_ns", location)
        _require(
            0 <= receipt - stamp <= MAXIMUM_RECEIPT_AGE_NS,
            f"{location}: source/receipt time is stale or noncausal",
        )
        stamps.append(stamp)
        receipts.append(receipt)
        state_sequence = _row_integer(row, "hardware_state_sequence", location)
        state_source = _row_integer(
            row, "hardware_state_source_monotonic_ns", location
        )
        _require(
            state_sequence > 0 and state_source == stamp,
            f"{location}: top-level time must be authoritative hardware source time",
        )
        state_sequences.append(state_sequence)
        _validate_binding_columns(row, expected_binding, location)
        _require(
            SHA256_RE.fullmatch(row["hardware_state_sha256"].strip()) is not None,
            f"{location}: hardware state SHA invalid",
        )
        joint = row["joint"].strip().upper()
        revision = row["revision"].strip().upper()
        _require(joint in JOINT_NAMES, f"{location}: invalid joint")
        _require(revision in REVISION_ORDER, f"{location}: invalid revision")
        _require(
            row["test_id"].strip() == f"{joint}-{revision}",
            f"{location}: test_id mismatch",
        )
        if joint not in first_joint_order:
            first_joint_order.append(joint)
        grouped[(joint, revision)].append(row)
        revisions_by_joint[joint].add(revision)
        phase = row["phase"].strip().upper()
        _require(phase in POSITION_PHASES, f"{location}: invalid phase")
        _require(
            row["related_domain"].strip() == DOMAIN_BY_JOINT[joint],
            f"{location}: related domain mismatch",
        )
        target_rad = math.radians(_row_number(row, "target_deg", location))
        trajectory_sha = row["trajectory_sha256"].strip()
        dwell_document = _parse_json_text(
            row["endpoint_dwell_trace_json"],
            f"{location}.endpoint_dwell_trace_json",
        )
        dwell_samples = _validate_motion_trace(
            dwell_document,
            row["endpoint_dwell_trace_sha256"].strip(),
            field=f"{location}.endpoint_dwell_trace",
            schema="V15.31B-endpoint-dwell-trace-v1",
            joint=joint,
            target_rad=target_rad,
            trajectory_sha256=trajectory_sha,
            require_dwell=True,
        )
        _require(
            int(dwell_samples[-1]["hardware_state_sequence"]) == state_sequence
            and int(dwell_samples[-1]["hardware_state_source_monotonic_ns"])
            == state_source
            and int(dwell_samples[-1]["receipt_monotonic_ns"]) == receipt
            and dwell_samples[-1]["hardware_state_sha256"]
            == row["hardware_state_sha256"].strip(),
            f"{location}: top-level endpoint does not bind the final dwell frame",
        )
        dwell_span_s = (
            int(dwell_samples[-1]["hardware_state_source_monotonic_ns"])
            - int(dwell_samples[0]["hardware_state_source_monotonic_ns"])
        ) / 1.0e9
        _require(
            math.isclose(
                _row_number(row, "endpoint_dwell_s", location),
                dwell_span_s,
                abs_tol=1.0e-9,
            ),
            f"{location}: endpoint_dwell_s disagrees with source trace",
        )
        execution_document = _parse_json_text(
            row["segment_execution_trace_json"],
            f"{location}.segment_execution_trace_json",
        )
        center_observation = phase == "CENTER_START"
        execution_samples = _validate_motion_trace(
            execution_document,
            row["segment_execution_trace_sha256"].strip(),
            field=f"{location}.segment_execution_trace",
            schema="V15.31B-segment-execution-trace-v1",
            joint=joint,
            target_rad=target_rad,
            trajectory_sha256=trajectory_sha,
            require_dwell=False,
            allow_empty=center_observation,
        )
        confirmation_requirements.extend(
            (int(sample["receipt_monotonic_ns"]), 1.0)
            for sample in [*execution_samples, *dwell_samples]
        )
        motors = MOTOR_BY_JOINT[joint]
        expected_controller_mode = "/".join(
            "hold" if center_observation else "position" for _motor in motors
        )
        _require(
            row["controller_mode"].strip().lower() == expected_controller_mode,
            f"{location}: endpoint controller mode mismatch",
        )
        if center_observation:
            for field in (
                "trajectory_sha256",
                "trajectory_duration_ns",
                "trajectory_execute_at_monotonic_ns",
                "plan_token_id",
                "plan_recipe_sha256",
                "collision_proof_sha256",
                "gui_command_sequence",
                "gui_command_source_instance_id",
                "gui_command_source_monotonic_ns",
            ):
                _require(
                    row[field].strip() == "CENTER_OBSERVATION",
                    f"{location}.{field}: CENTER_OBSERVATION sentinel required",
                )
            _require(not execution_samples, f"{location}: center must not claim an execution trace")
        else:
            for field in (
                "trajectory_sha256",
                "plan_token_id",
                "plan_recipe_sha256",
                "collision_proof_sha256",
            ):
                _require(
                    SHA256_RE.fullmatch(row[field].strip()) is not None,
                    f"{location}.{field}: SHA-256 required",
                )
            duration_ns = _row_integer(row, "trajectory_duration_ns", location)
            execute_at_ns = _row_integer(
                row, "trajectory_execute_at_monotonic_ns", location
            )
            _require(
                0 < duration_ns <= 15_000_000_000
                and 0 < execute_at_ns <= receipt,
                f"{location}: trajectory duration/execute time invalid",
            )
            descriptor = (duration_ns, execute_at_ns)
            previous_descriptor = trajectories.get(trajectory_sha)
            _require(
                previous_descriptor is None
                or previous_descriptor[0] == duration_ns,
                f"{location}: trajectory SHA reused with a different duration",
            )
            trajectories.setdefault(trajectory_sha, descriptor)
            trajectory_duration_total_ns += duration_ns
            gui_sequence = _row_integer(row, "gui_command_sequence", location)
            gui_source = _row_integer(
                row, "gui_command_source_monotonic_ns", location
            )
            _require(
                gui_sequence > 0
                and re.fullmatch(
                    r"[0-9a-f]{32}",
                    row["gui_command_source_instance_id"].strip(),
                )
                is not None
                and 0 < gui_source <= receipt,
                f"{location}: GUI command identity/time invalid",
            )
            gui_sequences.append(gui_sequence)
            gui_sources.append(gui_source)
            _require(execution_samples, f"{location}: motion execution trace missing")
            _require(
                execution_samples[-1]["hardware_state_sha256"]
                == dwell_samples[-1]["hardware_state_sha256"],
                f"{location}: execution and dwell do not converge on the same frame",
            )
    _strictly_increasing(stamps, f"{path.name}.monotonic_ns")
    _strictly_increasing(receipts, f"{path.name}.receipt_monotonic_ns")
    _strictly_increasing(state_sequences, f"{path.name}.hardware_state_sequence")
    _strictly_increasing(gui_sequences, f"{path.name}.gui_command_sequence")
    _strictly_increasing(gui_sources, f"{path.name}.gui_command_source_monotonic_ns")
    _require(
        first_joint_order == list(POSITION_ORDER),
        "position validation joint order must be J1,J6,J5,J4,J3,J2",
    )
    results: dict[str, float] = {}
    center_targets: dict[str, float] = {}
    maximum_sync = 0.0
    for joint in JOINT_NAMES:
        revisions = revisions_by_joint.get(joint, set())
        _require(revisions, f"position validation missing {joint}")
        _require(len(revisions) <= 3, f"{joint}: more than BASELINE + two revisions")
        final_revision = max(revisions, key=lambda value: REVISION_ORDER[value])
        final_rows = grouped[(joint, final_revision)]
        phases = tuple(row["phase"].strip().upper() for row in final_rows)
        _require(phases == POSITION_PHASES, f"{joint}: final revision phase sequence mismatch")
        _require(all(row["status"].strip().upper() == "PASS" for row in final_rows), f"{joint}: final revision contains non-PASS endpoint")
        center = _row_number(final_rows[0], "target_deg", f"{joint}.CENTER_START")
        center_targets[joint] = center
        plus = _row_number(final_rows[1], "target_deg", f"{joint}.PLUS_5")
        minus = _row_number(final_rows[3], "target_deg", f"{joint}.MINUS_5")
        _require(math.isclose(plus - center, 5.0, abs_tol=1.0e-6), f"{joint}: positive target must be exactly +5 degrees")
        _require(math.isclose(center - minus, 5.0, abs_tol=1.0e-6), f"{joint}: negative target must be exactly -5 degrees")
        actual_center = _row_number(
            final_rows[0], "actual_center_start_deg", f"{joint}.actual_center"
        )
        for row, phase in zip(final_rows, POSITION_PHASES):
            location = f"{joint}.{final_revision}.{phase}"
            target = _row_number(row, "target_deg", location)
            actual = _row_number(row, "actual_deg", location)
            error = _row_number(row, "error_deg", location)
            _require(math.isclose(error, target - actual, abs_tol=1.0e-6), f"{location}: error does not equal target-actual")
            _require(abs(error) <= 0.5 + 1.0e-9, f"{location}: endpoint error exceeds 0.5 degree")
            _require(_row_number(row, "endpoint_dwell_s", location) >= 0.5, f"{location}: dwell below 0.5 second")
            _require(_row_number(row, "temperature_c", location) < 60.0, f"{location}: hard thermal stop reached")
            _require(_row_integer(row, "merror", location) == 0, f"{location}: nonzero merror")
            _require(_row_bool(row, "communication_ok", location), f"{location}: communication failure")
            persisted_center = _row_number(row, "actual_center_start_deg", location)
            displacement = _row_number(
                row, "actual_displacement_from_center_deg", location
            )
            minimum_displacement = _row_number(
                row, "minimum_required_actual_displacement_deg", location
            )
            _require(
                math.isclose(persisted_center, actual_center, abs_tol=1.0e-6)
                and math.isclose(displacement, actual - actual_center, abs_tol=1.0e-6),
                f"{location}: actual center/displacement is not self-consistent",
            )
            if phase == "PLUS_5":
                _require(
                    math.isclose(minimum_displacement, 5.0, abs_tol=1.0e-12)
                    and displacement >= 5.0 - 1.0e-6,
                    f"{location}: measured positive displacement is below +5 degrees",
                )
            elif phase == "MINUS_5":
                _require(
                    math.isclose(minimum_displacement, 5.0, abs_tol=1.0e-12)
                    and displacement <= -5.0 + 1.0e-6,
                    f"{location}: measured negative displacement is above -5 degrees",
                )
            else:
                _require(
                    math.isclose(minimum_displacement, 0.0, abs_tol=1.0e-12),
                    f"{location}: center phase minimum displacement marker invalid",
                )
            sync_text = row.get("j2_e_sync_deg", "").strip()
            if sync_text:
                sync = abs(_row_number(row, "j2_e_sync_deg", location))
                maximum_sync = max(maximum_sync, sync)
                _require(sync <= 0.5, f"{location}: J2 hard sync threshold exceeded")
                if joint == "J2":
                    _require(sync <= 0.25, f"{location}: J2 warning sync threshold exceeded")
        results[joint] = max(abs(float(row["error_deg"])) for row in final_rows)
    return "PASS", {
        "maximum_error_deg": results,
        "maximum_j2_sync_deg": maximum_sync,
        "initial_center_deg": center_targets,
        "binding": dict(zip(
            ("envelope_id", "envelope_sha256", "session_id", "state_instance_id", "anchor_sha256"),
            _expected_binding_tuple(expected_binding),
        )),
        "trajectory_descriptors": trajectories,
        "trajectory_duration_total_ns": trajectory_duration_total_ns,
        "first_hardware_state_sequence": state_sequences[0],
        "first_hardware_state_source_ns": stamps[0],
        "last_hardware_state_sequence": state_sequences[-1],
        "last_hardware_state_source_ns": stamps[-1],
        "confirmation_requirements": confirmation_requirements,
    }


J2_COMPARISON_FIELDS = (
    "timestamp_utc", "monotonic_ns", "receipt_monotonic_ns",
    "condition", "window_elapsed_s",
    "position_error_deg", "j2_e_sync_deg",
    "j2a_tau_feedback_rotor_nm", "j2b_tau_feedback_rotor_nm",
    "j2a_pd_rotor_nm", "j2b_pd_rotor_nm",
    "j2a_gravity_ff_rotor_nm", "j2b_gravity_ff_rotor_nm",
    "j2a_total_command_rotor_nm", "j2b_total_command_rotor_nm",
    "j2a_temperature_c", "j2b_temperature_c",
    "saturation_observed", "internal_opposition_detected", "status", "reason",
    "envelope_id", "envelope_sha256", "session_id", "state_instance_id",
    "anchor_sha256", "hardware_state_sha256",
    "target_j2_rad", "frozen_pose_rad_json", "frozen_pose_sha256",
    "frozen_initial_hardware_state_sha256",
    "actual_pose_rad_json", "maximum_pose_error_deg",
    "hardware_state_sequence", "hardware_state_source_monotonic_ns",
)


def _validate_j2_comparison(
    path: Path, expected_binding: Mapping[str, Any]
) -> tuple[str, dict[str, Any]]:
    rows = _read_csv(path, J2_COMPARISON_FIELDS)
    nonpass = _csv_all_structured_nonpass(rows, path.name)
    if nonpass:
        return nonpass, {}
    stamps: list[int] = []
    receipts: list[int] = []
    state_sequences: list[int] = []
    groups: dict[str, list[dict[str, str]]] = defaultdict(list)
    condition_stream: list[str] = []
    frozen_identity: tuple[float, tuple[float, ...], str, str] | None = None
    confirmation_requirements: list[tuple[int, float]] = []
    for index, row in enumerate(rows, start=2):
        location = f"{path.name}:{index}"
        _require(row["status"].strip().upper() == "PASS", f"{location}: comparison row must be PASS")
        _timestamp(row["timestamp_utc"], f"{location}.timestamp_utc")
        stamp = _row_integer(row, "monotonic_ns", location)
        receipt = _row_integer(row, "receipt_monotonic_ns", location)
        state_sequence = _row_integer(row, "hardware_state_sequence", location)
        state_source = _row_integer(
            row, "hardware_state_source_monotonic_ns", location
        )
        _require(
            stamp > 0
            and state_sequence > 0
            and state_source == stamp
            and 0 <= receipt - stamp <= MAXIMUM_RECEIPT_AGE_NS,
            f"{location}: comparison source/receipt/state identity invalid",
        )
        stamps.append(stamp)
        receipts.append(receipt)
        state_sequences.append(state_sequence)
        condition = row["condition"].strip().upper()
        _require(condition in {"WITHOUT_FF", "WITH_FF"}, f"{location}: invalid comparison condition")
        groups[condition].append(row)
        condition_stream.append(condition)
        confirmation_requirements.append(
            (receipt, None if condition == "WITHOUT_FF" else 1.0)
        )
        _validate_binding_columns(row, expected_binding, location)
        _require(
            SHA256_RE.fullmatch(row["hardware_state_sha256"].strip()) is not None,
            f"{location}: hardware state SHA invalid",
        )
        for field in (
            "window_elapsed_s", "position_error_deg", "j2_e_sync_deg",
            "j2a_tau_feedback_rotor_nm", "j2b_tau_feedback_rotor_nm",
            "j2a_pd_rotor_nm", "j2b_pd_rotor_nm",
            "j2a_gravity_ff_rotor_nm", "j2b_gravity_ff_rotor_nm",
            "j2a_total_command_rotor_nm", "j2b_total_command_rotor_nm",
            "j2a_temperature_c", "j2b_temperature_c", "target_j2_rad",
            "maximum_pose_error_deg",
        ):
            _row_number(row, field, location)
        _require(abs(_row_number(row, "j2_e_sync_deg", location)) <= 0.25, f"{location}: J2 sync warning threshold exceeded")
        _require(_row_number(row, "j2a_temperature_c", location) < 60.0 and _row_number(row, "j2b_temperature_c", location) < 60.0, f"{location}: hard thermal stop reached")
        _row_bool(row, "saturation_observed", location)
        _require(not _row_bool(row, "internal_opposition_detected", location), f"{location}: gravity/PD internal opposition detected")
        frozen_pose = _finite_vector(
            _parse_json_text(
                row["frozen_pose_rad_json"], f"{location}.frozen_pose_rad_json"
            ),
            6,
            f"{location}.frozen_pose_rad_json",
        )
        actual_pose = _finite_vector(
            _parse_json_text(
                row["actual_pose_rad_json"], f"{location}.actual_pose_rad_json"
            ),
            6,
            f"{location}.actual_pose_rad_json",
        )
        target_j2_rad = _row_number(row, "target_j2_rad", location)
        initial_hardware_sha = row[
            "frozen_initial_hardware_state_sha256"
        ].strip()
        persisted_frozen_sha = row["frozen_pose_sha256"].strip()
        _require(
            SHA256_RE.fullmatch(initial_hardware_sha) is not None,
            f"{location}: frozen initial hardware SHA invalid",
        )
        frozen_document = {
            "target_j2_rad": target_j2_rad,
            "pose_rad": list(frozen_pose),
            "initial_hardware_state_sha256": initial_hardware_sha,
        }
        _require_document_sha(
            frozen_document, persisted_frozen_sha, f"{location}.frozen_pose_sha256"
        )
        candidate_identity = (
            target_j2_rad,
            frozen_pose,
            initial_hardware_sha,
            persisted_frozen_sha,
        )
        if frozen_identity is None:
            frozen_identity = candidate_identity
        _require(
            candidate_identity == frozen_identity,
            f"{location}: comparison target/frozen pose changed between conditions",
        )
        pose_errors = [
            abs(math.degrees(actual - target))
            for actual, target in zip(actual_pose, frozen_pose)
        ]
        maximum_pose_error = _row_number(row, "maximum_pose_error_deg", location)
        _require(
            max(pose_errors) <= 0.5 + 1.0e-9
            and math.isclose(maximum_pose_error, max(pose_errors), abs_tol=1.0e-6)
            and math.isclose(
                _row_number(row, "position_error_deg", location),
                math.degrees(target_j2_rad - actual_pose[1]),
                abs_tol=1.0e-6,
            ),
            f"{location}: frozen-pose error fields are inconsistent",
        )
        for prefix in ("j2a", "j2b"):
            pd = _row_number(row, f"{prefix}_pd_rotor_nm", location)
            ff = _row_number(row, f"{prefix}_gravity_ff_rotor_nm", location)
            total = _row_number(row, f"{prefix}_total_command_rotor_nm", location)
            _require(
                math.isclose(total, pd + ff, abs_tol=1.0e-9),
                f"{location}: {prefix} total command != PD + gravity FF",
            )
    _strictly_increasing(stamps, f"{path.name}.monotonic_ns")
    _strictly_increasing(receipts, f"{path.name}.receipt_monotonic_ns")
    _strictly_increasing(state_sequences, f"{path.name}.hardware_state_sequence")
    _require(set(groups) == {"WITHOUT_FF", "WITH_FF"}, "J2 comparison requires WITHOUT_FF and WITH_FF")
    collapsed_conditions = [
        value
        for index, value in enumerate(condition_stream)
        if index == 0 or value != condition_stream[index - 1]
    ]
    _require(
        collapsed_conditions == ["WITHOUT_FF", "WITH_FF"],
        "J2 comparison must execute WITHOUT_FF before WITH_FF",
    )
    metrics: dict[str, Any] = {}
    for condition, selected in groups.items():
        _require(
            len(selected) >= 6,
            f"{condition}: continuous comparison requires at least six samples",
        )
        elapsed = [_row_number(row, "window_elapsed_s", f"{condition}.window_elapsed_s") for row in selected]
        _require(max(elapsed) - min(elapsed) >= 0.5, f"{condition}: stable comparison window below 0.5 second")
        condition_stamps = [int(row["monotonic_ns"]) for row in selected]
        condition_start_ns = condition_stamps[0]
        condition_start_elapsed = float(selected[0]["window_elapsed_s"])
        for index, row in enumerate(selected):
            _require(
                math.isclose(
                    float(row["window_elapsed_s"])
                    - condition_start_elapsed,
                    (condition_stamps[index] - condition_start_ns) * 1.0e-9,
                    abs_tol=0.010,
                ),
                f"{condition}: elapsed time disagrees with monotonic clock",
            )
        _require(
            all(
                0 < current - previous <= 100_000_000
                for previous, current in zip(
                    condition_stamps, condition_stamps[1:]
                )
            ),
            f"{condition}: comparison sample gap exceeds 100 ms",
        )
        ff_values = [
            max(
                abs(float(row["j2a_gravity_ff_rotor_nm"])),
                abs(float(row["j2b_gravity_ff_rotor_nm"])),
            )
            for row in selected
        ]
        if condition == "WITHOUT_FF":
            _require(
                max(ff_values) <= 1.0e-12,
                "WITHOUT_FF comparison contains nonzero gravity feedforward",
            )
        else:
            _require(
                min(ff_values) > 1.0e-6,
                "WITH_FF comparison lacks gravity feedforward",
            )
        saturation = sum(_row_bool(row, "saturation_observed", condition) for row in selected) / len(selected)
        torque = max(
            sum(abs(float(row[field])) for row in selected) / len(selected)
            for field in ("j2a_tau_feedback_rotor_nm", "j2b_tau_feedback_rotor_nm")
        )
        metrics[condition] = {
            "first_hardware_state_sequence": int(
                selected[0]["hardware_state_sequence"]
            ),
            "first_hardware_state_source_ns": int(
                selected[0]["hardware_state_source_monotonic_ns"]
            ),
            "last_hardware_state_sequence": int(
                selected[-1]["hardware_state_sequence"]
            ),
            "last_hardware_state_source_ns": int(
                selected[-1]["hardware_state_source_monotonic_ns"]
            ),
            "saturation_ratio": saturation,
            "sustained_rotor_torque_nm": torque,
            "mean_abs_position_error_deg": sum(
                abs(float(row["position_error_deg"])) for row in selected
            ) / len(selected),
            "peak_abs_position_error_deg": max(
                abs(float(row["position_error_deg"])) for row in selected
            ),
            "mean_abs_gravity_ff_rotor_nm_by_motor": {
                "J2A": sum(
                    abs(float(row["j2a_gravity_ff_rotor_nm"]))
                    for row in selected
                ) / len(selected),
                "J2B": sum(
                    abs(float(row["j2b_gravity_ff_rotor_nm"]))
                    for row in selected
                ) / len(selected),
            },
        }
        for prefix, fields in (
            ("pd", ("j2a_pd_rotor_nm", "j2b_pd_rotor_nm")),
            (
                "feedback",
                (
                    "j2a_tau_feedback_rotor_nm",
                    "j2b_tau_feedback_rotor_nm",
                ),
            ),
        ):
            values = [
                abs(float(row[field])) for row in selected for field in fields
            ]
            metrics[condition][f"mean_abs_{prefix}_rotor_nm"] = (
                sum(values) / len(values)
            )
            metrics[condition][f"peak_abs_{prefix}_rotor_nm"] = max(values)
    assert frozen_identity is not None
    metrics["frozen_pose_sha256"] = frozen_identity[3]
    metrics["confirmation_requirements"] = confirmation_requirements
    return "PASS", metrics


def _pass_like(value: Any, field: str) -> bool:
    if value is True:
        return True
    return _is_pass(value, field)


POST_SEGMENT_FIELDS = {
    "joint",
    "trajectory_sha256",
    "trajectory_duration_ns",
    "trajectory_execute_at_monotonic_ns",
    "plan_token_id",
    "recipe_sha256",
    "collision_proof_sha256",
    "target_rad",
    "actual_rad",
    "endpoint_error_deg",
    "endpoint_dwell_s",
    "hardware_state_sha256",
    "gui_command_sequence",
    "gui_command_source_instance_id",
    "gui_command_source_monotonic_ns",
    "execution_trace",
    "execution_trace_sha256",
    "endpoint_dwell_trace",
    "endpoint_dwell_trace_sha256",
}


def _validate_post_segments(
    value: Any,
    *,
    field: str,
    recipe_sha256: str,
    final_target_deg: Sequence[float],
) -> tuple[
    list[Mapping[str, Any]],
    list[int],
    list[int],
    dict[str, tuple[int, int]],
    int,
]:
    _require(isinstance(value, list) and value, f"{field}: non-empty list required")
    gui_sequences: list[int] = []
    gui_sources: list[int] = []
    source_instances: set[str] = set()
    records: list[Mapping[str, Any]] = []
    last_target_by_joint: dict[str, float] = {}
    previous_segment_sequence: int | None = None
    previous_segment_source_ns: int | None = None
    trajectories: dict[str, tuple[int, int]] = {}
    trajectory_duration_total_ns = 0
    for index, record in enumerate(value):
        location = f"{field}[{index}]"
        _require(
            isinstance(record, Mapping) and set(record) == POST_SEGMENT_FIELDS,
            f"{location}: exact segment fields required",
        )
        joint = str(record.get("joint", "")).strip().upper()
        _require(joint in JOINT_NAMES, f"{location}: joint invalid")
        for sha_field in (
            "trajectory_sha256",
            "plan_token_id",
            "recipe_sha256",
            "collision_proof_sha256",
            "hardware_state_sha256",
        ):
            _require(
                isinstance(record.get(sha_field), str)
                and SHA256_RE.fullmatch(str(record[sha_field])) is not None,
                f"{location}.{sha_field}: SHA-256 required",
            )
        _require(
            record.get("recipe_sha256") == recipe_sha256,
            f"{location}: recipe SHA changed",
        )
        target_rad = _number(record.get("target_rad"), f"{location}.target_rad")
        actual_rad = _number(record.get("actual_rad"), f"{location}.actual_rad")
        endpoint_error = _number(
            record.get("endpoint_error_deg"), f"{location}.endpoint_error_deg"
        )
        _require(
            math.isclose(
                endpoint_error,
                math.degrees(target_rad - actual_rad),
                abs_tol=1.0e-6,
            )
            and abs(endpoint_error) <= 0.5 + 1.0e-9,
            f"{location}: endpoint target/actual/error invalid",
        )
        execution_document = {
            "schema": "V15.31B-segment-execution-trace-v1",
            "samples": record.get("execution_trace"),
        }
        execution = _validate_motion_trace(
            execution_document,
            record.get("execution_trace_sha256"),
            field=f"{location}.execution_trace",
            schema="V15.31B-segment-execution-trace-v1",
            joint=joint,
            target_rad=target_rad,
            trajectory_sha256=str(record["trajectory_sha256"]),
            require_dwell=False,
        )
        duration_ns = _integer(
            record.get("trajectory_duration_ns"),
            f"{location}.trajectory_duration_ns",
            minimum=1,
        )
        execute_at_ns = _integer(
            record.get("trajectory_execute_at_monotonic_ns"),
            f"{location}.trajectory_execute_at_monotonic_ns",
            minimum=1,
        )
        _require(
            duration_ns <= 15_000_000_000
            and execute_at_ns <= int(execution[-1]["receipt_monotonic_ns"]),
            f"{location}: planned segment duration/execute time invalid",
        )
        trajectory_sha = str(record["trajectory_sha256"])
        descriptor = (duration_ns, execute_at_ns)
        previous_descriptor = trajectories.get(trajectory_sha)
        _require(
            previous_descriptor is None
            or previous_descriptor[0] == duration_ns,
            f"{location}: trajectory SHA reused with a different duration",
        )
        trajectories.setdefault(trajectory_sha, descriptor)
        trajectory_duration_total_ns += duration_ns
        _require(
            int(execution[-1]["hardware_state_source_monotonic_ns"])
            - int(execution[0]["hardware_state_source_monotonic_ns"])
            <= 15_000_000_000
            and abs(
                math.degrees(
                    target_rad - float(execution[0]["actual_rad"])
                )
            )
            <= 5.5,
            f"{location}: segment exceeds 15 s or the validated 5-degree envelope",
        )
        endpoint_document = {
            "schema": "V15.31B-endpoint-dwell-trace-v1",
            "samples": record.get("endpoint_dwell_trace"),
        }
        endpoint = _validate_motion_trace(
            endpoint_document,
            record.get("endpoint_dwell_trace_sha256"),
            field=f"{location}.endpoint_dwell_trace",
            schema="V15.31B-endpoint-dwell-trace-v1",
            joint=joint,
            target_rad=target_rad,
            trajectory_sha256=str(record["trajectory_sha256"]),
            require_dwell=True,
        )
        if previous_segment_sequence is not None:
            assert previous_segment_source_ns is not None
            _require(
                int(execution[0]["hardware_state_sequence"])
                > previous_segment_sequence
                and int(execution[0]["hardware_state_source_monotonic_ns"])
                > previous_segment_source_ns,
                f"{location}: segment machine trace is reordered",
            )
        previous_segment_sequence = int(
            endpoint[-1]["hardware_state_sequence"]
        )
        previous_segment_source_ns = int(
            endpoint[-1]["hardware_state_source_monotonic_ns"]
        )
        endpoint_span_s = (
            int(endpoint[-1]["hardware_state_source_monotonic_ns"])
            - int(endpoint[0]["hardware_state_source_monotonic_ns"])
        ) / 1.0e9
        _require(
            execution[-1]["hardware_state_sha256"]
            == endpoint[-1]["hardware_state_sha256"]
            == record.get("hardware_state_sha256")
            and math.isclose(
                actual_rad, float(endpoint[-1]["actual_rad"]), abs_tol=1.0e-9
            )
            and math.isclose(
                _number(record.get("endpoint_dwell_s"), f"{location}.endpoint_dwell_s"),
                endpoint_span_s,
                abs_tol=1.0e-9,
            ),
            f"{location}: endpoint summary does not bind the retained traces",
        )
        gui_sequence = _integer(
            record.get("gui_command_sequence"),
            f"{location}.gui_command_sequence",
            minimum=1,
        )
        gui_source = _integer(
            record.get("gui_command_source_monotonic_ns"),
            f"{location}.gui_command_source_monotonic_ns",
            minimum=1,
        )
        source_id = record.get("gui_command_source_instance_id")
        _require(
            isinstance(source_id, str)
            and re.fullmatch(r"[0-9a-f]{32}", source_id) is not None
            and gui_source <= int(execution[0]["receipt_monotonic_ns"]),
            f"{location}: GUI command identity/time invalid",
        )
        gui_sequences.append(gui_sequence)
        gui_sources.append(gui_source)
        source_instances.add(source_id)
        last_target_by_joint[joint] = target_rad
        records.append(record)
    _strictly_increasing(gui_sequences, f"{field}.gui_command_sequence")
    _strictly_increasing(gui_sources, f"{field}.gui_command_source_monotonic_ns")
    _require(
        len(source_instances) == 1,
        f"{field}: GUI command source instance changed",
    )
    for joint, target_rad in last_target_by_joint.items():
        expected = math.radians(float(final_target_deg[JOINT_NAMES.index(joint)]))
        _require(
            math.isclose(target_rad, expected, abs_tol=1.0e-9),
            f"{field}: final {joint} segment does not reach the frozen target",
        )
    return (
        records,
        gui_sequences,
        gui_sources,
        trajectories,
        trajectory_duration_total_ns,
    )


def _post_stage_bounds(
    records: Sequence[Mapping[str, Any]],
    final_trace: Sequence[Mapping[str, Any]],
    field: str,
) -> tuple[int, int, int, int]:
    first_execution = records[0]["execution_trace"][0]
    last_endpoint = records[-1]["endpoint_dwell_trace"][-1]
    first_final = final_trace[0]
    last_final = final_trace[-1]
    _require(
        int(first_final["hardware_state_sequence"])
        > int(last_endpoint["hardware_state_sequence"])
        and int(first_final["hardware_state_source_monotonic_ns"])
        > int(last_endpoint["hardware_state_source_monotonic_ns"]),
        f"{field}: final HOLD trace precedes its last motion segment",
    )
    return (
        int(first_execution["hardware_state_sequence"]),
        int(first_execution["hardware_state_source_monotonic_ns"]),
        int(last_final["hardware_state_sequence"]),
        int(last_final["hardware_state_source_monotonic_ns"]),
    )


def _validate_multi_joint(
    document: Mapping[str, Any],
    expected_binding: Mapping[str, Any],
    initial_center_deg: Mapping[str, Any],
) -> tuple[str, dict[str, Any]]:
    status = _document_status(document, "multi_joint_validation.json")
    if status not in {"PASS", "FULL_PASS"}:
        return status, {}
    _require(
        document.get("schema") == "V15.31B-multi-joint-validation-v1",
        "multi_joint schema mismatch",
    )
    _require(
        set(document)
        == {
            "schema", "result", "binding", "planned_preview",
            "preview_before_real", "planned_twin", "actual_twin",
            "explicit_user_submit", "actual_twin_source", "start_deg",
            "target_deg", "final_deg", "max_error_deg_by_joint",
            "minimum_dwell_s", "recipe_sha256", "segments",
            "final_dwell_trace", "final_dwell_trace_sha256",
            "final_hardware_state_sha256", "thermal_setup",
            "restore_initial",
        },
        "multi_joint top-level fields mismatch",
    )
    binding = document.get("binding")
    _require(isinstance(binding, Mapping), "multi_joint.binding object required")
    _require(
        set(binding)
        == {"envelope_id", "envelope_sha256", "session_id", "state_instance_id", "anchor_sha256"}
        and _expected_binding_tuple(binding) == _expected_binding_tuple(expected_binding),
        "multi_joint binding mismatch",
    )
    for field in ("planned_preview", "preview_before_real", "planned_twin", "actual_twin"):
        _require(_pass_like(document.get(field), f"multi_joint.{field}"), f"multi_joint.{field} must PASS")
    _require(document.get("explicit_user_submit") is True, "multi_joint requires explicit user submit")
    _require(str(document.get("actual_twin_source", "")).upper() == "REAL_ENCODER", "multi_joint actual twin must use real encoder")
    start = _finite_vector(document.get("start_deg"), 6, "multi_joint.start_deg")
    target = _finite_vector(document.get("target_deg"), 6, "multi_joint.target_deg")
    final = _finite_vector(document.get("final_deg"), 6, "multi_joint.final_deg")
    session_center = tuple(
        _number(initial_center_deg.get(joint), f"initial_center_deg.{joint}")
        for joint in JOINT_NAMES
    )
    _require(
        all(
            abs(end - center) <= 5.0 + 1.0e-6
            for center, end in zip(session_center, target)
        ),
        "multi_joint target exceeds the session +/-5 degree envelope",
    )
    changed = [
        joint
        for joint, begin, end in zip(JOINT_NAMES, start, target)
        if abs(end - begin) > 0.01
    ]
    _require(len(changed) >= 2, "multi_joint must move at least two joints")
    recipe_sha = document.get("recipe_sha256")
    _require(
        isinstance(recipe_sha, str) and SHA256_RE.fullmatch(recipe_sha) is not None,
        "multi_joint.recipe_sha256 invalid",
    )
    (
        segments,
        _multi_gui_sequences,
        _multi_gui_sources,
        multi_trajectories,
        multi_trajectory_duration_ns,
    ) = _validate_post_segments(
        document.get("segments"),
        field="multi_joint.segments",
        recipe_sha256=recipe_sha,
        final_target_deg=target,
    )
    _require(
        set(changed).issubset({str(record["joint"]) for record in segments}),
        "multi_joint segments do not cover every changed joint",
    )
    final_trace = _validate_final_dwell_trace(
        document.get("final_dwell_trace"),
        document.get("final_dwell_trace_sha256"),
        field="multi_joint.final_dwell_trace",
        target_deg=target,
    )
    multi_bounds = _post_stage_bounds(
        segments, final_trace, "multi_joint"
    )
    final_trace_position = _finite_vector(
        final_trace[-1]["position_rad"], 6, "multi_joint.final_trace.position_rad"
    )
    _require(
        all(
            math.isclose(value, math.degrees(final_trace_position[index]), abs_tol=1.0e-6)
            for index, value in enumerate(final)
        )
        and document.get("final_hardware_state_sha256")
        == final_trace[-1]["hardware_state_sha256"],
        "multi_joint final summary does not bind the final dwell frame",
    )
    errors = _exact_keys(document.get("max_error_deg_by_joint"), JOINT_NAMES, "multi_joint.max_error_deg_by_joint")
    maximum = 0.0
    for joint in JOINT_NAMES:
        value = abs(_number(errors[joint], f"multi_joint.max_error_deg_by_joint.{joint}"))
        _require(value <= 0.5 + 1.0e-9, f"multi_joint {joint} error exceeds 0.5 degree")
        maximum = max(maximum, value)
    dwell_s = (
        int(final_trace[-1]["hardware_state_source_monotonic_ns"])
        - int(final_trace[0]["hardware_state_source_monotonic_ns"])
    ) / 1.0e9
    _require(
        math.isclose(
            _number(document.get("minimum_dwell_s"), "multi_joint.minimum_dwell_s"),
            dwell_s,
            abs_tol=1.0e-9,
        ),
        "multi_joint dwell disagrees with source trace",
    )
    for joint_index, joint in enumerate(JOINT_NAMES):
        observed_max = max(
            _number(
                sample["error_deg_by_joint"][joint],
                f"multi_joint.final_dwell_trace.{joint}",
            )
            for sample in final_trace
        )
        _require(
            math.isclose(
                _number(errors[joint], f"multi_joint.max_error_deg_by_joint.{joint}"),
                observed_max,
                abs_tol=1.0e-6,
            ),
            f"multi_joint {joint} maximum error disagrees with final dwell trace",
        )
    thermal_setup = document.get("thermal_setup")
    _require(
        isinstance(thermal_setup, Mapping)
        and set(thermal_setup)
        == {
            "result", "status", "actual_twin_source", "start_deg",
            "target_deg", "final_deg", "max_error_deg_by_joint",
            "minimum_dwell_s", "recipe_sha256", "segments",
            "final_dwell_trace", "final_dwell_trace_sha256",
            "final_hardware_state_sha256",
        }
        and _pass_like(thermal_setup.get("result"), "thermal_setup.result")
        and _pass_like(thermal_setup.get("status"), "thermal_setup.status")
        and thermal_setup.get("actual_twin_source") == "REAL_ENCODER",
        "thermal_setup machine observation invalid",
    )
    thermal_start = _finite_vector(
        thermal_setup.get("start_deg"), 6, "thermal_setup.start_deg"
    )
    thermal_target = _finite_vector(
        thermal_setup.get("target_deg"), 6, "thermal_setup.target_deg"
    )
    thermal_final = _finite_vector(
        thermal_setup.get("final_deg"), 6, "thermal_setup.final_deg"
    )
    _require(
        abs(thermal_target[1] - thermal_start[1]) > 0.01
        and all(
            abs(end - center) <= 5.0 + 1.0e-6
            for center, end in zip(session_center, thermal_target)
        ),
        "thermal_setup must move J2 within the validated envelope",
    )
    thermal_recipe = thermal_setup.get("recipe_sha256")
    _require(
        isinstance(thermal_recipe, str)
        and SHA256_RE.fullmatch(thermal_recipe) is not None,
        "thermal_setup.recipe_sha256 invalid",
    )
    (
        thermal_segments,
        _thermal_gui_sequences,
        _thermal_gui_sources,
        thermal_setup_trajectories,
        thermal_setup_trajectory_duration_ns,
    ) = _validate_post_segments(
        thermal_setup.get("segments"),
        field="thermal_setup.segments",
        recipe_sha256=thermal_recipe,
        final_target_deg=thermal_target,
    )
    _require(
        "J2" in {str(record["joint"]) for record in thermal_segments},
        "thermal_setup has no observed J2 segment",
    )
    thermal_trace = _validate_final_dwell_trace(
        thermal_setup.get("final_dwell_trace"),
        thermal_setup.get("final_dwell_trace_sha256"),
        field="thermal_setup.final_dwell_trace",
        target_deg=thermal_target,
    )
    thermal_setup_bounds = _post_stage_bounds(
        thermal_segments, thermal_trace, "thermal_setup"
    )
    _require(
        thermal_setup_bounds[0] > multi_bounds[2]
        and thermal_setup_bounds[1] > multi_bounds[3],
        "THERMAL_SETUP machine trace must follow MULTI_JOINT",
    )
    thermal_trace_position = _finite_vector(
        thermal_trace[-1]["position_rad"], 6, "thermal_setup.final_trace.position_rad"
    )
    _require(
        all(
            math.isclose(value, math.degrees(thermal_trace_position[index]), abs_tol=1.0e-6)
            for index, value in enumerate(thermal_final)
        )
        and thermal_setup.get("final_hardware_state_sha256")
        == thermal_trace[-1]["hardware_state_sha256"],
        "thermal_setup final summary does not bind its final dwell frame",
    )
    thermal_dwell_s = (
        int(thermal_trace[-1]["hardware_state_source_monotonic_ns"])
        - int(thermal_trace[0]["hardware_state_source_monotonic_ns"])
    ) / 1.0e9
    _require(
        math.isclose(
            _number(
                thermal_setup.get("minimum_dwell_s"),
                "thermal_setup.minimum_dwell_s",
            ),
            thermal_dwell_s,
            abs_tol=1.0e-9,
        ),
        "thermal_setup dwell disagrees with source trace",
    )
    thermal_errors = _exact_keys(
        thermal_setup.get("max_error_deg_by_joint"),
        JOINT_NAMES,
        "thermal_setup.max_error_deg_by_joint",
    )
    for joint in JOINT_NAMES:
        observed_max = max(
            _number(
                sample["error_deg_by_joint"][joint],
                f"thermal_setup.final_dwell_trace.{joint}",
            )
            for sample in thermal_trace
        )
        _require(
            math.isclose(
                _number(thermal_errors[joint], f"thermal_setup.max_error.{joint}"),
                observed_max,
                abs_tol=1.0e-6,
            ),
            f"thermal_setup {joint} maximum error mismatch",
        )

    restore = document.get("restore_initial")
    _require(isinstance(restore, Mapping), "multi_joint.restore_initial object required")
    _require(
        set(restore)
        == {
            "result", "status", "planned_preview", "explicit_user_submit",
            "actual_twin_source", "target_deg", "final_deg",
            "max_error_deg_by_joint", "minimum_dwell_s", "recipe_sha256",
            "segments", "final_dwell_trace", "final_dwell_trace_sha256",
            "final_hardware_state_sha256",
        },
        "multi_joint.restore_initial fields mismatch",
    )
    _require(
        _pass_like(restore.get("result"), "restore.result")
        and _pass_like(restore.get("status"), "restore.status")
        and _pass_like(restore.get("planned_preview"), "restore.planned_preview")
        and restore.get("explicit_user_submit") is True
        and restore.get("actual_twin_source") == "REAL_ENCODER",
        "restore_initial UI/status attestation invalid",
    )
    restore_target = _finite_vector(
        restore.get("target_deg"), 6, "restore_initial.target_deg"
    )
    _require(
        all(
            math.isclose(actual, expected, abs_tol=1.0e-6)
            for actual, expected in zip(restore_target, session_center)
        ),
        "restore_initial target is not the session initialization pose",
    )
    restore_final = _finite_vector(
        restore.get("final_deg"), 6, "restore_initial.final_deg"
    )
    restore_recipe = restore.get("recipe_sha256")
    _require(
        isinstance(restore_recipe, str)
        and SHA256_RE.fullmatch(restore_recipe) is not None,
        "restore_initial.recipe_sha256 invalid",
    )
    (
        restore_segments,
        _restore_gui_sequences,
        _restore_gui_sources,
        restore_trajectories,
        restore_trajectory_duration_ns,
    ) = _validate_post_segments(
        restore.get("segments"),
        field="restore_initial.segments",
        recipe_sha256=restore_recipe,
        final_target_deg=restore_target,
    )
    _require(restore_segments, "restore_initial requires observed machine segments")
    restore_trace = _validate_final_dwell_trace(
        restore.get("final_dwell_trace"),
        restore.get("final_dwell_trace_sha256"),
        field="restore_initial.final_dwell_trace",
        target_deg=restore_target,
    )
    restore_bounds = _post_stage_bounds(
        restore_segments, restore_trace, "restore_initial"
    )
    _require(
        restore_bounds[0] > thermal_setup_bounds[2]
        and restore_bounds[1] > thermal_setup_bounds[3],
        "RESTORE_INITIAL machine trace must follow THERMAL_SETUP",
    )
    restore_trace_position = _finite_vector(
        restore_trace[-1]["position_rad"], 6, "restore_initial.final_trace.position_rad"
    )
    _require(
        all(
            math.isclose(value, math.degrees(restore_trace_position[index]), abs_tol=1.0e-6)
            for index, value in enumerate(restore_final)
        )
        and restore.get("final_hardware_state_sha256")
        == restore_trace[-1]["hardware_state_sha256"],
        "restore_initial final summary does not bind the final dwell frame",
    )
    restore_dwell_s = (
        int(restore_trace[-1]["hardware_state_source_monotonic_ns"])
        - int(restore_trace[0]["hardware_state_source_monotonic_ns"])
    ) / 1.0e9
    _require(
        math.isclose(
            _number(restore.get("minimum_dwell_s"), "restore.minimum_dwell_s"),
            restore_dwell_s,
            abs_tol=1.0e-9,
        ),
        "restore_initial dwell disagrees with source trace",
    )
    restore_errors = _exact_keys(
        restore.get("max_error_deg_by_joint"),
        JOINT_NAMES,
        "restore_initial.max_error_deg_by_joint",
    )
    for joint in JOINT_NAMES:
        observed_max = max(
            _number(
                sample["error_deg_by_joint"][joint],
                f"restore_initial.final_dwell_trace.{joint}",
            )
            for sample in restore_trace
        )
        _require(
            math.isclose(
                _number(restore_errors[joint], f"restore_initial.max_error.{joint}"),
                observed_max,
                abs_tol=1.0e-6,
            ),
            f"restore_initial {joint} maximum error mismatch",
        )
    all_trajectories: dict[str, tuple[int, int]] = {}
    for collection in (
        multi_trajectories,
        thermal_setup_trajectories,
        restore_trajectories,
    ):
        for sha, descriptor in collection.items():
            _require(
                sha not in all_trajectories
                or all_trajectories[sha][0] == descriptor[0],
                "multi/post trajectory SHA duration conflict",
            )
            all_trajectories.setdefault(sha, descriptor)
    confirmation_requirements: list[tuple[int, float]] = []
    for records, final_samples in (
        (segments, final_trace),
        (thermal_segments, thermal_trace),
        (restore_segments, restore_trace),
    ):
        for record in records:
            for sample in (
                list(record["execution_trace"])
                + list(record["endpoint_dwell_trace"])
            ):
                confirmation_requirements.append(
                    (int(sample["receipt_monotonic_ns"]), 1.0)
                )
        confirmation_requirements.extend(
            (int(sample["receipt_monotonic_ns"]), 1.0)
            for sample in final_samples
        )
    return status, {
        "maximum_error_deg": maximum,
        "restore_initial": restore,
        "restored_final_hardware_state_sha256": restore[
            "final_hardware_state_sha256"
        ],
        "restored_final_deg": list(restore_final),
        "multi_first_sequence": multi_bounds[0],
        "multi_first_source_ns": multi_bounds[1],
        "multi_last_sequence": multi_bounds[2],
        "multi_last_source_ns": multi_bounds[3],
        "thermal_setup_first_sequence": thermal_setup_bounds[0],
        "thermal_setup_first_source_ns": thermal_setup_bounds[1],
        "thermal_setup_last_sequence": thermal_setup_bounds[2],
        "thermal_setup_last_source_ns": thermal_setup_bounds[3],
        "restore_first_sequence": restore_bounds[0],
        "restore_first_source_ns": restore_bounds[1],
        "restore_last_sequence": restore_bounds[2],
        "restore_last_source_ns": restore_bounds[3],
        "trajectory_descriptors": all_trajectories,
        "trajectory_duration_total_ns": (
            multi_trajectory_duration_ns
            + thermal_setup_trajectory_duration_ns
            + restore_trajectory_duration_ns
        ),
        "confirmation_requirements": confirmation_requirements,
    }


THERMAL_FIELDS = (
    "timestamp_utc", "monotonic_ns", "receipt_monotonic_ns", "elapsed_s",
    "j2a_temperature_c", "j2b_temperature_c",
    "j2a_slope_c_per_min", "j2b_slope_c_per_min",
    "j2a_tau_feedback_rotor_nm", "j2b_tau_feedback_rotor_nm",
    "j2a_gravity_ff_rotor_nm", "j2b_gravity_ff_rotor_nm",
    "j2a_pd_rotor_nm", "j2b_pd_rotor_nm", "position_error_deg",
    "j2_e_sync_deg", "saturation_observed", "j2a_merror", "j2b_merror",
    "j2a_communication_ok", "j2b_communication_ok", "status", "reason",
    "envelope_id", "envelope_sha256", "session_id", "state_instance_id",
    "anchor_sha256", "hardware_state_sha256",
    "hardware_state_sequence", "hardware_state_source_monotonic_ns",
)


def _validate_thermal_csv(
    path: Path,
    duration_min: int,
    expected_binding: Mapping[str, Any],
) -> tuple[str, dict[str, Any]]:
    rows = _read_csv(path, THERMAL_FIELDS)
    nonpass = _csv_all_structured_nonpass(rows, path.name)
    if nonpass:
        if nonpass == "NOT_RUN_EARLY_FUNDAMENTAL_LIMIT":
            _require(
                len(rows) == 1
                and rows[0]["reason"].strip().upper()
                == "NOT_RUN_EARLY_FUNDAMENTAL_LIMIT",
                f"{path.name}: early-B NOT_RUN requires one structured row",
            )
            _timestamp(rows[0]["timestamp_utc"], f"{path.name}.timestamp_utc")
            _validate_binding_columns(rows[0], expected_binding, path.name)
        return nonpass, {"not_run_reason": nonpass}
    stamps: list[int] = []
    receipts: list[int] = []
    state_sequences: list[int] = []
    elapsed: list[float] = []
    maximum_temperature = -math.inf
    saturation_count = 0
    row_binding: tuple[str, str, str, str, str] | None = None
    for index, row in enumerate(rows, start=2):
        location = f"{path.name}:{index}"
        _require(row["status"].strip().upper() == "PASS", f"{location}: completed thermal row must be PASS")
        _timestamp(row["timestamp_utc"], f"{location}.timestamp_utc")
        stamp = _row_integer(row, "monotonic_ns", location)
        receipt = _row_integer(row, "receipt_monotonic_ns", location)
        state_sequence = _row_integer(row, "hardware_state_sequence", location)
        state_source = _row_integer(
            row, "hardware_state_source_monotonic_ns", location
        )
        _require(
            stamp > 0
            and state_sequence > 0
            and state_source == stamp
            and 0 <= receipt - stamp <= MAXIMUM_RECEIPT_AGE_NS,
            f"{location}: thermal source/receipt/state identity invalid",
        )
        stamps.append(stamp)
        receipts.append(receipt)
        state_sequences.append(state_sequence)
        elapsed.append(_row_number(row, "elapsed_s", location))
        candidate_binding = (
            row["envelope_id"].strip(),
            row["envelope_sha256"].strip(),
            row["session_id"].strip(),
            row["state_instance_id"].strip(),
            row["anchor_sha256"].strip(),
        )
        _validate_binding_columns(row, expected_binding, location)
        _require(candidate_binding == _expected_binding_tuple(expected_binding), f"{location}: thermal evidence binding mismatch")
        if row_binding is None:
            row_binding = candidate_binding
        _require(candidate_binding == row_binding, f"{location}: thermal binding changed")
        _require(
            SHA256_RE.fullmatch(row["hardware_state_sha256"].strip()) is not None,
            f"{location}: hardware state SHA invalid",
        )
        for field in (
            "elapsed_s", "j2a_temperature_c", "j2b_temperature_c",
            "j2a_slope_c_per_min", "j2b_slope_c_per_min",
            "j2a_tau_feedback_rotor_nm", "j2b_tau_feedback_rotor_nm",
            "j2a_gravity_ff_rotor_nm", "j2b_gravity_ff_rotor_nm",
            "j2a_pd_rotor_nm", "j2b_pd_rotor_nm", "position_error_deg",
            "j2_e_sync_deg",
        ):
            _row_number(row, field, location)
        a_temp = _row_number(row, "j2a_temperature_c", location)
        b_temp = _row_number(row, "j2b_temperature_c", location)
        maximum_temperature = max(maximum_temperature, a_temp, b_temp)
        _require(max(a_temp, b_temp) < 60.0, f"{location}: PASS data reaches hard thermal stop")
        _require(
            abs(_row_number(row, "position_error_deg", location)) <= 0.5,
            f"{location}: J2 position error exceeds 0.5 degree",
        )
        _require(abs(_row_number(row, "j2_e_sync_deg", location)) <= 0.25, f"{location}: J2 sync warning threshold exceeded")
        saturation_count += int(_row_bool(row, "saturation_observed", location))
        _require(_row_integer(row, "j2a_merror", location) == 0 and _row_integer(row, "j2b_merror", location) == 0, f"{location}: nonzero J2 merror")
        _require(_row_bool(row, "j2a_communication_ok", location) and _row_bool(row, "j2b_communication_ok", location), f"{location}: J2 communication failure")
    _strictly_increasing(stamps, f"{path.name}.monotonic_ns")
    _strictly_increasing(receipts, f"{path.name}.receipt_monotonic_ns")
    _strictly_increasing(state_sequences, f"{path.name}.hardware_state_sequence")
    _require(
        max(
            float(rows[0]["j2a_temperature_c"]),
            float(rows[0]["j2b_temperature_c"]),
        )
        < 55.0,
        f"{path.name}: thermal stage entry temperature must be below 55 C",
    )
    minimum_samples = math.ceil(duration_min * 60.0 / 2.0) + 1
    _require(
        len(rows) >= minimum_samples,
        f"{path.name}: continuous thermal evidence requires at least "
        f"{minimum_samples} samples",
    )
    for index, ((previous_stamp, current_stamp), (previous_elapsed, current_elapsed)) in enumerate(
        zip(zip(stamps, stamps[1:]), zip(elapsed, elapsed[1:])),
        start=3,
    ):
        gap_ns = current_stamp - previous_stamp
        _require(
            gap_ns <= THERMAL_MAXIMUM_SAMPLE_GAP_NS,
            f"{path.name}:{index}: thermal sample gap exceeds 2 seconds",
        )
        elapsed_delta = current_elapsed - previous_elapsed
        _require(
            elapsed_delta > 0.0,
            f"{path.name}:{index}: elapsed_s must be strictly increasing",
        )
        monotonic_delta_s = gap_ns * 1.0e-9
        _require(
            math.isclose(
                elapsed_delta,
                monotonic_delta_s,
                rel_tol=0.0,
                abs_tol=THERMAL_ELAPSED_MONOTONIC_TOLERANCE_S,
            ),
            f"{path.name}:{index}: elapsed_s is inconsistent with monotonic_ns",
        )
    _require(min(elapsed) <= 1.0, f"{path.name}: stage does not begin near zero")
    _require(max(elapsed) - min(elapsed) >= duration_min * 60.0, f"{path.name}: stage duration below {duration_min} minutes")
    trend_rows = [
        row
        for row in rows
        if stamps[-1] - int(row["monotonic_ns"]) <= 60_000_000_000
    ]
    _require(
        trend_rows
        and int(trend_rows[-1]["monotonic_ns"])
        - int(trend_rows[0]["monotonic_ns"])
        >= 60_000_000_000,
        f"{path.name}: terminal 60-second trend window missing",
    )
    return "PASS", {
        "start_j2a_c": float(rows[0]["j2a_temperature_c"]),
        "start_j2b_c": float(rows[0]["j2b_temperature_c"]),
        "end_j2a_c": float(rows[-1]["j2a_temperature_c"]),
        "end_j2b_c": float(rows[-1]["j2b_temperature_c"]),
        "final_slope_j2a_c_per_min": float(rows[-1]["j2a_slope_c_per_min"]),
        "final_slope_j2b_c_per_min": float(rows[-1]["j2b_slope_c_per_min"]),
        "maximum_temperature_c": maximum_temperature,
        "saturation_ratio": saturation_count / len(rows),
        "temperature_rise_j2a_c": (
            float(rows[-1]["j2a_temperature_c"])
            - float(rows[0]["j2a_temperature_c"])
        ),
        "temperature_rise_j2b_c": (
            float(rows[-1]["j2b_temperature_c"])
            - float(rows[0]["j2b_temperature_c"])
        ),
        "minimum_terminal_60s_slope_j2a_c_per_min": min(
            float(row["j2a_slope_c_per_min"]) for row in trend_rows
        ),
        "minimum_terminal_60s_slope_j2b_c_per_min": min(
            float(row["j2b_slope_c_per_min"]) for row in trend_rows
        ),
        "first_hardware_state_sequence": state_sequences[0],
        "first_hardware_state_source_ns": stamps[0],
        "last_hardware_state_sequence": state_sequences[-1],
        "last_hardware_state_source_ns": stamps[-1],
        "confirmation_requirements": [
            (receipt, 1.0) for receipt in receipts
        ],
    }


def _positive_number_anywhere(value: Any) -> bool:
    if type(value) in {int, float}:
        return math.isfinite(float(value)) and float(value) > 0.0
    if isinstance(value, Mapping):
        return any(_positive_number_anywhere(item) for item in value.values())
    if isinstance(value, list):
        return any(_positive_number_anywhere(item) for item in value)
    return False


def _materially_lower(before: float, after: float) -> bool:
    required = max(
        MINIMUM_ABSOLUTE_PD_IMPROVEMENT_NM,
        before * MINIMUM_RELATIVE_PD_IMPROVEMENT,
    )
    return after < before and before - after >= required


def _validate_thermal_summary(
    document: Mapping[str, Any],
    stage_statuses: Mapping[str, str],
    comparison_metrics: Mapping[str, Any],
    stage_metrics: Mapping[str, Mapping[str, Any]],
    expected_binding: Mapping[str, Any],
) -> tuple[str, dict[str, Any]]:
    raw_status = document.get("result", document.get("status"))
    _require(raw_status is not None, "thermal_summary.json: result/status required")
    status = _status_token(raw_status, "thermal_summary.json.result")
    classification = str(document.get("classification", "")).strip().upper()
    if status not in {"PASS", "FULL_PASS", "HARDWARE_COUNTERBALANCE_REQUIRED"}:
        reason = document.get("reason", document.get("primary_blocker"))
        _require(
            isinstance(reason, str) and reason.strip(),
            "blocked thermal summary requires reason",
        )
        _require(
            classification.startswith("UNDETERMINED"),
            "blocked thermal summary must not claim final A/B classification",
        )
        return status, {"classification": classification}

    _require(
        document.get("schema", document.get("schema_version"))
        == "V15.31B-thermal-summary-v1",
        "thermal summary schema mismatch",
    )
    binding = document.get("binding")
    _require(isinstance(binding, Mapping), "thermal summary binding object required")
    for field in (
        "envelope_id", "envelope_sha256", "session_id",
        "state_instance_id", "anchor_sha256",
    ):
        _require(
            binding.get(field) == expected_binding.get(field),
            f"thermal summary binding mismatch: {field}",
        )
    _require(
        classification in {
            "CONTROL_EXCESS_TORQUE_SOLVED",
            "FUNDAMENTAL_CONTINUOUS_LOAD_LIMIT",
        },
        "thermal classification must be contract A or B",
    )
    counterbalance = str(document.get("counterbalance_required", "")).strip().upper()
    _require(
        counterbalance in {"YES", "NO"},
        "thermal counterbalance_required must be YES/NO",
    )

    motors: dict[str, Mapping[str, Any]] = {}
    for name in ("J2A", "J2B"):
        value = document.get(name)
        _require(isinstance(value, Mapping), f"thermal_summary.{name}: object required")
        for field in ("start_temperature_c", "final_slope_c_per_min"):
            _number(value.get(field), f"thermal_summary.{name}.{field}")
        _integer(
            value.get("final_slope_observed_stage_min"),
            f"thermal_summary.{name}.final_slope_observed_stage_min",
            minimum=5,
        )
        motors[name] = value

    torque = document.get("j2_sustained_rotor_torque")
    _require(
        isinstance(torque, Mapping),
        "thermal_summary.j2_sustained_rotor_torque object required",
    )
    before = _number(
        torque.get("before_gravity_nm"), "thermal_summary.before_gravity_nm"
    )
    after = _number(
        torque.get("after_gravity_nm"), "thermal_summary.after_gravity_nm"
    )
    saturation = document.get("saturation_ratio")
    _require(
        isinstance(saturation, Mapping),
        "thermal_summary.saturation_ratio object required",
    )
    before_sat = _number(
        saturation.get("before"), "thermal_summary.saturation_ratio.before"
    )
    after_sat = _number(
        saturation.get("after"), "thermal_summary.saturation_ratio.after"
    )
    _require(
        0.0 <= before_sat <= 1.0 and 0.0 <= after_sat <= 1.0,
        "thermal saturation ratios must be in [0,1]",
    )

    without_ff = comparison_metrics.get("WITHOUT_FF")
    with_ff = comparison_metrics.get("WITH_FF")
    _require(
        isinstance(without_ff, Mapping) and isinstance(with_ff, Mapping),
        "thermal summary requires validated WITHOUT_FF/WITH_FF comparison",
    )
    _require(
        math.isclose(
            before,
            float(without_ff["sustained_rotor_torque_nm"]),
            abs_tol=1.0e-6,
        )
        and math.isclose(
            after,
            float(with_ff["sustained_rotor_torque_nm"]),
            abs_tol=1.0e-6,
        ),
        "thermal sustained torque disagrees with J2 comparison CSV",
    )
    _require(
        math.isclose(
            before_sat, float(without_ff["saturation_ratio"]), abs_tol=1.0e-6
        )
        and math.isclose(
            after_sat, float(with_ff["saturation_ratio"]), abs_tol=1.0e-6
        ),
        "thermal saturation ratio disagrees with J2 comparison CSV",
    )

    stage_order = ("5min", "15min", "30min")
    completed = [
        name for name in stage_order
        if stage_statuses.get(name) in {"PASS", "FULL_PASS"}
    ]
    _require(completed, "thermal summary requires at least one completed stage")
    _require(
        completed == list(stage_order[:len(completed)]),
        "thermal completed stages must be an ordered prefix",
    )
    final_completed = completed[-1]
    disposition = document.get("thermal_stage_disposition")
    _require(
        isinstance(disposition, Mapping)
        and set(disposition) == {"5", "15", "30"},
        "thermal_stage_disposition must contain exact 5/15/30 keys",
    )
    for stage_name, stage_number in zip(stage_order, (5, 15, 30)):
        expected_disposition = (
            "PASS_OBSERVED"
            if stage_name in completed
            else "NOT_RUN_EARLY_FUNDAMENTAL_LIMIT"
        )
        _require(
            disposition.get(str(stage_number)) == expected_disposition,
            f"thermal stage {stage_number} disposition mismatch",
        )
    for motor, prefix in (("J2A", "j2a"), ("J2B", "j2b")):
        expected_start = float(stage_metrics["5min"][f"start_{prefix}_c"])
        _require(
            math.isclose(
                float(motors[motor]["start_temperature_c"]),
                expected_start,
                abs_tol=1.0e-6,
            ),
            f"thermal_summary.{motor}.start_temperature_c disagrees with thermal CSV",
        )
        for stage_name, summary_field in zip(
            stage_order,
            ("temperature_5min_c", "temperature_15min_c", "temperature_30min_c"),
        ):
            if stage_name in completed:
                expected = float(stage_metrics[stage_name][f"end_{prefix}_c"])
                _require(
                    math.isclose(
                        _number(
                            motors[motor][summary_field],
                            f"thermal_summary.{motor}.{summary_field}",
                        ),
                        expected,
                        abs_tol=1.0e-6,
                    ),
                    f"thermal_summary.{motor}.{summary_field} disagrees with thermal CSV",
                )
            else:
                endpoint = motors[motor][summary_field]
                _require(
                    isinstance(endpoint, Mapping)
                    and set(endpoint)
                    == {
                        "status", "observed", "value_c",
                        "last_observed_stage_min", "last_observed_value_c",
                    }
                    and endpoint.get("status")
                    == "NOT_RUN_EARLY_FUNDAMENTAL_LIMIT"
                    and endpoint.get("observed") is False
                    and endpoint.get("value_c") is None
                    and endpoint.get("last_observed_stage_min")
                    == int(final_completed.removesuffix("min"))
                    and math.isclose(
                        _number(
                            endpoint.get("last_observed_value_c"),
                            f"thermal_summary.{motor}.{summary_field}.last_observed_value_c",
                        ),
                        float(stage_metrics[final_completed][f"end_{prefix}_c"]),
                        abs_tol=1.0e-6,
                    ),
                    f"thermal_summary.{motor}.{summary_field}: structured NOT_RUN evidence invalid",
                )
        expected_slope = float(
            stage_metrics[final_completed][f"final_slope_{prefix}_c_per_min"]
        )
        _require(
            math.isclose(
                float(motors[motor]["final_slope_c_per_min"]),
                expected_slope,
                abs_tol=1.0e-6,
            ),
            f"thermal_summary.{motor}.final_slope_c_per_min disagrees with thermal CSV",
        )
        _require(
            motors[motor]["final_slope_observed_stage_min"]
            == int(final_completed.removesuffix("min")),
            f"thermal_summary.{motor}.final slope stage mismatch",
        )

    terminal_slopes = {
        name: max(
            abs(float(stage_metrics[name]["final_slope_j2a_c_per_min"])),
            abs(float(stage_metrics[name]["final_slope_j2b_c_per_min"])),
        )
        for name in completed
    }
    a_criteria = {
        "mean_abs_position_error_strictly_lower": (
            float(with_ff["mean_abs_position_error_deg"])
            < float(without_ff["mean_abs_position_error_deg"]) - 1.0e-9
        ),
        "peak_abs_position_error_strictly_lower": (
            float(with_ff["peak_abs_position_error_deg"])
            < float(without_ff["peak_abs_position_error_deg"]) - 1.0e-9
        ),
        "mean_abs_pd_materially_lower": _materially_lower(
            float(without_ff["mean_abs_pd_rotor_nm"]),
            float(with_ff["mean_abs_pd_rotor_nm"]),
        ),
        "peak_abs_pd_materially_lower": _materially_lower(
            float(without_ff["peak_abs_pd_rotor_nm"]),
            float(with_ff["peak_abs_pd_rotor_nm"]),
        ),
        "mean_abs_feedback_torque_materially_lower": _materially_lower(
            float(without_ff["mean_abs_feedback_rotor_nm"]),
            float(with_ff["mean_abs_feedback_rotor_nm"]),
        ),
        "peak_abs_feedback_torque_materially_lower": _materially_lower(
            float(without_ff["peak_abs_feedback_rotor_nm"]),
            float(with_ff["peak_abs_feedback_rotor_nm"]),
        ),
        "saturation_strictly_lower_and_eliminated": (
            before_sat > after_sat
            and math.isclose(after_sat, 0.0, abs_tol=1.0e-12)
        ),
        "with_ff_no_internal_opposition": True,
        "thermal_trend_improved": (
            set(terminal_slopes) == set(stage_order)
            and terminal_slopes["30min"]
            <= MAXIMUM_STABLE_THERMAL_SLOPE_C_PER_MIN
            and terminal_slopes["30min"] <= terminal_slopes["15min"]
            and terminal_slopes["15min"] <= terminal_slopes["5min"]
            and terminal_slopes["5min"] - terminal_slopes["30min"]
            >= MINIMUM_THERMAL_SLOPE_IMPROVEMENT_C_PER_MIN
        ),
        "thirty_minute_hold_stable": (
            "30min" in completed
            and math.isclose(
                float(stage_metrics["30min"]["saturation_ratio"]),
                0.0,
                abs_tol=1.0e-12,
            )
        ),
    }
    objective = document.get("classification_a_objective_metrics")
    _require(
        isinstance(objective, Mapping)
        and isinstance(objective.get("criteria"), Mapping),
        "thermal summary objective metrics missing",
    )
    for name, value in a_criteria.items():
        _require(
            objective["criteria"].get(name) is value,
            f"thermal summary objective criterion mismatch: {name}",
        )

    if classification == "CONTROL_EXCESS_TORQUE_SOLVED":
        _require(
            status in {"PASS", "FULL_PASS"},
            "classification A requires PASS thermal summary",
        )
        _require(counterbalance == "NO", "classification A requires counterbalance NO")
        _require(
            completed == list(stage_order),
            "classification A requires all three thermal stages PASS",
        )
        _require(
            all(a_criteria.values()),
            "classification A objective improvement is not proven by raw evidence",
        )
    else:
        _require(
            status == "HARDWARE_COUNTERBALANCE_REQUIRED",
            "classification B requires HARDWARE_COUNTERBALANCE_REQUIRED result",
        )
        _require(counterbalance == "YES", "classification B requires counterbalance YES")
        _require(
            not all(a_criteria.values()),
            "classification B contradicts proven classification A",
        )
        if len(completed) < len(stage_order):
            for stage_name in stage_order[len(completed):]:
                _require(
                    stage_statuses.get(stage_name)
                    == "NOT_RUN_EARLY_FUNDAMENTAL_LIMIT",
                    "early classification B requires structured NOT_RUN evidence",
                )
            early_metrics = stage_metrics[final_completed]
            _require(
                float(early_metrics["minimum_terminal_60s_slope_j2a_c_per_min"])
                >= MINIMUM_EARLY_FUNDAMENTAL_SLOPE_C_PER_MIN
                and float(early_metrics["minimum_terminal_60s_slope_j2b_c_per_min"])
                >= MINIMUM_EARLY_FUNDAMENTAL_SLOPE_C_PER_MIN
                and float(early_metrics["temperature_rise_j2a_c"])
                >= MINIMUM_EARLY_FUNDAMENTAL_RISE_C
                and float(early_metrics["temperature_rise_j2b_c"])
                >= MINIMUM_EARLY_FUNDAMENTAL_RISE_C
                and float(early_metrics["maximum_temperature_c"]) < 55.0,
                "early classification B lacks the raw safe 60-second hot-trend proof",
            )
        recommendation = document.get("mechanical_recommendation")
        _require(
            isinstance(recommendation, Mapping),
            "classification B requires mechanical_recommendation",
        )
        _require(
            recommendation.get("schema")
            == "V15.31B-mechanical-counterbalance-recommendation-v1",
            "mechanical recommendation schema mismatch",
        )
        with_ff_gravity = with_ff.get("mean_abs_gravity_ff_rotor_nm_by_motor")
        _require(
            isinstance(with_ff_gravity, Mapping),
            "WITH_FF gravity load metrics missing",
        )
        observed_per_motor = {
            motor: _number(
                with_ff_gravity.get(motor),
                f"comparison.WITH_FF.gravity.{motor}",
            )
            for motor in ("J2A", "J2B")
        }
        target_per_motor = {
            motor: value * (1.0 - MECHANICAL_UNLOAD_FRACTION)
            for motor, value in observed_per_motor.items()
        }
        observed_joint_nm = sum(observed_per_motor.values()) * GO_GEAR_RATIO
        target_joint_nm = sum(target_per_motor.values()) * GO_GEAR_RATIO
        expected_reduction = observed_joint_nm - target_joint_nm
        reduction = _number(
            recommendation.get("required_j2_torque_reduction_nm"),
            "mechanical_recommendation.required_j2_torque_reduction_nm",
        )
        _require(
            expected_reduction > 0.0
            and math.isclose(reduction, expected_reduction, abs_tol=1.0e-6),
            "mechanical recommendation reduction is not the raw WITH_FF 20% unload target",
        )
        basis = recommendation.get("observed_load_basis")
        _require(isinstance(basis, Mapping), "mechanical observed_load_basis missing")
        basis_observed = _exact_keys(
            basis.get("per_motor_mean_abs_gravity_rotor_nm"),
            ("J2A", "J2B"),
            "mechanical.observed_per_motor",
        )
        basis_target = _exact_keys(
            basis.get("per_motor_target_abs_gravity_rotor_nm"),
            ("J2A", "J2B"),
            "mechanical.target_per_motor",
        )
        _require(
            all(
                math.isclose(
                    _number(basis_observed[motor], f"mechanical.observed.{motor}"),
                    observed_per_motor[motor],
                    abs_tol=1.0e-6,
                )
                and math.isclose(
                    _number(basis_target[motor], f"mechanical.target.{motor}"),
                    target_per_motor[motor],
                    abs_tol=1.0e-6,
                )
                for motor in ("J2A", "J2B")
            )
            and math.isclose(
                _number(basis.get("go_gear_ratio"), "mechanical.go_gear_ratio"),
                GO_GEAR_RATIO,
                abs_tol=1.0e-9,
            )
            and math.isclose(
                _number(
                    basis.get("observed_j2_gravity_joint_nm"),
                    "mechanical.observed_j2_gravity_joint_nm",
                ),
                observed_joint_nm,
                abs_tol=1.0e-6,
            )
            and math.isclose(
                _number(
                    basis.get("target_j2_gravity_joint_nm"),
                    "mechanical.target_j2_gravity_joint_nm",
                ),
                target_joint_nm,
                abs_tol=1.0e-6,
            )
            and math.isclose(
                _number(
                    basis.get("minimum_mechanical_unload_fraction"),
                    "mechanical.minimum_unload_fraction",
                ),
                MECHANICAL_UNLOAD_FRACTION,
                abs_tol=1.0e-12,
            ),
            "mechanical recommendation observed-load basis disagrees with raw comparison",
        )
        checked_options = 0
        option_specs = {
            "counterweight": (
                {"mass_kg", "lever_arm_m", "gravity_m_s2", "estimated_torque_nm"},
                lambda option: option["mass_kg"] * option["gravity_m_s2"] * option["lever_arm_m"],
            ),
            "spring": (
                {"spring_rate_n_per_m", "deflection_m", "lever_arm_m", "estimated_torque_nm"},
                lambda option: option["spring_rate_n_per_m"] * option["deflection_m"] * option["lever_arm_m"],
            ),
            "gas_spring": (
                {"force_n", "lever_arm_m", "estimated_torque_nm"},
                lambda option: option["force_n"] * option["lever_arm_m"],
            ),
        }
        for name, (keys, formula) in option_specs.items():
            raw_option = recommendation.get(name)
            if raw_option is None:
                continue
            _require(
                isinstance(raw_option, Mapping) and set(raw_option) == keys,
                f"mechanical.{name}: exact quantitative fields required",
            )
            option = {
                key: _number(raw_option[key], f"mechanical.{name}.{key}")
                for key in keys
            }
            _require(
                all(value > 0.0 for value in option.values()),
                f"mechanical.{name}: values must be positive",
            )
            if name == "counterweight":
                _require(
                    math.isclose(
                        option["gravity_m_s2"], STANDARD_GRAVITY_M_S2, abs_tol=1.0e-6
                    ),
                    "mechanical.counterweight gravity constant invalid",
                )
            calculated = float(formula(option))
            _require(
                math.isclose(
                    option["estimated_torque_nm"], calculated, abs_tol=1.0e-6
                )
                and reduction <= calculated <= reduction * 1.25,
                f"mechanical.{name}: torque formula/range invalid",
            )
            checked_options += 1
        _require(
            checked_options >= 1,
            "classification B requires one quantitative counterweight/spring/gas-spring option",
        )
    return status, {
        "classification": classification,
        "counterbalance_required": counterbalance,
        "J2A": motors["J2A"],
        "J2B": motors["J2B"],
        "before_torque_nm": before,
        "after_torque_nm": after,
        "before_saturation": before_sat,
        "after_saturation": after_sat,
        "classification_a_criteria": a_criteria,
    }


GUI_CHECKS = (
    "single_slider_group", "drag_controls_planned_only",
    "planned_mujoco_animation", "actual_twin_encoder_only",
    "preview_before_real", "explicit_real_submit", "execution_status",
    "progress_percent", "estimated_remaining", "heartbeat",
    "temperature_table", "online_status_7_motors", "temperature_colors",
    "position_target_error",
)


def _validate_gui(
    document: Mapping[str, Any], expected_binding: Mapping[str, Any]
) -> tuple[str, dict[str, Any]]:
    status = _document_status(document, "gui_powered_validation.json")
    if status not in {"PASS", "FULL_PASS"}:
        return status, {}
    binding = document.get("binding")
    _require(
        isinstance(binding, Mapping)
        and _expected_binding_tuple(binding) == _expected_binding_tuple(expected_binding),
        "GUI powered evidence binding mismatch",
    )
    checks = document.get("checks")
    _require(isinstance(checks, Mapping), "gui_powered_validation.checks object required")
    for field in GUI_CHECKS:
        _require(_pass_like(checks.get(field), f"gui.{field}"), f"GUI powered check failed: {field}")
    return status, {}


def _extract_final_fields(document: Mapping[str, Any]) -> dict[str, Any]:
    if isinstance(document.get("report_fields"), list):
        values: dict[str, Any] = {}
        rows = document["report_fields"]
        _require(len(rows) == 31, "final_result.report_fields must contain exactly 31 fields")
        for expected_id, expected_name, row in zip(range(1, 32), FINAL_FIELD_NAMES, rows):
            _require(isinstance(row, Mapping), f"final_result field {expected_id}: object required")
            _require(row.get("id") == expected_id, f"final_result field id {expected_id} missing/out of order")
            _require(row.get("name") == expected_name, f"final_result field {expected_id} must be named {expected_name!r}")
            _require("value" in row, f"final_result field {expected_id} value missing")
            _require(expected_name not in values, f"final_result duplicate field {expected_name}")
            values[expected_name] = row["value"]
        return values
    report = document.get("report")
    if isinstance(report, Mapping):
        _require(tuple(report.keys()) == FINAL_FIELD_NAMES, "final_result.report must contain the exact 31 fields in contract order")
        return dict(report)
    _require(all(name in document for name in FINAL_FIELD_NAMES), "final_result must contain report_fields or report")
    return {name: document[name] for name in FINAL_FIELD_NAMES}


def _extract_commit(value: Any, field: str, *, allow_unavailable: bool) -> str | None:
    if isinstance(value, str):
        sha = value.strip()
    elif isinstance(value, Mapping):
        sha_value = value.get("sha", value.get("commit"))
        if sha_value is None and allow_unavailable and _is_nonpass(value, field):
            return None
        _require(isinstance(sha_value, str), f"{field}: sha/commit string required")
        sha = sha_value.strip()
    else:
        raise EvidenceValidationError(f"{field}: commit SHA required")
    _require(COMMIT_RE.fullmatch(sha) is not None, f"{field}: real lowercase 40-hex commit SHA required")
    return sha


def _commit_resolves(repo_root: Path, sha: str, field: str) -> None:
    result = subprocess.run(
        ["git", "-C", str(repo_root), "cat-file", "-e", f"{sha}^{{commit}}"],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    _require(result.returncode == 0, f"{field}: commit {sha} does not resolve")


def _git_output(repo_root: Path, arguments: Sequence[str], field: str) -> str:
    result = subprocess.run(
        ["git", "-C", str(repo_root), *arguments],
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="strict",
        check=False,
    )
    _require(
        result.returncode == 0,
        f"{field}: git command failed: {result.stderr.strip()}",
    )
    return result.stdout.strip()


def _validate_sealed_repository(
    repo_root: Path,
    evidence_dir: Path,
    implementation_sha: str | None,
    run_base_sha: str | None,
    *,
    allow_unsealed_repository: bool,
) -> dict[str, Any]:
    """Validate the post-push seal without impossible commit self-reference.

    The 31-field artifact records the clean implementation/run-base commit
    used by the live process.  The evidence-containing seal commit is the
    current repository HEAD and is reported by this external validator.
    """

    expected_evidence_dir = (repo_root / "hardware" / "v15_31b_ft").resolve()
    if evidence_dir != expected_evidence_dir:
        _require(
            allow_unsealed_repository,
            "production validation requires repo_root/hardware/v15_31b_ft and a sealed repository",
        )
        return {"sealed_repository_checked": False}
    _require(
        implementation_sha is not None and run_base_sha is not None,
        "sealed repository requires implementation and run-base commits",
    )
    branch = _git_output(repo_root, ["branch", "--show-current"], "branch")
    _require(branch == EXPECTED_BRANCH, "sealed repository branch mismatch")
    head = _git_output(repo_root, ["rev-parse", "HEAD"], "HEAD")
    _require(COMMIT_RE.fullmatch(head) is not None, "sealed HEAD is invalid")
    status = _git_output(
        repo_root,
        ["status", "--porcelain=v1", "--untracked-files=all"],
        "git status",
    )
    _require(status == "", "sealed repository worktree is not clean")
    upstream_name = _git_output(
        repo_root,
        ["rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{u}"],
        "upstream",
    )
    _require(
        upstream_name == f"origin/{EXPECTED_BRANCH}",
        "sealed repository upstream mismatch",
    )
    upstream_head = _git_output(
        repo_root, ["rev-parse", "@{u}"], "upstream HEAD"
    )
    _require(upstream_head == head, "sealed repository upstream is not HEAD")
    ahead_behind = _git_output(
        repo_root,
        ["rev-list", "--left-right", "--count", "HEAD...@{u}"],
        "ahead/behind",
    ).split()
    _require(
        ahead_behind == ["0", "0"],
        "sealed repository ahead/behind must be 0/0",
    )
    _require(
        implementation_sha == run_base_sha,
        "evidence commit must identify the live run-base implementation commit",
    )
    _require(
        _git_output(repo_root, ["rev-parse", f"{head}^"], "seal parent")
        == implementation_sha,
        "seal commit parent must be the implementation/run-base commit",
    )
    _require(
        _git_output(
            repo_root,
            ["rev-parse", f"{implementation_sha}^"],
            "implementation parent",
        )
        == SOURCE_COMMIT,
        "implementation commit must directly follow the frozen V15.31A source",
    )
    _require(
        _git_output(
            repo_root,
            ["show", "-s", "--format=%s", implementation_sha],
            "implementation subject",
        )
        == IMPLEMENTATION_COMMIT_SUBJECT,
        "implementation commit subject mismatch",
    )
    _require(
        _git_output(
            repo_root,
            ["show", "-s", "--format=%s", head],
            "seal subject",
        )
        == EVIDENCE_SEAL_COMMIT_SUBJECT,
        "evidence seal commit subject mismatch",
    )
    _require(
        _git_output(
            repo_root,
            ["rev-list", "--count", f"{SOURCE_COMMIT}..{head}"],
            "contract commit count",
        )
        == "2",
        "contract requires exactly two commits after the frozen source",
    )
    tracked = set(
        _git_output(
            repo_root,
            ["ls-tree", "-r", "--name-only", "HEAD", "--", "hardware/v15_31b_ft"],
            "sealed evidence tree",
        ).splitlines()
    )
    expected_tracked = {
        f"hardware/v15_31b_ft/{name}" for name in REQUIRED_ARTIFACTS
    }
    _require(
        tracked == expected_tracked,
        "seal commit must track exactly the 14 contracted evidence files",
    )
    seal_changes = set(
        _git_output(
            repo_root,
            ["diff-tree", "--no-commit-id", "--name-only", "-r", head],
            "seal commit changed paths",
        ).splitlines()
    )
    _require(
        seal_changes == expected_tracked,
        "seal commit may change only the exact 14 contracted evidence files",
    )
    origin_url = _git_output(
        repo_root, ["remote", "get-url", "origin"], "origin URL"
    )
    _require(
        origin_url
        in {
            "https://github.com/zhaowuc/go-m8010-robot-arm.git",
            "git@github.com:zhaowuc/go-m8010-robot-arm.git",
            "ssh://git@github.com/zhaowuc/go-m8010-robot-arm.git",
        },
        "origin does not identify the contracted GitHub repository",
    )
    remote_ref = f"refs/heads/{EXPECTED_BRANCH}"
    remote_parts = _git_output(
        repo_root,
        ["ls-remote", "--exit-code", "origin", remote_ref],
        "GitHub branch head",
    ).split()
    _require(
        remote_parts == [head, remote_ref],
        "GitHub branch does not resolve to the sealed HEAD",
    )
    return {
        "sealed_repository_checked": True,
        "sealed_evidence_commit": head,
        "upstream": upstream_name,
        "origin": origin_url,
        "ahead": 0,
        "behind": 0,
    }


def _field_number(value: Any, keys: Sequence[str], field: str) -> float:
    _require(isinstance(value, Mapping), f"{field}: structured value required")
    for key in keys:
        if key in value:
            return _number(value[key], f"{field}.{key}")
    raise EvidenceValidationError(f"{field}: missing one of {list(keys)}")


def _yes_no(value: Any, field: str) -> str:
    if value is True:
        return "YES"
    if value is False:
        return "NO"
    _require(isinstance(value, str), f"{field}: YES/NO required")
    token = value.strip().upper()
    _require(token in {"YES", "NO"}, f"{field}: YES/NO required")
    return token


def _field_claims_pass(value: Any, field: str) -> bool:
    try:
        return _is_pass(value, field)
    except EvidenceValidationError:
        return False


def _prevent_fake_pass(field_value: Any, artifact_status: str, field: str) -> None:
    if artifact_status not in {"PASS", "FULL_PASS"}:
        _require(not _field_claims_pass(field_value, field), f"{field}: claims PASS while underlying evidence is {artifact_status}")


def _validate_run_git_provenance(value: Any, field: str) -> Mapping[str, Any]:
    _require(isinstance(value, Mapping), f"{field}: object required")
    expected_keys = {
        "schema", "repo_root", "output_directory", "branch",
        "run_base_commit", "upstream_ref", "upstream_commit",
        "ahead_count", "behind_count", "allowed_untracked_paths",
        "captured_at_utc", "evidence_commit_semantics",
    }
    _require(set(value) == expected_keys, f"{field}: exact provenance fields required")
    _require(
        value.get("schema") == "V15.31B-run-git-provenance-v1"
        and value.get("branch") == EXPECTED_BRANCH
        and value.get("upstream_ref") == f"origin/{EXPECTED_BRANCH}"
        and value.get("run_base_commit") == value.get("upstream_commit")
        and type(value.get("ahead_count")) is int
        and value.get("ahead_count") == 0
        and type(value.get("behind_count")) is int
        and value.get("behind_count") == 0
        and value.get("evidence_commit_semantics")
        == "EVIDENCE_RUN_BASE_COMMIT_NOT_SELF_REFERENTIAL_SEAL_COMMIT",
        f"{field}: branch/upstream/run-base provenance invalid",
    )
    _require(
        isinstance(value.get("run_base_commit"), str)
        and COMMIT_RE.fullmatch(str(value["run_base_commit"])) is not None,
        f"{field}.run_base_commit invalid",
    )
    _timestamp(value.get("captured_at_utc"), f"{field}.captured_at_utc")
    expected_untracked = sorted(
        f"hardware/v15_31b_ft/{name}"
        for name in (
            "power_on_readonly.json",
            "model_session_anchor_validation.json",
            "gravity_readonly_validation.json",
        )
    )
    _require(
        value.get("allowed_untracked_paths") == expected_untracked,
        f"{field}: allowed untracked paths are not the exact three supporting artifacts",
    )
    _require(
        isinstance(value.get("repo_root"), str)
        and isinstance(value.get("output_directory"), str)
        and Path(str(value["output_directory"])).name == "v15_31b_ft",
        f"{field}: repository/output paths invalid",
    )
    return value


def _validate_operator_confirmation_trace(
    binding: Mapping[str, Any],
    requirements: Sequence[tuple[int, float | None]],
) -> None:
    samples = binding.get("operator_confirmation_trace")
    _require(
        isinstance(samples, list) and samples,
        "final_result operator confirmation trace must not be empty",
    )
    document = {
        "schema": "V15.31B-operator-confirmation-trace-v1",
        "samples": samples,
    }
    _require_document_sha(
        document,
        binding.get("operator_confirmation_trace_sha256"),
        "final_result.operator_confirmation_trace_sha256",
    )
    expected_fields = {
        "schema", "source_instance_id", "sequence", "source_monotonic_ns",
        "envelope_id", "envelope_sha256", "session_id", "state_instance_id",
        "target_gravity_scale", "operator_stop_ready",
        "j2_j3_support_reliable", "clearance_confirmed", "no_person_contact",
        "observer_receipt_monotonic_ns",
    }
    last_by_source: dict[str, tuple[int, int]] = {}
    observer_receipts: list[int] = []
    normalized: list[tuple[int, int, float]] = []
    for index, sample in enumerate(samples):
        field = f"operator_confirmation_trace[{index}]"
        _require(
            isinstance(sample, Mapping) and set(sample) == expected_fields,
            f"{field}: exact confirmation fields required",
        )
        source_id = sample.get("source_instance_id")
        _require(
            sample.get("schema") == "go-m8010-empirical-stage-confirmation/1.0"
            and isinstance(source_id, str)
            and re.fullmatch(r"[0-9a-f]{32}", source_id) is not None,
            f"{field}: confirmation schema/source invalid",
        )
        sequence = _integer(sample.get("sequence"), f"{field}.sequence", minimum=1)
        source_ns = _integer(
            sample.get("source_monotonic_ns"), f"{field}.source_monotonic_ns", minimum=1
        )
        observer_ns = _integer(
            sample.get("observer_receipt_monotonic_ns"),
            f"{field}.observer_receipt_monotonic_ns",
            minimum=1,
        )
        _require(
            0 <= observer_ns - source_ns <= 30_000_000_000,
            f"{field}: confirmation was stale when observed",
        )
        previous = last_by_source.get(source_id)
        _require(
            previous is None
            or (sequence > previous[0] and source_ns > previous[1]),
            f"{field}: confirmation replay/order invalid",
        )
        last_by_source[source_id] = (sequence, source_ns)
        observer_receipts.append(observer_ns)
        target = _number(
            sample.get("target_gravity_scale"),
            f"{field}.target_gravity_scale",
        )
        _require(
            target in GRAVITY_LEVELS
            and sample.get("envelope_id") == binding.get("envelope_id")
            and sample.get("envelope_sha256") == binding.get("envelope_sha256")
            and sample.get("session_id") == binding.get("session_id")
            and sample.get("state_instance_id") == binding.get("state_instance_id")
            and all(
                sample.get(name) is True
                for name in (
                    "operator_stop_ready", "j2_j3_support_reliable",
                    "clearance_confirmed", "no_person_contact",
                )
            ),
            f"{field}: confirmation binding/physical facts invalid",
        )
        normalized.append((source_ns, observer_ns, target))
    _strictly_increasing(
        observer_receipts, "operator confirmation observer receipt time"
    )
    _require(requirements, "active evidence has no confirmation requirements")
    for observation_ns, target in requirements:
        _require(
            any(
                (target is None or event_target == target)
                and event_observer_ns <= observation_ns
                and 0 <= observation_ns - event_source_ns
                <= 30_000_000_000
                for event_source_ns, event_observer_ns, event_target in normalized
            ),
            "active evidence sample lacks a fresh matching physical/operator confirmation",
        )


def _validate_final_brake(
    value: Any,
    expected_binding: Mapping[str, Any],
    expected_center_deg: Mapping[str, Any],
    *,
    after_sequence: int,
    after_source_ns: int,
) -> list[tuple[int, float]]:
    _require(isinstance(value, Mapping), "final brake: object required")
    expected_keys = {
        "status", "go", "j6", "hardware_state_sha256",
        "j6_disabled_raw_sha256", "continuous_dwell_s",
        "hardware_source_dwell_s", "j6_raw_source_dwell_s",
        "receipt_dwell_s", "maximum_source_gap_ms", "session_center_rad",
        "maximum_position_error_from_session_center_deg",
        "paired_sample_count", "paired_trace", "paired_trace_schema",
        "paired_trace_sha256",
    }
    _require(set(value) == expected_keys, "final brake: exact paired-trace fields required")
    _require(
        _is_pass(value.get("status"), "final brake.status")
        and str(value.get("go", "")).upper() == "BRAKE"
        and str(value.get("j6", "")).upper() == "DISABLED",
        "final brake GO/J6 terminal state invalid",
    )
    center_rad = _finite_vector(
        value.get("session_center_rad"), 6, "final brake.session_center_rad"
    )
    expected_center = tuple(
        math.radians(_number(expected_center_deg.get(joint), f"center.{joint}"))
        for joint in JOINT_NAMES
    )
    _require(
        all(
            math.isclose(actual, expected, abs_tol=1.0e-9)
            for actual, expected in zip(center_rad, expected_center)
        ),
        "final brake session center differs from position initialization center",
    )
    trace = value.get("paired_trace")
    _require(isinstance(trace, list), "final brake paired_trace list required")
    count = _integer(
        value.get("paired_sample_count"), "final brake.paired_sample_count", minimum=6
    )
    _require(len(trace) == count, "final brake paired sample count mismatch")
    trace_document = {
        "schema": "V15.31B-final-brake-paired-trace-v1",
        "samples": trace,
    }
    _require(
        value.get("paired_trace_schema") == trace_document["schema"],
        "final brake paired trace schema mismatch",
    )
    _require_document_sha(
        trace_document, value.get("paired_trace_sha256"), "final brake.paired_trace_sha256"
    )
    sample_keys = {
        "hardware_state_sequence", "hardware_state_source_monotonic_ns",
        "hardware_state_sha256", "position_rad",
        "position_error_from_session_center_deg", "go_controller_modes",
        "j6_aggregated_controller_mode", "j6_raw_source_monotonic_ns",
        "j6_raw_source_instance_id", "j6_raw_sequence", "j6_raw_session_id",
        "j6_raw_state_instance_id", "j6_raw_sha256", "j6_raw_drive_state",
        "j6_raw_controller_mode", "pair_receipt_monotonic_ns",
    }
    hardware_sequences: list[int] = []
    hardware_sources: list[int] = []
    raw_sequences: list[int] = []
    raw_sources: list[int] = []
    receipts: list[int] = []
    raw_source_instances: set[str] = set()
    maximum_position_error = 0.0
    for index, sample in enumerate(trace):
        field = f"final brake.paired_trace[{index}]"
        _require(
            isinstance(sample, Mapping) and set(sample) == sample_keys,
            f"{field}: exact sample fields required",
        )
        hardware_sequences.append(
            _integer(sample.get("hardware_state_sequence"), f"{field}.hardware_sequence", minimum=1)
        )
        hardware_source = _integer(
            sample.get("hardware_state_source_monotonic_ns"),
            f"{field}.hardware_source",
            minimum=1,
        )
        raw_source = _integer(
            sample.get("j6_raw_source_monotonic_ns"), f"{field}.raw_source", minimum=1
        )
        receipt = _integer(
            sample.get("pair_receipt_monotonic_ns"), f"{field}.receipt", minimum=1
        )
        raw_sequence = _integer(
            sample.get("j6_raw_sequence"), f"{field}.raw_sequence", minimum=1
        )
        _require(
            abs(hardware_source - raw_source) <= TRACE_MAXIMUM_SAMPLE_GAP_NS
            and 0 <= receipt - max(hardware_source, raw_source)
            <= MAXIMUM_RECEIPT_AGE_NS,
            f"{field}: hardware/raw pairing is stale or noncausal",
        )
        hardware_sources.append(hardware_source)
        raw_sources.append(raw_source)
        receipts.append(receipt)
        raw_sequences.append(raw_sequence)
        for sha_field in ("hardware_state_sha256", "j6_raw_sha256"):
            _require(
                isinstance(sample.get(sha_field), str)
                and SHA256_RE.fullmatch(str(sample[sha_field])) is not None,
                f"{field}.{sha_field}: SHA-256 required",
            )
        modes = _exact_keys(
            sample.get("go_controller_modes"), GO_MOTOR_NAMES, f"{field}.go_controller_modes"
        )
        _require(
            all(str(modes[motor]).lower() == "brake" for motor in GO_MOTOR_NAMES)
            and str(sample.get("j6_aggregated_controller_mode", "")).lower()
            == "brake"
            and sample.get("j6_raw_drive_state") == 0
            and str(sample.get("j6_raw_controller_mode", "")).lower() == "brake",
            f"{field}: GO/J6 is not continuously BRAKE/DISABLED",
        )
        source_id = sample.get("j6_raw_source_instance_id")
        _require(
            isinstance(source_id, str)
            and re.fullmatch(r"[0-9a-f]{32}", source_id) is not None,
            f"{field}: J6 raw source identity invalid",
        )
        raw_source_instances.add(source_id)
        _require(
            sample.get("j6_raw_session_id") == expected_binding.get("session_id")
            and sample.get("j6_raw_state_instance_id")
            == expected_binding.get("state_instance_id"),
            f"{field}: J6 raw binding mismatch",
        )
        position = _finite_vector(sample.get("position_rad"), 6, f"{field}.position_rad")
        errors = _finite_vector(
            sample.get("position_error_from_session_center_deg"),
            6,
            f"{field}.position_error_from_session_center_deg",
        )
        recomputed = tuple(
            abs(math.degrees(actual - center))
            for actual, center in zip(position, center_rad)
        )
        _require(
            all(
                math.isclose(actual, expected, abs_tol=1.0e-6)
                for actual, expected in zip(errors, recomputed)
            )
            and max(recomputed) <= 0.5 + 1.0e-9,
            f"{field}: restored pose drift evidence invalid",
        )
        maximum_position_error = max(maximum_position_error, max(recomputed))
    for values, field in (
        (hardware_sequences, "hardware sequence"),
        (hardware_sources, "hardware source"),
        (raw_sequences, "raw sequence"),
        (raw_sources, "raw source"),
        (receipts, "receipt"),
    ):
        _strictly_increasing(values, f"final brake {field}")
    for values, field in (
        (hardware_sources, "hardware source"),
        (raw_sources, "raw source"),
        (receipts, "receipt"),
    ):
        _require(
            all(
                current - previous <= TRACE_MAXIMUM_SAMPLE_GAP_NS
                for previous, current in zip(values, values[1:])
            ),
            f"final brake {field} gap exceeds 100 ms",
        )
    _require(len(raw_source_instances) == 1, "final brake J6 raw source changed")
    _require(
        hardware_sequences[0] > after_sequence
        and hardware_sources[0] > after_source_ns,
        "final brake paired trace must follow RESTORE_INITIAL",
    )
    spans = {
        "hardware_source_dwell_s": (hardware_sources[-1] - hardware_sources[0]) / 1.0e9,
        "j6_raw_source_dwell_s": (raw_sources[-1] - raw_sources[0]) / 1.0e9,
        "receipt_dwell_s": (receipts[-1] - receipts[0]) / 1.0e9,
    }
    for field, expected in spans.items():
        _require(
            expected >= 0.5
            and math.isclose(
                _number(value.get(field), f"final brake.{field}"),
                expected,
                abs_tol=1.0e-9,
            ),
            f"final brake {field} disagrees with trace",
        )
    _require(
        math.isclose(
            _number(value.get("continuous_dwell_s"), "final brake.continuous_dwell_s"),
            min(spans.values()),
            abs_tol=1.0e-9,
        )
        and math.isclose(
            _number(value.get("maximum_source_gap_ms"), "final brake.maximum_source_gap_ms"),
            100.0,
            abs_tol=1.0e-12,
        )
        and math.isclose(
            _number(
                value.get("maximum_position_error_from_session_center_deg"),
                "final brake.maximum_position_error",
            ),
            maximum_position_error,
            abs_tol=1.0e-6,
        )
        and value.get("hardware_state_sha256") == trace[-1]["hardware_state_sha256"]
        and value.get("j6_disabled_raw_sha256") == trace[-1]["j6_raw_sha256"],
        "final brake summary/hash does not match its paired trace",
    )
    return [(receipt, 1.0) for receipt in receipts]


def _validate_final_result(
    document: Mapping[str, Any],
    repo_root: Path,
    statuses: Mapping[str, str],
    position_metrics: Mapping[str, Any],
    scale_metrics: Mapping[str, Any],
    multi_metrics: Mapping[str, Any],
    thermal_metrics: Mapping[str, Any],
    confirmation_requirements: Sequence[tuple[int, float | None]],
) -> dict[str, Any]:
    _require(
        document.get("schema") == "V15.31B-final-result-v1",
        "final_result schema mismatch",
    )
    document_result = _status_token(document.get("result"), "final_result.result")
    _require(
        document_result in {"FULL_PASS", "HARDWARE_COUNTERBALANCE_REQUIRED"},
        "final_result result must be a terminal contract outcome",
    )
    fields = _extract_final_fields(document)
    result_status = _status_token(fields["V15.31B RESULT"], "V15.31B RESULT")
    final_status = _status_token(fields["FINAL TASK RESULT"], "FINAL TASK RESULT")
    ready = fields["READY_FOR_NEXT_STAGE"]
    _require(type(ready) is bool, "READY_FOR_NEXT_STAGE must be boolean")
    full_pass = result_status in {"PASS", "FULL_PASS"} or final_status in {"PASS", "FULL_PASS"}
    _require((result_status in {"PASS", "FULL_PASS"}) == (final_status in {"PASS", "FULL_PASS"}), "V15.31B RESULT and FINAL TASK RESULT disagree")
    _require(ready is full_pass, "READY_FOR_NEXT_STAGE must be true exactly for full PASS")
    _require(
        document_result == result_status == final_status,
        "final_result document/result fields disagree",
    )
    binding = document.get("binding")
    _require(isinstance(binding, Mapping), "final_result.binding object required")
    _require(
        set(binding)
        == {
            "envelope_id", "envelope_sha256", "session_id",
            "state_instance_id", "anchor_sha256",
            "worker_supervisor_instance_id", "worker_supervisor_pid",
            "run_git_provenance", "preseal_git_provenance",
            "operator_confirmation_trace",
            "operator_confirmation_trace_sha256",
        }
        and _expected_binding_tuple(binding) == _expected_binding_tuple(scale_metrics),
        "final_result binding mismatch",
    )
    _require(
        isinstance(binding.get("worker_supervisor_instance_id"), str)
        and bool(str(binding["worker_supervisor_instance_id"]).strip())
        and type(binding.get("worker_supervisor_pid")) is int
        and int(binding["worker_supervisor_pid"]) > 1,
        "final_result worker supervisor identity invalid",
    )
    run_provenance = _validate_run_git_provenance(
        binding.get("run_git_provenance"), "final_result.run_git_provenance"
    )
    preseal_provenance = _validate_run_git_provenance(
        binding.get("preseal_git_provenance"),
        "final_result.preseal_git_provenance",
    )
    comparable_provenance_fields = set(run_provenance) - {"captured_at_utc"}
    _require(
        all(
            run_provenance[field] == preseal_provenance[field]
            for field in comparable_provenance_fields
        ),
        "startup and preseal Git provenance changed during the powered run",
    )
    _require(fields["branch"] == EXPECTED_BRANCH, "final_result branch mismatch")
    terminal_complete = result_status in {
        "FULL_PASS", "HARDWARE_COUNTERBALANCE_REQUIRED"
    }
    terminal_required_pass = (
        "power",
        "anchor",
        "gravity_readonly",
        "gravity_scale",
        "position",
        "j2_comparison",
        "multi_joint",
        "gui",
    )
    if terminal_complete:
        _require(
            all(
                statuses[name] in {"PASS", "FULL_PASS"}
                for name in terminal_required_pass
            ),
            "terminal result requires every non-thermal acceptance artifact to PASS",
        )
    implementation_sha = _extract_commit(fields["implementation commit"], "implementation commit", allow_unavailable=not terminal_complete)
    evidence_sha = _extract_commit(fields["evidence commit"], "evidence commit", allow_unavailable=not terminal_complete)
    if terminal_complete:
        _require(
            implementation_sha == evidence_sha == run_provenance["run_base_commit"],
            "implementation/evidence fields must equal the live run-base commit",
        )
    if implementation_sha:
        _commit_resolves(repo_root, implementation_sha, "implementation commit")
    if evidence_sha:
        _commit_resolves(repo_root, evidence_sha, "evidence commit")

    field_to_artifact = {
        "Power-on read-only": "power",
        "Anchor": "anchor",
        "Gravity calculation": "gravity_readonly",
        "Gravity powered result": "gravity_scale",
        "multi-joint": "multi_joint",
        "GUI powered result": "gui",
    }
    for field, artifact in field_to_artifact.items():
        _prevent_fake_pass(fields[field], statuses[artifact], field)
    for joint in JOINT_NAMES:
        _prevent_fake_pass(fields[f"{joint} error"], statuses["position"], f"{joint} error")
    _prevent_fake_pass(fields["J2 max sync"], statuses["position"], "J2 max sync")

    if statuses["position"] in {"PASS", "FULL_PASS"}:
        errors = position_metrics["maximum_error_deg"]
        for joint in JOINT_NAMES:
            value = fields[f"{joint} error"]
            _require(_field_claims_pass(value, f"{joint} error"), f"{joint} error must PASS")
            reported = _field_number(value, ("max_error_deg", "error_deg"), f"{joint} error")
            _require(math.isclose(reported, errors[joint], abs_tol=1.0e-6), f"{joint} error disagrees with position CSV")
        sync_value = fields["J2 max sync"]
        _require(_field_claims_pass(sync_value, "J2 max sync"), "J2 max sync must PASS")
        reported_sync = _field_number(sync_value, ("max_sync_error_deg", "max_sync_deg"), "J2 max sync")
        expected_sync = max(position_metrics["maximum_j2_sync_deg"], scale_metrics.get("maximum_j2_sync_deg", 0.0))
        _require(math.isclose(reported_sync, expected_sync, abs_tol=1.0e-6), "J2 max sync disagrees with evidence")
    if statuses["multi_joint"] in {"PASS", "FULL_PASS"}:
        _require(_field_claims_pass(fields["multi-joint"], "multi-joint"), "multi-joint field must PASS")
        _require(_field_claims_pass(fields["restore initial"], "restore initial"), "restore initial field must PASS")
        restore = multi_metrics.get("restore_initial", {})
        _require(isinstance(restore, Mapping) and _pass_like(restore.get("status", restore.get("result")), "restore_initial.status"), "restore initial underlying evidence must PASS")
        _require(
            fields["restore initial"] == restore,
            "final_result restore initial field disagrees with multi-joint evidence",
        )
    else:
        _prevent_fake_pass(fields["restore initial"], statuses["multi_joint"], "restore initial")

    if thermal_metrics.get("classification") in {"CONTROL_EXCESS_TORQUE_SOLVED", "FUNDAMENTAL_CONTINUOUS_LOAD_LIMIT"}:
        _require(
            (
                thermal_metrics["classification"]
                == "CONTROL_EXCESS_TORQUE_SOLVED"
                and full_pass
            )
            or (
                thermal_metrics["classification"]
                == "FUNDAMENTAL_CONTINUOUS_LOAD_LIMIT"
                and result_status == "HARDWARE_COUNTERBALANCE_REQUIRED"
            ),
            "final terminal outcome disagrees with thermal A/B classification",
        )
        _require(str(fields["thermal classification"]).strip().upper() == thermal_metrics["classification"], "thermal classification disagrees with summary")
        _require(_yes_no(fields["counterbalance required"], "counterbalance required") == thermal_metrics["counterbalance_required"], "counterbalance field disagrees with thermal summary")
        for motor in ("J2A", "J2B"):
            report = fields[motor]
            _require(isinstance(report, Mapping), f"{motor}: structured temperature result required")
            expected = thermal_metrics[motor]
            key_pairs = (
                ("start_temp_c", "start_temperature_c"),
                ("temp_5min_c", "temperature_5min_c"),
                ("temp_15min_c", "temperature_15min_c"),
                ("temp_30min_c", "temperature_30min_c"),
                ("final_slope_c_per_min", "final_slope_c_per_min"),
            )
            for report_key, summary_key in key_pairs:
                actual = report.get(report_key)
                expected_value = expected.get(summary_key)
                if isinstance(expected_value, Mapping):
                    _require(
                        actual == expected_value,
                        f"{motor}.{report_key} structured NOT_RUN value disagrees with thermal summary",
                    )
                else:
                    _require(
                        math.isclose(
                            _number(actual, f"{motor}.{report_key}"),
                            _number(
                                expected_value,
                                f"thermal_summary.{motor}.{summary_key}",
                            ),
                            abs_tol=1.0e-6,
                        ),
                        f"{motor}.{report_key} disagrees with thermal summary",
                    )
        torque = fields["J2 sustained rotor torque"]
        _require(isinstance(torque, Mapping), "J2 sustained rotor torque: object required")
        _require(math.isclose(_number(torque.get("before_gravity_nm"), "J2 torque.before"), thermal_metrics["before_torque_nm"], abs_tol=1.0e-6), "before-gravity torque disagrees with summary")
        _require(math.isclose(_number(torque.get("after_gravity_nm"), "J2 torque.after"), thermal_metrics["after_torque_nm"], abs_tol=1.0e-6), "after-gravity torque disagrees with summary")
        saturation = fields["saturation ratio"]
        _require(isinstance(saturation, Mapping), "saturation ratio: object required")
        _require(math.isclose(_number(saturation.get("before"), "saturation.before"), thermal_metrics["before_saturation"], abs_tol=1.0e-6), "before saturation disagrees with summary")
        _require(math.isclose(_number(saturation.get("after"), "saturation.after"), thermal_metrics["after_saturation"], abs_tol=1.0e-6), "after saturation disagrees with summary")
    else:
        _require(not full_pass, "full PASS cannot use undetermined thermal classification")

    _require(_yes_no(fields["motor zero modified"], "motor zero modified") == "NO", "motor zero must not be modified")
    _require(_yes_no(fields["MuJoCo modified"], "MuJoCo modified") == "NO", "MuJoCo production model must not be modified")
    _require(_yes_no(fields["git clean"], "git clean") == "YES", "git clean must be YES")
    blocker = fields["PRIMARY BLOCKER"]
    if terminal_complete:
        _require(
            _field_claims_pass(fields["final brake"], "final brake"),
            "terminal result requires final brake PASS",
        )
        brake_confirmation_requirements = _validate_final_brake(
            fields["final brake"],
            scale_metrics,
            position_metrics.get("initial_center_deg", {}),
            after_sequence=int(
                multi_metrics.get("restore_last_sequence", -1)
            ),
            after_source_ns=int(
                multi_metrics.get("restore_last_source_ns", -1)
            ),
        )
        _validate_operator_confirmation_trace(
            binding,
            [*confirmation_requirements, *brake_confirmation_requirements],
        )
    if full_pass:
        _require(isinstance(blocker, str) and blocker.strip().upper() in {"NONE", "无"}, "full PASS requires PRIMARY BLOCKER=NONE")
        required_pass = ("power", "anchor", "gravity_readonly", "gravity_scale", "position", "j2_comparison", "multi_joint", "thermal_5min", "thermal_15min", "thermal_30min", "gui")
        _require(all(statuses[name] in {"PASS", "FULL_PASS"} for name in required_pass), "full PASS has non-PASS underlying evidence")
        _require(thermal_metrics.get("classification") == "CONTROL_EXCESS_TORQUE_SOLVED", "READY=true requires stable classification A; hardware counterbalance remains a blocker")
    else:
        _require(isinstance(blocker, str) and blocker.strip() and blocker.strip().upper() not in {"NONE", "无"}, "non-PASS result requires one PRIMARY BLOCKER")
        _require(not _field_claims_pass(fields["final brake"], "final brake") or all(statuses[name] in {"PASS", "FULL_PASS"} for name in ("position", "multi_joint")), "final brake PASS lacks powered motion evidence")
    return {"full_pass": full_pass, "implementation_commit": implementation_sha, "evidence_commit": evidence_sha}


def validate_evidence(
    evidence_dir: Path,
    repo_root: Path,
    *,
    allow_unsealed_repository: bool = False,
) -> dict[str, Any]:
    """Validate one V15.31B evidence directory without hardware access."""

    evidence_dir = Path(evidence_dir).resolve()
    repo_root = Path(repo_root).resolve()
    _require(evidence_dir.is_dir(), f"evidence directory does not exist: {evidence_dir}")
    actual_entries = {entry.name for entry in evidence_dir.iterdir()}
    _require(
        actual_entries == set(REQUIRED_ARTIFACTS),
        "evidence directory must contain exactly the 14 contracted files",
    )
    _require(
        all(
            (evidence_dir / name).is_file()
            and not (evidence_dir / name).is_symlink()
            for name in REQUIRED_ARTIFACTS
        ),
        "all 14 evidence artifacts must be regular non-symlink files",
    )
    missing = [name for name in REQUIRED_ARTIFACTS if not (evidence_dir / name).is_file()]
    _require(not missing, f"missing required evidence files: {missing}")
    _validate_no_placeholders(evidence_dir)
    _validate_manifest(evidence_dir)

    power_document = _load_json(evidence_dir / "power_on_readonly.json")
    anchor_document = _load_json(evidence_dir / "model_session_anchor_validation.json")
    gravity_document = _load_json(evidence_dir / "gravity_readonly_validation.json")
    multi_document = _load_json(evidence_dir / "multi_joint_validation.json")
    thermal_document = _load_json(evidence_dir / "thermal_summary.json")
    gui_document = _load_json(evidence_dir / "gui_powered_validation.json")
    final_document = _load_json(evidence_dir / "final_result.json")

    power_status, power_metrics = _validate_power_on(power_document)
    power_file_sha256 = hashlib.sha256(
        (evidence_dir / "power_on_readonly.json").read_bytes()
    ).hexdigest()
    anchor_status, anchor_metrics = _validate_anchor(
        anchor_document, power_metrics, power_file_sha256
    )
    gravity_status, _gravity_metrics = _validate_gravity_readonly(gravity_document, power_metrics, anchor_metrics)
    scale_status, scale_metrics = _validate_gravity_scale(
        evidence_dir / "gravity_scale_validation.csv",
        power_metrics,
        anchor_metrics,
    )
    position_status, position_metrics = _validate_position(
        evidence_dir / "position_validation.csv", scale_metrics
    )
    comparison_status, comparison_metrics = _validate_j2_comparison(
        evidence_dir / "j2_control_comparison.csv", scale_metrics
    )
    multi_status, multi_metrics = _validate_multi_joint(
        multi_document,
        scale_metrics,
        position_metrics.get("initial_center_deg", {}),
    )
    _require(
        all(
            status in {"PASS", "FULL_PASS"}
            for status in (
                scale_status,
                position_status,
                comparison_status,
                multi_status,
            )
        ),
        "terminal acceptance requires gravity, comparison, position, and multi-joint PASS",
    )
    without_ff = comparison_metrics.get("WITHOUT_FF")
    with_ff = comparison_metrics.get("WITH_FF")
    _require(
        isinstance(without_ff, Mapping) and isinstance(with_ff, Mapping),
        "global powered sequence requires both J2 comparison conditions",
    )
    ordered_boundaries = (
        (
            without_ff,
            "last_hardware_state_sequence",
            "last_hardware_state_source_ns",
            scale_metrics,
            "first_hardware_state_sequence",
            "first_hardware_state_source_ns",
            "WITHOUT_FF must precede the powered gravity ladder",
        ),
        (
            scale_metrics,
            "last_hardware_state_sequence",
            "last_hardware_state_source_ns",
            with_ff,
            "first_hardware_state_sequence",
            "first_hardware_state_source_ns",
            "powered gravity ladder must precede WITH_FF",
        ),
        (
            with_ff,
            "last_hardware_state_sequence",
            "last_hardware_state_source_ns",
            position_metrics,
            "first_hardware_state_sequence",
            "first_hardware_state_source_ns",
            "WITH_FF must precede POSITION",
        ),
        (
            position_metrics,
            "last_hardware_state_sequence",
            "last_hardware_state_source_ns",
            multi_metrics,
            "multi_first_sequence",
            "multi_first_source_ns",
            "POSITION must precede MULTI_JOINT",
        ),
    )
    for (
        previous,
        previous_sequence_field,
        previous_source_field,
        current,
        current_sequence_field,
        current_source_field,
        message,
    ) in ordered_boundaries:
        _require(
            int(current[current_sequence_field])
            > int(previous[previous_sequence_field])
            and int(current[current_source_field])
            > int(previous[previous_source_field]),
            message,
        )
    position_duration_ns = _integer(
        position_metrics.get("trajectory_duration_total_ns"),
        "position trajectory duration total",
        minimum=1,
    )
    post_duration_ns = _integer(
        multi_metrics.get("trajectory_duration_total_ns"),
        "post-position trajectory duration total",
        minimum=1,
    )
    _require(
        position_duration_ns + post_duration_ns <= 600_000_000_000,
        "accepted POSITION trajectory duration exceeds the 600-second envelope budget",
    )
    thermal_5_status, thermal_5_metrics = _validate_thermal_csv(
        evidence_dir / "thermal_5min.csv", 5, scale_metrics
    )
    thermal_15_status, thermal_15_metrics = _validate_thermal_csv(
        evidence_dir / "thermal_15min.csv", 15, scale_metrics
    )
    thermal_30_status, thermal_30_metrics = _validate_thermal_csv(
        evidence_dir / "thermal_30min.csv", 30, scale_metrics
    )
    completed_thermal_metrics = [
        metrics
        for status, metrics in (
            (thermal_5_status, thermal_5_metrics),
            (thermal_15_status, thermal_15_metrics),
            (thermal_30_status, thermal_30_metrics),
        )
        if status in {"PASS", "FULL_PASS"}
    ]
    if completed_thermal_metrics:
        _require(
            completed_thermal_metrics[0]["first_hardware_state_sequence"]
            > multi_metrics.get("thermal_setup_last_sequence", -1)
            and completed_thermal_metrics[0]["first_hardware_state_source_ns"]
            > multi_metrics.get("thermal_setup_last_source_ns", -1),
            "thermal 5-minute evidence must follow THERMAL_SETUP",
        )
        for previous, current in zip(
            completed_thermal_metrics, completed_thermal_metrics[1:]
        ):
            _require(
                current["first_hardware_state_sequence"]
                > previous["last_hardware_state_sequence"]
                and current["first_hardware_state_source_ns"]
                > previous["last_hardware_state_source_ns"],
                "thermal 5/15/30 evidence is reordered",
            )
        _require(
            multi_metrics.get("restore_first_sequence", -1)
            > completed_thermal_metrics[-1]["last_hardware_state_sequence"]
            and multi_metrics.get("restore_first_source_ns", -1)
            > completed_thermal_metrics[-1]["last_hardware_state_source_ns"],
            "RESTORE_INITIAL must follow the last completed thermal stage",
        )
    stage_statuses = {"5min": thermal_5_status, "15min": thermal_15_status, "30min": thermal_30_status}
    if thermal_5_status not in {"PASS", "FULL_PASS"}:
        _require(thermal_15_status not in {"PASS", "FULL_PASS"} and thermal_30_status not in {"PASS", "FULL_PASS"}, "later thermal stage cannot PASS after 5-minute stage failed/blocked")
    if thermal_15_status not in {"PASS", "FULL_PASS"}:
        _require(thermal_30_status not in {"PASS", "FULL_PASS"}, "30-minute thermal stage cannot PASS after 15-minute stage failed/blocked")
    thermal_status, thermal_metrics = _validate_thermal_summary(
        thermal_document,
        stage_statuses,
        comparison_metrics,
        {
            "5min": thermal_5_metrics,
            "15min": thermal_15_metrics,
            "30min": thermal_30_metrics,
        },
        scale_metrics,
    )
    gui_status, _gui_metrics = _validate_gui(gui_document, scale_metrics)

    confirmation_requirements: list[tuple[int, float | None]] = []
    for metrics in (
        scale_metrics,
        position_metrics,
        comparison_metrics,
        multi_metrics,
        thermal_5_metrics,
        thermal_15_metrics,
        thermal_30_metrics,
    ):
        requirements = metrics.get("confirmation_requirements", [])
        _require(
            isinstance(requirements, list),
            "confirmation requirement collection invalid",
        )
        confirmation_requirements.extend(requirements)

    statuses = {
        "power": power_status,
        "anchor": anchor_status,
        "gravity_readonly": gravity_status,
        "gravity_scale": scale_status,
        "position": position_status,
        "j2_comparison": comparison_status,
        "multi_joint": multi_status,
        "thermal_5min": thermal_5_status,
        "thermal_15min": thermal_15_status,
        "thermal_30min": thermal_30_status,
        "thermal_summary": thermal_status,
        "gui": gui_status,
    }
    final_metrics = _validate_final_result(
        final_document,
        repo_root,
        statuses,
        position_metrics,
        scale_metrics,
        multi_metrics,
        thermal_metrics,
        confirmation_requirements,
    )
    seal_metrics = _validate_sealed_repository(
        repo_root,
        evidence_dir,
        final_metrics["implementation_commit"],
        final_metrics["evidence_commit"],
        allow_unsealed_repository=allow_unsealed_repository,
    )
    result = {
        "schema_version": "V15.31B-FT-evidence-validation-v1",
        "evidence_directory": str(evidence_dir),
        "required_file_count": 14,
        "sha256_verified": "13/13",
        "report_field_count": 31,
        "artifact_status": statuses,
        "comparison_metrics": comparison_metrics,
        "thermal_stage_metrics": {
            "5min": thermal_5_metrics,
            "15min": thermal_15_metrics,
            "30min": thermal_30_metrics,
        },
        "FINAL_TASK_RESULT": "PASS" if final_metrics["full_pass"] else "PARTIAL_PASS",
        "READY_FOR_NEXT_STAGE": final_metrics["full_pass"],
    }
    result.update(seal_metrics)
    return result


def _default_repo_root() -> Path:
    return Path(__file__).resolve().parents[1]


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "evidence_directory",
        nargs="?",
        type=Path,
        default=Path("hardware/v15_31b_ft"),
    )
    parser.add_argument("--repo-root", type=Path, default=_default_repo_root())
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        result = validate_evidence(args.evidence_directory, args.repo_root)
    except EvidenceValidationError as exc:
        print(json.dumps({
            "schema_version": "V15.31B-FT-evidence-validation-v1",
            "status": "FAIL",
            "reason": str(exc),
        }, ensure_ascii=False, sort_keys=True))
        return 2
    print(json.dumps(result, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

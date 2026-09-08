#!/usr/bin/env python3
"""Create a one-shot V15.31B empirical gravity-validation envelope.

This is an offline contract tool.  It validates hash-pinned PASS evidence from
the powered read-only, MODEL_SESSION_ANCHOR_V2, and gravity read-only phases,
plus the frozen model/configuration files.  It never opens ROS, UDP, serial,
CAN, or a motor device.

The resulting envelope is deliberately *not* a motor rating.  It can only be
consumed by a separately reviewed runtime gate for the bounded 0/25/50/75/100
percent gravity ladder encoded in the document.  Validation-only is the
default; ``--apply`` requires an explicit confirmation and publishes a new
file atomically without overwriting an existing file.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import stat
import tempfile
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence

import yaml


ENVELOPE_SCHEMA = "go-m8010-empirical-validation-envelope/1.0"
RESULT_SCHEMA = "go-m8010-empirical-validation-envelope-create-result/1.0"
POWER_ON_SCHEMAS = frozenset({
    "go-m8010-v15-31b-power-on-readonly/1.0",
    "V15.31B-power-on-readonly-v1",
})
ANCHOR_VALIDATION_SCHEMA = "V15.31B-anchor-validation-v1"
GRAVITY_READONLY_SCHEMA = "V15.31B-gravity-readonly-v1"
ANCHOR_SCHEMA = "go-m8010-gravity-model-anchor-v2/2.0"
APPLY_GATE = "V15_31B_CREATE_EMPIRICAL_VALIDATION_ENVELOPE=YES"

EXACT_ANCHOR_FIELDS = frozenset({
    "schema",
    "model_sha256",
    "session_id",
    "state_instance_id",
    "created_utc",
    "gear_ratio",
    "motor_direction_sign",
    "motor_raw_reference_rad",
    "motor_encoder_branch",
    "logical_joint_reference_rad",
    "model_absolute_joint_rad",
})
EXACT_ANCHOR_VALIDATION_FIELDS = frozenset({
    "schema",
    "result",
    "session_id",
    "state_instance_id",
    "source_power_on_readonly_sha256",
    "anchor_sha256",
    "runtime_anchor_sha256",
    "anchor_field_count",
    "anchor",
    "checks",
})
EXACT_ANCHOR_VALIDATION_CHECK_FIELDS = frozenset({
    "readonly_sample_match",
    "session_match",
    "state_instance_match",
    "model_hash_match",
    "motor_internal_zero_modified",
    "flash_written",
    "eeprom_written",
    "mujoco_model_modified",
    "motor_zero_modified",
    "rid_modified",
    "flash_or_eeprom_written",
})

PRODUCTION_MODEL_SHA256 = (
    "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
)
GRAVITY_CONFIG_SHA256 = (
    "307469b8384fd35547327ba1d5f80aa440e6b9663406ab7c9d9469bea263335d"
)
THERMAL_CONFIG_SHA256 = (
    "1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467"
)
GO_GEAR_RATIO = 6.329999923706055
MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
GO_MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5")
FROZEN_MOTOR_SIGNS = {
    "J1": +1,
    "J2A": -1,
    "J2B": +1,
    "J3": +1,
    "J4": -1,
    "J5": +1,
    "J6": -1,
}
# These are frozen software/model feedforward clamps, never continuous ratings.
SOFTWARE_GRAVITY_ROTOR_LIMIT_NM = {
    "J1": 0.20,
    "J2A": 1.75,
    "J2B": 1.75,
    "J3": 1.10,
    "J4": 0.40,
    "J5": 0.20,
    "J6": 0.0,
}
GRAVITY_LEVELS = (0.0, 0.25, 0.50, 0.75, 1.0)
MINIMUM_HOLD_SECONDS = 5.0
MAXIMUM_HOLD_SECONDS = 10.0
RAMP_SECONDS = 2.0
# Explicit, bounded allowance for 100 Hz stage-boundary scheduling and the
# per-stage operator-confirmation handoff.  It is part of the signed envelope,
# not a hidden runtime comparison tolerance.
SCHEDULER_TRANSITION_SLACK_SECONDS = 0.5
INTERSTAGE_CONFIRMATION_WINDOW_SECONDS = 30.0
INTERSTAGE_CONFIRMATION_COUNT = 4
MINIMUM_LIFETIME_SECONDS = 60
MAXIMUM_LIFETIME_SECONDS = 4200
DEFAULT_LIFETIME_SECONDS = 4200
MAXIMUM_INPUT_BYTES = 64 * 1024 * 1024
MAXIMUM_FEEDBACK_AGE_MS = 250.0
EMPIRICAL_ENTRY_TEMPERATURE_C = 55.0
HARD_THERMAL_STOP_C = 60.0
MAXIMUM_STAGE_TEMPERATURE_RISE_C = 2.0
MAXIMUM_POSITION_VALIDATION_SECONDS = 600.0
MAXIMUM_POSITION_SEGMENT_SECONDS = 15.0
POSITION_BUDGET_FIELD = (
    "maximum_cumulative_accepted_trajectory_seconds"
)
POSITION_VALIDATION_DISPLACEMENT_DEG = 5.0
POSITION_ENDPOINT_ERROR_DEG = 0.1
POSITION_PRECISION_CONTRACT_ID = "go-m8010-position-accuracy/0.1deg-v1"
POSITION_MINIMUM_ACTUAL_DISPLACEMENT_DEG = 5.0 - 2 * POSITION_ENDPOINT_ERROR_DEG
POSITION_ENDPOINT_DWELL_SECONDS = 0.5
J2_SYNC_WARNING_DEG = 0.25
J2_SYNC_HARD_DEG = 0.50


class EnvelopeCreationError(ValueError):
    """Input evidence cannot authorize the bounded empirical ladder."""


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise EnvelopeCreationError(message)


def _no_duplicate_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise EnvelopeCreationError(f"JSON contains duplicate key: {key}")
        result[key] = value
    return result


def _reject_json_constant(value: str) -> None:
    raise EnvelopeCreationError(f"JSON contains non-finite constant: {value}")


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def normalized_sha256(value: Any, label: str) -> str:
    _require(
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value),
        f"{label} must be a lowercase SHA-256",
    )
    return value


def nonempty_text(value: Any, label: str, maximum: int = 512) -> str:
    _require(
        isinstance(value, str) and bool(value) and len(value) <= maximum,
        f"{label} must be non-empty text",
    )
    return value


def finite_number(value: Any, label: str) -> float:
    _require(
        type(value) in {int, float} and math.isfinite(float(value)),
        f"{label} must be finite",
    )
    return float(value)


def finite_vector(value: Any, length: int, label: str) -> tuple[float, ...]:
    _require(
        isinstance(value, list) and len(value) == length,
        f"{label} must contain exactly {length} values",
    )
    return tuple(
        finite_number(item, f"{label}[{index}]")
        for index, item in enumerate(value)
    )


def exact_motor_mapping(value: Any, label: str) -> Mapping[str, Any]:
    _require(
        isinstance(value, Mapping) and set(value) == set(MOTOR_NAMES),
        f"{label} must contain all seven motors exactly",
    )
    return value


def status_pass(document: Mapping[str, Any], label: str) -> None:
    value = document.get("status", document.get("result"))
    _require(
        isinstance(value, str) and value.strip().upper() in {"PASS", "FULL_PASS"},
        f"{label} status is not PASS",
    )


def schema_value(document: Mapping[str, Any]) -> Any:
    return document.get("schema", document.get("schema_version"))


def read_json_source(
    path: Path, expected_sha256: str, label: str
) -> tuple[dict[str, Any], str]:
    absolute = Path(os.path.abspath(path))
    _require(not absolute.is_symlink(), f"{label} must not be a symbolic link")
    try:
        resolved = absolute.resolve(strict=True)
        _require(resolved.is_file(), f"{label} must be a regular file")
        data = resolved.read_bytes()
    except OSError as exc:
        raise EnvelopeCreationError(f"cannot read {label}: {exc}") from exc
    _require(0 < len(data) <= MAXIMUM_INPUT_BYTES, f"{label} size is invalid")
    digest = sha256_bytes(data)
    _require(
        digest == normalized_sha256(expected_sha256, f"expected {label}"),
        f"{label} SHA-256 does not match",
    )
    try:
        document = json.loads(
            data.decode("utf-8-sig"),
            object_pairs_hook=_no_duplicate_object,
            parse_constant=_reject_json_constant,
        )
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise EnvelopeCreationError(f"{label} is not valid UTF-8 JSON") from exc
    _require(isinstance(document, dict), f"{label} must be a JSON object")
    return document, digest


def read_frozen_file(path: Path, expected_sha256: str, label: str) -> bytes:
    absolute = Path(os.path.abspath(path))
    _require(not absolute.is_symlink(), f"{label} must not be a symbolic link")
    try:
        resolved = absolute.resolve(strict=True)
        _require(resolved.is_file(), f"{label} must be a regular file")
        data = resolved.read_bytes()
    except OSError as exc:
        raise EnvelopeCreationError(f"cannot read {label}: {exc}") from exc
    _require(bool(data), f"{label} is empty")
    _require(
        sha256_bytes(data) == expected_sha256,
        f"{label} differs from the frozen SHA-256",
    )
    return data


def _validate_safe_motor_summary(name: str, value: Any) -> None:
    _require(isinstance(value, Mapping), f"power-on {name} summary is invalid")
    _require(value.get("online") is True, f"power-on {name} is not online")
    age = finite_number(value.get("max_feedback_age_ms"), f"power-on {name} age")
    _require(0.0 <= age <= MAXIMUM_FEEDBACK_AGE_MS, f"power-on {name} feedback is stale")
    temperature = finite_number(
        value.get("max_temperature_c"), f"power-on {name} temperature"
    )
    _require(
        0.0 <= temperature < EMPIRICAL_ENTRY_TEMPERATURE_C,
        f"power-on {name} is not below the empirical entry temperature",
    )
    _require(
        type(value.get("max_abs_merror")) is int
        and value.get("max_abs_merror") == 0,
        f"power-on {name} merror is nonzero",
    )
    _require(
        type(value.get("communication_interruptions")) is int
        and value.get("communication_interruptions") == 0,
        f"power-on {name} communication was interrupted",
    )
    _require(
        value.get("abnormal_velocity_detected") is False,
        f"power-on {name} reports abnormal velocity",
    )
    modes = value.get("observed_modes")
    # The whole-arm state model intentionally normalizes the J6 drive's raw
    # DISABLED state to ``brake``.  Accept either spelling for J6 while still
    # rejecting every active controller mode.
    allowed = {"brake", "disabled"} if name == "J6" else {"brake"}
    _require(
        isinstance(modes, list)
        and bool(modes)
        and {str(item).lower() for item in modes} <= allowed,
        f"power-on {name} left BRAKE/DISABLED",
    )


def validate_power_on(
    document: Mapping[str, Any], expected_session: str, expected_instance: str
) -> None:
    _require(schema_value(document) in POWER_ON_SCHEMAS, "power-on schema mismatch")
    status_pass(document, "power-on read-only")
    _require(document.get("session_id") == expected_session, "power-on session mismatch")
    _require(
        document.get("state_instance_id") == expected_instance,
        "power-on state instance mismatch",
    )
    model_sha = document.get("model_sha256")
    if model_sha is not None:
        _require(model_sha == PRODUCTION_MODEL_SHA256, "power-on model SHA mismatch")

    summaries = document.get("per_motor")
    if summaries is not None:
        for name, summary in exact_motor_mapping(summaries, "power-on per_motor").items():
            _validate_safe_motor_summary(name, summary)
        duration = finite_number(document.get("duration_s"), "power-on duration_s")
        _require(duration >= 10.0, "power-on read-only duration is below 10 seconds")
        j2 = document.get("j2")
        _require(isinstance(j2, Mapping), "power-on J2 summary is missing")
        sync = finite_number(j2.get("max_abs_e_sync_deg"), "power-on J2 sync")
        _require(sync <= J2_SYNC_WARNING_DEG, "power-on J2 sync exceeds warning")
        safety = document.get("safety")
        _require(isinstance(safety, Mapping), "power-on safety summary is missing")
        for field in (
            "position_enabled",
            "motor_internal_zero_modified",
            "flash_written",
            "eeprom_written",
        ):
            _require(safety.get(field) is False, f"power-on safety gate failed: {field}")
        _require(
            safety.get("active_command_count") == 0,
            "power-on read-only phase sent an active command",
        )
        return

    samples = document.get("samples")
    _require(
        isinstance(samples, list) and len(samples) >= 2,
        "power-on hardware-state samples are missing",
    )
    first_ns: int | None = None
    previous_ns = 0
    for sample_index, sample in enumerate(samples):
        label = f"power-on samples[{sample_index}]"
        _require(isinstance(sample, Mapping), f"{label} is invalid")
        _require(sample.get("session_id") == expected_session, f"{label} session mismatch")
        _require(
            sample.get("state_instance_id") == expected_instance,
            f"{label} state instance mismatch",
        )
        source_ns = sample.get("source_monotonic_ns")
        _require(
            type(source_ns) is int and source_ns > previous_ns,
            f"{label} monotonic timestamp is invalid",
        )
        first_ns = source_ns if first_ns is None else first_ns
        previous_ns = source_ns
        _require(sample.get("j2_sync_fault") is False, f"{label} J2 sync fault")
        sync_rad = finite_number(sample.get("j2_e_sync_rad"), f"{label} J2 sync")
        _require(
            abs(math.degrees(sync_rad)) <= J2_SYNC_WARNING_DEG,
            f"{label} J2 sync exceeds warning",
        )
        modes = exact_motor_mapping(
            sample.get("controller_mode_by_motor"), f"{label} modes"
        )
        motors = exact_motor_mapping(sample.get("per_motor"), f"{label} per_motor")
        for name in MOTOR_NAMES:
            allowed_modes = (
                {"brake", "disabled"} if name == "J6" else {"brake"}
            )
            _require(
                modes[name] in allowed_modes,
                f"{label} {name} left BRAKE/DISABLED",
            )
            motor = motors[name]
            _require(isinstance(motor, Mapping), f"{label} {name} record is invalid")
            _require(
                motor.get("communication_ok") is True and motor.get("fresh") is True,
                f"{label} {name} communication is not fresh",
            )
            _require(motor.get("merror") == 0, f"{label} {name} merror is nonzero")
            age = finite_number(motor.get("age_ms"), f"{label} {name} age")
            _require(0.0 <= age <= MAXIMUM_FEEDBACK_AGE_MS, f"{label} {name} feedback is stale")
            temperature = finite_number(
                motor.get("temperature_c"), f"{label} {name} temperature"
            )
            _require(
                0.0 <= temperature < EMPIRICAL_ENTRY_TEMPERATURE_C,
                f"{label} {name} is not below the empirical entry temperature",
            )
    assert first_ns is not None
    _require(
        previous_ns - first_ns >= 10_000_000_000,
        "power-on read-only samples cover less than 10 seconds",
    )


def validate_anchor(
    document: Mapping[str, Any],
    expected_session: str,
    expected_instance: str,
    expected_power_on_sha256: str,
) -> str:
    _require(
        set(document) == EXACT_ANCHOR_VALIDATION_FIELDS,
        "anchor validation field set mismatch",
    )
    _require(
        document.get("schema") == ANCHOR_VALIDATION_SCHEMA,
        "anchor validation schema mismatch",
    )
    _require(
        document.get("result") == "PASS",
        "MODEL_SESSION_ANCHOR_V2 validation status is not PASS",
    )
    _require(
        document.get("session_id") == expected_session,
        "anchor validation session mismatch",
    )
    _require(
        document.get("state_instance_id") == expected_instance,
        "anchor validation state instance mismatch",
    )
    _require(
        document.get("source_power_on_readonly_sha256")
        == normalized_sha256(
            expected_power_on_sha256, "expected power-on read-only SHA-256"
        ),
        "anchor source power-on evidence SHA mismatch",
    )
    anchor = document.get("anchor")
    _require(isinstance(anchor, Mapping), "anchor validation has no anchor")
    _require(set(anchor) == EXACT_ANCHOR_FIELDS, "runtime anchor field set mismatch")
    _require(anchor.get("schema") == ANCHOR_SCHEMA, "runtime anchor schema mismatch")
    _require(anchor.get("model_sha256") == PRODUCTION_MODEL_SHA256, "anchor model SHA mismatch")
    _require(anchor.get("session_id") == expected_session, "anchor session mismatch")
    _require(
        anchor.get("state_instance_id") == expected_instance,
        "anchor state instance mismatch",
    )
    _require(
        math.isclose(
            finite_number(anchor.get("gear_ratio"), "anchor gear ratio"),
            GO_GEAR_RATIO,
            rel_tol=0.0,
            abs_tol=1.0e-12,
        ),
        "anchor gear ratio mismatch",
    )
    signs = exact_motor_mapping(anchor.get("motor_direction_sign"), "anchor signs")
    _require(dict(signs) == FROZEN_MOTOR_SIGNS, "anchor signs mismatch")
    raw_references = exact_motor_mapping(
        anchor.get("motor_raw_reference_rad"), "anchor raw references"
    )
    encoder_branches = exact_motor_mapping(
        anchor.get("motor_encoder_branch"), "anchor encoder branches"
    )
    for name in MOTOR_NAMES:
        finite_number(raw_references[name], f"anchor {name} raw reference")
        _require(
            type(encoder_branches[name]) is int,
            f"anchor {name} encoder branch must be an integer",
        )
    finite_vector(anchor.get("logical_joint_reference_rad"), 6, "anchor logical reference")
    finite_vector(anchor.get("model_absolute_joint_rad"), 6, "anchor model reference")
    created_utc = nonempty_text(anchor.get("created_utc"), "anchor created_utc")
    try:
        created_at = datetime.fromisoformat(created_utc.replace("Z", "+00:00"))
    except ValueError as exc:
        raise EnvelopeCreationError("anchor created_utc is invalid") from exc
    _require(
        created_at.tzinfo is not None and created_at.utcoffset() is not None,
        "anchor created_utc must include a timezone",
    )
    computed_anchor_sha = sha256_bytes(json_bytes(dict(anchor)))
    declared_anchor_sha = normalized_sha256(
        document.get("anchor_sha256"), "anchor SHA-256"
    )
    declared_runtime_sha = normalized_sha256(
        document.get("runtime_anchor_sha256"), "runtime anchor SHA-256"
    )
    _require(
        declared_anchor_sha == declared_runtime_sha == computed_anchor_sha,
        "anchor SHA-256 does not match the embedded runtime anchor",
    )
    _require(
        type(document.get("anchor_field_count")) is int
        and document.get("anchor_field_count") == len(EXACT_ANCHOR_FIELDS),
        "anchor field count mismatch",
    )
    checks = document.get("checks")
    _require(isinstance(checks, Mapping), "anchor validation checks are missing")
    _require(
        set(checks) == EXACT_ANCHOR_VALIDATION_CHECK_FIELDS,
        "anchor validation check field set mismatch",
    )
    for field in (
        "readonly_sample_match",
        "session_match",
        "state_instance_match",
        "model_hash_match",
    ):
        _require(checks.get(field) is True, f"anchor validation check failed: {field}")
    for field in (
        "motor_internal_zero_modified",
        "flash_written",
        "eeprom_written",
        "mujoco_model_modified",
        "motor_zero_modified",
        "rid_modified",
        "flash_or_eeprom_written",
    ):
        _require(checks.get(field) is False, f"anchor prohibited modification: {field}")
    return computed_anchor_sha


def validate_gravity_readonly(
    document: Mapping[str, Any],
    expected_session: str,
    expected_instance: str,
    expected_anchor_sha256: str,
) -> dict[str, float]:
    _require(
        schema_value(document) == GRAVITY_READONLY_SCHEMA,
        "gravity read-only schema mismatch",
    )
    status_pass(document, "gravity read-only validation")
    _require(document.get("session_id") == expected_session, "gravity read-only session mismatch")
    _require(
        document.get("state_instance_id") == expected_instance,
        "gravity read-only state instance mismatch",
    )
    _require(
        document.get("anchor_sha256") == expected_anchor_sha256,
        "gravity read-only anchor SHA mismatch",
    )
    _require(document.get("hardware_tff_enabled") is False, "gravity read-only enabled hardware Tff")
    _require(document.get("tff_transmitted") is False, "gravity read-only transmitted Tff")
    checks = document.get("checks")
    _require(isinstance(checks, Mapping), "gravity read-only checks are missing")
    for field in (
        "finite",
        "continuous",
        "direction_reasonable",
        "direction_independent_of_motion",
        "j2_split_50_50",
        "j2a_sign_correct",
        "j2b_sign_correct",
    ):
        _require(checks.get(field) is True, f"gravity read-only check failed: {field}")
    samples = document.get("samples")
    _require(
        isinstance(samples, list) and len(samples) >= 2,
        "gravity read-only requires at least two samples",
    )
    maximum = {name: 0.0 for name in MOTOR_NAMES}
    for sample_index, sample in enumerate(samples):
        label = f"gravity read-only samples[{sample_index}]"
        _require(isinstance(sample, Mapping), f"{label} is invalid")
        finite_vector(sample.get("q_actual_rad"), 6, f"{label} q_actual")
        finite_vector(sample.get("model_q_rad"), 6, f"{label} model_q")
        gravity = finite_vector(sample.get("gravity_joint_nm"), 6, f"{label} gravity")
        predicted = sample.get("predicted_rotor_nm")
        _require(
            isinstance(predicted, Mapping) and set(predicted) == set(GO_MOTOR_NAMES),
            f"{label} predicted rotor map mismatch",
        )
        expected = {
            "J1": gravity[0] / GO_GEAR_RATIO,
            "J2A": -gravity[1] / (2.0 * GO_GEAR_RATIO),
            "J2B": +gravity[1] / (2.0 * GO_GEAR_RATIO),
            "J3": gravity[2] / GO_GEAR_RATIO,
            "J4": -gravity[3] / GO_GEAR_RATIO,
            "J5": gravity[4] / GO_GEAR_RATIO,
        }
        for name in GO_MOTOR_NAMES:
            value = finite_number(predicted[name], f"{label} {name} prediction")
            _require(
                math.isclose(value, expected[name], rel_tol=1.0e-8, abs_tol=1.0e-8),
                f"{label} {name} prediction violates frozen torque mapping",
            )
            magnitude = abs(value)
            _require(
                magnitude <= SOFTWARE_GRAVITY_ROTOR_LIMIT_NM[name] + 1.0e-12,
                f"{label} {name} gravity demand exceeds the software hard limit",
            )
            maximum[name] = max(maximum[name], magnitude)
    return maximum


def validate_frozen_authority(
    model_path: Path, gravity_config_path: Path, thermal_config_path: Path
) -> dict[str, Any]:
    read_frozen_file(model_path, PRODUCTION_MODEL_SHA256, "production model")
    gravity_bytes = read_frozen_file(
        gravity_config_path, GRAVITY_CONFIG_SHA256, "gravity configuration"
    )
    thermal_bytes = read_frozen_file(
        thermal_config_path, THERMAL_CONFIG_SHA256, "thermal configuration"
    )
    try:
        gravity = yaml.safe_load(gravity_bytes.decode("utf-8"))
        thermal = yaml.safe_load(thermal_bytes.decode("utf-8"))
    except (UnicodeError, yaml.YAMLError) as exc:
        raise EnvelopeCreationError("frozen configuration is not valid UTF-8 YAML") from exc
    _require(isinstance(gravity, Mapping), "gravity configuration is invalid")
    _require(
        gravity.get("schema") == "go-m8010-gravity-control/1.0"
        and gravity.get("production_model_sha256") == PRODUCTION_MODEL_SHA256
        and gravity.get("enabled_for_hardware") is False
        and gravity.get("continuous_rotor_limits_authoritative") is False
        and gravity.get("gravity_scale_levels") == [0.25, 0.50, 0.75, 1.00]
        and finite_number(gravity.get("gravity_scale_ramp_seconds"), "gravity ramp")
        == RAMP_SECONDS,
        "gravity configuration no longer matches the fail-closed empirical bootstrap",
    )
    _require(isinstance(thermal, Mapping), "thermal configuration is invalid")
    _require(
        thermal.get("schema") == "go-m8010-thermal-limits/1.0",
        "thermal configuration schema mismatch",
    )
    normal = finite_number(thermal.get("normal_below_c"), "thermal normal threshold")
    warning = finite_number(thermal.get("warning_below_c"), "thermal warning threshold")
    derating = finite_number(thermal.get("derating_start_c"), "thermal derating threshold")
    stop = finite_number(thermal.get("thermal_stop_c"), "thermal stop threshold")
    rearm = finite_number(thermal.get("rearm_below_c"), "thermal rearm threshold")
    _require(
        normal < warning < derating == EMPIRICAL_ENTRY_TEMPERATURE_C < stop
        and rearm == derating
        and stop == HARD_THERMAL_STOP_C,
        "thermal thresholds differ from the frozen project policy",
    )
    _require(
        thermal.get("continuous_rotor_torque_limit_nm") is None
        and thermal.get("short_peak_rotor_torque_limit_nm") is None,
        "frozen thermal configuration unexpectedly claims a torque rating",
    )
    authority = nonempty_text(
        thermal.get("threshold_authority"), "thermal threshold authority"
    )
    _require(
        "NOT_VENDOR_CONTINUOUS_RATING" in authority,
        "thermal threshold authority does not disclaim a vendor continuous rating",
    )
    return {
        "normal_below_c": normal,
        "warning_below_c": warning,
        "derating_start_c": derating,
        "thermal_stop_c": stop,
        "rearm_below_c": rearm,
        "threshold_authority": authority,
    }


def _utc_text(value: datetime) -> str:
    return value.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")


def build_envelope(
    *,
    session_id: str,
    state_instance_id: str,
    anchor_sha256: str,
    evidence_hashes: Mapping[str, str],
    maximum_predicted_rotor_nm: Mapping[str, float],
    thermal_policy: Mapping[str, Any],
    hold_seconds: float,
    lifetime_seconds: int,
    created_at: datetime,
    assisted_teach: bool = False,
) -> dict[str, Any]:
    _require(type(assisted_teach) is bool, "assisted_teach must be boolean")
    session = nonempty_text(session_id, "session_id")
    instance = nonempty_text(state_instance_id, "state_instance_id")
    normalized_sha256(anchor_sha256, "anchor SHA-256")
    _require(
        type(lifetime_seconds) is int
        and MINIMUM_LIFETIME_SECONDS <= lifetime_seconds <= MAXIMUM_LIFETIME_SECONDS,
        f"lifetime_seconds must be {MINIMUM_LIFETIME_SECONDS}-{MAXIMUM_LIFETIME_SECONDS}",
    )
    hold = finite_number(hold_seconds, "hold_seconds")
    _require(
        MINIMUM_HOLD_SECONDS <= hold <= MAXIMUM_HOLD_SECONDS,
        "hold_seconds must be 5-10 seconds",
    )
    _require(
        created_at.tzinfo is not None and created_at.utcoffset() is not None,
        "created_at must be timezone-aware",
    )
    created = created_at.astimezone(timezone.utc)
    expires = created + timedelta(seconds=lifetime_seconds)
    source_evidence = {
        name: {"sha256": normalized_sha256(digest, f"{name} SHA-256")}
        for name, digest in evidence_hashes.items()
    }
    maxima = {
        name: finite_number(maximum_predicted_rotor_nm[name], f"{name} maximum")
        for name in MOTOR_NAMES
    }
    stages = []
    for index, level in enumerate(GRAVITY_LEVELS):
        stages.append({
            "index": index,
            "gravity_scale": level,
            "ramp_from_previous_seconds": 0.0 if index == 0 else RAMP_SECONDS,
            "hold_seconds": hold,
            "entry_requires_previous_stage_pass": index > 0,
            "operator_stop_reconfirmation_required": index > 0,
            "support_reconfirmation_required": index > 0,
        })
    envelope: dict[str, Any] = {
        "schema": ENVELOPE_SCHEMA,
        "authority_class": "EMPIRICAL_VALIDATION_ENVELOPE",
        "rating_classification": "NOT_OFFICIAL_CONTINUOUS_RATING",
        "purpose": "V15.31B_STAGED_POWERED_GRAVITY_VALIDATION_ONLY",
        "single_use": True,
        "binding": {
            "session_id": session,
            "state_instance_id": instance,
            "anchor_sha256": anchor_sha256,
            "model_sha256": PRODUCTION_MODEL_SHA256,
            "gravity_config_sha256": GRAVITY_CONFIG_SHA256,
            "thermal_config_sha256": THERMAL_CONFIG_SHA256,
            "all_fields_must_match_runtime": True,
        },
        "session_id": session,
        "state_instance_id": instance,
        "anchor_sha256": anchor_sha256,
        "created_at_utc": _utc_text(created),
        "expires_at_utc": _utc_text(expires),
        "lifetime_seconds": lifetime_seconds,
        "source_evidence": source_evidence,
        "frozen_authority": {
            "model_sha256": PRODUCTION_MODEL_SHA256,
            "gravity_config_sha256": GRAVITY_CONFIG_SHA256,
            "thermal_config_sha256": THERMAL_CONFIG_SHA256,
            "gravity_config_enabled_for_hardware": False,
            "continuous_rotor_limits_authoritative": False,
            "official_continuous_rotor_rating_available": False,
            "runtime_consumer_must_validate_envelope": True,
        },
        "torque_basis": {
            "source": "MODEL_DERIVED_GRAVITY_ONLY",
            "software_hard_limit_rotor_nm": dict(SOFTWARE_GRAVITY_ROTOR_LIMIT_NM),
            "observed_max_abs_predicted_rotor_nm": maxima,
            "maximum_output_used_as_continuous_rating": False,
            "peak_output_used_as_continuous_rating": False,
            "historical_feedback_used_as_continuous_rating": False,
            "historical_software_guard_used_as_continuous_rating": False,
            "any_peak_or_history_claim_used_as_continuous_rating": False,
            "continuous_operation_authorized": False,
        },
        "staged_activation": {
            "strict_order": True,
            "levels": list(GRAVITY_LEVELS),
            "ramp_seconds": RAMP_SECONDS,
            "minimum_hold_seconds": MINIMUM_HOLD_SECONDS,
            "maximum_hold_seconds": MAXIMUM_HOLD_SECONDS,
            "configured_hold_seconds": hold,
            # The runtime has one whole-arm gravity scale, not a per-joint
            # feedforward mask.  This is therefore an observation/evidence
            # priority plus a physical-support gate; it must not claim that
            # every other joint was held at zero feedforward first.
            "observation_joint_priority": ["J2", "J3"],
            "j2_j3_observation_priority_required": True,
            "scheduler_transition_slack_seconds": (
                SCHEDULER_TRANSITION_SLACK_SECONDS
            ),
            "maximum_interstage_confirmation_seconds": (
                INTERSTAGE_CONFIRMATION_WINDOW_SECONDS
            ),
            "interstage_confirmation_count": (
                INTERSTAGE_CONFIRMATION_COUNT
            ),
            "maximum_total_active_seconds": (
                4.0 * RAMP_SECONDS
                + 5.0 * hold
                + INTERSTAGE_CONFIRMATION_COUNT
                * INTERSTAGE_CONFIRMATION_WINDOW_SECONDS
                + SCHEDULER_TRANSITION_SLACK_SECONDS
            ),
            "absolute_maximum_total_active_seconds": (
                4.0 * RAMP_SECONDS
                + 5.0 * MAXIMUM_HOLD_SECONDS
                + INTERSTAGE_CONFIRMATION_COUNT
                * INTERSTAGE_CONFIRMATION_WINDOW_SECONDS
                + SCHEDULER_TRANSITION_SLACK_SECONDS
            ),
            "stages": stages,
        },
        "position_validation": {
            "enabled": True,
            "unlock_requires_completed_gravity_ladder": True,
            "authority_class": "EMPIRICAL_VALIDATION_ENVELOPE",
            "rating_classification": "NOT_OFFICIAL_CONTINUOUS_RATING",
            "allowed_after_scale": 1.0,
            "single_joint_first_required": True,
            "maximum_moving_joints_before_single_joint_pass": 1,
            "precision_contract_id": POSITION_PRECISION_CONTRACT_ID,
            "nominal_command_displacement_deg": (
                POSITION_VALIDATION_DISPLACEMENT_DEG
            ),
            "minimum_actual_displacement_deg": POSITION_MINIMUM_ACTUAL_DISPLACEMENT_DEG,
            "maximum_abs_segment_displacement_deg": (
                POSITION_VALIDATION_DISPLACEMENT_DEG
            ),
            "maximum_segment_seconds": MAXIMUM_POSITION_SEGMENT_SECONDS,
            POSITION_BUDGET_FIELD: (
                MAXIMUM_POSITION_VALIDATION_SECONDS
            ),
            "wall_clock_hold_observation_counts_against_budget": False,
            "endpoint_error_limit_deg": POSITION_ENDPOINT_ERROR_DEG,
            "endpoint_dwell_seconds": POSITION_ENDPOINT_DWELL_SECONDS,
            "planned_path_gravity_basis": "MODEL_DERIVED_GRAVITY_ONLY",
            "planned_path_gravity_must_remain_within_software_hard_limits": True,
            "dynamic_inverse_load_used_as_continuous_rating": False,
            "pd_output_remains_bounded_by_worker_software_guards": True,
            "no_progress_watchdog_required_every_cycle": True,
            "temperature_required_every_cycle": True,
            "operator_stop_reconfirmation_maximum_age_seconds": 30.0,
            "continuous_operation_authorized": False,
        },
        "live_gates": {
            "feedback": {
                "all_seven_motors_online": True,
                "maximum_age_ms": MAXIMUM_FEEDBACK_AGE_MS,
                "communication_ok_required_every_frame": True,
                "abnormal_velocity_permitted": False,
            },
            "temperature": {
                "realtime_valid_required": True,
                "entry_below_c": EMPIRICAL_ENTRY_TEMPERATURE_C,
                "valid_must_remain_below_c": HARD_THERMAL_STOP_C,
                "derating_start_c": thermal_policy["derating_start_c"],
                "hard_stop_c": thermal_policy["thermal_stop_c"],
                "maximum_rise_within_one_stage_c": MAXIMUM_STAGE_TEMPERATURE_RISE_C,
                "rapid_rise_aborts_stage": True,
                "threshold_authority": thermal_policy["threshold_authority"],
            },
            "merror": {"required_value": 0, "required_every_frame": True},
            "j2_sync": {
                "realtime_valid_required": True,
                "warning_above_deg": J2_SYNC_WARNING_DEG,
                "warning_action": "ABORT_STAGE_AND_BRAKE_J2_DOMAIN",
                "hard_above_deg": J2_SYNC_HARD_DEG,
                "hard_action": "LATCH_BOTH_J2_MOTORS_BRAKE",
            },
            "no_progress": {
                "worker_watchdog_required": True,
                "authority": "SOFTWARE_GUARD_NOT_CONTINUOUS_RATING",
                "minimum_position_error_deg": 2.0,
                "minimum_improvement_deg": 0.25,
                "window_seconds": 3.0,
                "minimum_qualifying_frames": 100,
                "trip_action": "CANCEL_TRAJECTORY_AND_BRAKE_RELATED_DOMAIN",
            },
            "operator_stop": {
                "required_before_each_nonzero_stage": True,
                "confirmation_maximum_age_seconds": 30.0,
                "must_remain_available": True,
                "loss_action": "ABORT_STAGE_AND_BRAKE_RELATED_DOMAIN",
            },
            "physical_support": {
                "j2_j3_reliable_support_required": True,
                "reconfirmation_required_before_each_nonzero_stage": True,
            },
        },
        "failure_policy": {
            "stop_current_stage_only": True,
            "gravity_scale_target_after_failure": 0.0,
            "cancel_active_trajectory": True,
            "brake_related_domain": True,
            "keep_worker_and_telemetry_running": True,
            "invalidate_this_envelope": True,
            "automatic_retry_permitted": False,
        },
        "runtime_consumption": {
            "consumer_implementation_required": True,
            "file_sha256_pin_required": True,
            "session_and_state_instance_match_required": True,
            "anchor_sha256_match_required": True,
            "expiry_check_required": True,
            "strict_stage_sequence_required": True,
            "live_gate_check_required_every_cycle": True,
            "this_file_alone_enables_hardware": False,
        },
    }
    if assisted_teach:
        envelope["assisted_teach"] = {
            "schema": "go-m8010-assisted-teach-envelope/1.0",
            "enabled": True,
            "allowed_joints": ["J1", "J2", "J3", "J4", "J5"],
            "maximum_selected_joints": 1,
            "maximum_excursion_from_press_deg": 5.0,
            "maximum_press_seconds": 30.0,
            "maximum_velocity_deg_s": 5.0,
            "soft_limit_action": "capture_selected_hold",
            "stopping_hold_seconds": 1.0,
            "max_stopping_error_deg": 2.0,
            "unlock_requires_completed_gravity_ladder": True,
            "allowed_after_scale": 1.0,
            "nonselected_joints_fixed_hold_required": True,
            "j6_fixed_hold_required": True,
            "continuous_operation_authorized": False,
        }
        envelope["assisted_teach"]["final_confirmation_policy"] = (
            "ONCE_AFTER_LADDER_THEN_LIVE_GATES_FOR_MANUAL_SESSION"
        )
    identity_seed = json.dumps(
        envelope, ensure_ascii=True, allow_nan=False, sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    envelope["envelope_id"] = "v15-31b-empirical-" + sha256_bytes(identity_seed)[:20]
    return envelope


def json_bytes(document: Mapping[str, Any]) -> bytes:
    return (json.dumps(document, ensure_ascii=False, allow_nan=False, indent=2) + "\n").encode("utf-8")


def fsync_directory(path: Path) -> None:
    if os.name == "nt":
        return
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def atomic_write_new(path: Path, data: bytes) -> None:
    output = Path(os.path.abspath(path))
    _require(
        not output.exists() and not output.is_symlink(),
        f"refuse to overwrite existing envelope: {output}",
    )
    output.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    _require(
        output.parent.is_dir() and not output.parent.is_symlink(),
        "envelope output directory is unsafe",
    )
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.", suffix=".tmp", dir=output.parent
    )
    temporary = Path(temporary_name)
    published = False
    try:
        stream = os.fdopen(descriptor, "wb", closefd=True)
        descriptor = -1
        with stream:
            if hasattr(os, "fchmod"):
                os.fchmod(stream.fileno(), stat.S_IRUSR | stat.S_IWUSR)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        if not hasattr(os, "fchmod"):
            os.chmod(temporary, stat.S_IRUSR | stat.S_IWUSR)
        try:
            os.link(temporary, output)
        except FileExistsError as exc:
            raise EnvelopeCreationError(
                f"refuse to overwrite existing envelope: {output}"
            ) from exc
        published = True
        fsync_directory(output.parent)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        temporary.unlink(missing_ok=True)
    _require(
        published and output.read_bytes() == data,
        "envelope atomic publication verification failed",
    )


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--power-on-readonly", type=Path, required=True)
    parser.add_argument("--expected-power-on-sha256", required=True)
    parser.add_argument("--model-session-anchor-validation", type=Path, required=True)
    parser.add_argument("--expected-anchor-validation-sha256", required=True)
    parser.add_argument("--gravity-readonly-validation", type=Path, required=True)
    parser.add_argument("--expected-gravity-readonly-sha256", required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--gravity-config", type=Path, required=True)
    parser.add_argument("--thermal-config", type=Path, required=True)
    parser.add_argument("--expected-session-id", required=True)
    parser.add_argument("--expected-state-instance-id", required=True)
    parser.add_argument("--hold-seconds", type=float, default=MINIMUM_HOLD_SECONDS)
    parser.add_argument(
        "--assisted-teach", action="store_true",
        help="Explicitly permit one J1-J5 joint at a time after the full gravity ladder, at most 5 degrees and 30 seconds per press; J6 remains HOLD.",
    )
    parser.add_argument(
        "--lifetime-seconds", type=int, default=DEFAULT_LIFETIME_SECONDS
    )
    parser.add_argument("--output", type=Path, required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--validate-only", action="store_true")
    mode.add_argument("--apply", action="store_true")
    parser.add_argument("--confirm", default="")
    return parser.parse_args(argv)


def run(args: argparse.Namespace, *, now: datetime | None = None) -> dict[str, Any]:
    session = nonempty_text(args.expected_session_id, "expected session_id")
    instance = nonempty_text(
        args.expected_state_instance_id, "expected state_instance_id"
    )
    power, power_sha = read_json_source(
        args.power_on_readonly,
        args.expected_power_on_sha256,
        "power_on_readonly.json",
    )
    anchor, anchor_validation_sha = read_json_source(
        args.model_session_anchor_validation,
        args.expected_anchor_validation_sha256,
        "model_session_anchor_validation.json",
    )
    gravity, gravity_sha = read_json_source(
        args.gravity_readonly_validation,
        args.expected_gravity_readonly_sha256,
        "gravity_readonly_validation.json",
    )
    validate_power_on(power, session, instance)
    runtime_anchor_sha = validate_anchor(
        anchor,
        session,
        instance,
        power_sha,
    )
    maximum_predicted = validate_gravity_readonly(
        gravity, session, instance, runtime_anchor_sha
    )
    thermal_policy = validate_frozen_authority(
        args.model, args.gravity_config, args.thermal_config
    )
    created_at = datetime.now(timezone.utc) if now is None else now
    envelope = build_envelope(
        session_id=session,
        state_instance_id=instance,
        anchor_sha256=runtime_anchor_sha,
        evidence_hashes={
            "power_on_readonly.json": power_sha,
            "model_session_anchor_validation.json": anchor_validation_sha,
            "gravity_readonly_validation.json": gravity_sha,
        },
        maximum_predicted_rotor_nm=maximum_predicted,
        thermal_policy=thermal_policy,
        hold_seconds=args.hold_seconds,
        lifetime_seconds=args.lifetime_seconds,
        created_at=created_at,
        assisted_teach=getattr(args, "assisted_teach", False),
    )
    data = json_bytes(envelope)
    if args.apply:
        _require(
            args.confirm == APPLY_GATE,
            f"apply requires --confirm {APPLY_GATE}",
        )
        atomic_write_new(args.output, data)
    return {
        "schema": RESULT_SCHEMA,
        "status": "PASS",
        "mode": "APPLY" if args.apply else "VALIDATE_ONLY",
        "applied": bool(args.apply),
        "output_path": str(Path(os.path.abspath(args.output))),
        "envelope_sha256": sha256_bytes(data),
        "envelope": envelope,
        "hardware_accessed": False,
        "official_continuous_rating_claimed": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    try:
        result = run(parse_args(argv))
    except (EnvelopeCreationError, OSError, ValueError) as exc:
        print(json.dumps({
            "schema": RESULT_SCHEMA,
            "status": "BLOCKED",
            "reason": str(exc),
            "applied": False,
            "hardware_accessed": False,
            "official_continuous_rating_claimed": False,
        }, ensure_ascii=False, indent=2), flush=True)
        return 4
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

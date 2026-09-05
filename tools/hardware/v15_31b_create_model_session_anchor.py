#!/usr/bin/env python3
"""Create exact MODEL_SESSION_ANCHOR_V2 metadata from powered read-only evidence.

The tool is deliberately offline.  It reads one hash-pinned
``power_on_readonly.json`` document, validates at least ten seconds of
same-session BRAKE/DISABLED hardware-state samples, and derives the exact
eleven-field runtime anchor consumed by ``GravityModelAnchorV2`` together with
the independently retained ``V15.31B-anchor-validation-v1`` evidence artifact.
It has no serial, CAN, UDP, ROS, motor-zero, RID, Flash, or EEPROM code path.

Validation-only is the default.  Apply mode publishes both new JSON files with
atomic no-overwrite operations and requires an explicit confirmation gate.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import re
import stat
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Any, Mapping, Sequence


POWER_ON_SCHEMA = "go-m8010-v15-31b-power-on-readonly/1.0"
HARDWARE_STATE_SCHEMA = "go-m8010-hardware-state/1.1"
ANCHOR_SCHEMA = "go-m8010-gravity-model-anchor-v2/2.0"
ANCHOR_VALIDATION_SCHEMA = "V15.31B-anchor-validation-v1"
RESULT_SCHEMA = "go-m8010-model-session-anchor-create-result/1.0"
APPLY_GATE = "V15_31B_CREATE_MODEL_SESSION_ANCHOR=YES"
PRODUCTION_MODEL_SHA256 = (
    "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
)
GO_GEAR_RATIO = 6.329999923706055
MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
GO_MOTOR_NAMES = frozenset(MOTOR_NAMES[:-1])
JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
MOTOR_TO_JOINT_INDEX = {
    "J1": 0,
    "J2A": 1,
    "J2B": 1,
    "J3": 2,
    "J4": 3,
    "J5": 4,
    "J6": 5,
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
FROZEN_MOTOR_GEAR_RATIOS = {
    "J1": GO_GEAR_RATIO,
    "J2A": GO_GEAR_RATIO,
    "J2B": GO_GEAR_RATIO,
    "J3": GO_GEAR_RATIO,
    "J4": GO_GEAR_RATIO,
    "J5": GO_GEAR_RATIO,
    "J6": 1.0,
}
SAFE_MODE_BY_MOTOR = {
    "J1": "brake",
    "J2A": "brake",
    "J2B": "brake",
    "J3": "brake",
    "J4": "brake",
    "J5": "brake",
    "J6": "disabled",
}
ALLOWED_SAFE_MODES_BY_MOTOR = {
    name: ({"brake"} if name in GO_MOTOR_NAMES else {"brake", "disabled"})
    for name in MOTOR_NAMES
}
EXACT_ANCHOR_FIELDS = {
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
}
EXACT_VALIDATION_FIELDS = {
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
}
EXACT_VALIDATION_CHECK_FIELDS = {
    "readonly_sample_match",
    "session_match",
    "state_instance_match",
    "model_hash_match",
    "motor_internal_zero_modified",
    "flash_written",
    "eeprom_written",
    "mujoco_model_modified",
    # Live runner spellings.  Keep both forms in one immutable artifact so
    # every consumer verifies the same producer-owned facts.
    "motor_zero_modified",
    "rid_modified",
    "flash_or_eeprom_written",
}
MINIMUM_CAPTURE_NS = 10_000_000_000
MAXIMUM_FEEDBACK_AGE_MS = 250.0
MAXIMUM_LOGICAL_POSITION_SPAN_RAD = math.radians(0.20)
MAXIMUM_STATIONARY_VELOCITY_RAD_S = math.radians(0.25)
MAXIMUM_J2_SYNC_RAD = math.radians(0.25)
MAXIMUM_GO_RAW_DELTA_RAD = GO_GEAR_RATIO * math.radians(0.20)
MAXIMUM_J6_RAW_DELTA_RAD = math.radians(0.20)
MAXIMUM_SAMPLE_GAP_NS = 250_000_000
MAXIMUM_INPUT_BYTES = 64 * 1024 * 1024
SESSION_ID_RE = re.compile(
    r"^persistent:[0-9a-f]{16}:j2session:[0-9a-f]{16}:"
    r"goauxsession:[0-9a-f]{16}$"
)
MODEL_JOINT_LIMITS_RAD = (
    None,
    (-2.96705972839036, 2.96705972839036),
    (-2.96705972839036, 2.96705972839036),
    (-2.02458193231342, 2.77507351067098),
    (-1.232202451908, 2.63893782901543),
    (-math.pi, math.pi),
)


class AnchorCreationError(ValueError):
    """The source evidence cannot authorize a runtime model-session anchor."""


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def normalized_sha256(value: Any, label: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise AnchorCreationError(f"{label} must be a lowercase SHA-256")
    return value


def nonempty_text(value: Any, label: str, maximum: int = 512) -> str:
    if not isinstance(value, str) or not value or len(value) > maximum:
        raise AnchorCreationError(f"{label} must be non-empty text")
    return value


def state_instance_id(value: Any, label: str = "state_instance_id") -> str:
    text = nonempty_text(value, label, 32)
    if len(text) != 32 or any(character not in "0123456789abcdef" for character in text):
        raise AnchorCreationError(f"{label} must be 32 lowercase hexadecimal characters")
    return text


def utc_timestamp(value: Any, label: str) -> str:
    text = nonempty_text(value, label, 128)
    try:
        parsed = datetime.fromisoformat(text.replace("Z", "+00:00"))
    except ValueError as exc:
        raise AnchorCreationError(f"{label} is not an ISO-8601 timestamp") from exc
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise AnchorCreationError(f"{label} must include a timezone")
    return parsed.isoformat().replace("+00:00", "Z")


def finite_number(value: Any, label: str) -> float:
    if type(value) not in {int, float} or not math.isfinite(float(value)):
        raise AnchorCreationError(f"{label} must be finite")
    return float(value)


def finite_vector(value: Any, length: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise AnchorCreationError(f"{label} must contain exactly {length} values")
    return tuple(finite_number(item, f"{label}[{index}]") for index, item in enumerate(value))


def exact_motor_mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or set(value) != set(MOTOR_NAMES):
        raise AnchorCreationError(f"{label} must contain all seven motors exactly")
    return value


def integer(value: Any, label: str, minimum: int | None = None) -> int:
    if type(value) is not int or (minimum is not None and value < minimum):
        raise AnchorCreationError(f"{label} must be an integer")
    return value


def finite_summary(value: Any, label: str) -> Mapping[str, float] | float:
    if type(value) in {int, float}:
        return finite_number(value, label)
    if not isinstance(value, Mapping) or not value:
        raise AnchorCreationError(f"{label} must be a number or numeric summary")
    converted: dict[str, float] = {}
    for key, item in value.items():
        if not isinstance(key, str):
            raise AnchorCreationError(f"{label} summary keys must be text")
        if item is not None:
            converted[key] = finite_number(item, f"{label}.{key}")
    if not converted:
        raise AnchorCreationError(f"{label} numeric summary is empty")
    return converted


def summary_mean(value: Any, label: str) -> float:
    summary = finite_summary(value, label)
    if isinstance(summary, Mapping):
        if "mean" not in summary:
            raise AnchorCreationError(f"{label} summary must contain mean")
        return summary["mean"]
    return summary


def raw_reference_matches_observation(name: str, reference: float, observed: float) -> bool:
    delta = (
        observed - reference
        if name == "J6"
        else math.remainder(observed - reference, 2.0 * math.pi)
    )
    maximum = MAXIMUM_J6_RAW_DELTA_RAD if name == "J6" else MAXIMUM_GO_RAW_DELTA_RAD
    return abs(delta) <= maximum


def normalized_phase(value: float) -> float:
    phase = (value + math.pi) % (2.0 * math.pi) - math.pi
    return 0.0 if phase == -0.0 else phase


def validate_operator_and_write_guards(document: Mapping[str, Any]) -> None:
    confirmation = document.get("operator_confirmation")
    required_confirmation = {
        "mechanism_stationary": True,
        "model_pose_aligned": True,
        "support_reliable": True,
        "not_at_mechanical_limit": True,
    }
    if not isinstance(confirmation, Mapping) or any(
        confirmation.get(field) is not expected
        for field, expected in required_confirmation.items()
    ):
        raise AnchorCreationError("operator physical/model alignment confirmation is incomplete")
    utc_timestamp(confirmation.get("confirmed_at_utc"), "operator confirmed_at_utc")

    writes = document.get("writes")
    required_writes = {
        "motor_internal_zero_modified": False,
        "rid_written": False,
        "flash_or_eeprom_written": False,
    }
    if not isinstance(writes, Mapping) or any(
        writes.get(field) is not expected for field, expected in required_writes.items()
    ):
        raise AnchorCreationError("source evidence records a forbidden hardware write")


def validate_power_summary(
    document: Mapping[str, Any],
    *,
    raw_reference: Mapping[str, float],
    logical_reference: Sequence[float],
) -> tuple[int, float]:
    """Validate the committed V15.31B read-only summary beside raw samples."""

    result = document.get("result", document.get("status"))
    if result not in {"PASS", "FULL_PASS"}:
        raise AnchorCreationError("power-on read-only result is not PASS")
    if "result" in document and "status" in document and document["result"] != document["status"]:
        raise AnchorCreationError("power-on result/status disagree")
    duration_s = finite_number(document.get("duration_s"), "duration_s")
    if duration_s < MINIMUM_CAPTURE_NS / 1.0e9:
        raise AnchorCreationError("power-on read-only duration is less than 10 seconds")
    frame_count = integer(document.get("valid_frame_count"), "valid_frame_count", 2)

    motors = exact_motor_mapping(document.get("per_motor"), "per_motor")
    for name in MOTOR_NAMES:
        label = f"per_motor.{name}"
        record = motors[name]
        if not isinstance(record, Mapping):
            raise AnchorCreationError(f"{label} must be an object")
        if record.get("online") is not True:
            raise AnchorCreationError(f"{label} did not remain online")
        age_ms = finite_number(record.get("max_feedback_age_ms"), f"{label}.max_feedback_age_ms")
        if not 0.0 <= age_ms <= MAXIMUM_FEEDBACK_AGE_MS:
            raise AnchorCreationError(f"{label} feedback age exceeds the read-only bound")
        raw_mean = summary_mean(record.get("raw_position_rad"), f"{label}.raw_position_rad")
        if not raw_reference_matches_observation(name, raw_reference[name], raw_mean):
            raise AnchorCreationError(f"{label} raw summary does not match the anchor reference")
        logical_mean = summary_mean(
            record.get("logical_position_rad"), f"{label}.logical_position_rad"
        )
        logical_target = logical_reference[MOTOR_TO_JOINT_INDEX[name]]
        if abs(logical_mean - logical_target) > MAXIMUM_LOGICAL_POSITION_SPAN_RAD:
            raise AnchorCreationError(f"{label} logical summary does not match the anchor reference")
        maximum_velocity = finite_number(
            record.get("max_abs_velocity_rad_s"), f"{label}.max_abs_velocity_rad_s"
        )
        if maximum_velocity < 0.0:
            raise AnchorCreationError(f"{label} maximum raw velocity must be nonnegative")
        maximum_temperature = finite_number(
            record.get("max_temperature_c"), f"{label}.max_temperature_c"
        )
        if not -40.0 <= maximum_temperature < 60.0:
            raise AnchorCreationError(f"{label} temperature is outside read-only qualification")
        if record.get("abnormal_velocity_detected") is not False:
            raise AnchorCreationError(f"{label} reports abnormal velocity")
        if integer(record.get("max_abs_merror"), f"{label}.max_abs_merror", 0) != 0:
            raise AnchorCreationError(f"{label} merror is nonzero")
        if integer(
            record.get("communication_interruptions"),
            f"{label}.communication_interruptions",
            0,
        ) != 0:
            raise AnchorCreationError(f"{label} reports a communication interruption")
        observed_modes = record.get("observed_modes")
        if not isinstance(observed_modes, list) or not observed_modes:
            raise AnchorCreationError(f"{label}.observed_modes must be a non-empty list")
        normalized_modes = {
            nonempty_text(value, f"{label}.observed_modes").strip().lower()
            for value in observed_modes
        }
        if not normalized_modes <= ALLOWED_SAFE_MODES_BY_MOTOR[name]:
            raise AnchorCreationError(f"{label} observed an active mode")
        if name == "J6":
            for field in (
                "tau_cmd_rotor_nm",
                "tau_feedback_rotor_nm",
                "tau_joint_estimated_nm",
            ):
                if record.get(field) is not None:
                    raise AnchorCreationError(f"{label}.{field} must be null for POS_VEL")
        else:
            finite_summary(record.get("tau_feedback_rotor_nm"), f"{label}.tau_feedback_rotor_nm")
            finite_summary(record.get("tau_joint_estimated_nm"), f"{label}.tau_joint_estimated_nm")
            maximum_command = finite_number(
                record.get("max_abs_tau_cmd_rotor_nm"),
                f"{label}.max_abs_tau_cmd_rotor_nm",
            )
            if not 0.0 <= maximum_command <= 1.0e-12:
                raise AnchorCreationError(f"{label} contains a nonzero active torque command")

    j2 = document.get("j2")
    if not isinstance(j2, Mapping):
        raise AnchorCreationError("j2 read-only summary must be an object")
    if abs(
        summary_mean(j2.get("logical_position_rad"), "j2.logical_position_rad")
        - logical_reference[1]
    ) > MAXIMUM_LOGICAL_POSITION_SPAN_RAD:
        raise AnchorCreationError("j2 logical summary does not match the anchor reference")
    maximum_sync_deg = finite_number(j2.get("max_abs_e_sync_deg"), "j2.max_abs_e_sync_deg")
    if not 0.0 <= maximum_sync_deg <= 0.25:
        raise AnchorCreationError("j2 synchronization exceeds 0.25 degrees")
    finite_summary(
        j2.get("j2a_logical_torque_contribution_nm"),
        "j2.j2a_logical_torque_contribution_nm",
    )
    finite_summary(
        j2.get("j2b_logical_torque_contribution_nm"),
        "j2.j2b_logical_torque_contribution_nm",
    )

    safety = document.get("safety")
    if not isinstance(safety, Mapping):
        raise AnchorCreationError("safety read-only summary must be an object")
    for field in (
        "position_enabled",
        "motor_internal_zero_modified",
        "flash_written",
        "eeprom_written",
    ):
        if safety.get(field) is not False:
            raise AnchorCreationError(f"safety.{field} must be false")
    if integer(safety.get("active_command_count"), "safety.active_command_count", 0) != 0:
        raise AnchorCreationError("read-only evidence records an active command")
    return frame_count, duration_s


def validate_authority_mappings(
    document: Mapping[str, Any],
) -> tuple[dict[str, float], dict[str, int]]:
    signs = exact_motor_mapping(document.get("motor_direction_sign"), "motor_direction_sign")
    for name in MOTOR_NAMES:
        if type(signs[name]) is not int or signs[name] != FROZEN_MOTOR_SIGNS[name]:
            raise AnchorCreationError(f"direction sign differs from frozen authority for {name}")

    ratios = exact_motor_mapping(document.get("motor_gear_ratio"), "motor_gear_ratio")
    for name in MOTOR_NAMES:
        ratio = finite_number(ratios[name], f"motor_gear_ratio.{name}")
        if not math.isclose(
            ratio, FROZEN_MOTOR_GEAR_RATIOS[name], rel_tol=0.0, abs_tol=1.0e-12
        ):
            raise AnchorCreationError(f"gear ratio differs from frozen authority for {name}")

    raw_input = exact_motor_mapping(
        document.get("motor_raw_reference_rad"), "motor_raw_reference_rad"
    )
    branch_input = exact_motor_mapping(
        document.get("motor_encoder_branch"), "motor_encoder_branch"
    )
    raw_reference: dict[str, float] = {}
    branches: dict[str, int] = {}
    for name in MOTOR_NAMES:
        raw = finite_number(raw_input[name], f"motor_raw_reference_rad.{name}")
        if abs(raw) > (12.5 if name == "J6" else float(1 << 16)):
            raise AnchorCreationError(f"raw reference exceeds protocol range for {name}")
        branch = branch_input[name]
        if type(branch) is not int:
            raise AnchorCreationError(f"encoder branch for {name} must be an integer")
        if name == "J6":
            if branch != 0:
                raise AnchorCreationError("J6 is non-wrapped and must use encoder branch 0")
        else:
            derived = round((raw - normalized_phase(raw)) / (2.0 * math.pi))
            if branch != derived:
                raise AnchorCreationError(f"encoder branch is inconsistent with raw reference for {name}")
        raw_reference[name] = raw
        branches[name] = branch
    return raw_reference, branches


def validate_samples(
    document: Mapping[str, Any],
    *,
    session: str,
    instance: str,
    raw_reference: Mapping[str, float],
    logical_reference: Sequence[float],
    declared_frame_count: int,
    declared_duration_s: float,
) -> None:
    samples = document.get("samples")
    if not isinstance(samples, list) or len(samples) < 2:
        raise AnchorCreationError("power-on evidence requires at least two hardware-state samples")
    if len(samples) != declared_frame_count:
        raise AnchorCreationError("valid_frame_count does not match the retained sample count")
    previous_sequence = 0
    previous_source_ns = 0
    first_source_ns: int | None = None
    last_source_ns = 0
    for sample_index, sample in enumerate(samples):
        label = f"samples[{sample_index}]"
        if not isinstance(sample, Mapping):
            raise AnchorCreationError(f"{label} must be an object")
        if sample.get("schema") != HARDWARE_STATE_SCHEMA:
            raise AnchorCreationError(f"{label} hardware-state schema mismatch")
        if sample.get("session_id") != session or sample.get("state_instance_id") != instance:
            raise AnchorCreationError(f"{label} is not bound to the same session/state instance")
        sequence = sample.get("sequence")
        source_ns = sample.get("source_monotonic_ns")
        if type(sequence) is not int or sequence <= previous_sequence:
            raise AnchorCreationError(f"{label} sequence is replayed or invalid")
        if type(source_ns) is not int or source_ns <= previous_source_ns:
            raise AnchorCreationError(f"{label} monotonic timestamp is replayed or invalid")
        if sample_index and source_ns - previous_source_ns > MAXIMUM_SAMPLE_GAP_NS:
            raise AnchorCreationError(f"{label} breaks continuous read-only coverage")
        previous_sequence = sequence
        previous_source_ns = source_ns
        first_source_ns = source_ns if first_source_ns is None else first_source_ns
        last_source_ns = source_ns

        positions = finite_vector(sample.get("position_rad"), 6, f"{label}.position_rad")
        velocities = finite_vector(sample.get("velocity_rad_s"), 6, f"{label}.velocity_rad_s")
        if sample.get("velocity_source") != "POSITION_SPAN_0P40S":
            raise AnchorCreationError(
                f"{label} mechanical velocity observer is not position-span based"
            )
        for joint_index, joint_name in enumerate(JOINT_NAMES):
            if abs(positions[joint_index] - logical_reference[joint_index]) > MAXIMUM_LOGICAL_POSITION_SPAN_RAD:
                raise AnchorCreationError(f"{label} {joint_name} is not stationary at the logical reference")
            if abs(velocities[joint_index]) > MAXIMUM_STATIONARY_VELOCITY_RAD_S:
                raise AnchorCreationError(f"{label} {joint_name} velocity exceeds the stationary bound")

        if sample.get("j2_sync_fault") is not False:
            raise AnchorCreationError(f"{label} reports a J2 synchronization fault")
        if abs(finite_number(sample.get("j2_e_sync_rad"), f"{label}.j2_e_sync_rad")) > MAXIMUM_J2_SYNC_RAD:
            raise AnchorCreationError(f"{label} J2 synchronization error exceeds the warning bound")
        modes = exact_motor_mapping(
            sample.get("controller_mode_by_motor"), f"{label}.controller_mode_by_motor"
        )
        per_motor = exact_motor_mapping(sample.get("per_motor"), f"{label}.per_motor")
        for name in MOTOR_NAMES:
            mode = modes[name]
            if not isinstance(mode, str) or mode.strip().lower() not in ALLOWED_SAFE_MODES_BY_MOTOR[name]:
                raise AnchorCreationError(f"{label} {name} left BRAKE/DISABLED")
            record = per_motor[name]
            if not isinstance(record, Mapping):
                raise AnchorCreationError(f"{label} {name} record must be an object")
            if record.get("communication_ok") is not True or record.get("fresh") is not True:
                raise AnchorCreationError(f"{label} {name} communication is not fresh and valid")
            if type(record.get("merror")) is not int or record.get("merror") != 0:
                raise AnchorCreationError(f"{label} {name} merror is nonzero")
            age_ms = finite_number(record.get("age_ms"), f"{label}.{name}.age_ms")
            if not 0.0 <= age_ms <= MAXIMUM_FEEDBACK_AGE_MS:
                raise AnchorCreationError(f"{label} {name} feedback age is out of bounds")
            temperature = finite_number(
                record.get("temperature_c"), f"{label}.{name}.temperature_c"
            )
            if not -40.0 <= temperature < 60.0:
                raise AnchorCreationError(f"{label} {name} temperature is outside read-only qualification")
            raw = finite_number(record.get("raw_position_rad"), f"{label}.{name}.raw_position_rad")
            if not raw_reference_matches_observation(name, raw_reference[name], raw):
                raise AnchorCreationError(f"{label} {name} raw phase is not stationary at the reference")

    assert first_source_ns is not None
    observed_duration_ns = last_source_ns - first_source_ns
    if observed_duration_ns < MINIMUM_CAPTURE_NS:
        raise AnchorCreationError("power-on read-only capture covers less than 10 seconds")
    if not math.isclose(
        declared_duration_s,
        observed_duration_ns / 1.0e9,
        rel_tol=0.0,
        abs_tol=1.0e-6,
    ):
        raise AnchorCreationError("duration_s does not match retained sample timestamps")


def build_anchor(
    document: Mapping[str, Any],
    *,
    expected_session_id: str,
    expected_state_instance_id: str,
) -> dict[str, Any]:
    if (
        document.get("schema") != POWER_ON_SCHEMA
        or document.get("result", document.get("status")) not in {"PASS", "FULL_PASS"}
    ):
        raise AnchorCreationError("power-on read-only schema/status mismatch")
    if document.get("model_sha256") != PRODUCTION_MODEL_SHA256:
        raise AnchorCreationError("power-on evidence is not bound to the frozen model")
    session = nonempty_text(document.get("session_id"), "session_id")
    if SESSION_ID_RE.fullmatch(session) is None:
        raise AnchorCreationError("session_id is not a complete persistent J2/GO-AUX session")
    instance = state_instance_id(document.get("state_instance_id"))
    if session != nonempty_text(expected_session_id, "expected session_id"):
        raise AnchorCreationError("power-on evidence session_id does not match the caller pin")
    if instance != state_instance_id(expected_state_instance_id, "expected state_instance_id"):
        raise AnchorCreationError("power-on evidence state_instance_id does not match the caller pin")
    created_utc = utc_timestamp(document.get("recorded_at_utc"), "recorded_at_utc")
    validate_operator_and_write_guards(document)
    raw_reference, branches = validate_authority_mappings(document)
    logical_reference = finite_vector(
        document.get("logical_joint_reference_rad"), 6, "logical_joint_reference_rad"
    )
    model_absolute = finite_vector(
        document.get("model_absolute_joint_rad"), 6, "model_absolute_joint_rad"
    )
    for index, limits in enumerate(MODEL_JOINT_LIMITS_RAD):
        if limits is not None and not limits[0] <= model_absolute[index] <= limits[1]:
            raise AnchorCreationError(
                f"model_absolute_joint_rad[{index}] is outside the frozen model joint range"
            )
    frame_count, duration_s = validate_power_summary(
        document,
        raw_reference=raw_reference,
        logical_reference=logical_reference,
    )
    validate_samples(
        document,
        session=session,
        instance=instance,
        raw_reference=raw_reference,
        logical_reference=logical_reference,
        declared_frame_count=frame_count,
        declared_duration_s=duration_s,
    )
    anchor = {
        "schema": ANCHOR_SCHEMA,
        "model_sha256": PRODUCTION_MODEL_SHA256,
        "session_id": session,
        "state_instance_id": instance,
        "created_utc": created_utc,
        "gear_ratio": GO_GEAR_RATIO,
        "motor_direction_sign": dict(FROZEN_MOTOR_SIGNS),
        "motor_raw_reference_rad": raw_reference,
        "motor_encoder_branch": branches,
        "logical_joint_reference_rad": list(logical_reference),
        "model_absolute_joint_rad": list(model_absolute),
    }
    if set(anchor) != EXACT_ANCHOR_FIELDS:
        raise AssertionError("internal anchor field set differs from the runtime contract")
    return anchor


def json_bytes(document: Mapping[str, Any]) -> bytes:
    return (json.dumps(document, ensure_ascii=False, indent=2) + "\n").encode("utf-8")


def build_validation_document(
    anchor: Mapping[str, Any], *, source_power_on_readonly_sha256: str,
) -> dict[str, Any]:
    """Bind one validated source file to the exact runtime-anchor bytes."""

    if set(anchor) != EXACT_ANCHOR_FIELDS:
        raise AnchorCreationError("runtime anchor field set is invalid")
    source_sha256 = normalized_sha256(
        source_power_on_readonly_sha256, "source power-on evidence"
    )
    embedded_anchor = dict(anchor)
    anchor_sha256 = sha256_bytes(json_bytes(embedded_anchor))
    checks = {
        "readonly_sample_match": True,
        "session_match": True,
        "state_instance_match": True,
        "model_hash_match": True,
        "motor_internal_zero_modified": False,
        "flash_written": False,
        "eeprom_written": False,
        "mujoco_model_modified": False,
        "motor_zero_modified": False,
        "rid_modified": False,
        "flash_or_eeprom_written": False,
    }
    if set(checks) != EXACT_VALIDATION_CHECK_FIELDS:
        raise AssertionError("internal anchor validation check set mismatch")
    document = {
        "schema": ANCHOR_VALIDATION_SCHEMA,
        "result": "PASS",
        "session_id": embedded_anchor["session_id"],
        "state_instance_id": embedded_anchor["state_instance_id"],
        "source_power_on_readonly_sha256": source_sha256,
        "anchor_sha256": anchor_sha256,
        "runtime_anchor_sha256": anchor_sha256,
        "anchor_field_count": len(embedded_anchor),
        "anchor": embedded_anchor,
        "checks": checks,
    }
    if set(document) != EXACT_VALIDATION_FIELDS:
        raise AssertionError("internal anchor validation field set mismatch")
    return document


def read_source(path: Path, expected_sha256: str) -> tuple[dict[str, Any], str]:
    absolute = Path(os.path.abspath(path))
    if absolute.is_symlink():
        raise AnchorCreationError("power-on evidence must not be a symbolic link")
    resolved = absolute.resolve(strict=True)
    if not resolved.is_file():
        raise AnchorCreationError("power-on evidence is not a regular file")
    data = resolved.read_bytes()
    if not data or len(data) > MAXIMUM_INPUT_BYTES:
        raise AnchorCreationError("power-on evidence size is invalid")
    digest = sha256_bytes(data)
    if digest != normalized_sha256(expected_sha256, "expected power-on evidence"):
        raise AnchorCreationError("power-on evidence SHA-256 does not match")
    try:
        document = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise AnchorCreationError("power-on evidence is not valid UTF-8 JSON") from exc
    if not isinstance(document, dict):
        raise AnchorCreationError("power-on evidence must be a JSON object")
    return document, digest


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
    if output.exists() or output.is_symlink():
        raise AnchorCreationError(f"refuse to overwrite existing anchor: {output}")
    output.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    if output.parent.is_symlink() or not output.parent.is_dir():
        raise AnchorCreationError("anchor output directory is unsafe")
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
            raise AnchorCreationError(f"refuse to overwrite existing anchor: {output}") from exc
        published = True
        fsync_directory(output.parent)
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        temporary.unlink(missing_ok=True)
    if not published or output.read_bytes() != data:
        raise AnchorCreationError("anchor atomic publication verification failed")


def atomic_write_pair_new(
    runtime_anchor_path: Path,
    runtime_anchor_data: bytes,
    validation_path: Path,
    validation_data: bytes,
) -> None:
    """Atomically publish each file, with the runtime activation point last.

    Two arbitrary paths cannot become visible in one filesystem transaction.
    Publish the validation/evidence file first and the runtime-consumed anchor
    last: a crash can therefore leave validation-only evidence, never a newly
    active runtime anchor without its validation.  Both individual files use
    atomic no-overwrite publication.
    """

    runtime_output = Path(os.path.abspath(runtime_anchor_path))
    validation_output = Path(os.path.abspath(validation_path))
    if runtime_output == validation_output:
        raise AnchorCreationError(
            "runtime anchor and validation output paths must differ"
        )
    for output, label in (
        (runtime_output, "runtime anchor"),
        (validation_output, "anchor validation"),
    ):
        if output.exists() or output.is_symlink():
            raise AnchorCreationError(f"refuse to overwrite existing {label}: {output}")

    atomic_write_new(validation_output, validation_data)
    atomic_write_new(runtime_output, runtime_anchor_data)

    if (
        runtime_output.read_bytes() != runtime_anchor_data
        or validation_output.read_bytes() != validation_data
    ):
        raise AnchorCreationError("anchor publication pair verification failed")


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--power-on-readonly", type=Path, required=True)
    parser.add_argument("--expected-power-on-sha256", required=True)
    parser.add_argument("--expected-session-id", required=True)
    parser.add_argument("--expected-state-instance-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--validation-output", type=Path, required=True)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--validate-only", action="store_true")
    mode.add_argument("--apply", action="store_true")
    parser.add_argument("--confirm", default="")
    return parser.parse_args(argv)


def run(args: argparse.Namespace) -> dict[str, Any]:
    document, source_sha256 = read_source(
        args.power_on_readonly, args.expected_power_on_sha256
    )
    anchor = build_anchor(
        document,
        expected_session_id=args.expected_session_id,
        expected_state_instance_id=args.expected_state_instance_id,
    )
    data = json_bytes(anchor)
    validation = build_validation_document(
        anchor, source_power_on_readonly_sha256=source_sha256
    )
    validation_data = json_bytes(validation)
    if args.apply:
        if args.confirm != APPLY_GATE:
            raise AnchorCreationError(f"apply requires --confirm {APPLY_GATE}")
        atomic_write_pair_new(
            args.output,
            data,
            args.validation_output,
            validation_data,
        )
    anchor_sha256 = sha256_bytes(data)
    return {
        "schema": RESULT_SCHEMA,
        "status": "PASS",
        "mode": "APPLY" if args.apply else "VALIDATE_ONLY",
        "applied": bool(args.apply),
        "output_path": str(Path(os.path.abspath(args.output))),
        "validation_output_path": str(
            Path(os.path.abspath(args.validation_output))
        ),
        "source_power_on_readonly_sha256": source_sha256,
        "anchor_sha256": anchor_sha256,
        "runtime_anchor_sha256": anchor_sha256,
        "validation_schema": ANCHOR_VALIDATION_SCHEMA,
        "validation_sha256": sha256_bytes(validation_data),
        "anchor_field_count": len(anchor),
        "anchor": anchor,
        "checks": validation["checks"],
        "hardware_accessed": False,
        "motor_internal_zero_modified": False,
        "rid_written": False,
        "flash_or_eeprom_written": False,
    }


def main(argv: Sequence[str] | None = None) -> int:
    args: argparse.Namespace | None = None
    try:
        args = parse_args(argv)
        result = run(args)
    except (AnchorCreationError, OSError, ValueError) as exc:
        print(
            json.dumps(
                {
                    "schema": RESULT_SCHEMA,
                    "status": "BLOCKED",
                    "reason": str(exc),
                    "applied": False,
                    "hardware_accessed": False,
                    "motor_internal_zero_modified": False,
                    "rid_written": False,
                    "flash_or_eeprom_written": False,
                },
                ensure_ascii=False,
                indent=2,
            ),
            flush=True,
        )
        return 4
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

"""Strict runtime consumer for the V15.31B empirical validation envelope.

The envelope is a short-lived, single-process permit for the staged powered
gravity validation.  It is deliberately separate from continuous motor-rating
authority and cannot turn the frozen ``continuous_*_authoritative`` flags on.
"""

from __future__ import annotations

import hashlib
import json
import math
import os
import stat
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Mapping, Optional


ENVELOPE_SCHEMA = "go-m8010-empirical-validation-envelope/1.0"
CONFIRMATION_SCHEMA = "go-m8010-empirical-stage-confirmation/1.0"
HAND_GUIDANCE_CONFIRMATION_SCHEMA = "go-m8010-empirical-stage-confirmation/1.1"
AUTHORITY_CLASS = "EMPIRICAL_VALIDATION_ENVELOPE"
RATING_CLASSIFICATION = "NOT_OFFICIAL_CONTINUOUS_RATING"
PURPOSE = "V15.31B_STAGED_POWERED_GRAVITY_VALIDATION_ONLY"
PRODUCTION_MODEL_SHA256 = (
    "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
)
GRAVITY_CONFIG_SHA256 = (
    "307469b8384fd35547327ba1d5f80aa440e6b9663406ab7c9d9469bea263335d"
)
THERMAL_CONFIG_SHA256 = (
    "1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467"
)
MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
LEVELS = (0.0, 0.25, 0.50, 0.75, 1.0)
SOFTWARE_GRAVITY_ROTOR_LIMIT_NM = {
    "J1": 0.20,
    "J2A": 1.75,
    "J2B": 1.75,
    "J3": 1.10,
    "J4": 0.40,
    "J5": 0.20,
    "J6": 0.0,
}
MAXIMUM_FILE_BYTES = 4 * 1024 * 1024
MAXIMUM_CONFIRMATION_AGE_NS = 30_000_000_000
MAXIMUM_FEEDBACK_AGE_MS = 250.0
ENTRY_TEMPERATURE_C = 55.0
HARD_STOP_TEMPERATURE_C = 60.0
MAXIMUM_STAGE_TEMPERATURE_RISE_C = 2.0
J2_SYNC_WARNING_DEG = 0.25
J2_SYNC_HARD_DEG = 0.50
MAXIMUM_HOLD_VELOCITY_RAD_S = math.radians(5.0)
ASSISTED_TEACH_JOINTS = ("J1", "J2", "J3", "J4", "J5")
MAXIMUM_TEACH_EXCURSION_DEG = 5.0
MAXIMUM_TEACH_SECONDS = 30.0
MAXIMUM_TEACH_VELOCITY_DEG_S = 5.0
HAND_GUIDANCE_JOINTS = (*ASSISTED_TEACH_JOINTS, "J6")
MAXIMUM_HAND_GUIDANCE_EXCURSION_DEG = 20.0
MAXIMUM_HAND_GUIDANCE_SECONDS = 600.0
MAXIMUM_HAND_GUIDANCE_VELOCITY_DEG_S = 30.0
TEACH_STOPPING_HOLD_NS = 1_000_000_000
TEACH_STOPPING_ERROR_RAD = math.radians(2.0)
MAXIMUM_SCHEDULER_TRANSITION_SLACK_SECONDS = 1.0
POSITION_PRECISION_CONTRACT_ID = "go-m8010-position-accuracy/0.1deg-v1"
POSITION_ENDPOINT_ERROR_DEG = 0.1
POSITION_MINIMUM_ACTUAL_DISPLACEMENT_DEG = 5.0 - 2 * POSITION_ENDPOINT_ERROR_DEG


class EmpiricalEnvelopeError(ValueError):
    """The empirical permit is missing, stale, malformed, or out of scope."""


def select_runtime_torque_authority(
    *,
    official_continuous_authoritative: bool,
    empirical_authoritative: bool,
    empirical_envelope_configured: bool,
) -> str:
    """Choose exactly one authority class without changing either claim."""

    if official_continuous_authoritative and empirical_envelope_configured:
        return "BLOCKED_AMBIGUOUS_AUTHORITY"
    if official_continuous_authoritative:
        return "OFFICIAL_CONTINUOUS_RATING"
    if empirical_authoritative:
        return AUTHORITY_CLASS
    return "NONE"


def _require(condition: bool, message: str) -> None:
    if not condition:
        raise EmpiricalEnvelopeError(message)


def _object_no_duplicates(pairs: list[tuple[str, object]]) -> dict:
    value = {}
    for key, item in pairs:
        _require(key not in value, f"JSON_DUPLICATE_KEY:{key}")
        value[key] = item
    return value


def _reject_constant(value: str) -> None:
    raise EmpiricalEnvelopeError(f"JSON_NONFINITE:{value}")


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _valid_sha256(value: object) -> bool:
    return bool(
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _finite(value: object, label: str) -> float:
    _require(type(value) in {int, float}, f"{label}_TYPE")
    converted = float(value)
    _require(math.isfinite(converted), f"{label}_NONFINITE")
    return converted


def _utc(value: object, label: str) -> datetime:
    _require(isinstance(value, str) and value.endswith("Z"), f"{label}_FORMAT")
    try:
        parsed = datetime.fromisoformat(value[:-1] + "+00:00")
    except ValueError as exc:
        raise EmpiricalEnvelopeError(f"{label}_FORMAT") from exc
    _require(parsed.tzinfo is not None, f"{label}_TIMEZONE")
    return parsed.astimezone(timezone.utc)


def _exact_mapping(value: object, keys: set[str], label: str) -> Mapping:
    _require(isinstance(value, Mapping) and set(value) == keys, f"{label}_FIELDS")
    return value


def _read_pinned_json(path: Path, expected_sha256: str) -> tuple[dict, str]:
    _require(_valid_sha256(expected_sha256), "ENVELOPE_EXPECTED_SHA256_INVALID")
    absolute = Path(os.path.abspath(path))
    _require(not absolute.is_symlink(), "ENVELOPE_SYMLINK_FORBIDDEN")
    try:
        resolved = absolute.resolve(strict=True)
        _require(resolved.is_file(), "ENVELOPE_NOT_REGULAR_FILE")
        data = resolved.read_bytes()
    except OSError as exc:
        raise EmpiricalEnvelopeError(f"ENVELOPE_UNREADABLE:{exc}") from exc
    _require(0 < len(data) <= MAXIMUM_FILE_BYTES, "ENVELOPE_SIZE_INVALID")
    digest = _sha256(data)
    _require(digest == expected_sha256, "ENVELOPE_SHA256_MISMATCH")
    try:
        value = json.loads(
            data.decode("utf-8-sig"),
            object_pairs_hook=_object_no_duplicates,
            parse_constant=_reject_constant,
        )
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise EmpiricalEnvelopeError("ENVELOPE_JSON_INVALID") from exc
    _require(isinstance(value, dict), "ENVELOPE_ROOT_INVALID")
    return value, digest


@dataclass(frozen=True)
class EmpiricalValidationEnvelope:
    envelope_id: str
    sha256: str
    session_id: str
    state_instance_id: str
    anchor_sha256: str
    created_at_utc: datetime
    expires_at_utc: datetime
    lifetime_seconds: int
    configured_hold_seconds: float
    ramp_seconds: float
    maximum_total_active_seconds: float
    maximum_position_validation_seconds: float
    maximum_position_segment_seconds: float
    maximum_abs_position_segment_deg: float
    loaded_monotonic_ns: int
    monotonic_deadline_ns: int
    assisted_teach_enabled: bool = False
    hand_guidance_enabled: bool = False
    hand_guidance_excursion_deg: float = 10.0

    @classmethod
    def from_path(
        cls,
        path: Path,
        expected_sha256: str,
        *,
        now_utc: Optional[datetime] = None,
        now_monotonic_ns: Optional[int] = None,
    ) -> "EmpiricalValidationEnvelope":
        value, digest = _read_pinned_json(path, expected_sha256)
        now = datetime.now(timezone.utc) if now_utc is None else now_utc
        _require(now.tzinfo is not None, "RUNTIME_UTC_REQUIRED")
        now = now.astimezone(timezone.utc)
        loaded_monotonic_ns = (
            time.monotonic_ns()
            if now_monotonic_ns is None
            else now_monotonic_ns
        )
        _require(
            type(loaded_monotonic_ns) is int and loaded_monotonic_ns > 0,
            "RUNTIME_MONOTONIC_REQUIRED",
        )
        _require(value.get("schema") == ENVELOPE_SCHEMA, "ENVELOPE_SCHEMA_MISMATCH")
        _require(value.get("authority_class") == AUTHORITY_CLASS, "AUTHORITY_CLASS_INVALID")
        _require(
            value.get("rating_classification") == RATING_CLASSIFICATION,
            "RATING_CLASSIFICATION_INVALID",
        )
        _require(value.get("purpose") == PURPOSE, "ENVELOPE_PURPOSE_INVALID")
        hand_guidance_enabled = "hand_guidance" in value
        _require(not (hand_guidance_enabled and "assisted_teach" in value), "HAND_GUIDANCE_ASSISTED_TEACH_MUTUALLY_EXCLUSIVE")
        _require(value.get("single_use") is True, "ENVELOPE_MUST_BE_SINGLE_USE")
        envelope_id = value.get("envelope_id")
        _require(
            isinstance(envelope_id, str)
            and envelope_id.startswith("v15-31b-empirical-")
            and len(envelope_id) == 38,
            "ENVELOPE_ID_INVALID",
        )

        binding = value.get("binding")
        _require(isinstance(binding, Mapping), "ENVELOPE_BINDING_MISSING")
        session_id = binding.get("session_id")
        state_instance_id = binding.get("state_instance_id")
        anchor_sha256 = binding.get("anchor_sha256")
        _require(isinstance(session_id, str) and session_id, "SESSION_ID_INVALID")
        _require(
            isinstance(state_instance_id, str) and state_instance_id,
            "STATE_INSTANCE_ID_INVALID",
        )
        _require(_valid_sha256(anchor_sha256), "ANCHOR_SHA256_INVALID")
        _require(binding.get("model_sha256") == PRODUCTION_MODEL_SHA256, "MODEL_SHA256_MISMATCH")
        _require(binding.get("gravity_config_sha256") == GRAVITY_CONFIG_SHA256, "GRAVITY_CONFIG_SHA256_MISMATCH")
        _require(binding.get("thermal_config_sha256") == THERMAL_CONFIG_SHA256, "THERMAL_CONFIG_SHA256_MISMATCH")
        _require(binding.get("all_fields_must_match_runtime") is True, "RUNTIME_BINDING_NOT_REQUIRED")
        _require(value.get("session_id") == session_id, "SESSION_ID_DUPLICATE_MISMATCH")
        _require(value.get("state_instance_id") == state_instance_id, "STATE_INSTANCE_ID_DUPLICATE_MISMATCH")
        _require(value.get("anchor_sha256") == anchor_sha256, "ANCHOR_SHA256_DUPLICATE_MISMATCH")

        created = _utc(value.get("created_at_utc"), "CREATED_AT_UTC")
        expires = _utc(value.get("expires_at_utc"), "EXPIRES_AT_UTC")
        lifetime = value.get("lifetime_seconds")
        _require(type(lifetime) is int and 60 <= lifetime <= 4200, "LIFETIME_INVALID")
        _require(abs((expires - created).total_seconds() - lifetime) <= 1.0e-6, "LIFETIME_MISMATCH")
        _require(created <= now < expires, "ENVELOPE_EXPIRED_OR_NOT_YET_VALID")

        frozen = value.get("frozen_authority")
        _require(isinstance(frozen, Mapping), "FROZEN_AUTHORITY_MISSING")
        _require(frozen.get("model_sha256") == PRODUCTION_MODEL_SHA256, "FROZEN_MODEL_SHA256_MISMATCH")
        _require(frozen.get("gravity_config_sha256") == GRAVITY_CONFIG_SHA256, "FROZEN_GRAVITY_SHA256_MISMATCH")
        _require(frozen.get("thermal_config_sha256") == THERMAL_CONFIG_SHA256, "FROZEN_THERMAL_SHA256_MISMATCH")
        for field in (
            "gravity_config_enabled_for_hardware",
            "continuous_rotor_limits_authoritative",
            "official_continuous_rotor_rating_available",
        ):
            _require(frozen.get(field) is False, f"{field.upper()}_MUST_REMAIN_FALSE")
        _require(frozen.get("runtime_consumer_must_validate_envelope") is True, "RUNTIME_CONSUMER_NOT_REQUIRED")

        torque = value.get("torque_basis")
        _require(isinstance(torque, Mapping), "TORQUE_BASIS_MISSING")
        _require(torque.get("source") == "MODEL_DERIVED_GRAVITY_ONLY", "TORQUE_SOURCE_OUT_OF_SCOPE")
        limits = _exact_mapping(
            torque.get("software_hard_limit_rotor_nm"),
            set(MOTOR_NAMES),
            "SOFTWARE_LIMITS",
        )
        observed = _exact_mapping(
            torque.get("observed_max_abs_predicted_rotor_nm"),
            set(MOTOR_NAMES),
            "OBSERVED_GRAVITY",
        )
        for name in MOTOR_NAMES:
            _require(
                abs(_finite(limits[name], f"{name}_SOFTWARE_LIMIT") - SOFTWARE_GRAVITY_ROTOR_LIMIT_NM[name]) <= 1.0e-12,
                f"{name}_SOFTWARE_LIMIT_MISMATCH",
            )
            observed_value = _finite(observed[name], f"{name}_OBSERVED_GRAVITY")
            _require(0.0 <= observed_value <= SOFTWARE_GRAVITY_ROTOR_LIMIT_NM[name] + 1.0e-12, f"{name}_OBSERVED_GRAVITY_LIMIT")
        for field in (
            "maximum_output_used_as_continuous_rating",
            "peak_output_used_as_continuous_rating",
            "historical_feedback_used_as_continuous_rating",
            "historical_software_guard_used_as_continuous_rating",
            "any_peak_or_history_claim_used_as_continuous_rating",
            "continuous_operation_authorized",
        ):
            _require(torque.get(field) is False, f"{field.upper()}_MUST_BE_FALSE")

        staged = value.get("staged_activation")
        _require(isinstance(staged, Mapping), "STAGED_ACTIVATION_MISSING")
        _require(staged.get("strict_order") is True, "STRICT_STAGE_ORDER_REQUIRED")
        _require(tuple(staged.get("levels", ())) == LEVELS, "STAGE_LEVELS_INVALID")
        ramp = _finite(staged.get("ramp_seconds"), "RAMP_SECONDS")
        hold = _finite(staged.get("configured_hold_seconds"), "HOLD_SECONDS")
        _require(ramp == 2.0, "RAMP_SECONDS_MUST_BE_TWO")
        _require(5.0 <= hold <= 10.0, "HOLD_SECONDS_OUT_OF_RANGE")
        _require(
            staged.get("j2_j3_observation_priority_required") is True,
            "J2_J3_OBSERVATION_PRIORITY_REQUIRED",
        )
        _require(
            staged.get("observation_joint_priority") == ["J2", "J3"],
            "OBSERVATION_JOINT_PRIORITY_INVALID",
        )
        stages = staged.get("stages")
        _require(isinstance(stages, list) and len(stages) == len(LEVELS), "STAGE_RECORDS_INVALID")
        for index, (stage, level) in enumerate(zip(stages, LEVELS)):
            _require(isinstance(stage, Mapping), f"STAGE_{index}_INVALID")
            _require(stage.get("index") == index and stage.get("gravity_scale") == level, f"STAGE_{index}_IDENTITY_INVALID")
            _require(stage.get("ramp_from_previous_seconds") == (0.0 if index == 0 else ramp), f"STAGE_{index}_RAMP_INVALID")
            _require(stage.get("hold_seconds") == hold, f"STAGE_{index}_HOLD_INVALID")
            _require(stage.get("entry_requires_previous_stage_pass") is (index > 0), f"STAGE_{index}_PREVIOUS_GATE_INVALID")
            _require(stage.get("operator_stop_reconfirmation_required") is (index > 0), f"STAGE_{index}_STOP_GATE_INVALID")
            _require(stage.get("support_reconfirmation_required") is (index > 0 and not hand_guidance_enabled), f"STAGE_{index}_SUPPORT_GATE_INVALID")
            if hand_guidance_enabled:
                _require(stage.get("position_hold_reconfirmation_required") is (index > 0), f"STAGE_{index}_POSITION_HOLD_GATE_INVALID")
        maximum_total = _finite(
            staged.get("maximum_total_active_seconds"),
            "MAXIMUM_TOTAL_ACTIVE_SECONDS",
        )
        scheduler_slack = _finite(
            staged.get("scheduler_transition_slack_seconds"),
            "SCHEDULER_TRANSITION_SLACK_SECONDS",
        )
        _require(
            0.0 < scheduler_slack
            <= MAXIMUM_SCHEDULER_TRANSITION_SLACK_SECONDS,
            "SCHEDULER_TRANSITION_SLACK_SECONDS_INVALID",
        )
        confirmation_window = _finite(
            staged.get("maximum_interstage_confirmation_seconds"),
            "MAXIMUM_INTERSTAGE_CONFIRMATION_SECONDS",
        )
        confirmation_count = staged.get("interstage_confirmation_count")
        _require(
            confirmation_window == 30.0
            and confirmation_count == len(LEVELS) - 1,
            "INTERSTAGE_CONFIRMATION_BUDGET_INVALID",
        )
        _require(
            abs(
                maximum_total
                - (
                    4.0 * ramp
                    + 5.0 * hold
                    + confirmation_count * confirmation_window
                    + scheduler_slack
                )
            ) <= 1.0e-12,
            "MAXIMUM_TOTAL_ACTIVE_SECONDS_INVALID",
        )
        absolute_maximum = _finite(
            staged.get("absolute_maximum_total_active_seconds"),
            "ABSOLUTE_MAXIMUM_TOTAL_ACTIVE_SECONDS",
        )
        _require(
            absolute_maximum == (
                4.0 * ramp
                + 50.0
                + confirmation_count * confirmation_window
                + scheduler_slack
            )
            and absolute_maximum <= 179.0
            and maximum_total <= absolute_maximum,
            "ABSOLUTE_MAXIMUM_TOTAL_ACTIVE_SECONDS_INVALID",
        )

        position = value.get("position_validation")
        _require(isinstance(position, Mapping), "POSITION_VALIDATION_MISSING")
        _require(
            position.get("enabled") is True
            and position.get("unlock_requires_completed_gravity_ladder") is True
            and position.get("authority_class") == AUTHORITY_CLASS
            and position.get("rating_classification") == RATING_CLASSIFICATION
            and position.get("allowed_after_scale") == 1.0
            and position.get("single_joint_first_required") is True
            and position.get("maximum_moving_joints_before_single_joint_pass") == 1,
            "POSITION_VALIDATION_SCOPE_INVALID",
        )
        position_displacement = _finite(
            position.get("maximum_abs_segment_displacement_deg"),
            "POSITION_MAXIMUM_DISPLACEMENT_DEG",
        )
        position_segment_seconds = _finite(
            position.get("maximum_segment_seconds"),
            "POSITION_MAXIMUM_SEGMENT_SECONDS",
        )
        position_total_seconds = _finite(
            position.get("maximum_cumulative_accepted_trajectory_seconds"),
            "POSITION_MAXIMUM_CUMULATIVE_TRAJECTORY_SECONDS",
        )
        _require(
            position.get("precision_contract_id") == POSITION_PRECISION_CONTRACT_ID
            and position.get("nominal_command_displacement_deg") == 5.0
            and position.get("minimum_actual_displacement_deg") == POSITION_MINIMUM_ACTUAL_DISPLACEMENT_DEG
            and position_displacement == 5.0
            and position_segment_seconds == 15.0
            and 0.0 < position_total_seconds <= 600.0
            and position.get("endpoint_error_limit_deg") == POSITION_ENDPOINT_ERROR_DEG
            and position.get("endpoint_dwell_seconds") == 0.5,
            "POSITION_VALIDATION_BOUNDS_INVALID",
        )
        _require(
            position.get("wall_clock_hold_observation_counts_against_budget")
            is False,
            "POSITION_WALL_CLOCK_HOLD_MUST_NOT_SPEND_TRAJECTORY_BUDGET",
        )
        _require(
            position.get("planned_path_gravity_basis")
            == "MODEL_DERIVED_GRAVITY_ONLY"
            and position.get(
                "planned_path_gravity_must_remain_within_software_hard_limits"
            ) is True
            and position.get("dynamic_inverse_load_used_as_continuous_rating")
            is False
            and position.get("pd_output_remains_bounded_by_worker_software_guards")
            is True
            and position.get("no_progress_watchdog_required_every_cycle") is True
            and position.get("temperature_required_every_cycle") is True
            and position.get("operator_stop_reconfirmation_maximum_age_seconds")
            == 30.0
            and position.get("continuous_operation_authorized") is False,
            "POSITION_VALIDATION_AUTHORITY_INVALID",
        )

        assisted_teach_enabled = "assisted_teach" in value
        if assisted_teach_enabled:
            teach = _exact_mapping(value["assisted_teach"], {
                "schema", "enabled", "allowed_joints", "maximum_selected_joints",
                "maximum_excursion_from_press_deg", "maximum_press_seconds",
                "maximum_velocity_deg_s", "unlock_requires_completed_gravity_ladder",
                "allowed_after_scale", "nonselected_joints_fixed_hold_required",
                "j6_fixed_hold_required", "continuous_operation_authorized",
                "final_confirmation_policy",
                "soft_limit_action", "stopping_hold_seconds", "max_stopping_error_deg",
                "low_speed_stopping_error_action", "restricted_hold_rearm_error_deg",
                "restricted_hold_rearm_velocity_deg_s", "restricted_hold_minimum_stable_seconds",
            }, "ASSISTED_TEACH")
            _require(
                teach["schema"] == "go-m8010-assisted-teach-envelope/1.0"
                and teach["enabled"] is True
                and teach["allowed_joints"] == list(ASSISTED_TEACH_JOINTS)
                and type(teach["maximum_selected_joints"]) is int
                and teach["maximum_selected_joints"] == 1
                and teach["unlock_requires_completed_gravity_ladder"] is True
                and teach["nonselected_joints_fixed_hold_required"] is True
                and teach["j6_fixed_hold_required"] is True
                and teach["continuous_operation_authorized"] is False,
                "ASSISTED_TEACH_SCOPE_INVALID",
            )
            _require(teach["soft_limit_action"] == "capture_selected_hold", "ASSISTED_TEACH_SCOPE_INVALID")
            _require(teach["low_speed_stopping_error_action"] == "restricted_fixed_hold", "ASSISTED_TEACH_SCOPE_INVALID")
            _require(
                teach["final_confirmation_policy"] ==
                "ONCE_AFTER_LADDER_THEN_LIVE_GATES_FOR_MANUAL_SESSION",
                "ASSISTED_TEACH_SCOPE_INVALID",
            )
            for field, expected in (
                ("maximum_excursion_from_press_deg", MAXIMUM_TEACH_EXCURSION_DEG),
                ("maximum_press_seconds", MAXIMUM_TEACH_SECONDS),
                ("maximum_velocity_deg_s", MAXIMUM_TEACH_VELOCITY_DEG_S),
                ("allowed_after_scale", 1.0),
                ("stopping_hold_seconds", 1.0),
                ("max_stopping_error_deg", 2.0),
                ("restricted_hold_rearm_error_deg", 0.25),
                ("restricted_hold_rearm_velocity_deg_s", 0.25),
                ("restricted_hold_minimum_stable_seconds", 0.5),
            ):
                _require(_finite(teach[field], "ASSISTED_TEACH_" + field.upper()) == expected,
                         "ASSISTED_TEACH_BOUNDS_INVALID")

        if hand_guidance_enabled:
            guidance = _exact_mapping(value["hand_guidance"], {
                "schema", "enabled", "allowed_joints", "maximum_selected_joints",
                "maximum_excursion_from_press_deg", "maximum_press_seconds",
                "maximum_velocity_deg_s", "reference_lead_deg", "control_semantics",
                "unlock_requires_completed_gravity_ladder", "allowed_after_scale",
                "nonselected_joints_fixed_hold_required", "continuous_operation_authorized",
                "final_confirmation_policy", "normal_exit_action", "time_limit_action",
                "drive_release_requires",
            }, "HAND_GUIDANCE")
            _require(
                guidance["schema"] == "go-m8010-hand-guidance-envelope/1.0"
                and guidance["enabled"] is True
                and guidance["allowed_joints"] == list(HAND_GUIDANCE_JOINTS)
                and type(guidance["maximum_selected_joints"]) is int
                and guidance["maximum_selected_joints"] == 6
                and guidance["control_semantics"] == "POSITION_OUTER_ADMITTANCE"
                and guidance["unlock_requires_completed_gravity_ladder"] is True
                and guidance["nonselected_joints_fixed_hold_required"] is True
                and guidance["continuous_operation_authorized"] is False
                and guidance["final_confirmation_policy"] == "ONCE_AFTER_LADDER_THEN_LIVE_GATES_FOR_MANUAL_SESSION"
                and guidance["normal_exit_action"] == "KEEP_POSITION_HOLD"
                and guidance["time_limit_action"] == "KEEP_POSITION_HOLD"
                and guidance["drive_release_requires"] == "VERIFIED_VERTICAL_POSE",
                "HAND_GUIDANCE_SCOPE_INVALID",
            )
            _require(_finite(guidance["maximum_excursion_from_press_deg"], "HAND_GUIDANCE_EXCURSION") in (10.0, MAXIMUM_HAND_GUIDANCE_EXCURSION_DEG), "HAND_GUIDANCE_BOUNDS_INVALID")
            for field, expected in (
                ("maximum_press_seconds", MAXIMUM_HAND_GUIDANCE_SECONDS),
                ("maximum_velocity_deg_s", MAXIMUM_HAND_GUIDANCE_VELOCITY_DEG_S),
                ("reference_lead_deg", 2.0), ("allowed_after_scale", 1.0),
            ):
                _require(_finite(guidance[field], "HAND_GUIDANCE_" + field.upper()) == expected, "HAND_GUIDANCE_BOUNDS_INVALID")

        live = value.get("live_gates")
        _require(isinstance(live, Mapping), "LIVE_GATES_MISSING")
        feedback = live.get("feedback")
        temperature = live.get("temperature")
        sync = live.get("j2_sync")
        no_progress = live.get("no_progress")
        operator = live.get("operator_stop")
        support = live.get("physical_support")
        _require(isinstance(feedback, Mapping) and feedback.get("all_seven_motors_online") is True, "FEEDBACK_GATE_INVALID")
        _require(_finite(feedback.get("maximum_age_ms"), "MAXIMUM_AGE_MS") == MAXIMUM_FEEDBACK_AGE_MS, "MAXIMUM_AGE_MISMATCH")
        _require(feedback.get("communication_ok_required_every_frame") is True and feedback.get("abnormal_velocity_permitted") is False, "FEEDBACK_LIVE_GATE_INVALID")
        _require(isinstance(temperature, Mapping) and temperature.get("realtime_valid_required") is True, "TEMPERATURE_GATE_INVALID")
        _require(_finite(temperature.get("entry_below_c"), "ENTRY_TEMPERATURE") == ENTRY_TEMPERATURE_C, "ENTRY_TEMPERATURE_MISMATCH")
        _require(_finite(temperature.get("hard_stop_c"), "HARD_STOP_TEMPERATURE") == HARD_STOP_TEMPERATURE_C, "HARD_STOP_TEMPERATURE_MISMATCH")
        _require(_finite(temperature.get("maximum_rise_within_one_stage_c"), "MAXIMUM_STAGE_RISE") == MAXIMUM_STAGE_TEMPERATURE_RISE_C, "MAXIMUM_STAGE_RISE_MISMATCH")
        _require(isinstance(sync, Mapping) and _finite(sync.get("warning_above_deg"), "J2_SYNC_WARNING") == J2_SYNC_WARNING_DEG and _finite(sync.get("hard_above_deg"), "J2_SYNC_HARD") == J2_SYNC_HARD_DEG, "J2_SYNC_GATE_INVALID")
        _require(isinstance(no_progress, Mapping) and no_progress.get("worker_watchdog_required") is True, "NO_PROGRESS_GATE_INVALID")
        _require(isinstance(operator, Mapping) and operator.get("required_before_each_nonzero_stage") is True and _finite(operator.get("confirmation_maximum_age_seconds"), "CONFIRMATION_MAX_AGE") == 30.0 and operator.get("must_remain_available") is True, "OPERATOR_STOP_GATE_INVALID")
        if hand_guidance_enabled:
            support = _exact_mapping(support, {"base_fixed", "external_arm_support", "established_position_hold_required"}, "HAND_GUIDANCE_PHYSICAL_SUPPORT")
            _require(support["base_fixed"] is True and support["external_arm_support"] is False
                     and support["established_position_hold_required"] is True, "HAND_GUIDANCE_PHYSICAL_SUPPORT_INVALID")
        else:
            _require(isinstance(support, Mapping) and support.get("j2_j3_reliable_support_required") is True and support.get("reconfirmation_required_before_each_nonzero_stage") is True, "PHYSICAL_SUPPORT_GATE_INVALID")

        failure = value.get("failure_policy")
        runtime = value.get("runtime_consumption")
        _require(isinstance(failure, Mapping), "FAILURE_POLICY_MISSING")
        for field in (
            "stop_current_stage_only",
            "cancel_active_trajectory",
            "brake_related_domain",
            "keep_worker_and_telemetry_running",
            "invalidate_this_envelope",
        ):
            _require(failure.get(field) is True, f"FAILURE_POLICY_{field.upper()}_INVALID")
        _require(failure.get("gravity_scale_target_after_failure") == 0.0 and failure.get("automatic_retry_permitted") is False, "FAILURE_ZERO_OR_RETRY_POLICY_INVALID")
        _require(isinstance(runtime, Mapping), "RUNTIME_CONSUMPTION_MISSING")
        for field in (
            "consumer_implementation_required",
            "file_sha256_pin_required",
            "session_and_state_instance_match_required",
            "anchor_sha256_match_required",
            "expiry_check_required",
            "strict_stage_sequence_required",
            "live_gate_check_required_every_cycle",
        ):
            _require(runtime.get(field) is True, f"RUNTIME_{field.upper()}_INVALID")
        _require(runtime.get("this_file_alone_enables_hardware") is False, "ENVELOPE_ALONE_MUST_NOT_ENABLE_HARDWARE")
        remaining_ns = int((expires - now).total_seconds() * 1.0e9)
        _require(remaining_ns > 0, "ENVELOPE_EXPIRED_OR_NOT_YET_VALID")
        return cls(
            envelope_id=envelope_id,
            sha256=digest,
            session_id=session_id,
            state_instance_id=state_instance_id,
            anchor_sha256=anchor_sha256,
            created_at_utc=created,
            expires_at_utc=expires,
            lifetime_seconds=lifetime,
            configured_hold_seconds=hold,
            ramp_seconds=ramp,
            maximum_total_active_seconds=maximum_total,
            maximum_position_validation_seconds=position_total_seconds,
            maximum_position_segment_seconds=position_segment_seconds,
            maximum_abs_position_segment_deg=position_displacement,
            loaded_monotonic_ns=loaded_monotonic_ns,
            monotonic_deadline_ns=loaded_monotonic_ns + remaining_ns,
            assisted_teach_enabled=assisted_teach_enabled,
            hand_guidance_enabled=hand_guidance_enabled,
            hand_guidance_excursion_deg=guidance["maximum_excursion_from_press_deg"] if hand_guidance_enabled else 10.0,
        )

    def claim_single_use(self, claim_directory: Path) -> Path:
        """Atomically spend this envelope across node restarts on one host."""

        directory = Path(os.path.abspath(claim_directory))
        _require(not directory.is_symlink(), "EMPIRICAL_CLAIM_DIRECTORY_SYMLINK")
        directory.mkdir(parents=True, exist_ok=True, mode=0o700)
        _require(directory.is_dir() and not directory.is_symlink(), "EMPIRICAL_CLAIM_DIRECTORY_INVALID")
        claim = directory / f"{self.sha256}.claim"
        flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0)
        if hasattr(os, "O_NOFOLLOW"):
            flags |= os.O_NOFOLLOW
        try:
            descriptor = os.open(claim, flags, stat.S_IRUSR | stat.S_IWUSR)
        except FileExistsError as exc:
            raise EmpiricalEnvelopeError("EMPIRICAL_ENVELOPE_ALREADY_CLAIMED") from exc
        try:
            payload = json.dumps({
                "schema": "go-m8010-empirical-envelope-runtime-claim/1.0",
                "envelope_id": self.envelope_id,
                "envelope_sha256": self.sha256,
                "pid": os.getpid(),
                "claimed_at_utc": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            }, ensure_ascii=True, sort_keys=True, separators=(",", ":")).encode("ascii") + b"\n"
            os.write(descriptor, payload)
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
        return claim

    def runtime_blocker(
        self,
        *,
        session_id: str,
        state_instance_id: str,
        anchor_sha256: str,
        now_utc: Optional[datetime] = None,
        now_monotonic_ns: Optional[int] = None,
    ) -> str:
        now = datetime.now(timezone.utc) if now_utc is None else now_utc
        checked_ns = (
            time.monotonic_ns()
            if now_monotonic_ns is None
            else now_monotonic_ns
        )
        if (
            type(checked_ns) is not int
            or checked_ns < self.loaded_monotonic_ns
            or checked_ns >= self.monotonic_deadline_ns
        ):
            return "EMPIRICAL_ENVELOPE_MONOTONIC_DEADLINE_EXPIRED"
        if now.tzinfo is None or not self.created_at_utc <= now.astimezone(timezone.utc) < self.expires_at_utc:
            return "EMPIRICAL_ENVELOPE_EXPIRED"
        if session_id != self.session_id:
            return "EMPIRICAL_SESSION_MISMATCH"
        if state_instance_id != self.state_instance_id:
            return "EMPIRICAL_STATE_INSTANCE_MISMATCH"
        if anchor_sha256 != self.anchor_sha256:
            return "EMPIRICAL_ANCHOR_SHA256_MISMATCH"
        return ""


def validate_stage_confirmation(
    value: object,
    *,
    envelope: EmpiricalValidationEnvelope,
    target_scale: float,
    now_monotonic_ns: int,
) -> tuple[str, int, int]:
    required = {
        "schema", "source_instance_id", "sequence", "source_monotonic_ns",
        "envelope_id", "envelope_sha256", "session_id", "state_instance_id",
        "target_gravity_scale", "operator_stop_ready",
        "j2_j3_support_reliable", "clearance_confirmed", "no_person_contact",
    }
    hand_guidance = getattr(envelope, "hand_guidance_enabled", False)
    if hand_guidance:
        required.remove("j2_j3_support_reliable")
        required.update({"base_fixed", "external_arm_support", "established_position_hold"})
    mapping = _exact_mapping(value, required, "STAGE_CONFIRMATION")
    _require(mapping.get("schema") == (HAND_GUIDANCE_CONFIRMATION_SCHEMA if hand_guidance else CONFIRMATION_SCHEMA), "STAGE_CONFIRMATION_SCHEMA")
    source = mapping.get("source_instance_id")
    _require(isinstance(source, str) and len(source) == 32 and all(c in "0123456789abcdef" for c in source), "STAGE_CONFIRMATION_SOURCE")
    sequence = mapping.get("sequence")
    source_ns = mapping.get("source_monotonic_ns")
    _require(type(sequence) is int and sequence > 0, "STAGE_CONFIRMATION_SEQUENCE")
    _require(type(source_ns) is int and 0 < source_ns <= now_monotonic_ns and now_monotonic_ns - source_ns <= MAXIMUM_CONFIRMATION_AGE_NS, "STAGE_CONFIRMATION_STALE")
    _require(mapping.get("envelope_id") == envelope.envelope_id and mapping.get("envelope_sha256") == envelope.sha256, "STAGE_CONFIRMATION_ENVELOPE_MISMATCH")
    _require(mapping.get("session_id") == envelope.session_id and mapping.get("state_instance_id") == envelope.state_instance_id, "STAGE_CONFIRMATION_SESSION_MISMATCH")
    _require(_finite(mapping.get("target_gravity_scale"), "STAGE_CONFIRMATION_TARGET") == target_scale, "STAGE_CONFIRMATION_TARGET_MISMATCH")
    for field in ("operator_stop_ready", "clearance_confirmed", "no_person_contact"):
        _require(mapping.get(field) is True, f"STAGE_CONFIRMATION_{field.upper()}_FALSE")
    if hand_guidance:
        _require(mapping["base_fixed"] is True and mapping["external_arm_support"] is False
                 and (mapping["established_position_hold"] is True or
                      (target_scale == 0.0 and mapping["established_position_hold"] is False)),
                 "STAGE_CONFIRMATION_POSITION_HOLD_INVALID")
    else:
        _require(mapping["j2_j3_support_reliable"] is True, "STAGE_CONFIRMATION_J2_J3_SUPPORT_RELIABLE_FALSE")
    return source, sequence, source_ns


def assisted_teach_stopping_joint(hardware_state: Mapping, now_monotonic_ns: int) -> Optional[int]:
    """One fresh native HOLD proof grants only its selected-axis stop window.

    A historical expired proof remains useful evidence; it grants no velocity
    exemption. The raw-state decoder independently binds ownership and epoch.
    """
    if hardware_state.get("assisted_teach_exit_hold_validated") is not True:
        return None
    proof = hardware_state.get("assisted_teach_exit_hold")
    source_ns = hardware_state.get("assisted_teach_exit_hold_source_monotonic_ns")
    if not isinstance(proof, Mapping) or proof.get("schema") != "go-m8010-teach-exit-hold/1.0":
        return None
    if proof.get("restricted") is True:
        return None  # Restricted HOLD never extends the ordinary speed gate.
    joint = proof.get("joint_index")
    started, deadline = proof.get("started_monotonic_ns"), proof.get("deadline_monotonic_ns")
    if (type(joint) is not int or not 0 <= joint < 5 or type(started) is not int
            or type(deadline) is not int or type(source_ns) is not int
            or not 0 < started <= source_ns <= hardware_state.get("source_monotonic_ns", 0) <= now_monotonic_ns
            or deadline - started != TEACH_STOPPING_HOLD_NS
            or not now_monotonic_ns < deadline
            or now_monotonic_ns - source_ns > int(MAXIMUM_FEEDBACK_AGE_MS * 1e6)):
        return None
    modes = hardware_state.get("controller_mode_by_motor")
    if not isinstance(modes, Mapping) or set(modes) != set(MOTOR_NAMES) or any(mode != "hold" for mode in modes.values()):
        return None
    actual, targets = hardware_state.get("position_rad"), proof.get("targets_rad")
    if (not isinstance(actual, list) or len(actual) != 6 or not isinstance(targets, list) or len(targets) != 6
            or any(type(x) not in {int, float} or not math.isfinite(float(x)) for x in actual + targets)
            or abs(actual[joint] - targets[joint]) > TEACH_STOPPING_ERROR_RAD):
        return None
    return joint


def live_hardware_blocker(
    hardware_state: object,
    *,
    session_id: str,
    state_instance_id: str,
    now_monotonic_ns: int,
    stage_start_temperature_c: Optional[Mapping[str, float]] = None,
    require_current_position_hold: bool,
    require_entry_temperature: bool = False,
    allow_assisted_teach: bool = False,
    allow_hand_guidance: bool = False,
    allow_return_only_warnings: bool = False,
) -> tuple[str, Optional[dict[str, float]]]:
    if not isinstance(hardware_state, Mapping):
        return "EMPIRICAL_HARDWARE_STATE_MISSING", None
    if hardware_state.get("session_id") != session_id or hardware_state.get("state_instance_id") != state_instance_id:
        return "EMPIRICAL_HARDWARE_STATE_IDENTITY_MISMATCH", None
    source_ns = hardware_state.get("source_monotonic_ns")
    if type(source_ns) is not int or not 0 < source_ns <= now_monotonic_ns or now_monotonic_ns - source_ns > int(MAXIMUM_FEEDBACK_AGE_MS * 1.0e6):
        return "EMPIRICAL_HARDWARE_STATE_STALE", None
    if hardware_state.get("healthy") is not True or hardware_state.get("safety_metadata_ready") is not True:
        return "EMPIRICAL_HARDWARE_SAFETY_NOT_READY", None
    if hardware_state.get("j2_sync_fault") is not False:
        return "EMPIRICAL_J2_SYNC_FAULT", None
    sync = hardware_state.get("j2_e_sync_rad")
    if type(sync) not in {int, float} or not math.isfinite(float(sync)):
        return "EMPIRICAL_J2_SYNC_INVALID" if allow_return_only_warnings else "EMPIRICAL_J2_SYNC_WARNING", None
    if allow_return_only_warnings and abs(math.degrees(float(sync))) > J2_SYNC_HARD_DEG:
        return "EMPIRICAL_J2_SYNC_HARD_LIMIT", None
    if not allow_return_only_warnings and abs(math.degrees(float(sync))) > J2_SYNC_WARNING_DEG:
        return "EMPIRICAL_J2_SYNC_WARNING", None
    velocities = hardware_state.get("velocity_rad_s")
    if hardware_state.get("assisted_teach_exit_hold_validated") is False:
        return "EMPIRICAL_ASSISTED_TEACH_EXIT_PROOF_INVALID", None
    stopping_joint = assisted_teach_stopping_joint(hardware_state, now_monotonic_ns) if allow_assisted_teach and not allow_hand_guidance else None
    maximum_velocity = math.radians(MAXIMUM_HAND_GUIDANCE_VELOCITY_DEG_S) if allow_hand_guidance else MAXIMUM_HOLD_VELOCITY_RAD_S
    if not isinstance(velocities, list) or len(velocities) != 6 or any(type(v) not in {int, float} or not math.isfinite(float(v)) or (index != stopping_joint and abs(float(v)) > maximum_velocity) for index, v in enumerate(velocities)):
        return "EMPIRICAL_ABNORMAL_VELOCITY", None
    per_motor = hardware_state.get("per_motor")
    if not isinstance(per_motor, Mapping) or set(per_motor) != set(MOTOR_NAMES):
        return "EMPIRICAL_PER_MOTOR_INVALID", None
    temperatures: dict[str, float] = {}
    for name in MOTOR_NAMES:
        sample = per_motor[name]
        if not isinstance(sample, Mapping):
            return f"EMPIRICAL_{name}_STATE_INVALID", None
        if sample.get("fresh") is not True or sample.get("communication_ok") is not True or sample.get("merror") != 0:
            return f"EMPIRICAL_{name}_FEEDBACK_INVALID", None
        age = sample.get("age_ms")
        temperature = sample.get("temperature_c")
        if type(age) not in {int, float} or not math.isfinite(float(age)) or not 0.0 <= float(age) <= MAXIMUM_FEEDBACK_AGE_MS:
            return f"EMPIRICAL_{name}_FEEDBACK_STALE", None
        if type(temperature) not in {int, float} or not math.isfinite(float(temperature)) or not 0.0 <= float(temperature) < HARD_STOP_TEMPERATURE_C:
            return f"EMPIRICAL_{name}_TEMPERATURE_INVALID", None
        temperatures[name] = float(temperature)
        if require_entry_temperature and temperatures[name] >= ENTRY_TEMPERATURE_C:
            return f"EMPIRICAL_{name}_ENTRY_TEMPERATURE", None
        if sample.get("thermal_metadata_status") != "OBSERVED" or sample.get("thermal_fault_latched") is not False:
            return f"EMPIRICAL_{name}_THERMAL_GUARD_INVALID", None
        if sample.get("no_progress_metadata_status") != "OBSERVED" or sample.get("load_limit_no_progress") is not False:
            return f"EMPIRICAL_{name}_NO_PROGRESS_GUARD_INVALID", None
        if not allow_return_only_warnings and stage_start_temperature_c is not None and temperatures[name] - float(stage_start_temperature_c[name]) > MAXIMUM_STAGE_TEMPERATURE_RISE_C:
            return f"EMPIRICAL_{name}_STAGE_TEMPERATURE_RISE", None
    if require_current_position_hold:
        modes = hardware_state.get("controller_mode_by_motor")
        if not isinstance(modes, Mapping) or set(modes) != set(MOTOR_NAMES):
            return "EMPIRICAL_CURRENT_POSITION_HOLD_NOT_CONFIRMED", None
        taught = {name for name in MOTOR_NAMES if modes[name] == "teach"}
        if taught:
            allowed_domains = ({"J1"}, {"J2A", "J2B"}, {"J3"}, {"J4"}, {"J5"})
            scope_allowed = (("J2A" in taught) == ("J2B" in taught)) if allow_hand_guidance else allow_assisted_teach and taught in allowed_domains
            if (not scope_allowed
                    or any(modes[name] != "hold" for name in set(MOTOR_NAMES) - taught)):
                return "EMPIRICAL_ASSISTED_TEACH_MODE_SCOPE_INVALID", None
        elif any(modes[name] not in {"hold", "position"} for name in MOTOR_NAMES):
            return "EMPIRICAL_CURRENT_POSITION_HOLD_NOT_CONFIRMED", None
    return "", temperatures


class EmpiricalStageGate:
    """One-shot strict 0/25/50/75/100 stage sequencer."""

    def __init__(self, envelope: EmpiricalValidationEnvelope) -> None:
        self.envelope = envelope
        self.stage_index = 0
        self.stage_started_ns: Optional[int] = None
        self.hold_started_ns: Optional[int] = None
        self.active_started_ns: Optional[int] = None
        self.stage_start_temperature_c: Optional[dict[str, float]] = None
        self.stage_complete = False
        self.phase = "GRAVITY_LADDER"
        self.position_started_ns: Optional[int] = None
        self.last_position_confirmation_ns: Optional[int] = None
        self.invalidated = False
        self.blocker = "EMPIRICAL_ZERO_HOLD_NOT_STARTED"
        self.motion_warnings: list[str] = []
        self._confirmation: Optional[tuple[dict, tuple[str, int, int]]] = None
        self._last_confirmation_by_source: dict[str, tuple[int, int]] = {}

    def observe_confirmation(self, value: object, *, now_monotonic_ns: int) -> bool:
        target = LEVELS[min(self.stage_index + 1, len(LEVELS) - 1)]
        try:
            identity = validate_stage_confirmation(
                value,
                envelope=self.envelope,
                target_scale=target,
                now_monotonic_ns=now_monotonic_ns,
            )
            previous = self._last_confirmation_by_source.get(identity[0])
            if previous is not None and (identity[1] <= previous[0] or identity[2] <= previous[1]):
                raise EmpiricalEnvelopeError("STAGE_CONFIRMATION_REPLAY")
            self._last_confirmation_by_source[identity[0]] = (identity[1], identity[2])
            self._confirmation = (dict(value), identity)
            return True
        except (EmpiricalEnvelopeError, TypeError, ValueError):
            self._confirmation = None
            return False

    def step(
        self,
        *,
        requested_scale: float,
        applied_scale: float,
        hardware_state: object,
        session_id: str,
        state_instance_id: str,
        anchor_sha256: str,
        hardware_enable_requested: bool,
        now_monotonic_ns: int,
        now_utc: Optional[datetime] = None,
    ) -> bool:
        self.motion_warnings = []
        if self.invalidated:
            return False
        runtime_blocker = self.envelope.runtime_blocker(
            session_id=session_id,
            state_instance_id=state_instance_id,
            anchor_sha256=anchor_sha256,
            now_utc=now_utc,
            now_monotonic_ns=now_monotonic_ns,
        )
        if runtime_blocker:
            self.invalidate(runtime_blocker)
            return False
        if not hardware_enable_requested:
            self.blocker = "EMPIRICAL_HARDWARE_ENABLE_NOT_REQUESTED"
            return False
        if requested_scale not in LEVELS:
            self.invalidate("EMPIRICAL_SCALE_NOT_APPROVED")
            return False
        current_level = LEVELS[self.stage_index]
        if requested_scale < current_level:
            self.invalidate("EMPIRICAL_STAGE_ROLLBACK_INVALIDATES_ENVELOPE")
            return False
        if requested_scale > current_level:
            if self.stage_index + 1 >= len(LEVELS) or requested_scale != LEVELS[self.stage_index + 1]:
                self.invalidate("EMPIRICAL_STAGE_SEQUENCE_VIOLATION")
                return False
            if not self.stage_complete:
                self.blocker = "EMPIRICAL_PREVIOUS_STAGE_NOT_COMPLETE"
                return False
            if self._confirmation is None:
                self.blocker = "EMPIRICAL_STAGE_RECONFIRMATION_REQUIRED"
                return False
            confirmation_ns = self._confirmation[1][2]
            if not 0 <= now_monotonic_ns - confirmation_ns <= MAXIMUM_CONFIRMATION_AGE_NS:
                self._confirmation = None
                self.blocker = "EMPIRICAL_STAGE_RECONFIRMATION_EXPIRED"
                return False
            self.stage_index += 1
            current_level = requested_scale
            self.stage_started_ns = now_monotonic_ns
            self.hold_started_ns = None
            self.stage_start_temperature_c = None
            self.stage_complete = False
            self._confirmation = None
            if self.active_started_ns is None:
                self.active_started_ns = now_monotonic_ns
        if self.active_started_ns is not None and (now_monotonic_ns - self.active_started_ns) * 1.0e-9 > self.envelope.maximum_total_active_seconds:
            if not (self.stage_index == len(LEVELS) - 1 and self.stage_complete):
                self.invalidate("EMPIRICAL_MAXIMUM_ACTIVE_TIME_EXCEEDED")
                return False
        qualified_guidance = bool(
            self.envelope.hand_guidance_enabled
            and self.stage_index == len(LEVELS) - 1 and self.stage_complete
            and self.phase == "POSITION_VALIDATION"
            and self.position_started_ns is not None
            and self.stage_start_temperature_c is not None
            and self.last_position_confirmation_ns is not None
            and 0 <= now_monotonic_ns - self.last_position_confirmation_ns
            and requested_scale == 1.0 and abs(applied_scale - 1.0) <= 1.0e-6
        )
        live_blocker, temperatures = live_hardware_blocker(
            hardware_state,
            session_id=session_id,
            state_instance_id=state_instance_id,
            now_monotonic_ns=now_monotonic_ns,
            stage_start_temperature_c=self.stage_start_temperature_c,
            # Stage zero must first make a zero-feedforward authority
            # available so the Router can command the current-position HOLD.
            # Its hold timer starts only after that HOLD is observed below.
            require_current_position_hold=self.stage_index > 0,
            require_entry_temperature=self.stage_start_temperature_c is None,
            allow_assisted_teach=bool(
                self.envelope.assisted_teach_enabled
                and self.stage_index == len(LEVELS) - 1 and self.stage_complete
                and self.phase == "POSITION_VALIDATION"
                and self.last_position_confirmation_ns is not None
                and 0 <= now_monotonic_ns - self.last_position_confirmation_ns
            ),
            allow_hand_guidance=bool(
                self.envelope.hand_guidance_enabled
                and self.stage_index == len(LEVELS) - 1 and self.stage_complete
                and self.phase == "POSITION_VALIDATION"
                and self.last_position_confirmation_ns is not None
                and 0 <= now_monotonic_ns - self.last_position_confirmation_ns
            ),
            allow_return_only_warnings=qualified_guidance,
        )
        if not live_blocker and qualified_guidance:
            # All hard checks above must finish before any qualification
            # warning can retain authority. Keep the original stage baseline
            # and thresholds; this is not a new successful qualification.
            if abs(math.degrees(float(hardware_state["j2_e_sync_rad"]))) > J2_SYNC_WARNING_DEG:
                self.motion_warnings.append("EMPIRICAL_J2_SYNC_WARNING")
            self.motion_warnings.extend(
                f"EMPIRICAL_{name}_STAGE_TEMPERATURE_RISE" for name in MOTOR_NAMES
                if temperatures[name] - self.stage_start_temperature_c[name] > MAXIMUM_STAGE_TEMPERATURE_RISE_C
            )
            if self.motion_warnings:
                for name in MOTOR_NAMES:
                    if temperatures[name] >= ENTRY_TEMPERATURE_C:
                        live_blocker = f"EMPIRICAL_{name}_RETURN_ONLY_TEMPERATURE_LIMIT"
                        break
        if (not live_blocker and self.envelope.hand_guidance_enabled
                and self.stage_index > 0 and self.phase != "POSITION_VALIDATION"
                and any(mode != "hold" for mode in hardware_state["controller_mode_by_motor"].values())):
            live_blocker = "EMPIRICAL_CURRENT_POSITION_HOLD_NOT_CONFIRMED"
        if live_blocker:
            # Before the first current-position HOLD, stage zero is merely a
            # pending zero-output gate.  Once that HOLD spends the envelope's
            # active budget, every live fault is terminal just like a later
            # rung; recovery must never silently retry the same permit.
            if self.stage_index > 0 or self.active_started_ns is not None:
                self.invalidate(live_blocker)
            else:
                self.blocker = live_blocker
            return False
        if self.stage_started_ns is None:
            self.stage_started_ns = now_monotonic_ns
        if self.stage_start_temperature_c is None:
            self.stage_start_temperature_c = temperatures
        if self.stage_index == 0:
            modes = (
                hardware_state.get("controller_mode_by_motor")
                if isinstance(hardware_state, Mapping) else None
            )
            if (
                not isinstance(modes, Mapping)
                or set(modes) != set(MOTOR_NAMES)
                or any(modes[name] != "hold" for name in MOTOR_NAMES)
            ):
                if self.active_started_ns is not None:
                    self.invalidate(
                        "EMPIRICAL_ZERO_CURRENT_POSITION_HOLD_LOST"
                    )
                    return False
                self.hold_started_ns = None
                self.blocker = "EMPIRICAL_ZERO_CURRENT_POSITION_HOLD_PENDING"
                return True
            if self.active_started_ns is None:
                self.active_started_ns = now_monotonic_ns
        if abs(applied_scale - current_level) > 1.0e-6:
            self.hold_started_ns = None
            self.blocker = "EMPIRICAL_STAGE_RAMPING"
            return True
        if self.hold_started_ns is None:
            self.hold_started_ns = now_monotonic_ns
        held_s = (now_monotonic_ns - self.hold_started_ns) * 1.0e-9
        self.stage_complete = held_s >= self.envelope.configured_hold_seconds
        if self.stage_index == len(LEVELS) - 1 and self.stage_complete:
            self.phase = "POSITION_VALIDATION_PENDING_RECONFIRMATION"
            if self._confirmation is not None:
                confirmation_ns = self._confirmation[1][2]
                if 0 <= now_monotonic_ns - confirmation_ns <= MAXIMUM_CONFIRMATION_AGE_NS:
                    self.last_position_confirmation_ns = confirmation_ns
                    if self.position_started_ns is None:
                        self.position_started_ns = now_monotonic_ns
                    self._confirmation = None
            if (
                self.last_position_confirmation_ns is not None
                and 0 <= now_monotonic_ns - self.last_position_confirmation_ns
                and (self.envelope.assisted_teach_enabled or self.envelope.hand_guidance_enabled or
                     now_monotonic_ns - self.last_position_confirmation_ns
                     <= MAXIMUM_CONFIRMATION_AGE_NS)
            ):
                self.phase = "POSITION_VALIDATION"
            elif self.position_started_ns is not None:
                self.invalidate(
                    "EMPIRICAL_POSITION_OPERATOR_STOP_CONFIRMATION_EXPIRED"
                )
                return False
            # The 600 s bound is a Router-accounted cumulative budget of
            # unique accepted POSITION trajectory durations.  Static HOLD and
            # 5/15/30 minute thermal observation wall time do not spend it.
        self.blocker = "" if self.stage_complete else "EMPIRICAL_STAGE_HOLDING"
        return True

    def invalidate(self, blocker: str) -> None:
        self.invalidated = True
        self.stage_complete = False
        self.motion_warnings = []
        self.blocker = blocker or "EMPIRICAL_ENVELOPE_INVALIDATED"

    def status(self) -> dict:
        guidance = self.envelope.hand_guidance_enabled
        teach_enabled = self.envelope.assisted_teach_enabled or guidance
        teach_authorized = bool(teach_enabled and self.phase == "POSITION_VALIDATION"
            and self.stage_index == len(LEVELS) - 1 and self.stage_complete
            and not self.invalidated and not self.blocker and not self.motion_warnings)
        return {
            "authority_class": AUTHORITY_CLASS,
            "rating_classification": RATING_CLASSIFICATION,
            "envelope_id": self.envelope.envelope_id,
            "envelope_sha256": self.envelope.sha256,
            "anchor_sha256": self.envelope.anchor_sha256,
            "expires_at_utc": self.envelope.expires_at_utc.isoformat().replace("+00:00", "Z"),
            "stage_index": self.stage_index,
            "stage_level": LEVELS[self.stage_index],
            "stage_complete": self.stage_complete,
            "phase": self.phase,
            "position_validation_authorized": (
                self.phase == "POSITION_VALIDATION" and not self.invalidated
            ),
            # Position authority in this combination is restricted by the
            # Router/native consumers to signed return segments toward the
            # already bound session origin. It does not grant arbitrary moves.
            "return_only": bool(guidance and self.motion_warnings and not self.invalidated),
            "motion_warnings": list(self.motion_warnings),
            "assisted_teach_authorized": teach_authorized,
            "hand_guidance_authorized": guidance and teach_authorized,
            "maximum_teach_excursion_deg": self.envelope.hand_guidance_excursion_deg if guidance else MAXIMUM_TEACH_EXCURSION_DEG if teach_enabled else None,
            "maximum_teach_seconds": MAXIMUM_HAND_GUIDANCE_SECONDS if guidance else MAXIMUM_TEACH_SECONDS if teach_enabled else None,
            "maximum_teach_velocity_deg_s": MAXIMUM_HAND_GUIDANCE_VELOCITY_DEG_S if guidance else MAXIMUM_TEACH_VELOCITY_DEG_S if teach_enabled else None,
            "allowed_teach_joints": list(HAND_GUIDANCE_JOINTS if guidance else ASSISTED_TEACH_JOINTS) if teach_enabled else [],
            "maximum_position_segment_seconds": (
                self.envelope.maximum_position_segment_seconds
            ),
            "maximum_abs_position_segment_deg": (
                self.envelope.maximum_abs_position_segment_deg
            ),
            "maximum_cumulative_position_trajectory_seconds": (
                self.envelope.maximum_position_validation_seconds
            ),
            "invalidated": self.invalidated,
            "blocker": self.blocker or None,
            "continuous_operation_authorized": False,
            "official_continuous_rating_claimed": False,
        }


__all__ = [
    "AUTHORITY_CLASS",
    "CONFIRMATION_SCHEMA",
    "EmpiricalEnvelopeError",
    "EmpiricalStageGate",
    "EmpiricalValidationEnvelope",
    "RATING_CLASSIFICATION",
    "SOFTWARE_GRAVITY_ROTOR_LIMIT_NM",
    "live_hardware_blocker",
    "validate_stage_confirmation",
    "select_runtime_torque_authority",
]

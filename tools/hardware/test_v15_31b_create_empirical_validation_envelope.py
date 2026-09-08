from __future__ import annotations

import copy
import ast
import hashlib
import importlib.util
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import pytest


SCRIPT = Path(__file__).with_name(
    "v15_31b_create_empirical_validation_envelope.py"
)
SPEC = importlib.util.spec_from_file_location("v15_31b_empirical", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(tool)

RUNNER_SCRIPT = Path(__file__).with_name(
    "v15_31b_active_acceptance_runner.py"
)
RUNNER_SPEC = importlib.util.spec_from_file_location(
    "v15_31b_active_acceptance_runner_for_envelope_integration",
    RUNNER_SCRIPT,
)
assert RUNNER_SPEC is not None and RUNNER_SPEC.loader is not None
runner_mod = importlib.util.module_from_spec(RUNNER_SPEC)
sys.modules[RUNNER_SPEC.name] = runner_mod
RUNNER_SPEC.loader.exec_module(runner_mod)

REPO_ROOT = Path(__file__).resolve().parents[2]
MODEL = (
    REPO_ROOT
    / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
    / "mujoco_v15_14"
    / "go_m8010_arm_v15_14_kinematic.xml"
)
CONFIG_ROOT = (
    REPO_ROOT
    / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
    / "ros2_ws"
    / "src"
    / "go_m8010_arm_hardware"
    / "config"
)
GRAVITY_CONFIG = CONFIG_ROOT / "gravity_control.yaml"
THERMAL_CONFIG = CONFIG_ROOT / "thermal_limits.yaml"
SESSION_ID = (
    "persistent:0123456789abcdef:"
    "j2session:1111111111111111:goauxsession:2222222222222222"
)
STATE_INSTANCE_ID = "3" * 32
NOW = datetime(2026, 8, 31, 12, 0, 0, tzinfo=timezone.utc)


def _power_on() -> dict:
    per_motor = {}
    for name in tool.MOTOR_NAMES:
        per_motor[name] = {
            "online": True,
            "max_feedback_age_ms": 8.0,
            "max_temperature_c": 34.0,
            "max_abs_merror": 0,
            "communication_interruptions": 0,
            "abnormal_velocity_detected": False,
            "observed_modes": ["brake"],
        }
    return {
        "schema_version": "V15.31B-power-on-readonly-v1",
        "result": "PASS",
        "duration_s": 10.0,
        "session_id": SESSION_ID,
        "state_instance_id": STATE_INSTANCE_ID,
        "per_motor": per_motor,
        "j2": {"max_abs_e_sync_deg": 0.10},
        "safety": {
            "position_enabled": False,
            "motor_internal_zero_modified": False,
            "flash_written": False,
            "eeprom_written": False,
            "active_command_count": 0,
        },
    }


def _anchor_validation(
    source_power_on_sha256: str = "f" * 64,
) -> dict:
    anchor = {
        "schema": tool.ANCHOR_SCHEMA,
        "model_sha256": tool.PRODUCTION_MODEL_SHA256,
        "session_id": SESSION_ID,
        "state_instance_id": STATE_INSTANCE_ID,
        "created_utc": "2026-08-31T11:59:00Z",
        "gear_ratio": tool.GO_GEAR_RATIO,
        "motor_direction_sign": dict(tool.FROZEN_MOTOR_SIGNS),
        "motor_raw_reference_rad": {
            name: float(index) for index, name in enumerate(tool.MOTOR_NAMES)
        },
        "motor_encoder_branch": {name: 0 for name in tool.MOTOR_NAMES},
        "logical_joint_reference_rad": [0.0] * 6,
        "model_absolute_joint_rad": [0.0] * 6,
    }
    anchor_sha256 = hashlib.sha256(tool.json_bytes(anchor)).hexdigest()
    return {
        "schema": tool.ANCHOR_VALIDATION_SCHEMA,
        "result": "PASS",
        "session_id": SESSION_ID,
        "state_instance_id": STATE_INSTANCE_ID,
        "source_power_on_readonly_sha256": source_power_on_sha256,
        "anchor_sha256": anchor_sha256,
        "runtime_anchor_sha256": anchor_sha256,
        "anchor_field_count": len(anchor),
        "anchor": anchor,
        "checks": {
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
        },
    }


ANCHOR_SHA256 = _anchor_validation()["runtime_anchor_sha256"]


def _gravity_sample() -> dict:
    gravity = [
        0.10 * tool.GO_GEAR_RATIO,
        1.00 * tool.GO_GEAR_RATIO,
        0.50 * tool.GO_GEAR_RATIO,
        0.10 * tool.GO_GEAR_RATIO,
        0.05 * tool.GO_GEAR_RATIO,
        0.0,
    ]
    return {
        "q_actual_rad": [0.0] * 6,
        "model_q_rad": [0.0] * 6,
        "gravity_joint_nm": gravity,
        "predicted_rotor_nm": {
            "J1": 0.10,
            "J2A": -0.50,
            "J2B": +0.50,
            "J3": 0.50,
            "J4": -0.10,
            "J5": 0.05,
        },
    }


def _gravity_readonly() -> dict:
    sample = _gravity_sample()
    second = copy.deepcopy(sample)
    second["q_actual_rad"][1] = math.radians(0.1)
    second["model_q_rad"][1] = math.radians(0.1)
    return {
        "schema_version": tool.GRAVITY_READONLY_SCHEMA,
        "result": "PASS",
        "session_id": SESSION_ID,
        "state_instance_id": STATE_INSTANCE_ID,
        "anchor_sha256": ANCHOR_SHA256,
        "hardware_tff_enabled": False,
        "tff_transmitted": False,
        "checks": {
            "finite": True,
            "continuous": True,
            "direction_reasonable": True,
            "direction_independent_of_motion": True,
            "j2_split_50_50": True,
            "j2a_sign_correct": True,
            "j2b_sign_correct": True,
        },
        "samples": [sample, second],
    }


def _write_json(path: Path, document: dict) -> str:
    path.write_text(
        json.dumps(document, ensure_ascii=False, allow_nan=False, indent=2) + "\n",
        encoding="utf-8",
    )
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _arguments(
    tmp_path: Path,
    *,
    power: dict | None = None,
    anchor: dict | None = None,
    gravity: dict | None = None,
    extra: list[str] | None = None,
) -> list[str]:
    power_path = tmp_path / "power_on_readonly.json"
    anchor_path = tmp_path / "model_session_anchor_validation.json"
    gravity_path = tmp_path / "gravity_readonly_validation.json"
    power_sha = _write_json(power_path, _power_on() if power is None else power)
    anchor_sha = _write_json(
        anchor_path,
        _anchor_validation(power_sha) if anchor is None else anchor,
    )
    gravity_sha = _write_json(
        gravity_path, _gravity_readonly() if gravity is None else gravity
    )
    return [
        "--power-on-readonly", str(power_path),
        "--expected-power-on-sha256", power_sha,
        "--model-session-anchor-validation", str(anchor_path),
        "--expected-anchor-validation-sha256", anchor_sha,
        "--gravity-readonly-validation", str(gravity_path),
        "--expected-gravity-readonly-sha256", gravity_sha,
        "--model", str(MODEL),
        "--gravity-config", str(GRAVITY_CONFIG),
        "--thermal-config", str(THERMAL_CONFIG),
        "--expected-session-id", SESSION_ID,
        "--expected-state-instance-id", STATE_INSTANCE_ID,
        "--output", str(tmp_path / "runtime" / "empirical_envelope.json"),
        *(extra or []),
    ]


def _run(tmp_path: Path, **kwargs) -> dict:
    return tool.run(tool.parse_args(_arguments(tmp_path, **kwargs)), now=NOW)


def test_assisted_teach_cli_opt_in_is_separate_and_changes_bound_identity(tmp_path):
    ordinary = _run(tmp_path)
    opted = _run(tmp_path, extra=["--assisted-teach"])
    assert "assisted_teach" not in ordinary["envelope"]
    assert ordinary["envelope"]["envelope_id"] != opted["envelope"]["envelope_id"]
    assert ordinary["envelope_sha256"] != opted["envelope_sha256"]
    teach = opted["envelope"]["assisted_teach"]
    assert teach["allowed_joints"] == ["J1", "J2", "J3", "J4", "J5"]
    assert teach["maximum_selected_joints"] == 1
    assert teach["maximum_press_seconds"] == 30.0
    assert teach["maximum_excursion_from_press_deg"] == 5.0
    assert teach["maximum_velocity_deg_s"] == 5.0
    assert opted["envelope"]["position_validation"] == ordinary["envelope"]["position_validation"]
    assert opted["hardware_accessed"] is False and opted["applied"] is False


def test_hand_guidance_cli_is_distinct_and_records_unsupported_arm_hold_policy(tmp_path):
    ordinary = _run(tmp_path)["envelope"]
    result = _run(tmp_path, extra=["--hand-guidance"])
    envelope = result["envelope"]
    guidance = envelope["hand_guidance"]
    assert "assisted_teach" not in envelope
    assert guidance["schema"] == "go-m8010-hand-guidance-envelope/1.0"
    assert guidance["allowed_joints"] == ["J1", "J2", "J3", "J4", "J5", "J6"]
    assert guidance["maximum_selected_joints"] == 6
    assert guidance["maximum_excursion_from_press_deg"] == 10.0
    assert guidance["maximum_press_seconds"] == 600.0
    assert guidance["maximum_velocity_deg_s"] == 30.0
    assert guidance["reference_lead_deg"] == 2.0
    assert guidance["normal_exit_action"] == guidance["time_limit_action"] == "KEEP_POSITION_HOLD"
    assert guidance["drive_release_requires"] == "VERIFIED_VERTICAL_POSE"
    assert envelope["live_gates"]["physical_support"] == {
        "base_fixed": True, "external_arm_support": False,
        "established_position_hold_required": True,
    }
    assert all(stage["support_reconfirmation_required"] is False for stage in envelope["staged_activation"]["stages"])
    assert envelope["torque_basis"] == ordinary["torque_basis"]
    assert envelope["position_validation"] == ordinary["position_validation"]
    assert result["hardware_accessed"] is False and result["applied"] is False
    with pytest.raises(SystemExit):
        tool.parse_args(_arguments(tmp_path, extra=["--hand-guidance", "--assisted-teach"]))


def test_default_is_validate_only_and_builds_exact_bounded_ladder(
    tmp_path: Path,
) -> None:
    result = _run(tmp_path)
    envelope = result["envelope"]

    assert result["mode"] == "VALIDATE_ONLY"
    assert result["applied"] is False
    assert result["hardware_accessed"] is False
    assert not (tmp_path / "runtime" / "empirical_envelope.json").exists()
    assert envelope["schema"] == tool.ENVELOPE_SCHEMA
    assert envelope["authority_class"] == "EMPIRICAL_VALIDATION_ENVELOPE"
    assert envelope["rating_classification"] == "NOT_OFFICIAL_CONTINUOUS_RATING"
    assert envelope["binding"] == {
        "session_id": SESSION_ID,
        "state_instance_id": STATE_INSTANCE_ID,
        "anchor_sha256": ANCHOR_SHA256,
        "model_sha256": tool.PRODUCTION_MODEL_SHA256,
        "gravity_config_sha256": tool.GRAVITY_CONFIG_SHA256,
        "thermal_config_sha256": tool.THERMAL_CONFIG_SHA256,
        "all_fields_must_match_runtime": True,
    }
    assert envelope["session_id"] == SESSION_ID
    assert envelope["state_instance_id"] == STATE_INSTANCE_ID
    assert envelope["anchor_sha256"] == ANCHOR_SHA256
    assert envelope["expires_at_utc"] == "2026-08-31T13:10:00Z"
    ladder = envelope["staged_activation"]
    assert ladder["levels"] == [0.0, 0.25, 0.5, 0.75, 1.0]
    assert ladder["ramp_seconds"] == 2.0
    assert ladder["configured_hold_seconds"] == 5.0
    assert ladder["observation_joint_priority"] == ["J2", "J3"]
    assert ladder["j2_j3_observation_priority_required"] is True
    assert ladder["maximum_interstage_confirmation_seconds"] == 30.0
    assert ladder["interstage_confirmation_count"] == 4
    assert ladder["maximum_total_active_seconds"] == 153.5
    assert ladder["absolute_maximum_total_active_seconds"] == 178.5
    assert [stage["gravity_scale"] for stage in ladder["stages"]] == ladder["levels"]
    assert all(
        stage["ramp_from_previous_seconds"] == 2.0
        for stage in ladder["stages"][1:]
    )
    position = envelope["position_validation"]
    assert position["precision_contract_id"] == "go-m8010-position-accuracy/0.1deg-v1"
    assert position["endpoint_error_limit_deg"] == 0.1
    assert position["nominal_command_displacement_deg"] == 5.0
    assert position["minimum_actual_displacement_deg"] == 4.8
    assert position[tool.POSITION_BUDGET_FIELD] == 600.0
    assert position[
        "wall_clock_hold_observation_counts_against_budget"
    ] is False


def test_real_creator_envelope_loads_directly_into_runner_binding(
    tmp_path: Path,
) -> None:
    result = _run(tmp_path)
    envelope = result["envelope"]
    assert tool.POSITION_BUDGET_FIELD == runner_mod.POSITION_BUDGET_FIELD
    envelope_path = tmp_path / "runtime" / "empirical_envelope.json"
    envelope_path.parent.mkdir()
    envelope_path.write_bytes(tool.json_bytes(envelope))

    loaded = runner_mod.EvidenceBinding.from_paths(
        envelope_path,
        tmp_path / "model_session_anchor_validation.json",
        result["envelope_sha256"],
    )

    assert loaded.session_id == SESSION_ID
    assert loaded.state_instance_id == STATE_INSTANCE_ID
    assert loaded.anchor_sha256 == ANCHOR_SHA256
    assert loaded.maximum_position_seconds == 600.0
    assert loaded.maximum_segment_seconds == 15.0

    legacy = copy.deepcopy(envelope)
    legacy_position = legacy["position_validation"]
    legacy_position["maximum_total_seconds"] = legacy_position.pop(
        tool.POSITION_BUDGET_FIELD
    )
    with pytest.raises(
        runner_mod.AcceptanceError,
        match="POSITION_MAXIMUM_TOTAL_INVALID",
    ):
        runner_mod.EvidenceBinding.from_documents(
            legacy,
            _anchor_validation(),
            result["envelope_sha256"],
        )


def test_creator_rejects_tampered_embedded_anchor_with_retained_sha(
    tmp_path: Path,
) -> None:
    arguments = _arguments(tmp_path)
    validation_path = tmp_path / "model_session_anchor_validation.json"
    validation = json.loads(validation_path.read_text(encoding="utf-8"))
    validation["anchor"]["motor_raw_reference_rad"]["J1"] += 0.125
    tampered_sha = _write_json(validation_path, validation)
    sha_index = arguments.index("--expected-anchor-validation-sha256") + 1
    arguments[sha_index] = tampered_sha

    with pytest.raises(
        tool.EnvelopeCreationError,
        match="anchor SHA-256 does not match the embedded runtime anchor",
    ):
        tool.run(tool.parse_args(arguments), now=NOW)


def test_real_hand_guidance_package_reaches_binding_and_both_confirmation_readers(tmp_path):
    now = datetime.now(timezone.utc)
    result = tool.run(tool.parse_args(_arguments(tmp_path, extra=["--hand-guidance"])), now=now)
    envelope = result["envelope"]
    envelope_path = tmp_path / "hand_guidance_envelope.json"
    envelope_path.write_bytes(tool.json_bytes(envelope))
    bound = runner_mod.EvidenceBinding.from_paths(envelope_path,
        tmp_path / "model_session_anchor_validation.json", result["envelope_sha256"])
    assert bound.hand_guidance_enabled
    hardware_source = CONFIG_ROOT.parent
    sys.path.insert(0, str(hardware_source))
    from go_m8010_arm_hardware.empirical_validation_envelope import (
        EmpiricalValidationEnvelope, EmpiricalEnvelopeError, validate_stage_confirmation,
    )
    from v15_31b_acceptance_signal import SignalPayloadSequence

    runtime = EmpiricalValidationEnvelope.from_path(envelope_path,
        result["envelope_sha256"], now_utc=now, now_monotonic_ns=5_000_000_000)
    run = runner_mod.ActiveAcceptanceRunner(bound)
    source_clock = [5_000_000_000]
    sequence = SignalPayloadSequence(bound, monotonic_ns=lambda: source_clock[0])
    # Execute the actual demo conversion, which keeps the generic signal
    # client's old supported-arm schema away from the hand-guidance topic.
    demo_path = SCRIPT.with_name("v15_31d_gui_j1_demo.py")
    tree = ast.parse(demo_path.read_text(encoding="utf-8"))
    live = next(item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == "run_live")
    confirm = next(item for item in live.body if isinstance(item, ast.FunctionDef) and item.name == "confirm")
    conversion = next(item for item in confirm.body if isinstance(item, ast.If)
                      and isinstance(item.test, ast.Name) and item.test.id == "hand_guidance")
    for target, mode, valid in ((0.0, "brake", True), (0.25, "brake", False), (0.25, "hold", True)):
        source_clock[0] += 1_000_000
        confirmation = sequence.confirmation(target)
        assert not run.observe_confirmation(confirmation, now_ns=source_clock[0])
        context = {"hand_guidance": True, "confirmation": confirmation,
                   "hardware": {"controller_mode_by_motor": {name: mode for name in tool.MOTOR_NAMES}}}
        exec(compile(ast.Module(body=[conversion], type_ignores=[]), str(demo_path), "exec"), context)
        assert confirmation["schema"] == "go-m8010-empirical-stage-confirmation/1.1"
        assert "j2_j3_support_reliable" not in confirmation
        assert confirmation["external_arm_support"] is False
        assert run.observe_confirmation(confirmation, now_ns=source_clock[0]) is valid
        if valid:
            validate_stage_confirmation(confirmation, envelope=runtime, target_scale=target,
                                        now_monotonic_ns=source_clock[0])
        else:
            with pytest.raises(EmpiricalEnvelopeError, match="POSITION_HOLD"):
                validate_stage_confirmation(confirmation, envelope=runtime, target_scale=target,
                                            now_monotonic_ns=source_clock[0])
    anchor = json.loads((tmp_path / "model_session_anchor_validation.json").read_bytes())
    for invalid in ({"foo": True}, {**envelope["hand_guidance"], "maximum_selected_joints": True}):
        malformed = {**envelope, "hand_guidance": invalid}
        with pytest.raises(runner_mod.AcceptanceError, match="HAND_GUIDANCE"):
            runner_mod.EvidenceBinding.from_documents(malformed, anchor, result["envelope_sha256"])
    unsupported_legacy = dict(envelope)
    unsupported_legacy.pop("hand_guidance")
    with pytest.raises(runner_mod.AcceptanceError, match="SUPPORT_GATE"):
        runner_mod.EvidenceBinding.from_documents(unsupported_legacy, anchor, result["envelope_sha256"])


def test_runner_rejects_replaced_anchor_validation_file_by_source_pin(
    tmp_path: Path,
) -> None:
    result = _run(tmp_path)
    envelope_path = tmp_path / "runtime" / "empirical_envelope.json"
    envelope_path.parent.mkdir()
    envelope_path.write_bytes(tool.json_bytes(result["envelope"]))
    validation_path = tmp_path / "model_session_anchor_validation.json"
    validation_path.write_bytes(validation_path.read_bytes() + b"\n")

    with pytest.raises(
        runner_mod.AcceptanceError,
        match="ANCHOR_VALIDATION_FILE_SHA256_MISMATCH",
    ):
        runner_mod.EvidenceBinding.from_paths(
            envelope_path,
            validation_path,
            result["envelope_sha256"],
        )


def test_envelope_disclaims_ratings_and_encodes_every_live_gate(
    tmp_path: Path,
) -> None:
    envelope = _run(tmp_path)["envelope"]

    assert envelope["frozen_authority"]["continuous_rotor_limits_authoritative"] is False
    basis = envelope["torque_basis"]
    assert basis["maximum_output_used_as_continuous_rating"] is False
    assert basis["peak_output_used_as_continuous_rating"] is False
    assert basis["historical_feedback_used_as_continuous_rating"] is False
    assert basis["any_peak_or_history_claim_used_as_continuous_rating"] is False
    assert basis["continuous_operation_authorized"] is False
    gates = envelope["live_gates"]
    assert set(gates) == {
        "feedback", "temperature", "merror", "j2_sync", "no_progress",
        "operator_stop", "physical_support",
    }
    assert gates["temperature"]["entry_below_c"] == 55.0
    assert gates["temperature"]["valid_must_remain_below_c"] == 60.0
    assert gates["temperature"]["hard_stop_c"] == 60.0
    assert gates["merror"]["required_value"] == 0
    assert gates["j2_sync"]["warning_above_deg"] == 0.25
    assert gates["j2_sync"]["hard_above_deg"] == 0.5
    assert gates["operator_stop"]["required_before_each_nonzero_stage"] is True
    assert envelope["failure_policy"]["automatic_retry_permitted"] is False
    assert envelope["runtime_consumption"]["this_file_alone_enables_hardware"] is False


def test_apply_requires_exact_gate_is_atomic_and_never_overwrites(
    tmp_path: Path,
) -> None:
    common = _arguments(tmp_path, extra=["--apply"])
    with pytest.raises(tool.EnvelopeCreationError, match="apply requires"):
        tool.run(tool.parse_args(common), now=NOW)

    applied = tool.run(tool.parse_args([
        *common, "--confirm", tool.APPLY_GATE,
    ]), now=NOW)
    output = tmp_path / "runtime" / "empirical_envelope.json"
    assert applied["applied"] is True
    before = output.read_bytes()
    stored = json.loads(before)
    assert stored["envelope_id"].startswith("v15-31b-empirical-")

    with pytest.raises(tool.EnvelopeCreationError, match="refuse to overwrite"):
        tool.run(tool.parse_args([
            *common, "--confirm", tool.APPLY_GATE,
        ]), now=NOW)
    assert output.read_bytes() == before


def test_rejects_model_demand_above_software_hard_limit(tmp_path: Path) -> None:
    gravity = _gravity_readonly()
    for sample in gravity["samples"]:
        sample["gravity_joint_nm"][0] = 0.21 * tool.GO_GEAR_RATIO
        sample["predicted_rotor_nm"]["J1"] = 0.21
    with pytest.raises(tool.EnvelopeCreationError, match="software hard limit"):
        _run(tmp_path, gravity=gravity)


@pytest.mark.parametrize(
    ("mutator", "message"),
    [
        (lambda power, gravity: power["per_motor"]["J3"].update(max_temperature_c=55.0), "entry temperature"),
        (lambda power, gravity: power["per_motor"]["J2A"].update(max_abs_merror=1), "merror"),
        (lambda power, gravity: power["per_motor"]["J6"].update(observed_modes=["position"]), "BRAKE/DISABLED"),
        (lambda power, gravity: power["j2"].update(max_abs_e_sync_deg=0.251), "sync exceeds warning"),
        (lambda power, gravity: gravity.update(hardware_tff_enabled=True), "enabled hardware Tff"),
        (lambda power, gravity: gravity.update(tff_transmitted=True), "transmitted Tff"),
        (lambda power, gravity: gravity.update(state_instance_id="4" * 32), "state instance mismatch"),
    ],
)
def test_readonly_and_live_entry_gates_are_fail_closed(
    tmp_path: Path, mutator, message: str
) -> None:
    power = _power_on()
    gravity = _gravity_readonly()
    mutator(power, gravity)
    with pytest.raises(tool.EnvelopeCreationError, match=message):
        _run(tmp_path, power=power, gravity=gravity)


@pytest.mark.parametrize(
    ("extra", "message"),
    [
        (["--hold-seconds", "4.99"], "5-10 seconds"),
        (["--hold-seconds", "10.01"], "5-10 seconds"),
        (["--lifetime-seconds", "59"], "60-4200"),
        (["--lifetime-seconds", "4201"], "60-4200"),
    ],
)
def test_stage_and_expiry_bounds_are_strict(
    tmp_path: Path, extra: list[str], message: str
) -> None:
    with pytest.raises(tool.EnvelopeCreationError, match=message):
        _run(tmp_path, extra=extra)


def test_all_three_evidence_inputs_are_hash_pinned(tmp_path: Path) -> None:
    arguments = _arguments(tmp_path)
    power_path = tmp_path / "power_on_readonly.json"
    power_path.write_bytes(power_path.read_bytes() + b" ")
    with pytest.raises(tool.EnvelopeCreationError, match="SHA-256 does not match"):
        tool.run(tool.parse_args(arguments), now=NOW)


def test_frozen_model_and_configs_are_exact_hash_authority(tmp_path: Path) -> None:
    changed = tmp_path / "gravity_control.yaml"
    changed.write_bytes(GRAVITY_CONFIG.read_bytes() + b"\n")
    arguments = _arguments(tmp_path)
    index = arguments.index("--gravity-config") + 1
    arguments[index] = str(changed)
    with pytest.raises(tool.EnvelopeCreationError, match="frozen SHA-256"):
        tool.run(tool.parse_args(arguments), now=NOW)

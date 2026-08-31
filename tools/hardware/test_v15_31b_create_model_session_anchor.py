from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path
import sys

import pytest


SCRIPT = Path(__file__).with_name("v15_31b_create_model_session_anchor.py")
SPEC = importlib.util.spec_from_file_location("v15_31b_anchor", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
anchor_tool = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(anchor_tool)

PACKAGE_ROOT = (
    Path(__file__).resolve().parents[2]
    / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
    / "ros2_ws"
    / "src"
    / "go_m8010_arm_hardware"
)
sys.path.insert(0, str(PACKAGE_ROOT))
from go_m8010_arm_hardware.gravity_model import GravityModelAnchorV2  # noqa: E402


SESSION_ID = (
    "persistent:0123456789abcdef:"
    "j2session:1111111111111111:goauxsession:2222222222222222"
)
STATE_INSTANCE_ID = "3" * 32


def _raw_references() -> dict[str, float]:
    return {
        "J1": 0.10,
        "J2A": 2.0 * math.pi + 0.12,
        "J2B": -2.0 * math.pi - 0.13,
        "J3": 4.0 * math.pi + 0.14,
        "J4": -4.0 * math.pi - 0.15,
        "J5": 0.16,
        "J6": 0.17,
    }


def _branches(raw: dict[str, float]) -> dict[str, int]:
    result = {}
    for name, value in raw.items():
        if name == "J6":
            result[name] = 0
        else:
            phase = (value + math.pi) % (2.0 * math.pi) - math.pi
            result[name] = round((value - phase) / (2.0 * math.pi))
    return result


def _summary(values: list[float]) -> dict[str, float]:
    return {
        "min": min(values),
        "max": max(values),
        "mean": sum(values) / len(values),
    }


def valid_power_on_document() -> dict:
    raw = _raw_references()
    logical = [0.01, -0.02, 0.03, -0.04, 0.05, -0.06]
    samples = []
    for index in range(41):
        source_ns = 1_000_000_000 + index * 250_000_000
        signed_delta = (index - 20) * 1.0e-7
        per_motor = {}
        for name, raw_reference in raw.items():
            if name == "J6":
                observed_raw = raw_reference + signed_delta
            else:
                observed_raw = anchor_tool.normalized_phase(raw_reference) + signed_delta
            per_motor[name] = {
                "raw_position_rad": observed_raw,
                "age_ms": 5.0,
                "temperature_c": 30.0,
                "merror": 0,
                "communication_ok": True,
                "fresh": True,
            }
        samples.append({
            "schema": anchor_tool.HARDWARE_STATE_SCHEMA,
            "sequence": index + 1,
            "source_monotonic_ns": source_ns,
            "session_id": SESSION_ID,
            "state_instance_id": STATE_INSTANCE_ID,
            "position_rad": [value + signed_delta for value in logical],
            "velocity_rad_s": [0.0] * 6,
            "velocity_source": "POSITION_SPAN_0P40S",
            "j2_e_sync_rad": 0.0,
            "j2_sync_fault": False,
            "controller_mode_by_motor": dict(anchor_tool.SAFE_MODE_BY_MOTOR),
            "per_motor": per_motor,
        })

    per_motor_summary = {}
    for name in anchor_tool.MOTOR_NAMES:
        records = [sample["per_motor"][name] for sample in samples]
        logical_values = [
            logical[anchor_tool.MOTOR_TO_JOINT_INDEX[name]] + (index - 20) * 1.0e-7
            for index in range(len(samples))
        ]
        summary = {
            "online": True,
            "max_feedback_age_ms": max(record["age_ms"] for record in records),
            "raw_position_rad": _summary([record["raw_position_rad"] for record in records]),
            "logical_position_rad": _summary(logical_values),
            "max_abs_velocity_rad_s": 0.0,
            "max_temperature_c": max(record["temperature_c"] for record in records),
            "abnormal_velocity_detected": False,
            "max_abs_merror": 0,
            "communication_interruptions": 0,
            "observed_modes": [anchor_tool.SAFE_MODE_BY_MOTOR[name]],
        }
        if name == "J6":
            summary.update({
                "tau_cmd_rotor_nm": None,
                "tau_feedback_rotor_nm": None,
                "tau_joint_estimated_nm": None,
            })
        else:
            summary.update({
                "tau_feedback_rotor_nm": _summary([
                    0.01 for _record in records
                ]),
                "tau_joint_estimated_nm": _summary([
                    anchor_tool.FROZEN_MOTOR_SIGNS[name]
                    * anchor_tool.GO_GEAR_RATIO
                    * 0.01
                    for _record in records
                ]),
                "max_abs_tau_cmd_rotor_nm": 0.0,
            })
        per_motor_summary[name] = summary

    return {
        "schema": anchor_tool.POWER_ON_SCHEMA,
        "status": "PASS",
        "duration_s": 10.0,
        "valid_frame_count": len(samples),
        "recorded_at_utc": "2026-08-31T12:00:00Z",
        "model_sha256": anchor_tool.PRODUCTION_MODEL_SHA256,
        "session_id": SESSION_ID,
        "state_instance_id": STATE_INSTANCE_ID,
        "motor_direction_sign": dict(anchor_tool.FROZEN_MOTOR_SIGNS),
        "motor_gear_ratio": dict(anchor_tool.FROZEN_MOTOR_GEAR_RATIOS),
        "motor_raw_reference_rad": raw,
        "motor_encoder_branch": _branches(raw),
        "logical_joint_reference_rad": logical,
        "model_absolute_joint_rad": [
            0.0,
            math.pi / 2.0,
            math.radians(-14.40),
            math.radians(13.49),
            math.radians(47.94),
            0.0,
        ],
        "samples": samples,
        "per_motor": per_motor_summary,
        "j2": {
            "logical_position_rad": _summary([
                sample["position_rad"][1] for sample in samples
            ]),
            "max_abs_e_sync_deg": 0.0,
            "j2a_logical_torque_contribution_nm": _summary([
                anchor_tool.FROZEN_MOTOR_SIGNS["J2A"]
                * anchor_tool.GO_GEAR_RATIO
                * 0.01
                for _sample in samples
            ]),
            "j2b_logical_torque_contribution_nm": _summary([
                anchor_tool.FROZEN_MOTOR_SIGNS["J2B"]
                * anchor_tool.GO_GEAR_RATIO
                * 0.01
                for _sample in samples
            ]),
        },
        "safety": {
            "position_enabled": False,
            "motor_internal_zero_modified": False,
            "flash_written": False,
            "eeprom_written": False,
            "active_command_count": 0,
        },
        "operator_confirmation": {
            "confirmed_at_utc": "2026-08-31T12:00:00Z",
            "mechanism_stationary": True,
            "model_pose_aligned": True,
            "support_reliable": True,
            "not_at_mechanical_limit": True,
        },
        "writes": {
            "motor_internal_zero_modified": False,
            "rid_written": False,
            "flash_or_eeprom_written": False,
        },
    }


def build(document: dict) -> dict:
    return anchor_tool.build_anchor(
        document,
        expected_session_id=SESSION_ID,
        expected_state_instance_id=STATE_INSTANCE_ID,
    )


def test_builds_exact_runtime_eleven_field_anchor() -> None:
    document = valid_power_on_document()
    anchor = build(document)
    assert set(anchor) == anchor_tool.EXACT_ANCHOR_FIELDS
    assert len(anchor) == 11
    assert anchor["schema"] == anchor_tool.ANCHOR_SCHEMA
    assert anchor["model_sha256"] == anchor_tool.PRODUCTION_MODEL_SHA256
    assert anchor["session_id"] == SESSION_ID
    assert anchor["state_instance_id"] == STATE_INSTANCE_ID
    assert anchor["motor_raw_reference_rad"] == document["motor_raw_reference_rad"]
    assert anchor["motor_encoder_branch"] == document["motor_encoder_branch"]
    assert anchor["logical_joint_reference_rad"] == document["logical_joint_reference_rad"]
    assert anchor["model_absolute_joint_rad"] == document["model_absolute_joint_rad"]


def test_runtime_loader_accepts_the_generated_anchor() -> None:
    anchor = build(valid_power_on_document())
    loaded = GravityModelAnchorV2.from_mapping(anchor)
    assert loaded.session_id == SESSION_ID
    assert loaded.state_instance_id == STATE_INSTANCE_ID
    assert loaded.model_absolute_joint_rad == tuple(anchor["model_absolute_joint_rad"])


@pytest.mark.parametrize(
    "mutate, reason",
    (
        (lambda value: value.update(model_sha256="0" * 64), "frozen model"),
        (lambda value: value["motor_direction_sign"].update(J2A=1), "direction sign"),
        (lambda value: value["motor_gear_ratio"].update(J6=6.33), "gear ratio"),
        (lambda value: value["motor_encoder_branch"].update(J3=0), "encoder branch"),
        (lambda value: value["motor_raw_reference_rad"].pop("J5"), "seven motors"),
        (lambda value: value.update(logical_joint_reference_rad=[0.0] * 5), "exactly 6"),
        (lambda value: value.update(model_absolute_joint_rad=[0.0] * 5), "exactly 6"),
    ),
)
def test_rejects_invalid_anchor_authority(mutate, reason: str) -> None:
    document = valid_power_on_document()
    mutate(document)
    with pytest.raises(anchor_tool.AnchorCreationError, match=reason):
        build(document)


def test_rejects_mixed_session_or_state_instance() -> None:
    for field, replacement in (
        ("session_id", "persistent:other"),
        ("state_instance_id", "4" * 32),
    ):
        document = valid_power_on_document()
        document["samples"][1][field] = replacement
        with pytest.raises(anchor_tool.AnchorCreationError, match="same session/state"):
            build(document)


def test_rejects_less_than_ten_seconds_or_unsafe_state() -> None:
    short = valid_power_on_document()
    short["duration_s"] = 9.999
    with pytest.raises(anchor_tool.AnchorCreationError, match="less than 10 seconds"):
        build(short)

    active = valid_power_on_document()
    active["samples"][1]["controller_mode_by_motor"]["J3"] = "hold"
    with pytest.raises(anchor_tool.AnchorCreationError, match="left BRAKE/DISABLED"):
        build(active)

    error = valid_power_on_document()
    error["samples"][1]["per_motor"]["J2B"]["merror"] = 1
    with pytest.raises(anchor_tool.AnchorCreationError, match="merror"):
        build(error)


def test_uses_position_span_velocity_and_preserves_noisy_vendor_diagnostics() -> None:
    noisy = valid_power_on_document()
    noisy["per_motor"]["J1"]["max_abs_velocity_rad_s"] = 0.14
    assert build(noisy)["schema"] == anchor_tool.ANCHOR_SCHEMA

    wrong_observer = valid_power_on_document()
    wrong_observer["samples"][1]["velocity_source"] = "RAW_VENDOR_DQ"
    with pytest.raises(anchor_tool.AnchorCreationError, match="position-span"):
        build(wrong_observer)


def test_rejects_summary_sample_or_model_pose_divergence() -> None:
    raw_summary = valid_power_on_document()
    raw_summary["per_motor"]["J1"]["raw_position_rad"]["mean"] += 0.10
    with pytest.raises(anchor_tool.AnchorCreationError, match="raw summary"):
        build(raw_summary)

    count = valid_power_on_document()
    count["valid_frame_count"] += 1
    with pytest.raises(anchor_tool.AnchorCreationError, match="retained sample count"):
        build(count)

    model_range = valid_power_on_document()
    model_range["model_absolute_joint_rad"][4] = math.pi
    with pytest.raises(anchor_tool.AnchorCreationError, match="frozen model joint range"):
        build(model_range)


def test_rejects_noncontinuous_samples_or_any_torque_command() -> None:
    gap = valid_power_on_document()
    gap["samples"][20]["source_monotonic_ns"] += 1
    with pytest.raises(anchor_tool.AnchorCreationError, match="continuous read-only coverage"):
        build(gap)

    commanded = valid_power_on_document()
    commanded["per_motor"]["J3"]["max_abs_tau_cmd_rotor_nm"] = 1.0e-6
    with pytest.raises(anchor_tool.AnchorCreationError, match="nonzero active torque command"):
        build(commanded)


def test_validate_only_is_default_and_does_not_write(tmp_path: Path) -> None:
    source = tmp_path / "power_on_readonly.json"
    source.write_text(json.dumps(valid_power_on_document()), encoding="utf-8")
    output = tmp_path / "runtime" / "gravity_model_anchor_v2.json"
    validation_output = tmp_path / "evidence" / "model_session_anchor_validation.json"
    source_sha = hashlib.sha256(source.read_bytes()).hexdigest()
    result = anchor_tool.run(anchor_tool.parse_args([
        "--power-on-readonly", str(source),
        "--expected-power-on-sha256", source_sha,
        "--expected-session-id", SESSION_ID,
        "--expected-state-instance-id", STATE_INSTANCE_ID,
        "--output", str(output),
        "--validation-output", str(validation_output),
    ]))
    assert result["mode"] == "VALIDATE_ONLY"
    assert result["applied"] is False
    assert result["anchor_field_count"] == 11
    assert result["runtime_anchor_sha256"] == result["anchor_sha256"]
    assert result["checks"] == {
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
    assert not output.exists()
    assert not validation_output.exists()


def test_apply_is_gated_atomic_and_no_overwrite(tmp_path: Path) -> None:
    source = tmp_path / "power_on_readonly.json"
    source.write_text(json.dumps(valid_power_on_document()), encoding="utf-8")
    source_sha = hashlib.sha256(source.read_bytes()).hexdigest()
    output = tmp_path / "runtime" / "gravity_model_anchor_v2.json"
    validation_output = tmp_path / "evidence" / "model_session_anchor_validation.json"
    common = [
        "--power-on-readonly", str(source),
        "--expected-power-on-sha256", source_sha,
        "--expected-session-id", SESSION_ID,
        "--expected-state-instance-id", STATE_INSTANCE_ID,
        "--output", str(output),
        "--validation-output", str(validation_output),
        "--apply",
    ]
    with pytest.raises(anchor_tool.AnchorCreationError, match="apply requires"):
        anchor_tool.run(anchor_tool.parse_args(common))
    assert not output.exists()
    assert not validation_output.exists()

    result = anchor_tool.run(anchor_tool.parse_args([
        *common, "--confirm", anchor_tool.APPLY_GATE,
    ]))
    assert result["applied"] is True
    if os.name != "nt":
        assert output.stat().st_mode & 0o777 == 0o600
        assert validation_output.stat().st_mode & 0o777 == 0o600
    stored = json.loads(output.read_text(encoding="utf-8"))
    assert set(stored) == anchor_tool.EXACT_ANCHOR_FIELDS
    validation = json.loads(validation_output.read_text(encoding="utf-8"))
    assert set(validation) == anchor_tool.EXACT_VALIDATION_FIELDS
    assert validation["schema"] == anchor_tool.ANCHOR_VALIDATION_SCHEMA
    assert validation["result"] == "PASS"
    assert validation["session_id"] == SESSION_ID
    assert validation["state_instance_id"] == STATE_INSTANCE_ID
    assert validation["anchor"] == stored
    assert validation["source_power_on_readonly_sha256"] == source_sha
    assert validation["anchor_sha256"] == validation["runtime_anchor_sha256"]
    assert validation["runtime_anchor_sha256"] == hashlib.sha256(
        output.read_bytes()
    ).hexdigest()
    assert result["validation_sha256"] == hashlib.sha256(
        validation_output.read_bytes()
    ).hexdigest()
    before = output.read_bytes()
    validation_before = validation_output.read_bytes()
    with pytest.raises(anchor_tool.AnchorCreationError, match="refuse to overwrite"):
        anchor_tool.run(anchor_tool.parse_args([
            *common, "--confirm", anchor_tool.APPLY_GATE,
        ]))
    assert output.read_bytes() == before
    assert validation_output.read_bytes() == validation_before


def test_validation_preexistence_prevents_partial_runtime_publication(
    tmp_path: Path,
) -> None:
    source = tmp_path / "power_on_readonly.json"
    source.write_text(json.dumps(valid_power_on_document()), encoding="utf-8")
    source_sha = hashlib.sha256(source.read_bytes()).hexdigest()
    output = tmp_path / "runtime" / "gravity_model_anchor_v2.json"
    validation_output = tmp_path / "model_session_anchor_validation.json"
    validation_output.write_text("owned\n", encoding="utf-8")

    with pytest.raises(anchor_tool.AnchorCreationError, match="refuse to overwrite"):
        anchor_tool.run(anchor_tool.parse_args([
            "--power-on-readonly", str(source),
            "--expected-power-on-sha256", source_sha,
            "--expected-session-id", SESSION_ID,
            "--expected-state-instance-id", STATE_INSTANCE_ID,
            "--output", str(output),
            "--validation-output", str(validation_output),
            "--apply",
            "--confirm", anchor_tool.APPLY_GATE,
        ]))

    assert not output.exists()
    assert validation_output.read_text(encoding="utf-8") == "owned\n"


def test_second_publication_failure_never_leaves_runtime_without_validation(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    runtime = tmp_path / "runtime" / "gravity_model_anchor_v2.json"
    validation = tmp_path / "evidence" / "model_session_anchor_validation.json"
    original_write = anchor_tool.atomic_write_new
    calls = 0

    def fail_second(path: Path, data: bytes) -> None:
        nonlocal calls
        calls += 1
        if calls == 2:
            raise anchor_tool.AnchorCreationError("injected runtime failure")
        original_write(path, data)

    monkeypatch.setattr(anchor_tool, "atomic_write_new", fail_second)
    with pytest.raises(anchor_tool.AnchorCreationError, match="injected"):
        anchor_tool.atomic_write_pair_new(
            runtime,
            b"runtime\n",
            validation,
            b"validation\n",
        )

    assert not runtime.exists()
    assert validation.read_bytes() == b"validation\n"


def test_runtime_and_validation_output_must_be_distinct(tmp_path: Path) -> None:
    shared = tmp_path / "anchor.json"
    with pytest.raises(anchor_tool.AnchorCreationError, match="must differ"):
        anchor_tool.atomic_write_pair_new(
            shared,
            b"runtime\n",
            shared,
            b"validation\n",
        )
    assert not shared.exists()


def test_source_hash_and_write_guards_are_mandatory(tmp_path: Path) -> None:
    document = valid_power_on_document()
    document["writes"]["flash_or_eeprom_written"] = True
    with pytest.raises(anchor_tool.AnchorCreationError, match="forbidden hardware write"):
        build(document)

    source = tmp_path / "power_on_readonly.json"
    source.write_text(json.dumps(valid_power_on_document()), encoding="utf-8")
    with pytest.raises(anchor_tool.AnchorCreationError, match="SHA-256 does not match"):
        anchor_tool.read_source(source, "0" * 64)


def test_input_is_not_mutated() -> None:
    document = valid_power_on_document()
    original = copy.deepcopy(document)
    build(document)
    assert document == original

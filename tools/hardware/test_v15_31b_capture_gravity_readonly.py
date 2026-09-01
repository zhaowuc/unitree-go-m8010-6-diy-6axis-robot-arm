from __future__ import annotations

import hashlib
import importlib.util
import json
import math
import sys
from pathlib import Path

import pytest


SCRIPT = Path(__file__).with_name("v15_31b_capture_gravity_readonly.py")
SPEC = importlib.util.spec_from_file_location("v15_31b_gravity_capture", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
capture = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = capture
SPEC.loader.exec_module(capture)

REPO_ROOT = SCRIPT.parents[2]
EVIDENCE_VALIDATOR_PATH = REPO_ROOT / "tools" / "validate_v15_31b_ft_evidence.py"
EMPIRICAL_TOOL_PATH = SCRIPT.with_name(
    "v15_31b_create_empirical_validation_envelope.py"
)
SESSION = "j2-session--go-aux-session"
INSTANCE = "3" * 32
ANCHOR_SHA = "a" * 64


def import_script(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    assert spec is not None and spec.loader is not None
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def anchor_mapping() -> dict:
    return {
        "schema": capture.ANCHOR_SCHEMA,
        "model_sha256": capture.PRODUCTION_MODEL_SHA256,
        "session_id": SESSION,
        "state_instance_id": INSTANCE,
        "created_utc": "2026-08-31T12:00:00Z",
        "gear_ratio": capture.GO_GEAR_RATIO,
        "motor_direction_sign": dict(capture.FROZEN_MOTOR_SIGNS),
        "motor_raw_reference_rad": {
            name: 0.0 for name in capture.MOTOR_NAMES
        },
        "motor_encoder_branch": {name: 0 for name in capture.MOTOR_NAMES},
        "logical_joint_reference_rad": [0.10, 0.20, 0.30, 0.40, 0.50, 0.60],
        "model_absolute_joint_rad": [0.01, 0.02, 0.03, 0.04, 0.05, 0.06],
    }


class FakeEvaluator:
    BASE = (
        0.10 * capture.GO_GEAR_RATIO,
        0.20 * 2.0 * capture.GO_GEAR_RATIO,
        0.10 * capture.GO_GEAR_RATIO,
        0.05 * capture.GO_GEAR_RATIO,
        0.02 * capture.GO_GEAR_RATIO,
        0.0,
    )

    def evaluate(self, model_q_rad) -> tuple[float, ...]:
        q = capture.finite_vector(model_q_rad, 6, "fake model q")
        # Linear, pose-dependent values make independent recomputation and the
        # +/- direction probes observable while keeping all GO demands within
        # the frozen empirical-envelope software clamps.
        return tuple(
            base + 0.01 * q[index]
            for index, base in enumerate(self.BASE)
        )


def anchor() -> object:
    return capture.ModelSessionAnchor.from_mapping(anchor_mapping())


def recorder(*, minimum_valid_samples: int = 2) -> object:
    return capture.GravityReadOnlyRecorder(
        anchor=anchor(),
        anchor_sha256=ANCHOR_SHA,
        evaluator=FakeEvaluator(),
        minimum_duration_ns=capture.MINIMUM_DURATION_NS,
        minimum_valid_samples=minimum_valid_samples,
    )


def pose_for(index: int) -> list[float]:
    # Keep the real mechanism stationary but retain tiny encoder variation.
    return [
        0.10 + index * 1.0e-8,
        0.20 - index * 1.0e-8,
        0.30 + index * 0.5e-8,
        0.40,
        0.50,
        0.60,
    ]


def hardware_state(sequence: int, source_ns: int, q_actual: list[float]) -> dict:
    motor_logical = {
        "J1": q_actual[0],
        "J2A": q_actual[1] - math.radians(0.05),
        "J2B": q_actual[1] + math.radians(0.05),
        "J3": q_actual[2],
        "J4": q_actual[3],
        "J5": q_actual[4],
        "J6": q_actual[5],
    }
    motors = {}
    for name in capture.MOTOR_NAMES:
        is_j6 = name == "J6"
        motors[name] = {
            "raw_position_rad": 0.0,
            "q_joint_rad": motor_logical[name],
            "dq_joint_rad_s": 0.0,
            "tau_cmd_rotor_nm": None if is_j6 else 0.0,
            "tau_feedback_rotor_nm": None if is_j6 else 0.1,
            "tau_joint_estimated_nm": None if is_j6 else 0.2,
            "temperature_c": 31.0,
            "merror": 0,
            "communication_ok": True,
            "fresh": True,
            "age_ms": 5.0,
        }
    return {
        "schema": capture.HARDWARE_STATE_SCHEMA,
        "sequence": sequence,
        "source_monotonic_ns": source_ns,
        "session_id": SESSION,
        "state_instance_id": INSTANCE,
        "position_rad": list(q_actual),
        "velocity_rad_s": [0.0] * 6,
        "velocity_source": "POSITION_SPAN_0P40S",
        "j2_e_sync_rad": math.radians(0.10),
        "j2_sync_fault": False,
        "controller_mode_by_motor": {
            name: ("disabled" if name == "J6" else "brake")
            for name in capture.MOTOR_NAMES
        },
        "per_motor": motors,
    }


def gravity_status(
    sequence: int,
    source_ns: int,
    state_document: dict,
    anchor_value=None,
) -> dict:
    anchor_value = anchor_value or anchor()
    q_actual = state_document["position_rad"]
    model_q = anchor_value.model_q_from_actual(q_actual)
    gravity = FakeEvaluator().evaluate(model_q)
    return {
        "schema": capture.GRAVITY_STATUS_SCHEMA,
        "source": "whole_arm_gravity_node",
        "source_instance_id": "4" * 32,
        "sequence": sequence,
        "source_monotonic_ns": source_ns,
        "model_sha256": capture.PRODUCTION_MODEL_SHA256,
        "production_model_hash_match": True,
        "gravity_config_sha256": capture.GRAVITY_CONFIG_SHA256,
        "anchor_valid": True,
        "session_id": SESSION,
        "state_instance_id": INSTANCE,
        "hardware_state_sequence": state_document["sequence"],
        "hardware_state_source_monotonic_ns": state_document[
            "source_monotonic_ns"
        ],
        "q_actual_sha256": capture.canonical_pose_sha256(q_actual),
        "joint_state_crosscheck": True,
        "calculation_rate_hz": 100.0,
        "last_update_age_s": 0.0,
        "gravity_joint_nm": list(gravity),
        "feedforward_nm": [0.0] * 6,
        "gravity_scale": 0.0,
        "gravity_scale_target": 0.0,
        "finite_bounded": True,
        "hardware_enable_requested": False,
        "hardware_tff_enabled": False,
    }


def add_ten_second_window(
    value, *, status_first: bool = False, sample_count: int = 41
) -> None:
    assert sample_count >= 2
    step_ns = 10_000_000_000 // (sample_count - 1)
    for index in range(sample_count):
        source_ns = 1_000_000_000 + index * step_ns
        state = hardware_state(index + 1, source_ns, pose_for(index))
        status = gravity_status(index + 1, source_ns + 1_000_000, state)
        if status_first:
            value.add_gravity_status(status)
            value.add_hardware_state(state)
        else:
            value.add_hardware_state(state)
            value.add_gravity_status(status)


def test_builds_validator_and_empirical_envelope_compatible_evidence() -> None:
    value = recorder(minimum_valid_samples=100)
    add_ten_second_window(value, sample_count=101)

    document = value.build_document()

    assert document["schema"] == capture.GRAVITY_READONLY_SCHEMA
    assert document["result"] == "PASS"
    assert document["duration_s"] == 10.0
    assert document["valid_sample_count"] == 101
    assert document["hardware_tff_enabled"] is False
    assert document["tff_transmitted"] is False
    assert all(document["checks"].values())
    assert document["safety"]["collector_command_publishers"] == 0
    assert len(document["direction_probes"]) == 6
    sample = document["samples"][0]
    expected_model_q = [0.01, 0.02, 0.03, 0.04, 0.05, 0.06]
    assert sample["model_q_rad"] == pytest.approx(expected_model_q)
    expected_j2 = sample["gravity_joint_nm"][1] / (
        2.0 * capture.GO_GEAR_RATIO
    )
    assert sample["predicted_rotor_nm"]["J2A"] == pytest.approx(-expected_j2)
    assert sample["predicted_rotor_nm"]["J2B"] == pytest.approx(+expected_j2)

    evidence_validator = import_script(
        EVIDENCE_VALIDATOR_PATH, "v15_31b_evidence_validator_for_gravity_test"
    )
    status, _summary = evidence_validator._validate_gravity_readonly(
        document,
        {"session_id": SESSION, "state_instance_id": INSTANCE},
        {"anchor_sha256": ANCHOR_SHA},
    )
    assert status == "PASS"

    empirical = import_script(
        EMPIRICAL_TOOL_PATH, "v15_31b_empirical_for_gravity_test"
    )
    maximum = empirical.validate_gravity_readonly(
        document, SESSION, INSTANCE, ANCHOR_SHA
    )
    assert maximum["J2A"] < empirical.SOFTWARE_GRAVITY_ROTOR_LIMIT_NM["J2A"]


def test_cross_topic_reordering_is_paired_without_weakening_identity() -> None:
    value = recorder()
    add_ten_second_window(value, status_first=True)
    assert value.complete is True
    assert value.build_document()["valid_sample_count"] == 41


def test_joint_state_ui_crosscheck_is_diagnostic_not_authoritative() -> None:
    value = recorder()
    state = hardware_state(1, 1_000_000_000, pose_for(0))
    status = gravity_status(1, 1_001_000_000, state)
    status["joint_state_crosscheck"] = False
    value.add_hardware_state(state)
    value.add_gravity_status(status)
    assert value.joint_state_crosscheck_false_count == 1

    invalid = recorder()
    status["joint_state_crosscheck"] = "false"
    invalid.add_hardware_state(state)
    with pytest.raises(
        capture.GravityReadOnlyCaptureError,
        match="cross-check is invalid",
    ):
        invalid.add_gravity_status(status)


def test_discards_only_pre_window_volatile_subscription_race() -> None:
    value = recorder()
    missed = hardware_state(10, 1_000_000_000, pose_for(0))
    value.add_gravity_status(gravity_status(10, 1_001_000_000, missed))
    first_observed = hardware_state(11, 1_250_000_000, pose_for(1))
    value.add_hardware_state(first_observed)
    assert value.pending_statuses == []
    first_status = gravity_status(11, 1_251_000_000, first_observed)
    value.add_gravity_status(first_status)
    assert len(value.samples) == 1

    # The same missing reference after capture starts is never discarded.
    with pytest.raises(
        capture.GravityReadOnlyCaptureError, match="unobserved hardware state"
    ):
        value.add_gravity_status(
            gravity_status(12, 1_501_000_000, missed)
        )


@pytest.mark.parametrize(
    ("mutation", "reason"),
    (
        (lambda status, state: status.update(hardware_tff_enabled=True), "hardware Tff"),
        (lambda status, state: status.update(hardware_enable_requested=True), "enable"),
        (lambda status, state: status.update(gravity_scale=0.25), "scale/target"),
        (lambda status, state: status.update(gravity_scale_target=0.25), "scale/target"),
        (
            lambda status, state: status.update(
                feedforward_nm=[0.0, 0.01, 0.0, 0.0, 0.0, 0.0]
            ),
            "feedforward",
        ),
        (lambda status, state: status.update(anchor_valid=False), "anchor"),
        (lambda status, state: status.update(q_actual_sha256="0" * 64), "q_actual"),
        (
            lambda status, state: status["gravity_joint_nm"].__setitem__(1, 9.0),
            "independent MuJoCo",
        ),
        (
            lambda status, state: state["controller_mode_by_motor"].update(
                J3="position"
            ),
            "active/unknown",
        ),
        (
            lambda status, state: state["per_motor"]["J2A"].update(
                tau_cmd_rotor_nm=0.01
            ),
            "nonzero active torque",
        ),
        (
            lambda status, state: state["per_motor"]["J5"].update(merror=1),
            "merror",
        ),
        (
            lambda status, state: state.update(
                j2_e_sync_rad=math.radians(0.251)
            ),
            "synchronization warning",
        ),
    ),
)
def test_rejects_any_actuation_or_invalid_model_state(mutation, reason: str) -> None:
    value = recorder()
    state = hardware_state(1, 1_000_000_000, pose_for(0))
    status = gravity_status(1, 1_001_000_000, state)
    mutation(status, state)
    with pytest.raises(capture.GravityReadOnlyCaptureError, match=reason):
        value.add_hardware_state(state)
        value.add_gravity_status(status)


def test_rejects_sequence_gap_time_gap_session_change_and_missing_pair() -> None:
    value = recorder()
    first = hardware_state(1, 1_000_000_000, pose_for(0))
    value.add_hardware_state(first)
    value.add_gravity_status(gravity_status(1, 1_001_000_000, first))
    skipped = hardware_state(3, 1_250_000_000, pose_for(1))
    with pytest.raises(capture.GravityReadOnlyCaptureError, match="sequence gap"):
        value.add_hardware_state(skipped)

    value = recorder()
    first = hardware_state(1, 1_000_000_000, pose_for(0))
    value.add_hardware_state(first)
    delayed = hardware_state(2, 1_250_000_001, pose_for(1))
    with pytest.raises(capture.GravityReadOnlyCaptureError, match="telemetry gap"):
        value.add_hardware_state(delayed)

    value = recorder()
    changed = hardware_state(1, 1_000_000_000, pose_for(0))
    changed["state_instance_id"] = "5" * 32
    with pytest.raises(capture.GravityReadOnlyCaptureError, match="instance"):
        value.add_hardware_state(changed)

def test_rejects_incomplete_or_zero_direction_evidence() -> None:
    value = recorder()
    first = hardware_state(1, 1_000_000_000, pose_for(0))
    value.add_hardware_state(first)
    value.add_gravity_status(gravity_status(1, 1_001_000_000, first))
    with pytest.raises(capture.GravityReadOnlyCaptureError, match="incomplete"):
        value.build_document()

    class ZeroEvaluator:
        def evaluate(self, _model_q):
            return (0.0,) * 6

    value = capture.GravityReadOnlyRecorder(
        anchor=anchor(),
        anchor_sha256=ANCHOR_SHA,
        evaluator=ZeroEvaluator(),
        minimum_duration_ns=capture.MINIMUM_DURATION_NS,
        minimum_valid_samples=2,
    )
    for index in range(41):
        source_ns = 1_000_000_000 + index * 250_000_000
        state = hardware_state(index + 1, source_ns, pose_for(index))
        status = gravity_status(index + 1, source_ns + 1_000_000, state)
        status["gravity_joint_nm"] = [0.0] * 6
        value.add_hardware_state(state)
        value.add_gravity_status(status)
    with pytest.raises(capture.GravityReadOnlyCaptureError, match="all-zero"):
        value.build_document()


def test_json_parser_rejects_duplicates_and_nonfinite_values() -> None:
    with pytest.raises(capture.GravityReadOnlyCaptureError, match="duplicate"):
        capture.decode_json_object('{"a":1,"a":2}', "sample")
    with pytest.raises(capture.GravityReadOnlyCaptureError, match="non-finite"):
        capture.decode_json_object('{"a":NaN}', "sample")


def test_anchor_hash_and_atomic_no_overwrite(tmp_path: Path) -> None:
    data = (json.dumps(anchor_mapping(), indent=2) + "\n").encode("utf-8")
    anchor_path = tmp_path / "anchor.json"
    anchor_path.write_bytes(data)
    digest = hashlib.sha256(data).hexdigest()
    loaded, observed = capture.read_anchor(anchor_path, digest)
    assert loaded.session_id == SESSION
    assert observed == digest
    with pytest.raises(capture.GravityReadOnlyCaptureError, match="SHA-256"):
        capture.read_anchor(anchor_path, "0" * 64)

    output = tmp_path / "evidence" / "gravity_readonly_validation.json"
    evidence = capture.json_bytes({"result": "PASS"})
    capture.atomic_write_new(output, evidence)
    assert output.read_bytes() == evidence
    with pytest.raises(capture.GravityReadOnlyCaptureError, match="overwrite"):
        capture.atomic_write_new(output, b"different")
    assert output.read_bytes() == evidence


def test_preflight_requires_exact_gate_and_caller_pins(tmp_path: Path) -> None:
    anchor_data = (json.dumps(anchor_mapping(), indent=2) + "\n").encode("utf-8")
    anchor_path = tmp_path / "anchor.json"
    anchor_path.write_bytes(anchor_data)
    digest = hashlib.sha256(anchor_data).hexdigest()
    model_path = (
        REPO_ROOT
        / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
        / "mujoco_v15_14"
        / "go_m8010_arm_v15_14_kinematic.xml"
    )
    args = capture.parse_args([
        "--execute-readonly",
        "--confirm", capture.READONLY_GATE,
        "--model-path", str(model_path),
        "--anchor", str(anchor_path),
        "--expected-anchor-sha256", digest,
        "--expected-session-id", SESSION,
        "--expected-state-instance-id", INSTANCE,
        "--output", str(tmp_path / "output.json"),
    ])
    loaded, observed, evaluator = capture.validate_preflight(
        args, evaluator_factory=lambda _path: FakeEvaluator()
    )
    assert loaded.session_id == SESSION
    assert observed == digest
    assert isinstance(evaluator, FakeEvaluator)

    args.confirm = ""
    with pytest.raises(capture.GravityReadOnlyCaptureError, match="confirm"):
        capture.validate_preflight(
            args, evaluator_factory=lambda _path: FakeEvaluator()
        )
    args.confirm = capture.READONLY_GATE
    args.expected_session_id = "different"
    with pytest.raises(capture.GravityReadOnlyCaptureError, match="session"):
        capture.validate_preflight(
            args, evaluator_factory=lambda _path: FakeEvaluator()
        )

    bad_model = tmp_path / "model.xml"
    bad_model.write_text("not the frozen model", encoding="utf-8")
    args.expected_session_id = SESSION
    args.model_path = bad_model
    with pytest.raises(capture.GravityReadOnlyCaptureError, match="model SHA"):
        capture.validate_preflight(
            args, evaluator_factory=lambda _path: FakeEvaluator()
        )


def test_capture_source_has_only_subscriptions_and_no_command_transport() -> None:
    source = SCRIPT.read_text(encoding="utf-8")
    assert "create_publisher" not in source
    assert "create_client" not in source
    assert "create_subscription" in source
    assert "import socket" not in source
    assert "python-can" not in source.lower()


def test_independent_dynamics_only_xml_removes_mesh_payload(tmp_path: Path) -> None:
    source = tmp_path / "model.xml"
    source.write_text(
        """<mujoco><size njmax="4000"/><visual/>
        <asset><mesh name="large" file="large.stl"/></asset>
        <contact/><worldbody><body name="link"><joint name="J1"/>
        <inertial mass="1" pos="0 0 0" diaginertia="1 1 1"/>
        <geom name="visual" type="mesh" mesh="large"/></body></worldbody>
        </mujoco>""",
        encoding="utf-8",
    )
    derived = capture.independent_dynamics_only_xml(source)
    assert "<joint name=\"J1\"" in derived
    assert "<inertial" in derived
    for forbidden in ("<asset", "<mesh", "<geom", "<contact", "<visual", "<size"):
        assert forbidden not in derived

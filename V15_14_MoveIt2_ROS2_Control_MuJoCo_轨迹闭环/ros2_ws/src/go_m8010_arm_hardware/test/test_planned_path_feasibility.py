import ast
import copy
from pathlib import Path

import pytest

from go_m8010_arm_hardware import planned_path_feasibility as subject


LIMITS = tuple((-2.0, 2.0) for _ in range(6))


def request_document():
    segment = subject.SegmentDescriptor(
        trajectory_sha256="0" * 64,
        start_rad=(0.0,) * 6,
        target_rad=(0.1, 0.0, 0.0, 0.0, 0.0, 0.0),
        joint_limits_rad=LIMITS,
        duration_s=0.04,
        maximum_sample_period_s=0.01,
        actual_sample_period_s=0.01,
        interval_count=4,
    )
    segment = subject.SegmentDescriptor(
        **{
            **segment.__dict__,
            "trajectory_sha256": subject._segment_semantic_sha256(segment),
        }
    )
    recipe_sha = subject._canonical_sha256({
        "schema": subject.RECIPE_SCHEMA,
        "start_rad": subject._canonical_vector(segment.start_rad),
        "target_rad": subject._canonical_vector(segment.target_rad),
        "joint_limits_rad": subject._canonical_limits(LIMITS),
        "profile": {
            "kind": subject.SEGMENTED_PROFILE,
            "duration_s": subject._canonical_float(0.04),
            "maximum_sample_period_s": subject._canonical_float(0.01),
        },
        "segment_sha256": [segment.trajectory_sha256],
    })
    value = {
        "schema": subject.REQUEST_SCHEMA,
        "source": "arm_control_gui",
        "source_instance_id": "1" * 32,
        "sequence": 7,
        "source_monotonic_ns": 1_000_000_000,
        "session_id": "session-a",
        "state_instance_id": "2" * 32,
        "model_sha256": subject.PRODUCTION_MODEL_SHA256,
        "gravity_config_sha256": subject.GRAVITY_CONFIG_SHA256,
        "thermal_config_sha256": subject.THERMAL_CONFIG_SHA256,
        "trajectory_sha256": recipe_sha,
        "trajectory": {
            "schema": subject.RECIPE_SCHEMA,
            "trajectory_sha256": recipe_sha,
            "start_rad": list(segment.start_rad),
            "target_rad": list(segment.target_rad),
            "joint_limits_rad": [list(pair) for pair in LIMITS],
            "profile": {
                "kind": subject.SEGMENTED_PROFILE,
                "duration_s": 0.04,
                "maximum_sample_period_s": 0.01,
                "actual_sample_period_s": 0.01,
            },
            "segments": [{
                "schema": subject.TRAJECTORY_SCHEMA,
                "trajectory_sha256": segment.trajectory_sha256,
                "start_rad": list(segment.start_rad),
                "target_rad": list(segment.target_rad),
                "joint_limits_rad": [list(pair) for pair in LIMITS],
                "profile": {
                    "kind": subject.QUINTIC_PROFILE,
                    "duration_s": 0.04,
                    "maximum_sample_period_s": 0.01,
                    "actual_sample_period_s": 0.01,
                    "interval_count": 4,
                },
            }],
        },
    }
    value["request_sha256"] = subject.request_identity_sha256(value)
    return value


def test_request_recomputes_segment_recipe_and_request_hashes():
    document = request_document()
    request = subject.parse_planned_path_request(
        document, now_monotonic_ns=1_000_000_010
    )
    assert request.trajectory_sha256 == document["trajectory_sha256"]
    assert request.sample_count == 5
    assert len(tuple(request.iter_joint_samples())) == 5

    tampered = copy.deepcopy(document)
    tampered["trajectory"]["segments"][0]["target_rad"][0] = 0.2
    with pytest.raises(subject.PlannedPathError, match="SHA256|final target"):
        subject.parse_planned_path_request(
            tampered, now_monotonic_ns=1_000_000_010
        )

    wrong_thermal_config = copy.deepcopy(document)
    wrong_thermal_config["thermal_config_sha256"] = "0" * 64
    wrong_thermal_config["request_sha256"] = subject.request_identity_sha256(
        wrong_thermal_config
    )
    with pytest.raises(subject.PlannedPathError, match="thermal config"):
        subject.parse_planned_path_request(wrong_thermal_config)


def test_request_rejects_stale_timestamp_and_nonexact_fields():
    document = request_document()
    with pytest.raises(subject.PlannedPathError, match="stale"):
        subject.parse_planned_path_request(
            document,
            now_monotonic_ns=(
                document["source_monotonic_ns"]
                + subject.REQUEST_MAXIMUM_AGE_NS
                + 1
            ),
        )
    extra = copy.deepcopy(document)
    extra["unexpected"] = True
    with pytest.raises(subject.PlannedPathError, match="fields"):
        subject.parse_planned_path_request(extra)


class FakeAnchor:
    def model_q_from_actual(self, q_actual):
        return tuple(value + 0.2 for value in q_actual)


class FakeEvaluator:
    def __init__(self):
        self.inverse_samples = []

    def evaluate(self, _model_q):
        return (6.33, 12.66, -6.33, 3.165, -1.0, 0.5)

    def evaluate_inverse(self, model_q, dq, ddq):
        self.inverse_samples.append((tuple(model_q), tuple(dq), tuple(ddq)))
        return tuple(
            gravity + acceleration
            for gravity, acceleration in zip(
                self.evaluate(model_q), ddq
            )
        )


def evaluated_envelope():
    request = subject.parse_planned_path_request(request_document())
    evaluator = FakeEvaluator()
    envelope = subject.evaluate_path_load_envelope(
        request,
        anchor=FakeAnchor(),
        evaluator=evaluator,
        j6_joint_to_rotor_scale=1.0,
    )
    return envelope, evaluator


def test_every_sample_uses_bias_and_inverse_dynamics_with_j2_split():
    envelope, evaluator = evaluated_envelope()
    assert envelope.evaluated_sample_count == envelope.request.sample_count == 5
    assert len(evaluator.inverse_samples) == 5
    assert any(any(abs(value) > 0.0 for value in sample[2])
               for sample in evaluator.inverse_samples)
    gravity = envelope.maximum_abs_gravity_rotor_torque_nm
    predicted = envelope.maximum_abs_predicted_rotor_torque_nm
    assert envelope.maximum_abs_gravity_joint_torque_nm["J2"] == pytest.approx(
        12.66
    )
    assert gravity["J2A"] == pytest.approx(1.0)
    assert gravity["J2B"] == pytest.approx(1.0)
    assert predicted["J1"] > gravity["J1"]
    assert predicted["J6"] == pytest.approx(0.5)


def proof(envelope, **overrides):
    values = dict(
        source_instance_id="a" * 32,
        sequence=9,
        source_monotonic_ns=2_000_000_000,
        model_sha256=subject.PRODUCTION_MODEL_SHA256,
        gravity_config_sha256=subject.GRAVITY_CONFIG_SHA256,
        thermal_config_sha256=subject.THERMAL_CONFIG_SHA256,
        continuous_config_authoritative=True,
        continuous_hardware_authoritative=True,
        continuous_rotor_limits_nm={name: 100.0 for name in subject.MOTOR_NAMES},
        short_peak_rotor_limits_nm={name: 200.0 for name in subject.MOTOR_NAMES},
        temperature_limits_authoritative=True,
        minimum_thermal_margin_c=5.0,
    )
    values.update(overrides)
    return subject.build_planned_path_proof(envelope, **values)


def test_authoritative_positive_margins_pass_and_are_exactly_bound():
    envelope, _evaluator = evaluated_envelope()
    result = proof(envelope)
    assert result["result"] == "PASS"
    assert result["load_feasibility"] == "PASS"
    assert result["thermal_feasibility"] == "PASS"
    assert result["current_temperature_margin_result"] == "PASS"
    assert result["trajectory_sha256"] == envelope.request.trajectory_sha256
    assert result["evaluated_sample_count"] == result["sample_count"] == 5
    assert "MJ_INVERSE" in result["load_evaluation_basis"]
    assert "NO_HEAT_RISE_MODEL" in result["thermal_evaluation_basis"]
    assert result[
        "minimum_predicted_continuous_rotor_torque_margin_nm"
    ] > 0.0


def test_false_continuous_authority_is_explicitly_blocked_not_passed():
    envelope, _evaluator = evaluated_envelope()
    result = proof(envelope, continuous_config_authoritative=False)
    assert result["result"] == "BLOCKED"
    assert result["load_feasibility"] == "BLOCKED"
    assert result["thermal_feasibility"] == "BLOCKED"
    assert result["blocker_code"] == "CONTINUOUS_TORQUE_CONFIG_AUTHORITY_FALSE"


def test_parsed_request_with_failed_evaluation_is_blocked_not_not_evaluated():
    request = subject.parse_planned_path_request(request_document())
    result = proof(
        None,
        request=request,
        evaluation_blocker="PLANNED_PATH_EVALUATION_FAILED:test",
    )
    assert result["trajectory_sha256"] == request.trajectory_sha256
    assert result["result"] == "BLOCKED"
    assert result["load_feasibility"] == "BLOCKED"
    assert result["blocker_code"].startswith("PLANNED_PATH_EVALUATION_FAILED")


def test_zero_load_or_thermal_margin_fails_closed():
    envelope, _evaluator = evaluated_envelope()
    continuous = {name: 100.0 for name in subject.MOTOR_NAMES}
    limiting_motor = max(
        subject.MOTOR_NAMES,
        key=lambda name: envelope.maximum_abs_gravity_rotor_torque_nm[name],
    )
    continuous[limiting_motor] = envelope.maximum_abs_gravity_rotor_torque_nm[
        limiting_motor
    ]
    zero_load = proof(envelope, continuous_rotor_limits_nm=continuous)
    assert zero_load["load_feasibility"] == "FAIL"
    assert zero_load["minimum_rotor_torque_margin_nm"] == pytest.approx(0.0)
    zero_thermal = proof(envelope, minimum_thermal_margin_c=0.0)
    assert zero_thermal["thermal_feasibility"] == "FAIL"


def test_dynamic_load_above_continuous_stays_thermal_blocked_without_duty_model():
    envelope, _evaluator = evaluated_envelope()
    continuous = {name: 100.0 for name in subject.MOTOR_NAMES}
    motor = next(
        name for name in subject.MOTOR_NAMES
        if envelope.maximum_abs_predicted_rotor_torque_nm[name]
        > envelope.maximum_abs_gravity_rotor_torque_nm[name]
    )
    continuous[motor] = 0.5 * (
        envelope.maximum_abs_predicted_rotor_torque_nm[motor]
        + envelope.maximum_abs_gravity_rotor_torque_nm[motor]
    )
    result = proof(envelope, continuous_rotor_limits_nm=continuous)
    assert result["load_feasibility"] == "PASS"
    assert result["current_temperature_margin_result"] == "PASS"
    assert result["thermal_feasibility"] == "BLOCKED"
    assert result[
        "minimum_predicted_continuous_rotor_torque_margin_nm"
    ] < 0.0
    assert "DUTY_MODEL_AUTHORITY_MISSING" in result["blocker_code"]


def hardware_state(*, continuous=True, temperature=40.0):
    return {
        "session_id": "session-a",
        "state_instance_id": "2" * 32,
        "per_motor": {
            name: {
                "fresh": True,
                "communication_ok": True,
                "merror": 0,
                "temperature_c": temperature,
                "thermal_metadata_status": "OBSERVED",
                "thermal_config_sha256": subject.THERMAL_CONFIG_SHA256,
                "thermal_fault_latched": False,
                "thermal_state": "NORMAL",
                "continuous_rating_authoritative": continuous,
            }
            for name in subject.MOTOR_NAMES
        },
    }


def test_temperature_authority_requires_all_seven_fresh_exact_config_samples():
    observed = subject.temperature_observation(
        hardware_state(),
        session_id="session-a",
        state_instance_id="2" * 32,
        derating_start_c=55.0,
    )
    assert observed == (True, 15.0, True, "")
    invalid = hardware_state()
    invalid["per_motor"]["J2B"]["thermal_config_sha256"] = "0" * 64
    observed = subject.temperature_observation(
        invalid,
        session_id="session-a",
        state_instance_id="2" * 32,
        derating_start_c=55.0,
    )
    assert observed[0] is False
    assert observed[2] is False
    assert observed[3] == "TEMPERATURE_J2B_AUTHORITY_INVALID"


def test_pose_authority_rejects_stale_motor_inside_fresh_outer_snapshot():
    now_ns = 5_000_000_000
    state = {
        "schema": "go-m8010-hardware-state/1.1",
        "session_id": "session-a",
        "state_instance_id": "2" * 32,
        "source_monotonic_ns": now_ns - 1_000_000,
        "joint_names": list(subject.HARDWARE_STATE_JOINT_NAMES),
        "healthy": True,
        "per_motor": {
            name: {
                "fresh": True,
                "communication_ok": True,
                "merror": 0,
                "reference_captured": True,
                "q_joint_rad": 0.0,
                "dq_joint_rad_s": 0.0,
                "feedback_source_monotonic_ns": now_ns - 2_000_000,
                "feedback_receipt_monotonic_ns": now_ns - 1_500_000,
                "last_valid_feedback_monotonic_ns": now_ns - 2_000_000,
            }
            for name in subject.MOTOR_NAMES
        },
    }
    assert subject.hardware_pose_feedback_blocker(
        state,
        now_monotonic_ns=now_ns,
        maximum_age_ns=250_000_000,
        session_id="session-a",
        state_instance_id="2" * 32,
    ) == ""

    planning_names = copy.deepcopy(state)
    planning_names["joint_names"] = list(subject.JOINT_NAMES)
    assert subject.hardware_pose_feedback_blocker(
        planning_names,
        now_monotonic_ns=now_ns,
        maximum_age_ns=250_000_000,
        session_id="session-a",
        state_instance_id="2" * 32,
    ) == "HARDWARE_POSE_STATE_IDENTITY_INVALID"

    stale = copy.deepcopy(state)
    stale["per_motor"]["J2B"]["fresh"] = False
    stale["healthy"] = False
    assert "NOT_HEALTHY" in subject.hardware_pose_feedback_blocker(
        stale,
        now_monotonic_ns=now_ns,
        maximum_age_ns=250_000_000,
        session_id="session-a",
        state_instance_id="2" * 32,
    )

    stale_last_valid = copy.deepcopy(state)
    stale_last_valid["per_motor"]["J5"][
        "last_valid_feedback_monotonic_ns"
    ] = now_ns - 250_000_001
    assert "J5_LAST_VALID_FEEDBACK_MONOTONIC_NS_STALE" in (
        subject.hardware_pose_feedback_blocker(
            stale_last_valid,
            now_monotonic_ns=now_ns,
            maximum_age_ns=250_000_000,
            session_id="session-a",
            state_instance_id="2" * 32,
        )
    )


def test_parse_and_envelope_honor_latest_generation_cancellation():
    calls = 0

    def cancelled():
        nonlocal calls
        calls += 1
        return calls >= 3

    with pytest.raises(subject.PlannedPathError, match="cancelled"):
        subject.parse_planned_path_request(
            request_document(), cancellation_requested=cancelled
        )

    request = subject.parse_planned_path_request(request_document())
    with pytest.raises(subject.PlannedPathError, match="cancelled"):
        subject.evaluate_path_load_envelope(
            request,
            anchor=FakeAnchor(),
            evaluator=FakeEvaluator(),
            j6_joint_to_rotor_scale=1.0,
            cancellation_requested=lambda: True,
        )


def test_gravity_subscription_callback_never_parses_or_evaluates_the_path():
    node_path = (
        Path(__file__).resolve().parents[1]
        / "go_m8010_arm_hardware"
        / "whole_arm_gravity_node.py"
    )
    tree = ast.parse(node_path.read_text(encoding="utf-8"))
    node_class = next(
        item for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "WholeArmGravityNode"
    )
    callback = next(
        item for item in node_class.body
        if isinstance(item, ast.FunctionDef)
        and item.name == "_on_planned_path_request"
    )
    callback_calls = {
        item.func.id
        for item in ast.walk(callback)
        if isinstance(item, ast.Call) and isinstance(item.func, ast.Name)
    }
    assert "parse_planned_path_request" not in callback_calls
    assert "evaluate_path_load_envelope" not in callback_calls
    source = ast.get_source_segment(
        node_path.read_text(encoding="utf-8"), callback
    )
    assert "_planned_executor.submit" in source
    assert "self._planned_cancel_event.set()" in source
    assert "json.loads" not in source
    assert ".encode(" not in source
    assert "parse_planned_path_request" not in source

    worker = next(
        item for item in node_class.body
        if isinstance(item, ast.FunctionDef)
        and item.name == "_decode_parse_and_evaluate_planned_path"
    )
    worker_source = ast.get_source_segment(
        node_path.read_text(encoding="utf-8"), worker
    )
    assert "json.loads(serialized)" in worker_source
    assert "now_monotonic_ns=time.monotonic_ns()" in worker_source
    assert "cancellation_requested=cancel_event.is_set" in worker_source
    assert "planned-path request replayed" in worker_source
    assert "MAXIMUM_PLANNED_REQUEST_SOURCES" in worker_source

    evaluator = next(
        item for item in node_class.body
        if isinstance(item, ast.FunctionDef)
        and item.name == "_evaluate_planned_path"
    )
    evaluator_source = ast.get_source_segment(
        node_path.read_text(encoding="utf-8"), evaluator
    )
    assert "cancellation_requested=cancel_event.is_set" in evaluator_source

    destroy = next(
        item for item in node_class.body
        if isinstance(item, ast.FunctionDef) and item.name == "destroy_node"
    )
    destroy_source = ast.get_source_segment(
        node_path.read_text(encoding="utf-8"), destroy
    )
    assert "self._planned_cancel_event.set()" in destroy_source
    assert "cancel_futures=True" in destroy_source

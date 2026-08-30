from dataclasses import FrozenInstanceError, replace
import math
from pathlib import Path

import pytest

from go_m8010_arm_gui.workflow_contract import (
    ContractViolation,
    PlanToken,
    PreviewChecks,
    QuinticProfile,
    RealSubmitRejected,
    TrajectoryPlan,
    TrajectoryRecipe,
    TrajectorySample,
    WorkflowState,
    finite_joint_vector,
    generate_quintic_trajectory,
    generate_segmented_quintic_recipe,
    joint_limits_sha256,
    joint_vector_sha256,
    planned_trajectory_feasibility_request,
    quintic_duration_for_limits,
    trajectory_command_descriptor,
    trajectory_plan_manifest,
    trajectory_sample_index_at,
    validated_joint_limits,
)


MODEL_HASH = "a" * 64
SECOND_MODEL_HASH = "b" * 64
LIMITS = tuple((-2.0, 2.0) for _ in range(6))
ACTUAL = (0.0, 0.1, -0.2, 0.3, -0.4, 0.5)
TARGET = (0.2, -0.1, 0.0, 0.1, -0.2, 0.3)


def plan(start=ACTUAL, target=TARGET, limits=LIMITS):
    return generate_quintic_trajectory(
        start,
        target,
        limits,
        duration_s=2.0,
        maximum_sample_period_s=0.1,
    )


def previewed_state(*, actual=ACTUAL, target=TARGET, nonce="preview-1"):
    state = WorkflowState.initialize(actual, LIMITS, MODEL_HASH)
    state = state.change_plan_target(target)
    trajectory = plan(actual, target)
    return state.accept_successful_preview(
        trajectory,
        PreviewChecks.successful(),
        created_monotonic_ns=123456789,
        nonce=nonce,
    )


def test_module_is_pure_python_without_ros_or_qt_imports():
    source = (
        Path(__file__).parents[1]
        / "go_m8010_arm_gui"
        / "workflow_contract.py"
    ).read_text(encoding="utf-8")
    assert "import rclpy" not in source
    assert "from rclpy" not in source
    assert "PySide" not in source
    assert "QtCore" not in source


@pytest.mark.parametrize(
    "values",
    [
        [0.0] * 5,
        [0.0] * 7,
        [0.0] * 5 + [math.nan],
        [0.0] * 5 + [math.inf],
        [0.0] * 5 + [True],
        [0.0] * 5 + ["0.0"],
        "0,0,0,0,0,0",
    ],
)
def test_joint_vectors_require_exactly_six_strict_finite_numbers(values):
    with pytest.raises(ContractViolation):
        finite_joint_vector(values, "q")


@pytest.mark.parametrize(
    "limits",
    [
        LIMITS[:5],
        LIMITS + ((-1.0, 1.0),),
        LIMITS[:5] + ((0.0, 0.0),),
        LIMITS[:5] + ((1.0, -1.0),),
        LIMITS[:5] + ((math.nan, 1.0),),
        LIMITS[:5] + ((False, 1.0),),
        LIMITS[:5] + ((-1.0,),),
    ],
)
def test_joint_limits_require_six_finite_strictly_ordered_pairs(limits):
    with pytest.raises(ContractViolation):
        validated_joint_limits(limits)


def test_quintic_generation_is_deterministic_and_rest_to_rest():
    first = plan()
    second = plan()
    assert first == second
    assert first.sha256 == second.sha256
    assert len(first.samples) == 21
    assert first.samples[0].time_s == 0.0
    assert first.samples[-1].time_s == 2.0
    assert first.samples[0].q_rad == ACTUAL
    assert first.samples[-1].q_rad == TARGET
    assert first.samples[0].dq_rad_s == (0.0,) * 6
    assert first.samples[-1].dq_rad_s == (0.0,) * 6
    assert first.samples[0].ddq_rad_s2 == (0.0,) * 6
    assert first.samples[-1].ddq_rad_s2 == (0.0,) * 6
    assert all(
        later.time_s > earlier.time_s
        for earlier, later in zip(first.samples, first.samples[1:])
    )
    midpoint = first.samples[len(first.samples) // 2]
    assert midpoint.q_rad == pytest.approx(
        tuple((source + wanted) / 2.0 for source, wanted in zip(ACTUAL, TARGET))
    )


def test_quintic_profile_rejects_invalid_duration_period_or_unbounded_samples():
    for duration, period in (
        (0.0, 0.1),
        (-1.0, 0.1),
        (1.0, 0.0),
        (1.0, -0.1),
        (math.nan, 0.1),
        (1.0, math.inf),
        (1_000_001.0, 1.0),
    ):
        with pytest.raises(ContractViolation):
            QuinticProfile(duration, period)


def test_trajectory_rejects_out_of_limit_start_or_target():
    with pytest.raises(ContractViolation):
        plan(start=(2.1,) + ACTUAL[1:])
    with pytest.raises(ContractViolation):
        plan(target=TARGET[:5] + (2.1,))


def test_trajectory_rejects_nonmonotonic_or_nondeterministic_sample_time():
    valid = plan()
    samples = list(valid.samples)
    samples[1] = replace(samples[1], time_s=samples[0].time_s)
    with pytest.raises(ContractViolation, match="strictly increase"):
        replace(valid, samples=tuple(samples))

    samples = list(valid.samples)
    samples[1] = replace(samples[1], time_s=samples[1].time_s + 1.0e-6)
    with pytest.raises(ContractViolation, match="time grid"):
        replace(valid, samples=tuple(samples))


def test_trajectory_rejects_nonfinite_or_nonquintic_sample_data():
    valid = plan()
    with pytest.raises(ContractViolation, match="finite"):
        TrajectorySample(
            0.0,
            (math.nan,) + ACTUAL[1:],
            (0.0,) * 6,
            (0.0,) * 6,
        )

    samples = list(valid.samples)
    changed = list(samples[1].q_rad)
    changed[0] += 1.0e-4
    samples[1] = replace(samples[1], q_rad=tuple(changed))
    with pytest.raises(ContractViolation, match="declared quintic"):
        replace(valid, samples=tuple(samples))


def test_trajectory_constructor_requires_full_sample_count_and_type():
    valid = plan()
    with pytest.raises(ContractViolation, match="sample count"):
        replace(valid, samples=valid.samples[:-1])
    with pytest.raises(ContractViolation, match="invalid item"):
        TrajectoryPlan(
            valid.start_rad,
            valid.target_rad,
            valid.joint_limits_rad,
            valid.profile,
            tuple([object()] * len(valid.samples)),
        )


def test_four_states_are_deeply_frozen_and_never_alias_input_lists():
    source = list(ACTUAL)
    state = WorkflowState.initialize(source, [list(pair) for pair in LIMITS], MODEL_HASH)
    source[0] = 1.5
    assert state.q_actual == ACTUAL
    assert state.q_plan_target == ACTUAL
    assert state.q_plan_trajectory is None
    assert state.q_hardware_command is None
    assert isinstance(state.q_actual, tuple)
    assert isinstance(state.joint_limits_rad, tuple)
    with pytest.raises(FrozenInstanceError):
        state.q_actual = TARGET
    with pytest.raises(TypeError):
        state.q_actual[0] = 1.0


def test_actual_target_trajectory_and_hardware_command_are_isolated():
    initial = WorkflowState.initialize(ACTUAL, LIMITS, MODEL_HASH)
    edited = initial.change_plan_target(TARGET)
    assert edited.q_actual == ACTUAL
    assert edited.q_plan_target == TARGET
    assert edited.q_plan_trajectory is None
    assert edited.q_hardware_command is None

    approved = edited.accept_successful_preview(
        plan(),
        PreviewChecks.successful(),
        created_monotonic_ns=100,
        nonce="n1",
    )
    assert approved.q_actual == ACTUAL
    assert approved.q_plan_target == TARGET
    assert approved.q_plan_trajectory is not None
    assert approved.q_hardware_command is None

    moved_actual = tuple(value + 0.001 for value in ACTUAL)
    observed = approved.update_actual(moved_actual)
    assert observed.q_actual == moved_actual
    assert observed.q_plan_target == TARGET
    assert observed.q_plan_trajectory == approved.q_plan_trajectory
    assert observed.q_hardware_command is None


@pytest.mark.parametrize(
    "failed_field",
    [
        "trajectory_pass",
        "limits_pass",
        "collision_pass",
        "gravity_pass",
        "planned_load_thermal_pass",
        "thermal_pass",
        "feedback_fresh",
        "communication_pass",
    ],
)
def test_every_preview_check_is_required_before_token_issuance(failed_field):
    state = WorkflowState.initialize(ACTUAL, LIMITS, MODEL_HASH).change_plan_target(TARGET)
    values = PreviewChecks.successful().as_dict()
    values[failed_field] = False
    checks = PreviewChecks(**values)
    with pytest.raises(ContractViolation, match="complete successful preview"):
        state.accept_successful_preview(
            plan(), checks, created_monotonic_ns=1, nonce="failed"
        )


def test_preview_checks_reject_non_boolean_fields():
    values = PreviewChecks.successful().as_dict()
    values["thermal_pass"] = 1
    with pytest.raises(ContractViolation, match="must be boolean"):
        PreviewChecks(**values)


def test_plan_token_contains_required_authority_and_is_immutable():
    state = previewed_state()
    token = state.current_plan_token
    assert token is not None
    assert token.source_actual_rad == ACTUAL
    assert token.target_rad == TARGET
    assert token.trajectory_sha256 == state.q_plan_trajectory.sha256
    assert token.model_sha256 == MODEL_HASH
    assert token.limits_sha256 == joint_limits_sha256(LIMITS)
    assert token.created_monotonic_ns == 123456789
    assert token.nonce == "preview-1"
    assert len(token.token_id) == 64
    with pytest.raises(FrozenInstanceError):
        token.nonce = "changed"


def test_plan_token_cannot_be_created_through_public_constructor():
    with pytest.raises(TypeError):
        PlanToken()


@pytest.mark.parametrize(
    "created,nonce",
    [(0, "n"), (-1, "n"), (True, "n"), (1, ""), (1, None)],
)
def test_token_requires_strict_positive_timestamp_and_nonce(created, nonce):
    state = WorkflowState.initialize(ACTUAL, LIMITS, MODEL_HASH).change_plan_target(TARGET)
    with pytest.raises(ContractViolation):
        state.accept_successful_preview(
            plan(),
            PreviewChecks.successful(),
            created_monotonic_ns=created,
            nonce=nonce,
        )


def test_preview_must_match_current_actual_target_and_limits():
    state = WorkflowState.initialize(ACTUAL, LIMITS, MODEL_HASH).change_plan_target(TARGET)
    checks = PreviewChecks.successful()
    with pytest.raises(ContractViolation, match="source"):
        state.accept_successful_preview(
            plan(tuple(value + 0.1 for value in ACTUAL), TARGET),
            checks,
            created_monotonic_ns=1,
            nonce="wrong-start",
        )
    with pytest.raises(ContractViolation, match="target"):
        state.accept_successful_preview(
            plan(ACTUAL, tuple(value + 0.1 for value in TARGET)),
            checks,
            created_monotonic_ns=1,
            nonce="wrong-target",
        )
    narrower = tuple((-1.5, 1.5) for _ in range(6))
    with pytest.raises(ContractViolation, match="limits"):
        state.accept_successful_preview(
            plan(ACTUAL, TARGET, narrower),
            checks,
            created_monotonic_ns=1,
            nonce="wrong-limits",
        )


def test_candidate_change_invalidates_old_token_without_changing_hardware_command():
    approved = previewed_state()
    token = approved.current_plan_token
    assert token is not None
    submitted = approved.explicit_real_submit(token.token_id)
    assert submitted.q_hardware_command == TARGET

    new_target = (0.3, -0.2, 0.1, 0.0, -0.1, 0.2)
    edited = submitted.change_plan_target(new_target)
    assert edited.candidate_revision == submitted.candidate_revision + 1
    assert edited.current_plan_token is None
    assert edited.q_plan_trajectory is None
    assert edited.q_actual == submitted.q_actual
    assert edited.q_hardware_command == TARGET
    with pytest.raises(RealSubmitRejected):
        edited.explicit_real_submit(token.token_id)


def test_preview_success_never_sets_hardware_command_and_submit_is_explicit():
    approved = previewed_state()
    token = approved.current_plan_token
    assert token is not None
    assert approved.q_hardware_command is None
    assert not approved.can_submit_to_real("f" * 64)
    with pytest.raises(RealSubmitRejected):
        approved.explicit_real_submit("f" * 64)
    assert approved.q_hardware_command is None

    submitted = approved.explicit_real_submit(token.token_id)
    assert submitted.q_hardware_command == TARGET
    assert submitted.submitted_token_id == token.token_id
    assert submitted.current_plan_token is None
    assert not submitted.can_submit_to_real(token.token_id)
    with pytest.raises(RealSubmitRejected):
        submitted.explicit_real_submit(token.token_id)


def test_actual_pose_drift_is_checked_again_at_explicit_submit():
    approved = previewed_state()
    token = approved.current_plan_token
    assert token is not None
    within = approved.update_actual(
        tuple(value + 0.001 for value in approved.q_actual)
    )
    assert within.can_submit_to_real(token.token_id, actual_tolerance_rad=0.001)
    assert not within.can_submit_to_real(token.token_id, actual_tolerance_rad=0.0009)
    with pytest.raises(RealSubmitRejected):
        within.explicit_real_submit(token.token_id, actual_tolerance_rad=0.0009)


def test_invalid_actual_submit_tolerance_is_rejected():
    approved = previewed_state()
    token = approved.current_plan_token
    assert token is not None
    for tolerance in (-0.1, math.nan, math.inf, True, "0.1"):
        with pytest.raises(ContractViolation):
            approved.can_submit_to_real(
                token.token_id, actual_tolerance_rad=tolerance
            )


def test_model_or_limits_authority_change_invalidates_current_token():
    approved = previewed_state()
    token = approved.current_plan_token
    assert token is not None
    changed = approved.replace_authority(
        joint_limits_rad=LIMITS, model_sha256=SECOND_MODEL_HASH
    )
    assert changed.current_plan_token is None
    assert changed.q_plan_trajectory is None
    assert changed.candidate_revision == approved.candidate_revision + 1
    assert not changed.can_submit_to_real(token.token_id)


@pytest.mark.parametrize(
    "bad_hash",
    ["a" * 63, "A" * 64, "g" * 64, "", None],
)
def test_model_hash_is_strict_lowercase_sha256(bad_hash):
    with pytest.raises(ContractViolation):
        WorkflowState.initialize(ACTUAL, LIMITS, bad_hash)


def test_same_candidate_is_a_noop_and_does_not_invalidate_current_token():
    approved = previewed_state()
    unchanged = approved.change_plan_target(list(TARGET))
    assert unchanged is approved
    assert unchanged.current_plan_token is approved.current_plan_token


def test_plan_token_carries_session_pose_hashes_profile_and_gravity_authority():
    state = WorkflowState.initialize(
        ACTUAL,
        LIMITS,
        MODEL_HASH,
        session_id="session-v15-31a",
        state_instance_id="0123456789abcdef0123456789abcdef",
        gravity_config_sha256="3" * 64,
    ).change_plan_target(TARGET)
    plan = generate_quintic_trajectory(
        ACTUAL, TARGET, LIMITS, duration_s=2.0
    )
    approved = state.accept_successful_preview(
        plan,
        PreviewChecks.successful(),
        created_monotonic_ns=123,
        nonce="unit-test",
    )
    token = approved.current_plan_token
    assert token is not None
    assert token.session_id == "session-v15-31a"
    assert token.state_instance_id == "0123456789abcdef0123456789abcdef"
    assert token.source_actual_sha256 == joint_vector_sha256(ACTUAL)
    assert token.target_sha256 == joint_vector_sha256(TARGET)
    assert token.trajectory_profile == "quintic-rest-to-rest-v1"
    assert token.gravity_config_sha256 == "3" * 64


def test_session_change_and_excess_actual_drift_invalidate_plan_token():
    approved = previewed_state()
    changed_session = approved.replace_authority(
        joint_limits_rad=LIMITS,
        model_sha256=MODEL_HASH,
        session_id="new-session",
        state_instance_id="new-state-instance",
    )
    assert changed_session.current_plan_token is None
    drifted = approved.update_actual(
        tuple(value + 0.01 for value in ACTUAL),
        invalidation_tolerance_rad=0.001,
    )
    assert drifted.current_plan_token is None
    assert drifted.q_plan_trajectory == approved.q_plan_trajectory


def test_quintic_duration_obeys_requested_velocity_and_acceleration_bounds():
    start = (0.0,) * 6
    target = (1.0, 0.0, 0.0, 0.0, 0.0, 0.0)
    duration = quintic_duration_for_limits(
        start,
        target,
        maximum_velocity_rad_s=0.5,
        maximum_acceleration_rad_s2=1.0,
    )
    plan_value = generate_quintic_trajectory(
        start, target, LIMITS, duration_s=duration
    )
    assert max(abs(sample.dq_rad_s[0]) for sample in plan_value.samples) <= 0.5
    assert max(abs(sample.ddq_rad_s2[0]) for sample in plan_value.samples) <= 1.0


def test_explicit_preview_invalidation_never_mutates_hardware_command():
    approved = previewed_state()
    token = approved.current_plan_token
    assert token is not None
    submitted = approved.explicit_real_submit(token.token_id)
    invalid = submitted.invalidate_preview(clear_trajectory=False)
    assert invalid.current_plan_token is None
    assert invalid.q_plan_trajectory == submitted.q_plan_trajectory
    assert invalid.q_hardware_command == TARGET


def test_planned_trajectory_can_be_animated_without_real_submit_authority():
    state = WorkflowState.initialize(ACTUAL, LIMITS, MODEL_HASH).change_plan_target(TARGET)
    planned = state.set_plan_trajectory(plan())
    assert planned.q_plan_trajectory is not None
    assert planned.current_plan_token is None
    assert planned.q_hardware_command is None


def test_preview_acceptance_allows_only_bounded_actual_drift_from_plan_start():
    state = WorkflowState.initialize(ACTUAL, LIMITS, MODEL_HASH).change_plan_target(TARGET)
    trajectory = plan()
    drifted = state.update_actual(tuple(value + 0.001 for value in ACTUAL))
    approved = drifted.accept_successful_preview(
        trajectory,
        PreviewChecks.successful(),
        created_monotonic_ns=456,
        nonce="bounded-drift",
        actual_tolerance_rad=0.001,
    )
    assert approved.current_plan_token.source_actual_rad == ACTUAL
    with pytest.raises(ContractViolation, match="no longer matches"):
        drifted.accept_successful_preview(
            trajectory,
            PreviewChecks.successful(),
            created_monotonic_ns=457,
            nonce="too-small-tolerance",
            actual_tolerance_rad=0.0009,
        )


def test_segmented_recipe_is_ordered_bounded_and_uses_exact_quintic_segments():
    start = (0.0,) * 6
    target = (0.60, 0.0, -0.30, 0.0, 0.0, 0.0)
    recipe = generate_segmented_quintic_recipe(
        start,
        target,
        LIMITS,
        maximum_velocity_rad_s=0.5,
        maximum_acceleration_rad_s2=1.0,
        maximum_segment_delta_rad=0.25,
        maximum_sample_period_s=0.01,
    )
    assert isinstance(recipe, TrajectoryRecipe)
    assert len(recipe.segments) == 5
    assert recipe.start_rad == start
    assert recipe.target_rad == target
    assert recipe.segments[0].target_rad[0] == pytest.approx(0.25)
    assert recipe.segments[1].target_rad[0] == pytest.approx(0.50)
    assert recipe.segments[2].target_rad[0] == pytest.approx(0.60)
    assert recipe.segments[3].target_rad[2] == pytest.approx(-0.25)
    assert recipe.segments[4].target_rad[2] == pytest.approx(-0.30)
    assert recipe.samples[0].q_rad == start
    assert recipe.samples[-1].q_rad == target
    assert all(
        later.time_s > earlier.time_s
        for earlier, later in zip(recipe.samples, recipe.samples[1:])
    )
    for segment in recipe.segments:
        changed = [
            index
            for index, (source, wanted) in enumerate(
                zip(segment.start_rad, segment.target_rad)
            )
            if source != wanted
        ]
        assert len(changed) == 1
        joint = changed[0]
        assert max(abs(sample.dq_rad_s[joint]) for sample in segment.samples) <= 0.5
        assert max(abs(sample.ddq_rad_s2[joint]) for sample in segment.samples) <= 1.0


def test_recipe_can_issue_one_token_and_descriptors_reuse_previewed_segments():
    start = (0.0,) * 6
    target = (0.2, 0.0, -0.1, 0.0, 0.0, 0.0)
    recipe = generate_segmented_quintic_recipe(
        start,
        target,
        LIMITS,
        maximum_velocity_rad_s=0.5,
        maximum_acceleration_rad_s2=1.0,
        maximum_segment_delta_rad=0.25,
    )
    state = WorkflowState.initialize(start, LIMITS, MODEL_HASH)
    state = state.change_plan_target(target).set_plan_trajectory(recipe)
    approved = state.accept_successful_preview(
        recipe,
        PreviewChecks.successful(),
        created_monotonic_ns=123,
        nonce="segmented-preview",
    )
    token = approved.current_plan_token
    assert token is not None
    assert token.trajectory_sha256 == recipe.sha256
    assert token.trajectory_profile == "segmented-quintic-rest-to-rest-v1"
    descriptor = trajectory_command_descriptor(
        recipe.segments[0],
        plan_token_id=token.token_id,
        execute_at_monotonic_ns=9_000_000_000,
        segment_index=0,
        segment_count=len(recipe.segments),
    )
    wire = descriptor["trajectory"]
    assert wire["trajectory_sha256"] == recipe.segments[0].sha256
    assert wire["start_rad"] == list(recipe.segments[0].start_rad)
    assert wire["target_rad"] == list(recipe.segments[0].target_rad)
    assert wire["duration_ns"] == round(
        recipe.segments[0].profile.duration_s * 1.0e9
    )
    assert wire["interval_count"] == recipe.segments[0].profile.interval_count
    assert wire["duration_ns"] % wire["interval_count"] == 0
    assert 0 < wire["duration_ns"] // wire["interval_count"] <= 10_000_000
    manifest = trajectory_plan_manifest(recipe)
    assert manifest == {
        "schema": "go-m8010-plan-manifest/1.0",
        "recipe_sha256": recipe.sha256,
        "segment_sha256": [segment.sha256 for segment in recipe.segments],
    }


def test_recipe_rejects_unchanged_target_and_descriptor_rejects_bad_identity():
    with pytest.raises(ContractViolation, match="changed target"):
        generate_segmented_quintic_recipe(
            ACTUAL,
            ACTUAL,
            LIMITS,
            maximum_velocity_rad_s=0.5,
            maximum_acceleration_rad_s2=1.0,
            maximum_segment_delta_rad=0.25,
        )
    segment = plan()
    with pytest.raises(ContractViolation, match="plan_token_id"):
        trajectory_command_descriptor(
            segment,
            plan_token_id="bad",
            execute_at_monotonic_ns=1,
            segment_index=0,
            segment_count=1,
        )


def test_descriptor_rejects_non_nanosecond_preview_duration():
    segment = generate_quintic_trajectory(
        ACTUAL,
        TARGET,
        LIMITS,
        duration_s=0.1234567894,
    )
    with pytest.raises(ContractViolation, match="align exactly"):
        trajectory_command_descriptor(
            segment,
            plan_token_id="a" * 64,
            execute_at_monotonic_ns=1,
            segment_index=0,
            segment_count=1,
        )


def test_descriptor_rejects_non_integer_nanosecond_grid():
    segment = generate_quintic_trajectory(
        ACTUAL,
        TARGET,
        LIMITS,
        duration_s=1.000000003,
        maximum_sample_period_s=0.01,
    )
    with pytest.raises(ContractViolation, match="integer time grid"):
        trajectory_command_descriptor(
            segment,
            plan_token_id="a" * 64,
            execute_at_monotonic_ns=1,
            segment_index=0,
            segment_count=1,
        )


def test_shared_integer_sample_index_pins_both_endpoints():
    execute_at = 10_000_000_000
    assert trajectory_sample_index_at(
        execute_at - 1,
        execute_at_monotonic_ns=execute_at,
        duration_ns=1_000_000_003,
        interval_count=101,
    ) == 0
    assert trajectory_sample_index_at(
        execute_at + 500_000_001,
        execute_at_monotonic_ns=execute_at,
        duration_ns=1_000_000_003,
        interval_count=101,
    ) == (500_000_001 * 101) // 1_000_000_003
    assert trajectory_sample_index_at(
        execute_at + 1_000_000_003,
        execute_at_monotonic_ns=execute_at,
        duration_ns=1_000_000_003,
        interval_count=101,
    ) == 101


def test_planned_path_request_contains_exact_reproducible_recipe_before_token():
    recipe = generate_segmented_quintic_recipe(
        ACTUAL,
        TARGET,
        LIMITS,
        maximum_velocity_rad_s=0.5,
        maximum_acceleration_rad_s2=1.0,
        maximum_segment_delta_rad=0.25,
        maximum_sample_period_s=0.01,
    )
    request = planned_trajectory_feasibility_request(
        recipe,
        source_instance_id="1" * 32,
        sequence=7,
        source_monotonic_ns=123456789,
        session_id="session-a",
        state_instance_id="2" * 32,
        model_sha256="3" * 64,
        gravity_config_sha256="4" * 64,
        thermal_config_sha256="5" * 64,
    )
    assert request["schema"] == (
        "go-m8010-planned-path-feasibility-request/1.0"
    )
    assert request["trajectory_sha256"] == recipe.sha256
    assert request["trajectory"]["trajectory_sha256"] == recipe.sha256
    assert [
        item["trajectory_sha256"]
        for item in request["trajectory"]["segments"]
    ] == [segment.sha256 for segment in recipe.segments]
    assert len(request["request_sha256"]) == 64
    assert request["thermal_config_sha256"] == "5" * 64
    assert "plan_token_id" not in request


def test_planned_path_request_rejects_invalid_source_or_clock():
    recipe = generate_segmented_quintic_recipe(
        ACTUAL,
        TARGET,
        LIMITS,
        maximum_velocity_rad_s=0.5,
        maximum_acceleration_rad_s2=1.0,
        maximum_segment_delta_rad=0.25,
    )
    common = dict(
        recipe=recipe,
        source_instance_id="1" * 32,
        sequence=1,
        source_monotonic_ns=1,
        session_id="session-a",
        state_instance_id="2" * 32,
        model_sha256="3" * 64,
        gravity_config_sha256="4" * 64,
        thermal_config_sha256="5" * 64,
    )
    for replacement in ("short", "g" * 32):
        candidate = dict(common, source_instance_id=replacement)
        with pytest.raises(ContractViolation, match="source_instance_id"):
            planned_trajectory_feasibility_request(**candidate)
    with pytest.raises(ContractViolation, match="timestamp"):
        planned_trajectory_feasibility_request(
            **dict(common, source_monotonic_ns=0)
        )

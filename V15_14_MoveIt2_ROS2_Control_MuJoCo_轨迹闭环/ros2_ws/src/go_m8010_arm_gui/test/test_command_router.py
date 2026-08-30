import json
import math
import time
from copy import deepcopy
from pathlib import Path

import pytest
import yaml

from go_m8010_arm_gui.command_router import (
    COLLISION_GUARD_BOUNDARY_TOLERANCE_DEG,
    COLLISION_GUARD_MARGIN_DEG,
    COLLISION_GUARD_MAX_STEP_DEG,
    COLLISION_GUARD_PROOF_MAX_AGE_NS,
    COLLISION_GUARD_RESULT_SCHEMA,
    COLLISION_HOLD_TRACKING_TOLERANCE_DEG,
    COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG,
    COLLISION_MARGIN_POLICY,
    COLLISION_TUBE_AXIS_PROBE_STEP_DEG,
    COLLISION_TUBE_CROSS_SECTION_MAX_POSE_COUNT,
    COLLISION_TUBE_GRID_MAX_POSE_COUNT,
    COLLISION_TUBE_GRID_OFFSETS,
    COLLISION_TUBE_PROBE_JOINT_NAMES,
    KD_LIMITS,
    KP_LIMITS,
    GUI_COMMAND_SCHEMA_V12,
    GUI_COMMAND_SCHEMA_V13,
    GRAVITY_CONFIG_SHA256,
    GRAVITY_STATUS_SCHEMA,
    MODEL_COMMAND_LOWER_RAD,
    MODEL_COMMAND_UPPER_RAD,
    MOTOR_NAMES,
    PLAN_MANIFEST_SCHEMA,
    PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG,
    PRODUCTION_COLLISION_CONTRACT_SHA256,
    PRODUCTION_KINEMATIC_GUARD_SHA256,
    PRODUCTION_MODEL_SHA256,
    QUINTIC_COMMAND_SCHEMA,
    QUINTIC_PROFILE,
    TRAJECTORY_MAXIMUM_EXECUTE_LEAD_NS,
    TRAJECTORY_MINIMUM_EXECUTE_LEAD_NS,
    UINT64_MAXIMUM,
    CollisionGuardProofGate,
    CommandRouter,
    CommandReplayGuard,
    GravityAuthorityGate,
    PlanManifestGate,
    RejectionTracker,
    canonical_collision_hardware_state_sha256,
    canonical_six_doubles_sha256,
    payload_for_domain,
    validate_command as production_validate_command,
)


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "arm_gui.yaml"
LAUNCH_PATH = Path(__file__).resolve().parents[1] / "launch" / "arm_gui.launch.py"
COMMAND_SOURCE = "0123456789abcdef0123456789abcdef"
STATE_SOURCE = "fedcba9876543210fedcba9876543210"
SESSION_POSE_SHA256 = "a" * 64
PLAN_TOKEN_ID = "b" * 64
TRAJECTORY_SHA256 = "c" * 64
RECIPE_SHA256 = "d" * 64


def validate_command(text, now_ns=None):
    """Exercise the explicit offline v1.2 compatibility path in old tests."""

    return production_validate_command(
        text,
        now_ns=now_ns,
        allow_legacy_v12_position=True,
    )


def collision_guard_result(
    targets,
    *,
    now_ns=None,
    source_instance_id=COMMAND_SOURCE,
    request_sequence=1,
    kind="execute",
    safe=True,
    moving_joint_mask=None,
    start_relative_rad=None,
):
    if now_ns is None:
        now_ns = time.monotonic_ns()
    if moving_joint_mask is None:
        moving_joint_mask = [True, False, False, False, False, False]
    if start_relative_rad is None:
        start_relative_rad = [
            0.0 if moving else float(target)
            for moving, target in zip(moving_joint_mask, targets)
        ]
    hardware_source_ns = now_ns - 3
    hardware_modes = {name: "hold" for name in MOTOR_NAMES}
    hardware_state = {
        "schema": "go-m8010-hardware-state/1.1",
        "session_id": "vertical-session",
        "state_instance_id": STATE_SOURCE,
        "sequence": 91,
        "source_monotonic_ns": hardware_source_ns,
        "position_rad": list(start_relative_rad),
        "velocity_rad_s": [0.0] * 6,
        "controller_mode_by_motor": hardware_modes,
    }
    moving_count = sum(moving_joint_mask)
    result = {
        "schema": COLLISION_GUARD_RESULT_SCHEMA,
        "source_instance_id": source_instance_id,
        "request_sequence": request_sequence,
        "source_monotonic_ns": now_ns - 2,
        "kind": kind,
        "session_id": "vertical-session",
        "state_instance_id": STATE_SOURCE,
        "moving_joint_mask": list(moving_joint_mask),
        "start_relative_rad": list(start_relative_rad),
        "target_relative_rad": list(targets),
        "target_sha256": canonical_six_doubles_sha256(targets),
        "collision_margin_deg": COLLISION_GUARD_MARGIN_DEG,
        "session_pose_sha256": SESSION_POSE_SHA256,
        "checked_monotonic_ns": now_ns - 1,
        "safe": safe,
        "reason": "clear" if safe else "self_collision",
        "recommended_relative_rad": list(targets) if safe else list(start_relative_rad),
        "contact_pairs": [] if safe else [["a", "b"]],
        "model_sha256": PRODUCTION_MODEL_SHA256,
        "kinematic_guard_sha256": PRODUCTION_KINEMATIC_GUARD_SHA256,
        "collision_contract_sha256": PRODUCTION_COLLISION_CONTRACT_SHA256,
        "ground_contact_policy": "UNSAFE_GROUND_COLLISION_NOT_SELF_COLLISION",
        "max_step_deg": COLLISION_GUARD_MAX_STEP_DEG,
        "boundary_tolerance_deg": COLLISION_GUARD_BOUNDARY_TOLERANCE_DEG,
        "hardware_state_sequence": hardware_state["sequence"],
        "hardware_state_source_monotonic_ns": hardware_source_ns,
        "hardware_position_rad": list(start_relative_rad),
        "hardware_velocity_rad_s": [0.0] * 6,
        "hardware_controller_mode_by_motor": hardware_modes,
        "hardware_state_sha256": canonical_collision_hardware_state_sha256(
            hardware_state
        ),
        "margin_policy": COLLISION_MARGIN_POLICY,
        "joint_space_tube_radius_deg": COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG,
        "hold_tracking_tolerance_deg": COLLISION_HOLD_TRACKING_TOLERANCE_DEG,
        "tube_probe_joint_names": list(COLLISION_TUBE_PROBE_JOINT_NAMES),
        "tube_grid_offsets": list(COLLISION_TUBE_GRID_OFFSETS),
        "tube_grid_max_pose_count": COLLISION_TUBE_GRID_MAX_POSE_COUNT,
        "tube_axis_probe_step_deg": COLLISION_TUBE_AXIS_PROBE_STEP_DEG,
        "tube_cross_section_max_pose_count": (
            COLLISION_TUBE_CROSS_SECTION_MAX_POSE_COUNT
        ),
        "absolute_joint_limits_deg": [
            list(bounds) for bounds in PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG
        ],
        "single_joint_path_required": True,
        "step_count": 1,
        "path_sample_count": 1,
        "pose_check_count": 1,
        "tube_probe_count": 1,
        "tube_probe_cache_hit_count": 0,
        "moving_joint": (
            COLLISION_TUBE_PROBE_JOINT_NAMES[moving_joint_mask.index(True)]
            if moving_count == 1 else None
        ),
        "moving_joint_count": moving_count,
    }
    return result


def command(
    mode="position", target=0.0, active_joint_mask=None,
    moving_joint_mask=None,
):
    if active_joint_mask is None:
        active_joint_mask = [False] * 6 if mode == "brake" else [True] * 6
    if moving_joint_mask is None:
        moving_joint_mask = (
            [True, False, False, False, False, False]
            if mode == "position" else [False] * 6
        )
    document = {
        "schema": "go-m8010-gui-command/1.2",
        "sequence": 1,
        "source_instance_id": COMMAND_SOURCE,
        "source_monotonic_ns": time.monotonic_ns(),
        "mode": mode,
        "targets_rad": [target] * 6,
        "active_joint_mask": active_joint_mask,
        "moving_joint_mask": moving_joint_mask,
        "activation_epoch": 1,
        "maximum_velocity_rad_s": math.radians(5.0),
        "maximum_acceleration_rad_s2": math.radians(20.0),
    }
    if mode == "position":
        document["collision_guard_proof"] = collision_guard_result(
            document["targets_rad"],
            moving_joint_mask=document["moving_joint_mask"],
        )
    return json.dumps(document)


def command_v13(
    *,
    moving_index=0,
    displacement=0.01,
    duration_ns=500_000_000,
    interval_count=50,
    now_ns=None,
):
    if now_ns is None:
        now_ns = time.monotonic_ns()
    document = json.loads(command())
    moving_joint_mask = [False] * 6
    moving_joint_mask[moving_index] = True
    start_rad = [0.0] * 6
    target_rad = list(start_rad)
    target_rad[moving_index] = displacement
    document.update({
        "schema": GUI_COMMAND_SCHEMA_V13,
        "source_monotonic_ns": now_ns,
        "targets_rad": target_rad,
        "active_joint_mask": [True] * 6,
        "moving_joint_mask": moving_joint_mask,
        "collision_guard_proof": collision_guard_result(
            target_rad,
            now_ns=now_ns,
            moving_joint_mask=moving_joint_mask,
            start_relative_rad=start_rad,
        ),
        "plan_token_id": PLAN_TOKEN_ID,
        "plan_manifest": {
            "schema": PLAN_MANIFEST_SCHEMA,
            "recipe_sha256": RECIPE_SHA256,
            "segment_sha256": [TRAJECTORY_SHA256],
        },
        "trajectory": {
            "schema": QUINTIC_COMMAND_SCHEMA,
            "trajectory_sha256": TRAJECTORY_SHA256,
            "profile": QUINTIC_PROFILE,
            "start_rad": list(start_rad),
            "target_rad": list(target_rad),
            "duration_ns": duration_ns,
            "interval_count": interval_count,
            "execute_at_monotonic_ns": now_ns + 250_000_000,
            "segment_index": 0,
            "segment_count": 1,
        },
    })
    return document


def gravity_status(
    *, now_ns, sequence=1, source_instance_id="1" * 32,
    scale=0.1, target=0.25,
):
    source_ns = now_ns - 10
    status = {
        "schema": GRAVITY_STATUS_SCHEMA,
        "source": "whole_arm_gravity_node",
        "source_instance_id": source_instance_id,
        "sequence": sequence,
        "source_monotonic_ns": source_ns,
        "model_sha256": PRODUCTION_MODEL_SHA256,
        "production_model_hash_match": True,
        "gravity_config_sha256": GRAVITY_CONFIG_SHA256,
        "anchor_valid": True,
        "session_id": "vertical-session",
        "state_instance_id": STATE_SOURCE,
        "hardware_state_sequence": 91 + sequence,
        "hardware_state_source_monotonic_ns": source_ns - 1,
        "q_actual_sha256": "2" * 64,
        "last_update_age_s": 0.0,
        "gravity_joint_nm": [1.0, 2.0, 3.0, 0.4, 0.5, 0.0],
        "feedforward_nm": [0.01, 0.02, 0.03, 0.01, 0.01, 0.0],
        "gravity_scale": scale,
        "gravity_scale_target": target,
        "finite_bounded": True,
        "pose_feasibility": "PASS",
        "blocker": None,
        "hardware_enable_requested": True,
        "continuous_rotor_limits_authoritative": True,
        "actuation_interface_present": True,
        "hardware_tff_enabled": target > 0.0,
    }
    gravity_maxima = {name: 0.1 for name in MOTOR_NAMES}
    predicted_maxima = {name: 0.2 for name in MOTOR_NAMES}
    continuous_limits = {name: 1.0 for name in MOTOR_NAMES}
    short_peak_limits = {name: 2.0 for name in MOTOR_NAMES}
    status["planned_trajectory_feasibility"] = {
        "schema": "go-m8010-planned-load-thermal-feasibility/1.0",
        "source": "whole_arm_gravity_node",
        "source_instance_id": source_instance_id,
        "sequence": sequence,
        "source_monotonic_ns": source_ns,
        "result": "PASS",
        "load_feasibility": "PASS",
        "thermal_feasibility": "PASS",
        "current_temperature_margin_result": "PASS",
        "request_sha256": "4" * 64,
        "trajectory_sha256": RECIPE_SHA256,
        "session_id": "vertical-session",
        "state_instance_id": STATE_SOURCE,
        "model_sha256": PRODUCTION_MODEL_SHA256,
        "gravity_config_sha256": GRAVITY_CONFIG_SHA256,
        "thermal_config_sha256": (
            "1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467"
        ),
        "continuous_rotor_limits_authoritative": True,
        "temperature_limits_authoritative": True,
        "sample_count": 101,
        "evaluated_sample_count": 101,
        "load_evaluation_basis": (
            "MUJOCO_QFRC_BIAS_CONTINUOUS_PLUS_MJ_INVERSE_SHORT_PEAK_EVERY_SAMPLE"
        ),
        "thermal_evaluation_basis": (
            "CURRENT_MEASURED_TEMPERATURE_TO_DERATING_THRESHOLD_PLUS_"
            "ALL_SAMPLE_PREDICTED_LOAD_WITHIN_CONTINUOUS_RATING_"
            "NO_HEAT_RISE_MODEL"
        ),
        "maximum_abs_gravity_joint_torque_nm_by_joint": {
            f"J{index}": 0.5 for index in range(1, 7)
        },
        "maximum_abs_gravity_rotor_torque_nm_by_motor": gravity_maxima,
        "maximum_abs_predicted_rotor_torque_nm_by_motor": predicted_maxima,
        "continuous_rotor_torque_limit_nm_by_motor": continuous_limits,
        "short_peak_rotor_torque_limit_nm_by_motor": short_peak_limits,
        "minimum_continuous_rotor_torque_margin_nm": 0.9,
        "minimum_short_peak_rotor_torque_margin_nm": 1.8,
        "minimum_predicted_continuous_rotor_torque_margin_nm": 0.8,
        "minimum_rotor_torque_margin_nm": 0.9,
        "minimum_thermal_margin_c": 5.0,
        "blocker_code": None,
        "blocker": None,
    }
    return status


def validate_v13(document, *, now_ns=None):
    if now_ns is None:
        now_ns = document["source_monotonic_ns"]
    return validate_command(json.dumps(document), now_ns=now_ns)


def test_gravity_gate_observes_then_injects_exact_bounded_authority():
    now_ns = 10_000_000_000
    gate = GravityAuthorityGate(maximum_age_ns=250_000_000)
    status = gravity_status(now_ns=now_ns)
    assert gate.observe_status(status, now_ns=now_ns)
    command_value = command_v13(now_ns=now_ns)
    gate.authorize(command_value, now_ns=now_ns + 1)
    assert command_value["feedforward_nm"] == status["feedforward_nm"]
    authority = command_value["gravity_authority"]
    assert authority["source_instance_id"] == status["source_instance_id"]
    assert authority["session_id"] == status["session_id"]
    assert authority["state_instance_id"] == status["state_instance_id"]
    assert authority["gravity_scale_target"] == 0.25
    assert set(authority) == {
        "schema", "source_instance_id", "sequence", "source_monotonic_ns",
        "model_sha256", "gravity_config_sha256", "session_id",
        "state_instance_id", "gravity_scale", "gravity_scale_target",
        "feedforward_nm",
    }


def test_gravity_gate_requires_exact_recipe_proof_and_strictly_positive_margin():
    now_ns = 10_000_000_000
    gate = GravityAuthorityGate(maximum_age_ns=250_000_000)
    status = gravity_status(now_ns=now_ns)
    status["planned_trajectory_feasibility"]["trajectory_sha256"] = "e" * 64
    assert gate.observe_status(status, now_ns=now_ns)
    with pytest.raises(ValueError, match="manifest"):
        gate.authorize(command_v13(now_ns=now_ns), now_ns=now_ns + 1)

    zero = gravity_status(now_ns=now_ns + 10, sequence=2)
    proof = zero["planned_trajectory_feasibility"]
    proof["continuous_rotor_torque_limit_nm_by_motor"] = {
        name: 0.1 for name in MOTOR_NAMES
    }
    proof["minimum_continuous_rotor_torque_margin_nm"] = 0.0
    proof["minimum_rotor_torque_margin_nm"] = 0.0
    assert gate.observe_status(zero, now_ns=now_ns + 10)
    assert gate.available
    with pytest.raises(ValueError, match="manifest"):
        gate.authorize(
            command_v13(now_ns=now_ns + 10), now_ns=now_ns + 11
        )


@pytest.mark.parametrize("invalid", [math.nan, math.inf, -math.inf])
@pytest.mark.parametrize("field", [
    "minimum_continuous_rotor_torque_margin_nm",
    "minimum_short_peak_rotor_torque_margin_nm",
    "minimum_predicted_continuous_rotor_torque_margin_nm",
    "minimum_rotor_torque_margin_nm",
    "minimum_thermal_margin_c",
])
def test_gravity_gate_rejects_nonfinite_planned_proof_scalars(field, invalid):
    now_ns = 10_000_000_000
    gate = GravityAuthorityGate(maximum_age_ns=250_000_000)
    status = gravity_status(now_ns=now_ns)
    status["planned_trajectory_feasibility"][field] = invalid
    # A malformed POSITION proof must not revoke independent HOLD gravity.
    assert gate.observe_status(status, now_ns=now_ns)
    assert gate.available
    hold = json.loads(command(mode="hold"))
    gate.authorize(hold, now_ns=now_ns + 1)
    assert hold["feedforward_nm"] == status["feedforward_nm"]
    with pytest.raises(ValueError, match="manifest"):
        gate.authorize(command_v13(now_ns=now_ns), now_ns=now_ns + 1)


def test_invalid_or_missing_path_proof_does_not_revoke_hold_gravity_authority():
    now_ns = 10_000_000_000
    gate = GravityAuthorityGate(maximum_age_ns=250_000_000)
    status = gravity_status(now_ns=now_ns)
    status["planned_trajectory_feasibility"] = {
        "result": "NOT_EVALUATED"
    }
    assert gate.observe_status(status, now_ns=now_ns)
    hold = json.loads(command(mode="hold"))
    gate.authorize(hold, now_ns=now_ns + 1)
    assert hold["feedforward_nm"] == status["feedforward_nm"]
    assert hold["gravity_authority"]["session_id"] == "vertical-session"


def test_gravity_gate_rejects_stale_replay_and_incomplete_hardware_binding():
    now_ns = 10_000_000_000
    gate = GravityAuthorityGate(maximum_age_ns=250_000_000)
    valid = gravity_status(now_ns=now_ns)
    assert gate.observe_status(valid, now_ns=now_ns)

    replay = gravity_status(now_ns=now_ns + 1, sequence=1)
    assert not gate.observe_status(replay, now_ns=now_ns + 1)
    assert not gate.available
    with pytest.raises(ValueError, match="不存在或已过期"):
        gate.authorize(command_v13(now_ns=now_ns), now_ns=now_ns + 2)

    for field, value in (
        ("hardware_state_sequence", 0),
        ("hardware_state_source_monotonic_ns", now_ns - 300_000_000),
        ("q_actual_sha256", "bad"),
        ("continuous_rotor_limits_authoritative", False),
    ):
        candidate = gravity_status(now_ns=now_ns + 100, sequence=2)
        candidate[field] = value
        assert not gate.observe_status(candidate, now_ns=now_ns + 100)


def test_gravity_policy_source_session_and_scale_freeze_within_epoch():
    now_ns = 10_000_000_000
    gate = GravityAuthorityGate(maximum_age_ns=250_000_000)
    assert gate.observe_status(
        gravity_status(now_ns=now_ns, sequence=1), now_ns=now_ns
    )
    first = command_v13(now_ns=now_ns)
    gate.authorize(first, now_ns=now_ns + 1)

    changed_scale = gravity_status(
        now_ns=now_ns + 10, sequence=2, scale=0.2, target=0.5,
    )
    assert gate.observe_status(changed_scale, now_ns=now_ns + 10)
    same_epoch = command_v13(now_ns=now_ns + 10)
    with pytest.raises(ValueError, match="policy发生变化"):
        gate.authorize(same_epoch, now_ns=now_ns + 11)

    same_epoch["activation_epoch"] += 1
    gate.authorize(same_epoch, now_ns=now_ns + 11)
    assert same_epoch["gravity_authority"]["gravity_scale_target"] == 0.5

    restarted = gravity_status(
        now_ns=now_ns + 20,
        sequence=1,
        source_instance_id="3" * 32,
        scale=0.2,
        target=0.5,
    )
    assert gate.observe_status(restarted, now_ns=now_ns + 20)
    with pytest.raises(ValueError, match="policy发生变化"):
        gate.authorize(same_epoch, now_ns=now_ns + 21)


def test_gravity_gate_forces_zero_for_brake_and_drag():
    gate = GravityAuthorityGate()
    for mode in ("brake", "drag"):
        value = json.loads(command(mode=mode))
        value["feedforward_nm"] = [1.0] * 6
        value["gravity_authority"] = {"forged": True}
        gate.authorize(value, now_ns=10_000_000_000)
        assert value["feedforward_nm"] == [0.0] * 6
        assert "gravity_authority" not in value


def normalized_plan_segment(
    *,
    index,
    segment_hashes,
    now_ns,
    activation_epoch,
    proof_sequence,
    source_value=0.0,
    target_value=0.01,
    token=PLAN_TOKEN_ID,
):
    document = command_v13(now_ns=now_ns)
    start = [0.0] * 6
    start[0] = source_value
    target = list(start)
    target[0] = target_value
    document.update({
        "sequence": index + 1,
        "activation_epoch": activation_epoch,
        "targets_rad": target,
        "plan_token_id": token,
        "collision_guard_proof": collision_guard_result(
            target,
            now_ns=now_ns,
            request_sequence=proof_sequence,
            start_relative_rad=start,
        ),
        "plan_manifest": {
            "schema": PLAN_MANIFEST_SCHEMA,
            "recipe_sha256": RECIPE_SHA256,
            "segment_sha256": list(segment_hashes),
        },
    })
    document["trajectory"].update({
        "trajectory_sha256": segment_hashes[index],
        "start_rad": start,
        "target_rad": target,
        "execute_at_monotonic_ns": now_ns + 250_000_000,
        "segment_index": index,
        "segment_count": len(segment_hashes),
    })
    normalized, _payload = validate_v13(document, now_ns=now_ns)
    return document, normalized


def test_valid_six_joint_command():
    value, payload = validate_command(command())
    assert value["mode"] == "position" and len(value["targets_rad"]) == 6
    assert json.loads(payload)["schema"] == "go-m8010-gui-command/1.2"
    assert value["source_instance_id"] == "0123456789abcdef0123456789abcdef"


def test_v13_position_normalizes_complete_quintic_authority():
    document = command_v13()
    value, payload = validate_v13(document)

    assert value["schema"] == GUI_COMMAND_SCHEMA_V13
    assert value["plan_token_id"] == PLAN_TOKEN_ID
    assert value["plan_manifest"] == document["plan_manifest"]
    assert value["trajectory"] == document["trajectory"]
    assert json.loads(payload)["trajectory"] == document["trajectory"]


@pytest.mark.parametrize(
    ("moving_index", "moving_domain"),
    [(0, "J1"), (1, "J2"), (3, "J345"), (5, "J6")],
)
def test_v13_fault_domain_forwarding_retains_only_moving_authority(
    moving_index, moving_domain
):
    value, _payload = validate_v13(command_v13(moving_index=moving_index))
    for domain in ("J1", "J2", "J345", "J6"):
        forwarded = json.loads(payload_for_domain(value, domain))
        assert "collision_guard_proof" not in forwarded
        assert "plan_manifest" not in forwarded
        if domain == moving_domain:
            assert forwarded["schema"] == GUI_COMMAND_SCHEMA_V13
            assert forwarded["mode"] == "position"
            assert forwarded["plan_token_id"] == PLAN_TOKEN_ID
            assert forwarded["trajectory"] == value["trajectory"]
        else:
            assert forwarded["schema"] == GUI_COMMAND_SCHEMA_V12
            assert forwarded["mode"] == "hold"
            assert forwarded["moving_joint_mask"] == [False] * 6
            assert "plan_token_id" not in forwarded
            assert "trajectory" not in forwarded


def test_v13_brake_ignores_malformed_trajectory_fields_and_downgrades():
    document = json.loads(command(mode="brake"))
    document.update({
        "schema": GUI_COMMAND_SCHEMA_V13,
        "plan_token_id": {"malformed": True},
        "trajectory": {
            "schema": "malformed",
            "duration_ns": False,
            "unexpected": float("nan"),
        },
        "plan_manifest": {"malformed": True},
    })
    value, payload = validate_command(json.dumps(document))

    assert value["schema"] == GUI_COMMAND_SCHEMA_V12
    assert value["mode"] == "brake"
    assert "plan_token_id" not in value
    assert "trajectory" not in value
    assert "plan_manifest" not in value
    assert "plan_token_id" not in json.loads(payload)
    assert "trajectory" not in json.loads(payload)
    for domain in ("J1", "J2", "J345", "J6"):
        forwarded = json.loads(payload_for_domain(value, domain))
        assert forwarded["schema"] == GUI_COMMAND_SCHEMA_V12
        assert forwarded["mode"] == "brake"
        assert "plan_token_id" not in forwarded
        assert "trajectory" not in forwarded
        assert "plan_manifest" not in forwarded


def test_explicit_offline_v12_position_compatibility_does_not_require_v13():
    value, payload = validate_command(command(target=0.0))

    assert value["schema"] == GUI_COMMAND_SCHEMA_V12
    assert value["mode"] == "position"
    assert "plan_token_id" not in value
    assert "trajectory" not in value
    assert json.loads(payload)["schema"] == GUI_COMMAND_SCHEMA_V12
    assert json.loads(payload_for_domain(value, "J1"))["mode"] == "position"


def test_production_validation_rejects_v12_position_by_default():
    with pytest.raises(ValueError, match="生产路由禁止旧版POSITION"):
        production_validate_command(command(target=0.0))

    # HOLD/BRAKE stay compatible; the switch narrows only active motion.
    hold, _payload = production_validate_command(command(mode="hold"))
    brake, _payload = production_validate_command(command(mode="brake"))
    assert hold["mode"] == "hold"
    assert brake["mode"] == "brake"


def test_launch_defaults_legacy_v12_position_switch_to_false():
    launch_source = LAUNCH_PATH.read_text(encoding="utf-8")
    assert '"allow_legacy_v12_position", default_value="false"' in launch_source
    assert 'self.declare_parameter("allow_legacy_v12_position", False)' in (
        Path(__file__).resolve().parents[1]
        / "go_m8010_arm_gui"
        / "command_router.py"
    ).read_text(encoding="utf-8")


@pytest.mark.parametrize(
    "missing_field", ["plan_token_id", "trajectory", "plan_manifest"]
)
def test_v13_position_requires_both_authority_fields(missing_field):
    document = command_v13()
    document.pop(missing_field)
    with pytest.raises(ValueError, match="缺少计划轨迹权限"):
        validate_v13(document)


@pytest.mark.parametrize(
    "plan_token_id",
    ["b" * 63, "B" * 64, "g" * 64, 1, None, True],
)
def test_v13_position_requires_lowercase_sha256_plan_token(plan_token_id):
    document = command_v13()
    document["plan_token_id"] = plan_token_id
    with pytest.raises(ValueError, match="计划令牌格式无效"):
        validate_v13(document)


@pytest.mark.parametrize(
    "mutate",
    [
        pytest.param(
            lambda manifest, _trajectory: manifest.update(extra=True),
            id="extra-field",
        ),
        pytest.param(
            lambda manifest, _trajectory: manifest.pop("recipe_sha256"),
            id="missing-field",
        ),
        pytest.param(
            lambda manifest, _trajectory: manifest.update(schema="wrong"),
            id="schema",
        ),
        pytest.param(
            lambda manifest, _trajectory: manifest.update(
                recipe_sha256="D" * 64
            ),
            id="recipe-hash",
        ),
        pytest.param(
            lambda manifest, _trajectory: manifest.update(segment_sha256=[]),
            id="empty-segments",
        ),
        pytest.param(
            lambda manifest, _trajectory: manifest.update(
                segment_sha256=["not-a-hash"]
            ),
            id="bad-segment-hash",
        ),
    ],
)
def test_v13_position_rejects_malformed_plan_manifest(mutate):
    document = command_v13()
    mutate(document["plan_manifest"], document["trajectory"])
    with pytest.raises(ValueError, match="计划清单格式无效"):
        validate_v13(document)


def test_v13_position_binds_manifest_count_and_current_segment_hash():
    count_mismatch = command_v13()
    count_mismatch["trajectory"]["segment_count"] = 2
    with pytest.raises(ValueError, match="计划清单与轨迹分段不匹配"):
        validate_v13(count_mismatch)

    hash_mismatch = command_v13()
    hash_mismatch["plan_manifest"]["segment_sha256"] = ["e" * 64]
    with pytest.raises(ValueError, match="计划清单与轨迹分段不匹配"):
        validate_v13(hash_mismatch)


@pytest.mark.parametrize(
    "mutate",
    [
        pytest.param(
            lambda trajectory: trajectory.update(unexpected=True),
            id="extra-field",
        ),
        pytest.param(
            lambda trajectory: trajectory.pop("profile"),
            id="missing-field",
        ),
        pytest.param(
            lambda trajectory: trajectory.update(schema="wrong"),
            id="schema",
        ),
        pytest.param(
            lambda trajectory: trajectory.update(trajectory_sha256="C" * 64),
            id="trajectory-sha256",
        ),
        pytest.param(
            lambda trajectory: trajectory.update(profile="wrong"),
            id="profile",
        ),
        pytest.param(
            lambda trajectory: trajectory.update(start_rad=[0.0] * 5),
            id="short-start",
        ),
        pytest.param(
            lambda trajectory: trajectory.update(
                target_rad=[float("nan")] * 6
            ),
            id="non-finite-target",
        ),
        pytest.param(
            lambda trajectory: trajectory.update(duration_ns=True),
            id="boolean-duration",
        ),
        pytest.param(
            lambda trajectory: trajectory.update(duration_ns=0),
            id="zero-duration",
        ),
        pytest.param(
            lambda trajectory: trajectory.update(
                duration_ns=UINT64_MAXIMUM + 1
            ),
            id="duration-beyond-go-uint64",
        ),
        pytest.param(
            lambda trajectory: trajectory.update(interval_count=1_000_001),
            id="too-many-intervals",
        ),
        pytest.param(
            lambda trajectory: trajectory.update(interval_count=50.0),
            id="non-integer-intervals",
        ),
        pytest.param(
            lambda trajectory: trajectory.update(execute_at_monotonic_ns=0),
            id="zero-execute-time",
        ),
        pytest.param(
            lambda trajectory: trajectory.update(
                execute_at_monotonic_ns=UINT64_MAXIMUM + 1
            ),
            id="execute-time-beyond-go-uint64",
        ),
        pytest.param(
            lambda trajectory: trajectory.update(segment_index=-1),
            id="negative-segment-index",
        ),
        pytest.param(
            lambda trajectory: trajectory.update(segment_count=0),
            id="zero-segment-count",
        ),
        pytest.param(
            lambda trajectory: trajectory.update(
                segment_index=1, segment_count=1
            ),
            id="segment-index-not-less-than-count",
        ),
    ],
)
def test_v13_position_rejects_malformed_quintic_descriptor(mutate):
    document = command_v13()
    mutate(document["trajectory"])
    with pytest.raises(ValueError, match="轨迹描述符格式无效"):
        validate_v13(document)


def test_v13_position_binds_descriptor_target_to_command_target():
    document = command_v13()
    document["trajectory"]["target_rad"][0] += 0.001
    with pytest.raises(ValueError, match="轨迹描述符与运动目标不匹配"):
        validate_v13(document)


@pytest.mark.parametrize("proof_mismatch", ["start", "target", "mask"])
def test_v13_position_binds_descriptor_to_collision_proof(proof_mismatch):
    document = command_v13()
    now_ns = document["source_monotonic_ns"]
    if proof_mismatch == "start":
        proof_start = [-0.001, 0.0, 0.0, 0.0, 0.0, 0.0]
        document["collision_guard_proof"] = collision_guard_result(
            document["targets_rad"],
            now_ns=now_ns,
            moving_joint_mask=document["moving_joint_mask"],
            start_relative_rad=proof_start,
        )
    elif proof_mismatch == "target":
        proof_target = list(document["targets_rad"])
        proof_target[0] += 0.001
        document["collision_guard_proof"] = collision_guard_result(
            proof_target,
            now_ns=now_ns,
            moving_joint_mask=document["moving_joint_mask"],
        )
    else:
        proof_mask = [False, True, False, False, False, False]
        document["collision_guard_proof"] = collision_guard_result(
            document["targets_rad"],
            now_ns=now_ns,
            moving_joint_mask=proof_mask,
        )
    with pytest.raises(ValueError, match="轨迹描述符与碰撞守卫不匹配"):
        validate_v13(document)


def test_v13_position_rejects_nonmoving_displacement():
    document = command_v13()
    document["trajectory"]["start_rad"][1] = 0.001
    document["collision_guard_proof"] = collision_guard_result(
        document["targets_rad"],
        now_ns=document["source_monotonic_ns"],
        moving_joint_mask=document["moving_joint_mask"],
        start_relative_rad=document["trajectory"]["start_rad"],
    )
    with pytest.raises(ValueError, match="轨迹描述符与运动目标不匹配"):
        validate_v13(document)


def test_v13_position_rejects_zero_moving_displacement():
    document = command_v13(displacement=0.0)
    with pytest.raises(ValueError, match="轨迹描述符与运动目标不匹配"):
        validate_v13(document)


def test_v13_position_accepts_10ms_grid_and_rejects_larger_grid():
    valid, _payload = validate_v13(
        command_v13(duration_ns=500_000_000, interval_count=50)
    )
    assert valid["trajectory"]["interval_count"] == 50

    invalid = command_v13(duration_ns=500_000_000, interval_count=49)
    with pytest.raises(ValueError, match="轨迹采样网格超过10ms"):
        validate_v13(invalid)


def test_v13_position_checks_quintic_peak_velocity_and_acceleration():
    displacement = 0.01
    duration_s = 0.5
    peak_velocity = (15.0 / 8.0) * displacement / duration_s
    peak_acceleration = (
        (10.0 / math.sqrt(3.0)) * displacement / (duration_s * duration_s)
    )
    document = command_v13(
        displacement=displacement,
        duration_ns=int(duration_s * 1_000_000_000),
    )
    document["maximum_velocity_rad_s"] = peak_velocity
    document["maximum_acceleration_rad_s2"] = peak_acceleration
    value, _payload = validate_v13(document)
    assert value["maximum_velocity_rad_s"] == peak_velocity
    assert value["maximum_acceleration_rad_s2"] == peak_acceleration

    below_velocity = deepcopy(document)
    below_velocity["maximum_velocity_rad_s"] = peak_velocity * 0.99
    with pytest.raises(ValueError, match="轨迹峰值速度或加速度超过命令上限"):
        validate_v13(below_velocity)

    below_acceleration = deepcopy(document)
    below_acceleration["maximum_acceleration_rad_s2"] = peak_acceleration * 0.99
    with pytest.raises(ValueError, match="轨迹峰值速度或加速度超过命令上限"):
        validate_v13(below_acceleration)


def test_v13_j2_enforces_existing_15_degree_acceleration_limit():
    duration_s = 0.5
    peak_acceleration = math.radians(16.0)
    displacement = (
        peak_acceleration * duration_s * duration_s
        / (10.0 / math.sqrt(3.0))
    )
    document = command_v13(
        moving_index=1,
        displacement=displacement,
        duration_ns=int(duration_s * 1_000_000_000),
    )
    document["maximum_acceleration_rad_s2"] = math.radians(20.0)
    with pytest.raises(ValueError, match="轨迹峰值速度或加速度超过命令上限"):
        validate_v13(document)

    j1_document = command_v13(
        moving_index=0,
        displacement=displacement,
        duration_ns=int(duration_s * 1_000_000_000),
    )
    j1_document["maximum_acceleration_rad_s2"] = math.radians(20.0)
    validate_v13(j1_document)


def test_plan_preview_may_use_hypothetical_start_but_never_caches_authority():
    now_ns = time.monotonic_ns()
    start = [0.1, 0.0, 0.0, 0.0, 0.0, 0.0]
    target = [0.2, 0.0, 0.0, 0.0, 0.0, 0.0]
    result = collision_guard_result(
        target,
        now_ns=now_ns,
        kind="plan_preview",
        start_relative_rad=start,
    )
    result["hardware_position_rad"] = [0.0] * 6
    hardware_state = {
        "schema": "go-m8010-hardware-state/1.1",
        "session_id": result["session_id"],
        "state_instance_id": result["state_instance_id"],
        "sequence": result["hardware_state_sequence"],
        "source_monotonic_ns": result["hardware_state_source_monotonic_ns"],
        "position_rad": [0.0] * 6,
        "velocity_rad_s": [0.0] * 6,
        "controller_mode_by_motor": result[
            "hardware_controller_mode_by_motor"
        ],
    }
    result["hardware_state_sha256"] = canonical_collision_hardware_state_sha256(
        hardware_state
    )
    gate = CollisionGuardProofGate()
    assert gate.observe_result(result, now_ns=now_ns)
    assert gate.cached_proof_count == 0


@pytest.mark.parametrize(
    "mutate",
    [
        pytest.param(
            lambda value: value.update(plan_token_id="d" * 64),
            id="plan-token",
        ),
        pytest.param(
            lambda value: value["trajectory"].update(
                trajectory_sha256="e" * 64
            ),
            id="trajectory-hash",
        ),
        pytest.param(
            lambda value: value["trajectory"].update(
                execute_at_monotonic_ns=(
                    value["trajectory"]["execute_at_monotonic_ns"] + 1
                )
            ),
            id="execution-time",
        ),
    ],
)
def test_v13_same_epoch_rejects_plan_or_descriptor_changes(mutate):
    now_ns = 80_000_000_000
    document = command_v13(now_ns=now_ns)
    normalized, _payload = validate_v13(document, now_ns=now_ns)
    gate = CollisionGuardProofGate()
    assert gate.observe_result(
        document["collision_guard_proof"], now_ns=now_ns
    )
    gate.authorize(normalized, now_ns=now_ns)
    gate.authorize(deepcopy(normalized), now_ns=now_ns + 1)

    changed = deepcopy(normalized)
    mutate(changed)
    with pytest.raises(ValueError, match="运动合同不匹配"):
        gate.authorize(changed, now_ns=now_ns + 2)


def test_plan_manifest_gate_accepts_exact_refresh_then_strict_next_segment():
    hashes = ("c" * 64, "e" * 64)
    first_now = 90_000_000_000
    _first_document, first = normalized_plan_segment(
        index=0,
        segment_hashes=hashes,
        now_ns=first_now,
        activation_epoch=10,
        proof_sequence=1,
    )
    gate = PlanManifestGate()
    gate.authorize(first, now_ns=first_now)
    # The frozen descriptor may be refreshed long after execute_at; only a new
    # segment is subject to a fresh lead-time check.
    gate.authorize(
        deepcopy(first),
        now_ns=first["trajectory"]["execute_at_monotonic_ns"] + 1,
    )

    second_now = first_now + 1_000_000_000
    _second_document, second = normalized_plan_segment(
        index=1,
        segment_hashes=hashes,
        now_ns=second_now,
        activation_epoch=11,
        proof_sequence=2,
        source_value=0.01,
        target_value=0.02,
    )
    gate.authorize(second, now_ns=second_now)
    assert gate.tracked_token_count == 1
    assert gate.completed_token_count == 1


def test_plan_manifest_gate_rejects_skip_count_hash_and_completed_token_reuse():
    hashes = ("c" * 64, "e" * 64, "f" * 64)
    now_ns = 100_000_000_000
    _document, first = normalized_plan_segment(
        index=0,
        segment_hashes=hashes,
        now_ns=now_ns,
        activation_epoch=20,
        proof_sequence=1,
    )
    gate = PlanManifestGate()
    gate.authorize(first, now_ns=now_ns)

    _document, skipped = normalized_plan_segment(
        index=2,
        segment_hashes=hashes,
        now_ns=now_ns + 1,
        activation_epoch=21,
        proof_sequence=2,
        source_value=0.02,
        target_value=0.03,
    )
    with pytest.raises(ValueError, match="顺序或激活纪元无效"):
        gate.authorize(skipped, now_ns=now_ns + 1)

    _document, discontinuous = normalized_plan_segment(
        index=1,
        segment_hashes=hashes,
        now_ns=now_ns + 2,
        activation_epoch=21,
        proof_sequence=2,
        source_value=0.005,
        target_value=0.015,
    )
    with pytest.raises(ValueError, match="顺序或激活纪元无效"):
        gate.authorize(discontinuous, now_ns=now_ns + 2)

    _document, changed_manifest = normalized_plan_segment(
        index=1,
        segment_hashes=("c" * 64, "a" * 64, "f" * 64),
        now_ns=now_ns + 3,
        activation_epoch=21,
        proof_sequence=2,
        source_value=0.01,
        target_value=0.02,
    )
    with pytest.raises(ValueError, match="计划清单与轨迹分段不匹配"):
        gate.authorize(changed_manifest, now_ns=now_ns + 3)

    for index, (source_value, target_value) in enumerate(
        ((0.01, 0.02), (0.02, 0.03)), start=1
    ):
        segment_now = now_ns + index * 1_000_000_000
        _document, segment = normalized_plan_segment(
            index=index,
            segment_hashes=hashes,
            now_ns=segment_now,
            activation_epoch=20 + index,
            proof_sequence=1 + index,
            source_value=source_value,
            target_value=target_value,
        )
        gate.authorize(segment, now_ns=segment_now)

    _document, reused = normalized_plan_segment(
        index=0,
        segment_hashes=hashes,
        now_ns=now_ns + 4_000_000_000,
        activation_epoch=30,
        proof_sequence=9,
    )
    with pytest.raises(ValueError, match="已完成且不得复用"):
        gate.authorize(reused, now_ns=now_ns + 4_000_000_000)


def test_plan_manifest_gate_rejects_same_index_epoch_proof_or_contract_change():
    now_ns = 110_000_000_000
    _document, first = normalized_plan_segment(
        index=0,
        segment_hashes=(TRAJECTORY_SHA256,),
        now_ns=now_ns,
        activation_epoch=31,
        proof_sequence=1,
    )
    gate = PlanManifestGate()
    gate.authorize(first, now_ns=now_ns)

    changed_epoch = deepcopy(first)
    changed_epoch["activation_epoch"] += 1
    with pytest.raises(ValueError, match="同一索引发生变化"):
        gate.authorize(changed_epoch, now_ns=now_ns + 1)

    changed_proof = deepcopy(first)
    changed_proof["collision_guard_proof"]["request_sequence"] += 1
    with pytest.raises(ValueError, match="同一索引发生变化"):
        gate.authorize(changed_proof, now_ns=now_ns + 1)

    changed_target = deepcopy(first)
    changed_target["targets_rad"][0] += 0.001
    changed_target["trajectory"]["target_rad"][0] += 0.001
    with pytest.raises(ValueError, match="同一索引发生变化"):
        gate.authorize(changed_target, now_ns=now_ns + 1)


@pytest.mark.parametrize(
    ("lead_ns", "reason"),
    [
        (TRAJECTORY_MINIMUM_EXECUTE_LEAD_NS - 1, "过早或已过期"),
        (TRAJECTORY_MAXIMUM_EXECUTE_LEAD_NS + 1, "超过上界"),
    ],
)
def test_plan_manifest_gate_rejects_first_packet_outside_execute_window(
    lead_ns, reason
):
    now_ns = 120_000_000_000
    _document, command_value = normalized_plan_segment(
        index=0,
        segment_hashes=(TRAJECTORY_SHA256,),
        now_ns=now_ns,
        activation_epoch=40,
        proof_sequence=1,
    )
    command_value["trajectory"]["execute_at_monotonic_ns"] = now_ns + lead_ns
    gate = PlanManifestGate()
    with pytest.raises(ValueError, match=reason):
        gate.authorize(command_value, now_ns=now_ns)


@pytest.mark.parametrize(
    "lead_ns",
    [
        TRAJECTORY_MINIMUM_EXECUTE_LEAD_NS,
        TRAJECTORY_MAXIMUM_EXECUTE_LEAD_NS,
    ],
)
def test_plan_manifest_gate_accepts_execute_window_integer_boundaries(lead_ns):
    now_ns = 130_000_000_000
    _document, command_value = normalized_plan_segment(
        index=0,
        segment_hashes=(TRAJECTORY_SHA256,),
        now_ns=now_ns,
        activation_epoch=50,
        proof_sequence=1,
    )
    command_value["trajectory"]["execute_at_monotonic_ns"] = now_ns + lead_ns
    PlanManifestGate().authorize(command_value, now_ns=now_ns)


def test_position_without_moving_mask_rejects_ambiguous_multi_axis_move():
    document = json.loads(command())
    document.pop("moving_joint_mask")
    with pytest.raises(ValueError, match="只能选择一个移动关节"):
        validate_command(json.dumps(document))


@pytest.mark.parametrize("mode", ["drag", "hold"])
def test_selected_j2_active_control_is_forwarded(mode):
    value, _payload = validate_command(command(mode=mode))
    assert json.loads(payload_for_domain(value, "J2"))["mode"] == mode
    for domain in ("J1", "J345", "J6"):
        assert json.loads(payload_for_domain(value, domain))["mode"] == mode


def test_j2_brake_request_remains_brake():
    value, _payload = validate_command(command(mode="brake"))
    assert json.loads(payload_for_domain(value, "J2"))["mode"] == "brake"


def test_position_rejects_partial_active_mask_instead_of_unloading_other_domains():
    document = command(
        active_joint_mask=[False, False, False, True, False, False],
        moving_joint_mask=[False, False, False, True, False, False],
    )
    with pytest.raises(ValueError, match="激活全部六个关节"):
        validate_command(document)


def test_one_moving_joint_keeps_other_active_domains_in_hold():
    document = json.loads(command())
    document["targets_rad"] = [
        math.radians(2.0), math.radians(-87.86), math.radians(155.24),
        math.radians(-32.01), math.radians(8.0), math.radians(-1.0),
    ]
    document["moving_joint_mask"] = [True, False, False, False, False, False]
    document["collision_guard_proof"] = collision_guard_result(
        document["targets_rad"],
        moving_joint_mask=document["moving_joint_mask"],
    )
    value, _payload = validate_command(json.dumps(document))
    assert json.loads(payload_for_domain(value, "J1"))["mode"] == "position"
    for domain in ("J2", "J345", "J6"):
        forwarded = json.loads(payload_for_domain(value, domain))
        assert forwarded["mode"] == "hold"
        assert forwarded["active_joint_mask"] == [True] * 6
        assert forwarded["moving_joint_mask"] == [False] * 6


def test_mixed_j345_domain_moves_only_selected_joint_and_holds_peers():
    document = json.loads(command())
    document["targets_rad"] = [
        math.radians(3.0), math.radians(-87.86), math.radians(155.24),
        math.radians(-2.0), math.radians(8.0), math.radians(-1.0),
    ]
    document["moving_joint_mask"] = [False, False, False, True, False, False]
    document["collision_guard_proof"] = collision_guard_result(
        document["targets_rad"],
        moving_joint_mask=document["moving_joint_mask"],
    )
    value, _payload = validate_command(json.dumps(document))
    forwarded = json.loads(payload_for_domain(value, "J345"))
    assert forwarded["mode"] == "position"
    assert forwarded["moving_joint_mask"] == [False, False, False, True, False, False]


def test_legacy_schema_is_rejected_fail_closed():
    document = json.loads(command())
    document["schema"] = "go-m8010-gui-command/1.0"
    with pytest.raises(ValueError, match="旧版协议仅允许制动"):
        validate_command(json.dumps(document))


def test_v11_active_command_is_rejected_fail_closed():
    document = json.loads(command())
    document["schema"] = "go-m8010-gui-command/1.1"
    with pytest.raises(ValueError, match="旧版协议仅允许制动"):
        validate_command(json.dumps(document))


def test_legacy_brake_remains_available_for_emergency_compatibility():
    document = json.loads(command(mode="brake"))
    document["schema"] = "go-m8010-gui-command/1.0"
    document.pop("active_joint_mask")
    value, payload = validate_command(json.dumps(document))
    assert value["mode"] == "brake"
    assert value["active_joint_mask"] == [False] * 6
    assert json.loads(payload)["schema"] == "go-m8010-gui-command/1.2"


@pytest.mark.parametrize("mask", [None, [True] * 5, [0, False, False, False, False, False]])
def test_active_joint_mask_must_be_six_strict_booleans(mask):
    document = json.loads(command())
    if mask is None:
        document.pop("active_joint_mask")
    else:
        document["active_joint_mask"] = mask
    with pytest.raises(ValueError, match="关节激活掩码"):
        validate_command(json.dumps(document))


@pytest.mark.parametrize("mask", [None, [True] * 5, [0, False, False, False, False, False]])
def test_moving_joint_mask_must_be_six_strict_booleans_when_supplied(mask):
    document = json.loads(command())
    if mask is None:
        document["moving_joint_mask"] = None
    else:
        document["moving_joint_mask"] = mask
    with pytest.raises(ValueError, match="关节移动掩码"):
        validate_command(json.dumps(document))


def test_moving_joint_must_also_be_active():
    document = json.loads(command(
        active_joint_mask=[False, True, True, True, True, True],
        moving_joint_mask=[True, False, False, False, False, False],
    ))
    with pytest.raises(ValueError, match="移动关节必须同时激活"):
        validate_command(json.dumps(document))


def test_brake_command_with_active_joint_is_normalized_fail_closed():
    value, payload = validate_command(
        command(mode="brake", active_joint_mask=[True] + [False] * 5)
    )
    assert value["active_joint_mask"] == [False] * 6
    assert value["moving_joint_mask"] == [False] * 6
    assert json.loads(payload)["active_joint_mask"] == [False] * 6


def test_brake_command_normalizes_out_of_range_measured_targets():
    document = json.loads(command(mode="brake"))
    document["targets_rad"] = [
        math.radians(3.0), math.radians(-88.0), math.radians(155.0),
        math.radians(-32.0), math.radians(8.0), math.radians(-1.0),
    ]
    value, payload = validate_command(json.dumps(document))
    assert value["targets_rad"] == [0.0] * 6
    assert json.loads(payload)["targets_rad"] == [0.0] * 6


def test_brake_ignores_malformed_motion_authority_fields():
    document = json.loads(command(mode="brake"))
    document["targets_rad"] = "not-a-vector"
    document["active_joint_mask"] = [True] * 6
    document["moving_joint_mask"] = {"unexpected": "object"}
    document["kp"] = [float("nan")] * 6
    document["kd"] = {"unexpected": "object"}
    document["activation_epoch"] = {"unexpected": "object"}
    document["maximum_velocity_rad_s"] = float("nan")
    document["maximum_acceleration_rad_s2"] = "not-a-number"
    document["sequence"] = {"unexpected": "object"}
    value, payload = validate_command(json.dumps(document))
    encoded = json.loads(payload)
    for result in (value, encoded):
        assert result["targets_rad"] == [0.0] * 6
        assert result["active_joint_mask"] == [False] * 6
        assert result["moving_joint_mask"] == [False] * 6
        assert result["kp"] == [0.0] * 6
        assert result["kd"] == [0.0] * 6
        assert result["activation_epoch"] == 0
        assert result["sequence"] == 0


@pytest.mark.parametrize("epoch", [-1, True, 1.5, "1", 1 << 63])
def test_activation_epoch_must_be_bounded_nonnegative_integer(epoch):
    document = json.loads(command())
    document["activation_epoch"] = epoch
    with pytest.raises(ValueError, match="激活纪元"):
        validate_command(json.dumps(document))


def test_active_command_requires_nonzero_activation_epoch():
    document = json.loads(command())
    document["activation_epoch"] = 0
    with pytest.raises(ValueError, match="必须大于零"):
        validate_command(json.dumps(document))


@pytest.mark.parametrize("value", ["0.1", True, None, {}, []])
def test_active_numeric_fields_reject_coerced_values(value):
    for field in ("targets_rad", "kp", "kd"):
        document = json.loads(command())
        document.setdefault("kp", [0.5, 1.0, 0.6, 0.5, 0.5, 0.0])
        document.setdefault("kd", [0.05, 0.10, 0.05, 0.05, 0.05, 0.0])
        document[field][0] = value
        with pytest.raises(ValueError):
            validate_command(json.dumps(document))


@pytest.mark.parametrize("value", ["1", True, -1, 0, 1 << 63])
def test_active_sequence_requires_bounded_positive_integer(value):
    document = json.loads(command())
    document["sequence"] = value
    with pytest.raises(ValueError, match="主动命令序号"):
        validate_command(json.dumps(document))


@pytest.mark.parametrize("source", [None, "", "A" * 32, "0" * 31, True])
def test_non_brake_command_requires_bounded_lowercase_hex_source(source):
    document = json.loads(command())
    if source is None:
        document.pop("source_instance_id")
    else:
        document["source_instance_id"] = source
    with pytest.raises(ValueError, match="来源实例"):
        validate_command(json.dumps(document))


@pytest.mark.parametrize("source_time", [None, "1", True, 0, -1])
def test_non_brake_command_requires_strict_source_timestamp(source_time):
    now_ns = 10_000_000_000
    document = json.loads(command())
    if source_time is None:
        document.pop("source_monotonic_ns")
    else:
        document["source_monotonic_ns"] = source_time
    with pytest.raises(ValueError, match="来源时间戳"):
        validate_command(json.dumps(document), now_ns=now_ns)


def test_non_brake_command_rejects_stale_or_future_source_timestamp():
    now_ns = 10_000_000_000
    for source_time in (now_ns - 250_000_001, now_ns + 1):
        document = json.loads(command())
        document["source_monotonic_ns"] = source_time
        with pytest.raises(ValueError, match="来源时间戳"):
            validate_command(json.dumps(document), now_ns=now_ns)


def test_router_preserves_gui_source_timestamp_instead_of_refreshing_replay():
    now_ns = 10_000_000_000
    document = json.loads(command())
    document["source_monotonic_ns"] = now_ns - 123_456
    value, payload = validate_command(json.dumps(document), now_ns=now_ns)
    assert value["source_monotonic_ns"] == now_ns - 123_456
    assert json.loads(payload)["source_monotonic_ns"] == now_ns - 123_456


def test_replay_guard_rejects_duplicate_lower_sequence_and_time_regression():
    guard = CommandReplayGuard()
    now_ns = 10_000_000_000
    document = json.loads(command())
    document["source_monotonic_ns"] = now_ns - 3
    first, _payload = validate_command(json.dumps(document), now_ns=now_ns)
    guard.accept(first, now_ns=now_ns)

    with pytest.raises(ValueError, match="重放或乱序"):
        guard.accept(first, now_ns=now_ns)

    document["sequence"] = 2
    document["source_monotonic_ns"] = now_ns - 4
    regressed_time, _payload = validate_command(
        json.dumps(document), now_ns=now_ns
    )
    with pytest.raises(ValueError, match="重放或乱序"):
        guard.accept(regressed_time, now_ns=now_ns)


def test_replay_guard_accepts_strict_progress_and_stale_source_takeover():
    guard = CommandReplayGuard()
    now_ns = 10_000_000_000
    first_document = json.loads(command())
    first_document["source_monotonic_ns"] = now_ns - 3
    first, _payload = validate_command(json.dumps(first_document), now_ns=now_ns)
    guard.accept(first, now_ns=now_ns)

    first_document["sequence"] = 2
    first_document["source_monotonic_ns"] = now_ns - 2
    second, _payload = validate_command(json.dumps(first_document), now_ns=now_ns)
    guard.accept(second, now_ns=now_ns)

    first_document["source_instance_id"] = "fedcba9876543210fedcba9876543210"
    first_document["sequence"] = 1
    takeover_ns = now_ns + 500_000_001
    first_document["source_monotonic_ns"] = takeover_ns
    restarted, _payload = validate_command(
        json.dumps(first_document), now_ns=takeover_ns
    )
    guard.accept(restarted, now_ns=takeover_ns)


def test_replay_guard_rejects_competing_live_gui_but_brake_always_passes():
    guard = CommandReplayGuard()
    now_ns = 10_000_000_000
    first_document = json.loads(command())
    first_document["source_monotonic_ns"] = now_ns
    first, _payload = validate_command(json.dumps(first_document), now_ns=now_ns)
    guard.accept(first, now_ns=now_ns)

    competing_document = dict(first_document)
    competing_document["source_instance_id"] = (
        "fedcba9876543210fedcba9876543210"
    )
    competing_document["source_monotonic_ns"] = now_ns + 1
    competing, _payload = validate_command(
        json.dumps(competing_document), now_ns=now_ns + 1
    )
    with pytest.raises(ValueError, match="另一GUI来源"):
        guard.accept(competing, now_ns=now_ns + 1)

    brake_document = json.loads(command(mode="brake"))
    brake, _payload = validate_command(
        json.dumps(brake_document), now_ns=now_ns + 2
    )
    guard.accept(brake, now_ns=now_ns + 2)


def test_drag_requires_fresh_source_but_brake_ignores_malformed_source():
    drag = json.loads(command(mode="drag"))
    drag.pop("source_instance_id")
    with pytest.raises(ValueError, match="来源实例"):
        validate_command(json.dumps(drag))

    brake = json.loads(command(mode="brake"))
    brake["source_instance_id"] = {"malformed": True}
    brake["source_monotonic_ns"] = "stale"
    brake["sequence"] = {"malformed": True}
    value, _payload = validate_command(json.dumps(brake))
    assert value["mode"] == "brake"
    assert value["sequence"] == 0


def test_unknown_domain_is_rejected():
    value, _payload = validate_command(command())
    with pytest.raises(ValueError, match="未知硬件故障域"):
        payload_for_domain(value, "UNKNOWN")


def _normalized_position_for_gate(
    proof,
    *,
    now_ns,
    targets=None,
    activation_epoch=1,
    sequence=1,
):
    document = json.loads(command())
    if targets is None:
        targets = proof["target_relative_rad"]
    document.update({
        "sequence": sequence,
        "source_monotonic_ns": now_ns,
        "source_instance_id": proof["source_instance_id"],
        "targets_rad": list(targets),
        "active_joint_mask": [True] * 6,
        "moving_joint_mask": list(proof["moving_joint_mask"]),
        "activation_epoch": activation_epoch,
        "collision_guard_proof": proof,
    })
    normalized, _payload = validate_command(
        json.dumps(document), now_ns=now_ns
    )
    return normalized


def test_position_requires_exactly_one_moving_joint_all_active_and_proof_shape():
    missing = json.loads(command())
    missing.pop("collision_guard_proof")
    with pytest.raises(ValueError, match="缺少碰撞守卫证明"):
        validate_command(json.dumps(missing))

    no_move = json.loads(command())
    no_move["moving_joint_mask"] = [False] * 6
    with pytest.raises(ValueError, match="只能选择一个"):
        validate_command(json.dumps(no_move))

    multi_move = json.loads(command())
    multi_move["moving_joint_mask"] = [True, True, False, False, False, False]
    with pytest.raises(ValueError, match="只能选择一个"):
        validate_command(json.dumps(multi_move))

    partial = json.loads(command())
    partial["active_joint_mask"][5] = False
    with pytest.raises(ValueError, match="全部六个关节"):
        validate_command(json.dumps(partial))

    preview = json.loads(command())
    preview["collision_guard_proof"]["kind"] = "preview"
    with pytest.raises(ValueError, match="证明格式无效"):
        validate_command(json.dumps(preview))


def test_collision_proof_gate_accepts_independent_result_and_same_epoch_repeats():
    now_ns = 10_000_000_000
    targets = [math.radians(1.0), 0.0, 0.0, 0.0, 0.0, 0.0]
    proof = collision_guard_result(targets, now_ns=now_ns)
    gate = CollisionGuardProofGate()
    assert gate.observe_result(proof, now_ns=now_ns)
    assert gate.cached_proof_count == 1

    first = _normalized_position_for_gate(
        proof, now_ns=now_ns, activation_epoch=7, sequence=1
    )
    gate.authorize(first, now_ns=now_ns)
    assert gate.cached_proof_count == 0
    assert gate.bound_proof_count == 1

    repeated = _normalized_position_for_gate(
        proof,
        now_ns=now_ns + COLLISION_GUARD_PROOF_MAX_AGE_NS + 1,
        activation_epoch=7,
        sequence=2,
    )
    gate.authorize(
        repeated, now_ns=now_ns + COLLISION_GUARD_PROOF_MAX_AGE_NS + 1
    )


def test_same_epoch_position_contract_cannot_change_after_intermediate_hold():
    now_ns = 12_000_000_000
    first_document = command_v13(
        displacement=0.01,
        now_ns=now_ns,
    )
    first, _payload = validate_v13(first_document, now_ns=now_ns)
    gate = CollisionGuardProofGate()
    assert gate.observe_result(
        first["collision_guard_proof"], now_ns=now_ns
    )
    gate.authorize(first, now_ns=now_ns)

    hold_document = json.loads(command(mode="hold"))
    hold_document["activation_epoch"] = first["activation_epoch"]
    hold, _payload = validate_command(
        json.dumps(hold_document), now_ns=hold_document["source_monotonic_ns"]
    )
    gate.authorize(hold, now_ns=now_ns + 1)

    second_document = command_v13(
        displacement=0.015,
        now_ns=now_ns + 2,
    )
    second_document["activation_epoch"] = first["activation_epoch"]
    second_document["sequence"] = first["sequence"] + 2
    second_document["collision_guard_proof"]["request_sequence"] = 2
    second, _payload = validate_v13(second_document, now_ns=now_ns + 2)
    assert gate.observe_result(
        second["collision_guard_proof"], now_ns=now_ns + 2
    )
    with pytest.raises(ValueError, match="运动合同不匹配"):
        gate.authorize(second, now_ns=now_ns + 2)


def test_position_authority_table_exhaustion_fails_closed_without_eviction():
    now_ns = 15_000_000_000
    first_proof = collision_guard_result(
        [0.01, 0.0, 0.0, 0.0, 0.0, 0.0], now_ns=now_ns
    )
    gate = CollisionGuardProofGate(maximum_bound_proofs=1)
    assert gate.observe_result(first_proof, now_ns=now_ns)
    first = _normalized_position_for_gate(
        first_proof, now_ns=now_ns, activation_epoch=1
    )
    gate.authorize(first, now_ns=now_ns)

    second_proof = collision_guard_result(
        [0.02, 0.0, 0.0, 0.0, 0.0, 0.0],
        now_ns=now_ns + 1,
        request_sequence=2,
    )
    assert gate.observe_result(second_proof, now_ns=now_ns + 1)
    second = _normalized_position_for_gate(
        second_proof,
        now_ns=now_ns + 1,
        activation_epoch=2,
        sequence=2,
    )
    with pytest.raises(ValueError, match="位置权限冻结表已满"):
        gate.authorize(second, now_ns=now_ns + 1)
    # The original immutable epoch remains authorized; it was not evicted.
    gate.authorize(first, now_ns=now_ns + 2)


def test_collision_proof_gate_rejects_unobserved_stale_and_cross_epoch_reuse():
    now_ns = 20_000_000_000
    targets = [math.radians(2.0), 0.0, 0.0, 0.0, 0.0, 0.0]
    proof = collision_guard_result(targets, now_ns=now_ns)
    command_value = _normalized_position_for_gate(proof, now_ns=now_ns)

    with pytest.raises(ValueError, match="独立接收"):
        CollisionGuardProofGate().authorize(command_value, now_ns=now_ns)

    stale_gate = CollisionGuardProofGate()
    assert stale_gate.observe_result(proof, now_ns=now_ns)
    with pytest.raises(ValueError, match="已过期"):
        stale_gate.authorize(
            command_value,
            now_ns=now_ns + COLLISION_GUARD_PROOF_MAX_AGE_NS + 1,
        )

    gate = CollisionGuardProofGate()
    assert gate.observe_result(proof, now_ns=now_ns)
    gate.authorize(command_value, now_ns=now_ns)
    next_epoch = _normalized_position_for_gate(
        proof, now_ns=now_ns + 1, activation_epoch=2, sequence=2
    )
    with pytest.raises(ValueError, match="不得跨激活纪元重用"):
        gate.authorize(next_epoch, now_ns=now_ns + 1)


def test_collision_proof_gate_rejects_target_forgery_and_exact_result_mismatch():
    now_ns = 30_000_000_000
    targets = [math.radians(3.0), 0.0, 0.0, 0.0, 0.0, 0.0]
    proof = collision_guard_result(targets, now_ns=now_ns)
    gate = CollisionGuardProofGate()
    assert gate.observe_result(proof, now_ns=now_ns)

    target_forgery = _normalized_position_for_gate(
        proof,
        now_ns=now_ns,
        targets=[math.radians(4.0), 0.0, 0.0, 0.0, 0.0, 0.0],
    )
    with pytest.raises(ValueError, match="与运动目标不匹配"):
        gate.authorize(target_forgery, now_ns=now_ns)

    exact_mismatch = json.loads(json.dumps(proof))
    exact_mismatch["session_pose_sha256"] = "b" * 64
    mismatch_command = _normalized_position_for_gate(
        exact_mismatch, now_ns=now_ns
    )
    with pytest.raises(ValueError, match="独立接收"):
        gate.authorize(mismatch_command, now_ns=now_ns)


def test_collision_proof_gate_binds_moving_mask_profile_and_nonmoving_targets():
    now_ns = 35_000_000_000
    targets = [math.radians(0.1), math.radians(0.1), 0.0, 0.0, 0.0, 0.0]
    proof = collision_guard_result(targets, now_ns=now_ns)
    gate = CollisionGuardProofGate()
    assert gate.observe_result(proof, now_ns=now_ns)
    first = _normalized_position_for_gate(proof, now_ns=now_ns)
    gate.authorize(first, now_ns=now_ns)

    changed_mask = dict(first)
    changed_mask["moving_joint_mask"] = [False, True, False, False, False, False]
    with pytest.raises(ValueError, match="运动合同不匹配"):
        gate.authorize(changed_mask, now_ns=now_ns + 1)

    changed_profile = dict(first)
    changed_profile["maximum_velocity_rad_s"] *= 0.5
    with pytest.raises(ValueError, match="运动合同不匹配"):
        gate.authorize(changed_profile, now_ns=now_ns + 1)

    far_nonmoving_targets = [
        math.radians(3.0), math.radians(1.0), 0.0, 0.0, 0.0, 0.0
    ]
    far_nonmoving_proof = collision_guard_result(
        far_nonmoving_targets,
        now_ns=now_ns + 2,
        request_sequence=2,
        start_relative_rad=[0.0] * 6,
    )
    second_gate = CollisionGuardProofGate()
    assert second_gate.observe_result(far_nonmoving_proof, now_ns=now_ns + 2)
    far_nonmoving = _normalized_position_for_gate(
        far_nonmoving_proof, now_ns=now_ns + 2, activation_epoch=2
    )
    with pytest.raises(ValueError, match="运动合同不匹配"):
        second_gate.authorize(far_nonmoving, now_ns=now_ns + 2)


@pytest.mark.parametrize(
    "mutation",
    [
        lambda result: result.update(schema="wrong"),
        lambda result: result.update(model_sha256="0" * 64),
        lambda result: result.update(kinematic_guard_sha256="0" * 64),
        lambda result: result.update(collision_contract_sha256="0" * 64),
        lambda result: result.update(session_pose_sha256="not-a-sha256"),
        lambda result: result.update(collision_margin_deg=1.999),
        lambda result: result.update(target_sha256="0" * 64),
        lambda result: result.update(checked_monotonic_ns=0),
    ],
)
def test_collision_proof_gate_does_not_cache_invalid_results(mutation):
    now_ns = 40_000_000_000
    result = collision_guard_result([0.0] * 6, now_ns=now_ns)
    mutation(result)
    gate = CollisionGuardProofGate()
    assert not gate.observe_result(result, now_ns=now_ns)
    assert gate.cached_proof_count == 0


def test_preview_and_unsafe_results_never_authorize_and_result_replay_is_ignored():
    now_ns = 50_000_000_000
    targets = [0.0] * 6
    preview = collision_guard_result(
        targets, now_ns=now_ns, request_sequence=1, kind="preview"
    )
    unsafe = collision_guard_result(
        targets,
        now_ns=now_ns + 10,
        request_sequence=2,
        kind="execute",
        safe=False,
    )
    gate = CollisionGuardProofGate()
    assert gate.observe_result(preview, now_ns=now_ns)
    assert gate.cached_proof_count == 0
    assert gate.observe_result(unsafe, now_ns=now_ns + 10)
    assert gate.cached_proof_count == 0
    assert not gate.observe_result(unsafe, now_ns=now_ns + 10)


def test_later_preview_invalidates_older_unconsumed_execute_proof():
    now_ns = 55_000_000_000
    targets = [math.radians(1.0), 0.0, 0.0, 0.0, 0.0, 0.0]
    execute = collision_guard_result(
        targets, now_ns=now_ns, request_sequence=1, kind="execute"
    )
    preview = collision_guard_result(
        targets, now_ns=now_ns + 1, request_sequence=2, kind="preview"
    )
    gate = CollisionGuardProofGate()
    assert gate.observe_result(execute, now_ns=now_ns)
    assert gate.cached_proof_count == 1
    assert gate.observe_result(preview, now_ns=now_ns + 1)
    assert gate.cached_proof_count == 0
    command_value = _normalized_position_for_gate(
        execute, now_ns=now_ns + 1
    )
    with pytest.raises(ValueError, match="独立接收"):
        gate.authorize(command_value, now_ns=now_ns + 1)


@pytest.mark.parametrize("mode", ["brake", "drag", "hold", "position"])
def test_collision_guard_proof_never_leaks_to_hardware_domain_payloads(mode):
    document = json.loads(command(mode=mode))
    if mode == "hold":
        document["collision_guard_proof"] = collision_guard_result(
            document["targets_rad"]
        )
    value, _payload = validate_command(json.dumps(document))
    if mode in {"brake", "drag"}:
        value["collision_guard_proof"] = {"malformed": "must still be stripped"}
    for domain in ("J1", "J2", "J345", "J6"):
        hardware_payload = json.loads(payload_for_domain(value, domain))
        assert hardware_payload["schema"] == "go-m8010-gui-command/1.2"
        assert "collision_guard_proof" not in hardware_payload


def test_router_rejects_unobserved_proof_without_brake_or_replay_commit():
    class FakeSocket:
        def __init__(self):
            self.sent = []

        def sendto(self, payload, destination):
            self.sent.append((json.loads(payload), destination))

    fake = type("FakeRouter", (), {})()
    fake.socket = FakeSocket()
    fake.destinations = [
        ("J1", ("127.0.0.1", 1)),
        ("J2", ("127.0.0.1", 2)),
        ("J345", ("127.0.0.1", 3)),
        ("J6", ("127.0.0.1", 4)),
    ]
    fake.last_command = None
    fake.replay_guard = CommandReplayGuard()
    fake.collision_guard_gate = CollisionGuardProofGate()
    fake.plan_manifest_gate = PlanManifestGate()
    fake.gravity_authority_gate = GravityAuthorityGate()
    fake.allow_legacy_v12_position = True
    fake.rejection_tracker = RejectionTracker()
    fake.rejected = 0
    fake._log_rejection_reports = lambda _reports: None

    text = command()
    CommandRouter.on_command(fake, type("Message", (), {"data": text})())
    assert fake.socket.sent == []
    assert fake.last_command is None
    assert fake.rejected == 1

    document = json.loads(text)
    assert fake.collision_guard_gate.observe_result(
        document["collision_guard_proof"]
    )
    # The first proof rejection did not consume the command replay sequence.
    CommandRouter.on_command(fake, type("Message", (), {"data": text})())
    assert len(fake.socket.sent) == 4
    assert fake.last_command is not None
    assert all(payload["mode"] != "brake" for payload, _ in fake.socket.sent)


def test_router_checks_first_segment_execute_window_before_any_domain_send():
    class FakeSocket:
        def __init__(self):
            self.sent = []

        def sendto(self, payload, destination):
            self.sent.append((json.loads(payload), destination))

    fake = type("FakeRouter", (), {})()
    fake.socket = FakeSocket()
    fake.destinations = [
        ("J1", ("127.0.0.1", 1)),
        ("J2", ("127.0.0.1", 2)),
        ("J345", ("127.0.0.1", 3)),
        ("J6", ("127.0.0.1", 4)),
    ]
    fake.last_command = None
    fake.replay_guard = CommandReplayGuard()
    fake.collision_guard_gate = CollisionGuardProofGate()
    fake.plan_manifest_gate = PlanManifestGate()
    fake.gravity_authority_gate = GravityAuthorityGate()
    fake.allow_legacy_v12_position = False
    fake.rejection_tracker = RejectionTracker()
    fake.rejected = 0
    fake._log_rejection_reports = lambda _reports: None

    now_ns = time.monotonic_ns()
    document = command_v13(now_ns=now_ns)
    document["trajectory"]["execute_at_monotonic_ns"] = now_ns + 1
    assert fake.collision_guard_gate.observe_result(
        document["collision_guard_proof"], now_ns=time.monotonic_ns()
    )
    CommandRouter.on_command(
        fake,
        type("Message", (), {"data": json.dumps(document)})(),
    )

    assert fake.socket.sent == []
    assert fake.last_command is None
    assert fake.rejected == 1


def test_collision_proof_cache_is_bounded_and_rejects_result_reordering():
    now_ns = 60_000_000_000
    gate = CollisionGuardProofGate(maximum_cached_proofs=2)
    for sequence in (1, 2, 3):
        proof = collision_guard_result(
            [math.radians(sequence), 0.0, 0.0, 0.0, 0.0, 0.0],
            now_ns=now_ns + sequence,
            source_instance_id=f"{sequence:032x}",
            request_sequence=1,
        )
        assert gate.observe_result(proof, now_ns=now_ns + sequence)
    assert gate.cached_proof_count == 2
    reordered = collision_guard_result(
        [0.0] * 6,
        now_ns=now_ns + 4,
        source_instance_id=f"{3:032x}",
        request_sequence=1,
    )
    assert not gate.observe_result(reordered, now_ns=now_ns + 4)
    assert gate.cached_proof_count == 2


@pytest.mark.parametrize("mode", ["real_to_sim", "sim_to_real", "torque"])
def test_non_hardware_modes_rejected(mode):
    with pytest.raises(ValueError):
        validate_command(command(mode=mode))


def test_out_of_envelope_rejected():
    document = json.loads(command())
    document["targets_rad"][1] = math.radians(80.01)
    with pytest.raises(ValueError, match="模型机械限位"):
        validate_command(json.dumps(document))


def test_hold_target_uses_full_mechanical_envelope_not_small_move_window():
    document = json.loads(command(mode="hold"))
    document["targets_rad"] = [
        math.radians(120.0), math.radians(-87.86), math.radians(155.24),
        math.radians(-32.01), math.radians(8.0), math.radians(-100.0),
    ]
    value, _payload = validate_command(json.dumps(document))
    assert value["mode"] == "hold"
    assert value["targets_rad"] == document["targets_rad"]


def test_position_rejects_multiple_moving_joints_at_production_boundary():
    document = json.loads(command())
    document["targets_rad"] = [
        0.0, math.radians(-87.86), math.radians(155.24),
        math.radians(-32.01), math.radians(8.0), 0.0,
    ]
    document["moving_joint_mask"] = [True, False, False, False, False, True]
    with pytest.raises(ValueError, match="只能选择一个移动关节"):
        validate_command(json.dumps(document))


def test_hold_target_outside_model_mechanical_envelope_is_rejected():
    document = json.loads(command(mode="hold"))
    document["targets_rad"][1] = math.radians(-260.01)
    with pytest.raises(ValueError, match="关节目标超出模型机械限位"):
        validate_command(json.dumps(document))


@pytest.mark.parametrize("mode", ["position", "hold"])
def test_position_and_hold_accept_every_full_model_endpoint(mode):
    for endpoint_set in (MODEL_COMMAND_LOWER_RAD, MODEL_COMMAND_UPPER_RAD):
        document = json.loads(command(mode=mode))
        document["targets_rad"] = list(endpoint_set)
        if mode == "position":
            document["collision_guard_proof"] = collision_guard_result(
                document["targets_rad"],
                moving_joint_mask=document["moving_joint_mask"],
            )
        value, _payload = validate_command(json.dumps(document))
        assert value["targets_rad"] == list(endpoint_set)


@pytest.mark.parametrize("mode", ["position", "hold"])
@pytest.mark.parametrize("joint_index", range(6))
def test_position_and_hold_reject_only_beyond_each_model_endpoint(mode, joint_index):
    document = json.loads(command(mode=mode))
    document["targets_rad"] = [0.0] * 6
    document["targets_rad"][joint_index] = (
        MODEL_COMMAND_UPPER_RAD[joint_index] + math.radians(0.01)
    )
    with pytest.raises(ValueError, match="模型机械限位"):
        validate_command(json.dumps(document))


@pytest.mark.parametrize("mode", ["position", "hold"])
def test_rejected_out_of_range_target_preserves_existing_authority_and_sends_no_brake(mode):
    class FakeSocket:
        def __init__(self):
            self.sent = []

        def sendto(self, payload, destination):
            self.sent.append((json.loads(payload), destination))

    fake = type("FakeRouter", (), {})()
    fake.socket = FakeSocket()
    fake.destinations = [
        ("J1", ("127.0.0.1", 1)),
        ("J2", ("127.0.0.1", 2)),
        ("J345", ("127.0.0.1", 3)),
        ("J6", ("127.0.0.1", 4)),
    ]
    fake.last_command = None
    fake.replay_guard = CommandReplayGuard()
    fake.collision_guard_gate = CollisionGuardProofGate()
    fake.plan_manifest_gate = PlanManifestGate()
    fake.gravity_authority_gate = GravityAuthorityGate()
    fake.allow_legacy_v12_position = True
    fake.rejection_tracker = RejectionTracker()
    fake.rejected = 0
    fake._log_rejection_reports = lambda _reports: None

    if mode == "hold":
        gravity_now_ns = time.monotonic_ns()
        assert fake.gravity_authority_gate.observe_status(
            gravity_status(now_ns=gravity_now_ns), now_ns=gravity_now_ns
        )
    accepted_text = command(mode=mode)
    if mode == "position":
        accepted_document = json.loads(accepted_text)
        assert fake.collision_guard_gate.observe_result(
            accepted_document["collision_guard_proof"]
        )
    CommandRouter.on_command(
        fake, type("Message", (), {"data": accepted_text})()
    )
    accepted = fake.last_command
    sent_before_rejection = list(fake.socket.sent)
    assert accepted is not None and accepted["mode"] == mode
    assert len(sent_before_rejection) == 4
    assert all(payload["mode"] != "brake" for payload, _ in sent_before_rejection)

    invalid = json.loads(command(mode=mode))
    invalid["sequence"] = 2
    invalid["source_monotonic_ns"] = time.monotonic_ns()
    invalid["targets_rad"][1] = MODEL_COMMAND_UPPER_RAD[1] + math.radians(0.01)
    CommandRouter.on_command(
        fake,
        type("Message", (), {"data": json.dumps(invalid)})(),
    )

    assert fake.last_command is accepted
    assert fake.last_command["mode"] == mode
    assert fake.socket.sent == sent_before_rejection
    assert fake.rejected == 1


def test_real_arm_gui_yaml_gain_contract_accepts_hold_and_normalizes_brake():
    config = yaml.safe_load(CONFIG_PATH.read_text(encoding="utf-8"))
    joints = config["关节"]
    kp = [float(joints[f"J{index}"].get("Kp", 0.0)) for index in range(1, 7)]
    kd = [float(joints[f"J{index}"].get("Kd", 0.0)) for index in range(1, 7)]
    assert tuple(kp) == KP_LIMITS
    assert tuple(kd) == KD_LIMITS

    hold = json.loads(command(mode="hold"))
    hold["kp"] = kp
    hold["kd"] = kd
    hold_value, hold_payload = validate_command(json.dumps(hold))
    assert hold_value["kp"] == kp
    assert hold_value["kd"] == kd
    assert json.loads(hold_payload)["mode"] == "hold"

    brake = dict(hold)
    brake["mode"] = "brake"
    brake["targets_rad"] = [
        math.radians(float(joints[f"J{index}"]["最小角度_度"]))
        for index in range(1, 7)
    ]
    brake["active_joint_mask"] = [True] * 6
    brake["kp"] = [item + 100.0 for item in kp]
    brake["kd"] = [item + 100.0 for item in kd]
    brake_value, brake_payload = validate_command(json.dumps(brake))
    for result in (brake_value, json.loads(brake_payload)):
        assert result["targets_rad"] == [0.0] * 6
        assert result["active_joint_mask"] == [False] * 6
        assert result["moving_joint_mask"] == [False] * 6
        assert result["kp"] == [0.0] * 6
        assert result["kd"] == [0.0] * 6


def test_drag_ignores_motion_fields_and_reaches_every_domain_as_drag():
    document = json.loads(command(mode="drag"))
    document["targets_rad"] = "not-a-vector"
    document["active_joint_mask"] = "not-a-mask"
    document["moving_joint_mask"] = "not-a-mask"
    document["kp"] = [float("nan")] * 6
    document["activation_epoch"] = {"unexpected": "object"}
    value, _payload = validate_command(json.dumps(document))
    assert value["active_joint_mask"] == [True] * 6
    assert value["moving_joint_mask"] == [False] * 6
    for domain in ("J1", "J2", "J345", "J6"):
        assert json.loads(payload_for_domain(value, domain))["mode"] == "drag"


@pytest.mark.parametrize("field,limits", [("kp", KP_LIMITS), ("kd", KD_LIMITS)])
def test_active_gain_above_frozen_worker_limit_is_rejected(field, limits):
    document = json.loads(command(mode="hold"))
    document["kp"] = list(KP_LIMITS)
    document["kd"] = list(KD_LIMITS)
    document[field][0] = limits[0] + 0.001
    with pytest.raises(ValueError, match="增益超过冻结上限"):
        validate_command(json.dumps(document))


def test_rejection_tracker_counts_exactly_and_aggregates_by_reason():
    tracker = RejectionTracker(log_interval_seconds=1.0)
    first = tracker.record(ValueError("关节目标超出模型机械限位"), now_ns=0)
    assert len(first) == 1 and first[0]["initial"]

    for now_ns in range(1, 101):
        assert tracker.record(
            ValueError("关节目标超出模型机械限位"),
            now_ns=now_ns,
        ) == []
    assert tracker.total == 101
    assert tracker.last_rejection_ns == 100
    assert tracker.by_reason == {"关节目标超出模型机械限位": 101}

    summary = tracker.flush(now_ns=1_000_000_000)
    assert len(summary) == 1
    assert summary[0]["suppressed_since_last"] == 100
    assert summary[0]["reason_count"] == 101
    assert summary[0]["rejected_total"] == 101

    second_reason = tracker.record(
        ValueError("关节增益超过冻结上限"), now_ns=1_000_000_001
    )
    assert len(second_reason) == 1 and second_reason[0]["initial"]
    assert tracker.total == 102
    assert tracker.last_rejection_ns == 1_000_000_001
    assert tracker.by_reason["关节增益超过冻结上限"] == 1


@pytest.mark.parametrize("field,value", [
    ("maximum_velocity_rad_s", float("nan")),
    ("maximum_velocity_rad_s", float("inf")),
    ("maximum_velocity_rad_s", "0.1"),
    ("maximum_velocity_rad_s", True),
    ("maximum_velocity_rad_s", -0.1),
    ("maximum_acceleration_rad_s2", 0.0),
    ("maximum_acceleration_rad_s2", "0.1"),
    ("maximum_acceleration_rad_s2", False),
    ("maximum_acceleration_rad_s2", -0.1),
])
def test_invalid_profile_rejected(field, value):
    document = json.loads(command())
    document[field] = value
    with pytest.raises(ValueError):
        validate_command(json.dumps(document))

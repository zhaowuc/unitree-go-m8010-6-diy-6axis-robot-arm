import json
import math
import time
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
    MODEL_COMMAND_LOWER_RAD,
    MODEL_COMMAND_UPPER_RAD,
    MOTOR_NAMES,
    PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG,
    PRODUCTION_COLLISION_CONTRACT_SHA256,
    PRODUCTION_KINEMATIC_GUARD_SHA256,
    PRODUCTION_MODEL_SHA256,
    CollisionGuardProofGate,
    CommandRouter,
    CommandReplayGuard,
    RejectionTracker,
    canonical_collision_hardware_state_sha256,
    canonical_six_doubles_sha256,
    payload_for_domain,
    validate_command,
)


CONFIG_PATH = Path(__file__).resolve().parents[1] / "config" / "arm_gui.yaml"
COMMAND_SOURCE = "0123456789abcdef0123456789abcdef"
STATE_SOURCE = "fedcba9876543210fedcba9876543210"
SESSION_POSE_SHA256 = "a" * 64


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


def test_valid_six_joint_command():
    value, payload = validate_command(command())
    assert value["mode"] == "position" and len(value["targets_rad"]) == 6
    assert json.loads(payload)["schema"] == "go-m8010-gui-command/1.2"
    assert value["source_instance_id"] == "0123456789abcdef0123456789abcdef"


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
    fake.rejection_tracker = RejectionTracker()
    fake.rejected = 0
    fake._log_rejection_reports = lambda _reports: None

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

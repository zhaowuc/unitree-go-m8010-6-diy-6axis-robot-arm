import hashlib
import importlib.util
import io
import json
import math
from pathlib import Path
import struct
import sys
import threading
import time
from dataclasses import replace
from types import ModuleType, SimpleNamespace
from unittest import mock

import numpy  # Imported before temporary ROS module stubs are installed.
import pytest


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "go_m8010_arm_hardware"
    / "mujoco_mirror_node.py"
)


def load_module():
    package_name = "mujoco_mirror_guard_test_package"
    module_name = f"{package_name}.mujoco_mirror_node"
    package = ModuleType(package_name)
    package.__path__ = []
    state_model = ModuleType(f"{package_name}.state_model")
    state_model.JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
    state_model.MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
    rclpy = ModuleType("rclpy")
    node_module = ModuleType("rclpy.node")

    class Node:
        def destroy_node(self):
            self.base_destroyed = True
            return True

    node_module.Node = Node
    executors_module = ModuleType("rclpy.executors")
    executors_module.ExternalShutdownException = type(
        "ExternalShutdownException", (Exception,), {}
    )
    signals_module = ModuleType("rclpy.signals")
    signals_module.SignalHandlerOptions = SimpleNamespace(NO=object())
    sensor_msgs = ModuleType("sensor_msgs")
    sensor_msgs_msg = ModuleType("sensor_msgs.msg")
    sensor_msgs_msg.JointState = type("JointState", (), {})
    std_msgs = ModuleType("std_msgs")
    std_msgs_msg = ModuleType("std_msgs.msg")
    std_msgs_msg.Float64MultiArray = type("Float64MultiArray", (), {})
    class String:
        def __init__(self, *, data=""):
            self.data = data

    std_msgs_msg.String = String
    stubs = {
        package_name: package,
        f"{package_name}.state_model": state_model,
        "rclpy": rclpy,
        "rclpy.node": node_module,
        "rclpy.executors": executors_module,
        "rclpy.signals": signals_module,
        "sensor_msgs": sensor_msgs,
        "sensor_msgs.msg": sensor_msgs_msg,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }
    with mock.patch.dict(sys.modules, stubs):
        spec = importlib.util.spec_from_file_location(module_name, SOURCE)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
    return module


@pytest.fixture(scope="module")
def mirror():
    return load_module()


def request_payload(mirror, now_ns, *, start=None, target=None):
    start = [0.0] * 6 if start is None else list(start)
    target = [0.0] * 6 if target is None else list(target)
    return {
        "schema": mirror.COLLISION_GUARD_REQUEST_SCHEMA,
        "source_instance_id": "1" * 32,
        "request_sequence": 7,
        "source_monotonic_ns": now_ns - 1_000_000,
        "kind": "preview",
        "session_id": "persistent:test-session",
        "session_pose_sha256": mirror.canonical_six_doubles_sha256([0.0] * 6),
        "state_instance_id": "2" * 32,
        "moving_joint_mask": [True, False, False, False, False, False],
        "start_relative_rad": start,
        "target_relative_rad": target,
        "target_sha256": mirror.canonical_six_doubles_sha256(target),
        "collision_margin_deg": 2.0,
    }


def hardware_payload(now_ns, *, position=None):
    motor_names = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
    return {
        "schema": "go-m8010-hardware-state/1.1",
        "joint_names": [f"joint{index}" for index in range(1, 7)],
        "session_id": "persistent:test-session",
        "state_instance_id": "2" * 32,
        "sequence": 91,
        "source_monotonic_ns": now_ns - 2_000_000,
        "position_rad": [0.0] * 6 if position is None else list(position),
        "velocity_rad_s": [0.0] * 6,
        "per_motor": {
            name: {
                "fresh": True,
                "reference_captured": True,
                "communication_ok": True,
                "merror": 0,
            }
            for name in motor_names
        },
        "controller_mode_by_motor": {name: "hold" for name in motor_names},
        "controller_fault_by_motor": {name: False for name in motor_names},
        "j2_sync_fault": False,
        "initial_pose_sha256": "a" * 64,
    }


def parsed_request(mirror, target_deg, *, start_deg=None, moving_index=None):
    start_deg = [0.0] * 6 if start_deg is None else list(start_deg)
    if moving_index is None:
        changed = [
            index
            for index, (start, target) in enumerate(zip(start_deg, target_deg))
            if abs(float(target) - float(start)) > 1.0e-12
        ]
        moving_index = changed[0] if len(changed) == 1 else 0
    start = [math.radians(value) for value in start_deg]
    target = [math.radians(value) for value in target_deg]
    modes = {name: "hold" for name in mirror.MOTOR_NAMES}
    proof_state = {
        "schema": "go-m8010-hardware-state/1.1",
        "session_id": "session",
        "state_instance_id": "2" * 32,
        "sequence": 1,
        "source_monotonic_ns": 1,
        "position_rad": tuple(start),
        "velocity_rad_s": (0.0,) * 6,
        "controller_mode_by_motor": modes,
    }
    return mirror.CollisionGuardRequest(
        source_instance_id="1" * 32,
        request_sequence=1,
        source_monotonic_ns=1,
        kind="preview",
        session_id="session",
        session_pose_sha256=mirror.canonical_six_doubles_sha256([0.0] * 6),
        state_instance_id="2" * 32,
        moving_joint_mask=tuple(index == moving_index for index in range(6)),
        start_relative_rad=tuple(start),
        target_relative_rad=tuple(target),
        target_sha256=mirror.canonical_six_doubles_sha256(target),
        collision_margin_deg=2.0,
        hardware_state_sequence=1,
        hardware_state_source_monotonic_ns=1,
        hardware_position_rad=tuple(start),
        hardware_velocity_rad_s=(0.0,) * 6,
        hardware_controller_mode_by_motor=tuple(modes.items()),
        hardware_state_sha256=mirror.canonical_collision_hardware_state_sha256(
            proof_state
        ),
    )


def assert_strict_fail_closed_result_matches(mirror, result, request, now_ns):
    """Mirror the GUI's exact proof-envelope checks for an unsafe result."""

    assert result["schema"] == mirror.COLLISION_GUARD_RESULT_SCHEMA
    for field in (
        "source_instance_id",
        "request_sequence",
        "source_monotonic_ns",
        "kind",
        "session_id",
        "session_pose_sha256",
        "state_instance_id",
        "moving_joint_mask",
        "start_relative_rad",
        "target_relative_rad",
        "target_sha256",
        "collision_margin_deg",
    ):
        assert result[field] == request[field]
    assert request["source_monotonic_ns"] <= result["checked_monotonic_ns"] <= now_ns
    assert now_ns - result["checked_monotonic_ns"] <= 8_000_000_000
    assert result["safe"] is False
    assert isinstance(result["reason"], str) and result["reason"]
    assert result["recommended_relative_rad"] is None
    assert result["contact_pairs"] == []
    assert result["model_sha256"] == mirror.PRODUCTION_MODEL_SHA256
    assert (
        result["kinematic_guard_sha256"]
        == mirror.PRODUCTION_KINEMATIC_GUARD_SHA256
    )
    assert (
        result["collision_contract_sha256"]
        == mirror.PRODUCTION_COLLISION_CONTRACT_SHA256
    )
    assert result["ground_contact_policy"] == (
        "UNSAFE_GROUND_COLLISION_NOT_SELF_COLLISION"
    )
    assert result["max_step_deg"] == mirror.COLLISION_GUARD_MAX_STEP_DEG
    assert (
        result["boundary_tolerance_deg"]
        == mirror.COLLISION_BOUNDARY_TOLERANCE_DEG
    )
    assert result["margin_policy"] == mirror.COLLISION_MARGIN_POLICY
    assert result["joint_space_tube_radius_deg"] == pytest.approx(2.25)
    assert result["hold_tracking_tolerance_deg"] == pytest.approx(0.25)
    assert result["tube_probe_joint_names"] == list(mirror.MUJOCO_JOINT_NAMES)
    assert result["tube_grid_offsets"] == [-1, 0, 1]
    assert result["tube_grid_max_pose_count"] == 729
    assert result["tube_axis_probe_step_deg"] == pytest.approx(0.25)
    assert result["tube_cross_section_max_pose_count"] == 825
    assert result["absolute_joint_limits_deg"] == [
        list(bounds) for bounds in mirror.PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG
    ]
    assert result["single_joint_path_required"] is True


class ThresholdGuard:
    def __init__(self, *, collision_at=None, collision_class="self_collision"):
        self.collision_at = collision_at
        self.collision_class = collision_class

    def check_pose_deg(self, values):
        values = [float(value) for value in values]
        if values[1] < -170.0 - 1.0e-9 or values[1] > 170.0 + 1.0e-9:
            return {
                "safe": False,
                "reason": "position_limit",
                "contacts": [],
            }
        if self.collision_at is not None and values[0] >= self.collision_at:
            is_ground = self.collision_class == "ground"
            return {
                "safe": False,
                "reason": "ground_collision" if is_ground else "self_collision",
                "contacts": [
                    {
                        "contact_class": "ground" if is_ground else "self_collision",
                        "proxy_pair": (
                            ["J6_Gripper_Connector_Collision_Proxy", "ground"]
                            if is_ground
                            else ["Forearm_Collision_Proxy", "J1_Fixed_Collision_Proxy"]
                        ),
                    }
                ],
            }
        return {"safe": True, "reason": "clear", "contacts": []}


class ExactTargetCollisionGuard(ThresholdGuard):
    def check_pose_deg(self, values):
        values = [float(value) for value in values]
        if abs(values[0] - 9.0) <= 1.0e-10:
            return {
                "safe": False,
                "reason": "self_collision",
                "contacts": [{
                    "contact_class": "self_collision",
                    "proxy_pair": ["A", "B"],
                }],
            }
        return {"safe": True, "reason": "clear", "contacts": []}


def collision_result(pair=("A", "B")):
    return {
        "safe": False,
        "reason": "self_collision",
        "contacts": [{
            "contact_class": "self_collision",
            "proxy_pair": list(pair),
        }],
    }


class PerpendicularThresholdGuard(ThresholdGuard):
    """Collision exists only off the J1 motion ray, along positive J2."""

    def check_pose_deg(self, values):
        values = [float(value) for value in values]
        # Deliberately clear again by the +/-2.25-degree tube boundary.  This
        # defeats endpoint-only probes and proves the interior axis sweep.
        if 0.99 <= values[1] <= 1.01:
            return collision_result(("J2-offset-obstacle", "arm"))
        return {"safe": True, "reason": "clear", "contacts": []}


class SupportTrackingThresholdGuard(ThresholdGuard):
    """Collision exists at the positive endpoint of J2's HOLD gate."""

    def check_pose_deg(self, values):
        values = [float(value) for value in values]
        if 0.249 <= values[1] <= 0.251:
            return collision_result(("J2-tracking-edge-obstacle", "arm"))
        return {"safe": True, "reason": "clear", "contacts": []}


class TwoAxisCornerGuard(ThresholdGuard):
    """Single-axis probes are clear; a J3/J4 tube corner collides."""

    def check_pose_deg(self, values):
        values = [float(value) for value in values]
        if values[2] >= 2.0 and values[3] >= 2.0:
            return collision_result(("J3-J4-corner-obstacle", "arm"))
        return {"safe": True, "reason": "clear", "contacts": []}


def engine(mirror, guard):
    return mirror.CollisionGuardEngine(
        guard,
        [0.0] * 6,
        absolute_joint_limits_deg=(
            (-180.0, 180.0),
            (-170.0, 170.0),
            (-170.0, 170.0),
            (-116.0, 159.0),
            (-70.6, 151.2),
            (-180.0, 180.0),
        ),
        model_sha256=mirror.PRODUCTION_MODEL_SHA256,
        guard_sha256=mirror.PRODUCTION_KINEMATIC_GUARD_SHA256,
        contract_sha256=mirror.PRODUCTION_COLLISION_CONTRACT_SHA256,
    )


def test_canonical_target_digest_is_network_order_binary64(mirror):
    values = [0.0, -0.0, 1.25, -2.5, math.pi, 1.0e-9]
    expected = hashlib.sha256(struct.pack(">6d", *values)).hexdigest()
    assert mirror.canonical_six_doubles_sha256(values) == expected


def test_request_is_bound_to_fresh_hardware_state_and_replay_order(mirror):
    now_ns = 9_000_000_000
    value = request_payload(mirror, now_ns)
    request = mirror.parse_collision_guard_request(
        value,
        now_ns=now_ns,
        hardware_state=hardware_payload(now_ns),
        expected_session_pose_sha256=mirror.canonical_six_doubles_sha256(
            [0.0] * 6
        ),
    )
    assert request.target_sha256 == value["target_sha256"]
    assert request.moving_joint_mask == (True, False, False, False, False, False)
    assert request.hardware_state_sequence == 91
    assert request.hardware_velocity_rad_s == (0.0,) * 6
    assert dict(request.hardware_controller_mode_by_motor) == {
        name: "hold" for name in mirror.MOTOR_NAMES
    }
    assert mirror.valid_sha256(request.hardware_state_sha256)
    with pytest.raises(ValueError, match="replayed"):
        mirror.parse_collision_guard_request(
            value,
            now_ns=now_ns,
            hardware_state=hardware_payload(now_ns),
            expected_session_pose_sha256=mirror.canonical_six_doubles_sha256(
                [0.0] * 6
            ),
            previous_request=(request.request_sequence, request.source_monotonic_ns),
        )


def test_multi_joint_pose_preview_is_non_authorizing_and_checks_target_tube(mirror):
    now_ns = 9_100_000_000
    target = [math.radians(5.0), math.radians(5.0), 0.0, 0.0, 0.0, 0.0]
    value = request_payload(mirror, now_ns, target=target)
    value["kind"] = "pose_preview"
    value["moving_joint_mask"] = [True, True, False, False, False, False]
    request = mirror.parse_collision_guard_request(
        value,
        now_ns=now_ns,
        hardware_state=hardware_payload(now_ns),
        expected_session_pose_sha256=value["session_pose_sha256"],
    )
    assert request.kind == "pose_preview"
    assert sum(request.moving_joint_mask) == 2

    clear = engine(mirror, ThresholdGuard()).evaluate(request)
    assert clear["safe"] is True
    assert clear["moving_joint"] is None
    assert clear["moving_joint_count"] == 2
    assert clear["step_count"] == 0
    assert clear["path_sample_count"] == 1

    blocked = engine(mirror, ThresholdGuard(collision_at=4.0)).evaluate(request)
    assert blocked["safe"] is False
    assert blocked["reason"] == "self_collision_margin"
    assert blocked["moving_joint"] is None
    assert blocked["moving_joint_count"] == 2
    assert blocked["contact_pairs"]


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (lambda value: value.update(target_sha256="0" * 64), "does not bind"),
        (lambda value: value.update(session_id="wrong"), "session_mismatch"),
        (
            lambda value: value.update(session_pose_sha256="f" * 64),
            "session_pose_mismatch",
        ),
        (lambda value: value.update(state_instance_id="3" * 32), "state_instance_mismatch"),
        (
            lambda value: value.update(start_relative_rad=[math.radians(0.26)] + [0.0] * 5),
            "start_state_mismatch",
        ),
        (lambda value: value.update(collision_margin_deg=1.99), "exactly 2.0"),
        (
            lambda value: value.update(moving_joint_mask=[False] * 6),
            "exactly one true",
        ),
        (
            lambda value: value.update(
                moving_joint_mask=[True, True, False, False, False, False]
            ),
            "exactly one true",
        ),
        (
            lambda value: value.update(
                target_relative_rad=[0.0, math.radians(0.251), 0.0, 0.0, 0.0, 0.0],
                target_sha256=hashlib.sha256(
                    struct.pack(">6d", 0.0, math.radians(0.251), 0.0, 0.0, 0.0, 0.0)
                ).hexdigest(),
            ),
            "support_joint_start_mismatch",
        ),
    ],
)
def test_request_fail_closed_bindings(mirror, mutation, message):
    now_ns = 9_000_000_000
    value = request_payload(mirror, now_ns)
    mutation(value)
    with pytest.raises(ValueError, match=message):
        mirror.parse_collision_guard_request(
            value,
            now_ns=now_ns,
            hardware_state=hardware_payload(now_ns),
            expected_session_pose_sha256=mirror.canonical_six_doubles_sha256(
                [0.0] * 6
            ),
        )


def test_moving_mask_cannot_hide_motion_on_a_support_joint(mirror):
    now_ns = 9_000_000_000
    value = request_payload(mirror, now_ns)
    target = [0.0, math.radians(1.0), 0.0, 0.0, 0.0, 0.0]
    value["target_relative_rad"] = target
    value["target_sha256"] = mirror.canonical_six_doubles_sha256(target)
    with pytest.raises(ValueError, match="support_joint_start_mismatch"):
        mirror.parse_collision_guard_request(
            value,
            now_ns=now_ns,
            hardware_state=hardware_payload(now_ns),
            expected_session_pose_sha256=mirror.canonical_six_doubles_sha256(
                [0.0] * 6
            ),
        )


@pytest.mark.parametrize(
    ("mutation", "message"),
    [
        (
            lambda state: state.update(
                joint_names=["joint2", "joint1", "joint3", "joint4", "joint5", "joint6"]
            ),
            "joint_names/order",
        ),
        (
            lambda state: state.update(position_rad=[0.0] * 5 + [math.nan]),
            "non-finite",
        ),
        (
            lambda state: state["per_motor"]["J6"].update(fresh=False),
            "J6 feedback is not fresh",
        ),
        (
            lambda state: state["per_motor"]["J2B"].update(
                reference_captured=False
            ),
            "J2B reference is unavailable",
        ),
        (
            lambda state: state["per_motor"]["J5"].update(
                communication_ok=False
            ),
            "J5 communication is not healthy",
        ),
        (
            lambda state: state["per_motor"]["J3"].update(merror=9),
            "J3 merror",
        ),
        (
            lambda state: state["controller_fault_by_motor"].update(J4=True),
            "J4 controller fault is active",
        ),
        (
            lambda state: state["controller_mode_by_motor"].update(J6="position"),
            "J6 controller mode is not HOLD",
        ),
        (
            lambda state: state["velocity_rad_s"].__setitem__(
                3, math.radians(0.251)
            ),
            "velocity exceeds 0.25 deg/s",
        ),
        (
            lambda state: state.update(j2_sync_fault=True),
            "J2 synchronization fault is active",
        ),
        (
            lambda state: state.update(initial_pose_sha256="not-a-sha256"),
            "initial_pose_sha256",
        ),
    ],
)
def test_any_axis_or_motor_readiness_failure_rejects_guard_request(
    mirror, mutation, message
):
    """A J1 move must still be rejected for a fault on any stationary axis."""

    now_ns = 9_000_000_000
    state = hardware_payload(now_ns)
    mutation(state)
    value = request_payload(
        mirror,
        now_ns,
        target=[math.radians(1.0), 0.0, 0.0, 0.0, 0.0, 0.0],
    )
    with pytest.raises(ValueError, match=message):
        mirror.parse_collision_guard_request(
            value,
            now_ns=now_ns,
            hardware_state=state,
            expected_session_pose_sha256=mirror.canonical_six_doubles_sha256(
                [0.0] * 6
            ),
        )


@pytest.mark.parametrize(
    "mutation",
    [
        lambda state: state["per_motor"]["J6"].update(fresh=1),
        lambda state: state["per_motor"].pop("J6"),
        lambda state: state["controller_fault_by_motor"].update(J6=0),
        lambda state: state["controller_fault_by_motor"].pop("J6"),
        lambda state: state["controller_mode_by_motor"].update(J6="HOLD"),
        lambda state: state["controller_mode_by_motor"].pop("J6"),
        lambda state: state.update(velocity_rad_s=[0.0] * 5 + [math.nan]),
        lambda state: state.update(j2_sync_fault=0),
    ],
)
def test_hardware_readiness_types_and_motor_sets_are_exact(mirror, mutation):
    now_ns = 9_000_000_000
    state = hardware_payload(now_ns)
    mutation(state)
    with pytest.raises(ValueError):
        mirror.hardware_state_for_collision_guard(state, now_ns)


def test_hardware_snapshot_copy_remains_revalidatable(mirror):
    now_ns = 9_000_000_000
    normalized = mirror.hardware_state_for_collision_guard(
        hardware_payload(now_ns), now_ns
    )
    assert mirror.hardware_state_for_collision_guard(normalized, now_ns) == normalized


def test_new_invalid_motor_state_immediately_revokes_cached_guard_state(mirror):
    now_ns = 9_000_000_000
    node = object.__new__(mirror.WholeArmMujocoMirror)
    node.active_hardware_state_instance_id = None
    node.active_hardware_state_receipt_ns = None
    node.hardware_sequences_by_instance = {}
    node.latest_collision_hardware_state = None
    node.collision_guard_rejections = 0

    healthy = hardware_payload(now_ns)
    with mock.patch.object(mirror.time, "monotonic_ns", return_value=now_ns):
        mirror.WholeArmMujocoMirror.on_collision_hardware_state(
            node, SimpleNamespace(data=json.dumps(healthy))
        )
    assert node.latest_collision_hardware_state is not None

    faulted = hardware_payload(now_ns)
    faulted["sequence"] += 1
    faulted["per_motor"]["J6"]["fresh"] = False
    with mock.patch.object(mirror.time, "monotonic_ns", return_value=now_ns):
        mirror.WholeArmMujocoMirror.on_collision_hardware_state(
            node, SimpleNamespace(data=json.dumps(faulted))
        )
    assert node.latest_collision_hardware_state is None
    assert node.collision_guard_rejections == 1


def test_session_pose_digest_is_echoed_and_engine_rejects_wrong_pose(mirror):
    request = parsed_request(mirror, [1, 0, 0, 0, 0, 0])
    result = engine(mirror, ThresholdGuard()).evaluate(request)
    assert result["session_pose_sha256"] == request.session_pose_sha256
    assert result["hardware_state_sha256"] == request.hardware_state_sha256
    assert result["hardware_velocity_rad_s"] == [0.0] * 6
    assert set(result["hardware_controller_mode_by_motor"].values()) == {"hold"}
    assert result["margin_policy"] == mirror.COLLISION_MARGIN_POLICY
    assert result["joint_space_tube_radius_deg"] == pytest.approx(2.25)

    wrong_pose_request = mirror.CollisionGuardRequest(
        **{**request.__dict__, "session_pose_sha256": "f" * 64}
    )
    rejected = engine(mirror, ThresholdGuard()).evaluate(wrong_pose_request)
    assert rejected["safe"] is False
    assert rejected["reason"] == "session_pose_mismatch"


def test_engine_rejects_tampered_hardware_mode_proof(mirror):
    request = parsed_request(mirror, [1, 0, 0, 0, 0, 0])
    modes = dict(request.hardware_controller_mode_by_motor)
    modes["J6"] = "position"
    tampered = mirror.CollisionGuardRequest(
        **{
            **request.__dict__,
            "hardware_controller_mode_by_motor": tuple(modes.items()),
        }
    )
    result = engine(mirror, ThresholdGuard()).evaluate(tampered)
    assert result["safe"] is False
    assert result["reason"] == "hardware_state_proof_invalid"


def test_fail_closed_result_preserves_session_pose_binding(mirror):
    now_ns = 9_000_000_000
    value = request_payload(mirror, now_ns)
    with mock.patch.object(mirror.time, "monotonic_ns", return_value=now_ns):
        result = mirror.fail_closed_collision_result(value, "validation_error")
    assert result["safe"] is False
    assert result["session_pose_sha256"] == value["session_pose_sha256"]
    assert_strict_fail_closed_result_matches(mirror, result, value, now_ns)


def test_parser_failure_immediately_publishes_matchable_fail_closed_result(mirror):
    now_ns = 9_000_000_000
    value = request_payload(mirror, now_ns)
    value["target_sha256"] = "0" * 64
    published = []
    node = object.__new__(mirror.WholeArmMujocoMirror)
    node.collision_guard_rejections = 0
    node.collision_requests_by_source = {}
    node.latest_collision_hardware_state = hardware_payload(now_ns)
    node.session_pose_sha256 = mirror.canonical_six_doubles_sha256([0.0] * 6)
    node.collision_guard_worker = SimpleNamespace(submit=mock.Mock())
    node.collision_result_publish_lock = threading.Lock()
    node.collision_results_enabled = True
    node.collision_result_publisher = SimpleNamespace(
        publish=lambda message: published.append(json.loads(message.data))
    )

    with mock.patch.object(mirror.time, "monotonic_ns", return_value=now_ns):
        mirror.WholeArmMujocoMirror.on_collision_guard_request(
            node, mirror.String(data=json.dumps(value))
        )
        assert len(published) == 1
        assert_strict_fail_closed_result_matches(
            mirror, published[0], value, now_ns
        )

    assert "does not bind" in published[0]["reason"]
    assert node.collision_guard_rejections == 1
    node.collision_guard_worker.submit.assert_not_called()


def test_worker_internal_error_immediately_publishes_matchable_rejection(mirror):
    request = parsed_request(mirror, [1, 0, 0, 0, 0, 0])
    published = []
    publication = threading.Event()

    def evaluator(_request, _cancelled):
        raise RuntimeError("synthetic evaluator failure")

    def publish(result):
        published.append(result)
        publication.set()

    worker = mirror.LatestCollisionGuardWorker(evaluator, publish)
    try:
        worker.submit(request)
        assert publication.wait(0.5)
        assert len(published) == 1
        now_ns = time.monotonic_ns()
        assert_strict_fail_closed_result_matches(
            mirror, published[0], request.echoed_fields(), now_ns
        )
        assert published[0]["reason"] == "internal_error:RuntimeError"
    finally:
        worker.close()


def test_mechanical_endpoint_is_not_uniformly_shrunk(mirror):
    checker = engine(mirror, ThresholdGuard())
    at_endpoint = checker.evaluate(parsed_request(
        mirror,
        [0, 170, 0, 0, 0, 0],
        start_deg=[0, 169, 0, 0, 0, 0],
    ))
    outside = checker.evaluate(parsed_request(
        mirror,
        [0, 170.01, 0, 0, 0, 0],
        start_deg=[0, 169, 0, 0, 0, 0],
    ))
    assert at_endpoint["safe"] is True
    assert outside["safe"] is False
    assert outside["reason"] == "position_limit"
    assert outside["recommended_relative_rad"] is None


def test_continuous_mjcf_j1_still_obeys_production_one_turn_limit(mirror):
    checker = engine(mirror, ThresholdGuard())
    at_endpoint = checker.evaluate(parsed_request(
        mirror,
        [180, 0, 0, 0, 0, 0],
        start_deg=[179, 0, 0, 0, 0, 0],
    ))
    outside = checker.evaluate(parsed_request(
        mirror,
        [180.01, 0, 0, 0, 0, 0],
        start_deg=[179, 0, 0, 0, 0, 0],
    ))
    assert at_endpoint["safe"] is True
    assert outside["safe"] is False
    assert outside["reason"] == "position_limit"


def test_first_self_collision_boundary_backs_off_full_tube_radius(mirror):
    checker = engine(mirror, ThresholdGuard(collision_at=10.0))
    result = checker.evaluate(parsed_request(mirror, [20, 0, 0, 0, 0, 0]))
    assert result["safe"] is False
    assert result["reason"] == "self_collision_margin"
    # The checked interval is the 20° command plus 2.25° at both ends.
    assert result["step_count"] == 98
    recommended_deg = math.degrees(result["recommended_relative_rad"][0])
    assert recommended_deg == pytest.approx(7.75, abs=0.02)
    assert result["first_unsafe_path_deg"] == pytest.approx(7.75, abs=0.02)
    assert result["contact_pairs"] == [
        ["Forearm_Collision_Proxy", "J1_Fixed_Collision_Proxy"]
    ]


def test_target_one_degree_before_collision_is_rejected_by_margin_probe(mirror):
    checker = engine(mirror, ThresholdGuard(collision_at=10.0))
    result = checker.evaluate(parsed_request(mirror, [9, 0, 0, 0, 0, 0]))
    assert result["safe"] is False
    assert result["reason"] == "self_collision_margin"
    assert math.degrees(result["recommended_relative_rad"][0]) == pytest.approx(
        7.75, abs=0.02
    )


def test_requested_target_is_an_explicit_sweep_sample(mirror):
    checker = engine(mirror, ExactTargetCollisionGuard())
    result = checker.evaluate(parsed_request(mirror, [9, 0, 0, 0, 0, 0]))
    assert result["safe"] is False
    assert result["reason"] == "self_collision_margin"


def test_two_degrees_is_not_enough_when_hold_tracking_is_included(mirror):
    checker = engine(mirror, ThresholdGuard(collision_at=10.0))
    result = checker.evaluate(parsed_request(mirror, [8, 0, 0, 0, 0, 0]))
    assert result["safe"] is False
    assert result["reason"] == "self_collision_margin"

    clear = checker.evaluate(parsed_request(mirror, [7.74, 0, 0, 0, 0, 0]))
    assert clear["safe"] is True
    assert clear["reason"] == "clear"


def test_ground_only_contact_has_distinct_unsafe_reason(mirror):
    checker = engine(
        mirror, ThresholdGuard(collision_at=5.0, collision_class="ground")
    )
    result = checker.evaluate(parsed_request(mirror, [10, 0, 0, 0, 0, 0]))
    assert result["safe"] is False
    assert result["reason"] == "ground_collision_margin"
    assert "self_collision" not in result["reason"]
    assert result["ground_contact_policy"] == (
        "UNSAFE_GROUND_COLLISION_NOT_SELF_COLLISION"
    )
    assert result["kinematic_guard_sha256"] == (
        mirror.PRODUCTION_KINEMATIC_GUARD_SHA256
    )
    assert result["collision_contract_sha256"] == (
        mirror.PRODUCTION_COLLISION_CONTRACT_SHA256
    )


def test_execute_checks_each_support_axis_hold_tracking_endpoint(mirror):
    checker = engine(mirror, SupportTrackingThresholdGuard())
    result = checker.evaluate(parsed_request(mirror, [1, 0, 0, 0, 0, 0]))
    assert result["safe"] is False
    assert result["reason"] == "self_collision_margin"
    assert result["first_unsafe_path_deg"] == 0.0
    assert result["margin_probe_offsets_deg"][1] == pytest.approx(0.25)
    assert result["contact_pairs"] == [["J2-tracking-edge-obstacle", "arm"]]


def test_final_pose_preview_checks_perpendicular_two_degree_clearance(mirror):
    checker = engine(mirror, PerpendicularThresholdGuard())
    request = replace(
        parsed_request(mirror, [1, 0, 0, 0, 0, 0]),
        kind="pose_preview",
        moving_joint_mask=(True, True, False, False, False, False),
    )
    result = checker.evaluate(request)
    assert result["safe"] is False
    assert result["reason"] == "self_collision_margin"
    assert result["margin_probe_offsets_deg"][1] == pytest.approx(1.0)
    assert result["contact_pairs"] == [["J2-offset-obstacle", "arm"]]


def test_final_pose_preview_checks_multi_axis_clearance_corner(mirror):
    checker = engine(mirror, TwoAxisCornerGuard())
    request = replace(
        parsed_request(mirror, [1, 0, 0, 0, 0, 0]),
        kind="pose_preview",
        moving_joint_mask=(True, True, False, False, False, False),
    )
    result = checker.evaluate(request)
    assert result["safe"] is False
    assert result["reason"] == "self_collision_margin"
    assert result["margin_probe_offsets_deg"][2] == pytest.approx(2.25)
    assert result["margin_probe_offsets_deg"][3] == pytest.approx(2.25)
    assert sum(
        abs(value) > 0.0 for value in result["margin_probe_offsets_deg"]
    ) >= 2


def test_support_tracking_error_does_not_create_a_false_second_moving_joint(mirror):
    checker = engine(mirror, ThresholdGuard())
    result = checker.evaluate(parsed_request(
        mirror,
        [1, 0, 0, 0, 0, 0],
        start_deg=[0, -0.1, 0, 0, 0, 0],
        moving_index=0,
    ))
    assert result["safe"] is True
    assert result["moving_joint"] == "J1"
    assert result["moving_joint_count"] == 1
    assert result["recommended_relative_rad"] == pytest.approx(
        [math.radians(1), 0, 0, 0, 0, 0]
    )


def test_engine_tube_grid_remains_promptly_cancellable(mirror):
    checker = engine(mirror, ThresholdGuard())
    polls = 0

    def cancelled():
        nonlocal polls
        polls += 1
        return polls >= 25

    with pytest.raises(mirror.GuardEvaluationCancelled):
        checker.evaluate(
            parsed_request(mirror, [20, 0, 0, 0, 0, 0]),
            cancelled,
        )
    assert polls == 25


def test_latest_worker_cancels_obsolete_request(mirror):
    first_started = threading.Event()
    published = []

    def evaluator(request, cancelled):
        if request.request_sequence == 1:
            first_started.set()
            deadline = time.monotonic() + 1.0
            while not cancelled() and time.monotonic() < deadline:
                time.sleep(0.001)
            if cancelled():
                raise mirror.GuardEvaluationCancelled()
        return {"request_sequence": request.request_sequence}

    worker = mirror.LatestCollisionGuardWorker(evaluator, published.append)
    try:
        first = parsed_request(mirror, [1, 0, 0, 0, 0, 0])
        second = mirror.CollisionGuardRequest(
            **{**first.__dict__, "request_sequence": 2}
        )
        worker.submit(first)
        assert first_started.wait(0.5)
        worker.submit(second)
        deadline = time.monotonic() + 1.0
        while not published and time.monotonic() < deadline:
            time.sleep(0.005)
        assert published == [{"request_sequence": 2}]
    finally:
        worker.close()


def test_worker_close_cancels_active_evaluation_and_joins_deterministically(mirror):
    started = threading.Event()
    cancellation_seen = threading.Event()
    published = []

    def evaluator(_request, cancelled):
        started.set()
        while not cancelled():
            time.sleep(0.001)
        cancellation_seen.set()
        raise mirror.GuardEvaluationCancelled()

    worker = mirror.LatestCollisionGuardWorker(evaluator, published.append)
    worker.submit(parsed_request(mirror, [1, 0, 0, 0, 0, 0]))
    assert started.wait(0.5)

    closer = threading.Thread(target=worker.close)
    closer.start()
    closer.join(1.0)
    assert not closer.is_alive()
    assert cancellation_seen.is_set()
    assert not worker.thread.is_alive()
    assert published == []
    worker.close()  # idempotent after a complete join
    with pytest.raises(RuntimeError, match="stopping"):
        worker.submit(parsed_request(mirror, [2, 0, 0, 0, 0, 0]))


def test_destroy_disables_async_results_before_cancelling_worker(mirror):
    node = object.__new__(mirror.WholeArmMujocoMirror)
    node.collision_result_publish_lock = threading.Lock()
    node.collision_results_enabled = True
    published = []
    node.collision_result_publisher = SimpleNamespace(
        publish=lambda message: published.append(message.data)
    )
    node.capture_stream = io.StringIO()
    node.write_validation = mock.Mock()

    started = threading.Event()
    cancellation_seen = threading.Event()

    def evaluator(_request, cancelled):
        started.set()
        while not cancelled():
            time.sleep(0.001)
        cancellation_seen.set()
        # Even an evaluator that returns normally after cancellation must not
        # reach the publisher during node teardown.
        return {"safe": False, "reason": "cancelled"}

    node.collision_guard_worker = mirror.LatestCollisionGuardWorker(
        evaluator,
        lambda result: mirror.WholeArmMujocoMirror.publish_collision_guard_result(
            node, result
        ),
    )
    node.collision_guard_worker.submit(
        parsed_request(mirror, [1, 0, 0, 0, 0, 0])
    )
    assert started.wait(0.5)

    assert mirror.WholeArmMujocoMirror.destroy_node(node) is True
    assert cancellation_seen.is_set()
    assert not node.collision_guard_worker.thread.is_alive()
    assert published == []
    assert node.capture_stream.closed
    assert node.write_validation.call_count == 1
    assert node.base_destroyed is True

    mirror.WholeArmMujocoMirror.publish_collision_guard_result(
        node, {"safe": True}
    )
    assert published == []


def test_destroy_runs_validation_capture_and_base_cleanup_if_worker_close_fails(
    mirror,
):
    node = object.__new__(mirror.WholeArmMujocoMirror)
    node.collision_result_publish_lock = threading.Lock()
    node.collision_results_enabled = True
    node.capture_stream = io.StringIO()
    node.write_validation = mock.Mock()

    class FailingWorker:
        def close(self):
            raise RuntimeError("synthetic close failure")

    node.collision_guard_worker = FailingWorker()
    with pytest.raises(RuntimeError, match="synthetic close failure"):
        mirror.WholeArmMujocoMirror.destroy_node(node)
    assert node.collision_results_enabled is False
    assert node.write_validation.call_count == 1
    assert node.capture_stream.closed
    assert node.base_destroyed is True


def test_frozen_hash_constants_match_repository_artifacts(mirror):
    root = Path(__file__).resolve().parents[4] / "mujoco_v15_14"
    assert mirror.sha256_file(root / "go_m8010_arm_v15_14_kinematic.xml") == (
        mirror.PRODUCTION_MODEL_SHA256
    )
    assert mirror.sha256_file(root / "kinematic_guard.py") == (
        mirror.PRODUCTION_KINEMATIC_GUARD_SHA256
    )
    assert mirror.sha256_file(root / "collision_pair_contract_v15_14.json") == (
        mirror.PRODUCTION_COLLISION_CONTRACT_SHA256
    )

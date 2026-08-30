"""Offline integration tests for the complete collision-proof authority chain.

The tests deliberately load the production Python sources with tiny ROS
stubs.  They never construct a ROS node, open a GUI, create a UDP socket, or
touch hardware.
"""

from __future__ import annotations

import ast
from copy import deepcopy
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import ModuleType, SimpleNamespace
from typing import Optional
from unittest import mock

import numpy as np
import pytest


PROJECT = Path(__file__).resolve().parents[1]
ROS_SRC = PROJECT.parent
GUI_SOURCE = PROJECT / "go_m8010_arm_gui" / "main_window.py"
ROUTER_SOURCE = PROJECT / "go_m8010_arm_gui" / "command_router.py"
MIRROR_SOURCE = (
    ROS_SRC
    / "go_m8010_arm_hardware"
    / "go_m8010_arm_hardware"
    / "mujoco_mirror_node.py"
)


class FrozenClock:
    def __init__(self, now_ns: int = 100_000_000_000) -> None:
        self.now_ns = now_ns

    def monotonic_ns(self) -> int:
        return self.now_ns

    def advance(self, nanoseconds: int = 1_000_000) -> None:
        self.now_ns += nanoseconds


def _ros_stubs(*, package_name: Optional[str] = None) -> dict[str, ModuleType]:
    rclpy = ModuleType("rclpy")
    node_module = ModuleType("rclpy.node")
    node_module.Node = type("Node", (), {})
    executors = ModuleType("rclpy.executors")
    executors.ExternalShutdownException = type(
        "ExternalShutdownException", (Exception,), {}
    )
    std_msgs = ModuleType("std_msgs")
    std_msgs_msg = ModuleType("std_msgs.msg")

    class String:
        def __init__(self, *, data: str = "") -> None:
            self.data = data

    std_msgs_msg.String = String
    stubs = {
        "rclpy": rclpy,
        "rclpy.node": node_module,
        "rclpy.executors": executors,
        "std_msgs": std_msgs,
        "std_msgs.msg": std_msgs_msg,
    }
    if package_name is not None:
        package = ModuleType(package_name)
        package.__path__ = []
        state_model = ModuleType(f"{package_name}.state_model")
        state_model.JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
        state_model.MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
        signals = ModuleType("rclpy.signals")
        signals.SignalHandlerOptions = SimpleNamespace(NO=object())
        sensor_msgs = ModuleType("sensor_msgs")
        sensor_msgs_msg = ModuleType("sensor_msgs.msg")
        sensor_msgs_msg.JointState = type("JointState", (), {})
        std_msgs_msg.Float64MultiArray = type("Float64MultiArray", (), {})
        stubs.update({
            package_name: package,
            f"{package_name}.state_model": state_model,
            "rclpy.signals": signals,
            "sensor_msgs": sensor_msgs,
            "sensor_msgs.msg": sensor_msgs_msg,
        })
    return stubs


def _load_router():
    module_name = "collision_pipeline_command_router"
    with mock.patch.dict(sys.modules, _ros_stubs()):
        spec = importlib.util.spec_from_file_location(module_name, ROUTER_SOURCE)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
    return module


def _load_mirror():
    package_name = "collision_pipeline_mirror_package"
    module_name = f"{package_name}.mujoco_mirror_node"
    with mock.patch.dict(sys.modules, _ros_stubs(package_name=package_name)):
        spec = importlib.util.spec_from_file_location(module_name, MIRROR_SOURCE)
        assert spec is not None and spec.loader is not None
        module = importlib.util.module_from_spec(spec)
        sys.modules[module_name] = module
        spec.loader.exec_module(module)
    return module


def _gui_function(name: str, namespace: dict):
    tree = ast.parse(GUI_SOURCE.read_text(encoding="utf-8"))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )
    compiled = ast.Module(body=[function], type_ignores=[])
    globals_by_name = dict(namespace)
    exec(
        compile(ast.fix_missing_locations(compiled), str(GUI_SOURCE), "exec"),
        globals_by_name,
    )
    return globals_by_name[name]


def _gui_method(class_name: str, name: str, namespace: dict):
    tree = ast.parse(GUI_SOURCE.read_text(encoding="utf-8"))
    selected_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == class_name
    )
    method = next(
        node
        for node in selected_class.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )
    compiled = ast.Module(body=[method], type_ignores=[])
    globals_by_name = dict(namespace)
    exec(
        compile(ast.fix_missing_locations(compiled), str(GUI_SOURCE), "exec"),
        globals_by_name,
    )
    return globals_by_name[name]


@pytest.fixture()
def pipeline():
    clock = FrozenClock()
    router = _load_router()
    mirror = _load_mirror()
    router.time = clock
    mirror.time = clock

    gui_hash = _gui_function(
        "collision_target_sha256",
        {"hashlib": __import__("hashlib"), "math": math, "struct": __import__("struct")},
    )
    gui_source_valid = _gui_function(
        "source_instance_id_valid",
        {"SOURCE_INSTANCE_ID_HEX_LENGTH": 32},
    )
    gui_limits = _gui_function(
        "moving_targets_within_model_limits",
        {"math": math, "RAD": math.pi / 180.0},
    )

    class ArmMode:
        SIM_TO_REAL = object()

    request_method = _gui_method(
        "MainWindow",
        "_new_collision_request",
        {
            "Optional": Optional,
            "ArmMode": ArmMode,
            "time": clock,
            "source_instance_id_valid": gui_source_valid,
            "collision_motion_state_ready": lambda *_args, **_kwargs: True,
            "moving_targets_within_model_limits": gui_limits,
            "collision_target_sha256": gui_hash,
            "COLLISION_MARGIN_DEG": 2.0,
        },
    )
    matcher = _gui_function(
        "collision_guard_result_matches",
        {
            "math": math,
            "time": clock,
            "PRODUCTION_MODEL_SHA256": router.PRODUCTION_MODEL_SHA256,
            "PRODUCTION_COLLISION_CONTRACT_SHA256": (
                router.PRODUCTION_COLLISION_CONTRACT_SHA256
            ),
            "PRODUCTION_KINEMATIC_GUARD_SHA256": (
                router.PRODUCTION_KINEMATIC_GUARD_SHA256
            ),
            "COLLISION_GROUND_POLICY": router.GROUND_CONTACT_POLICY,
            "COLLISION_GUARD_MAX_STEP_DEG": router.COLLISION_GUARD_MAX_STEP_DEG,
            "COLLISION_BOUNDARY_TOLERANCE_DEG": (
                router.COLLISION_GUARD_BOUNDARY_TOLERANCE_DEG
            ),
            "COLLISION_MARGIN_POLICY": router.COLLISION_MARGIN_POLICY,
            "COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG": (
                router.COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG
            ),
            "COLLISION_HOLD_TRACKING_TOLERANCE_DEG": (
                router.COLLISION_HOLD_TRACKING_TOLERANCE_DEG
            ),
            "COLLISION_TUBE_PROBE_JOINT_NAMES": (
                router.COLLISION_TUBE_PROBE_JOINT_NAMES
            ),
            "COLLISION_TUBE_GRID_OFFSETS": router.COLLISION_TUBE_GRID_OFFSETS,
            "COLLISION_TUBE_GRID_MAX_POSE_COUNT": (
                router.COLLISION_TUBE_GRID_MAX_POSE_COUNT
            ),
            "COLLISION_TUBE_AXIS_PROBE_STEP_DEG": (
                router.COLLISION_TUBE_AXIS_PROBE_STEP_DEG
            ),
            "COLLISION_TUBE_CROSS_SECTION_MAX_POSE_COUNT": (
                router.COLLISION_TUBE_CROSS_SECTION_MAX_POSE_COUNT
            ),
            "PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG": (
                router.PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG
            ),
            "COLLISION_HOLD_VELOCITY_TOLERANCE_RAD_S": (
                router.COLLISION_HARDWARE_MAX_ABS_VELOCITY_RAD_S
            ),
            "COLLISION_START_MATCH_TOLERANCE_RAD": (
                router.COLLISION_NONMOVING_TARGET_TOLERANCE_RAD
            ),
            "MOTOR_NAMES": router.MOTOR_NAMES,
            "canonical_collision_hardware_state_sha256": (
                mirror.canonical_collision_hardware_state_sha256
            ),
            "COLLISION_GUARD_TIMEOUT_S": (
                router.COLLISION_GUARD_PROOF_MAX_AGE_NS / 1.0e9
            ),
        },
    )

    return SimpleNamespace(
        clock=clock,
        router=router,
        mirror=mirror,
        ArmMode=ArmMode,
        gui_hash=gui_hash,
        request_method=request_method,
        matcher=matcher,
    )


def _hardware_state(pipeline, position):
    now_ns = pipeline.clock.monotonic_ns()
    motors = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
    return {
        "schema": "go-m8010-hardware-state/1.1",
        "joint_names": [f"joint{index}" for index in range(1, 7)],
        "session_id": "persistent:vertical-test",
        "state_instance_id": "2" * 32,
        "sequence": 17,
        "source_monotonic_ns": now_ns - 1,
        "position_rad": list(position),
        "velocity_rad_s": [0.0] * 6,
        "per_motor": {
            motor: {
                "fresh": True,
                "reference_captured": True,
                "communication_ok": True,
                "merror": 0,
            }
            for motor in motors
        },
        "controller_fault_by_motor": {motor: False for motor in motors},
        "controller_mode_by_motor": {motor: "hold" for motor in motors},
        "j2_sync_fault": False,
        "initial_pose_sha256": "a" * 64,
    }


def _gui_request(pipeline, target, *, pending_mask=None):
    if pending_mask is None:
        pending_mask = [True, False, False, False, False, False]
    hardware = _hardware_state(pipeline, [0.0] * 6)

    class Node:
        command_source_instance_id = "1" * 32
        latest_hardware = hardware

        def __init__(self):
            self.published = []

        def control_streams_fresh(self):
            return True

        def publish_collision_request(self, value):
            self.published.append(deepcopy(value))

    window = SimpleNamespace(
        direction=pipeline.ArmMode.SIM_TO_REAL,
        session_id=hardware["session_id"],
        state_instance_id=hardware["state_instance_id"],
        command_stream_suspended=False,
        pending_target_joint_mask=list(pending_mask),
        targets=list(target),
        command_targets=[0.0] * 6,
        requested_active_joint_mask=[True] * 6,
        hardware_mode="hold",
        edit_limits=[(-180.0, 180.0)] * 6,
        collision_request_sequence=0,
        collision_requests={},
        session_pose_sha256=pipeline.mirror.canonical_six_doubles_sha256(
            [0.0] * 6
        ),
        node=Node(),
    )
    request = pipeline.request_method(window, "execute")
    return window, request, hardware


class AlwaysSafeGuard:
    def check_pose_deg(self, _pose):
        return {"safe": True, "reason": "clear", "contacts": []}


def _mirror_proof(pipeline, target, *, pending_mask=None):
    window, request, hardware = _gui_request(
        pipeline, target, pending_mask=pending_mask
    )
    assert request is not None
    assert window.node.published == [request]
    parsed = pipeline.mirror.parse_collision_guard_request(
        request,
        now_ns=pipeline.clock.monotonic_ns(),
        hardware_state=hardware,
        expected_session_pose_sha256=request["session_pose_sha256"],
    )
    engine = pipeline.mirror.CollisionGuardEngine(
        AlwaysSafeGuard(),
        [0.0] * 6,
        absolute_joint_limits_deg=(
            pipeline.mirror.PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG
        ),
        model_sha256=pipeline.router.PRODUCTION_MODEL_SHA256,
        guard_sha256=pipeline.router.PRODUCTION_KINEMATIC_GUARD_SHA256,
        contract_sha256=pipeline.router.PRODUCTION_COLLISION_CONTRACT_SHA256,
    )
    proof = engine.evaluate(parsed)
    assert proof["safe"] is True
    assert pipeline.matcher(proof, request)
    return request, proof


def _position_command(
    pipeline,
    proof,
    *,
    target=None,
    moving_mask=None,
    sequence=1,
    activation_epoch=1,
):
    if target is None:
        target = proof["target_relative_rad"]
    if moving_mask is None:
        moving_mask = [True, False, False, False, False, False]
    return {
        "schema": "go-m8010-gui-command/1.2",
        "sequence": sequence,
        "source_instance_id": proof["source_instance_id"],
        "source_monotonic_ns": pipeline.clock.monotonic_ns(),
        "mode": "position",
        "targets_rad": list(target),
        "active_joint_mask": [True] * 6,
        "moving_joint_mask": list(moving_mask),
        "activation_epoch": activation_epoch,
        "maximum_velocity_rad_s": math.radians(5.0),
        "maximum_acceleration_rad_s2": math.radians(20.0),
        "collision_guard_proof": deepcopy(proof),
    }


class FakeSocket:
    def __init__(self):
        self.sent = []

    def sendto(self, payload, destination):
        self.sent.append((json.loads(payload), destination))


def _router_harness(pipeline, *, last_command=None):
    router = pipeline.router
    harness = SimpleNamespace(
        socket=FakeSocket(),
        destinations=[
            ("J1", ("127.0.0.1", 1)),
            ("J2", ("127.0.0.1", 2)),
            ("J345", ("127.0.0.1", 3)),
            ("J6", ("127.0.0.1", 4)),
        ],
        last_command=last_command,
        replay_guard=router.CommandReplayGuard(),
        collision_guard_gate=router.CollisionGuardProofGate(),
        rejection_tracker=router.RejectionTracker(),
        rejected=0,
        _log_rejection_reports=lambda _reports: None,
    )
    return harness


def _observe(pipeline, harness, proof):
    message = SimpleNamespace(data=json.dumps(proof))
    pipeline.router.CommandRouter.on_collision_guard_result(harness, message)


def _route(pipeline, harness, command):
    message = SimpleNamespace(data=json.dumps(command))
    pipeline.router.CommandRouter.on_command(harness, message)


def _assert_rejected_without_brake(harness, *, sent_before=0, last_before=None):
    assert len(harness.socket.sent) == sent_before
    assert harness.last_command is last_before
    assert harness.rejected >= 1
    assert all(payload["mode"] != "brake" for payload, _ in harness.socket.sent)


def test_single_moving_joint_passes_gui_mirror_matcher_and_router(pipeline):
    target = [math.radians(1.0), 0.0, 0.0, 0.0, 0.0, 0.0]
    request, proof = _mirror_proof(pipeline, target)
    assert proof["target_sha256"] == request["target_sha256"]

    command = _position_command(pipeline, proof)
    normalized, _ = pipeline.router.validate_command(
        json.dumps(command), now_ns=pipeline.clock.monotonic_ns()
    )
    assert normalized["moving_joint_mask"] == [True, False, False, False, False, False]

    harness = _router_harness(pipeline)
    _observe(pipeline, harness, proof)
    _route(pipeline, harness, command)

    assert harness.rejected == 0
    assert harness.last_command is not None
    assert [payload["mode"] for payload, _ in harness.socket.sent] == [
        "position", "hold", "hold", "hold"
    ]


def test_multi_moving_is_rejected_at_gui_and_router_without_brake(pipeline):
    target = [math.radians(1.0), math.radians(1.0), 0.0, 0.0, 0.0, 0.0]
    window, request, _hardware = _gui_request(
        pipeline,
        target,
        pending_mask=[True, True, False, False, False, False],
    )
    assert request is None
    assert window.node.published == []

    _request, proof = _mirror_proof(
        pipeline,
        [math.radians(1.0), 0.0, 0.0, 0.0, 0.0, 0.0],
    )
    command = _position_command(
        pipeline,
        proof,
        moving_mask=[True, True, False, False, False, False],
    )
    with pytest.raises(ValueError, match="只能选择一个移动关节"):
        pipeline.router.validate_command(
            json.dumps(command), now_ns=pipeline.clock.monotonic_ns()
        )

    sentinel = {"mode": "hold", "immutable": True}
    harness = _router_harness(pipeline, last_command=sentinel)
    _observe(pipeline, harness, proof)
    _route(pipeline, harness, command)
    _assert_rejected_without_brake(harness, last_before=sentinel)


def test_target_rewrite_and_nonmoving_start_deviation_are_inert(pipeline):
    _request, proof = _mirror_proof(
        pipeline, [math.radians(1.0), 0.0, 0.0, 0.0, 0.0, 0.0]
    )

    rewritten = _position_command(
        pipeline,
        proof,
        target=[math.radians(2.0), 0.0, 0.0, 0.0, 0.0, 0.0],
    )
    # The command envelope itself is valid; the independently received proof
    # must be what rejects the post-check target rewrite.
    normalized, _ = pipeline.router.validate_command(
        json.dumps(rewritten), now_ns=pipeline.clock.monotonic_ns()
    )
    gate = pipeline.router.CollisionGuardProofGate()
    assert gate.observe_result(proof, now_ns=pipeline.clock.monotonic_ns())
    with pytest.raises(ValueError, match="与运动目标不匹配"):
        gate.authorize(normalized, now_ns=pipeline.clock.monotonic_ns())

    sentinel = {"mode": "hold", "immutable": True}
    harness = _router_harness(pipeline, last_command=sentinel)
    _observe(pipeline, harness, proof)
    _route(pipeline, harness, rewritten)
    _assert_rejected_without_brake(harness, last_before=sentinel)

    pipeline.clock.advance()
    _window, invalid_request, hardware = _gui_request(
        pipeline,
        [math.radians(1.0), math.radians(0.5), 0.0, 0.0, 0.0, 0.0],
    )
    assert invalid_request is not None
    with pytest.raises(ValueError, match="support_joint_start_mismatch"):
        pipeline.mirror.parse_collision_guard_request(
            invalid_request,
            now_ns=pipeline.clock.monotonic_ns(),
            hardware_state=hardware,
            expected_session_pose_sha256=(
                invalid_request["session_pose_sha256"]
            ),
        )


def test_mask_rewrite_after_binding_is_rejected_without_new_output(pipeline):
    _request, proof = _mirror_proof(
        pipeline, [math.radians(0.1), 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    harness = _router_harness(pipeline)
    first = _position_command(pipeline, proof, sequence=1, activation_epoch=9)
    _observe(pipeline, harness, proof)
    _route(pipeline, harness, first)
    assert len(harness.socket.sent) == 4
    accepted = harness.last_command

    pipeline.clock.advance()
    changed_mask = _position_command(
        pipeline,
        proof,
        moving_mask=[False, True, False, False, False, False],
        sequence=2,
        activation_epoch=9,
    )
    _route(pipeline, harness, changed_mask)
    _assert_rejected_without_brake(
        harness, sent_before=4, last_before=accepted
    )
    assert any(
        "运动合同不匹配" in reason
        for reason in harness.rejection_tracker.by_reason
    )


def test_expired_proof_is_rejected_without_brake(pipeline):
    _request, proof = _mirror_proof(
        pipeline, [math.radians(1.0), 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    sentinel = {"mode": "hold", "immutable": True}
    harness = _router_harness(pipeline, last_command=sentinel)
    _observe(pipeline, harness, proof)
    pipeline.clock.advance(pipeline.router.COLLISION_GUARD_PROOF_MAX_AGE_NS + 1)
    command = _position_command(pipeline, proof)
    _route(pipeline, harness, command)
    _assert_rejected_without_brake(harness, last_before=sentinel)
    assert any("已过期" in reason for reason in harness.rejection_tracker.by_reason)


def test_cross_epoch_and_command_or_result_replay_are_inert(pipeline):
    _request, proof = _mirror_proof(
        pipeline, [math.radians(1.0), 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    harness = _router_harness(pipeline)
    first = _position_command(pipeline, proof, sequence=1, activation_epoch=3)
    _observe(pipeline, harness, proof)
    assert harness.collision_guard_gate.cached_proof_count == 1
    _observe(pipeline, harness, proof)
    assert harness.collision_guard_gate.cached_proof_count == 1
    _route(pipeline, harness, first)
    assert len(harness.socket.sent) == 4
    accepted = harness.last_command

    # Exact command replay is rejected before it can emit any domain payload.
    _route(pipeline, harness, first)
    _assert_rejected_without_brake(
        harness, sent_before=4, last_before=accepted
    )

    pipeline.clock.advance()
    next_epoch = _position_command(
        pipeline, proof, sequence=2, activation_epoch=4
    )
    _route(pipeline, harness, next_epoch)
    _assert_rejected_without_brake(
        harness, sent_before=4, last_before=accepted
    )
    assert any(
        "不得跨激活纪元重用" in reason
        for reason in harness.rejection_tracker.by_reason
    )

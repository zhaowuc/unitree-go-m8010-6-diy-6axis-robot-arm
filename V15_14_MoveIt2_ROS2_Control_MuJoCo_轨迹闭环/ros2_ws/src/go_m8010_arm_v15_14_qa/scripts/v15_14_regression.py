#!/usr/bin/env python3
from __future__ import annotations

import argparse
import array
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import time
from typing import Any

import mujoco
import numpy as np
import rclpy
from action_msgs.msg import GoalStatus
from control_msgs.msg import JointTrajectoryControllerState
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    OrientationConstraint,
    PlanningSceneComponents,
    PositionConstraint,
)
from moveit_msgs.srv import GetPlanningScene, GetPositionIK, GetStateValidity
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.srv import GetParameters
from rclpy.action import ActionClient
from rclpy.action.graph import get_action_server_names_and_types_by_node
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.serialization import serialize_message
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener


JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")
POSITION_BOUNDS = {
    "J2": (-2.96705972839036, 2.96705972839036),
    "J3": (-2.96705972839036, 2.96705972839036),
    "J4": (-2.02458193231342, 2.77507351067098),
    "J5": (-1.232202451908, 2.63893782901543),
    "J6": (-3.14159265358979, 3.14159265358979),
}
MAX_VELOCITY = 0.5
MAX_ACCELERATION = 1.0
MAX_COLLISION_SAMPLE_STEP = math.radians(0.25)
# TopicBasedSystem reads the authoritative raw state on one controller cycle
# and JointStateBroadcaster publishes that value on the following update.  At
# the pinned 50 Hz rate the exact-value latency is two scheduler periods in the
# current Humble pipeline (~40-45 ms); 60 ms is a strict three-period ceiling.
MAX_STATE_PAIR_TIME_SKEW_S = 0.06
THRESHOLDS = {
    "path_sample_max_joint_step_rad": MAX_COLLISION_SAMPLE_STEP,
    "final_joint_max_abs_error_rad": 1.0e-6,
    "final_tcp_position_error_m": 1.0e-4,
    "final_tcp_orientation_error_rad": 1.0e-3,
    "trajectory_tracking_max_abs_error_rad": 0.05,
    "trajectory_velocity_tracking_max_abs_error_rad_s": 0.10,
    "max_controller_state_gap_s": 0.10,
    "max_raw_state_gap_s": 0.05,
    "max_public_state_gap_s": 0.05,
    # JSB re-stamps the latest TopicBasedSystem sample.  The trace audit pairs
    # it with the identical raw value inside a one-to-one ±60 ms pipeline
    # correlation window.  This is not a physical synchronization tolerance:
    # joint_state_broadcaster re-stamps the state it republishes.
    "public_raw_joint_sync_rad": 1.0e-6,
    "final_public_raw_joint_sync_rad": 1.0e-6,
    "public_raw_joint_velocity_sync_rad_s": 0.05,
    "max_state_pair_time_skew_s": MAX_STATE_PAIR_TIME_SKEW_S,
    "tf_mujoco_tcp_position_m": 1.0e-8,
    "tf_mujoco_tcp_orientation_rad": 1.0e-7,
    "negative_no_execution_joint_excursion_rad": 1.0e-9,
}

REQUIRED_TARGET_IDS = {
    "central",
    "high",
    "left",
    "front",
    "low",
    "right",
    "back",
    "random_valid_seed_1514",
    "near_joint_limit_clear",
    "near_self_collision_clear",
    "outside_joint_limit_reject",
    "self_collision_reject",
}
ACCEPTED_TARGET_MANIFEST_SHA256 = (
    "f2b230e2cb61c251c81228f5f3b89ced0163b05cf82f0b93db8fd0bb85572d57"
)
ACCEPTED_MODEL_SHA256 = (
    "ce6ec828d32f445ff25fb4b3ba3167f7f0bec61cdfca5962ee4072df4b022844"
)
ACCEPTED_COLLISION_CONTRACT_SHA256 = (
    "6d802909e44f238816f007ef33b57e1b57099c5522a473cd2dcc2705397af9c9"
)
ACCEPTED_GUARD_SHA256 = (
    "648b9de953a84be46cd5f95e24daf9f683f3025e73048dc068ef7bd8e5670c6a"
)
ACCEPTED_RUNTIME_MESH_MANIFEST_SHA256 = (
    "1174abcfef3b87ee77d4ce50af5afc1aa62d45759f1560ace361997e8c60b19d"
)
ACCEPTED_RUNTIME_ASSET_SET_SHA256 = (
    "fe517900413488414f9a606d7fd8f05ea8a7f2be08043672e75c57ae669f0425"
)
ACCEPTED_SOURCE_MESH_MANIFEST_SHA256 = (
    "22cfb9a194495b442731405a023cec8f83fb03d17b2e814cf13febc924d1ce3a"
)


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def semantic_message_value(value):
    """Convert a ROS message recursively to its complete semantic field tree."""
    if hasattr(value, "get_fields_and_field_types"):
        return {
            field: semantic_message_value(getattr(value, field))
            for field in value.get_fields_and_field_types()
        }
    if isinstance(value, dict):
        return {
            str(key): semantic_message_value(item)
            for key, item in value.items()
        }
    if isinstance(value, (list, tuple, array.array)):
        return [semantic_message_value(item) for item in value]
    if isinstance(value, (bytes, bytearray, memoryview)):
        return list(value)
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    raise TypeError(f"unsupported ROS semantic field type: {type(value)!r}")


def semantic_message_sha256(message) -> str:
    """Hash all ROS fields without non-semantic CDR alignment padding."""
    encoded = json.dumps(
        semantic_message_value(message),
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
        allow_nan=False,
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def duration_seconds(duration) -> float:
    return float(duration.sec) + float(duration.nanosec) * 1.0e-9


def quaternion_error(first: np.ndarray, second: np.ndarray) -> float:
    a = np.asarray(first, dtype=float)
    b = np.asarray(second, dtype=float)
    a /= np.linalg.norm(a)
    b /= np.linalg.norm(b)
    return float((Rotation.from_quat(a).inv() * Rotation.from_quat(b)).magnitude())


def wrapped_joint_error(commanded: np.ndarray, actual: np.ndarray) -> np.ndarray:
    error = np.asarray(actual) - np.asarray(commanded)
    error[0] = math.atan2(math.sin(error[0]), math.cos(error[0]))
    return error


def state_values(message: JointState | None) -> np.ndarray | None:
    if message is None or set(message.name) != set(JOINTS):
        return None
    lookup = dict(zip(message.name, message.position))
    return np.array([lookup[name] for name in JOINTS], dtype=float)


def state_velocity_values(message: JointState | None) -> np.ndarray | None:
    if (
        message is None
        or set(message.name) != set(JOINTS)
        or len(message.velocity) != len(message.name)
    ):
        return None
    lookup = dict(zip(message.name, message.velocity))
    values = np.array([lookup[name] for name in JOINTS], dtype=float)
    return values if np.all(np.isfinite(values)) else None


class Regression(Node):
    def __init__(self, model_path: Path, output_dir: Path) -> None:
        super().__init__("go_m8010_arm_v15_14_regression")
        self.model_path = model_path
        self.model_sha256 = sha256(model_path)
        if self.model_sha256 != ACCEPTED_MODEL_SHA256:
            raise RuntimeError(
                f"unaccepted V15.14 MJCF hash: {self.model_sha256}"
            )
        self.collision_contract_path = (
            model_path.parent / "collision_pair_contract_v15_14.json"
        )
        if not self.collision_contract_path.is_file():
            raise FileNotFoundError(self.collision_contract_path)
        self.collision_contract_sha256 = sha256(self.collision_contract_path)
        if (
            self.collision_contract_sha256
            != ACCEPTED_COLLISION_CONTRACT_SHA256
        ):
            raise RuntimeError(
                "unaccepted V15.14 collision contract hash: "
                f"{self.collision_contract_sha256}"
            )
        self.collision_contract = json.loads(
            self.collision_contract_path.read_text(encoding="utf-8")
        )
        self.guard_path = model_path.parent / "kinematic_guard.py"
        self.guard_sha256 = sha256(self.guard_path)
        if self.guard_sha256 != ACCEPTED_GUARD_SHA256:
            raise RuntimeError(
                f"unaccepted V15.14 kinematic guard hash: {self.guard_sha256}"
            )
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.trace_dir = self.output_dir / "traces"
        self.trace_dir.mkdir(parents=True, exist_ok=True)

        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        # V15.13's accepted pair contract intentionally includes two directly
        # connected-link collision pairs. MuJoCo's default parent filter would
        # remove them before KinematicGuard can apply the audited pair matrix.
        self.model.opt.disableflags |= int(
            mujoco.mjtDisableBit.mjDSBL_FILTERPARENT
        )
        self.fk_data = mujoco.MjData(self.model)
        self.guard = self._load_guard()
        self.qpos_addresses = []
        for name in JOINTS:
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            self.qpos_addresses.append(int(self.model.jnt_qposadr[joint_id]))
        self.tcp_site_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_SITE, "tcp_nominal"
        )
        if self.tcp_site_id < 0:
            raise RuntimeError("MJCF site tcp_nominal is required")

        self.raw_state: JointState | None = None
        self.public_state: JointState | None = None
        self.mujoco_tcp: PoseStamped | None = None
        self.bridge_status: dict[str, Any] = {}
        self.raw_state_receipt_monotonic: float | None = None
        self.public_state_receipt_monotonic: float | None = None
        self.bridge_status_receipt_monotonic: float | None = None
        self.mujoco_tcp_receipt_monotonic: float | None = None
        self.active_trace_id: str | None = None
        self.active_trace: list[dict[str, Any]] = []
        self.active_raw_samples: list[dict[str, Any]] = []
        self.active_public_samples: list[dict[str, Any]] = []
        self.create_subscription(
            JointState, "/mujoco/joint_states_raw", self._raw_callback, 50
        )
        self.create_subscription(JointState, "/joint_states", self._public_callback, 50)
        self.create_subscription(
            PoseStamped,
            "/mujoco_bridge/tcp_nominal_pose",
            self._tcp_callback,
            50,
        )
        self.create_subscription(
            String, "/mujoco_bridge/status", self._status_callback, 50
        )
        self.create_subscription(
            JointTrajectoryControllerState,
            "/arm_controller/controller_state",
            self._controller_state_callback,
            100,
        )

        self.validity_client = self.create_client(
            GetStateValidity, "/check_state_validity"
        )
        self.ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.list_controllers_client = self.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        self.controller_manager_parameters_client = self.create_client(
            GetParameters, "/controller_manager/get_parameters"
        )
        self.arm_controller_parameters_client = self.create_client(
            GetParameters, "/arm_controller/get_parameters"
        )
        self.get_planning_scene_client = self.create_client(
            GetPlanningScene, "/get_planning_scene"
        )
        self.move_group_client = ActionClient(self, MoveGroup, "/move_action")
        self.execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, "/execute_trajectory"
        )
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def _load_guard(self):
        path = self.model_path.parent / "kinematic_guard.py"
        spec = importlib.util.spec_from_file_location("v15_13_guard_for_qa", path)
        if spec is None or spec.loader is None:
            raise RuntimeError(f"cannot load {path}")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module.KinematicGuard(
            model=self.model, data=mujoco.MjData(self.model)
        )

    def _raw_callback(self, message: JointState) -> None:
        self.raw_state = message
        self.raw_state_receipt_monotonic = time.monotonic()
        if self.active_trace_id is not None:
            values = state_values(message)
            if values is not None:
                velocities = state_velocity_values(message)
                self.active_raw_samples.append(
                    {
                        "stamp_ns": int(message.header.stamp.sec) * 1_000_000_000
                        + int(message.header.stamp.nanosec),
                        "position_rad": values.tolist(),
                        "velocity_rad_s": (
                            velocities.tolist() if velocities is not None else []
                        ),
                    }
                )

    def _public_callback(self, message: JointState) -> None:
        self.public_state = message
        self.public_state_receipt_monotonic = time.monotonic()
        if self.active_trace_id is not None:
            values = state_values(message)
            if values is not None:
                velocities = state_velocity_values(message)
                self.active_public_samples.append(
                    {
                        "stamp_ns": int(message.header.stamp.sec) * 1_000_000_000
                        + int(message.header.stamp.nanosec),
                        "position_rad": values.tolist(),
                        "velocity_rad_s": (
                            velocities.tolist() if velocities is not None else []
                        ),
                    }
                )

    def _tcp_callback(self, message: PoseStamped) -> None:
        self.mujoco_tcp = message
        self.mujoco_tcp_receipt_monotonic = time.monotonic()

    def _status_callback(self, message: String) -> None:
        try:
            self.bridge_status = json.loads(message.data)
            self.bridge_status_receipt_monotonic = time.monotonic()
        except json.JSONDecodeError:
            self.bridge_status = {"parse_error": message.data}
            self.bridge_status_receipt_monotonic = None

    def _controller_state_callback(
        self, message: JointTrajectoryControllerState
    ) -> None:
        if self.active_trace_id is None:
            return
        self.active_trace.append(
            {
                "time_ns": int(message.header.stamp.sec) * 1_000_000_000
                + int(message.header.stamp.nanosec),
                "source_stamp_ns": int(message.header.stamp.sec)
                * 1_000_000_000
                + int(message.header.stamp.nanosec),
                "receipt_time_ns": time.monotonic_ns(),
                "joint_names": list(message.joint_names),
                "desired_position_rad": list(message.desired.positions),
                "desired_velocity_rad_s": list(message.desired.velocities),
                "desired_time_from_start_s": duration_seconds(
                    message.desired.time_from_start
                ),
                "actual_position_rad": list(message.actual.positions),
                "actual_velocity_rad_s": list(message.actual.velocities),
                "error_position_rad": list(message.error.positions),
            }
        )

    def wait_future(self, future, timeout_sec: float):
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        if not future.done():
            raise TimeoutError(f"future timed out after {timeout_sec}s")
        return future.result()

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def audit_controller_runtime_parameters(self) -> dict[str, Any]:
        manager_request = GetParameters.Request()
        manager_request.names = ["update_rate"]
        manager_response = self.wait_future(
            self.controller_manager_parameters_client.call_async(manager_request),
            timeout_sec=10.0,
        )
        controller_request = GetParameters.Request()
        controller_request.names = [
            "state_publish_rate",
            *[f"gains.{joint}.angle_wraparound" for joint in JOINTS],
        ]
        controller_response = self.wait_future(
            self.arm_controller_parameters_client.call_async(controller_request),
            timeout_sec=10.0,
        )
        manager_value = manager_response.values[0]
        controller_values = controller_response.values
        update_rate_hz = (
            int(manager_value.integer_value)
            if manager_value.type == ParameterType.PARAMETER_INTEGER
            else None
        )
        state_publish_rate_hz = (
            float(controller_values[0].double_value)
            if controller_values[0].type == ParameterType.PARAMETER_DOUBLE
            else None
        )
        angle_wraparound = {
            joint: (
                bool(value.bool_value)
                if value.type == ParameterType.PARAMETER_BOOL
                else None
            )
            for joint, value in zip(JOINTS, controller_values[1:])
        }
        passed = bool(
            update_rate_hz == 50
            and state_publish_rate_hz == 50.0
            and angle_wraparound == {
                "J1": True,
                "J2": False,
                "J3": False,
                "J4": False,
                "J5": False,
                "J6": False,
            }
        )
        return {
            "controller_manager_update_rate_hz": update_rate_hz,
            "arm_controller_state_publish_rate_hz": state_publish_rate_hz,
            "angle_wraparound": angle_wraparound,
            "pass": passed,
        }

    def wait_ready(self) -> None:
        if not self.validity_client.wait_for_service(15.0):
            raise RuntimeError("/check_state_validity unavailable")
        if not self.ik_client.wait_for_service(15.0):
            raise RuntimeError("/compute_ik unavailable")
        if not self.move_group_client.wait_for_server(15.0):
            raise RuntimeError("/move_action unavailable")
        if not self.execute_trajectory_client.wait_for_server(15.0):
            raise RuntimeError("/execute_trajectory unavailable")
        if not self.list_controllers_client.wait_for_service(15.0):
            raise RuntimeError("/controller_manager/list_controllers unavailable")
        if not self.controller_manager_parameters_client.wait_for_service(15.0):
            raise RuntimeError("/controller_manager/get_parameters unavailable")
        if not self.arm_controller_parameters_client.wait_for_service(15.0):
            raise RuntimeError("/arm_controller/get_parameters unavailable")
        if not self.get_planning_scene_client.wait_for_service(15.0):
            raise RuntimeError("/get_planning_scene unavailable")
        deadline = time.monotonic() + 15.0
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            if (
                state_values(self.raw_state) is not None
                and state_values(self.public_state) is not None
                and self.mujoco_tcp is not None
                and self.bridge_status.get("schema")
                == "go-m8010-arm-v15.14-mujoco-bridge-status/1.0"
            ):
                if (
                    self.bridge_status.get("execution_mode")
                    != "kinematic_position_tracking"
                    or self.bridge_status.get("dynamics_valid") is not False
                    or self.bridge_status.get("parent_collision_filter_disabled")
                    is not True
                    or self.bridge_status.get("fault_latched") is not False
                    or self.bridge_status.get("command_stream_primed") is not True
                    or self.bridge_status.get("runtime_mesh_integrity", {}).get(
                        "pass"
                    )
                    is not True
                ):
                    raise RuntimeError(
                        f"invalid MuJoCo bridge status: {self.bridge_status}"
                    )
                envelope = self.bridge_status.get(
                    "simulation_execution_envelope", {}
                )
                if (
                    envelope.get("max_velocity_rad_s") != MAX_VELOCITY
                    or envelope.get("max_acceleration_rad_s2")
                    != MAX_ACCELERATION
                ):
                    raise RuntimeError(
                        "MuJoCo bridge execution envelope does not match the "
                        f"accepted V15.14 profile: {envelope}"
                    )
                return
        raise RuntimeError("joint/TCP state topics did not become ready")

    def bridge_runtime_contract(self) -> dict[str, Any]:
        status = dict(self.bridge_status)
        envelope = status.get("simulation_execution_envelope", {})
        model = status.get("model", {})
        collision_contract = status.get("collision_contract", {})
        guard = status.get("kinematic_guard", {})
        runtime_mesh_integrity = status.get("runtime_mesh_integrity", {})
        j1_branch_normalization = status.get(
            "j1_continuous_branch_normalization", {}
        )
        controller_parameters = self.audit_controller_runtime_parameters()
        now = time.monotonic()
        receipt_ages = {
            "raw_state_s": (
                now - self.raw_state_receipt_monotonic
                if self.raw_state_receipt_monotonic is not None
                else None
            ),
            "public_state_s": (
                now - self.public_state_receipt_monotonic
                if self.public_state_receipt_monotonic is not None
                else None
            ),
            "bridge_status_s": (
                now - self.bridge_status_receipt_monotonic
                if self.bridge_status_receipt_monotonic is not None
                else None
            ),
            "mujoco_tcp_s": (
                now - self.mujoco_tcp_receipt_monotonic
                if self.mujoco_tcp_receipt_monotonic is not None
                else None
            ),
        }
        state_streams_fresh = bool(
            all(value is not None and value <= 0.25 for value in receipt_ages.values())
        )
        passed = bool(
            status.get("schema")
            == "go-m8010-arm-v15.14-mujoco-bridge-status/1.0"
            and status.get("execution_mode") == "kinematic_position_tracking"
            and status.get("dynamics_valid") is False
            and status.get("parent_collision_filter_disabled") is True
            and status.get("command_stream_primed") is True
            and status.get("command_subscription_history") == "keep_last"
            and status.get("command_subscription_depth") == 1
            and j1_branch_normalization.get("enabled") is True
            and j1_branch_normalization.get("joint_name") == "J1"
            and j1_branch_normalization.get("method")
            == "nearest_equivalent_to_current"
            and j1_branch_normalization.get("pi_tie_break")
            == "match_joint_trajectory_controller"
            and isinstance(
                j1_branch_normalization.get("accepted_adjustment_count"), int
            )
            and j1_branch_normalization.get("accepted_adjustment_count") >= 0
            and math.isfinite(
                float(j1_branch_normalization.get("max_abs_adjustment_rad", math.nan))
            )
            and controller_parameters["pass"]
            and status.get("moving_command_watchdog_active") is True
            and status.get("latch_fault_enabled") is True
            and status.get("moving_watchdog_timeout_count") == 0
            and status.get("fault_latched") is False
            and envelope.get("max_velocity_rad_s") == MAX_VELOCITY
            and envelope.get("max_acceleration_rad_s2") == MAX_ACCELERATION
            and envelope.get("publish_rate_hz") == 50.0
            and envelope.get("absolute_calculation_tolerance") == 0.02
            and envelope.get("velocity_consistency_tolerance_rad_s") == 0.04
            and envelope.get("command_timeout_s") == 0.1
            and envelope.get("max_command_source_age_s", math.inf) <= 0.1
            and envelope.get("prime_position_tolerance_rad") == 1.0e-6
            and envelope.get("prime_velocity_tolerance_rad_s") == 1.0e-6
            and envelope.get("guard_max_step_deg") == 0.25
            and model.get("path") == str(self.model_path)
            and model.get("sha256") == self.model_sha256
            and collision_contract.get("path")
            == str(self.collision_contract_path)
            and collision_contract.get("sha256")
            == self.collision_contract_sha256
            and collision_contract.get("schema")
            == "go-m8010-arm-v15.14-self-collision-pair-contract/2.0"
            and collision_contract.get("proxy_count") == 25
            and collision_contract.get("all_unordered_pair_count") == 300
            and collision_contract.get("runtime_full_pair_count") == 231
            and collision_contract.get("runtime_excluded_pair_count") == 69
            and guard.get("path") == str(self.guard_path)
            and guard.get("sha256") == self.guard_sha256
            and runtime_mesh_integrity.get("pass") is True
            and runtime_mesh_integrity.get("runtime_manifest_sha256")
            == ACCEPTED_RUNTIME_MESH_MANIFEST_SHA256
            and runtime_mesh_integrity.get("mesh_count") == 1008
            and runtime_mesh_integrity.get("visual_chunk_count") == 45
            and runtime_mesh_integrity.get("collision_piece_count") == 961
            and runtime_mesh_integrity.get("motion_proxy_piece_count") == 2
            and runtime_mesh_integrity.get("total_bytes") == 383_827_772
            and runtime_mesh_integrity.get("asset_set_sha256")
            == ACCEPTED_RUNTIME_ASSET_SET_SHA256
            and runtime_mesh_integrity.get("source_manifest_sha256")
            == ACCEPTED_SOURCE_MESH_MANIFEST_SHA256
            and runtime_mesh_integrity.get("explicit_geom_pair_count") == 82
            and runtime_mesh_integrity.get("explicit_geom_pair_set_pass") is True
            and state_streams_fresh
        )
        return {
            "status": status,
            "expected_model": {
                "path": str(self.model_path),
                "sha256": self.model_sha256,
            },
            "expected_collision_contract": {
                "path": str(self.collision_contract_path),
                "sha256": self.collision_contract_sha256,
            },
            "expected_kinematic_guard": {
                "path": str(self.guard_path),
                "sha256": self.guard_sha256,
            },
            "expected_runtime_mesh_integrity": {
                "runtime_manifest_sha256": ACCEPTED_RUNTIME_MESH_MANIFEST_SHA256,
                "mesh_count": 1008,
                "visual_chunk_count": 45,
                "collision_piece_count": 961,
                "motion_proxy_piece_count": 2,
                "total_bytes": 383_827_772,
                "asset_set_sha256": ACCEPTED_RUNTIME_ASSET_SET_SHA256,
                "source_manifest_sha256": ACCEPTED_SOURCE_MESH_MANIFEST_SHA256,
                "explicit_geom_pair_count": 82,
                "explicit_geom_pair_set_pass": True,
            },
            "j1_continuous_branch_normalization": j1_branch_normalization,
            "controller_runtime_parameters": controller_parameters,
            "receipt_ages": receipt_ages,
            "state_streams_fresh": state_streams_fresh,
            "pass": passed,
        }

    def audit_ground_scene(self) -> dict[str, Any]:
        request = GetPlanningScene.Request()
        request.components.components = (
            PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
        )
        response = self.wait_future(
            self.get_planning_scene_client.call_async(request), timeout_sec=10.0
        )
        matches = [
            obj
            for obj in response.scene.world.collision_objects
            if obj.id == "v15_14_mujoco_ground"
        ]
        exact = False
        details: dict[str, Any] = {"object_count": len(matches)}
        if len(matches) == 1 and len(matches[0].primitives) == 1:
            obj = matches[0]
            primitive = obj.primitives[0]
            pose = obj.primitive_poses[0]
            dimensions = list(primitive.dimensions)
            object_position = np.array(
                [obj.pose.position.x, obj.pose.position.y, obj.pose.position.z],
                dtype=float,
            )
            object_quaternion = np.array(
                [
                    obj.pose.orientation.x,
                    obj.pose.orientation.y,
                    obj.pose.orientation.z,
                    obj.pose.orientation.w,
                ],
                dtype=float,
            )
            primitive_position = np.array(
                [pose.position.x, pose.position.y, pose.position.z], dtype=float
            )
            primitive_quaternion = np.array(
                [
                    pose.orientation.x,
                    pose.orientation.y,
                    pose.orientation.z,
                    pose.orientation.w,
                ],
                dtype=float,
            )
            object_rotation = Rotation.from_quat(object_quaternion)
            effective_rotation = object_rotation * Rotation.from_quat(
                primitive_quaternion
            )
            center = (
                object_position + object_rotation.apply(primitive_position)
            ).tolist()
            quaternion = effective_rotation.as_quat().tolist()
            source_primitive_quaternion = [
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w,
            ]
            exact = bool(
                primitive.type == SolidPrimitive.BOX
                and np.max(np.abs(np.asarray(dimensions) - [4.0, 4.0, 0.02]))
                <= 1.0e-12
                and np.max(np.abs(np.asarray(center) - [0.0, 0.0, -0.10]))
                <= 1.0e-12
                and np.max(np.abs(np.asarray(quaternion) - [0.0, 0.0, 0.0, 1.0]))
                <= 1.0e-12
            )
            details.update(
                {
                    "frame_id": obj.header.frame_id,
                    "dimensions_m": dimensions,
                    "center_m": center,
                    "quaternion_xyzw": quaternion,
                    "stored_object_pose": {
                        "position_m": object_position.tolist(),
                        "quaternion_xyzw": object_quaternion.tolist(),
                    },
                    "stored_primitive_pose": {
                        "position_m": primitive_position.tolist(),
                        "quaternion_xyzw": source_primitive_quaternion,
                    },
                }
            )
            exact = exact and obj.header.frame_id == "world"
        ground_geom_id = mujoco.mj_name2id(
            self.model, mujoco.mjtObj.mjOBJ_GEOM, "ground"
        )
        mujoco_ground_pass = False
        mujoco_ground: dict[str, Any] = {"geom_id": int(ground_geom_id)}
        if ground_geom_id >= 0:
            geom_position = self.model.geom_pos[ground_geom_id].tolist()
            geom_type = int(self.model.geom_type[ground_geom_id])
            contype = int(self.model.geom_contype[ground_geom_id])
            conaffinity = int(self.model.geom_conaffinity[ground_geom_id])
            mujoco_ground_pass = bool(
                geom_type == int(mujoco.mjtGeom.mjGEOM_PLANE)
                and np.max(np.abs(np.asarray(geom_position) - [0.0, 0.0, -0.09]))
                <= 1.0e-12
                and contype == 4
                and conaffinity == 0
            )
            mujoco_ground.update(
                {
                    "type": geom_type,
                    "position_m": geom_position,
                    "contype": contype,
                    "conaffinity": conaffinity,
                    "pass": mujoco_ground_pass,
                }
            )
        details["mujoco_ground"] = mujoco_ground
        moving_collision_geom_count = 0
        ground_affinity_violations = []
        for geom_id in range(self.model.ngeom):
            geom_name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_GEOM, geom_id
            )
            if not geom_name or not geom_name.startswith("collision__"):
                continue
            body_id = int(self.model.geom_bodyid[geom_id])
            body_name = mujoco.mj_id2name(
                self.model, mujoco.mjtObj.mjOBJ_BODY, body_id
            )
            # Fixed base geometry intentionally does not collide with its
            # support plane. UpperArm Motion is pair-scoped (contype=0) and
            # its full upper-arm counterpart already covers ground contact.
            if body_name == "base_link" or "UpperArm_Motion" in geom_name:
                continue
            moving_collision_geom_count += 1
            if int(self.model.geom_conaffinity[geom_id]) & 4 == 0:
                ground_affinity_violations.append(
                    {
                        "geom_id": geom_id,
                        "geom_name": geom_name,
                        "body_name": body_name,
                        "conaffinity": int(self.model.geom_conaffinity[geom_id]),
                    }
                )
        robot_ground_collision_bits_pass = bool(
            moving_collision_geom_count > 0 and not ground_affinity_violations
        )
        details["mujoco_robot_ground_affinity"] = {
            "moving_collision_geom_count": moving_collision_geom_count,
            "violation_count": len(ground_affinity_violations),
            "violations": ground_affinity_violations,
            "pass": robot_ground_collision_bits_pass,
        }
        details["pass"] = bool(
            exact and mujoco_ground_pass and robot_ground_collision_bits_pass
        )
        return details

    def audit_ros_graph(self) -> dict[str, Any]:
        def endpoint_names(rows) -> list[str]:
            return sorted(
                {
                    f"{row.node_namespace.rstrip('/')}/{row.node_name}"
                    if row.node_namespace != "/"
                    else f"/{row.node_name}"
                    for row in rows
                }
            )

        publisher_endpoints = self.get_publishers_info_by_topic("/joint_states")
        joint_state_publishers = endpoint_names(publisher_endpoints)
        command_publishers = endpoint_names(
            self.get_publishers_info_by_topic("/mujoco/joint_commands")
        )
        command_subscribers = endpoint_names(
            self.get_subscriptions_info_by_topic("/mujoco/joint_commands")
        )
        raw_state_publishers = endpoint_names(
            self.get_publishers_info_by_topic("/mujoco/joint_states_raw")
        )
        raw_state_subscribers = endpoint_names(
            self.get_subscriptions_info_by_topic("/mujoco/joint_states_raw")
        )
        audited_raw_state_subscribers = [
            name
            for name in raw_state_subscribers
            if name != self.get_fully_qualified_name()
        ]

        controllers_response = self.wait_future(
            self.list_controllers_client.call_async(ListControllers.Request()),
            timeout_sec=10.0,
        )
        controllers = {
            controller.name: {
                "type": controller.type,
                "state": controller.state,
            }
            for controller in controllers_response.controller
        }
        arm_controller = controllers.get("arm_controller", {})
        state_broadcaster = controllers.get("joint_state_broadcaster", {})

        action_servers: dict[str, list[dict[str, Any]]] = {}
        for node_name, node_namespace in self.get_node_names_and_namespaces():
            try:
                rows = get_action_server_names_and_types_by_node(
                    self, node_name, node_namespace
                )
            except Exception:
                continue
            full_node_name = (
                f"{node_namespace.rstrip('/')}/{node_name}"
                if node_namespace != "/"
                else f"/{node_name}"
            )
            for action_name, action_types in rows:
                action_servers.setdefault(action_name, []).append(
                    {"node": full_node_name, "types": sorted(action_types)}
                )

        follow_action = "/arm_controller/follow_joint_trajectory"
        follow_providers = action_servers.get(follow_action, [])
        follow_provider_pass = bool(
            len(follow_providers) == 1
            and follow_providers[0]["node"] == "/arm_controller"
            and "control_msgs/action/FollowJointTrajectory"
            in follow_providers[0]["types"]
        )
        execute_action = "/execute_trajectory"
        execute_providers = action_servers.get(execute_action, [])
        execute_provider_pass = bool(
            len(execute_providers) == 1
            and execute_providers[0]["node"] == "/move_group"
            and "moveit_msgs/action/ExecuteTrajectory"
            in execute_providers[0]["types"]
        )
        bridge_actions = sorted(
            action_name
            for action_name, providers in action_servers.items()
            if any(provider["node"] == "/mujoco_bridge" for provider in providers)
        )
        single_state_source = joint_state_publishers == [
            "/joint_state_broadcaster"
        ]
        controller_pass = bool(
            arm_controller.get("type")
            == "joint_trajectory_controller/JointTrajectoryController"
            and arm_controller.get("state") == "active"
            and state_broadcaster.get("type")
            == "joint_state_broadcaster/JointStateBroadcaster"
            and state_broadcaster.get("state") == "active"
        )
        no_private_bridge_action = not bridge_actions
        standard_topic_path_pass = bool(
            command_publishers
            == ["/topic_based_ros2_control_MujocoTopicSystem"]
            and command_subscribers == ["/mujoco_bridge"]
            and raw_state_publishers == ["/mujoco_bridge"]
            and audited_raw_state_subscribers
            == ["/topic_based_ros2_control_MujocoTopicSystem"]
        )
        return {
            "follow_joint_trajectory_server": follow_action,
            "follow_joint_trajectory_providers": follow_providers,
            "follow_joint_trajectory_provider_pass": follow_provider_pass,
            "execute_trajectory_server": execute_action,
            "execute_trajectory_providers": execute_providers,
            "execute_trajectory_provider_pass": execute_provider_pass,
            "controllers": controllers,
            "controller_types_and_states_pass": controller_pass,
            "joint_state_publishers": joint_state_publishers,
            "single_joint_state_source": single_state_source,
            "mujoco_joint_command_publishers": command_publishers,
            "mujoco_joint_command_subscribers": command_subscribers,
            "mujoco_raw_state_publishers": raw_state_publishers,
            "mujoco_raw_state_subscribers": raw_state_subscribers,
            "mujoco_raw_state_hardware_subscribers": audited_raw_state_subscribers,
            "standard_topic_based_ros2_control_path": standard_topic_path_pass,
            "mujoco_bridge_action_servers": bridge_actions,
            "no_private_moveit_mujoco_edge": no_private_bridge_action,
            "pass": bool(
                follow_provider_pass
                and execute_provider_pass
                and controller_pass
                and single_state_source
                and no_private_bridge_action
                and standard_topic_path_pass
            ),
        }

    def fk_pose(self, q: np.ndarray) -> dict[str, Any]:
        self.fk_data.qpos[self.qpos_addresses] = q
        mujoco.mj_forward(self.model, self.fk_data)
        position = self.fk_data.site_xpos[self.tcp_site_id].copy()
        matrix = self.fk_data.site_xmat[self.tcp_site_id].reshape(3, 3).copy()
        quaternion = Rotation.from_matrix(matrix).as_quat()
        if quaternion[3] < 0.0:
            quaternion *= -1.0
        return {
            "frame": "world",
            "position_m": position.tolist(),
            "quaternion_xyzw": quaternion.tolist(),
        }

    def within_bounds(self, q: np.ndarray) -> tuple[bool, list[dict[str, Any]]]:
        violations = []
        for index, name in enumerate(JOINTS):
            if name not in POSITION_BOUNDS:
                continue
            lower, upper = POSITION_BOUNDS[name]
            if q[index] < lower or q[index] > upper:
                violations.append(
                    {
                        "joint": name,
                        "value": float(q[index]),
                        "allowed": [lower, upper],
                    }
                )
        return not violations, violations

    def moveit_validity(self, q: np.ndarray) -> dict[str, Any]:
        request = GetStateValidity.Request()
        request.group_name = "arm"
        request.robot_state.joint_state.name = list(JOINTS)
        request.robot_state.joint_state.position = q.astype(float).tolist()
        response = self.wait_future(
            self.validity_client.call_async(request), timeout_sec=20.0
        )
        return {
            "valid": bool(response.valid),
            "contacts": [
                {
                    "body1": contact.contact_body_1,
                    "body2": contact.contact_body_2,
                    "depth_m": float(contact.depth),
                }
                for contact in response.contacts
            ],
        }

    def compute_ik(self, pose: dict[str, Any], seed: np.ndarray) -> dict[str, Any]:
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = "arm"
        ik.ik_link_name = "tcp_nominal"
        ik.avoid_collisions = True
        ik.timeout = Duration(seconds=2.0).to_msg()
        ik.robot_state.joint_state.name = list(JOINTS)
        ik.robot_state.joint_state.position = seed.astype(float).tolist()
        ik.pose_stamped.header.frame_id = "world"
        p = pose["position_m"]
        q = pose["quaternion_xyzw"]
        ik.pose_stamped.pose.position.x = p[0]
        ik.pose_stamped.pose.position.y = p[1]
        ik.pose_stamped.pose.position.z = p[2]
        ik.pose_stamped.pose.orientation.x = q[0]
        ik.pose_stamped.pose.orientation.y = q[1]
        ik.pose_stamped.pose.orientation.z = q[2]
        ik.pose_stamped.pose.orientation.w = q[3]
        response = self.wait_future(self.ik_client.call_async(request), 5.0)
        solution_lookup = dict(
            zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            )
        )
        solution = [solution_lookup[name] for name in JOINTS] if all(
            name in solution_lookup for name in JOINTS
        ) else []
        return {
            "error_code": int(response.error_code.val),
            "kinematic_solution_found": int(response.error_code.val) == 1,
            "collision_aware_pass": int(response.error_code.val) == 1,
            "solution_rad": solution,
        }

    def constraints_for_target(
        self, target: dict[str, Any], pose: dict[str, Any]
    ) -> Constraints:
        constraints = Constraints(name=target["id"])
        if target["planning_mode"] == "joint":
            constraints.joint_constraints = [
                JointConstraint(
                    joint_name=name,
                    position=float(value),
                    tolerance_above=1.0e-6,
                    tolerance_below=1.0e-6,
                    weight=1.0,
                )
                for name, value in zip(JOINTS, target["joint_position_rad"])
            ]
            return constraints

        position = PositionConstraint()
        position.header.frame_id = "world"
        position.link_name = "tcp_nominal"
        sphere = SolidPrimitive(type=SolidPrimitive.SPHERE, dimensions=[2.0e-5])
        sphere_pose = Pose()
        sphere_pose.position.x, sphere_pose.position.y, sphere_pose.position.z = pose[
            "position_m"
        ]
        sphere_pose.orientation.w = 1.0
        position.constraint_region.primitives = [sphere]
        position.constraint_region.primitive_poses = [sphere_pose]
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header.frame_id = "world"
        orientation.link_name = "tcp_nominal"
        quaternion = pose["quaternion_xyzw"]
        orientation.orientation.x = quaternion[0]
        orientation.orientation.y = quaternion[1]
        orientation.orientation.z = quaternion[2]
        orientation.orientation.w = quaternion[3]
        orientation.absolute_x_axis_tolerance = 2.0e-4
        orientation.absolute_y_axis_tolerance = 2.0e-4
        orientation.absolute_z_axis_tolerance = 2.0e-4
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        return constraints

    def send_move_group(
        self, target: dict[str, Any], pose: dict[str, Any], plan_only: bool
    ) -> dict[str, Any]:
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = "arm"
        request.pipeline_id = "ompl"
        request.planner_id = "RRTConnectkConfigDefault"
        request.num_planning_attempts = 5
        request.allowed_planning_time = 10.0
        request.max_velocity_scaling_factor = 0.40
        request.max_acceleration_scaling_factor = 0.40
        request.goal_constraints = [self.constraints_for_target(target, pose)]
        goal.planning_options.plan_only = plan_only
        goal.planning_options.replan = False

        self.active_trace_id = target["id"]
        self.active_trace = []
        self.active_raw_samples = []
        self.active_public_samples = []
        goal_handle = self.wait_future(
            self.move_group_client.send_goal_async(goal), timeout_sec=15.0
        )
        if not goal_handle.accepted:
            self.active_trace_id = None
            return {
                "accepted": False,
                "goal_uuid": bytes(goal_handle.goal_id.uuid).hex(),
                "status": None,
                "error_code": None,
                "planning_time_s": None,
                "trajectory": None,
                "robot_trajectory": None,
                "trace": [],
                "raw_samples": self.active_raw_samples,
                "public_samples": self.active_public_samples,
            }
        result_wrapper = self.wait_future(
            goal_handle.get_result_async(), timeout_sec=180.0
        )
        self.spin_for(0.30)
        trace = self.active_trace
        raw_samples = self.active_raw_samples
        public_samples = self.active_public_samples
        self.active_trace_id = None
        result = result_wrapper.result
        return {
            "accepted": True,
            "goal_uuid": bytes(goal_handle.goal_id.uuid).hex(),
            "status": int(result_wrapper.status),
            "error_code": int(result.error_code.val),
            "planning_time_s": float(result.planning_time),
            "trajectory": result.planned_trajectory.joint_trajectory,
            "robot_trajectory": result.planned_trajectory,
            "trace": trace,
            "raw_samples": raw_samples,
            "public_samples": public_samples,
        }

    def execute_validated_trajectory(
        self, target_id: str, robot_trajectory
    ) -> dict[str, Any]:
        """Execute one already validated MoveIt RobotTrajectory exactly once.

        `/execute_trajectory` belongs to MoveIt's TrajectoryExecutionManager,
        which selects the configured MoveItSimpleControllerManager handle and
        forwards this exact trajectory to the standard
        `/arm_controller/follow_joint_trajectory` action.  The MuJoCo bridge is
        not an action server and remains behind ros2_control.
        """
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = robot_trajectory
        joint_trajectory = robot_trajectory.joint_trajectory
        joint_order = [joint_trajectory.joint_names.index(name) for name in JOINTS]
        expected_start_position = [
            float(joint_trajectory.points[0].positions[index])
            for index in joint_order
        ]
        expected_final_position = [
            float(joint_trajectory.points[-1].positions[index])
            for index in joint_order
        ]
        execute_request_cdr_sha256 = hashlib.sha256(
            serialize_message(robot_trajectory)
        ).hexdigest()
        execute_request_semantic_sha256 = semantic_message_sha256(
            robot_trajectory
        )
        expected_duration_s = duration_seconds(
            robot_trajectory.joint_trajectory.points[-1].time_from_start
        )
        self.active_trace_id = target_id
        self.active_trace = []
        self.active_raw_samples = []
        self.active_public_samples = []
        goal_handle = self.wait_future(
            self.execute_trajectory_client.send_goal_async(goal), timeout_sec=15.0
        )
        if not goal_handle.accepted:
            self.active_trace_id = None
            return {
                "accepted": False,
                "goal_uuid": bytes(goal_handle.goal_id.uuid).hex(),
                "status": None,
                "error_code": None,
                "expected_duration_s": expected_duration_s,
                "expected_start_position_rad": expected_start_position,
                "expected_final_position_rad": expected_final_position,
                "execute_request_cdr_sha256": execute_request_cdr_sha256,
                "execute_request_semantic_sha256": (
                    execute_request_semantic_sha256
                ),
                "trace": [],
                "raw_samples": self.active_raw_samples,
                "public_samples": self.active_public_samples,
            }
        result_wrapper = self.wait_future(
            goal_handle.get_result_async(), timeout_sec=180.0
        )
        self.spin_for(0.30)
        trace = self.active_trace
        raw_samples = self.active_raw_samples
        public_samples = self.active_public_samples
        self.active_trace_id = None
        return {
            "accepted": True,
            "goal_uuid": bytes(goal_handle.goal_id.uuid).hex(),
            "status": int(result_wrapper.status),
            "error_code": int(result_wrapper.result.error_code.val),
            "expected_duration_s": expected_duration_s,
            "expected_start_position_rad": expected_start_position,
            "expected_final_position_rad": expected_final_position,
            "execute_request_cdr_sha256": execute_request_cdr_sha256,
            "execute_request_semantic_sha256": execute_request_semantic_sha256,
            "trace": trace,
            "raw_samples": raw_samples,
            "public_samples": public_samples,
        }

    def execution_trace_metrics(
        self, move_result: dict[str, Any], validated_trajectory
    ) -> dict[str, Any]:
        controller_trace = move_result["trace"]
        tracking_errors = []
        velocity_tracking_errors = []
        for row in controller_trace:
            desired = row["desired_position_rad"]
            actual = row["actual_position_rad"]
            if len(desired) == 6 and len(actual) == 6:
                tracking_errors.append(
                    float(
                        np.max(
                            np.abs(
                                wrapped_joint_error(
                                    np.asarray(desired, dtype=float),
                                    np.asarray(actual, dtype=float),
                                )
                            )
                        )
                    )
                )
            desired_velocity = np.asarray(
                row["desired_velocity_rad_s"], dtype=float
            )
            actual_velocity = np.asarray(
                row["actual_velocity_rad_s"], dtype=float
            )
            if (
                desired_velocity.shape == (6,)
                and actual_velocity.shape == (6,)
                and np.all(np.isfinite(desired_velocity))
                and np.all(np.isfinite(actual_velocity))
            ):
                velocity_tracking_errors.append(
                    float(np.max(np.abs(desired_velocity - actual_velocity)))
                )
        controller_time_gaps = [
            (second["time_ns"] - first["time_ns"]) * 1.0e-9
            for first, second in zip(controller_trace, controller_trace[1:])
        ]
        controller_receipt_time_gaps = [
            (second["receipt_time_ns"] - first["receipt_time_ns"]) * 1.0e-9
            for first, second in zip(controller_trace, controller_trace[1:])
        ]
        controller_timestamps_strictly_increasing = bool(
            len(controller_trace) >= 2
            and all(gap > 0.0 for gap in controller_time_gaps)
        )
        controller_receipts_strictly_increasing = bool(
            len(controller_trace) >= 2
            and all(gap > 0.0 for gap in controller_receipt_time_gaps)
        )
        controller_trace_span_s = (
            (controller_trace[-1]["time_ns"] - controller_trace[0]["time_ns"])
            * 1.0e-9
            if len(controller_trace) >= 2
            else 0.0
        )
        expected_duration_s = float(move_result.get("expected_duration_s") or 0.0)
        expected_start = np.asarray(
            move_result.get("expected_start_position_rad", []), dtype=float
        )
        expected_final = np.asarray(
            move_result.get("expected_final_position_rad", []), dtype=float
        )
        dynamic_controller_sample_count = 0
        dynamic_controller_trace_span_s = 0.0
        dynamic_controller_max_gap_s = math.inf
        dynamic_start_time_ns = None
        dynamic_end_time_ns = None
        controller_start_matches_plan = False
        controller_final_matches_plan = False
        controller_spline_position_errors: list[float] = []
        controller_spline_velocity_errors: list[float] = []
        controller_spline_times: list[float] = []
        controller_spline_invalid_sample_count = 0
        controller_spline_window_sample_count = 0
        if controller_trace:
            first_desired = np.asarray(
                controller_trace[0]["desired_position_rad"], dtype=float
            )
            final_desired = np.asarray(
                controller_trace[-1]["desired_position_rad"], dtype=float
            )
            controller_start_matches_plan = bool(
                expected_start.shape == (6,)
                and first_desired.shape == (6,)
                and np.max(
                    np.abs(wrapped_joint_error(expected_start, first_desired))
                )
                <= 1.0e-6
            )
            controller_final_matches_plan = bool(
                expected_final.shape == (6,)
                and final_desired.shape == (6,)
                and np.max(
                    np.abs(wrapped_joint_error(expected_final, final_desired))
                )
                <= 1.0e-6
            )
            moving_indices = []
            settled_indices = []
            previous_desired = None
            for row_index, row in enumerate(controller_trace):
                desired = np.asarray(row["desired_position_rad"], dtype=float)
                desired_velocity = np.asarray(
                    row["desired_velocity_rad_s"], dtype=float
                )
                desired_position_step = (
                    float(np.max(np.abs(desired - previous_desired)))
                    if desired.shape == (6,) and previous_desired is not None
                    else 0.0
                )
                is_final_settled = bool(
                    desired.shape == (6,)
                    and desired_velocity.shape == (6,)
                    and expected_final.shape == (6,)
                    and np.max(np.abs(desired - expected_final)) <= 1.0e-6
                    and np.max(np.abs(desired_velocity)) <= 1.0e-6
                )
                if (
                    desired.shape == (6,)
                    and desired_velocity.shape == (6,)
                    and not is_final_settled
                    and (
                        desired_position_step > 1.0e-6
                        or np.max(np.abs(desired_velocity)) > 1.0e-6
                    )
                ):
                    dynamic_controller_sample_count += 1
                    moving_indices.append(row_index)
                if is_final_settled:
                    settled_indices.append(row_index)
                if desired.shape == (6,):
                    previous_desired = desired
            if moving_indices:
                dynamic_start_index = max(0, moving_indices[0] - 1)
                settled_after_motion = [
                    index
                    for index in settled_indices
                    if index > moving_indices[-1]
                ]
                dynamic_end_index = (
                    settled_after_motion[0]
                    if settled_after_motion
                    else moving_indices[-1]
                )
                dynamic_controller_trace_span_s = (
                    controller_trace[dynamic_end_index]["time_ns"]
                    - controller_trace[dynamic_start_index]["time_ns"]
                ) * 1.0e-9
                dynamic_start_time_ns = int(
                    controller_trace[dynamic_start_index]["time_ns"]
                )
                dynamic_end_time_ns = int(
                    controller_trace[dynamic_end_index]["time_ns"]
                )
                dynamic_gaps = controller_time_gaps[
                    dynamic_start_index:dynamic_end_index
                ]
                dynamic_controller_max_gap_s = max(dynamic_gaps or [math.inf])
                # Compare every desired controller sample in the actual motion
                # window with the exact JTC spline that passed pre-execution
                # collision/limit validation.  JointTrajectoryControllerState
                # supplies the spline's source time via desired.time_from_start.
                # Post-settle controller states may legitimately report
                # time_from_start beyond the final waypoint.  The moving rows
                # themselves must all lie on and match the validated spline.
                spline_indices = moving_indices
                controller_spline_window_sample_count = len(spline_indices)
                for row_index in spline_indices:
                    row = controller_trace[row_index]
                    try:
                        if tuple(row.get("joint_names", ())) != JOINTS:
                            raise ValueError("controller joint order changed")
                        desired_time_s = float(
                            row.get("desired_time_from_start_s", math.nan)
                        )
                        desired_position = np.asarray(
                            row["desired_position_rad"], dtype=float
                        )
                        desired_velocity = np.asarray(
                            row["desired_velocity_rad_s"], dtype=float
                        )
                        if (
                            desired_position.shape != (6,)
                            or desired_velocity.shape != (6,)
                            or not np.all(np.isfinite(desired_position))
                            or not np.all(np.isfinite(desired_velocity))
                        ):
                            raise ValueError("invalid controller desired state")
                        expected_state = self.evaluate_jtc_trajectory_state(
                            validated_trajectory, desired_time_s
                        )
                        controller_spline_times.append(desired_time_s)
                        controller_spline_position_errors.append(
                            float(
                                np.max(
                                    np.abs(
                                        wrapped_joint_error(
                                            expected_state["position_rad"],
                                            desired_position,
                                        )
                                    )
                                )
                            )
                        )
                        controller_spline_velocity_errors.append(
                            float(
                                np.max(
                                    np.abs(
                                        desired_velocity
                                        - expected_state["velocity_rad_s"]
                                    )
                                )
                            )
                        )
                    except (KeyError, TypeError, ValueError):
                        controller_spline_invalid_sample_count += 1

        controller_spline_times_nondecreasing = bool(
            controller_spline_times
            and all(
                second + 1.0e-12 >= first
                for first, second in zip(
                    controller_spline_times, controller_spline_times[1:]
                )
            )
        )
        controller_spline_time_coverage_pass = bool(
            controller_spline_times
            and expected_duration_s > 0.0
            and min(controller_spline_times) <= 0.05
            and max(controller_spline_times) >= expected_duration_s - 0.05
            and max(controller_spline_times) - min(controller_spline_times)
            >= expected_duration_s * 0.90
        )
        max_controller_spline_position_error = max(
            controller_spline_position_errors or [math.inf]
        )
        max_controller_spline_velocity_error = max(
            controller_spline_velocity_errors or [math.inf]
        )
        controller_desired_matches_validated_spline = bool(
            controller_spline_window_sample_count >= 5
            and controller_spline_invalid_sample_count == 0
            and len(controller_spline_position_errors)
            == controller_spline_window_sample_count
            and len(controller_spline_velocity_errors)
            == controller_spline_window_sample_count
            and controller_spline_times_nondecreasing
            and controller_spline_time_coverage_pass
            and max_controller_spline_position_error <= 1.0e-6
            and max_controller_spline_velocity_error <= 1.0e-5
        )

        raw_samples = move_result["raw_samples"]
        public_samples = move_result["public_samples"]
        raw_time_gaps = [
            (second["stamp_ns"] - first["stamp_ns"]) * 1.0e-9
            for first, second in zip(raw_samples, raw_samples[1:])
        ]
        public_time_gaps = [
            (second["stamp_ns"] - first["stamp_ns"]) * 1.0e-9
            for first, second in zip(public_samples, public_samples[1:])
        ]
        raw_timestamps_strictly_increasing = bool(
            len(raw_samples) >= 2 and all(gap > 0.0 for gap in raw_time_gaps)
        )
        public_timestamps_strictly_increasing = bool(
            len(public_samples) >= 2
            and all(gap > 0.0 for gap in public_time_gaps)
        )
        raw_state_span_s = (
            (raw_samples[-1]["stamp_ns"] - raw_samples[0]["stamp_ns"])
            * 1.0e-9
            if len(raw_samples) >= 2
            else 0.0
        )
        public_state_span_s = (
            (public_samples[-1]["stamp_ns"] - public_samples[0]["stamp_ns"])
            * 1.0e-9
            if len(public_samples) >= 2
            else 0.0
        )
        dynamic_window_tolerance_ns = int(
            THRESHOLDS["max_state_pair_time_skew_s"] * 1.0e9
        )
        raw_covers_dynamic_window = bool(
            dynamic_start_time_ns is not None
            and dynamic_end_time_ns is not None
            and raw_samples
            and raw_samples[0]["stamp_ns"]
            <= dynamic_start_time_ns + dynamic_window_tolerance_ns
            and raw_samples[-1]["stamp_ns"]
            >= dynamic_end_time_ns - dynamic_window_tolerance_ns
        )
        public_covers_dynamic_window = bool(
            dynamic_start_time_ns is not None
            and dynamic_end_time_ns is not None
            and public_samples
            and public_samples[0]["stamp_ns"]
            <= dynamic_start_time_ns + dynamic_window_tolerance_ns
            and public_samples[-1]["stamp_ns"]
            >= dynamic_end_time_ns - dynamic_window_tolerance_ns
        )
        paired_errors = []
        paired_velocity_errors = []
        paired_skews = []
        if raw_samples and public_samples:
            raw_stamps = np.asarray([row["stamp_ns"] for row in raw_samples], dtype=np.int64)
            last_used_raw_index = -1
            for public_row in public_samples:
                public_stamp = int(public_row["stamp_ns"])
                candidate_indices = np.flatnonzero(
                    np.abs(raw_stamps - public_stamp)
                    <= int(THRESHOLDS["max_state_pair_time_skew_s"] * 1.0e9)
                )
                candidate_indices = candidate_indices[
                    candidate_indices > last_used_raw_index
                ]
                if not len(candidate_indices):
                    continue
                public_position = np.asarray(public_row["position_rad"], dtype=float)
                public_velocity = np.asarray(
                    public_row.get("velocity_rad_s", []), dtype=float
                )
                ranked_candidates = []
                for candidate_index in candidate_indices:
                    raw_position = np.asarray(
                        raw_samples[int(candidate_index)]["position_rad"], dtype=float
                    )
                    raw_velocity = np.asarray(
                        raw_samples[int(candidate_index)].get("velocity_rad_s", []),
                        dtype=float,
                    )
                    position_error = float(
                        np.max(np.abs(public_position - raw_position))
                    )
                    velocity_error = (
                        float(np.max(np.abs(public_velocity - raw_velocity)))
                        if public_velocity.shape == (6,)
                        and raw_velocity.shape == (6,)
                        else math.inf
                    )
                    signed_skew_s = (
                        public_stamp - int(raw_stamps[int(candidate_index)])
                    ) * 1.0e-9
                    if (
                        position_error
                        <= THRESHOLDS["public_raw_joint_sync_rad"]
                        and velocity_error
                        <= THRESHOLDS["public_raw_joint_velocity_sync_rad_s"]
                    ):
                        ranked_candidates.append(
                            (
                                position_error,
                                velocity_error,
                                abs(signed_skew_s),
                                signed_skew_s,
                                int(candidate_index),
                            )
                        )
                if not ranked_candidates:
                    continue
                (
                    position_error,
                    velocity_error,
                    _absolute_skew_s,
                    signed_skew_s,
                    index,
                ) = min(
                    ranked_candidates
                )
                last_used_raw_index = index
                paired_errors.append(position_error)
                if math.isfinite(velocity_error):
                    paired_velocity_errors.append(velocity_error)
                paired_skews.append(
                    signed_skew_s
                )
        max_tracking_error = max(tracking_errors or [math.inf])
        max_velocity_tracking_error = max(
            velocity_tracking_errors or [math.inf]
        )
        max_sync_error = max(paired_errors or [math.inf])
        max_velocity_sync_error = max(paired_velocity_errors or [math.inf])
        max_pair_skew = max([abs(value) for value in paired_skews] or [math.inf])
        min_signed_pair_skew = min(paired_skews or [math.inf])
        max_signed_pair_skew = max(paired_skews or [-math.inf])
        pair_coverage = (
            len(paired_errors) / len(public_samples) if public_samples else 0.0
        )
        return {
            "controller_sample_count": len(controller_trace),
            "controller_tracking_sample_count": len(tracking_errors),
            "controller_velocity_tracking_sample_count": len(
                velocity_tracking_errors
            ),
            "dynamic_controller_sample_count": dynamic_controller_sample_count,
            "expected_trajectory_duration_s": expected_duration_s,
            "controller_trace_span_s": controller_trace_span_s,
            "dynamic_controller_trace_span_s": dynamic_controller_trace_span_s,
            "controller_start_matches_planned_first_point": controller_start_matches_plan,
            "controller_final_matches_planned_last_point": controller_final_matches_plan,
            "controller_spline_window_sample_count": controller_spline_window_sample_count,
            "controller_spline_invalid_sample_count": controller_spline_invalid_sample_count,
            "controller_spline_times_nondecreasing": controller_spline_times_nondecreasing,
            "controller_spline_time_coverage_pass": controller_spline_time_coverage_pass,
            "max_controller_desired_validated_spline_position_error_rad": (
                max_controller_spline_position_error
                if controller_spline_position_errors
                else None
            ),
            "max_controller_desired_validated_spline_velocity_error_rad_s": (
                max_controller_spline_velocity_error
                if controller_spline_velocity_errors
                else None
            ),
            "controller_desired_matches_validated_spline": (
                controller_desired_matches_validated_spline
            ),
            "controller_timestamps_strictly_increasing": controller_timestamps_strictly_increasing,
            "controller_receipts_strictly_increasing": controller_receipts_strictly_increasing,
            "max_desired_actual_joint_error_rad": (
                max_tracking_error if tracking_errors else None
            ),
            "max_desired_actual_joint_velocity_error_rad_s": (
                max_velocity_tracking_error if velocity_tracking_errors else None
            ),
            "max_controller_sample_gap_s": max(controller_time_gaps or [0.0]),
            "max_controller_receipt_gap_s": max(
                controller_receipt_time_gaps or [0.0]
            ),
            "max_dynamic_controller_sample_gap_s": (
                dynamic_controller_max_gap_s
                if math.isfinite(dynamic_controller_max_gap_s)
                else None
            ),
            "raw_state_sample_count": len(raw_samples),
            "raw_state_span_s": raw_state_span_s,
            "raw_state_covers_dynamic_controller_window": raw_covers_dynamic_window,
            "raw_state_timestamps_strictly_increasing": raw_timestamps_strictly_increasing,
            "max_raw_state_sample_gap_s": max(raw_time_gaps or [math.inf]),
            "public_state_sample_count": len(public_samples),
            "public_state_span_s": public_state_span_s,
            "public_state_covers_dynamic_controller_window": public_covers_dynamic_window,
            "public_state_timestamps_strictly_increasing": public_timestamps_strictly_increasing,
            "max_public_state_sample_gap_s": max(public_time_gaps or [math.inf]),
            "paired_state_sample_count": len(paired_errors),
            "paired_velocity_sample_count": len(paired_velocity_errors),
            "public_state_pair_coverage": pair_coverage,
            "max_public_raw_joint_error_rad": (
                max_sync_error if paired_errors else None
            ),
            "max_public_raw_joint_velocity_error_rad_s": (
                max_velocity_sync_error if paired_velocity_errors else None
            ),
            "max_public_raw_pair_time_skew_s": (
                max_pair_skew if paired_skews else None
            ),
            "min_public_minus_raw_signed_time_skew_s": (
                min_signed_pair_skew if paired_skews else None
            ),
            "max_public_minus_raw_signed_time_skew_s": (
                max_signed_pair_skew if paired_skews else None
            ),
            "pass": bool(
                tracking_errors
                and velocity_tracking_errors
                and len(velocity_tracking_errors) == len(tracking_errors)
                and paired_errors
                and paired_velocity_errors
                and len(paired_velocity_errors) == len(paired_errors)
                and dynamic_controller_sample_count >= 5
                and controller_start_matches_plan
                and controller_final_matches_plan
                and controller_desired_matches_validated_spline
                and controller_timestamps_strictly_increasing
                and controller_receipts_strictly_increasing
                and raw_timestamps_strictly_increasing
                and public_timestamps_strictly_increasing
                and dynamic_controller_trace_span_s
                >= max(0.0, expected_duration_s * 0.90)
                and raw_state_span_s >= max(0.0, expected_duration_s * 0.90)
                and public_state_span_s >= max(0.0, expected_duration_s * 0.90)
                and raw_covers_dynamic_window
                and public_covers_dynamic_window
                and max_tracking_error
                <= THRESHOLDS["trajectory_tracking_max_abs_error_rad"]
                and max_velocity_tracking_error
                <= THRESHOLDS[
                    "trajectory_velocity_tracking_max_abs_error_rad_s"
                ]
                and dynamic_controller_max_gap_s
                <= THRESHOLDS["max_controller_state_gap_s"]
                and max(raw_time_gaps or [math.inf])
                <= THRESHOLDS["max_raw_state_gap_s"]
                and max(public_time_gaps or [math.inf])
                <= THRESHOLDS["max_public_state_gap_s"]
                and pair_coverage >= 0.90
                and max_sync_error <= THRESHOLDS["public_raw_joint_sync_rad"]
                and max_velocity_sync_error
                <= THRESHOLDS["public_raw_joint_velocity_sync_rad_s"]
                and max_pair_skew <= THRESHOLDS["max_state_pair_time_skew_s"]
            ),
        }

    @staticmethod
    def _evaluate_polynomial(
        coefficients: np.ndarray, time_s: float, derivative: int = 0
    ) -> np.ndarray:
        """Evaluate an ascending-power vector polynomial and its derivatives."""
        result = np.zeros(coefficients.shape[1], dtype=float)
        for power in range(derivative, coefficients.shape[0]):
            factor = math.factorial(power) / math.factorial(power - derivative)
            result += (
                coefficients[power]
                * factor
                * (time_s ** (power - derivative))
            )
        return result

    @staticmethod
    def _segment_coefficients(
        q0: np.ndarray,
        q1: np.ndarray,
        v0: np.ndarray | None,
        v1: np.ndarray | None,
        a0: np.ndarray | None,
        a1: np.ndarray | None,
        duration_s: float,
    ) -> tuple[str, np.ndarray]:
        """Match joint_trajectory_controller spline interpolation semantics."""
        t = duration_s
        delta = q1 - q0
        if v0 is None or v1 is None:
            coefficients = np.zeros((2, 6), dtype=float)
            coefficients[0] = q0
            coefficients[1] = delta / t
            return "linear_position", coefficients
        if a0 is None or a1 is None:
            coefficients = np.zeros((4, 6), dtype=float)
            coefficients[0] = q0
            coefficients[1] = v0
            coefficients[2] = 3.0 * delta / t**2 - (2.0 * v0 + v1) / t
            coefficients[3] = -2.0 * delta / t**3 + (v0 + v1) / t**2
            return "cubic_position_velocity", coefficients
        coefficients = np.zeros((6, 6), dtype=float)
        coefficients[0] = q0
        coefficients[1] = v0
        coefficients[2] = 0.5 * a0
        coefficients[3] = (
            20.0 * delta
            - (8.0 * v1 + 12.0 * v0) * t
            - (3.0 * a0 - a1) * t**2
        ) / (2.0 * t**3)
        coefficients[4] = (
            -30.0 * delta
            + (14.0 * v1 + 16.0 * v0) * t
            + (3.0 * a0 - 2.0 * a1) * t**2
        ) / (2.0 * t**4)
        coefficients[5] = (
            12.0 * delta
            - (6.0 * v1 + 6.0 * v0) * t
            - (a0 - a1) * t**2
        ) / (2.0 * t**5)
        return "quintic_position_velocity_acceleration", coefficients

    def evaluate_jtc_trajectory_state(
        self, trajectory, time_s: float
    ) -> dict[str, Any]:
        """Evaluate the exact validated JTC spline at ``time_s``.

        The controller state exposes ``desired.time_from_start``.  Evaluating
        the submitted trajectory at that same source-time proves that the
        controller's intermediate desired path, not only its endpoints, is the
        one that passed the pre-execution collision and limit gates.
        """
        if trajectory is None or len(trajectory.points) < 2:
            raise ValueError("trajectory requires at least two points")
        if set(trajectory.joint_names) != set(JOINTS):
            raise ValueError("trajectory joint names do not match J1..J6")
        if not math.isfinite(time_s):
            raise ValueError("trajectory evaluation time is not finite")

        order = [trajectory.joint_names.index(name) for name in JOINTS]
        points = trajectory.points
        times = [duration_seconds(point.time_from_start) for point in points]
        if time_s < times[0] - 1.0e-9 or time_s > times[-1] + 1.0e-9:
            raise ValueError(
                f"trajectory evaluation time {time_s} outside [{times[0]}, {times[-1]}]"
            )
        clamped_time = min(max(time_s, times[0]), times[-1])
        segment_index = len(points) - 2
        for index in range(len(points) - 1):
            if clamped_time <= times[index + 1] + 1.0e-12:
                segment_index = index
                break

        def field(point, name: str) -> np.ndarray | None:
            values = getattr(point, name)
            if len(values) != len(trajectory.joint_names):
                return None
            return np.asarray([values[index] for index in order], dtype=float)

        q0 = field(points[segment_index], "positions")
        q1 = field(points[segment_index + 1], "positions")
        if q0 is None or q1 is None:
            raise ValueError("trajectory positions are incomplete")
        v0 = field(points[segment_index], "velocities")
        v1 = field(points[segment_index + 1], "velocities")
        a0 = field(points[segment_index], "accelerations")
        a1 = field(points[segment_index + 1], "accelerations")
        duration = times[segment_index + 1] - times[segment_index]
        interpolation, coefficients = self._segment_coefficients(
            q0, q1, v0, v1, a0, a1, duration
        )
        local_time = clamped_time - times[segment_index]
        return {
            "time_s": clamped_time,
            "position_rad": self._evaluate_polynomial(
                coefficients, local_time, derivative=0
            ),
            "velocity_rad_s": self._evaluate_polynomial(
                coefficients, local_time, derivative=1
            ),
            "acceleration_rad_s2": self._evaluate_polynomial(
                coefficients, local_time, derivative=2
            ),
            "interpolation": interpolation,
        }

    @staticmethod
    def _real_roots_in_interval(
        ascending_coefficients: np.ndarray, duration_s: float
    ) -> list[float]:
        coefficients = np.trim_zeros(ascending_coefficients[::-1], trim="f")
        if len(coefficients) <= 1:
            return []
        roots = np.roots(coefficients)
        return [
            float(root.real)
            for root in roots
            if abs(float(root.imag)) <= 1.0e-10
            and 0.0 < float(root.real) < duration_s
        ]

    def sample_trajectory(self, trajectory) -> list[dict[str, Any]]:
        """Sample the exact JTC spline, including derivative extrema.

        Regular sample spacing is selected from the exact segment maximum
        velocity so every consecutive position step is no greater than the
        accepted 0.25 degree swept-collision increment.  Roots of acceleration
        and jerk are added so internal velocity/acceleration extrema are not
        hidden between waypoints.
        """
        if trajectory is None or not trajectory.points:
            return []
        order = [trajectory.joint_names.index(name) for name in JOINTS]
        points = trajectory.points
        times = [duration_seconds(point.time_from_start) for point in points]
        positions = [
            np.asarray([point.positions[index] for index in order], dtype=float)
            for point in points
        ]
        velocity_complete = all(
            len(point.velocities) == len(trajectory.joint_names) for point in points
        )
        acceleration_complete = all(
            len(point.accelerations) == len(trajectory.joint_names)
            for point in points
        )
        velocities = (
            [
                np.asarray(
                    [point.velocities[index] for index in order], dtype=float
                )
                for point in points
            ]
            if velocity_complete
            else [None] * len(points)
        )
        accelerations = (
            [
                np.asarray(
                    [point.accelerations[index] for index in order], dtype=float
                )
                for point in points
            ]
            if acceleration_complete
            else [None] * len(points)
        )
        first_velocity = (
            velocities[0] if velocities[0] is not None else np.zeros(6, dtype=float)
        )
        first_acceleration = (
            accelerations[0]
            if accelerations[0] is not None
            else np.zeros(6, dtype=float)
        )
        samples: list[dict[str, Any]] = [
            {
                "time_s": times[0],
                "position_rad": positions[0],
                "velocity_rad_s": first_velocity,
                "acceleration_rad_s2": first_acceleration,
                "interpolation": "waypoint",
            }
        ]
        for segment_index in range(len(points) - 1):
            duration = times[segment_index + 1] - times[segment_index]
            if not math.isfinite(duration) or duration <= 0.0:
                return []
            interpolation, coefficients = self._segment_coefficients(
                positions[segment_index],
                positions[segment_index + 1],
                velocities[segment_index],
                velocities[segment_index + 1],
                accelerations[segment_index],
                accelerations[segment_index + 1],
                duration,
            )
            critical_times = {0.0, duration}
            for joint_index in range(6):
                velocity_coefficients = np.asarray(
                    [
                        power * coefficients[power, joint_index]
                        for power in range(1, coefficients.shape[0])
                    ],
                    dtype=float,
                )
                acceleration_coefficients = np.asarray(
                    [
                        power * (power - 1) * coefficients[power, joint_index]
                        for power in range(2, coefficients.shape[0])
                    ],
                    dtype=float,
                )
                jerk_coefficients = np.asarray(
                    [
                        power
                        * (power - 1)
                        * (power - 2)
                        * coefficients[power, joint_index]
                        for power in range(3, coefficients.shape[0])
                    ],
                    dtype=float,
                )
                critical_times.update(
                    self._real_roots_in_interval(velocity_coefficients, duration)
                )
                critical_times.update(
                    self._real_roots_in_interval(
                        acceleration_coefficients, duration
                    )
                )
                critical_times.update(
                    self._real_roots_in_interval(jerk_coefficients, duration)
                )
            max_segment_velocity = max(
                float(
                    np.max(
                        np.abs(
                            self._evaluate_polynomial(
                                coefficients, sample_time, derivative=1
                            )
                        )
                    )
                )
                for sample_time in critical_times
            )
            regular_step_count = max(
                1,
                int(
                    math.ceil(
                        duration * max_segment_velocity / MAX_COLLISION_SAMPLE_STEP
                    )
                ),
            )
            sample_times = set(critical_times)
            sample_times.update(
                duration * step / regular_step_count
                for step in range(1, regular_step_count + 1)
            )
            for local_time in sorted(sample_times):
                if local_time <= 1.0e-12:
                    continue
                samples.append(
                    {
                        "time_s": times[segment_index] + local_time,
                        "position_rad": self._evaluate_polynomial(
                            coefficients, local_time, derivative=0
                        ),
                        "velocity_rad_s": self._evaluate_polynomial(
                            coefficients, local_time, derivative=1
                        ),
                        "acceleration_rad_s2": self._evaluate_polynomial(
                            coefficients, local_time, derivative=2
                        ),
                        "interpolation": interpolation,
                    }
                )
        return samples

    def validate_trajectory(self, trajectory) -> dict[str, Any]:
        if trajectory is None or not trajectory.points:
            return {
                "point_count": 0,
                "path_sample_count": 0,
                "evaluated_sample_count": 0,
                "pass": False,
                "reason": "no_trajectory",
            }
        joint_names_valid = bool(
            len(trajectory.joint_names) == 6
            and set(trajectory.joint_names) == set(JOINTS)
        )
        positions_complete = bool(
            joint_names_valid
            and all(
                len(point.positions) == len(trajectory.joint_names)
                for point in trajectory.points
            )
        )
        if not positions_complete:
            return {
                "point_count": len(trajectory.points),
                "path_sample_count": 0,
                "evaluated_sample_count": 0,
                "joint_names_valid": joint_names_valid,
                "positions_complete": positions_complete,
                "pass": False,
                "reason": "invalid_joint_names_or_position_fields",
            }
        times = [duration_seconds(point.time_from_start) for point in trajectory.points]
        strict_time = all(second > first for first, second in zip(times, times[1:]))
        starts_at_zero = bool(times and abs(times[0]) <= 1.0e-12)
        enough_points = len(trajectory.points) >= 2
        order = [trajectory.joint_names.index(name) for name in JOINTS]
        position_rows = np.array(
            [
                [point.positions[index] for index in order]
                for point in trajectory.points
            ],
            dtype=float,
        )
        velocity_fields_complete = all(
            len(point.velocities) == len(trajectory.joint_names)
            for point in trajectory.points
        )
        acceleration_fields_complete = all(
            len(point.accelerations) == len(trajectory.joint_names)
            for point in trajectory.points
        )
        kinematics_source = "jtc_spline_reconstruction"
        if velocity_fields_complete:
            velocity_rows = np.array(
                [
                    [point.velocities[index] for index in order]
                    for point in trajectory.points
                ],
                dtype=float,
            )
        elif strict_time and len(times) >= 2:
            kinematics_source = "finite_difference"
            velocity_rows = np.diff(position_rows, axis=0) / np.diff(times)[:, None]
        else:
            velocity_rows = np.empty((0, 6), dtype=float)

        if acceleration_fields_complete:
            acceleration_rows = np.array(
                [
                    [point.accelerations[index] for index in order]
                    for point in trajectory.points
                ],
                dtype=float,
            )
        elif kinematics_source == "finite_difference" and len(velocity_rows) >= 2:
            segment_dt = np.diff(times)
            midpoint_dt = 0.5 * (segment_dt[1:] + segment_dt[:-1])
            acceleration_rows = np.diff(velocity_rows, axis=0) / midpoint_dt[:, None]
        else:
            acceleration_rows = np.empty((0, 6), dtype=float)

        kinematics_available = bool(
            velocity_fields_complete and acceleration_fields_complete
        )
        finite = bool(
            np.all(np.isfinite(times))
            and np.all(np.isfinite(position_rows))
            and kinematics_available
            and np.all(np.isfinite(velocity_rows))
            and np.all(np.isfinite(acceleration_rows))
        )
        samples = (
            self.sample_trajectory(trajectory)
            if finite and strict_time and starts_at_zero and enough_points
            else []
        )
        max_sample_joint_step = max(
            [
                float(
                    np.max(
                        np.abs(
                            np.asarray(second["position_rad"], dtype=float)
                            - np.asarray(first["position_rad"], dtype=float)
                        )
                    )
                )
                for first, second in zip(samples, samples[1:])
            ]
            or [math.inf]
        )
        spline_velocity_rows = np.asarray(
            [row["velocity_rad_s"] for row in samples], dtype=float
        ) if samples else np.empty((0, 6), dtype=float)
        spline_acceleration_rows = np.asarray(
            [row["acceleration_rad_s2"] for row in samples], dtype=float
        ) if samples else np.empty((0, 6), dtype=float)
        max_velocity = (
            float(np.max(np.abs(spline_velocity_rows)))
            if spline_velocity_rows.size
            else math.inf
        )
        max_acceleration = (
            float(np.max(np.abs(spline_acceleration_rows)))
            if spline_acceleration_rows.size
            else math.inf
        )
        endpoint_velocity_max = (
            float(
                max(
                    np.max(np.abs(spline_velocity_rows[0])),
                    np.max(np.abs(spline_velocity_rows[-1])),
                )
            )
            if spline_velocity_rows.size
            else math.inf
        )
        moveit_collision_count = 0
        mujoco_collision_count = 0
        bounds_violation_count = 0
        cross_engine_mismatch_count = 0
        evaluated_sample_count = 0
        for sample_row in samples:
            sample = np.asarray(sample_row["position_rad"], dtype=float)
            evaluated_sample_count += 1
            bounds_pass, _ = self.within_bounds(sample)
            moveit_result = self.moveit_validity(sample)
            guard_result = self.guard.check_pose_deg(np.degrees(sample))
            moveit_safe = bool(moveit_result["valid"] and bounds_pass)
            mujoco_safe = bool(guard_result["safe"] and bounds_pass)
            moveit_collision_count += int(not moveit_result["valid"])
            mujoco_collision_count += int(not guard_result["safe"])
            bounds_violation_count += int(not bounds_pass)
            cross_engine_mismatch_count += int(moveit_safe != mujoco_safe)
            if (
                moveit_collision_count
                or mujoco_collision_count
                or bounds_violation_count
                or cross_engine_mismatch_count
            ):
                break
        return {
            "point_count": len(trajectory.points),
            "duration_s": times[-1],
            "joint_names_valid": joint_names_valid,
            "positions_complete": positions_complete,
            "velocity_fields_complete": velocity_fields_complete,
            "acceleration_fields_complete": acceleration_fields_complete,
            "kinematics_source": kinematics_source,
            "kinematics_available": kinematics_available,
            "time_strictly_increasing": strict_time,
            "starts_at_zero": starts_at_zero,
            "enough_points": enough_points,
            "finite": finite,
            "interpolation_modes": sorted(
                {row["interpolation"] for row in samples}
            ),
            "max_velocity_rad_s": (
                max_velocity if spline_velocity_rows.size else None
            ),
            "max_acceleration_rad_s2": (
                max_acceleration if spline_acceleration_rows.size else None
            ),
            "endpoint_velocity_max_abs_rad_s": (
                endpoint_velocity_max if spline_velocity_rows.size else None
            ),
            "endpoint_velocity_zero_pass": bool(
                endpoint_velocity_max <= 1.0e-6
            ),
            "velocity_limits_pass": bool(
                spline_velocity_rows.size
                and max_velocity <= MAX_VELOCITY + 1.0e-9
            ),
            "acceleration_limits_pass": bool(
                spline_acceleration_rows.size
                and max_acceleration <= MAX_ACCELERATION + 1.0e-9
            ),
            "path_sample_count": len(samples),
            "max_sample_joint_step_rad": max_sample_joint_step,
            "sample_step_limit_pass": bool(
                max_sample_joint_step
                <= THRESHOLDS["path_sample_max_joint_step_rad"] + 1.0e-12
            ),
            "evaluated_sample_count": evaluated_sample_count,
            "moveit_path_collision_count": moveit_collision_count,
            "mujoco_path_collision_count": mujoco_collision_count,
            "bounds_violation_count": bounds_violation_count,
            "cross_engine_mismatch_count": cross_engine_mismatch_count,
            "pass": bool(
                strict_time
                and starts_at_zero
                and enough_points
                and finite
                and joint_names_valid
                and positions_complete
                and kinematics_available
                and max_velocity <= MAX_VELOCITY + 1.0e-9
                and max_acceleration <= MAX_ACCELERATION + 1.0e-9
                and endpoint_velocity_max <= 1.0e-6
                and max_sample_joint_step
                <= THRESHOLDS["path_sample_max_joint_step_rad"] + 1.0e-12
                and moveit_collision_count == 0
                and mujoco_collision_count == 0
                and bounds_violation_count == 0
                and cross_engine_mismatch_count == 0
            ),
        }

    def current_tf_pose(self) -> dict[str, Any]:
        transform = self.tf_buffer.lookup_transform(
            "world", "tcp_nominal", Time(), timeout=Duration(seconds=3.0)
        ).transform
        return {
            "position_m": [
                transform.translation.x,
                transform.translation.y,
                transform.translation.z,
            ],
            "quaternion_xyzw": [
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w,
            ],
        }

    def save_trace(self, target_id: str, trace: list[dict[str, Any]]) -> dict[str, Any]:
        path = self.trace_dir / f"{target_id}.jsonl"
        path.write_text(
            "".join(json.dumps(row, separators=(",", ":")) + "\n" for row in trace),
            encoding="utf-8",
        )
        return {"path": str(path), "sha256": sha256(path), "sample_count": len(trace)}

    def run_target(self, target: dict[str, Any]) -> dict[str, Any]:
        target_q = np.array(target["joint_position_rad"], dtype=float)
        target_pose = self.fk_pose(target_q)
        within_bounds, violations = self.within_bounds(target_q)
        moveit_goal = self.moveit_validity(target_q)
        mujoco_goal = self.guard.check_pose_deg(np.degrees(target_q))
        moveit_self_contacts = [
            row
            for row in moveit_goal["contacts"]
            if "v15_14_mujoco_ground" not in (row["body1"], row["body2"])
        ]
        moveit_ground_contacts = [
            row
            for row in moveit_goal["contacts"]
            if "v15_14_mujoco_ground" in (row["body1"], row["body2"])
        ]
        mujoco_self_contacts = [
            row
            for row in mujoco_goal["contacts"]
            if row.get("contact_class") == "self_collision"
        ]
        mujoco_ground_contacts = [
            row
            for row in mujoco_goal["contacts"]
            if row.get("contact_class") == "ground"
        ]
        expected = target["expected_outcome"]
        ik = (
            self.compute_ik(target_pose, target_q)
            if expected == "execute_success"
            else {
                "error_code": None,
                "kinematic_solution_found": None,
                "collision_aware_pass": None,
                "solution_rad": [],
            }
        )
        state_before_request = state_values(self.raw_state)
        if state_before_request is None:
            raise RuntimeError("MuJoCo state unavailable before MoveGroup request")
        state_before_request = state_before_request.copy()
        bridge_counters_before_request = {
            "accepted_moving_command_count": self.bridge_status.get(
                "accepted_moving_command_count"
            ),
            "rejected_command_count": self.bridge_status.get(
                "rejected_command_count"
            ),
        }
        move_result = self.send_move_group(
            target, target_pose, plan_only=True
        )
        planning_trace_info = self.save_trace(
            f"{target['id']}__planning", move_result["trace"]
        )

        if expected != "execute_success":
            trace_info = planning_trace_info
            negative_state_rows = [
                np.asarray(row["position_rad"], dtype=float)
                for row in move_result["raw_samples"]
            ]
            negative_velocity_rows = [
                np.asarray(row.get("velocity_rad_s", []), dtype=float)
                for row in move_result["raw_samples"]
                if len(row.get("velocity_rad_s", [])) == 6
            ]
            state_after_request = state_values(self.raw_state)
            if state_after_request is not None:
                negative_state_rows.append(state_after_request)
            max_negative_excursion = max(
                [
                    float(np.max(np.abs(row - state_before_request)))
                    for row in negative_state_rows
                ]
                or [math.inf]
            )
            max_negative_velocity = max(
                [float(np.max(np.abs(row))) for row in negative_velocity_rows]
                or [math.inf]
            )
            bridge_counters_after_request = {
                "accepted_moving_command_count": self.bridge_status.get(
                    "accepted_moving_command_count"
                ),
                "rejected_command_count": self.bridge_status.get(
                    "rejected_command_count"
                ),
            }
            no_moving_command_accepted = bool(
                bridge_counters_before_request["accepted_moving_command_count"]
                is not None
                and bridge_counters_after_request[
                    "accepted_moving_command_count"
                ]
                == bridge_counters_before_request[
                    "accepted_moving_command_count"
                ]
            )
            no_command_rejected = bool(
                bridge_counters_before_request["rejected_command_count"]
                is not None
                and bridge_counters_after_request["rejected_command_count"]
                == bridge_counters_before_request["rejected_command_count"]
            )
            no_execution_verified = bool(
                move_result["raw_samples"]
                and move_result["accepted"]
                and move_result["status"]
                in (GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_ABORTED)
                and max_negative_excursion
                <= THRESHOLDS["negative_no_execution_joint_excursion_rad"]
                and max_negative_velocity
                <= THRESHOLDS["negative_no_execution_joint_excursion_rad"]
                and no_moving_command_accepted
                and no_command_rejected
                and self.bridge_runtime_contract()["pass"]
            )
            trajectory = move_result["trajectory"]
            planned_final: list[float] = []
            planned_final_within_bounds = None
            planned_final_matches_requested = None
            planned_final_matches_frozen_clamp = None
            negative_trajectory_validation = None
            if trajectory is not None and trajectory.points:
                order = [trajectory.joint_names.index(name) for name in JOINTS]
                planned_final_array = np.array(
                    [
                        trajectory.points[-1].positions[index]
                        for index in order
                    ],
                    dtype=float,
                )
                planned_final = planned_final_array.tolist()
                planned_final_within_bounds, _ = self.within_bounds(
                    planned_final_array
                )
                planned_final_matches_requested = bool(
                    np.max(
                        np.abs(
                            wrapped_joint_error(planned_final_array, target_q)
                        )
                    )
                    <= 1.0e-6
                )
                expected_clamped = target_q.copy()
                for joint_index, joint_name in enumerate(JOINTS):
                    if joint_name in POSITION_BOUNDS:
                        lower, upper = POSITION_BOUNDS[joint_name]
                        expected_clamped[joint_index] = np.clip(
                            expected_clamped[joint_index], lower, upper
                        )
                planned_final_matches_frozen_clamp = bool(
                    np.max(
                        np.abs(
                            wrapped_joint_error(
                                planned_final_array, expected_clamped
                            )
                        )
                    )
                    <= 1.0e-6
                )
                if expected == "bounds_rejection":
                    negative_trajectory_validation = self.validate_trajectory(
                        trajectory
                    )

            allowed_planner_rejection_codes = {
                99999,  # MoveIt FAILURE after an accepted, completed request
                -1,     # PLANNING_FAILED
                -2,     # INVALID_MOTION_PLAN
                -10,    # START_STATE_IN_COLLISION
                -11,    # START_STATE_VIOLATES_PATH_CONSTRAINTS
                -12,    # GOAL_IN_COLLISION
                -13,    # GOAL_VIOLATES_PATH_CONSTRAINTS
                -31,    # NO_IK_SOLUTION
            }
            expected_rejection = bool(
                move_result["error_code"] in allowed_planner_rejection_codes
                and no_execution_verified
            )
            rejection_mode = "planner_rejected"
            if expected == "bounds_rejection":
                safe_clamp = bool(
                    move_result["error_code"] == 1
                    and planned_final_within_bounds
                    and planned_final_matches_requested is False
                    and planned_final_matches_frozen_clamp is True
                    and negative_trajectory_validation is not None
                    and negative_trajectory_validation["pass"]
                )
                expected_rejection = bool(
                    not within_bounds
                    and (
                        move_result["error_code"]
                        in allowed_planner_rejection_codes
                        or safe_clamp
                    )
                    and no_execution_verified
                )
                rejection_mode = (
                    "moveit_clamped_to_frozen_limit_no_execution"
                    if safe_clamp
                    else "planner_rejected"
                )
            if expected == "collision_rejection":
                # This target is specifically the self-collision negative
                # control.  A ground-only rejection must not masquerade as a
                # successful self-collision test merely because both engines
                # report an invalid state.
                expected_rejection = bool(
                    expected_rejection
                    and not moveit_goal["valid"]
                    and bool(moveit_self_contacts)
                    and not mujoco_goal["safe"]
                    and mujoco_goal["reason"] == "self_collision"
                    and bool(mujoco_self_contacts)
                    and no_execution_verified
                )
            return {
                "id": target["id"],
                "category": target["category"],
                "planning_mode": target["planning_mode"],
                "expected_outcome": expected,
                "tcp_target_pose": target_pose,
                "ik": ik,
                "planning": {
                    "accepted": move_result["accepted"],
                    "action_status": move_result["status"],
                    "moveit_error_code": move_result["error_code"],
                    "planning_time_s": move_result["planning_time_s"],
                    "rejection_mode": rejection_mode,
                    "planned_final_joint_position_rad": planned_final,
                    "planned_final_within_bounds": planned_final_within_bounds,
                    "planned_final_matches_requested": planned_final_matches_requested,
                    "planned_final_matches_frozen_clamp": planned_final_matches_frozen_clamp,
                    "trajectory_validation": negative_trajectory_validation,
                    "pass": expected_rejection,
                },
                "collision": {
                    "moveit_goal_collision": not moveit_goal["valid"],
                    "mujoco_goal_collision": not mujoco_goal["safe"],
                    "moveit_self_collision": bool(moveit_self_contacts),
                    "moveit_ground_collision": bool(moveit_ground_contacts),
                    "moveit_self_contacts": moveit_self_contacts,
                    "moveit_ground_contacts": moveit_ground_contacts,
                    "mujoco_reason": mujoco_goal["reason"],
                    "mujoco_self_collision": bool(mujoco_self_contacts),
                    "mujoco_ground_collision": bool(mujoco_ground_contacts),
                    "mujoco_self_contacts": mujoco_self_contacts,
                    "mujoco_ground_contacts": mujoco_ground_contacts,
                    "cross_engine_boolean_match": (
                        moveit_goal["valid"] == mujoco_goal["safe"]
                        if expected == "collision_rejection"
                        else None
                    ),
                },
                "joint_limits": {
                    "goal_within_bounds": within_bounds,
                    "violations": violations,
                },
                "controller": {
                    "action_name": "/arm_controller/follow_joint_trajectory",
                    "move_group_goal_uuid": move_result["goal_uuid"],
                    "execution_requested": False,
                },
                "execution": {
                    "trace": trace_info,
                    "max_joint_excursion_during_plan_only_rad": max_negative_excursion,
                    "max_joint_velocity_during_plan_only_rad_s": max_negative_velocity,
                    "bridge_counters_before_request": bridge_counters_before_request,
                    "bridge_counters_after_request": bridge_counters_after_request,
                    "no_moving_command_accepted": no_moving_command_accepted,
                    "no_command_rejected": no_command_rejected,
                    "no_execution_verified": no_execution_verified,
                },
                "verdict": "EXPECTED_REJECTION_PASS"
                if expected_rejection
                else "FAIL",
            }

        if (
            not move_result["accepted"]
            or move_result["error_code"] != 1
            or move_result["trajectory"] is None
            or not move_result["trajectory"].points
        ):
            return {
                "id": target["id"],
                "category": target["category"],
                "planning_mode": target["planning_mode"],
                "expected_outcome": expected,
                "tcp_target_pose": target_pose,
                "ik": ik,
                "planning": {
                    "accepted": move_result["accepted"],
                    "action_status": move_result["status"],
                    "moveit_error_code": move_result["error_code"],
                    "planner_id": "RRTConnectkConfigDefault",
                    "planning_time_s": move_result["planning_time_s"],
                    "pass": False,
                },
                "collision": {
                    "moveit_goal_collision": not moveit_goal["valid"],
                    "mujoco_goal_collision": not mujoco_goal["safe"],
                    "path_sample_count": 0,
                    "cross_engine_boolean_match": moveit_goal["valid"]
                    == mujoco_goal["safe"],
                },
                "joint_limits": {
                    "goal_within_bounds": within_bounds,
                    "violations": violations,
                    "trajectory_within_bounds": None,
                },
                "trajectory": {
                    "point_count": 0,
                    "pass": False,
                    "reason": "planning_or_trajectory_unavailable",
                },
                "controller": {
                    "action_name": "/arm_controller/follow_joint_trajectory",
                    "move_group_goal_uuid": move_result["goal_uuid"],
                    "accepted": move_result["accepted"],
                    "result_code": move_result["error_code"],
                },
                "execution": {
                    "max_abs_final_joint_error_rad": None,
                    "trace": planning_trace_info,
                    "trace_metrics": {
                        "pass": False,
                        "reason": "trajectory_not_executed",
                    },
                },
                "tcp_result": {
                    "position_error_m": None,
                    "orientation_error_rad": None,
                },
                "sync": {"max_joint_position_error_rad": None},
                "verdict": "FAIL",
            }

        trajectory_validation = self.validate_trajectory(move_result["trajectory"])
        trajectory = move_result["trajectory"]
        robot_trajectory = move_result["robot_trajectory"]
        multi_dof_empty = bool(
            robot_trajectory is not None
            and not robot_trajectory.multi_dof_joint_trajectory.joint_names
            and not robot_trajectory.multi_dof_joint_trajectory.points
        )
        planned_trajectory_cdr_sha256 = (
            hashlib.sha256(serialize_message(robot_trajectory)).hexdigest()
            if robot_trajectory is not None
            else None
        )
        planned_trajectory_semantic_sha256 = (
            semantic_message_sha256(robot_trajectory)
            if robot_trajectory is not None
            else None
        )
        order = [trajectory.joint_names.index(name) for name in JOINTS]
        planned_first = np.asarray(
            [trajectory.points[0].positions[index] for index in order], dtype=float
        )
        live_raw_before_execute = state_values(self.raw_state)
        live_public_before_execute = state_values(self.public_state)
        start_raw_error = (
            float(
                np.max(
                    np.abs(
                        wrapped_joint_error(
                            planned_first, live_raw_before_execute
                        )
                    )
                )
            )
            if live_raw_before_execute is not None
            else math.inf
        )
        start_public_error = (
            float(
                np.max(
                    np.abs(
                        wrapped_joint_error(
                            planned_first, live_public_before_execute
                        )
                    )
                )
            )
            if live_public_before_execute is not None
            else math.inf
        )
        before_execute_runtime_contract = self.bridge_runtime_contract()
        pre_execution_gate_pass = bool(
            move_result["accepted"]
            and move_result["status"] == GoalStatus.STATUS_SUCCEEDED
            and move_result["error_code"] == 1
            and trajectory_validation["pass"]
            and multi_dof_empty
            and start_raw_error <= 1.0e-6
            and start_public_error <= 1.0e-6
            and before_execute_runtime_contract["pass"]
        )
        trajectory_validation["multi_dof_trajectory_empty"] = multi_dof_empty
        trajectory_validation["planned_trajectory_cdr_sha256"] = (
            planned_trajectory_cdr_sha256
        )
        trajectory_validation["planned_trajectory_semantic_sha256"] = (
            planned_trajectory_semantic_sha256
        )
        trajectory_validation["planned_first_joint_position_rad"] = (
            planned_first.tolist()
        )
        trajectory_validation["live_raw_start_error_rad"] = start_raw_error
        trajectory_validation["live_public_start_error_rad"] = start_public_error
        trajectory_validation["pre_execution_gate_pass"] = pre_execution_gate_pass
        if not pre_execution_gate_pass:
            return {
                "id": target["id"],
                "category": target["category"],
                "planning_mode": target["planning_mode"],
                "expected_outcome": expected,
                "tcp_target_pose": target_pose,
                "ik": ik,
                "planning": {
                    "accepted": move_result["accepted"],
                    "action_status": move_result["status"],
                    "moveit_error_code": move_result["error_code"],
                    "planner_id": "RRTConnectkConfigDefault",
                    "planning_time_s": move_result["planning_time_s"],
                    "pass": True,
                },
                "collision": {
                    "moveit_goal_collision": not moveit_goal["valid"],
                    "mujoco_goal_collision": not mujoco_goal["safe"],
                    "path_sample_count": trajectory_validation[
                        "path_sample_count"
                    ],
                    "cross_engine_boolean_match": (
                        trajectory_validation["cross_engine_mismatch_count"] == 0
                    ),
                },
                "joint_limits": {
                    "goal_within_bounds": within_bounds,
                    "violations": violations,
                    "trajectory_within_bounds": (
                        trajectory_validation["bounds_violation_count"] == 0
                    ),
                },
                "trajectory": trajectory_validation,
                "controller": {
                    "action_name": "/arm_controller/follow_joint_trajectory",
                    "moveit_execute_action": "/execute_trajectory",
                    "execution_requested": False,
                },
                "execution": {
                    "pre_execution_gate_pass": False,
                    "bridge_runtime_contract": before_execute_runtime_contract,
                    "max_abs_final_joint_error_rad": None,
                    "trace": planning_trace_info,
                    "trace_metrics": {
                        "pass": False,
                        "reason": "blocked_by_pre_execution_trajectory_validation",
                    },
                },
                "tcp_result": {
                    "position_error_m": None,
                    "orientation_error_rad": None,
                },
                "sync": {"max_joint_position_error_rad": None},
                "verdict": "FAIL",
            }

        execute_result = self.execute_validated_trajectory(
            target["id"], robot_trajectory
        )
        execute_request_payload_matches_validated = bool(
            execute_result["execute_request_semantic_sha256"]
            == planned_trajectory_semantic_sha256
        )
        trace_info = self.save_trace(target["id"], execute_result["trace"])
        trace_metrics = self.execution_trace_metrics(execute_result, trajectory)
        commanded_final = np.array(
            [trajectory.points[-1].positions[index] for index in order], dtype=float
        )
        self.spin_for(0.30)
        final_runtime_contract = self.bridge_runtime_contract()
        raw = state_values(self.raw_state)
        public = state_values(self.public_state)
        if raw is None or public is None or self.mujoco_tcp is None:
            raise RuntimeError("final state unavailable")
        joint_error = wrapped_joint_error(commanded_final, raw)
        actual_pose = {
            "frame": self.mujoco_tcp.header.frame_id,
            "position_m": [
                self.mujoco_tcp.pose.position.x,
                self.mujoco_tcp.pose.position.y,
                self.mujoco_tcp.pose.position.z,
            ],
            "quaternion_xyzw": [
                self.mujoco_tcp.pose.orientation.x,
                self.mujoco_tcp.pose.orientation.y,
                self.mujoco_tcp.pose.orientation.z,
                self.mujoco_tcp.pose.orientation.w,
            ],
        }
        tf_pose = self.current_tf_pose()
        tcp_position_error = float(
            np.linalg.norm(
                np.asarray(actual_pose["position_m"])
                - np.asarray(target_pose["position_m"])
            )
        )
        tcp_orientation_error = quaternion_error(
            actual_pose["quaternion_xyzw"], target_pose["quaternion_xyzw"]
        )
        tf_position_error = float(
            np.linalg.norm(
                np.asarray(tf_pose["position_m"])
                - np.asarray(actual_pose["position_m"])
            )
        )
        tf_orientation_error = quaternion_error(
            tf_pose["quaternion_xyzw"], actual_pose["quaternion_xyzw"]
        )
        sync_error = float(np.max(np.abs(public - raw)))
        final_joint_error = float(np.max(np.abs(joint_error)))
        execution_pass = bool(
            move_result["accepted"]
            and move_result["status"] == GoalStatus.STATUS_SUCCEEDED
            and move_result["error_code"] == 1
            and execute_result["accepted"]
            and execute_result["status"] == GoalStatus.STATUS_SUCCEEDED
            and execute_result["error_code"] == 1
            and execute_request_payload_matches_validated
            and final_joint_error <= THRESHOLDS["final_joint_max_abs_error_rad"]
            and tcp_position_error <= THRESHOLDS["final_tcp_position_error_m"]
            and tcp_orientation_error
            <= THRESHOLDS["final_tcp_orientation_error_rad"]
            and sync_error <= THRESHOLDS["final_public_raw_joint_sync_rad"]
            and tf_position_error <= THRESHOLDS["tf_mujoco_tcp_position_m"]
            and tf_orientation_error
            <= THRESHOLDS["tf_mujoco_tcp_orientation_rad"]
            and trajectory_validation["pass"]
            and trace_metrics["pass"]
            and final_runtime_contract["pass"]
            and ik["collision_aware_pass"]
            and moveit_goal["valid"]
            and mujoco_goal["safe"]
            and not bool(self.bridge_status.get("fault_latched", False))
        )
        return {
            "id": target["id"],
            "category": target["category"],
            "planning_mode": target["planning_mode"],
            "expected_outcome": expected,
            "start_joint_position_rad": execute_result["trace"][0]["actual_position_rad"]
            if execute_result["trace"]
            else [],
            "tcp_target_pose": target_pose,
            "ik": ik,
            "planning": {
                "accepted": move_result["accepted"],
                "action_status": move_result["status"],
                "moveit_error_code": move_result["error_code"],
                "planner_id": "RRTConnectkConfigDefault",
                "planning_time_s": move_result["planning_time_s"],
                "pass": move_result["error_code"] == 1,
            },
            "collision": {
                "moveit_goal_collision": not moveit_goal["valid"],
                "mujoco_goal_collision": not mujoco_goal["safe"],
                "path_sample_count": trajectory_validation["path_sample_count"],
                "moveit_path_collision_count": trajectory_validation[
                    "moveit_path_collision_count"
                ],
                "mujoco_path_collision_count": trajectory_validation[
                    "mujoco_path_collision_count"
                ],
                "cross_engine_boolean_match": trajectory_validation[
                    "cross_engine_mismatch_count"
                ]
                == 0,
            },
            "joint_limits": {
                "goal_within_bounds": within_bounds,
                "violations": violations,
                "trajectory_within_bounds": trajectory_validation[
                    "bounds_violation_count"
                ]
                == 0,
            },
            "trajectory": trajectory_validation,
            "controller": {
                "action_name": "/arm_controller/follow_joint_trajectory",
                "moveit_execute_action": "/execute_trajectory",
                "controller_type": "joint_trajectory_controller/JointTrajectoryController",
                "move_group_goal_uuid": move_result["goal_uuid"],
                "execute_goal_uuid": execute_result["goal_uuid"],
                "accepted": execute_result["accepted"],
                "result_code": execute_result["error_code"],
            },
            "execution": {
                "pre_execution_gate_pass": pre_execution_gate_pass,
                "validated_trajectory_cdr_sha256": planned_trajectory_cdr_sha256,
                "validated_trajectory_semantic_sha256": (
                    planned_trajectory_semantic_sha256
                ),
                "execute_request_trajectory_cdr_sha256": execute_result[
                    "execute_request_cdr_sha256"
                ],
                "execute_request_trajectory_semantic_sha256": execute_result[
                    "execute_request_semantic_sha256"
                ],
                "execute_request_payload_matches_validated": (
                    execute_request_payload_matches_validated
                ),
                "bridge_runtime_contract_before_execute": (
                    before_execute_runtime_contract
                ),
                "commanded_final_joint_position_rad": commanded_final.tolist(),
                "actual_final_joint_position_rad": raw.tolist(),
                "final_joint_error_rad": joint_error.tolist(),
                "max_abs_final_joint_error_rad": final_joint_error,
                "trace": trace_info,
                "trace_metrics": trace_metrics,
                "bridge_runtime_contract": final_runtime_contract,
            },
            "tcp_result": {
                "actual_pose": actual_pose,
                "position_error_m": tcp_position_error,
                "orientation_error_rad": tcp_orientation_error,
            },
            "sync": {
                "public_joint_state_rad": public.tolist(),
                "raw_mujoco_joint_state_rad": raw.tolist(),
                "max_joint_position_error_rad": sync_error,
                "tf_tcp_pose": tf_pose,
                "max_tf_tcp_position_error_m": tf_position_error,
                "max_tf_tcp_orientation_error_rad": tf_orientation_error,
            },
            "bridge_fault_latched": bool(
                self.bridge_status.get("fault_latched", False)
            ),
            "verdict": "PASS" if execution_pass else "FAIL",
        }


def render_markdown(report: dict[str, Any]) -> str:
    lines = [
        "# V15.14 MoveIt2 + ROS2 Control + MuJoCo 轨迹闭环验收",
        "",
        f"- 总状态：**{report['status']}**",
        "- 执行语义：`kinematic_position_tracking`（空载运动学；不宣称动力学有效）",
        "- 标准接口：`/arm_controller/follow_joint_trajectory`",
        "- 唯一公开状态源：`joint_state_broadcaster → /joint_states`",
        "- 工具中心：`tcp_nominal`",
        "",
        "## 多目标结果",
        "",
        "| 目标 | 类型 | IK | 规划/拒绝 | MoveIt/MuJoCo碰撞一致 | 最大关节误差(rad) | TCP位置误差(m) | 结论 |",
        "|---|---|---:|---:|---:|---:|---:|---|",
    ]
    for result in report["tests"]:
        ik = result["ik"].get("collision_aware_pass")
        planning = result["planning"].get("pass")
        collision = result["collision"].get("cross_engine_boolean_match")
        joint_error = result.get("execution", {}).get(
            "max_abs_final_joint_error_rad", "-"
        )
        tcp_error = result.get("tcp_result", {}).get("position_error_m", "-")
        lines.append(
            f"| {result['id']} | {result['planning_mode']} | {ik} | {planning} | "
            f"{collision} | {joint_error} | {tcp_error} | {result['verdict']} |"
        )
    lines += [
        "",
        "## 安全边界",
        "",
        "- J1～J6 的 origin、axis、position limit 未修改。",
        "- `tcp_nominal` 与冻结相机外参未修改。",
        "- 速度 0.5 rad/s、加速度 1.0 rad/s² 仅为 V15.14 仿真执行包络，不是实机额定参数。",
        "- MoveIt 使用 25 个组件碰撞代理（含 UpperArm Motion 专用代理）与 69 对已审计排除；没有全局关闭碰撞。",
        "- MuJoCo bridge 对每个插值命令执行 0.25° 扫掠守卫，并且不提供私有 FollowJointTrajectory server。",
        "",
        "## 汇总",
        "",
        "```json",
        json.dumps(report["aggregate"], ensure_ascii=False, indent=2),
        "```",
        "",
    ]
    return "\n".join(lines)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--targets", type=Path, required=True)
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    manifest = json.loads(args.targets.read_text(encoding="utf-8"))
    manifest_sha256 = sha256(args.targets.resolve())
    target_ids = [row.get("id") for row in manifest.get("targets", [])]
    if (
        manifest.get("schema") != "go-m8010-arm-v15.14-target-manifest/1.0"
        or manifest_sha256 != ACCEPTED_TARGET_MANIFEST_SHA256
        or len(target_ids) != len(REQUIRED_TARGET_IDS)
        or set(target_ids) != REQUIRED_TARGET_IDS
        or len(target_ids) != len(set(target_ids))
    ):
        raise RuntimeError(
            "target manifest must contain exactly the accepted 12 V15.14 targets"
        )

    rclpy.init()
    node = Regression(args.model.resolve(), args.output_dir.resolve())
    started = time.time()
    try:
        node.wait_ready()
        startup_bridge_contract = node.bridge_runtime_contract()
        ground_scene = node.audit_ground_scene()
        ros_graph = node.audit_ros_graph()
        results = []
        for target in manifest["targets"]:
            node.get_logger().info(f"V15.14 target start: {target['id']}")
            result = node.run_target(target)
            results.append(result)
            node.get_logger().info(
                f"V15.14 target done: {target['id']} => {result['verdict']}"
            )
        positive = [row for row in results if row["expected_outcome"] == "execute_success"]
        negative = [row for row in results if row["expected_outcome"] != "execute_success"]
        node.spin_for(0.20)
        post_run_bridge_contract = node.bridge_runtime_contract()
        all_expectations = bool(
            len(results) == len(manifest["targets"])
            and ros_graph["pass"]
            and ground_scene["pass"]
            and startup_bridge_contract["pass"]
            and post_run_bridge_contract["pass"]
            and all(
                row["verdict"] in ("PASS", "EXPECTED_REJECTION_PASS")
                for row in results
            )
        )
        joint_errors = [
            row.get("execution", {}).get("max_abs_final_joint_error_rad")
            for row in positive
        ]
        tcp_position_errors = [
            row.get("tcp_result", {}).get("position_error_m") for row in positive
        ]
        tcp_orientation_errors = [
            row.get("tcp_result", {}).get("orientation_error_rad")
            for row in positive
        ]
        sync_errors = [
            row.get("sync", {}).get("max_joint_position_error_rad")
            for row in positive
        ]
        tracking_errors = [
            row.get("execution", {})
            .get("trace_metrics", {})
            .get("max_desired_actual_joint_error_rad")
            for row in positive
        ]
        full_run_sync_errors = [
            row.get("execution", {})
            .get("trace_metrics", {})
            .get("max_public_raw_joint_error_rad")
            for row in positive
        ]
        joint_errors = [value for value in joint_errors if value is not None]
        tcp_position_errors = [
            value for value in tcp_position_errors if value is not None
        ]
        tcp_orientation_errors = [
            value for value in tcp_orientation_errors if value is not None
        ]
        sync_errors = [value for value in sync_errors if value is not None]
        tracking_errors = [value for value in tracking_errors if value is not None]
        full_run_sync_errors = [
            value for value in full_run_sync_errors if value is not None
        ]
        aggregate = {
            "positive_target_count": len(positive),
            "expected_rejection_count": len(negative),
            "all_expectations_met": all_expectations,
            "ros_graph_pass": ros_graph["pass"],
            "ground_scene_pass": ground_scene["pass"],
            "startup_bridge_runtime_contract_pass": startup_bridge_contract[
                "pass"
            ],
            "post_run_bridge_runtime_contract_pass": post_run_bridge_contract[
                "pass"
            ],
            "max_final_joint_error_rad": max(joint_errors or [0.0]),
            "max_tcp_position_error_m": max(tcp_position_errors or [0.0]),
            "max_tcp_orientation_error_rad": max(
                tcp_orientation_errors or [0.0]
            ),
            "max_rviz_mujoco_joint_sync_error_rad": max(sync_errors or [0.0]),
            "max_trajectory_tracking_error_rad": max(tracking_errors or [0.0]),
            "max_full_run_public_raw_sync_error_rad": max(
                full_run_sync_errors or [0.0]
            ),
            "pass": all_expectations,
        }
        report = {
            "schema": "go-m8010-arm-v15.14-trajectory-closed-loop-qa/1.0",
            "revision": "V15.14-MoveIt2-ros2_control-MuJoCo",
            "generated_at_unix": time.time(),
            "duration_s": time.time() - started,
            "status": "PASS" if all_expectations else "FAIL",
            "scope": {
                "execution_mode": "kinematic_position_tracking",
                "dynamics_valid": False,
                "simulation_only": True,
                "tcp_frame": "tcp_nominal",
            },
            "thresholds": THRESHOLDS,
            "target_manifest": {
                "path": str(args.targets.resolve()),
                "sha256": sha256(args.targets.resolve()),
            },
            "mujoco_model": {
                "path": str(args.model.resolve()),
                "sha256": sha256(args.model.resolve()),
            },
            "ros_graph": ros_graph,
            "ground_scene": ground_scene,
            "bridge_runtime_contract": {
                "startup": startup_bridge_contract,
                "post_run": post_run_bridge_contract,
            },
            "tests": results,
            "aggregate": aggregate,
        }
        json_path = args.output_dir / "QA_V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环.json"
        markdown_path = args.output_dir / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环验收.md"
        json_path.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        markdown_path.write_text(render_markdown(report), encoding="utf-8")
        print(json.dumps(report["aggregate"], ensure_ascii=False, indent=2))
        raise SystemExit(0 if all_expectations else 2)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

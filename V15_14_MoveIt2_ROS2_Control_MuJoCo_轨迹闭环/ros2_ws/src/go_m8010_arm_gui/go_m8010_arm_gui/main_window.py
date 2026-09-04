"""V15.30A-FT 六自由度机械臂全中文 PySide6 主窗口。"""

from __future__ import annotations

import csv
import bisect
import concurrent.futures
import hashlib
import json
import math
import secrets
import struct
import sys
import threading
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
import yaml
import mujoco
import numpy as np

from go_m8010_arm_hardware.thermal_manager import load_thermal_limits

from PySide6.QtCore import QEvent, Qt, QTimer, Signal
from PySide6.QtGui import QCloseEvent, QColor, QFont, QImage, QPixmap
from PySide6.QtWidgets import (
    QAbstractItemView, QAbstractScrollArea, QApplication, QDoubleSpinBox, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QMainWindow, QMessageBox, QProgressBar, QPushButton,
    QScrollArea, QTableWidget, QTableWidgetItem,
    QSizePolicy, QSlider, QVBoxLayout, QWidget,
)

from .state_machine import ArmMode, ArrivalTracker, MODE_TEXT, ModeMachine
from .workflow_contract import (
    ContractViolation,
    PreviewChecks,
    RealSubmitRejected,
    TrajectoryPlan,
    TrajectoryRecipe,
    WorkflowState,
    generate_segmented_quintic_recipe,
    planned_trajectory_feasibility_request,
    trajectory_command_descriptor,
    trajectory_plan_manifest,
    trajectory_sample_index_at,
)


JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
JOINT_LABELS = tuple(f"关节{index} J{index}" for index in range(1, 7))
MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
MOTOR_GROUPS = (
    ("J1",), ("J2A", "J2B"), ("J3",),
    ("J4",), ("J5",), ("J6",),
)
MOTOR_LOGICAL_JOINT = {
    "J1": "J1", "J2A": "J2", "J2B": "J2", "J3": "J3",
    "J4": "J4", "J5": "J5", "J6": "J6",
}
MOTOR_BUS_LOCAL_ID = {
    "J1": "0", "J2A": "0", "J2B": "1", "J3": "3",
    "J4": "4", "J5": "5", "J6": "1",
}
THERMAL_STATE_CN = {
    "OFFLINE": "离线",
    "NORMAL": "正常",
    "WARNING": "温度预警",
    "DERATING": "热降额",
    "THERMAL_STOP": "热停机",
    "COOLDOWN": "冷却中",
    "WAIT_OPERATOR_CONFIRM": "等待确认",
}
CONTROLLER_MODES = frozenset({"brake", "drag", "hold", "position", "unknown"})
DEG = 180.0 / math.pi
RAD = math.pi / 180.0
CONTROL_STREAM_TIMEOUT_S = 0.5
MUJOCO_STREAM_TIMEOUT_S = 0.5
ROUTER_STATUS_TIMEOUT_S = 1.5
GRAVITY_STATUS_TIMEOUT_S = 0.5
GRAVITY_STATUS_SCHEMA = "go-m8010-gravity-status/1.1"
PLANNED_FEASIBILITY_SCHEMA = (
    "go-m8010-planned-load-thermal-feasibility/1.0"
)
EMPIRICAL_PLANNED_FEASIBILITY_SCHEMA = (
    "go-m8010-empirical-planned-load-thermal-feasibility/1.0"
)
EMPIRICAL_AUTHORITY_CLASS = "EMPIRICAL_VALIDATION_ENVELOPE"
EMPIRICAL_RATING_CLASSIFICATION = "NOT_OFFICIAL_CONTINUOUS_RATING"
EMPIRICAL_THERMAL_EVALUATION_BASIS = (
    "CURRENT_MEASURED_TEMPERATURE_BELOW_EMPIRICAL_ENTRY_"
    "MODEL_GRAVITY_WITHIN_SOFTWARE_HARD_LIMIT_"
    "NO_CONTINUOUS_RATING_CLAIM"
)
PREVIEW_PLAN_BUILD_TIMEOUT_S = 15.0
PLANNED_REQUEST_PUBLISH_MAX_AGE_NS = 1_000_000_000
GRAVITY_SCALE_LEVELS = (0.0, 0.25, 0.50, 0.75, 1.0)
GRAVITY_ROTOR_FEEDFORWARD_LIMIT_NM = (0.20, 1.75, 1.10, 0.40, 0.20, 0.0)
EMPIRICAL_GRAVITY_ROTOR_LIMIT_NM = (
    0.20, 1.75, 1.75, 1.10, 0.40, 0.20, 0.0,
)
TRAJECTORY_EXECUTE_LEAD_NS = 250_000_000
SUMMARY_REFRESH_PERIOD_S = 0.2
ROS_CALLBACK_BUDGET_PER_TICK = 8
ROUTER_REJECTION_WARNING_WINDOW_MS = 5000.0
OPERATOR_NOTICE_DURATION_S = 8.0
TARGET_SELECTION_DEADBAND_RAD = math.radians(0.01)
COLLISION_MARGIN_DEG = 2.0
COLLISION_PREVIEW_DEBOUNCE_MS = 60
COLLISION_GUARD_TIMEOUT_S = 8.0
COLLISION_START_MATCH_TOLERANCE_RAD = math.radians(0.25)
COLLISION_HOLD_TRACKING_TOLERANCE_RAD = math.radians(0.25)
COLLISION_HOLD_VELOCITY_TOLERANCE_RAD_S = math.radians(0.25)
COLLISION_GUARD_MAX_STEP_DEG = 0.25
COLLISION_BOUNDARY_TOLERANCE_DEG = 0.01
COLLISION_GROUND_POLICY = "UNSAFE_GROUND_COLLISION_NOT_SELF_COLLISION"
COLLISION_MARGIN_POLICY = "SINGLE_JOINT_EXTENDED_SWEEP_SUPPORT_CROSS_FINAL_LINF_V2"
COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG = 2.25
COLLISION_HOLD_TRACKING_TOLERANCE_DEG = 0.25
COLLISION_TUBE_PROBE_JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
COLLISION_TUBE_GRID_OFFSETS = (-1, 0, 1)
COLLISION_TUBE_GRID_MAX_POSE_COUNT = 729
COLLISION_TUBE_AXIS_PROBE_STEP_DEG = 0.25
COLLISION_TUBE_CROSS_SECTION_MAX_POSE_COUNT = 825
COLLISION_EXECUTE_SEGMENT_MAX_DEG = 30.0
PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG = (
    (-180.0, 180.0), (-170.0, 170.0), (-170.0, 170.0),
    (-116.0, 159.0), (-70.6, 151.2), (-180.0, 180.0),
)
PRODUCTION_MODEL_SHA256 = (
    "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
)
GRAVITY_CONFIG_SHA256 = (
    "307469b8384fd35547327ba1d5f80aa440e6b9663406ab7c9d9469bea263335d"
)
THERMAL_CONFIG_SHA256 = (
    "1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467"
)
PLAN_ACTUAL_DRIFT_TOLERANCE_RAD = math.radians(0.25)
PRODUCTION_COLLISION_CONTRACT_SHA256 = (
    "6d802909e44f238816f007ef33b57e1b57099c5522a473cd2dcc2705397af9c9"
)
PRODUCTION_KINEMATIC_GUARD_SHA256 = (
    "648b9de953a84be46cd5f95e24daf9f683f3025e73048dc068ef7bd8e5670c6a"
)


def build_virtual_preview_plan(snapshot: dict) -> dict:
    """Build and serialize one immutable preview request off the Qt thread.

    ``snapshot`` contains only frozen values captured by the event thread.  In
    particular this worker never reads a widget, mutates ``MainWindow``, calls
    a ROS publisher, or grants hardware authority.  The request timestamp is
    taken only after the potentially long trajectory generation has finished,
    leaving the independent proof node its full freshness budget.
    """

    required = {
        "generation", "workflow", "actual_rad", "target_rad", "limits_rad",
        "maximum_velocity_rad_s", "maximum_acceleration_rad_s2",
        "maximum_segment_delta_rad", "maximum_sample_period_s",
        "source_instance_id", "request_sequence", "session_id",
        "state_instance_id", "base_candidate_revision",
    }
    if not isinstance(snapshot, dict) or set(snapshot) != required:
        raise ContractViolation("preview-plan worker snapshot fields are not exact")
    workflow = snapshot["workflow"]
    if not isinstance(workflow, WorkflowState):
        raise ContractViolation("preview-plan worker requires WorkflowState")
    workflow = workflow.replace_authority(
        joint_limits_rad=snapshot["limits_rad"],
        model_sha256=PRODUCTION_MODEL_SHA256,
        session_id=snapshot["session_id"],
        state_instance_id=snapshot["state_instance_id"],
        gravity_config_sha256=GRAVITY_CONFIG_SHA256,
    )
    workflow = workflow.update_actual(snapshot["actual_rad"])
    workflow = workflow.change_plan_target(snapshot["target_rad"])
    trajectory = generate_segmented_quintic_recipe(
        workflow.q_actual,
        workflow.q_plan_target,
        workflow.joint_limits_rad,
        maximum_velocity_rad_s=snapshot["maximum_velocity_rad_s"],
        maximum_acceleration_rad_s2=snapshot["maximum_acceleration_rad_s2"],
        maximum_segment_delta_rad=snapshot["maximum_segment_delta_rad"],
        maximum_sample_period_s=snapshot["maximum_sample_period_s"],
    )
    workflow = workflow.set_plan_trajectory(trajectory)
    source_monotonic_ns = time.monotonic_ns()
    payload = planned_trajectory_feasibility_request(
        trajectory,
        source_instance_id=snapshot["source_instance_id"],
        sequence=snapshot["request_sequence"],
        source_monotonic_ns=source_monotonic_ns,
        session_id=snapshot["session_id"],
        state_instance_id=snapshot["state_instance_id"],
        model_sha256=PRODUCTION_MODEL_SHA256,
        gravity_config_sha256=GRAVITY_CONFIG_SHA256,
        thermal_config_sha256=THERMAL_CONFIG_SHA256,
    )
    serialized_request = json.dumps(
        payload,
        ensure_ascii=False,
        allow_nan=False,
        separators=(",", ":"),
    )
    return {
        "generation": snapshot["generation"],
        "base_candidate_revision": snapshot["base_candidate_revision"],
        "actual_rad": tuple(snapshot["actual_rad"]),
        "target_rad": tuple(snapshot["target_rad"]),
        "session_id": snapshot["session_id"],
        "state_instance_id": snapshot["state_instance_id"],
        "maximum_velocity_rad_s": snapshot["maximum_velocity_rad_s"],
        "maximum_acceleration_rad_s2": snapshot[
            "maximum_acceleration_rad_s2"
        ],
        "maximum_segment_delta_rad": snapshot["maximum_segment_delta_rad"],
        "maximum_sample_period_s": snapshot["maximum_sample_period_s"],
        "limits_rad": tuple(tuple(pair) for pair in snapshot["limits_rad"]),
        "workflow": workflow,
        "trajectory": trajectory,
        "request_source_monotonic_ns": source_monotonic_ns,
        "serialized_request": serialized_request,
    }


HARDWARE_STATE_SOURCE_MAX_AGE_NS = 250_000_000
HARDWARE_STATE_SOURCE_TAKEOVER_TIMEOUT_NS = 500_000_000
SOURCE_INSTANCE_ID_HEX_LENGTH = 32
MAX_TRACKED_STATE_SOURCES = 32


def set_widget_text_if_changed(widget: object, text: str) -> bool:
    """Avoid redundant Qt text writes and the resulting layout/repaint work."""

    if widget.text() == text:
        return False
    widget.setText(text)
    return True


def set_widget_enabled_if_changed(widget: object, enabled: bool) -> bool:
    """Change a widget's enabled state only when its state really changed."""

    wanted = bool(enabled)
    if widget.isEnabled() == wanted:
        return False
    widget.setEnabled(wanted)
    return True


def set_widget_tooltip_if_changed(widget: object, text: str) -> bool:
    """Avoid re-polishing controls with an identical tooltip every GUI tick."""

    if widget.toolTip() == text:
        return False
    widget.setToolTip(text)
    return True


def set_widget_value_if_changed(widget: object, value: int) -> bool:
    """Avoid redundant slider updates and their paint/event side effects."""

    if widget.value() == value:
        return False
    widget.setValue(value)
    return True


def pump_ros_callbacks(node: object, spin_once: object) -> None:
    """Drain a bounded number of ready callbacks without blocking the Qt loop."""

    for _ in range(ROS_CALLBACK_BUDGET_PER_TICK):
        spin_once(node, timeout_sec=0.0)


def fitted_window_size(
    available_width: int, available_height: int,
) -> tuple[int, int]:
    """Return a comfortable initial size that never exceeds the work area."""

    return (
        max(1, min(1920, int(available_width * 0.95))),
        max(1, min(980, int(available_height * 0.95))),
    )


def verified_edit_limits(
    mechanical_limits: list[tuple[float, float]],
) -> list[tuple[float, float]]:
    """Validate and expose the complete model-derived per-joint angle domain."""

    if len(mechanical_limits) != 6:
        raise ValueError("机械限位必须包含六个关节")
    result = []
    for bounds in mechanical_limits:
        if len(bounds) != 2:
            raise ValueError("机械限位必须是上下界对")
        lower, upper = (float(bounds[0]), float(bounds[1]))
        if not (
            math.isfinite(lower)
            and math.isfinite(upper)
            and lower < upper
        ):
            raise ValueError("机械限位无效")
        result.append((lower, upper))
    return result


def model_session_relative_limits(
    model_path: Path, session_pose_deg: str,
) -> list[tuple[float, float]]:
    """Read the frozen MJCF limits and express them around this GUI session."""

    pose_deg = np.degrees(parse_pose_degrees(session_pose_deg))
    root = ET.parse(model_path).getroot()
    compiler = root.find("compiler")
    if compiler is not None and compiler.get("angle", "degree") != "radian":
        raise ValueError("冻结MuJoCo模型必须使用弧度关节范围")
    joints = {
        element.get("name"): element
        for element in root.iter("joint")
        if element.get("name") in {f"J{index}" for index in range(1, 7)}
    }
    if set(joints) != {f"J{index}" for index in range(1, 7)}:
        raise ValueError("冻结MuJoCo模型缺少J1..J6关节")
    limits = []
    for index in range(6):
        name = f"J{index + 1}"
        joint = joints[name]
        if name == "J1" and joint.get("limited", "false") == "false":
            absolute_lower, absolute_upper = -180.0, 180.0
        else:
            fields = (joint.get("range") or "").split()
            if len(fields) != 2:
                raise ValueError(f"冻结MuJoCo模型{name}缺少机械范围")
            absolute_lower, absolute_upper = (
                math.degrees(float(fields[0])),
                math.degrees(float(fields[1])),
            )
        limits.append((
            absolute_lower - float(pose_deg[index]),
            absolute_upper - float(pose_deg[index]),
        ))
    return verified_edit_limits(limits)


def collision_target_sha256(values_rad: list[float]) -> str:
    """Canonical target identity shared by the GUI and MuJoCo guard."""

    if len(values_rad) != 6 or not all(math.isfinite(float(v)) for v in values_rad):
        raise ValueError("碰撞检查目标必须包含六个有限角度")
    return hashlib.sha256(
        struct.pack(">6d", *(float(value) for value in values_rad))
    ).hexdigest()


def session_pose_sha256(session_pose_rad: list[float]) -> str:
    """Bind collision proofs to the exact six-axis absolute session pose."""

    return collision_target_sha256(session_pose_rad)


def canonical_collision_hardware_state_sha256(value: dict) -> str:
    document = {
        "schema": value["schema"],
        "session_id": value["session_id"],
        "state_instance_id": value["state_instance_id"],
        "sequence": value["sequence"],
        "source_monotonic_ns": value["source_monotonic_ns"],
        "position_rad": list(value["position_rad"]),
        "velocity_rad_s": list(value["velocity_rad_s"]),
        "controller_mode_by_motor": [
            [name, value["controller_mode_by_motor"][name]]
            for name in MOTOR_NAMES
        ],
    }
    encoded = json.dumps(
        document,
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def collision_guard_result_matches(result: object, request: dict) -> bool:
    """Accept only the exact frozen-model proof requested by this GUI."""

    try:
        if (
            not isinstance(result, dict)
            or result.get("schema")
            != "go-m8010-collision-guard-result/1.0"
            or result.get("source_instance_id")
            != request["source_instance_id"]
            or result.get("request_sequence") != request["request_sequence"]
            or result.get("source_monotonic_ns")
            != request["source_monotonic_ns"]
            or result.get("kind") != request["kind"]
            or result.get("session_id") != request["session_id"]
            or result.get("state_instance_id") != request["state_instance_id"]
            or result.get("session_pose_sha256")
            != request["session_pose_sha256"]
            or result.get("moving_joint_mask")
            != request["moving_joint_mask"]
            or result.get("target_sha256") != request["target_sha256"]
            or type(result.get("collision_margin_deg")) not in {int, float}
            or not math.isfinite(float(result["collision_margin_deg"]))
            or abs(
                float(result["collision_margin_deg"])
                - float(request["collision_margin_deg"])
            ) > 1.0e-12
            or result.get("model_sha256") != PRODUCTION_MODEL_SHA256
            or result.get("collision_contract_sha256")
            != PRODUCTION_COLLISION_CONTRACT_SHA256
            or result.get("kinematic_guard_sha256")
            != PRODUCTION_KINEMATIC_GUARD_SHA256
            or result.get("ground_contact_policy") != COLLISION_GROUND_POLICY
            or result.get("margin_policy") != COLLISION_MARGIN_POLICY
            or result.get("joint_space_tube_radius_deg")
            != COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG
            or result.get("hold_tracking_tolerance_deg")
            != COLLISION_HOLD_TRACKING_TOLERANCE_DEG
            or result.get("tube_probe_joint_names")
            != list(COLLISION_TUBE_PROBE_JOINT_NAMES)
            or result.get("tube_grid_offsets")
            != list(COLLISION_TUBE_GRID_OFFSETS)
            or result.get("tube_grid_max_pose_count")
            != COLLISION_TUBE_GRID_MAX_POSE_COUNT
            or result.get("tube_axis_probe_step_deg")
            != COLLISION_TUBE_AXIS_PROBE_STEP_DEG
            or result.get("tube_cross_section_max_pose_count")
            != COLLISION_TUBE_CROSS_SECTION_MAX_POSE_COUNT
            or result.get("absolute_joint_limits_deg")
            != [list(bounds) for bounds in PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG]
            or result.get("single_joint_path_required") is not True
            or type(result.get("safe")) is not bool
        ):
            return False
        for field in ("start_relative_rad", "target_relative_rad"):
            expected = request[field]
            observed = result.get(field)
            if (
                not isinstance(observed, list)
                or len(observed) != 6
                or any(
                    type(value) not in {int, float}
                    or not math.isfinite(float(value))
                    for value in observed
                )
                or any(
                    abs(float(actual) - float(wanted)) > 1.0e-12
                    for actual, wanted in zip(observed, expected)
                )
            ):
                return False
        hardware_sequence = result.get("hardware_state_sequence")
        hardware_source_ns = result.get("hardware_state_source_monotonic_ns")
        hardware_position = result.get("hardware_position_rad")
        hardware_velocity = result.get("hardware_velocity_rad_s")
        hardware_modes = result.get("hardware_controller_mode_by_motor")
        if (
            type(hardware_sequence) is not int
            or hardware_sequence <= 0
            or type(hardware_source_ns) is not int
            or hardware_source_ns <= 0
            or not isinstance(hardware_position, list)
            or len(hardware_position) != 6
            or not isinstance(hardware_velocity, list)
            or len(hardware_velocity) != 6
            or any(
                type(value) not in {int, float}
                or not math.isfinite(float(value))
                for value in hardware_position + hardware_velocity
            )
            or any(
                abs(float(value)) >
                COLLISION_HOLD_VELOCITY_TOLERANCE_RAD_S + 1.0e-12
                for value in hardware_velocity
            )
            or (
                request["kind"] != "plan_preview"
                and any(
                    abs(float(measured) - float(start)) >
                    COLLISION_START_MATCH_TOLERANCE_RAD + 1.0e-12
                    for measured, start in zip(
                        hardware_position, request["start_relative_rad"]
                    )
                )
            )
            or not isinstance(hardware_modes, dict)
            or set(hardware_modes) != set(MOTOR_NAMES)
            or any(hardware_modes[name] != "hold" for name in MOTOR_NAMES)
            or result.get("hardware_state_sha256")
            != canonical_collision_hardware_state_sha256({
                "schema": "go-m8010-hardware-state/1.1",
                "session_id": request["session_id"],
                "state_instance_id": request["state_instance_id"],
                "sequence": hardware_sequence,
                "source_monotonic_ns": hardware_source_ns,
                "position_rad": [float(value) for value in hardware_position],
                "velocity_rad_s": [float(value) for value in hardware_velocity],
                "controller_mode_by_motor": hardware_modes,
            })
        ):
            return False
        max_step = result.get("max_step_deg")
        boundary_tolerance = result.get("boundary_tolerance_deg")
        if (
            type(max_step) not in {int, float}
            or not math.isfinite(float(max_step))
            or not 0.0 < float(max_step) <= COLLISION_GUARD_MAX_STEP_DEG
            or type(boundary_tolerance) not in {int, float}
            or not math.isfinite(float(boundary_tolerance))
            or not 0.0 < float(boundary_tolerance)
            <= COLLISION_BOUNDARY_TOLERANCE_DEG
        ):
            return False
        reason = result.get("reason")
        pairs = result.get("contact_pairs")
        if (
            not isinstance(reason, str)
            or not reason
            or not isinstance(pairs, list)
            or any(
                not isinstance(pair, list)
                or len(pair) != 2
                or not all(isinstance(name, str) and name for name in pair)
                for pair in pairs
            )
        ):
            return False
        moving_mask = request["moving_joint_mask"]
        if (
            not isinstance(moving_mask, list)
            or len(moving_mask) != 6
            or not all(type(value) is bool for value in moving_mask)
            or sum(moving_mask) < 1
            or (request["kind"] != "pose_preview" and sum(moving_mask) != 1)
            or result.get("moving_joint_count") != sum(moving_mask)
            or result.get("moving_joint") != (
                COLLISION_TUBE_PROBE_JOINT_NAMES[moving_mask.index(True)]
                if sum(moving_mask) == 1 else None
            )
        ):
            return False
        recommendation = result.get("recommended_relative_rad")
        if result["safe"]:
            if reason != "clear" or pairs:
                return False
            if (
                not isinstance(recommendation, list)
                or len(recommendation) != 6
                or any(
                    type(value) not in {int, float}
                    or not math.isfinite(float(value))
                    for value in recommendation
                )
                or any(
                    abs(float(actual) - float(wanted)) > 1.0e-12
                    for actual, wanted in zip(
                        recommendation, request["target_relative_rad"]
                    )
                )
            ):
                return False
            for field in (
                "step_count", "path_sample_count", "pose_check_count",
                "tube_probe_count", "tube_probe_cache_hit_count",
            ):
                if type(result.get(field)) is not int or result[field] < 0:
                    return False
        elif recommendation is not None and (
            not isinstance(recommendation, list)
            or len(recommendation) != 6
            or any(
                type(value) not in {int, float}
                or not math.isfinite(float(value))
                for value in recommendation
            )
        ):
            return False
        checked_ns = result.get("checked_monotonic_ns")
        now_ns = time.monotonic_ns()
        return bool(
            type(checked_ns) is int
            and request["source_monotonic_ns"] <= checked_ns <= now_ns
            and hardware_source_ns <= checked_ns
            and now_ns - checked_ns
            <= int(COLLISION_GUARD_TIMEOUT_S * 1.0e9)
        )
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def strict_bool_parameter(value: object, name: str) -> bool:
    if type(value) is not bool:
        raise TypeError(f"ROS参数{name}必须是严格布尔值")
    return value


def source_instance_id_valid(value: object) -> bool:
    return bool(
        isinstance(value, str)
        and len(value) == SOURCE_INSTANCE_ID_HEX_LENGTH
        and all(character in "0123456789abcdef" for character in value)
    )


def optional_sha256_valid(value: object) -> bool:
    return bool(
        value is None
        or (
            isinstance(value, str)
            and len(value) == 64
            and all(character in "0123456789abcdef" for character in value)
        )
    )


def hardware_state_source_is_fresh(value: dict, now_ns: int) -> bool:
    source_ns = value.get("source_monotonic_ns")
    return bool(
        type(source_ns) is int
        and source_ns > 0
        and source_ns <= now_ns
        and now_ns - source_ns <= HARDWARE_STATE_SOURCE_MAX_AGE_NS
    )


def hardware_state_contract_valid(value: object) -> bool:
    """Accept only a complete state-node snapshot as a freshness heartbeat."""

    try:
        if (
            not isinstance(value, dict)
            or value.get("schema") != "go-m8010-hardware-state/1.1"
            or tuple(value.get("joint_names", ())) != JOINT_NAMES
            or type(value.get("sequence")) is not int
            or value["sequence"] <= 0
            or not isinstance(value.get("session_id"), str)
            or not value["session_id"]
            or not source_instance_id_valid(value.get("state_instance_id"))
            or type(value.get("source_monotonic_ns")) is not int
            or value["source_monotonic_ns"] <= 0
            or value.get("reference") not in {
                "SESSION_REFERENCE_V1", "PERSISTENT_SOFTWARE_ZERO_V1",
            }
            or not optional_sha256_valid(value.get("persistent_zero_sha256"))
            or not optional_sha256_valid(value.get("initial_pose_sha256"))
            or type(value.get("j2_sync_fault")) is not bool
            or type(value.get("j2_e_sync_rad")) not in {int, float}
            or not math.isfinite(float(value.get("j2_e_sync_rad")))
        ):
            return False
        for field in ("position_rad", "velocity_rad_s"):
            items = value.get(field)
            if (
                not isinstance(items, list)
                or len(items) != 6
                or not all(
                    type(item) in {int, float} and math.isfinite(float(item))
                    for item in items
                )
            ):
                return False
        per_motor = value.get("per_motor")
        modes = value.get("controller_mode_by_motor")
        faults = value.get("controller_fault_by_motor")
        lease_holds = value.get("lease_safe_hold_by_motor")
        if (
            not isinstance(per_motor, dict)
            or set(per_motor) != set(MOTOR_NAMES)
            or not isinstance(modes, dict)
            or set(modes) != set(MOTOR_NAMES)
            or not isinstance(faults, dict)
            or set(faults) != set(MOTOR_NAMES)
            or not isinstance(lease_holds, dict)
            or set(lease_holds) != set(MOTOR_NAMES)
        ):
            return False
        for name in MOTOR_NAMES:
            item = per_motor[name]
            if (
                not isinstance(item, dict)
                or type(item.get("communication_ok")) is not bool
                or type(item.get("fresh")) is not bool
                or type(item.get("reference_captured")) is not bool
                or type(item.get("merror")) is not int
                or any(
                    type(item.get(field)) not in {int, float}
                    or not math.isfinite(float(item[field]))
                    for field in (
                        "q_joint_rad", "dq_joint_rad_s", "temperature_c",
                    )
                )
                or modes[name] not in CONTROLLER_MODES
                or type(faults[name]) is not bool
                or type(lease_holds[name]) is not bool
            ):
                return False
        return True
    except (KeyError, TypeError, ValueError, OverflowError):
        return False


def initial_pose_binding_valid(
    document: object, raw_bytes: bytes, hardware: object
) -> bool:
    """Bind the GUI action to the exact initial pose loaded by the state node."""

    if not isinstance(document, dict) or not isinstance(hardware, dict):
        return False
    persistent_sha256 = hardware.get("persistent_zero_sha256")
    initial_pose_sha256 = hardware.get("initial_pose_sha256")
    if (
        hardware.get("reference") != "PERSISTENT_SOFTWARE_ZERO_V1"
        or not isinstance(persistent_sha256, str)
        or not optional_sha256_valid(persistent_sha256)
        or not isinstance(initial_pose_sha256, str)
        or not optional_sha256_valid(initial_pose_sha256)
    ):
        return False
    return bool(
        document.get("有效") is True
        and document.get("会话标识")
        == f"persistent:{persistent_sha256[:16]}"
        and hashlib.sha256(raw_bytes).hexdigest() == initial_pose_sha256
    )


def classify_logical_joint_observations(
    hardware: dict,
) -> tuple[list[bool], list[bool], list[bool]]:
    """Separate confirmed health/fault from missing or stale observation."""

    per_motor = hardware["per_motor"]
    controller_faults = hardware["controller_fault_by_motor"]
    connected = []
    faulted = []
    uncertain = []
    for index, names in enumerate(MOTOR_GROUPS):
        samples = [per_motor[name] for name in names]
        observed = all(
            sample["fresh"] and sample["reference_captured"]
            for sample in samples
        )
        hard_fault = observed and (
            any(controller_faults[name] for name in names)
            or (
                index == 1
                # The GO worker owns the consecutive-frame/hysteresis filter.
                # A raw A/B sample skew is diagnostic only and must never make
                # the GUI withdraw an otherwise healthy J2 holding command.
                and hardware["j2_sync_fault"]
            )
        )
        explicit_feedback_fault = observed and any(
            not sample["communication_ok"] or sample["merror"] != 0
            for sample in samples
        )
        is_faulted = bool(hard_fault or explicit_feedback_fault)
        is_uncertain = bool(not is_faulted and not observed)
        faulted.append(is_faulted)
        uncertain.append(is_uncertain)
        connected.append(bool(observed and not is_faulted))
    return connected, faulted, uncertain


def collision_motion_state_ready(
    hardware: object,
    command_targets: list[float],
    requested_active_joint_mask: list[bool],
    hardware_mode: str,
    *,
    now_ns: Optional[int] = None,
) -> bool:
    """Require a stationary, fully observed six-axis HOLD before any move.

    The production workers profile each fault domain independently.  Starting
    only one new joint from a confirmed whole-arm HOLD keeps the physical path
    equivalent to the one-axis MuJoCo sweep and keeps every other load-bearing
    joint under its existing immutable position target.
    """

    checked_ns = time.monotonic_ns() if now_ns is None else now_ns
    if (
        hardware_mode != "hold"
        or not hardware_state_contract_valid(hardware)
        or not hardware_state_source_is_fresh(hardware, checked_ns)
        or len(command_targets) != 6
        or len(requested_active_joint_mask) != 6
        or not all(type(value) is bool and value for value in requested_active_joint_mask)
        or not all(math.isfinite(float(value)) for value in command_targets)
    ):
        return False
    connected, faulted, uncertain = classify_logical_joint_observations(hardware)
    if not all(connected) or any(faulted) or any(uncertain):
        return False
    controller_modes = hardware["controller_mode_by_motor"]
    if any(controller_modes[name] != "hold" for name in MOTOR_NAMES):
        return False
    if any(
        abs(float(measured) - float(target))
        > COLLISION_HOLD_TRACKING_TOLERANCE_RAD
        for measured, target in zip(hardware["position_rad"], command_targets)
    ):
        return False
    return not any(
        abs(float(velocity)) > COLLISION_HOLD_VELOCITY_TOLERANCE_RAD_S
        for velocity in hardware["velocity_rad_s"]
    )


def virtual_target_edit_state_ready(
    hardware: object,
    requested_active_joint_mask: list[bool],
    hardware_mode: str,
    *,
    now_ns: Optional[int] = None,
) -> bool:
    """Allow virtual-only editing from a healthy whole-arm HOLD.

    Editing a preview target grants no motion authority.  The stricter
    stationary/tracking gate remains mandatory when Execute requests a real
    collision proof and again when the approved target is committed.
    """

    # Keep ``now_ns`` in the public helper signature for callers/tests that
    # compare this preview-only gate with the execution gate.  A snapshot was
    # already required to be <=250 ms old when its callback was accepted, and
    # ArmGuiWindow additionally requires the receipt stream to be <=500 ms old.
    # Reapplying the tighter 250 ms threshold here made controls alternate
    # enabled/disabled between otherwise healthy frames.  Editing grants no
    # motor authority; Execute still uses collision_motion_state_ready(), which
    # retains the strict source-age, tracking, velocity and collision checks.
    _ = now_ns
    if (
        hardware_mode != "hold"
        or not hardware_state_contract_valid(hardware)
        or len(requested_active_joint_mask) != 6
        or not all(
            type(value) is bool and value
            for value in requested_active_joint_mask
        )
    ):
        return False
    connected, faulted, uncertain = classify_logical_joint_observations(hardware)
    if not all(connected) or any(faulted) or any(uncertain):
        return False
    controller_modes = hardware["controller_mode_by_motor"]
    return all(controller_modes[name] == "hold" for name in MOTOR_NAMES)


def moving_targets_within_model_limits(
    targets: list[float], moving_joint_mask: list[bool],
    limits_deg: list[tuple[float, float]],
) -> bool:
    if (
        len(targets) != 6
        or len(moving_joint_mask) != 6
        or len(limits_deg) != 6
    ):
        return False
    return all(
        not moving or (
            math.isfinite(target)
            and limits_deg[index][0] * RAD - 1e-12
            <= target
            <= limits_deg[index][1] * RAD + 1e-12
        )
        for index, (target, moving) in enumerate(zip(targets, moving_joint_mask))
    )


def moving_trajectory_feedback_complete(
    hardware: object,
    moving_joint_index: object,
    trajectory_descriptor: object,
) -> bool:
    """Require the moving worker(s) to echo this exact completed segment."""

    return moving_trajectory_feedback_fraction(
        hardware, moving_joint_index, trajectory_descriptor
    ) == 1.0


def moving_trajectory_feedback_fraction(
    hardware: object,
    moving_joint_index: object,
    trajectory_descriptor: object,
) -> Optional[float]:
    """Return conservative progress from fresh, identity-matched workers.

    The returned value is based only on the integer trajectory sample echoed
    by every motor in the logical joint.  In particular, local GUI wall time
    and the already-finished virtual preview can never advance real progress.
    ``None`` means that no trustworthy progress value is available.
    """

    if (
        not isinstance(hardware, dict)
        or type(moving_joint_index) is not int
        or not 0 <= moving_joint_index < len(MOTOR_GROUPS)
        or not isinstance(trajectory_descriptor, dict)
        or set(trajectory_descriptor) != {"plan_token_id", "trajectory"}
    ):
        return None
    token_id = trajectory_descriptor.get("plan_token_id")
    trajectory = trajectory_descriptor.get("trajectory")
    per_motor = hardware.get("per_motor")
    if (
        not isinstance(token_id, str)
        or not isinstance(trajectory, dict)
        or not isinstance(per_motor, dict)
    ):
        return None
    trajectory_sha256 = trajectory.get("trajectory_sha256")
    interval_count = trajectory.get("interval_count")
    if (
        not isinstance(trajectory_sha256, str)
        or type(interval_count) is not int
        or interval_count <= 0
    ):
        return None
    fractions: list[float] = []
    for motor_name in MOTOR_GROUPS[moving_joint_index]:
        sample = per_motor.get(motor_name)
        state = sample.get("trajectory_state") if isinstance(sample, dict) else None
        sample_index = (
            sample.get("trajectory_sample_index")
            if isinstance(sample, dict) else None
        )
        if (
            not isinstance(sample, dict)
            or sample.get("fresh") is not True
            or sample.get("trajectory_plan_token_id") != token_id
            or sample.get("trajectory_sha256") != trajectory_sha256
            or sample.get("trajectory_interval_count") != interval_count
            or state not in {"PREPARED", "RUNNING", "COMPLETE"}
            or type(sample_index) is not int
            or not 0 <= sample_index <= interval_count
            or (state == "PREPARED" and sample_index != 0)
            or (state == "RUNNING" and sample_index >= interval_count)
            or (state == "COMPLETE" and sample_index != interval_count)
        ):
            return None
        fractions.append(sample_index / interval_count)
    return min(fractions)


def planned_load_feasibility_authorizes(
    status: object,
    *,
    trajectory_sha256: str,
    session_id: str,
    state_instance_id: str,
    now_monotonic_ns: Optional[int] = None,
) -> bool:
    """Require an authoritative PASS proof bound to the exact planned path.

    Current-pose gravity authority is intentionally insufficient.  A producer
    that has not evaluated the immutable trajectory simply omits this nested
    proof (or reports a non-PASS result), which keeps PLAN_TOKEN fail-closed.
    """

    if not isinstance(status, dict):
        return False
    proof = status.get("planned_trajectory_feasibility")
    if not isinstance(proof, dict):
        return False
    checked_ns = (
        time.monotonic_ns()
        if now_monotonic_ns is None else now_monotonic_ns
    )
    source_ns = proof.get("source_monotonic_ns")
    minimum_load_margin = proof.get("minimum_rotor_torque_margin_nm")
    minimum_thermal_margin = proof.get("minimum_thermal_margin_c")
    request_sha256 = proof.get("request_sha256")
    sample_count = proof.get("sample_count")
    evaluated_sample_count = proof.get("evaluated_sample_count")
    gravity_maxima = proof.get(
        "maximum_abs_gravity_rotor_torque_nm_by_motor"
    )
    gravity_joint_maxima = proof.get(
        "maximum_abs_gravity_joint_torque_nm_by_joint"
    )
    predicted_maxima = proof.get(
        "maximum_abs_predicted_rotor_torque_nm_by_motor"
    )
    if proof.get("schema") == EMPIRICAL_PLANNED_FEASIBILITY_SCHEMA:
        empirical = status.get("empirical_validation")
        empirical_limits = proof.get(
            "empirical_gravity_rotor_limit_nm_by_motor"
        )
        go_motors = MOTOR_NAMES[:-1]
        exact_empirical_loads = bool(
            isinstance(gravity_joint_maxima, dict)
            and set(gravity_joint_maxima)
            == {"J1", "J2", "J3", "J4", "J5", "J6"}
            and all(
                type(value) in {int, float}
                and math.isfinite(float(value))
                and float(value) >= 0.0
                for value in gravity_joint_maxima.values()
            )
            and isinstance(gravity_maxima, dict)
            and set(gravity_maxima) == set(MOTOR_NAMES)
            and isinstance(predicted_maxima, dict)
            and set(predicted_maxima) == set(MOTOR_NAMES)
            and isinstance(empirical_limits, dict)
            and set(empirical_limits) == set(MOTOR_NAMES)
            and all(
                type(gravity_maxima[name]) in {int, float}
                and math.isfinite(float(gravity_maxima[name]))
                and float(gravity_maxima[name]) >= 0.0
                and type(predicted_maxima[name]) in {int, float}
                and math.isfinite(float(predicted_maxima[name]))
                and float(predicted_maxima[name]) >= 0.0
                and type(empirical_limits[name]) in {int, float}
                and math.isfinite(float(empirical_limits[name]))
                and abs(
                    float(empirical_limits[name])
                    - EMPIRICAL_GRAVITY_ROTOR_LIMIT_NM[index]
                ) <= 1.0e-12
                for index, name in enumerate(go_motors)
            )
            and (
                gravity_maxima["J6"] is None
                or (
                    type(gravity_maxima["J6"]) in {int, float}
                    and math.isfinite(float(gravity_maxima["J6"]))
                    and float(gravity_maxima["J6"]) >= 0.0
                )
            )
            and (
                predicted_maxima["J6"] is None
                or (
                    type(predicted_maxima["J6"]) in {int, float}
                    and math.isfinite(float(predicted_maxima["J6"]))
                    and float(predicted_maxima["J6"]) >= 0.0
                )
            )
            and type(empirical_limits["J6"]) in {int, float}
            and math.isfinite(float(empirical_limits["J6"]))
            and abs(
                float(empirical_limits["J6"])
                - EMPIRICAL_GRAVITY_ROTOR_LIMIT_NM[-1]
            ) <= 1.0e-12
        )
        reproduced_empirical_margin = (
            min(
                float(empirical_limits[name]) - float(gravity_maxima[name])
                for name in go_motors
            )
            if exact_empirical_loads else None
        )
        reported_empirical_margin = proof.get(
            "minimum_empirical_gravity_rotor_margin_nm"
        )
        return bool(
            gravity_status_authorizes_hardware(
                status,
                session_id=session_id,
                state_instance_id=state_instance_id,
                now_monotonic_ns=checked_ns,
            )
            and isinstance(empirical, dict)
            and empirical.get("position_validation_authorized") is True
            and empirical.get("phase") == "POSITION_VALIDATION"
            and empirical.get("stage_index") == 4
            and empirical.get("stage_complete") is True
            and proof.get("source") == "whole_arm_gravity_node"
            and proof.get("source_instance_id")
            == status.get("source_instance_id")
            and type(proof.get("sequence")) is int
            and proof["sequence"] > 0
            and proof.get("sequence") == status.get("sequence")
            and type(source_ns) is int
            and source_ns == status.get("source_monotonic_ns")
            and 0 < source_ns <= checked_ns
            and checked_ns - source_ns
            <= int(GRAVITY_STATUS_TIMEOUT_S * 1.0e9)
            and proof.get("result") == "PASS"
            and proof.get("load_feasibility") == "PASS"
            and proof.get("thermal_feasibility") == "PASS"
            and proof.get("current_temperature_margin_result") == "PASS"
            and isinstance(trajectory_sha256, str)
            and len(trajectory_sha256) == 64
            and all(
                character in "0123456789abcdef"
                for character in trajectory_sha256
            )
            and proof.get("trajectory_sha256") == trajectory_sha256
            and proof.get("session_id") == session_id
            and proof.get("state_instance_id") == state_instance_id
            and proof.get("model_sha256") == PRODUCTION_MODEL_SHA256
            and proof.get("gravity_config_sha256") == GRAVITY_CONFIG_SHA256
            and proof.get("thermal_config_sha256") == THERMAL_CONFIG_SHA256
            and isinstance(request_sha256, str)
            and len(request_sha256) == 64
            and all(
                character in "0123456789abcdef"
                for character in request_sha256
            )
            and proof.get("continuous_rotor_limits_authoritative") is False
            and proof.get("authority_class") == EMPIRICAL_AUTHORITY_CLASS
            and proof.get("rating_classification")
            == EMPIRICAL_RATING_CLASSIFICATION
            and proof.get("empirical_validation_authoritative") is True
            and proof.get("empirical_envelope_id")
            == empirical.get("envelope_id")
            and proof.get("empirical_envelope_sha256")
            == empirical.get("envelope_sha256")
            and proof.get("temperature_limits_authoritative") is True
            and proof.get("load_evaluation_basis")
            == (
                "MUJOCO_QFRC_BIAS_CONTINUOUS_PLUS_"
                "MJ_INVERSE_SHORT_PEAK_EVERY_SAMPLE"
            )
            and proof.get("thermal_evaluation_basis")
            == EMPIRICAL_THERMAL_EVALUATION_BASIS
            and type(sample_count) is int
            and sample_count > 0
            and evaluated_sample_count == sample_count
            and exact_empirical_loads
            and reproduced_empirical_margin is not None
            and reproduced_empirical_margin >= 0.0
            and type(reported_empirical_margin) in {int, float}
            and math.isfinite(float(reported_empirical_margin))
            and abs(
                float(reported_empirical_margin)
                - reproduced_empirical_margin
            ) <= 1.0e-12
            and type(minimum_load_margin) in {int, float}
            and math.isfinite(float(minimum_load_margin))
            and abs(
                float(minimum_load_margin) - reproduced_empirical_margin
            ) <= 1.0e-12
            and type(minimum_thermal_margin) in {int, float}
            and math.isfinite(float(minimum_thermal_margin))
            and float(minimum_thermal_margin) > 0.0
            and proof.get("blocker_code") is None
            and proof.get("blocker") is None
        )
    continuous_limits = proof.get(
        "continuous_rotor_torque_limit_nm_by_motor"
    )
    short_peak_limits = proof.get(
        "short_peak_rotor_torque_limit_nm_by_motor"
    )
    exact_named_loads = bool(
        isinstance(gravity_maxima, dict)
        and isinstance(gravity_joint_maxima, dict)
        and set(gravity_joint_maxima)
        == {"J1", "J2", "J3", "J4", "J5", "J6"}
        and all(
            type(value) in {int, float}
            and math.isfinite(float(value))
            and float(value) >= 0.0
            for value in gravity_joint_maxima.values()
        )
        and isinstance(predicted_maxima, dict)
        and isinstance(continuous_limits, dict)
        and isinstance(short_peak_limits, dict)
        and set(gravity_maxima) == set(MOTOR_NAMES)
        and set(predicted_maxima) == set(MOTOR_NAMES)
        and set(continuous_limits) == set(MOTOR_NAMES)
        and set(short_peak_limits) == set(MOTOR_NAMES)
        and all(
            type(gravity_maxima[name]) in {int, float}
            and math.isfinite(float(gravity_maxima[name]))
            and float(gravity_maxima[name]) >= 0.0
            and type(predicted_maxima[name]) in {int, float}
            and math.isfinite(float(predicted_maxima[name]))
            and float(predicted_maxima[name]) >= 0.0
            and type(continuous_limits[name]) in {int, float}
            and math.isfinite(float(continuous_limits[name]))
            and float(continuous_limits[name]) > 0.0
            and type(short_peak_limits[name]) in {int, float}
            and math.isfinite(float(short_peak_limits[name]))
            and float(short_peak_limits[name]) > 0.0
            for name in MOTOR_NAMES
        )
    )
    reproduced_continuous_margin = (
        min(
            float(continuous_limits[name]) - float(gravity_maxima[name])
            for name in MOTOR_NAMES
        ) if exact_named_loads else None
    )
    reproduced_short_peak_margin = (
        min(
            float(short_peak_limits[name]) - float(predicted_maxima[name])
            for name in MOTOR_NAMES
        ) if exact_named_loads else None
    )
    reproduced_predicted_continuous_margin = (
        min(
            float(continuous_limits[name]) - float(predicted_maxima[name])
            for name in MOTOR_NAMES
        ) if exact_named_loads else None
    )
    reproduced_minimum_margin = (
        min(reproduced_continuous_margin, reproduced_short_peak_margin)
        if reproduced_continuous_margin is not None
        and reproduced_short_peak_margin is not None
        else None
    )
    return bool(
        proof.get("schema") == PLANNED_FEASIBILITY_SCHEMA
        and proof.get("source") == "whole_arm_gravity_node"
        and proof.get("source_instance_id") == status.get("source_instance_id")
        and type(proof.get("sequence")) is int
        and proof["sequence"] > 0
        and proof.get("sequence") == status.get("sequence")
        and type(source_ns) is int
        and source_ns == status.get("source_monotonic_ns")
        and 0 < source_ns <= checked_ns
        and checked_ns - source_ns <= int(GRAVITY_STATUS_TIMEOUT_S * 1.0e9)
        and proof.get("result") == "PASS"
        and proof.get("load_feasibility") == "PASS"
        and proof.get("thermal_feasibility") == "PASS"
        and proof.get("current_temperature_margin_result") == "PASS"
        and proof.get("trajectory_sha256") == trajectory_sha256
        and isinstance(trajectory_sha256, str)
        and len(trajectory_sha256) == 64
        and all(character in "0123456789abcdef"
                for character in trajectory_sha256)
        and proof.get("session_id") == session_id
        and proof.get("state_instance_id") == state_instance_id
        and proof.get("model_sha256") == PRODUCTION_MODEL_SHA256
        and proof.get("gravity_config_sha256") == GRAVITY_CONFIG_SHA256
        and proof.get("thermal_config_sha256") == THERMAL_CONFIG_SHA256
        and isinstance(request_sha256, str)
        and len(request_sha256) == 64
        and all(character in "0123456789abcdef" for character in request_sha256)
        and proof.get("continuous_rotor_limits_authoritative") is True
        and proof.get("temperature_limits_authoritative") is True
        and proof.get("load_evaluation_basis")
        == "MUJOCO_QFRC_BIAS_CONTINUOUS_PLUS_MJ_INVERSE_SHORT_PEAK_EVERY_SAMPLE"
        and proof.get("thermal_evaluation_basis")
        == (
            "CURRENT_MEASURED_TEMPERATURE_TO_DERATING_THRESHOLD_PLUS_"
            "ALL_SAMPLE_PREDICTED_LOAD_WITHIN_CONTINUOUS_RATING_"
            "NO_HEAT_RISE_MODEL"
        )
        and type(sample_count) is int
        and sample_count > 0
        and evaluated_sample_count == sample_count
        and exact_named_loads
        and type(proof.get("minimum_continuous_rotor_torque_margin_nm"))
        in {int, float}
        and math.isfinite(float(
            proof["minimum_continuous_rotor_torque_margin_nm"]
        ))
        and type(proof.get("minimum_short_peak_rotor_torque_margin_nm"))
        in {int, float}
        and math.isfinite(float(
            proof["minimum_short_peak_rotor_torque_margin_nm"]
        ))
        and type(proof.get(
            "minimum_predicted_continuous_rotor_torque_margin_nm"
        )) in {int, float}
        and math.isfinite(float(
            proof["minimum_predicted_continuous_rotor_torque_margin_nm"]
        ))
        and reproduced_continuous_margin is not None
        and reproduced_short_peak_margin is not None
        and abs(
            float(proof["minimum_continuous_rotor_torque_margin_nm"])
            - reproduced_continuous_margin
        ) <= 1.0e-12
        and abs(
            float(proof["minimum_short_peak_rotor_torque_margin_nm"])
            - reproduced_short_peak_margin
        ) <= 1.0e-12
        and reproduced_continuous_margin > 0.0
        and reproduced_short_peak_margin > 0.0
        and reproduced_predicted_continuous_margin is not None
        and reproduced_predicted_continuous_margin > 0.0
        and abs(float(
            proof["minimum_predicted_continuous_rotor_torque_margin_nm"]
        ) - reproduced_predicted_continuous_margin) <= 1.0e-12
        and type(minimum_load_margin) in {int, float}
        and math.isfinite(float(minimum_load_margin))
        and float(minimum_load_margin) > 0.0
        and reproduced_minimum_margin is not None
        and abs(float(minimum_load_margin) - reproduced_minimum_margin) <= 1.0e-12
        and type(minimum_thermal_margin) in {int, float}
        and math.isfinite(float(minimum_thermal_margin))
        and float(minimum_thermal_margin) > 0.0
    )


def planned_path_preview_display(
    status: object,
    *,
    trajectory_sha256: Optional[str],
    session_id: Optional[str],
    state_instance_id: Optional[str],
    limits_pass: bool,
    collision_result: str,
    now_monotonic_ns: Optional[int] = None,
) -> tuple[str, str, str]:
    """Format the required read-only whole-path preview evidence summary."""

    limit_text = "PASS" if limits_pass else "NOT_EVALUATED"
    if collision_result not in {"PASS", "CHECKING", "FAIL", "NOT_EVALUATED"}:
        collision_result = "NOT_EVALUATED"
    if not isinstance(status, dict):
        return (
            "等待整轨重力证明",
            "等待整轨电机负载证明",
            f"限位 {limit_text} ｜ 碰撞 {collision_result} ｜ 热负载 NOT_EVALUATED",
        )
    proof = status.get("planned_trajectory_feasibility")
    if (
        not isinstance(proof, dict)
        or proof.get("trajectory_sha256") != trajectory_sha256
        or proof.get("schema") != PLANNED_FEASIBILITY_SCHEMA
        or proof.get("source") != "whole_arm_gravity_node"
        or proof.get("source_instance_id") != status.get("source_instance_id")
        or proof.get("sequence") != status.get("sequence")
        or proof.get("source_monotonic_ns")
        != status.get("source_monotonic_ns")
        or proof.get("session_id") != session_id
        or proof.get("state_instance_id") != state_instance_id
        or status.get("session_id") != session_id
        or status.get("state_instance_id") != state_instance_id
    ):
        return (
            "等待与当前会话和轨迹绑定的重力证明",
            "等待与当前会话和轨迹绑定的电机负载证明",
            f"限位 {limit_text} ｜ 碰撞 {collision_result} ｜ 热负载 NOT_EVALUATED",
        )
    checked_ns = (
        time.monotonic_ns()
        if now_monotonic_ns is None else now_monotonic_ns
    )
    source_ns = proof.get("source_monotonic_ns")
    if (
        type(source_ns) is not int
        or source_ns <= 0
        or source_ns > checked_ns
        or checked_ns - source_ns
        > int(GRAVITY_STATUS_TIMEOUT_S * 1.0e9)
    ):
        return (
            "整轨重力证明已过期（STALE）",
            "整轨电机负载证明已过期（STALE）",
            f"限位 {limit_text} ｜ 碰撞 {collision_result} ｜ "
            "负载 STALE ｜ 热负载 STALE",
        )
    joint = proof.get("maximum_abs_gravity_joint_torque_nm_by_joint")
    if isinstance(joint, dict):
        joint_values = {
            name: float(value)
            for name, value in joint.items()
            if name in {"J1", "J2", "J3", "J4", "J5", "J6"}
            and type(value) in {int, float}
            and math.isfinite(float(value))
            and float(value) >= 0.0
        }
    else:
        joint_values = {}
    if len(joint_values) == 6:
        gravity_name, gravity_value = max(
            joint_values.items(), key=lambda item: item[1]
        )
        gravity_text = f"{gravity_name} {gravity_value:.3f} N·m（qfrc_bias）"
    else:
        gravity_text = "正在计算逐样本 qfrc_bias"
    motor = proof.get("maximum_abs_predicted_rotor_torque_nm_by_motor")
    if isinstance(motor, dict):
        motor_values = {
            name: float(value)
            for name, value in motor.items()
            if name in MOTOR_NAMES
            and type(value) in {int, float}
            and math.isfinite(float(value))
            and float(value) >= 0.0
        }
    else:
        motor_values = {}
    if motor_values:
        motor_name, motor_value = max(
            motor_values.items(), key=lambda item: item[1]
        )
        missing = [name for name in MOTOR_NAMES if name not in motor_values]
        motor_text = f"{motor_name} {motor_value:.3f} N·m（mj_inverse转子侧）"
        if missing:
            motor_text += "；" + "/".join(missing) + "映射未授权"
    else:
        motor_text = "正在计算逐样本 mj_inverse 电机力矩"
    load_result = str(proof.get("load_feasibility", "NOT_EVALUATED"))
    thermal_result = str(proof.get("thermal_feasibility", "NOT_EVALUATED"))
    blocker = proof.get("blocker_code")
    checks = (
        f"限位 {limit_text} ｜ 碰撞 {collision_result} ｜ "
        f"负载 {load_result} ｜ 热负载 {thermal_result}"
    )
    if isinstance(blocker, str) and blocker:
        checks += f" ｜ {blocker}"
    return gravity_text, motor_text, checks


def gravity_status_authorizes_hardware(
    status: object,
    *,
    session_id: str,
    state_instance_id: str,
    now_monotonic_ns: Optional[int] = None,
) -> bool:
    """Validate the fail-closed status consumed by GUI and Router."""

    if not isinstance(status, dict):
        return False
    checked_ns = (
        time.monotonic_ns()
        if now_monotonic_ns is None else now_monotonic_ns
    )
    feedforward = status.get("feedforward_nm")
    gravity = status.get("gravity_joint_nm")
    scale = status.get("gravity_scale")
    target = status.get("gravity_scale_target")
    source_ns = status.get("source_monotonic_ns")
    hardware_source_ns = status.get("hardware_state_source_monotonic_ns")
    continuous_authoritative = status.get(
        "continuous_rotor_limits_authoritative"
    )
    empirical = status.get("empirical_validation")
    empirical_expiry_valid = False
    empirical_stage_valid = False
    empirical_stage_level = None
    if isinstance(empirical, dict):
        expires_text = empirical.get("expires_at_utc")
        if isinstance(expires_text, str) and expires_text.endswith("Z"):
            try:
                expires_timestamp = datetime.fromisoformat(
                    expires_text[:-1] + "+00:00"
                ).astimezone(timezone.utc).timestamp()
                remaining_seconds = expires_timestamp - time.time()
                empirical_expiry_valid = bool(
                    math.isfinite(expires_timestamp)
                    and 0.0 < remaining_seconds <= 4200.0
                )
            except (OverflowError, ValueError):
                empirical_expiry_valid = False
        stage_index = empirical.get("stage_index")
        stage_level = empirical.get("stage_level")
        phase = empirical.get("phase")
        stage_complete = empirical.get("stage_complete")
        position_authorized = empirical.get(
            "position_validation_authorized"
        )
        if (
            type(stage_index) is int
            and 0 <= stage_index < len(GRAVITY_SCALE_LEVELS)
            and type(stage_level) in {int, float}
            and math.isfinite(float(stage_level))
            and abs(
                float(stage_level) - GRAVITY_SCALE_LEVELS[stage_index]
            ) <= 1.0e-12
            and type(stage_complete) is bool
            and type(position_authorized) is bool
        ):
            empirical_stage_level = float(stage_level)
            empirical_stage_valid = bool(
                (
                    position_authorized
                    and phase == "POSITION_VALIDATION"
                    and stage_index == len(GRAVITY_SCALE_LEVELS) - 1
                    and stage_complete
                    and empirical.get("blocker") is None
                )
                or (
                    not position_authorized
                    and (
                        phase == "GRAVITY_LADDER"
                        or (
                            phase
                            == "POSITION_VALIDATION_PENDING_RECONFIRMATION"
                            and stage_index
                            == len(GRAVITY_SCALE_LEVELS) - 1
                            and stage_complete
                        )
                    )
                )
            )
    official_authority_valid = bool(
        continuous_authoritative is True
        and status.get("empirical_validation_authoritative") is not True
        and status.get("torque_authority_class")
        != EMPIRICAL_AUTHORITY_CLASS
    )
    empirical_authority_valid = bool(
        continuous_authoritative is False
        and status.get("empirical_validation_authoritative") is True
        and status.get("torque_authority_class")
        == EMPIRICAL_AUTHORITY_CLASS
        and isinstance(empirical, dict)
        and empirical.get("authority_class") == EMPIRICAL_AUTHORITY_CLASS
        and empirical.get("rating_classification")
        == EMPIRICAL_RATING_CLASSIFICATION
        and isinstance(empirical.get("envelope_id"), str)
        and len(empirical["envelope_id"]) == 38
        and empirical["envelope_id"].startswith("v15-31b-empirical-")
        and isinstance(empirical.get("envelope_sha256"), str)
        and len(empirical["envelope_sha256"]) == 64
        and all(
            character in "0123456789abcdef"
            for character in empirical["envelope_sha256"]
        )
        and isinstance(empirical.get("anchor_sha256"), str)
        and len(empirical["anchor_sha256"]) == 64
        and all(
            character in "0123456789abcdef"
            for character in empirical["anchor_sha256"]
        )
        and empirical_expiry_valid
        and empirical_stage_valid
        and empirical.get("invalidated") is False
        and empirical.get("continuous_operation_authorized") is False
        and empirical.get("official_continuous_rating_claimed") is False
        and type(empirical.get("maximum_position_segment_seconds"))
        in {int, float}
        and math.isfinite(float(
            empirical["maximum_position_segment_seconds"]
        ))
        and float(empirical["maximum_position_segment_seconds"]) == 15.0
        and type(empirical.get("maximum_abs_position_segment_deg"))
        in {int, float}
        and math.isfinite(float(
            empirical["maximum_abs_position_segment_deg"]
        ))
        and float(empirical["maximum_abs_position_segment_deg"]) == 5.0
        and type(empirical.get(
            "maximum_cumulative_position_trajectory_seconds"
        )) in {int, float}
        and math.isfinite(float(
            empirical["maximum_cumulative_position_trajectory_seconds"]
        ))
        and float(
            empirical["maximum_cumulative_position_trajectory_seconds"]
        ) == 600.0
    )
    return bool(
        status.get("schema") == GRAVITY_STATUS_SCHEMA
        and status.get("source") == "whole_arm_gravity_node"
        and isinstance(status.get("source_instance_id"), str)
        and len(status["source_instance_id"]) == 32
        and all(character in "0123456789abcdef"
                for character in status["source_instance_id"])
        and type(status.get("sequence")) is int
        and status["sequence"] > 0
        and type(source_ns) is int
        and 0 < source_ns <= checked_ns
        and checked_ns - source_ns <= int(GRAVITY_STATUS_TIMEOUT_S * 1.0e9)
        and type(status.get("hardware_state_sequence")) is int
        and status["hardware_state_sequence"] > 0
        and type(hardware_source_ns) is int
        and 0 < hardware_source_ns <= source_ns
        and source_ns - hardware_source_ns <= HARDWARE_STATE_SOURCE_MAX_AGE_NS
        and isinstance(status.get("q_actual_sha256"), str)
        and len(status["q_actual_sha256"]) == 64
        and all(character in "0123456789abcdef"
                for character in status["q_actual_sha256"])
        and status.get("session_id") == session_id
        and status.get("state_instance_id") == state_instance_id
        and status.get("anchor_valid") is True
        and status.get("production_model_hash_match") is True
        and status.get("model_sha256") == PRODUCTION_MODEL_SHA256
        and status.get("gravity_config_sha256") == GRAVITY_CONFIG_SHA256
        and status.get("finite_bounded") is True
        and status.get("pose_feasibility") == "PASS"
        and status.get("blocker") is None
        and status.get("hardware_enable_requested") is True
        and (official_authority_valid or empirical_authority_valid)
        and status.get("actuation_interface_present") is True
        and isinstance(status.get("last_update_age_s"), (int, float))
        and math.isfinite(float(status["last_update_age_s"]))
        and 0.0 <= float(status["last_update_age_s"]) <= 0.25
        and isinstance(gravity, (list, tuple))
        and len(gravity) == 6
        and all(type(value) in {int, float} and math.isfinite(float(value))
                for value in gravity)
        and isinstance(feedforward, (list, tuple))
        and len(feedforward) == 6
        and all(type(value) in {int, float} and math.isfinite(float(value))
                for value in feedforward)
        and all(abs(float(value)) <= limit + 1.0e-12
                for value, limit in zip(
                    feedforward, GRAVITY_ROTOR_FEEDFORWARD_LIMIT_NM
                ))
        and abs(float(feedforward[5])) <= 1.0e-12
        and type(scale) in {int, float}
        and math.isfinite(float(scale))
        and 0.0 <= float(scale) <= 1.0
        and type(target) in {int, float}
        and math.isfinite(float(target))
        and any(abs(float(target) - level) <= 1.0e-12
                for level in GRAVITY_SCALE_LEVELS)
        and (
            not empirical_authority_valid
            or (
                empirical_stage_level is not None
                and abs(float(target) - empirical_stage_level) <= 1.0e-12
            )
        )
        and status.get("hardware_tff_enabled") is bool(float(target) > 0.0)
    )


def fixed_hold_targets_after_position_stop(
    command_targets: list[float],
    actual: list[float],
    active_joint_mask: list[bool],
    moving_joint_mask: list[bool],
) -> list[float]:
    """Stop moving axes at feedback while preserving every fixed HOLD target."""

    if not all(
        len(values) == 6
        for values in (
            command_targets, actual, active_joint_mask, moving_joint_mask,
        )
    ):
        raise ValueError("position-stop vectors must contain six joints")
    return [
        locked if active and not moving else measured
        for locked, measured, active, moving in zip(
            command_targets, actual, active_joint_mask, moving_joint_mask
        )
    ]


def parse_pose_degrees(value: str) -> np.ndarray:
    fields = [field.strip() for field in value.split(",")]
    if len(fields) != 6:
        raise ValueError("MuJoCo绝对姿态必须包含六个逗号分隔角度")
    pose = np.radians(np.asarray([float(field) for field in fields], dtype=float))
    if not np.all(np.isfinite(pose)):
        raise ValueError("MuJoCo绝对姿态包含无效数值")
    return pose


GUI_VISUAL_LINKS = (
    "base_link", "link1", "link2", "link3",
    "link4", "link5", "link6", "gripper",
)
GUI_VISUAL_RGBA = {
    "base_link": "0.22 0.31 0.62 1",
    "link1": "0.48 0.20 0.52 1",
    "link2": "0.55 0.16 0.22 1",
    "link3": "0.55 0.16 0.22 1",
    "link4": "0.10 0.10 0.12 1",
    "link5": "0.10 0.10 0.12 1",
    "link6": "0.15 0.20 0.55 1",
    "gripper": "0.20 0.55 0.68 1",
}
GUI_STL_TRIANGLES_PER_CHUNK = 190_000


def _split_binary_stl(raw: bytes, source: Path) -> list[bytes]:
    """Split an audited binary STL without changing a vertex or normal."""
    if len(raw) < 84:
        raise ValueError(f"GUI可视网格过短：{source}")
    triangle_count = struct.unpack_from("<I", raw, 80)[0]
    expected_size = 84 + 50 * triangle_count
    if triangle_count < 1 or len(raw) != expected_size:
        raise ValueError(f"GUI可视网格不是有效的二进制STL：{source}")
    chunks = []
    for start in range(0, triangle_count, GUI_STL_TRIANGLES_PER_CHUNK):
        count = min(GUI_STL_TRIANGLES_PER_CHUNK, triangle_count - start)
        first = 84 + 50 * start
        last = first + 50 * count
        chunks.append(raw[:80] + struct.pack("<I", count) + raw[first:last])
    return chunks


def load_gui_visual_model(model_path: Path) -> mujoco.MjModel:
    """Load a low-memory display model while preserving the frozen kinematics.

    The production XML contains 1008 full collision/render meshes and needs
    several gigabytes per process.  The embedded GUI only needs link visuals,
    so it reuses the already-audited RViz LOD meshes, splits them below
    MuJoCo's per-STL face limit in memory, and keeps every body/joint transform
    from the frozen XML.  The production XML and collision assets are never
    modified.
    """
    tree = ET.parse(model_path)
    root = tree.getroot()
    asset = root.find("asset")
    if asset is None:
        raise ValueError("冻结MuJoCo模型缺少asset节点")
    for child in list(asset):
        if child.tag == "mesh":
            asset.remove(child)

    repository_root = model_path.parents[2]
    visual_directory = (
        repository_root / "mujoco_kinematic_v1" / "meshes" / "ros_visual_mm"
    )
    in_memory_assets: dict[str, bytes] = {}
    body_meshes: dict[str, list[str]] = {}
    for link in GUI_VISUAL_LINKS:
        source = visual_directory / f"{link}.stl"
        if not source.is_file():
            raise FileNotFoundError(f"GUI验收可视网格不存在：{source}")
        mesh_names = []
        for index, chunk in enumerate(_split_binary_stl(source.read_bytes(), source), 1):
            file_name = f"gui_{link}_{index:03d}.stl"
            mesh_name = file_name.removesuffix(".stl")
            in_memory_assets[file_name] = chunk
            ET.SubElement(
                asset,
                "mesh",
                name=mesh_name,
                file=file_name,
                scale="0.001 0.001 0.001",
            )
            mesh_names.append(mesh_name)
        body_meshes[link] = mesh_names

    for parent in root.iter():
        for child in list(parent):
            if child.tag == "geom" and child.get("mesh"):
                parent.remove(child)
    contact = root.find("contact")
    if contact is not None:
        root.remove(contact)

    found_links = set()
    for body in root.iter("body"):
        link = body.get("name", "")
        if link not in body_meshes:
            continue
        found_links.add(link)
        for index, mesh_name in enumerate(body_meshes[link], 1):
            ET.SubElement(
                body,
                "geom",
                name=f"gui_visual__{link}__{index:03d}",
                type="mesh",
                mesh=mesh_name,
                rgba=GUI_VISUAL_RGBA[link],
                contype="0",
                conaffinity="0",
                group="1",
                mass="0",
            )
    missing_links = set(GUI_VISUAL_LINKS) - found_links
    if missing_links:
        raise ValueError(
            "冻结MuJoCo模型缺少GUI连杆：" + ", ".join(sorted(missing_links))
        )
    xml = ET.tostring(root, encoding="unicode")
    return mujoco.MjModel.from_xml_string(xml, assets=in_memory_assets)


class EmbeddedMujocoPreview(QGroupBox):
    """Render one twin without running MuJoCo work on the Qt event thread."""

    render_completed = Signal(object)
    render_failed = Signal(str)

    def __init__(
        self, model_path: Path, session_pose_deg: str,
        title: str = "MuJoCo实时三维预览",
    ) -> None:
        super().__init__(title)
        layout = QVBoxLayout(self)
        self.image = QLabel("正在初始化MuJoCo内嵌渲染…")
        self.image.setAlignment(Qt.AlignCenter)
        self.image.setMinimumSize(420, 360)
        self.image.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.image.setStyleSheet("background: #111820; color: #cfd8dc;")
        self.image.setMouseTracking(True)
        self.image.installEventFilter(self)
        self.camera_help = QLabel(
            "鼠标左键拖动旋转｜右键拖动平移｜滚轮缩放｜双击自动适应全机"
        )
        self.camera_help.setAlignment(Qt.AlignCenter)
        camera_buttons = QHBoxLayout()
        for text, callback in (
            ("适应全机", self.fit_camera),
            ("正视", lambda: self.set_camera_view(90.0, 0.0)),
            ("侧视", lambda: self.set_camera_view(0.0, 0.0)),
            ("俯视", lambda: self.set_camera_view(90.0, -89.0)),
            ("复位视角", self.reset_camera),
        ):
            button = QPushButton(text)
            button.clicked.connect(callback)
            camera_buttons.addWidget(button)
        self.pose_text = QLabel()
        self.pose_text.setAlignment(Qt.AlignCenter)
        self.pose_text.setWordWrap(True)
        layout.addWidget(self.image, 1)
        layout.addLayout(camera_buttons)
        layout.addWidget(self.camera_help)
        layout.addWidget(self.pose_text)

        if not model_path.is_file():
            raise FileNotFoundError(f"MuJoCo模型不存在：{model_path}")
        self.session_pose = parse_pose_degrees(session_pose_deg)
        self.model = load_gui_visual_model(model_path)
        self.data = mujoco.MjData(self.model)
        addresses = []
        for name in ("J1", "J2", "J3", "J4", "J5", "J6"):
            joint_id = mujoco.mj_name2id(
                self.model, mujoco.mjtObj.mjOBJ_JOINT, name
            )
            if joint_id < 0:
                raise ValueError(f"MuJoCo模型缺少关节：{name}")
            addresses.append(int(self.model.jnt_qposadr[joint_id]))
        self.qpos_addresses = np.asarray(addresses, dtype=int)
        # The renderer owns an OpenGL context, so it is created, used and
        # destroyed on this preview's one dedicated worker thread.  Requests
        # are coalesced: a slow frame can never build an unbounded queue or
        # stall the GUI refresh timer.
        self.renderer = None
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        self.camera.lookat[:] = self.model.stat.center
        self.camera.distance = max(0.5, 1.65 * float(self.model.stat.extent))
        self.camera.azimuth = 135.0
        self.camera.elevation = -22.0
        self.camera_state = {
            "lookat": tuple(float(item) for item in self.camera.lookat),
            "distance": float(self.camera.distance),
            "azimuth": float(self.camera.azimuth),
            "elevation": float(self.camera.elevation),
        }
        self.last_relative: Optional[np.ndarray] = None
        self.last_image: Optional[QImage] = None
        self.mouse_position = None
        self._render_lock = threading.Lock()
        self._render_executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="m8010-mujoco-preview",
        )
        self._render_in_flight = False
        self._pending_render = None
        self._render_sequence = 0
        self._latest_requested_sequence = 0
        self._renderer_closed = False
        self.render_completed.connect(self._apply_render_result)
        self.render_failed.connect(self._show_render_error)
        self.set_relative_pose([0.0] * 6, force=True, fit=True)

    def set_relative_pose(
        self, relative_rad, force: bool = False, fit: bool = False,
    ) -> None:
        relative = np.asarray(relative_rad, dtype=float)
        if relative.shape != (6,) or not np.all(np.isfinite(relative)):
            return
        if not force and self.last_relative is not None and np.allclose(
            relative, self.last_relative, atol=1.0e-7, rtol=0.0
        ):
            return
        absolute = self.session_pose + relative
        absolute_deg = np.degrees(absolute)
        self.pose_text.setText(
            "绝对姿态：" + "　".join(
                f"J{index + 1}={value:+.2f}°"
                for index, value in enumerate(absolute_deg)
            )
        )
        self.last_relative = relative.copy()
        self._schedule_render(relative, fit=fit or self.last_image is None)

    def _render_scene(self) -> None:
        relative = (
            np.zeros(6, dtype=float)
            if self.last_relative is None else self.last_relative.copy()
        )
        self._schedule_render(relative, fit=False)

    def _schedule_render(self, relative: np.ndarray, *, fit: bool) -> None:
        with self._render_lock:
            if self._renderer_closed:
                return
            self._render_sequence += 1
            request = {
                "sequence": self._render_sequence,
                "relative": np.asarray(relative, dtype=float).copy(),
                "fit": bool(fit),
                "camera": {
                    "lookat": tuple(self.camera_state["lookat"]),
                    "distance": float(self.camera_state["distance"]),
                    "azimuth": float(self.camera_state["azimuth"]),
                    "elevation": float(self.camera_state["elevation"]),
                },
            }
            self._latest_requested_sequence = request["sequence"]
            if self._render_in_flight:
                self._pending_render = request
                return
            self._render_in_flight = True
            future = self._render_executor.submit(self._render_request, request)
            future.add_done_callback(self._render_future_done)

    def _render_request(self, request: dict) -> dict:
        if self.renderer is None:
            self.renderer = mujoco.Renderer(self.model, height=420, width=520)
        relative = request["relative"]
        absolute = self.session_pose + relative
        self.data.qpos[self.qpos_addresses] = absolute
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        camera = request["camera"]
        self.camera.lookat[:] = camera["lookat"]
        self.camera.distance = camera["distance"]
        self.camera.azimuth = camera["azimuth"]
        self.camera.elevation = camera["elevation"]
        if request["fit"]:
            self._fit_camera_worker()
        self.renderer.update_scene(self.data, camera=self.camera)
        pixels = np.ascontiguousarray(self.renderer.render())
        height, width, channels = pixels.shape
        if channels != 3:
            raise RuntimeError("MuJoCo渲染图像通道数异常")
        image = QImage(
            pixels.data, width, height, width * channels, QImage.Format_RGB888
        ).copy()
        return {
            "sequence": request["sequence"],
            "image": image,
            "camera": {
                "lookat": tuple(float(item) for item in self.camera.lookat),
                "distance": float(self.camera.distance),
                "azimuth": float(self.camera.azimuth),
                "elevation": float(self.camera.elevation),
            },
        }

    def _render_future_done(self, future) -> None:
        try:
            result = future.result()
        except Exception as exc:  # delivered to the Qt thread via a signal
            self.render_failed.emit(str(exc))
        else:
            self.render_completed.emit(result)
        with self._render_lock:
            next_request = None if self._renderer_closed else self._pending_render
            self._pending_render = None
            if next_request is None:
                self._render_in_flight = False
                return
            next_future = self._render_executor.submit(
                self._render_request, next_request
            )
            next_future.add_done_callback(self._render_future_done)

    def _apply_render_result(self, result: object) -> None:
        if not isinstance(result, dict):
            return
        sequence = result.get("sequence")
        image = result.get("image")
        camera = result.get("camera")
        if (
            type(sequence) is not int
            or sequence != self._latest_requested_sequence
            or not isinstance(image, QImage)
            or not isinstance(camera, dict)
        ):
            return
        self.camera_state = camera
        self.last_image = image
        self._update_pixmap()

    def _show_render_error(self, detail: str) -> None:
        if not self._renderer_closed:
            self.image.setText(f"MuJoCo渲染失败：{detail}")

    def _update_pixmap(self) -> None:
        image = getattr(self, "last_image", None)
        if image is None:
            return
        available = self.image.size()
        pixmap = QPixmap.fromImage(image).scaled(
            max(1, available.width()), max(1, available.height()),
            Qt.KeepAspectRatio, Qt.SmoothTransformation,
        )
        self.image.setPixmap(pixmap)

    def _fit_camera_worker(self) -> None:
        plane = int(mujoco.mjtGeom.mjGEOM_PLANE)
        indices = np.flatnonzero(self.model.geom_type != plane)
        if indices.size:
            positions = np.asarray(self.data.geom_xpos[indices], dtype=float)
            radii = np.asarray(self.model.geom_rbound[indices], dtype=float)
            radii = np.maximum(radii, 0.005)
            lower = np.min(positions - radii[:, None], axis=0)
            upper = np.max(positions + radii[:, None], axis=0)
            self.camera.lookat[:] = 0.5 * (lower + upper)
            radius = max(0.1, 0.5 * float(np.linalg.norm(upper - lower)))
            self.camera.distance = max(0.35, 3.0 * radius)
        else:
            self.camera.lookat[:] = self.model.stat.center
            self.camera.distance = max(0.5, 2.4 * float(self.model.stat.extent))

    def fit_camera(self, _checked: bool = False, render: bool = True) -> None:
        if render:
            relative = (
                np.zeros(6, dtype=float)
                if self.last_relative is None else self.last_relative.copy()
            )
            self._schedule_render(relative, fit=True)

    def reset_camera(self, _checked: bool = False) -> None:
        self.camera_state["azimuth"] = 135.0
        self.camera_state["elevation"] = -22.0
        self.fit_camera(render=True)

    def set_camera_view(self, azimuth: float, elevation: float) -> None:
        self.camera_state["azimuth"] = float(azimuth)
        self.camera_state["elevation"] = float(elevation)
        self.fit_camera(render=True)

    def eventFilter(self, watched, event) -> bool:
        if watched is self.image:
            if event.type() == QEvent.MouseButtonDblClick:
                self.fit_camera(render=True)
                return True
            if event.type() == QEvent.MouseButtonPress:
                self.mouse_position = event.position()
                return True
            if event.type() == QEvent.MouseMove and self.mouse_position is not None:
                position = event.position()
                delta = position - self.mouse_position
                self.mouse_position = position
                buttons = event.buttons()
                if buttons & Qt.LeftButton:
                    self.camera_state["azimuth"] += 0.45 * delta.x()
                    self.camera_state["elevation"] = float(np.clip(
                        self.camera_state["elevation"] - 0.45 * delta.y(),
                        -89.0, 89.0,
                    ))
                    self._render_scene()
                    return True
                if buttons & (Qt.RightButton | Qt.MiddleButton):
                    scale = max(
                        1.0e-4, self.camera_state["distance"] * 0.0015
                    )
                    angle = math.radians(self.camera_state["azimuth"])
                    right = np.array([math.cos(angle), math.sin(angle), 0.0])
                    lookat = np.asarray(self.camera_state["lookat"], dtype=float)
                    lookat -= delta.x() * scale * right
                    lookat[2] += delta.y() * scale
                    self.camera_state["lookat"] = tuple(
                        float(item) for item in lookat
                    )
                    self._render_scene()
                    return True
            if event.type() == QEvent.MouseButtonRelease:
                self.mouse_position = None
                return True
            if event.type() == QEvent.Wheel:
                steps = event.angleDelta().y() / 120.0
                self.camera_state["distance"] = float(np.clip(
                    self.camera_state["distance"] * math.exp(-0.12 * steps),
                    0.08, 20.0,
                ))
                self._render_scene()
                return True
        return super().eventFilter(watched, event)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._update_pixmap()

    def close_renderer(self) -> None:
        with self._render_lock:
            if self._renderer_closed:
                return
            self._renderer_closed = True
            self._pending_render = None

        def close_on_worker() -> None:
            if self.renderer is not None:
                self.renderer.close()

        # Context destruction happens on the same thread as render creation.
        self._render_executor.submit(close_on_worker)
        self._render_executor.shutdown(wait=True, cancel_futures=False)


def receipt_is_fresh(receipt: float, now: float, timeout_s: float) -> bool:
    age = now - receipt
    return receipt > 0.0 and 0.0 <= age <= timeout_s


def mujoco_status_text(status: Optional[dict], fresh: bool) -> str:
    if status is None:
        return "等待"
    if not fresh:
        return "数据过期"
    result = status.get("direct_qpos_numeric_result")
    if result == "PASS":
        return "正常"
    if result == "NOT_EVALUATED":
        return "未验证"
    return "异常"


def command_router_status_text(
    status: dict | None,
    fresh: bool,
    lease_ms: float,
    expected_mode: str,
) -> tuple[str, str]:
    """Format router acknowledgement without treating a request as confirmation."""
    if status is None:
        return "收到=等待，拒绝=等待，命令年龄=等待，最近模式=等待", "critical"

    received = status.get("received") is True
    try:
        rejected = int(status.get("rejected_commands", 0))
        if rejected < 0:
            raise ValueError
    except (TypeError, ValueError):
        rejected = None

    raw_age = status.get("last_command_age_ms")
    try:
        age_ms = None if raw_age is None else float(raw_age)
        if age_ms is not None and (not math.isfinite(age_ms) or age_ms < 0.0):
            raise ValueError
    except (TypeError, ValueError):
        age_ms = None

    raw_rejection_age = status.get("last_rejection_age_ms")
    try:
        rejection_age_ms = (
            None if raw_rejection_age is None else float(raw_rejection_age)
        )
        if rejection_age_ms is not None and (
            not math.isfinite(rejection_age_ms) or rejection_age_ms < 0.0
        ):
            raise ValueError
    except (TypeError, ValueError):
        rejection_age_ms = None

    last_mode = status.get("last_mode")
    mode_text = str(last_mode) if last_mode in {"brake", "drag", "hold", "position"} else "等待"
    j2_forwarded_mode = status.get("j2_forwarded_mode")
    j2_forwarded_text = (
        str(j2_forwarded_mode)
        if j2_forwarded_mode in {"brake", "drag", "hold", "position"}
        else "等待"
    )
    active_joint_mask = status.get("last_active_joint_mask")
    j2_active_requested = bool(
        isinstance(active_joint_mask, list)
        and len(active_joint_mask) == 6
        and all(type(active) is bool for active in active_joint_mask)
        and active_joint_mask[1]
    )
    j2_forwarding_contradiction = bool(
        j2_active_requested
        and last_mode in {"hold", "position"}
        and j2_forwarded_mode == "brake"
    )
    rejected_text = "无效" if rejected is None else str(rejected)
    age_text = "等待" if age_ms is None else f"{age_ms:.0f}毫秒"
    rejection_age_text = (
        "无" if rejected == 0
        else "等待" if rejection_age_ms is None
        else f"{rejection_age_ms:.0f}毫秒前"
    )
    text = (
        f"收到={'是' if received else '否'}，拒绝={rejected_text}，"
        f"最近拒绝={rejection_age_text}，命令年龄={age_text}，最近模式={mode_text}，"
        f"J2转发模式={j2_forwarded_text}"
    )
    if j2_forwarding_contradiction:
        text += (
            f"；J2转发矛盾：已请求J2 {mode_text}主动控制，"
            "但路由向J2转发brake，J2未收到该保持请求"
        )

    if not fresh:
        return f"状态过期；{text}", "critical"
    if j2_forwarding_contradiction:
        return text, "critical"
    if rejected is None or not received or age_ms is None or age_ms > lease_ms:
        return text, "critical"
    if mode_text != expected_mode or (
        rejected > 0
        and rejection_age_ms is not None
        and rejection_age_ms <= ROUTER_REJECTION_WARNING_WINDOW_MS
    ):
        return text, "warning"
    return text, "normal"


def control_feedback_ready(
    have_first_state: bool,
    streams_fresh: bool,
    connected: list[bool],
    require_all: bool = False,
) -> bool:
    """Return whether an action has enough fresh, healthy joint feedback.

    Normal motion actions are isolated per joint by the hardware workers, so one
    healthy logical joint is sufficient.  Operations that save or command a
    complete six-joint pose explicitly opt into the all-joints requirement.
    """
    if not have_first_state or not streams_fresh or not connected:
        return False
    return all(connected) if require_all else any(connected)


def effective_active_joint_mask(
    requested: list[bool], connected: list[bool], mode: str
) -> list[bool]:
    """Return the per-joint activation mask sent to the hardware workers.

    A disconnected joint is removed from every active command before
    publication. Brake never carries an active bit.
    """
    if len(requested) != 6 or len(connected) != 6:
        raise ValueError("关节激活掩码必须包含六项")
    if mode == "brake":
        return [False] * 6
    return [bool(requested[index] and connected[index]) for index in range(6)]


def clear_mask_on_connection_loss(
    requested: list[bool], previous_connected: list[bool], connected: list[bool]
) -> list[bool]:
    """Latch an active request off after feedback health is lost.

    A later feedback recovery must never re-enable a joint without a fresh
    operator action.
    """
    if len(requested) != 6 or len(previous_connected) != 6 or len(connected) != 6:
        raise ValueError("关节连接状态必须包含六项")
    return [
        bool(requested[index] and not (
            previous_connected[index] and not connected[index]
        ))
        for index in range(6)
    ]


def run_ros_context_operation(context_ok, operation) -> bool:
    """Run one ROS operation, distinguishing shutdown from runtime failure.

    A ROS shutdown can race with either the preflight check or the operation
    itself.  Only an exception accompanied by an invalid context is treated as
    normal shutdown; exceptions while the context remains valid are re-raised.
    """
    if not context_ok():
        return False
    try:
        operation()
    except Exception:
        if context_ok():
            raise
        return False
    return context_ok()


@dataclass
class VirtualWidgets:
    value: QLabel
    target: QDoubleSpinBox
    # Optional keeps older logic-only fakes source-compatible; every real GUI
    # row owns this single operator-editable planned-target slider.
    slider: Optional[QSlider] = None


@dataclass
class RealWidgets:
    # The real side is an encoder table, never a second target slider set.
    slider: Optional[QSlider]
    actual: QLabel
    target: QLabel
    error: QLabel
    state: QLabel


class ArmGuiNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_control_gui")
        self.declare_parameter("config_path", "")
        self.declare_parameter("joint_limits_path", "")
        self.declare_parameter("initial_pose_path", "")
        self.declare_parameter("initial_pose_read_only", False)
        self.declare_parameter("log_directory", "logs/arm_gui")
        self.declare_parameter("embedded_model_path", "")
        self.declare_parameter("embedded_session_pose_deg", "0,0,0,0,0,0")
        self.declare_parameter("thermal_config_path", "")
        self.config_path = Path(str(self.get_parameter("config_path").value)).resolve()
        self.joint_limits_path = Path(
            str(self.get_parameter("joint_limits_path").value)
        ).resolve()
        self.initial_pose_path = Path(str(self.get_parameter("initial_pose_path").value)).resolve()
        self.initial_pose_read_only = strict_bool_parameter(
            self.get_parameter("initial_pose_read_only").value,
            "initial_pose_read_only",
        )
        self.log_directory = Path(str(self.get_parameter("log_directory").value)).resolve()
        self.embedded_model_path = Path(
            str(self.get_parameter("embedded_model_path").value)
        ).resolve()
        self.embedded_session_pose_deg = str(
            self.get_parameter("embedded_session_pose_deg").value
        )
        thermal_config_text = str(
            self.get_parameter("thermal_config_path").value
        )
        if not thermal_config_text:
            raise ValueError("thermal_config_path is required")
        self.thermal_limits = load_thermal_limits(
            Path(thermal_config_text).resolve()
        )
        self.command_publisher = self.create_publisher(String, "/whole_arm/gui_command", 10)
        self.target_publisher = self.create_publisher(Float64MultiArray, "/whole_arm/gui_targets", 10)
        self.mode_publisher = self.create_publisher(String, "/whole_arm/gui_mode", 10)
        self.collision_request_publisher = self.create_publisher(
            String, "/whole_arm/collision_guard_request", 10
        )
        self.planned_path_request_publisher = self.create_publisher(
            String, "/whole_arm/planned_path_feasibility_request", 10
        )
        self.latest_joint_state: Optional[JointState] = None
        self.latest_hardware: Optional[dict] = None
        self.latest_mujoco: Optional[dict] = None
        self.latest_control_status: Optional[dict] = None
        self.latest_gravity_status: Optional[dict] = None
        self.latest_collision_result: Optional[dict] = None
        self.last_joint_receipt = 0.0
        self.last_hardware_receipt = 0.0
        self.last_mujoco_receipt = 0.0
        self.last_control_status_receipt = 0.0
        self.last_gravity_status_receipt = 0.0
        self.last_collision_result_receipt = 0.0
        self.command_source_instance_id = secrets.token_hex(16)
        self.planned_path_request_sequence = 0
        self.hardware_sequences_by_instance: dict[str, int] = {}
        self.active_hardware_state_instance_id: Optional[str] = None
        self.active_hardware_source_received_ns: Optional[int] = None
        # These are latest-state streams rather than event logs.  Keep only the
        # newest sample so a temporarily busy Qt loop cannot replay stale UI
        # states after the hardware has already advanced.
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 1)
        self.create_subscription(String, "/whole_arm/hardware_state", self._hardware_callback, 1)
        self.create_subscription(String, "/whole_arm/mujoco_mirror_status", self._mujoco_callback, 10)
        self.create_subscription(
            String, "/whole_arm/control_status", self._control_status_callback, 10
        )
        self.create_subscription(
            String, "/whole_arm/gravity_status", self._gravity_status_callback, 10
        )
        self.create_subscription(
            String,
            "/whole_arm/collision_guard_result",
            self._collision_result_callback,
            10,
        )

    def _joint_callback(self, message: JointState) -> None:
        if (
            tuple(message.name) == JOINT_NAMES
            and len(message.position) == 6
            and all(math.isfinite(float(value)) for value in message.position)
        ):
            self.latest_joint_state = message
            self.last_joint_receipt = time.monotonic()

    def _hardware_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            now_ns = time.monotonic_ns()
            if (
                not hardware_state_contract_valid(value)
                or not hardware_state_source_is_fresh(value, now_ns)
            ):
                return
            source = value["state_instance_id"]
            sequence = value["sequence"]
            if (
                self.active_hardware_state_instance_id is not None
                and source != self.active_hardware_state_instance_id
                and self.active_hardware_source_received_ns is not None
                and now_ns - self.active_hardware_source_received_ns
                <= HARDWARE_STATE_SOURCE_TAKEOVER_TIMEOUT_NS
            ):
                return
            previous_sequence = self.hardware_sequences_by_instance.get(source)
            if previous_sequence is not None and sequence <= previous_sequence:
                return
            if (
                source not in self.hardware_sequences_by_instance
                and len(self.hardware_sequences_by_instance)
                >= MAX_TRACKED_STATE_SOURCES
            ):
                oldest_source = next(iter(self.hardware_sequences_by_instance))
                self.hardware_sequences_by_instance.pop(oldest_source)
            self.hardware_sequences_by_instance[source] = sequence
            self.active_hardware_state_instance_id = source
            self.active_hardware_source_received_ns = now_ns
            self.latest_hardware = value
            self.last_hardware_receipt = time.monotonic()
        except json.JSONDecodeError:
            pass

    def _mujoco_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                return
            self.latest_mujoco = value
            self.last_mujoco_receipt = time.monotonic()
        except json.JSONDecodeError:
            pass

    def _control_status_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if (
                not isinstance(value, dict)
                or value.get("schema") != "go-m8010-command-router-status/1.0"
            ):
                return
            self.latest_control_status = value
            self.last_control_status_receipt = time.monotonic()
        except json.JSONDecodeError:
            pass

    def _gravity_status_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if (
                not isinstance(value, dict)
                or value.get("schema") != GRAVITY_STATUS_SCHEMA
            ):
                return
            self.latest_gravity_status = value
            self.last_gravity_status_receipt = time.monotonic()
        except json.JSONDecodeError:
            pass

    def _collision_result_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if (
                not isinstance(value, dict)
                or value.get("schema")
                != "go-m8010-collision-guard-result/1.0"
            ):
                return
            self.latest_collision_result = value
            self.last_collision_result_receipt = time.monotonic()
        except json.JSONDecodeError:
            pass

    def control_streams_fresh(self, now: Optional[float] = None) -> bool:
        checked_at = time.monotonic() if now is None else now
        return receipt_is_fresh(
            self.last_joint_receipt, checked_at, CONTROL_STREAM_TIMEOUT_S
        ) and receipt_is_fresh(
            self.last_hardware_receipt, checked_at, CONTROL_STREAM_TIMEOUT_S
        )

    def mujoco_stream_fresh(self, now: Optional[float] = None) -> bool:
        checked_at = time.monotonic() if now is None else now
        return receipt_is_fresh(
            self.last_mujoco_receipt, checked_at, MUJOCO_STREAM_TIMEOUT_S
        )

    def control_status_fresh(self, now: Optional[float] = None) -> bool:
        checked_at = time.monotonic() if now is None else now
        return receipt_is_fresh(
            self.last_control_status_receipt, checked_at, ROUTER_STATUS_TIMEOUT_S
        )

    def gravity_status_fresh(self, now: Optional[float] = None) -> bool:
        checked_at = time.monotonic() if now is None else now
        return receipt_is_fresh(
            self.last_gravity_status_receipt,
            checked_at,
            GRAVITY_STATUS_TIMEOUT_S,
        )

    def publish_command(
        self, sequence: int, mode: str, command_targets: list[float],
        virtual_targets: list[float], active_joint_mask: list[bool],
        moving_joint_mask: list[bool],
        activation_epoch: int, config: dict,
        collision_guard_proof: Optional[dict] = None,
        trajectory_descriptor: Optional[dict] = None,
        plan_manifest: Optional[dict] = None,
    ) -> None:
        control = config["控制"]
        is_brake = mode == "brake"
        payload = {
            "schema": "go-m8010-gui-command/1.2",
            "sequence": sequence,
            "source_instance_id": self.command_source_instance_id,
            "source_monotonic_ns": time.monotonic_ns(),
            "mode": mode,
            "targets_rad": [0.0] * 6 if is_brake else command_targets,
            "active_joint_mask": [False] * 6 if is_brake else active_joint_mask,
            "moving_joint_mask": [False] * 6 if is_brake else moving_joint_mask,
            "activation_epoch": 0 if is_brake else activation_epoch,
            "maximum_velocity_rad_s": float(control["最大速度_度每秒"]) * RAD,
            "maximum_acceleration_rad_s2": float(control["最大加速度_度每二次方秒"]) * RAD,
            "kp": [0.0] * 6 if is_brake else [
                float(config["关节"][f"J{i + 1}"].get("Kp", 0.0))
                for i in range(6)
            ],
            "kd": [0.0] * 6 if is_brake else [
                float(config["关节"][f"J{i + 1}"].get("Kd", 0.0))
                for i in range(6)
            ],
        }
        if mode == "position":
            if not isinstance(collision_guard_proof, dict):
                raise ValueError("POSITION命令缺少已批准的碰撞守卫证明")
            if (
                not isinstance(trajectory_descriptor, dict)
                or set(trajectory_descriptor) != {"plan_token_id", "trajectory"}
            ):
                raise ValueError("POSITION命令缺少已预演的quintic描述符")
            if (
                not isinstance(plan_manifest, dict)
                or set(plan_manifest)
                != {"schema", "recipe_sha256", "segment_sha256"}
                or plan_manifest.get("schema")
                != "go-m8010-plan-manifest/1.0"
            ):
                raise ValueError("POSITION命令缺少不可变分段manifest")
            payload["schema"] = "go-m8010-gui-command/1.3"
            payload.update(json.loads(json.dumps(trajectory_descriptor)))
            payload["plan_manifest"] = json.loads(json.dumps(plan_manifest))
            payload["collision_guard_proof"] = json.loads(
                json.dumps(collision_guard_proof)
            )
        self.command_publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.target_publisher.publish(Float64MultiArray(data=virtual_targets))

    def publish_collision_request(self, payload: dict) -> None:
        self.collision_request_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )

    def publish_planned_path_request(
        self,
        recipe: TrajectoryRecipe,
        *,
        session_id: str,
        state_instance_id: str,
    ) -> dict:
        """Request an independent proof; this publishes no motor command."""

        sequence = self.reserve_planned_path_request_sequence()
        payload = planned_trajectory_feasibility_request(
            recipe,
            source_instance_id=self.command_source_instance_id,
            sequence=sequence,
            source_monotonic_ns=time.monotonic_ns(),
            session_id=session_id,
            state_instance_id=state_instance_id,
            model_sha256=PRODUCTION_MODEL_SHA256,
            gravity_config_sha256=GRAVITY_CONFIG_SHA256,
            thermal_config_sha256=THERMAL_CONFIG_SHA256,
        )
        self.publish_serialized_planned_path_request(
            json.dumps(payload, ensure_ascii=False, allow_nan=False)
        )
        return payload

    def reserve_planned_path_request_sequence(self) -> int:
        """Reserve one monotonic request identity on the Qt/ROS owner thread."""

        self.planned_path_request_sequence += 1
        return self.planned_path_request_sequence

    def publish_serialized_planned_path_request(self, serialized: str) -> None:
        """Publish worker-built JSON without doing its serialization in Qt."""

        if not isinstance(serialized, str) or not serialized:
            raise ValueError("planned-path request JSON is required")
        self.planned_path_request_publisher.publish(String(data=serialized))


class MainWindow(QMainWindow):
    preview_plan_completed = Signal(object)
    preview_plan_failed = Signal(object)

    def __init__(self, node: ArmGuiNode) -> None:
        super().__init__()
        self.node = node
        self.config = yaml.safe_load(node.config_path.read_text(encoding="utf-8"))
        limits_document = yaml.safe_load(node.joint_limits_path.read_text(encoding="utf-8"))
        if limits_document.get("单位") != "度" or limits_document.get("参考") != "SESSION_REFERENCE_V1":
            raise ValueError("关节限位文件语义不匹配")
        configured_limits = []
        for index in range(6):
            bounds = [float(value) for value in limits_document[f"J{index + 1}"]]
            if len(bounds) != 2 or not all(math.isfinite(value) for value in bounds) or bounds[0] >= bounds[1]:
                raise ValueError("关节限位文件无效")
            configured_limits.append((bounds[0], bounds[1]))
        if hashlib.sha256(node.embedded_model_path.read_bytes()).hexdigest() != PRODUCTION_MODEL_SHA256:
            raise ValueError("冻结生产MuJoCo模型哈希不匹配")
        model_limits = model_session_relative_limits(
            node.embedded_model_path, node.embedded_session_pose_deg
        )
        if any(
            abs(configured - model) > 1.0e-6
            for configured_pair, model_pair in zip(configured_limits, model_limits)
            for configured, model in zip(configured_pair, model_pair)
        ):
            raise ValueError("关节限位配置与冻结3D模型／本次竖直会话锚点不一致")
        self.limits = model_limits
        self.edit_limits = verified_edit_limits(self.limits)
        self.session_pose_rad = parse_pose_degrees(
            node.embedded_session_pose_deg
        ).tolist()
        self.session_pose_sha256 = session_pose_sha256(self.session_pose_rad)
        self.session_pose_deg = np.degrees(self.session_pose_rad).tolist()
        self.absolute_limits = [
            (lower + self.session_pose_deg[index],
             upper + self.session_pose_deg[index])
            for index, (lower, upper) in enumerate(self.limits)
        ]
        control = self.config["控制"]
        self.machine = ModeMachine()
        # V15.31A exposes only the virtual-first position workflow.  Editing is
        # non-authorizing and the command stream remains suspended at startup.
        self.direction = ArmMode.SIM_TO_REAL
        self.hardware_mode = "brake"
        # The candidate is the final, operator-visible virtual pose.  ``targets``
        # remains the exact target of the current physical segment.  Keeping the
        # two separate prevents a <=30 degree segment from replacing the final
        # pose in the MuJoCo preview while a real sequence is running.
        self.candidate_targets = [0.0] * 6
        self.targets = [0.0] * 6
        self.command_targets = [0.0] * 6
        self.candidate_joint_mask = [False] * 6
        self.requested_active_joint_mask = [False] * 6
        self.pending_target_joint_mask = [False] * 6
        self.moving_joint_mask = [False] * 6
        self.activation_epoch = max(1, time.monotonic_ns() & ((1 << 63) - 1))
        self.actual = [0.0] * 6
        workflow_limits_rad = tuple(
            (lower * RAD, upper * RAD) for lower, upper in self.edit_limits
        )
        self.workflow_contract = WorkflowState.initialize(
            self.actual,
            workflow_limits_rad,
            PRODUCTION_MODEL_SHA256,
            session_id="UNBOUND_SESSION",
            state_instance_id="UNBOUND_STATE_INSTANCE",
            gravity_config_sha256=GRAVITY_CONFIG_SHA256,
        )
        self.preview_requested_by_operator = False
        self.preview_animation_started_at: Optional[float] = None
        self.preview_animation_complete = False
        self.preview_collision_safe = False
        self.preview_collision_segment_index = 0
        self.preview_collision_segment_sha256: list[str] = []
        self.preview_collision_request_segment_by_sequence: dict[int, int] = {}
        self.preview_frame_index = 0
        self.preview_pose = tuple(self.actual)
        self.task_id: Optional[str] = None
        self.task_started_at: Optional[float] = None
        self.task_started_wall_utc: Optional[str] = None
        self.task_execution_started_at: Optional[float] = None
        self.task_completed_at: Optional[float] = None
        self.task_segment_count = 0
        self.task_completed_segments = 0
        self.last_task_heartbeat = time.monotonic()
        self.task_status_labels: dict[str, QLabel] = {}
        self.motor_status_table: Optional[QTableWidget] = None
        self.j2_motor_summary: Optional[QLabel] = None
        self.connected = [False] * 6
        self.faulted = [False] * 6
        self.observation_uncertain = [False] * 6
        self.active_observation_uncertain = False
        self.have_first_state = False
        self.session_id: Optional[str] = None
        self.state_instance_id: Optional[str] = None
        self.command_sequence = 0
        self.collision_request_sequence = 0
        self.collision_requests: dict[int, dict] = {}
        self.latest_collision_preview_sequence: Optional[int] = None
        self.collision_preview_state = "idle"
        self.collision_preview_started_at = 0.0
        self.approved_candidate_sha256: Optional[str] = None
        self.approved_candidate_session_id: Optional[str] = None
        self.approved_candidate_state_instance_id: Optional[str] = None
        self.pending_collision_execute_sequence: Optional[int] = None
        self.active_collision_proof: Optional[dict] = None
        self.queued_pose_target: Optional[list[float]] = None
        self.queued_joint_indices: list[int] = []
        self.queued_trajectory_segments: list[TrajectoryPlan] = []
        self.trajectory_segment_count = 0
        self.active_trajectory_segment: Optional[TrajectoryPlan] = None
        self.active_trajectory_segment_index: Optional[int] = None
        self.active_trajectory_descriptor: Optional[dict] = None
        self.active_plan_manifest: Optional[dict] = None
        self.active_trajectory_first_publish_pending = False
        self.active_sequence_joint: Optional[int] = None
        self.last_consumed_collision_sequence = 0
        self.last_collision_popup_signature: Optional[str] = None
        self.collision_popup: Optional[QMessageBox] = None
        # A newly opened/restarted GUI must not overwrite an independently
        # holding worker with this window's local default BRAKE state.
        self.command_stream_suspended = True
        self.command_stream_suspended_reason = "等待操作员在新鲜反馈下明确接管"
        self.last_log_at = 0.0
        self.last_summary_refresh_at = 0.0
        self.operator_notice_text = ""
        self.operator_notice_level = "info"
        self.operator_notice_until = 0.0
        self.current_notice_level = ""
        self.arrival = ArrivalTracker(
            tolerance_rad=float(control["到位容差_度"]) * RAD,
            dwell_s=float(control["到位持续_秒"]),
            timeout_s=float(control["目标超时_秒"]),
            per_joint_tolerance_rad=tuple(
                [float(control["到位容差_度"]) * RAD] * 5
                + [float(control["J6到位容差_度"]) * RAD]
            ),
        )
        self.virtual_widgets: list[VirtualWidgets] = []
        self.virtual_edit_requested = False
        self.real_widgets: list[RealWidgets] = []
        self.planned_mujoco_preview: Optional[EmbeddedMujocoPreview] = None
        self.actual_mujoco_preview: Optional[EmbeddedMujocoPreview] = None
        # Compatibility alias for old diagnostics; it always names the plan
        # renderer and is never fed encoder state.
        self.mujoco_preview: Optional[EmbeddedMujocoPreview] = None
        # Exact trajectory construction is CPU-heavy (several seconds for a
        # wide six-axis pose).  One worker plus cancellation of the one queued
        # future implements latest-only coalescing without an unbounded queue.
        self._preview_plan_lock = threading.Lock()
        self._preview_plan_executor = concurrent.futures.ThreadPoolExecutor(
            max_workers=1,
            thread_name_prefix="m8010-preview-plan",
        )
        self._preview_plan_generation = 0
        self._preview_plan_future = None
        self._preview_plan_started_at: Optional[float] = None
        self._preview_plan_closed = False
        self.preview_plan_completed.connect(self._apply_preview_plan_result)
        self.preview_plan_failed.connect(self._apply_preview_plan_failure)
        self.setWindowTitle(
            "纯仿真零位调姿（不连接真机）"
            if node.joint_limits_path.name == "gui_joint_limits_pose_adjust.yaml"
            else "六自由度机械臂控制系统"
        )
        self._build_ui()
        self._fit_window_to_available_screen()
        self.collision_preview_timer = QTimer(self)
        self.collision_preview_timer.setSingleShot(True)
        self.collision_preview_timer.timeout.connect(
            self._request_collision_preview
        )
        self._open_log()

        refresh_ms = round(1000.0 / float(self.config["界面"]["刷新频率_赫兹"]))
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(refresh_ms)
        self.statusBar().showMessage("正在等待六关节编码器；当前为会话相对角度")

    def _build_ui(self) -> None:
        root = QWidget()
        outer = QVBoxLayout(root)
        title = QLabel("六自由度机械臂控制系统")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Sans Serif", 18, QFont.Bold))
        subtitle = QLabel("当前为会话相对角度　｜　计算机辅助设计零位：待定　｜　机器人系统零位：待定")
        subtitle.setAlignment(Qt.AlignCenter)
        outer.addWidget(title)
        outer.addWidget(subtitle)

        columns = QHBoxLayout()
        columns.addWidget(self._virtual_panel(), 2)
        columns.addWidget(self._real_panel(), 3)
        outer.addLayout(columns)

        # Two independent renderers deliberately own different MjData objects.
        # The planned side consumes only q_plan_target/q_plan_trajectory; the
        # actual side consumes only fresh encoder q_actual.
        twins = QHBoxLayout()
        for attribute, title in (
            ("planned_mujoco_preview", "计划／虚拟机械臂（不代表真机）"),
            ("actual_mujoco_preview", "现实机械臂数字孪生（仅编码器）"),
        ):
            try:
                preview = EmbeddedMujocoPreview(
                    self.node.embedded_model_path,
                    self.node.embedded_session_pose_deg,
                    title,
                )
                setattr(self, attribute, preview)
                twins.addWidget(preview, 1)
            except Exception as exc:
                unavailable = QGroupBox(title)
                unavailable_layout = QVBoxLayout(unavailable)
                unavailable_label = QLabel(f"内嵌模型不可用：{exc}")
                unavailable_label.setWordWrap(True)
                unavailable_label.setAlignment(Qt.AlignCenter)
                unavailable_layout.addWidget(unavailable_label)
                twins.addWidget(unavailable, 1)
        self.mujoco_preview = self.planned_mujoco_preview
        outer.addLayout(twins, 1)
        outer.addWidget(self._control_panel())
        outer.addWidget(self._task_status_panel())
        outer.addWidget(self._motor_status_panel())

        self.safety_notice = QLabel(
            "安全状态：等待控制状态流与命令路由确认；控制请求不代表硬件已执行"
        )
        self.safety_notice.setWordWrap(True)
        self.safety_notice.setAlignment(Qt.AlignCenter)
        outer.addWidget(self.safety_notice)

        self.summary = QLabel(
            "健康反馈：正在连接　｜　控制状态流：等待数据　｜　"
            "控制请求：制动　｜　控制器确认：等待　｜　命令路由：等待"
        )
        self.summary.setWordWrap(True)
        self.summary.setAlignment(Qt.AlignCenter)
        self.summary.setMinimumHeight(58)
        self.summary.setStyleSheet("padding: 8px; background: #263238; color: white;")
        outer.addWidget(self.summary)
        scroll = QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setSizeAdjustPolicy(QAbstractScrollArea.AdjustIgnored)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAsNeeded)
        # A permanent vertical gutter prevents the summary's word wrapping
        # from making the scrollbar appear/disappear and recursively changing
        # the viewport width (perceived as whole-window flashing).
        scroll.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOn)
        scroll.setWidget(root)
        self.setCentralWidget(scroll)
        self.setStyleSheet(
            "QGroupBox { font-weight: bold; border: 1px solid #78909c; margin-top: 8px; padding-top: 8px; }"
            "QPushButton { min-height: 32px; padding: 3px 10px; }"
            "QLabel { min-height: 20px; }"
        )
        self._set_virtual_editable(True)

    def _fit_window_to_available_screen(self) -> None:
        """Keep the GUI inside the usable desktop without shortening sliders."""

        screen = QApplication.primaryScreen()
        if screen is None:
            self.resize(1280, 720)
            return
        available = screen.availableGeometry()
        self.resize(*fitted_window_size(available.width(), available.height()))

    def _slider(
        self, index: int, bounds: Optional[tuple[float, float]] = None
    ) -> QSlider:
        slider = QSlider(Qt.Horizontal)
        lower, upper = self.limits[index] if bounds is None else bounds
        slider.setRange(round(lower * 100.0), round(upper * 100.0))
        slider.setSingleStep(1)
        slider.setPageStep(100)
        slider.setMinimumWidth(420)
        slider.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)
        return slider

    def _virtual_panel(self) -> QGroupBox:
        box = QGroupBox("计划目标（唯一可编辑滑条组）")
        layout = QGridLayout(box)
        layout.setColumnStretch(1, 1)
        layout.addWidget(QLabel("关节"), 0, 0)
        layout.addWidget(QLabel("计划目标滑条"), 0, 1)
        layout.addWidget(QLabel("目标角度"), 0, 2)
        layout.addWidget(QLabel("精确输入"), 0, 3)
        for index, label_text in enumerate(JOINT_LABELS):
            edit_bounds = self.edit_limits[index]
            absolute_bounds = self.absolute_limits[index]
            value = QLabel("+0.00°")
            slider = self._slider(index, edit_bounds)
            spin = QDoubleSpinBox()
            spin.setRange(*edit_bounds)
            spin.setDecimals(2)
            spin.setSingleStep(0.1)
            spin.setMinimumWidth(180)
            limit_text = (
                f"历史3D模型完整关节范围："
                f"绝对{absolute_bounds[0]:+.2f}° .. {absolute_bounds[1]:+.2f}°，"
                f"本会话{edit_bounds[0]:+.2f}° .. {edit_bounds[1]:+.2f}°；"
                "可先设置多个关节并由虚拟机械臂预演；"
                f"沿本次运动路径预留{COLLISION_MARGIN_DEG:.0f}°碰撞余量"
            )
            spin.setToolTip(limit_text)
            slider.setToolTip(limit_text)
            slider.valueChanged.connect(
                lambda hundredths, i=index: self._virtual_slider_changed(
                    i, hundredths
                )
            )
            spin.valueChanged.connect(lambda degrees, i=index: self._virtual_spin_changed(i, degrees))
            row = index + 1
            layout.addWidget(QLabel(label_text), row, 0)
            layout.addWidget(slider, row, 1)
            layout.addWidget(value, row, 2)
            layout.addWidget(spin, row, 3)
            self.virtual_widgets.append(VirtualWidgets(value, spin, slider))
        return box

    def _real_panel(self) -> QGroupBox:
        box = QGroupBox("现实机械臂（编码器反馈）")
        layout = QGridLayout(box)
        for column, text in enumerate(("关节", "实际角度", "硬件命令", "位置误差", "状态")):
            layout.addWidget(QLabel(text), 0, column)
        for index, label_text in enumerate(JOINT_LABELS):
            widgets = RealWidgets(None, QLabel("+0.00°"), QLabel("+0.00°"), QLabel("+0.00°"), QLabel("未连接"))
            row = index + 1
            layout.addWidget(QLabel(label_text), row, 0)
            for column, widget in enumerate((widgets.actual, widgets.target, widgets.error, widgets.state), 1):
                layout.addWidget(widget, row, column)
            self.real_widgets.append(widgets)
        return box

    def _task_status_panel(self) -> QGroupBox:
        box = QGroupBox("当前任务状态")
        layout = QGridLayout(box)
        fields = (
            ("任务ID", "task_id"), ("当前状态", "state"),
            ("当前阶段", "phase"), ("进度", "progress"),
            ("开始时间", "started"), ("已用时间", "elapsed"),
            ("预计剩余", "remaining"), ("最后控制心跳", "heartbeat"),
            ("最后编码器反馈", "encoder"), ("当前目标", "target"),
            ("当前实际", "actual"), ("最大位置误差", "max_error"),
            ("当前最大温度", "max_temperature"),
            ("当前热状态", "thermal"),
            ("计划最大重力矩", "planned_gravity"),
            ("计划最大电机矩", "planned_motor"),
            ("限位/碰撞/热负载", "planned_checks"),
            ("失败原因", "failure"),
        )
        for index, (title, key) in enumerate(fields):
            row = index // 3
            column = (index % 3) * 2
            layout.addWidget(QLabel(title + "："), row, column)
            value = QLabel("—")
            value.setWordWrap(True)
            layout.addWidget(value, row, column + 1)
            self.task_status_labels[key] = value
        return box

    def _motor_status_panel(self) -> QGroupBox:
        box = QGroupBox("物理电机状态（七颗电机；力矩侧别与单位明确）")
        layout = QVBoxLayout(box)
        headers = (
            "逻辑关节", "物理电机", "ID", "在线", "新鲜度ms", "当前模式",
            "原始位置rad", "逻辑位置°", "速度°/s",
            "tau反馈\nrotor N·m", "估算关节力矩\nN·m", "温度°C",
            "merror", "热状态", "控制状态", "最后反馈时间\nmonotonic s",
        )
        table = QTableWidget(len(MOTOR_NAMES), len(headers))
        table.setHorizontalHeaderLabels(list(headers))
        table.setVerticalHeaderLabels(list(MOTOR_NAMES))
        table.setEditTriggers(QAbstractItemView.NoEditTriggers)
        table.setSelectionMode(QAbstractItemView.NoSelection)
        table.setAlternatingRowColors(True)
        table.setMinimumHeight(250)
        for row, name in enumerate(MOTOR_NAMES):
            for column in range(len(headers)):
                table.setItem(row, column, QTableWidgetItem("—"))
            table.item(row, 0).setText(MOTOR_LOGICAL_JOINT[name])
            table.item(row, 1).setText(name)
            table.item(row, 2).setText(MOTOR_BUS_LOCAL_ID[name])
        table.resizeColumnsToContents()
        self.motor_status_table = table
        layout.addWidget(table)
        self.j2_motor_summary = QLabel(
            "J2：等待双电机反馈（逻辑实际／目标／误差／同步误差／力矩分担／温差）"
        )
        self.j2_motor_summary.setWordWrap(True)
        layout.addWidget(self.j2_motor_summary)
        return box

    def _button(self, text: str, callback, object_name: str = "") -> QPushButton:
        button = QPushButton(text)
        if object_name:
            button.setObjectName(object_name)
        button.clicked.connect(callback)
        return button

    def _control_panel(self) -> QGroupBox:
        box = QGroupBox("固定位置模式：虚拟先行，明确确认后才可下发现实")
        layout = QGridLayout(box)
        buttons = [
            ("保持当前位置", self._hold_current),
            ("预演轨迹", self._start_virtual_preview),
            ("下发到现实", self._execute_target),
            ("恢复初始化姿态", self._return_initial_pose),
            ("停止并制动", self._emergency_brake),
        ]
        shortcuts = {
            "保持当前位置": "Alt+H",
            "预演轨迹": "Alt+P",
            "下发到现实": "Alt+E",
            "恢复初始化姿态": "Alt+R",
            "停止并制动": "Alt+B",
        }
        for index, (text, callback) in enumerate(buttons):
            button = self._button(text, callback)
            button.setShortcut(shortcuts[text])
            if text == "保持当前位置":
                self.hold_current_button = button
                button.setToolTip(
                    "在六轴新鲜健康反馈下仅捕获一次当前角度，"
                    "建立全轴闭环HOLD；不发布POSITION轨迹"
                )
            if text == "下发到现实":
                self.execute_target_button = button
                button.setEnabled(False)
                button.setToolTip("必须先完成与当前候选完全匹配的轨迹预演")
            if text == "停止并制动":
                button.setStyleSheet(
                    "background: #4a0000; color: #ffeb3b; font-weight: bold; min-height: 46px;"
                )
            layout.addWidget(button, 0, index)
        self.mode_label = QLabel(
            "当前固定模式：位置控制　｜　控制请求：制动　｜　硬件确认：等待状态反馈"
        )
        self.mode_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.mode_label, 1, 0, 1, 5)
        servo_notice = QLabel(
            "说明：调整计划滑条只改变虚拟目标，不会发布真实运动。"
            "停止并制动会撤销位置伺服和承重保持，重载关节必须有可靠机械支撑。"
        )
        servo_notice.setWordWrap(True)
        layout.addWidget(servo_notice, 2, 0, 1, 5)
        self.workflow_status = QLabel(
            "工作流：等待设置虚拟候选姿态；未授权现实运动"
        )
        self.workflow_status.setWordWrap(True)
        self.workflow_status.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.workflow_status, 3, 0, 1, 5)
        self.workflow_progress = QProgressBar()
        self.workflow_progress.setRange(0, 100)
        self.workflow_progress.setValue(0)
        self.workflow_progress.setFormat("虚拟预演未开始")
        layout.addWidget(self.workflow_progress, 4, 0, 1, 5)
        return box

    def _set_workflow_state(
        self, state: str, text: str, progress: Optional[int] = None,
    ) -> None:
        """Keep virtual solving/approval/real execution state permanently visible."""

        self.collision_preview_state = state
        if (
            state in {"complete", "timeout", "blocked", "unsafe", "stale"}
            and getattr(self, "task_id", None) is not None
            and getattr(self, "task_completed_at", None) is None
        ):
            self.task_completed_at = time.monotonic()
        label = getattr(self, "workflow_status", None)
        if label is not None:
            set_widget_text_if_changed(label, f"工作流：{text}")
        bar = getattr(self, "workflow_progress", None)
        if bar is not None:
            if progress is None:
                bar.setRange(0, 0)
                bar.setFormat(text)
            else:
                bar.setRange(0, 100)
                bar.setValue(max(0, min(100, int(progress))))
                bar.setFormat(f"%p% — {text}")
        self._refresh_execute_target_enabled()

    def _invalidate_preview_plan_build(self) -> int:
        """Invalidate the running/queued planner and return the new generation."""

        with self._preview_plan_lock:
            self._preview_plan_generation += 1
            generation = self._preview_plan_generation
            future = self._preview_plan_future
            self._preview_plan_future = None
            self._preview_plan_started_at = None
        if future is not None:
            # A running pure worker may finish, but its generation can no
            # longer publish or mutate the workflow.  A queued worker is
            # removed immediately, which bounds the executor to latest-only.
            future.cancel()
        return generation

    def _preview_plan_future_done(
        self, generation: int, future: concurrent.futures.Future,
    ) -> None:
        """Bridge a worker result to Qt without touching GUI state here."""

        if future.cancelled():
            return
        try:
            result = future.result()
        except Exception as exc:  # delivered and rendered on the Qt thread
            event = {"generation": generation, "detail": str(exc)}
            with self._preview_plan_lock:
                closed = self._preview_plan_closed
            if not closed:
                self.preview_plan_failed.emit(event)
            return
        with self._preview_plan_lock:
            closed = self._preview_plan_closed
        if not closed:
            self.preview_plan_completed.emit(result)

    def _apply_preview_plan_failure(self, event: object) -> None:
        """Fail one current planning generation on the Qt event thread."""

        if not isinstance(event, dict):
            return
        generation = event.get("generation")
        with self._preview_plan_lock:
            if (
                self._preview_plan_closed
                or type(generation) is not int
                or generation != self._preview_plan_generation
            ):
                return
        detail = event.get("detail")
        if not isinstance(detail, str) or not detail:
            detail = "未知后台规划错误"
        state = event.get("state", "unsafe")
        if state not in {"unsafe", "stale", "timeout"}:
            state = "unsafe"
        self._clear_candidate_approval()
        message = f"轨迹解算失败：{detail}"
        if state == "timeout":
            message = detail
        elif state == "stale":
            message = f"轨迹解算结果已过期：{detail}"
        self._notify(message, "warning")
        self._set_workflow_state(state, message, 0)

    def _apply_preview_plan_result(self, result: object) -> None:
        """Freshness-check and publish one worker result on the Qt thread."""

        if not isinstance(result, dict):
            self._apply_preview_plan_failure({
                "generation": self._preview_plan_generation,
                "detail": "后台规划结果格式无效",
            })
            return
        generation = result.get("generation")
        with self._preview_plan_lock:
            if (
                self._preview_plan_closed
                or type(generation) is not int
                or generation != self._preview_plan_generation
            ):
                return

        workflow = result.get("workflow")
        trajectory = result.get("trajectory")
        serialized = result.get("serialized_request")
        source_ns = result.get("request_source_monotonic_ns")
        now = time.monotonic()
        now_ns = time.monotonic_ns()
        try:
            control = self.config["控制"]
            current_velocity = float(control["最大速度_度每秒"]) * RAD
            current_acceleration = (
                float(control["最大加速度_度每二次方秒"]) * RAD
            )
            current_limits = tuple(
                (lower * RAD, upper * RAD) for lower, upper in self.edit_limits
            )
            current_actual = tuple(float(value) for value in self.actual)
            result_actual = tuple(result["actual_rad"])
            result_target = tuple(result["target_rad"])
        except (KeyError, TypeError, ValueError):
            self._apply_preview_plan_failure({
                "generation": generation,
                "detail": "当前配置或位姿无法重新验证",
                "state": "stale",
            })
            return
        result_shape_valid = bool(
            isinstance(workflow, WorkflowState)
            and isinstance(trajectory, TrajectoryRecipe)
            and workflow.q_plan_trajectory is trajectory
            and isinstance(serialized, str)
            and bool(serialized)
            and type(source_ns) is int
            and source_ns > 0
            and len(result_actual) == 6
            and len(result_target) == 6
            and all(
                type(value) in {int, float} and math.isfinite(float(value))
                for value in result_actual + result_target
            )
        )
        identity_fresh = bool(
            result_shape_valid
            and result.get("session_id") == self.session_id
            and result.get("state_instance_id") == self.state_instance_id
            and result_target == tuple(self.candidate_targets)
            and result_target == self.workflow_contract.q_plan_target
            and result.get("base_candidate_revision")
            == self.workflow_contract.candidate_revision
            and result.get("limits_rad") == current_limits
            and result.get("maximum_velocity_rad_s") == current_velocity
            and result.get("maximum_acceleration_rad_s2")
            == current_acceleration
            and result.get("maximum_segment_delta_rad")
            == float(COLLISION_EXECUTE_SEGMENT_MAX_DEG) * RAD
            and result.get("maximum_sample_period_s") == 0.01
            and self.node.control_streams_fresh(now)
            and self.connected == [True] * 6
            and len(current_actual) == 6
            and all(
                math.isfinite(current) and math.isfinite(source)
                and abs(current - source) <= PLAN_ACTUAL_DRIFT_TOLERANCE_RAD
                for current, source in zip(current_actual, result_actual)
            )
            and 0 <= now_ns - source_ns
            <= PLANNED_REQUEST_PUBLISH_MAX_AGE_NS
        )
        if not identity_fresh:
            self._apply_preview_plan_failure({
                "generation": generation,
                "detail": (
                    "session/状态实例/候选目标/实际姿态/配置或"
                    "请求新鲜度已变化，未发布证明请求"
                ),
                "state": "stale",
            })
            return
        try:
            # ROS publication remains on the Qt/ROS owner thread.  The JSON
            # bytes and their identity hash were already built by the worker.
            self.node.publish_serialized_planned_path_request(serialized)
        except Exception as exc:
            self._apply_preview_plan_failure({
                "generation": generation,
                "detail": f"计划路径证明请求发布失败：{exc}",
            })
            return
        with self._preview_plan_lock:
            if (
                self._preview_plan_closed
                or generation != self._preview_plan_generation
            ):
                return
            self._preview_plan_future = None
            self._preview_plan_started_at = None
        self.workflow_contract = workflow
        self.preview_animation_started_at = time.monotonic()
        self.preview_animation_complete = False
        self.preview_collision_safe = False
        self.preview_frame_index = 0
        self.preview_pose = trajectory.start_rad
        self.last_task_heartbeat = self.preview_animation_started_at
        self._set_workflow_state(
            "previewing",
            f"后台轨迹已生成；正在虚拟预演，预计"
            f"{trajectory.profile.duration_s:.2f}秒",
            10,
        )
        self._request_collision_preview()

    def _update_preview_plan_build_heartbeat(self, now: float) -> None:
        """Keep planning visibly alive and fail one bounded stage on timeout."""

        with self._preview_plan_lock:
            future = self._preview_plan_future
            generation = self._preview_plan_generation
            started_at = self._preview_plan_started_at
            closed = self._preview_plan_closed
        if closed or future is None or started_at is None or future.done():
            return
        self.last_task_heartbeat = now
        if now - started_at <= PREVIEW_PLAN_BUILD_TIMEOUT_S:
            return
        self._apply_preview_plan_failure({
            "generation": generation,
            "state": "timeout",
            "detail": (
                "阶段超时：节点=arm_control_gui，关节=六轴计划，"
                "等待=后台分段五次轨迹与证明请求序列化"
            ),
        })

    def _shutdown_preview_plan_executor(self) -> None:
        """Cancel planning without waiting in the Qt close callback."""

        with self._preview_plan_lock:
            if self._preview_plan_closed:
                return
            self._preview_plan_closed = True
            self._preview_plan_generation += 1
            future = self._preview_plan_future
            self._preview_plan_future = None
            self._preview_plan_started_at = None
        if future is not None:
            future.cancel()
        self._preview_plan_executor.shutdown(wait=False, cancel_futures=True)

    def _clear_candidate_approval(self) -> None:
        """Revoke a preview approval without changing any hardware authority."""

        self._invalidate_preview_plan_build()
        self.approved_candidate_sha256 = None
        self.approved_candidate_session_id = None
        self.approved_candidate_state_instance_id = None
        self.preview_requested_by_operator = False
        self.preview_animation_started_at = None
        self.preview_animation_complete = False
        self.preview_collision_safe = False
        self.preview_collision_segment_index = 0
        self.preview_collision_segment_sha256 = []
        self.preview_collision_request_segment_by_sequence = {}
        if hasattr(self, "workflow_contract"):
            self.workflow_contract = self.workflow_contract.invalidate_preview(
                clear_trajectory=True
            )
        # A late result from a superseded/edited candidate must never re-enable
        # the real execution button.
        self.latest_collision_preview_sequence = None
        self._refresh_execute_target_enabled()

    def _preview_approval_matches_candidate(self) -> bool:
        try:
            digest = collision_target_sha256(self.candidate_targets)
        except (AttributeError, TypeError, ValueError):
            return False
        return bool(
            self.preview_requested_by_operator
            and self.preview_animation_complete
            and self.collision_preview_state == "safe"
            and self.approved_candidate_sha256 == digest
            and self.approved_candidate_session_id == self.session_id
            and self.approved_candidate_state_instance_id == self.state_instance_id
            and self.workflow_contract.current_plan_token is not None
        )

    def _refresh_execute_target_enabled(self) -> None:
        button = getattr(self, "execute_target_button", None)
        if button is None:
            return
        approved = self._preview_approval_matches_candidate()
        blocked = bool(
            self.hardware_mode == "position"
            or any(self.moving_joint_mask)
            or self.pending_collision_execute_sequence is not None
            or self.queued_pose_target is not None
        )
        enabled = bool(approved and not blocked)
        set_widget_enabled_if_changed(button, enabled)
        if enabled:
            tooltip = (
                "当前虚拟候选姿态已通过SAFE预演；"
                "点击后仍会重新校验六轴健康、静止HOLD和每个物理分段"
            )
        elif approved:
            tooltip = "现实轨迹／安全分段正在执行，不能重复确认"
        else:
            tooltip = "必须先获得与当前虚拟候选姿态完全匹配的SAFE预演结果"
        set_widget_tooltip_if_changed(button, tooltip)

    def _set_virtual_editable(self, enabled: bool) -> None:
        self.virtual_edit_requested = bool(enabled)
        self._refresh_virtual_editability()

    def _refresh_virtual_editability(self) -> None:
        # Virtual target selection is non-authorizing and deliberately remains
        # available when hardware feedback is missing/faulted.  Real execution
        # still passes the strict full-arm gate in _execute_target(), again in
        # _new_collision_request("execute"), and again at commit time.
        edit_blocked = bool(
            self.hardware_mode == "position"
            or self.pending_collision_execute_sequence is not None
            or self.queued_pose_target is not None
        )
        for index, widgets in enumerate(self.virtual_widgets):
            joint_enabled = bool(
                self.virtual_edit_requested
                and self.direction is ArmMode.SIM_TO_REAL
                and not edit_blocked
            )
            set_widget_enabled_if_changed(widgets.target, joint_enabled)
            if widgets.slider is not None:
                set_widget_enabled_if_changed(widgets.slider, joint_enabled)
            if self.virtual_edit_requested and edit_blocked:
                text = (
                    "已有POSITION运动、逐轴安全序列或碰撞执行检查正在进行；"
                    "请先停止轨迹并确认全轴HOLD，再选择下一关节目标"
                )
            else:
                bounds = self.edit_limits[index]
                absolute_bounds = self.absolute_limits[index]
                limit_text = (
                    f"历史3D模型完整关节范围："
                    f"绝对{absolute_bounds[0]:+.2f}° .. {absolute_bounds[1]:+.2f}°，"
                    f"本会话{bounds[0]:+.2f}° .. {bounds[1]:+.2f}°；"
                    "虚拟编辑不依赖真机健康且不下发运动；"
                    "确认现实执行前必须获得匹配的SAFE预演，"
                    f"每一物理分段都重新检查并预留{COLLISION_MARGIN_DEG:.0f}°余量"
                )
                text = limit_text
            set_widget_tooltip_if_changed(widgets.target, text)
            if widgets.slider is not None:
                set_widget_tooltip_if_changed(widgets.slider, text)
        self._refresh_execute_target_enabled()

    def _record_virtual_target(self, index: int, target_rad: float) -> None:
        """Record a virtual-first target without granting hardware authority."""

        self.candidate_targets[index] = target_rad
        self.targets[index] = target_rad
        try:
            updated_workflow = self.workflow_contract.change_plan_target(
                self.candidate_targets
            )
        except ContractViolation as exc:
            self._notify(f"计划目标无效：{exc}", "warning")
            return
        if (
            updated_workflow is self.workflow_contract
            and getattr(self.workflow_contract, "q_plan_target", None)
            == tuple(self.candidate_targets)
        ):
            self._show_targets_on_virtual()
            return
        self.workflow_contract = updated_workflow
        # Candidate selection remains visible even if a hardware connection is
        # absent.  ``pending_target_joint_mask`` is reserved for the one-axis
        # physical segment created only after explicit confirmation.
        self.candidate_joint_mask = [
            abs(candidate - locked) > TARGET_SELECTION_DEADBAND_RAD
            for candidate, locked in zip(
                self.candidate_targets, self.command_targets
            )
        ]
        self.pending_target_joint_mask = [False] * 6
        self._clear_candidate_approval()
        self._set_workflow_state(
            "pending",
            "候选姿态已更改，等待完整3D模型预演；现实运动未授权",
            10,
        )
        self._show_targets_on_virtual()
        self.collision_preview_timer.stop()

    def _virtual_spin_changed(self, index: int, degrees: float) -> None:
        if (
            self.direction is not ArmMode.SIM_TO_REAL
            or self.hardware_mode == "position"
            or self.pending_collision_execute_sequence is not None
            or self.queued_pose_target is not None
        ):
            return
        widgets = self.virtual_widgets[index]
        widgets.value.setText(f"{degrees:+.2f}°")
        self._record_virtual_target(index, degrees * RAD)

    def _virtual_slider_changed(self, index: int, hundredths_degree: int) -> None:
        """Update only the planned candidate; never publish a motor command."""

        if (
            self.direction is not ArmMode.SIM_TO_REAL
            or self.hardware_mode == "position"
            or self.pending_collision_execute_sequence is not None
            or self.queued_pose_target is not None
        ):
            return
        self._record_virtual_target(index, float(hundredths_degree) * 0.01 * RAD)

    def _start_virtual_preview(self) -> None:
        """Queue the exact recipe without blocking the Qt event thread."""

        if not self._require_control_feedback(
            "轨迹预演需要六轴新鲜编码器和硬件状态。", require_all=True
        ):
            return
        if not self.session_id or not source_instance_id_valid(self.state_instance_id):
            self._notify("当前session/状态实例无效，不能建立PLAN_TOKEN。", "warning")
            return
        self._clear_candidate_approval()
        with self._preview_plan_lock:
            if self._preview_plan_closed:
                self._notify("后台轨迹规划器已关闭。", "warning")
                return
            generation = self._preview_plan_generation
        self.preview_requested_by_operator = True
        try:
            limits_rad = tuple(
                (lower * RAD, upper * RAD) for lower, upper in self.edit_limits
            )
            control = self.config["控制"]
            actual_rad = tuple(float(value) for value in self.actual)
            target_rad = tuple(float(value) for value in self.candidate_targets)
            request_sequence = (
                self.node.reserve_planned_path_request_sequence()
            )
            snapshot = {
                "generation": generation,
                "workflow": self.workflow_contract,
                "actual_rad": actual_rad,
                "target_rad": target_rad,
                "limits_rad": limits_rad,
                "maximum_velocity_rad_s": (
                    float(control["最大速度_度每秒"]) * RAD
                ),
                "maximum_acceleration_rad_s2": (
                    float(control["最大加速度_度每二次方秒"]) * RAD
                ),
                "maximum_segment_delta_rad": (
                    float(COLLISION_EXECUTE_SEGMENT_MAX_DEG) * RAD
                ),
                "maximum_sample_period_s": 0.01,
                "source_instance_id": self.node.command_source_instance_id,
                "request_sequence": request_sequence,
                "session_id": self.session_id,
                "state_instance_id": self.state_instance_id,
                "base_candidate_revision": (
                    self.workflow_contract.candidate_revision
                ),
            }
        except (ContractViolation, KeyError, TypeError, ValueError) as exc:
            self._apply_preview_plan_failure({
                "generation": generation,
                "detail": str(exc),
            })
            return
        self.task_id = secrets.token_hex(8)
        self.task_started_at = time.monotonic()
        self.task_started_wall_utc = datetime.now(timezone.utc).isoformat()
        self.task_execution_started_at = None
        self.task_completed_at = None
        self.task_segment_count = 0
        self.task_completed_segments = 0
        self.last_task_heartbeat = self.task_started_at
        self.preview_animation_started_at = None
        self.preview_animation_complete = False
        self.preview_collision_safe = False
        self.preview_frame_index = 0
        self.preview_pose = actual_rad
        self._set_workflow_state(
            "planning",
            "正在后台生成精确分段五次轨迹；Qt界面保持响应",
            None,
        )
        try:
            future = self._preview_plan_executor.submit(
                build_virtual_preview_plan, snapshot
            )
        except RuntimeError as exc:
            self._apply_preview_plan_failure({
                "generation": generation,
                "detail": f"后台规划器不可用：{exc}",
            })
            return
        with self._preview_plan_lock:
            if (
                self._preview_plan_closed
                or generation != self._preview_plan_generation
            ):
                future.cancel()
                return
            self._preview_plan_future = future
            self._preview_plan_started_at = self.task_started_at
        future.add_done_callback(
            lambda completed, requested_generation=generation:
            self._preview_plan_future_done(requested_generation, completed)
        )

    def _update_preview_animation(self, now: float) -> None:
        if (
            not self.preview_requested_by_operator
            or self.preview_animation_started_at is None
            or self.preview_animation_complete
            or self.workflow_contract.q_plan_trajectory is None
        ):
            return
        trajectory = self.workflow_contract.q_plan_trajectory
        elapsed = max(0.0, now - self.preview_animation_started_at)
        sample_times = [sample.time_s for sample in trajectory.samples]
        index = min(
            len(trajectory.samples) - 1,
            max(0, bisect.bisect_right(sample_times, elapsed) - 1),
        )
        self.preview_frame_index = max(self.preview_frame_index, index)
        sample = trajectory.samples[self.preview_frame_index]
        self.preview_pose = sample.q_rad
        self.last_task_heartbeat = now
        progress = round(100.0 * sample.time_s / trajectory.profile.duration_s)
        if self.preview_frame_index >= len(trajectory.samples) - 1:
            self.preview_animation_complete = True
            self.preview_pose = trajectory.target_rad
            self._try_finalize_preview(now)
        else:
            self._set_workflow_state(
                "previewing",
                f"正在进行虚拟预演；{sample.time_s:.2f}/"
                f"{trajectory.profile.duration_s:.2f}秒",
                progress,
            )

    def _update_planned_execution_pose(self, now_ns: int) -> None:
        """Render the exact dispatched sample, never an endpoint shortcut."""

        segment = self.active_trajectory_segment
        descriptor = self.active_trajectory_descriptor
        if not isinstance(segment, TrajectoryPlan) or not isinstance(
            descriptor, dict
        ):
            return
        trajectory = descriptor.get("trajectory")
        if not isinstance(trajectory, dict):
            return
        try:
            index = trajectory_sample_index_at(
                now_ns,
                execute_at_monotonic_ns=trajectory["execute_at_monotonic_ns"],
                duration_ns=trajectory["duration_ns"],
                interval_count=trajectory["interval_count"],
            )
        except (ContractViolation, KeyError):
            return
        if index >= len(segment.samples):
            return
        self.preview_pose = segment.samples[index].q_rad

    def _current_preview_checks(self, now: float) -> PreviewChecks:
        hardware = self.node.latest_hardware or {}
        per_motor = hardware.get("per_motor", {})
        communication_pass = bool(
            isinstance(per_motor, dict)
            and all(
                isinstance(per_motor.get(name), dict)
                and per_motor[name].get("fresh") is True
                and per_motor[name].get("communication_ok") is True
                and per_motor[name].get("merror") == 0
                for name in MOTOR_NAMES
            )
        )
        thermal_allowed_states = {"NORMAL", "WARNING"}
        thermal_stop_c = getattr(
            getattr(self.node, "thermal_limits", None),
            "thermal_stop_c",
            None,
        )
        thermal_pass = bool(
            communication_pass
            and type(thermal_stop_c) in {int, float}
            and math.isfinite(float(thermal_stop_c))
            and all(
                isinstance(per_motor[name].get("temperature_c"), (int, float))
                and math.isfinite(float(per_motor[name]["temperature_c"]))
                and float(per_motor[name]["temperature_c"])
                < float(thermal_stop_c)
                and per_motor[name].get("thermal_state") in thermal_allowed_states
                for name in MOTOR_NAMES
            )
        )
        gravity = self.node.latest_gravity_status or {}
        gravity_pass = bool(
            self.node.gravity_status_fresh(now)
            and gravity_status_authorizes_hardware(
                gravity,
                session_id=self.session_id,
                state_instance_id=self.state_instance_id,
                now_monotonic_ns=time.monotonic_ns(),
            )
        )
        trajectory = self.workflow_contract.q_plan_trajectory
        planned_load_thermal_pass = bool(
            gravity_pass
            and isinstance(trajectory, TrajectoryRecipe)
            and planned_load_feasibility_authorizes(
                gravity,
                trajectory_sha256=trajectory.sha256,
                session_id=self.session_id,
                state_instance_id=self.state_instance_id,
                now_monotonic_ns=time.monotonic_ns(),
            )
        )
        exact_recipe_proof = bool(
            isinstance(trajectory, TrajectoryRecipe)
            and getattr(self, "preview_collision_segment_sha256", [])
            == [segment.sha256 for segment in trajectory.segments]
        )
        return PreviewChecks(
            trajectory_pass=bool(
                self.preview_animation_complete
                and isinstance(trajectory, TrajectoryRecipe)
            ),
            limits_pass=isinstance(trajectory, TrajectoryRecipe),
            collision_pass=bool(self.preview_collision_safe and exact_recipe_proof),
            gravity_pass=gravity_pass,
            planned_load_thermal_pass=planned_load_thermal_pass,
            thermal_pass=thermal_pass,
            feedback_fresh=self.node.control_streams_fresh(now),
            communication_pass=communication_pass,
        )

    def _try_finalize_preview(self, now: Optional[float] = None) -> None:
        checked_at = time.monotonic() if now is None else now
        if (
            not self.preview_requested_by_operator
            or not self.preview_animation_complete
            or not self.preview_collision_safe
            or self.workflow_contract.q_plan_trajectory is None
        ):
            return
        checks = self._current_preview_checks(checked_at)
        if not checks.complete_success:
            failed = [name for name, passed in checks.as_dict().items() if not passed]
            self.workflow_contract = self.workflow_contract.invalidate_preview()
            self._set_workflow_state(
                "blocked",
                "预演完成但现实下发被阻止：" + "、".join(failed),
                100,
            )
            return
        try:
            self.workflow_contract = self.workflow_contract.accept_successful_preview(
                self.workflow_contract.q_plan_trajectory,
                checks,
                created_monotonic_ns=time.monotonic_ns(),
                nonce=secrets.token_hex(16),
                actual_tolerance_rad=PLAN_ACTUAL_DRIFT_TOLERANCE_RAD,
            )
        except ContractViolation as exc:
            self._set_workflow_state("stale", f"PLAN_TOKEN签发失败：{exc}", 0)
            return
        self._set_workflow_state(
            "safe", "预演通过，等待用户下发；PLAN_TOKEN已锁定", 100
        )

    def _new_collision_request(self, kind: str) -> Optional[dict]:
        if kind not in {"preview", "pose_preview", "plan_preview", "execute"}:
            raise ValueError("碰撞检查类型无效")
        hardware = self.node.latest_hardware
        now_ns = time.monotonic_ns()
        plan_segment = None
        if kind == "plan_preview":
            recipe = self.workflow_contract.q_plan_trajectory
            segment_index = getattr(self, "preview_collision_segment_index", -1)
            if (
                not isinstance(recipe, TrajectoryRecipe)
                or type(segment_index) is not int
                or not 0 <= segment_index < len(recipe.segments)
            ):
                return None
            plan_segment = recipe.segments[segment_index]
            target_values = plan_segment.target_rad
            start_values = plan_segment.start_rad
            moving_indices = [
                index
                for index, (source, target) in enumerate(
                    zip(plan_segment.start_rad, plan_segment.target_rad)
                )
                if source != target
            ]
        else:
            target_values = (
                self.targets if kind == "execute" else self.candidate_targets
            )
            selected_mask = (
                self.pending_target_joint_mask
                if kind == "execute" else self.candidate_joint_mask
            )
            moving_indices = [
                index
                for index, pending in enumerate(selected_mask)
                if pending
            ]
            start_values = None
        if (
            self.direction is not ArmMode.SIM_TO_REAL
            or not self.session_id
            or not source_instance_id_valid(self.state_instance_id)
            or not self.node.control_streams_fresh()
            or not moving_indices
            or not hardware_state_contract_valid(hardware)
            or hardware["session_id"] != self.session_id
            or hardware["state_instance_id"] != self.state_instance_id
        ):
            return None
        if kind == "execute" and (
            self.command_stream_suspended
            # Each execute proof authorizes exactly one physical swept segment.
            or len(moving_indices) != 1
            or not collision_motion_state_ready(
                hardware,
                self.command_targets,
                self.requested_active_joint_mask,
                self.hardware_mode,
                now_ns=now_ns,
            )
        ):
            return None
        if kind in {"preview", "plan_preview"} and len(moving_indices) != 1:
            return None
        if kind == "plan_preview" and not collision_motion_state_ready(
            hardware,
            hardware["position_rad"],
            [True] * 6,
            "hold",
            now_ns=now_ns,
        ):
            return None
        if not moving_targets_within_model_limits(
            target_values, [True] * 6, self.edit_limits
        ):
            return None
        active_segment = getattr(self, "active_trajectory_segment", None)
        if start_values is None:
            start_values = (
                active_segment.start_rad
                if kind == "execute" and isinstance(active_segment, TrajectoryPlan)
                else self.command_targets
                if kind == "execute"
                else hardware["position_rad"]
            )
        self.collision_request_sequence += 1
        payload = {
            "schema": "go-m8010-collision-guard-request/1.0",
            "source_instance_id": self.node.command_source_instance_id,
            "request_sequence": self.collision_request_sequence,
            "source_monotonic_ns": now_ns,
            "kind": kind,
            "session_id": self.session_id,
            "state_instance_id": self.state_instance_id,
            "session_pose_sha256": self.session_pose_sha256,
            "moving_joint_mask": [
                index in moving_indices for index in range(6)
            ],
            "start_relative_rad": [float(value) for value in start_values],
            "target_relative_rad": [float(value) for value in target_values],
            "target_sha256": collision_target_sha256(target_values),
            "collision_margin_deg": COLLISION_MARGIN_DEG,
        }
        self.collision_requests[self.collision_request_sequence] = payload
        if kind == "plan_preview":
            self.preview_collision_request_segment_by_sequence[
                self.collision_request_sequence
            ] = self.preview_collision_segment_index
        # Results are processed monotonically; bound stale request memory too.
        for sequence in sorted(self.collision_requests)[:-16]:
            self.collision_requests.pop(sequence, None)
        self.node.publish_collision_request(payload)
        return payload

    def _request_collision_preview(self) -> None:
        self.candidate_joint_mask = [
            abs(candidate - locked) > TARGET_SELECTION_DEADBAND_RAD
            for candidate, locked in zip(
                self.candidate_targets, self.command_targets
            )
        ]
        moving_count = sum(self.candidate_joint_mask)
        if moving_count < 1:
            self._clear_candidate_approval()
            self._set_workflow_state(
                "idle", "候选姿态与当前锁定目标相同，无需执行", 0
            )
            return
        workflow = getattr(self, "workflow_contract", None)
        trajectory = getattr(workflow, "q_plan_trajectory", None)
        if isinstance(trajectory, TrajectoryRecipe):
            segment_index = getattr(self, "preview_collision_segment_index", 0)
            if segment_index >= len(trajectory.segments):
                self.preview_collision_safe = True
                self.approved_candidate_sha256 = collision_target_sha256(
                    self.candidate_targets
                )
                self.approved_candidate_session_id = self.session_id
                self.approved_candidate_state_instance_id = self.state_instance_id
                if self.preview_animation_complete:
                    self._try_finalize_preview()
                return
            request_kind = "plan_preview"
        else:
            # Compatibility fallback for isolated legacy logic tests.  The
            # production V15.31A path always has an immutable recipe here.
            request_kind = "preview" if moving_count == 1 else "pose_preview"
            segment_index = 0
        request = self._new_collision_request(request_kind)
        if request is None:
            self.preview_collision_safe = False
            self._set_workflow_state(
                "waiting_hardware",
                "候选姿态已在虚拟机械臂中加载；"
                "等待六轴健康、静止且全部HOLD后启动完整3D解算",
                15,
            )
            return
        self.latest_collision_preview_sequence = request["request_sequence"]
        self.collision_preview_started_at = time.monotonic()
        total_segments = (
            len(trajectory.segments)
            if isinstance(trajectory, TrajectoryRecipe) else 1
        )
        self._set_workflow_state(
            "solving",
            f"正在解算预演分段{segment_index + 1}/{total_segments}"
            f"（请求{request['request_sequence']}）；"
            "结果出来前现实执行保持禁用",
            None,
        )

    def _request_collision_execute(self) -> bool:
        self.collision_preview_timer.stop()
        request = self._new_collision_request("execute")
        if request is None:
            self._notify(
                "完整3D模型碰撞检查尚不可用；新目标未下发，原位置保持继续。",
                "warning",
            )
            return False
        self.pending_collision_execute_sequence = request["request_sequence"]
        self._refresh_virtual_editability()
        self._set_workflow_state(
            "execute_proof",
            f"正在复核现实分段的3D路径（请求{request['request_sequence']}）",
            None,
        )
        self._update_mode_label(
            "正在按历史3D模型预演下一个单关节物理分段；"
            "其余关节原位置HOLD继续"
        )
        return True

    def _cancel_queued_pose(self, *, restore_command_target: bool) -> None:
        # Invalidate an in-flight execute request as well.  A late result may
        # still arrive, but _consume_collision_guard_result can no longer turn
        # it into hardware authority after an operator/fault cancellation.
        self.pending_collision_execute_sequence = None
        self.collision_preview_timer.stop()
        self.queued_pose_target = None
        self.queued_joint_indices = []
        self.queued_trajectory_segments = []
        self.trajectory_segment_count = 0
        self.active_trajectory_segment = None
        self.active_trajectory_segment_index = None
        self.active_trajectory_descriptor = None
        self.active_plan_manifest = None
        self.active_trajectory_first_publish_pending = False
        self.active_sequence_joint = None
        clear_approval = getattr(self, "_clear_candidate_approval", None)
        if clear_approval is not None:
            clear_approval()
        if restore_command_target:
            self.targets = list(self.command_targets)
            self.candidate_targets = list(self.command_targets)
            self.candidate_joint_mask = [False] * 6
            self.pending_target_joint_mask = [False] * 6
            self._show_targets_on_virtual()
        self._refresh_virtual_editability()

    def _begin_next_queued_segment(self) -> None:
        """Authorize one physical joint from a multi-joint virtual target.

        Current workers do not share an atomic multi-domain start.  The GUI
        therefore accepts a complete virtual pose in one operation, but emits
        a sequence of individually swept and signed physical segments.  This
        removes manual one-joint-at-a-time editing without claiming that four
        independent UDP domains execute one synchronized collision proof.
        """

        if (
            self.queued_pose_target is None
            or self.active_sequence_joint is not None
            or self.pending_collision_execute_sequence is not None
        ):
            return
        while self.queued_trajectory_segments:
            segment = self.queued_trajectory_segments.pop(0)
            changed = [
                index
                for index, (source, target) in enumerate(
                    zip(segment.start_rad, segment.target_rad)
                )
                if source != target
            ]
            if len(changed) != 1 or any(
                abs(source - command) > 1.0e-12
                for source, command in zip(
                    segment.start_rad, self.command_targets
                )
            ):
                self._notify(
                    "已预演分段与当前锁定目标不再一致；剩余计划已作废。",
                    "warning",
                )
                self._cancel_queued_pose(restore_command_target=True)
                return
            index = changed[0]
            self.active_trajectory_segment = segment
            self.active_trajectory_segment_index = (
                self.trajectory_segment_count
                - len(self.queued_trajectory_segments)
                - 1
            )
            self.preview_pose = segment.start_rad
            self.targets = list(segment.target_rad)
            self.pending_target_joint_mask = [False] * 6
            self.pending_target_joint_mask[index] = True
            self.active_sequence_joint = index
            self.queued_joint_indices = [
                next(
                    joint
                    for joint, (source, target) in enumerate(
                        zip(queued.start_rad, queued.target_rad)
                    )
                    if source != target
                )
                for queued in self.queued_trajectory_segments
            ]
            self._show_targets_on_virtual()
            if self._request_collision_execute():
                self._notify(
                    f"正在检查已预演分段"
                    f"{self.active_trajectory_segment_index + 1}/"
                    f"{self.trajectory_segment_count}：J{index + 1}，"
                    "其余关节保持原锁定角度。",
                    "info",
                )
                return
            self._cancel_queued_pose(restore_command_target=True)
            return

        if getattr(self, "task_execution_started_at", None) is not None:
            self.task_completed_segments = max(
                getattr(self, "task_completed_segments", 0),
                getattr(self, "task_segment_count", 0),
            )
            if getattr(self, "task_completed_at", None) is None:
                self.task_completed_at = time.monotonic()
        self._cancel_queued_pose(restore_command_target=True)
        self._set_workflow_state(
            "complete",
            "候选姿态的所有安全分段已完成；六轴继续闭环保持",
            100,
        )
        self._notify(
            "虚拟目标的所有安全分段均已完成；六轴继续闭环保持最终角度。",
            "info",
        )

    def _continue_queued_sequence_if_ready(self) -> None:
        if (
            self.queued_pose_target is not None
            and self.active_sequence_joint is None
            and self.pending_collision_execute_sequence is None
            and not self.command_stream_suspended
            and collision_motion_state_ready(
                self.node.latest_hardware,
                self.command_targets,
                self.requested_active_joint_mask,
                self.hardware_mode,
            )
        ):
            self._begin_next_queued_segment()

    def _abort_queued_pose_after_collision(
        self, result: dict, request: dict
    ) -> None:
        # The current servo authority stays untouched; only the remaining
        # virtual plan is discarded.
        self._cancel_queued_pose(restore_command_target=True)
        if result.get("reason") in {
            "self_collision", "self_collision_margin",
            "ground_collision", "ground_collision_margin",
        }:
            self._show_collision_popup(result)
        self._notify(
            "虚拟目标的下一分段未通过完整3D安全检查；剩余序列已停止，"
            "现实机械臂继续保持最后一个已确认角度。",
            "warning",
        )

    def _collision_recommendation(self, result: dict, request: dict) -> list[float]:
        recommendation = result.get("recommended_relative_rad")
        if (
            not isinstance(recommendation, list)
            or len(recommendation) != 6
            or not all(
                type(value) in {int, float} and math.isfinite(float(value))
                for value in recommendation
            )
        ):
            recommendation = request["start_relative_rad"]
        values = [float(value) for value in recommendation]
        moving_mask = request.get("moving_joint_mask")
        if (
            isinstance(moving_mask, list)
            and len(moving_mask) == 6
            and all(type(value) is bool for value in moving_mask)
        ):
            # A recommendation may fall back to measured start feedback.  It
            # must never turn a support-axis HOLD tracking error into a new
            # virtual edit; support targets remain the immutable request target.
            values = [
                value if moving else float(request["target_relative_rad"][index])
                for index, (value, moving) in enumerate(zip(values, moving_mask))
            ]
        if not moving_targets_within_model_limits(
            values, [True] * 6, self.edit_limits
        ):
            return [float(value) for value in request["start_relative_rad"]]
        return values

    def _show_collision_popup(self, result: dict) -> None:
        self_collision = str(result.get("reason", "")).startswith(
            "self_collision"
        )
        pairs = result.get("contact_pairs")
        pair_text = ""
        if isinstance(pairs, list) and pairs:
            formatted = []
            for pair in pairs[:3]:
                if isinstance(pair, list) and len(pair) == 2:
                    formatted.append(f"{pair[0]} ↔ {pair[1]}")
            if formatted:
                pair_text = "\n碰撞部件：" + "；".join(formatted)
        signature = json.dumps(
            [result.get("reason"), pairs], ensure_ascii=False, sort_keys=True
        )
        if signature == self.last_collision_popup_signature:
            return
        self.last_collision_popup_signature = signature
        if self.collision_popup is not None:
            self.collision_popup.close()
        dialog = QMessageBox(self)
        dialog.setWindowTitle(
            "不允许机械臂互撞" if self_collision else "不允许模型地面碰撞"
        )
        dialog.setIcon(QMessageBox.Icon.Warning)
        dialog.setText(
            (
                "该滑动目标会进入机械臂自碰撞区域，"
                if self_collision
                else "该滑动目标会进入模型地面碰撞区域，"
            )
            + "已按历史3D模型拒绝，"
            f"并沿本次虚拟预演路径预留至少{COLLISION_MARGIN_DEG:.0f}°余量。\n"
            "目标未下发；现有POSITION/HOLD及原锁定角度保持不变。"
            + pair_text
        )
        dialog.setStandardButtons(QMessageBox.StandardButton.Ok)
        dialog.setModal(False)
        dialog.finished.connect(self._collision_popup_finished)
        self.collision_popup = dialog
        dialog.open()

    def _collision_popup_finished(self, _result: int) -> None:
        self.collision_popup = None
        # Deduplicate only while a dialog is actually open.  A later operator
        # edit that reaches the same forbidden geometry must show a new popup.
        self.last_collision_popup_signature = None

    def _apply_collision_rejection(self, result: dict, request: dict) -> None:
        # Only the editable virtual target is rolled back.  Never mutate the
        # authoritative command target/mode/masks/epoch: existing position
        # torque must continue while an unsafe candidate is rejected.
        current_candidate = getattr(self, "candidate_targets", self.targets)
        if any(
            abs(current - requested) > 1.0e-12
            for current, requested in zip(
                current_candidate, request["target_relative_rad"]
            )
        ):
            return
        self.candidate_targets = self._collision_recommendation(result, request)
        self.targets = list(self.candidate_targets)
        self.candidate_joint_mask = [
            abs(target - locked) > TARGET_SELECTION_DEADBAND_RAD
            for target, locked in zip(
                self.candidate_targets, self.command_targets
            )
        ]
        self.pending_target_joint_mask = [False] * 6
        clear_approval = getattr(self, "_clear_candidate_approval", None)
        if clear_approval is not None:
            clear_approval()
        self._show_targets_on_virtual()
        self._show_collision_popup(result)
        collision_text = (
            "不允许机械臂互撞"
            if str(result.get("reason", "")).startswith("self_collision")
            else "不允许模型地面碰撞"
        )
        self._notify(
            f"{collision_text}：候选目标已回退；原位置伺服保持继续。",
            "warning",
        )
        # Do not immediately re-submit the rejected pose.  In particular, an
        # already-unsafe start has no model-safe recommendation and would
        # otherwise create a 60 ms fail-closed request loop.  A new operator
        # edit is the only event that starts another preview.

    def _consume_collision_guard_result(self) -> None:
        result = self.node.latest_collision_result
        if not isinstance(result, dict):
            return
        sequence = result.get("request_sequence")
        if type(sequence) is not int or sequence <= self.last_consumed_collision_sequence:
            return
        request = self.collision_requests.get(sequence)
        if request is None or not collision_guard_result_matches(result, request):
            return
        self.last_consumed_collision_sequence = sequence
        kind = request["kind"]
        if kind == "plan_preview":
            if sequence != getattr(self, "latest_collision_preview_sequence", None):
                return
            workflow = getattr(self, "workflow_contract", None)
            recipe = getattr(workflow, "q_plan_trajectory", None)
            segment_by_sequence = getattr(
                self, "preview_collision_request_segment_by_sequence", {}
            )
            segment_index = segment_by_sequence.get(sequence)
            candidate_still_matches = bool(
                isinstance(recipe, TrajectoryRecipe)
                and type(segment_index) is int
                and segment_index
                == getattr(self, "preview_collision_segment_index", -1)
                and 0 <= segment_index < len(recipe.segments)
                and request["session_id"] == self.session_id
                and request["state_instance_id"] == self.state_instance_id
                and collision_target_sha256(self.candidate_targets)
                == collision_target_sha256(recipe.target_rad)
                and tuple(request["start_relative_rad"])
                == recipe.segments[segment_index].start_rad
                and tuple(request["target_relative_rad"])
                == recipe.segments[segment_index].target_rad
            )
            if not candidate_still_matches:
                self._clear_candidate_approval()
                self._set_workflow_state(
                    "stale",
                    "分段预演返回时recipe、候选姿态或会话已变化；结果已作废",
                    0,
                )
                return
            assert isinstance(recipe, TrajectoryRecipe)
            assert type(segment_index) is int
            if not result["safe"]:
                self._clear_candidate_approval()
                self._notify(
                    f"第{segment_index + 1}个预演分段未通过3D碰撞／余量检查；"
                    "现实执行已禁用。",
                    "warning",
                )
                self._set_workflow_state(
                    "unsafe",
                    f"预演分段{segment_index + 1}/{len(recipe.segments)}"
                    f"未通过：{result.get('reason', '未知原因')}",
                    0,
                )
                return
            self.preview_collision_segment_sha256.append(
                recipe.segments[segment_index].sha256
            )
            self.preview_collision_segment_index = segment_index + 1
            if self.preview_collision_segment_index < len(recipe.segments):
                self._set_workflow_state(
                    "solving",
                    f"已通过{self.preview_collision_segment_index}/"
                    f"{len(recipe.segments)}个轨迹分段；正在检查下一段",
                    round(
                        100.0
                        * self.preview_collision_segment_index
                        / len(recipe.segments)
                    ),
                )
                self._request_collision_preview()
                return
            self.preview_collision_safe = True
            self.approved_candidate_sha256 = collision_target_sha256(
                self.candidate_targets
            )
            self.approved_candidate_session_id = self.session_id
            self.approved_candidate_state_instance_id = self.state_instance_id
            if self.preview_animation_complete:
                self._try_finalize_preview()
            else:
                self._set_workflow_state(
                    "previewing",
                    f"全部{len(recipe.segments)}个预演轨迹分段已通过碰撞检查；"
                    "正在继续播放虚拟动画",
                    None,
                )
            return
        if kind in {"preview", "pose_preview"}:
            if sequence != getattr(self, "latest_collision_preview_sequence", None):
                return
            candidate = getattr(self, "candidate_targets", self.targets)
            candidate_still_matches = bool(
                request["session_id"] == self.session_id
                and request["state_instance_id"] == self.state_instance_id
                and request["target_sha256"]
                == collision_target_sha256(candidate)
            )
            if not candidate_still_matches:
                self._clear_candidate_approval()
                self._set_workflow_state(
                    "stale",
                    "预演返回时候选姿态或会话已变化；结果已作废",
                    0,
                )
                return
            if result["safe"]:
                self.last_collision_popup_signature = None
                self.preview_collision_safe = True
                self.approved_candidate_sha256 = request["target_sha256"]
                self.approved_candidate_session_id = request["session_id"]
                self.approved_candidate_state_instance_id = request[
                    "state_instance_id"
                ]
                pose_count = result.get("pose_check_count")
                pose_text = (
                    f"，已检查{pose_count}个姿态"
                    if type(pose_count) is int and pose_count >= 0 else ""
                )
                elapsed = max(
                    0.0,
                    time.monotonic()
                    - getattr(self, "collision_preview_started_at", 0.0),
                )
                if self.preview_animation_complete:
                    self._try_finalize_preview()
                else:
                    trajectory = self.workflow_contract.q_plan_trajectory
                    progress = (
                        round(
                            100.0 * self.preview_frame_index
                            / max(1, len(trajectory.samples) - 1)
                        )
                        if trajectory is not None else 0
                    )
                    self._set_workflow_state(
                        "previewing",
                        f"碰撞检查已通过{pose_text}，用时{elapsed:.2f}秒；"
                        "正在继续播放完整计划轨迹",
                        progress,
                    )
            elif result.get("reason") in {
                "self_collision", "self_collision_margin",
                "ground_collision", "ground_collision_margin",
            }:
                self._apply_collision_rejection(result, request)
                self._set_workflow_state(
                    "unsafe",
                    "UNSAFE：候选姿态未通过3D碰撞／余量检查；现实执行已禁用",
                    0,
                )
            else:
                self._clear_candidate_approval()
                self._set_workflow_state(
                    "unsafe",
                    f"预演未通过：{result.get('reason', '未知原因')}；现实执行已禁用",
                    0,
                )
            return
        if sequence != self.pending_collision_execute_sequence:
            return
        self.pending_collision_execute_sequence = None
        self._refresh_virtual_editability()
        if not result["safe"]:
            if self.queued_pose_target is not None:
                self._abort_queued_pose_after_collision(result, request)
            elif result.get("reason") in {
                    "self_collision", "self_collision_margin",
                    "ground_collision", "ground_collision_margin",
            }:
                self._apply_collision_rejection(result, request)
            else:
                self._notify(
                    "3D模型路径检查未通过；新目标未下发，原位置保持继续。",
                    "warning",
                )
            self._set_workflow_state(
                "unsafe", "现实分段复核未通过；轨迹已取消", 0
            )
            return
        if (
            self.session_id != request["session_id"]
            or self.state_instance_id != request["state_instance_id"]
            or any(
                abs(current - requested) > 1.0e-12
                for current, requested in zip(
                    self.targets, request["target_relative_rad"]
                )
            )
            or any(
                abs(current - checked) > COLLISION_START_MATCH_TOLERANCE_RAD
                for current, checked in zip(
                    self.actual, request["start_relative_rad"]
                )
            )
        ):
            if self.queued_pose_target is not None:
                self._cancel_queued_pose(restore_command_target=True)
            self._notify(
                "姿态或会话在碰撞检查期间发生变化；"
                "已取消剩余序列，原位置保持继续。",
                "warning",
            )
            self._set_workflow_state(
                "stale", "复核期间姿态／会话变化；轨迹已取消", 0
            )
            return
        if not self._commit_checked_target(request, result):
            if self.queued_pose_target is not None:
                self._cancel_queued_pose(restore_command_target=True)
            self._notify(
                "碰撞证明通过后的二次状态校验失败；"
                "已取消剩余序列，旧目标保持不变。",
                "warning",
            )
            self._set_workflow_state(
                "blocked", "复核后的二次现实状态校验失败；轨迹已取消", 0
            )

    def _expire_collision_request(self, now: float) -> None:
        preview_sequence = getattr(self, "latest_collision_preview_sequence", None)
        if (
            getattr(self, "collision_preview_state", "idle") == "solving"
            and preview_sequence is not None
        ):
            preview_request = self.collision_requests.get(preview_sequence)
            if (
                preview_request is None
                or now - preview_request["source_monotonic_ns"] * 1.0e-9
                > COLLISION_GUARD_TIMEOUT_S
            ):
                self._clear_candidate_approval()
                self._set_workflow_state(
                    "timeout",
                    "完整3D候选姿态预演超时；该请求的迟到结果不会授权现实运动",
                    0,
                )
        sequence = self.pending_collision_execute_sequence
        if sequence is None:
            return
        request = self.collision_requests.get(sequence)
        if request is None:
            self.pending_collision_execute_sequence = None
            if self.queued_pose_target is not None:
                self._cancel_queued_pose(restore_command_target=True)
            self._set_workflow_state(
                "timeout",
                "现实分段复核请求已丢失；轨迹已取消，迟到结果不会授权运动",
                0,
            )
            return
        age = now - request["source_monotonic_ns"] * 1.0e-9
        if age <= COLLISION_GUARD_TIMEOUT_S:
            return
        self.pending_collision_execute_sequence = None
        if self.queued_pose_target is not None:
            self._cancel_queued_pose(restore_command_target=True)
        self._refresh_virtual_editability()
        self._notify(
            "3D碰撞检查超时；新目标未下发，现有POSITION/HOLD保持继续。",
            "warning",
        )
        self._set_workflow_state(
            "timeout",
            "现实分段3D碰撞复核超时；轨迹已取消，迟到结果不会授权运动",
            0,
        )

    def _real_to_sim(self) -> None:
        self._cancel_queued_pose(restore_command_target=True)
        was_positioning = self.hardware_mode == "position"
        self.direction = ArmMode.REAL_TO_SIM
        self.machine.real_to_sim()
        self.pending_target_joint_mask = [False] * 6
        self._set_virtual_editable(False)
        if was_positioning:
            now = time.monotonic()
            streams_fresh = self.node.control_streams_fresh(now)
            self._update_connected(now)
            if (
                not self.command_stream_suspended
                and control_feedback_ready(
                    self.have_first_state, streams_fresh, self.connected
                )
            ):
                self._transition_position_to_fixed_hold()
            else:
                self._suspend_command_stream(
                    "方向切换时无法确认新鲜反馈；保留底层租约安全保持"
                )
        elif self.hardware_mode in {"brake", "drag"}:
            self.targets = list(self.actual)
            self.command_targets = list(self.actual)
            self.candidate_targets = list(self.actual)
            self._show_targets_on_virtual()
        self._update_mode_label("方向切换不会撤销当前承重保持")

    def _sim_to_real(self) -> None:
        self._cancel_queued_pose(restore_command_target=True)
        was_positioning = self.hardware_mode == "position"
        self.direction = ArmMode.SIM_TO_REAL
        self.machine.sim_to_real()
        self.pending_target_joint_mask = [False] * 6
        if was_positioning:
            now = time.monotonic()
            streams_fresh = self.node.control_streams_fresh(now)
            self._update_connected(now)
            if (
                not self.command_stream_suspended
                and control_feedback_ready(
                    self.have_first_state, streams_fresh, self.connected
                )
            ):
                self._transition_position_to_fixed_hold()
            else:
                self._suspend_command_stream(
                    "方向切换时无法确认新鲜反馈；保留底层租约安全保持"
                )
        elif self.hardware_mode in {"brake", "drag"}:
            self.targets = list(self.actual)
            self.command_targets = list(self.actual)
            self.candidate_targets = list(self.actual)
            self._show_targets_on_virtual()
        self._set_virtual_editable(True)
        self._update_mode_label("方向切换不会撤销当前承重保持")

    def _position_mode(self) -> None:
        if not self._require_control_feedback(
            "进入现实位置模式要求六个关节均有新鲜、健康反馈。",
            require_all=True,
        ):
            return
        self.direction = ArmMode.SIM_TO_REAL
        if self.hardware_mode == "position":
            self._notify(
                "POSITION目标仍在持续闭环保持；再次点击位置模式不会把外力偏移"
                "采成新目标。如需另设目标，请先使用“停止轨迹并保持”。",
                "info",
            )
            return
        elif self.hardware_mode in {"brake", "drag"} or not all(
            self.requested_active_joint_mask
        ):
            if self.hardware_mode in {"brake", "drag"}:
                self._prepare_hold_at_actual()
            else:
                self._prepare_retakeover_hold_preserving_active_targets()
        else:
            # Re-entering target editing from an established HOLD must never
            # adopt an externally displaced feedback sample as the new target.
            self.active_collision_proof = None
            # A transient stream timeout latches publication off.  Once this
            # explicit operator action has revalidated all six axes, resume the
            # exact established HOLD target/mask/epoch instead of leaving the
            # button and editor permanently disabled.
            self._resume_command_stream()
        self._set_virtual_editable(True)
        self._update_mode_label(
            "位置目标可编辑；六轴刚性HOLD已确认后先由虚拟机械臂预演"
        )

    def _fixed_hold_after_arrival_on(self) -> None:
        self._set_fixed_hold_after_arrival(True)

    def _fixed_hold_after_arrival_off(self) -> None:
        self._set_fixed_hold_after_arrival(False)

    def _set_fixed_hold_after_arrival(self, enabled: bool) -> None:
        """Change only the post-arrival policy while position servo stays active.

        HOLD/POSITION already own an immutable target and activation epoch, so a
        policy toggle must not recapture feedback or mutate any authority mask.
        BRAKE/DRAG first enter a fresh-feedback current-angle HOLD to avoid a
        discontinuous position-servo enable, then expose target editing.
        """

        entering_position_servo = self.hardware_mode in {"brake", "drag"}
        if entering_position_servo and not self._require_control_feedback(
            "从制动／可拖动状态进入位置伺服需要新鲜硬件反馈，"
            "且六个关节均健康连接。",
            require_all=True,
        ):
            return

        self.machine.set_fixed_hold_after_arrival(enabled)
        self.direction = ArmMode.SIM_TO_REAL
        if entering_position_servo:
            self._prepare_hold_at_actual()
        self._set_virtual_editable(True)

        policy = "开" if enabled else "关"
        behavior = (
            "目标到位后转为固定目标HOLD"
            if enabled else
            "目标到位后继续发送POSITION目标"
        )
        entry = (
            "已基于新鲜反馈无冲击进入当前角度HOLD；"
            if entering_position_servo else
            "原有HOLD／POSITION目标、掩码与激活纪元均保持不变；"
        )
        message = (
            f"到位后固定保持策略已{policy}；{entry}{behavior}。"
            "位置伺服始终会产生必要驱动力，策略关闭不是物理脱力。"
        )
        self._update_mode_label(message)
        self._notify(message, "info")

    def _enter_drag_after_confirmation(self) -> bool:
        now = time.monotonic()
        self._update_connected(now)
        if not self.node.control_streams_fresh(now):
            self._suspend_for_stale_feedback()
            self._notify(
                "关节或硬件状态数据不新鲜，GUI未发送新的脱力命令；"
                "底层状态尚未确认，请继续可靠支撑机械臂。",
                "critical",
            )
            return False
        self._cancel_queued_pose(restore_command_target=True)
        self.direction = ArmMode.REAL_TO_SIM
        self.machine.drag()
        self.hardware_mode = "drag"
        self.active_collision_proof = None
        self.pending_target_joint_mask = [False] * 6
        self.moving_joint_mask = [False] * 6
        self._authorize_active_joints(self.connected)
        self._resume_command_stream()
        self._set_virtual_editable(False)
        self._update_mode_label("已请求关闭驱动力；等待硬件确认，24V／主电源仍可能接通")
        return True

    def _drag_mode(self) -> None:
        dialog = QMessageBox(self)
        dialog.setWindowTitle("可拖动模式安全提示")
        dialog.setText(
            "当前为无重力补偿拖动模式。\nJ2／J3 等重载关节可能因机械臂自重下落，\n"
            "请可靠支撑机械臂后操作。"
        )
        confirm = dialog.addButton("确认进入", QMessageBox.ButtonRole.AcceptRole)
        dialog.addButton("取消", QMessageBox.ButtonRole.RejectRole)
        dialog.exec()
        if dialog.clickedButton() is not confirm:
            return
        if self._enter_drag_after_confirmation():
            self._update_mode_label(
                "已请求可拖动模式；等待硬件确认，请持续可靠支撑机械臂"
            )

    def _emergency_brake(self) -> None:
        """Explicitly withdraw drive authority without any feedback gate."""

        dialog = QMessageBox(self)
        dialog.setWindowTitle("紧急制动／撤销驱动确认")
        dialog.setText(
            "此操作将从任何当前状态立即发送真实BRAKE／DISABLE，\n"
            "撤销位置伺服和承重保持。重载关节可能因自重下落。\n"
            "只有在机械臂已可靠支撑时才可确认。"
        )
        confirm = dialog.addButton("已可靠支撑，立即制动", QMessageBox.ButtonRole.AcceptRole)
        dialog.addButton("取消", QMessageBox.ButtonRole.RejectRole)
        dialog.exec()
        if dialog.clickedButton() is not confirm:
            return

        self._cancel_queued_pose(restore_command_target=True)
        # BRAKE is the one command that deliberately bypasses state freshness,
        # active-observation and replay ownership gates.  The router normalizes
        # it again, and each worker maps it to its real vendor stop primitive.
        self.machine.stop()
        self.hardware_mode = "brake"
        self.active_collision_proof = None
        self.direction = ArmMode.REAL_TO_SIM
        self.requested_active_joint_mask = [False] * 6
        self.pending_target_joint_mask = [False] * 6
        self.moving_joint_mask = [False] * 6
        self._resume_command_stream()
        self._set_virtual_editable(False)
        self._update_mode_label(
            "已立即请求真实BRAKE／DISABLE；它不等于承重保持"
        )
        self._notify(
            "紧急制动已发送；请继续可靠支撑，并以控制器BRAKE／DISABLED反馈为准。",
            "critical",
        )
        self._publish_command()

    def _hold_current(self) -> None:
        if not self._require_control_feedback(
            "建立当前姿态HOLD需要六个关节均有新鲜、健康反馈。",
            require_all=True,
        ):
            return
        self._prepare_hold_at_actual()
        self._update_mode_label("已请求健康关节保持当前位置；等待硬件确认")

    def _prepare_hold_at_actual(self) -> None:
        """Freeze fresh feedback as a HOLD request without claiming confirmation."""
        self._cancel_queued_pose(restore_command_target=False)
        self.targets = list(self.actual)
        self.command_targets = list(self.actual)
        self.candidate_targets = list(self.actual)
        # HOLD is captured exactly once. Feedback refresh must never move this
        # setpoint: an external displacement must create a restoring error
        # instead of becoming the next target.
        self._set_virtual_editable(False)
        self._show_targets_on_virtual()
        self.pending_target_joint_mask = [False] * 6
        self.moving_joint_mask = [False] * 6
        self.active_collision_proof = None
        self._authorize_active_joints(self.connected)
        self.machine.hold()
        self.hardware_mode = "hold"
        self.active_collision_proof = None
        self._resume_command_stream()
        self.arrival.start(time.monotonic())

    def _prepare_retakeover_hold_preserving_active_targets(self) -> None:
        """Activate recovered axes without recapturing already held targets."""

        self._cancel_queued_pose(restore_command_target=False)
        preserved_active = list(self.requested_active_joint_mask)
        merged_targets = [
            locked if active else measured
            for locked, measured, active in zip(
                self.command_targets, self.actual, preserved_active
            )
        ]
        self.targets = list(merged_targets)
        self.command_targets = list(merged_targets)
        self.candidate_targets = list(merged_targets)
        self._set_virtual_editable(False)
        self._show_targets_on_virtual()
        self.pending_target_joint_mask = [False] * 6
        self.moving_joint_mask = [False] * 6
        self.active_collision_proof = None
        self._authorize_active_joints(self.connected)
        self.machine.hold()
        self.hardware_mode = "hold"
        self._resume_command_stream()
        self.arrival.start(time.monotonic())

    def _transition_position_to_fixed_hold(self) -> None:
        """Stop moving axes without re-capturing already fixed load-bearing axes."""

        self._cancel_queued_pose(restore_command_target=True)
        self.command_targets = fixed_hold_targets_after_position_stop(
            self.command_targets,
            self.actual,
            self.requested_active_joint_mask,
            self.moving_joint_mask,
        )
        self.targets = list(self.command_targets)
        self.candidate_targets = list(self.command_targets)
        self._set_virtual_editable(False)
        self._show_targets_on_virtual()
        self.pending_target_joint_mask = [False] * 6
        self.moving_joint_mask = [False] * 6
        # Keep the activation epoch.  A previously fixed axis then retains
        # the exact immutable target recorded for that epoch; only an axis
        # that was moving captures its fresh stop angle.
        self.machine.hold()
        self.hardware_mode = "hold"
        self.active_collision_proof = None
        self._resume_command_stream()
        self.arrival.start(time.monotonic())

    def _execute_target(self) -> None:
        if not self._preview_approval_matches_candidate():
            self._notify(
                "现实下发已拒绝：必须先由“预演轨迹”完成当前候选的完整预演并取得PLAN_TOKEN。",
                "warning",
            )
            return
        if not self._require_control_feedback(
            "执行现实目标要求六个关节均有新鲜、健康反馈。",
            require_all=True,
        ):
            return
        checks = self._current_preview_checks(time.monotonic())
        if not checks.complete_success:
            self._clear_candidate_approval()
            self._notify(
                "现实下发前复核失败：编码器、通信、温度、重力authority或碰撞状态已变化。",
                "critical",
            )
            return
        token = self.workflow_contract.current_plan_token
        trajectory = self.workflow_contract.q_plan_trajectory
        if token is None or not isinstance(trajectory, TrajectoryRecipe):
            self._notify("PLAN_TOKEN已失效，必须重新预演。", "warning")
            return
        if self.direction is not ArmMode.SIM_TO_REAL:
            self._notify("请先选择“虚拟驱动现实”。")
            return
        if (
            self.hardware_mode == "position"
            or any(self.moving_joint_mask)
            or self.queued_pose_target is not None
        ):
            self._notify(
                "已有POSITION轨迹正在保持／执行；请先停止轨迹并确认全轴HOLD，"
                "原目标和力矩保持未改变。",
                "warning",
            )
            return

        prospective_active_mask = list(self.connected)
        if self.command_stream_suspended:
            hardware = self.node.latest_hardware
            controller_modes = (
                hardware.get("controller_mode_by_motor", {})
                if isinstance(hardware, dict) else {}
            )
            if not all(controller_modes.get(name) == "hold" for name in MOTOR_NAMES):
                self._clear_candidate_approval()
                self._notify(
                    "GUI命令流仍暂停且七电机未全部确认HOLD；未授权现实运动。",
                    "warning",
                )
                return
            # ``require_all=True`` above guarantees all six logical joints are
            # freshly observed.  Preserve that checked connectivity explicitly
            # instead of pre-arming a literal all-true mask that could outlive
            # a future topology or health-policy change.
            prospective_active_mask = list(self.connected)
        elif self.hardware_mode != "hold":
            self._notify(
                "现实下发前必须处于全轴HOLD；PLAN_TOKEN尚未消费。",
                "warning",
            )
            return

        # The recipe is bound to the encoder pose used for preview.  Explicit
        # submit deliberately rebases the next HOLD/trajectory epoch to that
        # checked pose (within the plan drift tolerance), rather than requiring
        # mathematically exact equality with an older fixed-HOLD target.
        execution_start = list(trajectory.start_rad)
        final_target = list(trajectory.target_rad)
        pending_indices = [
            index
            for index, (target, start, connected) in enumerate(zip(
                final_target, execution_start, self.connected
            ))
            if connected
            and abs(target - start) > TARGET_SELECTION_DEADBAND_RAD
        ]
        if not pending_indices:
            self._notify(
                "候选目标与预演起点在选择死区内；PLAN_TOKEN尚未消费。",
                "info",
            )
            return
        if not collision_motion_state_ready(
            self.node.latest_hardware,
            execution_start,
            prospective_active_mask,
            "hold",
        ):
            self._notify(
                "现实下发前必须确认六轴均健康、静止并处于承重HOLD；"
                "新目标尚未授权，原锁定角度继续保持。",
                "warning",
            )
            return
        if not moving_targets_within_model_limits(
            final_target, [True] * 6, self.edit_limits
        ):
            self._notify(
                "目标超出历史3D模型关节范围；新目标未发布，原位置保持继续。",
                "warning",
            )
            self._update_mode_label("模型机械范围外目标已拒绝；原位置保持继续")
            return

        # Consume the one-shot token only after every ordinary runtime gate
        # above has succeeded.  A retryable HOLD/mode/proof readiness failure
        # must not silently exhaust operator approval.
        try:
            self.workflow_contract = self.workflow_contract.explicit_real_submit(
                token.token_id,
                actual_tolerance_rad=PLAN_ACTUAL_DRIFT_TOLERANCE_RAD,
            )
        except (ContractViolation, RealSubmitRejected) as exc:
            self._clear_candidate_approval()
            self._notify(f"现实下发已拒绝：{exc}", "warning")
            return
        assert self.workflow_contract.q_hardware_command is not None
        self.preview_requested_by_operator = False
        self.preview_animation_started_at = None
        self.targets = list(self.workflow_contract.q_hardware_command)
        self.command_targets = execution_start
        self._authorize_active_joints(prospective_active_mask)
        self.hardware_mode = "hold"
        if self.command_stream_suspended:
            self._resume_command_stream()
        self.pending_target_joint_mask = [
            index in pending_indices for index in range(6)
        ]
        # Consume the exact immutable recipe already animated in MuJoCo.  No
        # endpoint is re-segmented or re-timed after explicit submission.
        self.queued_pose_target = list(self.targets)
        self.queued_trajectory_segments = list(trajectory.segments)
        self.trajectory_segment_count = len(trajectory.segments)
        self.task_execution_started_at = time.monotonic()
        self.task_completed_at = None
        self.task_segment_count = self.trajectory_segment_count
        self.task_completed_segments = 0
        self.active_plan_manifest = trajectory_plan_manifest(trajectory)
        self.queued_joint_indices = [
            next(
                index
                for index, (source, target) in enumerate(
                    zip(segment.start_rad, segment.target_rad)
                )
                if source != target
            )
            for segment in trajectory.segments
        ]
        self.active_sequence_joint = None
        self.active_trajectory_segment = None
        self.active_trajectory_segment_index = None
        self.active_trajectory_descriptor = None
        self.active_trajectory_first_publish_pending = False
        self._begin_next_queued_segment()

    def _commit_checked_target(
        self, checked_request: dict, approved_result: dict
    ) -> bool:
        """Commit only a matching, fresh full-model collision approval."""

        if not self._require_control_feedback(
            "碰撞检查完成时六轴反馈已不可用；新目标未下发，原位置保持继续。",
            require_all=True,
        ):
            return False
        pending_indices = [
            index for index, pending in enumerate(self.pending_target_joint_mask)
            if pending
        ]
        if (
            self.direction is not ArmMode.SIM_TO_REAL
            or checked_request.get("kind") != "execute"
            or not collision_guard_result_matches(
                approved_result, checked_request
            )
            or collision_target_sha256(self.targets)
            != checked_request.get("target_sha256")
            or len(pending_indices) != 1
            or self.active_sequence_joint != pending_indices[0]
            or self.hardware_mode != "hold"
            or any(self.moving_joint_mask)
            or self.command_stream_suspended
            or not collision_motion_state_ready(
                self.node.latest_hardware,
                self.command_targets,
                self.requested_active_joint_mask,
                self.hardware_mode,
            )
        ):
            return False
        candidate_moving_joint_mask = [
            bool(connected and pending)
            for pending, connected in zip(
                self.pending_target_joint_mask, self.connected
            )
        ]
        if sum(candidate_moving_joint_mask) != 1:
            return False
        if not moving_targets_within_model_limits(
            self.targets, candidate_moving_joint_mask, self.edit_limits
        ):
            unchanged = {
                "hold": "继续保持原锁定目标",
                "position": "现有位置轨迹与移动掩码保持不变",
                "brake": "仍保持制动请求",
                "drag": "仍保持关闭驱动请求",
            }[self.hardware_mode]
            self._notify(
                "目标超出历史3D模型关节范围；"
                f"新目标未发布，{unchanged}。",
                "warning",
            )
            self._update_mode_label(f"无效移动目标已拒绝；{unchanged}")
            return False
        segment = self.active_trajectory_segment
        segment_index = self.active_trajectory_segment_index
        token_id = self.workflow_contract.submitted_token_id
        manifest = self.active_plan_manifest
        manifest_segments = (
            manifest.get("segment_sha256")
            if isinstance(manifest, dict) else None
        )
        if (
            not isinstance(segment, TrajectoryPlan)
            or type(segment_index) is not int
            or not isinstance(token_id, str)
            or not isinstance(manifest, dict)
            or manifest.get("schema") != "go-m8010-plan-manifest/1.0"
            or not isinstance(manifest.get("recipe_sha256"), str)
            or not isinstance(manifest_segments, list)
            or len(manifest_segments) != self.trajectory_segment_count
            or not 0 <= segment_index < len(manifest_segments)
            or manifest_segments[segment_index] != segment.sha256
            or segment.start_rad != tuple(self.command_targets)
            or segment.target_rad != tuple(self.targets)
        ):
            self._notify(
                "碰撞证明未绑定到已预演的不可变quintic分段；现实下发已拒绝。",
                "critical",
            )
            return False
        try:
            self.active_trajectory_descriptor = trajectory_command_descriptor(
                segment,
                plan_token_id=token_id,
                execute_at_monotonic_ns=(
                    time.monotonic_ns() + TRAJECTORY_EXECUTE_LEAD_NS
                ),
                segment_index=segment_index,
                segment_count=self.trajectory_segment_count,
            )
        except ContractViolation as exc:
            self._notify(f"quintic命令描述符无效：{exc}", "critical")
            return False
        self.machine.position()
        self.hardware_mode = "position"
        self.command_targets = list(self.targets)
        self.moving_joint_mask = candidate_moving_joint_mask
        self.active_collision_proof = json.loads(json.dumps(approved_result))
        self.active_trajectory_first_publish_pending = True
        # Unedited load-bearing joints remain active at their captured targets;
        # otherwise the domain router would normalize them to BRAKE.
        self._authorize_active_joints([True] * 6)
        self._resume_command_stream()
        self.pending_target_joint_mask = [False] * 6
        self._refresh_virtual_editability()
        self.arrival.start(time.monotonic())
        self._update_mode_label(
            f"已授权 J{pending_indices[0] + 1} 执行当前安全分段；"
            "其余五轴继续固定目标HOLD"
        )
        return True

    def _stop(self) -> None:
        # Stop always aborts any not-yet-authorized virtual segments.  It does
        # not itself withdraw the current servo authority.
        self._cancel_queued_pose(restore_command_target=True)
        now = time.monotonic()
        streams_fresh = self.node.control_streams_fresh(now)
        self._update_connected(now)
        if self.active_observation_uncertain:
            self._suspend_command_stream(
                "停止时活动关节观测未知；保留底层租约安全保持"
            )
            self._notify(
                "活动关节遥测陈旧，GUI未发送制动；请继续机械支撑。",
                "critical",
            )
            return
        if control_feedback_ready(
            self.have_first_state, streams_fresh, self.connected
        ):
            if self.hardware_mode == "position":
                self._transition_position_to_fixed_hold()
                stop_text = "轨迹已停止；移动轴停于当前角度，其余轴保留原锁定目标"
            elif self.hardware_mode == "hold":
                # A stop request while already holding is not permission to
                # adopt an externally displaced feedback angle as the target.
                self.pending_target_joint_mask = [False] * 6
                self.moving_joint_mask = [False] * 6
                self.targets = list(self.command_targets)
                self._set_virtual_editable(False)
                self._show_targets_on_virtual()
                self._resume_command_stream()
                stop_text = "已保留原锁定HOLD目标；未重采样外力偏移后的角度"
            else:
                self._prepare_hold_at_actual()
                stop_text = "已请求保持当前姿态"
            self._update_mode_label(stop_text + "，等待硬件确认")
            self._notify(
                stop_text + "；请以命令路由和控制器确认状态为准。",
                "info",
            )
            self._publish_command()
            return
        if self.hardware_mode in {"hold", "position"} or self.command_stream_suspended:
            self._suspend_command_stream(
                "停止时反馈不可用；停止发送新命令并保留底层租约安全保持"
            )
            self._update_mode_label(
                "反馈不可用；未发送制动，等待底层安全保持并要求机械支撑"
            )
            self._notify(
                "反馈不可用，GUI已停止发送新命令；健康控制器应进入租约安全保持，"
                "但该状态尚未确认，请持续机械支撑。",
                "critical",
            )
            return
        self.machine.stop()
        self.hardware_mode = "brake"
        self.direction = ArmMode.REAL_TO_SIM
        self.targets = list(self.actual)
        self.command_targets = list(self.actual)
        self.candidate_targets = list(self.actual)
        self.requested_active_joint_mask = [False] * 6
        self.pending_target_joint_mask = [False] * 6
        self.moving_joint_mask = [False] * 6
        self._resume_command_stream()
        self._set_virtual_editable(False)
        self._show_targets_on_virtual()
        self._update_mode_label("反馈不可用；已请求制动，未确认承重保持")
        self._notify(
            "无可用的新鲜健康反馈，只能请求制动；制动不等于承重保持，请持续机械支撑。",
            "critical",
        )
        self._publish_command()

    def _save_initial_pose(self) -> None:
        if self.node.initial_pose_read_only:
            self._notify("生产初始化姿态受只读保护，GUI拒绝覆盖。", "warning")
            return
        if not self._require_control_feedback(
            "设置完整六轴初始化姿态要求六个关节均具有新鲜状态且健康连接。",
            require_all=True,
        ):
            return
        if not self.session_id:
            self._notify("尚未建立有效会话参考。")
            return
        document = {
            "有效": True,
            "会话标识": self.session_id,
            "时间戳": datetime.now(timezone.utc).isoformat(),
            "语义": "用户初始化姿态；仅为当前会话相对角度",
            "电机内部零位": "未修改",
            "CAD零位": "PENDING",
            "ROS零位": "PENDING",
            "关节位置_弧度": {f"J{i + 1}": self.actual[i] for i in range(6)},
        }
        self.node.initial_pose_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = self.node.initial_pose_path.with_suffix(
            self.node.initial_pose_path.suffix + ".tmp"
        )
        temporary_path.write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        temporary_path.replace(self.node.initial_pose_path)
        self._notify("初始化姿态已保存。", "info")

    def _return_initial_pose(self) -> None:
        if self.queued_pose_target is not None or self.hardware_mode == "position":
            self._notify(
                "已有位置分段正在执行；请先停止轨迹并确认六轴HOLD。",
                "warning",
            )
            return
        if not self._require_control_feedback(
            "返回完整六轴初始化姿态要求六个关节均具有新鲜状态且健康连接。",
            require_all=True,
        ):
            return
        try:
            raw_bytes = self.node.initial_pose_path.read_bytes()
            document = json.loads(raw_bytes)
            if not initial_pose_binding_valid(
                document, raw_bytes, self.node.latest_hardware
            ):
                raise ValueError
            positions = document["关节位置_弧度"]
            restored_target = [float(positions[f"J{i + 1}"]) for i in range(6)]
            if not all(
                math.isfinite(value) and self.limits[index][0] * RAD <= value <= self.limits[index][1] * RAD
                for index, value in enumerate(restored_target)
            ):
                raise ValueError
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            self._notify("初始化姿态需要重新设置。")
            return
        self.direction = ArmMode.SIM_TO_REAL
        self.candidate_targets = list(restored_target)
        self.targets = list(restored_target)
        self._clear_candidate_approval()
        try:
            self.workflow_contract = self.workflow_contract.change_plan_target(
                self.candidate_targets
            )
        except ContractViolation:
            self._notify("初始化姿态不满足当前计划限位。", "warning")
            return
        self.pending_target_joint_mask = [
            connected and abs(target - command) > TARGET_SELECTION_DEADBAND_RAD
            for target, command, connected in zip(
                self.targets, self.command_targets, self.connected
            )
        ]
        self._set_virtual_editable(True)
        self._show_targets_on_virtual()
        self._start_virtual_preview()

    def _notify(self, text: str, level: str = "warning") -> None:
        """Record one non-modal operator notice; repeated states replace text."""
        if level not in {"info", "warning", "critical"}:
            raise ValueError("界面提示级别无效")
        self.operator_notice_text = text
        self.operator_notice_level = level
        self.operator_notice_until = time.monotonic() + OPERATOR_NOTICE_DURATION_S
        self._refresh_safety_notice()

    def _require_control_feedback(self, message: str, require_all: bool = False) -> bool:
        now = time.monotonic()
        streams_fresh = self.node.control_streams_fresh(now)
        self._update_connected(now)
        if self.active_observation_uncertain:
            self._notify(
                "既有活动关节的遥测已变为未知；GUI保持暂停发送，"
                "等待底层租约安全保持，请持续机械支撑。",
                "critical",
            )
            return False
        if control_feedback_ready(
            self.have_first_state, streams_fresh, self.connected, require_all
        ):
            return True
        if not streams_fresh:
            self._suspend_for_stale_feedback()
        self._notify(message, "critical" if not streams_fresh else "warning")
        return False

    def _resume_command_stream(self) -> None:
        self.command_stream_suspended = False
        self.command_stream_suspended_reason = ""

    def _suspend_command_stream(self, reason: str) -> None:
        """Stop GUI publication without replacing a healthy hold by BRAKE."""
        self.command_stream_suspended = True
        self.command_stream_suspended_reason = reason
        if self.queued_pose_target is not None:
            self._cancel_queued_pose(restore_command_target=True)

    def _suspend_for_stale_feedback(self) -> None:
        if self.hardware_mode in {"drag", "hold", "position"}:
            self._suspend_command_stream(
                "关节或硬件状态已过期；GUI停止发送，等待底层租约安全策略"
            )
            self._update_mode_label(
                "状态过期；GUI发送已暂停，未用制动覆盖底层保持"
            )

    def _show_targets_on_virtual(self) -> None:
        planned_targets = getattr(self, "candidate_targets", self.targets)
        for index, target in enumerate(planned_targets):
            degrees = target * DEG
            widgets = self.virtual_widgets[index]
            widgets.target.blockSignals(True)
            if widgets.slider is not None:
                widgets.slider.blockSignals(True)
                widgets.slider.setValue(round(degrees * 100.0))
            widgets.target.setValue(degrees)
            if widgets.slider is not None:
                widgets.slider.blockSignals(False)
            widgets.target.blockSignals(False)
            widgets.value.setText(f"{degrees:+.2f}°")

    def _update_mode_label(self, extra: str = "") -> None:
        direction_text = "现实驱动虚拟" if self.direction is ArmMode.REAL_TO_SIM else "虚拟驱动现实"
        drive_text = {
            "brake": "制动", "drag": "关闭驱动力／拖动",
            "hold": "保持", "position": "位置控制",
        }[self.hardware_mode]
        fixed_hold_text = (
            "开" if self.machine.fixed_hold_after_arrival
            else "关（位置伺服仍有驱动力）"
        )
        text = (
            f"当前方向：{direction_text}　｜　控制请求：{drive_text}"
            f"　｜　到位后固定保持：{fixed_hold_text}"
        )
        if extra:
            text += f"　｜　{extra}"
        set_widget_text_if_changed(self.mode_label, text)

    def _sync_workflow_contract(self, now: float) -> None:
        if not self.have_first_state or not self.session_id or not self.state_instance_id:
            return
        token_before = self.workflow_contract.current_plan_token
        try:
            if (
                self.workflow_contract.session_id != self.session_id
                or self.workflow_contract.state_instance_id != self.state_instance_id
            ):
                self.workflow_contract = self.workflow_contract.replace_authority(
                    joint_limits_rad=self.workflow_contract.joint_limits_rad,
                    model_sha256=PRODUCTION_MODEL_SHA256,
                    session_id=self.session_id,
                    state_instance_id=self.state_instance_id,
                    gravity_config_sha256=GRAVITY_CONFIG_SHA256,
                )
            self.workflow_contract = self.workflow_contract.update_actual(
                self.actual,
                invalidation_tolerance_rad=PLAN_ACTUAL_DRIFT_TOLERANCE_RAD,
            )
            if tuple(self.candidate_targets) != self.workflow_contract.q_plan_target:
                self.workflow_contract = self.workflow_contract.change_plan_target(
                    self.candidate_targets
                )
        except ContractViolation as exc:
            self._clear_candidate_approval()
            self._set_workflow_state("unsafe", f"工作流状态无效：{exc}", 0)
            return
        token_after = self.workflow_contract.current_plan_token
        if token_before is not None and token_after is None:
            self._clear_candidate_approval()
            self._set_workflow_state(
                "stale", "实际姿态/session已变化，旧PLAN_TOKEN自动失效", 0
            )
            return
        if token_after is not None and not self._current_preview_checks(now).complete_success:
            self._clear_candidate_approval()
            self._set_workflow_state(
                "blocked", "温度、通信、重力或碰撞状态已变化，PLAN_TOKEN失效", 0
            )

    def _refresh_task_status_panel(self, now: float) -> None:
        labels = getattr(self, "task_status_labels", {})
        if not labels:
            return
        state_map = {
            "idle": "空闲", "pending": "正在检查限位",
            "planning": "正在解算轨迹",
            "previewing": "正在计算重力", "solving": "正在检查碰撞",
            "waiting_hardware": "读取现实状态", "safe": "等待用户下发",
            "execute_proof": "正在下发", "blocked": "安全门禁阻止",
            "unsafe": "已取消", "stale": "状态已过期",
            "timeout": "阶段超时", "complete": "已完成",
        }
        workflow_state = self.collision_preview_state
        terminal_states = {"complete", "timeout", "blocked", "unsafe", "stale"}
        execution_started = getattr(self, "task_execution_started_at", None)
        completed_at = getattr(self, "task_completed_at", None)
        state_text = state_map.get(workflow_state, "空闲")
        if workflow_state not in terminal_states and execution_started is not None:
            if self.hardware_mode == "position":
                state_text = "现实机械臂执行中"
            elif self.pending_collision_execute_sequence is not None:
                state_text = "正在复核现实分段"
            elif self.queued_pose_target is not None:
                state_text = "正在到位稳定"

        segment_count = max(0, int(getattr(self, "task_segment_count", 0)))
        completed_segments = min(
            segment_count,
            max(0, int(getattr(self, "task_completed_segments", 0))),
        )
        active_fraction: Optional[float] = None
        active_index = self.active_trajectory_segment_index
        moving_indices = [
            index for index, moving in enumerate(self.moving_joint_mask) if moving
        ]
        if (
            execution_started is not None
            and type(active_index) is int
            and len(moving_indices) == 1
            and isinstance(self.active_trajectory_descriptor, dict)
        ):
            active_fraction = moving_trajectory_feedback_fraction(
                self.node.latest_hardware,
                moving_indices[0],
                self.active_trajectory_descriptor,
            )
        if execution_started is None:
            progress = (
                self.workflow_progress.value()
                if self.workflow_progress.maximum() > 0 else 0
            )
        elif workflow_state == "complete":
            progress = 100
        elif segment_count > 0:
            progress_units = float(completed_segments)
            if type(active_index) is int and active_fraction is not None:
                progress_units = max(
                    progress_units, float(active_index) + active_fraction
                )
            progress = round(
                100.0 * min(segment_count, progress_units) / segment_count
            )
        else:
            progress = 0
        elapsed = (
            0.0 if self.task_started_at is None
            else max(
                0.0,
                (completed_at if completed_at is not None else now)
                - self.task_started_at,
            )
        )
        trajectory = self.workflow_contract.q_plan_trajectory
        if execution_started is not None:
            if workflow_state == "complete":
                remaining_text = "0.0s"
            elif workflow_state in terminal_states:
                remaining_text = "已终止"
            elif isinstance(trajectory, TrajectoryRecipe) and segment_count > 0:
                remaining_s = sum(
                    segment.profile.duration_s
                    for segment in trajectory.segments[completed_segments:]
                )
                if (
                    type(active_index) is int
                    and active_fraction is not None
                    and completed_segments <= active_index < len(trajectory.segments)
                ):
                    remaining_s -= (
                        trajectory.segments[active_index].profile.duration_s
                        * active_fraction
                    )
                remaining_text = f"{max(0.0, remaining_s):.1f}s"
            else:
                remaining_text = "待反馈"
        elif workflow_state == "planning":
            remaining_text = "后台规划中"
        elif trajectory is None:
            remaining_text = "—"
        elif workflow_state in terminal_states:
            remaining_text = "已终止"
        elif self.preview_animation_complete:
            remaining_text = (
                "等待下发" if workflow_state == "safe" else "0.0s"
            )
        else:
            preview_elapsed = max(
                0.0, now - (self.preview_animation_started_at or now)
            )
            remaining_text = (
                f"{max(0.0, trajectory.profile.duration_s - preview_elapsed):.1f}s"
            )
        hardware = self.node.latest_hardware or {}
        per_motor = hardware.get("per_motor", {})
        temperatures = [
            float(item["temperature_c"])
            for item in per_motor.values()
            if isinstance(item, dict)
            and type(item.get("temperature_c")) in {int, float}
            and math.isfinite(float(item["temperature_c"]))
        ] if isinstance(per_motor, dict) else []
        thermal_priority = (
            "THERMAL_STOP", "WAIT_OPERATOR_CONFIRM", "COOLDOWN",
            "DERATING", "WARNING", "NORMAL", "OFFLINE",
        )
        observed_states = {
            item.get("thermal_state", "OFFLINE")
            for item in per_motor.values()
            if isinstance(item, dict)
        } if isinstance(per_motor, dict) else {"OFFLINE"}
        worst_thermal = next(
            (item for item in thermal_priority if item in observed_states),
            "OFFLINE",
        )
        controller_modes = (
            hardware.get("controller_mode_by_motor", {})
            if isinstance(hardware, dict) else {}
        )
        brake_observed = (
            hardware.get("brake_observed_by_motor", {})
            if isinstance(hardware, dict) else {}
        )
        load_limit_status = (
            hardware.get("load_limit_no_progress_by_motor", {})
            if isinstance(hardware, dict) else {}
        )
        no_progress_status = (
            hardware.get("no_progress_status_by_motor", {})
            if isinstance(hardware, dict) else {}
        )
        communication_fault = bool(
            self.task_id is not None
            and (
                not isinstance(per_motor, dict)
                or any(
                    not isinstance(per_motor.get(name), dict)
                    or per_motor[name].get("fresh") is not True
                    or per_motor[name].get("communication_ok") is not True
                    or per_motor[name].get("merror") != 0
                    for name in MOTOR_NAMES
                )
            )
        )
        load_limit_fault = bool(
            isinstance(load_limit_status, dict)
            and any(load_limit_status.get(name) is True for name in MOTOR_NAMES)
        )
        trip_reasons = {
            record.get("trip_reason")
            for record in no_progress_status.values()
            if isinstance(record, dict) and record.get("fault_latched") is True
        } if isinstance(no_progress_status, dict) else set()
        arrival_timeout_fault = "POSITION_ARRIVAL_TIMEOUT" in trip_reasons
        overload_fault = bool(
            trip_reasons.intersection({
                "LOAD_LIMIT_NO_PROGRESS",
                "EXACT_TRAJECTORY_LOAD_GOVERNOR_ABORT",
            })
            or (load_limit_fault and not trip_reasons)
        )
        all_braked = bool(
            self.hardware_mode == "brake"
            and isinstance(controller_modes, dict)
            and isinstance(brake_observed, dict)
            and all(
                controller_modes.get(name) == "brake"
                and brake_observed.get(name) is True
                for name in MOTOR_NAMES
            )
        )
        planned_proof = (
            (self.node.latest_gravity_status or {}).get(
                "planned_trajectory_feasibility"
            )
            if isinstance(self.node.latest_gravity_status, dict)
            else None
        )
        planned_result = (
            planned_proof.get("result")
            if isinstance(planned_proof, dict)
            and isinstance(trajectory, TrajectoryRecipe)
            and planned_proof.get("trajectory_sha256") == trajectory.sha256
            and planned_proof.get("session_id")
            == getattr(self, "session_id", None)
            and planned_proof.get("state_instance_id")
            == getattr(self, "state_instance_id", None)
            else None
        )
        valid_feedback_times_ns = [
            sample["last_valid_feedback_monotonic_ns"]
            for sample in per_motor.values()
            if isinstance(sample, dict)
            and type(sample.get("last_valid_feedback_monotonic_ns")) is int
            and sample["last_valid_feedback_monotonic_ns"] > 0
        ] if isinstance(per_motor, dict) else []
        all_motor_last_feedback_age_ms = (
            max(0.0, now - min(valid_feedback_times_ns) / 1.0e9) * 1000.0
            if len(valid_feedback_times_ns) == len(MOTOR_NAMES)
            else None
        )

        # Contract state taxonomy uses measured safety conditions ahead of
        # nominal workflow phases.  It never turns missing authority into a
        # load PASS or a fabricated overload measurement.
        if workflow_state == "timeout":
            state_text = "阶段超时"
        elif worst_thermal in {"THERMAL_STOP", "COOLDOWN", "WAIT_OPERATOR_CONFIRM"}:
            state_text = "热停机"
        elif overload_fault or planned_result == "FAIL":
            state_text = "负载超限"
        elif arrival_timeout_fault:
            state_text = "未到位"
        elif communication_fault:
            state_text = "通信故障"
        elif all_braked:
            state_text = "已停止并制动"
        elif worst_thermal == "DERATING" and self.task_id is not None:
            state_text = "热降额"
        elif self.pending_collision_execute_sequence is not None:
            state_text = "正在下发"
        elif self.hardware_mode == "position" and execution_started is not None:
            if getattr(self, "active_trajectory_first_publish_pending", False):
                state_text = "正在下发"
            elif active_fraction is not None and active_fraction >= 1.0:
                state_text = "正在到位稳定"
            else:
                state_text = "现实机械臂执行中"
        elif workflow_state == "safe":
            state_text = "预演通过"
        elif workflow_state == "previewing" and isinstance(planned_proof, dict):
            state_text = "正在检查热负载"
        target_text = " ".join(
            f"J{i + 1}{value * DEG:+.1f}°"
            for i, value in enumerate(self.candidate_targets)
        )
        actual_text = " ".join(
            f"J{i + 1}{value * DEG:+.1f}°"
            for i, value in enumerate(self.actual)
        )
        max_error = max(
            abs(target - actual)
            for target, actual in zip(self.candidate_targets, self.actual)
        ) * DEG
        phase = self.workflow_status.text().removeprefix("工作流：")
        failure = (
            self.operator_notice_text
            if self.collision_preview_state
            in {"blocked", "unsafe", "stale", "timeout"}
            else "—"
        )
        trajectory_sha256 = (
            trajectory.sha256
            if isinstance(trajectory, TrajectoryRecipe) else None
        )
        if self.preview_collision_safe:
            collision_result = "PASS"
        elif workflow_state in {"blocked", "unsafe", "timeout"}:
            collision_result = "FAIL"
        elif isinstance(trajectory, TrajectoryRecipe):
            collision_result = "CHECKING"
        else:
            collision_result = "NOT_EVALUATED"
        (
            planned_gravity_text,
            planned_motor_text,
            planned_checks_text,
        ) = planned_path_preview_display(
            self.node.latest_gravity_status,
            trajectory_sha256=trajectory_sha256,
            session_id=getattr(self, "session_id", None),
            state_instance_id=getattr(self, "state_instance_id", None),
            limits_pass=isinstance(trajectory, TrajectoryRecipe),
            collision_result=collision_result,
            now_monotonic_ns=time.monotonic_ns(),
        )
        heartbeat_now = completed_at if completed_at is not None else now
        values = {
            "task_id": self.task_id or "—",
            "state": state_text,
            "phase": phase,
            "progress": f"{progress}%",
            "started": self.task_started_wall_utc or "—",
            "elapsed": f"{elapsed:.1f}s" if self.task_started_at is not None else "—",
            "remaining": remaining_text,
            "heartbeat": (
                f"{max(0.0, heartbeat_now - self.last_task_heartbeat) * 1000.0:.0f}ms前"
            ),
            "encoder": (
                f"{all_motor_last_feedback_age_ms:.0f}ms前"
                if all_motor_last_feedback_age_ms is not None else "未收到"
            ),
            "target": target_text,
            "actual": actual_text,
            "max_error": f"{max_error:.2f}°",
            "max_temperature": (
                f"{max(temperatures):.0f}°C" if temperatures else "离线"
            ),
            "thermal": THERMAL_STATE_CN.get(worst_thermal, worst_thermal),
            "planned_gravity": planned_gravity_text,
            "planned_motor": planned_motor_text,
            "planned_checks": planned_checks_text,
            "failure": failure,
        }
        for key, text in values.items():
            set_widget_text_if_changed(labels[key], text)

    def _refresh_motor_status_panel(self, now: float) -> None:
        table = getattr(self, "motor_status_table", None)
        if table is None:
            return
        hardware = self.node.latest_hardware or {}
        per_motor = hardware.get("per_motor", {})
        modes = hardware.get("controller_mode_by_motor", {})
        faults = hardware.get("controller_fault_by_motor", {})
        lease_holds = hardware.get("lease_safe_hold_by_motor", {})
        warning_orange_c = getattr(
            getattr(self.node, "thermal_limits", None),
            "warning_below_c",
            None,
        )
        for row, name in enumerate(MOTOR_NAMES):
            sample = per_motor.get(name, {}) if isinstance(per_motor, dict) else {}
            fresh = sample.get("fresh") is True
            communication_ok = sample.get("communication_ok") is True
            online = fresh and communication_ok
            thermal_state = sample.get("thermal_state", "OFFLINE") if fresh else "OFFLINE"
            temperature = sample.get("temperature_c")

            def number(field: str, scale: float = 1.0, suffix: str = "") -> str:
                value = sample.get(field)
                if type(value) not in {int, float} or not math.isfinite(float(value)):
                    return "N/A"
                return f"{float(value) * scale:+.3f}{suffix}"

            control_state = (
                "控制故障" if faults.get(name) is True
                else "租约安全保持" if lease_holds.get(name) is True
                else "热锁存" if sample.get("thermal_fault_latched") is True
                else "正常" if online else "状态未知"
            )
            trajectory_state = sample.get("trajectory_state", "INACTIVE")
            if trajectory_state != "INACTIVE":
                sample_index = sample.get("trajectory_sample_index", 0)
                interval_count = sample.get("trajectory_interval_count", 0)
                control_state += (
                    f" / quintic {trajectory_state} "
                    f"{sample_index}/{interval_count}"
                )
            values = (
                MOTOR_LOGICAL_JOINT[name], name, MOTOR_BUS_LOCAL_ID[name],
                "在线" if online else "离线",
                (
                    f"{float(sample['age_ms']):.1f}"
                    if type(sample.get("age_ms")) in {int, float} else "N/A"
                ),
                str(modes.get(name, "unknown")),
                number("raw_position_rad"),
                number("q_joint_rad", DEG),
                number("dq_joint_rad_s", DEG),
                number("tau_feedback_rotor_nm"),
                number("tau_joint_estimated_nm"),
                (
                    f"{float(temperature):.0f}"
                    if type(temperature) in {int, float} else "N/A"
                ),
                str(sample.get("merror", "N/A")),
                THERMAL_STATE_CN.get(str(thermal_state), str(thermal_state)),
                control_state,
                (
                    f"{float(sample['last_valid_feedback_monotonic_ns']) / 1.0e9:.3f}"
                    if type(sample.get("last_valid_feedback_monotonic_ns")) is int
                    and sample["last_valid_feedback_monotonic_ns"] > 0
                    else "N/A"
                ),
            )
            for column, text in enumerate(values):
                item = table.item(row, column)
                if item.text() != text:
                    item.setText(text)

            temperature_item = table.item(row, 11)
            state_item = table.item(row, 13)
            if thermal_state == "OFFLINE":
                background, foreground = "#616161", "#ffffff"
            elif thermal_state == "NORMAL":
                background, foreground = "#2e7d32", "#ffffff"
            elif (
                thermal_state == "WARNING"
                and (
                    type(temperature) not in {int, float}
                    or not math.isfinite(float(temperature))
                    or type(warning_orange_c) not in {int, float}
                    or not math.isfinite(float(warning_orange_c))
                    or float(temperature) >= float(
                        warning_orange_c
                    )
                )
            ):
                background, foreground = "#ef6c00", "#ffffff"
            elif thermal_state == "WARNING":
                background, foreground = "#fdd835", "#000000"
            elif thermal_state == "DERATING":
                background, foreground = "#c62828", "#ffffff"
            elif thermal_state == "THERMAL_STOP":
                background, foreground = "#4a0000", "#ffeb3b"
                if int(now * 2.0) % 2:
                    state_item.setText("⚠ 热停机 ⚠")
            else:
                background, foreground = "#8d3b00", "#ffffff"
            for item in (temperature_item, state_item):
                item.setBackground(QColor(background))
                item.setForeground(QColor(foreground))

        j2a = per_motor.get("J2A", {}) if isinstance(per_motor, dict) else {}
        j2b = per_motor.get("J2B", {}) if isinstance(per_motor, dict) else {}
        if self.j2_motor_summary is not None:
            def optional_value(mapping: dict, field: str, scale: float = 1.0) -> str:
                value = mapping.get(field)
                return (
                    f"{float(value) * scale:+.2f}"
                    if type(value) in {int, float} and math.isfinite(float(value))
                    else "N/A"
                )
            temperature_difference = (
                abs(float(j2a["temperature_c"]) - float(j2b["temperature_c"]))
                if type(j2a.get("temperature_c")) in {int, float}
                and type(j2b.get("temperature_c")) in {int, float}
                else None
            )
            text = (
                f"J2逻辑实际={self.actual[1] * DEG:+.2f}°　"
                f"目标={self.candidate_targets[1] * DEG:+.2f}°　"
                f"误差={(self.candidate_targets[1] - self.actual[1]) * DEG:+.2f}°　"
                f"e_sync={float(hardware.get('j2_e_sync_rad', 0.0)) * DEG:+.2f}°　"
                f"rotor力矩分担 A={optional_value(j2a, 'tau_feedback_rotor_nm')} / "
                f"B={optional_value(j2b, 'tau_feedback_rotor_nm')} N·m　"
                f"温差={'N/A' if temperature_difference is None else f'{temperature_difference:.0f}°C'}"
            )
            set_widget_text_if_changed(self.j2_motor_summary, text)

    def _tick(self) -> None:
        if not run_ros_context_operation(
            self._ros_context_ok,
            lambda: pump_ros_callbacks(self.node, rclpy.spin_once),
        ):
            self._close_for_ros_shutdown()
            return
        now = time.monotonic()
        hardware = self.node.latest_hardware
        if hardware_state_contract_valid(hardware):
            self.actual = [float(value) for value in hardware["position_rad"]]
            if not self.have_first_state:
                self.targets = list(self.actual)
                self.command_targets = list(self.actual)
                self.candidate_targets = list(self.actual)
                self.have_first_state = True
        self._update_connected(now)
        self._sync_workflow_contract(now)
        self._update_preview_plan_build_heartbeat(now)
        self._update_preview_animation(now)
        self._update_planned_execution_pose(time.monotonic_ns())
        self._consume_collision_guard_result()
        self._expire_collision_request(now)
        if not self.node.control_streams_fresh(now):
            self._suspend_for_stale_feedback()
        if (
            self.hardware_mode in {"hold", "position"}
            and any(self.requested_active_joint_mask)
            and (
                not self.node.gravity_status_fresh(now)
                or not gravity_status_authorizes_hardware(
                    self.node.latest_gravity_status,
                    session_id=self.session_id,
                    state_instance_id=self.state_instance_id,
                    now_monotonic_ns=time.monotonic_ns(),
                )
            )
        ):
            self._suspend_command_stream(
                "重力authority失效或过期；停止刷新命令并保留底层租约安全保持"
            )
        self._refresh_joint_widgets()
        # Arrival first requests the exact-target HOLD barrier.  A later tick
        # observes all seven motors actually reporting HOLD and stationary;
        # only then may this advance to a freshly checked next segment.
        self._continue_queued_sequence_if_ready()
        if self.planned_mujoco_preview is not None:
            try:
                self.planned_mujoco_preview.set_relative_pose(
                    self.preview_pose
                    if (
                        self.preview_requested_by_operator
                        or self.active_trajectory_segment is not None
                        or self.queued_pose_target is not None
                    )
                    else self.candidate_targets
                )
            except Exception as exc:
                self.planned_mujoco_preview.image.setText(
                    f"计划MuJoCo渲染失败：{exc}"
                )
        if self.actual_mujoco_preview is not None:
            try:
                self.actual_mujoco_preview.set_relative_pose(self.actual)
            except Exception as exc:
                self.actual_mujoco_preview.image.setText(
                    f"现实数字孪生渲染失败：{exc}"
                )
        if not run_ros_context_operation(self._ros_context_ok, self._publish_command):
            self._close_for_ros_shutdown()
            return
        self._refresh_task_status_panel(now)
        self._refresh_motor_status_panel(now)
        self._refresh_summary()
        self._write_log()

    def _ros_context_ok(self) -> bool:
        return rclpy.ok(context=self.node.context)

    def _close_for_ros_shutdown(self) -> None:
        self.timer.stop()
        self.close()

    def _update_connected(self, now: Optional[float] = None) -> None:
        checked_at = time.monotonic() if now is None else now
        if not self.node.control_streams_fresh(checked_at):
            if self.queued_pose_target is not None:
                self._cancel_queued_pose(restore_command_target=True)
            self.active_observation_uncertain = bool(
                self.hardware_mode in {"hold", "position"}
                and any(self.requested_active_joint_mask)
            )
            if self.active_observation_uncertain:
                self._suspend_command_stream(
                    "活动关节状态流过期；不以BRAKE覆盖底层租约保持"
                )
            self.connected = [False] * 6
            self.faulted = [False] * 6
            self.observation_uncertain = [True] * 6
            self._refresh_virtual_editability()
            return
        hardware = self.node.latest_hardware
        if not hardware_state_contract_valid(hardware):
            if self.queued_pose_target is not None:
                self._cancel_queued_pose(restore_command_target=True)
            self.active_observation_uncertain = bool(
                self.hardware_mode in {"hold", "position"}
                and any(self.requested_active_joint_mask)
            )
            if self.active_observation_uncertain:
                self._suspend_command_stream(
                    "硬件状态合同无效；不发送空授权BRAKE"
                )
            self.connected = [False] * 6
            self.faulted = [False] * 6
            self.observation_uncertain = [True] * 6
            self._refresh_virtual_editability()
            return
        incoming_session = hardware.get("session_id")
        incoming_state_instance = hardware.get("state_instance_id")
        reference_changed = bool(
            incoming_session
            and self.session_id
            and incoming_session != self.session_id
        )
        state_publisher_changed = bool(
            incoming_state_instance
            and self.state_instance_id
            and incoming_state_instance != self.state_instance_id
        )
        if reference_changed or state_publisher_changed:
            change_reason = (
                "硬件参考会话已变化"
                if reference_changed
                else "硬件状态进程已重启"
            )
            self.targets = list(self.actual)
            self.command_targets = list(self.actual)
            self.candidate_targets = list(self.actual)
            self.requested_active_joint_mask = [False] * 6
            self.pending_target_joint_mask = [False] * 6
            self.moving_joint_mask = [False] * 6
            self.active_collision_proof = None
            self._cancel_queued_pose(restore_command_target=False)
            self._suspend_command_stream(
                f"{change_reason}；GUI授权已清除，等待操作员重新接管"
            )
            self._update_mode_label(
                f"{change_reason}；发送已暂停，未自动请求制动"
            )
            self._notify(
                f"检测到{change_reason}，已清除GUI授权并停止发送；底层状态未确认，"
                "请持续机械支撑并重新选择保持。",
                "critical",
            )
        if incoming_session:
            self.session_id = str(incoming_session)
        if incoming_state_instance:
            self.state_instance_id = str(incoming_state_instance)
        connected, faulted, uncertain = classify_logical_joint_observations(
            hardware
        )
        uncertain_active = [
            bool(
                uncertain[index]
                and self.hardware_mode in {"hold", "position"}
                and self.requested_active_joint_mask[index]
            )
            for index in range(6)
        ]
        preserved_requested = list(self.requested_active_joint_mask)
        preserved_pending = list(self.pending_target_joint_mask)
        preserved_moving = list(self.moving_joint_mask)
        self._apply_connection_state(connected, faulted)
        for index, preserve in enumerate(uncertain_active):
            if preserve:
                self.requested_active_joint_mask[index] = preserved_requested[index]
                self.pending_target_joint_mask[index] = preserved_pending[index]
                self.moving_joint_mask[index] = preserved_moving[index]
        self.observation_uncertain = uncertain
        self.active_observation_uncertain = any(uncertain_active)
        if self.active_observation_uncertain:
            self._suspend_command_stream(
                "活动关节遥测陈旧／未知；暂停GUI命令并保留底层租约保持"
            )
        self._refresh_virtual_editability()

    def _apply_connection_state(
        self, connected: list[bool], faulted: list[bool]
    ) -> None:
        sequence_invalidated = bool(
            self.queued_pose_target is not None
            and (
                any(faulted)
                or any(was_connected and not is_connected for was_connected, is_connected in zip(
                    self.connected, connected
                ))
            )
        )
        if sequence_invalidated:
            # Do not allow a recovered link to resume an old plan.  The current
            # worker authority/target is left intact; only future segments and
            # any pending proof are revoked.
            self._cancel_queued_pose(restore_command_target=True)
        self.requested_active_joint_mask = clear_mask_on_connection_loss(
            self.requested_active_joint_mask, self.connected, connected
        )
        self.pending_target_joint_mask = clear_mask_on_connection_loss(
            self.pending_target_joint_mask, self.connected, connected
        )
        self.moving_joint_mask = clear_mask_on_connection_loss(
            self.moving_joint_mask, self.connected, connected
        )
        # A confirmed hard fault revokes authority even if the preceding
        # observation was already unknown/disconnected, so it has no fresh
        # true->false edge for clear_mask_on_connection_loss to detect.
        for index, hard_fault in enumerate(faulted):
            if hard_fault:
                self.requested_active_joint_mask[index] = False
                self.pending_target_joint_mask[index] = False
                self.moving_joint_mask[index] = False
        self.connected = connected
        self.faulted = faulted

    def _authorize_active_joints(self, requested: list[bool]) -> None:
        if self.activation_epoch >= (1 << 63) - 1:
            raise RuntimeError("激活纪元已耗尽，必须安全重启完整控制会话")
        self.requested_active_joint_mask = [
            bool(requested[index] and self.connected[index])
            for index in range(6)
        ]
        self.activation_epoch = max(
            self.activation_epoch + 1,
            time.monotonic_ns() & ((1 << 63) - 1),
        )

    def _confirmed_joint_modes(self) -> list[Optional[str]]:
        by_motor = (self.node.latest_hardware or {}).get(
            "controller_mode_by_motor", {}
        )
        motor_groups = (
            ("J1",), ("J2A", "J2B"), ("J3",),
            ("J4",), ("J5",), ("J6",),
        )
        result: list[Optional[str]] = []
        for names in motor_groups:
            values = [by_motor.get(name) for name in names]
            result.append(
                values[0]
                if values and all(value == values[0] for value in values)
                else None
            )
        return result

    def _refresh_joint_widgets(self) -> None:
        now = time.monotonic()
        hardware = self.node.latest_hardware or {}
        errors = [target - actual for target, actual in zip(self.command_targets, self.actual)]
        active_joint_mask = effective_active_joint_mask(
            self.requested_active_joint_mask, self.connected, self.hardware_mode
        )
        confirmed_modes = self._confirmed_joint_modes()
        if self.hardware_mode in {"position", "hold"}:
            tracking_connected = [
                connected and active
                for connected, active in zip(self.connected, active_joint_mask)
            ]
            states = self.arrival.update(now, errors, tracking_connected)
            for index, connected in enumerate(self.connected):
                if not connected:
                    continue
                expected_mode = (
                    "hold"
                    if (
                        self.hardware_mode == "position"
                        and active_joint_mask[index]
                        and not self.moving_joint_mask[index]
                    )
                    else self.hardware_mode if active_joint_mask[index] else "brake"
                )
                if confirmed_modes[index] != expected_mode:
                    states[index] = (
                        "位置待确认" if expected_mode == "position"
                        else "保持待确认" if expected_mode == "hold"
                        else "制动待确认"
                    )
                elif not active_joint_mask[index]:
                    states[index] = "已制动"
            moving_indices = [
                index
                for index, moving in enumerate(self.moving_joint_mask)
                if moving and active_joint_mask[index]
            ]
            if (
                self.hardware_mode == "position"
                and moving_indices
                and isinstance(self.active_trajectory_descriptor, dict)
                and moving_trajectory_feedback_complete(
                    hardware,
                    moving_indices[0],
                    self.active_trajectory_descriptor,
                )
                and all(
                    states[index] in {"已到位", "保持中"}
                    and confirmed_modes[index] == "position"
                    for index in moving_indices
                )
            ):
                sequence_joint = self.active_sequence_joint
                completed_segment_index = self.active_trajectory_segment_index
                if type(completed_segment_index) is int:
                    self.task_completed_segments = max(
                        getattr(self, "task_completed_segments", 0),
                        completed_segment_index + 1,
                    )
                if (
                    sequence_joint is not None
                    or self.machine.fixed_hold_after_arrival
                ):
                    # A completed trajectory becomes fixed-target HOLD without
                    # sampling feedback again. Later external displacement must
                    # therefore generate restoring error toward the exact target.
                    self.machine.hold()
                    self.hardware_mode = "hold"
                    self.moving_joint_mask = [False] * 6
                    self.active_collision_proof = None
                    self.active_trajectory_descriptor = None
                    self.active_trajectory_first_publish_pending = False
                    self.targets = list(self.command_targets)
                    self.preview_pose = tuple(self.command_targets)
                    self.pending_target_joint_mask = [False] * 6
                    self._show_targets_on_virtual()
                    self.arrival.start(now)
                    if sequence_joint is not None:
                        # A physical sequence always inserts an exact-target
                        # HOLD barrier, even if the optional final policy is
                        # "continue POSITION".  Only confirmed all-axis HOLD
                        # may authorize the next separately proven segment.
                        self.active_sequence_joint = None
                        self.active_trajectory_segment = None
                        self.active_trajectory_segment_index = None
                        self.active_trajectory_descriptor = None
                        self._update_mode_label(
                            f"J{sequence_joint + 1}分段已到位；"
                            "已转为原目标HOLD且未重采样反馈，"
                            "等待六轴HOLD确认后继续"
                        )
                    else:
                        self._update_mode_label(
                            "目标已到位；已按策略转为固定目标HOLD，"
                            "未重采样反馈角度"
                        )
                else:
                    # Keep both the exact target and moving mask so every
                    # domain continues to receive POSITION, not implicit HOLD.
                    self._update_mode_label(
                        "目标已到位；固定保持策略为关，"
                        "继续发送POSITION目标，位置伺服仍会产生必要驱动力"
                    )
        elif self.hardware_mode == "brake":
            states = [
                ("已制动" if confirmed_modes[index] == "brake" else "制动待确认")
                if item else ("故障" if self.faulted[index] else "未连接")
                for index, item in enumerate(self.connected)
            ]
        elif self.hardware_mode == "drag":
            states = []
            for index, connected in enumerate(self.connected):
                if not connected:
                    states.append("故障" if self.faulted[index] else "未连接")
                elif active_joint_mask[index]:
                    states.append(
                        "可拖动" if confirmed_modes[index] == "drag" else "拖动待确认"
                    )
                else:
                    states.append(
                        "已制动" if confirmed_modes[index] == "brake" else "制动待确认"
                    )
        else:
            states = ["待机"] * 6
        for index, faulted in enumerate(self.faulted):
            if faulted:
                states[index] = "故障"
            elif self.observation_uncertain[index]:
                states[index] = "状态未知"
        if (
            self.node.control_streams_fresh(now)
            and not self.observation_uncertain[1]
            and hardware.get("j2_sync_fault", False)
        ):
            states[1] = "同步异常"
        for index in range(6):
            actual_deg = self.actual[index] * DEG
            target_deg = self.command_targets[index] * DEG
            error_deg = errors[index] * DEG
            real = self.real_widgets[index]
            if real.slider is not None:
                set_widget_value_if_changed(real.slider, round(actual_deg * 100.0))
            set_widget_text_if_changed(real.actual, f"{actual_deg:+.2f}°")
            set_widget_text_if_changed(real.target, f"{target_deg:+.2f}°")
            set_widget_text_if_changed(real.error, f"{error_deg:+.2f}°")
            set_widget_text_if_changed(real.state, states[index])
        if (
            self.direction is ArmMode.REAL_TO_SIM
            and self.hardware_mode in {"brake", "drag"}
        ):
            self.targets = list(self.actual)
            self.command_targets = list(self.actual)
            self.candidate_targets = list(self.actual)
            self._show_targets_on_virtual()

    def _publish_command(self) -> bool:
        active_joint_mask = effective_active_joint_mask(
            self.requested_active_joint_mask, self.connected, self.hardware_mode
        )
        # GUI status and the virtual target are observability channels, not
        # hardware authority.  Keep them alive while the command stream is
        # suspended so a fault does not make the interface look frozen and the
        # MuJoCo preview can still explain exactly what is selected.
        self.node.mode_publisher.publish(String(data=json.dumps({
            "direction": self.direction.value,
            "hardware_mode": self.hardware_mode,
            "active_joint_mask": active_joint_mask,
            "moving_joint_mask": self.moving_joint_mask,
            "pending_target_joint_mask": self.pending_target_joint_mask,
            "connected_joint_mask": self.connected,
            "faulted_joint_mask": self.faulted,
            "activation_epoch": self.activation_epoch,
            "command_stream_suspended": self.command_stream_suspended,
            "command_stream_suspended_reason": self.command_stream_suspended_reason,
        }, ensure_ascii=False)))
        if self.command_stream_suspended:
            self.node.target_publisher.publish(
                Float64MultiArray(data=self.targets)
            )
            return False
        if (
            self.hardware_mode == "position"
            and self.active_trajectory_first_publish_pending
        ):
            segment = self.active_trajectory_segment
            segment_index = self.active_trajectory_segment_index
            token_id = self.workflow_contract.submitted_token_id
            if (
                not isinstance(segment, TrajectoryPlan)
                or type(segment_index) is not int
                or not isinstance(token_id, str)
                or not isinstance(self.active_plan_manifest, dict)
            ):
                self._suspend_command_stream(
                    "首次轨迹发送缺少不可变PLAN_TOKEN或分段manifest"
                )
                return False
            try:
                # Allocate the cross-domain lead immediately before the first
                # ROS publish; Qt rendering/collision callbacks cannot consume
                # the worker preparation window before it is placed on wire.
                self.active_trajectory_descriptor = trajectory_command_descriptor(
                    segment,
                    plan_token_id=token_id,
                    execute_at_monotonic_ns=(
                        time.monotonic_ns() + TRAJECTORY_EXECUTE_LEAD_NS
                    ),
                    segment_index=segment_index,
                    segment_count=self.trajectory_segment_count,
                )
            except ContractViolation as exc:
                self._suspend_command_stream(f"轨迹首次发送已拒绝：{exc}")
                return False
        self.command_sequence += 1
        self.node.publish_command(
            self.command_sequence, self.hardware_mode,
            self.command_targets, self.targets, active_joint_mask,
            self.moving_joint_mask,
            self.activation_epoch, self.config,
            self.active_collision_proof,
            self.active_trajectory_descriptor,
            self.active_plan_manifest,
        )
        if self.hardware_mode == "position":
            self.active_trajectory_first_publish_pending = False
        if self.task_id is not None:
            self.last_task_heartbeat = time.monotonic()
        return True

    def _refresh_safety_notice(self, now: Optional[float] = None) -> None:
        checked_at = time.monotonic() if now is None else now
        streams_fresh = self.node.control_streams_fresh(checked_at)
        lease_ms = float(self.config["控制"]["命令租约_秒"]) * 1000.0
        router_text, router_level = command_router_status_text(
            self.node.latest_control_status,
            self.node.control_status_fresh(checked_at),
            lease_ms,
            self.hardware_mode,
        )
        notices: list[tuple[str, str]] = []
        if self.command_stream_suspended:
            notices.append((
                "GUI命令发送已暂停：" + self.command_stream_suspended_reason,
                "critical",
            ))
        if not streams_fresh:
            notices.append((
                "关节或硬件状态流未建立／已过期；主动承重保持未确认，请机械支撑",
                "critical",
            ))
        if router_level != "normal":
            notices.append((f"命令路由：{router_text}", router_level))

        hardware = self.node.latest_hardware or {}
        j2_observed = bool(
            streams_fresh
            and all(
                hardware.get("per_motor", {}).get(name, {}).get("fresh", False)
                and hardware.get("per_motor", {}).get(name, {}).get(
                    "reference_captured", False
                )
                for name in ("J2A", "J2B")
            )
        )
        lease_safe_hold_motors = (
            sorted(
                name for name, active in hardware.get(
                    "lease_safe_hold_by_motor", {}
                ).items() if active
            )
            if streams_fresh else []
        )
        if lease_safe_hold_motors:
            notices.append((
                "底层租约安全保持已确认：" + ",".join(lease_safe_hold_motors),
                "warning",
            ))
        try:
            sync_error = abs(float(hardware.get("j2_e_sync_rad", 0.0)))
        except (TypeError, ValueError):
            sync_error = math.inf
        if j2_observed and hardware.get("j2_sync_fault", False):
            notices.append((
                "J2同步硬联锁已锁存；J2A/J2B均应处于BRAKE，"
                "须完成显式恢复流程后才能重新授权",
                "critical",
            ))
        elif j2_observed and sync_error > math.radians(0.5):
            notices.append((
                "J2同步偏差已超过0.5°，但硬联锁状态尚未回报；"
                "遥测不一致，禁止继续主动授权。",
                "critical",
            ))
        elif j2_observed and sync_error > math.radians(0.25):
            notices.append((
                "J2同步偏差已超过0.25°警告阈值；"
                "超过0.5°的单个有效配对帧会锁存J2A/J2B BOTH BRAKE。",
                "warning",
            ))
        if any(self.faulted):
            notices.append(("存在关节故障；故障关节主动授权已撤销", "critical"))
        if any(self.observation_uncertain):
            notices.append((
                "存在遥测陈旧／未知关节；不将未知状态解释为制动授权",
                "critical" if self.active_observation_uncertain else "warning",
            ))
        elif self.have_first_state and any(self.connected) and not all(self.connected):
            notices.append(("仅部分关节具有健康反馈；其余关节未获主动授权", "warning"))

        if checked_at <= self.operator_notice_until and self.operator_notice_text:
            notices.append((self.operator_notice_text, self.operator_notice_level))
        elif checked_at > self.operator_notice_until:
            self.operator_notice_text = ""

        level_rank = {"normal": 0, "info": 0, "warning": 1, "critical": 2}
        if notices:
            level = max((item[1] for item in notices), key=level_rank.__getitem__)
            text = "安全状态：" + "；".join(item[0] for item in notices)
        else:
            level = "normal"
            text = "安全状态：界面未发现当前告警；动作仍须以路由接收和控制器确认为准"
        styles = {
            "normal": "padding: 7px; background: #1b5e20; color: white; font-weight: bold;",
            "info": "padding: 7px; background: #0d47a1; color: white; font-weight: bold;",
            "warning": "padding: 7px; background: #e65100; color: white; font-weight: bold;",
            "critical": "padding: 7px; background: #b71c1c; color: white; font-weight: bold;",
        }
        if self.safety_notice.text() != text:
            self.safety_notice.setText(text)
            self.statusBar().showMessage(text)
        if self.current_notice_level != level:
            self.safety_notice.setStyleSheet(styles[level])
            self.current_notice_level = level

    def _refresh_summary(self) -> None:
        now = time.monotonic()
        # Safety text remains immediate.  Only the verbose telemetry summary is
        # rate-limited, since it contains continuously changing ages/rates that
        # otherwise force a word-wrapped layout at the full 25 Hz GUI rate.
        self._refresh_safety_notice(now)
        if now - self.last_summary_refresh_at < SUMMARY_REFRESH_PERIOD_S:
            return
        self.last_summary_refresh_at = now
        ros_ok = self.node.control_streams_fresh(now)
        hardware_text = "全部健康" if all(self.connected) else ("部分健康" if any(self.connected) else "无健康关节")
        mujoco = self.node.latest_mujoco
        mujoco_text = mujoco_status_text(mujoco, self.node.mujoco_stream_fresh(now))
        router_text, _router_level = command_router_status_text(
            self.node.latest_control_status,
            self.node.control_status_fresh(now),
            float(self.config["控制"]["命令租约_秒"]) * 1000.0,
            self.hardware_mode,
        )
        sync_text = "等待"
        j2_observed = bool(
            ros_ok
            and self.node.latest_hardware
            and all(
                self.node.latest_hardware.get("per_motor", {}).get(
                    name, {}
                ).get("fresh", False)
                and self.node.latest_hardware.get("per_motor", {}).get(
                    name, {}
                ).get("reference_captured", False)
                for name in ("J2A", "J2B")
            )
        )
        if j2_observed:
            sync_deg = float(self.node.latest_hardware.get("j2_e_sync_rad", 0.0)) * DEG
            sync_text = f"{sync_deg:+.3f}°"
        actual_modes = set(
            (self.node.latest_hardware or {}).get("controller_mode_by_motor", {}).values()
        ) if ros_ok else set()
        mode_text = {
            "brake": "已制动", "drag": "可拖动", "hold": "保持中",
            "position": "位置控制", "unknown": "未知",
        }
        confirmed = "、".join(sorted(mode_text.get(item, "未知") for item in actual_modes)) or "等待确认"
        lease_safe_hold_motors = (
            sorted(
                name for name, active in (self.node.latest_hardware or {}).get(
                    "lease_safe_hold_by_motor", {}
                ).items() if active
            )
            if ros_ok else []
        )
        lease_hold_text = (
            "等待确认" if not ros_ok else
            (",".join(lease_safe_hold_motors) if lease_safe_hold_motors else "无")
        )
        requested = {
            "brake": "制动", "drag": "关闭驱动力／拖动",
            "hold": "保持", "position": "位置控制",
        }[self.hardware_mode]
        fixed_hold_policy_text = (
            "开" if self.machine.fixed_hold_after_arrival
            else "关（位置伺服仍有驱动力）"
        )
        if self.command_stream_suspended:
            requested += "（GUI发送暂停）"
        measured_rate = (
            (self.node.latest_hardware or {}).get("timing", {}).get("median_publish_rate_hz")
            if ros_ok else None
        )
        rate_text = "等待" if measured_rate is None else f"{float(measured_rate):.1f}赫兹"
        joint_states = [widgets.state.text() for widgets in self.real_widgets]
        if any(state in {"故障", "同步异常"} for state in joint_states):
            overall = "存在关节故障"
        elif any(state.endswith("待确认") for state in joint_states):
            overall = "控制请求等待硬件确认"
        elif any(state == "未到位" for state in joint_states):
            overall = "部分关节未到位"
        elif joint_states and all(state in {"已到位", "保持中"} for state in joint_states):
            overall = "全部关节已到位"
        else:
            overall = "状态监视中"
        summary_text = (
            f"健康反馈：{hardware_text}　｜　控制状态流：{'新鲜' if ros_ok else '过期／等待'}　｜　"
            f"控制请求：{requested}　｜　到位后固定保持：{fixed_hold_policy_text}　｜　"
            f"控制器确认：{confirmed}　｜　命令路由：{router_text}　｜　"
            f"租约安全保持：{lease_hold_text}　｜　"
            f"MuJoCo：{mujoco_text}　｜　实测更新频率：{rate_text}　｜　"
            f"J2双电机同步误差：{sync_text}　｜　整体：{overall}"
        )
        set_widget_text_if_changed(self.summary, summary_text)

    def _open_log(self) -> None:
        self.node.log_directory.mkdir(parents=True, exist_ok=True)
        token = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_stream = (self.node.log_directory / f"arm_gui_{token}.csv").open("w", encoding="utf-8", newline="")
        fields = ["timestamp", "mode"]
        for index in range(1, 7):
            fields.extend((f"J{index}_target_rad", f"J{index}_actual_rad"))
        fields.extend(("J2_sync_error_rad", "fault_state"))
        self.log_writer = csv.DictWriter(self.log_stream, fieldnames=fields)
        self.log_writer.writeheader()

    def _write_log(self) -> None:
        now = time.monotonic()
        frequency = float(self.config["界面"]["日志频率_赫兹"])
        if now - self.last_log_at < 1.0 / frequency:
            return
        self.last_log_at = now
        row = {"timestamp": datetime.now(timezone.utc).isoformat(), "mode": self.hardware_mode}
        for index in range(6):
            row[f"J{index + 1}_target_rad"] = self.command_targets[index]
            row[f"J{index + 1}_actual_rad"] = self.actual[index]
        hardware = self.node.latest_hardware or {}
        row["J2_sync_error_rad"] = hardware.get("j2_e_sync_rad", "")
        row["fault_state"] = "" if all(self.connected) else "部分关节未连接"
        self.log_writer.writerow(row)
        self.log_stream.flush()

    def closeEvent(self, event: QCloseEvent) -> None:
        self._shutdown_preview_plan_executor()
        self._cancel_queued_pose(restore_command_target=True)
        now = time.monotonic()
        streams_fresh = self.node.control_streams_fresh(now)
        self._update_connected(now)
        hold_before_close = (
            not self.command_stream_suspended
            and self.hardware_mode in {"hold", "position"}
            and control_feedback_ready(
                self.have_first_state, streams_fresh, self.connected
            )
        )
        publish_before_close = True
        if self.command_stream_suspended:
            publish_before_close = False
        elif hold_before_close:
            if self.hardware_mode == "position":
                self._transition_position_to_fixed_hold()
            else:
                # Closing the window is not permission to recapture a HOLD
                # target from a possibly externally displaced feedback sample.
                self.moving_joint_mask = [False] * 6
            self._update_mode_label(
                "窗口关闭前保留已锁定目标；等待底层安全保持接管"
            )
        elif self.hardware_mode in {"hold", "position"}:
            self._suspend_command_stream(
                "窗口关闭且反馈不新鲜；不发送制动，交由底层租约安全保持"
            )
            publish_before_close = False
        else:
            self.machine.stop()
            self.hardware_mode = "brake"
            self.requested_active_joint_mask = [False] * 6
            self.pending_target_joint_mask = [False] * 6
            self.moving_joint_mask = [False] * 6

        def publish_requested_state_and_spin() -> None:
            self._publish_command()
            rclpy.spin_once(self.node, timeout_sec=0.01)

        if publish_before_close:
            for _ in range(5):
                if not run_ros_context_operation(
                    self._ros_context_ok, publish_requested_state_and_spin
                ):
                    break
        self.log_stream.flush()
        self.log_stream.close()
        for preview in (
            self.planned_mujoco_preview, self.actual_mujoco_preview,
        ):
            if preview is not None:
                preview.close_renderer()
        event.accept()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmGuiNode()
    app = QApplication(sys.argv)
    app.setApplicationDisplayName("六自由度机械臂控制系统")
    window = MainWindow(node)
    window.show()
    exit_code = app.exec()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()

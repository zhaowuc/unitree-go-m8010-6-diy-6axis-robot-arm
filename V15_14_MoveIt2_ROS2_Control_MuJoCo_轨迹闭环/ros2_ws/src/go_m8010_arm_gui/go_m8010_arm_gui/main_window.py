"""V15.30A-FT 六自由度机械臂全中文 PySide6 主窗口。"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import secrets
import struct
import sys
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

from PySide6.QtCore import QEvent, Qt, QTimer
from PySide6.QtGui import QCloseEvent, QFont, QImage, QPixmap
from PySide6.QtWidgets import (
    QAbstractScrollArea, QApplication, QDoubleSpinBox, QGridLayout, QGroupBox,
    QHBoxLayout, QLabel, QMainWindow, QMessageBox, QProgressBar, QPushButton,
    QScrollArea,
    QSizePolicy, QSlider, QVBoxLayout, QWidget,
)

from .state_machine import ArmMode, ArrivalTracker, MODE_TEXT, ModeMachine


JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
JOINT_LABELS = tuple(f"关节{index} J{index}" for index in range(1, 7))
MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
MOTOR_GROUPS = (
    ("J1",), ("J2A", "J2B"), ("J3",),
    ("J4",), ("J5",), ("J6",),
)
CONTROLLER_MODES = frozenset({"brake", "drag", "hold", "position", "unknown"})
DEG = 180.0 / math.pi
RAD = math.pi / 180.0
CONTROL_STREAM_TIMEOUT_S = 0.5
MUJOCO_STREAM_TIMEOUT_S = 0.5
ROUTER_STATUS_TIMEOUT_S = 1.5
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
PRODUCTION_COLLISION_CONTRACT_SHA256 = (
    "6d802909e44f238816f007ef33b57e1b57099c5522a473cd2dcc2705397af9c9"
)
PRODUCTION_KINEMATIC_GUARD_SHA256 = (
    "648b9de953a84be46cd5f95e24daf9f683f3025e73048dc068ef7bd8e5670c6a"
)
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
            or any(
                abs(float(measured) - float(start)) >
                COLLISION_START_MATCH_TOLERANCE_RAD + 1.0e-12
                for measured, start in zip(
                    hardware_position, request["start_relative_rad"]
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
    """Render the frozen MuJoCo model directly inside the control window."""

    def __init__(self, model_path: Path, session_pose_deg: str) -> None:
        super().__init__("MuJoCo实时三维预览")
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
        self.renderer = mujoco.Renderer(self.model, height=420, width=520)
        self.camera = mujoco.MjvCamera()
        mujoco.mjv_defaultCamera(self.camera)
        self.camera.lookat[:] = self.model.stat.center
        self.camera.distance = max(0.5, 1.65 * float(self.model.stat.extent))
        self.camera.azimuth = 135.0
        self.camera.elevation = -22.0
        self.last_relative: Optional[np.ndarray] = None
        self.last_image: Optional[QImage] = None
        self.mouse_position = None
        self.set_relative_pose([0.0] * 6, force=True)

    def set_relative_pose(self, relative_rad, force: bool = False) -> None:
        relative = np.asarray(relative_rad, dtype=float)
        if relative.shape != (6,) or not np.all(np.isfinite(relative)):
            return
        if not force and self.last_relative is not None and np.allclose(
            relative, self.last_relative, atol=1.0e-7, rtol=0.0
        ):
            return
        absolute = self.session_pose + relative
        self.data.qpos[self.qpos_addresses] = absolute
        self.data.qvel[:] = 0.0
        mujoco.mj_forward(self.model, self.data)
        if self.last_relative is None:
            self.fit_camera(render=False)
        self._render_scene()
        absolute_deg = np.degrees(absolute)
        self.pose_text.setText(
            "绝对姿态：" + "　".join(
                f"J{index + 1}={value:+.2f}°"
                for index, value in enumerate(absolute_deg)
            )
        )
        self.last_relative = relative.copy()

    def _render_scene(self) -> None:
        self.renderer.update_scene(self.data, camera=self.camera)
        pixels = np.ascontiguousarray(self.renderer.render())
        height, width, channels = pixels.shape
        if channels != 3:
            raise RuntimeError("MuJoCo渲染图像通道数异常")
        image = QImage(
            pixels.data, width, height, width * channels, QImage.Format_RGB888
        ).copy()
        self.last_image = image
        self._update_pixmap()

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

    def fit_camera(self, _checked: bool = False, render: bool = True) -> None:
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
        if render and hasattr(self, "renderer"):
            self._render_scene()

    def reset_camera(self, _checked: bool = False) -> None:
        self.camera.azimuth = 135.0
        self.camera.elevation = -22.0
        self.fit_camera(render=True)

    def set_camera_view(self, azimuth: float, elevation: float) -> None:
        self.camera.azimuth = azimuth
        self.camera.elevation = elevation
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
                    self.camera.azimuth += 0.45 * delta.x()
                    self.camera.elevation = float(np.clip(
                        self.camera.elevation - 0.45 * delta.y(), -89.0, 89.0
                    ))
                    self._render_scene()
                    return True
                if buttons & (Qt.RightButton | Qt.MiddleButton):
                    scale = max(1.0e-4, self.camera.distance * 0.0015)
                    angle = math.radians(self.camera.azimuth)
                    right = np.array([math.cos(angle), math.sin(angle), 0.0])
                    self.camera.lookat[:] -= delta.x() * scale * right
                    self.camera.lookat[2] += delta.y() * scale
                    self._render_scene()
                    return True
            if event.type() == QEvent.MouseButtonRelease:
                self.mouse_position = None
                return True
            if event.type() == QEvent.Wheel:
                steps = event.angleDelta().y() / 120.0
                self.camera.distance = float(np.clip(
                    self.camera.distance * math.exp(-0.12 * steps), 0.08, 20.0
                ))
                self._render_scene()
                return True
        return super().eventFilter(watched, event)

    def resizeEvent(self, event) -> None:
        super().resizeEvent(event)
        self._update_pixmap()

    def close_renderer(self) -> None:
        self.renderer.close()


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


@dataclass
class RealWidgets:
    slider: QSlider
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
        self.command_publisher = self.create_publisher(String, "/whole_arm/gui_command", 10)
        self.target_publisher = self.create_publisher(Float64MultiArray, "/whole_arm/gui_targets", 10)
        self.mode_publisher = self.create_publisher(String, "/whole_arm/gui_mode", 10)
        self.collision_request_publisher = self.create_publisher(
            String, "/whole_arm/collision_guard_request", 10
        )
        self.latest_joint_state: Optional[JointState] = None
        self.latest_hardware: Optional[dict] = None
        self.latest_mujoco: Optional[dict] = None
        self.latest_control_status: Optional[dict] = None
        self.latest_collision_result: Optional[dict] = None
        self.last_joint_receipt = 0.0
        self.last_hardware_receipt = 0.0
        self.last_mujoco_receipt = 0.0
        self.last_control_status_receipt = 0.0
        self.last_collision_result_receipt = 0.0
        self.command_source_instance_id = secrets.token_hex(16)
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

    def publish_command(
        self, sequence: int, mode: str, command_targets: list[float],
        virtual_targets: list[float], active_joint_mask: list[bool],
        moving_joint_mask: list[bool],
        activation_epoch: int, config: dict,
        collision_guard_proof: Optional[dict] = None,
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
            payload["collision_guard_proof"] = json.loads(
                json.dumps(collision_guard_proof)
            )
        self.command_publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.target_publisher.publish(Float64MultiArray(data=virtual_targets))

    def publish_collision_request(self, payload: dict) -> None:
        self.collision_request_publisher.publish(
            String(data=json.dumps(payload, ensure_ascii=False))
        )


class MainWindow(QMainWindow):
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
        self.direction = ArmMode.REAL_TO_SIM
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
        self.mujoco_preview: Optional[EmbeddedMujocoPreview] = None
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
        try:
            self.mujoco_preview = EmbeddedMujocoPreview(
                self.node.embedded_model_path,
                self.node.embedded_session_pose_deg,
            )
            columns.addWidget(self.mujoco_preview, 4)
        except Exception as exc:
            unavailable = QGroupBox("MuJoCo实时三维预览")
            unavailable_layout = QVBoxLayout(unavailable)
            unavailable_label = QLabel(f"内嵌模型不可用：{exc}")
            unavailable_label.setWordWrap(True)
            unavailable_label.setAlignment(Qt.AlignCenter)
            unavailable_layout.addWidget(unavailable_label)
            columns.addWidget(unavailable, 4)
        outer.addLayout(columns, 1)
        outer.addWidget(self._control_panel())

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
        self._set_virtual_editable(False)

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
        box = QGroupBox("虚拟机械臂（目标／仿真）")
        layout = QGridLayout(box)
        layout.setColumnStretch(2, 1)
        layout.addWidget(QLabel("关节"), 0, 0)
        layout.addWidget(QLabel("角度"), 0, 1)
        layout.addWidget(QLabel("精确输入"), 0, 2)
        for index, label_text in enumerate(JOINT_LABELS):
            edit_bounds = self.edit_limits[index]
            absolute_bounds = self.absolute_limits[index]
            value = QLabel("+0.00°")
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
            spin.valueChanged.connect(lambda degrees, i=index: self._virtual_spin_changed(i, degrees))
            row = index + 1
            layout.addWidget(QLabel(label_text), row, 0)
            layout.addWidget(value, row, 1)
            layout.addWidget(spin, row, 2)
            self.virtual_widgets.append(VirtualWidgets(value, spin))
        return box

    def _real_panel(self) -> QGroupBox:
        box = QGroupBox("现实机械臂（编码器反馈）")
        layout = QGridLayout(box)
        layout.setColumnStretch(1, 1)
        for column, text in enumerate(("关节", "实际滑条", "实际角度", "目标角度", "位置误差", "状态")):
            layout.addWidget(QLabel(text), 0, column)
        for index, label_text in enumerate(JOINT_LABELS):
            slider = self._slider(index)
            slider.setEnabled(False)
            widgets = RealWidgets(slider, QLabel("+0.00°"), QLabel("+0.00°"), QLabel("+0.00°"), QLabel("未连接"))
            row = index + 1
            layout.addWidget(QLabel(label_text), row, 0)
            for column, widget in enumerate((widgets.slider, widgets.actual, widgets.target, widgets.error, widgets.state), 1):
                layout.addWidget(widget, row, column)
            self.real_widgets.append(widgets)
        return box

    def _button(self, text: str, callback, object_name: str = "") -> QPushButton:
        button = QPushButton(text)
        if object_name:
            button.setObjectName(object_name)
        button.clicked.connect(callback)
        return button

    def _control_panel(self) -> QGroupBox:
        box = QGroupBox("控制模式与安全操作")
        layout = QGridLayout(box)
        buttons = [
            ("现实驱动虚拟", self._real_to_sim), ("虚拟驱动现实", self._sim_to_real),
            ("位置模式／重新接管（先保持当前角度）", self._position_mode),
            ("到位后固定保持开（位置伺服始终有效）", self._fixed_hold_after_arrival_on),
            ("到位后固定保持关（位置伺服始终有效）", self._fixed_hold_after_arrival_off),
            ("可拖动模式（普通脱力）", self._drag_mode),
            ("重新接管并保持当前位置", self._hold_current), ("设置初始化姿态", self._save_initial_pose),
            ("回到初始化姿态（仅虚拟预演）", self._return_initial_pose),
            ("确认执行现实轨迹", self._execute_target),
        ]
        for index, (text, callback) in enumerate(buttons):
            button = self._button(text, callback)
            if text == "设置初始化姿态":
                button.setEnabled(not self.node.initial_pose_read_only)
                if self.node.initial_pose_read_only:
                    button.setToolTip("生产初始化姿态受只读保护，禁止由GUI覆盖")
            if text == "确认执行现实轨迹":
                self.execute_target_button = button
                button.setEnabled(False)
                button.setToolTip("必须先获得与当前虚拟候选姿态完全匹配的SAFE预演结果")
            layout.addWidget(button, index // 5, index % 5)
        emergency = self._button(
            "紧急制动／撤销驱动（需可靠支撑）",
            self._emergency_brake,
        )
        emergency.setStyleSheet(
            "background: #4a0000; color: #ffeb3b; font-weight: bold; min-height: 46px;"
        )
        layout.addWidget(emergency, 2, 0, 1, 5)
        stop = self._button("停止轨迹并保持（承重HOLD）", self._stop)
        stop.setStyleSheet("background: #b71c1c; color: white; font-weight: bold; min-height: 46px;")
        layout.addWidget(stop, 3, 0, 1, 5)
        self.mode_label = QLabel(
            "当前方向：现实驱动虚拟　｜　控制请求：制动　｜　硬件确认：等待状态反馈"
        )
        self.mode_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.mode_label, 4, 0, 1, 5)
        servo_notice = QLabel(
            "说明：位置伺服无论“到位后固定保持”开或关，都会产生到达和维持目标所需的驱动力；"
            "“关”不是物理脱力。普通脱力只能使用“可拖动模式”；紧急撤销驱动请使用独立紧急制动按钮。"
        )
        servo_notice.setWordWrap(True)
        layout.addWidget(servo_notice, 5, 0, 1, 5)
        self.workflow_status = QLabel(
            "工作流：等待设置虚拟候选姿态；未授权现实运动"
        )
        self.workflow_status.setWordWrap(True)
        self.workflow_status.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.workflow_status, 6, 0, 1, 5)
        self.workflow_progress = QProgressBar()
        self.workflow_progress.setRange(0, 100)
        self.workflow_progress.setValue(0)
        self.workflow_progress.setFormat("虚拟预演未开始")
        layout.addWidget(self.workflow_progress, 7, 0, 1, 5)
        return box

    def _set_workflow_state(
        self, state: str, text: str, progress: Optional[int] = None,
    ) -> None:
        """Keep virtual solving/approval/real execution state permanently visible."""

        self.collision_preview_state = state
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

    def _clear_candidate_approval(self) -> None:
        """Revoke a preview approval without changing any hardware authority."""

        self.approved_candidate_sha256 = None
        self.approved_candidate_session_id = None
        self.approved_candidate_state_instance_id = None
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
            self.collision_preview_state == "safe"
            and self.approved_candidate_sha256 == digest
            and self.approved_candidate_session_id == self.session_id
            and self.approved_candidate_state_instance_id == self.state_instance_id
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
        self._refresh_execute_target_enabled()

    def _record_virtual_target(self, index: int, target_rad: float) -> None:
        """Record a virtual-first target without granting hardware authority."""

        self.candidate_targets[index] = target_rad
        self.targets[index] = target_rad
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
        self.collision_preview_timer.start(COLLISION_PREVIEW_DEBOUNCE_MS)

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

    def _new_collision_request(self, kind: str) -> Optional[dict]:
        if kind not in {"preview", "pose_preview", "execute"}:
            raise ValueError("碰撞检查类型无效")
        hardware = self.node.latest_hardware
        now_ns = time.monotonic_ns()
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
        if (
            self.direction is not ArmMode.SIM_TO_REAL
            or not self.session_id
            or not source_instance_id_valid(self.state_instance_id)
            or not self.node.control_streams_fresh()
            or self.command_stream_suspended
            # Each proof authorizes exactly one physical swept segment.  A
            # complete virtual pose is decomposed by _begin_next_queued_segment.
            or not moving_indices
            or (kind != "pose_preview" and len(moving_indices) != 1)
            or not collision_motion_state_ready(
                hardware,
                self.command_targets,
                self.requested_active_joint_mask,
                self.hardware_mode,
                now_ns=now_ns,
            )
            or hardware["session_id"] != self.session_id
            or hardware["state_instance_id"] != self.state_instance_id
        ):
            return None
        if not moving_targets_within_model_limits(
            target_values, [True] * 6, self.edit_limits
        ):
            return None
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
            "start_relative_rad": [
                float(value) for value in hardware["position_rad"]
            ],
            "target_relative_rad": [float(value) for value in target_values],
            "target_sha256": collision_target_sha256(target_values),
            "collision_margin_deg": COLLISION_MARGIN_DEG,
        }
        self.collision_requests[self.collision_request_sequence] = payload
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
        # Multi-joint editing checks the final pose immediately but never
        # grants motion authority.  Execute still uses fresh one-joint path
        # proofs for every physical segment.
        request = self._new_collision_request(
            "preview" if moving_count == 1 else "pose_preview"
        )
        if request is None:
            self._clear_candidate_approval()
            self._set_workflow_state(
                "waiting_hardware",
                "候选姿态已在虚拟机械臂中加载；"
                "等待六轴健康、静止且全部HOLD后启动完整3D解算",
                15,
            )
            return
        self.latest_collision_preview_sequence = request["request_sequence"]
        self.collision_preview_started_at = time.monotonic()
        self._set_workflow_state(
            "solving",
            f"正在解算候选姿态（请求{request['request_sequence']}）；"
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
        while self.queued_joint_indices:
            index = self.queued_joint_indices[0]
            final_target = self.queued_pose_target[index]
            delta = final_target - self.command_targets[index]
            if abs(delta) <= TARGET_SELECTION_DEADBAND_RAD:
                self.queued_joint_indices.pop(0)
                continue
            maximum_segment = math.radians(COLLISION_EXECUTE_SEGMENT_MAX_DEG)
            target = self.command_targets[index] + math.copysign(
                min(abs(delta), maximum_segment), delta
            )
            if abs(delta) <= maximum_segment + 1.0e-12:
                self.queued_joint_indices.pop(0)
            self.targets = list(self.command_targets)
            self.targets[index] = target
            self.pending_target_joint_mask = [False] * 6
            self.pending_target_joint_mask[index] = True
            self.active_sequence_joint = index
            self._show_targets_on_virtual()
            if self._request_collision_execute():
                self._notify(
                    f"虚拟目标已拆分：正在检查并执行 J{index + 1}，"
                    "其余关节保持原锁定角度。",
                    "info",
                )
                return
            self._cancel_queued_pose(restore_command_target=True)
            return

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
                self._set_workflow_state(
                    "safe",
                    f"SAFE：当前候选姿态通过完整3D预演"
                    f"{pose_text}，用时{elapsed:.2f}秒；"
                    "请目视虚拟机械臂，确认合理后才可执行现实轨迹",
                    100,
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
        if self.collision_preview_state == "solving" and preview_sequence is not None:
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
            "关节与硬件状态流需保持新鲜，且至少一个关节健康连接，当前不能保持。"
        ):
            return
        self._prepare_hold_at_actual()
        self._update_mode_label("已请求健康关节保持当前位置；等待硬件确认")

    def _prepare_hold_at_actual(self) -> None:
        """Freeze fresh feedback as a HOLD request without claiming confirmation."""
        self._cancel_queued_pose(restore_command_target=False)
        self.targets = list(self.actual)
        self.command_targets = list(self.actual)
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
        if not self._require_control_feedback(
            "执行现实目标要求六个关节均有新鲜、健康反馈。",
            require_all=True,
        ):
            return
        if self.direction is not ArmMode.SIM_TO_REAL:
            self._notify("请先选择“虚拟驱动现实”。")
            return
        if self.command_stream_suspended:
            self._notify(
                "GUI命令流已暂停；请先在新鲜六轴反馈下重新接管HOLD，"
                "未授权新运动。",
                "warning",
            )
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
        pending_indices = [
            index
            for index, (target, command_target, connected) in enumerate(zip(
                self.targets, self.command_targets, self.connected
            ))
            if connected
            and abs(target - command_target) > TARGET_SELECTION_DEADBAND_RAD
        ]
        self.pending_target_joint_mask = [
            index in pending_indices for index in range(6)
        ]
        if not pending_indices:
            if self.hardware_mode in {"brake", "drag"}:
                self._prepare_hold_at_actual()
                self._notify(
                    "没有选中新目标；已锁定当前实际姿态并请求承重HOLD。",
                    "info",
                )
            else:
                # Repeated Execute in an authoritative HOLD/POSITION epoch is
                # not permission to adopt an externally displaced sample.
                self._notify(
                    "没有选中新目标；保留现有权威目标与命令流，未重采样实际角度。",
                    "info",
                )
            return
        if not collision_motion_state_ready(
            self.node.latest_hardware,
            self.command_targets,
            self.requested_active_joint_mask,
            self.hardware_mode,
        ):
            self._notify(
                "现实下发前必须确认六轴均健康、静止并处于承重HOLD；"
                "新目标尚未授权，原锁定角度继续保持。",
                "warning",
            )
            return
        if not moving_targets_within_model_limits(
            self.targets, [True] * 6, self.edit_limits
        ):
            self._notify(
                "目标超出历史3D模型关节范围；新目标未发布，原位置保持继续。",
                "warning",
            )
            self._update_mode_label("模型机械范围外目标已拒绝；原位置保持继续")
            return
        # Preserve the complete virtual pose, then expose only one changed
        # joint to each physical proof/command epoch.  This is the strongest
        # executable contract available across the independent GO/J2/J6
        # transports: the operator edits once, while every real segment is
        # freshly swept against the actual pose reached by the previous one.
        self.queued_pose_target = list(self.targets)
        self.queued_joint_indices = list(pending_indices)
        self.active_sequence_joint = None
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
        self.machine.position()
        self.hardware_mode = "position"
        self.command_targets = list(self.targets)
        self.moving_joint_mask = candidate_moving_joint_mask
        self.active_collision_proof = json.loads(json.dumps(approved_result))
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
            self.targets = [float(positions[f"J{i + 1}"]) for i in range(6)]
            if not all(
                math.isfinite(value) and self.limits[index][0] * RAD <= value <= self.limits[index][1] * RAD
                for index, value in enumerate(self.targets)
            ):
                raise ValueError
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            self._notify("初始化姿态需要重新设置。")
            return
        self.direction = ArmMode.SIM_TO_REAL
        self.pending_target_joint_mask = [
            connected and abs(target - command) > TARGET_SELECTION_DEADBAND_RAD
            for target, command, connected in zip(
                self.targets, self.command_targets, self.connected
            )
        ]
        self._set_virtual_editable(True)
        self._show_targets_on_virtual()
        self._execute_target()

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
        for index, target in enumerate(self.targets):
            degrees = target * DEG
            widgets = self.virtual_widgets[index]
            widgets.slider.blockSignals(True)
            widgets.target.blockSignals(True)
            widgets.slider.setValue(round(degrees * 100.0))
            widgets.target.setValue(degrees)
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
                self.have_first_state = True
        self._consume_collision_guard_result()
        self._expire_collision_request(now)
        self._update_connected(now)
        if not self.node.control_streams_fresh(now):
            self._suspend_for_stale_feedback()
        self._refresh_joint_widgets()
        # Arrival first requests the exact-target HOLD barrier.  A later tick
        # observes all seven motors actually reporting HOLD and stationary;
        # only then may this advance to a freshly checked next segment.
        self._continue_queued_sequence_if_ready()
        if self.mujoco_preview is not None:
            preview_pose = (
                self.targets
                if self.direction is ArmMode.SIM_TO_REAL
                else self.actual
            )
            try:
                self.mujoco_preview.set_relative_pose(preview_pose)
            except Exception as exc:
                self.mujoco_preview.image.setText(f"MuJoCo渲染失败：{exc}")
        if not run_ros_context_operation(self._ros_context_ok, self._publish_command):
            self._close_for_ros_shutdown()
            return
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
        self.activation_epoch += 1

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
                and all(
                    states[index] in {"已到位", "保持中"}
                    and confirmed_modes[index] == "position"
                    for index in moving_indices
                )
            ):
                sequence_joint = self.active_sequence_joint
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
                    self.targets = list(self.command_targets)
                    self.pending_target_joint_mask = [False] * 6
                    self._show_targets_on_virtual()
                    self.arrival.start(now)
                    if sequence_joint is not None:
                        # A physical sequence always inserts an exact-target
                        # HOLD barrier, even if the optional final policy is
                        # "continue POSITION".  Only confirmed all-axis HOLD
                        # may authorize the next separately proven segment.
                        self.active_sequence_joint = None
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
        hardware = self.node.latest_hardware or {}
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
        self.command_sequence += 1
        self.node.publish_command(
            self.command_sequence, self.hardware_mode,
            self.command_targets, self.targets, active_joint_mask,
            self.moving_joint_mask,
            self.activation_epoch, self.config,
            self.active_collision_proof,
        )
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
            notices.append(("J2同步状态异常；界面显示不代表该轴仍在保持", "critical"))
        elif j2_observed and sync_error > math.radians(0.5):
            notices.append((
                "J2单帧同步偏差正在硬件端连续确认；"
                "当前不撤销位置保持。",
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
        if self.mujoco_preview is not None:
            self.mujoco_preview.close_renderer()
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

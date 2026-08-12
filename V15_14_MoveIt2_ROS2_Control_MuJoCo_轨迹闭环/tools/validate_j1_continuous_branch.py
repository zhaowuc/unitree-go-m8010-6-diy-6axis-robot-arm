#!/usr/bin/env python3
"""Fail-closed J1 +/-2*pi branch regression for the V15.14 closed loop.

This test deliberately sends two standard FollowJointTrajectory goals whose
J1 positions use neighbouring 2*pi representations.  The controller command
must preserve that requested representation while the authoritative MuJoCo
state remains continuous on the physical branch nearest its current state.

Run this only against a fresh V15.14 bringup in an isolated ROS_DOMAIN_ID.  The
fresh instance must start at the six-joint mechanical zero; the script refuses
to move the robot otherwise.  No private bridge command interface is used.
"""

from __future__ import annotations

import argparse
import bisect
import copy
import json
import math
import os
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Iterable

import rclpy
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")
ACTION_NAME = "/arm_controller/follow_joint_trajectory"
COMMAND_TOPIC = "/mujoco/joint_commands"
RAW_STATE_TOPIC = "/mujoco/joint_states_raw"
CONTROLLER_STATE_TOPIC = "/arm_controller/controller_state"
BRIDGE_STATUS_TOPIC = "/mujoco_bridge/status"
BRIDGE_STATUS_SCHEMA = "go-m8010-arm-v15.14-mujoco-bridge-status/1.0"

TWO_PI = 2.0 * math.pi
BRANCH_OFFSET_TOLERANCE_RAD = 0.02
BRANCH_MAX_ADJUSTMENT_TOLERANCE_RAD = 1.0e-9
RAW_JUMP_LIMIT_RAD = 0.05
RAW_VELOCITY_LIMIT_RAD_S = 0.52
BRIDGE_ACCELERATION_LIMIT_RAD_S2 = 1.02
OTHER_JOINT_LIMIT_RAD = 1.0e-5
FINAL_POSITION_TOLERANCE_RAD = 1.0e-3
CONTROLLER_WRAPPED_TRACKING_LIMIT_RAD = 0.03
MIN_DYNAMIC_SAMPLE_COUNT = 20
PAIR_MAX_RECEIPT_DELTA_S = 0.08


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def ordered_vector(
    names: Iterable[str], values: Iterable[float]
) -> list[float] | None:
    names_list = list(names)
    values_list = list(values)
    if len(names_list) != len(values_list) or set(names_list) != set(JOINTS):
        return None
    lookup = dict(zip(names_list, values_list))
    result = [float(lookup[name]) for name in JOINTS]
    return result if all(math.isfinite(value) for value in result) else None


def optional_ordered_vector(
    names: Iterable[str], values: Iterable[float]
) -> list[float]:
    values_list = list(values)
    if not values_list:
        return []
    result = ordered_vector(names, values_list)
    return result if result is not None else []


def wrapped_difference(first: float, second: float) -> float:
    """Return first-second on the nearest angular branch."""
    return math.atan2(math.sin(first - second), math.cos(first - second))


def max_or_none(values: Iterable[float]) -> float | None:
    materialized = list(values)
    return max(materialized) if materialized else None


def bridge_projection(status: dict[str, Any]) -> dict[str, Any]:
    normalization = status.get("j1_continuous_branch_normalization", {})
    envelope = status.get("simulation_execution_envelope", {})
    guard = status.get("last_guard_result", {})
    return {
        "schema": status.get("schema"),
        "fault_latched": status.get("fault_latched"),
        "command_stream_primed": status.get("command_stream_primed"),
        "accepted_command_count": status.get("accepted_command_count"),
        "accepted_moving_command_count": status.get(
            "accepted_moving_command_count"
        ),
        "rejected_command_count": status.get("rejected_command_count"),
        "moving_watchdog_timeout_count": status.get(
            "moving_watchdog_timeout_count"
        ),
        "joint_position_rad": status.get("joint_position_rad"),
        "joint_velocity_rad_s": status.get("joint_velocity_rad_s"),
        "last_guard_safe": guard.get("safe"),
        "last_guard_reason": guard.get("reason"),
        "j1_continuous_branch_normalization": {
            "enabled": normalization.get("enabled"),
            "joint_name": normalization.get("joint_name"),
            "method": normalization.get("method"),
            "accepted_adjustment_count": normalization.get(
                "accepted_adjustment_count"
            ),
            "max_abs_adjustment_rad": normalization.get(
                "max_abs_adjustment_rad"
            ),
        },
        "execution_observations": {
            "max_observed_command_velocity_rad_s": envelope.get(
                "max_observed_command_velocity_rad_s"
            ),
            "max_observed_command_acceleration_rad_s2": envelope.get(
                "max_observed_command_acceleration_rad_s2"
            ),
        },
    }


def counter(status: dict[str, Any], name: str) -> int:
    value = status.get(name)
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"bridge status counter {name!r} is invalid: {value!r}")
    return value


def branch_adjustment_count(status: dict[str, Any]) -> int:
    normalization = status.get("j1_continuous_branch_normalization", {})
    value = normalization.get("accepted_adjustment_count")
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(
            "bridge J1 accepted_adjustment_count is invalid: " f"{value!r}"
        )
    return value


def branch_max_adjustment(status: dict[str, Any]) -> float:
    normalization = status.get("j1_continuous_branch_normalization", {})
    value = normalization.get("max_abs_adjustment_rad")
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(
            "bridge J1 max_abs_adjustment_rad is invalid: " f"{value!r}"
        )
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("bridge J1 max_abs_adjustment_rad is non-finite")
    return result


class BranchRegression(Node):
    def __init__(self) -> None:
        super().__init__("go_m8010_arm_v15_14_j1_branch_regression")
        self.action_client = ActionClient(
            self, FollowJointTrajectory, ACTION_NAME
        )
        self.latest_raw: dict[str, Any] | None = None
        self.latest_status: dict[str, Any] = {}
        self.latest_controller: dict[str, Any] | None = None
        self.active_segment: dict[str, list[dict[str, Any]]] | None = None

        self.create_subscription(
            JointState, COMMAND_TOPIC, self.command_callback, 100
        )
        self.create_subscription(
            JointState, RAW_STATE_TOPIC, self.raw_callback, 100
        )
        self.create_subscription(
            JointTrajectoryControllerState,
            CONTROLLER_STATE_TOPIC,
            self.controller_callback,
            100,
        )
        self.create_subscription(
            String, BRIDGE_STATUS_TOPIC, self.status_callback, 100
        )

    def command_callback(self, message: JointState) -> None:
        position = ordered_vector(message.name, message.position)
        if position is None:
            return
        row = {
            "receipt_monotonic_ns": time.monotonic_ns(),
            "source_stamp_ns": stamp_ns(message),
            "position_rad": position,
            "velocity_rad_s": optional_ordered_vector(
                message.name, message.velocity
            ),
        }
        if self.active_segment is not None:
            self.active_segment["command"].append(row)

    def raw_callback(self, message: JointState) -> None:
        position = ordered_vector(message.name, message.position)
        if position is None:
            return
        row = {
            "receipt_monotonic_ns": time.monotonic_ns(),
            "source_stamp_ns": stamp_ns(message),
            "position_rad": position,
            "velocity_rad_s": optional_ordered_vector(
                message.name, message.velocity
            ),
        }
        self.latest_raw = row
        if self.active_segment is not None:
            self.active_segment["raw_state"].append(row)

    def controller_callback(
        self, message: JointTrajectoryControllerState
    ) -> None:
        desired_position = ordered_vector(
            message.joint_names, message.desired.positions
        )
        actual_position = ordered_vector(
            message.joint_names, message.actual.positions
        )
        if desired_position is None or actual_position is None:
            return
        row = {
            "receipt_monotonic_ns": time.monotonic_ns(),
            "source_stamp_ns": stamp_ns(message),
            "desired_position_rad": desired_position,
            "desired_velocity_rad_s": optional_ordered_vector(
                message.joint_names, message.desired.velocities
            ),
            "actual_position_rad": actual_position,
            "actual_velocity_rad_s": optional_ordered_vector(
                message.joint_names, message.actual.velocities
            ),
            "error_position_rad": optional_ordered_vector(
                message.joint_names, message.error.positions
            ),
        }
        self.latest_controller = row
        if self.active_segment is not None:
            self.active_segment["controller_state"].append(row)

    def status_callback(self, message: String) -> None:
        try:
            parsed = json.loads(message.data)
        except json.JSONDecodeError:
            self.latest_status = {"parse_error": message.data}
            return
        self.latest_status = parsed
        if self.active_segment is not None:
            self.active_segment["bridge_status"].append(
                {
                    "receipt_monotonic_ns": time.monotonic_ns(),
                    **bridge_projection(parsed),
                }
            )

    def spin_for(self, seconds: float) -> None:
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_future(self, future: Any, timeout_s: float) -> Any:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
        if not future.done():
            raise TimeoutError(f"ROS future timed out after {timeout_s}s")
        return future.result()

    def wait_ready(self, timeout_s: float) -> None:
        if not self.action_client.wait_for_server(timeout_sec=timeout_s):
            raise TimeoutError(f"action server unavailable: {ACTION_NAME}")
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
            normalization = self.latest_status.get(
                "j1_continuous_branch_normalization", {}
            )
            if (
                self.latest_raw is not None
                and self.latest_controller is not None
                and self.latest_status.get("schema") == BRIDGE_STATUS_SCHEMA
                and self.latest_status.get("command_stream_primed") is True
                and self.latest_status.get("fault_latched") is False
                and normalization.get("enabled") is True
                and normalization.get("joint_name") == "J1"
                and normalization.get("method")
                == "nearest_equivalent_to_current"
            ):
                return
        raise TimeoutError("V15.14 controller/bridge did not become ready")

    def publisher_audit(self) -> dict[str, Any]:
        audit: dict[str, Any] = {}
        for topic in (
            COMMAND_TOPIC,
            RAW_STATE_TOPIC,
            CONTROLLER_STATE_TOPIC,
            BRIDGE_STATUS_TOPIC,
        ):
            endpoints = self.get_publishers_info_by_topic(topic)
            audit[topic] = {
                "publisher_count": len(endpoints),
                "publishers": [
                    {
                        "node_name": endpoint.node_name,
                        "node_namespace": endpoint.node_namespace,
                        "topic_type": endpoint.topic_type,
                    }
                    for endpoint in endpoints
                ],
            }
        return audit

    def run_goal(
        self,
        segment_id: str,
        start_j1_rad: float,
        final_j1_rad: float,
        duration_s: float,
        timeout_s: float,
    ) -> dict[str, Any]:
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINTS)
        start = JointTrajectoryPoint()
        start.positions = [start_j1_rad, 0.0, 0.0, 0.0, 0.0, 0.0]
        start.velocities = [0.0] * 6
        start.time_from_start = Duration(sec=0, nanosec=0)
        finish = JointTrajectoryPoint()
        finish.positions = [final_j1_rad, 0.0, 0.0, 0.0, 0.0, 0.0]
        finish.velocities = [0.0] * 6
        total_duration_ns = int(round(duration_s * 1.0e9))
        whole_seconds, remaining_nanoseconds = divmod(
            total_duration_ns, 1_000_000_000
        )
        finish.time_from_start = Duration(
            sec=whole_seconds,
            nanosec=remaining_nanoseconds,
        )
        goal.trajectory.points = [start, finish]
        goal.goal_time_tolerance = Duration(sec=2, nanosec=0)

        before = copy.deepcopy(self.latest_status)
        traces: dict[str, list[dict[str, Any]]] = {
            "command": [],
            "raw_state": [],
            "controller_state": [],
            "bridge_status": [],
        }
        self.active_segment = traces
        started_monotonic_ns = time.monotonic_ns()
        goal_handle = self.wait_future(
            self.action_client.send_goal_async(goal), timeout_s=10.0
        )
        result_record: dict[str, Any] = {
            "accepted": bool(goal_handle.accepted),
            "goal_uuid": bytes(goal_handle.goal_id.uuid).hex(),
            "status": None,
            "error_code": None,
            "error_string": None,
        }
        if goal_handle.accepted:
            wrapper = self.wait_future(
                goal_handle.get_result_async(), timeout_s=timeout_s
            )
            result_record.update(
                {
                    "status": int(wrapper.status),
                    "error_code": int(wrapper.result.error_code),
                    "error_string": str(wrapper.result.error_string),
                }
            )
        self.spin_for(0.40)
        finished_monotonic_ns = time.monotonic_ns()
        self.active_segment = None
        after = copy.deepcopy(self.latest_status)

        return {
            "id": segment_id,
            "goal": {
                "action_name": ACTION_NAME,
                "joint_names": list(JOINTS),
                "points": [
                    {
                        "time_from_start_s": 0.0,
                        "position_rad": list(start.positions),
                        "velocity_rad_s": list(start.velocities),
                    },
                    {
                        "time_from_start_s": duration_s,
                        "position_rad": list(finish.positions),
                        "velocity_rad_s": list(finish.velocities),
                    },
                ],
            },
            "result": result_record,
            "elapsed_wall_s": (
                finished_monotonic_ns - started_monotonic_ns
            )
            * 1.0e-9,
            "bridge_before": bridge_projection(before),
            "bridge_after": bridge_projection(after),
            "traces": traces,
            "_bridge_before_full": before,
            "_bridge_after_full": after,
        }


def pair_branch_offsets(
    commands: list[dict[str, Any]],
    raw_samples: list[dict[str, Any]],
    expected_offset_rad: float,
) -> dict[str, Any]:
    if expected_offset_rad > 0.0:
        branch_commands = [
            row for row in commands if row["position_rad"][0] > math.pi
        ]
    else:
        branch_commands = [
            row for row in commands if row["position_rad"][0] < -math.pi
        ]
    raw_sorted = sorted(
        raw_samples, key=lambda row: row["receipt_monotonic_ns"]
    )
    raw_times = [row["receipt_monotonic_ns"] for row in raw_sorted]
    offsets: list[float] = []
    receipt_deltas_s: list[float] = []
    for command in branch_commands:
        command_time = command["receipt_monotonic_ns"]
        insertion = bisect.bisect_left(raw_times, command_time)
        candidates = [
            index
            for index in (insertion - 1, insertion)
            if 0 <= index < len(raw_sorted)
        ]
        if not candidates:
            continue
        nearest = min(
            candidates,
            key=lambda index: abs(raw_times[index] - command_time),
        )
        delta_s = abs(raw_times[nearest] - command_time) * 1.0e-9
        if delta_s > PAIR_MAX_RECEIPT_DELTA_S:
            continue
        offsets.append(
            float(
                command["position_rad"][0]
                - raw_sorted[nearest]["position_rad"][0]
            )
        )
        receipt_deltas_s.append(delta_s)
    residuals = [abs(value - expected_offset_rad) for value in offsets]
    inliers = [
        residual <= BRANCH_OFFSET_TOLERANCE_RAD for residual in residuals
    ]
    return {
        "branch_command_sample_count": len(branch_commands),
        "paired_sample_count": len(offsets),
        "expected_command_minus_raw_rad": expected_offset_rad,
        "median_command_minus_raw_rad": (
            statistics.median(offsets) if offsets else None
        ),
        "max_abs_offset_residual_rad": max_or_none(residuals),
        "inlier_tolerance_rad": BRANCH_OFFSET_TOLERANCE_RAD,
        "inlier_fraction": (
            sum(inliers) / len(inliers) if inliers else None
        ),
        "max_receipt_pair_delta_s": max_or_none(receipt_deltas_s),
    }


def analyze_segment(
    segment: dict[str, Any],
    expected_offset_rad: float,
    expected_final_physical_j1_rad: float,
) -> None:
    traces = segment["traces"]
    commands = traces["command"]
    raw_samples = traces["raw_state"]
    controller_samples = traces["controller_state"]
    before = segment.pop("_bridge_before_full")
    after = segment.pop("_bridge_after_full")

    raw_jumps = [
        abs(
            second["position_rad"][0] - first["position_rad"][0]
        )
        for first, second in zip(raw_samples, raw_samples[1:])
    ]
    raw_velocities = [
        abs(value)
        for row in raw_samples
        for value in row["velocity_rad_s"]
    ]
    other_positions = [
        abs(value)
        for row in raw_samples
        for value in row["position_rad"][1:]
    ]
    other_velocities = [
        abs(value)
        for row in raw_samples
        for value in row["velocity_rad_s"][1:]
    ]
    controller_errors = [
        abs(
            wrapped_difference(actual, desired)
            if joint_index == 0
            else actual - desired
        )
        for row in controller_samples
        for joint_index, (desired, actual) in enumerate(
            zip(
                row["desired_position_rad"],
                row["actual_position_rad"],
            )
        )
    ]
    final_position = raw_samples[-1]["position_rad"] if raw_samples else None
    final_error = (
        max(
            abs(final_position[0] - expected_final_physical_j1_rad),
            *(abs(value) for value in final_position[1:]),
        )
        if final_position is not None
        else None
    )
    pairing = pair_branch_offsets(
        commands, raw_samples, expected_offset_rad
    )

    adjustment_delta = branch_adjustment_count(after) - branch_adjustment_count(
        before
    )
    rejected_delta = counter(after, "rejected_command_count") - counter(
        before, "rejected_command_count"
    )
    watchdog_delta = counter(
        after, "moving_watchdog_timeout_count"
    ) - counter(before, "moving_watchdog_timeout_count")
    moving_delta = counter(
        after, "accepted_moving_command_count"
    ) - counter(before, "accepted_moving_command_count")
    maximum_adjustment = branch_max_adjustment(after)
    envelope = after.get("simulation_execution_envelope", {})
    observed_velocity = envelope.get("max_observed_command_velocity_rad_s")
    observed_acceleration = envelope.get(
        "max_observed_command_acceleration_rad_s2"
    )

    metrics = {
        "sample_count": {
            "command": len(commands),
            "raw_state": len(raw_samples),
            "controller_state": len(controller_samples),
            "bridge_status": len(traces["bridge_status"]),
        },
        "branch_pairing": pairing,
        "max_consecutive_raw_j1_jump_rad": max_or_none(raw_jumps),
        "max_abs_raw_velocity_rad_s": max_or_none(raw_velocities),
        "max_abs_other_joint_position_rad": max_or_none(other_positions),
        "max_abs_other_joint_velocity_rad_s": max_or_none(other_velocities),
        "max_controller_wrapped_tracking_error_rad": max_or_none(
            controller_errors
        ),
        "expected_final_physical_position_rad": [
            expected_final_physical_j1_rad,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
        ],
        "actual_final_physical_position_rad": final_position,
        "max_final_physical_position_error_rad": final_error,
        "bridge_counter_delta": {
            "j1_accepted_adjustment_count": adjustment_delta,
            "accepted_moving_command_count": moving_delta,
            "rejected_command_count": rejected_delta,
            "moving_watchdog_timeout_count": watchdog_delta,
        },
        "bridge_max_abs_j1_branch_adjustment_rad": maximum_adjustment,
        "bridge_max_observed_command_velocity_rad_s": observed_velocity,
        "bridge_max_observed_command_acceleration_rad_s2": (
            observed_acceleration
        ),
    }
    median_offset = pairing["median_command_minus_raw_rad"]
    inlier_fraction = pairing["inlier_fraction"]
    gates = {
        "standard_follow_joint_trajectory_goal_accepted": segment["result"][
            "accepted"
        ],
        "action_status_succeeded": segment["result"]["status"]
        == GoalStatus.STATUS_SUCCEEDED,
        "controller_result_successful": segment["result"]["error_code"]
        == FollowJointTrajectory.Result.SUCCESSFUL,
        "minimum_command_samples": len(commands) >= MIN_DYNAMIC_SAMPLE_COUNT,
        "minimum_raw_state_samples": len(raw_samples)
        >= MIN_DYNAMIC_SAMPLE_COUNT,
        "minimum_controller_state_samples": len(controller_samples)
        >= MIN_DYNAMIC_SAMPLE_COUNT,
        "paired_branch_samples_present": pairing["paired_sample_count"]
        >= MIN_DYNAMIC_SAMPLE_COUNT,
        "command_minus_raw_branch_offset": (
            median_offset is not None
            and abs(median_offset - expected_offset_rad)
            <= BRANCH_OFFSET_TOLERANCE_RAD
            and inlier_fraction is not None
            and inlier_fraction >= 0.95
        ),
        "raw_j1_has_no_branch_jump": bool(
            raw_jumps and max(raw_jumps) <= RAW_JUMP_LIMIT_RAD
        ),
        "raw_velocity_within_envelope": bool(
            raw_velocities
            and max(raw_velocities) <= RAW_VELOCITY_LIMIT_RAD_S
        ),
        "other_joints_remain_zero": bool(
            other_positions
            and other_velocities
            and max(other_positions) <= OTHER_JOINT_LIMIT_RAD
            and max(other_velocities) <= OTHER_JOINT_LIMIT_RAD
        ),
        "controller_wrapped_tracking_error_within_limit": bool(
            controller_errors
            and max(controller_errors)
            <= CONTROLLER_WRAPPED_TRACKING_LIMIT_RAD
        ),
        "final_physical_position_matches": final_error is not None
        and final_error <= FINAL_POSITION_TOLERANCE_RAD,
        "bridge_adjusted_accepted_commands": adjustment_delta > 0,
        "bridge_max_adjustment_is_two_pi": abs(
            maximum_adjustment - TWO_PI
        )
        <= BRANCH_MAX_ADJUSTMENT_TOLERANCE_RAD,
        "bridge_accepted_moving_commands": moving_delta > 0,
        "bridge_rejected_no_commands": rejected_delta == 0,
        "bridge_watchdog_no_timeout": watchdog_delta == 0,
        "bridge_fault_not_latched": after.get("fault_latched") is False,
        "bridge_guard_safe": after.get("last_guard_result", {}).get("safe")
        is True,
        "bridge_observed_velocity_within_envelope": isinstance(
            observed_velocity, (int, float)
        )
        and not isinstance(observed_velocity, bool)
        and float(observed_velocity) <= RAW_VELOCITY_LIMIT_RAD_S,
        "bridge_observed_acceleration_within_envelope": isinstance(
            observed_acceleration, (int, float)
        )
        and not isinstance(observed_acceleration, bool)
        and float(observed_acceleration) <= BRIDGE_ACCELERATION_LIMIT_RAD_S2,
    }
    segment["metrics"] = metrics
    segment["gates"] = gates
    segment["pass"] = all(gates.values())


def validate_initial_state(node: BranchRegression) -> dict[str, Any]:
    raw = copy.deepcopy(node.latest_raw)
    status = copy.deepcopy(node.latest_status)
    topology = node.publisher_audit()
    position = raw["position_rad"] if raw else None
    velocity = raw["velocity_rad_s"] if raw else None
    normalization = status.get("j1_continuous_branch_normalization", {})
    gates = {
        "raw_state_present": raw is not None,
        "mechanical_zero_position": position is not None
        and max(abs(value) for value in position) <= FINAL_POSITION_TOLERANCE_RAD,
        "stationary_velocity": bool(velocity)
        and max(abs(value) for value in velocity) <= OTHER_JOINT_LIMIT_RAD,
        "bridge_schema": status.get("schema") == BRIDGE_STATUS_SCHEMA,
        "bridge_primed": status.get("command_stream_primed") is True,
        "bridge_fault_not_latched": status.get("fault_latched") is False,
        "bridge_no_rejections": status.get("rejected_command_count") == 0,
        "bridge_no_watchdog_timeout": status.get(
            "moving_watchdog_timeout_count"
        )
        == 0,
        "j1_normalization_enabled": normalization.get("enabled") is True,
        "j1_normalization_joint": normalization.get("joint_name") == "J1",
        "j1_normalization_method": normalization.get("method")
        == "nearest_equivalent_to_current",
        "fresh_adjustment_counter": normalization.get(
            "accepted_adjustment_count"
        )
        == 0,
        "single_command_publisher": topology[COMMAND_TOPIC][
            "publisher_count"
        ]
        == 1,
        "single_raw_state_publisher": topology[RAW_STATE_TOPIC][
            "publisher_count"
        ]
        == 1,
        "single_controller_state_publisher": topology[
            CONTROLLER_STATE_TOPIC
        ]["publisher_count"]
        == 1,
        "single_bridge_status_publisher": topology[BRIDGE_STATUS_TOPIC][
            "publisher_count"
        ]
        == 1,
        "publisher_message_types": all(
            endpoint["topic_type"] == expected_type
            for topic, expected_type in (
                (COMMAND_TOPIC, "sensor_msgs/msg/JointState"),
                (RAW_STATE_TOPIC, "sensor_msgs/msg/JointState"),
                (
                    CONTROLLER_STATE_TOPIC,
                    "control_msgs/msg/JointTrajectoryControllerState",
                ),
                (BRIDGE_STATUS_TOPIC, "std_msgs/msg/String"),
            )
            for endpoint in topology[topic]["publishers"]
        ),
    }
    return {
        "raw_state": raw,
        "bridge_status": bridge_projection(status),
        "publisher_topology": topology,
        "action_server": ACTION_NAME,
        "gates": gates,
        "pass": all(gates.values()),
    }


def validate_final_state(node: BranchRegression) -> dict[str, Any]:
    node.spin_for(0.30)
    raw = copy.deepcopy(node.latest_raw)
    status = copy.deepcopy(node.latest_status)
    position = raw["position_rad"] if raw else None
    velocity = raw["velocity_rad_s"] if raw else None
    gates = {
        "raw_state_present": raw is not None,
        "returned_to_mechanical_zero": position is not None
        and max(abs(value) for value in position) <= FINAL_POSITION_TOLERANCE_RAD,
        "stationary_velocity": bool(velocity)
        and max(abs(value) for value in velocity) <= OTHER_JOINT_LIMIT_RAD,
        "bridge_fault_not_latched": status.get("fault_latched") is False,
        "bridge_no_rejections": status.get("rejected_command_count") == 0,
        "bridge_no_watchdog_timeout": status.get(
            "moving_watchdog_timeout_count"
        )
        == 0,
        "bridge_adjustments_observed": branch_adjustment_count(status) > 0,
        "bridge_max_adjustment_is_two_pi": abs(
            branch_max_adjustment(status) - TWO_PI
        )
        <= BRANCH_MAX_ADJUSTMENT_TOLERANCE_RAD,
    }
    return {
        "raw_state": raw,
        "bridge_status": bridge_projection(status),
        "gates": gates,
        "pass": all(gates.values()),
    }


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output",
        type=Path,
        default=Path("j1_continuous_branch_regression_v15_14.json"),
        help="machine-readable JSON output path",
    )
    parser.add_argument(
        "--ready-timeout-s", type=float, default=30.0
    )
    parser.add_argument(
        "--action-timeout-s", type=float, default=30.0
    )
    parser.add_argument(
        "--segment-duration-s", type=float, default=4.0
    )
    args = parser.parse_args(argv)
    if args.ready_timeout_s <= 0.0 or args.action_timeout_s <= 0.0:
        parser.error("timeouts must be positive")
    if args.segment_duration_s < 3.0:
        parser.error("segment duration must be at least 3 seconds")
    return args


def run(args: argparse.Namespace) -> tuple[dict[str, Any], int]:
    report: dict[str, Any] = {
        "schema": "go-m8010-arm-v15.14-j1-continuous-branch-regression/1.0",
        "generated_utc": utc_now(),
        "ros_domain_id": os.environ.get("ROS_DOMAIN_ID"),
        "contract": {
            "scope": "J1 continuous representation through standard FollowJointTrajectory",
            "action_name": ACTION_NAME,
            "joint_names": list(JOINTS),
            "segment_duration_s": args.segment_duration_s,
            "positive_branch_goal_rad": [TWO_PI, TWO_PI + 0.12],
            "negative_branch_goal_rad": [-TWO_PI + 0.12, -TWO_PI],
            "expected_physical_motion_rad": [0.0, 0.12, 0.0],
            "thresholds": {
                "branch_offset_tolerance_rad": BRANCH_OFFSET_TOLERANCE_RAD,
                "branch_max_adjustment_tolerance_rad": (
                    BRANCH_MAX_ADJUSTMENT_TOLERANCE_RAD
                ),
                "raw_j1_jump_limit_rad": RAW_JUMP_LIMIT_RAD,
                "raw_velocity_limit_rad_s": RAW_VELOCITY_LIMIT_RAD_S,
                "bridge_acceleration_limit_rad_s2": (
                    BRIDGE_ACCELERATION_LIMIT_RAD_S2
                ),
                "other_joint_limit_rad": OTHER_JOINT_LIMIT_RAD,
                "final_position_tolerance_rad": (
                    FINAL_POSITION_TOLERANCE_RAD
                ),
                "controller_wrapped_tracking_limit_rad": (
                    CONTROLLER_WRAPPED_TRACKING_LIMIT_RAD
                ),
                "minimum_dynamic_sample_count": MIN_DYNAMIC_SAMPLE_COUNT,
            },
        },
        "initial": None,
        "segments": [],
        "final": None,
        "exception": None,
        "pass": False,
    }
    node: BranchRegression | None = None
    try:
        rclpy.init()
        node = BranchRegression()
        node.wait_ready(args.ready_timeout_s)
        report["initial"] = validate_initial_state(node)
        if not report["initial"]["pass"]:
            raise RuntimeError(
                "fresh-domain zero-state precondition failed; refusing motion"
            )

        positive = node.run_goal(
            "positive_2pi_branch",
            start_j1_rad=TWO_PI,
            final_j1_rad=TWO_PI + 0.12,
            duration_s=args.segment_duration_s,
            timeout_s=args.action_timeout_s,
        )
        report["segments"].append(positive)
        analyze_segment(
            positive,
            expected_offset_rad=TWO_PI,
            expected_final_physical_j1_rad=0.12,
        )
        if not positive["pass"]:
            raise RuntimeError(
                "positive 2*pi branch segment failed; refusing second motion"
            )

        negative = node.run_goal(
            "negative_2pi_branch",
            start_j1_rad=-TWO_PI + 0.12,
            final_j1_rad=-TWO_PI,
            duration_s=args.segment_duration_s,
            timeout_s=args.action_timeout_s,
        )
        report["segments"].append(negative)
        analyze_segment(
            negative,
            expected_offset_rad=-TWO_PI,
            expected_final_physical_j1_rad=0.0,
        )
        report["final"] = validate_final_state(node)
        report["pass"] = bool(
            report["initial"]["pass"]
            and len(report["segments"]) == 2
            and all(segment["pass"] for segment in report["segments"])
            and report["final"]["pass"]
        )
    except Exception as error:
        report["exception"] = {
            "type": type(error).__name__,
            "message": str(error),
        }
        report["pass"] = False
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    report["completed_utc"] = utc_now()
    return report, 0 if report["pass"] else 1


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(argv)
    report, exit_code = run(args)
    output = args.output.expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(
            report,
            ensure_ascii=False,
            indent=2,
            sort_keys=True,
            allow_nan=False,
        )
        + "\n",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "output": str(output),
                "pass": report["pass"],
                "exception": report["exception"],
            },
            ensure_ascii=False,
            sort_keys=True,
        )
    )
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

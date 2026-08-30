"""把 ROS2 GUI 命令严格校验后转发给本机独立硬件控制进程。"""

from __future__ import annotations

import json
import hashlib
import math
import socket
import struct
import time
from collections import OrderedDict
from copy import deepcopy
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


ALLOWED_MODES = {"brake", "drag", "hold", "position"}
MODEL_COMMAND_LOWER_RAD = tuple(math.radians(value) for value in (
    -180.0, -260.0, -155.6, -129.49, -118.54, -180.0,
))
MODEL_COMMAND_UPPER_RAD = tuple(math.radians(value) for value in (
    180.0, 80.0, 184.4, 145.51, 103.26, 180.0,
))
KP_LIMITS = (1.5, 3.0, 2.0, 2.0, 1.5, 0.0)
KD_LIMITS = (0.15, 0.30, 0.15, 0.15, 0.12, 0.0)
DEFAULT_KP = (0.5, 1.0, 0.6, 0.5, 0.5, 0.0)
DEFAULT_KD = (0.05, 0.10, 0.05, 0.05, 0.05, 0.0)
REJECTION_LOG_INTERVAL_SECONDS = 5.0
COMMAND_SOURCE_MAX_AGE_NS = 250_000_000
COMMAND_SOURCE_TAKEOVER_TIMEOUT_NS = 500_000_000
COMMAND_SOURCE_ID_HEX_LENGTH = 32
MAX_TRACKED_COMMAND_SOURCES = 32
COLLISION_GUARD_RESULT_SCHEMA = "go-m8010-collision-guard-result/1.0"
COLLISION_GUARD_RESULT_TOPIC = "/whole_arm/collision_guard_result"
COLLISION_GUARD_PROOF_MAX_AGE_NS = 8_000_000_000
COLLISION_GUARD_MARGIN_DEG = 2.0
COLLISION_GUARD_MAX_STEP_DEG = 0.25
COLLISION_GUARD_BOUNDARY_TOLERANCE_DEG = 0.01
COLLISION_NONMOVING_TARGET_TOLERANCE_RAD = math.radians(0.25)
COLLISION_HARDWARE_MAX_ABS_VELOCITY_RAD_S = math.radians(0.25)
COLLISION_MARGIN_POLICY = "SINGLE_JOINT_EXTENDED_SWEEP_SUPPORT_CROSS_FINAL_LINF_V2"
COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG = 2.25
COLLISION_HOLD_TRACKING_TOLERANCE_DEG = 0.25
COLLISION_TUBE_PROBE_JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
COLLISION_TUBE_GRID_OFFSETS = (-1, 0, 1)
COLLISION_TUBE_GRID_MAX_POSE_COUNT = 729
COLLISION_TUBE_AXIS_PROBE_STEP_DEG = 0.25
COLLISION_TUBE_CROSS_SECTION_MAX_POSE_COUNT = 825
MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG = (
    (-180.0, 180.0), (-170.0, 170.0), (-170.0, 170.0),
    (-116.0, 159.0), (-70.6, 151.2), (-180.0, 180.0),
)
MAX_CACHED_COLLISION_PROOFS = 64
MAX_BOUND_COLLISION_PROOFS = 64
PRODUCTION_MODEL_SHA256 = (
    "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
)
PRODUCTION_COLLISION_CONTRACT_SHA256 = (
    "6d802909e44f238816f007ef33b57e1b57099c5522a473cd2dcc2705397af9c9"
)
PRODUCTION_KINEMATIC_GUARD_SHA256 = (
    "648b9de953a84be46cd5f95e24daf9f683f3025e73048dc068ef7bd8e5670c6a"
)
GROUND_CONTACT_POLICY = "UNSAFE_GROUND_COLLISION_NOT_SELF_COLLISION"
COLLISION_GUARD_REQUIRED_FIELDS = frozenset({
    "schema",
    "source_instance_id",
    "request_sequence",
    "source_monotonic_ns",
    "kind",
    "session_id",
    "state_instance_id",
    "moving_joint_mask",
    "start_relative_rad",
    "target_relative_rad",
    "target_sha256",
    "collision_margin_deg",
    "session_pose_sha256",
    "checked_monotonic_ns",
    "safe",
    "reason",
    "recommended_relative_rad",
    "contact_pairs",
    "model_sha256",
    "kinematic_guard_sha256",
    "collision_contract_sha256",
    "ground_contact_policy",
    "max_step_deg",
    "boundary_tolerance_deg",
    "hardware_state_sequence",
    "hardware_state_source_monotonic_ns",
    "hardware_position_rad",
    "hardware_velocity_rad_s",
    "hardware_controller_mode_by_motor",
    "hardware_state_sha256",
    "margin_policy",
    "joint_space_tube_radius_deg",
    "hold_tracking_tolerance_deg",
    "tube_probe_joint_names",
    "tube_grid_offsets",
    "tube_grid_max_pose_count",
    "tube_axis_probe_step_deg",
    "tube_cross_section_max_pose_count",
    "absolute_joint_limits_deg",
    "single_joint_path_required",
})
COLLISION_GUARD_OPTIONAL_FIELDS = frozenset({
    "step_count",
    "first_unsafe_fraction",
    "first_unsafe_path_deg",
    "target_path_length_deg",
    "margin_probe_directions",
    "margin_probe_offsets_deg",
    "path_sample_count",
    "pose_check_count",
    "tube_probe_count",
    "tube_probe_cache_hit_count",
    "moving_joint",
    "moving_joint_count",
})
J2_ACTIVE_CONTROL_BLOCKED = False
DOMAIN_JOINT_INDICES = {
    "J1": frozenset({0}),
    "J2": frozenset({1}),
    "J345": frozenset({2, 3, 4}),
    "J6": frozenset({5}),
}

KNOWN_REJECTION_REASONS = frozenset({
    "命令格式不匹配",
    "控制模式不允许",
    "旧版协议仅允许制动",
    "必须包含六个逻辑关节目标",
    "关节目标必须是有限数",
    "关节目标超出模型机械限位",
    "关节激活掩码必须是六个布尔值",
    "关节移动掩码必须是六个布尔值",
    "移动关节必须同时激活",
    "位置运动必须激活全部六个关节",
    "位置运动必须且只能选择一个移动关节",
    "位置运动缺少碰撞守卫证明",
    "碰撞守卫证明格式无效",
    "碰撞守卫证明与运动目标不匹配",
    "碰撞守卫证明与运动合同不匹配",
    "碰撞守卫证明未由路由器独立接收",
    "碰撞守卫证明已过期",
    "碰撞守卫证明不得跨激活纪元重用",
    "激活纪元必须是非负整数",
    "主动命令的激活纪元必须大于零",
    "主动命令来源实例无效",
    "主动命令来源时间戳无效或过期",
    "主动命令序号必须是正整数",
    "命令序号已重放或乱序",
    "另一GUI来源仍在控制",
    "关节增益格式不正确",
    "关节增益超过冻结上限",
    "速度或加速度必须为正的有限数",
})


def _valid_source_instance_id(value: object) -> bool:
    return bool(
        isinstance(value, str)
        and len(value) == COMMAND_SOURCE_ID_HEX_LENGTH
        and all(character in "0123456789abcdef" for character in value)
    )


def _valid_sha256(value: object) -> bool:
    return bool(
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def canonical_six_doubles_sha256(values: object) -> str:
    """Hash six binary64 targets independently of their JSON formatting."""

    vector = _finite_six(values, "碰撞守卫关节向量")
    return hashlib.sha256(struct.pack(">6d", *vector)).hexdigest()


def _finite_six(values: object, label: str) -> tuple[float, ...]:
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError(f"{label}必须包含六个数值")
    result = []
    for value in values:
        if type(value) not in {int, float}:
            raise ValueError(f"{label}必须包含六个数值")
        converted = float(value)
        if not math.isfinite(converted):
            raise ValueError(f"{label}必须包含六个有限数值")
        result.append(converted)
    return tuple(result)


def _strict_finite_number(value: object, label: str) -> float:
    if type(value) not in {int, float} or not math.isfinite(float(value)):
        raise ValueError(f"{label}无效")
    return float(value)


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


def _validated_collision_guard_result(
    value: object,
    *,
    now_ns: Optional[int],
    require_authorizing: bool,
) -> dict:
    """Validate the complete frozen guard result without trusting GUI echoes."""

    try:
        if not isinstance(value, dict):
            raise ValueError
        fields = frozenset(value)
        if not COLLISION_GUARD_REQUIRED_FIELDS.issubset(fields):
            raise ValueError
        if fields - COLLISION_GUARD_REQUIRED_FIELDS - COLLISION_GUARD_OPTIONAL_FIELDS:
            raise ValueError
        if value.get("schema") != COLLISION_GUARD_RESULT_SCHEMA:
            raise ValueError
        source = value.get("source_instance_id")
        state_source = value.get("state_instance_id")
        if not _valid_source_instance_id(source) or not _valid_source_instance_id(
            state_source
        ):
            raise ValueError
        sequence = value.get("request_sequence")
        source_ns = value.get("source_monotonic_ns")
        checked_ns = value.get("checked_monotonic_ns")
        if (
            type(sequence) is not int
            or not 1 <= sequence <= (1 << 63) - 1
            or type(source_ns) is not int
            or source_ns <= 0
            or type(checked_ns) is not int
            or checked_ns < source_ns
            or (now_ns is not None and checked_ns > now_ns)
        ):
            raise ValueError
        kind = value.get("kind")
        if kind not in {"preview", "pose_preview", "execute"}:
            raise ValueError
        session_id = value.get("session_id")
        if not isinstance(session_id, str) or not 1 <= len(session_id) <= 512:
            raise ValueError
        start = _finite_six(value.get("start_relative_rad"), "碰撞守卫起点")
        target = _finite_six(value.get("target_relative_rad"), "碰撞守卫目标")
        moving_mask = value.get("moving_joint_mask")
        if (
            not isinstance(moving_mask, list)
            or len(moving_mask) != 6
            or not all(type(item) is bool for item in moving_mask)
            or sum(moving_mask) < 1
            or (kind != "pose_preview" and sum(moving_mask) != 1)
        ):
            raise ValueError
        target_sha256 = value.get("target_sha256")
        if (
            not _valid_sha256(target_sha256)
            or target_sha256 != canonical_six_doubles_sha256(target)
            or not _valid_sha256(value.get("session_pose_sha256"))
        ):
            raise ValueError
        margin = _strict_finite_number(
            value.get("collision_margin_deg"), "碰撞守卫安全余量"
        )
        max_step = _strict_finite_number(
            value.get("max_step_deg"), "碰撞守卫扫描步长"
        )
        boundary_tolerance = _strict_finite_number(
            value.get("boundary_tolerance_deg"), "碰撞守卫边界精度"
        )
        if (
            margin != COLLISION_GUARD_MARGIN_DEG
            or max_step != COLLISION_GUARD_MAX_STEP_DEG
            or boundary_tolerance != COLLISION_GUARD_BOUNDARY_TOLERANCE_DEG
            or value.get("model_sha256") != PRODUCTION_MODEL_SHA256
            or value.get("kinematic_guard_sha256")
            != PRODUCTION_KINEMATIC_GUARD_SHA256
            or value.get("collision_contract_sha256")
            != PRODUCTION_COLLISION_CONTRACT_SHA256
            or value.get("ground_contact_policy") != GROUND_CONTACT_POLICY
            or value.get("margin_policy") != COLLISION_MARGIN_POLICY
            or value.get("joint_space_tube_radius_deg")
            != COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG
            or value.get("hold_tracking_tolerance_deg")
            != COLLISION_HOLD_TRACKING_TOLERANCE_DEG
            or value.get("tube_probe_joint_names")
            != list(COLLISION_TUBE_PROBE_JOINT_NAMES)
            or value.get("tube_grid_offsets")
            != list(COLLISION_TUBE_GRID_OFFSETS)
            or value.get("tube_grid_max_pose_count")
            != COLLISION_TUBE_GRID_MAX_POSE_COUNT
            or value.get("tube_axis_probe_step_deg")
            != COLLISION_TUBE_AXIS_PROBE_STEP_DEG
            or value.get("tube_cross_section_max_pose_count")
            != COLLISION_TUBE_CROSS_SECTION_MAX_POSE_COUNT
            or value.get("absolute_joint_limits_deg")
            != [list(bounds) for bounds in PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG]
            or value.get("single_joint_path_required") is not True
        ):
            raise ValueError
        hardware_sequence = value.get("hardware_state_sequence")
        hardware_source_ns = value.get("hardware_state_source_monotonic_ns")
        hardware_position = _finite_six(
            value.get("hardware_position_rad"), "碰撞守卫硬件位置"
        )
        hardware_velocity = _finite_six(
            value.get("hardware_velocity_rad_s"), "碰撞守卫硬件速度"
        )
        hardware_modes = value.get("hardware_controller_mode_by_motor")
        hardware_sha256 = value.get("hardware_state_sha256")
        if (
            type(hardware_sequence) is not int
            or hardware_sequence <= 0
            or type(hardware_source_ns) is not int
            or hardware_source_ns <= 0
            or hardware_source_ns > checked_ns
            or not isinstance(hardware_modes, dict)
            or set(hardware_modes) != set(MOTOR_NAMES)
            or any(hardware_modes[name] != "hold" for name in MOTOR_NAMES)
            or any(
                abs(velocity) >
                COLLISION_HARDWARE_MAX_ABS_VELOCITY_RAD_S + 1.0e-12
                for velocity in hardware_velocity
            )
            or any(
                abs(measured - requested) >
                COLLISION_NONMOVING_TARGET_TOLERANCE_RAD + 1.0e-12
                for measured, requested in zip(hardware_position, start)
            )
            or not _valid_sha256(hardware_sha256)
            or hardware_sha256 != canonical_collision_hardware_state_sha256({
                "schema": "go-m8010-hardware-state/1.1",
                "session_id": session_id,
                "state_instance_id": state_source,
                "sequence": hardware_sequence,
                "source_monotonic_ns": hardware_source_ns,
                "position_rad": hardware_position,
                "velocity_rad_s": hardware_velocity,
                "controller_mode_by_motor": hardware_modes,
            })
        ):
            raise ValueError
        safe = value.get("safe")
        reason = value.get("reason")
        if type(safe) is not bool or not isinstance(reason, str) or not reason:
            raise ValueError
        contacts = value.get("contact_pairs")
        if (
            not isinstance(contacts, list)
            or any(
                not isinstance(pair, list)
                or len(pair) != 2
                or not all(isinstance(name, str) and name for name in pair)
                for pair in contacts
            )
        ):
            raise ValueError
        recommendation = value.get("recommended_relative_rad")
        if recommendation is not None:
            recommendation = _finite_six(recommendation, "碰撞守卫建议目标")
        if safe and (
            reason != "clear"
            or contacts != []
            or recommendation is None
            or tuple(recommendation) != tuple(target)
        ):
            raise ValueError
        count_fields = {
            "step_count", "path_sample_count", "pose_check_count",
            "tube_probe_count", "tube_probe_cache_hit_count",
            "moving_joint_count",
        }
        for field in count_fields:
            if field in value and (
                type(value[field]) is not int or value[field] < 0
            ):
                raise ValueError
        for field in {
            "first_unsafe_fraction", "first_unsafe_path_deg",
            "target_path_length_deg",
        }:
            if field in value:
                number = _strict_finite_number(value[field], field)
                if field == "first_unsafe_fraction" and not 0.0 <= number <= 1.0:
                    raise ValueError
                if field != "first_unsafe_fraction" and number < 0.0:
                    raise ValueError
        if "margin_probe_directions" in value and (
            not isinstance(value["margin_probe_directions"], list)
            or len(value["margin_probe_directions"]) != 6
            or not all(
                type(item) is int and item in {-1, 0, 1}
                for item in value["margin_probe_directions"]
            )
        ):
            raise ValueError
        if "margin_probe_offsets_deg" in value:
            _finite_six(value["margin_probe_offsets_deg"], "碰撞余量探针偏移")
        if "moving_joint" in value and value["moving_joint"] not in {
            None, *COLLISION_TUBE_PROBE_JOINT_NAMES,
        }:
            raise ValueError
        if safe and kind != "pose_preview" and (
            any(field not in value for field in count_fields - {"moving_joint_count"})
            or value.get("moving_joint_count") != 1
            or value.get("moving_joint")
            != COLLISION_TUBE_PROBE_JOINT_NAMES[moving_mask.index(True)]
        ):
            raise ValueError
        if safe and kind == "pose_preview" and (
            any(field not in value for field in count_fields - {"moving_joint_count"})
            or value.get("moving_joint_count") != sum(moving_mask)
            or value.get("moving_joint")
            != (
                COLLISION_TUBE_PROBE_JOINT_NAMES[moving_mask.index(True)]
                if sum(moving_mask) == 1 else None
            )
        ):
            raise ValueError
        if require_authorizing and (not safe or kind != "execute"):
            raise ValueError
    except (KeyError, TypeError, ValueError, OverflowError):
        raise ValueError("碰撞守卫证明格式无效") from None
    return deepcopy(value)


class CollisionGuardProofGate:
    """One-shot, independently observed collision-proof authorization gate."""

    def __init__(
        self,
        *,
        maximum_cached_proofs: int = MAX_CACHED_COLLISION_PROOFS,
        maximum_bound_proofs: int = MAX_BOUND_COLLISION_PROOFS,
        maximum_sources: int = MAX_TRACKED_COMMAND_SOURCES,
        proof_max_age_ns: int = COLLISION_GUARD_PROOF_MAX_AGE_NS,
    ) -> None:
        if min(
            maximum_cached_proofs,
            maximum_bound_proofs,
            maximum_sources,
            proof_max_age_ns,
        ) <= 0:
            raise ValueError("collision proof gate bounds must be positive")
        self.maximum_cached_proofs = maximum_cached_proofs
        self.maximum_bound_proofs = maximum_bound_proofs
        self.maximum_sources = maximum_sources
        self.proof_max_age_ns = proof_max_age_ns
        self._last_result_by_source: OrderedDict[str, tuple[int, int]] = OrderedDict()
        self._cached: OrderedDict[tuple[str, int], dict] = OrderedDict()
        self._bindings: OrderedDict[tuple[str, int, str], dict] = OrderedDict()
        self._proof_to_binding: dict[tuple[str, int], tuple[str, int, str]] = {}

    @property
    def cached_proof_count(self) -> int:
        return len(self._cached)

    @property
    def bound_proof_count(self) -> int:
        return len(self._bindings)

    def observe_result(self, value: object, now_ns: Optional[int] = None) -> bool:
        """Observe the guard topic; only a fresh safe execute result is cached."""

        observed_ns = time.monotonic_ns() if now_ns is None else now_ns
        try:
            result = _validated_collision_guard_result(
                value, now_ns=observed_ns, require_authorizing=False
            )
        except ValueError:
            return False
        source = result["source_instance_id"]
        current = (result["request_sequence"], result["source_monotonic_ns"])
        previous = self._last_result_by_source.get(source)
        if previous is not None and (
            current[0] <= previous[0] or current[1] <= previous[1]
        ):
            return False
        if source not in self._last_result_by_source and (
            len(self._last_result_by_source) >= self.maximum_sources
        ):
            self._last_result_by_source.popitem(last=False)
        self._last_result_by_source[source] = current
        self._last_result_by_source.move_to_end(source)
        self._expire_cached(observed_ns)
        # A later request from the same GUI supersedes every still-unconsumed
        # proof from that source, including when the later result is a preview
        # or an unsafe result.  Already-bound active epochs remain unaffected.
        for proof_key in tuple(self._cached):
            if proof_key[0] == source and proof_key[1] < current[0]:
                self._cached.pop(proof_key, None)
        if not result["safe"] or result["kind"] != "execute":
            return True
        if (
            observed_ns - result["checked_monotonic_ns"] > self.proof_max_age_ns
            or observed_ns - result["source_monotonic_ns"] > self.proof_max_age_ns
        ):
            return False
        proof_key = (source, result["request_sequence"])
        self._cached[proof_key] = result
        self._cached.move_to_end(proof_key)
        while len(self._cached) > self.maximum_cached_proofs:
            self._cached.popitem(last=False)
        return True

    def authorize(self, command: dict, now_ns: Optional[int] = None) -> None:
        """Authorize a moving POSITION or leave non-moving modes unchanged."""

        if command.get("mode") != "position":
            return
        if command.get("active_joint_mask") != [True] * 6:
            raise ValueError("位置运动必须激活全部六个关节")
        moving_values = command.get("moving_joint_mask")
        if (
            not isinstance(moving_values, list)
            or len(moving_values) != 6
            or not all(type(value) is bool for value in moving_values)
            or sum(moving_values) != 1
        ):
            raise ValueError("位置运动必须且只能选择一个移动关节")
        authorized_ns = time.monotonic_ns() if now_ns is None else now_ns
        proof = command.get("collision_guard_proof")
        if proof is None:
            raise ValueError("位置运动缺少碰撞守卫证明")
        proof = _validated_collision_guard_result(
            proof, now_ns=authorized_ns, require_authorizing=True
        )
        target_sha256 = canonical_six_doubles_sha256(command["targets_rad"])
        moving_mask = tuple(command["moving_joint_mask"])
        if (
            proof["source_instance_id"] != command["source_instance_id"]
            or proof["target_sha256"] != target_sha256
        ):
            raise ValueError("碰撞守卫证明与运动目标不匹配")
        if proof["moving_joint_mask"] != list(moving_mask):
            raise ValueError("碰撞守卫证明与运动合同不匹配")
        if any(
            not moving
            and abs(target - start) > COLLISION_NONMOVING_TARGET_TOLERANCE_RAD
            for moving, start, target in zip(
                moving_mask,
                proof["start_relative_rad"],
                command["targets_rad"],
            )
        ):
            raise ValueError("碰撞守卫证明与运动合同不匹配")
        command_contract = (
            moving_mask,
            command["maximum_velocity_rad_s"],
            command["maximum_acceleration_rad_s2"],
        )
        proof_key = (proof["source_instance_id"], proof["request_sequence"])
        binding = (
            command["source_instance_id"],
            command["activation_epoch"],
            target_sha256,
        )
        existing = self._bindings.get(binding)
        if existing is not None:
            if (
                existing["proof"] != proof
                or existing["command_contract"] != command_contract
            ):
                raise ValueError("碰撞守卫证明与运动合同不匹配")
            self._bindings.move_to_end(binding)
            return
        claimed_binding = self._proof_to_binding.get(proof_key)
        if claimed_binding is not None and claimed_binding != binding:
            raise ValueError("碰撞守卫证明不得跨激活纪元重用")
        cached = self._cached.get(proof_key)
        if cached is None or cached != proof:
            self._expire_cached(authorized_ns)
            raise ValueError("碰撞守卫证明未由路由器独立接收")
        if (
            authorized_ns - proof["checked_monotonic_ns"] > self.proof_max_age_ns
            or authorized_ns - proof["source_monotonic_ns"] > self.proof_max_age_ns
        ):
            self._cached.pop(proof_key, None)
            raise ValueError("碰撞守卫证明已过期")
        # Consume the independently received proof exactly once, then retain a
        # bounded binding for high-rate repeats of this one activation epoch.
        self._cached.pop(proof_key, None)
        self._bindings[binding] = {
            "proof": proof,
            "command_contract": command_contract,
        }
        self._proof_to_binding[proof_key] = binding
        while len(self._bindings) > self.maximum_bound_proofs:
            old_binding, old_record = self._bindings.popitem(last=False)
            old_proof = old_record["proof"]
            old_key = (
                old_proof["source_instance_id"], old_proof["request_sequence"]
            )
            if self._proof_to_binding.get(old_key) == old_binding:
                self._proof_to_binding.pop(old_key, None)

    def _expire_cached(self, now_ns: int) -> None:
        expired = [
            key
            for key, proof in self._cached.items()
            if now_ns < proof["checked_monotonic_ns"]
            or now_ns - proof["checked_monotonic_ns"] > self.proof_max_age_ns
        ]
        for key in expired:
            self._cached.pop(key, None)


def _validated_command_source(
    value: dict, now_ns: int
) -> tuple[str, int, int]:
    source_instance_id = value.get("source_instance_id")
    if not _valid_source_instance_id(source_instance_id):
        raise ValueError("主动命令来源实例无效")
    source_monotonic_ns = value.get("source_monotonic_ns")
    if (
        type(source_monotonic_ns) is not int
        or source_monotonic_ns <= 0
        or source_monotonic_ns > now_ns
        or now_ns - source_monotonic_ns > COMMAND_SOURCE_MAX_AGE_NS
    ):
        raise ValueError("主动命令来源时间戳无效或过期")
    sequence = value.get("sequence")
    if (
        type(sequence) is not int
        or not 1 <= sequence <= (1 << 63) - 1
    ):
        raise ValueError("主动命令序号必须是正整数")
    return source_instance_id, source_monotonic_ns, sequence


class CommandReplayGuard:
    """Reject duplicate/out-of-order commands without weakening emergency BRAKE."""

    def __init__(self, maximum_sources: int = MAX_TRACKED_COMMAND_SOURCES) -> None:
        self.maximum_sources = maximum_sources
        self._last_by_source: dict[str, tuple[int, int]] = {}
        self._active_source: Optional[str] = None
        self._active_source_last_accepted_ns: Optional[int] = None

    def check(self, command: dict, now_ns: Optional[int] = None) -> None:
        """Check replay/source ownership without advancing accepted state."""

        if command["mode"] == "brake":
            return
        checked_at_ns = time.monotonic_ns() if now_ns is None else now_ns
        source = command["source_instance_id"]
        if (
            self._active_source is not None
            and source != self._active_source
            and self._active_source_last_accepted_ns is not None
            and checked_at_ns - self._active_source_last_accepted_ns
            <= COMMAND_SOURCE_TAKEOVER_TIMEOUT_NS
        ):
            raise ValueError("另一GUI来源仍在控制")
        current = (command["sequence"], command["source_monotonic_ns"])
        previous = self._last_by_source.get(source)
        if previous is not None and (
            current[0] <= previous[0] or current[1] <= previous[1]
        ):
            raise ValueError("命令序号已重放或乱序")

    def commit(self, command: dict, now_ns: Optional[int] = None) -> None:
        """Advance state after every other authorization gate has succeeded."""

        if command["mode"] == "brake":
            return
        checked_at_ns = time.monotonic_ns() if now_ns is None else now_ns
        source = command["source_instance_id"]
        current = (command["sequence"], command["source_monotonic_ns"])
        if source not in self._last_by_source and (
            len(self._last_by_source) >= self.maximum_sources
        ):
            oldest_source = next(iter(self._last_by_source))
            self._last_by_source.pop(oldest_source)
        self._last_by_source[source] = current
        self._active_source = source
        self._active_source_last_accepted_ns = checked_at_ns

    def accept(self, command: dict, now_ns: Optional[int] = None) -> None:
        checked_at_ns = time.monotonic_ns() if now_ns is None else now_ns
        self.check(command, now_ns=checked_at_ns)
        self.commit(command, now_ns=checked_at_ns)


def _normalize_brake_fields(command: dict) -> dict:
    """Remove every position or gain authority from a BRAKE payload."""

    normalized = dict(command)
    normalized["mode"] = "brake"
    normalized["targets_rad"] = [0.0] * 6
    normalized["active_joint_mask"] = [False] * 6
    normalized["moving_joint_mask"] = [False] * 6
    normalized["kp"] = [0.0] * 6
    normalized["kd"] = [0.0] * 6
    return normalized


def _normalize_drag_fields(command: dict) -> dict:
    """Deliver an explicit, confirmed global torque-release request.

    Like BRAKE, DRAG must not be blocked by irrelevant position/gain fields.
    Keeping every domain selected lets workers report DRAG distinctly while
    they command mode 0 and fence any queued active command.
    """

    normalized = dict(command)
    normalized["mode"] = "drag"
    normalized["targets_rad"] = [0.0] * 6
    normalized["active_joint_mask"] = [True] * 6
    normalized["moving_joint_mask"] = [False] * 6
    normalized["kp"] = [0.0] * 6
    normalized["kd"] = [0.0] * 6
    return normalized


def rejection_reason(error: Exception) -> str:
    """Return a stable, bounded-cardinality key for rejection aggregation."""

    if isinstance(error, json.JSONDecodeError):
        return "JSON格式无效"
    message = str(error).strip()
    if message in KNOWN_REJECTION_REASONS:
        return message
    if isinstance(error, KeyError):
        return "命令缺少必需字段"
    if isinstance(error, (TypeError, ValueError, OverflowError)):
        return "命令字段类型或数值无效"
    return type(error).__name__


class RejectionTracker:
    """Count every rejection while rate-limiting logs independently by reason."""

    def __init__(self, log_interval_seconds: float = REJECTION_LOG_INTERVAL_SECONDS) -> None:
        self.log_interval_ns = int(log_interval_seconds * 1.0e9)
        self.total = 0
        self.by_reason: dict[str, int] = {}
        self.last_rejection_ns: Optional[int] = None
        self._last_logged_ns: dict[str, int] = {}
        self._pending_by_reason: dict[str, int] = {}

    def record(self, error: Exception, now_ns: Optional[int] = None) -> list[dict]:
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        reason = rejection_reason(error)
        self.last_rejection_ns = now_ns
        self.total += 1
        self.by_reason[reason] = self.by_reason.get(reason, 0) + 1
        if reason not in self._last_logged_ns:
            self._last_logged_ns[reason] = now_ns
            self._pending_by_reason[reason] = 0
            return [self._report(reason, 0, initial=True)]
        self._pending_by_reason[reason] += 1
        return self.flush(now_ns=now_ns, reasons=(reason,))

    def flush(
        self, now_ns: Optional[int] = None, reasons: Optional[tuple[str, ...]] = None
    ) -> list[dict]:
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        reports = []
        selected = tuple(self.by_reason) if reasons is None else reasons
        for reason in selected:
            pending = self._pending_by_reason.get(reason, 0)
            last_logged_ns = self._last_logged_ns.get(reason)
            if (
                pending == 0
                or last_logged_ns is None
                or now_ns - last_logged_ns < self.log_interval_ns
            ):
                continue
            reports.append(self._report(reason, pending, initial=False))
            self._pending_by_reason[reason] = 0
            self._last_logged_ns[reason] = now_ns
        return reports

    def pending_by_reason(self) -> dict[str, int]:
        return {
            reason: count
            for reason, count in self._pending_by_reason.items()
            if count > 0
        }

    def _report(self, reason: str, suppressed: int, *, initial: bool) -> dict:
        return {
            "reason": reason,
            "reason_count": self.by_reason[reason],
            "rejected_total": self.total,
            "suppressed_since_last": suppressed,
            "initial": initial,
        }


def validate_command(
    text: str, now_ns: Optional[int] = None
) -> tuple[dict, bytes]:
    checked_at_ns = time.monotonic_ns() if now_ns is None else now_ns
    value = json.loads(text)
    if not isinstance(value, dict):
        raise ValueError("命令字段类型或数值无效")
    schema = value.get("schema")
    if schema not in {
        "go-m8010-gui-command/1.0",
        "go-m8010-gui-command/1.1",
        "go-m8010-gui-command/1.2",
    }:
        raise ValueError("命令格式不匹配")
    mode = value.get("mode")
    if mode not in ALLOWED_MODES:
        raise ValueError("控制模式不允许")
    if schema != "go-m8010-gui-command/1.2" and mode != "brake":
        raise ValueError("旧版协议仅允许制动")
    if mode == "brake":
        source_instance_id = "emergency-brake"
        source_monotonic_ns = checked_at_ns
        sequence = 0
    else:
        (
            source_instance_id,
            source_monotonic_ns,
            sequence,
        ) = _validated_command_source(value, checked_at_ns)
    if mode in {"brake", "drag"}:
        targets = [0.0] * 6
        active_joint_mask = [mode == "drag"] * 6
        moving_joint_mask = [False] * 6
        kp = [0.0] * 6
        kd = [0.0] * 6
    else:
        targets = value.get("targets_rad")
        if (
            not isinstance(targets, list)
            or len(targets) != 6
            or not all(type(item) in {int, float} for item in targets)
        ):
            raise ValueError("必须包含六个逻辑关节目标")
        targets = [float(item) for item in targets]
        if not all(math.isfinite(item) for item in targets):
            raise ValueError("关节目标必须是有限数")
        active_joint_mask = value.get("active_joint_mask")
        if (
            not isinstance(active_joint_mask, list)
            or len(active_joint_mask) != 6
            or not all(type(item) is bool for item in active_joint_mask)
        ):
            raise ValueError("关节激活掩码必须是六个布尔值")
        supplied_moving_mask = value.get("moving_joint_mask")
        if "moving_joint_mask" not in value:
            moving_joint_mask = (
                list(active_joint_mask) if mode == "position" else [False] * 6
            )
        elif (
            not isinstance(supplied_moving_mask, list)
            or len(supplied_moving_mask) != 6
            or not all(type(item) is bool for item in supplied_moving_mask)
        ):
            raise ValueError("关节移动掩码必须是六个布尔值")
        else:
            moving_joint_mask = list(supplied_moving_mask)
        if mode == "hold":
            moving_joint_mask = [False] * 6
        if any(
            moving and not active
            for moving, active in zip(moving_joint_mask, active_joint_mask)
        ):
            raise ValueError("移动关节必须同时激活")
        if mode == "position" and active_joint_mask != [True] * 6:
            raise ValueError("位置运动必须激活全部六个关节")
        if mode == "position" and sum(moving_joint_mask) != 1:
            raise ValueError("位置运动必须且只能选择一个移动关节")
        # POSITION 与 HOLD 共用当前竖直会话锚点下的冻结 3D
        # 模型限位。路由器不再把运动目标收窄到统一 ±10°/J2 ±5°。
        for index, (target, active) in enumerate(zip(
            targets, active_joint_mask
        )):
            if not active:
                continue
            if not (
                MODEL_COMMAND_LOWER_RAD[index] - 1e-12
                <= target
                <= MODEL_COMMAND_UPPER_RAD[index] + 1e-12
            ):
                raise ValueError("关节目标超出模型机械限位")
        raw_kp = value.get("kp", DEFAULT_KP)
        raw_kd = value.get("kd", DEFAULT_KD)
        if (
            not isinstance(raw_kp, (list, tuple))
            or not isinstance(raw_kd, (list, tuple))
            or len(raw_kp) != 6
            or len(raw_kd) != 6
            or not all(
                type(item) in {int, float} for item in (*raw_kp, *raw_kd)
            )
        ):
            raise ValueError("关节增益格式不正确")
        kp = [float(item) for item in raw_kp]
        kd = [float(item) for item in raw_kd]
        if (
            len(kp) != 6
            or len(kd) != 6
            or not all(math.isfinite(item) and item >= 0.0 for item in kp + kd)
        ):
            raise ValueError("关节增益格式不正确")
        if any(item > limit + 1e-12 for item, limit in zip(kp, KP_LIMITS)) or any(
            item > limit + 1e-12 for item, limit in zip(kd, KD_LIMITS)
        ):
            raise ValueError("关节增益超过冻结上限")
    collision_guard_proof = None
    if mode == "position":
        if "collision_guard_proof" not in value:
            raise ValueError("位置运动缺少碰撞守卫证明")
        collision_guard_proof = _validated_collision_guard_result(
            value["collision_guard_proof"],
            now_ns=None,
            require_authorizing=True,
        )
    elif mode == "hold" and "collision_guard_proof" in value:
        collision_guard_proof = _validated_collision_guard_result(
            value["collision_guard_proof"],
            now_ns=None,
            require_authorizing=True,
        )
    if mode in {"brake", "drag"}:
        # Once JSON/schema/mode identify an emergency BRAKE request, no
        # motion-only metadata is allowed to prevent its fail-closed delivery.
        activation_epoch = 0
        maximum_velocity = math.radians(5.0)
        maximum_acceleration = math.radians(20.0)
        if mode == "brake":
            sequence = 0
    else:
        activation_epoch = value.get("activation_epoch", 0)
        if (
            schema == "go-m8010-gui-command/1.2"
            and (
                type(activation_epoch) is not int
                or not 0 <= activation_epoch <= (1 << 63) - 1
            )
        ):
            raise ValueError("激活纪元必须是非负整数")
        if schema != "go-m8010-gui-command/1.2":
            activation_epoch = 0
        if any(active_joint_mask) and activation_epoch == 0:
            raise ValueError("主动命令的激活纪元必须大于零")
        maximum_velocity = value.get(
            "maximum_velocity_rad_s", math.radians(5.0)
        )
        maximum_acceleration = value.get(
            "maximum_acceleration_rad_s2", math.radians(20.0)
        )
        if (
            type(maximum_velocity) not in {int, float}
            or type(maximum_acceleration) not in {int, float}
        ):
            raise ValueError("速度或加速度必须为正的有限数")
        maximum_velocity = float(maximum_velocity)
        maximum_acceleration = float(maximum_acceleration)
        if (not math.isfinite(maximum_velocity) or maximum_velocity <= 0.0 or
                not math.isfinite(maximum_acceleration) or maximum_acceleration <= 0.0):
            raise ValueError("速度或加速度必须为正的有限数")
    normalized = {
        "schema": "go-m8010-gui-command/1.2",
        "sequence": sequence,
        "source_instance_id": source_instance_id,
        "source_monotonic_ns": source_monotonic_ns,
        "mode": mode,
        "targets_rad": targets,
        "active_joint_mask": list(active_joint_mask),
        "moving_joint_mask": list(moving_joint_mask),
        "activation_epoch": activation_epoch,
        "maximum_velocity_rad_s": min(maximum_velocity, math.radians(5.0)),
        "maximum_acceleration_rad_s2": min(maximum_acceleration, math.radians(20.0)),
        "kp": kp,
        "kd": kd,
    }
    if collision_guard_proof is not None:
        normalized["collision_guard_proof"] = collision_guard_proof
    if mode == "brake":
        normalized = _normalize_brake_fields(normalized)
    elif mode == "drag":
        normalized = _normalize_drag_fields(normalized)
    return normalized, json.dumps(normalized, separators=(",", ":")).encode("utf-8")


def payload_for_domain(normalized: dict, domain: str) -> bytes:
    """Forward only commands that select a joint owned by this fault domain."""

    if domain not in DOMAIN_JOINT_INDICES:
        raise ValueError("未知硬件故障域")
    # Collision proof is consumed only at this ROS trust boundary.  Legacy
    # hardware workers continue receiving the frozen command/1.2 schema.
    worker_command = {
        key: value
        for key, value in normalized.items()
        if key != "collision_guard_proof"
    }
    if worker_command["mode"] == "brake":
        domain_command = _normalize_brake_fields(worker_command)
        return json.dumps(domain_command, separators=(",", ":")).encode("utf-8")
    domain_command = worker_command
    active_mode = worker_command["mode"] in {"drag", "hold", "position"}
    domain_selected = any(
        worker_command["active_joint_mask"][index]
        for index in DOMAIN_JOINT_INDICES[domain]
    )
    if active_mode and not domain_selected:
        domain_command = _normalize_brake_fields(worker_command)
    elif worker_command["mode"] == "position" and not any(
        worker_command["moving_joint_mask"][index]
        for index in DOMAIN_JOINT_INDICES[domain]
    ):
        domain_command = dict(worker_command)
        domain_command["mode"] = "hold"
        domain_command["moving_joint_mask"] = [False] * 6
    return json.dumps(domain_command, separators=(",", ":")).encode("utf-8")


class CommandRouter(Node):
    def __init__(self) -> None:
        super().__init__("arm_gui_command_router")
        self.declare_parameter("go_udp_port", 15310)
        self.declare_parameter("j2_udp_port", 15312)
        self.declare_parameter("j345_udp_port", 15313)
        self.declare_parameter("j6_udp_port", 15311)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.destinations = [
            ("J1", ("127.0.0.1", int(self.get_parameter("go_udp_port").value))),
            ("J2", ("127.0.0.1", int(self.get_parameter("j2_udp_port").value))),
            ("J345", ("127.0.0.1", int(self.get_parameter("j345_udp_port").value))),
            ("J6", ("127.0.0.1", int(self.get_parameter("j6_udp_port").value))),
        ]
        self.publisher = self.create_publisher(String, "/whole_arm/control_status", 10)
        self.subscription = self.create_subscription(
            String, "/whole_arm/gui_command", self.on_command, 10
        )
        self.collision_guard_subscription = self.create_subscription(
            String,
            COLLISION_GUARD_RESULT_TOPIC,
            self.on_collision_guard_result,
            10,
        )
        self.last_command: Optional[dict] = None
        self.replay_guard = CommandReplayGuard()
        self.collision_guard_gate = CollisionGuardProofGate()
        self.rejected = 0
        self.rejection_tracker = RejectionTracker()
        self.timer = self.create_timer(0.5, self.publish_status)
        self.get_logger().info("GUI命令路由已启动，仅允许本机UDP目标")

    def on_collision_guard_result(self, message: String) -> None:
        """Independently observe proofs; malformed/replayed results stay inert."""

        try:
            value = json.loads(message.data)
        except (json.JSONDecodeError, TypeError):
            return
        self.collision_guard_gate.observe_result(value, now_ns=time.monotonic_ns())

    def on_command(self, message: String) -> None:
        try:
            now_ns = time.monotonic_ns()
            normalized, _payload = validate_command(
                message.data, now_ns=now_ns
            )
            self.replay_guard.check(normalized, now_ns=now_ns)
            self.collision_guard_gate.authorize(normalized, now_ns=now_ns)
            self.replay_guard.commit(normalized, now_ns=now_ns)
            for domain, destination in self.destinations:
                self.socket.sendto(payload_for_domain(normalized, domain), destination)
            self.last_command = normalized
        except Exception as exc:
            reports = self.rejection_tracker.record(exc)
            self.rejected = self.rejection_tracker.total
            self._log_rejection_reports(reports)

    def _log_rejection_reports(self, reports: list[dict]) -> None:
        for report in reports:
            if report["initial"]:
                self.get_logger().warning(
                    f"拒绝GUI命令：{report['reason']}; "
                    f"reason_count={report['reason_count']}; "
                    f"rejected_total={report['rejected_total']}"
                )
            else:
                self.get_logger().warning(
                    f"拒绝GUI命令汇总：reason={report['reason']}; "
                    f"suppressed={report['suppressed_since_last']}; "
                    f"reason_count={report['reason_count']}; "
                    f"rejected_total={report['rejected_total']}"
                )

    def publish_status(self) -> None:
        self._log_rejection_reports(self.rejection_tracker.flush())
        now_ns = time.monotonic_ns()
        age_ms = None
        if self.last_command is not None:
            age_ms = (now_ns - self.last_command["source_monotonic_ns"]) / 1.0e6
        last_rejection_age_ms = None
        if self.rejection_tracker.last_rejection_ns is not None:
            last_rejection_age_ms = (
                now_ns - self.rejection_tracker.last_rejection_ns
            ) / 1.0e6
        self.publisher.publish(String(data=json.dumps({
            "schema": "go-m8010-command-router-status/1.0",
            "received": self.last_command is not None,
            "last_mode": None if self.last_command is None else self.last_command["mode"],
            "j2_active_control_blocked": J2_ACTIVE_CONTROL_BLOCKED,
            "j2_forwarded_mode": None if self.last_command is None else
                json.loads(payload_for_domain(self.last_command, "J2"))["mode"],
            "last_active_joint_mask": None if self.last_command is None else
                self.last_command["active_joint_mask"],
            "last_moving_joint_mask": None if self.last_command is None else
                self.last_command["moving_joint_mask"],
            "last_activation_epoch": None if self.last_command is None else
                self.last_command["activation_epoch"],
            "last_command_age_ms": age_ms,
            "rejected_commands": self.rejected,
            "last_rejection_age_ms": last_rejection_age_ms,
            "rejected_commands_by_reason": dict(sorted(
                self.rejection_tracker.by_reason.items()
            )),
            "suppressed_rejection_logs_pending": dict(sorted(
                self.rejection_tracker.pending_by_reason().items()
            )),
        }, ensure_ascii=False)))

    def destroy_node(self) -> bool:
        self.socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[CommandRouter] = None
    try:
        node = CommandRouter()
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

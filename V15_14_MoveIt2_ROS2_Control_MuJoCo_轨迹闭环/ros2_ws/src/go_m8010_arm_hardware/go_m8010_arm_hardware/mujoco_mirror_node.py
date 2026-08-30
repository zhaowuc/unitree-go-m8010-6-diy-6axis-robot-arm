"""Direct-qpos MuJoCo mirror for session-relative measured JointState data."""

from __future__ import annotations

import csv
import hashlib
import importlib.util
import itertools
import json
import math
import re
import signal
import statistics
import struct
import threading
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from .state_model import JOINT_NAMES, MOTOR_NAMES


MUJOCO_JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG = (
    (-180.0, 180.0),
    (-170.0, 170.0),
    (-170.0, 170.0),
    (-116.0, 159.0),
    (-70.6, 151.2),
    (-180.0, 180.0),
)
PRODUCTION_MODEL_SHA256 = "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
PRODUCTION_COLLISION_CONTRACT_SHA256 = (
    "6d802909e44f238816f007ef33b57e1b57099c5522a473cd2dcc2705397af9c9"
)
PRODUCTION_KINEMATIC_GUARD_SHA256 = (
    "648b9de953a84be46cd5f95e24daf9f683f3025e73048dc068ef7bd8e5670c6a"
)
GUI_TARGET_WARNING_INTERVAL_S = 5.0
COLLISION_GUARD_REQUEST_SCHEMA = "go-m8010-collision-guard-request/1.0"
COLLISION_GUARD_RESULT_SCHEMA = "go-m8010-collision-guard-result/1.0"
COLLISION_GUARD_REQUEST_TOPIC = "/whole_arm/collision_guard_request"
COLLISION_GUARD_RESULT_TOPIC = "/whole_arm/collision_guard_result"
COLLISION_GUARD_REQUEST_MAX_AGE_NS = 2_000_000_000
HARDWARE_STATE_SOURCE_MAX_AGE_NS = 250_000_000
HARDWARE_STATE_SOURCE_TAKEOVER_NS = 500_000_000
COLLISION_START_MATCH_TOLERANCE_RAD = math.radians(0.25)
COLLISION_HARDWARE_MAX_ABS_VELOCITY_RAD_S = math.radians(0.25)
COLLISION_GUARD_MAX_STEP_DEG = 0.25
COLLISION_BOUNDARY_TOLERANCE_DEG = 0.01
COLLISION_MARGIN_DEG = 2.0
COLLISION_HOLD_TRACKING_TOLERANCE_DEG = 0.25
COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG = (
    COLLISION_MARGIN_DEG + COLLISION_HOLD_TRACKING_TOLERANCE_DEG
)
COLLISION_MARGIN_POLICY = "SINGLE_JOINT_EXTENDED_SWEEP_SUPPORT_CROSS_FINAL_LINF_V2"
COLLISION_TUBE_GRID_OFFSETS = (-1, 0, 1)
COLLISION_TUBE_AXIS_PROBE_STEP_DEG = 0.25
COLLISION_TUBE_GRID_DIRECTIONS = tuple(
    sorted(
        (
            directions
            for directions in itertools.product(COLLISION_TUBE_GRID_OFFSETS, repeat=6)
            if any(directions)
        ),
        key=lambda directions: (sum(value != 0 for value in directions), directions),
    )
)
MAX_TRACKED_SOURCES = 32
HEX_32 = re.compile(r"^[0-9a-f]{32}$")
HEX_64 = re.compile(r"^[0-9a-f]{64}$")
COLLISION_REQUEST_FIELDS = frozenset(
    {
        "schema",
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
    }
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_pose_degrees(value: str) -> np.ndarray:
    fields = [field.strip() for field in value.split(",")]
    if len(fields) != 6:
        raise ValueError("session_pose_deg must contain exactly six comma-separated values")
    pose = np.radians(np.asarray([float(field) for field in fields], dtype=float))
    if not np.all(np.isfinite(pose)):
        raise ValueError("session pose contains non-finite values")
    return pose


def canonical_six_doubles_sha256(values) -> str:
    """Hash exactly six IEEE-754 binary64 values in network byte order.

    This is deliberately independent of JSON number formatting:
    ``sha256(struct.pack(">6d", *values)).hexdigest()``.
    """

    vector = finite_six(values, "canonical values")
    return hashlib.sha256(struct.pack(">6d", *vector)).hexdigest()


def finite_six(values, label: str) -> tuple[float, ...]:
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        raise ValueError(f"{label} must contain exactly six values")
    result = []
    for value in values:
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ValueError(f"{label} must contain only JSON numbers")
        converted = float(value)
        if not math.isfinite(converted):
            raise ValueError(f"{label} contains a non-finite value")
        result.append(converted)
    return tuple(result)


def valid_instance_id(value: object) -> bool:
    return isinstance(value, str) and HEX_32.fullmatch(value) is not None


def valid_sha256(value: object) -> bool:
    return isinstance(value, str) and HEX_64.fullmatch(value) is not None


@dataclass(frozen=True)
class CollisionGuardRequest:
    source_instance_id: str
    request_sequence: int
    source_monotonic_ns: int
    kind: str
    session_id: str
    session_pose_sha256: str
    state_instance_id: str
    moving_joint_mask: tuple[bool, ...]
    start_relative_rad: tuple[float, ...]
    target_relative_rad: tuple[float, ...]
    target_sha256: str
    collision_margin_deg: float
    hardware_state_sequence: int = 0
    hardware_state_source_monotonic_ns: int = 0
    hardware_position_rad: tuple[float, ...] = ()
    hardware_velocity_rad_s: tuple[float, ...] = ()
    hardware_controller_mode_by_motor: tuple[tuple[str, str], ...] = ()
    hardware_state_sha256: str = ""

    def echoed_fields(self) -> dict:
        return {
            "source_instance_id": self.source_instance_id,
            "request_sequence": self.request_sequence,
            "source_monotonic_ns": self.source_monotonic_ns,
            "kind": self.kind,
            "session_id": self.session_id,
            "session_pose_sha256": self.session_pose_sha256,
            "state_instance_id": self.state_instance_id,
            "moving_joint_mask": list(self.moving_joint_mask),
            "start_relative_rad": list(self.start_relative_rad),
            "target_relative_rad": list(self.target_relative_rad),
            "target_sha256": self.target_sha256,
            "collision_margin_deg": self.collision_margin_deg,
            "hardware_state_sequence": self.hardware_state_sequence,
            "hardware_state_source_monotonic_ns": (
                self.hardware_state_source_monotonic_ns
            ),
            "hardware_position_rad": list(self.hardware_position_rad),
            "hardware_velocity_rad_s": list(self.hardware_velocity_rad_s),
            "hardware_controller_mode_by_motor": dict(
                self.hardware_controller_mode_by_motor
            ),
            "hardware_state_sha256": self.hardware_state_sha256,
        }


def canonical_collision_hardware_state_sha256(value: dict) -> str:
    """Digest the normalized collision-authority snapshot deterministically."""

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


def hardware_state_for_collision_guard(value: object, now_ns: int) -> dict:
    """Return a copied all-motor-ready snapshot or fail closed.

    Collision geometry always depends on the complete six-joint pose.  A
    healthy moving joint cannot compensate for stale or faulted feedback on a
    nominally stationary joint, so every one of the seven physical motors is
    part of this authorization boundary.
    """

    if not isinstance(value, dict):
        raise ValueError("hardware_state is unavailable")
    if value.get("schema") != "go-m8010-hardware-state/1.1":
        raise ValueError("hardware_state schema mismatch")
    if tuple(value.get("joint_names", ())) != tuple(JOINT_NAMES):
        raise ValueError("hardware_state joint_names/order mismatch")
    session_id = value.get("session_id")
    state_instance_id = value.get("state_instance_id")
    sequence = value.get("sequence")
    source_ns = value.get("source_monotonic_ns")
    if not isinstance(session_id, str) or not session_id:
        raise ValueError("hardware_state session_id is invalid")
    if not valid_instance_id(state_instance_id):
        raise ValueError("hardware_state state_instance_id is invalid")
    if type(sequence) is not int or sequence <= 0:
        raise ValueError("hardware_state sequence is invalid")
    if (
        type(source_ns) is not int
        or source_ns <= 0
        or source_ns > now_ns
        or now_ns - source_ns > HARDWARE_STATE_SOURCE_MAX_AGE_NS
    ):
        raise ValueError("hardware_state is stale")
    positions = finite_six(value.get("position_rad"), "hardware_state position_rad")
    velocities = finite_six(
        value.get("velocity_rad_s"), "hardware_state velocity_rad_s"
    )
    if any(
        abs(velocity) > COLLISION_HARDWARE_MAX_ABS_VELOCITY_RAD_S + 1.0e-12
        for velocity in velocities
    ):
        raise ValueError("hardware_state joint velocity exceeds 0.25 deg/s")
    per_motor = value.get("per_motor")
    if not isinstance(per_motor, dict) or set(per_motor) != set(MOTOR_NAMES):
        raise ValueError("hardware_state per_motor set mismatch")
    normalized_per_motor = {}
    for name in MOTOR_NAMES:
        sample = per_motor.get(name)
        if not isinstance(sample, dict):
            raise ValueError(f"hardware_state {name} feedback is invalid")
        if type(sample.get("fresh")) is not bool or not sample["fresh"]:
            raise ValueError(f"hardware_state {name} feedback is not fresh")
        if (
            type(sample.get("reference_captured")) is not bool
            or not sample["reference_captured"]
        ):
            raise ValueError(f"hardware_state {name} reference is unavailable")
        if (
            type(sample.get("communication_ok")) is not bool
            or not sample["communication_ok"]
        ):
            raise ValueError(f"hardware_state {name} communication is not healthy")
        if type(sample.get("merror")) is not int or sample["merror"] != 0:
            raise ValueError(f"hardware_state {name} merror is nonzero or invalid")
        normalized_per_motor[name] = {
            "fresh": True,
            "reference_captured": True,
            "communication_ok": True,
            "merror": 0,
        }

    controller_faults = value.get("controller_fault_by_motor")
    if (
        not isinstance(controller_faults, dict)
        or set(controller_faults) != set(MOTOR_NAMES)
    ):
        raise ValueError("hardware_state controller fault set mismatch")
    normalized_faults = {}
    for name in MOTOR_NAMES:
        faulted = controller_faults.get(name)
        if type(faulted) is not bool:
            raise ValueError(f"hardware_state {name} controller fault is invalid")
        if faulted:
            raise ValueError(f"hardware_state {name} controller fault is active")
        normalized_faults[name] = False

    controller_modes = value.get("controller_mode_by_motor")
    if not isinstance(controller_modes, dict) or set(controller_modes) != set(
        MOTOR_NAMES
    ):
        raise ValueError("hardware_state controller mode set mismatch")
    normalized_modes = {}
    for name in MOTOR_NAMES:
        mode = controller_modes.get(name)
        if mode != "hold":
            raise ValueError(f"hardware_state {name} controller mode is not HOLD")
        normalized_modes[name] = "hold"

    j2_sync_fault = value.get("j2_sync_fault")
    if type(j2_sync_fault) is not bool:
        raise ValueError("hardware_state j2_sync_fault is invalid")
    if j2_sync_fault:
        raise ValueError("hardware_state J2 synchronization fault is active")

    # The 1.1 contract exposes an artifact-byte digest for the saved initial
    # pose.  It is not the same representation as the canonical six-double
    # session-pose digest, so validate its shape but do not conflate the two.
    initial_pose_sha256 = value.get("initial_pose_sha256")
    if initial_pose_sha256 is not None and not valid_sha256(initial_pose_sha256):
        raise ValueError("hardware_state initial_pose_sha256 is invalid")
    return {
        "schema": value["schema"],
        "joint_names": list(JOINT_NAMES),
        "session_id": session_id,
        "state_instance_id": state_instance_id,
        "sequence": sequence,
        "source_monotonic_ns": source_ns,
        "position_rad": positions,
        "velocity_rad_s": velocities,
        "per_motor": normalized_per_motor,
        "controller_mode_by_motor": normalized_modes,
        "controller_fault_by_motor": normalized_faults,
        "j2_sync_fault": False,
        "initial_pose_sha256": initial_pose_sha256,
    }


def parse_collision_guard_request(
    value: object,
    *,
    now_ns: int,
    hardware_state: object,
    expected_session_pose_sha256: str,
    previous_request: Optional[tuple[int, int]] = None,
) -> CollisionGuardRequest:
    """Validate authority, freshness, state binding and canonical target identity."""

    if not isinstance(value, dict):
        raise ValueError("request must be a JSON object")
    if frozenset(value) != COLLISION_REQUEST_FIELDS:
        raise ValueError("request fields do not exactly match the schema")
    if value.get("schema") != COLLISION_GUARD_REQUEST_SCHEMA:
        raise ValueError("request schema mismatch")
    source = value.get("source_instance_id")
    state_instance_id = value.get("state_instance_id")
    if not valid_instance_id(source):
        raise ValueError("source_instance_id must be 32 lowercase hex characters")
    if not valid_instance_id(state_instance_id):
        raise ValueError("state_instance_id must be 32 lowercase hex characters")
    sequence = value.get("request_sequence")
    source_ns = value.get("source_monotonic_ns")
    if type(sequence) is not int or sequence <= 0:
        raise ValueError("request_sequence must be a positive integer")
    if (
        type(source_ns) is not int
        or source_ns <= 0
        or source_ns > now_ns
        or now_ns - source_ns > COLLISION_GUARD_REQUEST_MAX_AGE_NS
    ):
        raise ValueError("source_monotonic_ns is future-dated or stale")
    if previous_request is not None and (
        sequence <= previous_request[0] or source_ns <= previous_request[1]
    ):
        raise ValueError("request nonce or monotonic timestamp was replayed")
    kind = value.get("kind")
    if kind not in {"preview", "pose_preview", "execute"}:
        raise ValueError("kind must be preview, pose_preview or execute")
    session_id = value.get("session_id")
    if not isinstance(session_id, str) or not session_id or len(session_id) > 512:
        raise ValueError("session_id is invalid")
    session_pose_sha256 = value.get("session_pose_sha256")
    if not valid_sha256(expected_session_pose_sha256):
        raise ValueError("expected session pose digest is invalid")
    if not valid_sha256(session_pose_sha256):
        raise ValueError("session_pose_sha256 is invalid")
    if session_pose_sha256 != expected_session_pose_sha256:
        raise ValueError("session_pose_mismatch")
    moving_joint_mask_value = value.get("moving_joint_mask")
    if (
        not isinstance(moving_joint_mask_value, list)
        or len(moving_joint_mask_value) != 6
        or not all(type(item) is bool for item in moving_joint_mask_value)
        or sum(moving_joint_mask_value) < 1
        or (kind != "pose_preview" and sum(moving_joint_mask_value) != 1)
    ):
        raise ValueError(
            "moving_joint_mask must contain exactly one true for preview/execute; "
            "pose_preview must contain at least one true"
        )
    moving_joint_mask = tuple(moving_joint_mask_value)
    start = finite_six(value.get("start_relative_rad"), "start_relative_rad")
    target = finite_six(value.get("target_relative_rad"), "target_relative_rad")
    if kind != "pose_preview" and any(
        not moving_joint_mask[index]
        and abs(target[index] - start[index])
        > COLLISION_START_MATCH_TOLERANCE_RAD + 1.0e-12
        for index in range(6)
    ):
        raise ValueError("support_joint_start_mismatch")
    target_hash = value.get("target_sha256")
    if not valid_sha256(target_hash):
        raise ValueError("target_sha256 is invalid")
    if target_hash != canonical_six_doubles_sha256(target):
        raise ValueError("target_sha256 does not bind target_relative_rad")
    margin = value.get("collision_margin_deg")
    if (
        isinstance(margin, bool)
        or not isinstance(margin, (int, float))
        or not math.isfinite(float(margin))
        or abs(float(margin) - COLLISION_MARGIN_DEG) > 1.0e-12
    ):
        raise ValueError("collision_margin_deg must be exactly 2.0")

    state = hardware_state_for_collision_guard(hardware_state, now_ns)
    if state["session_id"] != session_id:
        raise ValueError("session_mismatch")
    if state["state_instance_id"] != state_instance_id:
        raise ValueError("state_instance_mismatch")
    if any(
        abs(requested - measured) > COLLISION_START_MATCH_TOLERANCE_RAD
        for requested, measured in zip(start, state["position_rad"])
    ):
        raise ValueError("start_state_mismatch")
    hardware_state_sha256 = canonical_collision_hardware_state_sha256(state)
    return CollisionGuardRequest(
        source_instance_id=source,
        request_sequence=sequence,
        source_monotonic_ns=source_ns,
        kind=kind,
        session_id=session_id,
        session_pose_sha256=session_pose_sha256,
        state_instance_id=state_instance_id,
        moving_joint_mask=moving_joint_mask,
        start_relative_rad=start,
        target_relative_rad=target,
        target_sha256=target_hash,
        collision_margin_deg=float(margin),
        hardware_state_sequence=state["sequence"],
        hardware_state_source_monotonic_ns=state["source_monotonic_ns"],
        hardware_position_rad=state["position_rad"],
        hardware_velocity_rad_s=state["velocity_rad_s"],
        hardware_controller_mode_by_motor=tuple(
            (name, state["controller_mode_by_motor"][name])
            for name in MOTOR_NAMES
        ),
        hardware_state_sha256=hardware_state_sha256,
    )


def _contact_pairs(result: dict) -> list[list[str]]:
    pairs: set[tuple[str, str]] = set()
    contacts = result.get("contacts", [])
    if not isinstance(contacts, list):
        raise RuntimeError("guard contacts must be a list")
    for contact in contacts:
        if not isinstance(contact, dict):
            raise RuntimeError("guard contact must be an object")
        pair = contact.get("proxy_pair")
        if (
            not isinstance(pair, list)
            or len(pair) != 2
            or not all(isinstance(token, str) and token for token in pair)
        ):
            raise RuntimeError("guard contact proxy_pair is invalid")
        pairs.add(tuple(sorted(pair)))
    return [list(pair) for pair in sorted(pairs)]


def _unsafe_reason(result: dict) -> Optional[str]:
    if not isinstance(result, dict) or type(result.get("safe")) is not bool:
        raise RuntimeError("guard returned an invalid result")
    if result["safe"]:
        return None
    if result.get("reason") == "position_limit":
        return "position_limit"
    contacts = result.get("contacts", [])
    if not isinstance(contacts, list):
        raise RuntimeError("guard contacts must be a list")
    classes = {
        contact.get("contact_class")
        for contact in contacts
        if isinstance(contact, dict)
    }
    if "self_collision" in classes or result.get("reason") == "self_collision":
        return "self_collision"
    if "ground" in classes or result.get("reason") == "ground_collision":
        return "ground_collision"
    raise RuntimeError("guard rejected a pose without a recognized reason")


class GuardEvaluationCancelled(RuntimeError):
    pass


class CollisionGuardEngine:
    """Pure swept-path policy around the frozen V15.14 KinematicGuard."""

    def __init__(
        self,
        guard,
        session_pose_rad,
        *,
        absolute_joint_limits_deg,
        model_sha256: str,
        guard_sha256: str,
        contract_sha256: str,
    ) -> None:
        self.guard = guard
        self.session_pose_rad = np.asarray(session_pose_rad, dtype=float)
        if self.session_pose_rad.shape != (6,) or not np.all(
            np.isfinite(self.session_pose_rad)
        ):
            raise ValueError("session pose must contain six finite angles")
        if len(absolute_joint_limits_deg) != 6:
            raise ValueError("absolute joint limits must contain J1..J6")
        normalized_limits = []
        for bounds in absolute_joint_limits_deg:
            if bounds is None:
                normalized_limits.append(None)
                continue
            if len(bounds) != 2:
                raise ValueError("absolute joint limit must be a lower/upper pair")
            lower, upper = float(bounds[0]), float(bounds[1])
            if not math.isfinite(lower) or not math.isfinite(upper) or lower >= upper:
                raise ValueError("absolute joint limit is invalid")
            normalized_limits.append((lower, upper))
        self.absolute_joint_limits_deg = tuple(normalized_limits)
        self.model_sha256 = model_sha256
        self.guard_sha256 = guard_sha256
        self.contract_sha256 = contract_sha256
        self.session_pose_sha256 = canonical_six_doubles_sha256(
            self.session_pose_rad.tolist()
        )

    def _base_result(self, request: CollisionGuardRequest) -> dict:
        return {
            "schema": COLLISION_GUARD_RESULT_SCHEMA,
            **request.echoed_fields(),
            "checked_monotonic_ns": time.monotonic_ns(),
            "safe": False,
            "reason": "internal_error",
            "recommended_relative_rad": None,
            "contact_pairs": [],
            "model_sha256": self.model_sha256,
            "kinematic_guard_sha256": self.guard_sha256,
            "collision_contract_sha256": self.contract_sha256,
            "ground_contact_policy": "UNSAFE_GROUND_COLLISION_NOT_SELF_COLLISION",
            "max_step_deg": COLLISION_GUARD_MAX_STEP_DEG,
            "boundary_tolerance_deg": COLLISION_BOUNDARY_TOLERANCE_DEG,
            # These fields are part of the authorization proof.  Consumers
            # must match every value exactly; a result from the retired
            # forward-ray policy must never authorize physical motion.
            "margin_policy": COLLISION_MARGIN_POLICY,
            "joint_space_tube_radius_deg": COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG,
            "hold_tracking_tolerance_deg": (
                COLLISION_HOLD_TRACKING_TOLERANCE_DEG
            ),
            "tube_probe_joint_names": list(MUJOCO_JOINT_NAMES),
            "tube_grid_offsets": list(COLLISION_TUBE_GRID_OFFSETS),
            "tube_grid_max_pose_count": 3 ** len(MUJOCO_JOINT_NAMES),
            "tube_axis_probe_step_deg": COLLISION_TUBE_AXIS_PROBE_STEP_DEG,
            "tube_cross_section_max_pose_count": (
                3 ** len(MUJOCO_JOINT_NAMES)
                + 2
                * len(MUJOCO_JOINT_NAMES)
                * int(
                    math.floor(
                        (
                            COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG - 1.0e-12
                        )
                        / COLLISION_TUBE_AXIS_PROBE_STEP_DEG
                    )
                )
            ),
            "absolute_joint_limits_deg": [
                list(bounds) if bounds is not None else None
                for bounds in self.absolute_joint_limits_deg
            ],
            "single_joint_path_required": True,
        }

    def _tube_axis_interior_probes(self, center_deg: np.ndarray):
        """Sample every single-axis spoke inside the tube at 0.25 degrees."""

        seen = {tuple(round(float(value), 12) for value in center_deg)}
        maximum_step = int(math.floor(
            (COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG - 1.0e-12)
            / COLLISION_TUBE_AXIS_PROBE_STEP_DEG
        ))
        for index in range(6):
            bounds = self.absolute_joint_limits_deg[index]
            for direction in (-1, 1):
                for step in range(1, maximum_step + 1):
                    endpoint = float(center_deg[index]) + (
                        direction * step * COLLISION_TUBE_AXIS_PROBE_STEP_DEG
                    )
                    if bounds is not None:
                        endpoint = min(bounds[1], max(bounds[0], endpoint))
                    probe = center_deg.copy()
                    probe[index] = endpoint
                    key = tuple(round(float(value), 12) for value in probe)
                    if key in seen:
                        continue
                    seen.add(key)
                    directions = [0] * 6
                    directions[index] = direction
                    offsets = [0.0] * 6
                    offsets[index] = endpoint - float(center_deg[index])
                    yield tuple(directions), tuple(offsets), probe

    def _tube_grid_probes(self, center_deg: np.ndarray):
        """Yield the in-domain {-r, 0, +r}^6 grid, without duplicates."""

        seen = {tuple(round(float(value), 12) for value in center_deg)}
        for directions in COLLISION_TUBE_GRID_DIRECTIONS:
            probe = center_deg.copy()
            offsets = []
            for index, direction in enumerate(directions):
                endpoint = float(center_deg[index]) + (
                    direction * COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG
                )
                bounds = self.absolute_joint_limits_deg[index]
                if bounds is not None:
                    endpoint = min(bounds[1], max(bounds[0], endpoint))
                probe[index] = endpoint
                offsets.append(endpoint - float(center_deg[index]))
            # Clipping an outward probe at a mechanical endpoint can collapse
            # several ternary-grid points onto the same physical pose.
            key = tuple(round(float(value), 12) for value in probe)
            if key in seen:
                continue
            seen.add(key)
            yield directions, tuple(offsets), probe

    def _guard_check_cached(
        self,
        pose_deg: np.ndarray,
        should_cancel: Callable[[], bool],
        pose_cache: dict[tuple[float, ...], dict],
    ) -> tuple[dict, bool]:
        if should_cancel():
            raise GuardEvaluationCancelled("collision check superseded")
        key = tuple(round(float(value), 12) for value in pose_deg)
        cached = pose_cache.get(key)
        if cached is not None:
            return cached, True
        violations = [
            {
                "joint": MUJOCO_JOINT_NAMES[index],
                "value_deg": float(value),
                "allowed_deg": list(bounds),
            }
            for index, (value, bounds) in enumerate(
                zip(pose_deg, self.absolute_joint_limits_deg)
            )
            if bounds is not None
            and not (
                bounds[0] - 1.0e-9 <= float(value) <= bounds[1] + 1.0e-9
            )
        ]
        checked = (
            {
                "safe": False,
                "reason": "position_limit",
                "contacts": [],
                "violations": violations,
            }
            if violations
            else self.guard.check_pose_deg(pose_deg)
        )
        pose_cache[key] = checked
        return checked, False

    def _check_tube_cross_section(
        self,
        center_deg: np.ndarray,
        should_cancel: Callable[[], bool],
        pose_cache: dict[tuple[float, ...], dict],
    ) -> dict:
        """Check a center pose plus its complete ternary L-infinity grid."""

        center_result, center_cached = self._guard_check_cached(
            center_deg, should_cancel, pose_cache
        )
        center_reason = _unsafe_reason(center_result)
        checks = 0 if center_cached else 1
        cache_hits = 1 if center_cached else 0
        if center_reason is not None:
            return {
                "reason": center_reason,
                "guard_result": center_result,
                "probe_directions": (0,) * 6,
                "probe_offsets_deg": (0.0,) * 6,
                "pose_check_count": checks,
                "tube_probe_count": 0,
                "tube_probe_cache_hit_count": cache_hits,
            }

        tube_probe_count = 0
        for directions, offsets_deg, probe_deg in self._tube_axis_interior_probes(
            center_deg
        ):
            probe_result, was_cached = self._guard_check_cached(
                probe_deg, should_cancel, pose_cache
            )
            checks += 0 if was_cached else 1
            cache_hits += 1 if was_cached else 0
            tube_probe_count += 1
            probe_reason = _unsafe_reason(probe_result)
            if probe_reason is not None:
                return {
                    "reason": probe_reason,
                    "guard_result": probe_result,
                    "probe_directions": directions,
                    "probe_offsets_deg": offsets_deg,
                    "pose_check_count": checks,
                    "tube_probe_count": tube_probe_count,
                    "tube_probe_cache_hit_count": cache_hits,
                }
        for directions, offsets_deg, probe_deg in self._tube_grid_probes(center_deg):
            probe_result, was_cached = self._guard_check_cached(
                probe_deg, should_cancel, pose_cache
            )
            checks += 0 if was_cached else 1
            cache_hits += 1 if was_cached else 0
            tube_probe_count += 1
            probe_reason = _unsafe_reason(probe_result)
            if probe_reason is not None:
                return {
                    "reason": probe_reason,
                    "guard_result": probe_result,
                    "probe_directions": directions,
                    "probe_offsets_deg": offsets_deg,
                    "pose_check_count": checks,
                    "tube_probe_count": tube_probe_count,
                    "tube_probe_cache_hit_count": cache_hits,
                }
        return {
            "reason": None,
            "guard_result": center_result,
            "probe_directions": (0,) * 6,
            "probe_offsets_deg": (0.0,) * 6,
            "pose_check_count": checks,
            "tube_probe_count": tube_probe_count,
            "tube_probe_cache_hit_count": cache_hits,
        }

    def _check_execute_support_cross_section(
        self,
        center_deg: np.ndarray,
        moving_index: int,
        should_cancel: Callable[[], bool],
        pose_cache: dict[tuple[float, ...], dict],
    ) -> dict:
        """Check the path center plus each HOLD-axis tracking endpoint.

        Physical execution changes exactly one joint.  The moving-axis 2°
        clearance is represented by extending the entire swept interval; the
        other five joints remain in immutable HOLD and are each checked at
        both ends of the independently enforced ±0.25° tracking gate.  This
        is the authorizing real-time policy.  Multi-joint virtual final-pose
        preview remains the more conservative full 825-pose L-infinity grid.
        """

        center_result, center_cached = self._guard_check_cached(
            center_deg, should_cancel, pose_cache
        )
        checks = 0 if center_cached else 1
        cache_hits = 1 if center_cached else 0
        center_reason = _unsafe_reason(center_result)
        if center_reason is not None:
            return {
                "reason": center_reason,
                "guard_result": center_result,
                "probe_directions": (0,) * 6,
                "probe_offsets_deg": (0.0,) * 6,
                "pose_check_count": checks,
                "tube_probe_count": 0,
                "tube_probe_cache_hit_count": cache_hits,
            }

        tube_probe_count = 0
        for support_index in range(6):
            if support_index == moving_index:
                continue
            for direction in (-1, 1):
                directions = [0] * 6
                offsets = [0.0] * 6
                directions[support_index] = direction
                offsets[support_index] = (
                    direction * COLLISION_HOLD_TRACKING_TOLERANCE_DEG
                )
                probe_deg = center_deg.copy()
                endpoint = float(probe_deg[support_index]) + offsets[
                    support_index
                ]
                bounds = self.absolute_joint_limits_deg[support_index]
                if bounds is not None:
                    endpoint = min(bounds[1], max(bounds[0], endpoint))
                probe_deg[support_index] = endpoint
                offsets[support_index] = (
                    endpoint - float(center_deg[support_index])
                )
                probe_result, was_cached = self._guard_check_cached(
                    probe_deg, should_cancel, pose_cache
                )
                checks += 0 if was_cached else 1
                cache_hits += 1 if was_cached else 0
                tube_probe_count += 1
                probe_reason = _unsafe_reason(probe_result)
                if probe_reason is not None:
                    return {
                        "reason": probe_reason,
                        "guard_result": probe_result,
                        "probe_directions": tuple(directions),
                        "probe_offsets_deg": tuple(offsets),
                        "pose_check_count": checks,
                        "tube_probe_count": tube_probe_count,
                        "tube_probe_cache_hit_count": cache_hits,
                    }
        return {
            "reason": None,
            "guard_result": center_result,
            "probe_directions": (0,) * 6,
            "probe_offsets_deg": (0.0,) * 6,
            "pose_check_count": checks,
            "tube_probe_count": tube_probe_count,
            "tube_probe_cache_hit_count": cache_hits,
        }

    def evaluate(
        self,
        request: CollisionGuardRequest,
        should_cancel: Callable[[], bool] = lambda: False,
    ) -> dict:
        start_relative = np.asarray(request.start_relative_rad, dtype=float)
        target_relative = np.asarray(request.target_relative_rad, dtype=float)
        start_deg = np.degrees(self.session_pose_rad + start_relative)
        target_deg = np.degrees(self.session_pose_rad + target_relative)
        requested_delta_deg = target_deg - start_deg
        result = self._base_result(request)

        # Keep this check inside the pure engine as defence in depth for any
        # caller that constructs CollisionGuardRequest without the ROS parser.
        if request.session_pose_sha256 != self.session_pose_sha256:
            result.update(
                checked_monotonic_ns=time.monotonic_ns(),
                reason="session_pose_mismatch",
            )
            return result

        target_limit_violations = [
            index
            for index, (value, bounds) in enumerate(
                zip(target_deg, self.absolute_joint_limits_deg)
            )
            if bounds is not None
            and not (
                bounds[0] - 1.0e-9 <= float(value) <= bounds[1] + 1.0e-9
            )
        ]
        if target_limit_violations:
            result.update(
                checked_monotonic_ns=time.monotonic_ns(),
                reason="position_limit",
                recommended_relative_rad=None,
                contact_pairs=[],
            )
            return result

        if request.kind == "pose_preview":
            # Multi-joint editing gets a non-authorizing final-pose margin
            # preview.  Physical execution still requires fresh one-joint
            # `execute` proofs for every sequential segment.
            moving_count = sum(request.moving_joint_mask)
            result.update(
                moving_joint=(
                    MUJOCO_JOINT_NAMES[request.moving_joint_mask.index(True)]
                    if moving_count == 1 else None
                ),
                moving_joint_count=moving_count,
            )
            pose_cache: dict[tuple[float, ...], dict] = {}
            cross_section = self._check_tube_cross_section(
                target_deg, should_cancel, pose_cache
            )
            common = {
                "checked_monotonic_ns": time.monotonic_ns(),
                "step_count": 0,
                "path_sample_count": 1,
                "pose_check_count": cross_section["pose_check_count"],
                "tube_probe_count": cross_section["tube_probe_count"],
                "tube_probe_cache_hit_count": cross_section[
                    "tube_probe_cache_hit_count"
                ],
            }
            if cross_section["reason"] is None:
                result.update(
                    **common,
                    safe=True,
                    reason="clear",
                    recommended_relative_rad=list(request.target_relative_rad),
                    contact_pairs=[],
                )
                return result
            if cross_section["reason"] == "position_limit":
                result.update(
                    **common,
                    reason="position_limit",
                    contact_pairs=[],
                )
                return result
            if cross_section["reason"] not in {
                "self_collision", "ground_collision",
            }:
                raise RuntimeError("pose preview returned an invalid reason")
            result.update(
                **common,
                reason=f"{cross_section['reason']}_margin",
                recommended_relative_rad=list(request.start_relative_rad),
                contact_pairs=_contact_pairs(cross_section["guard_result"]),
                margin_probe_directions=list(
                    cross_section["probe_directions"]
                ),
                margin_probe_offsets_deg=list(
                    cross_section["probe_offsets_deg"]
                ),
            )
            return result

        if (
            not isinstance(request.moving_joint_mask, tuple)
            or len(request.moving_joint_mask) != 6
            or not all(type(item) is bool for item in request.moving_joint_mask)
            or sum(request.moving_joint_mask) != 1
        ):
            result.update(
                checked_monotonic_ns=time.monotonic_ns(),
                reason="moving_joint_mask_invalid",
                moving_joint=None,
                moving_joint_count=0,
            )
            return result
        moving_index = request.moving_joint_mask.index(True)
        if any(
            index != moving_index
            and abs(float(requested_delta_deg[index]))
            > math.degrees(COLLISION_START_MATCH_TOLERANCE_RAD) + 1.0e-12
            for index in range(6)
        ):
            result.update(
                checked_monotonic_ns=time.monotonic_ns(),
                reason="support_joint_start_mismatch",
                moving_joint=MUJOCO_JOINT_NAMES[moving_index],
                moving_joint_count=1,
            )
            return result

        try:
            proof_modes = dict(request.hardware_controller_mode_by_motor)
            proof_state = {
                "schema": "go-m8010-hardware-state/1.1",
                "session_id": request.session_id,
                "state_instance_id": request.state_instance_id,
                "sequence": request.hardware_state_sequence,
                "source_monotonic_ns": request.hardware_state_source_monotonic_ns,
                "position_rad": finite_six(
                    list(request.hardware_position_rad), "hardware proof position_rad"
                ),
                "velocity_rad_s": finite_six(
                    list(request.hardware_velocity_rad_s),
                    "hardware proof velocity_rad_s",
                ),
                "controller_mode_by_motor": proof_modes,
            }
            if (
                type(request.hardware_state_sequence) is not int
                or request.hardware_state_sequence <= 0
                or type(request.hardware_state_source_monotonic_ns) is not int
                or request.hardware_state_source_monotonic_ns <= 0
                or set(proof_modes) != set(MOTOR_NAMES)
                or any(proof_modes[name] != "hold" for name in MOTOR_NAMES)
                or any(
                    abs(value)
                    > COLLISION_HARDWARE_MAX_ABS_VELOCITY_RAD_S + 1.0e-12
                    for value in proof_state["velocity_rad_s"]
                )
                or any(
                    abs(requested - measured)
                    > COLLISION_START_MATCH_TOLERANCE_RAD + 1.0e-12
                    for requested, measured in zip(
                        request.start_relative_rad,
                        proof_state["position_rad"],
                    )
                )
                or request.hardware_state_sha256
                != canonical_collision_hardware_state_sha256(proof_state)
            ):
                raise ValueError("hardware proof mismatch")
        except (KeyError, TypeError, ValueError):
            result.update(
                checked_monotonic_ns=time.monotonic_ns(),
                reason="hardware_state_proof_invalid",
                moving_joint=MUJOCO_JOINT_NAMES[moving_index],
                moving_joint_count=1,
            )
            return result

        moving_joint = MUJOCO_JOINT_NAMES[moving_index]
        result.update(
            moving_joint=moving_joint,
            moving_joint_count=1,
        )

        # The physical command changes only the masked joint.  Expand that
        # complete swept interval by the requested 2° collision clearance plus
        # the 0.25° HOLD tracking gate.  At every 0.25° path sample, probe both
        # tracking endpoints of each immutable support joint.  Unlike the old
        # 825-pose grid at every path sample, this policy matches the actual
        # one-moving-axis authority and completes within the 8 s GUI budget.
        center_start_deg = target_deg.copy()
        center_start_deg[moving_index] = start_deg[moving_index]
        command_start_deg = float(center_start_deg[moving_index])
        command_target_deg = float(target_deg[moving_index])
        target_path_length_deg = abs(command_target_deg - command_start_deg)
        pose_cache: dict[tuple[float, ...], dict] = {}

        moving_bounds = self.absolute_joint_limits_deg[moving_index]
        if moving_bounds is None:
            moving_bounds = (-math.inf, math.inf)
        clearance_deg = (
            COLLISION_MARGIN_DEG + COLLISION_HOLD_TRACKING_TOLERANCE_DEG
        )
        movement_direction = (
            1.0 if command_target_deg >= command_start_deg else -1.0
        )
        sweep_start_deg = min(moving_bounds[1], max(
            moving_bounds[0],
            command_start_deg - movement_direction * clearance_deg,
        ))
        sweep_end_deg = min(moving_bounds[1], max(
            moving_bounds[0],
            command_target_deg + movement_direction * clearance_deg,
        ))
        clearance_path_length_deg = abs(sweep_end_deg - sweep_start_deg)
        clearance_steps = (
            max(1, int(math.ceil(
                clearance_path_length_deg / COLLISION_GUARD_MAX_STEP_DEG
            )))
            if clearance_path_length_deg > 0.0
            else 0
        )
        clearance_samples_deg = (
            [
                sweep_start_deg
                + (sweep_end_deg - sweep_start_deg) * step / clearance_steps
                for step in range(clearance_steps + 1)
            ]
            if clearance_steps
            else [sweep_start_deg]
        )
        pose_check_count = 0
        tube_probe_count = 0
        tube_probe_cache_hit_count = 0
        path_sample_count = 0

        # Check the exact measured start as well as the conservative tube
        # centered on immutable support targets.
        exact_start_result, exact_start_cached = self._guard_check_cached(
            start_deg, should_cancel, pose_cache
        )
        pose_check_count += 0 if exact_start_cached else 1
        tube_probe_cache_hit_count += 1 if exact_start_cached else 0
        exact_start_reason = _unsafe_reason(exact_start_result)
        if exact_start_reason is not None:
            result.update(
                checked_monotonic_ns=time.monotonic_ns(),
                reason=(
                    "position_limit"
                    if exact_start_reason == "position_limit"
                    else f"{exact_start_reason}_margin"
                ),
                contact_pairs=(
                    []
                    if exact_start_reason == "position_limit"
                    else _contact_pairs(exact_start_result)
                ),
                first_unsafe_fraction=0.0,
                first_unsafe_path_deg=0.0,
                target_path_length_deg=target_path_length_deg,
                margin_probe_directions=[0] * 6,
                margin_probe_offsets_deg=[
                    float(start_deg[index] - center_start_deg[index])
                    for index in range(6)
                ],
                step_count=clearance_steps,
                path_sample_count=0,
                pose_check_count=pose_check_count,
                tube_probe_count=0,
                tube_probe_cache_hit_count=tube_probe_cache_hit_count,
            )
            return result

        first_unsafe_check: Optional[dict] = None
        first_unsafe_moving_deg: Optional[float] = None
        previous_safe_moving_deg: Optional[float] = None
        for moving_sample_deg in clearance_samples_deg:
            center_deg = target_deg.copy()
            center_deg[moving_index] = moving_sample_deg
            cross_section = self._check_execute_support_cross_section(
                center_deg,
                moving_index,
                should_cancel,
                pose_cache,
            )
            path_sample_count += 1
            pose_check_count += cross_section["pose_check_count"]
            tube_probe_count += cross_section["tube_probe_count"]
            tube_probe_cache_hit_count += cross_section[
                "tube_probe_cache_hit_count"
            ]
            if cross_section["reason"] is None:
                previous_safe_moving_deg = moving_sample_deg
            else:
                first_unsafe_check = cross_section
                first_unsafe_moving_deg = moving_sample_deg
                break

        if first_unsafe_check is None or first_unsafe_moving_deg is None:
            result.update(
                checked_monotonic_ns=time.monotonic_ns(),
                safe=True,
                reason="clear",
                recommended_relative_rad=list(request.target_relative_rad),
                contact_pairs=[],
                step_count=clearance_steps,
                path_sample_count=path_sample_count,
                pose_check_count=pose_check_count,
                tube_probe_count=tube_probe_count,
                tube_probe_cache_hit_count=tube_probe_cache_hit_count,
            )
            return result

        boundary_reason = first_unsafe_check["reason"]
        if boundary_reason == "position_limit":
            rejection_reason = "position_limit"
            contacts = []
        elif boundary_reason in {"self_collision", "ground_collision"}:
            if previous_safe_moving_deg is not None:
                low_deg = previous_safe_moving_deg
                high_deg = first_unsafe_moving_deg
                while abs(high_deg - low_deg) > COLLISION_BOUNDARY_TOLERANCE_DEG:
                    midpoint_deg = (low_deg + high_deg) * 0.5
                    center_deg = target_deg.copy()
                    center_deg[moving_index] = midpoint_deg
                    midpoint_check = self._check_execute_support_cross_section(
                        center_deg,
                        moving_index,
                        should_cancel,
                        pose_cache,
                    )
                    pose_check_count += midpoint_check["pose_check_count"]
                    tube_probe_count += midpoint_check["tube_probe_count"]
                    tube_probe_cache_hit_count += midpoint_check[
                        "tube_probe_cache_hit_count"
                    ]
                    if midpoint_check["reason"] is None:
                        low_deg = midpoint_deg
                    else:
                        high_deg = midpoint_deg
                        first_unsafe_check = midpoint_check
                first_unsafe_moving_deg = high_deg
            rejection_reason = f"{boundary_reason}_margin"
            contacts = _contact_pairs(first_unsafe_check["guard_result"])
        else:
            raise RuntimeError("clearance sweep changed to an invalid reason")
        if rejection_reason == "position_limit":
            recommendation = None
            recommended_progress_deg = 0.0
        else:
            recommended_moving_deg = (
                first_unsafe_moving_deg
                - movement_direction * clearance_deg
            )
            recommended_progress_deg = min(target_path_length_deg, max(
                0.0,
                movement_direction
                * (recommended_moving_deg - command_start_deg),
            ))
            recommended_absolute_deg = target_deg.copy()
            recommended_absolute_deg[moving_index] = (
                command_start_deg
                + movement_direction * recommended_progress_deg
            )
            recommendation = (
                np.radians(recommended_absolute_deg) - self.session_pose_rad
            ).tolist()
        unsafe_fraction = (
            recommended_progress_deg / target_path_length_deg
            if target_path_length_deg > 0.0 else 0.0
        )
        result.update(
            checked_monotonic_ns=time.monotonic_ns(),
            reason=rejection_reason,
            # A rejected clearance sweep never creates a new physical target.
            # Roll the virtual edit back to the checked start and keep the
            # existing HOLD authority unchanged.
            recommended_relative_rad=recommendation,
            contact_pairs=contacts,
            first_unsafe_fraction=unsafe_fraction,
            first_unsafe_path_deg=(
                unsafe_fraction * target_path_length_deg
            ),
            target_path_length_deg=target_path_length_deg,
            margin_probe_directions=list(
                first_unsafe_check["probe_directions"]
            ),
            margin_probe_offsets_deg=list(
                first_unsafe_check["probe_offsets_deg"]
            ),
            step_count=clearance_steps,
            path_sample_count=path_sample_count,
            pose_check_count=pose_check_count,
            tube_probe_count=tube_probe_count,
            tube_probe_cache_hit_count=tube_probe_cache_hit_count,
        )
        return result


class LatestCollisionGuardWorker:
    """Run one MuJoCo guard at a time and coalesce queued preview requests."""

    def __init__(
        self,
        evaluator: Callable[[CollisionGuardRequest, Callable[[], bool]], dict],
        publish: Callable[[dict], None],
    ) -> None:
        self.evaluator = evaluator
        self.publish = publish
        self.condition = threading.Condition()
        self.pending: Optional[tuple[int, CollisionGuardRequest]] = None
        self.generation = 0
        self.stopping = False
        self.thread = threading.Thread(
            target=self._run,
            name="mujoco-collision-guard",
            daemon=True,
        )
        self.thread.start()

    def submit(self, request: CollisionGuardRequest) -> None:
        with self.condition:
            if self.stopping:
                raise RuntimeError("collision guard worker is stopping")
            self.generation += 1
            self.pending = (self.generation, request)
            self.condition.notify()

    def _run(self) -> None:
        while True:
            with self.condition:
                while self.pending is None and not self.stopping:
                    self.condition.wait()
                if self.stopping:
                    return
                generation, request = self.pending
                self.pending = None

            def superseded() -> bool:
                with self.condition:
                    return self.stopping or generation != self.generation

            try:
                result = self.evaluator(request, superseded)
            except GuardEvaluationCancelled:
                continue
            except Exception as exc:
                result = fail_closed_collision_result(
                    request.echoed_fields(),
                    f"internal_error:{type(exc).__name__}",
                )
            if not superseded():
                self.publish(result)

    def close(self) -> None:
        """Cancel all work and wait until the worker can no longer publish."""

        with self.condition:
            if self.stopping and not self.thread.is_alive():
                return
            self.stopping = True
            self.pending = None
            self.generation += 1
            self.condition.notify_all()
        if threading.current_thread() is self.thread:
            raise RuntimeError("collision guard worker cannot join itself")
        # CollisionGuardEngine polls the cancellation predicate throughout its
        # bounded sweep and boundary search.  An unbounded join here is
        # intentional: teardown must not continue while an evaluator can still
        # reach the publisher.
        self.thread.join()


def fail_closed_collision_result(value: object, reason: str) -> dict:
    """Publish a non-authoritative rejection even when request parsing failed."""

    source = value if isinstance(value, dict) else {}

    def text_field(name: str) -> str:
        field = source.get(name)
        return field if isinstance(field, str) else ""

    def positive_integer(name: str) -> int:
        field = source.get(name)
        return field if type(field) is int and field > 0 else 0

    def vector_field(name: str) -> list[float]:
        try:
            return list(finite_six(source.get(name), name))
        except ValueError:
            return []

    def boolean_vector_field(name: str) -> list[bool]:
        field = source.get(name)
        if (
            isinstance(field, (list, tuple))
            and len(field) == 6
            and all(type(item) is bool for item in field)
        ):
            return list(field)
        return []

    raw_modes = source.get("hardware_controller_mode_by_motor")
    proof_modes = (
        dict(raw_modes)
        if isinstance(raw_modes, dict)
        and set(raw_modes) == set(MOTOR_NAMES)
        and all(isinstance(raw_modes[name], str) for name in MOTOR_NAMES)
        else {}
    )

    raw_margin = source.get("collision_margin_deg")
    margin = (
        float(raw_margin)
        if not isinstance(raw_margin, bool)
        and isinstance(raw_margin, (int, float))
        and math.isfinite(float(raw_margin))
        else COLLISION_MARGIN_DEG
    )
    return {
        "schema": COLLISION_GUARD_RESULT_SCHEMA,
        "source_instance_id": text_field("source_instance_id"),
        "request_sequence": positive_integer("request_sequence"),
        "source_monotonic_ns": positive_integer("source_monotonic_ns"),
        "kind": text_field("kind") or "invalid",
        "session_id": text_field("session_id"),
        "session_pose_sha256": text_field("session_pose_sha256"),
        "state_instance_id": text_field("state_instance_id"),
        "moving_joint_mask": boolean_vector_field("moving_joint_mask"),
        "start_relative_rad": vector_field("start_relative_rad"),
        "target_relative_rad": vector_field("target_relative_rad"),
        "target_sha256": text_field("target_sha256"),
        "collision_margin_deg": margin,
        "hardware_state_sequence": positive_integer("hardware_state_sequence"),
        "hardware_state_source_monotonic_ns": positive_integer(
            "hardware_state_source_monotonic_ns"
        ),
        "hardware_position_rad": vector_field("hardware_position_rad"),
        "hardware_velocity_rad_s": vector_field("hardware_velocity_rad_s"),
        "hardware_controller_mode_by_motor": proof_modes,
        "hardware_state_sha256": text_field("hardware_state_sha256"),
        "checked_monotonic_ns": time.monotonic_ns(),
        "safe": False,
        "reason": reason[:160],
        "recommended_relative_rad": None,
        "contact_pairs": [],
        "model_sha256": PRODUCTION_MODEL_SHA256,
        "kinematic_guard_sha256": PRODUCTION_KINEMATIC_GUARD_SHA256,
        "collision_contract_sha256": PRODUCTION_COLLISION_CONTRACT_SHA256,
        "ground_contact_policy": "UNSAFE_GROUND_COLLISION_NOT_SELF_COLLISION",
        "max_step_deg": COLLISION_GUARD_MAX_STEP_DEG,
        "boundary_tolerance_deg": COLLISION_BOUNDARY_TOLERANCE_DEG,
        "margin_policy": COLLISION_MARGIN_POLICY,
        "joint_space_tube_radius_deg": COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG,
        "hold_tracking_tolerance_deg": COLLISION_HOLD_TRACKING_TOLERANCE_DEG,
        "tube_probe_joint_names": list(MUJOCO_JOINT_NAMES),
        "tube_grid_offsets": list(COLLISION_TUBE_GRID_OFFSETS),
        "tube_grid_max_pose_count": 3 ** len(MUJOCO_JOINT_NAMES),
        "tube_axis_probe_step_deg": COLLISION_TUBE_AXIS_PROBE_STEP_DEG,
        "tube_cross_section_max_pose_count": (
            3 ** len(MUJOCO_JOINT_NAMES)
            + 2
            * len(MUJOCO_JOINT_NAMES)
            * int(math.floor(
                (COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG - 1.0e-12)
                / COLLISION_TUBE_AXIS_PROBE_STEP_DEG
            ))
        ),
        "absolute_joint_limits_deg": [
            list(bounds) for bounds in PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG
        ],
        "single_joint_path_required": True,
    }


def load_verified_kinematic_guard(model_path: Path, model, mujoco_module):
    guard_path = model_path.parent / "kinematic_guard.py"
    contract_path = model_path.parent / "collision_pair_contract_v15_14.json"
    if not guard_path.is_file() or not contract_path.is_file():
        raise FileNotFoundError("frozen kinematic guard or collision contract is missing")
    guard_hash = sha256_file(guard_path)
    contract_hash = sha256_file(contract_path)
    if guard_hash != PRODUCTION_KINEMATIC_GUARD_SHA256:
        raise RuntimeError(f"冻结运动学守卫哈希不匹配: {guard_hash}")
    if contract_hash != PRODUCTION_COLLISION_CONTRACT_SHA256:
        raise RuntimeError(f"冻结碰撞配对契约哈希不匹配: {contract_hash}")
    spec = importlib.util.spec_from_file_location(
        "go_m8010_v15_14_frozen_kinematic_guard", guard_path
    )
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load frozen kinematic guard: {guard_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if Path(module.MODEL_XML).resolve() != model_path:
        raise RuntimeError("frozen guard model path does not match mirror model")
    if Path(module.PAIR_CONTRACT).resolve() != contract_path:
        raise RuntimeError("frozen guard contract path does not match verified contract")
    guard_data = mujoco_module.MjData(model)
    return module.KinematicGuard(model=model, data=guard_data), guard_hash, contract_hash


class WholeArmMujocoMirror(Node):
    def __init__(self) -> None:
        super().__init__("whole_arm_mujoco_mirror")
        self.declare_parameter("model_path", "")
        self.declare_parameter("session_pose_deg", "0,0,0,0,0,0")
        self.declare_parameter("pose_matched", False)
        self.declare_parameter("session_relative_baseline", False)
        self.declare_parameter("numeric_test_only", False)
        self.declare_parameter("use_viewer", True)
        self.declare_parameter("evidence_directory", "logs/arm_gui")

        self.pose_matched = bool(self.get_parameter("pose_matched").value)
        self.session_relative_baseline = bool(
            self.get_parameter("session_relative_baseline").value
        )
        self.numeric_test_only = bool(self.get_parameter("numeric_test_only").value)
        if not self.pose_matched and not self.session_relative_baseline and not self.numeric_test_only:
            raise RuntimeError(
                "operator visual pose match is required: set pose_matched:=true and session_pose_deg"
            )
        model_path = Path(str(self.get_parameter("model_path").value)).expanduser().resolve()
        if not model_path.is_file():
            raise FileNotFoundError(f"frozen production MuJoCo model not found: {model_path}")
        self.session_pose = parse_pose_degrees(str(self.get_parameter("session_pose_deg").value))
        self.session_pose_sha256 = canonical_six_doubles_sha256(
            self.session_pose.tolist()
        )

        import mujoco

        self.mujoco = mujoco
        self.model_path = model_path
        self.model_hash = sha256_file(model_path)
        if self.model_hash != PRODUCTION_MODEL_SHA256:
            raise RuntimeError(
                f"冻结生产MuJoCo模型哈希不匹配: {self.model_hash}"
            )
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        # The audited 231-pair collision contract includes two directly
        # connected-link pairs.  Let KinematicGuard's explicit allow-list,
        # rather than MuJoCo's broad parent filter, decide those contacts.
        self.model.opt.disableflags |= int(
            mujoco.mjtDisableBit.mjDSBL_FILTERPARENT
        )
        self.data = mujoco.MjData(self.model)
        (
            self.collision_guard,
            self.guard_hash,
            self.collision_contract_hash,
        ) = load_verified_kinematic_guard(model_path, self.model, mujoco)
        qpos_addresses = []
        qvel_addresses = []
        absolute_joint_limits_deg: list[Optional[tuple[float, float]]] = []
        for index, name in enumerate(MUJOCO_JOINT_NAMES):
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise RuntimeError(f"MuJoCo model is missing joint {name}")
            qpos_addresses.append(int(self.model.jnt_qposadr[joint_id]))
            qvel_addresses.append(int(self.model.jnt_dofadr[joint_id]))
            # J1 is continuous in the frozen MJCF for kinematic branch
            # handling, but the production GUI/physical contract intentionally
            # limits it to one turn (-180..+180).  Pin every joint to that
            # audited production envelope instead of treating MJCF
            # `limited=false` as unlimited physical authorization.
            absolute_joint_limits_deg.append(
                PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG[index]
            )
        self.qpos_addresses = np.asarray(qpos_addresses, dtype=int)
        self.qvel_addresses = np.asarray(qvel_addresses, dtype=int)
        self.absolute_joint_limits_deg = tuple(absolute_joint_limits_deg)
        self.collision_guard_engine = CollisionGuardEngine(
            self.collision_guard,
            self.session_pose,
            absolute_joint_limits_deg=self.absolute_joint_limits_deg,
            model_sha256=self.model_hash,
            guard_sha256=self.guard_hash,
            contract_sha256=self.collision_contract_hash,
        )
        self.data.qpos[self.qpos_addresses] = self.session_pose
        self.data.qvel[self.qvel_addresses] = 0.0
        mujoco.mj_forward(self.model, self.data)

        evidence = Path(str(self.get_parameter("evidence_directory").value)).resolve()
        evidence.mkdir(parents=True, exist_ok=True)
        self.validation_path = evidence / "mirror_validation.json"
        capture_path = evidence / "mujoco_mirror_capture.csv"
        self.capture_stream = capture_path.open("w", encoding="utf-8", newline="")
        fieldnames = ["sequence", "ros_stamp_ns", "receipt_monotonic_ns", "latency_ms", "max_error_rad"]
        fieldnames += [f"{name}_target_qpos_rad" for name in JOINT_NAMES]
        fieldnames += [f"{name}_actual_qpos_rad" for name in JOINT_NAMES]
        self.capture_writer = csv.DictWriter(self.capture_stream, fieldnames=fieldnames)
        self.capture_writer.writeheader()

        self.subscription = self.create_subscription(JointState, "/joint_states", self.on_joint_state, 1)
        self.status_publisher = self.create_publisher(String, "/whole_arm/mujoco_mirror_status", 10)
        self.collision_result_publisher = self.create_publisher(
            String, COLLISION_GUARD_RESULT_TOPIC, 10
        )
        self.collision_result_publish_lock = threading.Lock()
        self.collision_results_enabled = True
        self.sequence = 0
        self.rejected_messages = 0
        self.collision_guard_rejections = 0
        self.latest_collision_hardware_state: Optional[dict] = None
        self.active_hardware_state_instance_id: Optional[str] = None
        self.active_hardware_state_receipt_ns: Optional[int] = None
        self.hardware_sequences_by_instance: dict[str, int] = {}
        self.collision_requests_by_source: dict[str, tuple[int, int]] = {}
        self.latencies_ms: list[float] = []
        self.errors_rad: list[float] = []
        self.last_target: Optional[np.ndarray] = None
        self.gui_target: Optional[np.ndarray] = None
        self.gui_direction = "real_to_sim"
        self.gui_hardware_mode = "brake"
        self.last_gui_target_warning_at = 0.0
        self.suppressed_gui_target_warnings = 0
        self.target_subscription = self.create_subscription(
            Float64MultiArray, "/whole_arm/gui_targets", self.on_gui_target, 10
        )
        self.mode_subscription = self.create_subscription(
            String, "/whole_arm/gui_mode", self.on_gui_mode, 10
        )
        self.hardware_state_subscription = self.create_subscription(
            String, "/whole_arm/hardware_state", self.on_collision_hardware_state, 10
        )
        self.collision_request_subscription = self.create_subscription(
            String,
            COLLISION_GUARD_REQUEST_TOPIC,
            self.on_collision_guard_request,
            10,
        )
        self.collision_guard_worker = LatestCollisionGuardWorker(
            self.collision_guard_engine.evaluate,
            self.publish_collision_guard_result,
        )
        self.get_logger().info(
            ("MUJOCO_SESSION_POSE_MATCHED=YES" if self.pose_matched else
             "MUJOCO_SESSION_REFERENCE=SESSION_RELATIVE_BASELINE" if self.session_relative_baseline else
             "MUJOCO_SESSION_POSE_MATCHED=NO_NUMERIC_TEST_ONLY")
            + "; direct qpos mirror active; "
            f"model_sha256={self.model_hash}; guard_sha256={self.guard_hash}; "
            f"contract_sha256={self.collision_contract_hash}; "
            "collision_guard=FAIL_CLOSED_SINGLE_JOINT_LINF_GRID_AXIS_SWEEP_2P25DEG"
        )

    def on_joint_state(self, message: JointState) -> None:
        receipt_ns = time.monotonic_ns()
        try:
            if tuple(message.name) != JOINT_NAMES:
                raise ValueError(f"joint names/order must be exactly {JOINT_NAMES}")
            if len(message.position) != 6 or len(message.velocity) not in (0, 6):
                raise ValueError("JointState requires six positions and zero or six velocities")
            relative = np.asarray(message.position, dtype=float)
            velocity = np.zeros(6, dtype=float) if not message.velocity else np.asarray(message.velocity, dtype=float)
            if not np.all(np.isfinite(relative)) or not np.all(np.isfinite(velocity)):
                raise ValueError("JointState contains non-finite values")
            if self.gui_direction == "sim_to_real" and self.gui_target is not None:
                relative = self.gui_target.copy()
                velocity = np.zeros(6, dtype=float)
            target = self.session_pose + relative
            self.data.qpos[self.qpos_addresses] = target
            self.data.qvel[self.qvel_addresses] = velocity
            self.mujoco.mj_forward(self.model, self.data)
            actual = self.data.qpos[self.qpos_addresses].copy()
            error = actual - target
            max_error = float(np.max(np.abs(error)))
            self.errors_rad.append(max_error)
            self.errors_rad = self.errors_rad[-5000:]
            self.last_target = target
            self.sequence += 1

            ros_stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            now_ros_ns = self.get_clock().now().nanoseconds
            latency_ms = max(0.0, (now_ros_ns - ros_stamp_ns) / 1.0e6) if ros_stamp_ns else 0.0
            self.latencies_ms.append(latency_ms)
            self.latencies_ms = self.latencies_ms[-5000:]
            row = {
                "sequence": self.sequence,
                "ros_stamp_ns": ros_stamp_ns,
                "receipt_monotonic_ns": receipt_ns,
                "latency_ms": latency_ms,
                "max_error_rad": max_error,
            }
            for index, name in enumerate(JOINT_NAMES):
                row[f"{name}_target_qpos_rad"] = target[index]
                row[f"{name}_actual_qpos_rad"] = actual[index]
            self.capture_writer.writerow(row)
            if self.sequence % 50 == 0:
                self.capture_stream.flush()
                self.write_validation()
            self.status_publisher.publish(String(data=json.dumps(self.validation(), ensure_ascii=False)))
        except Exception as exc:
            self.rejected_messages += 1
            self.get_logger().warning(f"rejected /joint_states: {exc}")

    def on_collision_hardware_state(self, message: String) -> None:
        """Track the same fresh state authority that a GUI request must bind."""

        now_ns = time.monotonic_ns()
        try:
            raw = json.loads(message.data)
            state = hardware_state_for_collision_guard(raw, now_ns)
            source = state["state_instance_id"]
            if (
                self.active_hardware_state_instance_id is not None
                and source != self.active_hardware_state_instance_id
                and self.active_hardware_state_receipt_ns is not None
                and now_ns - self.active_hardware_state_receipt_ns
                <= HARDWARE_STATE_SOURCE_TAKEOVER_NS
            ):
                return
            previous = self.hardware_sequences_by_instance.get(source)
            if previous is not None and state["sequence"] <= previous:
                return
            if (
                source not in self.hardware_sequences_by_instance
                and len(self.hardware_sequences_by_instance) >= MAX_TRACKED_SOURCES
            ):
                oldest = next(iter(self.hardware_sequences_by_instance))
                self.hardware_sequences_by_instance.pop(oldest)
            self.hardware_sequences_by_instance[source] = state["sequence"]
            self.active_hardware_state_instance_id = source
            self.active_hardware_state_receipt_ns = now_ns
            self.latest_collision_hardware_state = state
        except (json.JSONDecodeError, TypeError, ValueError):
            # Hardware-state validity is already surfaced by the state node and
            # GUI.  Do not turn a high-rate invalid stream into warning spam.
            # Do, however, revoke the cached authorization immediately: using
            # the preceding healthy frame after any all-motor readiness failure
            # would make a newly faulted stationary axis invisible to the guard.
            self.latest_collision_hardware_state = None
            self.collision_guard_rejections += 1

    def on_collision_guard_request(self, message: String) -> None:
        now_ns = time.monotonic_ns()
        raw: object = {}
        try:
            raw = json.loads(message.data)
            source = raw.get("source_instance_id") if isinstance(raw, dict) else None
            previous = (
                self.collision_requests_by_source.get(source)
                if isinstance(source, str)
                else None
            )
            request = parse_collision_guard_request(
                raw,
                now_ns=now_ns,
                hardware_state=self.latest_collision_hardware_state,
                expected_session_pose_sha256=self.session_pose_sha256,
                previous_request=previous,
            )
            if (
                request.source_instance_id not in self.collision_requests_by_source
                and len(self.collision_requests_by_source) >= MAX_TRACKED_SOURCES
            ):
                oldest = next(iter(self.collision_requests_by_source))
                self.collision_requests_by_source.pop(oldest)
            self.collision_requests_by_source[request.source_instance_id] = (
                request.request_sequence,
                request.source_monotonic_ns,
            )
            self.collision_guard_worker.submit(request)
        except Exception as exc:
            self.collision_guard_rejections += 1
            reason = str(exc)
            self.publish_collision_guard_result(
                fail_closed_collision_result(raw, reason)
            )

    def publish_collision_guard_result(self, result: dict) -> None:
        with self.collision_result_publish_lock:
            if not self.collision_results_enabled:
                return
            self.collision_result_publisher.publish(
                String(data=json.dumps(result, ensure_ascii=False, separators=(",", ":")))
            )

    def on_gui_target(self, message: Float64MultiArray) -> None:
        try:
            target = np.asarray(message.data, dtype=float)
            if target.shape != (6,) or not np.all(np.isfinite(target)):
                raise ValueError("GUI目标必须是六个有限关节角")
            absolute_deg = np.degrees(self.session_pose + target)
            for value, limits in zip(absolute_deg, self.absolute_joint_limits_deg):
                if limits is not None and not (
                    limits[0] - 1.0e-9 <= value <= limits[1] + 1.0e-9
                ):
                    raise ValueError("GUI目标超出冻结3D模型机械关节范围")
            self.gui_target = target
        except Exception as exc:
            now = time.monotonic()
            if now - self.last_gui_target_warning_at >= GUI_TARGET_WARNING_INTERVAL_S:
                suffix = (
                    f"；已抑制重复警告{self.suppressed_gui_target_warnings}条"
                    if self.suppressed_gui_target_warnings else ""
                )
                self.get_logger().warning(f"拒绝GUI仿真目标：{exc}{suffix}")
                self.last_gui_target_warning_at = now
                self.suppressed_gui_target_warnings = 0
            else:
                self.suppressed_gui_target_warnings += 1

    def on_gui_mode(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            direction = value.get("direction")
            if direction not in {"real_to_sim", "sim_to_real"}:
                raise ValueError("方向模式不允许")
            self.gui_direction = direction
            self.gui_hardware_mode = str(value.get("hardware_mode", "brake"))
        except Exception as exc:
            self.get_logger().warning(f"拒绝GUI模式：{exc}")

    def validation(self) -> dict:
        return {
            "schema": "go-m8010-v15.30a-direct-qpos-mirror/1.0",
            "method": "DIRECT_QPOS_MIRROR",
            "model_path": str(self.model_path),
            "model_sha256": self.model_hash,
            "collision_guard_schema": COLLISION_GUARD_RESULT_SCHEMA,
            "collision_guard_ready": True,
            "collision_guard_sha256": self.guard_hash,
            "collision_contract_sha256": self.collision_contract_hash,
            "collision_guard_rejections": self.collision_guard_rejections,
            "collision_guard_max_step_deg": COLLISION_GUARD_MAX_STEP_DEG,
            "collision_guard_margin_deg": COLLISION_MARGIN_DEG,
            "collision_guard_margin_policy": COLLISION_MARGIN_POLICY,
            "collision_guard_joint_space_tube_radius_deg": (
                COLLISION_JOINT_SPACE_TUBE_RADIUS_DEG
            ),
            "collision_guard_hold_tracking_tolerance_deg": (
                COLLISION_HOLD_TRACKING_TOLERANCE_DEG
            ),
            "collision_guard_tube_grid_offsets": list(
                COLLISION_TUBE_GRID_OFFSETS
            ),
            "collision_guard_tube_grid_max_pose_count": (
                3 ** len(MUJOCO_JOINT_NAMES)
            ),
            "collision_guard_tube_axis_probe_step_deg": (
                COLLISION_TUBE_AXIS_PROBE_STEP_DEG
            ),
            "collision_guard_hardware_max_abs_velocity_rad_s": (
                COLLISION_HARDWARE_MAX_ABS_VELOCITY_RAD_S
            ),
            "collision_guard_absolute_joint_limits_deg": [
                list(bounds) for bounds in self.absolute_joint_limits_deg
            ],
            "collision_guard_ground_policy": (
                "UNSAFE_GROUND_COLLISION_NOT_SELF_COLLISION"
            ),
            "session_reference": "SESSION_REFERENCE_V1",
            "session_pose_sha256": self.session_pose_sha256,
            "mujoco_session_pose_matched": "YES" if self.pose_matched else "NO",
            "mujoco_pose_semantics": (
                "VISUALLY_MATCHED" if self.pose_matched else
                "SESSION_RELATIVE_BASELINE" if self.session_relative_baseline else
                "NUMERIC_TEST_ONLY"
            ),
            "session_pose_rad": self.session_pose.tolist(),
            "joint_state_messages": self.sequence,
            "rejected_messages": self.rejected_messages,
            "median_latency_ms": statistics.median(self.latencies_ms) if self.latencies_ms else None,
            "p95_latency_ms": percentile(self.latencies_ms, 0.95),
            "maximum_mirror_error_rad": max(self.errors_rad) if self.errors_rad else None,
            "direct_qpos_numeric_result": (
                "PASS" if self.errors_rad and max(self.errors_rad) <= 1.0e-12 else "NOT_EVALUATED"
            ),
            "cad_zero": "PENDING",
            "ros_zero": "PENDING",
            "pid_used": False,
            "actuator_tracking_used": False,
            "trajectory_simulation_used": False,
            "display_source": (
                "GUI_TARGET" if self.gui_direction == "sim_to_real" else "REAL_ENCODER"
            ),
        }

    def write_validation(self) -> None:
        self.validation_path.write_text(
            json.dumps(self.validation(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )

    def destroy_node(self) -> bool:
        try:
            # Serialize with publish_collision_guard_result so that, once this
            # flag is cleared, neither the worker nor a rejected request can
            # publish against a publisher being destroyed.
            with self.collision_result_publish_lock:
                self.collision_results_enabled = False
            try:
                self.collision_guard_worker.close()
            finally:
                try:
                    self.write_validation()
                finally:
                    if not self.capture_stream.closed:
                        self.capture_stream.flush()
                        self.capture_stream.close()
        finally:
            base_result = super().destroy_node()
        return base_result


def percentile(values: list[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def install_shutdown_handlers(stop_requested: threading.Event) -> dict[int, object]:
    """Keep ROS and GLFW teardown out of asynchronous signal handlers."""

    previous_handlers = {}

    def request_stop(_signum, _frame) -> None:
        stop_requested.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, request_stop)
    return previous_handlers


def restore_shutdown_handlers(previous_handlers: dict[int, object]) -> None:
    for signum, handler in previous_handlers.items():
        signal.signal(signum, handler)


def close_viewer_and_wait(viewer, timeout_sec: float = 2.0) -> None:
    """Close the passive viewer and wait for its native UI thread to release data.

    MuJoCo's Handle.close() only raises the native exit request.  The render
    thread can still hold the model and data for a short period afterwards, so
    returning from Python immediately can race interpreter teardown and crash.
    """
    viewer.close()
    simulate_reference = getattr(viewer, "_sim", None)
    if not callable(simulate_reference):
        return
    deadline = time.monotonic() + timeout_sec
    while simulate_reference() is not None:
        if time.monotonic() >= deadline:
            raise RuntimeError("MuJoCo passive viewer did not close within timeout")
        time.sleep(0.01)


def main(args=None) -> None:
    stop_requested = threading.Event()
    previous_handlers = install_shutdown_handlers(stop_requested)
    initialized = False
    node: Optional[WholeArmMujocoMirror] = None
    viewer = None
    try:
        # rclpy's default handler asynchronously shuts down its context while
        # MuJoCo's passive GLFW thread is still alive.  Own SIGINT/SIGTERM here
        # and perform every native teardown step serially on this main thread.
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        if stop_requested.is_set():
            return
        node = WholeArmMujocoMirror()
        if bool(node.get_parameter("use_viewer").value):
            import mujoco.viewer

            viewer = mujoco.viewer.launch_passive(node.model, node.data)
            while not stop_requested.is_set() and rclpy.ok() and viewer.is_running():
                rclpy.spin_once(node, timeout_sec=0.01)
                viewer.sync()
        else:
            while not stop_requested.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
    except ExternalShutdownException:
        pass
    finally:
        try:
            if viewer is not None:
                closing_viewer = viewer
                viewer = None
                close_viewer_and_wait(closing_viewer)
                del closing_viewer
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                try:
                    if initialized and rclpy.ok():
                        rclpy.shutdown()
                finally:
                    restore_shutdown_handlers(previous_handlers)


if __name__ == "__main__":
    main()

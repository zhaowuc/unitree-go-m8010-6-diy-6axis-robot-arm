"""Fail-closed whole-arm gravity mapping for the frozen production model.

The model/session anchor is software metadata.  It never changes a motor zero,
Flash, EEPROM, or the frozen MuJoCo XML.  Runtime MuJoCo ownership is isolated
from both GUI renderers by :class:`StaticGravityEvaluator`.
"""

from __future__ import annotations

import hashlib
import importlib
import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Mapping, Optional, Sequence

from .torque_semantics import (
    GO_GEAR_RATIO,
    J2A_ROTOR_TO_JOINT_SIGN,
    J2B_ROTOR_TO_JOINT_SIGN,
    joint_torque_to_rotor_torque,
    split_j2_joint_torque,
)


PRODUCTION_MODEL_SHA256 = (
    "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
)
GRAVITY_CONFIG_SHA256 = (
    "307469b8384fd35547327ba1d5f80aa440e6b9663406ab7c9d9469bea263335d"
)
GRAVITY_ANCHOR_SCHEMA = "go-m8010-gravity-model-anchor-v2/2.0"
MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
JOINT_STATE_NAME_ALIASES = {
    **{name: name for name in JOINT_NAMES},
    **{f"joint{index}": name for index, name in enumerate(JOINT_NAMES, start=1)},
}
GRAVITY_SCALE_LEVELS = (0.0, 0.25, 0.50, 0.75, 1.00)
FROZEN_MOTOR_SIGNS = {
    "J1": +1,
    "J2A": -1,
    "J2B": +1,
    "J3": +1,
    "J4": -1,
    "J5": +1,
    "J6": -1,
}


class GravityAnchorError(ValueError):
    """A bounded category for invalid or stale software pose authority."""


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def dynamics_only_xml(model_path: Path) -> str:
    """Derive an in-memory dynamics model without mesh/collision payloads.

    The frozen source XML remains the hash authority and is never modified.
    Every moving link in that model has an explicit ``inertial`` element, so
    zero-mass visual/collision geoms do not contribute to gravity or inverse
    dynamics. Removing them avoids loading the 1008-mesh, multi-gigabyte
    rendering/collision model into every control-side MuJoCo process.
    """

    try:
        root = ET.fromstring(Path(model_path).read_bytes())
    except (OSError, ET.ParseError) as exc:
        raise ValueError("frozen production MuJoCo XML cannot be parsed") from exc
    if root.tag != "mujoco":
        raise ValueError("frozen production model root must be mujoco")
    for tag in ("asset", "visual", "contact", "size"):
        child = root.find(tag)
        if child is not None:
            root.remove(child)
    for parent in root.iter():
        for child in list(parent):
            if child.tag in {"geom", "camera", "light"}:
                parent.remove(child)
    if any(root.iter("geom")) or root.find("asset") is not None:
        raise ValueError("dynamics-only MuJoCo derivation retained mesh geometry")
    return ET.tostring(root, encoding="unicode")


def _finite_vector(value: object, length: int, name: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise GravityAnchorError(f"{name} must contain {length} values")
    converted = tuple(float(item) for item in value)
    if not all(math.isfinite(item) for item in converted):
        raise GravityAnchorError(f"{name} must be finite")
    return converted


def _exact_motor_mapping(value: object, name: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping) or set(value) != set(MOTOR_NAMES):
        raise GravityAnchorError(f"{name} must contain all seven motors exactly")
    return value


def normalize_named_joint_positions(
    names: Sequence[str], positions: Sequence[float],
) -> tuple[float, ...]:
    """Extract the six logical joints by name without relying on array order."""

    if len(names) != len(positions):
        raise ValueError("joint state name/position lengths differ")
    observed = {}
    for name, value in zip(names, positions):
        canonical_name = JOINT_STATE_NAME_ALIASES.get(name)
        if canonical_name is not None:
            if canonical_name in observed:
                raise ValueError(f"duplicate joint state name: {canonical_name}")
            converted = float(value)
            if not math.isfinite(converted):
                raise ValueError(f"non-finite joint state: {name}")
            observed[canonical_name] = converted
    if set(observed) != set(JOINT_NAMES):
        raise ValueError("joint state does not contain J1..J6 exactly")
    return tuple(observed[name] for name in JOINT_NAMES)


@dataclass(frozen=True)
class GravityModelAnchorV2:
    model_sha256: str
    session_id: str
    state_instance_id: str
    created_utc: str
    gear_ratio: float
    motor_direction_sign: Mapping[str, int]
    motor_raw_reference_rad: Mapping[str, float]
    motor_encoder_branch: Mapping[str, int]
    logical_joint_reference_rad: tuple[float, ...]
    model_absolute_joint_rad: tuple[float, ...]

    @classmethod
    def from_mapping(cls, value: object) -> "GravityModelAnchorV2":
        required = {
            "schema",
            "model_sha256",
            "session_id",
            "state_instance_id",
            "created_utc",
            "gear_ratio",
            "motor_direction_sign",
            "motor_raw_reference_rad",
            "motor_encoder_branch",
            "logical_joint_reference_rad",
            "model_absolute_joint_rad",
        }
        if not isinstance(value, Mapping) or set(value) != required:
            raise GravityAnchorError("anchor schema fields do not match V2")
        if value.get("schema") != GRAVITY_ANCHOR_SCHEMA:
            raise GravityAnchorError("anchor schema identifier is invalid")
        model_sha256 = value.get("model_sha256")
        if model_sha256 != PRODUCTION_MODEL_SHA256:
            raise GravityAnchorError("anchor model hash is not the frozen model")
        session_id = value.get("session_id")
        state_instance_id = value.get("state_instance_id")
        if not isinstance(session_id, str) or not session_id:
            raise GravityAnchorError("anchor session_id is required")
        if not isinstance(state_instance_id, str) or not state_instance_id:
            raise GravityAnchorError("anchor state_instance_id is required")
        created_utc = value.get("created_utc")
        try:
            created = datetime.fromisoformat(str(created_utc).replace("Z", "+00:00"))
        except ValueError as exc:
            raise GravityAnchorError("anchor created_utc is invalid") from exc
        if created.tzinfo is None:
            raise GravityAnchorError("anchor created_utc must include a timezone")
        ratio = float(value.get("gear_ratio"))
        if not math.isfinite(ratio) or abs(ratio - GO_GEAR_RATIO) > 1.0e-12:
            raise GravityAnchorError("anchor gear ratio differs from frozen mapping")

        signs_raw = _exact_motor_mapping(
            value.get("motor_direction_sign"), "motor_direction_sign"
        )
        signs = {}
        for name in MOTOR_NAMES:
            sign = signs_raw[name]
            if type(sign) is not int or sign != FROZEN_MOTOR_SIGNS[name]:
                raise GravityAnchorError(f"anchor sign differs for {name}")
            signs[name] = sign

        raw_reference_input = _exact_motor_mapping(
            value.get("motor_raw_reference_rad"), "motor_raw_reference_rad"
        )
        raw_reference = {name: float(raw_reference_input[name]) for name in MOTOR_NAMES}
        if not all(math.isfinite(item) for item in raw_reference.values()):
            raise GravityAnchorError("motor raw references must be finite")

        branch_input = _exact_motor_mapping(
            value.get("motor_encoder_branch"), "motor_encoder_branch"
        )
        branches = {}
        for name in MOTOR_NAMES:
            branch = branch_input[name]
            if type(branch) is not int:
                raise GravityAnchorError(f"encoder branch for {name} must be an integer")
            branches[name] = branch

        return cls(
            model_sha256=model_sha256,
            session_id=session_id,
            state_instance_id=state_instance_id,
            created_utc=str(created_utc),
            gear_ratio=ratio,
            motor_direction_sign=signs,
            motor_raw_reference_rad=raw_reference,
            motor_encoder_branch=branches,
            logical_joint_reference_rad=_finite_vector(
                value.get("logical_joint_reference_rad"), 6,
                "logical_joint_reference_rad",
            ),
            model_absolute_joint_rad=_finite_vector(
                value.get("model_absolute_joint_rad"), 6,
                "model_absolute_joint_rad",
            ),
        )

    @classmethod
    def from_path(cls, path: Path) -> "GravityModelAnchorV2":
        try:
            value = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError) as exc:
            raise GravityAnchorError("anchor file cannot be read") from exc
        return cls.from_mapping(value)

    def matches_runtime(self, session_id: str, state_instance_id: str) -> bool:
        return bool(
            session_id == self.session_id
            and state_instance_id == self.state_instance_id
        )

    def model_q_from_actual(
        self, q_actual_rad: Sequence[float],
    ) -> tuple[float, ...]:
        actual = _finite_vector(q_actual_rad, 6, "q_actual_rad")
        return tuple(
            model_anchor + actual_value - logical_anchor
            for model_anchor, actual_value, logical_anchor in zip(
                self.model_absolute_joint_rad,
                actual,
                self.logical_joint_reference_rad,
            )
        )


def gravity_joint_to_rotor_commands(
    gravity_joint_nm: Sequence[float],
    *,
    gravity_scale: float,
    gear_ratio: float = GO_GEAR_RATIO,
) -> dict[str, float]:
    """Map six model joint torques to the five GO domains; J6 stays POS_VEL."""

    joint = _finite_vector(gravity_joint_nm, 6, "gravity_joint_nm")
    scale = float(gravity_scale)
    if not math.isfinite(scale) or not 0.0 <= scale <= 1.0:
        raise ValueError("gravity_scale must be finite and within [0, 1]")
    j2 = split_j2_joint_torque(scale * joint[1], gear_ratio=gear_ratio)
    return {
        "J1": joint_torque_to_rotor_torque(
            scale * joint[0], direction_sign=+1, gear_ratio=gear_ratio,
        ),
        "J2A": j2.tau_cmd_j2a_rotor_nm,
        "J2B": j2.tau_cmd_j2b_rotor_nm,
        "J3": joint_torque_to_rotor_torque(
            scale * joint[2], direction_sign=+1, gear_ratio=gear_ratio,
        ),
        "J4": joint_torque_to_rotor_torque(
            scale * joint[3], direction_sign=-1, gear_ratio=gear_ratio,
        ),
        "J5": joint_torque_to_rotor_torque(
            scale * joint[4], direction_sign=+1, gear_ratio=gear_ratio,
        ),
        # DM-G6220 remains in the validated POS_VEL path in V15.31A.
        "J6": 0.0,
    }


def gravity_joint_to_logical_rotor_feedforward(
    gravity_joint_nm: Sequence[float],
    *,
    gravity_scale: float,
    gear_ratio: float = GO_GEAR_RATIO,
) -> tuple[float, ...]:
    """Return the six-value GO command field in rotor-side N.m.

    ``feedforward_nm[J2]`` is the common logical contribution consumed by
    both J2 motors; the GO worker applies the frozen ``-1/+1`` motor signs.
    It therefore contains *half* of the logical J2 joint torque divided by
    the gear ratio.  J6 remains zero because it uses DM POS_VEL rather than a
    GO torque field.

    The vector stays in logical coordinates. J4's negative motor sign is
    applied once by the worker, not by both this function and the worker.
    """

    rotor = gravity_joint_to_rotor_commands(
        gravity_joint_nm,
        gravity_scale=gravity_scale,
        gear_ratio=gear_ratio,
    )
    if abs(rotor["J2A"] + rotor["J2B"]) > 1.0e-12:
        raise ValueError("J2 gravity split lost its opposite-sign invariant")
    return (
        rotor["J1"],
        rotor["J2B"],
        rotor["J3"],
        -rotor["J4"],
        rotor["J5"],
        0.0,
    )


class StaticGravityEvaluator:
    """Own a control-only MjModel/MjData pair and evaluate static qfrc_bias."""

    def __init__(self, model_path: Path, mujoco_module=None) -> None:
        self.model_path = Path(model_path)
        actual_hash = sha256_file(self.model_path)
        if actual_hash != PRODUCTION_MODEL_SHA256:
            raise ValueError("frozen production MuJoCo model hash mismatch")
        self._mujoco = (
            importlib.import_module("mujoco")
            if mujoco_module is None else mujoco_module
        )
        self.model = self._mujoco.MjModel.from_xml_string(
            dynamics_only_xml(self.model_path)
        )
        if self.model.nq != 6 or self.model.nv != 6:
            raise ValueError("production MuJoCo model must expose six coordinates")
        self.data = self._mujoco.MjData(self.model)

    def evaluate(self, model_absolute_q_rad: Sequence[float]) -> tuple[float, ...]:
        pose = _finite_vector(
            model_absolute_q_rad, 6, "model_absolute_q_rad"
        )
        self.model.opt.gravity[:] = (0.0, 0.0, -9.81)
        self.data.qpos[:] = pose
        self.data.qvel[:] = (0.0,) * 6
        self.data.qacc[:] = (0.0,) * 6
        self._mujoco.mj_forward(self.model, self.data)
        result = tuple(float(value) for value in self.data.qfrc_bias[:6])
        if not all(math.isfinite(value) for value in result):
            raise ValueError("MuJoCo returned non-finite gravity torque")
        return result

    def evaluate_inverse(
        self,
        model_absolute_q_rad: Sequence[float],
        velocity_rad_s: Sequence[float],
        acceleration_rad_s2: Sequence[float],
    ) -> tuple[float, ...]:
        """Return full inverse-dynamics joint demand for one exact path sample."""

        pose = _finite_vector(model_absolute_q_rad, 6, "model_absolute_q_rad")
        velocity = _finite_vector(velocity_rad_s, 6, "velocity_rad_s")
        acceleration = _finite_vector(
            acceleration_rad_s2, 6, "acceleration_rad_s2"
        )
        if not hasattr(self._mujoco, "mj_inverse"):
            raise ValueError("MuJoCo inverse dynamics is unavailable")
        self.model.opt.gravity[:] = (0.0, 0.0, -9.81)
        self.data.qpos[:] = pose
        self.data.qvel[:] = velocity
        self.data.qacc[:] = acceleration
        self._mujoco.mj_inverse(self.model, self.data)
        result = tuple(float(value) for value in self.data.qfrc_inverse[:6])
        if not all(math.isfinite(value) for value in result):
            raise ValueError("MuJoCo returned non-finite inverse torque")
        return result


class GravityScaleRamp:
    """Slew a dimensionless gravity scale without ever stepping the output.

    A fresh instance always starts at zero.  ``ramp_seconds`` is the minimum
    time for a full-scale 0 -> 1 (or 1 -> 0) transition; smaller changes use
    the same bounded rate.  Time reversal is rejected because accepting it
    could bypass the slew bound after a clock/source reset.
    """

    def __init__(
        self,
        ramp_seconds: float,
        initial_scale: float = 0.0,
        *,
        fixed_transition_duration: bool = False,
    ) -> None:
        duration = float(ramp_seconds)
        initial = float(initial_scale)
        if not math.isfinite(duration) or duration <= 0.0:
            raise ValueError("ramp_seconds must be finite and positive")
        if not math.isfinite(initial) or not 0.0 <= initial <= 1.0:
            raise ValueError("initial_scale must be within [0, 1]")
        self.ramp_seconds = duration
        self.fixed_transition_duration = bool(fixed_transition_duration)
        self.current_scale = initial
        self.target_scale = initial
        self._last_time_s: Optional[float] = None
        self._transition_started_s: Optional[float] = None
        self._transition_start_scale = initial

    def set_target(self, scale: float) -> None:
        target = float(scale)
        if not math.isfinite(target) or not 0.0 <= target <= 1.0:
            raise ValueError("target gravity scale must be within [0, 1]")
        if target != self.target_scale:
            self.target_scale = target
            if self.fixed_transition_duration:
                # The next step owns the timestamp so set_target remains a
                # pure validation/update call and cannot credit time between
                # a ROS parameter read and the control-cycle boundary.
                self._transition_started_s = None
                self._transition_start_scale = self.current_scale

    def step(self, now_s: float) -> float:
        now = float(now_s)
        if not math.isfinite(now) or now < 0.0:
            raise ValueError("now_s must be finite and non-negative")
        if self._last_time_s is None:
            self._last_time_s = now
            return self.current_scale
        elapsed = now - self._last_time_s
        if elapsed < 0.0:
            raise ValueError("gravity ramp time moved backwards")
        self._last_time_s = now
        if self.fixed_transition_duration:
            if self.current_scale == self.target_scale:
                self._transition_started_s = None
                self._transition_start_scale = self.current_scale
                return self.current_scale
            if self._transition_started_s is None:
                self._transition_started_s = now
                self._transition_start_scale = self.current_scale
                return self.current_scale
            transition_elapsed = now - self._transition_started_s
            fraction = min(1.0, transition_elapsed / self.ramp_seconds)
            self.current_scale = self._transition_start_scale + fraction * (
                self.target_scale - self._transition_start_scale
            )
            if fraction >= 1.0:
                self.current_scale = self.target_scale
                self._transition_started_s = None
                self._transition_start_scale = self.current_scale
            return self.current_scale
        maximum_change = elapsed / self.ramp_seconds
        difference = self.target_scale - self.current_scale
        if abs(difference) <= maximum_change:
            self.current_scale = self.target_scale
        else:
            self.current_scale += math.copysign(maximum_change, difference)
        return self.current_scale

    def force_zero(self, now_s: Optional[float] = None) -> None:
        """Fail closed immediately; enabling again still starts from zero."""

        self.current_scale = 0.0
        self.target_scale = 0.0
        self._transition_started_s = None
        self._transition_start_scale = 0.0
        if now_s is not None:
            now = float(now_s)
            if not math.isfinite(now) or now < 0.0:
                raise ValueError("now_s must be finite and non-negative")
            self._last_time_s = now


class RotorTorqueSlewLimiter:
    """Bound changes in named rotor-side N.m commands for all seven motors."""

    def __init__(self, maximum_slew_nm_per_s: float) -> None:
        slew = float(maximum_slew_nm_per_s)
        if not math.isfinite(slew) or slew <= 0.0:
            raise ValueError("maximum_slew_nm_per_s must be finite and positive")
        self.maximum_slew_nm_per_s = slew
        self.values = {name: 0.0 for name in MOTOR_NAMES}
        self._last_time_s: Optional[float] = None

    def step(self, targets: Mapping[str, float], now_s: float) -> dict[str, float]:
        if not isinstance(targets, Mapping) or set(targets) != set(MOTOR_NAMES):
            raise ValueError("rotor torque target must contain all seven motors")
        converted = {name: float(targets[name]) for name in MOTOR_NAMES}
        if not all(math.isfinite(value) for value in converted.values()):
            raise ValueError("rotor torque targets must be finite")
        now = float(now_s)
        if not math.isfinite(now) or now < 0.0:
            raise ValueError("now_s must be finite and non-negative")
        if self._last_time_s is None:
            self._last_time_s = now
            return dict(self.values)
        elapsed = now - self._last_time_s
        if elapsed < 0.0:
            raise ValueError("torque slew time moved backwards")
        self._last_time_s = now
        maximum_change = elapsed * self.maximum_slew_nm_per_s
        for name in MOTOR_NAMES:
            difference = converted[name] - self.values[name]
            if abs(difference) <= maximum_change:
                self.values[name] = converted[name]
            else:
                self.values[name] += math.copysign(maximum_change, difference)
        return dict(self.values)

    def force_zero(self, now_s: Optional[float] = None) -> dict[str, float]:
        self.values = {name: 0.0 for name in MOTOR_NAMES}
        if now_s is not None:
            now = float(now_s)
            if not math.isfinite(now) or now < 0.0:
                raise ValueError("now_s must be finite and non-negative")
            self._last_time_s = now
        return dict(self.values)


class GravityFeedforwardController:
    """Compose scale ramp, J2 split and per-rotor slew for command authority.

    This pure state machine is intentionally separate from ROS.  Disabling
    authority is fail-closed and immediately clears its output.  A normal
    operator-requested ramp-out is performed by keeping authority enabled and
    selecting target scale ``0.0``; only after the reported current scale is
    zero should the surrounding supervisor disable the authority.
    """

    def __init__(
        self,
        *,
        ramp_seconds: float,
        maximum_slew_nm_per_s: float,
    ) -> None:
        self.scale = GravityScaleRamp(ramp_seconds, initial_scale=0.0)
        self.rotor = RotorTorqueSlewLimiter(maximum_slew_nm_per_s)
        self._enabled = False

    def configure_fixed_stage_transition_ramp(
        self, ramp_seconds: float
    ) -> None:
        """Select fixed-duration stage ramps before empirical enablement."""

        if self._enabled or self.scale.current_scale != 0.0:
            raise ValueError("cannot reconfigure an active gravity ramp")
        self.scale = GravityScaleRamp(
            ramp_seconds,
            initial_scale=0.0,
            fixed_transition_duration=True,
        )

    @staticmethod
    def validate_scale_target(value: float) -> float:
        target = float(value)
        if not math.isfinite(target) or not any(
            abs(target - level) <= 1.0e-12 for level in GRAVITY_SCALE_LEVELS
        ):
            raise ValueError("gravity scale target must be 0/25/50/75/100 percent")
        return min(GRAVITY_SCALE_LEVELS, key=lambda level: abs(level - target))

    def step(
        self,
        gravity_joint_nm: Sequence[float],
        *,
        enabled: bool,
        target_scale: float,
        now_s: float,
    ) -> tuple[float, tuple[float, ...], dict[str, float]]:
        target = self.validate_scale_target(target_scale)
        if not enabled:
            self.scale.force_zero(now_s)
            physical = self.rotor.force_zero(now_s)
            self._enabled = False
            return 0.0, (0.0,) * 6, physical
        if not self._enabled:
            # Enabling establishes a fresh ramp/slew epoch at exactly zero.
            # Time spent disabled must never be credited toward either bound.
            self.scale.force_zero(now_s)
            self.rotor.force_zero(now_s)
            self._enabled = True
        self.scale.set_target(target)
        current_scale = self.scale.step(now_s)
        physical_target = gravity_joint_to_rotor_commands(
            gravity_joint_nm,
            gravity_scale=current_scale,
        )
        physical = self.rotor.step(physical_target, now_s)
        logical = (
            physical["J1"],
            physical["J2B"],
            physical["J3"],
            physical["J4"],
            physical["J5"],
            0.0,
        )
        if abs(physical["J2A"] + physical["J2B"]) > 1.0e-12:
            raise ValueError("J2 slew limiter lost its equal/opposite split")
        if not all(math.isfinite(value) for value in logical):
            raise ValueError("gravity feedforward contains a non-finite value")
        return current_scale, logical, physical

    def force_zero(
        self, now_s: Optional[float] = None,
    ) -> tuple[float, tuple[float, ...], dict[str, float]]:
        self.scale.force_zero(now_s)
        physical = self.rotor.force_zero(now_s)
        self._enabled = False
        return 0.0, (0.0,) * 6, physical

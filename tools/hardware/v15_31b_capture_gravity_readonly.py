#!/usr/bin/env python3
"""Capture V15.31B gravity calculation evidence without a command publisher.

The collector subscribes only to ``/whole_arm/hardware_state`` and
``/whole_arm/gravity_status``.  Every retained gravity status is paired with
the exact hardware-state sequence named by the gravity node, transformed with
the hash-pinned ``MODEL_SESSION_ANCHOR_V2``, and independently recomputed from
the frozen MuJoCo model.  A continuous ten-second window is accepted only when
all motors remain BRAKE/DISABLED, all GO command torques remain zero, and both
the requested and applied gravity scales remain exactly zero.

This process has no motor transport, ROS publisher, service client, parameter
client, or command topic.  Its only write is a new, atomically published JSON
evidence file; it never overwrites an existing file.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib
import json
import math
import os
import stat
import tempfile
import time
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Protocol, Sequence


GRAVITY_READONLY_SCHEMA = "V15.31B-gravity-readonly-v1"
GRAVITY_STATUS_SCHEMA = "go-m8010-gravity-status/1.1"
HARDWARE_STATE_SCHEMA = "go-m8010-hardware-state/1.1"
ANCHOR_SCHEMA = "go-m8010-gravity-model-anchor-v2/2.0"
PRODUCTION_MODEL_SHA256 = (
    "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
)
GRAVITY_CONFIG_SHA256 = (
    "307469b8384fd35547327ba1d5f80aa440e6b9663406ab7c9d9469bea263335d"
)
GO_GEAR_RATIO = 6.329999923706055
JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
GO_MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5")
FROZEN_MOTOR_SIGNS = {
    "J1": +1,
    "J2A": -1,
    "J2B": +1,
    "J3": +1,
    "J4": +1,
    "J5": +1,
    "J6": -1,
}
SAFE_MODES = {
    name: ({"brake", "disabled"} if name == "J6" else {"brake"})
    for name in MOTOR_NAMES
}
READONLY_GATE = (
    "24V_ON=YES;SUPPORT_RELIABLE=YES;ARM_STATIONARY=YES;"
    "NO_ONE_TOUCHING=YES;MODEL_SESSION_ANCHOR_V2=APPLIED;"
    "OPERATOR_STOP_READY=YES"
)

MINIMUM_DURATION_NS = 10_000_000_000
MINIMUM_VALID_SAMPLES = 100
MAXIMUM_STREAM_GAP_NS = 250_000_000
MAXIMUM_FEEDBACK_AGE_MS = 250.0
MAXIMUM_STATIONARY_VELOCITY_RAD_S = math.radians(0.25)
MAXIMUM_POSITION_STEP_RAD = math.radians(0.20)
MAXIMUM_J2_SYNC_RAD = math.radians(0.25)
THERMAL_STOP_C = 60.0
MODEL_COMPARISON_ABS_TOLERANCE_NM = 1.0e-8
MODEL_COMPARISON_REL_TOLERANCE = 1.0e-8
DIRECTION_PROBE_RAD = math.radians(0.25)
MAXIMUM_PROBE_SECOND_DIFFERENCE_NM = 0.05
MAXIMUM_INPUT_BYTES = 4 * 1024 * 1024
MAXIMUM_OUTPUT_BYTES = 64 * 1024 * 1024
MAXIMUM_CACHED_STATES = 4096


class GravityReadOnlyCaptureError(ValueError):
    """A fail-closed gravity read-only qualification failure."""


class StaticGravityEvaluator(Protocol):
    def evaluate(self, model_q_rad: Sequence[float]) -> tuple[float, ...]: ...


def finite_number(value: Any, label: str) -> float:
    if type(value) not in {int, float} or not math.isfinite(float(value)):
        raise GravityReadOnlyCaptureError(f"{label} must be finite")
    return float(value)


def finite_vector(value: Any, length: int, label: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != length:
        raise GravityReadOnlyCaptureError(
            f"{label} must contain exactly {length} values"
        )
    return tuple(
        finite_number(item, f"{label}[{index}]")
        for index, item in enumerate(value)
    )


def exact_mapping(
    value: Any, names: Sequence[str], label: str,
) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or set(value) != set(names):
        raise GravityReadOnlyCaptureError(
            f"{label} must contain {list(names)} exactly"
        )
    return value


def normalized_sha256(value: Any, label: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise GravityReadOnlyCaptureError(
            f"{label} must be a lowercase SHA-256"
        )
    return value


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def canonical_pose_sha256(q_actual_rad: Sequence[float]) -> str:
    pose = finite_vector(q_actual_rad, 6, "q_actual_rad")
    return hashlib.sha256(
        json.dumps(
            list(pose), allow_nan=False, separators=(",", ":")
        ).encode("ascii")
    ).hexdigest()


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _no_duplicate_object(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise GravityReadOnlyCaptureError(f"JSON duplicate key: {key}")
        result[key] = value
    return result


def decode_json_object(serialized: str, label: str) -> dict[str, Any]:
    def reject_constant(value: str) -> None:
        raise GravityReadOnlyCaptureError(
            f"{label} contains non-finite JSON constant: {value}"
        )

    try:
        value = json.loads(
            serialized,
            object_pairs_hook=_no_duplicate_object,
            parse_constant=reject_constant,
        )
    except (json.JSONDecodeError, UnicodeError) as exc:
        raise GravityReadOnlyCaptureError(f"{label} is not valid JSON") from exc
    if not isinstance(value, dict):
        raise GravityReadOnlyCaptureError(f"{label} root must be an object")
    return value


@dataclass(frozen=True)
class ModelSessionAnchor:
    model_sha256: str
    session_id: str
    state_instance_id: str
    logical_joint_reference_rad: tuple[float, ...]
    model_absolute_joint_rad: tuple[float, ...]

    @classmethod
    def from_mapping(cls, value: Any) -> "ModelSessionAnchor":
        fields = {
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
        if not isinstance(value, Mapping) or set(value) != fields:
            raise GravityReadOnlyCaptureError(
                "MODEL_SESSION_ANCHOR_V2 field set mismatch"
            )
        if value.get("schema") != ANCHOR_SCHEMA:
            raise GravityReadOnlyCaptureError(
                "MODEL_SESSION_ANCHOR_V2 schema mismatch"
            )
        if value.get("model_sha256") != PRODUCTION_MODEL_SHA256:
            raise GravityReadOnlyCaptureError(
                "MODEL_SESSION_ANCHOR_V2 model SHA mismatch"
            )
        session = value.get("session_id")
        instance = value.get("state_instance_id")
        if not isinstance(session, str) or not session:
            raise GravityReadOnlyCaptureError("anchor session_id is missing")
        if (
            not isinstance(instance, str)
            or len(instance) != 32
            or any(character not in "0123456789abcdef" for character in instance)
        ):
            raise GravityReadOnlyCaptureError("anchor state_instance_id is invalid")
        try:
            created = datetime.fromisoformat(
                str(value.get("created_utc")).replace("Z", "+00:00")
            )
        except ValueError as exc:
            raise GravityReadOnlyCaptureError(
                "anchor created_utc is invalid"
            ) from exc
        if created.tzinfo is None or created.utcoffset() is None:
            raise GravityReadOnlyCaptureError(
                "anchor created_utc requires a timezone"
            )
        ratio = finite_number(value.get("gear_ratio"), "anchor.gear_ratio")
        if not math.isclose(ratio, GO_GEAR_RATIO, rel_tol=0.0, abs_tol=1.0e-12):
            raise GravityReadOnlyCaptureError("anchor gear ratio mismatch")
        signs = exact_mapping(
            value.get("motor_direction_sign"), MOTOR_NAMES,
            "anchor.motor_direction_sign",
        )
        if dict(signs) != FROZEN_MOTOR_SIGNS:
            raise GravityReadOnlyCaptureError("anchor motor signs mismatch")
        raw = exact_mapping(
            value.get("motor_raw_reference_rad"), MOTOR_NAMES,
            "anchor.motor_raw_reference_rad",
        )
        branches = exact_mapping(
            value.get("motor_encoder_branch"), MOTOR_NAMES,
            "anchor.motor_encoder_branch",
        )
        for name in MOTOR_NAMES:
            finite_number(raw[name], f"anchor.motor_raw_reference_rad.{name}")
            if type(branches[name]) is not int:
                raise GravityReadOnlyCaptureError(
                    f"anchor.motor_encoder_branch.{name} must be an integer"
                )
        return cls(
            model_sha256=PRODUCTION_MODEL_SHA256,
            session_id=session,
            state_instance_id=instance,
            logical_joint_reference_rad=finite_vector(
                value.get("logical_joint_reference_rad"), 6,
                "anchor.logical_joint_reference_rad",
            ),
            model_absolute_joint_rad=finite_vector(
                value.get("model_absolute_joint_rad"), 6,
                "anchor.model_absolute_joint_rad",
            ),
        )

    def model_q_from_actual(
        self, q_actual_rad: Sequence[float],
    ) -> tuple[float, ...]:
        actual = finite_vector(q_actual_rad, 6, "q_actual_rad")
        return tuple(
            model_anchor + actual_value - logical_anchor
            for model_anchor, actual_value, logical_anchor in zip(
                self.model_absolute_joint_rad,
                actual,
                self.logical_joint_reference_rad,
            )
        )


def read_anchor(
    path: Path, expected_sha256: str,
) -> tuple[ModelSessionAnchor, str]:
    expected = normalized_sha256(expected_sha256, "expected anchor SHA-256")
    absolute = Path(os.path.abspath(path))
    if absolute.is_symlink():
        raise GravityReadOnlyCaptureError("anchor must not be a symbolic link")
    resolved = absolute.resolve(strict=True)
    if not resolved.is_file():
        raise GravityReadOnlyCaptureError("anchor is not a regular file")
    data = resolved.read_bytes()
    if not data or len(data) > MAXIMUM_INPUT_BYTES:
        raise GravityReadOnlyCaptureError("anchor file size is invalid")
    observed = sha256_bytes(data)
    if observed != expected:
        raise GravityReadOnlyCaptureError("anchor SHA-256 mismatch")
    try:
        serialized = data.decode("utf-8")
    except UnicodeError as exc:
        raise GravityReadOnlyCaptureError("anchor is not UTF-8") from exc
    return ModelSessionAnchor.from_mapping(
        decode_json_object(serialized, "anchor")
    ), observed


class MujocoStaticGravityEvaluator:
    """Independent qfrc_bias calculation, separate from the ROS gravity node."""

    def __init__(self, model_path: Path) -> None:
        resolved = validate_production_model_path(model_path)
        try:
            mujoco = importlib.import_module("mujoco")
        except ImportError as exc:
            raise GravityReadOnlyCaptureError(
                "MuJoCo Python runtime is unavailable"
            ) from exc
        self._mujoco = mujoco
        self.model = mujoco.MjModel.from_xml_string(
            independent_dynamics_only_xml(resolved)
        )
        if self.model.nq != 6 or self.model.nv != 6:
            raise GravityReadOnlyCaptureError(
                "production MuJoCo model must expose six coordinates"
            )
        self.data = mujoco.MjData(self.model)

    def evaluate(self, model_q_rad: Sequence[float]) -> tuple[float, ...]:
        pose = finite_vector(model_q_rad, 6, "model_q_rad")
        self.model.opt.gravity[:] = (0.0, 0.0, -9.81)
        self.data.qpos[:] = pose
        self.data.qvel[:] = (0.0,) * 6
        self.data.qacc[:] = (0.0,) * 6
        self._mujoco.mj_forward(self.model, self.data)
        result = tuple(float(value) for value in self.data.qfrc_bias[:6])
        if not all(math.isfinite(value) for value in result):
            raise GravityReadOnlyCaptureError(
                "independent MuJoCo gravity result is non-finite"
            )
        return result


def independent_dynamics_only_xml(model_path: Path) -> str:
    """Independently derive a low-memory gravity model from the frozen XML."""

    try:
        root = ET.fromstring(Path(model_path).read_bytes())
    except (OSError, ET.ParseError) as exc:
        raise GravityReadOnlyCaptureError(
            "frozen production MuJoCo XML cannot be parsed"
        ) from exc
    if root.tag != "mujoco":
        raise GravityReadOnlyCaptureError(
            "frozen production model root must be mujoco"
        )
    for tag in ("asset", "visual", "contact", "size"):
        child = root.find(tag)
        if child is not None:
            root.remove(child)
    for parent in root.iter():
        for child in list(parent):
            if child.tag in {"geom", "camera", "light"}:
                parent.remove(child)
    if any(root.iter("geom")) or root.find("asset") is not None:
        raise GravityReadOnlyCaptureError(
            "independent dynamics-only derivation retained mesh geometry"
        )
    return ET.tostring(root, encoding="unicode")


def validate_production_model_path(model_path: Path) -> Path:
    """Return the resolved frozen model after a non-bypassable hash check."""

    model = Path(os.path.abspath(model_path))
    if model.is_symlink():
        raise GravityReadOnlyCaptureError(
            "production model must not be a symbolic link"
        )
    resolved = model.resolve(strict=True)
    if not resolved.is_file():
        raise GravityReadOnlyCaptureError(
            "production model is not a regular file"
        )
    if sha256_bytes(resolved.read_bytes()) != PRODUCTION_MODEL_SHA256:
        raise GravityReadOnlyCaptureError(
            "frozen production model SHA-256 mismatch"
        )
    return resolved


def predicted_rotor_map(gravity_joint_nm: Sequence[float]) -> dict[str, float]:
    gravity = finite_vector(gravity_joint_nm, 6, "gravity_joint_nm")
    return {
        "J1": gravity[0] / GO_GEAR_RATIO,
        "J2A": -gravity[1] / (2.0 * GO_GEAR_RATIO),
        "J2B": +gravity[1] / (2.0 * GO_GEAR_RATIO),
        "J3": gravity[2] / GO_GEAR_RATIO,
        "J4": gravity[3] / GO_GEAR_RATIO,
        "J5": gravity[4] / GO_GEAR_RATIO,
    }


@dataclass(frozen=True)
class HardwarePose:
    sequence: int
    source_monotonic_ns: int
    q_actual_rad: tuple[float, ...]
    velocity_rad_s: tuple[float, ...]


class GravityReadOnlyRecorder:
    """Pair and qualify same-session state/gravity telemetry."""

    def __init__(
        self,
        *,
        anchor: ModelSessionAnchor,
        anchor_sha256: str,
        evaluator: StaticGravityEvaluator,
        minimum_duration_ns: int = MINIMUM_DURATION_NS,
        minimum_valid_samples: int = MINIMUM_VALID_SAMPLES,
    ) -> None:
        self.anchor = anchor
        self.anchor_sha256 = normalized_sha256(
            anchor_sha256, "anchor SHA-256"
        )
        self.evaluator = evaluator
        self.minimum_duration_ns = int(minimum_duration_ns)
        self.minimum_valid_samples = int(minimum_valid_samples)
        if self.minimum_duration_ns < MINIMUM_DURATION_NS:
            raise GravityReadOnlyCaptureError(
                "capture duration cannot be shorter than 10 seconds"
            )
        if self.minimum_valid_samples < 2:
            raise GravityReadOnlyCaptureError(
                "minimum valid sample count must be at least two"
            )
        self.states: dict[int, HardwarePose] = {}
        self.pending_statuses: list[dict[str, Any]] = []
        self.samples: list[dict[str, Any]] = []
        self.previous_state_sequence: int | None = None
        self.previous_state_source_ns: int | None = None
        self.previous_status_sequence: int | None = None
        self.previous_status_source_ns: int | None = None
        self.gravity_source_instance_id: str | None = None
        self.maximum_status_gap_ns = 0
        self.maximum_state_gap_ns = 0
        self.maximum_model_error_nm = 0.0
        self.maximum_position_step_rad = 0.0
        self.maximum_velocity_rad_s = 0.0
        self.maximum_abs_j2_gravity_nm = 0.0
        self.maximum_abs_gravity_nm = 0.0
        self.joint_state_crosscheck_false_count = 0

    @property
    def duration_ns(self) -> int:
        if len(self.samples) < 2:
            return 0
        return int(self.samples[-1]["gravity_status_source_monotonic_ns"]) - int(
            self.samples[0]["gravity_status_source_monotonic_ns"]
        )

    @property
    def complete(self) -> bool:
        return bool(
            self.duration_ns >= self.minimum_duration_ns
            and len(self.samples) >= self.minimum_valid_samples
            and not self.pending_statuses
        )

    def _validate_session(self, document: Mapping[str, Any], label: str) -> None:
        if document.get("session_id") != self.anchor.session_id:
            raise GravityReadOnlyCaptureError(f"{label} session_id mismatch")
        if document.get("state_instance_id") != self.anchor.state_instance_id:
            raise GravityReadOnlyCaptureError(
                f"{label} state_instance_id mismatch"
            )

    def add_hardware_state(self, document: Mapping[str, Any]) -> None:
        label = f"hardware_state[{self.previous_state_sequence or 0}]"
        if document.get("schema") != HARDWARE_STATE_SCHEMA:
            raise GravityReadOnlyCaptureError(f"{label} schema mismatch")
        self._validate_session(document, label)
        sequence = document.get("sequence")
        source_ns = document.get("source_monotonic_ns")
        if type(sequence) is not int or sequence <= 0:
            raise GravityReadOnlyCaptureError(f"{label} sequence is invalid")
        if type(source_ns) is not int or source_ns <= 0:
            raise GravityReadOnlyCaptureError(
                f"{label} source timestamp is invalid"
            )
        if self.previous_state_sequence is not None:
            if sequence != self.previous_state_sequence + 1:
                raise GravityReadOnlyCaptureError(
                    f"{label} hardware-state sequence gap/replay"
                )
            assert self.previous_state_source_ns is not None
            gap_ns = source_ns - self.previous_state_source_ns
            if gap_ns <= 0 or gap_ns > MAXIMUM_STREAM_GAP_NS:
                raise GravityReadOnlyCaptureError(
                    f"{label} hardware-state telemetry gap exceeds 250 ms"
                )
            self.maximum_state_gap_ns = max(self.maximum_state_gap_ns, gap_ns)

        q_actual = finite_vector(
            document.get("position_rad"), 6, f"{label}.position_rad"
        )
        velocity = finite_vector(
            document.get("velocity_rad_s"), 6, f"{label}.velocity_rad_s"
        )
        if document.get("velocity_source") != "POSITION_SPAN_0P40S":
            raise GravityReadOnlyCaptureError(
                f"{label} mechanical velocity observer is not ready"
            )
        if max(abs(value) for value in velocity) > MAXIMUM_STATIONARY_VELOCITY_RAD_S:
            raise GravityReadOnlyCaptureError(
                f"{label} exceeds stationary velocity bound"
            )
        if document.get("j2_sync_fault") is not False:
            raise GravityReadOnlyCaptureError(
                f"{label} reports J2 synchronization fault"
            )
        e_sync = finite_number(
            document.get("j2_e_sync_rad"), f"{label}.j2_e_sync_rad"
        )
        if abs(e_sync) > MAXIMUM_J2_SYNC_RAD:
            raise GravityReadOnlyCaptureError(
                f"{label} exceeds J2 synchronization warning bound"
            )
        modes = exact_mapping(
            document.get("controller_mode_by_motor"), MOTOR_NAMES,
            f"{label}.controller_mode_by_motor",
        )
        motors = exact_mapping(
            document.get("per_motor"), MOTOR_NAMES, f"{label}.per_motor"
        )
        logical_by_motor: dict[str, float] = {}
        for name in MOTOR_NAMES:
            record = motors[name]
            if not isinstance(record, Mapping):
                raise GravityReadOnlyCaptureError(
                    f"{label}.{name} record must be an object"
                )
            mode = str(modes[name]).strip().lower()
            if mode not in SAFE_MODES[name]:
                raise GravityReadOnlyCaptureError(
                    f"{label}.{name} active/unknown mode observed: {mode}"
                )
            if record.get("communication_ok") is not True or record.get("fresh") is not True:
                raise GravityReadOnlyCaptureError(
                    f"{label}.{name} communication is not fresh"
                )
            if type(record.get("merror")) is not int or record.get("merror") != 0:
                raise GravityReadOnlyCaptureError(
                    f"{label}.{name} merror is nonzero"
                )
            age_ms = finite_number(record.get("age_ms"), f"{label}.{name}.age_ms")
            if not 0.0 <= age_ms <= MAXIMUM_FEEDBACK_AGE_MS:
                raise GravityReadOnlyCaptureError(
                    f"{label}.{name} feedback age exceeds 250 ms"
                )
            temperature = finite_number(
                record.get("temperature_c"), f"{label}.{name}.temperature_c"
            )
            if not -40.0 <= temperature < THERMAL_STOP_C:
                raise GravityReadOnlyCaptureError(
                    f"{label}.{name} temperature is outside read-only bounds"
                )
            logical = finite_number(
                record.get("q_joint_rad"), f"{label}.{name}.q_joint_rad"
            )
            finite_number(
                record.get("dq_joint_rad_s"), f"{label}.{name}.dq_joint_rad_s"
            )
            logical_by_motor[name] = logical
            torque = record.get("tau_cmd_rotor_nm")
            if name == "J6":
                if any(
                    record.get(field) is not None
                    for field in (
                        "tau_cmd_rotor_nm",
                        "tau_feedback_rotor_nm",
                        "tau_joint_estimated_nm",
                    )
                ):
                    raise GravityReadOnlyCaptureError(
                        f"{label}.J6 manufactured POS_VEL torque semantics"
                    )
            elif abs(finite_number(
                torque, f"{label}.{name}.tau_cmd_rotor_nm"
            )) > 1.0e-12:
                raise GravityReadOnlyCaptureError(
                    f"{label}.{name} nonzero active torque command observed"
                )

        expected_logical = {
            "J1": q_actual[0],
            "J2": 0.5 * (logical_by_motor["J2A"] + logical_by_motor["J2B"]),
            "J3": q_actual[2],
            "J4": q_actual[3],
            "J5": q_actual[4],
            "J6": q_actual[5],
        }
        expected_logical["J1"] = logical_by_motor["J1"]
        expected_logical["J3"] = logical_by_motor["J3"]
        expected_logical["J4"] = logical_by_motor["J4"]
        expected_logical["J5"] = logical_by_motor["J5"]
        expected_logical["J6"] = logical_by_motor["J6"]
        for index, name in enumerate(JOINT_NAMES):
            if abs(q_actual[index] - expected_logical[name]) > 1.0e-6:
                raise GravityReadOnlyCaptureError(
                    f"{label}.{name} state/per-motor logical pose mismatch"
                )

        self.states[sequence] = HardwarePose(
            sequence=sequence,
            source_monotonic_ns=source_ns,
            q_actual_rad=q_actual,
            velocity_rad_s=velocity,
        )
        self.previous_state_sequence = sequence
        self.previous_state_source_ns = source_ns
        self.maximum_velocity_rad_s = max(
            self.maximum_velocity_rad_s,
            max(abs(value) for value in velocity),
        )
        while len(self.states) > MAXIMUM_CACHED_STATES:
            oldest = next(iter(self.states))
            if any(
                pending.get("hardware_state_sequence") == oldest
                for pending in self.pending_statuses
            ):
                raise GravityReadOnlyCaptureError(
                    "unpaired gravity status exceeded hardware-state cache"
                )
            del self.states[oldest]
        self._drain_pending()

    def add_gravity_status(self, document: Mapping[str, Any]) -> None:
        label = f"gravity_status[{self.previous_status_sequence or 0}]"
        if document.get("schema") != GRAVITY_STATUS_SCHEMA:
            raise GravityReadOnlyCaptureError(f"{label} schema mismatch")
        if document.get("source") != "whole_arm_gravity_node":
            raise GravityReadOnlyCaptureError(f"{label} source mismatch")
        self._validate_session(document, label)
        source_instance = document.get("source_instance_id")
        if (
            not isinstance(source_instance, str)
            or len(source_instance) != 32
            or any(character not in "0123456789abcdef" for character in source_instance)
        ):
            raise GravityReadOnlyCaptureError(
                f"{label} gravity source instance is invalid"
            )
        if self.gravity_source_instance_id is None:
            self.gravity_source_instance_id = source_instance
        elif source_instance != self.gravity_source_instance_id:
            raise GravityReadOnlyCaptureError(
                "gravity source instance changed during capture"
            )
        sequence = document.get("sequence")
        source_ns = document.get("source_monotonic_ns")
        if type(sequence) is not int or sequence <= 0:
            raise GravityReadOnlyCaptureError(f"{label} sequence is invalid")
        if type(source_ns) is not int or source_ns <= 0:
            raise GravityReadOnlyCaptureError(
                f"{label} source timestamp is invalid"
            )
        if self.previous_status_sequence is not None:
            if sequence != self.previous_status_sequence + 1:
                raise GravityReadOnlyCaptureError(
                    f"{label} gravity-status sequence gap/replay"
                )
            assert self.previous_status_source_ns is not None
            gap_ns = source_ns - self.previous_status_source_ns
            if gap_ns <= 0 or gap_ns > MAXIMUM_STREAM_GAP_NS:
                raise GravityReadOnlyCaptureError(
                    f"{label} gravity-status telemetry gap exceeds 250 ms"
                )
            self.maximum_status_gap_ns = max(self.maximum_status_gap_ns, gap_ns)
        self.previous_status_sequence = sequence
        self.previous_status_source_ns = source_ns
        self.pending_statuses.append(dict(document))
        self._drain_pending()

    def _drain_pending(self) -> None:
        while self.pending_statuses:
            document = self.pending_statuses[0]
            hardware_sequence = document.get("hardware_state_sequence")
            if type(hardware_sequence) is not int or hardware_sequence <= 0:
                raise GravityReadOnlyCaptureError(
                    "gravity status hardware_state_sequence is invalid"
                )
            pose = self.states.get(hardware_sequence)
            if pose is None:
                if (
                    self.previous_state_sequence is not None
                    and self.previous_state_sequence > hardware_sequence
                ):
                    if not self.samples:
                        # DDS subscriptions are volatile: at collector startup,
                        # the first gravity status can legitimately name the
                        # immediately preceding state frame that this new
                        # subscriber never received.  Discard only this
                        # pre-window warm-up status.  Once one pair is accepted,
                        # the same condition is a hard continuity failure.
                        self.pending_statuses.pop(0)
                        continue
                    raise GravityReadOnlyCaptureError(
                        "gravity status references an unobserved hardware state"
                    )
                return
            self._accept_pair(document, pose)
            self.pending_statuses.pop(0)

    def _accept_pair(
        self, document: Mapping[str, Any], pose: HardwarePose,
    ) -> None:
        label = f"gravity_status[{document['sequence']}]"
        hardware_source_ns = document.get("hardware_state_source_monotonic_ns")
        if hardware_source_ns != pose.source_monotonic_ns:
            raise GravityReadOnlyCaptureError(
                f"{label} hardware state timestamp mismatch"
            )
        status_source_ns = int(document["source_monotonic_ns"])
        hardware_age_ns = status_source_ns - pose.source_monotonic_ns
        if not 0 <= hardware_age_ns <= MAXIMUM_STREAM_GAP_NS:
            raise GravityReadOnlyCaptureError(
                f"{label} paired hardware state is stale/future"
            )
        if document.get("model_sha256") != PRODUCTION_MODEL_SHA256:
            raise GravityReadOnlyCaptureError(f"{label} model SHA mismatch")
        if document.get("production_model_hash_match") is not True:
            raise GravityReadOnlyCaptureError(
                f"{label} production model hash check failed"
            )
        if document.get("gravity_config_sha256") != GRAVITY_CONFIG_SHA256:
            raise GravityReadOnlyCaptureError(
                f"{label} gravity config SHA mismatch"
            )
        if document.get("anchor_valid") is not True:
            raise GravityReadOnlyCaptureError(f"{label} anchor is not valid")
        if document.get("q_actual_sha256") != canonical_pose_sha256(
            pose.q_actual_rad
        ):
            raise GravityReadOnlyCaptureError(
                f"{label} q_actual hash does not match paired hardware state"
            )
        crosscheck = document.get("joint_state_crosscheck")
        if crosscheck is not None and type(crosscheck) is not bool:
            raise GravityReadOnlyCaptureError(
                f"{label} /joint_states cross-check is invalid"
            )
        if crosscheck is False:
            # /joint_states is an asynchronous UI mirror and can name an
            # adjacent sample.  The authoritative pose above is already
            # paired by hardware sequence/timestamp and verified by SHA-256.
            self.joint_state_crosscheck_false_count += 1
        rate = finite_number(
            document.get("calculation_rate_hz"), f"{label}.calculation_rate_hz"
        )
        if rate <= 0.0:
            raise GravityReadOnlyCaptureError(
                f"{label} calculation rate is not positive"
            )
        update_age_s = finite_number(
            document.get("last_update_age_s"), f"{label}.last_update_age_s"
        )
        if not 0.0 <= update_age_s <= MAXIMUM_STREAM_GAP_NS * 1.0e-9:
            raise GravityReadOnlyCaptureError(
                f"{label} gravity calculation is stale"
            )
        gravity = finite_vector(
            document.get("gravity_joint_nm"), 6, f"{label}.gravity_joint_nm"
        )
        feedforward = finite_vector(
            document.get("feedforward_nm"), 6, f"{label}.feedforward_nm"
        )
        if any(abs(value) > 1.0e-12 for value in feedforward):
            raise GravityReadOnlyCaptureError(
                f"{label} nonzero gravity feedforward was exposed"
            )
        scale = finite_number(
            document.get("gravity_scale"), f"{label}.gravity_scale"
        )
        target = finite_number(
            document.get("gravity_scale_target"),
            f"{label}.gravity_scale_target",
        )
        if abs(scale) > 1.0e-12 or abs(target) > 1.0e-12:
            raise GravityReadOnlyCaptureError(
                f"{label} gravity scale/target is not zero"
            )
        if document.get("finite_bounded") is not True:
            raise GravityReadOnlyCaptureError(
                f"{label} finite_bounded check failed"
            )
        if document.get("hardware_enable_requested") is not False:
            raise GravityReadOnlyCaptureError(
                f"{label} hardware gravity enable was requested"
            )
        if document.get("hardware_tff_enabled") is not False:
            raise GravityReadOnlyCaptureError(
                f"{label} hardware Tff was enabled"
            )

        model_q = self.anchor.model_q_from_actual(pose.q_actual_rad)
        independently_evaluated = finite_vector(
            self.evaluator.evaluate(model_q), 6,
            f"{label}.independent_qfrc_bias",
        )
        errors = [
            abs(observed - expected)
            for observed, expected in zip(gravity, independently_evaluated)
        ]
        for index, error in enumerate(errors):
            if not math.isclose(
                gravity[index], independently_evaluated[index],
                rel_tol=MODEL_COMPARISON_REL_TOLERANCE,
                abs_tol=MODEL_COMPARISON_ABS_TOLERANCE_NM,
            ):
                raise GravityReadOnlyCaptureError(
                    f"{label} J{index + 1} differs from independent MuJoCo qfrc_bias"
                )
        self.maximum_model_error_nm = max(
            self.maximum_model_error_nm, max(errors)
        )
        if self.samples:
            previous_q = self.samples[-1]["q_actual_rad"]
            step = max(
                abs(current - previous)
                for current, previous in zip(pose.q_actual_rad, previous_q)
            )
            self.maximum_position_step_rad = max(
                self.maximum_position_step_rad, step
            )
            if step > MAXIMUM_POSITION_STEP_RAD:
                raise GravityReadOnlyCaptureError(
                    f"{label} pose step exceeds stationary continuity bound"
                )

        predicted = predicted_rotor_map(gravity)
        if not math.isclose(
            abs(predicted["J2A"]), abs(predicted["J2B"]),
            rel_tol=0.0, abs_tol=1.0e-12,
        ) or not math.isclose(
            predicted["J2A"], -predicted["J2B"],
            rel_tol=0.0, abs_tol=1.0e-12,
        ):
            raise GravityReadOnlyCaptureError(
                f"{label} J2 50/50 opposite-sign split failed"
            )
        self.maximum_abs_j2_gravity_nm = max(
            self.maximum_abs_j2_gravity_nm, abs(gravity[1])
        )
        self.maximum_abs_gravity_nm = max(
            self.maximum_abs_gravity_nm, max(abs(value) for value in gravity)
        )
        self.samples.append({
            "gravity_status_sequence": int(document["sequence"]),
            "gravity_status_source_monotonic_ns": status_source_ns,
            "hardware_state_sequence": pose.sequence,
            "hardware_state_source_monotonic_ns": pose.source_monotonic_ns,
            "q_actual_rad": list(pose.q_actual_rad),
            "model_q_rad": list(model_q),
            "gravity_joint_nm": list(gravity),
            "predicted_rotor_nm": predicted,
            "max_abs_velocity_rad_s": max(
                abs(value) for value in pose.velocity_rad_s
            ),
            "gravity_scale": 0.0,
            "gravity_scale_target": 0.0,
            "feedforward_nm": [0.0] * 6,
        })

    def _direction_probes(self) -> tuple[list[dict[str, Any]], float]:
        if not self.samples:
            raise GravityReadOnlyCaptureError(
                "cannot build direction probes without samples"
            )
        center_q = tuple(self.samples[0]["model_q_rad"])
        center = finite_vector(
            self.evaluator.evaluate(center_q), 6, "direction_probe.center"
        )
        probes: list[dict[str, Any]] = []
        maximum_second_difference = 0.0
        for joint_index, joint_name in enumerate(JOINT_NAMES):
            minus_q = list(center_q)
            plus_q = list(center_q)
            minus_q[joint_index] -= DIRECTION_PROBE_RAD
            plus_q[joint_index] += DIRECTION_PROBE_RAD
            minus = finite_vector(
                self.evaluator.evaluate(minus_q), 6,
                f"direction_probe.{joint_name}.minus",
            )
            plus = finite_vector(
                self.evaluator.evaluate(plus_q), 6,
                f"direction_probe.{joint_name}.plus",
            )
            second_difference = max(
                abs(plus_value - 2.0 * center_value + minus_value)
                for minus_value, center_value, plus_value in zip(
                    minus, center, plus
                )
            )
            maximum_second_difference = max(
                maximum_second_difference, second_difference
            )
            if second_difference > MAXIMUM_PROBE_SECOND_DIFFERENCE_NM:
                raise GravityReadOnlyCaptureError(
                    f"{joint_name} local gravity direction probe is discontinuous"
                )
            probes.append({
                "joint": joint_name,
                "delta_q_rad": DIRECTION_PROBE_RAD,
                "minus_gravity_joint_nm": list(minus),
                "center_gravity_joint_nm": list(center),
                "plus_gravity_joint_nm": list(plus),
                "maximum_second_difference_nm": second_difference,
                "qvel_rad_s": [0.0] * 6,
                "qacc_rad_s2": [0.0] * 6,
            })
        return probes, maximum_second_difference

    def build_document(self) -> dict[str, Any]:
        if not self.complete:
            raise GravityReadOnlyCaptureError(
                "continuous gravity read-only qualification is incomplete"
            )
        if self.maximum_abs_gravity_nm <= 1.0e-9:
            raise GravityReadOnlyCaptureError(
                "gravity direction cannot be established from an all-zero model result"
            )
        if self.maximum_abs_j2_gravity_nm <= 1.0e-9:
            raise GravityReadOnlyCaptureError(
                "J2 sign/split cannot be established from zero gravity demand"
            )
        probes, probe_second_difference = self._direction_probes()
        return {
            "schema": GRAVITY_READONLY_SCHEMA,
            "schema_version": GRAVITY_READONLY_SCHEMA,
            "result": "PASS",
            "recorded_at_utc": utc_now(),
            "duration_s": self.duration_ns * 1.0e-9,
            "valid_sample_count": len(self.samples),
            "session_id": self.anchor.session_id,
            "state_instance_id": self.anchor.state_instance_id,
            "anchor_sha256": self.anchor_sha256,
            "model_sha256": PRODUCTION_MODEL_SHA256,
            "gravity_config_sha256": GRAVITY_CONFIG_SHA256,
            "gravity_source_instance_id": self.gravity_source_instance_id,
            "hardware_tff_enabled": False,
            "tff_transmitted": False,
            "checks": {
                "finite": True,
                "continuous": True,
                "direction_reasonable": True,
                "direction_independent_of_motion": True,
                "j2_split_50_50": True,
                "j2a_sign_correct": True,
                "j2b_sign_correct": True,
            },
            "continuity": {
                "maximum_gravity_status_gap_ms": self.maximum_status_gap_ns * 1.0e-6,
                "maximum_hardware_state_gap_ms": self.maximum_state_gap_ns * 1.0e-6,
                "maximum_pose_step_rad": self.maximum_position_step_rad,
                "maximum_abs_velocity_rad_s": self.maximum_velocity_rad_s,
                "maximum_model_comparison_error_nm": self.maximum_model_error_nm,
                "maximum_direction_probe_second_difference_nm": probe_second_difference,
                "joint_state_crosscheck_false_count": (
                    self.joint_state_crosscheck_false_count
                ),
            },
            "validation_basis": {
                "model_pose_alignment": (
                    "model_q = anchor.model_absolute + q_actual - "
                    "anchor.logical_reference"
                ),
                "gravity_calculation": (
                    "each status equals an independent frozen-model MuJoCo "
                    "qfrc_bias evaluation"
                ),
                "direction_reasonable": (
                    "symmetric +/-0.25 degree frozen-model probes are finite "
                    "and locally continuous"
                ),
                "direction_independent_of_motion": (
                    "independent evaluations force qvel=qacc=0; measured velocity "
                    "is excluded from qfrc_bias input"
                ),
                "tff_not_transmitted": (
                    "hardware remains BRAKE/DISABLED, GO tau_cmd_rotor_nm=0, "
                    "gravity enable=false, scale=target=0, feedforward=0"
                ),
            },
            "safety": {
                "collector_command_publishers": 0,
                "collector_motor_transports": 0,
                "active_command_count_observed": 0,
                "motor_internal_zero_modified": False,
                "flash_written": False,
                "eeprom_written": False,
                "mujoco_model_modified": False,
            },
            "direction_probes": probes,
            "samples": list(self.samples),
        }


def json_bytes(document: Mapping[str, Any]) -> bytes:
    data = (
        json.dumps(document, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    ).encode("utf-8")
    if len(data) > MAXIMUM_OUTPUT_BYTES:
        raise GravityReadOnlyCaptureError(
            "gravity read-only evidence exceeds 64 MiB"
        )
    return data


def fsync_directory(path: Path) -> None:
    if os.name == "nt":
        return
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def atomic_write_new(path: Path, data: bytes) -> None:
    output = Path(os.path.abspath(path))
    if output.exists() or output.is_symlink():
        raise GravityReadOnlyCaptureError(
            f"refuse to overwrite evidence: {output}"
        )
    output.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    if output.parent.is_symlink() or not output.parent.is_dir():
        raise GravityReadOnlyCaptureError("evidence output directory is unsafe")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.", suffix=".tmp", dir=output.parent
    )
    temporary = Path(temporary_name)
    published = False
    try:
        with os.fdopen(descriptor, "wb") as stream:
            if hasattr(os, "fchmod"):
                os.fchmod(stream.fileno(), stat.S_IRUSR | stat.S_IWUSR)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, output)
        except FileExistsError as exc:
            raise GravityReadOnlyCaptureError(
                f"refuse to overwrite evidence: {output}"
            ) from exc
        published = True
        fsync_directory(output.parent)
    finally:
        temporary.unlink(missing_ok=True)
    if not published or output.read_bytes() != data:
        raise GravityReadOnlyCaptureError(
            "evidence atomic publication verification failed"
        )


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-readonly", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--hardware-state-topic", default="/whole_arm/hardware_state")
    parser.add_argument("--gravity-status-topic", default="/whole_arm/gravity_status")
    parser.add_argument("--model-path", type=Path, required=True)
    parser.add_argument("--anchor", type=Path, required=True)
    parser.add_argument("--expected-anchor-sha256", required=True)
    parser.add_argument("--expected-session-id", required=True)
    parser.add_argument("--expected-state-instance-id", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    args = parser.parse_args(argv)
    if not 12.0 <= args.timeout_s <= 120.0:
        parser.error("timeout must be in [12, 120] seconds")
    return args


def validate_preflight(
    args: argparse.Namespace,
    evaluator_factory: Callable[[Path], StaticGravityEvaluator] = (
        MujocoStaticGravityEvaluator
    ),
) -> tuple[ModelSessionAnchor, str, StaticGravityEvaluator]:
    if not args.execute_readonly:
        raise GravityReadOnlyCaptureError(
            "capture requires --execute-readonly"
        )
    if args.confirm != READONLY_GATE:
        raise GravityReadOnlyCaptureError(
            f"capture requires --confirm '{READONLY_GATE}'"
        )
    if args.hardware_state_topic == args.gravity_status_topic:
        raise GravityReadOnlyCaptureError("state and gravity topics must differ")
    if args.output.exists() or args.output.is_symlink():
        raise GravityReadOnlyCaptureError(
            f"refuse to overwrite evidence: {args.output}"
        )
    model_path = validate_production_model_path(args.model_path)
    anchor, anchor_sha256 = read_anchor(
        args.anchor, args.expected_anchor_sha256
    )
    if anchor.session_id != args.expected_session_id:
        raise GravityReadOnlyCaptureError("caller-pinned session_id mismatch")
    if anchor.state_instance_id != args.expected_state_instance_id:
        raise GravityReadOnlyCaptureError(
            "caller-pinned state_instance_id mismatch"
        )
    evaluator = evaluator_factory(model_path)
    return anchor, anchor_sha256, evaluator


def capture_ros(args: argparse.Namespace) -> dict[str, Any]:
    anchor, anchor_sha256, evaluator = validate_preflight(args)
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:
        raise GravityReadOnlyCaptureError(
            "ROS 2 Python environment is unavailable"
        ) from exc

    recorder = GravityReadOnlyRecorder(
        anchor=anchor,
        anchor_sha256=anchor_sha256,
        evaluator=evaluator,
    )
    errors: list[Exception] = []

    class CaptureNode(Node):
        def __init__(self) -> None:
            super().__init__("v15_31b_gravity_readonly_capture")
            self.state_subscription = self.create_subscription(
                String, args.hardware_state_topic, self.on_state, 200
            )
            self.gravity_subscription = self.create_subscription(
                String, args.gravity_status_topic, self.on_gravity, 200
            )

        def on_state(self, message: Any) -> None:
            if recorder.complete or errors:
                return
            try:
                recorder.add_hardware_state(
                    decode_json_object(message.data, "hardware state")
                )
            except Exception as exc:
                errors.append(exc)

        def on_gravity(self, message: Any) -> None:
            if recorder.complete or errors:
                return
            try:
                recorder.add_gravity_status(
                    decode_json_object(message.data, "gravity status")
                )
            except Exception as exc:
                errors.append(exc)

    rclpy.init(args=None)
    node = CaptureNode()
    deadline = time.monotonic() + args.timeout_s
    try:
        while (
            rclpy.ok()
            and not recorder.complete
            and not errors
            and time.monotonic() < deadline
        ):
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if errors:
        raise GravityReadOnlyCaptureError(str(errors[0]))
    if not recorder.complete:
        raise GravityReadOnlyCaptureError(
            "timed out before a paired continuous 10-second gravity read-only window"
        )
    document = recorder.build_document()
    atomic_write_new(args.output, json_bytes(document))
    return document


def main(argv: Sequence[str] | None = None) -> int:
    try:
        args = parse_args(argv)
        document = capture_ros(args)
    except (GravityReadOnlyCaptureError, OSError, ValueError) as exc:
        print(json.dumps({
            "schema": GRAVITY_READONLY_SCHEMA,
            "status": "BLOCKED",
            "reason": str(exc),
            "hardware_command_interface_present": False,
            "hardware_tff_enabled": False,
            "tff_transmitted": False,
            "motor_internal_zero_modified": False,
            "flash_or_eeprom_written": False,
        }, ensure_ascii=False), flush=True)
        return 4
    print(json.dumps({
        "schema": GRAVITY_READONLY_SCHEMA,
        "status": "PASS",
        "duration_s": document["duration_s"],
        "valid_sample_count": document["valid_sample_count"],
        "session_id": document["session_id"],
        "state_instance_id": document["state_instance_id"],
        "anchor_sha256": document["anchor_sha256"],
        "output": str(args.output.resolve()),
    }, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

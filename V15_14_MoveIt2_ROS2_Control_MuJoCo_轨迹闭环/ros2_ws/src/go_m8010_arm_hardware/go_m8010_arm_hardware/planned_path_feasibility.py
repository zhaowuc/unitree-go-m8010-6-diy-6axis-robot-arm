"""Pure, fail-closed whole-path gravity/load/thermal feasibility contracts.

The GUI owns trajectory generation, but it does not own load authority.  This
module independently reconstructs and hashes every deterministic quintic
sample before a control-only MuJoCo evaluator is allowed to produce a path
load envelope.  No function in this module opens a motor transport or writes
hardware state.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass
from typing import Callable, Iterable, Mapping, Optional, Sequence

from .torque_semantics import GO_GEAR_RATIO


REQUEST_SCHEMA = "go-m8010-planned-path-feasibility-request/1.0"
PROOF_SCHEMA = "go-m8010-planned-load-thermal-feasibility/1.0"
EMPIRICAL_PROOF_SCHEMA = (
    "go-m8010-empirical-planned-load-thermal-feasibility/1.0"
)
TRAJECTORY_SCHEMA = "go-m8010-quintic-trajectory/1.0"
RECIPE_SCHEMA = "go-m8010-segmented-quintic-recipe/1.0"
QUINTIC_PROFILE = "quintic-rest-to-rest-v1"
SEGMENTED_PROFILE = "segmented-quintic-rest-to-rest-v1"
PRODUCTION_MODEL_SHA256 = (
    "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
)
GRAVITY_CONFIG_SHA256 = (
    "307469b8384fd35547327ba1d5f80aa440e6b9663406ab7c9d9469bea263335d"
)
THERMAL_CONFIG_SHA256 = (
    "1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467"
)
MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
HARDWARE_STATE_JOINT_NAMES = (
    "joint1", "joint2", "joint3", "joint4", "joint5", "joint6",
)
MAXIMUM_SEGMENTS = 4096
MAXIMUM_INTERVALS = 1_000_000
REQUEST_MAXIMUM_AGE_NS = 2_000_000_000


class PlannedPathError(ValueError):
    """A bounded invalid-request or unavailable-authority category."""


def _raise_if_cancelled(
    cancellation_requested: Optional[Callable[[], bool]],
) -> None:
    if cancellation_requested is not None and cancellation_requested():
        raise PlannedPathError("planned-path evaluation cancelled")


def _finite(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise PlannedPathError(f"{name} must be numeric")
    result = float(value)
    if not math.isfinite(result):
        raise PlannedPathError(f"{name} must be finite")
    return result


def _vector(value: object, name: str) -> tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise PlannedPathError(f"{name} must contain six values")
    return tuple(_finite(item, f"{name}[{index}]") for index, item in enumerate(value))


def _limits(value: object) -> tuple[tuple[float, float], ...]:
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise PlannedPathError("joint_limits_rad must contain six pairs")
    result = []
    for index, pair in enumerate(value):
        if not isinstance(pair, (list, tuple)) or len(pair) != 2:
            raise PlannedPathError(f"joint_limits_rad[{index}] must be a pair")
        lower = _finite(pair[0], f"joint_limits_rad[{index}][0]")
        upper = _finite(pair[1], f"joint_limits_rad[{index}][1]")
        if lower >= upper:
            raise PlannedPathError("joint limit pairs must strictly increase")
        result.append((lower, upper))
    return tuple(result)


def _sha256(value: object, name: str) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise PlannedPathError(f"{name} must be a lowercase SHA256")
    return value


def _source_instance(value: object) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 32
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise PlannedPathError("source_instance_id is invalid")
    return value


def _canonical_float(value: float) -> str:
    return float(value).hex()


def _canonical_vector(value: Sequence[float]) -> list[str]:
    return [_canonical_float(item) for item in value]


def _canonical_limits(
    value: Sequence[Sequence[float]],
) -> list[list[str]]:
    return [[_canonical_float(pair[0]), _canonical_float(pair[1])] for pair in value]


def _canonical_sha256(
    value: object,
    cancellation_requested: Optional[Callable[[], bool]] = None,
) -> str:
    digest = hashlib.sha256()
    encoder = json.JSONEncoder(
        ensure_ascii=True, sort_keys=True, separators=(",", ":")
    )
    for chunk in encoder.iterencode(value):
        _raise_if_cancelled(cancellation_requested)
        digest.update(chunk.encode("ascii"))
    return digest.hexdigest()


def _exact_nanoseconds(value_s: float) -> Optional[int]:
    value_ns = int(round(value_s * 1.0e9))
    if value_ns <= 0 or value_ns / 1.0e9 != value_s:
        return None
    return value_ns


def _trajectory_interval_count(duration_s: float, period_s: float) -> int:
    duration_ns = _exact_nanoseconds(duration_s)
    period_ns = _exact_nanoseconds(period_s)
    if duration_ns is not None and period_ns is not None:
        return (duration_ns + period_ns - 1) // period_ns
    return int(math.ceil(duration_s / period_s))


def request_identity_sha256(value: Mapping[str, object]) -> str:
    """Hash only immutable request identity; trajectory content has its own hash."""

    fields = (
        "schema",
        "source",
        "source_instance_id",
        "sequence",
        "source_monotonic_ns",
        "session_id",
        "state_instance_id",
        "model_sha256",
        "gravity_config_sha256",
        "thermal_config_sha256",
        "trajectory_sha256",
    )
    try:
        identity = {name: value[name] for name in fields}
    except KeyError as exc:
        raise PlannedPathError("request identity is incomplete") from exc
    return _canonical_sha256(identity)


def _quintic_components(
    start: Sequence[float], target: Sequence[float], time_s: float, duration_s: float,
) -> tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    normalized = time_s / duration_s
    blend = 10.0 * normalized**3 - 15.0 * normalized**4 + 6.0 * normalized**5
    first = (
        30.0 * normalized**2
        - 60.0 * normalized**3
        + 30.0 * normalized**4
    ) / duration_s
    second = (
        60.0 * normalized
        - 180.0 * normalized**2
        + 120.0 * normalized**3
    ) / (duration_s * duration_s)
    delta = tuple(wanted - source for source, wanted in zip(start, target))
    return (
        tuple(source + change * blend for source, change in zip(start, delta)),
        tuple(change * first for change in delta),
        tuple(change * second for change in delta),
    )


def _sample_at(
    start: Sequence[float], target: Sequence[float], duration_s: float,
    interval_count: int, index: int,
) -> tuple[float, tuple[float, ...], tuple[float, ...], tuple[float, ...]]:
    time_s = duration_s if index == interval_count else duration_s * index / interval_count
    q_rad, dq_rad_s, ddq_rad_s2 = _quintic_components(
        start, target, time_s, duration_s
    )
    if index == 0:
        q_rad = tuple(start)
        dq_rad_s = ddq_rad_s2 = (0.0,) * 6
    elif index == interval_count:
        q_rad = tuple(target)
        dq_rad_s = ddq_rad_s2 = (0.0,) * 6
    return time_s, q_rad, dq_rad_s, ddq_rad_s2


def _segment_semantic_sha256(
    segment: "SegmentDescriptor",
    cancellation_requested: Optional[Callable[[], bool]] = None,
) -> str:
    digest = hashlib.sha256()
    encoder = json.JSONEncoder(
        ensure_ascii=True, sort_keys=True, separators=(",", ":")
    )

    def append_json(value: object) -> None:
        for chunk in encoder.iterencode(value):
            _raise_if_cancelled(cancellation_requested)
            digest.update(chunk.encode("ascii"))

    # Stream the exact sort_keys=True representation.  A one-million-sample
    # request therefore stays cancellable without materializing a giant JSON
    # sample list on the proof worker.
    digest.update(b'{"joint_limits_rad":')
    append_json(_canonical_limits(segment.joint_limits_rad))
    digest.update(b',"profile":')
    append_json({
        "kind": QUINTIC_PROFILE,
        "duration_s": _canonical_float(segment.duration_s),
        "maximum_sample_period_s": _canonical_float(
            segment.maximum_sample_period_s
        ),
        "actual_sample_period_s": _canonical_float(
            segment.actual_sample_period_s
        ),
    })
    digest.update(b',"samples":[')
    for index in range(segment.interval_count + 1):
        _raise_if_cancelled(cancellation_requested)
        time_s, q_rad, dq_rad_s, ddq_rad_s2 = _sample_at(
            segment.start_rad,
            segment.target_rad,
            segment.duration_s,
            segment.interval_count,
            index,
        )
        if index:
            digest.update(b",")
        append_json({
            "time_s": _canonical_float(time_s),
            "q_rad": _canonical_vector(q_rad),
            "dq_rad_s": _canonical_vector(dq_rad_s),
            "ddq_rad_s2": _canonical_vector(ddq_rad_s2),
        })
    digest.update(b'],"schema":')
    append_json(TRAJECTORY_SCHEMA)
    digest.update(b',"start_rad":')
    append_json(_canonical_vector(segment.start_rad))
    digest.update(b',"target_rad":')
    append_json(_canonical_vector(segment.target_rad))
    digest.update(b"}")
    return digest.hexdigest()


@dataclass(frozen=True)
class SegmentDescriptor:
    trajectory_sha256: str
    start_rad: tuple[float, ...]
    target_rad: tuple[float, ...]
    joint_limits_rad: tuple[tuple[float, float], ...]
    duration_s: float
    maximum_sample_period_s: float
    actual_sample_period_s: float
    interval_count: int


@dataclass(frozen=True)
class PlannedPathRequest:
    request_sha256: str
    source_instance_id: str
    sequence: int
    source_monotonic_ns: int
    session_id: str
    state_instance_id: str
    trajectory_sha256: str
    start_rad: tuple[float, ...]
    target_rad: tuple[float, ...]
    joint_limits_rad: tuple[tuple[float, float], ...]
    segments: tuple[SegmentDescriptor, ...]
    sample_count: int

    def iter_joint_positions(self) -> Iterable[tuple[float, ...]]:
        for q_rad, _dq_rad_s, _ddq_rad_s2 in self.iter_joint_samples():
            yield q_rad

    def iter_joint_samples(
        self,
    ) -> Iterable[
        tuple[tuple[float, ...], tuple[float, ...], tuple[float, ...]]
    ]:
        for segment_index, segment in enumerate(self.segments):
            first_index = 0 if segment_index == 0 else 1
            for sample_index in range(first_index, segment.interval_count + 1):
                _time_s, q_rad, dq_rad_s, ddq_rad_s2 = _sample_at(
                    segment.start_rad,
                    segment.target_rad,
                    segment.duration_s,
                    segment.interval_count,
                    sample_index,
                )
                yield q_rad, dq_rad_s, ddq_rad_s2


def parse_planned_path_request(
    value: object,
    *,
    now_monotonic_ns: Optional[int] = None,
    cancellation_requested: Optional[Callable[[], bool]] = None,
) -> PlannedPathRequest:
    """Strictly validate and independently reproduce GUI trajectory hashes."""

    _raise_if_cancelled(cancellation_requested)
    required = {
        "schema", "source", "source_instance_id", "sequence",
        "source_monotonic_ns", "session_id", "state_instance_id",
        "model_sha256", "gravity_config_sha256", "thermal_config_sha256",
        "trajectory_sha256",
        "request_sha256", "trajectory",
    }
    if not isinstance(value, Mapping) or set(value) != required:
        raise PlannedPathError("planned-path request fields are not exact")
    if value["schema"] != REQUEST_SCHEMA or value["source"] != "arm_control_gui":
        raise PlannedPathError("planned-path request schema/source is invalid")
    source_instance_id = _source_instance(value["source_instance_id"])
    sequence = value["sequence"]
    source_ns = value["source_monotonic_ns"]
    if type(sequence) is not int or sequence <= 0:
        raise PlannedPathError("request sequence must be positive")
    if type(source_ns) is not int or source_ns <= 0:
        raise PlannedPathError("request timestamp must be positive")
    if now_monotonic_ns is not None and (
        source_ns > now_monotonic_ns
        or now_monotonic_ns - source_ns > REQUEST_MAXIMUM_AGE_NS
    ):
        raise PlannedPathError("planned-path request timestamp is stale")
    session_id = value["session_id"]
    state_instance_id = value["state_instance_id"]
    if not isinstance(session_id, str) or not session_id:
        raise PlannedPathError("request session_id is invalid")
    if not isinstance(state_instance_id, str) or not state_instance_id:
        raise PlannedPathError("request state_instance_id is invalid")
    if value["model_sha256"] != PRODUCTION_MODEL_SHA256:
        raise PlannedPathError("request model hash is not frozen production")
    if value["gravity_config_sha256"] != GRAVITY_CONFIG_SHA256:
        raise PlannedPathError("request gravity config hash is not frozen")
    if value["thermal_config_sha256"] != THERMAL_CONFIG_SHA256:
        raise PlannedPathError("request thermal config hash is not frozen")
    trajectory_sha256 = _sha256(value["trajectory_sha256"], "trajectory_sha256")
    request_sha256 = _sha256(value["request_sha256"], "request_sha256")
    if request_identity_sha256(value) != request_sha256:
        raise PlannedPathError("request identity hash mismatch")

    trajectory = value["trajectory"]
    trajectory_fields = {
        "schema", "trajectory_sha256", "start_rad", "target_rad",
        "joint_limits_rad", "profile", "segments",
    }
    if not isinstance(trajectory, Mapping) or set(trajectory) != trajectory_fields:
        raise PlannedPathError("trajectory request fields are not exact")
    if (
        trajectory["schema"] != RECIPE_SCHEMA
        or trajectory["trajectory_sha256"] != trajectory_sha256
    ):
        raise PlannedPathError("recipe identity is inconsistent")
    start_rad = _vector(trajectory["start_rad"], "trajectory.start_rad")
    target_rad = _vector(trajectory["target_rad"], "trajectory.target_rad")
    joint_limits = _limits(trajectory["joint_limits_rad"])
    for vector in (start_rad, target_rad):
        if any(
            not lower <= item <= upper
            for item, (lower, upper) in zip(vector, joint_limits)
        ):
            raise PlannedPathError("trajectory endpoint exceeds joint limits")
    profile = trajectory["profile"]
    if not isinstance(profile, Mapping) or set(profile) != {
        "kind", "duration_s", "maximum_sample_period_s",
        "actual_sample_period_s",
    } or profile["kind"] != SEGMENTED_PROFILE:
        raise PlannedPathError("segmented profile is invalid")
    recipe_duration = _finite(profile["duration_s"], "profile.duration_s")
    recipe_maximum_period = _finite(
        profile["maximum_sample_period_s"], "profile.maximum_sample_period_s"
    )
    recipe_actual_period = _finite(
        profile["actual_sample_period_s"], "profile.actual_sample_period_s"
    )
    raw_segments = trajectory["segments"]
    if (
        not isinstance(raw_segments, list)
        or not raw_segments
        or len(raw_segments) > MAXIMUM_SEGMENTS
    ):
        raise PlannedPathError("recipe segment count is invalid")
    segments = []
    expected_start = start_rad
    total_intervals = 0
    for segment_index, item in enumerate(raw_segments):
        _raise_if_cancelled(cancellation_requested)
        fields = {
            "schema", "trajectory_sha256", "start_rad", "target_rad",
            "joint_limits_rad", "profile",
        }
        if not isinstance(item, Mapping) or set(item) != fields:
            raise PlannedPathError("segment fields are not exact")
        if item["schema"] != TRAJECTORY_SCHEMA:
            raise PlannedPathError("segment schema is invalid")
        segment_start = _vector(item["start_rad"], "segment.start_rad")
        segment_target = _vector(item["target_rad"], "segment.target_rad")
        segment_limits = _limits(item["joint_limits_rad"])
        if segment_limits != joint_limits or segment_start != expected_start:
            raise PlannedPathError("recipe segment chain/limits mismatch")
        changed = [
            index for index, pair in enumerate(zip(segment_start, segment_target))
            if pair[0] != pair[1]
        ]
        if len(changed) != 1:
            raise PlannedPathError("each segment must move exactly one joint")
        if any(
            not lower <= item_value <= upper
            for item_value, (lower, upper) in zip(segment_target, joint_limits)
        ):
            raise PlannedPathError("segment target exceeds joint limits")
        segment_profile = item["profile"]
        if not isinstance(segment_profile, Mapping) or set(segment_profile) != {
            "kind", "duration_s", "maximum_sample_period_s",
            "actual_sample_period_s", "interval_count",
        } or segment_profile["kind"] != QUINTIC_PROFILE:
            raise PlannedPathError("segment profile is invalid")
        duration = _finite(segment_profile["duration_s"], "segment.duration_s")
        maximum_period = _finite(
            segment_profile["maximum_sample_period_s"],
            "segment.maximum_sample_period_s",
        )
        actual_period = _finite(
            segment_profile["actual_sample_period_s"],
            "segment.actual_sample_period_s",
        )
        interval_count = segment_profile["interval_count"]
        if (
            duration <= 0.0
            or maximum_period <= 0.0
            or type(interval_count) is not int
            or interval_count <= 0
            or interval_count > MAXIMUM_INTERVALS
            or _trajectory_interval_count(duration, maximum_period)
            != interval_count
            or actual_period != duration / interval_count
            or actual_period > maximum_period + 1.0e-15
        ):
            raise PlannedPathError("segment time grid is invalid")
        total_intervals += interval_count
        if total_intervals > MAXIMUM_INTERVALS:
            raise PlannedPathError("recipe sample count exceeds the proof bound")
        segment = SegmentDescriptor(
            trajectory_sha256=_sha256(
                item["trajectory_sha256"], f"segments[{segment_index}].sha256"
            ),
            start_rad=segment_start,
            target_rad=segment_target,
            joint_limits_rad=joint_limits,
            duration_s=duration,
            maximum_sample_period_s=maximum_period,
            actual_sample_period_s=actual_period,
            interval_count=interval_count,
        )
        if _segment_semantic_sha256(
            segment, cancellation_requested
        ) != segment.trajectory_sha256:
            raise PlannedPathError("segment semantic SHA256 mismatch")
        segments.append(segment)
        expected_start = segment_target
    if expected_start != target_rad:
        raise PlannedPathError("recipe final target mismatch")
    expected_duration = sum(segment.duration_s for segment in segments)
    expected_maximum = max(segment.maximum_sample_period_s for segment in segments)
    expected_actual = max(segment.actual_sample_period_s for segment in segments)
    if (
        recipe_duration != expected_duration
        or recipe_maximum_period != expected_maximum
        or recipe_actual_period != expected_actual
    ):
        raise PlannedPathError("recipe profile summary mismatch")
    expected_recipe_hash = _canonical_sha256({
        "schema": RECIPE_SCHEMA,
        "start_rad": _canonical_vector(start_rad),
        "target_rad": _canonical_vector(target_rad),
        "joint_limits_rad": _canonical_limits(joint_limits),
        "profile": {
            "kind": SEGMENTED_PROFILE,
            "duration_s": _canonical_float(recipe_duration),
            "maximum_sample_period_s": _canonical_float(recipe_maximum_period),
        },
        "segment_sha256": [segment.trajectory_sha256 for segment in segments],
    })
    if expected_recipe_hash != trajectory_sha256:
        raise PlannedPathError("recipe semantic SHA256 mismatch")
    return PlannedPathRequest(
        request_sha256=request_sha256,
        source_instance_id=source_instance_id,
        sequence=sequence,
        source_monotonic_ns=source_ns,
        session_id=session_id,
        state_instance_id=state_instance_id,
        trajectory_sha256=trajectory_sha256,
        start_rad=start_rad,
        target_rad=target_rad,
        joint_limits_rad=joint_limits,
        segments=tuple(segments),
        sample_count=total_intervals + 1,
    )


def parse_continuous_rotor_limits(value: object) -> Optional[dict[str, float]]:
    """Accept a positive shared limit or exact seven-motor future authority."""

    if value is None:
        return None
    if isinstance(value, Mapping):
        if set(value) != set(MOTOR_NAMES):
            raise PlannedPathError("continuous torque limits must name seven motors")
        result = {name: _finite(value[name], f"continuous limit {name}") for name in MOTOR_NAMES}
    else:
        shared = _finite(value, "continuous_rotor_torque_limit_nm")
        result = {name: shared for name in MOTOR_NAMES}
    if any(limit <= 0.0 for limit in result.values()):
        raise PlannedPathError("continuous torque limits must be positive")
    return result


def predicted_rotor_loads(
    joint_torque_nm: Sequence[float], *, j6_joint_to_rotor_scale: Optional[float],
) -> dict[str, Optional[float]]:
    """Map static or inverse-dynamics joint demand to physical rotors."""

    joint = _vector(joint_torque_nm, "joint_torque_nm")
    j2_half = joint[1] / (2.0 * GO_GEAR_RATIO)
    j6 = None
    if j6_joint_to_rotor_scale is not None:
        scale = _finite(j6_joint_to_rotor_scale, "j6_joint_to_rotor_scale")
        if scale <= 0.0:
            raise PlannedPathError("J6 joint-to-rotor scale must be positive")
        j6 = -joint[5] * scale
    result: dict[str, Optional[float]] = {
        "J1": joint[0] / GO_GEAR_RATIO,
        "J2A": -j2_half,
        "J2B": +j2_half,
        "J3": joint[2] / GO_GEAR_RATIO,
        "J4": joint[3] / GO_GEAR_RATIO,
        "J5": joint[4] / GO_GEAR_RATIO,
        "J6": j6,
    }
    if abs(float(result["J2A"]) + float(result["J2B"])) > 1.0e-12:
        raise PlannedPathError("J2 split lost equal/opposite invariant")
    return result


@dataclass(frozen=True)
class PathLoadEnvelope:
    request: PlannedPathRequest
    evaluated_sample_count: int
    maximum_abs_gravity_joint_torque_nm: Mapping[str, float]
    maximum_abs_gravity_rotor_torque_nm: Mapping[str, Optional[float]]
    maximum_abs_predicted_rotor_torque_nm: Mapping[str, Optional[float]]


def evaluate_path_load_envelope(
    request: PlannedPathRequest,
    *,
    anchor: object,
    evaluator: object,
    j6_joint_to_rotor_scale: Optional[float],
    cancellation_requested: Optional[Callable[[], bool]] = None,
) -> PathLoadEnvelope:
    """Evaluate MuJoCo qfrc_bias and physical rotor load at every exact sample."""

    if not isinstance(request, PlannedPathRequest):
        raise TypeError("request must be PlannedPathRequest")
    maximum_gravity: dict[str, Optional[float]] = {
        name: (None if name == "J6" and j6_joint_to_rotor_scale is None else 0.0)
        for name in MOTOR_NAMES
    }
    maximum_predicted = dict(maximum_gravity)
    maximum_gravity_joint = {name: 0.0 for name in JOINT_NAMES}
    count = 0
    for q_actual, dq_rad_s, ddq_rad_s2 in request.iter_joint_samples():
        _raise_if_cancelled(cancellation_requested)
        model_q = anchor.model_q_from_actual(q_actual)
        gravity = evaluator.evaluate(model_q)
        for name, value in zip(JOINT_NAMES, gravity):
            maximum_gravity_joint[name] = max(
                maximum_gravity_joint[name], abs(float(value))
            )
        predicted_joint = evaluator.evaluate_inverse(
            model_q, dq_rad_s, ddq_rad_s2
        )
        gravity_loads = predicted_rotor_loads(
            gravity, j6_joint_to_rotor_scale=j6_joint_to_rotor_scale
        )
        predicted_loads = predicted_rotor_loads(
            predicted_joint, j6_joint_to_rotor_scale=j6_joint_to_rotor_scale
        )
        for name, value in gravity_loads.items():
            if value is not None:
                maximum_gravity[name] = max(
                    float(maximum_gravity[name] or 0.0), abs(value)
                )
        for name, value in predicted_loads.items():
            if value is not None:
                maximum_predicted[name] = max(
                    float(maximum_predicted[name] or 0.0), abs(value)
                )
        count += 1
    if count != request.sample_count:
        raise PlannedPathError("not every trajectory sample was evaluated")
    return PathLoadEnvelope(
        request, count, maximum_gravity_joint, maximum_gravity,
        maximum_predicted
    )


def hardware_pose_feedback_blocker(
    hardware_state: object,
    *,
    now_monotonic_ns: int,
    maximum_age_ns: int,
    session_id: str,
    state_instance_id: str,
) -> str:
    """Reject a fresh outer snapshot that contains stale motor pose evidence."""

    if (
        type(now_monotonic_ns) is not int
        or now_monotonic_ns <= 0
        or type(maximum_age_ns) is not int
        or maximum_age_ns <= 0
    ):
        raise PlannedPathError("hardware pose freshness bound is invalid")
    if not isinstance(hardware_state, Mapping):
        return "HARDWARE_POSE_STATE_MISSING"
    if (
        hardware_state.get("schema") != "go-m8010-hardware-state/1.1"
        or hardware_state.get("session_id") != session_id
        or hardware_state.get("state_instance_id") != state_instance_id
        or tuple(hardware_state.get("joint_names", ()))
        != HARDWARE_STATE_JOINT_NAMES
    ):
        return "HARDWARE_POSE_STATE_IDENTITY_INVALID"
    source_ns = hardware_state.get("source_monotonic_ns")
    if (
        type(source_ns) is not int
        or source_ns <= 0
        or source_ns > now_monotonic_ns
        or now_monotonic_ns - source_ns > maximum_age_ns
    ):
        return "HARDWARE_POSE_STATE_STALE"
    if hardware_state.get("healthy") is not True:
        return "HARDWARE_POSE_STATE_NOT_HEALTHY"
    per_motor = hardware_state.get("per_motor")
    if not isinstance(per_motor, Mapping) or set(per_motor) != set(MOTOR_NAMES):
        return "HARDWARE_POSE_PER_MOTOR_INVALID"
    for name in MOTOR_NAMES:
        sample = per_motor[name]
        if not isinstance(sample, Mapping):
            return f"HARDWARE_POSE_{name}_MISSING"
        if (
            sample.get("fresh") is not True
            or sample.get("communication_ok") is not True
            or type(sample.get("merror")) is not int
            or sample.get("merror") != 0
            or sample.get("reference_captured") is not True
        ):
            return f"HARDWARE_POSE_{name}_AUTHORITY_INVALID"
        for field in ("q_joint_rad", "dq_joint_rad_s"):
            value = sample.get(field)
            if (
                type(value) not in {int, float}
                or not math.isfinite(float(value))
            ):
                return f"HARDWARE_POSE_{name}_{field.upper()}_INVALID"
        for field in (
            "feedback_source_monotonic_ns",
            "feedback_receipt_monotonic_ns",
            "last_valid_feedback_monotonic_ns",
        ):
            timestamp = sample.get(field)
            if (
                type(timestamp) is not int
                or timestamp <= 0
                or timestamp > now_monotonic_ns
                or now_monotonic_ns - timestamp > maximum_age_ns
            ):
                return f"HARDWARE_POSE_{name}_{field.upper()}_STALE"
    return ""


def temperature_observation(
    hardware_state: object,
    *,
    session_id: str,
    state_instance_id: str,
    derating_start_c: float,
) -> tuple[bool, Optional[float], bool, str]:
    """Return temperature authority, margin, hardware continuous authority, blocker."""

    if not isinstance(hardware_state, Mapping):
        return False, None, False, "TEMPERATURE_HARDWARE_STATE_MISSING"
    if (
        hardware_state.get("session_id") != session_id
        or hardware_state.get("state_instance_id") != state_instance_id
    ):
        return False, None, False, "TEMPERATURE_SESSION_MISMATCH"
    per_motor = hardware_state.get("per_motor")
    if not isinstance(per_motor, Mapping):
        return False, None, False, "TEMPERATURE_PER_MOTOR_MISSING"
    temperatures = []
    hardware_continuous = True
    for name in MOTOR_NAMES:
        sample = per_motor.get(name)
        if not isinstance(sample, Mapping):
            return False, None, False, f"TEMPERATURE_{name}_MISSING"
        temperature = sample.get("temperature_c")
        if (
            sample.get("fresh") is not True
            or sample.get("communication_ok") is not True
            or sample.get("merror") != 0
            or sample.get("thermal_metadata_status") != "OBSERVED"
            or sample.get("thermal_config_sha256") != THERMAL_CONFIG_SHA256
            or sample.get("thermal_fault_latched") is not False
            or sample.get("thermal_state") not in {"NORMAL", "WARNING", "DERATING"}
            or type(temperature) not in {int, float}
            or not math.isfinite(float(temperature))
            or float(temperature) < 0.0
        ):
            return False, None, False, f"TEMPERATURE_{name}_AUTHORITY_INVALID"
        temperatures.append(float(temperature))
        hardware_continuous = bool(
            hardware_continuous
            and sample.get("continuous_rating_authoritative") is True
        )
    return (
        True,
        float(derating_start_c) - max(temperatures),
        hardware_continuous,
        "",
    )


def build_planned_path_proof(
    envelope: Optional[PathLoadEnvelope],
    *,
    request: Optional[PlannedPathRequest] = None,
    source_instance_id: str,
    sequence: int,
    source_monotonic_ns: int,
    model_sha256: str,
    gravity_config_sha256: str,
    thermal_config_sha256: str,
    continuous_config_authoritative: bool,
    continuous_hardware_authoritative: bool,
    continuous_rotor_limits_nm: Optional[Mapping[str, float]],
    short_peak_rotor_limits_nm: Optional[Mapping[str, float]],
    temperature_limits_authoritative: bool,
    minimum_thermal_margin_c: Optional[float],
    temperature_blocker: str = "",
    evaluation_blocker: str = "",
    empirical_validation_authoritative: bool = False,
    empirical_rotor_limits_nm: Optional[Mapping[str, float]] = None,
    empirical_envelope_id: Optional[str] = None,
    empirical_envelope_sha256: Optional[str] = None,
) -> dict:
    """Combine immutable load envelope with fresh measured temperature authority."""

    bound_request = envelope.request if envelope is not None else request
    gravity_maxima = (
        None
        if envelope is None
        else dict(envelope.maximum_abs_gravity_rotor_torque_nm)
    )
    predicted_maxima = (
        None
        if envelope is None
        else dict(envelope.maximum_abs_predicted_rotor_torque_nm)
    )
    gravity_joint_maxima = (
        None
        if envelope is None
        else dict(envelope.maximum_abs_gravity_joint_torque_nm)
    )
    minimum_load_margin = None
    minimum_continuous_margin = None
    minimum_short_peak_margin = None
    minimum_predicted_continuous_margin = None
    load_result = "BLOCKED"
    load_blocker = evaluation_blocker
    if envelope is not None:
        if empirical_validation_authoritative:
            go_motors = ("J1", "J2A", "J2B", "J3", "J4", "J5")
            if (
                empirical_rotor_limits_nm is None
                or set(empirical_rotor_limits_nm) != set(MOTOR_NAMES)
                or gravity_maxima is None
                or any(gravity_maxima[name] is None for name in go_motors)
                or not isinstance(empirical_envelope_id, str)
                or not isinstance(empirical_envelope_sha256, str)
            ):
                load_blocker = "EMPIRICAL_GRAVITY_AUTHORITY_INCOMPLETE"
            else:
                empirical_margins = {
                    name: float(empirical_rotor_limits_nm[name])
                    - float(gravity_maxima[name])
                    for name in go_motors
                }
                minimum_load_margin = min(empirical_margins.values())
                load_result = "PASS" if minimum_load_margin >= 0.0 else "FAIL"
                load_blocker = (
                    "" if load_result == "PASS"
                    else "EMPIRICAL_MODEL_GRAVITY_HARD_LIMIT_EXCEEDED"
                )
        elif not continuous_config_authoritative:
            load_blocker = "CONTINUOUS_TORQUE_CONFIG_AUTHORITY_FALSE"
        elif not continuous_hardware_authoritative:
            load_blocker = "CONTINUOUS_TORQUE_HARDWARE_AUTHORITY_FALSE"
        elif continuous_rotor_limits_nm is None:
            load_blocker = "CONTINUOUS_TORQUE_LIMITS_MISSING"
        elif short_peak_rotor_limits_nm is None:
            load_blocker = "SHORT_PEAK_TORQUE_LIMITS_MISSING"
        elif (
            gravity_maxima is None
            or predicted_maxima is None
            or any(
                gravity_maxima[name] is None or predicted_maxima[name] is None
                for name in MOTOR_NAMES
            )
        ):
            load_blocker = "J6_ROTOR_TORQUE_MAPPING_AUTHORITY_MISSING"
        else:
            continuous_margins = {
                name: float(continuous_rotor_limits_nm[name])
                - float(gravity_maxima[name])
                for name in MOTOR_NAMES
            }
            short_peak_margins = {
                name: float(short_peak_rotor_limits_nm[name])
                - float(predicted_maxima[name])
                for name in MOTOR_NAMES
            }
            minimum_continuous_margin = min(continuous_margins.values())
            minimum_short_peak_margin = min(short_peak_margins.values())
            minimum_predicted_continuous_margin = min(
                float(continuous_rotor_limits_nm[name])
                - float(predicted_maxima[name])
                for name in MOTOR_NAMES
            )
            minimum_load_margin = min(
                minimum_continuous_margin, minimum_short_peak_margin
            )
            load_result = "PASS" if minimum_load_margin > 0.0 else "FAIL"
            load_blocker = (
                ""
                if load_result == "PASS"
                else "CONTINUOUS_OR_SHORT_PEAK_TORQUE_MARGIN_NOT_POSITIVE"
            )
    temperature_margin_result = "BLOCKED"
    thermal_result = "BLOCKED"
    thermal_blocker = temperature_blocker
    if not temperature_limits_authoritative:
        thermal_blocker = "TEMPERATURE_LIMIT_CONFIG_AUTHORITY_FALSE"
    elif minimum_thermal_margin_c is None:
        thermal_blocker = thermal_blocker or "TEMPERATURE_MARGIN_UNAVAILABLE"
    elif minimum_thermal_margin_c <= 0.0:
        temperature_margin_result = "FAIL"
        thermal_result = "FAIL"
        thermal_blocker = "EXACT_TRAJECTORY_DERATING_THRESHOLD_REACHED"
    else:
        temperature_margin_result = "PASS"
        if load_result != "PASS":
            thermal_blocker = "PLANNED_THERMAL_REQUIRES_LOAD_FEASIBILITY_PASS"
        elif empirical_validation_authoritative:
            thermal_result = "PASS"
            thermal_blocker = ""
        elif minimum_predicted_continuous_margin is None:
            thermal_blocker = "PREDICTED_CONTINUOUS_THERMAL_MARGIN_UNAVAILABLE"
        elif minimum_predicted_continuous_margin <= 0.0:
            thermal_blocker = (
                "THERMAL_PEAK_DUTY_MODEL_AUTHORITY_MISSING_"
                "PREDICTED_LOAD_NOT_BELOW_CONTINUOUS_RATING"
            )
        else:
            thermal_result = "PASS"
            thermal_blocker = ""
    if load_result == "FAIL" or thermal_result == "FAIL":
        result = "FAIL"
    elif load_result == "PASS" and thermal_result == "PASS":
        result = "PASS"
    else:
        result = "BLOCKED"
    blocker_code = (
        thermal_blocker
        if thermal_result == "FAIL"
        else load_blocker or thermal_blocker
    )
    if bound_request is None:
        result = load_result = thermal_result = "NOT_EVALUATED"
        blocker_code = evaluation_blocker or "PLANNED_PATH_REQUEST_MISSING"
    return {
        "schema": (
            EMPIRICAL_PROOF_SCHEMA
            if empirical_validation_authoritative else PROOF_SCHEMA
        ),
        "source": "whole_arm_gravity_node",
        "source_instance_id": source_instance_id,
        "sequence": sequence,
        "source_monotonic_ns": source_monotonic_ns,
        "result": result,
        "load_feasibility": load_result,
        "thermal_feasibility": thermal_result,
        "current_temperature_margin_result": temperature_margin_result,
        "request_sha256": (
            None if bound_request is None else bound_request.request_sha256
        ),
        "trajectory_sha256": (
            None if bound_request is None else bound_request.trajectory_sha256
        ),
        "session_id": None if bound_request is None else bound_request.session_id,
        "state_instance_id": (
            None if bound_request is None else bound_request.state_instance_id
        ),
        "model_sha256": model_sha256,
        "gravity_config_sha256": gravity_config_sha256,
        "thermal_config_sha256": thermal_config_sha256,
        "continuous_rotor_limits_authoritative": bool(
            continuous_config_authoritative and continuous_hardware_authoritative
        ),
        "authority_class": (
            "EMPIRICAL_VALIDATION_ENVELOPE"
            if empirical_validation_authoritative
            else "OFFICIAL_CONTINUOUS_RATING"
        ),
        "rating_classification": (
            "NOT_OFFICIAL_CONTINUOUS_RATING"
            if empirical_validation_authoritative
            else "OFFICIAL_CONTINUOUS_RATING"
        ),
        "empirical_validation_authoritative": bool(
            empirical_validation_authoritative
        ),
        "empirical_envelope_id": empirical_envelope_id,
        "empirical_envelope_sha256": empirical_envelope_sha256,
        "temperature_limits_authoritative": temperature_limits_authoritative,
        "sample_count": (
            None if bound_request is None else bound_request.sample_count
        ),
        "evaluated_sample_count": (
            None if envelope is None else envelope.evaluated_sample_count
        ),
        "load_evaluation_basis": (
            "MUJOCO_QFRC_BIAS_CONTINUOUS_PLUS_MJ_INVERSE_SHORT_PEAK_EVERY_SAMPLE"
        ),
        "thermal_evaluation_basis": (
            (
                "CURRENT_MEASURED_TEMPERATURE_BELOW_EMPIRICAL_ENTRY_"
                "MODEL_GRAVITY_WITHIN_SOFTWARE_HARD_LIMIT_"
                "NO_CONTINUOUS_RATING_CLAIM"
            ) if empirical_validation_authoritative else (
                "CURRENT_MEASURED_TEMPERATURE_TO_DERATING_THRESHOLD_PLUS_"
                "ALL_SAMPLE_PREDICTED_LOAD_WITHIN_CONTINUOUS_RATING_"
                "NO_HEAT_RISE_MODEL"
            )
        ),
        "maximum_abs_gravity_joint_torque_nm_by_joint": gravity_joint_maxima,
        "maximum_abs_gravity_rotor_torque_nm_by_motor": gravity_maxima,
        "maximum_abs_predicted_rotor_torque_nm_by_motor": predicted_maxima,
        "continuous_rotor_torque_limit_nm_by_motor": (
            None if continuous_rotor_limits_nm is None
            else dict(continuous_rotor_limits_nm)
        ),
        "empirical_gravity_rotor_limit_nm_by_motor": (
            None if empirical_rotor_limits_nm is None
            else dict(empirical_rotor_limits_nm)
        ),
        "minimum_empirical_gravity_rotor_margin_nm": (
            minimum_load_margin if empirical_validation_authoritative
            else None
        ),
        "short_peak_rotor_torque_limit_nm_by_motor": (
            None if short_peak_rotor_limits_nm is None
            else dict(short_peak_rotor_limits_nm)
        ),
        "minimum_continuous_rotor_torque_margin_nm": minimum_continuous_margin,
        "minimum_short_peak_rotor_torque_margin_nm": minimum_short_peak_margin,
        "minimum_predicted_continuous_rotor_torque_margin_nm": (
            minimum_predicted_continuous_margin
        ),
        "minimum_rotor_torque_margin_nm": minimum_load_margin,
        "minimum_thermal_margin_c": minimum_thermal_margin_c,
        "blocker_code": blocker_code or None,
        "blocker": blocker_code or None,
    }


__all__ = [
    "GRAVITY_CONFIG_SHA256",
    "EMPIRICAL_PROOF_SCHEMA",
    "JOINT_NAMES",
    "MOTOR_NAMES",
    "PRODUCTION_MODEL_SHA256",
    "PROOF_SCHEMA",
    "REQUEST_SCHEMA",
    "THERMAL_CONFIG_SHA256",
    "PathLoadEnvelope",
    "PlannedPathError",
    "PlannedPathRequest",
    "build_planned_path_proof",
    "evaluate_path_load_envelope",
    "hardware_pose_feedback_blocker",
    "parse_continuous_rotor_limits",
    "parse_planned_path_request",
    "predicted_rotor_loads",
    "request_identity_sha256",
    "temperature_observation",
]

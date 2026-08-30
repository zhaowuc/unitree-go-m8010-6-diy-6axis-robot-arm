"""Pure V15.31A virtual-first GUI workflow contracts.

This module deliberately has no ROS or Qt dependency.  It owns only immutable
values and deterministic validation.  In particular, editing a planned target
cannot mutate encoder feedback or a hardware command, and accepting a preview
cannot itself grant hardware authority.  The sole state transition which sets
``q_hardware_command`` is :meth:`WorkflowState.explicit_real_submit`.
"""

from __future__ import annotations

import hashlib
import json
import math
from dataclasses import dataclass, field, replace
from typing import Iterable, Optional, Sequence, Tuple, Union


JOINT_COUNT = 6
TRAJECTORY_SCHEMA = "go-m8010-quintic-trajectory/1.0"
TRAJECTORY_RECIPE_SCHEMA = "go-m8010-segmented-quintic-recipe/1.0"
TRAJECTORY_COMMAND_SCHEMA = "go-m8010-quintic-command/1.0"
PLAN_TOKEN_SCHEMA = "go-m8010-plan-token/1.0"
PLANNED_PATH_REQUEST_SCHEMA = (
    "go-m8010-planned-path-feasibility-request/1.0"
)
MAXIMUM_TRAJECTORY_INTERVALS = 1_000_000

JointVector = Tuple[float, float, float, float, float, float]
JointLimits = Tuple[
    Tuple[float, float],
    Tuple[float, float],
    Tuple[float, float],
    Tuple[float, float],
    Tuple[float, float],
    Tuple[float, float],
]


class ContractViolation(ValueError):
    """Raised when an immutable workflow value violates its schema."""


class RealSubmitRejected(PermissionError):
    """Raised when real submission lacks the current preview authority."""


def _strict_finite_float(value: object, name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ContractViolation(f"{name} must be a real number")
    result = float(value)
    if not math.isfinite(result):
        raise ContractViolation(f"{name} must be finite")
    return result


def finite_joint_vector(values: Iterable[object], name: str) -> JointVector:
    """Freeze exactly six finite numeric joint values."""

    if isinstance(values, (str, bytes)):
        raise ContractViolation(f"{name} must contain exactly six numbers")
    try:
        items = tuple(values)
    except TypeError as exc:
        raise ContractViolation(f"{name} must be iterable") from exc
    if len(items) != JOINT_COUNT:
        raise ContractViolation(f"{name} must contain exactly six values")
    checked = tuple(
        _strict_finite_float(value, f"{name}[{index}]")
        for index, value in enumerate(items)
    )
    return checked  # type: ignore[return-value]


def validated_joint_limits(values: Iterable[Sequence[object]]) -> JointLimits:
    """Freeze six finite, strictly ordered joint-limit pairs in radians."""

    if isinstance(values, (str, bytes)):
        raise ContractViolation("joint_limits_rad must contain six pairs")
    try:
        pairs = tuple(values)
    except TypeError as exc:
        raise ContractViolation("joint_limits_rad must be iterable") from exc
    if len(pairs) != JOINT_COUNT:
        raise ContractViolation("joint_limits_rad must contain exactly six pairs")
    result = []
    for index, pair in enumerate(pairs):
        if isinstance(pair, (str, bytes)):
            raise ContractViolation(f"joint_limits_rad[{index}] must be a pair")
        try:
            bounds = tuple(pair)
        except TypeError as exc:
            raise ContractViolation(
                f"joint_limits_rad[{index}] must be iterable"
            ) from exc
        if len(bounds) != 2:
            raise ContractViolation(f"joint_limits_rad[{index}] must be a pair")
        lower = _strict_finite_float(bounds[0], f"joint_limits_rad[{index}][0]")
        upper = _strict_finite_float(bounds[1], f"joint_limits_rad[{index}][1]")
        if lower >= upper:
            raise ContractViolation(
                f"joint_limits_rad[{index}] must be strictly increasing"
            )
        result.append((lower, upper))
    return tuple(result)  # type: ignore[return-value]


def _validate_vector_within_limits(
    values: JointVector, limits: JointLimits, name: str
) -> None:
    for index, (value, (lower, upper)) in enumerate(zip(values, limits)):
        if not lower <= value <= upper:
            raise ContractViolation(
                f"{name}[{index}]={value!r} is outside [{lower!r}, {upper!r}]"
            )


def _validated_sha256(value: object, name: str) -> str:
    if not isinstance(value, str) or len(value) != 64:
        raise ContractViolation(f"{name} must be a lowercase SHA256 string")
    if any(character not in "0123456789abcdef" for character in value):
        raise ContractViolation(f"{name} must be a lowercase SHA256 string")
    return value


def _canonical_float(value: float) -> str:
    return float(value).hex()


def _canonical_vector(values: JointVector) -> list[str]:
    return [_canonical_float(value) for value in values]


def _canonical_limits(values: JointLimits) -> list[list[str]]:
    return [
        [_canonical_float(lower), _canonical_float(upper)]
        for lower, upper in values
    ]


def _canonical_sha256(value: object) -> str:
    encoded = json.dumps(
        value, ensure_ascii=True, sort_keys=True, separators=(",", ":")
    ).encode("ascii")
    return hashlib.sha256(encoded).hexdigest()


def joint_vector_sha256(values: Iterable[object]) -> str:
    vector = finite_joint_vector(values, "joint_vector")
    return _canonical_sha256(_canonical_vector(vector))


def joint_limits_sha256(values: Iterable[Sequence[object]]) -> str:
    limits = validated_joint_limits(values)
    return _canonical_sha256(_canonical_limits(limits))


def _exact_nanoseconds(value_s: float) -> Optional[int]:
    """Return an exact positive nanosecond representation when one exists."""

    value_ns = int(round(value_s * 1.0e9))
    if value_ns <= 0 or value_ns / 1.0e9 != value_s:
        return None
    return value_ns


def _trajectory_interval_count(duration_s: float, period_s: float) -> int:
    """Prefer integer arithmetic for the production nanosecond time grid."""

    duration_ns = _exact_nanoseconds(duration_s)
    period_ns = _exact_nanoseconds(period_s)
    if duration_ns is not None and period_ns is not None:
        return (duration_ns + period_ns - 1) // period_ns
    return int(math.ceil(duration_s / period_s))


@dataclass(frozen=True)
class QuinticProfile:
    """Deterministic rest-to-rest quintic sampling profile."""

    duration_s: float
    maximum_sample_period_s: float = 0.01
    kind: str = "quintic-rest-to-rest-v1"

    def __post_init__(self) -> None:
        duration = _strict_finite_float(self.duration_s, "duration_s")
        period = _strict_finite_float(
            self.maximum_sample_period_s, "maximum_sample_period_s"
        )
        if duration <= 0.0:
            raise ContractViolation("duration_s must be greater than zero")
        if period <= 0.0:
            raise ContractViolation(
                "maximum_sample_period_s must be greater than zero"
            )
        if self.kind != "quintic-rest-to-rest-v1":
            raise ContractViolation("unsupported trajectory profile")
        intervals = _trajectory_interval_count(duration, period)
        if intervals < 1 or intervals > MAXIMUM_TRAJECTORY_INTERVALS:
            raise ContractViolation("trajectory sample count is outside safe bounds")
        object.__setattr__(self, "duration_s", duration)
        object.__setattr__(self, "maximum_sample_period_s", period)

    @property
    def interval_count(self) -> int:
        return _trajectory_interval_count(
            self.duration_s, self.maximum_sample_period_s
        )

    @property
    def actual_sample_period_s(self) -> float:
        return self.duration_s / self.interval_count


@dataclass(frozen=True)
class SegmentedQuinticProfile:
    """Summary of an ordered sequence of rest-to-rest quintic segments."""

    duration_s: float
    maximum_sample_period_s: float
    actual_sample_period_s: float
    kind: str = "segmented-quintic-rest-to-rest-v1"

    def __post_init__(self) -> None:
        duration = _strict_finite_float(self.duration_s, "duration_s")
        maximum = _strict_finite_float(
            self.maximum_sample_period_s, "maximum_sample_period_s"
        )
        actual = _strict_finite_float(
            self.actual_sample_period_s, "actual_sample_period_s"
        )
        if duration <= 0.0 or maximum <= 0.0 or actual <= 0.0:
            raise ContractViolation("segmented trajectory periods must be positive")
        if actual > maximum + 1.0e-15:
            raise ContractViolation("segmented sample period exceeds its limit")
        if self.kind != "segmented-quintic-rest-to-rest-v1":
            raise ContractViolation("unsupported segmented trajectory profile")
        object.__setattr__(self, "duration_s", duration)
        object.__setattr__(self, "maximum_sample_period_s", maximum)
        object.__setattr__(self, "actual_sample_period_s", actual)


@dataclass(frozen=True)
class TrajectorySample:
    time_s: float
    q_rad: JointVector
    dq_rad_s: JointVector
    ddq_rad_s2: JointVector

    def __post_init__(self) -> None:
        time_s = _strict_finite_float(self.time_s, "sample.time_s")
        if time_s < 0.0:
            raise ContractViolation("sample.time_s must be non-negative")
        object.__setattr__(self, "time_s", time_s)
        object.__setattr__(
            self, "q_rad", finite_joint_vector(self.q_rad, "sample.q_rad")
        )
        object.__setattr__(
            self,
            "dq_rad_s",
            finite_joint_vector(self.dq_rad_s, "sample.dq_rad_s"),
        )
        object.__setattr__(
            self,
            "ddq_rad_s2",
            finite_joint_vector(self.ddq_rad_s2, "sample.ddq_rad_s2"),
        )


def _quintic_components(
    start: JointVector, target: JointVector, time_s: float, duration_s: float
) -> tuple[JointVector, JointVector, JointVector]:
    normalized = time_s / duration_s
    blend = (
        10.0 * normalized**3
        - 15.0 * normalized**4
        + 6.0 * normalized**5
    )
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
    q = tuple(source + change * blend for source, change in zip(start, delta))
    dq = tuple(change * first for change in delta)
    ddq = tuple(change * second for change in delta)
    return q, dq, ddq  # type: ignore[return-value]


def _vectors_close(
    observed: JointVector, expected: JointVector, tolerance: float = 1.0e-12
) -> bool:
    return all(
        abs(actual - wanted) <= tolerance
        for actual, wanted in zip(observed, expected)
    )


@dataclass(frozen=True)
class TrajectoryPlan:
    """An immutable, fully validated deterministic quintic trajectory."""

    start_rad: JointVector
    target_rad: JointVector
    joint_limits_rad: JointLimits
    profile: QuinticProfile
    samples: Tuple[TrajectorySample, ...]
    sha256: str = field(init=False)
    limits_sha256: str = field(init=False)

    def __post_init__(self) -> None:
        start = finite_joint_vector(self.start_rad, "trajectory.start_rad")
        target = finite_joint_vector(self.target_rad, "trajectory.target_rad")
        limits = validated_joint_limits(self.joint_limits_rad)
        if not isinstance(self.profile, QuinticProfile):
            raise ContractViolation("trajectory.profile must be QuinticProfile")
        try:
            samples = tuple(self.samples)
        except TypeError as exc:
            raise ContractViolation("trajectory.samples must be iterable") from exc
        if len(samples) != self.profile.interval_count + 1:
            raise ContractViolation("trajectory sample count does not match profile")
        if not all(isinstance(sample, TrajectorySample) for sample in samples):
            raise ContractViolation("trajectory.samples contain an invalid item")
        _validate_vector_within_limits(start, limits, "trajectory.start_rad")
        _validate_vector_within_limits(target, limits, "trajectory.target_rad")

        previous_time = -1.0
        for index, sample in enumerate(samples):
            if sample.time_s <= previous_time:
                raise ContractViolation("trajectory sample times must strictly increase")
            previous_time = sample.time_s
            expected_time = (
                self.profile.duration_s
                if index == self.profile.interval_count
                else self.profile.duration_s * index / self.profile.interval_count
            )
            if abs(sample.time_s - expected_time) > 1.0e-15:
                raise ContractViolation("trajectory sample time grid is not deterministic")
            expected = _quintic_components(
                start, target, expected_time, self.profile.duration_s
            )
            if not (
                _vectors_close(sample.q_rad, expected[0])
                and _vectors_close(sample.dq_rad_s, expected[1])
                and _vectors_close(sample.ddq_rad_s2, expected[2])
            ):
                raise ContractViolation("trajectory sample is not the declared quintic")
            _validate_vector_within_limits(
                sample.q_rad, limits, f"trajectory.samples[{index}].q_rad"
            )

        if samples[0].time_s != 0.0:
            raise ContractViolation("trajectory must start at time zero")
        if samples[-1].time_s != self.profile.duration_s:
            raise ContractViolation("trajectory duration does not match profile")
        if not _vectors_close(samples[0].q_rad, start):
            raise ContractViolation("trajectory first position does not match start")
        if not _vectors_close(samples[-1].q_rad, target):
            raise ContractViolation("trajectory last position does not match target")

        object.__setattr__(self, "start_rad", start)
        object.__setattr__(self, "target_rad", target)
        object.__setattr__(self, "joint_limits_rad", limits)
        object.__setattr__(self, "samples", samples)
        limits_hash = joint_limits_sha256(limits)
        object.__setattr__(self, "limits_sha256", limits_hash)
        payload = {
            "schema": TRAJECTORY_SCHEMA,
            "start_rad": _canonical_vector(start),
            "target_rad": _canonical_vector(target),
            "joint_limits_rad": _canonical_limits(limits),
            "profile": {
                "kind": self.profile.kind,
                "duration_s": _canonical_float(self.profile.duration_s),
                "maximum_sample_period_s": _canonical_float(
                    self.profile.maximum_sample_period_s
                ),
                "actual_sample_period_s": _canonical_float(
                    self.profile.actual_sample_period_s
                ),
            },
            "samples": [
                {
                    "time_s": _canonical_float(sample.time_s),
                    "q_rad": _canonical_vector(sample.q_rad),
                    "dq_rad_s": _canonical_vector(sample.dq_rad_s),
                    "ddq_rad_s2": _canonical_vector(sample.ddq_rad_s2),
                }
                for sample in samples
            ],
        }
        object.__setattr__(self, "sha256", _canonical_sha256(payload))


def generate_quintic_trajectory(
    start_rad: Iterable[object],
    target_rad: Iterable[object],
    joint_limits_rad: Iterable[Sequence[object]],
    *,
    duration_s: float,
    maximum_sample_period_s: float = 0.01,
) -> TrajectoryPlan:
    """Generate the one deterministic trajectory used by preview and execute."""

    start = finite_joint_vector(start_rad, "start_rad")
    target = finite_joint_vector(target_rad, "target_rad")
    limits = validated_joint_limits(joint_limits_rad)
    _validate_vector_within_limits(start, limits, "start_rad")
    _validate_vector_within_limits(target, limits, "target_rad")
    profile = QuinticProfile(duration_s, maximum_sample_period_s)
    samples = []
    for index in range(profile.interval_count + 1):
        time_s = (
            profile.duration_s
            if index == profile.interval_count
            else profile.duration_s * index / profile.interval_count
        )
        q, dq, ddq = _quintic_components(
            start, target, time_s, profile.duration_s
        )
        # Pin exact rest-to-rest endpoints rather than depending on floating
        # cancellation in the polynomial at s=0 and s=1.
        if index == 0:
            q = start
            dq = ddq = (0.0,) * JOINT_COUNT  # type: ignore[assignment]
        elif index == profile.interval_count:
            q = target
            dq = ddq = (0.0,) * JOINT_COUNT  # type: ignore[assignment]
        samples.append(TrajectorySample(time_s, q, dq, ddq))
    return TrajectoryPlan(start, target, limits, profile, tuple(samples))


def quintic_duration_for_limits(
    start_rad: Iterable[object],
    target_rad: Iterable[object],
    *,
    maximum_velocity_rad_s: float,
    maximum_acceleration_rad_s2: float,
    minimum_duration_s: float = 0.25,
) -> float:
    """Return one shared duration satisfying quintic peak v/a limits.

    For the rest-to-rest blend ``10s^3-15s^4+6s^5``, the normalized
    derivative maxima are exactly 15/8 and 10/sqrt(3).  All joints share the
    largest required duration so preview and execution stay synchronized.
    """

    start = finite_joint_vector(start_rad, "start_rad")
    target = finite_joint_vector(target_rad, "target_rad")
    velocity = _strict_finite_float(
        maximum_velocity_rad_s, "maximum_velocity_rad_s"
    )
    acceleration = _strict_finite_float(
        maximum_acceleration_rad_s2, "maximum_acceleration_rad_s2"
    )
    minimum = _strict_finite_float(minimum_duration_s, "minimum_duration_s")
    if velocity <= 0.0 or acceleration <= 0.0 or minimum <= 0.0:
        raise ContractViolation("trajectory limits and minimum duration must be positive")
    largest_delta = max(abs(wanted - source) for source, wanted in zip(start, target))
    velocity_duration = (15.0 / 8.0) * largest_delta / velocity
    acceleration_duration = math.sqrt(
        (10.0 / math.sqrt(3.0)) * largest_delta / acceleration
    )
    return max(minimum, velocity_duration, acceleration_duration)


@dataclass(frozen=True)
class TrajectoryRecipe:
    """Immutable single-moving-axis recipe previewed before real execution.

    Each member is itself a complete six-dimensional trajectory, but exactly
    one joint changes per segment.  This matches the existing collision proof
    and four independent hardware fault domains without pretending they share
    an atomic multi-axis start barrier.
    """

    start_rad: JointVector
    target_rad: JointVector
    joint_limits_rad: JointLimits
    segments: Tuple[TrajectoryPlan, ...]
    profile: SegmentedQuinticProfile = field(init=False)
    samples: Tuple[TrajectorySample, ...] = field(init=False)
    sha256: str = field(init=False)
    limits_sha256: str = field(init=False)

    def __post_init__(self) -> None:
        start = finite_joint_vector(self.start_rad, "recipe.start_rad")
        target = finite_joint_vector(self.target_rad, "recipe.target_rad")
        limits = validated_joint_limits(self.joint_limits_rad)
        try:
            segments = tuple(self.segments)
        except TypeError as exc:
            raise ContractViolation("recipe.segments must be iterable") from exc
        if not segments or not all(
            isinstance(segment, TrajectoryPlan) for segment in segments
        ):
            raise ContractViolation("recipe requires validated trajectory segments")
        if len(segments) > MAXIMUM_TRAJECTORY_INTERVALS:
            raise ContractViolation("recipe contains too many segments")

        expected_start = start
        flattened = []
        time_offset = 0.0
        maximum_period = 0.0
        maximum_actual_period = 0.0
        total_intervals = 0
        for segment_index, segment in enumerate(segments):
            if segment.joint_limits_rad != limits:
                raise ContractViolation("recipe segment limits do not match")
            if segment.start_rad != expected_start:
                raise ContractViolation("recipe segment chain is discontinuous")
            changed = [
                index
                for index, (source, wanted) in enumerate(
                    zip(segment.start_rad, segment.target_rad)
                )
                if source != wanted
            ]
            if len(changed) != 1:
                raise ContractViolation(
                    "each recipe segment must move exactly one joint"
                )
            total_intervals += segment.profile.interval_count
            if total_intervals > MAXIMUM_TRAJECTORY_INTERVALS:
                raise ContractViolation("recipe sample count is outside safe bounds")
            maximum_period = max(
                maximum_period, segment.profile.maximum_sample_period_s
            )
            maximum_actual_period = max(
                maximum_actual_period, segment.profile.actual_sample_period_s
            )
            for sample_index, sample in enumerate(segment.samples):
                if segment_index and sample_index == 0:
                    continue
                flattened.append(TrajectorySample(
                    time_offset + sample.time_s,
                    sample.q_rad,
                    sample.dq_rad_s,
                    sample.ddq_rad_s2,
                ))
            time_offset += segment.profile.duration_s
            expected_start = segment.target_rad

        if expected_start != target:
            raise ContractViolation("recipe final target does not match")
        if flattened[0].q_rad != start or flattened[-1].q_rad != target:
            raise ContractViolation("recipe endpoints do not match")
        profile = SegmentedQuinticProfile(
            time_offset, maximum_period, maximum_actual_period
        )
        limits_hash = joint_limits_sha256(limits)
        payload = {
            "schema": TRAJECTORY_RECIPE_SCHEMA,
            "start_rad": _canonical_vector(start),
            "target_rad": _canonical_vector(target),
            "joint_limits_rad": _canonical_limits(limits),
            "profile": {
                "kind": profile.kind,
                "duration_s": _canonical_float(profile.duration_s),
                "maximum_sample_period_s": _canonical_float(
                    profile.maximum_sample_period_s
                ),
            },
            "segment_sha256": [segment.sha256 for segment in segments],
        }
        object.__setattr__(self, "start_rad", start)
        object.__setattr__(self, "target_rad", target)
        object.__setattr__(self, "joint_limits_rad", limits)
        object.__setattr__(self, "segments", segments)
        object.__setattr__(self, "profile", profile)
        object.__setattr__(self, "samples", tuple(flattened))
        object.__setattr__(self, "limits_sha256", limits_hash)
        object.__setattr__(self, "sha256", _canonical_sha256(payload))


TrajectoryAuthority = Union[TrajectoryPlan, TrajectoryRecipe]


def generate_segmented_quintic_recipe(
    start_rad: Iterable[object],
    target_rad: Iterable[object],
    joint_limits_rad: Iterable[Sequence[object]],
    *,
    maximum_velocity_rad_s: float,
    maximum_acceleration_rad_s2: float,
    maximum_segment_delta_rad: float,
    maximum_sample_period_s: float = 0.01,
    minimum_duration_s: float = 0.25,
) -> TrajectoryRecipe:
    """Generate the exact ordered recipe shared by preview and workers."""

    start = finite_joint_vector(start_rad, "start_rad")
    target = finite_joint_vector(target_rad, "target_rad")
    limits = validated_joint_limits(joint_limits_rad)
    maximum_delta = _strict_finite_float(
        maximum_segment_delta_rad, "maximum_segment_delta_rad"
    )
    if maximum_delta <= 0.0:
        raise ContractViolation("maximum_segment_delta_rad must be positive")
    _validate_vector_within_limits(start, limits, "start_rad")
    _validate_vector_within_limits(target, limits, "target_rad")
    current = list(start)
    segments = []
    for joint, final_target in enumerate(target):
        while abs(final_target - current[joint]) > 1.0e-15:
            remaining = final_target - current[joint]
            step = math.copysign(min(abs(remaining), maximum_delta), remaining)
            segment_start = finite_joint_vector(current, "segment_start")
            next_target = list(current)
            next_target[joint] += step
            # Pin the final endpoint exactly after the last bounded step.
            if abs(final_target - next_target[joint]) <= 1.0e-15:
                next_target[joint] = final_target
            segment_target = finite_joint_vector(next_target, "segment_target")
            required_duration = quintic_duration_for_limits(
                segment_start,
                segment_target,
                maximum_velocity_rad_s=maximum_velocity_rad_s,
                maximum_acceleration_rad_s2=maximum_acceleration_rad_s2,
                minimum_duration_s=minimum_duration_s,
            )
            required_duration_ns = max(
                1, int(math.ceil(required_duration * 1.0e9))
            )
            maximum_period_ns = int(
                math.floor(maximum_sample_period_s * 1.0e9)
            )
            if maximum_period_ns <= 0:
                raise ContractViolation(
                    "maximum_sample_period_s must be at least one nanosecond"
                )
            interval_count = (
                required_duration_ns + maximum_period_ns - 1
            ) // maximum_period_ns
            duration_ns = interval_count * maximum_period_ns
            segment = generate_quintic_trajectory(
                segment_start,
                segment_target,
                limits,
                duration_s=duration_ns / 1.0e9,
                maximum_sample_period_s=maximum_period_ns / 1.0e9,
            )
            if segment.profile.interval_count != interval_count:
                raise ContractViolation(
                    "trajectory time grid is not exactly representable"
                )
            segments.append(segment)
            current = list(segment_target)
    if not segments:
        raise ContractViolation("a segmented recipe requires a changed target")
    return TrajectoryRecipe(start, target, limits, tuple(segments))


def trajectory_command_descriptor(
    segment: TrajectoryPlan,
    *,
    plan_token_id: str,
    execute_at_monotonic_ns: int,
    segment_index: int,
    segment_count: int,
) -> dict:
    """Serialize one already-previewed segment; never re-plan an endpoint."""

    if not isinstance(segment, TrajectoryPlan):
        raise ContractViolation("segment must be a validated TrajectoryPlan")
    token = _validated_sha256(plan_token_id, "plan_token_id")
    if type(execute_at_monotonic_ns) is not int or execute_at_monotonic_ns <= 0:
        raise ContractViolation("execute_at_monotonic_ns must be positive")
    if (
        type(segment_index) is not int
        or type(segment_count) is not int
        or segment_index < 0
        or segment_count <= 0
        or segment_index >= segment_count
    ):
        raise ContractViolation("segment index/count is invalid")
    duration_ns = int(round(segment.profile.duration_s * 1.0e9))
    if duration_ns <= 0:
        raise ContractViolation("segment duration is invalid")
    if duration_ns / 1.0e9 != segment.profile.duration_s:
        raise ContractViolation("segment duration must align exactly to nanoseconds")
    if duration_ns % segment.profile.interval_count != 0:
        raise ContractViolation("segment duration must use an integer time grid")
    grid_ns = duration_ns // segment.profile.interval_count
    if grid_ns <= 0 or grid_ns > 10_000_000:
        raise ContractViolation("segment time grid must be within (0, 10ms]")
    return {
        "plan_token_id": token,
        "trajectory": {
            "schema": TRAJECTORY_COMMAND_SCHEMA,
            "trajectory_sha256": segment.sha256,
            "profile": segment.profile.kind,
            "start_rad": list(segment.start_rad),
            "target_rad": list(segment.target_rad),
            "duration_ns": duration_ns,
            "interval_count": segment.profile.interval_count,
            "execute_at_monotonic_ns": execute_at_monotonic_ns,
            "segment_index": segment_index,
            "segment_count": segment_count,
        },
    }


def trajectory_plan_manifest(recipe: TrajectoryRecipe) -> dict:
    """Return the immutable ordered segment manifest consumed by the Router."""

    if not isinstance(recipe, TrajectoryRecipe):
        raise ContractViolation("plan manifest requires a TrajectoryRecipe")
    return {
        "schema": "go-m8010-plan-manifest/1.0",
        "recipe_sha256": recipe.sha256,
        "segment_sha256": [segment.sha256 for segment in recipe.segments],
    }


def planned_trajectory_feasibility_request(
    recipe: TrajectoryRecipe,
    *,
    source_instance_id: str,
    sequence: int,
    source_monotonic_ns: int,
    session_id: str,
    state_instance_id: str,
    model_sha256: str,
    gravity_config_sha256: str,
    thermal_config_sha256: str,
) -> dict:
    """Serialize an exact, independently reproducible whole-path proof request.

    This request grants no command authority.  The gravity producer recreates
    every quintic sample and both the segment and recipe semantic hashes before
    it evaluates MuJoCo load.  Keeping this separate from command/1.3 lets the
    proof exist before a successful preview is allowed to mint PLAN_TOKEN.
    """

    if not isinstance(recipe, TrajectoryRecipe):
        raise ContractViolation("planned-path request requires a trajectory recipe")
    if (
        not isinstance(source_instance_id, str)
        or len(source_instance_id) != 32
        or any(
            character not in "0123456789abcdef"
            for character in source_instance_id
        )
    ):
        raise ContractViolation("source_instance_id must be 32 lowercase hex digits")
    if type(sequence) is not int or sequence <= 0:
        raise ContractViolation("planned-path sequence must be positive")
    if type(source_monotonic_ns) is not int or source_monotonic_ns <= 0:
        raise ContractViolation("planned-path timestamp must be positive")
    if not isinstance(session_id, str) or not session_id:
        raise ContractViolation("planned-path session_id is required")
    if not isinstance(state_instance_id, str) or not state_instance_id:
        raise ContractViolation("planned-path state_instance_id is required")
    model_hash = _validated_sha256(model_sha256, "model_sha256")
    gravity_hash = _validated_sha256(
        gravity_config_sha256, "gravity_config_sha256"
    )
    thermal_hash = _validated_sha256(
        thermal_config_sha256, "thermal_config_sha256"
    )
    trajectory = {
        "schema": TRAJECTORY_RECIPE_SCHEMA,
        "trajectory_sha256": recipe.sha256,
        "start_rad": list(recipe.start_rad),
        "target_rad": list(recipe.target_rad),
        "joint_limits_rad": [list(pair) for pair in recipe.joint_limits_rad],
        "profile": {
            "kind": recipe.profile.kind,
            "duration_s": recipe.profile.duration_s,
            "maximum_sample_period_s": recipe.profile.maximum_sample_period_s,
            "actual_sample_period_s": recipe.profile.actual_sample_period_s,
        },
        "segments": [
            {
                "schema": TRAJECTORY_SCHEMA,
                "trajectory_sha256": segment.sha256,
                "start_rad": list(segment.start_rad),
                "target_rad": list(segment.target_rad),
                "joint_limits_rad": [
                    list(pair) for pair in segment.joint_limits_rad
                ],
                "profile": {
                    "kind": segment.profile.kind,
                    "duration_s": segment.profile.duration_s,
                    "maximum_sample_period_s": (
                        segment.profile.maximum_sample_period_s
                    ),
                    "actual_sample_period_s": (
                        segment.profile.actual_sample_period_s
                    ),
                    "interval_count": segment.profile.interval_count,
                },
            }
            for segment in recipe.segments
        ],
    }
    request = {
        "schema": PLANNED_PATH_REQUEST_SCHEMA,
        "source": "arm_control_gui",
        "source_instance_id": source_instance_id,
        "sequence": sequence,
        "source_monotonic_ns": source_monotonic_ns,
        "session_id": session_id,
        "state_instance_id": state_instance_id,
        "model_sha256": model_hash,
        "gravity_config_sha256": gravity_hash,
        "thermal_config_sha256": thermal_hash,
        "trajectory_sha256": recipe.sha256,
        "trajectory": trajectory,
    }
    identity_fields = (
        "schema", "source", "source_instance_id", "sequence",
        "source_monotonic_ns", "session_id", "state_instance_id",
        "model_sha256", "gravity_config_sha256", "thermal_config_sha256",
        "trajectory_sha256",
    )
    request["request_sha256"] = _canonical_sha256({
        name: request[name] for name in identity_fields
    })
    return request


def trajectory_sample_index_at(
    now_monotonic_ns: int,
    *,
    execute_at_monotonic_ns: int,
    duration_ns: int,
    interval_count: int,
) -> int:
    """Return the exact integer-grid sample used by both worker families."""

    for label, value in (
        ("now_monotonic_ns", now_monotonic_ns),
        ("execute_at_monotonic_ns", execute_at_monotonic_ns),
        ("duration_ns", duration_ns),
        ("interval_count", interval_count),
    ):
        if type(value) is not int or value < 0:
            raise ContractViolation(f"{label} must be a non-negative integer")
    if execute_at_monotonic_ns <= 0 or duration_ns <= 0 or interval_count <= 0:
        raise ContractViolation("trajectory timing values must be positive")
    if now_monotonic_ns <= execute_at_monotonic_ns:
        return 0
    elapsed_ns = now_monotonic_ns - execute_at_monotonic_ns
    if elapsed_ns >= duration_ns:
        return interval_count
    return (elapsed_ns * interval_count) // duration_ns


@dataclass(frozen=True)
class PreviewChecks:
    """Every check required before a preview may issue real-submit authority."""

    trajectory_pass: bool
    limits_pass: bool
    collision_pass: bool
    gravity_pass: bool
    planned_load_thermal_pass: bool
    thermal_pass: bool
    feedback_fresh: bool
    communication_pass: bool

    def __post_init__(self) -> None:
        for name, value in self.as_dict().items():
            if type(value) is not bool:
                raise ContractViolation(f"preview check {name} must be boolean")

    def as_dict(self) -> dict[str, bool]:
        return {
            "trajectory_pass": self.trajectory_pass,
            "limits_pass": self.limits_pass,
            "collision_pass": self.collision_pass,
            "gravity_pass": self.gravity_pass,
            "planned_load_thermal_pass": self.planned_load_thermal_pass,
            "thermal_pass": self.thermal_pass,
            "feedback_fresh": self.feedback_fresh,
            "communication_pass": self.communication_pass,
        }

    @property
    def complete_success(self) -> bool:
        return all(self.as_dict().values())

    @classmethod
    def successful(cls) -> "PreviewChecks":
        return cls(True, True, True, True, True, True, True, True)


@dataclass(frozen=True, init=False)
class PlanToken:
    """Unforgeable through the public constructor; issued only after preview."""

    schema: str
    session_id: str
    state_instance_id: str
    candidate_revision: int
    source_actual_rad: JointVector
    source_actual_sha256: str
    target_rad: JointVector
    target_sha256: str
    trajectory_sha256: str
    trajectory_profile: str
    trajectory_duration_s: float
    trajectory_sample_period_s: float
    model_sha256: str
    limits_sha256: str
    gravity_config_sha256: str
    preview_checks_sha256: str
    created_monotonic_ns: int
    nonce: str
    token_id: str

    def __init__(self) -> None:
        raise TypeError("PlanToken is issued only by a successful preview")

    @classmethod
    def _issue(
        cls,
        *,
        session_id: str,
        state_instance_id: str,
        candidate_revision: int,
        source_actual_rad: JointVector,
        target_rad: JointVector,
        trajectory_sha256: str,
        trajectory_profile: Union[QuinticProfile, SegmentedQuinticProfile],
        model_sha256: str,
        limits_sha256: str,
        gravity_config_sha256: str,
        checks: PreviewChecks,
        created_monotonic_ns: int,
        nonce: str,
    ) -> "PlanToken":
        if not isinstance(session_id, str) or not session_id:
            raise ContractViolation("token.session_id must be non-empty")
        if not isinstance(state_instance_id, str) or not state_instance_id:
            raise ContractViolation("token.state_instance_id must be non-empty")
        if type(candidate_revision) is not int or candidate_revision < 0:
            raise ContractViolation("candidate_revision must be a non-negative integer")
        source = finite_joint_vector(source_actual_rad, "token.source_actual_rad")
        target = finite_joint_vector(target_rad, "token.target_rad")
        trajectory_hash = _validated_sha256(
            trajectory_sha256, "token.trajectory_sha256"
        )
        model_hash = _validated_sha256(model_sha256, "token.model_sha256")
        limits_hash = _validated_sha256(limits_sha256, "token.limits_sha256")
        gravity_hash = _validated_sha256(
            gravity_config_sha256, "token.gravity_config_sha256"
        )
        if not isinstance(
            trajectory_profile, (QuinticProfile, SegmentedQuinticProfile)
        ):
            raise ContractViolation("token trajectory profile is unsupported")
        if not isinstance(checks, PreviewChecks) or not checks.complete_success:
            raise ContractViolation("a complete successful preview is required")
        if type(created_monotonic_ns) is not int or created_monotonic_ns <= 0:
            raise ContractViolation("created_monotonic_ns must be a positive integer")
        if not isinstance(nonce, str) or not nonce or len(nonce) > 256:
            raise ContractViolation("nonce must be a non-empty bounded string")
        checks_hash = _canonical_sha256(checks.as_dict())
        payload = {
            "schema": PLAN_TOKEN_SCHEMA,
            "session_id": session_id,
            "state_instance_id": state_instance_id,
            "candidate_revision": candidate_revision,
            "source_actual_rad": _canonical_vector(source),
            "source_actual_sha256": joint_vector_sha256(source),
            "target_rad": _canonical_vector(target),
            "target_sha256": joint_vector_sha256(target),
            "trajectory_sha256": trajectory_hash,
            "trajectory_profile": {
                "kind": trajectory_profile.kind,
                "duration_s": _canonical_float(trajectory_profile.duration_s),
                "sample_period_s": _canonical_float(
                    trajectory_profile.actual_sample_period_s
                ),
            },
            "model_sha256": model_hash,
            "limits_sha256": limits_hash,
            "gravity_config_sha256": gravity_hash,
            "preview_checks_sha256": checks_hash,
            "created_monotonic_ns": created_monotonic_ns,
            "nonce": nonce,
        }
        instance = object.__new__(cls)
        for name, value in (
            ("schema", PLAN_TOKEN_SCHEMA),
            ("session_id", session_id),
            ("state_instance_id", state_instance_id),
            ("candidate_revision", candidate_revision),
            ("source_actual_rad", source),
            ("source_actual_sha256", joint_vector_sha256(source)),
            ("target_rad", target),
            ("target_sha256", joint_vector_sha256(target)),
            ("trajectory_sha256", trajectory_hash),
            ("trajectory_profile", trajectory_profile.kind),
            ("trajectory_duration_s", trajectory_profile.duration_s),
            (
                "trajectory_sample_period_s",
                trajectory_profile.actual_sample_period_s,
            ),
            ("model_sha256", model_hash),
            ("limits_sha256", limits_hash),
            ("gravity_config_sha256", gravity_hash),
            ("preview_checks_sha256", checks_hash),
            ("created_monotonic_ns", created_monotonic_ns),
            ("nonce", nonce),
            ("token_id", _canonical_sha256(payload)),
        ):
            object.__setattr__(instance, name, value)
        return instance


@dataclass(frozen=True)
class WorkflowState:
    """Immutable four-state GUI workflow with explicit real submission."""

    q_actual: JointVector
    q_plan_target: JointVector
    q_plan_trajectory: Optional[TrajectoryAuthority]
    q_hardware_command: Optional[JointVector]
    joint_limits_rad: JointLimits
    model_sha256: str
    session_id: str = "offline-session"
    state_instance_id: str = "offline-state-instance"
    gravity_config_sha256: str = "0" * 64
    candidate_revision: int = 0
    current_plan_token: Optional[PlanToken] = None
    submitted_token_id: Optional[str] = None

    def __post_init__(self) -> None:
        actual = finite_joint_vector(self.q_actual, "q_actual")
        target = finite_joint_vector(self.q_plan_target, "q_plan_target")
        limits = validated_joint_limits(self.joint_limits_rad)
        model_hash = _validated_sha256(self.model_sha256, "model_sha256")
        if not isinstance(self.session_id, str) or not self.session_id:
            raise ContractViolation("session_id must be non-empty")
        if not isinstance(self.state_instance_id, str) or not self.state_instance_id:
            raise ContractViolation("state_instance_id must be non-empty")
        gravity_hash = _validated_sha256(
            self.gravity_config_sha256, "gravity_config_sha256"
        )
        _validate_vector_within_limits(actual, limits, "q_actual")
        _validate_vector_within_limits(target, limits, "q_plan_target")
        if type(self.candidate_revision) is not int or self.candidate_revision < 0:
            raise ContractViolation("candidate_revision must be a non-negative integer")
        trajectory = self.q_plan_trajectory
        if trajectory is not None:
            if not isinstance(trajectory, (TrajectoryPlan, TrajectoryRecipe)):
                raise ContractViolation("q_plan_trajectory type is unsupported")
            if trajectory.target_rad != target:
                raise ContractViolation("trajectory target does not match q_plan_target")
            if trajectory.limits_sha256 != joint_limits_sha256(limits):
                raise ContractViolation("trajectory limits do not match workflow limits")
        hardware = self.q_hardware_command
        if hardware is not None:
            hardware = finite_joint_vector(hardware, "q_hardware_command")
            _validate_vector_within_limits(hardware, limits, "q_hardware_command")
        token = self.current_plan_token
        if token is not None:
            if not isinstance(token, PlanToken):
                raise ContractViolation("current_plan_token must be PlanToken")
            if trajectory is None:
                raise ContractViolation("a plan token requires q_plan_trajectory")
            if (
                token.candidate_revision != self.candidate_revision
                or token.target_rad != target
                or token.trajectory_sha256 != trajectory.sha256
                or token.model_sha256 != model_hash
                or token.limits_sha256 != joint_limits_sha256(limits)
                or token.session_id != self.session_id
                or token.state_instance_id != self.state_instance_id
                or token.gravity_config_sha256 != gravity_hash
            ):
                raise ContractViolation("plan token does not match current candidate")
        if self.submitted_token_id is not None:
            _validated_sha256(self.submitted_token_id, "submitted_token_id")
        object.__setattr__(self, "q_actual", actual)
        object.__setattr__(self, "q_plan_target", target)
        object.__setattr__(self, "joint_limits_rad", limits)
        object.__setattr__(self, "model_sha256", model_hash)
        object.__setattr__(self, "gravity_config_sha256", gravity_hash)
        object.__setattr__(self, "q_hardware_command", hardware)

    @classmethod
    def initialize(
        cls,
        q_actual: Iterable[object],
        joint_limits_rad: Iterable[Sequence[object]],
        model_sha256: str,
        *,
        session_id: str = "offline-session",
        state_instance_id: str = "offline-state-instance",
        gravity_config_sha256: str = "0" * 64,
    ) -> "WorkflowState":
        actual = finite_joint_vector(q_actual, "q_actual")
        limits = validated_joint_limits(joint_limits_rad)
        return cls(
            actual, actual, None, None, limits, model_sha256,
            session_id, state_instance_id, gravity_config_sha256,
        )

    def update_actual(
        self, q_actual: Iterable[object], *,
        invalidation_tolerance_rad: Optional[float] = None,
    ) -> "WorkflowState":
        """Update encoder state without touching target, plan, or command."""

        actual = finite_joint_vector(q_actual, "q_actual")
        _validate_vector_within_limits(actual, self.joint_limits_rad, "q_actual")
        invalidate = False
        if invalidation_tolerance_rad is not None:
            tolerance = _strict_finite_float(
                invalidation_tolerance_rad, "invalidation_tolerance_rad"
            )
            if tolerance < 0.0:
                raise ContractViolation(
                    "invalidation_tolerance_rad must be non-negative"
                )
            token = self.current_plan_token
            invalidate = bool(
                token is not None
                and any(
                    abs(current - source) > tolerance
                    and not math.isclose(
                        abs(current - source), tolerance,
                        rel_tol=0.0, abs_tol=1.0e-15,
                    )
                    for current, source in zip(actual, token.source_actual_rad)
                )
            )
        return replace(
            self,
            q_actual=actual,
            current_plan_token=None if invalidate else self.current_plan_token,
        )

    def change_plan_target(
        self, q_plan_target: Iterable[object]
    ) -> "WorkflowState":
        """Change only the candidate and invalidate every old preview token."""

        target = finite_joint_vector(q_plan_target, "q_plan_target")
        _validate_vector_within_limits(
            target, self.joint_limits_rad, "q_plan_target"
        )
        if target == self.q_plan_target:
            return self
        return replace(
            self,
            q_plan_target=target,
            q_plan_trajectory=None,
            candidate_revision=self.candidate_revision + 1,
            current_plan_token=None,
        )

    def invalidate_preview(self, *, clear_trajectory: bool = False) -> "WorkflowState":
        """Revoke real-submit authority without mutating any other state."""

        return replace(
            self,
            q_plan_trajectory=None if clear_trajectory else self.q_plan_trajectory,
            current_plan_token=None,
        )

    def set_plan_trajectory(
        self, trajectory: TrajectoryAuthority
    ) -> "WorkflowState":
        """Attach the exact trajectory to animate/check, without issuing a token."""

        if not isinstance(trajectory, (TrajectoryPlan, TrajectoryRecipe)):
            raise ContractViolation("trajectory type is unsupported")
        if trajectory.start_rad != self.q_actual:
            raise ContractViolation("trajectory source does not match q_actual")
        if trajectory.target_rad != self.q_plan_target:
            raise ContractViolation("trajectory target does not match q_plan_target")
        if trajectory.limits_sha256 != joint_limits_sha256(self.joint_limits_rad):
            raise ContractViolation("trajectory limits do not match current authority")
        return replace(
            self, q_plan_trajectory=trajectory, current_plan_token=None
        )

    def replace_authority(
        self,
        *,
        joint_limits_rad: Iterable[Sequence[object]],
        model_sha256: str,
        session_id: Optional[str] = None,
        state_instance_id: Optional[str] = None,
        gravity_config_sha256: Optional[str] = None,
    ) -> "WorkflowState":
        """Replace model/limits authority and invalidate the candidate proof."""

        limits = validated_joint_limits(joint_limits_rad)
        _validate_vector_within_limits(self.q_actual, limits, "q_actual")
        _validate_vector_within_limits(self.q_plan_target, limits, "q_plan_target")
        return replace(
            self,
            joint_limits_rad=limits,
            model_sha256=_validated_sha256(model_sha256, "model_sha256"),
            session_id=self.session_id if session_id is None else session_id,
            state_instance_id=(
                self.state_instance_id
                if state_instance_id is None else state_instance_id
            ),
            gravity_config_sha256=(
                self.gravity_config_sha256
                if gravity_config_sha256 is None else gravity_config_sha256
            ),
            q_plan_trajectory=None,
            candidate_revision=self.candidate_revision + 1,
            current_plan_token=None,
        )

    def accept_successful_preview(
        self,
        trajectory: TrajectoryAuthority,
        checks: PreviewChecks,
        *,
        created_monotonic_ns: int,
        nonce: str,
        actual_tolerance_rad: float = 0.0,
    ) -> "WorkflowState":
        """Attach a preview and issue a token only when every check passed."""

        if not isinstance(trajectory, (TrajectoryPlan, TrajectoryRecipe)):
            raise ContractViolation("trajectory type is unsupported")
        if not isinstance(checks, PreviewChecks) or not checks.complete_success:
            raise ContractViolation("a complete successful preview is required")
        tolerance = _strict_finite_float(
            actual_tolerance_rad, "actual_tolerance_rad"
        )
        if tolerance < 0.0:
            raise ContractViolation("actual_tolerance_rad must be non-negative")
        if any(
            abs(current - source) > tolerance
            and not math.isclose(
                abs(current - source), tolerance,
                rel_tol=0.0, abs_tol=1.0e-15,
            )
            for current, source in zip(self.q_actual, trajectory.start_rad)
        ):
            raise ContractViolation("preview source no longer matches current q_actual")
        if trajectory.target_rad != self.q_plan_target:
            raise ContractViolation("preview target does not match q_plan_target")
        if trajectory.limits_sha256 != joint_limits_sha256(self.joint_limits_rad):
            raise ContractViolation("preview limits do not match current authority")
        token = PlanToken._issue(
            session_id=self.session_id,
            state_instance_id=self.state_instance_id,
            candidate_revision=self.candidate_revision,
            source_actual_rad=trajectory.start_rad,
            target_rad=self.q_plan_target,
            trajectory_sha256=trajectory.sha256,
            trajectory_profile=trajectory.profile,
            model_sha256=self.model_sha256,
            limits_sha256=trajectory.limits_sha256,
            gravity_config_sha256=self.gravity_config_sha256,
            checks=checks,
            created_monotonic_ns=created_monotonic_ns,
            nonce=nonce,
        )
        return replace(
            self, q_plan_trajectory=trajectory, current_plan_token=token
        )

    def can_submit_to_real(
        self, token_id: object, *, actual_tolerance_rad: float = 0.0
    ) -> bool:
        """Return whether the current, unconsumed token authorizes submission."""

        tolerance = _strict_finite_float(
            actual_tolerance_rad, "actual_tolerance_rad"
        )
        if tolerance < 0.0:
            raise ContractViolation("actual_tolerance_rad must be non-negative")
        token = self.current_plan_token
        trajectory = self.q_plan_trajectory
        if (
            token is None
            or trajectory is None
            or not isinstance(token_id, str)
            or token_id != token.token_id
            or self.submitted_token_id == token.token_id
            or token.candidate_revision != self.candidate_revision
            or token.target_rad != self.q_plan_target
            or token.trajectory_sha256 != trajectory.sha256
            or token.model_sha256 != self.model_sha256
            or token.limits_sha256 != joint_limits_sha256(self.joint_limits_rad)
            or token.session_id != self.session_id
            or token.state_instance_id != self.state_instance_id
            or token.gravity_config_sha256 != self.gravity_config_sha256
        ):
            return False
        return all(
            abs(current - source) <= tolerance
            or math.isclose(
                abs(current - source), tolerance, rel_tol=0.0, abs_tol=1.0e-15
            )
            for current, source in zip(self.q_actual, token.source_actual_rad)
        )

    def explicit_real_submit(
        self, token_id: object, *, actual_tolerance_rad: float = 0.0
    ) -> "WorkflowState":
        """Consume the current preview token and set hardware command authority."""

        if not self.can_submit_to_real(
            token_id, actual_tolerance_rad=actual_tolerance_rad
        ):
            raise RealSubmitRejected(
                "real submission requires the current complete preview token"
            )
        assert self.current_plan_token is not None
        return replace(
            self,
            q_hardware_command=self.current_plan_token.target_rad,
            submitted_token_id=self.current_plan_token.token_id,
            current_plan_token=None,
        )


__all__ = [
    "ContractViolation",
    "JointLimits",
    "JointVector",
    "PlanToken",
    "PreviewChecks",
    "QuinticProfile",
    "SegmentedQuinticProfile",
    "RealSubmitRejected",
    "TrajectoryAuthority",
    "TrajectoryPlan",
    "TrajectoryRecipe",
    "TrajectorySample",
    "WorkflowState",
    "finite_joint_vector",
    "generate_quintic_trajectory",
    "generate_segmented_quintic_recipe",
    "joint_limits_sha256",
    "joint_vector_sha256",
    "quintic_duration_for_limits",
    "trajectory_command_descriptor",
    "planned_trajectory_feasibility_request",
    "trajectory_plan_manifest",
    "trajectory_sample_index_at",
    "validated_joint_limits",
]

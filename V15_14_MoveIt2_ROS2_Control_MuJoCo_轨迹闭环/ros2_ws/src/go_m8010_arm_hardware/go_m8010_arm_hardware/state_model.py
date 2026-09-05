"""Pure mapping logic for session and persistent software references.

This module contains no ROS or hardware writes so the seven-motor to six-joint
contract can be tested without a robot or ROS installation.
"""

from __future__ import annotations

import math
import statistics
from collections import deque
from dataclasses import dataclass
from typing import Deque, Dict, Iterable, Mapping, Optional


GEAR_RATIO = 6.329999923706055
MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
JOINT_NAMES = ("joint1", "joint2", "joint3", "joint4", "joint5", "joint6")
FEEDBACK_SOURCE_MAX_AGE_NS = 100_000_000
LOAD_LIMIT_WATCHDOG_AUTHORITY = "SOFTWARE_GUARD_NOT_CONTINUOUS_RATING"
J6_NO_PROGRESS_WATCHDOG_AUTHORITY = "J6_TARGET_TIMEOUT_POSITION_ERROR_V1"
THERMAL_CONFIG_SHA256 = (
    "1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467"
)
THERMAL_STATES = frozenset({
    "OFFLINE",
    "NORMAL",
    "WARNING",
    "DERATING",
    "THERMAL_STOP",
    "COOLDOWN",
    "WAIT_OPERATOR_CONFIRM",
})
THERMAL_TRIP_REASONS = frozenset({
    "",
    "RAW_TEMPERATURE_LIMIT",
    "EXACT_TRAJECTORY_DERATING_ABORT",
})
GRAVITY_SCALE_TARGET_LEVELS = (0.0, 0.25, 0.5, 0.75, 1.0)
# These are frozen-model software envelopes.  They are deliberately not
# represented as continuous motor ratings anywhere in the state contract.
GRAVITY_FEEDFORWARD_LIMITS_NM = (0.20, 1.75, 1.10, 0.40, 0.20, 0.0)
PRODUCTION_MODEL_SHA256 = (
    "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
)
GRAVITY_CONFIG_SHA256 = (
    "307469b8384fd35547327ba1d5f80aa440e6b9663406ab7c9d9469bea263335d"
)
WORKER_SUPERVISOR_STATUS_SCHEMA = "go-m8010-worker-supervisor-status/1.0"
WORKER_CONTROL_DOMAINS = ("J1", "J2", "J345", "J6")
WORKER_COMMAND_PORT_BY_DOMAIN = {
    "J1": 15310,
    "J2": 15312,
    "J345": 15313,
    "J6": 15311,
}
WORKER_READY_REASON = "ready"
WORKER_UNAVAILABLE_REASONS = frozenset({
    "starting",
    "process_exited",
    "udp_owner_lost",
    "supervisor_stopping",
})
J2_SESSION_MOTORS = frozenset({"J2A", "J2B"})
GO_AUX_SESSION_MOTORS = frozenset({"J1", "J3", "J4", "J5"})
POWER_SESSION_MOTORS = J2_SESSION_MOTORS | GO_AUX_SESSION_MOTORS


def _valid_sha256(value: object) -> bool:
    return bool(
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )
J2_SESSION_STARTUP_TOLERANCE_RAD = math.radians(2.0)
SESSION_STARTUP_TOLERANCE_RAD = J2_SESSION_STARTUP_TOLERANCE_RAD
VALID_FEEDBACK_MOTOR_SETS = frozenset({
    frozenset({"J1"}),
    frozenset({"J2A", "J2B"}),
    frozenset({"J3", "J4", "J5"}),
    frozenset({"J6"}),
    frozenset(MOTOR_NAMES),  # offline mock feedback
})


class WorkerSupervisorStatusError(ValueError):
    """Bounded fail-closed category for the local supervisor status file."""

    def __init__(self, reason: str) -> None:
        if reason not in {"supervisor_status_invalid", "supervisor_status_stale"}:
            raise ValueError("unknown supervisor status error category")
        self.reason = reason
        super().__init__(reason)


def unavailable_worker_control_status(reason: str) -> dict:
    """Return an explicit all-domain interlock without changing telemetry health."""

    if not isinstance(reason, str) or not reason or len(reason) > 128:
        raise ValueError("worker control reason must be bounded non-empty text")
    return {
        "worker_supervisor_instance_id": None,
        "worker_supervisor_pid": None,
        "worker_supervisor_sequence": None,
        "worker_supervisor_source_monotonic_ns": None,
        "worker_supervisor_status_age_ms": None,
        "worker_process_alive_by_domain": {
            domain: False for domain in WORKER_CONTROL_DOMAINS
        },
        "worker_udp_owner_confirmed_by_domain": {
            domain: False for domain in WORKER_CONTROL_DOMAINS
        },
        "control_available_by_domain": {
            domain: False for domain in WORKER_CONTROL_DOMAINS
        },
        "control_reason_by_domain": {
            domain: reason for domain in WORKER_CONTROL_DOMAINS
        },
    }


def validate_worker_supervisor_status(
    value: Mapping, now_monotonic_ns: int, maximum_age_ns: int,
) -> dict:
    """Validate PID/UDP-owner evidence emitted by the top-level supervisor.

    This status is deliberately independent of motor feedback freshness.  A
    stale feedback producer can therefore never stand in for a live command
    receiver.
    """

    try:
        if (
            not isinstance(value, Mapping)
            or set(value) != {
                "schema",
                "supervisor_instance_id",
                "supervisor_pid",
                "sequence",
                "source_monotonic_ns",
                "domains",
            }
            or value.get("schema") != WORKER_SUPERVISOR_STATUS_SCHEMA
            or type(now_monotonic_ns) is not int
            or now_monotonic_ns <= 0
            or type(maximum_age_ns) is not int
            or maximum_age_ns <= 0
        ):
            raise WorkerSupervisorStatusError("supervisor_status_invalid")
        instance_id = value.get("supervisor_instance_id")
        if (
            not isinstance(instance_id, str)
            or not 1 <= len(instance_id) <= 128
            or any(
                character not in "abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789-_.:"
                for character in instance_id
            )
        ):
            raise WorkerSupervisorStatusError("supervisor_status_invalid")
        supervisor_pid = value.get("supervisor_pid")
        sequence = value.get("sequence")
        source_ns = value.get("source_monotonic_ns")
        if (
            type(supervisor_pid) is not int
            or supervisor_pid <= 1
            or type(sequence) is not int
            or not 1 <= sequence <= (1 << 63) - 1
            or type(source_ns) is not int
            or source_ns <= 0
            or source_ns > now_monotonic_ns
        ):
            raise WorkerSupervisorStatusError("supervisor_status_invalid")
        age_ns = now_monotonic_ns - source_ns
        if age_ns > maximum_age_ns:
            raise WorkerSupervisorStatusError("supervisor_status_stale")
        domains = value.get("domains")
        if not isinstance(domains, Mapping) or set(domains) != set(
            WORKER_CONTROL_DOMAINS
        ):
            raise WorkerSupervisorStatusError("supervisor_status_invalid")

        process_alive = {}
        owner_confirmed = {}
        control_available = {}
        reasons = {}
        for domain in WORKER_CONTROL_DOMAINS:
            item = domains[domain]
            if (
                not isinstance(item, Mapping)
                or set(item) != {
                    "worker_pid",
                    "command_port",
                    "process_alive",
                    "udp_owner_confirmed",
                    "control_available",
                    "reason",
                }
            ):
                raise WorkerSupervisorStatusError("supervisor_status_invalid")
            worker_pid = item.get("worker_pid")
            command_port = item.get("command_port")
            alive = item.get("process_alive")
            owns_port = item.get("udp_owner_confirmed")
            available = item.get("control_available")
            reason = item.get("reason")
            if (
                type(worker_pid) is not int
                or worker_pid <= 1
                or type(command_port) is not int
                or command_port != WORKER_COMMAND_PORT_BY_DOMAIN[domain]
                or type(alive) is not bool
                or type(owns_port) is not bool
                or type(available) is not bool
                or not isinstance(reason, str)
                or reason not in WORKER_UNAVAILABLE_REASONS | {WORKER_READY_REASON}
                or available != (
                    alive and owns_port and reason == WORKER_READY_REASON
                )
            ):
                raise WorkerSupervisorStatusError("supervisor_status_invalid")
            process_alive[domain] = alive
            owner_confirmed[domain] = owns_port
            control_available[domain] = available
            reasons[domain] = reason

        return {
            "worker_supervisor_instance_id": instance_id,
            "worker_supervisor_pid": supervisor_pid,
            "worker_supervisor_sequence": sequence,
            "worker_supervisor_source_monotonic_ns": source_ns,
            "worker_supervisor_status_age_ms": age_ns / 1.0e6,
            "worker_process_alive_by_domain": process_alive,
            "worker_udp_owner_confirmed_by_domain": owner_confirmed,
            "control_available_by_domain": control_available,
            "control_reason_by_domain": reasons,
        }
    except WorkerSupervisorStatusError:
        raise
    except (KeyError, TypeError, ValueError, OverflowError) as exc:
        raise WorkerSupervisorStatusError("supervisor_status_invalid") from exc


@dataclass(frozen=True)
class MotorSpec:
    joint_name: str
    sign: int
    gear_ratio: float
    wrapped: bool = True


MOTOR_SPECS: Mapping[str, MotorSpec] = {
    "J1": MotorSpec("joint1", +1, GEAR_RATIO),
    "J2A": MotorSpec("joint2", -1, GEAR_RATIO),
    "J2B": MotorSpec("joint2", +1, GEAR_RATIO),
    "J3": MotorSpec("joint3", +1, GEAR_RATIO),
    "J4": MotorSpec("joint4", -1, GEAR_RATIO),
    "J5": MotorSpec("joint5", +1, GEAR_RATIO),
    "J6": MotorSpec("joint6", -1, 1.0, wrapped=False),
}


@dataclass(frozen=True)
class MotorFeedback:
    motor: str
    position_rad: float
    velocity_rad_s: float
    temperature_c: float
    merror: int
    communication_ok: bool
    source_monotonic_ns: int
    receipt_monotonic_ns: int
    # GO workers keep actual wire feed-forward, measured rotor torque, and the
    # signed reducer-side estimate distinct.  J6 publishes explicit None for
    # all three because POS_VEL supplies no authoritative torque channel.
    tau_cmd_rotor_nm: Optional[float] = None
    tau_feedback_rotor_nm: Optional[float] = None
    tau_joint_estimated_nm: Optional[float] = None
    last_valid_feedback_monotonic_ns: Optional[int] = None
    trajectory_plan_token_id: Optional[str] = None
    trajectory_sha256: Optional[str] = None
    trajectory_state: str = "INACTIVE"
    trajectory_sample_index: int = 0
    trajectory_interval_count: int = 0
    gravity_feedforward_rotor_nm: Optional[float] = None
    thermal_fault_latched: Optional[bool] = None
    load_limit_no_progress: Optional[bool] = None


def feedback_sample_is_fresh(
    feedback: MotorFeedback, now_monotonic_ns: int, freshness_ns: int
) -> bool:
    receipt_age = now_monotonic_ns - feedback.receipt_monotonic_ns
    source_age = now_monotonic_ns - feedback.source_monotonic_ns
    return bool(
        0 <= receipt_age <= freshness_ns
        and 0 <= source_age <= freshness_ns
    )


class AngleUnwrapper:
    def __init__(self, wrapped: bool) -> None:
        self.wrapped = wrapped
        self.previous: Optional[float] = None
        self.value: Optional[float] = None

    def update(self, raw: float) -> float:
        if not math.isfinite(raw):
            raise ValueError("non-finite motor position")
        if self.previous is None:
            self.previous = raw
            self.value = raw
            return raw
        delta = raw - self.previous
        if self.wrapped:
            delta = (delta + math.pi) % (2.0 * math.pi) - math.pi
        elif abs(delta) > math.pi:
            raise ValueError("unwrapped motor position jumped by more than pi")
        assert self.value is not None
        self.value += delta
        self.previous = raw
        return self.value


class PositionSpanVelocityObserver:
    """Estimate joint motion from encoder position span, not noisy vendor dq.

    The GO and DM feedback velocity fields have a sizeable zero-speed noise
    floor.  A short rolling encoder-position window provides conservative
    motion evidence for GUI/collision authorization while each motor's raw dq
    remains available in ``per_motor`` for diagnostics.
    """

    def __init__(
        self, window_s: float = 0.40, minimum_window_s: float = 0.35,
    ) -> None:
        if (
            not math.isfinite(window_s)
            or not math.isfinite(minimum_window_s)
            or window_s <= 0.0
            or minimum_window_s <= 0.0
            or minimum_window_s > window_s
        ):
            raise ValueError("invalid position-span velocity window")
        self.window_ns = int(window_s * 1.0e9)
        self.minimum_window_ns = int(minimum_window_s * 1.0e9)
        self.history: Deque[tuple[int, tuple[float, ...]]] = deque()

    def update(
        self, monotonic_ns: int, position_rad: Iterable[float],
    ) -> Optional[list[float]]:
        if type(monotonic_ns) is not int or monotonic_ns <= 0:
            raise ValueError("monotonic_ns must be a positive integer")
        values = tuple(position_rad)
        if (
            len(values) != len(JOINT_NAMES)
            or any(type(value) not in {int, float} for value in values)
            or not all(math.isfinite(float(value)) for value in values)
        ):
            raise ValueError("position_rad must contain six finite numbers")
        positions = tuple(float(value) for value in values)
        if self.history:
            sample_gap_ns = monotonic_ns - self.history[-1][0]
            if sample_gap_ns <= 0:
                raise ValueError("position-span samples must be strictly ordered")
            if sample_gap_ns > self.window_ns:
                self.history.clear()
        self.history.append((monotonic_ns, positions))
        cutoff_ns = monotonic_ns - self.window_ns
        while len(self.history) > 2 and self.history[1][0] <= cutoff_ns:
            self.history.popleft()
        duration_ns = self.history[-1][0] - self.history[0][0]
        if duration_ns < self.minimum_window_ns:
            return None
        duration_s = duration_ns * 1.0e-9
        result = []
        samples = list(self.history)
        for joint_index in range(len(JOINT_NAMES)):
            indexed = [
                (sample_index, sample[1][joint_index])
                for sample_index, sample in enumerate(samples)
            ]
            minimum_index, minimum = min(indexed, key=lambda item: item[1])
            maximum_index, maximum = max(indexed, key=lambda item: item[1])
            span = maximum - minimum
            if span == 0.0:
                result.append(0.0)
                continue
            direction = 1.0 if maximum_index > minimum_index else -1.0
            result.append(direction * span / duration_s)
        return result


class MirrorSessionReferenceV1:
    """Produce six logical joints from either captured or persistent references."""

    def __init__(
        self,
        capture_samples: int = 25,
        freshness_s: float = 0.10,
        capture_max_span_joint_rad: float = math.radians(0.25),
        capture_max_tail_drift_joint_rad: float = math.radians(0.10),
        persistent_references: Optional[Mapping[str, float]] = None,
        recovery_hints: Optional[Mapping[str, float]] = None,
        j2_session_references: Optional[Mapping[str, float]] = None,
        j2_session_hints: Optional[Mapping[str, float]] = None,
        go_aux_session_references: Optional[Mapping[str, float]] = None,
        go_aux_session_hints: Optional[Mapping[str, float]] = None,
    ) -> None:
        if capture_samples < 3 or freshness_s <= 0.0:
            raise ValueError("invalid session reference configuration")
        self.capture_samples = capture_samples
        self.freshness_ns = int(freshness_s * 1.0e9)
        self.capture_max_span_joint_rad = capture_max_span_joint_rad
        self.capture_max_tail_drift_joint_rad = capture_max_tail_drift_joint_rad
        configured = {} if persistent_references is None else dict(persistent_references)
        if configured and set(configured) != set(MOTOR_NAMES):
            raise ValueError("persistent software zero must contain exactly seven motors")
        if not all(math.isfinite(float(value)) for value in configured.values()):
            raise ValueError("persistent software zero contains a non-finite reference")
        self.configured_references = {
            name: float(configured[name]) for name in MOTOR_NAMES if name in configured
        }
        hints = {} if recovery_hints is None else dict(recovery_hints)
        if not set(hints).issubset(set(MOTOR_NAMES)):
            raise ValueError("recovery hints contain an unknown motor")
        if not all(math.isfinite(float(value)) for value in hints.values()):
            raise ValueError("recovery hints contain a non-finite position")
        self.recovery_hints = {name: float(value) for name, value in hints.items()}
        session_references = (
            {} if j2_session_references is None else dict(j2_session_references)
        )
        session_hints = {} if j2_session_hints is None else dict(j2_session_hints)
        if bool(session_references) != bool(session_hints):
            raise ValueError("J2 session references and hints must be configured together")
        if session_references and not self.configured_references:
            raise ValueError("J2 session references require a persistent parent")
        if session_references and set(session_references) != J2_SESSION_MOTORS:
            raise ValueError("J2 session references must contain exactly J2A and J2B")
        if session_hints and set(session_hints) != J2_SESSION_MOTORS:
            raise ValueError("J2 session hints must contain exactly J2A and J2B")
        if not all(
            math.isfinite(float(value))
            for value in (*session_references.values(), *session_hints.values())
        ):
            raise ValueError("J2 session reference contains a non-finite value")
        self.j2_session_references = {
            name: float(value) for name, value in session_references.items()
        }
        self.j2_session_hints = {
            name: float(value) for name, value in session_hints.items()
        }
        for name, hint in self.j2_session_hints.items():
            configured_hint = self.recovery_hints.get(name)
            if configured_hint is not None and abs(configured_hint - hint) > 1.0e-9:
                raise ValueError(f"J2 session hint disagrees with recovery hint for {name}")
        aux_references = (
            {} if go_aux_session_references is None
            else dict(go_aux_session_references)
        )
        aux_hints = (
            {} if go_aux_session_hints is None else dict(go_aux_session_hints)
        )
        if bool(aux_references) != bool(aux_hints):
            raise ValueError(
                "GO-AUX session references and logical positions must be configured together"
            )
        if aux_references and not self.configured_references:
            raise ValueError("GO-AUX session references require a persistent parent")
        if aux_references and set(aux_references) != GO_AUX_SESSION_MOTORS:
            raise ValueError(
                "GO-AUX session references must contain exactly J1/J3/J4/J5"
            )
        if aux_hints and set(aux_hints) != GO_AUX_SESSION_MOTORS:
            raise ValueError(
                "GO-AUX session logical positions must contain exactly J1/J3/J4/J5"
            )
        if not all(
            math.isfinite(float(value))
            for value in (*aux_references.values(), *aux_hints.values())
        ):
            raise ValueError("GO-AUX session reference contains a non-finite value")
        self.go_aux_session_references = {
            name: float(value) for name, value in aux_references.items()
        }
        self.go_aux_session_hints = {
            name: float(value) for name, value in aux_hints.items()
        }
        # GO-AUX logical positions intentionally come from the hash-bound,
        # read-only initial_pose.  The old recovery hints are preserved only as
        # immutable evidence and must not override the new power-session phase.
        self.power_session_references = {
            **self.j2_session_references,
            **self.go_aux_session_references,
        }
        self.power_session_hints = {
            **self.j2_session_hints,
            **self.go_aux_session_hints,
        }
        # GO-M8010-6 loses its accumulated multi-turn count at motor power-off.
        # A previous-session raw value therefore cannot select J2's new startup
        # branch.  When a persistent parent is in use, J2 remains unavailable
        # until a separately evidenced, parent-bound session reference exists.
        self.reference_blocked_motors = (
            POWER_SESSION_MOTORS - set(self.power_session_references)
            if self.configured_references
            else frozenset()
        )
        self.reference_name = (
            "PERSISTENT_SOFTWARE_ZERO_V1"
            if self.configured_references
            else "SESSION_REFERENCE_V1"
        )
        self.unwrappers = {
            name: AngleUnwrapper(MOTOR_SPECS[name].wrapped) for name in MOTOR_NAMES
        }
        self.history: Dict[str, Deque[float]] = {
            name: deque(maxlen=capture_samples) for name in MOTOR_NAMES
        }
        self.latest: Dict[str, MotorFeedback] = {}
        self.latest_unwrapped: Dict[str, float] = {}
        self.references: Dict[str, float] = {}
        self.captured_monotonic_ns: Optional[int] = None

    @property
    def ready(self) -> bool:
        return len(self.references) == len(MOTOR_NAMES)

    def update(self, feedback: MotorFeedback) -> None:
        if feedback.motor not in MOTOR_SPECS:
            raise ValueError(f"unknown motor {feedback.motor!r}")
        numeric = (
            feedback.position_rad,
            feedback.velocity_rad_s,
            feedback.temperature_c,
        )
        if not all(math.isfinite(value) for value in numeric):
            raise ValueError(f"non-finite feedback from {feedback.motor}")
        if feedback.receipt_monotonic_ns <= 0:
            raise ValueError("invalid receipt timestamp")
        if (
            feedback.source_monotonic_ns <= 0
            or feedback.source_monotonic_ns > feedback.receipt_monotonic_ns
            or feedback.receipt_monotonic_ns - feedback.source_monotonic_ns
            > FEEDBACK_SOURCE_MAX_AGE_NS
        ):
            raise ValueError("invalid or stale source timestamp")
        previous = self.latest.get(feedback.motor)
        if previous is not None and (
            feedback.source_monotonic_ns <= previous.source_monotonic_ns
            or feedback.receipt_monotonic_ns <= previous.receipt_monotonic_ns
        ):
            raise ValueError("replayed or out-of-order motor feedback")
        unwrapped = self.unwrappers[feedback.motor].update(feedback.position_rad)
        self.latest[feedback.motor] = feedback
        self.latest_unwrapped[feedback.motor] = unwrapped
        if (
            feedback.motor not in self.references
            and feedback.motor in self.power_session_references
        ):
            phase_reference = self.power_session_references[feedback.motor]
            hint = self.power_session_hints[feedback.motor]
            spec = MOTOR_SPECS[feedback.motor]
            desired_reference = unwrapped - spec.sign * spec.gear_ratio * hint
            reference = phase_reference + round(
                (desired_reference - phase_reference) / (2.0 * math.pi)
            ) * (2.0 * math.pi)
            recovered = spec.sign * (unwrapped - reference) / spec.gear_ratio
            if abs(recovered - hint) > SESSION_STARTUP_TOLERANCE_RAD:
                raise ValueError(
                    f"power-session reference startup mismatch for {feedback.motor}"
                )
            self.references[feedback.motor] = reference
            if self.captured_monotonic_ns is None:
                self.captured_monotonic_ns = feedback.receipt_monotonic_ns
        elif (
            feedback.motor not in self.references
            and feedback.motor in self.configured_references
            and feedback.motor not in self.reference_blocked_motors
        ):
            persistent_reference = self.configured_references[feedback.motor]
            hint = self.recovery_hints.get(feedback.motor)
            if hint is None:
                reference = persistent_reference + round(
                    (unwrapped - persistent_reference) / (2.0 * math.pi)
                ) * (2.0 * math.pi)
            else:
                spec = MOTOR_SPECS[feedback.motor]
                desired_reference = unwrapped - spec.sign * spec.gear_ratio * hint
                modulo_error = math.remainder(
                    desired_reference - persistent_reference, 2.0 * math.pi
                )
                if abs(modulo_error) > 0.10:
                    raise ValueError(
                        f"recovery hint branch inconsistent for {feedback.motor}"
                    )
                reference = persistent_reference + round(
                    (desired_reference - persistent_reference) / (2.0 * math.pi)
                ) * (2.0 * math.pi)
            self.references[feedback.motor] = reference
            if self.captured_monotonic_ns is None:
                self.captured_monotonic_ns = feedback.receipt_monotonic_ns
        if (
            feedback.motor not in self.references
            and feedback.motor not in self.reference_blocked_motors
            and feedback.communication_ok
            and feedback.merror == 0
        ):
            self.history[feedback.motor].append(unwrapped)

    def update_batch(self, feedbacks: Iterable[MotorFeedback]) -> None:
        """Atomically apply one fault-domain feedback datagram.

        A J2 datagram contains two motor samples. If either sample or its
        persistent-reference branch is invalid, neither sample may alter the
        published model state.
        """

        samples = list(feedbacks)
        names = [feedback.motor for feedback in samples]
        if len(names) != len(set(names)):
            raise ValueError("feedback batch contains a duplicate motor")
        missing = object()
        checkpoints = {}
        for name in names:
            if name not in self.unwrappers:
                raise ValueError(f"unknown motor {name!r}")
            unwrapper = self.unwrappers[name]
            checkpoints[name] = {
                "unwrapper": (unwrapper.previous, unwrapper.value),
                "history": deque(
                    self.history[name], maxlen=self.history[name].maxlen
                ),
                "latest": self.latest.get(name, missing),
                "latest_unwrapped": self.latest_unwrapped.get(name, missing),
                "reference": self.references.get(name, missing),
            }
        captured_monotonic_ns = self.captured_monotonic_ns
        try:
            for feedback in samples:
                self.update(feedback)
        except Exception:
            self.captured_monotonic_ns = captured_monotonic_ns
            for name, checkpoint in checkpoints.items():
                self.unwrappers[name].previous, self.unwrappers[name].value = (
                    checkpoint["unwrapper"]
                )
                self.history[name] = checkpoint["history"]
                for mapping_name in (
                    "latest", "latest_unwrapped", "references",
                ):
                    mapping = getattr(self, mapping_name)
                    checkpoint_name = (
                        "reference" if mapping_name == "references"
                        else mapping_name
                    )
                    prior = checkpoint[checkpoint_name]
                    if prior is missing:
                        mapping.pop(name, None)
                    else:
                        mapping[name] = prior
            raise

    def try_capture_available(self, now_monotonic_ns: int) -> bool:
        """Capture each healthy motor independently so one missing joint cannot block the GUI."""
        for name in MOTOR_NAMES:
            if name in self.references or len(self.history[name]) < self.capture_samples:
                continue
            sample = self.latest.get(name)
            if sample is None or not feedback_sample_is_fresh(
                sample, now_monotonic_ns, self.freshness_ns
            ):
                continue
            if not sample.communication_ok or sample.merror != 0:
                continue
            spec = MOTOR_SPECS[name]
            if (max(self.history[name]) - min(self.history[name])) / spec.gear_ratio > self.capture_max_span_joint_rad:
                continue
            reference = statistics.median(self.history[name])
            tail_count = min(10, len(self.history[name]))
            tail = list(self.history[name])[-tail_count:]
            if abs(statistics.median(tail) - reference) / spec.gear_ratio > self.capture_max_tail_drift_joint_rad:
                continue
            self.references[name] = reference
            if self.captured_monotonic_ns is None:
                self.captured_monotonic_ns = now_monotonic_ns
        return bool(self.references)

    def try_capture(self, now_monotonic_ns: int) -> bool:
        if self.ready:
            return True
        if any(len(self.history[name]) < self.capture_samples for name in MOTOR_NAMES):
            return False
        if any(
            not feedback_sample_is_fresh(
                self.latest[name], now_monotonic_ns, self.freshness_ns
            )
            for name in MOTOR_NAMES
        ):
            return False
        if any(
            not self.latest[name].communication_ok or self.latest[name].merror != 0
            for name in MOTOR_NAMES
        ):
            return False
        if any(
            (max(self.history[name]) - min(self.history[name]))
            / MOTOR_SPECS[name].gear_ratio
            > self.capture_max_span_joint_rad
            for name in MOTOR_NAMES
        ):
            return False
        if any(
            abs(statistics.median(list(self.history[name])[-min(10, self.capture_samples):]) -
                statistics.median(self.history[name])) / MOTOR_SPECS[name].gear_ratio
            > self.capture_max_tail_drift_joint_rad
            for name in MOTOR_NAMES
        ):
            return False
        self.references = {
            name: statistics.median(self.history[name]) for name in MOTOR_NAMES
        }
        self.captured_monotonic_ns = now_monotonic_ns
        return True

    def snapshot(self, now_monotonic_ns: int) -> dict:
        if not self.ready:
            raise RuntimeError("SESSION_REFERENCE_V1 is not captured")
        missing = [name for name in MOTOR_NAMES if name not in self.latest]
        if missing:
            raise RuntimeError(f"missing feedback: {missing}")

        per_motor = {}
        for name in MOTOR_NAMES:
            spec = MOTOR_SPECS[name]
            sample = self.latest[name]
            joint_position = (
                spec.sign
                * (self.latest_unwrapped[name] - self.references[name])
                / spec.gear_ratio
            )
            joint_velocity = spec.sign * sample.velocity_rad_s / spec.gear_ratio
            per_motor[name] = {
                "raw_position_rad": sample.position_rad,
                "q_joint_rad": joint_position,
                "dq_joint_rad_s": joint_velocity,
                "tau_cmd_rotor_nm": sample.tau_cmd_rotor_nm,
                "tau_feedback_rotor_nm": sample.tau_feedback_rotor_nm,
                "tau_joint_estimated_nm": sample.tau_joint_estimated_nm,
                # Compatibility alias for pre-V15.31A log readers.
                "estimated_joint_torque_nm": sample.tau_joint_estimated_nm,
                "trajectory_plan_token_id": sample.trajectory_plan_token_id,
                "trajectory_sha256": sample.trajectory_sha256,
                "trajectory_state": sample.trajectory_state,
                "trajectory_sample_index": sample.trajectory_sample_index,
                "trajectory_interval_count": sample.trajectory_interval_count,
                "gravity_feedforward_rotor_nm": (
                    sample.gravity_feedforward_rotor_nm
                ),
                "thermal_fault_latched_sample": sample.thermal_fault_latched,
                "load_limit_no_progress_sample": sample.load_limit_no_progress,
                "temperature_c": sample.temperature_c,
                "merror": sample.merror,
                "communication_ok": sample.communication_ok,
                "feedback_source_monotonic_ns": sample.source_monotonic_ns,
                "feedback_receipt_monotonic_ns": sample.receipt_monotonic_ns,
                "last_valid_feedback_monotonic_ns": (
                    sample.last_valid_feedback_monotonic_ns
                ),
                "fresh": feedback_sample_is_fresh(
                    sample, now_monotonic_ns, self.freshness_ns
                ),
                "age_ms": max(
                    0.0,
                    (now_monotonic_ns - sample.receipt_monotonic_ns) / 1.0e6,
                    (now_monotonic_ns - sample.source_monotonic_ns) / 1.0e6,
                ),
                "source_latency_ms": max(
                    0.0,
                    (sample.receipt_monotonic_ns - sample.source_monotonic_ns) / 1.0e6,
                ),
            }

        position = [
            per_motor["J1"]["q_joint_rad"],
            0.5 * (per_motor["J2A"]["q_joint_rad"] + per_motor["J2B"]["q_joint_rad"]),
            per_motor["J3"]["q_joint_rad"],
            per_motor["J4"]["q_joint_rad"],
            per_motor["J5"]["q_joint_rad"],
            per_motor["J6"]["q_joint_rad"],
        ]
        velocity = [
            per_motor["J1"]["dq_joint_rad_s"],
            0.5 * (per_motor["J2A"]["dq_joint_rad_s"] + per_motor["J2B"]["dq_joint_rad_s"]),
            per_motor["J3"]["dq_joint_rad_s"],
            per_motor["J4"]["dq_joint_rad_s"],
            per_motor["J5"]["dq_joint_rad_s"],
            per_motor["J6"]["dq_joint_rad_s"],
        ]
        return {
            "reference": self.reference_name,
            "joint_names": list(JOINT_NAMES),
            "position_rad": position,
            "velocity_rad_s": velocity,
            "j2_qA_rad": per_motor["J2A"]["q_joint_rad"],
            "j2_qB_rad": per_motor["J2B"]["q_joint_rad"],
            "j2_dqA_rad_s": per_motor["J2A"]["dq_joint_rad_s"],
            "j2_dqB_rad_s": per_motor["J2B"]["dq_joint_rad_s"],
            "j2_e_sync_rad": per_motor["J2A"]["q_joint_rad"]
            - per_motor["J2B"]["q_joint_rad"],
            "tau_j2_logical_total_nm": (
                None
                if per_motor["J2A"]["tau_joint_estimated_nm"] is None
                or per_motor["J2B"]["tau_joint_estimated_nm"] is None
                else per_motor["J2A"]["tau_joint_estimated_nm"]
                + per_motor["J2B"]["tau_joint_estimated_nm"]
            ),
            "per_motor": per_motor,
            "healthy": all(
                value["communication_ok"]
                and value["fresh"]
                and value["merror"] == 0
                for value in per_motor.values()
            ),
        }

    def snapshot_available(self, now_monotonic_ns: int) -> dict:
        """Return six logical joints while explicitly marking unavailable sources."""
        if not self.references:
            raise RuntimeError("no available session reference has been captured")
        per_motor = {}
        for name in MOTOR_NAMES:
            spec = MOTOR_SPECS[name]
            sample = self.latest.get(name)
            available = name in self.references and sample is not None
            if available:
                assert sample is not None
                joint_position = spec.sign * (
                    self.latest_unwrapped[name] - self.references[name]
                ) / spec.gear_ratio
                joint_velocity = spec.sign * sample.velocity_rad_s / spec.gear_ratio
                age_ms = max(
                    0.0,
                    (now_monotonic_ns - sample.receipt_monotonic_ns) / 1.0e6,
                    (now_monotonic_ns - sample.source_monotonic_ns) / 1.0e6,
                )
                fresh = feedback_sample_is_fresh(
                    sample, now_monotonic_ns, self.freshness_ns
                )
                source_latency_ms = max(
                    0.0,
                    (sample.receipt_monotonic_ns - sample.source_monotonic_ns) / 1.0e6,
                )
            else:
                joint_position = joint_velocity = 0.0
                age_ms = None
                fresh = False
                source_latency_ms = None
            per_motor[name] = {
                "raw_position_rad": None if sample is None else sample.position_rad,
                "q_joint_rad": joint_position,
                "dq_joint_rad_s": joint_velocity,
                "tau_cmd_rotor_nm": (
                    None if sample is None else sample.tau_cmd_rotor_nm
                ),
                "tau_feedback_rotor_nm": (
                    None if sample is None else sample.tau_feedback_rotor_nm
                ),
                "tau_joint_estimated_nm": (
                    None if sample is None else sample.tau_joint_estimated_nm
                ),
                "estimated_joint_torque_nm": (
                    None if sample is None else sample.tau_joint_estimated_nm
                ),
                "trajectory_plan_token_id": (
                    None if sample is None else sample.trajectory_plan_token_id
                ),
                "trajectory_sha256": (
                    None if sample is None else sample.trajectory_sha256
                ),
                "trajectory_state": (
                    "INACTIVE" if sample is None else sample.trajectory_state
                ),
                "trajectory_sample_index": (
                    0 if sample is None else sample.trajectory_sample_index
                ),
                "trajectory_interval_count": (
                    0 if sample is None else sample.trajectory_interval_count
                ),
                "gravity_feedforward_rotor_nm": (
                    None if sample is None else sample.gravity_feedforward_rotor_nm
                ),
                "thermal_fault_latched_sample": (
                    None if sample is None else sample.thermal_fault_latched
                ),
                "load_limit_no_progress_sample": (
                    None if sample is None else sample.load_limit_no_progress
                ),
                "temperature_c": 0.0 if sample is None else sample.temperature_c,
                "merror": -1 if sample is None else sample.merror,
                "communication_ok": bool(available and sample and sample.communication_ok),
                "feedback_source_monotonic_ns": (
                    None if sample is None else sample.source_monotonic_ns
                ),
                "feedback_receipt_monotonic_ns": (
                    None if sample is None else sample.receipt_monotonic_ns
                ),
                "last_valid_feedback_monotonic_ns": (
                    None
                    if sample is None
                    else sample.last_valid_feedback_monotonic_ns
                ),
                "fresh": fresh,
                "age_ms": age_ms,
                "source_latency_ms": source_latency_ms,
                "reference_captured": name in self.references,
            }

        j2_available = "J2A" in self.references and "J2B" in self.references
        position = [
            per_motor["J1"]["q_joint_rad"],
            0.5 * (per_motor["J2A"]["q_joint_rad"] + per_motor["J2B"]["q_joint_rad"])
            if j2_available else 0.0,
            per_motor["J3"]["q_joint_rad"],
            per_motor["J4"]["q_joint_rad"],
            per_motor["J5"]["q_joint_rad"],
            per_motor["J6"]["q_joint_rad"],
        ]
        velocity = [
            per_motor["J1"]["dq_joint_rad_s"],
            0.5 * (per_motor["J2A"]["dq_joint_rad_s"] + per_motor["J2B"]["dq_joint_rad_s"])
            if j2_available else 0.0,
            per_motor["J3"]["dq_joint_rad_s"],
            per_motor["J4"]["dq_joint_rad_s"],
            per_motor["J5"]["dq_joint_rad_s"],
            per_motor["J6"]["dq_joint_rad_s"],
        ]
        return {
            "reference": self.reference_name,
            "joint_names": list(JOINT_NAMES),
            "position_rad": position,
            "velocity_rad_s": velocity,
            "j2_qA_rad": per_motor["J2A"]["q_joint_rad"],
            "j2_qB_rad": per_motor["J2B"]["q_joint_rad"],
            "j2_dqA_rad_s": per_motor["J2A"]["dq_joint_rad_s"],
            "j2_dqB_rad_s": per_motor["J2B"]["dq_joint_rad_s"],
            "j2_e_sync_rad": (
                per_motor["J2A"]["q_joint_rad"] - per_motor["J2B"]["q_joint_rad"]
                if j2_available else 0.0
            ),
            "tau_j2_logical_total_nm": (
                None
                if per_motor["J2A"]["tau_joint_estimated_nm"] is None
                or per_motor["J2B"]["tau_joint_estimated_nm"] is None
                else per_motor["J2A"]["tau_joint_estimated_nm"]
                + per_motor["J2B"]["tau_joint_estimated_nm"]
            ),
            "per_motor": per_motor,
            "available_motors": sorted(self.references),
            "healthy": len(self.references) == len(MOTOR_NAMES) and all(
                value["communication_ok"] and value["fresh"] and value["merror"] == 0
                for value in per_motor.values()
            ),
        }


def validate_j6_feedback_identity(
    payload: Mapping[str, object],
    receipt_monotonic_ns: int,
    *,
    previous: Optional[Mapping[str, object]] = None,
    expected_session_id: Optional[str] = None,
    expected_state_instance_id: Optional[str] = None,
) -> dict[str, object]:
    """Bind raw J6 DISABLED evidence to one worker and model session."""

    source = payload.get("source_instance_id")
    sequence = payload.get("sequence")
    source_ns = payload.get("source_monotonic_ns")
    session = payload.get("session_id")
    state = payload.get("state_instance_id")
    lower_hex = "0123456789abcdef"
    if (
        not isinstance(source, str)
        or len(source) != 32
        or any(character not in lower_hex for character in source)
        or type(sequence) is not int
        or not 1 <= sequence <= (1 << 63) - 1
        or type(source_ns) is not int
        or not 0 < source_ns <= receipt_monotonic_ns
        or receipt_monotonic_ns - source_ns > FEEDBACK_SOURCE_MAX_AGE_NS
        or not isinstance(session, str)
        or not session
        or not isinstance(state, str)
        or len(state) != 32
        or any(character not in lower_hex for character in state)
    ):
        raise ValueError("J6 feedback identity is missing or invalid")
    if expected_session_id is not None and session != expected_session_id:
        raise ValueError("J6 feedback session binding mismatch")
    if (
        expected_state_instance_id is not None
        and state != expected_state_instance_id
    ):
        raise ValueError("J6 feedback state binding mismatch")
    if previous is not None and (
        source != previous.get("source_instance_id")
        or session != previous.get("session_id")
        or state != previous.get("state_instance_id")
        or sequence <= previous.get("sequence", 0)
        or source_ns <= previous.get("source_monotonic_ns", 0)
    ):
        raise ValueError("J6 feedback identity changed or replayed")
    return {
        "source_instance_id": source,
        "sequence": sequence,
        "source_monotonic_ns": source_ns,
        "session_id": session,
        "state_instance_id": state,
    }


def parse_feedback_payload(
    payload: Mapping, receipt_monotonic_ns: int
) -> tuple[MotorFeedback, ...]:
    """Validate one complete fault-domain JSON datagram."""

    if type(receipt_monotonic_ns) is not int or receipt_monotonic_ns <= 0:
        raise ValueError("receipt_monotonic_ns must be a positive integer")
    samples = payload.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError("feedback payload requires a non-empty samples list")
    if "source_monotonic_ns" not in payload:
        raise ValueError("source_monotonic_ns is required")
    default_source_ns = payload["source_monotonic_ns"]
    if type(default_source_ns) is not int:
        raise ValueError("source_monotonic_ns must be an integer")
    seen_motors = set()
    feedbacks = []
    for item in samples:
        if not isinstance(item, Mapping):
            raise ValueError("each feedback sample must be an object")
        if type(item.get("motor")) is not str:
            raise ValueError("motor must be a string")
        motor = item["motor"].upper()
        if motor not in MOTOR_SPECS:
            raise ValueError(f"unknown motor {motor!r}")
        if motor in seen_motors:
            raise ValueError(f"duplicate motor sample {motor!r}")
        seen_motors.add(motor)
        source_ns = item.get("source_monotonic_ns", default_source_ns)
        if type(source_ns) is not int:
            raise ValueError("source_monotonic_ns must be an integer")
        if (
            source_ns <= 0
            or source_ns > receipt_monotonic_ns
            or receipt_monotonic_ns - source_ns > FEEDBACK_SOURCE_MAX_AGE_NS
        ):
            raise ValueError("source_monotonic_ns is invalid, stale, or replayed")
        for field in ("position_rad", "velocity_rad_s", "temperature_c"):
            if field not in item or type(item[field]) not in {int, float}:
                raise ValueError(f"{field} must be a number")
        if "communication_ok" not in item:
            raise ValueError("communication_ok is required")
        if type(item["communication_ok"]) is not bool:
            raise ValueError("communication_ok must be boolean")
        if "merror" not in item:
            raise ValueError("merror is required")
        merror = item["merror"]
        if type(merror) is not int:
            raise ValueError("merror must be an integer")
        torque_values: dict[str, Optional[float]] = {}
        for torque_field in (
            "tau_cmd_rotor_nm",
            "tau_feedback_rotor_nm",
            "tau_joint_estimated_nm",
        ):
            if torque_field not in item:
                raise ValueError(f"{torque_field} is required")
            torque_value = item.get(torque_field)
            if torque_value is not None:
                if type(torque_value) not in {int, float} or not math.isfinite(
                    float(torque_value)
                ):
                    raise ValueError(f"{torque_field} must be finite or null")
                torque_value = float(torque_value)
            torque_values[torque_field] = torque_value
        tau_command = torque_values["tau_cmd_rotor_nm"]
        tau_feedback = torque_values["tau_feedback_rotor_nm"]
        tau_joint_estimated = torque_values["tau_joint_estimated_nm"]
        if motor == "J6" and any(
            value is not None for value in torque_values.values()
        ):
            raise ValueError("J6 POS_VEL torque fields must be null")
        if tau_joint_estimated is not None:
            if tau_feedback is None:
                raise ValueError(
                    "tau_joint_estimated_nm requires tau_feedback_rotor_nm"
                )
            expected_joint_torque = (
                MOTOR_SPECS[motor].sign
                * tau_feedback
                * MOTOR_SPECS[motor].gear_ratio
            )
            if not math.isclose(
                tau_joint_estimated,
                expected_joint_torque,
                rel_tol=1.0e-9,
                abs_tol=1.0e-9,
            ):
                raise ValueError(
                    "tau_joint_estimated_nm disagrees with rotor feedback"
                )
        if "last_valid_feedback_monotonic_ns" not in item:
            raise ValueError("last_valid_feedback_monotonic_ns is required")
        last_valid_feedback_ns = item["last_valid_feedback_monotonic_ns"]
        if last_valid_feedback_ns is not None and (
            type(last_valid_feedback_ns) is not int
            or last_valid_feedback_ns <= 0
            or last_valid_feedback_ns > receipt_monotonic_ns
        ):
            raise ValueError(
                "last_valid_feedback_monotonic_ns must be a past positive integer or null"
            )
        if item["communication_ok"] and last_valid_feedback_ns is None:
            raise ValueError(
                "communication_ok requires last_valid_feedback_monotonic_ns"
            )
        trajectory_plan_token_id = item.get("trajectory_plan_token_id")
        trajectory_sha256 = item.get("trajectory_sha256")
        if trajectory_plan_token_id == "":
            trajectory_plan_token_id = None
        if trajectory_sha256 == "":
            trajectory_sha256 = None
        for field_name, field_value in (
            ("trajectory_plan_token_id", trajectory_plan_token_id),
            ("trajectory_sha256", trajectory_sha256),
        ):
            if field_value is not None and (
                not isinstance(field_value, str)
                or len(field_value) != 64
                or any(character not in "0123456789abcdef" for character in field_value)
            ):
                raise ValueError(f"{field_name} must be lowercase SHA256 or null")
        if (trajectory_plan_token_id is None) != (trajectory_sha256 is None):
            raise ValueError("trajectory token and hash must be present together")
        trajectory_state = item.get("trajectory_state", "INACTIVE")
        if trajectory_state not in {"PREPARED", "RUNNING", "COMPLETE", "INACTIVE"}:
            raise ValueError("trajectory_state is invalid")
        trajectory_sample_index = item.get("trajectory_sample_index", 0)
        trajectory_interval_count = item.get("trajectory_interval_count", 0)
        if (
            type(trajectory_sample_index) is not int
            or type(trajectory_interval_count) is not int
            or trajectory_sample_index < 0
            or trajectory_interval_count < 0
            or trajectory_sample_index > trajectory_interval_count
        ):
            raise ValueError("trajectory sample index/count is invalid")
        if trajectory_state in {"PREPARED", "RUNNING", "COMPLETE"} and (
            trajectory_plan_token_id is None
            or trajectory_interval_count <= 0
        ):
            raise ValueError("active trajectory state requires identity and samples")
        gravity_feedforward_rotor_nm = item.get(
            "gravity_feedforward_rotor_nm"
        )
        if gravity_feedforward_rotor_nm is not None:
            if (
                type(gravity_feedforward_rotor_nm) not in {int, float}
                or not math.isfinite(float(gravity_feedforward_rotor_nm))
            ):
                raise ValueError(
                    "gravity_feedforward_rotor_nm must be finite or null"
                )
            gravity_feedforward_rotor_nm = float(
                gravity_feedforward_rotor_nm
            )
        thermal_fault_latched = item.get("thermal_fault_latched")
        load_limit_no_progress = item.get("load_limit_no_progress")
        for field_name, field_value in (
            ("thermal_fault_latched", thermal_fault_latched),
            ("load_limit_no_progress", load_limit_no_progress),
        ):
            if field_value is not None and type(field_value) is not bool:
                raise ValueError(f"{field_name} must be boolean or null")
        feedback = MotorFeedback(
            motor=motor,
            position_rad=float(item["position_rad"]),
            velocity_rad_s=float(item["velocity_rad_s"]),
            temperature_c=float(item["temperature_c"]),
            merror=merror,
            communication_ok=item["communication_ok"],
            source_monotonic_ns=source_ns,
            receipt_monotonic_ns=receipt_monotonic_ns,
            tau_cmd_rotor_nm=tau_command,
            tau_feedback_rotor_nm=tau_feedback,
            tau_joint_estimated_nm=tau_joint_estimated,
            last_valid_feedback_monotonic_ns=last_valid_feedback_ns,
            trajectory_plan_token_id=trajectory_plan_token_id,
            trajectory_sha256=trajectory_sha256,
            trajectory_state=trajectory_state,
            trajectory_sample_index=trajectory_sample_index,
            trajectory_interval_count=trajectory_interval_count,
            gravity_feedforward_rotor_nm=gravity_feedforward_rotor_nm,
            thermal_fault_latched=thermal_fault_latched,
            load_limit_no_progress=load_limit_no_progress,
        )
        if not all(math.isfinite(value) for value in (
            feedback.position_rad,
            feedback.velocity_rad_s,
            feedback.temperature_c,
        )):
            raise ValueError(f"non-finite feedback from {motor}")
        feedbacks.append(feedback)
    if frozenset(seen_motors) not in VALID_FEEDBACK_MOTOR_SETS:
        raise ValueError("feedback samples do not form one complete fault domain")
    if seen_motors == {"J6"}:
        validate_j6_feedback_identity(payload, receipt_monotonic_ns)
    if "tau_j2_logical_total_nm" not in payload:
        raise ValueError("tau_j2_logical_total_nm is required")
    reported_j2_total = payload["tau_j2_logical_total_nm"]
    if seen_motors == {"J2A", "J2B"}:
        if type(reported_j2_total) not in {int, float} or not math.isfinite(
            float(reported_j2_total)
        ):
            raise ValueError(
                "tau_j2_logical_total_nm must be finite for the J2 domain"
            )
        estimated_by_motor = {
            sample.motor: sample.tau_joint_estimated_nm
            for sample in feedbacks
        }
        if any(value is None for value in estimated_by_motor.values()):
            raise ValueError(
                "tau_j2_logical_total_nm requires both J2 torque estimates"
            )
        expected_j2_total = sum(
            float(value) for value in estimated_by_motor.values()
        )
        if not math.isclose(
            float(reported_j2_total),
            expected_j2_total,
            rel_tol=1.0e-9,
            abs_tol=1.0e-9,
        ):
            raise ValueError("tau_j2_logical_total_nm disagrees with J2 samples")
    elif reported_j2_total is not None:
        raise ValueError(
            "tau_j2_logical_total_nm must be null outside the J2 domain"
        )
    return tuple(feedbacks)


def _finite_float(value: object, field_name: str) -> float:
    if type(value) not in {int, float} or not math.isfinite(float(value)):
        raise ValueError(f"{field_name} must be a finite number")
    return float(value)


def _non_negative_int(value: object, field_name: str) -> int:
    if type(value) is not int or value < 0:
        raise ValueError(f"{field_name} must be a non-negative integer")
    return value


def _optional_finite_float(value: object, field_name: str) -> Optional[float]:
    if value is None:
        return None
    return _finite_float(value, field_name)


def parse_controller_feedback_metadata(
    payload: Mapping, feedbacks: Iterable[MotorFeedback]
) -> dict[str, dict]:
    """Normalize one worker datagram's controller metadata per motor.

    The motor samples and these records are intended to be committed together
    by the state node.  Older feedback/1.0 producers remain readable, but an
    absent optional metadata group is represented as ``UNKNOWN``/``None``;
    absence is never converted to a healthy latch, a thermal PASS, or an
    authoritative load rating.
    """

    if not isinstance(payload, Mapping):
        raise ValueError("feedback payload must be an object")
    samples = tuple(feedbacks)
    if not samples or len({sample.motor for sample in samples}) != len(samples):
        raise ValueError("controller metadata requires unique motor feedback")
    motors = {sample.motor for sample in samples}

    controller_mode = payload.get("controller_mode", "unknown")
    if controller_mode not in {"brake", "drag", "hold", "position", "unknown"}:
        raise ValueError("controller mode is invalid")
    modes_by_motor = payload.get("controller_mode_by_motor", {})
    if not isinstance(modes_by_motor, Mapping):
        raise ValueError("controller_mode_by_motor must be an object")
    if set(modes_by_motor) != motors:
        raise ValueError(
            "controller_mode_by_motor must contain exactly this domain"
        )
    normalized_modes = {}
    for motor in motors:
        mode = modes_by_motor[motor]
        if mode not in {"brake", "drag", "hold", "position", "unknown"}:
            raise ValueError("controller mode is invalid")
        normalized_modes[motor] = mode

    if "domain_fault" not in payload or type(payload["domain_fault"]) is not bool:
        raise ValueError("domain_fault must be present and boolean")
    domain_fault = payload["domain_fault"]
    lease_value = payload.get("lease_safe_hold")
    if lease_value is not None and type(lease_value) is not bool:
        raise ValueError("lease_safe_hold must be boolean or absent")

    thermal_fields = {
        "thermal_fault_latched",
        "thermal_cooldown_ready",
        "thermal_release_observed",
        "thermal_rearm_pending",
        "thermal_rearm_pending_next_cycle",
        "thermal_cooldown_valid_brake_frames",
        "thermal_trip_activation_epoch",
        "thermal_minimum_rearm_epoch",
        "thermal_state",
        "thermal_state_by_motor",
        "thermal_trip_reason",
        "thermal_config_sha256",
        "thermal_derating_factor",
        "thermal_raw_temperature_c",
        "thermal_window_median_c",
        "thermal_slope_c_per_min",
    }
    thermal_observed = bool(thermal_fields.intersection(payload))
    if thermal_observed:
        required = {
            "thermal_fault_latched",
            "thermal_cooldown_ready",
            "thermal_release_observed",
            "thermal_cooldown_valid_brake_frames",
            "thermal_trip_activation_epoch",
            "thermal_minimum_rearm_epoch",
            "thermal_state",
            "thermal_state_by_motor",
            "thermal_trip_reason",
            "thermal_config_sha256",
        }
        if not required.issubset(payload) or not (
            "thermal_rearm_pending_next_cycle" in payload
            or "thermal_rearm_pending" in payload
        ):
            raise ValueError("thermal controller metadata is incomplete")
        thermal_fault_latched = payload["thermal_fault_latched"]
        thermal_cooldown_ready = payload["thermal_cooldown_ready"]
        thermal_release_observed = payload["thermal_release_observed"]
        thermal_rearm_pending = payload.get(
            "thermal_rearm_pending_next_cycle",
            payload.get("thermal_rearm_pending"),
        )
        for field_name, field_value in (
            ("thermal_fault_latched", thermal_fault_latched),
            ("thermal_cooldown_ready", thermal_cooldown_ready),
            ("thermal_release_observed", thermal_release_observed),
            ("thermal_rearm_pending_next_cycle", thermal_rearm_pending),
        ):
            if type(field_value) is not bool:
                raise ValueError(f"{field_name} must be boolean")
        if (
            "thermal_rearm_pending" in payload
            and "thermal_rearm_pending_next_cycle" in payload
            and payload["thermal_rearm_pending"]
            is not payload["thermal_rearm_pending_next_cycle"]
        ):
            raise ValueError("thermal rearm aliases disagree")
        if "thermal_fault" in payload and (
            type(payload["thermal_fault"]) is not bool
            or payload["thermal_fault"] is not thermal_fault_latched
        ):
            raise ValueError("thermal fault aliases disagree")
        thermal_cooldown_frames = _non_negative_int(
            payload["thermal_cooldown_valid_brake_frames"],
            "thermal_cooldown_valid_brake_frames",
        )
        thermal_trip_epoch = _non_negative_int(
            payload["thermal_trip_activation_epoch"],
            "thermal_trip_activation_epoch",
        )
        thermal_minimum_epoch = _non_negative_int(
            payload["thermal_minimum_rearm_epoch"],
            "thermal_minimum_rearm_epoch",
        )
        reported_thermal_state = payload["thermal_state"]
        if reported_thermal_state not in THERMAL_STATES:
            raise ValueError("thermal_state is invalid")
        thermal_state_by_motor = payload["thermal_state_by_motor"]
        if (
            not isinstance(thermal_state_by_motor, Mapping)
            or set(thermal_state_by_motor) != motors
        ):
            raise ValueError(
                "thermal_state_by_motor must contain exactly this domain"
            )
        normalized_thermal_state_by_motor = {}
        for motor in motors:
            motor_thermal_state = thermal_state_by_motor[motor]
            if motor_thermal_state not in THERMAL_STATES:
                raise ValueError("thermal_state_by_motor contains an invalid state")
            normalized_thermal_state_by_motor[motor] = motor_thermal_state
        thermal_trip_reason = payload["thermal_trip_reason"]
        if thermal_trip_reason not in THERMAL_TRIP_REASONS:
            raise ValueError("thermal_trip_reason is invalid")
        if bool(thermal_trip_reason) is not thermal_fault_latched:
            raise ValueError("thermal trip reason disagrees with latch")
        thermal_config_sha256 = payload["thermal_config_sha256"]
        if thermal_config_sha256 != THERMAL_CONFIG_SHA256:
            raise ValueError("thermal config hash mismatch")
        thermal_derating_factor = payload.get("thermal_derating_factor")
        if thermal_derating_factor is not None:
            thermal_derating_factor = _finite_float(
                thermal_derating_factor, "thermal_derating_factor"
            )
            if not 0.0 <= thermal_derating_factor <= 1.0:
                raise ValueError("thermal_derating_factor is outside [0,1]")
        thermal_raw_temperature_c = _optional_finite_float(
            payload.get("thermal_raw_temperature_c"),
            "thermal_raw_temperature_c",
        )
        thermal_window_median_c = _optional_finite_float(
            payload.get("thermal_window_median_c"),
            "thermal_window_median_c",
        )
        thermal_slope_c_per_min = _optional_finite_float(
            payload.get("thermal_slope_c_per_min"),
            "thermal_slope_c_per_min",
        )
        thermal = {
            "metadata_status": "OBSERVED",
            "fault_latched": thermal_fault_latched,
            "cooldown_ready": thermal_cooldown_ready,
            "release_observed": thermal_release_observed,
            "rearm_pending_next_cycle": thermal_rearm_pending,
            "cooldown_valid_brake_frames": thermal_cooldown_frames,
            "trip_activation_epoch": thermal_trip_epoch,
            "minimum_rearm_epoch": thermal_minimum_epoch,
            "domain_reported_state": reported_thermal_state,
            "reported_state_by_motor": normalized_thermal_state_by_motor,
            "trip_reason": thermal_trip_reason,
            "thermal_config_sha256": thermal_config_sha256,
            "derating_factor": thermal_derating_factor,
            "raw_temperature_c": thermal_raw_temperature_c,
            "window_median_c": thermal_window_median_c,
            "slope_c_per_min": thermal_slope_c_per_min,
        }
    else:
        thermal = {
            "metadata_status": "UNKNOWN",
            "fault_latched": None,
            "cooldown_ready": None,
            "release_observed": None,
            "rearm_pending_next_cycle": None,
            "cooldown_valid_brake_frames": None,
            "trip_activation_epoch": None,
            "minimum_rearm_epoch": None,
            "domain_reported_state": None,
            "reported_state_by_motor": None,
            "reported_state": None,
            "trip_reason": None,
            "thermal_config_sha256": None,
            "derating_factor": None,
            "raw_temperature_c": None,
            "window_median_c": None,
            "slope_c_per_min": None,
        }

    no_progress_fields = {
        "no_progress_fault",
        "load_limit_fault",
        "load_limit_no_progress",
        "no_progress_release_observed",
        "no_progress_rearm_pending_next_cycle",
        "no_progress_watchdog_qualifying_frames",
        "no_progress_observation_valid",
        "no_progress_position_error_rad",
        "no_progress_trip_position_error_rad",
        "position_safety_trip_reason",
        "software_saturation_observed",
        "load_limit_watchdog_authority",
        "no_progress_trip_activation_epoch",
        "no_progress_minimum_rearm_epoch",
    }
    no_progress_observed = bool(no_progress_fields.intersection(payload))
    if no_progress_observed:
        required = {
            "no_progress_fault",
            "load_limit_no_progress",
            "no_progress_release_observed",
            "no_progress_rearm_pending_next_cycle",
            "no_progress_watchdog_qualifying_frames",
            "no_progress_observation_valid",
            "no_progress_position_error_rad",
            "no_progress_trip_position_error_rad",
            "load_limit_watchdog_authority",
            "no_progress_trip_activation_epoch",
            "no_progress_minimum_rearm_epoch",
        }
        if not required.issubset(payload):
            raise ValueError("no-progress controller metadata is incomplete")
        no_progress_fault = payload["no_progress_fault"]
        load_limit_no_progress = payload["load_limit_no_progress"]
        release_observed = payload["no_progress_release_observed"]
        rearm_pending = payload["no_progress_rearm_pending_next_cycle"]
        observation_valid = payload["no_progress_observation_valid"]
        for field_name, field_value in (
            ("no_progress_fault", no_progress_fault),
            ("load_limit_no_progress", load_limit_no_progress),
            ("no_progress_release_observed", release_observed),
            ("no_progress_rearm_pending_next_cycle", rearm_pending),
            ("no_progress_observation_valid", observation_valid),
        ):
            if type(field_value) is not bool:
                raise ValueError(f"{field_name} must be boolean")
        if load_limit_no_progress is not no_progress_fault:
            raise ValueError("no-progress fault aliases disagree")
        if "load_limit_fault" in payload and (
            type(payload["load_limit_fault"]) is not bool
            or payload["load_limit_fault"] is not no_progress_fault
        ):
            raise ValueError("load-limit fault aliases disagree")
        software_saturation = payload.get("software_saturation_observed")
        if software_saturation is not None and type(software_saturation) is not bool:
            raise ValueError("software_saturation_observed must be boolean")
        authority = payload["load_limit_watchdog_authority"]
        if motors == {"J6"}:
            expected_no_progress_authority = J6_NO_PROGRESS_WATCHDOG_AUTHORITY
        elif "J6" not in motors:
            expected_no_progress_authority = LOAD_LIMIT_WATCHDOG_AUTHORITY
        else:
            raise ValueError("no-progress metadata mixes producer domains")
        if authority != expected_no_progress_authority:
            raise ValueError("load-limit watchdog authority is invalid")
        position_error = _finite_float(
            payload["no_progress_position_error_rad"],
            "no_progress_position_error_rad",
        )
        trip_position_error = _finite_float(
            payload["no_progress_trip_position_error_rad"],
            "no_progress_trip_position_error_rad",
        )
        if position_error < 0.0 or trip_position_error < 0.0:
            raise ValueError("no-progress position errors must be non-negative")
        trip_reason = payload.get("position_safety_trip_reason")
        if trip_reason is not None:
            if trip_reason not in {
                "",
                "LOAD_LIMIT_NO_PROGRESS",
                "POSITION_ARRIVAL_TIMEOUT",
                "EXACT_TRAJECTORY_LOAD_GOVERNOR_ABORT",
            }:
                raise ValueError("position safety trip reason is invalid")
            if bool(trip_reason) is not no_progress_fault:
                raise ValueError("position safety trip reason disagrees with latch")
        no_progress = {
            "metadata_status": "OBSERVED",
            "fault_latched": no_progress_fault,
            "release_observed": release_observed,
            "rearm_pending_next_cycle": rearm_pending,
            "qualifying_frames": _non_negative_int(
                payload["no_progress_watchdog_qualifying_frames"],
                "no_progress_watchdog_qualifying_frames",
            ),
            "observation_valid": observation_valid,
            "position_error_rad": position_error,
            "trip_position_error_rad": trip_position_error,
            "trip_reason": trip_reason,
            "software_saturation_observed": software_saturation,
            "watchdog_authority": authority,
            # This wording is a software guard disclaimer, not continuous
            # rotor-load evidence.  Keep that distinction machine-readable.
            "continuous_rating_authoritative": False,
            "trip_activation_epoch": _non_negative_int(
                payload["no_progress_trip_activation_epoch"],
                "no_progress_trip_activation_epoch",
            ),
            "minimum_rearm_epoch": _non_negative_int(
                payload["no_progress_minimum_rearm_epoch"],
                "no_progress_minimum_rearm_epoch",
            ),
        }
    else:
        no_progress = {
            "metadata_status": "UNKNOWN",
            "fault_latched": None,
            "release_observed": None,
            "rearm_pending_next_cycle": None,
            "qualifying_frames": None,
            "observation_valid": None,
            "position_error_rad": None,
            "trip_position_error_rad": None,
            "trip_reason": None,
            "software_saturation_observed": None,
            "watchdog_authority": None,
            "continuous_rating_authoritative": False,
            "trip_activation_epoch": None,
            "minimum_rearm_epoch": None,
        }

    gravity_fields = {
        "gravity_authority_present",
        "gravity_scale",
        "gravity_scale_target",
        "feedforward_nm",
    }
    gravity_identity_fields = {
        "gravity_policy_identity_status",
        "gravity_source_instance_id",
        "gravity_session_id",
        "gravity_state_instance_id",
        "gravity_model_sha256",
        "gravity_config_sha256",
        "gravity_continuous_rotor_limits_authoritative",
    }
    gravity_empirical_identity_fields = {
        "gravity_authority_class",
        "gravity_rating_classification",
        "gravity_empirical_envelope_id",
        "gravity_empirical_envelope_sha256",
        "gravity_empirical_envelope_expires_at_utc",
        "gravity_anchor_sha256",
        "gravity_empirical_stage_index",
        "gravity_empirical_position_validation_authorized",
    }
    gravity_observed = bool(
        (
            gravity_fields
            | gravity_identity_fields
            | gravity_empirical_identity_fields
        ).intersection(payload)
    )
    if gravity_observed:
        if not gravity_fields.issubset(payload):
            raise ValueError("gravity controller metadata is incomplete")
        authority_present = payload["gravity_authority_present"]
        if type(authority_present) is not bool:
            raise ValueError("gravity_authority_present must be boolean")
        gravity_scale = _finite_float(payload["gravity_scale"], "gravity_scale")
        gravity_scale_target = _finite_float(
            payload["gravity_scale_target"], "gravity_scale_target"
        )
        if not 0.0 <= gravity_scale <= 1.0:
            raise ValueError("gravity_scale is outside [0,1]")
        if gravity_scale_target not in GRAVITY_SCALE_TARGET_LEVELS:
            raise ValueError("gravity_scale_target is not a discrete level")
        feedforward = payload["feedforward_nm"]
        if not isinstance(feedforward, (list, tuple)) or len(feedforward) != 6:
            raise ValueError("feedforward_nm must contain six values")
        feedforward_nm = tuple(
            _finite_float(value, "feedforward_nm") for value in feedforward
        )
        if any(
            abs(value) > limit + 1.0e-12
            for value, limit in zip(
                feedforward_nm, GRAVITY_FEEDFORWARD_LIMITS_NM
            )
        ):
            raise ValueError("feedforward_nm exceeds the software envelope")
        if feedforward_nm[5] != 0.0:
            raise ValueError("J6 gravity feedforward must be zero")
        identity_observed = bool(gravity_identity_fields.intersection(payload))
        if identity_observed and not gravity_identity_fields.issubset(payload):
            raise ValueError("gravity policy identity metadata is incomplete")
        empirical_identity_observed = bool(
            gravity_empirical_identity_fields.intersection(payload)
        )
        if (
            empirical_identity_observed
            and not gravity_empirical_identity_fields.issubset(payload)
        ):
            raise ValueError("empirical gravity identity metadata is incomplete")
        identity_status = "NOT_ECHOED"
        source_instance_id = session_id = state_instance_id = None
        model_sha256 = gravity_config_sha256 = None
        continuous_authoritative = None
        authority_class = rating_classification = None
        empirical_envelope_id = empirical_envelope_sha256 = None
        empirical_envelope_expires_at_utc = anchor_sha256 = None
        empirical_stage_index = None
        empirical_position_validation_authorized = None
        if identity_observed:
            identity_status = payload["gravity_policy_identity_status"]
            source_instance_id = payload["gravity_source_instance_id"]
            session_id = payload["gravity_session_id"]
            state_instance_id = payload["gravity_state_instance_id"]
            model_sha256 = payload["gravity_model_sha256"]
            gravity_config_sha256 = payload["gravity_config_sha256"]
            continuous_authoritative = payload[
                "gravity_continuous_rotor_limits_authoritative"
            ]
            if empirical_identity_observed:
                authority_class = payload["gravity_authority_class"]
                rating_classification = payload[
                    "gravity_rating_classification"
                ]
                empirical_envelope_id = payload[
                    "gravity_empirical_envelope_id"
                ]
                empirical_envelope_sha256 = payload[
                    "gravity_empirical_envelope_sha256"
                ]
                empirical_envelope_expires_at_utc = payload[
                    "gravity_empirical_envelope_expires_at_utc"
                ]
                anchor_sha256 = payload["gravity_anchor_sha256"]
                empirical_stage_index = payload[
                    "gravity_empirical_stage_index"
                ]
                empirical_position_validation_authorized = payload[
                    "gravity_empirical_position_validation_authorized"
                ]
            if type(continuous_authoritative) is not bool:
                raise ValueError("gravity continuous authority must be boolean")
            if authority_present:
                if (
                    identity_status != "ECHOED"
                    or not isinstance(source_instance_id, str)
                    or len(source_instance_id) != 32
                    or any(
                        character not in "0123456789abcdef"
                        for character in source_instance_id
                    )
                    or not isinstance(session_id, str)
                    or not 1 <= len(session_id) <= 512
                    or not isinstance(state_instance_id, str)
                    or not 1 <= len(state_instance_id) <= 512
                    or model_sha256 != PRODUCTION_MODEL_SHA256
                    or gravity_config_sha256 != GRAVITY_CONFIG_SHA256
                    or continuous_authoritative not in {True, False}
                    or (
                        continuous_authoritative is False
                        and (
                            not empirical_identity_observed
                            or authority_class
                            != "EMPIRICAL_VALIDATION_ENVELOPE"
                    or rating_classification
                    != "NOT_OFFICIAL_CONTINUOUS_RATING"
                    or not isinstance(empirical_envelope_id, str)
                    or len(empirical_envelope_id) != 38
                    or not empirical_envelope_id.startswith(
                        "v15-31b-empirical-"
                    )
                    or not _valid_sha256(empirical_envelope_sha256)
                    or not isinstance(
                        empirical_envelope_expires_at_utc, str
                    )
                    or not empirical_envelope_expires_at_utc.endswith("Z")
                    or not _valid_sha256(anchor_sha256)
                    or type(empirical_stage_index) is not int
                    or not 0 <= empirical_stage_index < 5
                    or gravity_scale_target
                    != GRAVITY_SCALE_TARGET_LEVELS[empirical_stage_index]
                    or type(empirical_position_validation_authorized)
                    is not bool
                    or (
                        empirical_position_validation_authorized
                        and empirical_stage_index != 4
                    )
                        )
                    )
                    or (
                        continuous_authoritative is True
                        and empirical_identity_observed
                        and (
                            any(
                                value != ""
                                for value in (
                                    authority_class,
                                    rating_classification,
                                    empirical_envelope_id,
                                    empirical_envelope_sha256,
                                    empirical_envelope_expires_at_utc,
                                    anchor_sha256,
                                )
                            )
                            or empirical_stage_index != 0
                            or empirical_position_validation_authorized
                            is not False
                        )
                    )
                ):
                    raise ValueError("gravity policy identity is invalid")
            elif (
                identity_status != "NO_AUTHORITY"
                or any(
                    value != ""
                    for value in (
                        source_instance_id, session_id, state_instance_id,
                        model_sha256, gravity_config_sha256,
                    )
                )
                or continuous_authoritative is not False
                or (
                    empirical_identity_observed
                    and (
                        empirical_stage_index != 0
                        or empirical_position_validation_authorized is not False
                        or any(
                            value != ""
                            for value in (
                                authority_class, rating_classification,
                                empirical_envelope_id,
                                empirical_envelope_sha256,
                                empirical_envelope_expires_at_utc,
                                anchor_sha256,
                            )
                        )
                    )
                )
                or gravity_scale != 0.0
                or gravity_scale_target != 0.0
                or any(value != 0.0 for value in feedforward_nm)
            ):
                raise ValueError("absent gravity authority metadata is inconsistent")
        gravity = {
            "metadata_status": "OBSERVED",
            "policy_identity_status": identity_status,
            "authority_present": authority_present,
            "scale": gravity_scale,
            "scale_target": gravity_scale_target,
            "feedforward_nm": list(feedforward_nm),
            "source_instance_id": source_instance_id,
            "session_id": session_id,
            "state_instance_id": state_instance_id,
            "model_sha256": model_sha256,
            "gravity_config_sha256": gravity_config_sha256,
            "continuous_rotor_limits_authoritative": continuous_authoritative,
            "authority_class": authority_class,
            "rating_classification": rating_classification,
            "empirical_envelope_id": empirical_envelope_id,
            "empirical_envelope_sha256": empirical_envelope_sha256,
            "empirical_envelope_expires_at_utc": (
                empirical_envelope_expires_at_utc
            ),
            "anchor_sha256": anchor_sha256,
            "empirical_stage_index": empirical_stage_index,
            "empirical_position_validation_authorized": (
                empirical_position_validation_authorized
            ),
        }
    else:
        gravity = {
            "metadata_status": "UNKNOWN",
            "policy_identity_status": "UNKNOWN",
            "authority_present": None,
            "scale": None,
            "scale_target": None,
            "feedforward_nm": None,
            "source_instance_id": None,
            "session_id": None,
            "state_instance_id": None,
            "model_sha256": None,
            "gravity_config_sha256": None,
            "continuous_rotor_limits_authoritative": None,
            "authority_class": None,
            "rating_classification": None,
            "empirical_envelope_id": None,
            "empirical_envelope_sha256": None,
            "empirical_envelope_expires_at_utc": None,
            "anchor_sha256": None,
            "empirical_stage_index": None,
            "empirical_position_validation_authorized": None,
        }

    by_motor = {}
    for sample in samples:
        if sample.thermal_fault_latched is not None:
            if not thermal_observed or (
                sample.thermal_fault_latched is not thermal["fault_latched"]
            ):
                raise ValueError("sample thermal latch disagrees with domain")
        elif thermal_observed and sample.motor != "J6":
            raise ValueError("GO sample is missing its thermal latch")
        if sample.load_limit_no_progress is not None:
            if not no_progress_observed or (
                sample.load_limit_no_progress is not no_progress["fault_latched"]
            ):
                raise ValueError("sample no-progress latch disagrees with domain")
        elif no_progress_observed and sample.motor != "J6":
            raise ValueError("GO sample is missing its no-progress latch")

        rotor_feedforward = sample.gravity_feedforward_rotor_nm
        if gravity_observed and sample.motor != "J6":
            if rotor_feedforward is None:
                raise ValueError("GO sample is missing gravity feedforward")
            joint_index = {
                "J1": 0,
                "J2A": 1,
                "J2B": 1,
                "J3": 2,
                "J4": 3,
                "J5": 4,
            }[sample.motor]
            expected = MOTOR_SPECS[sample.motor].sign * gravity[
                "feedforward_nm"
            ][joint_index]
            if abs(rotor_feedforward - expected) > 1.0e-9:
                raise ValueError(
                    "sample gravity feedforward disagrees with domain"
                )
        elif not gravity_observed and rotor_feedforward is not None:
            raise ValueError("sample gravity feedforward lacks domain policy")

        motor_thermal = dict(thermal)
        motor_thermal["reported_state"] = (
            thermal["reported_state_by_motor"][sample.motor]
            if thermal_observed else None
        )
        by_motor[sample.motor] = {
            "source_monotonic_ns": sample.source_monotonic_ns,
            "receipt_monotonic_ns": sample.receipt_monotonic_ns,
            "controller_mode": normalized_modes[sample.motor],
            "domain_fault": domain_fault,
            "lease_safe_hold": lease_value,
            "thermal": motor_thermal,
            "no_progress": dict(no_progress),
            "gravity": {
                **gravity,
                "feedforward_nm": (
                    None
                    if gravity["feedforward_nm"] is None
                    else list(gravity["feedforward_nm"])
                ),
                "applied_rotor_nm": rotor_feedforward,
            },
        }
    return by_motor

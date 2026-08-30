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
    "J4": MotorSpec("joint4", +1, GEAR_RATIO),
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
                "q_joint_rad": joint_position,
                "dq_joint_rad_s": joint_velocity,
                "temperature_c": sample.temperature_c,
                "merror": sample.merror,
                "communication_ok": sample.communication_ok,
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
                "q_joint_rad": joint_position,
                "dq_joint_rad_s": joint_velocity,
                "temperature_c": 0.0 if sample is None else sample.temperature_c,
                "merror": -1 if sample is None else sample.merror,
                "communication_ok": bool(available and sample and sample.communication_ok),
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
            "per_motor": per_motor,
            "available_motors": sorted(self.references),
            "healthy": len(self.references) == len(MOTOR_NAMES) and all(
                value["communication_ok"] and value["fresh"] and value["merror"] == 0
                for value in per_motor.values()
            ),
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
        feedback = MotorFeedback(
            motor=motor,
            position_rad=float(item["position_rad"]),
            velocity_rad_s=float(item["velocity_rad_s"]),
            temperature_c=float(item["temperature_c"]),
            merror=merror,
            communication_ok=item["communication_ok"],
            source_monotonic_ns=source_ns,
            receipt_monotonic_ns=receipt_monotonic_ns,
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
    return tuple(feedbacks)

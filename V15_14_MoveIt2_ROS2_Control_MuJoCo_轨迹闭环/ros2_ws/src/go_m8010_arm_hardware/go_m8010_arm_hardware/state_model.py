"""Pure mapping logic for SESSION_REFERENCE_V1.

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


class MirrorSessionReferenceV1:
    """Capture startup references and produce six relative logical joints."""

    def __init__(
        self,
        capture_samples: int = 25,
        freshness_s: float = 0.10,
        capture_max_span_joint_rad: float = math.radians(0.25),
        capture_max_tail_drift_joint_rad: float = math.radians(0.10),
    ) -> None:
        if capture_samples < 3 or freshness_s <= 0.0:
            raise ValueError("invalid session reference configuration")
        self.capture_samples = capture_samples
        self.freshness_ns = int(freshness_s * 1.0e9)
        self.capture_max_span_joint_rad = capture_max_span_joint_rad
        self.capture_max_tail_drift_joint_rad = capture_max_tail_drift_joint_rad
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
        unwrapped = self.unwrappers[feedback.motor].update(feedback.position_rad)
        self.latest[feedback.motor] = feedback
        self.latest_unwrapped[feedback.motor] = unwrapped
        if feedback.motor not in self.references and feedback.communication_ok and feedback.merror == 0:
            self.history[feedback.motor].append(unwrapped)

    def try_capture_available(self, now_monotonic_ns: int) -> bool:
        """Capture each healthy motor independently so one missing joint cannot block the GUI."""
        for name in MOTOR_NAMES:
            if name in self.references or len(self.history[name]) < self.capture_samples:
                continue
            sample = self.latest.get(name)
            if sample is None or now_monotonic_ns - sample.receipt_monotonic_ns > self.freshness_ns:
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
            now_monotonic_ns - self.latest[name].receipt_monotonic_ns > self.freshness_ns
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
                "fresh": now_monotonic_ns - sample.receipt_monotonic_ns <= self.freshness_ns,
                "age_ms": max(0.0, (now_monotonic_ns - sample.receipt_monotonic_ns) / 1.0e6),
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
            "reference": "SESSION_REFERENCE_V1",
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
                age_ms = max(0.0, (now_monotonic_ns - sample.receipt_monotonic_ns) / 1.0e6)
                fresh = now_monotonic_ns - sample.receipt_monotonic_ns <= self.freshness_ns
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
            "reference": "SESSION_REFERENCE_V1",
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


def parse_feedback_payload(payload: Mapping, receipt_monotonic_ns: int) -> Iterable[MotorFeedback]:
    """Validate the bus-owner JSON contract and yield normalized samples."""

    samples = payload.get("samples")
    if not isinstance(samples, list) or not samples:
        raise ValueError("feedback payload requires a non-empty samples list")
    default_source_ns = int(payload.get("source_monotonic_ns", receipt_monotonic_ns))
    for item in samples:
        if not isinstance(item, Mapping):
            raise ValueError("each feedback sample must be an object")
        motor = str(item["motor"]).upper()
        source_ns = int(item.get("source_monotonic_ns", default_source_ns))
        if source_ns <= 0 or source_ns > receipt_monotonic_ns + 1_000_000:
            raise ValueError("source_monotonic_ns is invalid or from a different clock")
        yield MotorFeedback(
            motor=motor,
            position_rad=float(item["position_rad"]),
            velocity_rad_s=float(item.get("velocity_rad_s", 0.0)),
            temperature_c=float(item.get("temperature_c", 0.0)),
            merror=int(item.get("merror", 0)),
            communication_ok=bool(item.get("communication_ok", True)),
            source_monotonic_ns=source_ns,
            receipt_monotonic_ns=receipt_monotonic_ns,
        )

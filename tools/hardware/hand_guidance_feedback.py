"""Pair existing hardware/gravity snapshots for outer admittance; no device I/O.

The residual is an external-torque estimate, not a force sensor. Bias samples
must be collected during an explicitly hands-off stationary HOLD. A constant
human force cannot be distinguished from model bias using these data alone.
"""
from dataclasses import dataclass
import math
import statistics


MOTOR_GROUPS = (("J1",), ("J2A", "J2B"), ("J3",), ("J4",), ("J5",), ("J6",))


class GuidanceMeasurementUnavailable(ValueError):
    """Valid measurements need refreshing; no replacement data is invented."""

    def __init__(self, reason, diagnostics):
        self.reason, self.diagnostics = reason, diagnostics
        super().__init__(reason)


def _number(value, name):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"INVALID_{name}")
    return float(value)


def _six(value, name):
    if not isinstance(value, (list, tuple)) or len(value) != 6:
        raise ValueError(f"INVALID_{name}")
    return tuple(_number(x, name) for x in value)


def _fresh(stamp, now_ns, maximum_age_ns):
    return type(stamp) is int and 0 < stamp <= now_ns and now_ns - stamp <= maximum_age_ns


@dataclass(frozen=True)
class GuidanceObservation:
    source_monotonic_ns: int
    q: tuple
    dq: tuple
    motor_torque_nm: tuple
    gravity_nm: tuple
    residual_nm: tuple
    holding: bool


def matched_observation(hardware_history, gravity, *, now_ns, maximum_age_ns=100_000_000):
    """None means the matching asynchronous hardware snapshot has not arrived."""
    if type(now_ns) is not int or now_ns <= 0 or not isinstance(gravity, dict):
        raise ValueError("INVALID_GUIDANCE_OBSERVATION")
    if not _fresh(gravity.get("source_monotonic_ns"), now_ns, maximum_age_ns):
        raise ValueError("GRAVITY_OBSERVATION_STALE_OR_FUTURE")
    sequence = gravity.get("hardware_state_sequence")
    if type(sequence) is not int:
        raise ValueError("GRAVITY_HARDWARE_SEQUENCE_INVALID")
    hardware = next((item for item in reversed(hardware_history)
                     if item.get("sequence") == sequence
                     and item.get("session_id") == gravity.get("session_id")
                     and item.get("state_instance_id") == gravity.get("state_instance_id")), None)
    if hardware is None:
        return None
    stamp = hardware.get("source_monotonic_ns")
    if (not _fresh(stamp, now_ns, maximum_age_ns)
            or stamp != gravity.get("hardware_state_source_monotonic_ns")):
        raise ValueError("GRAVITY_HARDWARE_SOURCE_MISMATCH")
    if (hardware.get("healthy") is not True or hardware.get("safety_metadata_ready") is not True
            or hardware.get("j2_sync_fault") is not False
            or abs(_number(hardware.get("j2_e_sync_rad"), "J2_SYNC")) > math.radians(0.25)):
        raise ValueError("GUIDANCE_HARDWARE_UNHEALTHY")
    if (gravity.get("anchor_valid") is not True or gravity.get("production_model_hash_match") is not True
            or gravity.get("finite_bounded") is not True or gravity.get("pose_feasibility") != "PASS"):
        raise ValueError("GUIDANCE_GRAVITY_MAPPING_INVALID")
    # /joint_states is an asynchronous UI mirror, not the atomic pose paired
    # above by hardware sequence/source. Its diagnostic cross-check is no gate.
    q, dq = _six(hardware.get("position_rad"), "Q"), _six(hardware.get("velocity_rad_s"), "DQ")
    predicted = _six(gravity.get("gravity_joint_nm"), "GRAVITY")
    modes, per_motor = hardware.get("controller_mode_by_motor", {}), hardware.get("per_motor", {})
    efforts = []
    sources = {"hardware": stamp, "gravity": gravity["source_monotonic_ns"]}
    pending = []
    diagnostics = {"now_monotonic_ns": now_ns, "source_monotonic_ns": sources, "per_motor": {}}

    def measurement_stamp(value, label):
        if type(value) is not int or not 0 < value <= now_ns:
            raise ValueError(label + "_MISSING_INVALID_OR_FUTURE")
        sources[label] = value
        age = now_ns - value
        if age > 150_000_000:
            raise ValueError(label + "_EXPIRED")
        if age > maximum_age_ns:
            pending.append(label + "_WAITING_FOR_NEW_MEASUREMENT")
        return value

    for index, names in enumerate(MOTOR_GROUPS):
        total = 0.0
        for name in names:
            sample = per_motor.get(name, {})
            diagnostics["per_motor"][name] = {key: sample.get(key) for key in (
                "feedback_source_monotonic_ns", "last_valid_feedback_monotonic_ns",
                "motor_torque_source_monotonic_ns", "motor_torque_estimate_metadata_status")}
            if (sample.get("fresh") is not True or sample.get("communication_ok") is not True
                    or sample.get("merror") != 0 or modes.get(name) not in ("hold", "teach")):
                raise ValueError(f"{name}_GUIDANCE_FEEDBACK_INVALID")
            measurement_stamp(sample.get("feedback_source_monotonic_ns"), name + "_FEEDBACK_SOURCE")
            actual_stamp = measurement_stamp(sample.get("last_valid_feedback_monotonic_ns"), name + "_MEASUREMENT")
            temperature = _number(sample.get("temperature_c"), name + "_TEMPERATURE")
            if not 0 <= temperature < 55:
                raise ValueError(f"{name}_GUIDANCE_TEMPERATURE_LIMIT")
            if index == 5:
                digest = sample.get("motor_torque_qualification_sha256")
                status = sample.get("motor_torque_estimate_metadata_status")
                if (type(status) is not str or status not in {"OBSERVED", "STALE"}
                        or "joint_motor_torque_estimated_nm" not in sample
                        or not isinstance(digest, str) or len(digest) != 64
                        or any(c not in "0123456789abcdef" for c in digest)):
                    raise ValueError("J6_TORQUE_OBSERVATION_UNQUALIFIED")
                torque_stamp = measurement_stamp(sample.get("motor_torque_source_monotonic_ns"), "J6_TORQUE_MEASUREMENT")
                if torque_stamp != actual_stamp:
                    raise ValueError("J6_TORQUE_MEASUREMENT_SOURCE_MISMATCH")
                torque = sample["joint_motor_torque_estimated_nm"]
                if status == "STALE":
                    if torque is not None:
                        raise ValueError("J6_STALE_TORQUE_MUST_BE_UNAVAILABLE")
                    pending.append("J6_TORQUE_WAITING_FOR_NEW_MEASUREMENT")
                else:
                    total += _number(torque, "J6_TORQUE")
            else:
                total += _number(sample.get("tau_joint_estimated_nm"), name + "_TORQUE")
        efforts.append(total)
    # Do not let a temporary gap hide malformed metadata on another motor.
    if pending:
        raise GuidanceMeasurementUnavailable(pending[0], diagnostics)
    return GuidanceObservation(min(sources.values()), q, dq, tuple(efforts), predicted,
        tuple(g - effort for g, effort in zip(predicted, efforts)),
        all(modes.get(name) == "hold" for names in MOTOR_GROUPS for name in names))


def stationary_bias(observations, *, operator_hands_off):
    """Return a frozen median bias and noise MAD, never adapt during guiding."""
    if operator_hands_off is not True:
        raise ValueError("HANDS_OFF_BASELINE_REQUIRED")
    samples = list({sample.source_monotonic_ns: sample for sample in observations}.values())
    samples.sort(key=lambda sample: sample.source_monotonic_ns)
    if len(samples) < 50 or samples[-1].source_monotonic_ns - samples[0].source_monotonic_ns < 1_000_000_000:
        raise ValueError("ONE_SECOND_DISTINCT_BASELINE_REQUIRED")
    if any(not sample.holding or any(abs(v) > math.radians(0.25) for v in sample.dq) for sample in samples):
        raise ValueError("STATIONARY_HOLD_BASELINE_REQUIRED")
    if any(max(s.q[j] for s in samples) - min(s.q[j] for s in samples) > math.radians(0.1) for j in range(6)):
        raise ValueError("BASELINE_POSE_MOVED")
    bias = tuple(statistics.median(s.residual_nm[j] for s in samples) for j in range(6))
    noise = tuple(statistics.median(abs(s.residual_nm[j] - bias[j]) for s in samples) for j in range(6))
    return bias, noise

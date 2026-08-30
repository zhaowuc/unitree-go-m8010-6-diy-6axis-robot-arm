"""与硬件无关的 J6 位置参考与 POS_VEL 保护上限。"""

from __future__ import annotations

import math


MAX_QUINTIC_INTERVALS = 1_000_000


def thermal_derating_factor(
    temperature_c: float,
    derating_start_c: float,
    thermal_stop_c: float,
    minimum_active_factor: float = 0.1,
) -> float:
    """Return the conservative POS_VEL authority retained below thermal stop.

    J6 has no commissioned torque-command interface in this controller, so its
    enforceable thermal output is the unsigned POS_VEL speed cap (and the
    acceleration used to slew that cap).  At/above the stop threshold this
    returns zero; the controller must independently latch and disable the drive.
    """

    values = (
        temperature_c,
        derating_start_c,
        thermal_stop_c,
        minimum_active_factor,
    )
    if not all(
        type(value) in {int, float} and math.isfinite(float(value))
        for value in values
    ):
        raise ValueError("thermal derating inputs must be finite real numbers")
    temperature = float(temperature_c)
    start = float(derating_start_c)
    stop = float(thermal_stop_c)
    floor = float(minimum_active_factor)
    if not start < stop or not 0.0 < floor <= 1.0:
        raise ValueError("thermal derating thresholds are invalid")
    if temperature < start:
        return 1.0
    if temperature >= stop:
        return 0.0
    linear = (stop - temperature) / (stop - start)
    return max(floor, min(1.0, linear))


def thermal_derated_posvel_limits(
    vmax: float,
    amax: float,
    restore_limit: float,
    derating_factor: float,
) -> tuple[float, float, float]:
    """Scale every enforceable J6 POS_VEL output limit by one thermal factor."""

    values = (vmax, amax, restore_limit, derating_factor)
    if not all(
        type(value) in {int, float} and math.isfinite(float(value))
        for value in values
    ):
        raise ValueError("thermal POS_VEL inputs must be finite real numbers")
    velocity = float(vmax)
    acceleration = float(amax)
    restore = float(restore_limit)
    factor = float(derating_factor)
    if (
        velocity <= 0.0
        or acceleration <= 0.0
        or not 0.0 < restore <= velocity
        or not 0.0 <= factor <= 1.0
    ):
        raise ValueError("thermal POS_VEL inputs are outside safe bounds")
    if factor == 0.0:
        return 0.0, 0.0, 0.0
    derated_velocity = velocity * factor
    derated_acceleration = acceleration * factor
    derated_restore = min(restore * factor, derated_velocity)
    return derated_velocity, derated_acceleration, derated_restore


def _strict_positive_int(value: object, name: str) -> int:
    if type(value) is not int or value <= 0:
        raise ValueError(f"{name} must be a positive integer")
    return value


def quintic_sample_index(
    now_monotonic_ns: int,
    execute_at_monotonic_ns: int,
    duration_ns: int,
    interval_count: int,
) -> int:
    """Select one deterministic sample with integer clock arithmetic.

    Before the common start this pins sample zero; at and after the endpoint it
    pins the final sample.  Integer multiplication/division keeps all workers on
    the same sample grid without accumulating a local control-loop timestep.
    """

    if type(now_monotonic_ns) is not int or now_monotonic_ns < 0:
        raise ValueError("now_monotonic_ns must be a non-negative integer")
    execute_at = _strict_positive_int(
        execute_at_monotonic_ns, "execute_at_monotonic_ns"
    )
    duration = _strict_positive_int(duration_ns, "duration_ns")
    intervals = _strict_positive_int(interval_count, "interval_count")
    if intervals > MAX_QUINTIC_INTERVALS:
        raise ValueError("interval_count exceeds the safe bound")
    if now_monotonic_ns <= execute_at:
        return 0
    elapsed_ns = now_monotonic_ns - execute_at
    if elapsed_ns >= duration:
        return intervals
    return (elapsed_ns * intervals) // duration


def quintic_sample(
    start: float,
    target: float,
    duration_ns: int,
    interval_count: int,
    sample_index: int,
) -> tuple[float, float]:
    """Return ``(q_ref, dq_ref)`` for a rest-to-rest quintic sample."""

    if not all(
        type(value) in {int, float} and math.isfinite(float(value))
        for value in (start, target)
    ):
        raise ValueError("quintic endpoints must be finite real numbers")
    start_value = float(start)
    target_value = float(target)
    duration = _strict_positive_int(duration_ns, "duration_ns")
    intervals = _strict_positive_int(interval_count, "interval_count")
    if intervals > MAX_QUINTIC_INTERVALS:
        raise ValueError("interval_count exceeds the safe bound")
    if type(sample_index) is not int or not 0 <= sample_index <= intervals:
        raise ValueError("sample_index is outside the trajectory grid")
    # Exact pins are part of the wire contract; do not depend on polynomial
    # cancellation at u=0/1.
    if sample_index == 0:
        return start_value, 0.0
    if sample_index == intervals:
        return target_value, 0.0
    normalized = sample_index / intervals
    blend = normalized**3 * (
        10.0 + normalized * (-15.0 + 6.0 * normalized)
    )
    derivative = (
        30.0 * normalized**2 * (1.0 - normalized) ** 2
    ) / (duration * 1.0e-9)
    delta = target_value - start_value
    return start_value + delta * blend, delta * derivative


def quintic_reference_at(
    start: float,
    target: float,
    duration_ns: int,
    interval_count: int,
    execute_at_monotonic_ns: int,
    now_monotonic_ns: int,
) -> tuple[float, float, int, str]:
    """Evaluate the common grid and report PREPARED/RUNNING/COMPLETE."""

    index = quintic_sample_index(
        now_monotonic_ns,
        execute_at_monotonic_ns,
        duration_ns,
        interval_count,
    )
    q_ref, dq_ref = quintic_sample(
        start, target, duration_ns, interval_count, index
    )
    if now_monotonic_ns < execute_at_monotonic_ns:
        state = "PREPARED"
    elif index >= interval_count:
        state = "COMPLETE"
    else:
        state = "RUNNING"
    return q_ref, dq_ref, index, state


def quintic_posvel_speed_limit(
    dq_ref: float, vmax: float, restore_limit: float
) -> float:
    """Map signed reference velocity to a non-negative DM POS_VEL cap.

    This value is only a bounded actuator protection limit.  It never changes
    the descriptor-derived position reference and retains a small endpoint
    restore authority for fixed-load holding.
    """

    values = (dq_ref, vmax, restore_limit)
    if not all(
        type(value) in {int, float} and math.isfinite(float(value))
        for value in values
    ):
        raise ValueError("quintic POS_VEL limits must be finite real numbers")
    velocity_limit = float(vmax)
    restore = float(restore_limit)
    if velocity_limit <= 0.0 or not 0.0 <= restore <= velocity_limit:
        raise ValueError("quintic POS_VEL limits are outside safe bounds")
    return min(velocity_limit, max(restore, abs(float(dq_ref))))


def update_profile(
    q_command: float,
    dq_command: float,
    target: float,
    vmax: float,
    amax: float,
    period: float,
) -> tuple[float, float]:
    error = target - q_command
    stopping_speed = math.sqrt(2.0 * amax * abs(error))
    wanted = math.copysign(min(vmax, stopping_speed), error)
    previous_speed = abs(dq_command)
    delta_v = max(-amax * period, min(amax * period, wanted - dq_command))
    dq_command += delta_v
    snap_distance = abs(dq_command) * period + 0.5 * amax * period * period
    if (previous_speed <= amax * period and abs(dq_command) <= amax * period and
            abs(error) <= snap_distance):
        return target, 0.0
    return q_command + dq_command * period, dq_command


def update_posvel_speed_limit(
    current_limit: float,
    actual_position: float,
    target_position: float,
    vmax: float,
    amax: float,
    restore_limit: float,
    period: float,
) -> float:
    """Ramp the positive DM POS_VEL speed cap while keeping ``p_des`` final.

    DM POS_VEL interprets its second float as a non-negative maximum absolute
    speed, not as a signed trajectory velocity.  A positive floor is retained
    at the endpoint so a fixed target can restore an externally displaced J6.
    """

    values = (
        current_limit,
        actual_position,
        target_position,
        vmax,
        amax,
        restore_limit,
        period,
    )
    if not all(math.isfinite(value) for value in values):
        raise ValueError("POS_VEL speed-limit inputs must be finite")
    if vmax <= 0.0 or amax <= 0.0 or period <= 0.0:
        raise ValueError("POS_VEL vmax, amax, and period must be positive")
    if not 0.0 < restore_limit <= vmax:
        raise ValueError("POS_VEL restore limit must be positive and at most vmax")

    current = max(0.0, float(current_limit))
    error = abs(target_position - actual_position)
    braking_limit = math.sqrt(2.0 * amax * error)
    wanted = min(vmax, max(restore_limit, braking_limit))
    maximum_change = amax * period
    updated = current + max(
        -maximum_change,
        min(maximum_change, wanted - current),
    )
    return min(vmax, max(0.0, updated))

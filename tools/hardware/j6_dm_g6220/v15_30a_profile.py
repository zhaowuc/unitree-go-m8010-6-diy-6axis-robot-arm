"""与硬件无关的有界梯形位置轨迹。"""

from __future__ import annotations

import math


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

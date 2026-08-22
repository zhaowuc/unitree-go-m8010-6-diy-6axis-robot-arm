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

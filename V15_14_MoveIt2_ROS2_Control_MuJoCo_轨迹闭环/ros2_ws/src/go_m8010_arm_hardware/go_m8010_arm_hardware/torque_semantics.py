"""Explicit GO-M8010 torque units and reducer-side conversions.

All values accepted or returned by this module include the physical side in
their name.  The GO SDK ``MotorCmd.tau`` and ``MotorData.tau`` fields are
motor-rotor torque in N.m.  The wire representation is signed Q8 and the
frozen SDK clamps command counts to +/-32765.

This module is deliberately independent of ROS, MuJoCo, and device I/O so the
unit authority can be regression-tested while the arm is powered off.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


GO_GEAR_RATIO = 6.329999923706055
GO_TAU_Q8_COUNTS_PER_NM = 256
GO_TAU_Q8_EFFECTIVE_COUNT_LIMIT = 32765
J2A_ROTOR_TO_JOINT_SIGN = -1
J2B_ROTOR_TO_JOINT_SIGN = +1


def _finite(value: float, name: str) -> float:
    converted = float(value)
    if not math.isfinite(converted):
        raise ValueError(f"{name} must be finite")
    return converted


def _gear_ratio(value: float) -> float:
    ratio = _finite(value, "gear_ratio")
    if ratio <= 0.0:
        raise ValueError("gear_ratio must be positive")
    return ratio


def _direction_sign(value: int) -> int:
    if type(value) is not int or value not in {-1, +1}:
        raise ValueError("direction_sign must be exactly -1 or +1")
    return value


def encode_go_tau_q8(tau_cmd_rotor_nm: float) -> int:
    """Mirror the frozen GO SDK's saturating, toward-zero Q8 serializer."""

    torque = _finite(tau_cmd_rotor_nm, "tau_cmd_rotor_nm")
    raw_count = math.trunc(torque * GO_TAU_Q8_COUNTS_PER_NM)
    return max(
        -GO_TAU_Q8_EFFECTIVE_COUNT_LIMIT,
        min(GO_TAU_Q8_EFFECTIVE_COUNT_LIMIT, raw_count),
    )


def decode_go_tau_feedback(raw_q8_count: int) -> float:
    """Decode a signed int16 feedback count to rotor-side N.m."""

    if type(raw_q8_count) is not int or not -(1 << 15) <= raw_q8_count < (1 << 15):
        raise ValueError("raw_q8_count must be a signed int16")
    return raw_q8_count / GO_TAU_Q8_COUNTS_PER_NM


def joint_torque_to_rotor_torque(
    tau_joint_nm: float,
    *,
    direction_sign: int,
    gear_ratio: float = GO_GEAR_RATIO,
    load_share: float = 1.0,
) -> float:
    """Map logical joint torque to one motor's rotor-side command torque."""

    joint_torque = _finite(tau_joint_nm, "tau_joint_nm")
    share = _finite(load_share, "load_share")
    if not 0.0 <= share <= 1.0:
        raise ValueError("load_share must be within [0, 1]")
    return (
        _direction_sign(direction_sign)
        * joint_torque
        * share
        / _gear_ratio(gear_ratio)
    )


def rotor_torque_to_joint_torque(
    tau_feedback_rotor_nm: float,
    *,
    direction_sign: int,
    gear_ratio: float = GO_GEAR_RATIO,
) -> float:
    """Map one motor's rotor feedback to its signed logical-joint effort."""

    return (
        _direction_sign(direction_sign)
        * _finite(tau_feedback_rotor_nm, "tau_feedback_rotor_nm")
        * _gear_ratio(gear_ratio)
    )


@dataclass(frozen=True)
class J2RotorTorqueSplit:
    tau_cmd_j2a_rotor_nm: float
    tau_cmd_j2b_rotor_nm: float


def split_j2_joint_torque(
    tau_j2_logical_total_nm: float,
    *,
    gear_ratio: float = GO_GEAR_RATIO,
) -> J2RotorTorqueSplit:
    """Split one logical J2 torque equally across its opposed GO motors."""

    return J2RotorTorqueSplit(
        tau_cmd_j2a_rotor_nm=joint_torque_to_rotor_torque(
            tau_j2_logical_total_nm,
            direction_sign=J2A_ROTOR_TO_JOINT_SIGN,
            gear_ratio=gear_ratio,
            load_share=0.5,
        ),
        tau_cmd_j2b_rotor_nm=joint_torque_to_rotor_torque(
            tau_j2_logical_total_nm,
            direction_sign=J2B_ROTOR_TO_JOINT_SIGN,
            gear_ratio=gear_ratio,
            load_share=0.5,
        ),
    )


def j2_rotor_feedback_to_logical_total(
    tau_feedback_j2a_rotor_nm: float,
    tau_feedback_j2b_rotor_nm: float,
    *,
    gear_ratio: float = GO_GEAR_RATIO,
) -> float:
    """Combine both rotor feedback values into total logical J2 torque."""

    return rotor_torque_to_joint_torque(
        tau_feedback_j2a_rotor_nm,
        direction_sign=J2A_ROTOR_TO_JOINT_SIGN,
        gear_ratio=gear_ratio,
    ) + rotor_torque_to_joint_torque(
        tau_feedback_j2b_rotor_nm,
        direction_sign=J2B_ROTOR_TO_JOINT_SIGN,
        gear_ratio=gear_ratio,
    )

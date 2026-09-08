"""Pure J6 protocol-scale qualification; no transport or hardware imports.

Damiao protocol V1.4 section 4.1 and appendix define TMAX in Nm for feedback
in every mode. The decoded quantity is the driver's motor-torque estimate,
not an independently measured external torque or current.
"""
from __future__ import annotations

import hashlib
import json
import math
from collections.abc import Mapping

QUERY_RIDS = (7, 8, 10, 21, 22, 23, 1, 20)
RID_NAMES = {7: "master_id", 8: "motor_id", 10: "control_mode", 21: "pmax_rad",
             22: "vmax_rad_s", 23: "tmax_nm", 1: "kt_out_nm_per_a", 20: "gear_ratio"}
EXPECTED_READBACK = {"7": 0, "8": 1, "10": 2, "21": 12.5, "22": 45.0, "23": 10.0}


def readback_sha256(readback: Mapping) -> str:
    return hashlib.sha256(json.dumps(dict(readback), sort_keys=True,
        separators=(",", ":"), allow_nan=False).encode("ascii")).hexdigest()


def qualify_readback(readback: Mapping, *, expected_sha256: str,
                     protocol_to_joint_sign: int = -1) -> dict:
    if not isinstance(readback, Mapping) or set(readback) != {str(r) for r in QUERY_RIDS}:
        raise ValueError("J6_READBACK_FIELDS_INVALID")
    if type(protocol_to_joint_sign) is not int or protocol_to_joint_sign != -1:
        raise ValueError("J6_PROTOCOL_TO_JOINT_SIGN_MISMATCH")
    if readback_sha256(readback) != expected_sha256:
        raise ValueError("J6_READBACK_SHA256_MISMATCH")
    for key, value in readback.items():
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError("J6_READBACK_NONFINITE_OR_TYPE_INVALID")
        if key in EXPECTED_READBACK and not math.isclose(value, EXPECTED_READBACK[key], rel_tol=0, abs_tol=1e-6):
            raise ValueError(f"J6_RID_{key}_MISMATCH")
    # KT_OUT=0 is the documented auto/theoretical coefficient, not an error.
    if readback["1"] < 0 or readback["20"] <= 0:
        raise ValueError("J6_KT_OR_GEAR_INVALID")
    return {"schema": "go-m8010-j6-protocol-torque-qualification/1.0",
        "readback_sha256": expected_sha256, "control_mode": "POS_VEL",
        "tmax_nm": float(readback["23"]), "protocol_to_joint_sign": -1,
        "kt_out_nm_per_a": float(readback["1"]), "gear_ratio": float(readback["20"]),
        "accuracy_not_physically_calibrated": True, "external_torque_measured": False,
        "independent_current_measured": False, "continuous_rating_authority": False}


def signed_motor_torque_estimate(protocol_torque: float, readback: Mapping, *,
                                expected_readback_sha256: str,
                                protocol_to_joint_sign: int = -1) -> dict:
    qualification = qualify_readback(readback, expected_sha256=expected_readback_sha256,
                                    protocol_to_joint_sign=protocol_to_joint_sign)
    if type(protocol_torque) not in (int, float) or not math.isfinite(protocol_torque) or abs(protocol_torque) > qualification["tmax_nm"]:
        raise ValueError("J6_PROTOCOL_TORQUE_INVALID")
    return {**qualification, "motor_torque_estimated_nm": float(protocol_torque),
            "joint_motor_torque_estimated_nm": -float(protocol_torque)}


def validate_readonly_request(can_id: int, payload: bytes) -> None:
    parameter_read = (len(payload) == 8 and payload[:3] == bytes((1, 0, 0x33))
                      and payload[3] in QUERY_RIDS and payload[4:] == bytes(4))
    refresh = payload == bytes((1, 0, 0xCC, 0, 0, 0, 0, 0))
    if can_id != 0x7FF or not (parameter_read or refresh):
        raise ValueError("J6_READONLY_QUERY_FORBIDS_CONTROL_OR_PARAMETER_WRITE")

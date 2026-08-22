#!/usr/bin/env python3
"""Fail-closed analyzer and phase-gate writer for V15.24F-FT."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
import struct
import sys
from collections import Counter
from pathlib import Path


DEG = 180.0 / math.pi
RAD = math.pi / 180.0
GEAR = 6.3299999237060547
PERIOD_S = 0.01
VMAX_RAD_S = 10.0 * RAD
ACCEL_RAD_S2 = 30.0 * RAD
CAPTURE_SPAN_LIMIT_DEG = 0.2
CAPTURE_TAIL_MEDIAN_LIMIT_DEG = 0.1
SOURCE_HEAD = "5129a975d5c456aa04fed2e7ff47456a56785489"
GATE_SCHEMA = "V15_24F_NEXT_PHASE_GATE_V1"

FIELDS = (
    "tick,timestamp_s,phase,level,target_kp,active_kp,"
    "kp_cmd_count,kp_cmd_decoded,kd_cmd_count,kd_cmd_decoded,"
    "tff_nm,a_tff_count,b_tff_count,"
    "a_mode,b_mode,a_q_cmd,b_q_cmd,a_dq_cmd,b_dq_cmd,"
    "a_raw,b_raw,qA_logical_rad,qB_logical_rad,qJ2_logical_rad,"
    "q_ref_rad,e_common_rad,e_sync_rad,a_dq_feedback,b_dq_feedback,"
    "a_dq_logical,b_dq_logical,a_tau,b_tau,tau_J2_feedback,"
    "expected_A_PD_tau,expected_B_PD_tau,"
    "a_temp,b_temp,a_merror,b_merror,a_received_id,b_received_id,"
    "a_send_recv,b_send_recv,a_correct,b_correct,a_crc_ok,b_crc_ok,"
    "a_valid,b_valid,cycle_period_ms,cycle_jitter_ms"
).split(",")

J2_TERMINATION_FIELDS = ["termination_reason"]
FIELDS_WITH_TERMINATION = FIELDS[:-2] + J2_TERMINATION_FIELDS + FIELDS[-2:]

CONFIGS = {
    "j2_kp100_run1.csv": {
        "phase": "j2-kp100-run1", "kp": 1.00, "ramp_rows": 51,
        "level": "J2_KP100", "kind": "run1",
    },
    "j2_kp100_repeat.csv": {
        "phase": "j2-kp100-repeat", "kp": 1.00, "ramp_rows": 51,
        "level": "J2_KP100", "kind": "repeat",
    },
    "j2_kp140_run1.csv": {
        "phase": "j2-kp140-run1", "kp": 1.40, "ramp_rows": 76,
        "level": "J2_KP140", "kind": "run1",
    },
    "j2_kp140_repeat.csv": {
        "phase": "j2-kp140-repeat", "kp": 1.40, "ramp_rows": 76,
        "level": "J2_KP140", "kind": "repeat",
    },
}

J3_FIELDS = (
    "tick,timestamp_s,phase,run_label,kp_cmd_literal,kp_cmd_count,"
    "kp_cmd_decoded,kd_cmd_literal,kd_cmd_count,kd_cmd_decoded,"
    "tff_cmd_literal,tff_cmd_count,tff_cmd_decoded,j3_mode,"
    "q_cmd_motor_rad,dq_cmd_motor_rad_s,q_feedback_motor_rad,"
    "q_relative_ros_rad,q_ref_rad,position_error_rad,expected_pd_tau_nm,"
    "dq_feedback_motor_rad_s,dq_logical_rad_s,fast_velocity_deg_s,"
    "slow_velocity_deg_s,tau_feedback_nm,"
    "logical_tau_feedback_nm,temperature_c,merror,received_id,"
    "send_recv,correct,crc_ok,valid,abort_reason,"
    "j4_topology,j4_mode,j4_temperature_c,j4_merror,j4_received_id,"
    "j4_send_recv,j4_correct,j4_crc_ok,j4_valid,"
    "j5_topology,j5_mode,j5_temperature_c,j5_merror,j5_received_id,"
    "j5_send_recv,j5_correct,j5_crc_ok,j5_valid,"
    "final_brake_row_pass,cycle_period_ms,cycle_jitter_ms"
).split(",")
J3_ACCEPTED_FIELDS = (J3_FIELDS,)

J3_CONFIGS = {
    "j3_installed_run.csv": {
        "phase": "j3-kp060-run1", "kp": 0.60, "kind": "run1",
    },
    "j3_installed_repeat.csv": {
        "phase": "j3-kp060-repeat", "kp": 0.60, "kind": "repeat",
    },
    "j3_kp070_run.csv": {
        "phase": "j3-kp070-run1", "kp": 0.70, "kind": "run1",
    },
    "j3_kp070_repeat.csv": {
        "phase": "j3-kp070-repeat", "kp": 0.70, "kind": "repeat",
    },
}

ROUTE_PHASES = (
    "PLUS_5_PROFILE", "PLUS_5_ENDPOINT",
    "FIRST_CENTER_PROFILE", "FIRST_CENTER_ENDPOINT",
    "MINUS_5_PROFILE", "MINUS_5_ENDPOINT",
    "FINAL_CENTER_PROFILE", "FINAL_CENTER_ENDPOINT",
)

ENDPOINT_COUNTS = {
    "PLUS_5_ENDPOINT": 40,
    "FIRST_CENTER_ENDPOINT": 40,
    "MINUS_5_ENDPOINT": 40,
    "FINAL_CENTER_ENDPOINT": 50,
}
ENDPOINT_TOLERANCES = {
    "PLUS_5_ENDPOINT": 1.0,
    "FIRST_CENTER_ENDPOINT": 0.75,
    "MINUS_5_ENDPOINT": 1.0,
    "FINAL_CENTER_ENDPOINT": 0.75,
}
ENDPOINT_REASON_PREFIX = {
    "PLUS_5_ENDPOINT": "PLUS_5",
    "FIRST_CENTER_ENDPOINT": "FIRST_CENTER",
    "MINUS_5_ENDPOINT": "MINUS_5",
    "FINAL_CENTER_ENDPOINT": "FINAL_CENTER",
}


def finite(value: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"nonfinite value: {value}")
    return result


def maybe_float(value: str) -> float | None:
    try:
        result = float(value)
    except ValueError:
        return None
    return result if math.isfinite(result) else None


def is_nan(value: str) -> bool:
    try:
        return math.isnan(float(value))
    except (TypeError, ValueError):
        return False


def close(actual: float, expected: float, tolerance: float = 1e-7) -> bool:
    return math.isfinite(actual) and abs(actual - expected) <= tolerance


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_csv_strict(
        path: Path, accepted_fields: tuple[list[str], ...]) -> tuple[list[str], list[dict[str, str]]]:
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.DictReader(stream)
        fieldnames = reader.fieldnames
        if fieldnames is None or fieldnames not in accepted_fields:
            raise ValueError("CSV schema mismatch")
        if len(fieldnames) != len(set(fieldnames)):
            raise ValueError("CSV duplicate column")
        rows = list(reader)
    expected_keys = set(fieldnames)
    for row_number, row in enumerate(rows, start=2):
        if None in row or set(row) != expected_keys or any(
                value is None for value in row.values()):
            raise ValueError(f"CSV row arity mismatch at line {row_number}")
    if not rows:
        raise ValueError("CSV empty")
    return fieldnames, rows


def float32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", value))[0]


def sdk_gain_count(value: float) -> int:
    if not math.isfinite(value) or value < 0.0:
        raise ValueError("gain outside encoder domain")
    return int(float32(value) * 1280.0)


def smoothstep5(value: float) -> float:
    value = min(1.0, max(0.0, value))
    return value ** 3 * (10.0 + value * (-15.0 + 6.0 * value))


def trapezoid_sample(displacement: float, time_s: float) -> tuple[float, float]:
    sign = -1.0 if displacement < 0.0 else 1.0
    distance = abs(displacement)
    threshold = VMAX_RAD_S * VMAX_RAD_S / ACCEL_RAD_S2
    if distance <= threshold:
        t_acc = math.sqrt(distance / ACCEL_RAD_S2)
        v_peak = ACCEL_RAD_S2 * t_acc
        t_cruise = 0.0
    else:
        t_acc = VMAX_RAD_S / ACCEL_RAD_S2
        v_peak = VMAX_RAD_S
        t_cruise = (distance - ACCEL_RAD_S2 * t_acc * t_acc) / VMAX_RAD_S
    duration = 2.0 * t_acc + t_cruise
    time_s = min(duration, max(0.0, time_s))
    d_acc = 0.5 * ACCEL_RAD_S2 * t_acc * t_acc
    if time_s < t_acc:
        position = 0.5 * ACCEL_RAD_S2 * time_s * time_s
        velocity = ACCEL_RAD_S2 * time_s
    elif time_s < t_acc + t_cruise:
        elapsed = time_s - t_acc
        position = d_acc + v_peak * elapsed
        velocity = v_peak
    elif time_s < duration:
        remaining = duration - time_s
        position = distance - 0.5 * ACCEL_RAD_S2 * remaining * remaining
        velocity = ACCEL_RAD_S2 * remaining
    else:
        position = distance
        velocity = 0.0
    return sign * position, sign * velocity


def expected_reference(phase: str, phase_index: int) -> tuple[float, float]:
    fixed = {
        "KP_RAMP_HOLD": 0.0,
        "CURRENT_HOLD": 0.0,
        "PLUS_5_ENDPOINT": 5.0 * RAD,
        "FIRST_CENTER_ENDPOINT": 0.0,
        "MINUS_5_ENDPOINT": -5.0 * RAD,
        "FINAL_CENTER_ENDPOINT": 0.0,
    }
    if phase in fixed:
        return fixed[phase], 0.0
    profiles = {
        "PLUS_5_PROFILE": (0.0, 5.0 * RAD),
        "FIRST_CENTER_PROFILE": (5.0 * RAD, -5.0 * RAD),
        "MINUS_5_PROFILE": (0.0, -5.0 * RAD),
        "FINAL_CENTER_PROFILE": (-5.0 * RAD, 5.0 * RAD),
    }
    if phase not in profiles:
        raise ValueError(f"unexpected active phase: {phase}")
    start, displacement = profiles[phase]
    delta, velocity = trapezoid_sample(displacement, phase_index * PERIOD_S)
    return start + delta, velocity


def contiguous_segments(rows: list[dict[str, str]]) -> list[tuple[str, int]]:
    segments: list[list[object]] = []
    for row in rows:
        phase = row["phase"]
        if not segments or segments[-1][0] != phase:
            segments.append([phase, 1])
        else:
            segments[-1][1] = int(segments[-1][1]) + 1
    return [(str(name), int(count)) for name, count in segments]


def capture_static_result(rows: list[dict[str, str]], fields: tuple[str, ...],
                          gear: float = GEAR) -> tuple[bool, dict[str, float]]:
    metrics: dict[str, float] = {}
    passed = bool(rows)
    for field in fields:
        values = [finite(row[field]) for row in rows]
        span_deg = (max(values) - min(values)) / gear * DEG
        full_median = statistics.median(values)
        tail_count = min(10, len(values))
        tail_offset_deg = abs(statistics.median(values[-tail_count:]) -
                              full_median) / gear * DEG
        metrics[f"{field}_span_deg"] = span_deg
        metrics[f"{field}_tail_median_offset_deg"] = tail_offset_deg
        passed = passed and span_deg <= CAPTURE_SPAN_LIMIT_DEG and \
            tail_offset_deg <= CAPTURE_TAIL_MEDIAN_LIMIT_DEG
    return passed, metrics


def median_field(rows: list[dict[str, str]], field: str, tail: int) -> float:
    if len(rows) < tail:
        raise ValueError(f"{field}: endpoint has {len(rows)} rows, needs {tail}")
    return statistics.median(finite(row[field]) for row in rows[-tail:])


def endpoint(rows_by_phase: dict[str, list[dict[str, str]]], phase: str,
             target_deg: float, tail: int) -> dict[str, float] | None:
    values = rows_by_phase.get(phase)
    if not values or len(values) < tail:
        return None
    actual = median_field(values, "qJ2_logical_rad", tail) * DEG
    sync = max(abs(finite(row["e_sync_rad"])) * DEG
               for row in values[-tail:])
    return {
        "actual_deg": actual,
        "error_deg": abs(actual - target_deg),
        "e_sync_deg": sync,
    }


def j3_endpoint(rows_by_phase: dict[str, list[dict[str, str]]], phase: str,
                target_deg: float, tail: int) -> dict[str, float] | None:
    values = rows_by_phase.get(phase)
    if not values or len(values) < tail:
        return None
    actual = median_field(values, "q_relative_ros_rad", tail) * DEG
    return {"actual_deg": actual, "error_deg": abs(actual - target_deg)}


def checkpoint_failed(summary: dict[str, float] | None,
                      tolerance: float) -> bool:
    if summary is None:
        return False
    endpoint_sync = summary.get("e_sync_deg")
    cumulative_sync = summary.get("cumulative_max_e_sync_deg")
    return bool(summary["error_deg"] > tolerance or
                (endpoint_sync is not None and endpoint_sync > 0.3) or
                (cumulative_sync is not None and cumulative_sync > 0.7))


def fail_closed_center_gate(
        completed_names: list[str],
        checkpoints: list[tuple[str, dict[str, float] | None, float]]) -> bool:
    route_index = {name: index for index, name in enumerate(ROUTE_PHASES)}
    for name, summary, tolerance in checkpoints:
        if not checkpoint_failed(summary, tolerance):
            continue
        checkpoint_index = route_index[name]
        if any(candidate in route_index and
               route_index[candidate] > checkpoint_index
               for candidate in completed_names):
            return False
    return "MOVE_TO_TEST_CENTER_PROFILE" not in completed_names and \
        "MOVE_TO_TEST_CENTER_ENDPOINT" not in completed_names


def timing_summary(all_rows: list[dict[str, str]],
                   timed_rows: list[dict[str, str]],
                   logged_scope: str) -> dict[str, object]:
    errors: list[str] = []
    try:
        all_timestamps = [finite(row["timestamp_s"]) for row in all_rows]
    except ValueError as error:
        return {"result": "FAIL", "samples": 0, "median_ms": None,
                "p95_ms": None, "max_ms": None,
                "errors": [f"timestamp:{error}"]}
    if any(right <= left for left, right in
           zip(all_timestamps, all_timestamps[1:])):
        errors.append("TIMESTAMP_NOT_STRICTLY_INCREASING")

    logged_rows = all_rows if logged_scope == "all" else timed_rows
    logged_ids = {id(row) for row in logged_rows}
    for index, row in enumerate(logged_rows):
        period = maybe_float(row["cycle_period_ms"])
        jitter = maybe_float(row["cycle_jitter_ms"])
        if index == 0:
            if not is_nan(row["cycle_period_ms"]) or \
               not is_nan(row["cycle_jitter_ms"]):
                errors.append("FIRST_LOGGED_TIMING_FIELDS_NOT_NAN")
            continue
        expected = (finite(row["timestamp_s"]) -
                    finite(logged_rows[index - 1]["timestamp_s"])) * 1000.0
        if period is None or jitter is None or not close(period, expected, 1e-8) or \
           not close(jitter, abs(expected - 10.0), 1e-8):
            errors.append(f"TIMING_FIELD_MISMATCH:{row['tick']}")
            break
    if logged_scope == "active":
        for row in all_rows:
            if id(row) not in logged_ids and (
                    not is_nan(row["cycle_period_ms"]) or
                    not is_nan(row["cycle_jitter_ms"])):
                errors.append(f"NONACTIVE_TIMING_FIELDS_NOT_NAN:{row['tick']}")
                break

    periods = [
        (finite(right["timestamp_s"]) - finite(left["timestamp_s"])) * 1000.0
        for left, right in zip(timed_rows, timed_rows[1:])
    ]
    if not periods:
        return {"result": "FAIL", "samples": 0, "median_ms": None,
                "p95_ms": None, "max_ms": None,
                "errors": errors + ["NO_TIMING_SAMPLES"]}
    ordered = sorted(periods)
    p95 = ordered[max(0, math.ceil(0.95 * len(ordered)) - 1)]
    median = statistics.median(ordered)
    maximum = max(ordered)
    passed = not errors and 9.0 <= median <= 11.5 and \
        p95 <= 20.0 and maximum <= 100.0
    return {"result": "PASS" if passed else "FAIL", "samples": len(periods),
            "median_ms": median, "p95_ms": p95, "max_ms": maximum,
            "errors": errors}


def termination_summary(rows: list[dict[str, str]], fieldnames: list[str],
                        final_rows: list[dict[str, str]],
                        reason_field: str = "termination_reason") -> dict[str, object]:
    if reason_field not in fieldnames:
        return {"field_present": False, "reason": None,
                "safety_abort_latched": None, "consistent": False,
                "final_rows_complete": False}
    meaningful = [row[reason_field].strip() for row in rows
                  if row[reason_field].strip() not in
                  {"", "NONE", "RUNNING", "N/A"}]
    reasons = list(dict.fromkeys(meaningful))
    latch_values: list[int] = []
    if "safety_abort_latched" in fieldnames:
        try:
            latch_values = [int(row["safety_abort_latched"]) for row in rows]
        except ValueError:
            return {"field_present": True, "reason": None,
                    "safety_abort_latched": None, "consistent": False,
                    "final_rows_complete": False}
    reason = reasons[-1] if reasons else None
    final_reasons = [row[reason_field].strip() for row in final_rows]
    return {
        "field_present": True,
        "reason": reason,
        "safety_abort_latched": any(value != 0 for value in latch_values),
        "consistent": len(reasons) <= 1 and
                      all(value in (0, 1) for value in latch_values),
        "final_rows_complete": len(final_rows) == 5 and reason is not None and
                               all(value == reason for value in final_reasons),
    }


def exact_failed_endpoint(
        segments: list[tuple[str, int]],
        checkpoints: list[tuple[str, dict[str, float] | None, float]],
        termination: dict[str, object]) -> str | None:
    if len(segments) < 2:
        return None
    name, count = segments[-2]
    if name not in ENDPOINT_COUNTS or count != ENDPOINT_COUNTS[name]:
        return None
    summaries = {checkpoint_name: (summary, tolerance)
                 for checkpoint_name, summary, tolerance in checkpoints}
    summary, tolerance = summaries[name]
    if not checkpoint_failed(summary, tolerance):
        return None
    if not termination["field_present"] or not termination["consistent"] or \
       not termination["final_rows_complete"] or \
       termination["safety_abort_latched"]:
        return None
    reason = termination["reason"]
    expected_prefix = ENDPOINT_REASON_PREFIX[name]
    expected_reason = f"{expected_prefix}_ACCEPTANCE_FAILED_STOP_ROUTE"
    return name if reason == expected_reason else None


def audit_phase_prefix(segments: list[tuple[str, int]], ramp_rows: int) -> tuple[bool, str]:
    expected = [
        ("SESSION_BRAKE_CAPTURE", 50),
        ("KP_RAMP_HOLD", ramp_rows),
        ("CURRENT_HOLD", 100),
        ("PLUS_5_PROFILE", 85),
        ("PLUS_5_ENDPOINT", 40),
        ("FIRST_CENTER_PROFILE", 85),
        ("FIRST_CENTER_ENDPOINT", 40),
        ("MINUS_5_PROFILE", 85),
        ("MINUS_5_ENDPOINT", 40),
        ("FINAL_CENTER_PROFILE", 85),
        ("FINAL_CENTER_ENDPOINT", 50),
    ]
    if not segments or segments[-1] != ("FINAL_DUAL_BRAKE", 5):
        return False, "FINAL_DUAL_BRAKE_NOT_EXACT_5"
    active = segments[:-1]
    if not active or len(active) > len(expected):
        return False, "ACTIVE_PHASE_PREFIX_LENGTH_INVALID"
    for index, (name, count) in enumerate(active):
        expected_name, expected_count = expected[index]
        if name != expected_name:
            return False, f"PHASE_ORDER_INVALID:{name}:expected:{expected_name}"
        if count != expected_count:
            if index != len(active) - 1 or not (1 <= count <= expected_count):
                return False, f"PHASE_COUNT_INVALID:{name}:{count}:{expected_count}"
    return True, "PASS"


def audit_j3_phase_prefix(segments: list[tuple[str, int]]) -> tuple[bool, str]:
    expected = [
        ("SESSION_BRAKE_CAPTURE", 50),
        ("CURRENT_HOLD", 150),
        ("PLUS_5_PROFILE", 85),
        ("PLUS_5_ENDPOINT", 40),
        ("FIRST_CENTER_PROFILE", 85),
        ("FIRST_CENTER_ENDPOINT", 40),
        ("MINUS_5_PROFILE", 85),
        ("MINUS_5_ENDPOINT", 40),
        ("FINAL_CENTER_PROFILE", 85),
        ("FINAL_CENTER_ENDPOINT", 50),
    ]
    if not segments or segments[-1] != ("FINAL_BRAKE", 5):
        return False, "FINAL_J3_BRAKE_NOT_EXACT_5"
    active = segments[:-1]
    if not active or len(active) > len(expected):
        return False, "J3_ACTIVE_PHASE_PREFIX_LENGTH_INVALID"
    for index, (name, count) in enumerate(active):
        expected_name, expected_count = expected[index]
        if name != expected_name:
            return False, f"J3_PHASE_ORDER_INVALID:{name}:expected:{expected_name}"
        if count != expected_count:
            if index != len(active) - 1 or not (1 <= count <= expected_count):
                return False, f"J3_PHASE_COUNT_INVALID:{name}:{count}:{expected_count}"
    return True, "PASS"


def audit_j2_brake_rows(rows: list[dict[str, str]],
                        label: str,
                        final: bool) -> tuple[bool, list[str], dict[str, float]]:
    errors: list[str] = []
    for row in rows:
        try:
            if int(row["a_mode"]) != 0 or int(row["b_mode"]) != 0:
                raise ValueError("mode")
            if int(row["a_valid"]) != 1 or int(row["b_valid"]) != 1:
                raise ValueError("valid")
            if int(row["a_merror"]) != 0 or int(row["b_merror"]) != 0:
                raise ValueError("merror")
            if int(row["a_received_id"]) != 0 or int(row["b_received_id"]) != 1:
                raise ValueError("id")
            if any(int(row[name]) != 1 for name in
                   ("a_send_recv", "b_send_recv", "a_correct", "b_correct",
                    "a_crc_ok", "b_crc_ok")):
                raise ValueError("communication")
            if not (0 <= int(row["a_temp"]) < 60 and
                    0 <= int(row["b_temp"]) < 60):
                raise ValueError("temperature")
            if int(row["kp_cmd_count"]) != 0 or \
               int(row["kd_cmd_count"]) != 0 or \
               int(row["a_tff_count"]) != 0 or \
               int(row["b_tff_count"]) != 0:
                raise ValueError("nonzero encoded command")
            for field in ("active_kp", "kp_cmd_decoded", "kd_cmd_decoded",
                          "tff_nm", "a_q_cmd", "b_q_cmd", "a_dq_cmd",
                          "b_dq_cmd"):
                if not close(finite(row[field]), 0.0, 0.0):
                    raise ValueError(f"nonzero {field}")
            if not is_nan(row["q_ref_rad"]):
                raise ValueError("q_ref_rad not NaN")
            raw_a = finite(row["a_raw"])
            raw_b = finite(row["b_raw"])
            dq_a = finite(row["a_dq_feedback"])
            dq_b = finite(row["b_dq_feedback"])
            tau_a = finite(row["a_tau"])
            tau_b = finite(row["b_tau"])
            if final:
                if not close(finite(row["a_dq_logical"]), -dq_a / GEAR) or \
                   not close(finite(row["b_dq_logical"]), +dq_b / GEAR):
                    raise ValueError("logical velocity mapping")
                if not close(finite(row["tau_J2_feedback"]),
                             GEAR * (-tau_a + tau_b)):
                    raise ValueError("logical tau mapping")
                if not close(finite(row["expected_A_PD_tau"]),
                             0.0 * (0.0 - raw_a), 0.0) or \
                   not close(finite(row["expected_B_PD_tau"]),
                             0.0 * (0.0 - raw_b), 0.0):
                    raise ValueError("brake expected PD")
            else:
                for field in ("qA_logical_rad", "qB_logical_rad",
                              "qJ2_logical_rad", "e_common_rad", "e_sync_rad",
                              "a_dq_logical", "b_dq_logical",
                              "tau_J2_feedback", "expected_A_PD_tau",
                              "expected_B_PD_tau"):
                    if not is_nan(row[field]):
                        raise ValueError(f"capture non-NaN {field}")
        except (KeyError, TypeError, ValueError) as error:
            errors.append(f"{label}_BRAKE_ROW_{row.get('tick', '?')}:{error}")
            break
    static_pass = False
    static_metrics: dict[str, float] = {}
    if not errors and rows:
        static_pass, static_metrics = capture_static_result(
            rows, ("a_raw", "b_raw"))
        if not static_pass:
            errors.append(f"{label}_NOT_STATIC")
    return not errors, errors, static_metrics


def attach_cumulative_j2_sync(
        rows: list[dict[str, str]],
        summaries: dict[str, dict[str, float] | None]) -> None:
    running = 0.0
    cumulative: dict[str, float] = {}
    for row in rows:
        phase = row["phase"]
        if phase not in ROUTE_PHASES:
            continue
        running = max(running, abs(finite(row["e_sync_rad"])) * DEG)
        if phase in ENDPOINT_COUNTS:
            cumulative[phase] = running
    for phase, summary in summaries.items():
        if summary is not None and phase in cumulative:
            summary["cumulative_max_e_sync_deg"] = cumulative[phase]


def least_squares_slope(samples: list[tuple[float, float]]) -> float:
    mean_t = sum(sample[0] for sample in samples) / len(samples)
    mean_q = sum(sample[1] for sample in samples) / len(samples)
    numerator = sum((time_s - mean_t) * (position - mean_q)
                    for time_s, position in samples)
    denominator = sum((time_s - mean_t) ** 2 for time_s, _ in samples)
    if not denominator > 0.0:
        raise ValueError("velocity estimator denominator")
    return numerator / denominator


def expected_j3_velocities(
        rows: list[dict[str, str]]) -> list[tuple[float, float]]:
    window: list[tuple[float, float]] = []
    result: list[tuple[float, float]] = []
    for row in rows:
        window.append((finite(row["timestamp_s"]),
                       finite(row["q_relative_ros_rad"])))
        if len(window) > 11:
            window.pop(0)
        fast = least_squares_slope(window[-5:]) if len(window) >= 5 else 0.0
        slow = least_squares_slope(window[-11:]) if len(window) >= 11 else 0.0
        result.append((abs(fast * DEG), abs(slow * DEG)))
    return result


def audit_j3_aux_rows(
        rows: list[dict[str, str]]) -> tuple[bool, list[str], dict[str, str]]:
    errors: list[str] = []
    topology: dict[str, str] = {}
    for joint, expected_id in (("j4", 4), ("j5", 5)):
        values = {row[f"{joint}_topology"] for row in rows}
        if len(values) != 1 or next(iter(values), "") not in {
                "connected", "disconnected"}:
            errors.append(f"{joint.upper()}_TOPOLOGY_NOT_EXACT_OR_CONSTANT")
            continue
        selected = next(iter(values))
        topology[joint] = selected
        for row in rows:
            try:
                if selected == "connected":
                    if int(row[f"{joint}_mode"]) != 0 or \
                       int(row[f"{joint}_merror"]) != 0 or \
                       int(row[f"{joint}_received_id"]) != expected_id or \
                       not (0 <= int(row[f"{joint}_temperature_c"]) < 60) or \
                       any(int(row[f"{joint}_{field}"]) != 1 for field in
                           ("send_recv", "correct", "crc_ok", "valid")):
                        raise ValueError("connected BRAKE evidence")
                elif int(row[f"{joint}_mode"]) != -1 or \
                     int(row[f"{joint}_temperature_c"]) != -1 or \
                     int(row[f"{joint}_merror"]) != -1 or \
                     int(row[f"{joint}_received_id"]) != -1 or \
                     any(int(row[f"{joint}_{field}"]) != 0 for field in
                         ("send_recv", "correct", "crc_ok", "valid")):
                    raise ValueError("disconnected sentinel evidence")
            except (KeyError, TypeError, ValueError) as error:
                errors.append(
                    f"{joint.upper()}_ROW_{row.get('tick', '?')}:{error}")
                break
    return not errors, errors, topology


def audit_j3_brake_rows(
        rows: list[dict[str, str]], label: str,
        final: bool) -> tuple[bool, list[str], dict[str, float]]:
    errors: list[str] = []
    for row in rows:
        try:
            if int(row["j3_mode"]) != 0 or int(row["valid"]) != 1:
                raise ValueError("mode/valid")
            if int(row["merror"]) != 0 or int(row["received_id"]) != 3:
                raise ValueError("merror/id")
            if not (0 <= int(row["temperature_c"]) < 60):
                raise ValueError("temperature")
            if any(int(row[field]) != 1 for field in
                   ("send_recv", "correct", "crc_ok")):
                raise ValueError("communication")
            if any(int(row[field]) != 0 for field in
                   ("kp_cmd_count", "kd_cmd_count", "tff_cmd_count")):
                raise ValueError("nonzero encoded command")
            for field in ("kp_cmd_literal", "kp_cmd_decoded",
                          "kd_cmd_literal", "kd_cmd_decoded",
                          "tff_cmd_literal", "tff_cmd_decoded",
                          "q_cmd_motor_rad", "dq_cmd_motor_rad_s"):
                if not close(finite(row[field]), 0.0, 0.0):
                    raise ValueError(f"nonzero {field}")
            for field in ("q_ref_rad", "position_error_rad",
                          "expected_pd_tau_nm", "fast_velocity_deg_s",
                          "slow_velocity_deg_s"):
                if not is_nan(row[field]):
                    raise ValueError(f"non-NaN {field}")
            if not final and not is_nan(row["q_relative_ros_rad"]):
                raise ValueError("capture q_relative_ros_rad")
            raw = finite(row["q_feedback_motor_rad"])
            dq = finite(row["dq_feedback_motor_rad_s"])
            tau = finite(row["tau_feedback_nm"])
            if not close(finite(row["dq_logical_rad_s"]), dq / GEAR):
                raise ValueError("logical velocity mapping")
            if not close(finite(row["logical_tau_feedback_nm"]), GEAR * tau):
                raise ValueError("logical tau mapping")
            expected_final_flag = 1 if final else -1
            if int(row["final_brake_row_pass"]) != expected_final_flag:
                raise ValueError("final brake row flag")
            if not final and row["abort_reason"].strip():
                raise ValueError("capture abort reason")
        except (KeyError, TypeError, ValueError) as error:
            errors.append(f"{label}_BRAKE_ROW_{row.get('tick', '?')}:{error}")
            break
    static_pass = False
    static_metrics: dict[str, float] = {}
    if not errors and rows:
        static_pass, static_metrics = capture_static_result(
            rows, ("q_feedback_motor_rad",))
        if not static_pass:
            errors.append(f"{label}_NOT_STATIC")
    return not errors, errors, static_metrics


def audit_run(path: Path, operator: str) -> dict[str, object]:
    config = CONFIGS.get(path.name)
    if config is None:
        raise ValueError(f"unsupported evidence filename: {path.name}")
    fieldnames, rows = read_csv_strict(
        path, (FIELDS, FIELDS_WITH_TERMINATION))
    if [int(row["tick"]) for row in rows] != list(range(len(rows))):
        raise ValueError("tick sequence mismatch")
    if any(row["level"] != config["level"] for row in rows):
        raise ValueError("level label mismatch")
    if any(not close(finite(row["target_kp"]), float(config["kp"]), 1e-12)
           for row in rows):
        raise ValueError("target Kp mismatch")

    segments = contiguous_segments(rows)
    prefix_pass, prefix_reason = audit_phase_prefix(segments, int(config["ramp_rows"]))
    rows_by_phase: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        rows_by_phase.setdefault(row["phase"], []).append(row)

    invariant_errors: list[str] = []
    if "termination_reason" in fieldnames and any(
            row["termination_reason"].strip() != "RUNNING"
            for row in rows if row["phase"] != "FINAL_DUAL_BRAKE"):
        invariant_errors.append("NONFINAL_TERMINATION_REASON_NOT_RUNNING")
    capture = rows_by_phase.get("SESSION_BRAKE_CAPTURE", [])
    capture_brake, capture_errors, capture_static = audit_j2_brake_rows(
        capture, "SESSION_CAPTURE", False)
    capture_brake = capture_brake and len(capture) == 50
    if not capture_brake:
        invariant_errors.extend(capture_errors or
                                ["SESSION_CENTER_NOT_EXACT_50_VALID_STATIC_FRAMES"])
    a0 = statistics.median(finite(row["a_raw"]) for row in capture) \
        if capture_brake else None
    b0 = statistics.median(finite(row["b_raw"]) for row in capture) \
        if capture_brake else None

    active_rows = [row for row in rows if row["phase"] != "SESSION_BRAKE_CAPTURE"
                   and row["phase"] != "FINAL_DUAL_BRAKE"]
    phase_indices: Counter[str] = Counter()
    for row in active_rows:
        try:
            phase = row["phase"]
            phase_index = phase_indices[phase]
            phase_indices[phase] += 1
            expected_q_ref, expected_dq_ref = expected_reference(
                phase, phase_index)
            kp = finite(row["active_kp"])
            q_ref = finite(row["q_ref_rad"])
            raw_a = finite(row["a_raw"])
            raw_b = finite(row["b_raw"])
            q_a = finite(row["qA_logical_rad"])
            q_b = finite(row["qB_logical_rad"])
            q_j2 = finite(row["qJ2_logical_rad"])
            a_cmd = finite(row["a_q_cmd"])
            b_cmd = finite(row["b_q_cmd"])
            if int(row["a_mode"]) != 1 or int(row["b_mode"]) != 1:
                raise ValueError("active mode")
            if int(row["a_valid"]) != 1 or int(row["b_valid"]) != 1:
                raise ValueError("active feedback validity")
            if int(row["a_merror"]) != 0 or int(row["b_merror"]) != 0:
                raise ValueError("merror")
            if any(int(row[name]) != 1 for name in
                   ("a_send_recv", "b_send_recv", "a_correct", "b_correct",
                    "a_crc_ok", "b_crc_ok")):
                raise ValueError("communication integrity")
            if int(row["a_received_id"]) != 0 or int(row["b_received_id"]) != 1:
                raise ValueError("motor id")
            if not (0 <= int(row["a_temp"]) < 60 and
                    0 <= int(row["b_temp"]) < 60):
                raise ValueError("temperature")
            if int(row["a_tff_count"]) != 0 or int(row["b_tff_count"]) != 0:
                raise ValueError("Tff count")
            if not close(finite(row["tff_nm"]), 0.0, 0.0):
                raise ValueError("Tff literal")
            if int(row["kd_cmd_count"]) != 64 or not close(
                    finite(row["kd_cmd_decoded"]), 0.05, 1e-12):
                raise ValueError("Kd encoding")
            if phase == "KP_RAMP_HOLD":
                intervals = int(config["ramp_rows"]) - 1
                expected_kp = 0.60 + (float(config["kp"]) - 0.60) * \
                    smoothstep5(phase_index / intervals)
            else:
                expected_kp = float(config["kp"])
            if not close(kp, expected_kp, 1e-12):
                raise ValueError("active Kp trajectory")
            kp_count = int(row["kp_cmd_count"])
            if kp_count != sdk_gain_count(kp):
                raise ValueError("Kp encoded count")
            kp_decoded = finite(row["kp_cmd_decoded"])
            if not close(kp_decoded, kp_count / 1280.0, 1e-12):
                raise ValueError("Kp decode")
            if abs(kp_decoded - kp) > 1.0 / 1280.0 + 1e-12:
                raise ValueError("Kp quantization residual")
            if a0 is None or b0 is None:
                raise ValueError("missing center")
            if not close(q_ref, expected_q_ref, 1e-12):
                raise ValueError("q_ref trajectory")
            if not close(q_a, -(raw_a - a0) / GEAR):
                raise ValueError("qA mapping")
            if not close(q_b, +(raw_b - b0) / GEAR):
                raise ValueError("qB mapping")
            if not close(q_j2, 0.5 * (q_a + q_b)):
                raise ValueError("qJ2 mapping")
            if not close(finite(row["e_sync_rad"]), q_a - q_b):
                raise ValueError("e_sync")
            if not close(finite(row["e_common_rad"]), q_ref - q_j2):
                raise ValueError("position error")
            if not close(a_cmd, a0 - GEAR * q_ref):
                raise ValueError("A command mapping")
            if not close(b_cmd, b0 + GEAR * q_ref):
                raise ValueError("B command mapping")
            if not close(finite(row["a_dq_cmd"]),
                         -GEAR * expected_dq_ref, 1e-12):
                raise ValueError("A dq command trajectory")
            if not close(finite(row["b_dq_cmd"]),
                         +GEAR * expected_dq_ref, 1e-12):
                raise ValueError("B dq command trajectory")
            if not close(finite(row["a_dq_logical"]),
                         -finite(row["a_dq_feedback"]) / GEAR):
                raise ValueError("A dq feedback mapping")
            if not close(finite(row["b_dq_logical"]),
                         +finite(row["b_dq_feedback"]) / GEAR):
                raise ValueError("B dq feedback mapping")
            # The frozen V15.24F evidence definition uses literal active Kp
            # times position error only.  Kd is audited independently above.
            expected_a = kp * (a_cmd - raw_a)
            expected_b = kp * (b_cmd - raw_b)
            if not close(finite(row["expected_A_PD_tau"]), expected_a):
                raise ValueError("expected A PD")
            if not close(finite(row["expected_B_PD_tau"]), expected_b):
                raise ValueError("expected B PD")
            logical_tau = GEAR * (-finite(row["a_tau"]) + finite(row["b_tau"]))
            if not close(finite(row["tau_J2_feedback"]), logical_tau):
                raise ValueError("logical tau feedback")
            if abs(finite(row["a_dq_logical"])) * DEG > 25.0 or \
               abs(finite(row["b_dq_logical"])) * DEG > 25.0:
                raise ValueError("logical velocity")
            if abs(finite(row["e_sync_rad"])) * DEG > 1.0:
                raise ValueError("sync hard abort")
        except (KeyError, TypeError, ValueError) as error:
            invariant_errors.append(f"row {row['tick']}: {error}")
            break

    ramp = rows_by_phase.get("KP_RAMP_HOLD", [])
    hold_rows = ramp + rows_by_phase.get("CURRENT_HOLD", [])
    hold_full = len(ramp) == int(config["ramp_rows"]) and \
        len(rows_by_phase.get("CURRENT_HOLD", [])) == 100
    hold_max_motion = max((abs(finite(row["qJ2_logical_rad"])) * DEG
                           for row in hold_rows), default=None)
    hold_max_sync = max((abs(finite(row["e_sync_rad"])) * DEG
                         for row in hold_rows), default=None)
    current_hold = rows_by_phase.get("CURRENT_HOLD", [])
    hold_final_sync = abs(median_field(current_hold, "e_sync_rad", 50) * DEG) \
        if len(current_hold) >= 50 else None
    hold_pass = bool(hold_full and hold_max_motion is not None and
                     hold_max_motion <= 1.0 and hold_max_sync is not None and
                     hold_max_sync <= 0.3 and hold_final_sync is not None and
                     hold_final_sync <= 0.3)

    plus = endpoint(rows_by_phase, "PLUS_5_ENDPOINT", 5.0, 30)
    first = endpoint(rows_by_phase, "FIRST_CENTER_ENDPOINT", 0.0, 30)
    minus = endpoint(rows_by_phase, "MINUS_5_ENDPOINT", -5.0, 30)
    final = endpoint(rows_by_phase, "FINAL_CENTER_ENDPOINT", 0.0, 40)
    endpoint_summaries = {
        "PLUS_5_ENDPOINT": plus,
        "FIRST_CENTER_ENDPOINT": first,
        "MINUS_5_ENDPOINT": minus,
        "FINAL_CENTER_ENDPOINT": final,
    }
    attach_cumulative_j2_sync(rows, endpoint_summaries)
    motion_rows = [row for row in rows if row["phase"] in ROUTE_PHASES]
    max_motion_sync = max((abs(finite(row["e_sync_rad"])) * DEG
                           for row in motion_rows), default=None)
    max_logical_tau = max((abs(finite(row["tau_J2_feedback"]))
                           for row in active_rows), default=None)
    max_expected_logical_pd = max((abs(GEAR *
        (-finite(row["expected_A_PD_tau"]) + finite(row["expected_B_PD_tau"])))
        for row in active_rows), default=None)
    max_velocity = max((max(abs(finite(row["a_dq_logical"])),
                            abs(finite(row["b_dq_logical"]))) * DEG
                        for row in active_rows), default=None)

    full_route = all(value is not None for value in (plus, first, minus, final))
    endpoint_pass = bool(full_route and plus["error_deg"] <= 1.0 and
        plus["e_sync_deg"] <= 0.3 and first["error_deg"] <= 0.75 and
        first["e_sync_deg"] <= 0.3 and minus["error_deg"] <= 1.0 and
        minus["e_sync_deg"] <= 0.3 and final["error_deg"] <= 0.75 and
        final["e_sync_deg"] <= 0.3)
    final_brake_rows = rows_by_phase.get("FINAL_DUAL_BRAKE", [])
    final_brake, final_brake_errors, final_brake_static = audit_j2_brake_rows(
        final_brake_rows, "FINAL", True)
    final_brake = final_brake and len(final_brake_rows) == 5
    if a0 is not None and b0 is not None and final_brake:
        try:
            for row in final_brake_rows:
                q_a = -(finite(row["a_raw"]) - a0) / GEAR
                q_b = +(finite(row["b_raw"]) - b0) / GEAR
                q_j2 = 0.5 * (q_a + q_b)
                if not close(finite(row["qA_logical_rad"]), q_a) or \
                   not close(finite(row["qB_logical_rad"]), q_b) or \
                   not close(finite(row["qJ2_logical_rad"]), q_j2) or \
                   not close(finite(row["e_sync_rad"]), q_a - q_b) or \
                   not close(finite(row["e_common_rad"]), -q_j2):
                    raise ValueError("final logical position mapping")
        except (KeyError, TypeError, ValueError) as error:
            final_brake = False
            final_brake_errors.append(f"FINAL_BRAKE:{error}")
    if not final_brake:
        invariant_errors.extend(final_brake_errors or
                                ["FINAL_DUAL_BRAKE_NOT_EXACT_5_VALID_STATIC_FRAMES"])
    termination = termination_summary(
        rows, fieldnames, final_brake_rows, "termination_reason")
    completed_termination = bool(
        termination["field_present"] and termination["consistent"] and
        termination["final_rows_complete"] and
        termination["reason"] == "COMPLETED_ROUTE")
    timing = timing_summary(rows, active_rows, "all")
    numeric_pass = bool(prefix_pass and not invariant_errors and hold_pass and
        endpoint_pass and max_motion_sync is not None and max_motion_sync <= 0.7 and
        timing["result"] == "PASS" and final_brake and completed_termination)
    operator_safe = operator == "SAFE"
    result = "PASS" if numeric_pass and operator_safe else "FAIL"
    if numeric_pass and operator == "PENDING":
        result = "PENDING_OPERATOR"

    completed_names = [name for name, _ in segments]
    checkpoint_order = [
        ("PLUS_5_ENDPOINT", plus, 1.0),
        ("FIRST_CENTER_ENDPOINT", first, 0.75),
        ("MINUS_5_ENDPOINT", minus, 1.0),
        ("FINAL_CENTER_ENDPOINT", final, 0.75),
    ]
    gate_fix = fail_closed_center_gate(completed_names, checkpoint_order)
    failed_endpoint = exact_failed_endpoint(
        segments, checkpoint_order, termination)
    safe_tracking_fail = bool(result == "FAIL" and hold_pass and operator_safe and
        prefix_pass and not invariant_errors and final_brake and
        timing["result"] == "PASS" and
        gate_fix and failed_endpoint is not None and
        max_motion_sync is not None and max_motion_sync <= 0.7 and
        max_velocity is not None and max_velocity <= 25.0)

    return {
        "schema": "V15_24F_J2_RUN_ANALYSIS_V1",
        "joint": "J2",
        "source_head": SOURCE_HEAD,
        "csv": str(path),
        "csv_sha256": sha256(path),
        "rows": len(rows),
        "phase": config["phase"],
        "kp": config["kp"],
        "a0_raw_rad": a0,
        "b0_raw_rad": b0,
        "session_center_source": "50_VALID_BRAKE_FRAME_MEDIAN",
        "session_center_redefined": False,
        "session_capture_brake": "PASS" if capture_brake else "FAIL",
        "session_capture_static": capture_static,
        "hold": {"result": "PASS" if hold_pass else "FAIL",
                 "max_motion_deg": hold_max_motion,
                 "max_e_sync_deg": hold_max_sync,
                 "final_e_sync_deg": hold_final_sync},
        "route": {"plus_5": plus, "first_center": first,
                  "minus_5": minus, "final_center": final},
        "max_motion_e_sync_deg": max_motion_sync,
        "max_abs_logical_tau_feedback_nm": max_logical_tau,
        "max_abs_expected_logical_pd_tau_nm": max_expected_logical_pd,
        "max_logical_velocity_deg_s": max_velocity,
        "timing_100hz": timing,
        "phase_segments": segments,
        "phase_prefix_audit": {"result": "PASS" if prefix_pass else "FAIL",
                               "reason": prefix_reason},
        "invariant_errors": invariant_errors,
        "center_gate_fix": "YES" if gate_fix else "NO",
        "operator_observation": operator,
        "final_brake": "PASS" if final_brake else "FAIL",
        "final_brake_static": final_brake_static,
        "termination": termination,
        "failed_endpoint": failed_endpoint,
        "numeric_result": "PASS" if numeric_pass else "FAIL",
        "safe_tracking_fail_eligible_for_kp140": safe_tracking_fail,
        "result": result,
    }


def audit_j3_run(path: Path, operator: str) -> dict[str, object]:
    config = J3_CONFIGS.get(path.name)
    if config is None:
        raise ValueError(f"unsupported J3 evidence filename: {path.name}")
    fieldnames, rows = read_csv_strict(path, J3_ACCEPTED_FIELDS)
    if [int(row["tick"]) for row in rows] != list(range(len(rows))):
        raise ValueError("J3 tick sequence mismatch")
    if any(row["run_label"] != config["phase"] for row in rows):
        raise ValueError("J3 run label mismatch")

    segments = contiguous_segments(rows)
    prefix_pass, prefix_reason = audit_j3_phase_prefix(segments)
    rows_by_phase: dict[str, list[dict[str, str]]] = {}
    for row in rows:
        rows_by_phase.setdefault(row["phase"], []).append(row)
    invariant_errors: list[str] = []
    capture = rows_by_phase.get("SESSION_BRAKE_CAPTURE", [])
    capture_brake, capture_errors, capture_static = audit_j3_brake_rows(
        capture, "SESSION_CAPTURE", False)
    capture_brake = capture_brake and len(capture) == 50
    if not capture_brake:
        invariant_errors.extend(capture_errors or
                                ["J3_SESSION_CENTER_NOT_EXACT_50_VALID_STATIC_FRAMES"])
    q0 = statistics.median(finite(row["q_feedback_motor_rad"])
                           for row in capture) if capture_brake else None
    active_rows = [row for row in rows if row["phase"] not in
                   {"SESSION_BRAKE_CAPTURE", "FINAL_BRAKE"}]
    aux_pass, aux_errors, aux_topology = audit_j3_aux_rows(rows)
    if not aux_pass:
        invariant_errors.extend(aux_errors)
    try:
        velocity_expectations = expected_j3_velocities(active_rows)
    except (KeyError, TypeError, ValueError) as error:
        velocity_expectations = []
        invariant_errors.append(f"J3_VELOCITY_RECOMPUTE:{error}")
    expected_kp_count = sdk_gain_count(float(config["kp"]))
    phase_indices: Counter[str] = Counter()
    for row_index, row in enumerate(active_rows):
        try:
            phase = row["phase"]
            phase_index = phase_indices[phase]
            phase_indices[phase] += 1
            expected_q_ref, expected_dq_ref = expected_reference(
                phase, phase_index)
            kp = finite(row["kp_cmd_literal"])
            q_cmd = finite(row["q_cmd_motor_rad"])
            raw = finite(row["q_feedback_motor_rad"])
            q_rel = finite(row["q_relative_ros_rad"])
            q_ref = finite(row["q_ref_rad"])
            if int(row["j3_mode"]) != 1 or int(row["valid"]) != 1:
                raise ValueError("J3 active mode/validity")
            if int(row["merror"]) != 0:
                raise ValueError("J3 merror")
            if any(int(row[name]) != 1 for name in
                   ("send_recv", "correct", "crc_ok")):
                raise ValueError("J3 communication integrity")
            if int(row["received_id"]) != 3:
                raise ValueError("J3 received id")
            if not (0 <= int(row["temperature_c"]) < 60):
                raise ValueError("J3 temperature")
            if not close(kp, float(config["kp"]), 1e-12) or \
               int(row["kp_cmd_count"]) != expected_kp_count or \
               not close(finite(row["kp_cmd_decoded"]),
                         expected_kp_count / 1280.0, 1e-12):
                raise ValueError("J3 Kp encoding")
            if not close(finite(row["kd_cmd_literal"]), 0.05, 1e-12) or \
               int(row["kd_cmd_count"]) != 64 or \
               not close(finite(row["kd_cmd_decoded"]), 0.05, 1e-12):
                raise ValueError("J3 Kd encoding")
            if not close(finite(row["tff_cmd_literal"]), 0.0, 0.0) or \
               int(row["tff_cmd_count"]) != 0 or \
               not close(finite(row["tff_cmd_decoded"]), 0.0, 0.0):
                raise ValueError("J3 Tff zero contract")
            if q0 is None:
                raise ValueError("J3 missing session center")
            if not close(q_ref, expected_q_ref, 1e-12):
                raise ValueError("J3 q_ref trajectory")
            if not close(q_rel, (raw - q0) / GEAR):
                raise ValueError("J3 relative mapping")
            if not close(q_cmd, q0 + GEAR * q_ref):
                raise ValueError("J3 command mapping")
            if not close(finite(row["dq_cmd_motor_rad_s"]),
                         GEAR * expected_dq_ref, 1e-12):
                raise ValueError("J3 dq command trajectory")
            if not close(finite(row["position_error_rad"]), q_ref - q_rel):
                raise ValueError("J3 position error")
            if not close(finite(row["expected_pd_tau_nm"]), kp * (q_cmd - raw)):
                raise ValueError("J3 expected PD")
            if not close(finite(row["dq_logical_rad_s"]),
                         finite(row["dq_feedback_motor_rad_s"]) / GEAR):
                raise ValueError("J3 logical velocity mapping")
            if abs(finite(row["dq_logical_rad_s"])) * DEG > 25.0:
                raise ValueError("J3 logical velocity")
            if row_index >= len(velocity_expectations):
                raise ValueError("J3 velocity expectation missing")
            expected_fast, expected_slow = velocity_expectations[row_index]
            fast = finite(row["fast_velocity_deg_s"])
            slow = finite(row["slow_velocity_deg_s"])
            if not close(fast, expected_fast, 1e-6) or \
               not close(slow, expected_slow, 1e-6):
                raise ValueError("J3 fast/slow velocity recomputation")
            if fast < 0.0 or slow < 0.0 or fast > 25.0 or slow > 25.0:
                raise ValueError("J3 fast/slow velocity envelope")
            if not close(finite(row["logical_tau_feedback_nm"]),
                         GEAR * finite(row["tau_feedback_nm"])):
                raise ValueError("J3 logical tau")
            if int(row["final_brake_row_pass"]) != -1:
                raise ValueError("J3 active final brake row flag")
            if row["abort_reason"].strip():
                raise ValueError("J3 active hard-abort reason")
            if abs(q_rel) > 8.0 * RAD:
                raise ValueError("J3 feedback envelope")
        except (KeyError, TypeError, ValueError) as error:
            invariant_errors.append(f"row {row['tick']}: {error}")
            break

    hold = rows_by_phase.get("CURRENT_HOLD", [])
    hold_max = max((abs(finite(row["q_relative_ros_rad"])) * DEG
                    for row in hold), default=None)
    hold_final = abs(median_field(hold, "q_relative_ros_rad", 50) * DEG) \
        if len(hold) >= 50 else None
    hold_pass = len(hold) == 150 and hold_max is not None and hold_max <= 1.0 and \
        hold_final is not None and hold_final <= 0.5
    plus = j3_endpoint(rows_by_phase, "PLUS_5_ENDPOINT", 5.0, 30)
    first = j3_endpoint(rows_by_phase, "FIRST_CENTER_ENDPOINT", 0.0, 30)
    minus = j3_endpoint(rows_by_phase, "MINUS_5_ENDPOINT", -5.0, 30)
    final = j3_endpoint(rows_by_phase, "FINAL_CENTER_ENDPOINT", 0.0, 40)
    full_route = all(value is not None for value in (plus, first, minus, final))
    endpoint_pass = bool(full_route and plus["error_deg"] <= 1.0 and
        first["error_deg"] <= 0.75 and minus["error_deg"] <= 1.0 and
        final["error_deg"] <= 0.75)
    final_brake_rows = rows_by_phase.get("FINAL_BRAKE", [])
    final_brake, final_brake_errors, final_brake_static = audit_j3_brake_rows(
        final_brake_rows, "FINAL", True)
    final_brake = final_brake and len(final_brake_rows) == 5
    if q0 is not None and final_brake:
        try:
            for row in final_brake_rows:
                if not close(finite(row["q_relative_ros_rad"]),
                             (finite(row["q_feedback_motor_rad"]) - q0) / GEAR):
                    raise ValueError("final relative mapping")
        except (KeyError, TypeError, ValueError) as error:
            final_brake = False
            final_brake_errors.append(f"FINAL_BRAKE:{error}")
    if not final_brake:
        invariant_errors.extend(final_brake_errors or
                                ["FINAL_J3_BRAKE_NOT_EXACT_5_VALID_STATIC_FRAMES"])
    termination = termination_summary(
        rows, fieldnames, final_brake_rows, "abort_reason")
    completed_termination = bool(
        termination["field_present"] and termination["consistent"] and
        termination["final_rows_complete"] and
        termination["reason"] == "COMPLETED_ROUTE")
    timing = timing_summary(rows, active_rows, "active")
    max_velocity = max((max(abs(finite(row["dq_logical_rad_s"])) * DEG,
                            finite(row["fast_velocity_deg_s"]),
                            finite(row["slow_velocity_deg_s"]))
                        for row in active_rows), default=None)
    max_tau = max((abs(finite(row["logical_tau_feedback_nm"]))
                   for row in active_rows), default=None)
    numeric_pass = bool(prefix_pass and not invariant_errors and hold_pass and
        endpoint_pass and final_brake and timing["result"] == "PASS" and
        completed_termination)
    result = "PASS" if numeric_pass and operator == "SAFE" else "FAIL"
    if numeric_pass and operator == "PENDING":
        result = "PENDING_OPERATOR"
    completed_names = [name for name, _ in segments]
    checkpoints = [
        ("PLUS_5_ENDPOINT", plus, 1.0),
        ("FIRST_CENTER_ENDPOINT", first, 0.75),
        ("MINUS_5_ENDPOINT", minus, 1.0),
        ("FINAL_CENTER_ENDPOINT", final, 0.75),
    ]
    center_gate = fail_closed_center_gate(completed_names, checkpoints)
    failed_endpoint = exact_failed_endpoint(segments, checkpoints, termination)
    safe_tracking_fail = bool(result == "FAIL" and hold_pass and operator == "SAFE" and
        prefix_pass and not invariant_errors and final_brake and
        timing["result"] == "PASS" and center_gate and
        failed_endpoint is not None and
        max_velocity is not None and max_velocity <= 25.0 and
        termination["reason"] ==
        f"{ENDPOINT_REASON_PREFIX[failed_endpoint]}_ACCEPTANCE_FAILED_STOP_ROUTE")
    return {
        "schema": "V15_24F_J3_RUN_ANALYSIS_V1",
        "joint": "J3",
        "source_head": SOURCE_HEAD,
        "csv": str(path),
        "csv_sha256": sha256(path),
        "rows": len(rows),
        "phase": config["phase"],
        "kp": config["kp"],
        "q0_raw_rad": q0,
        "session_center_source": "50_VALID_BRAKE_FRAME_MEDIAN",
        "session_center_redefined": False,
        "session_capture_brake": "PASS" if capture_brake else "FAIL",
        "session_capture_static": capture_static,
        "aux_topology": aux_topology,
        "aux_brake_evidence": "PASS" if aux_pass else "FAIL",
        "hold": {"result": "PASS" if hold_pass else "FAIL",
                 "max_motion_deg": hold_max, "final_error_deg": hold_final},
        "route": {"plus_5": plus, "first_center": first,
                  "minus_5": minus, "final_center": final},
        "max_logical_velocity_deg_s": max_velocity,
        "max_abs_logical_tau_feedback_nm": max_tau,
        "timing_100hz": timing,
        "phase_segments": segments,
        "phase_prefix_audit": {"result": "PASS" if prefix_pass else "FAIL",
                               "reason": prefix_reason},
        "invariant_errors": invariant_errors,
        "center_gate_fix": "YES" if center_gate else "NO",
        "operator_observation": operator,
        "final_brake": "PASS" if final_brake else "FAIL",
        "final_brake_static": final_brake_static,
        "termination": termination,
        "failed_endpoint": failed_endpoint,
        "numeric_result": "PASS" if numeric_pass else "FAIL",
        "safe_tracking_fail_eligible_for_kp070": safe_tracking_fail,
        "result": result,
    }


def next_gate(summary: dict[str, object]) -> dict[str, object] | None:
    phase = str(summary["phase"])
    result = str(summary["result"])
    if phase == "j2-kp100-run1" and result == "PASS":
        allowed, reason = "j2-kp100-repeat", "J2_KP100_RUN1_PASS"
    elif phase == "j2-kp100-run1" and summary[
            "safe_tracking_fail_eligible_for_kp140"]:
        allowed, reason = "j2-kp140-run1", "J2_KP100_RUN1_SAFE_TRACKING_FAIL"
    elif phase == "j2-kp140-run1" and result == "PASS":
        allowed, reason = "j2-kp140-repeat", "J2_KP140_RUN1_PASS"
    elif phase in {"j2-kp100-repeat", "j2-kp140-repeat"} and result == "PASS":
        allowed, reason = "j3-kp060-run1", "J2_EXACT_REPEAT_PASS"
    elif phase == "j3-kp060-run1" and result == "PASS":
        allowed, reason = "j3-kp060-repeat", "J3_KP060_RUN1_PASS"
    elif phase == "j3-kp060-run1" and summary[
            "safe_tracking_fail_eligible_for_kp070"]:
        allowed, reason = "j3-kp070-run1", "J3_KP060_RUN1_SAFE_TRACKING_FAIL"
    elif phase == "j3-kp070-run1" and result == "PASS":
        allowed, reason = "j3-kp070-repeat", "J3_KP070_RUN1_PASS"
    else:
        return None
    safe_tracking_fail = bool(
        summary.get("safe_tracking_fail_eligible_for_kp140", False) or
        summary.get("safe_tracking_fail_eligible_for_kp070", False))
    return {
        "schema": GATE_SCHEMA,
        "eligible": True,
        "allowed_phase": allowed,
        "reason": reason,
        "source_head": SOURCE_HEAD,
        "prerequisite_csv_path": str(summary["csv"]),
        "prerequisite_csv_sha256": str(summary["csv_sha256"]),
        "operator_observation": summary["operator_observation"],
        "final_brake": summary["final_brake"],
        "center_gate_fix": summary["center_gate_fix"],
        "timing_100hz_result": summary["timing_100hz"]["result"],
        "hold_result": summary["hold"]["result"],
        "prerequisite_result": summary["result"],
        "safe_tracking_fail_eligible": safe_tracking_fail,
        "failed_endpoint": summary.get("failed_endpoint"),
        "termination_reason": summary.get("termination", {}).get("reason"),
    }


def self_test() -> None:
    assert len(FIELDS) == 52
    assert len(FIELDS_WITH_TERMINATION) == 53
    assert len(J3_FIELDS) == 56
    assert sdk_gain_count(0.60) == 768
    assert sdk_gain_count(0.70) == 895
    assert sdk_gain_count(1.00) == 1280
    assert sdk_gain_count(1.40) == 1791
    assert close(smoothstep5(0.0), 0.0, 0.0)
    assert close(smoothstep5(0.5), 0.5, 1e-15)
    assert close(smoothstep5(1.0), 1.0, 0.0)
    plus_q, plus_dq = expected_reference("PLUS_5_PROFILE", 84)
    assert close(plus_q, 5.0 * RAD, 1e-15) and close(plus_dq, 0.0, 0.0)
    assert audit_phase_prefix([
        ("SESSION_BRAKE_CAPTURE", 50), ("KP_RAMP_HOLD", 51),
        ("CURRENT_HOLD", 100), ("PLUS_5_PROFILE", 85),
        ("PLUS_5_ENDPOINT", 40), ("FINAL_DUAL_BRAKE", 5),
    ], 51)[0]
    assert audit_j3_phase_prefix([
        ("SESSION_BRAKE_CAPTURE", 50), ("CURRENT_HOLD", 150),
        ("PLUS_5_PROFILE", 85), ("PLUS_5_ENDPOINT", 40),
        ("FINAL_BRAKE", 5),
    ])[0]
    assert not audit_j3_phase_prefix([
        ("SESSION_BRAKE_CAPTURE", 50), ("PLUS_5_PROFILE", 85),
        ("FINAL_BRAKE", 5),
    ])[0]
    assert not audit_phase_prefix([
        ("SESSION_BRAKE_CAPTURE", 50), ("KP_RAMP_HOLD", 51),
        ("CURRENT_HOLD", 100), ("PLUS_5_ENDPOINT", 40),
        ("FINAL_DUAL_BRAKE", 5),
    ], 51)[0]
    failed_center = {"actual_deg": 1.0, "error_deg": 1.0,
                     "e_sync_deg": 0.1}
    assert not fail_closed_center_gate(
        ["PLUS_5_ENDPOINT", "FIRST_CENTER_ENDPOINT", "MINUS_5_PROFILE"],
        [("PLUS_5_ENDPOINT", {"actual_deg": 5.0, "error_deg": 0.0,
                               "e_sync_deg": 0.1}, 1.0),
         ("FIRST_CENTER_ENDPOINT", failed_center, 0.75),
         ("MINUS_5_ENDPOINT", None, 1.0),
         ("FINAL_CENTER_ENDPOINT", None, 0.75)])
    assert fail_closed_center_gate(
        ["PLUS_5_ENDPOINT", "FIRST_CENTER_ENDPOINT"],
        [("PLUS_5_ENDPOINT", {"actual_deg": 5.0, "error_deg": 0.0,
                              "e_sync_deg": 0.1}, 1.0),
         ("FIRST_CENTER_ENDPOINT", failed_center, 0.75),
         ("MINUS_5_ENDPOINT", None, 1.0),
         ("FINAL_CENTER_ENDPOINT", None, 0.75)])
    # A J3 endpoint has no e_sync field and must not raise KeyError.
    assert fail_closed_center_gate(
        ["PLUS_5_ENDPOINT"],
        [("PLUS_5_ENDPOINT", {"actual_deg": 5.0,
                               "error_deg": 0.0}, 1.0)])
    assert checkpoint_failed(
        {"actual_deg": 5.0, "error_deg": 0.0, "e_sync_deg": 0.1,
         "cumulative_max_e_sync_deg": 0.71}, 1.0)
    signed_sync_rows = {
        "PLUS_5_ENDPOINT": [
            {"qJ2_logical_rad": str(5.0 * RAD),
             "e_sync_rad": str(value * RAD)}
            for value in ([0.4, -0.4] * 15)
        ]
    }
    signed_sync = endpoint(
        signed_sync_rows, "PLUS_5_ENDPOINT", 5.0, 30)
    assert signed_sync is not None and close(
        signed_sync["e_sync_deg"], 0.4, 1e-12)
    failed_plus = {"actual_deg": 3.5, "error_deg": 1.5,
                   "e_sync_deg": 0.1,
                   "cumulative_max_e_sync_deg": 0.2}
    checkpoints = [("PLUS_5_ENDPOINT", failed_plus, 1.0)]
    exact_segments = [("PLUS_5_ENDPOINT", 40), ("FINAL_DUAL_BRAKE", 5)]
    endpoint_reason = {
        "field_present": True,
        "reason": "PLUS_5_ACCEPTANCE_FAILED_STOP_ROUTE",
        "safety_abort_latched": False,
        "consistent": True,
        "final_rows_complete": True,
    }
    assert exact_failed_endpoint(
        exact_segments, checkpoints, endpoint_reason) == "PLUS_5_ENDPOINT"
    assert exact_failed_endpoint(
        [("PLUS_5_ENDPOINT", 39), ("FINAL_DUAL_BRAKE", 5)],
        checkpoints, endpoint_reason) is None
    missing_reason = dict(endpoint_reason)
    missing_reason["field_present"] = False
    assert exact_failed_endpoint(
        exact_segments, checkpoints, missing_reason) is None
    timing_rows = []
    for tick in range(5):
        timing_rows.append({
            "tick": str(tick),
            "timestamp_s": str(tick * 0.01),
            "cycle_period_ms": "nan" if tick == 0 else "10.0",
            "cycle_jitter_ms": "nan" if tick == 0 else "0.0",
        })
    assert timing_summary(timing_rows, timing_rows, "all")["result"] == "PASS"
    velocity_rows = [
        {"timestamp_s": str(index * 0.01),
         "q_relative_ros_rad": str(index * 0.01)}
        for index in range(11)
    ]
    velocities = expected_j3_velocities(velocity_rows)
    assert velocities[0] == (0.0, 0.0)
    assert close(velocities[4][0], DEG, 1e-10)
    assert close(velocities[10][1], DEG, 1e-10)
    assert audit_phase_prefix([
        ("SESSION_BRAKE_CAPTURE", 50), ("KP_RAMP_HOLD", 51),
        ("CURRENT_HOLD", 100), ("PLUS_5_PROFILE", 85),
        ("PLUS_5_ENDPOINT", 40), ("FIRST_CENTER_PROFILE", 85),
        ("FIRST_CENTER_ENDPOINT", 40), ("MINUS_5_PROFILE", 85),
        ("FINAL_DUAL_BRAKE", 5),
    ], 51)[0]
    gate = next_gate({
        "phase": "j2-kp100-run1", "result": "PASS",
        "safe_tracking_fail_eligible_for_kp140": False,
        "csv": "hardware/v15_24f_ft/j2_kp100_run1.csv",
        "csv_sha256": "0" * 64, "operator_observation": "SAFE",
        "final_brake": "PASS", "center_gate_fix": "YES",
        "timing_100hz": {"result": "PASS"},
        "hold": {"result": "PASS"},
        "failed_endpoint": None,
        "termination": {"reason": "COMPLETED_ROUTE"},
    })
    assert gate is not None
    assert {"center_gate_fix", "timing_100hz_result", "hold_result",
            "prerequisite_result", "safe_tracking_fail_eligible"} <= set(gate)
    print("V15_24F_ANALYZER_SELF_TEST=PASS")
    print("FAIL_CLOSED_PHASE_PREFIX=PASS")
    print("FAILED_CENTER_FORBIDS_LATER_PROFILE_OR_ENDPOINT=PASS")
    print("EXACT_ENDPOINT_TERMINATION_GATE=PASS")
    print("J3_56_COLUMN_AUX_AND_VELOCITY_SCHEMA=PASS")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--csv", type=Path)
    parser.add_argument("--operator-observation",
                        choices=("SAFE", "UNSAFE", "PENDING"), default="PENDING")
    parser.add_argument("--summary-output", type=Path)
    parser.add_argument("--gate-output", type=Path)
    args = parser.parse_args()
    if args.gate_output:
        args.gate_output.unlink(missing_ok=True)
    if args.self_test:
        self_test()
        return 0
    if args.csv is None:
        parser.error("--csv is required")
    if args.csv.name in CONFIGS:
        summary = audit_run(args.csv, args.operator_observation)
    elif args.csv.name in J3_CONFIGS:
        summary = audit_j3_run(args.csv, args.operator_observation)
    else:
        raise ValueError(f"unsupported evidence filename: {args.csv.name}")
    encoded = json.dumps(summary, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.summary_output:
        args.summary_output.write_text(encoded, encoding="utf-8", newline="\n")
    print(encoded, end="")
    gate = next_gate(summary)
    if args.gate_output:
        if gate is None:
            print("NEXT_PHASE_GATE=NOT_WRITTEN", file=sys.stderr)
        else:
            temporary_gate = args.gate_output.with_name(
                args.gate_output.name + ".tmp")
            temporary_gate.unlink(missing_ok=True)
            try:
                temporary_gate.write_text(
                    json.dumps(gate, ensure_ascii=False, indent=2,
                               sort_keys=True) + "\n",
                    encoding="utf-8", newline="\n")
                temporary_gate.replace(args.gate_output)
            finally:
                temporary_gate.unlink(missing_ok=True)
            print(f"NEXT_PHASE_GATE={gate['allowed_phase']}", file=sys.stderr)
    return 0 if summary["result"] in {"PASS", "PENDING_OPERATOR"} else 2


if __name__ == "__main__":
    raise SystemExit(main())

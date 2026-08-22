#!/usr/bin/env python3
"""Audit V15.24E pose-aware J2 gravity runs and build contract artifacts.

This program is deliberately offline: it only reads CSV/JSON evidence and writes
JSON/Markdown/SHA256 artifacts.  It has no serial, SDK, or motor-control path.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
import struct
import subprocess
from collections import Counter
from dataclasses import dataclass
from pathlib import Path
from statistics import median
from typing import Any


DEG = 180.0 / math.pi
RAD = math.pi / 180.0
GEAR = 6.329999923706055
SIGN_A = -1
SIGN_B = +1
EXPECTED_SOURCE_HEAD = "4fc673c77fee836a4931fabec731ea85346f25b3"
EXPECTED_BRANCH = "agent/v15-24e-ft-j2-auto-gravity-compensation"
EXPECTED_MODEL_SHA256 = (
    "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
)
MODEL_ANCHOR_SCHEMA = "SESSION_LOCAL_GRAVITY_ANCHOR_V1_MODEL"
FINAL_ANCHOR_SCHEMA = "SESSION_LOCAL_GRAVITY_ANCHOR_V1"
NEXT_GATE_SCHEMA = "V15_24E_NEXT_PHASE_GATE_V1"
ANCHOR_GATE = "MUJOCO_PHYSICAL_POSE_MATCHED=YES"
BOUNDARY_ROUTE_CONFIRMATION_SOURCE = (
    "USER_CHAT_EXPLICIT_BOUNDARY_B_PLUS_5DEG_STAGE_ROUTE_AUTHORIZATION")
IMPROVEMENT_THRESHOLD_DEG = 0.05
BOUNDARY_TO_TEST_CENTER_RAD = 5.0 * RAD
CENTER_EXCURSION_RAD = 5.0 * RAD
COMMAND_LOWER_BOUNDARY_RAD = 0.0
COMMAND_UPPER_BOUNDARY_RAD = (
    BOUNDARY_TO_TEST_CENTER_RAD + CENTER_EXCURSION_RAD
)
FEEDBACK_LOWER_BOUNDARY_RAD = -0.5 * RAD
FEEDBACK_UPPER_BOUNDARY_RAD = 12.0 * RAD
FORMAL_CENTER_RELATIVE_ROUTE_DEG = (0.0, 5.0, 0.0, -5.0, 0.0)
FULL_BOUNDARY_RELATIVE_ROUTE_DEG = (0.0, 5.0, 10.0, 5.0, 0.0, 5.0, 0.0)

EXPECTED_COLUMNS = (
    "tick", "timestamp_s", "phase", "level", "alpha", "a_mode", "b_mode",
    "a_q_cmd", "b_q_cmd", "a_dq_cmd", "b_dq_cmd",
    "tau_g_model_cmd_nm", "qJ2_model_cmd_rad",
    "a_tff_unclamped", "b_tff_unclamped",
    "a_tau_cmd_literal", "b_tau_cmd_literal",
    "a_tau_cmd_count", "b_tau_cmd_count",
    "a_tau_cmd_decoded", "b_tau_cmd_decoded",
    "a_tff_clamped", "b_tff_clamped", "tff_clamp_used",
    "tau_J2_ff_cmd_decoded", "a_raw", "b_raw",
    "qA_logical_rad", "qB_logical_rad", "qJ2_logical_rad",
    "qJ2_pre_tff_baseline_rad", "qJ2_tff_displacement_rad",
    "q_ref_rad", "e_common_rad", "e_sync_rad",
    "a_dq_feedback", "b_dq_feedback", "a_dq_logical", "b_dq_logical",
    "a_tau", "b_tau", "tau_J2_feedback", "a_temp", "b_temp",
    "a_merror", "b_merror", "a_received_id", "b_received_id",
    "a_send_recv", "b_send_recv", "a_correct", "b_correct",
    "a_crc_ok", "b_crc_ok", "a_valid", "b_valid",
    "cycle_period_ms", "cycle_jitter_ms",
)

PHASE_ORDER = (
    "SESSION_BRAKE_CAPTURE",
    "ZERO_TFF_BASELINE_HOLD",
    "AUTO_GRAVITY_RAMP",
    "AUTO_GRAVITY_HOLD",
    "MOVE_TO_TEST_CENTER_PROFILE",
    "MOVE_TO_TEST_CENTER_ENDPOINT",
    "PLUS_5_PROFILE",
    "PLUS_5_ENDPOINT",
    "FIRST_CENTER_PROFILE",
    "FIRST_CENTER_ENDPOINT",
    "MINUS_5_PROFILE",
    "MINUS_5_ENDPOINT",
    "FINAL_CENTER_PROFILE",
    "FINAL_CENTER_ENDPOINT",
    "RETURN_TO_BOUNDARY_PROFILE",
    "RETURN_TO_BOUNDARY_ENDPOINT",
    "FINAL_DUAL_BRAKE",
)

FIXED_PHASE_COUNTS = {
    "SESSION_BRAKE_CAPTURE": 50,
    "ZERO_TFF_BASELINE_HOLD": 50,
    "AUTO_GRAVITY_HOLD": 100,
    "MOVE_TO_TEST_CENTER_PROFILE": 85,
    "MOVE_TO_TEST_CENTER_ENDPOINT": 40,
    "PLUS_5_PROFILE": 85,
    "PLUS_5_ENDPOINT": 40,
    "FIRST_CENTER_PROFILE": 85,
    "FIRST_CENTER_ENDPOINT": 40,
    "MINUS_5_PROFILE": 85,
    "MINUS_5_ENDPOINT": 40,
    "FINAL_CENTER_PROFILE": 85,
    "FINAL_CENTER_ENDPOINT": 50,
    "RETURN_TO_BOUNDARY_PROFILE": 85,
    "RETURN_TO_BOUNDARY_ENDPOINT": 50,
    "FINAL_DUAL_BRAKE": 5,
}

FORMAL_ROUTE_PHASES = frozenset((
    "PLUS_5_PROFILE", "PLUS_5_ENDPOINT",
    "FIRST_CENTER_PROFILE", "FIRST_CENTER_ENDPOINT",
    "MINUS_5_PROFILE", "MINUS_5_ENDPOINT",
    "FINAL_CENTER_PROFILE", "FINAL_CENTER_ENDPOINT",
))
STAGE_TO_CENTER_PHASES = frozenset((
    "MOVE_TO_TEST_CENTER_PROFILE", "MOVE_TO_TEST_CENTER_ENDPOINT",
))
RETURN_TO_BOUNDARY_PHASES = frozenset((
    "RETURN_TO_BOUNDARY_PROFILE", "RETURN_TO_BOUNDARY_ENDPOINT",
))
BOUNDARY_MOTION_PHASES = frozenset((
    *STAGE_TO_CENTER_PHASES,
    *FORMAL_ROUTE_PHASES,
    *RETURN_TO_BOUNDARY_PHASES,
))
ROUTE_PHASES = BOUNDARY_MOTION_PHASES
BRAKE_PHASES = frozenset(("SESSION_BRAKE_CAPTURE", "FINAL_DUAL_BRAKE"))


@dataclass(frozen=True)
class RunSpec:
    key: str
    phase_name: str
    relative_path: str
    level: str
    alpha: float
    cap_nm: float
    ramp_s: float
    ramp_count: int


RUN_SPECS = {
    "level_a_run": RunSpec(
        "level_a_run", "level-a-run",
        "hardware/v15_24e_ft/j2_pose_gravity_025_run.csv",
        "LEVEL_A_025", 0.25, 0.30, 1.0, 101),
    "level_a_repeat": RunSpec(
        "level_a_repeat", "level-a-repeat",
        "hardware/v15_24e_ft/j2_pose_gravity_025_repeat.csv",
        "LEVEL_A_025", 0.25, 0.30, 1.0, 101),
    "level_b_run": RunSpec(
        "level_b_run", "level-b-run",
        "hardware/v15_24e_ft/j2_pose_gravity_050_run.csv",
        "LEVEL_B_050", 0.50, 0.60, 1.5, 151),
    "level_b_repeat": RunSpec(
        "level_b_repeat", "level-b-repeat",
        "hardware/v15_24e_ft/j2_pose_gravity_050_repeat.csv",
        "LEVEL_B_050", 0.50, 0.60, 1.5, 151),
}


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise RuntimeError(reason)


def close(a: float, b: float, tolerance: float = 1e-11) -> bool:
    return math.isfinite(a) and math.isfinite(b) and abs(a - b) <= tolerance


def is_nan(value: float) -> bool:
    return math.isnan(value)


def f(row: dict[str, str], key: str) -> float:
    try:
        return float(row[key])
    except (KeyError, ValueError) as error:
        raise RuntimeError(f"invalid float field {key!r}") from error


def i(row: dict[str, str], key: str) -> int:
    try:
        return int(row[key])
    except (KeyError, ValueError) as error:
        raise RuntimeError(f"invalid integer field {key!r}") from error


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_json(path: Path) -> dict[str, Any]:
    require(path.is_file(), f"JSON missing: {path}")
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as error:
        raise RuntimeError(f"JSON invalid: {path}") from error
    require(isinstance(value, dict), f"JSON root is not object: {path}")
    return value


def write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=False,
                   allow_nan=False) + "\n",
        encoding="utf-8", newline="\n")


def repo_relative(repo: Path, path: Path) -> str:
    resolved = path.resolve()
    try:
        return resolved.relative_to(repo).as_posix()
    except ValueError as error:
        raise RuntimeError(f"path outside repository: {path}") from error


def percentile_nearest_rank(values: list[float], fraction: float) -> float:
    require(values, "percentile on empty values")
    ordered = sorted(values)
    index = max(0, min(len(ordered) - 1, math.ceil(fraction * len(ordered)) - 1))
    return ordered[index]


def float32(value: float) -> float:
    return struct.unpack("<f", struct.pack("<f", value))[0]


def q8_count(literal: float) -> int:
    return math.trunc(float32(literal) * 256.0)


def smoothstep5(value: float) -> float:
    value = min(1.0, max(0.0, value))
    return value * value * value * (10.0 + value * (-15.0 + 6.0 * value))


def trapezoid_sample(displacement: float, time_s: float) -> tuple[float, float, float]:
    vmax = 10.0 * RAD
    accel = 30.0 * RAD
    sign = -1.0 if displacement < 0.0 else 1.0
    distance = abs(displacement)
    threshold = vmax * vmax / accel
    if distance <= threshold:
        t_acc = math.sqrt(distance / accel)
        v_peak = accel * t_acc
        t_cruise = 0.0
    else:
        t_acc = vmax / accel
        v_peak = vmax
        t_cruise = (distance - accel * t_acc * t_acc) / vmax
    duration = 2.0 * t_acc + t_cruise
    time_s = min(duration, max(0.0, time_s))
    d_acc = 0.5 * accel * t_acc * t_acc
    if time_s < t_acc:
        q = 0.5 * accel * time_s * time_s
        velocity = accel * time_s
    elif time_s < t_acc + t_cruise:
        elapsed = time_s - t_acc
        q = d_acc + v_peak * elapsed
        velocity = v_peak
    elif time_s < duration:
        remaining = duration - time_s
        q = distance - 0.5 * accel * remaining * remaining
        velocity = accel * remaining
    else:
        q = distance
        velocity = 0.0
    return sign * q, sign * velocity, duration


def expected_reference(phase: str, index: int) -> tuple[float, float]:
    if phase in ("ZERO_TFF_BASELINE_HOLD", "AUTO_GRAVITY_RAMP",
                 "AUTO_GRAVITY_HOLD"):
        return COMMAND_LOWER_BOUNDARY_RAD, 0.0
    if phase in ("MOVE_TO_TEST_CENTER_ENDPOINT", "FIRST_CENTER_ENDPOINT",
                 "FINAL_CENTER_ENDPOINT"):
        return BOUNDARY_TO_TEST_CENTER_RAD, 0.0
    if phase == "PLUS_5_ENDPOINT":
        return COMMAND_UPPER_BOUNDARY_RAD, 0.0
    if phase in ("MINUS_5_ENDPOINT", "RETURN_TO_BOUNDARY_ENDPOINT"):
        return COMMAND_LOWER_BOUNDARY_RAD, 0.0
    profile_parameters = {
        "MOVE_TO_TEST_CENTER_PROFILE": (
            COMMAND_LOWER_BOUNDARY_RAD, BOUNDARY_TO_TEST_CENTER_RAD),
        "PLUS_5_PROFILE": (
            BOUNDARY_TO_TEST_CENTER_RAD, CENTER_EXCURSION_RAD),
        "FIRST_CENTER_PROFILE": (
            COMMAND_UPPER_BOUNDARY_RAD, -CENTER_EXCURSION_RAD),
        "MINUS_5_PROFILE": (
            BOUNDARY_TO_TEST_CENTER_RAD, -CENTER_EXCURSION_RAD),
        "FINAL_CENTER_PROFILE": (
            COMMAND_LOWER_BOUNDARY_RAD, CENTER_EXCURSION_RAD),
        "RETURN_TO_BOUNDARY_PROFILE": (
            BOUNDARY_TO_TEST_CENTER_RAD, -BOUNDARY_TO_TEST_CENTER_RAD),
    }
    require(phase in profile_parameters, f"no active reference for {phase}")
    start, displacement = profile_parameters[phase]
    delta_q, velocity, _ = trapezoid_sample(displacement, index * 0.01)
    return start + delta_q, velocity


def validate_model_anchor(path: Path) -> dict[str, Any]:
    doc = load_json(path)
    require(doc.get("schema") == MODEL_ANCHOR_SCHEMA, "model anchor schema mismatch")
    require(doc.get("scope") == "SESSION_ONLY_NOT_PERMANENT_ZERO",
            "model anchor scope mismatch")
    require(doc.get("operator_gate") == ANCHOR_GATE, "model anchor gate mismatch")
    require(doc.get("operator_gate_entry_method") ==
            BOUNDARY_ROUTE_CONFIRMATION_SOURCE,
            "boundary-B anchor confirmation source mismatch")
    require(doc.get("permanent_zero_modified") is False,
            "permanent zero was modified")
    require(doc.get("cad_zero") == "PENDING" and doc.get("ros_zero") == "PENDING",
            "CAD/ROS zero authority mismatch")
    model = doc.get("model", {})
    require(model.get("sha256") == EXPECTED_MODEL_SHA256,
            "frozen model hash mismatch in anchor")
    require(model.get("mujoco_version") == "3.11.0", "MuJoCo version mismatch")
    require(model.get("runtime_gravity_m_s2") == [0.0, 0.0, -9.81],
            "runtime gravity mismatch")
    q = doc.get("q_anchor_model_rad")
    require(isinstance(q, dict), "q_anchor_model_rad missing")
    for joint in ("J2", "J3", "J4", "J5", "J6"):
        require(joint in q and math.isfinite(float(q[joint])),
                f"invalid model anchor {joint}")
    audit = doc.get("uncertainty_audit", {})
    require(audit.get("coverage_complete") is True,
            "anchor uncertainty coverage incomplete")
    require(audit.get("required_sample_count") == 405 and
            audit.get("sample_count") == 405,
            "anchor uncertainty sample count mismatch")
    require(audit.get("all_samples_finite") is True and
            audit.get("finite_sample_count") == 405 and
            audit.get("nonfinite_sample_count") == 0,
            "anchor uncertainty finite-sample contract mismatch")
    require(audit.get("joint_values_clipped") is False,
            "anchor uncertainty audit clipped model joint values")
    require(audit.get("sign_robust") is True, "gravity sign is not robust")
    require(
        audit.get("method") ==
        "BOUNDARY_B_ABSOLUTE_ROUTE_0_TO_PLUS10_5_POINT_X_"
        "J3_J4_J5_J6_SIMULTANEOUS_PLUS_MINUS_5DEG_CORNERS",
        "anchor uncertainty method is not the boundary-B route")
    require(audit.get("reference_pose_label") == "BOUNDARY_B_MODEL_REFERENCE" and
            audit.get("anchor_role") == "STABLE_BOUNDARY_B" and
            audit.get("test_center_role") ==
            "RELATIVE_COMMAND_POINT_NOT_MODEL_ANCHOR",
            "boundary-B/test-center-C anchor roles mismatch")
    require(close(float(audit.get(
                "boundary_to_test_center_j2_offset_deg", math.nan)), 5.0),
            "boundary-to-test-center offset mismatch")
    require(audit.get("formal_test_route_center_relative_deg") ==
            list(FORMAL_CENTER_RELATIVE_ROUTE_DEG),
            "formal center-relative route mismatch")
    require(audit.get("full_planned_route_boundary_relative_deg") ==
            list(FULL_BOUNDARY_RELATIVE_ROUTE_DEG),
            "full boundary-relative route mismatch")
    require(audit.get("audited_boundary_relative_offsets_deg") ==
            [0.0, 2.5, 5.0, 7.5, 10.0],
            "boundary-relative uncertainty offsets mismatch")
    require(close(float(audit.get(
                "planned_route_boundary_relative_min_deg", math.nan)), 0.0) and
            close(float(audit.get(
                "planned_route_boundary_relative_max_deg", math.nan)), 10.0) and
            audit.get("planned_route_crosses_below_boundary") is False,
            "planned route does not preserve the boundary-B interval")
    tau_anchor = float(audit["anchor_tau_g_j2_nm"])
    tau_min = float(audit["tau_g_j2_min_nm"])
    tau_max = float(audit["tau_g_j2_max_nm"])
    require(all(math.isfinite(v) for v in (tau_anchor, tau_min, tau_max)),
            "nonfinite anchor gravity torque")
    require(tau_min <= tau_anchor <= tau_max, "anchor torque outside envelope")
    require((tau_min > 0.0 and tau_max > 0.0) or
            (tau_min < 0.0 and tau_max < 0.0),
            "anchor gravity envelope crosses zero")
    sample_records = audit.get("sample_records")
    require(isinstance(sample_records, list) and len(sample_records) == 405,
            "anchor uncertainty sample records incomplete")
    allowed_j2_offsets = {0.0, 2.5, 5.0, 7.5, 10.0}
    allowed_downstream_offsets = {-5.0, 0.0, 5.0}
    for index, record in enumerate(sample_records):
        require(isinstance(record, dict),
                f"anchor uncertainty record {index} is not an object")
        require(float(record.get("j2_route_delta_deg", math.nan)) in
                allowed_j2_offsets,
                f"anchor uncertainty record {index} has invalid J2 offset")
        downstream = record.get("downstream_delta_deg")
        require(isinstance(downstream, dict) and
                set(downstream) == {"J3", "J4", "J5", "J6"} and
                all(float(value) in allowed_downstream_offsets
                    for value in downstream.values()),
                f"anchor uncertainty record {index} has invalid downstream offsets")
        evaluated = record.get("q_model_rad_evaluated_without_clipping")
        require(isinstance(evaluated, dict) and
                set(evaluated) == {"J2", "J3", "J4", "J5", "J6"} and
                all(math.isfinite(float(value)) for value in evaluated.values()),
                f"anchor uncertainty record {index} has invalid model coordinates")
        require(math.isfinite(float(record.get("tau_g_j2_nm", math.nan))),
                f"anchor uncertainty record {index} has nonfinite torque")
    ledgers = doc.get("frozen_ledgers_sha256", {})
    expected_ledgers = {
        "V15_15_实测质量账本_v1.json":
            "658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a",
        "V15_15_COM账本_v2.json":
            "1cb975890dd121eb41d41bfee414a2c4643f4d917089970fac60009736fc77ae",
        "V15_16_刚体惯量_Engineering_V1.json":
            "c6c398532d7144ed6aa340a8d8b765b333d01f1fddc2e3280106c90565614401",
    }
    require(ledgers == expected_ledgers, "frozen ledger hashes mismatch")
    return doc


def normalized_history(repo: Path, b_path: Path, d_path: Path) -> dict[str, Any]:
    b = load_json(b_path)
    d = load_json(d_path)
    require(b.get("status") == "FAIL", "V15.24B inventory status mismatch")
    b_run = b.get("j2_run_a", {})
    require(close(float(b_run.get("kp", math.nan)), 0.60),
            "V15.24B authoritative run is not Kp=.60")
    require(close(float(b.get("test_parameters", {}).get("tff", math.nan)), 0.0),
            "V15.24B authoritative run is not zero-Tff")
    require(b_run.get("result") == "FAIL", "V15.24B authoritative result mismatch")

    require(d.get("status") == "FAIL", "V15.24D inventory status mismatch")
    d_run = d.get("run1", {})
    command = d.get("command", {})
    require(close(float(command.get("kp", math.nan)), 0.60),
            "V15.24D fixed-Tff run is not Kp=.60")
    require(close(abs(float(command.get("tff_literal_a_rotor_nm", math.nan))), 0.05),
            "V15.24D fixed-Tff magnitude mismatch")
    require(d_run.get("result") == "FAIL", "V15.24D run result mismatch")

    def check_referenced_csv(inventory_path: Path, item: dict[str, Any]) -> None:
        relative = item.get("evidence_file")
        expected = item.get("sha256")
        require(isinstance(relative, str) and isinstance(expected, str),
                f"historical evidence reference missing: {inventory_path}")
        evidence = repo / relative
        require(evidence.is_file(), f"historical CSV missing: {evidence}")
        require(sha256(evidence) == expected,
                f"historical CSV hash mismatch: {evidence}")

    check_referenced_csv(b_path, b_run)
    check_referenced_csv(d_path, d_run)

    def history_item(name: str, run: dict[str, Any]) -> dict[str, Any]:
        plus_actual = float(run["plus_5_actual_deg"])
        minus_actual = float(run["minus_5_actual_deg"])
        plus_error = float(run["plus_5_error_deg"])
        minus_error = float(run["minus_5_error_deg"])
        require(close(plus_error, abs(5.0 - plus_actual), 1e-9),
                f"{name} +5 error inconsistent")
        require(close(minus_error, abs(-5.0 - minus_actual), 1e-9),
                f"{name} -5 error inconsistent")
        logical_tau = run.get("max_abs_logical_paired_tau_feedback_nm")
        return {
            "name": name,
            "inventory_path": repo_relative(repo, b_path if name == "zero_tff" else d_path),
            "inventory_sha256": sha256(b_path if name == "zero_tff" else d_path),
            "csv_path": str(run["evidence_file"]),
            "csv_sha256": str(run["sha256"]),
            "plus_5_actual_deg": plus_actual,
            "plus_5_error_deg": plus_error,
            "minus_5_actual_deg": minus_actual,
            "minus_5_error_deg": minus_error,
            "first_center_error_deg": float(run["first_center_error_deg"]),
            "final_center_error_deg": float(run["final_center_error_deg"]),
            "max_motion_e_sync_deg": float(
                run.get("max_motion_e_sync_deg",
                        run.get("maximum_motion_e_sync_deg"))),
            "max_abs_logical_tau_feedback_nm": (
                float(logical_tau) if logical_tau is not None else None),
            "directional_asymmetry_deg": abs(abs(plus_actual) - abs(minus_actual)),
        }

    return {
        "zero_tff": history_item("zero_tff", b_run),
        "fixed_tff_005": history_item("fixed_tff_005", d_run),
    }


def read_csv_strict(path: Path) -> list[dict[str, str]]:
    require(path.is_file(), f"run CSV missing: {path}")
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        require(tuple(reader.fieldnames or ()) == EXPECTED_COLUMNS,
                f"CSV schema mismatch: {path}")
        rows = list(reader)
    require(rows, f"empty run CSV: {path}")
    require(all(None not in row and set(row) == set(EXPECTED_COLUMNS) for row in rows),
            f"malformed CSV row: {path}")
    return rows


def phase_rows(rows: list[dict[str, str]], name: str) -> list[dict[str, str]]:
    result = [row for row in rows if row["phase"] == name]
    require(result, f"missing phase: {name}")
    return result


def tail_median(rows: list[dict[str, str]], key: str, count: int) -> float:
    require(len(rows) >= count, f"insufficient endpoint rows: {key}")
    return median(f(row, key) for row in rows[-count:])


def expected_phase_counts(spec: RunSpec) -> dict[str, int]:
    result = dict(FIXED_PHASE_COUNTS)
    result["AUTO_GRAVITY_RAMP"] = spec.ramp_count
    return {name: result[name] for name in PHASE_ORDER}


def compare_to_history(run: dict[str, Any], history: dict[str, Any]) -> dict[str, Any]:
    comparisons: dict[str, Any] = {}
    all_improvements: list[float] = []
    for name in ("zero_tff", "fixed_tff_005"):
        baseline = history[name]
        plus_improvement = baseline["plus_5_error_deg"] - run["plus_5_error_deg"]
        minus_improvement = baseline["minus_5_error_deg"] - run["minus_5_error_deg"]
        all_improvements.extend((plus_improvement, minus_improvement))
        comparisons[name] = {
            "plus_endpoint_error_improvement_deg": plus_improvement,
            "minus_endpoint_error_improvement_deg": minus_improvement,
            "both_directions_improve_at_least_0p05deg":
                plus_improvement >= IMPROVEMENT_THRESHOLD_DEG and
                minus_improvement >= IMPROVEMENT_THRESHOLD_DEG,
            "directional_asymmetry_change_deg":
                run["directional_asymmetry_deg"] - baseline["directional_asymmetry_deg"],
        }
    comparisons["improvement_vs_zero_and_fixed_both_directions"] = all(
        value >= IMPROVEMENT_THRESHOLD_DEG for value in all_improvements)
    comparisons["meaningful_improvement_threshold_deg"] = IMPROVEMENT_THRESHOLD_DEG
    return comparisons


def audit_run(path: Path, spec: RunSpec, anchor: dict[str, Any],
              operator_observation: str) -> dict[str, Any]:
    rows = read_csv_strict(path)
    counts = expected_phase_counts(spec)
    require(len(rows) == sum(counts.values()),
            f"{spec.key} total row count mismatch")
    require(Counter(row["phase"] for row in rows) == counts,
            f"{spec.key} phase counts mismatch")
    expected_sequence = [phase for phase in PHASE_ORDER
                         for _ in range(counts[phase])]
    require([row["phase"] for row in rows] == expected_sequence,
            f"{spec.key} phase order/contiguity mismatch")
    require([i(row, "tick") for row in rows] == list(range(len(rows))),
            f"{spec.key} tick sequence mismatch")
    require(all(row["level"] == spec.level for row in rows),
            f"{spec.key} level label mismatch")

    timestamps = [f(row, "timestamp_s") for row in rows]
    require(all(math.isfinite(value) for value in timestamps),
            f"{spec.key} nonfinite timestamp")
    require(all(right > left for left, right in zip(timestamps, timestamps[1:])),
            f"{spec.key} timestamps not strictly increasing")
    require(is_nan(f(rows[0], "cycle_period_ms")) and
            is_nan(f(rows[0], "cycle_jitter_ms")),
            f"{spec.key} first timing fields must be nan")
    for index in range(1, len(rows)):
        period = (timestamps[index] - timestamps[index - 1]) * 1000.0
        require(close(f(rows[index], "cycle_period_ms"), period, 1e-8),
                f"{spec.key} cycle period field mismatch at row {index}")
        require(close(f(rows[index], "cycle_jitter_ms"), abs(period - 10.0), 1e-8),
                f"{spec.key} cycle jitter field mismatch at row {index}")
    intra_phase_periods = [
        (timestamps[index] - timestamps[index - 1]) * 1000.0
        for index in range(1, len(rows))
        if rows[index]["phase"] == rows[index - 1]["phase"]
    ]
    period_median = median(intra_phase_periods)
    period_p95 = percentile_nearest_rank(intra_phase_periods, 0.95)
    period_max = max(intra_phase_periods)
    rate_pass = (9.0 <= period_median <= 11.5 and period_p95 <= 20.0 and
                 period_max <= 100.0)
    require(rate_pass, f"{spec.key} 100Hz timing audit failed")

    capture = phase_rows(rows, "SESSION_BRAKE_CAPTURE")
    a0 = median(f(row, "a_raw") for row in capture)
    b0 = median(f(row, "b_raw") for row in capture)
    require(all(math.isfinite(f(row, "a_raw")) and math.isfinite(f(row, "b_raw"))
                for row in capture), f"{spec.key} invalid raw anchor")

    active = [row for row in rows if row["phase"] not in BRAKE_PHASES]
    pre_terminal = [row for row in rows if row["phase"] != "FINAL_DUAL_BRAKE"]
    require(all(i(row, "a_valid") == 1 and i(row, "b_valid") == 1
                for row in pre_terminal),
            f"{spec.key} invalid pre-terminal feedback frame")
    require(all(i(row, "a_send_recv") == 1 and i(row, "b_send_recv") == 1 and
                i(row, "a_correct") == 1 and i(row, "b_correct") == 1 and
                i(row, "a_crc_ok") == 1 and i(row, "b_crc_ok") == 1
                for row in pre_terminal), f"{spec.key} feedback integrity failure")
    require(all(i(row, "a_merror") == 0 and i(row, "b_merror") == 0
                for row in pre_terminal),
            f"{spec.key} merror nonzero")
    require(all(i(row, "a_received_id") == 0 and i(row, "b_received_id") == 1
                for row in pre_terminal), f"{spec.key} received ID mismatch")
    require(all(0 <= i(row, "a_temp") < 60 and 0 <= i(row, "b_temp") < 60
                for row in pre_terminal), f"{spec.key} temperature envelope failure")

    phase_indices: Counter[str] = Counter()
    max_a_command_residual = 0.0
    max_b_command_residual = 0.0
    max_feedback_mapping_residual = 0.0
    max_formula_residual = 0.0
    max_q8_residual = 0.0
    max_model_pose_residual = 0.0
    clamp_used = False
    previous_active_q = 0.0
    q_anchor_j2 = float(anchor["q_anchor_model_rad"]["J2"])
    audit = anchor["uncertainty_audit"]
    torque_min_authority = float(audit["tau_g_j2_min_nm"])
    torque_max_authority = float(audit["tau_g_j2_max_nm"])
    gravity_sign = 1 if torque_min_authority > 0.0 else -1
    model_torques: list[float] = []
    tff_a_values: list[float] = []
    tff_b_values: list[float] = []

    for row_number, row in enumerate(rows):
        phase = row["phase"]
        index = phase_indices[phase]
        phase_indices[phase] += 1
        brake = phase in BRAKE_PHASES
        expected_mode = 0 if brake else 1
        if phase != "FINAL_DUAL_BRAKE":
            require(i(row, "a_mode") == expected_mode and
                    i(row, "b_mode") == expected_mode,
                    f"{spec.key} feedback mode mismatch at row {row_number}")
        if brake:
            require(close(f(row, "alpha"), 0.0) and
                    i(row, "a_tau_cmd_count") == 0 and
                    i(row, "b_tau_cmd_count") == 0 and
                    close(f(row, "a_tau_cmd_literal"), 0.0) and
                    close(f(row, "b_tau_cmd_literal"), 0.0),
                    f"{spec.key} brake command is not zero at row {row_number}")
            require(close(f(row, "tau_J2_ff_cmd_decoded"), 0.0),
                    f"{spec.key} brake logical Tff is not zero")
            continue

        if phase == "AUTO_GRAVITY_RAMP":
            expected_alpha = spec.alpha * smoothstep5(index / (spec.ramp_count - 1))
        elif phase == "ZERO_TFF_BASELINE_HOLD":
            expected_alpha = 0.0
        else:
            expected_alpha = spec.alpha
        alpha = f(row, "alpha")
        require(close(alpha, expected_alpha, 2e-12),
                f"{spec.key} alpha schedule mismatch at row {row_number}")

        expected_q, expected_dq = expected_reference(phase, index)
        require(close(f(row, "q_ref_rad"), expected_q, 2e-12),
                f"{spec.key} trajectory q mismatch at row {row_number}")
        require(COMMAND_LOWER_BOUNDARY_RAD - 1e-12 <= expected_q <=
                COMMAND_UPPER_BOUNDARY_RAD + 1e-12,
                f"{spec.key} logical command crossed the B-relative route "
                f"boundary at row {row_number}")
        require(close(f(row, "a_dq_cmd"), SIGN_A * GEAR * expected_dq, 2e-11) and
                close(f(row, "b_dq_cmd"), SIGN_B * GEAR * expected_dq, 2e-11),
                f"{spec.key} trajectory dq mapping mismatch at row {row_number}")
        expected_a_q_cmd = a0 + SIGN_A * GEAR * expected_q
        expected_b_q_cmd = b0 + SIGN_B * GEAR * expected_q
        a_command_residual = abs(f(row, "a_q_cmd") - expected_a_q_cmd)
        b_command_residual = abs(f(row, "b_q_cmd") - expected_b_q_cmd)
        max_a_command_residual = max(max_a_command_residual, a_command_residual)
        max_b_command_residual = max(max_b_command_residual, b_command_residual)
        require(a_command_residual <= 1e-11 and b_command_residual <= 1e-11,
                f"{spec.key} position command mapping mismatch at row {row_number}")

        q_a = SIGN_A * (f(row, "a_raw") - a0) / GEAR
        q_b = SIGN_B * (f(row, "b_raw") - b0) / GEAR
        q_j2 = 0.5 * (q_a + q_b)
        e_sync = q_a - q_b
        e_common = expected_q - q_j2
        feedback_residual = max(
            abs(f(row, "qA_logical_rad") - q_a),
            abs(f(row, "qB_logical_rad") - q_b),
            abs(f(row, "qJ2_logical_rad") - q_j2),
            abs(f(row, "e_sync_rad") - e_sync),
            abs(f(row, "e_common_rad") - e_common),
            abs(f(row, "a_dq_logical") - SIGN_A * f(row, "a_dq_feedback") / GEAR),
            abs(f(row, "b_dq_logical") - SIGN_B * f(row, "b_dq_feedback") / GEAR),
            abs(f(row, "tau_J2_feedback") - GEAR *
                (SIGN_A * f(row, "a_tau") + SIGN_B * f(row, "b_tau"))),
        )
        max_feedback_mapping_residual = max(max_feedback_mapping_residual,
                                            feedback_residual)
        require(feedback_residual <= 1e-11,
                f"{spec.key} feedback mapping mismatch at row {row_number}")
        require(FEEDBACK_LOWER_BOUNDARY_RAD - 1e-12 <= q_a <=
                FEEDBACK_UPPER_BOUNDARY_RAD + 1e-12 and
                FEEDBACK_LOWER_BOUNDARY_RAD - 1e-12 <= q_b <=
                FEEDBACK_UPPER_BOUNDARY_RAD + 1e-12 and
                FEEDBACK_LOWER_BOUNDARY_RAD - 1e-12 <= q_j2 <=
                FEEDBACK_UPPER_BOUNDARY_RAD + 1e-12,
                f"{spec.key} one-sided feedback position envelope exceeded")
        require(abs(f(row, "a_dq_logical")) <= 30.0 * RAD + 1e-12 and
                abs(f(row, "b_dq_logical")) <= 30.0 * RAD + 1e-12,
                f"{spec.key} feedback velocity envelope exceeded")

        tau_g = f(row, "tau_g_model_cmd_nm")
        model_q = f(row, "qJ2_model_cmd_rad")
        require(math.isfinite(tau_g) and gravity_sign * tau_g > 0.0,
                f"{spec.key} model gravity sign/finite failure")
        require(torque_min_authority - 1e-9 <= tau_g <= torque_max_authority + 1e-9,
                f"{spec.key} model torque outside anchor uncertainty envelope")
        pose_residual = abs(model_q - (q_anchor_j2 + previous_active_q))
        max_model_pose_residual = max(max_model_pose_residual, pose_residual)
        require(pose_residual <= 1e-11,
                f"{spec.key} model pose is not prior-cycle real J2 at row {row_number}")
        previous_active_q = q_j2
        model_torques.append(tau_g)

        expected_a_unclamped = SIGN_A * alpha * tau_g / (2.0 * GEAR)
        expected_b_unclamped = SIGN_B * alpha * tau_g / (2.0 * GEAR)
        a_unclamped = f(row, "a_tff_unclamped")
        b_unclamped = f(row, "b_tff_unclamped")
        formula_residual = max(abs(a_unclamped - expected_a_unclamped),
                               abs(b_unclamped - expected_b_unclamped))
        max_formula_residual = max(max_formula_residual, formula_residual)
        require(formula_residual <= 1e-12 and close(a_unclamped, -b_unclamped, 1e-12),
                f"{spec.key} pose-gravity Tff formula mismatch at row {row_number}")
        expected_a_literal = min(spec.cap_nm, max(-spec.cap_nm, a_unclamped))
        expected_b_literal = min(spec.cap_nm, max(-spec.cap_nm, b_unclamped))
        a_literal = f(row, "a_tau_cmd_literal")
        b_literal = f(row, "b_tau_cmd_literal")
        require(close(a_literal, expected_a_literal, 1e-12) and
                close(b_literal, expected_b_literal, 1e-12) and
                close(a_literal, -b_literal, 1e-12),
                f"{spec.key} cap/symmetry mismatch at row {row_number}")
        expected_a_clamped = int(abs(expected_a_literal - a_unclamped) > 1e-12)
        expected_b_clamped = int(abs(expected_b_literal - b_unclamped) > 1e-12)
        require(i(row, "a_tff_clamped") == expected_a_clamped and
                i(row, "b_tff_clamped") == expected_b_clamped and
                i(row, "tff_clamp_used") == int(
                    bool(expected_a_clamped or expected_b_clamped)),
                f"{spec.key} clamp flag mismatch at row {row_number}")
        clamp_used = clamp_used or bool(expected_a_clamped or expected_b_clamped)

        a_count = i(row, "a_tau_cmd_count")
        b_count = i(row, "b_tau_cmd_count")
        require(a_count == q8_count(a_literal) and b_count == q8_count(b_literal),
                f"{spec.key} Q8 truncation mismatch at row {row_number}")
        require(a_count == -b_count and
                abs(a_count) <= math.trunc(spec.cap_nm * 256.0) and
                abs(b_count) <= math.trunc(spec.cap_nm * 256.0),
                f"{spec.key} Q8 cap/symmetry failure at row {row_number}")
        a_decoded = a_count / 256.0
        b_decoded = b_count / 256.0
        q8_residual = max(abs(f(row, "a_tau_cmd_decoded") - a_decoded),
                          abs(f(row, "b_tau_cmd_decoded") - b_decoded))
        max_q8_residual = max(max_q8_residual, q8_residual)
        require(q8_residual <= 1e-15,
                f"{spec.key} decoded Q8 mismatch at row {row_number}")
        logical_decoded = GEAR * (SIGN_A * a_decoded + SIGN_B * b_decoded)
        require(close(f(row, "tau_J2_ff_cmd_decoded"), logical_decoded, 1e-12),
                f"{spec.key} logical decoded Tff mismatch at row {row_number}")
        tff_a_values.append(a_decoded)
        tff_b_values.append(b_decoded)

    def command_group_audit(
            name: str, phases: frozenset[str], expected_min: float,
            expected_max: float) -> dict[str, Any]:
        group = [row for row in rows if row["phase"] in phases]
        require(group, f"{spec.key} empty boundary command group: {name}")
        values = [f(row, "q_ref_rad") for row in group]
        require(all(math.isfinite(value) for value in values),
                f"{spec.key} nonfinite q_ref in boundary command group: {name}")
        negative_count = sum(value < -1e-12 for value in values)
        require(negative_count == 0,
                f"{spec.key} negative B-relative command in group: {name}")
        require(close(min(values), expected_min, 2e-12) and
                close(max(values), expected_max, 2e-12),
                f"{spec.key} command range mismatch in group: {name}")
        return {
            "row_count": len(group),
            "min_q_ref_boundary_relative_deg": min(values) * DEG,
            "max_q_ref_boundary_relative_deg": max(values) * DEG,
            "negative_q_ref_row_count": negative_count,
            "all_commands_nonnegative": True,
            "result": "PASS",
        }

    active_commands = [f(row, "q_ref_rad") for row in active]
    require(all(math.isfinite(value) for value in active_commands),
            f"{spec.key} active command is nonfinite")
    require(close(min(active_commands), COMMAND_LOWER_BOUNDARY_RAD, 2e-12) and
            close(max(active_commands), COMMAND_UPPER_BOUNDARY_RAD, 2e-12) and
            all(value >= -1e-12 for value in active_commands),
            f"{spec.key} active B-relative command envelope mismatch")
    command_group_audits = {
        "stage_boundary_b_to_center_c": command_group_audit(
            "stage_boundary_b_to_center_c", STAGE_TO_CENTER_PHASES,
            COMMAND_LOWER_BOUNDARY_RAD, BOUNDARY_TO_TEST_CENTER_RAD),
        "formal_center_relative_route": command_group_audit(
            "formal_center_relative_route", FORMAL_ROUTE_PHASES,
            COMMAND_LOWER_BOUNDARY_RAD, COMMAND_UPPER_BOUNDARY_RAD),
        "return_center_c_to_boundary_b": command_group_audit(
            "return_center_c_to_boundary_b", RETURN_TO_BOUNDARY_PHASES,
            COMMAND_LOWER_BOUNDARY_RAD, BOUNDARY_TO_TEST_CENTER_RAD),
    }

    # The runner applies the sampled feedback guard after the 50-frame BRAKE
    # anchor has been established, including holds, all motion, the return to B,
    # and the terminal BRAKE frames.  Recompute it from raw feedback rather than
    # trusting the derived CSV columns.
    protected_feedback = rows[len(capture):]
    feedback_q_a: list[float] = []
    feedback_q_b: list[float] = []
    feedback_q_j2: list[float] = []
    for row_number, row in enumerate(protected_feedback, start=len(capture)):
        require(i(row, "a_valid") == 1 and i(row, "b_valid") == 1,
                f"{spec.key} invalid boundary-protection feedback at row "
                f"{row_number}")
        q_a = SIGN_A * (f(row, "a_raw") - a0) / GEAR
        q_b = SIGN_B * (f(row, "b_raw") - b0) / GEAR
        q_j2 = 0.5 * (q_a + q_b)
        residual = max(
            abs(f(row, "qA_logical_rad") - q_a),
            abs(f(row, "qB_logical_rad") - q_b),
            abs(f(row, "qJ2_logical_rad") - q_j2),
            abs(f(row, "e_sync_rad") - (q_a - q_b)),
        )
        max_feedback_mapping_residual = max(
            max_feedback_mapping_residual, residual)
        require(residual <= 1e-11,
                f"{spec.key} boundary feedback mapping mismatch at row "
                f"{row_number}")
        feedback_q_a.append(q_a)
        feedback_q_b.append(q_b)
        feedback_q_j2.append(q_j2)
    feedback_min = min(feedback_q_a + feedback_q_b + feedback_q_j2)
    feedback_max = max(feedback_q_a + feedback_q_b + feedback_q_j2)
    require(feedback_min >= FEEDBACK_LOWER_BOUNDARY_RAD - 1e-12,
            f"{spec.key} sampled feedback crossed more than 0.5 deg below B")
    require(feedback_max <= FEEDBACK_UPPER_BOUNDARY_RAD + 1e-12,
            f"{spec.key} sampled feedback exceeded the +12 deg envelope from B")

    zero = phase_rows(rows, "ZERO_TFF_BASELINE_HOLD")
    ramp = phase_rows(rows, "AUTO_GRAVITY_RAMP")
    gravity_hold = phase_rows(rows, "AUTO_GRAVITY_HOLD")
    require(all(i(row, "a_tau_cmd_count") == 0 and
                i(row, "b_tau_cmd_count") == 0 for row in zero),
            f"{spec.key} zero-Tff hold is not zero")
    ramp_elapsed_s = f(ramp[-1], "timestamp_s") - f(ramp[0], "timestamp_s")
    require(0.90 * spec.ramp_s <= ramp_elapsed_s <= 1.25 * spec.ramp_s,
            f"{spec.key} gravity ramp wall duration mismatch")
    ramp_a_counts = [i(row, "a_tau_cmd_count") for row in ramp]
    ramp_b_counts = [i(row, "b_tau_cmd_count") for row in ramp]
    # This is a pose-aware ramp: alpha is monotone, but gravity(q) is
    # recomputed from the previous cycle's measured pose.  Therefore the Q8
    # magnitude is not required to be monotone when the held arm shifts.
    # Per-row alpha, MuJoCo torque, formula, truncation, cap, and A/B
    # opposition are audited independently above.
    require(all(a_count == -b_count for a_count, b_count in
                zip(ramp_a_counts, ramp_b_counts)),
            f"{spec.key} ramp Q8 commands are not opposite")

    baseline_q = tail_median(zero, "qJ2_logical_rad", 30)
    zero_final_sync = abs(tail_median(zero, "e_sync_rad", 30)) * DEG
    zero_max_sync = max(abs(f(row, "e_sync_rad")) for row in zero) * DEG
    zero_max_motion = max(abs(f(row, "qJ2_logical_rad")) for row in zero) * DEG
    zero_hold_pass = (zero_final_sync <= 0.3 and zero_max_sync <= 0.3 and
                      zero_max_motion <= 1.0)
    gravity_q = tail_median(gravity_hold, "qJ2_logical_rad", 50)
    gravity_displacement = (gravity_q - baseline_q) * DEG
    gravity_hold_max_sync = max(abs(f(row, "e_sync_rad"))
                                for row in gravity_hold) * DEG
    ramp_max_sync = max(abs(f(row, "e_sync_rad")) for row in ramp) * DEG
    gravity_max_abs_displacement = max(
        abs(f(row, "qJ2_logical_rad") - baseline_q)
        for row in ramp + gravity_hold) * DEG
    gravity_hold_pass = (gravity_hold_max_sync <= 0.3 and
                         gravity_max_abs_displacement <= 1.5 and
                         ramp_max_sync <= 1.0)

    staged_center = phase_rows(rows, "MOVE_TO_TEST_CENTER_ENDPOINT")
    plus = phase_rows(rows, "PLUS_5_ENDPOINT")
    first = phase_rows(rows, "FIRST_CENTER_ENDPOINT")
    minus = phase_rows(rows, "MINUS_5_ENDPOINT")
    final = phase_rows(rows, "FINAL_CENTER_ENDPOINT")
    boundary_return = phase_rows(rows, "RETURN_TO_BOUNDARY_ENDPOINT")
    staged_center_actual = tail_median(
        staged_center, "qJ2_logical_rad", 30) * DEG
    staged_center_error = abs(staged_center_actual - 5.0)
    staged_center_sync = abs(tail_median(
        staged_center, "e_sync_rad", 30)) * DEG
    plus_absolute = tail_median(plus, "qJ2_logical_rad", 30) * DEG
    plus_actual = plus_absolute - 5.0
    plus_error = abs(plus_actual - 5.0)
    plus_sync = abs(tail_median(plus, "e_sync_rad", 30)) * DEG
    first_absolute = tail_median(first, "qJ2_logical_rad", 30) * DEG
    first_error = abs(first_absolute - 5.0)
    minus_absolute = tail_median(minus, "qJ2_logical_rad", 30) * DEG
    minus_actual = minus_absolute - 5.0
    minus_error = abs(minus_actual + 5.0)
    minus_sync = abs(tail_median(minus, "e_sync_rad", 30)) * DEG
    final_absolute = tail_median(final, "qJ2_logical_rad", 40) * DEG
    final_error = abs(final_absolute - 5.0)
    boundary_return_actual = tail_median(
        boundary_return, "qJ2_logical_rad", 40) * DEG
    boundary_return_error = abs(boundary_return_actual)
    boundary_return_sync = abs(tail_median(
        boundary_return, "e_sync_rad", 40)) * DEG
    route = [row for row in rows if row["phase"] in ROUTE_PHASES]
    max_motion_sync = max(abs(f(row, "e_sync_rad")) for row in route) * DEG
    max_common_error = max(abs(f(row, "e_common_rad")) for row in route) * DEG
    center_pass = first_error <= 0.75 and final_error <= 0.75
    staged_center_pass = staged_center_error <= 0.75
    boundary_return_pass = boundary_return_error <= 0.75
    route_sync_pass = (staged_center_sync <= 0.3 and plus_sync <= 0.3 and
                       minus_sync <= 0.3 and boundary_return_sync <= 0.3 and
                       max_motion_sync <= 0.7)
    position_pass = plus_error <= 1.0 and minus_error <= 1.0 and center_pass
    boundary_route_pass = (
        staged_center_pass and boundary_return_pass and route_sync_pass and
        feedback_min >= FEEDBACK_LOWER_BOUNDARY_RAD - 1e-12 and
        feedback_max <= FEEDBACK_UPPER_BOUNDARY_RAD + 1e-12)

    final_brake = phase_rows(rows, "FINAL_DUAL_BRAKE")
    final_brake_pass = len(final_brake) == 5 and all(
        i(row, "a_mode") == 0 and i(row, "b_mode") == 0 and
        i(row, "a_valid") == 1 and i(row, "b_valid") == 1 and
        i(row, "a_tau_cmd_count") == 0 and i(row, "b_tau_cmd_count") == 0
        for row in final_brake)
    numeric_pass = (zero_hold_pass and gravity_hold_pass and
                    boundary_route_pass and position_pass and
                    final_brake_pass and rate_pass)
    operator_safe = operator_observation == "SAFE"
    result = "PASS" if numeric_pass and operator_safe else "FAIL"
    max_logical_tau = max(abs(f(row, "tau_J2_feedback")) for row in active)
    max_logical_velocity = max(
        max(abs(f(row, "a_dq_logical")), abs(f(row, "b_dq_logical")))
        for row in active) * DEG
    max_temp = max(max(i(row, "a_temp"), i(row, "b_temp")) for row in rows)

    return {
        "phase": spec.phase_name,
        "evidence_file": None,
        "sha256": sha256(path),
        "rows": len(rows),
        "operator_observation": operator_observation,
        "operator_safe": operator_safe,
        "alpha": spec.alpha,
        "cap_per_motor_nm": spec.cap_nm,
        "ramp_duration_contract_s": spec.ramp_s,
        "ramp_elapsed_s": ramp_elapsed_s,
        "a0_raw_rad": a0,
        "b0_raw_rad": b0,
        "capture_valid_frames_each": 50,
        "zero_tff_startup_hold": "PASS" if zero_hold_pass else "FAIL",
        "zero_hold_final_e_sync_deg": zero_final_sync,
        "zero_hold_max_e_sync_deg": zero_max_sync,
        "zero_hold_max_abs_qj2_deg": zero_max_motion,
        "gravity_hold_displacement_deg": gravity_displacement,
        "gravity_ramp_max_e_sync_deg": ramp_max_sync,
        "gravity_hold_max_e_sync_deg": gravity_hold_max_sync,
        "gravity_hold_max_abs_displacement_deg": gravity_max_abs_displacement,
        "gravity_hold_numeric_result": "PASS" if gravity_hold_pass else "FAIL",
        "model_j2_torque_min_nm": min(model_torques),
        "model_j2_torque_max_nm": max(model_torques),
        "j2a_tff_decoded_min_nm": min(tff_a_values),
        "j2a_tff_decoded_max_nm": max(tff_a_values),
        "j2b_tff_decoded_min_nm": min(tff_b_values),
        "j2b_tff_decoded_max_nm": max(tff_b_values),
        "clamp_used": clamp_used,
        "clamp_audit_pass": True,
        "boundary_route": {
            "startup_anchor": "STABLE_BOUNDARY_B",
            "test_center": "C_EQUALS_B_PLUS_5_DEG",
            "formal_center_relative_route_deg":
                list(FORMAL_CENTER_RELATIVE_ROUTE_DEG),
            "full_boundary_relative_route_deg":
                list(FULL_BOUNDARY_RELATIVE_ROUTE_DEG),
            "command_lower_boundary_deg": 0.0,
            "command_upper_boundary_deg": 10.0,
            "no_additional_boundary_margin": True,
            "additional_command_margin_deg": 0.0,
            "actual_no_boundary_crossing_absolute_guarantee": False,
            "sampled_feedback_lower_abort_deg": -0.5,
            "sampled_feedback_upper_abort_deg": 12.0,
            "all_active_commands_nonnegative": True,
            "active_command_min_boundary_relative_deg":
                min(active_commands) * DEG,
            "active_command_max_boundary_relative_deg":
                max(active_commands) * DEG,
            "active_negative_q_ref_row_count": sum(
                value < -1e-12 for value in active_commands),
            "command_groups": command_group_audits,
            "sampled_feedback_rows": len(protected_feedback),
            "qA_feedback_min_boundary_relative_deg": min(feedback_q_a) * DEG,
            "qA_feedback_max_boundary_relative_deg": max(feedback_q_a) * DEG,
            "qB_feedback_min_boundary_relative_deg": min(feedback_q_b) * DEG,
            "qB_feedback_max_boundary_relative_deg": max(feedback_q_b) * DEG,
            "qJ2_feedback_min_boundary_relative_deg": min(feedback_q_j2) * DEG,
            "qJ2_feedback_max_boundary_relative_deg": max(feedback_q_j2) * DEG,
            "sampled_feedback_protection": "PASS",
            "result": "PASS" if boundary_route_pass else "FAIL",
        },
        "move_to_test_center_actual_boundary_relative_deg":
            staged_center_actual,
        "move_to_test_center_error_deg": staged_center_error,
        "move_to_test_center_e_sync_deg": staged_center_sync,
        "move_to_test_center_acceptance":
            "PASS" if staged_center_pass and staged_center_sync <= 0.3 else "FAIL",
        "plus_5_actual_deg": plus_actual,
        "plus_5_absolute_boundary_relative_deg": plus_absolute,
        "plus_5_error_deg": plus_error,
        "plus_5_e_sync_deg": plus_sync,
        "first_center_absolute_boundary_relative_deg": first_absolute,
        "first_center_error_deg": first_error,
        "minus_5_actual_deg": minus_actual,
        "minus_5_absolute_boundary_relative_deg": minus_absolute,
        "minus_5_error_deg": minus_error,
        "minus_5_e_sync_deg": minus_sync,
        "final_center_absolute_boundary_relative_deg": final_absolute,
        "final_center_error_deg": final_error,
        "return_to_boundary_actual_boundary_relative_deg":
            boundary_return_actual,
        "return_to_boundary_error_deg": boundary_return_error,
        "return_to_boundary_e_sync_deg": boundary_return_sync,
        "return_to_boundary_acceptance":
            "PASS" if boundary_return_pass and boundary_return_sync <= 0.3 else
            "FAIL",
        "max_motion_e_sync_deg": max_motion_sync,
        "max_common_mode_error_deg": max_common_error,
        "directional_asymmetry_deg": abs(abs(plus_actual) - abs(minus_actual)),
        "center_acceptance": "PASS" if center_pass else "FAIL",
        "sync_acceptance": "PASS" if route_sync_pass else "FAIL",
        "position_tracking": "PASS" if position_pass else "FAIL",
        "boundary_route_acceptance":
            "PASS" if boundary_route_pass else "FAIL",
        "max_abs_logical_paired_tau_feedback_nm": max_logical_tau,
        "max_abs_logical_velocity_deg_s": max_logical_velocity,
        "max_temperature_c": max_temp,
        "active_invalid_frames": 0,
        "merror_nonzero_frames": 0,
        "gravity_sign_runtime": "PASS",
        "final_both_brake": "PASS" if final_brake_pass else "FAIL",
        "rate_100hz": {
            "result": "PASS", "median_period_ms": period_median,
            "p95_period_ms": period_p95, "max_period_ms": period_max,
        },
        "invariants": {
            "phase_counts": counts,
            "max_a_command_mapping_residual_rad": max_a_command_residual,
            "max_b_command_mapping_residual_rad": max_b_command_residual,
            "max_feedback_mapping_residual": max_feedback_mapping_residual,
            "max_pose_gravity_formula_residual_nm": max_formula_residual,
            "max_q8_decode_residual_nm": max_q8_residual,
            "max_model_prior_cycle_pose_residual_rad": max_model_pose_residual,
            "trajectory_samples_exact": True,
            "boundary_stage_route_return_phase_counts_exact": True,
            "all_active_logical_commands_nonnegative": True,
            "sampled_real_feedback_boundary_guard": True,
            "no_additional_boundary_margin": True,
            "q8_truncation_and_opposite_symmetry": True,
            "cap_flags_and_envelope": True,
            "terminal_five_pair_brake": final_brake_pass,
        },
        "numeric_result": "PASS" if numeric_pass else "FAIL",
        "result": result,
    }


def validate_output_paths(repo: Path, args: argparse.Namespace) -> dict[str, Path]:
    paths = {
        "gravity_anchor": args.gravity_anchor_output.resolve(),
        "inventory": args.inventory_output.resolve(),
        "report": args.report_output.resolve(),
        "sums": args.sums_output.resolve(),
        "gate": args.next_phase_gate.resolve(),
    }
    expected = {
        "gravity_anchor": repo / "hardware/v15_24e_ft/gravity_anchor_v1.json",
        "inventory": repo / "hardware/v15_24e_ft/j2_pose_gravity_inventory.json",
        "report": repo / "docs/V15_24E_FT_J2_POSE_AWARE_GRAVITY_COMPENSATION.md",
        "sums": repo / "hardware/v15_24e_ft/SHA256SUMS",
    }
    for key, expected_path in expected.items():
        require(paths[key] == expected_path.resolve(), f"{key} output path not allowed")
    require(paths["gate"] == Path("/tmp/v15_24e_next_phase_gate.json"),
            "next-phase gate path not allowed")
    return paths


def current_git_value(repo: Path, *arguments: str) -> str:
    try:
        process = subprocess.run(
            ("git", "-C", str(repo), *arguments), check=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True,
            encoding="utf-8")
    except (OSError, subprocess.CalledProcessError) as error:
        raise RuntimeError(f"git query failed: {' '.join(arguments)}") from error
    return process.stdout.strip()


def build_gate(allowed_phase: str, reason: str, prerequisite: dict[str, Any],
               prerequisite_path: str, anchor_sha: str, source_head: str,
               extra: dict[str, Any]) -> dict[str, Any]:
    result: dict[str, Any] = {
        "schema": NEXT_GATE_SCHEMA,
        "eligible": True,
        "allowed_phase": allowed_phase,
        "reason": reason,
        "prerequisite_csv_path": prerequisite_path,
        "prerequisite_csv_sha256": prerequisite["sha256"],
        "prerequisite_result": prerequisite["result"],
        "anchor_sha256": anchor_sha,
        "source_head": source_head,
    }
    result.update(extra)
    return result


def make_markdown(inventory: dict[str, Any]) -> str:
    anchor = inventory["session_anchor"]
    history = inventory["historical_comparison"]
    a = inventory["level_a"]["run"]
    b = inventory["level_b"].get("run")
    repeat = inventory["repeat"]

    def run_lines(label: str, run: dict[str, Any] | None) -> str:
        if run is None:
            return f"## {label}\n\nNot executed.\n"
        boundary = run["boundary_route"]
        return f"""## {label}

- Result: `{run['result']}` (numeric `{run['numeric_result']}`), alpha `{run['alpha']}`.
- Operator observation: `{run['operator_observation']}`; drive-force observation: `{run.get('operator_force_observation', 'NOT_REPORTED')}`.
- Model J2 torque range: `{run['model_j2_torque_min_nm']:.12g}` to `{run['model_j2_torque_max_nm']:.12g} N·m`.
- J2A/J2B decoded Tff ranges: `{run['j2a_tff_decoded_min_nm']:.12g}` to `{run['j2a_tff_decoded_max_nm']:.12g}` / `{run['j2b_tff_decoded_min_nm']:.12g}` to `{run['j2b_tff_decoded_max_nm']:.12g} N·m`; clamp used: `{'YES' if run['clamp_used'] else 'NO'}`.
- Gravity hold displacement / max e_sync: `{run['gravity_hold_displacement_deg']:.12g} / {run['gravity_hold_max_e_sync_deg']:.12g} deg`.
- Stage B->C actual/error/e_sync: `{run['move_to_test_center_actual_boundary_relative_deg']:.12g} / {run['move_to_test_center_error_deg']:.12g} / {run['move_to_test_center_e_sync_deg']:.12g} deg`.
- C-relative +5 actual/error: `{run['plus_5_actual_deg']:.12g} / {run['plus_5_error_deg']:.12g} deg` (absolute from B `{run['plus_5_absolute_boundary_relative_deg']:.12g} deg`); first-center error: `{run['first_center_error_deg']:.12g} deg`.
- C-relative -5 actual/error: `{run['minus_5_actual_deg']:.12g} / {run['minus_5_error_deg']:.12g} deg` (absolute from B `{run['minus_5_absolute_boundary_relative_deg']:.12g} deg`); final-center error: `{run['final_center_error_deg']:.12g} deg`.
- Return C->B actual/error/e_sync: `{run['return_to_boundary_actual_boundary_relative_deg']:.12g} / {run['return_to_boundary_error_deg']:.12g} / {run['return_to_boundary_e_sync_deg']:.12g} deg`; acceptance `{run['return_to_boundary_acceptance']}`.
- Max route e_sync / directional asymmetry: `{run['max_motion_e_sync_deg']:.12g} / {run['directional_asymmetry_deg']:.12g} deg`.
- B-relative command min/max: `{boundary['active_command_min_boundary_relative_deg']:.12g} / {boundary['active_command_max_boundary_relative_deg']:.12g} deg`; negative command rows: `{boundary['active_negative_q_ref_row_count']}`.
- Sampled real qA/qB/qJ2 minima: `{boundary['qA_feedback_min_boundary_relative_deg']:.12g} / {boundary['qB_feedback_min_boundary_relative_deg']:.12g} / {boundary['qJ2_feedback_min_boundary_relative_deg']:.12g} deg`; sampled boundary protection `{boundary['sampled_feedback_protection']}`.
- Valid/merror/final BOTH BRAKE/100 Hz: `PASS / PASS / {run['final_both_brake']} / {run['rate_100hz']['result']}`.
"""

    q = anchor["q_anchor_model_rad"]
    confirmation_method = anchor.get("operator_gate_entry_method", "UNKNOWN")
    return f"""# V15.24E-FT J2 pose-aware gravity compensation

## Result

`FINAL_TASK_RESULT = {inventory['final_task_result']}`. `J2_LOCAL_MOTION_AUTHORITY = {inventory['classification']['J2_LOCAL_MOTION_AUTHORITY']}`.

## Session-local anchor and mapping

The operator matched a rendered MuJoCo pose to the unchanged physical assembly at the stable startup boundary B and explicitly authorized recording the current sliders; the anchor stores the exact gate semantic `{ANCHOR_GATE}` with entry method `{confirmation_method}`. This is a session-local model/encoder anchor, not a permanent joint zero. Test center C is a relative command point at B+5 degrees, not a second model anchor. `CAD_ZERO = PENDING`, `ROS_ZERO = PENDING`, and `PERMANENT_ZERO_MODIFIED = NO`.

- Frozen model SHA256: `{EXPECTED_MODEL_SHA256}`; MuJoCo `3.11.0`; runtime gravity `[0, 0, -9.81] m/s²`.
- Boundary-B q_anchor_model J2..J6: `{q['J2']:.12g}, {q['J3']:.12g}, {q['J4']:.12g}, {q['J5']:.12g}, {q['J6']:.12g} rad`.
- Boundary-B J2 gravity torque / uncertainty envelope: `{anchor['anchor_tau_g_j2_nm']:.12g} / [{anchor['tau_g_j2_min_nm']:.12g}, {anchor['tau_g_j2_max_nm']:.12g}] N·m`; sign robust: `YES` over 405 required samples spanning B-relative J2 0..10 degrees and all J3..J6 +/-5-degree corners.
- Real mapping per run uses 50 valid BRAKE frames: `qA=-1*(rawA-A0)/G`, `qB=+1*(rawB-B0)/G`, `qJ2=(qA+qB)/2`, with `G={GEAR:.17g}`.

## Boundary-segmented route and protection

The exact command route is startup B -> C, then the formal C-relative `0 -> +5 -> 0 -> -5 -> 0`, then C -> B and BOTH BRAKE. In B-relative coordinates this is `0 -> 5 -> 10 -> 5 -> 0 -> 5 -> 0` degrees. `MOVE_TO_TEST_CENTER_PROFILE/ENDPOINT` and `RETURN_TO_BOUNDARY_PROFILE/ENDPOINT` are separate audited phases; every profile has 85 rows, the stage endpoint has 40 rows, and the return endpoint has 50 rows.

All active logical commands are audited fail-closed inside `[0, 10]` degrees relative to B. Sampled real qA, qB, and paired qJ2 are independently reconstructed from raw feedback and guarded inside `[-0.5, 12]` degrees after the B reference is established, including terminal BRAKE feedback. Stage-to-C and return-to-B errors must each be <=0.75 degrees, and their endpoint e_sync must be <=0.3 degrees.

`NO_ADDITIONAL_BOUNDARY_MARGIN = YES`: the lower command is exactly B (`0 deg`), with no inward offset. The `-0.5 deg` sampled-feedback abort is a reactive protection tolerance, not command margin and not proof that the mechanism can never cross B between 100 Hz samples. Therefore `ACTUAL_NO_BOUNDARY_CROSSING_ABSOLUTE_GUARANTEE = NO`.

## Real-time gravity and A/B torque conversion

At 100 Hz, the runner maps the latest measured relative J2 pose onto the frozen model anchor, holds J3..J6 at the visual anchor, sets qvel/qacc to zero, calls MuJoCo forward dynamics, and reads J2 `qfrc_bias`. It then commands `Tff_A=-alpha*tau_g/(2G)` and `Tff_B=+alpha*tau_g/(2G)`, applies the per-level cap, and audits the actual Q8 truncation and decoded values. The analyzer verified the prior-cycle pose mapping, formula, cap flags, opposite symmetry, Q8 encoding, feedback validity, merror, trajectory, and terminal brake for every CSV row.

{run_lines('LEVEL-A alpha=.25', a)}
{run_lines('LEVEL-B alpha=.50', b)}

## Historical comparison and directional asymmetry

- V15.24B authoritative zero-Tff: +5/-5 errors `{history['zero_tff']['plus_5_error_deg']:.12g} / {history['zero_tff']['minus_5_error_deg']:.12g} deg`; asymmetry `{history['zero_tff']['directional_asymmetry_deg']:.12g} deg`.
- V15.24D fixed Tff=.05: +5/-5 errors `{history['fixed_tff_005']['plus_5_error_deg']:.12g} / {history['fixed_tff_005']['minus_5_error_deg']:.12g} deg`; asymmetry `{history['fixed_tff_005']['directional_asymmetry_deg']:.12g} deg`.
- Auto-gravity assessment uses the C-relative +5/-5 endpoint metrics: better than zero-Tff `{inventory['combined']['better_than_zero_tff']}`, better than fixed Tff=.05 `{inventory['combined']['better_than_fixed_tff_005']}`, directional asymmetry reduced `{inventory['combined']['directional_asymmetry_reduced']}`.
- The frozen Level-B eligibility threshold is a >=`{IMPROVEMENT_THRESHOLD_DEG}` degree endpoint-error improvement in both directions versus both historical runs, while Level A remains a formal position FAIL and all safety/sync/sign/center/clamp audits pass.

## Alpha strategy and scope

Level A uses alpha `.25`, a 1.0 s quintic ramp, and a `.30 N·m` per-motor cap. Level B is permitted only by the offline eligibility gate and uses alpha `.50`, a 1.5 s quintic ramp, and a `.60 N·m` cap. Alpha above `.50` is prohibited. Repeat status: `{repeat['used']}` / `{repeat['result']}`. Best validated alpha: `{inventory['combined']['best_alpha']}`.

This task tested position-control gravity feedforward only. `ZERO_GRAVITY_TEACH_MODE = NOT_IMPLEMENTED`. J3 work was not used. A full-state gravity controller is worth considering only if the final repeated authority is PASS; otherwise the unresolved torque/current limit, static friction/brake, model-load mismatch, and anchor accuracy must be isolated in a later task.
"""


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    parser.add_argument("--repo", type=Path)
    parser.add_argument("--model-anchor", type=Path,
                        default=Path("/tmp/v15_24e_model_anchor.json"))
    parser.add_argument("--level-a-run", type=Path)
    parser.add_argument("--level-a-repeat", type=Path)
    parser.add_argument("--level-b-run", type=Path)
    parser.add_argument("--level-b-repeat", type=Path)
    parser.add_argument("--level-a-run-operator", choices=("SAFE", "ABNORMAL"))
    parser.add_argument(
        "--level-a-run-force-observation",
        choices=("NOT_REPORTED", "HIGHER_DRIVE_FORCE_REQUIRED"),
        default="NOT_REPORTED")
    parser.add_argument("--level-a-repeat-operator", choices=("SAFE", "ABNORMAL"))
    parser.add_argument("--level-b-run-operator", choices=("SAFE", "ABNORMAL"))
    parser.add_argument("--level-b-repeat-operator", choices=("SAFE", "ABNORMAL"))
    parser.add_argument("--history-b-inventory", type=Path)
    parser.add_argument("--history-d-inventory", type=Path)
    parser.add_argument("--gravity-anchor-output", type=Path)
    parser.add_argument("--inventory-output", type=Path)
    parser.add_argument("--report-output", type=Path)
    parser.add_argument("--sums-output", type=Path)
    parser.add_argument("--next-phase-gate", type=Path,
                        default=Path("/tmp/v15_24e_next_phase_gate.json"))
    parser.add_argument("--power-off-confirmed", choices=("YES", "NO", "PENDING"),
                        default="PENDING")
    parser.add_argument("--source-head")
    args = parser.parse_args()

    if args.self_test:
        require(len(EXPECTED_COLUMNS) == 58, "CSV column count self-test")
        level_a_counts = expected_phase_counts(RUN_SPECS["level_a_run"])
        level_b_counts = expected_phase_counts(RUN_SPECS["level_b_run"])
        require(sum(level_a_counts.values()) == 1076,
                "Level-A phase count self-test")
        require(sum(level_b_counts.values()) == 1126,
                "Level-B phase count self-test")
        require(level_a_counts["MOVE_TO_TEST_CENTER_PROFILE"] == 85 and
                level_a_counts["MOVE_TO_TEST_CENTER_ENDPOINT"] == 40 and
                level_a_counts["RETURN_TO_BOUNDARY_PROFILE"] == 85 and
                level_a_counts["RETURN_TO_BOUNDARY_ENDPOINT"] == 50,
                "boundary stage/return phase count self-test")
        active_reference_samples = [
            expected_reference(phase, index)[0]
            for phase in PHASE_ORDER if phase not in BRAKE_PHASES
            for index in range(level_a_counts[phase])
        ]
        require(close(min(active_reference_samples), 0.0) and
                close(max(active_reference_samples), 10.0 * RAD) and
                all(value >= -1e-12 for value in active_reference_samples),
                "boundary route command envelope self-test")
        require([
            expected_reference(phase, 0)[0] * DEG
            for phase in (
                "ZERO_TFF_BASELINE_HOLD",
                "MOVE_TO_TEST_CENTER_ENDPOINT", "PLUS_5_ENDPOINT",
                "FIRST_CENTER_ENDPOINT", "MINUS_5_ENDPOINT",
                "FINAL_CENTER_ENDPOINT", "RETURN_TO_BOUNDARY_ENDPOINT")
        ] == list(FULL_BOUNDARY_RELATIVE_ROUTE_DEG),
                "boundary route waypoint self-test")
        require(q8_count(-0.05) == -12 and q8_count(0.05) == 12,
                "Q8 truncation self-test")
        _, _, duration = trapezoid_sample(5.0 * RAD, 0.0)
        require(math.ceil(duration * 100.0) + 1 == 85,
                "trajectory count self-test")
        print("V15_24E_POSE_GRAVITY_ANALYZER_SELF_TEST=PASS")
        print("SERIAL_OR_MOTOR_ACCESS=NO")
        print("LEVEL_A_ROWS=1076")
        print("LEVEL_B_ROWS=1126")
        print("COMMAND_ROUTE_B_RELATIVE_DEG=0_TO_5_TO_10_TO_5_TO_0_TO_5_TO_0")
        print("ALL_ACTIVE_LOGICAL_COMMANDS_NONNEGATIVE=YES")
        print("NO_ADDITIONAL_BOUNDARY_MARGIN=YES")
        print("ACTUAL_NO_BOUNDARY_CROSSING_ABSOLUTE_GUARANTEE=NO")
        print("MEANINGFUL_IMPROVEMENT_THRESHOLD_DEG=0.05")
        return 0

    require(args.repo is not None, "--repo is required")
    repo = args.repo.resolve()
    require(repo.is_dir(), "repository directory missing")
    defaults = {
        "level_a_run": repo / RUN_SPECS["level_a_run"].relative_path,
        "history_b_inventory": repo / "hardware/v15_24b_ft/final_installed_load_inventory.json",
        "history_d_inventory": repo / "hardware/v15_24d_ft/j2_tff005_inventory.json",
        "gravity_anchor_output": repo / "hardware/v15_24e_ft/gravity_anchor_v1.json",
        "inventory_output": repo / "hardware/v15_24e_ft/j2_pose_gravity_inventory.json",
        "report_output": repo / "docs/V15_24E_FT_J2_POSE_AWARE_GRAVITY_COMPENSATION.md",
        "sums_output": repo / "hardware/v15_24e_ft/SHA256SUMS",
    }
    for name, value in defaults.items():
        if getattr(args, name) is None:
            setattr(args, name, value)
    require(args.level_a_run_operator is not None,
            "--level-a-run-operator is required")
    paths = validate_output_paths(repo, args)

    source_head = args.source_head or current_git_value(repo, "rev-parse", "HEAD")
    require(re.fullmatch(r"[0-9a-f]{40}", source_head) is not None,
            "source HEAD is not a full SHA")
    require(source_head == EXPECTED_SOURCE_HEAD, "source HEAD mismatch")
    branch = current_git_value(repo, "branch", "--show-current")
    require(branch == EXPECTED_BRANCH, "branch mismatch")

    anchor_path = args.model_anchor.resolve()
    require(anchor_path == Path("/tmp/v15_24e_model_anchor.json"),
            "model anchor path not allowed")
    anchor = validate_model_anchor(anchor_path)
    anchor_sha = sha256(anchor_path)
    history = normalized_history(
        repo, args.history_b_inventory.resolve(), args.history_d_inventory.resolve())

    input_paths: dict[str, Path | None] = {
        "level_a_run": args.level_a_run.resolve(),
        "level_a_repeat": args.level_a_repeat.resolve() if args.level_a_repeat else None,
        "level_b_run": args.level_b_run.resolve() if args.level_b_run else None,
        "level_b_repeat": args.level_b_repeat.resolve() if args.level_b_repeat else None,
    }
    operators = {
        "level_a_run": args.level_a_run_operator,
        "level_a_repeat": args.level_a_repeat_operator,
        "level_b_run": args.level_b_run_operator,
        "level_b_repeat": args.level_b_repeat_operator,
    }
    evidence_dir = repo / "hardware/v15_24e_ft"
    allowed_evidence_names = {
        "gravity_anchor_v1.json",
        "j2_pose_gravity_025_run.csv",
        "j2_pose_gravity_025_repeat.csv",
        "j2_pose_gravity_050_run.csv",
        "j2_pose_gravity_050_repeat.csv",
        "j2_pose_gravity_inventory.json",
        "SHA256SUMS",
    }
    if evidence_dir.exists():
        unexpected = sorted(
            item.name for item in evidence_dir.iterdir()
            if not item.is_file() or item.name not in allowed_evidence_names)
        require(not unexpected,
                "unexpected V15.24E evidence entries: " + ", ".join(unexpected))
    for key, spec in RUN_SPECS.items():
        expected_csv = repo / spec.relative_path
        if input_paths[key] is None:
            require(not expected_csv.exists(),
                    f"unanalysed optional CSV exists: {spec.relative_path}")
    runs: dict[str, dict[str, Any]] = {}
    for key, input_path in input_paths.items():
        if input_path is None:
            require(operators[key] is None, f"operator observation without {key} CSV")
            continue
        require(repo_relative(repo, input_path) == RUN_SPECS[key].relative_path,
                f"{key} input path not allowed")
        require(operators[key] is not None, f"{key} operator observation required")
        run = audit_run(input_path, RUN_SPECS[key], anchor, operators[key])
        if key == "level_a_run":
            run["operator_force_observation"] = (
                args.level_a_run_force_observation)
        run["evidence_file"] = RUN_SPECS[key].relative_path
        run["historical_comparison"] = compare_to_history(run, history)
        runs[key] = run

    require("level_a_run" in runs, "Level-A first run is required")
    a = runs["level_a_run"]
    a_comparison = a["historical_comparison"]
    a_safety_and_sync_pass = (
        a["operator_safe"] and a["zero_tff_startup_hold"] == "PASS" and
        a["gravity_hold_numeric_result"] == "PASS" and
        a["sync_acceptance"] == "PASS" and
        a["center_acceptance"] == "PASS" and
        a["boundary_route_acceptance"] == "PASS" and
        a["gravity_sign_runtime"] == "PASS" and
        a["active_invalid_frames"] == 0 and a["merror_nonzero_frames"] == 0 and
        a["clamp_audit_pass"] and a["final_both_brake"] == "PASS" and
        a["rate_100hz"]["result"] == "PASS"
    )
    level_b_eligible = (
        a["numeric_result"] == "FAIL" and a["position_tracking"] == "FAIL" and
        a_safety_and_sync_pass and
        a_comparison["improvement_vs_zero_and_fixed_both_directions"]
    )

    if "level_a_repeat" in runs:
        require(a["result"] == "PASS", "Level-A repeat without first-run PASS")
    if a["result"] == "PASS":
        require("level_b_run" not in runs and "level_b_repeat" not in runs,
                "Level B prohibited after Level-A PASS")
    if "level_b_run" in runs:
        require(level_b_eligible, "Level-B run without frozen eligibility")
    if "level_b_repeat" in runs:
        require("level_b_run" in runs and runs["level_b_run"]["result"] == "PASS",
                "Level-B repeat without first-run PASS")

    gate: dict[str, Any] | None = None
    if a["result"] == "PASS" and "level_a_repeat" not in runs:
        gate = build_gate(
            "level-a-repeat", "LEVEL_A_FIRST_RUN_PASS", a,
            RUN_SPECS["level_a_run"].relative_path, anchor_sha, source_head,
            {"level_a_numeric_result": "PASS"})
    elif level_b_eligible and "level_b_run" not in runs:
        gate = build_gate(
            "level-b-run", "LEVEL_A_SAFE_BIDIRECTIONAL_IMPROVEMENT_POSITION_FAIL", a,
            RUN_SPECS["level_a_run"].relative_path, anchor_sha, source_head,
            {
                "level_a_numeric_result": "FAIL",
                "safety_and_sync_pass": True,
                "gravity_sign_robust": True,
                "boundary_route_pass": True,
                "all_active_commands_nonnegative": True,
                "sampled_feedback_boundary_protection": "PASS",
                "no_additional_boundary_margin": True,
                "improvement_vs_zero_and_fixed_both_directions": True,
                "meaningful_improvement_threshold_deg": IMPROVEMENT_THRESHOLD_DEG,
            })
    elif ("level_b_run" in runs and runs["level_b_run"]["result"] == "PASS" and
          "level_b_repeat" not in runs):
        b_run = runs["level_b_run"]
        gate = build_gate(
            "level-b-repeat", "LEVEL_B_FIRST_RUN_PASS", b_run,
            RUN_SPECS["level_b_run"].relative_path, anchor_sha, source_head,
            {"level_b_numeric_result": "PASS"})
    if gate is None:
        paths["gate"].unlink(missing_ok=True)
    else:
        write_json(paths["gate"], gate)

    real_anchors = {}
    for key, run in runs.items():
        real_anchors[key] = {
            "csv_path": run["evidence_file"],
            "csv_sha256": run["sha256"],
            "capture_mode": "BRAKE",
            "valid_frames_each": 50,
            "j2a_raw_anchor_rad": run["a0_raw_rad"],
            "j2b_raw_anchor_rad": run["b0_raw_rad"],
        }
    audit = anchor["uncertainty_audit"]
    gravity_anchor = {
        "schema": FINAL_ANCHOR_SCHEMA,
        "scope": "SESSION_ONLY_NOT_PERMANENT_ZERO",
        "anchor_method": (
            "OPERATOR_VISUAL_MUJOCO_BOUNDARY_B_POSE_MATCH_PLUS_PER_RUN_"
            "50_VALID_BRAKE_FRAMES"),
        "model_anchor_role": "STABLE_BOUNDARY_B",
        "test_center_role": "C_EQUALS_BOUNDARY_B_PLUS_5_DEG_RELATIVE_COMMAND",
        "boundary_to_test_center_j2_offset_deg": 5.0,
        "formal_test_route_center_relative_deg":
            list(FORMAL_CENTER_RELATIVE_ROUTE_DEG),
        "full_planned_route_boundary_relative_deg":
            list(FULL_BOUNDARY_RELATIVE_ROUTE_DEG),
        "no_additional_boundary_margin": True,
        "additional_command_margin_deg": 0.0,
        "actual_no_boundary_crossing_absolute_guarantee": False,
        "operator_gate": ANCHOR_GATE,
        "operator_gate_entry_method": anchor.get("operator_gate_entry_method"),
        "slider_screenshot_sha256": anchor.get("slider_screenshot_sha256"),
        "model_anchor_source_path": str(anchor_path),
        "model_anchor_source_sha256": anchor_sha,
        "model": anchor["model"],
        "frozen_ledgers_sha256": anchor["frozen_ledgers_sha256"],
        "q_anchor_model_rad": anchor["q_anchor_model_rad"],
        "q_anchor_model_deg": anchor["q_anchor_model_deg"],
        "anchor_tau_g_j2_nm": float(audit["anchor_tau_g_j2_nm"]),
        "tau_g_j2_min_nm": float(audit["tau_g_j2_min_nm"]),
        "tau_g_j2_max_nm": float(audit["tau_g_j2_max_nm"]),
        "gravity_sign_robust": True,
        "uncertainty_coverage_complete": True,
        "uncertainty_sample_count": int(audit["sample_count"]),
        "nominal_joint_range_extrapolation_sample_count": int(
            audit.get("nominal_joint_range_extrapolation_sample_count", 0)),
        "nominal_joint_range_violation_record_count": int(
            audit.get("nominal_joint_range_violation_record_count", 0)),
        "real_encoder_anchors": real_anchors,
        "real_to_model_mapping": {
            "gear_ratio": GEAR, "j2a_sign": SIGN_A, "j2b_sign": SIGN_B,
            "qA": "-1*(rawA-A0)/G", "qB": "+1*(rawB-B0)/G",
            "qJ2_relative": "0.5*(qA+qB)",
            "qJ2_model": "q_anchor_model_J2+qJ2_relative",
        },
        "permanent_zero_modified": False,
        "cad_zero": "PENDING", "ros_zero": "PENDING",
    }
    write_json(paths["gravity_anchor"], gravity_anchor)

    authority_pass = False
    repeated_level = None
    if ("level_a_repeat" in runs and a["result"] == "PASS" and
            runs["level_a_repeat"]["result"] == "PASS"):
        authority_pass = True
        repeated_level = "LEVEL_A_025"
    if ("level_b_repeat" in runs and runs["level_b_run"]["result"] == "PASS" and
            runs["level_b_repeat"]["result"] == "PASS"):
        authority_pass = True
        repeated_level = "LEVEL_B_050"

    available_primary = [a]
    if "level_b_run" in runs:
        available_primary.append(runs["level_b_run"])
    best_observed = min(available_primary,
                        key=lambda run: run["plus_5_error_deg"] +
                        run["minus_5_error_deg"])
    best_comparison = best_observed["historical_comparison"]
    better_zero = ("YES" if best_comparison["zero_tff"]
                   ["both_directions_improve_at_least_0p05deg"] else "NO")
    better_fixed = ("YES" if best_comparison["fixed_tff_005"]
                    ["both_directions_improve_at_least_0p05deg"] else "NO")
    asymmetry_supported = all(
        best_observed["directional_asymmetry_deg"] <=
        history[name]["directional_asymmetry_deg"] - IMPROVEMENT_THRESHOLD_DEG
        for name in ("zero_tff", "fixed_tff_005"))
    asymmetry_result = "SUPPORTED" if asymmetry_supported else "NOT_SUPPORTED"

    if authority_pass:
        final_state = "PASS" if args.power_off_confirmed == "YES" else "IN_PROGRESS_POWER_OFF_REQUIRED"
    elif gate is not None:
        final_state = "IN_PROGRESS_NEXT_PHASE_AUTHORIZED"
    else:
        final_state = "FAIL" if args.power_off_confirmed == "YES" else "IN_PROGRESS_POWER_OFF_REQUIRED"
    repeat_keys = [key for key in ("level_a_repeat", "level_b_repeat") if key in runs]
    repeat_result = runs[repeat_keys[0]]["result"] if repeat_keys else "NOT_EXECUTED"
    best_alpha: Any = (0.25 if repeated_level == "LEVEL_A_025" else
                       0.50 if repeated_level == "LEVEL_B_050" else
                       "NOT_VALIDATED")

    inventory = {
        "schema": "V15_24E_J2_POSE_GRAVITY_INVENTORY_V1",
        "task": "V15.24E-FT J2 pose-aware automatic gravity compensation",
        "source_head": source_head,
        "branch": branch,
        "status": final_state,
        "frozen_authority": {
            "model_sha256_verified": True,
            "mass_com_inertia_modified": False,
            "permanent_zero_modified": False,
            "j2a_id": 0, "j2b_id": 1,
            "j2a_raw_to_ros_sign": -1, "j2b_raw_to_ros_sign": 1,
            "gear_ratio": GEAR, "tff_authority": "MOTOR_ROTOR_TORQUE_Q8",
        },
        "session_anchor": {
            "artifact": repo_relative(repo, paths["gravity_anchor"]),
            "artifact_sha256": sha256(paths["gravity_anchor"]),
            "model_anchor_sha256": anchor_sha,
            "anchor_role": "STABLE_BOUNDARY_B",
            "operator_gate_entry_method":
                anchor.get("operator_gate_entry_method"),
            "test_center_c_boundary_relative_deg": 5.0,
            "q_anchor_model_rad": anchor["q_anchor_model_rad"],
            "raw_anchor_level_a_run": {
                "j2a_rad": a["a0_raw_rad"], "j2b_rad": a["b0_raw_rad"]},
            "gravity_sign_robust": True,
            "anchor_tau_g_j2_nm": float(audit["anchor_tau_g_j2_nm"]),
            "tau_g_j2_min_nm": float(audit["tau_g_j2_min_nm"]),
            "tau_g_j2_max_nm": float(audit["tau_g_j2_max_nm"]),
        },
        "boundary_route_contract": {
            "startup": "STABILIZE_AT_BOUNDARY_B",
            "stage": "ACTIVE_B_TO_C_PLUS_5_DEG",
            "formal_route_center_relative_deg":
                list(FORMAL_CENTER_RELATIVE_ROUTE_DEG),
            "full_route_boundary_relative_deg":
                list(FULL_BOUNDARY_RELATIVE_ROUTE_DEG),
            "terminal": "RETURN_TO_BOUNDARY_B_THEN_BOTH_BRAKE",
            "phase_order": list(PHASE_ORDER),
            "level_a_phase_counts":
                expected_phase_counts(RUN_SPECS["level_a_run"]),
            "level_b_phase_counts":
                expected_phase_counts(RUN_SPECS["level_b_run"]),
            "command_range_boundary_relative_deg": [0.0, 10.0],
            "sampled_feedback_abort_range_boundary_relative_deg": [-0.5, 12.0],
            "no_additional_boundary_margin": True,
            "additional_command_margin_deg": 0.0,
            "actual_no_boundary_crossing_absolute_guarantee": False,
        },
        "level_a": {
            "run": a,
            "level_b_eligibility": {
                "eligible": level_b_eligible,
                "level_a_formal_position_fail": a["position_tracking"] == "FAIL",
                "safety_and_sync_pass": a_safety_and_sync_pass,
                "gravity_sign_robust": True,
                "center_pass": a["center_acceptance"] == "PASS",
                "boundary_route_pass":
                    a["boundary_route_acceptance"] == "PASS",
                "clamp_audit_pass": a["clamp_audit_pass"],
                "improvement_vs_zero_and_fixed_both_directions":
                    a_comparison["improvement_vs_zero_and_fixed_both_directions"],
                "meaningful_improvement_threshold_deg": IMPROVEMENT_THRESHOLD_DEG,
            },
            "repeat": runs.get("level_a_repeat", {
                "executed": False, "result": "NOT_EXECUTED"}),
        },
        "level_b": {
            "executed": "level_b_run" in runs,
            "run": runs.get("level_b_run"),
            "repeat": runs.get("level_b_repeat", {
                "executed": False, "result": "NOT_EXECUTED"}),
        },
        "repeat": {
            "used": "YES" if repeat_keys else "NO",
            "level": repeated_level or "N/A",
            "result": repeat_result,
        },
        "historical_comparison": history,
        "combined": {
            "selected_observed_level": best_observed["phase"],
            "better_than_zero_tff": better_zero,
            "better_than_fixed_tff_005": better_fixed,
            "directional_asymmetry_reduced": asymmetry_result,
            "gravity_contribution_confidence":
                "HIGH" if authority_pass and better_zero == "YES" and
                better_fixed == "YES" else "INCONCLUSIVE",
            "best_alpha": best_alpha,
        },
        "classification": {
            "POSITION_CONTROL_GRAVITY_FEEDFORWARD": "TESTED",
            "ZERO_GRAVITY_TEACH_MODE": "NOT_IMPLEMENTED",
            "NO_ADDITIONAL_BOUNDARY_MARGIN": "YES",
            "ACTUAL_NO_BOUNDARY_CROSSING_ABSOLUTE_GUARANTEE": "NO",
            "J2_INSTALLED_LOAD_BIDIRECTIONAL_5DEG":
                "PASS" if authority_pass else "FAIL",
            "J2_LOCAL_MOTION_AUTHORITY": "PASS" if authority_pass else "NOT_GRANTED",
        },
        "safety": {
            "final_both_brake": list(runs.values())[-1]["final_both_brake"],
            "operator_observations_all_safe": all(
                run["operator_safe"] for run in runs.values()),
            "boundary_routes_all_pass": all(
                run["boundary_route_acceptance"] == "PASS"
                for run in runs.values()),
            "all_active_commands_nonnegative": all(
                run["boundary_route"]["all_active_commands_nonnegative"]
                for run in runs.values()),
            "sampled_feedback_boundary_protection_all_pass": all(
                run["boundary_route"]["sampled_feedback_protection"] == "PASS"
                for run in runs.values()),
            "power_24v_off": args.power_off_confirmed == "YES",
            "j3_work_used": False,
        },
        "next_phase_gate": gate or {"eligible": False, "allowed_phase": None},
        "unresolved_items": ([] if authority_pass else [
            "J2 repeated local-motion authority is not established.",
            "Internal torque/current limit, static friction/brake, model-load mismatch, and anchor accuracy remain possible contributors.",
            "Alpha above 0.50, Kp increase, friction compensation, and velocity tuning were not authorized.",
        ]),
        "next_single_task": ("J3_INSTALLED_LOAD_VALIDATION" if authority_pass else
                             "J2_TORQUE_LIMIT_FRICTION_MODEL_ANCHOR_DIAGNOSTIC"),
        "final_task_result": final_state,
    }
    write_json(paths["inventory"], inventory)
    paths["report"].parent.mkdir(parents=True, exist_ok=True)
    paths["report"].write_text(make_markdown(inventory), encoding="utf-8", newline="\n")

    owned: list[Path] = [paths["gravity_anchor"]]
    owned.extend(input_paths[key] for key in RUN_SPECS if input_paths[key] is not None)
    owned.extend((paths["inventory"], paths["report"],
                  repo / "tools/hardware/v15_24e_ft_j2_pose_gravity.cpp",
                  repo / "tools/hardware/v15_24e_ft_j2_brake_pose_capture.cpp",
                  repo / "tools/j2_gravity_anchor_v15_24e.py",
                  repo / "tools/analyze_j2_pose_gravity_v15_24e.py"))
    unique_owned: list[Path] = []
    for path in owned:
        require(path is not None and path.is_file(), f"owned artifact missing: {path}")
        if path not in unique_owned:
            unique_owned.append(path)
    paths["sums"].parent.mkdir(parents=True, exist_ok=True)
    paths["sums"].write_text("".join(
        f"{sha256(path)}  {repo_relative(repo, path)}\n" for path in unique_owned),
        encoding="utf-8", newline="\n")

    print(json.dumps({
        "level_a_result": a["result"],
        "level_b_eligible": level_b_eligible,
        "allowed_next_phase": gate["allowed_phase"] if gate else None,
        "repeat_used": bool(repeat_keys),
        "j2_local_motion_authority":
            inventory["classification"]["J2_LOCAL_MOTION_AUTHORITY"],
        "final_task_result": final_state,
        "final_both_brake": inventory["safety"]["final_both_brake"],
        "power_24v_off": inventory["safety"]["power_24v_off"],
    }, ensure_ascii=False, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

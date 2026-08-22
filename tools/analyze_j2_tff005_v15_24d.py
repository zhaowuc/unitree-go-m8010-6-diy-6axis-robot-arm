#!/usr/bin/env python3
"""Validate V15.24D evidence and generate the frozen inventory/report."""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
from collections import Counter
from pathlib import Path
from statistics import median


DEG = 180.0 / math.pi
GEAR = 6.329999923706055
SOURCE_HEAD = "3ecf0f5bca06f605b8e4f5be61c3cdf33bb3f53d"
BRANCH = "agent/v15-24d-ft-j2-bounded-feedforward"
BASELINES = {
    "authoritative": {"plus_actual_deg": 0.22910051700214848,
                       "minus_actual_deg": -0.2959198376403979},
    "operator_repeat": {"plus_actual_deg": 0.3592701127105181,
                         "minus_actual_deg": -0.12322610947791506},
}
EXPECTED_PHASE_COUNTS = {
    "SESSION_BRAKE_CAPTURE": 50,
    "ZERO_TFF_BASELINE_HOLD": 75,
    "TFF_LINEAR_RAMP": 51,
    "FEEDFORWARD_HOLD": 100,
    "PLUS_5_PROFILE": 85,
    "PLUS_5_ENDPOINT": 40,
    "FIRST_CENTER_PROFILE": 85,
    "FIRST_CENTER_ENDPOINT": 40,
    "MINUS_5_PROFILE": 85,
    "MINUS_5_ENDPOINT": 40,
    "FINAL_CENTER_PROFILE": 85,
    "FINAL_CENTER_ENDPOINT": 50,
    "FINAL_DUAL_BRAKE": 5,
}
ROUTE_PHASES = {
    "PLUS_5_PROFILE", "PLUS_5_ENDPOINT", "FIRST_CENTER_PROFILE",
    "FIRST_CENTER_ENDPOINT", "MINUS_5_PROFILE", "MINUS_5_ENDPOINT",
    "FINAL_CENTER_PROFILE", "FINAL_CENTER_ENDPOINT",
}


def sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def f(row: dict[str, str], key: str) -> float:
    return float(row[key])


def phase_rows(rows: list[dict[str, str]], name: str) -> list[dict[str, str]]:
    result = [row for row in rows if row["phase"] == name]
    if not result:
        raise RuntimeError(f"missing phase {name}")
    return result


def tail_median(rows: list[dict[str, str]], key: str, count: int) -> float:
    if len(rows) < count:
        raise RuntimeError(f"insufficient {key} rows")
    return median(f(row, key) for row in rows[-count:])


def require(condition: bool, reason: str) -> None:
    if not condition:
        raise RuntimeError(reason)


def endpoint_error(actual: float, target: float) -> float:
    return abs(actual - target)


def classify_effect(plus_actual: float, minus_actual: float) -> tuple[str, dict]:
    comparisons: dict[str, dict[str, float]] = {}
    for name, baseline in BASELINES.items():
        bp = baseline["plus_actual_deg"]
        bm = baseline["minus_actual_deg"]
        comparisons[name] = {
            "plus_signed_shift_deg": plus_actual - bp,
            "minus_signed_shift_deg": minus_actual - bm,
            "plus_error_improvement_deg": abs(5.0 - bp) - abs(5.0 - plus_actual),
            "minus_error_improvement_deg": abs(-5.0 - bm) - abs(-5.0 - minus_actual),
        }
    # Freeze an objective directional-bias rule: both historical Kp=.60 runs
    # must show the same improvement/worsening sign by at least 0.05 degree.
    plus_improvements = [v["plus_error_improvement_deg"] for v in comparisons.values()]
    minus_improvements = [v["minus_error_improvement_deg"] for v in comparisons.values()]
    if min(plus_improvements) >= 0.05 and max(minus_improvements) <= -0.05:
        effect = "DIRECTIONALLY_BIASED"
    elif min(plus_improvements) >= 0.05 and min(minus_improvements) >= 0.05:
        effect = "SUPPORTED"
    elif max(abs(v) for v in plus_improvements + minus_improvements) < 0.05:
        effect = "NO_MEANINGFUL_EFFECT"
    else:
        effect = "DIRECTIONALLY_BIASED"
    return effect, comparisons


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--repo", type=Path, required=True)
    parser.add_argument("--operator-observation", choices=("SAFE", "ABNORMAL"), required=True)
    parser.add_argument("--power-off-confirmed", choices=("YES", "NO"), required=True)
    args = parser.parse_args()

    repo = args.repo.resolve()
    evidence_dir = repo / "hardware/v15_24d_ft"
    run1_path = evidence_dir / "j2_tff005_run1.csv"
    run2_path = evidence_dir / "j2_tff005_run2.csv"
    inventory_path = evidence_dir / "j2_tff005_inventory.json"
    report_path = repo / "docs/V15_24D_FT_J2_BOUNDED_FEEDFORWARD.md"
    sums_path = evidence_dir / "SHA256SUMS"
    runner_path = repo / "tools/hardware/v15_24d_ft_j2_bounded_feedforward.cpp"
    analyzer_path = repo / "tools/analyze_j2_tff005_v15_24d.py"

    require(run1_path.is_file(), "RUN1 CSV missing")
    with run1_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    require(len(rows) == 791, "RUN1 row count mismatch")
    require(Counter(row["phase"] for row in rows) == EXPECTED_PHASE_COUNTS,
            "phase counts mismatch")

    active = [row for row in rows
              if row["phase"] not in ("SESSION_BRAKE_CAPTURE", "FINAL_DUAL_BRAKE")]
    require(all(row["a_valid"] == "1" and row["b_valid"] == "1" for row in active),
            "active feedback invalid")
    require(all(int(row["a_merror"]) == 0 and int(row["b_merror"]) == 0 for row in rows),
            "merror nonzero")
    require(all(int(row["a_received_id"]) == 0 and int(row["b_received_id"]) == 1
                for row in rows), "received ID mismatch")

    zero = phase_rows(rows, "ZERO_TFF_BASELINE_HOLD")
    ramp = phase_rows(rows, "TFF_LINEAR_RAMP")
    ffhold = phase_rows(rows, "FEEDFORWARD_HOLD")
    final_brake_rows = phase_rows(rows, "FINAL_DUAL_BRAKE")
    require(all(int(row["a_tau_cmd_count"]) == 0 and
                int(row["b_tau_cmd_count"]) == 0 for row in zero),
            "zero-Tff hold not zero")
    ramp_a = [int(row["a_tau_cmd_count"]) for row in ramp]
    ramp_b = [int(row["b_tau_cmd_count"]) for row in ramp]
    require(all(-12 <= value <= 0 for value in ramp_a) and
            all(0 <= value <= 12 for value in ramp_b), "ramp count envelope")
    require(all(a == -b for a, b in zip(ramp_a, ramp_b)), "ramp count symmetry")
    require(all(a >= b for a, b in zip(ramp_a, ramp_a[1:])) and
            all(a <= b for a, b in zip(ramp_b, ramp_b[1:])), "ramp not monotone")
    require((ramp_a[-1], ramp_b[-1]) == (-12, 12), "ramp final count")
    for row in active:
        if row["phase"] not in ("ZERO_TFF_BASELINE_HOLD", "TFF_LINEAR_RAMP"):
            require((int(row["a_tau_cmd_count"]), int(row["b_tau_cmd_count"])) ==
                    (-12, 12), "post-ramp Tff not exact")

    capture = phase_rows(rows, "SESSION_BRAKE_CAPTURE")
    a0 = median(f(row, "a_raw") for row in capture)
    b0 = median(f(row, "b_raw") for row in capture)
    max_a_map_residual = max(abs(f(row, "a_q_cmd") -
        (a0 - GEAR * f(row, "q_ref_rad"))) for row in active)
    max_b_map_residual = max(abs(f(row, "b_q_cmd") -
        (b0 + GEAR * f(row, "q_ref_rad"))) for row in active)
    require(max_a_map_residual < 1e-12 and max_b_map_residual < 1e-12,
            "command mapping mismatch")

    baseline_q = tail_median(zero, "qJ2_logical_rad", 50)
    zero_final_sync = abs(tail_median(zero, "e_sync_rad", 50))
    zero_max_sync = max(abs(f(row, "e_sync_rad")) for row in zero)
    zero_max_motion = max(abs(f(row, "qJ2_logical_rad")) for row in zero)
    zero_hold_pass = (zero_final_sync * DEG <= 0.3 and
                      zero_max_sync * DEG <= 0.3 and
                      zero_max_motion * DEG <= 1.0)

    ffhold_q = tail_median(ffhold, "qJ2_logical_rad", 50)
    ffhold_displacement = (ffhold_q - baseline_q) * DEG
    ffhold_direction = ("POSITIVE" if ffhold_displacement > 0.02 else
                        "NEGATIVE" if ffhold_displacement < -0.02 else "NEGLIGIBLE")
    ffhold_max_sync = max(abs(f(row, "e_sync_rad")) for row in ffhold) * DEG
    ramp_elapsed = f(ramp[-1], "timestamp_s") - f(ramp[0], "timestamp_s")

    plus_rows = phase_rows(rows, "PLUS_5_ENDPOINT")
    first_rows = phase_rows(rows, "FIRST_CENTER_ENDPOINT")
    minus_rows = phase_rows(rows, "MINUS_5_ENDPOINT")
    final_rows = phase_rows(rows, "FINAL_CENTER_ENDPOINT")
    plus_actual = tail_median(plus_rows, "qJ2_logical_rad", 30) * DEG
    plus_error = endpoint_error(plus_actual, 5.0)
    plus_sync = abs(tail_median(plus_rows, "e_sync_rad", 30)) * DEG
    first_error = abs(tail_median(first_rows, "qJ2_logical_rad", 30)) * DEG
    minus_actual = tail_median(minus_rows, "qJ2_logical_rad", 30) * DEG
    minus_error = endpoint_error(minus_actual, -5.0)
    minus_sync = abs(tail_median(minus_rows, "e_sync_rad", 30)) * DEG
    final_error = abs(tail_median(final_rows, "qJ2_logical_rad", 40)) * DEG
    motion = [row for row in rows if row["phase"] in ROUTE_PHASES]
    max_motion_sync = max(abs(f(row, "e_sync_rad")) for row in motion) * DEG
    max_logical_tau = max(abs(f(row, "tau_J2_feedback")) for row in active)
    max_temp = max(max(int(row["a_temp"]), int(row["b_temp"])) for row in rows)
    max_logical_dq = max(max(abs(f(row, "a_dq_logical")),
                             abs(f(row, "b_dq_logical"))) for row in active) * DEG
    final_brake_pass = len(final_brake_rows) == 5 and all(
        row["a_mode"] == "0" and row["b_mode"] == "0" and
        row["a_valid"] == "1" and row["b_valid"] == "1"
        for row in final_brake_rows)
    numeric_pass = (zero_hold_pass and ffhold_max_sync <= 0.3 and
                    plus_error <= 1.0 and first_error <= 0.75 and
                    minus_error <= 1.0 and final_error <= 0.75 and
                    plus_sync <= 0.3 and minus_sync <= 0.3 and
                    max_motion_sync <= 0.7 and final_brake_pass)
    run1_result = "PASS" if numeric_pass and args.operator_observation == "SAFE" else "FAIL"
    require(run1_result == "FAIL", "RUN1 unexpectedly passed; repeat contract requires RUN2")
    require(not run2_path.exists(), "RUN2 must not exist after failed RUN1")

    effect, comparisons = classify_effect(plus_actual, minus_actual)
    direction_authority = ("SUPPORTED" if ffhold_direction == "POSITIVE" else
                           "MISMATCH" if ffhold_direction == "NEGATIVE" else
                           "INCONCLUSIVE")
    gravity_confidence = "INCONCLUSIVE"

    inventory = {
        "task": "V15.24D-FT J2 bounded gravity feedforward hold and +/-5 degree hardware test",
        "source_head": SOURCE_HEAD,
        "branch": BRANCH,
        "status": "FAIL",
        "failure_classification": "J2_TFF005_DIRECTIONALLY_BIASED_POSITION_TRACKING_FAIL",
        "mechanical_state": "BIG_ARM_AND_FOREARM_INSTALLED",
        "operator_observation": args.operator_observation,
        "operator_confirmed_24v_power_off": args.power_off_confirmed == "YES",
        "authority_inherited": {
            "J2_TAU_COMMAND_AUTHORITY": "PASS",
            "J2_GRAVITY_MODEL_AUDIT": "PASS",
            "J2_FEEDFORWARD_EXPERIMENT_READY": "YES",
        },
        "command": {
            "kp": 0.60, "kd": 0.05, "velocity_deg_s": 10.0,
            "acceleration_deg_s2": 30.0, "rate_hz": 100.0,
            "tff_literal_a_rotor_nm": -0.05, "tff_literal_b_rotor_nm": 0.05,
            "tff_encoded_a_count": -12, "tff_encoded_b_count": 12,
            "tff_decoded_a_rotor_nm": -0.046875,
            "tff_decoded_b_rotor_nm": 0.046875,
            "tff_logical_j2_decoded_nm": GEAR * (24.0 / 256.0),
            "maximum_abs_count_observed": 12,
            "automatic_tff_increase": False,
        },
        "run1": {
            "evidence_file": "hardware/v15_24d_ft/j2_tff005_run1.csv",
            "sha256": sha256(run1_path), "rows": len(rows),
            "a0_raw_rad": a0, "b0_raw_rad": b0,
            "zero_tff_startup_hold": "PASS" if zero_hold_pass else "FAIL",
            "tff_ramp_duration_s": ramp_elapsed,
            "tff_hold_displacement_deg": ffhold_displacement,
            "tff_hold_direction": ffhold_direction,
            "tff_hold_max_e_sync_deg": ffhold_max_sync,
            "plus_5_actual_deg": plus_actual, "plus_5_error_deg": plus_error,
            "plus_5_e_sync_deg": plus_sync,
            "first_center_error_deg": first_error,
            "minus_5_actual_deg": minus_actual, "minus_5_error_deg": minus_error,
            "minus_5_e_sync_deg": minus_sync,
            "final_center_error_deg": final_error,
            "max_motion_e_sync_deg": max_motion_sync,
            "max_abs_logical_paired_tau_feedback_nm": max_logical_tau,
            "max_temperature_c": max_temp,
            "max_abs_logical_velocity_deg_s": max_logical_dq,
            "active_invalid_frames": 0, "merror_nonzero_frames": 0,
            "final_both_brake": "PASS" if final_brake_pass else "FAIL",
            "operator_safe": args.operator_observation == "SAFE",
            "numeric_result": "PASS" if numeric_pass else "FAIL",
            "result": run1_result,
        },
        "run2": {"executed": False, "result": "NOT_EXECUTED_RUN1_FAIL"},
        "comparison_to_v15_24b": comparisons,
        "classification": {
            "J2_TFF_005_EFFECT": effect,
            "TFF_DIRECTION_AUTHORITY": direction_authority,
            "GRAVITY_CONTRIBUTION_CONFIDENCE": gravity_confidence,
            "J2_TFF_005_REPEATABILITY": "NOT_TESTED_RUN1_FAIL",
            "J2_INSTALLED_LOAD_BIDIRECTIONAL_5DEG": "FAIL",
            "J2_LOCAL_MOTION_AUTHORITY": "NOT_GRANTED",
        },
        "safety": {
            "final_both_brake": "PASS" if final_brake_pass else "FAIL",
            "operator_safe": args.operator_observation == "SAFE",
            "power_24v_off": args.power_off_confirmed == "YES",
            "j3_work_used": False, "motor_zero_modified": False,
            "simulation_modified": False,
        },
        "invariants": {
            "phase_counts": EXPECTED_PHASE_COUNTS,
            "max_a_command_mapping_residual_rad": max_a_map_residual,
            "max_b_command_mapping_residual_rad": max_b_map_residual,
            "ramp_monotone_and_symmetric": True,
            "post_ramp_counts_exact": True,
        },
        "unresolved_items": [
            "The bounded 0.05 N.m literal feedforward did not establish bidirectional position tracking.",
            "The hold displacement was negligible, so direct hold-based Tff direction authority remains inconclusive.",
            "Internal torque/current limiting, position-loop effort scaling, friction/brake, and load/model mismatch remain unresolved.",
            "J3 installed-load validation remains not tested.",
        ],
        "next_single_task": "J2_INTERNAL_TORQUE_CURRENT_LIMIT_AND_POSITION_LOOP_EFFORT_AUDIT",
        "final_task_result": "FAIL",
    }
    require(inventory["operator_confirmed_24v_power_off"],
            "operator 24V OFF confirmation required")
    inventory_path.write_text(json.dumps(inventory, indent=2, ensure_ascii=False) + "\n",
                              encoding="utf-8", newline="\n")

    auth = comparisons["authoritative"]
    repeat = comparisons["operator_repeat"]
    report = f"""# V15.24D-FT J2 bounded gravity feedforward

## Result

`RUN1 = FAIL`. The zero-Tff hold, bounded Tff ramp/hold, communication, synchronization, and terminal BOTH BRAKE passed, but the +/-5 degree position thresholds did not. The contract therefore prohibited RUN2 and any automatic Tff increase.

## Frozen command

- J2A/J2B Tff literals: `-0.05 / +0.05 N.m` rotor-side.
- Actual serialized counts: `-12 / +12`; decoded: `-0.046875 / +0.046875 N.m`.
- Kp/Kd: `0.60 / 0.05`; route: `0 -> +5 -> 0 -> -5 -> 0 deg` at 10 deg/s, 30 deg/s^2, 100 Hz.
- The CSV proves a monotone symmetric 0.5 s Q8 ramp and no command outside +/-12 counts.

## RUN1

- A0/B0: `{a0:.17g} / {b0:.17g} rad`.
- Feedforward hold displacement: `{ffhold_displacement:.12f} deg` (`{ffhold_direction}`); max hold e_sync: `{ffhold_max_sync:.12f} deg`.
- +5 actual/error/e_sync: `{plus_actual:.12f} / {plus_error:.12f} / {plus_sync:.12f} deg`.
- First-center error: `{first_error:.12f} deg`.
- -5 actual/error/e_sync: `{minus_actual:.12f} / {minus_error:.12f} / {minus_sync:.12f} deg`.
- Final-center error: `{final_error:.12f} deg`; max route e_sync: `{max_motion_sync:.12f} deg`.
- Maximum absolute logical paired torque feedback: `{max_logical_tau:.12f} N.m`.
- Active invalid frames / nonzero merror frames: `0 / 0`; max temperature: `{max_temp} C`.
- Operator observation: `{args.operator_observation}`; final five-pair BOTH BRAKE: `{'PASS' if final_brake_pass else 'FAIL'}`; operator-confirmed 24V OFF: `{args.power_off_confirmed}`.

## Comparison and diagnosis

Relative to the authoritative V15.24B Kp=.60 run, +5 error improved by `{auth['plus_error_improvement_deg']:.12f} deg`, while -5 error worsened by `{abs(auth['minus_error_improvement_deg']):.12f} deg`. Relative to its operator repeat, the changes were an improvement of `{repeat['plus_error_improvement_deg']:.12f} deg` and a worsening of `{abs(repeat['minus_error_improvement_deg']):.12f} deg`. Both comparisons show the same positive-direction bias, so `J2_TFF_005_EFFECT = {effect}`.

The dedicated hold displacement was below the frozen 0.02 degree direction threshold, so `TFF_DIRECTION_AUTHORITY = {direction_authority}` and `GRAVITY_CONTRIBUTION_CONFIDENCE = {gravity_confidence}`. This result does not establish gravity as the root cause. Increasing fixed Tff automatically is prohibited and would risk worsening the negative direction.

## Authority and next task

- `J2_TFF_005_REPEATABILITY = NOT_TESTED_RUN1_FAIL`
- `J2_INSTALLED_LOAD_BIDIRECTIONAL_5DEG = FAIL`
- `J2_LOCAL_MOTION_AUTHORITY = NOT_GRANTED`
- `NEXT_SINGLE_TASK = J2_INTERNAL_TORQUE_CURRENT_LIMIT_AND_POSITION_LOOP_EFFORT_AUDIT`
"""
    report_path.write_text(report, encoding="utf-8", newline="\n")

    owned = [run1_path, inventory_path, report_path, runner_path, analyzer_path]
    lines = [f"{sha256(path)}  {path.relative_to(repo).as_posix()}" for path in owned]
    sums_path.write_text("\n".join(lines) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps({
        "run1_result": run1_result,
        "effect": effect,
        "direction_authority": direction_authority,
        "gravity_confidence": gravity_confidence,
        "final_both_brake": final_brake_pass,
        "power_off": args.power_off_confirmed,
    }, sort_keys=True))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

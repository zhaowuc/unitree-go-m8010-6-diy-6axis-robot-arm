#!/usr/bin/env python3
"""Independent offline tests for validate_v15_31b_ft_evidence.py."""

from __future__ import annotations

import csv
import hashlib
import importlib.util
import json
import math
import subprocess
from pathlib import Path
from typing import Any, Callable

import pytest


MODULE_PATH = Path(__file__).with_name("validate_v15_31b_ft_evidence.py")
SPEC = importlib.util.spec_from_file_location("validate_v15_31b_ft_evidence", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
validator = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(validator)

REPO_ROOT = Path(__file__).resolve().parents[1]
UTC = "2026-08-31T00:00:00Z"
SESSION_ID = "persistent:session:j2session:abc:goauxsession:def"
STATE_INSTANCE_ID = "1" * 32


def _head_commit() -> str:
    return subprocess.check_output(
        ["git", "-C", str(REPO_ROOT), "rev-parse", "HEAD"],
        text=True,
        encoding="utf-8",
    ).strip()


def _write_json(path: Path, value: Any) -> None:
    path.write_text(
        json.dumps(value, ensure_ascii=False, indent=2, sort_keys=False) + "\n",
        encoding="utf-8",
    )


def _write_csv(path: Path, fields: tuple[str, ...], rows: list[dict[str, Any]]) -> None:
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields, lineterminator="\n")
        writer.writeheader()
        writer.writerows(rows)


def _rewrite_manifest(bundle: Path) -> None:
    lines = []
    for name in validator.HASHED_ARTIFACTS:
        digest = hashlib.sha256((bundle / name).read_bytes()).hexdigest()
        lines.append(f"{digest}  {name}")
    (bundle / "SHA256SUMS").write_text("\n".join(lines) + "\n", encoding="ascii")


def _read_csv(path: Path) -> tuple[list[str], list[dict[str, str]]]:
    with path.open("r", encoding="utf-8", newline="") as stream:
        reader = csv.DictReader(stream)
        assert reader.fieldnames is not None
        return list(reader.fieldnames), list(reader)


def _document_sha256(value: Any) -> str:
    payload = json.dumps(
        value,
        ensure_ascii=True,
        allow_nan=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("ascii")
    return hashlib.sha256(payload).hexdigest()


def _binding_document() -> dict[str, str]:
    return {
        "envelope_id": "v15-31b-empirical-" + "3" * 19,
        "envelope_sha256": "4" * 64,
        "session_id": SESSION_ID,
        "state_instance_id": STATE_INSTANCE_ID,
        "anchor_sha256": _current_anchor_sha256(),
    }


def _motion_trace(
    *,
    joint: str,
    target_deg: float,
    trajectory_sha256: str,
    source_start_ns: int,
    sequence_start: int,
    center_observation: bool = False,
) -> list[dict[str, Any]]:
    joint_index = validator.JOINT_NAMES.index(joint)
    position_rad = [0.0] * 6
    position_rad[joint_index] = math.radians(target_deg)
    moving_motors = set(validator.MOTOR_BY_JOINT[joint])
    rows: list[dict[str, Any]] = []
    for offset in range(6):
        source_ns = source_start_ns + offset * 100_000_000
        modes = {
            motor: (
                "hold"
                if center_observation or motor not in moving_motors
                else "position"
            )
            for motor in validator.MOTOR_NAMES
        }
        rows.append({
            "hardware_state_sequence": sequence_start + offset,
            "hardware_state_source_monotonic_ns": source_ns,
            "receipt_monotonic_ns": source_ns + 1_000_000,
            "hardware_state_sha256": f"{sequence_start + offset:064x}",
            "joint": joint,
            "target_rad": math.radians(target_deg),
            "actual_rad": math.radians(target_deg),
            "error_deg": 0.0,
            "velocity_deg_s": 0.0,
            "position_rad": list(position_rad),
            "controller_mode_by_motor": modes,
            "moving_joint_mask": (
                [False] * 6
                if center_observation
                else [name == joint for name in validator.JOINT_NAMES]
            ),
            "trajectory_sha256": trajectory_sha256,
        })
    return rows


def _modify_csv(
    path: Path,
    predicate: Callable[[dict[str, str]], bool],
    updates: dict[str, str],
) -> None:
    fields, rows = _read_csv(path)
    matches = [row for row in rows if predicate(row)]
    assert len(matches) == 1
    matches[0].update(updates)
    _write_csv(path, tuple(fields), rows)


def _pass_status() -> dict[str, str]:
    return {"status": "PASS"}


def _blocked_status() -> dict[str, str]:
    return {"status": "BLOCKED_POWER_OFF", "reason": "POWER_OFF"}


def _power_on_document() -> dict[str, Any]:
    per_motor: dict[str, Any] = {}
    for name in validator.MOTOR_NAMES:
        item: dict[str, Any] = {
            "online": True,
            "max_feedback_age_ms": 10.0,
            "raw_position_rad": {"min": 0.0, "max": 0.0},
            "logical_position_rad": {"min": 0.0, "max": 0.0},
            "max_abs_velocity_rad_s": 0.0,
            "max_temperature_c": 30.0,
            "abnormal_velocity_detected": False,
            "max_abs_merror": 0,
            "communication_interruptions": 0,
            "observed_modes": ["disabled" if name == "J6" else "brake"],
        }
        if name == "J6":
            item.update({
                "tau_cmd_rotor_nm": None,
                "tau_feedback_rotor_nm": None,
                "tau_joint_estimated_nm": None,
            })
        else:
            item.update({
                "tau_feedback_rotor_nm": {"min": 0.0, "max": 0.0},
                "tau_joint_estimated_nm": {"min": 0.0, "max": 0.0},
                "max_abs_tau_cmd_rotor_nm": 0.0,
            })
        per_motor[name] = item
    samples = []
    modes = {
        name: "disabled" if name == "J6" else "brake"
        for name in validator.MOTOR_NAMES
    }
    for index in range(101):
        sample_motors = {}
        for name in validator.MOTOR_NAMES:
            sample_motors[name] = {
                "raw_position_rad": 0.0,
                "q_joint_rad": 0.0,
                "dq_joint_rad_s": 0.0,
                "age_ms": 10.0,
                "temperature_c": 30.0,
                "merror": 0,
                "communication_ok": True,
                "fresh": True,
                "tau_cmd_rotor_nm": None if name == "J6" else 0.0,
                "tau_feedback_rotor_nm": None if name == "J6" else 0.0,
                "tau_joint_estimated_nm": None if name == "J6" else 0.0,
            }
        samples.append({
            "schema": validator.HARDWARE_STATE_SCHEMA,
            "sequence": index + 1,
            "source_monotonic_ns": 1_000_000_000 + index * 100_000_000,
            "session_id": SESSION_ID,
            "state_instance_id": STATE_INSTANCE_ID,
            "position_rad": [0.0] * 6,
            "velocity_rad_s": [0.0] * 6,
            "j2_e_sync_rad": 0.0,
            "j2_sync_fault": False,
            "controller_mode_by_motor": dict(modes),
            "per_motor": sample_motors,
        })
    return {
        "schema": validator.POWER_ON_SCHEMA,
        "schema_version": "V15.31B-power-on-readonly-v1",
        "result": "PASS",
        "duration_s": 10.0,
        "valid_frame_count": len(samples),
        "model_sha256": validator.PRODUCTION_MODEL_SHA256,
        "session_id": SESSION_ID,
        "state_instance_id": STATE_INSTANCE_ID,
        "per_motor": per_motor,
        "j2": {
            "logical_position_rad": {"min": 0.0, "max": 0.0},
            "max_abs_e_sync_deg": 0.0,
            "j2a_logical_torque_contribution_nm": {"min": 0.0, "max": 0.0},
            "j2b_logical_torque_contribution_nm": {"min": 0.0, "max": 0.0},
        },
        "safety": {
            "position_enabled": False,
            "motor_internal_zero_modified": False,
            "flash_written": False,
            "eeprom_written": False,
            "active_command_count": 0,
        },
        "samples": samples,
        "operator_confirmation": {
            "confirmed_at_utc": UTC,
            "mechanism_stationary": True,
            "model_pose_aligned": True,
            "support_reliable": True,
            "not_at_mechanical_limit": True,
            "operator_stop_ready": True,
        },
        "writes": {
            "motor_internal_zero_modified": False,
            "rid_written": False,
            "flash_or_eeprom_written": False,
        },
    }


def _anchor_document(power_sha256: str) -> dict[str, Any]:
    anchor = {
        "schema": validator.GRAVITY_ANCHOR_SCHEMA,
        "model_sha256": validator.PRODUCTION_MODEL_SHA256,
        "session_id": SESSION_ID,
        "state_instance_id": STATE_INSTANCE_ID,
        "created_utc": UTC,
        "gear_ratio": validator.GO_GEAR_RATIO,
        "motor_direction_sign": dict(validator.FROZEN_MOTOR_SIGNS),
        "motor_raw_reference_rad": {name: 0.0 for name in validator.MOTOR_NAMES},
        "motor_encoder_branch": {name: 0 for name in validator.MOTOR_NAMES},
        "logical_joint_reference_rad": [0.0] * 6,
        "model_absolute_joint_rad": [0.0] * 6,
    }
    anchor_sha256 = hashlib.sha256(
        (json.dumps(anchor, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    ).hexdigest()
    return {
        "schema_version": "V15.31B-anchor-validation-v1",
        "result": "PASS",
        "source_power_on_readonly_sha256": power_sha256,
        "anchor_sha256": anchor_sha256,
        "runtime_anchor_sha256": anchor_sha256,
        "anchor_field_count": len(anchor),
        "anchor": anchor,
        "checks": {
            "readonly_sample_match": True,
            "session_match": True,
            "state_instance_match": True,
            "model_hash_match": True,
            "motor_internal_zero_modified": False,
            "flash_written": False,
            "eeprom_written": False,
            "mujoco_model_modified": False,
        },
    }


def _current_anchor_sha256() -> str:
    return _anchor_document("0" * 64)["runtime_anchor_sha256"]


def _gravity_readonly_document(anchor_sha256: str) -> dict[str, Any]:
    gravity = [validator.GO_GEAR_RATIO, 2.0 * validator.GO_GEAR_RATIO,
               validator.GO_GEAR_RATIO, 0.0, 0.5 * validator.GO_GEAR_RATIO, 0.0]
    samples = []
    for index in range(101):
        status_ns = 20_000_000_000 + index * 100_000_000
        samples.append({
            "gravity_status_sequence": index + 1,
            "gravity_status_source_monotonic_ns": status_ns,
            "hardware_state_sequence": index + 1,
            "hardware_state_source_monotonic_ns": status_ns - 10_000_000,
            "q_actual_rad": [0.0] * 6,
            "model_q_rad": [0.0] * 6,
            "gravity_joint_nm": gravity,
            "predicted_rotor_nm": {
                "J1": 1.0,
                "J2A": -1.0,
                "J2B": 1.0,
                "J3": 1.0,
                "J4": 0.0,
                "J5": 0.5,
            },
            "max_abs_velocity_rad_s": 0.0,
            "gravity_scale": 0.0,
            "gravity_scale_target": 0.0,
            "feedforward_nm": [0.0] * 6,
        })
    direction_probes = [
        {
            "joint": joint,
            "delta_q_rad": math.radians(0.25),
            "minus_gravity_joint_nm": list(gravity),
            "center_gravity_joint_nm": list(gravity),
            "plus_gravity_joint_nm": list(gravity),
            "maximum_second_difference_nm": 0.0,
            "qvel_rad_s": [0.0] * 6,
            "qacc_rad_s2": [0.0] * 6,
        }
        for joint in validator.JOINT_NAMES
    ]
    return {
        "schema": validator.GRAVITY_READONLY_SCHEMA,
        "schema_version": "V15.31B-gravity-readonly-v1",
        "result": "PASS",
        "duration_s": 10.0,
        "valid_sample_count": len(samples),
        "session_id": SESSION_ID,
        "state_instance_id": STATE_INSTANCE_ID,
        "anchor_sha256": anchor_sha256,
        "model_sha256": validator.PRODUCTION_MODEL_SHA256,
        "gravity_config_sha256": validator.GRAVITY_CONFIG_SHA256,
        "gravity_source_instance_id": "2" * 32,
        "hardware_tff_enabled": False,
        "tff_transmitted": False,
        "checks": {
            "finite": True,
            "continuous": True,
            "direction_reasonable": True,
            "direction_independent_of_motion": True,
            "j2_split_50_50": True,
            "j2a_sign_correct": True,
            "j2b_sign_correct": True,
        },
        "continuity": {
            "maximum_gravity_status_gap_ms": 100.0,
            "maximum_hardware_state_gap_ms": 100.0,
            "maximum_pose_step_rad": 0.0,
            "maximum_abs_velocity_rad_s": 0.0,
            "maximum_model_comparison_error_nm": 0.0,
            "maximum_direction_probe_second_difference_nm": 0.0,
        },
        "safety": {
            "collector_command_publishers": 0,
            "collector_motor_transports": 0,
            "active_command_count_observed": 0,
            "motor_internal_zero_modified": False,
            "flash_written": False,
            "eeprom_written": False,
            "mujoco_model_modified": False,
        },
        "direction_probes": direction_probes,
        "samples": samples,
    }


def _gravity_scale_rows() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    monotonic_ns = 30_000_000_000
    previous = 0.0
    for level in validator.GRAVITY_LEVELS:
        if level > 0.0:
            for tenth in range(20):
                elapsed = tenth / 10.0
                applied = previous + (level - previous) * elapsed / 2.0
                rows.append(_gravity_scale_row(
                    monotonic_ns, len(rows) + 1, level, applied, "RAMP", elapsed
                ))
                monotonic_ns += 100_000_000
        for tenth in range(51):
            elapsed = tenth / 10.0
            rows.append(_gravity_scale_row(
                monotonic_ns, len(rows) + 1, level, level, "HOLD", elapsed
            ))
            monotonic_ns += 100_000_000
        if level < validator.GRAVITY_LEVELS[-1]:
            rows.append(_gravity_scale_row(
                monotonic_ns,
                len(rows) + 1,
                level,
                level,
                "AWAIT_INTERSTAGE_CONFIRMATION",
                0.0,
            ))
            monotonic_ns += 100_000_000
        previous = level
    for index, row in enumerate(rows):
        current_sequence = index + 100
        current_source_ns = int(row["monotonic_ns"]) - 100_000
        hardware_source_ns = int(row["monotonic_ns"]) - 200_000
        worker_source_ns = int(row["monotonic_ns"]) - 300_000
        current_node = (
            0.0,
            0.1 * float(row["gravity_scale_applied"]),
            0.0,
            0.0,
            0.0,
            0.0,
        )
        if index == 0:
            matched_sequence = 99
            matched_source_ns = worker_source_ns - 50_000_000
            matched_node = (0.0,) * 6
        else:
            previous_row = rows[index - 1]
            matched_sequence = int(previous_row["gravity_status_sequence"])
            matched_source_ns = int(
                previous_row["gravity_status_source_monotonic_ns"]
            )
            matched_node = tuple(json.loads(
                previous_row["node_feedforward_nm_json"]
            ))
        matched_expected = (
            matched_node[0],
            -matched_node[1],
            matched_node[1],
            matched_node[2],
            matched_node[3],
            matched_node[4],
        )
        row.update({
            "gravity_status_sequence": current_sequence,
            "gravity_status_source_monotonic_ns": current_source_ns,
            "hardware_state_sequence": current_sequence,
            "hardware_state_source_monotonic_ns": hardware_source_ns,
            "node_feedforward_nm_json": json.dumps(
                current_node, separators=(",", ":")
            ),
            "worker_feedforward_rotor_nm_json": json.dumps(
                matched_expected, separators=(",", ":")
            ),
            "matched_node_feedforward_nm_json": json.dumps(
                matched_node, separators=(",", ":")
            ),
            "matched_expected_worker_feedforward_rotor_nm_json": json.dumps(
                matched_expected, separators=(",", ":")
            ),
            "worker_echo_source_monotonic_ns_json": json.dumps(
                [worker_source_ns] * 6, separators=(",", ":")
            ),
            "worker_echo_match_gravity_status_sequence": matched_sequence,
            "worker_echo_match_gravity_status_source_monotonic_ns": (
                matched_source_ns
            ),
            "worker_echo_lag_ms": (
                worker_source_ns - matched_source_ns
            ) / 1.0e6,
        })
    return rows


def _gravity_scale_row(
    monotonic_ns: int,
    sequence: int,
    target: float,
    applied: float,
    phase: str,
    elapsed: float,
) -> dict[str, Any]:
    return {
        "timestamp_utc": UTC,
        "monotonic_ns": monotonic_ns,
        "observer_receipt_monotonic_ns": monotonic_ns,
        "stage": f"{int(target * 100)}%",
        "phase": phase,
        "gravity_scale_target": target,
        "gravity_scale_applied": applied,
        "phase_elapsed_s": elapsed,
        "position_error_deg": 0.1,
        "tau_feedback_rotor_nm": 0.2,
        "pd_contribution_rotor_nm": 0.1,
        "gravity_ff_contribution_rotor_nm": 0.1,
        "total_command_rotor_nm": 0.2,
        "temperature_c": 31.0,
        "temperature_slope_c_per_min": 0.1,
        "j2_e_sync_deg": 0.1,
        "saturation_observed": "false",
        "merror": 0,
        "communication_ok": "true",
        "operator_stop_available": "true",
        "status": "PASS",
        "reason": "",
        "envelope_id": "v15-31b-empirical-" + "3" * 19,
        "envelope_sha256": "4" * 64,
        "session_id": SESSION_ID,
        "state_instance_id": STATE_INSTANCE_ID,
        "anchor_sha256": _current_anchor_sha256(),
        "gravity_source_instance_id": "2" * 32,
        "gravity_status_sequence": sequence,
        "gravity_status_source_monotonic_ns": monotonic_ns - 1_000_000,
        "hardware_state_sequence": sequence,
        "hardware_state_source_monotonic_ns": monotonic_ns - 2_000_000,
        "node_feedforward_nm_json": "[0,0.1,0,0,0,0]",
        "worker_feedforward_rotor_nm_json": "[0,-0.1,0.1,0,0,0]",
        "matched_node_feedforward_nm_json": "[0,0.1,0,0,0,0]",
        "matched_expected_worker_feedforward_rotor_nm_json": (
            "[0,-0.1,0.1,0,0,0]"
        ),
        "worker_echo_source_monotonic_ns_json": json.dumps(
            [monotonic_ns - 2_000_000] * 6,
            separators=(",", ":"),
        ),
        "worker_echo_match_gravity_status_sequence": sequence,
        "worker_echo_match_gravity_status_source_monotonic_ns": (
            monotonic_ns - 52_000_000
        ),
        "worker_echo_lag_ms": 50.0,
        "gravity_status_sha256": "5" * 64,
        "hardware_state_sha256": "6" * 64,
    }


def _position_rows(*, actual_center_deg=0.0, endpoint_error_deg=0.0) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    targets = (0.0, 5.0, 0.0, -5.0, 0.0)
    row_index = 0
    for joint in validator.POSITION_ORDER:
        for phase, target in zip(validator.POSITION_PHASES, targets):
            row_index += 1
            center_observation = phase == "CENTER_START"
            source_start_ns = 200_000_000_000 + row_index * 1_000_000_000
            sequence_start = 1_000 + row_index * 10
            trajectory_sha = (
                "CENTER_OBSERVATION" if center_observation else f"{row_index:064x}"
            )
            dwell_samples = _motion_trace(
                joint=joint,
                target_deg=target,
                trajectory_sha256=trajectory_sha,
                source_start_ns=source_start_ns,
                sequence_start=sequence_start,
                center_observation=center_observation,
            )
            actual = actual_center_deg if target == 0.0 else target - math.copysign(endpoint_error_deg, target)
            for sample in dwell_samples:
                sample["actual_rad"] = math.radians(actual)
                sample["position_rad"][validator.JOINT_NAMES.index(joint)] = math.radians(actual)
                sample["error_deg"] = target - actual
            dwell_document = {
                "schema": "V15.31B-endpoint-dwell-trace-v1",
                "samples": dwell_samples,
            }
            execution_document = {
                "schema": "V15.31B-segment-execution-trace-v1",
                "samples": [] if center_observation else dwell_samples,
            }
            top = dwell_samples[-1]
            binding = _binding_document()
            rows.append({
                "timestamp_utc": UTC,
                "monotonic_ns": top["hardware_state_source_monotonic_ns"],
                "receipt_monotonic_ns": top["receipt_monotonic_ns"],
                "test_id": f"{joint}-BASELINE",
                "joint": joint,
                "revision": "BASELINE",
                "phase": phase,
                "target_deg": target,
                "actual_deg": actual,
                "error_deg": target - actual,
                "endpoint_dwell_s": 0.5,
                "j2_e_sync_deg": 0.1,
                "temperature_c": 32.0,
                "merror": 0,
                "communication_ok": "true",
                "controller_mode": "/".join(
                    "hold" if center_observation else "position"
                    for _motor in validator.MOTOR_BY_JOINT[joint]
                ),
                "status": "PASS",
                "reason": "",
                **binding,
                "trajectory_sha256": trajectory_sha,
                "trajectory_duration_ns": (
                    "CENTER_OBSERVATION" if center_observation else 2_000_000_000
                ),
                "trajectory_execute_at_monotonic_ns": (
                    "CENTER_OBSERVATION"
                    if center_observation else source_start_ns - 500_000
                ),
                "plan_token_id": (
                    "CENTER_OBSERVATION"
                    if center_observation else f"{row_index + 1000:064x}"
                ),
                "plan_recipe_sha256": (
                    "CENTER_OBSERVATION"
                    if center_observation else f"{row_index + 2000:064x}"
                ),
                "collision_proof_sha256": (
                    "CENTER_OBSERVATION"
                    if center_observation else f"{row_index + 3000:064x}"
                ),
                "hardware_state_sha256": top["hardware_state_sha256"],
                "related_domain": validator.DOMAIN_BY_JOINT[joint],
                "endpoint_dwell_trace_json": json.dumps(
                    dwell_document, separators=(",", ":")
                ),
                "endpoint_dwell_trace_sha256": _document_sha256(dwell_document),
                "segment_execution_trace_json": json.dumps(
                    execution_document, separators=(",", ":")
                ),
                "segment_execution_trace_sha256": _document_sha256(
                    execution_document
                ),
                "hardware_state_sequence": top["hardware_state_sequence"],
                "hardware_state_source_monotonic_ns": top[
                    "hardware_state_source_monotonic_ns"
                ],
                "gui_command_sequence": (
                    "CENTER_OBSERVATION" if center_observation else row_index
                ),
                "gui_command_source_instance_id": (
                    "CENTER_OBSERVATION" if center_observation else "7" * 32
                ),
                "gui_command_source_monotonic_ns": (
                    "CENTER_OBSERVATION"
                    if center_observation else source_start_ns - 1_000_000
                ),
                "actual_center_start_deg": actual_center_deg,
                "actual_displacement_from_center_deg": actual - actual_center_deg,
                "minimum_required_actual_displacement_deg": (
                    4.8 if phase in {"PLUS_5", "MINUS_5"} else 0.0
                ),
                "precision_contract_id": validator.POSITION_PRECISION_CONTRACT_ID,
                "endpoint_error_limit_deg": 0.1,
                "nominal_command_displacement_deg": 0.0 if center_observation else 5.0,
            })
    return rows


def _j2_comparison_rows() -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    frozen_pose = [0.0] * 6
    frozen_initial_sha = "8" * 64
    frozen_document = {
        "target_j2_rad": 0.0,
        "pose_rad": frozen_pose,
        "initial_hardware_state_sha256": frozen_initial_sha,
    }
    frozen_sha = _document_sha256(frozen_document)
    for condition in ("WITHOUT_FF", "WITH_FF"):
        monotonic_ns = (
            20_000_000_000 if condition == "WITHOUT_FF"
            else 100_000_000_000
        )
        sequence = 0 if condition == "WITHOUT_FF" else 500
        for tenth in range(6):
            elapsed = tenth / 10.0
            with_ff = condition == "WITH_FF"
            position_error_deg = 0.1 if with_ff else 0.2
            actual_pose = [0.0] * 6
            actual_pose[1] = -math.radians(position_error_deg)
            sequence += 1
            rows.append({
                "timestamp_utc": UTC,
                "monotonic_ns": monotonic_ns,
                "receipt_monotonic_ns": monotonic_ns + 1_000_000,
                "condition": condition,
                "window_elapsed_s": elapsed,
                "position_error_deg": position_error_deg,
                "j2_e_sync_deg": 0.1,
                "j2a_tau_feedback_rotor_nm": 0.2 if with_ff else 0.4,
                "j2b_tau_feedback_rotor_nm": 0.2 if with_ff else 0.4,
                "j2a_pd_rotor_nm": 0.1 if with_ff else 0.2,
                "j2b_pd_rotor_nm": 0.1 if with_ff else 0.2,
                "j2a_gravity_ff_rotor_nm": 0.1 if with_ff else 0.0,
                "j2b_gravity_ff_rotor_nm": 0.1 if with_ff else 0.0,
                "j2a_total_command_rotor_nm": 0.2,
                "j2b_total_command_rotor_nm": 0.2,
                "j2a_temperature_c": 33.0,
                "j2b_temperature_c": 33.0,
                "saturation_observed": "false" if with_ff else "true",
                "internal_opposition_detected": "false",
                "status": "PASS",
                "reason": "",
                **_binding_document(),
                "hardware_state_sha256": f"{sequence:064x}",
                "target_j2_rad": 0.0,
                "frozen_pose_rad_json": json.dumps(
                    frozen_pose, separators=(",", ":")
                ),
                "frozen_pose_sha256": frozen_sha,
                "frozen_initial_hardware_state_sha256": frozen_initial_sha,
                "actual_pose_rad_json": json.dumps(
                    actual_pose, separators=(",", ":")
                ),
                "maximum_pose_error_deg": position_error_deg,
                "hardware_state_sequence": sequence,
                "hardware_state_source_monotonic_ns": monotonic_ns,
            })
            monotonic_ns += 100_000_000
    return rows


def _final_dwell_trace(
    target_deg: list[float], source_start_ns: int, sequence_start: int
) -> list[dict[str, Any]]:
    position_rad = [math.radians(value) for value in target_deg]
    return [
        {
            "hardware_state_sequence": sequence_start + offset,
            "hardware_state_source_monotonic_ns": (
                source_start_ns + offset * 100_000_000
            ),
            "receipt_monotonic_ns": (
                source_start_ns + offset * 100_000_000 + 1_000_000
            ),
            "hardware_state_sha256": f"{sequence_start + offset:064x}",
            "position_rad": list(position_rad),
            "velocity_rad_s": [0.0] * 6,
            "error_deg_by_joint": {
                joint: 0.0 for joint in validator.JOINT_NAMES
            },
            "controller_mode_by_motor": {
                motor: "hold" for motor in validator.MOTOR_NAMES
            },
        }
        for offset in range(6)
    ]


def _post_segment(
    *,
    joint: str,
    target_deg: float,
    recipe_sha256: str,
    source_start_ns: int,
    sequence_start: int,
    gui_sequence: int,
) -> dict[str, Any]:
    trajectory_sha = f"{sequence_start + 10_000:064x}"
    samples = _motion_trace(
        joint=joint,
        target_deg=target_deg,
        trajectory_sha256=trajectory_sha,
        source_start_ns=source_start_ns,
        sequence_start=sequence_start,
    )
    execution_document = {
        "schema": "V15.31B-segment-execution-trace-v1",
        "samples": samples,
    }
    endpoint_document = {
        "schema": "V15.31B-endpoint-dwell-trace-v1",
        "samples": samples,
    }
    return {
        "joint": joint,
        "trajectory_sha256": trajectory_sha,
        "trajectory_duration_ns": 2_000_000_000,
        "trajectory_execute_at_monotonic_ns": source_start_ns - 500_000,
        "plan_token_id": f"{sequence_start + 20_000:064x}",
        "recipe_sha256": recipe_sha256,
        "collision_proof_sha256": f"{sequence_start + 30_000:064x}",
        "target_rad": math.radians(target_deg),
        "actual_rad": math.radians(target_deg),
        "endpoint_error_deg": 0.0,
        "endpoint_dwell_s": 0.5,
        "hardware_state_sha256": samples[-1]["hardware_state_sha256"],
        "gui_command_sequence": gui_sequence,
        "gui_command_source_instance_id": "9" * 32,
        "gui_command_source_monotonic_ns": source_start_ns - 1_000_000,
        "execution_trace": samples,
        "execution_trace_sha256": _document_sha256(execution_document),
        "endpoint_dwell_trace": samples,
        "endpoint_dwell_trace_sha256": _document_sha256(endpoint_document),
    }


def _post_stage(
    *,
    target_deg: list[float],
    segment_joints: list[str],
    recipe_sha256: str,
    source_start_ns: int,
    sequence_start: int,
    restore: bool = False,
) -> dict[str, Any]:
    segments = [
        _post_segment(
            joint=joint,
            target_deg=target_deg[validator.JOINT_NAMES.index(joint)],
            recipe_sha256=recipe_sha256,
            source_start_ns=source_start_ns + index * 1_000_000_000,
            sequence_start=sequence_start + index * 10,
            gui_sequence=index + 1,
        )
        for index, joint in enumerate(segment_joints)
    ]
    final_trace = _final_dwell_trace(
        target_deg,
        source_start_ns + len(segments) * 1_000_000_000,
        sequence_start + len(segments) * 10,
    )
    common: dict[str, Any] = {
        "result": "PASS",
        "status": "PASS",
        "actual_twin_source": "REAL_ENCODER",
        "target_deg": list(target_deg),
        "final_deg": list(target_deg),
        "max_error_deg_by_joint": {
            joint: 0.0 for joint in validator.JOINT_NAMES
        },
        "minimum_dwell_s": 0.5,
        "recipe_sha256": recipe_sha256,
        "segments": segments,
        "final_dwell_trace": final_trace,
        "final_dwell_trace_sha256": _document_sha256({
            "schema": "V15.31B-final-dwell-trace-v1",
            "samples": final_trace,
        }),
        "final_hardware_state_sha256": final_trace[-1][
            "hardware_state_sha256"
        ],
    }
    if restore:
        common.update({
            "planned_preview": "PASS",
            "explicit_user_submit": True,
        })
    else:
        common["start_deg"] = [1.0, 1.0, 0.0, 0.0, 0.0, 0.0]
    return common


def _multi_joint_document() -> dict[str, Any]:
    target = [1.0, 1.0, 0.0, 0.0, 0.0, 0.0]
    recipe = "9" * 64
    segments = [
        _post_segment(
            joint=joint,
            target_deg=1.0,
            recipe_sha256=recipe,
            source_start_ns=300_000_000_000 + index * 1_000_000_000,
            sequence_start=2_000 + index * 10,
            gui_sequence=index + 1,
        )
        for index, joint in enumerate(("J1", "J2"))
    ]
    final_trace = _final_dwell_trace(target, 302_000_000_000, 2_020)
    thermal_setup = _post_stage(
        target_deg=[0.0, 2.0, 0.0, 0.0, 0.0, 0.0],
        segment_joints=["J2"],
        recipe_sha256="a" * 64,
        source_start_ns=310_000_000_000,
        sequence_start=2_100,
    )
    restore = _post_stage(
        target_deg=[0.0] * 6,
        segment_joints=["J2"],
        recipe_sha256="b" * 64,
        source_start_ns=400_000_000_000_000,
        sequence_start=400_000,
        restore=True,
    )
    return {
        "schema": "V15.31B-multi-joint-validation-v1",
        "result": "PASS",
        "binding": _binding_document(),
        "planned_preview": "PASS",
        "preview_before_real": True,
        "planned_twin": "PASS",
        "actual_twin": "PASS",
        "explicit_user_submit": True,
        "actual_twin_source": "REAL_ENCODER",
        "start_deg": [0.0] * 6,
        "target_deg": target,
        "final_deg": target,
        "max_error_deg_by_joint": {joint: 0.0 for joint in validator.JOINT_NAMES},
        "minimum_dwell_s": 0.5,
        "recipe_sha256": recipe,
        "segments": segments,
        "final_dwell_trace": final_trace,
        "final_dwell_trace_sha256": _document_sha256({
            "schema": "V15.31B-final-dwell-trace-v1",
            "samples": final_trace,
        }),
        "final_hardware_state_sha256": final_trace[-1][
            "hardware_state_sha256"
        ],
        "thermal_setup": thermal_setup,
        "restore_initial": restore,
    }


def _thermal_rows(duration_min: int) -> list[dict[str, Any]]:
    duration_s = duration_min * 60
    base_ns = duration_min * 10_000_000_000_000
    slope = {5: 0.3, 15: 0.2, 30: 0.1}[duration_min]
    return [
        _thermal_row(
            base_ns + elapsed_s * 1_000_000_000,
            float(elapsed_s),
            34.0 + elapsed_s / duration_s,
            slope,
        )
        for elapsed_s in range(0, duration_s + 1, 2)
    ]


def _thermal_row(
    monotonic_ns: int,
    elapsed_s: float,
    temperature_c: float,
    slope_c_per_min: float = 0.1,
) -> dict[str, Any]:
    return {
        "timestamp_utc": UTC,
        "monotonic_ns": monotonic_ns,
        "receipt_monotonic_ns": monotonic_ns + 1_000_000,
        "elapsed_s": elapsed_s,
        "j2a_temperature_c": temperature_c,
        "j2b_temperature_c": temperature_c,
        "j2a_slope_c_per_min": slope_c_per_min,
        "j2b_slope_c_per_min": slope_c_per_min,
        "j2a_tau_feedback_rotor_nm": 0.2,
        "j2b_tau_feedback_rotor_nm": 0.2,
        "j2a_gravity_ff_rotor_nm": 0.1,
        "j2b_gravity_ff_rotor_nm": 0.1,
        "j2a_pd_rotor_nm": 0.1,
        "j2b_pd_rotor_nm": 0.1,
        "position_error_deg": 0.1,
        "j2_e_sync_deg": 0.1,
        "saturation_observed": "false",
        "j2a_merror": 0,
        "j2b_merror": 0,
        "j2a_communication_ok": "true",
        "j2b_communication_ok": "true",
        "status": "PASS",
        "reason": "",
        **_binding_document(),
        "hardware_state_sha256": f"{monotonic_ns // 1_000_000_000:064x}",
        "hardware_state_sequence": monotonic_ns // 1_000_000_000,
        "hardware_state_source_monotonic_ns": monotonic_ns,
    }


def _thermal_summary_document() -> dict[str, Any]:
    motor = {
        "start_temperature_c": 34.0,
        "temperature_5min_c": 35.0,
        "temperature_15min_c": 35.0,
        "temperature_30min_c": 35.0,
        "final_slope_c_per_min": 0.1,
        "final_slope_observed_stage_min": 30,
    }
    return {
        "schema_version": "V15.31B-thermal-summary-v1",
        "result": "PASS",
        "classification": "CONTROL_EXCESS_TORQUE_SOLVED",
        "counterbalance_required": "NO",
        "J2A": dict(motor),
        "J2B": dict(motor),
        "j2_sustained_rotor_torque": {"before_gravity_nm": 0.4, "after_gravity_nm": 0.2},
        "saturation_ratio": {"before": 1.0, "after": 0.0},
        "classification_a_objective_metrics": {
            "criteria": {
                "mean_abs_position_error_strictly_lower": True,
                "peak_abs_position_error_strictly_lower": True,
                "mean_abs_pd_materially_lower": True,
                "peak_abs_pd_materially_lower": True,
                "mean_abs_feedback_torque_materially_lower": True,
                "peak_abs_feedback_torque_materially_lower": True,
                "saturation_strictly_lower_and_eliminated": True,
                "with_ff_no_internal_opposition": True,
                "thermal_trend_improved": True,
                "thirty_minute_hold_stable": True,
            },
        },
        "thermal_stage_disposition": {
            "5": "PASS_OBSERVED",
            "15": "PASS_OBSERVED",
            "30": "PASS_OBSERVED",
        },
        "binding": _binding_document(),
    }


def _thermal_not_run_row() -> dict[str, Any]:
    row: dict[str, Any] = _blocked_csv_row(validator.THERMAL_FIELDS)
    row.update({
        "timestamp_utc": UTC,
        "status": "NOT_RUN_EARLY_FUNDAMENTAL_LIMIT",
        "reason": "NOT_RUN_EARLY_FUNDAMENTAL_LIMIT",
        **_binding_document(),
    })
    return row


def _early_b_thermal_summary(temperature_5min_c: float) -> dict[str, Any]:
    not_run_endpoint = {
        "status": "NOT_RUN_EARLY_FUNDAMENTAL_LIMIT",
        "observed": False,
        "value_c": None,
        "last_observed_stage_min": 5,
        "last_observed_value_c": temperature_5min_c,
    }
    motor = {
        "start_temperature_c": 34.0,
        "temperature_5min_c": temperature_5min_c,
        "temperature_15min_c": dict(not_run_endpoint),
        "temperature_30min_c": dict(not_run_endpoint),
        "final_slope_c_per_min": 0.3,
        "final_slope_observed_stage_min": 5,
    }
    observed_per_motor = {"J2A": 0.1, "J2B": 0.1}
    target_per_motor = {
        motor_name: value * (1.0 - validator.MECHANICAL_UNLOAD_FRACTION)
        for motor_name, value in observed_per_motor.items()
    }
    observed_joint_nm = sum(observed_per_motor.values()) * validator.GO_GEAR_RATIO
    target_joint_nm = sum(target_per_motor.values()) * validator.GO_GEAR_RATIO
    reduction_nm = observed_joint_nm - target_joint_nm
    return {
        "schema_version": "V15.31B-thermal-summary-v1",
        "result": "HARDWARE_COUNTERBALANCE_REQUIRED",
        "classification": "FUNDAMENTAL_CONTINUOUS_LOAD_LIMIT",
        "counterbalance_required": "YES",
        "J2A": dict(motor),
        "J2B": dict(motor),
        "j2_sustained_rotor_torque": {
            "before_gravity_nm": 0.4,
            "after_gravity_nm": 0.2,
        },
        "saturation_ratio": {"before": 1.0, "after": 0.0},
        "classification_a_objective_metrics": {
            "criteria": {
                "mean_abs_position_error_strictly_lower": True,
                "peak_abs_position_error_strictly_lower": True,
                "mean_abs_pd_materially_lower": True,
                "peak_abs_pd_materially_lower": True,
                "mean_abs_feedback_torque_materially_lower": True,
                "peak_abs_feedback_torque_materially_lower": True,
                "saturation_strictly_lower_and_eliminated": True,
                "with_ff_no_internal_opposition": True,
                "thermal_trend_improved": False,
                "thirty_minute_hold_stable": False,
            },
        },
        "thermal_stage_disposition": {
            "5": "PASS_OBSERVED",
            "15": "NOT_RUN_EARLY_FUNDAMENTAL_LIMIT",
            "30": "NOT_RUN_EARLY_FUNDAMENTAL_LIMIT",
        },
        "mechanical_recommendation": {
            "schema": "V15.31B-mechanical-counterbalance-recommendation-v1",
            "required_j2_torque_reduction_nm": reduction_nm,
            "observed_load_basis": {
                "per_motor_mean_abs_gravity_rotor_nm": observed_per_motor,
                "per_motor_target_abs_gravity_rotor_nm": target_per_motor,
                "go_gear_ratio": validator.GO_GEAR_RATIO,
                "observed_j2_gravity_joint_nm": observed_joint_nm,
                "target_j2_gravity_joint_nm": target_joint_nm,
                "minimum_mechanical_unload_fraction": (
                    validator.MECHANICAL_UNLOAD_FRACTION
                ),
                "joint_torque_formula": (
                    "(|J2A_rotor_Nm|+|J2B_rotor_Nm|)*GO_GEAR_RATIO"
                ),
                "required_reduction_formula": (
                    "observed_j2_gravity_joint_nm-target_j2_gravity_joint_nm"
                ),
                "authority": (
                    "EMPIRICAL_REVALIDATION_TARGET_NOT_OFFICIAL_CONTINUOUS_RATING"
                ),
            },
            "gas_spring": {
                "force_n": reduction_nm / 0.1,
                "lever_arm_m": 0.1,
                "estimated_torque_nm": reduction_nm,
            },
        },
        "binding": _binding_document(),
    }


def _gui_document() -> dict[str, Any]:
    return {
        "schema_version": "V15.31B-gui-powered-v1",
        "result": "PASS",
        "binding": _binding_document(),
        "checks": {name: "PASS" for name in validator.GUI_CHECKS},
    }


def _git_provenance() -> dict[str, Any]:
    head = _head_commit()
    return {
        "schema": "V15.31B-run-git-provenance-v1",
        "repo_root": str(REPO_ROOT),
        "output_directory": str(REPO_ROOT / "hardware" / "v15_31b_ft"),
        "branch": validator.EXPECTED_BRANCH,
        "run_base_commit": head,
        "upstream_ref": f"origin/{validator.EXPECTED_BRANCH}",
        "upstream_commit": head,
        "ahead_count": 0,
        "behind_count": 0,
        "allowed_untracked_paths": sorted(
            f"hardware/v15_31b_ft/{name}"
            for name in (
                "power_on_readonly.json",
                "model_session_anchor_validation.json",
                "gravity_readonly_validation.json",
            )
        ),
        "captured_at_utc": UTC,
        "evidence_commit_semantics": (
            "EVIDENCE_RUN_BASE_COMMIT_NOT_SELF_REFERENTIAL_SEAL_COMMIT"
        ),
    }


def _final_brake_document() -> dict[str, Any]:
    trace: list[dict[str, Any]] = []
    source_instance_id = "5" * 32
    center_rad = [0.0] * 6
    for offset in range(6):
        source_ns = 500_000_000_000_000 + offset * 100_000_000
        sequence = 500_000 + offset
        trace.append({
            "hardware_state_sequence": sequence,
            "hardware_state_source_monotonic_ns": source_ns,
            "hardware_state_sha256": f"{sequence:064x}",
            "position_rad": list(center_rad),
            "position_error_from_session_center_deg": [0.0] * 6,
            "go_controller_modes": {
                motor: "brake" for motor in validator.GO_MOTOR_NAMES
            },
            "j6_aggregated_controller_mode": "brake",
            "j6_raw_source_monotonic_ns": source_ns,
            "j6_raw_source_instance_id": source_instance_id,
            "j6_raw_sequence": offset + 1,
            "j6_raw_session_id": SESSION_ID,
            "j6_raw_state_instance_id": STATE_INSTANCE_ID,
            "j6_raw_sha256": f"{sequence + 10_000:064x}",
            "j6_raw_drive_state": 0,
            "j6_raw_controller_mode": "brake",
            "pair_receipt_monotonic_ns": source_ns + 1_000_000,
        })
    trace_document = {
        "schema": "V15.31B-final-brake-paired-trace-v1",
        "samples": trace,
    }
    return {
        "status": "PASS",
        "go": "BRAKE",
        "j6": "DISABLED",
        "hardware_state_sha256": trace[-1]["hardware_state_sha256"],
        "j6_disabled_raw_sha256": trace[-1]["j6_raw_sha256"],
        "continuous_dwell_s": 0.5,
        "hardware_source_dwell_s": 0.5,
        "j6_raw_source_dwell_s": 0.5,
        "receipt_dwell_s": 0.5,
        "maximum_source_gap_ms": 100.0,
        "session_center_rad": center_rad,
        "maximum_position_error_from_session_center_deg": 0.0,
        "paired_sample_count": len(trace),
        "paired_trace": trace,
        "paired_trace_schema": trace_document["schema"],
        "paired_trace_sha256": _document_sha256(trace_document),
    }


def _operator_confirmation_trace() -> tuple[list[dict[str, Any]], str]:
    requirements: dict[int, float] = {}

    def add(receipt_ns: Any, target: float) -> None:
        receipt = int(receipt_ns)
        previous = requirements.get(receipt)
        assert previous is None or math.isclose(previous, target, abs_tol=1.0e-12)
        requirements[receipt] = target

    for row in _gravity_scale_rows():
        add(row["observer_receipt_monotonic_ns"], float(row["gravity_scale_target"]))
    for row in _j2_comparison_rows():
        add(
            row["receipt_monotonic_ns"],
            0.0 if row["condition"] == "WITHOUT_FF" else 1.0,
        )
    for row in _position_rows():
        for field in (
            "endpoint_dwell_trace_json",
            "segment_execution_trace_json",
        ):
            for sample in json.loads(row[field])["samples"]:
                add(sample["receipt_monotonic_ns"], 1.0)

    def add_nested_receipts(value: Any) -> None:
        if isinstance(value, dict):
            for key, item in value.items():
                if key == "receipt_monotonic_ns":
                    add(item, 1.0)
                else:
                    add_nested_receipts(item)
        elif isinstance(value, list):
            for item in value:
                add_nested_receipts(item)

    add_nested_receipts(_multi_joint_document())
    for stage in (5, 15, 30):
        for row in _thermal_rows(stage):
            add(row["receipt_monotonic_ns"], 1.0)
    for sample in _final_brake_document()["paired_trace"]:
        add(sample["pair_receipt_monotonic_ns"], 1.0)

    binding = _binding_document()
    samples = [
        {
            "schema": "go-m8010-empirical-stage-confirmation/1.0",
            "source_instance_id": "c" * 32,
            "sequence": sequence,
            "source_monotonic_ns": receipt_ns,
            "envelope_id": binding["envelope_id"],
            "envelope_sha256": binding["envelope_sha256"],
            "session_id": binding["session_id"],
            "state_instance_id": binding["state_instance_id"],
            "target_gravity_scale": target,
            "operator_stop_ready": True,
            "j2_j3_support_reliable": True,
            "clearance_confirmed": True,
            "no_person_contact": True,
            "observer_receipt_monotonic_ns": receipt_ns,
        }
        for sequence, (receipt_ns, target) in enumerate(
            sorted(requirements.items()), start=1
        )
    ]
    document = {
        "schema": "V15.31B-operator-confirmation-trace-v1",
        "samples": samples,
    }
    return samples, _document_sha256(document)


def _full_pass_report() -> dict[str, Any]:
    head = _head_commit()
    motor = {
        "start_temp_c": 34.0,
        "temp_5min_c": 35.0,
        "temp_15min_c": 35.0,
        "temp_30min_c": 35.0,
        "final_slope_c_per_min": 0.1,
    }
    values: dict[str, Any] = {
        "V15.31B RESULT": "FULL_PASS",
        "branch": validator.EXPECTED_BRANCH,
        "implementation commit": head,
        "evidence commit": head,
        "Power-on read-only": _pass_status(),
        "Anchor": _pass_status(),
        "Gravity calculation": _pass_status(),
        "Gravity powered result": _pass_status(),
        "multi-joint": _pass_status(),
        "J2A": dict(motor),
        "J2B": dict(motor),
        "J2 sustained rotor torque": {"before_gravity_nm": 0.4, "after_gravity_nm": 0.2},
        "saturation ratio": {"before": 1.0, "after": 0.0},
        "thermal classification": "CONTROL_EXCESS_TORQUE_SOLVED",
        "counterbalance required": "NO",
        "GUI powered result": _pass_status(),
        "restore initial": _multi_joint_document()["restore_initial"],
        "final brake": _final_brake_document(),
        "motor zero modified": "NO",
        "MuJoCo modified": "NO",
        "git clean": "YES",
        "PRIMARY BLOCKER": "NONE",
        "FINAL TASK RESULT": "FULL_PASS",
        "READY_FOR_NEXT_STAGE": True,
    }
    for joint in validator.JOINT_NAMES:
        values[f"{joint} error"] = {"status": "PASS", "max_error_deg": 0.0}
    values["J2 max sync"] = {"status": "PASS", "max_sync_error_deg": 0.1}
    return values


def _hardware_counterbalance_report() -> dict[str, Any]:
    values = _full_pass_report()
    summary = _early_b_thermal_summary(35.0)
    for field in ("V15.31B RESULT", "FINAL TASK RESULT"):
        values[field] = "HARDWARE_COUNTERBALANCE_REQUIRED"
    values["READY_FOR_NEXT_STAGE"] = False
    values["PRIMARY BLOCKER"] = "HARDWARE_COUNTERBALANCE_REQUIRED"
    values["thermal classification"] = "FUNDAMENTAL_CONTINUOUS_LOAD_LIMIT"
    values["counterbalance required"] = "YES"
    for motor in ("J2A", "J2B"):
        values[motor] = {
            "start_temp_c": summary[motor]["start_temperature_c"],
            "temp_5min_c": summary[motor]["temperature_5min_c"],
            "temp_15min_c": summary[motor]["temperature_15min_c"],
            "temp_30min_c": summary[motor]["temperature_30min_c"],
            "final_slope_c_per_min": summary[motor]["final_slope_c_per_min"],
        }
    return values


def _blocked_report() -> dict[str, Any]:
    blocked = _blocked_status
    values: dict[str, Any] = {
        "V15.31B RESULT": blocked(),
        "branch": validator.EXPECTED_BRANCH,
        "implementation commit": blocked(),
        "evidence commit": blocked(),
        "Power-on read-only": blocked(),
        "Anchor": blocked(),
        "Gravity calculation": blocked(),
        "Gravity powered result": blocked(),
        "multi-joint": blocked(),
        "J2A": blocked(),
        "J2B": blocked(),
        "J2 sustained rotor torque": blocked(),
        "saturation ratio": blocked(),
        "thermal classification": "UNDETERMINED_POWER_OFF",
        "counterbalance required": blocked(),
        "GUI powered result": blocked(),
        "restore initial": blocked(),
        "final brake": blocked(),
        "motor zero modified": "NO",
        "MuJoCo modified": "NO",
        "git clean": "YES",
        "PRIMARY BLOCKER": "POWER_OFF",
        "FINAL TASK RESULT": blocked(),
        "READY_FOR_NEXT_STAGE": False,
    }
    for joint in validator.JOINT_NAMES:
        values[f"{joint} error"] = blocked()
    values["J2 max sync"] = blocked()
    return values


def _final_document(values: dict[str, Any]) -> dict[str, Any]:
    provenance = _git_provenance()
    confirmation_trace, confirmation_sha = _operator_confirmation_trace()
    return {
        "schema": "V15.31B-final-result-v1",
        "result": values["V15.31B RESULT"],
        "binding": {
            **_binding_document(),
            "worker_supervisor_instance_id": "6" * 32,
            "worker_supervisor_pid": 4242,
            "run_git_provenance": provenance,
            "preseal_git_provenance": dict(provenance),
            "operator_confirmation_trace": confirmation_trace,
            "operator_confirmation_trace_sha256": confirmation_sha,
        },
        "report_fields": [
            {"id": index, "name": name, "value": values[name]}
            for index, name in enumerate(validator.FINAL_FIELD_NAMES, start=1)
        ],
    }


def _blocked_csv_row(fields: tuple[str, ...]) -> dict[str, str]:
    row = {field: "" for field in fields}
    row["status"] = "BLOCKED_POWER_OFF"
    row["reason"] = "POWER_OFF"
    return row


def _make_pass_bundle(bundle: Path) -> Path:
    bundle.mkdir()
    _write_json(bundle / "power_on_readonly.json", _power_on_document())
    power_sha256 = hashlib.sha256(
        (bundle / "power_on_readonly.json").read_bytes()
    ).hexdigest()
    anchor_document = _anchor_document(power_sha256)
    _write_json(bundle / "model_session_anchor_validation.json", anchor_document)
    _write_json(
        bundle / "gravity_readonly_validation.json",
        _gravity_readonly_document(anchor_document["runtime_anchor_sha256"]),
    )
    _write_csv(bundle / "gravity_scale_validation.csv", validator.GRAVITY_SCALE_FIELDS, _gravity_scale_rows())
    _write_csv(bundle / "position_validation.csv", validator.POSITION_FIELDS, _position_rows())
    _write_csv(bundle / "j2_control_comparison.csv", validator.J2_COMPARISON_FIELDS, _j2_comparison_rows())
    _write_json(bundle / "multi_joint_validation.json", _multi_joint_document())
    for duration in (5, 15, 30):
        _write_csv(bundle / f"thermal_{duration}min.csv", validator.THERMAL_FIELDS, _thermal_rows(duration))
    _write_json(bundle / "thermal_summary.json", _thermal_summary_document())
    _write_json(bundle / "gui_powered_validation.json", _gui_document())
    _write_json(bundle / "final_result.json", _final_document(_full_pass_report()))
    _rewrite_manifest(bundle)
    return bundle


def _make_blocked_bundle(bundle: Path) -> Path:
    bundle.mkdir()
    blocked_json = {"schema_version": "V15.31B-blocked-v1", "result": "BLOCKED_POWER_OFF", "reason": "POWER_OFF"}
    for name in (
        "power_on_readonly.json",
        "model_session_anchor_validation.json",
        "gravity_readonly_validation.json",
        "multi_joint_validation.json",
        "gui_powered_validation.json",
    ):
        _write_json(bundle / name, blocked_json)
    _write_csv(bundle / "gravity_scale_validation.csv", validator.GRAVITY_SCALE_FIELDS, [_blocked_csv_row(validator.GRAVITY_SCALE_FIELDS)])
    _write_csv(bundle / "position_validation.csv", validator.POSITION_FIELDS, [_blocked_csv_row(validator.POSITION_FIELDS)])
    _write_csv(bundle / "j2_control_comparison.csv", validator.J2_COMPARISON_FIELDS, [_blocked_csv_row(validator.J2_COMPARISON_FIELDS)])
    for duration in (5, 15, 30):
        _write_csv(bundle / f"thermal_{duration}min.csv", validator.THERMAL_FIELDS, [_blocked_csv_row(validator.THERMAL_FIELDS)])
    _write_json(bundle / "thermal_summary.json", {
        "schema_version": "V15.31B-thermal-summary-v1",
        "result": "BLOCKED_POWER_OFF",
        "reason": "POWER_OFF",
        "classification": "UNDETERMINED_POWER_OFF",
    })
    _write_json(bundle / "final_result.json", _final_document(_blocked_report()))
    _rewrite_manifest(bundle)
    return bundle


def _make_hardware_counterbalance_bundle(bundle: Path) -> Path:
    _make_pass_bundle(bundle)
    for stage in (15, 30):
        _write_csv(
            bundle / f"thermal_{stage}min.csv",
            validator.THERMAL_FIELDS,
            [_thermal_not_run_row()],
        )
    _write_json(
        bundle / "thermal_summary.json", _early_b_thermal_summary(35.0)
    )
    _write_json(
        bundle / "final_result.json",
        _final_document(_hardware_counterbalance_report()),
    )
    _rewrite_manifest(bundle)
    return bundle


def _update_final_value(bundle: Path, field: str, value: Any) -> None:
    path = bundle / "final_result.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    row = next(item for item in document["report_fields"] if item["name"] == field)
    row["value"] = value
    _write_json(path, document)
    _rewrite_manifest(bundle)


def _assert_rejected(bundle: Path, text: str) -> None:
    with pytest.raises(validator.EvidenceValidationError, match=text):
        validator.validate_evidence(
            bundle, REPO_ROOT, allow_unsealed_repository=True
        )


def test_complete_pass_bundle_verifies_14_files_31_fields_and_commits(tmp_path: Path) -> None:
    bundle = _make_pass_bundle(tmp_path / "pass")

    result = validator.validate_evidence(
        bundle, REPO_ROOT, allow_unsealed_repository=True
    )

    assert result["required_file_count"] == 14
    assert result["sha256_verified"] == "13/13"
    assert result["report_field_count"] == 31
    assert result["FINAL_TASK_RESULT"] == "PASS"
    assert result["READY_FOR_NEXT_STAGE"] is True


def test_structured_power_off_bundle_cannot_claim_a_terminal_contract_result(
    tmp_path: Path,
) -> None:
    bundle = _make_blocked_bundle(tmp_path / "blocked")

    _assert_rejected(
        bundle,
        "terminal acceptance requires gravity, comparison, position, and multi-joint PASS",
    )


@pytest.mark.parametrize(
    ("artifact_name", "report_field"),
    [
        ("gravity_readonly_validation.json", "Gravity calculation"),
        ("gui_powered_validation.json", "GUI powered result"),
    ],
)
def test_hardware_counterbalance_terminal_requires_nonthermal_passes(
    tmp_path: Path,
    artifact_name: str,
    report_field: str,
) -> None:
    bundle = _make_hardware_counterbalance_bundle(
        tmp_path / f"hcr-{artifact_name}"
    )
    _write_json(
        bundle / artifact_name,
        {
            "schema_version": "V15.31B-blocked-v1",
            "result": "BLOCKED_POWER_OFF",
            "reason": "POWER_OFF",
        },
    )
    _update_final_value(bundle, report_field, _blocked_status())

    _assert_rejected(
        bundle,
        "terminal result requires every non-thermal acceptance artifact to PASS",
    )


@pytest.mark.parametrize("drift", (0.3, 0.51))
def test_validator_terminal_support_window_is_not_active_endpoint_precision(drift):
    document = _final_brake_document()
    document["maximum_position_error_from_session_center_deg"] = drift
    for sample in document["paired_trace"]:
        sample["position_rad"][0] = math.radians(drift)
        sample["position_error_from_session_center_deg"][0] = drift
    document["paired_trace_sha256"] = _document_sha256({
        "schema": document["paired_trace_schema"], "samples": document["paired_trace"]})
    args = (document, _binding_document(), dict.fromkeys(validator.JOINT_NAMES, 0.0))
    if drift < 0.5:
        assert validator._validate_final_brake(*args, after_sequence=0, after_source_ns=0)
    else:
        with pytest.raises(validator.EvidenceValidationError, match="restored pose drift evidence invalid"):
            validator._validate_final_brake(*args, after_sequence=0, after_source_ns=0)


@pytest.mark.parametrize("actual_center_deg", (-0.1, 0.1))
def test_precision_contract_accepts_two_endpoint_error_budget_without_overshoot(tmp_path, actual_center_deg):
    rows = _position_rows(actual_center_deg=actual_center_deg, endpoint_error_deg=0.1)
    path = tmp_path / "position_validation.csv"
    _write_csv(path, validator.POSITION_FIELDS, rows)
    result, metrics = validator._validate_position(path, _binding_document())
    assert result == "PASS"
    assert max(metrics["maximum_error_deg"].values()) == pytest.approx(0.1)
    boundary_phase = "PLUS_5" if actual_center_deg > 0 else "MINUS_5"
    boundary = next(row for row in rows if row["joint"] == "J1" and row["phase"] == boundary_phase)
    assert abs(boundary["actual_displacement_from_center_deg"]) == pytest.approx(4.8)
    assert abs(boundary["actual_deg"]) == pytest.approx(4.9)
    assert boundary["nominal_command_displacement_deg"] == 5.0


@pytest.mark.parametrize("field,value", (("precision_contract_id", ""),
    ("endpoint_error_limit_deg", 0.5), ("nominal_command_displacement_deg", 4.8),
    ("minimum_required_actual_displacement_deg", 5.0)))
def test_validator_rejects_old_or_mixed_precision_evidence(tmp_path, field, value):
    rows = _position_rows()
    row = next(item for item in rows if item["joint"] == "J1" and item["phase"] == "PLUS_5")
    row[field] = value
    path = tmp_path / "position_validation.csv"
    _write_csv(path, validator.POSITION_FIELDS, rows)
    with pytest.raises(validator.EvidenceValidationError):
        validator._validate_position(path, _binding_document())


def test_validator_rejects_unversioned_legacy_position_csv_header(tmp_path):
    precision_fields = {"precision_contract_id", "endpoint_error_limit_deg", "nominal_command_displacement_deg"}
    legacy_fields = tuple(field for field in validator.POSITION_FIELDS if field not in precision_fields)
    rows = [{field: row[field] for field in legacy_fields} for row in _position_rows()]
    path = tmp_path / "position_validation.csv"
    _write_csv(path, legacy_fields, rows)
    with pytest.raises(validator.EvidenceValidationError, match="CSV fields/order"):
        validator._validate_position(path, _binding_document())


@pytest.mark.parametrize("held_joint", (False, True))
def test_validator_rejects_0_1001_degree_error_on_moving_or_held_joint(tmp_path, held_joint):
    rows = _position_rows(endpoint_error_deg=0.0 if held_joint else 0.1001)
    if held_joint:
        row = next(item for item in rows if item["joint"] == "J1" and item["phase"] == "PLUS_5")
        for prefix in ("endpoint_dwell_trace", "segment_execution_trace"):
            document = json.loads(row[prefix + "_json"])
            for sample in document["samples"]:
                sample["position_rad"][1] = math.radians(0.1001)
            row[prefix + "_json"] = json.dumps(document)
            row[prefix + "_sha256"] = _document_sha256(document)
    path = tmp_path / "position_validation.csv"
    _write_csv(path, validator.POSITION_FIELDS, rows)
    with pytest.raises(validator.EvidenceValidationError):
        validator._validate_position(path, _binding_document())


@pytest.mark.parametrize(
    ("updates", "message"),
    [
        ({"target_deg": "4.99", "actual_deg": "4.89", "error_deg": "0.10"}, "target/actual/error is not self-consistent"),
        ({"actual_deg": "4.89", "error_deg": "0.11"}, "endpoint error exceeds 0.1 degree"),
        ({"endpoint_dwell_s": "0.49"}, "endpoint_dwell_s disagrees with source trace"),
    ],
)
def test_position_envelope_error_and_dwell_are_fail_closed(
    tmp_path: Path,
    updates: dict[str, str],
    message: str,
) -> None:
    bundle = _make_pass_bundle(tmp_path / "position-threshold")
    _modify_csv(
        bundle / "position_validation.csv",
        lambda row: row["joint"] == "J1" and row["phase"] == "PLUS_5",
        updates,
    )
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, message)


@pytest.mark.parametrize(
    ("joint", "sync", "message"),
    [
        ("J2", "0.26", "J2 warning sync threshold exceeded"),
        ("J1", "0.51", "J2 hard sync threshold exceeded"),
    ],
)
def test_j2_warning_and_hard_sync_thresholds_are_enforced(
    tmp_path: Path,
    joint: str,
    sync: str,
    message: str,
) -> None:
    bundle = _make_pass_bundle(tmp_path / "j2-threshold")
    _modify_csv(
        bundle / "position_validation.csv",
        lambda row: row["joint"] == joint and row["phase"] == "PLUS_5",
        {"j2_e_sync_deg": sync},
    )
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, message)


def test_short_thermal_stage_is_rejected(tmp_path: Path) -> None:
    bundle = _make_pass_bundle(tmp_path / "short-thermal")
    _modify_csv(
        bundle / "thermal_15min.csv",
        lambda row: row["elapsed_s"] == "900.0",
        {
            "elapsed_s": "899.0",
            "monotonic_ns": str(15 * 10_000_000_000_000 + 899_000_000_000),
            "receipt_monotonic_ns": str(
                15 * 10_000_000_000_000 + 899_001_000_000
            ),
            "hardware_state_source_monotonic_ns": str(
                15 * 10_000_000_000_000 + 899_000_000_000
            ),
        },
    )
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, "stage duration below 15 minutes")


def test_later_thermal_stage_cannot_pass_after_5min_block(tmp_path: Path) -> None:
    bundle = _make_pass_bundle(tmp_path / "thermal-order")
    _write_csv(
        bundle / "thermal_5min.csv",
        validator.THERMAL_FIELDS,
        [_blocked_csv_row(validator.THERMAL_FIELDS)],
    )
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, "later thermal stage cannot PASS")


def test_final_brake_cannot_fake_pass_without_motion_evidence(tmp_path: Path) -> None:
    bundle = _make_pass_bundle(tmp_path / "fake-brake")
    _update_final_value(bundle, "final brake", {"status": "PASS", "go": "BRAKE", "j6": "DISABLED"})

    _assert_rejected(bundle, "final brake: exact paired-trace fields required")


def test_final_summary_cannot_fake_pass_over_blocked_artifact(tmp_path: Path) -> None:
    bundle = _make_blocked_bundle(tmp_path / "fake-pass")
    document = _final_document(_full_pass_report())
    for field in (
        "envelope_id", "envelope_sha256", "session_id",
        "state_instance_id", "anchor_sha256",
    ):
        document["binding"][field] = None
    _write_json(bundle / "final_result.json", document)
    _rewrite_manifest(bundle)

    _assert_rejected(
        bundle,
        "terminal acceptance requires gravity, comparison, position, and multi-joint PASS",
    )


def test_placeholder_is_rejected_even_when_manifest_matches(tmp_path: Path) -> None:
    bundle = _make_blocked_bundle(tmp_path / "placeholder")
    path = bundle / "power_on_readonly.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    document["reason"] = "SAME_AS_COMMIT"
    _write_json(path, document)
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, "prohibited placeholder text")


def test_checksum_manifest_detects_tampering(tmp_path: Path) -> None:
    bundle = _make_pass_bundle(tmp_path / "tampered")
    path = bundle / "gui_powered_validation.json"
    path.write_text(path.read_text(encoding="utf-8") + " ", encoding="utf-8")

    _assert_rejected(bundle, "digest mismatch for gui_powered_validation.json")


def test_gravity_worker_echo_cannot_self_declare_unpublished_node_value(
    tmp_path: Path,
) -> None:
    bundle = _make_pass_bundle(tmp_path / "forged-gravity-echo")
    path = bundle / "gravity_scale_validation.csv"
    fields, rows = _read_csv(path)
    forged_node = [0.0, 0.9, 0.0, 0.0, 0.0, 0.0]
    forged_expected = [0.0, -0.9, 0.9, 0.0, 0.0, 0.0]
    rows[1]["matched_node_feedforward_nm_json"] = json.dumps(forged_node)
    rows[1]["matched_expected_worker_feedforward_rotor_nm_json"] = (
        json.dumps(forged_expected)
    )
    rows[1]["worker_feedforward_rotor_nm_json"] = json.dumps(
        forged_expected
    )
    _write_csv(path, tuple(fields), rows)
    _rewrite_manifest(bundle)

    _assert_rejected(
        bundle,
        "matched node publication is not a unique prior CSV row",
    )


def test_gravity_zero_bootstrap_exception_is_only_the_first_zero_row(
    tmp_path: Path,
) -> None:
    bundle = _make_pass_bundle(tmp_path / "forged-zero-bootstrap")
    path = bundle / "gravity_scale_validation.csv"
    fields, rows = _read_csv(path)
    rows[0]["node_feedforward_nm_json"] = "[0,0.1,0,0,0,0]"
    _write_csv(path, tuple(fields), rows)
    _rewrite_manifest(bundle)

    _assert_rejected(
        bundle,
        "matched node publication is not a unique prior CSV row",
    )


def test_gravity_observer_receipt_must_be_causal(tmp_path: Path) -> None:
    bundle = _make_pass_bundle(tmp_path / "gravity-receipt")
    path = bundle / "gravity_scale_validation.csv"
    fields, rows = _read_csv(path)
    rows[0]["observer_receipt_monotonic_ns"] = str(
        int(rows[0]["monotonic_ns"]) - 1
    )
    _write_csv(path, tuple(fields), rows)
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, "observer receipt is before source or stale")


def test_gravity_interstage_confirmation_cannot_be_omitted(tmp_path: Path) -> None:
    bundle = _make_pass_bundle(tmp_path / "gravity-interstage")
    path = bundle / "gravity_scale_validation.csv"
    _modify_csv(
        path,
        lambda row: (
            row["stage"] == "0%"
            and row["phase"] == "AWAIT_INTERSTAGE_CONFIRMATION"
        ),
        {"phase": "HOLD"},
    )
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, "gravity scale 0.0: phase order mismatch")


def test_global_powered_artifact_reordering_is_rejected(tmp_path: Path) -> None:
    bundle = _make_pass_bundle(tmp_path / "global-reorder")
    path = bundle / "j2_control_comparison.csv"
    fields, rows = _read_csv(path)
    with_ff_index = 0
    for row in rows:
        if row["condition"] != "WITH_FF":
            continue
        source_ns = 25_000_000_000 + with_ff_index * 100_000_000
        row["monotonic_ns"] = str(source_ns)
        row["receipt_monotonic_ns"] = str(source_ns + 1_000_000)
        row["hardware_state_sequence"] = str(10 + with_ff_index)
        row["hardware_state_source_monotonic_ns"] = str(source_ns)
        with_ff_index += 1
    _write_csv(path, tuple(fields), rows)
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, "powered gravity ladder must precede WITH_FF")


def test_position_execution_trace_must_converge_on_endpoint_trace(
    tmp_path: Path,
) -> None:
    bundle = _make_pass_bundle(tmp_path / "position-trace")
    path = bundle / "position_validation.csv"
    fields, rows = _read_csv(path)
    row = next(
        item for item in rows
        if item["joint"] == "J1" and item["phase"] == "PLUS_5"
    )
    document = json.loads(row["segment_execution_trace_json"])
    document["samples"][-1]["hardware_state_sha256"] = "f" * 64
    row["segment_execution_trace_json"] = json.dumps(
        document, separators=(",", ":")
    )
    row["segment_execution_trace_sha256"] = _document_sha256(document)
    _write_csv(path, tuple(fields), rows)
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, "execution and dwell do not converge on the same frame")


@pytest.mark.parametrize(
    "segment_path",
    ["multi", "thermal_setup", "restore_initial"],
)
def test_nested_machine_segment_summary_must_bind_retained_traces(
    tmp_path: Path, segment_path: str,
) -> None:
    bundle = _make_pass_bundle(tmp_path / f"nested-{segment_path}")
    path = bundle / "multi_joint_validation.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    if segment_path == "multi":
        segment = document["segments"][0]
    else:
        segment = document[segment_path]["segments"][0]
    segment["hardware_state_sha256"] = "f" * 64
    _write_json(path, document)
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, "endpoint summary does not bind the retained traces")


def test_post_segment_execute_at_may_follow_first_sample_but_not_last(
    tmp_path: Path,
) -> None:
    bundle = _make_pass_bundle(tmp_path / "post-execute-at")
    path = bundle / "multi_joint_validation.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    segment = document["segments"][0]
    first_receipt = int(segment["execution_trace"][0]["receipt_monotonic_ns"])
    last_receipt = int(segment["execution_trace"][-1]["receipt_monotonic_ns"])
    segment["trajectory_execute_at_monotonic_ns"] = (
        first_receipt + last_receipt
    ) // 2
    _write_json(path, document)
    _rewrite_manifest(bundle)

    result = validator.validate_evidence(
        bundle, REPO_ROOT, allow_unsealed_repository=True
    )
    assert result["FINAL_TASK_RESULT"] == "PASS"

    segment["trajectory_execute_at_monotonic_ns"] = last_receipt + 1
    _write_json(path, document)
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, "planned segment duration/execute time invalid")


def test_reused_trajectory_sha_is_charged_for_every_accepted_record(
    tmp_path: Path,
) -> None:
    bundle = _make_pass_bundle(tmp_path / "reused-sha-budget")
    path = bundle / "position_validation.csv"
    fields, rows = _read_csv(path)
    for row in rows:
        if row["phase"] != "CENTER_START":
            row["trajectory_duration_ns"] = str(15_000_000_000)

    revision_rows: list[dict[str, str]] = []
    for index, original in enumerate(rows, start=1):
        row = dict(original)
        row["revision"] = "REVISION_1"
        row["test_id"] = f"{row['joint']}-REVISION_1"
        center_observation = row["phase"] == "CENTER_START"
        source_start_ns = 240_000_000_000 + index * 1_000_000_000
        sequence_start = 1_400 + index * 10
        trajectory_sha = row["trajectory_sha256"]
        samples = _motion_trace(
            joint=row["joint"],
            target_deg=float(row["target_deg"]),
            trajectory_sha256=trajectory_sha,
            source_start_ns=source_start_ns,
            sequence_start=sequence_start,
            center_observation=center_observation,
        )
        dwell_document = {
            "schema": "V15.31B-endpoint-dwell-trace-v1",
            "samples": samples,
        }
        execution_document = {
            "schema": "V15.31B-segment-execution-trace-v1",
            "samples": [] if center_observation else samples,
        }
        top = samples[-1]
        row.update({
            "monotonic_ns": str(top["hardware_state_source_monotonic_ns"]),
            "receipt_monotonic_ns": str(top["receipt_monotonic_ns"]),
            "endpoint_dwell_trace_json": json.dumps(
                dwell_document, separators=(",", ":")
            ),
            "endpoint_dwell_trace_sha256": _document_sha256(dwell_document),
            "segment_execution_trace_json": json.dumps(
                execution_document, separators=(",", ":")
            ),
            "segment_execution_trace_sha256": _document_sha256(
                execution_document
            ),
            "hardware_state_sequence": str(top["hardware_state_sequence"]),
            "hardware_state_source_monotonic_ns": str(
                top["hardware_state_source_monotonic_ns"]
            ),
            "hardware_state_sha256": top["hardware_state_sha256"],
        })
        if not center_observation:
            row["trajectory_duration_ns"] = str(15_000_000_000)
            row["trajectory_execute_at_monotonic_ns"] = str(
                source_start_ns - 500_000
            )
            row["gui_command_sequence"] = str(100 + index)
            row["gui_command_source_monotonic_ns"] = str(
                source_start_ns - 1_000_000
            )
        revision_rows.append(row)
    rows.extend(revision_rows)
    _write_csv(path, tuple(fields), rows)

    status, metrics = validator._validate_position(path, _binding_document())
    assert status == "PASS"
    assert metrics["trajectory_duration_total_ns"] == 720_000_000_000
    _rewrite_manifest(bundle)
    _assert_rejected(
        bundle,
        "accepted POSITION trajectory duration exceeds the 600-second envelope budget",
    )


def test_early_b_requires_raw_safe_hot_trend_not_only_structured_not_run(
    tmp_path: Path,
) -> None:
    bundle = _make_pass_bundle(tmp_path / "early-b-proof")
    for stage in (15, 30):
        _write_csv(
            bundle / f"thermal_{stage}min.csv",
            validator.THERMAL_FIELDS,
            [_thermal_not_run_row()],
        )
    _modify_csv(
        bundle / "thermal_5min.csv",
        lambda row: row["elapsed_s"] == "300.0",
        {
            "j2a_temperature_c": "34.9",
            "j2b_temperature_c": "34.9",
        },
    )
    _write_json(
        bundle / "thermal_summary.json",
        _early_b_thermal_summary(34.9),
    )
    _rewrite_manifest(bundle)

    _assert_rejected(
        bundle,
        "early classification B lacks the raw safe 60-second hot-trend proof",
    )


def test_final_brake_rejects_replayed_j6_raw_sequence(tmp_path: Path) -> None:
    bundle = _make_pass_bundle(tmp_path / "final-brake-replay")
    path = bundle / "final_result.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    final_brake = next(
        item["value"] for item in document["report_fields"]
        if item["name"] == "final brake"
    )
    final_brake["paired_trace"][2]["j6_raw_sequence"] = final_brake[
        "paired_trace"
    ][1]["j6_raw_sequence"]
    final_brake["paired_trace_sha256"] = _document_sha256({
        "schema": final_brake["paired_trace_schema"],
        "samples": final_brake["paired_trace"],
    })
    _write_json(path, document)
    _rewrite_manifest(bundle)

    _assert_rejected(
        bundle,
        "final brake raw sequence: timestamps must be strictly increasing",
    )


def test_startup_and_preseal_git_provenance_must_remain_identical(
    tmp_path: Path,
) -> None:
    bundle = _make_pass_bundle(tmp_path / "provenance-changed")
    path = bundle / "final_result.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    document["binding"]["preseal_git_provenance"]["repo_root"] = "D:/changed"
    _write_json(path, document)
    _rewrite_manifest(bundle)

    _assert_rejected(
        bundle, "startup and preseal Git provenance changed during the powered run"
    )


def test_default_validation_rejects_nonstandard_unsealed_fixture_path(
    tmp_path: Path,
) -> None:
    bundle = _make_pass_bundle(tmp_path / "unsealed")

    with pytest.raises(
        validator.EvidenceValidationError,
        match="production validation requires repo_root/hardware/v15_31b_ft",
    ):
        validator.validate_evidence(bundle, REPO_ROOT)


def test_exact_31_field_contract_is_required(tmp_path: Path) -> None:
    bundle = _make_pass_bundle(tmp_path / "fields")
    path = bundle / "final_result.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    document["report_fields"].pop()
    _write_json(path, document)
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, "must contain exactly 31 fields")


def test_unresolvable_commit_is_rejected(tmp_path: Path) -> None:
    bundle = _make_pass_bundle(tmp_path / "bad-commit")
    path = bundle / "final_result.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    for item in document["report_fields"]:
        if item["name"] in {"implementation commit", "evidence commit"}:
            item["value"] = "f" * 40
    for provenance_name in ("run_git_provenance", "preseal_git_provenance"):
        provenance = document["binding"][provenance_name]
        provenance["run_base_commit"] = "f" * 40
        provenance["upstream_commit"] = "f" * 40
    _write_json(path, document)
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, "does not resolve")


def test_extra_evidence_file_is_rejected(tmp_path: Path) -> None:
    bundle = _make_pass_bundle(tmp_path / "extra-file")
    (bundle / "raw.log").write_text("not contracted\n", encoding="utf-8")

    _assert_rejected(bundle, "exactly the 14 contracted files")


def test_thermal_two_point_window_cannot_fake_continuous_pass(
    tmp_path: Path,
) -> None:
    bundle = _make_pass_bundle(tmp_path / "thermal-two-point")
    _write_csv(
        bundle / "thermal_30min.csv",
        validator.THERMAL_FIELDS,
        [
            _thermal_row(1_000_000_000, 0.0, 34.0),
            _thermal_row(1_801_000_000_000, 1800.0, 35.0),
        ],
    )
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, "continuous thermal evidence requires at least")


def test_thermal_gap_and_elapsed_monotonic_mismatch_are_rejected(
    tmp_path: Path,
) -> None:
    gap_bundle = _make_pass_bundle(tmp_path / "thermal-gap")
    fields, rows = _read_csv(gap_bundle / "thermal_5min.csv")
    rows.pop(1)
    _write_csv(gap_bundle / "thermal_5min.csv", tuple(fields), rows)
    _rewrite_manifest(gap_bundle)
    _assert_rejected(gap_bundle, "continuous thermal evidence requires at least")

    mismatch_bundle = _make_pass_bundle(tmp_path / "thermal-clock-mismatch")
    fields, rows = _read_csv(mismatch_bundle / "thermal_5min.csv")
    rows[1]["elapsed_s"] = "1.5"
    _write_csv(mismatch_bundle / "thermal_5min.csv", tuple(fields), rows)
    _rewrite_manifest(mismatch_bundle)
    _assert_rejected(mismatch_bundle, "elapsed_s is inconsistent with monotonic_ns")


def test_thermal_summary_cannot_lie_about_raw_comparison_metrics(
    tmp_path: Path,
) -> None:
    bundle = _make_pass_bundle(tmp_path / "thermal-summary-comparison-lie")
    path = bundle / "thermal_summary.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    document["j2_sustained_rotor_torque"]["after_gravity_nm"] = 0.1
    _write_json(path, document)
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, "sustained torque disagrees with J2 comparison CSV")


def test_thermal_summary_temperatures_must_match_continuous_csv(
    tmp_path: Path,
) -> None:
    bundle = _make_pass_bundle(tmp_path / "thermal-summary-temperature-lie")
    path = bundle / "thermal_summary.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    document["J2A"]["temperature_15min_c"] = 36.0
    _write_json(path, document)
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, "temperature_15min_c disagrees with thermal CSV")


def test_classification_a_requires_material_feedback_torque_reduction(
    tmp_path: Path,
) -> None:
    bundle = _make_pass_bundle(tmp_path / "thermal-no-feedback-improvement")
    comparison_path = bundle / "j2_control_comparison.csv"
    fields, rows = _read_csv(comparison_path)
    for row in rows:
        if row["condition"] == "WITHOUT_FF":
            row["j2a_tau_feedback_rotor_nm"] = "0.2"
            row["j2b_tau_feedback_rotor_nm"] = "0.2"
    _write_csv(comparison_path, tuple(fields), rows)
    path = bundle / "thermal_summary.json"
    document = json.loads(path.read_text(encoding="utf-8"))
    document["j2_sustained_rotor_torque"]["before_gravity_nm"] = 0.2
    document["classification_a_objective_metrics"]["criteria"][
        "mean_abs_feedback_torque_materially_lower"
    ] = False
    document["classification_a_objective_metrics"]["criteria"][
        "peak_abs_feedback_torque_materially_lower"
    ] = False
    _write_json(path, document)
    _update_final_value(
        bundle,
        "J2 sustained rotor torque",
        {"before_gravity_nm": 0.2, "after_gravity_nm": 0.2},
    )
    _rewrite_manifest(bundle)

    _assert_rejected(bundle, "classification A objective improvement is not proven")


def test_external_seal_reports_real_head_and_requires_clean_pushed_two_commit_chain(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    repo = tmp_path / "repo"
    evidence = repo / "hardware" / "v15_31b_ft"
    evidence.mkdir(parents=True)
    implementation = "a" * 40
    seal = "b" * 40
    expected_tree = "\n".join(
        f"hardware/v15_31b_ft/{name}" for name in validator.REQUIRED_ARTIFACTS
    )
    outputs = {
        ("branch", "--show-current"): validator.EXPECTED_BRANCH,
        ("rev-parse", "HEAD"): seal,
        ("status", "--porcelain=v1", "--untracked-files=all"): "",
        (
            "rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{u}",
        ): f"origin/{validator.EXPECTED_BRANCH}",
        ("rev-parse", "@{u}"): seal,
        ("rev-list", "--left-right", "--count", "HEAD...@{u}"): "0 0",
        ("rev-parse", f"{seal}^"): implementation,
        ("rev-parse", f"{implementation}^"): validator.SOURCE_COMMIT,
        (
            "show", "-s", "--format=%s", implementation,
        ): validator.IMPLEMENTATION_COMMIT_SUBJECT,
        (
            "show", "-s", "--format=%s", seal,
        ): validator.EVIDENCE_SEAL_COMMIT_SUBJECT,
        ("rev-list", "--count", f"{validator.SOURCE_COMMIT}..{seal}"): "2",
        (
            "ls-tree", "-r", "--name-only", "HEAD", "--",
            "hardware/v15_31b_ft",
        ): expected_tree,
        (
            "diff-tree", "--no-commit-id", "--name-only", "-r", seal,
        ): expected_tree,
        (
            "remote", "get-url", "origin",
        ): "https://github.com/zhaowuc/go-m8010-robot-arm.git",
        (
            "ls-remote", "--exit-code", "origin",
            f"refs/heads/{validator.EXPECTED_BRANCH}",
        ): f"{seal}\trefs/heads/{validator.EXPECTED_BRANCH}",
    }

    def fake_git(_repo: Path, arguments: list[str], _field: str) -> str:
        return outputs[tuple(arguments)]

    monkeypatch.setattr(validator, "_git_output", fake_git)
    result = validator._validate_sealed_repository(
        repo.resolve(), evidence.resolve(), implementation, implementation,
        allow_unsealed_repository=False,
    )

    assert result["sealed_evidence_commit"] == seal
    assert result["ahead"] == result["behind"] == 0

    seal_changes_key = (
        "diff-tree", "--no-commit-id", "--name-only", "-r", seal,
    )
    outputs[seal_changes_key] = expected_tree + "\ntools/unsafe.py"
    with pytest.raises(
        validator.EvidenceValidationError,
        match="seal commit may change only the exact 14 contracted evidence files",
    ):
        validator._validate_sealed_repository(
            repo.resolve(), evidence.resolve(), implementation, implementation,
            allow_unsealed_repository=False,
        )
    outputs[seal_changes_key] = expected_tree

    outputs[("status", "--porcelain=v1", "--untracked-files=all")] = (
        " M tools/unsafe.py"
    )
    with pytest.raises(
        validator.EvidenceValidationError,
        match="worktree is not clean",
    ):
        validator._validate_sealed_repository(
            repo.resolve(), evidence.resolve(), implementation, implementation,
            allow_unsealed_repository=False,
        )

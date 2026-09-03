#!/usr/bin/env python3
"""Fail-closed V15.31B powered position/thermal acceptance collector.

The default CLI action is an offline dry run.  Live mode is deliberately
separate and requires both an exact environment gate and an explicit command
line token.  Even in live mode this process never opens CAN, serial, a motor
UDP port, or a worker API.  It observes commands emitted on the existing GUI
topic and verifies that the existing Router, collision proof, planned-path
proof, gravity authority and hardware feedback all agree before retaining
evidence.

The only command this process can originate is an emergency ``brake`` message
on ``/whole_arm/gui_command``.  The existing Router expands that message to all
fault domains.  This is a conservative superset of "brake the related domain"
and avoids bypassing the Router with a direct worker socket.

Position motion is never synthesized here.  The GUI remains responsible for
planned-twin preview, collision checking, explicit submission and command/1.3
publication.  This runner publishes the next expected endpoint on its status
topic and merely collects/verifies the resulting execution.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import secrets
import stat
import subprocess
import sys
import tempfile
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Mapping, Optional, Sequence


EXPECTED_BRANCH = "agent/v15-31b-ft-powered-position-thermal-validation"
ENVELOPE_SCHEMA = "go-m8010-empirical-validation-envelope/1.0"
ANCHOR_VALIDATION_SCHEMA = "V15.31B-anchor-validation-v1"
HARDWARE_STATE_SCHEMA = "go-m8010-hardware-state/1.1"
GRAVITY_STATUS_SCHEMA = "go-m8010-gravity-status/1.1"
ROUTER_STATUS_SCHEMA = "go-m8010-command-router-status/1.0"
CONFIRMATION_SCHEMA = "go-m8010-empirical-stage-confirmation/1.0"
GUI_COMMAND_SCHEMA = "go-m8010-gui-command/1.3"
CONTROL_SCHEMA = "go-m8010-v15-31b-acceptance-control/1.0"
STATUS_SCHEMA = "go-m8010-v15-31b-acceptance-status/1.0"
PRODUCTION_MODEL_SHA256 = (
    "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
)
GRAVITY_CONFIG_SHA256 = (
    "307469b8384fd35547327ba1d5f80aa440e6b9663406ab7c9d9469bea263335d"
)
THERMAL_CONFIG_SHA256 = (
    "1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467"
)

JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
GO_MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5")
POSITION_ORDER = ("J1", "J6", "J5", "J4", "J3", "J2")
POSITION_PHASES = (
    "CENTER_START",
    "PLUS_5",
    "CENTER_AFTER_PLUS",
    "MINUS_5",
    "CENTER_FINAL",
)
PHASE_OFFSET_DEG = (0.0, 5.0, 0.0, -5.0, 0.0)
MOTOR_BY_JOINT = {
    "J1": ("J1",),
    "J2": ("J2A", "J2B"),
    "J3": ("J3",),
    "J4": ("J4",),
    "J5": ("J5",),
    "J6": ("J6",),
}
DOMAIN_BY_JOINT = {
    "J1": "J1",
    "J2": "J2",
    "J3": "J345",
    "J4": "J345",
    "J5": "J345",
    "J6": "J6",
}

PHYSICAL_CONFIRMATION_ENV = "V15_31B_PHYSICAL_CONFIRMATION"
PHYSICAL_CONFIRMATION_GATE = (
    "24V_ON=YES;SUPPORT_RELIABLE=YES;"
    "WHOLE_ARM_VERTICAL_INITIALIZATION_POSE=YES;ARM_STATIONARY=YES;"
    "NOT_AT_MECHANICAL_LIMIT=YES;PLUS_MINUS_5_DEG_CLEAR=YES;"
    "NO_PERSON_TOUCHING=YES;OPERATOR_STOP_READY=YES"
)
LIVE_ENABLE_TOKEN = "I_UNDERSTAND_THIS_OBSERVES_POWERED_MOTION"

MAXIMUM_CONFIRMATION_AGE_NS = 30_000_000_000
MAXIMUM_STATE_AGE_NS = 250_000_000
MAXIMUM_GRAVITY_AGE_NS = 250_000_000
MAXIMUM_STREAM_GAP_NS = 2_000_000_000
MAXIMUM_SEGMENT_NS = 15_000_000_000
MAXIMUM_POSITION_TOTAL_NS = 600_000_000_000
POSITION_BUDGET_FIELD = (
    "maximum_cumulative_accepted_trajectory_seconds"
)
ENDPOINT_DWELL_NS = 500_000_000
MAXIMUM_ENDPOINT_SAMPLE_GAP_NS = 100_000_000
MAXIMUM_COMPARISON_SAMPLE_GAP_NS = 100_000_000
MAXIMUM_GRAVITY_LADDER_SAMPLE_GAP_NS = 100_000_000
MAXIMUM_GRAVITY_WORKER_ECHO_LAG_NS = 300_000_000
MAXIMUM_GRAVITY_HARDWARE_PAIR_WAIT_NS = 250_000_000
MAXIMUM_GRAVITY_HARDWARE_CACHE_ENTRIES = 64
MINIMUM_HALF_SECOND_SAMPLE_COUNT = 6
GRAVITY_LADDER_LEVELS = (0.0, 0.25, 0.5, 0.75, 1.0)
GRAVITY_LADDER_MINIMUM_RAMP_NS = 2_000_000_000
GRAVITY_LADDER_MINIMUM_HOLD_NS = 5_000_000_000
GRAVITY_LADDER_MAXIMUM_HOLD_NS = 10_000_000_000
MINIMUM_RELATIVE_PD_IMPROVEMENT = 0.10
MINIMUM_ABSOLUTE_PD_IMPROVEMENT_NM = 0.05
MAXIMUM_STABLE_THERMAL_SLOPE_C_PER_MIN = 0.5
MINIMUM_THERMAL_SLOPE_IMPROVEMENT_C_PER_MIN = 0.05
MINIMUM_EARLY_FUNDAMENTAL_SLOPE_C_PER_MIN = 0.25
MINIMUM_EARLY_FUNDAMENTAL_RISE_C = 1.0
ENDPOINT_ERROR_DEG = 0.5
POSITION_DISPLACEMENT_DEG = 5.0
J2_SYNC_WARNING_DEG = 0.25
J2_SYNC_HARD_DEG = 0.5
HARD_THERMAL_STOP_C = 60.0
THERMAL_STAGE_SECONDS = {5: 300.0, 15: 900.0, 30: 1800.0}
THERMAL_SAMPLE_PERIOD_NS = 1_000_000_000
GO_GEAR_RATIO = 6.329999923706055
STANDARD_GRAVITY_M_S2 = 9.80665
MECHANICAL_UNLOAD_FRACTION = 0.20

POSITION_FIELDS = (
    "timestamp_utc", "monotonic_ns", "receipt_monotonic_ns",
    "test_id", "joint", "revision",
    "phase", "target_deg", "actual_deg", "error_deg",
    "endpoint_dwell_s", "j2_e_sync_deg", "temperature_c", "merror",
    "communication_ok", "controller_mode", "status", "reason",
    "envelope_id", "envelope_sha256", "session_id", "state_instance_id",
    "anchor_sha256", "trajectory_sha256", "trajectory_duration_ns",
    "trajectory_execute_at_monotonic_ns", "plan_token_id",
    "plan_recipe_sha256", "collision_proof_sha256",
    "hardware_state_sha256", "related_domain",
    "endpoint_dwell_trace_json", "endpoint_dwell_trace_sha256",
    "segment_execution_trace_json", "segment_execution_trace_sha256",
    "hardware_state_sequence", "hardware_state_source_monotonic_ns",
    "gui_command_sequence", "gui_command_source_instance_id",
    "gui_command_source_monotonic_ns",
    "actual_center_start_deg", "actual_displacement_from_center_deg",
    "minimum_required_actual_displacement_deg",
)
J2_COMPARISON_FIELDS = (
    "timestamp_utc", "monotonic_ns", "receipt_monotonic_ns",
    "condition", "window_elapsed_s",
    "position_error_deg", "j2_e_sync_deg",
    "j2a_tau_feedback_rotor_nm", "j2b_tau_feedback_rotor_nm",
    "j2a_pd_rotor_nm", "j2b_pd_rotor_nm",
    "j2a_gravity_ff_rotor_nm", "j2b_gravity_ff_rotor_nm",
    "j2a_total_command_rotor_nm", "j2b_total_command_rotor_nm",
    "j2a_temperature_c", "j2b_temperature_c",
    "saturation_observed", "internal_opposition_detected", "status", "reason",
    "envelope_id", "envelope_sha256", "session_id", "state_instance_id",
    "anchor_sha256", "hardware_state_sha256",
    "target_j2_rad", "frozen_pose_rad_json", "frozen_pose_sha256",
    "frozen_initial_hardware_state_sha256",
    "actual_pose_rad_json", "maximum_pose_error_deg",
    "hardware_state_sequence", "hardware_state_source_monotonic_ns",
)
THERMAL_FIELDS = (
    "timestamp_utc", "monotonic_ns", "receipt_monotonic_ns", "elapsed_s",
    "j2a_temperature_c", "j2b_temperature_c",
    "j2a_slope_c_per_min", "j2b_slope_c_per_min",
    "j2a_tau_feedback_rotor_nm", "j2b_tau_feedback_rotor_nm",
    "j2a_gravity_ff_rotor_nm", "j2b_gravity_ff_rotor_nm",
    "j2a_pd_rotor_nm", "j2b_pd_rotor_nm", "position_error_deg",
    "j2_e_sync_deg", "saturation_observed", "j2a_merror", "j2b_merror",
    "j2a_communication_ok", "j2b_communication_ok", "status", "reason",
    "envelope_id", "envelope_sha256", "session_id", "state_instance_id",
    "anchor_sha256", "hardware_state_sha256",
    "hardware_state_sequence", "hardware_state_source_monotonic_ns",
)
GRAVITY_SCALE_FIELDS = (
    "timestamp_utc", "monotonic_ns", "stage", "phase",
    "observer_receipt_monotonic_ns",
    "gravity_scale_target", "gravity_scale_applied", "phase_elapsed_s",
    "position_error_deg", "tau_feedback_rotor_nm",
    "pd_contribution_rotor_nm", "gravity_ff_contribution_rotor_nm",
    "total_command_rotor_nm", "temperature_c",
    "temperature_slope_c_per_min", "j2_e_sync_deg",
    "saturation_observed", "merror", "communication_ok",
    "operator_stop_available", "status", "reason",
    "envelope_id", "envelope_sha256", "session_id", "state_instance_id",
    "anchor_sha256", "gravity_source_instance_id",
    "gravity_status_sequence", "gravity_status_source_monotonic_ns",
    "hardware_state_sequence", "hardware_state_source_monotonic_ns",
    "node_feedforward_nm_json", "worker_feedforward_rotor_nm_json",
    "matched_node_feedforward_nm_json",
    "matched_expected_worker_feedforward_rotor_nm_json",
    "worker_echo_source_monotonic_ns_json",
    "worker_echo_match_gravity_status_sequence",
    "worker_echo_match_gravity_status_source_monotonic_ns",
    "worker_echo_lag_ms",
    "gravity_status_sha256", "hardware_state_sha256",
)
HASHED_ARTIFACTS = tuple(sorted((
    "power_on_readonly.json",
    "model_session_anchor_validation.json",
    "gravity_readonly_validation.json",
    "gravity_scale_validation.csv",
    "position_validation.csv",
    "j2_control_comparison.csv",
    "multi_joint_validation.json",
    "thermal_5min.csv",
    "thermal_15min.csv",
    "thermal_30min.csv",
    "thermal_summary.json",
    "gui_powered_validation.json",
    "final_result.json",
)))
EXACT_BUNDLE_FILES = HASHED_ARTIFACTS + ("SHA256SUMS",)
SUPPORTING_ARTIFACT_NAMES = (
    "power_on_readonly.json",
    "model_session_anchor_validation.json",
    "gravity_readonly_validation.json",
)
FINAL_FIELD_NAMES = (
    "V15.31B RESULT", "branch", "implementation commit", "evidence commit",
    "Power-on read-only", "Anchor", "Gravity calculation",
    "Gravity powered result", "J1 error", "J2 error", "J2 max sync",
    "J3 error", "J4 error", "J5 error", "J6 error", "multi-joint",
    "J2A", "J2B", "J2 sustained rotor torque", "saturation ratio",
    "thermal classification", "counterbalance required",
    "GUI powered result", "restore initial", "final brake",
    "motor zero modified", "MuJoCo modified", "git clean",
    "PRIMARY BLOCKER", "FINAL TASK RESULT", "READY_FOR_NEXT_STAGE",
)


class AcceptanceError(ValueError):
    """A bounded, machine-recordable acceptance failure."""


def _require(condition: bool, reason: str) -> None:
    if not condition:
        raise AcceptanceError(reason)


@dataclass(frozen=True)
class RunGitProvenance:
    """Read-only, independently captured Git state at powered-run startup."""

    repo_root: str
    output_directory: str
    branch: str
    run_base_commit: str
    upstream_ref: str
    upstream_commit: str
    ahead_count: int
    behind_count: int
    allowed_untracked_paths: tuple[str, ...]
    captured_at_utc: str

    def as_document(self) -> dict[str, Any]:
        return {
            "schema": "V15.31B-run-git-provenance-v1",
            "repo_root": self.repo_root,
            "output_directory": self.output_directory,
            "branch": self.branch,
            "run_base_commit": self.run_base_commit,
            "upstream_ref": self.upstream_ref,
            "upstream_commit": self.upstream_commit,
            "ahead_count": self.ahead_count,
            "behind_count": self.behind_count,
            "allowed_untracked_paths": list(self.allowed_untracked_paths),
            "captured_at_utc": self.captured_at_utc,
            "evidence_commit_semantics": (
                "EVIDENCE_RUN_BASE_COMMIT_NOT_SELF_REFERENTIAL_SEAL_COMMIT"
            ),
        }


def _git_output(repo_root: Path, *arguments: str, binary: bool = False) -> Any:
    try:
        completed = subprocess.run(
            ["git", *arguments],
            cwd=str(repo_root),
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            check=False,
            timeout=10.0,
        )
    except (OSError, subprocess.SubprocessError) as exc:
        raise AcceptanceError("GIT_PROVENANCE_COMMAND_UNAVAILABLE") from exc
    _require(completed.returncode == 0, "GIT_PROVENANCE_COMMAND_FAILED")
    if binary:
        return completed.stdout
    try:
        return completed.stdout.decode("utf-8", errors="strict").strip()
    except UnicodeDecodeError as exc:
        raise AcceptanceError("GIT_PROVENANCE_OUTPUT_NOT_UTF8") from exc


def capture_run_git_provenance(
    repo_root: Path, output_directory: Path,
) -> RunGitProvenance:
    """Capture branch/HEAD/upstream and the exact permitted evidence staging."""

    repo = repo_root.resolve()
    output = output_directory.resolve()
    _require(
        repo.is_dir() and not repo.is_symlink(),
        "GIT_PROVENANCE_REPO_ROOT_INVALID",
    )
    _require(
        output.is_dir()
        and not output.is_symlink()
        and output.is_relative_to(repo),
        "GIT_PROVENANCE_OUTPUT_DIRECTORY_INVALID",
    )
    output_entries = list(output.iterdir())
    regular_support = {
        path.name
        for path in output_entries
        if path.is_file() and not path.is_symlink()
    }
    _require(
        regular_support == set(SUPPORTING_ARTIFACT_NAMES)
        and {path.name for path in output_entries}
        == set(SUPPORTING_ARTIFACT_NAMES)
        and all(
            (output / name).is_file()
            and not (output / name).is_symlink()
            for name in SUPPORTING_ARTIFACT_NAMES
        ),
        "GIT_PROVENANCE_REQUIRES_EXACT_THREE_SUPPORTING_FILES",
    )
    top_level = Path(
        _git_output(repo, "rev-parse", "--show-toplevel")
    ).resolve()
    _require(top_level == repo, "GIT_PROVENANCE_REPO_ROOT_NOT_TOPLEVEL")
    branch = _git_output(repo, "branch", "--show-current")
    _require(branch == EXPECTED_BRANCH, "GIT_PROVENANCE_BRANCH_MISMATCH")
    head = _git_output(repo, "rev-parse", "HEAD")
    _require(bool(
        isinstance(head, str)
        and len(head) == 40
        and all(character in "0123456789abcdef" for character in head)
    ), "GIT_PROVENANCE_HEAD_INVALID")
    upstream_ref = _git_output(
        repo, "rev-parse", "--abbrev-ref", "--symbolic-full-name", "@{u}"
    )
    upstream_commit = _git_output(repo, "rev-parse", "@{u}")
    _require(
        isinstance(upstream_commit, str)
        and len(upstream_commit) == 40
        and all(
            character in "0123456789abcdef"
            for character in upstream_commit
        ),
        "GIT_PROVENANCE_UPSTREAM_HEAD_INVALID",
    )
    counts = _git_output(
        repo, "rev-list", "--left-right", "--count", "HEAD...@{u}"
    ).split()
    _require(
        len(counts) == 2 and all(item.isdecimal() for item in counts),
        "GIT_PROVENANCE_AHEAD_BEHIND_INVALID",
    )
    ahead, behind = (int(counts[0]), int(counts[1]))
    _require(
        head == upstream_commit and ahead == 0 and behind == 0,
        "GIT_PROVENANCE_NOT_SYNCED_WITH_UPSTREAM",
    )
    status_payload = _git_output(
        repo,
        "-c", "core.quotepath=false",
        "status", "--porcelain=v1", "-z", "--untracked-files=all",
        binary=True,
    )
    try:
        status_items = [
            item.decode("utf-8", errors="strict")
            for item in status_payload.split(b"\0") if item
        ]
    except UnicodeDecodeError as exc:
        raise AcceptanceError("GIT_PROVENANCE_STATUS_NOT_UTF8") from exc
    _require(
        all(item.startswith("?? ") for item in status_items),
        "GIT_PROVENANCE_TRACKED_OR_UNEXPECTED_CHANGE_PRESENT",
    )
    actual_untracked = {item[3:].replace("\\", "/") for item in status_items}
    expected_untracked = {
        (output / filename).relative_to(repo).as_posix()
        for filename in SUPPORTING_ARTIFACT_NAMES
    }
    _require(
        actual_untracked == expected_untracked,
        "GIT_PROVENANCE_UNTRACKED_SET_NOT_EXACT_SUPPORTING_FILES",
    )
    return RunGitProvenance(
        repo_root=str(repo),
        output_directory=str(output),
        branch=branch,
        run_base_commit=head,
        upstream_ref=upstream_ref,
        upstream_commit=upstream_commit,
        ahead_count=ahead,
        behind_count=behind,
        allowed_untracked_paths=tuple(sorted(actual_untracked)),
        captured_at_utc=_utc_text(),
    )


def _finite(value: object, reason: str) -> float:
    _require(type(value) in {int, float}, reason)
    result = float(value)
    _require(math.isfinite(result), reason)
    return result


def _finite_vector(value: object, length: int, reason: str) -> tuple[float, ...]:
    _require(isinstance(value, (list, tuple)) and len(value) == length, reason)
    return tuple(_finite(item, reason) for item in value)


def _expected_go_worker_feedforward(
    node_feedforward: tuple[float, ...],
) -> dict[str, float]:
    _require(
        len(node_feedforward) == 6
        and abs(node_feedforward[5]) <= 1.0e-12,
        "GRAVITY_LADDER_NODE_FF_SHAPE_OR_J6_INVALID",
    )
    return {
        "J1": node_feedforward[0],
        "J2A": -node_feedforward[1],
        "J2B": node_feedforward[1],
        "J3": node_feedforward[2],
        "J4": node_feedforward[3],
        "J5": node_feedforward[4],
    }


def _latest_go_worker_feedforward_echo_match(
    history: list[tuple[int, int, tuple[float, ...], dict[str, float]]],
    actual: Mapping[str, float],
    *,
    echo_source_ns_by_motor: Mapping[str, int],
) -> Optional[tuple[int, int, tuple[float, ...], dict[str, float]]]:
    """Match worker echoes to one prior node publication on the host clock.

    Receipt time is deliberately not accepted here.  Every GO worker's raw
    feedback source timestamp must be after the node publication and within
    the explicit 300 ms propagation bound.
    """

    for item in reversed(history):
        published_ns, _sequence, _node_ff, expected = item
        if any(
            motor not in echo_source_ns_by_motor
            or type(echo_source_ns_by_motor[motor]) is not int
            or not 0
            <= echo_source_ns_by_motor[motor] - published_ns
            <= MAXIMUM_GRAVITY_WORKER_ECHO_LAG_NS
            for motor in GO_MOTOR_NAMES
        ):
            continue
        if all(
            motor in actual
            and abs(float(actual[motor]) - expected[motor]) <= 1.0e-9
            for motor in GO_MOTOR_NAMES
        ):
            return item
    return None


def _worker_echo_propagation_pending(
    matched_echo: object, phase: str, phase_elapsed_ns: int,
) -> bool:
    return bool(
        matched_echo is None
        and phase == "RAMP"
        and 0 <= phase_elapsed_ns <= MAXIMUM_GRAVITY_WORKER_ECHO_LAG_NS
    )


def _valid_sha256(value: object) -> bool:
    return bool(
        isinstance(value, str)
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _valid_source_id(value: object) -> bool:
    return bool(
        isinstance(value, str)
        and len(value) == 32
        and all(character in "0123456789abcdef" for character in value)
    )


def _canonical_json_bytes(value: object) -> bytes:
    return json.dumps(
        value, ensure_ascii=True, allow_nan=False, sort_keys=True,
        separators=(",", ":"),
    ).encode("ascii")


def _document_sha256(value: object) -> str:
    return hashlib.sha256(_canonical_json_bytes(value)).hexdigest()


def _file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _utc_text() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def _rad_to_deg(value: float) -> float:
    return math.degrees(value)


def _exact_bool(value: Mapping[str, Any], field: str, reason: str) -> None:
    _require(value.get(field) is True, reason)


def validate_physical_startup_gate(environment: Mapping[str, str]) -> None:
    """Require the exact fresh physical gate before a live ROS node exists."""

    _require(
        environment.get(PHYSICAL_CONFIRMATION_ENV) == PHYSICAL_CONFIRMATION_GATE,
        "PHYSICAL_STARTUP_CONFIRMATION_MISSING_OR_MISMATCHED",
    )


@dataclass(frozen=True)
class EvidenceBinding:
    envelope_id: str
    envelope_sha256: str
    session_id: str
    state_instance_id: str
    anchor_sha256: str
    expires_at_utc: datetime
    maximum_position_seconds: float
    maximum_segment_seconds: float
    maximum_segment_displacement_deg: float
    maximum_stage_temperature_rise_c: float

    @classmethod
    def from_paths(
        cls,
        envelope_path: Path,
        anchor_validation_path: Path,
        expected_envelope_sha256: str,
    ) -> "EvidenceBinding":
        envelope_path = envelope_path.resolve()
        anchor_validation_path = anchor_validation_path.resolve()
        _require(envelope_path.is_file(), "EMPIRICAL_ENVELOPE_NOT_FOUND")
        _require(anchor_validation_path.is_file(), "ANCHOR_VALIDATION_NOT_FOUND")
        try:
            envelope_bytes = envelope_path.read_bytes()
            anchor_validation_bytes = anchor_validation_path.read_bytes()
        except OSError as exc:
            raise AcceptanceError("BINDING_FILE_READ_FAILED") from exc
        actual_envelope_sha = hashlib.sha256(envelope_bytes).hexdigest()
        _require(_valid_sha256(expected_envelope_sha256), "EXPECTED_ENVELOPE_SHA256_INVALID")
        _require(
            actual_envelope_sha == expected_envelope_sha256,
            "EMPIRICAL_ENVELOPE_FILE_SHA256_MISMATCH",
        )
        try:
            envelope = json.loads(envelope_bytes)
        except (UnicodeError, json.JSONDecodeError) as exc:
            raise AcceptanceError("BINDING_JSON_INVALID") from exc
        _require(isinstance(envelope, Mapping), "EMPIRICAL_ENVELOPE_INVALID")
        source_evidence = envelope.get("source_evidence")
        _require(
            isinstance(source_evidence, Mapping),
            "EMPIRICAL_SOURCE_EVIDENCE_MISSING",
        )
        anchor_source = source_evidence.get(
            "model_session_anchor_validation.json"
        )
        _require(
            isinstance(anchor_source, Mapping)
            and _valid_sha256(anchor_source.get("sha256")),
            "ANCHOR_VALIDATION_SOURCE_SHA256_INVALID",
        )
        actual_anchor_validation_sha = hashlib.sha256(
            anchor_validation_bytes
        ).hexdigest()
        _require(
            actual_anchor_validation_sha == anchor_source.get("sha256"),
            "ANCHOR_VALIDATION_FILE_SHA256_MISMATCH",
        )
        try:
            anchor_validation = json.loads(anchor_validation_bytes)
        except (UnicodeError, json.JSONDecodeError) as exc:
            raise AcceptanceError("BINDING_JSON_INVALID") from exc
        return cls.from_documents(
            envelope,
            anchor_validation,
            actual_envelope_sha,
        )

    @classmethod
    def from_documents(
        cls,
        envelope: object,
        anchor_validation: object,
        envelope_sha256: str,
    ) -> "EvidenceBinding":
        _require(isinstance(envelope, Mapping), "EMPIRICAL_ENVELOPE_INVALID")
        _require(envelope.get("schema") == ENVELOPE_SCHEMA, "EMPIRICAL_ENVELOPE_SCHEMA_MISMATCH")
        _require(envelope.get("single_use") is True, "EMPIRICAL_ENVELOPE_NOT_SINGLE_USE")
        _require(
            envelope.get("authority_class") == "EMPIRICAL_VALIDATION_ENVELOPE"
            and envelope.get("rating_classification")
            == "NOT_OFFICIAL_CONTINUOUS_RATING",
            "EMPIRICAL_AUTHORITY_CLASS_INVALID",
        )
        _require(_valid_sha256(envelope_sha256), "EMPIRICAL_ENVELOPE_SHA256_INVALID")
        envelope_id = envelope.get("envelope_id")
        _require(
            isinstance(envelope_id, str)
            and envelope_id.startswith("v15-31b-empirical-"),
            "EMPIRICAL_ENVELOPE_ID_INVALID",
        )
        binding = envelope.get("binding")
        _require(isinstance(binding, Mapping), "EMPIRICAL_BINDING_MISSING")
        session_id = binding.get("session_id")
        state_instance_id = binding.get("state_instance_id")
        anchor_sha = binding.get("anchor_sha256")
        _require(isinstance(session_id, str) and session_id, "EMPIRICAL_SESSION_ID_INVALID")
        _require(
            isinstance(state_instance_id, str) and state_instance_id,
            "EMPIRICAL_STATE_INSTANCE_ID_INVALID",
        )
        _require(_valid_sha256(anchor_sha), "EMPIRICAL_ANCHOR_SHA256_INVALID")
        _require(
            binding.get("model_sha256") == PRODUCTION_MODEL_SHA256
            and binding.get("gravity_config_sha256") == GRAVITY_CONFIG_SHA256
            and binding.get("thermal_config_sha256") == THERMAL_CONFIG_SHA256
            and binding.get("all_fields_must_match_runtime") is True,
            "EMPIRICAL_FROZEN_BINDING_MISMATCH",
        )
        position = envelope.get("position_validation")
        _require(isinstance(position, Mapping), "POSITION_VALIDATION_AUTHORITY_MISSING")
        _require(
            position.get("enabled") is True
            and position.get("unlock_requires_completed_gravity_ladder") is True
            and position.get("single_joint_first_required") is True
            and position.get("maximum_moving_joints_before_single_joint_pass") == 1
            and position.get("no_progress_watchdog_required_every_cycle") is True
            and position.get("temperature_required_every_cycle") is True
            and position.get("continuous_operation_authorized") is False,
            "POSITION_VALIDATION_AUTHORITY_INVALID",
        )
        maximum_position = _finite(
            position.get(POSITION_BUDGET_FIELD),
            "POSITION_MAXIMUM_TOTAL_INVALID",
        )
        maximum_segment = _finite(
            position.get("maximum_segment_seconds"),
            "POSITION_MAXIMUM_SEGMENT_INVALID",
        )
        maximum_displacement = _finite(
            position.get("maximum_abs_segment_displacement_deg"),
            "POSITION_MAXIMUM_DISPLACEMENT_INVALID",
        )
        _require(
            0.0 < maximum_position <= 600.0
            and 0.0 < maximum_segment <= 15.0
            and maximum_displacement == 5.0
            and _finite(position.get("endpoint_error_limit_deg"), "POSITION_ENDPOINT_ERROR_INVALID")
            == ENDPOINT_ERROR_DEG
            and _finite(position.get("endpoint_dwell_seconds"), "POSITION_ENDPOINT_DWELL_INVALID")
            == 0.5,
            "POSITION_VALIDATION_BOUNDS_MISMATCH",
        )
        live = envelope.get("live_gates")
        _require(isinstance(live, Mapping), "EMPIRICAL_LIVE_GATES_MISSING")
        temperature = live.get("temperature")
        operator = live.get("operator_stop")
        support = live.get("physical_support")
        sync = live.get("j2_sync")
        _require(isinstance(temperature, Mapping), "EMPIRICAL_TEMPERATURE_GATE_MISSING")
        maximum_rise = _finite(
            temperature.get("maximum_rise_within_one_stage_c"),
            "EMPIRICAL_STAGE_TEMPERATURE_RISE_INVALID",
        )
        _require(
            temperature.get("realtime_valid_required") is True
            and _finite(temperature.get("hard_stop_c"), "EMPIRICAL_HARD_STOP_INVALID")
            == HARD_THERMAL_STOP_C
            and temperature.get("rapid_rise_aborts_stage") is True
            and maximum_rise > 0.0,
            "EMPIRICAL_TEMPERATURE_GATE_INVALID",
        )
        _require(
            isinstance(operator, Mapping)
            and operator.get("must_remain_available") is True
            and _finite(operator.get("confirmation_maximum_age_seconds"), "OPERATOR_CONFIRMATION_AGE_INVALID")
            == 30.0,
            "EMPIRICAL_OPERATOR_GATE_INVALID",
        )
        _require(
            isinstance(support, Mapping)
            and support.get("j2_j3_reliable_support_required") is True,
            "EMPIRICAL_SUPPORT_GATE_INVALID",
        )
        _require(
            isinstance(sync, Mapping)
            and _finite(sync.get("warning_above_deg"), "J2_SYNC_WARNING_INVALID")
            == J2_SYNC_WARNING_DEG
            and _finite(sync.get("hard_above_deg"), "J2_SYNC_HARD_INVALID")
            == J2_SYNC_HARD_DEG,
            "EMPIRICAL_J2_SYNC_GATE_INVALID",
        )
        try:
            expires = datetime.fromisoformat(
                str(envelope.get("expires_at_utc", "")).replace("Z", "+00:00")
            )
        except ValueError as exc:
            raise AcceptanceError("EMPIRICAL_EXPIRY_INVALID") from exc
        _require(expires.tzinfo is not None, "EMPIRICAL_EXPIRY_TIMEZONE_MISSING")
        _require(isinstance(anchor_validation, Mapping), "ANCHOR_VALIDATION_INVALID")
        _require(
            anchor_validation.get("schema") == ANCHOR_VALIDATION_SCHEMA
            and anchor_validation.get("result") == "PASS",
            "ANCHOR_VALIDATION_NOT_PASS",
        )
        _require(
            anchor_validation.get("session_id") == session_id
            and anchor_validation.get("state_instance_id") == state_instance_id
            and anchor_validation.get("runtime_anchor_sha256") == anchor_sha,
            "ANCHOR_VALIDATION_BINDING_MISMATCH",
        )
        checks = anchor_validation.get("checks")
        _require(
            isinstance(checks, Mapping)
            and checks.get("session_match") is True
            and checks.get("state_instance_match") is True
            and checks.get("model_hash_match") is True
            and checks.get("motor_zero_modified") is False
            and checks.get("rid_modified") is False
            and checks.get("flash_or_eeprom_written") is False,
            "ANCHOR_VALIDATION_CHECK_FAILED",
        )
        return cls(
            envelope_id=envelope_id,
            envelope_sha256=envelope_sha256,
            session_id=session_id,
            state_instance_id=state_instance_id,
            anchor_sha256=anchor_sha,
            expires_at_utc=expires.astimezone(timezone.utc),
            maximum_position_seconds=maximum_position,
            maximum_segment_seconds=maximum_segment,
            maximum_segment_displacement_deg=maximum_displacement,
            maximum_stage_temperature_rise_c=maximum_rise,
        )

    def ensure_not_expired(self, now_utc: Optional[datetime] = None) -> None:
        checked = datetime.now(timezone.utc) if now_utc is None else now_utc
        _require(checked.tzinfo is not None, "CURRENT_TIME_TIMEZONE_MISSING")
        _require(
            checked.astimezone(timezone.utc) < self.expires_at_utc,
            "EMPIRICAL_ENVELOPE_EXPIRED",
        )


@dataclass
class ActiveSegment:
    joint: str
    phase: str
    target_rad: tuple[float, ...]
    trajectory_sha256: str
    plan_token_id: str
    recipe_sha256: str
    collision_proof_sha256: str
    accepted_monotonic_ns: int
    execute_at_monotonic_ns: int
    duration_ns: int
    gui_command_sequence: int
    gui_command_source_instance_id: str
    gui_command_source_monotonic_ns: int
    echo_observed: bool = False
    router_accepted: bool = False
    execution_trace: Optional[list[dict[str, Any]]] = None


@dataclass
class PostPositionExecution:
    kind: str
    start_rad: tuple[float, ...]
    target_rad: tuple[float, ...]
    commanded_rad: tuple[float, ...]
    started_monotonic_ns: int
    active_segment: Optional[ActiveSegment] = None
    awaiting_gui_command: bool = True
    endpoint_dwell_started_ns: Optional[int] = None
    endpoint_last_sample_ns: Optional[int] = None
    final_dwell_started_ns: Optional[int] = None
    final_last_sample_ns: Optional[int] = None
    recipe_sha256: Optional[str] = None
    manifest_segment_sha256: Optional[tuple[str, ...]] = None
    segment_records: Optional[list[dict[str, Any]]] = None
    maximum_error_deg: Optional[list[float]] = None
    endpoint_dwell_trace: Optional[list[dict[str, Any]]] = None
    final_dwell_trace: Optional[list[dict[str, Any]]] = None

    def __post_init__(self) -> None:
        if self.segment_records is None:
            self.segment_records = []
        if self.maximum_error_deg is None:
            self.maximum_error_deg = [0.0] * 6
        if self.endpoint_dwell_trace is None:
            self.endpoint_dwell_trace = []
        if self.final_dwell_trace is None:
            self.final_dwell_trace = []


@dataclass(frozen=True)
class FailureRecord:
    reason: str
    related_domains: tuple[str, ...]
    monotonic_ns: int
    brake_scope: str = "ALL_DOMAINS_FAIL_CLOSED_SUPERSET"


class ActiveAcceptanceRunner:
    """Pure state machine shared by offline tests/replay and the ROS wrapper."""

    def __init__(
        self,
        binding: EvidenceBinding,
        *,
        brake_callback: Optional[Callable[[FailureRecord], None]] = None,
        run_git_provenance: Optional[RunGitProvenance] = None,
    ) -> None:
        self.binding = binding
        self.brake_callback = brake_callback
        self.run_git_provenance = run_git_provenance
        self.preseal_git_provenance: Optional[RunGitProvenance] = None
        self.confirmation: Optional[dict[str, Any]] = None
        self.confirmation_by_target: dict[float, dict[str, Any]] = {}
        self.operator_confirmation_trace: list[dict[str, Any]] = []
        self._last_confirmation_by_source: dict[str, tuple[int, int]] = {}
        self._last_control_by_source: dict[str, tuple[int, int]] = {}
        self.latest_hardware: Optional[dict[str, Any]] = None
        self.latest_gravity: Optional[dict[str, Any]] = None
        self.latest_router: Optional[dict[str, Any]] = None
        self.latest_hardware_received_ns: Optional[int] = None
        self.latest_gravity_received_ns: Optional[int] = None
        self.latest_hardware_sequence: Optional[int] = None
        self.latest_hardware_source_ns: Optional[int] = None
        self.gravity_hardware_state_cache: dict[
            tuple[str, str, int, int], tuple[dict[str, Any], int]
        ] = {}
        self.pending_gravity_hardware_pairs: list[
            tuple[dict[str, Any], int]
        ] = []
        self.worker_supervisor_identity: Optional[tuple[str, int]] = None
        self.failure: Optional[FailureRecord] = None

        # The powered ladder is collected from the already authoritative
        # gravity/status + hardware/state streams.  It never drives a rung.
        self.gravity_ladder_active = False
        self.gravity_ladder_complete = False
        self.gravity_ladder_center_rad: Optional[tuple[float, ...]] = None
        self.gravity_ladder_rows: list[dict[str, Any]] = []
        self.gravity_ladder_stage_index = 0
        self.gravity_ladder_phase: Optional[str] = None
        self.gravity_ladder_phase_started_ns: Optional[int] = None
        self.gravity_ladder_last_sample_ns: Optional[int] = None
        self.gravity_ladder_last_applied = 0.0
        self.gravity_ladder_completed_levels: list[float] = []
        self.gravity_source_identity: Optional[str] = None
        self.gravity_source_last: Optional[tuple[int, int]] = None
        self.gravity_worker_echo_source_identity: Optional[str] = None
        self.gravity_worker_echo_source_last: Optional[tuple[int, int]] = None
        self.gravity_worker_echo_history: list[
            tuple[int, int, tuple[float, ...], dict[str, float]]
        ] = []

        self.position_started_ns: Optional[int] = None
        # The 600 s contract is a sum of every newly accepted POSITION
        # segment.  Immutable refreshes of an active segment do not consume
        # it again, but a later segment always does even if its reported
        # trajectory SHA-256 repeats.  Stationary HOLD, endpoint dwell and
        # thermal observation deliberately do not consume this budget.
        self.position_trajectory_budget_ns = 0
        self._budgeted_trajectory_sha256: set[str] = set()
        self.position_complete = False
        self.position_joint_index = 0
        self.position_phase_index = 0
        self.position_revision = {joint: "BASELINE" for joint in JOINT_NAMES}
        self.center_rad: Optional[tuple[float, ...]] = None
        self.position_actual_center_start_rad: dict[str, float] = {}
        self.endpoint_dwell_started_ns: Optional[int] = None
        self.endpoint_last_stable_sample_ns: Optional[int] = None
        self.endpoint_dwell_trace: list[dict[str, Any]] = []
        self.active_segment: Optional[ActiveSegment] = None
        self.awaiting_position_command = False
        self.position_rows: list[dict[str, Any]] = []
        self.post_execution: Optional[PostPositionExecution] = None
        self.completed_post_execution: dict[str, dict[str, Any]] = {}
        self.multi_joint_ui_attestation: Optional[dict[str, Any]] = None
        self.restore_ui_attestation: Optional[dict[str, Any]] = None
        self.thermal_setup_target_j2_rad: Optional[float] = None
        self.post_workflow_phase = "SINGLE_JOINT"
        self.completed_post_order: list[str] = []
        self.restore_completed_ns: Optional[int] = None
        self.restore_guard_hardware_state_sha256: Optional[str] = None

        self.comparison_condition: Optional[str] = None
        self.comparison_started_ns: Optional[int] = None
        self.comparison_started_source_ns: Optional[int] = None
        self.comparison_last_sample_ns: Optional[int] = None
        self.comparison_target_j2_rad: Optional[float] = None
        self.comparison_rows: list[dict[str, Any]] = []
        self.comparison_complete: set[str] = set()
        self.comparison_frozen_pose_rad: Optional[tuple[float, ...]] = None
        self.comparison_frozen_target_j2_rad: Optional[float] = None
        self.comparison_frozen_hardware_state_sha256: Optional[str] = None

        self.thermal_stage: Optional[int] = None
        self.thermal_started_ns: Optional[int] = None
        self.thermal_started_source_ns: Optional[int] = None
        self.thermal_target_j2_rad: Optional[float] = None
        self.thermal_last_sample_ns: Optional[int] = None
        self.thermal_rows: dict[int, list[dict[str, Any]]] = {5: [], 15: [], 30: []}
        self.thermal_pending_review: Optional[int] = None
        self.thermal_approved: set[int] = set()
        self.thermal_summary: Optional[dict[str, Any]] = None
        self.thermal_not_run_stages: dict[int, str] = {}

        self.multi_joint_document: Optional[dict[str, Any]] = None
        self.gui_document: Optional[dict[str, Any]] = None
        self.final_brake_requested = False
        self.final_brake_requested_ns: Optional[int] = None
        self.final_brake_observed = False
        self.final_brake_hardware_state_sha256: Optional[str] = None
        self.j6_disabled_raw_observed_ns: Optional[int] = None
        self.j6_disabled_raw_sha256: Optional[str] = None
        self.j6_disabled_raw_source_ns: Optional[int] = None
        self.j6_raw_source_instance_id: Optional[str] = None
        self.j6_raw_last_sequence: Optional[int] = None
        self.j6_raw_last_source_ns: Optional[int] = None
        self.j6_disabled_raw_sequence: Optional[int] = None
        self.final_brake_last_paired_raw_source_ns: Optional[int] = None
        self.final_brake_dwell_started_ns: Optional[int] = None
        self.final_brake_last_pair_ns: Optional[int] = None
        self.final_brake_pair_count = 0
        self.final_brake_dwell_seconds = 0.0
        self.final_brake_pairs: list[dict[str, Any]] = []
        self.final_result_document: Optional[dict[str, Any]] = None
        self.bundle_sealed = False
        self.bundle_manifest_sha256: Optional[str] = None
        self.shutdown_requested = False
        self.success_path_mode_events: list[dict[str, Any]] = []
        self.success_path_hardware_events: list[dict[str, Any]] = []

    @property
    def active(self) -> bool:
        return self.failure is None

    @property
    def expected_joint(self) -> Optional[str]:
        if self.position_complete or self.position_started_ns is None:
            return None
        return POSITION_ORDER[self.position_joint_index]

    @property
    def expected_phase(self) -> Optional[str]:
        if self.expected_joint is None:
            return None
        return POSITION_PHASES[self.position_phase_index]

    def _fresh_confirmation(
        self, now_ns: int, target_scale: Optional[float] = None,
    ) -> bool:
        confirmation = (
            self.confirmation
            if target_scale is None
            else self.confirmation_by_target.get(float(target_scale))
        )
        if confirmation is None:
            return False
        source_ns = confirmation.get("source_monotonic_ns")
        return bool(
            type(source_ns) is int
            and 0 <= now_ns - source_ns <= MAXIMUM_CONFIRMATION_AGE_NS
        )

    def observe_confirmation(self, value: object, *, now_ns: int) -> bool:
        """Observe the same strict confirmation consumed by the gravity node."""

        try:
            required = {
                "schema", "source_instance_id", "sequence",
                "source_monotonic_ns", "envelope_id", "envelope_sha256",
                "session_id", "state_instance_id", "target_gravity_scale",
                "operator_stop_ready", "j2_j3_support_reliable",
                "clearance_confirmed", "no_person_contact",
            }
            _require(isinstance(value, Mapping) and set(value) == required, "CONFIRMATION_FIELDS_INVALID")
            _require(value.get("schema") == CONFIRMATION_SCHEMA, "CONFIRMATION_SCHEMA_MISMATCH")
            source = value.get("source_instance_id")
            sequence = value.get("sequence")
            source_ns = value.get("source_monotonic_ns")
            _require(_valid_source_id(source), "CONFIRMATION_SOURCE_INVALID")
            _require(type(sequence) is int and sequence > 0, "CONFIRMATION_SEQUENCE_INVALID")
            _require(
                type(source_ns) is int
                and 0 < source_ns <= now_ns
                and now_ns - source_ns <= MAXIMUM_CONFIRMATION_AGE_NS,
                "CONFIRMATION_STALE",
            )
            previous = self._last_confirmation_by_source.get(source)
            _require(
                previous is None
                or (sequence > previous[0] and source_ns > previous[1]),
                "CONFIRMATION_REPLAYED",
            )
            _require(
                value.get("envelope_id") == self.binding.envelope_id
                and value.get("envelope_sha256") == self.binding.envelope_sha256
                and value.get("session_id") == self.binding.session_id
                and value.get("state_instance_id") == self.binding.state_instance_id,
                "CONFIRMATION_BINDING_MISMATCH",
            )
            target = _finite(
                value.get("target_gravity_scale"),
                "CONFIRMATION_TARGET_INVALID",
            )
            _require(
                target in GRAVITY_LADDER_LEVELS,
                "CONFIRMATION_TARGET_NOT_APPROVED_LADDER_LEVEL",
            )
            for field in (
                "operator_stop_ready", "j2_j3_support_reliable",
                "clearance_confirmed", "no_person_contact",
            ):
                _exact_bool(value, field, f"CONFIRMATION_{field.upper()}_FALSE")
            retained = dict(value)
            retained["observer_receipt_monotonic_ns"] = now_ns
            self.operator_confirmation_trace.append(retained)
            self._last_confirmation_by_source[source] = (sequence, source_ns)
            self.confirmation = dict(value)
            self.confirmation_by_target[target] = dict(value)
            return True
        except AcceptanceError:
            return False

    def validate_control(self, value: object, *, now_ns: int) -> dict[str, Any]:
        _require(isinstance(value, Mapping), "CONTROL_NOT_OBJECT")
        _require(value.get("schema") == CONTROL_SCHEMA, "CONTROL_SCHEMA_MISMATCH")
        source = value.get("source_instance_id")
        sequence = value.get("sequence")
        source_ns = value.get("source_monotonic_ns")
        _require(_valid_source_id(source), "CONTROL_SOURCE_INVALID")
        _require(type(sequence) is int and sequence > 0, "CONTROL_SEQUENCE_INVALID")
        _require(
            type(source_ns) is int
            and 0 < source_ns <= now_ns
            and now_ns - source_ns <= MAXIMUM_CONFIRMATION_AGE_NS,
            "CONTROL_STALE",
        )
        previous = self._last_control_by_source.get(source)
        _require(
            previous is None or (sequence > previous[0] and source_ns > previous[1]),
            "CONTROL_REPLAYED",
        )
        _require(
            value.get("envelope_id") == self.binding.envelope_id
            and value.get("envelope_sha256") == self.binding.envelope_sha256
            and value.get("session_id") == self.binding.session_id
            and value.get("state_instance_id") == self.binding.state_instance_id
            and value.get("anchor_sha256") == self.binding.anchor_sha256,
            "CONTROL_BINDING_MISMATCH",
        )
        self._last_control_by_source[source] = (sequence, source_ns)
        return dict(value)

    def _record_gravity_worker_echo_publication(
        self, value: Mapping[str, Any], *, now_ns: int,
    ) -> None:
        """Retain authoritative node publications by their source timestamp."""

        source = value.get("source_instance_id")
        sequence = value.get("sequence")
        source_ns = value.get("source_monotonic_ns")
        _require(
            _valid_source_id(source)
            and type(sequence) is int
            and sequence > 0
            and type(source_ns) is int
            and 0 < source_ns <= now_ns,
            "GRAVITY_WORKER_ECHO_PUBLICATION_IDENTITY_INVALID",
        )
        if self.gravity_worker_echo_source_identity is None:
            self.gravity_worker_echo_source_identity = source
            self.gravity_worker_echo_source_last = None
            self.gravity_worker_echo_history = []
        elif self.gravity_worker_echo_source_identity != source:
            _require(
                not self.gravity_ladder_active,
                "GRAVITY_LADDER_SOURCE_RESTARTED",
            )
            self.gravity_worker_echo_source_identity = source
            self.gravity_worker_echo_source_last = None
            self.gravity_worker_echo_history = []
        if self.gravity_worker_echo_source_last is not None:
            _require(
                sequence > self.gravity_worker_echo_source_last[0]
                and source_ns > self.gravity_worker_echo_source_last[1],
                "GRAVITY_WORKER_ECHO_PUBLICATION_REPLAYED",
            )
        node_ff = _finite_vector(
            value.get("feedforward_nm"), 6,
            "GRAVITY_LADDER_NODE_FEEDFORWARD_INVALID",
        )
        self.gravity_worker_echo_history.append(
            (
                source_ns,
                sequence,
                node_ff,
                _expected_go_worker_feedforward(node_ff),
            )
        )
        # A current gravity callback can be newer than the worker raw sample
        # embedded in its paired hardware snapshot.  Keep a bounded two-lag
        # history so a causally prior publication remains available.
        minimum_history_ns = (
            source_ns - 2 * MAXIMUM_GRAVITY_WORKER_ECHO_LAG_NS
        )
        self.gravity_worker_echo_history = [
            item for item in self.gravity_worker_echo_history
            if item[0] >= minimum_history_ns
        ][-256:]
        self.gravity_worker_echo_source_last = (sequence, source_ns)

    def _gravity_hardware_pair_key(
        self, gravity: Mapping[str, Any]
    ) -> tuple[str, str, int, int]:
        sequence = gravity.get("hardware_state_sequence")
        source_ns = gravity.get("hardware_state_source_monotonic_ns")
        _require(
            type(sequence) is int
            and sequence > 0
            and type(source_ns) is int
            and source_ns > 0,
            "GRAVITY_LADDER_HARDWARE_PAIR_IDENTITY_INVALID",
        )
        return (
            self.binding.session_id,
            self.binding.state_instance_id,
            sequence,
            source_ns,
        )

    def _cache_gravity_hardware_state(
        self, state: Mapping[str, Any], receipt_ns: int,
    ) -> None:
        key = (
            state["session_id"],
            state["state_instance_id"],
            state["sequence"],
            state["source_monotonic_ns"],
        )
        self.gravity_hardware_state_cache[key] = (dict(state), receipt_ns)
        while (
            len(self.gravity_hardware_state_cache)
            > MAXIMUM_GRAVITY_HARDWARE_CACHE_ENTRIES
        ):
            del self.gravity_hardware_state_cache[
                next(iter(self.gravity_hardware_state_cache))
            ]

    def _drain_gravity_hardware_pairs(self, *, now_ns: int) -> None:
        """Drain gravity statuses in source order only after exact pairing."""

        while self.pending_gravity_hardware_pairs:
            gravity, gravity_receipt_ns = (
                self.pending_gravity_hardware_pairs[0]
            )
            key = self._gravity_hardware_pair_key(gravity)
            matched = self.gravity_hardware_state_cache.get(key)
            if matched is None:
                for cached_key in self.gravity_hardware_state_cache:
                    if (
                        cached_key[:2] == key[:2]
                        and (
                            cached_key[2] == key[2]
                            or cached_key[3] == key[3]
                        )
                    ):
                        raise AcceptanceError(
                            "GRAVITY_LADDER_HARDWARE_PAIR_CONFLICT"
                        )
                break
            state, hardware_receipt_ns = matched
            gravity_source_ns = gravity["source_monotonic_ns"]
            hardware_source_ns = state["source_monotonic_ns"]
            _require(
                0 <= gravity_source_ns - hardware_source_ns
                <= MAXIMUM_STATE_AGE_NS,
                "GRAVITY_LADDER_HARDWARE_PAIR_SOURCE_TIME_INVALID",
            )
            self.pending_gravity_hardware_pairs.pop(0)
            self._capture_gravity_ladder(
                gravity,
                gravity_receipt_ns,
                state=state,
                hardware_receipt_ns=hardware_receipt_ns,
            )

    def observe_gravity_status(self, value: object, *, now_ns: int) -> None:
        try:
            _require(isinstance(value, Mapping), "GRAVITY_STATUS_NOT_OBJECT")
            _require(value.get("schema") == GRAVITY_STATUS_SCHEMA, "GRAVITY_STATUS_SCHEMA_MISMATCH")
            source_ns = value.get("source_monotonic_ns")
            _require(
                type(source_ns) is int
                and 0 < source_ns <= now_ns
                and now_ns - source_ns <= MAXIMUM_GRAVITY_AGE_NS,
                "GRAVITY_STATUS_STALE",
            )
            _require(
                value.get("session_id") == self.binding.session_id
                and value.get("state_instance_id") == self.binding.state_instance_id
                and value.get("model_sha256") == PRODUCTION_MODEL_SHA256
                and value.get("gravity_config_sha256") == GRAVITY_CONFIG_SHA256,
                "GRAVITY_STATUS_BINDING_MISMATCH",
            )
            empirical = value.get("empirical_validation")
            _require(isinstance(empirical, Mapping), "EMPIRICAL_GRAVITY_STATUS_MISSING")
            _require(
                value.get("anchor_valid") is True
                and value.get("finite_bounded") is True
                and value.get("empirical_validation_authoritative") is True
                and value.get("continuous_rotor_limits_authoritative") is False
                and empirical.get("authority_class") == "EMPIRICAL_VALIDATION_ENVELOPE"
                and empirical.get("rating_classification")
                == "NOT_OFFICIAL_CONTINUOUS_RATING"
                and empirical.get("envelope_id") == self.binding.envelope_id
                and empirical.get("envelope_sha256") == self.binding.envelope_sha256
                and empirical.get("anchor_sha256") == self.binding.anchor_sha256
                and empirical.get("invalidated") is False
                and empirical.get("continuous_operation_authorized") is False
                and empirical.get("official_continuous_rating_claimed") is False,
                "EMPIRICAL_GRAVITY_AUTHORITY_INVALID",
            )
            self._record_gravity_worker_echo_publication(
                value, now_ns=now_ns
            )
            self.latest_gravity = dict(value)
            self.latest_gravity_received_ns = now_ns
            if self.gravity_ladder_active:
                self.pending_gravity_hardware_pairs.append(
                    (dict(self.latest_gravity), now_ns)
                )
                _require(
                    len(self.pending_gravity_hardware_pairs)
                    <= MAXIMUM_GRAVITY_HARDWARE_CACHE_ENTRIES,
                    "GRAVITY_LADDER_HARDWARE_PAIR_QUEUE_FULL",
                )
                self._drain_gravity_hardware_pairs(now_ns=now_ns)
        except AcceptanceError as exc:
            if (
                self.gravity_ladder_active
                or self.position_started_ns is not None
                or self.post_execution is not None
                or self.comparison_condition is not None
                or self.thermal_stage is not None
            ):
                self.fail(str(exc), ("J1", "J2", "J345", "J6"), now_ns)

    def start_gravity_ladder(self, *, now_ns: int) -> None:
        """Begin passive collection of the authoritative 0/.25/.5/.75/1 ladder."""

        _require(self.failure is None, "RUNNER_ALREADY_FAILED")
        _require(not self.gravity_ladder_active, "GRAVITY_LADDER_ALREADY_ACTIVE")
        _require(not self.gravity_ladder_complete, "GRAVITY_LADDER_ALREADY_COMPLETE")
        _require(self.position_started_ns is None, "GRAVITY_LADDER_AFTER_POSITION_FORBIDDEN")
        _require(
            "WITHOUT_FF" in self.comparison_complete,
            "GRAVITY_LADDER_REQUIRES_COMPLETED_WITHOUT_FF_COMPARISON",
        )
        _require(self.latest_hardware is not None, "HARDWARE_STATE_MISSING")
        _require(
            self.latest_hardware_received_ns is not None
            and now_ns - self.latest_hardware_received_ns <= MAXIMUM_STATE_AGE_NS,
            "HARDWARE_STATE_RECEIPT_STALE",
        )
        _require(self._fresh_confirmation(now_ns), "OPERATOR_CONFIRMATION_STALE")
        modes = self.latest_hardware["controller_mode_by_motor"]
        _require(
            all(modes[motor] == "hold" for motor in MOTOR_NAMES),
            "GRAVITY_LADDER_REQUIRES_WHOLE_ARM_HOLD",
        )
        self.gravity_ladder_active = True
        self.gravity_ladder_center_rad = tuple(self.latest_hardware["position_rad"])
        self.gravity_ladder_rows = []
        self.gravity_ladder_stage_index = 0
        self.gravity_ladder_phase = None
        self.gravity_ladder_phase_started_ns = None
        self.gravity_ladder_last_sample_ns = None
        self.gravity_ladder_last_applied = 0.0
        self.gravity_ladder_completed_levels = []
        self.gravity_source_identity = None
        self.gravity_source_last = None

    def _capture_gravity_ladder(
        self,
        gravity: Mapping[str, Any],
        now_ns: int,
        *,
        state: Optional[Mapping[str, Any]] = None,
        hardware_receipt_ns: Optional[int] = None,
    ) -> None:
        if state is None:
            state = self.latest_hardware
        _require(state is not None, "GRAVITY_LADDER_HARDWARE_MISSING")
        if hardware_receipt_ns is None:
            hardware_receipt_ns = self.latest_hardware_received_ns
        _require(
            hardware_receipt_ns is not None
            and abs(now_ns - hardware_receipt_ns) <= MAXIMUM_STATE_AGE_NS,
            "GRAVITY_LADDER_HARDWARE_STALE",
        )
        source = gravity.get("source_instance_id")
        sequence = gravity.get("sequence")
        source_ns = gravity.get("source_monotonic_ns")
        _require(
            _valid_source_id(source)
            and type(sequence) is int and sequence > 0
            and type(source_ns) is int and 0 < source_ns <= now_ns,
            "GRAVITY_LADDER_SOURCE_IDENTITY_INVALID",
        )
        if self.gravity_source_identity is None:
            self.gravity_source_identity = source
        _require(source == self.gravity_source_identity, "GRAVITY_LADDER_SOURCE_RESTARTED")
        if self.gravity_source_last is not None:
            _require(
                sequence > self.gravity_source_last[0]
                and source_ns > self.gravity_source_last[1],
                "GRAVITY_LADDER_STATUS_REPLAYED",
            )
        self.gravity_source_last = (sequence, source_ns)
        _require(
            gravity.get("hardware_state_sequence") == state.get("sequence")
            and gravity.get("hardware_state_source_monotonic_ns")
            == state.get("source_monotonic_ns"),
            "GRAVITY_LADDER_STATUS_HARDWARE_PAIR_MISMATCH",
        )
        empirical = gravity.get("empirical_validation")
        _require(isinstance(empirical, Mapping), "EMPIRICAL_GRAVITY_STATUS_MISSING")
        stage_index = empirical.get("stage_index")
        stage_level = empirical.get("stage_level")
        _require(
            type(stage_index) is int
            and 0 <= stage_index < len(GRAVITY_LADDER_LEVELS)
            and _finite(stage_level, "GRAVITY_LADDER_STAGE_LEVEL_INVALID")
            == GRAVITY_LADDER_LEVELS[stage_index],
            "GRAVITY_LADDER_STAGE_IDENTITY_INVALID",
        )
        expected_index = len(self.gravity_ladder_completed_levels)
        if expected_index == len(GRAVITY_LADDER_LEVELS):
            return
        awaiting_interstage_confirmation = bool(
            expected_index > 0
            and stage_index == expected_index - 1
            and empirical.get("stage_complete") is True
        )
        _require(
            awaiting_interstage_confirmation
            or stage_index == expected_index,
            "GRAVITY_LADDER_STAGE_ORDER_INVALID",
        )
        level = GRAVITY_LADDER_LEVELS[stage_index]
        target = _finite(
            gravity.get("gravity_scale_target"),
            "GRAVITY_LADDER_TARGET_INVALID",
        )
        requested = _finite(
            gravity.get("gravity_scale_requested"),
            "GRAVITY_LADDER_REQUESTED_INVALID",
        )
        applied = _finite(
            gravity.get("gravity_scale"),
            "GRAVITY_LADDER_APPLIED_INVALID",
        )
        previous_level = 0.0 if stage_index == 0 else GRAVITY_LADDER_LEVELS[stage_index - 1]
        _require(
            abs(target - level) <= 1.0e-12
            and abs(requested - level) <= 1.0e-12
            and previous_level - 1.0e-9 <= applied <= level + 1.0e-9
            and applied + 1.0e-9 >= self.gravity_ladder_last_applied,
            "GRAVITY_LADDER_SCALE_NOT_BOUND_MONOTONIC",
        )
        _require(
            self._fresh_confirmation(
                now_ns, None if stage_index == 0 else level
            ),
            "OPERATOR_CONFIRMATION_STALE_OR_WRONG_LADDER_TARGET",
        )
        modes = state["controller_mode_by_motor"]
        _require(
            all(modes[motor] == "hold" for motor in MOTOR_NAMES),
            "GRAVITY_LADDER_NOT_WHOLE_ARM_CONTINUOUS_HOLD",
        )
        phase = (
            "AWAIT_INTERSTAGE_CONFIRMATION"
            if awaiting_interstage_confirmation
            else "HOLD" if abs(applied - level) <= 1.0e-6 else "RAMP"
        )
        sample_ns = source_ns
        if (
            stage_index > 0
            and phase == "HOLD"
            and self.gravity_ladder_phase != "HOLD"
        ):
            _require(
                self.gravity_ladder_phase == "RAMP"
                and self.gravity_ladder_phase_started_ns is not None
                and sample_ns - self.gravity_ladder_phase_started_ns
                >= GRAVITY_LADDER_MINIMUM_RAMP_NS,
                "GRAVITY_LADDER_RAMP_NOT_OBSERVED_FOR_2_SECONDS",
            )
        previous_phase = self.gravity_ladder_phase
        if phase != previous_phase:
            self.gravity_ladder_phase = phase
            self.gravity_ladder_phase_started_ns = sample_ns
            if (
                phase == "AWAIT_INTERSTAGE_CONFIRMATION"
                or previous_phase == "AWAIT_INTERSTAGE_CONFIRMATION"
            ):
                # Confirmation wait is continuous safety observation, not an
                # extension of the completed 5-10 s HOLD or the next rung's
                # sampling clock.
                self.gravity_ladder_last_sample_ns = None
        _require(self.gravity_ladder_phase_started_ns is not None, "GRAVITY_LADDER_PHASE_START_MISSING")
        if self.gravity_ladder_last_sample_ns is not None:
            _require(
                0 < sample_ns - self.gravity_ladder_last_sample_ns
                <= MAXIMUM_GRAVITY_LADDER_SAMPLE_GAP_NS,
                "GRAVITY_LADDER_SAMPLE_GAP_EXCEEDED_100MS",
            )
        self.gravity_ladder_last_sample_ns = sample_ns
        phase_elapsed_ns = (
            sample_ns - self.gravity_ladder_phase_started_ns
        )
        _require(
            phase != "HOLD" or phase_elapsed_ns <= GRAVITY_LADDER_MAXIMUM_HOLD_NS,
            "GRAVITY_LADDER_HOLD_EXCEEDED_10_SECONDS",
        )
        _require(
            phase != "AWAIT_INTERSTAGE_CONFIRMATION"
            or phase_elapsed_ns <= MAXIMUM_CONFIRMATION_AGE_NS,
            "GRAVITY_LADDER_INTERSTAGE_CONFIRMATION_EXCEEDED_30_SECONDS",
        )
        assert self.gravity_ladder_center_rad is not None
        position_error = max(
            abs(_rad_to_deg(actual - center))
            for actual, center in zip(
                state["position_rad"], self.gravity_ladder_center_rad
            )
        )
        per_motor = state["per_motor"]
        node_ff = _finite_vector(
            gravity.get("feedforward_nm"), 6,
            "GRAVITY_LADDER_NODE_FEEDFORWARD_INVALID",
        )
        actual_worker_ff = {
            motor: _finite(
                per_motor[motor].get("gravity_feedforward_rotor_nm"),
                f"GRAVITY_LADDER_{motor}_WORKER_FF_INVALID",
            )
            for motor in GO_MOTOR_NAMES
        }
        worker_echo_source_ns_by_motor: dict[str, int] = {}
        for motor in GO_MOTOR_NAMES:
            worker_source_ns = per_motor[motor].get(
                "feedback_source_monotonic_ns"
            )
            _require(
                type(worker_source_ns) is int
                and 0 < worker_source_ns <= state["source_monotonic_ns"],
                f"GRAVITY_LADDER_{motor}_WORKER_ECHO_SOURCE_INVALID",
            )
            worker_echo_source_ns_by_motor[motor] = worker_source_ns
        matched_echo = _latest_go_worker_feedforward_echo_match(
            self.gravity_worker_echo_history,
            actual_worker_ff,
            echo_source_ns_by_motor=worker_echo_source_ns_by_motor,
        )
        if _worker_echo_propagation_pending(
            matched_echo, phase, phase_elapsed_ns
        ):
            return
        _require(
            matched_echo is not None,
            "GRAVITY_LADDER_NODE_WORKER_FEEDFORWARD_ECHO_NOT_PROPAGATED_300MS",
        )
        assert matched_echo is not None
        (
            matched_published_ns,
            matched_sequence,
            matched_node_ff,
            matched_expected_worker_ff,
        ) = matched_echo
        _require(
            per_motor["J6"].get("gravity_feedforward_rotor_nm") is None
            and per_motor["J6"].get("tau_feedback_rotor_nm") is None
            and per_motor["J6"].get("tau_cmd_rotor_nm") is None,
            "GRAVITY_LADDER_J6_POS_VEL_TORQUE_SEMANTICS_INVALID",
        )
        feedback = [
            abs(_finite(per_motor[motor].get("tau_feedback_rotor_nm"), "GRAVITY_LADDER_FEEDBACK_INVALID"))
            for motor in GO_MOTOR_NAMES
        ]
        ff = [
            _finite(per_motor[motor].get("gravity_feedforward_rotor_nm"), "GRAVITY_LADDER_FF_INVALID")
            for motor in GO_MOTOR_NAMES
        ]
        command = [
            _finite(per_motor[motor].get("tau_cmd_rotor_nm"), "GRAVITY_LADDER_COMMAND_INVALID")
            for motor in GO_MOTOR_NAMES
        ]
        slopes = state.get("thermal_status_by_motor")
        _require(isinstance(slopes, Mapping), "GRAVITY_LADDER_THERMAL_STATUS_MISSING")
        temperature = max(
            float(per_motor[motor]["temperature_c"]) for motor in MOTOR_NAMES
        )
        stage_label = f"{int(level * 100)}%"
        stage_rows = [
            row for row in self.gravity_ladder_rows
            if row["stage"] == stage_label
        ]
        reported_slopes = [
            self._optional_metadata_number(slopes, motor, "slope_c_per_min")
            for motor in MOTOR_NAMES
        ]
        temperature_slope = max(
            [abs(self._derived_temperature_slope(
                stage_rows, sample_ns, temperature, "temperature_c"
            ))]
            + [abs(value) for value in reported_slopes if value is not None]
        )
        no_progress = state.get("no_progress_status_by_motor")
        saturation = bool(
            isinstance(no_progress, Mapping)
            and any(
                isinstance(no_progress.get(motor), Mapping)
                and no_progress[motor].get("software_saturation_observed") is True
                for motor in MOTOR_NAMES
            )
        )
        self.gravity_ladder_rows.append({
            "timestamp_utc": _utc_text(),
            "monotonic_ns": sample_ns,
            "observer_receipt_monotonic_ns": now_ns,
            "stage": stage_label,
            "phase": phase,
            "gravity_scale_target": target,
            "gravity_scale_applied": applied,
            "phase_elapsed_s": phase_elapsed_ns / 1.0e9,
            "position_error_deg": position_error,
            "tau_feedback_rotor_nm": max(feedback),
            "pd_contribution_rotor_nm": max(
                abs(total - contribution)
                for total, contribution in zip(command, ff)
            ),
            "gravity_ff_contribution_rotor_nm": max(abs(value) for value in ff),
            "total_command_rotor_nm": max(abs(value) for value in command),
            "temperature_c": temperature,
            "temperature_slope_c_per_min": temperature_slope,
            "j2_e_sync_deg": _rad_to_deg(state["j2_e_sync_rad"]),
            "saturation_observed": saturation,
            "merror": max(int(per_motor[motor]["merror"]) for motor in MOTOR_NAMES),
            "communication_ok": all(per_motor[motor]["communication_ok"] is True for motor in MOTOR_NAMES),
            "operator_stop_available": True,
            "status": "PASS",
            "reason": "",
            "envelope_id": self.binding.envelope_id,
            "envelope_sha256": self.binding.envelope_sha256,
            "session_id": self.binding.session_id,
            "state_instance_id": self.binding.state_instance_id,
            "anchor_sha256": self.binding.anchor_sha256,
            "gravity_source_instance_id": source,
            "gravity_status_sequence": sequence,
            "gravity_status_source_monotonic_ns": source_ns,
            "hardware_state_sequence": state["sequence"],
            "hardware_state_source_monotonic_ns": state["source_monotonic_ns"],
            "node_feedforward_nm_json": json.dumps(
                list(node_ff), separators=(",", ":")
            ),
            "worker_feedforward_rotor_nm_json": json.dumps(
                [actual_worker_ff[motor] for motor in GO_MOTOR_NAMES],
                separators=(",", ":"),
            ),
            "matched_node_feedforward_nm_json": json.dumps(
                list(matched_node_ff), separators=(",", ":")
            ),
            "matched_expected_worker_feedforward_rotor_nm_json": json.dumps(
                [
                    matched_expected_worker_ff[motor]
                    for motor in GO_MOTOR_NAMES
                ],
                separators=(",", ":"),
            ),
            "worker_echo_source_monotonic_ns_json": json.dumps(
                [
                    worker_echo_source_ns_by_motor[motor]
                    for motor in GO_MOTOR_NAMES
                ],
                separators=(",", ":"),
            ),
            "worker_echo_match_gravity_status_sequence": matched_sequence,
            "worker_echo_match_gravity_status_source_monotonic_ns": (
                matched_published_ns
            ),
            "worker_echo_lag_ms": (
                max(worker_echo_source_ns_by_motor.values())
                - matched_published_ns
            ) / 1.0e6,
            "gravity_status_sha256": _document_sha256(gravity),
            "hardware_state_sha256": _document_sha256(state),
        })
        self.gravity_ladder_last_applied = applied
        if (
            empirical.get("stage_complete") is True
            and not awaiting_interstage_confirmation
        ):
            _require(phase == "HOLD", "GRAVITY_LADDER_COMPLETE_OUTSIDE_HOLD")
            _require(
                GRAVITY_LADDER_MINIMUM_HOLD_NS
                <= phase_elapsed_ns
                <= GRAVITY_LADDER_MAXIMUM_HOLD_NS,
                "GRAVITY_LADDER_HOLD_NOT_5_TO_10_SECONDS",
            )
            hold_rows = [
                row for row in self.gravity_ladder_rows
                if row["stage"] == f"{int(level * 100)}%"
                and row["phase"] == "HOLD"
            ]
            _require(
                len(hold_rows)
                >= math.ceil(GRAVITY_LADDER_MINIMUM_HOLD_NS / MAXIMUM_GRAVITY_LADDER_SAMPLE_GAP_NS) + 1,
                "GRAVITY_LADDER_HOLD_SAMPLE_COUNT_INSUFFICIENT",
            )
            self.gravity_ladder_completed_levels.append(level)
            self.gravity_ladder_phase = None
            self.gravity_ladder_phase_started_ns = None
            self.gravity_ladder_last_sample_ns = None
            if len(self.gravity_ladder_completed_levels) == len(GRAVITY_LADDER_LEVELS):
                self.gravity_ladder_active = False
                self.gravity_ladder_complete = True

    def _require_full_gravity_position_authority(self, now_ns: int) -> Mapping[str, Any]:
        _require(self.latest_gravity is not None, "GRAVITY_STATUS_MISSING")
        _require(
            self.latest_gravity_received_ns is not None
            and now_ns - self.latest_gravity_received_ns <= MAXIMUM_GRAVITY_AGE_NS,
            "GRAVITY_STATUS_RECEIPT_STALE",
        )
        gravity = self.latest_gravity
        empirical = gravity.get("empirical_validation")
        _require(isinstance(empirical, Mapping), "EMPIRICAL_GRAVITY_STATUS_MISSING")
        _require(
            empirical.get("phase") == "POSITION_VALIDATION"
            and empirical.get("position_validation_authorized") is True
            and empirical.get("stage_index") == 4
            and empirical.get("stage_level") == 1.0
            and empirical.get("stage_complete") is True
            and gravity.get("gravity_scale_target") == 1.0
            and abs(_finite(gravity.get("gravity_scale"), "GRAVITY_SCALE_INVALID") - 1.0)
            <= 1.0e-6
            and gravity.get("hardware_tff_enabled") is True,
            "FULL_GRAVITY_POSITION_AUTHORITY_NOT_READY",
        )
        return gravity

    def observe_router_status(self, value: object, *, now_ns: int) -> None:
        if not isinstance(value, Mapping) or value.get("schema") != ROUTER_STATUS_SCHEMA:
            return
        self.latest_router = dict(value)
        segment = self.active_segment
        if segment is None and self.post_execution is not None:
            segment = self.post_execution.active_segment
        if segment is None:
            return
        moving_mask = value.get("last_moving_joint_mask")
        expected_mask = [False] * 6
        expected_mask[JOINT_NAMES.index(segment.joint)] = True
        if (
            value.get("last_mode") == "position"
            and moving_mask == expected_mask
            and _finite(value.get("last_command_age_ms"), "ROUTER_COMMAND_AGE_INVALID")
            <= 1000.0
        ):
            segment.router_accepted = True

    def _planned_proof_for_trajectory(self, trajectory_sha256: str) -> Mapping[str, Any]:
        _require(self.latest_gravity is not None, "PLANNED_PATH_PROOF_STATUS_MISSING")
        proof = self.latest_gravity.get("planned_trajectory_feasibility")
        _require(isinstance(proof, Mapping), "PLANNED_PATH_PROOF_MISSING")
        _require(
            proof.get("schema")
            == "go-m8010-empirical-planned-load-thermal-feasibility/1.0"
            and proof.get("result") == "PASS"
            and proof.get("load_feasibility") == "PASS"
            and proof.get("thermal_feasibility") == "PASS"
            and proof.get("trajectory_sha256") == trajectory_sha256
            and proof.get("session_id") == self.binding.session_id
            and proof.get("state_instance_id") == self.binding.state_instance_id
            and proof.get("model_sha256") == PRODUCTION_MODEL_SHA256
            and proof.get("gravity_config_sha256") == GRAVITY_CONFIG_SHA256
            and proof.get("thermal_config_sha256") == THERMAL_CONFIG_SHA256
            and proof.get("empirical_validation_authoritative") is True
            and proof.get("empirical_envelope_id") == self.binding.envelope_id
            and proof.get("empirical_envelope_sha256") == self.binding.envelope_sha256,
            "PLANNED_PATH_PROOF_NOT_BOUND_PASS",
        )
        return proof

    def observe_gui_command(self, value: object, *, now_ns: int) -> None:
        """Observe GUI output; never mint or repair a motion command here."""

        if self.failure is not None or not isinstance(value, Mapping):
            return
        mode = value.get("mode")
        workflow_active = bool(
            self.comparison_condition is not None
            or self.position_started_ns is not None
            or self.post_execution is not None
            or self.thermal_stage is not None
            or self.thermal_pending_review is not None
        )
        if workflow_active:
            self.success_path_mode_events.append({
                "monotonic_ns": now_ns,
                "mode": mode,
                "final_brake_requested": self.final_brake_requested,
            })
        if (
            workflow_active
            and mode in {"brake", "drag"}
            and not self.final_brake_requested
        ):
            self.fail(
                "SUCCESS_PATH_UNEXPECTED_BRAKE_OR_DRAG",
                ("J1", "J2", "J345", "J6"),
                now_ns,
            )
            return
        if mode != "position":
            return
        try:
            _require(self.position_started_ns is not None, "POSITION_COMMAND_BEFORE_RUNNER_START")
            if self.position_complete:
                self._observe_post_position_command(value, now_ns)
                return
            if self.active_segment is not None and not self.awaiting_position_command:
                self._validate_exact_active_refresh(value, now_ns)
                return
            _require(self.awaiting_position_command, "UNEXPECTED_POSITION_COMMAND_WHILE_SEGMENT_ACTIVE")
            self.binding.ensure_not_expired()
            self._require_full_gravity_position_authority(now_ns)
            _require(self._fresh_confirmation(now_ns, 1.0), "OPERATOR_CONFIRMATION_STALE")
            _require(value.get("schema") == GUI_COMMAND_SCHEMA, "POSITION_COMMAND_SCHEMA_NOT_1_3")
            _require(_valid_source_id(value.get("source_instance_id")), "POSITION_COMMAND_SOURCE_INVALID")
            _require(
                type(value.get("sequence")) is int
                and value["sequence"] > 0,
                "POSITION_COMMAND_SEQUENCE_INVALID",
            )
            source_ns = value.get("source_monotonic_ns")
            _require(
                type(source_ns) is int
                and 0 < source_ns <= now_ns
                and now_ns - source_ns <= 250_000_000,
                "POSITION_COMMAND_STALE",
            )
            joint = self.expected_joint
            phase = self.expected_phase
            _require(joint is not None and phase is not None and phase != "CENTER_START", "POSITION_PHASE_NOT_COMMANDABLE")
            joint_index = JOINT_NAMES.index(joint)
            expected_mask = [False] * 6
            expected_mask[joint_index] = True
            _require(value.get("moving_joint_mask") == expected_mask, "POSITION_MOVING_MASK_MISMATCH")
            _require(value.get("active_joint_mask") == [True] * 6, "POSITION_ACTIVE_MASK_MISMATCH")
            targets = _finite_vector(value.get("targets_rad"), 6, "POSITION_TARGETS_INVALID")
            _require(self.center_rad is not None, "POSITION_CENTER_NOT_CAPTURED")
            expected_targets = list(self.center_rad)
            expected_targets[joint_index] += math.radians(PHASE_OFFSET_DEG[self.position_phase_index])
            _require(
                all(abs(actual - expected) <= 1.0e-6 for actual, expected in zip(targets, expected_targets)),
                "POSITION_TARGET_SEQUENCE_MISMATCH",
            )
            trajectory = value.get("trajectory")
            _require(isinstance(trajectory, Mapping), "POSITION_TRAJECTORY_MISSING")
            trajectory_sha = trajectory.get("trajectory_sha256")
            plan_token = value.get("plan_token_id")
            _require(_valid_sha256(trajectory_sha), "POSITION_TRAJECTORY_SHA256_INVALID")
            _require(_valid_sha256(plan_token), "POSITION_PLAN_TOKEN_INVALID")
            _require(
                trajectory.get("schema") == "go-m8010-quintic-command/1.0"
                and trajectory.get("profile") == "quintic-rest-to-rest-v1",
                "POSITION_TRAJECTORY_PROFILE_INVALID",
            )
            trajectory_start = _finite_vector(trajectory.get("start_rad"), 6, "POSITION_TRAJECTORY_START_INVALID")
            trajectory_target = _finite_vector(trajectory.get("target_rad"), 6, "POSITION_TRAJECTORY_TARGET_INVALID")
            _require(
                all(abs(actual - expected) <= 1.0e-9 for actual, expected in zip(trajectory_target, targets)),
                "POSITION_TRAJECTORY_TARGET_COMMAND_MISMATCH",
            )
            changed = [
                index for index, (start, target) in enumerate(zip(trajectory_start, trajectory_target))
                if abs(target - start) > math.radians(0.01)
            ]
            _require(changed == [joint_index], "TRAJECTORY_MOVES_MORE_THAN_ONE_JOINT")
            displacement_deg = abs(_rad_to_deg(trajectory_target[joint_index] - trajectory_start[joint_index]))
            _require(
                displacement_deg <= self.binding.maximum_segment_displacement_deg + 1.0e-6,
                "POSITION_SEGMENT_EXCEEDS_5_DEG",
            )
            duration_ns = trajectory.get("duration_ns")
            execute_at_ns = trajectory.get("execute_at_monotonic_ns")
            _require(
                type(duration_ns) is int
                and 0 < duration_ns <= int(self.binding.maximum_segment_seconds * 1.0e9)
                and duration_ns <= MAXIMUM_SEGMENT_NS,
                "POSITION_SEGMENT_EXCEEDS_15_SECONDS",
            )
            _require(type(execute_at_ns) is int and execute_at_ns > 0, "POSITION_EXECUTE_TIME_INVALID")
            manifest = value.get("plan_manifest")
            _require(isinstance(manifest, Mapping), "POSITION_PLAN_MANIFEST_MISSING")
            recipe_sha = manifest.get("recipe_sha256")
            segment_shas = manifest.get("segment_sha256")
            _require(
                manifest.get("schema") == "go-m8010-plan-manifest/1.0"
                and _valid_sha256(recipe_sha)
                and isinstance(segment_shas, list)
                and segment_shas
                and all(_valid_sha256(item) for item in segment_shas)
                and trajectory_sha in segment_shas,
                "POSITION_PLAN_MANIFEST_INVALID",
            )
            proof = value.get("collision_guard_proof")
            _require(isinstance(proof, Mapping), "POSITION_COLLISION_PROOF_MISSING")
            _require(
                proof.get("schema") == "go-m8010-collision-guard-result/1.0"
                and proof.get("safe") is True
                and proof.get("session_id") == self.binding.session_id
                and proof.get("state_instance_id") == self.binding.state_instance_id
                and proof.get("moving_joint_mask") == expected_mask
                and proof.get("model_sha256") == PRODUCTION_MODEL_SHA256,
                "POSITION_COLLISION_PROOF_INVALID",
            )
            self._planned_proof_for_trajectory(trajectory_sha)
            next_budget = self.position_trajectory_budget_ns + duration_ns
            _require(
                next_budget
                <= min(
                    MAXIMUM_POSITION_TOTAL_NS,
                    int(self.binding.maximum_position_seconds * 1.0e9),
                ),
                "POSITION_TRAJECTORY_BUDGET_EXCEEDED_600_SECONDS",
            )
            self.position_trajectory_budget_ns = next_budget
            self._budgeted_trajectory_sha256.add(trajectory_sha)
            self.active_segment = ActiveSegment(
                joint=joint,
                phase=phase,
                target_rad=targets,
                trajectory_sha256=trajectory_sha,
                plan_token_id=plan_token,
                recipe_sha256=recipe_sha,
                collision_proof_sha256=_document_sha256(proof),
                accepted_monotonic_ns=now_ns,
                execute_at_monotonic_ns=execute_at_ns,
                duration_ns=duration_ns,
                gui_command_sequence=value["sequence"],
                gui_command_source_instance_id=value["source_instance_id"],
                gui_command_source_monotonic_ns=source_ns,
            )
            self.awaiting_position_command = False
            self.endpoint_dwell_started_ns = None
            self.endpoint_last_stable_sample_ns = None
        except AcceptanceError as exc:
            domain = DOMAIN_BY_JOINT.get(self.expected_joint or "", "J1")
            self.fail(str(exc), (domain,), now_ns)

    def _validate_exact_active_refresh(
        self, value: Mapping[str, Any], now_ns: int,
    ) -> None:
        """Permit only the Router's immutable refresh of the current segment."""

        segment = self.active_segment
        assert segment is not None
        source_ns = value.get("source_monotonic_ns")
        _require(
            value.get("schema") == GUI_COMMAND_SCHEMA
            and _valid_source_id(value.get("source_instance_id"))
            and type(source_ns) is int
            and 0 < source_ns <= now_ns
            and now_ns - source_ns <= 250_000_000,
            "POSITION_REFRESH_SOURCE_OR_TIME_INVALID",
        )
        trajectory = value.get("trajectory")
        manifest = value.get("plan_manifest")
        proof = value.get("collision_guard_proof")
        joint_index = JOINT_NAMES.index(segment.joint)
        expected_mask = [False] * 6
        expected_mask[joint_index] = True
        _require(
            value.get("moving_joint_mask") == expected_mask
            and value.get("active_joint_mask") == [True] * 6
            and tuple(_finite_vector(value.get("targets_rad"), 6, "POSITION_REFRESH_TARGET_INVALID"))
            == segment.target_rad
            and value.get("plan_token_id") == segment.plan_token_id
            and isinstance(trajectory, Mapping)
            and trajectory.get("trajectory_sha256") == segment.trajectory_sha256
            and trajectory.get("execute_at_monotonic_ns") == segment.execute_at_monotonic_ns
            and trajectory.get("duration_ns") == segment.duration_ns
            and isinstance(manifest, Mapping)
            and manifest.get("recipe_sha256") == segment.recipe_sha256
            and isinstance(proof, Mapping)
            and _document_sha256(proof) == segment.collision_proof_sha256,
            "POSITION_REFRESH_MUTATED_IMMUTABLE_DESCRIPTOR",
        )

    def start_post_position_execution(
        self,
        kind: str,
        target_rad: Sequence[float],
        *,
        now_ns: int,
    ) -> None:
        """Arm an observed GUI execution after the six single-joint passes."""

        kind = kind.strip().upper()
        _require(
            kind in {"MULTI_JOINT", "THERMAL_SETUP", "RESTORE_INITIAL"},
            "POST_POSITION_KIND_INVALID",
        )
        _require(self.failure is None, "RUNNER_ALREADY_FAILED")
        _require(self.position_complete, "POST_POSITION_REQUIRES_SINGLE_JOINT_PASS")
        _require(self.post_execution is None, "POST_POSITION_EXECUTION_ALREADY_ACTIVE")
        _require(kind not in self.completed_post_execution, "POST_POSITION_KIND_ALREADY_COMPLETE")
        required_phase = {
            "MULTI_JOINT": "AWAIT_MULTI_JOINT",
            "THERMAL_SETUP": "AWAIT_THERMAL_SETUP",
            "RESTORE_INITIAL": "AWAIT_RESTORE_INITIAL",
        }[kind]
        _require(
            self.post_workflow_phase == required_phase,
            f"POST_POSITION_ORDER_REQUIRES_{required_phase}",
        )
        _require(self.latest_hardware is not None, "HARDWARE_STATE_MISSING")
        _require(self._fresh_confirmation(now_ns, 1.0), "OPERATOR_CONFIRMATION_STALE")
        self._require_full_gravity_position_authority(now_ns)
        start = tuple(float(item) for item in self.latest_hardware["position_rad"])
        target = _finite_vector(target_rad, 6, "POST_POSITION_TARGET_INVALID")
        _require(self.center_rad is not None, "POSITION_CENTER_NOT_CAPTURED")
        _require(
            all(
                abs(_rad_to_deg(value - center))
                <= POSITION_DISPLACEMENT_DEG + 1.0e-6
                for value, center in zip(target, self.center_rad)
            ),
            "POST_POSITION_TARGET_OUTSIDE_VERIFIED_PLUS_MINUS_5_DEG",
        )
        changed = [
            index for index, (begin, end) in enumerate(zip(start, target))
            if abs(end - begin) > math.radians(0.01)
        ]
        _require(changed, "POST_POSITION_TARGET_HAS_NO_REAL_MOTION")
        if kind == "MULTI_JOINT":
            _require(len(changed) >= 2, "MULTI_JOINT_TARGET_CHANGES_FEWER_THAN_TWO_JOINTS")
        if kind == "THERMAL_SETUP":
            _require(1 in changed, "THERMAL_SETUP_MUST_MOVE_J2")
        if kind == "RESTORE_INITIAL":
            _require(
                all(abs(value - center) <= 1.0e-6 for value, center in zip(target, self.center_rad)),
                "RESTORE_INITIAL_TARGET_NOT_SESSION_INITIAL_POSE",
            )
        self.post_execution = PostPositionExecution(
            kind=kind,
            start_rad=start,
            target_rad=target,
            commanded_rad=start,
            started_monotonic_ns=now_ns,
        )

    def _observe_post_position_command(
        self, value: Mapping[str, Any], now_ns: int,
    ) -> None:
        post = self.post_execution
        _require(post is not None, "POSITION_COMMAND_WITHOUT_POST_POSITION_PHASE")
        if post.active_segment is not None:
            self._validate_post_refresh(value, post.active_segment, now_ns)
            return
        _require(post.awaiting_gui_command, "POST_POSITION_COMMAND_NOT_EXPECTED")
        self.binding.ensure_not_expired()
        self._require_full_gravity_position_authority(now_ns)
        _require(self._fresh_confirmation(now_ns, 1.0), "OPERATOR_CONFIRMATION_STALE")
        _require(value.get("schema") == GUI_COMMAND_SCHEMA, "POST_POSITION_COMMAND_SCHEMA_NOT_1_3")
        source_ns = value.get("source_monotonic_ns")
        _require(
            _valid_source_id(value.get("source_instance_id"))
            and type(source_ns) is int
            and 0 < source_ns <= now_ns
            and now_ns - source_ns <= 250_000_000,
            "POST_POSITION_COMMAND_SOURCE_OR_TIME_INVALID",
        )
        _require(
            type(value.get("sequence")) is int and value["sequence"] > 0,
            "POST_POSITION_COMMAND_SEQUENCE_INVALID",
        )
        moving_mask = value.get("moving_joint_mask")
        _require(
            isinstance(moving_mask, list)
            and len(moving_mask) == 6
            and all(type(item) is bool for item in moving_mask)
            and sum(moving_mask) == 1
            and value.get("active_joint_mask") == [True] * 6,
            "POST_POSITION_REQUIRES_EXACTLY_ONE_MOVING_JOINT",
        )
        moving_index = moving_mask.index(True)
        joint = JOINT_NAMES[moving_index]
        targets = _finite_vector(value.get("targets_rad"), 6, "POST_POSITION_TARGETS_INVALID")
        for index in range(6):
            if index == moving_index:
                continue
            _require(
                abs(targets[index] - post.commanded_rad[index]) <= 1.0e-6,
                "POST_POSITION_NONMOVING_TARGET_MUTATED",
            )
        current = post.commanded_rad[moving_index]
        final = post.target_rad[moving_index]
        requested = targets[moving_index]
        direction = 1.0 if final > current else -1.0
        _require(
            abs(final - current) > math.radians(0.01)
            and direction * (requested - current) > 0.0
            and direction * (final - requested) >= -1.0e-6,
            "POST_POSITION_SEGMENT_DOES_NOT_PROGRESS_TO_FROZEN_TARGET",
        )
        trajectory = value.get("trajectory")
        _require(isinstance(trajectory, Mapping), "POST_POSITION_TRAJECTORY_MISSING")
        trajectory_sha = trajectory.get("trajectory_sha256")
        plan_token = value.get("plan_token_id")
        _require(_valid_sha256(trajectory_sha), "POST_POSITION_TRAJECTORY_SHA_INVALID")
        _require(_valid_sha256(plan_token), "POST_POSITION_PLAN_TOKEN_INVALID")
        trajectory_start = _finite_vector(
            trajectory.get("start_rad"), 6, "POST_POSITION_TRAJECTORY_START_INVALID"
        )
        trajectory_target = _finite_vector(
            trajectory.get("target_rad"), 6, "POST_POSITION_TRAJECTORY_TARGET_INVALID"
        )
        changed = [
            index for index, (begin, end) in enumerate(zip(trajectory_start, trajectory_target))
            if abs(end - begin) > math.radians(0.01)
        ]
        _require(
            trajectory.get("schema") == "go-m8010-quintic-command/1.0"
            and trajectory.get("profile") == "quintic-rest-to-rest-v1"
            and changed == [moving_index]
            and all(abs(left - right) <= 1.0e-9 for left, right in zip(trajectory_target, targets)),
            "POST_POSITION_TRAJECTORY_NOT_SINGLE_JOINT_BOUND",
        )
        displacement = abs(_rad_to_deg(
            trajectory_target[moving_index] - trajectory_start[moving_index]
        ))
        duration_ns = trajectory.get("duration_ns")
        execute_at_ns = trajectory.get("execute_at_monotonic_ns")
        _require(
            displacement <= self.binding.maximum_segment_displacement_deg + 1.0e-6
            and type(duration_ns) is int
            and 0 < duration_ns
            <= min(
                MAXIMUM_SEGMENT_NS,
                int(self.binding.maximum_segment_seconds * 1.0e9),
            )
            and type(execute_at_ns) is int
            and execute_at_ns > 0,
            "POST_POSITION_SEGMENT_BOUND_EXCEEDED",
        )
        manifest = value.get("plan_manifest")
        _require(isinstance(manifest, Mapping), "POST_POSITION_MANIFEST_MISSING")
        recipe_sha = manifest.get("recipe_sha256")
        segment_shas = manifest.get("segment_sha256")
        _require(
            manifest.get("schema") == "go-m8010-plan-manifest/1.0"
            and _valid_sha256(recipe_sha)
            and isinstance(segment_shas, list)
            and trajectory_sha in segment_shas,
            "POST_POSITION_MANIFEST_INVALID",
        )
        if post.recipe_sha256 is None:
            post.recipe_sha256 = recipe_sha
            post.manifest_segment_sha256 = tuple(segment_shas)
        else:
            _require(
                recipe_sha == post.recipe_sha256
                and tuple(segment_shas) == post.manifest_segment_sha256,
                "POST_POSITION_RECIPE_OR_MANIFEST_CHANGED",
            )
        proof = value.get("collision_guard_proof")
        _require(
            isinstance(proof, Mapping)
            and proof.get("schema") == "go-m8010-collision-guard-result/1.0"
            and proof.get("safe") is True
            and proof.get("session_id") == self.binding.session_id
            and proof.get("state_instance_id") == self.binding.state_instance_id
            and proof.get("moving_joint_mask") == moving_mask
            and proof.get("model_sha256") == PRODUCTION_MODEL_SHA256,
            "POST_POSITION_COLLISION_PROOF_INVALID",
        )
        self._planned_proof_for_trajectory(trajectory_sha)
        next_budget = self.position_trajectory_budget_ns + duration_ns
        _require(
            next_budget
            <= min(
                MAXIMUM_POSITION_TOTAL_NS,
                int(self.binding.maximum_position_seconds * 1.0e9),
            ),
            "POSITION_TRAJECTORY_BUDGET_EXCEEDED_600_SECONDS",
        )
        self.position_trajectory_budget_ns = next_budget
        self._budgeted_trajectory_sha256.add(trajectory_sha)
        post.active_segment = ActiveSegment(
            joint=joint,
            phase=post.kind,
            target_rad=targets,
            trajectory_sha256=trajectory_sha,
            plan_token_id=plan_token,
            recipe_sha256=recipe_sha,
            collision_proof_sha256=_document_sha256(proof),
            accepted_monotonic_ns=now_ns,
            execute_at_monotonic_ns=execute_at_ns,
            duration_ns=duration_ns,
            gui_command_sequence=value["sequence"],
            gui_command_source_instance_id=value["source_instance_id"],
            gui_command_source_monotonic_ns=source_ns,
        )
        post.awaiting_gui_command = False
        post.endpoint_dwell_started_ns = None
        post.endpoint_last_sample_ns = None

    @staticmethod
    def _validate_post_refresh(
        value: Mapping[str, Any], segment: ActiveSegment, now_ns: int,
    ) -> None:
        source_ns = value.get("source_monotonic_ns")
        trajectory = value.get("trajectory")
        manifest = value.get("plan_manifest")
        proof = value.get("collision_guard_proof")
        moving_mask = [False] * 6
        moving_mask[JOINT_NAMES.index(segment.joint)] = True
        _require(
            value.get("schema") == GUI_COMMAND_SCHEMA
            and _valid_source_id(value.get("source_instance_id"))
            and type(source_ns) is int
            and 0 < source_ns <= now_ns
            and now_ns - source_ns <= 250_000_000
            and value.get("moving_joint_mask") == moving_mask
            and tuple(_finite_vector(value.get("targets_rad"), 6, "POST_REFRESH_TARGET_INVALID"))
            == segment.target_rad
            and value.get("plan_token_id") == segment.plan_token_id
            and isinstance(trajectory, Mapping)
            and trajectory.get("trajectory_sha256") == segment.trajectory_sha256
            and trajectory.get("duration_ns") == segment.duration_ns
            and trajectory.get("execute_at_monotonic_ns") == segment.execute_at_monotonic_ns
            and isinstance(manifest, Mapping)
            and manifest.get("recipe_sha256") == segment.recipe_sha256
            and isinstance(proof, Mapping)
            and _document_sha256(proof) == segment.collision_proof_sha256,
            "POST_POSITION_REFRESH_MUTATED",
        )

    def _validated_hardware_state(self, value: object, now_ns: int) -> dict[str, Any]:
        _require(isinstance(value, Mapping), "HARDWARE_STATE_NOT_OBJECT")
        _require(value.get("schema") == HARDWARE_STATE_SCHEMA, "HARDWARE_STATE_SCHEMA_MISMATCH")
        _require(
            value.get("session_id") == self.binding.session_id
            and value.get("state_instance_id") == self.binding.state_instance_id,
            "HARDWARE_STATE_BINDING_MISMATCH",
        )
        source_ns = value.get("source_monotonic_ns")
        sequence = value.get("sequence")
        _require(
            type(source_ns) is int
            and 0 < source_ns <= now_ns
            and now_ns - source_ns <= MAXIMUM_STATE_AGE_NS,
            "HARDWARE_STATE_STALE",
        )
        _require(type(sequence) is int and sequence > 0, "HARDWARE_STATE_SEQUENCE_INVALID")
        if self.latest_hardware_sequence is not None:
            _require(
                self.latest_hardware_source_ns is not None
                and sequence > self.latest_hardware_sequence
                and source_ns > self.latest_hardware_source_ns,
                "HARDWARE_STATE_REPLAYED",
            )
        _require(
            value.get("healthy") is True
            and value.get("telemetry_healthy") is True
            and value.get("safety_metadata_ready") is True
            and value.get("j2_sync_fault") is False,
            "HARDWARE_STATE_NOT_SAFETY_READY",
        )
        positions = _finite_vector(value.get("position_rad"), 6, "HARDWARE_POSITION_INVALID")
        velocities = _finite_vector(value.get("velocity_rad_s"), 6, "HARDWARE_VELOCITY_INVALID")
        _finite(value.get("j2_e_sync_rad"), "HARDWARE_J2_SYNC_INVALID")
        per_motor = value.get("per_motor")
        _require(isinstance(per_motor, Mapping), "HARDWARE_PER_MOTOR_MISSING")
        for motor in MOTOR_NAMES:
            item = per_motor.get(motor)
            _require(isinstance(item, Mapping), f"HARDWARE_{motor}_MISSING")
            _require(
                item.get("communication_ok") is True
                and item.get("fresh") is True
                and item.get("merror") == 0
                and item.get("controller_metadata_status") == "OBSERVED"
                and item.get("thermal_metadata_status") == "OBSERVED"
                and item.get("thermal_state") in {"NORMAL", "WARNING"}
                and item.get("thermal_fault_latched") is False
                and item.get("no_progress_metadata_status") == "OBSERVED"
                and item.get("load_limit_no_progress") is False,
                f"HARDWARE_{motor}_LIVE_GATE_FAILED",
            )
            temperature = _finite(item.get("temperature_c"), f"HARDWARE_{motor}_TEMPERATURE_INVALID")
            _require(0.0 <= temperature < HARD_THERMAL_STOP_C, f"HARDWARE_{motor}_THERMAL_STOP")
            _finite(item.get("q_joint_rad"), f"HARDWARE_{motor}_POSITION_INVALID")
            _finite(item.get("dq_joint_rad_s"), f"HARDWARE_{motor}_VELOCITY_INVALID")
        modes = value.get("controller_mode_by_motor")
        _require(
            isinstance(modes, Mapping)
            and all(motor in modes for motor in MOTOR_NAMES),
            "HARDWARE_CONTROLLER_MODES_MISSING",
        )
        faults = value.get("controller_fault_observed_by_motor")
        _require(
            isinstance(faults, Mapping)
            and all(faults.get(motor) is False for motor in MOTOR_NAMES),
            "HARDWARE_CONTROLLER_FAULT",
        )
        available = value.get("control_available_by_domain")
        _require(
            isinstance(available, Mapping)
            and all(available.get(domain) is True for domain in ("J1", "J2", "J345", "J6")),
            "HARDWARE_WORKER_CONTROL_UNAVAILABLE",
        )
        supervisor_instance = value.get("worker_supervisor_instance_id")
        supervisor_pid = value.get("worker_supervisor_pid")
        _require(
            isinstance(supervisor_instance, str)
            and supervisor_instance
            and type(supervisor_pid) is int
            and supervisor_pid > 1,
            "WORKER_SUPERVISOR_IDENTITY_MISSING",
        )
        supervisor_identity = (supervisor_instance, supervisor_pid)
        if self.worker_supervisor_identity is None:
            self.worker_supervisor_identity = supervisor_identity
        else:
            _require(
                supervisor_identity == self.worker_supervisor_identity,
                "WORKER_SUPERVISOR_RESTART_OR_SESSION_TAKEOVER",
            )
        normalized = dict(value)
        normalized["position_rad"] = positions
        normalized["velocity_rad_s"] = velocities
        return normalized

    def observe_hardware_state(self, value: object, *, now_ns: int) -> None:
        if self.failure is not None:
            return
        try:
            if (
                self.latest_hardware_received_ns is not None
                and (
                    self.gravity_ladder_active
                    or self.position_started_ns is not None
                    or self.post_execution is not None
                    or self.comparison_condition is not None
                    or self.thermal_stage is not None
                )
            ):
                _require(
                    now_ns - self.latest_hardware_received_ns <= MAXIMUM_STREAM_GAP_NS,
                    "HARDWARE_STATE_STREAM_GAP",
                )
            state = self._validated_hardware_state(value, now_ns)
            self.latest_hardware = state
            self.latest_hardware_received_ns = now_ns
            self.latest_hardware_sequence = state["sequence"]
            self.latest_hardware_source_ns = state["source_monotonic_ns"]
            self._cache_gravity_hardware_state(state, now_ns)
            if self.gravity_ladder_active:
                self._drain_gravity_hardware_pairs(now_ns=now_ns)
            powered_success_path = bool(
                self.gravity_ladder_active
                or self.gravity_ladder_complete
                or self.position_started_ns is not None
                or self.comparison_condition is not None
                or self.post_execution is not None
                or self.thermal_stage is not None
                or self.thermal_pending_review is not None
                or self.post_workflow_phase != "SINGLE_JOINT"
            )
            if powered_success_path:
                modes = state["controller_mode_by_motor"]
                self.success_path_hardware_events.append({
                    "monotonic_ns": now_ns,
                    "modes": dict(modes),
                    "j6_control_available": state[
                        "control_available_by_domain"
                    ].get("J6"),
                    "final_brake_requested": self.final_brake_requested,
                    "workflow_phase": self.post_workflow_phase,
                })
                if not self.final_brake_requested:
                    _require(
                        all(
                            modes[motor] in {"hold", "position"}
                            for motor in MOTOR_NAMES
                        )
                        and state["control_available_by_domain"].get("J6")
                        is True,
                        "SUCCESS_PATH_BRAKE_DISABLED_OR_J6_INACTIVE",
                    )
            if self.position_started_ns is not None and not self.position_complete:
                self._advance_position(state, now_ns)
            if self.post_execution is not None:
                self._advance_post_position(state, now_ns)
            if self.comparison_condition is not None:
                self._capture_comparison(state, now_ns)
            if self.thermal_stage is not None:
                self._capture_thermal(state, now_ns)
            if (
                self.post_workflow_phase == "AWAIT_FINAL_BRAKE"
                and self.post_execution is None
                and not self.final_brake_requested
            ):
                self._guard_restored_pose(state, now_ns)
            if self.final_brake_requested:
                modes = state["controller_mode_by_motor"]
                _require(
                    self.center_rad is not None,
                    "FINAL_BRAKE_SESSION_CENTER_MISSING",
                )
                final_brake_position_error_deg = [
                    abs(_rad_to_deg(actual - center))
                    for actual, center in zip(
                        state["position_rad"], self.center_rad
                    )
                ]
                _require(
                    max(final_brake_position_error_deg)
                    <= ENDPOINT_ERROR_DEG,
                    "FINAL_BRAKE_RESTORED_POSITION_DRIFT_EXCEEDED_0_5_DEG",
                )
                raw_is_fresh_disabled = bool(
                    all(modes[motor] == "brake" for motor in MOTOR_NAMES)
                    and self.j6_disabled_raw_observed_ns is not None
                    and 0 <= now_ns - self.j6_disabled_raw_observed_ns
                    <= MAXIMUM_ENDPOINT_SAMPLE_GAP_NS
                    and self.j6_disabled_raw_source_ns is not None
                )
                if raw_is_fresh_disabled:
                    aggregated_identity = state.get(
                        "j6_raw_feedback_identity"
                    )
                    _require(
                        isinstance(aggregated_identity, Mapping)
                        and aggregated_identity.get("source_instance_id")
                        == self.j6_raw_source_instance_id
                        and aggregated_identity.get("sequence")
                        == self.j6_disabled_raw_sequence
                        and aggregated_identity.get("source_monotonic_ns")
                        == self.j6_disabled_raw_source_ns
                        and aggregated_identity.get("session_id")
                        == self.binding.session_id
                        and aggregated_identity.get("state_instance_id")
                        == self.binding.state_instance_id,
                        "FINAL_BRAKE_J6_RAW_AGGREGATOR_IDENTITY_MISMATCH",
                    )
                if (
                    raw_is_fresh_disabled
                    and self.j6_disabled_raw_source_ns
                    != self.final_brake_last_paired_raw_source_ns
                    and abs(
                        int(state["source_monotonic_ns"])
                        - self.j6_disabled_raw_source_ns
                    ) <= MAXIMUM_ENDPOINT_SAMPLE_GAP_NS
                ):
                    previous_state_source_ns = (
                        None
                        if not self.final_brake_pairs
                        else int(self.final_brake_pairs[-1][
                            "hardware_state_source_monotonic_ns"
                        ])
                    )
                    if (
                        self.final_brake_last_pair_ns is None
                        or now_ns - self.final_brake_last_pair_ns
                        > MAXIMUM_ENDPOINT_SAMPLE_GAP_NS
                        or previous_state_source_ns is None
                        or int(state["source_monotonic_ns"])
                        - previous_state_source_ns
                        > MAXIMUM_ENDPOINT_SAMPLE_GAP_NS
                    ):
                        self.final_brake_dwell_started_ns = now_ns
                        self.final_brake_pair_count = 1
                        self.final_brake_pairs = []
                    else:
                        _require(
                            now_ns > self.final_brake_last_pair_ns,
                            "FINAL_BRAKE_PAIR_TIME_NOT_INCREASING",
                        )
                        self.final_brake_pair_count += 1
                    self.final_brake_last_pair_ns = now_ns
                    self.final_brake_last_paired_raw_source_ns = (
                        self.j6_disabled_raw_source_ns
                    )
                    self.final_brake_pairs.append({
                        "hardware_state_sequence": state["sequence"],
                        "hardware_state_source_monotonic_ns": state[
                            "source_monotonic_ns"
                        ],
                        "hardware_state_sha256": _document_sha256(state),
                        "position_rad": [
                            float(value) for value in state["position_rad"]
                        ],
                        "position_error_from_session_center_deg": (
                            final_brake_position_error_deg
                        ),
                        "go_controller_modes": {
                            motor: modes[motor] for motor in GO_MOTOR_NAMES
                        },
                        "j6_aggregated_controller_mode": modes["J6"],
                        "j6_raw_source_monotonic_ns": (
                            self.j6_disabled_raw_source_ns
                        ),
                        "j6_raw_source_instance_id": (
                            self.j6_raw_source_instance_id
                        ),
                        "j6_raw_sequence": self.j6_disabled_raw_sequence,
                        "j6_raw_session_id": self.binding.session_id,
                        "j6_raw_state_instance_id": (
                            self.binding.state_instance_id
                        ),
                        "j6_raw_sha256": self.j6_disabled_raw_sha256,
                        "j6_raw_drive_state": 0,
                        "j6_raw_controller_mode": "brake",
                        "pair_receipt_monotonic_ns": now_ns,
                    })
                    assert self.final_brake_dwell_started_ns is not None
                    first_pair = self.final_brake_pairs[0]
                    receipt_dwell_ns = now_ns - int(
                        first_pair["pair_receipt_monotonic_ns"]
                    )
                    hardware_source_dwell_ns = int(
                        state["source_monotonic_ns"]
                    ) - int(first_pair[
                        "hardware_state_source_monotonic_ns"
                    ])
                    raw_source_dwell_ns = int(
                        self.j6_disabled_raw_source_ns
                    ) - int(first_pair["j6_raw_source_monotonic_ns"])
                    dwell_ns = min(
                        receipt_dwell_ns,
                        hardware_source_dwell_ns,
                        raw_source_dwell_ns,
                    )
                    self.final_brake_dwell_seconds = dwell_ns / 1.0e9
                    if (
                        dwell_ns >= ENDPOINT_DWELL_NS
                        and self.final_brake_pair_count
                        >= MINIMUM_HALF_SECOND_SAMPLE_COUNT
                    ):
                        self.final_brake_observed = True
                        self.final_brake_hardware_state_sha256 = _document_sha256(state)
                        self.post_workflow_phase = "COMPLETE"
        except AcceptanceError as exc:
            workflow_started = bool(
                self.gravity_ladder_active
                or self.gravity_ladder_complete
                or self.position_started_ns is not None
                or self.post_execution is not None
                or self.comparison_condition is not None
                or self.thermal_stage is not None
                or self.thermal_pending_review is not None
                or self.final_brake_requested
                or self.post_workflow_phase != "SINGLE_JOINT"
            )
            if not workflow_started and self.latest_hardware is None:
                return
            domains = self._related_domains_for_state_failure(str(exc))
            self.fail(str(exc), domains, now_ns)

    def _guard_restored_pose(
        self, state: Mapping[str, Any], now_ns: int,
    ) -> None:
        """Invalidate stale restore evidence on any pre-final drift or mode change."""

        _require(self.center_rad is not None, "RESTORE_GUARD_CENTER_MISSING")
        _require(
            self.completed_post_order
            and self.completed_post_order[-1] == "RESTORE_INITIAL",
            "RESTORE_GUARD_LATEST_ACTION_NOT_RESTORE",
        )
        _require(
            all(
                abs(_rad_to_deg(actual - center)) <= ENDPOINT_ERROR_DEG
                for actual, center in zip(state["position_rad"], self.center_rad)
            ),
            "RESTORE_GUARD_POSITION_DRIFT_EXCEEDED_0_5_DEG",
        )
        _require(
            max(abs(_rad_to_deg(value)) for value in state["velocity_rad_s"])
            <= 0.25,
            "RESTORE_GUARD_ARM_NOT_STATIONARY",
        )
        _require(
            all(
                state["controller_mode_by_motor"][motor] == "hold"
                for motor in MOTOR_NAMES
            ),
            "RESTORE_GUARD_NOT_WHOLE_ARM_HOLD",
        )
        _require(self._fresh_confirmation(now_ns, 1.0), "OPERATOR_CONFIRMATION_STALE")
        self._require_full_gravity_position_authority(now_ns)
        self.restore_guard_hardware_state_sha256 = _document_sha256(state)

    def observe_motor_feedback_raw(self, value: object, *, now_ns: int) -> None:
        """Pair final J6 DISABLED drive-state proof with the state aggregator."""

        if not self.final_brake_requested or not isinstance(value, Mapping):
            return
        samples = value.get("samples")
        if (
            value.get("schema") != "go-m8010-motor-feedback/1.0"
            or not isinstance(samples, list)
            or len(samples) != 1
            or not isinstance(samples[0], Mapping)
            or samples[0].get("motor") != "J6"
        ):
            return
        source = value.get("source_instance_id")
        sequence = value.get("sequence")
        source_ns = value.get("source_monotonic_ns")
        sample = samples[0]
        try:
            _require(
                _valid_source_id(source)
                and type(sequence) is int
                and sequence > 0
                and type(source_ns) is int
                and 0 < source_ns <= now_ns
                and now_ns - source_ns <= MAXIMUM_STATE_AGE_NS,
                "J6_DISABLED_RAW_IDENTITY_OR_TIME_INVALID",
            )
            _require(
                value.get("session_id") == self.binding.session_id
                and value.get("state_instance_id")
                == self.binding.state_instance_id,
                "J6_DISABLED_RAW_BINDING_MISMATCH",
            )
            _require(
                value.get("drive_state") == 0
                and value.get("controller_mode") == "brake"
                and isinstance(value.get("controller_mode_by_motor"), Mapping)
                and value["controller_mode_by_motor"].get("J6") == "brake"
                and sample.get("communication_ok") is True
                and sample.get("merror") == 0,
                "J6_DISABLED_RAW_NOT_HEALTHY_DISABLED",
            )
            if self.j6_raw_source_instance_id is None:
                self.j6_raw_source_instance_id = source
            _require(
                source == self.j6_raw_source_instance_id,
                "J6_DISABLED_RAW_SOURCE_RESTARTED",
            )
            if self.j6_raw_last_sequence is not None:
                _require(
                    self.j6_raw_last_source_ns is not None
                    and sequence > self.j6_raw_last_sequence
                    and source_ns > self.j6_raw_last_source_ns,
                    "J6_DISABLED_RAW_SAMPLE_NOT_INCREASING",
                )
                if (
                    source_ns - self.j6_raw_last_source_ns
                    > MAXIMUM_ENDPOINT_SAMPLE_GAP_NS
                ):
                    self.final_brake_dwell_started_ns = None
                    self.final_brake_last_pair_ns = None
                    self.final_brake_last_paired_raw_source_ns = None
                    self.final_brake_pair_count = 0
                    self.final_brake_dwell_seconds = 0.0
                    self.final_brake_pairs = []
            self.j6_raw_last_sequence = sequence
            self.j6_raw_last_source_ns = source_ns
            self.j6_disabled_raw_observed_ns = now_ns
            self.j6_disabled_raw_source_ns = source_ns
            self.j6_disabled_raw_sequence = sequence
            self.j6_disabled_raw_sha256 = _document_sha256(value)
        except AcceptanceError as exc:
            self.fail(str(exc), ("J6",), now_ns)

    def _related_domains_for_state_failure(self, reason: str) -> tuple[str, ...]:
        if "J2" in reason or "SYNC" in reason:
            return ("J2",)
        for joint in ("J1", "J3", "J4", "J5", "J6"):
            if joint in reason:
                return (DOMAIN_BY_JOINT[joint],)
        if self.expected_joint is not None:
            return (DOMAIN_BY_JOINT[self.expected_joint],)
        return ("J1", "J2", "J345", "J6")

    def start_position(self, *, now_ns: int) -> None:
        _require(self.failure is None, "RUNNER_ALREADY_FAILED")
        _require(self.position_started_ns is None, "POSITION_ALREADY_STARTED")
        _require(
            self.gravity_ladder_complete,
            "POSITION_REQUIRES_OBSERVED_COMPLETE_GRAVITY_LADDER",
        )
        _require(
            self.comparison_complete == {"WITHOUT_FF", "WITH_FF"},
            "POSITION_REQUIRES_COMPLETED_WITHOUT_AND_WITH_FF_COMPARISON",
        )
        self.binding.ensure_not_expired()
        _require(self._fresh_confirmation(now_ns, 1.0), "OPERATOR_CONFIRMATION_STALE")
        self._require_full_gravity_position_authority(now_ns)
        _require(self.latest_hardware is not None, "HARDWARE_STATE_MISSING")
        _require(
            self.latest_hardware_received_ns is not None
            and now_ns - self.latest_hardware_received_ns <= MAXIMUM_STATE_AGE_NS,
            "HARDWARE_STATE_RECEIPT_STALE",
        )
        modes = self.latest_hardware["controller_mode_by_motor"]
        _require(
            all(modes[motor] == "hold" for motor in MOTOR_NAMES),
            "POSITION_START_REQUIRES_CONTINUOUS_WHOLE_ARM_HOLD",
        )
        velocities = self.latest_hardware["velocity_rad_s"]
        _require(
            max(abs(_rad_to_deg(value)) for value in velocities) <= 0.25,
            "POSITION_START_NOT_STATIONARY",
        )
        self.center_rad = tuple(self.latest_hardware["position_rad"])
        self.position_started_ns = now_ns
        self.position_joint_index = 0
        self.position_phase_index = 0
        self.endpoint_dwell_started_ns = None
        self.endpoint_last_stable_sample_ns = None
        self.awaiting_position_command = False

    def _advance_position(self, state: Mapping[str, Any], now_ns: int) -> None:
        _require(self.position_started_ns is not None, "POSITION_NOT_STARTED")
        _require(self._fresh_confirmation(now_ns, 1.0), "OPERATOR_CONFIRMATION_STALE")
        self._require_full_gravity_position_authority(now_ns)
        sync_deg = abs(_rad_to_deg(_finite(state.get("j2_e_sync_rad"), "J2_SYNC_INVALID")))
        _require(sync_deg <= J2_SYNC_WARNING_DEG, "J2_SYNC_WARNING_EXCEEDED")
        joint = self.expected_joint
        phase = self.expected_phase
        _require(joint is not None and phase is not None, "POSITION_SEQUENCE_INVALID")
        joint_index = JOINT_NAMES.index(joint)
        _require(self.center_rad is not None, "POSITION_CENTER_NOT_CAPTURED")
        target_rad = self.center_rad[joint_index] + math.radians(
            PHASE_OFFSET_DEG[self.position_phase_index]
        )
        actual_rad = state["position_rad"][joint_index]
        sample_source_ns = int(state["source_monotonic_ns"])
        error_deg = _rad_to_deg(target_rad - actual_rad)
        modes = state["controller_mode_by_motor"]
        segment = self.active_segment
        if phase != "CENTER_START":
            if self.awaiting_position_command:
                return
            _require(segment is not None, "POSITION_ACTIVE_SEGMENT_MISSING")
            _require(
                now_ns - segment.accepted_monotonic_ns
                <= min(
                    MAXIMUM_SEGMENT_NS,
                    int(self.binding.maximum_segment_seconds * 1.0e9),
                ),
                "POSITION_SEGMENT_RUNTIME_EXCEEDED_15_SECONDS",
            )
            moving_motors = set(MOTOR_BY_JOINT[joint])
            _require(
                all(
                    modes[motor] == "hold"
                    for motor in MOTOR_NAMES if motor not in moving_motors
                ),
                "NONMOVING_DOMAIN_LEFT_CONTINUOUS_HOLD",
            )
            _require(
                all(modes[motor] == "position" for motor in moving_motors),
                "MOVING_DOMAIN_NOT_POSITION_ACTIVE",
            )
            for motor in moving_motors:
                item = state["per_motor"][motor]
                if (
                    item.get("trajectory_plan_token_id") == segment.plan_token_id
                    and item.get("trajectory_sha256") == segment.trajectory_sha256
                    and item.get("trajectory_state") in {"ACTIVE", "COMPLETE"}
                ):
                    segment.echo_observed = True
            if not segment.router_accepted and self.latest_router is not None:
                self.observe_router_status(self.latest_router, now_ns=now_ns)
            if segment.execution_trace is None:
                segment.execution_trace = []
            segment.execution_trace.append(
                self._motion_trace_sample(
                    state, now_ns, joint, target_rad, segment
                )
            )
        else:
            _require(
                all(modes[motor] == "hold" for motor in MOTOR_NAMES),
                "CENTER_DWELL_NOT_WHOLE_ARM_CONTINUOUS_HOLD",
            )
        velocities = state["velocity_rad_s"]
        stable = (
            abs(error_deg) <= ENDPOINT_ERROR_DEG
            and abs(_rad_to_deg(velocities[joint_index])) <= 0.25
        )
        if not stable:
            self.endpoint_dwell_started_ns = None
            self.endpoint_last_stable_sample_ns = None
            self.endpoint_dwell_trace = []
            return
        dwell_sample = self._motion_trace_sample(
            state, now_ns, joint, target_rad, segment
        )
        if (
            self.endpoint_dwell_started_ns is None
            or self.endpoint_last_stable_sample_ns is None
            or sample_source_ns - self.endpoint_last_stable_sample_ns
            > MAXIMUM_ENDPOINT_SAMPLE_GAP_NS
        ):
            self.endpoint_dwell_started_ns = sample_source_ns
            self.endpoint_last_stable_sample_ns = sample_source_ns
            self.endpoint_dwell_trace = [dwell_sample]
            return
        _require(
            sample_source_ns > self.endpoint_last_stable_sample_ns,
            "ENDPOINT_STABLE_SAMPLE_NOT_STRICTLY_INCREASING",
        )
        self.endpoint_last_stable_sample_ns = sample_source_ns
        self.endpoint_dwell_trace.append(dwell_sample)
        dwell_ns = sample_source_ns - self.endpoint_dwell_started_ns
        if dwell_ns < ENDPOINT_DWELL_NS:
            return
        if segment is not None:
            _require(segment.echo_observed, "TRAJECTORY_SHA_NOT_ECHOED_BY_WORKER")
            _require(segment.router_accepted, "ROUTER_ACCEPTANCE_NOT_OBSERVED")
        _require(
            len(self.endpoint_dwell_trace) >= MINIMUM_HALF_SECOND_SAMPLE_COUNT,
            "ENDPOINT_DWELL_TRACE_SAMPLE_COUNT_INSUFFICIENT",
        )
        self._record_position_endpoint(state, now_ns, dwell_ns)
        self.endpoint_dwell_started_ns = None
        self.endpoint_last_stable_sample_ns = None
        self.endpoint_dwell_trace = []
        self.active_segment = None
        if self.position_phase_index == len(POSITION_PHASES) - 1:
            self.position_joint_index += 1
            self.position_phase_index = 0
            if self.position_joint_index == len(POSITION_ORDER):
                self.position_complete = True
                self.awaiting_position_command = False
                self.post_workflow_phase = "AWAIT_MULTI_JOINT"
                return
            self.awaiting_position_command = False
        else:
            self.position_phase_index += 1
            self.awaiting_position_command = True

    @staticmethod
    def _motion_trace_sample(
        state: Mapping[str, Any],
        receipt_ns: int,
        joint: str,
        target_rad: float,
        segment: Optional[ActiveSegment],
    ) -> dict[str, Any]:
        index = JOINT_NAMES.index(joint)
        return {
            "hardware_state_sequence": int(state["sequence"]),
            "hardware_state_source_monotonic_ns": int(
                state["source_monotonic_ns"]
            ),
            "receipt_monotonic_ns": receipt_ns,
            "hardware_state_sha256": _document_sha256(state),
            "joint": joint,
            "target_rad": target_rad,
            "actual_rad": float(state["position_rad"][index]),
            "error_deg": _rad_to_deg(
                target_rad - float(state["position_rad"][index])
            ),
            "velocity_deg_s": _rad_to_deg(
                float(state["velocity_rad_s"][index])
            ),
            "position_rad": [float(value) for value in state["position_rad"]],
            "controller_mode_by_motor": dict(
                state["controller_mode_by_motor"]
            ),
            "moving_joint_mask": (
                [False] * 6
                if segment is None
                else [name == segment.joint for name in JOINT_NAMES]
            ),
            "trajectory_sha256": (
                "CENTER_OBSERVATION"
                if segment is None else segment.trajectory_sha256
            ),
        }

    def _record_position_endpoint(
        self, state: Mapping[str, Any], now_ns: int, dwell_ns: int,
    ) -> None:
        joint = self.expected_joint
        phase = self.expected_phase
        assert joint is not None and phase is not None and self.center_rad is not None
        joint_index = JOINT_NAMES.index(joint)
        target_rad = self.center_rad[joint_index] + math.radians(
            PHASE_OFFSET_DEG[self.position_phase_index]
        )
        actual_rad = state["position_rad"][joint_index]
        if phase == "CENTER_START":
            self.position_actual_center_start_rad[joint] = actual_rad
        _require(
            joint in self.position_actual_center_start_rad,
            "POSITION_ACTUAL_CENTER_START_MISSING",
        )
        actual_center_start_rad = self.position_actual_center_start_rad[joint]
        signed_displacement_deg = _rad_to_deg(
            actual_rad - actual_center_start_rad
        )
        if phase == "PLUS_5":
            _require(
                signed_displacement_deg
                >= POSITION_DISPLACEMENT_DEG - 1.0e-6,
                "POSITION_PLUS_ACTUAL_MOTION_BELOW_5_DEG",
            )
        elif phase == "MINUS_5":
            _require(
                signed_displacement_deg
                <= -POSITION_DISPLACEMENT_DEG + 1.0e-6,
                "POSITION_MINUS_ACTUAL_MOTION_BELOW_5_DEG",
            )
        motors = MOTOR_BY_JOINT[joint]
        temperature = max(
            float(state["per_motor"][motor]["temperature_c"]) for motor in motors
        )
        merror = max(int(state["per_motor"][motor]["merror"]) for motor in motors)
        communication = all(
            state["per_motor"][motor]["communication_ok"] is True for motor in motors
        )
        segment = self.active_segment
        if segment is not None:
            _require(
                0 < segment.duration_ns <= MAXIMUM_SEGMENT_NS,
                "POSITION_RETAINED_TRAJECTORY_DURATION_INVALID",
            )
            _require(
                segment.execute_at_monotonic_ns > 0,
                "POSITION_RETAINED_TRAJECTORY_EXECUTE_TIME_INVALID",
            )
        dwell_trace_document = {
            "schema": "V15.31B-endpoint-dwell-trace-v1",
            "samples": list(self.endpoint_dwell_trace),
        }
        execution_trace = (
            [] if segment is None or segment.execution_trace is None
            else list(segment.execution_trace)
        )
        execution_trace_document = {
            "schema": "V15.31B-segment-execution-trace-v1",
            "samples": execution_trace,
        }
        self.position_rows.append({
            "timestamp_utc": _utc_text(),
            "monotonic_ns": state["source_monotonic_ns"],
            "receipt_monotonic_ns": now_ns,
            "test_id": f"{joint}-{self.position_revision[joint]}",
            "joint": joint,
            "revision": self.position_revision[joint],
            "phase": phase,
            "target_deg": _rad_to_deg(target_rad),
            "actual_deg": _rad_to_deg(actual_rad),
            "error_deg": _rad_to_deg(target_rad - actual_rad),
            "endpoint_dwell_s": dwell_ns / 1.0e9,
            "j2_e_sync_deg": _rad_to_deg(float(state["j2_e_sync_rad"])),
            "temperature_c": temperature,
            "merror": merror,
            "communication_ok": communication,
            "controller_mode": "/".join(str(state["controller_mode_by_motor"][motor]) for motor in motors),
            "status": "PASS",
            "reason": "",
            "envelope_id": self.binding.envelope_id,
            "envelope_sha256": self.binding.envelope_sha256,
            "session_id": self.binding.session_id,
            "state_instance_id": self.binding.state_instance_id,
            "anchor_sha256": self.binding.anchor_sha256,
            "trajectory_sha256": "CENTER_OBSERVATION" if segment is None else segment.trajectory_sha256,
            "trajectory_duration_ns": (
                "CENTER_OBSERVATION" if segment is None else segment.duration_ns
            ),
            "trajectory_execute_at_monotonic_ns": (
                "CENTER_OBSERVATION"
                if segment is None else segment.execute_at_monotonic_ns
            ),
            "plan_token_id": "CENTER_OBSERVATION" if segment is None else segment.plan_token_id,
            "plan_recipe_sha256": "CENTER_OBSERVATION" if segment is None else segment.recipe_sha256,
            "collision_proof_sha256": "CENTER_OBSERVATION" if segment is None else segment.collision_proof_sha256,
            "hardware_state_sha256": _document_sha256(state),
            "related_domain": DOMAIN_BY_JOINT[joint],
            "hardware_state_sequence": state["sequence"],
            "hardware_state_source_monotonic_ns": state[
                "source_monotonic_ns"
            ],
            "gui_command_sequence": (
                "CENTER_OBSERVATION"
                if segment is None else segment.gui_command_sequence
            ),
            "gui_command_source_instance_id": (
                "CENTER_OBSERVATION"
                if segment is None
                else segment.gui_command_source_instance_id
            ),
            "gui_command_source_monotonic_ns": (
                "CENTER_OBSERVATION"
                if segment is None
                else segment.gui_command_source_monotonic_ns
            ),
            "actual_center_start_deg": _rad_to_deg(
                actual_center_start_rad
            ),
            "actual_displacement_from_center_deg": (
                signed_displacement_deg
            ),
            "minimum_required_actual_displacement_deg": (
                POSITION_DISPLACEMENT_DEG
                if phase in {"PLUS_5", "MINUS_5"} else 0.0
            ),
            "endpoint_dwell_trace_json": json.dumps(
                dwell_trace_document,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            ),
            "endpoint_dwell_trace_sha256": _document_sha256(
                dwell_trace_document
            ),
            "segment_execution_trace_json": json.dumps(
                execution_trace_document,
                ensure_ascii=False,
                allow_nan=False,
                sort_keys=True,
                separators=(",", ":"),
            ),
            "segment_execution_trace_sha256": _document_sha256(
                execution_trace_document
            ),
        })

    def _advance_post_position(
        self, state: Mapping[str, Any], now_ns: int,
    ) -> None:
        post = self.post_execution
        assert post is not None
        _require(self._fresh_confirmation(now_ns, 1.0), "OPERATOR_CONFIRMATION_STALE")
        self._require_full_gravity_position_authority(now_ns)
        sync_deg = abs(_rad_to_deg(state["j2_e_sync_rad"]))
        _require(sync_deg <= J2_SYNC_WARNING_DEG, "J2_SYNC_WARNING_EXCEEDED")
        modes = state["controller_mode_by_motor"]
        sample_source_ns = int(state["source_monotonic_ns"])
        segment = post.active_segment
        if segment is not None:
            index = JOINT_NAMES.index(segment.joint)
            moving_motors = set(MOTOR_BY_JOINT[segment.joint])
            _require(
                all(
                    modes[motor] == "hold"
                    for motor in MOTOR_NAMES if motor not in moving_motors
                ),
                "POST_POSITION_NONMOVING_DOMAIN_LEFT_HOLD",
            )
            _require(
                all(modes[motor] == "position" for motor in moving_motors),
                "POST_POSITION_MOVING_DOMAIN_NOT_POSITION",
            )
            _require(
                now_ns - segment.accepted_monotonic_ns
                <= min(
                    MAXIMUM_SEGMENT_NS,
                    int(self.binding.maximum_segment_seconds * 1.0e9),
                ),
                "POST_POSITION_SEGMENT_RUNTIME_EXCEEDED_15_SECONDS",
            )
            for motor in moving_motors:
                item = state["per_motor"][motor]
                if (
                    item.get("trajectory_plan_token_id") == segment.plan_token_id
                    and item.get("trajectory_sha256") == segment.trajectory_sha256
                    and item.get("trajectory_state") in {"ACTIVE", "COMPLETE"}
                ):
                    segment.echo_observed = True
            if not segment.router_accepted and self.latest_router is not None:
                self.observe_router_status(self.latest_router, now_ns=now_ns)
            if segment.execution_trace is None:
                segment.execution_trace = []
            segment.execution_trace.append(
                self._motion_trace_sample(
                    state,
                    now_ns,
                    segment.joint,
                    segment.target_rad[index],
                    segment,
                )
            )
            error_deg = _rad_to_deg(
                segment.target_rad[index] - state["position_rad"][index]
            )
            stable = (
                abs(error_deg) <= ENDPOINT_ERROR_DEG
                and abs(_rad_to_deg(state["velocity_rad_s"][index])) <= 0.25
            )
            if not stable:
                post.endpoint_dwell_started_ns = None
                post.endpoint_last_sample_ns = None
                assert post.endpoint_dwell_trace is not None
                post.endpoint_dwell_trace = []
                return
            dwell_sample = self._motion_trace_sample(
                state,
                now_ns,
                segment.joint,
                segment.target_rad[index],
                segment,
            )
            if (
                post.endpoint_dwell_started_ns is None
                or post.endpoint_last_sample_ns is None
                or sample_source_ns - post.endpoint_last_sample_ns
                > MAXIMUM_ENDPOINT_SAMPLE_GAP_NS
            ):
                post.endpoint_dwell_started_ns = sample_source_ns
                post.endpoint_last_sample_ns = sample_source_ns
                post.endpoint_dwell_trace = [dwell_sample]
                return
            _require(
                sample_source_ns > post.endpoint_last_sample_ns,
                "POST_POSITION_ENDPOINT_SAMPLE_NOT_INCREASING",
            )
            post.endpoint_last_sample_ns = sample_source_ns
            assert post.endpoint_dwell_trace is not None
            post.endpoint_dwell_trace.append(dwell_sample)
            dwell_ns = sample_source_ns - post.endpoint_dwell_started_ns
            if dwell_ns < ENDPOINT_DWELL_NS:
                return
            _require(segment.echo_observed, "POST_POSITION_TRAJECTORY_NOT_ECHOED")
            _require(segment.router_accepted, "POST_POSITION_ROUTER_ACCEPTANCE_NOT_OBSERVED")
            _require(
                len(post.endpoint_dwell_trace)
                >= MINIMUM_HALF_SECOND_SAMPLE_COUNT,
                "POST_POSITION_ENDPOINT_DWELL_TRACE_INSUFFICIENT",
            )
            commanded = list(post.commanded_rad)
            commanded[index] = segment.target_rad[index]
            post.commanded_rad = tuple(commanded)
            assert post.segment_records is not None
            execution_trace = list(segment.execution_trace or [])
            endpoint_trace = list(post.endpoint_dwell_trace)
            execution_trace_document = {
                "schema": "V15.31B-segment-execution-trace-v1",
                "samples": execution_trace,
            }
            endpoint_trace_document = {
                "schema": "V15.31B-endpoint-dwell-trace-v1",
                "samples": endpoint_trace,
            }
            _require(
                0 < segment.duration_ns <= MAXIMUM_SEGMENT_NS,
                "POST_POSITION_RETAINED_TRAJECTORY_DURATION_INVALID",
            )
            _require(
                segment.execute_at_monotonic_ns > 0,
                "POST_POSITION_RETAINED_TRAJECTORY_EXECUTE_TIME_INVALID",
            )
            post.segment_records.append({
                "joint": segment.joint,
                "trajectory_sha256": segment.trajectory_sha256,
                "trajectory_duration_ns": segment.duration_ns,
                "trajectory_execute_at_monotonic_ns": (
                    segment.execute_at_monotonic_ns
                ),
                "plan_token_id": segment.plan_token_id,
                "recipe_sha256": segment.recipe_sha256,
                "collision_proof_sha256": segment.collision_proof_sha256,
                "target_rad": segment.target_rad[index],
                "actual_rad": state["position_rad"][index],
                "endpoint_error_deg": error_deg,
                "endpoint_dwell_s": dwell_ns / 1.0e9,
                "hardware_state_sha256": _document_sha256(state),
                "gui_command_sequence": segment.gui_command_sequence,
                "gui_command_source_instance_id": (
                    segment.gui_command_source_instance_id
                ),
                "gui_command_source_monotonic_ns": (
                    segment.gui_command_source_monotonic_ns
                ),
                "execution_trace": execution_trace,
                "execution_trace_sha256": _document_sha256(
                    execution_trace_document
                ),
                "endpoint_dwell_trace": endpoint_trace,
                "endpoint_dwell_trace_sha256": _document_sha256(
                    endpoint_trace_document
                ),
            })
            post.active_segment = None
            post.endpoint_dwell_started_ns = None
            post.endpoint_last_sample_ns = None
            post.endpoint_dwell_trace = []
            post.awaiting_gui_command = not all(
                abs(commanded_value - final_value) <= 1.0e-6
                for commanded_value, final_value in zip(
                    post.commanded_rad, post.target_rad
                )
            )
            # The worker echo that completes the segment still reports the
            # moving domain in POSITION.  Require a subsequent independent
            # whole-arm HOLD frame before beginning final-pose dwell.
            return
        elif post.awaiting_gui_command:
            _require(
                all(modes[motor] == "hold" for motor in MOTOR_NAMES),
                "POST_POSITION_BETWEEN_SEGMENTS_NOT_WHOLE_ARM_HOLD",
            )
            return
        _require(
            all(modes[motor] == "hold" for motor in MOTOR_NAMES),
            "POST_POSITION_FINAL_DWELL_NOT_WHOLE_ARM_HOLD",
        )
        errors = [
            abs(_rad_to_deg(target - actual))
            for target, actual in zip(post.target_rad, state["position_rad"])
        ]
        assert post.maximum_error_deg is not None
        post.maximum_error_deg = [
            max(previous, current)
            for previous, current in zip(post.maximum_error_deg, errors)
        ]
        stable_all = (
            max(errors) <= ENDPOINT_ERROR_DEG
            and max(abs(_rad_to_deg(value)) for value in state["velocity_rad_s"])
            <= 0.25
        )
        if not stable_all:
            post.final_dwell_started_ns = None
            post.final_last_sample_ns = None
            post.final_dwell_trace = []
            return
        final_sample = {
            "hardware_state_sequence": int(state["sequence"]),
            "hardware_state_source_monotonic_ns": int(
                state["source_monotonic_ns"]
            ),
            "receipt_monotonic_ns": now_ns,
            "hardware_state_sha256": _document_sha256(state),
            "position_rad": [float(value) for value in state["position_rad"]],
            "velocity_rad_s": [
                float(value) for value in state["velocity_rad_s"]
            ],
            "error_deg_by_joint": {
                joint: errors[index]
                for index, joint in enumerate(JOINT_NAMES)
            },
            "controller_mode_by_motor": dict(modes),
        }
        if (
            post.final_dwell_started_ns is None
            or post.final_last_sample_ns is None
            or sample_source_ns - post.final_last_sample_ns
            > MAXIMUM_ENDPOINT_SAMPLE_GAP_NS
        ):
            post.final_dwell_started_ns = sample_source_ns
            post.final_last_sample_ns = sample_source_ns
            post.final_dwell_trace = [final_sample]
            return
        _require(
            sample_source_ns > post.final_last_sample_ns,
            "POST_POSITION_FINAL_SAMPLE_NOT_INCREASING",
        )
        post.final_last_sample_ns = sample_source_ns
        assert post.final_dwell_trace is not None
        post.final_dwell_trace.append(final_sample)
        final_dwell_ns = sample_source_ns - post.final_dwell_started_ns
        if final_dwell_ns < ENDPOINT_DWELL_NS:
            return
        assert post.segment_records is not None
        _require(
            len(post.final_dwell_trace) >= MINIMUM_HALF_SECOND_SAMPLE_COUNT,
            "POST_POSITION_FINAL_DWELL_TRACE_INSUFFICIENT",
        )
        observation = {
            "kind": post.kind,
            "start_deg": [_rad_to_deg(value) for value in post.start_rad],
            "target_deg": [_rad_to_deg(value) for value in post.target_rad],
            "final_deg": [_rad_to_deg(value) for value in state["position_rad"]],
            "max_error_deg_by_joint": {
                joint: post.maximum_error_deg[index]
                for index, joint in enumerate(JOINT_NAMES)
            },
            "minimum_dwell_s": final_dwell_ns / 1.0e9,
            "recipe_sha256": post.recipe_sha256,
            "segments": list(post.segment_records),
            "final_dwell_trace": list(post.final_dwell_trace),
            "final_dwell_trace_sha256": _document_sha256({
                "schema": "V15.31B-final-dwell-trace-v1",
                "samples": list(post.final_dwell_trace),
            }),
            "final_hardware_state_sha256": _document_sha256(state),
            "binding": {
                "envelope_id": self.binding.envelope_id,
                "envelope_sha256": self.binding.envelope_sha256,
                "session_id": self.binding.session_id,
                "state_instance_id": self.binding.state_instance_id,
                "anchor_sha256": self.binding.anchor_sha256,
            },
        }
        self.completed_post_execution[post.kind] = observation
        self.completed_post_order.append(post.kind)
        if post.kind == "THERMAL_SETUP":
            self.thermal_setup_target_j2_rad = post.target_rad[1]
            self.post_workflow_phase = "AWAIT_THERMAL_5"
        elif post.kind == "MULTI_JOINT":
            self.post_workflow_phase = "AWAIT_THERMAL_SETUP"
        else:
            self.post_workflow_phase = "AWAIT_FINAL_BRAKE"
            self.restore_completed_ns = now_ns
            self.restore_guard_hardware_state_sha256 = _document_sha256(state)
        self.post_execution = None
        self._maybe_build_multi_joint_document()

    def _maybe_build_multi_joint_document(self) -> None:
        multi = self.completed_post_execution.get("MULTI_JOINT")
        thermal_setup = self.completed_post_execution.get("THERMAL_SETUP")
        restore = self.completed_post_execution.get("RESTORE_INITIAL")
        if (
            multi is None
            or thermal_setup is None
            or restore is None
            or self.multi_joint_ui_attestation is None
            or self.restore_ui_attestation is None
        ):
            return
        ui = self.multi_joint_ui_attestation
        restore_ui = self.restore_ui_attestation
        self.multi_joint_document = {
            "schema": "V15.31B-multi-joint-validation-v1",
            "result": "PASS",
            "binding": multi["binding"],
            "planned_preview": ui["planned_preview"],
            "preview_before_real": ui["preview_before_real"],
            "planned_twin": ui["planned_twin"],
            "actual_twin": ui["actual_twin"],
            "explicit_user_submit": True,
            "actual_twin_source": "REAL_ENCODER",
            "start_deg": multi["start_deg"],
            "target_deg": multi["target_deg"],
            "final_deg": multi["final_deg"],
            "max_error_deg_by_joint": multi["max_error_deg_by_joint"],
            "minimum_dwell_s": multi["minimum_dwell_s"],
            "recipe_sha256": multi["recipe_sha256"],
            "segments": multi["segments"],
            "final_dwell_trace": multi["final_dwell_trace"],
            "final_dwell_trace_sha256": multi[
                "final_dwell_trace_sha256"
            ],
            "final_hardware_state_sha256": multi["final_hardware_state_sha256"],
            "thermal_setup": {
                "result": "PASS",
                "status": "PASS",
                "actual_twin_source": "REAL_ENCODER",
                "start_deg": thermal_setup["start_deg"],
                "target_deg": thermal_setup["target_deg"],
                "final_deg": thermal_setup["final_deg"],
                "max_error_deg_by_joint": thermal_setup[
                    "max_error_deg_by_joint"
                ],
                "minimum_dwell_s": thermal_setup["minimum_dwell_s"],
                "recipe_sha256": thermal_setup["recipe_sha256"],
                "segments": thermal_setup["segments"],
                "final_dwell_trace": thermal_setup[
                    "final_dwell_trace"
                ],
                "final_dwell_trace_sha256": thermal_setup[
                    "final_dwell_trace_sha256"
                ],
                "final_hardware_state_sha256": thermal_setup[
                    "final_hardware_state_sha256"
                ],
            },
            "restore_initial": {
                "result": "PASS",
                "status": "PASS",
                "planned_preview": restore_ui["planned_preview"],
                "explicit_user_submit": True,
                "actual_twin_source": "REAL_ENCODER",
                "target_deg": restore["target_deg"],
                "final_deg": restore["final_deg"],
                "max_error_deg_by_joint": restore["max_error_deg_by_joint"],
                "minimum_dwell_s": restore["minimum_dwell_s"],
                "recipe_sha256": restore["recipe_sha256"],
                "segments": restore["segments"],
                "final_dwell_trace": restore["final_dwell_trace"],
                "final_dwell_trace_sha256": restore[
                    "final_dwell_trace_sha256"
                ],
                "final_hardware_state_sha256": restore[
                    "final_hardware_state_sha256"
                ],
            },
        }

    def start_comparison(
        self, condition: str, target_j2_rad: float, *, now_ns: int,
    ) -> None:
        condition = condition.strip().upper()
        _require(condition in {"WITHOUT_FF", "WITH_FF"}, "COMPARISON_CONDITION_INVALID")
        _require(condition not in self.comparison_complete, "COMPARISON_CONDITION_ALREADY_COMPLETE")
        if condition == "WITHOUT_FF":
            _require(
                not self.comparison_complete,
                "COMPARISON_WITHOUT_FF_MUST_BE_FIRST",
            )
        else:
            _require(
                self.comparison_complete == {"WITHOUT_FF"},
                "COMPARISON_WITH_FF_REQUIRES_WITHOUT_FF_FIRST",
            )
            _require(
                self.gravity_ladder_complete,
                "COMPARISON_WITH_FF_REQUIRES_COMPLETE_GRAVITY_LADDER",
            )
        _require(self.comparison_condition is None, "COMPARISON_ALREADY_ACTIVE")
        _require(self.latest_hardware is not None, "HARDWARE_STATE_MISSING")
        _finite(target_j2_rad, "COMPARISON_TARGET_INVALID")
        _require(self.latest_gravity is not None, "GRAVITY_STATUS_MISSING")
        expected_scale = 0.0 if condition == "WITHOUT_FF" else 1.0
        _require(
            self._fresh_confirmation(
                now_ns, None if expected_scale == 0.0 else 1.0
            ),
            "OPERATOR_CONFIRMATION_STALE_OR_WRONG_GRAVITY_TARGET",
        )
        _require(
            self.latest_gravity.get("gravity_scale_target") == expected_scale
            and abs(_finite(self.latest_gravity.get("gravity_scale"), "COMPARISON_GRAVITY_SCALE_INVALID") - expected_scale)
            <= 1.0e-6,
            "COMPARISON_GRAVITY_SCALE_MISMATCH",
        )
        target_value = float(target_j2_rad)
        if condition == "WITHOUT_FF":
            _require(
                abs(_rad_to_deg(
                    target_value - self.latest_hardware["position_rad"][1]
                )) <= ENDPOINT_ERROR_DEG,
                "COMPARISON_WITHOUT_FF_TARGET_NOT_AT_CURRENT_POSE",
            )
            frozen = list(self.latest_hardware["position_rad"])
            frozen[1] = target_value
            self.comparison_frozen_pose_rad = tuple(frozen)
            self.comparison_frozen_target_j2_rad = target_value
            self.comparison_frozen_hardware_state_sha256 = _document_sha256(
                self.latest_hardware
            )
        else:
            _require(
                self.comparison_frozen_pose_rad is not None
                and self.comparison_frozen_target_j2_rad is not None
                and abs(target_value - self.comparison_frozen_target_j2_rad)
                <= 1.0e-9,
                "COMPARISON_WITH_FF_TARGET_DIFFERS_FROM_WITHOUT_FF",
            )
            _require(
                all(
                    abs(_rad_to_deg(actual - target))
                    <= ENDPOINT_ERROR_DEG
                    for actual, target in zip(
                        self.latest_hardware["position_rad"],
                        self.comparison_frozen_pose_rad,
                    )
                ),
                "COMPARISON_WITH_FF_POSE_DIFFERS_FROM_WITHOUT_FF",
            )
        self.comparison_condition = condition
        self.comparison_target_j2_rad = float(target_j2_rad)
        self.comparison_started_ns = now_ns
        self.comparison_started_source_ns = int(
            self.latest_hardware["source_monotonic_ns"]
        )
        self.comparison_last_sample_ns = None

    def stop_comparison(self, *, now_ns: int) -> None:
        _require(self.comparison_condition is not None, "COMPARISON_NOT_ACTIVE")
        _require(self.comparison_started_ns is not None, "COMPARISON_START_MISSING")
        _require(
            self.comparison_started_source_ns is not None,
            "COMPARISON_SOURCE_START_MISSING",
        )
        _require(now_ns - self.comparison_started_ns >= ENDPOINT_DWELL_NS, "COMPARISON_WINDOW_BELOW_0_5_SECONDS")
        condition = self.comparison_condition
        selected = [
            row for row in self.comparison_rows if row["condition"] == condition
        ]
        _require(
            len(selected) >= MINIMUM_HALF_SECOND_SAMPLE_COUNT,
            "COMPARISON_CONTINUOUS_SAMPLE_COUNT_INSUFFICIENT",
        )
        stamps = [
            int(row["hardware_state_source_monotonic_ns"])
            for row in selected
        ]
        _require(
            all(
                0 < current - previous <= MAXIMUM_COMPARISON_SAMPLE_GAP_NS
                for previous, current in zip(stamps, stamps[1:])
            ),
            "COMPARISON_SAMPLE_GAP_EXCEEDED_100MS",
        )
        _require(
            stamps[-1] - stamps[0] >= ENDPOINT_DWELL_NS,
            "COMPARISON_CONTINUOUS_COVERAGE_BELOW_0_5_SECONDS",
        )
        self.comparison_complete.add(condition)
        self.comparison_condition = None
        self.comparison_started_ns = None
        self.comparison_started_source_ns = None
        self.comparison_target_j2_rad = None
        self.comparison_last_sample_ns = None

    def _capture_comparison(self, state: Mapping[str, Any], now_ns: int) -> None:
        assert self.comparison_condition is not None
        assert self.comparison_started_ns is not None
        assert self.comparison_started_source_ns is not None
        assert self.comparison_target_j2_rad is not None
        sample_source_ns = int(state["source_monotonic_ns"])
        expected_scale = 0.0 if self.comparison_condition == "WITHOUT_FF" else 1.0
        _require(
            self._fresh_confirmation(
                now_ns, None if expected_scale == 0.0 else 1.0
            ),
            "OPERATOR_CONFIRMATION_STALE_OR_WRONG_GRAVITY_TARGET",
        )
        if self.comparison_last_sample_ns is not None:
            _require(
                0 < sample_source_ns - self.comparison_last_sample_ns
                <= MAXIMUM_COMPARISON_SAMPLE_GAP_NS,
                "COMPARISON_SAMPLE_GAP_EXCEEDED_100MS",
            )
        self.comparison_last_sample_ns = sample_source_ns
        condition = self.comparison_condition
        expected_scale = 0.0 if condition == "WITHOUT_FF" else 1.0
        _require(self.latest_gravity is not None, "GRAVITY_STATUS_MISSING")
        _require(
            self.latest_gravity.get("gravity_scale_target") == expected_scale
            and abs(float(self.latest_gravity.get("gravity_scale")) - expected_scale) <= 1.0e-6,
            "COMPARISON_GRAVITY_SCALE_CHANGED",
        )
        modes = state["controller_mode_by_motor"]
        _require(
            all(modes[motor] == "hold" for motor in MOTOR_NAMES),
            "COMPARISON_NOT_WHOLE_ARM_CONTINUOUS_HOLD",
        )
        _require(
            self.comparison_frozen_pose_rad is not None
            and self.comparison_frozen_target_j2_rad is not None,
            "COMPARISON_FROZEN_POSE_MISSING",
        )
        pose_errors = [
            abs(_rad_to_deg(actual - target))
            for actual, target in zip(
                state["position_rad"], self.comparison_frozen_pose_rad
            )
        ]
        _require(
            max(pose_errors) <= ENDPOINT_ERROR_DEG,
            "COMPARISON_ACTUAL_POSE_LEFT_FROZEN_0_5_DEG_WINDOW",
        )
        a = state["per_motor"]["J2A"]
        b = state["per_motor"]["J2B"]
        a_ff = _finite(a.get("gravity_feedforward_rotor_nm"), "COMPARISON_J2A_GRAVITY_INVALID")
        b_ff = _finite(b.get("gravity_feedforward_rotor_nm"), "COMPARISON_J2B_GRAVITY_INVALID")
        a_cmd = _finite(a.get("tau_cmd_rotor_nm"), "COMPARISON_J2A_COMMAND_INVALID")
        b_cmd = _finite(b.get("tau_cmd_rotor_nm"), "COMPARISON_J2B_COMMAND_INVALID")
        a_pd = a_cmd - a_ff
        b_pd = b_cmd - b_ff
        opposition = any(
            abs(ff) >= 0.05
            and ff * pd < 0.0
            and abs(pd) >= 0.8 * abs(ff)
            for ff, pd in ((a_ff, a_pd), (b_ff, b_pd))
        )
        _require(not opposition, "J2_GRAVITY_PD_INTERNAL_OPPOSITION")
        no_progress = state.get("no_progress_status_by_motor", {})
        saturation = bool(
            isinstance(no_progress, Mapping)
            and any(
                isinstance(no_progress.get(motor), Mapping)
                and no_progress[motor].get("software_saturation_observed") is True
                for motor in ("J2A", "J2B")
            )
        )
        self.comparison_rows.append({
            "timestamp_utc": _utc_text(),
            "monotonic_ns": sample_source_ns,
            "receipt_monotonic_ns": now_ns,
            "condition": condition,
            "window_elapsed_s": (
                sample_source_ns - self.comparison_started_source_ns
            ) / 1.0e9,
            "position_error_deg": _rad_to_deg(self.comparison_target_j2_rad - state["position_rad"][1]),
            "j2_e_sync_deg": _rad_to_deg(state["j2_e_sync_rad"]),
            "j2a_tau_feedback_rotor_nm": _finite(a.get("tau_feedback_rotor_nm"), "COMPARISON_J2A_FEEDBACK_INVALID"),
            "j2b_tau_feedback_rotor_nm": _finite(b.get("tau_feedback_rotor_nm"), "COMPARISON_J2B_FEEDBACK_INVALID"),
            "j2a_pd_rotor_nm": a_pd,
            "j2b_pd_rotor_nm": b_pd,
            "j2a_gravity_ff_rotor_nm": a_ff,
            "j2b_gravity_ff_rotor_nm": b_ff,
            "j2a_total_command_rotor_nm": a_cmd,
            "j2b_total_command_rotor_nm": b_cmd,
            "j2a_temperature_c": a["temperature_c"],
            "j2b_temperature_c": b["temperature_c"],
            "saturation_observed": saturation,
            "internal_opposition_detected": False,
            "status": "PASS",
            "reason": "",
            "target_j2_rad": self.comparison_frozen_target_j2_rad,
            "frozen_pose_rad_json": json.dumps(
                list(self.comparison_frozen_pose_rad),
                separators=(",", ":"),
            ),
            "frozen_pose_sha256": _document_sha256({
                "target_j2_rad": self.comparison_frozen_target_j2_rad,
                "pose_rad": list(self.comparison_frozen_pose_rad),
                "initial_hardware_state_sha256": (
                    self.comparison_frozen_hardware_state_sha256
                ),
            }),
            "frozen_initial_hardware_state_sha256": (
                self.comparison_frozen_hardware_state_sha256
            ),
            "actual_pose_rad_json": json.dumps(
                list(state["position_rad"]), separators=(",", ":")
            ),
            "maximum_pose_error_deg": max(pose_errors),
            **self._binding_columns(state),
        })

    def start_thermal_stage(
        self, duration_min: int, target_j2_rad: float, *, now_ns: int,
    ) -> None:
        _require(duration_min in THERMAL_STAGE_SECONDS, "THERMAL_STAGE_INVALID")
        _require(self.failure is None, "RUNNER_ALREADY_FAILED")
        _require(self.position_complete, "THERMAL_REQUIRES_ALL_SINGLE_JOINT_POSITION_PASS")
        _require(
            "THERMAL_SETUP" in self.completed_post_execution
            and self.thermal_setup_target_j2_rad is not None,
            "THERMAL_REQUIRES_OBSERVED_J2_REPRESENTATIVE_POSE",
        )
        required_phase = {
            5: "AWAIT_THERMAL_5",
            15: "AWAIT_THERMAL_15",
            30: "AWAIT_THERMAL_30",
        }[duration_min]
        _require(
            self.post_workflow_phase == required_phase,
            f"THERMAL_STAGE_ORDER_REQUIRES_{required_phase}",
        )
        _require(self.thermal_stage is None and self.thermal_pending_review is None, "THERMAL_STAGE_ALREADY_ACTIVE_OR_PENDING")
        required_previous = {5: set(), 15: {5}, 30: {5, 15}}[duration_min]
        _require(required_previous.issubset(self.thermal_approved), "THERMAL_PREVIOUS_STAGE_NOT_APPROVED")
        _require(duration_min not in self.thermal_approved, "THERMAL_STAGE_ALREADY_APPROVED")
        _require(self._fresh_confirmation(now_ns, 1.0), "OPERATOR_CONFIRMATION_STALE")
        self._require_full_gravity_position_authority(now_ns)
        _finite(target_j2_rad, "THERMAL_TARGET_INVALID")
        _require(
            abs(float(target_j2_rad) - self.thermal_setup_target_j2_rad)
            <= math.radians(0.01),
            "THERMAL_TARGET_NOT_OBSERVED_SETUP_POSE",
        )
        _require(self.latest_hardware is not None, "HARDWARE_STATE_MISSING")
        for motor in ("J2A", "J2B"):
            _require(
                self.latest_hardware["per_motor"][motor]["temperature_c"] < 55.0,
                "THERMAL_STAGE_ENTRY_TEMPERATURE_NOT_BELOW_55C",
            )
        self.thermal_stage = duration_min
        self.thermal_started_ns = now_ns
        self.thermal_started_source_ns = int(
            self.latest_hardware["source_monotonic_ns"]
        )
        self.thermal_target_j2_rad = float(target_j2_rad)
        self.thermal_last_sample_ns = None
        self.thermal_rows[duration_min] = []

    def _capture_thermal(self, state: Mapping[str, Any], now_ns: int) -> None:
        assert self.thermal_stage is not None
        assert self.thermal_started_ns is not None
        assert self.thermal_started_source_ns is not None
        assert self.thermal_target_j2_rad is not None
        sample_source_ns = int(state["source_monotonic_ns"])
        _require(self._fresh_confirmation(now_ns, 1.0), "OPERATOR_CONFIRMATION_STALE")
        self._require_full_gravity_position_authority(now_ns)
        modes = state["controller_mode_by_motor"]
        _require(
            all(modes[motor] == "hold" for motor in MOTOR_NAMES),
            "THERMAL_NOT_WHOLE_ARM_CONTINUOUS_HOLD",
        )
        _require(
            abs(_rad_to_deg(
                self.thermal_target_j2_rad - state["position_rad"][1]
            )) <= ENDPOINT_ERROR_DEG,
            "THERMAL_REPRESENTATIVE_POSE_ERROR_EXCEEDED_0_5_DEG",
        )
        if (
            self.thermal_last_sample_ns is not None
            and sample_source_ns - self.thermal_last_sample_ns
            < THERMAL_SAMPLE_PERIOD_NS
        ):
            return
        self.thermal_last_sample_ns = sample_source_ns
        a = state["per_motor"]["J2A"]
        b = state["per_motor"]["J2B"]
        rows = self.thermal_rows[self.thermal_stage]
        a_temperature = float(a["temperature_c"])
        b_temperature = float(b["temperature_c"])
        if rows:
            start_max = max(
                float(rows[0]["j2a_temperature_c"]),
                float(rows[0]["j2b_temperature_c"]),
            )
            _require(
                max(a_temperature, b_temperature) - start_max
                <= self.binding.maximum_stage_temperature_rise_c,
                "THERMAL_STAGE_RAPID_OR_EXCESSIVE_RISE",
            )
        a_ff = _finite(a.get("gravity_feedforward_rotor_nm"), "THERMAL_J2A_GRAVITY_INVALID")
        b_ff = _finite(b.get("gravity_feedforward_rotor_nm"), "THERMAL_J2B_GRAVITY_INVALID")
        a_cmd = _finite(a.get("tau_cmd_rotor_nm"), "THERMAL_J2A_COMMAND_INVALID")
        b_cmd = _finite(b.get("tau_cmd_rotor_nm"), "THERMAL_J2B_COMMAND_INVALID")
        thermal_status = state.get("thermal_status_by_motor", {})
        no_progress = state.get("no_progress_status_by_motor", {})
        a_reported_slope = self._optional_metadata_number(
            thermal_status, "J2A", "slope_c_per_min"
        )
        b_reported_slope = self._optional_metadata_number(
            thermal_status, "J2B", "slope_c_per_min"
        )
        a_slope = (
            self._derived_temperature_slope(
                rows, sample_source_ns, a_temperature, "j2a_temperature_c"
            )
            if a_reported_slope is None else a_reported_slope
        )
        b_slope = (
            self._derived_temperature_slope(
                rows, sample_source_ns, b_temperature, "j2b_temperature_c"
            )
            if b_reported_slope is None else b_reported_slope
        )
        saturation = any(
            isinstance(no_progress, Mapping)
            and isinstance(no_progress.get(motor), Mapping)
            and no_progress[motor].get("software_saturation_observed") is True
            for motor in ("J2A", "J2B")
        )
        rows.append({
            "timestamp_utc": _utc_text(),
            "monotonic_ns": sample_source_ns,
            "receipt_monotonic_ns": now_ns,
            "elapsed_s": (
                sample_source_ns - self.thermal_started_source_ns
            ) / 1.0e9,
            "j2a_temperature_c": a_temperature,
            "j2b_temperature_c": b_temperature,
            "j2a_slope_c_per_min": a_slope,
            "j2b_slope_c_per_min": b_slope,
            "j2a_tau_feedback_rotor_nm": _finite(a.get("tau_feedback_rotor_nm"), "THERMAL_J2A_FEEDBACK_INVALID"),
            "j2b_tau_feedback_rotor_nm": _finite(b.get("tau_feedback_rotor_nm"), "THERMAL_J2B_FEEDBACK_INVALID"),
            "j2a_gravity_ff_rotor_nm": a_ff,
            "j2b_gravity_ff_rotor_nm": b_ff,
            "j2a_pd_rotor_nm": a_cmd - a_ff,
            "j2b_pd_rotor_nm": b_cmd - b_ff,
            "position_error_deg": _rad_to_deg(self.thermal_target_j2_rad - state["position_rad"][1]),
            "j2_e_sync_deg": _rad_to_deg(state["j2_e_sync_rad"]),
            "saturation_observed": saturation,
            "j2a_merror": a["merror"],
            "j2b_merror": b["merror"],
            "j2a_communication_ok": a["communication_ok"],
            "j2b_communication_ok": b["communication_ok"],
            "status": "PASS",
            "reason": "",
            **self._binding_columns(state),
        })
        required_ns = int(THERMAL_STAGE_SECONDS[self.thermal_stage] * 1.0e9)
        if sample_source_ns - self.thermal_started_source_ns >= required_ns:
            self.thermal_pending_review = self.thermal_stage
            self.thermal_stage = None
            self.thermal_started_ns = None
            self.thermal_started_source_ns = None
            self.thermal_target_j2_rad = None
            self.thermal_last_sample_ns = None

    @staticmethod
    def _metadata_number(
        mapping: object, motor: str, field: str,
    ) -> float:
        _require(isinstance(mapping, Mapping), f"{motor}_{field}_MISSING")
        item = mapping.get(motor)
        _require(isinstance(item, Mapping), f"{motor}_{field}_MISSING")
        return _finite(item.get(field), f"{motor}_{field}_INVALID")

    @staticmethod
    def _optional_metadata_number(
        mapping: object, motor: str, field: str,
    ) -> Optional[float]:
        _require(isinstance(mapping, Mapping), f"{motor}_{field}_MISSING")
        item = mapping.get(motor)
        _require(isinstance(item, Mapping), f"{motor}_{field}_MISSING")
        value = item.get(field)
        return None if value is None else _finite(value, f"{motor}_{field}_INVALID")

    @staticmethod
    def _derived_temperature_slope(
        rows: Sequence[Mapping[str, Any]], sample_ns: int,
        temperature_c: float, temperature_field: str,
    ) -> float:
        if not rows:
            return 0.0
        elapsed_ns = sample_ns - int(rows[0]["monotonic_ns"])
        _require(elapsed_ns >= 0, "TEMPERATURE_SLOPE_TIME_REGRESSION")
        if elapsed_ns == 0:
            return 0.0
        start_c = _finite(
            rows[0][temperature_field], "TEMPERATURE_SLOPE_START_INVALID"
        )
        return (temperature_c - start_c) * 60.0e9 / elapsed_ns

    def approve_thermal_stage(self, duration_min: int, *, now_ns: int) -> None:
        _require(self.thermal_pending_review == duration_min, "THERMAL_STAGE_NOT_PENDING_REVIEW")
        _require(self._fresh_confirmation(now_ns, 1.0), "OPERATOR_CONFIRMATION_STALE")
        rows = self.thermal_rows[duration_min]
        self._validate_thermal_rows(duration_min, rows)
        _require(
            float(rows[-1]["elapsed_s"]) >= THERMAL_STAGE_SECONDS[duration_min],
            "THERMAL_STAGE_DURATION_INCOMPLETE",
        )
        self.thermal_approved.add(duration_min)
        self.thermal_pending_review = None
        self.post_workflow_phase = {
            5: "AWAIT_THERMAL_15",
            15: "AWAIT_THERMAL_30",
            30: "AWAIT_THERMAL_SUMMARY",
        }[duration_min]

    @staticmethod
    def _validate_thermal_rows(
        duration_min: int, rows: Sequence[Mapping[str, Any]],
    ) -> None:
        _require(rows, "THERMAL_STAGE_HAS_NO_ROWS")
        required_seconds = THERMAL_STAGE_SECONDS[duration_min]
        minimum_samples = math.ceil(required_seconds / 2.0) + 1
        _require(
            len(rows) >= minimum_samples,
            "THERMAL_CONTINUOUS_SAMPLE_COUNT_INSUFFICIENT",
        )
        stamps = [
            int(row["hardware_state_source_monotonic_ns"])
            for row in rows
        ]
        elapsed = [float(row["elapsed_s"]) for row in rows]
        _require(
            all(
                0 < current - previous <= MAXIMUM_STREAM_GAP_NS
                for previous, current in zip(stamps, stamps[1:])
            ),
            "THERMAL_SAMPLE_GAP_EXCEEDED_2_SECONDS",
        )
        base_stamp = stamps[0]
        base_elapsed = elapsed[0]
        _require(
            all(
                abs(
                    (stamp - base_stamp) / 1.0e9
                    - (elapsed_value - base_elapsed)
                ) <= 1.0e-6
                for stamp, elapsed_value in zip(stamps, elapsed)
            ),
            "THERMAL_ELAPSED_MONOTONIC_MISMATCH",
        )
        _require(
            elapsed[0] <= 1.0
            and elapsed[-1] - elapsed[0] >= required_seconds,
            "THERMAL_CONTINUOUS_DURATION_INCOMPLETE",
        )

    def finalize_thermal_summary(
        self,
        *,
        classification: str,
        mechanical_recommendation: Optional[Mapping[str, Any]] = None,
        now_ns: Optional[int] = None,
    ) -> dict[str, Any]:
        classification = classification.strip().upper()
        _require(
            classification in {
                "CONTROL_EXCESS_TORQUE_SOLVED",
                "FUNDAMENTAL_CONTINUOUS_LOAD_LIMIT",
            },
            "THERMAL_CLASSIFICATION_INVALID",
        )
        if classification == "CONTROL_EXCESS_TORQUE_SOLVED":
            _require(
                self.post_workflow_phase == "AWAIT_THERMAL_SUMMARY"
                and self.thermal_approved == {5, 15, 30},
                "CLASSIFICATION_A_REQUIRES_ALL_5_15_30_STAGES",
            )
        else:
            _require(
                self.post_workflow_phase in {
                    "AWAIT_THERMAL_15",
                    "AWAIT_THERMAL_30",
                    "AWAIT_THERMAL_SUMMARY",
                }
                and self.thermal_approved in (
                    {5}, {5, 15}, {5, 15, 30}
                ),
                "CLASSIFICATION_B_REQUIRES_COMPLETED_5_OR_15_OR_30_MINUTE_WINDOW",
            )
            if self.thermal_approved != {5, 15, 30}:
                _require(
                    type(now_ns) is int,
                    "CLASSIFICATION_B_EARLY_REQUIRES_CURRENT_MONOTONIC_TIME",
                )
                self._require_safe_early_fundamental_limit(now_ns)
                for stage in (5, 15, 30):
                    if stage not in self.thermal_approved:
                        self.thermal_not_run_stages[stage] = (
                            "NOT_RUN_EARLY_FUNDAMENTAL_LIMIT"
                        )
        groups = {
            condition: [row for row in self.comparison_rows if row["condition"] == condition]
            for condition in ("WITHOUT_FF", "WITH_FF")
        }
        _require(all(groups.values()), "THERMAL_SUMMARY_REQUIRES_J2_COMPARISON")
        _require(
            self.comparison_complete == {"WITHOUT_FF", "WITH_FF"},
            "THERMAL_SUMMARY_REQUIRES_COMPLETED_J2_COMPARISON",
        )
        before_torque = self._mean_peak_abs_feedback(groups["WITHOUT_FF"])
        after_torque = self._mean_peak_abs_feedback(groups["WITH_FF"])
        before_sat = sum(bool(row["saturation_observed"]) for row in groups["WITHOUT_FF"]) / len(groups["WITHOUT_FF"])
        after_sat = sum(bool(row["saturation_observed"]) for row in groups["WITH_FF"]) / len(groups["WITH_FF"])
        quality = self._classification_a_quality(groups)
        if classification == "CONTROL_EXCESS_TORQUE_SOLVED":
            _require(
                all(quality["criteria"].values()),
                "CLASSIFICATION_A_OBJECTIVE_IMPROVEMENT_NOT_PROVEN",
            )
            result = "PASS"
            counterbalance = "NO"
        else:
            _require(
                not all(quality["criteria"].values()),
                "CLASSIFICATION_B_CONTRADICTS_PROVEN_CLASSIFICATION_A",
            )
            mechanical_recommendation = self._validate_mechanical_recommendation(
                mechanical_recommendation, groups["WITH_FF"]
            )
            result = "HARDWARE_COUNTERBALANCE_REQUIRED"
            counterbalance = "YES"
        motors = {}
        for motor, prefix in (("J2A", "j2a"), ("J2B", "j2b")):
            completed = [
                self.thermal_rows[stage]
                for stage in (5, 15, 30)
                if self.thermal_rows[stage]
            ]
            _require(completed, f"THERMAL_SUMMARY_{motor}_NO_DATA")
            motors[motor] = {
                "start_temperature_c": float(completed[0][0][f"{prefix}_temperature_c"]),
                "temperature_5min_c": self._thermal_endpoint(5, f"{prefix}_temperature_c"),
                "temperature_15min_c": self._thermal_endpoint(15, f"{prefix}_temperature_c"),
                "temperature_30min_c": self._thermal_endpoint(30, f"{prefix}_temperature_c"),
                "final_slope_c_per_min": float(completed[-1][-1][f"{prefix}_slope_c_per_min"]),
                "final_slope_observed_stage_min": max(
                    stage for stage in (5, 15, 30)
                    if self.thermal_rows[stage]
                ),
            }
        document: dict[str, Any] = {
            "schema": "V15.31B-thermal-summary-v1",
            "result": result,
            "classification": classification,
            "counterbalance_required": counterbalance,
            "J2A": motors["J2A"],
            "J2B": motors["J2B"],
            "j2_sustained_rotor_torque": {
                "before_gravity_nm": before_torque,
                "after_gravity_nm": after_torque,
            },
            "saturation_ratio": {"before": before_sat, "after": after_sat},
            "classification_a_objective_metrics": quality,
            "thermal_stage_disposition": {
                str(stage): (
                    "PASS_OBSERVED"
                    if stage in self.thermal_approved
                    else self.thermal_not_run_stages.get(stage, "UNAVAILABLE")
                )
                for stage in (5, 15, 30)
            },
            "binding": {
                "envelope_id": self.binding.envelope_id,
                "envelope_sha256": self.binding.envelope_sha256,
                "session_id": self.binding.session_id,
                "state_instance_id": self.binding.state_instance_id,
                "anchor_sha256": self.binding.anchor_sha256,
            },
        }
        if mechanical_recommendation is not None:
            document["mechanical_recommendation"] = dict(mechanical_recommendation)
        self.thermal_summary = document
        self.post_workflow_phase = "AWAIT_RESTORE_INITIAL"
        return document

    def _validate_mechanical_recommendation(
        self,
        value: object,
        with_ff_rows: Sequence[Mapping[str, Any]],
    ) -> dict[str, Any]:
        _require(isinstance(value, Mapping), "CLASSIFICATION_B_REQUIRES_MECHANICAL_RECOMMENDATION")
        _require(
            value.get("schema")
            == "V15.31B-mechanical-counterbalance-recommendation-v1",
            "CLASSIFICATION_B_MECHANICAL_RECOMMENDATION_SCHEMA_INVALID",
        )
        mean_rotor = {
            motor: sum(
                abs(float(row[field])) for row in with_ff_rows
            ) / len(with_ff_rows)
            for motor, field in (
                ("J2A", "j2a_gravity_ff_rotor_nm"),
                ("J2B", "j2b_gravity_ff_rotor_nm"),
            )
        }
        observed_joint_nm = (
            mean_rotor["J2A"] + mean_rotor["J2B"]
        ) * GO_GEAR_RATIO
        target_rotor = {
            motor: demand * (1.0 - MECHANICAL_UNLOAD_FRACTION)
            for motor, demand in mean_rotor.items()
        }
        target_joint_nm = (
            target_rotor["J2A"] + target_rotor["J2B"]
        ) * GO_GEAR_RATIO
        reduction = observed_joint_nm - target_joint_nm
        caller_reduction = _finite(
            value.get("required_j2_torque_reduction_nm"),
            "CLASSIFICATION_B_TORQUE_REDUCTION_INVALID",
        )
        _require(
            observed_joint_nm > 0.0
            and reduction > 0.0
            and math.isclose(
                caller_reduction,
                reduction,
                rel_tol=1.0e-6,
                abs_tol=1.0e-6,
            ),
            "CLASSIFICATION_B_REDUCTION_NOT_RUNNER_DERIVED_20_PERCENT_UNLOAD",
        )
        checked_options: dict[str, dict[str, float]] = {}
        option_specs = {
            "counterweight": (
                ("mass_kg", "lever_arm_m", "gravity_m_s2", "estimated_torque_nm"),
                lambda option: option["mass_kg"] * option["gravity_m_s2"] * option["lever_arm_m"],
            ),
            "spring": (
                ("spring_rate_n_per_m", "deflection_m", "lever_arm_m", "estimated_torque_nm"),
                lambda option: option["spring_rate_n_per_m"] * option["deflection_m"] * option["lever_arm_m"],
            ),
            "gas_spring": (
                ("force_n", "lever_arm_m", "estimated_torque_nm"),
                lambda option: option["force_n"] * option["lever_arm_m"],
            ),
        }
        for name, (fields, formula) in option_specs.items():
            option_value = value.get(name)
            if option_value is None:
                continue
            _require(isinstance(option_value, Mapping), f"CLASSIFICATION_B_{name.upper()}_INVALID")
            option = {
                field: _finite(
                    option_value.get(field),
                    f"CLASSIFICATION_B_{name.upper()}_{field.upper()}_INVALID",
                )
                for field in fields
            }
            _require(
                all(number > 0.0 for number in option.values()),
                f"CLASSIFICATION_B_{name.upper()}_VALUES_NOT_POSITIVE",
            )
            if name == "counterweight":
                _require(
                    abs(option["gravity_m_s2"] - STANDARD_GRAVITY_M_S2)
                    <= 1.0e-6,
                    "CLASSIFICATION_B_COUNTERWEIGHT_GRAVITY_CONSTANT_INVALID",
                )
            calculated = float(formula(option))
            _require(
                math.isclose(
                    option["estimated_torque_nm"],
                    calculated,
                    rel_tol=1.0e-6,
                    abs_tol=1.0e-6,
                )
                and reduction <= calculated <= reduction * 1.25,
                f"CLASSIFICATION_B_{name.upper()}_TORQUE_FORMULA_OR_RANGE_INVALID",
            )
            checked_options[name] = option
        _require(checked_options, "CLASSIFICATION_B_QUANTITATIVE_OPTION_MISSING")
        return {
            "schema": value["schema"],
            "required_j2_torque_reduction_nm": reduction,
            "observed_load_basis": {
                "per_motor_mean_abs_gravity_rotor_nm": dict(mean_rotor),
                "per_motor_target_abs_gravity_rotor_nm": dict(
                    target_rotor
                ),
                "go_gear_ratio": GO_GEAR_RATIO,
                "observed_j2_gravity_joint_nm": observed_joint_nm,
                "target_j2_gravity_joint_nm": target_joint_nm,
                "minimum_mechanical_unload_fraction": (
                    MECHANICAL_UNLOAD_FRACTION
                ),
                "joint_torque_formula": (
                    "(|J2A_rotor_Nm|+|J2B_rotor_Nm|)*GO_GEAR_RATIO"
                ),
                "required_reduction_formula": (
                    "observed_j2_gravity_joint_nm-"
                    "target_j2_gravity_joint_nm"
                ),
                "authority": (
                    "EMPIRICAL_REVALIDATION_TARGET_NOT_OFFICIAL_"
                    "CONTINUOUS_RATING"
                ),
            },
            **checked_options,
        }

    def _require_safe_early_fundamental_limit(self, now_ns: int) -> None:
        """Allow a non-fault early B only from a safe, sustained hot trend."""

        _require(self.latest_hardware is not None, "EARLY_B_HARDWARE_STATE_MISSING")
        _require(
            self.latest_hardware_received_ns is not None
            and 0 <= now_ns - self.latest_hardware_received_ns
            <= MAXIMUM_STATE_AGE_NS,
            "EARLY_B_HARDWARE_STATE_STALE",
        )
        _require(self._fresh_confirmation(now_ns, 1.0), "OPERATOR_CONFIRMATION_STALE")
        self._require_full_gravity_position_authority(now_ns)
        state = self.latest_hardware
        _require(
            all(
                state["controller_mode_by_motor"][motor] == "hold"
                for motor in MOTOR_NAMES
            ),
            "EARLY_B_REQUIRES_SAFE_WHOLE_ARM_HOLD",
        )
        _require(
            abs(_rad_to_deg(state["j2_e_sync_rad"]))
            <= J2_SYNC_WARNING_DEG,
            "EARLY_B_J2_SYNC_NOT_SAFE",
        )
        for motor in ("J2A", "J2B"):
            sample = state["per_motor"][motor]
            _require(
                sample.get("communication_ok") is True
                and sample.get("merror") == 0
                and sample.get("load_limit_no_progress") is False
                and sample.get("thermal_fault_latched") is False
                and float(sample.get("temperature_c")) < 55.0,
                f"EARLY_B_{motor}_NOT_SAFE_BELOW_55C",
            )
        stage = max(self.thermal_approved)
        rows = self.thermal_rows[stage]
        self._validate_thermal_rows(stage, rows)
        final_ns = int(rows[-1]["monotonic_ns"])
        trend_rows = [
            row for row in rows
            if final_ns - int(row["monotonic_ns"]) <= 60_000_000_000
        ]
        _require(
            len(trend_rows) >= 31
            and int(trend_rows[-1]["monotonic_ns"])
            - int(trend_rows[0]["monotonic_ns"])
            >= 60_000_000_000,
            "EARLY_B_SUSTAINED_TREND_WINDOW_BELOW_60_SECONDS",
        )
        _require(
            all(
                float(row[field])
                >= MINIMUM_EARLY_FUNDAMENTAL_SLOPE_C_PER_MIN
                for row in trend_rows
                for field in (
                    "j2a_slope_c_per_min", "j2b_slope_c_per_min",
                )
            ),
            "EARLY_B_SUSTAINED_THERMAL_SLOPE_NOT_PROVEN",
        )
        rises = [
            float(rows[-1][field]) - float(rows[0][field])
            for field in ("j2a_temperature_c", "j2b_temperature_c")
        ]
        _require(
            min(rises) >= MINIMUM_EARLY_FUNDAMENTAL_RISE_C,
            "EARLY_B_MINIMUM_OBSERVED_TEMPERATURE_RISE_NOT_PROVEN",
        )

    def _thermal_endpoint(self, stage: int, field: str) -> Any:
        rows = self.thermal_rows[stage]
        if rows:
            return float(rows[-1][field])
        if stage in self.thermal_not_run_stages:
            completed_stage = max(
                item for item in (5, 15, 30) if self.thermal_rows[item]
            )
            return {
                "status": self.thermal_not_run_stages[stage],
                "observed": False,
                "value_c": None,
                "last_observed_stage_min": completed_stage,
                "last_observed_value_c": float(
                    self.thermal_rows[completed_stage][-1][field]
                ),
            }
        completed = [self.thermal_rows[item] for item in (5, 15, 30) if self.thermal_rows[item]]
        _require(completed, "THERMAL_ENDPOINT_UNAVAILABLE")
        return float(completed[-1][-1][field])

    @staticmethod
    def _mean_peak_abs_feedback(rows: Sequence[Mapping[str, Any]]) -> float:
        return max(
            sum(abs(float(row[field])) for row in rows) / len(rows)
            for field in (
                "j2a_tau_feedback_rotor_nm",
                "j2b_tau_feedback_rotor_nm",
            )
        )

    @staticmethod
    def _mean_and_peak_abs(
        rows: Sequence[Mapping[str, Any]], fields: Sequence[str],
    ) -> tuple[float, float]:
        values = [
            abs(float(row[field]))
            for row in rows
            for field in fields
        ]
        _require(bool(values), "OBJECTIVE_METRIC_HAS_NO_SAMPLES")
        return sum(values) / len(values), max(values)

    @staticmethod
    def _materially_lower(before: float, after: float) -> bool:
        required = max(
            MINIMUM_ABSOLUTE_PD_IMPROVEMENT_NM,
            before * MINIMUM_RELATIVE_PD_IMPROVEMENT,
        )
        return after < before and before - after >= required

    def _classification_a_quality(
        self, groups: Mapping[str, Sequence[Mapping[str, Any]]],
    ) -> dict[str, Any]:
        before = groups["WITHOUT_FF"]
        after = groups["WITH_FF"]
        before_error_mean, before_error_peak = self._mean_and_peak_abs(
            before, ("position_error_deg",)
        )
        after_error_mean, after_error_peak = self._mean_and_peak_abs(
            after, ("position_error_deg",)
        )
        before_pd_mean, before_pd_peak = self._mean_and_peak_abs(
            before, ("j2a_pd_rotor_nm", "j2b_pd_rotor_nm")
        )
        after_pd_mean, after_pd_peak = self._mean_and_peak_abs(
            after, ("j2a_pd_rotor_nm", "j2b_pd_rotor_nm")
        )
        before_feedback_mean, before_feedback_peak = self._mean_and_peak_abs(
            before,
            ("j2a_tau_feedback_rotor_nm", "j2b_tau_feedback_rotor_nm"),
        )
        after_feedback_mean, after_feedback_peak = self._mean_and_peak_abs(
            after,
            ("j2a_tau_feedback_rotor_nm", "j2b_tau_feedback_rotor_nm"),
        )
        before_sat = sum(bool(row["saturation_observed"]) for row in before) / len(before)
        after_sat = sum(bool(row["saturation_observed"]) for row in after) / len(after)
        terminal_slopes = {
            stage: max(
                abs(float(self.thermal_rows[stage][-1][field]))
                for field in (
                    "j2a_slope_c_per_min",
                    "j2b_slope_c_per_min",
                )
            )
            for stage in (5, 15, 30)
            if self.thermal_rows[stage]
        }
        rows_30 = self.thermal_rows[30]
        stable_30 = bool(rows_30) and all(
            not bool(row["saturation_observed"])
            and abs(float(row["position_error_deg"])) <= ENDPOINT_ERROR_DEG
            and abs(float(row["j2_e_sync_deg"])) <= J2_SYNC_WARNING_DEG
            and int(row["j2a_merror"]) == 0
            and int(row["j2b_merror"]) == 0
            and bool(row["j2a_communication_ok"])
            and bool(row["j2b_communication_ok"])
            for row in rows_30
        )
        slope_improved = (
            set(terminal_slopes) == {5, 15, 30}
            and
            terminal_slopes[30] <= MAXIMUM_STABLE_THERMAL_SLOPE_C_PER_MIN
            and terminal_slopes[30] <= terminal_slopes[15]
            and terminal_slopes[15] <= terminal_slopes[5]
            and terminal_slopes[5] - terminal_slopes[30]
            >= MINIMUM_THERMAL_SLOPE_IMPROVEMENT_C_PER_MIN
        )
        criteria = {
            "mean_abs_position_error_strictly_lower": (
                after_error_mean < before_error_mean - 1.0e-9
            ),
            "peak_abs_position_error_strictly_lower": (
                after_error_peak < before_error_peak - 1.0e-9
            ),
            "mean_abs_pd_materially_lower": self._materially_lower(
                before_pd_mean, after_pd_mean
            ),
            "peak_abs_pd_materially_lower": self._materially_lower(
                before_pd_peak, after_pd_peak
            ),
            "mean_abs_feedback_torque_materially_lower": self._materially_lower(
                before_feedback_mean, after_feedback_mean
            ),
            "peak_abs_feedback_torque_materially_lower": self._materially_lower(
                before_feedback_peak, after_feedback_peak
            ),
            "saturation_strictly_lower_and_eliminated": (
                before_sat > after_sat and after_sat == 0.0
            ),
            "with_ff_no_internal_opposition": all(
                not bool(row["internal_opposition_detected"])
                for row in after
            ),
            "thermal_trend_improved": slope_improved,
            "thirty_minute_hold_stable": stable_30,
        }
        return {
            "thresholds": {
                "minimum_relative_pd_improvement": MINIMUM_RELATIVE_PD_IMPROVEMENT,
                "minimum_absolute_pd_improvement_nm": MINIMUM_ABSOLUTE_PD_IMPROVEMENT_NM,
                "maximum_stable_thermal_slope_c_per_min": MAXIMUM_STABLE_THERMAL_SLOPE_C_PER_MIN,
                "minimum_thermal_slope_improvement_c_per_min": MINIMUM_THERMAL_SLOPE_IMPROVEMENT_C_PER_MIN,
            },
            "before": {
                "mean_abs_position_error_deg": before_error_mean,
                "peak_abs_position_error_deg": before_error_peak,
                "mean_abs_pd_rotor_nm": before_pd_mean,
                "peak_abs_pd_rotor_nm": before_pd_peak,
                "saturation_ratio": before_sat,
                "mean_abs_feedback_rotor_nm": before_feedback_mean,
                "peak_abs_feedback_rotor_nm": before_feedback_peak,
            },
            "after": {
                "mean_abs_position_error_deg": after_error_mean,
                "peak_abs_position_error_deg": after_error_peak,
                "mean_abs_pd_rotor_nm": after_pd_mean,
                "peak_abs_pd_rotor_nm": after_pd_peak,
                "saturation_ratio": after_sat,
                "mean_abs_feedback_rotor_nm": after_feedback_mean,
                "peak_abs_feedback_rotor_nm": after_feedback_peak,
            },
            "terminal_slope_c_per_min": {
                str(stage): value for stage, value in terminal_slopes.items()
            },
            "criteria": criteria,
        }

    def _binding_columns(self, state: Mapping[str, Any]) -> dict[str, Any]:
        return {
            "envelope_id": self.binding.envelope_id,
            "envelope_sha256": self.binding.envelope_sha256,
            "session_id": self.binding.session_id,
            "state_instance_id": self.binding.state_instance_id,
            "anchor_sha256": self.binding.anchor_sha256,
            "hardware_state_sha256": _document_sha256(state),
            "hardware_state_sequence": state["sequence"],
            "hardware_state_source_monotonic_ns": state[
                "source_monotonic_ns"
            ],
        }

    def record_multi_joint_attestation(self, document: object) -> None:
        """Accept UI-only facts; encoder/Router facts come from observation."""

        _require(
            "MULTI_JOINT" in self.completed_post_execution,
            "MULTI_JOINT_UI_ATTESTATION_BEFORE_MACHINE_OBSERVATION",
        )
        _require(isinstance(document, Mapping), "MULTI_JOINT_ATTESTATION_INVALID")
        binding = document.get("binding")
        _require(self._binding_matches(binding), "MULTI_JOINT_BINDING_MISMATCH")
        _require(document.get("explicit_user_submit") is True, "MULTI_JOINT_USER_SUBMIT_MISSING")
        _require(document.get("actual_twin_source") == "REAL_ENCODER", "MULTI_JOINT_ACTUAL_SOURCE_INVALID")
        for field in ("planned_preview", "preview_before_real", "planned_twin", "actual_twin"):
            _require(document.get(field) == "PASS", f"MULTI_JOINT_{field.upper()}_NOT_PASS")
        _require(
            "start_deg" not in document
            and "target_deg" not in document
            and "final_deg" not in document
            and "max_error_deg_by_joint" not in document
            and "minimum_dwell_s" not in document
            and "segments" not in document,
            "MULTI_JOINT_ATTESTATION_MUST_NOT_SUPPLY_MACHINE_EVIDENCE",
        )
        self.multi_joint_ui_attestation = dict(document)
        self._maybe_build_multi_joint_document()

    def record_restore_attestation(self, document: object) -> None:
        _require(
            self.completed_post_order
            and self.completed_post_order[-1] == "RESTORE_INITIAL",
            "RESTORE_UI_ATTESTATION_BEFORE_MACHINE_OBSERVATION",
        )
        _require(isinstance(document, Mapping), "RESTORE_ATTESTATION_INVALID")
        _require(
            self._binding_matches(document.get("binding")),
            "RESTORE_BINDING_MISMATCH",
        )
        _require(
            document.get("planned_preview") == "PASS"
            and document.get("explicit_user_submit") is True
            and document.get("actual_twin_source") == "REAL_ENCODER",
            "RESTORE_INITIAL_ATTESTATION_INVALID",
        )
        _require(
            not any(
                field in document
                for field in (
                    "target_deg", "final_deg", "max_error_deg_by_joint",
                    "minimum_dwell_s", "segments",
                )
            ),
            "RESTORE_ATTESTATION_MUST_NOT_SUPPLY_MACHINE_EVIDENCE",
        )
        self.restore_ui_attestation = dict(document)
        self._maybe_build_multi_joint_document()

    def record_gui_attestation(self, document: object) -> None:
        _require(isinstance(document, Mapping), "GUI_ATTESTATION_INVALID")
        _require(self._binding_matches(document.get("binding")), "GUI_BINDING_MISMATCH")
        checks = document.get("checks")
        required = {
            "single_slider_group", "drag_controls_planned_only",
            "planned_mujoco_animation", "actual_twin_encoder_only",
            "preview_before_real", "explicit_real_submit", "execution_status",
            "progress_percent", "estimated_remaining", "heartbeat",
            "temperature_table", "online_status_7_motors", "temperature_colors",
            "position_target_error",
        }
        _require(
            isinstance(checks, Mapping)
            and all(checks.get(field) == "PASS" for field in required),
            "GUI_POWERED_CHECK_NOT_PASS",
        )
        self.gui_document = dict(document)

    def request_final_brake(self, *, now_ns: int) -> None:
        """Finish through the Router without turning a successful run into FAIL."""

        _require(self.failure is None, "RUNNER_ALREADY_FAILED")
        _require(self.position_complete, "FINAL_BRAKE_REQUIRES_POSITION_COMPLETE")
        _require(self.thermal_summary is not None, "FINAL_BRAKE_REQUIRES_THERMAL_CLASSIFICATION")
        _require(
            self.post_workflow_phase == "AWAIT_FINAL_BRAKE",
            "FINAL_BRAKE_OUT_OF_ORDER_OR_RESTORE_NOT_LATEST",
        )
        _require(
            self.completed_post_order
            == ["MULTI_JOINT", "THERMAL_SETUP", "RESTORE_INITIAL"],
            "FINAL_BRAKE_REQUIRES_EXACT_POST_POSITION_ORDER",
        )
        _require(self.latest_hardware is not None, "FINAL_BRAKE_HARDWARE_STATE_MISSING")
        _require(
            self.latest_hardware_received_ns is not None
            and now_ns - self.latest_hardware_received_ns <= MAXIMUM_STATE_AGE_NS,
            "FINAL_BRAKE_RESTORE_STATE_STALE",
        )
        self._guard_restored_pose(self.latest_hardware, now_ns)
        _require(
            _valid_sha256(self.restore_guard_hardware_state_sha256),
            "FINAL_BRAKE_RESTORE_GUARD_NOT_OBSERVED",
        )
        _require(not self.final_brake_requested, "FINAL_BRAKE_ALREADY_REQUESTED")
        self.final_brake_requested = True
        self.final_brake_requested_ns = now_ns
        self.post_workflow_phase = "FINAL_BRAKE_REQUESTED"
        self.final_brake_dwell_started_ns = None
        self.final_brake_last_pair_ns = None
        self.final_brake_last_paired_raw_source_ns = None
        self.final_brake_pair_count = 0
        self.final_brake_dwell_seconds = 0.0
        self.final_brake_pairs = []
        self.j6_disabled_raw_observed_ns = None
        self.j6_disabled_raw_sha256 = None
        self.j6_disabled_raw_source_ns = None
        self.j6_disabled_raw_sequence = None
        self.j6_raw_source_instance_id = None
        self.j6_raw_last_sequence = None
        self.j6_raw_last_source_ns = None
        if self.brake_callback is not None:
            self.brake_callback(FailureRecord(
                "FINAL_ACCEPTANCE_COMPLETE",
                ("J1", "J2", "J345", "J6"),
                now_ns,
            ))

    def finalize_result(
        self,
        *,
        evidence_directory: Path,
    ) -> dict[str, Any]:
        """Build the exact 31-field report only after observed terminal safety."""

        _require(self.failure is None, "FINAL_RESULT_RUNNER_FAILED")
        _require(self.position_complete and len(self.position_rows) == 30, "FINAL_RESULT_POSITION_INCOMPLETE")
        _require(self.comparison_complete == {"WITHOUT_FF", "WITH_FF"}, "FINAL_RESULT_COMPARISON_INCOMPLETE")
        _require(self.thermal_summary is not None, "FINAL_RESULT_THERMAL_MISSING")
        classification = self.thermal_summary.get("classification")
        _require(
            (
                classification == "CONTROL_EXCESS_TORQUE_SOLVED"
                and self.thermal_summary.get("result") == "PASS"
            )
            or (
                classification == "FUNDAMENTAL_CONTINUOUS_LOAD_LIMIT"
                and self.thermal_summary.get("result")
                == "HARDWARE_COUNTERBALANCE_REQUIRED"
            ),
            "FINAL_RESULT_THERMAL_CLASSIFICATION_INVALID",
        )
        full_pass = classification == "CONTROL_EXCESS_TORQUE_SOLVED"
        _require(self.multi_joint_document is not None, "FINAL_RESULT_MULTI_JOINT_MISSING")
        _require(self.gui_document is not None, "FINAL_RESULT_GUI_MISSING")
        _require(
            self.final_brake_requested
            and self.final_brake_observed
            and _valid_sha256(self.final_brake_hardware_state_sha256),
            "FINAL_RESULT_BRAKE_NOT_OBSERVED",
        )
        _require(
            self.run_git_provenance is not None
            and self.preseal_git_provenance is not None,
            "FINAL_RESULT_GIT_PROVENANCE_NOT_CAPTURED",
        )
        implementation_commit = self.run_git_provenance.run_base_commit
        evidence_commit = self.run_git_provenance.run_base_commit
        directory = evidence_directory.resolve()
        supporting = {
            "power_on_readonly.json": "Power-on read-only",
            "model_session_anchor_validation.json": "Anchor",
            "gravity_readonly_validation.json": "Gravity calculation",
        }
        support_status: dict[str, str] = {}
        for filename, field in supporting.items():
            path = directory / filename
            _require(path.is_file(), f"FINAL_RESULT_SUPPORTING_ARTIFACT_MISSING:{filename}")
            try:
                document = json.loads(path.read_text(encoding="utf-8"))
            except (OSError, UnicodeError, json.JSONDecodeError) as exc:
                raise AcceptanceError(f"FINAL_RESULT_SUPPORTING_ARTIFACT_INVALID:{filename}") from exc
            _require(
                isinstance(document, Mapping)
                and document.get("result", document.get("status")) in {"PASS", "FULL_PASS"},
                f"FINAL_RESULT_SUPPORTING_ARTIFACT_NOT_PASS:{filename}",
            )
            support_status[field] = "PASS"
        # Active evidence has not been published yet.  Validate the exact
        # in-memory ladder that the one-shot seal will write, never a
        # pre-created CSV supplied by the control caller.
        scale_rows = [dict(row) for row in self.gravity_ladder_rows]
        _require(
            self.gravity_ladder_complete
            and scale_rows
            and all(row.get("status", "").strip().upper() == "PASS" for row in scale_rows),
            "FINAL_RESULT_IN_MEMORY_GRAVITY_SCALE_NOT_COMPLETE_PASS",
        )
        maximum_error = {
            joint: max(abs(float(row["error_deg"])) for row in self.position_rows if row["joint"] == joint)
            for joint in JOINT_NAMES
        }
        scale_sync = max(
            abs(float(row.get("j2_e_sync_deg", "0") or "0"))
            for row in scale_rows
        )
        maximum_sync = max(
            scale_sync,
            max(abs(float(row["j2_e_sync_deg"])) for row in self.position_rows),
        )
        confirmation_samples = [
            dict(sample) for sample in self.operator_confirmation_trace
        ]
        _require(
            bool(confirmation_samples),
            "FINAL_RESULT_OPERATOR_CONFIRMATION_TRACE_MISSING",
        )
        confirmation_trace_document = {
            "schema": "V15.31B-operator-confirmation-trace-v1",
            "samples": confirmation_samples,
        }
        thermal = self.thermal_summary
        restore = self.multi_joint_document["restore_initial"]
        _require(
            len(self.final_brake_pairs) == self.final_brake_pair_count
            and len(self.final_brake_pairs)
            >= MINIMUM_HALF_SECOND_SAMPLE_COUNT,
            "FINAL_RESULT_BRAKE_TRACE_INCOMPLETE",
        )
        first_brake_pair = self.final_brake_pairs[0]
        last_brake_pair = self.final_brake_pairs[-1]
        final_brake_trace_document = {
            "schema": "V15.31B-final-brake-paired-trace-v1",
            "samples": list(self.final_brake_pairs),
        }
        final_brake_hardware_source_dwell_s = (
            int(last_brake_pair["hardware_state_source_monotonic_ns"])
            - int(first_brake_pair["hardware_state_source_monotonic_ns"])
        ) / 1.0e9
        final_brake_raw_source_dwell_s = (
            int(last_brake_pair["j6_raw_source_monotonic_ns"])
            - int(first_brake_pair["j6_raw_source_monotonic_ns"])
        ) / 1.0e9
        final_brake_receipt_dwell_s = (
            int(last_brake_pair["pair_receipt_monotonic_ns"])
            - int(first_brake_pair["pair_receipt_monotonic_ns"])
        ) / 1.0e9
        _require(
            min(
                final_brake_hardware_source_dwell_s,
                final_brake_raw_source_dwell_s,
                final_brake_receipt_dwell_s,
            ) >= ENDPOINT_DWELL_NS / 1.0e9,
            "FINAL_RESULT_BRAKE_TRACE_SOURCE_DWELL_INCOMPLETE",
        )
        terminal_result = (
            "FULL_PASS" if full_pass else "HARDWARE_COUNTERBALANCE_REQUIRED"
        )
        values = {
            "V15.31B RESULT": terminal_result,
            "branch": EXPECTED_BRANCH,
            "implementation commit": implementation_commit,
            "evidence commit": evidence_commit,
            **support_status,
            "Gravity powered result": "PASS",
            "J1 error": {"status": "PASS", "max_error_deg": maximum_error["J1"]},
            "J2 error": {"status": "PASS", "max_error_deg": maximum_error["J2"]},
            "J2 max sync": {"status": "PASS", "max_sync_deg": maximum_sync},
            "J3 error": {"status": "PASS", "max_error_deg": maximum_error["J3"]},
            "J4 error": {"status": "PASS", "max_error_deg": maximum_error["J4"]},
            "J5 error": {"status": "PASS", "max_error_deg": maximum_error["J5"]},
            "J6 error": {"status": "PASS", "max_error_deg": maximum_error["J6"]},
            "multi-joint": "PASS",
            "J2A": {
                "start_temp_c": thermal["J2A"]["start_temperature_c"],
                "temp_5min_c": thermal["J2A"]["temperature_5min_c"],
                "temp_15min_c": thermal["J2A"]["temperature_15min_c"],
                "temp_30min_c": thermal["J2A"]["temperature_30min_c"],
                "final_slope_c_per_min": thermal["J2A"]["final_slope_c_per_min"],
            },
            "J2B": {
                "start_temp_c": thermal["J2B"]["start_temperature_c"],
                "temp_5min_c": thermal["J2B"]["temperature_5min_c"],
                "temp_15min_c": thermal["J2B"]["temperature_15min_c"],
                "temp_30min_c": thermal["J2B"]["temperature_30min_c"],
                "final_slope_c_per_min": thermal["J2B"]["final_slope_c_per_min"],
            },
            "J2 sustained rotor torque": thermal["j2_sustained_rotor_torque"],
            "saturation ratio": thermal["saturation_ratio"],
            "thermal classification": thermal["classification"],
            "counterbalance required": thermal["counterbalance_required"],
            "GUI powered result": "PASS",
            "restore initial": restore,
            "final brake": {
                "status": "PASS",
                "go": "BRAKE",
                "j6": "DISABLED",
                "hardware_state_sha256": self.final_brake_hardware_state_sha256,
                "j6_disabled_raw_sha256": self.j6_disabled_raw_sha256,
                "continuous_dwell_s": self.final_brake_dwell_seconds,
                "hardware_source_dwell_s": (
                    final_brake_hardware_source_dwell_s
                ),
                "j6_raw_source_dwell_s": final_brake_raw_source_dwell_s,
                "receipt_dwell_s": final_brake_receipt_dwell_s,
                "maximum_source_gap_ms": (
                    MAXIMUM_ENDPOINT_SAMPLE_GAP_NS / 1.0e6
                ),
                "session_center_rad": [
                    float(value) for value in self.center_rad
                ],
                "maximum_position_error_from_session_center_deg": max(
                    max(pair[
                        "position_error_from_session_center_deg"
                    ])
                    for pair in self.final_brake_pairs
                ),
                "paired_sample_count": self.final_brake_pair_count,
                "paired_trace": list(self.final_brake_pairs),
                "paired_trace_schema": final_brake_trace_document["schema"],
                "paired_trace_sha256": _document_sha256(
                    final_brake_trace_document
                ),
            },
            "motor zero modified": "NO",
            "MuJoCo modified": "NO",
            "git clean": "YES",
            "PRIMARY BLOCKER": (
                "NONE" if full_pass else "HARDWARE_COUNTERBALANCE_REQUIRED"
            ),
            "FINAL TASK RESULT": terminal_result,
            "READY_FOR_NEXT_STAGE": full_pass,
        }
        _require(set(values) == set(FINAL_FIELD_NAMES), "FINAL_RESULT_31_FIELDS_INTERNAL_MISMATCH")
        document = {
            "schema": "V15.31B-final-result-v1",
            "result": terminal_result,
            "report_fields": [
                {"id": index, "name": name, "value": values[name]}
                for index, name in enumerate(FINAL_FIELD_NAMES, start=1)
            ],
            "binding": {
                "envelope_id": self.binding.envelope_id,
                "envelope_sha256": self.binding.envelope_sha256,
                "session_id": self.binding.session_id,
                "state_instance_id": self.binding.state_instance_id,
                "anchor_sha256": self.binding.anchor_sha256,
                "worker_supervisor_instance_id": (
                    None if self.worker_supervisor_identity is None
                    else self.worker_supervisor_identity[0]
                ),
                "worker_supervisor_pid": (
                    None if self.worker_supervisor_identity is None
                    else self.worker_supervisor_identity[1]
                ),
                "operator_confirmation_trace": confirmation_samples,
                "operator_confirmation_trace_sha256": _document_sha256(
                    confirmation_trace_document
                ),
                "run_git_provenance": (
                    self.run_git_provenance.as_document()
                ),
                "preseal_git_provenance": (
                    self.preseal_git_provenance.as_document()
                ),
            },
        }
        self.final_result_document = document
        return document

    def seal_evidence_bundle(
        self,
        *,
        evidence_directory: Path,
    ) -> dict[str, Any]:
        """Build from memory, publish once, checksum, then request clean exit."""

        _require(not self.bundle_sealed, "EVIDENCE_BUNDLE_ALREADY_SEALED")
        directory = evidence_directory.resolve()
        _require(
            directory.is_dir() and not directory.is_symlink(),
            "EVIDENCE_DIRECTORY_INVALID",
        )
        supporting_names = set(SUPPORTING_ARTIFACT_NAMES)
        existing = {
            path.name
            for path in directory.iterdir()
            if path.is_file() and not path.is_symlink()
        }
        _require(
            existing == supporting_names,
            "EVIDENCE_PRESEAL_REQUIRES_EXACT_THREE_SUPPORTING_FILES",
        )
        _require(
            self.run_git_provenance is not None,
            "EVIDENCE_PRESEAL_STARTUP_GIT_PROVENANCE_MISSING",
        )
        _require(
            Path(self.run_git_provenance.output_directory).resolve()
            == directory,
            "EVIDENCE_PRESEAL_OUTPUT_DIFFERS_FROM_STARTUP_CAPTURE",
        )
        current_provenance = capture_run_git_provenance(
            Path(self.run_git_provenance.repo_root), directory
        )
        _require(
            current_provenance.branch == self.run_git_provenance.branch
            and current_provenance.run_base_commit
            == self.run_git_provenance.run_base_commit
            and current_provenance.upstream_ref
            == self.run_git_provenance.upstream_ref
            and current_provenance.upstream_commit
            == self.run_git_provenance.upstream_commit
            and current_provenance.ahead_count == 0
            and current_provenance.behind_count == 0
            and current_provenance.allowed_untracked_paths
            == self.run_git_provenance.allowed_untracked_paths,
            "EVIDENCE_PRESEAL_GIT_PROVENANCE_CHANGED_DURING_RUN",
        )
        self.preseal_git_provenance = current_provenance
        document = self.finalize_result(
            evidence_directory=directory,
        )
        self.write_evidence(directory)
        _require(
            {
                path.name
                for path in directory.iterdir()
                if path.is_file() and not path.is_symlink()
            }
            == set(HASHED_ARTIFACTS),
            "EVIDENCE_PRE_MANIFEST_FILE_SET_NOT_EXACT_AFTER_WRITE",
        )
        manifest = finalize_sha256_manifest(directory)
        self.bundle_manifest_sha256 = _file_sha256(manifest)
        self.bundle_sealed = True
        self.shutdown_requested = True
        return document

    def _binding_matches(self, value: object) -> bool:
        return bool(
            isinstance(value, Mapping)
            and value.get("envelope_id") == self.binding.envelope_id
            and value.get("envelope_sha256") == self.binding.envelope_sha256
            and value.get("session_id") == self.binding.session_id
            and value.get("state_instance_id") == self.binding.state_instance_id
            and value.get("anchor_sha256") == self.binding.anchor_sha256
        )

    def tick(self, *, now_ns: int) -> None:
        if self.failure is not None:
            return
        workflow_active = bool(
            self.gravity_ladder_active
            or self.comparison_condition is not None
            or self.position_started_ns is not None
            or self.post_execution is not None
            or self.thermal_stage is not None
            or self.thermal_pending_review is not None
            or self.final_brake_requested
        )
        if not workflow_active:
            return
        if self.pending_gravity_hardware_pairs:
            _gravity, pending_received_ns = (
                self.pending_gravity_hardware_pairs[0]
            )
            if (
                now_ns - pending_received_ns
                > MAXIMUM_GRAVITY_HARDWARE_PAIR_WAIT_NS
            ):
                self.fail(
                    "GRAVITY_LADDER_HARDWARE_PAIR_MISSING_TIMEOUT",
                    ("J1", "J2", "J345", "J6"),
                    now_ns,
                )
                return
        confirmation_target: Optional[float] = None
        if self.comparison_condition == "WITHOUT_FF":
            confirmation_target = None
        elif (
            self.position_started_ns is not None
            or self.post_execution is not None
            or self.thermal_stage is not None
            or self.thermal_pending_review is not None
            or self.final_brake_requested
        ):
            confirmation_target = 1.0
        if not self._fresh_confirmation(now_ns, confirmation_target):
            self.fail("OPERATOR_CONFIRMATION_STALE", ("J1", "J2", "J345", "J6"), now_ns)
            return
        if (
            self.latest_hardware_received_ns is None
            or now_ns - self.latest_hardware_received_ns > MAXIMUM_STREAM_GAP_NS
        ):
            self.fail("HARDWARE_STATE_STREAM_TIMEOUT", ("J1", "J2", "J345", "J6"), now_ns)
            return
        if (
            self.latest_gravity_received_ns is None
            or now_ns - self.latest_gravity_received_ns > MAXIMUM_STREAM_GAP_NS
        ):
            self.fail("GRAVITY_STATUS_STREAM_TIMEOUT", ("J1", "J2", "J345", "J6"), now_ns)
            return
        if (
            self.active_segment is not None
            and now_ns - self.active_segment.accepted_monotonic_ns
            > min(
                MAXIMUM_SEGMENT_NS,
                int(self.binding.maximum_segment_seconds * 1.0e9),
            )
        ):
            self.fail(
                "POSITION_SEGMENT_RUNTIME_EXCEEDED_15_SECONDS",
                (DOMAIN_BY_JOINT[self.active_segment.joint],),
                now_ns,
            )
            return
        if (
            self.post_execution is not None
            and self.post_execution.active_segment is not None
            and now_ns - self.post_execution.active_segment.accepted_monotonic_ns
            > min(
                MAXIMUM_SEGMENT_NS,
                int(self.binding.maximum_segment_seconds * 1.0e9),
            )
        ):
            self.fail(
                "POST_POSITION_SEGMENT_RUNTIME_EXCEEDED_15_SECONDS",
                (DOMAIN_BY_JOINT[self.post_execution.active_segment.joint],),
                now_ns,
            )
            return
        if (
            self.final_brake_requested
            and not self.final_brake_observed
            and self.final_brake_requested_ns is not None
            and now_ns - self.final_brake_requested_ns > 10_000_000_000
        ):
            self.fail(
                "FINAL_BRAKE_OR_J6_DISABLED_NOT_OBSERVED_WITHIN_10_SECONDS",
                ("J1", "J2", "J345", "J6"),
                now_ns,
            )

    def fail(
        self, reason: str, related_domains: Sequence[str], now_ns: int,
    ) -> None:
        if self.failure is not None:
            return
        bounded_reason = str(reason).strip()[:256] or "UNSPECIFIED_FAIL_CLOSED"
        domains = tuple(dict.fromkeys(str(item) for item in related_domains))
        self.failure = FailureRecord(bounded_reason, domains, now_ns)
        self.active_segment = None
        self.post_execution = None
        self.awaiting_position_command = False
        self.comparison_condition = None
        self.thermal_stage = None
        if self.brake_callback is not None:
            self.brake_callback(self.failure)

    def status(self, *, now_ns: Optional[int] = None) -> dict[str, Any]:
        checked = time.monotonic_ns() if now_ns is None else now_ns
        expected_joint = self.expected_joint
        expected_phase = self.expected_phase
        expected_target_rad: Optional[float] = None
        expected_target_vector_rad: Optional[list[float]] = None
        if (
            expected_joint is not None
            and expected_phase is not None
            and self.center_rad is not None
        ):
            joint_index = JOINT_NAMES.index(expected_joint)
            expected_target_vector_rad = list(self.center_rad)
            expected_target_vector_rad[joint_index] += math.radians(
                PHASE_OFFSET_DEG[self.position_phase_index]
            )
            expected_target_rad = expected_target_vector_rad[joint_index]
        return {
            "schema": STATUS_SCHEMA,
            "source_monotonic_ns": checked,
            "result": (
                "ABORTED" if self.failure is not None
                else "COMPLETE" if self.bundle_sealed
                else "FINALIZING" if self.final_brake_requested
                else "RUNNING"
            ),
            "binding": {
                "envelope_id": self.binding.envelope_id,
                "envelope_sha256": self.binding.envelope_sha256,
                "session_id": self.binding.session_id,
                "state_instance_id": self.binding.state_instance_id,
                "anchor_sha256": self.binding.anchor_sha256,
            },
            "operator_confirmation_fresh": self._fresh_confirmation(checked),
            "gravity_ladder": {
                "active": self.gravity_ladder_active,
                "complete": self.gravity_ladder_complete,
                "expected_levels": list(GRAVITY_LADDER_LEVELS),
                "completed_levels": list(self.gravity_ladder_completed_levels),
                "rows": len(self.gravity_ladder_rows),
                "maximum_sample_gap_ms": (
                    MAXIMUM_GRAVITY_LADDER_SAMPLE_GAP_NS / 1.0e6
                ),
            },
            "position": {
                "started": self.position_started_ns is not None,
                "complete": self.position_complete,
                "joint_order": list(POSITION_ORDER),
                "expected_joint": expected_joint,
                "expected_phase": expected_phase,
                "expected_target_rad": expected_target_rad,
                "expected_target_deg": (
                    None
                    if expected_target_rad is None
                    else _rad_to_deg(expected_target_rad)
                ),
                "expected_target_vector_rad": expected_target_vector_rad,
                "awaiting_gui_command": self.awaiting_position_command,
                "rows": len(self.position_rows),
                POSITION_BUDGET_FIELD: self.binding.maximum_position_seconds,
                "consumed_accepted_segment_seconds": (
                    self.position_trajectory_budget_ns / 1.0e9
                ),
                "maximum_segment_seconds": self.binding.maximum_segment_seconds,
            },
            "comparison": {
                "active_condition": self.comparison_condition,
                "complete_conditions": sorted(self.comparison_complete),
                "rows": len(self.comparison_rows),
            },
            "post_position": {
                "workflow_phase": self.post_workflow_phase,
                "active_kind": (
                    None if self.post_execution is None
                    else self.post_execution.kind
                ),
                "awaiting_gui_command": (
                    False if self.post_execution is None
                    else self.post_execution.awaiting_gui_command
                ),
                "completed_kinds": sorted(self.completed_post_execution),
                "completed_order": list(self.completed_post_order),
                "machine_observed_multi_joint": (
                    "MULTI_JOINT" in self.completed_post_execution
                ),
                "machine_observed_thermal_setup": (
                    "THERMAL_SETUP" in self.completed_post_execution
                ),
                "machine_observed_restore_initial": (
                    "RESTORE_INITIAL" in self.completed_post_execution
                ),
            },
            "thermal": {
                "active_stage_min": self.thermal_stage,
                "pending_review_min": self.thermal_pending_review,
                "approved_stages_min": sorted(self.thermal_approved),
            },
            "failure": None if self.failure is None else {
                "reason": self.failure.reason,
                "related_domains": list(self.failure.related_domains),
                "monotonic_ns": self.failure.monotonic_ns,
                "brake_scope": self.failure.brake_scope,
            },
            "final_brake_requested": self.final_brake_requested,
            "final_brake_observed": self.final_brake_observed,
            "final_brake_paired_sample_count": self.final_brake_pair_count,
            "final_brake_continuous_dwell_s": self.final_brake_dwell_seconds,
            "j6_disabled_raw_observed": self.j6_disabled_raw_observed_ns is not None,
            "bundle_sealed": self.bundle_sealed,
            "bundle_manifest_sha256": self.bundle_manifest_sha256,
            "shutdown_requested": self.shutdown_requested,
            "success_path_brake_or_drag_observed": any(
                event["mode"] in {"brake", "drag"}
                and not event["final_brake_requested"]
                for event in self.success_path_mode_events
            ),
            "success_path_hardware_brake_disabled_or_j6_inactive": any(
                (
                    not event["final_brake_requested"]
                    and (
                        event["j6_control_available"] is not True
                        or any(
                            mode not in {"hold", "position"}
                            for mode in event["modes"].values()
                        )
                    )
                )
                for event in self.success_path_hardware_events
            ),
        }

    def write_evidence(self, output_directory: Path) -> list[Path]:
        """Atomically publish only completed artifacts; never overwrite."""

        output = output_directory.resolve()
        output.mkdir(parents=True, exist_ok=True, mode=0o700)
        _require(output.is_dir() and not output.is_symlink(), "OUTPUT_DIRECTORY_INVALID")
        written: list[Path] = []
        if (self.gravity_ladder_complete and self.gravity_ladder_rows) or (
            self.failure is not None and self.gravity_ladder_rows
        ):
            path = output / "gravity_scale_validation.csv"
            _atomic_csv(
                path,
                GRAVITY_SCALE_FIELDS,
                self._rows_for_result(self.gravity_ladder_rows),
            )
            written.append(path)
        if self.position_complete or self.failure is not None:
            rows = self._rows_for_result(self.position_rows)
            if not rows:
                rows = [self._empty_position_failure_row()]
            path = output / "position_validation.csv"
            _atomic_csv(path, POSITION_FIELDS, rows)
            written.append(path)
        if self.comparison_complete == {"WITHOUT_FF", "WITH_FF"} or self.failure is not None:
            rows = self._rows_for_result(self.comparison_rows)
            if not rows:
                rows = [self._empty_comparison_failure_row()]
            path = output / "j2_control_comparison.csv"
            _atomic_csv(path, J2_COMPARISON_FIELDS, rows)
            written.append(path)
        for stage in (5, 15, 30):
            if (
                stage in self.thermal_approved
                or stage in self.thermal_not_run_stages
                or (self.failure is not None and self.thermal_rows[stage])
            ):
                if stage in self.thermal_approved:
                    self._validate_thermal_rows(stage, self.thermal_rows[stage])
                    rows = self._rows_for_result(self.thermal_rows[stage])
                elif stage in self.thermal_not_run_stages:
                    rows = [self._empty_thermal_not_run_row(stage)]
                else:
                    rows = self._rows_for_result(self.thermal_rows[stage])
                path = output / f"thermal_{stage}min.csv"
                _atomic_csv(path, THERMAL_FIELDS, rows)
                written.append(path)
        if self.thermal_summary is not None:
            path = output / "thermal_summary.json"
            _atomic_json(path, self.thermal_summary)
            written.append(path)
        if self.multi_joint_document is not None:
            path = output / "multi_joint_validation.json"
            _atomic_json(path, self.multi_joint_document)
            written.append(path)
        if self.gui_document is not None:
            path = output / "gui_powered_validation.json"
            _atomic_json(path, self.gui_document)
            written.append(path)
        if self.final_result_document is not None:
            path = output / "final_result.json"
            _atomic_json(path, self.final_result_document)
            written.append(path)
        return written

    def _rows_for_result(self, rows: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
        copies = [dict(row) for row in rows]
        if self.failure is not None:
            for row in copies:
                row["status"] = "ABORTED"
                row["reason"] = self.failure.reason
        return copies

    def _empty_position_failure_row(self) -> dict[str, Any]:
        assert self.failure is not None
        row = {field: "" for field in POSITION_FIELDS}
        row.update({
            "timestamp_utc": _utc_text(), "monotonic_ns": self.failure.monotonic_ns,
            "receipt_monotonic_ns": self.failure.monotonic_ns,
            "test_id": "FAIL_CLOSED", "joint": self.expected_joint or "J1",
            "revision": "BASELINE", "phase": self.expected_phase or "CENTER_START",
            "target_deg": 0.0, "actual_deg": 0.0, "error_deg": 0.0,
            "endpoint_dwell_s": 0.0, "j2_e_sync_deg": 0.0,
            "temperature_c": 0.0, "merror": 0, "communication_ok": False,
            "controller_mode": "brake", "status": "ABORTED",
            "reason": self.failure.reason, "envelope_id": self.binding.envelope_id,
            "envelope_sha256": self.binding.envelope_sha256,
            "session_id": self.binding.session_id,
            "state_instance_id": self.binding.state_instance_id,
            "anchor_sha256": self.binding.anchor_sha256,
            "related_domain": "/".join(self.failure.related_domains),
        })
        return row

    def _empty_comparison_failure_row(self) -> dict[str, Any]:
        assert self.failure is not None
        row = {field: "" for field in J2_COMPARISON_FIELDS}
        row.update({
            "timestamp_utc": _utc_text(), "monotonic_ns": self.failure.monotonic_ns,
            "receipt_monotonic_ns": self.failure.monotonic_ns,
            "condition": "WITHOUT_FF", "window_elapsed_s": 0.0,
            "position_error_deg": 0.0, "j2_e_sync_deg": 0.0,
            "j2a_tau_feedback_rotor_nm": 0.0, "j2b_tau_feedback_rotor_nm": 0.0,
            "j2a_pd_rotor_nm": 0.0, "j2b_pd_rotor_nm": 0.0,
            "j2a_gravity_ff_rotor_nm": 0.0, "j2b_gravity_ff_rotor_nm": 0.0,
            "j2a_total_command_rotor_nm": 0.0, "j2b_total_command_rotor_nm": 0.0,
            "j2a_temperature_c": 0.0, "j2b_temperature_c": 0.0,
            "saturation_observed": False, "internal_opposition_detected": False,
            "status": "ABORTED", "reason": self.failure.reason,
            "envelope_id": self.binding.envelope_id,
            "envelope_sha256": self.binding.envelope_sha256,
            "session_id": self.binding.session_id,
            "state_instance_id": self.binding.state_instance_id,
            "anchor_sha256": self.binding.anchor_sha256,
        })
        return row

    def _empty_thermal_not_run_row(self, stage: int) -> dict[str, Any]:
        reason = self.thermal_not_run_stages[stage]
        row = {field: "" for field in THERMAL_FIELDS}
        row.update({
            "timestamp_utc": _utc_text(),
            "monotonic_ns": (
                self.latest_hardware_source_ns
                if self.latest_hardware_source_ns is not None else 0
            ),
            "receipt_monotonic_ns": (
                self.latest_hardware_received_ns
                if self.latest_hardware_received_ns is not None else 0
            ),
            "elapsed_s": 0.0,
            "status": reason,
            "reason": reason,
            "envelope_id": self.binding.envelope_id,
            "envelope_sha256": self.binding.envelope_sha256,
            "session_id": self.binding.session_id,
            "state_instance_id": self.binding.state_instance_id,
            "anchor_sha256": self.binding.anchor_sha256,
            "hardware_state_sha256": (
                "" if self.latest_hardware is None
                else _document_sha256(self.latest_hardware)
            ),
            "hardware_state_sequence": (
                "" if self.latest_hardware is None
                else self.latest_hardware["sequence"]
            ),
            "hardware_state_source_monotonic_ns": (
                "" if self.latest_hardware is None
                else self.latest_hardware["source_monotonic_ns"]
            ),
        })
        return row


def _atomic_csv(
    path: Path, fields: Sequence[str], rows: Sequence[Mapping[str, Any]],
) -> None:
    _require(rows, f"{path.name.upper()}_NO_ROWS")
    _require(not path.exists(), f"OUTPUT_EXISTS:{path.name}")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent), text=True,
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=list(fields), extrasaction="ignore")
            writer.writeheader()
            for row in rows:
                writer.writerow(row)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, stat.S_IRUSR | stat.S_IWUSR)
        os.replace(temporary, path)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def _atomic_json(path: Path, document: Mapping[str, Any]) -> None:
    _require(not path.exists(), f"OUTPUT_EXISTS:{path.name}")
    payload = json.dumps(
        document, ensure_ascii=False, allow_nan=False, sort_keys=True, indent=2,
    ).encode("utf-8") + b"\n"
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent),
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, stat.S_IRUSR | stat.S_IWUSR)
        os.replace(temporary, path)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def finalize_sha256_manifest(evidence_directory: Path) -> Path:
    """Seal exactly thirteen artifacts into the fourteenth bundle file."""

    directory = evidence_directory.resolve()
    _require(directory.is_dir() and not directory.is_symlink(), "EVIDENCE_DIRECTORY_INVALID")
    manifest = directory / "SHA256SUMS"
    _require(not manifest.exists(), "OUTPUT_EXISTS:SHA256SUMS")
    regular_files = {
        path.name for path in directory.iterdir()
        if path.is_file() and not path.is_symlink()
    }
    _require(
        regular_files == set(HASHED_ARTIFACTS),
        "EVIDENCE_PRE_MANIFEST_FILE_SET_NOT_EXACT",
    )
    payload = "".join(
        f"{_file_sha256(directory / filename)}  {filename}\n"
        for filename in HASHED_ARTIFACTS
    ).encode("ascii")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=".SHA256SUMS.", suffix=".tmp", dir=str(directory)
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, stat.S_IRUSR | stat.S_IWUSR)
        os.replace(temporary, manifest)
    except Exception:
        temporary.unlink(missing_ok=True)
        raise
    _require(
        {
            path.name for path in directory.iterdir()
            if path.is_file() and not path.is_symlink()
        } == set(EXACT_BUNDLE_FILES),
        "EVIDENCE_FINAL_FILE_SET_NOT_EXACT",
    )
    return manifest


def dry_run_plan() -> dict[str, Any]:
    return {
        "schema": "go-m8010-v15-31b-active-acceptance-plan/1.0",
        "mode": "OFFLINE_DRY_RUN",
        "opens_can": False,
        "opens_serial": False,
        "opens_worker_udp": False,
        "publishes_position": False,
        "only_live_command": "ROUTER_MEDIATED_ALL_DOMAIN_BRAKE_ON_FAILURE",
        "observed_topics": [
            "/whole_arm/gui_command",
            "/whole_arm/control_status",
            "/whole_arm/hardware_state",
            "/whole_arm/motor_feedback_raw",
            "/whole_arm/gravity_status",
            "/whole_arm/empirical_stage_confirmation",
        ],
        "position_order": list(POSITION_ORDER),
        "position_phases": list(POSITION_PHASES),
        "maximum_moving_joints": 1,
        "maximum_segment_displacement_deg": 5.0,
        "maximum_segment_seconds": 15.0,
        POSITION_BUDGET_FIELD: 600.0,
        "endpoint_error_deg": 0.5,
        "endpoint_dwell_seconds": 0.5,
        "operator_confirmation_maximum_age_seconds": 30.0,
        "recommended_confirmation_period_seconds": 10.0,
        "git_provenance": {
            "capture": "STARTUP_AND_PRESEAL_READ_ONLY_GIT",
            "branch": EXPECTED_BRANCH,
            "requires_head_equal_upstream": True,
            "requires_ahead_behind": [0, 0],
            "only_allowed_untracked": list(SUPPORTING_ARTIFACT_NAMES),
            "evidence_commit_semantics": (
                "EVIDENCE_RUN_BASE_COMMIT_NOT_SELF_REFERENTIAL_SEAL_COMMIT"
            ),
        },
        "gravity_ladder": {
            "levels": list(GRAVITY_LADDER_LEVELS),
            "minimum_ramp_seconds": 2.0,
            "hold_seconds": [5.0, 10.0],
            "maximum_sample_gap_ms": 100.0,
            "producer": "PASSIVE_GRAVITY_STATUS_PLUS_HARDWARE_STATE_COLLECTOR",
        },
        "thermal_stages_min": [5, 15, 30],
        "thermal_note": (
            "The same session/envelope continues in bounded thermal HOLD; "
            "every newly Router-accepted POSITION segment duration consumes "
            "the 600 s motion budget; immutable active-segment refreshes do not."
        ),
    }


def _handle_control(
    runner: ActiveAcceptanceRunner, value: object, now_ns: int,
    evidence_directory: Optional[Path] = None,
) -> None:
    control = runner.validate_control(value, now_ns=now_ns)
    action = str(control.get("action", "")).strip().upper()
    if action == "START_GRAVITY_LADDER":
        runner.start_gravity_ladder(now_ns=now_ns)
    elif action == "START_POSITION":
        runner.start_position(now_ns=now_ns)
    elif action in {
        "START_MULTI_JOINT", "START_THERMAL_SETUP", "START_RESTORE_INITIAL"
    }:
        kind = action.removeprefix("START_")
        runner.start_post_position_execution(
            kind,
            _finite_vector(
                control.get("target_rad"), 6, "CONTROL_POST_TARGET_INVALID"
            ),
            now_ns=now_ns,
        )
    elif action == "START_J2_COMPARISON":
        runner.start_comparison(
            str(control.get("condition", "")),
            _finite(control.get("target_j2_rad"), "CONTROL_TARGET_J2_INVALID"),
            now_ns=now_ns,
        )
    elif action == "STOP_J2_COMPARISON":
        runner.stop_comparison(now_ns=now_ns)
    elif action == "START_THERMAL_STAGE":
        duration = control.get("duration_min")
        _require(type(duration) is int, "CONTROL_THERMAL_DURATION_INVALID")
        runner.start_thermal_stage(
            duration,
            _finite(control.get("target_j2_rad"), "CONTROL_TARGET_J2_INVALID"),
            now_ns=now_ns,
        )
    elif action == "APPROVE_THERMAL_STAGE":
        duration = control.get("duration_min")
        _require(type(duration) is int, "CONTROL_THERMAL_DURATION_INVALID")
        runner.approve_thermal_stage(duration, now_ns=now_ns)
    elif action == "FINALIZE_THERMAL":
        recommendation = control.get("mechanical_recommendation")
        _require(
            recommendation is None or isinstance(recommendation, Mapping),
            "CONTROL_MECHANICAL_RECOMMENDATION_INVALID",
        )
        runner.finalize_thermal_summary(
            classification=str(control.get("classification", "")),
            mechanical_recommendation=recommendation,
            now_ns=now_ns,
        )
    elif action == "RECORD_MULTI_JOINT":
        runner.record_multi_joint_attestation(control.get("document"))
    elif action == "RECORD_RESTORE_INITIAL":
        runner.record_restore_attestation(control.get("document"))
    elif action == "RECORD_GUI_POWERED":
        runner.record_gui_attestation(control.get("document"))
    elif action == "FINAL_BRAKE":
        runner.request_final_brake(now_ns=now_ns)
    elif action == "FINALIZE_RESULT":
        _require(evidence_directory is not None, "CONTROL_EVIDENCE_DIRECTORY_UNAVAILABLE")
        runner.seal_evidence_bundle(
            evidence_directory=evidence_directory,
        )
    elif action == "STOP_AND_BRAKE":
        runner.fail("OPERATOR_STOP_AND_BRAKE", ("J1", "J2", "J345", "J6"), now_ns)
    else:
        raise AcceptanceError("CONTROL_ACTION_INVALID")


def run_live(args: argparse.Namespace, binding: EvidenceBinding) -> int:
    """Create the ROS observer only after all offline/live gates pass."""

    _require(args.enable_live == LIVE_ENABLE_TOKEN, "LIVE_ENABLE_TOKEN_MISSING")
    validate_physical_startup_gate(os.environ)
    binding.ensure_not_expired()
    _require(args.repo_root is not None, "--repo-root is required for live mode")
    _require(args.output_dir is not None, "--output-dir is required for live mode")
    run_git_provenance = capture_run_git_provenance(
        Path(args.repo_root), Path(args.output_dir)
    )
    try:
        import rclpy
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:
        raise AcceptanceError("ROS2_PYTHON_RUNTIME_UNAVAILABLE") from exc

    class AcceptanceNode(Node):
        def __init__(self) -> None:
            super().__init__("v15_31b_active_acceptance_runner")
            self.brake_publisher = self.create_publisher(
                String, "/whole_arm/gui_command", 10
            )
            self.status_publisher = self.create_publisher(
                String, "/whole_arm/v15_31b/acceptance_status", 10
            )
            self.runner = ActiveAcceptanceRunner(
                binding,
                brake_callback=self._publish_brake,
                run_git_provenance=run_git_provenance,
            )
            self.create_subscription(
                String, "/whole_arm/gui_command", self._on_gui_command, 50
            )
            self.create_subscription(
                String, "/whole_arm/control_status", self._on_router_status, 20
            )
            self.create_subscription(
                String, "/whole_arm/hardware_state", self._on_hardware_state, 100
            )
            self.create_subscription(
                String, "/whole_arm/motor_feedback_raw",
                self._on_motor_feedback_raw, 100,
            )
            self.create_subscription(
                String, "/whole_arm/gravity_status", self._on_gravity_status, 100
            )
            self.create_subscription(
                String, "/whole_arm/empirical_stage_confirmation",
                self._on_confirmation, 20,
            )
            self.create_subscription(
                String, "/whole_arm/v15_31b/acceptance_control",
                self._on_control, 20,
            )
            self.timer = self.create_timer(0.1, self._tick)
            self._brake_message: Optional[String] = None
            self._brake_publish_count = 0
            self._shutdown_initiated = False

        @staticmethod
        def _decode(message: Any) -> object:
            try:
                return json.loads(message.data)
            except (json.JSONDecodeError, TypeError):
                return None

        def _on_gui_command(self, message: Any) -> None:
            self.runner.observe_gui_command(
                self._decode(message), now_ns=time.monotonic_ns()
            )

        def _on_router_status(self, message: Any) -> None:
            self.runner.observe_router_status(
                self._decode(message), now_ns=time.monotonic_ns()
            )

        def _on_hardware_state(self, message: Any) -> None:
            self.runner.observe_hardware_state(
                self._decode(message), now_ns=time.monotonic_ns()
            )

        def _on_motor_feedback_raw(self, message: Any) -> None:
            self.runner.observe_motor_feedback_raw(
                self._decode(message), now_ns=time.monotonic_ns()
            )

        def _on_gravity_status(self, message: Any) -> None:
            self.runner.observe_gravity_status(
                self._decode(message), now_ns=time.monotonic_ns()
            )

        def _on_confirmation(self, message: Any) -> None:
            self.runner.observe_confirmation(
                self._decode(message), now_ns=time.monotonic_ns()
            )

        def _on_control(self, message: Any) -> None:
            now_ns = time.monotonic_ns()
            try:
                _handle_control(
                    self.runner,
                    self._decode(message),
                    now_ns,
                    evidence_directory=Path(args.output_dir),
                )
            except AcceptanceError as exc:
                self.runner.fail(
                    str(exc), ("J1", "J2", "J345", "J6"), now_ns
                )

        def _publish_brake(self, failure: FailureRecord) -> None:
            payload = {
                "schema": "go-m8010-gui-command/1.2",
                "sequence": 0,
                "source_instance_id": "emergency-brake",
                "source_monotonic_ns": failure.monotonic_ns,
                "mode": "brake",
                "targets_rad": [0.0] * 6,
                "active_joint_mask": [False] * 6,
                "moving_joint_mask": [False] * 6,
                "activation_epoch": 0,
                "maximum_velocity_rad_s": math.radians(5.0),
                "maximum_acceleration_rad_s2": math.radians(20.0),
                "kp": [0.0] * 6,
                "kd": [0.0] * 6,
                "v15_31b_failure": {
                    "reason": failure.reason,
                    "related_domains": list(failure.related_domains),
                    "brake_scope": failure.brake_scope,
                },
            }
            self._brake_message = String(data=json.dumps(payload, ensure_ascii=False))
            self.brake_publisher.publish(self._brake_message)
            self._brake_publish_count = 1

        def _tick(self) -> None:
            now_ns = time.monotonic_ns()
            self.runner.tick(now_ns=now_ns)
            if self._brake_message is not None and self._brake_publish_count < 10:
                self.brake_publisher.publish(self._brake_message)
                self._brake_publish_count += 1
            self.status_publisher.publish(
                String(data=json.dumps(self.runner.status(now_ns=now_ns), ensure_ascii=False))
            )
            if (
                self.runner.shutdown_requested
                and not self._shutdown_initiated
            ):
                self._shutdown_initiated = True
                rclpy.shutdown()

        def destroy_node(self) -> bool:
            if self.runner.bundle_sealed:
                return super().destroy_node()
            try:
                self.runner.write_evidence(Path(args.output_dir))
            except AcceptanceError as exc:
                self.get_logger().error(f"evidence write failed: {exc}")
            return super().destroy_node()

    rclpy.init(args=None)
    node: Optional[AcceptanceNode] = None
    completed = False
    try:
        node = AcceptanceNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        if (
            node is not None
            and node.runner.failure is None
            and not node.runner.bundle_sealed
        ):
            node.runner.fail(
                "RUNNER_INTERRUPTED",
                ("J1", "J2", "J345", "J6"),
                time.monotonic_ns(),
            )
    finally:
        if node is not None:
            completed = node.runner.bundle_sealed
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if completed else 2


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--mode", choices=("dry-run", "validate-bindings", "live"),
        default="dry-run",
    )
    parser.add_argument("--envelope", type=Path)
    parser.add_argument("--anchor-validation", type=Path)
    parser.add_argument("--expected-envelope-sha256", default="")
    parser.add_argument("--output-dir", type=Path)
    parser.add_argument("--repo-root", type=Path)
    parser.add_argument("--enable-live", default="")
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = build_parser().parse_args(argv)
    if args.mode == "dry-run":
        print(json.dumps(dry_run_plan(), ensure_ascii=False, indent=2))
        return 0
    _require(args.envelope is not None, "--envelope is required")
    _require(args.anchor_validation is not None, "--anchor-validation is required")
    binding = EvidenceBinding.from_paths(
        args.envelope,
        args.anchor_validation,
        args.expected_envelope_sha256,
    )
    if args.mode == "validate-bindings":
        print(json.dumps({
            "result": "PASS",
            "mode": "OFFLINE_VALIDATE_BINDINGS",
            "envelope_id": binding.envelope_id,
            "envelope_sha256": binding.envelope_sha256,
            "session_id": binding.session_id,
            "state_instance_id": binding.state_instance_id,
            "anchor_sha256": binding.anchor_sha256,
        }, ensure_ascii=False, indent=2))
        return 0
    _require(args.output_dir is not None, "--output-dir is required for live mode")
    _require(args.repo_root is not None, "--repo-root is required for live mode")
    return run_live(args, binding)


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except AcceptanceError as error:
        print(f"V15.31B ACTIVE ACCEPTANCE BLOCKED: {error}", file=sys.stderr)
        raise SystemExit(2)

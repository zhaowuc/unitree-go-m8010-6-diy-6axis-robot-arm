#!/usr/bin/env python3
"""Fail-closed final acceptance aggregation for GO-M8010 V15.14.

This tool does not run ROS or MuJoCo and does not modify model sources.  It only
reads the seven authoritative evidence JSON files supplied on the command line
and writes one machine-readable JSON report plus one Markdown summary.

Every required field is checked explicitly.  A missing/unreadable input, a
missing field, a type mismatch, or a failed gate always makes the aggregate
result FAIL and returns a non-zero exit status.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import sys
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, Iterable


SCHEMA = "go-m8010-arm-v15.14-final-acceptance/1.0"
MAIN_QA_SCHEMA = "go-m8010-arm-v15.14-trajectory-closed-loop-qa/1.0"
MAIN_QA_REVISION = "V15.14-MoveIt2-ros2_control-MuJoCo"
TARGET_MANIFEST_SHA256 = (
    "f2b230e2cb61c251c81228f5f3b89ced0163b05cf82f0b93db8fd0bb85572d57"
)
V15_14_MJCF_SHA256 = (
    "ce6ec828d32f445ff25fb4b3ba3167f7f0bec61cdfca5962ee4072df4b022844"
)
V15_14_COLLISION_CONTRACT_SHA256 = (
    "6d802909e44f238816f007ef33b57e1b57099c5522a473cd2dcc2705397af9c9"
)
V15_14_GUARD_SHA256 = (
    "648b9de953a84be46cd5f95e24daf9f683f3025e73048dc068ef7bd8e5670c6a"
)
V15_13_VALIDATOR_SHA256 = (
    "9be9fab3e5c7850f001c2e46b83aebf033a02feff2a9e27dbc15347db4098ea4"
)

REQUIRED_TARGETS = {
    "central": "execute_success",
    "high": "execute_success",
    "left": "execute_success",
    "front": "execute_success",
    "low": "execute_success",
    "right": "execute_success",
    "back": "execute_success",
    "random_valid_seed_1514": "execute_success",
    "near_joint_limit_clear": "execute_success",
    "near_self_collision_clear": "execute_success",
    "outside_joint_limit_reject": "bounds_rejection",
    "self_collision_reject": "collision_rejection",
}

REQUIRED_V15_13_TESTS = {
    "build_model",
    "validate_model",
    "validate_joint_motion_semantics",
    "validate_virtual_camera",
}

MISSING = object()


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def field(document: Any, dotted_path: str) -> Any:
    current = document
    for part in dotted_path.split("."):
        if not isinstance(current, dict) or part not in current:
            return MISSING
        current = current[part]
    return current


def json_safe(value: Any) -> Any:
    if value is MISSING:
        return None
    if isinstance(value, float) and not math.isfinite(value):
        return str(value)
    try:
        json.dumps(value, ensure_ascii=False, allow_nan=False)
        return value
    except (TypeError, ValueError):
        return repr(value)


def compact(value: Any, limit: int = 320) -> str:
    if value is MISSING:
        return "<missing>"
    rendered = json.dumps(
        json_safe(value), ensure_ascii=False, sort_keys=True, separators=(",", ":")
    )
    return rendered if len(rendered) <= limit else rendered[: limit - 3] + "..."


def strict_equal(actual: Any, expected: Any) -> bool:
    if isinstance(expected, bool):
        return actual is expected
    if isinstance(expected, int) and not isinstance(expected, bool):
        return isinstance(actual, int) and not isinstance(actual, bool) and actual == expected
    if isinstance(expected, float):
        return (
            isinstance(actual, (int, float))
            and not isinstance(actual, bool)
            and math.isfinite(float(actual))
            and float(actual) == expected
        )
    return type(actual) is type(expected) and actual == expected


@dataclass
class Gate:
    gate_id: str
    passed: bool
    expected: Any
    actual: Any
    detail: str

    def as_dict(self) -> dict[str, Any]:
        return {
            "id": self.gate_id,
            "pass": self.passed,
            "expected": json_safe(self.expected),
            "actual": json_safe(self.actual),
            "detail": self.detail,
        }


class Gates:
    def __init__(self) -> None:
        self.rows: list[Gate] = []

    def add(
        self,
        gate_id: str,
        passed: bool,
        expected: Any,
        actual: Any,
        detail: str = "",
    ) -> None:
        self.rows.append(
            Gate(gate_id, passed is True, expected, actual, detail)
        )

    def equal(self, gate_id: str, document: Any, dotted_path: str, expected: Any) -> None:
        actual = field(document, dotted_path)
        detail = "required field is missing" if actual is MISSING else ""
        self.add(gate_id, strict_equal(actual, expected), expected, actual, detail)

    def predicate(
        self,
        gate_id: str,
        actual: Any,
        expected: Any,
        predicate: Callable[[Any], bool],
        detail: str = "",
    ) -> None:
        try:
            passed = actual is not MISSING and predicate(actual) is True
        except (KeyError, TypeError, ValueError, OverflowError):
            passed = False
        if actual is MISSING and not detail:
            detail = "required field is missing"
        self.add(gate_id, passed, expected, actual, detail)

    @property
    def passed(self) -> bool:
        return bool(self.rows) and all(row.passed is True for row in self.rows)


def read_input(label: str, path: Path, gates: Gates) -> tuple[dict[str, Any], dict[str, Any]]:
    metadata: dict[str, Any] = {"path": str(path.resolve())}
    try:
        raw = path.read_bytes()
        metadata.update({"size_bytes": len(raw), "sha256": hashlib.sha256(raw).hexdigest()})
        parsed = json.loads(raw.decode("utf-8"))
        if not isinstance(parsed, dict):
            raise TypeError("JSON root must be an object")
    except Exception as exc:  # All load failures are evidence failures, not crashes.
        metadata["error"] = f"{type(exc).__name__}: {exc}"
        gates.add(
            f"input.{label}.loaded",
            False,
            "readable UTF-8 JSON object",
            metadata.get("error"),
            "input could not be loaded",
        )
        return {}, metadata
    gates.add(
        f"input.{label}.loaded",
        True,
        "readable UTF-8 JSON object",
        "loaded",
    )
    return parsed, metadata


def normalized_hash(value: Any) -> str | None:
    return value.lower() if isinstance(value, str) else None


def validate_main_qa(doc: dict[str, Any], gates: Gates) -> None:
    prefix = "main_qa"
    gates.equal(f"{prefix}.schema", doc, "schema", MAIN_QA_SCHEMA)
    gates.equal(f"{prefix}.revision", doc, "revision", MAIN_QA_REVISION)
    gates.equal(f"{prefix}.status", doc, "status", "PASS")
    gates.equal(f"{prefix}.scope.execution_mode", doc, "scope.execution_mode", "kinematic_position_tracking")
    gates.equal(f"{prefix}.scope.dynamics_invalid", doc, "scope.dynamics_valid", False)
    gates.equal(f"{prefix}.scope.simulation_only", doc, "scope.simulation_only", True)
    gates.equal(f"{prefix}.scope.tcp_frame", doc, "scope.tcp_frame", "tcp_nominal")
    gates.predicate(
        f"{prefix}.target_manifest_hash",
        field(doc, "target_manifest.sha256"),
        TARGET_MANIFEST_SHA256,
        lambda value: normalized_hash(value) == TARGET_MANIFEST_SHA256,
    )
    gates.predicate(
        f"{prefix}.mujoco_model_hash",
        field(doc, "mujoco_model.sha256"),
        V15_14_MJCF_SHA256,
        lambda value: normalized_hash(value) == V15_14_MJCF_SHA256,
    )
    for name in (
        "pass",
        "follow_joint_trajectory_provider_pass",
        "controller_types_and_states_pass",
        "single_joint_state_source",
        "standard_topic_based_ros2_control_path",
        "no_private_moveit_mujoco_edge",
    ):
        gates.equal(f"{prefix}.ros_graph.{name}", doc, f"ros_graph.{name}", True)
    gates.equal(f"{prefix}.ground_scene", doc, "ground_scene.pass", True)

    for phase in ("startup", "post_run"):
        base = f"bridge_runtime_contract.{phase}"
        gates.equal(f"{prefix}.{phase}.contract", doc, f"{base}.pass", True)
        gates.equal(f"{prefix}.{phase}.controller_parameters", doc, f"{base}.controller_runtime_parameters.pass", True)
        gates.equal(f"{prefix}.{phase}.state_streams_fresh", doc, f"{base}.state_streams_fresh", True)
        gates.equal(f"{prefix}.{phase}.fault_clear", doc, f"{base}.status.fault_latched", False)
        gates.equal(f"{prefix}.{phase}.watchdog_clear", doc, f"{base}.status.moving_watchdog_timeout_count", 0)
        gates.equal(f"{prefix}.{phase}.j1_normalization_enabled", doc, f"{base}.j1_continuous_branch_normalization.enabled", True)
        gates.equal(f"{prefix}.{phase}.j1_normalization_joint", doc, f"{base}.j1_continuous_branch_normalization.joint_name", "J1")
        gates.equal(
            f"{prefix}.{phase}.j1_normalization_method",
            doc,
            f"{base}.j1_continuous_branch_normalization.method",
            "nearest_equivalent_to_current",
        )

    gates.equal(f"{prefix}.aggregate.positive_count", doc, "aggregate.positive_target_count", 10)
    gates.equal(f"{prefix}.aggregate.rejection_count", doc, "aggregate.expected_rejection_count", 2)
    for name in (
        "all_expectations_met",
        "ros_graph_pass",
        "ground_scene_pass",
        "startup_bridge_runtime_contract_pass",
        "post_run_bridge_runtime_contract_pass",
        "pass",
    ):
        gates.equal(f"{prefix}.aggregate.{name}", doc, f"aggregate.{name}", True)

    tests = field(doc, "tests")
    summary: dict[str, Any] = {}
    valid_structure = isinstance(tests, list) and all(isinstance(row, dict) for row in tests)
    if valid_structure:
        for row in tests:
            target_id = row.get("id")
            if isinstance(target_id, str):
                summary[target_id] = {
                    "expected_outcome": row.get("expected_outcome"),
                    "verdict": row.get("verdict"),
                }
    gates.add(f"{prefix}.tests.count", valid_structure and len(tests) == 12, 12, len(tests) if isinstance(tests, list) else tests)
    gates.add(
        f"{prefix}.tests.unique_required_ids",
        valid_structure
        and len(summary) == 12
        and set(summary) == set(REQUIRED_TARGETS),
        sorted(REQUIRED_TARGETS),
        sorted(summary),
    )
    semantics_pass = valid_structure and len(summary) == 12
    if semantics_pass:
        for target_id, expected_outcome in REQUIRED_TARGETS.items():
            expected_verdict = "PASS" if expected_outcome == "execute_success" else "EXPECTED_REJECTION_PASS"
            row = summary.get(target_id, {})
            if row.get("expected_outcome") != expected_outcome or row.get("verdict") != expected_verdict:
                semantics_pass = False
                break
    gates.add(
        f"{prefix}.tests.expectations_and_verdicts",
        semantics_pass,
        "10 execute_success=>PASS and 2 accepted rejections=>EXPECTED_REJECTION_PASS",
        summary,
    )


def validate_collision_503(doc: dict[str, Any], gates: Gates) -> None:
    prefix = "collision_503"
    gates.equal(f"{prefix}.status", doc, "status", "PASS")
    gates.equal(f"{prefix}.revision", doc, "revision", "V15.14-MoveIt-MuJoCo-503-pose-collision-cross-regression")
    gates.equal(f"{prefix}.model_not_modified", doc, "method.models_are_not_modified", True)
    gates.equal(f"{prefix}.expected_pose_count", doc, "expected_total_pose_count", 503)
    gates.equal(f"{prefix}.pose_count", doc, "total_pose_count", 503)
    gates.equal(f"{prefix}.pose_count_gate", doc, "pose_count_gate_pass", True)
    frozen_pose_set_sha256 = (
        "4d9cfaf15cc20fe16393d1aa3bccddc5273e77e2de08cf9d0045976c301799aa"
    )
    gates.equal(f"{prefix}.frozen_pose_set_gate", doc, "frozen_pose_set_gate.pass", True)
    for name in ("expected_sha256", "actual_sha256"):
        gates.predicate(
            f"{prefix}.frozen_pose_set.{name}",
            field(doc, f"frozen_pose_set_gate.{name}"),
            frozen_pose_set_sha256,
            lambda value: normalized_hash(value) == frozen_pose_set_sha256,
        )
    gates.equal(f"{prefix}.input_hash_gate", doc, "input_hashes_pass", True)
    gates.equal(f"{prefix}.match_count", doc, "match_count", 503)
    gates.equal(f"{prefix}.mismatch_count", doc, "mismatch_count", 0)
    gates.equal(f"{prefix}.boolean_mismatch_count", doc, "boolean_mismatch_count", 0)
    gates.equal(f"{prefix}.overall_boolean_mismatch_count", doc, "overall_boolean_mismatch_count", 0)
    gates.equal(f"{prefix}.self_boolean_mismatch_count", doc, "self_boolean_mismatch_count", 0)
    gates.equal(f"{prefix}.ground_boolean_mismatch_count", doc, "ground_boolean_mismatch_count", 0)
    gates.equal(f"{prefix}.pair_set_mismatch_count", doc, "pair_set_mismatch_count", 0)
    gates.equal(f"{prefix}.self_pair_set_mismatch_count", doc, "self_pair_set_mismatch_count", 0)
    gates.equal(f"{prefix}.booleans_match", doc, "all_collision_booleans_match", True)
    gates.equal(f"{prefix}.overall_booleans_match", doc, "all_overall_collision_booleans_match", True)
    gates.equal(f"{prefix}.self_booleans_match", doc, "all_self_collision_booleans_match", True)
    gates.equal(f"{prefix}.ground_booleans_match", doc, "all_ground_collision_booleans_match", True)
    gates.equal(f"{prefix}.pair_sets_match", doc, "all_collision_pair_sets_match", True)
    gates.equal(f"{prefix}.self_pair_sets_match", doc, "all_self_collision_pair_sets_match", True)
    # Deliberately do not gate all_ground_collision_pair_sets_match_diagnostic:
    # engine-specific ground-facet ownership changes the contact manifold while
    # the independently gated ground collision boolean remains identical.
    gates.equal(f"{prefix}.frozen_collision_counts", doc, "frozen_collision_counts_pass", True)
    gates.equal(
        f"{prefix}.expected_frozen_collision_counts",
        doc,
        "expected_frozen_collision_pose_counts",
        {"overall": 182, "self": 126, "ground": 118},
    )
    for engine in ("moveit", "mujoco"):
        for collision_class, expected in (
            ("overall", 182),
            ("self", 126),
            ("ground", 118),
        ):
            field_name = (
                f"{engine}_collision_pose_count"
                if collision_class == "overall"
                else f"{engine}_{collision_class}_collision_pose_count"
            )
            gates.equal(
                f"{prefix}.count.{engine}.{collision_class}",
                doc,
                field_name,
                expected,
            )
            gates.equal(
                f"{prefix}.count_gate.{collision_class}.{engine}",
                doc,
                f"frozen_collision_count_gates.{collision_class}.{engine}_actual_pose_count",
                expected,
            )
    for collision_class in ("overall", "self", "ground"):
        gates.equal(
            f"{prefix}.count_gate.{collision_class}.pass",
            doc,
            f"frozen_collision_count_gates.{collision_class}.pass",
            True,
        )
    gates.equal(f"{prefix}.negative_control_count", doc, "negative_control_count", 5)
    gates.equal(f"{prefix}.negative_controls", doc, "negative_controls_pass", True)
    gates.equal(f"{prefix}.accepted_model_unmodified", doc, "mujoco_parent_child_filter.accepted_model_modified", False)
    gates.equal(f"{prefix}.parent_filter_enabled_by_guard", doc, "mujoco_parent_child_filter.enabled_by_guard", True)
    gates.predicate(
        f"{prefix}.model_hash",
        field(doc, "inputs.model_sha256"),
        V15_14_MJCF_SHA256,
        lambda value: normalized_hash(value) == V15_14_MJCF_SHA256,
    )
    gates.predicate(
        f"{prefix}.guard_hash",
        field(doc, "inputs.guard_sha256"),
        V15_14_GUARD_SHA256,
        lambda value: normalized_hash(value) == V15_14_GUARD_SHA256,
    )
    gates.predicate(
        f"{prefix}.validator_hash",
        field(doc, "inputs.validator_sha256"),
        V15_13_VALIDATOR_SHA256,
        lambda value: normalized_hash(value) == V15_13_VALIDATOR_SHA256,
    )
    controls = field(doc, "reference_terminal_collision_negative_controls")
    expected_control_indices = {43, 73, 75, 83, 87}
    controls_pass = (
        isinstance(controls, list)
        and len(controls) == 5
        and {
            row.get("pose_index")
            for row in controls
            if isinstance(row, dict)
        }
        == expected_control_indices
        and all(
            isinstance(row, dict)
            and row.get("category") == "coupled_random_seed_1513"
            and row.get("moveit_collision") is True
            and row.get("mujoco_collision") is True
            and row.get("moveit_self_collision") is True
            and row.get("mujoco_self_collision") is True
            and row.get("pass") is True
            for row in controls
        )
    )
    gates.add(
        f"{prefix}.negative_controls_exact_all_pass",
        controls_pass,
        "five accepted self-collision controls at indices 43,73,75,83,87",
        controls,
    )


def validate_collision_static(doc: dict[str, Any], gates: Gates) -> None:
    prefix = "collision_static"
    gates.equal(f"{prefix}.pass", doc, "static_validation.pass", True)
    for name, expected in (
        ("proxy_count", 25),
        ("all_unordered_pair_count", 300),
        ("runtime_full_pair_count", 231),
        ("runtime_excluded_pair_count", 69),
        ("motion_enabled_pair_count", 2),
        ("motion_excluded_pair_count", 22),
    ):
        gates.equal(f"{prefix}.pair_partition.{name}", doc, f"static_validation.pair_partition.{name}", expected)
    gates.equal(f"{prefix}.runtime_token_coverage", doc, "static_validation.runtime_token_coverage.pass", True)
    gates.equal(f"{prefix}.inherited_meshes", doc, "static_validation.inherited_proxy_mesh_validation.pass", True)
    gates.equal(f"{prefix}.explicit_geom_pairs", doc, "static_validation.motion_proxy.mjcf_explicit_geom_pair_count", 82)
    for name, expected in (
        ("contract", V15_14_COLLISION_CONTRACT_SHA256),
        ("mjcf", V15_14_MJCF_SHA256),
        ("guard", V15_14_GUARD_SHA256),
    ):
        gates.predicate(
            f"{prefix}.generated_hash.{name}",
            field(doc, f"static_validation.generated_hashes.{name}"),
            expected,
            lambda value, expected=expected: normalized_hash(value) == expected,
        )


def validate_collision_runtime(doc: dict[str, Any], gates: Gates) -> None:
    prefix = "collision_runtime"
    gates.equal(f"{prefix}.top_pass", doc, "pass", True)
    gates.equal(f"{prefix}.pass", doc, "runtime_validation.pass", True)
    gates.equal(f"{prefix}.nq", doc, "runtime_validation.model_nq", 6)
    gates.equal(f"{prefix}.npair", doc, "runtime_validation.model_npair", 82)
    gates.equal(f"{prefix}.expected_pairs", doc, "runtime_validation.expected_explicit_geom_pairs", 82)
    gates.equal(f"{prefix}.frozen_pose_gate", doc, "runtime_validation.frozen_pose_set_gate.pass", True)
    gates.equal(f"{prefix}.frozen_pose_count", doc, "runtime_validation.frozen_pose_set_gate.pose_count", 503)
    cases = field(doc, "runtime_validation.cases")
    cases_pass = (
        isinstance(cases, list)
        and len(cases) >= 3
        and all(isinstance(row, dict) and row.get("pass") is True for row in cases)
    )
    gates.add(f"{prefix}.cases", cases_pass, "at least 3 runtime cases, every pass=true", cases)


def validate_v15_13(doc: dict[str, Any], gates: Gates) -> None:
    prefix = "v15_13_regression"
    gates.equal(f"{prefix}.schema", doc, "schema", "go_m8010_v15_13_regression_summary_v1")
    gates.equal(f"{prefix}.overall_status", doc, "overall_status", "PASS")
    gates.equal(f"{prefix}.source_not_written", doc, "execution.authoritative_source_written", False)
    gates.equal(f"{prefix}.source_hash_unchanged", doc, "execution.source_hash_unchanged", True)
    gates.equal(f"{prefix}.xml_deterministic", doc, "deterministic_rebuild.xml_matches_authoritative", True)
    gates.equal(f"{prefix}.contract_deterministic", doc, "deterministic_rebuild.model_contract_matches_authoritative", True)
    tests = field(doc, "tests")
    test_summary: dict[str, Any] = {}
    valid = isinstance(tests, list) and all(isinstance(row, dict) for row in tests)
    if valid:
        test_summary = {
            row.get("name"): {"status": row.get("status"), "exit_code": row.get("exit_code")}
            for row in tests
            if isinstance(row.get("name"), str)
        }
    gates.add(
        f"{prefix}.required_test_set",
        valid and set(test_summary) == REQUIRED_V15_13_TESTS,
        sorted(REQUIRED_V15_13_TESTS),
        sorted(test_summary),
    )
    gates.add(
        f"{prefix}.all_tests_pass",
        valid
        and set(test_summary) == REQUIRED_V15_13_TESTS
        and all(row == {"status": "PASS", "exit_code": 0} for row in test_summary.values()),
        "all four tests status=PASS and exit_code=0",
        test_summary,
    )


def validate_frozen_baseline(doc: dict[str, Any], gates: Gates) -> None:
    prefix = "frozen_baseline"
    gates.equal(f"{prefix}.schema", doc, "schema", "go-m8010-arm-v15.14-frozen-geometry-baseline/2.0")
    gates.equal(f"{prefix}.accepted_contract", doc, "accepted_contract_pass", True)
    gates.equal(f"{prefix}.all_frozen_joints", doc, "all_frozen_joints_identical", True)
    gates.equal(f"{prefix}.all_frozen_elements", doc, "derived_mjcf.all_frozen_elements_identical", True)
    gates.equal(f"{prefix}.runtime_token_coverage", doc, "runtime_token_coverage.pass", True)
    gates.equal(f"{prefix}.inherited_meshes", doc, "inherited_proxy_mesh_validation.pass", True)
    for name, expected in (
        ("collision_contract_sha256", V15_14_COLLISION_CONTRACT_SHA256),
        ("mjcf_sha256", V15_14_MJCF_SHA256),
        ("guard_sha256", V15_14_GUARD_SHA256),
    ):
        gates.predicate(
            f"{prefix}.generated_hash.{name}",
            field(doc, f"generated_hashes.{name}"),
            expected,
            lambda value, expected=expected: normalized_hash(value) == expected,
        )
    gates.equal(f"{prefix}.position_limits_inherited", doc, "simulation_only_limits.position_limits", "inherited unchanged from V15.13")


def validate_j1_branch(doc: dict[str, Any], gates: Gates) -> None:
    """Validate the exact report emitted by validate_j1_continuous_branch.py."""

    prefix = "j1_branch"
    two_pi = 2.0 * math.pi
    joint_names = ["J1", "J2", "J3", "J4", "J5", "J6"]
    expected_initial_gates = {
        "raw_state_present",
        "mechanical_zero_position",
        "stationary_velocity",
        "bridge_schema",
        "bridge_primed",
        "bridge_fault_not_latched",
        "bridge_no_rejections",
        "bridge_no_watchdog_timeout",
        "j1_normalization_enabled",
        "j1_normalization_joint",
        "j1_normalization_method",
        "fresh_adjustment_counter",
        "single_command_publisher",
        "single_raw_state_publisher",
        "single_controller_state_publisher",
        "single_bridge_status_publisher",
        "publisher_message_types",
    }
    expected_segment_gates = {
        "standard_follow_joint_trajectory_goal_accepted",
        "action_status_succeeded",
        "controller_result_successful",
        "minimum_command_samples",
        "minimum_raw_state_samples",
        "minimum_controller_state_samples",
        "paired_branch_samples_present",
        "command_minus_raw_branch_offset",
        "raw_j1_has_no_branch_jump",
        "raw_velocity_within_envelope",
        "other_joints_remain_zero",
        "controller_wrapped_tracking_error_within_limit",
        "final_physical_position_matches",
        "bridge_adjusted_accepted_commands",
        "bridge_max_adjustment_is_two_pi",
        "bridge_accepted_moving_commands",
        "bridge_rejected_no_commands",
        "bridge_watchdog_no_timeout",
        "bridge_fault_not_latched",
        "bridge_guard_safe",
        "bridge_observed_velocity_within_envelope",
        "bridge_observed_acceleration_within_envelope",
    }
    expected_final_gates = {
        "raw_state_present",
        "returned_to_mechanical_zero",
        "stationary_velocity",
        "bridge_fault_not_latched",
        "bridge_no_rejections",
        "bridge_no_watchdog_timeout",
        "bridge_adjustments_observed",
        "bridge_max_adjustment_is_two_pi",
    }

    def finite_number(value: Any) -> bool:
        return (
            isinstance(value, (int, float))
            and not isinstance(value, bool)
            and math.isfinite(float(value))
        )

    def finite_le(value: Any, limit: float) -> bool:
        return finite_number(value) and 0.0 <= float(value) <= limit

    def vector_close(value: Any, expected: list[float], tolerance: float) -> bool:
        return (
            isinstance(value, list)
            and len(value) == len(expected)
            and all(
                finite_number(actual)
                and abs(float(actual) - reference) <= tolerance
                for actual, reference in zip(value, expected)
            )
        )

    def gate_map_is_exact_true(value: Any, expected_keys: set[str]) -> bool:
        return (
            isinstance(value, dict)
            and set(value) == expected_keys
            and all(item is True for item in value.values())
        )

    gates.equal(
        f"{prefix}.schema",
        doc,
        "schema",
        "go-m8010-arm-v15.14-j1-continuous-branch-regression/1.0",
    )
    gates.equal(f"{prefix}.pass", doc, "pass", True)
    gates.equal(f"{prefix}.exception", doc, "exception", None)
    gates.equal(
        f"{prefix}.contract.scope",
        doc,
        "contract.scope",
        "J1 continuous representation through standard FollowJointTrajectory",
    )
    gates.equal(
        f"{prefix}.contract.action",
        doc,
        "contract.action_name",
        "/arm_controller/follow_joint_trajectory",
    )
    gates.equal(f"{prefix}.contract.joints", doc, "contract.joint_names", joint_names)
    gates.predicate(
        f"{prefix}.contract.segment_duration",
        field(doc, "contract.segment_duration_s"),
        "finite and >= 3.0 s",
        lambda value: finite_number(value) and float(value) >= 3.0,
    )
    for path, expected in (
        ("contract.positive_branch_goal_rad", [two_pi, two_pi + 0.12]),
        ("contract.negative_branch_goal_rad", [-two_pi + 0.12, -two_pi]),
        ("contract.expected_physical_motion_rad", [0.0, 0.12, 0.0]),
    ):
        gates.predicate(
            f"{prefix}.{path}",
            field(doc, path),
            expected,
            lambda value, expected=expected: vector_close(value, expected, 1.0e-12),
        )
    for name, expected in (
        ("branch_offset_tolerance_rad", 0.02),
        ("branch_max_adjustment_tolerance_rad", 1.0e-9),
        ("raw_j1_jump_limit_rad", 0.05),
        ("raw_velocity_limit_rad_s", 0.52),
        ("bridge_acceleration_limit_rad_s2", 1.02),
        ("other_joint_limit_rad", 1.0e-5),
        ("final_position_tolerance_rad", 1.0e-3),
        ("controller_wrapped_tracking_limit_rad", 0.03),
        ("minimum_dynamic_sample_count", 20),
    ):
        gates.equal(
            f"{prefix}.contract.threshold.{name}",
            doc,
            f"contract.thresholds.{name}",
            expected,
        )

    gates.equal(f"{prefix}.initial.pass", doc, "initial.pass", True)
    initial_gate_map = field(doc, "initial.gates")
    gates.add(
        f"{prefix}.initial.all_exact_gates",
        gate_map_is_exact_true(initial_gate_map, expected_initial_gates),
        sorted(expected_initial_gates),
        initial_gate_map,
    )
    gates.predicate(
        f"{prefix}.initial.zero_position",
        field(doc, "initial.raw_state.position_rad"),
        "six finite joints within 1e-3 rad of zero",
        lambda value: vector_close(value, [0.0] * 6, 1.0e-3),
    )
    gates.predicate(
        f"{prefix}.initial.stationary_velocity",
        field(doc, "initial.raw_state.velocity_rad_s"),
        "six finite joints within 1e-5 rad/s of zero",
        lambda value: vector_close(value, [0.0] * 6, 1.0e-5),
    )
    for name, expected in (
        ("schema", "go-m8010-arm-v15.14-mujoco-bridge-status/1.0"),
        ("fault_latched", False),
        ("command_stream_primed", True),
        ("rejected_command_count", 0),
        ("moving_watchdog_timeout_count", 0),
    ):
        gates.equal(
            f"{prefix}.initial.bridge.{name}",
            doc,
            f"initial.bridge_status.{name}",
            expected,
        )
    for name, expected in (
        ("enabled", True),
        ("joint_name", "J1"),
        ("method", "nearest_equivalent_to_current"),
        ("accepted_adjustment_count", 0),
    ):
        gates.equal(
            f"{prefix}.initial.normalization.{name}",
            doc,
            f"initial.bridge_status.j1_continuous_branch_normalization.{name}",
            expected,
        )

    segments = field(doc, "segments")
    gates.add(
        f"{prefix}.segments.count",
        isinstance(segments, list) and len(segments) == 2,
        2,
        len(segments) if isinstance(segments, list) else segments,
    )
    expected_segments = (
        ("positive_2pi_branch", two_pi, two_pi + 0.12, two_pi, 0.12),
        ("negative_2pi_branch", -two_pi + 0.12, -two_pi, -two_pi, 0.0),
    )
    for index, (
        segment_id,
        start_j1,
        finish_j1,
        expected_offset,
        expected_final,
    ) in enumerate(expected_segments):
        base = f"segments.{index}"
        segment = segments[index] if isinstance(segments, list) and len(segments) > index and isinstance(segments[index], dict) else {}
        gates.equal(f"{prefix}.{segment_id}.id", segment, "id", segment_id)
        gates.equal(
            f"{prefix}.{segment_id}.action",
            segment,
            "goal.action_name",
            "/arm_controller/follow_joint_trajectory",
        )
        gates.equal(f"{prefix}.{segment_id}.joints", segment, "goal.joint_names", joint_names)
        points = field(segment, "goal.points")
        point_contract_pass = (
            isinstance(points, list)
            and len(points) == 2
            and isinstance(points[0], dict)
            and isinstance(points[1], dict)
            and strict_equal(points[0].get("time_from_start_s", MISSING), 0.0)
            and finite_number(points[1].get("time_from_start_s"))
            and float(points[1]["time_from_start_s"]) >= 3.0
            and vector_close(points[0].get("position_rad"), [start_j1, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0e-12)
            and vector_close(points[1].get("position_rad"), [finish_j1, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0e-12)
            and vector_close(points[0].get("velocity_rad_s"), [0.0] * 6, 0.0)
            and vector_close(points[1].get("velocity_rad_s"), [0.0] * 6, 0.0)
        )
        gates.add(
            f"{prefix}.{segment_id}.two_point_goal",
            point_contract_pass,
            {"start_j1_rad": start_j1, "finish_j1_rad": finish_j1},
            points,
        )
        gates.equal(f"{prefix}.{segment_id}.pass", segment, "pass", True)
        segment_gate_map = field(segment, "gates")
        gates.add(
            f"{prefix}.{segment_id}.all_exact_gates",
            gate_map_is_exact_true(segment_gate_map, expected_segment_gates),
            sorted(expected_segment_gates),
            segment_gate_map,
        )
        gates.equal(f"{prefix}.{segment_id}.goal_accepted", segment, "result.accepted", True)
        gates.equal(f"{prefix}.{segment_id}.action_status", segment, "result.status", 4)
        gates.equal(f"{prefix}.{segment_id}.controller_result", segment, "result.error_code", 0)
        for stream_name in ("command", "raw_state", "controller_state"):
            gates.predicate(
                f"{prefix}.{segment_id}.samples.{stream_name}",
                field(segment, f"metrics.sample_count.{stream_name}"),
                ">= 20",
                lambda value: isinstance(value, int)
                and not isinstance(value, bool)
                and value >= 20,
            )
        pairing = field(segment, "metrics.branch_pairing")
        gates.equal(
            f"{prefix}.{segment_id}.pairing.expected_offset",
            segment,
            "metrics.branch_pairing.expected_command_minus_raw_rad",
            expected_offset,
        )
        gates.predicate(
            f"{prefix}.{segment_id}.pairing.sample_count",
            field(segment, "metrics.branch_pairing.paired_sample_count"),
            ">= 20",
            lambda value: isinstance(value, int)
            and not isinstance(value, bool)
            and value >= 20,
        )
        gates.predicate(
            f"{prefix}.{segment_id}.pairing.median",
            field(segment, "metrics.branch_pairing.median_command_minus_raw_rad"),
            f"within 0.02 rad of {expected_offset}",
            lambda value, expected_offset=expected_offset: finite_number(value)
            and abs(float(value) - expected_offset) <= 0.02,
        )
        gates.predicate(
            f"{prefix}.{segment_id}.pairing.inliers",
            field(segment, "metrics.branch_pairing.inlier_fraction"),
            ">= 0.95",
            lambda value: finite_number(value) and 0.95 <= float(value) <= 1.0,
        )
        for name, threshold in (
            ("max_consecutive_raw_j1_jump_rad", 0.05),
            ("max_abs_raw_velocity_rad_s", 0.52),
            ("max_abs_other_joint_position_rad", 1.0e-5),
            ("max_abs_other_joint_velocity_rad_s", 1.0e-5),
            ("max_controller_wrapped_tracking_error_rad", 0.03),
            ("max_final_physical_position_error_rad", 1.0e-3),
            ("bridge_max_observed_command_velocity_rad_s", 0.52),
            ("bridge_max_observed_command_acceleration_rad_s2", 1.02),
        ):
            gates.predicate(
                f"{prefix}.{segment_id}.metric.{name}",
                field(segment, f"metrics.{name}"),
                f"finite and <= {threshold}",
                lambda value, threshold=threshold: finite_le(value, threshold),
            )
        gates.predicate(
            f"{prefix}.{segment_id}.metric.expected_final",
            field(segment, "metrics.expected_final_physical_position_rad"),
            [expected_final, 0.0, 0.0, 0.0, 0.0, 0.0],
            lambda value, expected_final=expected_final: vector_close(
                value, [expected_final, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0e-12
            ),
        )
        gates.predicate(
            f"{prefix}.{segment_id}.metric.actual_final",
            field(segment, "metrics.actual_final_physical_position_rad"),
            f"within 1e-3 rad of physical J1={expected_final}, others zero",
            lambda value, expected_final=expected_final: vector_close(
                value, [expected_final, 0.0, 0.0, 0.0, 0.0, 0.0], 1.0e-3
            ),
        )
        for name, predicate, expected in (
            ("j1_accepted_adjustment_count", lambda value: isinstance(value, int) and not isinstance(value, bool) and value > 0, "> 0"),
            ("accepted_moving_command_count", lambda value: isinstance(value, int) and not isinstance(value, bool) and value > 0, "> 0"),
            ("rejected_command_count", lambda value: strict_equal(value, 0), 0),
            ("moving_watchdog_timeout_count", lambda value: strict_equal(value, 0), 0),
        ):
            gates.predicate(
                f"{prefix}.{segment_id}.counter.{name}",
                field(segment, f"metrics.bridge_counter_delta.{name}"),
                expected,
                predicate,
            )
        gates.predicate(
            f"{prefix}.{segment_id}.bridge.max_adjustment",
            field(segment, "metrics.bridge_max_abs_j1_branch_adjustment_rad"),
            f"2*pi +/- 1e-9 ({two_pi})",
            lambda value: finite_number(value)
            and abs(float(value) - two_pi) <= 1.0e-9,
        )
        gates.equal(
            f"{prefix}.{segment_id}.bridge.fault_clear",
            segment,
            "bridge_after.fault_latched",
            False,
        )
        gates.equal(
            f"{prefix}.{segment_id}.bridge.rejections",
            segment,
            "bridge_after.rejected_command_count",
            0,
        )
        gates.equal(
            f"{prefix}.{segment_id}.bridge.watchdog",
            segment,
            "bridge_after.moving_watchdog_timeout_count",
            0,
        )
        gates.equal(
            f"{prefix}.{segment_id}.bridge.normalization.enabled",
            segment,
            "bridge_after.j1_continuous_branch_normalization.enabled",
            True,
        )
        gates.equal(
            f"{prefix}.{segment_id}.bridge.normalization.joint",
            segment,
            "bridge_after.j1_continuous_branch_normalization.joint_name",
            "J1",
        )
        gates.equal(
            f"{prefix}.{segment_id}.bridge.normalization.method",
            segment,
            "bridge_after.j1_continuous_branch_normalization.method",
            "nearest_equivalent_to_current",
        )

    gates.equal(f"{prefix}.final.pass", doc, "final.pass", True)
    final_gate_map = field(doc, "final.gates")
    gates.add(
        f"{prefix}.final.all_exact_gates",
        gate_map_is_exact_true(final_gate_map, expected_final_gates),
        sorted(expected_final_gates),
        final_gate_map,
    )
    gates.predicate(
        f"{prefix}.final.zero_position",
        field(doc, "final.raw_state.position_rad"),
        "six finite joints within 1e-3 rad of zero",
        lambda value: vector_close(value, [0.0] * 6, 1.0e-3),
    )
    gates.predicate(
        f"{prefix}.final.stationary_velocity",
        field(doc, "final.raw_state.velocity_rad_s"),
        "six finite joints within 1e-5 rad/s of zero",
        lambda value: vector_close(value, [0.0] * 6, 1.0e-5),
    )
    for name, expected in (
        ("schema", "go-m8010-arm-v15.14-mujoco-bridge-status/1.0"),
        ("fault_latched", False),
        ("rejected_command_count", 0),
        ("moving_watchdog_timeout_count", 0),
    ):
        gates.equal(
            f"{prefix}.final.bridge.{name}",
            doc,
            f"final.bridge_status.{name}",
            expected,
        )
    gates.predicate(
        f"{prefix}.final.normalization.adjustments",
        field(
            doc,
            "final.bridge_status.j1_continuous_branch_normalization.accepted_adjustment_count",
        ),
        "> 0",
        lambda value: isinstance(value, int)
        and not isinstance(value, bool)
        and value > 0,
    )
    gates.predicate(
        f"{prefix}.final.normalization.max_adjustment",
        field(
            doc,
            "final.bridge_status.j1_continuous_branch_normalization.max_abs_adjustment_rad",
        ),
        f"2*pi +/- 1e-9 ({two_pi})",
        lambda value: finite_number(value)
        and abs(float(value) - two_pi) <= 1.0e-9,
    )


def render_markdown(report: dict[str, Any]) -> str:
    status = report["status"]
    lines = [
        "# V15.14 最终验收聚合报告",
        "",
        f"- 结论：**{status}**",
        f"- 生成时间（UTC）：`{report['generated_at_utc']}`",
        f"- Gate：`{report['gate_summary']['passed']}/{report['gate_summary']['total']}` PASS",
        "",
        "## Gate 明细",
        "",
        "| Gate | 结果 | 期望 | 实际 | 说明 |",
        "|---|---:|---|---|---|",
    ]
    for row in report["gates"]:
        result = "PASS" if row["pass"] else "FAIL"
        expected = compact(row["expected"]).replace("|", "\\|")
        actual = compact(row["actual"]).replace("|", "\\|")
        detail = str(row.get("detail", "")).replace("|", "\\|").replace("\n", " ")
        lines.append(f"| `{row['id']}` | **{result}** | `{expected}` | `{actual}` | {detail} |")
    lines.extend(["", "## 输入证据", "", "| 名称 | 路径 | SHA-256 |", "|---|---|---|"])
    for label, metadata in report["inputs"].items():
        path = str(metadata.get("path", "")).replace("|", "\\|")
        digest = metadata.get("sha256", "<unavailable>")
        lines.append(f"| `{label}` | `{path}` | `{digest}` |")
    lines.append("")
    return "\n".join(lines)


def aggregate(args: argparse.Namespace) -> tuple[dict[str, Any], int]:
    gates = Gates()
    specs: list[tuple[str, Path, Callable[[dict[str, Any], Gates], None]]] = [
        ("main_qa", args.main_qa, validate_main_qa),
        ("collision_503", args.collision_503, validate_collision_503),
        ("collision_static", args.collision_static, validate_collision_static),
        ("collision_runtime", args.collision_runtime, validate_collision_runtime),
        ("v15_13_summary", args.v15_13_summary, validate_v15_13),
        ("frozen_baseline", args.frozen_baseline, validate_frozen_baseline),
        ("j1_branch", args.j1_branch, validate_j1_branch),
    ]
    documents: dict[str, dict[str, Any]] = {}
    inputs: dict[str, dict[str, Any]] = {}
    for label, path, _validator in specs:
        documents[label], inputs[label] = read_input(label, path, gates)
    for label, _path, validator in specs:
        validator(documents[label], gates)

    passed_count = sum(row.passed for row in gates.rows)
    report = {
        "schema": SCHEMA,
        "revision": "V15.14-MoveIt2-ROS2-Control-MuJoCo-final-acceptance",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": "PASS" if gates.passed else "FAIL",
        "pass": gates.passed,
        "fail_closed": True,
        "inputs": inputs,
        "gate_summary": {
            "total": len(gates.rows),
            "passed": passed_count,
            "failed": len(gates.rows) - passed_count,
        },
        "gates": [row.as_dict() for row in gates.rows],
    }
    return report, 0 if gates.passed else 2


def self_test() -> int:
    gates = Gates()
    gates.equal("self.true", {"value": True}, "value", True)
    gates.equal("self.strict_bool", {"value": 1}, "value", True)
    gates.equal("self.missing", {}, "value", True)
    assert gates.rows[0].passed is True
    assert gates.rows[1].passed is False
    assert gates.rows[2].passed is False
    assert gates.passed is False
    assert normalized_hash("ABC") == "abc"
    assert strict_equal(503, 503)
    assert not strict_equal(True, 1)
    assert field({"a": {"b": 2}}, "a.b") == 2
    assert field({"a": {}}, "a.b") is MISSING
    sample = {
        "status": "FAIL",
        "generated_at_utc": "self-test",
        "gate_summary": {"passed": 1, "total": 3},
        "gates": [row.as_dict() for row in gates.rows],
        "inputs": {},
    }
    rendered = render_markdown(sample)
    assert "self.strict_bool" in rendered and "FAIL" in rendered
    print("aggregate_v15_14_acceptance.py self-test: PASS")
    return 0


def parse_args(argv: Iterable[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--main-qa", type=Path, required=True, help="12-target V15.14 closed-loop QA JSON")
    parser.add_argument("--collision-503", type=Path, required=True, help="fresh 503-pose MoveIt/MuJoCo cross-regression JSON")
    parser.add_argument("--collision-static", type=Path, required=True, help="static collision-layer validation JSON")
    parser.add_argument("--collision-runtime", type=Path, required=True, help="MuJoCo runtime collision-layer validation JSON")
    parser.add_argument("--v15-13-summary", type=Path, required=True, help="V15.13 regression summary JSON")
    parser.add_argument("--frozen-baseline", type=Path, required=True, help="frozen geometry baseline JSON")
    parser.add_argument("--j1-branch", type=Path, required=True, help="dedicated J1 +/-2pi branch regression JSON")
    parser.add_argument("--output-json", type=Path, required=True, help="output aggregate JSON")
    parser.add_argument("--output-markdown", type=Path, required=True, help="output aggregate Markdown")
    parser.add_argument("--self-test", action="store_true", help=argparse.SUPPRESS)
    return parser.parse_args(argv)


def main(argv: Iterable[str] | None = None) -> int:
    if argv is None:
        argv = sys.argv[1:]
    argv = list(argv)
    if argv == ["--self-test"]:
        return self_test()
    args = parse_args(argv)
    report, exit_code = aggregate(args)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_markdown.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(
        json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    args.output_markdown.write_text(render_markdown(report), encoding="utf-8")
    print(
        json.dumps(
            {
                "status": report["status"],
                "passed_gates": report["gate_summary"]["passed"],
                "total_gates": report["gate_summary"]["total"],
                "output_json": str(args.output_json.resolve()),
                "output_markdown": str(args.output_markdown.resolve()),
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())

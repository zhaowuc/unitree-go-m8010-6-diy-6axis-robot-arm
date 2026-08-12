#!/usr/bin/env python3
from __future__ import annotations

"""Cross-check the frozen V15.13 503 poses in MoveIt and MuJoCo.

This tool is deliberately read-only with respect to the robot model.  The pose
set is imported from the accepted V15.13 validator so its range endpoints,
random seed and category ordering cannot silently drift in V15.14.
"""

import argparse
import hashlib
import importlib.util
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

import mujoco
import rclpy
from moveit_msgs.srv import GetStateValidity
from rclpy.node import Node
from sensor_msgs.msg import JointState


JOINT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]
EXPECTED_TOTAL_POSE_COUNT = 503
EXPECTED_POSE_SET_SHA256 = (
    "4d9cfaf15cc20fe16393d1aa3bccddc5273e77e2de08cf9d0045976c301799aa"
)
EXPECTED_COLLISION_POSE_COUNTS = {
    "overall": 182,
    "self": 126,
    "ground": 118,
}
EXPECTED_INPUT_SHA256 = {
    "validator": "9be9fab3e5c7850f001c2e46b83aebf033a02feff2a9e27dbc15347db4098ea4",
    "guard": "648b9de953a84be46cd5f95e24daf9f683f3025e73048dc068ef7bd8e5670c6a",
    "model": "ce6ec828d32f445ff25fb4b3ba3167f7f0bec61cdfca5962ee4072df4b022844",
}
REFERENCE_TERMINAL_COLLISION_INDICES = {
    "coupled_random_seed_1513": [43, 73, 75, 83, 87],
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def load_module(path: Path, module_name: str):
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot import {path}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def contact_to_dict(contact: Any) -> dict[str, Any]:
    return {
        "body_1": str(contact.contact_body_1),
        "body_2": str(contact.contact_body_2),
        "depth_m": float(contact.depth),
    }


def compact_mujoco_contacts(contacts: list[dict[str, Any]]) -> list[dict[str, Any]]:
    """Keep one worst-depth record per proxy pair instead of thousands of facets."""
    compact: dict[tuple[str, str, str], dict[str, Any]] = {}
    for contact in contacts:
        key = (
            str(contact["proxy_pair"][0]),
            str(contact["proxy_pair"][1]),
            str(contact["contact_class"]),
        )
        candidate = {
            "proxy_pair": list(contact["proxy_pair"]),
            "contact_class": str(contact["contact_class"]),
            "minimum_distance_m": float(contact["distance_m"]),
        }
        prior = compact.get(key)
        if prior is None or candidate["minimum_distance_m"] < prior["minimum_distance_m"]:
            compact[key] = candidate
    return [compact[key] for key in sorted(compact)]


def normalize_moveit_body(name: str) -> str:
    value = str(name)
    if value == "v15_14_mujoco_ground":
        return "ground"
    prefix = "collision_proxy__"
    return value[len(prefix) :] if value.startswith(prefix) else value


def moveit_proxy_pairs(contacts: list[dict[str, Any]]) -> list[list[str]]:
    pairs = {
        tuple(
            sorted(
                (
                    normalize_moveit_body(contact["body_1"]),
                    normalize_moveit_body(contact["body_2"]),
                )
            )
        )
        for contact in contacts
    }
    return [list(pair) for pair in sorted(pairs)]


def mujoco_proxy_pairs(contacts: list[dict[str, Any]]) -> list[list[str]]:
    pairs = {tuple(sorted(contact["proxy_pair"])) for contact in contacts}
    return [list(pair) for pair in sorted(pairs)]


def split_proxy_pairs(
    pairs: list[list[str]],
) -> tuple[list[list[str]], list[list[str]]]:
    """Split normalized pairs into robot self and robot/ground contacts."""
    self_pairs = [pair for pair in pairs if "ground" not in pair]
    ground_pairs = [pair for pair in pairs if "ground" in pair]
    return self_pairs, ground_pairs


class CrossValidator(Node):
    def __init__(self) -> None:
        super().__init__("v15_14_collision_cross_validator_503")
        self.client = self.create_client(GetStateValidity, "/check_state_validity")

    def wait_ready(self, timeout_sec: float) -> None:
        if not self.client.wait_for_service(timeout_sec=timeout_sec):
            raise TimeoutError("/check_state_validity is not available")

    def check_moveit(self, radians: list[float], timeout_sec: float) -> dict[str, Any]:
        request = GetStateValidity.Request()
        request.group_name = "arm"
        request.robot_state.joint_state = JointState(
            name=JOINT_NAMES,
            position=radians,
        )
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)
        if not future.done():
            raise TimeoutError("/check_state_validity call timed out")
        response = future.result()
        if response is None:
            raise RuntimeError("/check_state_validity returned no response")
        return {
            "valid": bool(response.valid),
            "collision": not bool(response.valid),
            "contacts": [contact_to_dict(row) for row in response.contacts],
            "cost_source_count": len(response.cost_sources),
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--validator", type=Path, required=True)
    parser.add_argument("--guard", type=Path, required=True)
    parser.add_argument("--model", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout", type=float, default=10.0)
    args = parser.parse_args()

    validator_path = args.validator.resolve()
    guard_path = args.guard.resolve()
    model_path = args.model.resolve()
    for required in (validator_path, guard_path, model_path):
        if not required.is_file():
            raise FileNotFoundError(required)

    validator = load_module(validator_path, "v15_13_validator_pose_source")
    guard_module = load_module(guard_path, "v15_13_kinematic_guard_source")
    pose_sets = validator.build_pose_sets()
    total_pose_count = sum(len(rows) for rows in pose_sets.values())
    pose_set_payload = json.dumps(
        pose_sets, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    pose_set_sha256 = hashlib.sha256(pose_set_payload).hexdigest()
    pose_set_hash_pass = pose_set_sha256 == EXPECTED_POSE_SET_SHA256
    if total_pose_count != EXPECTED_TOTAL_POSE_COUNT:
        raise RuntimeError(
            f"frozen pose source produced {total_pose_count}, "
            f"expected {EXPECTED_TOTAL_POSE_COUNT}"
        )

    input_hashes = {
        "validator": sha256(validator_path),
        "guard": sha256(guard_path),
        "model": sha256(model_path),
    }
    input_hash_gates = {
        name: {
            "expected_sha256": expected,
            "actual_sha256": input_hashes[name],
            "pass": input_hashes[name].lower() == expected,
        }
        for name, expected in EXPECTED_INPUT_SHA256.items()
    }
    input_hashes_pass = all(gate["pass"] for gate in input_hash_gates.values())

    guard = guard_module.KinematicGuard(model_path=model_path)
    # V15.14 closes the two accepted directly-connected-link checks by
    # disabling MuJoCo's broad parent filter in memory. The exact V15.13
    # runtime_full_pairs/excluded-pairs contract remains the authority.
    guard.model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
    args.output.parent.mkdir(parents=True, exist_ok=True)

    rclpy.init()
    node = CrossValidator()
    try:
        node.wait_ready(args.timeout)
        rows: list[dict[str, Any]] = []
        category_summary: dict[str, Any] = {}
        global_index = 0
        for category, poses in pose_sets.items():
            category_rows = []
            for pose_index, pose in enumerate(poses):
                degrees = [float(pose[f"j{index}_deg"]) for index in range(1, 7)]
                radians = [math.radians(value) for value in degrees]
                mujoco_result = guard.check_pose_deg(degrees)
                if mujoco_result["reason"] == "position_limit":
                    raise RuntimeError(
                        f"frozen pose unexpectedly violates MuJoCo limits: {category}[{pose_index}]"
                    )
                moveit_result = node.check_moveit(radians, args.timeout)
                compact_contacts = compact_mujoco_contacts(mujoco_result["contacts"])
                mujoco_self_collision = any(
                    row["contact_class"] == "self_collision" for row in compact_contacts
                )
                mujoco_ground_collision = any(
                    row["contact_class"] == "ground" for row in compact_contacts
                )
                mujoco_collision = bool(
                    mujoco_self_collision or mujoco_ground_collision
                )
                moveit_pairs = moveit_proxy_pairs(moveit_result["contacts"])
                mujoco_pairs = mujoco_proxy_pairs(compact_contacts)
                moveit_self_pairs, moveit_ground_pairs = split_proxy_pairs(moveit_pairs)
                mujoco_self_pairs, mujoco_ground_pairs = split_proxy_pairs(mujoco_pairs)
                moveit_self_collision = bool(moveit_self_pairs)
                moveit_ground_collision = bool(moveit_ground_pairs)
                moveit_result["self_collision"] = moveit_self_collision
                moveit_result["ground_collision"] = moveit_ground_collision

                overall_boolean_match = (
                    moveit_result["collision"] == mujoco_collision
                )
                self_boolean_match = (
                    moveit_self_collision == mujoco_self_collision
                )
                ground_boolean_match = (
                    moveit_ground_collision == mujoco_ground_collision
                )
                required_booleans_match = bool(
                    overall_boolean_match
                    and self_boolean_match
                    and ground_boolean_match
                )
                self_pair_set_match = moveit_self_pairs == mujoco_self_pairs
                # Ground proxy/facet ownership differs between the two collision
                # engines.  Preserve its exact pair comparison as a diagnostic,
                # while keeping the exact robot self-pair set fail-closed.
                ground_pair_set_match = moveit_ground_pairs == mujoco_ground_pairs
                match = bool(required_booleans_match and self_pair_set_match)
                row = {
                    "global_index": global_index,
                    "category": category,
                    "pose_index": pose_index,
                    "angles_deg": dict(zip(JOINT_NAMES, degrees)),
                    "moveit": moveit_result,
                    "moveit_proxy_pairs": moveit_pairs,
                    "moveit_self_proxy_pairs": moveit_self_pairs,
                    "moveit_ground_proxy_pairs": moveit_ground_pairs,
                    "mujoco": {
                        "safe": bool(mujoco_result["safe"]),
                        "collision": mujoco_collision,
                        "self_collision": mujoco_self_collision,
                        "ground_collision": mujoco_ground_collision,
                        "reason": mujoco_result["reason"],
                        "contacts": compact_contacts,
                    },
                    "mujoco_proxy_pairs": mujoco_pairs,
                    "mujoco_self_proxy_pairs": mujoco_self_pairs,
                    "mujoco_ground_proxy_pairs": mujoco_ground_pairs,
                    "overall_collision_boolean_match": overall_boolean_match,
                    "self_collision_boolean_match": self_boolean_match,
                    "ground_collision_boolean_match": ground_boolean_match,
                    "required_collision_booleans_match": required_booleans_match,
                    "self_collision_pair_set_match": self_pair_set_match,
                    "ground_collision_pair_set_match": ground_pair_set_match,
                    # Compatibility fields now carry the complete required
                    # boolean gate and the exact self-pair gate respectively.
                    "collision_boolean_match": required_booleans_match,
                    "collision_pair_set_match": self_pair_set_match,
                    "collision_match": match,
                }
                rows.append(row)
                category_rows.append(row)
                global_index += 1

            mismatches = [row["pose_index"] for row in category_rows if not row["collision_match"]]
            boolean_mismatches = [
                row["pose_index"]
                for row in category_rows
                if not row["required_collision_booleans_match"]
            ]
            overall_boolean_mismatches = [
                row["pose_index"]
                for row in category_rows
                if not row["overall_collision_boolean_match"]
            ]
            self_boolean_mismatches = [
                row["pose_index"]
                for row in category_rows
                if not row["self_collision_boolean_match"]
            ]
            ground_boolean_mismatches = [
                row["pose_index"]
                for row in category_rows
                if not row["ground_collision_boolean_match"]
            ]
            pair_mismatches = [
                row["pose_index"]
                for row in category_rows
                if not row["self_collision_pair_set_match"]
            ]
            ground_pair_mismatches = [
                row["pose_index"]
                for row in category_rows
                if not row["ground_collision_pair_set_match"]
            ]
            moveit_collision_indices = [row["pose_index"] for row in category_rows if row["moveit"]["collision"]]
            moveit_self_collision_indices = [row["pose_index"] for row in category_rows if row["moveit"]["self_collision"]]
            moveit_ground_indices = [row["pose_index"] for row in category_rows if row["moveit"]["ground_collision"]]
            mujoco_collision_indices = [row["pose_index"] for row in category_rows if row["mujoco"]["collision"]]
            mujoco_self_collision_indices = [row["pose_index"] for row in category_rows if row["mujoco"]["self_collision"]]
            mujoco_ground_indices = [row["pose_index"] for row in category_rows if row["mujoco"]["ground_collision"]]
            category_summary[category] = {
                "pose_count": len(category_rows),
                "moveit_collision_count": len(moveit_collision_indices),
                "moveit_self_collision_count": len(moveit_self_collision_indices),
                "moveit_ground_collision_count": len(moveit_ground_indices),
                "mujoco_collision_count": len(mujoco_collision_indices),
                "mujoco_self_collision_count": len(mujoco_self_collision_indices),
                "moveit_collision_indices": moveit_collision_indices,
                "moveit_self_collision_indices": moveit_self_collision_indices,
                "moveit_ground_collision_indices": moveit_ground_indices,
                "mujoco_collision_indices": mujoco_collision_indices,
                "mujoco_self_collision_indices": mujoco_self_collision_indices,
                "mujoco_ground_collision_count": len(mujoco_ground_indices),
                "mujoco_ground_collision_indices": mujoco_ground_indices,
                "mismatch_indices": mismatches,
                "boolean_mismatch_indices": boolean_mismatches,
                "overall_boolean_mismatch_indices": overall_boolean_mismatches,
                "self_boolean_mismatch_indices": self_boolean_mismatches,
                "ground_boolean_mismatch_indices": ground_boolean_mismatches,
                "pair_set_mismatch_indices": pair_mismatches,
                "self_pair_set_mismatch_indices": pair_mismatches,
                "ground_pair_set_diagnostic_mismatch_indices": ground_pair_mismatches,
                "match_count": len(category_rows) - len(mismatches),
            }
            print(
                f"{category}: poses={len(category_rows)} "
                f"moveit_collision={len(moveit_collision_indices)} "
                f"mujoco_collision={len(mujoco_collision_indices)} "
                f"required_mismatch={len(mismatches)} "
                f"ground_pair_diagnostic_mismatch={len(ground_pair_mismatches)}",
                flush=True,
            )

        mismatch_rows = [row for row in rows if not row["collision_match"]]
        boolean_mismatch_rows = [
            row for row in rows if not row["required_collision_booleans_match"]
        ]
        overall_boolean_mismatch_rows = [
            row for row in rows if not row["overall_collision_boolean_match"]
        ]
        self_boolean_mismatch_rows = [
            row for row in rows if not row["self_collision_boolean_match"]
        ]
        ground_boolean_mismatch_rows = [
            row for row in rows if not row["ground_collision_boolean_match"]
        ]
        pair_mismatch_rows = [
            row for row in rows if not row["self_collision_pair_set_match"]
        ]
        ground_pair_mismatch_rows = [
            row for row in rows if not row["ground_collision_pair_set_match"]
        ]
        moveit_collision_count = sum(row["moveit"]["collision"] for row in rows)
        moveit_self_collision_count = sum(
            row["moveit"]["self_collision"] for row in rows
        )
        moveit_ground_collision_count = sum(
            row["moveit"]["ground_collision"] for row in rows
        )
        mujoco_collision_count = sum(row["mujoco"]["collision"] for row in rows)
        mujoco_self_collision_count = sum(row["mujoco"]["self_collision"] for row in rows)
        mujoco_ground_collision_count = sum(row["mujoco"]["ground_collision"] for row in rows)

        negative_controls = []
        for category, indices in REFERENCE_TERMINAL_COLLISION_INDICES.items():
            by_index = {
                row["pose_index"]: row for row in rows if row["category"] == category
            }
            for pose_index in indices:
                row = by_index[pose_index]
                negative_controls.append(
                    {
                        "category": category,
                        "pose_index": pose_index,
                        "moveit_collision": row["moveit"]["self_collision"],
                        "mujoco_collision": row["mujoco"]["self_collision"],
                        "moveit_self_collision": row["moveit"]["self_collision"],
                        "mujoco_self_collision": row["mujoco"]["self_collision"],
                        "pass": row["moveit"]["self_collision"]
                        and row["mujoco"]["self_collision"],
                    }
                )

        all_boolean_match = not boolean_mismatch_rows
        all_pair_sets_match = not pair_mismatch_rows
        all_match = bool(all_boolean_match and all_pair_sets_match)
        negative_controls_pass = bool(negative_controls) and all(
            row["pass"] for row in negative_controls
        )
        actual_collision_counts = {
            "moveit": {
                "overall": int(moveit_collision_count),
                "self": int(moveit_self_collision_count),
                "ground": int(moveit_ground_collision_count),
            },
            "mujoco": {
                "overall": int(mujoco_collision_count),
                "self": int(mujoco_self_collision_count),
                "ground": int(mujoco_ground_collision_count),
            },
        }
        frozen_count_gates = {
            collision_class: {
                "expected_pose_count": expected,
                "moveit_actual_pose_count": actual_collision_counts["moveit"][collision_class],
                "mujoco_actual_pose_count": actual_collision_counts["mujoco"][collision_class],
                "pass": (
                    actual_collision_counts["moveit"][collision_class] == expected
                    and actual_collision_counts["mujoco"][collision_class] == expected
                ),
            }
            for collision_class, expected in EXPECTED_COLLISION_POSE_COUNTS.items()
        }
        frozen_counts_pass = all(
            gate["pass"] for gate in frozen_count_gates.values()
        )
        pose_count_gate_pass = total_pose_count == EXPECTED_TOTAL_POSE_COUNT
        status_pass = bool(
            pose_count_gate_pass
            and pose_set_hash_pass
            and input_hashes_pass
            and frozen_counts_pass
            and negative_controls_pass
            and all_match
        )
        report = {
            "status": "PASS" if status_pass else "FAIL",
            "revision": "V15.14-MoveIt-MuJoCo-503-pose-collision-cross-regression",
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "method": {
                "moveit": "/check_state_validity with explicit J1..J6 RobotState, group arm, including the installed V15.14 ground object",
                "mujoco": "V15.14 KinematicGuard.check_pose_deg using runtime_full_pairs and ground contacts",
                "comparison": (
                    "per pose: exact overall/self/ground collision booleans plus "
                    "the exact normalized robot self proxy-pair set"
                ),
                "ground_pair_comparison": (
                    "exact normalized robot/ground proxy-pair set is diagnostic "
                    "only: MoveIt uses a finite 4x4x0.02 m slab spanning "
                    "z=[-0.11,-0.09] m, while MuJoCo uses the infinite "
                    "half-space z<=-0.09 m; the specialized UpperArm_Motion "
                    "proxy is also intentionally ground-disabled in MuJoCo"
                ),
                "mujoco_runtime_option": (
                    "mjDSBL_FILTERPARENT enabled in disableflags so audited "
                    "direct parent/child pairs reach KinematicGuard"
                ),
                "ground_scope": (
                    "MoveIt and MuJoCo share the ground top elevation z=-0.09 m "
                    "but not identical ground geometry or proxy affinity; "
                    "ground class booleans and robot self-collision are included"
                ),
                "models_are_not_modified": True,
            },
            "inputs": {
                "validator": str(validator_path),
                "validator_sha256": input_hashes["validator"],
                "guard": str(guard_path),
                "guard_sha256": input_hashes["guard"],
                "model": str(model_path),
                "model_sha256": input_hashes["model"],
            },
            "input_hash_gates": input_hash_gates,
            "input_hashes_pass": input_hashes_pass,
            "expected_total_pose_count": EXPECTED_TOTAL_POSE_COUNT,
            "total_pose_count": total_pose_count,
            "pose_count_gate_pass": pose_count_gate_pass,
            "frozen_pose_set_gate": {
                "expected_sha256": EXPECTED_POSE_SET_SHA256,
                "actual_sha256": pose_set_sha256,
                "pass": pose_set_hash_pass,
            },
            "match_count": total_pose_count - len(mismatch_rows),
            "mismatch_count": len(mismatch_rows),
            "boolean_mismatch_count": len(boolean_mismatch_rows),
            "overall_boolean_mismatch_count": len(overall_boolean_mismatch_rows),
            "self_boolean_mismatch_count": len(self_boolean_mismatch_rows),
            "ground_boolean_mismatch_count": len(ground_boolean_mismatch_rows),
            "pair_set_mismatch_count": len(pair_mismatch_rows),
            "self_pair_set_mismatch_count": len(pair_mismatch_rows),
            "ground_pair_set_diagnostic_mismatch_count": len(
                ground_pair_mismatch_rows
            ),
            "ground_pair_set_diagnostic_match_count": total_pose_count
            - len(ground_pair_mismatch_rows),
            "moveit_collision_pose_count": int(moveit_collision_count),
            "moveit_self_collision_pose_count": int(moveit_self_collision_count),
            "moveit_ground_collision_pose_count": int(moveit_ground_collision_count),
            "mujoco_collision_pose_count": int(mujoco_collision_count),
            "mujoco_self_collision_pose_count": int(mujoco_self_collision_count),
            "mujoco_ground_collision_pose_count": int(mujoco_ground_collision_count),
            "expected_frozen_collision_pose_counts": EXPECTED_COLLISION_POSE_COUNTS,
            "actual_collision_pose_counts": actual_collision_counts,
            "frozen_collision_count_gates": frozen_count_gates,
            "frozen_collision_counts_pass": frozen_counts_pass,
            "all_collision_booleans_match": all_boolean_match,
            "all_overall_collision_booleans_match": not overall_boolean_mismatch_rows,
            "all_self_collision_booleans_match": not self_boolean_mismatch_rows,
            "all_ground_collision_booleans_match": not ground_boolean_mismatch_rows,
            "all_collision_pair_sets_match": all_pair_sets_match,
            "all_self_collision_pair_sets_match": all_pair_sets_match,
            "all_ground_collision_pair_sets_match_diagnostic": not ground_pair_mismatch_rows,
            "reference_terminal_collision_negative_controls": negative_controls,
            "negative_control_count": len(negative_controls),
            "negative_controls_pass": negative_controls_pass,
            "mujoco_parent_child_filter": {
                "accepted_model_modified": False,
                "in_memory_disable_bit": "mjDSBL_FILTERPARENT",
                "enabled_by_guard": True,
            },
            "categories": category_summary,
            "mismatches": mismatch_rows,
            "poses": rows,
        }
        args.output.write_text(
            json.dumps(report, ensure_ascii=False, indent=2) + "\n",
            encoding="utf-8",
        )
        print(
            json.dumps(
                {
                    "status": report["status"],
                    "total_pose_count": total_pose_count,
                    "match_count": report["match_count"],
                    "mismatch_count": report["mismatch_count"],
                    "overall_boolean_mismatch_count": report[
                        "overall_boolean_mismatch_count"
                    ],
                    "self_boolean_mismatch_count": report[
                        "self_boolean_mismatch_count"
                    ],
                    "ground_boolean_mismatch_count": report[
                        "ground_boolean_mismatch_count"
                    ],
                    "self_pair_set_mismatch_count": report[
                        "self_pair_set_mismatch_count"
                    ],
                    "ground_pair_set_diagnostic_mismatch_count": report[
                        "ground_pair_set_diagnostic_mismatch_count"
                    ],
                    "moveit_collision_pose_count": report["moveit_collision_pose_count"],
                    "moveit_self_collision_pose_count": report[
                        "moveit_self_collision_pose_count"
                    ],
                    "moveit_ground_collision_pose_count": report[
                        "moveit_ground_collision_pose_count"
                    ],
                    "mujoco_collision_pose_count": report["mujoco_collision_pose_count"],
                    "mujoco_self_collision_pose_count": report[
                        "mujoco_self_collision_pose_count"
                    ],
                    "mujoco_ground_collision_pose_count": report["mujoco_ground_collision_pose_count"],
                    "input_hashes_pass": input_hashes_pass,
                    "frozen_pose_set_hash_pass": pose_set_hash_pass,
                    "frozen_collision_counts_pass": frozen_counts_pass,
                    "negative_controls_pass": negative_controls_pass,
                    "output": str(args.output),
                },
                ensure_ascii=False,
            ),
            flush=True,
        )
        return 0 if report["status"] == "PASS" else 2
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

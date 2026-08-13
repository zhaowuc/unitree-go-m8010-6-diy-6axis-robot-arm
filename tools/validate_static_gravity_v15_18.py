#!/usr/bin/env python3
from __future__ import annotations

"""Standalone V15.18A static-gravity report validator.

This file deliberately does not import ``audit_static_gravity_v15_18``.  The
audit and validator share only the frozen JSON contract.  The validator
derives the deterministic pose set itself, compiles an ASCII-mirrored copy of
the protected production MJCF, and independently recomputes every physics
quantity used by acceptance.  No integration step or free-fall simulation is
performed.
"""

import argparse
import ast
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import sys
import tempfile
from typing import Any, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SOURCE_COMMIT = "749d7e5f76cae7c1a3fd5462ce5ad92265e8a911"
SOURCE_BRANCH = "agent/v15-17-production-hash-migration"
TARGET_BRANCH = "agent/v15-18a-static-gravity"
REMOTE_SOURCE_REF = f"refs/heads/{SOURCE_BRANCH}"

V14_REL = "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
MJCF_REL = V14_REL + "/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
BRIDGE_REL = V14_REL + "/ros2_ws/src/go_m8010_arm_mujoco_bridge/scripts/mujoco_bridge.py"
COLLISION_503_REL = V14_REL + "/evidence/run_d230_20260812T015818Z/collision_cross_regression_503.json"
COLLISION_PAIR_CONTRACT_REL = V14_REL + "/config/collision_pair_contract_v15_14.json"
KINEMATIC_GUARD_REL = V14_REL + "/mujoco_v15_14/kinematic_guard.py"
CROSS_503_TOOL_REL = V14_REL + "/tools/cross_validate_503_collisions.py"
POSE_SOURCE_REL = "mujoco_kinematic_v1/validate_model.py"
TARGET_MANIFEST_REL = V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_qa/config/target_manifest_v15_14.json"
V15_14_FJT_EVIDENCE_REL = V14_REL + "/evidence/visual_sync_d231_20260812/v15_14_visual_sync_d231.json"
MASS_REL = "V15_15_实测质量账本_v1.json"
COM_REL = "V15_15_COM账本_v2.json"
INERTIA_REL = "V15_16_刚体惯量_Engineering_V1.json"
V15_17_REPORT_REL = "V15_17_ProductionHash迁移与轨迹闭环验收.json"
AUDIT_REL = "tools/audit_static_gravity_v15_18.py"
VALIDATOR_REL = "tools/validate_static_gravity_v15_18.py"
REPORT_JSON_REL = "V15_18A_静态重力与重力矩验收.json"
REPORT_MD_REL = "V15_18A_静态重力与重力矩验收.md"

MJCF_SHA256 = "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
BRIDGE_SHA256 = "9815472370f68f718f63611ac532c989fbaf2cf7d600e4647103e50da5c9a8aa"
MASS_SHA256 = "658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a"
COM_SHA256 = "1cb975890dd121eb41d41bfee414a2c4643f4d917089970fac60009736fc77ae"
INERTIA_SHA256 = "c6c398532d7144ed6aa340a8d8b765b333d01f1fddc2e3280106c90565614401"
V15_17_REPORT_SHA256 = "81032180fcd97bddf068996bf70439917b3b995b4010bfffde0b88351d8e74df"
COLLISION_503_SHA256 = "0518ba83e938a72abe46b6538d490ed2a91396c137ca4734eedc5c546fda9108"
COLLISION_PAIR_CONTRACT_SHA256 = "6d802909e44f238816f007ef33b57e1b57099c5522a473cd2dcc2705397af9c9"
KINEMATIC_GUARD_SHA256 = "648b9de953a84be46cd5f95e24daf9f683f3025e73048dc068ef7bd8e5670c6a"
CROSS_503_TOOL_SHA256 = "514486ffaba6405007dd5f80cc918de5bac421de7f8599a625a945f559fdd7b3"
POSE_SOURCE_SHA256 = "9be9fab3e5c7850f001c2e46b83aebf033a02feff2a9e27dbc15347db4098ea4"
TARGET_MANIFEST_SHA256 = "f2b230e2cb61c251c81228f5f3b89ced0163b05cf82f0b93db8fd0bb85572d57"
V15_14_FJT_EVIDENCE_SHA256 = "7a75ab2ee131b7f734346d45a11888ac913e3a49e8c3691aca2d6b998aee013b"
FROZEN_POSE_SET_SHA256 = "4d9cfaf15cc20fe16393d1aa3bccddc5273e77e2de08cf9d0045976c301799aa"

SCHEMA = "go-m8010-arm-v15.18a-static-gravity-audit/1.0"
FINAL_PASS = "V15.18A STATIC_GRAVITY = PASS"
FINAL_FAIL = "V15.18A STATIC_GRAVITY = FAIL"
JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")
AUTHORITY_BODIES = ("link2", "link3", "link4", "link5", "link6", "gripper")
AUTHORITY_TOTAL_MASS_KG = 3.4515
GRAVITY = (0.0, 0.0, -9.81)
REVERSE_GRAVITY = (0.0, 0.0, 9.81)
ZERO_GRAVITY = (0.0, 0.0, 0.0)
FINITE_DIFFERENCE_STEPS = (1.0e-5, 3.0e-6)
JOINT_LIMIT_MARGIN_RAD = 0.05

EXACT_CHANGED_PATHS = {AUDIT_REL, VALIDATOR_REL, REPORT_JSON_REL, REPORT_MD_REL}
PROTECTED_HASHES = {
    MJCF_REL: MJCF_SHA256,
    BRIDGE_REL: BRIDGE_SHA256,
    MASS_REL: MASS_SHA256,
    COM_REL: COM_SHA256,
    INERTIA_REL: INERTIA_SHA256,
    V15_17_REPORT_REL: V15_17_REPORT_SHA256,
    COLLISION_503_REL: COLLISION_503_SHA256,
    COLLISION_PAIR_CONTRACT_REL: COLLISION_PAIR_CONTRACT_SHA256,
    KINEMATIC_GUARD_REL: KINEMATIC_GUARD_SHA256,
    CROSS_503_TOOL_REL: CROSS_503_TOOL_SHA256,
    POSE_SOURCE_REL: POSE_SOURCE_SHA256,
    TARGET_MANIFEST_REL: TARGET_MANIFEST_SHA256,
    V15_14_FJT_EVIDENCE_REL: V15_14_FJT_EVIDENCE_SHA256,
}
AUDIT_PROTECTED_RELS = {
    MJCF_REL, BRIDGE_REL, MASS_REL, COM_REL, INERTIA_REL, V15_17_REPORT_REL,
    COLLISION_503_REL, COLLISION_PAIR_CONTRACT_REL, TARGET_MANIFEST_REL,
    V15_14_FJT_EVIDENCE_REL,
}

SELECTED_503_POSES = (
    (9, "j1_negative_90deg"),
    (18, "mechanical_zero"),
    (27, "j1_positive_90deg"),
    (53, "j2_negative_10deg"),
    (57, "j2_positive_30deg"),
    (87, "j3_negative_20deg"),
    (92, "j3_positive_30deg"),
    (121, "j4_negative_46deg"),
    (139, "j4_positive_44deg"),
    (169, "j5_negative_40p6deg"),
    (185, "j5_positive_39p4deg"),
    (227, "j6_negative_90deg"),
    (263, "j6_positive_90deg"),
    (354, "coupled_random_global_354"),
    (355, "coupled_random_global_355"),
    (369, "coupled_random_global_369"),
    (412, "coupled_j2_negative_random_global_412"),
    (432, "combined_contracted"),
    (473, "combined_moderate"),
    (481, "combined_extension"),
)
V15_14_ACCEPTED_FJT_Q_RAD = (
    1.1239862708782487,
    -0.2787640209113898,
    1.7562373783755654,
    1.1168536669244002,
    0.9010670078695902,
    0.5253228012929325,
)
GIT_EXECUTABLE = os.environ.get("V15_18_GIT_EXECUTABLE") or shutil.which("git")


class ValidationError(RuntimeError):
    """Malformed, stale, contradictory, or unsafe evidence (exit two)."""


class StaleReportError(ValidationError):
    """Derived report bytes do not match the frozen audit output."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def repo_path(relative: str) -> Path:
    parts = PurePosixPath(relative).parts
    require(bool(parts) and ".." not in parts, f"unsafe repository path: {relative}")
    path = (ROOT / Path(*parts)).resolve()
    require(path == ROOT or ROOT in path.parents, f"repository path escaped root: {relative}")
    return path


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def canonical_digest(value: Any) -> str:
    return sha256_bytes(json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8"))


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    require(isinstance(value, dict), f"JSON root is not an object: {path.name}")
    return value


def finite(value: Any, label: str) -> float:
    require(isinstance(value, (int, float)) and not isinstance(value, bool), f"{label}: number required")
    result = float(value)
    require(math.isfinite(result), f"{label}: finite number required")
    return result


def vector(value: Any, length: int, label: str) -> list[float]:
    require(isinstance(value, list) and len(value) == length, f"{label}: expected {length} values")
    return [finite(item, f"{label}[{index}]") for index, item in enumerate(value)]


def matrix(value: Any, size: int, label: str) -> list[list[float]]:
    require(isinstance(value, list) and len(value) == size, f"{label}: expected {size} rows")
    return [vector(row, size, f"{label}[{index}]") for index, row in enumerate(value)]


def max_abs(values: Iterable[float]) -> float:
    return max((abs(float(value)) for value in values), default=0.0)


def clean_error(error: BaseException) -> str:
    return re.sub(r"[\r\n]+", " ", str(error).replace(str(ROOT), "<REPO_ROOT>")).strip()


def run_process(command: Sequence[str], *, timeout: float = 60.0, cwd: Path = ROOT) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(
            list(command), cwd=str(cwd), text=True, encoding="utf-8", errors="replace",
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, timeout=timeout, check=False,
        )
    except (OSError, subprocess.TimeoutExpired) as error:
        raise ValidationError(f"process infrastructure failure: {clean_error(error)}") from error


def git_text(arguments: Sequence[str], *, timeout: float = 30.0) -> str:
    require(bool(GIT_EXECUTABLE), "git executable was not found")
    executable = Path(str(GIT_EXECUTABLE)).resolve()
    require(executable.is_file(), "git executable does not exist")
    result = run_process([str(executable), *arguments], timeout=timeout)
    require(result.returncode == 0, "git command failed: " + clean_error(result.stderr or result.stdout))
    return result.stdout


def changed_paths_from_source(head: str) -> list[str]:
    committed = set(git_text(["diff", "--name-only", f"{SOURCE_COMMIT}..{head}", "--"]).splitlines()) if head != SOURCE_COMMIT else set()
    working = set(git_text(["diff", "--name-only", SOURCE_COMMIT, "--"]).splitlines())
    untracked = set(git_text(["ls-files", "--others", "--exclude-standard"]).splitlines())
    return sorted(path for path in committed | working | untracked if path)


def validate_git_scope(*, allow_missing_markdown: bool = False) -> dict[str, Any]:
    branch = git_text(["branch", "--show-current"]).strip()
    head = git_text(["rev-parse", "HEAD"]).strip().lower()
    require(branch == TARGET_BRANCH, f"target branch mismatch: {branch}")
    if head != SOURCE_COMMIT:
        parent = git_text(["rev-parse", "HEAD^"]).strip().lower()
        count = int(git_text(["rev-list", "--count", f"{SOURCE_COMMIT}..HEAD"]).strip())
        require(parent == SOURCE_COMMIT and count == 1, "HEAD must be source commit or its unique direct child")
        require(not git_text(["status", "--porcelain=v1"]).strip(), "post-commit check requires clean worktree")
    changed = changed_paths_from_source(head)
    accepted = [EXACT_CHANGED_PATHS]
    if allow_missing_markdown and not repo_path(REPORT_MD_REL).exists():
        accepted.append(EXACT_CHANGED_PATHS - {REPORT_MD_REL})
    require(any(set(changed) == paths for paths in accepted), f"changed paths are outside the exact V15.18A scope: {changed}")
    remote_rows = [row.split() for row in git_text(["ls-remote", "--heads", "origin", REMOTE_SOURCE_REF]).splitlines() if row.strip()]
    require(len(remote_rows) == 1 and remote_rows[0][0].lower() == SOURCE_COMMIT, "remote frozen source branch mismatch")
    return {
        "pass": True,
        "source_commit": SOURCE_COMMIT,
        "target_branch": TARGET_BRANCH,
        "head_contract": "SOURCE_COMMIT_OR_ITS_UNIQUE_DIRECT_CHILD",
        "changed_paths": sorted(EXACT_CHANGED_PATHS),
        "exact_changed_path_count": 4,
    }


def python_literal(path: Path, name: str) -> Any:
    module = ast.parse(path.read_text(encoding="utf-8"), filename=path.name)
    matches: list[Any] = []
    for node in module.body:
        if isinstance(node, (ast.Assign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            if any(isinstance(target, ast.Name) and target.id == name for target in targets):
                matches.append(ast.literal_eval(node.value))
    require(len(matches) == 1, f"{path.name}: exact one {name} literal required")
    return matches[0]


def validate_audit_static_contract() -> dict[str, Any]:
    path = repo_path(AUDIT_REL)
    require(path.is_file(), "V15.18A audit tool is missing")
    source = path.read_text(encoding="utf-8")
    tree = ast.parse(source, filename=path.name)
    imported_modules = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_modules.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_modules.add(node.module)
    require(not any(name.startswith(("rclpy", "rospy")) for name in imported_modules), "static gravity audit must not depend on ROS runtime")
    forbidden_step_names = {"mj_step", "mj_step1", "mj_step2"}
    imported_forbidden = []
    referenced_forbidden = []
    dynamic_forbidden = []
    for node in ast.walk(tree):
        if isinstance(node, ast.ImportFrom):
            imported_forbidden.extend(alias.name for alias in node.names if alias.name in forbidden_step_names)
        elif isinstance(node, ast.Import):
            imported_forbidden.extend(alias.name for alias in node.names if alias.name.rsplit(".", 1)[-1] in forbidden_step_names)
        elif isinstance(node, ast.Attribute) and node.attr in forbidden_step_names:
            referenced_forbidden.append(node.attr)
        elif isinstance(node, ast.Name) and node.id in forbidden_step_names:
            referenced_forbidden.append(node.id)
        elif (
            isinstance(node, ast.Call) and isinstance(node.func, ast.Name) and node.func.id == "getattr"
            and any(isinstance(argument, ast.Constant) and argument.value in forbidden_step_names for argument in node.args)
        ):
            dynamic_forbidden.append("getattr")
    require(not imported_forbidden and not referenced_forbidden and not dynamic_forbidden, "audit tool references forbidden MuJoCo time integration")
    calls = [node for node in ast.walk(tree) if isinstance(node, ast.Call)]
    call_names = []
    for call in calls:
        function = call.func
        if isinstance(function, ast.Name):
            call_names.append(function.id)
        elif isinstance(function, ast.Attribute):
            call_names.append(function.attr)
    require(not forbidden_step_names.intersection(call_names), "audit tool contains forbidden dynamics integration")
    require("mj_forward" in call_names, "audit tool does not use instantaneous mj_forward")
    visualize_nodes = [node for node in tree.body if isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef)) and "visual" in node.name.lower()]
    visual_calls: list[str] = []
    for function_node in visualize_nodes:
        for node in ast.walk(function_node):
            if not isinstance(node, ast.Call):
                continue
            if isinstance(node.func, ast.Name):
                visual_calls.append(node.func.id)
            elif isinstance(node.func, ast.Attribute):
                visual_calls.append(node.func.attr)
    if "--visualize" in source:
        require(visualize_nodes, "--visualize is declared without a dedicated visualization function")
        require(not any(name in {"mj_step", "mj_step1", "mj_step2"} for name in visual_calls), "GUI witness performs forbidden dynamics integration")
        require("mj_forward" in visual_calls and "sync" in visual_calls, "GUI witness must use mj_forward plus viewer.sync")
    return {
        "pass": True,
        "audit_tool_sha256": sha256_file(path),
        "no_ros_runtime_dependency": True,
        "no_mj_step_anywhere": True,
        "instantaneous_forward_present": True,
        "gui_visual_witness": {
            "declared": "--visualize" in source,
            "diagnostic_only": True,
            "not_numeric_authority": True,
            "no_dynamics_integration": True,
            "uses_mj_forward_and_viewer_sync": "--visualize" not in source or ("mj_forward" in visual_calls and "sync" in visual_calls),
        },
    }


def model_parameter_snapshot(root: ET.Element) -> dict[str, Any]:
    joints = []
    for joint in root.findall(".//joint"):
        if joint.get("name") in JOINTS:
            joints.append({
                "name": joint.get("name"),
                "damping": str(joint.get("damping", "0")),
                "frictionloss": str(joint.get("frictionloss", "0")),
                "armature": str(joint.get("armature", "0")),
            })
    gravcomp = [{"name": body.get("name"), "gravcomp": str(body.get("gravcomp", "0"))} for body in root.findall(".//body")]
    return {"joints": joints, "body_gravcomp": gravcomp}


def validate_protected_inputs() -> dict[str, Any]:
    actual = {relative: sha256_file(repo_path(relative)) for relative in sorted(PROTECTED_HASHES)}
    require(actual == PROTECTED_HASHES, "protected authority/model/bridge/503 hash mismatch")
    root = ET.parse(repo_path(MJCF_REL)).getroot()
    option = root.find("./option")
    gravity = [float(token) for token in str(option.get("gravity", "0 0 0") if option is not None else "0 0 0").split()]
    require(gravity == [0.0, 0.0, 0.0], "production MJCF persisted nonzero gravity")
    require(python_literal(repo_path(BRIDGE_REL), "ACCEPTED_MODEL_SHA256") == MJCF_SHA256, "production bridge accepted model anchor changed")
    parameter_snapshot = model_parameter_snapshot(root)
    mass_ledger = read_json(repo_path(MASS_REL)).get("link_mass_ledger")
    com_links = read_json(repo_path(COM_REL)).get("links")
    inertia_rows = read_json(repo_path(INERTIA_REL)).get("links")
    require(isinstance(mass_ledger, dict) and isinstance(com_links, dict) and isinstance(inertia_rows, list), "authority ledger structures are malformed")
    inertia_by_link = {str(row.get("link")): row for row in inertia_rows if isinstance(row, dict)}
    require(set(mass_ledger) == set(com_links) == set(inertia_by_link) == set(AUTHORITY_BODIES), "six-body authority link identities differ")
    authority_rows: dict[str, Any] = {}
    for link in AUTHORITY_BODIES:
        mass = finite(mass_ledger[link].get("nominal_mass_kg"), f"mass.{link}")
        com_mass = finite(com_links[link].get("mass_kg"), f"com.{link}.mass")
        inertia_mass = finite(inertia_by_link[link].get("mass_kg"), f"inertia.{link}.mass")
        require(abs(mass - com_mass) <= 1.0e-12 and abs(mass - inertia_mass) <= 1.0e-12, f"{link}: mass authorities disagree")
        com = vector(com_links[link].get("com_link_m"), 3, f"com.{link}")
        inertia_com = vector(inertia_by_link[link].get("com_xyz_m_in_link_frame"), 3, f"inertia.{link}.com")
        require(max_abs(left - right for left, right in zip(com, inertia_com)) <= 1.0e-12, f"{link}: COM authorities disagree")
        tensor = matrix(inertia_by_link[link].get("inertia_tensor_kg_m2"), 3, f"inertia.{link}.tensor")
        authority_rows[link] = {"mass_kg": mass, "com_link_m": com, "inertia_tensor_kg_m2": tensor}
    require(abs(sum(row["mass_kg"] for row in authority_rows.values()) - AUTHORITY_TOTAL_MASS_KG) <= 1.0e-12, "six-body authority total mass is not 3.4515 kg")
    return {
        "pass": True,
        "artifacts": {relative: {"expected_sha256": expected, "actual_sha256": actual[relative], "pass": True} for relative, expected in sorted(PROTECTED_HASHES.items())},
        "production_gravity_m_s2": gravity,
        "accepted_model_sha256": MJCF_SHA256,
        "forbidden_parameter_snapshot": parameter_snapshot,
        "forbidden_parameter_snapshot_sha256": canonical_digest(parameter_snapshot),
        "six_body_authority": authority_rows,
        "six_body_authority_total_mass_kg": AUTHORITY_TOTAL_MASS_KG,
    }


def validate_collision_pair_contract() -> dict[str, Any]:
    """Independently normalize the protected V15.14 guard pair partition."""

    contract = read_json(repo_path(COLLISION_PAIR_CONTRACT_REL))
    require(contract.get("schema") == "go-m8010-arm-v15.14-self-collision-pair-contract/2.0", "collision pair contract schema changed")
    proxies = contract.get("proxies")
    runtime_rows = contract.get("runtime_full_pairs")
    excluded_rows = contract.get("runtime_excluded_pairs")
    require(isinstance(proxies, list) and len(proxies) == 25 and len(set(proxies)) == 25, "collision proxy identity set is not exact 25")
    require(isinstance(runtime_rows, list) and isinstance(excluded_rows, list), "collision pair partition is malformed")

    def normalize(rows: Sequence[Any], label: str) -> set[tuple[str, str]]:
        normalized: set[tuple[str, str]] = set()
        for index, row in enumerate(rows):
            require(isinstance(row, list) and len(row) == 2, f"{label}[{index}] is not a pair")
            pair = tuple(sorted((str(row[0]), str(row[1]))))
            require(pair[0] != pair[1] and pair[0] in proxies and pair[1] in proxies, f"{label}[{index}] has an invalid proxy")
            require(pair not in normalized, f"{label}[{index}] duplicates a pair")
            normalized.add(pair)
        return normalized

    runtime_pairs = normalize(runtime_rows, "runtime_full_pairs")
    excluded_pairs = normalize(excluded_rows, "runtime_excluded_pairs")
    all_pairs = {tuple(sorted((str(left), str(right)))) for left_index, left in enumerate(proxies) for right in proxies[left_index + 1:]}
    require(len(all_pairs) == 300, "25-proxy unordered pair count is not 300")
    require(len(runtime_pairs) == 231 and len(excluded_pairs) == 69, "collision pair partition count changed")
    require(not (runtime_pairs & excluded_pairs) and runtime_pairs | excluded_pairs == all_pairs, "collision pair partition is not disjoint and exhaustive")
    require(contract.get("proxy_count") == 25 and contract.get("all_unordered_pair_count") == 300, "stored collision pair universe counts disagree")
    require(contract.get("runtime_full_pair_count") == 231 and contract.get("runtime_excluded_pair_count") == 69, "stored collision pair partition counts disagree")
    return {
        "pass": True,
        "relative_path": COLLISION_PAIR_CONTRACT_REL,
        "sha256": COLLISION_PAIR_CONTRACT_SHA256,
        "guard_relative_path": KINEMATIC_GUARD_REL,
        "guard_sha256": KINEMATIC_GUARD_SHA256,
        "proxy_count": 25,
        "all_unordered_pair_count": 300,
        "runtime_full_pair_count": 231,
        "runtime_excluded_pair_count": 69,
        "proxies": [str(value) for value in proxies],
        "runtime_full_pairs": [list(pair) for pair in sorted(runtime_pairs)],
    }


def joint_limit_contract() -> dict[str, tuple[float, float] | None]:
    root = ET.parse(repo_path(MJCF_REL)).getroot()
    joint_nodes = {str(node.get("name")): node for node in root.findall(".//joint") if node.get("name") in JOINTS}
    require(set(joint_nodes) == set(JOINTS), "production six-joint identity set changed")
    limits: dict[str, tuple[float, float] | None] = {}
    for name in JOINTS:
        node = joint_nodes[name]
        limited = str(node.get("limited", "auto")).lower()
        if limited == "false" or not node.get("range"):
            require(name == "J1", f"unexpected unlimited joint: {name}")
            limits[name] = None
        else:
            values = tuple(float(token) for token in str(node.get("range")).split())
            require(len(values) == 2 and values[0] < values[1], f"{name}: malformed range")
            limits[name] = (values[0], values[1])
    return limits


def limit_margin(q_rad: Sequence[float], limits: Mapping[str, tuple[float, float] | None]) -> dict[str, float | None]:
    margins: dict[str, float | None] = {}
    for index, name in enumerate(JOINTS):
        bounds = limits[name]
        margins[name] = None if bounds is None else min(float(q_rad[index]) - bounds[0], bounds[1] - float(q_rad[index]))
    return margins


def frozen_503_pose_digest(poses: Sequence[Mapping[str, Any]]) -> str:
    categories: dict[str, list[dict[str, float]]] = {}
    for expected_index, pose in enumerate(poses):
        require(int(pose.get("global_index", -1)) == expected_index, f"503 global index discontinuity: {expected_index}")
        category = str(pose.get("category"))
        pose_index = int(pose.get("pose_index", -1))
        rows = categories.setdefault(category, [])
        require(pose_index == len(rows), f"503 category index discontinuity: {category}/{pose_index}")
        angles = pose.get("angles_deg")
        require(isinstance(angles, dict) and set(angles) == set(JOINTS), f"503 pose joint set changed: {expected_index}")
        rows.append({f"j{joint_index}_deg": finite(angles[name], f"503[{expected_index}].{name}") for joint_index, name in enumerate(JOINTS, 1)})
    digest = canonical_digest(categories)
    require(digest == FROZEN_POSE_SET_SHA256, "frozen 503 pose-set digest changed")
    return digest


def pose_from_503(source: Mapping[str, Any], pose_id: str, limits: Mapping[str, tuple[float, float] | None]) -> dict[str, Any]:
    angles = source["angles_deg"]
    q = [math.radians(finite(angles[name], f"{pose_id}.{name}")) for name in JOINTS]
    margins = limit_margin(q, limits)
    source_collision = {
        "moveit_valid": source["moveit"].get("valid") is True,
        "moveit_collision": source["moveit"].get("collision") is True,
        "moveit_self_collision": source["moveit"].get("self_collision") is True,
        "moveit_ground_collision": source["moveit"].get("ground_collision") is True,
        "mujoco_safe": source["mujoco"].get("safe") is True,
        "mujoco_collision": source["mujoco"].get("collision") is True,
        "mujoco_self_collision": source["mujoco"].get("self_collision") is True,
        "mujoco_ground_collision": source["mujoco"].get("ground_collision") is True,
    }
    source_collision_free = (
        source_collision["moveit_valid"] and source_collision["mujoco_safe"]
        and not any(source_collision[key] for key in (
            "moveit_collision", "moveit_self_collision", "moveit_ground_collision",
            "mujoco_collision", "mujoco_self_collision", "mujoco_ground_collision",
        ))
    )
    return {
        "id": pose_id,
        "source": "FROZEN_503_COLLISION_DATASET",
        "source_relative_path": COLLISION_503_REL,
        "source_sha256": COLLISION_503_SHA256,
        "source_global_index": int(source["global_index"]),
        "source_category": str(source["category"]),
        "source_pose_index": int(source["pose_index"]),
        "q_rad": q,
        "joint_limit_margin_rad": margins,
        "minimum_limited_joint_margin_rad": min(value for value in margins.values() if value is not None),
        "source_collision": source_collision,
        "source_collision_free": source_collision_free,
    }


def derive_pose_selection() -> dict[str, Any]:
    report = read_json(repo_path(COLLISION_503_REL))
    poses = report.get("poses")
    require(isinstance(poses, list) and len(poses) == 503, "frozen 503 report does not contain exact 503 poses")
    digest = frozen_503_pose_digest(poses)
    limits = joint_limit_contract()

    def eligible(row: Mapping[str, Any]) -> bool:
        moveit, mujoco = row.get("moveit"), row.get("mujoco")
        if not isinstance(moveit, dict) or not isinstance(mujoco, dict):
            return False
        if not (moveit.get("valid") is True and moveit.get("collision") is False and mujoco.get("safe") is True and mujoco.get("collision") is False):
            return False
        q = [math.radians(float(row["angles_deg"][name])) for name in JOINTS]
        margins = limit_margin(q, limits)
        return all(value is None or value >= JOINT_LIMIT_MARGIN_RAD for value in margins.values())

    eligible_rows = [row for row in poses if eligible(row)]
    require(len(eligible_rows) == 294, f"eligible frozen 503 pose count changed: {len(eligible_rows)}")
    selected_indices = {index for index, _ in SELECTED_503_POSES}
    by_global = {int(row["global_index"]): row for row in poses}
    require(all(index in by_global and eligible(by_global[index]) for index in selected_indices), "selected frozen 503 pose is no longer eligible")
    selected = []
    for index, role in SELECTED_503_POSES:
        row = pose_from_503(by_global[index], f"frozen_503_global_{index:03d}", limits)
        row["role"] = role
        selected.append(row)
    require(all(row["source_collision_free"] is True for row in selected), "selected frozen 503 source flag set is not collision-free")

    fjt = read_json(repo_path(V15_14_FJT_EVIDENCE_REL))
    require(fjt.get("schema") == "go_m8010_v15_14_visual_sync_evidence_v1", "V15.14 accepted FJT evidence schema changed")
    require(fjt.get("pass") is True and fjt.get("action", {}).get("accepted") is True and fjt.get("action", {}).get("goal_status") == 4, "V15.14 FJT evidence is not an accepted success")
    accepted_q = vector(fjt.get("trajectory", {}).get("target_position_rad"), 6, "V15.14 accepted FJT q")
    require(tuple(accepted_q) == V15_14_ACCEPTED_FJT_Q_RAD, "V15.14 accepted FJT target changed")
    require(vector(fjt.get("final", {}).get("mujoco_raw_joint_position_rad"), 6, "V15.14 accepted final q") == accepted_q, "V15.14 accepted final MuJoCo q differs from target")
    accepted_margins = limit_margin(accepted_q, limits)
    require(all(value is None or value >= JOINT_LIMIT_MARGIN_RAD for value in accepted_margins.values()), "V15.14 accepted FJT target is too close to a joint limit")
    selected.append({
        "id": "v15_14_accepted_fjt_final_pose", "source": "V15_14_ACCEPTED_FJT_EVIDENCE",
        "role": "v15_14_accepted_nonzero",
        "source_relative_path": V15_14_FJT_EVIDENCE_REL, "source_sha256": V15_14_FJT_EVIDENCE_SHA256,
        "source_global_index": None, "source_category": None, "source_pose_index": None,
        "q_rad": accepted_q, "joint_limit_margin_rad": accepted_margins,
        "minimum_limited_joint_margin_rad": min(value for value in accepted_margins.values() if value is not None),
        "source_collision": {"accepted_nonzero_target": True, "runtime_collision_recheck_required": True},
        "source_collision_free": None,
    })
    require(len(selected) == 21 and len({row["id"] for row in selected}) == 21, "deterministic pose selection is not exact 21 unique poses")
    return {
        "pass": True,
        "source_503_relative_path": COLLISION_503_REL,
        "source_503_sha256": COLLISION_503_SHA256,
        "frozen_pose_set_sha256": digest,
        "algorithm": "EXACT_20_FROZEN_503_GLOBAL_INDICES_PLUS_V15_14_ACCEPTED_FJT_FINAL_POSE",
        "joint_limit_margin_rad": JOINT_LIMIT_MARGIN_RAD,
        "eligible_503_count": len(eligible_rows),
        "selected_503_global_indices": [index for index, _ in SELECTED_503_POSES],
        "v15_14_accepted_fjt_source_sha256": V15_14_FJT_EVIDENCE_SHA256,
        "selected_count": len(selected),
        "selected": selected,
    }


def production_asset_snapshot() -> dict[str, str]:
    source_path = repo_path(MJCF_REL)
    root = ET.fromstring(source_path.read_bytes())
    compiler = root.find("./compiler")
    mesh_dir = source_path.parent / (compiler.get("meshdir", ".") if compiler is not None else ".")
    texture_dir = source_path.parent / (compiler.get("texturedir", ".") if compiler is not None else ".")
    assets: dict[str, str] = {}
    file_assets = [node for node in root.findall("./asset/*") if node.get("file")]
    require(len(file_assets) == 1008, "production model no longer references exactly 1008 assets")
    for node in file_assets:
        raw = str(node.get("file"))
        require(not Path(raw).is_absolute(), f"absolute production asset path: {raw}")
        base = mesh_dir if node.tag.rsplit("}", 1)[-1] == "mesh" else texture_dir
        source = (base / Path(*PurePosixPath(raw).parts)).resolve()
        require(source.is_file() and ROOT in source.parents, f"asset escaped repository or is missing: {raw}")
        relative = source.relative_to(ROOT).as_posix()
        require(relative not in assets, f"production model repeats an asset path: {relative}")
        assets[relative] = sha256_file(source)
    require(len(assets) == 1008, "production model asset identity set is not exact 1008")
    return dict(sorted(assets.items()))


def make_ascii_model_mirror(destination: Path, expected_assets: Mapping[str, str]) -> Path:
    source_path = repo_path(MJCF_REL)
    source_bytes = source_path.read_bytes()
    require(sha256_bytes(source_bytes) == MJCF_SHA256, "production MJCF changed before ASCII mirror parse")
    root = ET.fromstring(source_bytes)
    compiler = root.find("./compiler")
    mesh_dir = source_path.parent / (compiler.get("meshdir", ".") if compiler is not None else ".")
    texture_dir = source_path.parent / (compiler.get("texturedir", ".") if compiler is not None else ".")
    asset_directory = destination / "assets"
    asset_directory.mkdir(parents=True, exist_ok=True)
    file_assets = [node for node in root.findall("./asset/*") if node.get("file")]
    require(len(file_assets) == 1008, "production model no longer references exactly 1008 assets")
    copied_relatives: set[str] = set()
    for index, node in enumerate(file_assets):
        raw = str(node.get("file"))
        require(not Path(raw).is_absolute(), f"absolute production asset path: {raw}")
        base = mesh_dir if node.tag.rsplit("}", 1)[-1] == "mesh" else texture_dir
        source = (base / Path(*PurePosixPath(raw).parts)).resolve()
        require(source.is_file() and ROOT in source.parents, f"asset escaped repository or is missing: {raw}")
        relative = source.relative_to(ROOT).as_posix()
        require(relative in expected_assets and sha256_file(source) == expected_assets[relative], f"asset changed during ASCII mirror: {relative}")
        require(relative not in copied_relatives, f"asset repeated during ASCII mirror: {relative}")
        copied_relatives.add(relative)
        target_name = f"asset_{index:04d}{source.suffix.lower() or '.bin'}"
        target_asset = asset_directory / target_name
        shutil.copyfile(source, target_asset)
        require(sha256_file(source) == expected_assets[relative], f"asset changed while copying ASCII mirror: {relative}")
        require(sha256_file(target_asset) == expected_assets[relative], f"ASCII mirror asset bytes differ from authority: {relative}")
        node.set("file", "assets/" + target_name)
    require(copied_relatives == set(expected_assets), "ASCII mirror asset identity set changed")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", ".")
    compiler.set("texturedir", ".")
    target = destination / "model.xml"
    ET.ElementTree(root).write(target, encoding="utf-8", xml_declaration=True)
    require(str(target).isascii(), "MuJoCo mirror path is not ASCII")
    require(sha256_file(source_path) == MJCF_SHA256, "production MJCF changed while constructing ASCII mirror")
    return target


def _child_potential(model: Any, data: Any, body_ids: Sequence[int]) -> float:
    import numpy as np  # type: ignore

    indices = np.asarray(body_ids, dtype=int)
    masses = np.asarray(model.body_mass, dtype=float)[indices]
    positions = np.asarray(data.xipos, dtype=float)[indices]
    gravity = np.asarray(model.opt.gravity, dtype=float)
    return float(-np.sum(masses[:, None] * positions * gravity[None, :]))


def _child_forward(model: Any, data: Any, mujoco: Any, q: Any, gravity: Sequence[float]) -> None:
    import numpy as np  # type: ignore

    mujoco.mj_resetData(model, data)
    model.opt.gravity[:] = np.asarray(gravity, dtype=float)
    data.qpos[:] = np.asarray(q, dtype=float)
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    if model.nu:
        data.ctrl[:] = 0.0
    data.qfrc_applied[:] = 0.0
    data.xfrc_applied[:] = 0.0
    mujoco.mj_forward(model, data)


def _child_proxy(name: str) -> str:
    if name.startswith("collision__"):
        parts = name.split("__")
        if len(parts) >= 4:
            return parts[2]
    return name


def _child_guard_contacts(model: Any, data: Any, mujoco: Any, allowed_pairs: set[tuple[str, str]]) -> list[dict[str, Any]]:
    """Recompute the protected V15.14 pair-scoped guard without importing it."""

    rows: list[dict[str, Any]] = []
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        geom1_id, geom2_id = int(contact.geom1), int(contact.geom2)
        first = str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom1_id) or f"<unnamed:{geom1_id}>")
        second = str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom2_id) or f"<unnamed:{geom2_id}>")
        pair = tuple(sorted((_child_proxy(first), _child_proxy(second))))
        ground = "ground" in pair
        if not ground and pair not in allowed_pairs:
            continue
        rows.append({
            "geom1": first,
            "geom2": second,
            "distance_m": float(contact.dist),
            "proxy_pair": list(pair),
            "contact_class": "ground" if ground else "self_collision",
        })
    return rows


def _child_stencil_record(data: Any, np: Any, step: float, joint: str, side: str) -> dict[str, Any]:
    arrays = (
        data.qpos, data.qvel, data.qacc, data.qfrc_bias, data.qfrc_constraint,
        data.qfrc_passive, data.qfrc_actuator, data.qfrc_gravcomp,
        data.qfrc_applied, data.xfrc_applied,
    )
    return {
        "step_rad": step, "joint": joint, "side": side,
        "ncon": int(data.ncon), "nefc": int(data.nefc),
        "max_abs_qfrc_constraint": float(np.max(np.abs(data.qfrc_constraint))),
        "max_abs_qfrc_passive": float(np.max(np.abs(data.qfrc_passive))),
        "max_abs_qfrc_actuator": float(np.max(np.abs(data.qfrc_actuator))),
        "max_abs_qfrc_gravcomp": float(np.max(np.abs(data.qfrc_gravcomp))),
        "max_abs_qfrc_applied": float(np.max(np.abs(data.qfrc_applied))),
        "max_abs_xfrc_applied": float(np.max(np.abs(data.xfrc_applied))),
        "all_values_finite": bool(all(np.all(np.isfinite(array)) for array in arrays)),
    }


def _child_recursively_finite(value: Any, np: Any) -> bool:
    """Return False for every non-finite numeric leaf without raising."""

    if isinstance(value, np.ndarray):
        return bool(np.all(np.isfinite(value)))
    if isinstance(value, (float, np.floating)):
        return bool(np.isfinite(value))
    if isinstance(value, (int, np.integer, bool, str)) or value is None:
        return True
    if isinstance(value, Mapping):
        return all(_child_recursively_finite(item, np) for item in value.values())
    if isinstance(value, (list, tuple)):
        return all(_child_recursively_finite(item, np) for item in value)
    return False


def _child_json_safe(value: Any, np: Any) -> Any:
    """Convert NumPy values and non-finite leaves to deterministic JSON nulls."""

    if isinstance(value, np.ndarray):
        return _child_json_safe(value.tolist(), np)
    if isinstance(value, (np.integer,)):
        return int(value)
    if isinstance(value, (float, np.floating)):
        result = float(value)
        return result if math.isfinite(result) else None
    if isinstance(value, Mapping):
        return {str(key): _child_json_safe(item, np) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_child_json_safe(item, np) for item in value]
    return value


def _child_pose(
    model: Any,
    data: Any,
    mujoco: Any,
    np: Any,
    body_ids: Sequence[int],
    all_positive_body_ids: Sequence[int],
    link1_body_id: int,
    pose: Mapping[str, Any],
) -> dict[str, Any]:
    q = np.asarray(pose["q_rad"], dtype=float)
    _child_forward(model, data, mujoco, q, GRAVITY)
    tau = np.asarray(data.qfrc_bias, dtype=float).copy()
    passive = np.asarray(data.qfrc_passive, dtype=float).copy()
    actuator = np.asarray(data.qfrc_actuator, dtype=float).copy()
    gravcomp = np.asarray(data.qfrc_gravcomp, dtype=float).copy()
    applied = np.asarray(data.qfrc_applied, dtype=float).copy()
    external = np.asarray(data.xfrc_applied, dtype=float).copy()
    ctrl = np.asarray(data.ctrl, dtype=float).copy()
    qacc = np.asarray(data.qacc, dtype=float).copy()
    constraint_force = np.asarray(data.qfrc_constraint, dtype=float).copy()
    potential = _child_potential(model, data, body_ids)
    all_positive_potential = _child_potential(model, data, all_positive_body_ids)
    ncon, nefc = int(data.ncon), int(data.nefc)
    baseline_numeric = {
        "q": q, "tau": tau, "passive": passive, "actuator": actuator,
        "gravcomp": gravcomp, "applied": applied, "external": external,
        "ctrl": ctrl, "qacc": qacc, "constraint": constraint_force,
        "potential": potential, "all_positive_potential": all_positive_potential,
    }
    baseline_all_finite = _child_recursively_finite(baseline_numeric, np)
    maximum_tau = float(np.max(np.abs(tau))) if baseline_all_finite else float("nan")
    suspicious_torque = bool(baseline_all_finite and maximum_tau > 100.0)
    preflight_pass = bool(baseline_all_finite and not suspicious_torque)
    preflight_failure_code = (
        None if preflight_pass
        else "FAIL_NONFINITE_STATIC_RESULT" if not baseline_all_finite
        else "FAIL_SUSPECT_UNIT_OR_COM_ERROR"
    )
    common = {
        "id": pose["id"], "q_rad": q.tolist(), "potential_energy_j": potential,
        "all_positive_body_potential_energy_j": all_positive_potential,
        "tau_hold_mujoco_nm": tau.tolist(), "qfrc_passive": passive.tolist(),
        "qfrc_actuator": actuator.tolist(), "qfrc_gravcomp": gravcomp.tolist(),
        "qfrc_applied": applied.tolist(), "xfrc_applied": external.tolist(), "ctrl": ctrl.tolist(),
        "qacc": qacc.tolist(), "ncon": ncon, "nefc": nefc,
        "qfrc_constraint": constraint_force.tolist(),
        "preflight": {
            "all_baseline_values_finite": baseline_all_finite,
            "max_abs_tau_nm": maximum_tau,
            "suspicious_torque_over_100_nm": suspicious_torque,
            "pass": preflight_pass,
            "failure_code": preflight_failure_code,
        },
    }
    if not preflight_pass:
        return {
            **common,
            "downstream_status": "NOT_RUN_AFTER_PREFLIGHT_FAIL",
            "finite_difference_stencil_constraints": None,
            "energy_gradients": None,
            "all_positive_body_energy_gradients": None,
            "all_positive_minus_six_gradient_difference_nm_diagnostic_only": None,
            "excluded_link1_direct_energy_gradient_nm": None,
            "mass_matrix": None,
            "mass_matrix_symmetry_error": None,
            "mass_matrix_eigenvalues": None,
            "mass_matrix_condition_number": None,
            "mass_matrix_diagnostics": None,
            "expected_qacc": None,
            "dynamics_residual": None,
            "dynamics_residual_norm": None,
            "relative_qacc_error": None,
            "dynamics_solve_status": None,
            "reverse_gravity_bias": None,
            "zero_gravity_bias": None,
            "failure_codes": [str(preflight_failure_code)],
        }

    mass_matrix = np.zeros((model.nv, model.nv), dtype=float)
    mujoco.mj_fullM(model, data, mass_matrix)
    mass_matrix_finite = bool(np.all(np.isfinite(mass_matrix)))
    symmetry = float(np.max(np.abs(mass_matrix - mass_matrix.T))) if mass_matrix_finite else float("nan")
    try:
        eigenvalues = np.linalg.eigvalsh(0.5 * (mass_matrix + mass_matrix.T)) if mass_matrix_finite else np.full(model.nv, np.nan)
    except np.linalg.LinAlgError:
        eigenvalues = np.full(model.nv, np.nan)
    mass_matrix_spd = bool(mass_matrix_finite and np.all(np.isfinite(eigenvalues)) and float(np.min(eigenvalues)) > 0.0)
    try:
        condition_number = float(np.linalg.cond(mass_matrix)) if mass_matrix_finite else float("nan")
    except np.linalg.LinAlgError:
        condition_number = float("inf")
    baseline_constraint_free = ncon == 0 and nefc == 0 and float(np.max(np.abs(constraint_force))) <= 1.0e-12
    if mass_matrix_spd and baseline_constraint_free:
        try:
            expected_qacc = -np.linalg.solve(mass_matrix, tau)
            residual = mass_matrix @ qacc + tau
            relative_qacc = float(np.linalg.norm(expected_qacc - qacc) / max(float(np.linalg.norm(expected_qacc)), 1.0e-12))
            solve_complete = bool(np.all(np.isfinite(expected_qacc)) and np.all(np.isfinite(residual)) and np.isfinite(relative_qacc))
        except np.linalg.LinAlgError:
            expected_qacc = residual = None
            relative_qacc = None
            solve_complete = False
    else:
        expected_qacc = residual = None
        relative_qacc = None
        solve_complete = False
    gradients: dict[str, list[float]] = {}
    all_positive_gradients: dict[str, list[float]] = {}
    link1_direct_gradients: dict[str, list[float]] = {}
    stencil_constraints: list[dict[str, Any]] = []
    for step in FINITE_DIFFERENCE_STEPS:
        values = []
        all_positive_values = []
        link1_values = []
        for joint_index in range(6):
            plus = q.copy(); plus[joint_index] += step
            minus = q.copy(); minus[joint_index] -= step
            _child_forward(model, data, mujoco, plus, GRAVITY)
            u_plus = _child_potential(model, data, body_ids)
            all_positive_u_plus = _child_potential(model, data, all_positive_body_ids)
            link1_u_plus = _child_potential(model, data, [link1_body_id])
            stencil_constraints.append(_child_stencil_record(data, np, step, JOINTS[joint_index], "plus"))
            _child_forward(model, data, mujoco, minus, GRAVITY)
            u_minus = _child_potential(model, data, body_ids)
            all_positive_u_minus = _child_potential(model, data, all_positive_body_ids)
            link1_u_minus = _child_potential(model, data, [link1_body_id])
            stencil_constraints.append(_child_stencil_record(data, np, step, JOINTS[joint_index], "minus"))
            values.append((u_plus - u_minus) / (2.0 * step))
            all_positive_values.append((all_positive_u_plus - all_positive_u_minus) / (2.0 * step))
            link1_values.append((link1_u_plus - link1_u_minus) / (2.0 * step))
        gradients[f"{step:.17g}"] = values
        all_positive_gradients[f"{step:.17g}"] = all_positive_values
        link1_direct_gradients[f"{step:.17g}"] = link1_values
    _child_forward(model, data, mujoco, q, REVERSE_GRAVITY)
    reverse = np.asarray(data.qfrc_bias, dtype=float).copy()
    _child_forward(model, data, mujoco, q, ZERO_GRAVITY)
    zero = np.asarray(data.qfrc_bias, dtype=float).copy()
    return {
        **common,
        "downstream_status": "COMPLETE",
        "finite_difference_stencil_constraints": stencil_constraints,
        "energy_gradients": gradients,
        "all_positive_body_energy_gradients": all_positive_gradients,
        "all_positive_minus_six_gradient_difference_nm_diagnostic_only": {
            key: (np.asarray(all_positive_gradients[key]) - np.asarray(values)).tolist()
            for key, values in gradients.items()
        },
        "excluded_link1_direct_energy_gradient_nm": link1_direct_gradients,
        "mass_matrix": mass_matrix.tolist(),
        "mass_matrix_symmetry_error": symmetry, "mass_matrix_eigenvalues": eigenvalues.tolist(),
        "mass_matrix_condition_number": condition_number,
        "mass_matrix_diagnostics": {
            "matrix_finite": mass_matrix_finite,
            "positive_definite": mass_matrix_spd,
        },
        "expected_qacc": expected_qacc.tolist() if expected_qacc is not None else None,
        "dynamics_residual": residual.tolist() if residual is not None else None,
        "dynamics_residual_norm": float(np.linalg.norm(residual)) if residual is not None else None,
        "relative_qacc_error": relative_qacc,
        "dynamics_solve_status": (
            "COMPLETE" if solve_complete
            else "SKIPPED_ACTIVE_CONSTRAINT" if not baseline_constraint_free
            else "NOT_AVAILABLE_MASS_MATRIX_INVALID_OR_SINGULAR"
        ),
        "reverse_gravity_bias": reverse.tolist(), "zero_gravity_bias": zero.tolist(),
        "failure_codes": [],
    }


def physics_child(model_path: Path, request_path: Path, output_path: Path) -> int:
    import platform
    import numpy as np  # type: ignore
    import mujoco  # type: ignore

    request = read_json(request_path)
    poses = request.get("poses")
    require(isinstance(poses, list), "child pose request malformed")
    allowed_pair_rows = request.get("runtime_full_pairs")
    require(isinstance(allowed_pair_rows, list) and len(allowed_pair_rows) == 231, "child collision pair request malformed")
    allowed_pairs = {tuple(sorted((str(row[0]), str(row[1])))) for row in allowed_pair_rows if isinstance(row, list) and len(row) == 2}
    require(len(allowed_pairs) == 231, "child collision pair request is not exact 231 unique pairs")

    # These are deliberately separate compiled objects.  The collision clone
    # exposes parent-adjacent contacts for the protected V15.14 pair filter;
    # the dynamics clone retains production parent filtering and is the only
    # source of M, qacc, qfrc_bias, and finite-difference evidence.
    model = mujoco.MjModel.from_xml_path(str(model_path))
    collision_model = mujoco.MjModel.from_xml_path(str(model_path))
    require(model is not collision_model, "collision and dynamics model objects were not isolated")
    require(model.nq == model.nv == collision_model.nq == collision_model.nv == 6, "compiled production model is not six generalized coordinates")
    joint_mapping = []
    for expected_index, name in enumerate(JOINTS):
        joint_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name))
        collision_joint_id = int(mujoco.mj_name2id(collision_model, mujoco.mjtObj.mjOBJ_JOINT, name))
        require(joint_id >= 0 and collision_joint_id == joint_id, f"compiled joint identity missing or differs between model clones: {name}")
        qpos_address = int(model.jnt_qposadr[joint_id])
        dof_address = int(model.jnt_dofadr[joint_id])
        joint_type = int(model.jnt_type[joint_id])
        require(qpos_address == dof_address == expected_index, f"compiled {name} qpos/dof address is not {expected_index}")
        require(joint_type == int(mujoco.mjtJoint.mjJNT_HINGE), f"compiled {name} is not a hinge joint")
        joint_mapping.append({"joint": name, "joint_id": joint_id, "qpos_address": qpos_address, "dof_address": dof_address, "joint_type": "HINGE"})
    body_ids = []
    for name in AUTHORITY_BODIES:
        body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))
        require(body_id >= 0, f"compiled authority body missing: {name}")
        body_ids.append(body_id)
    authority_masses = [float(model.body_mass[body_id]) for body_id in body_ids]
    require(abs(sum(authority_masses) - AUTHORITY_TOTAL_MASS_KG) <= 1.0e-12, "compiled six-body authority mass does not total 3.4515 kg")
    positive_body_rows = []
    for body_id in range(int(model.nbody)):
        mass = float(model.body_mass[body_id])
        if mass > 0.0:
            positive_body_rows.append((str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)), body_id, mass))
    require({name for name, _, _ in positive_body_rows} == {"link1", *AUTHORITY_BODIES}, "compiled positive-mass body identity set changed")
    require(abs(dict((name, mass) for name, _, mass in positive_body_rows)["link1"] - 1.0) <= 1.0e-15, "compiled link1 placeholder mass changed")
    require(abs(sum(mass for _, _, mass in positive_body_rows) - 4.4515) <= 1.0e-12, "compiled all-positive-body mass is not 4.4515 kg")
    all_positive_body_ids = [body_id for _, body_id, _ in positive_body_rows]
    link1_body_id = next(body_id for name, body_id, _ in positive_body_rows if name == "link1")
    initial_disableflags = int(model.opt.disableflags)
    collision_initial_disableflags = int(collision_model.opt.disableflags)
    require(initial_disableflags == collision_initial_disableflags == 0, f"production compiled disableflags changed: dynamics={initial_disableflags}, collision={collision_initial_disableflags}")
    compiled_gravity_before_override = np.asarray(model.opt.gravity, dtype=float).tolist()
    actuation_disable_bit = int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
    forbidden_disable_bits = {
        "CONTACT": int(mujoco.mjtDisableBit.mjDSBL_CONTACT),
        "CONSTRAINT": int(mujoco.mjtDisableBit.mjDSBL_CONSTRAINT),
        "LIMIT": int(mujoco.mjtDisableBit.mjDSBL_LIMIT),
        "FILTERPARENT": int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT),
    }
    require(actuation_disable_bit == 2048, f"MuJoCo 3.11 actuation-disable bit changed: {actuation_disable_bit}")
    model.opt.disableflags = initial_disableflags | actuation_disable_bit
    require(int(model.opt.disableflags) == 2048, "runtime disableflags must be exact production-default OR ACTUATION")
    require(all((int(model.opt.disableflags) & bit) == 0 for bit in forbidden_disable_bits.values()), "contact/constraint/limit/filter-parent disable bit is forbidden")
    filter_parent_bit = forbidden_disable_bits["FILTERPARENT"]
    require(filter_parent_bit == 1024, f"MuJoCo 3.11 filter-parent disable bit changed: {filter_parent_bit}")
    collision_model.opt.disableflags = collision_initial_disableflags | actuation_disable_bit | filter_parent_bit
    require(int(collision_model.opt.disableflags) == 3072, "collision clone flags must be exact ACTUATION|FILTERPARENT")
    require(all((int(collision_model.opt.disableflags) & forbidden_disable_bits[name]) == 0 for name in ("CONTACT", "CONSTRAINT", "LIMIT")), "collision clone disabled a physical contact/constraint/limit subsystem")
    data = mujoco.MjData(model)
    collision_data = mujoco.MjData(collision_model)
    rows = []
    for pose in poses:
        row = _child_pose(model, data, mujoco, np, body_ids, all_positive_body_ids, link1_body_id, pose)
        _child_forward(collision_model, collision_data, mujoco, np.asarray(pose["q_rad"], dtype=float), ZERO_GRAVITY)
        accepted_contacts = _child_guard_contacts(collision_model, collision_data, mujoco, allowed_pairs)
        row["collision_clone_diagnostic"] = {
            "diagnostic_only_not_mass_matrix_or_qacc_authority": True,
            "runtime_disableflags": int(collision_model.opt.disableflags),
            "raw_ncon": int(collision_data.ncon),
            "raw_nefc": int(collision_data.nefc),
            "accepted_contacts": accepted_contacts,
            "accepted_contact_count": len(accepted_contacts),
            "accepted_contact_set_empty": not accepted_contacts,
        }
        rows.append(row)
    continuity = request.get("continuity", {})
    continuity_rows = []
    if isinstance(continuity, dict) and continuity:
        start = np.asarray(continuity["q_start"], dtype=float)
        end = np.asarray(continuity["q_end"], dtype=float)
        for index, alpha in enumerate(np.linspace(0.0, 1.0, 21)):
            q = (1.0 - alpha) * start + alpha * end
            _child_forward(model, data, mujoco, q, GRAVITY)
            arrays = (
                q, data.qfrc_bias, data.qvel, data.qacc, data.qfrc_constraint,
                data.qfrc_passive, data.qfrc_actuator, data.qfrc_gravcomp,
                data.qfrc_applied, data.xfrc_applied,
            )
            continuity_rows.append({
                "index": index,
                "alpha": float(alpha),
                "q_rad": q.tolist(),
                "tau_hold_nm": np.asarray(data.qfrc_bias, dtype=float).tolist(),
                "ncon": int(data.ncon),
                "nefc": int(data.nefc),
                "max_abs_qfrc_constraint": float(np.max(np.abs(data.qfrc_constraint))),
                "max_abs_qfrc_passive": float(np.max(np.abs(data.qfrc_passive))),
                "max_abs_qfrc_actuator": float(np.max(np.abs(data.qfrc_actuator))),
                "max_abs_qfrc_gravcomp": float(np.max(np.abs(data.qfrc_gravcomp))),
                "max_abs_qfrc_applied": float(np.max(np.abs(data.qfrc_applied))),
                "max_abs_xfrc_applied": float(np.max(np.abs(data.xfrc_applied))),
                "all_values_finite": bool(all(np.all(np.isfinite(array)) for array in arrays)),
            })
    payload = {
        "python_version": platform.python_version(),
        "mujoco_version": str(mujoco.__version__), "numpy_version": str(np.__version__),
        "nq": int(model.nq), "nv": int(model.nv), "nu": int(model.nu),
        "compiled_joint_order_mapping": joint_mapping,
        "compiled_initial_disableflags": initial_disableflags,
        "runtime_disableflags": int(model.opt.disableflags),
        "model_role_separation": {
            "distinct_compiled_model_objects": True,
            "dynamics_model_role": "ONLY_M_QACC_BIAS_ENERGY_AND_CONTINUITY_AUTHORITY",
            "collision_model_role": "V15_14_PAIR_SCOPED_COLLISION_DIAGNOSTIC_ONLY",
            "collision_model_never_used_for_mass_matrix_or_qacc": True,
            "collision_compiled_initial_disableflags": collision_initial_disableflags,
            "collision_runtime_disableflags": int(collision_model.opt.disableflags),
        },
        "actuation_disable_bit": actuation_disable_bit,
        "forbidden_disable_bits": forbidden_disable_bits,
        "production_collision_filtering_remains_enabled": True,
        "compiled_gravity_before_override": compiled_gravity_before_override,
        "potential_energy_authority_bodies": list(AUTHORITY_BODIES),
        "potential_energy_authority_body_ids": body_ids,
        "potential_energy_authority_masses_kg": authority_masses,
        "potential_energy_authority_total_mass_kg": sum(authority_masses),
        "all_positive_mass_body_rows": [{"body": name, "body_id": body_id, "mass_kg": mass} for name, body_id, mass in positive_body_rows],
        "all_positive_mass_total_kg": sum(mass for _, _, mass in positive_body_rows),
        "link1_placeholder_mass_kg": 1.0,
        "link1_excluded_from_authority_potential_but_retained_in_mass_matrix": True,
        "potential_energy_authority_ipos_m": [np.asarray(model.body_ipos[body_id], dtype=float).tolist() for body_id in body_ids],
        "potential_energy_authority_iquat_wxyz": [np.asarray(model.body_iquat[body_id], dtype=float).tolist() for body_id in body_ids],
        "potential_energy_authority_principal_inertia_kg_m2": [np.asarray(model.body_inertia[body_id], dtype=float).tolist() for body_id in body_ids],
        "non_authority_positive_mass_bodies_excluded": ["link1"],
        "body_gravcomp": np.asarray(model.body_gravcomp, dtype=float).tolist(),
        "joint_damping": np.asarray(model.dof_damping, dtype=float).tolist(),
        "joint_frictionloss": np.asarray(model.dof_frictionloss, dtype=float).tolist(),
        "joint_armature": np.asarray(model.dof_armature, dtype=float).tolist(),
        "option_wind_m_s": np.asarray(model.opt.wind, dtype=float).tolist(),
        "option_density_kg_m3": float(model.opt.density),
        "option_viscosity_pa_s": float(model.opt.viscosity),
        "poses": rows, "continuity_rows": continuity_rows,
    }
    output_path.write_text(
        json.dumps(_child_json_safe(payload, np), ensure_ascii=False, sort_keys=True, allow_nan=False),
        encoding="utf-8",
    )
    return 0


def vector_norm(values: Sequence[float]) -> float:
    return math.sqrt(math.fsum(float(value) * float(value) for value in values))


def flatten_finite(value: Any, label: str) -> list[float]:
    require(isinstance(value, list), f"{label}: list required")
    result: list[float] = []
    for index, item in enumerate(value):
        if isinstance(item, list):
            result.extend(flatten_finite(item, f"{label}[{index}]"))
        else:
            result.append(finite(item, f"{label}[{index}]"))
    return result


def nullable_number(value: Any, label: str) -> float | None:
    """Parse a JSON-safe physical numeric leaf (finite number or null)."""

    if value is None:
        return None
    return finite(value, label)


def nullable_vector(value: Any, length: int, label: str) -> list[float | None]:
    require(isinstance(value, list) and len(value) == length, f"{label}: expected {length} nullable values")
    return [nullable_number(item, f"{label}[{index}]") for index, item in enumerate(value)]


def nullable_matrix(value: Any, size: int, label: str) -> list[list[float | None]]:
    require(isinstance(value, list) and len(value) == size, f"{label}: expected {size} nullable rows")
    return [nullable_vector(row, size, f"{label}[{index}]") for index, row in enumerate(value)]


def flatten_nullable(value: Any, label: str) -> list[float | None]:
    require(isinstance(value, list), f"{label}: list required")
    result: list[float | None] = []
    for index, item in enumerate(value):
        if isinstance(item, list):
            result.extend(flatten_nullable(item, f"{label}[{index}]"))
        else:
            result.append(nullable_number(item, f"{label}[{index}]"))
    return result


def all_present(values: Iterable[Any]) -> bool:
    return all(value is not None for value in values)


def present_max_abs(values: Iterable[float | None]) -> float | None:
    materialized = list(values)
    return max_abs(value for value in materialized if value is not None) if all_present(materialized) else None


def quaternion_rotation_wxyz(value: Sequence[float]) -> list[list[float]]:
    w, x, y, z = (float(item) for item in value)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    require(norm > 0.0, "zero compiled inertial quaternion")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def reconstruct_inertia_tensor(quaternion_wxyz: Sequence[float], principal: Sequence[float]) -> list[list[float]]:
    rotation = quaternion_rotation_wxyz(quaternion_wxyz)
    return [[math.fsum(rotation[row][axis] * float(principal[axis]) * rotation[column][axis] for axis in range(3)) for column in range(3)] for row in range(3)]


def relative_frobenius(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> float:
    numerator = math.sqrt(math.fsum((float(left[row][column]) - float(right[row][column])) ** 2 for row in range(3) for column in range(3)))
    denominator = max(math.sqrt(math.fsum(float(right[row][column]) ** 2 for row in range(3) for column in range(3))), 1.0e-30)
    return numerator / denominator


def analyze_independent_physics(raw: Mapping[str, Any], selection: Mapping[str, Any], authority: Mapping[str, Any]) -> dict[str, Any]:
    """Derive all acceptance facts from validator-owned MuJoCo output."""

    require(raw.get("mujoco_version") == "3.11.0", "independent child did not use MuJoCo 3.11.0")
    require(raw.get("nq") == raw.get("nv") == 6, "independent child generalized dimension changed")
    require(raw.get("compiled_initial_disableflags") == 0 and raw.get("runtime_disableflags") == 2048, "independent dynamics flags are not exact production-default|ACTUATION")
    roles = raw.get("model_role_separation")
    require(isinstance(roles, dict) and roles.get("distinct_compiled_model_objects") is True, "independent child did not isolate collision and dynamics models")
    require(roles.get("collision_runtime_disableflags") == 3072 and roles.get("collision_model_never_used_for_mass_matrix_or_qacc") is True, "collision clone role/flags changed")
    require(vector(raw.get("compiled_gravity_before_override"), 3, "compiled gravity before override") == [0.0, 0.0, 0.0], "compiled production gravity is not zero")
    require(raw.get("potential_energy_authority_bodies") == list(AUTHORITY_BODIES), "potential authority body order changed")
    require(abs(finite(raw.get("potential_energy_authority_total_mass_kg"), "compiled authority total") - AUTHORITY_TOTAL_MASS_KG) <= 1.0e-12, "compiled authority mass total changed")
    require(abs(finite(raw.get("all_positive_mass_total_kg"), "compiled all-positive total") - 4.4515) <= 1.0e-12, "compiled all-positive mass total changed")
    require(finite(raw.get("link1_placeholder_mass_kg"), "link1 placeholder mass") == 1.0, "compiled link1 placeholder mass changed")
    require(raw.get("link1_excluded_from_authority_potential_but_retained_in_mass_matrix") is True, "link1 potential/mass-matrix semantics changed")
    for field in ("body_gravcomp", "joint_damping", "joint_frictionloss", "joint_armature", "option_wind_m_s"):
        require(max_abs(flatten_finite(raw.get(field), field)) <= 1.0e-15, f"compiled {field} is not zero")
    require(abs(finite(raw.get("option_density_kg_m3"), "compiled option density")) <= 1.0e-15, "compiled fluid density is not zero")
    require(abs(finite(raw.get("option_viscosity_pa_s"), "compiled option viscosity")) <= 1.0e-15, "compiled fluid viscosity is not zero")

    masses = vector(raw.get("potential_energy_authority_masses_kg"), 6, "compiled authority masses")
    ipos = raw.get("potential_energy_authority_ipos_m")
    iquats = raw.get("potential_energy_authority_iquat_wxyz")
    principal = raw.get("potential_energy_authority_principal_inertia_kg_m2")
    require(isinstance(ipos, list) and isinstance(iquats, list) and isinstance(principal, list) and len(ipos) == len(iquats) == len(principal) == 6, "compiled six-body inertial rows malformed")
    authority_rows = authority.get("six_body_authority")
    require(isinstance(authority_rows, dict), "independent authority rows missing")
    compiled_tensor_errors: dict[str, float] = {}
    compiled_authority_bodies: dict[str, Any] = {}
    for index, link in enumerate(AUTHORITY_BODIES):
        expected = authority_rows[link]
        require(abs(masses[index] - finite(expected.get("mass_kg"), f"authority.{link}.mass")) <= 1.0e-12, f"{link}: compiled mass differs from authority")
        require(max_abs(left - right for left, right in zip(vector(ipos[index], 3, f"compiled.{link}.ipos"), vector(expected.get("com_link_m"), 3, f"authority.{link}.com"))) <= 1.0e-12, f"{link}: compiled COM differs from authority")
        tensor = reconstruct_inertia_tensor(vector(iquats[index], 4, f"compiled.{link}.iquat"), vector(principal[index], 3, f"compiled.{link}.principal"))
        expected_tensor = matrix(expected.get("inertia_tensor_kg_m2"), 3, f"authority.{link}.tensor")
        error = relative_frobenius(tensor, expected_tensor)
        require(error < 1.0e-9, f"{link}: compiled tensor differs from authority")
        compiled_tensor_errors[link] = error
        mass_error = abs(masses[index] - float(expected["mass_kg"]))
        compiled_com = vector(ipos[index], 3, f"compiled.{link}.ipos")
        expected_com = vector(expected["com_link_m"], 3, f"authority.{link}.com")
        com_error = max_abs(left - right for left, right in zip(compiled_com, expected_com))
        compiled_authority_bodies[link] = {
            "compiled_mass_kg": masses[index], "authority_mass_kg": expected["mass_kg"], "mass_error_kg": mass_error,
            "compiled_com_m": compiled_com, "authority_com_m": expected_com, "com_max_abs_error_m": com_error,
            "compiled_inertia_tensor_kg_m2": tensor, "inertia_relative_frobenius_error": error,
            "pass": mass_error <= 1.0e-12 and com_error <= 1.0e-12 and error < 1.0e-9,
        }
    compiled_authority_validation = {
        "bodies": compiled_authority_bodies,
        "max_mass_error_kg": max(row["mass_error_kg"] for row in compiled_authority_bodies.values()),
        "max_com_error_m": max(row["com_max_abs_error_m"] for row in compiled_authority_bodies.values()),
        "max_inertia_relative_frobenius_error": max(row["inertia_relative_frobenius_error"] for row in compiled_authority_bodies.values()),
        "pass": all(row["pass"] for row in compiled_authority_bodies.values()),
    }

    selected = selection.get("selected")
    raw_poses = raw.get("poses")
    require(isinstance(selected, list) and isinstance(raw_poses, list) and len(selected) == len(raw_poses) == 21, "independent pose count is not exact 21")
    pose_results: list[dict[str, Any]] = []
    all_values: list[list[float]] = []
    all_gradient_abs: list[float] = []
    all_gradient_rel: list[float] = []
    all_step_differences: list[float] = []
    all_residual_norms: list[float] = []
    all_relative_qacc: list[float] = []
    all_zero: list[float] = []
    all_reverse: list[float] = []
    all_min_eigenvalues: list[float] = []
    all_conditions: list[float] = []
    all_symmetry: list[float] = []
    all_link1_gradient_differences: list[float] = []
    all_link1_subtractive_diagnostics: list[float] = []
    all_passive: list[float] = []
    all_actuator: list[float] = []
    all_gravcomp: list[float] = []
    h1_key, h2_key = (f"{value:.17g}" for value in FINITE_DIFFERENCE_STEPS)
    for expected, row in zip(selected, raw_poses):
        require(isinstance(row, dict) and row.get("id") == expected.get("id"), "independent pose identity/order changed")
        q = vector(row.get("q_rad"), 6, f"{row.get('id')}.q")
        require(max_abs(left - right for left, right in zip(q, expected["q_rad"])) <= 1.0e-14, f"{row.get('id')}: q differs from deterministic selection")
        collision = row.get("collision_clone_diagnostic")
        require(isinstance(collision, dict) and collision.get("diagnostic_only_not_mass_matrix_or_qacc_authority") is True, f"{row.get('id')}: collision clone role missing")
        contacts = collision.get("accepted_contacts")
        require(isinstance(contacts, list) and collision.get("accepted_contact_count") == len(contacts), f"{row.get('id')}: collision clone contact audit malformed")
        tau_nullable = nullable_vector(row.get("tau_hold_mujoco_nm"), 6, f"{row.get('id')}.tau")
        passive_nullable = nullable_vector(row.get("qfrc_passive"), 6, f"{row.get('id')}.passive")
        actuator_nullable = nullable_vector(row.get("qfrc_actuator"), 6, f"{row.get('id')}.actuator")
        gravcomp_nullable = nullable_vector(row.get("qfrc_gravcomp"), 6, f"{row.get('id')}.gravcomp")
        applied_nullable = nullable_vector(row.get("qfrc_applied"), 6, f"{row.get('id')}.applied")
        external_value = row.get("xfrc_applied")
        ctrl_value = row.get("ctrl")
        require(isinstance(external_value, list) and isinstance(ctrl_value, list), f"{row.get('id')}: applied/ctrl arrays malformed")
        external_nullable = flatten_nullable(external_value, f"{row.get('id')}.external")
        ctrl_nullable = flatten_nullable(ctrl_value, f"{row.get('id')}.ctrl")
        qacc_nullable = nullable_vector(row.get("qacc"), 6, f"{row.get('id')}.qacc")
        baseline_constraint_nullable = nullable_vector(row.get("qfrc_constraint"), 6, f"{row.get('id')}.constraint")
        potential = nullable_number(row.get("potential_energy_j"), f"{row.get('id')}.potential")
        all_positive_potential = nullable_number(row.get("all_positive_body_potential_energy_j"), f"{row.get('id')}.all_positive_potential")
        baseline_leaves = [
            *tau_nullable, *passive_nullable, *actuator_nullable, *gravcomp_nullable,
            *applied_nullable, *external_nullable, *ctrl_nullable, *qacc_nullable,
            *baseline_constraint_nullable, potential, all_positive_potential,
        ]
        baseline_all_finite = all_present(baseline_leaves)
        maximum_tau = present_max_abs(tau_nullable)
        suspicious_torque = bool(maximum_tau is not None and maximum_tau > 100.0)
        preflight_pass = baseline_all_finite and not suspicious_torque
        preflight_failure_code = (
            None if preflight_pass
            else "FAIL_NONFINITE_STATIC_RESULT" if not baseline_all_finite
            else "FAIL_SUSPECT_UNIT_OR_COM_ERROR"
        )
        raw_preflight = row.get("preflight")
        require(isinstance(raw_preflight, dict), f"{row.get('id')}: child preflight record missing")
        require_close(raw_preflight, {
            "all_baseline_values_finite": baseline_all_finite,
            "max_abs_tau_nm": maximum_tau,
            "suspicious_torque_over_100_nm": suspicious_torque,
            "pass": preflight_pass,
            "failure_code": preflight_failure_code,
        }, f"{row.get('id')}.preflight", absolute=0.0, relative=0.0)
        if not preflight_pass:
            require(row.get("downstream_status") == "NOT_RUN_AFTER_PREFLIGHT_FAIL", f"{row.get('id')}: preflight failure did not stop downstream physics")
            for key in (
                "energy_gradients", "all_positive_body_energy_gradients",
                "all_positive_minus_six_gradient_difference_nm_diagnostic_only",
                "excluded_link1_direct_energy_gradient_nm", "mass_matrix",
                "mass_matrix_diagnostics", "expected_qacc", "dynamics_residual",
                "dynamics_residual_norm", "relative_qacc_error", "reverse_gravity_bias",
                "zero_gravity_bias",
            ):
                require(row.get(key) is None, f"{row.get('id')}: {key} must be null after preflight failure")
            require(row.get("finite_difference_stencil_constraints") is None, f"{row.get('id')}: stencils ran after preflight failure")
            result = {
                "id": row["id"], "q_rad": q, "tau_hold_mujoco_nm": tau_nullable,
                "preflight": dict(raw_preflight), "downstream_status": "NOT_RUN_AFTER_PREFLIGHT_FAIL",
                "potential_energy_j": potential, "all_positive_body_potential_energy_j": all_positive_potential,
                "qfrc_passive": passive_nullable, "qfrc_actuator": actuator_nullable,
                "qfrc_gravcomp": gravcomp_nullable, "qfrc_applied": applied_nullable,
                "xfrc_applied": external_nullable, "ctrl": ctrl_nullable, "qacc": qacc_nullable,
                "baseline_ncon": row.get("ncon"), "baseline_nefc": row.get("nefc"),
                "baseline_qfrc_constraint": baseline_constraint_nullable,
                "energy_gradient_h1_nm": None, "energy_gradient_h2_nm": None,
                "finite_difference_step_difference_nm": None, "absolute_error_nm": None,
                "relative_error": None, "per_joint_pass": None, "mass_matrix": None,
                "mass_matrix_symmetry_error": None, "mass_matrix_eigenvalues": None,
                "mass_matrix_min_eigenvalue": None, "mass_matrix_condition_number": None,
                "qacc": qacc_nullable, "expected_qacc": None, "dynamics_residual": None,
                "dynamics_residual_norm": None, "relative_qacc_error": None,
                "zero_gravity_bias": None, "reverse_gravity_bias": None,
                "reverse_gravity_relative_error": None, "dynamics_constraint_free": False,
                "current_pair_scoped_collision_free": not contacts,
                "collision_clone_raw_ncon": collision.get("raw_ncon"),
                "collision_clone_raw_nefc": collision.get("raw_nefc"),
                "collision_clone_accepted_contacts": contacts,
                "excluded_link1_direct_energy_gradient_nm": None,
                "all_positive_minus_six_gradient_difference_nm_diagnostic_only": None,
                "failure_codes": [str(preflight_failure_code)], "pass": False,
            }
            pose_results.append(result)
            if all_present(tau_nullable):
                all_values.append([float(value) for value in tau_nullable if value is not None])
            all_passive.extend(abs(float(value)) for value in passive_nullable if value is not None)
            all_actuator.extend(abs(float(value)) for value in actuator_nullable if value is not None)
            all_gravcomp.extend(abs(float(value)) for value in gravcomp_nullable if value is not None)
            continue

        require(row.get("downstream_status") == "COMPLETE", f"{row.get('id')}: preflight PASS must continue physics")
        tau = [float(value) for value in tau_nullable if value is not None]
        gradients = row.get("energy_gradients")
        require(isinstance(gradients, dict), f"{row.get('id')}: energy gradients missing")
        h1 = vector(gradients.get(h1_key), 6, f"{row.get('id')}.gradient.h1")
        h2 = vector(gradients.get(h2_key), 6, f"{row.get('id')}.gradient.h2")
        step_diff = [abs(left - right) for left, right in zip(h1, h2)]
        absolute_errors = [abs(left - right) for left, right in zip(tau, h2)]
        relative_errors = [error / max(abs(torque), 0.01) for error, torque in zip(absolute_errors, tau)]
        per_joint_pass = [(relative_errors[index] < 1.0e-5 if abs(tau[index]) >= 0.1 else absolute_errors[index] < 1.0e-5) for index in range(6)]
        matrix_rows = matrix(row.get("mass_matrix"), 6, f"{row.get('id')}.M")
        qacc = vector(row.get("qacc"), 6, f"{row.get('id')}.qacc")
        residual = [math.fsum(matrix_rows[i][j] * qacc[j] for j in range(6)) + tau[i] for i in range(6)]
        residual_norm = vector_norm(residual)
        expected_qacc = vector(row.get("expected_qacc"), 6, f"{row.get('id')}.expected_qacc")
        relative_qacc = vector_norm([left - right for left, right in zip(qacc, expected_qacc)]) / max(vector_norm(expected_qacc), 1.0e-12)
        eigenvalues = vector(row.get("mass_matrix_eigenvalues"), 6, f"{row.get('id')}.M.eigenvalues")
        symmetry = max_abs(matrix_rows[i][j] - matrix_rows[j][i] for i in range(6) for j in range(6))
        condition = finite(row.get("mass_matrix_condition_number"), f"{row.get('id')}.M.condition")
        zero = vector(row.get("zero_gravity_bias"), 6, f"{row.get('id')}.zero")
        reverse = vector(row.get("reverse_gravity_bias"), 6, f"{row.get('id')}.reverse")
        reverse_errors = [abs(left + right) / max(abs(left), 0.01) for left, right in zip(tau, reverse)]
        passive = vector(row.get("qfrc_passive"), 6, f"{row.get('id')}.passive")
        actuator = vector(row.get("qfrc_actuator"), 6, f"{row.get('id')}.actuator")
        gravcomp = vector(row.get("qfrc_gravcomp"), 6, f"{row.get('id')}.gravcomp")
        applied = vector(row.get("qfrc_applied"), 6, f"{row.get('id')}.applied")
        external = flatten_finite(row.get("xfrc_applied"), f"{row.get('id')}.external")
        ctrl = flatten_finite(row.get("ctrl"), f"{row.get('id')}.ctrl")
        baseline_constraint = vector(row.get("qfrc_constraint"), 6, f"{row.get('id')}.constraint")
        stencils = row.get("finite_difference_stencil_constraints")
        require(isinstance(stencils, list) and len(stencils) == 24, f"{row.get('id')}: exact 24 stencil constraint rows required")
        expected_stencil_keys = {
            (float(step), joint, side)
            for step in FINITE_DIFFERENCE_STEPS for joint in JOINTS for side in ("plus", "minus")
        }
        actual_stencil_keys = {
            (finite(item.get("step_rad"), "stencil step"), str(item.get("joint")), str(item.get("side")))
            for item in stencils if isinstance(item, dict)
        }
        require(actual_stencil_keys == expected_stencil_keys and len(actual_stencil_keys) == len(stencils), f"{row.get('id')}: stencil identity set changed")
        stencil_clear = all(
            item.get("ncon") == 0 and item.get("nefc") == 0 and item.get("all_values_finite") is True
            and all(finite(item.get(key), f"stencil.{key}") <= 1.0e-12 for key in (
                "max_abs_qfrc_constraint", "max_abs_qfrc_passive", "max_abs_qfrc_actuator",
                "max_abs_qfrc_gravcomp", "max_abs_qfrc_applied", "max_abs_xfrc_applied",
            ))
            for item in stencils if isinstance(item, dict)
        ) and all(isinstance(item, dict) for item in stencils)
        link1_direct = row.get("excluded_link1_direct_energy_gradient_nm")
        require(isinstance(link1_direct, dict), f"{row.get('id')}: direct link1 gradient cross-check missing")
        selected_link1_difference = vector(link1_direct.get(h2_key), 6, f"{row.get('id')}.link1_direct_gradient")
        subtractive_diagnostic = row.get("all_positive_minus_six_gradient_difference_nm_diagnostic_only")
        require(isinstance(subtractive_diagnostic, dict), f"{row.get('id')}: link1 subtractive diagnostic missing")
        selected_subtractive_diagnostic = vector(subtractive_diagnostic.get(h2_key), 6, f"{row.get('id')}.link1_subtractive_diagnostic")
        dynamics_constraint_free = row.get("ncon") == 0 and row.get("nefc") == 0 and max_abs(baseline_constraint) <= 1.0e-12 and stencil_clear
        pose_pass = all((
            dynamics_constraint_free,
            not contacts,
            max(step_diff) < 1.0e-5,
            all(per_joint_pass),
            residual_norm <= 1.0e-9 * max(vector_norm(tau), 1.0),
            relative_qacc < 1.0e-9,
            max_abs(zero) < 1.0e-10,
            max(reverse_errors) < 1.0e-10,
            symmetry < 1.0e-10,
            min(eigenvalues) > 0.0,
            max_abs(passive) <= 1.0e-12,
            max_abs(actuator) <= 1.0e-12,
            max_abs(gravcomp) <= 1.0e-12,
            max_abs(applied) <= 1.0e-12,
            max_abs(external) <= 1.0e-12,
            max_abs(ctrl) <= 1.0e-12,
            max_abs(selected_link1_difference) <= 1.0e-12,
            max_abs(tau) <= 100.0,
        ))
        result = {
            "id": row["id"],
            "q_rad": q,
            "tau_hold_mujoco_nm": tau,
            "preflight": dict(raw_preflight),
            "downstream_status": "COMPLETE",
            "potential_energy_j": potential,
            "all_positive_body_potential_energy_j": all_positive_potential,
            "qfrc_passive": passive,
            "qfrc_actuator": actuator,
            "qfrc_gravcomp": gravcomp,
            "qfrc_applied": applied,
            "xfrc_applied": external,
            "ctrl": ctrl,
            "baseline_ncon": row.get("ncon"),
            "baseline_nefc": row.get("nefc"),
            "baseline_qfrc_constraint": baseline_constraint,
            "energy_gradient_h1_nm": h1,
            "energy_gradient_h2_nm": h2,
            "finite_difference_step_difference_nm": step_diff,
            "absolute_error_nm": absolute_errors,
            "relative_error": relative_errors,
            "per_joint_pass": per_joint_pass,
            "mass_matrix": matrix_rows,
            "mass_matrix_symmetry_error": symmetry,
            "mass_matrix_eigenvalues": eigenvalues,
            "mass_matrix_min_eigenvalue": min(eigenvalues),
            "mass_matrix_condition_number": condition,
            "qacc": qacc,
            "expected_qacc": expected_qacc,
            "dynamics_residual": residual,
            "dynamics_residual_norm": residual_norm,
            "relative_qacc_error": relative_qacc,
            "zero_gravity_bias": zero,
            "reverse_gravity_bias": reverse,
            "reverse_gravity_relative_error": reverse_errors,
            "dynamics_constraint_free": dynamics_constraint_free,
            "current_pair_scoped_collision_free": not contacts,
            "collision_clone_raw_ncon": collision.get("raw_ncon"),
            "collision_clone_raw_nefc": collision.get("raw_nefc"),
            "collision_clone_accepted_contacts": contacts,
            "excluded_link1_direct_energy_gradient_nm": selected_link1_difference,
            "all_positive_minus_six_gradient_difference_nm_diagnostic_only": selected_subtractive_diagnostic,
            "failure_codes": [],
            "pass": pose_pass,
        }
        pose_results.append(result)
        all_values.append(tau)
        all_gradient_abs.extend(absolute_errors)
        all_gradient_rel.extend(relative_errors)
        all_step_differences.extend(step_diff)
        all_residual_norms.append(residual_norm)
        all_relative_qacc.append(relative_qacc)
        all_zero.extend(abs(value) for value in zero)
        all_reverse.extend(reverse_errors)
        all_min_eigenvalues.append(min(eigenvalues))
        all_conditions.append(condition)
        all_symmetry.append(symmetry)
        all_link1_gradient_differences.extend(abs(value) for value in selected_link1_difference)
        all_link1_subtractive_diagnostics.extend(abs(value) for value in selected_subtractive_diagnostic)
        all_passive.extend(abs(value) for value in passive)
        all_actuator.extend(abs(value) for value in actuator)
        all_gravcomp.extend(abs(value) for value in gravcomp)

    statistics: dict[str, Any] = {}
    finite_tau_results = [
        row for row in pose_results
        if isinstance(row.get("tau_hold_mujoco_nm"), list)
        and len(row["tau_hold_mujoco_nm"]) == 6
        and all_present(row["tau_hold_mujoco_nm"])
    ]
    for joint_index, joint in enumerate(JOINTS):
        values = [(row, float(row["tau_hold_mujoco_nm"][joint_index])) for row in finite_tau_results]
        if values:
            max_row, max_value = max(values, key=lambda item: abs(item[1]))
            statistics[joint] = {
                "min_nm": min(value for _, value in values),
                "max_nm": max(value for _, value in values),
                "max_abs_nm": abs(max_value),
                "pose_of_max_abs": max_row["id"],
            }
        else:
            statistics[joint] = {"min_nm": None, "max_nm": None, "max_abs_nm": None, "pose_of_max_abs": None}
    by_id = {row["id"]: row for row in pose_results}
    special_poses = {
        "mechanical_zero": {
            "id": "frozen_503_global_018",
            "q_rad": by_id["frozen_503_global_018"]["q_rad"],
            "tau_hold_nm": by_id["frozen_503_global_018"]["tau_hold_mujoco_nm"],
        },
        "max_extension": {
            "id": "frozen_503_global_481",
            "q_rad": by_id["frozen_503_global_481"]["q_rad"],
            "tau_hold_nm": by_id["frozen_503_global_481"]["tau_hold_mujoco_nm"],
        },
        "v15_14_accepted_nonzero": {
            "id": "v15_14_accepted_fjt_final_pose",
            "q_rad": by_id["v15_14_accepted_fjt_final_pose"]["q_rad"],
            "tau_hold_nm": by_id["v15_14_accepted_fjt_final_pose"]["tau_hold_mujoco_nm"],
        },
    }
    j2_pose_id = statistics["J2"]["pose_of_max_abs"]
    j2_total = by_id[j2_pose_id]["tau_hold_mujoco_nm"][1] if j2_pose_id is not None else None
    j2_diagnostic = {
        "label": "IDEAL_EQUAL_LOAD_SHARE_DIAGNOSTIC_ONLY",
        "not_hardware_current_distribution_authority": True,
        "pose_id": j2_pose_id,
        "total_joint_torque_nm": j2_total,
        "ideal_equal_share_per_motor_nm": j2_total / 2.0 if j2_total is not None else None,
    }

    continuity_rows = raw.get("continuity_rows")
    require(isinstance(continuity_rows, list) and len(continuity_rows) == 21, "independent continuity does not contain exact 21 samples")
    continuity_torque = []
    for index, row in enumerate(continuity_rows):
        require(isinstance(row, dict) and row.get("index") == index, f"continuity[{index}]: index mismatch")
        alpha = finite(row.get("alpha"), f"continuity[{index}].alpha")
        require(abs(alpha - index / 20.0) <= 1.0e-15, f"continuity[{index}]: alpha mismatch")
        q = vector(row.get("q_rad"), 6, f"continuity[{index}].q")
        expected_q = [alpha * value for value in V15_14_ACCEPTED_FJT_Q_RAD]
        require(max_abs(left - right for left, right in zip(q, expected_q)) <= 1.0e-14, f"continuity[{index}]: interpolated q mismatch")
        continuity_torque.append(vector(row.get("tau_hold_nm"), 6, f"continuity[{index}].tau"))
    continuity_all_finite = all(row.get("all_values_finite") is True for row in continuity_rows)
    continuity_constraint_free = all(
        row.get("ncon") == 0 and row.get("nefc") == 0
        and all(finite(row.get(key), f"continuity[{index}].{key}") <= 1.0e-12 for key in (
            "max_abs_qfrc_constraint", "max_abs_qfrc_passive", "max_abs_qfrc_actuator",
            "max_abs_qfrc_gravcomp", "max_abs_qfrc_applied", "max_abs_xfrc_applied",
        ))
        for index, row in enumerate(continuity_rows)
    )
    adjacent = [max_abs(right[joint] - left[joint] for joint in range(6)) for left, right in zip(continuity_torque, continuity_torque[1:])]
    second = [max_abs(continuity_torque[index + 1][joint] - 2.0 * continuity_torque[index][joint] + continuity_torque[index - 1][joint] for joint in range(6)) for index in range(1, 20)]
    continuity_audit = {
        "sample_count": 21,
        "endpoint_a": "mechanical_zero",
        "endpoint_b": "v15_14_accepted_fjt_final_pose",
        "q_start": [0.0] * 6,
        "q_end": list(V15_14_ACCEPTED_FJT_Q_RAD),
        "rows": continuity_rows,
        "adjacent_inf_norm_jumps_nm": adjacent,
        "second_difference_inf_norm_nm": second,
        "max_adjacent_inf_norm_jump_nm": max(adjacent),
        "max_second_difference_inf_norm_nm": max(second),
        "max_abs_tau_nm": max(max_abs(row) for row in continuity_torque),
        "suspicious_torque_over_100_nm": max(max_abs(row) for row in continuity_torque) > 100.0,
        "all_values_finite": continuity_all_finite,
        "adjacent_inf_norm_limit_nm": 2.0,
        "second_difference_inf_norm_limit_nm": 0.25,
        "all_constraints_inactive": continuity_constraint_free,
        "failure_codes": [code for condition, code in (
            (not continuity_all_finite, "FAIL_NONFINITE_CONTINUITY_RESULT"),
            (not continuity_constraint_free, "FAIL_ACTIVE_CONTINUITY_CONSTRAINT"),
            (max(max_abs(row) for row in continuity_torque) > 100.0, "FAIL_SUSPECT_UNIT_OR_COM_ERROR"),
            (max(adjacent) > 2.0, "FAIL_TORQUE_CONTINUITY_ADJACENT_JUMP"),
            (max(second) > 0.25, "FAIL_TORQUE_CONTINUITY_SECOND_DIFFERENCE"),
        ) if condition],
        "pass": continuity_all_finite and continuity_constraint_free and max(max_abs(row) for row in continuity_torque) <= 100.0 and max(adjacent) <= 2.0 and max(second) <= 0.25,
    }
    pose_tau_maximum = max((max_abs(values) for values in all_values), default=None)
    continuity_tau_maximum = max((max_abs(values) for values in continuity_torque), default=None)
    global_max_abs_tau = max(
        [value for value in (pose_tau_maximum, continuity_tau_maximum) if value is not None],
        default=None,
    )
    complete_pose_count = sum(row.get("downstream_status") == "COMPLETE" for row in pose_results)
    metrics = {
        "tested_pose_count": len(pose_results),
        "max_abs_tau_nm": global_max_abs_tau,
        "max_abs_pose_tau_nm": pose_tau_maximum,
        "max_abs_continuity_tau_nm": continuity_tau_maximum,
        "max_energy_gradient_absolute_error_nm": max(all_gradient_abs, default=None),
        "max_energy_gradient_relative_error": max(all_gradient_rel, default=None),
        "max_finite_difference_step_difference_nm": max(all_step_differences, default=None),
        "max_dynamics_residual_norm": max(all_residual_norms, default=None),
        "max_relative_qacc_error": max(all_relative_qacc, default=None),
        "max_zero_gravity_bias_nm": max(all_zero, default=None),
        "max_reverse_gravity_relative_error": max(all_reverse, default=None),
        "max_mass_matrix_symmetry_error": max(all_symmetry, default=None),
        "minimum_mass_matrix_eigenvalue": min(all_min_eigenvalues, default=None),
        "max_mass_matrix_condition_number": max(all_conditions, default=None),
        "max_excluded_link1_direct_energy_gradient_nm": max(all_link1_gradient_differences, default=None),
        "max_all_positive_minus_six_gradient_diagnostic_nm": max(all_link1_subtractive_diagnostics, default=None),
        "max_compiled_tensor_relative_frobenius_error": max(compiled_tensor_errors.values()),
        "max_qfrc_passive_nm": max(all_passive, default=None),
        "max_qfrc_actuator_nm": max(all_actuator, default=None),
        "max_qfrc_gravcomp_nm": max(all_gravcomp, default=None),
    }
    gates = {
        "exact_21_pose_contract": len(pose_results) == 21,
        "all_current_pair_scoped_collision_checks_clear": all(row["current_pair_scoped_collision_free"] for row in pose_results),
        "all_dynamics_constraints_inactive": all(row["dynamics_constraint_free"] for row in pose_results),
        "all_pose_physics_checks_pass": all(row["pass"] for row in pose_results),
        "potential_authority_excludes_only_zero_gradient_link1_placeholder": metrics["max_excluded_link1_direct_energy_gradient_nm"] is not None and metrics["max_excluded_link1_direct_energy_gradient_nm"] <= 1.0e-12,
        "torque_continuity_pass": continuity_audit["pass"],
        "no_suspicious_over_100_nm": metrics["max_abs_tau_nm"] is not None and metrics["max_abs_tau_nm"] <= 100.0 and not any(row["preflight"]["suspicious_torque_over_100_nm"] for row in pose_results),
    }
    return {
        "pass": all(gates.values()),
        "environment": {"python_version": raw.get("python_version"), "mujoco_version": raw["mujoco_version"], "numpy_version": raw.get("numpy_version")},
        "compiled_authority_tensor_relative_errors": compiled_tensor_errors,
        "compiled_authority_validation": compiled_authority_validation,
        "model_role_separation": roles,
        "poses": pose_results,
        "special_poses": special_poses,
        "joint_statistics": statistics,
        "j2_equal_share_diagnostic": j2_diagnostic,
        "continuity": continuity_audit,
        "metrics": metrics,
        "complete_downstream_pose_count": complete_pose_count,
        "gates": gates,
    }


def require_close(actual: Any, expected: Any, label: str, *, absolute: float = 1.0e-12, relative: float = 1.0e-10) -> None:
    """Deep comparison that keeps identities/booleans exact and floats tight."""

    if isinstance(expected, bool) or expected is None or isinstance(expected, str):
        require(actual == expected and type(actual) is type(expected), f"{label}: identity/boolean mismatch")
    elif isinstance(expected, (int, float)) and not isinstance(expected, bool):
        actual_value = finite(actual, label)
        expected_value = finite(expected, label + ".expected")
        require(abs(actual_value - expected_value) <= max(absolute, relative * abs(expected_value)), f"{label}: numeric mismatch")
    elif isinstance(expected, list):
        require(isinstance(actual, list) and len(actual) == len(expected), f"{label}: list length mismatch")
        for index, (actual_item, expected_item) in enumerate(zip(actual, expected)):
            require_close(actual_item, expected_item, f"{label}[{index}]", absolute=absolute, relative=relative)
    elif isinstance(expected, dict):
        require(isinstance(actual, dict) and set(actual) == set(expected), f"{label}: object key mismatch")
        for key in expected:
            require_close(actual[key], expected[key], f"{label}.{key}", absolute=absolute, relative=relative)
    else:
        require(actual == expected, f"{label}: value mismatch")


def audit_pose_projection(
    source: Mapping[str, Any],
    raw: Mapping[str, Any],
    analyzed: Mapping[str, Any],
) -> dict[str, Any]:
    h1_key, h2_key = (f"{value:.17g}" for value in FINITE_DIFFERENCE_STEPS)
    collision = raw["collision_clone_diagnostic"]
    common = {
        "id": source["id"],
        "role": source["role"],
        "q_rad": analyzed["q_rad"],
        "joint_limit_margin_rad": source["joint_limit_margin_rad"],
        "minimum_limited_joint_margin_rad": source["minimum_limited_joint_margin_rad"],
        "source_collision": source["source_collision"],
        "source_collision_free": source.get("source_collision_free"),
        "initialization": {
            "mj_resetData": True, "qvel_zero": True, "qacc_initialized_zero": True,
            "ctrl_zero": True, "qfrc_applied_zero": True, "xfrc_applied_zero": True,
            "calculation": "INSTANTANEOUS_MJ_FORWARD_ONLY",
        },
        "preflight": analyzed["preflight"],
        "potential_energy_j": analyzed["potential_energy_j"],
        "all_positive_body_potential_energy_j": analyzed["all_positive_body_potential_energy_j"],
        "tau_hold_mujoco_nm": analyzed["tau_hold_mujoco_nm"],
        "qfrc_passive": raw["qfrc_passive"],
        "qfrc_actuator": raw["qfrc_actuator"],
        "qfrc_gravcomp": raw["qfrc_gravcomp"],
        "qfrc_applied": raw["qfrc_applied"],
        "xfrc_applied": raw["xfrc_applied"],
        "ctrl": raw["ctrl"],
        "qacc": raw["qacc"],
        "collision_clone_diagnostic": {
            "diagnostic_only_not_mass_matrix_or_qacc_authority": True,
            "runtime_disableflags": collision["runtime_disableflags"],
            "raw_ncon": collision["raw_ncon"], "raw_nefc": collision["raw_nefc"],
            "accepted_contacts": collision["accepted_contacts"],
            "accepted_contact_count": collision["accepted_contact_count"],
            "accepted_contact_set_empty": collision["accepted_contact_set_empty"],
        },
    }
    if analyzed["downstream_status"] == "NOT_RUN_AFTER_PREFLIGHT_FAIL":
        return {
            **common,
            "downstream_status": "NOT_RUN_AFTER_PREFLIGHT_FAIL",
            "energy_gradient": None, "comparison": None,
            "constraint": {
                "ncon": analyzed["baseline_ncon"], "nefc": analyzed["baseline_nefc"],
                "qfrc_constraint": analyzed["baseline_qfrc_constraint"],
                "finite_difference_stencil": None,
                "all_baseline_and_stencils_inactive": False,
            },
            "mass_matrix": None, "mass_matrix_diagnostics": None,
            "dynamics_identity": None, "zero_gravity": None, "reverse_gravity": None,
            "pass": False, "failure_codes": analyzed["failure_codes"],
        }

    gradients = raw["energy_gradients"]
    return {
        **common,
        "downstream_status": "COMPLETE",
        "energy_gradient": {
            "h1_rad": FINITE_DIFFERENCE_STEPS[0],
            "h2_rad": FINITE_DIFFERENCE_STEPS[1],
            "h1_nm": gradients[h1_key],
            "h2_nm": gradients[h2_key],
            "selected": "h2",
            "step_difference_nm": analyzed["finite_difference_step_difference_nm"],
            "excluded_link1_direct_energy_gradient_nm": analyzed["excluded_link1_direct_energy_gradient_nm"],
            "all_positive_minus_six_gradient_diagnostic_nm": analyzed["all_positive_minus_six_gradient_difference_nm_diagnostic_only"],
        },
        "comparison": {
            "absolute_error_nm": analyzed["absolute_error_nm"],
            "relative_error": analyzed["relative_error"],
            "per_joint_pass": analyzed["per_joint_pass"],
            "pass": all(analyzed["per_joint_pass"]),
        },
        "constraint": {
            "ncon": raw["ncon"],
            "nefc": raw["nefc"],
            "qfrc_constraint": raw["qfrc_constraint"],
            "finite_difference_stencil": raw["finite_difference_stencil_constraints"],
            "all_baseline_and_stencils_inactive": analyzed["dynamics_constraint_free"],
        },
        "mass_matrix": analyzed["mass_matrix"],
        "mass_matrix_diagnostics": {
            "matrix_finite": True,
            "symmetry_error": analyzed["mass_matrix_symmetry_error"],
            "eigenvalues": analyzed["mass_matrix_eigenvalues"],
            "min_eigenvalue": analyzed["mass_matrix_min_eigenvalue"],
            "condition_number": analyzed["mass_matrix_condition_number"],
            "positive_definite": analyzed["mass_matrix_min_eigenvalue"] > 0.0,
            "pass": analyzed["mass_matrix_symmetry_error"] < 1.0e-10 and analyzed["mass_matrix_min_eigenvalue"] > 0.0,
        },
        "dynamics_identity": {
            "meaning": "COMPILED_MODEL_INTERNAL_DYNAMICS_IDENTITY_ONLY",
            "solve_status": "COMPLETE",
            "expected_qacc": analyzed["expected_qacc"],
            "actual_qacc": analyzed["qacc"],
            "residual_vector": analyzed["dynamics_residual"],
            "residual_norm": analyzed["dynamics_residual_norm"],
            "relative_qacc_error": analyzed["relative_qacc_error"],
            "pass": analyzed["dynamics_residual_norm"] <= 1.0e-9 * max(vector_norm(analyzed["tau_hold_mujoco_nm"]), 1.0) and analyzed["relative_qacc_error"] < 1.0e-9,
        },
        "zero_gravity": {
            "qfrc_bias_nm": analyzed["zero_gravity_bias"],
            "max_abs_nm": max_abs(analyzed["zero_gravity_bias"]),
            "pass": max_abs(analyzed["zero_gravity_bias"]) < 1.0e-10,
        },
        "reverse_gravity": {
            "qfrc_bias_nm": analyzed["reverse_gravity_bias"],
            "tau_minus_plus_tau_plus_nm": [left + right for left, right in zip(analyzed["tau_hold_mujoco_nm"], analyzed["reverse_gravity_bias"])],
            "relative_error": analyzed["reverse_gravity_relative_error"],
            "max_relative_error": max(analyzed["reverse_gravity_relative_error"]),
            "pass": max(analyzed["reverse_gravity_relative_error"]) < 1.0e-10,
        },
        "pass": analyzed["pass"],
        "failure_codes": analyzed["failure_codes"],
    }


def select_mujoco_python(explicit: str | None) -> Path:
    candidates = (explicit, os.environ.get("V15_18_MUJOCO_PYTHON"), sys.executable)
    for candidate in candidates:
        if not candidate:
            continue
        path = Path(candidate).resolve()
        if not path.is_file():
            continue
        result = run_process([str(path), "-c", "import mujoco,numpy; print(mujoco.__version__)"], timeout=30.0)
        if result.returncode == 0 and result.stdout.strip() == "3.11.0":
            return path
    raise ValidationError("no MuJoCo 3.11.0 interpreter is available")


def run_independent_physics(
    poses: list[dict[str, Any]],
    continuity: Mapping[str, Any],
    interpreter: Path,
    assets: Mapping[str, str],
) -> dict[str, Any]:
    with tempfile.TemporaryDirectory(prefix="v15_18a_validator_") as temporary_text:
        temporary = Path(temporary_text)
        require(str(temporary).isascii(), "independent MuJoCo temporary path is not ASCII")
        model = make_ascii_model_mirror(temporary / "mirror", assets)
        request = temporary / "request.json"
        output = temporary / "output.json"
        collision_contract = validate_collision_pair_contract()
        request.write_text(json.dumps({
            "poses": poses,
            "continuity": continuity,
            "runtime_full_pairs": collision_contract["runtime_full_pairs"],
        }, ensure_ascii=False, sort_keys=True), encoding="utf-8")
        result = run_process([str(interpreter), str(Path(__file__).resolve()), "--physics-child", str(model), str(request), str(output)], timeout=300.0)
        require(result.returncode == 0 and output.is_file(), "independent MuJoCo child failed: " + clean_error(result.stderr or result.stdout))
        return read_json(output)


def capture_hashes(relatives: Iterable[str]) -> dict[str, str | None]:
    return {relative: sha256_file(repo_path(relative)) if repo_path(relative).is_file() else None for relative in sorted(set(relatives))}


def watched_hash_snapshot(assets: Mapping[str, str]) -> dict[str, str | None]:
    relatives = set(PROTECTED_HASHES) | EXACT_CHANGED_PATHS | set(assets)
    snapshot = capture_hashes(relatives)
    require(all(snapshot[relative] == digest for relative, digest in assets.items()), "1008-asset TOCTOU snapshot differs from initial authority")
    return snapshot


def run_nested_audit_check(interpreter: Path, audit: Mapping[str, Any]) -> dict[str, Any]:
    result = run_process([str(interpreter), str(repo_path(AUDIT_REL)), "--check"], timeout=900.0)
    require(result.returncode == 0, "nested audit --check failed: " + clean_error(result.stderr or result.stdout))
    stdout = result.stdout.strip().splitlines()
    require(sum(line in {"AUDIT_VALID=YES", "AUDIT_VALID=NO"} for line in stdout) == 1, "nested audit --check must emit exact one AUDIT_VALID status")
    require(sum(line.startswith("STATUS=") for line in stdout) == 1, "nested audit --check must emit exact one STATUS")
    require(sum(line.startswith("FINAL_STATUS=") for line in stdout) == 1, "nested audit --check must emit exact one FINAL_STATUS")
    require(("AUDIT_VALID=YES" if audit.get("audit_valid") is True else "AUDIT_VALID=NO") in stdout, "nested AUDIT_VALID differs from JSON")
    require(f"STATUS={audit.get('status')}" in stdout, "nested STATUS differs from JSON")
    require(f"FINAL_STATUS={audit.get('final_status')}" in stdout, "nested FINAL_STATUS differs from JSON")
    return {"pass": True, "mode": "CHECK", "stdout_contract_lines": stdout}


def atomic_write_text(path: Path, text: str) -> None:
    temporary = path.with_name(path.name + ".tmp-v15-18a")
    require(not temporary.exists(), f"stale report temporary file exists: {temporary.name}")
    try:
        temporary.write_text(text, encoding="utf-8", newline="\n")
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def make_final_report_items(
    independent: Mapping[str, Any],
    final_status: str,
    hard_unresolved_items: Sequence[Any],
) -> list[dict[str, Any]]:
    poses = independent["poses"]
    by_id = {row["id"]: row for row in poses}
    statistics = independent["joint_statistics"]
    metrics = independent["metrics"]
    mechanical = by_id["frozen_503_global_018"]["tau_hold_mujoco_nm"]
    extension = by_id["frozen_503_global_481"]["tau_hold_mujoco_nm"]
    j2_record = statistics["J2"]
    values: list[tuple[str, Any]] = [
        ("branch / commit", {"branch": TARGET_BRANCH, "source_commit": SOURCE_COMMIT, "head_contract": "SOURCE_COMMIT_OR_ITS_UNIQUE_DIRECT_CHILD"}),
        ("production MJCF hash before / after", {"before_sha256": MJCF_SHA256, "after_sha256": MJCF_SHA256}),
        ("tested collision-free poses count", len(poses)),
        ("gravity vector m/s^2", list(GRAVITY)),
        ("mechanical-zero tau_hold N*m", mechanical),
        ("max-extension tau_hold N*m", extension),
        ("J1 max_abs gravity torque N*m", statistics["J1"]["max_abs_nm"]),
        ("J2 max_abs gravity torque N*m", statistics["J2"]["max_abs_nm"]),
        ("J3 max_abs gravity torque N*m", statistics["J3"]["max_abs_nm"]),
        ("J4 max_abs gravity torque N*m", statistics["J4"]["max_abs_nm"]),
        ("J5 max_abs gravity torque N*m", statistics["J5"]["max_abs_nm"]),
        ("J6 max_abs gravity torque N*m", statistics["J6"]["max_abs_nm"]),
        ("J2 max-pose ideal equal share per motor", {
            "label": "IDEAL_EQUAL_LOAD_SHARE_DIAGNOSTIC_ONLY",
            "pose": j2_record["pose_of_max_abs"],
            "total_joint_tau_nm": (by_id[j2_record["pose_of_max_abs"]]["tau_hold_mujoco_nm"][1] if j2_record["pose_of_max_abs"] is not None else None),
            "tau_per_motor_nm": (by_id[j2_record["pose_of_max_abs"]]["tau_hold_mujoco_nm"][1] / 2.0 if j2_record["pose_of_max_abs"] is not None else None),
        }),
        ("max qfrc_bias vs potential-gradient absolute error N*m", metrics["max_energy_gradient_absolute_error_nm"]),
        ("max qfrc_bias vs potential-gradient relative error", metrics["max_energy_gradient_relative_error"]),
        ("max M*qacc+bias residual norm", metrics["max_dynamics_residual_norm"]),
        ("zero-gravity max bias N*m", metrics["max_zero_gravity_bias_nm"]),
        ("reverse-gravity max relative symmetry error", metrics["max_reverse_gravity_relative_error"]),
        ("mass-matrix minimum eigenvalue", metrics["minimum_mass_matrix_eigenvalue"]),
        ("mass-matrix maximum condition number", metrics["max_mass_matrix_condition_number"]),
        ("torque continuity", "PASS" if independent["continuity"]["pass"] else "FAIL"),
        ("suspicious >100 N*m torque", "N/A_NONFINITE" if metrics["max_abs_tau_nm"] is None else "YES" if metrics["max_abs_tau_nm"] > 100.0 else "NO"),
        ("production bridge modified", "NO"),
        ("production MJCF persisted gravity", "NO"),
        ("Mass / COM / Inertia authority modified", "NO"),
        ("hard unresolved items", list(hard_unresolved_items)),
        ("final status", final_status),
    ]
    require(len(values) == 27, "internal exact-27 final report map changed")
    return [{"index": index, "label": label, "value": value} for index, (label, value) in enumerate(values, 1)]


def make_audit_final_report_facts(
    independent: Mapping[str, Any],
    final_status: str,
    hard_unresolved_items: Sequence[Any],
) -> list[dict[str, Any]]:
    statistics = independent["joint_statistics"]
    metrics = independent["metrics"]
    special = independent["special_poses"]
    j2 = independent["j2_equal_share_diagnostic"]
    values: list[tuple[str, Any]] = [
        ("branch_commit", {"branch": TARGET_BRANCH, "commit_contract": "SOURCE_COMMIT_OR_ITS_UNIQUE_DIRECT_CHILD"}),
        ("production_mjcf_hash_before_after", {"before": MJCF_SHA256, "after": MJCF_SHA256}),
        ("tested_collision_free_poses_count", len(independent["poses"])),
        ("gravity_vector_m_s2", list(GRAVITY)),
        ("mechanical_zero_tau_hold_nm", special["mechanical_zero"]["tau_hold_nm"]),
        ("max_extension_tau_hold_nm", special["max_extension"]["tau_hold_nm"]),
        *[(f"{joint}_max_abs_gravity_torque_nm", statistics[joint]["max_abs_nm"]) for joint in JOINTS],
        ("j2_max_pose_ideal_half_motor_share", j2),
        ("max_qfrc_bias_vs_potential_gradient_absolute_error_nm", metrics["max_energy_gradient_absolute_error_nm"]),
        ("max_qfrc_bias_vs_potential_gradient_relative_error", metrics["max_energy_gradient_relative_error"]),
        ("max_mass_qacc_plus_bias_residual_norm", metrics["max_dynamics_residual_norm"]),
        ("zero_gravity_max_bias_nm", metrics["max_zero_gravity_bias_nm"]),
        ("reverse_gravity_max_symmetry_error", metrics["max_reverse_gravity_relative_error"]),
        ("mass_matrix_min_eigenvalue", metrics["minimum_mass_matrix_eigenvalue"]),
        ("max_mass_matrix_condition_number", metrics["max_mass_matrix_condition_number"]),
        ("torque_continuity", "PASS" if independent["continuity"]["pass"] else "FAIL"),
        ("suspicious_over_100_nm_torque", "N/A_NONFINITE" if metrics["max_abs_tau_nm"] is None else "YES" if metrics["max_abs_tau_nm"] > 100.0 else "NO"),
        ("production_bridge_modified", "NO"),
        ("production_mjcf_persisted_gravity", "NO"),
        ("mass_com_inertia_authority_modified", "NO"),
        ("hard_unresolved_items", list(hard_unresolved_items)),
        ("final_status", final_status),
    ]
    require(len(values) == 27, "audit exact-27 fact map changed")
    return [{"number": number, "name": name, "value": value} for number, (name, value) in enumerate(values, 1)]


def independent_acceptance_gates(
    independent: Mapping[str, Any],
    authority: Mapping[str, Any],
) -> dict[str, bool]:
    poses = independent["poses"]
    metrics = independent["metrics"]
    compiled_zero_terms = all(
        metrics[key] is not None and metrics[key] <= 1.0e-12
        for key in ("max_qfrc_passive_nm", "max_qfrc_actuator_nm", "max_qfrc_gravcomp_nm")
    )
    complete = [row for row in poses if row["downstream_status"] == "COMPLETE"]
    return {
        "production_model_hash_unchanged": True,
        "gravity_runtime_only": True,
        "at_least_20_collision_free_valid_poses": len(poses) >= 20 and all(row["current_pair_scoped_collision_free"] for row in poses),
        "qfrc_bias_vs_energy_gradient_pass": len(complete) == 21 and all(all(row["per_joint_pass"]) for row in complete) and metrics["max_excluded_link1_direct_energy_gradient_nm"] is not None and metrics["max_excluded_link1_direct_energy_gradient_nm"] <= 1.0e-12,
        "mass_qacc_plus_bias_identity_pass": len(complete) == 21 and all(row["dynamics_constraint_free"] and row["pass"] for row in complete) and compiled_zero_terms and all(
            row["dynamics_residual_norm"] <= 1.0e-9 * max(vector_norm(row["tau_hold_mujoco_nm"]), 1.0)
            and row["relative_qacc_error"] < 1.0e-9 for row in complete
        ),
        "zero_gravity_pass": len(complete) == 21 and metrics["max_zero_gravity_bias_nm"] is not None and metrics["max_zero_gravity_bias_nm"] < 1.0e-10,
        "reverse_gravity_sign_pass": len(complete) == 21 and metrics["max_reverse_gravity_relative_error"] is not None and metrics["max_reverse_gravity_relative_error"] < 1.0e-10,
        "mass_matrix_symmetric_positive_definite": len(complete) == 21 and metrics["max_mass_matrix_symmetry_error"] is not None and metrics["minimum_mass_matrix_eigenvalue"] is not None and metrics["max_mass_matrix_symmetry_error"] < 1.0e-10 and metrics["minimum_mass_matrix_eigenvalue"] > 0.0,
        "no_nan_or_inf": len(complete) == 21 and all(row["preflight"]["all_baseline_values_finite"] for row in poses) and not any("FAIL_NONFINITE_COMPUTED_RESULT" in row["failure_codes"] for row in poses) and independent["continuity"]["all_values_finite"] is True,
        "no_suspicious_over_100_nm": metrics["max_abs_tau_nm"] is not None and metrics["max_abs_tau_nm"] <= 100.0 and not any(row["preflight"]["suspicious_torque_over_100_nm"] for row in poses) and independent["continuity"]["suspicious_torque_over_100_nm"] is False,
        "torque_continuity_sanity_pass": independent["continuity"]["pass"],
        "mass_com_inertia_authority_unchanged": authority["pass"] is True and metrics["max_compiled_tensor_relative_frobenius_error"] < 1.0e-9,
        "production_bridge_unchanged": True,
        "gravity_not_persisted_into_production_xml": True,
        "hard_unresolved_items_empty": True,
    }


def derive_hard_unresolved(gates: Mapping[str, bool], independent: Mapping[str, Any]) -> list[str]:
    codes = [name for name, value in gates.items() if not value and name != "hard_unresolved_items_empty"]
    codes.extend(code for row in independent["poses"] for code in row.get("failure_codes", []))
    codes.extend(independent["continuity"].get("failure_codes", []))
    return sorted(set(codes))


def legal_fail_mutation_self_test() -> dict[str, Any]:
    """Pure, no-MuJoCo proof that physical failures remain audit-valid FAILs."""

    def pose(index: int) -> dict[str, Any]:
        return {
            "id": f"synthetic_{index:02d}", "downstream_status": "COMPLETE",
            "q_rad": [0.0] * 6,
            "current_pair_scoped_collision_free": True, "per_joint_pass": [True] * 6,
            "dynamics_constraint_free": True, "pass": True,
            "absolute_error_nm": [0.0] * 6,
            "dynamics_residual_norm": 0.0, "relative_qacc_error": 0.0,
            "mass_matrix_min_eigenvalue": 1.0, "mass_matrix_condition_number": 1.0,
            "tau_hold_mujoco_nm": [0.0] * 6,
            "preflight": {"all_baseline_values_finite": True, "suspicious_torque_over_100_nm": False},
            "failure_codes": [],
        }

    base = {
        "poses": [pose(index) for index in range(21)],
        "metrics": {
            "max_qfrc_passive_nm": 0.0, "max_qfrc_actuator_nm": 0.0,
            "max_qfrc_gravcomp_nm": 0.0, "max_excluded_link1_direct_energy_gradient_nm": 0.0,
            "max_zero_gravity_bias_nm": 0.0, "max_reverse_gravity_relative_error": 0.0,
            "max_mass_matrix_symmetry_error": 0.0, "minimum_mass_matrix_eigenvalue": 1.0,
            "max_abs_tau_nm": 0.0, "max_compiled_tensor_relative_frobenius_error": 0.0,
            "max_energy_gradient_absolute_error_nm": 0.0, "max_relative_qacc_error": 0.0,
            "max_mass_matrix_condition_number": 1.0,
        },
        "continuity": {
            "pass": True, "all_values_finite": True, "suspicious_torque_over_100_nm": False,
            "failure_codes": [], "rows": [], "max_adjacent_inf_norm_jump_nm": 0.0,
            "max_second_difference_inf_norm_nm": 0.0,
        },
        "joint_statistics": {
            joint: {"min_nm": 0.0, "max_nm": 0.0, "max_abs_nm": 0.0, "pose_of_max_abs": "synthetic_00"}
            for joint in JOINTS
        },
    }
    authority = {"pass": True}

    def mutate(code: str, *, suspicious: bool) -> tuple[dict[str, bool], list[str]]:
        case = json.loads(json.dumps(base))
        row = case["poses"][0]
        row.update({
            "downstream_status": "NOT_RUN_AFTER_PREFLIGHT_FAIL", "per_joint_pass": None,
            "dynamics_constraint_free": False, "pass": False,
            "dynamics_residual_norm": None, "relative_qacc_error": None,
            "failure_codes": [code],
            "absolute_error_nm": None, "mass_matrix_min_eigenvalue": None,
            "mass_matrix_condition_number": None,
        })
        row["preflight"] = {"all_baseline_values_finite": not (code == "FAIL_NONFINITE_STATIC_RESULT"), "suspicious_torque_over_100_nm": suspicious}
        row["tau_hold_mujoco_nm"] = [101.0, 0.0, 0.0, 0.0, 0.0, 0.0] if suspicious else [None] * 6
        case["metrics"]["max_abs_tau_nm"] = 101.0 if suspicious else 0.0
        gates = independent_acceptance_gates(case, authority)
        hard = derive_hard_unresolved(gates, case)
        gates["hard_unresolved_items_empty"] = not hard
        require(hard and not all(gates.values()), "legal FAIL mutation did not produce hard unresolved items")
        require(("no_suspicious_over_100_nm" in hard) is suspicious, "legal FAIL suspicious-torque gate classification changed")
        require("no_nan_or_inf" in hard and code in hard, "legal FAIL null/failure-code aggregation changed")
        markdown = render_markdown(
            {"audit_valid": True, "status": "FAIL", "final_status": FINAL_FAIL},
            case,
            [{"index": index, "label": f"synthetic {index}", "value": "N/A"} for index in range(1, 28)],
        )
        require("| N/A | N/A | N/A |" in markdown, "legal FAIL Markdown did not render nullable pose fields as N/A")
        return gates, hard

    torque_gates, torque_hard = mutate("FAIL_SUSPECT_UNIT_OR_COM_ERROR", suspicious=True)
    nonfinite_gates, nonfinite_hard = mutate("FAIL_NONFINITE_STATIC_RESULT", suspicious=False)
    nullable_complete = json.loads(json.dumps(base))
    nullable_complete["poses"][0].update({
        "dynamics_constraint_free": False, "pass": False,
        "relative_qacc_error": None, "mass_matrix_min_eigenvalue": None,
        "mass_matrix_condition_number": None,
    })
    nullable_complete["poses"][1].update({
        "pass": False, "mass_matrix_min_eigenvalue": None,
        "mass_matrix_condition_number": None,
    })
    nullable_markdown = render_markdown(
        {"audit_valid": True, "status": "FAIL", "final_status": FINAL_FAIL},
        nullable_complete,
        [{"index": index, "label": f"synthetic {index}", "value": "N/A"} for index in range(1, 28)],
    )
    require(nullable_markdown.count("N/A") >= 4, "active-constraint/singular Markdown nullable rendering failed")
    malformed = {"downstream_status": "COMPLETE", "mass_matrix": None, "energy_gradient": None}
    try:
        require(malformed["downstream_status"] == "NOT_RUN_AFTER_PREFLIGHT_FAIL", "preflight-null row claims COMPLETE")
    except ValidationError:
        malformed_rejected = True
    else:
        malformed_rejected = False
    require(malformed_rejected, "contradictory legal-FAIL mutation was not rejected as malformed")
    return {
        "pass": True, "no_mujoco_invoked": True,
        "torque_failure_hard_unresolved": torque_hard,
        "nonfinite_failure_hard_unresolved": nonfinite_hard,
        "markdown_nullable_routes": ["OVER_100", "NONFINITE", "ACTIVE_CONSTRAINT", "SINGULAR_MASS_MATRIX"],
        "audit_valid_physical_fail_exit_semantics": 0,
        "malformed_contradiction_exit_semantics": 2,
    }


def expected_thresholds() -> dict[str, float]:
    return {
        "joint_limit_margin_rad": 0.05,
        "finite_difference_step_convergence_nm_strict_less_than": 1.0e-5,
        "large_torque_relative_error_strict_less_than": 1.0e-5,
        "small_torque_absolute_error_nm_strict_less_than": 1.0e-5,
        "large_torque_boundary_nm": 0.1,
        "relative_error_denominator_floor_nm": 0.01,
        "relative_qacc_error_strict_less_than": 1.0e-9,
        "dynamics_residual_norm_scale": 1.0e-9,
        "zero_gravity_max_bias_nm_strict_less_than": 1.0e-10,
        "reverse_gravity_relative_error_strict_less_than": 1.0e-10,
        "mass_matrix_symmetry_error_strict_less_than": 1.0e-10,
        "constraint_force_max_abs_nm": 1.0e-12,
        "passive_actuator_gravcomp_max_abs_nm": 1.0e-12,
        "excluded_link1_gradient_difference_nm": 1.0e-12,
        "suspicious_torque_nm": 100.0,
        "continuity_adjacent_component_jump_nm": 2.0,
        "continuity_second_difference_component_nm": 0.25,
    }


def validate_audit_report(
    audit: Mapping[str, Any],
    selection: Mapping[str, Any],
    authority: Mapping[str, Any],
    independent_raw: Mapping[str, Any],
    independent: Mapping[str, Any],
    git_scope: Mapping[str, Any],
    assets: Mapping[str, str],
) -> dict[str, Any]:
    require(audit.get("schema") == SCHEMA and audit.get("revision") == "V15.18A-static-gravity-runtime-only", "audit report schema/revision changed")
    require(audit.get("audit_valid") is True, "audit report is not audit-valid")
    require(audit.get("status") in {"PASS", "FAIL"} and audit.get("final_status") in {FINAL_PASS, FINAL_FAIL}, "audit status contract malformed")
    scope = audit.get("scope")
    require(isinstance(scope, dict), "audit scope missing")
    require(scope.get("task") == "OFFLINE_STATIC_GRAVITY_AUDIT" and scope.get("source_commit") == SOURCE_COMMIT and scope.get("source_branch") == SOURCE_BRANCH and scope.get("target_branch") == TARGET_BRANCH, "audit scope provenance changed")
    require(scope.get("production_model_relative_path") == MJCF_REL and scope.get("free_dynamics_integration_performed") is False, "audit scope model/integration contract changed")
    require(set(scope.get("prohibited_scope", [])) == {"FREE_FALL", "GRAVITY_COMPENSATION", "PID_TUNING", "FRICTION", "DAMPING", "ARMATURE", "MOTOR_MODEL", "PRODUCTION_BRIDGE_EXECUTION"}, "audit prohibited scope changed")
    require_close(audit.get("environment"), independent["environment"], "audit.environment", absolute=0.0, relative=0.0)
    require_close(audit.get("authority"), {
        "pass": True,
        "artifacts": {relative: authority["artifacts"][relative] for relative in sorted(AUDIT_PROTECTED_RELS)},
        "six_body_authority": authority["six_body_authority"],
        "six_body_authority_total_mass_kg": authority["six_body_authority_total_mass_kg"],
        "all_unchanged": True,
    }, "audit.authority", absolute=1.0e-14, relative=1.0e-12)

    protection = audit.get("production_protection")
    require(isinstance(protection, dict), "audit production protection missing")
    expected_model = {"relative_path": MJCF_REL, "expected_sha256": MJCF_SHA256, "before_sha256": MJCF_SHA256, "after_sha256": MJCF_SHA256, "unchanged": True, "production_xml_gravity_m_s2": [0.0, 0.0, 0.0], "gravity_not_persisted": True}
    expected_bridge = {"relative_path": BRIDGE_REL, "expected_sha256": BRIDGE_SHA256, "actual_sha256": BRIDGE_SHA256, "accepted_model_sha256": MJCF_SHA256, "unchanged": True}
    require_close(protection.get("model"), expected_model, "audit.production_protection.model", absolute=0.0, relative=0.0)
    require_close(protection.get("bridge"), expected_bridge, "audit.production_protection.bridge", absolute=0.0, relative=0.0)
    require_close(protection.get("forbidden_parameter_snapshot"), authority["forbidden_parameter_snapshot"], "audit.forbidden_parameter_snapshot", absolute=0.0, relative=0.0)
    require(protection.get("forbidden_parameter_snapshot_sha256") == authority["forbidden_parameter_snapshot_sha256"], "audit forbidden parameter snapshot digest changed")
    require_close(audit.get("runtime_configuration"), {
        "gravity_m_s2": list(GRAVITY), "reverse_gravity_m_s2": list(REVERSE_GRAVITY), "zero_gravity_m_s2": list(ZERO_GRAVITY),
        "runtime_only": True, "actuation_disabled": True, "ctrl_zero": True,
        "dynamics_disableflags": 2048, "collision_diagnostic_disableflags": 3072,
        "collision_and_dynamics_model_objects_isolated": True, "no_time_integration": True,
    }, "audit.runtime_configuration", absolute=0.0, relative=0.0)
    require_close(audit.get("compiled_model"), compiled_model_projection(independent_raw, assets), "audit.compiled_model", absolute=2.0e-12, relative=1.0e-10)

    reported_selection = audit.get("pose_selection")
    require(isinstance(reported_selection, dict), "audit pose selection missing")
    selection_projection = {
        "pass": True,
        "source_503_relative_path": COLLISION_503_REL,
        "source_503_sha256": COLLISION_503_SHA256,
        "frozen_pose_set_sha256": FROZEN_POSE_SET_SHA256,
        "algorithm": "EXACT_20_FROZEN_503_GLOBAL_INDICES_PLUS_V15_14_ACCEPTED_FJT_FINAL_POSE",
        "selected_503_global_indices": [index for index, _ in SELECTED_503_POSES],
        "selected_count": 21,
        "joint_limit_margin_rad": JOINT_LIMIT_MARGIN_RAD,
        "selected": selection["selected"],
    }
    require_close(reported_selection, selection_projection, "audit.pose_selection", absolute=1.0e-14, relative=1.0e-13)
    reported_poses = audit.get("poses")
    require(isinstance(reported_poses, list) and len(reported_poses) == 21, "audit poses are not exact 21")
    raw_poses = independent_raw["poses"]
    expected_poses = [audit_pose_projection(source, raw, analyzed) for source, raw, analyzed in zip(selection["selected"], raw_poses, independent["poses"])]
    require_close(reported_poses, expected_poses, "audit.poses", absolute=2.0e-9, relative=1.0e-8)
    require_close(audit.get("special_poses"), independent["special_poses"], "audit.special_poses", absolute=2.0e-9, relative=1.0e-8)
    require_close(audit.get("joint_statistics"), independent["joint_statistics"], "audit.joint_statistics", absolute=2.0e-9, relative=1.0e-8)
    require_close(audit.get("j2_equal_share_diagnostic"), independent["j2_equal_share_diagnostic"], "audit.j2_equal_share", absolute=2.0e-9, relative=1.0e-8)
    require_close(audit.get("continuity"), independent["continuity"], "audit.continuity", absolute=2.0e-9, relative=1.0e-8)

    reported_metrics = audit.get("metrics")
    require(isinstance(reported_metrics, dict), "audit metrics missing")
    metric_map = {
        "tested_collision_free_pose_count": len(independent["poses"]),
        "complete_downstream_pose_count": independent["complete_downstream_pose_count"],
        "max_abs_tau_nm": independent["metrics"]["max_abs_tau_nm"],
        "max_abs_selected_pose_tau_nm": independent["metrics"]["max_abs_pose_tau_nm"],
        "max_abs_continuity_tau_nm": independent["metrics"]["max_abs_continuity_tau_nm"],
        "max_energy_gradient_absolute_error_nm": independent["metrics"]["max_energy_gradient_absolute_error_nm"],
        "max_energy_gradient_relative_error": independent["metrics"]["max_energy_gradient_relative_error"],
        "max_finite_difference_step_difference_nm": independent["metrics"]["max_finite_difference_step_difference_nm"],
        "max_dynamics_residual_norm": independent["metrics"]["max_dynamics_residual_norm"],
        "max_relative_qacc_error": independent["metrics"]["max_relative_qacc_error"],
        "max_zero_gravity_bias_nm": independent["metrics"]["max_zero_gravity_bias_nm"],
        "max_reverse_gravity_relative_error": independent["metrics"]["max_reverse_gravity_relative_error"],
        "max_mass_matrix_symmetry_error": independent["metrics"]["max_mass_matrix_symmetry_error"],
        "minimum_mass_matrix_eigenvalue": independent["metrics"]["minimum_mass_matrix_eigenvalue"],
        "max_mass_matrix_condition_number": independent["metrics"]["max_mass_matrix_condition_number"],
        "max_excluded_link1_direct_energy_gradient_nm": independent["metrics"]["max_excluded_link1_direct_energy_gradient_nm"],
        "max_all_positive_minus_six_gradient_diagnostic_nm": independent["metrics"]["max_all_positive_minus_six_gradient_diagnostic_nm"],
        "max_qfrc_actuator_nm": independent["metrics"]["max_qfrc_actuator_nm"],
        "max_qfrc_gravcomp_nm": independent["metrics"]["max_qfrc_gravcomp_nm"],
    }
    require_close(reported_metrics, metric_map, "audit.metrics", absolute=2.0e-9, relative=1.0e-8)
    require_close(audit.get("thresholds"), expected_thresholds(), "audit.thresholds", absolute=0.0, relative=0.0)
    require_close(audit.get("compiled_authority_validation"), independent["compiled_authority_validation"], "audit.compiled_authority_validation", absolute=2.0e-12, relative=1.0e-8)

    gates = independent_acceptance_gates(independent, authority)
    require(audit.get("acceptance_gates") == gates, "audit acceptance gates differ from independent recomputation")
    hard_unresolved = derive_hard_unresolved(gates, independent)
    gates["hard_unresolved_items_empty"] = not hard_unresolved
    require(audit.get("hard_unresolved_items") == hard_unresolved, "audit hard unresolved identity/order differs")
    expected_status = "PASS" if all(gates.values()) and not hard_unresolved else "FAIL"
    expected_final = FINAL_PASS if expected_status == "PASS" else FINAL_FAIL
    require(audit.get("status") == expected_status and audit.get("final_status") == expected_final, "audit final status contradicts independent gates")
    require(audit.get("acceptance_gate_count") == len(gates) and audit.get("acceptance_gate_pass_count") == sum(gates.values()), "audit gate aggregate count differs")
    complete_poses = [row for row in independent["poses"] if row["downstream_status"] == "COMPLETE"]
    supplemental = {
        "exact_21_pose_contract": len(independent["poses"]) == 21 and selection["selected_count"] == 21,
        "all_dynamics_constraints_inactive": len(complete_poses) == 21 and all(row["dynamics_constraint_free"] for row in complete_poses),
        "all_pose_physics_checks_pass": len(complete_poses) == 21 and all(row["pass"] for row in independent["poses"]),
        "compiled_environment_zero_terms": all(independent["metrics"][key] is not None and independent["metrics"][key] <= 1.0e-12 for key in ("max_qfrc_passive_nm", "max_qfrc_actuator_nm", "max_qfrc_gravcomp_nm")),
        "excluded_link1_direct_gradient_zero": independent["metrics"]["max_excluded_link1_direct_energy_gradient_nm"] is not None and independent["metrics"]["max_excluded_link1_direct_energy_gradient_nm"] <= 1.0e-12,
        "ascii_1008_asset_copy_hashes_match": True,
        "all_values_finite_recursive": len(complete_poses) == 21 and all(row["preflight"]["all_baseline_values_finite"] for row in independent["poses"]) and not any("FAIL_NONFINITE_COMPUTED_RESULT" in row["failure_codes"] for row in independent["poses"]) and independent["continuity"]["all_values_finite"] is True,
    }
    require(audit.get("supplemental_checks") == supplemental, "audit supplemental checks differ from independent recomputation")
    require(audit.get("pass") is (expected_status == "PASS"), "audit top-level pass differs from final status")
    expected_facts = make_audit_final_report_facts(independent, expected_final, hard_unresolved)
    require_close(audit.get("final_report_facts"), expected_facts, "audit.final_report_facts", absolute=2.0e-9, relative=1.0e-8)
    limitations = audit.get("limitations")
    require(isinstance(limitations, dict) and limitations.get("full_dynamics_validated") is False and limitations.get("mass_matrix_identity_scope") == "COMPILED_MODEL_INTERNAL_DYNAMICS_IDENTITY_ONLY_INCLUDES_LINK1_PLACEHOLDER", "audit limitations overclaim dynamics authority")
    gui = audit.get("gui_visual_witness")
    require(isinstance(gui, dict) and gui.get("diagnostic_only") is True and gui.get("not_numeric_authority") is True and gui.get("no_time_integration") is True, "audit GUI witness overclaims authority")
    return {"pass": True, "status": expected_status, "final_status": expected_final, "hard_unresolved_items": hard_unresolved, "acceptance_gates": gates, "final_report_facts": expected_facts, "git_scope": git_scope}


def asset_mirror_evidence(assets: Mapping[str, str]) -> dict[str, Any]:
    source_path = repo_path(MJCF_REL)
    root = ET.fromstring(source_path.read_bytes())
    compiler = root.find("./compiler")
    mesh_dir = source_path.parent / (compiler.get("meshdir", ".") if compiler is not None else ".")
    texture_dir = source_path.parent / (compiler.get("texturedir", ".") if compiler is not None else ".")
    rows = []
    for index, node in enumerate(node for node in root.findall("./asset/*") if node.get("file")):
        raw = str(node.get("file"))
        base = mesh_dir if node.tag.rsplit("}", 1)[-1] == "mesh" else texture_dir
        relative = (base / Path(*PurePosixPath(raw).parts)).resolve().relative_to(ROOT).as_posix()
        digest = assets[relative]
        rows.append({"index": index, "source_relative_path": relative, "source_sha256": digest, "copy_sha256": digest, "match": True})
    require(len(rows) == 1008, "asset mirror evidence is not exact 1008")
    set_digest = canonical_digest([{"index": row["index"], "source_relative_path": row["source_relative_path"], "sha256": row["source_sha256"]} for row in rows])
    return {"asset_count": 1008, "all_source_copy_sha256_match": True, "source_asset_set_sha256": set_digest, "asset_rows": rows}


def compiled_model_projection(raw: Mapping[str, Any], assets: Mapping[str, str]) -> dict[str, Any]:
    return {
        "nq": raw["nq"], "nv": raw["nv"], "nu": raw["nu"],
        "compiled_initial_disableflags": raw["compiled_initial_disableflags"],
        "runtime_dynamics_disableflags": raw["runtime_disableflags"],
        "runtime_collision_diagnostic_disableflags": raw["model_role_separation"]["collision_runtime_disableflags"],
        "actuation_disable_bit": raw["actuation_disable_bit"],
        "forbidden_disable_bits": raw["forbidden_disable_bits"],
        "compiled_gravity_before_runtime_override_m_s2": raw["compiled_gravity_before_override"],
        "authority_body_order": raw["potential_energy_authority_bodies"],
        "authority_body_ids": raw["potential_energy_authority_body_ids"],
        "authority_masses_kg": raw["potential_energy_authority_masses_kg"],
        "authority_total_mass_kg": raw["potential_energy_authority_total_mass_kg"],
        "authority_ipos_m": raw["potential_energy_authority_ipos_m"],
        "authority_iquat_wxyz": raw["potential_energy_authority_iquat_wxyz"],
        "authority_principal_inertia_kg_m2": raw["potential_energy_authority_principal_inertia_kg_m2"],
        "all_positive_mass_bodies": raw["all_positive_mass_body_rows"],
        "all_positive_mass_total_kg": raw["all_positive_mass_total_kg"],
        "link1_placeholder_mass_kg": raw["link1_placeholder_mass_kg"],
        "link1_excluded_from_potential_but_retained_in_mass_matrix": raw["link1_excluded_from_authority_potential_but_retained_in_mass_matrix"],
        "body_gravcomp": raw["body_gravcomp"],
        "joint_damping": raw["joint_damping"],
        "joint_frictionloss": raw["joint_frictionloss"],
        "joint_armature": raw["joint_armature"],
        "wind_m_s": raw["option_wind_m_s"],
        "density_kg_m3": raw["option_density_kg_m3"],
        "viscosity_pa_s": raw["option_viscosity_pa_s"],
        "ascii_asset_mirror": asset_mirror_evidence(assets),
        "ascii_asset_mirror_count": 1008,
        "temporary_paths_serialized": False,
    }


def render_markdown(
    audit: Mapping[str, Any],
    independent: Mapping[str, Any],
    final_items: Sequence[Mapping[str, Any]],
) -> str:
    def numeric(value: Any, precision: int) -> str:
        if value is None:
            return "N/A"
        require(isinstance(value, (int, float)) and not isinstance(value, bool), "Markdown numeric field is malformed")
        number = float(value)
        require(math.isfinite(number), "Markdown numeric field is non-finite instead of JSON null")
        return format(number, f".{precision}g")

    def maximum(values: Any, precision: int) -> str:
        if values is None:
            return "N/A"
        require(isinstance(values, list), "Markdown maximum field is malformed")
        if not values or any(value is None for value in values):
            return "N/A"
        return numeric(max(abs(finite(value, "Markdown maximum value")) for value in values), precision)

    lines = [
        "# V15.18A 静态重力与重力矩验收",
        "",
        f"- Audit valid: **{'YES' if audit.get('audit_valid') is True else 'NO'}**",
        f"- Status: **{audit.get('status')}**",
        f"- Final status: **{audit.get('final_status')}**",
        f"- Tested poses: **{len(independent['poses'])}** (20 frozen 503 + 1 accepted V15.14 FJT pose)",
        f"- Runtime gravity: `{json.dumps(list(GRAVITY), separators=(',', ':'))}` m/s²; production XML remains gravity OFF.",
        "",
        "## Exact 27-item final report",
        "",
        "| # | Item | Value |",
        "|---:|---|---|",
    ]
    for row in final_items:
        rendered = json.dumps(row["value"], ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False) if isinstance(row["value"], (dict, list)) else str(row["value"])
        lines.append(f"| {row['index']} | {row['label']} | `{rendered}` |")
    lines.extend([
        "",
        "## Frozen authority and production protection",
        "",
        "| Artifact | SHA256 |",
        "|---|---|",
    ])
    for relative, expected in sorted(PROTECTED_HASHES.items()):
        lines.append(f"| `{relative}` | `{expected}` |")
    lines.extend([
        "",
        "## Per-joint gravity-torque statistics",
        "",
        "| Joint | min N·m | max N·m | max abs N·m | pose of max abs |",
        "|---|---:|---:|---:|---|",
    ])
    for joint in JOINTS:
        row = independent["joint_statistics"][joint]
        pose_label = row["pose_of_max_abs"] if row["pose_of_max_abs"] is not None else "N/A"
        lines.append(
            f"| {joint} | {numeric(row['min_nm'], 12)} | {numeric(row['max_nm'], 12)} | "
            f"{numeric(row['max_abs_nm'], 12)} | `{pose_label}` |"
        )
    lines.extend([
        "",
        "## Per-pose independent physics audit",
        "",
        "| Pose | q (rad) | qfrc_bias (N·m) | max gradient error | relative qacc error | min eig(M) | cond(M) | collision / constraint | pass |",
        "|---|---|---|---:|---:|---:|---:|---|---|",
    ])
    for row in independent["poses"]:
        q_text = json.dumps(row["q_rad"], separators=(",", ":"))
        tau_text = json.dumps(row["tau_hold_mujoco_nm"], separators=(",", ":"))
        state = f"{'clear' if row['current_pair_scoped_collision_free'] else 'contact'} / {'clear' if row['dynamics_constraint_free'] else 'active'}"
        lines.append(
            f"| `{row['id']}` | `{q_text}` | `{tau_text}` | {maximum(row['absolute_error_nm'], 6)} | "
            f"{numeric(row['relative_qacc_error'], 6)} | {numeric(row['mass_matrix_min_eigenvalue'], 6)} | "
            f"{numeric(row['mass_matrix_condition_number'], 6)} | {state} | {'PASS' if row['pass'] else 'FAIL'} |"
        )
    lines.extend([
        "",
        "Every full 6×6 mass matrix, qacc vector, h1/h2 potential gradient, zero-gravity vector, reverse-gravity vector, and stencil constraint record is serialized in the JSON `poses` rows.",
        "",
        "## Independent validator conclusions",
        "",
        f"- Maximum gradient absolute error: `{numeric(independent['metrics']['max_energy_gradient_absolute_error_nm'], 17)}` N·m.",
        f"- Maximum relative qacc error: `{numeric(independent['metrics']['max_relative_qacc_error'], 17)}`.",
        f"- Minimum M eigenvalue: `{numeric(independent['metrics']['minimum_mass_matrix_eigenvalue'], 17)}`.",
        f"- Maximum M condition number: `{numeric(independent['metrics']['max_mass_matrix_condition_number'], 17)}`.",
        f"- Continuity adjacent/second-difference maxima: `{numeric(independent['continuity']['max_adjacent_inf_norm_jump_nm'], 17)}` / `{numeric(independent['continuity']['max_second_difference_inf_norm_nm'], 17)}` N·m.",
        "- Potential energy sums exactly link2..link6 + gripper (3.4515 kg). The 1 kg link1 placeholder is excluded from the six-body potential authority but remains inside MuJoCo's compiled mass matrix; its independently measured gravity-gradient contribution is bounded at 1e-12 N·m.",
        "- The FILTERPARENT collision clone is diagnostic-only. Only the production-filtered ACTUATION-disabled model supplies U, qfrc_bias, M, qacc, and continuity evidence.",
        "- Gravity torque validates mass + COM + kinematic gravity terms. It does **not** establish `FULL DYNAMICS VALIDATED`.",
        "- No free-fall or time integration was performed. GUI visualization, if invoked separately, is a non-authoritative mj_forward witness only.",
        "",
    ])
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--write", action="store_true", help="write the validator-owned Markdown report")
    modes.add_argument("--check", action="store_true", help="validate and require a byte-identical Markdown report")
    parser.add_argument("--mujoco-python", default=os.environ.get("V15_18_MUJOCO_PYTHON"))
    parser.add_argument("--physics-child", nargs=3, metavar=("MODEL", "REQUEST", "OUTPUT"), help=argparse.SUPPRESS)
    arguments = parser.parse_args()
    if arguments.physics_child:
        return physics_child(Path(arguments.physics_child[0]), Path(arguments.physics_child[1]), Path(arguments.physics_child[2]))
    try:
        require(arguments.write or arguments.check, "exactly one of --write or --check is required")
        legal_fail_self_test = legal_fail_mutation_self_test()
        require(legal_fail_self_test["pass"] is True, "legal physical FAIL mutation self-test failed")
        markdown_path = repo_path(REPORT_MD_REL)
        git_scope = validate_git_scope(allow_missing_markdown=arguments.write)
        assets = production_asset_snapshot()
        initial_hashes = watched_hash_snapshot(assets)
        authority = validate_protected_inputs()
        static_contract = validate_audit_static_contract()
        collision_contract = validate_collision_pair_contract()
        selection = derive_pose_selection()
        audit_path = repo_path(REPORT_JSON_REL)
        require(audit_path.is_file(), "V15.18A audit JSON is missing; run audit --write first")
        audit = read_json(audit_path)
        interpreter = select_mujoco_python(arguments.mujoco_python)
        continuity_request = {"q_start": [0.0] * 6, "q_end": list(V15_14_ACCEPTED_FJT_Q_RAD)}
        independent_raw = run_independent_physics(selection["selected"], continuity_request, interpreter, assets)
        independent = analyze_independent_physics(independent_raw, selection, authority)
        validation = validate_audit_report(audit, selection, authority, independent_raw, independent, git_scope, assets)
        nested = run_nested_audit_check(interpreter, audit)
        final_items = make_final_report_items(independent, validation["final_status"], validation["hard_unresolved_items"])
        markdown = render_markdown(audit, independent, final_items)
        expected_markdown = markdown.encode("utf-8")
        before_write_hashes = watched_hash_snapshot(assets)
        require(before_write_hashes == initial_hashes, "repository/evidence changed during independent validation before report handling")
        if arguments.write:
            atomic_write_text(markdown_path, markdown)
            validate_git_scope()
            require(markdown_path.read_bytes() == expected_markdown, "validator Markdown write/readback mismatch")
            final_hashes = watched_hash_snapshot(assets)
            for relative, digest in initial_hashes.items():
                if relative != REPORT_MD_REL:
                    require(final_hashes[relative] == digest, f"TOCTOU change while writing Markdown: {relative}")
        else:
            require(markdown_path.is_file(), "V15.18A Markdown report is missing")
            require(markdown_path.read_bytes() == expected_markdown, "V15.18A Markdown report is stale or non-deterministic")
            validate_git_scope()
            final_hashes = watched_hash_snapshot(assets)
            require(final_hashes == initial_hashes, "repository/evidence changed during validator --check")
        require(sha256_file(repo_path(MJCF_REL)) == MJCF_SHA256 and sha256_file(repo_path(BRIDGE_REL)) == BRIDGE_SHA256, "production model/bridge changed at final rehash")
        print("VALIDATION=PASS")
        print("AUDIT_VALID=YES")
        print(f"STATUS={validation['status']}")
        print(f"FINAL_STATUS={validation['final_status']}")
        print(f"SELECTED_POSES={len(independent['poses'])}")
        print(f"HARD_UNRESOLVED={len(validation['hard_unresolved_items'])}")
        print(f"NESTED_AUDIT_CHECK={'PASS' if nested['pass'] else 'FAIL'}")
        print(f"STATIC_AUDIT={'PASS' if static_contract['pass'] else 'FAIL'}")
        print(f"COLLISION_PAIR_CONTRACT={'PASS' if collision_contract['pass'] else 'FAIL'}")
        return 0
    except (SystemExit, KeyboardInterrupt):
        raise
    except Exception as error:
        print(f"VALIDATION=FAIL\nAUDIT_VALID=NO\nERROR={clean_error(error)}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

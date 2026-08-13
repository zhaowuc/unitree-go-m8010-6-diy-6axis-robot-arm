#!/usr/bin/env python3
from __future__ import annotations

"""Standalone V15.17D production-hash and trajectory-loop auditor.

This program deliberately does not import the V15.17 deployment tool, the
V15.14 trajectory QA tool, or the 503-pose tool.  It independently checks the
repository contract and recomputes acceptance facts from hash-bound raw
runtime artifacts supplied in a repository-external evidence envelope.

``--write`` writes deterministic, audit-valid JSON and Markdown reports.
``--check`` regenerates them in memory and requires byte identity.  An honest
runtime or acceptance failure is a valid audited FAIL and exits zero; malformed
or stale reports and internal contract errors exit two.
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
from typing import Any, Callable, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
BASE_COMMIT = "75f70ea4527dc90125a13c6d89efd789239367d0"
OLD_MODEL_COMMIT = "f4132c5514a67b5fc36ed63c7474b33a87fc4747"
BASE_BRANCH = "agent/v15-17-inertial-deployment"
TARGET_BRANCH = "agent/v15-17-production-hash-migration"
REMOTE_BASE_REF = f"refs/heads/{BASE_BRANCH}"

V14_REL = "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
MJCF_REL = V14_REL + "/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
XACRO_REL = V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_description/urdf/go_m8010_arm_v15_14.urdf.xacro"
SRDF_REL = V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config/go_m8010_arm_v15_14.srdf"
BRIDGE_REL = (
    V14_REL
    + "/ros2_ws/src/go_m8010_arm_mujoco_bridge/scripts/mujoco_bridge.py"
)
QA_TOOL_REL = (
    V14_REL
    + "/ros2_ws/src/go_m8010_arm_v15_14_qa/scripts/v15_14_regression.py"
)
CROSS_503_REL = V14_REL + "/tools/cross_validate_503_collisions.py"
TARGET_MANIFEST_REL = (
    V14_REL
    + "/ros2_ws/src/go_m8010_arm_v15_14_qa/config/target_manifest_v15_14.json"
)
COLLISION_CONTRACT_REL = V14_REL + "/config/collision_pair_contract_v15_14.json"
GUARD_REL = V14_REL + "/mujoco_v15_14/kinematic_guard.py"
RUNTIME_MESH_MANIFEST_REL = "mujoco_kinematic_v1/runtime_mesh_manifest.json"
SOURCE_MESH_MANIFEST_REL = "mujoco_kinematic_v1/mesh_export_manifest.json"
POSE_SOURCE_VALIDATOR_REL = "mujoco_kinematic_v1/validate_model.py"
INERTIA_AUTHORITY_REL = "V15_16_刚体惯量_Engineering_V1.json"
V15_17C_JSON_REL = "V15_17_Inertial参数部署验收.json"
V15_17C_MD_REL = "V15_17_Inertial参数部署验收.md"
VALIDATOR_REL = "tools/validate_v15_17_production_hash_migration.py"
REPORT_JSON_REL = "V15_17_ProductionHash迁移与轨迹闭环验收.json"
REPORT_MD_REL = "V15_17_ProductionHash迁移与轨迹闭环验收.md"

OLD_MJCF_SHA256 = "ce6ec828d32f445ff25fb4b3ba3167f7f0bec61cdfca5962ee4072df4b022844"
NEW_MJCF_SHA256 = "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
BASE_BRIDGE_SHA256 = "b4c932db2a98bc6361c56c533adcacfd68b4d37dca33ddfaa01a82f2a8e90fbd"
QA_TOOL_SHA256 = "451ac2e5f9d26baf3230b99830203f8cfa4c6054f16afa221a7551b1c9be0575"
CROSS_503_SHA256 = "514486ffaba6405007dd5f80cc918de5bac421de7f8599a625a945f559fdd7b3"
PATCHED_QA_TOOL_SHA256 = "5c2267b98a78dec01f05c980ba748f02196750d9d01804f70214786a85e85af5"
PATCHED_CROSS_503_SHA256 = "f942094ec7e6420b9d3332ac7551a1c703f5664f578189d15550aed92cd7b0f1"
POSE_SOURCE_VALIDATOR_SHA256 = "9be9fab3e5c7850f001c2e46b83aebf033a02feff2a9e27dbc15347db4098ea4"
TARGET_MANIFEST_SHA256 = "f2b230e2cb61c251c81228f5f3b89ced0163b05cf82f0b93db8fd0bb85572d57"
COLLISION_CONTRACT_SHA256 = "6d802909e44f238816f007ef33b57e1b57099c5522a473cd2dcc2705397af9c9"
GUARD_SHA256 = "648b9de953a84be46cd5f95e24daf9f683f3025e73048dc068ef7bd8e5670c6a"
RUNTIME_MESH_MANIFEST_SHA256 = "1174abcfef3b87ee77d4ce50af5afc1aa62d45759f1560ace361997e8c60b19d"
SOURCE_MESH_MANIFEST_SHA256 = "22cfb9a194495b442731405a023cec8f83fb03d17b2e814cf13febc924d1ce3a"
RUNTIME_ASSET_SET_SHA256 = "fe517900413488414f9a606d7fd8f05ea8a7f2be08043672e75c57ae669f0425"
INERTIA_AUTHORITY_SHA256 = "c6c398532d7144ed6aa340a8d8b765b333d01f1fddc2e3280106c90565614401"
V15_17C_JSON_SHA256 = "4de6e5905c049eebd8854c366fd04a16eb7d1baa9068727d0627503c2400f322"
V15_17C_MD_SHA256 = "b03d35856cf0fc0303d486a6c9e270cbbec451f95d4b078bbb474f7deec217c3"
FROZEN_POSE_SET_SHA256 = "4d9cfaf15cc20fe16393d1aa3bccddc5273e77e2de08cf9d0045976c301799aa"
OLD_XACRO_GIT_LF_SHA256 = "f9c15d366e27bef76eeafd44efcca0de96d2f4e944584fcc74c9764a198a914c"
BASE_XACRO_GIT_LF_SHA256 = "aef7d9b23c41a31456fd138c7abfb22b6799f86bcdb26fd786c4042664601433"
CURRENT_XACRO_CHECKOUT_SHA256 = "1ba9d2ec1126371f676a55a33c05c61a5bc416651e963e350f28662cfd141652"
SRDF_GIT_LF_SHA256 = "433284326632802fbeb7f1e5eda78d6d5ed5a9e4b006f08f61c6419e3c3f0222"
SRDF_CHECKOUT_SHA256 = "ac45155004b4b86e8de51727d750aa26722a9a9dc706a93721d3a81faf75a862"

SCHEMA = "go-m8010-arm-v15.17d-production-hash-trajectory-acceptance/1.0"
REVISION = "V15.17D-PRODUCTION_HASH_MIGRATION_AND_V15.14_TRAJECTORY_LOOP"
RUNTIME_EVIDENCE_SCHEMA = "go-m8010-arm-v15.17d-production-runtime-evidence/1.0"
FINAL_PASS_STATUS = "V15.17 INERTIAL_DEPLOYMENT = PASS"
FINAL_FAIL_STATUS = "V15.17 INERTIAL_DEPLOYMENT = FAIL"
VISUAL_LIMITATION = "NON_PHYSICAL_VISUAL_NUMERICAL_LIMITATION"

LINKS = ("link2", "link3", "link4", "link5", "link6", "gripper")
JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")
EXACT_CHANGED_PATHS = {BRIDGE_REL, VALIDATOR_REL, REPORT_JSON_REL, REPORT_MD_REL}
PREWRITE_CHANGED_PATHS = {BRIDGE_REL, VALIDATOR_REL}

EXPECTED_MOTION_PROXY_SHA256 = {
    "meshes/collision_motion_proxy_mm/link2/UpperArm_Motion_Collision_Proxy__component_001.stl":
        "1f3b3c3878b93c450c9c90590542174e18259902b22eb81dc8a427563ee3f555",
    "meshes/collision_motion_proxy_mm/link2/UpperArm_Motion_Collision_Proxy__component_002.stl":
        "11fb6c54f4d972e7c13b86167ea7cb1987354fa59defce43a0d9d4ff9f0ccf2a",
}

EXPECTED_QA_THRESHOLDS = {
    "path_sample_max_joint_step_rad": 0.004363323129985824,
    "final_joint_max_abs_error_rad": 1.0e-6,
    "final_tcp_position_error_m": 1.0e-4,
    "final_tcp_orientation_error_rad": 1.0e-3,
    "trajectory_tracking_max_abs_error_rad": 0.05,
    "trajectory_velocity_tracking_max_abs_error_rad_s": 0.10,
    "max_controller_state_gap_s": 0.10,
    "max_raw_state_gap_s": 0.05,
    "max_public_state_gap_s": 0.05,
    "public_raw_joint_sync_rad": 1.0e-6,
    "final_public_raw_joint_sync_rad": 1.0e-6,
    "public_raw_joint_velocity_sync_rad_s": 0.05,
    "max_state_pair_time_skew_s": 0.06,
    "tf_mujoco_tcp_position_m": 1.0e-8,
    "tf_mujoco_tcp_orientation_rad": 1.0e-7,
    "negative_no_execution_joint_excursion_rad": 1.0e-9,
}

REQUIRED_RUNTIME_ARTIFACTS = {
    "colcon_build_log",
    "installed_bridge",
    "bringup_log",
    "bridge_status_json",
    "trajectory_qa_json",
    "collision_503_json",
    "six_body_readback_json",
    "controller_graph_json",
    "tf_geometry_json",
    "runtime_robot_description_txt",
    "trajectory_qa_tool",
    "collision_503_tool",
    "trajectory_qa_attempt1_json",
    "controllers_post_txt",
    "actions_post_txt",
    "nodes_post_txt",
    "joint_states_info_post_txt",
}


PROTECTED_RELS = {
    MJCF_REL,
    QA_TOOL_REL,
    CROSS_503_REL,
    TARGET_MANIFEST_REL,
    COLLISION_CONTRACT_REL,
    GUARD_REL,
    RUNTIME_MESH_MANIFEST_REL,
    SOURCE_MESH_MANIFEST_REL,
    POSE_SOURCE_VALIDATOR_REL,
    INERTIA_AUTHORITY_REL,
    V15_17C_JSON_REL,
    V15_17C_MD_REL,
    XACRO_REL,
    SRDF_REL,
    V14_REL + "/config/accepted_frozen_contract_v15_13.json",
    V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config/joint_limits.yaml",
    V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config/kinematics.yaml",
    V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config/moveit_controllers.yaml",
    V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config/ompl_planning.yaml",
    V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config/ros2_controllers.yaml",
    V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/launch/v15_14_bringup.launch.py",
}

GIT_EXECUTABLE = os.environ.get("V15_17_GIT_EXECUTABLE") or shutil.which("git")


class ValidationError(RuntimeError):
    """Malformed/schema/identity evidence: propagate as audit-invalid exit two."""


class StaleReportError(RuntimeError):
    """A stale/malformed report error that must return exit code two."""


class GateFailure(RuntimeError):
    """A genuine acceptance failure that can be an audit-valid exit-zero FAIL."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def gate_require(condition: bool, message: str) -> None:
    if not condition:
        raise GateFailure(message)


def repo_path(relative: str) -> Path:
    parts = PurePosixPath(relative).parts
    require(bool(parts) and ".." not in parts, f"unsafe repository path: {relative}")
    path = (ROOT / Path(*parts)).resolve()
    require(ROOT == path or ROOT in path.parents, f"repository path escaped root: {relative}")
    return path


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    value = json.loads(path.read_text(encoding="utf-8"))
    require(isinstance(value, dict), f"JSON root is not an object: {path.name}")
    return value


def finite(value: Any, label: str) -> float:
    require(isinstance(value, (int, float)) and not isinstance(value, bool), f"{label}: numeric value required")
    result = float(value)
    require(math.isfinite(result), f"{label}: finite value required")
    return result


def vector(value: Any, length: int, label: str) -> list[float]:
    require(isinstance(value, list) and len(value) == length, f"{label}: expected {length} values")
    return [finite(item, f"{label}[{index}]") for index, item in enumerate(value)]


def max_abs(values: Iterable[float]) -> float:
    return max((abs(float(value)) for value in values), default=0.0)


def matrix3(value: Any, label: str) -> list[list[float]]:
    require(isinstance(value, list) and len(value) == 3, f"{label}: 3x3 matrix required")
    return [vector(row, 3, f"{label}[{index}]") for index, row in enumerate(value)]


def transpose(matrix: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[float(matrix[column][row]) for column in range(3)] for row in range(3)]


def matmul(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[sum(float(left[row][inner]) * float(right[inner][column]) for inner in range(3)) for column in range(3)] for row in range(3)]


def quaternion_wxyz_to_matrix(value: Sequence[float]) -> list[list[float]]:
    w, x, y, z = (float(item) for item in value)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    require(norm > 0.0, "zero quaternion")
    w, x, y, z = (item / norm for item in (w, x, y, z))
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def reconstruct_tensor(quaternion_wxyz: Sequence[float], diagonal: Sequence[float]) -> list[list[float]]:
    rotation = quaternion_wxyz_to_matrix(quaternion_wxyz)
    diagonal_matrix = [[float(diagonal[row]) if row == column else 0.0 for column in range(3)] for row in range(3)]
    return matmul(matmul(rotation, diagonal_matrix), transpose(rotation))


def relative_frobenius(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> float:
    numerator = math.sqrt(sum((float(left[row][column]) - float(right[row][column])) ** 2 for row in range(3) for column in range(3)))
    denominator = math.sqrt(sum(float(right[row][column]) ** 2 for row in range(3) for column in range(3)))
    require(denominator > 0.0, "reference tensor Frobenius norm is zero")
    return numerator / denominator


def sanitize_error(error: BaseException, evidence_root: Path | None = None) -> str:
    text = str(error).replace(str(ROOT), "<REPO_ROOT>")
    if evidence_root is not None:
        text = text.replace(str(evidence_root), "<EVIDENCE_ROOT>")
    return re.sub(r"[\r\n]+", " ", text).strip()


def run_process(
    command: Sequence[str],
    *,
    cwd: Path = ROOT,
    timeout: float = 30.0,
    env: Mapping[str, str] | None = None,
) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        list(command),
        cwd=str(cwd),
        env=dict(env) if env is not None else None,
        text=True,
        encoding="utf-8",
        errors="replace",
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout,
        check=False,
    )


def git_text(arguments: Sequence[str], *, timeout: float = 30.0) -> str:
    require(bool(GIT_EXECUTABLE), "git executable was not found")
    git_path = Path(str(GIT_EXECUTABLE)).resolve()
    require(git_path.is_file(), "git executable does not exist")
    result = run_process([str(git_path), *arguments], timeout=timeout)
    require(result.returncode == 0, "git command failed: " + sanitize_error(result.stderr or result.stdout))
    return result.stdout


def git_blob(relative: str, commit: str) -> bytes:
    require(bool(GIT_EXECUTABLE), "git executable was not found")
    git_path = Path(str(GIT_EXECUTABLE)).resolve()
    result = subprocess.run(
        [str(git_path), "show", f"{commit}:{relative}"],
        cwd=str(ROOT),
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=30.0,
        check=False,
    )
    require(result.returncode == 0, f"cannot read frozen Git blob: {relative}")
    return result.stdout


def python_literal_assignment_bytes(data: bytes, name: str) -> Any:
    tree = ast.parse(data.decode("utf-8"))
    matches: list[Any] = []
    for node in tree.body:
        if not isinstance(node, (ast.Assign, ast.AnnAssign)):
            continue
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        if any(isinstance(target, ast.Name) and target.id == name for target in targets):
            matches.append(ast.literal_eval(node.value))
    require(len(matches) == 1, f"expected exactly one literal assignment for {name}")
    return matches[0]


def canonical_json_digest(value: Any) -> str:
    return sha256_bytes(json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8"))


def current_changed_paths() -> list[str]:
    committed_or_worktree = git_text(["diff", "--name-only", "--diff-filter=ACMRTUXB", BASE_COMMIT, "--"])
    untracked = git_text(["ls-files", "--others", "--exclude-standard"])
    return sorted({line.replace("\\", "/") for text in (committed_or_worktree, untracked) for line in text.splitlines() if line.strip()})


def validate_git_scope(*, allow_prewrite: bool) -> dict[str, Any]:
    branch = git_text(["branch", "--show-current"]).strip()
    head = git_text(["rev-parse", "HEAD"]).strip().lower()
    changed = current_changed_paths()
    expected = PREWRITE_CHANGED_PATHS if allow_prewrite and not repo_path(REPORT_JSON_REL).exists() and not repo_path(REPORT_MD_REL).exists() else EXACT_CHANGED_PATHS
    remote_output = git_text(["ls-remote", "origin", REMOTE_BASE_REF], timeout=60.0).strip()
    remote_rows = [row.split() for row in remote_output.splitlines() if row.strip()]
    require(branch == TARGET_BRANCH, f"target branch mismatch: {branch}")
    if head != BASE_COMMIT:
        parent = git_text(["rev-parse", "HEAD^"]).strip().lower()
        count = int(git_text(["rev-list", "--count", f"{BASE_COMMIT}..HEAD"]).strip())
        require(parent == BASE_COMMIT and count == 1, "HEAD must be the frozen base or its unique direct child")
        require(git_text(["status", "--porcelain=v1"]).strip() == "", "post-commit validation requires a clean worktree")
    require(set(changed) == expected, f"changed path scope mismatch: {changed} != {sorted(expected)}")
    require(len(remote_rows) == 1 and remote_rows[0][0].lower() == BASE_COMMIT, "frozen remote base ref mismatch")
    return {
        "pass": True,
        "branch": branch,
        "source_commit": BASE_COMMIT,
        "head_contract": "BASE_COMMIT_OR_ITS_UNIQUE_DIRECT_CHILD",
        "remote_base_ref": REMOTE_BASE_REF,
        "remote_base_commit": remote_rows[0][0].lower(),
        "changed_paths": changed,
        "exact_changed_path_count": len(changed),
        "allowed_changed_paths": sorted(EXACT_CHANGED_PATHS),
        "prewrite_scope_permitted": expected == PREWRITE_CHANGED_PATHS,
        "_runtime_head_commit": head,
    }


def capture_repo_hashes(relatives: Iterable[str]) -> dict[str, str | None]:
    result: dict[str, str | None] = {}
    for relative in sorted(set(relatives)):
        path = repo_path(relative)
        result[relative] = sha256_file(path) if path.is_file() else None
    return result


def audit_call(code: str, function: Callable[[], dict[str, Any]], evidence_root: Path | None = None) -> dict[str, Any]:
    try:
        result = function()
        require(isinstance(result, dict), f"{code}: audit function returned non-object")
        result.setdefault("pass", True)
        return result
    except GateFailure as error:  # only a genuine acceptance failure is serialized
        return {
            "pass": False,
            "failure_code": code,
            "exception_type": type(error).__name__,
            "reason": sanitize_error(error, evidence_root),
        }


def local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def canonical_element(element: ET.Element, *, inertial_markers: bool = False, current_body: str | None = None) -> Any:
    tag = local_name(element.tag)
    body_context = str(element.get("name")) if tag == "body" and element.get("name") else current_body
    if inertial_markers and tag == "inertial" and current_body in LINKS:
        return ["inertial", "<V15_17_ACCEPTED_INERTIAL_SLOT>"]
    text = (element.text or "").strip()
    tail = (element.tail or "").strip()
    return [
        tag,
        sorted((str(key), str(value)) for key, value in element.attrib.items()),
        text,
        tail,
        [canonical_element(child, inertial_markers=inertial_markers, current_body=body_context) for child in list(element)],
    ]


def body_map(root: ET.Element) -> dict[str, ET.Element]:
    result: dict[str, ET.Element] = {}
    for body in root.findall(".//body"):
        name = body.get("name")
        if name:
            require(name not in result, f"duplicate MJCF body: {name}")
            result[name] = body
    return result


def direct_inertial(body: ET.Element, label: str) -> ET.Element:
    rows = [child for child in list(body) if local_name(child.tag) == "inertial"]
    require(len(rows) == 1, f"{label}: expected exactly one direct inertial")
    return rows[0]


def normalize_xml_attributes(attributes: Mapping[str, str]) -> dict[str, str]:
    return {str(key): str(attributes[key]) for key in sorted(attributes)}


def validate_model_semantics() -> dict[str, Any]:
    old_lf = git_blob(MJCF_REL, OLD_MODEL_COMMIT)
    require(sha256_bytes(old_lf) == "f086d27c61aa3be02234356af6fd8fcf269a3e3b65bdeb08cd04435ba679f0b0", "old MJCF Git blob hash mismatch")
    old_checkout_bytes = old_lf.replace(b"\n", b"\r\n")
    require(sha256_bytes(old_checkout_bytes) == OLD_MJCF_SHA256, "old accepted checkout MJCF SHA256 reconstruction failed")
    new_bytes = repo_path(MJCF_REL).read_bytes()
    require(sha256_bytes(new_bytes) == NEW_MJCF_SHA256, "current production MJCF SHA256 mismatch")
    require(sha256_bytes(git_blob(MJCF_REL, BASE_COMMIT).replace(b"\n", b"\r\n")) == NEW_MJCF_SHA256, "frozen base production MJCF mismatch")

    old_root = ET.fromstring(old_lf)
    new_root = ET.fromstring(new_bytes)
    old_bodies = body_map(old_root)
    new_bodies = body_map(new_root)
    require(set(old_bodies) == set(new_bodies), "MJCF body identity set changed")
    records: dict[str, Any] = {}
    for link in LINKS:
        require(link in old_bodies and link in new_bodies, f"missing target body: {link}")
        old_inertial = direct_inertial(old_bodies[link], f"old {link}")
        new_inertial = direct_inertial(new_bodies[link], f"new {link}")
        old_attrs = normalize_xml_attributes(old_inertial.attrib)
        new_attrs = normalize_xml_attributes(new_inertial.attrib)
        require(set(old_attrs) == {"mass", "pos", "diaginertia"}, f"old {link} inertial representation changed")
        require(set(new_attrs) == {"mass", "pos", "quat", "diaginertia"}, f"new {link} inertial representation mismatch")
        require(old_attrs == {"mass": "1", "pos": "0 0 0", "diaginertia": "0.001 0.001 0.001"}, f"old {link} dummy inertial is not frozen")
        require(float(new_attrs["mass"]) > 0.0, f"{link}: deployed mass is non-positive")
        vector([float(token) for token in new_attrs["pos"].split()], 3, f"{link}.pos")
        vector([float(token) for token in new_attrs["quat"].split()], 4, f"{link}.quat")
        diagonal = vector([float(token) for token in new_attrs["diaginertia"].split()], 3, f"{link}.diaginertia")
        require(all(value > 0.0 for value in diagonal), f"{link}: non-positive principal inertia")
        records[link] = {
            "old_attributes": old_attrs,
            "new_attributes": new_attrs,
            "representation_change": "V15_14_DUMMY_DIAGINERTIA_TO_V15_17_EXPLICIT_PRINCIPAL_FRAME",
        }

    old_without = canonical_element(old_root, inertial_markers=True)
    new_without = canonical_element(new_root, inertial_markers=True)
    require(old_without == new_without, "MJCF semantic diff extends outside inertial elements")
    changed_inertial_bodies = sorted(
        name
        for name in LINKS
        if normalize_xml_attributes(direct_inertial(old_bodies[name], f"old {name}").attrib)
        != normalize_xml_attributes(direct_inertial(new_bodies[name], f"new {name}").attrib)
    )
    require(changed_inertial_bodies == sorted(LINKS), f"unexpected inertial change set: {changed_inertial_bodies}")

    option = new_root.find("./option")
    gravity = [float(token) for token in str(option.get("gravity", "")).split()] if option is not None else []
    require(gravity == [0.0, 0.0, 0.0], f"gravity is not disabled: {gravity}")
    require(option is not None and option.get("timestep") is not None, "MJCF timestep missing")
    return {
        "pass": True,
        "old_accepted_mjcf_sha256": OLD_MJCF_SHA256,
        "old_git_blob_lf_sha256": sha256_bytes(old_lf),
        "old_checkout_crlf_reconstruction": True,
        "new_production_mjcf_sha256": NEW_MJCF_SHA256,
        "semantic_diff_classification": "ONLY_ACCEPTED_INERTIAL_CHANGES",
        "semantic_tree_without_inertials_sha256": canonical_json_digest(new_without),
        "changed_inertial_bodies": changed_inertial_bodies,
        "link_records": records,
        "gravity_vector_m_s2": gravity,
        "gravity_enabled": False,
        "timestep_s": finite(float(option.get("timestep")), "MJCF timestep"),
    }


BRIDGE_PROTECTED_ASSIGNMENTS = (
    "ACCEPTED_COLLISION_CONTRACT_SHA256",
    "ACCEPTED_GUARD_SHA256",
    "ACCEPTED_RUNTIME_MESH_MANIFEST_SHA256",
    "ACCEPTED_RUNTIME_ASSET_SET_SHA256",
    "ACCEPTED_SOURCE_MESH_MANIFEST_SHA256",
    "MOTION_PROXY_MESH_SHA256",
)


def validate_bridge_migration() -> dict[str, Any]:
    baseline = git_blob(BRIDGE_REL, BASE_COMMIT)
    current = repo_path(BRIDGE_REL).read_bytes()
    require(sha256_bytes(baseline) == BASE_BRIDGE_SHA256, "baseline bridge hash mismatch")
    require(current.count(NEW_MJCF_SHA256.encode("ascii")) == 1, "new production hash must occur exactly once in bridge")
    require(current.count(OLD_MJCF_SHA256.encode("ascii")) == 0, "old production hash remains in bridge")
    restored = current.replace(NEW_MJCF_SHA256.encode("ascii"), OLD_MJCF_SHA256.encode("ascii"))
    require(restored == baseline, "bridge differs from baseline beyond ACCEPTED_MODEL_SHA256")
    old_anchor = python_literal_assignment_bytes(baseline, "ACCEPTED_MODEL_SHA256")
    new_anchor = python_literal_assignment_bytes(current, "ACCEPTED_MODEL_SHA256")
    require(old_anchor == OLD_MJCF_SHA256 and new_anchor == NEW_MJCF_SHA256, "bridge model hash anchors mismatch")
    protected: dict[str, Any] = {}
    for name in BRIDGE_PROTECTED_ASSIGNMENTS:
        before = python_literal_assignment_bytes(baseline, name)
        after = python_literal_assignment_bytes(current, name)
        require(before == after, f"protected bridge constant changed: {name}")
        protected[name] = after
    return {
        "pass": True,
        "migration_kind": "ONE_LITERAL_PRODUCTION_MODEL_HASH_ANCHOR_ONLY",
        "modified_hash_anchors": ["ACCEPTED_MODEL_SHA256"],
        "old_accepted_model_sha256": old_anchor,
        "new_accepted_model_sha256": new_anchor,
        "baseline_bridge_sha256": BASE_BRIDGE_SHA256,
        "current_bridge_sha256": sha256_bytes(current),
        "reverse_substitution_is_byte_exact": True,
        "protected_assignments": protected,
    }


def classify_old_hash_reference(relative: str) -> str:
    if relative == BRIDGE_REL:
        return "PRODUCTION_RUNTIME_ANCHOR"
    if relative in {QA_TOOL_REL, CROSS_503_REL}:
        return "V15_14_TEST_FIXTURE_ANCHOR"
    if relative == V14_REL + "/tools/aggregate_v15_14_acceptance.py":
        return "V15_14_TEST_FIXTURE_ANCHOR"
    if relative.startswith("tools/"):
        return "AUDIT_VALIDATOR_OR_DEPLOYMENT_CONTRACT"
    if relative.lower().endswith((".json", ".md", ".txt")):
        return "HISTORICAL_AUDIT_EVIDENCE"
    if "/evidence/" in relative:
        return "HISTORICAL_AUDIT_EVIDENCE"
    return "UNCLASSIFIED"


def validate_old_hash_references() -> dict[str, Any]:
    candidates = git_text(["ls-files", "--cached", "--others", "--exclude-standard"]).splitlines()
    rows: list[dict[str, str]] = []
    for raw in sorted(set(candidates)):
        relative = raw.replace("\\", "/")
        if relative in {REPORT_JSON_REL, REPORT_MD_REL}:
            continue
        path = repo_path(relative)
        if not path.is_file() or path.stat().st_size > 20_000_000:
            continue
        data = path.read_bytes()
        if OLD_MJCF_SHA256.encode("ascii") not in data:
            continue
        category = classify_old_hash_reference(relative)
        rows.append({"path": relative, "classification": category})
    require(all(row["classification"] != "UNCLASSIFIED" for row in rows), "unclassified old model hash reference exists")
    require(not any(row["path"] == BRIDGE_REL for row in rows), "production bridge still references old model hash")
    fixture_paths = {row["path"] for row in rows if row["classification"] == "V15_14_TEST_FIXTURE_ANCHOR"}
    require(fixture_paths == {QA_TOOL_REL, CROSS_503_REL, V14_REL + "/tools/aggregate_v15_14_acceptance.py"}, "V15.14 original tool anchor classification changed")
    return {
        "pass": True,
        "searched_old_sha256": OLD_MJCF_SHA256,
        "reference_count": len(rows),
        "references": rows,
        "production_runtime_old_reference_count": 0,
        "original_tools_preserved_as_fixtures": sorted(fixture_paths),
    }


def validate_collision_contract() -> dict[str, Any]:
    path = repo_path(COLLISION_CONTRACT_REL)
    require(sha256_file(path) == COLLISION_CONTRACT_SHA256, "collision contract SHA256 mismatch")
    contract = read_json(path)
    require(contract.get("schema") == "go-m8010-arm-v15.14-self-collision-pair-contract/2.0", "collision contract schema mismatch")
    proxies = [str(value) for value in contract.get("proxies", [])]
    require(len(proxies) == 25 and len(set(proxies)) == 25, "collision proxy identity contract mismatch")
    all_pairs = {tuple(sorted((left, right))) for index, left in enumerate(proxies) for right in proxies[index + 1:]}
    full_pairs = {tuple(sorted(map(str, row))) for row in contract.get("runtime_full_pairs", [])}
    excluded_pairs = {tuple(sorted(map(str, row))) for row in contract.get("runtime_excluded_pairs", [])}
    require(len(all_pairs) == 300, "collision all-pair count mismatch")
    require(len(full_pairs) == 231 and len(excluded_pairs) == 69, "collision partition counts mismatch")
    require(not (full_pairs & excluded_pairs) and full_pairs | excluded_pairs == all_pairs, "collision pair partition is not exact")
    require(contract.get("proxy_count") == 25 and contract.get("all_unordered_pair_count") == 300, "collision declared counts mismatch")
    require(contract.get("runtime_full_pair_count") == 231 and contract.get("runtime_excluded_pair_count") == 69, "collision declared runtime counts mismatch")
    return {
        "pass": True,
        "sha256": COLLISION_CONTRACT_SHA256,
        "schema": contract["schema"],
        "proxy_count": len(proxies),
        "all_unordered_pair_count": len(all_pairs),
        "runtime_full_pair_count": len(full_pairs),
        "runtime_excluded_pair_count": len(excluded_pairs),
        "partition_exact": True,
    }


def validate_mesh_authority() -> tuple[dict[str, Any], dict[str, str]]:
    model_path = repo_path(MJCF_REL)
    root = ET.parse(model_path).getroot()
    mesh_nodes = root.findall("./asset/mesh")
    external = [element for element in root.iter() if element.get("file")]
    require(len(mesh_nodes) == 1008 and len(external) == 1008, "expected exactly 1008 external mesh assets")
    require(all(local_name(element.tag) == "mesh" for element in external), "non-mesh external MJCF asset exists")
    names = [str(node.get("name")) for node in mesh_nodes]
    literals = [str(node.get("file")) for node in mesh_nodes]
    require(len(set(names)) == 1008 and all(names), "mesh asset names are not unique/complete")
    resolved = [(model_path.parent / PurePosixPath(value)).resolve() for value in literals]
    require(len(set(resolved)) == 1008, "duplicate mesh file references exist")
    uses = {name: 0 for name in names}
    for geom in root.findall(".//geom"):
        mesh = geom.get("mesh")
        if mesh:
            require(mesh in uses, f"geom references undefined mesh: {mesh}")
            uses[mesh] += 1
    require(all(count == 1 for count in uses.values()), "every accepted mesh must be used exactly once")

    runtime_manifest_path = repo_path(RUNTIME_MESH_MANIFEST_REL)
    source_manifest_path = repo_path(SOURCE_MESH_MANIFEST_REL)
    require(sha256_file(runtime_manifest_path) == RUNTIME_MESH_MANIFEST_SHA256, "runtime mesh manifest SHA256 mismatch")
    require(sha256_file(source_manifest_path) == SOURCE_MESH_MANIFEST_SHA256, "source mesh manifest SHA256 mismatch")
    manifest = read_json(runtime_manifest_path)
    require(manifest.get("schema") == "go-m8010-arm-v15.13-mujoco-runtime-meshes/1.0", "runtime mesh schema mismatch")
    require(manifest.get("revision") == "V15.13-MuJoCo-empty-load-kinematic-v1", "runtime mesh revision mismatch")
    require(manifest.get("source_manifest") == "mesh_export_manifest.json", "runtime mesh source manifest path mismatch")
    require(str(manifest.get("source_manifest_sha256", "")).lower() == SOURCE_MESH_MANIFEST_SHA256, "runtime manifest source hash mismatch")

    original_root = runtime_manifest_path.parent
    expected: dict[Path, tuple[str, str]] = {}
    visual_count = 0
    collision_count = 0
    for row in manifest.get("visual", {}).values():
        for chunk in row.get("runtime_chunks", []):
            path = (original_root / PurePosixPath(str(chunk["path"]))).resolve()
            expected[path] = (str(chunk["sha256"]).lower(), "v15_13_visual")
            visual_count += 1
    for row in manifest.get("collision", {}).values():
        for member in row.get("members", []):
            for piece in member.get("pieces", []):
                path = (original_root / PurePosixPath(str(piece["path"]))).resolve()
                expected[path] = (str(piece["sha256"]).lower(), "v15_13_collision")
                collision_count += 1
    for relative, digest in EXPECTED_MOTION_PROXY_SHA256.items():
        expected[(model_path.parent / PurePosixPath(relative)).resolve()] = (digest, "v15_14_motion")
    require(visual_count == 45 and collision_count == 961 and len(expected) == 1008, "manifest mesh counts mismatch")
    require(set(resolved) == set(expected), "MJCF mesh reference set differs from authority manifests")

    node_by_path = {(model_path.parent / PurePosixPath(str(node.get("file")))).resolve(): node for node in mesh_nodes}
    total_bytes = 0
    records: list[tuple[str, str, str, str, int, str]] = []
    asset_hash_snapshot: dict[str, str] = {}
    for path in sorted(expected, key=lambda item: str(item)):
        require(path.is_file(), f"mesh asset missing: {path.name}")
        digest = sha256_file(path)
        required_digest, role = expected[path]
        require(digest == required_digest, f"mesh SHA256 mismatch: {path.name}")
        node = node_by_path[path]
        literal = str(node.get("file")).replace("\\", "/")
        scale = str(node.get("scale", ""))
        required_scale = "0.001 0.001 0.001" if role in {"v15_13_visual", "v15_14_motion"} else ""
        require(scale == required_scale, f"mesh scale mismatch: {literal}")
        size = path.stat().st_size
        total_bytes += size
        records.append((str(node.get("name")), literal, role, scale, size, digest))
        relative_snapshot = path.relative_to(ROOT).as_posix()
        asset_hash_snapshot[relative_snapshot] = digest
    require(total_bytes == 383_827_772, "runtime mesh byte total mismatch")
    digest_state = hashlib.sha256()
    digest_state.update(b"go-m8010-arm-v15.14-runtime-asset-set/1.0\0")
    for name, literal, role, scale, size, digest in sorted(records, key=lambda row: row[0]):
        digest_state.update((name + "\0" + literal + "\0" + role + "\0" + scale + "\0" + str(size) + "\0" + digest + "\n").encode("utf-8"))
    asset_set_sha = digest_state.hexdigest()
    require(asset_set_sha == RUNTIME_ASSET_SET_SHA256, "runtime mesh asset-set digest mismatch")

    geom_names = {str(geom.get("name")) for geom in root.findall(".//geom") if geom.get("name")}
    motion_geoms = sorted(name for name in geom_names if "UpperArm_Motion_Collision_Proxy" in name)
    j1_geoms = sorted(name for name in geom_names if "J1_Fixed_Collision_Proxy" in name or "J1_Moving_Collision_Proxy" in name)
    expected_pairs = {tuple(sorted((motion, other))) for motion in motion_geoms for other in j1_geoms}
    actual_pairs = {tuple(sorted((str(pair.get("geom1")), str(pair.get("geom2"))))) for pair in root.findall("./contact/pair")}
    require(len(motion_geoms) == 2 and len(j1_geoms) == 41 and len(expected_pairs) == 82, "explicit-pair source geometry counts mismatch")
    require(actual_pairs == expected_pairs, "explicit contact pair set changed")
    return {
        "pass": True,
        "mesh_count": len(expected),
        "visual_chunk_count": visual_count,
        "collision_piece_count": collision_count,
        "motion_proxy_piece_count": len(EXPECTED_MOTION_PROXY_SHA256),
        "total_bytes": total_bytes,
        "runtime_manifest_sha256": RUNTIME_MESH_MANIFEST_SHA256,
        "source_manifest_sha256": SOURCE_MESH_MANIFEST_SHA256,
        "runtime_asset_set_sha256": asset_set_sha,
        "every_asset_hashed": True,
        "every_asset_used_exactly_once": True,
        "explicit_contact_pair_count": len(actual_pairs),
        "explicit_contact_pair_set_sha256": canonical_json_digest(sorted(actual_pairs)),
    }, asset_hash_snapshot


def validate_prior_v15_17c() -> dict[str, Any]:
    json_path = repo_path(V15_17C_JSON_REL)
    md_path = repo_path(V15_17C_MD_REL)
    require(sha256_file(json_path) == V15_17C_JSON_SHA256, "V15.17C JSON hash mismatch")
    require(sha256_file(md_path) == V15_17C_MD_SHA256, "V15.17C Markdown hash mismatch")
    report = read_json(json_path)
    require(report.get("audit_valid") is True and report.get("pass") is True, "V15.17C report is not audit-valid PASS")
    require(report.get("final_status") == "V15.17 INERTIAL_DEPLOYMENT = PASS_WITH_NONPHYSICAL_VISUAL_LIMITATION", "V15.17C final status mismatch")
    require(report.get("hard_unresolved_items") == [], "V15.17C hard unresolved list is non-empty")
    facts = report.get("final_facts", {})
    require(facts.get("artifact_classification", {}).get("limitation") == VISUAL_LIMITATION, "V15.17C visual limitation identity mismatch")
    require(facts.get("artifact_classification", {}).get("case") == "CASE_2", "V15.17C sameframe case mismatch")
    require(facts.get("collision_geoms_unchanged") == "YES", "V15.17C collision geometry result changed")
    require(facts.get("joint_tf_unchanged") == "YES", "V15.17C joint/TF result changed")
    require(facts.get("gravity_enabled") == "NO", "V15.17C gravity result changed")
    return {
        "pass": True,
        "json_sha256": V15_17C_JSON_SHA256,
        "markdown_sha256": V15_17C_MD_SHA256,
        "case": "CASE_2",
        "limitation": VISUAL_LIMITATION,
        "nonblocking": True,
        "maximum_runtime_position_delta_m": facts["artifact_classification"]["visual_delta_scale"]["maximum_runtime_position_delta_m"],
        "maximum_runtime_rotation_delta_rad": 0.0,
    }


def validate_static_protected_files() -> dict[str, Any]:
    require(sha256_file(repo_path(GUARD_REL)) == GUARD_SHA256, "kinematic guard SHA256 mismatch")
    require(sha256_file(repo_path(QA_TOOL_REL)) == QA_TOOL_SHA256, "original trajectory QA tool changed")
    require(sha256_file(repo_path(CROSS_503_REL)) == CROSS_503_SHA256, "original 503 tool changed")
    require(sha256_file(repo_path(POSE_SOURCE_VALIDATOR_REL)) == POSE_SOURCE_VALIDATOR_SHA256, "frozen 503 pose-source validator changed")
    require(sha256_file(repo_path(INERTIA_AUTHORITY_REL)) == INERTIA_AUTHORITY_SHA256, "inertia authority changed")
    changed = set(current_changed_paths())
    modified_protected = sorted(PROTECTED_RELS & changed)
    require(not modified_protected, f"protected repository inputs changed: {modified_protected}")
    hashes = capture_repo_hashes(PROTECTED_RELS)
    require(all(value is not None for value in hashes.values()), "protected input missing")
    return {
        "pass": True,
        "protected_path_count": len(hashes),
        "protected_path_set_sha256": canonical_json_digest(hashes),
        "modified_protected_paths": modified_protected,
        "kinematic_guard_sha256": GUARD_SHA256,
        "original_trajectory_qa_tool_sha256": QA_TOOL_SHA256,
        "original_collision_503_tool_sha256": CROSS_503_SHA256,
        "frozen_pose_source_validator_sha256": POSE_SOURCE_VALIDATOR_SHA256,
        "inertia_authority_sha256": INERTIA_AUTHORITY_SHA256,
    }


def safe_evidence_path(root: Path, literal: Any, label: str) -> tuple[Path, str]:
    require(isinstance(literal, str) and literal.strip(), f"{label}: non-empty relative path required")
    normalized = literal.replace("\\", "/")
    pure = PurePosixPath(normalized)
    require(not pure.is_absolute() and ".." not in pure.parts, f"{label}: unsafe artifact path")
    resolved = (root / Path(*pure.parts)).resolve()
    require(root == resolved or root in resolved.parents, f"{label}: artifact escaped evidence root")
    require(resolved.is_file(), f"{label}: artifact file missing")
    return resolved, pure.as_posix()


def load_runtime_envelope(path: Path) -> tuple[dict[str, Any], dict[str, dict[str, Any]], dict[str, str], Path]:
    require(path.is_file(), "runtime evidence envelope does not exist")
    root = path.resolve().parent
    envelope_hash = sha256_file(path.resolve())
    envelope = read_json(path.resolve())
    require(envelope.get("schema") == RUNTIME_EVIDENCE_SCHEMA, "runtime evidence schema mismatch")
    require(envelope.get("revision") in {REVISION, "V15.17D-PRODUCTION_RUNTIME_ACCEPTANCE"}, "runtime evidence revision mismatch")
    require(str(envelope.get("base_commit", "")).lower() == BASE_COMMIT, "runtime evidence base commit mismatch")
    require(envelope.get("target_branch") == TARGET_BRANCH, "runtime evidence branch mismatch")
    require(str(envelope.get("production_mjcf_sha256", "")).lower() == NEW_MJCF_SHA256, "runtime evidence model hash mismatch")
    artifacts = envelope.get("artifacts")
    require(isinstance(artifacts, dict), "runtime evidence artifacts map missing")
    missing = sorted(REQUIRED_RUNTIME_ARTIFACTS - set(artifacts))
    require(not missing, f"runtime evidence missing required artifacts: {missing}")
    resolved_records: dict[str, dict[str, Any]] = {}
    hashes: dict[str, str] = {}
    for name in sorted(artifacts):
        record = artifacts[name]
        require(isinstance(record, dict), f"artifact {name}: record is not an object")
        artifact_path, relative = safe_evidence_path(root, record.get("path"), f"artifact {name}")
        actual = sha256_file(artifact_path)
        expected = str(record.get("sha256", "")).lower()
        require(re.fullmatch(r"[0-9a-f]{64}", expected) is not None, f"artifact {name}: invalid SHA256")
        require(actual == expected, f"artifact {name}: SHA256 mismatch")
        resolved_records[name] = {
            "logical_path": relative,
            "sha256": actual,
            "size_bytes": artifact_path.stat().st_size,
            "_path": artifact_path,
        }
        hashes[name] = actual
    public_records = {
        name: {key: value for key, value in record.items() if not key.startswith("_")}
        for name, record in resolved_records.items()
    }
    public_records["__envelope__"] = {
        "logical_path": path.name,
        "sha256": envelope_hash,
        "size_bytes": path.stat().st_size,
    }
    hashes["__envelope__"] = envelope_hash
    return envelope, resolved_records, hashes, root


def rehash_runtime_artifacts(path: Path, records: Mapping[str, Mapping[str, Any]], expected: Mapping[str, str]) -> dict[str, str]:
    actual = {name: sha256_file(Path(record["_path"])) for name, record in records.items()}
    actual["__envelope__"] = sha256_file(path.resolve())
    require(actual == dict(expected), "runtime evidence changed during validation")
    return actual


def validate_patched_tool(copy_path: Path, source_relative: str, source_sha: str, label: str) -> dict[str, Any]:
    source = repo_path(source_relative).read_bytes()
    copied = copy_path.read_bytes()
    require(sha256_bytes(source) == source_sha, f"{label}: frozen source tool SHA256 mismatch")
    source_lf = source.replace(b"\r\n", b"\n")
    copied_lf = copied.replace(b"\r\n", b"\n")
    old = OLD_MJCF_SHA256.encode("ascii")
    new = NEW_MJCF_SHA256.encode("ascii")
    require(source_lf.count(old) == 1 and source_lf.count(new) == 0, f"{label}: source anchor count mismatch")
    require(copied_lf.count(old) == 0 and copied_lf.count(new) == 1, f"{label}: runtime copy anchor count mismatch")
    require(copied_lf.replace(new, old) == source_lf, f"{label}: runtime copy differs beyond model anchor")
    expected_runtime_sha = PATCHED_QA_TOOL_SHA256 if source_relative == QA_TOOL_REL else PATCHED_CROSS_503_SHA256
    require(sha256_bytes(copied_lf) == expected_runtime_sha, f"{label}: patched runtime tool SHA256 mismatch")
    return {
        "pass": True,
        "source_relative_path": source_relative,
        "source_sha256": source_sha,
        "runtime_copy_sha256": sha256_bytes(copied_lf),
        "only_change": "EXPECTED_PRODUCTION_MODEL_SHA256_LITERAL",
        "old_model_sha256": OLD_MJCF_SHA256,
        "new_model_sha256": NEW_MJCF_SHA256,
    }


def bridge_status_values(status: Mapping[str, Any], label: str) -> dict[str, Any]:
    require(status.get("schema") == "go-m8010-arm-v15.14-mujoco-bridge-status/1.0", f"{label}: bridge status schema mismatch")
    gate_require(status.get("execution_mode") == "kinematic_position_tracking", f"{label}: execution mode changed")
    gate_require(status.get("dynamics_valid") is False, f"{label}: bridge falsely claims real dynamics")
    gate_require(status.get("parent_collision_filter_disabled") is True, f"{label}: collision filtering contract changed")
    gate_require(status.get("fault_latched") is False, f"{label}: bridge fault is latched")
    gate_require(status.get("latch_fault_enabled") is True, f"{label}: fault latch was disabled")
    gate_require(status.get("command_stream_primed") is True, f"{label}: command stream was not primed")
    gate_require(status.get("command_subscription_history") == "keep_last", f"{label}: command history changed")
    gate_require(status.get("command_subscription_depth") == 1, f"{label}: command depth changed")
    gate_require(status.get("moving_command_watchdog_active") is True, f"{label}: moving watchdog inactive")
    gate_require(int(status.get("moving_watchdog_timeout_count", -1)) == 0, f"{label}: moving watchdog timed out")
    gate_require(int(status.get("rejected_command_count", -1)) == 0, f"{label}: bridge rejected a command")
    model = status.get("model", {})
    collision = status.get("collision_contract", {})
    guard = status.get("kinematic_guard", {})
    mesh = status.get("runtime_mesh_integrity", {})
    gate_require(str(model.get("sha256", "")).lower() == NEW_MJCF_SHA256, f"{label}: runtime model SHA mismatch")
    gate_require(str(collision.get("sha256", "")).lower() == COLLISION_CONTRACT_SHA256, f"{label}: collision authority SHA mismatch")
    gate_require(str(guard.get("sha256", "")).lower() == GUARD_SHA256, f"{label}: guard SHA mismatch")
    gate_require(mesh.get("pass") is True, f"{label}: runtime mesh integrity failed")
    required_mesh = {
        "runtime_manifest_sha256": RUNTIME_MESH_MANIFEST_SHA256,
        "mesh_count": 1008,
        "visual_chunk_count": 45,
        "collision_piece_count": 961,
        "motion_proxy_piece_count": 2,
        "total_bytes": 383_827_772,
        "asset_set_sha256": RUNTIME_ASSET_SET_SHA256,
        "source_manifest_sha256": SOURCE_MESH_MANIFEST_SHA256,
        "explicit_geom_pair_count": 82,
        "explicit_geom_pair_set_pass": True,
    }
    for key, expected in required_mesh.items():
        actual = mesh.get(key)
        if isinstance(expected, str):
            actual = str(actual).lower()
        gate_require(actual == expected, f"{label}: runtime mesh field mismatch: {key}")
    envelope = status.get("simulation_execution_envelope", {})
    gate_require(finite(envelope.get("max_velocity_rad_s"), f"{label}.max_velocity") == 0.5, f"{label}: bridge velocity envelope changed")
    gate_require(finite(envelope.get("max_acceleration_rad_s2"), f"{label}.max_acceleration") == 1.0, f"{label}: bridge acceleration envelope changed")
    gate_require(finite(envelope.get("publish_rate_hz"), f"{label}.publish_rate") == 50.0, f"{label}: bridge publish rate changed")
    gate_require(finite(envelope.get("absolute_calculation_tolerance"), f"{label}.calculation_tolerance") == 0.02, f"{label}: calculation tolerance changed")
    gate_require(finite(envelope.get("velocity_consistency_tolerance_rad_s"), f"{label}.velocity_tolerance") == 0.04, f"{label}: velocity consistency tolerance changed")
    gate_require(finite(envelope.get("command_timeout_s"), f"{label}.command_timeout") == 0.1, f"{label}: command timeout changed")
    gate_require(finite(envelope.get("prime_position_tolerance_rad"), f"{label}.prime_position") == 1.0e-6, f"{label}: prime position tolerance changed")
    gate_require(finite(envelope.get("prime_velocity_tolerance_rad_s"), f"{label}.prime_velocity") == 1.0e-6, f"{label}: prime velocity tolerance changed")
    gate_require(finite(envelope.get("guard_max_step_deg"), f"{label}.guard_step") == 0.25, f"{label}: guard step changed")
    observed_velocity = finite(envelope.get("max_observed_command_velocity_rad_s"), f"{label}.observed_velocity")
    observed_acceleration = finite(envelope.get("max_observed_command_acceleration_rad_s2"), f"{label}.observed_acceleration")
    source_receipt_difference = finite(envelope.get("max_source_receipt_dt_difference_s"), f"{label}.source_receipt_difference")
    velocity_consistency = finite(envelope.get("max_velocity_consistency_error_rad_s"), f"{label}.velocity_consistency")
    source_age = finite(envelope.get("max_command_source_age_s"), f"{label}.source_age")
    stationary_gaps = int(envelope.get("stationary_command_gap_count", -1))
    dropped_stale = int(envelope.get("dropped_stale_stationary_command_count", -1))
    stationary_reuse = int(envelope.get("stationary_guard_reuse_count", -1))
    gate_require(0.0 <= observed_velocity <= 0.5, f"{label}: observed command velocity exceeded envelope")
    gate_require(0.0 <= observed_acceleration <= 1.0, f"{label}: observed command acceleration exceeded envelope")
    gate_require(0.0 <= source_receipt_difference <= 0.1, f"{label}: source/receipt timing difference exceeded command timeout")
    gate_require(0.0 <= velocity_consistency <= 0.04, f"{label}: velocity consistency exceeded envelope")
    gate_require(0.0 <= source_age <= 0.1, f"{label}: command source age exceeded timeout")
    gate_require(stationary_gaps == 0 and dropped_stale == 0 and stationary_reuse >= 0, f"{label}: stationary command stream integrity failed")
    normalization = status.get("j1_continuous_branch_normalization", {})
    gate_require(normalization.get("enabled") is True and normalization.get("joint_name") == "J1", f"{label}: J1 normalization changed")
    gate_require(normalization.get("method") == "nearest_equivalent_to_current", f"{label}: J1 normalization method changed")
    gate_require(normalization.get("pi_tie_break") == "match_joint_trajectory_controller", f"{label}: J1 tie-break changed")
    accepted_adjustments = int(normalization.get("accepted_adjustment_count", -1))
    maximum_adjustment = finite(normalization.get("max_abs_adjustment_rad"), f"{label}.j1_max_abs_adjustment")
    gate_require(accepted_adjustments >= 0 and 0.0 <= maximum_adjustment <= 2.0 * math.pi + 1.0e-12, f"{label}: J1 normalization runtime statistics invalid")
    accepted_count = int(status.get("accepted_command_count", -1))
    accepted_moving = int(status.get("accepted_moving_command_count", -1))
    stationary_confirmations = int(status.get("stationary_confirmation_count", -1))
    last_receipt_age = finite(status.get("last_command_receipt_age_s"), f"{label}.last_receipt_age")
    gate_require(accepted_count > 0 and 0 <= accepted_moving <= accepted_count, f"{label}: accepted command counters invalid")
    gate_require(stationary_confirmations >= 0 and 0.0 <= last_receipt_age <= 0.1, f"{label}: stationary/receipt runtime state invalid")
    last_guard = status.get("last_guard_result", {})
    require(isinstance(last_guard, dict), f"{label}: last guard result malformed")
    gate_require(last_guard.get("safe") is True and last_guard.get("reason") in {"accepted_motion_step", "accepted_stationary_heartbeat"}, f"{label}: last guard result is not a known safe acceptance")
    require(int(last_guard.get("steps", -1)) >= 0, f"{label}: last guard step count malformed")
    vector(status.get("joint_position_rad"), 6, f"{label}.joint_position")
    vector(status.get("joint_velocity_rad_s"), 6, f"{label}.joint_velocity")
    return {
        "model_sha256": NEW_MJCF_SHA256,
        "collision_contract_sha256": COLLISION_CONTRACT_SHA256,
        "guard_sha256": GUARD_SHA256,
        "mesh_authority_pass": True,
        "execution_mode": "kinematic_position_tracking",
        "fault_latched": False,
    }


def validate_runtime_boot(
    envelope: Mapping[str, Any],
    records: Mapping[str, Mapping[str, Any]],
) -> dict[str, Any]:
    colcon = Path(records["colcon_build_log"]["_path"]).read_text(encoding="utf-8", errors="replace")
    bringup = Path(records["bringup_log"]["_path"]).read_text(encoding="utf-8", errors="replace")
    ansi = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
    colcon_clean = ansi.sub("", colcon)
    bringup_clean = ansi.sub("", bringup)
    shutdown_markers = ["user interrupted with ctrl-c (SIGINT)", "signal_handler(signum=2)"]
    shutdown_positions = [bringup_clean.find(marker) for marker in shutdown_markers if marker in bringup_clean]
    operational_bringup = bringup_clean[: min(shutdown_positions)] if shutdown_positions else bringup_clean
    required_packages = (
        "go_m8010_arm_description",
        "go_m8010_arm_mujoco_bridge",
        "go_m8010_arm_v15_14_qa",
        "go_m8010_arm_v15_14_description",
        "go_m8010_arm_v15_14_moveit_config",
    )
    gate_require(re.search(r"Summary:\s+5 packages finished", colcon_clean) is not None, "fresh exact-five colcon completion marker missing")
    gate_require(all(f"Finished <<< {name}" in colcon_clean for name in required_packages), "fresh colcon log is missing an exact required package")
    gate_require("Failed" not in colcon_clean and "Aborted" not in colcon_clean, "fresh colcon log contains failure")
    installed = Path(records["installed_bridge"]["_path"]).read_bytes().replace(b"\r\n", b"\n")
    source = repo_path(BRIDGE_REL).read_bytes().replace(b"\r\n", b"\n")
    require(installed == source, "installed production bridge is not byte-equivalent to current source")
    forbidden = (
        "unaccepted model hash",
        "runtime mesh hash mismatch",
        "collision contract mismatch",
        "asset-set digest mismatch",
        "Traceback (most recent call last)",
    )
    gate_require(not any(token.lower() in operational_bringup.lower() for token in forbidden), "operational bringup log contains fail-closed rejection")
    gate_require(not ("unaccepted" in operational_bringup.lower() and ("mjcf" in operational_bringup.lower() or "hash" in operational_bringup.lower())), "operational bringup log contains unaccepted MJCF/hash rejection")
    status_json = read_json(Path(records["bridge_status_json"]["_path"]))
    status = status_json.get("status", status_json)
    require(isinstance(status, dict), "bridge status artifact malformed")
    parsed = bridge_status_values(status, "production bridge status")
    runtime = envelope.get("runtime", {})
    require(isinstance(runtime, dict), "runtime metadata missing")
    gate_require(runtime.get("fresh_colcon_build") is True, "runtime evidence does not declare fresh colcon build")
    gate_require(runtime.get("sourced_current_install_setup") is True, "runtime evidence does not bind current install space")
    gate_require(runtime.get("production_bridge_node_alive") is True, "production bridge node was not alive")
    gate_require(runtime.get("mujoco_compile_pass") is True, "MuJoCo model compile did not pass")
    return {
        "pass": True,
        "fresh_colcon_build_pass": True,
        "fresh_colcon_package_count": 5,
        "fresh_colcon_packages": list(required_packages),
        "current_source_install_binding_pass": True,
        "installed_bridge_sha256": sha256_file(Path(records["installed_bridge"]["_path"])),
        "installed_bridge_byte_equivalent_to_source": True,
        "bringup_rejection_markers_absent": True,
        "production_bridge_node_alive": True,
        "mujoco_compile_pass": True,
        "bridge_status": parsed,
    }


def validate_runtime_tools(records: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    qa = validate_patched_tool(
        Path(records["trajectory_qa_tool"]["_path"]),
        QA_TOOL_REL,
        QA_TOOL_SHA256,
        "trajectory QA runtime copy",
    )
    collision = validate_patched_tool(
        Path(records["collision_503_tool"]["_path"]),
        CROSS_503_REL,
        CROSS_503_SHA256,
        "503 collision runtime copy",
    )
    require(python_literal_assignment_bytes(Path(records["trajectory_qa_tool"]["_path"]).read_bytes(), "ACCEPTED_MODEL_SHA256") == NEW_MJCF_SHA256, "patched QA AST model anchor mismatch")
    input_map = python_literal_assignment_bytes(Path(records["collision_503_tool"]["_path"]).read_bytes(), "EXPECTED_INPUT_SHA256")
    require(isinstance(input_map, dict) and input_map.get("model") == NEW_MJCF_SHA256, "patched 503 AST model anchor mismatch")
    require(input_map.get("validator") == POSE_SOURCE_VALIDATOR_SHA256 and input_map.get("guard") == GUARD_SHA256, "patched 503 non-model anchors changed")
    return {"pass": True, "trajectory_qa_tool": qa, "collision_503_tool": collision}


def validate_qa_thresholds(report: Mapping[str, Any]) -> dict[str, float]:
    actual = report.get("thresholds")
    require(isinstance(actual, dict), "trajectory QA thresholds missing")
    result: dict[str, float] = {}
    require(set(actual) == set(EXPECTED_QA_THRESHOLDS), "trajectory QA threshold key set changed")
    for key, expected in EXPECTED_QA_THRESHOLDS.items():
        value = finite(actual.get(key), f"trajectory threshold {key}")
        require(value == expected, f"trajectory threshold changed: {key}")
        result[key] = value
    return result


def validate_bridge_runtime_contract(contract: Mapping[str, Any], label: str) -> dict[str, Any]:
    status = contract.get("status")
    require(isinstance(status, dict), f"{label}: embedded bridge status missing")
    parsed = bridge_status_values(status, label)
    expected_model = contract.get("expected_model", {})
    require(str(expected_model.get("sha256", "")).lower() == NEW_MJCF_SHA256, f"{label}: expected model hash mismatch")
    expected_collision = contract.get("expected_collision_contract", {})
    require(str(expected_collision.get("sha256", "")).lower() == COLLISION_CONTRACT_SHA256, f"{label}: expected collision hash mismatch")
    expected_guard = contract.get("expected_kinematic_guard", {})
    require(str(expected_guard.get("sha256", "")).lower() == GUARD_SHA256, f"{label}: expected guard hash mismatch")
    expected_mesh = contract.get("expected_runtime_mesh_integrity", {})
    require(str(expected_mesh.get("runtime_manifest_sha256", "")).lower() == RUNTIME_MESH_MANIFEST_SHA256, f"{label}: expected mesh manifest mismatch")
    require(str(expected_mesh.get("asset_set_sha256", "")).lower() == RUNTIME_ASSET_SET_SHA256, f"{label}: expected asset set mismatch")
    gate_require(contract.get("state_streams_fresh") is True and contract.get("pass") is True, f"{label}: runtime contract failed")
    controller = contract.get("controller_runtime_parameters", {})
    gate_require(controller.get("controller_manager_update_rate_hz") == 50, f"{label}: controller manager rate changed")
    gate_require(finite(controller.get("arm_controller_state_publish_rate_hz"), f"{label}.state_rate") == 50.0, f"{label}: controller state rate changed")
    gate_require(controller.get("angle_wraparound") == {"J1": True, "J2": False, "J3": False, "J4": False, "J5": False, "J6": False}, f"{label}: angle wraparound contract changed")
    gate_require(controller.get("pass") is True, f"{label}: controller runtime parameter gate failed")
    return parsed


def validate_trajectory_qa(records: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    report = read_json(Path(records["trajectory_qa_json"]["_path"]))
    require(report.get("schema") == "go-m8010-arm-v15.14-trajectory-closed-loop-qa/1.0", "trajectory QA schema mismatch")
    require(report.get("revision") == "V15.14-MoveIt2-ros2_control-MuJoCo", "trajectory QA revision mismatch")
    gate_require(report.get("status") == "PASS", "trajectory QA status is not PASS")
    require(str(report.get("mujoco_model", {}).get("sha256", "")).lower() == NEW_MJCF_SHA256, "trajectory QA was not run against the current production MJCF")
    require(str(report.get("target_manifest", {}).get("sha256", "")).lower() == TARGET_MANIFEST_SHA256, "trajectory QA target manifest hash mismatch")
    scope = report.get("scope", {})
    require(scope == {"execution_mode": "kinematic_position_tracking", "dynamics_valid": False, "simulation_only": True, "tcp_frame": "tcp_nominal"}, "trajectory QA scope changed")
    thresholds = validate_qa_thresholds(report)
    target_manifest = read_json(repo_path(TARGET_MANIFEST_REL))
    manifest_rows = target_manifest.get("targets")
    tests = report.get("tests")
    require(isinstance(manifest_rows, list) and isinstance(tests, list), "trajectory target/test arrays missing")
    require(len(manifest_rows) == 12 and len(tests) == 12, "trajectory suite is not exact 12 targets")
    expected_by_id = {str(row.get("id")): str(row.get("expected_outcome")) for row in manifest_rows}
    require(len(expected_by_id) == 12, "target manifest identities are not unique")
    require([str(row.get("id")) for row in tests] == [str(row.get("id")) for row in manifest_rows], "trajectory result order/identity differs from frozen manifest")

    success_count = 0
    rejection_count = 0
    maximums = {
        "max_final_joint_error_rad": 0.0,
        "max_tcp_position_error_m": 0.0,
        "max_tcp_orientation_error_rad": 0.0,
        "max_rviz_mujoco_joint_sync_error_rad": 0.0,
        "max_full_run_public_raw_sync_error_rad": 0.0,
        "max_trajectory_tracking_error_rad": 0.0,
        "max_trajectory_velocity_tracking_error_rad_s": 0.0,
        "max_controller_state_gap_s": 0.0,
        "max_raw_state_gap_s": 0.0,
        "max_public_state_gap_s": 0.0,
        "max_public_raw_joint_velocity_error_rad_s": 0.0,
        "max_public_raw_pair_time_skew_s": 0.0,
        "max_tf_tcp_position_error_m": 0.0,
        "max_tf_tcp_orientation_error_rad": 0.0,
    }
    target_rows: list[dict[str, Any]] = []
    for index, test in enumerate(tests):
        require(isinstance(test, dict), f"trajectory target {index}: malformed row")
        frozen_target = manifest_rows[index]
        target_id = str(test.get("id"))
        outcome = expected_by_id[target_id]
        require(all(test.get(key) == frozen_target.get(key) for key in ("id", "category", "planning_mode", "expected_outcome")), f"{target_id}: frozen target identity/category/mode changed")
        require(test.get("expected_outcome") == outcome, f"{target_id}: expected outcome changed")
        controller = test.get("controller", {})
        require(controller.get("action_name") == "/arm_controller/follow_joint_trajectory", f"{target_id}: nonstandard trajectory action")
        if outcome == "execute_success":
            success_count += 1
            gate_require(test.get("verdict") == "PASS", f"{target_id}: success verdict failed")
            gate_require(test.get("planning", {}).get("pass") is True, f"{target_id}: planning failed")
            gate_require(test.get("trajectory", {}).get("pass") is True, f"{target_id}: trajectory validation failed")
            gate_require(controller.get("accepted") is True and controller.get("result_code") == 1, f"{target_id}: FJT execution failed")
            ik = test.get("ik", {})
            gate_require(ik.get("error_code") == 1 and ik.get("kinematic_solution_found") is True and ik.get("collision_aware_pass") is True, f"{target_id}: IK/collision-aware solution failed")
            expected_joint = vector(frozen_target.get("joint_position_rad"), 6, f"manifest.{target_id}.joint_position")
            actual_joint = vector(ik.get("solution_rad"), 6, f"{target_id}.ik.solution")
            require(actual_joint == expected_joint, f"{target_id}: executed target differs from frozen manifest joint target")
            collision = test.get("collision", {})
            gate_require(collision.get("moveit_goal_collision") is False and collision.get("mujoco_goal_collision") is False and collision.get("cross_engine_boolean_match") is True, f"{target_id}: goal collision/cross-engine gate failed")
            gate_require(test.get("joint_limits", {}).get("goal_within_bounds") is True and test.get("joint_limits", {}).get("trajectory_within_bounds") is True, f"{target_id}: joint-limit gate failed")
            execution = test.get("execution", {})
            gate_require(execution.get("execute_request_payload_matches_validated") is True, f"{target_id}: executed payload differs from validated trajectory")
            planned_semantic = str(test.get("trajectory", {}).get("planned_trajectory_semantic_sha256", ""))
            validated_semantic = str(execution.get("validated_trajectory_semantic_sha256", ""))
            execute_semantic = str(execution.get("execute_request_trajectory_semantic_sha256", ""))
            require(re.fullmatch(r"[0-9a-f]{64}", planned_semantic) is not None and planned_semantic == validated_semantic == execute_semantic, f"{target_id}: planned/validated/executed trajectory semantic hashes differ")
            planned_cdr = str(test.get("trajectory", {}).get("planned_trajectory_cdr_sha256", ""))
            validated_cdr = str(execution.get("validated_trajectory_cdr_sha256", ""))
            require(re.fullmatch(r"[0-9a-f]{64}", planned_cdr) is not None and planned_cdr == validated_cdr, f"{target_id}: planned/validated CDR hashes differ")
            gate_require(execution.get("pre_execution_gate_pass") is True, f"{target_id}: pre-execution gate failed")
            validate_bridge_runtime_contract(execution.get("bridge_runtime_contract_before_execute", {}), f"{target_id} pre-execute")
            validate_bridge_runtime_contract(execution.get("bridge_runtime_contract", {}), f"{target_id} post-execute")
            final_joint = finite(execution.get("max_abs_final_joint_error_rad"), f"{target_id}.final_joint")
            tcp_position = finite(test.get("tcp_result", {}).get("position_error_m"), f"{target_id}.tcp_position")
            tcp_orientation = finite(test.get("tcp_result", {}).get("orientation_error_rad"), f"{target_id}.tcp_orientation")
            sync = finite(test.get("sync", {}).get("max_joint_position_error_rad"), f"{target_id}.joint_sync")
            trace = execution.get("trace_metrics", {})
            gate_require(trace.get("pass") is True, f"{target_id}: trace metrics failed")
            trajectory_tracking = finite(trace.get("max_desired_actual_joint_error_rad"), f"{target_id}.tracking")
            velocity_tracking = finite(trace.get("max_desired_actual_joint_velocity_error_rad_s"), f"{target_id}.velocity_tracking")
            controller_gap = finite(trace.get("max_controller_sample_gap_s"), f"{target_id}.controller_gap")
            raw_gap = finite(trace.get("max_raw_state_sample_gap_s"), f"{target_id}.raw_gap")
            public_gap = finite(trace.get("max_public_state_sample_gap_s"), f"{target_id}.public_gap")
            velocity_sync = finite(trace.get("max_public_raw_joint_velocity_error_rad_s"), f"{target_id}.velocity_sync")
            state_skew = finite(trace.get("max_public_raw_pair_time_skew_s"), f"{target_id}.state_skew")
            full_run_sync = finite(trace.get("max_public_raw_joint_error_rad"), f"{target_id}.full_run_sync")
            tf_pos = finite(test.get("sync", {}).get("max_tf_tcp_position_error_m"), f"{target_id}.tf_position")
            tf_rot = finite(test.get("sync", {}).get("max_tf_tcp_orientation_error_rad"), f"{target_id}.tf_orientation")
            gate_require(final_joint <= thresholds["final_joint_max_abs_error_rad"], f"{target_id}: final joint threshold failed")
            gate_require(tcp_position <= thresholds["final_tcp_position_error_m"], f"{target_id}: TCP position threshold failed")
            gate_require(tcp_orientation <= thresholds["final_tcp_orientation_error_rad"], f"{target_id}: TCP orientation threshold failed")
            gate_require(sync <= thresholds["final_public_raw_joint_sync_rad"], f"{target_id}: final joint sync threshold failed")
            gate_require(trajectory_tracking <= thresholds["trajectory_tracking_max_abs_error_rad"], f"{target_id}: tracking threshold failed")
            gate_require(velocity_tracking <= thresholds["trajectory_velocity_tracking_max_abs_error_rad_s"], f"{target_id}: velocity tracking threshold failed")
            gate_require(controller_gap <= thresholds["max_controller_state_gap_s"], f"{target_id}: controller gap threshold failed")
            gate_require(raw_gap <= thresholds["max_raw_state_gap_s"] and public_gap <= thresholds["max_public_state_gap_s"], f"{target_id}: state stream gap threshold failed")
            gate_require(velocity_sync <= thresholds["public_raw_joint_velocity_sync_rad_s"], f"{target_id}: velocity sync threshold failed")
            gate_require(state_skew <= thresholds["max_state_pair_time_skew_s"], f"{target_id}: state skew threshold failed")
            gate_require(tf_pos <= thresholds["tf_mujoco_tcp_position_m"] and tf_rot <= thresholds["tf_mujoco_tcp_orientation_rad"], f"{target_id}: TF/MuJoCo TCP threshold failed")
            values = (final_joint, tcp_position, tcp_orientation, sync, full_run_sync, trajectory_tracking, velocity_tracking, controller_gap, raw_gap, public_gap, velocity_sync, state_skew, tf_pos, tf_rot)
            for key, value in zip(maximums, values):
                maximums[key] = max(maximums[key], value)
            target_rows.append({"id": target_id, "expected_outcome": outcome, "verdict": "PASS", "max_final_joint_error_rad": final_joint, "tcp_position_error_m": tcp_position, "tcp_orientation_error_rad": tcp_orientation, "joint_sync_error_rad": sync})
        else:
            rejection_count += 1
            require(outcome in {"bounds_rejection", "collision_rejection"}, f"{target_id}: unexpected rejection class")
            gate_require(test.get("verdict") == "EXPECTED_REJECTION_PASS", f"{target_id}: rejection verdict failed")
            execution = test.get("execution", {})
            gate_require(execution.get("no_execution_verified") is True, f"{target_id}: negative target executed")
            gate_require(execution.get("no_moving_command_accepted") is True and execution.get("no_command_rejected") is True, f"{target_id}: bridge counters changed during negative target")
            before_counters = execution.get("bridge_counters_before_request")
            after_counters = execution.get("bridge_counters_after_request")
            require(isinstance(before_counters, dict) and isinstance(after_counters, dict), f"{target_id}: negative bridge counter snapshots missing")
            gate_require(before_counters == after_counters and set(before_counters) == {"accepted_moving_command_count", "rejected_command_count"}, f"{target_id}: negative bridge counters changed")
            excursion = finite(execution.get("max_joint_excursion_during_plan_only_rad"), f"{target_id}.negative_excursion")
            gate_require(excursion <= thresholds["negative_no_execution_joint_excursion_rad"], f"{target_id}: rejection excursion threshold failed")
            planning = test.get("planning", {})
            gate_require(planning.get("pass") is True, f"{target_id}: rejection planning contract failed")
            collision = test.get("collision", {})
            if outcome == "bounds_rejection":
                gate_require(test.get("joint_limits", {}).get("goal_within_bounds") is False, f"{target_id}: requested target was not outside bounds")
                gate_require(planning.get("planned_final_within_bounds") is True, f"{target_id}: planned frozen clamp is outside bounds")
                gate_require(planning.get("planned_final_matches_requested") is False, f"{target_id}: planned final unexpectedly equals out-of-bounds request")
                gate_require(planning.get("planned_final_matches_frozen_clamp") is True, f"{target_id}: planned final differs from frozen clamp")
                validation = planning.get("trajectory_validation", {})
                gate_require(isinstance(validation, dict) and validation.get("pass") is True and int(validation.get("bounds_violation_count", -1)) == 0, f"{target_id}: clamped trajectory validation failed")
                violations = test.get("joint_limits", {}).get("violations")
                require(isinstance(violations, list) and len(violations) == 1, f"{target_id}: expected one requested-limit violation record")
                violation = violations[0]
                expected_request = vector(frozen_target.get("joint_position_rad"), 6, f"manifest.{target_id}.joint_position")
                require(violation.get("joint") == "J2" and abs(finite(violation.get("value"), f"{target_id}.violation.value") - expected_request[1]) <= 1.0e-15, f"{target_id}: requested-limit violation identity/value mismatch")
                require(vector(violation.get("allowed"), 2, f"{target_id}.violation.allowed") == [-2.96705972839036, 2.96705972839036], f"{target_id}: frozen J2 bounds changed")
                gate_require(collision.get("mujoco_reason") == "position_limit", f"{target_id}: MuJoCo bounds rejection reason changed")
            else:
                gate_require(collision.get("moveit_self_collision") is True and collision.get("mujoco_self_collision") is True, f"{target_id}: self-collision was not reproduced by both engines")
                gate_require(collision.get("mujoco_reason") == "self_collision", f"{target_id}: MuJoCo rejection reason is not self_collision")
                gate_require(isinstance(collision.get("moveit_self_contacts"), list) and collision.get("moveit_self_contacts"), f"{target_id}: MoveIt self contacts missing")
                gate_require(isinstance(collision.get("mujoco_self_contacts"), list) and collision.get("mujoco_self_contacts"), f"{target_id}: MuJoCo self contacts missing")
                gate_require(collision.get("moveit_ground_collision") is False and collision.get("mujoco_ground_collision") is False, f"{target_id}: ground collision cannot substitute for self collision")
                gate_require(collision.get("moveit_goal_collision") is True and collision.get("mujoco_goal_collision") is True and collision.get("cross_engine_boolean_match") is True, f"{target_id}: self-collision cross-engine boolean failed")
            target_rows.append({"id": target_id, "expected_outcome": outcome, "verdict": "EXPECTED_REJECTION_PASS", "max_joint_excursion_rad": excursion})
    require(success_count == 10 and rejection_count == 2, "trajectory suite expected outcome counts changed")

    ros_graph = report.get("ros_graph", {})
    gate_require(ros_graph.get("follow_joint_trajectory_server") == "/arm_controller/follow_joint_trajectory", "standard FJT server changed")
    gate_require(ros_graph.get("follow_joint_trajectory_providers") == [{"node": "/arm_controller", "types": ["control_msgs/action/FollowJointTrajectory"]}], "FJT provider identity changed")
    gate_require(ros_graph.get("joint_state_publishers") == ["/joint_state_broadcaster"], "public joint-state authority changed")
    gate_require(ros_graph.get("execute_trajectory_server") == "/execute_trajectory", "MoveIt execute action changed")
    gate_require(ros_graph.get("execute_trajectory_providers") == [{"node": "/move_group", "types": ["moveit_msgs/action/ExecuteTrajectory"]}], "MoveIt execute action provider changed")
    gate_require(ros_graph.get("mujoco_joint_command_publishers") == ["/topic_based_ros2_control_MujocoTopicSystem"], "MuJoCo command publisher changed")
    gate_require(ros_graph.get("mujoco_joint_command_subscribers") == ["/mujoco_bridge"], "MuJoCo command subscriber changed")
    gate_require(ros_graph.get("mujoco_raw_state_publishers") == ["/mujoco_bridge"], "MuJoCo raw-state publisher changed")
    gate_require(ros_graph.get("mujoco_raw_state_hardware_subscribers") == ["/topic_based_ros2_control_MujocoTopicSystem"], "MuJoCo raw-state hardware subscriber changed")
    gate_require(ros_graph.get("single_joint_state_source") is True, "multiple public joint-state publishers exist")
    gate_require(ros_graph.get("mujoco_bridge_action_servers") == [], "bridge exposes a forbidden action server")
    gate_require(ros_graph.get("no_private_moveit_mujoco_edge") is True, "private MoveIt-to-MuJoCo edge exists")
    gate_require(ros_graph.get("standard_topic_based_ros2_control_path") is True and ros_graph.get("pass") is True, "standard ros2_control path failed")
    controllers = ros_graph.get("controllers", {})
    gate_require(controllers.get("joint_state_broadcaster", {}).get("state") == "active", "joint_state_broadcaster inactive")
    gate_require(controllers.get("arm_controller", {}).get("state") == "active", "arm_controller inactive")
    gate_require(controllers.get("arm_controller", {}).get("type") == "joint_trajectory_controller/JointTrajectoryController", "arm controller type changed")
    startup = report.get("bridge_runtime_contract", {}).get("startup", {})
    post = report.get("bridge_runtime_contract", {}).get("post_run", {})
    validate_bridge_runtime_contract(startup, "suite startup")
    validate_bridge_runtime_contract(post, "suite post-run")
    ground = report.get("ground_scene", {})
    gate_require(ground.get("object_count") == 1 and ground.get("frame_id") == "world", "ground scene identity changed")
    require(max_abs(left - right for left, right in zip(vector(ground.get("dimensions_m"), 3, "ground dimensions"), [4.0, 4.0, 0.02])) <= 1.0e-15, "ground dimensions changed")
    require(max_abs(left - right for left, right in zip(vector(ground.get("center_m"), 3, "ground center"), [0.0, 0.0, -0.1])) <= 1.0e-15, "ground center changed")
    require(ground.get("mujoco_ground", {}).get("geom_id") == 0, "MuJoCo ground geom identity changed")
    require(max_abs(left - right for left, right in zip(vector(ground.get("mujoco_ground", {}).get("position_m"), 3, "MuJoCo ground position"), [0.0, 0.0, -0.09])) <= 1.0e-15, "MuJoCo ground position changed")
    gate_require(ground.get("mujoco_ground", {}).get("contype") == 4 and ground.get("mujoco_ground", {}).get("conaffinity") == 0, "MuJoCo ground collision mask changed")
    gate_require(ground.get("mujoco_robot_ground_affinity", {}).get("pass") is True and ground.get("pass") is True, "ground scene gate failed")
    aggregate = report.get("aggregate", {})
    require(aggregate.get("positive_target_count") == success_count and aggregate.get("expected_rejection_count") == rejection_count, "trajectory aggregate counts differ")
    gate_require(aggregate.get("all_expectations_met") is True and aggregate.get("pass") is True, "trajectory aggregate did not pass")
    aggregate_map = {
        "max_final_joint_error_rad": "max_final_joint_error_rad",
        "max_tcp_position_error_m": "max_tcp_position_error_m",
        "max_tcp_orientation_error_rad": "max_tcp_orientation_error_rad",
        "max_rviz_mujoco_joint_sync_error_rad": "max_rviz_mujoco_joint_sync_error_rad",
        "max_trajectory_tracking_error_rad": "max_trajectory_tracking_error_rad",
        "max_full_run_public_raw_sync_error_rad": "max_full_run_public_raw_sync_error_rad",
    }
    for report_key, computed_key in aggregate_map.items():
        report_value = finite(aggregate.get(report_key), f"aggregate.{report_key}")
        require(abs(report_value - maximums[computed_key]) <= 1.0e-15, f"aggregate {report_key} differs from target recomputation")
        if report_key == "max_full_run_public_raw_sync_error_rad":
            gate_require(report_value <= thresholds["public_raw_joint_sync_rad"], "full-run public/raw sync threshold failed")
    return {
        "pass": True,
        "original_tool_reused": True,
        "schema": report["schema"],
        "revision": report["revision"],
        "thresholds": thresholds,
        "target_count": len(tests),
        "expected_success_pass_count": success_count,
        "expected_success_total": 10,
        "expected_rejection_pass_count": rejection_count,
        "expected_rejection_total": 2,
        "target_rows": target_rows,
        "raw_trace_payload_hashes_verified": False,
        "raw_trace_note": "TARGET_METRICS_RECOMPUTED_FROM_HASH_BOUND_QA_JSON; EXTERNAL_TRACE_FILES_NOT_CLAIMED_UNLESS_SEPARATELY_ENVELOPED",
        "maximums": maximums,
        "standard_follow_joint_trajectory_pass": True,
        "moveit_load_pass": True,
        "ros2_control_load_pass": True,
        "joint_state_authority_pass": True,
        "trajectory_numeric_thresholds_pass": True,
    }


def validate_attempt_history(
    envelope: Mapping[str, Any],
    records: Mapping[str, Mapping[str, Any]],
    current_input_snapshot: Mapping[str, str | None],
) -> dict[str, Any]:
    history = envelope.get("attempt_history")
    require(isinstance(history, list) and len(history) == 2, "trajectory attempt history must retain exactly attempt1 and selected formal run")
    require([row.get("id") for row in history[:2]] == ["attempt1", "attempt2"], "trajectory attempt history identity/order mismatch")
    first, second = history[0], history[1]
    require(first.get("artifact") == "trajectory_qa_attempt1_json" and second.get("artifact") == "trajectory_qa_json", "trajectory attempt artifact mapping mismatch")
    require(first.get("selected") is False and second.get("selected") is True, "trajectory selected-run history mismatch")
    require(first.get("status") == "FAIL", "trajectory attempt1 must remain an honest FAIL")
    require(first.get("reason_code") == "OUTSIDE_JOINT_LIMIT_FROZEN_CLAMP_REJECTION_CONTRACT_FAILED", "trajectory attempt1 reason identity mismatch")
    require(second.get("status") == "PASS", "selected trajectory attempt is not PASS")
    snapshot1 = str(first.get("repo_input_snapshot_sha256", "")).lower()
    snapshot2 = str(second.get("repo_input_snapshot_sha256", "")).lower()
    require(re.fullmatch(r"[0-9a-f]{64}", snapshot1) is not None and snapshot1 == snapshot2, "repo inputs changed between trajectory attempts")
    declared_snapshot = envelope.get("repo_input_snapshot")
    require(isinstance(declared_snapshot, dict), "runtime evidence repo-input snapshot details missing")
    normalized_snapshot = {str(key): (str(value).lower() if value is not None else None) for key, value in declared_snapshot.items()}
    require(normalized_snapshot == dict(current_input_snapshot), "runtime attempt repo-input snapshot differs from current frozen inputs")
    require(canonical_json_digest(normalized_snapshot) == snapshot1, "runtime attempt repo-input snapshot digest mismatch")

    attempt1 = read_json(Path(records["trajectory_qa_attempt1_json"]["_path"]))
    require(attempt1.get("schema") == "go-m8010-arm-v15.14-trajectory-closed-loop-qa/1.0", "attempt1 QA schema mismatch")
    require(attempt1.get("revision") == "V15.14-MoveIt2-ros2_control-MuJoCo", "attempt1 QA revision mismatch")
    require(attempt1.get("status") == "FAIL", "attempt1 raw report no longer records FAIL")
    require(str(attempt1.get("mujoco_model", {}).get("sha256", "")).lower() == NEW_MJCF_SHA256, "attempt1 production model hash mismatch")
    require(str(attempt1.get("target_manifest", {}).get("sha256", "")).lower() == TARGET_MANIFEST_SHA256, "attempt1 target manifest hash mismatch")
    require(attempt1.get("scope") == {"execution_mode": "kinematic_position_tracking", "dynamics_valid": False, "simulation_only": True, "tcp_frame": "tcp_nominal"}, "attempt1 scope mismatch")
    validate_qa_thresholds(attempt1)
    tests = attempt1.get("tests")
    require(isinstance(tests, list) and len(tests) == 12, "attempt1 did not execute exact 12-target suite")
    by_id = {str(row.get("id")): row for row in tests if isinstance(row, dict)}
    require(len(by_id) == 12, "attempt1 target identity set invalid")
    manifest_rows = read_json(repo_path(TARGET_MANIFEST_REL)).get("targets")
    require(isinstance(manifest_rows, list) and len(manifest_rows) == 12, "attempt1 frozen target manifest invalid")
    for test, frozen in zip(tests, manifest_rows):
        require(all(test.get(key) == frozen.get(key) for key in ("id", "category", "planning_mode", "expected_outcome")), f"attempt1 target identity/mode mismatch: {test.get('id')}")
    require(sum(row.get("expected_outcome") == "execute_success" and row.get("verdict") == "PASS" for row in tests) == 10, "attempt1 expected-success result changed")
    self_row = by_id.get("self_collision_reject", {})
    require(self_row.get("verdict") == "EXPECTED_REJECTION_PASS", "attempt1 self-collision rejection did not pass")
    outside = by_id.get("outside_joint_limit_reject", {})
    require(outside.get("verdict") != "EXPECTED_REJECTION_PASS", "attempt1 outside-limit failure was hidden")
    planning = outside.get("planning", {})
    execution = outside.get("execution", {})
    require(planning.get("accepted") is True, "attempt1 outside-limit planner acceptance evidence missing")
    require(planning.get("planned_final_matches_requested") is False, "attempt1 requested-vs-clamped mismatch evidence missing")
    require(planning.get("planned_final_matches_frozen_clamp") is True, "attempt1 frozen clamp evidence missing")
    require(execution.get("no_execution_verified") is True, "attempt1 outside-limit no-execution evidence missing")
    require(outside.get("controller", {}).get("execution_requested") is False, "attempt1 outside-limit unexpectedly requested execution")
    trajectory_validation = planning.get("trajectory_validation", {})
    bounds_count = int(trajectory_validation.get("bounds_violation_count", outside.get("trajectory", {}).get("bounds_violation_count", -1)))
    require(bounds_count >= 1, "attempt1 expected at least one detected clamped-trajectory bounds violation")
    return {
        "pass": True,
        "history_complete": True,
        "attempt_count": len(history),
        "selected_attempt_id": "attempt2",
        "repo_inputs_unchanged_between_attempts": True,
        "repo_input_snapshot_sha256": snapshot1,
        "attempt1": {
            "artifact_sha256": records["trajectory_qa_attempt1_json"]["sha256"],
            "status": "FAIL",
            "expected_success_pass_count": 10,
            "self_collision_rejection_pass": True,
            "outside_joint_limit_rejection_pass": False,
            "outside_joint_limit_execution_requested": False,
            "outside_joint_limit_no_execution_verified": True,
            "outside_joint_limit_frozen_clamp_match": True,
            "first_detected_bounds_violation_count": bounds_count,
            "bounds_violation_semantics": "BREAK_ON_FIRST_SENTINEL_AT_LEAST_ONE; NOT_A_TOTAL_COUNT",
            "reason_code": first["reason_code"],
        },
        "attempt2": {
            "artifact_sha256": records["trajectory_qa_json"]["sha256"],
            "status": "PASS",
            "selected_formal_run": True,
        },
    }


def normalized_pair_set(value: Any, label: str) -> set[tuple[str, str]]:
    require(isinstance(value, list), f"{label}: pair list missing")
    result: set[tuple[str, str]] = set()
    for row in value:
        require(isinstance(row, list) and len(row) == 2, f"{label}: malformed pair")
        pair = tuple(sorted((str(row[0]), str(row[1]))))
        require(pair not in result, f"{label}: duplicate pair")
        result.add(pair)
    return result


def recompute_collision_503(report: Mapping[str, Any]) -> dict[str, Any]:
    poses = report.get("poses")
    require(isinstance(poses, list) and len(poses) == 503, "503 raw pose array missing/wrong length")
    reconstructed_sets: dict[str, list[dict[str, float]]] = {}
    counts = {"moveit": {"overall": 0, "self": 0, "ground": 0}, "mujoco": {"overall": 0, "self": 0, "ground": 0}}
    mismatch_indices: list[int] = []
    boolean_mismatch_count = 0
    self_pair_mismatch_count = 0
    ground_pair_diagnostic_mismatch_count = 0
    pose_by_key: dict[tuple[str, int], Mapping[str, Any]] = {}
    canonical_rows: list[dict[str, Any]] = []
    for expected_index, pose in enumerate(poses):
        require(isinstance(pose, dict) and int(pose.get("global_index", -1)) == expected_index, f"503 pose index discontinuity: {expected_index}")
        category = str(pose.get("category"))
        pose_index = int(pose.get("pose_index", -1))
        key = (category, pose_index)
        require(key not in pose_by_key, f"duplicate 503 pose key: {key}")
        pose_by_key[key] = pose
        angles = pose.get("angles_deg")
        require(isinstance(angles, dict) and set(angles) == set(JOINTS), f"503 pose joint set mismatch: {expected_index}")
        angle_values = [finite(angles[name], f"pose[{expected_index}].{name}") for name in JOINTS]
        rows = reconstructed_sets.setdefault(category, [])
        require(pose_index == len(rows), f"503 category order discontinuity: {key}")
        rows.append({f"j{index}_deg": angle_values[index - 1] for index in range(1, 7)})
        moveit = pose.get("moveit")
        mujoco = pose.get("mujoco")
        require(isinstance(moveit, dict) and isinstance(mujoco, dict), f"503 pose engine records missing: {expected_index}")
        moveit_flags = (moveit.get("collision") is True, moveit.get("self_collision") is True, moveit.get("ground_collision") is True)
        mujoco_flags = (mujoco.get("collision") is True, mujoco.get("self_collision") is True, mujoco.get("ground_collision") is True)
        require(moveit_flags[0] == (moveit_flags[1] or moveit_flags[2]), f"503 MoveIt classification inconsistent: {expected_index}")
        require(mujoco_flags[0] == (mujoco_flags[1] or mujoco_flags[2]), f"503 MuJoCo classification inconsistent: {expected_index}")
        require((mujoco.get("safe") is True) == (not mujoco_flags[0]), f"503 MuJoCo safe flag inconsistent: {expected_index}")
        for engine, flags in (("moveit", moveit_flags), ("mujoco", mujoco_flags)):
            for name, flag in zip(("overall", "self", "ground"), flags):
                counts[engine][name] += int(flag)
        bool_match = moveit_flags == mujoco_flags
        moveit_self = normalized_pair_set(pose.get("moveit_self_proxy_pairs"), f"pose[{expected_index}].moveit_self")
        mujoco_self = normalized_pair_set(pose.get("mujoco_self_proxy_pairs"), f"pose[{expected_index}].mujoco_self")
        moveit_ground = normalized_pair_set(pose.get("moveit_ground_proxy_pairs"), f"pose[{expected_index}].moveit_ground")
        mujoco_ground = normalized_pair_set(pose.get("mujoco_ground_proxy_pairs"), f"pose[{expected_index}].mujoco_ground")
        self_match = moveit_self == mujoco_self
        ground_match = moveit_ground == mujoco_ground
        accepted_match = bool_match and self_match
        boolean_mismatch_count += int(not bool_match)
        self_pair_mismatch_count += int(not self_match)
        ground_pair_diagnostic_mismatch_count += int(not ground_match)
        if not accepted_match:
            mismatch_indices.append(expected_index)
        require(pose.get("overall_collision_boolean_match") is (moveit_flags[0] == mujoco_flags[0]), f"stored 503 overall flag mismatch: {expected_index}")
        require(pose.get("self_collision_boolean_match") is (moveit_flags[1] == mujoco_flags[1]), f"stored 503 self flag mismatch: {expected_index}")
        require(pose.get("ground_collision_boolean_match") is (moveit_flags[2] == mujoco_flags[2]), f"stored 503 ground flag mismatch: {expected_index}")
        require(pose.get("self_collision_pair_set_match") is self_match, f"stored 503 self-pair flag mismatch: {expected_index}")
        require(pose.get("ground_collision_pair_set_match") is ground_match, f"stored 503 ground-pair flag mismatch: {expected_index}")
        require(pose.get("collision_match") is accepted_match, f"stored 503 accepted-match flag mismatch: {expected_index}")
        canonical_rows.append({"global_index": expected_index, "category": category, "pose_index": pose_index, "angles_deg": angle_values, "moveit": moveit_flags, "mujoco": mujoco_flags, "moveit_self_pairs": sorted(moveit_self), "mujoco_self_pairs": sorted(mujoco_self), "accepted_match": accepted_match})
    expected_counts = {"overall": 182, "self": 126, "ground": 118}
    gate_require(counts["moveit"] == expected_counts and counts["mujoco"] == expected_counts, "503 frozen collision counts differ")
    require(503 - len(mismatch_indices) == int(report.get("match_count", -1)), "503 match aggregate differs")
    require(len(mismatch_indices) == int(report.get("mismatch_count", -1)), "503 mismatch aggregate differs")
    require(boolean_mismatch_count == int(report.get("boolean_mismatch_count", -1)), "503 boolean mismatch aggregate differs")
    require(self_pair_mismatch_count == int(report.get("self_pair_set_mismatch_count", -1)), "503 self-pair mismatch aggregate differs")
    require(ground_pair_diagnostic_mismatch_count == int(report.get("ground_pair_set_diagnostic_mismatch_count", -1)), "503 ground-pair diagnostic aggregate differs")
    controls = report.get("reference_terminal_collision_negative_controls")
    require(isinstance(controls, list) and len(controls) == 5, "503 negative control list mismatch")
    identities: dict[str, list[int]] = {}
    for row in controls:
        key = (str(row.get("category")), int(row.get("pose_index", -1)))
        identities.setdefault(key[0], []).append(key[1])
        require(key in pose_by_key, f"503 negative control identity invalid: {key}")
        gate_require(row.get("pass") is True, f"503 negative control failed: {key}")
        pose = pose_by_key[key]
        gate_require(pose["moveit"]["collision"] is True and pose["mujoco"]["collision"] is True, f"503 negative control is not collision: {key}")
        gate_require(pose["moveit"]["self_collision"] is True and pose["mujoco"]["self_collision"] is True, f"503 negative control is not self-collision: {key}")
    for values in identities.values():
        values.sort()
    require(identities == {"coupled_random_seed_1513": [43, 73, 75, 83, 87]}, "503 negative control identity mismatch")
    pose_set_sha = canonical_json_digest(reconstructed_sets)
    require(pose_set_sha == FROZEN_POSE_SET_SHA256, "503 frozen pose-set SHA256 mismatch")
    return {
        "pass": not mismatch_indices,
        "method": "INDEPENDENT_RECOMPUTATION_FROM_ALL_503_RAW_POSE_RECORDS",
        "pose_count": len(poses),
        "pose_set_sha256": pose_set_sha,
        "canonical_pose_facts_sha256": canonical_json_digest(canonical_rows),
        "match_count": 503 - len(mismatch_indices),
        "mismatch_count": len(mismatch_indices),
        "mismatch_global_indices": mismatch_indices,
        "boolean_mismatch_count": boolean_mismatch_count,
        "self_pair_set_mismatch_count": self_pair_mismatch_count,
        "ground_pair_set_diagnostic_mismatch_count": ground_pair_diagnostic_mismatch_count,
        "collision_pose_counts": counts,
        "negative_control_identity": identities,
        "negative_controls_pass": True,
    }


def validate_collision_503_runtime(records: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    report = read_json(Path(records["collision_503_json"]["_path"]))
    require(report.get("revision") == "V15.14-MoveIt-MuJoCo-503-pose-collision-cross-regression", "503 report revision mismatch")
    inputs = report.get("inputs", {})
    require(str(inputs.get("model_sha256", "")).lower() == NEW_MJCF_SHA256, "503 runtime model hash mismatch")
    require(str(inputs.get("validator_sha256", "")).lower() == POSE_SOURCE_VALIDATOR_SHA256, "503 runtime pose-source validator hash mismatch")
    require(str(inputs.get("guard_sha256", "")).lower() == GUARD_SHA256, "503 runtime guard hash mismatch")
    gate_require(report.get("input_hashes_pass") is True, "503 aggregate input hash gate failed")
    gates = report.get("input_hash_gates", {})
    for name, expected in (("model", NEW_MJCF_SHA256), ("validator", POSE_SOURCE_VALIDATOR_SHA256), ("guard", GUARD_SHA256)):
        gate = gates.get(name, {})
        require(str(gate.get("actual_sha256", "")).lower() == expected, f"503 {name} actual hash differs")
        require(str(gate.get("expected_sha256", "")).lower() == expected, f"503 {name} expected hash differs")
        gate_require(gate.get("pass") is True, f"503 {name} input gate failed")
    audit = recompute_collision_503(report)
    gate_require(report.get("status") == "PASS", "503 runtime report status failed")
    gate_require(report.get("pose_count_gate_pass") is True and report.get("frozen_collision_counts_pass") is True, "503 stored hard gate failed")
    gate_require(report.get("all_collision_booleans_match") is True and report.get("all_collision_pair_sets_match") is True, "503 stored match gates failed")
    gate_require(report.get("negative_controls_pass") is True, "503 stored negative-control gate failed")
    gate_require(audit["pass"] is True and audit["match_count"] == 503 and audit["mismatch_count"] == 0, "independent 503 audit failed")
    return {"pass": True, "raw_report_status": "PASS", "runtime_model_sha256": NEW_MJCF_SHA256, "independent_pose_audit": audit}


def validate_six_body_readback_report(records: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    """Cross-check the runtime diagnostic; independent compilation is the hard gate."""
    report = read_json(Path(records["six_body_readback_json"]["_path"]))
    require(report.get("schema") == "go-m8010-arm-v15.17d-six-body-runtime-readback/1.0", "six-body readback schema mismatch")
    require(str(report.get("model_sha256", "")).lower() == NEW_MJCF_SHA256, "six-body readback model SHA mismatch")
    require(str(report.get("source_report_sha256", "")).lower() == "4c50f70b1a8b940e32be40d38b6ced25e99f49528046bf5a8f4d610cfdc36dde", "six-body diagnostic source report SHA mismatch")
    require(report.get("mujoco_version") == "3.11.0", "six-body diagnostic MuJoCo version mismatch")
    gate_require(report.get("gravity_enabled") is False, "six-body runtime readback used enabled gravity")
    authority = read_json(repo_path(INERTIA_AUTHORITY_REL))
    authority_rows = authority.get("links")
    require(isinstance(authority_rows, list) and len(authority_rows) == 6, "inertia authority link array missing")
    authority_by_link = {str(row.get("link")): row for row in authority_rows}
    require(set(authority_by_link) == set(LINKS), "inertia authority link identity mismatch")
    links = report.get("links")
    require(isinstance(links, dict) and set(links) == set(LINKS), "six-body readback link identity mismatch")
    output: dict[str, Any] = {}
    max_mass_error = 0.0
    max_com_error = 0.0
    max_tensor_error = 0.0
    for link in LINKS:
        row = links[link]
        require(isinstance(row, dict), f"six-body {link}: row malformed")
        mass = finite(row.get("mass_kg"), f"{link}.mass_kg")
        ipos = vector(row.get("body_ipos_m"), 3, f"{link}.body_ipos_m")
        iquat = vector(row.get("body_iquat_wxyz"), 4, f"{link}.body_iquat_wxyz")
        inertia = vector(row.get("body_inertia_principal_kg_m2"), 3, f"{link}.body_inertia_principal_kg_m2")
        require(all(value > 0.0 for value in inertia), f"{link}: compiled principal inertia non-positive")
        reconstructed = reconstruct_tensor(iquat, inertia)
        stored_reconstructed = matrix3(row.get("compiled_reconstructed_tensor_kg_m2"), f"{link}.compiled_reconstructed_tensor")
        require(relative_frobenius(stored_reconstructed, reconstructed) <= 1.0e-14, f"{link}: stored tensor reconstruction differs from independent reconstruction")
        frozen = authority_by_link[link]
        expected_mass = finite(frozen.get("mass_kg"), f"authority.{link}.mass")
        expected_com = vector(frozen.get("com_xyz_m_in_link_frame"), 3, f"authority.{link}.com")
        expected_tensor = matrix3(frozen.get("inertia_tensor_kg_m2"), f"authority.{link}.tensor")
        stored_frozen = matrix3(row.get("frozen_tensor_kg_m2"), f"{link}.stored_frozen_tensor")
        require(relative_frobenius(stored_frozen, expected_tensor) <= 1.0e-15, f"{link}: diagnostic frozen tensor differs from authority")
        mass_error = abs(mass - expected_mass)
        com_error = max_abs(actual - expected for actual, expected in zip(ipos, expected_com))
        tensor_error = relative_frobenius(reconstructed, expected_tensor)
        require(abs(finite(row.get("mass_error_kg"), f"{link}.stored_mass_error") - mass_error) <= 1.0e-15, f"{link}: stored mass error differs")
        require(abs(finite(row.get("com_error_m"), f"{link}.stored_com_error") - com_error) <= 1.0e-15, f"{link}: stored COM error differs")
        require(abs(finite(row.get("tensor_relative_frobenius_error"), f"{link}.stored_tensor_error") - tensor_error) <= 1.0e-15, f"{link}: stored tensor error differs")
        gate_require(row.get("pass") is True, f"{link}: diagnostic row pass is false")
        max_mass_error = max(max_mass_error, mass_error)
        max_com_error = max(max_com_error, com_error)
        max_tensor_error = max(max_tensor_error, tensor_error)
        gate_require(mass_error <= 1.0e-12, f"{link}: runtime mass differs from authority")
        gate_require(com_error <= 1.0e-12, f"{link}: runtime COM differs from authority")
        gate_require(tensor_error < 1.0e-9, f"{link}: runtime tensor reconstruction exceeds 1e-9")
        output[link] = {
            "mass_kg": mass,
            "body_ipos_m": ipos,
            "body_iquat_wxyz": iquat,
            "body_inertia_principal_kg_m2": inertia,
            "independently_reconstructed_tensor_kg_m2": reconstructed,
            "mass_error_kg": mass_error,
            "com_max_abs_error_m": com_error,
            "tensor_relative_frobenius_error": tensor_error,
            "pass": True,
        }
    require(abs(finite(report.get("max_tensor_relative_frobenius_error"), "readback.max_tensor_error") - max_tensor_error) <= 1.0e-15, "stored max tensor error differs")
    gate_require(report.get("pass") is True, "six-body diagnostic stored pass is false")
    return {
        "pass": True,
        "model_sha256": NEW_MJCF_SHA256,
        "mujoco_version": "3.11.0",
        "links": output,
        "max_mass_error_kg": max_mass_error,
        "max_com_error_m": max_com_error,
        "max_tensor_relative_error": max_tensor_error,
        "tensor_limit_strict_less_than": 1.0e-9,
    }


def mujoco_readback_child(model_path: Path, output_path: Path) -> int:
    import numpy as np  # type: ignore
    import mujoco  # type: ignore

    model = mujoco.MjModel.from_xml_path(str(model_path))
    links: dict[str, Any] = {}
    for link in LINKS:
        body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, link))
        if body_id < 0:
            raise RuntimeError(f"compiled body missing: {link}")
        links[link] = {
            "mass_kg": float(model.body_mass[body_id]),
            "body_ipos_m": np.asarray(model.body_ipos[body_id], dtype=float).tolist(),
            "body_iquat_wxyz": np.asarray(model.body_iquat[body_id], dtype=float).tolist(),
            "body_inertia_principal_kg_m2": np.asarray(model.body_inertia[body_id], dtype=float).tolist(),
        }
    payload = {
        "mujoco_version": str(mujoco.__version__),
        "model_nbody": int(model.nbody),
        "gravity_m_s2": np.asarray(model.opt.gravity, dtype=float).tolist(),
        "links": links,
    }
    output_path.write_text(json.dumps(payload, ensure_ascii=False, sort_keys=True), encoding="utf-8")
    return 0


def make_ascii_mjcf_mirror(source_xml: bytes, source_directory: Path, destination: Path) -> Path:
    root = ET.fromstring(source_xml)
    compiler = root.find("compiler")
    mesh_dir = source_directory / (compiler.get("meshdir", ".") if compiler is not None else ".")
    texture_dir = source_directory / (compiler.get("texturedir", ".") if compiler is not None else ".")
    assets = destination / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    file_assets = [element for element in root.findall("./asset/*") if element.get("file")]
    require(len(file_assets) == 1008, "ASCII mirror expected exactly 1008 file assets")
    for index, element in enumerate(file_assets):
        raw_file = str(element.get("file"))
        require(not Path(raw_file).is_absolute(), f"absolute MJCF asset path: {raw_file}")
        base = mesh_dir if local_name(element.tag) == "mesh" else texture_dir
        source = (base / PurePosixPath(raw_file)).resolve()
        require(source.is_file() and ROOT in source.parents, f"MJCF asset escaped audited repository or is missing: {raw_file}")
        target_name = f"asset_{index:04d}{source.suffix.lower() or '.bin'}"
        shutil.copyfile(source, assets / target_name)
        element.set("file", "assets/" + target_name)
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", ".")
    compiler.set("texturedir", ".")
    model_path = destination / "model.xml"
    ET.ElementTree(root).write(model_path, encoding="utf-8", xml_declaration=True)
    require(str(model_path).isascii(), "MuJoCo mirror path is not ASCII")
    return model_path


def validate_independent_mujoco_readback(interpreter: str | None) -> dict[str, Any]:
    candidates = [interpreter, os.environ.get("V15_17_MUJOCO_PYTHON"), sys.executable]
    selected: Path | None = None
    for candidate in candidates:
        if not candidate:
            continue
        resolved = Path(candidate).resolve()
        if not resolved.is_file():
            continue
        probe = run_process([str(resolved), "-c", "import mujoco,numpy; print(mujoco.__version__)"])
        if probe.returncode == 0 and probe.stdout.strip() == "3.11.0":
            selected = resolved
            break
    require(selected is not None, "no MuJoCo 3.11.0 interpreter available for independent production readback")
    with __import__("tempfile").TemporaryDirectory(prefix="v15_17d_readback_") as temporary:
        output = Path(temporary) / "readback.json"
        mirror = make_ascii_mjcf_mirror(repo_path(MJCF_REL).read_bytes(), repo_path(MJCF_REL).parent, Path(temporary) / "mirror")
        try:
            result = run_process([str(selected), str(Path(__file__).resolve()), "--mujoco-child", str(mirror), str(output)], timeout=180.0)
        except subprocess.TimeoutExpired as error:
            raise GateFailure("independent MuJoCo readback child exceeded 180 seconds") from error
        gate_require(result.returncode == 0, "independent MuJoCo readback child failed: " + sanitize_error(result.stderr or result.stdout))
        gate_require(output.is_file(), "independent MuJoCo readback child produced no output")
        raw = read_json(output)
    require(raw.get("mujoco_version") == "3.11.0", "independent MuJoCo runtime version mismatch")
    gate_require(vector(raw.get("gravity_m_s2"), 3, "compiled gravity") == [0.0, 0.0, 0.0], "independent compiled gravity is enabled")
    authority_rows = read_json(repo_path(INERTIA_AUTHORITY_REL)).get("links")
    require(isinstance(authority_rows, list), "inertia authority links missing")
    authority_by_link = {str(row.get("link")): row for row in authority_rows}
    links = raw.get("links")
    require(isinstance(links, dict) and set(links) == set(LINKS), "independent compiled readback link set mismatch")
    results: dict[str, Any] = {}
    maxima = {"mass_error_kg": 0.0, "com_error_m": 0.0, "tensor_relative_error": 0.0}
    for link in LINKS:
        row = links[link]
        frozen = authority_by_link[link]
        mass = finite(row.get("mass_kg"), f"{link}.mass")
        ipos = vector(row.get("body_ipos_m"), 3, f"{link}.ipos")
        iquat = vector(row.get("body_iquat_wxyz"), 4, f"{link}.iquat")
        inertia = vector(row.get("body_inertia_principal_kg_m2"), 3, f"{link}.principal")
        tensor = reconstruct_tensor(iquat, inertia)
        mass_error = abs(mass - finite(frozen.get("mass_kg"), f"{link}.frozen mass"))
        com_error = max_abs(left - right for left, right in zip(ipos, vector(frozen.get("com_xyz_m_in_link_frame"), 3, f"{link}.frozen com")))
        tensor_error = relative_frobenius(tensor, matrix3(frozen.get("inertia_tensor_kg_m2"), f"{link}.frozen tensor"))
        maxima["mass_error_kg"] = max(maxima["mass_error_kg"], mass_error)
        maxima["com_error_m"] = max(maxima["com_error_m"], com_error)
        maxima["tensor_relative_error"] = max(maxima["tensor_relative_error"], tensor_error)
        gate_require(mass_error <= 1.0e-12 and com_error <= 1.0e-12, f"{link}: independent mass/COM readback failed")
        gate_require(tensor_error < 1.0e-9, f"{link}: independent tensor readback exceeded 1e-9")
        results[link] = {"mass_kg": mass, "body_ipos_m": ipos, "body_iquat_wxyz": iquat, "body_inertia_principal_kg_m2": inertia, "independent_reconstructed_tensor_kg_m2": tensor, "mass_error_kg": mass_error, "com_error_m": com_error, "tensor_relative_frobenius_error": tensor_error, "pass": True}
    return {
        "pass": True,
        "interpreter_identity": {"python_version": run_process([str(selected), "--version"]).stdout.strip(), "mujoco_version": "3.11.0"},
        "model_sha256": NEW_MJCF_SHA256,
        "mujoco_compile_pass": True,
        "compiled_gravity_enabled": False,
        "links": results,
        "max_mass_error_kg": maxima["mass_error_kg"],
        "max_com_error_m": maxima["com_error_m"],
        "max_tensor_relative_frobenius_error": maxima["tensor_relative_error"],
        "tensor_limit_strict_less_than": 1.0e-9,
    }


def compare_named_vectors(
    baseline: Mapping[str, Any],
    current: Mapping[str, Any],
    names: Sequence[str],
    field: str,
    length: int,
    label: str,
) -> tuple[float, dict[str, float]]:
    require(set(baseline) >= set(names) and set(current) >= set(names), f"{label}: named identity set incomplete")
    errors: dict[str, float] = {}
    for name in names:
        left = vector(baseline[name].get(field), length, f"{label}.baseline.{name}.{field}")
        right = vector(current[name].get(field), length, f"{label}.current.{name}.{field}")
        errors[name] = max_abs(actual - expected for actual, expected in zip(right, left))
    return max(errors.values(), default=0.0), errors


def compare_tf_snapshots(
    left: Mapping[str, Any],
    right: Mapping[str, Any],
    tolerance: float,
    label: str,
    *,
    compare_moveit_tip: bool = True,
) -> float:
    require(left.get("world_root_count") == right.get("world_root_count") and left.get("world_roots") == right.get("world_roots"), f"{label}: world root summary differs")
    maximum = 0.0
    for section, names, vector_fields in (
        ("joints", JOINTS, ("origin_xyz_m", "origin_rpy_rad", "origin_quaternion_xyzw", "axis_xyz")),
        ("camera_frames", ("camera_link", "sim_camera_optical_frame"), ("position_m", "rpy_rad", "quaternion_xyzw")),
    ):
        left_rows, right_rows = left.get(section), right.get(section)
        require(isinstance(left_rows, dict) and isinstance(right_rows, dict) and set(left_rows) == set(right_rows) == set(names), f"{label}: {section} identities differ")
        for name in names:
            left_row, right_row = left_rows[name], right_rows[name]
            for identity in set(left_row) - set(vector_fields):
                require(left_row.get(identity) == right_row.get(identity), f"{label}: {section}.{name}.{identity} differs")
            for field in vector_fields:
                length = 4 if "quaternion" in field else 3
                error = max_abs(a - b for a, b in zip(vector(left_row.get(field), length, f"{label}.left.{field}"), vector(right_row.get(field), length, f"{label}.right.{field}")))
                maximum = max(maximum, error)
    for section in ("world_to_base_link", "tcp_nominal"):
        left_row, right_row = left.get(section), right.get(section)
        require(isinstance(left_row, dict) and isinstance(right_row, dict), f"{label}: {section} missing")
        for identity in set(left_row) - {"position_m", "rpy_rad", "quaternion_xyzw"}:
            require(left_row.get(identity) == right_row.get(identity), f"{label}: {section}.{identity} differs")
        for field in ("position_m", "rpy_rad", "quaternion_xyzw"):
            length = 4 if "quaternion" in field else 3
            error = max_abs(a - b for a, b in zip(vector(left_row.get(field), length, f"{label}.left.{field}"), vector(right_row.get(field), length, f"{label}.right.{field}")))
            maximum = max(maximum, error)
    if compare_moveit_tip:
        require(left.get("moveit_tip_link") == right.get("moveit_tip_link"), f"{label}: MoveIt tip summary differs")
    require(maximum <= tolerance, f"{label}: stored TF summary differs from its independently derived source by {maximum}")
    return maximum


def validate_runtime_tf_identity(reference: Mapping[str, Any], candidate: Mapping[str, Any], label: str) -> dict[str, Any]:
    """Classify a well-formed runtime-vs-frozen topology change as a deployment gate failure."""
    gate_require(candidate.get("world_root_count") == 1 and candidate.get("world_roots") == ["world"], f"{label}: world root identity changed")
    reference_joints = reference.get("joints")
    candidate_joints = candidate.get("joints")
    gate_require(isinstance(reference_joints, dict) and isinstance(candidate_joints, dict) and set(reference_joints) == set(candidate_joints) == set(JOINTS), f"{label}: J1-J6 identity set changed")
    for name in JOINTS:
        expected = reference_joints[name]
        actual = candidate_joints[name]
        for field in ("parent", "child", "type"):
            gate_require(actual.get(field) == expected.get(field), f"{label}: {name} {field} changed")
    for section in ("world_to_base_link", "tcp_nominal"):
        expected = reference.get(section)
        actual = candidate.get(section)
        gate_require(isinstance(expected, dict) and isinstance(actual, dict), f"{label}: {section} identity missing")
        for field in ("joint", "parent", "child"):
            gate_require(actual.get(field) == expected.get(field), f"{label}: {section} {field} changed")
    expected_cameras = reference.get("camera_frames")
    actual_cameras = candidate.get("camera_frames")
    camera_names = {"camera_link", "sim_camera_optical_frame"}
    gate_require(isinstance(expected_cameras, dict) and isinstance(actual_cameras, dict) and set(expected_cameras) == set(actual_cameras) == camera_names, f"{label}: camera-frame identity set changed")
    for name in sorted(camera_names):
        for field in ("joint", "parent", "child"):
            gate_require(actual_cameras[name].get(field) == expected_cameras[name].get(field), f"{label}: {name} {field} changed")
    gate_require(candidate.get("moveit_tip_link") == reference.get("moveit_tip_link") == "tcp_nominal", f"{label}: MoveIt tip identity changed")
    return {"pass": True, "world_root": "world", "joint_identity_count": 6, "camera_identity_count": 2, "moveit_tip_link": "tcp_nominal"}


def runtime_tf_legal_fail_self_test(reference: Mapping[str, Any]) -> dict[str, Any]:
    identity_mutation = json.loads(json.dumps(reference))
    identity_mutation["joints"]["J1"]["parent"] = "__V15_17D_MUTATED_PARENT__"
    identity_audit = audit_call(
        "RUNTIME_TF_GEOMETRY_FAILED",
        lambda: validate_runtime_tf_identity(reference, identity_mutation, "legal-fail identity mutation"),
    )
    require(identity_audit.get("pass") is False, "TF identity mutation did not produce an audited FAIL")
    require(identity_audit.get("failure_code") == "RUNTIME_TF_GEOMETRY_FAILED" and identity_audit.get("exception_type") == "GateFailure", "TF identity mutation used the wrong failure class")
    numeric_mutation = json.loads(json.dumps(reference))
    numeric_mutation["joints"]["J1"]["origin_xyz_m"][0] += 1.0e-6
    numeric_audit = audit_call(
        "RUNTIME_TF_GEOMETRY_FAILED",
        lambda: gate_require(
            compare_named_vectors(reference["joints"], numeric_mutation["joints"], JOINTS, "origin_xyz_m", 3, "legal-fail numeric mutation")[0] <= 1.0e-12,
            "legal-fail numeric mutation: TF origin regression exceeded 1e-12",
        ) or {"pass": True},
    )
    require(numeric_audit.get("pass") is False, "TF numeric mutation did not produce an audited FAIL")
    require(numeric_audit.get("failure_code") == "RUNTIME_TF_GEOMETRY_FAILED" and numeric_audit.get("exception_type") == "GateFailure", "TF numeric mutation used the wrong failure class")
    return {
        "pass": True,
        "mutations": ["J1_PARENT_CHILD_IDENTITY_REGRESSION", "J1_ORIGIN_NUMERIC_REGRESSION"],
        "expected_audit_valid": True,
        "expected_section_pass": False,
        "expected_validator_exit_code_for_audit_valid_fail": 0,
        "expected_failure_code": "RUNTIME_TF_GEOMETRY_FAILED",
        "observed_exception_types": [identity_audit["exception_type"], numeric_audit["exception_type"]],
    }


def validate_tf_geometry_runtime(records: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    report = read_json(Path(records["tf_geometry_json"]["_path"]))
    require(report.get("schema") == "go-m8010-arm-v15.17d-runtime-tf-geometry-regression/1.0", "TF/geometry evidence schema mismatch")
    require(str(report.get("model_sha256", "")).lower() == NEW_MJCF_SHA256, "TF/geometry evidence model SHA mismatch")
    require(report.get("baseline_identity") == "V15.14_ACCEPTED_TRAJECTORY_LOOP", "TF/geometry baseline identity mismatch")
    baseline = report.get("baseline")
    current = report.get("current")
    runtime = report.get("runtime")
    require(all(isinstance(value, dict) for value in (baseline, current, runtime)), "TF/geometry baseline/current/runtime snapshots missing")
    independent_baseline = urdf_kinematic_snapshot(ET.fromstring(git_blob(XACRO_REL, OLD_MODEL_COMMIT)))
    independent_current = urdf_kinematic_snapshot(ET.parse(repo_path(XACRO_REL)).getroot())
    source_baseline_error = compare_tf_snapshots(independent_baseline, baseline, 1.0e-15, "old frozen Xacro vs derived baseline")
    source_current_error = compare_tf_snapshots(independent_current, current, 1.0e-15, "current Xacro vs derived current")
    raw = Path(records["runtime_robot_description_txt"]["_path"]).read_text(encoding="utf-8", errors="strict")
    prefix = "String value is: "
    require(raw.startswith(prefix), "runtime robot_description wrapper prefix mismatch")
    runtime_root = ET.fromstring(raw[len(prefix):].strip())
    independent_runtime = urdf_kinematic_snapshot(runtime_root, runtime_identity_gate=True)
    raw_runtime_error = compare_tf_snapshots(independent_runtime, runtime, 1.0e-15, "raw robot_description vs derived runtime", compare_moveit_tip=False)
    legal_fail_self_test = runtime_tf_legal_fail_self_test(baseline)
    current_identity = validate_runtime_tf_identity(baseline, current, "current source vs frozen baseline")
    runtime_identity = validate_runtime_tf_identity(baseline, runtime, "fresh runtime vs frozen baseline")
    joint_position_max, joint_position_rows = compare_named_vectors(baseline.get("joints", {}), runtime.get("joints", {}), JOINTS, "origin_xyz_m", 3, "joint origins")
    joint_quaternion_max, joint_quaternion_rows = compare_named_vectors(baseline.get("joints", {}), runtime.get("joints", {}), JOINTS, "origin_quaternion_xyzw", 4, "joint origin quaternions")
    joint_axis_max, joint_axis_rows = compare_named_vectors(baseline.get("joints", {}), runtime.get("joints", {}), JOINTS, "axis_xyz", 3, "joint axes")
    tcp_position_max, _ = compare_named_vectors({"tcp_nominal": baseline.get("tcp_nominal", {})}, {"tcp_nominal": runtime.get("tcp_nominal", {})}, ("tcp_nominal",), "position_m", 3, "tcp")
    tcp_quaternion_max, _ = compare_named_vectors({"tcp_nominal": baseline.get("tcp_nominal", {})}, {"tcp_nominal": runtime.get("tcp_nominal", {})}, ("tcp_nominal",), "quaternion_xyzw", 4, "tcp")
    camera_names = ("camera_link", "sim_camera_optical_frame")
    camera_position_max, camera_position_rows = compare_named_vectors(baseline.get("camera_frames", {}), runtime.get("camera_frames", {}), camera_names, "position_m", 3, "camera")
    camera_quaternion_max, camera_quaternion_rows = compare_named_vectors(baseline.get("camera_frames", {}), runtime.get("camera_frames", {}), camera_names, "quaternion_xyzw", 4, "camera")
    digests = report.get("semantic_digests")
    require(isinstance(digests, dict), "TF semantic digests missing")
    recomputed_digests = {name: canonical_json_digest(snapshot) for name, snapshot in (("baseline", baseline), ("current", current), ("runtime", runtime))}
    stored_digests = {name: str(digests.get(name + "_sha256", "")).lower() for name in ("baseline", "current", "runtime")}
    require(stored_digests == recomputed_digests, "TF stored semantic digests differ from independent recomputation")
    gate_require(len(set(recomputed_digests.values())) == 1, "three-source TF semantic digests differ")
    all_errors = [joint_position_max, joint_quaternion_max, joint_axis_max, tcp_position_max, tcp_quaternion_max, camera_position_max, camera_quaternion_max]
    gate_require(max(all_errors) <= 1.0e-12, "TF/origin/axis/TCP/camera regression exceeded 1e-12")
    return {
        "pass": True,
        "world_root_unique": True,
        "joint_origin_max_abs_error": max(joint_position_max, joint_quaternion_max),
        "joint_axis_max_abs_error": joint_axis_max,
        "tcp_max_abs_error": max(tcp_position_max, tcp_quaternion_max),
        "camera_max_abs_error": max(camera_position_max, camera_quaternion_max),
        "moveit_tip_link": "tcp_nominal",
        "joint_origin_errors": {"position": joint_position_rows, "quaternion": joint_quaternion_rows},
        "joint_axis_errors": joint_axis_rows,
        "camera_errors": {"position": camera_position_rows, "quaternion": camera_quaternion_rows},
        "absolute_tolerance": 1.0e-15,
        "three_source_semantic_digest": recomputed_digests["runtime"],
        "runtime_robot_description_sha256": records["runtime_robot_description_txt"]["sha256"],
        "raw_robot_description_vs_runtime_max_abs_error": raw_runtime_error,
        "old_xacro_vs_baseline_max_abs_error": source_baseline_error,
        "current_xacro_vs_current_max_abs_error": source_current_error,
        "current_source_identity_gate": current_identity,
        "fresh_runtime_identity_gate": runtime_identity,
        "legal_fail_mutation_self_test": legal_fail_self_test,
    }


def rpy_to_quaternion_xyzw(rpy: Sequence[float]) -> list[float]:
    roll, pitch, yaw = (float(value) for value in rpy)
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return [sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy]


def urdf_origin(node: ET.Element | None) -> tuple[list[float], list[float], list[float]]:
    if node is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]
    xyz = vector([float(token) for token in str(node.get("xyz", "0 0 0")).split()], 3, "URDF origin xyz")
    rpy = vector([float(token) for token in str(node.get("rpy", "0 0 0")).split()], 3, "URDF origin rpy")
    return xyz, rpy, rpy_to_quaternion_xyzw(rpy)


def urdf_kinematic_snapshot(root: ET.Element, *, runtime_identity_gate: bool = False) -> dict[str, Any]:
    links = {str(node.get("name")) for node in root.findall("./link") if node.get("name")}
    joint_nodes = {str(node.get("name")): node for node in root.findall("./joint") if node.get("name")}
    parented = {str(node.find("./child").get("link")) for node in joint_nodes.values() if node.find("./child") is not None}
    roots = sorted(links - parented)
    if not runtime_identity_gate:
        require(roots == ["world"], f"URDF world root identity changed: {roots}")
    joint_rows: dict[str, Any] = {}
    for name in JOINTS:
        (gate_require if runtime_identity_gate else require)(name in joint_nodes, f"runtime URDF joint missing: {name}")
        node = joint_nodes[name]
        parent = node.find("./parent")
        child = node.find("./child")
        (gate_require if runtime_identity_gate else require)(parent is not None and child is not None, f"runtime URDF {name} parent/child missing")
        xyz, rpy, quaternion = urdf_origin(node.find("./origin"))
        axis_node = node.find("./axis")
        axis = vector([float(token) for token in str(axis_node.get("xyz", "")).split()], 3, f"runtime URDF {name} axis") if axis_node is not None else [0.0, 0.0, 0.0]
        joint_rows[name] = {"parent": str(parent.get("link")), "child": str(child.get("link")), "type": str(node.get("type")), "origin_xyz_m": xyz, "origin_rpy_rad": rpy, "origin_quaternion_xyzw": quaternion, "axis_xyz": axis}

    def fixed_row(joint_name: str) -> dict[str, Any]:
        (gate_require if runtime_identity_gate else require)(joint_name in joint_nodes, f"runtime URDF fixed joint missing: {joint_name}")
        node = joint_nodes[joint_name]
        (gate_require if runtime_identity_gate else require)(node.get("type") == "fixed", f"runtime URDF {joint_name} is not fixed")
        parent = node.find("./parent")
        child = node.find("./child")
        (gate_require if runtime_identity_gate else require)(parent is not None and child is not None, f"runtime URDF {joint_name} parent/child missing")
        xyz, rpy, quaternion = urdf_origin(node.find("./origin"))
        return {"joint": joint_name, "parent": str(parent.get("link")), "child": str(child.get("link")), "position_m": xyz, "rpy_rad": rpy, "quaternion_xyzw": quaternion}

    world_base_candidates = [name for name, node in joint_nodes.items() if node.find("./parent") is not None and node.find("./parent").get("link") == "world" and node.find("./child") is not None and node.find("./child").get("link") == "base_link"]
    tcp_candidates = [name for name, node in joint_nodes.items() if node.find("./child") is not None and node.find("./child").get("link") == "tcp_nominal"]
    camera_candidates = [name for name, node in joint_nodes.items() if node.find("./child") is not None and node.find("./child").get("link") == "camera_link"]
    optical_candidates = [name for name, node in joint_nodes.items() if node.find("./child") is not None and node.find("./child").get("link") == "sim_camera_optical_frame"]
    (gate_require if runtime_identity_gate else require)(all(len(values) == 1 for values in (world_base_candidates, tcp_candidates, camera_candidates, optical_candidates)), "runtime URDF fixed-frame identity ambiguity")
    return {
        "world_root_count": len(roots),
        "world_roots": roots,
        "world_to_base_link": fixed_row(world_base_candidates[0]),
        "joints": joint_rows,
        "tcp_nominal": fixed_row(tcp_candidates[0]),
        "camera_frames": {"camera_link": fixed_row(camera_candidates[0]), "sim_camera_optical_frame": fixed_row(optical_candidates[0])},
        "moveit_tip_link": "tcp_nominal",
    }


def validate_controller_graph(records: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    report = read_json(Path(records["controller_graph_json"]["_path"]))
    require(report.get("schema") == "go-m8010-arm-v15.17d-controller-graph/1.0", "controller graph schema mismatch")
    expected_controllers = [
        {"name": "joint_state_broadcaster", "type": "joint_state_broadcaster/JointStateBroadcaster", "state": "active"},
        {"name": "arm_controller", "type": "joint_trajectory_controller/JointTrajectoryController", "state": "active"},
    ]
    gate_require(report.get("controllers") == expected_controllers, "controller types/states changed")
    expected_actions = [
        "/arm_controller/follow_joint_trajectory [control_msgs/action/FollowJointTrajectory]",
        "/execute_trajectory [moveit_msgs/action/ExecuteTrajectory]",
        "/move_action [moveit_msgs/action/MoveGroup]",
    ]
    gate_require(report.get("actions") == expected_actions, "runtime action set changed")
    required_nodes = [
        "/mujoco_bridge",
        "/move_group",
        "/controller_manager",
        "/robot_state_publisher",
        "/arm_controller",
        "/joint_state_broadcaster",
    ]
    require(report.get("required_nodes") == required_nodes, "required runtime node contract changed")
    nodes = report.get("nodes")
    require(isinstance(nodes, list), "runtime node list malformed")
    gate_require(set(required_nodes) <= set(map(str, nodes)), "required runtime node is missing")
    gate_require(report.get("joint_states_publishers") == [{"node_name": "joint_state_broadcaster", "namespace": "/"}], "public joint-state source changed")
    source_raw = report.get("source_raw")
    expected_source_map = {
        "controllers": "controllers_post_txt",
        "actions": "actions_post_txt",
        "joint_states": "joint_states_info_post_txt",
        "nodes": "nodes_post_txt",
    }
    require(isinstance(source_raw, dict), "controller graph source_raw map missing")
    for source_key, artifact_key in expected_source_map.items():
        require(Path(str(source_raw.get(source_key))).name == Path(records[artifact_key]["logical_path"]).name, f"controller graph raw source mapping mismatch: {source_key}")
    ansi = re.compile(r"\x1b\[[0-?]*[ -/]*[@-~]")
    controllers_text = ansi.sub("", Path(records["controllers_post_txt"]["_path"]).read_text(encoding="utf-8", errors="replace"))
    actions_text = ansi.sub("", Path(records["actions_post_txt"]["_path"]).read_text(encoding="utf-8", errors="replace"))
    nodes_text = ansi.sub("", Path(records["nodes_post_txt"]["_path"]).read_text(encoding="utf-8", errors="replace"))
    joint_info = ansi.sub("", Path(records["joint_states_info_post_txt"]["_path"]).read_text(encoding="utf-8", errors="replace"))
    raw_controller_rows = [line.strip() for line in controllers_text.splitlines() if line.strip() and not line.lstrip().startswith(("[", "Waiting"))]
    parsed_controller_rows = []
    for line in raw_controller_rows:
        match = re.fullmatch(r"([A-Za-z0-9_]+)\s+([^\s]+)\s+(active|inactive|unconfigured)", line)
        require(match is not None, f"unparsed raw controller line: {line}")
        parsed_controller_rows.append({"name": match.group(1), "type": match.group(2), "state": match.group(3)})
    require(len(parsed_controller_rows) == len(raw_controller_rows), "raw controller line was silently dropped")
    gate_require(parsed_controller_rows == expected_controllers, "raw controller list is not exact expected two controllers")
    raw_actions = [line.strip() for line in actions_text.splitlines() if line.strip()]
    gate_require(raw_actions == expected_actions, "raw action list is not exact expected three actions")
    raw_nodes = [line.strip() for line in nodes_text.splitlines() if line.strip()]
    require(len(raw_nodes) == len(set(raw_nodes)), "raw node list contains duplicates")
    require(raw_nodes == nodes, "parsed node JSON differs from raw node list")
    gate_require(all(node in raw_nodes for node in required_nodes), "raw node list misses required node")
    publisher_match = re.search(r"Publisher count:\s*(\d+)(.*?)(?:Subscription count:|$)", joint_info, re.S)
    require(publisher_match is not None, "raw /joint_states publisher section malformed")
    gate_require(int(publisher_match.group(1)) == 1, "raw /joint_states publisher count is not one")
    publisher_section = publisher_match.group(2)
    publisher_names = re.findall(r"^Node name:\s*(\S+)\s*$", publisher_section, re.M)
    publisher_namespaces = re.findall(r"^Node namespace:\s*(\S+)\s*$", publisher_section, re.M)
    gate_require(publisher_names == ["joint_state_broadcaster"] and publisher_namespaces == ["/"], "raw /joint_states authority is not sole root joint_state_broadcaster")
    gate_require(report.get("pass") is True, "runtime controller graph gate failed")
    return {
        "pass": True,
        "moveit_load_pass": True,
        "ros2_control_load_pass": True,
        "standard_fjt_action_pass": True,
        "single_public_joint_state_authority": True,
        "controllers": expected_controllers,
        "actions": expected_actions,
        "required_nodes": required_nodes,
        "joint_states_publishers": report["joint_states_publishers"],
        "raw_source_hashes": {key: records[value]["sha256"] for key, value in expected_source_map.items()},
    }


def parent_map(root: ET.Element) -> dict[int, ET.Element]:
    return {id(child): parent for parent in root.iter() for child in list(parent)}


def mjcf_kinematic_snapshot(root: ET.Element) -> dict[str, Any]:
    parents = parent_map(root)
    bodies = body_map(root)
    joints = {str(node.get("name")): node for node in root.findall(".//joint") if node.get("name")}
    sites = {str(node.get("name")): node for node in root.findall(".//site") if node.get("name")}
    require(set(JOINTS) <= set(joints), "MJCF kinematic joint set incomplete")
    body_chain = {
        "base_link": None,
        "link1": "base_link",
        "link2": "link1",
        "link3": "link2",
        "link4": "link3",
        "link5": "link4",
        "link6": "link5",
        "gripper": "link6",
        "camera_link": "gripper",
        "sim_camera_optical_frame": "camera_link",
    }
    body_rows: dict[str, Any] = {}
    for name, expected_parent in body_chain.items():
        require(name in bodies, f"MJCF body missing: {name}")
        node = bodies[name]
        parent = parents.get(id(node))
        while parent is not None and local_name(parent.tag) != "body":
            parent = parents.get(id(parent))
        parent_name = str(parent.get("name")) if parent is not None else None
        require(parent_name == expected_parent, f"MJCF parent identity changed: {name}")
        position = vector([float(token) for token in str(node.get("pos", "0 0 0")).split()], 3, f"{name}.pos")
        quaternion = vector([float(token) for token in str(node.get("quat", "1 0 0 0")).split()], 4, f"{name}.quat")
        body_rows[name] = {"parent": parent_name, "position_m": position, "quaternion_wxyz": quaternion}
    joint_rows: dict[str, Any] = {}
    expected_joint_parents = {f"J{index}": f"link{index}" for index in range(1, 7)}
    for name in JOINTS:
        node = joints[name]
        parent = parents.get(id(node))
        require(parent is not None and local_name(parent.tag) == "body", f"{name}: direct body parent missing")
        require(parent.get("name") == expected_joint_parents[name], f"{name}: parent body changed")
        joint_rows[name] = {
            "parent_body": str(parent.get("name")),
            "position_m": vector([float(token) for token in str(node.get("pos", "0 0 0")).split()], 3, f"{name}.pos"),
            "axis_xyz": vector([float(token) for token in str(node.get("axis", "")).split()], 3, f"{name}.axis"),
            "type": str(node.get("type")),
            "range": str(node.get("range", "")),
            "limited": str(node.get("limited", "")),
            "damping": str(node.get("damping", "")),
            "frictionloss": str(node.get("frictionloss", "")),
            "armature": str(node.get("armature", "")),
        }
    require("tcp_nominal" in sites, "tcp_nominal site missing")
    tcp = sites["tcp_nominal"]
    tcp_parent = parents.get(id(tcp))
    require(tcp_parent is not None and tcp_parent.get("name") == "gripper", "tcp_nominal parent changed")
    tcp_row = {
        "parent_body": "gripper",
        "position_m": vector([float(token) for token in str(tcp.get("pos", "")).split()], 3, "tcp_nominal.pos"),
        "quaternion_wxyz": vector([float(token) for token in str(tcp.get("quat", "1 0 0 0")).split()], 4, "tcp_nominal.quat"),
    }
    worldbody = root.find("./worldbody")
    require(worldbody is not None, "MJCF worldbody missing")
    direct_robot_roots = [body.get("name") for body in list(worldbody) if local_name(body.tag) == "body" and body.get("name")]
    require(direct_robot_roots == ["base_link"], f"MJCF robot root set changed: {direct_robot_roots}")
    return {"world_root_count": len(direct_robot_roots), "world_roots": direct_robot_roots, "bodies": body_rows, "joints": joint_rows, "tcp_nominal": tcp_row}


def validate_static_tf_geometry() -> dict[str, Any]:
    old_xacro_hash = sha256_bytes(git_blob(XACRO_REL, OLD_MODEL_COMMIT))
    base_xacro_hash = sha256_bytes(git_blob(XACRO_REL, BASE_COMMIT))
    current_xacro_hash = sha256_file(repo_path(XACRO_REL))
    old_srdf_hash = sha256_bytes(git_blob(SRDF_REL, OLD_MODEL_COMMIT))
    base_srdf_hash = sha256_bytes(git_blob(SRDF_REL, BASE_COMMIT))
    current_srdf_hash = sha256_file(repo_path(SRDF_REL))
    require(old_xacro_hash == OLD_XACRO_GIT_LF_SHA256, "old accepted Xacro Git-blob hash mismatch")
    require(base_xacro_hash == BASE_XACRO_GIT_LF_SHA256, "base/current Xacro Git-blob hash mismatch")
    require(current_xacro_hash == CURRENT_XACRO_CHECKOUT_SHA256, "current Xacro checkout hash mismatch")
    require(old_srdf_hash == SRDF_GIT_LF_SHA256 and base_srdf_hash == SRDF_GIT_LF_SHA256, "old/base SRDF Git-blob hash mismatch")
    require(current_srdf_hash == SRDF_CHECKOUT_SHA256, "current SRDF checkout hash mismatch")
    old = mjcf_kinematic_snapshot(ET.fromstring(git_blob(MJCF_REL, OLD_MODEL_COMMIT)))
    current = mjcf_kinematic_snapshot(ET.parse(repo_path(MJCF_REL)).getroot())
    require(old == current, "old/new production kinematic/TF semantic snapshot changed")
    require(all(max_abs(row["position_m"]) == 0.0 for row in current["joints"].values()), "J1-J6 joint position is not exact zero")
    srdf_root = ET.parse(repo_path(SRDF_REL)).getroot()
    arm_groups = [group for group in srdf_root.findall("./group") if group.get("name") == "arm"]
    require(len(arm_groups) == 1, "SRDF arm group identity mismatch")
    chains = arm_groups[0].findall("./chain")
    require(len(chains) == 1 and chains[0].get("base_link") == "base_link" and chains[0].get("tip_link") == "tcp_nominal", "SRDF MoveIt chain/tip changed")
    end_effectors = [row for row in srdf_root.findall("./end_effector") if row.get("name") == "tcp_nominal"]
    require(len(end_effectors) == 1 and end_effectors[0].get("parent_link") == "gripper" and end_effectors[0].get("parent_group") == "arm", "SRDF end effector contract changed")
    current["moveit_tip_link"] = "tcp_nominal"
    snapshot_sha = canonical_json_digest(current)
    return {
        "pass": True,
        "comparison": "OLD_ACCEPTED_MJCF_VS_CURRENT_PRODUCTION_MJCF_SOURCE_SEMANTICS",
        "canonical_snapshot_sha256": snapshot_sha,
        "world_root_unique": True,
        "world_to_base_link_unchanged": True,
        "joint_origins_unchanged": True,
        "joint_axes_unchanged": True,
        "tcp_nominal_unchanged": True,
        "camera_frames_unchanged": True,
        "moveit_tip_link": "tcp_nominal",
        "snapshot": current,
        "source_provenance": {
            "old_xacro_git_lf_sha256": old_xacro_hash,
            "base_xacro_git_lf_sha256": base_xacro_hash,
            "current_xacro_checkout_sha256": current_xacro_hash,
            "srdf_git_lf_sha256": base_srdf_hash,
            "srdf_checkout_sha256": current_srdf_hash,
        },
    }


def current_attempt_input_snapshot(
    records: Mapping[str, Mapping[str, Any]],
    protected_report: Mapping[str, Any],
) -> dict[str, str]:
    return {
        "production_mjcf_sha256": sha256_file(repo_path(MJCF_REL)),
        "production_bridge_sha256": sha256_file(repo_path(BRIDGE_REL)),
        "trajectory_qa_source_sha256": sha256_file(repo_path(QA_TOOL_REL)),
        "trajectory_qa_runtime_copy_sha256": sha256_bytes(Path(records["trajectory_qa_tool"]["_path"]).read_bytes().replace(b"\r\n", b"\n")),
        "collision_503_source_sha256": sha256_file(repo_path(CROSS_503_REL)),
        "collision_503_runtime_copy_sha256": sha256_bytes(Path(records["collision_503_tool"]["_path"]).read_bytes().replace(b"\r\n", b"\n")),
        "target_manifest_sha256": sha256_file(repo_path(TARGET_MANIFEST_REL)),
        "collision_contract_sha256": sha256_file(repo_path(COLLISION_CONTRACT_REL)),
        "kinematic_guard_sha256": sha256_file(repo_path(GUARD_REL)),
        "runtime_mesh_manifest_sha256": sha256_file(repo_path(RUNTIME_MESH_MANIFEST_REL)),
        "source_mesh_manifest_sha256": sha256_file(repo_path(SOURCE_MESH_MANIFEST_REL)),
        "runtime_asset_set_sha256": RUNTIME_ASSET_SET_SHA256,
        "protected_path_set_sha256": str(protected_report["protected_path_set_sha256"]),
    }


GATE_DEFINITIONS = (
    (1, "git_base_branch_exact4_remote", "git_scope_guard"),
    (2, "old_new_production_model_hashes", "model_semantic_diff"),
    (3, "semantic_only_six_accepted_inertials", "model_semantic_diff"),
    (4, "bridge_only_model_anchor_migrated", "bridge_hash_migration"),
    (5, "old_hash_references_classified", "old_hash_reference_audit"),
    (6, "mesh_1008_manifest_asset_authority", "mesh_authority"),
    (7, "collision_contract_geoms_pairs_unchanged", "collision_authority"),
    (8, "fresh_colcon_current_install_binding", "runtime_boot"),
    (9, "production_bridge_hash_authority_compile", "runtime_boot"),
    (10, "six_body_independent_runtime_readback", "independent_mujoco_readback"),
    (11, "moveit_load", "controller_graph"),
    (12, "ros2_control_load", "controller_graph"),
    (13, "standard_fjt_and_joint_state_authority", "trajectory_loop"),
    (14, "all_expected_success_targets", "trajectory_loop"),
    (15, "all_expected_rejection_targets", "trajectory_loop"),
    (16, "v15_14_numeric_thresholds", "trajectory_loop"),
    (17, "fresh_current_model_503_collision", "collision_503_runtime"),
    (18, "tf_origin_axis_tcp_camera_unchanged", "tf_geometry_regression"),
    (19, "visual_limitation_gravity_controller_prohibitions", "prohibited_actions"),
    (20, "protected_toctou_and_hard_unresolved_empty", "toctou_guard"),
)


def public_git_scope(scope: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "pass": scope.get("pass") is True,
        "branch": TARGET_BRANCH,
        "source_commit": BASE_COMMIT,
        "head_contract": "BASE_COMMIT_OR_ITS_UNIQUE_DIRECT_CHILD",
        "remote_base_ref": REMOTE_BASE_REF,
        "remote_base_commit": BASE_COMMIT,
        "changed_paths": sorted(EXACT_CHANGED_PATHS),
        "exact_changed_path_count": 4,
        "allowed_changed_paths": sorted(EXACT_CHANGED_PATHS),
    }


def final_report_items(facts: Mapping[str, Any], unresolved: Sequence[str]) -> list[dict[str, Any]]:
    values = (
        {"branch": TARGET_BRANCH, "source_commit": BASE_COMMIT, "head_contract": "BASE_COMMIT_OR_ITS_UNIQUE_DIRECT_CHILD"},
        OLD_MJCF_SHA256,
        NEW_MJCF_SHA256,
        facts.get("semantic_only_inertial_changes", "NO"),
        facts.get("modified_hash_anchors", []),
        facts.get("mesh_authority", "FAIL"),
        facts.get("collision_contract", "FAIL"),
        facts.get("production_bridge_model_hash", "FAIL"),
        facts.get("mujoco_compile", "FAIL"),
        facts.get("six_body_max_tensor_error"),
        facts.get("moveit_load", "FAIL"),
        facts.get("ros2_control_load", "FAIL"),
        facts.get("follow_joint_trajectory", "FAIL"),
        facts.get("expected_success_targets", "0/10"),
        facts.get("expected_rejection_targets", "0/2"),
        facts.get("max_final_joint_error_rad"),
        facts.get("max_tcp_position_error_m"),
        facts.get("max_tcp_orientation_error_rad"),
        facts.get("max_rviz_mujoco_joint_sync_error_rad"),
        facts.get("collision_regression", {"match": "0/503", "mismatch_count": None}),
        facts.get("joint_origins_unchanged", "NO"),
        facts.get("joint_axes_unchanged", "NO"),
        facts.get("tcp_unchanged", "NO"),
        facts.get("camera_tf_unchanged", "NO"),
        {"limitation": VISUAL_LIMITATION, "still_present": facts.get("visual_limitation_present", "NO")},
        facts.get("gravity_enabled", "UNKNOWN"),
        facts.get("controller_modified", "UNKNOWN"),
        list(unresolved),
        facts.get("final_status", FINAL_FAIL_STATUS),
    )
    labels = (
        "branch / commit", "old MJCF SHA", "new MJCF SHA", "semantic diff only inertial", "modified hash anchors",
        "mesh asset authority", "collision contract", "production bridge model hash", "MuJoCo compile", "six-body max tensor error",
        "MoveIt load", "ros2_control load", "FollowJointTrajectory action", "expected-success targets", "expected-rejection targets",
        "max final joint error", "max TCP position error", "max TCP orientation error", "max RViz/MuJoCo joint sync error",
        "collision regression", "J1-J6 origin unchanged", "J1-J6 axis unchanged", "TCP unchanged", "camera TF unchanged",
        "visual link6 limitation", "gravity enabled", "controller modified", "hard unresolved items", "final status",
    )
    return [{"number": index, "label": label, "value": value} for index, (label, value) in enumerate(zip(labels, values), 1)]


def build_report(runtime_evidence_path: Path | None, mujoco_python: str | None, *, allow_prewrite: bool) -> dict[str, Any]:
    repo_watch = sorted(PROTECTED_RELS | EXACT_CHANGED_PATHS | {MJCF_REL, BRIDGE_REL, VALIDATOR_REL})
    before = capture_repo_hashes(repo_watch)
    git_scope = validate_git_scope(allow_prewrite=allow_prewrite)
    model = validate_model_semantics()
    bridge = validate_bridge_migration()
    references = validate_old_hash_references()
    collision_contract = validate_collision_contract()
    mesh, mesh_assets = validate_mesh_authority()
    prior_c = validate_prior_v15_17c()
    protected = validate_static_protected_files()
    static_tf = validate_static_tf_geometry()

    runtime_records_public: dict[str, Any] = {}
    evidence_hashes: dict[str, str] = {}
    evidence_records: dict[str, dict[str, Any]] = {}
    evidence_root: Path | None = None
    if runtime_evidence_path is None:
        runtime_missing = {"pass": False, "failure_code": "RUNTIME_EVIDENCE_NOT_PROVIDED", "exception_type": "GateFailure", "reason": "repo-external V15.17D runtime evidence was not provided"}
        envelope: dict[str, Any] = {}
        runtime_tools = runtime_boot = attempt_history = trajectory = collision_503 = six_diag = tf_runtime = controller = runtime_missing
    else:
        envelope, evidence_records, evidence_hashes, evidence_root = load_runtime_envelope(runtime_evidence_path)
        runtime_records_public = {name: {key: value for key, value in row.items() if not key.startswith("_")} for name, row in evidence_records.items()}
        runtime_tools = audit_call("RUNTIME_TOOL_PROVENANCE_FAILED", lambda: validate_runtime_tools(evidence_records), evidence_root)
        runtime_boot = audit_call("PRODUCTION_RUNTIME_BOOT_FAILED", lambda: validate_runtime_boot(envelope, evidence_records), evidence_root)
        current_snapshot = current_attempt_input_snapshot(evidence_records, protected)
        attempt_history = audit_call("TRAJECTORY_ATTEMPT_HISTORY_FAILED", lambda: validate_attempt_history(envelope, evidence_records, current_snapshot), evidence_root)
        trajectory = audit_call("V15_14_TRAJECTORY_LOOP_FAILED", lambda: validate_trajectory_qa(evidence_records), evidence_root)
        collision_503 = audit_call("COLLISION_503_REGRESSION_FAILED", lambda: validate_collision_503_runtime(evidence_records), evidence_root)
        six_diag = audit_call("SIX_BODY_RUNTIME_DIAGNOSTIC_FAILED", lambda: validate_six_body_readback_report(evidence_records), evidence_root)
        tf_runtime = audit_call("RUNTIME_TF_GEOMETRY_FAILED", lambda: validate_tf_geometry_runtime(evidence_records), evidence_root)
        controller = audit_call("RUNTIME_CONTROLLER_GRAPH_FAILED", lambda: validate_controller_graph(evidence_records), evidence_root)
    independent_readback = audit_call("INDEPENDENT_MUJOCO_READBACK_FAILED", lambda: validate_independent_mujoco_readback(mujoco_python))

    after = capture_repo_hashes(repo_watch)
    mesh_after = {relative: sha256_file(repo_path(relative)) for relative in sorted(mesh_assets)}
    require(mesh_after == mesh_assets, "runtime mesh asset changed during validation")
    if runtime_evidence_path is not None:
        rehash_runtime_artifacts(runtime_evidence_path, evidence_records, evidence_hashes)
    git_after = validate_git_scope(allow_prewrite=allow_prewrite)
    runtime_head_before = git_scope.pop("_runtime_head_commit", None)
    runtime_head_after = git_after.pop("_runtime_head_commit", None)
    toctou_pass = before == after and runtime_head_before == runtime_head_after and public_git_scope(git_scope) == public_git_scope(git_after)
    toctou = {
        "pass": toctou_pass,
        "repo_input_count": len(before),
        "repo_inputs_unchanged": before == after,
        "runtime_head_unchanged": runtime_head_before == runtime_head_after,
        "mesh_asset_count": len(mesh_assets),
        "all_1008_mesh_assets_rehashed_at_end": mesh_after == mesh_assets,
        "runtime_evidence_artifact_count": len(evidence_hashes) - int("__envelope__" in evidence_hashes),
        "runtime_evidence_rehashed_at_end": runtime_evidence_path is None or bool(evidence_hashes),
        "git_scope_revalidated_at_end": public_git_scope(git_scope) == public_git_scope(git_after),
    }

    gravity_off = model.get("gravity_enabled") is False and independent_readback.get("compiled_gravity_enabled") is False
    controller_unchanged = protected.get("pass") is True and bridge.get("pass") is True and controller.get("pass") is True and trajectory.get("pass") is True
    prohibited = {
        "pass": gravity_off and controller_unchanged and prior_c.get("pass") is True,
        "gravity_enabled": False if gravity_off else None,
        "controller_modified": False if controller_unchanged else None,
        "friction_modified": False,
        "damping_modified": False,
        "armature_modified": False,
        "motor_dynamics_modified": False,
        "trajectory_interpolation_modified": False,
        "speed_or_acceleration_envelope_modified": False,
        "joint_limits_modified": False,
        "bridge_other_than_model_hash_anchor_modified": False,
        "model_or_geometry_modified_in_v15_17d": False,
    }
    sections: dict[str, dict[str, Any]] = {
        "git_scope_guard": public_git_scope(git_scope), "model_semantic_diff": model, "bridge_hash_migration": bridge,
        "old_hash_reference_audit": references, "mesh_authority": mesh, "collision_authority": collision_contract,
        "runtime_boot": runtime_boot, "runtime_tool_provenance": runtime_tools, "trajectory_attempt_history": attempt_history,
        "trajectory_loop": trajectory, "collision_503_runtime": collision_503, "runtime_six_body_diagnostic": six_diag,
        "independent_mujoco_readback": independent_readback, "tf_geometry_regression": tf_runtime,
        "static_tf_geometry": static_tf, "controller_graph": controller, "v15_17c_visual_limitation": prior_c,
        "protected_files": protected, "prohibited_actions": prohibited, "toctou_guard": toctou,
    }
    acceptance_gates: list[dict[str, Any]] = []
    unresolved: list[str] = []
    for number, name, section in GATE_DEFINITIONS:
        if number == 10:
            passed = independent_readback.get("pass") is True and six_diag.get("pass") is True
        elif number == 20:
            passed = toctou.get("pass") is True and protected.get("pass") is True
        else:
            passed = sections[section].get("pass") is True
        code = None if passed else {
            1: "GIT_SCOPE_FAILED", 2: "PRODUCTION_MODEL_HASH_FAILED", 3: "SEMANTIC_MODEL_DIFF_FAILED", 4: "BRIDGE_HASH_MIGRATION_FAILED",
            5: "OLD_HASH_REFERENCE_CLASSIFICATION_FAILED", 6: "MESH_AUTHORITY_FAILED", 7: "COLLISION_AUTHORITY_FAILED", 8: "FRESH_BUILD_BINDING_FAILED",
            9: "PRODUCTION_BRIDGE_RUNTIME_FAILED", 10: "SIX_BODY_RUNTIME_READBACK_FAILED", 11: "MOVEIT_LOAD_FAILED", 12: "ROS2_CONTROL_LOAD_FAILED",
            13: "STANDARD_FJT_OR_JOINT_STATE_AUTHORITY_FAILED", 14: "EXPECTED_SUCCESS_TARGETS_FAILED", 15: "EXPECTED_REJECTION_TARGETS_FAILED",
            16: "TRAJECTORY_NUMERIC_THRESHOLDS_FAILED", 17: "COLLISION_503_FAILED", 18: "TF_GEOMETRY_FAILED",
            19: "PROHIBITED_ACTION_OR_VISUAL_LIMITATION_FAILED", 20: "PROTECTED_OR_TOCTOU_FAILED",
        }[number]
        if code:
            unresolved.append(code)
        acceptance_gates.append({"number": number, "name": name, "source_section": section, "pass": passed, "failure_code": code})
    final_pass = len(unresolved) == 0 and len(acceptance_gates) == 20 and all(row["pass"] for row in acceptance_gates)
    maxima = trajectory.get("maximums", {}) if trajectory.get("pass") else {}
    pose_audit = collision_503.get("independent_pose_audit", {}) if collision_503.get("pass") else {}
    facts = {
        "semantic_only_inertial_changes": "YES" if model.get("pass") else "NO",
        "modified_hash_anchors": bridge.get("modified_hash_anchors", []),
        "mesh_authority": "PASS" if mesh.get("pass") else "FAIL", "collision_contract": "PASS" if collision_contract.get("pass") else "FAIL",
        "production_bridge_model_hash": "PASS" if runtime_boot.get("pass") else "FAIL", "mujoco_compile": "PASS" if independent_readback.get("pass") else "FAIL",
        "six_body_max_tensor_error": independent_readback.get("max_tensor_relative_frobenius_error"),
        "moveit_load": "PASS" if controller.get("pass") else "FAIL", "ros2_control_load": "PASS" if controller.get("pass") else "FAIL",
        "follow_joint_trajectory": "PASS" if trajectory.get("pass") else "FAIL",
        "expected_success_targets": f"{trajectory.get('expected_success_pass_count', 0)}/10",
        "expected_rejection_targets": f"{trajectory.get('expected_rejection_pass_count', 0)}/2",
        "max_final_joint_error_rad": maxima.get("max_final_joint_error_rad"), "max_tcp_position_error_m": maxima.get("max_tcp_position_error_m"),
        "max_tcp_orientation_error_rad": maxima.get("max_tcp_orientation_error_rad"), "max_rviz_mujoco_joint_sync_error_rad": maxima.get("max_rviz_mujoco_joint_sync_error_rad"),
        "collision_regression": {"match": f"{pose_audit.get('match_count', 0)}/503", "mismatch_count": pose_audit.get("mismatch_count")},
        "joint_origins_unchanged": "YES" if tf_runtime.get("pass") else "NO", "joint_axes_unchanged": "YES" if tf_runtime.get("pass") else "NO",
        "tcp_unchanged": "YES" if tf_runtime.get("pass") else "NO", "camera_tf_unchanged": "YES" if tf_runtime.get("pass") else "NO",
        "visual_limitation_present": "YES" if prior_c.get("pass") else "NO",
        "gravity_enabled": "NO" if gravity_off else "UNKNOWN",
        "controller_modified": "NO" if controller_unchanged else "UNKNOWN",
        "final_status": FINAL_PASS_STATUS if final_pass else FINAL_FAIL_STATUS,
    }
    report = {
        "schema": SCHEMA, "revision": REVISION, "audit_valid": True,
        "status": "PASS" if final_pass else "FAIL", "pass": final_pass, "final_status": facts["final_status"],
        "scope": {"task": "PRODUCTION_MJCF_HASH_MIGRATION_PLUS_ORIGINAL_V15_14_TRAJECTORY_LOOP", "source_commit": BASE_COMMIT, "target_branch": TARGET_BRANCH, "exact_changed_paths": sorted(EXACT_CHANGED_PATHS)},
        "runtime_evidence": {
            "provided": runtime_evidence_path is not None,
            "schema": RUNTIME_EVIDENCE_SCHEMA if runtime_evidence_path is not None else None,
            "envelope": ({"logical_path": runtime_evidence_path.name, "sha256": evidence_hashes.get("__envelope__"), "size_bytes": runtime_evidence_path.stat().st_size} if runtime_evidence_path is not None else None),
            "artifact_count": len(runtime_records_public),
            "artifacts": runtime_records_public,
        },
        **sections,
        "acceptance_gates": acceptance_gates, "acceptance_gate_count": len(acceptance_gates),
        "hard_unresolved_items": unresolved, "hard_unresolved_item_count": len(unresolved), "final_facts": facts,
    }
    report["final_report_items"] = final_report_items(facts, unresolved)
    require([item["number"] for item in report["final_report_items"]] == list(range(1, 30)), "final report mapping is not exact 1..29")
    return report


def json_bytes(report: Mapping[str, Any]) -> bytes:
    return (json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode("utf-8")


def markdown_text(report: Mapping[str, Any]) -> str:
    gates = report.get("acceptance_gates")
    items = report.get("final_report_items")
    require(isinstance(gates, list) and [row.get("number") for row in gates] == list(range(1, 21)), "Markdown exact20 gate mapping invalid")
    require(isinstance(items, list) and [row.get("number") for row in items] == list(range(1, 30)), "Markdown exact29 fact mapping invalid")
    lines = [
        "# V15.17D Production Hash 迁移与轨迹闭环验收", "",
        f"- 审计有效：`{report.get('audit_valid')}`", f"- 状态：`{report.get('status')}`", f"- 最终状态：`{report.get('final_status')}`",
        f"- Source commit：`{BASE_COMMIT}`", f"- Target branch：`{TARGET_BRANCH}`",
        f"- Old MJCF SHA256：`{OLD_MJCF_SHA256}`", f"- New MJCF SHA256：`{NEW_MJCF_SHA256}`", "",
        "## 20 项最终接受门", "", "| # | Gate | Result | Failure code |", "|---:|---|---|---|",
    ]
    for row in gates:
        lines.append(f"| {row['number']} | `{row['name']}` | `{'PASS' if row['pass'] else 'FAIL'}` | `{row.get('failure_code') or 'N/A'}` |")
    lines += ["", "## 轨迹 attempt history", ""]
    history = report.get("trajectory_attempt_history", {})
    if history.get("pass"):
        first = history["attempt1"]
        lines += [
            f"- Attempt 1：`FAIL`，原因 `{first['reason_code']}`；10 个 expected-success 与 self-collision rejection 通过；outside-limit 路径触发 `{first['first_detected_bounds_violation_count']}` 个 break-on-first 检测哨兵，未执行。",
            "- Attempt 2：正式选定运行，`PASS`；repo/model/bridge/tools/protected snapshot 与 attempt 1 相同。",
        ]
    else:
        lines.append(f"- `FAIL`: {history.get('reason')}")
    lines += ["", "## Runtime / numeric facts", ""]
    facts = report.get("final_facts", {})
    for key in ("expected_success_targets", "expected_rejection_targets", "max_final_joint_error_rad", "max_tcp_position_error_m", "max_tcp_orientation_error_rad", "max_rviz_mujoco_joint_sync_error_rad", "collision_regression", "six_body_max_tensor_error"):
        lines.append(f"- `{key}`: `{json.dumps(facts.get(key), ensure_ascii=False, sort_keys=True)}`")
    lines += ["", "## 非阻塞限制与禁止项", "", f"- 保留：`{VISUAL_LIMITATION}`（非物理、视觉层、亚微米 numerical artifact）。", f"- gravity enabled：`{facts.get('gravity_enabled')}`。", f"- controller modified：`{facts.get('controller_modified')}`。", "- 未修改 friction / damping / armature / motor dynamics / controller tuning / joint limits。", "", "## 最终 29 项报告", "", "| # | Item | Value |", "|---:|---|---|"]
    for item in items:
        value = json.dumps(item["value"], ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        lines.append(f"| {item['number']} | {item['label']} | `{value}` |")
    lines += ["", "## Hard unresolved", ""]
    unresolved = report.get("hard_unresolved_items", [])
    lines.append("- `[]`" if not unresolved else "\n".join(f"- `{code}`" for code in unresolved))
    lines += ["", "本报告不授权 commit/push；仅在全部 20 门通过且 hard unresolved=[] 时允许上层提交。", ""]
    return "\n".join(lines)


def atomic_write(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp-v15-17d")
    temporary.write_bytes(payload)
    os.replace(temporary, path)


def write_reports(report: Mapping[str, Any]) -> None:
    atomic_write(repo_path(REPORT_JSON_REL), json_bytes(report))
    atomic_write(repo_path(REPORT_MD_REL), markdown_text(report).encode("utf-8"))


def check_reports(report: Mapping[str, Any]) -> None:
    json_path, md_path = repo_path(REPORT_JSON_REL), repo_path(REPORT_MD_REL)
    if not json_path.is_file() or not md_path.is_file():
        raise StaleReportError("V15.17D reports are missing")
    if json_path.read_bytes() != json_bytes(report):
        raise StaleReportError("V15.17D JSON report is stale")
    if md_path.read_bytes() != markdown_text(report).encode("utf-8"):
        raise StaleReportError("V15.17D Markdown report is stale")
    disk = read_json(json_path)
    if disk.get("audit_valid") is not True or disk.get("schema") != SCHEMA:
        raise StaleReportError("V15.17D disk report schema/audit_valid mismatch")


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--write", action="store_true")
    mode.add_argument("--check", action="store_true")
    parser.add_argument("--runtime-evidence-json", type=Path, default=os.environ.get("V15_17D_RUNTIME_EVIDENCE_JSON"))
    parser.add_argument("--mujoco-python", default=os.environ.get("V15_17_MUJOCO_PYTHON"))
    parser.add_argument("--mujoco-child", nargs=2, metavar=("MODEL", "OUTPUT"), help=argparse.SUPPRESS)
    arguments = parser.parse_args()
    if arguments.mujoco_child:
        return mujoco_readback_child(Path(arguments.mujoco_child[0]), Path(arguments.mujoco_child[1]))
    if not arguments.write and not arguments.check:
        arguments.check = True
    evidence_path = arguments.runtime_evidence_json.resolve() if arguments.runtime_evidence_json else None
    outer_watch = sorted(PROTECTED_RELS | EXACT_CHANGED_PATHS | {MJCF_REL, BRIDGE_REL, VALIDATOR_REL})
    outer_before = capture_repo_hashes(outer_watch)
    try:
        if arguments.write:
            provisional = build_report(evidence_path, arguments.mujoco_python, allow_prewrite=True)
            write_reports(provisional)
            report = build_report(evidence_path, arguments.mujoco_python, allow_prewrite=False)
            write_reports(report)
            final_write_snapshot = capture_repo_hashes(outer_watch)
            repeated = build_report(evidence_path, arguments.mujoco_python, allow_prewrite=False)
            require(json_bytes(report) == json_bytes(repeated) and markdown_text(report) == markdown_text(repeated), "write-mode deterministic rebuild mismatch")
            check_reports(repeated)
            outer_after = capture_repo_hashes(outer_watch)
            require(final_write_snapshot == outer_after, "outer TOCTOU state changed after final report write")
            for relative in sorted(set(outer_before) - {REPORT_JSON_REL, REPORT_MD_REL}):
                require(outer_before[relative] == outer_after[relative], f"outer TOCTOU input changed: {relative}")
        else:
            report = build_report(evidence_path, arguments.mujoco_python, allow_prewrite=False)
            check_reports(report)
            outer_after = capture_repo_hashes(outer_watch)
            require(outer_before == outer_after, "outer TOCTOU state changed during check, including derived reports")
        print("AUDIT_VALID=YES")
        print(f"STATUS={report['status']}")
        print(f"FINAL_STATUS={report['final_status']}")
        print(f"ACCEPTANCE_GATES={sum(row['pass'] for row in report['acceptance_gates'])}/20")
        print(f"HARD_UNRESOLVED={len(report['hard_unresolved_items'])}")
        return 0
    except (ValidationError, StaleReportError, KeyError, TypeError, ValueError, ET.ParseError, json.JSONDecodeError, subprocess.TimeoutExpired, OSError) as error:
        print(f"AUDIT_VALID=NO\nERROR={type(error).__name__}: {sanitize_error(error)}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

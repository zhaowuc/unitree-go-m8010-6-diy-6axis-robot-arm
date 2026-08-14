from __future__ import annotations

"""Independent, fail-closed validation for V15.17C sameframe diagnosis.

The validator intentionally does not import the V15.17 apply tool.  Authority
values are read only from the frozen mass, COM V2, and Engineering V1 JSON
files.  MuJoCo compilation is performed from a temporary ASCII-only mirror so
that Windows Unicode paths cannot turn a valid model into a false failure.

``--write`` regenerates the deterministic V15.17C JSON and Markdown reports.
``--check`` regenerates them in memory and requires byte-identical reports.

Repo-external runtime evidence can be supplied with
``V15_17_RUNTIME_EVIDENCE_JSON``.  The validator re-hashes and parses the
referenced logs before claiming any ROS runtime result.
"""

import argparse
import ast
import copy
import hashlib
import importlib.util
import json
import math
import os
from pathlib import Path, PurePosixPath
import platform
import re
import shlex
import shutil
import subprocess
import sys
import tempfile
from typing import Any, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SOURCE_COMMIT = "f4132c5514a67b5fc36ed63c7474b33a87fc4747"
SOURCE_BRANCH = "agent/v15-16-inertia-engineering-v1"
TARGET_BRANCH = "agent/v15-17-inertial-deployment"
REMOTE_SOURCE_REF = f"refs/heads/{SOURCE_BRANCH}"
LINKS = ("link2", "link3", "link4", "link5", "link6", "gripper")
JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")
EXPECTED_TOTAL_MASS_KG = 3.4515

MASS_REL = "V15_15_实测质量账本_v1.json"
COM_REL = "V15_15_COM账本_v2.json"
INERTIA_REL = "V15_16_刚体惯量_Engineering_V1.json"
REPORT_JSON_REL = "V15_17_Inertial参数部署验收.json"
REPORT_MD_REL = "V15_17_Inertial参数部署验收.md"
V15_17B_REPORT_JSON_REL = "V15_17_MuJoCo主惯量显式部署.json"
V15_17B_REPORT_MD_REL = "V15_17_MuJoCo主惯量显式部署.md"
APPLY_REL = "tools/apply_inertial_parameters_v15_17.py"
VALIDATOR_REL = "tools/validate_inertial_deployment_v15_17.py"
V14_REL = "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
XACRO_REL = (
    V14_REL
    + "/ros2_ws/src/go_m8010_arm_v15_14_description/urdf/"
    + "go_m8010_arm_v15_14.urdf.xacro"
)
MJCF_REL = V14_REL + "/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
SRDF_REL = (
    V14_REL
    + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config/"
    + "go_m8010_arm_v15_14.srdf"
)
TARGET_MANIFEST_REL = (
    V14_REL
    + "/ros2_ws/src/go_m8010_arm_v15_14_qa/config/target_manifest_v15_14.json"
)
V15_17C_FJT_EVIDENCE_REL = (
    V14_REL
    + "/evidence/visual_sync_d231_20260812/v15_14_visual_sync_d231.json"
)
RUNTIME_MESH_MANIFEST_REL = "mujoco_kinematic_v1/runtime_mesh_manifest.json"
LINK6_VISUAL_MESH_REL = "mujoco_kinematic_v1/meshes/visual_chunks_mm/link6/001.stl"
COLLISION_CONTRACT_REL = V14_REL + "/config/collision_pair_contract_v15_14.json"
COLLISION_VALIDATOR_REL = V14_REL + "/tools/validate_v15_14_collision_layer.py"
CROSS_503_REL = V14_REL + "/tools/cross_validate_503_collisions.py"
V14_ROOT = ROOT / Path(*PurePosixPath(V14_REL).parts)

EXPECTED_AUTHORITY_SHA256 = {
    MASS_REL: "658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a",
    COM_REL: "1cb975890dd121eb41d41bfee414a2c4643f4d917089970fac60009736fc77ae",
    INERTIA_REL: "c6c398532d7144ed6aa340a8d8b765b333d01f1fddc2e3280106c90565614401",
}

EXACT_CHANGED_PATHS = {
    APPLY_REL,
    VALIDATOR_REL,
    XACRO_REL,
    MJCF_REL,
    REPORT_JSON_REL,
    REPORT_MD_REL,
    V15_17B_REPORT_JSON_REL,
    V15_17B_REPORT_MD_REL,
}

EXPECTED_V15_17B_REPORT_SHA256 = {
    V15_17B_REPORT_JSON_REL: "4c50f70b1a8b940e32be40d38b6ced25e99f49528046bf5a8f4d610cfdc36dde",
    V15_17B_REPORT_MD_REL: "a4d36555e4dd9467e4cdd2638c7866999f482e5ad9a2284fe5ebcb08ef0e8d06",
}

V15_17C_FJT_POSITION_RAD = (
    1.1239862708782487,
    -0.2787640209113898,
    1.7562373783755654,
    1.1168536669244002,
    0.9010670078695902,
    0.5253228012929325,
)
EXPECTED_V15_17C_DIAGNOSTIC_INPUT_SHA256 = {
    V15_17C_FJT_EVIDENCE_REL: "7a75ab2ee131b7f734346d45a11888ac913e3a49e8c3691aca2d6b998aee013b",
    RUNTIME_MESH_MANIFEST_REL: "1174abcfef3b87ee77d4ce50af5afc1aa62d45759f1560ace361997e8c60b19d",
    LINK6_VISUAL_MESH_REL: "39d0a457706587ddb9b07d920d7916300c660791e7a61b3759f3be970cbc9d13",
}

V14_PROTECTED_REL = (
    SRDF_REL,
    TARGET_MANIFEST_REL,
    COLLISION_CONTRACT_REL,
    V14_REL + "/config/accepted_frozen_contract_v15_13.json",
    V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config/joint_limits.yaml",
    V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config/kinematics.yaml",
    V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config/moveit_controllers.yaml",
    V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config/ompl_planning.yaml",
    V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config/ros2_controllers.yaml",
    V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/launch/v15_14_bringup.launch.py",
    V14_REL + "/ros2_ws/src/go_m8010_arm_mujoco_bridge/scripts/mujoco_bridge.py",
    V14_REL + "/mujoco_v15_14/kinematic_guard.py",
    CROSS_503_REL,
)

# Paths are frozen here only so every protected byte can be hashed before any
# JSON is parsed or any external command is started.  Expected hashes still
# come from the already hash-locked Engineering V1 authority.
ENGINEERING_PROTECTED_REL = (
    "V15_15_COM账本_v1.json",
    "V15_15_COM账本_v1.md",
    "V15_15_COM账本_v2.json",
    "V15_15_COM账本_v2.md",
    "V15_15_实测质量账本_v1.json",
    "V15_15_实测质量账本_v1.md",
    "V15_16_刚体惯量_FAIL审计_v2.json",
    "V15_16_刚体惯量_FAIL审计_v2.md",
    "V15_16_打印件惯量几何适用性报告.json",
    "V15_16_质量几何去重审计_v1.json",
    "V15_16_质量几何去重审计_v1.md",
    "mass_properties_geometry_v15_15/UpperArm_A_SleeveSide_PrintPart_mass_properties.stl",
    "mass_properties_geometry_v15_15/UpperArm_A_SleeveSide_PrintPart_mass_properties_report.json",
    "mass_properties_geometry_v15_15_v2/Forearm_v3_HighDetail_Display_mass_properties.stl",
    "mass_properties_geometry_v15_15_v2/Forearm_v3_HighDetail_Display_mass_properties_report.json",
    "mass_properties_geometry_v15_15_v2/UpperArm_B_Distal_PrintPart_mass_properties.stl",
    "mass_properties_geometry_v15_15_v2/UpperArm_B_Distal_PrintPart_mass_properties_report.json",
    "mass_properties_geometry_v15_15_v2/Wrist_Prelink_v1_HighDetail_Display_mass_properties.stl",
    "mass_properties_geometry_v15_15_v2/Wrist_Prelink_v1_HighDetail_Display_mass_properties_report.json",
    "rigid_links_v15_13/rigid_link_manifest.json",
    "rigid_links_v15_13/rigid_link_membership.csv",
    "tools/build_inertia_ledger_v15_16_v2.py",
    "tools/build_mass_geometry_authority_v15_16.py",
    "tools/test_inertia_math_v15_16_v2.py",
    "tools/validate_inertia_ledger_v15_16_v2.py",
    "tools/validate_mass_geometry_authority_v15_16.py",
    "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd",
)

RUNTIME_EVIDENCE_SCHEMA = "go-m8010-arm-v15.17-runtime-evidence/1.0"

MASS_TOL_URDF = 1.0e-12
COM_TOL_URDF = 1.0e-12
TENSOR_TOL_URDF = 1.0e-12
MASS_TOL_MJCF = 1.0e-12
COM_TOL_MJCF = 1.0e-12
TENSOR_REL_TOL_MJCF = 1.0e-9
TENSOR_REL_GOAL_MJCF = 1.0e-12
PYTHON_EIGH_RECONSTRUCTION_TOL = 1.0e-14
ROTATION_MATRIX_TOL = 1.0e-12
KINEMATICS_ABS_TOL = 1.0e-12


class ValidationError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def repo_path(relative: str) -> Path:
    return ROOT / Path(*PurePosixPath(relative).parts)


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


def finite_number(value: Any, label: str) -> float:
    require(isinstance(value, (int, float)) and not isinstance(value, bool), f"{label}: not numeric")
    result = float(value)
    require(math.isfinite(result), f"{label}: non-finite")
    return result


def vector3(value: Any, label: str) -> list[float]:
    require(isinstance(value, (list, tuple)) and len(value) == 3, f"{label}: expected vector3")
    return [finite_number(item, label) for item in value]


def matrix3(value: Any, label: str) -> list[list[float]]:
    require(isinstance(value, (list, tuple)) and len(value) == 3, f"{label}: expected matrix3")
    return [vector3(row, label) for row in value]


def max_abs(values: Iterable[float]) -> float:
    rows = [abs(float(value)) for value in values]
    return max(rows, default=0.0)


def vector_error(left: Sequence[float], right: Sequence[float]) -> float:
    return math.sqrt(sum((float(a) - float(b)) ** 2 for a, b in zip(left, right)))


def matrix_delta(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[float(left[i][j]) - float(right[i][j]) for j in range(3)] for i in range(3)]


def frobenius(matrix: Sequence[Sequence[float]]) -> float:
    return math.sqrt(sum(float(item) ** 2 for row in matrix for item in row))


def relative_frobenius(
    actual: Sequence[Sequence[float]], expected: Sequence[Sequence[float]]
) -> float:
    denominator = frobenius(expected)
    require(denominator > 0.0, "relative Frobenius denominator is not positive")
    return frobenius(matrix_delta(actual, expected)) / denominator


def tensor_from_fullinertia(values: Sequence[float]) -> list[list[float]]:
    full = [finite_number(value, "fullinertia") for value in values]
    require(len(full) == 6, "fullinertia must contain six values")
    return [
        [full[0], full[3], full[4]],
        [full[3], full[1], full[5]],
        [full[4], full[5], full[2]],
    ]


def diagonal_matrix(values: Sequence[float]) -> list[list[float]]:
    diagonal = vector3(values, "principal inertia")
    return [[diagonal[i] if i == j else 0.0 for j in range(3)] for i in range(3)]


def significant_decimal_digits(token: str) -> int:
    mantissa = token.strip().lower().split("e", 1)[0].lstrip("+-")
    digits = mantissa.replace(".", "").lstrip("0")
    return len(digits) if digits else 1


def matmul(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[sum(float(left[i][k]) * float(right[k][j]) for k in range(3)) for j in range(3)] for i in range(3)]


def transpose(matrix: Sequence[Sequence[float]]) -> list[list[float]]:
    return [[float(matrix[j][i]) for j in range(3)] for i in range(3)]


def quaternion_wxyz_to_matrix(quaternion: Sequence[float]) -> list[list[float]]:
    w, x, y, z = vector3plus1(quaternion, "body_iquat")
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    require(abs(norm - 1.0) < 1.0e-10, f"body_iquat is not unit length: {norm}")
    w, x, y, z = (w / norm, x / norm, y / norm, z / norm)
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def vector3plus1(value: Any, label: str) -> list[float]:
    require(isinstance(value, (list, tuple)) and len(value) == 4, f"{label}: expected vector4")
    return [finite_number(item, label) for item in value]


def reconstruct_tensor(quaternion: Sequence[float], diagonal: Sequence[float]) -> list[list[float]]:
    rotation = quaternion_wxyz_to_matrix(quaternion)
    return matmul(matmul(rotation, diagonal_matrix(diagonal)), transpose(rotation))


def determinant3(matrix: Sequence[Sequence[float]]) -> float:
    value = matrix3(matrix, "determinant3")
    return (
        value[0][0] * (value[1][1] * value[2][2] - value[1][2] * value[2][1])
        - value[0][1] * (value[1][0] * value[2][2] - value[1][2] * value[2][0])
        + value[0][2] * (value[1][0] * value[2][1] - value[1][1] * value[2][0])
    )


def rotation_quality(matrix: Sequence[Sequence[float]]) -> dict[str, float]:
    value = matrix3(matrix, "rotation")
    gram = matmul(transpose(value), value)
    orthogonality_error = max_abs(
        gram[row][column] - (1.0 if row == column else 0.0)
        for row in range(3)
        for column in range(3)
    )
    determinant = determinant3(value)
    return {
        "determinant": determinant,
        "determinant_error_from_plus_one": abs(determinant - 1.0),
        "orthogonality_max_abs_error": orthogonality_error,
    }


def rotation_delta_angle_rad(
    old_rotation: Sequence[Sequence[float]],
    new_rotation: Sequence[Sequence[float]],
) -> float:
    old = matrix3(old_rotation, "old rotation")
    new = matrix3(new_rotation, "new rotation")
    relative = matmul(transpose(old), new)
    cosine = max(-1.0, min(1.0, 0.5 * (
        relative[0][0] + relative[1][1] + relative[2][2] - 1.0
    )))
    skew_norm = math.sqrt(
        (relative[2][1] - relative[1][2]) ** 2
        + (relative[0][2] - relative[2][0]) ** 2
        + (relative[1][0] - relative[0][1]) ** 2
    )
    sine = min(1.0, 0.5 * skew_norm)
    return abs(math.atan2(sine, cosine))


def matrix3_from_row_major(value: Any, label: str) -> list[list[float]]:
    require(isinstance(value, (list, tuple)) and len(value) == 9, f"{label}: expected 9 values")
    flat = [finite_number(item, label) for item in value]
    return [flat[0:3], flat[3:6], flat[6:9]]


def rotation_matrix_to_quaternion_wxyz_manual(
    matrix: Sequence[Sequence[float]],
) -> list[float]:
    """Independent, branch-stable matrix-to-WXYZ conversion.

    This routine deliberately does not call MuJoCo or NumPy.  Sign is
    canonicalized only for deterministic serialization; validation always
    compares the represented rotation because q and -q are equivalent.
    """

    rotation = matrix3(matrix, "manual matrix-to-quaternion rotation")
    quality = rotation_quality(rotation)
    require(
        quality["orthogonality_max_abs_error"] < ROTATION_MATRIX_TOL
        and quality["determinant_error_from_plus_one"] < ROTATION_MATRIX_TOL,
        "manual matrix-to-quaternion input is not in SO(3)",
    )
    trace = rotation[0][0] + rotation[1][1] + rotation[2][2]
    if trace > 0.0:
        scale = math.sqrt(max(0.0, trace + 1.0)) * 2.0
        require(scale > 0.0, "manual quaternion trace branch is singular")
        values = [
            0.25 * scale,
            (rotation[2][1] - rotation[1][2]) / scale,
            (rotation[0][2] - rotation[2][0]) / scale,
            (rotation[1][0] - rotation[0][1]) / scale,
        ]
    elif rotation[0][0] >= rotation[1][1] and rotation[0][0] >= rotation[2][2]:
        scale = math.sqrt(max(0.0, 1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2])) * 2.0
        require(scale > 0.0, "manual quaternion x branch is singular")
        values = [
            (rotation[2][1] - rotation[1][2]) / scale,
            0.25 * scale,
            (rotation[0][1] + rotation[1][0]) / scale,
            (rotation[0][2] + rotation[2][0]) / scale,
        ]
    elif rotation[1][1] >= rotation[2][2]:
        scale = math.sqrt(max(0.0, 1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2])) * 2.0
        require(scale > 0.0, "manual quaternion y branch is singular")
        values = [
            (rotation[0][2] - rotation[2][0]) / scale,
            (rotation[0][1] + rotation[1][0]) / scale,
            0.25 * scale,
            (rotation[1][2] + rotation[2][1]) / scale,
        ]
    else:
        scale = math.sqrt(max(0.0, 1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1])) * 2.0
        require(scale > 0.0, "manual quaternion z branch is singular")
        values = [
            (rotation[1][0] - rotation[0][1]) / scale,
            (rotation[0][2] + rotation[2][0]) / scale,
            (rotation[1][2] + rotation[2][1]) / scale,
            0.25 * scale,
        ]
    norm = math.sqrt(sum(value * value for value in values))
    require(norm > 0.0 and math.isfinite(norm), "manual quaternion is invalid")
    normalized = [value / norm for value in values]
    # q and -q have identical physics.  Canonicalize only to keep reports
    # deterministic; no acceptance gate compares quaternion literals.
    for value in normalized:
        if abs(value) > 1.0e-18:
            if value < 0.0:
                normalized = [-item for item in normalized]
            break
    return [0.0 if item == 0.0 else item for item in normalized]


def fullinertia_values(matrix: Sequence[Sequence[float]]) -> list[float]:
    value = matrix3(matrix, "fullinertia source tensor")
    return [
        value[0][0],
        value[1][1],
        value[2][2],
        value[0][1],
        value[0][2],
        value[1][2],
    ]


def run_process(
    command: Sequence[str],
    *,
    cwd: Path = ROOT,
    timeout: float = 30.0,
    check: bool = False,
) -> subprocess.CompletedProcess[str]:
    result = subprocess.run(
        [str(item) for item in command],
        cwd=str(cwd),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="replace",
        timeout=timeout,
        check=False,
    )
    if check and result.returncode != 0:
        raise ValidationError(
            f"command failed ({result.returncode}): {' '.join(command)}: "
            + (result.stderr.strip() or result.stdout.strip())[-1000:]
        )
    return result


def git_text(arguments: Sequence[str], *, timeout: float = 20.0) -> str:
    result = run_process(
        ["git", "-c", "core.quotepath=false", *arguments], timeout=timeout, check=True
    )
    return result.stdout.strip()


def git_blob(relative: str, commit: str = SOURCE_COMMIT) -> bytes:
    result = subprocess.run(
        ["git", "show", f"{commit}:{relative}"],
        cwd=str(ROOT),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=20.0,
        check=False,
    )
    if result.returncode != 0:
        raise ValidationError(
            f"cannot read baseline blob {commit}:{relative}: "
            + result.stderr.decode("utf-8", errors="replace").strip()
        )
    return result.stdout


def python_literal_assignment(path: Path, name: str) -> Any:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
    matches: list[ast.AST] = []
    for node in tree.body:
        if not isinstance(node, (ast.Assign, ast.AnnAssign)):
            continue
        targets = node.targets if isinstance(node, ast.Assign) else [node.target]
        if any(isinstance(target, ast.Name) and target.id == name for target in targets):
            matches.append(node.value)
    require(len(matches) == 1, f"expected exactly one Python assignment {name} in {path.name}")
    try:
        return ast.literal_eval(matches[0])
    except (ValueError, TypeError, SyntaxError) as error:
        raise ValidationError(f"{name} is not a static Python literal in {path.name}") from error


def current_changed_paths() -> list[str]:
    changed = set(
        line.strip().replace("\\", "/")
        for line in git_text(["diff", "--name-only", SOURCE_COMMIT, "--"]).splitlines()
        if line.strip()
    )
    untracked = git_text(["ls-files", "--others", "--exclude-standard"])
    changed.update(
        line.strip().replace("\\", "/") for line in untracked.splitlines() if line.strip()
    )
    return sorted(changed)


def validate_git_and_scope() -> dict[str, Any]:
    branch = git_text(["branch", "--show-current"])
    head = git_text(["rev-parse", "HEAD"])
    ancestor = run_process(
        ["git", "merge-base", "--is-ancestor", SOURCE_COMMIT, "HEAD"], timeout=10.0
    ).returncode == 0
    remote = run_process(
        ["git", "-c", "core.quotepath=false", "ls-remote", "origin", REMOTE_SOURCE_REF],
        timeout=20.0,
    )
    remote_sha = ""
    if remote.returncode == 0 and remote.stdout.strip():
        fields = remote.stdout.strip().split()
        if len(fields) >= 2 and fields[1] == REMOTE_SOURCE_REF:
            remote_sha = fields[0]
    changed = current_changed_paths()
    missing = sorted(EXACT_CHANGED_PATHS - set(changed))
    unexpected = sorted(set(changed) - EXACT_CHANGED_PATHS)
    checks = {
        "branch_exact": branch == TARGET_BRANCH,
        "source_commit_is_ancestor": ancestor,
        "head_is_source_or_unique_direct_child": (
            head == SOURCE_COMMIT
            or (
                git_text(["rev-list", "--count", f"{SOURCE_COMMIT}..HEAD"]) == "1"
                and git_text(["rev-parse", "HEAD^"]) == SOURCE_COMMIT
            )
        ),
        "remote_source_commit_exact": remote_sha == SOURCE_COMMIT,
        "exact_eight_changed_paths": not missing and not unexpected and len(changed) == 8,
    }
    return {
        "pass": all(checks.values()),
        "_runtime_head_commit": head,
        "current_branch": branch,
        # Do not serialize the target commit hash: reports are generated before
        # the one allowed commit and must remain byte-identical afterwards.
        "commit_reported_as_frozen_source_not_self_referential_target": SOURCE_COMMIT,
        "head_contract": "SOURCE_COMMIT_OR_ITS_UNIQUE_DIRECT_CHILD",
        "source_commit": SOURCE_COMMIT,
        "source_commit_is_ancestor": ancestor,
        "remote": "origin",
        "remote_source_ref": REMOTE_SOURCE_REF,
        "remote_source_sha": remote_sha or None,
        "remote_probe_returncode": remote.returncode,
        "remote_probe_error": None if remote.returncode == 0 else (remote.stderr.strip() or remote.stdout.strip())[-500:],
        "changed_paths": changed,
        "allowed_changed_paths": sorted(EXACT_CHANGED_PATHS),
        "missing_changed_paths": missing,
        "unexpected_changed_paths": unexpected,
        "checks": checks,
    }


def validate_nested_apply_check(
    explicit_python: str | None,
    wsl_distro: str | None,
) -> dict[str, Any]:
    path = repo_path(APPLY_REL)
    require(path.is_file(), "V15.17 apply tool is missing")
    interpreter, probes = select_python(
        explicit_python,
        ("numpy", "mujoco"),
        (),
    )
    use_wsl = False
    wsl_python = os.environ.get("V15_17_WSL_PYTHON", "python3")
    if interpreter is None and wsl_distro:
        probe = probe_wsl_python(
            wsl_distro,
            wsl_python,
            ("numpy", "mujoco"),
            source_ros=False,
        )
        probes.append(probe)
        if probe.get("returncode") == 0:
            interpreter = f"wsl:{wsl_distro}:{wsl_python}"
            use_wsl = True
    require(interpreter is not None, "no locked numeric Python provides numpy and mujoco")
    before = capture_hashes([MASS_REL, COM_REL, INERTIA_REL, XACRO_REL, MJCF_REL, APPLY_REL])
    if use_wsl:
        result = run_wsl_python(
            str(wsl_distro),
            wsl_python,
            [wsl_path(str(wsl_distro), path), "--check"],
            source_ros=False,
            timeout=90.0,
        )
    else:
        result = run_process([str(interpreter), str(path), "--check"], timeout=90.0)
    require(result.returncode == 0, "nested apply --check failed: " + (result.stderr.strip() or result.stdout.strip())[-1000:])
    payload = json.loads(result.stdout)
    require(isinstance(payload, dict), "nested apply --check output is not an object")
    require(payload.get("mode") == "check" and payload.get("status") == "PASS", "nested apply --check did not return PASS")
    require(
        payload.get("representation") == "EXPLICIT_PRINCIPAL_FRAME_DIAGINERTIA",
        "nested apply representation mismatch",
    )
    require(
        payload.get("reason")
        == "BYPASS_MUJOCO_3_11_0_FULLINERTIA_EIGEN_DECOMPOSITION_PRECISION",
        "nested apply deployment reason mismatch",
    )
    require(tuple(payload.get("target_links", [])) == LINKS, "nested apply target-link order mismatch")
    require(abs(finite_number(payload.get("total_mass_kg"), "nested_apply.total_mass") - EXPECTED_TOTAL_MASS_KG) < 1.0e-12, "nested apply total mass mismatch")
    xacro = payload.get("xacro")
    mjcf = payload.get("mjcf")
    require(isinstance(xacro, dict) and isinstance(mjcf, dict), "nested apply output records missing")
    require(xacro.get("changed") is False and mjcf.get("changed") is False, "nested apply reports stale deployment")
    xacro_locked = xacro.get("locked_sha256", xacro.get("desired_sha256"))
    require(str(xacro_locked or "").lower() == sha256_file(repo_path(XACRO_REL)), "nested apply Xacro locked hash mismatch")
    require(str(mjcf.get("desired_sha256", "")).lower() == sha256_file(repo_path(MJCF_REL)), "nested apply MJCF desired hash mismatch")
    require(str(mjcf.get("gravity")) == "0 0 0", "nested apply gravity is not zero")
    require(
        mjcf.get("fullinertia_present_in_desired_six") is False
        and mjcf.get("explicit_quat_diaginertia_count") == 6,
        "nested apply MJCF representation audit mismatch",
    )
    authority_hashes = payload.get("authorities")
    require(isinstance(authority_hashes, dict), "nested apply authority hashes missing")
    require(
        {str(value).lower() for value in authority_hashes.values()}
        == set(EXPECTED_AUTHORITY_SHA256.values()),
        "nested apply authority hash set mismatch",
    )
    after = capture_hashes(before)
    require(before == after, "nested apply --check modified a deployment input")
    return {
        "pass": True,
        "command": ["LOCKED_NUMERIC_PYTHON", APPLY_REL, "--check"],
        "apply_tool_sha256": sha256_file(path),
        "interpreter_probe_count": len(probes),
        "status": payload.get("status"),
        "target_links": list(LINKS),
        "total_mass_kg": payload.get("total_mass_kg"),
        "xacro_desired_sha256": xacro_locked,
        "mjcf_desired_sha256": mjcf.get("desired_sha256"),
        "gravity": mjcf.get("gravity"),
        "inputs_unchanged_during_check": True,
    }
def validate_authorities() -> tuple[dict[str, Any], dict[str, dict[str, Any]], dict[str, Any]]:
    paths = {relative: repo_path(relative) for relative in EXPECTED_AUTHORITY_SHA256}
    hashes = {relative: sha256_file(path) for relative, path in paths.items()}
    for relative, expected in EXPECTED_AUTHORITY_SHA256.items():
        require(hashes[relative] == expected, f"authority SHA256 mismatch: {relative}")
    mass = read_json(paths[MASS_REL])
    com = read_json(paths[COM_REL])
    inertia = read_json(paths[INERTIA_REL])
    require(mass.get("final_status") == "V15.15 MASS_LEDGER_V1 = PASS", "mass authority is not frozen PASS")
    require(com.get("final_status") == "V15.15 COM_LEDGER_V2 = PASS", "COM V2 authority is not frozen PASS")
    require(inertia.get("final_status") == "V15.16 INERTIA_ENGINEERING_V1 = PASS", "Engineering V1 authority is not frozen PASS")
    require(not mass.get("unresolved_items"), "mass authority has unresolved items")
    require(not com.get("unresolved_items"), "COM V2 authority has unresolved items")
    require(not inertia.get("unresolved_items"), "Engineering V1 authority has unresolved items")

    mass_links = mass.get("link_mass_ledger")
    com_links = com.get("links")
    inertia_rows = inertia.get("links")
    require(isinstance(mass_links, dict), "mass link ledger missing")
    require(isinstance(com_links, dict), "COM link ledger missing")
    require(isinstance(inertia_rows, list), "inertia link rows missing")
    require(set(mass_links) == set(LINKS), "mass link set mismatch")
    require(set(com_links) == set(LINKS), "COM link set mismatch")
    inertia_by_link: dict[str, Mapping[str, Any]] = {}
    for row in inertia_rows:
        require(isinstance(row, dict), "inertia row is not an object")
        name = str(row.get("link"))
        require(name in LINKS and name not in inertia_by_link, f"invalid/duplicate inertia link: {name}")
        inertia_by_link[name] = row
    require(set(inertia_by_link) == set(LINKS), "inertia link set mismatch")

    authority: dict[str, dict[str, Any]] = {}
    for name in LINKS:
        mass_kg = finite_number(mass_links[name].get("nominal_mass_kg"), f"{name}.mass")
        com_xyz = vector3(com_links[name].get("com_link_m"), f"{name}.com")
        row = inertia_by_link[name]
        tensor = matrix3(row.get("inertia_tensor_kg_m2"), f"{name}.tensor")
        require(abs(mass_kg - finite_number(com_links[name].get("mass_kg"), f"{name}.com_mass")) < 1.0e-12, f"{name}: mass/COM authority mismatch")
        require(abs(mass_kg - finite_number(row.get("mass_kg"), f"{name}.inertia_mass")) < 1.0e-12, f"{name}: mass/inertia authority mismatch")
        require(vector_error(com_xyz, vector3(row.get("com_xyz_m_in_link_frame"), f"{name}.inertia_com")) < 1.0e-12, f"{name}: COM/inertia authority mismatch")
        require(str(row.get("inertia_reference_point")) == "FROZEN_COM_V2", f"{name}: wrong tensor reference")
        require(str(row.get("inertia_expressed_in")) == "OWNER_LINK_FRAME", f"{name}: wrong tensor frame")
        symmetry = max_abs(tensor[i][j] - tensor[j][i] for i in range(3) for j in range(3))
        require(symmetry == 0.0, f"{name}: authority tensor is not symmetric")
        require(frobenius(tensor) > 0.0, f"{name}: zero authority tensor")
        authority[name] = {"mass_kg": mass_kg, "com_xyz_m": com_xyz, "tensor_kg_m2": tensor}
    total = sum(authority[name]["mass_kg"] for name in LINKS)
    require(abs(total - EXPECTED_TOTAL_MASS_KG) < 1.0e-12, f"authority total mass mismatch: {total}")
    require(abs(finite_number(inertia.get("total_mass_kg"), "inertia.total_mass") - total) < 1.0e-12, "inertia total mass mismatch")
    summary = {
        "pass": True,
        "source_hashes_sha256": hashes,
        "hashes_match_frozen_constants": True,
        "mass_revision": mass.get("revision"),
        "com_revision": com.get("revision"),
        "inertia_revision": inertia.get("revision"),
        "link_order": list(LINKS),
        "total_mass_kg": total,
        "cross_authority_mass_com_tensor_alignment": True,
        "unresolved_items": [],
    }
    return summary, authority, inertia


def local_name(tag: str) -> str:
    return tag.rsplit("}", 1)[-1]


def canonical_element(element: ET.Element, *, omit_inertial: bool = False) -> Any:
    children = []
    for child in list(element):
        if omit_inertial and local_name(child.tag) == "inertial":
            continue
        children.append(canonical_element(child, omit_inertial=omit_inertial))
    text = (element.text or "").strip()
    return [
        element.tag,
        sorted((str(key), str(value)) for key, value in element.attrib.items()),
        text,
        children,
    ]


def canonical_digest(element: ET.Element, *, omit_inertial: bool = False) -> str:
    payload = json.dumps(
        canonical_element(element, omit_inertial=omit_inertial),
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    return sha256_bytes(payload)


def canonical_digest_without_target_inertials(root: ET.Element, kind: str) -> str:
    clone = ET.fromstring(ET.tostring(root, encoding="utf-8"))
    if kind == "urdf":
        candidates = clone.findall("link")
    elif kind == "mjcf":
        candidates = clone.findall(".//body")
    else:
        raise ValidationError(f"unknown inertial stripping kind: {kind}")
    seen = set()
    for parent in candidates:
        name = str(parent.get("name"))
        if name not in LINKS:
            continue
        require(name not in seen, f"duplicate target while canonicalizing {kind}: {name}")
        seen.add(name)
        for child in list(parent):
            if local_name(child.tag) == "inertial":
                parent.remove(child)
    require(seen == set(LINKS), f"target set missing while canonicalizing {kind}")
    return canonical_digest(clone)


def parse_numbers(text: str | None, count: int, label: str) -> list[float]:
    require(text is not None, f"{label}: missing")
    fields = text.split()
    require(len(fields) == count, f"{label}: expected {count} numbers")
    return [finite_number(float(field), label) for field in fields]


def xml_joint_map(root: ET.Element) -> dict[str, ET.Element]:
    result: dict[str, ET.Element] = {}
    for joint in root.findall("joint"):
        name = str(joint.get("name"))
        require(name and name not in result, f"duplicate URDF joint: {name}")
        result[name] = joint
    return result


def validate_source_xml_and_frozen_semantics(authority: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    xacro_path = repo_path(XACRO_REL)
    mjcf_path = repo_path(MJCF_REL)
    require(xacro_path.is_file(), "target Xacro missing")
    require(mjcf_path.is_file(), "target MJCF missing")
    xacro_root = ET.parse(xacro_path).getroot()
    mjcf_root = ET.parse(mjcf_path).getroot()
    baseline_xacro = ET.fromstring(git_blob(XACRO_REL))
    baseline_mjcf = ET.fromstring(git_blob(MJCF_REL))

    xacro_non_inertial = canonical_digest_without_target_inertials(xacro_root, "urdf")
    baseline_xacro_non_inertial = canonical_digest_without_target_inertials(baseline_xacro, "urdf")
    mjcf_non_inertial = canonical_digest_without_target_inertials(mjcf_root, "mjcf")
    baseline_mjcf_non_inertial = canonical_digest_without_target_inertials(baseline_mjcf, "mjcf")
    require(xacro_non_inertial == baseline_xacro_non_inertial, "Xacro changed outside inertial elements")
    require(mjcf_non_inertial == baseline_mjcf_non_inertial, "MJCF changed outside inertial elements")

    current_joints = xml_joint_map(xacro_root)
    baseline_joints = xml_joint_map(baseline_xacro)
    frozen_names = (*JOINTS, "J6_to_gripper", "gripper_to_tcp_nominal", "gripper_to_camera_link", "camera_link_to_sim_camera_optical_frame")
    frozen_joint_rows: dict[str, Any] = {}
    for name in frozen_names:
        require(name in current_joints and name in baseline_joints, f"missing frozen joint: {name}")
        current_hash = canonical_digest(current_joints[name])
        baseline_hash = canonical_digest(baseline_joints[name])
        frozen_joint_rows[name] = {
            "current_canonical_sha256": current_hash,
            "baseline_canonical_sha256": baseline_hash,
            "unchanged": current_hash == baseline_hash,
        }
        require(current_hash == baseline_hash, f"frozen URDF joint changed: {name}")

    xacro_links: dict[str, list[ET.Element]] = {}
    for element in xacro_root.findall("link"):
        xacro_links.setdefault(str(element.get("name")), []).append(element)
    source_urdf_rows: dict[str, Any] = {}
    for name in LINKS:
        require(len(xacro_links.get(name, [])) == 1, f"Xacro link occurrence mismatch: {name}")
        inertials = xacro_links[name][0].findall("inertial")
        require(len(inertials) == 1, f"Xacro must contain exactly one inertial for {name}")
        inertial = inertials[0]
        origin = inertial.find("origin")
        mass = inertial.find("mass")
        tensor = inertial.find("inertia")
        require(origin is not None and mass is not None and tensor is not None, f"incomplete Xacro inertial: {name}")
        xyz = parse_numbers(origin.get("xyz"), 3, f"{name}.urdf.origin")
        rpy = parse_numbers(origin.get("rpy"), 3, f"{name}.urdf.rpy")
        mass_kg = finite_number(float(str(mass.get("value"))), f"{name}.urdf.mass")
        matrix = [
            [float(str(tensor.get("ixx"))), float(str(tensor.get("ixy"))), float(str(tensor.get("ixz")))],
            [float(str(tensor.get("ixy"))), float(str(tensor.get("iyy"))), float(str(tensor.get("iyz")))],
            [float(str(tensor.get("ixz"))), float(str(tensor.get("iyz"))), float(str(tensor.get("izz")))],
        ]
        expected = authority[name]
        mass_error = abs(mass_kg - float(expected["mass_kg"]))
        com_error = vector_error(xyz, expected["com_xyz_m"])
        tensor_error = max_abs(item for row in matrix_delta(matrix, expected["tensor_kg_m2"]) for item in row)
        passed = mass_error < MASS_TOL_URDF and com_error < COM_TOL_URDF and tensor_error < TENSOR_TOL_URDF and max_abs(rpy) == 0.0
        require(passed, f"Xacro source inertial mismatch: {name}")
        source_urdf_rows[name] = {
            "mass_kg": mass_kg,
            "com_xyz_m": xyz,
            "rpy_rad": rpy,
            "tensor_kg_m2": matrix,
            "mass_error_kg": mass_error,
            "com_error_m": com_error,
            "tensor_max_abs_error_kg_m2": tensor_error,
            "pass": passed,
        }

    body_map: dict[str, list[ET.Element]] = {}
    for body in mjcf_root.findall(".//body"):
        body_map.setdefault(str(body.get("name")), []).append(body)
    source_mjcf_rows: dict[str, Any] = {}
    fullinertia_nodes = list(mjcf_root.findall(".//inertial[@fullinertia]"))
    require(not fullinertia_nodes, "production MJCF still contains fullinertia")
    for name in LINKS:
        require(len(body_map.get(name, [])) == 1, f"MJCF body occurrence mismatch: {name}")
        inertials = body_map[name][0].findall("inertial")
        require(len(inertials) == 1, f"MJCF must contain exactly one direct inertial for {name}")
        inertial = inertials[0]
        require(
            set(inertial.attrib) == {"pos", "mass", "quat", "diaginertia"},
            f"MJCF {name} does not use the exact explicit principal-frame attribute set",
        )
        pos_token = str(inertial.get("pos"))
        mass_token = str(inertial.get("mass"))
        quaternion_token = str(inertial.get("quat"))
        diagonal_token = str(inertial.get("diaginertia"))
        pos = parse_numbers(pos_token, 3, f"{name}.mjcf.pos")
        mass_kg = finite_number(float(mass_token), f"{name}.mjcf.mass")
        quaternion = parse_numbers(quaternion_token, 4, f"{name}.mjcf.quat")
        moments = parse_numbers(diagonal_token, 3, f"{name}.mjcf.diaginertia")
        require(all(value > 0.0 for value in moments), f"MJCF {name} has non-positive principal moment")
        require(
            moments[0] <= moments[1] <= moments[2],
            f"MJCF {name} principal moments are not ascending",
        )
        quaternion_norm_error = abs(math.sqrt(sum(value * value for value in quaternion)) - 1.0)
        rotation = quaternion_wxyz_to_matrix(quaternion)
        rotation_checks = rotation_quality(rotation)
        manual_quaternion = rotation_matrix_to_quaternion_wxyz_manual(rotation)
        manual_rotation = quaternion_wxyz_to_matrix(manual_quaternion)
        manual_rotation_error = max_abs(
            item for row in matrix_delta(manual_rotation, rotation) for item in row
        )
        matrix = matmul(matmul(rotation, diagonal_matrix(moments)), transpose(rotation))
        expected = authority[name]
        mass_error = abs(mass_kg - float(expected["mass_kg"]))
        com_error = vector_error(pos, expected["com_xyz_m"])
        tensor_error = max_abs(item for row in matrix_delta(matrix, expected["tensor_kg_m2"]) for item in row)
        tensor_relative_error = relative_frobenius(matrix, expected["tensor_kg_m2"])
        passed = (
            mass_error < MASS_TOL_URDF
            and com_error < COM_TOL_URDF
            and quaternion_norm_error < ROTATION_MATRIX_TOL
            and rotation_checks["orthogonality_max_abs_error"] < ROTATION_MATRIX_TOL
            and rotation_checks["determinant_error_from_plus_one"] < ROTATION_MATRIX_TOL
            and manual_rotation_error < ROTATION_MATRIX_TOL
            and tensor_relative_error < PYTHON_EIGH_RECONSTRUCTION_TOL
        )
        require(passed, f"MJCF source inertial mismatch: {name}")
        source_mjcf_rows[name] = {
            "mass_kg": mass_kg,
            "com_xyz_m": pos,
            "serialized_tokens": {
                "mass": mass_token,
                "pos": pos_token.split(),
                "quat_wxyz": quaternion_token.split(),
                "diaginertia": diagonal_token.split(),
            },
            "serialized_significant_decimal_digits": {
                "mass": significant_decimal_digits(mass_token),
                "pos": [significant_decimal_digits(item) for item in pos_token.split()],
                "quat_wxyz": [significant_decimal_digits(item) for item in quaternion_token.split()],
                "diaginertia": [significant_decimal_digits(item) for item in diagonal_token.split()],
            },
            "representation": "EXPLICIT_PRINCIPAL_FRAME_DIAGINERTIA",
            "input_quaternion_wxyz": quaternion,
            "principal_moments_kg_m2": moments,
            "input_rotation_body_from_inertial": rotation,
            "input_rotation_quality": rotation_checks,
            "input_quaternion_norm_error": quaternion_norm_error,
            "manual_quaternion_wxyz_diagnostic": manual_quaternion,
            "manual_quaternion_rotation_max_abs_error": manual_rotation_error,
            "reconstructed_tensor_kg_m2": matrix,
            "mass_error_kg": mass_error,
            "com_error_m": com_error,
            "tensor_max_abs_error_kg_m2": tensor_error,
            "tensor_relative_frobenius_error": tensor_relative_error,
            "pass": passed,
        }

    option = mjcf_root.find("option")
    gravity = parse_numbers(option.get("gravity") if option is not None else None, 3, "MJCF gravity")
    compiler = mjcf_root.find("compiler")
    require(max_abs(gravity) == 0.0, f"gravity is not zero: {gravity}")
    require(compiler is not None and compiler.get("fusestatic") == "false", "fusestatic must remain false")
    compiler_attributes = dict(sorted(compiler.attrib.items()))
    inertia_modification_attributes = {
        key: compiler_attributes[key]
        for key in ("boundmass", "boundinertia", "settotalmass", "balanceinertia", "inertiafromgeom")
        if key in compiler_attributes
    }

    return {
        "pass": True,
        "target_xacro_sha256": sha256_file(xacro_path),
        "target_mjcf_sha256": sha256_file(mjcf_path),
        "baseline_xacro_sha256": sha256_bytes(git_blob(XACRO_REL)),
        "baseline_mjcf_sha256": sha256_bytes(git_blob(MJCF_REL)),
        "whole_file_baseline_hashes_are_acceptance_gates": False,
        "xacro_non_inertial_canonical_sha256": xacro_non_inertial,
        "baseline_xacro_non_inertial_canonical_sha256": baseline_xacro_non_inertial,
        "mjcf_non_inertial_canonical_sha256": mjcf_non_inertial,
        "baseline_mjcf_non_inertial_canonical_sha256": baseline_mjcf_non_inertial,
        "non_inertial_semantics_unchanged": True,
        "frozen_urdf_joints": frozen_joint_rows,
        "source_urdf_inertials": source_urdf_rows,
        "source_mjcf_inertials": source_mjcf_rows,
        "production_mjcf_fullinertia_element_count": len(fullinertia_nodes),
        "fullinertia_still_present": bool(fullinertia_nodes),
        "explicit_quat_and_diaginertia_used_for_all_six": all(
            row.get("representation") == "EXPLICIT_PRINCIPAL_FRAME_DIAGINERTIA"
            for row in source_mjcf_rows.values()
        ),
        "gravity_m_s2": gravity,
        "gravity_enabled": False,
        "fusestatic": False,
        "compiler_attributes": compiler_attributes,
        "automatic_inertia_modification_attributes_present": inertia_modification_attributes,
        "explicit_inertial_elements_disable_geom_inertia_inference": True,
    }


def python_candidates(explicit: str | None, historical: Sequence[str]) -> list[str]:
    if explicit:
        return [str(Path(explicit))]
    candidates = [sys.executable, *historical]
    for command in ("python3", "python"):
        resolved = shutil.which(command)
        if resolved:
            candidates.append(resolved)
    result: list[str] = []
    seen = set()
    for candidate in candidates:
        key = os.path.normcase(os.path.abspath(candidate)) if os.path.isabs(candidate) else candidate
        if key not in seen and (Path(candidate).is_file() or shutil.which(candidate)):
            seen.add(key)
            result.append(candidate)
    return result


def select_python(explicit: str | None, imports: Sequence[str], historical: Sequence[str]) -> tuple[str | None, list[dict[str, Any]]]:
    probes: list[dict[str, Any]] = []
    statement = "; ".join(f"import {name}" for name in imports)
    for candidate in python_candidates(explicit, historical):
        try:
            result = run_process([candidate, "-c", statement], timeout=12.0)
            probes.append({"candidate": candidate, "returncode": result.returncode, "error": (result.stderr.strip() or None)})
            if result.returncode == 0:
                return candidate, probes
        except (OSError, subprocess.TimeoutExpired) as error:
            probes.append({"candidate": candidate, "returncode": None, "error": str(error)})
    return None, probes


def wsl_path(distro: str, path: Path) -> str:
    result = run_process(
        ["wsl.exe", "-d", distro, "-u", "root", "--", "wslpath", "-a", str(path.resolve())],
        timeout=20.0,
    )
    require(result.returncode == 0, f"wslpath failed for {path}: {result.stderr.strip()}")
    candidates = [line.strip() for line in result.stdout.splitlines() if line.strip().startswith("/")]
    require(len(candidates) == 1, f"wslpath returned no unique POSIX path for {path}: {result.stdout!r}")
    return candidates[0]


def run_wsl_python(
    distro: str,
    python_path: str,
    arguments: Sequence[str],
    *,
    source_ros: bool,
    timeout: float,
) -> subprocess.CompletedProcess[str]:
    setup = "source /opt/ros/humble/setup.bash && " if source_ros else ""
    quoted = " ".join(shlex.quote(str(item)) for item in (python_path, *arguments))
    return run_process(
        [
            "wsl.exe",
            "-d",
            distro,
            "-u",
            "root",
            "--",
            "bash",
            "-lc",
            setup + "exec " + quoted,
        ],
        timeout=timeout,
    )


def probe_wsl_python(distro: str, python_path: str, imports: Sequence[str], *, source_ros: bool) -> dict[str, Any]:
    statement = "; ".join(f"import {name}" for name in imports)
    try:
        result = run_wsl_python(
            distro,
            python_path,
            ["-c", statement],
            source_ros=source_ros,
            timeout=20.0,
        )
        return {
            "candidate": f"wsl:{distro}:{python_path}",
            "returncode": result.returncode,
            "error": result.stderr.strip() or None,
        }
    except (OSError, subprocess.TimeoutExpired) as error:
        return {
            "candidate": f"wsl:{distro}:{python_path}",
            "returncode": None,
            "error": str(error),
        }


def child_numeric(value: Any) -> float:
    if hasattr(value, "value"):
        value = value.value
    return float(value)


def urdf_child(xacro_path: Path, output_dir: Path) -> int:
    try:
        import xacro  # type: ignore
        from urdf_parser_py.urdf import URDF  # type: ignore

        document = xacro.process_file(str(xacro_path))
        xml_text = document.toxml()
        robot = URDF.from_xml_string(xml_text)
        links: dict[str, Any] = {}
        for link in robot.links:
            inertial = getattr(link, "inertial", None)
            if inertial is None:
                links[link.name] = {"inertial": None}
                continue
            origin = inertial.origin
            inertia = inertial.inertia
            links[link.name] = {
                "inertial": {
                    "mass_kg": child_numeric(inertial.mass),
                    "origin_xyz_m": [float(item) for item in origin.xyz],
                    "origin_rpy_rad": [float(item) for item in origin.rpy],
                    "tensor_kg_m2": [
                        [child_numeric(inertia.ixx), child_numeric(inertia.ixy), child_numeric(inertia.ixz)],
                        [child_numeric(inertia.ixy), child_numeric(inertia.iyy), child_numeric(inertia.iyz)],
                        [child_numeric(inertia.ixz), child_numeric(inertia.iyz), child_numeric(inertia.izz)],
                    ],
                }
            }
        summary = {
            "pass": True,
            "python_version": platform.python_version(),
            "xacro_module": getattr(xacro, "__file__", None),
            "parser": "urdf_parser_py.URDF.from_xml_string",
            "robot_name": robot.name,
            "link_count": len(robot.links),
            "joint_count": len(robot.joints),
            "links": links,
        }
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "expanded.urdf").write_text(xml_text, encoding="utf-8")
        (output_dir / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        return 0
    except Exception as error:  # child must return a machine-readable failure
        output_dir.mkdir(parents=True, exist_ok=True)
        (output_dir / "summary.json").write_text(
            json.dumps({"pass": False, "error": f"{type(error).__name__}: {error}"}, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        return 1


def load_yaml(path: Path) -> Any:
    try:
        import yaml  # type: ignore
    except ImportError as error:
        raise ValidationError("PyYAML is required for static/offline ROS structure validation") from error
    return yaml.safe_load(path.read_text(encoding="utf-8"))


def validate_offline_ros_structures(expanded_xml: str) -> dict[str, Any]:
    root = ET.fromstring(expanded_xml)
    require(local_name(root.tag) == "robot", "expanded URDF root is not robot")
    link_names = [str(row.get("name")) for row in root.findall("link")]
    require(len(link_names) == len(set(link_names)), "expanded URDF has duplicate links")
    links = set(link_names)
    joints = root.findall("joint")
    joint_names = [str(row.get("name")) for row in joints]
    require(len(joint_names) == len(set(joint_names)), "expanded URDF has duplicate joints")
    parent_by_child: dict[str, str] = {}
    for joint in joints:
        parent = joint.find("parent")
        child = joint.find("child")
        require(parent is not None and child is not None, f"joint lacks parent/child: {joint.get('name')}")
        first = str(parent.get("link"))
        second = str(child.get("link"))
        require(first in links and second in links, f"joint references unknown link: {joint.get('name')}")
        require(second not in parent_by_child, f"link has multiple parents: {second}")
        parent_by_child[second] = first
    roots = sorted(links - set(parent_by_child))
    require(roots == ["world"], f"URDF root mismatch: {roots}")
    for link in links:
        cursor = link
        seen = set()
        while cursor in parent_by_child:
            require(cursor not in seen, f"URDF cycle at {cursor}")
            seen.add(cursor)
            cursor = parent_by_child[cursor]
        require(cursor == "world", f"URDF disconnected link: {link}")

    srdf_root = ET.parse(repo_path(SRDF_REL)).getroot()
    require(srdf_root.get("name") == root.get("name"), "SRDF/URDF robot name mismatch")
    arm_group = next((row for row in srdf_root.findall("group") if row.get("name") == "arm"), None)
    require(arm_group is not None, "MoveIt arm group missing")
    chain = arm_group.find("chain")
    require(chain is not None and chain.get("base_link") == "base_link" and chain.get("tip_link") == "tcp_nominal", "MoveIt arm chain mismatch")
    group_state = next((row for row in srdf_root.findall("group_state") if row.get("name") == "mechanical_zero"), None)
    require(group_state is not None, "MoveIt mechanical_zero group state missing")
    require([row.get("name") for row in group_state.findall("joint")] == list(JOINTS), "MoveIt group-state joints mismatch")

    config_dir = repo_path(V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config")
    kinematics = load_yaml(config_dir / "kinematics.yaml")
    joint_limits = load_yaml(config_dir / "joint_limits.yaml")
    moveit_controllers = load_yaml(config_dir / "moveit_controllers.yaml")
    ros_controllers = load_yaml(config_dir / "ros2_controllers.yaml")
    require(isinstance(kinematics, dict) and "arm" in kinematics, "MoveIt kinematics arm config missing")
    require(set(joint_limits.get("joint_limits", {})) == set(JOINTS), "MoveIt joint limit set mismatch")
    moveit_joints = moveit_controllers["moveit_simple_controller_manager"]["arm_controller"]["joints"]
    require(tuple(moveit_joints) == JOINTS, "MoveIt controller joint order mismatch")
    ros_joints = ros_controllers["arm_controller"]["ros__parameters"]["joints"]
    require(tuple(ros_joints) == JOINTS, "ros2_control controller joint order mismatch")

    controls = root.findall("ros2_control")
    require(len(controls) == 1, "expanded URDF must contain one ros2_control block")
    control = controls[0]
    plugin = control.find("./hardware/plugin")
    require(plugin is not None and (plugin.text or "").strip() == "topic_based_ros2_control/TopicBasedSystem", "ros2_control hardware plugin mismatch")
    control_joints = control.findall("joint")
    require(tuple(str(row.get("name")) for row in control_joints) == JOINTS, "ros2_control URDF joint order mismatch")
    for joint in control_joints:
        commands = {str(row.get("name")) for row in joint.findall("command_interface")}
        states = {str(row.get("name")) for row in joint.findall("state_interface")}
        require(commands == {"position", "velocity"}, f"ros2_control command interfaces mismatch: {joint.get('name')}")
        require(states == {"position", "velocity"}, f"ros2_control state interfaces mismatch: {joint.get('name')}")

    package_root = repo_path(V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/package.xml")
    dependencies = {(row.text or "").strip() for row in ET.parse(package_root).getroot().findall("exec_depend")}
    required_dependencies = {"robot_state_publisher", "controller_manager", "moveit_ros_move_group", "topic_based_ros2_control", "xacro"}
    require(required_dependencies <= dependencies, f"package runtime dependency mismatch: {sorted(required_dependencies - dependencies)}")
    launch_text = repo_path(V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/launch/v15_14_bringup.launch.py").read_text(encoding="utf-8")
    for token in ('package="robot_state_publisher"', 'package="controller_manager"', 'executable="ros2_control_node"', 'package="moveit_ros_move_group"', 'executable="move_group"'):
        require(token in launch_text, f"bringup launch missing {token}")

    return {
        "pass": True,
        "validation_class": "STATIC_OFFLINE_MODEL_STRUCTURE",
        "ros_runtime_executed": False,
        "ros_runtime_pass_claimed": False,
        "robot_state_publisher_structure_load": {
            "pass": True,
            "mode": "STATIC_OFFLINE_URDF_TREE",
            "root_link": "world",
            "link_count": len(links),
            "joint_count": len(joints),
        },
        "moveit_model_structure_load": {
            "pass": True,
            "mode": "STATIC_OFFLINE_URDF_SRDF_AND_YAML",
            "group": "arm",
            "base_link": "base_link",
            "tip_link": "tcp_nominal",
            "controller_joints": list(moveit_joints),
        },
        "ros2_control_structure_load": {
            "pass": True,
            "mode": "STATIC_OFFLINE_URDF_PLUGIN_AND_CONTROLLER_YAML",
            "hardware_plugin": (plugin.text or "").strip(),
            "controller_joints": list(ros_joints),
        },
        "runtime_status": {
            "robot_state_publisher": "NOT_RUN_NOT_CLAIMED",
            "move_group": "NOT_RUN_NOT_CLAIMED",
            "ros2_control_node": "NOT_RUN_NOT_CLAIMED",
        },
    }


def validate_urdf(
    authority: Mapping[str, Mapping[str, Any]],
    explicit_python: str | None,
    wsl_distro: str | None,
) -> tuple[dict[str, Any], dict[str, Any]]:
    interpreter, probes = select_python(explicit_python, ("xacro", "urdf_parser_py"), ("/usr/bin/python3",))
    wsl_python = os.environ.get("V15_17_WSL_PYTHON", "python3")
    use_wsl = False
    if interpreter is None and wsl_distro:
        wsl_probe = probe_wsl_python(
            wsl_distro,
            wsl_python,
            ("xacro", "urdf_parser_py"),
            source_ros=True,
        )
        probes.append(wsl_probe)
        if wsl_probe["returncode"] == 0:
            use_wsl = True
            interpreter = f"wsl:{wsl_distro}:{wsl_python}"
    require(interpreter is not None, "no Python interpreter provides both xacro and urdf_parser_py")
    with tempfile.TemporaryDirectory(prefix="v1517_urdf_ascii_") as raw:
        temp_root = Path(raw)
        require(str(temp_root).isascii(), f"URDF temporary path is not ASCII: {temp_root}")
        mirrored_xacro = temp_root / "model.xacro"
        shutil.copyfile(repo_path(XACRO_REL), mirrored_xacro)
        output_dir = temp_root / "out"
        if use_wsl:
            assert wsl_distro is not None
            result = run_wsl_python(
                wsl_distro,
                wsl_python,
                [
                    wsl_path(wsl_distro, Path(__file__).resolve()),
                    "--_urdf-child",
                    wsl_path(wsl_distro, mirrored_xacro),
                    wsl_path(wsl_distro, output_dir),
                ],
                source_ros=True,
                timeout=45.0,
            )
        else:
            result = run_process(
                [interpreter, str(Path(__file__).resolve()), "--_urdf-child", str(mirrored_xacro), str(output_dir)],
                timeout=45.0,
            )
        require((output_dir / "summary.json").is_file(), "URDF parser child produced no summary")
        child = read_json(output_dir / "summary.json")
        require(result.returncode == 0 and child.get("pass") is True, "URDF parser child failed: " + str(child.get("error") or result.stderr.strip()))
        expanded_xml = (output_dir / "expanded.urdf").read_text(encoding="utf-8")
        expanded_canonical_hash = canonical_digest(ET.fromstring(expanded_xml))

        check_urdf = shutil.which("check_urdf")
        check_result: dict[str, Any]
        if use_wsl:
            assert wsl_distro is not None
            expanded_wsl = wsl_path(wsl_distro, output_dir / "expanded.urdf")
            checked = run_process(
                [
                    "wsl.exe",
                    "-d",
                    wsl_distro,
                    "-u",
                    "root",
                    "--",
                    "bash",
                    "-lc",
                    "source /opt/ros/humble/setup.bash && exec check_urdf "
                    + shlex.quote(expanded_wsl),
                ],
                timeout=20.0,
            )
            check_result = {
                "available": True,
                "backend": f"wsl:{wsl_distro}",
                "returncode": checked.returncode,
                "pass": checked.returncode == 0,
                "stdout_tail": checked.stdout.strip()[-1000:],
                "stderr_tail": checked.stderr.strip()[-1000:],
            }
            require(checked.returncode == 0, "check_urdf rejected expanded URDF")
        elif check_urdf:
            checked = run_process([check_urdf, str(output_dir / "expanded.urdf")], timeout=20.0)
            check_result = {
                "available": True,
                "returncode": checked.returncode,
                "pass": checked.returncode == 0,
                "stdout_tail": checked.stdout.strip()[-1000:],
                "stderr_tail": checked.stderr.strip()[-1000:],
            }
            require(checked.returncode == 0, "check_urdf rejected expanded URDF")
        else:
            check_result = {"available": False, "pass": None, "status": "OPTIONAL_NOT_AVAILABLE"}

        rows: dict[str, Any] = {}
        for name in LINKS:
            parsed = child.get("links", {}).get(name, {}).get("inertial")
            require(isinstance(parsed, dict), f"urdf_parser_py did not return inertial for {name}")
            mass_kg = finite_number(parsed.get("mass_kg"), f"{name}.parsed.mass")
            com_xyz = vector3(parsed.get("origin_xyz_m"), f"{name}.parsed.com")
            rpy = vector3(parsed.get("origin_rpy_rad"), f"{name}.parsed.rpy")
            tensor = matrix3(parsed.get("tensor_kg_m2"), f"{name}.parsed.tensor")
            expected = authority[name]
            mass_error = abs(mass_kg - float(expected["mass_kg"]))
            com_error = vector_error(com_xyz, expected["com_xyz_m"])
            delta = matrix_delta(tensor, expected["tensor_kg_m2"])
            tensor_error = max_abs(item for row in delta for item in row)
            passed = mass_error < MASS_TOL_URDF and com_error < COM_TOL_URDF and tensor_error < TENSOR_TOL_URDF and max_abs(rpy) == 0.0
            require(passed, f"parsed URDF inertial mismatch: {name}")
            rows[name] = {
                "ledger": authority[name],
                "parsed": {"mass_kg": mass_kg, "com_xyz_m": com_xyz, "rpy_rad": rpy, "tensor_kg_m2": tensor},
                "mass_error_kg": mass_error,
                "com_error_m": com_error,
                "tensor_delta_kg_m2": delta,
                "tensor_max_abs_error_kg_m2": tensor_error,
                "pass": passed,
            }
        total = sum(rows[name]["parsed"]["mass_kg"] for name in LINKS)
        total_error = abs(total - EXPECTED_TOTAL_MASS_KG)
        require(total_error < MASS_TOL_URDF, f"parsed URDF total mass mismatch: {total}")
        offline = validate_offline_ros_structures(expanded_xml)
        report = {
            "pass": True,
            "interpreter": interpreter,
            "interpreter_probes": probes,
            "parser": child.get("parser"),
            "python_version": child.get("python_version"),
            "xacro_module": child.get("xacro_module"),
            "xacro_processed": True,
            "expanded_urdf_canonical_sha256": expanded_canonical_hash,
            "robot_name": child.get("robot_name"),
            "link_count": child.get("link_count"),
            "joint_count": child.get("joint_count"),
            "check_urdf": check_result,
            "links": rows,
            "total_mass_kg": total,
            "total_mass_error_kg": total_error,
            "max_com_error_m": max(rows[name]["com_error_m"] for name in LINKS),
            "max_tensor_abs_error_kg_m2": max(rows[name]["tensor_max_abs_error_kg_m2"] for name in LINKS),
            "tolerances": {"mass_strict_lt_kg": MASS_TOL_URDF, "com_norm_strict_lt_m": COM_TOL_URDF, "tensor_max_abs_strict_lt_kg_m2": TENSOR_TOL_URDF},
        }
        return report, offline


def sanitized_proxy(name: str | None) -> str:
    if name and name.startswith("collision__"):
        parts = name.split("__")
        if len(parts) >= 4:
            return parts[2]
    return str(name)


def mujoco_child(model_path: Path, contract_path: Path, targets_path: Path, output_path: Path) -> int:
    try:
        import mujoco  # type: ignore
        import numpy as np  # type: ignore

        model = mujoco.MjModel.from_xml_path(str(model_path))
        data = mujoco.MjData(model)
        contract = read_json(contract_path)
        manifest = read_json(targets_path)
        allowed_pairs = {tuple(sorted((str(pair[0]), str(pair[1])))) for pair in contract["runtime_full_pairs"]}
        runtime_tokens = {token for pair in allowed_pairs for token in pair}
        model_tokens = set()
        for geom_id in range(model.ngeom):
            geom_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
            if geom_name and geom_name.startswith("collision__"):
                model_tokens.add(sanitized_proxy(geom_name))
        require(runtime_tokens <= model_tokens, f"compiled MJCF lacks collision tokens: {sorted(runtime_tokens-model_tokens)}")

        bodies: dict[str, Any] = {}
        for name in LINKS:
            body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))
            require(body_id >= 0, f"compiled body missing: {name}")
            quaternion = np.asarray(model.body_iquat[body_id], dtype=np.float64)
            principal = np.asarray(model.body_inertia[body_id], dtype=np.float64)
            rotation_flat = np.empty(9, dtype=np.float64)
            mujoco.mju_quat2Mat(rotation_flat, quaternion)
            bodies[name] = {
                "body_id": body_id,
                "mass_kg": float(model.body_mass[body_id]),
                "body_ipos_m": [float(item) for item in model.body_ipos[body_id]],
                "body_iquat_wxyz": [float(item) for item in quaternion],
                "body_inertia_principal_kg_m2": [float(item) for item in principal],
                "rotation_mju_quat2Mat_row_major": [
                    [float(item) for item in rotation_flat.reshape(3, 3)[row]]
                    for row in range(3)
                ],
                "dtypes": {
                    "body_mass": str(model.body_mass.dtype),
                    "body_ipos": str(model.body_ipos.dtype),
                    "body_iquat": str(model.body_iquat.dtype),
                    "body_inertia": str(model.body_inertia.dtype),
                    "np_asarray_quaternion": str(quaternion.dtype),
                    "np_asarray_principal_inertia": str(principal.dtype),
                    "mju_quat2Mat_output": str(rotation_flat.dtype),
                },
            }

        # The synthetic tests reuse the exact link2 source representation but
        # remove hierarchy, meshes, and compiler distractions.  The parent
        # independently binds the reconstructed source tensor to frozen V15.16.
        source_root = ET.parse(model_path).getroot()
        source_link2 = next(
            body for body in source_root.findall(".//body") if body.get("name") == "link2"
        )
        source_inertial = source_link2.find("inertial")
        require(source_inertial is not None, "synthetic source link2 inertial missing")
        synthetic_record: dict[str, Any] | None = None
        source_fullinertia = source_inertial.get("fullinertia")
        source_quaternion = source_inertial.get("quat")
        source_diagonal = source_inertial.get("diaginertia")
        if source_fullinertia is not None:
            synthetic_xml = (
                '<mujoco><worldbody><body name="test"><freejoint/>'
                f'<inertial pos="0.01 0.02 0.03" mass="{source_inertial.get("mass")}" '
                f'fullinertia="{source_fullinertia}"/>'
                "</body></worldbody></mujoco>"
            )
            synthetic_model = mujoco.MjModel.from_xml_string(synthetic_xml)
            synthetic_id = int(mujoco.mj_name2id(synthetic_model, mujoco.mjtObj.mjOBJ_BODY, "test"))
            synthetic_q = np.asarray(synthetic_model.body_iquat[synthetic_id], dtype=np.float64)
            synthetic_d = np.asarray(synthetic_model.body_inertia[synthetic_id], dtype=np.float64)
            synthetic_r_flat = np.empty(9, dtype=np.float64)
            mujoco.mju_quat2Mat(synthetic_r_flat, synthetic_q)
            synthetic_r = synthetic_r_flat.reshape(3, 3)
            synthetic_reconstructed = synthetic_r @ np.diag(synthetic_d) @ synthetic_r.T
            synthetic_full = np.asarray(
                [float(item) for item in source_fullinertia.split()], dtype=np.float64
            )
            synthetic_expected = np.asarray(
                [
                    [synthetic_full[0], synthetic_full[3], synthetic_full[4]],
                    [synthetic_full[3], synthetic_full[1], synthetic_full[5]],
                    [synthetic_full[4], synthetic_full[5], synthetic_full[2]],
                ],
                dtype=np.float64,
            )
            synthetic_relative = float(
                np.linalg.norm(synthetic_reconstructed - synthetic_expected)
                / np.linalg.norm(synthetic_expected)
            )
            synthetic_record = {
                "method": "mujoco.MjModel.from_xml_string_MINIMAL_ONE_BODY",
                "source": "EXACT_DEPLOYED_LINK2_FULLINERTIA",
                "mass_kg": float(synthetic_model.body_mass[synthetic_id]),
                "body_ipos_m": [float(item) for item in synthetic_model.body_ipos[synthetic_id]],
                "body_iquat_wxyz": [float(item) for item in synthetic_q],
                "body_inertia_principal_kg_m2": [float(item) for item in synthetic_d],
                "rotation_mju_quat2Mat_row_major": [
                    [float(item) for item in synthetic_r[row]] for row in range(3)
                ],
                "source_xml_tensor_kg_m2": synthetic_expected.tolist(),
                "reconstructed_tensor_kg_m2": synthetic_reconstructed.tolist(),
                "relative_frobenius_error": synthetic_relative,
                "required_strict_lt": 1.0e-12,
                "pass": synthetic_relative < 1.0e-12,
            }
        elif source_quaternion is not None and source_diagonal is not None:
            synthetic_xml = (
                '<mujoco><worldbody><body name="test"><freejoint/>'
                f'<inertial pos="0.01 0.02 0.03" mass="{source_inertial.get("mass")}" '
                f'quat="{source_quaternion}" diaginertia="{source_diagonal}"/>'
                "</body></worldbody></mujoco>"
            )
            synthetic_model = mujoco.MjModel.from_xml_string(synthetic_xml)
            synthetic_id = int(
                mujoco.mj_name2id(synthetic_model, mujoco.mjtObj.mjOBJ_BODY, "test")
            )
            synthetic_q = np.asarray(
                synthetic_model.body_iquat[synthetic_id], dtype=np.float64
            )
            synthetic_d = np.asarray(
                synthetic_model.body_inertia[synthetic_id], dtype=np.float64
            )
            synthetic_r_flat = np.empty(9, dtype=np.float64)
            mujoco.mju_quat2Mat(synthetic_r_flat, synthetic_q)
            synthetic_r = synthetic_r_flat.reshape(3, 3)
            synthetic_reconstructed = synthetic_r @ np.diag(synthetic_d) @ synthetic_r.T
            source_q = np.asarray(
                [float(item) for item in source_quaternion.split()], dtype=np.float64
            )
            source_d = np.asarray(
                [float(item) for item in source_diagonal.split()], dtype=np.float64
            )
            source_r_flat = np.empty(9, dtype=np.float64)
            mujoco.mju_quat2Mat(source_r_flat, source_q)
            source_r = source_r_flat.reshape(3, 3)
            synthetic_expected = source_r @ np.diag(source_d) @ source_r.T
            synthetic_relative = float(
                np.linalg.norm(synthetic_reconstructed - synthetic_expected)
                / np.linalg.norm(synthetic_expected)
            )
            input_rotation_error = float(np.max(np.abs(synthetic_r - source_r)))
            synthetic_record = {
                "method": "mujoco.MjModel.from_xml_string_MINIMAL_ONE_BODY",
                "source": "EXACT_DEPLOYED_LINK2_QUAT_DIAGINERTIA",
                "mass_kg": float(synthetic_model.body_mass[synthetic_id]),
                "body_ipos_m": [float(item) for item in synthetic_model.body_ipos[synthetic_id]],
                "body_iquat_wxyz": [float(item) for item in synthetic_q],
                "body_inertia_principal_kg_m2": [float(item) for item in synthetic_d],
                "rotation_mju_quat2Mat_row_major": synthetic_r.tolist(),
                "input_rotation_row_major": source_r.tolist(),
                "source_xml_tensor_kg_m2": synthetic_expected.tolist(),
                "reconstructed_tensor_kg_m2": synthetic_reconstructed.tolist(),
                "relative_frobenius_error": synthetic_relative,
                "compiled_vs_input_rotation_max_abs_error": input_rotation_error,
                "required_relative_error_strict_lt": 1.0e-12,
                "required_rotation_error_strict_lt": 1.0e-12,
                "pass": synthetic_relative < 1.0e-12
                and input_rotation_error < 1.0e-12,
            }
        else:
            raise ValidationError("synthetic source link2 has no supported inertial representation")

        independent_principal_frames: dict[str, Any] = {}
        source_body_map = {
            str(body.get("name")): body for body in source_root.findall(".//body")
        }
        for name in LINKS:
            source_body = source_body_map.get(name)
            require(source_body is not None, f"source body missing for principal audit: {name}")
            source_node = source_body.find("inertial")
            require(source_node is not None, f"source inertial missing for principal audit: {name}")
            if source_node.get("quat") is None or source_node.get("diaginertia") is None:
                continue
            input_q = np.asarray(
                [float(item) for item in str(source_node.get("quat")).split()],
                dtype=np.float64,
            )
            input_d = np.asarray(
                [float(item) for item in str(source_node.get("diaginertia")).split()],
                dtype=np.float64,
            )
            input_r_flat = np.empty(9, dtype=np.float64)
            mujoco.mju_quat2Mat(input_r_flat, input_q)
            input_r = input_r_flat.reshape(3, 3)
            frozen_tensors = contract.get("v15_17b_frozen_tensors_kg_m2", {})
            frozen_tensor = frozen_tensors.get(name) if isinstance(frozen_tensors, dict) else None
            source_tensor = (
                np.asarray(frozen_tensor, dtype=np.float64)
                if frozen_tensor is not None
                else input_r @ np.diag(input_d) @ input_r.T
            )
            moments, axes = np.linalg.eigh(source_tensor)
            order = np.argsort(moments, kind="stable")
            moments = moments[order]
            axes = axes[:, order]
            for column in range(3):
                pivot = int(np.argmax(np.abs(axes[:, column])))
                if axes[pivot, column] < 0.0:
                    axes[:, column] *= -1.0
            if float(np.linalg.det(axes)) < 0.0:
                axes[:, 2] *= -1.0
            reconstructed = axes @ np.diag(moments) @ axes.T
            mju_q = np.empty(4, dtype=np.float64)
            mujoco.mju_mat2Quat(mju_q, np.ascontiguousarray(axes.reshape(9)))
            roundtrip_flat = np.empty(9, dtype=np.float64)
            mujoco.mju_quat2Mat(roundtrip_flat, mju_q)
            manual_q = rotation_matrix_to_quaternion_wxyz_manual(axes.tolist())
            manual_r = np.asarray(quaternion_wxyz_to_matrix(manual_q), dtype=np.float64)
            independent_principal_frames[name] = {
                "source_tensor_kg_m2": source_tensor.tolist(),
                "eigh_principal_moments_kg_m2": moments.tolist(),
                "eigh_axes_body_from_inertial_columns": axes.tolist(),
                "symmetry_max_abs_error": float(
                    np.max(np.abs(source_tensor - source_tensor.T))
                ),
                "determinant": float(np.linalg.det(axes)),
                "orthonormality_max_abs_error": float(
                    np.max(np.abs(axes.T @ axes - np.eye(3)))
                ),
                "eigh_reconstruction_relative_frobenius_error": float(
                    np.linalg.norm(reconstructed - source_tensor)
                    / np.linalg.norm(source_tensor)
                ),
                "mju_mat2Quat_roundtrip_rotation_max_abs_error": float(
                    np.max(np.abs(roundtrip_flat.reshape(3, 3) - axes))
                ),
                "manual_quaternion_wxyz": manual_q,
                "manual_roundtrip_rotation_max_abs_error": float(
                    np.max(np.abs(manual_r - axes))
                ),
                "manual_vs_mju_rotation_max_abs_error": float(
                    np.max(np.abs(manual_r - roundtrip_flat.reshape(3, 3)))
                ),
                "source_input_rotation_vs_canonical_eigh_axes_max_abs_error": float(
                    np.max(np.abs(input_r - axes))
                ),
            }

        joint_ids = {name: int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)) for name in JOINTS}
        require(all(value >= 0 for value in joint_ids.values()), "compiled J1..J6 set incomplete")
        tcp_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "tcp_nominal"))
        require(tcp_id >= 0, "compiled tcp_nominal site missing")
        camera_body_ids = {
            name: int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))
            for name in ("camera_link", "sim_camera_optical_frame")
        }
        require(all(value >= 0 for value in camera_body_ids.values()), "compiled camera bodies missing")

        rows: list[dict[str, Any]] = []
        targets = manifest.get("targets")
        require(isinstance(targets, list) and len(targets) == 12, "target manifest must contain 12 targets")
        for target in targets:
            values = [float(item) for item in target["joint_position_rad"]]
            require(len(values) == 6, f"target has wrong joint count: {target.get('id')}")
            violations = []
            for name, value in zip(JOINTS, values):
                joint_id = joint_ids[name]
                if bool(model.jnt_limited[joint_id]):
                    lower, upper = (float(item) for item in model.jnt_range[joint_id])
                    if value < lower - 1.0e-12 or value > upper + 1.0e-12:
                        violations.append({"joint": name, "value_rad": value, "range_rad": [lower, upper]})
                qpos_address = int(model.jnt_qposadr[joint_id])
                data.qpos[qpos_address] = value
            mujoco.mj_forward(model, data)
            contacts = []
            if not violations:
                for index in range(data.ncon):
                    contact = data.contact[index]
                    first = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1))
                    second = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2))
                    pair = tuple(sorted((sanitized_proxy(first), sanitized_proxy(second))))
                    is_ground = "ground" in pair
                    if is_ground or pair in allowed_pairs:
                        contacts.append({"proxy_pair": list(pair), "contact_class": "ground" if is_ground else "self_collision", "distance_m": float(contact.dist)})
            expected_outcome = str(target.get("expected_outcome"))
            if expected_outcome == "execute_success":
                outcome_pass = not violations and not contacts
            elif expected_outcome == "bounds_rejection":
                outcome_pass = bool(violations)
            elif expected_outcome == "collision_rejection":
                outcome_pass = not violations and bool(contacts)
            else:
                raise ValidationError(f"unknown target expected outcome: {expected_outcome}")
            joint_world = {
                name: {
                    "anchor_m": [float(item) for item in data.xanchor[joint_ids[name]]],
                    "axis": [float(item) for item in data.xaxis[joint_ids[name]]],
                }
                for name in JOINTS
            }
            cameras = {
                name: {
                    "xpos_m": [float(item) for item in data.xpos[body_id]],
                    "xmat_row_major": [float(item) for item in data.xmat[body_id]],
                }
                for name, body_id in camera_body_ids.items()
            }
            rows.append(
                {
                    "id": str(target.get("id")),
                    "expected_outcome": expected_outcome,
                    "joint_position_rad": values,
                    "violations": violations,
                    "contacts": contacts,
                    "actual_outcome": "bounds_rejection" if violations else ("collision_rejection" if contacts else "execute_success"),
                    "outcome_pass": outcome_pass,
                    "tcp": {
                        "xpos_m": [float(item) for item in data.site_xpos[tcp_id]],
                        "xmat_row_major": [float(item) for item in data.site_xmat[tcp_id]],
                    },
                    "joints_world": joint_world,
                    "camera_bodies": cameras,
                }
            )

        diagnostic_target = manifest.get("v15_17c_fjt_accepted_pose")
        require(
            isinstance(diagnostic_target, dict),
            "locked V15.14 accepted FJT diagnostic pose missing",
        )
        locked_nonzero = [
            float(value)
            for value in diagnostic_target.get("joint_position_rad", [])
        ]
        require(
            len(locked_nonzero) == 6 and any(value != 0.0 for value in locked_nonzero),
            "locked V15.14 accepted FJT pose is invalid",
        )
        diagnostic_pose_definitions = [
            {"id": "mechanical_zero", "joint_position_rad": [0.0] * 6},
            {"id": "j6_plus_0p5", "joint_position_rad": [0.0, 0.0, 0.0, 0.0, 0.0, 0.5]},
            {"id": "j6_minus_0p5", "joint_position_rad": [0.0, 0.0, 0.0, 0.0, 0.0, -0.5]},
            {
                "id": "v15_14_accepted_fjt_final_pose",
                "source_target_id": diagnostic_target.get("source_id"),
                "source_sha256": diagnostic_target.get("source_sha256"),
                "joint_position_rad": locked_nonzero,
            },
        ]
        visual_geom_name = "visual__link6__001"
        visual_geom_id = int(
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, visual_geom_name)
        )
        link6_body_id = int(
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "link6")
        )
        require(visual_geom_id >= 0 and link6_body_id >= 0, "link6 visual/body diagnostic IDs missing")
        collision_tagged_geom_ids = {
            str(name): geom_id
            for geom_id in range(model.ngeom)
            if (name := mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id))
            and str(name).startswith("collision__")
        }
        inactive_collision_tagged_names = sorted(
            name
            for name, geom_id in collision_tagged_geom_ids.items()
            if int(model.geom_contype[geom_id]) == 0
            and int(model.geom_conaffinity[geom_id]) == 0
        )
        collision_geom_ids = {
            name: geom_id
            for name, geom_id in collision_tagged_geom_ids.items()
            if int(model.geom_contype[geom_id]) != 0
            or int(model.geom_conaffinity[geom_id]) != 0
        }
        ground_geom_id = int(
            mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "ground")
        )
        require(ground_geom_id >= 0, "compiled model ground geom missing")
        require(
            int(model.geom_contype[ground_geom_id]) != 0
            or int(model.geom_conaffinity[ground_geom_id]) != 0,
            "compiled ground geom is not collision active",
        )
        collision_geom_ids["ground"] = ground_geom_id
        require(
            len(collision_tagged_geom_ids) == 963
            and len(inactive_collision_tagged_names) == 2
            and len(collision_geom_ids) == 962,
            "compiled collision identity/count contract changed",
        )
        non_target_named_geom_ids = {
            str(name): geom_id
            for geom_id in range(model.ngeom)
            if (name := mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id))
            and str(name) != visual_geom_name
        }
        require(
            len(non_target_named_geom_ids) == 1008,
            "compiled non-target named geom count changed",
        )
        mesh_id = int(model.geom_dataid[visual_geom_id])
        visual_bbox: dict[str, Any]
        if mesh_id >= 0:
            start = int(model.mesh_vertadr[mesh_id])
            count = int(model.mesh_vertnum[mesh_id])
            require(count > 0, "link6 visual mesh has no compiled vertices")
            vertices = np.asarray(model.mesh_vert[start : start + count], dtype=np.float64)
            dimensions = np.ptp(vertices, axis=0)
            visual_bbox = {
                "method": "MUJOCO_COMPILED_MESH_VERT_AABB",
                "mesh_id": mesh_id,
                "vertex_count": count,
                "dimensions_m": [float(value) for value in dimensions],
                "diagonal_m": float(np.linalg.norm(dimensions)),
                "maximum_dimension_m": float(np.max(dimensions)),
                "source_asset_identity": manifest.get(
                    "v15_17c_visual_asset_identity"
                ),
            }
        else:
            diameter = 2.0 * float(model.geom_rbound[visual_geom_id])
            visual_bbox = {
                "method": "MUJOCO_GEOM_BOUNDING_SPHERE_DIAMETER_FALLBACK",
                "mesh_id": None,
                "vertex_count": None,
                "dimensions_m": [diameter, diameter, diameter],
                "diagonal_m": math.sqrt(3.0) * diameter,
                "maximum_dimension_m": diameter,
                "source_asset_identity": manifest.get(
                    "v15_17c_visual_asset_identity"
                ),
            }
        sameframe_storage = {
            "visual_geom": visual_geom_name,
            "visual_geom_id": visual_geom_id,
            "geom_sameframe": int(model.geom_sameframe[visual_geom_id]),
            "geom_pos_m": [float(value) for value in model.geom_pos[visual_geom_id]],
            "geom_quat_wxyz": [float(value) for value in model.geom_quat[visual_geom_id]],
            "link6_body_id": link6_body_id,
            "body_ipos_m": [float(value) for value in model.body_ipos[link6_body_id]],
            "body_iquat_wxyz": [float(value) for value in model.body_iquat[link6_body_id]],
            "visual_bbox": visual_bbox,
            "compiled_storage_is_diagnostic_not_runtime_pose": True,
        }
        runtime_pose_rows: list[dict[str, Any]] = []
        for pose in diagnostic_pose_definitions:
            values = [float(value) for value in pose["joint_position_rad"]]
            data.qpos[:] = 0.0
            data.qvel[:] = 0.0
            data.qacc[:] = 0.0
            if model.nu:
                data.ctrl[:] = 0.0
            for name, value in zip(JOINTS, values):
                data.qpos[int(model.jnt_qposadr[joint_ids[name]])] = value
            mujoco.mj_forward(model, data)
            runtime_pose_rows.append(
                {
                    **pose,
                    "visual_runtime_world_pose": {
                        "xpos_m": [float(value) for value in data.geom_xpos[visual_geom_id]],
                        "xmat_row_major": [float(value) for value in data.geom_xmat[visual_geom_id]],
                    },
                    "link6_body_runtime_world_pose": {
                        "xpos_m": [float(value) for value in data.xpos[link6_body_id]],
                        "xmat_row_major": [float(value) for value in data.xmat[link6_body_id]],
                    },
                    "joints_world": {
                        name: {
                            "anchor_m": [float(value) for value in data.xanchor[joint_ids[name]]],
                            "axis": [float(value) for value in data.xaxis[joint_ids[name]]],
                        }
                        for name in JOINTS
                    },
                    "tcp": {
                        "xpos_m": [float(value) for value in data.site_xpos[tcp_id]],
                        "xmat_row_major": [float(value) for value in data.site_xmat[tcp_id]],
                    },
                    "camera_bodies": {
                        name: {
                            "xpos_m": [float(value) for value in data.xpos[body_id]],
                            "xmat_row_major": [float(value) for value in data.xmat[body_id]],
                        }
                        for name, body_id in camera_body_ids.items()
                    },
                    "collision_geom_runtime_world_poses": {
                        name: {
                            "xpos_m": [float(value) for value in data.geom_xpos[geom_id]],
                            "xmat_row_major": [float(value) for value in data.geom_xmat[geom_id]],
                        }
                        for name, geom_id in collision_geom_ids.items()
                    },
                    "all_collision_tagged_geom_runtime_world_poses": {
                        name: {
                            "xpos_m": [float(value) for value in data.geom_xpos[geom_id]],
                            "xmat_row_major": [float(value) for value in data.geom_xmat[geom_id]],
                        }
                        for name, geom_id in collision_tagged_geom_ids.items()
                    },
                    "non_target_named_geom_runtime_world_poses": {
                        name: {
                            "xpos_m": [float(value) for value in data.geom_xpos[geom_id]],
                            "xmat_row_major": [float(value) for value in data.geom_xmat[geom_id]],
                        }
                        for name, geom_id in non_target_named_geom_ids.items()
                    },
                }
            )

        # Zero/gravity-off generalized-physics diagnostics.  A separate child
        # runs the frozen fullinertia representation; the parent compares the
        # two without treating the old compiler result as authority.
        data.qpos[:] = 0.0
        data.qvel[:] = 0.0
        data.qacc[:] = 0.0
        mujoco.mj_forward(model, data)
        mass_matrix = np.empty((model.nv, model.nv), dtype=np.float64)
        mujoco.mj_fullM(model, data, mass_matrix)
        zero_pose = {
            "qpos": [float(item) for item in data.qpos],
            "bodies": {
                str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)):
                {
                    "xpos_m": [float(item) for item in data.xpos[body_id]],
                    "xmat_row_major": [float(item) for item in data.xmat[body_id]],
                }
                for body_id in range(1, model.nbody)
                if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
            },
            "joints": {
                str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)):
                {
                    "anchor_m": [float(item) for item in data.xanchor[joint_id]],
                    "axis": [float(item) for item in data.xaxis[joint_id]],
                }
                for joint_id in range(model.njnt)
                if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
            },
            "geoms": {
                str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)):
                {
                    "xpos_m": [float(item) for item in data.geom_xpos[geom_id]],
                    "xmat_row_major": [float(item) for item in data.geom_xmat[geom_id]],
                    "compiled_geom_sameframe": int(model.geom_sameframe[geom_id]),
                }
                for geom_id in range(model.ngeom)
                if mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id)
            },
        }

        summary = {
            "pass": all(row["outcome_pass"] for row in rows),
            "python_version": platform.python_version(),
            "mujoco_version": getattr(mujoco, "__version__", None),
            "model": {"nbody": int(model.nbody), "nq": int(model.nq), "njnt": int(model.njnt), "ngeom": int(model.ngeom), "npair": int(model.npair), "gravity_m_s2": [float(item) for item in model.opt.gravity]},
            "bodies": bodies,
            "dtype_audit": {
                "numpy_default_numeric_path": "float64",
                "all_compiled_readback_and_reconstruction_arrays_float64": all(
                    value == "float64"
                    for body in bodies.values()
                    for value in body["dtypes"].values()
                ),
            },
            "synthetic_link2_fullinertia": synthetic_record,
            "synthetic_link2": synthetic_record,
            "independent_principal_frames": independent_principal_frames,
            "v15_17c_sameframe_runtime_diagnostic": {
                "pass": True,
                "pose_source_manifest_schema": manifest.get("schema"),
                "pose_count": len(runtime_pose_rows),
                "sameframe_storage": sameframe_storage,
                "poses": runtime_pose_rows,
                "collision_active_geom_count_including_ground": len(
                    collision_geom_ids
                ),
                "collision_tagged_geom_count": len(collision_tagged_geom_ids),
                "inactive_collision_tagged_geom_count": len(
                    inactive_collision_tagged_names
                ),
                "inactive_collision_tagged_geom_names": inactive_collision_tagged_names,
                "non_target_named_geom_count": len(non_target_named_geom_ids),
                "runtime_world_pose_requires_mj_forward": True,
            },
            "zero_gravity_off": {
                "gravity_m_s2": [float(item) for item in model.opt.gravity],
                "pose_snapshot": zero_pose,
                "generalized_mass_matrix": mass_matrix.tolist(),
                "generalized_mass_matrix_dtype": str(mass_matrix.dtype),
                "diagnostic_not_frozen_tensor_authority": True,
            },
            "targets": rows,
        }
        output_path.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding="utf-8")
        return 0 if summary["pass"] else 2
    except Exception as error:
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(json.dumps({"pass": False, "error": f"{type(error).__name__}: {error}"}, ensure_ascii=False, indent=2), encoding="utf-8")
        return 1


def make_ascii_mjcf_mirror(source_xml: bytes, source_directory: Path, destination: Path) -> Path:
    root = ET.fromstring(source_xml)
    compiler = root.find("compiler")
    mesh_dir = source_directory / (compiler.get("meshdir", ".") if compiler is not None else ".")
    texture_dir = source_directory / (compiler.get("texturedir", ".") if compiler is not None else ".")
    assets = destination / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    file_assets = [element for element in root.findall("./asset/*") if element.get("file")]
    for index, element in enumerate(file_assets):
        raw_file = str(element.get("file"))
        require(not Path(raw_file).is_absolute(), f"absolute MJCF asset path is forbidden: {raw_file}")
        base = mesh_dir if local_name(element.tag) == "mesh" else texture_dir
        source = (base / raw_file).resolve()
        require(source.is_file(), f"MJCF asset missing: {raw_file}")
        suffix = source.suffix.lower() or ".bin"
        target_name = f"asset_{index:04d}{suffix}"
        shutil.copyfile(source, assets / target_name)
        element.set("file", "assets/" + target_name)
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", ".")
    compiler.set("texturedir", ".")
    model_path = destination / "model.xml"
    ET.ElementTree(root).write(model_path, encoding="utf-8", xml_declaration=True)
    require(str(model_path).isascii(), f"MuJoCo mirror path is not ASCII: {model_path}")
    return model_path


def legacy_fullinertia_model_bytes(
    explicit_xml: bytes,
    authority: Mapping[str, Mapping[str, Any]],
) -> bytes:
    """Build an in-memory A model from frozen tensors for diagnostics only."""

    root = ET.fromstring(explicit_xml)
    bodies: dict[str, list[ET.Element]] = {}
    for body in root.findall(".//body"):
        bodies.setdefault(str(body.get("name")), []).append(body)
    for name in LINKS:
        require(len(bodies.get(name, [])) == 1, f"legacy A body occurrence mismatch: {name}")
        inertials = bodies[name][0].findall("inertial")
        require(len(inertials) == 1, f"legacy A inertial occurrence mismatch: {name}")
        node = inertials[0]
        expected = authority[name]
        node.attrib.clear()
        node.set("pos", " ".join(format(float(item), ".17g") for item in expected["com_xyz_m"]))
        node.set("mass", format(float(expected["mass_kg"]), ".17g"))
        node.set(
            "fullinertia",
            " ".join(
                format(float(item), ".17g")
                for item in fullinertia_values(expected["tensor_kg_m2"])
            ),
        )
    return ET.tostring(root, encoding="utf-8", xml_declaration=True)


def run_mujoco_child(
    interpreter: str,
    model_path: Path,
    contract: Path,
    targets: Path,
    output: Path,
    *,
    wsl_distro: str | None = None,
    wsl_python: str | None = None,
) -> dict[str, Any]:
    if wsl_distro and wsl_python:
        result = run_wsl_python(
            wsl_distro,
            wsl_python,
            [
                wsl_path(wsl_distro, Path(__file__).resolve()),
                "--_mujoco-child",
                wsl_path(wsl_distro, model_path),
                wsl_path(wsl_distro, contract),
                wsl_path(wsl_distro, targets),
                wsl_path(wsl_distro, output),
            ],
            source_ros=False,
            timeout=90.0,
        )
    else:
        result = run_process(
            [interpreter, str(Path(__file__).resolve()), "--_mujoco-child", str(model_path), str(contract), str(targets), str(output)],
            timeout=90.0,
        )
    require(output.is_file(), "MuJoCo child produced no output")
    value = read_json(output)
    require(result.returncode in (0, 2) and "bodies" in value, "MuJoCo child failed: " + str(value.get("error") or result.stderr.strip()))
    return value


def flatten_numeric(value: Any) -> Iterable[float]:
    if isinstance(value, (int, float)) and not isinstance(value, bool):
        yield float(value)
    elif isinstance(value, list):
        for item in value:
            yield from flatten_numeric(item)
    elif isinstance(value, dict):
        for key in sorted(value):
            yield from flatten_numeric(value[key])


def target_semantic_view(row: Mapping[str, Any]) -> dict[str, Any]:
    return {
        "id": row["id"],
        "expected_outcome": row["expected_outcome"],
        "joint_position_rad": row["joint_position_rad"],
        "actual_outcome": row["actual_outcome"],
        "contact_pairs": sorted(tuple(item["proxy_pair"]) for item in row["contacts"]),
        "tcp": row["tcp"],
        "joints_world": row["joints_world"],
        "camera_bodies": row["camera_bodies"],
    }


def semantic_numeric_error(current: Mapping[str, Any], baseline: Mapping[str, Any]) -> float:
    left = list(flatten_numeric(current))
    right = list(flatten_numeric(baseline))
    require(len(left) == len(right), "kinematic semantic numeric shape mismatch")
    return max_abs(a - b for a, b in zip(left, right))


def validate_collision_static() -> dict[str, Any]:
    path = repo_path(COLLISION_VALIDATOR_REL)
    specification = importlib.util.spec_from_file_location("_v1517_collision_static", str(path))
    require(specification is not None and specification.loader is not None, "cannot load V15.14 collision validator")
    module = importlib.util.module_from_spec(specification)
    specification.loader.exec_module(module)
    result = module.static_validation(V14_ROOT)
    require(result.get("pass") is True, "V15.14 static collision regression failed")
    return {
        "pass": True,
        "validator": COLLISION_VALIDATOR_REL,
        "pair_partition": result.get("pair_partition"),
        "runtime_token_coverage": result.get("runtime_token_coverage"),
        "srdf_acm_exclusion_count": result.get("srdf_acm_exclusion_count"),
        "frozen_urdf_hashes": result.get("frozen_urdf_hashes"),
        "frozen_mjcf_hashes": result.get("frozen_mjcf_hashes"),
        "legacy_generated_whole_file_hashes": result.get("generated_hashes"),
        "legacy_generated_whole_file_hashes_used_as_current_acceptance_gate": False,
    }


def validate_mjcf(
    authority: Mapping[str, Mapping[str, Any]],
    explicit_python: str | None,
    wsl_distro: str | None,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    interpreter, probes = select_python(explicit_python, ("mujoco",), ())
    wsl_python = os.environ.get("V15_17_WSL_PYTHON", "python3")
    use_wsl = False
    if interpreter is None and wsl_distro:
        wsl_probe = probe_wsl_python(
            wsl_distro,
            wsl_python,
            ("mujoco",),
            source_ros=False,
        )
        probes.append(wsl_probe)
        if wsl_probe["returncode"] == 0:
            use_wsl = True
            interpreter = f"wsl:{wsl_distro}:{wsl_python}"
    require(interpreter is not None, "no Python interpreter provides mujoco")
    current_xml = repo_path(MJCF_REL).read_bytes()
    baseline_xml = git_blob(MJCF_REL)
    with tempfile.TemporaryDirectory(prefix="v1517_mjcf_ascii_") as raw:
        temp_root = Path(raw)
        require(str(temp_root).isascii(), f"MuJoCo temporary path is not ASCII: {temp_root}")
        current_dir = temp_root / "current"
        baseline_dir = temp_root / "baseline"
        current_dir.mkdir()
        baseline_dir.mkdir()
        source_directory = repo_path(MJCF_REL).parent
        current_model = make_ascii_mjcf_mirror(current_xml, source_directory, current_dir)
        baseline_model = make_ascii_mjcf_mirror(baseline_xml, source_directory, baseline_dir)
        contract = temp_root / "contract.json"
        targets = temp_root / "targets.json"
        collision_contract = read_json(repo_path(COLLISION_CONTRACT_REL))
        collision_contract["v15_17b_frozen_tensors_kg_m2"] = {
            name: authority[name]["tensor_kg_m2"] for name in LINKS
        }
        contract.write_text(
            json.dumps(collision_contract, ensure_ascii=False, sort_keys=True),
            encoding="utf-8",
        )
        write_v15_17c_child_manifest(targets)
        current = run_mujoco_child(
            interpreter,
            current_model,
            contract,
            targets,
            temp_root / "current.json",
            wsl_distro=wsl_distro if use_wsl else None,
            wsl_python=wsl_python if use_wsl else None,
        )
        baseline = run_mujoco_child(
            interpreter,
            baseline_model,
            contract,
            targets,
            temp_root / "baseline.json",
            wsl_distro=wsl_distro if use_wsl else None,
            wsl_python=wsl_python if use_wsl else None,
        )

    rows: dict[str, Any] = {}
    source_rows = validate_source_xml_and_frozen_semantics(authority)["source_mjcf_inertials"]
    global_manual_vs_mujoco_max = 0.0
    dtype_rows: dict[str, Any] = {}
    for name in LINKS:
        compiled = current["bodies"].get(name)
        require(isinstance(compiled, dict), f"compiled MJCF body missing: {name}")
        mass_kg = finite_number(compiled.get("mass_kg"), f"{name}.compiled.mass")
        com_xyz = vector3(compiled.get("body_ipos_m"), f"{name}.compiled.com")
        quaternion = vector3plus1(compiled.get("body_iquat_wxyz"), f"{name}.compiled.iquat")
        moments = vector3(compiled.get("body_inertia_principal_kg_m2"), f"{name}.compiled.inertia")
        require(all(item > 0.0 for item in moments), f"{name}: non-positive compiled principal inertia")
        ordered = sorted(moments)
        require(ordered[0] + ordered[1] >= ordered[2] - 1.0e-12, f"{name}: compiled principal triangle inequality failed")
        rotation_mujoco = matrix3(
            compiled.get("rotation_mju_quat2Mat_row_major"),
            f"{name}.compiled.rotation_mju",
        )
        rotation_manual = quaternion_wxyz_to_matrix(quaternion)
        rotation_agreement = max_abs(
            item for row in matrix_delta(rotation_manual, rotation_mujoco) for item in row
        )
        global_manual_vs_mujoco_max = max(global_manual_vs_mujoco_max, rotation_agreement)
        require(rotation_agreement < 1.0e-12, f"{name}: manual quaternion path disagrees with mju_quat2Mat")
        principal_matrix = diagonal_matrix(moments)
        reconstructed = matmul(matmul(rotation_mujoco, principal_matrix), transpose(rotation_mujoco))
        inverse_diagnostic = matmul(matmul(transpose(rotation_mujoco), principal_matrix), rotation_mujoco)
        expected = authority[name]
        source_xml = source_rows[name]
        xml_tensor = matrix3(source_xml["tensor_kg_m2"], f"{name}.source_xml.tensor")
        mass_error = abs(mass_kg - float(expected["mass_kg"]))
        com_error = vector_error(com_xyz, expected["com_xyz_m"])
        delta = matrix_delta(reconstructed, expected["tensor_kg_m2"])
        absolute = frobenius(delta)
        serialization_relative = relative_frobenius(xml_tensor, expected["tensor_kg_m2"])
        compile_relative = relative_frobenius(reconstructed, xml_tensor)
        relative = relative_frobenius(reconstructed, expected["tensor_kg_m2"])
        inverse_relative = relative_frobenius(inverse_diagnostic, expected["tensor_kg_m2"])
        body_dtypes = compiled.get("dtypes")
        require(isinstance(body_dtypes, dict), f"{name}: compiled dtype evidence missing")
        dtype_pass = bool(body_dtypes) and all(value == "float64" for value in body_dtypes.values())
        dtype_rows[name] = {"arrays": body_dtypes, "all_float64": dtype_pass}
        mass_pass = mass_error < MASS_TOL_MJCF
        com_pass = com_error < COM_TOL_MJCF
        tensor_pass = relative < TENSOR_REL_TOL_MJCF
        passed = mass_pass and com_pass and tensor_pass
        rows[name] = {
            "ledger": authority[name],
            "source_xml": source_xml,
            "compiled": {"mass_kg": mass_kg, "body_ipos_m": com_xyz, "body_iquat_wxyz": quaternion, "body_inertia_principal_kg_m2": moments},
            "dtypes": compiled.get("dtypes"),
            "rotation_mju_quat2Mat_row_major": rotation_mujoco,
            "rotation_manual_wxyz_row_major": rotation_manual,
            "rotation_manual_vs_mju_max_abs_error": rotation_agreement,
            "reconstructed_tensor_body_frame_kg_m2": reconstructed,
            "inverse_direction_diagnostic_tensor_kg_m2": inverse_diagnostic,
            "inverse_direction_relative_frobenius_error": inverse_relative,
            "direction_semantics": {
                "accepted": "R_mju_quat2Mat @ diag(body_inertia) @ R_mju_quat2Mat.T",
                "inverse_is_diagnostic_only": True,
                "automatic_best_direction_selection_forbidden": True,
            },
            "tensor_delta_kg_m2": delta,
            "mass_error_kg": mass_error,
            "com_error_m": com_error,
            "tensor_frobenius_error_kg_m2": absolute,
            "tensor_relative_frobenius_error": relative,
            "error_decomposition": {
                "serialization_relative_frobenius_error": serialization_relative,
                "compile_reconstruction_relative_frobenius_error": compile_relative,
                "total_relative_frobenius_error": relative,
            },
            "mass_pass": mass_pass,
            "com_pass": com_pass,
            "reconstructed_tensor_pass": tensor_pass,
            "pass": passed,
        }
    total = sum(rows[name]["compiled"]["mass_kg"] for name in LINKS)
    total_error = abs(total - EXPECTED_TOTAL_MASS_KG)
    require(total_error < MASS_TOL_MJCF, f"compiled MJCF total mass mismatch: {total}")
    gravity = vector3(current["model"].get("gravity_m_s2"), "compiled.gravity")
    require(max_abs(gravity) == 0.0, f"compiled gravity is enabled: {gravity}")

    current_targets = {str(row["id"]): row for row in current["targets"]}
    baseline_targets = {str(row["id"]): row for row in baseline["targets"]}
    require(set(current_targets) == set(baseline_targets) and len(current_targets) == 12, "current/baseline target set mismatch")
    regression_rows = []
    max_kinematic_error = 0.0
    for name in current_targets:
        current_view = target_semantic_view(current_targets[name])
        baseline_view = target_semantic_view(baseline_targets[name])
        nonnumeric_equal = (
            current_view["id"] == baseline_view["id"]
            and current_view["expected_outcome"] == baseline_view["expected_outcome"]
            and current_view["actual_outcome"] == baseline_view["actual_outcome"]
            and current_view["contact_pairs"] == baseline_view["contact_pairs"]
        )
        numeric_error = semantic_numeric_error(current_view, baseline_view)
        max_kinematic_error = max(max_kinematic_error, numeric_error)
        passed = bool(current_targets[name]["outcome_pass"]) and nonnumeric_equal and numeric_error < KINEMATICS_ABS_TOL
        require(passed, f"12-target kinematic/collision regression failed: {name}")
        regression_rows.append(
            {
                "id": name,
                "expected_outcome": current_targets[name]["expected_outcome"],
                "actual_outcome": current_targets[name]["actual_outcome"],
                "contact_pairs": current_view["contact_pairs"],
                "max_numeric_kinematic_delta": numeric_error,
                "semantic_outcome_unchanged": nonnumeric_equal,
                "pass": passed,
            }
        )
    collision_static = validate_collision_static()
    serialization_pass = all(
        rows[name]["error_decomposition"]["serialization_relative_frobenius_error"] < 1.0e-12
        for name in LINKS
    )
    dtype_pass = all(dtype_rows[name]["all_float64"] for name in LINKS)
    quaternion_pass = global_manual_vs_mujoco_max < 1.0e-12
    synthetic_child = current.get("synthetic_link2_fullinertia")
    require(isinstance(synthetic_child, dict), "current deployed synthetic link2 evidence missing")
    synthetic_q = vector3plus1(synthetic_child.get("body_iquat_wxyz"), "synthetic.iquat")
    synthetic_d = vector3(synthetic_child.get("body_inertia_principal_kg_m2"), "synthetic.inertia")
    synthetic_r = matrix3(synthetic_child.get("rotation_mju_quat2Mat_row_major"), "synthetic.rotation")
    synthetic_expected = matrix3(synthetic_child.get("source_xml_tensor_kg_m2"), "synthetic.source")
    synthetic_reconstructed = matmul(
        matmul(synthetic_r, diagonal_matrix(synthetic_d)), transpose(synthetic_r)
    )
    synthetic_relative = relative_frobenius(synthetic_reconstructed, synthetic_expected)
    require(
        abs(synthetic_relative - finite_number(synthetic_child.get("relative_frobenius_error"), "synthetic.reported_error")) < 1.0e-15,
        "synthetic child/parent relative error disagreement",
    )
    synthetic_manual_r = quaternion_wxyz_to_matrix(synthetic_q)
    synthetic_rotation_error = max_abs(
        item for row in matrix_delta(synthetic_manual_r, synthetic_r) for item in row
    )
    synthetic_pass = synthetic_relative < 1.0e-12 and synthetic_rotation_error < 1.0e-12
    synthetic_report = {
        **synthetic_child,
        "parent_independently_reconstructed_tensor_kg_m2": synthetic_reconstructed,
        "parent_independently_recomputed_relative_frobenius_error": synthetic_relative,
        "manual_wxyz_vs_mju_quat2Mat_max_abs_error": synthetic_rotation_error,
        "pass": synthetic_pass,
    }
    all_six_tensor_pass = all(
        rows[name]["tensor_relative_frobenius_error"] < 1.0e-9 for name in LINKS
    )
    compiler_transformation_unresolved = (
        serialization_pass
        and dtype_pass
        and quaternion_pass
        and not synthetic_pass
        and not all_six_tensor_pass
        and current.get("mujoco_version") == "3.11.0"
    )
    root_cause = (
        "MUJOCO_3_11_0_FULLINERTIA_INTERNAL_EIG3_EARLY_TERMINATION_PRECISION_LIMIT"
        if compiler_transformation_unresolved
        else "INERTIA_READBACK_ROOT_CAUSE_NOT_ISOLATED"
    )
    readback_pass = (
        all(rows[name]["pass"] for name in LINKS)
        and serialization_pass
        and dtype_pass
        and quaternion_pass
        and synthetic_pass
    )
    mjcf_report = {
        "pass": readback_pass,
        "compile_pass": True,
        "interpreter": interpreter,
        "interpreter_probes": probes,
        "python_version": current.get("python_version"),
        "mujoco_version": current.get("mujoco_version"),
        "compile_method": "mujoco.MjModel.from_xml_path",
        "four_stage_trace": {
            "stage_a": "FROZEN_INERTIA_ENGINEERING_V1_LEDGER_TENSOR",
            "stage_b": "DIRECTLY_PARSED_MJCF_FULLINERTIA_TENSOR",
            "stage_c": "MUJOCO_BODY_INERTIA_AND_BODY_IQUAT_WXYZ",
            "stage_d": "R_MJU_QUAT2MAT_DIAG_R_TRANSPOSE_BODY_FRAME_TENSOR",
        },
        "serialization": {
            "formatter": "PYTHON_REPR_SHORTEST_BINARY64_ROUNDTRIP_EQUIVALENT_TO_AT_LEAST_17_SIGNIFICANT_DIGITS_WHEN_NEEDED",
            "per_link_significant_decimal_digits": {
                name: source_rows[name]["serialized_significant_decimal_digits"] for name in LINKS
            },
            "max_relative_frobenius_error": max(
                rows[name]["error_decomposition"]["serialization_relative_frobenius_error"]
                for name in LINKS
            ),
            "required_strict_lt": 1.0e-12,
            "pass": serialization_pass,
        },
        "dtype_audit": {
            "links": dtype_rows,
            "child_aggregate": current.get("dtype_audit"),
            "all_compiled_readback_and_reconstruction_arrays_float64": dtype_pass,
            "float32_conversion_detected": not dtype_pass,
            "pass": dtype_pass,
        },
        "quaternion_conversion_audit": {
            "mujoco_quaternion_order": "W_X_Y_Z",
            "official_path": "mujoco.mju_quat2Mat",
            "independent_path": "MANUAL_STANDARD_WXYZ_ROTATION_MATRIX",
            "max_abs_rotation_matrix_error": global_manual_vs_mujoco_max,
            "required_strict_lt": 1.0e-12,
            "pass": quaternion_pass,
        },
        "synthetic_link2_fullinertia": synthetic_report,
        "compiler_configuration": {
            "explicit_attributes": source_rows and validate_source_xml_and_frozen_semantics(authority).get("compiler_attributes"),
            "effective_inertia_related_defaults": {
                "boundmass": 0,
                "boundinertia": 0,
                "settotalmass": -1,
                "balanceinertia": False,
                "inertiafromgeom": "auto_but_disabled_per_body_by_explicit_inertial",
            },
            "eigendecomposition_tolerance_is_not_an_mjcf_compiler_attribute": True,
            "mass_scaling_or_inertia_balancing_or_lower_bound_applied": False,
        },
        "root_cause": {
            "code": root_cause,
            "localization": "XML serialization is exact, all numeric paths are float64, manual wxyz agrees with mju_quat2Mat, and the minimal one-body link2 model reproduces the robot error.",
            "random_precompensation_or_authority_adjustment_performed": False,
            "compiler_transformation_unresolved": compiler_transformation_unresolved,
        },
        "all_six_tensor_readbacks_strict_lt_1e_9": all_six_tensor_pass,
        "ascii_mirror_used": True,
        "asset_paths_rewritten_to_ascii_mirror": True,
        "model": current.get("model"),
        "links": rows,
        "readback_failed_links": [name for name in LINKS if not rows[name]["pass"]],
        "total_mass_kg": total,
        "total_mass_error_kg": total_error,
        "max_com_error_m": max(rows[name]["com_error_m"] for name in LINKS),
        "max_reconstructed_tensor_relative_frobenius_error": max(rows[name]["tensor_relative_frobenius_error"] for name in LINKS),
        "gravity_enabled": False,
        "tolerances": {"mass_strict_lt_kg": MASS_TOL_MJCF, "com_norm_strict_lt_m": COM_TOL_MJCF, "tensor_relative_frobenius_strict_lt": TENSOR_REL_TOL_MJCF},
    }
    collision_report = {
        "pass": True,
        "static_collision_layer_revalidation": collision_static,
        "target_count": len(regression_rows),
        "all_target_expected_outcomes_pass": True,
        "targets": regression_rows,
    }
    kinematics_report = {
        "pass": True,
        "mode": "COMPILED_MJCF_12_TARGET_SEMANTIC_EQUIVALENCE",
        "ros_trajectory_loop_runtime_executed": False,
        "ros_trajectory_loop_runtime_pass_claimed": False,
        "baseline_commit": SOURCE_COMMIT,
        "baseline_whole_mjcf_sha256": sha256_bytes(baseline_xml),
        "baseline_whole_hash_used_as_acceptance_gate": False,
        "compared": ["J1_J6_world_anchor", "J1_J6_world_axis", "tcp_nominal_world_transform", "camera_link_world_transform", "sim_camera_optical_frame_world_transform", "collision_outcome_and_proxy_pairs"],
        "target_count": len(regression_rows),
        "max_numeric_delta": max_kinematic_error,
        "tolerance_strict_lt": KINEMATICS_ABS_TOL,
        "world_root_unchanged": True,
        "joint_origins_unchanged": True,
        "joint_axes_unchanged": True,
        "tcp_unchanged": True,
        "camera_tf_unchanged": True,
    }
    return mjcf_report, collision_report, kinematics_report


def _zero_snapshot_delta(
    current: Mapping[str, Any], legacy: Mapping[str, Any]
) -> tuple[float, bool]:
    if set(current) != set(legacy):
        return math.inf, False
    left = list(flatten_numeric(dict(current)))
    right = list(flatten_numeric(dict(legacy)))
    if len(left) != len(right):
        return math.inf, False
    return max_abs(a - b for a, b in zip(left, right)), True


def compare_v15_17c_sameframe_runtime(
    explicit_child: Mapping[str, Any],
    legacy_child: Mapping[str, Any],
) -> dict[str, Any]:
    explicit = explicit_child.get("v15_17c_sameframe_runtime_diagnostic")
    legacy = legacy_child.get("v15_17c_sameframe_runtime_diagnostic")
    require(isinstance(explicit, dict) and isinstance(legacy, dict), "V15.17C child diagnostics missing")
    require(
        explicit.get("pose_count") == legacy.get("pose_count") == 4,
        "V15.17C diagnostic must contain exactly four poses",
    )
    for key, expected in (
        ("collision_active_geom_count_including_ground", 962),
        ("collision_tagged_geom_count", 963),
        ("inactive_collision_tagged_geom_count", 2),
        ("non_target_named_geom_count", 1008),
    ):
        require(
            explicit.get(key) == legacy.get(key) == expected,
            f"V15.17C child {key} contract mismatch",
        )
    require(
        explicit.get("inactive_collision_tagged_geom_names")
        == legacy.get("inactive_collision_tagged_geom_names"),
        "V15.17C inactive collision-tagged identity mismatch",
    )
    require(
        explicit.get("inactive_collision_tagged_geom_names")
        == [
            "collision__link2__UpperArm_Motion_Collision_Proxy__001",
            "collision__link2__UpperArm_Motion_Collision_Proxy__002",
        ],
        "V15.17C inactive collision-tagged identities changed",
    )
    explicit_storage = explicit.get("sameframe_storage")
    legacy_storage = legacy.get("sameframe_storage")
    require(isinstance(explicit_storage, dict) and isinstance(legacy_storage, dict), "sameframe storage evidence missing")
    require(
        explicit_storage.get("visual_geom")
        == legacy_storage.get("visual_geom")
        == "visual__link6__001",
        "sameframe diagnostic visual identity mismatch",
    )
    compiled_geom_pos_delta = vector_error(
        vector3(legacy_storage.get("geom_pos_m"), "legacy.geom_pos"),
        vector3(explicit_storage.get("geom_pos_m"), "explicit.geom_pos"),
    )
    legacy_geom_rotation = quaternion_wxyz_to_matrix(
        vector3plus1(legacy_storage.get("geom_quat_wxyz"), "legacy.geom_quat")
    )
    explicit_geom_rotation = quaternion_wxyz_to_matrix(
        vector3plus1(explicit_storage.get("geom_quat_wxyz"), "explicit.geom_quat")
    )
    compiled_geom_rotation_delta = rotation_delta_angle_rad(
        legacy_geom_rotation, explicit_geom_rotation
    )
    body_ipos_delta = vector_error(
        vector3(legacy_storage.get("body_ipos_m"), "legacy.body_ipos"),
        vector3(explicit_storage.get("body_ipos_m"), "explicit.body_ipos"),
    )
    body_iquat_rotation_delta = rotation_delta_angle_rad(
        quaternion_wxyz_to_matrix(
            vector3plus1(legacy_storage.get("body_iquat_wxyz"), "legacy.body_iquat")
        ),
        quaternion_wxyz_to_matrix(
            vector3plus1(explicit_storage.get("body_iquat_wxyz"), "explicit.body_iquat")
        ),
    )
    explicit_bbox = explicit_storage.get("visual_bbox")
    legacy_bbox = legacy_storage.get("visual_bbox")
    require(isinstance(explicit_bbox, dict) and isinstance(legacy_bbox, dict), "visual bbox evidence missing")
    bbox_dimensions = vector3(explicit_bbox.get("dimensions_m"), "explicit visual bbox dimensions")
    legacy_bbox_dimensions = vector3(legacy_bbox.get("dimensions_m"), "legacy visual bbox dimensions")
    require(vector_error(bbox_dimensions, legacy_bbox_dimensions) == 0.0, "A/B visual bbox dimensions changed")
    expected_asset_identity = {
        "runtime_mesh_manifest_sha256": EXPECTED_V15_17C_DIAGNOSTIC_INPUT_SHA256[
            RUNTIME_MESH_MANIFEST_REL
        ],
        "source_mesh_path": LINK6_VISUAL_MESH_REL,
        "source_mesh_sha256": EXPECTED_V15_17C_DIAGNOSTIC_INPUT_SHA256[
            LINK6_VISUAL_MESH_REL
        ],
    }
    require(
        explicit_bbox.get("source_asset_identity")
        == legacy_bbox.get("source_asset_identity")
        == expected_asset_identity,
        "V15.17C visual bbox source asset is not hash bound",
    )
    bbox_diagonal = finite_number(explicit_bbox.get("diagonal_m"), "visual bbox diagonal")
    bbox_maximum = finite_number(explicit_bbox.get("maximum_dimension_m"), "visual bbox maximum dimension")
    require(bbox_diagonal > 0.0 and bbox_maximum > 0.0, "visual bbox size is not positive")

    explicit_poses = {
        str(row.get("id")): row for row in explicit.get("poses", []) if isinstance(row, dict)
    }
    legacy_poses = {
        str(row.get("id")): row for row in legacy.get("poses", []) if isinstance(row, dict)
    }
    expected_pose_order = (
        "mechanical_zero",
        "j6_plus_0p5",
        "j6_minus_0p5",
        "v15_14_accepted_fjt_final_pose",
    )
    require(
        set(explicit_poses) == set(legacy_poses) == set(expected_pose_order),
        "V15.17C A/B pose identity mismatch",
    )
    pose_rows: list[dict[str, Any]] = []
    max_visual_position = 0.0
    max_visual_position_component = 0.0
    max_visual_rotation = 0.0
    max_body_position = 0.0
    max_body_rotation = 0.0
    max_joint_anchor = 0.0
    max_joint_axis = 0.0
    max_tf_position = 0.0
    max_tf_rotation = 0.0
    max_collision_position = 0.0
    max_collision_rotation = 0.0
    collision_comparison_count = 0
    collision_exact_count = 0
    collision_tagged_comparison_count = 0
    collision_tagged_exact_count = 0
    max_collision_tagged_position = 0.0
    max_collision_tagged_rotation = 0.0
    non_target_geom_comparison_count = 0
    non_target_geom_exact_count = 0
    max_non_target_geom_position = 0.0
    max_non_target_geom_rotation = 0.0

    def pose_delta(old: Mapping[str, Any], new: Mapping[str, Any], label: str) -> tuple[float, float]:
        position = vector_error(
            vector3(old.get("xpos_m"), f"{label}.old.xpos"),
            vector3(new.get("xpos_m"), f"{label}.new.xpos"),
        )
        rotation = rotation_delta_angle_rad(
            matrix3_from_row_major(old.get("xmat_row_major"), f"{label}.old.xmat"),
            matrix3_from_row_major(new.get("xmat_row_major"), f"{label}.new.xmat"),
        )
        return position, rotation

    def position_max_abs_delta(
        old: Mapping[str, Any], new: Mapping[str, Any], label: str
    ) -> float:
        old_position = vector3(old.get("xpos_m"), f"{label}.old.xpos")
        new_position = vector3(new.get("xpos_m"), f"{label}.new.xpos")
        return max_abs(a - b for a, b in zip(old_position, new_position))

    for pose_id in expected_pose_order:
        new = explicit_poses[pose_id]
        old = legacy_poses[pose_id]
        require(
            len(new.get("joint_position_rad", []))
            == len(old.get("joint_position_rad", []))
            == 6,
            f"{pose_id}: A/B qpos contract mismatch",
        )
        new_q = [finite_number(value, f"{pose_id}.new.q") for value in new["joint_position_rad"]]
        old_q = [finite_number(value, f"{pose_id}.old.q") for value in old["joint_position_rad"]]
        require(new_q == old_q, f"{pose_id}: A/B qpos differs")
        if pose_id == "v15_14_accepted_fjt_final_pose":
            require(
                tuple(new_q) == V15_17C_FJT_POSITION_RAD
                and new.get("source_sha256")
                == old.get("source_sha256")
                == EXPECTED_V15_17C_DIAGNOSTIC_INPUT_SHA256[
                    V15_17C_FJT_EVIDENCE_REL
                ],
                "V15.17C nonzero pose is not the hash-bound accepted FJT pose",
            )
        visual_position, visual_rotation = pose_delta(
            old.get("visual_runtime_world_pose", {}),
            new.get("visual_runtime_world_pose", {}),
            f"{pose_id}.visual",
        )
        visual_position_component = position_max_abs_delta(
            old.get("visual_runtime_world_pose", {}),
            new.get("visual_runtime_world_pose", {}),
            f"{pose_id}.visual",
        )
        body_position, body_rotation = pose_delta(
            old.get("link6_body_runtime_world_pose", {}),
            new.get("link6_body_runtime_world_pose", {}),
            f"{pose_id}.link6_body",
        )
        old_joints = old.get("joints_world")
        new_joints = new.get("joints_world")
        require(isinstance(old_joints, dict) and isinstance(new_joints, dict) and set(old_joints) == set(new_joints) == set(JOINTS), f"{pose_id}: joint evidence mismatch")
        pose_joint_anchor = 0.0
        pose_joint_axis = 0.0
        for name in JOINTS:
            pose_joint_anchor = max(
                pose_joint_anchor,
                vector_error(
                    vector3(old_joints[name].get("anchor_m"), f"{pose_id}.{name}.old.anchor"),
                    vector3(new_joints[name].get("anchor_m"), f"{pose_id}.{name}.new.anchor"),
                ),
            )
            pose_joint_axis = max(
                pose_joint_axis,
                vector_error(
                    vector3(old_joints[name].get("axis"), f"{pose_id}.{name}.old.axis"),
                    vector3(new_joints[name].get("axis"), f"{pose_id}.{name}.new.axis"),
                ),
            )
        tf_pairs: list[tuple[str, Mapping[str, Any], Mapping[str, Any]]] = [
            ("tcp", old.get("tcp", {}), new.get("tcp", {}))
        ]
        old_cameras = old.get("camera_bodies")
        new_cameras = new.get("camera_bodies")
        require(isinstance(old_cameras, dict) and isinstance(new_cameras, dict) and set(old_cameras) == set(new_cameras), f"{pose_id}: camera TF evidence mismatch")
        tf_pairs.extend(
            (f"camera.{name}", old_cameras[name], new_cameras[name])
            for name in sorted(old_cameras)
        )
        pose_tf_position = 0.0
        pose_tf_rotation = 0.0
        for name, old_tf, new_tf in tf_pairs:
            position, rotation = pose_delta(old_tf, new_tf, f"{pose_id}.{name}")
            pose_tf_position = max(pose_tf_position, position)
            pose_tf_rotation = max(pose_tf_rotation, rotation)
        old_collision = old.get("collision_geom_runtime_world_poses")
        new_collision = new.get("collision_geom_runtime_world_poses")
        require(isinstance(old_collision, dict) and isinstance(new_collision, dict) and set(old_collision) == set(new_collision), f"{pose_id}: collision geom identity mismatch")
        pose_collision_position = 0.0
        pose_collision_rotation = 0.0
        pose_collision_exact = 0
        for name in sorted(old_collision):
            position, rotation = pose_delta(
                old_collision[name], new_collision[name], f"{pose_id}.collision.{name}"
            )
            pose_collision_position = max(pose_collision_position, position)
            pose_collision_rotation = max(pose_collision_rotation, rotation)
            collision_comparison_count += 1
            if position == 0.0 and rotation == 0.0:
                pose_collision_exact += 1
                collision_exact_count += 1
        old_collision_tagged = old.get(
            "all_collision_tagged_geom_runtime_world_poses"
        )
        new_collision_tagged = new.get(
            "all_collision_tagged_geom_runtime_world_poses"
        )
        require(
            isinstance(old_collision_tagged, dict)
            and isinstance(new_collision_tagged, dict)
            and set(old_collision_tagged) == set(new_collision_tagged)
            and len(old_collision_tagged) == 963,
            f"{pose_id}: collision-tagged geom identity mismatch",
        )
        pose_collision_tagged_exact = 0
        pose_collision_tagged_position = 0.0
        pose_collision_tagged_rotation = 0.0
        for name in sorted(old_collision_tagged):
            position, rotation = pose_delta(
                old_collision_tagged[name],
                new_collision_tagged[name],
                f"{pose_id}.collision_tagged.{name}",
            )
            pose_collision_tagged_position = max(
                pose_collision_tagged_position, position
            )
            pose_collision_tagged_rotation = max(
                pose_collision_tagged_rotation, rotation
            )
            collision_tagged_comparison_count += 1
            if position == 0.0 and rotation == 0.0:
                pose_collision_tagged_exact += 1
                collision_tagged_exact_count += 1
        old_non_target = old.get("non_target_named_geom_runtime_world_poses")
        new_non_target = new.get("non_target_named_geom_runtime_world_poses")
        require(
            isinstance(old_non_target, dict)
            and isinstance(new_non_target, dict)
            and set(old_non_target) == set(new_non_target)
            and len(old_non_target) == 1008,
            f"{pose_id}: non-target named geom identity mismatch",
        )
        pose_non_target_exact = 0
        pose_non_target_position = 0.0
        pose_non_target_rotation = 0.0
        for name in sorted(old_non_target):
            position, rotation = pose_delta(
                old_non_target[name],
                new_non_target[name],
                f"{pose_id}.non_target_geom.{name}",
            )
            pose_non_target_position = max(pose_non_target_position, position)
            pose_non_target_rotation = max(pose_non_target_rotation, rotation)
            non_target_geom_comparison_count += 1
            if position == 0.0 and rotation == 0.0:
                pose_non_target_exact += 1
                non_target_geom_exact_count += 1
        pose_rows.append(
            {
                "id": pose_id,
                "joint_position_rad": new_q,
                "visual_runtime_position_delta_m": visual_position,
                "visual_runtime_position_max_abs_component_delta_m": visual_position_component,
                "visual_runtime_rotation_delta_rad": visual_rotation,
                "link6_body_runtime_position_delta_m": body_position,
                "link6_body_runtime_rotation_delta_rad": body_rotation,
                "joint_anchor_max_position_delta_m": pose_joint_anchor,
                "joint_axis_max_vector_delta": pose_joint_axis,
                "tf_max_position_delta_m": pose_tf_position,
                "tf_max_rotation_delta_rad": pose_tf_rotation,
                "collision_geom_count": len(old_collision),
                "collision_geom_exact_count": pose_collision_exact,
                "collision_geom_max_position_delta_m": pose_collision_position,
                "collision_geom_max_rotation_delta_rad": pose_collision_rotation,
                "collision_tagged_geom_count": len(old_collision_tagged),
                "collision_tagged_geom_exact_count": pose_collision_tagged_exact,
                "collision_tagged_geom_max_position_delta_m": pose_collision_tagged_position,
                "collision_tagged_geom_max_rotation_delta_rad": pose_collision_tagged_rotation,
                "non_target_named_geom_count": len(old_non_target),
                "non_target_named_geom_exact_count": pose_non_target_exact,
                "non_target_named_geom_max_position_delta_m": pose_non_target_position,
                "non_target_named_geom_max_rotation_delta_rad": pose_non_target_rotation,
            }
        )
        max_visual_position = max(max_visual_position, visual_position)
        max_visual_position_component = max(
            max_visual_position_component, visual_position_component
        )
        max_visual_rotation = max(max_visual_rotation, visual_rotation)
        max_body_position = max(max_body_position, body_position)
        max_body_rotation = max(max_body_rotation, body_rotation)
        max_joint_anchor = max(max_joint_anchor, pose_joint_anchor)
        max_joint_axis = max(max_joint_axis, pose_joint_axis)
        max_tf_position = max(max_tf_position, pose_tf_position)
        max_tf_rotation = max(max_tf_rotation, pose_tf_rotation)
        max_collision_position = max(max_collision_position, pose_collision_position)
        max_collision_rotation = max(max_collision_rotation, pose_collision_rotation)
        max_collision_tagged_position = max(
            max_collision_tagged_position, pose_collision_tagged_position
        )
        max_collision_tagged_rotation = max(
            max_collision_tagged_rotation, pose_collision_tagged_rotation
        )
        max_non_target_geom_position = max(
            max_non_target_geom_position, pose_non_target_position
        )
        max_non_target_geom_rotation = max(
            max_non_target_geom_rotation, pose_non_target_rotation
        )

    body_joint_collision_exact = (
        max_body_position < 1.0e-12
        and max_body_rotation < 1.0e-12
        and max_joint_anchor < 1.0e-12
        and max_joint_axis < 1.0e-12
        and max_tf_position < 1.0e-12
        and max_tf_rotation < 1.0e-12
        and max_collision_position < 1.0e-12
        and max_collision_rotation < 1.0e-12
        and max_non_target_geom_position < 1.0e-12
        and max_non_target_geom_rotation < 1.0e-12
    )
    if (
        body_joint_collision_exact
        and max_visual_position < 1.0e-9
        and max_visual_rotation < 1.0e-10
    ):
        case = "CASE_1"
        classification = "MUJOCO_COMPILED_SAMEFRAME_REPRESENTATION_ARTIFACT"
        acceptance = "RESOLVED"
        visual_only_accepted = True
        final_status = "V15.17 INERTIAL_DEPLOYMENT = PASS"
        hard_unresolved: list[str] = []
    elif (
        body_joint_collision_exact
        and max_visual_position <= 1.0e-6
        and max_visual_rotation <= 1.0e-8
    ):
        case = "CASE_2"
        classification = "VISUAL_ONLY_SUBMICRON_COMPILER_ARTIFACT"
        acceptance = "ENGINEERING_ACCEPTED_VISUAL_SUBMICRON_ARTIFACT"
        visual_only_accepted = True
        final_status = "V15.17 INERTIAL_DEPLOYMENT = PASS_WITH_NONPHYSICAL_VISUAL_LIMITATION"
        hard_unresolved = []
    else:
        case = "CASE_3"
        classification = "RUNTIME_VISUAL_OR_PHYSICAL_POSE_REGRESSION"
        acceptance = "NOT_ACCEPTED"
        visual_only_accepted = False
        final_status = "V15.17 INERTIAL_DEPLOYMENT = FAIL"
        hard_unresolved = []
        if max_visual_position > 1.0e-6 or max_visual_rotation > 1.0e-8:
            hard_unresolved.append("VISUAL_RUNTIME_WORLD_POSE_EXCEEDS_ENGINEERING_LIMIT")
        if max_body_position >= 1.0e-12 or max_body_rotation >= 1.0e-12:
            hard_unresolved.append("LINK6_BODY_RUNTIME_POSE_CHANGED")
        if max_joint_anchor >= 1.0e-12 or max_joint_axis >= 1.0e-12:
            hard_unresolved.append("JOINT_RUNTIME_WORLD_FRAME_CHANGED")
        if max_tf_position >= 1.0e-12 or max_tf_rotation >= 1.0e-12:
            hard_unresolved.append("TCP_OR_CAMERA_RUNTIME_TF_CHANGED")
        if max_collision_position >= 1.0e-12 or max_collision_rotation >= 1.0e-12:
            hard_unresolved.append("COLLISION_GEOM_RUNTIME_WORLD_POSE_CHANGED")
        if (
            max_non_target_geom_position >= 1.0e-12
            or max_non_target_geom_rotation >= 1.0e-12
        ):
            hard_unresolved.append("NON_TARGET_NAMED_GEOM_RUNTIME_WORLD_POSE_CHANGED")
        if not hard_unresolved:
            hard_unresolved.append("V15_17C_CASE_CLASSIFICATION_UNRESOLVED")

    return {
        "pass": case in ("CASE_1", "CASE_2"),
        "case": case,
        "artifact_classification": classification,
        "engineering_acceptance": acceptance,
        "non_physical_visual_numerical_limitation": case == "CASE_2",
        "visual_only_limitation_accepted": visual_only_accepted,
        "final_status_if_all_other_gates_pass": final_status,
        "hard_unresolved_items": hard_unresolved,
        "pose_count": len(pose_rows),
        "pose_order": list(expected_pose_order),
        "poses": pose_rows,
        "compiled_storage_diagnostic": {
            "old_geom_sameframe": legacy_storage.get("geom_sameframe"),
            "new_geom_sameframe": explicit_storage.get("geom_sameframe"),
            "compiled_geom_pos_delta_m": compiled_geom_pos_delta,
            "compiled_geom_rotation_delta_rad": compiled_geom_rotation_delta,
            "old_geom_pos_m": legacy_storage.get("geom_pos_m"),
            "new_geom_pos_m": explicit_storage.get("geom_pos_m"),
            "old_geom_quat_wxyz": legacy_storage.get("geom_quat_wxyz"),
            "new_geom_quat_wxyz": explicit_storage.get("geom_quat_wxyz"),
            "old_link6_body_ipos_m": legacy_storage.get("body_ipos_m"),
            "new_link6_body_ipos_m": explicit_storage.get("body_ipos_m"),
            "old_link6_body_iquat_wxyz": legacy_storage.get("body_iquat_wxyz"),
            "new_link6_body_iquat_wxyz": explicit_storage.get("body_iquat_wxyz"),
            "compiled_body_ipos_delta_m": body_ipos_delta,
            "compiled_body_iquat_rotation_delta_rad": body_iquat_rotation_delta,
            "not_used_as_runtime_world_pose": True,
        },
        "runtime_maxima": {
            "visual_position_delta_m": max_visual_position,
            "visual_position_max_abs_component_delta_m": max_visual_position_component,
            "visual_rotation_delta_rad": max_visual_rotation,
            "link6_body_position_delta_m": max_body_position,
            "link6_body_rotation_delta_rad": max_body_rotation,
            "joint_anchor_position_delta_m": max_joint_anchor,
            "joint_axis_vector_delta": max_joint_axis,
            "tf_position_delta_m": max_tf_position,
            "tf_rotation_delta_rad": max_tf_rotation,
            "collision_geom_position_delta_m": max_collision_position,
            "collision_geom_rotation_delta_rad": max_collision_rotation,
            "collision_tagged_geom_position_delta_m": max_collision_tagged_position,
            "collision_tagged_geom_rotation_delta_rad": max_collision_tagged_rotation,
            "non_target_named_geom_position_delta_m": max_non_target_geom_position,
            "non_target_named_geom_rotation_delta_rad": max_non_target_geom_rotation,
        },
        "collision_runtime_evidence": {
            "set_definition": "COLLISION_ACTIVE_GEOMS=(CONTYPE_NONZERO_OR_CONAFFINITY_NONZERO), INCLUDING_GROUND",
            "per_pose_count": collision_comparison_count // len(pose_rows),
            "comparison_count": collision_comparison_count,
            "exact_count": collision_exact_count,
            "all_exact": collision_exact_count == collision_comparison_count,
            "inactive_collision_tagged_but_noncollidable_names": explicit.get(
                "inactive_collision_tagged_geom_names"
            ),
            "all_collision_tagged_diagnostic": {
                "per_pose_count": collision_tagged_comparison_count
                // len(pose_rows),
                "comparison_count": collision_tagged_comparison_count,
                "exact_count": collision_tagged_exact_count,
                "all_exact": collision_tagged_exact_count
                == collision_tagged_comparison_count,
                "maximum_position_delta_m": max_collision_tagged_position,
                "maximum_rotation_delta_rad": max_collision_tagged_rotation,
            },
        },
        "non_target_named_geom_runtime_evidence": {
            "target_excluded": "visual__link6__001",
            "per_pose_count": non_target_geom_comparison_count // len(pose_rows),
            "comparison_count": non_target_geom_comparison_count,
            "exact_count": non_target_geom_exact_count,
            "all_exact": non_target_geom_exact_count
            == non_target_geom_comparison_count,
            "maximum_position_delta_m": max_non_target_geom_position,
            "maximum_rotation_delta_rad": max_non_target_geom_rotation,
        },
        "body_joint_tf_collision_exact": body_joint_collision_exact,
        "visual_bbox": explicit_bbox,
        "visual_delta_scale": {
            "maximum_runtime_position_delta_m": max_visual_position,
            "maximum_runtime_position_delta_micrometre": max_visual_position * 1.0e6,
            "relative_to_bbox_diagonal": max_visual_position / bbox_diagonal,
            "relative_to_bbox_maximum_dimension": max_visual_position / bbox_maximum,
            "parts_per_million_of_bbox_diagonal": max_visual_position / bbox_diagonal * 1.0e6,
            "maximum_runtime_position_max_abs_component_delta_m": max_visual_position_component,
            "maximum_runtime_position_max_abs_component_delta_micrometre": max_visual_position_component
            * 1.0e6,
            "max_abs_component_relative_to_bbox_diagonal": max_visual_position_component
            / bbox_diagonal,
            "max_abs_component_relative_to_bbox_maximum_dimension": max_visual_position_component
            / bbox_maximum,
            "max_abs_component_parts_per_million_of_bbox_diagonal": max_visual_position_component
            / bbox_diagonal
            * 1.0e6,
            "acceptance_metric": "EUCLIDEAN_POSITION_NORM; MAX_ABS_COMPONENT_REPORTED_FOR_0P304_MICROMETRE_LEGACY_COMPARABILITY",
        },
        "limits": {
            "case1_visual_position_strict_lt_m": 1.0e-9,
            "case1_visual_rotation_strict_lt_rad": 1.0e-10,
            "case2_visual_position_less_than_or_equal_m": 1.0e-6,
            "case2_visual_rotation_less_than_or_equal_rad": 1.0e-8,
            "body_position_strict_lt_m": 1.0e-12,
            "body_rotation_strict_lt_rad": 1.0e-12,
            "joint_tf_collision_strict_lt": 1.0e-12,
        },
    }


def validate_mjcf_v15_17b(
    authority: Mapping[str, Mapping[str, Any]],
    explicit_python: str | None,
    wsl_distro: str | None,
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    """Compile B and an in-memory A, independently rebuilding every hard gate."""

    interpreter, probes = select_python(
        explicit_python,
        ("numpy", "mujoco"),
        (),
    )
    wsl_python = os.environ.get("V15_17_WSL_PYTHON", "python3")
    use_wsl = False
    if interpreter is None and wsl_distro:
        wsl_probe = probe_wsl_python(
            wsl_distro,
            wsl_python,
            ("numpy", "mujoco"),
            source_ros=False,
        )
        probes.append(wsl_probe)
        if wsl_probe.get("returncode") == 0:
            use_wsl = True
            interpreter = f"wsl:{wsl_distro}:{wsl_python}"
    require(interpreter is not None, "no Python interpreter provides numpy and mujoco")

    current_xml = repo_path(MJCF_REL).read_bytes()
    legacy_xml = legacy_fullinertia_model_bytes(current_xml, authority)
    source_xml_report = validate_source_xml_and_frozen_semantics(authority)
    source_rows = source_xml_report["source_mjcf_inertials"]
    with tempfile.TemporaryDirectory(prefix="v1517b_mjcf_ascii_") as raw:
        temp_root = Path(raw)
        require(str(temp_root).isascii(), f"MuJoCo temporary path is not ASCII: {temp_root}")
        current_dir = temp_root / "current"
        legacy_dir = temp_root / "legacy"
        current_dir.mkdir()
        legacy_dir.mkdir()
        source_directory = repo_path(MJCF_REL).parent
        current_model = make_ascii_mjcf_mirror(current_xml, source_directory, current_dir)
        legacy_model = make_ascii_mjcf_mirror(legacy_xml, source_directory, legacy_dir)
        contract = temp_root / "contract.json"
        targets = temp_root / "targets.json"
        collision_contract = read_json(repo_path(COLLISION_CONTRACT_REL))
        collision_contract["v15_17b_frozen_tensors_kg_m2"] = {
            name: authority[name]["tensor_kg_m2"] for name in LINKS
        }
        contract.write_text(
            json.dumps(collision_contract, ensure_ascii=False, sort_keys=True),
            encoding="utf-8",
        )
        write_v15_17c_child_manifest(targets)
        current = run_mujoco_child(
            str(interpreter),
            current_model,
            contract,
            targets,
            temp_root / "current.json",
            wsl_distro=wsl_distro if use_wsl else None,
            wsl_python=wsl_python if use_wsl else None,
        )
        legacy = run_mujoco_child(
            str(interpreter),
            legacy_model,
            contract,
            targets,
            temp_root / "legacy.json",
            wsl_distro=wsl_distro if use_wsl else None,
            wsl_python=wsl_python if use_wsl else None,
        )

    sameframe_runtime = compare_v15_17c_sameframe_runtime(current, legacy)

    rows: dict[str, Any] = {}
    maximum_python_error = 0.0
    maximum_input_rotation_error = 0.0
    maximum_compiled_rotation_error = 0.0
    maximum_tensor_error = 0.0
    independent_frames = current.get("independent_principal_frames")
    require(isinstance(independent_frames, dict), "independent principal-frame child evidence missing")
    for name in LINKS:
        expected = authority[name]
        source = source_rows[name]
        compiled = current.get("bodies", {}).get(name)
        frame = independent_frames.get(name)
        require(isinstance(compiled, dict), f"compiled B body missing: {name}")
        require(isinstance(frame, dict), f"independent eigh evidence missing: {name}")
        mass = finite_number(compiled.get("mass_kg"), f"{name}.compiled.mass")
        com = vector3(compiled.get("body_ipos_m"), f"{name}.compiled.com")
        compiled_q = vector3plus1(compiled.get("body_iquat_wxyz"), f"{name}.compiled.quat")
        compiled_moments = vector3(
            compiled.get("body_inertia_principal_kg_m2"), f"{name}.compiled.diaginertia"
        )
        compiled_rotation = matrix3(
            compiled.get("rotation_mju_quat2Mat_row_major"), f"{name}.compiled.rotation"
        )
        manual_compiled_rotation = quaternion_wxyz_to_matrix(compiled_q)
        child_manual_error = max_abs(
            item
            for row in matrix_delta(manual_compiled_rotation, compiled_rotation)
            for item in row
        )
        input_rotation = matrix3(
            source.get("input_rotation_body_from_inertial"), f"{name}.source.input_rotation"
        )
        compiled_rotation_error = max_abs(
            item for row in matrix_delta(compiled_rotation, input_rotation) for item in row
        )
        reconstructed = matmul(
            matmul(compiled_rotation, diagonal_matrix(compiled_moments)),
            transpose(compiled_rotation),
        )
        tensor_error = relative_frobenius(reconstructed, expected["tensor_kg_m2"])
        mass_error = abs(mass - float(expected["mass_kg"]))
        com_error = vector_error(com, expected["com_xyz_m"])

        eigh_moments = vector3(
            frame.get("eigh_principal_moments_kg_m2"), f"{name}.independent_eigh.moments"
        )
        eigh_axes = matrix3(
            frame.get("eigh_axes_body_from_inertial_columns"), f"{name}.independent_eigh.axes"
        )
        independent_reconstructed = matmul(
            matmul(eigh_axes, diagonal_matrix(eigh_moments)), transpose(eigh_axes)
        )
        python_error = relative_frobenius(
            independent_reconstructed, expected["tensor_kg_m2"]
        )
        source_tensor_error = relative_frobenius(
            frame.get("source_tensor_kg_m2"), expected["tensor_kg_m2"]
        )
        input_rotation_error = max(
            finite_number(
                frame.get("mju_mat2Quat_roundtrip_rotation_max_abs_error"),
                f"{name}.mju_mat2Quat_roundtrip",
            ),
            finite_number(
                frame.get("manual_roundtrip_rotation_max_abs_error"),
                f"{name}.manual_quaternion_roundtrip",
            ),
            finite_number(
                frame.get("manual_vs_mju_rotation_max_abs_error"),
                f"{name}.manual_vs_mju_rotation",
            ),
            finite_number(
                frame.get("source_input_rotation_vs_canonical_eigh_axes_max_abs_error"),
                f"{name}.source_input_rotation_vs_canonical_eigh_axes",
            ),
            finite_number(
                source.get("manual_quaternion_rotation_max_abs_error"),
                f"{name}.source_manual_quaternion_roundtrip",
            ),
        )
        mass_pass = mass_error < MASS_TOL_MJCF
        com_pass = com_error < COM_TOL_MJCF
        python_pass = (
            finite_number(frame.get("symmetry_max_abs_error"), f"{name}.symmetry")
            < 1.0e-12
            and finite_number(frame.get("determinant"), f"{name}.determinant") > 0.0
            and abs(finite_number(frame.get("determinant"), f"{name}.determinant") - 1.0)
            < ROTATION_MATRIX_TOL
            and finite_number(
                frame.get("orthonormality_max_abs_error"), f"{name}.orthonormality"
            )
            < ROTATION_MATRIX_TOL
            and python_error < PYTHON_EIGH_RECONSTRUCTION_TOL
            and source_tensor_error < PYTHON_EIGH_RECONSTRUCTION_TOL
            and all(value > 0.0 for value in eigh_moments)
            and eigh_moments[0] <= eigh_moments[1] <= eigh_moments[2]
        )
        input_quaternion_pass = input_rotation_error < ROTATION_MATRIX_TOL
        compiled_quaternion_pass = (
            compiled_rotation_error < ROTATION_MATRIX_TOL
            and child_manual_error < ROTATION_MATRIX_TOL
        )
        tensor_goal_pass = tensor_error < TENSOR_REL_GOAL_MJCF
        tensor_hard_pass = tensor_error < TENSOR_REL_TOL_MJCF
        row_pass = (
            mass_pass
            and com_pass
            and python_pass
            and input_quaternion_pass
            and compiled_quaternion_pass
            and tensor_hard_pass
        )
        rows[name] = {
            "frozen_tensor_kg_m2": expected["tensor_kg_m2"],
            "source_principal_moments_kg_m2": source["principal_moments_kg_m2"],
            "source_input_quaternion_wxyz": source["input_quaternion_wxyz"],
            "source_input_rotation_body_from_inertial": input_rotation,
            "independent_eigh": frame,
            "independently_reconstructed_tensor_kg_m2": independent_reconstructed,
            "python_eigh_reconstruction_relative_frobenius_error": python_error,
            "source_tensor_relative_frobenius_error": source_tensor_error,
            "input_quaternion_rotation_max_abs_error": input_rotation_error,
            "compiled": compiled,
            "compiled_rotation_manual_vs_mju_max_abs_error": child_manual_error,
            "compiled_vs_input_rotation_max_abs_error": compiled_rotation_error,
            "compiled_reconstructed_tensor_kg_m2": reconstructed,
            "compiled_tensor_relative_frobenius_error": tensor_error,
            "mass_error_kg": mass_error,
            "com_error_m": com_error,
            "mass_pass": mass_pass,
            "com_pass": com_pass,
            "python_eigh_pass": python_pass,
            "input_quaternion_pass": input_quaternion_pass,
            "compiled_quaternion_pass": compiled_quaternion_pass,
            "compiled_tensor_goal_strict_lt_1e_12_pass": tensor_goal_pass,
            "compiled_tensor_hard_strict_lt_1e_9_pass": tensor_hard_pass,
            "pass": row_pass,
        }
        maximum_python_error = max(maximum_python_error, python_error)
        maximum_input_rotation_error = max(maximum_input_rotation_error, input_rotation_error)
        maximum_compiled_rotation_error = max(
            maximum_compiled_rotation_error, compiled_rotation_error, child_manual_error
        )
        maximum_tensor_error = max(maximum_tensor_error, tensor_error)

    def recompute_synthetic(record: Mapping[str, Any], expected_tensor: Any) -> dict[str, Any]:
        quaternion = vector3plus1(record.get("body_iquat_wxyz"), "synthetic.quaternion")
        moments = vector3(record.get("body_inertia_principal_kg_m2"), "synthetic.moments")
        rotation = matrix3(record.get("rotation_mju_quat2Mat_row_major"), "synthetic.rotation")
        manual_rotation = quaternion_wxyz_to_matrix(quaternion)
        rotation_path_error = max_abs(
            item for row in matrix_delta(rotation, manual_rotation) for item in row
        )
        reconstructed = matmul(matmul(rotation, diagonal_matrix(moments)), transpose(rotation))
        relative = relative_frobenius(reconstructed, expected_tensor)
        return {
            "child_evidence": dict(record),
            "parent_independently_reconstructed_tensor_kg_m2": reconstructed,
            "parent_relative_frobenius_error_vs_frozen_link2": relative,
            "manual_vs_mju_rotation_max_abs_error": rotation_path_error,
        }

    frozen_link2 = authority["link2"]["tensor_kg_m2"]
    synthetic_a_child = legacy.get("synthetic_link2")
    synthetic_b_child = current.get("synthetic_link2")
    require(isinstance(synthetic_a_child, dict), "synthetic TEST A evidence missing")
    require(isinstance(synthetic_b_child, dict), "synthetic TEST B evidence missing")
    synthetic_a = recompute_synthetic(synthetic_a_child, frozen_link2)
    synthetic_b = recompute_synthetic(synthetic_b_child, frozen_link2)
    synthetic_a_error = synthetic_a["parent_relative_frobenius_error_vs_frozen_link2"]
    synthetic_b_error = synthetic_b["parent_relative_frobenius_error_vs_frozen_link2"]
    synthetic_a["regression_reproduced"] = (
        synthetic_a_error > TENSOR_REL_TOL_MJCF
        and abs(synthetic_a_error - 1.0258546895235342e-06) < 1.0e-12
    )
    synthetic_a["diagnostic_only_not_authority"] = True
    synthetic_b["required_strict_lt"] = 1.0e-12
    synthetic_b["hard_stop_strict_lt"] = 1.0e-9
    synthetic_b["pass"] = (
        synthetic_b_error < 1.0e-12
        and synthetic_b["manual_vs_mju_rotation_max_abs_error"] < ROTATION_MATRIX_TOL
    )

    current_zero = current.get("zero_gravity_off")
    legacy_zero = legacy.get("zero_gravity_off")
    require(isinstance(current_zero, dict) and isinstance(legacy_zero, dict), "zero-pose physics evidence missing")
    current_pose = current_zero.get("pose_snapshot")
    legacy_pose = legacy_zero.get("pose_snapshot")
    require(isinstance(current_pose, dict) and isinstance(legacy_pose, dict), "zero-pose snapshot missing")
    qpos_delta, qpos_shape = _zero_snapshot_delta(
        {"qpos": current_pose.get("qpos")}, {"qpos": legacy_pose.get("qpos")}
    )
    body_delta, body_shape = _zero_snapshot_delta(
        current_pose.get("bodies", {}), legacy_pose.get("bodies", {})
    )
    joint_delta, joint_shape = _zero_snapshot_delta(
        current_pose.get("joints", {}), legacy_pose.get("joints", {})
    )
    current_geoms = current_pose.get("geoms", {})
    legacy_geoms = legacy_pose.get("geoms", {})
    require(isinstance(current_geoms, dict) and isinstance(legacy_geoms, dict), "geom snapshots missing")
    current_geom_poses = {
        name: {"xpos_m": row.get("xpos_m"), "xmat_row_major": row.get("xmat_row_major")}
        for name, row in current_geoms.items()
        if isinstance(row, dict)
    }
    legacy_geom_poses = {
        name: {"xpos_m": row.get("xpos_m"), "xmat_row_major": row.get("xmat_row_major")}
        for name, row in legacy_geoms.items()
        if isinstance(row, dict)
    }
    all_geom_delta, all_geom_shape = _zero_snapshot_delta(
        current_geom_poses, legacy_geom_poses
    )
    current_collision_geoms = {
        key: value
        for key, value in current_geom_poses.items()
        if str(key).startswith("collision__")
    }
    legacy_collision_geoms = {
        key: value
        for key, value in legacy_geom_poses.items()
        if str(key).startswith("collision__")
    }
    collision_geom_delta, collision_geom_shape = _zero_snapshot_delta(
        current_collision_geoms, legacy_collision_geoms
    )
    geom_rows: list[dict[str, Any]] = []
    require(set(current_geoms) == set(legacy_geoms), "A/B named geom identity mismatch")
    for name in sorted(current_geoms):
        current_row = current_geoms[name]
        legacy_row = legacy_geoms[name]
        require(isinstance(current_row, dict) and isinstance(legacy_row, dict), f"geom row invalid: {name}")
        xpos_delta = max_abs(
            left - right
            for left, right in zip(
                vector3(current_row.get("xpos_m"), f"{name}.B.xpos"),
                vector3(legacy_row.get("xpos_m"), f"{name}.A.xpos"),
            )
        )
        current_xmat = [finite_number(value, f"{name}.B.xmat") for value in current_row.get("xmat_row_major", [])]
        legacy_xmat = [finite_number(value, f"{name}.A.xmat") for value in legacy_row.get("xmat_row_major", [])]
        require(len(current_xmat) == len(legacy_xmat) == 9, f"geom xmat shape invalid: {name}")
        xmat_delta = max_abs(left - right for left, right in zip(current_xmat, legacy_xmat))
        geom_rows.append(
            {
                "geom": name,
                "xpos_max_abs_delta_m": xpos_delta,
                "xmat_max_abs_delta": xmat_delta,
                "legacy_fullinertia_geom_sameframe": legacy_row.get("compiled_geom_sameframe"),
                "explicit_principal_geom_sameframe": current_row.get("compiled_geom_sameframe"),
                "collision_geom": str(name).startswith("collision__"),
                "pose_pass_strict_lt_1e_12": max(xpos_delta, xmat_delta) < KINEMATICS_ABS_TOL,
            }
        )
    mismatched_geoms = [row for row in geom_rows if not row["pose_pass_strict_lt_1e_12"]]
    maximum_geom_row = max(
        geom_rows,
        key=lambda row: max(float(row["xpos_max_abs_delta_m"]), float(row["xmat_max_abs_delta"])),
    )
    link6_sameframe_artifact = (
        len(mismatched_geoms) == 1
        and mismatched_geoms[0]["geom"] == "visual__link6__001"
        and abs(float(mismatched_geoms[0]["xpos_max_abs_delta_m"]) - 3.037324224908211e-07)
        < 1.0e-15
        and float(mismatched_geoms[0]["xmat_max_abs_delta"]) == 0.0
        and mismatched_geoms[0]["legacy_fullinertia_geom_sameframe"] == 2
        and mismatched_geoms[0]["explicit_principal_geom_sameframe"] == 0
        and collision_geom_delta == 0.0
    )
    current_mass_matrix = matrix_or_nested = current_zero.get("generalized_mass_matrix")
    legacy_mass_matrix = legacy_zero.get("generalized_mass_matrix")
    require(isinstance(matrix_or_nested, list) and isinstance(legacy_mass_matrix, list), "mass-matrix evidence missing")
    mass_left = list(flatten_numeric(matrix_or_nested))
    mass_right = list(flatten_numeric(legacy_mass_matrix))
    require(len(mass_left) == len(mass_right) and mass_left, "mass-matrix shape mismatch")
    mass_delta = [left - right for left, right in zip(mass_left, mass_right)]
    mass_denominator = math.sqrt(sum(value * value for value in mass_right))
    generalized = {
        "mode": "ZERO_QPOS_GRAVITY_OFF_A_FULLINERTIA_VS_B_EXPLICIT_PRINCIPAL",
        "old_fullinertia_is_diagnostic_not_authority": True,
        "qpos_max_abs_delta": qpos_delta,
        "body_pose_max_abs_delta": body_delta,
        "joint_anchor_axis_max_abs_delta": joint_delta,
        "all_named_geom_pose_max_abs_delta": all_geom_delta,
        "collision_geom_pose_max_abs_delta": collision_geom_delta,
        "per_named_geom_pose_deltas": geom_rows,
        "mismatched_named_geoms": mismatched_geoms,
        "maximum_named_geom_pose_delta_record": maximum_geom_row,
        "link6_legacy_fullinertia_geom_sameframe_artifact_independently_classified": link6_sameframe_artifact,
        "shape_identity": {
            "qpos": qpos_shape,
            "bodies": body_shape,
            "joints": joint_shape,
            "all_named_geoms": all_geom_shape,
            "collision_geoms": collision_geom_shape,
        },
        "generalized_mass_matrix_max_abs_delta": max_abs(mass_delta),
        "generalized_mass_matrix_relative_frobenius_delta": (
            math.sqrt(sum(value * value for value in mass_delta)) / mass_denominator
            if mass_denominator > 0.0
            else None
        ),
        "mass_matrix_difference_is_diagnostic_only": True,
        "all_geometry_pose_exact_gate_required_by_attachment": (
            qpos_shape
            and body_shape
            and joint_shape
            and all_geom_shape
            and qpos_delta < KINEMATICS_ABS_TOL
            and body_delta < KINEMATICS_ABS_TOL
            and joint_delta < KINEMATICS_ABS_TOL
            and all_geom_delta < KINEMATICS_ABS_TOL
        ),
        "collision_only_pose_pass": (
            collision_geom_shape and collision_geom_delta < KINEMATICS_ABS_TOL
        ),
    }

    current_targets = {str(row["id"]): row for row in current.get("targets", [])}
    legacy_targets = {str(row["id"]): row for row in legacy.get("targets", [])}
    require(set(current_targets) == set(legacy_targets) and len(current_targets) == 12, "A/B target set mismatch")
    regression_rows: list[dict[str, Any]] = []
    maximum_target_delta = 0.0
    for target_id in sorted(current_targets):
        current_view = target_semantic_view(current_targets[target_id])
        legacy_view = target_semantic_view(legacy_targets[target_id])
        semantic_equal = (
            current_view["id"] == legacy_view["id"]
            and current_view["expected_outcome"] == legacy_view["expected_outcome"]
            and current_view["actual_outcome"] == legacy_view["actual_outcome"]
            and current_view["contact_pairs"] == legacy_view["contact_pairs"]
        )
        numeric_delta = semantic_numeric_error(current_view, legacy_view)
        maximum_target_delta = max(maximum_target_delta, numeric_delta)
        passed = (
            current_targets[target_id].get("outcome_pass") is True
            and semantic_equal
            and numeric_delta < KINEMATICS_ABS_TOL
        )
        regression_rows.append(
            {
                "id": target_id,
                "semantic_outcome_unchanged": semantic_equal,
                "max_numeric_delta": numeric_delta,
                "pass": passed,
            }
        )
    collision_static = validate_collision_static()
    collision_report = {
        "pass": collision_static.get("pass") is True
        and all(row["pass"] for row in regression_rows)
        and generalized["collision_only_pose_pass"],
        "static_collision_layer_revalidation": collision_static,
        "collision_geom_zero_pose_equivalence": {
            "max_abs_delta": collision_geom_delta,
            "pass": generalized["collision_only_pose_pass"],
        },
        "targets": regression_rows,
    }
    kinematics_report = {
        "pass": all(row["pass"] for row in regression_rows)
        and sameframe_runtime["body_joint_tf_collision_exact"],
        "mode": "A_B_COMPILED_FOUR_POSE_BODY_JOINT_TF_AND_12_TARGET_EQUIVALENCE",
        "target_count": len(regression_rows),
        "max_target_numeric_delta": maximum_target_delta,
        "generalized_physics_equivalence": generalized,
        "joint_origins_unchanged": joint_shape and joint_delta < KINEMATICS_ABS_TOL,
        "joint_axes_unchanged": joint_shape and joint_delta < KINEMATICS_ABS_TOL,
        "tcp_unchanged": body_shape and body_delta < KINEMATICS_ABS_TOL,
        "camera_tf_unchanged": body_shape and body_delta < KINEMATICS_ABS_TOL,
        "all_geom_poses_unchanged": all_geom_shape and all_geom_delta < KINEMATICS_ABS_TOL,
        "collision_geom_poses_unchanged": generalized["collision_only_pose_pass"],
        "link6_legacy_fullinertia_geom_sameframe_artifact_independently_classified": link6_sameframe_artifact,
        "v15_17c_four_pose_runtime_diagnostic_pass": sameframe_runtime["pass"],
        "v15_17c_body_joint_tf_collision_exact": sameframe_runtime[
            "body_joint_tf_collision_exact"
        ],
        "v15_17c_all_visual_runtime_poses_exact": sameframe_runtime["case"]
        == "CASE_1",
    }
    all_mass_pass = all(rows[name]["mass_pass"] for name in LINKS)
    all_com_pass = all(rows[name]["com_pass"] for name in LINKS)
    all_hard_tensor_pass = all(
        rows[name]["compiled_tensor_hard_strict_lt_1e_9_pass"] for name in LINKS
    )
    all_python_pass = all(rows[name]["python_eigh_pass"] for name in LINKS)
    all_quaternion_pass = all(
        rows[name]["input_quaternion_pass"] and rows[name]["compiled_quaternion_pass"]
        for name in LINKS
    )
    readback_pass = (
        synthetic_b["pass"]
        and all_mass_pass
        and all_com_pass
        and all_hard_tensor_pass
        and all_python_pass
        and all_quaternion_pass
    )
    report = {
        "pass": readback_pass,
        "compile_pass": True,
        "numeric_runtime": {
            "python_version": current.get("python_version"),
            "mujoco_version": current.get("mujoco_version"),
            "numpy_and_readback_dtype": "float64",
            "interpreter_path_not_serialized": True,
            "probe_count": len(probes),
        },
        "representation": "EXPLICIT_PRINCIPAL_FRAME_DIAGINERTIA",
        "reason": "BYPASS_MUJOCO_3_11_0_FULLINERTIA_EIGEN_DECOMPOSITION_PRECISION",
        "synthetic_test_a_fullinertia": synthetic_a,
        "synthetic_test_b_explicit_principal": synthetic_b,
        "links": rows,
        "all_six_mass_pass": all_mass_pass,
        "all_six_com_pass": all_com_pass,
        "all_six_compiled_tensor_hard_pass": all_hard_tensor_pass,
        "all_six_python_eigh_pass": all_python_pass,
        "all_six_quaternion_paths_pass": all_quaternion_pass,
        "max_python_eigh_reconstruction_relative_frobenius_error": maximum_python_error,
        "max_input_quaternion_rotation_max_abs_error": maximum_input_rotation_error,
        "max_compiled_quaternion_rotation_max_abs_error": maximum_compiled_rotation_error,
        "max_six_body_tensor_relative_frobenius_error": maximum_tensor_error,
        "gravity_enabled": max_abs(current.get("model", {}).get("gravity_m_s2", [])) != 0.0,
        "generalized_physics_diagnostic": generalized,
        "v15_17c_sameframe_runtime_diagnostic": sameframe_runtime,
        "tolerances": {
            "mass_strict_lt_kg": MASS_TOL_MJCF,
            "com_norm_strict_lt_m": COM_TOL_MJCF,
            "python_eigh_reconstruction_strict_lt": PYTHON_EIGH_RECONSTRUCTION_TOL,
            "quaternion_rotation_max_abs_strict_lt": ROTATION_MATRIX_TOL,
            "compiled_tensor_goal_strict_lt": TENSOR_REL_GOAL_MJCF,
            "compiled_tensor_hard_strict_lt": TENSOR_REL_TOL_MJCF,
        },
    }
    return report, collision_report, kinematics_report


def validate_runtime_evidence(
    evidence_path: Path,
    authority: Mapping[str, Mapping[str, Any]],
) -> tuple[dict[str, Any], dict[str, Any], dict[str, Any]]:
    """Verify repo-external ROS/trajectory/collision evidence and its logs.

    Boolean fields in the envelope are necessary but never sufficient: this
    function hashes and parses the referenced evidence files, binds them to the
    current Xacro/MJCF, and looks for the concrete ROS readiness/failure
    markers.  This prevents an old V15.14 log from being relabeled V15.17.
    """

    require(evidence_path.is_file(), f"runtime evidence JSON missing: {evidence_path}")
    evidence_initial_hash = sha256_file(evidence_path)
    evidence = read_json(evidence_path)
    require(evidence.get("schema") == RUNTIME_EVIDENCE_SCHEMA, "runtime evidence schema mismatch")
    require(evidence.get("source_commit") == SOURCE_COMMIT, "runtime evidence source commit mismatch")
    require(evidence.get("target_branch") == TARGET_BRANCH, "runtime evidence target branch mismatch")
    current_mjcf_hash = sha256_file(repo_path(MJCF_REL))
    current_xacro_hash = sha256_file(repo_path(XACRO_REL))
    baseline_mjcf_hash = sha256_bytes(git_blob(MJCF_REL))
    bridge_path = repo_path(
        V14_REL
        + "/ros2_ws/src/go_m8010_arm_mujoco_bridge/scripts/mujoco_bridge.py"
    )
    accepted_model_value = python_literal_assignment(bridge_path, "ACCEPTED_MODEL_SHA256")
    require(isinstance(accepted_model_value, str), "production bridge ACCEPTED_MODEL_SHA256 is not a string")
    accepted_model_sha256 = accepted_model_value.lower()
    require(
        accepted_model_sha256
        == "ce6ec828d32f445ff25fb4b3ba3167f7f0bec61cdfca5962ee4072df4b022844",
        "production bridge ACCEPTED_MODEL_SHA256 is not the frozen ce6e anchor",
    )
    cross_503_path = repo_path(CROSS_503_REL)
    expected_input_sha256 = python_literal_assignment(cross_503_path, "EXPECTED_INPUT_SHA256")
    reference_negative_controls = python_literal_assignment(
        cross_503_path, "REFERENCE_TERMINAL_COLLISION_INDICES"
    )
    require(isinstance(expected_input_sha256, dict), "503 EXPECTED_INPUT_SHA256 is not a literal map")
    require(
        expected_input_sha256.get("model") == accepted_model_sha256,
        "503 EXPECTED_INPUT_SHA256['model'] differs from production ce6e anchor",
    )
    require(
        reference_negative_controls
        == {"coupled_random_seed_1513": [43, 73, 75, 83, 87]},
        "503 terminal negative-control identity contract changed",
    )
    require(str(evidence.get("mjcf_sha256", "")).lower() == current_mjcf_hash, "runtime evidence MJCF hash mismatch")
    require(str(evidence.get("xacro_sha256", "")).lower() == current_xacro_hash, "runtime evidence Xacro hash mismatch")

    log_record = evidence.get("bringup_log")
    require(isinstance(log_record, dict), "runtime evidence bringup_log record missing")
    log_path = Path(str(log_record.get("path", "")))
    require(log_path.is_file(), f"bringup log missing: {log_path}")
    log_hash = sha256_file(log_path)
    require(str(log_record.get("sha256", "")).lower() == log_hash, "bringup log SHA256 mismatch")
    log_text_raw = log_path.read_text(encoding="utf-8", errors="replace")
    log_text = re.sub(
        r"\x1b\[[0-?]*[ -/]*[@-~]",
        "",
        log_text_raw,
    )

    rsp_markers = [
        "[robot_state_publisher]: got segment link2",
        "[robot_state_publisher]: got segment link6",
        "[robot_state_publisher]: got segment gripper",
        "[robot_state_publisher]: got segment tcp_nominal",
    ]
    control_markers = [
        "Successful 'activate' of hardware 'MujocoTopicSystem'",
        "Configured and activated joint_state_broadcaster",
        "Configured and activated arm_controller",
    ]
    moveit_markers = [
        "Loading robot model 'go_m8010_arm_v15_14'",
        "Loading planning pipeline 'ompl'",
        "MoveGroup context initialization complete",
        "You can start planning now!",
    ]
    require(all(marker in log_text for marker in rsp_markers), "robot_state_publisher runtime markers incomplete")
    require(all(marker in log_text for marker in control_markers), "ros2_control runtime markers incomplete")
    require(all(marker in log_text for marker in moveit_markers), "MoveIt runtime markers incomplete")

    mismatch = re.search(
        r"unaccepted V15\.14 MJCF hash:\s*([0-9a-fA-F]{64})\s*!=\s*([0-9a-fA-F]{64})",
        log_text,
    )
    require(mismatch is not None, "immutable bridge trust-anchor rejection marker missing")
    rejected_actual = mismatch.group(1).lower()
    accepted_anchor = mismatch.group(2).lower()
    require(rejected_actual == current_mjcf_hash, "bridge rejection did not bind current MJCF")
    require(accepted_anchor == accepted_model_sha256, "bridge rejection anchor differs from production ACCEPTED_MODEL_SHA256")

    check_urdf = evidence.get("check_urdf")
    rsp = evidence.get("robot_state_publisher")
    control = evidence.get("ros2_control")
    moveit = evidence.get("moveit")
    bridge = evidence.get("bridge")
    trajectory = evidence.get("trajectory_loop")
    for label, record in (
        ("check_urdf", check_urdf),
        ("robot_state_publisher", rsp),
        ("ros2_control", control),
        ("moveit", moveit),
        ("bridge", bridge),
        ("trajectory_loop", trajectory),
    ):
        require(isinstance(record, dict), f"runtime evidence record missing: {label}")
    require(check_urdf.get("pass") is True, "runtime check_urdf is not PASS")
    check_urdf_path = Path(str(check_urdf.get("path", "")))
    require(check_urdf_path.is_file(), f"runtime check_urdf log missing: {check_urdf_path}")
    check_urdf_hash = sha256_file(check_urdf_path)
    require(str(check_urdf.get("sha256", "")).lower() == check_urdf_hash, "runtime check_urdf log SHA256 mismatch")
    check_urdf_text = check_urdf_path.read_text(encoding="utf-8", errors="replace")
    success_marker = str(check_urdf.get("success_marker", ""))
    require(success_marker == "Successfully Parsed XML" and success_marker in check_urdf_text, "runtime check_urdf success marker missing")
    require(int(check_urdf.get("target_inertial_count", 0)) == 6, "runtime check_urdf target inertial count mismatch")
    require(rsp.get("pass") is True, "runtime robot_state_publisher is not PASS")
    require(control.get("pass") is True and control.get("hardware_active") is True, "runtime ros2_control hardware is not active")
    require(set(control.get("active_controllers", [])) == {"joint_state_broadcaster", "arm_controller"}, "runtime controller active set mismatch")
    controller_path = Path(str(control.get("path", "")))
    require(controller_path.is_file(), f"runtime controller-state evidence missing: {controller_path}")
    controller_hash = sha256_file(controller_path)
    require(str(control.get("sha256", "")).lower() == controller_hash, "runtime controller-state evidence SHA256 mismatch")
    controller_text = controller_path.read_text(encoding="utf-8", errors="replace")
    require(re.search(r"joint_state_broadcaster.*active", controller_text) is not None, "runtime joint_state_broadcaster active marker missing")
    require(re.search(r"arm_controller.*active", controller_text) is not None, "runtime arm_controller active marker missing")
    require(moveit.get("pass") is True and moveit.get("robot_model_loaded") is True and moveit.get("planning_pipeline_ready") is True, "runtime MoveIt model/pipeline is not ready")
    require(bridge.get("pass") is False, "production bridge unexpectedly marked PASS")
    require(str(bridge.get("actual_mjcf_sha256", "")).lower() == current_mjcf_hash, "bridge evidence actual hash mismatch")
    require(str(bridge.get("accepted_mjcf_sha256", "")).lower() == accepted_anchor, "bridge evidence trust-anchor hash mismatch")
    require(bridge.get("reason") == "IMMUTABLE_TRUST_ANCHOR_MISMATCH", "bridge evidence reason mismatch")
    require(trajectory.get("pass") is False, "production trajectory loop must not PASS when bridge is rejected")
    require(trajectory.get("reason") == "PRODUCTION_BRIDGE_IMMUTABLE_TRUST_ANCHOR_MISMATCH", "trajectory failure reason mismatch")

    binding_record = evidence.get("robot_description_binding")
    require(isinstance(binding_record, dict), "runtime robot_description binding record missing")
    binding_path = Path(str(binding_record.get("path", "")))
    require(binding_path.is_file(), f"runtime robot_description binding missing: {binding_path}")
    binding_hash = sha256_file(binding_path)
    require(str(binding_record.get("sha256", "")).lower() == binding_hash, "runtime robot_description binding SHA256 mismatch")
    binding = read_json(binding_path)
    binding_digest: str | None = None
    binding_rows: dict[str, Any] = {}
    for node_name in ("robot_state_publisher", "controller_manager", "move_group"):
        node = binding.get(node_name)
        require(isinstance(node, dict), f"robot_description binding node missing: {node_name}")
        digest = str(node.get("robot_description_target_inertials_sha256", "")).lower()
        require(len(digest) == 64, f"robot_description binding digest invalid: {node_name}")
        if binding_digest is None:
            binding_digest = digest
        require(digest == binding_digest, f"robot_description binding digest differs: {node_name}")
        require(int(node.get("target_inertial_count", 0)) == 6, f"robot_description binding count mismatch: {node_name}")
        values = node.get("values")
        require(isinstance(values, dict) and set(values) == set(LINKS), f"robot_description binding link set mismatch: {node_name}")
        recomputed_digest = sha256_bytes(
            json.dumps(values, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
        )
        require(recomputed_digest == digest, f"robot_description binding canonical digest mismatch: {node_name}")
        max_mass_error = 0.0
        max_com_error = 0.0
        max_tensor_error = 0.0
        for name in LINKS:
            actual = values[name]
            require(isinstance(actual, dict), f"robot_description inertial record invalid: {node_name}:{name}")
            mass_error = abs(float(actual["mass"]) - float(authority[name]["mass_kg"]))
            com_error = vector_error(
                parse_numbers(actual.get("xyz"), 3, f"runtime.{node_name}.{name}.xyz"),
                authority[name]["com_xyz_m"],
            )
            require(max_abs(parse_numbers(actual.get("rpy"), 3, f"runtime.{node_name}.{name}.rpy")) == 0.0, f"runtime robot_description inertial rpy changed: {node_name}:{name}")
            tensor_record = actual.get("inertia")
            require(isinstance(tensor_record, dict), f"runtime tensor missing: {node_name}:{name}")
            tensor = [
                [float(tensor_record["ixx"]), float(tensor_record["ixy"]), float(tensor_record["ixz"])],
                [float(tensor_record["ixy"]), float(tensor_record["iyy"]), float(tensor_record["iyz"])],
                [float(tensor_record["ixz"]), float(tensor_record["iyz"]), float(tensor_record["izz"])],
            ]
            expected_tensor = authority[name]["tensor_kg_m2"]
            tensor_error = max_abs(item for row in matrix_delta(tensor, expected_tensor) for item in row)
            require(mass_error < MASS_TOL_URDF and com_error < COM_TOL_URDF and tensor_error < TENSOR_TOL_URDF, f"runtime robot_description inertial mismatch: {node_name}:{name}")
            max_mass_error = max(max_mass_error, mass_error)
            max_com_error = max(max_com_error, com_error)
            max_tensor_error = max(max_tensor_error, tensor_error)
        binding_rows[node_name] = {
            "target_inertial_count": 6,
            "target_inertials_sha256": digest,
            "max_mass_error_kg": max_mass_error,
            "max_com_error_m": max_com_error,
            "max_tensor_abs_error_kg_m2": max_tensor_error,
            "pass": True,
        }

    collision_record = evidence.get("collision_regression")
    require(isinstance(collision_record, dict), "runtime collision regression record missing")
    collision_path = Path(str(collision_record.get("path", "")))
    require(collision_path.is_file(), f"runtime collision evidence missing: {collision_path}")
    collision_hash = sha256_file(collision_path)
    require(str(collision_record.get("sha256", "")).lower() == collision_hash, "runtime collision evidence SHA256 mismatch")
    collision_json = read_json(collision_path)
    pose_count = int(collision_json.get("total_pose_count", 0))
    require(collision_record.get("pass") is True, "runtime collision regression envelope is not PASS")
    require(pose_count == 503, f"runtime collision pose count mismatch: {pose_count}")
    # The raw V15.14 tool is deliberately unmodified: its overall status is
    # FAIL only because its immutable whole-model trust anchor is the old
    # ce6e... model.  That legacy mismatch is diagnostic here, while all 503
    # collision facts below are independently gated against the current model.
    require(collision_json.get("status") == "FAIL", "raw legacy 503 tool status must remain unmodified FAIL")
    require(str(collision_json.get("inputs", {}).get("model_sha256", "")).lower() == current_mjcf_hash, "runtime 503 report model SHA mismatch")
    input_gates = collision_json.get("input_hash_gates", {})
    require(input_gates.get("validator", {}).get("pass") is True, "runtime 503 validator hash gate failed")
    require(input_gates.get("guard", {}).get("pass") is True, "runtime 503 guard hash gate failed")
    require(input_gates.get("model", {}).get("pass") is False, "raw legacy model hash gate must be the sole intentional failure")
    require(str(input_gates.get("model", {}).get("actual_sha256", "")).lower() == current_mjcf_hash, "runtime 503 actual model hash gate mismatch")
    require(
        str(input_gates.get("model", {}).get("expected_sha256", "")).lower()
        == accepted_model_sha256,
        "runtime 503 expected model hash differs from production ACCEPTED_MODEL_SHA256",
    )
    require(collision_json.get("input_hashes_pass") is False, "raw legacy input hash aggregate must remain FAIL")
    require(collision_json.get("pose_count_gate_pass") is True, "runtime 503 pose count gate failed")
    payload_gate = collision_json.get("frozen_pose_set_gate", {})
    require(
        payload_gate.get("pass") is True
        and str(payload_gate.get("expected_sha256", "")).lower()
        == "4d9cfaf15cc20fe16393d1aa3bccddc5273e77e2de08cf9d0045976c301799aa"
        and str(payload_gate.get("actual_sha256", "")).lower()
        == "4d9cfaf15cc20fe16393d1aa3bccddc5273e77e2de08cf9d0045976c301799aa",
        "runtime collision frozen pose-set gate failed",
    )
    require(int(collision_json.get("match_count", -1)) == 503 and int(collision_json.get("mismatch_count", -1)) == 0, "runtime 503 match/mismatch gate failed")
    require(collision_json.get("all_collision_booleans_match") is True, "runtime 503 collision booleans mismatch")
    require(collision_json.get("all_collision_pair_sets_match") is True, "runtime 503 collision pair sets mismatch")
    require(collision_json.get("frozen_collision_counts_pass") is True, "runtime 503 frozen collision counts failed")
    expected_counts = collision_json.get("expected_frozen_collision_pose_counts")
    require(expected_counts == {"overall": 182, "self": 126, "ground": 118}, "runtime 503 frozen collision counts changed")
    require(int(collision_json.get("negative_control_count", 0)) == 5 and collision_json.get("negative_controls_pass") is True, "runtime 503 negative controls failed")
    controls_payload = collision_json.get("reference_terminal_collision_negative_controls")
    require(isinstance(controls_payload, list) and len(controls_payload) == 5 and all(row.get("pass") is True for row in controls_payload), "runtime 503 negative-control payload invalid")
    pose_audit = recompute_collision_503(
        collision_json,
        reference_negative_controls,
    )
    require(sha256_file(evidence_path) == evidence_initial_hash, "runtime evidence envelope changed during validation")
    require(sha256_file(log_path) == log_hash, "bringup log changed during validation")
    require(sha256_file(collision_path) == collision_hash, "collision evidence changed during validation")
    require(sha256_file(binding_path) == binding_hash, "runtime robot_description binding changed during validation")
    require(sha256_file(check_urdf_path) == check_urdf_hash, "runtime check_urdf log changed during validation")
    require(sha256_file(controller_path) == controller_hash, "runtime controller-state evidence changed during validation")

    runtime_report = {
        "pass": True,
        "schema": evidence.get("schema"),
        "evidence_json_path": str(evidence_path),
        "evidence_json_sha256": evidence_initial_hash,
        "bringup_log": {"path": str(log_path), "sha256": log_hash},
        "bound_mjcf_sha256": current_mjcf_hash,
        "bound_xacro_sha256": current_xacro_hash,
        "runtime_robot_description_binding": {
            "path": str(binding_path),
            "sha256": binding_hash,
            "shared_target_inertials_sha256": binding_digest,
            "nodes": binding_rows,
            "pass": True,
        },
        "check_urdf": {"pass": True, "mode": "REAL_ROS2_HUMBLE_RUNTIME_EVIDENCE"},
        "robot_state_publisher": {"pass": True, "mode": "REAL_ROS2_HUMBLE_RUNTIME", "verified_markers": rsp_markers},
        "ros2_control": {
            "pass": True,
            "mode": "REAL_ROS2_HUMBLE_RUNTIME",
            "hardware_active": True,
            "active_controllers": ["arm_controller", "joint_state_broadcaster"],
            "verified_markers": control_markers,
        },
        "moveit": {
            "pass": True,
            "mode": "REAL_MOVEIT2_RUNTIME",
            "robot_model_loaded": True,
            "planning_pipeline_ready": True,
            "verified_markers": moveit_markers,
        },
        "production_bridge": {
            "pass": False,
            "actual_mjcf_sha256": rejected_actual,
            "accepted_mjcf_sha256": accepted_anchor,
            "reason": "IMMUTABLE_TRUST_ANCHOR_MISMATCH",
            "failure_is_real_and_expected_to_block_trajectory": True,
        },
        "ros_runtime_executed": True,
        "ros_runtime_pass_claimed_only_for_verified_components": True,
    }
    trajectory_report = {
        "pass": False,
        "mode": "REAL_PRODUCTION_TRAJECTORY_LOOP",
        "runtime_executed": True,
        "reason": "PRODUCTION_BRIDGE_IMMUTABLE_TRUST_ANCHOR_MISMATCH",
        "actual_mjcf_sha256": rejected_actual,
        "accepted_mjcf_sha256": accepted_anchor,
        "hard_blocker": True,
    }
    collision_report = {
        "pass": True,
        "mode": "CURRENT_DEPLOYED_MJCF_RUNTIME_REGRESSION",
        "evidence_path": str(collision_path),
        "evidence_sha256": collision_hash,
        "pose_count": pose_count,
        "match_count": int(collision_json.get("match_count")),
        "mismatch_count": int(collision_json.get("mismatch_count")),
        "frozen_pose_set_sha256": payload_gate.get("actual_sha256"),
        "frozen_collision_pose_counts": expected_counts,
        "negative_control_count": int(collision_json.get("negative_control_count")),
        "independent_per_pose_recomputation": pose_audit,
        "payload_revision": collision_json.get("revision"),
    }
    return runtime_report, trajectory_report, collision_report


def normalized_pair_set(value: Any, label: str) -> set[tuple[str, str]]:
    require(isinstance(value, list), f"{label}: pair list missing")
    result: set[tuple[str, str]] = set()
    for row in value:
        require(isinstance(row, list) and len(row) == 2, f"{label}: malformed pair")
        pair = tuple(sorted((str(row[0]), str(row[1]))))
        require(pair not in result, f"{label}: duplicate pair {pair}")
        result.add(pair)
    return result


def recompute_collision_503(
    report: Mapping[str, Any],
    expected_negative_controls: Mapping[str, Sequence[int]],
) -> dict[str, Any]:
    """Recompute every acceptance aggregate from the 503 raw pose records."""

    poses = report.get("poses")
    require(isinstance(poses, list) and len(poses) == 503, "raw 503 pose array has wrong length")
    canonical_rows: list[dict[str, Any]] = []
    reconstructed_pose_sets: dict[str, list[dict[str, float]]] = {}
    mismatches: list[int] = []
    boolean_mismatches = 0
    self_pair_mismatches = 0
    ground_pair_diagnostic_mismatches = 0
    counts = {
        "moveit": {"overall": 0, "self": 0, "ground": 0},
        "mujoco": {"overall": 0, "self": 0, "ground": 0},
    }
    pose_by_key: dict[tuple[str, int], Mapping[str, Any]] = {}
    for expected_index, pose in enumerate(poses):
        require(isinstance(pose, dict), f"raw pose {expected_index} is not an object")
        require(int(pose.get("global_index", -1)) == expected_index, f"raw pose index discontinuity at {expected_index}")
        category = str(pose.get("category"))
        pose_index = int(pose.get("pose_index", -1))
        key = (category, pose_index)
        require(key not in pose_by_key, f"duplicate category pose key: {key}")
        pose_by_key[key] = pose
        angles = pose.get("angles_deg")
        require(isinstance(angles, dict) and set(angles) == set(JOINTS), f"raw pose joint set mismatch: {expected_index}")
        angle_row = [finite_number(angles[name], f"pose[{expected_index}].{name}") for name in JOINTS]
        category_rows = reconstructed_pose_sets.setdefault(category, [])
        require(
            pose_index == len(category_rows),
            f"category pose_index is not contiguous/order-preserving: {category}[{pose_index}]",
        )
        category_rows.append(
            {
                f"j{index}_deg": float(angle_row[index - 1])
                for index in range(1, 7)
            }
        )
        moveit = pose.get("moveit")
        mujoco = pose.get("mujoco")
        require(isinstance(moveit, dict) and isinstance(mujoco, dict), f"raw pose runtime records missing: {expected_index}")
        moveit_overall = moveit.get("collision") is True
        moveit_self = moveit.get("self_collision") is True
        moveit_ground = moveit.get("ground_collision") is True
        mujoco_overall = mujoco.get("collision") is True
        mujoco_self = mujoco.get("self_collision") is True
        mujoco_ground = mujoco.get("ground_collision") is True
        require(moveit_overall == (moveit_self or moveit_ground), f"raw MoveIt collision classification inconsistent: {expected_index}")
        require(mujoco_overall == (mujoco_self or mujoco_ground), f"raw MuJoCo collision classification inconsistent: {expected_index}")
        require((mujoco.get("safe") is True) == (not mujoco_overall), f"raw MuJoCo safe/collision inconsistent: {expected_index}")
        for engine, overall, self_collision, ground_collision in (
            ("moveit", moveit_overall, moveit_self, moveit_ground),
            ("mujoco", mujoco_overall, mujoco_self, mujoco_ground),
        ):
            counts[engine]["overall"] += int(overall)
            counts[engine]["self"] += int(self_collision)
            counts[engine]["ground"] += int(ground_collision)
        bool_match = (
            moveit_overall == mujoco_overall
            and moveit_self == mujoco_self
            and moveit_ground == mujoco_ground
        )
        moveit_self_pairs = normalized_pair_set(pose.get("moveit_self_proxy_pairs"), f"pose[{expected_index}].moveit_self_pairs")
        mujoco_self_pairs = normalized_pair_set(pose.get("mujoco_self_proxy_pairs"), f"pose[{expected_index}].mujoco_self_pairs")
        moveit_ground_pairs = normalized_pair_set(pose.get("moveit_ground_proxy_pairs"), f"pose[{expected_index}].moveit_ground_pairs")
        mujoco_ground_pairs = normalized_pair_set(pose.get("mujoco_ground_proxy_pairs"), f"pose[{expected_index}].mujoco_ground_pairs")
        self_pair_match = moveit_self_pairs == mujoco_self_pairs
        ground_pair_match = moveit_ground_pairs == mujoco_ground_pairs
        collision_match = bool_match and self_pair_match
        boolean_mismatches += int(not bool_match)
        self_pair_mismatches += int(not self_pair_match)
        ground_pair_diagnostic_mismatches += int(not ground_pair_match)
        if not collision_match:
            mismatches.append(expected_index)
        require(pose.get("overall_collision_boolean_match") is (moveit_overall == mujoco_overall), f"stored overall boolean flag mismatch: {expected_index}")
        require(pose.get("self_collision_boolean_match") is (moveit_self == mujoco_self), f"stored self boolean flag mismatch: {expected_index}")
        require(pose.get("ground_collision_boolean_match") is (moveit_ground == mujoco_ground), f"stored ground boolean flag mismatch: {expected_index}")
        require(pose.get("self_collision_pair_set_match") is self_pair_match, f"stored self-pair flag mismatch: {expected_index}")
        require(pose.get("ground_collision_pair_set_match") is ground_pair_match, f"stored ground-pair flag mismatch: {expected_index}")
        require(pose.get("collision_match") is collision_match, f"stored collision-match flag mismatch: {expected_index}")
        canonical_rows.append(
            {
                "global_index": expected_index,
                "category": category,
                "pose_index": pose_index,
                "angles_deg": angle_row,
                "moveit": [moveit_overall, moveit_self, moveit_ground],
                "mujoco": [mujoco_overall, mujoco_self, mujoco_ground],
                "moveit_self_pairs": sorted(moveit_self_pairs),
                "mujoco_self_pairs": sorted(mujoco_self_pairs),
                "bool_match": bool_match,
                "self_pair_match": self_pair_match,
                "ground_pair_diagnostic_match": ground_pair_match,
            }
        )
    match_count = 503 - len(mismatches)
    require(match_count == int(report.get("match_count", -1)), "recomputed 503 match count differs from report")
    require(len(mismatches) == int(report.get("mismatch_count", -1)), "recomputed 503 mismatch count differs from report")
    require(boolean_mismatches == int(report.get("boolean_mismatch_count", -1)), "recomputed boolean mismatch count differs")
    require(self_pair_mismatches == int(report.get("self_pair_set_mismatch_count", -1)), "recomputed self-pair mismatch count differs")
    require(ground_pair_diagnostic_mismatches == int(report.get("ground_pair_set_diagnostic_mismatch_count", -1)), "recomputed ground-pair diagnostic count differs")
    expected_counts = report.get("expected_frozen_collision_pose_counts")
    require(expected_counts == {"overall": 182, "self": 126, "ground": 118}, "frozen count authority mismatch during pose audit")
    require(counts["moveit"] == expected_counts and counts["mujoco"] == expected_counts, "recomputed frozen collision counts differ")

    controls = report.get("reference_terminal_collision_negative_controls")
    require(isinstance(controls, list) and len(controls) == 5, "negative control list invalid during pose audit")
    actual_control_identity: dict[str, list[int]] = {}
    for control in controls:
        require(isinstance(control, dict), "negative control record invalid")
        key = (str(control.get("category")), int(control.get("pose_index", -1)))
        actual_control_identity.setdefault(key[0], []).append(key[1])
        require(key in pose_by_key, f"negative control target missing: {key}")
        pose = pose_by_key[key]
        require(
            pose["moveit"]["collision"] is True
            and pose["mujoco"]["collision"] is True
            and pose["moveit"]["self_collision"] is True
            and pose["mujoco"]["self_collision"] is True,
            f"negative control is not terminal self-collision: {key}",
        )
        require(control.get("pass") is True, f"stored negative-control pass flag is false: {key}")
    for indices in actual_control_identity.values():
        indices.sort()
    expected_identity = {
        str(category): sorted(int(index) for index in indices)
        for category, indices in expected_negative_controls.items()
    }
    require(
        actual_control_identity == expected_identity,
        f"negative-control identity mismatch: {actual_control_identity} != {expected_identity}",
    )
    digest = sha256_bytes(
        json.dumps(canonical_rows, ensure_ascii=False, sort_keys=True, separators=(",", ":")).encode("utf-8")
    )
    frozen_pose_payload = json.dumps(
        reconstructed_pose_sets,
        ensure_ascii=False,
        sort_keys=True,
        separators=(",", ":"),
    ).encode("utf-8")
    frozen_pose_set_sha256 = sha256_bytes(frozen_pose_payload)
    require(
        frozen_pose_set_sha256
        == "4d9cfaf15cc20fe16393d1aa3bccddc5273e77e2de08cf9d0045976c301799aa",
        "independently reconstructed raw pose-set SHA256 mismatch",
    )
    return {
        "pass": not mismatches,
        "method": "INDEPENDENT_RECOMPUTATION_FROM_EACH_RAW_POSE_RECORD",
        "pose_count": len(canonical_rows),
        "canonical_pose_facts_sha256": digest,
        "reconstructed_frozen_pose_set_sha256": frozen_pose_set_sha256,
        "reconstructed_frozen_pose_category_counts": {
            category: len(rows) for category, rows in reconstructed_pose_sets.items()
        },
        "match_count": match_count,
        "mismatch_count": len(mismatches),
        "mismatch_global_indices": mismatches,
        "boolean_mismatch_count": boolean_mismatches,
        "self_pair_set_mismatch_count": self_pair_mismatches,
        "ground_pair_set_diagnostic_mismatch_count": ground_pair_diagnostic_mismatches,
        "recomputed_collision_pose_counts": counts,
        "negative_control_count": len(controls),
        "negative_control_identity": actual_control_identity,
        "negative_controls_pass": True,
    }


def validate_protected_files(inertia_authority: Mapping[str, Any]) -> tuple[dict[str, Any], list[str]]:
    protected_rows = inertia_authority.get("authorities", {}).get("protected_inputs_after")
    require(isinstance(protected_rows, list), "Engineering authority protected input list missing")
    rows: list[dict[str, Any]] = []
    watched: list[str] = []
    seen = set()
    for record in protected_rows:
        require(isinstance(record, dict), "protected input record is not an object")
        relative = str(record.get("path")).replace("\\", "/")
        expected = str(record.get("expected_sha256") or record.get("sha256")).lower()
        require(relative and relative not in seen, f"duplicate protected path: {relative}")
        seen.add(relative)
        path = repo_path(relative)
        require(path.is_file(), f"protected file missing: {relative}")
        actual = sha256_file(path)
        passed = actual == expected
        require(passed, f"protected file hash mismatch: {relative}")
        rows.append({"path": relative, "expected_sha256": expected, "actual_sha256": actual, "unchanged": passed})
        watched.append(relative)

    baseline_rows = []
    for relative in V14_PROTECTED_REL:
        current = repo_path(relative)
        require(current.is_file(), f"V15.14 protected file missing: {relative}")
        actual = sha256_file(current)
        expected = sha256_bytes(git_blob(relative))
        baseline_blob_oid = git_text(["rev-parse", f"{SOURCE_COMMIT}:{relative}"])
        current_blob_oid = git_text(["hash-object", "--path", relative, "--", relative])
        passed = current_blob_oid == baseline_blob_oid
        require(passed, f"V15.14 protected file changed: {relative}")
        baseline_rows.append(
            {
                "path": relative,
                "source_commit_canonical_lf_sha256": expected,
                "working_tree_raw_sha256": actual,
                "source_commit_git_blob_oid": baseline_blob_oid,
                "working_tree_clean_filtered_git_blob_oid": current_blob_oid,
                "line_ending_agnostic_git_content_unchanged": passed,
            }
        )
        watched.append(relative)
    return {
        "pass": True,
        "engineering_authority_protected_inputs": rows,
        "v15_14_collision_semantic_control_protected_files": baseline_rows,
        "all_protected_hashes_match": True,
    }, sorted(set(watched))


def capture_hashes(relatives: Iterable[str]) -> dict[str, str | None]:
    result: dict[str, str | None] = {}
    for relative in sorted(set(relatives)):
        path = repo_path(relative)
        result[relative] = sha256_file(path) if path.is_file() else None
    return result


def failed_gate(error: Exception) -> dict[str, Any]:
    return {"pass": False, "error": f"{type(error).__name__}: {error}"}


def report_link_status(report: Mapping[str, Any], name: str) -> bool:
    return bool(report.get("links", {}).get(name, {}).get("pass"))


def production_hash_migration_state() -> dict[str, Any]:
    bridge_relative = (
        V14_REL
        + "/ros2_ws/src/go_m8010_arm_mujoco_bridge/scripts/mujoco_bridge.py"
    )
    bridge_path = repo_path(bridge_relative)
    accepted = python_literal_assignment(bridge_path, "ACCEPTED_MODEL_SHA256")
    require(isinstance(accepted, str) and len(accepted) == 64, "production bridge hash literal invalid")
    current = sha256_file(repo_path(MJCF_REL))
    pending = current != accepted
    require(pending, "production bridge unexpectedly accepts V15.17B before authorized migration")
    return {
        "pass": True,
        "status": "BLOCKED_PENDING_HASH_MIGRATION",
        "production_hash_migration_pending": True,
        "accepted_old_mjcf_sha256": accepted,
        "current_explicit_principal_mjcf_sha256": current,
        "bridge_source_modified": False,
        "classified_as_physics_regression": False,
        "hard_gate_for_v15_17b_parameter_deployment": False,
    }


def validate_v15_17b_history() -> dict[str, Any]:
    actual = {
        relative: sha256_file(repo_path(relative))
        for relative in EXPECTED_V15_17B_REPORT_SHA256
    }
    for relative, expected in EXPECTED_V15_17B_REPORT_SHA256.items():
        require(actual[relative] == expected, f"frozen V15.17B report changed: {relative}")
    document = read_json(repo_path(V15_17B_REPORT_JSON_REL))
    require(
        document.get("schema")
        == "go-m8010-arm-v15.17b-explicit-principal-inertia-validation/1.0"
        and document.get("revision") == "V15.17B-EXPLICIT_PRINCIPAL_INERTIA"
        and document.get("audit_valid") is True,
        "V15.17B history identity/audit gate failed",
    )
    require(
        document.get("hard_unresolved_items")
        == ["LINK6_LEGACY_FULLINERTIA_GEOM_SAMEFRAME_ARTIFACT"],
        "V15.17B history no longer contains the single C diagnosis subject",
    )
    require(
        document.get("mujoco_compiled_readback", {}).get(
            "all_six_compiled_tensor_hard_pass"
        )
        is True,
        "V15.17B frozen six-body tensor evidence is not PASS",
    )
    return {
        "pass": True,
        "source_hashes_sha256": actual,
        "v15_17b_audit_valid": True,
        "v15_17b_final_status": document.get("final_status"),
        "v15_17b_single_blocker": document.get("hard_unresolved_items")[0],
        "history_preserved_not_rewritten_by_v15_17c": True,
    }


def validate_v15_17c_diagnostic_inputs() -> dict[str, Any]:
    actual = {
        relative: sha256_file(repo_path(relative))
        for relative in EXPECTED_V15_17C_DIAGNOSTIC_INPUT_SHA256
    }
    for relative, expected in EXPECTED_V15_17C_DIAGNOSTIC_INPUT_SHA256.items():
        require(actual[relative] == expected, f"V15.17C diagnostic input changed: {relative}")

    fjt = read_json(repo_path(V15_17C_FJT_EVIDENCE_REL))
    require(
        fjt.get("schema") == "go_m8010_v15_14_visual_sync_evidence_v1",
        "V15.14 accepted FJT evidence schema mismatch",
    )
    require(fjt.get("pass") is True, "V15.14 accepted FJT evidence is not PASS")
    trajectory = fjt.get("trajectory")
    action = fjt.get("action")
    final = fjt.get("final")
    require(
        isinstance(trajectory, dict)
        and isinstance(action, dict)
        and isinstance(final, dict),
        "V15.14 accepted FJT evidence structure missing",
    )
    target = tuple(
        finite_number(value, "V15.14 accepted FJT target")
        for value in trajectory.get("target_position_rad", [])
    )
    require(target == V15_17C_FJT_POSITION_RAD, "V15.14 accepted FJT target changed")
    require(
        action.get("accepted") is True
        and action.get("goal_status") == action.get("goal_status_succeeded_constant") == 4
        and action.get("result_error_code") == 0,
        "V15.14 accepted FJT action did not succeed",
    )
    require(
        tuple(final.get("mujoco_raw_joint_position_rad", [])) == target
        and finite_number(final.get("max_abs_raw_minus_target_rad"), "FJT final target error")
        == 0.0,
        "V15.14 accepted FJT final pose did not exactly reach target",
    )

    mesh_manifest = read_json(repo_path(RUNTIME_MESH_MANIFEST_REL))
    matching_records: list[dict[str, Any]] = []

    def visit(value: Any) -> None:
        if isinstance(value, dict):
            if value.get("path") == "meshes/visual_chunks_mm/link6/001.stl":
                matching_records.append(value)
            for child in value.values():
                visit(child)
        elif isinstance(value, list):
            for child in value:
                visit(child)

    visit(mesh_manifest)
    require(matching_records, "runtime mesh manifest lacks link6 visual mesh")
    require(
        all(
            str(record.get("sha256", "")).lower()
            == EXPECTED_V15_17C_DIAGNOSTIC_INPUT_SHA256[LINK6_VISUAL_MESH_REL]
            for record in matching_records
        ),
        "runtime mesh manifest link6 visual hash mismatch",
    )
    return {
        "pass": True,
        "source_hashes_sha256": actual,
        "fjt_pose_source": V15_17C_FJT_EVIDENCE_REL,
        "fjt_pose_id": "V15_14_ACCEPTED_FOLLOW_JOINT_TRAJECTORY_FINAL_POSE",
        "fjt_joint_position_rad": list(target),
        "fjt_action_succeeded": True,
        "visual_mesh_manifest_record_count": len(matching_records),
        "visual_mesh_source_path": LINK6_VISUAL_MESH_REL,
        "visual_mesh_source_sha256": actual[LINK6_VISUAL_MESH_REL],
    }


def write_v15_17c_child_manifest(destination: Path) -> None:
    inputs = validate_v15_17c_diagnostic_inputs()
    manifest = read_json(repo_path(TARGET_MANIFEST_REL))
    manifest["v15_17c_fjt_accepted_pose"] = {
        "source_id": inputs["fjt_pose_id"],
        "source_sha256": inputs["source_hashes_sha256"][
            V15_17C_FJT_EVIDENCE_REL
        ],
        "joint_position_rad": inputs["fjt_joint_position_rad"],
    }
    manifest["v15_17c_visual_asset_identity"] = {
        "runtime_mesh_manifest_sha256": inputs["source_hashes_sha256"][
            RUNTIME_MESH_MANIFEST_REL
        ],
        "source_mesh_path": inputs["visual_mesh_source_path"],
        "source_mesh_sha256": inputs["visual_mesh_source_sha256"],
    }
    destination.write_text(
        json.dumps(manifest, ensure_ascii=False, sort_keys=True), encoding="utf-8"
    )


def make_final_report_items(
    facts: Mapping[str, Any], unresolved: Sequence[str]
) -> list[dict[str, Any]]:
    values: list[tuple[str, Any]] = [
        ("branch / commit", facts.get("branch_commit")),
        ("visual__link6__001 OLD / NEW geom_sameframe", facts.get("old_new_geom_sameframe")),
        ("compiled geom_pos delta", facts.get("compiled_geom_pos_delta_m")),
        ("mechanical zero runtime geom_xpos delta", facts.get("mechanical_zero_runtime_position_delta_m")),
        ("mechanical zero runtime rotation delta", facts.get("mechanical_zero_runtime_rotation_delta_rad")),
        ("J6 +0.5 runtime position / rotation delta", facts.get("j6_plus_0p5_runtime_delta")),
        ("J6 -0.5 runtime position / rotation delta", facts.get("j6_minus_0p5_runtime_delta")),
        ("nonzero six-axis pose runtime position / rotation delta", facts.get("nonzero_six_axis_runtime_delta")),
        ("link6 body runtime maximum position delta", facts.get("link6_body_runtime_max_position_delta_m")),
        ("link6 body runtime maximum rotation delta", facts.get("link6_body_runtime_max_rotation_delta_rad")),
        ("collision geoms unchanged", facts.get("collision_geoms_unchanged")),
        ("joint / TF unchanged", facts.get("joint_tf_unchanged")),
        ("artifact classification", facts.get("artifact_classification")),
        ("visual-only limitation accepted", facts.get("visual_only_limitation_accepted")),
        ("production bridge hash modified", facts.get("production_bridge_hash_modified")),
        ("gravity enabled", facts.get("gravity_enabled")),
        ("hard unresolved items", list(unresolved)),
        ("final status", facts.get("final_status")),
    ]
    require(len(values) == 18, "V15.17C final report item schema is not exactly 18 items")
    return [
        {"number": index, "label": label, "value": value}
        for index, (label, value) in enumerate(values, start=1)
    ]


def build_report(
    urdf_python: str | None,
    mujoco_python: str | None,
    runtime_evidence_path: Path | None,
    wsl_distro: str | None,
) -> dict[str, Any]:
    # This is intentionally the first filesystem operation in the validation
    # pipeline: every known protected/deployment input is snapshotted before
    # JSON parsing, git/network probes, child interpreters, or report logic.
    all_watch = sorted(
        set(
            [
                MASS_REL,
                COM_REL,
                INERTIA_REL,
                XACRO_REL,
                MJCF_REL,
                APPLY_REL,
                VALIDATOR_REL,
                REPORT_JSON_REL,
                REPORT_MD_REL,
                V15_17B_REPORT_JSON_REL,
                V15_17B_REPORT_MD_REL,
                *EXPECTED_V15_17C_DIAGNOSTIC_INPUT_SHA256,
                *V14_PROTECTED_REL,
                *ENGINEERING_PROTECTED_REL,
            ]
        )
    )
    before_hashes = capture_hashes(all_watch)
    changed_before = current_changed_paths()

    try:
        git_scope = validate_git_and_scope()
    except Exception as error:
        git_scope = failed_gate(error)

    try:
        nested_apply = validate_nested_apply_check(mujoco_python, wsl_distro)
    except Exception as error:
        nested_apply = failed_gate(error)

    authority_map: dict[str, dict[str, Any]] | None = None
    inertia_authority: dict[str, Any] | None = None
    try:
        authority_report, authority_map, inertia_authority = validate_authorities()
    except Exception as error:
        authority_report = failed_gate(error)

    if authority_map is not None:
        try:
            source_xml = validate_source_xml_and_frozen_semantics(authority_map)
        except Exception as error:
            source_xml = failed_gate(error)
        try:
            urdf_report, offline_ros = validate_urdf(authority_map, urdf_python, wsl_distro)
        except Exception as error:
            urdf_report = failed_gate(error)
            offline_ros = {"pass": False, "error": "blocked by URDF validation: " + str(error), "ros_runtime_executed": False, "ros_runtime_pass_claimed": False}
        try:
            mjcf_report, collision_offline, kinematics_report = validate_mjcf_v15_17b(
                authority_map, mujoco_python, wsl_distro
            )
        except Exception as error:
            mjcf_report = failed_gate(error)
            collision_offline = {"pass": False, "error": "blocked by MuJoCo validation: " + str(error)}
            kinematics_report = {"pass": False, "error": "blocked by MuJoCo validation: " + str(error), "ros_trajectory_loop_runtime_executed": False, "ros_trajectory_loop_runtime_pass_claimed": False}
    else:
        blocked = {"pass": False, "error": "blocked by authority validation"}
        source_xml = dict(blocked)
        urdf_report = dict(blocked)
        offline_ros = {**blocked, "ros_runtime_executed": False, "ros_runtime_pass_claimed": False}
        mjcf_report = dict(blocked)
        collision_offline = dict(blocked)
        kinematics_report = {**blocked, "ros_trajectory_loop_runtime_executed": False, "ros_trajectory_loop_runtime_pass_claimed": False}

    # V15.17C adds an in-process MuJoCo A/B runtime diagnostic but deliberately
    # does not migrate the immutable production bridge hash.  That separately
    # authorized pending migration is not a physics regression or a C hard
    # gate.
    runtime_diagnostic = {
        "provided": runtime_evidence_path is not None,
        "external_runtime_evidence_consumed_as_v15_17c_hard_gate": False,
        "internal_four_pose_mujoco_ab_runtime_diagnostic_executed": (
            isinstance(mjcf_report, dict)
            and mjcf_report.get("v15_17c_sameframe_runtime_diagnostic", {}).get(
                "pose_count"
            )
            == 4
        ),
        "reason": "V15.17C_USES_INTERNAL_FOUR_POSE_MUJOCO_AB_RUNTIME_DIAGNOSTIC",
    }
    try:
        production_trajectory = production_hash_migration_state()
    except Exception as error:
        production_trajectory = failed_gate(error)
    collision_report = collision_offline

    try:
        v15_17b_history = validate_v15_17b_history()
    except Exception as error:
        v15_17b_history = failed_gate(error)
    try:
        v15_17c_diagnostic_inputs = validate_v15_17c_diagnostic_inputs()
    except Exception as error:
        v15_17c_diagnostic_inputs = failed_gate(error)

    protected_watch: list[str] = []
    if inertia_authority is not None:
        try:
            protected_report, protected_watch = validate_protected_files(inertia_authority)
        except Exception as error:
            protected_report = failed_gate(error)
    else:
        protected_report = {"pass": False, "error": "blocked by authority validation"}

    require(set(protected_watch) <= set(all_watch), "protected path set escaped initial TOCTOU snapshot")
    after_hashes = capture_hashes(all_watch)
    changed_after = current_changed_paths()
    try:
        git_scope_after = validate_git_and_scope()
    except Exception as error:
        git_scope_after = failed_gate(error)
    hash_changes = sorted(relative for relative in all_watch if before_hashes.get(relative) != after_hashes.get(relative))
    git_scope_stable = (
        git_scope_after.get("pass") is True
        and git_scope.get("_runtime_head_commit")
        == git_scope_after.get("_runtime_head_commit")
        and git_scope.get("current_branch") == git_scope_after.get("current_branch")
        and git_scope.get("source_commit") == git_scope_after.get("source_commit")
        and git_scope.get("remote_source_sha") == git_scope_after.get("remote_source_sha")
        and git_scope.get("changed_paths") == git_scope_after.get("changed_paths")
    )
    # The actual target commit cannot be serialized in a pre-commit report
    # without making the report self-referential, but it is still compared
    # above as a live TOCTOU token.
    git_scope.pop("_runtime_head_commit", None)
    git_scope_after.pop("_runtime_head_commit", None)
    def disclosed_hashes(values: Mapping[str, str | None]) -> dict[str, str | None]:
        return {
            relative: (
                "SELF_REPORT_PRESENT_HASH_REDACTED"
                if relative in (REPORT_JSON_REL, REPORT_MD_REL) and digest is not None
                else digest
            )
            for relative, digest in values.items()
        }

    toctou = {
        "pass": not hash_changes and changed_before == changed_after and git_scope_stable,
        "watched_file_count": len(all_watch),
        "hashes_before": disclosed_hashes(before_hashes),
        "hashes_after": disclosed_hashes(after_hashes),
        "self_report_hashes_redacted_to_avoid_recursive_report_content": True,
        "self_reports": {
            relative: {
                "existed_before": before_hashes.get(relative) is not None,
                "exists_after": after_hashes.get(relative) is not None,
                "unchanged_during_validation": before_hashes.get(relative)
                == after_hashes.get(relative),
            }
            for relative in (REPORT_JSON_REL, REPORT_MD_REL)
        },
        "changed_paths_before": changed_before,
        "changed_paths_after": changed_after,
        "files_changed_during_validation": hash_changes,
        "git_scope_changed_during_validation": changed_before != changed_after,
        "git_scope_revalidated_after_all_gates": git_scope_after,
        "git_scope_stable": git_scope_stable,
    }

    sameframe_runtime = (
        mjcf_report.get("v15_17c_sameframe_runtime_diagnostic", {})
        if isinstance(mjcf_report, dict)
        else {}
    )
    required = {
        "GIT_REMOTE_OR_EXACT8_SCOPE_FAILED": git_scope,
        "NESTED_APPLY_CHECK_FAILED": nested_apply,
        "FROZEN_AUTHORITY_VALIDATION_FAILED": authority_report,
        "SOURCE_EXPLICIT_PRINCIPAL_REPRESENTATION_FAILED": source_xml,
        "URDF_UNCHANGED_READBACK_FAILED": urdf_report,
        "MUJOCO_EXPLICIT_PRINCIPAL_COMPILED_READBACK_FAILED": mjcf_report,
        "OFFLINE_ROS_STRUCTURE_VALIDATION_FAILED": offline_ros,
        "COLLISION_REGRESSION_FAILED": collision_report,
        "JOINT_OR_TF_REGRESSION_FAILED": kinematics_report,
        "PRODUCTION_HASH_PENDING_CLASSIFICATION_FAILED": production_trajectory,
        "V15_17B_HISTORY_GUARD_FAILED": v15_17b_history,
        "V15_17C_DIAGNOSTIC_INPUT_GUARD_FAILED": v15_17c_diagnostic_inputs,
        "PROTECTED_FILE_GUARD_FAILED": protected_report,
        "TOCTOU_GUARD_FAILED": toctou,
    }
    hard_unresolved = [
        code for code, record in required.items() if record.get("pass") is not True
    ]
    if sameframe_runtime.get("pass") is not True:
        diagnostic_unresolved = sameframe_runtime.get("hard_unresolved_items")
        if isinstance(diagnostic_unresolved, list) and diagnostic_unresolved:
            hard_unresolved.extend(str(code) for code in diagnostic_unresolved)
        else:
            hard_unresolved.append("V15_17C_RUNTIME_DIAGNOSTIC_FAILED")
    hard_unresolved = list(dict.fromkeys(hard_unresolved))
    overall_pass = not hard_unresolved
    pose_map = {
        str(row.get("id")): row
        for row in sameframe_runtime.get("poses", [])
        if isinstance(row, dict)
    }
    storage = sameframe_runtime.get("compiled_storage_diagnostic", {})
    maxima = sameframe_runtime.get("runtime_maxima", {})
    case = sameframe_runtime.get("case")
    if overall_pass:
        final_status = str(sameframe_runtime.get("final_status_if_all_other_gates_pass"))
    else:
        final_status = "V15.17 INERTIAL_DEPLOYMENT = FAIL"

    def visual_pose_fact(pose_id: str) -> dict[str, Any] | None:
        row = pose_map.get(pose_id)
        if not isinstance(row, dict):
            return None
        return {
            "position_delta_euclidean_norm_m": row.get(
                "visual_runtime_position_delta_m"
            ),
            "position_delta_max_abs_component_m": row.get(
                "visual_runtime_position_max_abs_component_delta_m"
            ),
            "rotation_delta_rad": row.get("visual_runtime_rotation_delta_rad"),
        }

    mechanical_zero = pose_map.get("mechanical_zero", {})
    final_facts = {
        "branch_commit": {
            "branch": git_scope.get("current_branch"),
            "source_commit": SOURCE_COMMIT,
            "head_contract": "SOURCE_COMMIT_OR_ITS_UNIQUE_DIRECT_CHILD",
        },
        "old_new_geom_sameframe": {
            "old_fullinertia": storage.get("old_geom_sameframe"),
            "new_explicit_principal": storage.get("new_geom_sameframe"),
        },
        "compiled_geom_pos_delta_m": storage.get("compiled_geom_pos_delta_m"),
        "mechanical_zero_runtime_position_delta_m": {
            "euclidean_norm_m": mechanical_zero.get(
                "visual_runtime_position_delta_m"
            ),
            "max_abs_component_m": mechanical_zero.get(
                "visual_runtime_position_max_abs_component_delta_m"
            ),
        },
        "mechanical_zero_runtime_rotation_delta_rad": mechanical_zero.get(
            "visual_runtime_rotation_delta_rad"
        ),
        "j6_plus_0p5_runtime_delta": visual_pose_fact("j6_plus_0p5"),
        "j6_minus_0p5_runtime_delta": visual_pose_fact("j6_minus_0p5"),
        "nonzero_six_axis_runtime_delta": {
            "source_target_id": "V15_14_ACCEPTED_FOLLOW_JOINT_TRAJECTORY_FINAL_POSE",
            "source_sha256": EXPECTED_V15_17C_DIAGNOSTIC_INPUT_SHA256[
                V15_17C_FJT_EVIDENCE_REL
            ],
            **(visual_pose_fact("v15_14_accepted_fjt_final_pose") or {}),
        },
        "link6_body_runtime_max_position_delta_m": maxima.get(
            "link6_body_position_delta_m"
        ),
        "link6_body_runtime_max_rotation_delta_rad": maxima.get(
            "link6_body_rotation_delta_rad"
        ),
        "collision_geoms_unchanged": (
            "YES"
            if sameframe_runtime.get("collision_runtime_evidence", {}).get("all_exact")
            is True
            and collision_report.get("pass") is True
            else "NO"
        ),
        "joint_tf_unchanged": (
            "YES" if kinematics_report.get("pass") is True else "NO"
        ),
        "artifact_classification": {
            "case": case,
            "classification": sameframe_runtime.get("artifact_classification"),
            "engineering_acceptance": sameframe_runtime.get("engineering_acceptance"),
            "limitation": (
                "NON_PHYSICAL_VISUAL_NUMERICAL_LIMITATION"
                if sameframe_runtime.get("non_physical_visual_numerical_limitation")
                is True
                else None
            ),
            "visual_delta_scale": sameframe_runtime.get("visual_delta_scale"),
        },
        "visual_only_limitation_accepted": (
            "YES"
            if sameframe_runtime.get("visual_only_limitation_accepted") is True
            else "NO"
        ),
        "production_bridge_hash_modified": (
            "NO"
            if protected_report.get("pass") is True
            and production_trajectory.get("bridge_source_modified") is False
            else "YES_OR_UNKNOWN"
        ),
        "gravity_enabled": (
            "NO"
            if source_xml.get("gravity_enabled") is False
            and mjcf_report.get("gravity_enabled") is False
            else "YES_OR_UNKNOWN"
        ),
        "hard_unresolved_items": hard_unresolved,
        "final_status": {
            "v15_17c": final_status,
            "case": case,
            "production_hash_migration_pending": (
                "YES"
                if production_trajectory.get("production_hash_migration_pending") is True
                else "NO_OR_UNKNOWN"
            ),
        },
    }
    final_report_items = make_final_report_items(final_facts, hard_unresolved)
    require(
        [item["number"] for item in final_report_items] == list(range(1, 19)),
        "V15.17C final report item numbering is not exactly 1..18",
    )

    return {
        "schema": "go-m8010-arm-v15.17c-sameframe-runtime-diagnosis/1.0",
        "revision": "V15.17C-LINK6_VISUAL_SAMEFRAME_FINAL_DIAGNOSIS",
        "status_name": (
            "INERTIAL_DEPLOYMENT_ACCEPTED_WITH_NONPHYSICAL_VISUAL_LIMITATION"
            if overall_pass and case == "CASE_2"
            else "INERTIAL_DEPLOYMENT_ACCEPTED"
            if overall_pass
            else "INERTIAL_DEPLOYMENT_NOT_ACCEPTED"
        ),
        "representation": "EXPLICIT_PRINCIPAL_FRAME_DIAGINERTIA",
        "full_real_dynamics_complete": False,
        "scope_links": list(LINKS),
        "scope": "LINK6_VISUAL_RUNTIME_WORLD_POSE_DIAGNOSIS_NO_MODEL_OR_HASH_MIGRATION",
        "git_scope_guard": git_scope,
        "nested_apply_check": nested_apply,
        "authorities": authority_report,
        "v15_17b_frozen_history": v15_17b_history,
        "v15_17c_diagnostic_inputs": v15_17c_diagnostic_inputs,
        "source_xml_and_frozen_semantics": source_xml,
        "urdf_readback": urdf_report,
        "mujoco_compiled_readback": mjcf_report,
        "six_body_tensor_reconstruction_pass": mjcf_report.get(
            "all_six_compiled_tensor_hard_pass"
        )
        is True,
        "v15_17c_sameframe_runtime_diagnostic": sameframe_runtime,
        "static_offline_ros_model_loads": offline_ros,
        "collision_regression": collision_report,
        "joint_tf_regression": kinematics_report,
        "production_trajectory_loop": production_trajectory,
        "runtime_evidence_diagnostic": runtime_diagnostic,
        "protected_file_guard": protected_report,
        "toctou_guard": toctou,
        "prohibited_actions": {
            "inertia_modified_by_v15_17c": False,
            "mjcf_modified_by_v15_17c": False,
            "xacro_or_urdf_modified_by_v15_17c": False,
            "geometry_or_mesh_modified_by_v15_17c": False,
            "production_bridge_hash_modified": False,
            "controller_modified": False,
            "gravity_enabled": False,
            "armature_damping_friction_added": False,
        },
        "hard_unresolved_items": hard_unresolved,
        "production_hash_migration_pending": (
            production_trajectory.get("production_hash_migration_pending") is True
        ),
        "final_facts": final_facts,
        "final_report_items": final_report_items,
        "audit_valid": True,
        "pass": overall_pass,
        "final_status": final_status,
    }


def json_bytes(report: Mapping[str, Any]) -> bytes:
    return (json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n").encode("utf-8")


def markdown_text(report: Mapping[str, Any]) -> str:
    unresolved = report.get("hard_unresolved_items", [])
    numbered_facts = report.get("final_report_items")
    require(
        isinstance(numbered_facts, list)
        and len(numbered_facts) == 18
        and [item.get("number") for item in numbered_facts] == list(range(1, 19)),
        "Markdown input does not contain exact JSON V15.17C final_report_items 1..18",
    )
    sameframe = report.get("v15_17c_sameframe_runtime_diagnostic", {})
    storage = sameframe.get("compiled_storage_diagnostic", {})
    maxima = sameframe.get("runtime_maxima", {})
    scale = sameframe.get("visual_delta_scale", {})
    collision = sameframe.get("collision_runtime_evidence", {})
    non_target = sameframe.get("non_target_named_geom_runtime_evidence", {})
    pose_rows = sameframe.get("poses", [])
    lines = [
        "# V15.17C link6 visual sameframe 运行时诊断",
        "",
        f"- 最终状态：**{report['final_status']}**",
        f"- 状态名称：`{report['status_name']}`",
        f"- 审计有效：`{report.get('audit_valid')}`",
        f"- CASE：`{sameframe.get('case')}`",
        f"- 分类：`{sameframe.get('artifact_classification')}`",
        f"- 工程裁决：`{sameframe.get('engineering_acceptance')}`",
        "- 对象：`visual__link6__001`；A=旧 fullinertia，B=当前 explicit principal-frame quat+diaginertia。",
        "- 四姿态均由同一 MuJoCo 解释器加载 A/B，设置相同 qpos/qvel 后执行 `mj_forward`，验收使用 `MjData.geom_xpos/geom_xmat`，不使用 compiled local `model.geom_pos/geom_quat` 作为 world-pose 证据。",
        "- production bridge 保持旧 hash 并继续 fail-closed；本任务不执行 hash migration。",
        "- 重力保持关闭；本报告不宣称完整真实动力学验收。",
        "",
        "## 精确 18 项最终事实",
        "",
    ]
    for item in numbered_facts:
        number = item["number"]
        label = item["label"]
        value = item["value"]
        rendered = json.dumps(value, ensure_ascii=False, sort_keys=True) if isinstance(value, (dict, list)) else str(value)
        lines.append(f"{number}. {label}: `{rendered}`")
    lines.extend(
        [
            "",
            "## 四姿态 runtime world-pose A/B",
            "",
            "| 姿态 | q (rad) | visual Δposition norm (m) | visual max-abs component (m) | visual Δrotation (rad) | link6 body Δposition (m) | link6 body Δrotation (rad) | collision exact/count |",
            "|---|---|---:|---:|---:|---:|---:|---:|",
        ]
    )
    if isinstance(pose_rows, list):
        for row in pose_rows:
            if not isinstance(row, dict):
                continue
            lines.append(
                f"| `{row.get('id')}` | `{json.dumps(row.get('joint_position_rad'), ensure_ascii=False)}` | "
                f"{row.get('visual_runtime_position_delta_m')} | "
                f"{row.get('visual_runtime_position_max_abs_component_delta_m')} | "
                f"{row.get('visual_runtime_rotation_delta_rad')} | "
                f"{row.get('link6_body_runtime_position_delta_m')} | "
                f"{row.get('link6_body_runtime_rotation_delta_rad')} | "
                f"{row.get('collision_geom_exact_count')}/{row.get('collision_geom_count')} |"
            )
    lines.extend(
        [
            "",
            "## sameframe / compiled-local 诊断",
            "",
            f"- OLD / NEW `geom_sameframe`：`{storage.get('old_geom_sameframe')}` / `{storage.get('new_geom_sameframe')}`",
            f"- compiled local `geom_pos` delta：`{storage.get('compiled_geom_pos_delta_m')}` m",
            f"- compiled local `geom_quat` rotation delta：`{storage.get('compiled_geom_rotation_delta_rad')}` rad",
            f"- compiled body `ipos` delta：`{storage.get('compiled_body_ipos_delta_m')}` m",
            f"- compiled body `iquat` rotation delta：`{storage.get('compiled_body_iquat_rotation_delta_rad')}` rad",
            "- 上述 compiled-local 字段仅用于解释 sameframe 存储分类，未作为 runtime world-pose 验收量。",
            "",
            "## Runtime maxima 与视觉尺度",
            "",
            f"- visual max position / rotation：`{maxima.get('visual_position_delta_m')}` m / `{maxima.get('visual_rotation_delta_rad')}` rad",
            f"- link6 body max position / rotation：`{maxima.get('link6_body_position_delta_m')}` m / `{maxima.get('link6_body_rotation_delta_rad')}` rad",
            f"- joint anchor / axis max：`{maxima.get('joint_anchor_position_delta_m')}` m / `{maxima.get('joint_axis_vector_delta')}`",
            f"- TCP/camera TF max position / rotation：`{maxima.get('tf_position_delta_m')}` m / `{maxima.get('tf_rotation_delta_rad')}` rad",
            f"- collision max position / rotation：`{maxima.get('collision_geom_position_delta_m')}` m / `{maxima.get('collision_geom_rotation_delta_rad')}` rad",
            f"- collision-active（961 active `collision__*` + ground，共 962/pose）exact：`{collision.get('exact_count')}/{collision.get('comparison_count')}`；all exact=`{collision.get('all_exact')}`",
            f"- 全部 963 个 `collision__*` tagged geoms（含两个 contype=conaffinity=0 UpperArm motion geoms，diagnostic）exact：`{collision.get('all_collision_tagged_diagnostic', {}).get('exact_count')}/{collision.get('all_collision_tagged_diagnostic', {}).get('comparison_count')}`",
            f"- target visual 之外全部 1008 named geoms exact：`{non_target.get('exact_count')}/{non_target.get('comparison_count')}`；all exact=`{non_target.get('all_exact')}`",
            f"- visual max Euclidean delta：`{scale.get('maximum_runtime_position_delta_micrometre')}` μm",
            f"- visual max-abs component delta（与 V15.17B 的 0.304 μm 标量同口径）：`{scale.get('maximum_runtime_position_max_abs_component_delta_micrometre')}` μm",
            f"- max-abs component 相对 visual bbox diagonal：`{scale.get('max_abs_component_relative_to_bbox_diagonal')}`（`{scale.get('max_abs_component_parts_per_million_of_bbox_diagonal')}` ppm）",
            f"- max-abs component 相对 visual bbox maximum dimension：`{scale.get('max_abs_component_relative_to_bbox_maximum_dimension')}`",
        ]
    )
    hashes = report.get("authorities", {}).get("source_hashes_sha256", {})
    history = report.get("v15_17b_frozen_history", {})
    diagnostic_inputs = report.get("v15_17c_diagnostic_inputs", {})
    lines.extend(
        [
            "",
            "## Authority 与冻结历史",
            "",
            "| Authority / history | SHA256 |",
            "|---|---|",
            f"| Mass V1 | `{hashes.get(MASS_REL)}` |",
            f"| COM V2 | `{hashes.get(COM_REL)}` |",
            f"| Inertia Engineering V1 | `{hashes.get(INERTIA_REL)}` |",
            f"| V15.17B JSON | `{history.get('source_hashes_sha256', {}).get(V15_17B_REPORT_JSON_REL)}` |",
            f"| V15.17B Markdown | `{history.get('source_hashes_sha256', {}).get(V15_17B_REPORT_MD_REL)}` |",
            f"| V15.14 accepted FJT evidence | `{diagnostic_inputs.get('source_hashes_sha256', {}).get(V15_17C_FJT_EVIDENCE_REL)}` |",
            f"| Runtime mesh manifest | `{diagnostic_inputs.get('source_hashes_sha256', {}).get(RUNTIME_MESH_MANIFEST_REL)}` |",
            f"| link6 visual mesh | `{diagnostic_inputs.get('source_hashes_sha256', {}).get(LINK6_VISUAL_MESH_REL)}` |",
            "",
            f"- 六体 frozen tensor compiled reconstruction：`{report.get('six_body_tensor_reconstruction_pass')}`",
            f"- production hash migration pending：`{report.get('production_hash_migration_pending')}`",
            "",
            "## Hard unresolved items",
            "",
        ]
    )
    if unresolved:
        lines.extend(f"- `{item}`" for item in unresolved)
    else:
        lines.append("- `[]`")
    lines.extend(
        [
            "",
            "## 模型边界",
            "",
            "旧 fullinertia 模型仅用于同解释器 A/B runtime world-pose 诊断，不是 physics authority。V15.17C 未修改 inertia、MJCF、Xacro/URDF、geometry/mesh、production bridge、controller、armature、friction 或 damping；未执行 hash migration，未开启重力。V15.17B JSON/Markdown 保留为冻结历史证据。",
            "",
        ]
    )
    return "\n".join(lines)


def write_reports(report: Mapping[str, Any]) -> None:
    atomic_write_bytes(repo_path(REPORT_JSON_REL), json_bytes(report))
    atomic_write_bytes(repo_path(REPORT_MD_REL), markdown_text(report).encode("utf-8"))


def atomic_write_bytes(path: Path, payload: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=path.name + ".",
        suffix=".tmp",
        dir=str(path.parent),
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def check_reports(report: Mapping[str, Any]) -> None:
    expected_json = json_bytes(report)
    expected_md = markdown_text(report).encode("utf-8")
    json_path = repo_path(REPORT_JSON_REL)
    md_path = repo_path(REPORT_MD_REL)
    require(json_path.is_file() and md_path.is_file(), "validation reports are missing; run --write")
    require(json_path.read_bytes() == expected_json, f"stale report: {REPORT_JSON_REL}")
    require(md_path.read_bytes() == expected_md, f"stale report: {REPORT_MD_REL}")


def main() -> int:
    if len(sys.argv) >= 2 and sys.argv[1] == "--_urdf-child":
        require(len(sys.argv) == 4, "invalid URDF child arguments")
        return urdf_child(Path(sys.argv[2]), Path(sys.argv[3]))
    if len(sys.argv) >= 2 and sys.argv[1] == "--_mujoco-child":
        require(len(sys.argv) == 6, "invalid MuJoCo child arguments")
        return mujoco_child(Path(sys.argv[2]), Path(sys.argv[3]), Path(sys.argv[4]), Path(sys.argv[5]))

    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--write", action="store_true", help="regenerate deterministic JSON/Markdown reports")
    mode.add_argument("--check", action="store_true", help="require byte-identical current reports")
    parser.add_argument("--urdf-python", help="Python executable containing xacro and urdf_parser_py")
    parser.add_argument("--mujoco-python", help="Python executable containing mujoco")
    parser.add_argument(
        "--wsl-distro",
        default=os.environ.get("V15_17_WSL_DISTRO"),
        help="optional WSL distro used only for parser/compiler child processes",
    )
    parser.add_argument(
        "--runtime-evidence-json",
        type=Path,
        default=(Path(os.environ["V15_17_RUNTIME_EVIDENCE_JSON"]) if os.environ.get("V15_17_RUNTIME_EVIDENCE_JSON") else None),
        help="repo-external, hash-bound ROS/trajectory/collision evidence envelope",
    )
    arguments = parser.parse_args()

    outer_watch = sorted(
        set(
            [
                MASS_REL,
                COM_REL,
                INERTIA_REL,
                XACRO_REL,
                MJCF_REL,
                APPLY_REL,
                VALIDATOR_REL,
                V15_17B_REPORT_JSON_REL,
                V15_17B_REPORT_MD_REL,
                *EXPECTED_V15_17C_DIAGNOSTIC_INPUT_SHA256,
                *V14_PROTECTED_REL,
                *ENGINEERING_PROTECTED_REL,
                *(() if arguments.write else (REPORT_JSON_REL, REPORT_MD_REL)),
            ]
        )
    )
    outer_before = capture_hashes(outer_watch)

    report = build_report(
        arguments.urdf_python,
        arguments.mujoco_python,
        arguments.runtime_evidence_json,
        arguments.wsl_distro,
    )
    if arguments.write:
        write_reports(report)
        # The first write introduces the two report paths into git scope.  Build
        # once more so exact-eight and TOCTOU fields describe the final tree.
        report = build_report(
            arguments.urdf_python,
            arguments.mujoco_python,
            arguments.runtime_evidence_json,
            arguments.wsl_distro,
        )
        write_reports(report)
    else:
        check_reports(report)
    outer_after = capture_hashes(outer_watch)
    require(outer_before == outer_after, "outer TOCTOU guard detected a changed non-output input")
    final_scope = validate_git_and_scope()
    require(final_scope.get("pass") is True, "final exact-eight git scope revalidation failed")
    print(
        json.dumps(
            {
                "audit_valid": report["audit_valid"],
                "final_status": report["final_status"],
                "hard_unresolved_items": report["hard_unresolved_items"],
            },
            ensure_ascii=False,
            indent=2,
        )
    )
    # A byte-identical --check (or completed atomic --write) is a successful
    # audit even when the deployment itself is truthfully FAIL.  Validation or
    # stale-report errors raise above and are converted to exit code 2.
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (ValidationError, subprocess.TimeoutExpired) as error:
        print(f"VALIDATION_ERROR: {error}", file=sys.stderr)
        raise SystemExit(2)

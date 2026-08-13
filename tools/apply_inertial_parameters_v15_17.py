#!/usr/bin/env python3
from __future__ import annotations

"""Deterministically deploy the frozen V15.17B MuJoCo inertial parameters.

The three JSON authorities are the only numerical data sources.  This tool
keeps the accepted V15.17 URDF byte-for-byte unchanged and performs a minimal
six-line MJCF migration from ``fullinertia`` to explicit principal-frame
``quat`` plus ``diaginertia``.  Nothing is recomputed from CAD and no
dynamics/control setting is changed.
"""

import argparse
import hashlib
import json
import math
import os
from dataclasses import dataclass, replace
from pathlib import Path
import re
import subprocess
import sys
import xml.etree.ElementTree as ET

import mujoco  # type: ignore
import numpy as np


ROOT = Path(__file__).resolve().parents[1]
TARGET_LINKS = ("link2", "link3", "link4", "link5", "link6", "gripper")

MASS_AUTHORITY = ROOT / "V15_15_实测质量账本_v1.json"
COM_AUTHORITY = ROOT / "V15_15_COM账本_v2.json"
INERTIA_AUTHORITY = ROOT / "V15_16_刚体惯量_Engineering_V1.json"
XACRO_PATH = (
    ROOT
    / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
    / "ros2_ws"
    / "src"
    / "go_m8010_arm_v15_14_description"
    / "urdf"
    / "go_m8010_arm_v15_14.urdf.xacro"
)
XACRO_RELATIVE_PATH = (
    "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/"
    "ros2_ws/src/go_m8010_arm_v15_14_description/urdf/"
    "go_m8010_arm_v15_14.urdf.xacro"
)
MJCF_PATH = (
    ROOT
    / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
    / "mujoco_v15_14"
    / "go_m8010_arm_v15_14_kinematic.xml"
)
MJCF_RELATIVE_PATH = (
    "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/"
    "mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
)

LOCKED_AUTHORITY_SHA256 = {
    MASS_AUTHORITY: "658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a",
    COM_AUTHORITY: "1cb975890dd121eb41d41bfee414a2c4643f4d917089970fac60009736fc77ae",
    INERTIA_AUTHORITY: "c6c398532d7144ed6aa340a8d8b765b333d01f1fddc2e3280106c90565614401",
}

BASELINE_XACRO_SHA256 = (
    "9196daa600d82b07ada259044df4c815b2a824a7658ab7647ac56c4e3c05c968"
)
BASELINE_MJCF_SHA256 = (
    "ce6ec828d32f445ff25fb4b3ba3167f7f0bec61cdfca5962ee4072df4b022844"
)
SOURCE_BLOB_XACRO_SHA256 = (
    "f9c15d366e27bef76eeafd44efcca0de96d2f4e944584fcc74c9764a198a914c"
)
SOURCE_BLOB_MJCF_SHA256 = (
    "f086d27c61aa3be02234356af6fd8fcf269a3e3b65bdeb08cd04435ba679f0b0"
)

# Exact V15.17A input and V15.17B output locks.  V15.17B treats the already
# accepted URDF as read-only and migrates only the MJCF representation.
DEPLOYED_XACRO_SHA256 = (
    "1ba9d2ec1126371f676a55a33c05c61a5bc416651e963e350f28662cfd141652"
)
FULLINERTIA_MJCF_SHA256 = (
    "e43b96deb54b4961490b5c21c1052080c4eda2e03abac04ce1f47c8938934716"
)
DEPLOYED_MJCF_SHA256 = (
    "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
)

EXPECTED_SYNTHETIC_FULLINERTIA_RELATIVE_ERROR = 1.0258546895235342e-06
PYTHON_RECONSTRUCTION_RELATIVE_TOLERANCE = 1.0e-14
SYNTHETIC_PRINCIPAL_RELATIVE_TOLERANCE = 1.0e-12
COMPILED_TENSOR_RELATIVE_HARD_TOLERANCE = 1.0e-9
ROTATION_ABSOLUTE_TOLERANCE = 1.0e-12
LOCKED_NUMPY_VERSION = "2.5.2"
LOCKED_MUJOCO_VERSION = "3.11.0"

EXPECTED_MASS_KG = {
    "link2": 0.7615,
    "link3": 1.09,
    "link4": 0.675,
    "link5": 0.504,
    "link6": 0.125,
    "gripper": 0.296,
}
EXPECTED_TOTAL_MASS_KG = 3.4515
ABS_TOLERANCE = 1.0e-12
SOURCE_COMMIT = "f4132c5514a67b5fc36ed63c7474b33a87fc4747"
SOURCE_REMOTE_REF = "refs/heads/agent/v15-16-inertia-engineering-v1"
TARGET_BRANCH = "agent/v15-17-inertial-deployment"
ALLOWED_DIRTY_PATHS = {
    XACRO_RELATIVE_PATH,
    MJCF_RELATIVE_PATH,
    "tools/apply_inertial_parameters_v15_17.py",
    # The root task may create these three peer outputs after this deployer.
    "tools/validate_inertial_deployment_v15_17.py",
    "V15_17_Inertial参数部署验收.json",
    "V15_17_Inertial参数部署验收.md",
    "V15_17_MuJoCo主惯量显式部署.json",
    "V15_17_MuJoCo主惯量显式部署.md",
}


@dataclass(frozen=True)
class InertialRecord:
    link: str
    mass_kg: float
    com_m: tuple[float, float, float]
    tensor_kg_m2: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]


@dataclass(frozen=True)
class PrincipalFrameRecord:
    source: InertialRecord
    moments_kg_m2: tuple[float, float, float]
    axes_body_from_inertial: tuple[
        tuple[float, float, float],
        tuple[float, float, float],
        tuple[float, float, float],
    ]
    quaternion_wxyz: tuple[float, float, float, float]
    symmetry_max_abs_error: float
    determinant: float
    orthonormality_max_abs_error: float
    python_reconstruction_relative_frobenius_error: float
    mujoco_quaternion_rotation_max_abs_error: float
    manual_quaternion_rotation_max_abs_error: float
    manual_vs_mujoco_rotation_max_abs_error: float


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def require(condition: bool, message: str) -> None:
    if not condition:
        raise RuntimeError(message)


def finite_float(value: object, label: str) -> float:
    require(type(value) in (int, float), f"{label} is not a JSON number")
    result = float(value)
    require(math.isfinite(result), f"{label} is not finite")
    return result


def require_close(actual: float, expected: float, label: str) -> None:
    require(
        abs(actual - expected) <= ABS_TOLERANCE,
        f"{label} mismatch: {actual!r} != {expected!r}",
    )


def require_empty_list(value: object, label: str) -> None:
    require(isinstance(value, list) and not value, f"{label} must be []")


def load_locked_json(path: Path) -> dict:
    require(path.is_file(), f"authority is missing: {path}")
    actual_sha256 = sha256_file(path)
    expected_sha256 = LOCKED_AUTHORITY_SHA256[path]
    require(
        actual_sha256 == expected_sha256,
        f"authority SHA256 mismatch: {path.name}: {actual_sha256} != {expected_sha256}",
    )
    document = json.loads(path.read_text(encoding="utf-8"))
    require(isinstance(document, dict), f"authority root is not an object: {path}")
    return document


def git_text(*arguments: str, timeout_seconds: float = 30.0) -> str:
    completed = subprocess.run(
        ["git", "-C", str(ROOT), *arguments],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        encoding="utf-8",
        errors="strict",
        timeout=timeout_seconds,
    )
    require(
        completed.returncode == 0,
        "git command failed: "
        + " ".join(arguments)
        + f": {completed.stderr.strip()}",
    )
    # Preserve the leading XY status columns and NUL delimiters of
    # ``git status --porcelain -z``; only textual command line endings are
    # removed here.
    return completed.stdout.rstrip("\r\n")


def git_bytes(*arguments: str, timeout_seconds: float = 30.0) -> bytes:
    completed = subprocess.run(
        ["git", "-C", str(ROOT), *arguments],
        check=False,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        timeout=timeout_seconds,
    )
    require(
        completed.returncode == 0,
        "git command failed: "
        + " ".join(arguments)
        + ": "
        + completed.stderr.decode("utf-8", errors="replace").strip(),
    )
    return completed.stdout


def load_git_source_baseline(
    relative_path: str, blob_sha256: str, worktree_sha256: str
) -> tuple[str, str]:
    payload = git_bytes("show", f"{SOURCE_COMMIT}:{relative_path}")
    require(
        sha256_bytes(payload) == blob_sha256,
        f"source commit blob SHA256 mismatch: {relative_path}",
    )
    text = payload.decode("utf-8", errors="strict")
    require("\r" not in text and text.endswith("\n"), f"source blob newline contract failed: {relative_path}")
    worktree_text = text.replace("\n", "\r\n")
    require(
        sha256_bytes(worktree_text.encode("utf-8")) == worktree_sha256,
        f"deterministic source-to-worktree baseline mismatch: {relative_path}",
    )
    return worktree_text, "\r\n"


def validate_git_contract() -> dict:
    require(
        Path(git_text("rev-parse", "--show-toplevel")).resolve() == ROOT,
        "script is not running in the expected git worktree",
    )
    branch = git_text("rev-parse", "--abbrev-ref", "HEAD")
    head = git_text("rev-parse", "HEAD")
    require(branch == TARGET_BRANCH, f"unexpected branch: {branch}")
    if head != SOURCE_COMMIT:
        head_and_parents = git_text("rev-list", "--parents", "-n", "1", "HEAD").split()
        require(
            len(head_and_parents) == 2 and head_and_parents[1] == SOURCE_COMMIT,
            f"target branch is not based directly on the frozen source commit: {head}",
        )

    remote_rows = git_text(
        "ls-remote", "origin", SOURCE_REMOTE_REF, timeout_seconds=60.0
    ).splitlines()
    require(len(remote_rows) == 1, f"remote source ref is missing/ambiguous: {remote_rows}")
    remote_fields = remote_rows[0].split()
    require(
        len(remote_fields) == 2
        and remote_fields[0] == SOURCE_COMMIT
        and remote_fields[1] == SOURCE_REMOTE_REF,
        f"remote source commit mismatch: {remote_rows[0]}",
    )

    status_payload = git_text("status", "--porcelain=v1", "-z")
    entries = status_payload.split("\0") if status_payload else []
    dirty_paths: set[str] = set()
    index = 0
    while index < len(entries):
        entry = entries[index]
        if not entry:
            index += 1
            continue
        require(len(entry) >= 4, f"cannot parse git status entry: {entry!r}")
        status = entry[:2]
        path = entry[3:].replace("\\", "/")
        dirty_paths.add(path)
        if "R" in status or "C" in status:
            index += 1
            require(index < len(entries), "git rename/copy status is incomplete")
            dirty_paths.add(entries[index].replace("\\", "/"))
        index += 1
    unexpected = sorted(dirty_paths - ALLOWED_DIRTY_PATHS)
    require(not unexpected, f"unexpected dirty git paths: {unexpected}")
    return {
        "branch": branch,
        "head": head,
        "source_commit": SOURCE_COMMIT,
        "source_is_head_or_direct_parent": True,
        "remote_ref": SOURCE_REMOTE_REF,
        "remote_commit": remote_fields[0],
        "source_blob_sha256": {
            XACRO_RELATIVE_PATH: SOURCE_BLOB_XACRO_SHA256,
            MJCF_RELATIVE_PATH: SOURCE_BLOB_MJCF_SHA256,
        },
        "dirty_paths": sorted(dirty_paths),
        "scope_pass": True,
    }


def load_authority_records() -> dict[str, InertialRecord]:
    mass = load_locked_json(MASS_AUTHORITY)
    com = load_locked_json(COM_AUTHORITY)
    inertia = load_locked_json(INERTIA_AUTHORITY)

    require(
        mass.get("schema") == "go-m8010-arm-v15.15-mass-ledger-v1/1.0"
        and mass.get("revision") == "V15.15-MASS_LEDGER_V1"
        and mass.get("final_status") == "V15.15 MASS_LEDGER_V1 = PASS",
        "mass authority identity/status gate failed",
    )
    require(
        mass.get("validation", {}).get("all_assertions_pass") is True,
        "mass authority validation is not PASS",
    )
    require_empty_list(mass.get("unresolved_items"), "mass unresolved_items")
    require_close(
        finite_float(
            mass.get("total_link2_to_gripper_nominal_mass_kg"),
            "mass total_link2_to_gripper_nominal_mass_kg",
        ),
        EXPECTED_TOTAL_MASS_KG,
        "mass authority total",
    )

    require(
        com.get("schema") == "go-m8010-arm-v15.15-com-ledger-volume-v2/1.0"
        and com.get("revision") == "V15.15-COM_LEDGER_VOLUME_V2"
        and com.get("final_status") == "V15.15 COM_LEDGER_V2 = PASS",
        "COM V2 authority identity/status gate failed",
    )
    require(
        com.get("units", {}).get("published_com") == "m"
        and com.get("validation", {}).get("all_assertions_pass") is True,
        "COM V2 units/validation gate failed",
    )
    require_empty_list(com.get("unresolved_items"), "COM V2 unresolved_items")

    require(
        inertia.get("schema")
        == "go-m8010-arm-v15.16-rigid-inertia-engineering-v1/1.0"
        and inertia.get("revision")
        == "V15.16-RIGID_BODY_INERTIA-ENGINEERING_V1"
        and inertia.get("final_status")
        == "V15.16 INERTIA_ENGINEERING_V1 = PASS"
        and inertia.get("status") == "ENGINEERING_V1",
        "inertia authority identity/status gate failed",
    )
    require(
        inertia.get("gravity_enabled") is False
        and inertia.get("collision_proxy_used") is False
        and inertia.get("deployment_performed") is False,
        "inertia authority scope gate failed",
    )
    require_empty_list(inertia.get("unresolved_items"), "inertia unresolved_items")
    require_close(
        finite_float(inertia.get("total_mass_kg"), "inertia total_mass_kg"),
        EXPECTED_TOTAL_MASS_KG,
        "inertia authority total",
    )
    require(
        inertia.get("authorities", {}).get("all_protected_inputs_unchanged") is True
        and inertia.get("authorities", {}).get("mass", {}).get("sha256")
        == LOCKED_AUTHORITY_SHA256[MASS_AUTHORITY]
        and inertia.get("authorities", {}).get("com_v2", {}).get("sha256")
        == LOCKED_AUTHORITY_SHA256[COM_AUTHORITY],
        "inertia authority provenance gate failed",
    )

    mass_rows = mass.get("link_mass_ledger")
    com_rows = com.get("links")
    inertia_rows_raw = inertia.get("links")
    require(isinstance(mass_rows, dict), "mass link_mass_ledger is not an object")
    require(isinstance(com_rows, dict), "COM V2 links is not an object")
    require(isinstance(inertia_rows_raw, list), "inertia links is not an array")
    require(set(mass_rows) == set(TARGET_LINKS), "mass link set is not exactly six")
    require(set(com_rows) == set(TARGET_LINKS), "COM V2 link set is not exactly six")

    inertia_rows: dict[str, dict] = {}
    for row in inertia_rows_raw:
        require(isinstance(row, dict), "inertia link row is not an object")
        link = row.get("link")
        require(link in TARGET_LINKS, f"unexpected inertia link: {link!r}")
        require(link not in inertia_rows, f"duplicate inertia link: {link}")
        inertia_rows[link] = row
    require(set(inertia_rows) == set(TARGET_LINKS), "inertia link set is not exactly six")

    records: dict[str, InertialRecord] = {}
    for link in TARGET_LINKS:
        expected_mass = EXPECTED_MASS_KG[link]
        mass_row = mass_rows[link]
        com_row = com_rows[link]
        inertia_row = inertia_rows[link]
        require(isinstance(mass_row, dict), f"mass row is invalid: {link}")
        require(isinstance(com_row, dict), f"COM row is invalid: {link}")
        require(
            mass_row.get("is_final_v1_nominal_mass") is True,
            f"mass row is not frozen final V1: {link}",
        )
        ledger_mass = finite_float(
            mass_row.get("nominal_mass_kg"), f"mass {link}.nominal_mass_kg"
        )
        com_mass = finite_float(com_row.get("mass_kg"), f"COM {link}.mass_kg")
        inertia_mass = finite_float(
            inertia_row.get("mass_kg"), f"inertia {link}.mass_kg"
        )
        for label, actual in (
            ("mass ledger", ledger_mass),
            ("COM V2", com_mass),
            ("inertia", inertia_mass),
        ):
            require_close(actual, expected_mass, f"{label} {link} mass")
        require(
            com_row.get("validation", {}).get("pass") is True,
            f"COM V2 link validation failed: {link}",
        )

        com_vector_raw = com_row.get("com_link_m")
        inertia_com_raw = inertia_row.get("com_xyz_m_in_link_frame")
        require(
            isinstance(com_vector_raw, list) and len(com_vector_raw) == 3,
            f"COM V2 vector is invalid: {link}",
        )
        require(
            isinstance(inertia_com_raw, list) and len(inertia_com_raw) == 3,
            f"inertia COM vector is invalid: {link}",
        )
        com_vector = tuple(
            finite_float(value, f"COM {link}[{index}]")
            for index, value in enumerate(com_vector_raw)
        )
        inertia_com = tuple(
            finite_float(value, f"inertia COM {link}[{index}]")
            for index, value in enumerate(inertia_com_raw)
        )
        for index, (actual, expected) in enumerate(zip(inertia_com, com_vector)):
            require_close(actual, expected, f"inertia/COM V2 {link}[{index}]")

        require(
            inertia_row.get("inertia_reference_point") == "FROZEN_COM_V2"
            and inertia_row.get("inertia_expressed_in") == "OWNER_LINK_FRAME"
            and inertia_row.get("confidence_class") == "ENGINEERING_V1",
            f"inertia owner-frame semantics failed: {link}",
        )
        tensor_raw = inertia_row.get("inertia_tensor_kg_m2")
        require(
            isinstance(tensor_raw, list)
            and len(tensor_raw) == 3
            and all(isinstance(row, list) and len(row) == 3 for row in tensor_raw),
            f"inertia tensor is not 3x3: {link}",
        )
        tensor = tuple(
            tuple(
                finite_float(value, f"inertia tensor {link}[{r}][{c}]")
                for c, value in enumerate(row)
            )
            for r, row in enumerate(tensor_raw)
        )
        symmetry_error = max(
            abs(tensor[r][c] - tensor[c][r]) for r in range(3) for c in range(3)
        )
        require(symmetry_error <= 1.0e-15, f"inertia tensor is asymmetric: {link}")
        checks = inertia_row.get("tensor_checks", {})
        for gate in (
            "pass",
            "finite_pass",
            "symmetry_pass",
            "positive_definite_pass",
            "principal_triangle_inequality_pass",
            "unit_audit_pass",
            "radius_of_gyration_sanity_pass",
        ):
            require(checks.get(gate) is True, f"inertia tensor gate failed: {link}.{gate}")

        records[link] = InertialRecord(
            link=link,
            mass_kg=ledger_mass,
            com_m=com_vector,
            tensor_kg_m2=tensor,
        )

    require_close(
        sum(record.mass_kg for record in records.values()),
        EXPECTED_TOTAL_MASS_KG,
        "six-link deployed mass sum",
    )
    return records


def require_numeric_runtime() -> None:
    require(
        np.__version__ == LOCKED_NUMPY_VERSION,
        f"NumPy version mismatch: {np.__version__} != {LOCKED_NUMPY_VERSION}",
    )
    require(
        mujoco.__version__ == LOCKED_MUJOCO_VERSION,
        f"MuJoCo version mismatch: {mujoco.__version__} != {LOCKED_MUJOCO_VERSION}",
    )


def canonicalize_quaternion_sign(quaternion: np.ndarray) -> np.ndarray:
    result = np.asarray(quaternion, dtype=np.float64).copy()
    require(result.shape == (4,), "quaternion must have four components")
    require(np.all(np.isfinite(result)), "quaternion contains a non-finite value")
    norm = float(np.linalg.norm(result))
    require(norm > 0.0, "quaternion has zero norm")
    result /= norm
    # q and -q are the same orientation.  Use the largest-magnitude component,
    # with NumPy's first-index tie break, to make the serialized sign unique.
    pivot = int(np.argmax(np.abs(result)))
    if result[pivot] < 0.0:
        result *= -1.0
    return result


def manual_quaternion_wxyz_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = canonicalize_quaternion_sign(quaternion)
    return np.asarray(
        [
            [
                1.0 - 2.0 * (y * y + z * z),
                2.0 * (x * y - z * w),
                2.0 * (x * z + y * w),
            ],
            [
                2.0 * (x * y + z * w),
                1.0 - 2.0 * (x * x + z * z),
                2.0 * (y * z - x * w),
            ],
            [
                2.0 * (x * z - y * w),
                2.0 * (y * z + x * w),
                1.0 - 2.0 * (x * x + y * y),
            ],
        ],
        dtype=np.float64,
    )


def mujoco_quaternion_wxyz_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    flat = np.empty(9, dtype=np.float64)
    mujoco.mju_quat2Mat(flat, np.asarray(quaternion, dtype=np.float64))
    return flat.reshape(3, 3)


def manual_rotation_matrix_to_quaternion_wxyz(matrix: np.ndarray) -> np.ndarray:
    rotation = np.asarray(matrix, dtype=np.float64)
    require(rotation.shape == (3, 3), "rotation matrix must be 3x3")
    trace = float(np.trace(rotation))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.asarray(
            [
                0.25 * scale,
                (rotation[2, 1] - rotation[1, 2]) / scale,
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[1, 0] - rotation[0, 1]) / scale,
            ],
            dtype=np.float64,
        )
    elif rotation[0, 0] > rotation[1, 1] and rotation[0, 0] > rotation[2, 2]:
        scale = math.sqrt(
            1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]
        ) * 2.0
        quaternion = np.asarray(
            [
                (rotation[2, 1] - rotation[1, 2]) / scale,
                0.25 * scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
            ],
            dtype=np.float64,
        )
    elif rotation[1, 1] > rotation[2, 2]:
        scale = math.sqrt(
            1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]
        ) * 2.0
        quaternion = np.asarray(
            [
                (rotation[0, 2] - rotation[2, 0]) / scale,
                (rotation[0, 1] + rotation[1, 0]) / scale,
                0.25 * scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
            ],
            dtype=np.float64,
        )
    else:
        scale = math.sqrt(
            1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]
        ) * 2.0
        quaternion = np.asarray(
            [
                (rotation[1, 0] - rotation[0, 1]) / scale,
                (rotation[0, 2] + rotation[2, 0]) / scale,
                (rotation[1, 2] + rotation[2, 1]) / scale,
                0.25 * scale,
            ],
            dtype=np.float64,
        )
    return canonicalize_quaternion_sign(quaternion)


def relative_frobenius_error(actual: np.ndarray, expected: np.ndarray) -> float:
    denominator = float(np.linalg.norm(expected, ord="fro"))
    require(denominator > 0.0, "tensor Frobenius norm is zero")
    return float(np.linalg.norm(actual - expected, ord="fro")) / denominator


def compute_principal_frame(record: InertialRecord) -> PrincipalFrameRecord:
    tensor = np.asarray(record.tensor_kg_m2, dtype=np.float64)
    require(tensor.shape == (3, 3), f"tensor is not 3x3: {record.link}")
    require(np.all(np.isfinite(tensor)), f"tensor is non-finite: {record.link}")
    symmetry_error = float(np.max(np.abs(tensor - tensor.T)))
    require(
        symmetry_error < 1.0e-12,
        f"tensor symmetry strict gate failed: {record.link}: {symmetry_error}",
    )

    moments, axes = np.linalg.eigh(tensor)
    order = np.argsort(moments, kind="stable")
    moments = np.asarray(moments[order], dtype=np.float64)
    axes = np.asarray(axes[:, order], dtype=np.float64)
    require(
        bool(np.all(moments[:-1] <= moments[1:])),
        f"principal moments are not ascending: {record.link}",
    )
    require(bool(np.all(moments > 0.0)), f"principal moment is non-positive: {record.link}")
    require(
        float(moments[0] + moments[1] - moments[2]) >= -1.0e-12,
        f"principal triangle inequality failed: {record.link}",
    )

    # Eigenvector signs are mathematically arbitrary.  Canonicalize every
    # column before the one-column right-handed correction so identical
    # LAPACK output cannot serialize as an arbitrary q/-q or reflected frame.
    for column in range(3):
        pivot = int(np.argmax(np.abs(axes[:, column])))
        if axes[pivot, column] < 0.0:
            axes[:, column] *= -1.0
    if float(np.linalg.det(axes)) < 0.0:
        axes[:, 2] *= -1.0

    determinant = float(np.linalg.det(axes))
    orthonormality_error = float(np.max(np.abs(axes.T @ axes - np.eye(3))))
    require(determinant > 0.0, f"principal frame is left-handed: {record.link}")
    require(
        abs(determinant - 1.0) < ROTATION_ABSOLUTE_TOLERANCE,
        f"principal frame determinant gate failed: {record.link}: {determinant}",
    )
    require(
        orthonormality_error < ROTATION_ABSOLUTE_TOLERANCE,
        f"principal frame orthonormality gate failed: {record.link}: {orthonormality_error}",
    )

    reconstructed = axes @ np.diag(moments) @ axes.T
    python_error = relative_frobenius_error(reconstructed, tensor)
    require(
        python_error < PYTHON_RECONSTRUCTION_RELATIVE_TOLERANCE,
        f"Python eigh reconstruction gate failed: {record.link}: {python_error}",
    )

    mujoco_quaternion = np.empty(4, dtype=np.float64)
    mujoco.mju_mat2Quat(mujoco_quaternion, np.ascontiguousarray(axes.reshape(9)))
    mujoco_quaternion = canonicalize_quaternion_sign(mujoco_quaternion)
    manual_quaternion = manual_rotation_matrix_to_quaternion_wxyz(axes)
    mujoco_rotation = mujoco_quaternion_wxyz_to_matrix(mujoco_quaternion)
    manual_rotation = manual_quaternion_wxyz_to_matrix(manual_quaternion)
    mujoco_error = float(np.max(np.abs(mujoco_rotation - axes)))
    manual_error = float(np.max(np.abs(manual_rotation - axes)))
    cross_error = float(np.max(np.abs(manual_rotation - mujoco_rotation)))
    for label, error in (
        ("mju_mat2Quat", mujoco_error),
        ("manual matrix-to-quaternion", manual_error),
        ("manual/mju rotation agreement", cross_error),
    ):
        require(
            error < ROTATION_ABSOLUTE_TOLERANCE,
            f"{label} gate failed: {record.link}: {error}",
        )

    return PrincipalFrameRecord(
        source=record,
        moments_kg_m2=tuple(float(value) for value in moments),
        axes_body_from_inertial=tuple(
            tuple(float(value) for value in axes[row]) for row in range(3)
        ),
        quaternion_wxyz=tuple(float(value) for value in mujoco_quaternion),
        symmetry_max_abs_error=symmetry_error,
        determinant=determinant,
        orthonormality_max_abs_error=orthonormality_error,
        python_reconstruction_relative_frobenius_error=python_error,
        mujoco_quaternion_rotation_max_abs_error=mujoco_error,
        manual_quaternion_rotation_max_abs_error=manual_error,
        manual_vs_mujoco_rotation_max_abs_error=cross_error,
    )


def compute_principal_frames(
    records: dict[str, InertialRecord],
) -> dict[str, PrincipalFrameRecord]:
    return {link: compute_principal_frame(records[link]) for link in TARGET_LINKS}


def read_utf8_preserving_newline(path: Path) -> tuple[str, str]:
    require(path.is_file(), f"model file is missing: {path}")
    payload = path.read_bytes()
    require(not payload.startswith(b"\xef\xbb\xbf"), f"UTF-8 BOM is not accepted: {path}")
    text = payload.decode("utf-8", errors="strict")
    crlf_count = text.count("\r\n")
    bare_lf_count = text.replace("\r\n", "").count("\n")
    require(
        not (crlf_count and bare_lf_count),
        f"mixed newline styles are not accepted: {path}",
    )
    newline = "\r\n" if crlf_count else "\n"
    require(text.endswith(newline), f"model file must end with a newline: {path}")
    return text, newline


def format_number(value: float) -> str:
    value = finite_float(value, "deployment number")
    if value == 0.0:
        return "0"
    return format(value, ".17g")


def source_fullinertia(record: InertialRecord) -> tuple[float, ...]:
    matrix = record.tensor_kg_m2
    return (
        matrix[0][0],
        matrix[1][1],
        matrix[2][2],
        matrix[0][1],
        matrix[0][2],
        matrix[1][2],
    )


def principal_inertial_attributes(principal: PrincipalFrameRecord) -> str:
    source = principal.source
    pos = " ".join(format_number(value) for value in source.com_m)
    quaternion = " ".join(
        format_number(value) for value in principal.quaternion_wxyz
    )
    diagonal = " ".join(
        format_number(value) for value in principal.moments_kg_m2
    )
    return (
        f'pos="{pos}" quat="{quaternion}" '
        f'mass="{format_number(source.mass_kg)}" diaginertia="{diagonal}"'
    )


def compiled_inertial_readback(
    model: object,
    body_name: str,
    expected: InertialRecord,
    input_rotation: np.ndarray | None,
) -> dict:
    body_id = int(
        mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, body_name)
    )
    require(body_id >= 0, f"compiled body is missing: {body_name}")
    mass = float(model.body_mass[body_id])
    com = np.asarray(model.body_ipos[body_id], dtype=np.float64)
    quaternion = np.asarray(model.body_iquat[body_id], dtype=np.float64)
    moments = np.asarray(model.body_inertia[body_id], dtype=np.float64)
    require(
        model.body_mass.dtype == np.dtype("float64")
        and model.body_ipos.dtype == np.dtype("float64")
        and model.body_iquat.dtype == np.dtype("float64")
        and model.body_inertia.dtype == np.dtype("float64"),
        f"compiled MuJoCo arrays are not float64: {body_name}",
    )
    rotation = mujoco_quaternion_wxyz_to_matrix(quaternion)
    tensor = rotation @ np.diag(moments) @ rotation.T
    expected_tensor = np.asarray(expected.tensor_kg_m2, dtype=np.float64)
    mass_error = abs(mass - expected.mass_kg)
    com_error = float(np.linalg.norm(com - np.asarray(expected.com_m)))
    tensor_error = relative_frobenius_error(tensor, expected_tensor)
    rotation_error = (
        None
        if input_rotation is None
        else float(np.max(np.abs(rotation - input_rotation)))
    )
    return {
        "mass_kg": mass,
        "body_ipos_m": [float(value) for value in com],
        "body_iquat_wxyz": [float(value) for value in quaternion],
        "body_inertia_principal_kg_m2": [float(value) for value in moments],
        "reconstructed_tensor_kg_m2": [
            [float(value) for value in tensor[row]] for row in range(3)
        ],
        "mass_error_kg": mass_error,
        "com_error_m": com_error,
        "tensor_relative_frobenius_error": tensor_error,
        "compiled_vs_input_rotation_max_abs_error": rotation_error,
    }


def validate_synthetic_link2(
    principal: PrincipalFrameRecord,
) -> dict:
    source = principal.source
    fullinertia = " ".join(
        format_number(value) for value in source_fullinertia(source)
    )
    fullinertia_xml = (
        '<mujoco><worldbody><body name="test"><freejoint/>'
        f'<inertial pos="0.01 0.02 0.03" mass="{format_number(source.mass_kg)}" '
        f'fullinertia="{fullinertia}"/>'
        "</body></worldbody></mujoco>"
    )
    synthetic_source = InertialRecord(
        link="link2_synthetic",
        mass_kg=source.mass_kg,
        com_m=(0.01, 0.02, 0.03),
        tensor_kg_m2=source.tensor_kg_m2,
    )
    fullinertia_model = mujoco.MjModel.from_xml_string(fullinertia_xml)
    test_a = compiled_inertial_readback(
        fullinertia_model, "test", synthetic_source, None
    )
    fullinertia_error = float(test_a["tensor_relative_frobenius_error"])
    require(
        fullinertia_error > COMPILED_TENSOR_RELATIVE_HARD_TOLERANCE
        and abs(
            fullinertia_error - EXPECTED_SYNTHETIC_FULLINERTIA_RELATIVE_ERROR
        )
        < 1.0e-12,
        "synthetic TEST A no longer reproduces the locked MuJoCo 3.11.0 "
        f"fullinertia regression: {fullinertia_error}",
    )

    synthetic_principal = replace(principal, source=synthetic_source)
    principal_xml = (
        '<mujoco><worldbody><body name="test"><freejoint/>'
        f"<inertial {principal_inertial_attributes(synthetic_principal)}/>"
        "</body></worldbody></mujoco>"
    )
    principal_model = mujoco.MjModel.from_xml_string(principal_xml)
    input_rotation = np.asarray(principal.axes_body_from_inertial, dtype=np.float64)
    test_b = compiled_inertial_readback(
        principal_model, "test", synthetic_source, input_rotation
    )
    principal_error = float(test_b["tensor_relative_frobenius_error"])
    compiled_rotation_error = float(
        test_b["compiled_vs_input_rotation_max_abs_error"]
    )
    require(
        principal_error < SYNTHETIC_PRINCIPAL_RELATIVE_TOLERANCE,
        f"synthetic TEST B principal tensor gate failed: {principal_error}",
    )
    require(
        compiled_rotation_error < ROTATION_ABSOLUTE_TOLERANCE,
        f"synthetic TEST B compiled quaternion gate failed: {compiled_rotation_error}",
    )
    return {
        "pass": True,
        "test_a_fullinertia": test_a,
        "test_a_expected_relative_error": EXPECTED_SYNTHETIC_FULLINERTIA_RELATIVE_ERROR,
        "test_b_explicit_principal": test_b,
        "test_b_required_relative_error_strict_lt": SYNTHETIC_PRINCIPAL_RELATIVE_TOLERANCE,
    }


def validate_minimal_six_body_compiled_readback(
    principals: dict[str, PrincipalFrameRecord],
) -> dict:
    body_rows = []
    for link in TARGET_LINKS:
        body_rows.append(
            f'<body name="{link}"><freejoint/><inertial '
            f'{principal_inertial_attributes(principals[link])}/></body>'
        )
    xml = (
        '<mujoco><option gravity="0 0 0"/><worldbody>'
        + "".join(body_rows)
        + "</worldbody></mujoco>"
    )
    model = mujoco.MjModel.from_xml_string(xml)
    rows: dict[str, dict] = {}
    for link in TARGET_LINKS:
        principal = principals[link]
        row = compiled_inertial_readback(
            model,
            link,
            principal.source,
            np.asarray(principal.axes_body_from_inertial, dtype=np.float64),
        )
        require(
            float(row["mass_error_kg"]) < ABS_TOLERANCE,
            f"minimal compiled mass gate failed: {link}: {row['mass_error_kg']}",
        )
        require(
            float(row["com_error_m"]) < ABS_TOLERANCE,
            f"minimal compiled COM gate failed: {link}: {row['com_error_m']}",
        )
        require(
            float(row["compiled_vs_input_rotation_max_abs_error"])
            < ROTATION_ABSOLUTE_TOLERANCE,
            "minimal compiled quaternion rotation gate failed: "
            f"{link}: {row['compiled_vs_input_rotation_max_abs_error']}",
        )
        require(
            float(row["tensor_relative_frobenius_error"])
            < COMPILED_TENSOR_RELATIVE_HARD_TOLERANCE,
            "minimal compiled tensor hard gate failed: "
            f"{link}: {row['tensor_relative_frobenius_error']}",
        )
        rows[link] = row
    return {
        "pass": True,
        "method": "MINIMAL_SIX_FREE_BODIES_EXACT_SERIALIZED_INERTIAL_ATTRIBUTES",
        "links": rows,
        "max_mass_error_kg": max(float(row["mass_error_kg"]) for row in rows.values()),
        "max_com_error_m": max(float(row["com_error_m"]) for row in rows.values()),
        "max_compiled_rotation_error": max(
            float(row["compiled_vs_input_rotation_max_abs_error"])
            for row in rows.values()
        ),
        "max_tensor_relative_frobenius_error": max(
            float(row["tensor_relative_frobenius_error"])
            for row in rows.values()
        ),
        "tensor_relative_hard_gate_strict_lt": COMPILED_TENSOR_RELATIVE_HARD_TOLERANCE,
    }


def urdf_fragment(record: InertialRecord, indent: str, newline: str) -> list[str]:
    x, y, z = (format_number(value) for value in record.com_m)
    matrix = record.tensor_kg_m2
    attributes = {
        "ixx": matrix[0][0],
        "ixy": matrix[0][1],
        "ixz": matrix[0][2],
        "iyy": matrix[1][1],
        "iyz": matrix[1][2],
        "izz": matrix[2][2],
    }
    inertia_attributes = " ".join(
        f'{name}="{format_number(value)}"' for name, value in attributes.items()
    )
    return [
        f"{indent}<inertial>{newline}",
        f'{indent}  <origin xyz="{x} {y} {z}" rpy="0 0 0" />{newline}',
        f'{indent}  <mass value="{format_number(record.mass_kg)}" />{newline}',
        f"{indent}  <inertia {inertia_attributes} />{newline}",
        f"{indent}</inertial>{newline}",
    ]


def deploy_urdf_text(
    text: str, newline: str, records: dict[str, InertialRecord]
) -> str:
    for link in TARGET_LINKS:
        lines = text.splitlines(keepends=True)
        opening_matches = [
            index
            for index, line in enumerate(lines)
            if re.fullmatch(
                rf'(?P<indent>[ \t]*)<link name="{re.escape(link)}">\r?\n?', line
            )
        ]
        require(len(opening_matches) == 1, f"URDF link occurrence is not unique: {link}")
        opening_index = opening_matches[0]
        opening_text = lines[opening_index].rstrip("\r\n")
        indent = opening_text[: len(opening_text) - len(opening_text.lstrip())]
        child_indent = indent + "  "
        closing_index = next(
            (
                index
                for index in range(opening_index + 1, len(lines))
                if lines[index].rstrip("\r\n") == f"{indent}</link>"
            ),
            None,
        )
        require(closing_index is not None, f"URDF link is not closed: {link}")

        inertial_starts = [
            index
            for index in range(opening_index + 1, closing_index)
            if lines[index].startswith(f"{child_indent}<inertial")
        ]
        require(len(inertial_starts) <= 1, f"multiple direct URDF inertials: {link}")
        replacement = urdf_fragment(records[link], child_indent, newline)
        if inertial_starts:
            start = inertial_starts[0]
            stripped = lines[start].strip()
            if stripped.endswith("/>"):
                end = start
            else:
                end = next(
                    (
                        index
                        for index in range(start + 1, closing_index)
                        if lines[index].rstrip("\r\n")
                        == f"{child_indent}</inertial>"
                    ),
                    None,
                )
                require(end is not None, f"URDF inertial is not closed: {link}")
            lines[start : end + 1] = replacement
        else:
            lines[opening_index + 1 : opening_index + 1] = replacement
        text = "".join(lines)
    return text


def mjcf_inertial_line(
    principal: PrincipalFrameRecord, indent: str, newline: str
) -> str:
    return f"{indent}<inertial {principal_inertial_attributes(principal)} />{newline}"


def deploy_mjcf_text(
    text: str, newline: str, principals: dict[str, PrincipalFrameRecord]
) -> str:
    for link in TARGET_LINKS:
        lines = text.splitlines(keepends=True)
        opening_matches = [
            index
            for index, line in enumerate(lines)
            if re.match(
                rf'^[ \t]*<body name="{re.escape(link)}"(?:[ \t]|>)', line
            )
        ]
        require(len(opening_matches) == 1, f"MJCF body occurrence is not unique: {link}")
        opening_index = opening_matches[0]
        opening_text = lines[opening_index].rstrip("\r\n")
        indent = opening_text[: len(opening_text) - len(opening_text.lstrip())]
        child_indent = indent + "  "
        inertial_matches = [
            index
            for index in range(opening_index + 1, len(lines))
            if lines[index].startswith(f"{child_indent}<inertial ")
        ]
        # A nested body uses deeper indentation, so only the direct child can match.
        require(len(inertial_matches) == 1, f"direct MJCF inertial is not unique: {link}")
        index = inertial_matches[0]
        require(lines[index].strip().endswith("/>"), f"MJCF inertial is not empty: {link}")
        lines[index] = mjcf_inertial_line(principals[link], child_indent, newline)
        text = "".join(lines)
    return text


def parse_vector(text: str | None, length: int, label: str) -> tuple[float, ...]:
    require(text is not None, f"missing XML vector: {label}")
    fields = text.split()
    require(len(fields) == length, f"wrong XML vector length: {label}")
    return tuple(finite_float(float(field), f"{label}[{index}]") for index, field in enumerate(fields))


def validate_deployed_urdf(
    text: str, records: dict[str, InertialRecord]
) -> None:
    root = ET.fromstring(text)
    links = {row.get("name"): row for row in root.findall("link")}
    require(len(links) == len(root.findall("link")), "URDF contains duplicate link names")
    require(len(root.findall(".//inertial")) == 6, "URDF must contain exactly six inertials")
    for link in TARGET_LINKS:
        node = links.get(link)
        require(node is not None, f"URDF target link missing: {link}")
        inertials = node.findall("inertial")
        require(len(inertials) == 1, f"URDF target inertial count is not one: {link}")
        inertial = inertials[0]
        origin = inertial.find("origin")
        mass = inertial.find("mass")
        tensor = inertial.find("inertia")
        require(origin is not None and mass is not None and tensor is not None, f"URDF inertial structure is incomplete: {link}")
        require(origin.get("rpy") == "0 0 0", f"URDF inertial rpy changed: {link}")
        record = records[link]
        for index, (actual, expected) in enumerate(
            zip(parse_vector(origin.get("xyz"), 3, f"URDF {link} xyz"), record.com_m)
        ):
            require_close(actual, expected, f"URDF {link} COM[{index}]")
        require_close(
            finite_float(float(mass.get("value", "nan")), f"URDF {link} mass"),
            record.mass_kg,
            f"URDF {link} mass",
        )
        expected_attributes = {
            "ixx": record.tensor_kg_m2[0][0],
            "ixy": record.tensor_kg_m2[0][1],
            "ixz": record.tensor_kg_m2[0][2],
            "iyy": record.tensor_kg_m2[1][1],
            "iyz": record.tensor_kg_m2[1][2],
            "izz": record.tensor_kg_m2[2][2],
        }
        require(set(tensor.attrib) == set(expected_attributes), f"URDF inertia attributes changed: {link}")
        for name, expected in expected_attributes.items():
            require_close(
                finite_float(float(tensor.get(name, "nan")), f"URDF {link}.{name}"),
                expected,
                f"URDF {link}.{name}",
            )


def validate_deployed_mjcf(
    text: str, principals: dict[str, PrincipalFrameRecord]
) -> None:
    root = ET.fromstring(text)
    option = root.find("option")
    require(option is not None and option.get("gravity") == "0 0 0", "MJCF gravity is not frozen OFF")
    require(root.find("actuator") is None, "MJCF actuator block was unexpectedly added")
    bodies = {row.get("name"): row for row in root.findall(".//body")}
    require(len(bodies) == len(root.findall(".//body")), "MJCF contains duplicate body names")
    link1 = bodies.get("link1")
    require(link1 is not None, "MJCF link1 is missing")
    link1_inertials = link1.findall("inertial")
    require(
        len(link1_inertials) == 1
        and link1_inertials[0].attrib
        == {"pos": "0 0 0", "mass": "1", "diaginertia": "0.001 0.001 0.001"},
        "MJCF link1 placeholder inertial changed",
    )
    require(
        not root.findall(".//inertial[@fullinertia]"),
        "MJCF fullinertia must not remain after V15.17B deployment",
    )
    for joint_name in ("J1", "J2", "J3", "J4", "J5", "J6"):
        matches = root.findall(f'.//joint[@name="{joint_name}"]')
        require(len(matches) == 1, f"MJCF joint occurrence is not unique: {joint_name}")
        joint = matches[0]
        require(
            joint.get("damping") == "0"
            and joint.get("frictionloss") == "0"
            and joint.get("armature") == "0",
            f"MJCF control/dynamics field changed: {joint_name}",
        )
    for link in TARGET_LINKS:
        body = bodies.get(link)
        require(body is not None, f"MJCF target body missing: {link}")
        inertials = body.findall("inertial")
        require(len(inertials) == 1, f"MJCF target inertial count is not one: {link}")
        inertial = inertials[0]
        require(
            set(inertial.attrib) == {"pos", "quat", "mass", "diaginertia"},
            f"MJCF inertial attributes changed: {link}",
        )
        principal = principals[link]
        record = principal.source
        for index, (actual, expected) in enumerate(
            zip(parse_vector(inertial.get("pos"), 3, f"MJCF {link} pos"), record.com_m)
        ):
            require_close(actual, expected, f"MJCF {link} COM[{index}]")
        require_close(
            finite_float(float(inertial.get("mass", "nan")), f"MJCF {link} mass"),
            record.mass_kg,
            f"MJCF {link} mass",
        )
        quaternion = np.asarray(
            parse_vector(inertial.get("quat"), 4, f"MJCF {link} quat"),
            dtype=np.float64,
        )
        diagonal = np.asarray(
            parse_vector(
                inertial.get("diaginertia"), 3, f"MJCF {link} diaginertia"
            ),
            dtype=np.float64,
        )
        require(bool(np.all(diagonal > 0.0)), f"MJCF diaginertia is non-positive: {link}")
        for index, (actual, expected) in enumerate(
            zip(diagonal, principal.moments_kg_m2)
        ):
            require_close(
                float(actual), expected, f"MJCF {link} diaginertia[{index}]"
            )
        rotation = manual_quaternion_wxyz_to_matrix(quaternion)
        expected_rotation = np.asarray(
            principal.axes_body_from_inertial, dtype=np.float64
        )
        rotation_error = float(np.max(np.abs(rotation - expected_rotation)))
        require(
            rotation_error < ROTATION_ABSOLUTE_TOLERANCE,
            f"MJCF input quaternion rotation gate failed: {link}: {rotation_error}",
        )
        reconstructed = rotation @ np.diag(diagonal) @ rotation.T
        tensor_error = relative_frobenius_error(
            reconstructed, np.asarray(record.tensor_kg_m2, dtype=np.float64)
        )
        require(
            tensor_error < PYTHON_RECONSTRUCTION_RELATIVE_TOLERANCE,
            f"MJCF serialized principal tensor gate failed: {link}: {tensor_error}",
        )


def require_locked_model_input(
    path: Path, actual_sha256: str, allowed_sha256: set[str]
) -> None:
    require(
        actual_sha256 in allowed_sha256,
        f"model SHA256 is not an allowed frozen state: {path}: {actual_sha256}",
    )


def atomic_write(path: Path, payload: bytes) -> None:
    temporary = path.with_name(f".{path.name}.v15_17.tmp")
    require(not temporary.exists(), f"temporary output already exists: {temporary}")
    try:
        temporary.write_bytes(payload)
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()


def run(write: bool) -> int:
    git_contract = validate_git_contract()
    require_numeric_runtime()
    records = load_authority_records()
    principals = compute_principal_frames(records)
    synthetic = validate_synthetic_link2(principals["link2"])
    minimal_compiled = validate_minimal_six_body_compiled_readback(principals)

    xacro_text, xacro_newline = read_utf8_preserving_newline(XACRO_PATH)
    mjcf_text, mjcf_newline = read_utf8_preserving_newline(MJCF_PATH)
    current_xacro_sha256 = sha256_bytes(xacro_text.encode("utf-8"))
    current_mjcf_sha256 = sha256_bytes(mjcf_text.encode("utf-8"))
    require_locked_model_input(
        XACRO_PATH,
        current_xacro_sha256,
        {DEPLOYED_XACRO_SHA256},
    )
    require_locked_model_input(
        MJCF_PATH,
        current_mjcf_sha256,
        {BASELINE_MJCF_SHA256, FULLINERTIA_MJCF_SHA256, DEPLOYED_MJCF_SHA256},
    )

    source_mjcf_text, source_mjcf_newline = load_git_source_baseline(
        MJCF_RELATIVE_PATH, SOURCE_BLOB_MJCF_SHA256, BASELINE_MJCF_SHA256
    )
    require(
        xacro_newline == "\r\n",
        "read-only URDF newline style differs from its locked deployed bytes",
    )
    require(
        mjcf_newline == source_mjcf_newline,
        "MJCF worktree newline style differs from the frozen source transform",
    )
    # V15.17B never generates or writes the URDF.  Its exact deployed bytes are
    # a protected input, while structural parsing independently confirms the
    # six owner-frame tensors remain valid.
    validate_deployed_urdf(xacro_text, records)
    desired_mjcf = deploy_mjcf_text(
        source_mjcf_text, source_mjcf_newline, principals
    )
    validate_deployed_mjcf(desired_mjcf, principals)
    desired_mjcf_bytes = desired_mjcf.encode("utf-8")
    desired_mjcf_sha256 = sha256_bytes(desired_mjcf_bytes)
    mjcf_changed = desired_mjcf_sha256 != current_mjcf_sha256
    require(
        desired_mjcf_sha256 == DEPLOYED_MJCF_SHA256,
        "deterministic principal-frame MJCF output SHA256 mismatch: "
        f"{desired_mjcf_sha256} != {DEPLOYED_MJCF_SHA256}",
    )

    report = {
        "status": "NEEDS_WRITE" if mjcf_changed else "PASS",
        "mode": "write" if write else "check",
        "representation": "EXPLICIT_PRINCIPAL_FRAME_DIAGINERTIA",
        "reason": "BYPASS_MUJOCO_3_11_0_FULLINERTIA_EIGEN_DECOMPOSITION_PRECISION",
        "numeric_runtime": {
            "python": sys.version.split()[0],
            "numpy": np.__version__,
            "mujoco": mujoco.__version__,
            "dtype": "float64",
        },
        "git_contract": git_contract,
        "authorities": {
            path.name: digest for path, digest in LOCKED_AUTHORITY_SHA256.items()
        },
        "target_links": list(TARGET_LINKS),
        "total_mass_kg": EXPECTED_TOTAL_MASS_KG,
        "xacro": {
            "path": str(XACRO_PATH),
            "before_sha256": current_xacro_sha256,
            "locked_sha256": DEPLOYED_XACRO_SHA256,
            "read_only": True,
            "changed": False,
        },
        "mjcf": {
            "path": str(MJCF_PATH),
            "before_sha256": current_mjcf_sha256,
            "desired_sha256": desired_mjcf_sha256,
            "changed": mjcf_changed,
            "gravity": "0 0 0",
            "fullinertia_present_in_desired_six": False,
            "explicit_quat_diaginertia_count": 6,
        },
        "synthetic": synthetic,
        "minimal_six_body_compiled_readback": minimal_compiled,
        "principal_frames": {
            link: {
                "frozen_tensor_kg_m2": [
                    list(row) for row in principals[link].source.tensor_kg_m2
                ],
                "principal_moments_kg_m2": list(
                    principals[link].moments_kg_m2
                ),
                "principal_axes_body_from_inertial_columns": [
                    list(row)
                    for row in principals[link].axes_body_from_inertial
                ],
                "input_quaternion_wxyz": list(
                    principals[link].quaternion_wxyz
                ),
                "symmetry_max_abs_error": principals[
                    link
                ].symmetry_max_abs_error,
                "determinant": principals[link].determinant,
                "orthonormality_max_abs_error": principals[
                    link
                ].orthonormality_max_abs_error,
                "python_reconstruction_relative_frobenius_error": principals[
                    link
                ].python_reconstruction_relative_frobenius_error,
                "mujoco_quaternion_rotation_max_abs_error": principals[
                    link
                ].mujoco_quaternion_rotation_max_abs_error,
                "manual_quaternion_rotation_max_abs_error": principals[
                    link
                ].manual_quaternion_rotation_max_abs_error,
                "manual_vs_mujoco_rotation_max_abs_error": principals[
                    link
                ].manual_vs_mujoco_rotation_max_abs_error,
            }
            for link in TARGET_LINKS
        },
    }

    if write:
        if mjcf_changed:
            atomic_write(MJCF_PATH, desired_mjcf_bytes)
        require(
            sha256_file(XACRO_PATH) == DEPLOYED_XACRO_SHA256
            and sha256_file(MJCF_PATH) == DEPLOYED_MJCF_SHA256,
            "post-write model hash verification failed",
        )
        report["status"] = "PASS"
        report["written_paths"] = [str(MJCF_PATH)] if mjcf_changed else []

    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if report["status"] == "PASS" else 2


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--write", action="store_true", help="write exact deployment")
    mode.add_argument("--check", action="store_true", help="verify exact deployment")
    args = parser.parse_args(argv)
    try:
        return run(write=bool(args.write))
    except Exception as error:
        print(f"apply_inertial_parameters_v15_17.py: FAIL: {error}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

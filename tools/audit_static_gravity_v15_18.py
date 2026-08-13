#!/usr/bin/env python3
from __future__ import annotations

"""Deterministic V15.18A offline static-gravity audit.

The production MJCF is copied to an ASCII-only temporary mirror and gravity is
overridden only on the compiled in-memory model.  The tool performs no time
integration: every numerical result is produced by reset/set/``mj_forward``.

``--write`` recomputes the audit and writes the deterministic JSON report.
``--check`` recomputes it and requires byte-identical existing JSON.
``--visualize`` is a diagnostic-only pose witness; it is not acceptance
authority and likewise performs no dynamics integration.
"""

import argparse
import ast
import hashlib
import json
import math
from pathlib import Path, PurePosixPath
import platform
import re
import shutil
import sys
import tempfile
import time
from typing import Any, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SOURCE_COMMIT = "749d7e5f76cae7c1a3fd5462ce5ad92265e8a911"
SOURCE_BRANCH = "agent/v15-17-production-hash-migration"
TARGET_BRANCH = "agent/v15-18a-static-gravity"
SCHEMA = "go-m8010-arm-v15.18a-static-gravity-audit/1.0"
REVISION = "V15.18A-static-gravity-runtime-only"
FINAL_PASS = "V15.18A STATIC_GRAVITY = PASS"
FINAL_FAIL = "V15.18A STATIC_GRAVITY = FAIL"

V14_REL = "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
MJCF_REL = V14_REL + "/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
BRIDGE_REL = V14_REL + "/ros2_ws/src/go_m8010_arm_mujoco_bridge/scripts/mujoco_bridge.py"
COLLISION_503_REL = V14_REL + "/evidence/run_d230_20260812T015818Z/collision_cross_regression_503.json"
COLLISION_PAIR_CONTRACT_REL = V14_REL + "/config/collision_pair_contract_v15_14.json"
TARGET_MANIFEST_REL = V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_qa/config/target_manifest_v15_14.json"
V15_14_FJT_EVIDENCE_REL = V14_REL + "/evidence/visual_sync_d231_20260812/v15_14_visual_sync_d231.json"
MASS_REL = "V15_15_实测质量账本_v1.json"
COM_REL = "V15_15_COM账本_v2.json"
INERTIA_REL = "V15_16_刚体惯量_Engineering_V1.json"
V15_17_REPORT_REL = "V15_17_ProductionHash迁移与轨迹闭环验收.json"
REPORT_JSON_REL = "V15_18A_静态重力与重力矩验收.json"

MJCF_SHA256 = "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
BRIDGE_SHA256 = "9815472370f68f718f63611ac532c989fbaf2cf7d600e4647103e50da5c9a8aa"
MASS_SHA256 = "658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a"
COM_SHA256 = "1cb975890dd121eb41d41bfee414a2c4643f4d917089970fac60009736fc77ae"
INERTIA_SHA256 = "c6c398532d7144ed6aa340a8d8b765b333d01f1fddc2e3280106c90565614401"
V15_17_REPORT_SHA256 = "81032180fcd97bddf068996bf70439917b3b995b4010bfffde0b88351d8e74df"
COLLISION_503_SHA256 = "0518ba83e938a72abe46b6538d490ed2a91396c137ca4734eedc5c546fda9108"
COLLISION_PAIR_CONTRACT_SHA256 = "6d802909e44f238816f007ef33b57e1b57099c5522a473cd2dcc2705397af9c9"
TARGET_MANIFEST_SHA256 = "f2b230e2cb61c251c81228f5f3b89ced0163b05cf82f0b93db8fd0bb85572d57"
V15_14_FJT_EVIDENCE_SHA256 = "7a75ab2ee131b7f734346d45a11888ac913e3a49e8c3691aca2d6b998aee013b"
FROZEN_POSE_SET_SHA256 = "4d9cfaf15cc20fe16393d1aa3bccddc5273e77e2de08cf9d0045976c301799aa"

PROTECTED_HASHES = {
    MJCF_REL: MJCF_SHA256,
    BRIDGE_REL: BRIDGE_SHA256,
    MASS_REL: MASS_SHA256,
    COM_REL: COM_SHA256,
    INERTIA_REL: INERTIA_SHA256,
    V15_17_REPORT_REL: V15_17_REPORT_SHA256,
    COLLISION_503_REL: COLLISION_503_SHA256,
    COLLISION_PAIR_CONTRACT_REL: COLLISION_PAIR_CONTRACT_SHA256,
    TARGET_MANIFEST_REL: TARGET_MANIFEST_SHA256,
    V15_14_FJT_EVIDENCE_REL: V15_14_FJT_EVIDENCE_SHA256,
}

JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")
AUTHORITY_BODIES = ("link2", "link3", "link4", "link5", "link6", "gripper")
AUTHORITY_TOTAL_MASS_KG = 3.4515
GRAVITY = (0.0, 0.0, -9.81)
REVERSE_GRAVITY = (0.0, 0.0, 9.81)
ZERO_GRAVITY = (0.0, 0.0, 0.0)
FINITE_DIFFERENCE_STEPS = (1.0e-5, 3.0e-6)
JOINT_LIMIT_MARGIN_RAD = 0.05

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

THRESHOLDS = {
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


class AuditError(RuntimeError):
    """Malformed/stale input or runtime infrastructure failure (exit two)."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AuditError(message)


def repo_path(relative: str) -> Path:
    parts = PurePosixPath(relative).parts
    require(bool(parts) and ".." not in parts, f"unsafe repository path: {relative}")
    result = (ROOT / Path(*parts)).resolve()
    require(result == ROOT or ROOT in result.parents, f"path escaped repository: {relative}")
    return result


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


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


def max_abs(values: Iterable[float]) -> float:
    return max((abs(float(value)) for value in values), default=0.0)


def finite_absolute_numeric_leaves(value: Any) -> list[float]:
    """Collect finite magnitudes while treating JSON null as unavailable."""

    if value is None or isinstance(value, (str, bool)):
        return []
    if isinstance(value, (int, float)):
        number = float(value)
        return [abs(number)] if math.isfinite(number) else []
    if isinstance(value, Mapping):
        return [number for item in value.values() for number in finite_absolute_numeric_leaves(item)]
    if isinstance(value, (list, tuple)):
        return [number for item in value for number in finite_absolute_numeric_leaves(item)]
    return []


def vector_norm(values: Sequence[float]) -> float:
    return math.sqrt(math.fsum(float(value) * float(value) for value in values))


def recursively_finite(value: Any) -> bool:
    """Return false for any non-finite numeric leaf; null is an allowed N/A."""

    if value is None or isinstance(value, (str, bool)):
        return True
    if isinstance(value, (int, float)):
        return math.isfinite(float(value))
    if isinstance(value, Mapping):
        return all(recursively_finite(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return all(recursively_finite(item) for item in value)
    return False


def json_safe_numeric(value: Any) -> Any:
    """Replace non-finite numeric leaves with null for an auditable FAIL JSON."""

    if isinstance(value, bool) or value is None or isinstance(value, str):
        return value
    if isinstance(value, (int, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, Mapping):
        return {str(key): json_safe_numeric(item) for key, item in value.items()}
    if hasattr(value, "tolist"):
        return json_safe_numeric(value.tolist())
    if isinstance(value, (list, tuple)):
        return [json_safe_numeric(item) for item in value]
    return None


def quaternion_rotation_wxyz(value: Sequence[float]) -> list[list[float]]:
    w, x, y, z = (float(item) for item in value)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    require(norm > 0.0, "compiled inertial quaternion has zero norm")
    w, x, y, z = w / norm, x / norm, y / norm, z / norm
    return [
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)],
    ]


def reconstruct_inertia_tensor(quaternion_wxyz: Sequence[float], principal: Sequence[float]) -> list[list[float]]:
    rotation = quaternion_rotation_wxyz(quaternion_wxyz)
    return [
        [math.fsum(rotation[row][axis] * float(principal[axis]) * rotation[column][axis] for axis in range(3)) for column in range(3)]
        for row in range(3)
    ]


def relative_frobenius(left: Sequence[Sequence[float]], right: Sequence[Sequence[float]]) -> float:
    numerator = math.sqrt(math.fsum((float(left[row][column]) - float(right[row][column])) ** 2 for row in range(3) for column in range(3)))
    denominator = max(math.sqrt(math.fsum(float(right[row][column]) ** 2 for row in range(3) for column in range(3))), 1.0e-30)
    return numerator / denominator


def python_literal(path: Path, name: str) -> Any:
    tree = ast.parse(path.read_text(encoding="utf-8"), filename=path.name)
    matches: list[Any] = []
    for node in tree.body:
        if isinstance(node, (ast.Assign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            if any(isinstance(target, ast.Name) and target.id == name for target in targets):
                matches.append(ast.literal_eval(node.value))
    require(len(matches) == 1, f"{path.name}: exactly one literal {name} required")
    return matches[0]


def model_xml_contract() -> tuple[ET.Element, dict[str, tuple[float, float] | None], dict[str, Any]]:
    root = ET.parse(repo_path(MJCF_REL)).getroot()
    option = root.find("./option")
    gravity = [float(token) for token in str(option.get("gravity", "0 0 0") if option is not None else "0 0 0").split()]
    require(gravity == [0.0, 0.0, 0.0], "production MJCF persisted gravity")
    joint_nodes = {str(node.get("name")): node for node in root.findall(".//joint") if node.get("name") in JOINTS}
    require(set(joint_nodes) == set(JOINTS), "production joint identity set changed")
    limits: dict[str, tuple[float, float] | None] = {}
    joint_parameters = []
    for name in JOINTS:
        node = joint_nodes[name]
        if str(node.get("limited", "auto")).lower() == "false" or not node.get("range"):
            require(name == "J1", f"unexpected unlimited joint: {name}")
            limits[name] = None
        else:
            values = tuple(float(token) for token in str(node.get("range")).split())
            require(len(values) == 2 and values[0] < values[1], f"{name}: invalid range")
            limits[name] = values  # type: ignore[assignment]
        joint_parameters.append({
            "name": name,
            "damping": str(node.get("damping", "0")),
            "frictionloss": str(node.get("frictionloss", "0")),
            "armature": str(node.get("armature", "0")),
        })
    parameters = {
        "joints": joint_parameters,
        "body_gravcomp": [
            {"name": str(body.get("name")), "gravcomp": str(body.get("gravcomp", "0"))}
            for body in root.findall(".//body")
        ],
    }
    return root, limits, {"production_gravity_m_s2": gravity, "parameters": parameters}


def limit_margin(q: Sequence[float], limits: Mapping[str, tuple[float, float] | None]) -> dict[str, float | None]:
    return {
        name: None if limits[name] is None else min(float(q[index]) - limits[name][0], limits[name][1] - float(q[index]))  # type: ignore[index]
        for index, name in enumerate(JOINTS)
    }


def frozen_pose_digest(poses: Sequence[Mapping[str, Any]]) -> str:
    categories: dict[str, list[dict[str, float]]] = {}
    for global_index, pose in enumerate(poses):
        require(int(pose.get("global_index", -1)) == global_index, f"503 global index discontinuity: {global_index}")
        category = str(pose.get("category"))
        rows = categories.setdefault(category, [])
        require(int(pose.get("pose_index", -1)) == len(rows), f"503 category index discontinuity: {category}")
        angles = pose.get("angles_deg")
        require(isinstance(angles, dict) and set(angles) == set(JOINTS), f"503 pose joint set changed: {global_index}")
        rows.append({f"j{index}_deg": finite(angles[name], f"503[{global_index}].{name}") for index, name in enumerate(JOINTS, 1)})
    digest = canonical_digest(categories)
    require(digest == FROZEN_POSE_SET_SHA256, "frozen 503 pose-set digest changed")
    return digest


def pose_selection(limits: Mapping[str, tuple[float, float] | None]) -> dict[str, Any]:
    source = read_json(repo_path(COLLISION_503_REL))
    poses = source.get("poses")
    require(isinstance(poses, list) and len(poses) == 503, "frozen collision report lacks exact 503 poses")
    digest = frozen_pose_digest(poses)
    by_global = {int(row["global_index"]): row for row in poses}
    selected: list[dict[str, Any]] = []
    for global_index, role in SELECTED_503_POSES:
        row = by_global[global_index]
        moveit, mujoco_row = row.get("moveit"), row.get("mujoco")
        require(isinstance(moveit, dict) and isinstance(mujoco_row, dict), "503 collision row malformed")
        source_collision = {
            "moveit_valid": moveit.get("valid") is True,
            "moveit_collision": moveit.get("collision") is True,
            "moveit_self_collision": moveit.get("self_collision") is True,
            "moveit_ground_collision": moveit.get("ground_collision") is True,
            "mujoco_safe": mujoco_row.get("safe") is True,
            "mujoco_collision": mujoco_row.get("collision") is True,
            "mujoco_self_collision": mujoco_row.get("self_collision") is True,
            "mujoco_ground_collision": mujoco_row.get("ground_collision") is True,
        }
        collision_free = source_collision["moveit_valid"] and source_collision["mujoco_safe"] and not any(
            source_collision[key] for key in source_collision if key not in {"moveit_valid", "mujoco_safe"}
        )
        require(collision_free, f"selected frozen pose is not collision-free: {global_index}")
        angles = row["angles_deg"]
        q = [math.radians(float(angles[name])) for name in JOINTS]
        margins = limit_margin(q, limits)
        minimum = min(value for value in margins.values() if value is not None)
        require(minimum >= JOINT_LIMIT_MARGIN_RAD, f"selected frozen pose is near a joint limit: {global_index}")
        selected.append({
            "id": f"frozen_503_global_{global_index:03d}",
            "role": role,
            "source": "FROZEN_503_COLLISION_DATASET",
            "source_relative_path": COLLISION_503_REL,
            "source_sha256": COLLISION_503_SHA256,
            "source_global_index": global_index,
            "source_category": str(row["category"]),
            "source_pose_index": int(row["pose_index"]),
            "q_rad": q,
            "joint_limit_margin_rad": margins,
            "minimum_limited_joint_margin_rad": minimum,
            "source_collision": source_collision,
            "source_collision_free": True,
        })
    evidence = read_json(repo_path(V15_14_FJT_EVIDENCE_REL))
    accepted = [float(value) for value in evidence.get("trajectory", {}).get("target_position_rad", [])]
    require(tuple(accepted) == V15_14_ACCEPTED_FJT_Q_RAD, "accepted V15.14 FJT target changed")
    require(evidence.get("action", {}).get("accepted") is True and evidence.get("action", {}).get("goal_status") == 4, "V15.14 FJT source is not accepted")
    accepted_margins = limit_margin(accepted, limits)
    minimum = min(value for value in accepted_margins.values() if value is not None)
    require(minimum >= JOINT_LIMIT_MARGIN_RAD, "accepted V15.14 FJT pose is near a joint limit")
    selected.append({
        "id": "v15_14_accepted_fjt_final_pose",
        "role": "v15_14_accepted_nonzero",
        "source": "V15_14_ACCEPTED_FJT_EVIDENCE",
        "source_relative_path": V15_14_FJT_EVIDENCE_REL,
        "source_sha256": V15_14_FJT_EVIDENCE_SHA256,
        "source_global_index": None,
        "source_category": None,
        "source_pose_index": None,
        "q_rad": accepted,
        "joint_limit_margin_rad": accepted_margins,
        "minimum_limited_joint_margin_rad": minimum,
        "source_collision": {"accepted_nonzero_target": True, "runtime_collision_recheck_required": True},
        "source_collision_free": None,
    })
    require(len(selected) == 21 and len({row["id"] for row in selected}) == 21, "pose selection is not exact 21 unique poses")
    return {
        "pass": True,
        "source_503_relative_path": COLLISION_503_REL,
        "source_503_sha256": COLLISION_503_SHA256,
        "frozen_pose_set_sha256": digest,
        "algorithm": "EXACT_20_FROZEN_503_GLOBAL_INDICES_PLUS_V15_14_ACCEPTED_FJT_FINAL_POSE",
        "selected_503_global_indices": [index for index, _ in SELECTED_503_POSES],
        "selected_count": 21,
        "joint_limit_margin_rad": JOINT_LIMIT_MARGIN_RAD,
        "selected": selected,
    }


def runtime_full_pairs() -> list[list[str]]:
    contract = read_json(repo_path(COLLISION_PAIR_CONTRACT_REL))
    require(contract.get("schema") == "go-m8010-arm-v15.14-self-collision-pair-contract/2.0", "collision contract schema changed")
    proxies = contract.get("proxies")
    rows = contract.get("runtime_full_pairs")
    excluded_rows = contract.get("runtime_excluded_pairs")
    require(isinstance(proxies, list) and len(proxies) == 25 and len(set(str(value) for value in proxies)) == 25, "collision proxy set is not exact 25")
    require(isinstance(rows, list) and len(rows) == 231, "collision runtime_full_pairs count changed")
    require(isinstance(excluded_rows, list) and len(excluded_rows) == 69, "collision excluded-pair count changed")
    normalized = sorted({tuple(sorted((str(row[0]), str(row[1])))) for row in rows if isinstance(row, list) and len(row) == 2})
    excluded = {tuple(sorted((str(row[0]), str(row[1])))) for row in excluded_rows if isinstance(row, list) and len(row) == 2}
    require(len(normalized) == 231, "collision runtime_full_pairs are not exact unique 231")
    require(len(excluded) == 69 and not (set(normalized) & excluded), "collision pair partition overlaps or duplicates")
    proxy_names = [str(value) for value in proxies]
    universe = {tuple(sorted((left, right))) for index, left in enumerate(proxy_names) for right in proxy_names[index + 1:]}
    require(len(universe) == 300 and set(normalized) | excluded == universe, "collision 25-proxy pair partition is not exhaustive")
    return [list(row) for row in normalized]


def make_ascii_model_mirror(destination: Path) -> tuple[Path, dict[str, Any]]:
    source_path = repo_path(MJCF_REL)
    root = ET.fromstring(source_path.read_bytes())
    compiler = root.find("./compiler")
    mesh_dir = source_path.parent / (compiler.get("meshdir", ".") if compiler is not None else ".")
    texture_dir = source_path.parent / (compiler.get("texturedir", ".") if compiler is not None else ".")
    assets = destination / "assets"
    assets.mkdir(parents=True, exist_ok=True)
    file_assets = [node for node in root.findall("./asset/*") if node.get("file")]
    require(len(file_assets) == 1008, "production model no longer references exactly 1008 assets")
    asset_rows: list[dict[str, Any]] = []
    for index, node in enumerate(file_assets):
        raw = str(node.get("file"))
        require(not Path(raw).is_absolute(), f"absolute asset path: {raw}")
        base = mesh_dir if node.tag.rsplit("}", 1)[-1] == "mesh" else texture_dir
        source = (base / Path(*PurePosixPath(raw).parts)).resolve()
        require(source.is_file() and ROOT in source.parents, f"asset escaped repository or is missing: {raw}")
        target_name = f"asset_{index:04d}{source.suffix.lower() or '.bin'}"
        target_asset = assets / target_name
        source_hash = sha256_file(source)
        shutil.copyfile(source, target_asset)
        copied_hash = sha256_file(target_asset)
        require(copied_hash == source_hash, f"ASCII asset copy SHA256 mismatch: {raw}")
        asset_rows.append({
            "index": index,
            "source_relative_path": source.relative_to(ROOT).as_posix(),
            "source_sha256": source_hash,
            "copy_sha256": copied_hash,
            "match": True,
        })
        node.set("file", "assets/" + target_name)
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", ".")
    compiler.set("texturedir", ".")
    target = destination / "model.xml"
    ET.ElementTree(root).write(target, encoding="utf-8", xml_declaration=True)
    require(str(target).isascii(), "MuJoCo mirror path must be ASCII")
    source_set_sha = canonical_digest([
        {"index": row["index"], "source_relative_path": row["source_relative_path"], "sha256": row["source_sha256"]}
        for row in asset_rows
    ])
    return target, {
        "asset_count": len(asset_rows),
        "all_source_copy_sha256_match": all(row["match"] for row in asset_rows),
        "source_asset_set_sha256": source_set_sha,
        "asset_rows": asset_rows,
    }


def reset_forward(model: Any, data: Any, mujoco: Any, np: Any, q: Sequence[float], gravity: Sequence[float]) -> None:
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


def potential_energy(model: Any, data: Any, np: Any, body_ids: Sequence[int]) -> float:
    ids = np.asarray(body_ids, dtype=int)
    masses = np.asarray(model.body_mass, dtype=float)[ids]
    positions = np.asarray(data.xipos, dtype=float)[ids]
    gravity = np.asarray(model.opt.gravity, dtype=float)
    return float(-np.sum(masses[:, None] * positions * gravity[None, :]))


def proxy_name(name: str) -> str:
    if name.startswith("collision__"):
        pieces = name.split("__")
        if len(pieces) >= 4:
            return pieces[2]
    return name


def accepted_contacts(model: Any, data: Any, mujoco: Any, allowed_pairs: set[tuple[str, str]]) -> list[dict[str, Any]]:
    contacts: list[dict[str, Any]] = []
    for index in range(int(data.ncon)):
        contact = data.contact[index]
        first = str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or "<unnamed>")
        second = str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or "<unnamed>")
        pair = tuple(sorted((proxy_name(first), proxy_name(second))))
        ground = "ground" in pair
        if not ground and pair not in allowed_pairs:
            continue
        contacts.append({
            "geom1": first,
            "geom2": second,
            "proxy_pair": list(pair),
            "contact_class": "ground" if ground else "self_collision",
            "distance_m": float(contact.dist),
        })
    return contacts


def pose_physics(
    model: Any,
    data: Any,
    collision_model: Any,
    collision_data: Any,
    mujoco: Any,
    np: Any,
    pose: Mapping[str, Any],
    authority_ids: Sequence[int],
    all_positive_ids: Sequence[int],
    allowed_pairs: set[tuple[str, str]],
) -> dict[str, Any]:
    q = np.asarray(pose["q_rad"], dtype=float)
    reset_forward(model, data, mujoco, np, q, GRAVITY)
    tau = np.asarray(data.qfrc_bias, dtype=float).copy()
    passive = np.asarray(data.qfrc_passive, dtype=float).copy()
    actuator = np.asarray(data.qfrc_actuator, dtype=float).copy()
    gravcomp = np.asarray(data.qfrc_gravcomp, dtype=float).copy()
    applied = np.asarray(data.qfrc_applied, dtype=float).copy()
    external = np.asarray(data.xfrc_applied, dtype=float).copy()
    ctrl = np.asarray(data.ctrl, dtype=float).copy()
    qacc = np.asarray(data.qacc, dtype=float).copy()
    constraint = np.asarray(data.qfrc_constraint, dtype=float).copy()
    potential = potential_energy(model, data, np, authority_ids)
    all_positive_potential = potential_energy(model, data, np, all_positive_ids)
    baseline_ncon, baseline_nefc = int(data.ncon), int(data.nefc)
    baseline_constraint_free = baseline_ncon == 0 and baseline_nefc == 0 and float(np.max(np.abs(constraint))) <= 1.0e-12

    # The pair-scoped collision diagnostic is a distinct compiled object.  It
    # never provides bias, M, qacc, finite differences, or continuity data.
    reset_forward(collision_model, collision_data, mujoco, np, q, ZERO_GRAVITY)
    contacts = accepted_contacts(collision_model, collision_data, mujoco, allowed_pairs)
    collision_diagnostic = {
        "diagnostic_only_not_mass_matrix_or_qacc_authority": True,
        "runtime_disableflags": int(collision_model.opt.disableflags),
        "raw_ncon": int(collision_data.ncon), "raw_nefc": int(collision_data.nefc),
        "accepted_contacts": contacts, "accepted_contact_count": len(contacts),
        "accepted_contact_set_empty": not contacts,
    }

    baseline_numeric = {
        "q_rad": q.tolist(), "tau": tau.tolist(), "passive": passive.tolist(),
        "actuator": actuator.tolist(), "gravcomp": gravcomp.tolist(),
        "applied": applied.tolist(), "external": external.tolist(), "ctrl": ctrl.tolist(),
        "qacc": qacc.tolist(), "constraint": constraint.tolist(),
        "potential": potential, "all_positive_potential": all_positive_potential,
    }
    baseline_all_finite = recursively_finite(baseline_numeric)
    suspicious_torque = baseline_all_finite and float(np.max(np.abs(tau))) > 100.0
    preflight_pass = baseline_all_finite and not suspicious_torque
    preflight_failure_code = None
    if not baseline_all_finite:
        preflight_failure_code = "FAIL_NONFINITE_STATIC_RESULT"
    elif suspicious_torque:
        preflight_failure_code = "FAIL_SUSPECT_UNIT_OR_COM_ERROR"

    common = {
        "id": str(pose["id"]),
        "role": str(pose["role"]),
        "q_rad": json_safe_numeric(q),
        "joint_limit_margin_rad": pose["joint_limit_margin_rad"],
        "minimum_limited_joint_margin_rad": pose["minimum_limited_joint_margin_rad"],
        "source_collision": pose["source_collision"],
        "source_collision_free": pose["source_collision_free"],
        "initialization": {
            "mj_resetData": True, "qvel_zero": True, "qacc_initialized_zero": True,
            "ctrl_zero": True, "qfrc_applied_zero": True, "xfrc_applied_zero": True,
            "calculation": "INSTANTANEOUS_MJ_FORWARD_ONLY",
        },
        "preflight": {
            "all_baseline_values_finite": baseline_all_finite,
            "max_abs_tau_nm": json_safe_numeric(float(np.max(np.abs(tau))) if baseline_all_finite else float("nan")),
            "suspicious_torque_over_100_nm": bool(suspicious_torque),
            "pass": bool(preflight_pass),
            "failure_code": preflight_failure_code,
        },
        "potential_energy_j": json_safe_numeric(potential),
        "all_positive_body_potential_energy_j": json_safe_numeric(all_positive_potential),
        "tau_hold_mujoco_nm": json_safe_numeric(tau),
        "qfrc_passive": json_safe_numeric(passive),
        "qfrc_actuator": json_safe_numeric(actuator),
        "qfrc_gravcomp": json_safe_numeric(gravcomp),
        "qfrc_applied": json_safe_numeric(applied),
        "xfrc_applied": json_safe_numeric(external),
        "ctrl": json_safe_numeric(ctrl),
        "qacc": json_safe_numeric(qacc),
        "constraint": {
            "ncon": baseline_ncon, "nefc": baseline_nefc,
            "qfrc_constraint": json_safe_numeric(constraint),
            "finite_difference_stencil": None,
            "all_baseline_and_stencils_inactive": False,
        },
        "collision_clone_diagnostic": collision_diagnostic,
    }
    if not preflight_pass:
        return {
            **common,
            "downstream_status": "NOT_RUN_AFTER_PREFLIGHT_FAIL",
            "energy_gradient": None,
            "comparison": None,
            "mass_matrix": None,
            "mass_matrix_diagnostics": None,
            "dynamics_identity": None,
            "zero_gravity": None,
            "reverse_gravity": None,
            "pass": False,
            "failure_codes": [str(preflight_failure_code)],
        }

    mass_matrix = np.zeros((model.nv, model.nv), dtype=np.float64, order="C")
    mujoco.mj_fullM(model, data, mass_matrix)
    mass_matrix_finite = bool(np.all(np.isfinite(mass_matrix)))
    if mass_matrix_finite:
        symmetry_error = float(np.max(np.abs(mass_matrix - mass_matrix.T)))
        try:
            eigenvalues = np.linalg.eigvalsh(0.5 * (mass_matrix + mass_matrix.T))
        except np.linalg.LinAlgError:
            eigenvalues = np.full(model.nv, np.nan, dtype=float)
    else:
        symmetry_error = float("nan")
        eigenvalues = np.full(model.nv, np.nan, dtype=float)
    mass_matrix_spd = mass_matrix_finite and bool(np.all(np.isfinite(eigenvalues))) and float(np.min(eigenvalues)) > 0.0
    try:
        condition_number = float(np.linalg.cond(mass_matrix)) if mass_matrix_finite else float("nan")
    except np.linalg.LinAlgError:
        condition_number = float("inf")
    if mass_matrix_spd and baseline_constraint_free:
        try:
            expected_qacc = -np.linalg.solve(mass_matrix, tau)
            residual = mass_matrix @ qacc + tau
            relative_qacc = float(np.linalg.norm(qacc - expected_qacc) / max(float(np.linalg.norm(expected_qacc)), 1.0e-12))
            dynamics_solve_pass = bool(np.all(np.isfinite(expected_qacc)) and np.all(np.isfinite(residual)) and math.isfinite(relative_qacc))
        except np.linalg.LinAlgError:
            expected_qacc = residual = None
            relative_qacc = None
            dynamics_solve_pass = False
    else:
        expected_qacc = residual = None
        relative_qacc = None
        dynamics_solve_pass = False

    gradients: dict[str, list[float]] = {}
    all_gradients: dict[str, list[float]] = {}
    excluded_link1_gradients: dict[str, list[float]] = {}
    stencil_constraints: list[dict[str, Any]] = []
    excluded_ids = [body_id for body_id in all_positive_ids if body_id not in authority_ids]
    require(len(excluded_ids) == 1, "exact one non-authority positive-mass body required")
    for step in FINITE_DIFFERENCE_STEPS:
        values: list[float] = []
        all_values: list[float] = []
        excluded_values: list[float] = []
        for joint_index, joint in enumerate(JOINTS):
            plus = q.copy(); plus[joint_index] += step
            minus = q.copy(); minus[joint_index] -= step
            reset_forward(model, data, mujoco, np, plus, GRAVITY)
            plus_u = potential_energy(model, data, np, authority_ids)
            plus_all_u = potential_energy(model, data, np, all_positive_ids)
            plus_excluded_u = potential_energy(model, data, np, excluded_ids)
            stencil_constraints.append({
                "step_rad": step, "joint": joint, "side": "plus",
                "ncon": int(data.ncon), "nefc": int(data.nefc),
                "max_abs_qfrc_constraint": float(np.max(np.abs(data.qfrc_constraint))),
                "max_abs_qfrc_passive": float(np.max(np.abs(data.qfrc_passive))),
                "max_abs_qfrc_actuator": float(np.max(np.abs(data.qfrc_actuator))),
                "max_abs_qfrc_gravcomp": float(np.max(np.abs(data.qfrc_gravcomp))),
                "max_abs_qfrc_applied": float(np.max(np.abs(data.qfrc_applied))),
                "max_abs_xfrc_applied": float(np.max(np.abs(data.xfrc_applied))),
                "all_values_finite": bool(all(np.all(np.isfinite(array)) for array in (
                    data.qpos, data.qvel, data.qacc, data.qfrc_bias, data.qfrc_constraint,
                    data.qfrc_passive, data.qfrc_actuator, data.qfrc_gravcomp,
                    data.qfrc_applied, data.xfrc_applied,
                ))),
            })
            reset_forward(model, data, mujoco, np, minus, GRAVITY)
            minus_u = potential_energy(model, data, np, authority_ids)
            minus_all_u = potential_energy(model, data, np, all_positive_ids)
            minus_excluded_u = potential_energy(model, data, np, excluded_ids)
            stencil_constraints.append({
                "step_rad": step, "joint": joint, "side": "minus",
                "ncon": int(data.ncon), "nefc": int(data.nefc),
                "max_abs_qfrc_constraint": float(np.max(np.abs(data.qfrc_constraint))),
                "max_abs_qfrc_passive": float(np.max(np.abs(data.qfrc_passive))),
                "max_abs_qfrc_actuator": float(np.max(np.abs(data.qfrc_actuator))),
                "max_abs_qfrc_gravcomp": float(np.max(np.abs(data.qfrc_gravcomp))),
                "max_abs_qfrc_applied": float(np.max(np.abs(data.qfrc_applied))),
                "max_abs_xfrc_applied": float(np.max(np.abs(data.xfrc_applied))),
                "all_values_finite": bool(all(np.all(np.isfinite(array)) for array in (
                    data.qpos, data.qvel, data.qacc, data.qfrc_bias, data.qfrc_constraint,
                    data.qfrc_passive, data.qfrc_actuator, data.qfrc_gravcomp,
                    data.qfrc_applied, data.xfrc_applied,
                ))),
            })
            values.append((plus_u - minus_u) / (2.0 * step))
            all_values.append((plus_all_u - minus_all_u) / (2.0 * step))
            excluded_values.append((plus_excluded_u - minus_excluded_u) / (2.0 * step))
        gradients[f"{step:.17g}"] = values
        all_gradients[f"{step:.17g}"] = all_values
        excluded_link1_gradients[f"{step:.17g}"] = excluded_values

    reset_forward(model, data, mujoco, np, q, REVERSE_GRAVITY)
    reverse = np.asarray(data.qfrc_bias, dtype=float).copy()
    reset_forward(model, data, mujoco, np, q, ZERO_GRAVITY)
    zero = np.asarray(data.qfrc_bias, dtype=float).copy()

    h1_key, h2_key = (f"{step:.17g}" for step in FINITE_DIFFERENCE_STEPS)
    h1 = np.asarray(gradients[h1_key], dtype=float)
    h2 = np.asarray(gradients[h2_key], dtype=float)
    absolute_errors = np.abs(tau - h2)
    relative_errors = absolute_errors / np.maximum(np.abs(tau), 0.01)
    per_joint_pass = [
        bool(relative_errors[index] < 1.0e-5 if abs(tau[index]) >= 0.1 else absolute_errors[index] < 1.0e-5)
        for index in range(6)
    ]
    reverse_relative = np.abs(tau + reverse) / np.maximum(np.abs(tau), 0.01)
    all_minus_six_diagnostic = np.asarray(all_gradients[h2_key], dtype=float) - h2
    link1_direct_gradient = np.asarray(excluded_link1_gradients[h2_key], dtype=float)
    stencil_clear = all(
        row["ncon"] == 0
        and row["nefc"] == 0
        and row["all_values_finite"] is True
        and row["max_abs_qfrc_constraint"] <= 1.0e-12
        and row["max_abs_qfrc_passive"] <= 1.0e-12
        and row["max_abs_qfrc_actuator"] <= 1.0e-12
        and row["max_abs_qfrc_gravcomp"] <= 1.0e-12
        and row["max_abs_qfrc_applied"] <= 1.0e-12
        and row["max_abs_xfrc_applied"] <= 1.0e-12
        for row in stencil_constraints
    )
    dynamics_clear = baseline_ncon == 0 and baseline_nefc == 0 and float(np.max(np.abs(constraint))) <= 1.0e-12 and stencil_clear
    computed_values_finite = all((
        recursively_finite(gradients), recursively_finite(all_gradients), recursively_finite(excluded_link1_gradients),
        recursively_finite(reverse.tolist()), recursively_finite(zero.tolist()),
        recursively_finite(mass_matrix.tolist()), recursively_finite(eigenvalues.tolist()),
        math.isfinite(symmetry_error), math.isfinite(condition_number),
        recursively_finite(expected_qacc.tolist() if expected_qacc is not None else None),
        recursively_finite(residual.tolist() if residual is not None else None),
        relative_qacc is None or math.isfinite(relative_qacc),
    ))
    gradient_pass = computed_values_finite and float(np.max(np.abs(h1 - h2))) < 1.0e-5 and all(per_joint_pass)
    dynamics_identity_pass = (
        dynamics_solve_pass
        and residual is not None
        and relative_qacc is not None
        and float(np.linalg.norm(residual)) <= 1.0e-9 * max(float(np.linalg.norm(tau)), 1.0)
        and relative_qacc < 1.0e-9
    )
    mass_matrix_pass = mass_matrix_spd and math.isfinite(symmetry_error) and symmetry_error < 1.0e-10 and math.isfinite(condition_number)
    pose_pass = all((
        computed_values_finite,
        dynamics_clear,
        not contacts,
        gradient_pass,
        dynamics_identity_pass,
        float(np.max(np.abs(zero))) < 1.0e-10,
        float(np.max(reverse_relative)) < 1.0e-10,
        mass_matrix_pass,
        float(np.max(np.abs(passive))) <= 1.0e-12,
        float(np.max(np.abs(actuator))) <= 1.0e-12,
        float(np.max(np.abs(gravcomp))) <= 1.0e-12,
        float(np.max(np.abs(applied))) <= 1.0e-12,
        float(np.max(np.abs(external))) <= 1.0e-12,
        float(np.max(np.abs(link1_direct_gradient))) <= 1.0e-12,
        float(np.max(np.abs(tau))) <= 100.0,
    ))
    failure_codes: list[str] = []
    if not computed_values_finite:
        failure_codes.append("FAIL_NONFINITE_COMPUTED_RESULT")
    if not mass_matrix_spd:
        failure_codes.append("FAIL_MASS_MATRIX_NOT_POSITIVE_DEFINITE")
    if not dynamics_solve_pass:
        failure_codes.append("FAIL_MASS_MATRIX_SOLVE_OR_DYNAMICS_IDENTITY")
    if not gradient_pass:
        failure_codes.append("FAIL_POTENTIAL_ENERGY_GRADIENT")
    if not dynamics_clear:
        failure_codes.append("FAIL_ACTIVE_DYNAMICS_CONSTRAINT")
    if contacts:
        failure_codes.append("FAIL_CURRENT_COLLISION_RECHECK")
    return {
        **common,
        "downstream_status": "COMPLETE",
        "energy_gradient": {
            "h1_rad": FINITE_DIFFERENCE_STEPS[0], "h2_rad": FINITE_DIFFERENCE_STEPS[1],
            "h1_nm": json_safe_numeric(h1), "h2_nm": json_safe_numeric(h2), "selected": "h2",
            "step_difference_nm": json_safe_numeric(np.abs(h1 - h2)),
            "excluded_link1_direct_energy_gradient_nm": json_safe_numeric(link1_direct_gradient),
            "all_positive_minus_six_gradient_diagnostic_nm": json_safe_numeric(all_minus_six_diagnostic),
        },
        "comparison": {
            "absolute_error_nm": json_safe_numeric(absolute_errors),
            "relative_error": json_safe_numeric(relative_errors),
            "per_joint_pass": per_joint_pass,
            "pass": bool(gradient_pass),
        },
        "constraint": {
            "ncon": baseline_ncon, "nefc": baseline_nefc,
            "qfrc_constraint": json_safe_numeric(constraint),
            "finite_difference_stencil": json_safe_numeric(stencil_constraints),
            "all_baseline_and_stencils_inactive": dynamics_clear,
        },
        "mass_matrix": json_safe_numeric(mass_matrix),
        "mass_matrix_diagnostics": {
            "matrix_finite": mass_matrix_finite,
            "symmetry_error": json_safe_numeric(symmetry_error),
            "eigenvalues": json_safe_numeric(eigenvalues),
            "min_eigenvalue": json_safe_numeric(float(np.min(eigenvalues)) if np.all(np.isfinite(eigenvalues)) else float("nan")),
            "condition_number": json_safe_numeric(condition_number),
            "positive_definite": bool(mass_matrix_spd),
            "pass": bool(mass_matrix_pass),
        },
        "dynamics_identity": {
            "meaning": "COMPILED_MODEL_INTERNAL_DYNAMICS_IDENTITY_ONLY",
            "solve_status": (
                "COMPLETE" if dynamics_solve_pass
                else "SKIPPED_ACTIVE_CONSTRAINT" if not baseline_constraint_free
                else "NOT_AVAILABLE_MASS_MATRIX_INVALID_OR_SINGULAR"
            ),
            "expected_qacc": json_safe_numeric(expected_qacc),
            "actual_qacc": json_safe_numeric(qacc),
            "residual_vector": json_safe_numeric(residual),
            "residual_norm": json_safe_numeric(float(np.linalg.norm(residual)) if residual is not None else float("nan")),
            "relative_qacc_error": json_safe_numeric(relative_qacc),
            "pass": bool(dynamics_identity_pass),
        },
        "zero_gravity": {
            "qfrc_bias_nm": json_safe_numeric(zero),
            "max_abs_nm": json_safe_numeric(float(np.max(np.abs(zero)))),
            "pass": bool(np.all(np.isfinite(zero)) and np.max(np.abs(zero)) < 1.0e-10),
        },
        "reverse_gravity": {
            "qfrc_bias_nm": json_safe_numeric(reverse), "tau_minus_plus_tau_plus_nm": json_safe_numeric(tau + reverse),
            "relative_error": json_safe_numeric(reverse_relative), "max_relative_error": json_safe_numeric(float(np.max(reverse_relative))),
            "pass": bool(np.all(np.isfinite(reverse_relative)) and np.max(reverse_relative) < 1.0e-10),
        },
        "pass": pose_pass,
        "failure_codes": failure_codes,
    }


def continuity_audit(model: Any, data: Any, mujoco: Any, np: Any, q_end: Sequence[float]) -> dict[str, Any]:
    q_start = np.zeros(6, dtype=float)
    q_finish = np.asarray(q_end, dtype=float)
    rows: list[dict[str, Any]] = []
    for index, alpha in enumerate(np.linspace(0.0, 1.0, 21)):
        q = (1.0 - alpha) * q_start + alpha * q_finish
        reset_forward(model, data, mujoco, np, q, GRAVITY)
        tau = np.asarray(data.qfrc_bias, dtype=float).copy()
        sample_finite = bool(all(np.all(np.isfinite(array)) for array in (
            q, tau, data.qvel, data.qacc, data.qfrc_constraint, data.qfrc_passive,
            data.qfrc_actuator, data.qfrc_gravcomp, data.qfrc_applied, data.xfrc_applied,
        )))
        rows.append({
            "index": index, "alpha": float(alpha), "q_rad": json_safe_numeric(q),
            "tau_hold_nm": json_safe_numeric(tau),
            "ncon": int(data.ncon), "nefc": int(data.nefc),
            "max_abs_qfrc_constraint": json_safe_numeric(float(np.max(np.abs(data.qfrc_constraint)))),
            "max_abs_qfrc_passive": json_safe_numeric(float(np.max(np.abs(data.qfrc_passive)))),
            "max_abs_qfrc_actuator": json_safe_numeric(float(np.max(np.abs(data.qfrc_actuator)))),
            "max_abs_qfrc_gravcomp": json_safe_numeric(float(np.max(np.abs(data.qfrc_gravcomp)))),
            "max_abs_qfrc_applied": json_safe_numeric(float(np.max(np.abs(data.qfrc_applied)))),
            "max_abs_xfrc_applied": json_safe_numeric(float(np.max(np.abs(data.xfrc_applied)))),
            "all_values_finite": sample_finite,
        })
    all_finite = all(row["all_values_finite"] for row in rows)
    if all_finite:
        torques = [row["tau_hold_nm"] for row in rows]
        adjacent = [max_abs(right[j] - left[j] for j in range(6)) for left, right in zip(torques, torques[1:])]
        second = [max_abs(torques[index + 1][j] - 2.0 * torques[index][j] + torques[index - 1][j] for j in range(6)) for index in range(1, 20)]
        maximum_tau = max(max_abs(row) for row in torques)
    else:
        adjacent, second, maximum_tau = [], [], None
    constraints_clear = all(
        row["ncon"] == 0 and row["nefc"] == 0
        and row["max_abs_qfrc_constraint"] is not None and row["max_abs_qfrc_constraint"] <= 1.0e-12
        and row["max_abs_qfrc_passive"] is not None and row["max_abs_qfrc_passive"] <= 1.0e-12
        and row["max_abs_qfrc_actuator"] is not None and row["max_abs_qfrc_actuator"] <= 1.0e-12
        and row["max_abs_qfrc_gravcomp"] is not None and row["max_abs_qfrc_gravcomp"] <= 1.0e-12
        and row["max_abs_qfrc_applied"] is not None and row["max_abs_qfrc_applied"] <= 1.0e-12
        and row["max_abs_xfrc_applied"] is not None and row["max_abs_xfrc_applied"] <= 1.0e-12
        for row in rows
    )
    suspicious = maximum_tau is not None and maximum_tau > 100.0
    passed = all_finite and constraints_clear and not suspicious and max(adjacent) <= 2.0 and max(second) <= 0.25
    return {
        "endpoint_a": "mechanical_zero", "endpoint_b": "v15_14_accepted_fjt_final_pose",
        "q_start": q_start.tolist(), "q_end": q_finish.tolist(), "sample_count": 21,
        "rows": rows,
        "adjacent_inf_norm_jumps_nm": adjacent,
        "second_difference_inf_norm_nm": second,
        "max_adjacent_inf_norm_jump_nm": max(adjacent) if adjacent else None,
        "max_second_difference_inf_norm_nm": max(second) if second else None,
        "max_abs_tau_nm": maximum_tau,
        "suspicious_torque_over_100_nm": suspicious,
        "all_values_finite": all_finite,
        "adjacent_inf_norm_limit_nm": 2.0,
        "second_difference_inf_norm_limit_nm": 0.25,
        "all_constraints_inactive": constraints_clear,
        "failure_codes": ([] if passed else [
            code for condition, code in (
                (not all_finite, "FAIL_NONFINITE_CONTINUITY_RESULT"),
                (not constraints_clear, "FAIL_ACTIVE_CONTINUITY_CONSTRAINT"),
                (bool(suspicious), "FAIL_SUSPECT_UNIT_OR_COM_ERROR"),
                (bool(adjacent) and max(adjacent) > 2.0, "FAIL_TORQUE_CONTINUITY_ADJACENT_JUMP"),
                (bool(second) and max(second) > 0.25, "FAIL_TORQUE_CONTINUITY_SECOND_DIFFERENCE"),
            ) if condition
        ]),
        "pass": passed,
    }


def authority_evidence() -> dict[str, Any]:
    actual = {relative: sha256_file(repo_path(relative)) for relative in sorted(PROTECTED_HASHES)}
    require(actual == PROTECTED_HASHES, "protected authority/model/bridge/source hash mismatch")
    require(python_literal(repo_path(BRIDGE_REL), "ACCEPTED_MODEL_SHA256") == MJCF_SHA256, "production bridge model anchor changed")
    mass = read_json(repo_path(MASS_REL)).get("link_mass_ledger")
    com = read_json(repo_path(COM_REL)).get("links")
    inertia = read_json(repo_path(INERTIA_REL)).get("links")
    require(isinstance(mass, dict) and isinstance(com, dict) and isinstance(inertia, list), "authority structures malformed")
    inertia_by_link = {str(row.get("link")): row for row in inertia if isinstance(row, dict)}
    require(set(mass) == set(com) == set(inertia_by_link) == set(AUTHORITY_BODIES), "authority body set changed")
    six: dict[str, Any] = {}
    for link in AUTHORITY_BODIES:
        mass_value = finite(mass[link].get("nominal_mass_kg"), f"mass.{link}")
        require(abs(mass_value - finite(com[link].get("mass_kg"), f"com.{link}.mass")) <= 1.0e-12, f"{link}: mass authority mismatch")
        require(abs(mass_value - finite(inertia_by_link[link].get("mass_kg"), f"inertia.{link}.mass")) <= 1.0e-12, f"{link}: inertia mass mismatch")
        com_value = [float(value) for value in com[link]["com_link_m"]]
        inertia_com = [float(value) for value in inertia_by_link[link]["com_xyz_m_in_link_frame"]]
        require(max_abs(left - right for left, right in zip(com_value, inertia_com)) <= 1.0e-12, f"{link}: COM authority mismatch")
        six[link] = {
            "mass_kg": mass_value, "com_link_m": com_value,
            "inertia_tensor_kg_m2": inertia_by_link[link]["inertia_tensor_kg_m2"],
        }
    require(abs(sum(row["mass_kg"] for row in six.values()) - AUTHORITY_TOTAL_MASS_KG) <= 1.0e-12, "six-body authority total mass changed")
    return {
        "pass": True,
        "artifacts": {
            relative: {"expected_sha256": expected, "actual_sha256": actual[relative], "pass": actual[relative] == expected}
            for relative, expected in sorted(PROTECTED_HASHES.items())
        },
        "six_body_authority": six,
        "six_body_authority_total_mass_kg": AUTHORITY_TOTAL_MASS_KG,
        "all_unchanged": True,
    }


def run_physics(selection: Mapping[str, Any]) -> dict[str, Any]:
    try:
        import numpy as np  # type: ignore
        import mujoco  # type: ignore
    except ImportError as error:
        raise AuditError(f"MuJoCo/NumPy import failed: {error}") from error
    require(str(mujoco.__version__) == "3.11.0", f"MuJoCo version must be 3.11.0, got {mujoco.__version__}")
    with tempfile.TemporaryDirectory(prefix="v1518a_static_gravity_") as temporary:
        model_path, asset_mirror = make_ascii_model_mirror(Path(temporary) / "mirror")
        model = mujoco.MjModel.from_xml_path(str(model_path))
        collision_model = mujoco.MjModel.from_xml_path(str(model_path))
        require(model is not collision_model and model.nq == model.nv == collision_model.nq == collision_model.nv == 6, "isolated six-DOF model compile failed")
        initial_flags = int(model.opt.disableflags)
        collision_initial_flags = int(collision_model.opt.disableflags)
        require(initial_flags == collision_initial_flags == 0, "compiled production disableflags changed")
        actuation_bit = int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
        filter_parent_bit = int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
        require(actuation_bit == 2048 and filter_parent_bit == 1024, "MuJoCo disable-bit identities changed")
        model.opt.disableflags = initial_flags | actuation_bit
        collision_model.opt.disableflags = collision_initial_flags | actuation_bit | filter_parent_bit
        require(int(model.opt.disableflags) == 2048 and int(collision_model.opt.disableflags) == 3072, "runtime disableflag isolation failed")
        forbidden_bits = {
            "CONTACT": int(mujoco.mjtDisableBit.mjDSBL_CONTACT),
            "CONSTRAINT": int(mujoco.mjtDisableBit.mjDSBL_CONSTRAINT),
            "LIMIT": int(mujoco.mjtDisableBit.mjDSBL_LIMIT),
            "FILTERPARENT": filter_parent_bit,
        }
        require(all((int(model.opt.disableflags) & bit) == 0 for bit in forbidden_bits.values()), "dynamics model disabled forbidden physics")
        compiled_gravity = np.asarray(model.opt.gravity, dtype=float).copy().tolist()
        require(compiled_gravity == [0.0, 0.0, 0.0], "compiled production gravity is not zero")

        authority_ids = []
        for name in AUTHORITY_BODIES:
            body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))
            require(body_id >= 0, f"compiled authority body missing: {name}")
            authority_ids.append(body_id)
        authority_masses = [float(model.body_mass[body_id]) for body_id in authority_ids]
        require(abs(sum(authority_masses) - AUTHORITY_TOTAL_MASS_KG) <= 1.0e-12, "compiled authority mass total changed")
        positive = [
            (str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)), body_id, float(model.body_mass[body_id]))
            for body_id in range(int(model.nbody)) if float(model.body_mass[body_id]) > 0.0
        ]
        require({row[0] for row in positive} == {"link1", *AUTHORITY_BODIES}, "compiled positive-mass body set changed")
        require(abs(sum(row[2] for row in positive) - 4.4515) <= 1.0e-12, "compiled positive-mass total changed")
        all_positive_ids = [row[1] for row in positive]

        data = mujoco.MjData(model)
        collision_data = mujoco.MjData(collision_model)
        pairs = {tuple(row) for row in runtime_full_pairs()}
        poses = [
            pose_physics(model, data, collision_model, collision_data, mujoco, np, pose, authority_ids, all_positive_ids, pairs)
            for pose in selection["selected"]
        ]
        continuity = continuity_audit(model, data, mujoco, np, V15_14_ACCEPTED_FJT_Q_RAD)
        environment = {"python_version": platform.python_version(), "numpy_version": str(np.__version__), "mujoco_version": str(mujoco.__version__)}
        compiled = {
            "nq": int(model.nq), "nv": int(model.nv), "nu": int(model.nu),
            "compiled_initial_disableflags": initial_flags,
            "runtime_dynamics_disableflags": int(model.opt.disableflags),
            "runtime_collision_diagnostic_disableflags": int(collision_model.opt.disableflags),
            "actuation_disable_bit": actuation_bit,
            "forbidden_disable_bits": forbidden_bits,
            "compiled_gravity_before_runtime_override_m_s2": compiled_gravity,
            "authority_body_order": list(AUTHORITY_BODIES),
            "authority_body_ids": authority_ids,
            "authority_masses_kg": authority_masses,
            "authority_total_mass_kg": sum(authority_masses),
            "authority_ipos_m": [np.asarray(model.body_ipos[body_id], dtype=float).copy().tolist() for body_id in authority_ids],
            "authority_iquat_wxyz": [np.asarray(model.body_iquat[body_id], dtype=float).copy().tolist() for body_id in authority_ids],
            "authority_principal_inertia_kg_m2": [np.asarray(model.body_inertia[body_id], dtype=float).copy().tolist() for body_id in authority_ids],
            "all_positive_mass_bodies": [{"body": name, "body_id": body_id, "mass_kg": value} for name, body_id, value in positive],
            "all_positive_mass_total_kg": sum(row[2] for row in positive),
            "link1_placeholder_mass_kg": 1.0,
            "link1_excluded_from_potential_but_retained_in_mass_matrix": True,
            "body_gravcomp": np.asarray(model.body_gravcomp, dtype=float).tolist(),
            "joint_damping": np.asarray(model.dof_damping, dtype=float).tolist(),
            "joint_frictionloss": np.asarray(model.dof_frictionloss, dtype=float).tolist(),
            "joint_armature": np.asarray(model.dof_armature, dtype=float).tolist(),
            "wind_m_s": np.asarray(model.opt.wind, dtype=float).tolist(),
            "density_kg_m3": float(model.opt.density), "viscosity_pa_s": float(model.opt.viscosity),
            "ascii_asset_mirror": asset_mirror,
            "ascii_asset_mirror_count": asset_mirror["asset_count"],
            "temporary_paths_serialized": False,
        }
    return {"environment": environment, "compiled": compiled, "poses": poses, "continuity": continuity}


def build_report() -> dict[str, Any]:
    before_hash = sha256_file(repo_path(MJCF_REL))
    authority = authority_evidence()
    _, limits, xml = model_xml_contract()
    selection = pose_selection(limits)
    physics = run_physics(selection)
    after_hash = sha256_file(repo_path(MJCF_REL))
    bridge_after = sha256_file(repo_path(BRIDGE_REL))
    require(before_hash == after_hash == MJCF_SHA256, "production model changed during runtime audit")
    require(bridge_after == BRIDGE_SHA256, "production bridge changed during runtime audit")

    compiled = physics["compiled"]
    compiled_authority_rows: dict[str, Any] = {}
    for index, link in enumerate(AUTHORITY_BODIES):
        expected = authority["six_body_authority"][link]
        compiled_mass = float(compiled["authority_masses_kg"][index])
        compiled_com = [float(value) for value in compiled["authority_ipos_m"][index]]
        compiled_quaternion = [float(value) for value in compiled["authority_iquat_wxyz"][index]]
        compiled_principal = [float(value) for value in compiled["authority_principal_inertia_kg_m2"][index]]
        tensor = reconstruct_inertia_tensor(compiled_quaternion, compiled_principal)
        mass_error = abs(compiled_mass - float(expected["mass_kg"]))
        com_error = max_abs(left - right for left, right in zip(compiled_com, expected["com_link_m"]))
        tensor_error = relative_frobenius(tensor, expected["inertia_tensor_kg_m2"])
        compiled_authority_rows[link] = {
            "compiled_mass_kg": compiled_mass, "authority_mass_kg": expected["mass_kg"], "mass_error_kg": mass_error,
            "compiled_com_m": compiled_com, "authority_com_m": expected["com_link_m"], "com_max_abs_error_m": com_error,
            "compiled_inertia_tensor_kg_m2": tensor, "inertia_relative_frobenius_error": tensor_error,
            "pass": mass_error <= 1.0e-12 and com_error <= 1.0e-12 and tensor_error < 1.0e-9,
        }
    compiled_authority_validation = {
        "bodies": compiled_authority_rows,
        "max_mass_error_kg": max(row["mass_error_kg"] for row in compiled_authority_rows.values()),
        "max_com_error_m": max(row["com_max_abs_error_m"] for row in compiled_authority_rows.values()),
        "max_inertia_relative_frobenius_error": max(row["inertia_relative_frobenius_error"] for row in compiled_authority_rows.values()),
        "pass": all(row["pass"] for row in compiled_authority_rows.values()),
    }

    poses = physics["poses"]
    complete_poses = [row for row in poses if row.get("downstream_status") == "COMPLETE"]
    finite_tau_poses = [
        row for row in poses
        if isinstance(row.get("tau_hold_mujoco_nm"), list)
        and len(row["tau_hold_mujoco_nm"]) == 6
        and recursively_finite(row["tau_hold_mujoco_nm"])
        and all(value is not None for value in row["tau_hold_mujoco_nm"])
    ]
    joint_statistics: dict[str, Any] = {}
    for joint_index, joint in enumerate(JOINTS):
        values = [(row, float(row["tau_hold_mujoco_nm"][joint_index])) for row in finite_tau_poses]
        if values:
            max_row, max_value = max(values, key=lambda item: abs(item[1]))
            joint_statistics[joint] = {
                "min_nm": min(value for _, value in values), "max_nm": max(value for _, value in values),
                "max_abs_nm": abs(max_value), "pose_of_max_abs": max_row["id"],
            }
        else:
            joint_statistics[joint] = {"min_nm": None, "max_nm": None, "max_abs_nm": None, "pose_of_max_abs": None}
    by_role = {str(row["role"]): row for row in poses}
    mechanical = by_role["mechanical_zero"]
    extension = by_role["combined_extension"]
    j2_max_pose = joint_statistics["J2"]["pose_of_max_abs"]
    j2_row = next((row for row in poses if row["id"] == j2_max_pose), None)

    all_abs_errors = [number for row in complete_poses for number in finite_absolute_numeric_leaves(row["comparison"]["absolute_error_nm"])]
    all_relative_errors = [number for row in complete_poses for number in finite_absolute_numeric_leaves(row["comparison"]["relative_error"])]
    step_differences = [number for row in complete_poses for number in finite_absolute_numeric_leaves(row["energy_gradient"]["step_difference_nm"])]
    excluded_link1_gradients = [number for row in complete_poses for number in finite_absolute_numeric_leaves(row["energy_gradient"]["excluded_link1_direct_energy_gradient_nm"])]
    all_minus_six_diagnostics = [number for row in complete_poses for number in finite_absolute_numeric_leaves(row["energy_gradient"]["all_positive_minus_six_gradient_diagnostic_nm"])]
    actuator_magnitudes = [number for row in poses for number in finite_absolute_numeric_leaves(row["qfrc_actuator"])]
    gravcomp_magnitudes = [number for row in poses for number in finite_absolute_numeric_leaves(row["qfrc_gravcomp"])]
    pose_tau_maxima = [max_abs(row["tau_hold_mujoco_nm"]) for row in finite_tau_poses]
    continuity_tau_maximum = physics["continuity"].get("max_abs_tau_nm")
    global_tau_maximum = max(pose_tau_maxima + ([float(continuity_tau_maximum)] if continuity_tau_maximum is not None else []), default=None)
    metrics = {
        "tested_collision_free_pose_count": len(poses),
        "complete_downstream_pose_count": len(complete_poses),
        "max_abs_tau_nm": global_tau_maximum,
        "max_abs_selected_pose_tau_nm": max(pose_tau_maxima, default=None),
        "max_abs_continuity_tau_nm": continuity_tau_maximum,
        "max_energy_gradient_absolute_error_nm": max(all_abs_errors, default=None),
        "max_energy_gradient_relative_error": max(all_relative_errors, default=None),
        "max_finite_difference_step_difference_nm": max(step_differences, default=None),
        "max_dynamics_residual_norm": max((float(row["dynamics_identity"]["residual_norm"]) for row in complete_poses if row["dynamics_identity"]["residual_norm"] is not None), default=None),
        "max_relative_qacc_error": max((float(row["dynamics_identity"]["relative_qacc_error"]) for row in complete_poses if row["dynamics_identity"]["relative_qacc_error"] is not None), default=None),
        "max_zero_gravity_bias_nm": max((float(row["zero_gravity"]["max_abs_nm"]) for row in complete_poses if row["zero_gravity"]["max_abs_nm"] is not None), default=None),
        "max_reverse_gravity_relative_error": max((float(row["reverse_gravity"]["max_relative_error"]) for row in complete_poses if row["reverse_gravity"]["max_relative_error"] is not None), default=None),
        "max_mass_matrix_symmetry_error": max((float(row["mass_matrix_diagnostics"]["symmetry_error"]) for row in complete_poses if row["mass_matrix_diagnostics"]["symmetry_error"] is not None), default=None),
        "minimum_mass_matrix_eigenvalue": min((float(row["mass_matrix_diagnostics"]["min_eigenvalue"]) for row in complete_poses if row["mass_matrix_diagnostics"]["min_eigenvalue"] is not None), default=None),
        "max_mass_matrix_condition_number": max((float(row["mass_matrix_diagnostics"]["condition_number"]) for row in complete_poses if row["mass_matrix_diagnostics"]["condition_number"] is not None), default=None),
        "max_excluded_link1_direct_energy_gradient_nm": max(excluded_link1_gradients, default=None),
        "max_all_positive_minus_six_gradient_diagnostic_nm": max(all_minus_six_diagnostics, default=None),
        "max_qfrc_actuator_nm": max(actuator_magnitudes, default=None),
        "max_qfrc_gravcomp_nm": max(gravcomp_magnitudes, default=None),
    }
    supplemental_checks = {
        "exact_21_pose_contract": len(poses) == 21 and selection["selected_count"] == 21,
        "all_dynamics_constraints_inactive": len(complete_poses) == 21 and all(row["constraint"]["all_baseline_and_stencils_inactive"] for row in complete_poses),
        "all_pose_physics_checks_pass": len(complete_poses) == 21 and all(row["pass"] for row in poses),
        "compiled_environment_zero_terms": (
            max_abs(compiled["body_gravcomp"]) == 0.0
            and max_abs(compiled["joint_damping"]) == 0.0
            and max_abs(compiled["joint_frictionloss"]) == 0.0
            and max_abs(compiled["joint_armature"]) == 0.0
            and max_abs(compiled["wind_m_s"]) == 0.0
            and float(compiled["density_kg_m3"]) == 0.0
            and float(compiled["viscosity_pa_s"]) == 0.0
            and metrics["max_qfrc_actuator_nm"] == 0.0
            and metrics["max_qfrc_gravcomp_nm"] == 0.0
        ),
        "excluded_link1_direct_gradient_zero": (
            metrics["max_excluded_link1_direct_energy_gradient_nm"] is not None
            and metrics["max_excluded_link1_direct_energy_gradient_nm"] <= 1.0e-12
        ),
        "ascii_1008_asset_copy_hashes_match": (
            compiled["ascii_asset_mirror"]["asset_count"] == 1008
            and compiled["ascii_asset_mirror"]["all_source_copy_sha256_match"] is True
        ),
        "all_values_finite_recursive": (
            len(complete_poses) == 21
            and all(row["preflight"]["all_baseline_values_finite"] is True for row in poses)
            and all("FAIL_NONFINITE_COMPUTED_RESULT" not in row["failure_codes"] for row in poses)
            and physics["continuity"]["all_values_finite"] is True
            and recursively_finite(authority)
            and recursively_finite(compiled)
            and recursively_finite(compiled_authority_validation)
            and recursively_finite(poses)
            and recursively_finite(physics["continuity"])
        ),
    }
    gates_without_hard = {
        "production_model_hash_unchanged": (
            before_hash == after_hash == MJCF_SHA256
            and supplemental_checks["ascii_1008_asset_copy_hashes_match"]
        ),
        "gravity_runtime_only": (
            xml["production_gravity_m_s2"] == [0.0, 0.0, 0.0]
            and compiled["compiled_gravity_before_runtime_override_m_s2"] == [0.0, 0.0, 0.0]
            and compiled["runtime_dynamics_disableflags"] == 2048
        ),
        "at_least_20_collision_free_valid_poses": (
            supplemental_checks["exact_21_pose_contract"]
            and len(poses) >= 20
            and all(not row["collision_clone_diagnostic"]["accepted_contacts"] for row in poses)
            and all(row["source_collision_free"] is True for row in poses[:20])
        ),
        "qfrc_bias_vs_energy_gradient_pass": (
            len(complete_poses) == 21
            and all(row["comparison"]["pass"] for row in complete_poses)
            and supplemental_checks["excluded_link1_direct_gradient_zero"]
        ),
        "mass_qacc_plus_bias_identity_pass": (
            len(complete_poses) == 21
            and all(row["dynamics_identity"]["pass"] for row in complete_poses)
            and supplemental_checks["all_dynamics_constraints_inactive"]
            and supplemental_checks["compiled_environment_zero_terms"]
            and supplemental_checks["all_pose_physics_checks_pass"]
        ),
        "zero_gravity_pass": len(complete_poses) == 21 and all(row["zero_gravity"]["pass"] for row in complete_poses),
        "reverse_gravity_sign_pass": len(complete_poses) == 21 and all(row["reverse_gravity"]["pass"] for row in complete_poses),
        "mass_matrix_symmetric_positive_definite": len(complete_poses) == 21 and all(row["mass_matrix_diagnostics"]["pass"] for row in complete_poses),
        "no_nan_or_inf": supplemental_checks["all_values_finite_recursive"],
        "no_suspicious_over_100_nm": (
            metrics["max_abs_tau_nm"] is not None
            and metrics["max_abs_tau_nm"] <= 100.0
            and not any(row["preflight"]["suspicious_torque_over_100_nm"] for row in poses)
            and physics["continuity"]["suspicious_torque_over_100_nm"] is False
        ),
        "torque_continuity_sanity_pass": physics["continuity"]["pass"],
        "mass_com_inertia_authority_unchanged": authority["all_unchanged"] and compiled_authority_validation["pass"],
        "production_bridge_unchanged": bridge_after == BRIDGE_SHA256,
        "gravity_not_persisted_into_production_xml": after_hash == MJCF_SHA256 and xml["production_gravity_m_s2"] == [0.0, 0.0, 0.0],
    }
    hard_codes = [name for name, passed in gates_without_hard.items() if not passed]
    hard_codes.extend(code for row in poses for code in row.get("failure_codes", []))
    hard_codes.extend(physics["continuity"].get("failure_codes", []))
    hard_unresolved = sorted(set(hard_codes))
    gates = {**gates_without_hard, "hard_unresolved_items_empty": not hard_unresolved}
    require(len(gates) == 15, "acceptance gate contract must contain exact 15 items")
    status = "PASS" if all(gates.values()) and not hard_unresolved else "FAIL"
    final_status = FINAL_PASS if status == "PASS" else FINAL_FAIL
    suspicious_torque_fact = (
        "N/A_NONFINITE"
        if metrics["max_abs_tau_nm"] is None
        else "YES" if metrics["max_abs_tau_nm"] > 100.0 else "NO"
    )
    report = {
        "schema": SCHEMA,
        "revision": REVISION,
        "audit_valid": True,
        "pass": status == "PASS",
        "status": status,
        "final_status": final_status,
        "scope": {
            "task": "OFFLINE_STATIC_GRAVITY_AUDIT",
            "source_commit": SOURCE_COMMIT, "source_branch": SOURCE_BRANCH, "target_branch": TARGET_BRANCH,
            "production_model_relative_path": MJCF_REL,
            "prohibited_scope": ["FREE_FALL", "GRAVITY_COMPENSATION", "PID_TUNING", "FRICTION", "DAMPING", "ARMATURE", "MOTOR_MODEL", "PRODUCTION_BRIDGE_EXECUTION"],
            "free_dynamics_integration_performed": False,
        },
        "environment": physics["environment"],
        "authority": authority,
        "production_protection": {
            "model": {
                "relative_path": MJCF_REL, "expected_sha256": MJCF_SHA256,
                "before_sha256": before_hash, "after_sha256": after_hash,
                "unchanged": before_hash == after_hash == MJCF_SHA256,
                "production_xml_gravity_m_s2": xml["production_gravity_m_s2"],
                "gravity_not_persisted": xml["production_gravity_m_s2"] == [0.0, 0.0, 0.0],
            },
            "bridge": {
                "relative_path": BRIDGE_REL, "expected_sha256": BRIDGE_SHA256,
                "actual_sha256": bridge_after, "accepted_model_sha256": MJCF_SHA256,
                "unchanged": bridge_after == BRIDGE_SHA256,
            },
            "forbidden_parameter_snapshot": xml["parameters"],
            "forbidden_parameter_snapshot_sha256": canonical_digest(xml["parameters"]),
        },
        "runtime_configuration": {
            "gravity_m_s2": list(GRAVITY), "reverse_gravity_m_s2": list(REVERSE_GRAVITY), "zero_gravity_m_s2": list(ZERO_GRAVITY),
            "runtime_only": True, "actuation_disabled": True, "ctrl_zero": True,
            "dynamics_disableflags": 2048,
            "collision_diagnostic_disableflags": 3072,
            "collision_and_dynamics_model_objects_isolated": True,
            "no_time_integration": True,
        },
        "compiled_model": compiled,
        "compiled_authority_validation": compiled_authority_validation,
        "pose_selection": selection,
        "poses": poses,
        "special_poses": {
            "mechanical_zero": {"id": mechanical["id"], "q_rad": mechanical["q_rad"], "tau_hold_nm": mechanical["tau_hold_mujoco_nm"]},
            "max_extension": {"id": extension["id"], "q_rad": extension["q_rad"], "tau_hold_nm": extension["tau_hold_mujoco_nm"]},
            "v15_14_accepted_nonzero": {
                "id": "v15_14_accepted_fjt_final_pose", "q_rad": list(V15_14_ACCEPTED_FJT_Q_RAD),
                "tau_hold_nm": poses[-1]["tau_hold_mujoco_nm"],
            },
        },
        "joint_statistics": joint_statistics,
        "j2_equal_share_diagnostic": {
            "label": "IDEAL_EQUAL_LOAD_SHARE_DIAGNOSTIC_ONLY",
            "not_hardware_current_distribution_authority": True,
            "pose_id": j2_row["id"] if j2_row is not None else None,
            "total_joint_torque_nm": j2_row["tau_hold_mujoco_nm"][1] if j2_row is not None else None,
            "ideal_equal_share_per_motor_nm": j2_row["tau_hold_mujoco_nm"][1] / 2.0 if j2_row is not None else None,
        },
        "continuity": physics["continuity"],
        "metrics": metrics,
        "thresholds": THRESHOLDS,
        "supplemental_checks": supplemental_checks,
        "acceptance_gates": gates,
        "acceptance_gate_count": len(gates),
        "acceptance_gate_pass_count": sum(bool(value) for value in gates.values()),
        "hard_unresolved_items": hard_unresolved,
        "limitations": {
            "static_gravity_torque_depends_on": ["mass", "COM", "gravity", "kinematics"],
            "inertia_tensor_main_role": ["dynamic_acceleration", "mass_matrix", "transient_response"],
            "gravity_pass_claim": "MASS_COM_KINEMATIC_GRAVITY_PART_CREDIBLE",
            "full_dynamics_validated": False,
            "mass_matrix_identity_scope": "COMPILED_MODEL_INTERNAL_DYNAMICS_IDENTITY_ONLY_INCLUDES_LINK1_PLACEHOLDER",
        },
        "gui_visual_witness": {
            "available_via": "tools/audit_static_gravity_v15_18.py --visualize",
            "diagnostic_only": True, "not_numeric_authority": True,
            "method": "RESET_SET_Q_RUNTIME_GRAVITY_MJ_FORWARD_VIEWER_SYNC",
            "no_time_integration": True,
            "production_model_expected_sha256": MJCF_SHA256,
            "execution_witness_claimed_by_this_headless_report": False,
        },
    }
    report["final_report_facts"] = [
        {"number": 1, "name": "branch_commit", "value": {"branch": TARGET_BRANCH, "commit_contract": "SOURCE_COMMIT_OR_ITS_UNIQUE_DIRECT_CHILD"}},
        {"number": 2, "name": "production_mjcf_hash_before_after", "value": {"before": before_hash, "after": after_hash}},
        {"number": 3, "name": "tested_collision_free_poses_count", "value": len(poses)},
        {"number": 4, "name": "gravity_vector_m_s2", "value": list(GRAVITY)},
        {"number": 5, "name": "mechanical_zero_tau_hold_nm", "value": mechanical["tau_hold_mujoco_nm"]},
        {"number": 6, "name": "max_extension_tau_hold_nm", "value": extension["tau_hold_mujoco_nm"]},
        *[
            {"number": 7 + index, "name": f"{joint}_max_abs_gravity_torque_nm", "value": joint_statistics[joint]["max_abs_nm"]}
            for index, joint in enumerate(JOINTS)
        ],
        {"number": 13, "name": "j2_max_pose_ideal_half_motor_share", "value": report["j2_equal_share_diagnostic"]},
        {"number": 14, "name": "max_qfrc_bias_vs_potential_gradient_absolute_error_nm", "value": metrics["max_energy_gradient_absolute_error_nm"]},
        {"number": 15, "name": "max_qfrc_bias_vs_potential_gradient_relative_error", "value": metrics["max_energy_gradient_relative_error"]},
        {"number": 16, "name": "max_mass_qacc_plus_bias_residual_norm", "value": metrics["max_dynamics_residual_norm"]},
        {"number": 17, "name": "zero_gravity_max_bias_nm", "value": metrics["max_zero_gravity_bias_nm"]},
        {"number": 18, "name": "reverse_gravity_max_symmetry_error", "value": metrics["max_reverse_gravity_relative_error"]},
        {"number": 19, "name": "mass_matrix_min_eigenvalue", "value": metrics["minimum_mass_matrix_eigenvalue"]},
        {"number": 20, "name": "max_mass_matrix_condition_number", "value": metrics["max_mass_matrix_condition_number"]},
        {"number": 21, "name": "torque_continuity", "value": "PASS" if physics["continuity"]["pass"] else "FAIL"},
        {"number": 22, "name": "suspicious_over_100_nm_torque", "value": suspicious_torque_fact},
        {"number": 23, "name": "production_bridge_modified", "value": "NO" if bridge_after == BRIDGE_SHA256 else "YES"},
        {"number": 24, "name": "production_mjcf_persisted_gravity", "value": "NO" if xml["production_gravity_m_s2"] == [0.0, 0.0, 0.0] else "YES"},
        {"number": 25, "name": "mass_com_inertia_authority_modified", "value": "NO" if gates["mass_com_inertia_authority_unchanged"] else "YES"},
        {"number": 26, "name": "hard_unresolved_items", "value": hard_unresolved},
        {"number": 27, "name": "final_status", "value": final_status},
    ]
    require([row["number"] for row in report["final_report_facts"]] == list(range(1, 28)), "final report fact numbering is not exact 1..27")
    return report


def report_bytes(report: Mapping[str, Any]) -> bytes:
    return (json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")


def visualize(selection: Mapping[str, Any], dwell_seconds: float, cycles: int) -> None:
    """Open the diagnostic GUI while keeping every pose strictly static."""

    try:
        import numpy as np  # type: ignore
        import mujoco  # type: ignore
        import mujoco.viewer  # type: ignore
    except ImportError as error:
        raise AuditError(f"MuJoCo viewer import failed: {error}") from error
    require(str(mujoco.__version__) == "3.11.0", "GUI witness requires MuJoCo 3.11.0")
    before = sha256_file(repo_path(MJCF_REL))
    require(before == MJCF_SHA256, "GUI witness production hash mismatch before launch")
    with tempfile.TemporaryDirectory(prefix="v1518a_visual_witness_") as temporary:
        model_path, _asset_mirror = make_ascii_model_mirror(Path(temporary) / "mirror")
        model = mujoco.MjModel.from_xml_path(str(model_path))
        require(int(model.opt.disableflags) == 0, "GUI compiled disableflags changed")
        model.opt.disableflags = int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
        require(int(model.opt.disableflags) == 2048, "GUI actuation isolation failed")
        data = mujoco.MjData(model)
        with mujoco.viewer.launch_passive(model, data) as viewer:
            for _ in range(cycles):
                for pose in selection["selected"]:
                    if not viewer.is_running():
                        break
                    reset_forward(model, data, mujoco, np, pose["q_rad"], GRAVITY)
                    mujoco.mj_forward(model, data)
                    viewer.sync()
                    deadline = time.monotonic() + dwell_seconds
                    while viewer.is_running() and time.monotonic() < deadline:
                        viewer.sync()
                        time.sleep(min(0.05, max(0.0, deadline - time.monotonic())))
                if not viewer.is_running():
                    break
    after = sha256_file(repo_path(MJCF_REL))
    require(after == before == MJCF_SHA256, "GUI witness changed production MJCF")
    print("VISUAL_WITNESS=COMPLETE")
    print(f"POSES={len(selection['selected'])}")
    print("NUMERIC_AUTHORITY=NO")
    print("TIME_INTEGRATION=NO")
    print(f"PRODUCTION_MODEL_BEFORE_AFTER_SHA256={before}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="V15.18A static gravity audit")
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--write", action="store_true", help="write deterministic JSON audit")
    modes.add_argument("--check", action="store_true", help="recompute and compare deterministic JSON audit")
    modes.add_argument("--visualize", action="store_true", help="open diagnostic-only static pose GUI witness")
    parser.add_argument("--dwell-seconds", type=float, default=1.0, help="GUI dwell per pose")
    parser.add_argument("--cycles", type=int, default=1, help="GUI pose-set cycles")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        require(args.dwell_seconds > 0.0 and args.cycles >= 1, "GUI dwell/cycles must be positive")
        _, limits, _ = model_xml_contract()
        selection = pose_selection(limits)
        if args.visualize:
            visualize(selection, args.dwell_seconds, args.cycles)
            return 0
        report = build_report()
        payload = report_bytes(report)
        path = repo_path(REPORT_JSON_REL)
        if args.write:
            path.write_bytes(payload)
        else:
            require(path.is_file(), "V15.18A JSON report is missing")
            require(path.read_bytes() == payload, "V15.18A JSON report is stale or non-deterministic")
        print("AUDIT_VALID=YES")
        print(f"STATUS={report['status']}")
        print(f"FINAL_STATUS={report['final_status']}")
        print(f"SELECTED_POSES={report['pose_selection']['selected_count']}")
        print(f"HARD_UNRESOLVED={len(report['hard_unresolved_items'])}")
        return 0
    except (AuditError, OSError, ValueError, TypeError, json.JSONDecodeError) as error:
        message = re.sub(r"[\r\n]+", " ", str(error).replace(str(ROOT), "<REPO_ROOT>")).strip()
        print("AUDIT_VALID=NO", file=sys.stderr)
        print("STATUS=FAIL", file=sys.stderr)
        print(f"ERROR={message}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

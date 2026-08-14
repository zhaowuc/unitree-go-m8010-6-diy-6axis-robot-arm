#!/usr/bin/env python3
from __future__ import annotations

"""Independent validator for the V15.18B2 integrator-attribution audit.

The validator intentionally does not import ``audit_passive_gravity_v15_18b``.
It locks the accepted B1 six semantic pose slots without re-selecting them,
loads an ASCII-only byte mirror of the protected production MJCF/assets, and
independently repeats A/B/C/I4/RK4_2MS/RK4_1MS.  The audit uniquely owns both
JSON and Markdown; this validator only recomputes and validates them.
"""

import argparse
import ast
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import shutil
import subprocess
import sys
import tempfile
from typing import Any, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SOURCE_COMMIT = "6dc27f6f8196c691f9f6b1c7684202dec6af2b6a"
SOURCE_BRANCH = "agent/v15-18a-static-gravity"
TARGET_BRANCH = "agent/v15-18b-final-simulation-handoff"

V14_REL = "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
MJCF_REL = V14_REL + "/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
BRIDGE_REL = V14_REL + "/ros2_ws/src/go_m8010_arm_mujoco_bridge/scripts/mujoco_bridge.py"
BRIDGE_ENTRY_REL = V14_REL + "/ros2_ws/src/go_m8010_arm_mujoco_bridge/scripts/mujoco_bridge"
ROS2_CONTROLLERS_REL = V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config/ros2_controllers.yaml"
MOVEIT_CONTROLLERS_REL = V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config/moveit_controllers.yaml"
MASS_REL = "V15_15_实测质量账本_v1.json"
COM_REL = "V15_15_COM账本_v2.json"
INERTIA_REL = "V15_16_刚体惯量_Engineering_V1.json"
V15_17_REPORT_REL = "V15_17_ProductionHash迁移与轨迹闭环验收.json"
V15_17_INERTIAL_REL = "V15_17_Inertial参数部署验收.json"
V15_18A_JSON_REL = "V15_18A_静态重力与重力矩验收.json"
V15_18A_MD_REL = "V15_18A_静态重力与重力矩验收.md"
V15_18A_AUDIT_REL = "tools/audit_static_gravity_v15_18.py"
V15_18A_VALIDATOR_REL = "tools/validate_static_gravity_v15_18.py"
AUDIT_REL = "tools/audit_passive_gravity_v15_18b.py"
VALIDATOR_REL = "tools/validate_passive_gravity_v15_18b.py"
REPORT_JSON_REL = "V15_18B_短时被动重力动力学验收.json"
REPORT_MD_REL = "V15_18B_短时被动重力动力学验收.md"

MJCF_SHA256 = "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
BRIDGE_SHA256 = "d3273ae0ecfe3ee2bf427a5732f4106206593a2c23fd0bc39aea1f3381f7a5a0"
BRIDGE_ENTRY_SHA256 = "59dc2b9f1d069d0d3a6819d332cdf2f882f7feb2dc8cfb4368b883277818989b"
ROS2_CONTROLLERS_SHA256 = "c0dc9a55a383b536cc1461c2b1711b211002debdda99c1b00125f36c3e31ca65"
MOVEIT_CONTROLLERS_SHA256 = "2cf05c567fce92ce8cbb0ea7cf305159b20ca55ce800463eb100ce6d5135193f"
MASS_SHA256 = "658fa0b2aef1da4200d86fccf742e135215288e77f58fa5095a06f513eaba04a"
COM_SHA256 = "1cb975890dd121eb41d41bfee414a2c4643f4d917089970fac60009736fc77ae"
INERTIA_SHA256 = "c6c398532d7144ed6aa340a8d8b765b333d01f1fddc2e3280106c90565614401"
V15_17_REPORT_SHA256 = "e280996b9e894492dac8e2e2fc59ee86e49af34f29adab6b5cd989edc29cd455"
V15_17_INERTIAL_SHA256 = "07b0adc158423671a0de97b85796421424d5bae1603c1ace629f821a0938604e"
V15_18A_JSON_SHA256 = "640e9104cabd0e2548e07bdd54d5cdb2c66dbdf86a7981eff4d8c12e55b9dfe2"
V15_18A_MD_SHA256 = "c4752895ddce399a13ce7e54e909e91410894bc26c7de0342cdf1ea554d13abc"
V15_18A_AUDIT_SHA256 = "ecb8bd60631f3ae482d411155aada691cb77e7528ea78c2a6aaf5a5de0357da9"
V15_18A_VALIDATOR_SHA256 = "04df0861e717bdf4ae78fc048300bfa16578940cf8588632d8b61c510afafe32"
AUDIT_SHA256 = "bbba568b8b52694b8f995a63d6b0baa68840f343ccad59d0c016b6fa544a03f9"
REPORT_JSON_SHA256 = "4cde5893981e8388d76997bef3f6cd05abf427b814bb16fe2de6cab42569d730"
REPORT_MD_SHA256 = "d9e759068598b252743d37c8d7c10d732b1516d7a098b3cc3455fc95abeff5f7"

SCHEMA = "go-m8010-arm-v15.18b-passive-gravity-dynamics-audit/2.0"
REVISION = "V15.18B2-numerical-integrator-attribution"
FINAL_PASS = "V15.18B PASSIVE_GRAVITY_DYNAMICS = PASS_WITH_NUMERICAL_INTEGRATOR_LIMITATION"
FINAL_FAIL = "V15.18B PASSIVE_GRAVITY_DYNAMICS = FAIL"
PURE_COMPLETE = "PURE_SIMULATION_PHASE = COMPLETE"
NEXT_PHASE = "V15.19 REAL_HARDWARE_READONLY_BRINGUP"
PURE_INCOMPLETE = "PURE_SIMULATION_PHASE = INCOMPLETE"
NEXT_REVIEW = "V15.18B DYNAMICS_REVIEW"
JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")
AUTHORITY_BODIES = ("link2", "link3", "link4", "link5", "link6", "gripper")
GRAVITY = (0.0, 0.0, -9.81)
DURATION_S = 0.15
PROBE_S = 0.02
INITIAL_MARGIN_RAD = 0.15
RUNTIME_MARGIN_RAD = 0.02
ACTUATOR_TOL_NM = 1.0e-12
QACC_REL_TOL = 1.0e-10
RESIDUAL_TOL = 1.0e-10
EARLY_QACC_MIN = 0.05
EARLY_PREDICTED_DISPLACEMENT_MIN = 1.0e-7
ENERGY_DRIFT_PRODUCTION_MAX = 0.01
ENERGY_DRIFT_HALF_MAX = 0.005
ENERGY_FLOOR_J = 1.0e-8
DETERMINISM_TOL = 1.0e-12
MAX_QVEL = 20.0
MAX_QACC = 500.0
MAX_QPOS_STEP = 0.05
MAX_QVEL_STEP = 5.0

# V15.18B2 compares five numerical configurations plus C, the deterministic
# repeat of A.  Every profile is an isolated runtime-only compiled model.
ATTRIBUTION_CONFIGURATIONS = (
    ("A", "implicitfast", 0.002),
    ("B", "implicitfast", 0.001),
    ("C", "implicitfast", 0.002),
    ("I4", "implicitfast", 0.0005),
    ("RK4_2MS", "RK4", 0.002),
    ("RK4_1MS", "RK4", 0.001),
)
ROOT_CAUSE_CONDITION_KEYS = (
    "v15_18a_static_gravity_pass",
    "initial_qacc_identity_pass",
    "early_motion_direction_pass",
    "implicitfast_energy_convergence_pass",
    "implicitfast_trajectory_convergence_pass",
    "rk4_energy_reference_pass",
    "rk4_timestep_convergence_pass",
    "no_contacts_all_runs_pass",
    "no_active_joint_limits_all_runs_pass",
    "all_values_finite_all_runs_pass",
    "determinism_pass",
)
ROOT_CAUSE_CLASSIFICATION = "NUMERICAL_INTEGRATOR_TRUNCATION_ERROR_CONFIRMED"
CONTINUOUS_TIME_MODEL_PASS = "PASS"

# Root contract ruling: max_extension and frozen_503_global_481 are two
# semantic slots even though V15.18A gives them the same requested id/q.
REQUESTED_SLOTS = (
    (1, "mechanical_zero", "frozen_503_global_018", None),
    (2, "max_extension", "frozen_503_global_481", None),
    (3, "frozen_503_global_185", "frozen_503_global_185", None),
    (4, "frozen_503_global_432", "frozen_503_global_432", None),
    (5, "frozen_503_global_481", "frozen_503_global_481", 2),
    (6, "folded_low_torque_pose", None, None),
)
LOCKED_POSE_PROJECTION_SHA256 = "5b235a14c6046f8da73bf2a3d2d09f3464d08af1b3def85cee417ed35ab20c55"
LEGACY_RUNS_ABC_SHA256 = "07bd4a626f5b5cbab93a2464cb8bc92a70980409dbeaa7328c998119106f8a72"
LOCKED_POSES = (
    (1, "mechanical_zero", "frozen_503_global_018", "frozen_503_global_018", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
    (2, "max_extension", "frozen_503_global_481", "frozen_503_global_354", [-0.7245796444376641, 1.913905511611287, 0.9011701758715863, 1.4492272448237147, 1.9102428548891142, 0.2520079829930116]),
    (3, "frozen_503_global_185", "frozen_503_global_185", "frozen_503_global_185", [0.0, 0.0, 0.0, 0.0, 0.6876597252857658, 0.0]),
    (4, "frozen_503_global_432", "frozen_503_global_432", "frozen_503_global_473", [1.124128858085009, 0.6821041080272624, 1.257751285503815, 0.08700241052079227, -0.5032794983833337, 0.22999613584324316]),
    (5, "frozen_503_global_481", "frozen_503_global_481", "frozen_503_global_354", [-0.7245796444376641, 1.913905511611287, 0.9011701758715863, 1.4492272448237147, 1.9102428548891142, 0.2520079829930116]),
    (6, "folded_low_torque_pose", None, "frozen_503_global_369", [-0.07301686429765095, 1.1615298819825015, 1.2726638344330896, -1.3717138538115992, 0.7387706518083362, 0.4773492170113097]),
)

TOP_LEVEL_KEYS = {
    "schema", "revision", "audit_valid", "pass", "status", "final_status",
    "pure_simulation_phase", "next_phase", "source_commit", "source_branch",
    "target_branch", "production_model_relative_path", "production_model_sha256",
    "mujoco_version", "numpy_version", "gravity", "timestep",
    "runtime_configuration", "pose_selection", "per_pose_runs",
    "initial_acceleration_identity", "early_motion_direction", "energy_audit",
    "timestep_convergence", "determinism", "contacts", "joint_limit_margin",
    "stability", "trajectory_continuity", "protected_hashes", "thresholds", "acceptance_gates",
    "hard_unresolved_items", "unresolved_items", "limitations", "visual_witness",
    "report_contract", "root_cause_classification", "continuous_time_dynamics_model",
    "v15_18b_passive_gravity_dynamics", "production_integrator_limitation",
    "root_cause_condition_map_exact_11", "root_cause_condition_keys",
    "root_cause_mutation_consistency", "ubuntu_runtime_verification",
    "github_handoff_status", "production_integrator_diagnostic",
    "numerical_integrator_attribution",
}
GATE_ORDER = (
    "exact_six_pose_slots", "pose_selection_safe", "legacy_production_runs_preserved",
    "v15_18a_continuous_time_physics_pass", "initial_acceleration_identity_pass",
    "early_motion_direction_pass", "implicitfast_energy_convergence_pass",
    "implicitfast_trajectory_convergence_pass", "rk4_energy_reference_pass",
    "rk4_timestep_convergence_pass", "high_precision_reference_diagnostic_pass", "determinism_pass",
    "actuation_and_applied_force_isolation_pass", "no_contacts_pass",
    "no_active_joint_limits_pass", "joint_limit_margin_pass", "stability_pass",
    "trajectory_continuity_pass",
    "protected_hashes_unchanged", "production_model_hash_unchanged",
    "bridge_controller_unchanged", "all_values_finite",
    "numerical_integrator_truncation_error_confirmed",
    "hard_unresolved_items_empty",
)
GATE_KEYS = set(GATE_ORDER)
RUN_CORE_KEYS = {
    "active_joint_limit_constraint_count_max", "all_values_finite", "contact_count_max",
    "dt_s", "duration_s", "energy_summary", "failure_codes", "final_qpos_rad",
    "final_qvel_rad_s", "first_contact_rows", "independent_authority_potential_j",
    "independent_kinetic_j", "independent_potential_j", "independent_total_j",
    "initial_qacc_rad_s2", "label", "max_abs_ctrl", "max_abs_qacc_rad_s2",
    "max_abs_qfrc_actuator_nm", "max_abs_qfrc_applied_nm",
    "max_abs_qfrc_passive_nm", "max_abs_qpos_step_rad", "max_abs_qvel_rad_s",
    "max_abs_qvel_step_rad_s", "max_abs_xfrc_applied",
    "minimum_joint_limit_margin_rad", "mujoco_energy_j", "pass", "qpos_rad",
    "qvel_rad_s", "qacc_rad_s2", "max_abs_qacc_step_rad_s2", "continuity", "safety", "step_count", "time_s",
}

PROTECTED_HASHES = {
    MJCF_REL: MJCF_SHA256,
    BRIDGE_REL: BRIDGE_SHA256,
    BRIDGE_ENTRY_REL: BRIDGE_ENTRY_SHA256,
    ROS2_CONTROLLERS_REL: ROS2_CONTROLLERS_SHA256,
    MOVEIT_CONTROLLERS_REL: MOVEIT_CONTROLLERS_SHA256,
    MASS_REL: MASS_SHA256,
    COM_REL: COM_SHA256,
    INERTIA_REL: INERTIA_SHA256,
    V15_17_REPORT_REL: V15_17_REPORT_SHA256,
    V15_17_INERTIAL_REL: V15_17_INERTIAL_SHA256,
    V15_18A_JSON_REL: V15_18A_JSON_SHA256,
    V15_18A_MD_REL: V15_18A_MD_SHA256,
    V15_18A_AUDIT_REL: V15_18A_AUDIT_SHA256,
    V15_18A_VALIDATOR_REL: V15_18A_VALIDATOR_SHA256,
}

AUDIT_PROTECTED_HASHES = {
    relative: PROTECTED_HASHES[relative]
    for relative in (
        MJCF_REL, BRIDGE_ENTRY_REL, BRIDGE_REL, MOVEIT_CONTROLLERS_REL,
        ROS2_CONTROLLERS_REL, COM_REL, MASS_REL, INERTIA_REL,
        V15_17_INERTIAL_REL, V15_17_REPORT_REL, V15_18A_JSON_REL,
    )
}


class ValidationError(RuntimeError):
    pass


def require(condition: bool, message: str) -> None:
    if not condition:
        raise ValidationError(message)


def clean_error(error: BaseException | str) -> str:
    return " ".join(str(error).replace("\r", " ").replace("\n", " ").split())[:2000]


def repo_path(relative: str) -> Path:
    candidate = (ROOT / Path(*PurePosixPath(relative).parts)).resolve()
    require(candidate == ROOT or ROOT in candidate.parents, f"repository path escaped: {relative}")
    return candidate


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def read_json(path: Path) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except Exception as error:
        raise ValidationError(f"malformed JSON {path.name}: {clean_error(error)}") from error
    require(isinstance(value, dict), f"{path.name}: root must be object")
    return value


def finite(value: Any, label: str) -> float:
    require(isinstance(value, (int, float)) and not isinstance(value, bool), f"{label}: number required")
    number = float(value)
    require(math.isfinite(number), f"{label}: finite number required")
    return number


def vector(value: Any, length: int, label: str) -> list[float]:
    require(isinstance(value, list) and len(value) == length, f"{label}: expected length {length}")
    return [finite(item, f"{label}[{index}]") for index, item in enumerate(value)]


def max_abs(values: Iterable[float]) -> float:
    return max((abs(float(value)) for value in values), default=0.0)


def norm(values: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in values))


def same_vector(left: Sequence[float], right: Sequence[float], tolerance: float = 1.0e-12) -> bool:
    return len(left) == len(right) and max_abs(a - b for a, b in zip(left, right)) <= tolerance


def require_close(actual: Any, expected: Any, label: str, *, absolute: float = 2.0e-12, relative: float = 2.0e-10) -> None:
    if isinstance(expected, bool) or expected is None or isinstance(expected, str):
        require(actual == expected, f"{label}: {actual!r} != {expected!r}")
        return
    if isinstance(expected, (int, float)):
        a, e = finite(actual, label), finite(expected, label + ".expected")
        require(abs(a - e) <= absolute + relative * max(abs(a), abs(e)), f"{label}: {a!r} != {e!r}")
        return
    if isinstance(expected, list):
        require(isinstance(actual, list) and len(actual) == len(expected), f"{label}: list shape differs")
        for index, item in enumerate(expected):
            require_close(actual[index], item, f"{label}[{index}]", absolute=absolute, relative=relative)
        return
    if isinstance(expected, dict):
        require(isinstance(actual, dict), f"{label}: object required")
        require(set(expected) <= set(actual), f"{label}: missing keys {sorted(set(expected) - set(actual))}")
        for key, item in expected.items():
            require_close(actual[key], item, f"{label}.{key}", absolute=absolute, relative=relative)
        return
    raise ValidationError(f"{label}: unsupported comparison type {type(expected).__name__}")


def run_process(command: Sequence[str], *, timeout: float = 300.0) -> subprocess.CompletedProcess[str]:
    try:
        return subprocess.run(list(command), cwd=ROOT, text=True, encoding="utf-8", errors="replace", capture_output=True, timeout=timeout, check=False)
    except subprocess.TimeoutExpired as error:
        raise ValidationError(f"command timed out after {timeout:g}s: {Path(command[0]).name}") from error


def joint_limit_contract() -> dict[str, tuple[float, float] | None]:
    root = ET.parse(repo_path(MJCF_REL)).getroot()
    nodes = {str(node.get("name")): node for node in root.findall(".//joint") if node.get("name") in JOINTS}
    require(set(nodes) == set(JOINTS), "production six-joint identity changed")
    result: dict[str, tuple[float, float] | None] = {}
    for name in JOINTS:
        node = nodes[name]
        if str(node.get("limited", "auto")).lower() == "false" or not node.get("range"):
            require(name == "J1", f"unexpected unlimited joint: {name}")
            result[name] = None
        else:
            bounds = tuple(float(token) for token in str(node.get("range")).split())
            require(len(bounds) == 2 and bounds[0] < bounds[1], f"invalid range for {name}")
            result[name] = (bounds[0], bounds[1])
    return result


def minimum_margin(q: Sequence[float], limits: Mapping[str, tuple[float, float] | None]) -> float:
    values = [min(q[index] - bounds[0], bounds[1] - q[index]) for index, name in enumerate(JOINTS) if (bounds := limits[name]) is not None]
    return min(values)


def validate_protected_inputs() -> dict[str, str]:
    for relative, expected in PROTECTED_HASHES.items():
        path = repo_path(relative)
        require(path.is_file(), f"protected input missing: {relative}")
        require(sha256_file(path) == expected, f"protected input changed: {relative}")
    xml = ET.parse(repo_path(MJCF_REL)).getroot()
    option = xml.find("./option")
    require(option is not None, "production MJCF option missing")
    require(tuple(float(token) for token in str(option.get("gravity", "")).split()) == (0.0, 0.0, 0.0), "production gravity was persisted")
    require(abs(float(option.get("timestep", "nan")) - 0.002) < 1.0e-15, "production timestep changed")
    for joint in xml.findall(".//joint"):
        if joint.get("name") in JOINTS:
            for key in ("damping", "frictionloss", "armature"):
                require(abs(float(joint.get(key, "0"))) < 1.0e-15, f"new passive parameter on {joint.get('name')}: {key}")
    return dict(PROTECTED_HASHES)


def production_asset_snapshot() -> dict[str, str]:
    source_path = repo_path(MJCF_REL)
    root = ET.fromstring(source_path.read_bytes())
    compiler = root.find("./compiler")
    mesh_dir = source_path.parent / (compiler.get("meshdir", ".") if compiler is not None else ".")
    texture_dir = source_path.parent / (compiler.get("texturedir", ".") if compiler is not None else ".")
    assets: dict[str, str] = {}
    nodes = [node for node in root.findall("./asset/*") if node.get("file")]
    require(len(nodes) == 1008, "production model no longer references exactly 1008 assets")
    for node in nodes:
        raw = str(node.get("file"))
        require(not Path(raw).is_absolute(), f"absolute production asset path: {raw}")
        base = mesh_dir if node.tag.rsplit("}", 1)[-1] == "mesh" else texture_dir
        source = (base / Path(*PurePosixPath(raw).parts)).resolve()
        require(source.is_file() and ROOT in source.parents, f"missing/escaped production asset: {raw}")
        relative = source.relative_to(ROOT).as_posix()
        require(relative not in assets, f"duplicate production asset: {relative}")
        assets[relative] = sha256_file(source)
    require(len(assets) == 1008, "production asset identity set is not exact 1008")
    return dict(sorted(assets.items()))


def make_ascii_model_mirror(destination: Path, expected_assets: Mapping[str, str]) -> Path:
    source_path = repo_path(MJCF_REL)
    source_bytes = source_path.read_bytes()
    require(sha256_bytes(source_bytes) == MJCF_SHA256, "production MJCF changed before mirror")
    root = ET.fromstring(source_bytes)
    compiler = root.find("./compiler")
    mesh_dir = source_path.parent / (compiler.get("meshdir", ".") if compiler is not None else ".")
    texture_dir = source_path.parent / (compiler.get("texturedir", ".") if compiler is not None else ".")
    assets_dir = destination / "assets"
    assets_dir.mkdir(parents=True, exist_ok=True)
    copied: set[str] = set()
    nodes = [node for node in root.findall("./asset/*") if node.get("file")]
    require(len(nodes) == 1008, "production asset count changed while mirroring")
    for index, node in enumerate(nodes):
        raw = str(node.get("file"))
        base = mesh_dir if node.tag.rsplit("}", 1)[-1] == "mesh" else texture_dir
        source = (base / Path(*PurePosixPath(raw).parts)).resolve()
        relative = source.relative_to(ROOT).as_posix()
        require(relative in expected_assets and sha256_file(source) == expected_assets[relative], f"asset changed before mirror copy: {relative}")
        target = assets_dir / f"asset_{index:04d}{source.suffix.lower() or '.bin'}"
        shutil.copyfile(source, target)
        require(sha256_file(target) == expected_assets[relative], f"mirror asset differs: {relative}")
        copied.add(relative)
        node.set("file", "assets/" + target.name)
    require(copied == set(expected_assets), "mirror asset identity set differs")
    if compiler is None:
        compiler = ET.Element("compiler")
        root.insert(0, compiler)
    compiler.set("meshdir", ".")
    compiler.set("texturedir", ".")
    target_model = destination / "model.xml"
    ET.ElementTree(root).write(target_model, encoding="utf-8", xml_declaration=True)
    require(str(target_model).isascii(), "MuJoCo mirror path is not ASCII")
    require(sha256_file(source_path) == MJCF_SHA256, "production MJCF changed during mirror")
    return target_model


def derive_v15_18a_candidates() -> tuple[list[dict[str, Any]], dict[str, tuple[float, float] | None]]:
    report = read_json(repo_path(V15_18A_JSON_REL))
    require(report.get("audit_valid") is True and report.get("pass") is True, "V15.18A authority is not PASS")
    require(report.get("final_status") == "V15.18A STATIC_GRAVITY = PASS", "V15.18A final status changed")
    selected = report.get("pose_selection", {}).get("selected")
    poses = report.get("poses")
    require(isinstance(selected, list) and isinstance(poses, list) and len(selected) == len(poses) == 21, "V15.18A exact 21-pose authority missing")
    pose_rows = {str(row.get("id")): row for row in poses if isinstance(row, dict)}
    require(len(pose_rows) == 21, "V15.18A pose ids are not exact unique 21")
    limits = joint_limit_contract()
    candidates: list[dict[str, Any]] = []
    for selection in selected:
        require(isinstance(selection, dict), "V15.18A selection row malformed")
        pose_id = str(selection.get("id"))
        row = pose_rows.get(pose_id)
        require(row is not None and row.get("pass") is True, f"V15.18A selected pose not PASS: {pose_id}")
        q = vector(selection.get("q_rad"), 6, f"V15.18A.{pose_id}.q")
        require_close(row.get("q_rad"), q, f"V15.18A.{pose_id}.pose_q", absolute=0.0, relative=0.0)
        margin = minimum_margin(q, limits)
        require(margin >= 0.05, f"V15.18A pose below frozen minimum margin: {pose_id}")
        torque = vector(row.get("tau_hold_mujoco_nm"), 6, f"V15.18A.{pose_id}.tau")
        candidates.append({
            "id": pose_id,
            "q_rad": q,
            "role": str(selection.get("role")),
            "source_global_index": selection.get("source_global_index"),
            "source_collision_free": selection.get("source_collision_free") is True or pose_id == "v15_14_accepted_fjt_final_pose",
            "initial_margin_rad": margin,
            "gravity_torque_norm_nm": norm(torque),
        })
    return candidates, limits


def _child_json_safe(value: Any, np: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, dict):
        return {str(key): _child_json_safe(item, np) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_child_json_safe(item, np) for item in value]
    return value


def _child_all_finite(value: Any, np: Any) -> bool:
    if isinstance(value, dict):
        return all(_child_all_finite(item, np) for item in value.values())
    if isinstance(value, (list, tuple)):
        return all(_child_all_finite(item, np) for item in value)
    if isinstance(value, np.ndarray):
        return bool(np.all(np.isfinite(value)))
    if isinstance(value, (int, float, np.generic)) and not isinstance(value, bool):
        return bool(np.isfinite(value))
    return True


def _child_configure(model: Any, mujoco: Any, dt: float, integrator: str = "implicitfast") -> None:
    model.opt.gravity[:] = GRAVITY
    model.opt.timestep = dt
    integrators = {
        "implicitfast": int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST),
        "RK4": int(mujoco.mjtIntegrator.mjINT_RK4),
    }
    require(integrator in integrators, f"unsupported runtime-only integrator: {integrator}")
    model.opt.integrator = integrators[integrator]
    model.opt.enableflags |= int(mujoco.mjtEnableBit.mjENBL_ENERGY)
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)


def _child_energy(model: Any, data: Any, mujoco: Any, np: Any) -> tuple[list[float], float, float, float, float]:
    mujoco.mj_energyPos(model, data)
    mujoco.mj_energyVel(model, data)
    gravity = np.asarray(model.opt.gravity, dtype=float)
    positive = [body for body in range(1, model.nbody) if model.body_mass[body] > 0.0]
    authority = [mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name) for name in AUTHORITY_BODIES]
    require(all(body >= 0 for body in authority), "compiled authority body set differs")
    potential = -sum(float(model.body_mass[body]) * float(np.dot(gravity, data.xipos[body])) for body in positive)
    authority_potential = -sum(float(model.body_mass[body]) * float(np.dot(gravity, data.xipos[body])) for body in authority)
    mass = np.empty((model.nv, model.nv), dtype=float)
    mujoco.mj_fullM(model, data, mass)
    kinetic = 0.5 * float(data.qvel @ mass @ data.qvel)
    mujoco_energy = np.asarray(data.energy, dtype=float).copy().tolist()
    require(len(mujoco_energy) == 2, "MuJoCo energy vector shape differs")
    return [mujoco_energy[0], mujoco_energy[1], mujoco_energy[0] + mujoco_energy[1]], potential, authority_potential, kinetic, potential + kinetic


def _child_active_joint_limits(data: Any, mujoco: Any, np: Any) -> int:
    if int(data.nefc) == 0:
        return 0
    expected = int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
    return int(np.count_nonzero(np.asarray(data.efc_type[: data.nefc], dtype=int) == expected))


def _child_minimum_margin(model: Any, data: Any) -> float:
    margins: list[float] = []
    for joint_id in range(model.njnt):
        if int(model.jnt_type[joint_id]) != 3 or not bool(model.jnt_limited[joint_id]):
            continue
        address = int(model.jnt_qposadr[joint_id])
        low, high = (float(value) for value in model.jnt_range[joint_id])
        margins.append(min(float(data.qpos[address]) - low, high - float(data.qpos[address])))
    return min(margins)


def _child_joint_margins(model: Any, data: Any, mujoco: Any) -> dict[str, float | None]:
    result: dict[str, float | None] = {}
    for name in JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        require(joint_id >= 0, f"compiled joint missing: {name}")
        if not bool(model.jnt_limited[joint_id]):
            result[name] = None
        else:
            address = int(model.jnt_qposadr[joint_id])
            low, high = (float(value) for value in model.jnt_range[joint_id])
            result[name] = min(float(data.qpos[address]) - low, high - float(data.qpos[address]))
    return result


def _child_contact_rows(model: Any, data: Any, mujoco: Any, limit: int = 12) -> list[dict[str, Any]]:
    rows: list[dict[str, Any]] = []
    for index in range(min(int(data.ncon), limit)):
        contact = data.contact[index]
        geom1 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or f"geom_{int(contact.geom1)}"
        geom2 = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or f"geom_{int(contact.geom2)}"
        rows.append({"geom1": geom1, "geom2": geom2, "distance_m": float(contact.dist), "ground": "ground" in {geom1, geom2}})
    return rows


def _child_reset(model: Any, data: Any, mujoco: Any, np: Any, q: Sequence[float]) -> dict[str, Any]:
    mujoco.mj_resetData(model, data)
    data.qpos[:] = np.asarray(q, dtype=float)
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    if model.nu:
        data.ctrl[:] = 0.0
    data.qfrc_applied[:] = 0.0
    data.xfrc_applied[:] = 0.0
    mujoco.mj_forward(model, data)
    mass = np.empty((model.nv, model.nv), dtype=float)
    mujoco.mj_fullM(model, data, mass)
    bias = np.asarray(data.qfrc_bias, dtype=float).copy()
    actual = np.asarray(data.qacc, dtype=float).copy()
    expected = -np.linalg.solve(mass, bias)
    residual = mass @ actual + bias
    energy, potential, authority_potential, kinetic, total = _child_energy(model, data, mujoco, np)
    positive_rows = []
    for body in range(1, model.nbody):
        if model.body_mass[body] <= 0.0:
            continue
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body)
        positive_rows.append({"body": name, "body_id": body, "mass_kg": float(model.body_mass[body]), "com_world_m": np.asarray(data.xipos[body], dtype=float).copy()})
    result = {
        "qpos_rad": np.asarray(data.qpos, dtype=float).copy(),
        "qvel_rad_s": np.asarray(data.qvel, dtype=float).copy(),
        "qfrc_bias_nm": bias,
        "qacc_rad_s2": actual,
        "expected_qacc_rad_s2": expected,
        "mass_matrix": mass,
        "residual_vector": residual,
        "residual_norm": float(np.linalg.norm(residual)),
        "relative_qacc_error": float(np.linalg.norm(actual - expected) / max(float(np.linalg.norm(expected)), 1.0e-30)),
        "mujoco_energy_j": energy,
        "mujoco_potential_j": energy[0],
        "mujoco_kinetic_j": energy[1],
        "independent_potential_j": potential,
        "independent_all_positive_potential_j": potential,
        "independent_authority_potential_j": authority_potential,
        "independent_kinetic_j": kinetic,
        "independent_total_j": total,
        "body_com_world_m": positive_rows,
        "contacts": _child_contact_rows(model, data, mujoco),
        "ncon": int(data.ncon),
        "nefc": int(data.nefc),
        "active_joint_limit_constraint_count": _child_active_joint_limits(data, mujoco, np),
        "minimum_joint_limit_margin_rad": _child_minimum_margin(model, data),
        "joint_limit_margin_rad": _child_joint_margins(model, data, mujoco),
        "qfrc_actuator_nm": np.asarray(data.qfrc_actuator, dtype=float).copy(),
        "qfrc_passive_nm": np.asarray(data.qfrc_passive, dtype=float).copy(),
        "ctrl": np.asarray(data.ctrl, dtype=float).copy(),
        "qfrc_applied_nm": np.asarray(data.qfrc_applied, dtype=float).copy(),
        "xfrc_applied": np.asarray(data.xfrc_applied, dtype=float).copy(),
        "max_abs_qfrc_actuator_nm": float(np.max(np.abs(data.qfrc_actuator), initial=0.0)),
        "max_abs_qfrc_passive_nm": float(np.max(np.abs(data.qfrc_passive), initial=0.0)),
        "max_abs_qfrc_applied_nm": float(np.max(np.abs(data.qfrc_applied), initial=0.0)),
        "max_abs_xfrc_applied": float(np.max(np.abs(data.xfrc_applied), initial=0.0)),
    }
    result["all_values_finite"] = _child_all_finite(result, np)
    return _child_json_safe(result, np)


def _child_load_model(model_path: Path, dt: float, mujoco: Any, np: Any, integrator: str = "implicitfast") -> Any:
    model = mujoco.MjModel.from_xml_path(str(model_path))
    _child_configure(model, mujoco, dt, integrator)
    require(model.nq == model.nv == 6, "compiled model is not exact six scalar DOFs")
    names = tuple(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index) for index in range(model.njnt))
    require(names == JOINTS, f"compiled joint order changed: {names}")
    require(model.nu == 0, "production passive model unexpectedly has actuators")
    require(float(np.max(np.abs(model.dof_damping), initial=0.0)) == 0.0, "compiled damping is nonzero")
    require(float(np.max(np.abs(model.dof_frictionloss), initial=0.0)) == 0.0, "compiled frictionloss is nonzero")
    require(float(np.max(np.abs(model.dof_armature), initial=0.0)) == 0.0, "compiled armature is nonzero")
    return model


def _child_run(model: Any, q: Sequence[float], dt: float, label: str, mujoco: Any, np: Any) -> dict[str, Any]:
    require(abs(float(model.opt.timestep) - dt) < 1.0e-15, f"{label}: model timestep differs")
    data = mujoco.MjData(model)
    initial = _child_reset(model, data, mujoco, np, q)
    step_count = int(round(DURATION_S / dt))
    require(step_count > 0 and abs(step_count * dt - DURATION_S) < 1.0e-14 and DURATION_S <= 0.25, "duration/dt contract invalid")

    times = [0.0]
    qpos = [np.asarray(data.qpos, dtype=float).copy()]
    qvel = [np.asarray(data.qvel, dtype=float).copy()]
    qacc = [np.asarray(data.qacc, dtype=float).copy()]
    energy, potential, authority_potential, kinetic, total = _child_energy(model, data, mujoco, np)
    energies, potentials, authority_potentials, kinetics, totals = [energy], [potential], [authority_potential], [kinetic], [total]
    ncon = [int(data.ncon)]
    active_limits = [_child_active_joint_limits(data, mujoco, np)]
    margins = [_child_minimum_margin(model, data)]
    actuator = [float(np.max(np.abs(data.qfrc_actuator), initial=0.0))]
    passive = [float(np.max(np.abs(data.qfrc_passive), initial=0.0))]
    applied = [float(np.max(np.abs(data.qfrc_applied), initial=0.0))]
    xapplied = [float(np.max(np.abs(data.xfrc_applied), initial=0.0))]
    ctrl = [float(np.max(np.abs(data.ctrl), initial=0.0))]
    first_contacts = _child_contact_rows(model, data, mujoco) if data.ncon else []

    for index in range(1, step_count + 1):
        mujoco.mj_step(model, data)
        mujoco.mj_forward(model, data)
        energy, potential, authority_potential, kinetic, total = _child_energy(model, data, mujoco, np)
        times.append(index * dt)
        qpos.append(np.asarray(data.qpos, dtype=float).copy())
        qvel.append(np.asarray(data.qvel, dtype=float).copy())
        qacc.append(np.asarray(data.qacc, dtype=float).copy())
        energies.append(energy)
        potentials.append(potential)
        authority_potentials.append(authority_potential)
        kinetics.append(kinetic)
        totals.append(total)
        ncon.append(int(data.ncon))
        active_limits.append(_child_active_joint_limits(data, mujoco, np))
        margins.append(_child_minimum_margin(model, data))
        actuator.append(float(np.max(np.abs(data.qfrc_actuator), initial=0.0)))
        passive.append(float(np.max(np.abs(data.qfrc_passive), initial=0.0)))
        applied.append(float(np.max(np.abs(data.qfrc_applied), initial=0.0)))
        xapplied.append(float(np.max(np.abs(data.xfrc_applied), initial=0.0)))
        ctrl.append(float(np.max(np.abs(data.ctrl), initial=0.0)))
        if data.ncon and not first_contacts:
            first_contacts = _child_contact_rows(model, data, mujoco)

    qa, qv, qaa = np.asarray(qpos), np.asarray(qvel), np.asarray(qacc)
    ea, up, ke, et = np.asarray(energies), np.asarray(potentials), np.asarray(kinetics), np.asarray(totals)
    result = {
        "label": label,
        "dt_s": dt,
        "duration_s": DURATION_S,
        "step_count": step_count,
        "time_s": times,
        "qpos_rad": qa,
        "qvel_rad_s": qv,
        "qacc_rad_s2": qaa,
        "initial_qacc_rad_s2": qaa[0],
        "mujoco_energy_j": ea,
        "independent_potential_j": up,
        "independent_authority_potential_j": authority_potentials,
        "independent_kinetic_j": ke,
        "independent_total_j": et,
        "final_qpos_rad": qa[-1],
        "final_qvel_rad_s": qv[-1],
        "max_abs_qvel_rad_s": float(np.max(np.abs(qv), initial=0.0)),
        "max_abs_qacc_rad_s2": float(np.max(np.abs(qaa), initial=0.0)),
        "max_abs_qpos_step_rad": float(np.max(np.abs(np.diff(qa, axis=0)), initial=0.0)),
        "max_abs_qvel_step_rad_s": float(np.max(np.abs(np.diff(qv, axis=0)), initial=0.0)),
        "max_abs_qacc_step_rad_s2": float(np.max(np.abs(np.diff(qaa, axis=0)), initial=0.0)),
        "max_abs_qfrc_actuator_nm": max(actuator),
        "max_abs_qfrc_passive_nm": max(passive),
        "max_abs_qfrc_applied_nm": max(applied),
        "max_abs_xfrc_applied": max(xapplied),
        "max_abs_ctrl": max(ctrl),
        "contact_count_max": max(ncon),
        "active_joint_limit_constraint_count_max": max(active_limits),
        "minimum_joint_limit_margin_rad": min(margins),
        "first_contact_rows": first_contacts,
    }
    result["all_values_finite"] = _child_all_finite(result, np)
    result["raw_safety"] = {
        "all_values_finite": result["all_values_finite"],
        "no_contacts": result["contact_count_max"] == 0,
        "no_active_joint_limit_constraints": result["active_joint_limit_constraint_count_max"] == 0,
        "joint_limit_margin_pass": result["minimum_joint_limit_margin_rad"] > RUNTIME_MARGIN_RAD,
        "qvel_pass": result["max_abs_qvel_rad_s"] < MAX_QVEL,
        "qacc_pass": result["max_abs_qacc_rad_s2"] < MAX_QACC,
        "qpos_step_pass": result["max_abs_qpos_step_rad"] < MAX_QPOS_STEP,
        "qvel_step_pass": result["max_abs_qvel_step_rad_s"] < MAX_QVEL_STEP,
        "qacc_step_continuity_pass": result["max_abs_qacc_step_rad_s2"] < 50.0,
        "actuation_passive_and_applied_zero": (
            result["max_abs_qfrc_actuator_nm"] < ACTUATOR_TOL_NM
            and result["max_abs_qfrc_passive_nm"] < ACTUATOR_TOL_NM
            and result["max_abs_qfrc_applied_nm"] < ACTUATOR_TOL_NM
            and result["max_abs_xfrc_applied"] < ACTUATOR_TOL_NM
            and result["max_abs_ctrl"] < ACTUATOR_TOL_NM
        ),
    }
    result["raw_safety"]["pass"] = all(bool(value) for value in result["raw_safety"].values())
    return _child_json_safe(result, np)


def _child_selection_preview(model: Any, q: Sequence[float], mujoco: Any, np: Any) -> dict[str, Any]:
    """Ballistic q-space sweep using mj_forward only; never call mj_step."""
    data = mujoco.MjData(model)
    _child_reset(model, data, mujoco, np, q)
    q0 = np.asarray(q, dtype=float)
    qacc0 = np.asarray(data.qacc, dtype=float).copy()
    initial_ncon = int(data.ncon)
    initial_limits = _child_active_joint_limits(data, mujoco, np)
    initial_margin = _child_minimum_margin(model, data)
    initial_contacts = _child_contact_rows(model, data, mujoco)
    scale = 1.1
    displacement = 0.5 * np.abs(qacc0) * DURATION_S * DURATION_S * scale
    predicted_final_qvel = qacc0 * DURATION_S * scale
    segment_count = 30
    times = np.linspace(0.0, DURATION_S, segment_count + 1)
    sweep_rows = []
    first_contacts: list[dict[str, Any]] = []
    maximum_contacts = 0
    maximum_limits = 0
    minimum_margin_value = initial_margin
    all_finite = bool(np.all(np.isfinite(q0)) and np.all(np.isfinite(qacc0)))
    previous = q0.copy()
    max_segment = 0.0
    for index, sample_time in enumerate(times):
        predicted = q0 + 0.5 * qacc0 * float(sample_time) * float(sample_time) * scale
        mujoco.mj_resetData(model, data)
        data.qpos[:] = predicted
        data.qvel[:] = 0.0
        data.qacc[:] = 0.0
        if model.nu:
            data.ctrl[:] = 0.0
        data.qfrc_applied[:] = 0.0
        data.xfrc_applied[:] = 0.0
        mujoco.mj_forward(model, data)
        ncon = int(data.ncon)
        active = _child_active_joint_limits(data, mujoco, np)
        margin = _child_minimum_margin(model, data)
        finite_row = bool(all(np.all(np.isfinite(array)) for array in (data.qpos, data.qvel, data.qacc, data.qfrc_bias)))
        maximum_contacts = max(maximum_contacts, ncon)
        maximum_limits = max(maximum_limits, active)
        minimum_margin_value = min(minimum_margin_value, margin)
        all_finite = all_finite and finite_row
        if index:
            max_segment = max(max_segment, float(np.max(np.abs(predicted - previous))))
        if ncon and not first_contacts:
            first_contacts = _child_contact_rows(model, data, mujoco)
        sweep_rows.append({
            "index": index, "time_s": float(sample_time), "q_rad": predicted,
            "ncon": ncon, "active_joint_limit_constraint_count": active,
            "minimum_joint_limit_margin_rad": margin, "all_values_finite": finite_row,
        })
        previous = predicted.copy()
    collision_safe = initial_ncon == 0 and maximum_contacts == 0
    prediction_safe = minimum_margin_value > RUNTIME_MARGIN_RAD
    qvel_safe = max_abs(predicted_final_qvel) < MAX_QVEL
    qacc_safe = max_abs(qacc0) < MAX_QACC
    passed = initial_margin >= INITIAL_MARGIN_RAD and collision_safe and initial_limits == 0 and maximum_limits == 0 and prediction_safe and qvel_safe and qacc_safe and all_finite
    conditions = (
        (initial_margin < INITIAL_MARGIN_RAD, "FAIL_INITIAL_MARGIN_LT_0P15"),
        (initial_ncon != 0, "FAIL_INITIAL_CONTACT"),
        (initial_limits != 0, "FAIL_INITIAL_ACTIVE_JOINT_LIMIT"),
        (maximum_contacts != 0, "FAIL_QSPACE_PREDICTED_CONTACT"),
        (maximum_limits != 0, "FAIL_QSPACE_PREDICTED_ACTIVE_JOINT_LIMIT"),
        (not prediction_safe, "FAIL_CONSERVATIVE_QACC_MARGIN_PREDICTION"),
        (not qvel_safe, "FAIL_CONSERVATIVE_QACC_QVEL_PREDICTION"),
        (not qacc_safe, "FAIL_INITIAL_QACC_STABILITY_LIMIT"),
        (not all_finite, "FAIL_QSPACE_PREDICTION_NONFINITE"),
    )
    return _child_json_safe({
        "method": "INITIAL_QACC_BALLISTIC_PREDICTION_PLUS_RESET_SET_MJ_FORWARD_QSPACE_SWEEP",
        "prediction_formula": "q(t)=q0+0.5*1.1*qacc_initial*t^2; qvel_bound(t)=1.1*qacc_initial*t",
        "prediction_scale": scale, "duration_s": DURATION_S, "segment_count": segment_count,
        "sample_count": segment_count + 1, "time_grid_s": times,
        "candidate_free_dynamics_integration_performed": False, "mj_step_call_count": 0,
        "qvel_for_collision_forward_rad_s": [0.0] * 6,
        "initial_qacc_rad_s2": qacc0, "initial_margin_rad": initial_margin,
        "initial_contact_count": initial_ncon, "initial_contact_rows": initial_contacts,
        "initial_active_joint_limit_constraint_count": initial_limits,
        "per_joint_conservative_displacement_bound_rad": displacement,
        "predicted_final_q_rad": q0 + 0.5 * qacc0 * DURATION_S * DURATION_S * scale,
        "predicted_final_qvel_rad_s": predicted_final_qvel,
        "minimum_conservative_predicted_margin_rad": minimum_margin_value,
        "contact_count_max": maximum_contacts, "first_predicted_contact_rows": first_contacts,
        "active_joint_limit_constraint_count_max": maximum_limits,
        "max_abs_predicted_q_segment_rad": max_segment,
        "max_abs_predicted_qvel_rad_s": max_abs(predicted_final_qvel),
        "max_abs_initial_qacc_rad_s2": max_abs(qacc0), "sweep_rows": sweep_rows,
        "all_values_finite": all_finite, "pass": passed,
        "failure_codes": [code for failed, code in conditions if failed],
    }, np)


def physics_child(model_path: Path, request_path: Path, output_path: Path) -> int:
    try:
        import mujoco  # type: ignore
        import numpy as np  # type: ignore

        require(mujoco.__version__ == "3.11.0", f"MuJoCo version must be 3.11.0, got {mujoco.__version__}")
        require(np.__version__ == "2.2.6", f"NumPy version must be 2.2.6, got {np.__version__}")
        request = read_json(request_path)
        mode = request.get("mode")
        rows = request.get("poses")
        require(isinstance(rows, list), "physics child poses must be a list")
        output_rows: list[dict[str, Any]] = []
        if mode == "selection":
            model_a = _child_load_model(model_path, 0.001, mujoco, np)
            for row in rows:
                require(isinstance(row, dict), "physics child pose row malformed")
                q = vector(row.get("q_rad"), 6, f"child.{row.get('id')}.q")
                preview = _child_selection_preview(model_a, q, mujoco, np)
                output_rows.append({"id": row.get("id"), "q_rad": q, "preview": preview})
        elif mode == "official_ac":
            model_a = _child_load_model(model_path, 0.002, mujoco, np)
            for row in rows:
                require(isinstance(row, dict), "physics child pose row malformed")
                q = vector(row.get("q_rad"), 6, f"child.{row.get('id')}.q")
                data = mujoco.MjData(model_a)
                initial = _child_reset(model_a, data, mujoco, np, q)
                output_rows.append({
                    "slot_index": row.get("slot_index"), "category": row.get("category"),
                    "selected_pose_id": row.get("id"), "selected_q_rad": q,
                    "initial_state": initial,
                    "raw_runs": {
                        "A": _child_run(model_a, q, 0.002, "A", mujoco, np),
                        "C": _child_run(model_a, q, 0.002, "C", mujoco, np),
                    },
                })
        elif mode == "official_b":
            model_b = _child_load_model(model_path, 0.001, mujoco, np)
            for row in rows:
                require(isinstance(row, dict), "physics child pose row malformed")
                q = vector(row.get("q_rad"), 6, f"child.{row.get('id')}.q")
                output_rows.append({
                    "slot_index": row.get("slot_index"), "category": row.get("category"),
                    "selected_pose_id": row.get("id"), "selected_q_rad": q,
                    "raw_runs": {"B": _child_run(model_b, q, 0.001, "B", mujoco, np)},
                })
        elif mode == "attribution_configuration":
            integrator = str(request.get("integrator"))
            dt = finite(request.get("dt_s"), "physics child attribution dt")
            label = str(request.get("label"))
            expected = {key: (kind, step) for key, kind, step in ATTRIBUTION_CONFIGURATIONS}
            require(label in expected and expected[label] == (integrator, dt), "attribution configuration differs from frozen internal matrix")
            model_configuration = _child_load_model(model_path, dt, mujoco, np, integrator)
            for row in rows:
                require(isinstance(row, dict), "physics child pose row malformed")
                q = vector(row.get("q_rad"), 6, f"child.{row.get('id')}.q")
                output_row = {
                    "slot_index": row.get("slot_index"), "category": row.get("category"),
                    "selected_pose_id": row.get("id"), "selected_q_rad": q,
                    "raw_run": _child_run(model_configuration, q, dt, label, mujoco, np),
                }
                if label == "A":
                    initial_data = mujoco.MjData(model_configuration)
                    output_row["initial_state"] = _child_reset(model_configuration, initial_data, mujoco, np, q)
                output_rows.append(output_row)
        else:
            raise ValidationError(f"unknown physics child mode: {mode!r}")
        result = {"mujoco_version": mujoco.__version__, "numpy_version": np.__version__, "mode": mode, "rows": output_rows}
        if mode == "attribution_configuration":
            result.update({"label": label, "integrator": integrator, "dt_s": dt})
        output_path.write_text(json.dumps(_child_json_safe(result, np), ensure_ascii=False, sort_keys=True, separators=(",", ":")), encoding="utf-8")
        return 0
    except Exception as error:
        print(f"PHYSICS_CHILD=FAIL\nERROR={clean_error(error)}", file=sys.stderr)
        return 2


def merge_official_outputs(ac: Mapping[str, Any], b: Mapping[str, Any]) -> dict[str, Any]:
    ac_rows, b_rows = ac.get("rows"), b.get("rows")
    require(isinstance(ac_rows, list) and isinstance(b_rows, list) and len(ac_rows) == len(b_rows) == 6, "split official output row count differs")
    merged: list[dict[str, Any]] = []
    for left, right in zip(ac_rows, b_rows):
        require(isinstance(left, dict) and isinstance(right, dict), "split official row malformed")
        for key in ("slot_index", "category", "selected_pose_id", "selected_q_rad"):
            require_close(left.get(key), right.get(key), f"split official.{key}", absolute=0.0, relative=0.0)
        require(isinstance(left.get("raw_runs"), dict) and set(left["raw_runs"]) == {"A", "C"}, "official A/C run set differs")
        require(isinstance(right.get("raw_runs"), dict) and set(right["raw_runs"]) == {"B"}, "official B run set differs")
        row = dict(left)
        row["raw_runs"] = {"A": left["raw_runs"]["A"], "B": right["raw_runs"]["B"], "C": left["raw_runs"]["C"]}
        merged.append(row)
    require(ac.get("mujoco_version") == b.get("mujoco_version") and ac.get("numpy_version") == b.get("numpy_version"), "split official environment differs")
    return {"mujoco_version": ac.get("mujoco_version"), "numpy_version": ac.get("numpy_version"), "mode": "official", "rows": merged}


def select_mujoco_python(explicit: str | None) -> Path:
    candidates = (
        explicit,
        os.environ.get("V15_18B_MUJOCO_PYTHON"),
        sys.executable,
        shutil.which("python3"),
        shutil.which("python"),
    )
    seen: set[Path] = set()
    for candidate in candidates:
        if not candidate:
            continue
        path = Path(candidate).expanduser().resolve()
        if path in seen or not path.is_file():
            continue
        seen.add(path)
        result = run_process([str(path), "-c", "import mujoco,numpy; print(mujoco.__version__, numpy.__version__)"], timeout=30.0)
        if result.returncode == 0 and result.stdout.strip() == "3.11.0 2.2.6":
            return path
    raise ValidationError("no MuJoCo 3.11.0 + NumPy 2.2.6 interpreter is available")


def run_physics_request(interpreter: Path, model: Path, request: Mapping[str, Any], directory: Path, stem: str) -> dict[str, Any]:
    request_path = directory / f"{stem}_request.json"
    output_path = directory / f"{stem}_output.json"
    request_path.write_text(json.dumps(request, ensure_ascii=False, sort_keys=True, separators=(",", ":")), encoding="utf-8")
    result = run_process(
        [str(interpreter), str(Path(__file__).resolve()), "--physics-child", str(model), str(request_path), str(output_path)],
        timeout=300.0,
    )
    require(result.returncode == 0 and output_path.is_file(), f"independent MuJoCo {stem} child failed: {clean_error(result.stderr or result.stdout)}")
    return read_json(output_path)


def run_attribution_recomputation(
    interpreter: Path,
    model: Path,
    poses: Sequence[Mapping[str, Any]],
    directory: Path,
) -> dict[str, Any]:
    """Run each B2 configuration in an isolated compiled-model child."""
    outputs: dict[str, dict[str, Any]] = {}
    for label, integrator, dt in ATTRIBUTION_CONFIGURATIONS:
        outputs[label] = run_physics_request(
            interpreter,
            model,
            {
                "mode": "attribution_configuration",
                "label": label,
                "integrator": integrator,
                "dt_s": dt,
                "poses": list(poses),
            },
            directory,
            "attribution_" + label,
        )
    labels = tuple(label for label, _, _ in ATTRIBUTION_CONFIGURATIONS)
    rows: list[dict[str, Any]] = []
    for index in range(6):
        first = outputs[labels[0]]["rows"][index]
        merged = {
            "slot_index": first["slot_index"],
            "category": first["category"],
            "selected_pose_id": first["selected_pose_id"],
            "selected_q_rad": first["selected_q_rad"],
            "initial_state": first.get("initial_state"),
            "raw_runs": {},
        }
        require(isinstance(merged["initial_state"], dict), "B2 A-profile initial state missing")
        for label, integrator, dt in ATTRIBUTION_CONFIGURATIONS:
            output = outputs[label]
            require(output.get("label") == label and output.get("integrator") == integrator, f"{label}: child runtime identity differs")
            require_close(output.get("dt_s"), dt, f"{label}: child dt", absolute=0.0, relative=0.0)
            output_rows = output.get("rows")
            require(isinstance(output_rows, list) and len(output_rows) == 6, f"{label}: child row count differs")
            row = output_rows[index]
            for key in ("slot_index", "category", "selected_pose_id", "selected_q_rad"):
                require_close(row.get(key), merged[key], f"{label}.{key}", absolute=0.0, relative=0.0)
            merged["raw_runs"][label] = row["raw_run"]
        rows.append(merged)
    versions = {(output.get("mujoco_version"), output.get("numpy_version")) for output in outputs.values()}
    require(len(versions) == 1, "B2 child environments differ")
    mujoco_version, numpy_version = next(iter(versions))
    return {
        "mujoco_version": mujoco_version,
        "numpy_version": numpy_version,
        "configurations": [
            {"label": label, "integrator": integrator, "dt_s": dt}
            for label, integrator, dt in ATTRIBUTION_CONFIGURATIONS
        ],
        "rows": rows,
    }


def max_final_difference(left: Mapping[str, Any], right: Mapping[str, Any], field: str) -> float:
    a = vector(left[field], 6, "attribution." + field + ".left")
    b = vector(right[field], 6, "attribution." + field + ".right")
    return max_abs(x - y for x, y in zip(a, b))


def derive_root_cause_confirmed(conditions: Mapping[str, Any]) -> bool:
    require(set(conditions) == set(ROOT_CAUSE_CONDITION_KEYS), "root-cause exact eleven-condition map differs")
    require(all(isinstance(conditions[key], bool) for key in ROOT_CAUSE_CONDITION_KEYS), "root-cause conditions must be booleans")
    return all(bool(conditions[key]) for key in ROOT_CAUSE_CONDITION_KEYS)


def validate_root_cause_claims(
    conditions: Mapping[str, Any],
    classification: Any,
    continuous_time_model_status: Any,
) -> bool:
    confirmed = derive_root_cause_confirmed(conditions)
    claims_confirmation = classification == ROOT_CAUSE_CLASSIFICATION
    claims_continuous_pass = continuous_time_model_status == CONTINUOUS_TIME_MODEL_PASS
    require(claims_confirmation is confirmed, "root-cause classification contradicts exact eleven-condition conjunction")
    require(claims_continuous_pass is confirmed, "continuous-time model status contradicts exact eleven-condition conjunction")
    return confirmed


def root_cause_mutation_self_test() -> dict[str, Any]:
    baseline = {key: True for key in ROOT_CAUSE_CONDITION_KEYS}
    require(validate_root_cause_claims(baseline, ROOT_CAUSE_CLASSIFICATION, CONTINUOUS_TIME_MODEL_PASS), "root-cause PASS baseline rejected")
    mutations = {
        "contact": "no_contacts_all_runs_pass",
        "identity": "initial_qacc_identity_pass",
        "finite": "all_values_finite_all_runs_pass",
        "determinism": "determinism_pass",
    }
    rejected: list[str] = []
    for name, key in mutations.items():
        conditions = dict(baseline)
        conditions[key] = False
        require(not derive_root_cause_confirmed(conditions), f"root-cause {name} mutation remained confirmed")
        contradiction_rejected = False
        try:
            validate_root_cause_claims(conditions, ROOT_CAUSE_CLASSIFICATION, CONTINUOUS_TIME_MODEL_PASS)
        except ValidationError:
            contradiction_rejected = True
        require(contradiction_rejected, f"root-cause {name} mutation accepted stale CONFIRMED/PASS claims")
        rejected.append(name)
    return {"pass": True, "exact_condition_count": len(ROOT_CAUSE_CONDITION_KEYS), "rejected_mutations": rejected}


def select_pose_slots(candidates: Sequence[Mapping[str, Any]], preflight: Mapping[str, Any], limits: Mapping[str, tuple[float, float] | None]) -> dict[str, Any]:
    del limits
    by_id = {str(row["id"]): dict(row) for row in candidates}
    require(len(by_id) == 21, "candidate id set is not exact 21")
    raw_rows = preflight.get("rows")
    require(isinstance(raw_rows, list) and len(raw_rows) == 21, "preflight did not return exact 21 rows")
    previews: dict[str, dict[str, Any]] = {}
    for raw in raw_rows:
        pose_id = str(raw.get("id"))
        require(pose_id in by_id and isinstance(raw.get("preview"), dict), f"preflight row malformed: {pose_id}")
        require(same_vector(vector(raw.get("q_rad"), 6, f"preflight.{pose_id}.q"), by_id[pose_id]["q_rad"]), f"preflight pose differs: {pose_id}")
        previews[pose_id] = dict(raw["preview"])

    selected_keys: set[tuple[float, ...]] = set()
    slots: list[dict[str, Any]] = []

    def compound(row: Mapping[str, Any]) -> bool:
        role = str(row["role"]).lower()
        return "coupled" in role or "combined" in role or "accepted_nonzero" in role

    def nearest(target: Mapping[str, Any]) -> Mapping[str, Any]:
        pool = [row for row in candidates if compound(row) and previews[str(row["id"])]["pass"] and tuple(row["q_rad"]) not in selected_keys and row["id"] != target["id"]]
        require(pool, f"no safe same-type substitute for {target['id']}")
        return min(pool, key=lambda row: (norm([a-b for a, b in zip(row["q_rad"], target["q_rad"])]), 10**9 if row["source_global_index"] is None else int(row["source_global_index"]), str(row["id"])))

    for slot_index, category, requested_id, alias in REQUESTED_SLOTS[:-1]:
        target = by_id[str(requested_id)]
        requested_preview = previews[str(requested_id)]
        substitution = not bool(requested_preview["pass"])
        if alias is not None and substitution:
            alias_slot = next(row for row in slots if row["slot_index"] == alias)
            chosen = by_id[str(alias_slot["selected_pose_id"])]
            reason = f"SEMANTIC_ALIAS_INHERITS_SLOT_{alias}_SAFETY_SUBSTITUTE"
            algorithm = "EXACT_ALIAS_OF_MAX_EXTENSION_SLOT_AFTER_SAFETY_GATE"
        elif substitution:
            chosen = nearest(target)
            reason = "REQUESTED_POSE_FAILED_0P15S_SAFETY_PREFLIGHT:" + ",".join(requested_preview["failure_codes"])
            algorithm = "V15_18A_ACCEPTED_SAFE_COMPOUND_MINIMUM_QSPACE_DISTANCE_THEN_GLOBAL_INDEX"
        else:
            chosen = target
            reason = None
            algorithm = "REQUESTED_V15_18A_POSE_RETAINED"
        selected_preview = previews[str(chosen["id"])]
        require(selected_preview["pass"], f"selected pose failed preview: {chosen['id']}")
        selected_keys.add(tuple(chosen["q_rad"]))
        slots.append({
            "slot_index": slot_index, "category": category, "requested_pose_id": requested_id,
            "requested_q_rad": list(target["q_rad"]), "selected_pose_id": chosen["id"],
            "selected_q_rad": list(chosen["q_rad"]), "semantic_alias_of_slot": alias,
            "substitution_applied": substitution, "substitution_reason": reason,
            "substitution_algorithm": algorithm, "source_global_index": chosen["source_global_index"],
            "source_role": chosen["role"], "source_collision_free": chosen["source_collision_free"],
            "initial_margin_rad": selected_preview["initial_margin_rad"], "preflight": selected_preview,
            "requested_pose_preflight": requested_preview, "pass": bool(selected_preview["pass"]),
            "failure_codes": [] if selected_preview["pass"] else list(selected_preview["failure_codes"]),
        })
    folded_pool = [row for row in candidates if previews[str(row["id"])]["pass"] and tuple(row["q_rad"]) not in selected_keys]
    require(folded_pool, "no distinct folded low-torque pose remains")
    folded = min(folded_pool, key=lambda row: (float(row["gravity_torque_norm_nm"]), 10**9 if row["source_global_index"] is None else int(row["source_global_index"]), str(row["id"])))
    folded_preview = previews[str(folded["id"])]
    slots.append({
        "slot_index": 6, "category": "folded_low_torque_pose", "requested_pose_id": None,
        "requested_q_rad": None, "selected_pose_id": folded["id"], "selected_q_rad": list(folded["q_rad"]),
        "semantic_alias_of_slot": None, "substitution_applied": False, "substitution_reason": None,
        "substitution_algorithm": "NOT_APPLICABLE_DETERMINISTIC_FOLDED_SELECTION",
        "source_global_index": folded["source_global_index"], "source_role": folded["role"],
        "source_collision_free": folded["source_collision_free"], "initial_margin_rad": folded_preview["initial_margin_rad"],
        "preflight": folded_preview, "requested_pose_preflight": None, "pass": bool(folded_preview["pass"]),
        "failure_codes": [] if folded_preview["pass"] else list(folded_preview["failure_codes"]),
    })
    require(len(slots) == 6 and tuple(slots[5]["selected_q_rad"]) not in {tuple(row["selected_q_rad"]) for row in slots[:5]}, "six-slot/folded uniqueness failed")
    return {
        "algorithm": "FIVE_CONTRACT_SLOTS_WITH_SAFETY_SUBSTITUTION_THEN_DISTINCT_LOWEST_V15_18A_TAU_NORM",
        "source_report_relative_path": V15_18A_JSON_REL, "source_report_sha256": V15_18A_JSON_SHA256,
        "requested_slot_count": 6, "selected_slot_count": 6,
        "unique_selected_q_count": len({tuple(row["selected_q_rad"]) for row in slots}),
        "max_extension_and_frozen_481_are_v15_18a_semantic_aliases": True,
        "folded_low_torque_selection": {
            "eligible_pool": "V15_18A_EXACT_21_ACCEPTED_POSES_PASSING_0P15S_SAFETY_PREFLIGHT_AND_NOT_DUPLICATING_FIRST_FIVE_SELECTED_Q",
            "ranking": "V15_18A_TAU_HOLD_L2_NORM_ASC_THEN_SOURCE_GLOBAL_INDEX_ASC_THEN_ID",
            "selected_pose_id": folded["id"], "selected_v15_18a_tau_norm_nm": folded["gravity_torque_norm_nm"],
        },
        "slots": slots, "pass": all(bool(row["pass"]) for row in slots),
    }


def analyze_energy(run: Mapping[str, Any]) -> dict[str, Any]:
    mujoco_rows = [vector(row, 3, "mujoco_energy") for row in run["mujoco_energy_j"]]
    mujoco_total = [row[2] for row in mujoco_rows]
    potential = [finite(value, "potential") for value in run["independent_potential_j"]]
    kinetic = [finite(value, "kinetic") for value in run["independent_kinetic_j"]]
    total = [finite(value, "total") for value in run["independent_total_j"]]
    require(len(mujoco_total) == len(total) == len(potential) == len(kinetic), "energy sample shape differs")
    denominator = max(abs(potential[-1] - potential[0]), 1.0e-3)
    mujoco_error = max_abs(value - mujoco_total[0] for value in mujoco_total)
    independent_error = max_abs(value - total[0] for value in total)
    aligned = max_abs((mujoco_total[index] - mujoco_total[0]) - (total[index] - total[0]) for index in range(len(total)))
    normalized_limit = ENERGY_DRIFT_HALF_MAX if str(run["label"]) == "B" else ENERGY_DRIFT_PRODUCTION_MAX
    allowed = max(normalized_limit * denominator, ENERGY_FLOOR_J)
    max_step = max((abs(total[index] - total[index - 1]) for index in range(1, len(total))), default=0.0)
    jump_limit = max(0.05 * denominator, ENERGY_FLOOR_J)
    significant = norm(vector(run["initial_qacc_rad_s2"], 6, "energy.initial_qacc")) > EARLY_QACC_MIN
    trend = (not significant) or (potential[-1] < potential[0] and kinetic[-1] > kinetic[0])
    values_finite = all(math.isfinite(value) for value in mujoco_total + total + potential + kinetic)
    passed = values_finite and mujoco_error < allowed and independent_error < allowed and aligned <= 1.0e-8 and trend and max_step <= jump_limit
    return {
        "mujoco": {"initial_total_j": mujoco_total[0], "final_total_j": mujoco_total[-1], "max_abs_drift_j": mujoco_error, "normalized_drift": mujoco_error / denominator, "pass": mujoco_error < allowed},
        "independent": {
            "initial_potential_j": potential[0], "final_potential_j": potential[-1], "potential_change_j": potential[-1] - potential[0],
            "initial_kinetic_j": kinetic[0], "final_kinetic_j": kinetic[-1], "kinetic_change_j": kinetic[-1] - kinetic[0],
            "initial_total_j": total[0], "final_total_j": total[-1], "max_abs_drift_j": independent_error,
            "normalized_drift": independent_error / denominator, "pass": independent_error < allowed,
        },
        "normalization_denominator_j": denominator,
        "normalized_limit_strict_less_than": normalized_limit,
        "absolute_limit_with_floor_j": allowed,
        "max_aligned_change_difference_j": aligned,
        "dual_source_alignment_pass": aligned <= 1.0e-8,
        "significantly_nonzero_acceleration": significant,
        "potential_decreases": potential[-1] < potential[0], "kinetic_increases": kinetic[-1] > kinetic[0],
        "energy_direction_pass": trend,
        "max_abs_single_step_total_energy_change_j": max_step,
        "single_step_energy_change_limit_j": jump_limit,
        "no_unexplained_energy_jump": max_step <= jump_limit,
        "all_values_finite": values_finite, "pass": passed,
    }


def analyze_run(raw: Mapping[str, Any]) -> dict[str, Any]:
    raw_required = RUN_CORE_KEYS - {"energy_summary", "continuity", "safety", "pass", "failure_codes"}
    require(raw_required <= set(raw), f"raw run is missing core fields: {sorted(raw_required-set(raw))}")
    run = {key: raw[key] for key in raw_required}
    energy = analyze_energy(raw)
    safety = dict(raw["raw_safety"])
    continuity = {
        "qpos_all_finite": all(math.isfinite(finite(value, "qpos")) for row in raw["qpos_rad"] for value in row),
        "qvel_all_finite": all(math.isfinite(finite(value, "qvel")) for row in raw["qvel_rad_s"] for value in row),
        "qacc_all_finite": all(math.isfinite(finite(value, "qacc")) for row in raw["qacc_rad_s2"] for value in row),
        "energy_all_finite": bool(energy["all_values_finite"]),
        "max_abs_qpos_step_rad": raw["max_abs_qpos_step_rad"],
        "max_abs_qvel_step_rad_s": raw["max_abs_qvel_step_rad_s"],
        "max_abs_qacc_step_rad_s2": raw["max_abs_qacc_step_rad_s2"],
        "qpos_step_limit_rad_strict_less_than": MAX_QPOS_STEP,
        "qvel_step_limit_rad_s_strict_less_than": MAX_QVEL_STEP,
        "qacc_step_limit_rad_s2_strict_less_than": 50.0,
        "qpos_step_pass": safety["qpos_step_pass"],
        "qvel_step_pass": safety["qvel_step_pass"],
        "qacc_step_pass": safety["qacc_step_continuity_pass"],
        "energy_step_pass": bool(energy["no_unexplained_energy_jump"]),
    }
    continuity["pass"] = all(bool(value) for key, value in continuity.items() if key.endswith("_finite") or key.endswith("_pass"))
    failures = []
    checks = (
        (not raw["all_values_finite"], "FAIL_NONFINITE_DYNAMICS"),
        (not safety["no_contacts"], "FAIL_CONTACT_DURING_PASSIVE_RUN"),
        (not safety["no_active_joint_limit_constraints"], "FAIL_ACTIVE_JOINT_LIMIT_CONSTRAINT"),
        (not safety["joint_limit_margin_pass"], "FAIL_RUNTIME_JOINT_LIMIT_MARGIN"),
        (not safety["qvel_pass"], "FAIL_QVEL_STABILITY_LIMIT"),
        (not safety["qacc_pass"], "FAIL_QACC_STABILITY_LIMIT"),
        (not safety["qpos_step_pass"], "FAIL_QPOS_SINGLE_STEP_JUMP"),
        (not safety["qvel_step_pass"], "FAIL_QVEL_SINGLE_STEP_JUMP"),
        (not safety["qacc_step_continuity_pass"], "FAIL_QACC_SINGLE_STEP_DISCONTINUITY"),
        (not continuity["pass"], "FAIL_TRAJECTORY_CONTINUITY"),
        (not safety["actuation_passive_and_applied_zero"], "FAIL_ACTUATION_PASSIVE_OR_APPLIED_FORCE_ISOLATION"),
        (not energy["pass"], "FAIL_ENERGY_AUDIT"),
    )
    failures.extend(code for failed, code in checks if failed)
    run.update({"energy_summary": energy, "continuity": continuity, "safety": safety, "pass": safety["pass"] and energy["pass"], "failure_codes": failures})
    require(set(run) == RUN_CORE_KEYS, f"independent run projection schema differs: {sorted(set(run)^RUN_CORE_KEYS)}")
    return run


def finite_ratio(numerator: float, denominator: float) -> float:
    if denominator == 0.0:
        return 0.0 if numerator == 0.0 else math.inf
    return numerator / denominator


def early_motion_for_run(run: Mapping[str, Any], qacc_initial: Sequence[float]) -> dict[str, Any]:
    times = [finite(value, "probe.time") for value in run["time_s"]]
    probe_index = min(range(len(times)), key=lambda index: abs(times[index] - PROBE_S))
    require(abs(times[probe_index] - PROBE_S) <= 1.0e-15, "probe grid point absent")
    q0 = vector(run["qpos_rad"][0], 6, "probe.q0")
    qp = vector(run["qpos_rad"][probe_index], 6, "probe.q")
    actual = vector(qacc_initial, 6, "probe.qacc")
    joint_rows = []
    for index, joint in enumerate(JOINTS):
        predicted = 0.5 * abs(actual[index]) * PROBE_S * PROBE_S
        valid = abs(actual[index]) > EARLY_QACC_MIN and predicted > EARLY_PREDICTED_DISPLACEMENT_MIN
        displacement = qp[index] - q0[index]
        sign_match = (not valid) or (displacement != 0.0 and math.copysign(1.0, displacement) == math.copysign(1.0, actual[index]))
        joint_rows.append({
            "joint": joint, "initial_qacc_rad_s2": actual[index],
            "predicted_displacement_magnitude_rad": predicted,
            "actual_displacement_at_probe_rad": displacement,
            "valid_for_direction_check": valid, "sign_match": sign_match, "pass": sign_match,
        })
    return {
        "probe_time_s": PROBE_S, "probe_grid_index": probe_index, "per_joint": joint_rows,
        "j2_j3_j4_summary": {
            row["joint"]: {"initial_qacc_rad_s2": row["initial_qacc_rad_s2"], "displacement_rad": row["actual_displacement_at_probe_rad"], "valid": row["valid_for_direction_check"], "sign_match": row["sign_match"]}
            for row in joint_rows if row["joint"] in {"J2", "J3", "J4"}
        },
        "pass": all(bool(row["pass"]) for row in joint_rows),
    }


def aligned_trajectory_difference(coarse: Mapping[str, Any], fine: Mapping[str, Any]) -> dict[str, Any]:
    coarse_times = [finite(value, "coarse.time") for value in coarse["time_s"]]
    fine_times = [finite(value, "fine.time") for value in fine["time_s"]]
    coarse_dt, fine_dt = finite(coarse["dt_s"], "coarse.dt"), finite(fine["dt_s"], "fine.dt")
    stride_float = coarse_dt / fine_dt
    stride = int(round(stride_float))
    require(stride >= 1 and abs(stride_float - stride) <= 1.0e-12, "trajectory grids are not integer-nested")
    indices = [index * stride for index in range(len(coarse_times))]
    require(indices[-1] < len(fine_times), "fine trajectory does not span coarse grid")
    time_error = max_abs(coarse_times[index] - fine_times[fine_index] for index, fine_index in enumerate(indices))
    require(time_error <= 1.0e-15, "nested trajectory time alignment changed")
    qdiff: list[float] = []
    vdiff: list[float] = []
    for coarse_index, fine_index in enumerate(indices):
        qdiff.extend(a - b for a, b in zip(vector(coarse["qpos_rad"][coarse_index], 6, "coarse.q"), vector(fine["qpos_rad"][fine_index], 6, "fine.q")))
        vdiff.extend(a - b for a, b in zip(vector(coarse["qvel_rad_s"][coarse_index], 6, "coarse.v"), vector(fine["qvel_rad_s"][fine_index], 6, "fine.v")))
    return {
        "method": "FULL_SHARED_NESTED_TIME_GRID_MAX_OVER_ALL_SAMPLES_AND_JOINTS",
        "coarse_label": coarse["label"], "fine_label": fine["label"],
        "coarse_dt_s": coarse_dt, "fine_dt_s": fine_dt, "fine_grid_stride": stride,
        "shared_grid_sample_count": len(coarse_times), "max_abs_shared_time_error_s": time_error,
        "qpos_max_abs_difference_rad": max_abs(qdiff),
        "qpos_rms_difference_rad": math.sqrt(sum(value * value for value in qdiff) / len(qdiff)),
        "qvel_max_abs_difference_rad_s": max_abs(vdiff),
        "qvel_rms_difference_rad_s": math.sqrt(sum(value * value for value in vdiff) / len(vdiff)),
        "all_values_finite": all(math.isfinite(value) for value in qdiff + vdiff),
    }


def implicitfast_energy_evidence(runs: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    labels = ("A", "B", "I4")
    sources: dict[str, Any] = {}
    for source in ("mujoco", "independent"):
        errors = {label: float(runs[label]["energy_summary"][source]["normalized_drift"]) for label in labels}
        r1, r2 = finite_ratio(errors["B"], errors["A"]), finite_ratio(errors["I4"], errors["B"])
        monotonic = errors["I4"] < errors["B"] < errors["A"]
        ratios_pass = 0.35 <= r1 <= 0.70 and 0.35 <= r2 <= 0.70
        sources[source] = {
            "normalized_drift_by_run": errors, "r1_E_1ms_over_E_2ms": r1,
            "r2_E_0p5ms_over_E_1ms": r2,
            "monotonic_E_0p5ms_lt_E_1ms_lt_E_2ms": monotonic,
            "ratio_interval_inclusive": [0.35, 0.70], "ratios_pass": ratios_pass,
            "pass": monotonic and ratios_pass,
        }
    conservative = {label: max(float(runs[label]["energy_summary"][source]["normalized_drift"]) for source in ("mujoco", "independent")) for label in labels}
    passed = all(bool(row["pass"]) for row in sources.values())
    return {
        "method": "DUAL_ENERGY_NORMALIZED_MAX_DRIFT_THREE_LEVEL_IMPLICITFAST",
        "profiles": {"A": "IMPLICITFAST_2MS", "B": "IMPLICITFAST_1MS", "I4": "IMPLICITFAST_0P5MS"},
        "sources": sources, "conservative_max_normalized_drift_by_run": conservative,
        "conservative_r1_E_1ms_over_E_2ms": finite_ratio(conservative["B"], conservative["A"]),
        "conservative_r2_E_0p5ms_over_E_1ms": finite_ratio(conservative["I4"], conservative["B"]),
        "classification": "FIRST_ORDER_LIKE_TIMESTEP_CONVERGENCE" if passed else "NO_CONFIRMED_TIMESTEP_ORDER",
        "pass": passed,
    }


def implicitfast_trajectory_evidence(runs: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    first = aligned_trajectory_difference(runs["A"], runs["B"])
    second = aligned_trajectory_difference(runs["B"], runs["I4"])
    q_ratio = finite_ratio(float(second["qpos_max_abs_difference_rad"]), float(first["qpos_max_abs_difference_rad"]))
    v_ratio = finite_ratio(float(second["qvel_max_abs_difference_rad_s"]), float(first["qvel_max_abs_difference_rad_s"]))
    values_finite = bool(first["all_values_finite"] and second["all_values_finite"] and math.isfinite(q_ratio) and math.isfinite(v_ratio))
    return {
        "method": "FULL_ALIGNED_TRAJECTORY_MAX_MULTI_LEVEL_CONVERGENCE",
        "difference_2ms_vs_1ms": first, "difference_1ms_vs_0p5ms": second,
        "qpos_difference_ratio_D_1ms_0p5ms_over_D_2ms_1ms": q_ratio,
        "qvel_difference_ratio_D_1ms_0p5ms_over_D_2ms_1ms": v_ratio,
        "ratio_limit_strict_less_than": 0.70, "qpos_pass": q_ratio < 0.70,
        "qvel_pass": v_ratio < 0.70, "all_values_finite": values_finite,
        "pass": values_finite and q_ratio < 0.70 and v_ratio < 0.70,
    }


def rk4_energy_evidence(runs: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    profiles: dict[str, Any] = {}
    for label, limit in (("RK4_2MS", 0.001), ("RK4_1MS", 0.0005)):
        summary = runs[label]["energy_summary"]
        sources = {
            source: {
                "max_abs_drift_j": float(summary[source]["max_abs_drift_j"]),
                "normalized_drift": float(summary[source]["normalized_drift"]),
                "normalized_limit_strict_less_than": limit,
                "pass": float(summary[source]["normalized_drift"]) < limit,
            } for source in ("mujoco", "independent")
        }
        profiles[label] = {
            "sources": sources,
            "normalization_denominator_j": float(summary["normalization_denominator_j"]),
            "normalized_floor_equivalent_of_1e_8_j": ENERGY_FLOOR_J / float(summary["normalization_denominator_j"]),
            "dual_source_alignment_pass": bool(summary["dual_source_alignment_pass"]),
            "all_values_finite": bool(summary["all_values_finite"]),
            "pass": all(bool(row["pass"]) for row in sources.values()) and bool(summary["dual_source_alignment_pass"]) and bool(summary["all_values_finite"]),
        }
    normalized_floor = float(profiles["RK4_2MS"]["normalized_floor_equivalent_of_1e_8_j"])
    half_normalized = {
        source: {
            "rk4_2ms_normalized_drift": profiles["RK4_2MS"]["sources"][source]["normalized_drift"],
            "rk4_1ms_normalized_drift": profiles["RK4_1MS"]["sources"][source]["normalized_drift"],
            "normalized_floor": normalized_floor, "normalized_floor_j": ENERGY_FLOOR_J,
            "normalized_floor_denominator_j": profiles["RK4_2MS"]["normalization_denominator_j"],
            "normalized_floor_denominator_profile": "RK4_2MS_RHS_NOMINAL_REFERENCE",
            "normalized_floor_by_run": {label: profiles[label]["normalized_floor_equivalent_of_1e_8_j"] for label in ("RK4_2MS", "RK4_1MS")},
            "semantics": "RK4_1MS_NORMALIZED_DRIFT_LE_RK4_2MS_NORMALIZED_DRIFT_PLUS_1E_8_J_DIVIDED_BY_RK4_2MS_NORMALIZATION_DENOMINATOR",
            "pass": profiles["RK4_1MS"]["sources"][source]["normalized_drift"] <= profiles["RK4_2MS"]["sources"][source]["normalized_drift"] + normalized_floor,
        } for source in ("mujoco", "independent")
    }
    half_absolute = {
        source: {
            "rk4_2ms_max_abs_drift_j": profiles["RK4_2MS"]["sources"][source]["max_abs_drift_j"],
            "rk4_1ms_max_abs_drift_j": profiles["RK4_1MS"]["sources"][source]["max_abs_drift_j"],
            "additive_absolute_comparison_floor_j": ENERGY_FLOOR_J,
            "gate_authority": False, "semantics": "ABSOLUTE_J_COMPARISON_RETAINED_AS_DIAGNOSTIC_ONLY",
            "pass": profiles["RK4_1MS"]["sources"][source]["max_abs_drift_j"] <= profiles["RK4_2MS"]["sources"][source]["max_abs_drift_j"] + ENERGY_FLOOR_J,
        } for source in ("mujoco", "independent")
    }
    return {
        "method": "DUAL_ENERGY_RK4_RUNTIME_ONLY_REFERENCE",
        "normalization": "SAME_PER_RUN_ABS_POTENTIAL_CHANGE_DENOMINATOR_WITH_1E_3_J_DENOMINATOR_FLOOR_AS_LEGACY_AUDIT",
        "profiles": profiles, "half_step_normalized_drift_not_worse": half_normalized,
        "half_step_absolute_drift_not_worse": half_absolute,
        "pass": all(bool(row["pass"]) for row in profiles.values()) and all(bool(row["pass"]) for row in half_normalized.values()),
    }


def rk4_convergence_evidence(runs: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    qdiff = max_final_difference(runs["RK4_2MS"], runs["RK4_1MS"], "final_qpos_rad")
    vdiff = max_final_difference(runs["RK4_2MS"], runs["RK4_1MS"], "final_qvel_rad_s")
    return {
        "method": "RK4_2MS_VS_RK4_1MS_FINAL_STATE",
        "final_qpos_max_abs_difference_rad": qdiff, "final_qpos_limit_rad_strict_less_than": 5.0e-4,
        "final_qvel_max_abs_difference_rad_s": vdiff, "final_qvel_limit_rad_s_strict_less_than": 5.0e-3,
        "qpos_pass": qdiff < 5.0e-4, "qvel_pass": vdiff < 5.0e-3,
        "pass": qdiff < 5.0e-4 and vdiff < 5.0e-3,
    }


def high_precision_reference_evidence(runs: Mapping[str, Mapping[str, Any]], qacc_initial: Sequence[float]) -> dict[str, Any]:
    early = {label: early_motion_for_run(runs[label], qacc_initial) for label in ("I4", "RK4_1MS")}
    valid = sum(1 for result in early.values() for row in result["per_joint"] if row["valid_for_direction_check"])
    matched = sum(1 for result in early.values() for row in result["per_joint"] if row["valid_for_direction_check"] and row["sign_match"])
    qdiff = max_final_difference(runs["I4"], runs["RK4_1MS"], "final_qpos_rad")
    return {
        "method": "IMPLICITFAST_0P5MS_VS_RK4_1MS_DIAGNOSTIC", "reference_profile": "RK4_1MS",
        "early_motion_direction": early, "valid_early_direction_count": valid,
        "matching_early_direction_count": matched, "all_valid_early_directions_match": matched == valid,
        "final_qpos_max_abs_difference_rad": qdiff,
        "final_qpos_anomaly_limit_rad_strict_less_than": 0.02,
        "pass": matched == valid and all(bool(row["pass"]) for row in early.values()) and qdiff < 0.02,
    }


def analyze_pose(raw: Mapping[str, Any], selection: Mapping[str, Any]) -> dict[str, Any]:
    initial = raw.get("initial_state")
    require(isinstance(initial, dict), "official initial state missing")
    actual = vector(initial.get("qacc_rad_s2"), 6, "initial.qacc")
    expected = vector(initial.get("expected_qacc_rad_s2"), 6, "initial.expected_qacc")
    residual = finite(initial.get("residual_norm"), "initial.residual")
    relative = finite(initial.get("relative_qacc_error"), "initial.relative")
    identity_pass = relative < QACC_REL_TOL and residual < RESIDUAL_TOL
    runs_raw = raw.get("raw_runs")
    require(isinstance(runs_raw, dict) and set(runs_raw) == {"A", "B", "C"}, "official A/B/C run set differs")
    runs = {label: analyze_run(run) for label, run in runs_raw.items()}

    def early_for(run: Mapping[str, Any]) -> dict[str, Any]:
        times = [finite(value, "probe.time") for value in run["time_s"]]
        probe_index = min(range(len(times)), key=lambda index: abs(times[index] - PROBE_S))
        require(abs(times[probe_index] - PROBE_S) <= 1.0e-15, "probe grid point absent")
        q0 = vector(run["qpos_rad"][0], 6, "probe.q0")
        qp = vector(run["qpos_rad"][probe_index], 6, "probe.q")
        joint_rows = []
        for index, joint in enumerate(JOINTS):
            predicted = 0.5 * abs(actual[index]) * PROBE_S * PROBE_S
            valid = abs(actual[index]) > EARLY_QACC_MIN and predicted > EARLY_PREDICTED_DISPLACEMENT_MIN
            displacement = qp[index] - q0[index]
            sign_match = (not valid) or (displacement != 0.0 and math.copysign(1.0, displacement) == math.copysign(1.0, actual[index]))
            joint_rows.append({
                "joint": joint, "initial_qacc_rad_s2": actual[index],
                "predicted_displacement_magnitude_rad": predicted,
                "actual_displacement_at_probe_rad": displacement,
                "valid_for_direction_check": valid, "sign_match": sign_match, "pass": sign_match,
            })
        return {
            "probe_time_s": PROBE_S, "probe_grid_index": probe_index, "per_joint": joint_rows,
            "j2_j3_j4_summary": {
                row["joint"]: {"initial_qacc_rad_s2": row["initial_qacc_rad_s2"], "displacement_rad": row["actual_displacement_at_probe_rad"], "valid": row["valid_for_direction_check"], "sign_match": row["sign_match"]}
                for row in joint_rows if row["joint"] in {"J2", "J3", "J4"}
            },
            "pass": all(bool(row["pass"]) for row in joint_rows),
        }

    early_runs = {label: early_for(runs[label]) for label in ("A", "B")}
    early = {"runs": early_runs, "pass": early_runs["A"]["pass"] and early_runs["B"]["pass"]}
    a_q, a_v = runs["A"]["qpos_rad"], runs["A"]["qvel_rad_s"]
    b_q_full, b_v_full = runs["B"]["qpos_rad"], runs["B"]["qvel_rad_s"]
    b_q, b_v = b_q_full[::2], b_v_full[::2]
    require(len(a_q) == len(b_q) and len(a_v) == len(b_v), "half-dt resampling grid differs")
    q_diff = [[finite(a, "a.q") - finite(b, "b.q") for a, b in zip(left, right)] for left, right in zip(a_q, b_q)]
    v_diff = [[finite(a, "a.v") - finite(b, "b.v") for a, b in zip(left, right)] for left, right in zip(a_v, b_v)]
    q0a, q0b = vector(a_q[0], 6, "A.q0"), vector(b_q_full[0], 6, "B.q0")
    max_displacement = max(
        max_abs(finite(value, "q") - q0a[index] for row in a_q for index, value in enumerate(row)),
        max_abs(finite(value, "q") - q0b[index] for row in b_q_full for index, value in enumerate(row)),
    )
    max_speed = max(max_abs(value for row in a_v for value in row), max_abs(value for row in b_v_full for value in row))
    q_limit, v_limit = max(1.0e-3, 0.005 * max_displacement), max(1.0e-2, 0.01 * max_speed)
    q_final = max_abs(q_diff[-1])
    v_final = max_abs(v_diff[-1])
    convergence = {
        "method": "DT_OVER_2_LINEARLY_INTERPOLATED_TO_NOMINAL_TIME_GRID",
        "max_passive_displacement_rad": max_displacement, "max_abs_speed_rad_s": max_speed,
        "final_qpos_max_abs_difference_rad": q_final, "final_qpos_limit_rad": q_limit,
        "final_qvel_max_abs_difference_rad_s": v_final, "final_qvel_limit_rad_s": v_limit,
        "trajectory_qpos_max_abs_difference_rad": max_abs(value for row in q_diff for value in row),
        "trajectory_qpos_rms_difference_rad": math.sqrt(sum(value * value for row in q_diff for value in row) / (len(q_diff) * 6)),
        "trajectory_qvel_max_abs_difference_rad_s": max_abs(value for row in v_diff for value in row),
        "trajectory_qvel_rms_difference_rad_s": math.sqrt(sum(value * value for row in v_diff for value in row) / (len(v_diff) * 6)),
        "qpos_pass": q_final < q_limit, "qvel_pass": v_final < v_limit,
        "pass": q_final < q_limit and v_final < v_limit,
    }
    c_q, c_v = runs["C"]["qpos_rad"], runs["C"]["qvel_rad_s"]
    q_det = max_abs(finite(a, "A.q") - finite(c, "C.q") for ar, cr in zip(a_q, c_q) for a, c in zip(ar, cr))
    v_det = max_abs(finite(a, "A.v") - finite(c, "C.v") for ar, cr in zip(a_v, c_v) for a, c in zip(ar, cr))
    e_det = max_abs(finite(a, "A.e") - finite(c, "C.e") for ar, cr in zip(runs["A"]["mujoco_energy_j"], runs["C"]["mujoco_energy_j"]) for a, c in zip(ar, cr))
    bitwise = a_q == c_q and a_v == c_v and runs["A"]["mujoco_energy_j"] == runs["C"]["mujoco_energy_j"]
    determinism = {
        "qpos_max_abs_difference_rad": q_det,
        "qvel_max_abs_difference_rad_s": v_det,
        "mujoco_energy_max_abs_difference_j": e_det,
        "classification": "BITWISE_DETERMINISTIC" if bitwise else "NUMERICALLY_DETERMINISTIC",
        "bitwise_equal": bitwise,
        "pass": q_det < DETERMINISM_TOL and v_det < DETERMINISM_TOL and e_det < DETERMINISM_TOL,
    }
    energy = {
        "A": runs["A"]["energy_summary"],
        "B": runs["B"]["energy_summary"],
        "C": runs["C"]["energy_summary"],
    }
    energy["half_step_not_noticeably_worse"] = (
        energy["B"]["mujoco"]["max_abs_drift_j"] <= energy["A"]["mujoco"]["max_abs_drift_j"] + ENERGY_FLOOR_J
        and energy["B"]["independent"]["max_abs_drift_j"] <= energy["A"]["independent"]["max_abs_drift_j"] + ENERGY_FLOOR_J
    )
    energy["pass"] = energy["A"]["pass"] and energy["B"]["pass"] and energy["C"]["pass"] and energy["half_step_not_noticeably_worse"]
    safety = {
        "all_three_runs_pass": all(bool(run["safety"]["pass"]) for run in runs.values()),
        "per_run": {label: runs[label]["safety"] for label in ("A", "B", "C")},
    }
    safety["pass"] = safety["all_three_runs_pass"]
    failures: list[str] = []
    for condition, code in (
        (identity_pass, "FAIL_INITIAL_ACCELERATION_IDENTITY"),
        (early["pass"], "FAIL_EARLY_MOTION_DIRECTION"),
        (energy["pass"], "FAIL_DUAL_ENERGY_AUDIT"),
        (convergence["pass"], "FAIL_TIMESTEP_CONVERGENCE"),
        (determinism["pass"], "FAIL_DETERMINISM"),
        (safety["pass"], "FAIL_RUNTIME_SAFETY"),
    ):
        if not condition:
            failures.append(code)
    failures.extend(str(code) for run in runs.values() for code in run["failure_codes"])
    failures = sorted(set(failures))
    initial_projection = {
        "qpos_rad": vector(initial.get("qpos_rad"), 6, "initial.qpos"),
        "qvel_rad_s": vector(initial.get("qvel_rad_s"), 6, "initial.qvel"),
        "qfrc_bias_nm": vector(initial.get("qfrc_bias_nm"), 6, "initial.bias"),
        "qacc_rad_s2": actual, "expected_qacc_rad_s2": expected,
        "mass_matrix": initial.get("mass_matrix"),
        "residual_vector": vector(initial.get("residual_vector"), 6, "initial.residual_vector"),
        "residual_norm": residual, "relative_qacc_error": relative,
        "solve_available": True, "identity_pass": identity_pass,
        "mujoco_potential_j": finite(initial.get("mujoco_potential_j"), "initial.mjpotential"),
        "mujoco_kinetic_j": finite(initial.get("mujoco_kinetic_j"), "initial.mjkinetic"),
        "independent_authority_potential_j": finite(initial.get("independent_authority_potential_j"), "initial.authority_potential"),
        "independent_all_positive_potential_j": finite(initial.get("independent_all_positive_potential_j"), "initial.all_potential"),
        "body_com_world_m": initial.get("body_com_world_m"), "contacts": initial.get("contacts"),
        "ncon": int(initial.get("ncon", -1)), "nefc": int(initial.get("nefc", -1)),
        "active_joint_limit_constraints": int(initial.get("active_joint_limit_constraint_count", -1)),
        "joint_limit_margin_rad": initial.get("joint_limit_margin_rad"),
        "minimum_joint_limit_margin_rad": finite(initial.get("minimum_joint_limit_margin_rad"), "initial.margin"),
        "qfrc_actuator_nm": initial.get("qfrc_actuator_nm"), "qfrc_passive_nm": initial.get("qfrc_passive_nm"),
        "ctrl": initial.get("ctrl"), "qfrc_applied_nm": initial.get("qfrc_applied_nm"), "xfrc_applied": initial.get("xfrc_applied"),
    }
    return {
        "slot_index": int(raw["slot_index"]),
        "category": str(raw["category"]),
        "selected_pose_id": str(raw["selected_pose_id"]),
        "selected_q_rad": vector(raw["selected_q_rad"], 6, "selected.q"),
        "initial_state": initial_projection,
        "runs": runs,
        "early_motion_direction": early,
        "energy": energy,
        "convergence": convergence,
        "determinism": determinism,
        "safety": safety,
        "pass": identity_pass and early["pass"] and energy["pass"] and convergence["pass"] and determinism["pass"] and safety["pass"],
        "failure_codes": failures,
    }


def analyze_pose_b2(raw: Mapping[str, Any], selection: Mapping[str, Any]) -> dict[str, Any]:
    raw_runs = raw.get("raw_runs")
    labels = tuple(label for label, _, _ in ATTRIBUTION_CONFIGURATIONS)
    require(isinstance(raw_runs, dict) and tuple(raw_runs) == labels, "B2 exact six-run order differs")
    legacy_raw = dict(raw)
    legacy_raw["raw_runs"] = {label: raw_runs[label] for label in ("A", "B", "C")}
    legacy = analyze_pose(legacy_raw, selection)
    runs = dict(legacy["runs"])
    for label in ("I4", "RK4_2MS", "RK4_1MS"):
        runs[label] = analyze_run(raw_runs[label])
    implicit_energy = implicitfast_energy_evidence(runs)
    implicit_trajectory = implicitfast_trajectory_evidence(runs)
    rk4_energy = rk4_energy_evidence(runs)
    rk4_convergence = rk4_convergence_evidence(runs)
    reference = high_precision_reference_evidence(runs, legacy["initial_state"]["qacc_rad_s2"])
    attribution_pass = bool(implicit_energy["pass"] and implicit_trajectory["pass"] and rk4_energy["pass"] and rk4_convergence["pass"] and reference["pass"])
    all_safety = all(bool(runs[label]["safety"]["pass"]) for label in labels)
    all_continuity = all(bool(runs[label]["continuity"]["pass"]) for label in labels)
    legacy_codes: list[str] = []
    if not legacy["energy"]["pass"]:
        legacy_codes.append("LEGACY_FAIL_DUAL_ENERGY_AUDIT")
    if not legacy["convergence"]["pass"]:
        legacy_codes.append("LEGACY_FAIL_ABSOLUTE_FINAL_STATE_TIMESTEP_CONVERGENCE")
    failures: list[str] = []
    checks = (
        (legacy["initial_state"]["identity_pass"], "FAIL_INITIAL_ACCELERATION_IDENTITY"),
        (legacy["early_motion_direction"]["pass"], "FAIL_EARLY_MOTION_DIRECTION"),
        (implicit_energy["pass"], "FAIL_IMPLICITFAST_ENERGY_CONVERGENCE"),
        (implicit_trajectory["pass"], "FAIL_IMPLICITFAST_TRAJECTORY_CONVERGENCE"),
        (rk4_energy["pass"], "FAIL_RK4_ENERGY_REFERENCE"),
        (rk4_convergence["pass"], "FAIL_RK4_TIMESTEP_CONVERGENCE"),
        (reference["pass"], "FAIL_HIGH_PRECISION_REFERENCE_DIAGNOSTIC"),
        (legacy["determinism"]["pass"], "FAIL_DETERMINISM"),
        (all_safety, "FAIL_RUNTIME_SAFETY"),
        (all_continuity, "FAIL_TRAJECTORY_CONTINUITY"),
    )
    failures.extend(code for passed, code in checks if not passed)
    failures.extend(str(code) for label in labels for code in runs[label]["failure_codes"] if str(code) != "FAIL_ENERGY_AUDIT")
    failures = sorted(set(failures))
    return {
        "slot_index": int(raw["slot_index"]), "category": str(raw["category"]),
        "selected_pose_id": str(raw["selected_pose_id"]),
        "selected_q_rad": vector(raw["selected_q_rad"], 6, "selected.q"),
        "initial_state": legacy["initial_state"], "runs": runs,
        "early_motion_direction": legacy["early_motion_direction"], "energy": legacy["energy"],
        "convergence": legacy["convergence"],
        "legacy_production_integrator_diagnostic": {
            "classification": "PRODUCTION_INTEGRATOR_DIAGNOSTIC",
            "energy": legacy["energy"], "absolute_final_state_convergence": legacy["convergence"],
            "absolute_final_state_gate_model_validation_status": "SUPERSEDED_FOR_MODEL_VALIDATION_BY_MULTI_LEVEL_CONVERGENCE",
            "status": "PASS" if legacy["energy"]["pass"] and legacy["convergence"]["pass"] else "FAIL",
            "failure_codes": legacy_codes,
            "pass": bool(legacy["energy"]["pass"] and legacy["convergence"]["pass"]),
        },
        "numerical_integrator_attribution": {
            "implicitfast_energy_convergence": implicit_energy,
            "implicitfast_trajectory_convergence": implicit_trajectory,
            "rk4_energy_reference": rk4_energy, "rk4_timestep_convergence": rk4_convergence,
            "high_precision_reference_diagnostic": reference, "pass": attribution_pass,
        },
        "determinism": legacy["determinism"],
        "safety": {
            "all_six_runs_pass": all_safety, "all_six_runs_continuity_pass": all_continuity,
            "per_run": {label: runs[label]["safety"] for label in labels},
            "pass": all_safety and all_continuity,
        },
        "legacy_diagnostic_failure_codes": legacy_codes,
        "pass": bool(legacy["initial_state"]["identity_pass"] and legacy["early_motion_direction"]["pass"] and attribution_pass and legacy["determinism"]["pass"] and all_safety and all_continuity),
        "failure_codes": failures,
    }


def derive_acceptance_gates(selection: Mapping[str, Any], poses: Sequence[Mapping[str, Any]], protected_ok: bool = True) -> dict[str, bool]:
    return {
        "exact_six_pose_slots": len(poses) == 6 and [int(row["slot_index"]) for row in poses] == list(range(1, 7)),
        "pose_selection_safe": selection.get("pass") is True,
        "initial_acceleration_identity_pass": len(poses) == 6 and all(row["initial_state"]["identity_pass"] for row in poses),
        "early_motion_direction_pass": len(poses) == 6 and all(row["early_motion_direction"]["pass"] for row in poses),
        "dual_energy_pass": len(poses) == 6 and all(row["energy"]["pass"] for row in poses),
        "timestep_convergence_pass": len(poses) == 6 and all(row["convergence"]["pass"] for row in poses),
        "determinism_pass": len(poses) == 6 and all(row["determinism"]["pass"] for row in poses),
        "actuation_and_applied_force_isolation_pass": len(poses) == 6 and all(row["runs"][label]["safety"]["actuation_passive_and_applied_zero"] for row in poses for label in ("A", "B", "C")),
        "no_contacts_pass": len(poses) == 6 and all(int(row["runs"][label]["contact_count_max"]) == 0 for row in poses for label in ("A", "B", "C")),
        "no_active_joint_limits_pass": len(poses) == 6 and all(int(row["runs"][label]["active_joint_limit_constraint_count_max"]) == 0 for row in poses for label in ("A", "B", "C")),
        "joint_limit_margin_pass": len(poses) == 6 and all(float(row["runs"][label]["minimum_joint_limit_margin_rad"]) > RUNTIME_MARGIN_RAD for row in poses for label in ("A", "B", "C")),
        "stability_pass": len(poses) == 6 and all(row["safety"]["pass"] for row in poses),
        "trajectory_continuity_pass": len(poses) == 6 and all(row["runs"][label]["continuity"]["pass"] for row in poses for label in ("A", "B", "C")),
        "protected_hashes_unchanged": protected_ok,
        "production_model_hash_unchanged": protected_ok,
        "bridge_controller_unchanged": protected_ok,
        "all_values_finite": len(poses) == 6 and all(row["runs"][label]["all_values_finite"] for row in poses for label in ("A", "B", "C")),
        "hard_unresolved_items_empty": True,
    }


def hard_unresolved(gates: Mapping[str, bool], poses: Sequence[Mapping[str, Any]]) -> list[str]:
    failures = ["FAIL_GATE_" + name.upper() for name, passed in gates.items() if not passed and name != "hard_unresolved_items_empty"]
    failures.extend(code for row in poses for code in row.get("failure_codes", []))
    return sorted(set(failures))


def aggregate_independent(selection: Mapping[str, Any], raw: Mapping[str, Any]) -> dict[str, Any]:
    rows = raw.get("rows")
    require(isinstance(rows, list) and len(rows) == 6, "official physics did not return six rows")
    slots = selection.get("slots")
    require(isinstance(slots, list) and len(slots) == 6, "selection slots malformed")
    poses: list[dict[str, Any]] = []
    for expected, row in zip(slots, rows):
        require(int(row.get("slot_index", -1)) == int(expected["slot_index"]), "official slot order differs")
        require(str(row.get("selected_pose_id")) == expected["selected_pose_id"], "official selected pose id differs")
        require(same_vector(vector(row.get("selected_q_rad"), 6, "official.q"), expected["selected_q_rad"]), "official selected q differs")
        poses.append(analyze_pose(row, expected))
    gates = derive_acceptance_gates(selection, poses)
    unresolved = hard_unresolved(gates, poses)
    gates["hard_unresolved_items_empty"] = not unresolved
    initial_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "relative_qacc_error": row["initial_state"]["relative_qacc_error"], "residual_norm": row["initial_state"]["residual_norm"], "pass": row["initial_state"]["identity_pass"]} for row in poses]
    early_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "j2_j3_j4": row["early_motion_direction"]["runs"]["A"]["j2_j3_j4_summary"], "pass": row["early_motion_direction"]["pass"]} for row in poses]
    energy_rows = [{
        "slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"],
        "A_mujoco_normalized_drift": row["energy"]["A"]["mujoco"]["normalized_drift"],
        "A_independent_normalized_drift": row["energy"]["A"]["independent"]["normalized_drift"],
        "B_mujoco_normalized_drift": row["energy"]["B"]["mujoco"]["normalized_drift"],
        "B_independent_normalized_drift": row["energy"]["B"]["independent"]["normalized_drift"],
        "half_step_not_noticeably_worse": row["energy"]["half_step_not_noticeably_worse"], "pass": row["energy"]["pass"],
    } for row in poses]
    convergence_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], **row["convergence"]} for row in poses]
    determinism_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], **row["determinism"]} for row in poses]
    contacts_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "A": row["runs"]["A"]["contact_count_max"], "B": row["runs"]["B"]["contact_count_max"], "C": row["runs"]["C"]["contact_count_max"], "pass": all(row["runs"][label]["contact_count_max"] == 0 for label in ("A", "B", "C"))} for row in poses]
    margin_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "initial_margin_rad": row["initial_state"]["minimum_joint_limit_margin_rad"], "minimum_runtime_margin_rad": min(row["runs"][label]["minimum_joint_limit_margin_rad"] for label in ("A", "B", "C")), "pass": all(row["runs"][label]["minimum_joint_limit_margin_rad"] > RUNTIME_MARGIN_RAD for label in ("A", "B", "C"))} for row in poses]
    stability_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "per_run": {label: {key: row["runs"][label][key] for key in ("max_abs_qvel_rad_s", "max_abs_qacc_rad_s2", "max_abs_qpos_step_rad", "max_abs_qvel_step_rad_s", "max_abs_qacc_step_rad_s2")} for label in ("A", "B", "C")}, "pass": row["safety"]["pass"]} for row in poses]
    continuity_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "per_run": {label: row["runs"][label]["continuity"] for label in ("A", "B", "C")}, "pass": all(row["runs"][label]["continuity"]["pass"] for label in ("A", "B", "C"))} for row in poses]
    aggregates = {
        "initial_acceleration_identity": {"rows": initial_rows, "pass": all(row["pass"] for row in initial_rows)},
        "early_motion_direction": {"rows": early_rows, "pass": all(row["pass"] for row in early_rows)},
        "energy_audit": {"definition": "E=U+K; independent U=-sum(m_i*g dot xipos_i), independent K=0.5*qvel^T*M(q)*qvel; MuJoCo energy[0]+energy[1]; potential zeros aligned by changes", "rows": energy_rows, "pass": all(row["pass"] for row in energy_rows)},
        "timestep_convergence": {"rows": convergence_rows, "pass": all(row["pass"] for row in convergence_rows)},
        "determinism": {"rows": determinism_rows, "pass": all(row["pass"] for row in determinism_rows)},
        "contacts": {"rows": contacts_rows, "pass": all(row["pass"] for row in contacts_rows)},
        "joint_limit_margin": {"rows": margin_rows, "no_active_joint_limit_constraints": gates["no_active_joint_limits_pass"], "pass": all(row["pass"] for row in margin_rows) and gates["no_active_joint_limits_pass"]},
        "stability": {"rows": stability_rows, "pass": all(row["pass"] for row in stability_rows)},
        "trajectory_continuity": {"definition": "q(t), qvel(t), qacc(t), and energy(t) must be finite and free of prohibited single-step discontinuities", "rows": continuity_rows, "pass": all(row["pass"] for row in continuity_rows)},
    }
    return {
        "mujoco_version": raw.get("mujoco_version"),
        "numpy_version": raw.get("numpy_version"),
        "pose_selection": selection,
        "per_pose_runs": poses,
        **aggregates,
        "acceptance_gates": gates,
        "hard_unresolved_items": unresolved,
        "pass": all(gates.values()) and not unresolved,
    }


def canonical_digest(value: Any) -> str:
    payload = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return sha256_bytes(payload)


def aggregate_b2(selection: Mapping[str, Any], raw: Mapping[str, Any], legacy_authority: Mapping[str, Any]) -> dict[str, Any]:
    rows = raw.get("rows")
    require(isinstance(rows, list) and len(rows) == 6, "B2 physics did not return exact six rows")
    locked = [
        {"slot_index": slot, "category": category, "requested_pose_id": requested, "selected_pose_id": selected, "selected_q_rad": q}
        for slot, category, requested, selected, q in LOCKED_POSES
    ]
    poses: list[dict[str, Any]] = []
    for expected, row in zip(locked, rows):
        for key in ("slot_index", "category", "selected_pose_id", "selected_q_rad"):
            require_close(row.get(key), expected[key], "B2 frozen pose." + key, absolute=0.0, relative=0.0)
        poses.append(analyze_pose_b2(row, expected))
    labels = tuple(label for label, _, _ in ATTRIBUTION_CONFIGURATIONS)
    initial_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "relative_qacc_error": row["initial_state"]["relative_qacc_error"], "residual_norm": row["initial_state"]["residual_norm"], "pass": row["initial_state"]["identity_pass"]} for row in poses]
    early_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "j2_j3_j4": row["early_motion_direction"]["runs"]["A"]["j2_j3_j4_summary"], "pass": row["early_motion_direction"]["pass"]} for row in poses]
    energy_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "A_mujoco_normalized_drift": row["energy"]["A"]["mujoco"]["normalized_drift"], "A_independent_normalized_drift": row["energy"]["A"]["independent"]["normalized_drift"], "B_mujoco_normalized_drift": row["energy"]["B"]["mujoco"]["normalized_drift"], "B_independent_normalized_drift": row["energy"]["B"]["independent"]["normalized_drift"], "half_step_not_noticeably_worse": row["energy"]["half_step_not_noticeably_worse"], "pass": row["energy"]["pass"]} for row in poses]
    convergence_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], **row["convergence"]} for row in poses]
    determinism_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], **row["determinism"]} for row in poses]
    implicit_energy_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], **row["numerical_integrator_attribution"]["implicitfast_energy_convergence"]} for row in poses]
    implicit_trajectory_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], **row["numerical_integrator_attribution"]["implicitfast_trajectory_convergence"]} for row in poses]
    rk4_energy_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], **row["numerical_integrator_attribution"]["rk4_energy_reference"]} for row in poses]
    rk4_convergence_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], **row["numerical_integrator_attribution"]["rk4_timestep_convergence"]} for row in poses]
    reference_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], **row["numerical_integrator_attribution"]["high_precision_reference_diagnostic"]} for row in poses]
    contacts_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "per_run": {label: row["runs"][label]["contact_count_max"] for label in labels}, "pass": all(row["runs"][label]["contact_count_max"] == 0 for label in labels)} for row in poses]
    margin_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "initial_margin_rad": row["initial_state"]["minimum_joint_limit_margin_rad"], "minimum_runtime_margin_rad": min(row["runs"][label]["minimum_joint_limit_margin_rad"] for label in labels), "per_run": {label: row["runs"][label]["minimum_joint_limit_margin_rad"] for label in labels}, "pass": all(row["runs"][label]["minimum_joint_limit_margin_rad"] > RUNTIME_MARGIN_RAD for label in labels)} for row in poses]
    stability_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "per_run": {label: {key: row["runs"][label][key] for key in ("max_abs_qvel_rad_s", "max_abs_qacc_rad_s2", "max_abs_qpos_step_rad", "max_abs_qvel_step_rad_s", "max_abs_qacc_step_rad_s2")} for label in labels}, "pass": all(row["runs"][label]["safety"]["pass"] for label in labels)} for row in poses]
    continuity_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "per_run": {label: row["runs"][label]["continuity"] for label in labels}, "pass": all(row["runs"][label]["continuity"]["pass"] for label in labels)} for row in poses]
    independent_legacy_projection = [{"slot_index": row["slot_index"], "A": row["runs"]["A"], "B": row["runs"]["B"], "C": row["runs"]["C"]} for row in poses]
    independent_legacy_digest = canonical_digest(independent_legacy_projection)
    legacy_energy_pass = all(row["pass"] for row in energy_rows)
    legacy_convergence_pass = all(row["pass"] for row in convergence_rows)
    implicit_max = {label: max(row["conservative_max_normalized_drift_by_run"][label] for row in implicit_energy_rows) for label in ("A", "B", "I4")}
    implicit_global = {"max_conservative_normalized_drift_by_run": implicit_max, "r1_E_1ms_over_E_2ms": finite_ratio(implicit_max["B"], implicit_max["A"]), "r2_E_0p5ms_over_E_1ms": finite_ratio(implicit_max["I4"], implicit_max["B"]), "all_per_pose_pass": all(row["pass"] for row in implicit_energy_rows)}
    implicit_global["pass"] = bool(implicit_global["all_per_pose_pass"] and implicit_max["I4"] < implicit_max["B"] < implicit_max["A"] and 0.35 <= implicit_global["r1_E_1ms_over_E_2ms"] <= 0.70 and 0.35 <= implicit_global["r2_E_0p5ms_over_E_1ms"] <= 0.70)
    trajectory_global = {"Dq_2ms_1ms_rad": max(row["difference_2ms_vs_1ms"]["qpos_max_abs_difference_rad"] for row in implicit_trajectory_rows), "Dq_1ms_0p5ms_rad": max(row["difference_1ms_vs_0p5ms"]["qpos_max_abs_difference_rad"] for row in implicit_trajectory_rows), "Dv_2ms_1ms_rad_s": max(row["difference_2ms_vs_1ms"]["qvel_max_abs_difference_rad_s"] for row in implicit_trajectory_rows), "Dv_1ms_0p5ms_rad_s": max(row["difference_1ms_vs_0p5ms"]["qvel_max_abs_difference_rad_s"] for row in implicit_trajectory_rows), "all_per_pose_pass": all(row["pass"] for row in implicit_trajectory_rows)}
    trajectory_global["qpos_ratio"] = finite_ratio(trajectory_global["Dq_1ms_0p5ms_rad"], trajectory_global["Dq_2ms_1ms_rad"])
    trajectory_global["qvel_ratio"] = finite_ratio(trajectory_global["Dv_1ms_0p5ms_rad_s"], trajectory_global["Dv_2ms_1ms_rad_s"])
    trajectory_global["pass"] = bool(trajectory_global["all_per_pose_pass"] and trajectory_global["qpos_ratio"] < 0.70 and trajectory_global["qvel_ratio"] < 0.70)
    rk4_energy_global = {"max_normalized_drift_by_run": {label: max(max(row["profiles"][label]["sources"][source]["normalized_drift"] for source in ("mujoco", "independent")) for row in rk4_energy_rows) for label in ("RK4_2MS", "RK4_1MS")}, "all_per_pose_pass": all(row["pass"] for row in rk4_energy_rows)}
    rk4_energy_global["pass"] = bool(rk4_energy_global["all_per_pose_pass"] and rk4_energy_global["max_normalized_drift_by_run"]["RK4_2MS"] < 0.001 and rk4_energy_global["max_normalized_drift_by_run"]["RK4_1MS"] < 0.0005)
    rk4_conv_global = {"max_final_qpos_difference_rad": max(row["final_qpos_max_abs_difference_rad"] for row in rk4_convergence_rows), "max_final_qvel_difference_rad_s": max(row["final_qvel_max_abs_difference_rad_s"] for row in rk4_convergence_rows), "all_per_pose_pass": all(row["pass"] for row in rk4_convergence_rows)}
    rk4_conv_global["pass"] = bool(rk4_conv_global["all_per_pose_pass"] and rk4_conv_global["max_final_qpos_difference_rad"] < 5.0e-4 and rk4_conv_global["max_final_qvel_difference_rad_s"] < 5.0e-3)
    reference_global = {"valid_early_direction_count": sum(row["valid_early_direction_count"] for row in reference_rows), "matching_early_direction_count": sum(row["matching_early_direction_count"] for row in reference_rows), "expected_valid_early_direction_count": 70, "max_final_qpos_difference_rad": max(row["final_qpos_max_abs_difference_rad"] for row in reference_rows), "all_per_pose_pass": all(row["pass"] for row in reference_rows)}
    reference_global["pass"] = bool(reference_global["all_per_pose_pass"] and reference_global["valid_early_direction_count"] == reference_global["matching_early_direction_count"] == 70 and reference_global["max_final_qpos_difference_rad"] < 0.02)
    no_contacts, no_limits = all(row["pass"] for row in contacts_rows), all(row["runs"][label]["active_joint_limit_constraint_count_max"] == 0 for row in poses for label in labels)
    all_finite = all(row["runs"][label]["all_values_finite"] for row in poses for label in labels)
    legacy_valid = sum(1 for row in poses for label in ("A", "B") for joint in row["early_motion_direction"]["runs"][label]["per_joint"] if joint["valid_for_direction_check"])
    legacy_match = sum(1 for row in poses for label in ("A", "B") for joint in row["early_motion_direction"]["runs"][label]["per_joint"] if joint["valid_for_direction_check"] and joint["sign_match"])
    root_map = {
        "v15_18a_static_gravity_pass": bool(selection["v15_18a_source_pass"]),
        "initial_qacc_identity_pass": all(row["pass"] for row in initial_rows),
        "early_motion_direction_pass": all(row["pass"] for row in early_rows) and legacy_valid == legacy_match == 70 and reference_global["pass"],
        "implicitfast_energy_convergence_pass": implicit_global["pass"],
        "implicitfast_trajectory_convergence_pass": trajectory_global["pass"],
        "rk4_energy_reference_pass": rk4_energy_global["pass"], "rk4_timestep_convergence_pass": rk4_conv_global["pass"],
        "no_contacts_all_runs_pass": no_contacts, "no_active_joint_limits_all_runs_pass": no_limits,
        "all_values_finite_all_runs_pass": all_finite, "determinism_pass": all(row["pass"] for row in determinism_rows),
    }
    confirmed = derive_root_cause_confirmed(root_map)
    gates = {
        "exact_six_pose_slots": selection["locked_slot_projection_sha256"] == LOCKED_POSE_PROJECTION_SHA256,
        "pose_selection_safe": selection["pass"] is True, "legacy_production_runs_preserved": legacy_authority["preserved"] is True,
        "v15_18a_continuous_time_physics_pass": selection["v15_18a_source_pass"] is True,
        "initial_acceleration_identity_pass": root_map["initial_qacc_identity_pass"], "early_motion_direction_pass": all(row["pass"] for row in early_rows) and legacy_valid == legacy_match == 70,
        "implicitfast_energy_convergence_pass": implicit_global["pass"], "implicitfast_trajectory_convergence_pass": trajectory_global["pass"],
        "rk4_energy_reference_pass": rk4_energy_global["pass"], "rk4_timestep_convergence_pass": rk4_conv_global["pass"],
        "high_precision_reference_diagnostic_pass": reference_global["pass"], "determinism_pass": root_map["determinism_pass"],
        "actuation_and_applied_force_isolation_pass": all(row["runs"][label]["safety"]["actuation_passive_and_applied_zero"] for row in poses for label in labels),
        "no_contacts_pass": no_contacts, "no_active_joint_limits_pass": no_limits,
        "joint_limit_margin_pass": all(row["pass"] for row in margin_rows), "stability_pass": all(row["pass"] for row in stability_rows),
        "trajectory_continuity_pass": all(row["pass"] for row in continuity_rows), "protected_hashes_unchanged": True,
        "production_model_hash_unchanged": True, "bridge_controller_unchanged": True, "all_values_finite": all_finite,
        "numerical_integrator_truncation_error_confirmed": confirmed,
    }
    hard = sorted(set(["FAIL_GATE_" + key.upper() for key, passed in gates.items() if not passed] + [code for row in poses for code in row["failure_codes"]]))
    gates["hard_unresolved_items_empty"] = not hard
    require(tuple(gates) == GATE_ORDER, "independent B2 acceptance gate order differs")
    numerical = {
        "implicitfast_energy_convergence": {"rows": implicit_energy_rows, "global": implicit_global, "pass": implicit_global["pass"]},
        "implicitfast_trajectory_convergence": {"rows": implicit_trajectory_rows, "global": trajectory_global, "pass": trajectory_global["pass"]},
        "rk4_energy_reference": {"rows": rk4_energy_rows, "global": rk4_energy_global, "pass": rk4_energy_global["pass"]},
        "rk4_timestep_convergence": {"rows": rk4_convergence_rows, "global": rk4_conv_global, "pass": rk4_conv_global["pass"]},
        "high_precision_reference_diagnostic": {"rows": reference_rows, "global": reference_global, "pass": reference_global["pass"]},
        "numerical_attribution_pass": (not (legacy_energy_pass and legacy_convergence_pass)) and implicit_global["pass"] and trajectory_global["pass"] and rk4_energy_global["pass"] and rk4_conv_global["pass"] and reference_global["pass"],
        "root_cause_classification": ROOT_CAUSE_CLASSIFICATION if confirmed else "UNRESOLVED",
    }
    numerical["pass"] = numerical["numerical_attribution_pass"]
    return {
        "mujoco_version": raw["mujoco_version"], "numpy_version": raw["numpy_version"], "per_pose_runs": poses,
        "initial_acceleration_identity": {"rows": initial_rows, "pass": all(row["pass"] for row in initial_rows)},
        "early_motion_direction": {"rows": early_rows, "pass": all(row["pass"] for row in early_rows)},
        "energy_audit": {"definition": "LEGACY A/B/C E=U+K dual-source evidence preserved byte-for-byte at the run-object projection", "classification": "PRODUCTION_INTEGRATOR_DIAGNOSTIC", "rows": energy_rows, "pass": legacy_energy_pass},
        "timestep_convergence": {"classification": "PRODUCTION_INTEGRATOR_DIAGNOSTIC", "model_validation_status": "SUPERSEDED_FOR_MODEL_VALIDATION_BY_MULTI_LEVEL_CONVERGENCE", "rows": convergence_rows, "pass": legacy_convergence_pass},
        "determinism": {"rows": determinism_rows, "pass": all(row["pass"] for row in determinism_rows)},
        "contacts": {"rows": contacts_rows, "pass": no_contacts}, "joint_limit_margin": {"rows": margin_rows, "no_active_joint_limit_constraints": no_limits, "pass": all(row["pass"] for row in margin_rows) and no_limits},
        "stability": {"rows": stability_rows, "pass": all(row["pass"] for row in stability_rows)},
        "trajectory_continuity": {"definition": "q(t), qvel(t), qacc(t), and energy(t) must be finite and free of prohibited single-step discontinuities in every run profile", "rows": continuity_rows, "pass": all(row["pass"] for row in continuity_rows)},
        "production_integrator_diagnostic": {"classification": "PRODUCTION_INTEGRATOR_DIAGNOSTIC", "legacy_runs_projection_definition": "PER_POSE_SLOT_INDEX_PLUS_COMPLETE_RUN_OBJECTS_A_B_C_CANONICAL_JSON", "legacy_runs_abc_sha256": legacy_authority["actual_sha256"], "legacy_runs_abc_expected_sha256": legacy_authority["expected_sha256"], "legacy_runs_preserved": legacy_authority["preserved"], "energy_pass": legacy_energy_pass, "absolute_final_state_convergence_pass": legacy_convergence_pass, "status": "PASS" if legacy_energy_pass and legacy_convergence_pass else "FAIL", "acceptance_role": "DIAGNOSTIC_ONLY_NOT_CONTINUOUS_TIME_MODEL_AUTHORITY"},
        "numerical_integrator_attribution": numerical, "root_cause_condition_map_exact_11": root_map,
        "acceptance_gates": gates, "hard_unresolved_items": hard,
        "independent_legacy_recompute_sha256_diagnostic": independent_legacy_digest,
        "pass": all(gates.values()) and not hard,
    }


def expected_thresholds(production_dt: float = 0.002) -> dict[str, Any]:
    return {
        "duration_s_exact": DURATION_S,
        "probe_time_s": PROBE_S,
        "initial_joint_limit_margin_rad_minimum": INITIAL_MARGIN_RAD,
        "runtime_joint_limit_margin_rad_strict_greater_than": RUNTIME_MARGIN_RAD,
        "max_abs_qfrc_actuator_nm_strict_less_than": ACTUATOR_TOL_NM,
        "max_abs_qfrc_passive_nm_strict_less_than": ACTUATOR_TOL_NM,
        "max_abs_applied_force_strict_less_than": ACTUATOR_TOL_NM,
        "initial_qacc_relative_error_strict_less_than": QACC_REL_TOL,
        "initial_dynamics_residual_norm_strict_less_than": RESIDUAL_TOL,
        "early_direction_min_abs_qacc_rad_s2": EARLY_QACC_MIN,
        "early_direction_min_predicted_displacement_rad": EARLY_PREDICTED_DISPLACEMENT_MIN,
        "nominal_normalized_energy_drift_strict_less_than": ENERGY_DRIFT_PRODUCTION_MAX,
        "half_step_normalized_energy_drift_strict_less_than": ENERGY_DRIFT_HALF_MAX,
        "energy_numerical_floor_j": ENERGY_FLOOR_J,
        "half_step_energy_not_worse_additive_j": ENERGY_FLOOR_J,
        "dual_energy_aligned_change_max_abs_j": 1.0e-8,
        "final_q_convergence_absolute_floor_rad": 1.0e-3,
        "final_q_convergence_relative_to_max_displacement": 0.005,
        "final_qvel_convergence_absolute_floor_rad_s": 1.0e-2,
        "final_qvel_convergence_relative_to_max_speed": 0.01,
        "determinism_qpos_max_abs_rad_strict_less_than": DETERMINISM_TOL,
        "determinism_qvel_max_abs_rad_s_strict_less_than": DETERMINISM_TOL,
        "determinism_energy_max_abs_j_strict_less_than": DETERMINISM_TOL,
        "max_abs_qvel_rad_s_strict_less_than": MAX_QVEL,
        "max_abs_qacc_rad_s2_strict_less_than": MAX_QACC,
        "max_abs_single_qpos_step_rad_strict_less_than": MAX_QPOS_STEP,
        "max_abs_single_qvel_step_rad_s_strict_less_than": MAX_QVEL_STEP,
        "max_abs_single_qacc_step_rad_s2_strict_less_than": 50.0,
        "unexplained_single_step_energy_fraction": 0.05,
        "selection_qspace_sweep_segment_count": 30,
        "selection_initial_qacc_prediction_scale": 1.1,
        "implicitfast_energy_ratio_minimum_inclusive": 0.35,
        "implicitfast_energy_ratio_maximum_inclusive": 0.70,
        "implicitfast_trajectory_ratio_strict_less_than": 0.70,
        "rk4_2ms_normalized_energy_drift_strict_less_than": 0.001,
        "rk4_1ms_normalized_energy_drift_strict_less_than": 0.0005,
        "rk4_final_q_difference_rad_strict_less_than": 5.0e-4,
        "rk4_final_qvel_difference_rad_s_strict_less_than": 5.0e-3,
        "rk4_half_step_energy_not_worse_additive_floor_j": 1.0e-8,
        "implicitfast_0p5ms_vs_rk4_1ms_final_q_anomaly_rad_strict_less_than": 0.02,
    }


def validate_audit_static_contract() -> dict[str, Any]:
    path = repo_path(AUDIT_REL)
    require(path.is_file(), "V15.18B audit builder is missing")
    require(sha256_file(path) == AUDIT_SHA256, "V15.18B audit builder hash differs from frozen contract")
    source = path.read_text(encoding="utf-8")
    try:
        tree = ast.parse(source, filename=str(path))
    except SyntaxError as error:
        raise ValidationError(f"audit builder syntax error: {clean_error(error)}") from error
    imports: set[str] = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imports.update(alias.name for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imports.add(node.module)
    forbidden = [name for name in imports if name.split(".")[0] in {"rclpy", "moveit", "moveit_commander", "rospy"}]
    require(not forbidden, f"audit imports forbidden online stack: {forbidden}")
    require("validate_passive_gravity_v15_18b" not in source, "audit imports or reads validator implementation")
    for token in ("mj_step", "mjENBL_ENERGY", "mjDSBL_ACTUATION", "--write", "--check"):
        require(token in source, f"audit static contract token missing: {token}")
    require("0.15" in source and "0.25" in source, "audit duration guard constants are not visible")
    return {"pass": True, "imports": sorted(imports), "forbidden_online_imports": [], "independent_of_validator": True}


def validate_protected_hash_report(value: Any) -> None:
    require(isinstance(value, dict), "protected_hashes must be an object")
    require(set(value) == {"artifacts", "all_unchanged", "production_model_expected_sha256"}, "protected_hashes schema differs")
    require(value["all_unchanged"] is True and value["production_model_expected_sha256"] == MJCF_SHA256, "protected_hashes summary differs")
    artifacts = value["artifacts"]
    require(isinstance(artifacts, dict) and set(artifacts) == set(AUDIT_PROTECTED_HASHES), "protected artifact identity set differs")
    for relative, expected in AUDIT_PROTECTED_HASHES.items():
        row = artifacts[relative]
        require(isinstance(row, dict) and set(row) == {"before_sha256", "after_sha256", "unchanged"}, f"protected_hashes.{relative} schema differs")
        require(row["before_sha256"] == row["after_sha256"] == expected and row["unchanged"] is True, f"protected_hashes differs for {relative}")


def validate_locked_selection(
    selection: Any,
    candidates: Sequence[Mapping[str, Any]],
) -> Mapping[str, Any]:
    require(isinstance(selection, dict), "B2 pose_selection object missing")
    expected_keys = {
        "algorithm", "candidate_selector", "candidate_selector_free_dynamics_integration_performed",
        "candidate_selector_mj_step_call_count", "source_report_relative_path", "source_report_sha256",
        "v15_18a_source_pass", "requested_slot_count", "selected_slot_count", "unique_selected_q_count",
        "max_extension_and_frozen_481_are_v15_18a_semantic_aliases", "locked_slot_projection",
        "locked_slot_projection_sha256", "locked_slot_projection_expected_sha256", "slots", "pass",
    }
    require(set(selection) == expected_keys, "B2 pose_selection schema differs")
    projection = [
        {"slot_index": float(slot), "category": category, "requested_pose_id": requested, "selected_pose_id": selected, "selected_q_rad": list(q)}
        for slot, category, requested, selected, q in LOCKED_POSES
    ]
    require_close(selection["locked_slot_projection"], projection, "locked pose projection", absolute=0.0, relative=0.0)
    require(canonical_digest(projection) == selection["locked_slot_projection_sha256"] == selection["locked_slot_projection_expected_sha256"] == LOCKED_POSE_PROJECTION_SHA256, "locked pose projection SHA differs")
    require(selection["algorithm"] == "LOCKED_V15_18B_B1_SIX_SLOT_REUSE_NO_SELECTOR_RERUN", "B2 selection algorithm differs")
    require(selection["candidate_selector"] == "NOT_EXECUTED_IN_B2" and selection["candidate_selector_free_dynamics_integration_performed"] is False and selection["candidate_selector_mj_step_call_count"] == 0, "B2 improperly reran candidate selector")
    require(selection["source_report_relative_path"] == V15_18A_JSON_REL and selection["source_report_sha256"] == V15_18A_JSON_SHA256 and selection["v15_18a_source_pass"] is True, "V15.18A locked source differs")
    require(selection["requested_slot_count"] == selection["selected_slot_count"] == 6 and selection["unique_selected_q_count"] == 5 and selection["pass"] is True, "B2 six-slot counts differ")
    candidate_by_id = {str(row["id"]): row for row in candidates}
    slots = selection["slots"]
    require(isinstance(slots, list) and len(slots) == 6, "B2 slots must contain exact six rows")
    slot_keys = {"slot_index", "category", "requested_pose_id", "selected_pose_id", "selected_q_rad", "semantic_alias_of_slot", "substitution_applied", "substitution_reason", "substitution_algorithm", "requested_q_rad", "source_global_index", "source_role", "source_collision_free", "initial_margin_rad", "selection_provenance", "pass", "failure_codes"}
    for expected, row in zip(projection, slots):
        require(isinstance(row, dict) and set(row) == slot_keys, "B2 slot schema differs")
        for key in ("slot_index", "category", "requested_pose_id", "selected_pose_id", "selected_q_rad"):
            require_close(row[key], expected[key], "B2 slot." + key, absolute=0.0, relative=0.0)
        selected = str(row["selected_pose_id"])
        require(selected in candidate_by_id and same_vector(vector(row["selected_q_rad"], 6, "slot.q"), vector(candidate_by_id[selected]["q_rad"], 6, "candidate.q")), "locked slot q differs from protected V15.18A")
        require(row["selection_provenance"] == "V15_18B_B1_ACCEPTED_SLOT_REUSED_WITHOUT_RESELECTION" and row["pass"] is True and row["failure_codes"] == [], "locked slot provenance differs")
    return selection


def validate_status_semantics(report: Mapping[str, Any], independent_pass: bool | None = None) -> None:
    require(report.get("audit_valid") is True, "audit JSON is not structurally valid")
    status = report.get("status")
    passed = report.get("pass")
    hard = report.get("hard_unresolved_items")
    unresolved = report.get("unresolved_items")
    require(isinstance(hard, list) and isinstance(unresolved, list), "unresolved lists are required")
    if passed is True:
        require(status == "PASS" and report.get("final_status") == FINAL_PASS, "PASS status/final string contradiction")
        require(not hard and not unresolved, "PASS report has unresolved items")
        require(report.get("pure_simulation_phase") == PURE_COMPLETE and report.get("next_phase") == NEXT_PHASE, "PASS freeze declaration differs")
    elif passed is False:
        require(status == "FAIL" and report.get("final_status") == FINAL_FAIL, "FAIL status/final string contradiction")
        require(bool(hard or unresolved), "physical FAIL must expose unresolved evidence")
        require(report.get("pure_simulation_phase") == PURE_INCOMPLETE and report.get("next_phase") == NEXT_REVIEW, "FAIL review/freeze declaration differs")
    else:
        raise ValidationError("pass must be boolean")
    if independent_pass is not None:
        require(passed is independent_pass, "audit PASS/FAIL differs from independent recomputation")


def compare_run_report(actual: Mapping[str, Any], expected: Mapping[str, Any], label: str) -> None:
    require(set(actual) == RUN_CORE_KEYS, f"{label}: exact run schema differs: missing={sorted(RUN_CORE_KEYS-set(actual))}, extra={sorted(set(actual)-RUN_CORE_KEYS)}")
    require_close(actual, expected, label, absolute=3.0e-12, relative=3.0e-10)


def validate_audit_report(report: Mapping[str, Any], independent: Mapping[str, Any]) -> dict[str, Any]:
    require(set(report) == TOP_LEVEL_KEYS, f"top-level schema differs: missing={sorted(TOP_LEVEL_KEYS-set(report))}, extra={sorted(set(report)-TOP_LEVEL_KEYS)}")
    require(report.get("schema") == SCHEMA, "V15.18B schema differs")
    require(report.get("revision") == REVISION, "V15.18B revision differs")
    require(report.get("source_commit") == SOURCE_COMMIT, "source_commit differs")
    require(report.get("source_branch") == SOURCE_BRANCH, "source_branch differs")
    require(report.get("target_branch") == TARGET_BRANCH, "target_branch differs")
    require(report.get("production_model_relative_path") == MJCF_REL, "production model relative path differs")
    require(report.get("production_model_sha256") == MJCF_SHA256, "production model SHA differs")
    require(report.get("mujoco_version") == "3.11.0" == independent.get("mujoco_version"), "MuJoCo version differs")
    require(report.get("numpy_version") == independent.get("numpy_version"), "NumPy version differs")
    require_close(report.get("gravity"), list(GRAVITY), "gravity", absolute=0.0, relative=0.0)
    timestep = report.get("timestep")
    require(isinstance(timestep, dict), "timestep object missing")
    require_close(timestep, {"production_dt_s": 0.002, "half_dt_s": 0.001, "duration_s": DURATION_S, "nominal_steps": 75, "half_steps": 150, "production_integrator": "implicitfast"}, "timestep", absolute=1.0e-15, relative=0.0)
    thresholds = report.get("thresholds")
    require(isinstance(thresholds, dict), "thresholds object missing")
    require_close(thresholds, expected_thresholds(), "thresholds", absolute=1.0e-15, relative=0.0)
    validate_protected_hash_report(report.get("protected_hashes"))

    audit_selection = report.get("pose_selection")
    require(isinstance(audit_selection, dict), "pose_selection object missing")
    require_close(audit_selection, independent["pose_selection"], "pose_selection", absolute=3.0e-12, relative=3.0e-10)
    audit_poses = report.get("per_pose_runs")
    expected_poses = independent["per_pose_runs"]
    require(isinstance(audit_poses, list) and len(audit_poses) == len(expected_poses) == 6, "per_pose_runs must contain exact six rows")
    required_pose_keys = {"slot_index", "category", "selected_pose_id", "selected_q_rad", "initial_state", "runs", "early_motion_direction", "energy", "convergence", "determinism", "safety", "pass", "failure_codes"}
    for index, (actual, expected) in enumerate(zip(audit_poses, expected_poses), 1):
        require(isinstance(actual, dict) and set(actual) == required_pose_keys, f"per_pose_runs[{index}] schema differs")
        for key in ("slot_index", "category", "selected_pose_id", "selected_q_rad", "initial_state", "early_motion_direction", "energy", "convergence", "determinism", "safety", "pass", "failure_codes"):
            require_close(actual[key], expected[key], f"per_pose_runs[{index}].{key}", absolute=3.0e-12, relative=3.0e-10)
        require(isinstance(actual["runs"], dict) and set(actual["runs"]) == {"A", "B", "C"}, f"per_pose_runs[{index}].runs differs")
        for label in ("A", "B", "C"):
            compare_run_report(actual["runs"][label], expected["runs"][label], f"per_pose_runs[{index}].runs.{label}")

    for key in ("initial_acceleration_identity", "early_motion_direction", "energy_audit", "timestep_convergence", "determinism", "contacts", "joint_limit_margin", "stability", "trajectory_continuity"):
        require_close(report.get(key), independent[key], key, absolute=3.0e-12, relative=3.0e-10)
    gates = report.get("acceptance_gates")
    require(isinstance(gates, dict) and set(gates) == GATE_KEYS, "acceptance gate identity set differs")
    require_close(gates, independent["acceptance_gates"], "acceptance_gates", absolute=0.0, relative=0.0)
    require_close(report.get("hard_unresolved_items"), independent["hard_unresolved_items"], "hard_unresolved_items", absolute=0.0, relative=0.0)
    require(report.get("unresolved_items") == independent["hard_unresolved_items"], "unresolved_items differs from independent hard failures")
    validate_status_semantics(report, bool(independent["pass"]))
    runtime = report.get("runtime_configuration")
    require(isinstance(runtime, dict), "runtime_configuration object missing")
    require(runtime.get("gravity_runtime_only") is True and runtime.get("production_xml_gravity_m_s2") == [0.0, 0.0, 0.0], "runtime gravity declaration differs")
    require(runtime.get("actuation_explicitly_disabled") is True and runtime.get("ctrl_zero") is True and runtime.get("qfrc_applied_zero") is True and runtime.get("xfrc_applied_zero") is True, "actuation isolation declaration differs")
    require(runtime.get("energy_enabled") is True and runtime.get("three_isolated_compiled_models") is True and runtime.get("fresh_mjdata_per_pose_and_run") is True, "runtime model isolation declaration differs")
    runtime_models = runtime.get("runtime_models")
    require(isinstance(runtime_models, dict) and set(runtime_models) == {"A", "B", "C"}, "runtime model evidence differs")
    for label, dt in (("A", 0.002), ("B", 0.001), ("C", 0.002)):
        row = runtime_models[label]
        require_close(row.get("gravity_m_s2"), list(GRAVITY), f"runtime.{label}.gravity", absolute=0.0, relative=0.0)
        require_close(row.get("timestep_s"), dt, f"runtime.{label}.dt", absolute=1.0e-15, relative=0.0)
        require(row.get("runtime_disableflags") == row.get("actuation_disable_bit") and row.get("runtime_enableflags") == row.get("energy_enable_bit"), f"runtime.{label} flags differ")
    report_contract = report.get("report_contract")
    require(isinstance(report_contract, dict), "report_contract object missing")
    require_close(report_contract, {
        "json_relative_path": REPORT_JSON_REL, "markdown_relative_path": REPORT_MD_REL,
        "write_mode": "RECOMPUTE_THEN_WRITE_JSON_AND_MARKDOWN_EVEN_FOR_LEGAL_FAIL",
        "check_mode": "RECOMPUTE_AND_REQUIRE_BOTH_FILES_BYTE_IDENTICAL",
        "json_encoding": "UTF-8_LF_SORT_KEYS_INDENT_2_NO_NAN", "markdown_encoding": "UTF-8_LF",
        "legal_fail_serialization": "NONFINITE_NUMBERS_BECOME_JSON_NULL_AND_FAILURE_CODES_REMAIN_AUDITABLE",
    }, "report_contract", absolute=0.0, relative=0.0)
    return {"pass": True, "status": report["status"], "final_status": report["final_status"]}


def validate_frozen_legacy_authority(report: Mapping[str, Any]) -> dict[str, Any]:
    poses = report.get("per_pose_runs")
    require(isinstance(poses, list) and len(poses) == 6, "legacy authority requires exact six report poses")
    projection = [{"slot_index": row["slot_index"], "A": row["runs"]["A"], "B": row["runs"]["B"], "C": row["runs"]["C"]} for row in poses]
    digest = canonical_digest(projection)
    diagnostic = report.get("production_integrator_diagnostic")
    require(isinstance(diagnostic, dict), "production integrator diagnostic missing")
    require(digest == LEGACY_RUNS_ABC_SHA256, "audit-owned legacy A/B/C complete run projection changed")
    require(diagnostic.get("legacy_runs_abc_sha256") == digest, "embedded legacy actual SHA differs from direct projection")
    require(diagnostic.get("legacy_runs_abc_expected_sha256") == LEGACY_RUNS_ABC_SHA256, "embedded legacy expected SHA differs")
    require(diagnostic.get("legacy_runs_preserved") is True, "embedded legacy preserved gate is false")
    return {"actual_sha256": digest, "expected_sha256": LEGACY_RUNS_ABC_SHA256, "preserved": True}


def validate_audit_report_b2(report: Mapping[str, Any], independent: Mapping[str, Any]) -> dict[str, Any]:
    require(set(report) == TOP_LEVEL_KEYS, f"B2 top-level schema differs: missing={sorted(TOP_LEVEL_KEYS-set(report))}, extra={sorted(set(report)-TOP_LEVEL_KEYS)}")
    require(report.get("schema") == SCHEMA and report.get("revision") == REVISION, "B2 schema/revision differs")
    require(report.get("source_commit") == SOURCE_COMMIT and report.get("source_branch") == SOURCE_BRANCH and report.get("target_branch") == TARGET_BRANCH, "B2 source identity differs")
    require(report.get("production_model_relative_path") == MJCF_REL and report.get("production_model_sha256") == MJCF_SHA256, "B2 production model identity differs")
    require(report.get("mujoco_version") == independent.get("mujoco_version") == "3.11.0" and report.get("numpy_version") == independent.get("numpy_version") == "2.2.6", "B2 runtime versions differ")
    require_close(report.get("gravity"), list(GRAVITY), "B2 gravity", absolute=0.0, relative=0.0)
    require_close(report.get("thresholds"), expected_thresholds(), "B2 thresholds", absolute=0.0, relative=0.0)
    validate_protected_hash_report(report.get("protected_hashes"))
    timestep = report.get("timestep")
    require(isinstance(timestep, dict) and set(timestep) == {"duration_s", "production_integrator", "profiles"}, "B2 timestep schema differs")
    expected_profiles = {
        "A": {"integrator": "implicitfast", "dt_s": 0.002, "classification": "PRODUCTION_INTEGRATOR_DIAGNOSTIC"},
        "B": {"integrator": "implicitfast", "dt_s": 0.001, "classification": "PRODUCTION_INTEGRATOR_DIAGNOSTIC"},
        "C": {"integrator": "implicitfast", "dt_s": 0.002, "classification": "DETERMINISM_REPEAT_OF_A"},
        "I4": {"integrator": "implicitfast", "dt_s": 0.0005, "classification": "RUNTIME_ONLY_CONVERGENCE_REFERENCE"},
        "RK4_2MS": {"integrator": "RK4", "dt_s": 0.002, "classification": "RUNTIME_ONLY_ENERGY_REFERENCE"},
        "RK4_1MS": {"integrator": "RK4", "dt_s": 0.001, "classification": "RUNTIME_ONLY_HIGH_PRECISION_REFERENCE"},
    }
    require_close(timestep, {"duration_s": DURATION_S, "production_integrator": "implicitfast", "profiles": expected_profiles}, "B2 timestep", absolute=0.0, relative=0.0)
    actual_poses, expected_poses = report.get("per_pose_runs"), independent["per_pose_runs"]
    require(isinstance(actual_poses, list) and len(actual_poses) == len(expected_poses) == 6, "B2 per_pose_runs count differs")
    pose_keys = {"slot_index", "category", "selected_pose_id", "selected_q_rad", "initial_state", "runs", "early_motion_direction", "energy", "convergence", "legacy_production_integrator_diagnostic", "numerical_integrator_attribution", "determinism", "safety", "legacy_diagnostic_failure_codes", "pass", "failure_codes"}
    run_labels = {label for label, _, _ in ATTRIBUTION_CONFIGURATIONS}
    for index, (actual, expected) in enumerate(zip(actual_poses, expected_poses), 1):
        require(isinstance(actual, dict) and set(actual) == pose_keys, f"B2 per_pose_runs[{index}] schema differs")
        require(isinstance(actual["runs"], dict) and set(actual["runs"]) == run_labels, f"B2 per_pose_runs[{index}] run labels differ")
        for label in run_labels:
            require(set(actual["runs"][label]) == RUN_CORE_KEYS, f"B2 pose {index} run {label} schema differs")
        require_close(actual, expected, f"B2 per_pose_runs[{index}]", absolute=3.0e-12, relative=3.0e-10)
    validate_frozen_legacy_authority(report)
    for key in ("initial_acceleration_identity", "early_motion_direction", "energy_audit", "timestep_convergence", "production_integrator_diagnostic", "numerical_integrator_attribution", "determinism", "contacts", "joint_limit_margin", "stability", "trajectory_continuity"):
        require_close(report.get(key), independent[key], "B2 " + key, absolute=3.0e-12, relative=3.0e-10)
    gates = report.get("acceptance_gates")
    require(isinstance(gates, dict) and set(gates) == GATE_KEYS, "B2 acceptance gate identity set differs")
    require_close(gates, independent["acceptance_gates"], "B2 acceptance gates", absolute=0.0, relative=0.0)
    require(report.get("hard_unresolved_items") == independent["hard_unresolved_items"] == [] and report.get("unresolved_items") == [], "B2 hard unresolved items differ")
    root_map = report.get("root_cause_condition_map_exact_11")
    require_close(root_map, independent["root_cause_condition_map_exact_11"], "B2 root-cause condition map", absolute=0.0, relative=0.0)
    require(report.get("root_cause_condition_keys") == list(ROOT_CAUSE_CONDITION_KEYS), "B2 root-cause condition key order differs")
    confirmed = validate_root_cause_claims(root_map, report.get("root_cause_classification"), report.get("continuous_time_dynamics_model"))
    require(confirmed and report.get("root_cause_mutation_consistency") == {"condition_count_is_exactly_11": True, "classification_confirmed_iff_all_conditions_pass": True, "continuous_time_model_pass_iff_all_conditions_pass": True, "pass": True}, "B2 root-cause mutation consistency differs")
    require(report.get("audit_valid") is True and report.get("pass") is True and report.get("status") == "PASS" and report.get("final_status") == FINAL_PASS, "B2 PASS status differs")
    require(report.get("v15_18b_passive_gravity_dynamics") == "PASS_WITH_NUMERICAL_INTEGRATOR_LIMITATION", "B2 physics status differs")
    require(report.get("production_integrator_limitation") == "PRODUCTION_IMPLICITFAST_DT_2MS_IS_NOT_AN_ENERGY-CONSERVATION_REFERENCE_FOR_UNACTUATED_PASSIVE_MOTION", "B2 limitation text differs")
    require(report.get("pure_simulation_phase") == PURE_COMPLETE and report.get("next_phase") == NEXT_PHASE, "B2 phase transition differs")
    require(report.get("ubuntu_runtime_verification") == "PENDING_ON_TARGET_HOST" and report.get("github_handoff_status") == "OUTSIDE_THIS_NUMERICAL_AUDIT_SCOPE", "B2 pending/handoff semantics differ")
    runtime = report.get("runtime_configuration")
    require(isinstance(runtime, dict) and runtime.get("candidate_selector_executed_in_b2") is False and runtime.get("candidate_selector_mj_step_call_count") == 0 and runtime.get("six_isolated_compiled_models") is True, "B2 runtime isolation differs")
    runtime_models = runtime.get("runtime_models")
    require(isinstance(runtime_models, dict) and set(runtime_models) == run_labels, "B2 runtime model labels differ")
    for label, profile in expected_profiles.items():
        require(runtime_models[label].get("runtime_integrator") == profile["integrator"] and runtime_models[label].get("timestep_s") == profile["dt_s"] and runtime_models[label].get("production_xml_unchanged") is True, f"B2 runtime profile {label} differs")
    contract = report.get("report_contract")
    require_close(contract, {"canonical_runtime": "PYTHON_3P12_NUMPY_2P2P6_MUJOCO_3P11P0", "check_mode": "RECOMPUTE_AND_REQUIRE_BOTH_FILES_BYTE_IDENTICAL", "github_handoff_claimed": False, "json_encoding": "UTF-8_LF_SORT_KEYS_INDENT_2_NO_NAN", "json_relative_path": REPORT_JSON_REL, "markdown_encoding": "UTF-8_LF", "markdown_relative_path": REPORT_MD_REL, "write_mode": "RECOMPUTE_THEN_WRITE_JSON_AND_MARKDOWN_EVEN_FOR_LEGAL_FAIL"}, "B2 report contract", absolute=0.0, relative=0.0)
    return {"pass": True, "status": report["status"], "final_status": report["final_status"]}


def legal_fail_mutation_self_test() -> dict[str, Any]:
    synthetic_selection = {"selected_slot_count": 6, "slots": [{"pass": True}] * 6, "pass": True}
    run = {
        "contact_count_max": 0, "active_joint_limit_constraint_count_max": 0,
        "minimum_joint_limit_margin_rad": 1.0, "all_values_finite": True,
        "continuity": {"pass": True},
        "safety": {"actuation_passive_and_applied_zero": True},
    }
    synthetic_pose = {
        "slot_index": 1,
        "initial_state": {"relative_qacc_error": 0.0, "residual_norm": 0.0, "identity_pass": True},
        "runs": {label: json.loads(json.dumps(run)) for label in ("A", "B", "C")},
        "early_motion_direction": {"pass": True},
        "energy": {"pass": True},
        "convergence": {"pass": True},
        "determinism": {"pass": True},
        "safety": {"pass": True},
        "failure_codes": [],
    }
    poses = [json.loads(json.dumps({**synthetic_pose, "slot_index": index})) for index in range(1, 7)]
    base = derive_acceptance_gates(synthetic_selection, poses)
    require(all(base.values()), "synthetic PASS gates are not complete")
    mutations = {"contact": ("contact_count_max", 1, "no_contacts_pass"), "joint_limit": ("active_joint_limit_constraint_count_max", 1, "no_active_joint_limits_pass"), "nonfinite": ("all_values_finite", False, "all_values_finite")}
    observed: list[str] = []
    for name, (field, value, gate) in mutations.items():
        case = json.loads(json.dumps(poses))
        case[0]["runs"]["A"][field] = value
        mutated = derive_acceptance_gates(synthetic_selection, case)
        require(mutated[gate] is False, f"legal FAIL mutation did not trip {gate}")
        hard = hard_unresolved(mutated, case)
        require("FAIL_GATE_" + gate.upper() in hard, f"legal FAIL mutation did not expose {gate}")
        observed.append(name)
    case = json.loads(json.dumps(poses))
    case[0]["safety"]["pass"] = False
    mutated = derive_acceptance_gates(synthetic_selection, case)
    require(mutated["stability_pass"] is False, "legal stability FAIL mutation did not trip")
    observed.append("stability")
    for section, gate in (("early_motion_direction", "early_motion_direction_pass"), ("energy", "dual_energy_pass"), ("convergence", "timestep_convergence_pass"), ("determinism", "determinism_pass")):
        case = json.loads(json.dumps(poses))
        case[0][section]["pass"] = False
        mutated = derive_acceptance_gates(synthetic_selection, case)
        require(mutated[gate] is False, f"legal FAIL mutation did not trip {gate}")
        observed.append(section)
    valid_fail = {
        "audit_valid": True, "pass": False, "status": "FAIL", "final_status": FINAL_FAIL,
        "pure_simulation_phase": PURE_INCOMPLETE, "next_phase": NEXT_REVIEW,
        "hard_unresolved_items": ["synthetic_failure"], "unresolved_items": [],
    }
    validate_status_semantics(valid_fail)
    malformed_rejected = False
    try:
        validate_status_semantics({**valid_fail, "pass": True, "status": "PASS", "final_status": FINAL_PASS})
    except ValidationError:
        malformed_rejected = True
    require(malformed_rejected, "contradictory PASS/unresolved report was accepted")
    return {
        "pass": True,
        "no_mujoco_invoked": True,
        "mutations": observed,
        "audit_valid_physical_fail_exit_semantics": 0,
        "malformed_contradiction_exit_semantics": 2,
    }


def capture_hashes(relatives: Iterable[str]) -> dict[str, str | None]:
    return {relative: sha256_file(repo_path(relative)) if repo_path(relative).is_file() else None for relative in sorted(set(relatives))}


def watched_hash_snapshot(assets: Mapping[str, str]) -> dict[str, str | None]:
    relatives = set(PROTECTED_HASHES) | set(assets) | {AUDIT_REL, VALIDATOR_REL, REPORT_JSON_REL, REPORT_MD_REL}
    snapshot = capture_hashes(relatives)
    require(all(snapshot[relative] == digest for relative, digest in assets.items()), "asset TOCTOU snapshot differs from authority")
    return snapshot


def run_nested_audit_check(interpreter: Path, report: Mapping[str, Any]) -> dict[str, Any]:
    json_path = repo_path(REPORT_JSON_REL)
    before = sha256_file(json_path)
    result = run_process([str(interpreter), str(repo_path(AUDIT_REL)), "--check"], timeout=300.0)
    require(result.returncode == 0, "nested audit --check failed: " + clean_error(result.stderr or result.stdout))
    require(sha256_file(json_path) == before, "nested audit --check changed JSON")
    lines = [line.strip() for line in result.stdout.splitlines() if line.strip()]
    require("AUDIT_VALID=YES" in lines, "nested audit did not emit AUDIT_VALID=YES")
    require(f"STATUS={report.get('status')}" in lines, "nested audit STATUS differs")
    require(f"FINAL_STATUS={report.get('final_status')}" in lines, "nested audit FINAL_STATUS differs")
    return {"pass": True, "stdout_contract_lines": lines}


def numeric(value: Any) -> str:
    if value is None:
        return "N/A"
    if isinstance(value, bool):
        return "PASS" if value else "FAIL"
    if isinstance(value, (int, float)):
        return format(float(value), ".17g")
    return str(value)


def render_markdown(report: Mapping[str, Any], independent: Mapping[str, Any], static_contract: Mapping[str, Any], legal_fail: Mapping[str, Any]) -> str:
    del independent, static_contract, legal_fail
    status = str(report["final_status"])
    lines = [
        "# V15.18B 短时被动重力动力学验收",
        "",
        f"**最终状态：{status}**",
        "",
        f"- `{report['pure_simulation_phase']}`",
        f"- `NEXT_PHASE = {report['next_phase']}`",
        f"- Source commit: `{report['source_commit']}`",
        f"- Production MJCF SHA256: `{report['production_model_sha256']}`",
        f"- MuJoCo: `{report['mujoco_version']}`",
        "",
        "## 范围与方法",
        "",
        "仅使用离线 Python/MuJoCo；生产 XML 保持零重力且字节不变。审计仅在内存模型设置 `0 0 -9.81 m/s²`，显式关闭 actuation，并将 ctrl、qfrc_applied、xfrc_applied 清零。每个语义槽独立执行 A=生产步长、B=半步长、C=A 重复三组 0.15 s `mj_step`。",
        "",
        "双能量证据同时采用 MuJoCo `data.energy` 与独立 `U=-Σm(g·xipos)`、`K=0.5 qvelᵀM(q)qvel`。两套势能零点按变化量对齐。",
        "候选姿态筛选不执行自由动力学：使用初始 qacc 的 10% 保守加速度包络 `q(t)=q0+0.5×1.1×qacc·t²`，在 30 个等时段、31 个配置点逐点 reset/set/`mj_forward` 扫描碰撞和限位；selector 的 `mj_step` 调用数严格为 0。",
        "",
        "## 六姿态槽与确定性替代",
        "",
        "| 槽 | 类别 | 请求姿态 | 实际姿态 | 替代 | 初始裕量/rad | 预检 |",
        "|---:|---|---|---|---|---:|---|",
    ]
    selection = report.get("pose_selection")
    if isinstance(selection, Mapping):
        for row in selection.get("slots", []):
            lines.append(f"| {row['slot_index']} | {row['category']} | {row['requested_pose_id'] or '算法选择'} | {row['selected_pose_id']} | {'是' if row['substitution_applied'] else '否'} | {float(row['initial_margin_rad']):.6g} | {'PASS' if row['pass'] else 'FAIL'} |")
        substitutions = [row for row in selection.get("slots", []) if row.get("substitution_applied")]
        if substitutions:
            lines.extend(["", "替代原因：", ""])
            for row in substitutions:
                lines.append(f"- 槽 {row['slot_index']}：`{row['substitution_reason']}`；算法 `{row['substitution_algorithm']}`。请求角度完整保留在 JSON，未被改写。")
        lines.extend(["", f"前五槽中的 max_extension 与 frozen_503_global_481 是 V15.18A 的语义别名；本报告仍保留两个独立运行槽。实际 6 槽包含 {selection['unique_selected_q_count']} 组唯一 q。"])
    else:
        lines.append("| - | - | - | - | - | - | FAIL（审计基础设施失败） |")

    lines.extend(["", "## 核心数值结果", "", "| 槽 | 姿态 | A 能量漂移 | B 能量漂移 | q 终值差/限值 | qvel 终值差/限值 | 最大单步 Δqacc | 连续性 | 确定性 | 安全 | 结果 |", "|---:|---|---:|---:|---:|---:|---:|---|---|---|---|"])
    for row in report.get("per_pose_runs", []):
        a = row["energy"]["A"]["independent"]["normalized_drift"]
        b = row["energy"]["B"]["independent"]["normalized_drift"]
        conv = row["convergence"]
        max_qacc_step = max(float(row["runs"][label]["max_abs_qacc_step_rad_s2"]) for label in ("A", "B", "C"))
        continuity_pass = all(bool(row["runs"][label]["continuity"]["pass"]) for label in ("A", "B", "C"))
        lines.append(f"| {row['slot_index']} | {row['selected_pose_id']} | {100.0*float(a):.4f}% | {100.0*float(b):.4f}% | {float(conv['final_qpos_max_abs_difference_rad']):.6g}/{float(conv['final_qpos_limit_rad']):.6g} | {float(conv['final_qvel_max_abs_difference_rad_s']):.6g}/{float(conv['final_qvel_limit_rad_s']):.6g} | {max_qacc_step:.6g} | {'PASS' if continuity_pass else 'FAIL'} | {row['determinism']['classification']} | {'PASS' if row['safety']['pass'] else 'FAIL'} | {'PASS' if row['pass'] else 'FAIL'} |")

    lines.extend(["", "## J2 / J3 / J4 早期运动方向", "", "| 槽 | J2 | J3 | J4 |", "|---:|---|---|---|"])
    for row in report.get("early_motion_direction", {}).get("rows", []):
        cells = []
        for joint in ("J2", "J3", "J4"):
            item = row["j2_j3_j4"][joint]
            cells.append(f"qacc={float(item['initial_qacc_rad_s2']):.5g}, Δq={float(item['displacement_rad']):.5g}, {'PASS' if item['sign_match'] else 'FAIL'}")
        lines.append(f"| {row['slot_index']} | {cells[0]} | {cells[1]} | {cells[2]} |")

    lines.extend(["", "## Acceptance gates", ""])
    gate_values = report.get("acceptance_gates", {})
    for key in GATE_ORDER:
        value = gate_values[key]
        lines.append(f"- `{key}`: **{'PASS' if value else 'FAIL'}**")
    unresolved = list(report.get("hard_unresolved_items", []))
    lines.extend(["", "## 未解决项", ""])
    if unresolved:
        for code in unresolved:
            lines.append(f"- `{code}`")
        lines.extend(["", "这是合同定义的合法 FAIL：未改变生产模型、未加入阻尼/摩擦/电枢/控制器，也未降低阈值。不得进入 V15.19 硬件阶段。"])
    else:
        lines.append("- 无。")

    lines.extend([
        "", "## Authority 与限制", "",
        f"受保护文件 before/after 全部一致：**{'YES' if report.get('protected_hashes', {}).get('all_unchanged') else 'NO'}**。生产 MJCF 期望 SHA256：`{MJCF_SHA256}`。",
        "",
        "本验收不以摩擦辨识、阻尼辨识、电机电枢/转子惯量、重力补偿、长时间自由落体、高速运动或视觉抓取为前置条件；这些项目也未在本报告中验证。",
        "", "## VMware 可视见证", "",
        "在 VMware Ubuntu 22.04 的可见桌面会话中运行 `python3 tools/audit_passive_gravity_v15_18b.py --visualize`。该模式显示同六姿态、同 0.15 s 被动重力过程并逐步 `viewer.sync()`；不启动 ROS、MoveIt 或 bridge，且仅为诊断见证，不替代本 JSON 数值 authority。",
        "",
    ])
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    modes = parser.add_mutually_exclusive_group()
    modes.add_argument("--write", action="store_true", help="read-only compatibility validation of the audit-owned JSON/Markdown pair")
    modes.add_argument("--check", action="store_true", help="independently recompute and require the audit-owned JSON/Markdown pair")
    parser.add_argument("--mujoco-python", default=os.environ.get("V15_18B_MUJOCO_PYTHON"))
    parser.add_argument("--physics-child", nargs=3, metavar=("MODEL", "REQUEST", "OUTPUT"), help=argparse.SUPPRESS)
    arguments = parser.parse_args()
    if arguments.physics_child:
        return physics_child(Path(arguments.physics_child[0]), Path(arguments.physics_child[1]), Path(arguments.physics_child[2]))
    try:
        require(arguments.write or arguments.check, "exactly one of --write or --check is required")
        legal_fail = legal_fail_mutation_self_test()
        root_cause_mutations = root_cause_mutation_self_test()
        protected = validate_protected_inputs()
        static_contract = validate_audit_static_contract()
        candidates, _ = derive_v15_18a_candidates()
        assets = production_asset_snapshot()
        initial_hashes = watched_hash_snapshot(assets)
        report_path = repo_path(REPORT_JSON_REL)
        markdown_path = repo_path(REPORT_MD_REL)
        require(report_path.is_file() and markdown_path.is_file(), "V15.18B2 audit-owned report pair is missing")
        require(sha256_file(report_path) == REPORT_JSON_SHA256 and sha256_file(markdown_path) == REPORT_MD_SHA256, "V15.18B2 frozen report hashes differ")
        report = read_json(report_path)
        selection = validate_locked_selection(report.get("pose_selection"), candidates)
        legacy_authority = validate_frozen_legacy_authority(report)
        interpreter = select_mujoco_python(arguments.mujoco_python)
        with tempfile.TemporaryDirectory(prefix="v15_18b_validator_") as temporary_text:
            temporary = Path(temporary_text)
            require(str(temporary).isascii(), "independent MuJoCo temporary path is not ASCII")
            model = make_ascii_model_mirror(temporary / "mirror", assets)
            official_poses = [
                {"slot_index": slot, "category": category, "id": selected, "q_rad": q}
                for slot, category, _, selected, q in LOCKED_POSES
            ]
            raw = run_attribution_recomputation(interpreter, model, official_poses, temporary)
        independent = aggregate_b2(selection, raw, legacy_authority)
        validation = validate_audit_report_b2(report, independent)
        json_before_nested = sha256_file(report_path)
        nested = run_nested_audit_check(interpreter, report)
        require(sha256_file(report_path) == json_before_nested, "nested audit changed report JSON")
        before_report = watched_hash_snapshot(assets)
        require(before_report == initial_hashes, "repository/evidence changed before audit-owned report-byte check")
        require(sha256_file(report_path) == REPORT_JSON_SHA256 and sha256_file(markdown_path) == REPORT_MD_SHA256, "V15.18B2 report pair changed or is non-deterministic")
        final_hashes = watched_hash_snapshot(assets)
        require(final_hashes == initial_hashes, "repository/evidence changed during read-only validator")
        require(validate_protected_inputs() == protected, "protected authority changed at final rehash")
        print("VALIDATION=PASS")
        print("AUDIT_VALID=YES")
        print(f"STATUS={validation['status']}")
        print(f"FINAL_STATUS={validation['final_status']}")
        print("SELECTED_POSE_SLOTS=6")
        print(f"UNIQUE_SELECTED_Q={selection['unique_selected_q_count']}")
        print(f"HARD_UNRESOLVED_ITEMS={len(independent['hard_unresolved_items'])}")
        reference = independent["numerical_integrator_attribution"]["high_precision_reference_diagnostic"]["global"]
        print(f"EARLY_MOTION_DIRECTION={reference['matching_early_direction_count']}/{reference['expected_valid_early_direction_count']}")
        print(f"ROOT_CAUSE_CLASSIFICATION={report['root_cause_classification']}")
        print(f"ROOT_CAUSE_EXACT_CONDITIONS={len(ROOT_CAUSE_CONDITION_KEYS)}")
        print(f"FROZEN_LEGACY_PROJECTION_SHA256={LEGACY_RUNS_ABC_SHA256}")
        print(f"INDEPENDENT_LEGACY_RECOMPUTE_SHA256_DIAGNOSTIC={independent['independent_legacy_recompute_sha256_diagnostic']}")
        print(f"NESTED_AUDIT_CHECK={'PASS' if nested['pass'] else 'FAIL'}")
        print(f"LEGAL_FAIL_SELF_TEST={'PASS' if legal_fail['pass'] else 'FAIL'}")
        print(f"ROOT_CAUSE_MUTATION_SELF_TEST={'PASS' if root_cause_mutations['pass'] else 'FAIL'}")
        print(f"PROTECTED_FILE_COUNT={len(PROTECTED_HASHES)}")
        print(f"PROTECTED_ASSET_COUNT={len(assets)}")
        return 0
    except (SystemExit, KeyboardInterrupt):
        raise
    except Exception as error:
        print(f"VALIDATION=FAIL\nAUDIT_VALID=NO\nERROR={clean_error(error)}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

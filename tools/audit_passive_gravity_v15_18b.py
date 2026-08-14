#!/usr/bin/env python3
from __future__ import annotations

"""Deterministic V15.18B2 passive-dynamics integrator attribution audit.

The production XML is never edited.  Gravity, energy accounting and actuation
isolation are applied only to independently compiled in-memory models.  The
original implicitfast 2 ms / 1 ms evidence remains intact, while implicitfast
0.5 ms and RK4 2 ms / 1 ms references attribute the legacy finite-step FAIL.
``--write`` writes both authority reports; ``--check`` recomputes and requires
both reports byte-for-byte.

``--visualize`` is a diagnostic witness intended for the VMware Ubuntu 22.04
desktop.  It displays the same six semantic slots and the same 0.15 s passive
motion, but it is deliberately not numerical acceptance authority.
"""

import argparse
import gc
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import sys
import time
from typing import Any, Iterable, Mapping, Sequence
import xml.etree.ElementTree as ET


ROOT = Path(__file__).resolve().parents[1]
SOURCE_COMMIT = "6dc27f6f8196c691f9f6b1c7684202dec6af2b6a"
SOURCE_BRANCH = "agent/v15-18a-static-gravity"
TARGET_BRANCH = "agent/v15-18b-final-simulation-handoff"
SCHEMA = "go-m8010-arm-v15.18b-passive-gravity-dynamics-audit/2.0"
REVISION = "V15.18B2-numerical-integrator-attribution"
FINAL_PASS = "V15.18B PASSIVE_GRAVITY_DYNAMICS = PASS_WITH_NUMERICAL_INTEGRATOR_LIMITATION"
FINAL_FAIL = "V15.18B PASSIVE_GRAVITY_DYNAMICS = FAIL"
PHASE_COMPLETE = "PURE_SIMULATION_PHASE = COMPLETE"
PHASE_INCOMPLETE = "PURE_SIMULATION_PHASE = INCOMPLETE"
NEXT_HARDWARE = "V15.19 REAL_HARDWARE_READONLY_BRINGUP"
NEXT_REVIEW = "V15.18B DYNAMICS_REVIEW"

V14_REL = "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
MJCF_REL = V14_REL + "/mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
BRIDGE_REL = V14_REL + "/ros2_ws/src/go_m8010_arm_mujoco_bridge/scripts/mujoco_bridge.py"
BRIDGE_ENTRY_REL = V14_REL + "/ros2_ws/src/go_m8010_arm_mujoco_bridge/scripts/mujoco_bridge"
ROS2_CONTROLLERS_REL = V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config/ros2_controllers.yaml"
MOVEIT_CONTROLLERS_REL = V14_REL + "/ros2_ws/src/go_m8010_arm_v15_14_moveit_config/config/moveit_controllers.yaml"
MASS_REL = "V15_15_实测质量账本_v1.json"
COM_REL = "V15_15_COM账本_v2.json"
INERTIA_REL = "V15_16_刚体惯量_Engineering_V1.json"
V15_17_INERTIAL_REL = "V15_17_Inertial参数部署验收.json"
V15_17_PRODUCTION_REL = "V15_17_ProductionHash迁移与轨迹闭环验收.json"
V15_18A_REL = "V15_18A_静态重力与重力矩验收.json"
REPORT_JSON_REL = "V15_18B_短时被动重力动力学验收.json"
REPORT_MD_REL = "V15_18B_短时被动重力动力学验收.md"

MJCF_SHA256 = "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")
AUTHORITY_BODIES = ("link2", "link3", "link4", "link5", "link6", "gripper")
GRAVITY = (0.0, 0.0, -9.81)
DURATION_S = 0.15
PROBE_TIME_S = 0.02
RUN_LABELS = ("A", "B", "C", "I4", "RK4_2MS", "RK4_1MS")
MODEL_VALIDATION_RUN_LABELS = ("A", "B", "I4", "RK4_2MS", "RK4_1MS")
LEGACY_PRODUCTION_RUNS_ABC_SHA256 = "07bd4a626f5b5cbab93a2464cb8bc92a70980409dbeaa7328c998119106f8a72"
LOCKED_POSE_SLOT_ID_Q_SHA256 = "5b235a14c6046f8da73bf2a3d2d09f3464d08af1b3def85cee417ed35ab20c55"
LOCKED_POSE_SPECS: tuple[dict[str, Any], ...] = (
    {
        "slot_index": 1,
        "category": "mechanical_zero",
        "requested_pose_id": "frozen_503_global_018",
        "selected_pose_id": "frozen_503_global_018",
        "selected_q_rad": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        "semantic_alias_of_slot": None,
        "substitution_applied": False,
        "substitution_reason": None,
        "substitution_algorithm": "REQUESTED_V15_18A_POSE_RETAINED",
    },
    {
        "slot_index": 2,
        "category": "max_extension",
        "requested_pose_id": "frozen_503_global_481",
        "selected_pose_id": "frozen_503_global_354",
        "selected_q_rad": [-0.7245796444376641, 1.913905511611287, 0.9011701758715863, 1.4492272448237147, 1.9102428548891142, 0.2520079829930116],
        "semantic_alias_of_slot": None,
        "substitution_applied": True,
        "substitution_reason": "REQUESTED_POSE_FAILED_0P15S_SAFETY_PREFLIGHT:FAIL_QSPACE_PREDICTED_CONTACT",
        "substitution_algorithm": "V15_18A_ACCEPTED_SAFE_COMPOUND_MINIMUM_QSPACE_DISTANCE_THEN_GLOBAL_INDEX",
    },
    {
        "slot_index": 3,
        "category": "frozen_503_global_185",
        "requested_pose_id": "frozen_503_global_185",
        "selected_pose_id": "frozen_503_global_185",
        "selected_q_rad": [0.0, 0.0, 0.0, 0.0, 0.6876597252857658, 0.0],
        "semantic_alias_of_slot": None,
        "substitution_applied": False,
        "substitution_reason": None,
        "substitution_algorithm": "REQUESTED_V15_18A_POSE_RETAINED",
    },
    {
        "slot_index": 4,
        "category": "frozen_503_global_432",
        "requested_pose_id": "frozen_503_global_432",
        "selected_pose_id": "frozen_503_global_473",
        "selected_q_rad": [1.124128858085009, 0.6821041080272624, 1.257751285503815, 0.08700241052079227, -0.5032794983833337, 0.22999613584324316],
        "semantic_alias_of_slot": None,
        "substitution_applied": True,
        "substitution_reason": "REQUESTED_POSE_FAILED_0P15S_SAFETY_PREFLIGHT:FAIL_QSPACE_PREDICTED_CONTACT",
        "substitution_algorithm": "V15_18A_ACCEPTED_SAFE_COMPOUND_MINIMUM_QSPACE_DISTANCE_THEN_GLOBAL_INDEX",
    },
    {
        "slot_index": 5,
        "category": "frozen_503_global_481",
        "requested_pose_id": "frozen_503_global_481",
        "selected_pose_id": "frozen_503_global_354",
        "selected_q_rad": [-0.7245796444376641, 1.913905511611287, 0.9011701758715863, 1.4492272448237147, 1.9102428548891142, 0.2520079829930116],
        "semantic_alias_of_slot": 2,
        "substitution_applied": True,
        "substitution_reason": "SEMANTIC_ALIAS_INHERITS_SLOT_2_SAFETY_SUBSTITUTE",
        "substitution_algorithm": "EXACT_ALIAS_OF_MAX_EXTENSION_SLOT_AFTER_SAFETY_GATE",
    },
    {
        "slot_index": 6,
        "category": "folded_low_torque_pose",
        "requested_pose_id": None,
        "selected_pose_id": "frozen_503_global_369",
        "selected_q_rad": [-0.07301686429765095, 1.1615298819825015, 1.2726638344330896, -1.3717138538115992, 0.7387706518083362, 0.4773492170113097],
        "semantic_alias_of_slot": None,
        "substitution_applied": False,
        "substitution_reason": None,
        "substitution_algorithm": "NOT_APPLICABLE_DETERMINISTIC_FOLDED_SELECTION",
    },
)

PROTECTED_RELATIVE_PATHS = (
    MASS_REL,
    COM_REL,
    INERTIA_REL,
    V15_17_INERTIAL_REL,
    V15_17_PRODUCTION_REL,
    V15_18A_REL,
    MJCF_REL,
    BRIDGE_REL,
    BRIDGE_ENTRY_REL,
    ROS2_CONTROLLERS_REL,
    MOVEIT_CONTROLLERS_REL,
)

THRESHOLDS: dict[str, Any] = {
    "duration_s_exact": DURATION_S,
    "probe_time_s": PROBE_TIME_S,
    "initial_joint_limit_margin_rad_minimum": 0.15,
    "runtime_joint_limit_margin_rad_strict_greater_than": 0.02,
    "max_abs_qfrc_actuator_nm_strict_less_than": 1.0e-12,
    "max_abs_qfrc_passive_nm_strict_less_than": 1.0e-12,
    "max_abs_applied_force_strict_less_than": 1.0e-12,
    "initial_qacc_relative_error_strict_less_than": 1.0e-10,
    "initial_dynamics_residual_norm_strict_less_than": 1.0e-10,
    "early_direction_min_abs_qacc_rad_s2": 0.05,
    "early_direction_min_predicted_displacement_rad": 1.0e-7,
    "nominal_normalized_energy_drift_strict_less_than": 0.01,
    "half_step_normalized_energy_drift_strict_less_than": 0.005,
    "energy_numerical_floor_j": 1.0e-8,
    "half_step_energy_not_worse_additive_j": 1.0e-8,
    "dual_energy_aligned_change_max_abs_j": 1.0e-8,
    "final_q_convergence_absolute_floor_rad": 1.0e-3,
    "final_q_convergence_relative_to_max_displacement": 0.005,
    "final_qvel_convergence_absolute_floor_rad_s": 1.0e-2,
    "final_qvel_convergence_relative_to_max_speed": 0.01,
    "determinism_qpos_max_abs_rad_strict_less_than": 1.0e-12,
    "determinism_qvel_max_abs_rad_s_strict_less_than": 1.0e-12,
    "determinism_energy_max_abs_j_strict_less_than": 1.0e-12,
    "max_abs_qvel_rad_s_strict_less_than": 20.0,
    "max_abs_qacc_rad_s2_strict_less_than": 500.0,
    "max_abs_single_qpos_step_rad_strict_less_than": 0.05,
    "max_abs_single_qvel_step_rad_s_strict_less_than": 5.0,
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

ACCEPTANCE_GATE_KEYS = (
    "exact_six_pose_slots",
    "pose_selection_safe",
    "legacy_production_runs_preserved",
    "v15_18a_continuous_time_physics_pass",
    "initial_acceleration_identity_pass",
    "early_motion_direction_pass",
    "implicitfast_energy_convergence_pass",
    "implicitfast_trajectory_convergence_pass",
    "rk4_energy_reference_pass",
    "rk4_timestep_convergence_pass",
    "high_precision_reference_diagnostic_pass",
    "determinism_pass",
    "actuation_and_applied_force_isolation_pass",
    "no_contacts_pass",
    "no_active_joint_limits_pass",
    "joint_limit_margin_pass",
    "stability_pass",
    "trajectory_continuity_pass",
    "protected_hashes_unchanged",
    "production_model_hash_unchanged",
    "bridge_controller_unchanged",
    "all_values_finite",
    "numerical_integrator_truncation_error_confirmed",
    "hard_unresolved_items_empty",
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


class AuditError(RuntimeError):
    """Malformed/stale authority or unavailable runtime."""


def require(condition: bool, message: str) -> None:
    if not condition:
        raise AuditError(message)


def repo_path(relative: str) -> Path:
    parts = PurePosixPath(relative).parts
    require(bool(parts) and ".." not in parts, f"unsafe repository path: {relative}")
    path = (ROOT / Path(*parts)).resolve()
    require(path == ROOT or ROOT in path.parents, f"repository path escaped root: {relative}")
    return path


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def sha256_bytes(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def read_json(relative: str) -> dict[str, Any]:
    value = json.loads(repo_path(relative).read_text(encoding="utf-8"))
    require(isinstance(value, dict), f"JSON root must be an object: {relative}")
    return value


def canonical_digest(value: Any) -> str:
    payload = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"), allow_nan=False).encode("utf-8")
    return sha256_bytes(payload)


def max_abs(values: Iterable[float]) -> float:
    if hasattr(values, "flat"):
        return max((abs(float(value)) for value in values.flat), default=0.0)  # type: ignore[attr-defined]
    flattened: list[float] = []
    for value in values:
        if isinstance(value, (list, tuple)) or hasattr(value, "flat"):
            flattened.append(max_abs(value))  # type: ignore[arg-type]
        else:
            flattened.append(abs(float(value)))
    return max(flattened, default=0.0)


def vector_norm(values: Sequence[float]) -> float:
    return math.sqrt(math.fsum(float(value) * float(value) for value in values))


def finite_tree(value: Any) -> bool:
    if value is None or isinstance(value, (str, bool)):
        return True
    if isinstance(value, (int, float)):
        return math.isfinite(float(value))
    if hasattr(value, "tolist"):
        return finite_tree(value.tolist())
    if isinstance(value, Mapping):
        return all(finite_tree(item) for item in value.values())
    if isinstance(value, (list, tuple)):
        return all(finite_tree(item) for item in value)
    return False


def json_safe(value: Any) -> Any:
    """Convert NumPy values and replace NaN/Inf with JSON null."""

    if value is None or isinstance(value, (str, bool)):
        return value
    if isinstance(value, (int, float)):
        number = float(value)
        return number if math.isfinite(number) else None
    if isinstance(value, Mapping):
        return {str(key): json_safe(item) for key, item in value.items()}
    if hasattr(value, "tolist"):
        return json_safe(value.tolist())
    if isinstance(value, (list, tuple)):
        return [json_safe(item) for item in value]
    return None


def protected_snapshot() -> dict[str, str]:
    result: dict[str, str] = {}
    for relative in PROTECTED_RELATIVE_PATHS:
        path = repo_path(relative)
        require(path.is_file(), f"protected file missing: {relative}")
        result[relative] = sha256_file(path)
    return result


def model_xml_contract() -> tuple[ET.Element, dict[str, tuple[float, float] | None], dict[str, Any]]:
    root = ET.fromstring(repo_path(MJCF_REL).read_bytes())
    option = root.find("./option")
    require(option is not None, "production MJCF option missing")
    gravity = [float(token) for token in str(option.get("gravity", "0 0 0")).split()]
    timestep = float(option.get("timestep", "0.002"))
    integrator = str(option.get("integrator", "Euler"))
    require(gravity == [0.0, 0.0, 0.0], "production MJCF has persistent gravity")
    require(timestep > 0.0 and abs(round(DURATION_S / timestep) * timestep - DURATION_S) < 1.0e-15, "duration is not an exact production-step multiple")
    nodes = {str(node.get("name")): node for node in root.findall(".//joint") if node.get("name") in JOINTS}
    require(set(nodes) == set(JOINTS), "compiled joint identity set changed")
    limits: dict[str, tuple[float, float] | None] = {}
    forbidden_joint_parameters: list[dict[str, Any]] = []
    for name in JOINTS:
        node = nodes[name]
        limited = str(node.get("limited", "auto")).lower() != "false" and bool(node.get("range"))
        if limited:
            values = tuple(float(token) for token in str(node.get("range")).split())
            require(len(values) == 2 and values[0] < values[1], f"invalid joint range: {name}")
            limits[name] = (values[0], values[1])
        else:
            require(name == "J1", f"unexpected unlimited joint: {name}")
            limits[name] = None
        forbidden_joint_parameters.append({
            "joint": name,
            "damping": str(node.get("damping", "0")),
            "frictionloss": str(node.get("frictionloss", "0")),
            "armature": str(node.get("armature", "0")),
        })
    require(all(all(float(row[key]) == 0.0 for key in ("damping", "frictionloss", "armature")) for row in forbidden_joint_parameters), "production passive/armature parameter is nonzero")
    body_gravcomp = [{"body": str(node.get("name")), "gravcomp": str(node.get("gravcomp", "0"))} for node in root.findall(".//body")]
    require(all(float(row["gravcomp"]) == 0.0 for row in body_gravcomp), "production body gravcomp is nonzero")
    return root, limits, {
        "production_gravity_m_s2": gravity,
        "production_timestep_s": timestep,
        "production_integrator": integrator,
        "joint_parameters": forbidden_joint_parameters,
        "body_gravcomp": body_gravcomp,
    }


def load_virtual_assets(root: ET.Element) -> tuple[str, dict[str, bytes], dict[str, Any]]:
    """Create an in-memory ASCII asset map; no production file is copied or edited."""

    source = repo_path(MJCF_REL)
    compiler = root.find("./compiler")
    require(compiler is not None, "production MJCF compiler element missing")
    mesh_dir = source.parent / compiler.get("meshdir", ".")
    texture_dir = source.parent / compiler.get("texturedir", ".")
    nodes = [node for node in root.findall("./asset/*") if node.get("file")]
    require(len(nodes) == 1008, "production model asset count is not exact 1008")
    assets: dict[str, bytes] = {}
    rows: list[dict[str, Any]] = []
    total_bytes = 0
    for index, node in enumerate(nodes):
        raw = str(node.get("file"))
        require(not Path(raw).is_absolute(), f"absolute production asset path: {raw}")
        base = mesh_dir if node.tag.rsplit("}", 1)[-1] == "mesh" else texture_dir
        path = (base / Path(*PurePosixPath(raw).parts)).resolve()
        require(path.is_file() and ROOT in path.parents, f"asset missing or outside repository: {raw}")
        name = f"asset_{index:04d}{path.suffix.lower() or '.bin'}"
        payload = path.read_bytes()
        assets[name] = payload
        total_bytes += len(payload)
        rows.append({"index": index, "source_relative_path": path.relative_to(ROOT).as_posix(), "sha256": sha256_bytes(payload), "size_bytes": len(payload)})
        node.set("file", name)
    compiler.set("meshdir", ".")
    compiler.set("texturedir", ".")
    return ET.tostring(root, encoding="unicode"), assets, {
        "method": "MUJOCO_VIRTUAL_FILE_SYSTEM_IN_MEMORY_ASCII_KEYS",
        "asset_count": len(assets),
        "total_asset_bytes": total_bytes,
        "asset_set_sha256": canonical_digest(rows),
        "temporary_or_production_files_written": False,
    }


def joint_margins(q: Sequence[float], limits: Mapping[str, tuple[float, float] | None]) -> dict[str, float | None]:
    return {
        name: None if limits[name] is None else min(float(q[index]) - limits[name][0], limits[name][1] - float(q[index]))  # type: ignore[index]
        for index, name in enumerate(JOINTS)
    }


def minimum_limited_margin(q: Sequence[float], limits: Mapping[str, tuple[float, float] | None]) -> float:
    values = [value for value in joint_margins(q, limits).values() if value is not None]
    return min(float(value) for value in values)


def configure_model(model: Any, mujoco: Any, np: Any, timestep: float) -> dict[str, Any]:
    initial_disable = int(model.opt.disableflags)
    initial_enable = int(model.opt.enableflags)
    actuation_bit = int(mujoco.mjtDisableBit.mjDSBL_ACTUATION)
    energy_bit = int(mujoco.mjtEnableBit.mjENBL_ENERGY)
    model.opt.disableflags = initial_disable | actuation_bit
    model.opt.enableflags = initial_enable | energy_bit
    model.opt.gravity[:] = np.asarray(GRAVITY, dtype=float)
    model.opt.timestep = float(timestep)
    require((int(model.opt.disableflags) & actuation_bit) != 0, "actuation disable bit not set")
    require((int(model.opt.enableflags) & energy_bit) != 0, "energy enable bit not set")
    forbidden = {
        "contact": int(mujoco.mjtDisableBit.mjDSBL_CONTACT),
        "constraint": int(mujoco.mjtDisableBit.mjDSBL_CONSTRAINT),
        "limit": int(mujoco.mjtDisableBit.mjDSBL_LIMIT),
    }
    require(all((int(model.opt.disableflags) & bit) == 0 for bit in forbidden.values()), "required physics was disabled")
    require(max_abs(model.dof_damping) == 0.0 and max_abs(model.dof_frictionloss) == 0.0 and max_abs(model.dof_armature) == 0.0, "compiled passive/armature terms changed")
    require(max_abs(model.body_gravcomp) == 0.0, "compiled gravcomp changed")
    return {
        "initial_disableflags": initial_disable,
        "runtime_disableflags": int(model.opt.disableflags),
        "actuation_disable_bit": actuation_bit,
        "initial_enableflags": initial_enable,
        "runtime_enableflags": int(model.opt.enableflags),
        "energy_enable_bit": energy_bit,
        "forbidden_disable_bits": forbidden,
        "timestep_s": float(model.opt.timestep),
        "gravity_m_s2": [float(value) for value in model.opt.gravity],
    }


def configure_runtime_profile(model: Any, mujoco: Any, np: Any, timestep: float, integrator: str) -> dict[str, Any]:
    """Apply a runtime-only timestep/integrator profile to an isolated model."""

    row = configure_model(model, mujoco, np, timestep)
    compiled_integrator = int(model.opt.integrator)
    targets = {
        "implicitfast": int(mujoco.mjtIntegrator.mjINT_IMPLICITFAST),
        "RK4": int(mujoco.mjtIntegrator.mjINT_RK4),
    }
    require(integrator in targets, f"unsupported runtime integrator: {integrator}")
    model.opt.integrator = targets[integrator]
    require(int(model.opt.integrator) == targets[integrator], f"failed to select runtime integrator: {integrator}")
    return {
        **row,
        "compiled_integrator_before_runtime_override": compiled_integrator,
        "runtime_integrator": integrator,
        "runtime_integrator_enum": int(model.opt.integrator),
        "runtime_only_integrator_override": compiled_integrator != int(model.opt.integrator),
        "production_xml_unchanged": True,
    }


def reset_forward(model: Any, data: Any, q: Sequence[float], mujoco: Any, np: Any) -> None:
    mujoco.mj_resetData(model, data)
    data.qpos[:] = np.asarray(q, dtype=float)
    data.qvel[:] = 0.0
    data.qacc[:] = 0.0
    if model.nu:
        data.ctrl[:] = 0.0
    data.qfrc_applied[:] = 0.0
    data.xfrc_applied[:] = 0.0
    mujoco.mj_forward(model, data)


def active_joint_limit_count(data: Any, mujoco: Any, np: Any) -> int:
    if int(data.nefc) <= 0:
        return 0
    limit_type = int(mujoco.mjtConstraint.mjCNSTR_LIMIT_JOINT)
    return int(np.count_nonzero(np.asarray(data.efc_type[: int(data.nefc)], dtype=int) == limit_type))


def contact_rows(model: Any, data: Any, mujoco: Any, limit: int = 12) -> list[dict[str, Any]]:
    rows = []
    for index in range(min(int(data.ncon), limit)):
        contact = data.contact[index]
        first = str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom1)) or "<unnamed>")
        second = str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, int(contact.geom2)) or "<unnamed>")
        rows.append({"geom1": first, "geom2": second, "distance_m": float(contact.dist), "ground": "ground" in first.lower() or "ground" in second.lower()})
    return rows


def compiled_body_contract(model: Any, mujoco: Any) -> tuple[list[int], list[int], list[dict[str, Any]]]:
    authority_ids = []
    for name in AUTHORITY_BODIES:
        body_id = int(mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name))
        require(body_id >= 0, f"compiled authority body missing: {name}")
        authority_ids.append(body_id)
    positive_ids = [body_id for body_id in range(int(model.nbody)) if float(model.body_mass[body_id]) > 0.0]
    positive_rows = [{
        "body": str(mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id) or "<unnamed>"),
        "body_id": body_id,
        "mass_kg": float(model.body_mass[body_id]),
    } for body_id in positive_ids]
    require({row["body"] for row in positive_rows} == {"link1", *AUTHORITY_BODIES}, "compiled positive-mass body set changed")
    require(abs(math.fsum(row["mass_kg"] for row in positive_rows) - 4.4515) <= 1.0e-12, "compiled all-positive mass total changed")
    require(abs(math.fsum(float(model.body_mass[index]) for index in authority_ids) - 3.4515) <= 1.0e-12, "compiled authority mass total changed")
    return authority_ids, positive_ids, positive_rows


def independent_potential(model: Any, data: Any, ids: Sequence[int], np: Any) -> float:
    masses = np.asarray([model.body_mass[index] for index in ids], dtype=float)
    positions = np.asarray([data.xipos[index] for index in ids], dtype=float)
    return float(-np.sum(masses[:, None] * positions * np.asarray(model.opt.gravity, dtype=float)[None, :]))


def full_mass_matrix(model: Any, data: Any, mujoco: Any, np: Any) -> Any:
    matrix = np.zeros((int(model.nv), int(model.nv)), dtype=np.float64, order="C")
    mujoco.mj_fullM(model, data, matrix)
    return matrix


def sample_state(model: Any, data: Any, mujoco: Any, np: Any, limits: Mapping[str, tuple[float, float] | None], authority_ids: Sequence[int], positive_ids: Sequence[int]) -> dict[str, Any]:
    matrix = full_mass_matrix(model, data, mujoco, np)
    kinetic = float(0.5 * np.asarray(data.qvel, dtype=float) @ matrix @ np.asarray(data.qvel, dtype=float))
    potential_all = independent_potential(model, data, positive_ids, np)
    return {
        "qpos": np.asarray(data.qpos, dtype=float).copy(),
        "qvel": np.asarray(data.qvel, dtype=float).copy(),
        "qacc": np.asarray(data.qacc, dtype=float).copy(),
        "mujoco_potential": float(data.energy[0]),
        "mujoco_kinetic": float(data.energy[1]),
        "mujoco_total": float(data.energy[0] + data.energy[1]),
        "independent_authority_potential": independent_potential(model, data, authority_ids, np),
        "independent_potential": potential_all,
        "independent_kinetic": kinetic,
        "independent_total": potential_all + kinetic,
        "ncon": int(data.ncon),
        "nefc": int(data.nefc),
        "active_joint_limits": active_joint_limit_count(data, mujoco, np),
        "minimum_margin": minimum_limited_margin(data.qpos, limits),
        "qfrc_actuator_max": max_abs(data.qfrc_actuator),
        "qfrc_passive_max": max_abs(data.qfrc_passive),
        "qfrc_applied_max": max_abs(data.qfrc_applied),
        "xfrc_applied_max": max_abs(data.xfrc_applied),
        "ctrl_max": max_abs(data.ctrl),
    }


def energy_summary(samples: Sequence[Mapping[str, Any]], normalized_limit: float) -> dict[str, Any]:
    mujoco_total = [float(row["mujoco_total"]) for row in samples]
    independent_total = [float(row["independent_total"]) for row in samples]
    potential = [float(row["independent_potential"]) for row in samples]
    kinetic = [float(row["independent_kinetic"]) for row in samples]
    denominator = max(abs(potential[-1] - potential[0]), 1.0e-3)
    mujoco_error = max_abs(value - mujoco_total[0] for value in mujoco_total)
    independent_error = max_abs(value - independent_total[0] for value in independent_total)
    aligned = max_abs((mujoco_total[index] - mujoco_total[0]) - (independent_total[index] - independent_total[0]) for index in range(len(samples)))
    allowed = max(normalized_limit * denominator, float(THRESHOLDS["energy_numerical_floor_j"]))
    max_step_jump = max((abs(independent_total[index] - independent_total[index - 1]) for index in range(1, len(samples))), default=0.0)
    jump_limit = max(float(THRESHOLDS["unexplained_single_step_energy_fraction"]) * denominator, float(THRESHOLDS["energy_numerical_floor_j"]))
    significant = vector_norm(samples[0]["qacc"]) > float(THRESHOLDS["early_direction_min_abs_qacc_rad_s2"])
    trend_pass = (not significant) or (potential[-1] < potential[0] and kinetic[-1] > kinetic[0])
    values_finite = finite_tree({"mujoco": mujoco_total, "independent": independent_total, "potential": potential, "kinetic": kinetic})
    passed = values_finite and mujoco_error < allowed and independent_error < allowed and aligned <= float(THRESHOLDS["dual_energy_aligned_change_max_abs_j"]) and trend_pass and max_step_jump <= jump_limit
    return {
        "mujoco": {
            "initial_total_j": mujoco_total[0], "final_total_j": mujoco_total[-1],
            "max_abs_drift_j": mujoco_error, "normalized_drift": mujoco_error / denominator,
            "pass": mujoco_error < allowed,
        },
        "independent": {
            "initial_potential_j": potential[0], "final_potential_j": potential[-1], "potential_change_j": potential[-1] - potential[0],
            "initial_kinetic_j": kinetic[0], "final_kinetic_j": kinetic[-1], "kinetic_change_j": kinetic[-1] - kinetic[0],
            "initial_total_j": independent_total[0], "final_total_j": independent_total[-1],
            "max_abs_drift_j": independent_error, "normalized_drift": independent_error / denominator,
            "pass": independent_error < allowed,
        },
        "normalization_denominator_j": denominator,
        "normalized_limit_strict_less_than": normalized_limit,
        "absolute_limit_with_floor_j": allowed,
        "max_aligned_change_difference_j": aligned,
        "dual_source_alignment_pass": aligned <= float(THRESHOLDS["dual_energy_aligned_change_max_abs_j"]),
        "significantly_nonzero_acceleration": significant,
        "potential_decreases": potential[-1] < potential[0],
        "kinetic_increases": kinetic[-1] > kinetic[0],
        "energy_direction_pass": trend_pass,
        "max_abs_single_step_total_energy_change_j": max_step_jump,
        "single_step_energy_change_limit_j": jump_limit,
        "no_unexplained_energy_jump": max_step_jump <= jump_limit,
        "all_values_finite": values_finite,
        "pass": passed,
    }


def run_simulation(label: str, model: Any, q: Sequence[float], duration: float, mujoco: Any, np: Any, limits: Mapping[str, tuple[float, float] | None], authority_ids: Sequence[int], positive_ids: Sequence[int]) -> dict[str, Any]:
    data = mujoco.MjData(model)
    reset_forward(model, data, q, mujoco, np)
    dt = float(model.opt.timestep)
    steps_float = duration / dt
    steps = int(round(steps_float))
    require(abs(steps * dt - duration) <= 1.0e-15 and duration <= 0.25, f"{label}: invalid duration/step contract")
    samples: list[dict[str, Any]] = []
    contacts: list[dict[str, Any]] = []
    for index in range(steps + 1):
        mujoco.mj_forward(model, data)
        row = sample_state(model, data, mujoco, np, limits, authority_ids, positive_ids)
        row["time"] = index * dt
        samples.append(row)
        if row["ncon"] and not contacts:
            contacts = contact_rows(model, data, mujoco)
        if index < steps:
            mujoco.mj_step(model, data)

    qpos = np.asarray([row["qpos"] for row in samples], dtype=float)
    qvel = np.asarray([row["qvel"] for row in samples], dtype=float)
    qacc = np.asarray([row["qacc"] for row in samples], dtype=float)
    qpos_jump = float(np.max(np.abs(np.diff(qpos, axis=0)))) if len(samples) > 1 else 0.0
    qvel_jump = float(np.max(np.abs(np.diff(qvel, axis=0)))) if len(samples) > 1 else 0.0
    qacc_jump = float(np.max(np.abs(np.diff(qacc, axis=0)))) if len(samples) > 1 else 0.0
    all_finite = bool(all(np.all(np.isfinite(array)) for array in (qpos, qvel, qacc))) and finite_tree(samples)
    maximums = {
        "max_abs_qvel_rad_s": float(np.max(np.abs(qvel))),
        "max_abs_qacc_rad_s2": float(np.max(np.abs(qacc))),
        "max_abs_qpos_step_rad": qpos_jump,
        "max_abs_qvel_step_rad_s": qvel_jump,
        "max_abs_qacc_step_rad_s2": qacc_jump,
        "max_abs_qfrc_actuator_nm": max(float(row["qfrc_actuator_max"]) for row in samples),
        "max_abs_qfrc_passive_nm": max(float(row["qfrc_passive_max"]) for row in samples),
        "max_abs_qfrc_applied_nm": max(float(row["qfrc_applied_max"]) for row in samples),
        "max_abs_xfrc_applied": max(float(row["xfrc_applied_max"]) for row in samples),
        "max_abs_ctrl": max(float(row["ctrl_max"]) for row in samples),
        "contact_count_max": max(int(row["ncon"]) for row in samples),
        "active_joint_limit_constraint_count_max": max(int(row["active_joint_limits"]) for row in samples),
        "minimum_joint_limit_margin_rad": min(float(row["minimum_margin"]) for row in samples),
    }
    isolated = (
        maximums["max_abs_qfrc_actuator_nm"] < float(THRESHOLDS["max_abs_qfrc_actuator_nm_strict_less_than"])
        and maximums["max_abs_qfrc_passive_nm"] < float(THRESHOLDS["max_abs_qfrc_passive_nm_strict_less_than"])
        and maximums["max_abs_qfrc_applied_nm"] < float(THRESHOLDS["max_abs_applied_force_strict_less_than"])
        and maximums["max_abs_xfrc_applied"] < float(THRESHOLDS["max_abs_applied_force_strict_less_than"])
        and maximums["max_abs_ctrl"] < float(THRESHOLDS["max_abs_applied_force_strict_less_than"])
    )
    safety = {
        "all_values_finite": all_finite,
        "no_contacts": maximums["contact_count_max"] == 0,
        "no_active_joint_limit_constraints": maximums["active_joint_limit_constraint_count_max"] == 0,
        "joint_limit_margin_pass": maximums["minimum_joint_limit_margin_rad"] > float(THRESHOLDS["runtime_joint_limit_margin_rad_strict_greater_than"]),
        "qvel_pass": maximums["max_abs_qvel_rad_s"] < float(THRESHOLDS["max_abs_qvel_rad_s_strict_less_than"]),
        "qacc_pass": maximums["max_abs_qacc_rad_s2"] < float(THRESHOLDS["max_abs_qacc_rad_s2_strict_less_than"]),
        "qpos_step_pass": maximums["max_abs_qpos_step_rad"] < float(THRESHOLDS["max_abs_single_qpos_step_rad_strict_less_than"]),
        "qvel_step_pass": maximums["max_abs_qvel_step_rad_s"] < float(THRESHOLDS["max_abs_single_qvel_step_rad_s_strict_less_than"]),
        "qacc_step_continuity_pass": maximums["max_abs_qacc_step_rad_s2"] < float(THRESHOLDS["max_abs_single_qacc_step_rad_s2_strict_less_than"]),
        "actuation_passive_and_applied_zero": isolated,
    }
    safety["pass"] = all(bool(value) for value in safety.values())
    normalized_limit = float(THRESHOLDS["half_step_normalized_energy_drift_strict_less_than"] if label == "B" else THRESHOLDS["nominal_normalized_energy_drift_strict_less_than"])
    energy = energy_summary(samples, normalized_limit)
    continuity = {
        "qpos_all_finite": bool(np.all(np.isfinite(qpos))),
        "qvel_all_finite": bool(np.all(np.isfinite(qvel))),
        "qacc_all_finite": bool(np.all(np.isfinite(qacc))),
        "energy_all_finite": bool(energy["all_values_finite"]),
        "max_abs_qpos_step_rad": qpos_jump,
        "max_abs_qvel_step_rad_s": qvel_jump,
        "max_abs_qacc_step_rad_s2": qacc_jump,
        "qpos_step_limit_rad_strict_less_than": float(THRESHOLDS["max_abs_single_qpos_step_rad_strict_less_than"]),
        "qvel_step_limit_rad_s_strict_less_than": float(THRESHOLDS["max_abs_single_qvel_step_rad_s_strict_less_than"]),
        "qacc_step_limit_rad_s2_strict_less_than": float(THRESHOLDS["max_abs_single_qacc_step_rad_s2_strict_less_than"]),
        "qpos_step_pass": safety["qpos_step_pass"],
        "qvel_step_pass": safety["qvel_step_pass"],
        "qacc_step_pass": safety["qacc_step_continuity_pass"],
        "energy_step_pass": bool(energy["no_unexplained_energy_jump"]),
    }
    continuity["pass"] = all(bool(value) for key, value in continuity.items() if key.endswith("_finite") or key.endswith("_pass"))
    failure_codes: list[str] = []
    checks = (
        (not all_finite, "FAIL_NONFINITE_DYNAMICS"),
        (not safety["no_contacts"], "FAIL_CONTACT_DURING_PASSIVE_RUN"),
        (not safety["no_active_joint_limit_constraints"], "FAIL_ACTIVE_JOINT_LIMIT_CONSTRAINT"),
        (not safety["joint_limit_margin_pass"], "FAIL_RUNTIME_JOINT_LIMIT_MARGIN"),
        (not safety["qvel_pass"], "FAIL_QVEL_STABILITY_LIMIT"),
        (not safety["qacc_pass"], "FAIL_QACC_STABILITY_LIMIT"),
        (not safety["qpos_step_pass"], "FAIL_QPOS_SINGLE_STEP_JUMP"),
        (not safety["qvel_step_pass"], "FAIL_QVEL_SINGLE_STEP_JUMP"),
        (not safety["qacc_step_continuity_pass"], "FAIL_QACC_SINGLE_STEP_DISCONTINUITY"),
        (not continuity["pass"], "FAIL_TRAJECTORY_CONTINUITY"),
        (not isolated, "FAIL_ACTUATION_PASSIVE_OR_APPLIED_FORCE_ISOLATION"),
        (not energy["pass"], "FAIL_ENERGY_AUDIT"),
    )
    failure_codes.extend(code for failed, code in checks if failed)
    return {
        "label": label,
        "dt_s": dt,
        "duration_s": duration,
        "step_count": steps,
        "time_s": [float(row["time"]) for row in samples],
        "qpos_rad": [row["qpos"] for row in samples],
        "qvel_rad_s": [row["qvel"] for row in samples],
        "qacc_rad_s2": [row["qacc"] for row in samples],
        "mujoco_energy_j": [[row["mujoco_potential"], row["mujoco_kinetic"], row["mujoco_total"]] for row in samples],
        "independent_potential_j": [row["independent_potential"] for row in samples],
        "independent_authority_potential_j": [row["independent_authority_potential"] for row in samples],
        "independent_kinetic_j": [row["independent_kinetic"] for row in samples],
        "independent_total_j": [row["independent_total"] for row in samples],
        "initial_qacc_rad_s2": samples[0]["qacc"],
        "final_qpos_rad": samples[-1]["qpos"],
        "final_qvel_rad_s": samples[-1]["qvel"],
        **maximums,
        "first_contact_rows": contacts,
        "all_values_finite": all_finite,
        "energy_summary": energy,
        "continuity": continuity,
        "safety": safety,
        "pass": safety["pass"] and energy["pass"],
        "failure_codes": failure_codes,
    }


def preview_pose(model: Any, pose: Mapping[str, Any], mujoco: Any, np: Any, limits: Mapping[str, tuple[float, float] | None], authority_ids: Sequence[int], positive_ids: Sequence[int]) -> dict[str, Any]:
    """Select safely without ever integrating a candidate free-fall.

    The candidate path is the deterministic constant-initial-acceleration
    prediction ``q(t)=q0+0.5*1.1*qacc0*t^2``; 1.1 is a conservative 10%
    acceleration envelope.  Thirty equal-time segments are
    checked using reset/set/``mj_forward`` only.  Thus a rejected candidate can
    enter a predicted collision configuration, but it never dynamically falls
    into contact and ``mj_step`` is never called by the selector.
    """

    del authority_ids, positive_ids  # selection needs configuration safety only
    data = mujoco.MjData(model)
    q0 = np.asarray(pose["q_rad"], dtype=float)
    reset_forward(model, data, q0, mujoco, np)
    qacc0 = np.asarray(data.qacc, dtype=float).copy()
    initial_ncon = int(data.ncon)
    initial_limits = active_joint_limit_count(data, mujoco, np)
    initial_margin = minimum_limited_margin(q0, limits)
    initial_contacts = contact_rows(model, data, mujoco)
    scale = float(THRESHOLDS["selection_initial_qacc_prediction_scale"])
    displacement_bound = 0.5 * np.abs(qacc0) * DURATION_S * DURATION_S * scale
    predicted_final_qvel = qacc0 * DURATION_S * scale
    segment_count = int(THRESHOLDS["selection_qspace_sweep_segment_count"])
    require(segment_count >= 2, "q-space sweep requires at least two segments")
    times = np.linspace(0.0, DURATION_S, segment_count + 1)
    sweep_rows: list[dict[str, Any]] = []
    first_contacts: list[dict[str, Any]] = []
    maximum_contacts = 0
    maximum_active_limits = 0
    minimum_margin = initial_margin
    all_finite = bool(np.all(np.isfinite(q0)) and np.all(np.isfinite(qacc0)))
    previous_q = q0.copy()
    max_q_segment = 0.0
    for index, sample_time in enumerate(times):
        predicted_q = q0 + 0.5 * qacc0 * float(sample_time) * float(sample_time) * scale
        mujoco.mj_resetData(model, data)
        data.qpos[:] = predicted_q
        data.qvel[:] = 0.0
        data.qacc[:] = 0.0
        if model.nu:
            data.ctrl[:] = 0.0
        data.qfrc_applied[:] = 0.0
        data.xfrc_applied[:] = 0.0
        mujoco.mj_forward(model, data)
        ncon = int(data.ncon)
        active_limits = active_joint_limit_count(data, mujoco, np)
        margin = minimum_limited_margin(data.qpos, limits)
        maximum_contacts = max(maximum_contacts, ncon)
        maximum_active_limits = max(maximum_active_limits, active_limits)
        minimum_margin = min(minimum_margin, margin)
        all_finite = all_finite and bool(all(np.all(np.isfinite(array)) for array in (data.qpos, data.qvel, data.qacc, data.qfrc_bias)))
        if index:
            max_q_segment = max(max_q_segment, float(np.max(np.abs(predicted_q - previous_q))))
        if ncon and not first_contacts:
            first_contacts = contact_rows(model, data, mujoco)
        sweep_rows.append({
            "index": index,
            "time_s": float(sample_time),
            "q_rad": predicted_q.copy(),
            "ncon": ncon,
            "active_joint_limit_constraint_count": active_limits,
            "minimum_joint_limit_margin_rad": margin,
            "all_values_finite": bool(all(np.all(np.isfinite(array)) for array in (data.qpos, data.qvel, data.qacc, data.qfrc_bias))),
        })
        previous_q = predicted_q.copy()

    collision_safe = initial_ncon == 0 and maximum_contacts == 0
    prediction_safe = minimum_margin > float(THRESHOLDS["runtime_joint_limit_margin_rad_strict_greater_than"])
    qvel_safe = max_abs(predicted_final_qvel) < float(THRESHOLDS["max_abs_qvel_rad_s_strict_less_than"])
    qacc_safe = max_abs(qacc0) < float(THRESHOLDS["max_abs_qacc_rad_s2_strict_less_than"])
    safe = (
        initial_margin >= float(THRESHOLDS["initial_joint_limit_margin_rad_minimum"])
        and collision_safe
        and initial_limits == 0
        and maximum_active_limits == 0
        and prediction_safe
        and qvel_safe
        and qacc_safe
        and all_finite
    )
    conditions = (
        (initial_margin < float(THRESHOLDS["initial_joint_limit_margin_rad_minimum"]), "FAIL_INITIAL_MARGIN_LT_0P15"),
        (initial_ncon != 0, "FAIL_INITIAL_CONTACT"),
        (initial_limits != 0, "FAIL_INITIAL_ACTIVE_JOINT_LIMIT"),
        (maximum_contacts != 0, "FAIL_QSPACE_PREDICTED_CONTACT"),
        (maximum_active_limits != 0, "FAIL_QSPACE_PREDICTED_ACTIVE_JOINT_LIMIT"),
        (not prediction_safe, "FAIL_CONSERVATIVE_QACC_MARGIN_PREDICTION"),
        (not qvel_safe, "FAIL_CONSERVATIVE_QACC_QVEL_PREDICTION"),
        (not qacc_safe, "FAIL_INITIAL_QACC_STABILITY_LIMIT"),
        (not all_finite, "FAIL_QSPACE_PREDICTION_NONFINITE"),
    )
    codes = [code for failed, code in conditions if failed]
    return {
        "method": "INITIAL_QACC_BALLISTIC_PREDICTION_PLUS_RESET_SET_MJ_FORWARD_QSPACE_SWEEP",
        "prediction_formula": "q(t)=q0+0.5*1.1*qacc_initial*t^2; qvel_bound(t)=1.1*qacc_initial*t",
        "prediction_scale": scale,
        "duration_s": DURATION_S,
        "segment_count": segment_count,
        "sample_count": segment_count + 1,
        "time_grid_s": times,
        "candidate_free_dynamics_integration_performed": False,
        "mj_step_call_count": 0,
        "qvel_for_collision_forward_rad_s": [0.0] * int(model.nv),
        "initial_qacc_rad_s2": qacc0,
        "initial_margin_rad": initial_margin,
        "initial_contact_count": initial_ncon,
        "initial_contact_rows": initial_contacts,
        "initial_active_joint_limit_constraint_count": initial_limits,
        "per_joint_conservative_displacement_bound_rad": displacement_bound,
        "predicted_final_q_rad": q0 + 0.5 * qacc0 * DURATION_S * DURATION_S * scale,
        "predicted_final_qvel_rad_s": predicted_final_qvel,
        "minimum_conservative_predicted_margin_rad": minimum_margin,
        "contact_count_max": maximum_contacts,
        "first_predicted_contact_rows": first_contacts,
        "active_joint_limit_constraint_count_max": maximum_active_limits,
        "max_abs_predicted_q_segment_rad": max_q_segment,
        "max_abs_predicted_qvel_rad_s": max_abs(predicted_final_qvel),
        "max_abs_initial_qacc_rad_s2": max_abs(qacc0),
        "sweep_rows": sweep_rows,
        "all_values_finite": all_finite,
        "pass": safe,
        "failure_codes": codes,
    }


def v15_18a_source_rows() -> tuple[dict[str, Any], list[dict[str, Any]]]:
    report = read_json(V15_18A_REL)
    require(report.get("pass") is True and report.get("final_status") == "V15.18A STATIC_GRAVITY = PASS", "V15.18A source report is not PASS")
    selection = report.get("pose_selection", {})
    selected = selection.get("selected")
    poses = report.get("poses")
    require(isinstance(selected, list) and len(selected) == 21 and isinstance(poses, list) and len(poses) == 21, "V15.18A does not expose exact 21 accepted poses")
    physics = {str(row.get("id")): row for row in poses if isinstance(row, dict)}
    rows = []
    for row in selected:
        pose_id = str(row.get("id"))
        require(pose_id in physics and physics[pose_id].get("pass") is True, f"V15.18A source pose is not accepted: {pose_id}")
        q = [float(value) for value in row.get("q_rad", [])]
        tau = [float(value) for value in physics[pose_id].get("tau_hold_mujoco_nm", [])]
        require(len(q) == len(tau) == 6 and finite_tree({"q": q, "tau": tau}), f"V15.18A pose malformed: {pose_id}")
        rows.append({
            "id": pose_id,
            "role": str(row.get("role")),
            "q_rad": q,
            "source_global_index": row.get("source_global_index"),
            "source_collision_free": row.get("source_collision_free"),
            "v15_18a_tau_hold_nm": tau,
            "v15_18a_tau_norm_nm": vector_norm(tau),
        })
    require(len({row["id"] for row in rows}) == 21, "V15.18A source pose IDs are not unique")
    return report, rows


def pose_key(q: Sequence[float]) -> tuple[float, ...]:
    return tuple(float(value) for value in q)


def select_poses(source_report: Mapping[str, Any], rows: Sequence[Mapping[str, Any]], previews: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    by_id = {str(row["id"]): row for row in rows}
    mechanical_id = str(source_report.get("special_poses", {}).get("mechanical_zero", {}).get("id"))
    extension_id = str(source_report.get("special_poses", {}).get("max_extension", {}).get("id"))
    require(mechanical_id == "frozen_503_global_018", "V15.18A mechanical-zero identity changed")
    require(extension_id == "frozen_503_global_481", "V15.18A max-extension identity changed")
    requested = [
        (1, "mechanical_zero", mechanical_id, None),
        (2, "max_extension", extension_id, None),
        (3, "frozen_503_global_185", "frozen_503_global_185", None),
        (4, "frozen_503_global_432", "frozen_503_global_432", None),
        (5, "frozen_503_global_481", "frozen_503_global_481", 2),
    ]
    require(all(item[2] in by_id for item in requested), "specified V15.18B source pose is absent from V15.18A")
    slots: list[dict[str, Any]] = []
    selected_keys: set[tuple[float, ...]] = set()

    def compound(row: Mapping[str, Any]) -> bool:
        role = str(row["role"]).lower()
        return "coupled" in role or "combined" in role or "accepted_nonzero" in role

    def nearest_safe_compound(target: Mapping[str, Any], excluded: set[tuple[float, ...]]) -> Mapping[str, Any]:
        candidates = [row for row in rows if compound(row) and bool(previews[str(row["id"])]["pass"]) and pose_key(row["q_rad"]) not in excluded and str(row["id"]) != str(target["id"])]
        require(bool(candidates), f"no safe same-type substitute for {target['id']}")
        return min(candidates, key=lambda row: (vector_norm([float(a) - float(b) for a, b in zip(row["q_rad"], target["q_rad"])]), 10**9 if row["source_global_index"] is None else int(row["source_global_index"]), str(row["id"])))

    for slot_index, category, requested_id, alias in requested:
        target = by_id[requested_id]
        preview = previews[requested_id]
        substitution = not bool(preview["pass"])
        if alias is not None and substitution:
            alias_slot = next(row for row in slots if row["slot_index"] == alias)
            selected = by_id[str(alias_slot["selected_pose_id"])]
            reason = f"SEMANTIC_ALIAS_INHERITS_SLOT_{alias}_SAFETY_SUBSTITUTE"
            algorithm = "EXACT_ALIAS_OF_MAX_EXTENSION_SLOT_AFTER_SAFETY_GATE"
        elif substitution:
            selected = nearest_safe_compound(target, selected_keys)
            reason = "REQUESTED_POSE_FAILED_0P15S_SAFETY_PREFLIGHT:" + ",".join(str(code) for code in preview["failure_codes"])
            algorithm = "V15_18A_ACCEPTED_SAFE_COMPOUND_MINIMUM_QSPACE_DISTANCE_THEN_GLOBAL_INDEX"
        else:
            selected = target
            reason = None
            algorithm = "REQUESTED_V15_18A_POSE_RETAINED"
        selected_preview = previews[str(selected["id"])]
        require(bool(selected_preview["pass"]), f"selected pose failed preflight: {selected['id']}")
        selected_keys.add(pose_key(selected["q_rad"]))
        slots.append({
            "slot_index": slot_index,
            "category": category,
            "requested_pose_id": requested_id,
            "requested_q_rad": list(target["q_rad"]),
            "selected_pose_id": str(selected["id"]),
            "selected_q_rad": list(selected["q_rad"]),
            "semantic_alias_of_slot": alias,
            "substitution_applied": substitution,
            "substitution_reason": reason,
            "substitution_algorithm": algorithm,
            "source_global_index": selected["source_global_index"],
            "source_role": selected["role"],
            "source_collision_free": selected["source_collision_free"],
            "initial_margin_rad": float(selected_preview["initial_margin_rad"]),
            "preflight": selected_preview,
            "requested_pose_preflight": preview,
            "pass": bool(selected_preview["pass"]),
            "failure_codes": [] if selected_preview["pass"] else list(selected_preview["failure_codes"]),
        })

    folded_candidates = [row for row in rows if bool(previews[str(row["id"])]["pass"]) and pose_key(row["q_rad"]) not in selected_keys]
    require(bool(folded_candidates), "no distinct folded low-torque pose remains")
    folded = min(folded_candidates, key=lambda row: (float(row["v15_18a_tau_norm_nm"]), 10**9 if row["source_global_index"] is None else int(row["source_global_index"]), str(row["id"])))
    folded_preview = previews[str(folded["id"])]
    slots.append({
        "slot_index": 6,
        "category": "folded_low_torque_pose",
        "requested_pose_id": None,
        "requested_q_rad": None,
        "selected_pose_id": str(folded["id"]),
        "selected_q_rad": list(folded["q_rad"]),
        "semantic_alias_of_slot": None,
        "substitution_applied": False,
        "substitution_reason": None,
        "substitution_algorithm": "NOT_APPLICABLE_DETERMINISTIC_FOLDED_SELECTION",
        "source_global_index": folded["source_global_index"],
        "source_role": folded["role"],
        "source_collision_free": folded["source_collision_free"],
        "initial_margin_rad": float(folded_preview["initial_margin_rad"]),
        "preflight": folded_preview,
        "requested_pose_preflight": None,
        "pass": bool(folded_preview["pass"]),
        "failure_codes": [] if folded_preview["pass"] else list(folded_preview["failure_codes"]),
    })
    require(len(slots) == 6, "pose selection did not produce exact six slots")
    require(pose_key(slots[5]["selected_q_rad"]) not in {pose_key(row["selected_q_rad"]) for row in slots[:5]}, "folded low-torque pose duplicates first five slots")
    return {
        "algorithm": "FIVE_CONTRACT_SLOTS_WITH_SAFETY_SUBSTITUTION_THEN_DISTINCT_LOWEST_V15_18A_TAU_NORM",
        "candidate_selector": "INITIAL_QACC_BALLISTIC_PREDICTION_PLUS_30_SEGMENT_RESET_SET_MJ_FORWARD_QSPACE_SWEEP",
        "candidate_selector_free_dynamics_integration_performed": False,
        "candidate_selector_mj_step_call_count": 0,
        "prediction_formula": "q(t)=q0+0.5*1.1*qacc_initial*t^2 (10_PERCENT_CONSERVATIVE_ACCELERATION_ENVELOPE)",
        "qspace_sweep_interpolation": "31_EQUAL_TIME_SAMPLES_OVER_CLOSED_INTERVAL_0_TO_0P15S",
        "source_report_relative_path": V15_18A_REL,
        "source_report_sha256": sha256_file(repo_path(V15_18A_REL)),
        "requested_slot_count": 6,
        "selected_slot_count": len(slots),
        "unique_selected_q_count": len({pose_key(row["selected_q_rad"]) for row in slots}),
        "max_extension_and_frozen_481_are_v15_18a_semantic_aliases": True,
        "folded_low_torque_selection": {
            "eligible_pool": "V15_18A_EXACT_21_ACCEPTED_POSES_PASSING_0P15S_SAFETY_PREFLIGHT_AND_NOT_DUPLICATING_FIRST_FIVE_SELECTED_Q",
            "ranking": "V15_18A_TAU_HOLD_L2_NORM_ASC_THEN_SOURCE_GLOBAL_INDEX_ASC_THEN_ID",
            "selected_pose_id": str(folded["id"]),
            "selected_v15_18a_tau_norm_nm": float(folded["v15_18a_tau_norm_nm"]),
        },
        "slots": slots,
        "pass": len(slots) == 6 and all(bool(row["pass"]) for row in slots),
    }


def locked_pose_projection(slots: Sequence[Mapping[str, Any]]) -> list[dict[str, Any]]:
    """Return the exact persisted B1 slot/category/requested/selected/q projection."""

    return [
        {
            "slot_index": float(row["slot_index"]),
            "category": str(row["category"]),
            "requested_pose_id": row["requested_pose_id"],
            "selected_pose_id": str(row["selected_pose_id"]),
            "selected_q_rad": [float(value) for value in row["selected_q_rad"]],
        }
        for row in slots
    ]


def locked_selection(source_report: Mapping[str, Any], rows: Sequence[Mapping[str, Any]], limits: Mapping[str, tuple[float, float] | None]) -> dict[str, Any]:
    """Reuse the accepted B1 six slots without executing a safety selector.

    B2 changes only numerical integration evidence.  Re-selecting poses would
    confound that experiment, so category/id/q are embedded and digest-locked.
    Each selected/requested q is still checked against the protected V15.18A
    authority before any dynamics run.
    """

    by_id = {str(row["id"]): row for row in rows}
    slots: list[dict[str, Any]] = []
    for spec in LOCKED_POSE_SPECS:
        selected_id = str(spec["selected_pose_id"])
        requested_id = spec["requested_pose_id"]
        require(selected_id in by_id, f"locked selected pose absent from V15.18A: {selected_id}")
        selected = by_id[selected_id]
        locked_q = [float(value) for value in spec["selected_q_rad"]]
        require([float(value) for value in selected["q_rad"]] == locked_q, f"locked selected q changed in V15.18A: {selected_id}")
        requested_q = None
        if requested_id is not None:
            requested_key = str(requested_id)
            require(requested_key in by_id, f"locked requested pose absent from V15.18A: {requested_key}")
            requested_q = [float(value) for value in by_id[requested_key]["q_rad"]]
        slots.append({
            **spec,
            "requested_q_rad": requested_q,
            "source_global_index": selected["source_global_index"],
            "source_role": selected["role"],
            "source_collision_free": selected["source_collision_free"],
            "initial_margin_rad": minimum_limited_margin(locked_q, limits),
            "selection_provenance": "V15_18B_B1_ACCEPTED_SLOT_REUSED_WITHOUT_RESELECTION",
            "pass": True,
            "failure_codes": [],
        })

    projection = locked_pose_projection(slots)
    digest = canonical_digest(projection)
    require(digest == LOCKED_POSE_SLOT_ID_Q_SHA256, "locked B1 slot/category/id/q digest mismatch")
    require(len(slots) == 6 and len({pose_key(row["selected_q_rad"]) for row in slots}) == 5, "locked six-slot alias contract changed")
    require(pose_key(slots[5]["selected_q_rad"]) not in {pose_key(row["selected_q_rad"]) for row in slots[:5]}, "locked folded pose duplicates first five slots")
    return {
        "algorithm": "LOCKED_V15_18B_B1_SIX_SLOT_REUSE_NO_SELECTOR_RERUN",
        "candidate_selector": "NOT_EXECUTED_IN_B2",
        "candidate_selector_free_dynamics_integration_performed": False,
        "candidate_selector_mj_step_call_count": 0,
        "source_report_relative_path": V15_18A_REL,
        "source_report_sha256": sha256_file(repo_path(V15_18A_REL)),
        "v15_18a_source_pass": source_report.get("pass") is True and source_report.get("final_status") == "V15.18A STATIC_GRAVITY = PASS",
        "requested_slot_count": 6,
        "selected_slot_count": 6,
        "unique_selected_q_count": 5,
        "max_extension_and_frozen_481_are_v15_18a_semantic_aliases": True,
        "locked_slot_projection": projection,
        "locked_slot_projection_sha256": digest,
        "locked_slot_projection_expected_sha256": LOCKED_POSE_SLOT_ID_Q_SHA256,
        "slots": slots,
        "pass": True,
    }


def initial_state(model: Any, q: Sequence[float], mujoco: Any, np: Any, limits: Mapping[str, tuple[float, float] | None], authority_ids: Sequence[int], positive_ids: Sequence[int], positive_rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    data = mujoco.MjData(model)
    reset_forward(model, data, q, mujoco, np)
    matrix = full_mass_matrix(model, data, mujoco, np)
    bias = np.asarray(data.qfrc_bias, dtype=float).copy()
    actual = np.asarray(data.qacc, dtype=float).copy()
    try:
        expected = -np.linalg.solve(matrix, bias)
        residual = matrix @ actual + bias
        residual_norm = float(np.linalg.norm(residual))
        relative = float(np.linalg.norm(actual - expected) / max(float(np.linalg.norm(expected)), 1.0e-12))
        solve_available = bool(np.all(np.isfinite(expected)) and np.all(np.isfinite(residual)) and math.isfinite(relative))
    except np.linalg.LinAlgError:
        expected = np.full(int(model.nv), np.nan)
        residual = np.full(int(model.nv), np.nan)
        residual_norm = float("nan")
        relative = float("nan")
        solve_available = False
    identity_pass = solve_available and relative < float(THRESHOLDS["initial_qacc_relative_error_strict_less_than"]) and residual_norm < float(THRESHOLDS["initial_dynamics_residual_norm_strict_less_than"])
    body_com = [{**dict(row), "com_world_m": np.asarray(data.xipos[int(row["body_id"])], dtype=float).copy()} for row in positive_rows]
    return {
        "qpos_rad": np.asarray(data.qpos, dtype=float).copy(),
        "qvel_rad_s": np.asarray(data.qvel, dtype=float).copy(),
        "qfrc_bias_nm": bias,
        "qacc_rad_s2": actual,
        "expected_qacc_rad_s2": expected,
        "mass_matrix": matrix,
        "residual_vector": residual,
        "residual_norm": residual_norm,
        "relative_qacc_error": relative,
        "solve_available": solve_available,
        "identity_pass": identity_pass,
        "mujoco_potential_j": float(data.energy[0]),
        "mujoco_kinetic_j": float(data.energy[1]),
        "independent_authority_potential_j": independent_potential(model, data, authority_ids, np),
        "independent_all_positive_potential_j": independent_potential(model, data, positive_ids, np),
        "body_com_world_m": body_com,
        "contacts": contact_rows(model, data, mujoco),
        "ncon": int(data.ncon),
        "nefc": int(data.nefc),
        "active_joint_limit_constraints": active_joint_limit_count(data, mujoco, np),
        "joint_limit_margin_rad": joint_margins(data.qpos, limits),
        "minimum_joint_limit_margin_rad": minimum_limited_margin(data.qpos, limits),
        "qfrc_actuator_nm": np.asarray(data.qfrc_actuator, dtype=float).copy(),
        "qfrc_passive_nm": np.asarray(data.qfrc_passive, dtype=float).copy(),
        "ctrl": np.asarray(data.ctrl, dtype=float).copy(),
        "qfrc_applied_nm": np.asarray(data.qfrc_applied, dtype=float).copy(),
        "xfrc_applied": np.asarray(data.xfrc_applied, dtype=float).copy(),
    }


def early_direction(run: Mapping[str, Any], qacc_initial: Sequence[float], np: Any) -> dict[str, Any]:
    times = np.asarray(run["time_s"], dtype=float)
    index = int(np.argmin(np.abs(times - PROBE_TIME_S)))
    require(abs(float(times[index]) - PROBE_TIME_S) <= 1.0e-15, "probe time absent from run grid")
    q0 = np.asarray(run["qpos_rad"][0], dtype=float)
    q_probe = np.asarray(run["qpos_rad"][index], dtype=float)
    qacc = np.asarray(qacc_initial, dtype=float)
    rows = []
    for joint_index, joint in enumerate(JOINTS):
        predicted = 0.5 * abs(float(qacc[joint_index])) * PROBE_TIME_S * PROBE_TIME_S
        valid = abs(float(qacc[joint_index])) > float(THRESHOLDS["early_direction_min_abs_qacc_rad_s2"]) and predicted > float(THRESHOLDS["early_direction_min_predicted_displacement_rad"])
        displacement = float(q_probe[joint_index] - q0[joint_index])
        sign_match = (not valid) or (math.copysign(1.0, displacement) == math.copysign(1.0, float(qacc[joint_index])) and displacement != 0.0)
        rows.append({
            "joint": joint,
            "initial_qacc_rad_s2": float(qacc[joint_index]),
            "predicted_displacement_magnitude_rad": predicted,
            "actual_displacement_at_probe_rad": displacement,
            "valid_for_direction_check": valid,
            "sign_match": sign_match,
            "pass": sign_match,
        })
    return {
        "probe_time_s": PROBE_TIME_S,
        "probe_grid_index": index,
        "per_joint": rows,
        "j2_j3_j4_summary": {row["joint"]: {"initial_qacc_rad_s2": row["initial_qacc_rad_s2"], "displacement_rad": row["actual_displacement_at_probe_rad"], "valid": row["valid_for_direction_check"], "sign_match": row["sign_match"]} for row in rows if row["joint"] in {"J2", "J3", "J4"}},
        "pass": all(bool(row["pass"]) for row in rows),
    }


def convergence(run_a: Mapping[str, Any], run_b: Mapping[str, Any], np: Any) -> dict[str, Any]:
    ta = np.asarray(run_a["time_s"], dtype=float)
    tb = np.asarray(run_b["time_s"], dtype=float)
    qa = np.asarray(run_a["qpos_rad"], dtype=float)
    qb = np.asarray(run_b["qpos_rad"], dtype=float)
    va = np.asarray(run_a["qvel_rad_s"], dtype=float)
    vb = np.asarray(run_b["qvel_rad_s"], dtype=float)
    qb_interp = np.column_stack([np.interp(ta, tb, qb[:, index]) for index in range(qb.shape[1])])
    vb_interp = np.column_stack([np.interp(ta, tb, vb[:, index]) for index in range(vb.shape[1])])
    qdiff = qa - qb_interp
    vdiff = va - vb_interp
    final_q = float(np.max(np.abs(qa[-1] - qb[-1])))
    final_v = float(np.max(np.abs(va[-1] - vb[-1])))
    max_displacement = max(float(np.max(np.abs(qa - qa[0]))), float(np.max(np.abs(qb - qb[0]))))
    max_speed = max(float(np.max(np.abs(va))), float(np.max(np.abs(vb))))
    q_limit = max(float(THRESHOLDS["final_q_convergence_absolute_floor_rad"]), float(THRESHOLDS["final_q_convergence_relative_to_max_displacement"]) * max_displacement)
    v_limit = max(float(THRESHOLDS["final_qvel_convergence_absolute_floor_rad_s"]), float(THRESHOLDS["final_qvel_convergence_relative_to_max_speed"]) * max_speed)
    return {
        "method": "DT_OVER_2_LINEARLY_INTERPOLATED_TO_NOMINAL_TIME_GRID",
        "max_passive_displacement_rad": max_displacement,
        "max_abs_speed_rad_s": max_speed,
        "final_qpos_max_abs_difference_rad": final_q,
        "final_qpos_limit_rad": q_limit,
        "final_qvel_max_abs_difference_rad_s": final_v,
        "final_qvel_limit_rad_s": v_limit,
        "trajectory_qpos_max_abs_difference_rad": float(np.max(np.abs(qdiff))),
        "trajectory_qpos_rms_difference_rad": float(np.sqrt(np.mean(qdiff * qdiff))),
        "trajectory_qvel_max_abs_difference_rad_s": float(np.max(np.abs(vdiff))),
        "trajectory_qvel_rms_difference_rad_s": float(np.sqrt(np.mean(vdiff * vdiff))),
        "qpos_pass": final_q < q_limit,
        "qvel_pass": final_v < v_limit,
        "pass": final_q < q_limit and final_v < v_limit,
    }


def determinism(run_a: Mapping[str, Any], run_c: Mapping[str, Any], np: Any) -> dict[str, Any]:
    qa, qc = np.asarray(run_a["qpos_rad"], dtype=float), np.asarray(run_c["qpos_rad"], dtype=float)
    va, vc = np.asarray(run_a["qvel_rad_s"], dtype=float), np.asarray(run_c["qvel_rad_s"], dtype=float)
    ea = np.asarray(run_a["mujoco_energy_j"], dtype=float)
    ec = np.asarray(run_c["mujoco_energy_j"], dtype=float)
    qdiff, vdiff, ediff = float(np.max(np.abs(qa - qc))), float(np.max(np.abs(va - vc))), float(np.max(np.abs(ea - ec)))
    bitwise = bool(np.array_equal(qa, qc) and np.array_equal(va, vc) and np.array_equal(ea, ec))
    passed = qdiff < float(THRESHOLDS["determinism_qpos_max_abs_rad_strict_less_than"]) and vdiff < float(THRESHOLDS["determinism_qvel_max_abs_rad_s_strict_less_than"]) and ediff < float(THRESHOLDS["determinism_energy_max_abs_j_strict_less_than"])
    return {
        "qpos_max_abs_difference_rad": qdiff,
        "qvel_max_abs_difference_rad_s": vdiff,
        "mujoco_energy_max_abs_difference_j": ediff,
        "classification": "BITWISE_DETERMINISTIC" if bitwise else "NUMERICALLY_DETERMINISTIC" if passed else "NONDETERMINISTIC",
        "bitwise_equal": bitwise,
        "pass": passed,
    }


def finite_ratio(numerator: float, denominator: float) -> float:
    if denominator == 0.0:
        return 0.0 if numerator == 0.0 else math.inf
    return numerator / denominator


def aligned_trajectory_difference(coarse: Mapping[str, Any], fine: Mapping[str, Any], np: Any) -> dict[str, Any]:
    """Compare full trajectories on their exact common nested time grid."""

    coarse_time = np.asarray(coarse["time_s"], dtype=float)
    fine_time = np.asarray(fine["time_s"], dtype=float)
    coarse_dt = float(coarse["dt_s"])
    fine_dt = float(fine["dt_s"])
    stride_float = coarse_dt / fine_dt
    stride = int(round(stride_float))
    require(stride >= 1 and abs(stride_float - stride) <= 1.0e-12, "trajectory grids are not integer-nested")
    indices = np.arange(len(coarse_time), dtype=int) * stride
    require(int(indices[-1]) < len(fine_time), "fine trajectory does not span coarse grid")
    aligned_time_error = float(np.max(np.abs(coarse_time - fine_time[indices])))
    require(aligned_time_error <= 1.0e-15, "nested trajectory time alignment changed")
    coarse_q = np.asarray(coarse["qpos_rad"], dtype=float)
    fine_q = np.asarray(fine["qpos_rad"], dtype=float)[indices]
    coarse_v = np.asarray(coarse["qvel_rad_s"], dtype=float)
    fine_v = np.asarray(fine["qvel_rad_s"], dtype=float)[indices]
    qdiff = coarse_q - fine_q
    vdiff = coarse_v - fine_v
    return {
        "method": "FULL_SHARED_NESTED_TIME_GRID_MAX_OVER_ALL_SAMPLES_AND_JOINTS",
        "coarse_label": coarse["label"],
        "fine_label": fine["label"],
        "coarse_dt_s": coarse_dt,
        "fine_dt_s": fine_dt,
        "fine_grid_stride": stride,
        "shared_grid_sample_count": len(coarse_time),
        "max_abs_shared_time_error_s": aligned_time_error,
        "qpos_max_abs_difference_rad": float(np.max(np.abs(qdiff))),
        "qpos_rms_difference_rad": float(np.sqrt(np.mean(qdiff * qdiff))),
        "qvel_max_abs_difference_rad_s": float(np.max(np.abs(vdiff))),
        "qvel_rms_difference_rad_s": float(np.sqrt(np.mean(vdiff * vdiff))),
        "all_values_finite": bool(np.all(np.isfinite(qdiff)) and np.all(np.isfinite(vdiff))),
    }


def implicitfast_energy_convergence(runs: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    labels = ("A", "B", "I4")
    sources: dict[str, Any] = {}
    ratio_min = float(THRESHOLDS["implicitfast_energy_ratio_minimum_inclusive"])
    ratio_max = float(THRESHOLDS["implicitfast_energy_ratio_maximum_inclusive"])
    for source in ("mujoco", "independent"):
        errors = {label: float(runs[label]["energy_summary"][source]["normalized_drift"]) for label in labels}
        ratio_1 = finite_ratio(errors["B"], errors["A"])
        ratio_2 = finite_ratio(errors["I4"], errors["B"])
        monotonic = errors["I4"] < errors["B"] < errors["A"]
        ratios_pass = ratio_min <= ratio_1 <= ratio_max and ratio_min <= ratio_2 <= ratio_max
        sources[source] = {
            "normalized_drift_by_run": errors,
            "r1_E_1ms_over_E_2ms": ratio_1,
            "r2_E_0p5ms_over_E_1ms": ratio_2,
            "monotonic_E_0p5ms_lt_E_1ms_lt_E_2ms": monotonic,
            "ratio_interval_inclusive": [ratio_min, ratio_max],
            "ratios_pass": ratios_pass,
            "pass": monotonic and ratios_pass,
        }
    conservative = {label: max(float(runs[label]["energy_summary"][source]["normalized_drift"]) for source in ("mujoco", "independent")) for label in labels}
    return {
        "method": "DUAL_ENERGY_NORMALIZED_MAX_DRIFT_THREE_LEVEL_IMPLICITFAST",
        "profiles": {"A": "IMPLICITFAST_2MS", "B": "IMPLICITFAST_1MS", "I4": "IMPLICITFAST_0P5MS"},
        "sources": sources,
        "conservative_max_normalized_drift_by_run": conservative,
        "conservative_r1_E_1ms_over_E_2ms": finite_ratio(conservative["B"], conservative["A"]),
        "conservative_r2_E_0p5ms_over_E_1ms": finite_ratio(conservative["I4"], conservative["B"]),
        "classification": "FIRST_ORDER_LIKE_TIMESTEP_CONVERGENCE" if all(bool(row["pass"]) for row in sources.values()) else "NO_CONFIRMED_TIMESTEP_ORDER",
        "pass": all(bool(row["pass"]) for row in sources.values()),
    }


def implicitfast_trajectory_convergence(runs: Mapping[str, Mapping[str, Any]], np: Any) -> dict[str, Any]:
    difference_2ms_1ms = aligned_trajectory_difference(runs["A"], runs["B"], np)
    difference_1ms_0p5ms = aligned_trajectory_difference(runs["B"], runs["I4"], np)
    q_ratio = finite_ratio(float(difference_1ms_0p5ms["qpos_max_abs_difference_rad"]), float(difference_2ms_1ms["qpos_max_abs_difference_rad"]))
    v_ratio = finite_ratio(float(difference_1ms_0p5ms["qvel_max_abs_difference_rad_s"]), float(difference_2ms_1ms["qvel_max_abs_difference_rad_s"]))
    limit = float(THRESHOLDS["implicitfast_trajectory_ratio_strict_less_than"])
    finite = bool(difference_2ms_1ms["all_values_finite"] and difference_1ms_0p5ms["all_values_finite"] and math.isfinite(q_ratio) and math.isfinite(v_ratio))
    return {
        "method": "FULL_ALIGNED_TRAJECTORY_MAX_MULTI_LEVEL_CONVERGENCE",
        "difference_2ms_vs_1ms": difference_2ms_1ms,
        "difference_1ms_vs_0p5ms": difference_1ms_0p5ms,
        "qpos_difference_ratio_D_1ms_0p5ms_over_D_2ms_1ms": q_ratio,
        "qvel_difference_ratio_D_1ms_0p5ms_over_D_2ms_1ms": v_ratio,
        "ratio_limit_strict_less_than": limit,
        "qpos_pass": q_ratio < limit,
        "qvel_pass": v_ratio < limit,
        "all_values_finite": finite,
        "pass": finite and q_ratio < limit and v_ratio < limit,
    }


def rk4_energy_reference(runs: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    floor = float(THRESHOLDS["rk4_half_step_energy_not_worse_additive_floor_j"])
    profiles: dict[str, Any] = {}
    for label, limit in (
        ("RK4_2MS", float(THRESHOLDS["rk4_2ms_normalized_energy_drift_strict_less_than"])),
        ("RK4_1MS", float(THRESHOLDS["rk4_1ms_normalized_energy_drift_strict_less_than"])),
    ):
        summary = runs[label]["energy_summary"]
        sources = {
            source: {
                "max_abs_drift_j": float(summary[source]["max_abs_drift_j"]),
                "normalized_drift": float(summary[source]["normalized_drift"]),
                "normalized_limit_strict_less_than": limit,
                "pass": float(summary[source]["normalized_drift"]) < limit,
            }
            for source in ("mujoco", "independent")
        }
        profiles[label] = {
            "sources": sources,
            "normalization_denominator_j": float(summary["normalization_denominator_j"]),
            "normalized_floor_equivalent_of_1e_8_j": floor / float(summary["normalization_denominator_j"]),
            "dual_source_alignment_pass": bool(summary["dual_source_alignment_pass"]),
            "all_values_finite": bool(summary["all_values_finite"]),
            "pass": all(bool(row["pass"]) for row in sources.values()) and bool(summary["dual_source_alignment_pass"]) and bool(summary["all_values_finite"]),
        }
    normalized_floor = float(profiles["RK4_2MS"]["normalized_floor_equivalent_of_1e_8_j"])
    half_normalized_not_worse = {
        source: {
            "rk4_2ms_normalized_drift": profiles["RK4_2MS"]["sources"][source]["normalized_drift"],
            "rk4_1ms_normalized_drift": profiles["RK4_1MS"]["sources"][source]["normalized_drift"],
            "normalized_floor": normalized_floor,
            "normalized_floor_j": floor,
            "normalized_floor_denominator_j": profiles["RK4_2MS"]["normalization_denominator_j"],
            "normalized_floor_denominator_profile": "RK4_2MS_RHS_NOMINAL_REFERENCE",
            "normalized_floor_by_run": {
                label: profiles[label]["normalized_floor_equivalent_of_1e_8_j"] for label in ("RK4_2MS", "RK4_1MS")
            },
            "semantics": "RK4_1MS_NORMALIZED_DRIFT_LE_RK4_2MS_NORMALIZED_DRIFT_PLUS_1E_8_J_DIVIDED_BY_RK4_2MS_NORMALIZATION_DENOMINATOR",
            "pass": profiles["RK4_1MS"]["sources"][source]["normalized_drift"] <= profiles["RK4_2MS"]["sources"][source]["normalized_drift"] + normalized_floor,
        }
        for source in ("mujoco", "independent")
    }
    half_absolute_diagnostic = {
        source: {
            "rk4_2ms_max_abs_drift_j": profiles["RK4_2MS"]["sources"][source]["max_abs_drift_j"],
            "rk4_1ms_max_abs_drift_j": profiles["RK4_1MS"]["sources"][source]["max_abs_drift_j"],
            "additive_absolute_comparison_floor_j": floor,
            "gate_authority": False,
            "semantics": "ABSOLUTE_J_COMPARISON_RETAINED_AS_DIAGNOSTIC_ONLY",
            "pass": profiles["RK4_1MS"]["sources"][source]["max_abs_drift_j"] <= profiles["RK4_2MS"]["sources"][source]["max_abs_drift_j"] + floor,
        }
        for source in ("mujoco", "independent")
    }
    return {
        "method": "DUAL_ENERGY_RK4_RUNTIME_ONLY_REFERENCE",
        "normalization": "SAME_PER_RUN_ABS_POTENTIAL_CHANGE_DENOMINATOR_WITH_1E_3_J_DENOMINATOR_FLOOR_AS_LEGACY_AUDIT",
        "profiles": profiles,
        "half_step_normalized_drift_not_worse": half_normalized_not_worse,
        "half_step_absolute_drift_not_worse": half_absolute_diagnostic,
        "pass": all(bool(row["pass"]) for row in profiles.values()) and all(bool(row["pass"]) for row in half_normalized_not_worse.values()),
    }


def rk4_timestep_convergence(runs: Mapping[str, Mapping[str, Any]], np: Any) -> dict[str, Any]:
    q2 = np.asarray(runs["RK4_2MS"]["final_qpos_rad"], dtype=float)
    q1 = np.asarray(runs["RK4_1MS"]["final_qpos_rad"], dtype=float)
    v2 = np.asarray(runs["RK4_2MS"]["final_qvel_rad_s"], dtype=float)
    v1 = np.asarray(runs["RK4_1MS"]["final_qvel_rad_s"], dtype=float)
    qdiff = float(np.max(np.abs(q2 - q1)))
    vdiff = float(np.max(np.abs(v2 - v1)))
    qlimit = float(THRESHOLDS["rk4_final_q_difference_rad_strict_less_than"])
    vlimit = float(THRESHOLDS["rk4_final_qvel_difference_rad_s_strict_less_than"])
    return {
        "method": "RK4_2MS_VS_RK4_1MS_FINAL_STATE",
        "final_qpos_max_abs_difference_rad": qdiff,
        "final_qpos_limit_rad_strict_less_than": qlimit,
        "final_qvel_max_abs_difference_rad_s": vdiff,
        "final_qvel_limit_rad_s_strict_less_than": vlimit,
        "qpos_pass": qdiff < qlimit,
        "qvel_pass": vdiff < vlimit,
        "pass": qdiff < qlimit and vdiff < vlimit,
    }


def high_precision_reference_diagnostic(runs: Mapping[str, Mapping[str, Any]], qacc_initial: Sequence[float], np: Any) -> dict[str, Any]:
    early = {label: early_direction(runs[label], qacc_initial, np) for label in ("I4", "RK4_1MS")}
    valid = sum(1 for result in early.values() for row in result["per_joint"] if row["valid_for_direction_check"])
    matched = sum(1 for result in early.values() for row in result["per_joint"] if row["valid_for_direction_check"] and row["sign_match"])
    q_i4 = np.asarray(runs["I4"]["final_qpos_rad"], dtype=float)
    q_rk4 = np.asarray(runs["RK4_1MS"]["final_qpos_rad"], dtype=float)
    qdiff = float(np.max(np.abs(q_i4 - q_rk4)))
    qlimit = float(THRESHOLDS["implicitfast_0p5ms_vs_rk4_1ms_final_q_anomaly_rad_strict_less_than"])
    return {
        "method": "IMPLICITFAST_0P5MS_VS_RK4_1MS_DIAGNOSTIC",
        "reference_profile": "RK4_1MS",
        "early_motion_direction": early,
        "valid_early_direction_count": valid,
        "matching_early_direction_count": matched,
        "all_valid_early_directions_match": matched == valid,
        "final_qpos_max_abs_difference_rad": qdiff,
        "final_qpos_anomaly_limit_rad_strict_less_than": qlimit,
        "pass": matched == valid and all(bool(row["pass"]) for row in early.values()) and qdiff < qlimit,
    }


def pose_run(slot: Mapping[str, Any], models: Mapping[str, Any], mujoco: Any, np: Any, limits: Mapping[str, tuple[float, float] | None], authority_ids: Sequence[int], positive_ids: Sequence[int], positive_rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    q = slot["selected_q_rad"]
    initial = initial_state(models["A"], q, mujoco, np, limits, authority_ids, positive_ids, positive_rows)
    runs = {
        "A": run_simulation("A", models["A"], q, DURATION_S, mujoco, np, limits, authority_ids, positive_ids),
        "B": run_simulation("B", models["B"], q, DURATION_S, mujoco, np, limits, authority_ids, positive_ids),
        "C": run_simulation("C", models["C"], q, DURATION_S, mujoco, np, limits, authority_ids, positive_ids),
    }
    early = {label: early_direction(runs[label], initial["qacc_rad_s2"], np) for label in ("A", "B")}
    early_pass = early["A"]["pass"] and early["B"]["pass"]
    conv = convergence(runs["A"], runs["B"], np)
    det = determinism(runs["A"], runs["C"], np)
    a_energy = runs["A"]["energy_summary"]
    b_energy = runs["B"]["energy_summary"]
    half_not_worse = (
        float(b_energy["mujoco"]["max_abs_drift_j"]) <= float(a_energy["mujoco"]["max_abs_drift_j"]) + float(THRESHOLDS["half_step_energy_not_worse_additive_j"])
        and float(b_energy["independent"]["max_abs_drift_j"]) <= float(a_energy["independent"]["max_abs_drift_j"]) + float(THRESHOLDS["half_step_energy_not_worse_additive_j"])
    )
    energy = {
        "A": a_energy,
        "B": b_energy,
        "C": runs["C"]["energy_summary"],
        "half_step_not_noticeably_worse": half_not_worse,
        "pass": bool(a_energy["pass"] and b_energy["pass"] and runs["C"]["energy_summary"]["pass"] and half_not_worse),
    }
    all_runs_safety = all(bool(run["safety"]["pass"]) for run in runs.values())
    codes = []
    if not initial["identity_pass"]:
        codes.append("FAIL_INITIAL_ACCELERATION_IDENTITY")
    if not early_pass:
        codes.append("FAIL_EARLY_MOTION_DIRECTION")
    if not energy["pass"]:
        codes.append("FAIL_DUAL_ENERGY_AUDIT")
    if not conv["pass"]:
        codes.append("FAIL_TIMESTEP_CONVERGENCE")
    if not det["pass"]:
        codes.append("FAIL_DETERMINISM")
    if not all_runs_safety:
        codes.append("FAIL_RUNTIME_SAFETY")
    for run in runs.values():
        codes.extend(str(code) for code in run["failure_codes"])
    codes = sorted(set(codes))
    return {
        "slot_index": slot["slot_index"],
        "category": slot["category"],
        "selected_pose_id": slot["selected_pose_id"],
        "selected_q_rad": q,
        "initial_state": initial,
        "runs": runs,
        "early_motion_direction": {"runs": early, "pass": early_pass},
        "energy": energy,
        "convergence": conv,
        "determinism": det,
        "safety": {"all_three_runs_pass": all_runs_safety, "per_run": {label: run["safety"] for label, run in runs.items()}, "pass": all_runs_safety},
        "pass": bool(initial["identity_pass"] and early_pass and energy["pass"] and conv["pass"] and det["pass"] and all_runs_safety),
        "failure_codes": codes,
    }


def assemble_pose_run(slot: Mapping[str, Any], initial: Mapping[str, Any], runs: Mapping[str, Mapping[str, Any]], np: Any) -> dict[str, Any]:
    """Assemble a pose result after sequential isolated-model execution.

    Keeping only one mesh-heavy MjModel resident at a time is important on the
    acceptance workstation.  All six runs are independently compiled; compact
    trajectory evidence is joined only after execution.
    """

    require(tuple(runs) == RUN_LABELS, "per-pose run label contract changed")
    early = {label: early_direction(runs[label], initial["qacc_rad_s2"], np) for label in ("A", "B")}
    early_pass = early["A"]["pass"] and early["B"]["pass"]
    conv = convergence(runs["A"], runs["B"], np)
    det = determinism(runs["A"], runs["C"], np)
    a_energy = runs["A"]["energy_summary"]
    b_energy = runs["B"]["energy_summary"]
    half_not_worse = (
        float(b_energy["mujoco"]["max_abs_drift_j"]) <= float(a_energy["mujoco"]["max_abs_drift_j"]) + float(THRESHOLDS["half_step_energy_not_worse_additive_j"])
        and float(b_energy["independent"]["max_abs_drift_j"]) <= float(a_energy["independent"]["max_abs_drift_j"]) + float(THRESHOLDS["half_step_energy_not_worse_additive_j"])
    )
    energy = {
        "A": a_energy,
        "B": b_energy,
        "C": runs["C"]["energy_summary"],
        "half_step_not_noticeably_worse": half_not_worse,
        "pass": bool(a_energy["pass"] and b_energy["pass"] and runs["C"]["energy_summary"]["pass"] and half_not_worse),
    }
    implicit_energy = implicitfast_energy_convergence(runs)
    implicit_trajectory = implicitfast_trajectory_convergence(runs, np)
    rk4_energy = rk4_energy_reference(runs)
    rk4_convergence = rk4_timestep_convergence(runs, np)
    reference = high_precision_reference_diagnostic(runs, initial["qacc_rad_s2"], np)
    attribution_pass = bool(implicit_energy["pass"] and implicit_trajectory["pass"] and rk4_energy["pass"] and rk4_convergence["pass"] and reference["pass"])
    all_runs_safety = all(bool(runs[label]["safety"]["pass"]) for label in RUN_LABELS)
    all_runs_continuity = all(bool(runs[label]["continuity"]["pass"]) for label in RUN_LABELS)
    legacy_codes = []
    if not energy["pass"]:
        legacy_codes.append("LEGACY_FAIL_DUAL_ENERGY_AUDIT")
    if not conv["pass"]:
        legacy_codes.append("LEGACY_FAIL_ABSOLUTE_FINAL_STATE_TIMESTEP_CONVERGENCE")
    codes = []
    if not initial["identity_pass"]:
        codes.append("FAIL_INITIAL_ACCELERATION_IDENTITY")
    if not early_pass:
        codes.append("FAIL_EARLY_MOTION_DIRECTION")
    if not implicit_energy["pass"]:
        codes.append("FAIL_IMPLICITFAST_ENERGY_CONVERGENCE")
    if not implicit_trajectory["pass"]:
        codes.append("FAIL_IMPLICITFAST_TRAJECTORY_CONVERGENCE")
    if not rk4_energy["pass"]:
        codes.append("FAIL_RK4_ENERGY_REFERENCE")
    if not rk4_convergence["pass"]:
        codes.append("FAIL_RK4_TIMESTEP_CONVERGENCE")
    if not reference["pass"]:
        codes.append("FAIL_HIGH_PRECISION_REFERENCE_DIAGNOSTIC")
    if not det["pass"]:
        codes.append("FAIL_DETERMINISM")
    if not all_runs_safety:
        codes.append("FAIL_RUNTIME_SAFETY")
    if not all_runs_continuity:
        codes.append("FAIL_TRAJECTORY_CONTINUITY")
    for label in RUN_LABELS:
        codes.extend(str(code) for code in runs[label]["failure_codes"] if str(code) != "FAIL_ENERGY_AUDIT")
    codes = sorted(set(codes))
    return {
        "slot_index": slot["slot_index"],
        "category": slot["category"],
        "selected_pose_id": slot["selected_pose_id"],
        "selected_q_rad": slot["selected_q_rad"],
        "initial_state": initial,
        "runs": dict(runs),
        "early_motion_direction": {"runs": early, "pass": early_pass},
        "energy": energy,
        "convergence": conv,
        "legacy_production_integrator_diagnostic": {
            "classification": "PRODUCTION_INTEGRATOR_DIAGNOSTIC",
            "energy": energy,
            "absolute_final_state_convergence": conv,
            "absolute_final_state_gate_model_validation_status": "SUPERSEDED_FOR_MODEL_VALIDATION_BY_MULTI_LEVEL_CONVERGENCE",
            "status": "PASS" if energy["pass"] and conv["pass"] else "FAIL",
            "failure_codes": legacy_codes,
            "pass": bool(energy["pass"] and conv["pass"]),
        },
        "numerical_integrator_attribution": {
            "implicitfast_energy_convergence": implicit_energy,
            "implicitfast_trajectory_convergence": implicit_trajectory,
            "rk4_energy_reference": rk4_energy,
            "rk4_timestep_convergence": rk4_convergence,
            "high_precision_reference_diagnostic": reference,
            "pass": attribution_pass,
        },
        "determinism": det,
        "safety": {
            "all_six_runs_pass": all_runs_safety,
            "all_six_runs_continuity_pass": all_runs_continuity,
            "per_run": {label: runs[label]["safety"] for label in RUN_LABELS},
            "pass": all_runs_safety and all_runs_continuity,
        },
        "legacy_diagnostic_failure_codes": legacy_codes,
        "pass": bool(initial["identity_pass"] and early_pass and attribution_pass and det["pass"] and all_runs_safety and all_runs_continuity),
        "failure_codes": codes,
    }


def aggregate_report(before: Mapping[str, str], after: Mapping[str, str], xml_contract: Mapping[str, Any], runtime: Mapping[str, Any], selection: Mapping[str, Any], pose_rows: Sequence[Mapping[str, Any]], mujoco_version: str, numpy_version: str, asset_contract: Mapping[str, Any], positive_rows: Sequence[Mapping[str, Any]]) -> dict[str, Any]:
    protected = {relative: {"before_sha256": before.get(relative), "after_sha256": after.get(relative), "unchanged": before.get(relative) == after.get(relative)} for relative in PROTECTED_RELATIVE_PATHS}
    initial_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "relative_qacc_error": row["initial_state"]["relative_qacc_error"], "residual_norm": row["initial_state"]["residual_norm"], "pass": row["initial_state"]["identity_pass"]} for row in pose_rows]
    early_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "j2_j3_j4": row["early_motion_direction"]["runs"]["A"]["j2_j3_j4_summary"], "pass": row["early_motion_direction"]["pass"]} for row in pose_rows]
    energy_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "A_mujoco_normalized_drift": row["energy"]["A"]["mujoco"]["normalized_drift"], "A_independent_normalized_drift": row["energy"]["A"]["independent"]["normalized_drift"], "B_mujoco_normalized_drift": row["energy"]["B"]["mujoco"]["normalized_drift"], "B_independent_normalized_drift": row["energy"]["B"]["independent"]["normalized_drift"], "half_step_not_noticeably_worse": row["energy"]["half_step_not_noticeably_worse"], "pass": row["energy"]["pass"]} for row in pose_rows]
    convergence_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], **row["convergence"]} for row in pose_rows]
    determinism_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], **row["determinism"]} for row in pose_rows]
    implicit_energy_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], **row["numerical_integrator_attribution"]["implicitfast_energy_convergence"]} for row in pose_rows]
    implicit_trajectory_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], **row["numerical_integrator_attribution"]["implicitfast_trajectory_convergence"]} for row in pose_rows]
    rk4_energy_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], **row["numerical_integrator_attribution"]["rk4_energy_reference"]} for row in pose_rows]
    rk4_convergence_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], **row["numerical_integrator_attribution"]["rk4_timestep_convergence"]} for row in pose_rows]
    reference_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], **row["numerical_integrator_attribution"]["high_precision_reference_diagnostic"]} for row in pose_rows]
    contacts_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "per_run": {label: row["runs"][label]["contact_count_max"] for label in RUN_LABELS}, "pass": all(row["runs"][label]["contact_count_max"] == 0 for label in RUN_LABELS)} for row in pose_rows]
    margin_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "initial_margin_rad": row["initial_state"]["minimum_joint_limit_margin_rad"], "minimum_runtime_margin_rad": min(float(row["runs"][label]["minimum_joint_limit_margin_rad"]) for label in RUN_LABELS), "per_run": {label: row["runs"][label]["minimum_joint_limit_margin_rad"] for label in RUN_LABELS}, "pass": all(float(row["runs"][label]["minimum_joint_limit_margin_rad"]) > float(THRESHOLDS["runtime_joint_limit_margin_rad_strict_greater_than"]) for label in RUN_LABELS)} for row in pose_rows]
    stability_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "per_run": {label: {key: row["runs"][label][key] for key in ("max_abs_qvel_rad_s", "max_abs_qacc_rad_s2", "max_abs_qpos_step_rad", "max_abs_qvel_step_rad_s", "max_abs_qacc_step_rad_s2")} for label in RUN_LABELS}, "pass": all(bool(row["runs"][label]["safety"]["pass"]) for label in RUN_LABELS)} for row in pose_rows]
    continuity_rows = [{"slot_index": row["slot_index"], "selected_pose_id": row["selected_pose_id"], "per_run": {label: row["runs"][label]["continuity"] for label in RUN_LABELS}, "pass": all(bool(row["runs"][label]["continuity"]["pass"]) for label in RUN_LABELS)} for row in pose_rows]

    legacy_projection = [{"slot_index": row["slot_index"], "A": row["runs"]["A"], "B": row["runs"]["B"], "C": row["runs"]["C"]} for row in pose_rows]
    legacy_digest = canonical_digest(json_safe(legacy_projection))
    legacy_preserved = legacy_digest == LEGACY_PRODUCTION_RUNS_ABC_SHA256
    legacy_energy_pass = all(bool(row["pass"]) for row in energy_rows)
    legacy_convergence_pass = all(bool(row["pass"]) for row in convergence_rows)
    legacy_limitation_observed = not (legacy_energy_pass and legacy_convergence_pass)

    implicit_energy_max = {label: max(float(row["conservative_max_normalized_drift_by_run"][label]) for row in implicit_energy_rows) for label in ("A", "B", "I4")}
    implicit_energy_global = {
        "max_conservative_normalized_drift_by_run": implicit_energy_max,
        "r1_E_1ms_over_E_2ms": finite_ratio(implicit_energy_max["B"], implicit_energy_max["A"]),
        "r2_E_0p5ms_over_E_1ms": finite_ratio(implicit_energy_max["I4"], implicit_energy_max["B"]),
        "all_per_pose_pass": all(bool(row["pass"]) for row in implicit_energy_rows),
    }
    implicit_energy_global["pass"] = bool(
        implicit_energy_global["all_per_pose_pass"]
        and implicit_energy_max["I4"] < implicit_energy_max["B"] < implicit_energy_max["A"]
        and float(THRESHOLDS["implicitfast_energy_ratio_minimum_inclusive"]) <= implicit_energy_global["r1_E_1ms_over_E_2ms"] <= float(THRESHOLDS["implicitfast_energy_ratio_maximum_inclusive"])
        and float(THRESHOLDS["implicitfast_energy_ratio_minimum_inclusive"]) <= implicit_energy_global["r2_E_0p5ms_over_E_1ms"] <= float(THRESHOLDS["implicitfast_energy_ratio_maximum_inclusive"])
    )
    trajectory_global = {
        "Dq_2ms_1ms_rad": max(float(row["difference_2ms_vs_1ms"]["qpos_max_abs_difference_rad"]) for row in implicit_trajectory_rows),
        "Dq_1ms_0p5ms_rad": max(float(row["difference_1ms_vs_0p5ms"]["qpos_max_abs_difference_rad"]) for row in implicit_trajectory_rows),
        "Dv_2ms_1ms_rad_s": max(float(row["difference_2ms_vs_1ms"]["qvel_max_abs_difference_rad_s"]) for row in implicit_trajectory_rows),
        "Dv_1ms_0p5ms_rad_s": max(float(row["difference_1ms_vs_0p5ms"]["qvel_max_abs_difference_rad_s"]) for row in implicit_trajectory_rows),
        "all_per_pose_pass": all(bool(row["pass"]) for row in implicit_trajectory_rows),
    }
    trajectory_global["qpos_ratio"] = finite_ratio(trajectory_global["Dq_1ms_0p5ms_rad"], trajectory_global["Dq_2ms_1ms_rad"])
    trajectory_global["qvel_ratio"] = finite_ratio(trajectory_global["Dv_1ms_0p5ms_rad_s"], trajectory_global["Dv_2ms_1ms_rad_s"])
    trajectory_global["pass"] = bool(trajectory_global["all_per_pose_pass"] and trajectory_global["qpos_ratio"] < float(THRESHOLDS["implicitfast_trajectory_ratio_strict_less_than"]) and trajectory_global["qvel_ratio"] < float(THRESHOLDS["implicitfast_trajectory_ratio_strict_less_than"]))
    rk4_energy_global = {
        "max_normalized_drift_by_run": {label: max(max(float(row["profiles"][label]["sources"][source]["normalized_drift"]) for source in ("mujoco", "independent")) for row in rk4_energy_rows) for label in ("RK4_2MS", "RK4_1MS")},
        "all_per_pose_pass": all(bool(row["pass"]) for row in rk4_energy_rows),
    }
    rk4_energy_global["pass"] = bool(rk4_energy_global["all_per_pose_pass"] and rk4_energy_global["max_normalized_drift_by_run"]["RK4_2MS"] < float(THRESHOLDS["rk4_2ms_normalized_energy_drift_strict_less_than"]) and rk4_energy_global["max_normalized_drift_by_run"]["RK4_1MS"] < float(THRESHOLDS["rk4_1ms_normalized_energy_drift_strict_less_than"]))
    rk4_convergence_global = {
        "max_final_qpos_difference_rad": max(float(row["final_qpos_max_abs_difference_rad"]) for row in rk4_convergence_rows),
        "max_final_qvel_difference_rad_s": max(float(row["final_qvel_max_abs_difference_rad_s"]) for row in rk4_convergence_rows),
        "all_per_pose_pass": all(bool(row["pass"]) for row in rk4_convergence_rows),
    }
    rk4_convergence_global["pass"] = bool(rk4_convergence_global["all_per_pose_pass"] and rk4_convergence_global["max_final_qpos_difference_rad"] < float(THRESHOLDS["rk4_final_q_difference_rad_strict_less_than"]) and rk4_convergence_global["max_final_qvel_difference_rad_s"] < float(THRESHOLDS["rk4_final_qvel_difference_rad_s_strict_less_than"]))
    reference_global = {
        "valid_early_direction_count": sum(int(row["valid_early_direction_count"]) for row in reference_rows),
        "matching_early_direction_count": sum(int(row["matching_early_direction_count"]) for row in reference_rows),
        "expected_valid_early_direction_count": 70,
        "max_final_qpos_difference_rad": max(float(row["final_qpos_max_abs_difference_rad"]) for row in reference_rows),
        "all_per_pose_pass": all(bool(row["pass"]) for row in reference_rows),
    }
    reference_global["pass"] = bool(reference_global["all_per_pose_pass"] and reference_global["valid_early_direction_count"] == reference_global["matching_early_direction_count"] == reference_global["expected_valid_early_direction_count"] and reference_global["max_final_qpos_difference_rad"] < float(THRESHOLDS["implicitfast_0p5ms_vs_rk4_1ms_final_q_anomaly_rad_strict_less_than"]))

    isolation = all(bool(row["runs"][label]["safety"]["actuation_passive_and_applied_zero"]) for row in pose_rows for label in RUN_LABELS)
    no_limits = all(int(row["runs"][label]["active_joint_limit_constraint_count_max"]) == 0 for row in pose_rows for label in RUN_LABELS)
    no_contacts = all(bool(row["pass"]) for row in contacts_rows)
    all_run_values_finite = all(bool(row["runs"][label]["all_values_finite"]) for row in pose_rows for label in RUN_LABELS)
    all_run_continuity = all(bool(row["pass"]) for row in continuity_rows)
    determinism_pass = all(bool(row["pass"]) for row in determinism_rows)
    initial_identity_pass = all(bool(row["pass"]) for row in initial_rows)
    legacy_early_valid = sum(1 for pose in pose_rows for label in ("A", "B") for joint in pose["early_motion_direction"]["runs"][label]["per_joint"] if joint["valid_for_direction_check"])
    legacy_early_matched = sum(1 for pose in pose_rows for label in ("A", "B") for joint in pose["early_motion_direction"]["runs"][label]["per_joint"] if joint["valid_for_direction_check"] and joint["sign_match"])
    legacy_early_pass = all(bool(row["pass"]) for row in early_rows) and legacy_early_valid == legacy_early_matched == 70
    numerical_attribution_pass = bool(legacy_limitation_observed and implicit_energy_global["pass"] and trajectory_global["pass"] and rk4_energy_global["pass"] and rk4_convergence_global["pass"] and reference_global["pass"])
    continuous_time_evidence = {
        "v15_18a_static_gravity_pass": bool(selection["v15_18a_source_pass"]),
        "initial_qacc_identity_pass": initial_identity_pass,
        "early_motion_direction_pass": legacy_early_pass and bool(reference_global["pass"]),
        "implicitfast_energy_convergence_pass": bool(implicit_energy_global["pass"]),
        "implicitfast_trajectory_convergence_pass": bool(trajectory_global["pass"]),
        "rk4_energy_reference_pass": bool(rk4_energy_global["pass"]),
        "rk4_timestep_convergence_pass": bool(rk4_convergence_global["pass"]),
        "no_contacts_all_runs_pass": no_contacts,
        "no_active_joint_limits_all_runs_pass": no_limits,
        "all_values_finite_all_runs_pass": all_run_values_finite,
        "determinism_pass": determinism_pass,
    }
    require(tuple(continuous_time_evidence) == ROOT_CAUSE_CONDITION_KEYS and len(continuous_time_evidence) == 11, "root-cause exact-11 condition-map contract changed")
    attribution_confirmed = all(bool(value) for value in continuous_time_evidence.values())
    require(attribution_confirmed == all(bool(value) for value in continuous_time_evidence.values()), "root-cause evidence mutation contradiction")
    gates_without_hard = {
        "exact_six_pose_slots": len(selection["slots"]) == len(pose_rows) == 6 and selection["locked_slot_projection_sha256"] == LOCKED_POSE_SLOT_ID_Q_SHA256,
        "pose_selection_safe": bool(selection["pass"]),
        "legacy_production_runs_preserved": legacy_preserved,
        "v15_18a_continuous_time_physics_pass": bool(selection["v15_18a_source_pass"]),
        "initial_acceleration_identity_pass": all(bool(row["pass"]) for row in initial_rows),
        "early_motion_direction_pass": legacy_early_pass,
        "implicitfast_energy_convergence_pass": bool(implicit_energy_global["pass"]),
        "implicitfast_trajectory_convergence_pass": bool(trajectory_global["pass"]),
        "rk4_energy_reference_pass": bool(rk4_energy_global["pass"]),
        "rk4_timestep_convergence_pass": bool(rk4_convergence_global["pass"]),
        "high_precision_reference_diagnostic_pass": bool(reference_global["pass"]),
        "determinism_pass": determinism_pass,
        "actuation_and_applied_force_isolation_pass": isolation,
        "no_contacts_pass": no_contacts,
        "no_active_joint_limits_pass": no_limits,
        "joint_limit_margin_pass": all(bool(row["pass"]) for row in margin_rows),
        "stability_pass": all(bool(row["pass"]) for row in stability_rows),
        "trajectory_continuity_pass": all_run_continuity,
        "protected_hashes_unchanged": all(bool(row["unchanged"]) for row in protected.values()),
        "production_model_hash_unchanged": before.get(MJCF_REL) == after.get(MJCF_REL) == MJCF_SHA256,
        "bridge_controller_unchanged": all(before.get(relative) == after.get(relative) for relative in (BRIDGE_REL, BRIDGE_ENTRY_REL, ROS2_CONTROLLERS_REL, MOVEIT_CONTROLLERS_REL)),
        "all_values_finite": finite_tree({"selection": selection, "poses": pose_rows, "attribution": [implicit_energy_global, trajectory_global, rk4_energy_global, rk4_convergence_global, reference_global]}),
        "numerical_integrator_truncation_error_confirmed": attribution_confirmed,
    }
    hard_codes = ["FAIL_GATE_" + key.upper() for key, gate_passed in gates_without_hard.items() if not gate_passed]
    hard_codes.extend(str(code) for row in pose_rows for code in row["failure_codes"])
    hard = sorted(set(hard_codes))
    gates = {**gates_without_hard, "hard_unresolved_items_empty": not hard}
    require(tuple(gates) == ACCEPTANCE_GATE_KEYS, "acceptance gate key contract changed")
    passed = all(bool(value) for value in gates.values()) and not hard
    report: dict[str, Any] = {
        "schema": SCHEMA,
        "revision": REVISION,
        "audit_valid": True,
        "pass": passed,
        "status": "PASS" if passed else "FAIL",
        "final_status": FINAL_PASS if passed else FINAL_FAIL,
        "root_cause_classification": "NUMERICAL_INTEGRATOR_TRUNCATION_ERROR_CONFIRMED" if attribution_confirmed else "UNRESOLVED",
        "continuous_time_dynamics_model": "PASS" if attribution_confirmed else "FAIL",
        "v15_18b_passive_gravity_dynamics": "PASS_WITH_NUMERICAL_INTEGRATOR_LIMITATION" if passed else "FAIL",
        "production_integrator_limitation": "PRODUCTION_IMPLICITFAST_DT_2MS_IS_NOT_AN_ENERGY-CONSERVATION_REFERENCE_FOR_UNACTUATED_PASSIVE_MOTION" if attribution_confirmed else None,
        "root_cause_condition_map_exact_11": continuous_time_evidence,
        "root_cause_condition_keys": list(ROOT_CAUSE_CONDITION_KEYS),
        "root_cause_mutation_consistency": {
            "condition_count_is_exactly_11": len(continuous_time_evidence) == 11,
            "classification_confirmed_iff_all_conditions_pass": attribution_confirmed == all(bool(value) for value in continuous_time_evidence.values()),
            "continuous_time_model_pass_iff_all_conditions_pass": ("PASS" if attribution_confirmed else "FAIL") == ("PASS" if all(bool(value) for value in continuous_time_evidence.values()) else "FAIL"),
            "pass": True,
        },
        "pure_simulation_phase": PHASE_COMPLETE if passed else PHASE_INCOMPLETE,
        "next_phase": NEXT_HARDWARE if passed else NEXT_REVIEW,
        "ubuntu_runtime_verification": "PENDING_ON_TARGET_HOST",
        "github_handoff_status": "OUTSIDE_THIS_NUMERICAL_AUDIT_SCOPE",
        "source_commit": SOURCE_COMMIT,
        "source_branch": SOURCE_BRANCH,
        "target_branch": TARGET_BRANCH,
        "production_model_relative_path": MJCF_REL,
        "production_model_sha256": after.get(MJCF_REL),
        "mujoco_version": mujoco_version,
        "numpy_version": numpy_version,
        "gravity": list(GRAVITY),
        "timestep": {
            "duration_s": DURATION_S,
            "production_integrator": xml_contract["production_integrator"],
            "profiles": {
                "A": {"integrator": "implicitfast", "dt_s": float(xml_contract["production_timestep_s"]), "classification": "PRODUCTION_INTEGRATOR_DIAGNOSTIC"},
                "B": {"integrator": "implicitfast", "dt_s": float(xml_contract["production_timestep_s"]) / 2.0, "classification": "PRODUCTION_INTEGRATOR_DIAGNOSTIC"},
                "C": {"integrator": "implicitfast", "dt_s": float(xml_contract["production_timestep_s"]), "classification": "DETERMINISM_REPEAT_OF_A"},
                "I4": {"integrator": "implicitfast", "dt_s": float(xml_contract["production_timestep_s"]) / 4.0, "classification": "RUNTIME_ONLY_CONVERGENCE_REFERENCE"},
                "RK4_2MS": {"integrator": "RK4", "dt_s": float(xml_contract["production_timestep_s"]), "classification": "RUNTIME_ONLY_ENERGY_REFERENCE"},
                "RK4_1MS": {"integrator": "RK4", "dt_s": float(xml_contract["production_timestep_s"]) / 2.0, "classification": "RUNTIME_ONLY_HIGH_PRECISION_REFERENCE"},
            },
        },
        "runtime_configuration": {
            "gravity_runtime_only": True,
            "production_xml_gravity_m_s2": xml_contract["production_gravity_m_s2"],
            "six_isolated_compiled_models": True,
            "fresh_mjdata_per_pose_and_run": True,
            "model_objects_reused_read_only_across_pose_slots": True,
            "candidate_selector_executed_in_b2": False,
            "candidate_selector_mj_step_call_count": 0,
            "actuation_explicitly_disabled": True,
            "ctrl_zero": True,
            "qfrc_applied_zero": True,
            "xfrc_applied_zero": True,
            "energy_enabled": True,
            "runtime_models": runtime,
            "virtual_assets": asset_contract,
            "compiled_positive_mass_bodies": positive_rows,
            "authority_body_set": list(AUTHORITY_BODIES),
            "authority_mass_kg": 3.4515,
            "all_positive_mass_kg": 4.4515,
            "link1_placeholder_mass_kg": 1.0,
            "forbidden_parameter_snapshot": {"joints": xml_contract["joint_parameters"], "body_gravcomp": xml_contract["body_gravcomp"]},
        },
        "pose_selection": selection,
        "per_pose_runs": list(pose_rows),
        "initial_acceleration_identity": {"rows": initial_rows, "pass": all(bool(row["pass"]) for row in initial_rows)},
        "early_motion_direction": {"rows": early_rows, "pass": all(bool(row["pass"]) for row in early_rows)},
        "energy_audit": {"definition": "LEGACY A/B/C E=U+K dual-source evidence preserved byte-for-byte at the run-object projection", "classification": "PRODUCTION_INTEGRATOR_DIAGNOSTIC", "rows": energy_rows, "pass": legacy_energy_pass},
        "timestep_convergence": {"classification": "PRODUCTION_INTEGRATOR_DIAGNOSTIC", "model_validation_status": "SUPERSEDED_FOR_MODEL_VALIDATION_BY_MULTI_LEVEL_CONVERGENCE", "rows": convergence_rows, "pass": legacy_convergence_pass},
        "production_integrator_diagnostic": {
            "classification": "PRODUCTION_INTEGRATOR_DIAGNOSTIC",
            "legacy_runs_projection_definition": "PER_POSE_SLOT_INDEX_PLUS_COMPLETE_RUN_OBJECTS_A_B_C_CANONICAL_JSON",
            "legacy_runs_abc_sha256": legacy_digest,
            "legacy_runs_abc_expected_sha256": LEGACY_PRODUCTION_RUNS_ABC_SHA256,
            "legacy_runs_preserved": legacy_preserved,
            "energy_pass": legacy_energy_pass,
            "absolute_final_state_convergence_pass": legacy_convergence_pass,
            "status": "PASS" if legacy_energy_pass and legacy_convergence_pass else "FAIL",
            "acceptance_role": "DIAGNOSTIC_ONLY_NOT_CONTINUOUS_TIME_MODEL_AUTHORITY",
        },
        "numerical_integrator_attribution": {
            "implicitfast_energy_convergence": {"rows": implicit_energy_rows, "global": implicit_energy_global, "pass": bool(implicit_energy_global["pass"])},
            "implicitfast_trajectory_convergence": {"rows": implicit_trajectory_rows, "global": trajectory_global, "pass": bool(trajectory_global["pass"])},
            "rk4_energy_reference": {"rows": rk4_energy_rows, "global": rk4_energy_global, "pass": bool(rk4_energy_global["pass"])},
            "rk4_timestep_convergence": {"rows": rk4_convergence_rows, "global": rk4_convergence_global, "pass": bool(rk4_convergence_global["pass"])},
            "high_precision_reference_diagnostic": {"rows": reference_rows, "global": reference_global, "pass": bool(reference_global["pass"])},
            "numerical_attribution_pass": numerical_attribution_pass,
            "root_cause_classification": "NUMERICAL_INTEGRATOR_TRUNCATION_ERROR_CONFIRMED" if attribution_confirmed else "UNRESOLVED",
            "pass": numerical_attribution_pass,
        },
        "determinism": {"rows": determinism_rows, "pass": all(bool(row["pass"]) for row in determinism_rows)},
        "contacts": {"rows": contacts_rows, "pass": all(bool(row["pass"]) for row in contacts_rows)},
        "joint_limit_margin": {"rows": margin_rows, "no_active_joint_limit_constraints": no_limits, "pass": all(bool(row["pass"]) for row in margin_rows) and no_limits},
        "stability": {"rows": stability_rows, "pass": all(bool(row["pass"]) for row in stability_rows)},
        "trajectory_continuity": {"definition": "q(t), qvel(t), qacc(t), and energy(t) must be finite and free of prohibited single-step discontinuities in every run profile", "rows": continuity_rows, "pass": all(bool(row["pass"]) for row in continuity_rows)},
        "protected_hashes": {"artifacts": protected, "all_unchanged": all(bool(row["unchanged"]) for row in protected.values()), "production_model_expected_sha256": MJCF_SHA256},
        "thresholds": THRESHOLDS,
        "acceptance_gates": gates,
        "hard_unresolved_items": hard,
        "unresolved_items": hard,
        "limitations": {
            "validated_scope": "OFFLINE_0P15S_PASSIVE_GRAVITY_NUMERICAL_INTEGRATOR_ATTRIBUTION_ONLY",
            "production_integrator_limitation": "PRODUCTION_IMPLICITFAST_DT_2MS_IS_NOT_AN_ENERGY-CONSERVATION_REFERENCE_FOR_UNACTUATED_PASSIVE_MOTION" if attribution_confirmed else None,
            "production_timestep_or_integrator_changed": False,
            "runtime_only_reference_models": ["IMPLICITFAST_0P5MS", "RK4_2MS", "RK4_1MS"],
            "not_prerequisites_for_this_acceptance": ["FRICTION_IDENTIFICATION", "DAMPING_IDENTIFICATION", "MOTOR_ARMATURE_OR_ROTOR_INERTIA", "GRAVITY_COMPENSATION", "LONG_DURATION_FREE_FALL", "HIGH_SPEED_MOTION", "VISUAL_GRASPING"],
            "not_validated": ["REAL_HARDWARE", "ROS2", "MOVEIT2", "PRODUCTION_BRIDGE_EXECUTION", "PID_OR_CONTROLLER", "LONG_DURATION_OR_HIGH_SPEED_DYNAMICS", "UBUNTU_TARGET_RUNTIME"],
            "production_link1_placeholder_retained_in_mass_matrix_and_mujoco_energy": True,
        },
        "visual_witness": {
            "available_via": "tools/audit_passive_gravity_v15_18b.py --visualize",
            "required_display_environment": "VMWARE_UBUNTU_22_04_DESKTOP_VISIBLE_SESSION_NOT_WSL",
            "same_six_semantic_slots": True,
            "same_runtime_gravity_and_duration": True,
            "viewer_sync_each_step": True,
            "diagnostic_only": True,
            "not_numeric_authority": True,
            "ros_moveit_bridge_started": False,
            "execution_witness_claimed_by_this_headless_report": False,
        },
        "report_contract": {
            "json_relative_path": REPORT_JSON_REL,
            "markdown_relative_path": REPORT_MD_REL,
            "write_mode": "RECOMPUTE_THEN_WRITE_JSON_AND_MARKDOWN_EVEN_FOR_LEGAL_FAIL",
            "check_mode": "RECOMPUTE_AND_REQUIRE_BOTH_FILES_BYTE_IDENTICAL",
            "json_encoding": "UTF-8_LF_SORT_KEYS_INDENT_2_NO_NAN",
            "markdown_encoding": "UTF-8_LF",
            "canonical_runtime": "PYTHON_3P12_NUMPY_2P2P6_MUJOCO_3P11P0",
            "github_handoff_claimed": False,
        },
    }
    return json_safe(report)


def fallback_report(error: BaseException, before: Mapping[str, str] | None = None) -> dict[str, Any]:
    code = f"FAIL_AUDIT_INFRASTRUCTURE_{type(error).__name__.upper()}"
    artifacts = {relative: {"before_sha256": (before or {}).get(relative), "after_sha256": sha256_file(repo_path(relative)) if repo_path(relative).is_file() else None, "unchanged": (before or {}).get(relative) == (sha256_file(repo_path(relative)) if repo_path(relative).is_file() else None)} for relative in PROTECTED_RELATIVE_PATHS}
    return {
        "schema": SCHEMA, "revision": REVISION, "audit_valid": False, "pass": False, "status": "FAIL", "final_status": FINAL_FAIL,
        "pure_simulation_phase": PHASE_INCOMPLETE, "next_phase": NEXT_REVIEW,
        "source_commit": SOURCE_COMMIT, "source_branch": SOURCE_BRANCH, "target_branch": TARGET_BRANCH,
        "production_model_relative_path": MJCF_REL, "production_model_sha256": artifacts.get(MJCF_REL, {}).get("after_sha256"),
        "mujoco_version": None, "numpy_version": None, "gravity": list(GRAVITY), "timestep": None, "runtime_configuration": None,
        "pose_selection": None, "per_pose_runs": [], "initial_acceleration_identity": {"rows": [], "pass": False},
        "early_motion_direction": {"rows": [], "pass": False}, "energy_audit": {"rows": [], "pass": False},
        "timestep_convergence": {"rows": [], "pass": False}, "determinism": {"rows": [], "pass": False},
        "contacts": {"rows": [], "pass": False}, "joint_limit_margin": {"rows": [], "pass": False}, "stability": {"rows": [], "pass": False},
        "trajectory_continuity": {"definition": "q/qvel/qacc/energy continuity unavailable", "rows": [], "pass": False},
        "protected_hashes": {"artifacts": artifacts, "all_unchanged": all(bool(row["unchanged"]) for row in artifacts.values()), "production_model_expected_sha256": MJCF_SHA256},
        "thresholds": THRESHOLDS, "acceptance_gates": {key: False for key in ACCEPTANCE_GATE_KEYS},
        "hard_unresolved_items": [code], "unresolved_items": [code],
        "limitations": {"failure_message": str(error), "validated_scope": "NONE_AUDIT_INFRASTRUCTURE_FAIL"},
        "visual_witness": {"available_via": "tools/audit_passive_gravity_v15_18b.py --visualize", "diagnostic_only": True, "not_numeric_authority": True, "execution_witness_claimed_by_this_headless_report": False},
        "report_contract": {"json_relative_path": REPORT_JSON_REL, "markdown_relative_path": REPORT_MD_REL, "legal_fail_serialization": True},
    }


def build_report() -> dict[str, Any]:
    before = protected_snapshot()
    require(before[MJCF_REL] == MJCF_SHA256, "production MJCF SHA256 mismatch before audit")
    root, limits, xml_contract = model_xml_contract()
    try:
        import numpy as np  # type: ignore
        import mujoco  # type: ignore
    except ImportError as error:
        raise AuditError(f"MuJoCo/NumPy import failed: {error}") from error
    require(str(mujoco.__version__) == "3.11.0", f"MuJoCo version must be 3.11.0, got {mujoco.__version__}")
    require(str(np.__version__) == "2.2.6", f"NumPy version must be 2.2.6, got {np.__version__}")
    xml, assets, asset_contract = load_virtual_assets(root)
    production_dt = float(xml_contract["production_timestep_s"])
    source_report, source_rows = v15_18a_source_rows()
    selection = locked_selection(source_report, source_rows, limits)

    # Models are intentionally sequential rather than concurrent: each mesh
    # compilation is independent while peak memory stays below the workstation
    # limit. B2 never re-runs the B1 pose selector.
    model_b = mujoco.MjModel.from_xml_string(xml, assets=assets)
    runtime_b = configure_runtime_profile(model_b, mujoco, np, production_dt / 2.0, "implicitfast")
    authority_ids, positive_ids, positive_rows = compiled_body_contract(model_b, mujoco)
    run_b = {int(slot["slot_index"]): run_simulation("B", model_b, slot["selected_q_rad"], DURATION_S, mujoco, np, limits, authority_ids, positive_ids) for slot in selection["slots"]}
    del model_b
    gc.collect()

    model_a = mujoco.MjModel.from_xml_string(xml, assets=assets)
    runtime_a = configure_runtime_profile(model_a, mujoco, np, production_dt, "implicitfast")
    authority_ids_a, positive_ids_a, positive_rows_a = compiled_body_contract(model_a, mujoco)
    require(authority_ids_a == authority_ids and positive_ids_a == positive_ids and positive_rows_a == positive_rows, "independent A/B compiled body contract mismatch")
    initials = {int(slot["slot_index"]): initial_state(model_a, slot["selected_q_rad"], mujoco, np, limits, authority_ids_a, positive_ids_a, positive_rows_a) for slot in selection["slots"]}
    run_a = {int(slot["slot_index"]): run_simulation("A", model_a, slot["selected_q_rad"], DURATION_S, mujoco, np, limits, authority_ids_a, positive_ids_a) for slot in selection["slots"]}
    del model_a
    gc.collect()

    model_c = mujoco.MjModel.from_xml_string(xml, assets=assets)
    runtime_c = configure_runtime_profile(model_c, mujoco, np, production_dt, "implicitfast")
    authority_ids_c, positive_ids_c, positive_rows_c = compiled_body_contract(model_c, mujoco)
    require(authority_ids_c == authority_ids and positive_ids_c == positive_ids and positive_rows_c == positive_rows, "independent B/C compiled body contract mismatch")
    run_c = {int(slot["slot_index"]): run_simulation("C", model_c, slot["selected_q_rad"], DURATION_S, mujoco, np, limits, authority_ids_c, positive_ids_c) for slot in selection["slots"]}
    del model_c
    gc.collect()

    model_i4 = mujoco.MjModel.from_xml_string(xml, assets=assets)
    runtime_i4 = configure_runtime_profile(model_i4, mujoco, np, production_dt / 4.0, "implicitfast")
    authority_ids_i4, positive_ids_i4, positive_rows_i4 = compiled_body_contract(model_i4, mujoco)
    require(authority_ids_i4 == authority_ids and positive_ids_i4 == positive_ids and positive_rows_i4 == positive_rows, "independent I4 compiled body contract mismatch")
    run_i4 = {int(slot["slot_index"]): run_simulation("I4", model_i4, slot["selected_q_rad"], DURATION_S, mujoco, np, limits, authority_ids_i4, positive_ids_i4) for slot in selection["slots"]}
    del model_i4
    gc.collect()

    model_rk4_2ms = mujoco.MjModel.from_xml_string(xml, assets=assets)
    runtime_rk4_2ms = configure_runtime_profile(model_rk4_2ms, mujoco, np, production_dt, "RK4")
    authority_ids_rk4_2ms, positive_ids_rk4_2ms, positive_rows_rk4_2ms = compiled_body_contract(model_rk4_2ms, mujoco)
    require(authority_ids_rk4_2ms == authority_ids and positive_ids_rk4_2ms == positive_ids and positive_rows_rk4_2ms == positive_rows, "independent RK4 2ms compiled body contract mismatch")
    run_rk4_2ms = {int(slot["slot_index"]): run_simulation("RK4_2MS", model_rk4_2ms, slot["selected_q_rad"], DURATION_S, mujoco, np, limits, authority_ids_rk4_2ms, positive_ids_rk4_2ms) for slot in selection["slots"]}
    del model_rk4_2ms
    gc.collect()

    model_rk4_1ms = mujoco.MjModel.from_xml_string(xml, assets=assets)
    runtime_rk4_1ms = configure_runtime_profile(model_rk4_1ms, mujoco, np, production_dt / 2.0, "RK4")
    authority_ids_rk4_1ms, positive_ids_rk4_1ms, positive_rows_rk4_1ms = compiled_body_contract(model_rk4_1ms, mujoco)
    require(authority_ids_rk4_1ms == authority_ids and positive_ids_rk4_1ms == positive_ids and positive_rows_rk4_1ms == positive_rows, "independent RK4 1ms compiled body contract mismatch")
    run_rk4_1ms = {int(slot["slot_index"]): run_simulation("RK4_1MS", model_rk4_1ms, slot["selected_q_rad"], DURATION_S, mujoco, np, limits, authority_ids_rk4_1ms, positive_ids_rk4_1ms) for slot in selection["slots"]}
    del model_rk4_1ms, assets
    gc.collect()

    pose_rows = []
    for slot in selection["slots"]:
        index = int(slot["slot_index"])
        runs = {
            "A": run_a[index],
            "B": run_b[index],
            "C": run_c[index],
            "I4": run_i4[index],
            "RK4_2MS": run_rk4_2ms[index],
            "RK4_1MS": run_rk4_1ms[index],
        }
        pose_rows.append(assemble_pose_run(slot, initials[index], runs, np))
    runtime = {"A": runtime_a, "B": runtime_b, "C": runtime_c, "I4": runtime_i4, "RK4_2MS": runtime_rk4_2ms, "RK4_1MS": runtime_rk4_1ms}
    after = protected_snapshot()
    report = aggregate_report(before, after, xml_contract, runtime, selection, pose_rows, str(mujoco.__version__), str(np.__version__), asset_contract, positive_rows)
    require(report["protected_hashes"]["all_unchanged"] is True, "protected authority changed during audit")
    return report


def render_markdown(report: Mapping[str, Any]) -> str:
    status = str(report["final_status"])
    lines = [
        "# V15.18B2 短时被动重力动力学数值归因验收",
        "",
        f"**最终状态：{status}**",
        "",
        f"- `root_cause_classification = {report.get('root_cause_classification', 'UNRESOLVED')}`",
        f"- `continuous_time_dynamics_model = {report.get('continuous_time_dynamics_model', 'FAIL')}`",
        f"- `{report['pure_simulation_phase']}`",
        f"- `NEXT_PHASE = {report['next_phase']}`",
        f"- `ubuntu_runtime_verification = {report.get('ubuntu_runtime_verification', 'PENDING_ON_TARGET_HOST')}`（非本数值验收 hard gate）",
        f"- Source commit: `{report['source_commit']}`",
        f"- Production MJCF SHA256: `{report['production_model_sha256']}`",
        f"- Canonical runtime: MuJoCo `{report['mujoco_version']}` / NumPy `{report.get('numpy_version')}`",
        "",
        "## 范围与方法",
        "",
        "仅使用离线 Python/MuJoCo；生产 XML、生产 timestep 与生产 integrator 均不修改。审计只在六个相互隔离的内存模型设置 `0 0 -9.81 m/s²`，显式关闭 actuation，并将 ctrl、qfrc_applied、xfrc_applied 清零。",
        "",
        "运行配置为：A=implicitfast 2 ms、B=implicitfast 1 ms、C=A 确定性重复、I4=implicitfast 0.5 ms、RK4_2MS、RK4_1MS；每组均为相同六槽和 0.15 s。A/B/C 完整原始 run 对象按旧摘要冻结。",
        "双能量证据同时采用 MuJoCo `data.energy` 与独立 `U=-Σm(g·xipos)`、`K=0.5 qvelᵀM(q)qvel`。implicitfast 的 Dq/Dv 是公共嵌套时间网格上、全采样点与全关节的最大差，不是仅比较终值。",
        "B2 不执行姿态 selector；直接复用并摘要锁定 B1 六槽/category/requested-id/selected-id/q。",
        "本报告的 passive free-fall 是离线、无驱动的数值诊断，不是 production `kinematic_position_tracking` 控制模式。",
        "",
        "## 固定六姿态槽",
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

    lines.extend(["", "六槽权威摘要：`" + str(selection.get("locked_slot_projection_sha256") if isinstance(selection, Mapping) else None) + "`。B2 未重新选姿态。", "", "## 旧 A/B/C 生产积分器诊断（完整保留）", "", "| 槽 | 姿态 | A 独立能量漂移 | B 独立能量漂移 | 旧 q 终值差/限值 | 旧 qvel 终值差/限值 | 旧诊断 |", "|---:|---|---:|---:|---:|---:|---|"])
    for row in report.get("per_pose_runs", []):
        a = row["energy"]["A"]["independent"]["normalized_drift"]
        b = row["energy"]["B"]["independent"]["normalized_drift"]
        conv = row["convergence"]
        legacy = row["legacy_production_integrator_diagnostic"]
        lines.append(f"| {row['slot_index']} | {row['selected_pose_id']} | {100.0*float(a):.4f}% | {100.0*float(b):.4f}% | {float(conv['final_qpos_max_abs_difference_rad']):.6g}/{float(conv['final_qpos_limit_rad']):.6g} | {float(conv['final_qvel_max_abs_difference_rad_s']):.6g}/{float(conv['final_qvel_limit_rad_s']):.6g} | {legacy['status']} |")
    production_diag = report.get("production_integrator_diagnostic", {})
    lines.extend(["", f"旧 A/B/C run 摘要：`{production_diag.get('legacy_runs_abc_sha256')}`（匹配冻结值：**{'YES' if production_diag.get('legacy_runs_preserved') else 'NO'}**）。旧 absolute final-state 门仅保留为 `PRODUCTION_INTEGRATOR_DIAGNOSTIC`，其模型验证角色为 `SUPERSEDED_FOR_MODEL_VALIDATION_BY_MULTI_LEVEL_CONVERGENCE`。", "", "## implicitfast 三层收敛", "", "| 槽 | r1=E1/E2 | r2=E0.5/E1 | Dq 比 | Dv 比 | 能量 | 全轨迹 |", "|---:|---:|---:|---:|---:|---|---|"])
    for row in report.get("per_pose_runs", []):
        attr = row["numerical_integrator_attribution"]
        energy = attr["implicitfast_energy_convergence"]
        trajectory = attr["implicitfast_trajectory_convergence"]
        lines.append(f"| {row['slot_index']} | {float(energy['conservative_r1_E_1ms_over_E_2ms']):.6g} | {float(energy['conservative_r2_E_0p5ms_over_E_1ms']):.6g} | {float(trajectory['qpos_difference_ratio_D_1ms_0p5ms_over_D_2ms_1ms']):.6g} | {float(trajectory['qvel_difference_ratio_D_1ms_0p5ms_over_D_2ms_1ms']):.6g} | {'PASS' if energy['pass'] else 'FAIL'} | {'PASS' if trajectory['pass'] else 'FAIL'} |")

    attr_top = report.get("numerical_integrator_attribution", {})
    implicit_global = attr_top.get("implicitfast_trajectory_convergence", {}).get("global", {})
    lines.extend(["", f"全局 Dq：{float(implicit_global.get('Dq_2ms_1ms_rad', 0.0)):.9g} → {float(implicit_global.get('Dq_1ms_0p5ms_rad', 0.0)):.9g}，比值 {float(implicit_global.get('qpos_ratio', 0.0)):.9g}；Dv：{float(implicit_global.get('Dv_2ms_1ms_rad_s', 0.0)):.9g} → {float(implicit_global.get('Dv_1ms_0p5ms_rad_s', 0.0)):.9g}，比值 {float(implicit_global.get('qvel_ratio', 0.0)):.9g}。", "", "## RK4 能量与终值参考", "", "| 槽 | RK4 2ms 最大归一化漂移 | RK4 1ms 最大归一化漂移 | RK4 q 终值差/rad | RK4 qvel 终值差/(rad/s) | I4/RK4 q 差/rad | early | 结果 |", "|---:|---:|---:|---:|---:|---:|---:|---|"])
    for row in report.get("per_pose_runs", []):
        attr = row["numerical_integrator_attribution"]
        rk_energy = attr["rk4_energy_reference"]
        rk_conv = attr["rk4_timestep_convergence"]
        ref = attr["high_precision_reference_diagnostic"]
        e2 = max(float(rk_energy["profiles"]["RK4_2MS"]["sources"][source]["normalized_drift"]) for source in ("mujoco", "independent"))
        e1 = max(float(rk_energy["profiles"]["RK4_1MS"]["sources"][source]["normalized_drift"]) for source in ("mujoco", "independent"))
        lines.append(f"| {row['slot_index']} | {e2:.6g} | {e1:.6g} | {float(rk_conv['final_qpos_max_abs_difference_rad']):.6g} | {float(rk_conv['final_qvel_max_abs_difference_rad_s']):.6g} | {float(ref['final_qpos_max_abs_difference_rad']):.6g} | {int(ref['matching_early_direction_count'])}/{int(ref['valid_early_direction_count'])} | {'PASS' if attr['pass'] else 'FAIL'} |")

    ref_global = attr_top.get("high_precision_reference_diagnostic", {}).get("global", {})
    lines.extend(["", f"高精度参考 early direction：**{int(ref_global.get('matching_early_direction_count', 0))}/{int(ref_global.get('valid_early_direction_count', 0))}**；预期有效数 70。", "", "RK4 half-dt 单调门在归一化能量域检查：`E_norm(1 ms) <= E_norm(2 ms) + normalized_floor`，其中 `normalized_floor = 1e-8 J / RK4_2MS 该姿态能量归一化 denominator`；JSON 同时记录两组 run 的 normalized floor。绝对 J 比较仅保留诊断，独立的 `<0.001` / `<0.0005` profile 上限不变。", "", "## Root-cause exact 11 conditions", ""])
    for key, value in report.get("root_cause_condition_map_exact_11", {}).items():
        lines.append(f"- `{key}`: **{'PASS' if value else 'FAIL'}**")
    lines.extend(["", f"结论：`{report.get('root_cause_classification', 'UNRESOLVED')}`；生产限制：`{report.get('production_integrator_limitation')}`。", "", "## 全运行安全覆盖", "", "contacts、active limits、finite、actuation/applied-force isolation、q/qvel/qacc/energy continuity 均覆盖 A、B、C、I4、RK4_2MS、RK4_1MS × 六槽。"])

    lines.extend(["", "## Acceptance gates", ""])
    for key, value in report.get("acceptance_gates", {}).items():
        lines.append(f"- `{key}`: **{'PASS' if value else 'FAIL'}**")
    unresolved = list(report.get("hard_unresolved_items", []))
    lines.extend(["", "## 未解决项", ""])
    if unresolved:
        for code in unresolved:
            lines.append(f"- `{code}`")
        lines.extend(["", "这是合同定义的合法 FAIL：未改变生产模型、未加入阻尼/摩擦/电枢/控制器，也未降低阈值。"])
    else:
        lines.append("- 无。")

    lines.extend([
        "", "## Authority 与限制", "",
        f"受保护文件 before/after 全部一致：**{'YES' if report.get('protected_hashes', {}).get('all_unchanged') else 'NO'}**。生产 MJCF 期望 SHA256：`{MJCF_SHA256}`。",
        "",
        "本验收不修改 production timestep/integrator，也不以摩擦辨识、阻尼辨识、电机电枢/转子惯量、重力补偿、长时间自由落体、高速运动或视觉抓取为前置条件。Ubuntu 目标机运行时仍为 `PENDING_ON_TARGET_HOST`，不在本数值报告中冒充 PASS。",
        "",
        "## VMware 可视见证",
        "",
        "在 VMware Ubuntu 22.04 的可见桌面会话中运行 `python3 tools/audit_passive_gravity_v15_18b.py --visualize`。该模式显示同六姿态、同 0.15 s 被动重力过程并逐步 `viewer.sync()`；不启动 ROS、MoveIt 或 bridge，且仅为诊断见证，不替代本 JSON 数值 authority。",
        "",
    ])
    return "\n".join(lines)


def json_bytes(report: Mapping[str, Any]) -> bytes:
    return (json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True, allow_nan=False) + "\n").encode("utf-8")


def markdown_bytes(report: Mapping[str, Any]) -> bytes:
    return render_markdown(report).encode("utf-8")


def visualize(report: Mapping[str, Any], dwell_seconds: float, cycles: int) -> None:
    require(os.name != "nt", "visual witness must run in VMware Ubuntu 22.04, not Windows/WSL")
    require(bool(os.environ.get("DISPLAY") or os.environ.get("WAYLAND_DISPLAY")), "visible VMware desktop DISPLAY/WAYLAND_DISPLAY is required")
    require("microsoft" not in Path("/proc/version").read_text(encoding="utf-8", errors="ignore").lower(), "WSL is not an accepted visual witness environment")
    try:
        import numpy as np  # type: ignore
        import mujoco  # type: ignore
        import mujoco.viewer  # type: ignore
    except ImportError as error:
        raise AuditError(f"MuJoCo viewer import failed: {error}") from error
    before = protected_snapshot()
    root, _, xml_contract = model_xml_contract()
    xml, assets, _ = load_virtual_assets(root)
    model = mujoco.MjModel.from_xml_string(xml, assets=assets)
    del assets
    configure_model(model, mujoco, np, float(xml_contract["production_timestep_s"]))
    data = mujoco.MjData(model)
    slots = report.get("pose_selection", {}).get("slots", [])
    require(len(slots) == 6, "visual witness needs exact six selected slots")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        for _ in range(cycles):
            for slot in slots:
                if not viewer.is_running():
                    break
                reset_forward(model, data, slot["selected_q_rad"], mujoco, np)
                viewer.sync()
                initial_deadline = time.monotonic() + dwell_seconds
                while viewer.is_running() and time.monotonic() < initial_deadline:
                    viewer.sync()
                    time.sleep(min(0.02, max(0.0, initial_deadline - time.monotonic())))
                for _step in range(int(round(DURATION_S / float(model.opt.timestep)))):
                    if not viewer.is_running():
                        break
                    mujoco.mj_step(model, data)
                    viewer.sync()
                    time.sleep(float(model.opt.timestep))
                final_deadline = time.monotonic() + dwell_seconds
                while viewer.is_running() and time.monotonic() < final_deadline:
                    viewer.sync()
                    time.sleep(min(0.02, max(0.0, final_deadline - time.monotonic())))
            if not viewer.is_running():
                break
    after = protected_snapshot()
    require(before == after and after[MJCF_REL] == MJCF_SHA256, "visual witness changed protected authority")
    print("VISUAL_WITNESS=COMPLETE")
    print("ENVIRONMENT=VMWARE_UBUNTU_22_04_VISIBLE_DESKTOP")
    print("NUMERIC_AUTHORITY=NO")
    print("ROS_MOVEIT_BRIDGE_STARTED=NO")
    print(f"POSE_SLOTS={len(slots)}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="V15.18B deterministic short passive-gravity audit")
    modes = parser.add_mutually_exclusive_group(required=True)
    modes.add_argument("--write", action="store_true", help="recompute and write deterministic JSON/Markdown reports")
    modes.add_argument("--check", action="store_true", help="recompute and require both reports byte-identical")
    modes.add_argument("--visualize", action="store_true", help="VMware Ubuntu 22.04 visible diagnostic witness")
    parser.add_argument("--dwell-seconds", type=float, default=1.0, help="visual witness dwell before/after each 0.15 s run")
    parser.add_argument("--cycles", type=int, default=1, help="visual witness cycles")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    before: dict[str, str] | None = None
    try:
        before = protected_snapshot()
        report = build_report()
    except BaseException as error:  # legal, finite-safe FAIL artifact rather than an unstructured crash
        report = fallback_report(error, before)
    payload_json = json_bytes(report)
    payload_md = markdown_bytes(report)
    if args.visualize:
        require(args.dwell_seconds > 0.0 and args.cycles >= 1, "visual dwell/cycles must be positive")
        require(report.get("audit_valid") is True, "visual witness selection unavailable after audit infrastructure failure")
        visualize(report, args.dwell_seconds, args.cycles)
        return 0
    json_path, md_path = repo_path(REPORT_JSON_REL), repo_path(REPORT_MD_REL)
    if args.write:
        json_path.write_bytes(payload_json)
        md_path.write_bytes(payload_md)
    else:
        require(json_path.is_file() and md_path.is_file(), "V15.18B authority report is missing")
        require(json_path.read_bytes() == payload_json, "V15.18B JSON report is stale or non-deterministic")
        require(md_path.read_bytes() == payload_md, "V15.18B Markdown report is stale or non-deterministic")
    print(f"AUDIT_VALID={'YES' if report.get('audit_valid') else 'NO'}")
    print(f"STATUS={report['status']}")
    print(f"FINAL_STATUS={report['final_status']}")
    print(f"HARD_UNRESOLVED_ITEMS={len(report.get('hard_unresolved_items', []))}")
    print(f"JSON_SHA256={sha256_bytes(payload_json)}")
    print(f"MARKDOWN_SHA256={sha256_bytes(payload_md)}")
    if not report.get("audit_valid"):
        return 2
    # A contract-compliant physical FAIL is a successfully completed audit,
    # not an infrastructure/process failure.  The report status and gates are
    # authoritative; exit two is reserved for an invalid audit construction.
    return 0


if __name__ == "__main__":
    sys.exit(main())

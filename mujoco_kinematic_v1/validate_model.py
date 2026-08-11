from __future__ import annotations

"""Numerically validate V15.13 frames, axes, signs, limits and collisions."""

import json
import math
import random
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np


ROOT = Path(__file__).resolve().parent
MODEL_XML = ROOT / "go_m8010_arm_v15_13_kinematic.xml"
AXIS_JSON = ROOT / "V15_13_真实关节轴与相机上置零位.json"
SOURCE_MESH_MANIFEST = ROOT / "mesh_export_manifest.json"
RUNTIME_MESH_MANIFEST = ROOT / "runtime_mesh_manifest.json"
REFERENCE_QA = ROOT / "QA_V15_13_整机深度碰撞与限位.json"
PAIR_CONTRACT = ROOT / "V15_13_自碰撞对矩阵契约.json"
OUTPUT_QA = ROOT / "QA_MUJOCO_V15_13_空载运动学版.json"

JOINT_SOURCE = {"J1": "J1", "J2": "J2A", "J3": "J3", "J4": "J4", "J5": "J5", "J6": "J6"}
LINKS = ["base_link", "link1", "link2", "link3", "link4", "link5", "link6", "gripper"]

FRAME_POSITION_TOLERANCE_M = 2.0e-9
FRAME_ROTATION_TOLERANCE = 2.0e-9
AXIS_TOLERANCE = 2.0e-9
POSITIVE_SIGN_TOLERANCE_M = 2.0e-9
LIMIT_TOLERANCE_RAD = 1.0e-12
VISUAL_BOUNDS_TOLERANCE_M = 5.0e-5


def frame_matrix(frame: dict) -> np.ndarray:
    return np.column_stack(
        (
            frame["x_axis_world_at_zero"],
            frame["y_axis_world_at_zero"],
            frame["z_axis_world_at_zero"],
        )
    ).astype(float)


def inclusive_range(start: float, stop: float, step: float) -> list[float]:
    values = []
    current = float(start)
    while current <= stop + 1.0e-9:
        values.append(round(current, 10))
        current += step
    if abs(values[-1] - stop) > 1.0e-9:
        values.append(float(stop))
    return values


def zero_pose() -> dict[str, float]:
    return {f"j{index}_deg": 0.0 for index in range(1, 7)}


def build_pose_sets() -> dict[str, list[dict[str, float]]]:
    result = {}
    scan_specs = {
        "independent_J1_representative_full_turn": (-180.0, 180.0, 10.0),
        "independent_J2": (-170.0, 170.0, 10.0),
        "independent_J3": (-170.0, 170.0, 10.0),
        "independent_J4": (-116.0, 159.0, 5.0),
        "independent_J5": (-70.6, 151.2, 5.0),
        "independent_J6": (-180.0, 180.0, 5.0),
    }
    for category, (lower, upper, step) in scan_specs.items():
        joint_index = int(category.split("J")[-1].split("_")[0])
        poses = []
        for angle in inclusive_range(lower, upper, step):
            pose = zero_pose()
            pose[f"j{joint_index}_deg"] = angle
            poses.append(pose)
        result[category] = poses

    grid = []
    for j5 in (-70.6, 0.0, 151.2):
        for j6 in inclusive_range(-180.0, 180.0, 30.0):
            pose = zero_pose()
            pose["j5_deg"] = j5
            pose["j6_deg"] = j6
            grid.append(pose)
    result["coupled_J5_J6_grid"] = grid

    corners = []
    for j2 in (-170.0, 170.0):
        for j3 in (-170.0, 170.0):
            for j4 in (-116.0, 159.0):
                for j5 in (-70.6, 151.2):
                    for j6 in (-180.0, 180.0):
                        pose = zero_pose()
                        pose.update(
                            j2_deg=j2,
                            j3_deg=j3,
                            j4_deg=j4,
                            j5_deg=j5,
                            j6_deg=j6,
                        )
                        corners.append(pose)
    result["coupled_limit_corners"] = corners

    generator = random.Random(1513)
    random_poses = []
    for _ in range(150):
        random_poses.append(
            {
                "j1_deg": generator.uniform(-180.0, 180.0),
                "j2_deg": generator.uniform(-170.0, 170.0),
                "j3_deg": generator.uniform(-170.0, 170.0),
                "j4_deg": generator.uniform(-116.0, 159.0),
                "j5_deg": generator.uniform(-70.6, 151.2),
                "j6_deg": generator.uniform(-180.0, 180.0),
            }
        )
    result["coupled_random_seed_1513"] = random_poses
    return result


def rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / np.linalg.norm(axis)
    skew = np.array(
        [[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]]
    )
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def geom_name(model: mujoco.MjModel, geom_id: int) -> str:
    return mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom_{geom_id}"


def proxy_from_geom(name: str) -> str:
    if not name.startswith("collision__"):
        return name
    parts = name.split("__")
    return parts[2] if len(parts) >= 4 else name


def active_proxy_pairs(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    allowed_pairs: set[tuple[str, str]] | None = None,
) -> list[list[str]]:
    pairs = set()
    for index in range(data.ncon):
        contact = data.contact[index]
        first = proxy_from_geom(geom_name(model, int(contact.geom1)))
        second = proxy_from_geom(geom_name(model, int(contact.geom2)))
        if first == second:
            continue
        pair = tuple(sorted((first, second)))
        if allowed_pairs is not None and pair not in allowed_pairs:
            continue
        pairs.add(pair)
    return [list(pair) for pair in sorted(pairs)]


def apply_pose(model: mujoco.MjModel, data: mujoco.MjData, pose: dict[str, float]) -> None:
    for index, joint_name in enumerate(("J1", "J2", "J3", "J4", "J5", "J6"), 1):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        qadr = int(model.jnt_qposadr[joint_id])
        data.qpos[qadr] = math.radians(float(pose[f"j{index}_deg"]))
    mujoco.mj_forward(model, data)


def validate_frames_axes_signs_limits(model, data, axes) -> dict:
    frames = axes["frames_at_mechanical_zero"]
    source_joints = {row["name"]: row for row in axes["joints"]}
    base_rotation = frame_matrix(frames["base_link"])
    base_origin = np.asarray(frames["base_link"]["origin_world_mm"], dtype=float)
    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)

    link_rows = {}
    for link in LINKS:
        reference_name = "link6" if link == "gripper" else link
        frame = frames[reference_name]
        expected_pos = base_rotation.T @ (
            (np.asarray(frame["origin_world_mm"], dtype=float) - base_origin) * 0.001
        )
        expected_rot = base_rotation.T @ frame_matrix(frame)
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, link)
        actual_pos = data.xpos[body_id].copy()
        actual_rot = data.xmat[body_id].reshape(3, 3).copy()
        position_error = float(np.linalg.norm(actual_pos - expected_pos))
        rotation_error = float(np.max(np.abs(actual_rot - expected_rot)))
        link_rows[link] = {
            "position_error_m": position_error,
            "max_rotation_matrix_error": rotation_error,
            "pass": position_error <= FRAME_POSITION_TOLERANCE_M
            and rotation_error <= FRAME_ROTATION_TOLERANCE,
        }

    joint_rows = {}
    for joint_name, source_name in JOINT_SOURCE.items():
        source = source_joints[source_name]
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        expected_anchor = base_rotation.T @ (
            (np.asarray(source["joint_origin_world_mm"], dtype=float) - base_origin) * 0.001
        )
        expected_axis = base_rotation.T @ np.asarray(source["axis_world_at_zero"], dtype=float)
        expected_axis /= np.linalg.norm(expected_axis)
        anchor_error = float(np.linalg.norm(data.xanchor[joint_id] - expected_anchor))
        axis_error = float(np.linalg.norm(data.xaxis[joint_id] - expected_axis))

        site_id = mujoco.mj_name2id(
            model,
            mujoco.mjtObj.mjOBJ_SITE,
            f"{joint_name}_positive_witness",
        )
        zero_site = data.site_xpos[site_id].copy()
        qadr = int(model.jnt_qposadr[joint_id])
        delta = math.radians(0.5)
        data.qpos[qadr] = delta
        mujoco.mj_forward(model, data)
        actual_site = data.site_xpos[site_id].copy()
        expected_site = expected_anchor + rodrigues(expected_axis, delta) @ (
            zero_site - expected_anchor
        )
        sign_error = float(np.linalg.norm(actual_site - expected_site))
        data.qpos[qadr] = 0.0
        mujoco.mj_forward(model, data)

        expected_limited = bool(source["mujoco_mjcf"]["joint_limited"])
        actual_limited = bool(model.jnt_limited[joint_id])
        if expected_limited:
            expected_range = np.asarray(source["mujoco_mjcf"]["joint_range_rad"], dtype=float)
            actual_range = model.jnt_range[joint_id].copy()
            limit_error = float(np.max(np.abs(expected_range - actual_range)))
        else:
            expected_range = None
            actual_range = None
            limit_error = 0.0
        joint_rows[joint_name] = {
            "source_joint": source_name,
            "anchor_error_m": anchor_error,
            "axis_vector_error": axis_error,
            "positive_direction_witness_error_m": sign_error,
            "expected_limited": expected_limited,
            "actual_limited": actual_limited,
            "expected_range_rad": None if expected_range is None else expected_range.tolist(),
            "actual_range_rad": None if actual_range is None else actual_range.tolist(),
            "limit_error_rad": limit_error,
            "pass": anchor_error <= FRAME_POSITION_TOLERANCE_M
            and axis_error <= AXIS_TOLERANCE
            and sign_error <= POSITIVE_SIGN_TOLERANCE_M
            and expected_limited == actual_limited
            and limit_error <= LIMIT_TOLERANCE_RAD,
        }
    return {
        "links": link_rows,
        "joints": joint_rows,
        "all_links_pass": all(row["pass"] for row in link_rows.values()),
        "all_joints_pass": all(row["pass"] for row in joint_rows.values()),
    }


def validate_visual_bounds(source, runtime) -> dict:
    rows = {}
    for link in LINKS:
        source_bounds = np.asarray(
            [
                source["links"][link]["visual_bounds_local"]["min_mm"],
                source["links"][link]["visual_bounds_local"]["max_mm"],
            ],
            dtype=float,
        ) * 0.001
        runtime_bounds = np.asarray(runtime["visual"][link]["bounds_m"], dtype=float)
        error = float(np.max(np.abs(source_bounds - runtime_bounds)))
        rows[link] = {
            "max_bounds_error_m": error,
            "source_size_m": (source_bounds[1] - source_bounds[0]).tolist(),
            "runtime_size_m": (runtime_bounds[1] - runtime_bounds[0]).tolist(),
            "pass": error <= VISUAL_BOUNDS_TOLERANCE_M,
        }
    return {"links": rows, "all_pass": all(row["pass"] for row in rows.values())}


def validate_collision_regression(model, data, reference, pair_contract) -> dict:
    pose_sets = build_pose_sets()
    reference_categories = reference["sampled_collision_audit"]["categories"]
    full_pairs = {tuple(sorted(pair)) for pair in pair_contract["runtime_full_pairs"]}
    changed_terminal_pairs = {
        tuple(sorted(pair)) for pair in pair_contract["v15_13_changed_terminal_pairs"]
    }
    apply_pose(model, data, zero_pose())
    zero_active_pairs = active_proxy_pairs(model, data, full_pairs)
    rows = {}
    total_poses = 0
    for category, poses in pose_sets.items():
        actual_collisions = []
        for pose_index, pose in enumerate(poses):
            apply_pose(model, data, pose)
            # This exactly mirrors the V15.13 deep audit scope: the 503-pose
            # delta regression retests pairs involving the changed J6 output,
            # gripper or camera.  Unchanged upstream pairs inherit the prior
            # immutable audit, while the saved zero evaluates the full matrix.
            pairs = active_proxy_pairs(model, data, changed_terminal_pairs)
            if pairs:
                actual_collisions.append(
                    {
                        "pose_index": pose_index,
                        "angles_deg": pose,
                        "active_pairs": pairs,
                    }
                )
        expected_indices = {
            int(row["pose_index"]) for row in reference_categories[category]["collisions"]
        }
        actual_indices = {int(row["pose_index"]) for row in actual_collisions}
        rows[category] = {
            "pose_count": len(poses),
            "expected_collision_indices": sorted(expected_indices),
            "actual_collision_indices": sorted(actual_indices),
            "false_positive_indices": sorted(actual_indices - expected_indices),
            "false_negative_indices": sorted(expected_indices - actual_indices),
            "exact_collision_boolean_match": actual_indices == expected_indices,
            "collisions": actual_collisions,
        }
        total_poses += len(poses)
        print(
            f"collision {category}: expected={sorted(expected_indices)} "
            f"actual={sorted(actual_indices)}"
        )
    apply_pose(model, data, zero_pose())
    return {
        "total_pose_count": total_poses,
        "runtime_full_pair_count": len(full_pairs),
        "changed_terminal_pair_count": len(changed_terminal_pairs),
        "saved_zero_full_matrix_active_pairs": zero_active_pairs,
        "saved_zero_full_matrix_clear": not zero_active_pairs,
        "categories": rows,
        "all_503_pose_collision_booleans_match": all(
            row["exact_collision_boolean_match"] for row in rows.values()
        ),
    }


def main() -> None:
    axes = json.loads(AXIS_JSON.read_text(encoding="utf-8"))
    source_meshes = json.loads(SOURCE_MESH_MANIFEST.read_text(encoding="utf-8"))
    runtime_meshes = json.loads(RUNTIME_MESH_MANIFEST.read_text(encoding="utf-8"))
    reference = json.loads(REFERENCE_QA.read_text(encoding="utf-8"))
    pair_contract = json.loads(PAIR_CONTRACT.read_text(encoding="utf-8"))
    model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    data = mujoco.MjData(model)

    topology = {
        "nq": int(model.nq),
        "nv": int(model.nv),
        "joint_names": [
            mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_JOINT, index)
            for index in range(model.njnt)
        ],
        "expected_joint_names": ["J1", "J2", "J3", "J4", "J5", "J6"],
    }
    topology["pass"] = topology["nq"] == 6 and topology["joint_names"] == topology["expected_joint_names"]
    frames_axes_limits = validate_frames_axes_signs_limits(model, data, axes)
    visual_bounds = validate_visual_bounds(source_meshes, runtime_meshes)
    collision = validate_collision_regression(model, data, reference, pair_contract)

    passed = (
        topology["pass"]
        and frames_axes_limits["all_links_pass"]
        and frames_axes_limits["all_joints_pass"]
        and visual_bounds["all_pass"]
        and collision["saved_zero_full_matrix_clear"]
        and collision["all_503_pose_collision_booleans_match"]
    )
    report = {
        "status": "PASS" if passed else "FAIL",
        "revision": "V15.13-MuJoCo-empty-load-kinematic-v1",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "mujoco_version": mujoco.__version__,
        "model": str(MODEL_XML),
        "topology": topology,
        "frame_axis_sign_limit_validation": frames_axes_limits,
        "visual_bounds_validation": visual_bounds,
        "collision_regression": collision,
        "physics_scope": {
            "empty_load_kinematics_only": True,
            "mass_com_inertia_verified": False,
            "effort_velocity_damping_verified": False,
            "dynamics_use_forbidden": True,
        },
    }
    OUTPUT_QA.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"], "qa": str(OUTPUT_QA)}))
    if not passed:
        raise SystemExit(2)


if __name__ == "__main__":
    main()

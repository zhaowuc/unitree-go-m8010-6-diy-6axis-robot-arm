from __future__ import annotations

"""Deep per-axis V15.13 motion, mesh-follow, limit and penetration QA."""

import json
import math
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np

from kinematic_guard import KinematicGuard


ROOT = Path(__file__).resolve().parent
MODEL_XML = ROOT / "go_m8010_arm_v15_13_kinematic.xml"
AXIS_JSON = ROOT / "V15_13_真实关节轴与相机上置零位.json"
OUTPUT_QA = ROOT / "QA_MUJOCO_V15_13_逐轴运动语义与地面复核.json"

JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")
LINKS = ("base_link", "link1", "link2", "link3", "link4", "link5", "link6", "gripper")
CHILD_INDEX = {name: index for index, name in enumerate(JOINTS, 1)}
SOURCE_JOINT = {"J1": "J1", "J2": "J2A", "J3": "J3", "J4": "J4", "J5": "J5", "J6": "J6"}
SCAN = {
    "J1": (-180.0, 180.0, 5.0),
    "J2": (-170.0, 170.0, 5.0),
    "J3": (-170.0, 170.0, 5.0),
    "J4": (-116.0, 159.0, 5.0),
    "J5": (-70.6, 151.2, 5.0),
    "J6": (-180.0, 180.0, 5.0),
}

LINE_TOLERANCE_MM = 1.0e-4
AXIS_ALIGNMENT_TOLERANCE = 1.0e-10
TRANSFORM_TOLERANCE_M = 2.0e-9
ROTATION_TOLERANCE = 2.0e-9
LIMIT_TOLERANCE_RAD = 1.0e-12


def frame_matrix(frame: dict) -> np.ndarray:
    return np.column_stack(
        (
            frame["x_axis_world_at_zero"],
            frame["y_axis_world_at_zero"],
            frame["z_axis_world_at_zero"],
        )
    ).astype(float)


def rodrigues(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = np.asarray(axis, dtype=float)
    axis /= np.linalg.norm(axis)
    skew = np.array(
        [[0.0, -axis[2], axis[1]], [axis[2], 0.0, -axis[0]], [-axis[1], axis[0], 0.0]]
    )
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def inclusive_range(start: float, stop: float, step: float) -> list[float]:
    values = []
    current = float(start)
    while current <= stop + 1.0e-9:
        values.append(round(current, 10))
        current += step
    if abs(values[-1] - stop) > 1.0e-9:
        values.append(float(stop))
    return values


def set_pose(model: mujoco.MjModel, data: mujoco.MjData, values_deg) -> None:
    data.qpos[:] = 0.0
    for name, value in zip(JOINTS, values_deg):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        data.qpos[int(model.jnt_qposadr[joint_id])] = math.radians(float(value))
    mujoco.mj_forward(model, data)


def state(model: mujoco.MjModel, data: mujoco.MjData) -> dict:
    bodies = {}
    for name in LINKS:
        body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, name)
        bodies[name] = {
            "position": data.xpos[body_id].copy(),
            "rotation": data.xmat[body_id].reshape(3, 3).copy(),
        }
    geoms = {}
    for geom_id in range(model.ngeom):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_GEOM, geom_id) or f"geom_{geom_id}"
        if not name.startswith("visual__"):
            continue
        body_id = int(model.geom_bodyid[geom_id])
        body_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, body_id)
        geoms[name] = {
            "body": body_name,
            "position": data.geom_xpos[geom_id].copy(),
            "rotation": data.geom_xmat[geom_id].reshape(3, 3).copy(),
        }
    return {"bodies": bodies, "geoms": geoms}


def rotation_angle_deg(first: np.ndarray, second: np.ndarray) -> float:
    relative = second @ first.T
    cosine = float(np.clip((np.trace(relative) - 1.0) * 0.5, -1.0, 1.0))
    return math.degrees(math.acos(cosine))


def validate_motor_axis_lines(model, data, axes) -> dict:
    frames = axes["frames_at_mechanical_zero"]
    base_rotation = frame_matrix(frames["base_link"])
    base_origin_mm = np.asarray(frames["base_link"]["origin_world_mm"], dtype=float)
    source_rows = axes["joints"]
    set_pose(model, data, [0.0] * 6)
    rows = {}
    for joint_name in JOINTS:
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        anchor = data.xanchor[joint_id].copy()
        axis = data.xaxis[joint_id].copy()
        axis /= np.linalg.norm(axis)
        witnesses = []
        for source in source_rows:
            if source.get("kinematic_dof") != joint_name:
                continue
            witness = source["motor_output_flange_witness"]
            center = base_rotation.T @ (
                (np.asarray(witness["center_world_mm"], dtype=float) - base_origin_mm) * 0.001
            )
            source_axis = base_rotation.T @ np.asarray(witness["axis_world_undirected"], dtype=float)
            source_axis /= np.linalg.norm(source_axis)
            offset = center - anchor
            residual_mm = float(np.linalg.norm(offset - axis * np.dot(offset, axis)) * 1000.0)
            alignment = abs(float(np.dot(axis, source_axis)))
            witnesses.append(
                {
                    "source_joint": source["name"],
                    "motor_object": witness["object"],
                    "output_flange_center_model_m": center.tolist(),
                    "joint_axis_anchor_model_m": anchor.tolist(),
                    "axis_line_residual_mm": residual_mm,
                    "axis_alignment_abs_dot": alignment,
                    "axial_station_from_joint_anchor_mm": float(np.dot(offset, axis) * 1000.0),
                    "pass": residual_mm <= LINE_TOLERANCE_MM
                    and 1.0 - alignment <= AXIS_ALIGNMENT_TOLERANCE,
                }
            )
        rows[joint_name] = {
            "witnesses": witnesses,
            "maximum_axis_line_residual_mm": max(row["axis_line_residual_mm"] for row in witnesses),
            "minimum_axis_alignment_abs_dot": min(row["axis_alignment_abs_dot"] for row in witnesses),
            "pass": bool(witnesses) and all(row["pass"] for row in witnesses),
        }
    return {"joints": rows, "all_pass": all(row["pass"] for row in rows.values())}


def validate_positive_direction_and_following(model, data) -> dict:
    rows = {}
    delta_deg = 7.0
    delta_rad = math.radians(delta_deg)
    for joint_name in JOINTS:
        set_pose(model, data, [0.0] * 6)
        zero = state(model, data)
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        anchor = data.xanchor[joint_id].copy()
        axis = data.xaxis[joint_id].copy()
        axis /= np.linalg.norm(axis)
        witness_id = mujoco.mj_name2id(
            model, mujoco.mjtObj.mjOBJ_SITE, f"{joint_name}_positive_witness"
        )
        witness_zero = data.site_xpos[witness_id].copy()
        rotation = rodrigues(axis, delta_rad)
        values = [0.0] * 6
        values[JOINTS.index(joint_name)] = delta_deg
        set_pose(model, data, values)
        moved = state(model, data)
        witness_actual = data.site_xpos[witness_id].copy()
        witness_expected = anchor + rotation @ (witness_zero - anchor)
        sign_error = float(np.linalg.norm(witness_actual - witness_expected))

        first_moving = CHILD_INDEX[joint_name]
        upstream = list(LINKS[:first_moving])
        downstream = list(LINKS[first_moving:])
        upstream_position_error = 0.0
        upstream_rotation_error = 0.0
        downstream_position_error = 0.0
        downstream_rotation_error = 0.0
        downstream_motion = {}
        for link in upstream:
            upstream_position_error = max(
                upstream_position_error,
                float(np.linalg.norm(moved["bodies"][link]["position"] - zero["bodies"][link]["position"])),
            )
            upstream_rotation_error = max(
                upstream_rotation_error,
                float(np.max(np.abs(moved["bodies"][link]["rotation"] - zero["bodies"][link]["rotation"]))),
            )
        for link in downstream:
            p0 = zero["bodies"][link]["position"]
            r0 = zero["bodies"][link]["rotation"]
            expected_position = anchor + rotation @ (p0 - anchor)
            expected_rotation = rotation @ r0
            downstream_position_error = max(
                downstream_position_error,
                float(np.linalg.norm(moved["bodies"][link]["position"] - expected_position)),
            )
            downstream_rotation_error = max(
                downstream_rotation_error,
                float(np.max(np.abs(moved["bodies"][link]["rotation"] - expected_rotation))),
            )
            downstream_motion[link] = {
                "origin_displacement_mm": float(
                    np.linalg.norm(moved["bodies"][link]["position"] - p0) * 1000.0
                ),
                "orientation_change_deg": rotation_angle_deg(r0, moved["bodies"][link]["rotation"]),
            }

        visual_position_error = 0.0
        visual_rotation_error = 0.0
        visual_local_position_drift = 0.0
        visual_local_rotation_drift = 0.0
        visual_count = 0
        for name, geom_zero in zero["geoms"].items():
            geom_moved = moved["geoms"][name]
            body_name = geom_zero["body"]
            if body_name in downstream:
                expected_position = anchor + rotation @ (geom_zero["position"] - anchor)
                expected_rotation = rotation @ geom_zero["rotation"]
            else:
                expected_position = geom_zero["position"]
                expected_rotation = geom_zero["rotation"]
            visual_position_error = max(
                visual_position_error, float(np.linalg.norm(geom_moved["position"] - expected_position))
            )
            visual_rotation_error = max(
                visual_rotation_error,
                float(np.max(np.abs(geom_moved["rotation"] - expected_rotation))),
            )
            body_zero = zero["bodies"][body_name]
            body_moved = moved["bodies"][body_name]
            local_position_zero = body_zero["rotation"].T @ (geom_zero["position"] - body_zero["position"])
            local_position_moved = body_moved["rotation"].T @ (
                geom_moved["position"] - body_moved["position"]
            )
            local_rotation_zero = body_zero["rotation"].T @ geom_zero["rotation"]
            local_rotation_moved = body_moved["rotation"].T @ geom_moved["rotation"]
            visual_local_position_drift = max(
                visual_local_position_drift,
                float(np.linalg.norm(local_position_moved - local_position_zero)),
            )
            visual_local_rotation_drift = max(
                visual_local_rotation_drift,
                float(np.max(np.abs(local_rotation_moved - local_rotation_zero))),
            )
            visual_count += 1

        pass_row = (
            sign_error <= TRANSFORM_TOLERANCE_M
            and upstream_position_error <= TRANSFORM_TOLERANCE_M
            and upstream_rotation_error <= ROTATION_TOLERANCE
            and downstream_position_error <= TRANSFORM_TOLERANCE_M
            and downstream_rotation_error <= ROTATION_TOLERANCE
            and visual_position_error <= TRANSFORM_TOLERANCE_M
            and visual_rotation_error <= ROTATION_TOLERANCE
            and visual_local_position_drift <= TRANSFORM_TOLERANCE_M
            and visual_local_rotation_drift <= ROTATION_TOLERANCE
        )
        rows[joint_name] = {
            "test_positive_rotation_deg": delta_deg,
            "positive_direction_witness_error_m": sign_error,
            "positive_direction_rule": "right-hand rule about the measured directed axis",
            "upstream_links_expected_fixed": upstream,
            "downstream_links_expected_follow": downstream,
            "upstream_max_position_error_m": upstream_position_error,
            "upstream_max_rotation_matrix_error": upstream_rotation_error,
            "downstream_max_position_error_m": downstream_position_error,
            "downstream_max_rotation_matrix_error": downstream_rotation_error,
            "downstream_observed_motion": downstream_motion,
            "visual_stl_geom_count_checked": visual_count,
            "visual_world_position_error_m": visual_position_error,
            "visual_world_rotation_matrix_error": visual_rotation_error,
            "visual_body_local_position_drift_m": visual_local_position_drift,
            "visual_body_local_rotation_matrix_drift": visual_local_rotation_drift,
            "pass": pass_row,
        }
    return {"joints": rows, "all_pass": all(row["pass"] for row in rows.values())}


def validate_limits(model, axes, guard: KinematicGuard) -> dict:
    sources = {row["name"]: row for row in axes["joints"]}
    rows = {}
    for index, joint_name in enumerate(JOINTS):
        source = sources[SOURCE_JOINT[joint_name]]["mujoco_mjcf"]
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
        expected_limited = bool(source["joint_limited"])
        actual_limited = bool(model.jnt_limited[joint_id])
        values = [0.0] * 6
        if expected_limited:
            expected_range = np.asarray(source["joint_range_rad"], dtype=float)
            actual_range = model.jnt_range[joint_id].copy()
            range_error = float(np.max(np.abs(expected_range - actual_range)))
            expected_deg = np.degrees(expected_range)
            boundary_results = []
            outside_results = []
            for value in expected_deg:
                values[index] = float(value)
                result = guard.check_pose_deg(values)
                boundary_results.append(
                    {"value_deg": float(value), "reason": result["reason"], "position_limit_accepted": result["reason"] != "position_limit"}
                )
            for value in (expected_deg[0] - 0.01, expected_deg[1] + 0.01):
                values[index] = float(value)
                result = guard.check_pose_deg(values)
                outside_results.append(
                    {"value_deg": float(value), "reason": result["reason"], "position_limit_rejected": result["reason"] == "position_limit"}
                )
            pass_row = (
                actual_limited
                and range_error <= LIMIT_TOLERANCE_RAD
                and all(row["position_limit_accepted"] for row in boundary_results)
                and all(row["position_limit_rejected"] for row in outside_results)
            )
            row = {
                "position_limited": True,
                "expected_range_deg": expected_deg.tolist(),
                "actual_range_deg": np.degrees(actual_range).tolist(),
                "range_error_rad": range_error,
                "boundary_checks": boundary_results,
                "outside_by_0_01_deg_checks": outside_results,
                "pass": pass_row,
            }
        else:
            continuous_results = []
            for value in (-720.0, 720.0):
                values[index] = value
                result = guard.check_pose_deg(values)
                continuous_results.append(
                    {"value_deg": value, "reason": result["reason"], "position_limit_accepted": result["reason"] != "position_limit"}
                )
            row = {
                "position_limited": False,
                "representative_accumulated_turn_checks": continuous_results,
                "note": "J1 is continuous in the measured CAD contract; cable/slip-ring accumulated-turn limit remains hardware commissioning data.",
                "pass": not actual_limited and all(item["position_limit_accepted"] for item in continuous_results),
            }
        rows[joint_name] = row
    return {"joints": rows, "all_pass": all(row["pass"] for row in rows.values())}


def validate_penetration_scans(guard: KinematicGuard) -> dict:
    rows = {}
    total_samples = 0
    total_ground_unsafe = 0
    total_self_unsafe = 0
    guard_consistent = True
    for index, joint_name in enumerate(JOINTS):
        lower, upper, step = SCAN[joint_name]
        ground_rows = []
        self_rows = []
        min_distance = None
        values_list = inclusive_range(lower, upper, step)
        for value in values_list:
            pose = [0.0] * 6
            pose[index] = value
            result = guard.check_pose_deg(pose)
            total_samples += 1
            contacts = result["contacts"]
            if contacts and result["safe"]:
                guard_consistent = False
            if contacts:
                local_min = min(float(contact["distance_m"]) for contact in contacts)
                min_distance = local_min if min_distance is None else min(min_distance, local_min)
            ground_contacts = [row for row in contacts if row["contact_class"] == "ground"]
            self_contacts = [row for row in contacts if row["contact_class"] == "self_collision"]
            if ground_contacts:
                ground_rows.append(
                    {
                        "angle_deg": value,
                        "minimum_contact_distance_m": min(float(row["distance_m"]) for row in ground_contacts),
                        "contact_count": len(ground_contacts),
                    }
                )
            if self_contacts:
                self_rows.append(
                    {
                        "angle_deg": value,
                        "minimum_contact_distance_m": min(float(row["distance_m"]) for row in self_contacts),
                        "proxy_pairs": sorted({tuple(row["proxy_pair"]) for row in self_contacts}),
                    }
                )
        total_ground_unsafe += len(ground_rows)
        total_self_unsafe += len(self_rows)
        unsafe_angles = sorted(
            {row["angle_deg"] for row in ground_rows} | {row["angle_deg"] for row in self_rows}
        )
        swept_probe = None
        if unsafe_angles:
            probe_angle = min(unsafe_angles, key=abs)
            start = [0.0] * 6
            target = [0.0] * 6
            target[index] = probe_angle
            probe_result = guard.check_swept_deg(start, target, max_step_deg=2.5)
            swept_probe = {
                "target_angle_deg": probe_angle,
                "swept_safe": bool(probe_result.get("swept_safe", probe_result["safe"])),
                "reason": probe_result["reason"],
                "first_unsafe_step": probe_result.get("first_unsafe_step"),
                "step_count": probe_result.get("step_count"),
                "pass": not bool(probe_result.get("swept_safe", probe_result["safe"])),
            }
        pass_row = guard_consistent and (swept_probe is None or swept_probe["pass"])
        rows[joint_name] = {
            "scan_range_deg": [lower, upper],
            "scan_step_deg": step,
            "sample_count": len(values_list),
            "ground_collision_samples": ground_rows,
            "self_collision_samples": self_rows,
            "collision_prohibited_sample_count": len(unsafe_angles),
            "swept_guard_probe": swept_probe,
            "minimum_active_contact_distance_m": min_distance,
            "pass": pass_row,
            "interpretation": (
                "Mechanical-limit angles remain the hardware limits; every sampled floor/self penetration is fail-closed and prohibited by the runtime guard."
                if unsafe_angles
                else "No active ground or self penetration in this independent-axis scan."
            ),
        }
    zero_result = guard.check_pose_deg([0.0] * 6)
    return {
        "joints": rows,
        "total_sample_count": total_samples,
        "ground_collision_sample_count": total_ground_unsafe,
        "detected_self_collision_sample_count": total_self_unsafe,
        "zero_pose": zero_result,
        "guard_fail_closed_consistent": guard_consistent,
        "all_pass": (
            zero_result["safe"]
            and guard_consistent
            and all(row["pass"] for row in rows.values())
        ),
    }


def main() -> None:
    axes = json.loads(AXIS_JSON.read_text(encoding="utf-8"))
    model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    data = mujoco.MjData(model)
    guard = KinematicGuard(MODEL_XML)
    motor = validate_motor_axis_lines(model, data, axes)
    motion = validate_positive_direction_and_following(model, data)
    limits = validate_limits(model, axes, guard)
    penetration = validate_penetration_scans(guard)
    passed = motor["all_pass"] and motion["all_pass"] and limits["all_pass"] and penetration["all_pass"]
    report = {
        "schema": "go-m8010-arm-v15.13-per-axis-motion-semantics-ground-qa/1.0",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": "PASS" if passed else "FAIL",
        "model": MODEL_XML.name,
        "ground_policy": {
            "z_m": -0.09,
            "basis": "exact base_link visual minimum Z",
            "physical_collision": True,
            "fixed_base_excluded": True,
            "moving_links_active": True,
        },
        "motor_output_axis_line_validation": motor,
        "positive_direction_link_follow_and_stl_validation": motion,
        "mechanical_limit_validation": limits,
        "independent_axis_penetration_validation": penetration,
        "acceptance": {
            "axis_center_on_every_motor_output_flange_line": motor["all_pass"],
            "positive_direction_right_hand_rule": motion["all_pass"],
            "upstream_fixed_downstream_complete_follow": motion["all_pass"],
            "visual_stl_no_flyaway": motion["all_pass"],
            "mechanical_limits_exact_and_fail_closed": limits["all_pass"],
            "zero_clear_and_all_sampled_penetrations_are_fail_closed": penetration["all_pass"],
        },
    }
    OUTPUT_QA.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"], "qa": str(OUTPUT_QA)}, ensure_ascii=False))
    raise SystemExit(0 if passed else 2)


if __name__ == "__main__":
    main()

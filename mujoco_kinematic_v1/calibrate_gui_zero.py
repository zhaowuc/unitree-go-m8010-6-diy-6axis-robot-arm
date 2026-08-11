from __future__ import annotations

"""Solve and verify the aligned operational GUI zero from measured MuJoCo geometry."""

import json
import math
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import numpy as np
from scipy.optimize import root_scalar

from kinematic_guard import KinematicGuard


ROOT = Path(__file__).resolve().parent
MODEL_XML = ROOT / "go_m8010_arm_v15_13_kinematic.xml"
RUNTIME_MESH_MANIFEST = ROOT / "runtime_mesh_manifest.json"
DEFAULT_ZERO_CONFIG = ROOT / "gui_default_zero.json"
OUTPUT_QA = ROOT / "QA_MUJOCO_V15_13_GUI零位几何校准.json"

JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")
RECOVERED_USER_ZERO_DEG = np.array([-134.9, 148.7, -76.6, -88.7, 47.1, 0.9])
CAD_MECHANICAL_ZERO_DEG = np.zeros(6)
ROUNDED_CANDIDATE_DEG = np.array([-135.0, 150.0, -75.0, -90.0, 45.0, 0.0])
FIXED_NOMINAL_DEG = {
    "J1": -135.0,
    "J2": 150.0,
    "J4": -90.0,
    "J5": 45.0,
}
TARGET_FOREARM_ELEVATION_DEG = 90.0
TARGET_CAMERA_LONG_AXIS_ELEVATION_DEG = 0.0


def set_pose(model: mujoco.MjModel, data: mujoco.MjData, values_deg: np.ndarray) -> None:
    data.qpos[:] = np.radians(values_deg)
    mujoco.mj_forward(model, data)


def joint_anchors(model: mujoco.MjModel, data: mujoco.MjData) -> list[np.ndarray]:
    return [
        data.xanchor[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)].copy()
        for name in JOINTS
    ]


def elevation_deg(vector: np.ndarray) -> float:
    vector = np.asarray(vector, dtype=float)
    return math.degrees(math.asin(float(vector[2] / np.linalg.norm(vector))))


def camera_rotation(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    body_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
    return data.xmat[body_id].reshape(3, 3).copy()


def camera_metrics(rotation: np.ndarray) -> dict:
    local_plus_x = rotation[:, 0]
    local_plus_y = rotation[:, 1]
    long_axis_elevation = math.degrees(
        math.asin(float(np.clip(local_plus_y[2], -1.0, 1.0)))
    )
    long_axis_yaw = math.degrees(
        math.atan2(float(local_plus_y[1]), float(local_plus_y[0]))
    )
    return {
        "camera_local_plus_x_world_vector": [float(value) for value in local_plus_x],
        "camera_long_plus_y_world_vector": [float(value) for value in local_plus_y],
        "camera_long_plus_y_elevation_deg": long_axis_elevation,
        "camera_long_plus_y_yaw_deg": long_axis_yaw,
    }


def solve_zero(model: mujoco.MjModel, data: mujoco.MjData) -> np.ndarray:
    # Keep the four already intentional nominal joints unchanged.  Only J3 is
    # solved from the measured J3/J4 anchors and J6 from the imported camera
    # proxy orientation.  This avoids changing the camera pitch/view direction.
    solution = ROUNDED_CANDIDATE_DEG.copy()

    # For J1=-135 deg, this is the radial direction of the J2/J3/J4 motion plane.
    radial = np.array([math.sqrt(0.5), math.sqrt(0.5), 0.0])

    def forearm_radial_residual(j3_deg: float) -> float:
        candidate = solution.copy()
        candidate[2] = j3_deg
        set_pose(model, data, candidate)
        anchors = joint_anchors(model, data)
        return float(np.dot(anchors[3] - anchors[2], radial))

    solution[2] = root_scalar(
        forearm_radial_residual, bracket=(-90.0, -60.0), xtol=1.0e-12
    ).root

    def camera_long_axis_vertical_component(j6_deg: float) -> float:
        candidate = solution.copy()
        candidate[5] = j6_deg
        set_pose(model, data, candidate)
        return float(camera_rotation(model, data)[2, 1])

    solution[5] = root_scalar(
        camera_long_axis_vertical_component,
        bracket=(-10.0, 10.0),
        xtol=1.0e-12,
    ).root
    return solution


def pose_metrics(model: mujoco.MjModel, data: mujoco.MjData, values_deg: np.ndarray) -> dict:
    set_pose(model, data, values_deg)
    anchors = joint_anchors(model, data)
    forearm = anchors[3] - anchors[2]
    return {
        "angles_deg": {name: float(value) for name, value in zip(JOINTS, values_deg)},
        "upper_arm_J2_to_J3_elevation_deg": elevation_deg(anchors[2] - anchors[1]),
        "forearm_J3_to_J4_vector_mm": [float(value * 1000.0) for value in forearm],
        "forearm_J3_to_J4_horizontal_offset_mm": float(
            np.linalg.norm(forearm[:2]) * 1000.0
        ),
        "forearm_J3_to_J4_elevation_deg": elevation_deg(forearm),
        "camera": camera_metrics(camera_rotation(model, data)),
    }


def joint_limit_margins_deg(model: mujoco.MjModel, values_deg: np.ndarray) -> dict:
    margins = {}
    for name, value in zip(JOINTS, values_deg):
        joint_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        if not bool(model.jnt_limited[joint_id]):
            margins[name] = {
                "limited": False,
                "allowed_deg": None,
                "lower_margin_deg": None,
                "upper_margin_deg": None,
                "nearest_margin_deg": None,
            }
            continue
        lower, upper = np.degrees(model.jnt_range[joint_id])
        margins[name] = {
            "limited": True,
            "allowed_deg": [float(lower), float(upper)],
            "lower_margin_deg": float(value - lower),
            "upper_margin_deg": float(upper - value),
            "nearest_margin_deg": float(min(value - lower, upper - value)),
        }
    return margins


def neighborhood_checks(guard: KinematicGuard, center_deg: np.ndarray) -> dict:
    rows = []
    for joint_index, joint_name in enumerate(JOINTS):
        for magnitude in (0.25, 0.5, 1.0, 2.0):
            for sign in (-1.0, 1.0):
                candidate = center_deg.copy()
                candidate[joint_index] += sign * magnitude
                result = guard.check_pose_deg(candidate)
                rows.append(
                    {
                        "joint": joint_name,
                        "delta_deg": sign * magnitude,
                        "safe": bool(result["safe"]),
                        "reason": result["reason"],
                    }
                )
    return {
        "safe": all(row["safe"] for row in rows),
        "sample_count": len(rows),
        "samples": rows,
    }


def main() -> None:
    runtime = json.loads(RUNTIME_MESH_MANIFEST.read_text(encoding="utf-8"))
    camera_member = next(
        row
        for row in runtime["collision"]["gripper"]["members"]
        if row["token"] == "Gemini_Pro_Camera_Collision_Proxy"
    )
    model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    data = mujoco.MjData(model)
    calibrated = solve_zero(model, data)
    guard = KinematicGuard(MODEL_XML, model=model, data=data)
    point_check = guard.check_pose_deg(calibrated)
    swept_from_previous = guard.check_swept_deg(
        RECOVERED_USER_ZERO_DEG, calibrated, max_step_deg=0.25
    )
    swept_from_cad = guard.check_swept_deg(
        CAD_MECHANICAL_ZERO_DEG, calibrated, max_step_deg=0.25
    )
    swept_from_rounded = guard.check_swept_deg(
        ROUNDED_CANDIDATE_DEG, calibrated, max_step_deg=0.25
    )
    neighbor_check = neighborhood_checks(guard, calibrated)
    before = pose_metrics(model, data, RECOVERED_USER_ZERO_DEG)
    rounded = pose_metrics(model, data, ROUNDED_CANDIDATE_DEG)
    after = pose_metrics(model, data, calibrated)

    camera = after["camera"]
    fixed_nominal_ok = all(
        abs(after["angles_deg"][name] - target) <= 1.0e-12
        for name, target in FIXED_NOMINAL_DEG.items()
    )
    passed = (
        point_check["safe"]
        and swept_from_previous["safe"]
        and swept_from_cad["safe"]
        and swept_from_rounded["safe"]
        and neighbor_check["safe"]
        and fixed_nominal_ok
        and abs(after["forearm_J3_to_J4_elevation_deg"] - 90.0) <= 1.0e-9
        and abs(camera["camera_long_plus_y_elevation_deg"]) <= 1.0e-9
    )

    report = {
        "schema": "go-m8010-arm-v15.13-gui-zero-geometric-calibration/1.0",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "status": "PASS" if passed else "FAIL",
        "policy": "Operational GUI zero only; measured CAD/MJCF/URDF joint references and hardware encoder zeros remain unchanged.",
        "camera_proxy_geometry": {
            "token": camera_member["token"],
            "source": camera_member["source"],
            "source_bounds_m": camera_member["source_bounds_m"],
            "local_axis_interpretation": "+Y is the 102.896 mm housing long axis used for level; existing +X/+Z pitch branch is preserved",
        },
        "targets": {
            "fixed_nominal_deg": FIXED_NOMINAL_DEG,
            "forearm_J3_to_J4_elevation_deg": TARGET_FOREARM_ELEVATION_DEG,
            "camera_long_plus_y_elevation_deg": TARGET_CAMERA_LONG_AXIS_ELEVATION_DEG,
            "preserve_camera_pitch_and_view_branch": True,
        },
        "before_recovered_user_zero": before,
        "rounded_candidate_not_used_as_final": rounded,
        "after_calibrated_gui_zero": after,
        "point_collision_check": point_check,
        "swept_from_previous_zero_check": swept_from_previous,
        "swept_from_cad_mechanical_zero_check": swept_from_cad,
        "swept_from_rounded_candidate_check": swept_from_rounded,
        "single_axis_plus_minus_2deg_neighborhood_check": neighbor_check,
        "joint_limit_margins_deg": joint_limit_margins_deg(model, calibrated),
    }
    OUTPUT_QA.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    if passed:
        config = {
            "schema": "go-m8010-arm-v15.13-gui-default-zero/1.2",
            "saved_at_utc": datetime.now(timezone.utc).isoformat(),
            "semantics": "Geometrically calibrated GUI startup posture and slider-relative zero only; CAD/MJCF/URDF/hardware encoder zeros are unchanged.",
            "calibration_qa": OUTPUT_QA.name,
            "cad_absolute_deg": {
                name: float(value) for name, value in zip(JOINTS, calibrated)
            },
        }
        DEFAULT_ZERO_CONFIG.write_text(
            json.dumps(config, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
    print(
        json.dumps(
            {
                "status": report["status"],
                "calibrated_deg": report["after_calibrated_gui_zero"]["angles_deg"],
                "qa": str(OUTPUT_QA),
                "config": str(DEFAULT_ZERO_CONFIG),
            },
            ensure_ascii=False,
        )
    )
    raise SystemExit(0 if passed else 2)


if __name__ == "__main__":
    main()

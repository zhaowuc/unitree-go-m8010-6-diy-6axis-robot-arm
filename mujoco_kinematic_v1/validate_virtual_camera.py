from __future__ import annotations

"""Validate the virtual ROS-optical to MuJoCo-renderer camera chain.

The validation target is injected into a temporary MJCF as fixed world
geometry.  It never changes the production model's collision or dynamics.
"""

import json
import math
import os
import tempfile
import xml.etree.ElementTree as ET
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
import numpy as np
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parent
MODEL_XML = ROOT / "go_m8010_arm_v15_13_kinematic.xml"
OUTPUT_IMAGE = ROOT / "previews" / "virtual_camera_orientation_validation.png"
OUTPUT_RAW_IMAGE = ROOT / "previews" / "virtual_camera_orientation_raw.png"
OUTPUT_REPORT = ROOT / "QA_MUJOCO_V15_13_虚拟相机坐标闭环.json"

GRIPPER_TO_CAMERA_LINK_M = np.array(
    [-0.038917046930682, 0.000061006900000, 0.019635428243040], dtype=float
)
GRIPPER_TO_CAMERA_LINK_QUAT_WXYZ = np.array([0.5, -0.5, 0.5, -0.5])
CAMERA_LINK_TO_SIM_OPTICAL_M = np.array([0.0, -0.00905, -0.0128])
CAMERA_LINK_TO_SIM_OPTICAL_QUAT_WXYZ = np.array(
    [math.sqrt(0.5), math.sqrt(0.5), 0.0, 0.0]
)
SIM_OPTICAL_TO_MUJOCO_CAMERA_QUAT_WXYZ = np.array([0.0, 1.0, 0.0, 0.0])

CAMERA_NAME = "sim_gemini_pro_renderer"
POSITION_TOL_M = 1e-12
ROTATION_TOL = 1e-12
MIN_MARKER_PIXELS = 80


def fmt(values: np.ndarray | list[float]) -> str:
    return " ".join(f"{float(value):.15g}" for value in values)


def quat_wxyz_to_matrix(quaternion: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(quaternion, dtype=float)
    norm = math.sqrt(w * w + x * x + y * y + z * z)
    w, x, y, z = (w / norm, x / norm, y / norm, z / norm)
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quat_wxyz(matrix: np.ndarray) -> np.ndarray:
    result = np.empty(4, dtype=float)
    mujoco.mju_mat2Quat(result, np.asarray(matrix, dtype=float).reshape(9))
    if result[0] < 0:
        result *= -1
    return result


def max_abs(left: np.ndarray, right: np.ndarray) -> float:
    return float(np.max(np.abs(np.asarray(left) - np.asarray(right))))


def object_id(model: mujoco.MjModel, object_type: mujoco.mjtObj, name: str) -> int:
    result = mujoco.mj_name2id(model, object_type, name)
    if result < 0:
        raise RuntimeError(f"Required MuJoCo object is missing: {name}")
    return int(result)


def add_validation_geom(body: ET.Element, name: str, **attributes: str) -> None:
    ET.SubElement(
        body,
        "geom",
        name=name,
        contype="0",
        conaffinity="0",
        group="5",
        mass="0",
        **attributes,
    )


def make_validation_model(optical_world_pos: np.ndarray, optical_world_rot: np.ndarray) -> Path:
    tree = ET.parse(MODEL_XML)
    root = tree.getroot()
    worldbody = root.find("worldbody")
    if worldbody is None:
        raise RuntimeError("MJCF has no worldbody")
    fixture = ET.SubElement(
        worldbody,
        "body",
        name="sim_camera_validation_world_fixture",
        pos=fmt(optical_world_pos),
        quat=fmt(matrix_to_quat_wxyz(optical_world_rot)),
    )

    # Front calibration board and an intentionally off-centre fiducial.
    add_validation_geom(
        fixture,
        "qa_front_plane",
        type="box",
        pos="0 0 0.90",
        size="0.34 0.25 0.005",
        rgba="0.68 0.70 0.74 1",
    )
    add_validation_geom(
        fixture,
        "qa_board_vertical",
        type="box",
        pos="0 0 0.893",
        size="0.004 0.22 0.002",
        rgba="0.08 0.08 0.09 1",
    )
    add_validation_geom(
        fixture,
        "qa_board_horizontal",
        type="box",
        pos="0 0 0.893",
        size="0.29 0.004 0.002",
        rgba="0.08 0.08 0.09 1",
    )
    add_validation_geom(
        fixture,
        "qa_board_corner_fiducial",
        type="box",
        pos="-0.285 -0.195 0.890",
        size="0.025 0.025 0.003",
        rgba="1 0.45 0.02 1",
    )

    # Four deliberately different objects prove image handedness.
    add_validation_geom(
        fixture,
        "qa_left_red_tall",
        type="box",
        pos="-0.20 0 0.70",
        size="0.030 0.075 0.030",
        rgba="0.95 0.05 0.04 1",
    )
    add_validation_geom(
        fixture,
        "qa_right_green_round",
        type="sphere",
        pos="0.20 0 0.70",
        size="0.055",
        rgba="0.05 0.90 0.16 1",
    )
    add_validation_geom(
        fixture,
        "qa_up_blue_wide",
        type="box",
        pos="0 -0.15 0.70",
        size="0.075 0.027 0.030",
        rgba="0.04 0.22 1 1",
    )
    add_validation_geom(
        fixture,
        "qa_down_yellow_small",
        type="sphere",
        pos="0 0.15 0.70",
        size="0.040",
        rgba="1 0.88 0.03 1",
    )
    add_validation_geom(
        fixture,
        "qa_rear_purple_guard",
        type="sphere",
        pos="0 0 -0.40",
        size="0.08",
        rgba="0.70 0.05 0.90 1",
    )

    handle = tempfile.NamedTemporaryFile(
        mode="wb", suffix=".xml", prefix="virtual_camera_qa_", dir=ROOT, delete=False
    )
    handle.close()
    target = Path(handle.name)
    ET.indent(root, space="  ")
    tree.write(target, encoding="utf-8", xml_declaration=True)
    return target


def segmentation_mask(segmentation: np.ndarray, geom_id: int) -> np.ndarray:
    geom_type = int(mujoco.mjtObj.mjOBJ_GEOM)
    return (
        ((segmentation[:, :, 0] == geom_id) & (segmentation[:, :, 1] == geom_type))
        | ((segmentation[:, :, 1] == geom_id) & (segmentation[:, :, 0] == geom_type))
    )


def render_orientation_qa(
    optical_world_pos: np.ndarray, optical_world_rot: np.ndarray
) -> dict:
    temporary_model = make_validation_model(optical_world_pos, optical_world_rot)
    try:
        model = mujoco.MjModel.from_xml_path(str(temporary_model))
        data = mujoco.MjData(model)
        data.qpos[:] = 0.0
        mujoco.mj_forward(model, data)

        option = mujoco.MjvOption()
        option.geomgroup[:] = 0
        option.geomgroup[5] = 1
        renderer = mujoco.Renderer(model, height=720, width=960)
        renderer.update_scene(data, camera=CAMERA_NAME, scene_option=option)
        rgb = renderer.render().copy()
        OUTPUT_RAW_IMAGE.parent.mkdir(parents=True, exist_ok=True)
        Image.fromarray(rgb).save(OUTPUT_RAW_IMAGE)

        renderer.enable_segmentation_rendering()
        renderer.update_scene(data, camera=CAMERA_NAME, scene_option=option)
        segmentation = renderer.render().copy()
        renderer.disable_segmentation_rendering()
        renderer.close()

        marker_names = {
            "left": "qa_left_red_tall",
            "right": "qa_right_green_round",
            "up": "qa_up_blue_wide",
            "down": "qa_down_yellow_small",
            "front_plane": "qa_front_plane",
            "rear": "qa_rear_purple_guard",
        }
        results: dict[str, dict] = {}
        for label, name in marker_names.items():
            geom_id = object_id(model, mujoco.mjtObj.mjOBJ_GEOM, name)
            mask = segmentation_mask(segmentation, geom_id)
            rows, columns = np.nonzero(mask)
            results[label] = {
                "geom": name,
                "pixel_count": int(mask.sum()),
                "centroid_xy_px": (
                    [float(columns.mean()), float(rows.mean())] if columns.size else None
                ),
            }

        visible = all(
            results[name]["pixel_count"] >= MIN_MARKER_PIXELS
            for name in ("left", "right", "up", "down", "front_plane")
        )
        rear_hidden = results["rear"]["pixel_count"] == 0
        horizontal_ok = (
            results["left"]["centroid_xy_px"][0]
            < results["right"]["centroid_xy_px"][0]
        )
        vertical_ok = (
            results["up"]["centroid_xy_px"][1]
            < results["down"]["centroid_xy_px"][1]
        )

        annotated = Image.fromarray(rgb)
        draw = ImageDraw.Draw(annotated)
        draw.rectangle((0, 0, 960, 46), fill=(18, 20, 27))
        draw.text(
            (14, 8),
            "SIMULATION ONLY | ROS optical -> MuJoCo renderer | synthetic FOVY 60 deg",
            fill=(255, 255, 255),
        )
        draw.text(
            (14, 27),
            "Expected: LEFT red | RIGHT green | UP blue | DOWN yellow | rear purple hidden",
            fill=(188, 220, 255),
        )
        for label in ("left", "right", "up", "down"):
            centre = results[label]["centroid_xy_px"]
            if centre is None:
                continue
            x, y = centre
            draw.ellipse((x - 5, y - 5, x + 5, y + 5), outline=(255, 255, 255), width=2)
            draw.text((x + 8, y - 8), label.upper(), fill=(255, 255, 255))
        annotated.save(OUTPUT_IMAGE)

        return {
            "raw_image": str(OUTPUT_RAW_IMAGE),
            "annotated_image": str(OUTPUT_IMAGE),
            "resolution_px": [960, 720],
            "synthetic_fovy_deg": 60.0,
            "markers": results,
            "left_right_order_pass": bool(horizontal_ok),
            "up_down_order_pass": bool(vertical_ok),
            "front_visible_pass": bool(visible),
            "rear_hidden_pass": bool(rear_hidden),
            "pass": bool(horizontal_ok and vertical_ok and visible and rear_hidden),
        }
    finally:
        temporary_model.unlink(missing_ok=True)


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    data = mujoco.MjData(model)

    gripper_id = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "gripper")
    camera_link_id = object_id(model, mujoco.mjtObj.mjOBJ_BODY, "camera_link")
    optical_id = object_id(
        model, mujoco.mjtObj.mjOBJ_BODY, "sim_camera_optical_frame"
    )
    renderer_camera_id = object_id(model, mujoco.mjtObj.mjOBJ_CAMERA, CAMERA_NAME)

    r_gc = quat_wxyz_to_matrix(GRIPPER_TO_CAMERA_LINK_QUAT_WXYZ)
    r_co = quat_wxyz_to_matrix(CAMERA_LINK_TO_SIM_OPTICAL_QUAT_WXYZ)
    r_om = quat_wxyz_to_matrix(SIM_OPTICAL_TO_MUJOCO_CAMERA_QUAT_WXYZ)
    expected_r_co = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], dtype=float)
    expected_r_om = np.diag([1.0, -1.0, -1.0])

    local_checks = {
        "camera_parent_is_gripper": int(model.body_parentid[camera_link_id]) == gripper_id,
        "optical_parent_is_camera_link": int(model.body_parentid[optical_id]) == camera_link_id,
        "renderer_parent_is_optical": int(model.cam_bodyid[renderer_camera_id]) == optical_id,
        "frozen_camera_position_error_m": max_abs(
            model.body_pos[camera_link_id], GRIPPER_TO_CAMERA_LINK_M
        ),
        "frozen_camera_rotation_error": max_abs(
            quat_wxyz_to_matrix(model.body_quat[camera_link_id]), r_gc
        ),
        "optical_position_error_m": max_abs(
            model.body_pos[optical_id], CAMERA_LINK_TO_SIM_OPTICAL_M
        ),
        "optical_rotation_error": max_abs(
            quat_wxyz_to_matrix(model.body_quat[optical_id]), r_co
        ),
        "renderer_position_error_m": max_abs(
            model.cam_pos[renderer_camera_id], np.zeros(3)
        ),
        "renderer_rotation_error": max_abs(
            quat_wxyz_to_matrix(model.cam_quat[renderer_camera_id]), r_om
        ),
    }
    local_checks["pass"] = bool(
        local_checks["camera_parent_is_gripper"]
        and local_checks["optical_parent_is_camera_link"]
        and local_checks["renderer_parent_is_optical"]
        and max(
            value
            for key, value in local_checks.items()
            if key.endswith("_error_m") or key.endswith("_error")
        )
        <= max(POSITION_TOL_M, ROTATION_TOL)
    )

    axis_checks = {
        "x_opt_in_camera_link": r_co[:, 0].tolist(),
        "y_opt_in_camera_link": r_co[:, 1].tolist(),
        "z_opt_in_camera_link": r_co[:, 2].tolist(),
        "x_mj_in_optical": r_om[:, 0].tolist(),
        "y_mj_in_optical": r_om[:, 1].tolist(),
        "z_mj_in_optical": r_om[:, 2].tolist(),
        "ros_optical_matrix_error": max_abs(r_co, expected_r_co),
        "mujoco_camera_matrix_error": max_abs(r_om, expected_r_om),
        "ros_optical_determinant": float(np.linalg.det(r_co)),
        "mujoco_camera_determinant": float(np.linalg.det(r_om)),
        "mujoco_view_direction_in_optical": (r_om @ [0.0, 0.0, -1.0]).tolist(),
        "mujoco_up_direction_in_optical": (r_om @ [0.0, 1.0, 0.0]).tolist(),
    }
    axis_checks["pass"] = bool(
        axis_checks["ros_optical_matrix_error"] <= ROTATION_TOL
        and axis_checks["mujoco_camera_matrix_error"] <= ROTATION_TOL
        and abs(axis_checks["ros_optical_determinant"] - 1.0) <= ROTATION_TOL
        and abs(axis_checks["mujoco_camera_determinant"] - 1.0) <= ROTATION_TOL
    )

    poses = {
        "zero": {},
        "J1_plus_20": {"J1": math.radians(20)},
        "J2_plus_20": {"J2": math.radians(20)},
        "J3_minus_20": {"J3": math.radians(-20)},
        "J4_plus_20": {"J4": math.radians(20)},
        "J5_minus_20": {"J5": math.radians(-20)},
        "J6_plus_30": {"J6": math.radians(30)},
    }
    pose_results = []
    for pose_name, values in poses.items():
        data.qpos[:] = 0.0
        for joint_name, angle in values.items():
            joint_id = object_id(model, mujoco.mjtObj.mjOBJ_JOINT, joint_name)
            data.qpos[int(model.jnt_qposadr[joint_id])] = angle
        mujoco.mj_forward(model, data)

        p_wg = data.xpos[gripper_id].copy()
        r_wg = data.xmat[gripper_id].reshape(3, 3).copy()
        expected_p_wc = p_wg + r_wg @ GRIPPER_TO_CAMERA_LINK_M
        expected_r_wc = r_wg @ r_gc
        expected_p_wo = expected_p_wc + expected_r_wc @ CAMERA_LINK_TO_SIM_OPTICAL_M
        expected_r_wo = expected_r_wc @ r_co
        expected_p_wm = expected_p_wo
        expected_r_wm = expected_r_wo @ r_om
        pose_results.append(
            {
                "pose": pose_name,
                "camera_link_position_error_m": max_abs(
                    data.xpos[camera_link_id], expected_p_wc
                ),
                "camera_link_rotation_error": max_abs(
                    data.xmat[camera_link_id].reshape(3, 3), expected_r_wc
                ),
                "optical_position_error_m": max_abs(
                    data.xpos[optical_id], expected_p_wo
                ),
                "optical_rotation_error": max_abs(
                    data.xmat[optical_id].reshape(3, 3), expected_r_wo
                ),
                "renderer_position_error_m": max_abs(
                    data.cam_xpos[renderer_camera_id], expected_p_wm
                ),
                "renderer_rotation_error": max_abs(
                    data.cam_xmat[renderer_camera_id].reshape(3, 3), expected_r_wm
                ),
            }
        )

    pose_max_position_error = max(
        value
        for result in pose_results
        for key, value in result.items()
        if key.endswith("position_error_m")
    )
    pose_max_rotation_error = max(
        value
        for result in pose_results
        for key, value in result.items()
        if key.endswith("rotation_error")
    )
    rigid_follow_pass = bool(
        pose_max_position_error <= POSITION_TOL_M
        and pose_max_rotation_error <= ROTATION_TOL
    )

    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)
    optical_world_pos = data.xpos[optical_id].copy()
    optical_world_rot = data.xmat[optical_id].reshape(3, 3).copy()
    render_qa = render_orientation_qa(optical_world_pos, optical_world_rot)

    report = {
        "schema": "go-m8010-arm-v15.13-virtual-camera-closure-qa/1.0",
        "scope": "pure simulation; no real Gemini driver frames or CameraInfo",
        "model_xml": MODEL_XML.name,
        "frames": {
            "camera_link": {
                "parent": "gripper",
                "position_m": GRIPPER_TO_CAMERA_LINK_M.tolist(),
                "quaternion_xyzw": [-0.5, 0.5, -0.5, 0.5],
                "frozen_cad_extrinsic": True,
            },
            "sim_camera_optical_frame": {
                "parent": "camera_link",
                "position_m": CAMERA_LINK_TO_SIM_OPTICAL_M.tolist(),
                "rpy_rad": [math.pi / 2.0, 0.0, 0.0],
                "quaternion_xyzw": [math.sqrt(0.5), 0.0, 0.0, math.sqrt(0.5)],
            },
            "sim_gemini_pro_renderer": {
                "parent": "sim_camera_optical_frame",
                "position_m": [0.0, 0.0, 0.0],
                "quaternion_wxyz": SIM_OPTICAL_TO_MUJOCO_CAMERA_QUAT_WXYZ.tolist(),
                "synthetic_fovy_deg": 60.0,
            },
        },
        "camera_link_to_sim_camera_optical_frame_matrix": [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, -0.00905],
            [0.0, 1.0, 0.0, -0.0128],
            [0.0, 0.0, 0.0, 1.0],
        ],
        "sim_camera_optical_to_mujoco_camera_matrix": [
            [1.0, 0.0, 0.0, 0.0],
            [0.0, -1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0, 0.0],
            [0.0, 0.0, 0.0, 1.0],
        ],
        "local_transform_checks": local_checks,
        "axis_math_checks": axis_checks,
        "rigid_follow": {
            "pose_count": len(poses),
            "poses": pose_results,
            "max_position_error_m": pose_max_position_error,
            "max_rotation_error": pose_max_rotation_error,
            "pass": rigid_follow_pass,
        },
        "render_orientation_qa": render_qa,
    }
    report["pass"] = bool(
        local_checks["pass"]
        and axis_checks["pass"]
        and rigid_follow_pass
        and render_qa["pass"]
    )
    OUTPUT_REPORT.write_text(
        json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    print(json.dumps({"status": "PASS" if report["pass"] else "FAIL", "report": str(OUTPUT_REPORT), "image": str(OUTPUT_IMAGE)}))
    if not report["pass"]:
        raise SystemExit(2)


if __name__ == "__main__":
    main()

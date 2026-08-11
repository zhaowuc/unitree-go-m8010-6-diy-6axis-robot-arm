from __future__ import annotations

"""Export V15.13 rigid-link assets using the measured camera-up zero frame."""

from pathlib import Path

import build_robot_arm_rigid_links_v15 as exporter


PROJECT_ROOT = Path(
    r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15/"
    r"V15_13_整机深度复核_相机上置机械零位"
)

exporter.SOURCE_ROOT = PROJECT_ROOT
exporter.SOURCE_FCSTD = PROJECT_ROOT / "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
exporter.SOURCE_AXES_JSON = PROJECT_ROOT / "V15_13_真实关节轴与相机上置零位.json"

exporter.OUTPUT_ROOT = PROJECT_ROOT / "rigid_links_v15_13"
exporter.VISUAL_DIR = exporter.OUTPUT_ROOT / "visual"
exporter.COLLISION_DIR = exporter.OUTPUT_ROOT / "collision"
exporter.GRIPPER_INTERNAL_DIR = exporter.OUTPUT_ROOT / "gripper_internal_zero_reference"
exporter.ROS2_DIR = exporter.OUTPUT_ROOT / "ros2"
exporter.MUJOCO_DIR = exporter.OUTPUT_ROOT / "mujoco"
exporter.OUTPUT_FCSTD = PROJECT_ROOT / "机械臂完整装配_六轴_刚性Link拆分_v15_13.FCStd"
exporter.MANIFEST_JSON = exporter.OUTPUT_ROOT / "rigid_link_manifest.json"
exporter.MEMBERSHIP_CSV = exporter.OUTPUT_ROOT / "rigid_link_membership.csv"
exporter.QA_JSON = exporter.OUTPUT_ROOT / "qa_rigid_link_export.json"
exporter.README_MD = exporter.OUTPUT_ROOT / "README.md"

exporter.GRIPPER_STATIC_HARDWARE = (
    [f"Gripper_Hardware_{index:02d}" for index in (3, 4, 7, 8, 9, 10, 15, 16)]
    + exporter.numbered("DM_G6220_Connector_M4x14_Screw_", 6)
    + exporter.numbered("Gripper_Clevis_3DPrinted_M3x25_Screw_", 2)
)
exporter.GRIPPER_VISUAL = exporter.specs_from_names(
    [
        "J6_Gripper_Connector_STEP_Display",
        "Gripper_Servo_STEP_Display",
        "Gripper_Fixed_Frame_STEP_Display",
        "Gemini_Pro_Camera_STEP_Display",
    ]
    + exporter.GRIPPER_STATIC_HARDWARE
)
exporter.GRIPPER_COLLISION = exporter.specs_from_names(
    [
        "J6_Gripper_Connector_Collision_Proxy",
        "Gripper_Fixed_Frame_Collision_Proxy",
        "Gripper_Servo_Collision_Proxy",
        "Gemini_Pro_Camera_Collision_Proxy",
    ]
)
exporter.LINK_SPECS["gripper"] = {
    "source_group": "J6_Gripper_Assembly/static + Gemini camera",
    "visual": exporter.GRIPPER_VISUAL,
    "collision": exporter.GRIPPER_COLLISION,
}


if __name__ == "__main__":
    exporter.build()

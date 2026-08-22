"""State source + robot_state_publisher + RViz + direct-qpos MuJoCo mirror."""

from __future__ import annotations

import subprocess

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _setup(context):
    description_path = LaunchConfiguration("description_path").perform(context)
    result = subprocess.run(
        ["xacro", description_path], check=True, capture_output=True, text=True, encoding="utf-8"
    )
    robot_description = result.stdout
    # /joint_states is intentionally lower-case. Make a runtime-only URDF view
    # without changing the frozen V15.13 geometry, axes, limits, or mesh paths.
    for index in range(1, 7):
        robot_description = robot_description.replace(
            f'<joint name="J{index}"', f'<joint name="joint{index}"'
        )

    return [
        Node(
            package="go_m8010_arm_hardware",
            executable="whole_arm_state_node",
            name="whole_arm_state_node",
            output="screen",
            parameters=[LaunchConfiguration("state_config")],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="go_m8010_arm_hardware",
            executable="whole_arm_mujoco_mirror",
            name="whole_arm_mujoco_mirror",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_mujoco_mirror")),
            parameters=[{
                "model_path": LaunchConfiguration("model_path"),
                "session_pose_deg": LaunchConfiguration("session_pose_deg"),
                "pose_matched": LaunchConfiguration("pose_matched"),
                "use_viewer": LaunchConfiguration("use_mujoco_viewer"),
                "evidence_directory": LaunchConfiguration("evidence_directory"),
            }],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]


def generate_launch_description() -> LaunchDescription:
    hardware_share = get_package_share_directory("go_m8010_arm_hardware")
    description_share = get_package_share_directory("go_m8010_arm_description")
    return LaunchDescription([
        DeclareLaunchArgument(
            "description_path",
            default_value=description_share + "/urdf/go_m8010_arm_v15_13.urdf.xacro",
        ),
        DeclareLaunchArgument("model_path", default_value="", description="Absolute frozen production MuJoCo XML path"),
        DeclareLaunchArgument("session_pose_deg", default_value="0,0,0,0,0,0", description="Six visually matched MuJoCo joint angles in degrees"),
        DeclareLaunchArgument("pose_matched", default_value="false"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_mujoco_mirror", default_value="true"),
        DeclareLaunchArgument("use_mujoco_viewer", default_value="true"),
        DeclareLaunchArgument("evidence_directory", default_value="hardware/v15_30a_ft"),
        DeclareLaunchArgument("state_config", default_value=hardware_share + "/config/whole_arm_state.yaml"),
        OpaqueFunction(function=_setup),
    ])

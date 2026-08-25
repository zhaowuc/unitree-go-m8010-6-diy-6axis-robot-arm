"""全中文 GUI、状态节点、MuJoCo direct-qpos 与命令路由的一体启动。"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    gui_share = get_package_share_directory("go_m8010_arm_gui")
    hardware_share = get_package_share_directory("go_m8010_arm_hardware")
    state_home_value = os.environ.get("XDG_STATE_HOME", "")
    state_home = (
        Path(state_home_value).expanduser()
        if state_home_value
        else Path.home() / ".local" / "state"
    )
    if not state_home.is_absolute():
        state_home = Path.home() / ".local" / "state"
    initial_pose_default = state_home / "go_m8010_arm_gui" / "initial_pose.json"

    state = Node(
        package="go_m8010_arm_hardware",
        executable="whole_arm_state_node",
        output="screen",
        parameters=[hardware_share + "/config/whole_arm_state.yaml", {
            "evidence_directory": LaunchConfiguration("runtime_log_directory"),
        }],
    )
    router = Node(
        package="go_m8010_arm_gui",
        executable="arm_gui_command_router",
        name="arm_gui_command_router",
        output="screen",
    )
    mirror = Node(
        package="go_m8010_arm_hardware",
        executable="whole_arm_mujoco_mirror",
        name="whole_arm_mujoco_mirror",
        output="screen",
        parameters=[{
            "model_path": LaunchConfiguration("model_path"),
            "session_pose_deg": LaunchConfiguration("session_pose_deg"),
            "pose_matched": False,
            "session_relative_baseline": True,
            "numeric_test_only": False,
            "use_viewer": False,
            "evidence_directory": LaunchConfiguration("runtime_log_directory"),
        }],
    )
    gui = Node(
        package="go_m8010_arm_gui",
        executable="arm_gui",
        name="arm_control_gui",
        output="screen",
        parameters=[{
            "config_path": LaunchConfiguration("config_path"),
            "joint_limits_path": LaunchConfiguration("joint_limits_path"),
            "initial_pose_path": LaunchConfiguration("initial_pose_path"),
            "log_directory": LaunchConfiguration("runtime_log_directory"),
            "embedded_model_path": LaunchConfiguration("model_path"),
            "embedded_session_pose_deg": LaunchConfiguration("session_pose_deg"),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("model_path"),
        DeclareLaunchArgument("session_pose_deg", default_value="0,0,0,0,0,0"),
        DeclareLaunchArgument("config_path", default_value=gui_share + "/config/arm_gui.yaml"),
        DeclareLaunchArgument("joint_limits_path", default_value=gui_share + "/config/gui_joint_limits.yaml"),
        DeclareLaunchArgument("initial_pose_path", default_value=str(initial_pose_default)),
        DeclareLaunchArgument("runtime_log_directory", default_value="logs/arm_gui"),
        state,
        router,
        mirror,
        gui,
        RegisterEventHandler(OnProcessExit(
            target_action=gui,
            on_exit=[EmitEvent(event=Shutdown(reason="控制界面已关闭，停止全部ROS2进程"))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=state,
            on_exit=[EmitEvent(event=Shutdown(reason="硬件状态节点已退出，触发安全停止"))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=router,
            on_exit=[EmitEvent(event=Shutdown(reason="命令路由节点已退出，触发安全停止"))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=mirror,
            on_exit=[EmitEvent(event=Shutdown(reason="MuJoCo镜像节点已退出，触发安全停止"))],
        )),
    ])

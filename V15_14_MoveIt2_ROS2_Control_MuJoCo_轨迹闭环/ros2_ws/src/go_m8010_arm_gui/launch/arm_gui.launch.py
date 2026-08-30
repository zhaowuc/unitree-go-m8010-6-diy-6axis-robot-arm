"""全中文 GUI、状态节点、MuJoCo direct-qpos 与命令路由的一体启动。"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
            "persistent_zero_path": LaunchConfiguration("persistent_zero_path"),
            "recovery_hint_path": LaunchConfiguration("recovery_hint_path"),
            "initial_pose_path": LaunchConfiguration("initial_pose_path"),
            "j2_session_reference_path": LaunchConfiguration(
                "j2_session_reference_path"
            ),
            "go_aux_session_reference_path": LaunchConfiguration(
                "go_aux_session_reference_path"
            ),
            "thermal_config_path": hardware_share + "/config/thermal_limits.yaml",
        }],
    )
    router = Node(
        package="go_m8010_arm_gui",
        executable="arm_gui_command_router",
        name="arm_gui_command_router",
        output="screen",
        parameters=[{
            # Production is fail-closed.  The explicit launch override exists
            # only for isolated protocol regression tests with no workers.
            "allow_legacy_v12_position": ParameterValue(
                LaunchConfiguration("allow_legacy_v12_position"),
                value_type=bool,
            ),
        }],
    )
    mirror = Node(
        package="go_m8010_arm_hardware",
        executable="whole_arm_mujoco_mirror",
        name="whole_arm_mujoco_mirror",
        output="screen",
        parameters=[{
            "model_path": LaunchConfiguration("model_path"),
            "session_pose_deg": LaunchConfiguration("session_pose_deg"),
            "pose_matched": ParameterValue(
                LaunchConfiguration("pose_matched"), value_type=bool
            ),
            "session_relative_baseline": True,
            "numeric_test_only": False,
            "use_viewer": False,
            "evidence_directory": LaunchConfiguration("runtime_log_directory"),
        }],
    )
    gravity = Node(
        package="go_m8010_arm_hardware",
        executable="whole_arm_gravity_node",
        name="whole_arm_gravity_node",
        output="screen",
        parameters=[{
            "model_path": LaunchConfiguration("model_path"),
            "gravity_config_path": hardware_share + "/config/gravity_control.yaml",
            "thermal_config_path": hardware_share + "/config/thermal_limits.yaml",
            "anchor_path": LaunchConfiguration("gravity_anchor_path"),
            "calculation_rate_hz": 100.0,
            "joint_state_maximum_age_ms": 250.0,
            # Both values default fail-closed.  A powered validation stage
            # must explicitly provide a session-bound anchor, enable the
            # hardware authority and select one approved scale rung.
            "enabled_for_hardware": ParameterValue(
                LaunchConfiguration("gravity_enabled_for_hardware"),
                value_type=bool,
            ),
            "gravity_scale_target": ParameterValue(
                LaunchConfiguration("gravity_scale_target"),
                value_type=float,
            ),
            "gravity_scale_ramp_seconds": 2.0,
            "maximum_rotor_torque_slew_nm_per_s": 1.0,
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
            "initial_pose_read_only": ParameterValue(
                LaunchConfiguration("initial_pose_read_only"), value_type=bool
            ),
            "log_directory": LaunchConfiguration("runtime_log_directory"),
            "embedded_model_path": LaunchConfiguration("model_path"),
            "embedded_session_pose_deg": LaunchConfiguration("session_pose_deg"),
            "thermal_config_path": hardware_share + "/config/thermal_limits.yaml",
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument("model_path"),
        DeclareLaunchArgument("session_pose_deg", default_value="0,0,0,0,0,0"),
        DeclareLaunchArgument("pose_matched", default_value="false"),
        DeclareLaunchArgument(
            "allow_legacy_v12_position", default_value="false"
        ),
        DeclareLaunchArgument(
            "config_path", default_value=gui_share + "/config/arm_gui.yaml"
        ),
        DeclareLaunchArgument(
            "joint_limits_path",
            default_value=gui_share + "/config/gui_joint_limits.yaml",
        ),
        DeclareLaunchArgument(
            "initial_pose_path", default_value=str(initial_pose_default)
        ),
        DeclareLaunchArgument("initial_pose_read_only", default_value="false"),
        DeclareLaunchArgument("persistent_zero_path", default_value=""),
        DeclareLaunchArgument("recovery_hint_path", default_value=""),
        DeclareLaunchArgument("j2_session_reference_path", default_value=""),
        DeclareLaunchArgument("go_aux_session_reference_path", default_value=""),
        DeclareLaunchArgument("gravity_anchor_path", default_value=""),
        DeclareLaunchArgument(
            "gravity_enabled_for_hardware", default_value="false"
        ),
        DeclareLaunchArgument("gravity_scale_target", default_value="0.0"),
        DeclareLaunchArgument("runtime_log_directory", default_value="logs/arm_gui"),
        state,
        router,
        mirror,
        gravity,
        gui,
        RegisterEventHandler(OnProcessExit(
            target_action=gui,
            on_exit=[LogInfo(msg=(
                "控制界面已退出；保留状态/路由进程，硬件 worker 将按独立租约策略进入安全保持。"
                "不要在机械臂未支撑时终止顶层 supervisor。"
            ))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=state,
            on_exit=[LogInfo(msg=(
                "硬件状态节点已退出；GUI 将把状态判为过期，但不联动终止其它故障域。"
            ))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=router,
            on_exit=[LogInfo(msg=(
                "命令路由节点已退出；不再转发新目标，硬件 worker 将按独立租约策略处理。"
            ))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=mirror,
            on_exit=[LogInfo(msg=(
                "MuJoCo镜像节点已退出；仿真显示故障不联动撤销真机保持。"
            ))],
        )),
        RegisterEventHandler(OnProcessExit(
            target_action=gravity,
            on_exit=[LogInfo(msg=(
                "重力诊断节点已退出；硬件Tff保持禁用，GUI必须拒绝新的现实轨迹。"
            ))],
        )),
    ])

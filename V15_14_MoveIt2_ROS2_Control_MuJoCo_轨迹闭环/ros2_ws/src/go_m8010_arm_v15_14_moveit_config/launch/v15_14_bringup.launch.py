from __future__ import annotations

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description() -> LaunchDescription:
    model_path = LaunchConfiguration("mujoco_model_path")
    use_mujoco_viewer = LaunchConfiguration("use_mujoco_viewer")
    use_rviz = LaunchConfiguration("use_rviz")

    description_share = Path(
        get_package_share_directory("go_m8010_arm_v15_14_description")
    )
    xacro_path = description_share / "urdf" / "go_m8010_arm_v15_14.urdf.xacro"
    config_share = Path(
        get_package_share_directory("go_m8010_arm_v15_14_moveit_config")
    )

    moveit_config = (
        MoveItConfigsBuilder(
            "go_m8010_arm_v15_14",
            package_name="go_m8010_arm_v15_14_moveit_config",
        )
        .robot_description(
            file_path=str(xacro_path),
            mappings={
                "mujoco_joint_commands_topic": "/mujoco/joint_commands",
                "mujoco_joint_states_topic": "/mujoco/joint_states_raw",
            },
        )
        .robot_description_semantic(
            file_path="config/go_m8010_arm_v15_14.srdf"
        )
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .planning_pipelines(
            pipelines=["ompl"], default_planning_pipeline="ompl"
        )
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    controllers_yaml = str(config_share / "config" / "ros2_controllers.yaml")
    rviz_config = str(config_share / "rviz" / "moveit_v15_14.rviz")

    bridge = Node(
        package="go_m8010_arm_mujoco_bridge",
        executable="mujoco_bridge",
        name="mujoco_bridge",
        output="screen",
        parameters=[
            {
                "model_path": model_path,
                "command_topic": "/mujoco/joint_commands",
                "state_topic": "/mujoco/joint_states_raw",
                "publish_rate_hz": 50.0,
                "guard_max_step_deg": 0.25,
                # V15.14 simulation-only execution envelope. These are not
                # hardware ratings; keep them aligned with MoveIt limits and
                # the machine-readable regression contract.
                "max_velocity_rad_s": 0.5,
                "max_acceleration_rad_s2": 1.0,
                "execution_limit_abs_tolerance": 0.02,
                "velocity_consistency_tolerance_rad_s": 0.04,
                "command_timeout_s": 0.1,
                "prime_position_tolerance_rad": 1.0e-6,
                "prime_velocity_tolerance_rad_s": 1.0e-6,
                "latch_fault": True,
                "use_viewer": use_mujoco_viewer,
            }
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    static_ground_scene = Node(
        package="go_m8010_arm_v15_14_moveit_config",
        executable="static_ground_scene",
        output="screen",
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[moveit_config.robot_description, controllers_yaml],
    )

    joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )
    arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "arm_controller",
            "--controller-manager",
            "/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        output="screen",
    )

    move_group = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {
                "publish_robot_description": True,
                "publish_robot_description_semantic": True,
                "allow_trajectory_execution": True,
                "capabilities": "",
                "disable_capabilities": "",
            },
        ],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
        ],
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "mujoco_model_path",
                default_value=EnvironmentVariable(
                    "GO_M8010_V15_14_MJCF",
                    default_value="",
                ),
                description=(
                    "Absolute path to the V15.14 production MJCF with the audited "
                    "25-proxy collision layer. Pass mujoco_model_path:=... or set "
                    "GO_M8010_V15_14_MJCF."
                ),
            ),
            DeclareLaunchArgument("use_mujoco_viewer", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            bridge,
            robot_state_publisher,
            controller_manager,
            TimerAction(period=2.0, actions=[joint_state_broadcaster]),
            TimerAction(period=3.0, actions=[arm_controller]),
            TimerAction(period=4.0, actions=[move_group]),
            TimerAction(period=5.0, actions=[static_ground_scene]),
            TimerAction(period=6.0, actions=[rviz]),
        ]
    )

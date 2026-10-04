import math

from launch import LaunchDescription
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    # Accepted V15.13 GUI startup posture.  These are only initial /joint_states;
    # no URDF joint origin, axis, limit, or hardware encoder zero is changed.
    gui_default_zero = {
        "J1": math.radians(-135.0),
        "J2": math.radians(150.0),
        "J3": math.radians(-74.40207091282407),
        "J4": math.radians(-90.0),
        "J5": math.radians(45.0),
        "J6": math.radians(0.7134917101830321),
    }
    xacro_file = PathJoinSubstitution(
        [
            FindPackageShare("go_m8010_arm_description"),
            "urdf",
            "go_m8010_arm_v15_13.urdf.xacro",
        ]
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", xacro_file]),
        value_type=str,
    )

    return LaunchDescription(
        [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": robot_description}],
            ),
            Node(
                package="joint_state_publisher_gui",
                executable="joint_state_publisher_gui",
                name="joint_state_publisher_gui",
                output="screen",
                parameters=[
                    {f"zeros.{joint}": value for joint, value in gui_default_zero.items()}
                ],
            ),
        ]
    )

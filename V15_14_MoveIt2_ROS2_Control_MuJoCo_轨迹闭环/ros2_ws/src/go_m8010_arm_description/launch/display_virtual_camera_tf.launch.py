from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description() -> LaunchDescription:
    package_share = FindPackageShare("go_m8010_arm_description")
    base_display = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([package_share, "launch", "display_tf.launch.py"])
        )
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="virtual_camera_tf_rviz",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [package_share, "rviz", "virtual_camera_tf.rviz"]
            ),
        ],
    )
    return LaunchDescription([base_display, rviz])

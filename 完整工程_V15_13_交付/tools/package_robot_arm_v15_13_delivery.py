from __future__ import annotations

"""Create the self-contained V15.13 CAD/ROS2/MoveIt/MuJoCo delivery bundle."""

import hashlib
import json
import shutil
import zipfile
from pathlib import Path


WORKSPACE = Path(r"D:/AI_JIXIEBI/go_m8010_arm_gui")
ROOT = Path(
    r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15/"
    r"V15_13_整机深度复核_相机上置机械零位"
)
DELIVERY = ROOT / "完整工程_V15_13_交付"
ZIP_PATH = ROOT / "机械臂完整工程_V15_13_相机上置机械零位_交付.zip"
RIGID = ROOT / "rigid_links_v15_13"

MAIN_FCSTD = ROOT / "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
RIGID_FCSTD = ROOT / "机械臂完整装配_六轴_刚性Link拆分_v15_13.FCStd"
TREE_JSON = RIGID / "ros2" / "rigid_link_tree_staging.json"
LIMIT_JSON = ROOT / "V15_13_ROS2_MoveIt_MuJoCo_关节限位契约.json"

# Frozen frame contracts.  Keep these as text so regenerated Xacro preserves
# every reviewed decimal rather than shortening values through float formatting.
TCP_NOMINAL_XYZ_M = "-0.001187726400 0.000060729026 0.092493513872"
GRIPPER_TO_CAMERA_XYZ_M = "-0.038917046930682 0.000061006900000 0.019635428243040"
GRIPPER_TO_CAMERA_RPY_RAD = "-1.5707963267948966 0 -1.5707963267948966"
GRIPPER_TO_CAMERA_QUAT_WXYZ = "0.5 -0.5 0.5 -0.5"
CAMERA_TO_SIM_OPTICAL_XYZ_M = "0 -0.00905 -0.0128"
CAMERA_TO_SIM_OPTICAL_RPY_RAD = "1.5707963267948966 0 0"
CAMERA_TO_SIM_OPTICAL_QUAT_WXYZ = "0.707106781186548 0.707106781186548 0 0"
SIM_OPTICAL_TO_MUJOCO_CAMERA_QUAT_WXYZ = "0 1 0 0"

ROS_VISUAL_SOURCE = ROOT / "mujoco_kinematic_v1" / "meshes" / "ros_visual_mm"
ROS_DESCRIPTION_PACKAGE = "go_m8010_arm_description"


def copy_file(source: Path, target: Path):
    if not source.is_file() or source.stat().st_size == 0:
        raise FileNotFoundError(source)
    target.parent.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source, target)


def copy_tree(source: Path, target: Path):
    if not source.is_dir():
        raise FileNotFoundError(source)
    shutil.copytree(source, target)


def fmt(values):
    return " ".join(f"{float(value):.15g}" for value in values)


def write_urdf(tree):
    description = DELIVERY / "ros2_ws" / "src" / ROS_DESCRIPTION_PACKAGE
    urdf_dir = description / "urdf"
    urdf_dir.mkdir(parents=True, exist_ok=True)
    lines = [
        '<?xml version="1.0"?>',
        '<robot xmlns:xacro="http://www.ros.org/wiki/xacro" name="go_m8010_arm_v15_13">',
        '  <!-- Geometry/kinematics are verified. Effort and velocity default to 0 (fail-closed). -->',
        '  <xacro:arg name="mesh_prefix" default="package://go_m8010_arm_description/meshes"/>',
    ]
    for index in range(1, 7):
        lines.append(f'  <xacro:arg name="j{index}_effort_nm" default="0.0"/>')
        lines.append(f'  <xacro:arg name="j{index}_velocity_rad_s" default="0.0"/>')
    lines.extend(
        [
            '  <material name="base_visual"><color rgba="0.28 0.30 0.34 1"/></material>',
            '  <material name="arm_visual"><color rgba="0.54 0.09 0.13 1"/></material>',
            '  <material name="link6_visual"><color rgba="0.20 0.22 0.25 1"/></material>',
            '  <material name="gripper_visual"><color rgba="0.05 0.50 0.58 1"/></material>',
            '  <link name="world"/>',
        ]
    )
    material_by_link = {
        "base_link": "base_visual",
        "link6": "link6_visual",
        "gripper": "gripper_visual",
    }
    for link in tree["links"]:
        material = material_by_link.get(link, "arm_visual")
        lines.extend(
            [
                f'  <link name="{link}">',
                '    <visual>',
                '      <origin xyz="0 0 0" rpy="0 0 0"/>',
                f'      <geometry><mesh filename="$(arg mesh_prefix)/visual/{link}.stl" scale="0.001 0.001 0.001"/></geometry>',
                f'      <material name="{material}"/>',
                '    </visual>',
                '    <collision>',
                '      <origin xyz="0 0 0" rpy="0 0 0"/>',
                f'      <geometry><mesh filename="$(arg mesh_prefix)/collision/{link}.stl" scale="0.001 0.001 0.001"/></geometry>',
                '    </collision>',
                '    <!-- Inertial intentionally omitted until measured mass/COM/inertia are supplied. -->',
                '  </link>',
            ]
        )
    lines.extend(
        [
            '  <link name="tcp_nominal"/>',
            '  <!-- Simulation-only camera frames. camera_link preserves the frozen CAD',
            '       placement; sim_camera_optical_frame is not a vendor-driver frame. -->',
            '  <link name="camera_link"/>',
            '  <link name="sim_camera_optical_frame"/>',
            '  <joint name="world_to_base_link" type="fixed">',
            '    <parent link="world"/>',
            '    <child link="base_link"/>',
            '    <origin xyz="0 0 0" rpy="0 0 0"/>',
            '  </joint>',
        ]
    )
    for joint in tree["joints"]:
        name = joint["name"]
        joint_type = joint["joint_type_ros2"]
        lines.extend(
            [
                f'  <joint name="{name}" type="{joint_type}">',
                f'    <parent link="{joint["parent_link"]}"/>',
                f'    <child link="{joint["child_link"]}"/>',
                f'    <origin xyz="{fmt(joint["origin_xyz_m"])}" rpy="{fmt(joint["origin_rpy_rad"])}"/>',
            ]
        )
        if joint_type != "fixed":
            lines.append(f'    <axis xyz="{fmt(joint["axis_xyz_in_joint_frame"])}"/>')
            index = int(name[1:])
            if joint_type == "continuous":
                lines.append(
                    f'    <limit effort="$(arg j{index}_effort_nm)" velocity="$(arg j{index}_velocity_rad_s)"/>'
                )
            else:
                limits = joint["position_limit_rad"]
                lines.append(
                    f'    <limit lower="{limits["lower"]:.15g}" upper="{limits["upper"]:.15g}" '
                    f'effort="$(arg j{index}_effort_nm)" velocity="$(arg j{index}_velocity_rad_s)"/>'
                )
        lines.append('  </joint>')
    lines.extend(
        [
            '  <joint name="gripper_to_tcp_nominal" type="fixed">',
            '    <parent link="gripper"/>',
            '    <child link="tcp_nominal"/>',
            f'    <origin xyz="{TCP_NOMINAL_XYZ_M}" rpy="0 0 0"/>',
            '  </joint>',
            '  <joint name="gripper_to_camera_link" type="fixed">',
            '    <parent link="gripper"/>',
            '    <child link="camera_link"/>',
            f'    <origin xyz="{GRIPPER_TO_CAMERA_XYZ_M}"',
            f'            rpy="{GRIPPER_TO_CAMERA_RPY_RAD}"/>',
            '  </joint>',
            '  <joint name="camera_link_to_sim_camera_optical_frame" type="fixed">',
            '    <parent link="camera_link"/>',
            '    <child link="sim_camera_optical_frame"/>',
            f'    <origin xyz="{CAMERA_TO_SIM_OPTICAL_XYZ_M}" rpy="{CAMERA_TO_SIM_OPTICAL_RPY_RAD}"/>',
            '  </joint>',
            '</robot>',
        ]
    )
    (urdf_dir / "go_m8010_arm_v15_13.urdf.xacro").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    (description / "package.xml").write_text(
        """<?xml version="1.0"?>
<package format="3">
  <name>go_m8010_arm_description</name>
  <version>15.13.0</version>
  <description>Verified V15.13 geometry and kinematic frames for the GO-M8010 arm.</description>
  <maintainer email="replace@after.measurement.invalid">V15.13 handoff</maintainer>
  <license>Proprietary project data</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>launch</exec_depend>
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher</exec_depend>
  <exec_depend>joint_state_publisher_gui</exec_depend>
  <exec_depend>rviz2</exec_depend>
  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
""",
        encoding="utf-8",
    )
    (description / "CMakeLists.txt").write_text(
        """cmake_minimum_required(VERSION 3.8)
project(go_m8010_arm_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY launch urdf meshes rviz DESTINATION share/${PROJECT_NAME})
ament_package()
""",
        encoding="utf-8",
    )
    (description / "README.md").write_text(
        """# go_m8010_arm_description V15.13

几何、真实关节轴、关节原点、零位和位置限位已经复核。J6 在 `0°` 时相机位于上侧，位置限位为 `[-π,+π]`。

`j*_effort_nm` 与 `j*_velocity_rad_s` 默认值为 `0`，这是故障安全占位，不是电机参数。完成电机、减速器、热约束与线束实测后必须显式传入；质量、质心和惯量未测量，因此 URDF 暂不含 inertial，不能作为最终动力学模型。

`camera_link` 使用已冻结的 CAD 机械外参；`sim_camera_optical_frame` 仅用于纯仿真，不代表 Gemini Pro 实机驱动的 optical frame，也不包含编造的真实 CameraInfo/内参。
""",
        encoding="utf-8",
    )
    write_ros_runtime_assets(description)


def write_ros_runtime_assets(description: Path):
    """Persist the verified robot-state-publisher and RViz entry points."""

    launch_dir = description / "launch"
    rviz_dir = description / "rviz"
    launch_dir.mkdir(parents=True, exist_ok=True)
    rviz_dir.mkdir(parents=True, exist_ok=True)

    (launch_dir / "display_tf.launch.py").write_text(
        '''import math

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
''',
        encoding="utf-8",
    )
    (launch_dir / "display_virtual_camera_tf.launch.py").write_text(
        '''from launch import LaunchDescription
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
''',
        encoding="utf-8",
    )
    (rviz_dir / "virtual_camera_tf.rviz").write_text(
        '''Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views
Visualization Manager:
  Class: ""
  Displays:
    - Alpha: 1.0
      Class: rviz_default_plugins/RobotModel
      Description Source: Topic
      Description Topic:
        Depth: 5
        Durability Policy: Transient Local
        History Policy: Keep Last
        Reliability Policy: Reliable
        Value: /robot_description
      Enabled: true
      Name: RobotModel
      Visual Enabled: true
      Collision Enabled: false
    - Class: rviz_default_plugins/TF
      Enabled: true
      Frame Timeout: 15
      Frames:
        All Enabled: true
      Marker Scale: 0.16
      Name: TF — CAD + simulated optical
      Show Arrows: true
      Show Axes: true
      Show Names: true
      Update Interval: 0
  Enabled: true
  Global Options:
    Background Color: 35; 38; 45
    Fixed Frame: world
    Frame Rate: 30
  Name: root
  Tools:
    - Class: rviz_default_plugins/Interact
    - Class: rviz_default_plugins/MoveCamera
    - Class: rviz_default_plugins/Select
  Transformation:
    Current:
      Class: rviz_default_plugins/TF
  Views:
    Current:
      Class: rviz_default_plugins/Orbit
      Distance: 1.15
      Focal Point:
        X: -0.18
        Y: 0.0
        Z: 0.25
      Name: Virtual camera TF overview
      Near Clip Distance: 0.01
      Pitch: 0.55
      Target Frame: world
      Yaw: 2.45
    Saved: ~
Window Geometry:
  Height: 1000
  Width: 1500
  X: 40
  Y: 40
''',
        encoding="utf-8",
    )


def write_moveit(limits):
    package = DELIVERY / "ros2_ws" / "src" / "go_m8010_arm_moveit_config"
    config = package / "config"
    config.mkdir(parents=True, exist_ok=True)
    rows = ["joint_limits:"]
    for name in ("J1", "J2", "J3", "J4", "J5", "J6"):
        value = limits["position_limits_rad"][name]
        rows.append(f"  {name}:")
        rows.append(f"    has_position_limits: {'false' if value is None else 'true'}")
        if value is not None:
            rows.append(f"    min_position: {value[0]:.15g}")
            rows.append(f"    max_position: {value[1]:.15g}")
        rows.extend(
            [
                "    has_velocity_limits: false",
                "    has_acceleration_limits: false",
                "    has_jerk_limits: false",
            ]
        )
    (config / "joint_limits.yaml").write_text("\n".join(rows) + "\n", encoding="utf-8")
    (config / "go_m8010_arm_v15_13.srdf.template").write_text(
        """<?xml version="1.0"?>
<robot name="go_m8010_arm_v15_13">
  <group name="arm"><chain base_link="base_link" tip_link="gripper"/></group>
  <group_state name="camera_up_zero" group="arm">
    <joint name="J1" value="0"/><joint name="J2" value="0"/>
    <joint name="J3" value="0"/><joint name="J4" value="0"/>
    <joint name="J5" value="0"/><joint name="J6" value="0"/>
  </group_state>
  <!-- Do not blanket-disable adjacent-link collisions. Generate/inspect ACM in Setup Assistant. -->
</robot>
""",
        encoding="utf-8",
    )
    (config / "self_collision_policy.yaml").write_text(
        """runtime_collision_guard_required: true
freecad_reference_guard: j123456_camera_physical_collision_guard_v15_13.py
continuous_trajectory_validation_required: true
automatically_disable_all_adjacent_links: false
reason: >-
  The full 6-D joint space is non-rectangular. Five collisions were found in
  150 deterministic coupled samples even though all independent scans passed.
  Designed stator/output contacts are component-level exclusions and must not
  be converted into blanket adjacent-link exclusions without validation.
""",
        encoding="utf-8",
    )
    (package / "package.xml").write_text(
        """<?xml version="1.0"?>
<package format="3">
  <name>go_m8010_arm_moveit_config</name>
  <version>15.13.0</version>
  <description>MoveIt staging limits and collision policy for V15.13.</description>
  <maintainer email="replace@after.measurement.invalid">V15.13 handoff</maintainer>
  <license>Proprietary project data</license>
  <buildtool_depend>ament_cmake</buildtool_depend>
  <exec_depend>moveit_ros_move_group</exec_depend>
  <exec_depend>go_m8010_arm_description</exec_depend>
</package>
""",
        encoding="utf-8",
    )
    (package / "CMakeLists.txt").write_text(
        """cmake_minimum_required(VERSION 3.8)
project(go_m8010_arm_moveit_config)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
ament_package()
""",
        encoding="utf-8",
    )


def inertial_template(link):
    upper = link.upper()
    return (
        f'<inertial pos="__{upper}_COM_X__ __{upper}_COM_Y__ __{upper}_COM_Z__" '
        f'mass="__{upper}_MASS_KG__" fullinertia="__{upper}_IXX__ __{upper}_IYY__ '
        f'__{upper}_IZZ__ __{upper}_IXY__ __{upper}_IXZ__ __{upper}_IYZ__"/>'
    )


def write_mujoco(tree):
    target = DELIVERY / "mujoco"
    target.mkdir(parents=True, exist_ok=True)
    joint_by_parent = {joint["parent_link"]: joint for joint in tree["joints"]}
    children = {link: [] for link in tree["links"]}
    joint_for_child = {}
    for joint in tree["joints"]:
        children[joint["parent_link"]].append(joint["child_link"])
        joint_for_child[joint["child_link"]] = joint

    lines = [
        '<mujoco model="go_m8010_arm_v15_13">',
        '  <compiler angle="radian" meshdir="../ros2_ws/src/go_m8010_arm_description/meshes" inertiafromgeom="false"/>',
        '  <option gravity="0 0 -9.81"/>',
        '  <default>',
        '    <geom density="0" friction="__SLIDE_FRICTION__ __TORSIONAL_FRICTION__ __ROLLING_FRICTION__"/>',
        '  </default>',
        '  <asset>',
    ]
    for link in tree["links"]:
        lines.append(f'    <mesh name="{link}_visual" file="visual/{link}.stl" scale="0.001 0.001 0.001"/>')
        lines.append(f'    <mesh name="{link}_collision" file="collision/{link}.stl" scale="0.001 0.001 0.001"/>')
    lines.extend(['  </asset>', '  <worldbody>'])

    def emit_body(link, indent, joint=None):
        prefix = "  " * indent
        if joint is None:
            pos = "0 0 0"
            quat = "1 0 0 0"
        else:
            pos = fmt(joint["mujoco"]["body_pos_m_in_parent"])
            quat = fmt(joint["mujoco"]["body_quat_wxyz_in_parent"])
        lines.append(f'{prefix}<body name="{link}" pos="{pos}" quat="{quat}">')
        if link != "base_link":
            lines.append(f'{prefix}  {inertial_template(link)}')
        if joint is not None and joint["joint_type_ros2"] != "fixed":
            mj = joint["mujoco"]
            attrs = [
                f'name="{joint["name"]}"',
                'type="hinge"',
                f'pos="{fmt(mj["joint_pos_m_in_child_body"])}"',
                f'axis="{fmt(mj["joint_axis_in_child_body"])}"',
                'ref="0"',
            ]
            if mj["joint_limited"]:
                attrs.extend(['limited="true"', f'range="{fmt(mj["joint_range_rad"])}"'])
            else:
                attrs.append('limited="false"')
            lines.append(f'{prefix}  <joint {" ".join(attrs)}/>')
        lines.append(f'{prefix}  <geom name="{link}_visual" type="mesh" mesh="{link}_visual" contype="0" conaffinity="0" group="1" rgba="0.45 0.14 0.18 1"/>')
        lines.append(f'{prefix}  <geom name="{link}_collision" type="mesh" mesh="{link}_collision" contype="1" conaffinity="1" group="3" rgba="0.1 0.7 0.8 0.25"/>')
        if link == "gripper":
            lines.extend(
                [
                    f'{prefix}  <site name="tool_reference" pos="0 0 0" type="sphere" size="0.003" rgba="0.2 0.6 1 1" group="4"/>',
                    f'{prefix}  <site name="tcp_nominal" pos="{TCP_NOMINAL_XYZ_M}" type="sphere" size="0.006" rgba="1 0.05 0.95 1" group="0"/>',
                    f'{prefix}  <body name="camera_link" pos="{GRIPPER_TO_CAMERA_XYZ_M}" quat="{GRIPPER_TO_CAMERA_QUAT_WXYZ}">',
                    f'{prefix}    <body name="sim_camera_optical_frame" pos="{CAMERA_TO_SIM_OPTICAL_XYZ_M}" quat="{CAMERA_TO_SIM_OPTICAL_QUAT_WXYZ}">',
                    f'{prefix}      <site name="sim_camera_optical_origin" pos="0 0 0" type="sphere" size="0.004" rgba="0.05 0.85 1 1" group="4"/>',
                    f'{prefix}      <camera name="sim_gemini_pro_renderer" pos="0 0 0" quat="{SIM_OPTICAL_TO_MUJOCO_CAMERA_QUAT_WXYZ}" mode="fixed" fovy="60"/>',
                    f'{prefix}    </body>',
                    f'{prefix}  </body>',
                ]
            )
        for child in children.get(link, []):
            emit_body(child, indent + 1, joint_for_child[child])
        lines.append(f'{prefix}</body>')

    emit_body("base_link", 2)
    lines.extend(['  </worldbody>', '</mujoco>'])
    (target / "go_m8010_arm_v15_13.template.xml").write_text(
        "\n".join(lines) + "\n", encoding="utf-8"
    )
    (target / "README.md").write_text(
        """# MuJoCo V15.13 staging

关节树、轴、原点、J2～J6 位置限位和 visual/collision 网格已写入模板。J6 为 `hinge`，`ref=0` 时相机在上，`range=-π +π`。

`tcp_nominal`、已冻结的 `camera_link`、仿真专用 `sim_camera_optical_frame` 与 MuJoCo renderer camera 已同步固化。renderer 的 `fovy=60` 仅是合成渲染设定，不宣称为 Gemini Pro 真实内参。

模板中的质量、质心、完整惯量张量和摩擦参数均为 `__...__` 占位符，必须由实测/标定值替换；没有替换前不是可运行的动力学模型。不要让 MuJoCo 依据高细节网格默认密度猜惯量。组合动作仍需使用碰撞检测，因为独立关节限位的笛卡尔积包含自碰撞姿态。
""",
        encoding="utf-8",
    )


def package_guards():
    tools = DELIVERY / "tools"
    tools.mkdir(parents=True, exist_ok=True)
    sources = {
        "j123456_camera_physical_collision_guard_v15_13.py": WORKSPACE / "j123456_camera_physical_collision_guard_v15_13.py",
        "j123456_gripper_physical_collision_guard_v14.py": WORKSPACE / "j123456_gripper_physical_collision_guard_v14.py",
        "j123456_physical_collision_guard_v13.py": WORKSPACE / "j123456_physical_collision_guard_v13.py",
        "j12345_physical_collision_guard_v12.py": WORKSPACE / "j12345_physical_collision_guard_v12.py",
        "j1234_physical_collision_guard_v10.py": Path(r"D:/AI_JIXIEBI/模型/机械臂底座肩部大臂小臂腕前连接件蓝线水平翻转内置装配_v10/j1234_physical_collision_guard_v10.py"),
        "j1234_wrist_180_inward_collision_monitor_v9.py": Path(r"D:/AI_JIXIEBI/模型/机械臂底座肩部大臂小臂腕前连接件180度内向装配_v9/j1234_wrist_180_inward_collision_monitor_v9.py"),
        "j234_wrist_horizontal_collision_monitor_v8.py": Path(r"D:/AI_JIXIEBI/模型/机械臂底座肩部大臂小臂腕前连接件水平装配_v8/j234_wrist_horizontal_collision_monitor_v8.py"),
        "j234_wrist_collision_monitor_v7.py": Path(r"D:/AI_JIXIEBI/模型/机械臂底座肩部大臂小臂腕前连接件装配_v7/j234_wrist_collision_monitor_v7.py"),
        "j234_collision_monitor_v6.py": Path(r"D:/AI_JIXIEBI/模型/机械臂底座肩部大臂小臂末端电机装配_v6/j234_collision_monitor_v6.py"),
    }
    for name, source in sources.items():
        copy_file(source, tools / name)

    replacements = {
        "j12345_physical_collision_guard_v12.py": (
            'V10_GUARD = Path(\n    r"D:/AI_JIXIEBI/模型/机械臂底座肩部大臂小臂腕前连接件蓝线水平翻转内置装配_v10/"\n    "j1234_physical_collision_guard_v10.py"\n)',
            'V10_GUARD = ROOT / "j1234_physical_collision_guard_v10.py"',
        ),
        "j1234_physical_collision_guard_v10.py": (
            'V9_MONITOR = Path(\n    r"D:/AI_JIXIEBI/模型/机械臂底座肩部大臂小臂腕前连接件180度内向装配_v9/"\n    "j1234_wrist_180_inward_collision_monitor_v9.py"\n)',
            'V9_MONITOR = ROOT / "j1234_wrist_180_inward_collision_monitor_v9.py"',
        ),
        "j1234_wrist_180_inward_collision_monitor_v9.py": (
            'V8_MONITOR_PATH = Path(\n    r"D:/AI_JIXIEBI/模型/机械臂底座肩部大臂小臂腕前连接件水平装配_v8/j234_wrist_horizontal_collision_monitor_v8.py"\n)',
            'ROOT = Path(__file__).resolve().parent\nV8_MONITOR_PATH = ROOT / "j234_wrist_horizontal_collision_monitor_v8.py"',
        ),
        "j234_wrist_horizontal_collision_monitor_v8.py": (
            'V7_MONITOR_PATH = Path(\n    r"D:/AI_JIXIEBI/模型/机械臂底座肩部大臂小臂腕前连接件装配_v7/j234_wrist_collision_monitor_v7.py"\n)',
            'ROOT = Path(__file__).resolve().parent\nV7_MONITOR_PATH = ROOT / "j234_wrist_collision_monitor_v7.py"',
        ),
        "j234_wrist_collision_monitor_v7.py": (
            'V6_MONITOR_PATH = Path(\n    r"D:/AI_JIXIEBI/模型/机械臂底座肩部大臂小臂末端电机装配_v6/j234_collision_monitor_v6.py"\n)',
            'ROOT = Path(__file__).resolve().parent\nV6_MONITOR_PATH = ROOT / "j234_collision_monitor_v6.py"',
        ),
    }
    for name, (old, new) in replacements.items():
        target = tools / name
        text = target.read_text(encoding="utf-8")
        if old not in text:
            raise RuntimeError(f"cannot make guard dependency portable: {name}")
        target.write_text(text.replace(old, new), encoding="utf-8")

    for name in (
        "build_robot_arm_v15_13_camera_up_zero.py",
        "audit_robot_arm_v15_13_deep.py",
        "build_robot_arm_rigid_links_v15.py",
        "export_robot_arm_v15_13_complete_project.py",
        "export_v15_13_world_preview_assets.py",
        "render_v15_13_verified_world_previews.py",
        "regression_v15_13_collision_guard.py",
        "package_robot_arm_v15_13_delivery.py",
    ):
        copy_file(WORKSPACE / name, tools / name)


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def write_manifest():
    files = []
    for path in sorted(DELIVERY.rglob("*")):
        relative = path.relative_to(DELIVERY)
        excluded = (
            path.name == "PROJECT_MANIFEST.json"
            or "__pycache__" in relative.parts
            or path.suffix.lower()
            in {".pyc", ".pyo", ".pyd", ".log", ".tmp", ".bak", ".fcbak"}
        )
        if path.is_file() and not excluded:
            files.append(
                {
                    "path": relative.as_posix(),
                    "bytes": path.stat().st_size,
                    "sha256": sha256(path),
                }
            )
    manifest = {
        "schema": "go-m8010-arm-v15.13-complete-delivery/1.0",
        "revision": "V15.13-camera-up-mechanical-zero-deep-audited",
        "main_fcstd": "cad/机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd",
        "main_fcstd_sha256": sha256(MAIN_FCSTD),
        "file_count_excluding_manifest": len(files),
        "total_bytes_excluding_manifest": sum(row["bytes"] for row in files),
        "files": files,
    }
    (DELIVERY / "PROJECT_MANIFEST.json").write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2), encoding="utf-8"
    )


def write_readme():
    (DELIVERY / "README_完整工程_V15_13.md").write_text(
        """# 机械臂完整工程 V15.13

## 最终状态

- J6 的真实机械零位已重定标：`J6=0°` 时 Gemini Pro 相机位于上侧。
- 未旋转连接件相对 DM-G6220 输出法兰的装配关系；仅把完整 J6 输出刚体的物理时钟位重定义为零位。
- J6 保留 `-180°..+180°`，没有把 180° 边界伪装成零位。
- 夹爪保持绕自身安装轴反转后的正确方向，未增加转接片。
- 相机使用模型自带铰链耳直接贴装连接件。

## 深度验收

- 保存零位：无碰撞。
- 全零位矩阵：231 对有效碰撞对。
- 动态扫描：503 个姿态、14,860 次窄相检测。
- J1～J6 独立全限位扫描均通过；J5/J6 网格和 32 个组合边界角均通过。
- 150 个确定性随机六轴组合中发现 5 个真实自碰撞姿态，因此完整可行域不是各关节限位的简单笛卡尔积。
- 连续碰撞守卫回归通过：J6 以 0.25° 扫掠 0°→1°→0°，且 +181° 被拒绝。

## 使用边界

位置限位、真实轴、零位、刚性 Link、visual/collision 网格以及 ClosureAngle=0° 的名义 `tcp_nominal` 可用于 ROS2、MoveIt 和 MuJoCo 的几何集成。动态/实机 TCP 尚未标定；速度、力矩、质量、质心、惯量、摩擦和 J1 线束累计圈数没有可靠实测值，工程中没有猜填。ROS2 的速度/力矩默认 0（故障安全占位）；MuJoCo 模板保留显式占位符，替换实测值前不得当作最终动力学模型。

实机或轨迹规划必须使用连续碰撞检测，不能只做端点检查。
""",
        encoding="utf-8",
    )


def main():
    for required in (ROOT, MAIN_FCSTD, RIGID_FCSTD, TREE_JSON, LIMIT_JSON):
        if not required.exists():
            raise FileNotFoundError(required)
    if DELIVERY.exists() or ZIP_PATH.exists():
        raise FileExistsError("delivery target already exists; refusing to overwrite")

    DELIVERY.mkdir(parents=True)
    cad = DELIVERY / "cad"
    copy_file(MAIN_FCSTD, cad / MAIN_FCSTD.name)
    copy_file(RIGID_FCSTD, cad / RIGID_FCSTD.name)
    for source in ROOT.glob("*.FCBak"):
        copy_file(source, cad / source.name)
    for name in ("J6末端_相机上置机械零位_v15_13.step", "J6末端_相机上置机械零位_v15_13.stl"):
        copy_file(ROOT / name, cad / name)

    source_models = DELIVERY / "source_models"
    copy_file(Path(r"D:/AI_JIXIEBI/模型/夹爪改版.stl"), source_models / "夹爪改版.stl")
    copy_file(
        Path(r"D:/AI_JIXIEBI/模型/相机_Gemini+pro相机_带支架铰链.stp"),
        source_models / "相机_Gemini+pro相机_带支架铰链.stp",
    )

    definitions = DELIVERY / "definitions"
    for name in (
        "V15_13_真实关节轴与相机上置零位.json",
        "V15_13_ROS2_MoveIt_MuJoCo_关节限位契约.json",
        "QA_V15_13_相机上置机械零位_几何复核.json",
        "QA_V15_13_整机深度碰撞与限位.json",
        "QA_V15_13_整机深度碰撞与限位.md",
        "QA_V15_13_连续碰撞守卫回归.json",
        "README_V15_13_相机上置机械零位.md",
    ):
        copy_file(ROOT / name, definitions / name)
    for name in ("rigid_link_manifest.json", "rigid_link_membership.csv", "qa_rigid_link_export.json", "README.md"):
        copy_file(RIGID / name, definitions / "rigid_links" / name)
    copy_file(TREE_JSON, definitions / "ros2" / TREE_JSON.name)
    copy_file(
        RIGID / "mujoco" / "rigid_body_tree_staging.json",
        definitions / "mujoco" / "rigid_body_tree_staging.json",
    )

    description_meshes = DELIVERY / "ros2_ws" / "src" / ROS_DESCRIPTION_PACKAGE / "meshes"
    # The legacy rigid-link visual export double-counted placements on copied
    # nested TopoShapes.  Reuse the FCStd-derived, measured-link-frame meshes
    # already audited by the MuJoCo runtime instead of repackaging bad STL data.
    copy_tree(ROS_VISUAL_SOURCE, description_meshes / "visual")
    copy_tree(RIGID / "collision", description_meshes / "collision")
    copy_tree(RIGID / "gripper_internal_zero_reference", DELIVERY / "references" / "gripper_internal_zero_reference")
    copy_tree(ROOT / "装配预览图", DELIVERY / "previews")
    copy_tree(ROOT / "三视图资产_world", DELIVERY / "references" / "world_preview_assets")
    copy_tree(ROOT / "evidence", DELIVERY / "evidence")

    tree = json.loads(TREE_JSON.read_text(encoding="utf-8"))
    limits = json.loads(LIMIT_JSON.read_text(encoding="utf-8"))
    write_urdf(tree)
    write_moveit(limits)
    write_mujoco(tree)
    package_guards()
    write_readme()
    write_manifest()

    with zipfile.ZipFile(
        ZIP_PATH, "w", compression=zipfile.ZIP_DEFLATED, compresslevel=6, allowZip64=True
    ) as archive:
        for path in sorted(DELIVERY.rglob("*")):
            if path.is_file():
                archive.write(path, Path(DELIVERY.name) / path.relative_to(DELIVERY))

    checksum = {
        "zip": str(ZIP_PATH),
        "bytes": ZIP_PATH.stat().st_size,
        "sha256": sha256(ZIP_PATH),
        "delivery_folder": str(DELIVERY),
        "main_fcstd_sha256": sha256(MAIN_FCSTD),
    }
    checksum_path = ROOT / "机械臂完整工程_V15_13_交付校验.json"
    checksum_path.write_text(json.dumps(checksum, ensure_ascii=False, indent=2), encoding="utf-8")
    print(json.dumps(checksum, ensure_ascii=False, indent=2), flush=True)


if __name__ == "__main__":
    main()

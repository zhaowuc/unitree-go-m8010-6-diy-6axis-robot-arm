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
    description = DELIVERY / "ros2_ws" / "src" / "go_m8010_arm_description"
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
    lines.append('  <material name="arm_visual"><color rgba="0.55 0.18 0.22 1"/></material>')
    for link in tree["links"]:
        lines.extend(
            [
                f'  <link name="{link}">',
                '    <visual>',
                '      <origin xyz="0 0 0" rpy="0 0 0"/>',
                f'      <geometry><mesh filename="$(arg mesh_prefix)/visual/{link}.stl" scale="0.001 0.001 0.001"/></geometry>',
                '      <material name="arm_visual"/>',
                '    </visual>',
                '    <collision>',
                '      <origin xyz="0 0 0" rpy="0 0 0"/>',
                f'      <geometry><mesh filename="$(arg mesh_prefix)/collision/{link}.stl" scale="0.001 0.001 0.001"/></geometry>',
                '    </collision>',
                '    <!-- Inertial intentionally omitted until measured mass/COM/inertia are supplied. -->',
                '  </link>',
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
    lines.append('</robot>')
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
  <exec_depend>xacro</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
</package>
""",
        encoding="utf-8",
    )
    (description / "CMakeLists.txt").write_text(
        """cmake_minimum_required(VERSION 3.8)
project(go_m8010_arm_description)
find_package(ament_cmake REQUIRED)
install(DIRECTORY urdf meshes DESTINATION share/${PROJECT_NAME})
ament_package()
""",
        encoding="utf-8",
    )
    (description / "README.md").write_text(
        """# go_m8010_arm_description V15.13

几何、真实关节轴、关节原点、零位和位置限位已经复核。J6 在 `0°` 时相机位于上侧，位置限位为 `[-π,+π]`。

`j*_effort_nm` 与 `j*_velocity_rad_s` 默认值为 `0`，这是故障安全占位，不是电机参数。完成电机、减速器、热约束与线束实测后必须显式传入；质量、质心和惯量未测量，因此 URDF 暂不含 inertial，不能作为最终动力学模型。
""",
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
        if path.is_file() and path.name != "PROJECT_MANIFEST.json":
            files.append(
                {
                    "path": path.relative_to(DELIVERY).as_posix(),
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

位置限位、真实轴、零位、刚性 Link、visual/collision 网格可用于 ROS2、MoveIt 和 MuJoCo 的几何集成。速度、力矩、质量、质心、惯量、摩擦、TCP 和 J1 线束累计圈数没有可靠实测值，工程中没有猜填。ROS2 的速度/力矩默认 0（故障安全占位）；MuJoCo 模板保留显式占位符，替换实测值前不得当作最终动力学模型。

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

    description_meshes = DELIVERY / "ros2_ws" / "src" / "go_m8010_arm_description" / "meshes"
    copy_tree(RIGID / "visual", description_meshes / "visual")
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

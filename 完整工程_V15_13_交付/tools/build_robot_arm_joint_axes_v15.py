from __future__ import annotations

"""Create a V15 copy with measured joint axes and zero-frame metadata.

The source V14 FCStd is treated as immutable.  Axis definitions are accepted
only when three independent checks agree:

1. FreeCAD joint Placement1/Placement2 centers and local-Z axes coincide.
2. Cylindrical output/flange B-Rep witnesses are coaxial with the joint.
3. A +1 degree in-memory motion rotates the driven rigid body about that line.

No STEP is exported because STEP cannot preserve these kinematic definitions.
"""

import argparse
import hashlib
import importlib.util
import json
import math
import shutil
from datetime import datetime, timezone
from pathlib import Path

import FreeCAD as App
import Part


DEFAULT_SOURCE = Path(
    r"D:/AI_JIXIEBI/备份/机械臂完整工程_V14_交接备份_20260807_164438/"
    r"V14_正式工程/机械臂完整装配_六轴_舵机柔性二指夹爪_v14.FCStd"
)
DEFAULT_OUTPUT_ROOT = Path(r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15")
DEFAULT_GUARD = Path(
    r"D:/AI_JIXIEBI/go_m8010_arm_gui/"
    r"j123456_gripper_physical_collision_guard_v14.py"
)
OUTPUT_NAME = "机械臂完整装配_六轴_真实关节轴_v15.FCStd"
AXIS_JSON_NAME = "V15_真实关节轴定义.json"
AXIS_MD_NAME = "V15_真实关节轴定义.md"
QA_JSON_NAME = "QA_V15_真实关节轴与零位.json"
SOURCE_V14_SHA256 = (
    "9156F6EC5C402ECA00FE0388E7795C4CB293481C6716997E866289C97FE5CE6A"
)
REVISION = "V15.3-measured-axis-joint-limits-and-j6-tool-interface"

LINE_TOLERANCE_MM = 1.0e-5
AXIS_TOLERANCE_DEG = 1.0e-6
ZERO_TOLERANCE_DEG = 1.0e-9
MOTION_TRANSLATION_TOLERANCE_MM = 1.0e-8
MOTION_ROTATION_TOLERANCE_DEG = 1.0e-8

JOINT_CHAIN = (
    ("J1", "J1_Revolute", "J1_Fixed_Rigid", "J1_Moving_Rigid"),
    ("J2", "J2_Revolute", "J1_Moving_Rigid", "J2_Driven_Reference_Rigid"),
    ("J3", "J3_Revolute", "J2_Driven_Reference_Rigid", "J3_Driven_Rigid"),
    ("J4", "J4_Revolute", "J3_Driven_Rigid", "J4_Driven_Rigid"),
    ("J5", "J5_Revolute", "J4_Driven_Rigid", "J5_Driven_Rigid"),
    ("J6", "J6_Revolute", "J5_Driven_Rigid", "J6_Driven_Rigid"),
)

GEOMETRY_WITNESSES = {
    "J1": ("J1_Output_Rotor_Proxy", 29.5),
    "J3": ("J3_Output_Flange_Proxy", 29.5),
    "J4": ("J4_Output_Flange_Collision_Proxy", 29.5),
    "J5": ("J5_Output_Flange_Collision_Proxy", 29.5),
    "J6": ("J6_DM_G6220_Output_Rotor_Collision_Proxy", 17.5),
}

AXIS_COLORS = {
    "J1": (0.90, 0.18, 0.16),
    "J2": (0.96, 0.48, 0.12),
    "J3": (0.95, 0.74, 0.10),
    "J4": (0.16, 0.68, 0.34),
    "J5": (0.12, 0.45, 0.92),
    "J6": (0.52, 0.24, 0.88),
}


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(4 * 1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest().upper()


def vector_list(vector: App.Vector) -> list[float]:
    return [float(vector.x), float(vector.y), float(vector.z)]


def unit(vector: App.Vector) -> App.Vector:
    result = App.Vector(vector)
    if result.Length <= 1.0e-12:
        raise ValueError("零向量不能定义坐标轴")
    result.normalize()
    return result


def dot_coordinates(vector: App.Vector, basis: tuple[App.Vector, App.Vector, App.Vector]) -> list[float]:
    return [float(vector.dot(axis)) for axis in basis]


def angle_between_deg(first: App.Vector, second: App.Vector, directed: bool = False) -> float:
    a = unit(first)
    b = unit(second)
    cosine = max(-1.0, min(1.0, float(a.dot(b))))
    if not directed:
        cosine = abs(cosine)
    return math.degrees(math.acos(cosine))


def axis_line_distance(point: App.Vector, origin: App.Vector, axis: App.Vector) -> float:
    direction = unit(axis)
    delta = point - origin
    return float((delta - direction * delta.dot(direction)).Length)


def ensure_property(obj, kind: str, name: str, group: str) -> None:
    if name not in obj.PropertiesList:
        obj.addProperty(kind, name, group)


def set_string(obj, name: str, value: str, group: str = "V15 Joint Axis") -> None:
    ensure_property(obj, "App::PropertyString", name, group)
    setattr(obj, name, str(value))


def set_bool(obj, name: str, value: bool, group: str = "V15 Joint Axis") -> None:
    ensure_property(obj, "App::PropertyBool", name, group)
    setattr(obj, name, bool(value))


def set_float(obj, name: str, value: float, group: str = "V15 Joint Axis") -> None:
    ensure_property(obj, "App::PropertyFloat", name, group)
    setattr(obj, name, float(value))


def set_angle(obj, name: str, value: float, group: str = "V15 Joint Axis") -> None:
    ensure_property(obj, "App::PropertyAngle", name, group)
    setattr(obj, name, float(value))


def set_vector(obj, name: str, value: App.Vector, group: str = "V15 Joint Axis") -> None:
    ensure_property(obj, "App::PropertyVector", name, group)
    setattr(obj, name, App.Vector(value))


def joint_measurement(doc, joint_key: str, joint_name: str, parent_name: str, child_name: str) -> dict:
    joint = doc.getObject(joint_name)
    parent = doc.getObject(parent_name)
    child = doc.getObject(child_name)
    if any(item is None for item in (joint, parent, child)):
        raise RuntimeError(f"{joint_key} 缺少关节或父子刚体对象")

    placement1 = App.Placement(joint.Placement1)
    placement2 = App.Placement(joint.Placement2)
    axis1 = unit(placement1.Rotation.multVec(App.Vector(0, 0, 1)))
    axis2 = unit(placement2.Rotation.multVec(App.Vector(0, 0, 1)))
    center_error = float((placement1.Base - placement2.Base).Length)
    axis_error = angle_between_deg(axis1, axis2)
    zero_angle = float(joint.Angle)
    angle_min = float(joint.AngleMin)
    angle_max = float(joint.AngleMax)
    enable_angle_min = bool(joint.EnableAngleMin)
    enable_angle_max = bool(joint.EnableAngleMax)
    if center_error > LINE_TOLERANCE_MM:
        raise RuntimeError(f"{joint_key} Placement1/2 中心不重合: {center_error} mm")
    if axis_error > AXIS_TOLERANCE_DEG:
        raise RuntimeError(f"{joint_key} Placement1/2 轴不一致: {axis_error} deg")
    if abs(zero_angle) > ZERO_TOLERANCE_DEG:
        raise RuntimeError(f"{joint_key} 源文件不是机械零位: {zero_angle} deg")
    if enable_angle_min != enable_angle_max:
        raise RuntimeError(f"{joint_key} 仅启用单侧角度限位，拒绝生成不完整 URDF/MJCF 范围")
    if enable_angle_min and angle_min >= angle_max:
        raise RuntimeError(f"{joint_key} 角度限位顺序错误: {angle_min}..{angle_max} deg")
    return {
        "joint_key": joint_key,
        "source_joint": joint_name,
        "source_parent_rigid": parent_name,
        "source_child_rigid": child_name,
        "origin_world_mm": vector_list(placement1.Base),
        "axis_world_at_zero": vector_list(axis1),
        "placement1_placement2_center_error_mm": center_error,
        "placement1_placement2_axis_error_deg": axis_error,
        "source_angle_deg": zero_angle,
        "source_freecad_joint_type": str(joint.JointType),
        "angle_limit_enabled": enable_angle_min and enable_angle_max,
        "angle_min_deg": angle_min,
        "angle_max_deg": angle_max,
    }


def cylindrical_candidates(obj, joint_origin: App.Vector, joint_axis: App.Vector, target_radius: float) -> list[dict]:
    axis = unit(joint_axis)
    rows = []
    for index, face in enumerate(obj.Shape.Faces, start=1):
        surface = face.Surface
        if not all(hasattr(surface, attr) for attr in ("Radius", "Axis", "Center")):
            continue
        if abs(float(surface.Radius) - target_radius) > 0.02:
            continue
        face_axis = unit(App.Vector(surface.Axis))
        alignment = abs(float(face_axis.dot(axis)))
        if alignment < 0.999999:
            continue
        center = App.Vector(surface.Center)
        delta = center - joint_origin
        axial = float(delta.dot(axis))
        radial = axis_line_distance(center, joint_origin, axis)
        rows.append(
            {
                "face_index": index,
                "radius_mm": float(surface.Radius),
                "center_world_mm": vector_list(center),
                "axis_world_undirected": vector_list(face_axis),
                "axis_alignment_abs_dot": alignment,
                "axial_station_from_joint_origin_mm": axial,
                "axis_line_residual_mm": radial,
                "face_area_mm2": float(face.Area),
            }
        )
    return rows


def geometry_witness(doc, key: str, measurement: dict) -> dict:
    object_name, radius = GEOMETRY_WITNESSES[key]
    obj = doc.getObject(object_name)
    if obj is None or not hasattr(obj, "Shape") or obj.Shape.isNull():
        raise RuntimeError(f"{key} 缺少几何轴见证对象 {object_name}")
    origin = App.Vector(*measurement["origin_world_mm"])
    axis = App.Vector(*measurement["axis_world_at_zero"])
    candidates = cylindrical_candidates(obj, origin, axis, radius)
    if not candidates:
        raise RuntimeError(f"{key} 未找到半径 {radius} mm 的同轴圆柱面")
    row = min(candidates, key=lambda item: (item["axis_line_residual_mm"], abs(item["axial_station_from_joint_origin_mm"])))
    if row["axis_line_residual_mm"] > LINE_TOLERANCE_MM:
        raise RuntimeError(f"{key} 几何轴不通过关节中心: {row['axis_line_residual_mm']} mm")
    if math.degrees(math.acos(min(1.0, row["axis_alignment_abs_dot"]))) > AXIS_TOLERANCE_DEG:
        raise RuntimeError(f"{key} 几何圆柱轴与关节轴不平行")
    row["object"] = object_name
    row["selection_rule"] = f"coaxial cylindrical face radius {radius:.3f} mm"
    return row


def j2_dual_motor_witnesses(doc, measurement: dict) -> dict:
    origin = App.Vector(*measurement["origin_world_mm"])
    axis = App.Vector(*measurement["axis_world_at_zero"])
    result = {}
    specs = (
        ("J2A", "J2_Left_Joint_Motor_GO_M8010_6", min),
        ("J2B", "J2_Right_Joint_Motor_GO_M8010_6", max),
    )
    for role, object_name, selector in specs:
        obj = doc.getObject(object_name)
        if obj is None:
            raise RuntimeError(f"缺少 {role} 电机对象 {object_name}")
        rows = cylindrical_candidates(obj, origin, axis, 29.0)
        rows = [row for row in rows if row["axis_line_residual_mm"] <= LINE_TOLERANCE_MM]
        if not rows:
            raise RuntimeError(f"{role} 未找到真实输出端 Ø58 圆柱轴见证")
        chosen = selector(rows, key=lambda item: item["axial_station_from_joint_origin_mm"])
        chosen["object"] = object_name
        chosen["selection_rule"] = "outermost coaxial output-face cylinder radius 29.000 mm"
        result[role] = chosen

    center_a = App.Vector(*result["J2A"]["center_world_mm"])
    center_b = App.Vector(*result["J2B"]["center_world_mm"])
    midpoint = (center_a + center_b) * 0.5
    result["common_axis_checks"] = {
        "j2a_axis_line_residual_mm": axis_line_distance(center_a, origin, axis),
        "j2b_axis_line_residual_mm": axis_line_distance(center_b, origin, axis),
        "flange_midpoint_to_joint_origin_mm": float((midpoint - origin).Length),
        "flange_center_separation_mm": float((center_b - center_a).Length),
        "kinematic_dof_count": 1,
        "drive_note": "J2A/J2B are coaxial synchronized actuators for one J2 revolute DOF; physical motor command signs require commissioning.",
    }
    if result["common_axis_checks"]["flange_midpoint_to_joint_origin_mm"] > LINE_TOLERANCE_MM:
        raise RuntimeError("J2A/J2B 输出端中心的中点不等于 J2 关节原点")
    return result


def placement_error(first: App.Placement, second: App.Placement) -> tuple[float, float]:
    translation = float((first.Base - second.Base).Length)
    delta = first.Rotation.inverted().multiply(second.Rotation)
    rotation = abs(math.degrees(float(delta.Angle)))
    rotation = min(rotation, abs(360.0 - rotation))
    return translation, rotation


def rotation_about(origin: App.Vector, axis: App.Vector, angle_deg: float) -> App.Placement:
    rotation = App.Rotation(unit(axis), angle_deg)
    return App.Placement(origin - rotation.multVec(origin), rotation)


def load_guard(path: Path):
    if not path.is_file():
        raise FileNotFoundError(f"缺少 V14 运动学守卫用于独立小角度验证: {path}")
    spec = importlib.util.spec_from_file_location("_v15_axis_measure_guard", str(path))
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(module)
    return module


def finite_motion_checks(doc, guard, measurements: dict[str, dict]) -> dict:
    zero = {f"j{index}_deg": 0.0 for index in range(1, 6)}
    results = {}
    for index, (key, joint_name, _parent_name, child_name) in enumerate(JOINT_CHAIN[:5], start=1):
        guard._apply_j12345_explicit_unchecked(doc, zero)
        child = doc.getObject(child_name)
        before = App.Placement(child.Placement)
        target = dict(zero)
        target[f"j{index}_deg"] = 1.0
        guard._apply_j12345_explicit_unchecked(doc, target)
        after = App.Placement(child.Placement)
        origin = App.Vector(*measurements[key]["origin_world_mm"])
        axis = App.Vector(*measurements[key]["axis_world_at_zero"])
        expected = rotation_about(origin, axis, 1.0).multiply(before)
        translation_error, rotation_error = placement_error(expected, after)
        delta = after.multiply(before.inverse())
        measured_axis = unit(delta.Rotation.Axis)
        if measured_axis.dot(axis) < 0.0:
            measured_axis = -measured_axis
        results[key] = {
            "requested_deg": 1.0,
            "measured_delta_axis_world": vector_list(measured_axis),
            "axis_error_deg": angle_between_deg(measured_axis, axis, directed=True),
            "expected_placement_translation_error_mm": translation_error,
            "expected_placement_rotation_error_deg": rotation_error,
        }

    guard._apply_j12345_explicit_unchecked(doc, zero)
    guard._v13["_propagate_r6"](doc, 0.0)
    child = doc.getObject("J6_Driven_Rigid")
    before = App.Placement(child.Placement)
    guard._v13["_propagate_r6"](doc, 1.0)
    after = App.Placement(child.Placement)
    origin = App.Vector(*measurements["J6"]["origin_world_mm"])
    axis = App.Vector(*measurements["J6"]["axis_world_at_zero"])
    expected = rotation_about(origin, axis, 1.0).multiply(before)
    translation_error, rotation_error = placement_error(expected, after)
    delta = after.multiply(before.inverse())
    measured_axis = unit(delta.Rotation.Axis)
    if measured_axis.dot(axis) < 0.0:
        measured_axis = -measured_axis
    results["J6"] = {
        "requested_deg": 1.0,
        "measured_delta_axis_world": vector_list(measured_axis),
        "axis_error_deg": angle_between_deg(measured_axis, axis, directed=True),
        "expected_placement_translation_error_mm": translation_error,
        "expected_placement_rotation_error_deg": rotation_error,
    }

    guard._v13["_propagate_r6"](doc, 0.0)
    guard._apply_j12345_explicit_unchecked(doc, zero)
    for key, row in results.items():
        if row["axis_error_deg"] > AXIS_TOLERANCE_DEG:
            raise RuntimeError(f"{key} +1°实测旋转轴与定义轴不一致")
        if row["expected_placement_translation_error_mm"] > MOTION_TRANSLATION_TOLERANCE_MM:
            raise RuntimeError(f"{key} +1°运动平移残差超限")
        if row["expected_placement_rotation_error_deg"] > MOTION_ROTATION_TOLERANCE_DEG:
            raise RuntimeError(f"{key} +1°运动旋转残差超限")
    return results


def frame_record(name: str, origin: App.Vector, x_axis: App.Vector, y_axis: App.Vector, z_axis: App.Vector) -> dict:
    x = unit(x_axis)
    y = unit(y_axis)
    z = unit(z_axis)
    determinant = float(x.cross(y).dot(z))
    orthogonality = max(abs(float(x.dot(y))), abs(float(y.dot(z))), abs(float(z.dot(x))))
    norm_error = max(abs(x.Length - 1.0), abs(y.Length - 1.0), abs(z.Length - 1.0))
    if determinant < 1.0 - 1.0e-9 or orthogonality > 1.0e-9 or norm_error > 1.0e-9:
        raise RuntimeError(f"{name} 坐标系不是右手正交单位基")
    return {
        "name": name,
        "origin_world_mm": vector_list(origin),
        "x_axis_world_at_zero": vector_list(x),
        "y_axis_world_at_zero": vector_list(y),
        "z_axis_world_at_zero": vector_list(z),
        "right_handed_determinant": determinant,
        "maximum_orthogonality_error": orthogonality,
        "maximum_unit_norm_error": norm_error,
    }


def frame_from_placement(name: str, placement: App.Placement) -> dict:
    rotation = placement.Rotation
    return frame_record(
        name,
        placement.Base,
        rotation.multVec(App.Vector(1, 0, 0)),
        rotation.multVec(App.Vector(0, 1, 0)),
        rotation.multVec(App.Vector(0, 0, 1)),
    )


def build_engineering_frames(doc, measurements: dict[str, dict]) -> dict[str, dict]:
    origins = {key: App.Vector(*row["origin_world_mm"]) for key, row in measurements.items()}
    axes = {key: App.Vector(*row["axis_world_at_zero"]) for key, row in measurements.items()}

    # Link frames follow the URDF/MJCF convention: every child-link origin is
    # located at its incoming physical joint center at the measured zero pose.
    # Frame rotations are fixed only by measured neighboring axes, so the
    # requested categorical axes remain exact instead of being visual guesses.
    pitch_y = unit(axes["J2"])
    pitch_z = unit(axes["J1"])
    pitch_x = unit(pitch_y.cross(pitch_z))

    link4_x = unit(axes["J5"])
    link4_y = unit(axes["J4"])
    link4_z = unit(link4_x.cross(link4_y))

    wrist_x = unit(axes["J5"])
    wrist_z = unit(axes["J6"])
    wrist_y = unit(wrist_z.cross(wrist_x))

    return {
        "base_link": frame_record("base_link", origins["J1"], pitch_x, pitch_y, pitch_z),
        "link1": frame_record("link1", origins["J1"], pitch_x, pitch_y, pitch_z),
        "link2": frame_record("link2", origins["J2"], pitch_x, pitch_y, pitch_z),
        "link3": frame_record("link3", origins["J3"], pitch_x, pitch_y, pitch_z),
        "link4": frame_record("link4", origins["J4"], link4_x, link4_y, link4_z),
        "link5": frame_record("link5", origins["J5"], wrist_x, wrist_y, wrist_z),
        "link6": frame_record("link6", origins["J6"], wrist_x, wrist_y, wrist_z),
    }


def frame_basis(frame: dict) -> tuple[App.Vector, App.Vector, App.Vector]:
    return (
        App.Vector(*frame["x_axis_world_at_zero"]),
        App.Vector(*frame["y_axis_world_at_zero"]),
        App.Vector(*frame["z_axis_world_at_zero"]),
    )


def zero_transform_parent_to_child(parent: dict, child: dict) -> dict:
    parent_basis = frame_basis(parent)
    child_basis = frame_basis(child)
    parent_origin = App.Vector(*parent["origin_world_mm"])
    child_origin = App.Vector(*child["origin_world_mm"])
    translation_mm = dot_coordinates(child_origin - parent_origin, parent_basis)
    rotation = [
        [float(parent_basis[row].dot(child_basis[col])) for col in range(3)]
        for row in range(3)
    ]

    r00, r01, r02 = rotation[0]
    r10, r11, r12 = rotation[1]
    r20, r21, r22 = rotation[2]
    pitch = math.asin(max(-1.0, min(1.0, -r20)))
    if abs(math.cos(pitch)) > 1.0e-12:
        roll = math.atan2(r21, r22)
        yaw = math.atan2(r10, r00)
    else:
        roll = 0.0
        yaw = math.atan2(-r01, r11)

    trace = r00 + r11 + r22
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        qw = 0.25 * scale
        qx = (r21 - r12) / scale
        qy = (r02 - r20) / scale
        qz = (r10 - r01) / scale
    elif r00 > r11 and r00 > r22:
        scale = math.sqrt(1.0 + r00 - r11 - r22) * 2.0
        qw = (r21 - r12) / scale
        qx = 0.25 * scale
        qy = (r01 + r10) / scale
        qz = (r02 + r20) / scale
    elif r11 > r22:
        scale = math.sqrt(1.0 + r11 - r00 - r22) * 2.0
        qw = (r02 - r20) / scale
        qx = (r01 + r10) / scale
        qy = 0.25 * scale
        qz = (r12 + r21) / scale
    else:
        scale = math.sqrt(1.0 + r22 - r00 - r11) * 2.0
        qw = (r10 - r01) / scale
        qx = (r02 + r20) / scale
        qy = (r12 + r21) / scale
        qz = 0.25 * scale
    quaternion_norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    quaternion_xyzw = [value / quaternion_norm for value in (qx, qy, qz, qw)]

    return {
        "semantics": "pose of child link frame in parent link frame at CAD mechanical zero",
        "xyz_mm": translation_mm,
        "xyz_m": [value / 1000.0 for value in translation_mm],
        "rotation_matrix_row_major": rotation,
        "rpy_rad_urdf_fixed_axis": [roll, pitch, yaw],
        "quaternion_xyzw": quaternion_xyzw,
        "quaternion_wxyz_mujoco": [quaternion_xyzw[3], *quaternion_xyzw[:3]],
    }


def make_joint_rows(measurements: dict[str, dict], frames: dict[str, dict], j2_witnesses: dict) -> list[dict]:
    specs = (
        ("J1", "base_link", "link1", "Z", "J1", None),
        ("J2A", "link1", "link2", "Y", "J2", "J2A"),
        ("J2B", "link1", "link2", "Y", "J2", "J2B"),
        ("J3", "link2", "link3", "Y", "J3", None),
        ("J4", "link3", "link4", "Y", "J4", None),
        ("J5", "link4", "link5", "X", "J5", None),
        ("J6", "link5", "link6", "Z", "J6", None),
    )
    expected_vectors = {"X": [1.0, 0.0, 0.0], "Y": [0.0, 1.0, 0.0], "Z": [0.0, 0.0, 1.0]}
    rows = []
    for name, parent, child, canonical, source_key, motor_role in specs:
        source = measurements[source_key]
        origin = App.Vector(*source["origin_world_mm"])
        axis = App.Vector(*source["axis_world_at_zero"])
        parent_frame = frames[parent]
        child_frame = frames[child]
        parent_origin = App.Vector(*parent_frame["origin_world_mm"])
        parent_basis = frame_basis(parent_frame)
        child_basis = frame_basis(child_frame)
        local_axis_parent = dot_coordinates(axis, parent_basis)
        local_axis_child = dot_coordinates(axis, child_basis)
        expected = expected_vectors[canonical]
        parent_residual = math.sqrt(sum((a - b) ** 2 for a, b in zip(local_axis_parent, expected)))
        child_residual = math.sqrt(sum((a - b) ** 2 for a, b in zip(local_axis_child, expected)))
        if max(parent_residual, child_residual) > 1.0e-9:
            raise RuntimeError(
                f"{name} 精确轴向量不能同时映射到父/子坐标系 +{canonical}: "
                f"parent={local_axis_parent}, child={local_axis_child}"
            )
        zero_transform = zero_transform_parent_to_child(parent_frame, child_frame)
        child_origin = App.Vector(*child_frame["origin_world_mm"])
        if (child_origin - origin).Length > LINE_TOLERANCE_MM:
            raise RuntimeError(f"{name} 子 link 原点不在实际关节轴心")
        limited = bool(source["angle_limit_enabled"])
        export_joint_type = "revolute" if limited else "continuous"
        expected_joint_type = "continuous" if source_key == "J1" else "revolute"
        if export_joint_type != expected_joint_type:
            raise RuntimeError(
                f"{name} FreeCAD 限位状态与预期关节类型冲突: "
                f"expected={expected_joint_type}, actual={export_joint_type}"
            )
        if name == "J6" and (
            not limited
            or abs(source["angle_min_deg"] + 180.0) > ZERO_TOLERANCE_DEG
            or abs(source["angle_max_deg"] - 180.0) > ZERO_TOLERANCE_DEG
        ):
            raise RuntimeError("J6 必须是带 -180..+180 deg 限位的 revolute 关节")
        position_limit_deg = (
            {"lower": source["angle_min_deg"], "upper": source["angle_max_deg"]}
            if limited
            else None
        )
        position_limit_rad = (
            {
                "lower": math.radians(source["angle_min_deg"]),
                "upper": math.radians(source["angle_max_deg"]),
            }
            if limited
            else None
        )
        row = {
            "name": name,
            "parent_link": parent,
            "child_link": child,
            "canonical_axis": canonical,
            "axis_vector_in_parent_frame": local_axis_parent,
            "axis_vector_in_parent_link_frame": local_axis_parent,
            "axis_vector_in_joint_child_frame": local_axis_child,
            "axis_world_at_zero": vector_list(axis),
            "joint_origin_world_mm": vector_list(origin),
            "joint_origin_in_parent_frame_mm": dot_coordinates(origin - parent_origin, parent_basis),
            "child_link_frame_origin_matches_joint_center_mm": (child_origin - origin).Length,
            "parent_to_child_at_mechanical_zero": zero_transform,
            "joint_limit_definition": {
                "source": f"{source['source_joint']}.AngleMin/AngleMax + EnableAngleMin/EnableAngleMax",
                "position_limited": limited,
                "position_limit_deg": position_limit_deg,
                "position_limit_rad": position_limit_rad,
                "velocity_limit_rad_s": None,
                "effort_limit_nm": None,
                "velocity_effort_status": "PENDING_MOTOR_AND_REDUCER_ENGINEERING_DATA; do not guess",
            },
            "ros2_urdf": {
                "joint_type": export_joint_type,
                "origin_xyz_m": zero_transform["xyz_m"],
                "origin_rpy_rad": zero_transform["rpy_rad_urdf_fixed_axis"],
                "axis_xyz_in_joint_frame": local_axis_child,
                "position_limit_rad": position_limit_rad,
                "effort_limit_nm": None,
                "velocity_limit_rad_s": None,
                "emission_status": "POSITION_SEMANTICS_READY; URDF <limit> still requires verified effort and velocity",
            },
            "mujoco_mjcf": {
                "body_pos_m_in_parent": zero_transform["xyz_m"],
                "body_quat_wxyz_in_parent": zero_transform["quaternion_wxyz_mujoco"],
                "joint_type": "hinge",
                "joint_pos_m_in_child_body": [0.0, 0.0, 0.0],
                "joint_axis_in_child_body": local_axis_child,
                "joint_ref_rad": 0.0,
                "joint_limited": limited,
                "joint_range_rad": (
                    [position_limit_rad["lower"], position_limit_rad["upper"]]
                    if position_limit_rad is not None
                    else None
                ),
            },
            "mechanical_zero_deg": 0.0,
            "zero_definition": "USER_SPECIFIED_ACTUAL_MEASUREMENT + V14 saved mechanical assembly zero",
            "hardware_encoder_zero_status": "PENDING_LOW_TORQUE_PHYSICAL_COMMISSIONING",
            "measurement_status": "DEFINED_FROM_FREECAD_GEOMETRY_AND_FINITE_MOTION",
            "source_joint": source["source_joint"],
            "kinematic_dof": "J2" if name in ("J2A", "J2B") else name,
        }
        if motor_role:
            row["motor_output_flange_witness"] = j2_witnesses[motor_role]
            row["dual_motor_note"] = j2_witnesses["common_axis_checks"]["drive_note"]
        else:
            row["motor_output_flange_witness"] = source.get("geometry_witness")
        rows.append(row)
    return rows


def downstream_kinematic_chain(joint_rows: list[dict]) -> list[dict]:
    by_name = {row["name"]: row for row in joint_rows}
    selections = (
        ("J1", "J1", ["J1"]),
        ("J2", "J2A", ["J2A", "J2B"]),
        ("J3", "J3", ["J3"]),
        ("J4", "J4", ["J4"]),
        ("J5", "J5", ["J5"]),
        ("J6", "J6", ["J6"]),
    )
    chain = []
    for simulation_name, source_name, actuators in selections:
        source = by_name[source_name]
        chain.append(
            {
                "joint_name": simulation_name,
                "parent_link": source["parent_link"],
                "child_link": source["child_link"],
                "physical_actuators": actuators,
                "canonical_axis": source["canonical_axis"],
                "axis_world_at_zero": source["axis_world_at_zero"],
                "joint_limit_definition": source["joint_limit_definition"],
                "parent_to_child_at_mechanical_zero": source["parent_to_child_at_mechanical_zero"],
                "ros2_urdf": source["ros2_urdf"],
                "mujoco_mjcf": source["mujoco_mjcf"],
            }
        )
    return chain


def measure_j6_end_effector_interface(doc, frames: dict[str, dict], joint_rows: list[dict]) -> dict:
    j6_row = next(row for row in joint_rows if row["name"] == "J6")
    origin = App.Vector(*j6_row["joint_origin_world_mm"])
    axis = unit(App.Vector(*j6_row["axis_world_at_zero"]))
    interface = doc.getObject("J6_Output_Interface_Frame")
    connector = doc.getObject("J6_Gripper_Connector_Collision_Proxy")
    tool = doc.getObject("J6_Gripper_Assembly")
    status = doc.getObject("V14_Gripper_Physical_Test_Status")
    controller = doc.getObject("Gripper_Servo_Controller")
    mount_3hole = doc.getObject("J6_DM_G6220_Mount_3xM4_PCD60_Axes")
    if None in (interface, connector, tool, status, controller, mount_3hole):
        raise RuntimeError("V14 末端缺少 J6/DM-G6220/夹爪接口对象")

    interface_origin = App.Vector(*interface.OriginWorldAtZero)
    interface_axis = unit(App.Vector(*interface.AxisWorldAtZero))
    origin_error = (interface_origin - origin).Length
    axis_error = angle_between_deg(interface_axis, axis, directed=True)
    if origin_error > LINE_TOLERANCE_MM or axis_error > AXIS_TOLERANCE_DEG:
        raise RuntimeError("J6_Output_Interface_Frame 与 J6 实际轴不一致")

    connector_planar_witnesses = []
    for face_index, face in enumerate(connector.Shape.Faces, start=1):
        surface = face.Surface
        if getattr(surface, "TypeId", "") != "Part::GeomPlane":
            continue
        normal = unit(surface.Axis)
        alignment = abs(float(normal.dot(axis)))
        if alignment < 0.999999:
            continue
        stations = [float((vertex.Point - origin).dot(axis)) for vertex in face.Vertexes]
        connector_planar_witnesses.append(
            {
                "face_index": face_index,
                "axis_normal_alignment_abs_dot": alignment,
                "axial_station_mm": sum(stations) / len(stations),
                "axial_station_span_mm": max(stations) - min(stations),
                "face_area_mm2": float(face.Area),
            }
        )
    if not connector_planar_witnesses:
        raise RuntimeError("夹爪连接件未找到与 J6 轴垂直的实际安装平面")
    contact_plane = min(connector_planar_witnesses, key=lambda row: abs(row["axial_station_mm"]))
    connector_stations = [float((vertex.Point - origin).dot(axis)) for vertex in connector.Shape.Vertexes]

    connector_source_frame = frame_from_placement(
        "connector_source_frame",
        tool.ConnectorSourceToWorldAtJ6Zero,
    )
    gripper_source_frame = frame_from_placement(
        "gripper_source_frame",
        tool.GripperSourceToWorldAtJ6Zero,
    )
    link6_frame = frames["link6"]
    connector_source_pose = zero_transform_parent_to_child(link6_frame, connector_source_frame)
    gripper_source_pose = zero_transform_parent_to_child(link6_frame, gripper_source_frame)

    result = {
        "status": "DEFINED_FROM_V14_FREECAD_INTERFACE_OBJECTS_AND_CONNECTOR_BREP",
        "j6_output_mount_frame": {
            "frame_name": "gripper_mount",
            "origin_world_mm": vector_list(interface_origin),
            "axis_world_outward": vector_list(interface_axis),
            "origin_to_j6_joint_center_error_mm": origin_error,
            "axis_to_j6_error_deg": axis_error,
            "interface_geometry": str(interface.InterfaceGeometry),
            "dm_output_rotor_brep_witness": j6_row["motor_output_flange_witness"],
        },
        "connector_seat": {
            "source_object": connector.Name,
            "contact_plane_brep_witness": contact_plane,
            "seat_gap_mm_from_v14_freecad_qa": float(status.SeatGap.Value),
            "connector_axial_extent_from_mount_mm": [min(connector_stations), max(connector_stations)],
            "orientation_record": str(status.ConnectorOrientation),
        },
        "six_m4_interface": {
            "dm_output_pattern": "6xM4 / nominal PCD28",
            "maximum_connector_to_dm_hole_axis_error_mm": float(status.MaxSixM4AxisError.Value),
            "interpretation": "source connector clearance-hole pattern is not mathematically exact to nominal DM PCD28; preserve measured mismatch",
            "fastener_status": str(status.FastenerStatus),
        },
        "three_m4_stator_mount": {
            "source_object": mount_3hole.Name,
            "pattern": "3xM4 / nominal PCD60",
            "maximum_axis_error_mm": float(mount_3hole.MaxAxisErrorMM),
            "interpretation": "preserved FreeCAD registration residual; do not round to zero",
        },
        "clevis_tongue_interface": {
            "fit_record": str(status.ClevisFit),
            "maximum_cross_axis_error_mm": float(status.MaxClevisAxisError.Value),
            "radial_clearance_remaining_mm": float(status.ClevisRadialClearanceRemaining.Value),
            "nominal_width_clearance_status": "ZERO_NOMINAL_CLEARANCE: recorded slot and tongue are both 3.000 mm",
            "fastener_status": str(status.FastenerStatus),
        },
        "fixed_joint_contract": {
            "joint_name": "J6_to_gripper_mount",
            "joint_type": "fixed",
            "parent_link": "link6",
            "child_link": "gripper_mount",
            "origin_xyz_m": [0.0, 0.0, 0.0],
            "origin_rpy_rad": [0.0, 0.0, 0.0],
            "reason": "gripper_mount is defined coincident with the measured DM-G6220 output interface, not with the raw mesh origin",
        },
        "source_geometry_poses_in_gripper_mount_at_zero": {
            "connector_source": connector_source_pose,
            "gripper_source": gripper_source_pose,
        },
        "rigid_topology": {
            "owner": "J6_Driven_Rigid",
            "fixed_members": [
                "J6_DM_G6220_Output_Rotor_Collision_Proxy",
                "J6_Gripper_Connector_Collision_Proxy",
                "Gripper_Fixed_Frame_Collision_Proxy",
                "Gripper_Servo_Collision_Proxy",
            ],
            "tool_topology_record": str(tool.Topology),
        },
        "gripper_internal_joint": {
            "name": "gripper_closure",
            "cad_zero_deg": float(controller.ClosureAngle.Value),
            "cad_working_range_deg": [float(controller.AngleMin.Value), float(controller.AngleMax.Value)],
            "positive_convention": str(controller.PositiveConvention),
            "status": str(controller.RangeStatus),
        },
        "tcp_status": "NOT_DEFINED: fingertip/tool-center-point requires a separate measured task-frame decision",
        "mujoco_collision_contract_status": "PENDING: designed bolted/seated/hinge contact exclusions must be transferred from V14 collision audit before simulation release",
    }
    if abs(contact_plane["axial_station_mm"]) > LINE_TOLERANCE_MM:
        raise RuntimeError("夹爪连接件安装平面没有贴合 J6/DM-G6220 输出原点")
    if contact_plane["axis_normal_alignment_abs_dot"] < 1.0 - 1.0e-9:
        raise RuntimeError("夹爪连接件安装平面法向与 J6 轴不平行")
    return result


def add_reference_objects(
    doc,
    joint_rows: list[dict],
    frames: dict[str, dict],
    source_hash: str,
    end_effector_interface: dict,
) -> None:
    group = doc.addObject("App::DocumentObjectGroup", "V15_Kinematic_Definition")
    group.Label = "V15 真实关节轴、零位与工程坐标系（非物理参考）"

    registry = doc.addObject("App::FeaturePython", "V15_Joint_Axis_Registry")
    registry.Label = "V15真实关节轴注册表 / 几何与有限转动实测"
    set_string(registry, "Revision", REVISION, "V15 Registry")
    set_string(registry, "SourceV14SHA256", source_hash, "V15 Registry")
    set_string(
        registry,
        "Method",
        "coincident FreeCAD joint placements + coaxial BRep output/flange cylinders + in-memory +1 degree driven-rigid rotation",
        "V15 Registry",
    )
    set_string(registry, "CanonicalAxes", "J1=Z; J2A/J2B=Y(one DOF); J3=Y; J4=Y; J5=X; J6=Z", "V15 Registry")
    set_string(
        registry,
        "JointTypesAndPositionRanges",
        "J1=continuous; J2=-170..170deg; J3=-170..170deg; J4=-116..159deg; J5=-70.6..151.2deg; J6=revolute -180..180deg",
        "V15 Registry",
    )
    set_string(registry, "ZeroPolicy", "all CAD mechanical joint angles are 0 deg at the certified V14 saved pose", "V15 Registry")
    set_string(
        registry,
        "FrameContract",
        "ROS2 URDF / MoveIt / MuJoCo: each child-link frame origin is its incoming physical joint center at CAD mechanical zero",
        "V15 Registry",
    )
    set_string(registry, "HardwareZeroBoundary", "encoder electrical zero and motor command signs still require low-torque physical commissioning", "V15 Registry")
    set_bool(registry, "ReferenceOnly", True, "V15 Registry")
    set_bool(registry, "CollisionEnabled", False, "V15 Registry")
    set_bool(registry, "MassEnabled", False, "V15 Registry")
    group.addObject(registry)

    unique_axis_rows = {}
    for row in joint_rows:
        unique_axis_rows.setdefault(row["kinematic_dof"], row)

    for key, row in unique_axis_rows.items():
        axis = unit(App.Vector(*row["axis_world_at_zero"]))
        origin = App.Vector(*row["joint_origin_world_mm"])
        length = 180.0 if key == "J2" else 140.0
        shape = Part.makeCompound(
            [
                Part.makeCylinder(1.5, length, origin - axis * (length * 0.5), axis),
                Part.makeSphere(3.2, origin),
            ]
        )
        axis_obj = doc.addObject("Part::Feature", f"V15_{key}_Actual_Axis_Reference")
        axis_obj.Label = f"V15 {key} 实际旋转轴 / 几何+有限转动验证 / 非物理"
        axis_obj.Shape = shape
        set_string(axis_obj, "Joint", key)
        set_vector(axis_obj, "OriginWorldAtZero", origin)
        set_vector(axis_obj, "AxisWorldAtZero", axis)
        set_string(axis_obj, "CanonicalAxis", row["canonical_axis"])
        set_bool(axis_obj, "ReferenceOnly", True)
        set_bool(axis_obj, "CollisionEnabled", False)
        set_bool(axis_obj, "MassEnabled", False)
        set_bool(axis_obj, "ExportEnabled", False)
        try:
            axis_obj.ViewObject.ShapeColor = AXIS_COLORS[key]
            axis_obj.ViewObject.LineColor = AXIS_COLORS[key]
            axis_obj.ViewObject.Transparency = 8
        except Exception:
            pass
        group.addObject(axis_obj)

    for row in joint_rows:
        record = doc.addObject("App::FeaturePython", f"V15_Axis_Record_{row['name']}")
        record.Label = f"{row['name']} / {row['parent_link']} → {row['child_link']} / +{row['canonical_axis']} / 0°"
        set_string(record, "JointName", row["name"])
        set_string(record, "ParentLink", row["parent_link"])
        set_string(record, "ChildLink", row["child_link"])
        set_string(record, "CanonicalAxis", row["canonical_axis"])
        set_vector(record, "AxisVectorWorldAtZero", App.Vector(*row["axis_world_at_zero"]))
        set_vector(record, "AxisVectorInParentLinkFrame", App.Vector(*row["axis_vector_in_parent_link_frame"]))
        set_vector(record, "AxisVectorInJointChildFrame", App.Vector(*row["axis_vector_in_joint_child_frame"]))
        set_vector(record, "JointOriginWorldAtZero", App.Vector(*row["joint_origin_world_mm"]))
        set_vector(
            record,
            "ParentToChildZeroXYZmm",
            App.Vector(*row["parent_to_child_at_mechanical_zero"]["xyz_mm"]),
        )
        set_vector(
            record,
            "URDFOriginRPYrad",
            App.Vector(*row["parent_to_child_at_mechanical_zero"]["rpy_rad_urdf_fixed_axis"]),
        )
        set_angle(record, "MechanicalZero", 0.0)
        limit = row["joint_limit_definition"]
        set_string(record, "ROS2JointType", row["ros2_urdf"]["joint_type"])
        set_bool(record, "PositionLimitEnabled", limit["position_limited"])
        if limit["position_limited"]:
            set_angle(record, "PositionLimitMin", limit["position_limit_deg"]["lower"])
            set_angle(record, "PositionLimitMax", limit["position_limit_deg"]["upper"])
        set_string(record, "VelocityEffortStatus", limit["velocity_effort_status"])
        set_string(record, "MeasurementStatus", row["measurement_status"])
        set_string(record, "ZeroDefinition", row["zero_definition"])
        set_string(record, "HardwareEncoderZeroStatus", row["hardware_encoder_zero_status"])
        set_string(record, "KinematicDOF", row["kinematic_dof"])
        set_bool(record, "ReferenceOnly", True)
        set_bool(record, "CollisionEnabled", False)
        set_bool(record, "MassEnabled", False)
        group.addObject(record)

    for name, frame in frames.items():
        obj = doc.addObject("App::FeaturePython", f"V15_Frame_{name}")
        obj.Label = f"V15工程坐标系 {name} / 零位"
        set_string(obj, "FrameName", name, "V15 Engineering Frame")
        set_vector(obj, "OriginWorldAtZero", App.Vector(*frame["origin_world_mm"]), "V15 Engineering Frame")
        set_vector(obj, "XAxisWorldAtZero", App.Vector(*frame["x_axis_world_at_zero"]), "V15 Engineering Frame")
        set_vector(obj, "YAxisWorldAtZero", App.Vector(*frame["y_axis_world_at_zero"]), "V15 Engineering Frame")
        set_vector(obj, "ZAxisWorldAtZero", App.Vector(*frame["z_axis_world_at_zero"]), "V15 Engineering Frame")
        set_bool(obj, "ReferenceOnly", True, "V15 Engineering Frame")
        set_bool(obj, "CollisionEnabled", False, "V15 Engineering Frame")
        set_bool(obj, "MassEnabled", False, "V15 Engineering Frame")
        group.addObject(obj)

    ee = doc.addObject("App::FeaturePython", "V15_J6_End_Effector_Interface")
    ee.Label = "V15 J6 / DM-G6220 / 夹爪连接接口实测记录"
    mount = end_effector_interface["j6_output_mount_frame"]
    seat = end_effector_interface["connector_seat"]
    holes = end_effector_interface["six_m4_interface"]
    stator_mount = end_effector_interface["three_m4_stator_mount"]
    clevis = end_effector_interface["clevis_tongue_interface"]
    set_vector(ee, "MountOriginWorldAtZero", App.Vector(*mount["origin_world_mm"]), "V15 J6 Tool Interface")
    set_vector(ee, "MountAxisWorldOutward", App.Vector(*mount["axis_world_outward"]), "V15 J6 Tool Interface")
    set_float(ee, "ConnectorContactPlaneStationMM", seat["contact_plane_brep_witness"]["axial_station_mm"], "V15 J6 Tool Interface")
    set_float(ee, "ConnectorSeatGapMM", seat["seat_gap_mm_from_v14_freecad_qa"], "V15 J6 Tool Interface")
    set_float(ee, "MaxSixM4AxisErrorMM", holes["maximum_connector_to_dm_hole_axis_error_mm"], "V15 J6 Tool Interface")
    set_float(ee, "MaxThreeM4StatorAxisErrorMM", stator_mount["maximum_axis_error_mm"], "V15 J6 Tool Interface")
    set_float(ee, "MaxClevisCrossAxisErrorMM", clevis["maximum_cross_axis_error_mm"], "V15 J6 Tool Interface")
    set_float(ee, "ClevisRadialClearanceRemainingMM", clevis["radial_clearance_remaining_mm"], "V15 J6 Tool Interface")
    set_string(ee, "ClevisNominalClearanceStatus", clevis["nominal_width_clearance_status"], "V15 J6 Tool Interface")
    set_string(ee, "MuJoCoCollisionContractStatus", end_effector_interface["mujoco_collision_contract_status"], "V15 J6 Tool Interface")
    set_string(ee, "FixedJointContract", "link6 -> gripper_mount / fixed / identity at mechanical zero", "V15 J6 Tool Interface")
    set_string(ee, "TCPStatus", end_effector_interface["tcp_status"], "V15 J6 Tool Interface")
    set_bool(ee, "ReferenceOnly", True, "V15 J6 Tool Interface")
    set_bool(ee, "CollisionEnabled", False, "V15 J6 Tool Interface")
    set_bool(ee, "MassEnabled", False, "V15 J6 Tool Interface")
    group.addObject(ee)

    # Annotate the existing FreeCAD joint objects without changing their axes,
    # offsets, placements, limits, or angles.
    joint_row_by_source = {}
    for row in joint_rows:
        joint_row_by_source.setdefault(row["source_joint"], row)
    for source_joint, row in joint_row_by_source.items():
        joint = doc.getObject(source_joint)
        set_string(joint, "V15ParentLink", row["parent_link"])
        set_string(joint, "V15ChildLink", row["child_link"])
        set_string(joint, "V15CanonicalAxis", row["canonical_axis"])
        set_vector(joint, "V15AxisWorldAtZero", App.Vector(*row["axis_world_at_zero"]))
        set_vector(joint, "V15OriginWorldAtZero", App.Vector(*row["joint_origin_world_mm"]))
        set_angle(joint, "V15MechanicalZero", 0.0)
        set_string(joint, "V15ROS2JointType", row["ros2_urdf"]["joint_type"])
        set_bool(joint, "V15PositionLimitEnabled", row["joint_limit_definition"]["position_limited"])
        if row["joint_limit_definition"]["position_limited"]:
            set_angle(joint, "V15PositionLimitMin", row["joint_limit_definition"]["position_limit_deg"]["lower"])
            set_angle(joint, "V15PositionLimitMax", row["joint_limit_definition"]["position_limit_deg"]["upper"])
        set_string(joint, "V15DefinitionStatus", row["measurement_status"])
    j2 = doc.getObject("J2_Revolute")
    set_string(j2, "V15DualMotorTopology", "J2A + J2B coaxial synchronized actuators / one revolute DOF")


def markdown_report(data: dict) -> str:
    lines = [
        "# V15 真实关节轴定义",
        "",
        f"生成时间：{data['generated_at_utc']}",
        "",
        "本定义不是按外观推测；每根轴均由 FreeCAD 关节中心、实际输出/法兰圆柱轴和 +1° 从动刚体旋转三重验证。",
        "",
        "| 执行器 | 父链接 | 子链接 | 规范轴 | ROS 2 类型 | 位置范围 | 精确世界轴向量 |",
        "|---|---|---|---:|---|---|---|",
    ]
    for row in data["joints"]:
        axis = ", ".join(f"{value:+.12f}" for value in row["axis_world_at_zero"])
        limit = row["joint_limit_definition"]
        range_text = (
            f"{limit['position_limit_deg']['lower']:.6g}°～{limit['position_limit_deg']['upper']:.6g}°"
            if limit["position_limited"]
            else "未启用位置限位"
        )
        lines.append(
            f"| {row['name']} | {row['parent_link']} | {row['child_link']} | +{row['canonical_axis']} | {row['ros2_urdf']['joint_type']} | {range_text} | `[{axis}]` |"
        )
    lines.extend(
        [
            "",
            "## 关键说明",
            "",
            "- J2A 与 J2B 的两个输出圆中心位于同一轴线上，其中心点中点就是 J2 关节原点；它们是一个 J2 自由度的同轴同步双驱，不得导出成两个独立关节。",
            "- J4、J5、J6 的规范字母轴分别为 `+Y / +X / +Z`；机器文件同时保留精确世界向量，后续不得只根据字母重新猜测方向。",
            "- `base_link/link1～link6` 已按 ROS 2/URDF 与 MJCF 链语义重建：每个 child link 的零位原点就是其进入关节的实际轴心；JSON 同时给出 parent→child 零位位姿、URDF 轴和 MuJoCo body/joint 参数。",
            "- MoveIt 复用同一 URDF 关节树，不再建立另一套轴定义；J2 在 URDF/MJCF 中只保留一个运动学关节，J2A/J2B 仅作为双执行器映射。",
            "- J6 按已确认的达妙电机实际能力定义为有限 `revolute` 关节，位置范围 `-180°～+180°`；不得导出为 `continuous`。",
            "- 位置范围已写入 ROS 2/MoveIt 和 MuJoCo 契约；速度与力矩上限尚无已验证数据，保持空值并阻止猜测。",
            "- `gripper_mount` 与 J6/DM-G6220 输出接口实测原点、轴向完全重合；夹爪连接件安装平面贴合该原点，并作为 `link6 → gripper_mount` 固定关节导出。",
            f"- 连接件六孔与 DM 标称 6×M4/PCD28 的最大轴线误差为 `{data['j6_end_effector_interface']['six_m4_interface']['maximum_connector_to_dm_hole_axis_error_mm']:.9f} mm`，该制造/孔位风险被保留，未宣称完全对孔。",
            f"- DM 固定侧三孔最大轴线残差 `{data['j6_end_effector_interface']['three_m4_stator_mount']['maximum_axis_error_mm']:.9f} mm`；夹爪双耳横轴最大误差 `{data['j6_end_effector_interface']['clevis_tongue_interface']['maximum_cross_axis_error_mm']:.9f} mm`。",
            "- TCP 尚未定义；必须按实际夹持任务测量指尖/工装中心后再增加 `tool0`，不得使用外观中心代替。",
            "- 这里的 0° 是用户指定实际测量并与 V14 保存机械装配零位一致的 CAD 机械零位；编码器电气零位和双电机命令符号仍需实机低转矩标定。",
            "- STEP 不保存关节、轴和零位语义；MuJoCo/URDF/MoveIt 应读取同目录 JSON 或 FCStd 注册表。",
            "",
            "## 验证结果",
            "",
            f"- 源 V14 SHA-256：`{data['source_v14_sha256']}`",
            f"- J4⊥J5：{data['relationship_checks']['j4_j5_angle_deg']:.12f}°",
            f"- J5⊥J6：{data['relationship_checks']['j5_j6_angle_deg']:.12f}°",
            f"- J2A/J2B 输出圆中心距：{data['j2_dual_motor']['common_axis_checks']['flange_center_separation_mm']:.6f} mm",
            f"- J2A/J2B 中点到 J2 原点误差：{data['j2_dual_motor']['common_axis_checks']['flange_midpoint_to_joint_origin_mm']:.12g} mm",
            "- 六个关节的 +1° 有限旋转均与定义轴一致，刚体 Placement 残差通过。",
            "",
        ]
    )
    return "\n".join(lines)


def build(source: Path, output_root: Path, guard_path: Path) -> dict:
    if not source.is_file():
        raise FileNotFoundError(source)
    source_hash_before = sha256(source)
    if source_hash_before != SOURCE_V14_SHA256:
        raise RuntimeError(
            "拒绝从非认证 V14 构建 V15: "
            f"expected={SOURCE_V14_SHA256}, actual={source_hash_before}"
        )

    output_root.mkdir(parents=True, exist_ok=True)
    output_fcstd = output_root / OUTPUT_NAME
    if output_fcstd.exists():
        raise FileExistsError(f"V15 输出已存在，拒绝覆盖: {output_fcstd}")

    doc = App.openDocument(str(source))
    try:
        measurements = {
            key: joint_measurement(doc, key, joint_name, parent_name, child_name)
            for key, joint_name, parent_name, child_name in JOINT_CHAIN
        }
        for key in GEOMETRY_WITNESSES:
            measurements[key]["geometry_witness"] = geometry_witness(doc, key, measurements[key])
        j2_witnesses = j2_dual_motor_witnesses(doc, measurements["J2"])

        guard = load_guard(guard_path)
        motion_checks = finite_motion_checks(doc, guard, measurements)
        frames = build_engineering_frames(doc, measurements)
        joint_rows = make_joint_rows(measurements, frames, j2_witnesses)
        kinematic_chain = downstream_kinematic_chain(joint_rows)
        end_effector_interface = measure_j6_end_effector_interface(doc, frames, joint_rows)
        chain_by_name = {row["joint_name"]: row for row in kinematic_chain}
        expected_types = {"J1": "continuous", "J2": "revolute", "J3": "revolute", "J4": "revolute", "J5": "revolute", "J6": "revolute"}
        expected_ranges_deg = {
            "J1": None,
            "J2": [-170.0, 170.0],
            "J3": [-170.0, 170.0],
            "J4": [-116.0, 159.0],
            "J5": [-70.6, 151.2],
            "J6": [-180.0, 180.0],
        }
        for joint_name, expected_type in expected_types.items():
            row = chain_by_name[joint_name]
            if row["ros2_urdf"]["joint_type"] != expected_type:
                raise RuntimeError(f"{joint_name} ROS 2 关节类型错误")
            actual_range = row["joint_limit_definition"]["position_limit_deg"]
            expected_range = expected_ranges_deg[joint_name]
            if expected_range is None:
                if actual_range is not None:
                    raise RuntimeError(f"{joint_name} 不应导出位置范围")
            elif actual_range is None or any(
                abs(actual_range[key] - expected) > ZERO_TOLERANCE_DEG
                for key, expected in zip(("lower", "upper"), expected_range)
            ):
                raise RuntimeError(f"{joint_name} 导出位置范围错误: {actual_range}")

        axes = {key: App.Vector(*row["axis_world_at_zero"]) for key, row in measurements.items()}
        relationship_checks = {
            "j2_j3_angle_deg": angle_between_deg(axes["J2"], axes["J3"]),
            "j3_j4_angle_deg": angle_between_deg(axes["J3"], axes["J4"]),
            "j4_j5_angle_deg": angle_between_deg(axes["J4"], axes["J5"]),
            "j5_j6_angle_deg": angle_between_deg(axes["J5"], axes["J6"]),
        }
        if max(relationship_checks["j2_j3_angle_deg"], relationship_checks["j3_j4_angle_deg"]) > AXIS_TOLERANCE_DEG:
            raise RuntimeError("J2/J3/J4 实测轴不平行")
        if abs(relationship_checks["j4_j5_angle_deg"] - 90.0) > AXIS_TOLERANCE_DEG:
            raise RuntimeError("J4/J5 实测轴不垂直")
        if abs(relationship_checks["j5_j6_angle_deg"] - 90.0) > AXIS_TOLERANCE_DEG:
            raise RuntimeError("J5/J6 实测轴不垂直")

        add_reference_objects(doc, joint_rows, frames, source_hash_before, end_effector_interface)
        doc.Label = "机械臂完整装配 V15 / 六轴真实关节轴与机械零位"
        doc.recompute()
        doc.saveAs(str(output_fcstd))
    finally:
        if doc is not None:
            App.closeDocument(doc.Name)

    source_hash_after = sha256(source)
    if source_hash_after != source_hash_before:
        raise RuntimeError("V14 源文件在 V15 构建期间发生变化")

    data = {
        "schema": "go-m8010-arm-v15-measured-joint-axes/1.3",
        "revision": REVISION,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "source_v14": str(source),
        "source_v14_sha256": source_hash_before,
        "output_v15_fcstd": str(output_fcstd),
        "output_v15_fcstd_sha256": sha256(output_fcstd),
        "measurement_policy": "NO_VISUAL_GUESSING: coincident joint centers + coaxial BRep cylinders + finite driven-rigid rotation",
        "frames_at_mechanical_zero": frames,
        "joints": joint_rows,
        "downstream_contract": {
            "frame_semantics": "each child-link frame origin is its incoming physical joint center at CAD mechanical zero",
            "length_units": {"cad": "mm", "ros2_urdf": "m", "mujoco_mjcf": "m"},
            "angle_units": {"cad_report": "deg", "ros2_urdf": "rad", "mujoco_mjcf": "rad"},
            "moveit_axis_source": "ROS2 URDF generated from downstream_kinematic_chain; do not duplicate axes in SRDF",
            "j2_topology": "one kinematic revolute joint J2 with physical actuators J2A and J2B",
            "joint_type_and_position_limit_status": "DEFINED_AND_QA_CHECKED_FROM_FREECAD; J6 is revolute -180..180 deg",
            "velocity_and_effort_limit_status": "PENDING_VERIFIED_MOTOR_AND_REDUCER_DATA; URDF emission must not guess",
            "mujoco_collision_exclusion_status": "PENDING_TRANSFER_FROM_V14_DESIGNED_CONTACT_AUDIT",
        },
        "downstream_kinematic_chain": kinematic_chain,
        "j6_end_effector_interface": end_effector_interface,
        "source_joint_measurements": measurements,
        "j2_dual_motor": j2_witnesses,
        "finite_motion_checks": motion_checks,
        "relationship_checks": relationship_checks,
        "limitations": [
            "CAD mechanical zero is defined; physical encoder electrical zero still requires low-torque commissioning.",
            "J2A/J2B motor command signs are not inferred from shaft appearance and must be commissioned on hardware.",
            "The source gripper connector has a measured nonzero six-hole axis mismatch to nominal DM PCD28; do not replace it with an exact pattern in simulation metadata.",
            "The J6 stator three-hole residual, gripper clevis cross-axis error and zero nominal slot/tongue clearance are preserved as interface risks.",
            "ROS 2 position limits and MuJoCo joint ranges are defined; verified velocity/effort limits and MuJoCo designed-contact exclusions are still pending.",
            "gripper_mount is defined, but tool0/TCP is intentionally not defined until a physical task-frame measurement exists.",
            "STEP does not preserve joint-axis or zero-frame semantics.",
        ],
    }

    axis_json = output_root / AXIS_JSON_NAME
    axis_md = output_root / AXIS_MD_NAME
    qa_json = output_root / QA_JSON_NAME
    axis_json.write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")
    axis_md.write_text(markdown_report(data), encoding="utf-8")

    qa = {
        "schema": "go-m8010-arm-v15-joint-axis-qa/1.0",
        "revision": REVISION,
        "status": "PASS",
        "source_v14_hash_unchanged": source_hash_before == source_hash_after,
        "source_v14_sha256": source_hash_before,
        "output_v15_fcstd_sha256": data["output_v15_fcstd_sha256"],
        "joint_count": len(joint_rows),
        "kinematic_dof_count": 6,
        "j2a_j2b_one_dof": True,
        "canonical_axes": {row["name"]: row["canonical_axis"] for row in joint_rows},
        "ros2_joint_types": {name: chain_by_name[name]["ros2_urdf"]["joint_type"] for name in expected_types},
        "all_ros2_joint_types_match_freecad_limits": all(
            chain_by_name[name]["ros2_urdf"]["joint_type"] == expected
            for name, expected in expected_types.items()
        ),
        "all_position_limits_match_freecad": all(
            (
                chain_by_name[name]["joint_limit_definition"]["position_limit_deg"] is None
                if expected_ranges_deg[name] is None
                else all(
                    abs(chain_by_name[name]["joint_limit_definition"]["position_limit_deg"][key] - value) <= ZERO_TOLERANCE_DEG
                    for key, value in zip(("lower", "upper"), expected_ranges_deg[name])
                )
            )
            for name in expected_ranges_deg
        ),
        "all_mujoco_limited_flags_and_ranges_match": all(
            chain_by_name[name]["mujoco_mjcf"]["joint_limited"] == (expected_ranges_deg[name] is not None)
            and (
                chain_by_name[name]["mujoco_mjcf"]["joint_range_rad"] is None
                if expected_ranges_deg[name] is None
                else all(
                    abs(actual - math.radians(expected)) <= 1.0e-12
                    for actual, expected in zip(
                        chain_by_name[name]["mujoco_mjcf"]["joint_range_rad"],
                        expected_ranges_deg[name],
                    )
                )
            )
            for name in expected_ranges_deg
        ),
        "j6_is_revolute_with_pm180_position_limit": (
            chain_by_name["J6"]["ros2_urdf"]["joint_type"] == "revolute"
            and chain_by_name["J6"]["joint_limit_definition"]["position_limit_deg"] == {"lower": -180.0, "upper": 180.0}
        ),
        "velocity_and_effort_limits_not_guessed": all(
            row["joint_limit_definition"]["velocity_limit_rad_s"] is None
            and row["joint_limit_definition"]["effort_limit_nm"] is None
            for row in kinematic_chain
        ),
        "all_source_angles_zero": all(abs(row["source_angle_deg"]) <= ZERO_TOLERANCE_DEG for row in measurements.values()),
        "all_joint_centers_coincident": all(row["placement1_placement2_center_error_mm"] <= LINE_TOLERANCE_MM for row in measurements.values()),
        "all_joint_axes_coincident": all(row["placement1_placement2_axis_error_deg"] <= AXIS_TOLERANCE_DEG for row in measurements.values()),
        "all_child_link_origins_match_joint_centers": all(
            row["child_link_frame_origin_matches_joint_center_mm"] <= LINE_TOLERANCE_MM
            for row in joint_rows
        ),
        "all_parent_and_joint_frame_axes_are_canonical": all(
            max(
                abs(value - expected)
                for value, expected in zip(
                    row["axis_vector_in_parent_link_frame"],
                    {"X": [1.0, 0.0, 0.0], "Y": [0.0, 1.0, 0.0], "Z": [0.0, 0.0, 1.0]}[row["canonical_axis"]],
                )
            ) <= 1.0e-9
            and max(
                abs(value - expected)
                for value, expected in zip(
                    row["axis_vector_in_joint_child_frame"],
                    {"X": [1.0, 0.0, 0.0], "Y": [0.0, 1.0, 0.0], "Z": [0.0, 0.0, 1.0]}[row["canonical_axis"]],
                )
            ) <= 1.0e-9
            for row in joint_rows
        ),
        "all_geometry_witnesses_coaxial": all(
            row["geometry_witness"]["axis_line_residual_mm"] <= LINE_TOLERANCE_MM
            for key, row in measurements.items()
            if key != "J2"
        ),
        "all_finite_motion_checks_pass": all(
            row["axis_error_deg"] <= AXIS_TOLERANCE_DEG
            and row["expected_placement_translation_error_mm"] <= MOTION_TRANSLATION_TOLERANCE_MM
            and row["expected_placement_rotation_error_deg"] <= MOTION_ROTATION_TOLERANCE_DEG
            for row in motion_checks.values()
        ),
        "relationship_checks": relationship_checks,
        "j2_dual_motor_common_axis_checks": j2_witnesses["common_axis_checks"],
        "j6_output_mount_origin_matches_joint_center": end_effector_interface["j6_output_mount_frame"]["origin_to_j6_joint_center_error_mm"] <= LINE_TOLERANCE_MM,
        "j6_output_mount_axis_matches_joint_axis": end_effector_interface["j6_output_mount_frame"]["axis_to_j6_error_deg"] <= AXIS_TOLERANCE_DEG,
        "j6_connector_contact_plane_seated": abs(end_effector_interface["connector_seat"]["contact_plane_brep_witness"]["axial_station_mm"]) <= LINE_TOLERANCE_MM,
        "j6_connector_six_hole_mismatch_preserved_mm": end_effector_interface["six_m4_interface"]["maximum_connector_to_dm_hole_axis_error_mm"],
        "j6_stator_three_hole_error_preserved_mm": end_effector_interface["three_m4_stator_mount"]["maximum_axis_error_mm"],
        "gripper_clevis_cross_axis_error_preserved_mm": end_effector_interface["clevis_tongue_interface"]["maximum_cross_axis_error_mm"],
        "gripper_clevis_radial_clearance_preserved_mm": end_effector_interface["clevis_tongue_interface"]["radial_clearance_remaining_mm"],
        "mujoco_collision_contract_pending_and_not_silently_accepted": end_effector_interface["mujoco_collision_contract_status"].startswith("PENDING"),
        "tcp_intentionally_undefined": end_effector_interface["tcp_status"].startswith("NOT_DEFINED"),
        "axis_json_sha256": sha256(axis_json),
        "axis_md_sha256": sha256(axis_md),
    }
    qa_json.write_text(json.dumps(qa, ensure_ascii=False, indent=2), encoding="utf-8")
    shutil.copy2(Path(__file__).resolve(), output_root / Path(__file__).name)
    return {
        "fcstd": str(output_fcstd),
        "axis_json": str(axis_json),
        "axis_md": str(axis_md),
        "qa_json": str(qa_json),
        "status": "PASS",
        "canonical_axes": qa["canonical_axes"],
        "fcstd_sha256": data["output_v15_fcstd_sha256"],
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="构建 V15 真实关节轴定义副本")
    parser.add_argument("--source", type=Path, default=DEFAULT_SOURCE)
    parser.add_argument("--output-root", type=Path, default=DEFAULT_OUTPUT_ROOT)
    parser.add_argument("--guard", type=Path, default=DEFAULT_GUARD)
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_args()
    print(json.dumps(build(args.source, args.output_root, args.guard), ensure_ascii=False, indent=2))

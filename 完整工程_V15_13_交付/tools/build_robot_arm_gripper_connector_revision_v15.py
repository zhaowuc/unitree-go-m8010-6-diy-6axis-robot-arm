from __future__ import annotations

"""Replace the V15 gripper connector with the measured ``夹爪改版.stl``.

The source V15.3 FCStd is never modified.  The new connector is registered by
its fitted 6-hole PCD28 pattern, and the existing articulated gripper is moved
by its two source tongue-hole axes.  No placement is based on visual matching.
"""

import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import FreeCAD as App
import Mesh
import MeshPart
import Part


WORKSPACE = Path(r"D:/AI_JIXIEBI/go_m8010_arm_gui")
SOURCE_ROOT = Path(r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15")
SOURCE_FCSTD = SOURCE_ROOT / "机械臂完整装配_六轴_真实关节轴_v15.FCStd"
SOURCE_AXES_JSON = SOURCE_ROOT / "V15_真实关节轴定义.json"
CONNECTOR_STL = Path(r"D:/AI_JIXIEBI/模型/夹爪改版.stl")

OUTPUT_ROOT = SOURCE_ROOT / "夹爪改版连接件_v15"
OUTPUT_FCSTD = OUTPUT_ROOT / "机械臂完整装配_六轴_夹爪改版连接件_v15.FCStd"
OUTPUT_TOOL_STEP = OUTPUT_ROOT / "J6末端_夹爪改版连接件与舵机夹爪_零位.step"
OUTPUT_TOOL_STL = OUTPUT_ROOT / "J6末端_夹爪改版连接件与舵机夹爪_零位.stl"
OUTPUT_CONNECTOR_STEP = OUTPUT_ROOT / "夹爪改版_修复闭合实体.step"
OUTPUT_CONNECTOR_STL = OUTPUT_ROOT / "夹爪改版_修复闭合实体.stl"
OUTPUT_QA = OUTPUT_ROOT / "QA_V15_夹爪改版连接件装配.json"
OUTPUT_INTERFACE = OUTPUT_ROOT / "V15_夹爪改版接口定义.json"
OUTPUT_README = OUTPUT_ROOT / "README_夹爪改版连接件.md"

EXPECTED_CONNECTOR_SHA256 = "6D69A7FBF6E6E5375AEE4B949861F5E9DEB2DE1968EACDD3442E6FCD8552E875"
REVISION = "V15.5-measured-revised-gripper-connector"

# Fitted directly from sharp circular edges in the supplied STL.  Raw axes:
# six mounting holes are parallel to +Z; contact face is raw Z=10 mm.
CONNECTOR_PATTERN_CENTER = App.Vector(0.1122736000, 0.0610069000, 10.0)
CONNECTOR_HOLE_RADIUS_MM = 2.0
CONNECTOR_PCD_RADIUS_MM = 14.0
CONNECTOR_HOLE_CENTERS_XY = (
    (14.112273505049947, 0.061006960436770896),
    (7.112273573875480, 12.185362588085663),
    (-6.887726287047093, 12.185362588085685),
    (-13.887726426124582, 0.061006960436770896),
    (-6.887726287047087, -12.063348770141605),
    (7.112273573875470, -12.063348770141591),
)

# Fitted from the revised connector STL central clevis.  Axes are raw +X.
CONNECTOR_SLOT_X = (-1.5, 1.5)
CONNECTOR_CLEVIS_OUTER_X = (-11.0, 11.0)
CONNECTOR_SLOT_Z = (-8.0, 0.0)
CONNECTOR_CROSS_AXES_YZ = ((5.0, -4.0), (-5.0, -4.0))
CONNECTOR_CROSS_HOLE_DIAMETER_MM = 2.0

# Existing gripper STEP measurements retained from the accepted V14 build.
GRIPPER_TONGUE_Y = (-25.664357793295, -22.664357793295)
GRIPPER_TONGUE_Z = (-14.798383459992, -6.798383459992)
GRIPPER_TONGUE_HOLES_XZ = (
    (114.221654026735, -10.298383459992),
    (124.221654026735, -10.298383459992),
)
GRIPPER_TONGUE_HOLE_DIAMETER_MM = 3.0

# Unique source-gripper -> revised-connector registration:
# source X -> connector -Y, source Y -> connector +X, source Z -> connector +Z.
GRIPPER_TO_CONNECTOR_TRANSLATION = App.Vector(
    24.164357793295,
    119.221654026735,
    6.298383459992,
)

M4_SCREW_LENGTH_MM = 14.0
PRINTED_M3_SCREW_LENGTH_MM = 25.0
PENETRATION_TOLERANCE_MM3 = 1.0e-3


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def ensure_property(obj, kind: str, name: str, group: str):
    if name not in obj.PropertiesList:
        obj.addProperty(kind, name, group)


def add_text(obj, name: str, group: str, value) -> None:
    ensure_property(obj, "App::PropertyString", name, group)
    setattr(obj, name, str(value))


def add_bool(obj, name: str, group: str, value) -> None:
    ensure_property(obj, "App::PropertyBool", name, group)
    setattr(obj, name, bool(value))


def add_length(obj, name: str, group: str, value) -> None:
    ensure_property(obj, "App::PropertyLength", name, group)
    setattr(obj, name, float(value))


def add_placement(obj, name: str, group: str, value: App.Placement) -> None:
    ensure_property(obj, "App::PropertyPlacement", name, group)
    setattr(obj, name, App.Placement(value))


def add_vector(obj, name: str, group: str, value: App.Vector) -> None:
    ensure_property(obj, "App::PropertyVector", name, group)
    setattr(obj, name, App.Vector(value))


def add_float_list(obj, name: str, group: str, values) -> None:
    ensure_property(obj, "App::PropertyFloatList", name, group)
    setattr(obj, name, [float(value) for value in values])


def rotation_from_columns(x_axis: App.Vector, y_axis: App.Vector, z_axis: App.Vector) -> App.Rotation:
    matrix = App.Matrix()
    matrix.A11, matrix.A12, matrix.A13 = x_axis.x, y_axis.x, z_axis.x
    matrix.A21, matrix.A22, matrix.A23 = x_axis.y, y_axis.y, z_axis.y
    matrix.A31, matrix.A32, matrix.A33 = x_axis.z, y_axis.z, z_axis.z
    matrix.A44 = 1.0
    return App.Rotation(matrix)


def placement_from_frame(frame: dict) -> App.Placement:
    return App.Placement(
        App.Vector(*frame["origin_world_mm"]),
        rotation_from_columns(
            App.Vector(*frame["x_axis_world_at_zero"]),
            App.Vector(*frame["y_axis_world_at_zero"]),
            App.Vector(*frame["z_axis_world_at_zero"]),
        ),
    )


def placement_dict(value: App.Placement) -> dict:
    q = value.Rotation.Q
    return {
        "base_mm": [value.Base.x, value.Base.y, value.Base.z],
        "quaternion_xyzw": [q[0], q[1], q[2], q[3]],
    }


def local_placement(parent, desired_world: App.Placement) -> App.Placement:
    return parent.getGlobalPlacement().inverse().multiply(desired_world)


def configure_view(obj, color, transparency=0, visible=True) -> None:
    view = getattr(obj, "ViewObject", None)
    if view is None:
        return
    view.ShapeColor = tuple(float(item) for item in color)
    view.LineColor = tuple(max(0.0, float(item) * 0.35) for item in color)
    view.Transparency = int(transparency)
    view.Visibility = bool(visible)
    if "DisplayMode" in view.PropertiesList:
        view.DisplayMode = "Flat Lines"


def repair_connector_shape() -> tuple[Part.Shape, dict]:
    mesh = Mesh.Mesh(str(CONNECTOR_STL))
    before = {
        "point_count": int(mesh.CountPoints),
        "facet_count": int(mesh.CountFacets),
        "mesh_is_solid": bool(mesh.isSolid()),
        "mesh_volume_mm3": float(mesh.Volume),
    }
    operations = []
    for name in (
        "removeDuplicatedPoints",
        "removeDuplicatedFacets",
        "fixIndices",
        "fixDegenerations",
        "fixDeformations",
        "fixSelfIntersections",
        "harmonizeNormals",
        "fillupHoles",
        "harmonizeNormals",
    ):
        try:
            if name == "fillupHoles":
                getattr(mesh, name)(100)
            else:
                getattr(mesh, name)()
            operations.append({"operation": name, "status": "OK"})
        except Exception as exc:
            operations.append({"operation": name, "status": "SKIPPED", "message": str(exc)})

    shell_shape = Part.Shape()
    shell_shape.makeShapeFromMesh(mesh.Topology, 0.02)
    shell_rows = []
    candidates = []
    for index, shell in enumerate(shell_shape.Shells, 1):
        row = {
            "shell_index": index,
            "face_count": len(shell.Faces),
            "closed": bool(shell.isClosed()),
            "bbox_mm": [
                shell.BoundBox.XMin,
                shell.BoundBox.YMin,
                shell.BoundBox.ZMin,
                shell.BoundBox.XMax,
                shell.BoundBox.YMax,
                shell.BoundBox.ZMax,
            ],
        }
        try:
            solid = Part.makeSolid(shell)
            row["solid_valid"] = bool(solid.isValid())
            row["solid_closed"] = bool(solid.isClosed())
            row["volume_mm3"] = float(solid.Volume)
            if solid.isValid() and solid.isClosed() and len(solid.Solids) == 1 and solid.Volume > 1.0:
                candidates.append((index, solid))
        except Exception as exc:
            row["solid_error"] = str(exc)
        shell_rows.append(row)
    if not candidates:
        raise RuntimeError("夹爪改版 STL 修复后没有有效闭合主实体")
    selected_index, solid = max(candidates, key=lambda item: float(item[1].Volume))
    if not solid.isValid() or not solid.isClosed():
        raise RuntimeError("夹爪改版主实体 BRep 无效或未闭合")
    return solid, {
        "before": before,
        "operations": operations,
        "shells": shell_rows,
        "selected_shell_index": selected_index,
        "dropped_shell_reason": "4-face open zero-thickness fragment; not part of the closed manufactured solid",
        "after": {
            "brep_valid": bool(solid.isValid()),
            "brep_closed": bool(solid.isClosed()),
            "solid_count": len(solid.Solids),
            "face_count": len(solid.Faces),
            "volume_mm3": float(solid.Volume),
            "center_of_mass_raw_mm": [solid.CenterOfMass.x, solid.CenterOfMass.y, solid.CenterOfMass.z],
            "bbox_min_raw_mm": [solid.BoundBox.XMin, solid.BoundBox.YMin, solid.BoundBox.ZMin],
            "bbox_max_raw_mm": [solid.BoundBox.XMax, solid.BoundBox.YMax, solid.BoundBox.ZMax],
        },
    }


def world_shape(obj) -> Part.Shape:
    shape = obj.Shape.copy()
    shape.Placement = obj.getGlobalPlacement()
    return shape


def aabb_separated(first: Part.Shape, second: Part.Shape) -> bool:
    a, b = first.BoundBox, second.BoundBox
    tol = 1.0e-7
    return (
        a.XMax <= b.XMin + tol
        or b.XMax <= a.XMin + tol
        or a.YMax <= b.YMin + tol
        or b.YMax <= a.YMin + tol
        or a.ZMax <= b.ZMin + tol
        or b.ZMax <= a.ZMin + tol
    )


def exact_overlap(first, second) -> dict:
    first_shape = world_shape(first)
    second_shape = world_shape(second)
    if aabb_separated(first_shape, second_shape):
        return {"overlap_mm3": 0.0, "collision": False, "method": "AABB_SEPARATED"}
    common = first_shape.common(second_shape)
    volume = 0.0 if common.isNull() else float(common.Volume)
    return {
        "overlap_mm3": volume,
        "collision": volume > PENETRATION_TOLERANCE_MM3,
        "method": "EXACT_BREP_COMMON",
    }


def make_m4_screw_raw(x: float, y: float) -> Part.Shape:
    direction = App.Vector(0, 0, 1)
    shaft = Part.makeCylinder(2.0, M4_SCREW_LENGTH_MM, App.Vector(x, y, 0.0), direction)
    head = Part.makeCylinder(3.5, 3.0, App.Vector(x, y, -3.0), direction)
    washer = Part.makeCylinder(4.0, 0.8, App.Vector(x, y, -0.8), direction).cut(
        Part.makeCylinder(2.15, 0.8, App.Vector(x, y, -0.8), direction)
    )
    return Part.makeCompound([shaft, head, washer])


def make_printed_cross_screw_raw(y: float, z: float) -> Part.Shape:
    direction = App.Vector(1, 0, 0)
    start_x = -PRINTED_M3_SCREW_LENGTH_MM / 2.0
    shaft = Part.makeCylinder(1.5, PRINTED_M3_SCREW_LENGTH_MM, App.Vector(start_x, y, z), direction)
    head = Part.makeCylinder(3.0, 2.2, App.Vector(start_x - 2.2, y, z), direction)
    return Part.makeCompound([shaft, head])


def add_part(tool, name: str, label: str, shape: Part.Shape, source_world: App.Placement, color) -> object:
    obj = tool.newObject("Part::Feature", name)
    obj.Label = label
    obj.Shape = shape.copy()
    obj.Placement = local_placement(tool, source_world)
    configure_view(obj, color, 0, True)
    add_bool(obj, "CollisionEnabled", "Collision", False)
    add_bool(obj, "MassEnabled", "Physics", False)
    add_text(obj, "MotionRole", "Kinematics", "static")
    add_placement(obj, "ReferencePlacement", "Kinematics", obj.Placement)
    return obj


def remove_old_fasteners(doc) -> list[str]:
    prefixes = ("DM_G6220_Connector_M4x8_Screw_", "Gripper_Clevis_M3x16_Screw_")
    removed = []
    for obj in tuple(doc.Objects):
        if any(obj.Name.startswith(prefix) for prefix in prefixes):
            removed.append(obj.Name)
            doc.removeObject(obj.Name)
    return removed


def tool_export_objects(tool) -> list:
    result = []
    for obj in tool.Group:
        name = obj.Name
        if not hasattr(obj, "Shape") or obj.Shape.isNull():
            continue
        if "Collision_Proxy" in name or name.endswith("_Collision"):
            continue
        if name == "J6_Gripper_Connector_STEP_Display" or name.startswith("Gripper_") or name.startswith("DM_G6220_Connector_"):
            result.append(obj)
    return result


def build() -> None:
    for path in (SOURCE_FCSTD, SOURCE_AXES_JSON, CONNECTOR_STL):
        if not path.is_file():
            raise FileNotFoundError(path)
    connector_hash = sha256(CONNECTOR_STL)
    if connector_hash != EXPECTED_CONNECTOR_SHA256:
        raise RuntimeError(
            "夹爪改版.stl 已变化；拒绝套用旧测量数据。"
            f" expected={EXPECTED_CONNECTOR_SHA256}, actual={connector_hash}"
        )
    source_hash_before = sha256(SOURCE_FCSTD)
    axes = json.loads(SOURCE_AXES_JSON.read_text(encoding="utf-8"))
    if axes.get("revision") != "V15.3-measured-axis-joint-limits-and-j6-tool-interface":
        raise RuntimeError(f"unexpected axis definition revision: {axes.get('revision')}")

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    connector_shape, repair_info = repair_connector_shape()

    link6_frame = axes["frames_at_mechanical_zero"]["link6"]
    link6_world = placement_from_frame(link6_frame)
    lx = App.Vector(*link6_frame["x_axis_world_at_zero"])
    ly = App.Vector(*link6_frame["y_axis_world_at_zero"])
    lz = App.Vector(*link6_frame["z_axis_world_at_zero"])
    for axis in (lx, ly, lz):
        axis.normalize()

    # Revised connector raw X->link6 +X, raw Y->link6 -Y, raw Z->link6 -Z.
    connector_rotation = rotation_from_columns(lx, -ly, -lz)
    connector_translation = link6_world.Base - connector_rotation.multVec(CONNECTOR_PATTERN_CENTER)
    connector_world = App.Placement(connector_translation, connector_rotation)

    source_to_connector_rotation = rotation_from_columns(
        App.Vector(0, -1, 0), App.Vector(1, 0, 0), App.Vector(0, 0, 1)
    )
    gripper_to_connector = App.Placement(
        GRIPPER_TO_CONNECTOR_TRANSLATION, source_to_connector_rotation
    )
    gripper_world = connector_world.multiply(gripper_to_connector)

    doc = App.openDocument(str(SOURCE_FCSTD))
    if doc is None:
        raise RuntimeError("cannot open V15 source FCStd")
    doc.Label = "机械臂 V15.5 / 真实六轴 + 夹爪改版连接件"
    r6 = doc.getObject("J6_Driven_Rigid")
    tool = doc.getObject("J6_Gripper_Assembly")
    joint6 = doc.getObject("J6_Revolute")
    connector_display = doc.getObject("J6_Gripper_Connector_STEP_Display")
    connector_proxy = doc.getObject("J6_Gripper_Connector_Collision_Proxy")
    if None in (r6, tool, joint6, connector_display, connector_proxy):
        raise RuntimeError("V15 source is missing J6/tool/connector objects")

    # Record the old transform before replacement so orientation preservation is auditable.
    old_gripper_world = App.Placement(tool.GripperSourceToWorldAtJ6Zero)

    tool.Label = "J6末端执行器：夹爪改版连接件 + 舵机柔性二指夹爪"
    add_text(tool, "RigidOwner", "Assembly", "J6_Driven_Rigid")
    add_text(tool, "Topology", "Assembly", "revised bolted connector + fixed gripper frame; internal symmetric parallelogram")
    add_placement(tool, "ConnectorSourceToWorldAtJ6Zero", "Reference", connector_world)
    add_placement(tool, "GripperSourceToWorldAtJ6Zero", "Reference", gripper_world)
    add_placement(tool, "GripperSourceToR6AtZero", "Reference", local_placement(r6, gripper_world))

    for obj, label, collision, visible, color in (
        (
            connector_display,
            "夹爪改版 / 修复闭合实体 / 6×Ø4 PCD28 面贴 DM-G6220",
            False,
            True,
            (0.78, 0.16, 0.20),
        ),
        (
            connector_proxy,
            "夹爪改版精确碰撞体 / 修复后闭合主 Solid",
            True,
            False,
            (1.00, 0.42, 0.10),
        ),
    ):
        obj.Label = label
        obj.Shape = connector_shape.copy()
        obj.Placement = local_placement(tool, connector_world)
        add_placement(obj, "ReferencePlacement", "Kinematics", obj.Placement)
        add_bool(obj, "CollisionEnabled", "Collision", collision)
        add_bool(obj, "MassEnabled", "Physics", collision)
        add_text(obj, "SourceFile", "Source", str(CONNECTOR_STL))
        add_text(obj, "SourceSHA256", "Source", connector_hash)
        add_text(obj, "RepairStatus", "QA", "REPAIRED COPY / largest valid closed BRep shell / source STL unchanged")
        configure_view(obj, color, 0 if visible else 70, visible)

    gripper_objects = []
    for obj in doc.Objects:
        if not obj.Name.startswith("Gripper_"):
            continue
        if "SourcePlacement" not in obj.PropertiesList:
            continue
        source_placement = App.Placement(obj.SourcePlacement)
        reference = local_placement(tool, gripper_world.multiply(source_placement))
        obj.Placement = reference
        add_placement(obj, "ReferencePlacement", "Kinematics", reference)
        gripper_objects.append(obj.Name)

    controller = doc.getObject("Gripper_Servo_Controller")
    if controller is None:
        raise RuntimeError("missing Gripper_Servo_Controller")
    controller.ClosureAngle = 0.0
    add_placement(controller, "GripperSourceToR6AtZero", "Reference", local_placement(r6, gripper_world))
    add_text(controller, "ConnectorRevision", "Reference", REVISION)

    removed_fasteners = remove_old_fasteners(doc)
    m4_screws = []
    for index, (x, y) in enumerate(CONNECTOR_HOLE_CENTERS_XY, 1):
        screw = add_part(
            tool,
            f"DM_G6220_Connector_M4x14_Screw_{index:02d}",
            f"DM-G6220 输出盘→夹爪改版连接件 M4×14 暂定 {index}/6",
            make_m4_screw_raw(x, y),
            connector_world,
            (0.90, 0.92, 0.95),
        )
        add_text(screw, "Thread", "Fastener", "M4")
        add_length(screw, "Length", "Fastener", M4_SCREW_LENGTH_MM)
        add_length(screw, "ConnectorPlateThickness", "Fastener", 10.0)
        add_length(screw, "NominalMotorEngagement", "Fastener", 4.0)
        add_text(screw, "LengthStatus", "Fastener", "PROVISIONAL / verify DM-G6220 tapped depth and washer stack")
        m4_screws.append(screw)

    printed_screws = []
    for index, (y, z) in enumerate(CONNECTOR_CROSS_AXES_YZ, 1):
        screw = add_part(
            tool,
            f"Gripper_Clevis_3DPrinted_M3x25_Screw_{index:02d}",
            f"夹爪改版双耳→夹爪舌片 Ø3打印螺丝×25（压入Ø2孔） {index}/2",
            make_printed_cross_screw_raw(y, z),
            connector_world,
            (0.93, 0.78, 0.18),
        )
        add_text(screw, "Thread", "Fastener", "M3 / 3D PRINTED")
        add_length(screw, "NominalShaftDiameter", "Fastener", 3.0)
        add_length(screw, "ConnectorPilotHoleDiameter", "Fastener", CONNECTOR_CROSS_HOLE_DIAMETER_MM)
        add_length(screw, "RadialInterference", "Fastener", 0.5)
        add_length(screw, "Length", "Fastener", PRINTED_M3_SCREW_LENGTH_MM)
        add_text(screw, "FitStatus", "Fastener", "USER-CONFIRMED FORCE-FIT / screw is intentionally pressed into original Ø2 printed hole")
        add_text(screw, "LengthStatus", "Fastener", "PROVISIONAL / 22 mm clevis span + end protrusion; verify printed screw head and nut geometry")
        printed_screws.append(screw)

    old_status = doc.getObject("V14_Gripper_Physical_Test_Status")
    if old_status is not None:
        old_status.Label = "旧V14连接件验收记录（已由V15.5改版连接件取代）"
        add_text(old_status, "Status", "QA", "SUPERSEDED BY V15.5 REVISED CONNECTOR QA")

    record = doc.addObject("App::FeaturePython", "V15_Revised_Gripper_Connector_Record")
    record.Label = "V15.5 夹爪改版连接件实测接口与装配记录"
    add_text(record, "Revision", "Revision", REVISION)
    add_text(record, "MeasurementPolicy", "QA", "NO VISUAL GUESSING / fitted motor holes + clevis slot + cross-hole axes")
    add_text(record, "SourceSTL", "Source", str(CONNECTOR_STL))
    add_text(record, "SourceSHA256", "Source", connector_hash)
    add_float_list(record, "MotorHolePatternCenterRawMM", "MotorInterface", [0.1122736, 0.0610069, 10.0])
    add_length(record, "MotorHolePCD", "MotorInterface", 28.0)
    add_length(record, "MotorHoleDiameter", "MotorInterface", 4.0)
    add_vector(record, "ClevisAxis1Raw", "GripperInterface", App.Vector(0, 5, -4))
    add_vector(record, "ClevisAxis2Raw", "GripperInterface", App.Vector(0, -5, -4))
    add_length(record, "ClevisPilotHoleDiameter", "GripperInterface", 2.0)
    add_text(record, "PrintedScrewFit", "GripperInterface", "USER CONFIRMED: retain Ø2 holes; force Ø3 3D-printed screws into them")
    add_text(record, "PrintedScrewLength", "GripperInterface", "M3×25 representation is provisional; physical printed length must be verified")
    add_text(record, "ROS2MuJoCoRole", "Downstream", "connector + fixed gripper frame are fixed child gripper rigid body of link6")

    add_bool(r6, "GripperInstalled", "V15", True)
    add_text(r6, "V15DownstreamRole", "V15", "DM-G6220 output rotor + revised connector + articulated servo gripper")

    doc.recompute()

    # Independent numerical interface checks.
    seat_center = connector_world.multVec(CONNECTOR_PATTERN_CENTER)
    seat_center_error = (seat_center - link6_world.Base).Length
    connector_outward = connector_rotation.multVec(App.Vector(0, 0, -1))
    connector_outward.normalize()
    outward_axis_dot = connector_outward.dot(lz)

    six_hole_errors = []
    for index, (x, y) in enumerate(CONNECTOR_HOLE_CENTERS_XY):
        angle = math.radians(60.0 * index)
        ideal_raw = App.Vector(
            CONNECTOR_PATTERN_CENTER.x + CONNECTOR_PCD_RADIUS_MM * math.cos(angle),
            CONNECTOR_PATTERN_CENTER.y + CONNECTOR_PCD_RADIUS_MM * math.sin(angle),
            CONNECTOR_PATTERN_CENTER.z,
        )
        actual_world = connector_world.multVec(App.Vector(x, y, 10.0))
        ideal_world = connector_world.multVec(ideal_raw)
        six_hole_errors.append((actual_world - ideal_world).Length)

    cross_axis_errors = []
    transformed_cross_axes = []
    for (source_x, source_z), (target_y, target_z) in zip(
        GRIPPER_TONGUE_HOLES_XZ, CONNECTOR_CROSS_AXES_YZ
    ):
        transformed = gripper_to_connector.multVec(App.Vector(source_x, 0.0, source_z))
        transformed_cross_axes.append([transformed.x, transformed.y, transformed.z])
        cross_axis_errors.append(math.hypot(transformed.y - target_y, transformed.z - target_z))

    transformed_tongue_x = (
        GRIPPER_TONGUE_Y[0] + GRIPPER_TO_CONNECTOR_TRANSLATION.x,
        GRIPPER_TONGUE_Y[1] + GRIPPER_TO_CONNECTOR_TRANSLATION.x,
    )
    transformed_tongue_z = (
        GRIPPER_TONGUE_Z[0] + GRIPPER_TO_CONNECTOR_TRANSLATION.z,
        GRIPPER_TONGUE_Z[1] + GRIPPER_TO_CONNECTOR_TRANSLATION.z,
    )
    slot_width_error = max(
        abs(transformed_tongue_x[0] - CONNECTOR_SLOT_X[0]),
        abs(transformed_tongue_x[1] - CONNECTOR_SLOT_X[1]),
    )
    insertion_end_residual = max(
        abs(transformed_tongue_z[0] - CONNECTOR_SLOT_Z[0]),
        abs(transformed_tongue_z[1] - CONNECTOR_SLOT_Z[1]),
    )

    new_x = gripper_world.Rotation.multVec(App.Vector(1, 0, 0))
    new_y = gripper_world.Rotation.multVec(App.Vector(0, 1, 0))
    new_z = gripper_world.Rotation.multVec(App.Vector(0, 0, 1))
    old_x = old_gripper_world.Rotation.multVec(App.Vector(1, 0, 0))
    old_y = old_gripper_world.Rotation.multVec(App.Vector(0, 1, 0))
    old_z = old_gripper_world.Rotation.multVec(App.Vector(0, 0, 1))
    orientation_axis_errors_deg = [
        math.degrees(math.acos(max(-1.0, min(1.0, a.dot(b)))))
        for a, b in ((new_x, old_x), (new_y, old_y), (new_z, old_z))
    ]

    collision_pairs = {}
    for name in (
        "J6_DM_G6220_Output_Rotor_Collision_Proxy",
        "Gripper_Fixed_Frame_Collision_Proxy",
        "Gripper_Servo_Collision_Proxy",
    ):
        other = doc.getObject(name)
        if other is None:
            raise RuntimeError(f"missing collision QA object: {name}")
        collision_pairs[f"J6_Gripper_Connector_Collision_Proxy vs {name}"] = exact_overlap(
            connector_proxy, other
        )

    if seat_center_error > 1.0e-6:
        raise RuntimeError(f"connector seat center error: {seat_center_error} mm")
    if outward_axis_dot < 0.999999999:
        raise RuntimeError(f"connector outward direction mismatch: dot={outward_axis_dot}")
    if max(six_hole_errors) > 1.0e-4:
        raise RuntimeError(f"six-hole axis mismatch: {max(six_hole_errors)} mm")
    if max(cross_axis_errors) > 1.0e-6:
        raise RuntimeError(f"gripper cross-hole axis mismatch: {max(cross_axis_errors)} mm")
    if slot_width_error > 1.0e-6:
        raise RuntimeError(f"3 mm tongue/slot registration mismatch: {slot_width_error} mm")
    # The accepted old connector carried a measured 0.00724 degree hole-pattern
    # clock residual.  The revised connector has an exact PCD28 pattern.  Keeping
    # the new motor and clevis axes coaxial therefore removes that old residual;
    # forcing the old clock would deliberately de-center the revised holes.
    if max(orientation_axis_errors_deg) > 0.01:
        raise RuntimeError(
            "gripper orientation change exceeds the measured old-pattern clock correction: "
            f"{orientation_axis_errors_deg}"
        )
    unexpected_collisions = {
        name: row for name, row in collision_pairs.items() if row["collision"]
    }
    if unexpected_collisions:
        raise RuntimeError(f"unexpected connector interference: {unexpected_collisions}")

    j6_limit = {
        "type_id": joint6.TypeId,
        "enable_min": bool(joint6.EnableAngleMin),
        "enable_max": bool(joint6.EnableAngleMax),
        "min_deg": float(joint6.AngleMin),
        "max_deg": float(joint6.AngleMax),
    }
    if (
        not j6_limit["enable_min"]
        or not j6_limit["enable_max"]
        or abs(j6_limit["min_deg"] + 180.0) > 1.0e-9
        or abs(j6_limit["max_deg"] - 180.0) > 1.0e-9
    ):
        raise RuntimeError(f"J6 is not revolute ±180 degrees: {j6_limit}")

    qa = {
        "status": "PASS",
        "revision": REVISION,
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "measurement_policy": "NO VISUAL GUESSING",
        "sources": {
            "v15_fcstd": str(SOURCE_FCSTD),
            "v15_fcstd_sha256_before": source_hash_before,
            "axes_json": str(SOURCE_AXES_JSON),
            "axes_json_sha256": sha256(SOURCE_AXES_JSON),
            "connector_stl": str(CONNECTOR_STL),
            "connector_stl_sha256": connector_hash,
        },
        "connector_repair": repair_info,
        "placements": {
            "link6_world_at_mechanical_zero": placement_dict(link6_world),
            "connector_source_to_world_at_j6_zero": placement_dict(connector_world),
            "gripper_source_to_world_at_j6_zero": placement_dict(gripper_world),
            "gripper_source_to_connector": placement_dict(gripper_to_connector),
            "connector_axis_mapping": {
                "raw_x": "link6 +X",
                "raw_y": "link6 -Y",
                "raw_z": "link6 -Z",
            },
            "gripper_orientation_axis_change_from_previous_deg": orientation_axis_errors_deg,
        },
        "dm_g6220_interface": {
            "pattern": "6x Ø4 through / exact fitted PCD28",
            "contact_face_raw_z_mm": 10.0,
            "pattern_center_raw_mm": [0.1122736, 0.0610069, 10.0],
            "hole_centers_raw_xy_mm": [list(row) for row in CONNECTOR_HOLE_CENTERS_XY],
            "seat_center_error_mm": seat_center_error,
            "outward_axis_dot": outward_axis_dot,
            "hole_axis_errors_mm": six_hole_errors,
            "maximum_hole_axis_error_mm": max(six_hole_errors),
            "fasteners": "6x M4x14 representation; length/tapped depth remain provisional",
        },
        "gripper_clevis_interface": {
            "slot_raw_x_mm": list(CONNECTOR_SLOT_X),
            "slot_width_mm": CONNECTOR_SLOT_X[1] - CONNECTOR_SLOT_X[0],
            "transformed_tongue_raw_x_mm": list(transformed_tongue_x),
            "slot_width_registration_error_mm": slot_width_error,
            "connector_cross_axes_raw_yz_mm": [list(row) for row in CONNECTOR_CROSS_AXES_YZ],
            "transformed_source_hole_axes_raw_xyz_mm": transformed_cross_axes,
            "cross_axis_errors_mm": cross_axis_errors,
            "maximum_cross_axis_error_mm": max(cross_axis_errors),
            "connector_pilot_hole_diameter_mm": CONNECTOR_CROSS_HOLE_DIAMETER_MM,
            "printed_screw_nominal_diameter_mm": 3.0,
            "radial_interference_mm": 0.5,
            "fit_status": "USER-CONFIRMED FORCE-FIT; retain original Ø2 holes",
            "clevis_outer_span_mm": CONNECTOR_CLEVIS_OUTER_X[1] - CONNECTOR_CLEVIS_OUTER_X[0],
            "printed_screw_length_mm": PRINTED_M3_SCREW_LENGTH_MM,
            "printed_screw_length_status": "PROVISIONAL",
            "connector_slot_z_mm": list(CONNECTOR_SLOT_Z),
            "transformed_tongue_z_mm": list(transformed_tongue_z),
            "insertion_end_residual_mm": insertion_end_residual,
            "insertion_note": "hole axes are exact; tongue end extends 0.5 mm beyond each nominal slot end",
        },
        "collision_at_zero": {
            "penetration_tolerance_mm3": PENETRATION_TOLERANCE_MM3,
            "pairs": collision_pairs,
            "unexpected_collision": False,
            "intentional_exclusion": "Ø3 printed cross screws intentionally intersect Ø2 connector pilot holes",
        },
        "j6": {
            **j6_limit,
            "range_rad": [-math.pi, math.pi],
            "verified_plus_minus_180": True,
        },
        "gripper": {
            "source_component_count_repositioned": len(gripper_objects),
            "source_components_repositioned": gripper_objects,
            "closure_angle_at_saved_zero_deg": float(controller.ClosureAngle),
            "previous_orientation_preserved_within_deg": 0.01,
            "orientation_note": "0.00724 deg old connector clock residual removed so revised motor and clevis axes remain exact",
        },
        "fasteners": {
            "removed_old_objects": removed_fasteners,
            "motor": "6x M4x14 provisional",
            "clevis": "2x Ø3/M3 3D-printed screws, modeled length 25 mm provisional",
            "clevis_fit": "user-confirmed forced insertion into unchanged Ø2 holes",
        },
        "downstream": {
            "rigid_body_contract": "link6 -> fixed -> gripper; revised connector, fixed frame, servo and static fasteners remain one gripper rigid body",
            "ros2_mujoco": "joint frame unchanged; regenerate gripper visual/collision meshes from this FCStd",
            "tcp_status": "unchanged / not guessed",
        },
        "limitations": [
            "M4 tapped depth and final M4 length require physical confirmation",
            "M3 printed screw head/nut form and final length require physical confirmation",
            "the confirmed Ø3-in-Ø2 fit is intentionally interfering and is excluded from collision failure",
            "mass, center of mass and inertia are not inferred from STL density",
        ],
    }

    interface = {
        "schema": "go-m8010-arm-v15-revised-gripper-interface/1.0",
        "revision": REVISION,
        "parent_link": "link6",
        "child_link": "gripper",
        "fixed_joint_origin_xyz_m": [0.0, 0.0, 0.0],
        "fixed_joint_origin_rpy_rad": [0.0, 0.0, 0.0],
        "connector_source_pose_in_link6_mm": placement_dict(
            link6_world.inverse().multiply(connector_world)
        ),
        "gripper_source_pose_in_link6_mm": placement_dict(
            link6_world.inverse().multiply(gripper_world)
        ),
        "j6": {
            "type": "revolute/hinge",
            "axis": [0.0, 0.0, 1.0],
            "range_rad": [-math.pi, math.pi],
            "range_deg": [-180.0, 180.0],
        },
        "user_confirmed_fastener_process": "retain connector Ø2 holes and force in Ø3 3D-printed screws",
    }

    doc.recompute()
    doc.saveAs(str(OUTPUT_FCSTD))

    # Manufacturing/raw connector assets.
    connector_shape.exportStep(str(OUTPUT_CONNECTOR_STEP))
    connector_mesh = MeshPart.meshFromShape(
        Shape=connector_shape,
        LinearDeflection=0.08,
        AngularDeflection=0.15,
        Relative=False,
    )
    connector_mesh.write(str(OUTPUT_CONNECTOR_STL))

    export_objects = tool_export_objects(tool)
    if not export_objects:
        raise RuntimeError("no tool objects selected for export")
    Part.export(export_objects, str(OUTPUT_TOOL_STEP))
    Mesh.export(export_objects, str(OUTPUT_TOOL_STL))
    doc.save()
    App.closeDocument(doc.Name)

    source_hash_after = sha256(SOURCE_FCSTD)
    if source_hash_after != source_hash_before:
        raise RuntimeError("source V15.3 FCStd changed during derived build")
    qa["sources"]["v15_fcstd_sha256_after"] = source_hash_after
    qa["sources"]["v15_source_immutable"] = True
    qa["outputs"] = {
        "fcstd": str(OUTPUT_FCSTD),
        "fcstd_sha256": sha256(OUTPUT_FCSTD),
        "tool_step": str(OUTPUT_TOOL_STEP),
        "tool_stl": str(OUTPUT_TOOL_STL),
        "connector_step": str(OUTPUT_CONNECTOR_STEP),
        "connector_stl": str(OUTPUT_CONNECTOR_STL),
    }
    OUTPUT_QA.write_text(json.dumps(qa, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    OUTPUT_INTERFACE.write_text(
        json.dumps(interface, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )

    OUTPUT_README.write_text(
        f"""# V15.5 夹爪改版连接件装配

- 主装配：`{OUTPUT_FCSTD.name}`
- J6 末端零位 STEP/STL：`{OUTPUT_TOOL_STEP.name}` / `{OUTPUT_TOOL_STL.name}`
- 修复后的连接件：`{OUTPUT_CONNECTOR_STEP.name}` / `{OUTPUT_CONNECTOR_STL.name}`
- 接口验收：`{OUTPUT_QA.name}`
- ROS2 / MuJoCo 接口：`{OUTPUT_INTERFACE.name}`

## 已确认

- 新连接件 6×Ø4 孔按 STL 圆边拟合为 PCD28，安装面 raw Z=10，与 DM-G6220 输出中心和 J6 实测轴注册。
- 夹爪原 3 mm 舌片对准新版中央 3 mm 槽；两条固定孔轴的中心误差为 0（数值容差内）。
- 两个 Ø2 孔原样保留。按用户确认，使用 Ø3/M3 3D 打印螺丝强制压入，属于有意过盈，不是标准间隙孔。
- 模型暂用 M3×25 表示，以覆盖约 22 mm 双耳外宽；最终打印螺丝头、尾部和长度仍需实物确认。
- J6 保持转动关节，限位为 -180°～+180°。
- 夹爪整体朝向与替换前一致；只按新连接件实测接口改变安装位置。

## 下游约束

`link6 -> fixed -> gripper` 不变。连接件、舵机壳体、固定框架和静态紧固件属于同一 gripper 刚体；夹爪内部活动件仍按原运动角色拆分。质量、质心、惯量和 TCP 未经实测，不填写猜测值。
""",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "status": "PASS",
                "fcstd": str(OUTPUT_FCSTD),
                "fcstd_sha256": qa["outputs"]["fcstd_sha256"],
                "seat_center_error_mm": seat_center_error,
                "six_hole_max_axis_error_mm": max(six_hole_errors),
                "clevis_axis_max_error_mm": max(cross_axis_errors),
                "slot_width_error_mm": slot_width_error,
                "insertion_end_residual_mm": insertion_end_residual,
                "j6_range_deg": [-180.0, 180.0],
                "zero_collision_pairs": collision_pairs,
            },
            ensure_ascii=False,
            indent=2,
        ),
        flush=True,
    )


if __name__ == "__main__":
    build()

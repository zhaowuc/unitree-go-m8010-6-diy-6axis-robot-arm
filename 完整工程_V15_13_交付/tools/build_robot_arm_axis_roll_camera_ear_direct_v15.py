from __future__ import annotations

"""Build V15.12: keep the connector fixed and reverse only the gripper.

The revised connector keeps exactly the accepted V15.5 placement on the DM
motor.  Only the gripper rolls 180° about the centre line midway between its
two clevis axes.  This line is parallel to J6, not the transverse screw line.
It does not translate the gripper inward and does not add an adapter, arm,
spacer, or support plate.

The supplied Gemini STEP is attached through the rear hole of each of its two
integral black hinge ears.  The camera bracket face contacts the connector
directly.  The immutable connector source has 66.5 mm pilot pitch while the
measured camera-ear row is 62.600365 mm; therefore this derived connector copy
extends each existing Ø2 pilot inward by 1.949818 mm.  No material is added.
"""

import hashlib
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import FreeCAD as App
import Mesh
import MeshPart
import Part

sys.path.insert(0, r"D:/AI_JIXIEBI/go_m8010_arm_gui")
import build_robot_arm_gripper_connector_revision_v15 as base


SOURCE_ROOT = Path(
    r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15/夹爪改版连接件_v15"
)
SOURCE_FCSTD = SOURCE_ROOT / "机械臂完整装配_六轴_夹爪改版连接件_v15.FCStd"
AXES_JSON = Path(
    r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15/V15_真实关节轴定义.json"
)
CAMERA_STEP = Path(r"D:/AI_JIXIEBI/模型/相机_Gemini+pro相机_带支架铰链.stp")

OUTPUT_ROOT = Path(
    r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15/夹爪单独反转180_连接件不转_相机铰链耳直装_v15"
)
OUTPUT_FCSTD = OUTPUT_ROOT / "机械臂完整装配_六轴_夹爪单独反转180_连接件不转_相机铰链耳直装_v15.FCStd"
OUTPUT_TOOL_STEP = OUTPUT_ROOT / "J6末端_夹爪单独反转180_连接件不转_相机铰链耳直装_零位.step"
OUTPUT_TOOL_STL = OUTPUT_ROOT / "J6末端_夹爪单独反转180_连接件不转_相机铰链耳直装_零位.stl"
OUTPUT_CONNECTOR_STEP = OUTPUT_ROOT / "夹爪改版连接件_相机耳孔直装短槽.step"
OUTPUT_CONNECTOR_STL = OUTPUT_ROOT / "夹爪改版连接件_相机耳孔直装短槽.stl"
OUTPUT_CAMERA_STEP = OUTPUT_ROOT / "GeminiPro相机_铰链耳直装姿态.step"
OUTPUT_CAMERA_STL = OUTPUT_ROOT / "GeminiPro相机_铰链耳直装姿态.stl"
OUTPUT_QA = OUTPUT_ROOT / "QA_V15_夹爪单独反转_连接件不转_相机铰链耳直装.json"
OUTPUT_INTERFACE = OUTPUT_ROOT / "V15_夹爪单独反转与相机接口定义.json"
OUTPUT_README = OUTPUT_ROOT / "README_夹爪单独反转_连接件不转_相机铰链耳直装.md"

EXPECTED_SOURCE_SHA256 = "004BE47FC494BDAF9683D5E9C467275C8EA90EB411B0640838FE31C5A570213E"
EXPECTED_CAMERA_SHA256 = "BA4A86AF13938D9B40FB91CD60E34A8A72037E990C7817F3AC0FAB6C4D3CC971"
REVISION = "V15.12-gripper-only-centre-roll-180-connector-fixed-camera-integral-ear-direct"

ROLL_DEG = 180.0
# Exact centre of the two measured clevis axes in connector raw coordinates.
# The line direction is connector raw +Z (parallel to J6).  Its raw Y=0 is
# 0.0610069 mm from the fitted motor PCD centre because the supplied STL itself
# is slightly asymmetric; choosing the clevis midpoint preserves both gripper
# screw axes exactly instead of forcing a hidden 0.1220138 mm mismatch.
GRIPPER_ROLL_AXIS_POINT_CONNECTOR = App.Vector(
    base.CONNECTOR_PATTERN_CENTER.x, 0.0, -4.0
)
GRIPPER_ROLL_AXIS_DIRECTION_CONNECTOR = App.Vector(0, 0, 1)
CONNECTOR_CAMERA_HOLES_RAW = {
    "right": App.Vector(0.0, 33.25, 5.0),
    "left": App.Vector(0.0, -33.25, 5.0),
}
CONNECTOR_CAMERA_PILOT_DIAMETER_MM = 2.0
CONNECTOR_OUTER_FACE_X_MM = -23.054773330688477

# One measured hole from each of the two integral hinge ears shown in the
# user's physical photo.  These are the rear (+source Y) row, not camera-body
# holes and not an invented adapter interface.
CAMERA_EAR_REAR_HOLES_SOURCE = {
    "right": App.Vector(31.299995000370, 14.527032730908, 14.25),
    "left": App.Vector(-31.299995000357, 14.743823755147, 14.25),
}
CAMERA_EAR_FRONT_HOLES_SOURCE = {
    "right": App.Vector(31.2, 4.5275326934131, 14.25),
    "left": App.Vector(-31.199999999991, 4.744323717654, 14.25),
}
CAMERA_EAR_HOLE_DIAMETER_MM = 3.3
CAMERA_BRACKET_CONTACT_FACE_Z_MM = 15.7500000000003
CAMERA_LENS_CENTERS_SOURCE = (
    App.Vector(-22.5, -9.05, -12.8),
    App.Vector(22.5, -9.05, -12.8),
)
CAMERA_OPTICAL_AXIS_SOURCE = App.Vector(0, -1, 0)
RAY_LENGTH_MM = 500.0
RAY_RADIUS_MM = 0.20
PENETRATION_TOLERANCE_MM3 = 1.0e-3


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def vector(v: App.Vector) -> list[float]:
    return [float(v.x), float(v.y), float(v.z)]


def rotation_about(origin: App.Vector, axis: App.Vector, angle_deg: float) -> App.Placement:
    rotation = App.Rotation(axis, angle_deg)
    return App.Placement(origin - rotation.multVec(origin), rotation)


def shape_at(shape: Part.Shape, placement: App.Placement) -> Part.Shape:
    result = shape.copy()
    result.Placement = placement
    return result


def world_shape(obj) -> Part.Shape:
    return shape_at(obj.Shape, obj.getGlobalPlacement())


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


def overlap(first: Part.Shape, second: Part.Shape) -> float:
    if aabb_separated(first, second):
        return 0.0
    common = first.common(second)
    return 0.0 if common.isNull() else float(common.Volume)


def placement_error(first: App.Placement, second: App.Placement) -> dict:
    translation = (first.Base - second.Base).Length
    delta = first.Rotation.inverted().multiply(second.Rotation)
    angle = abs(float(delta.Angle) * 180.0 / math.pi)
    if angle > 180.0:
        angle = 360.0 - angle
    return {"translation_mm": translation, "rotation_deg": angle}


def camera_to_connector() -> App.Placement:
    # Camera +X -> connector +Y; +Y -> +Z; +Z -> +X.
    # Hence the measured optical direction -Y becomes connector -Z, which is
    # the real J6 outward direction.  The integral ear top face directly
    # contacts the connector outer face without a plate or spacer.
    rotation = base.rotation_from_columns(
        App.Vector(0, 1, 0),
        App.Vector(0, 0, 1),
        App.Vector(1, 0, 0),
    )
    row_y = sum(p.y for p in CAMERA_EAR_REAR_HOLES_SOURCE.values()) / 2.0
    return App.Placement(
        App.Vector(
            CONNECTOR_OUTER_FACE_X_MM - CAMERA_BRACKET_CONTACT_FACE_Z_MM,
            0.0,
            5.0 - row_y,
        ),
        rotation,
    )


def direct_mount_connector_shape(
    source_shape: Part.Shape, camera_holes_connector: dict[str, App.Vector]
) -> Part.Shape:
    result = source_shape.copy()
    # Extend only the two existing camera pilots.  The new cylinders overlap
    # the original Ø2 cylinders to form short slots; no material is added.
    for point in camera_holes_connector.values():
        cutter = Part.makeCylinder(
            CONNECTOR_CAMERA_PILOT_DIAMETER_MM / 2.0,
            9.0,
            App.Vector(CONNECTOR_OUTER_FACE_X_MM - 0.25, point.y, point.z),
            App.Vector(1, 0, 0),
        )
        result = result.cut(cutter)
    if not result.isValid() or not result.isClosed() or len(result.Solids) != 1:
        raise RuntimeError("camera-ear pilot extension did not leave one valid closed connector solid")
    return result


def camera_shape() -> Part.Shape:
    shape = Part.read(str(CAMERA_STEP))
    if shape.isNull() or len(shape.Solids) != 2:
        raise RuntimeError("unexpected Gemini camera STEP geometry")
    return shape


def export_shape(shape: Part.Shape, step_path: Path, stl_path: Path) -> None:
    shape.exportStep(str(step_path))
    mesh = MeshPart.meshFromShape(
        Shape=shape,
        LinearDeflection=0.08,
        AngularDeflection=0.15,
        Relative=False,
    )
    mesh.write(str(stl_path))


def build() -> None:
    for path in (SOURCE_FCSTD, AXES_JSON, CAMERA_STEP):
        if not path.is_file():
            raise FileNotFoundError(path)
    source_hash_before = sha256(SOURCE_FCSTD)
    camera_hash_before = sha256(CAMERA_STEP)
    if source_hash_before != EXPECTED_SOURCE_SHA256:
        raise RuntimeError(f"V15.5 source changed: {source_hash_before}")
    if camera_hash_before != EXPECTED_CAMERA_SHA256:
        raise RuntimeError(f"camera STEP changed: {camera_hash_before}")

    axes = json.loads(AXES_JSON.read_text(encoding="utf-8"))
    j6_frame = axes["frames_at_mechanical_zero"]["link6"]
    j6_origin = App.Vector(*j6_frame["origin_world_mm"])
    j6_axis = App.Vector(*j6_frame["z_axis_world_at_zero"])
    j6_axis.normalize()
    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    doc = App.openDocument(str(SOURCE_FCSTD))
    if doc is None:
        raise RuntimeError("cannot open immutable V15.5 source")
    try:
        r6 = doc.getObject("J6_Driven_Rigid")
        tool = doc.getObject("J6_Gripper_Assembly")
        joint6 = doc.getObject("J6_Revolute")
        connector_display = doc.getObject("J6_Gripper_Connector_STEP_Display")
        connector_proxy = doc.getObject("J6_Gripper_Connector_Collision_Proxy")
        controller = doc.getObject("Gripper_Servo_Controller")
        if None in (r6, tool, joint6, connector_display, connector_proxy, controller):
            raise RuntimeError("V15.5 is missing required J6/tool objects")

        connector_world_0 = App.Placement(tool.ConnectorSourceToWorldAtJ6Zero)
        gripper_world_0 = App.Placement(tool.GripperSourceToWorldAtJ6Zero)
        connector_to_gripper_0 = connector_world_0.inverse().multiply(gripper_world_0)
        # User-confirmed constraint: the connector must not rotate.  Preserve
        # its accepted V15.5 world placement bit-for-bit.
        connector_world = App.Placement(connector_world_0)
        gripper_roll_local = rotation_about(
            GRIPPER_ROLL_AXIS_POINT_CONNECTOR,
            GRIPPER_ROLL_AXIS_DIRECTION_CONNECTOR,
            ROLL_DEG,
        )
        gripper_world = connector_world.multiply(
            gripper_roll_local.multiply(connector_to_gripper_0)
        )

        # Roll only the gripper about the centre axis between the two clevis
        # axes.  This changes servo side while preserving approach and holes.
        for obj in doc.Objects:
            if not obj.Name.startswith("Gripper_"):
                continue
            if "SourcePlacement" not in obj.PropertiesList:
                continue
            desired = gripper_world.multiply(App.Placement(obj.SourcePlacement))
            obj.Placement = base.local_placement(tool, desired)
            base.add_placement(obj, "ReferencePlacement", "Kinematics", obj.Placement)

        # Connector-owned geometry and existing fasteners follow the same J6
        # clock; there is no gripper-only transverse hinge operation.
        for obj in doc.Objects:
            if obj.Name.startswith("DM_G6220_Connector_") or obj.Name.startswith("Gripper_Clevis_"):
                if hasattr(obj, "Shape") and not obj.Shape.isNull():
                    obj.Placement = base.local_placement(tool, connector_world)
                    base.add_placement(obj, "ReferencePlacement", "Kinematics", obj.Placement)

        camera_local = camera_to_connector()
        camera_world = connector_world.multiply(camera_local)
        camera_holes_connector = {
            side: camera_local.multVec(point)
            for side, point in CAMERA_EAR_REAR_HOLES_SOURCE.items()
        }

        source_connector_shape = connector_display.Shape.copy()
        source_connector_shape.Placement = App.Placement()
        revised_connector = direct_mount_connector_shape(
            source_connector_shape, camera_holes_connector
        )
        for obj, label, visible, transparency in (
            (
                connector_display,
                "夹爪改版连接件 / 保持V15.5方向不旋转 / 相机铰链耳直接接触 / 无加件",
                True,
                0,
            ),
            (
                connector_proxy,
                "夹爪改版连接件精确碰撞体 / 相机耳直装短槽",
                False,
                70,
            ),
        ):
            obj.Shape = revised_connector.copy()
            obj.Placement = base.local_placement(tool, connector_world)
            obj.Label = label
            base.add_placement(obj, "ReferencePlacement", "Kinematics", obj.Placement)
            base.add_text(obj, "CameraMountModification", "Camera", "only two original Ø2 camera pilots extended inward; no added material")
            base.configure_view(obj, (0.78, 0.16, 0.20), transparency, visible)

        old_camera = doc.getObject("Gemini_Pro_Camera_STEP_Display")
        if old_camera is not None:
            doc.removeObject(old_camera.Name)
        camera_obj = tool.newObject("Part::Feature", "Gemini_Pro_Camera_STEP_Display")
        camera_obj.Label = "Gemini Pro相机 / 使用自带黑色铰链双耳后排两孔直接安装"
        camera_obj.Shape = camera_shape()
        camera_obj.Placement = base.local_placement(tool, camera_world)
        base.add_bool(camera_obj, "CollisionEnabled", "Collision", True)
        base.add_bool(camera_obj, "MassEnabled", "Physics", False)
        base.add_bool(camera_obj, "TransitionPartPresent", "Mount", False)
        base.add_text(camera_obj, "MountPart", "Mount", "camera integral hinge ears from supplied STEP")
        base.add_text(camera_obj, "UsedEarHoles", "Mount", "rear row: one measured Ø3.3 hole on each of the two ears")
        base.add_text(camera_obj, "FastenerGeometry", "Mount", "not modeled; screw type and length not provided")
        base.add_text(camera_obj, "SourceFile", "Source", str(CAMERA_STEP))
        base.add_text(camera_obj, "SourceSHA256", "Source", camera_hash_before)
        base.add_placement(camera_obj, "ReferencePlacement", "Kinematics", camera_obj.Placement)
        base.configure_view(camera_obj, (0.02, 0.55, 0.64), 0, True)

        tool.Label = "J6末端：连接件不转 / 夹爪单独绕安装中心轴反转180° / Gemini铰链耳直装"
        base.add_placement(tool, "ConnectorSourceToWorldAtJ6Zero", "Reference", connector_world)
        base.add_placement(tool, "GripperSourceToWorldAtJ6Zero", "Reference", gripper_world)
        base.add_placement(tool, "GripperSourceToR6AtZero", "Reference", base.local_placement(r6, gripper_world))
        base.add_placement(tool, "CameraSourceToWorldAtJ6Zero", "Reference", camera_world)
        base.add_text(tool, "ReversalDefinition", "Assembly", "connector remains at V15.5 placement; only gripper rolls 180 deg about clevis-midpoint axis parallel to J6; approach unchanged")
        base.add_bool(tool, "AddedCameraAdapter", "Assembly", False)
        base.add_bool(tool, "AddedGripperSupport", "Assembly", False)

        controller.ClosureAngle = 0.0
        base.add_placement(controller, "GripperSourceToR6AtZero", "Reference", base.local_placement(r6, gripper_world))
        base.add_text(controller, "ConnectorRevision", "Reference", REVISION)

        record = doc.getObject("V15_12_Gripper_Only_Roll_Camera_Ear_Direct_Record")
        if record is None:
            record = doc.addObject("App::FeaturePython", "V15_12_Gripper_Only_Roll_Camera_Ear_Direct_Record")
        record.Label = "V15.12 连接件不转、夹爪单独反转与相机铰链耳直装实测记录"
        base.add_text(record, "Revision", "Revision", REVISION)
        base.add_float_list(record, "ToolClockAboutMeasuredJ6Deg", "Kinematics", [ROLL_DEG])
        base.add_text(record, "ConnectorMotion", "Kinematics", "0 deg / unchanged from accepted V15.5 placement")
        base.add_text(record, "GripperMotion", "Kinematics", "gripper only: 180 deg roll about clevis-midpoint centre axis parallel to J6; no inward folding and no transverse screw-line hinge")
        base.add_text(record, "CameraConnection", "Camera", "supplied camera integral black hinge ears directly contact connector")
        base.add_bool(record, "AnyAddedPlateArmSpacer", "Camera", False)
        base.add_vector(record, "CameraEarRearHoleRightSourceMM", "Camera", CAMERA_EAR_REAR_HOLES_SOURCE["right"])
        base.add_vector(record, "CameraEarRearHoleLeftSourceMM", "Camera", CAMERA_EAR_REAR_HOLES_SOURCE["left"])
        base.add_length(record, "CameraEarHoleDiameter", "Camera", CAMERA_EAR_HOLE_DIAMETER_MM)
        base.add_text(record, "CameraFastenerStatus", "Camera", "not modeled; real screw specification not supplied")

        doc.recompute()

        # --- Independent numerical QA ---
        relative_after = connector_world.inverse().multiply(gripper_world)
        expected_relative_after = gripper_roll_local.multiply(connector_to_gripper_0)
        rigid_error = placement_error(expected_relative_after, relative_after)
        if rigid_error["translation_mm"] > 1.0e-7 or rigid_error["rotation_deg"] > 1.0e-7:
            raise RuntimeError(f"gripper centre-axis roll transform error: {rigid_error}")
        connector_unchanged_error = placement_error(connector_world_0, connector_world)
        if (
            connector_unchanged_error["translation_mm"] > 1.0e-12
            or connector_unchanged_error["rotation_deg"] > 1.0e-12
        ):
            raise RuntimeError(f"connector moved despite user constraint: {connector_unchanged_error}")

        # The physical approach direction is J6.  Both clocks are about axes
        # parallel to J6, so this direction must remain exactly unchanged.
        j6_axis_after = connector_world.Rotation.multVec(
            GRIPPER_ROLL_AXIS_DIRECTION_CONNECTOR
        )
        j6_axis_after.normalize()
        j6_axis_dot = abs(j6_axis.dot(j6_axis_after))

        # The two source tongue-hole axes must swap onto the two connector
        # clevis axes exactly after the 180° centre-axis roll.
        source_holes = (
            App.Vector(114.221654026735, -24.164357793295, -10.298383459992),
            App.Vector(124.221654026735, -24.164357793295, -10.298383459992),
        )
        target_holes = (
            App.Vector(0.0, -5.0, -4.0),
            App.Vector(0.0, 5.0, -4.0),
        )
        clevis_axis_errors = []
        for source_point, target_point in zip(source_holes, target_holes):
            transformed = relative_after.multVec(source_point)
            # Both target lines are parallel to connector raw X; X displacement
            # is irrelevant to coaxiality, so compare their Y/Z coordinates.
            clevis_axis_errors.append(
                math.hypot(transformed.y - target_point.y, transformed.z - target_point.z)
            )
        if max(clevis_axis_errors) > 1.0e-7:
            raise RuntimeError(f"gripper clevis axes lost coaxiality: {clevis_axis_errors}")

        mount_axis_world = camera_world.Rotation.multVec(App.Vector(0, 0, 1))
        connector_mount_axis_world = connector_world.Rotation.multVec(App.Vector(1, 0, 0))
        mount_axis_world.normalize()
        connector_mount_axis_world.normalize()
        mount_axis_dot = abs(mount_axis_world.dot(connector_mount_axis_world))
        hole_errors = {}
        for side, source_point in CAMERA_EAR_REAR_HOLES_SOURCE.items():
            camera_point = camera_world.multVec(source_point)
            connector_point = connector_world.multVec(camera_holes_connector[side])
            delta = camera_point - connector_point
            parallel = abs(delta.dot(connector_mount_axis_world))
            perpendicular = math.sqrt(max(delta.Length * delta.Length - parallel * parallel, 0.0))
            hole_errors[side] = perpendicular
        max_hole_error = max(hole_errors.values())
        if max_hole_error > 1.0e-7 or mount_axis_dot < 0.999999999:
            raise RuntimeError(f"camera ear holes are not coaxial: {hole_errors}, dot={mount_axis_dot}")

        original_pitch = (
            CONNECTOR_CAMERA_HOLES_RAW["right"] - CONNECTOR_CAMERA_HOLES_RAW["left"]
        ).Length
        camera_pitch = (
            camera_holes_connector["right"] - camera_holes_connector["left"]
        ).Length
        pilot_extension_each = (original_pitch - camera_pitch) / 2.0

        camera_world_shape = shape_at(camera_obj.Shape, camera_world)
        connector_world_shape = shape_at(revised_connector, connector_world)
        collision_rows = {
            "camera_vs_connector_mm3": overlap(camera_world_shape, connector_world_shape)
        }
        obstacle_shapes = {"connector": connector_world_shape}
        for name in (
            "Gripper_Fixed_Frame_Collision_Proxy",
            "Gripper_Servo_Collision_Proxy",
            "Gripper_Left_Outer_Link",
            "Gripper_Right_Outer_Link",
            "Gripper_Left_Drive_Link",
            "Gripper_Right_Drive_Link",
            "Gripper_Left_Finger",
            "Gripper_Right_Finger",
        ):
            obj = doc.getObject(name)
            if obj is None:
                raise RuntimeError(f"missing gripper QA object: {name}")
            placed = world_shape(obj)
            obstacle_shapes[name] = placed
            collision_rows[f"camera_vs_{name}_mm3"] = overlap(camera_world_shape, placed)
        unexpected = {
            name: volume
            for name, volume in collision_rows.items()
            if volume > PENETRATION_TOLERANCE_MM3
        }
        if unexpected:
            raise RuntimeError(f"camera penetration after correct J6-axis roll: {unexpected}")

        optical_axis = camera_world.Rotation.multVec(CAMERA_OPTICAL_AXIS_SOURCE)
        optical_axis.normalize()
        optical_outward_dot = optical_axis.dot(j6_axis)
        ray_rows = []
        for index, lens in enumerate(CAMERA_LENS_CENTERS_SOURCE, 1):
            origin = camera_world.multVec(lens) + optical_axis * 0.5
            ray = Part.makeCylinder(
                RAY_RADIUS_MM, RAY_LENGTH_MM, origin, optical_axis
            )
            blockers = {
                name: overlap(ray, shape)
                for name, shape in obstacle_shapes.items()
            }
            ray_rows.append(
                {
                    "lens": index,
                    "origin_world_mm": vector(origin),
                    "axis_world": vector(optical_axis),
                    "blocker_overlap_mm3": blockers,
                    "clear": all(v <= PENETRATION_TOLERANCE_MM3 for v in blockers.values()),
                }
            )
        if optical_outward_dot < 0.999999999 or not all(row["clear"] for row in ray_rows):
            raise RuntimeError(
                f"camera optical center rays failed: dot={optical_outward_dot}, rays={ray_rows}"
            )

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
            raise RuntimeError(f"J6 is no longer +/-180 deg: {j6_limit}")

        qa = {
            "status": "PASS",
            "revision": REVISION,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "measurement_policy": "measured BRep/STL axes only; no visual placement",
            "sources": {
                "v15_5_fcstd": str(SOURCE_FCSTD),
                "v15_5_sha256_before": source_hash_before,
                "camera_step": str(CAMERA_STEP),
                "camera_sha256_before": camera_hash_before,
            },
            "corrected_gripper_reversal": {
                "connector_rotation_deg": 0.0,
                "connector_unchanged_from_v15_5_error": connector_unchanged_error,
                "gripper_roll_axis": "clevis-axis midpoint, connector raw +Z, parallel to J6",
                "gripper_roll_axis_point_connector_raw_mm": vector(GRIPPER_ROLL_AXIS_POINT_CONNECTOR),
                "gripper_roll_axis_direction_connector_raw": vector(GRIPPER_ROLL_AXIS_DIRECTION_CONNECTOR),
                "gripper_roll_axis_point_world_mm": vector(
                    connector_world.multVec(GRIPPER_ROLL_AXIS_POINT_CONNECTOR)
                ),
                "gripper_roll_axis_direction_world": vector(
                    connector_world.Rotation.multVec(GRIPPER_ROLL_AXIS_DIRECTION_CONNECTOR)
                ),
                "gripper_roll_axis_offset_from_fitted_j6_mm": abs(base.CONNECTOR_PATTERN_CENTER.y),
                "gripper_roll_deg": ROLL_DEG,
                "requested_roll_transform_error": rigid_error,
                "clevis_axis_errors_mm": clevis_axis_errors,
                "maximum_clevis_axis_error_mm": max(clevis_axis_errors),
                "approach_axis_j6_dot_before_after": j6_axis_dot,
                "inward_transverse_hinge_used": False,
                "gripper_translation_added_mm": 0.0,
            },
            "camera_integral_ear_direct_mount": {
                "physical_interface": "two black hinge ears contained in supplied camera STEP",
                "used_holes": "rear row; one Ø3.3 hole on each ear",
                "camera_rear_holes_source_mm": {
                    k: vector(v) for k, v in CAMERA_EAR_REAR_HOLES_SOURCE.items()
                },
                "camera_front_holes_source_mm_retained_unused": {
                    k: vector(v) for k, v in CAMERA_EAR_FRONT_HOLES_SOURCE.items()
                },
                "camera_holes_connector_raw_mm": {
                    k: vector(v) for k, v in camera_holes_connector.items()
                },
                "coaxial_perpendicular_errors_mm": hole_errors,
                "maximum_coaxial_error_mm": max_hole_error,
                "axis_dot": mount_axis_dot,
                "connector_original_pilot_pitch_mm": original_pitch,
                "camera_used_row_pitch_mm": camera_pitch,
                "pitch_mismatch_mm": original_pitch - camera_pitch,
                "required_inward_extension_each_mm": pilot_extension_each,
                "connector_change": "two existing Ø2 camera pilots extended inward as short slots; material removed only",
                "added_adapter_plate_arm_spacer_count": 0,
                "added_camera_fastener_solids_count": 0,
                "fastener_note": "real screw type and length were not supplied, so no screw geometry was invented",
                "direct_contact_faces": {
                    "camera_source_z_mm": CAMERA_BRACKET_CONTACT_FACE_Z_MM,
                    "connector_raw_x_mm": CONNECTOR_OUTER_FACE_X_MM,
                },
            },
            "collision_at_zero": {
                "penetration_tolerance_mm3": PENETRATION_TOLERANCE_MM3,
                "pairs": collision_rows,
                "unexpected_collision": False,
            },
            "camera_visibility": {
                "optical_axis_world": vector(optical_axis),
                "optical_outward_dot_j6": optical_outward_dot,
                "measured_lens_center_rays": ray_rows,
                "center_rays_clear": True,
                "limitation": "complete FOV still requires real Gemini intrinsics",
            },
            "j6": {
                **j6_limit,
                "range_rad": [-math.pi, math.pi],
                "verified_plus_minus_180": True,
            },
            "downstream": {
                "rigid_topology": "link6 -> fixed -> gripper",
                "camera_role": "fixed geometry in gripper rigid body",
                "ros2_mujoco": "J1-J6 topology unchanged; regenerate terminal visual/collision assets from this revision",
            },
            "limitations": [
                "camera screw type, length and tightening method were not provided and are not modeled",
                "full camera FOV needs actual Gemini calibration/intrinsics",
                "mass, center of mass and inertia are not guessed from nominal material",
            ],
        }

        interface = {
            "schema": "go-m8010-arm-v15-axis-roll-camera-ear-direct/1.0",
            "revision": REVISION,
            "parent_link": "link6",
            "child_link": "gripper",
            "fixed_joint_origin_xyz_m": [0.0, 0.0, 0.0],
            "fixed_joint_origin_rpy_rad": [0.0, 0.0, 0.0],
            "connector_rotation_from_v15_5_deg": 0.0,
            "gripper_roll_about_clevis_midpoint_axis_deg": ROLL_DEG,
            "connector_source_pose_in_link6_mm": base.placement_dict(
                r6.getGlobalPlacement().inverse().multiply(connector_world)
            ),
            "gripper_source_pose_in_link6_mm": base.placement_dict(
                r6.getGlobalPlacement().inverse().multiply(gripper_world)
            ),
            "camera_source_pose_in_link6_mm": base.placement_dict(
                r6.getGlobalPlacement().inverse().multiply(camera_world)
            ),
            "j6": {
                "type": "revolute/hinge",
                "axis": [0.0, 0.0, 1.0],
                "range_rad": [-math.pi, math.pi],
                "range_deg": [-180.0, 180.0],
            },
            "camera_mount": {
                "joint": "fixed",
                "interface": "supplied camera integral hinge ears / rear row",
                "added_transition_part": False,
            },
        }

        doc.recompute()
        doc.saveAs(str(OUTPUT_FCSTD))

        export_shape(revised_connector, OUTPUT_CONNECTOR_STEP, OUTPUT_CONNECTOR_STL)
        export_shape(camera_world_shape, OUTPUT_CAMERA_STEP, OUTPUT_CAMERA_STL)
        export_objects = base.tool_export_objects(tool)
        if camera_obj not in export_objects:
            export_objects.append(camera_obj)
        Part.export(export_objects, str(OUTPUT_TOOL_STEP))
        Mesh.export(export_objects, str(OUTPUT_TOOL_STL))
        doc.save()
    finally:
        App.closeDocument(doc.Name)

    source_hash_after = sha256(SOURCE_FCSTD)
    camera_hash_after = sha256(CAMERA_STEP)
    if source_hash_after != source_hash_before or camera_hash_after != camera_hash_before:
        raise RuntimeError("immutable source changed during derived build")
    qa["sources"].update(
        {
            "v15_5_sha256_after": source_hash_after,
            "camera_sha256_after": camera_hash_after,
            "sources_immutable": True,
        }
    )
    qa["outputs"] = {
        "fcstd": str(OUTPUT_FCSTD),
        "fcstd_sha256": sha256(OUTPUT_FCSTD),
        "tool_step": str(OUTPUT_TOOL_STEP),
        "tool_stl": str(OUTPUT_TOOL_STL),
        "connector_step": str(OUTPUT_CONNECTOR_STEP),
        "connector_stl": str(OUTPUT_CONNECTOR_STL),
        "camera_step_assembled": str(OUTPUT_CAMERA_STEP),
        "camera_stl_assembled": str(OUTPUT_CAMERA_STL),
    }
    OUTPUT_QA.write_text(json.dumps(qa, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    OUTPUT_INTERFACE.write_text(
        json.dumps(interface, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    OUTPUT_README.write_text(
        f"""# V15.12 连接件不转 + 夹爪单独反转 180° + Gemini 相机铰链耳直装

- 主装配：`{OUTPUT_FCSTD.name}`
- J6 末端 STEP/STL：`{OUTPUT_TOOL_STEP.name}` / `{OUTPUT_TOOL_STL.name}`
- 直装连接件 STEP/STL：`{OUTPUT_CONNECTOR_STEP.name}` / `{OUTPUT_CONNECTOR_STL.name}`
- 相机装配姿态 STEP/STL：`{OUTPUT_CAMERA_STEP.name}` / `{OUTPUT_CAMERA_STL.name}`
- 数值验收：`{OUTPUT_QA.name}`
- ROS2 / MuJoCo 接口：`{OUTPUT_INTERFACE.name}`

## 本次纠正

- 蓝色连接件保持 V15.5 在达妙电机上的原方向，旋转量为 `0°`；只把夹爪绕两颗固定孔正中、与 J6 平行的中心轴滚转 `180°`。夹爪伸出方向不变，未绕横向螺孔线向机械臂内部折回，也没有增加平移。
- 相机只使用随 STEP 提供的黑色铰链双耳；选用每个耳板的后排 Ø3.3 孔各一个，耳板安装面直接接触连接件。
- 新增转接片、支承臂、垫块、过渡件数量均为 `0`；因未提供相机螺钉规格，本版也没有虚构相机螺钉实体。

## 实测孔距差

连接件两处原 Ø2 相机预留孔距为 `{qa['camera_integral_ear_direct_mount']['connector_original_pilot_pitch_mm']:.6f} mm`，相机双耳所用后排孔距为 `{qa['camera_integral_ear_direct_mount']['camera_used_row_pitch_mm']:.6f} mm`，相差 `{qa['camera_integral_ear_direct_mount']['pitch_mismatch_mm']:.6f} mm`。派生连接件只将两个原孔各向内延伸 `{qa['camera_integral_ear_direct_mount']['required_inward_extension_each_mm']:.6f} mm` 成短槽，不增加材料；原始 V15.5 FCStd 和相机 STEP 均未修改。

## 已验收

- J6 保持 revolute `-180° .. +180°`。
- 连接件位姿与 V15.5 完全一致；两颗原夹爪固定孔在反转后互换，孔轴同轴误差为零（数值容差内）。
- 相机后排两安装孔轴与派生孔轴同轴；相机与连接件/夹爪精确 BRep 穿插为零。
- 两条实测镜头中心光线沿 J6 外向且无实体遮挡；完整 FOV 仍需真实 Gemini 内参验证。
""",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "status": qa["status"],
                "revision": REVISION,
                "fcstd": str(OUTPUT_FCSTD),
                "fcstd_sha256": qa["outputs"]["fcstd_sha256"],
                "j6_range_deg": [-180.0, 180.0],
                "connector_rotation_deg": 0.0,
                "gripper_centre_axis_roll_deg": ROLL_DEG,
                "maximum_clevis_axis_error_mm": qa["corrected_gripper_reversal"]["maximum_clevis_axis_error_mm"],
                "camera_mount_max_error_mm": qa["camera_integral_ear_direct_mount"]["maximum_coaxial_error_mm"],
                "added_transition_parts": 0,
                "camera_center_rays_clear": True,
            },
            ensure_ascii=False,
            indent=2,
        ),
        flush=True,
    )


if __name__ == "__main__":
    build()

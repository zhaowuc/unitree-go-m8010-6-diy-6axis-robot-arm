from __future__ import annotations

"""Build V15.13 with the user-confirmed camera-up J6 mechanical zero.

V15.12 already contains the accepted connector, gripper reversal and direct
Gemini hinge-ear mount.  This builder does not reclock any of those interfaces
relative to one another.  It rotates the complete J6 output rigid body by
180 degrees about the measured DM-G6220 axis, then defines that physical pose
as J6 = 0 degrees while retaining the full -180..+180 degree joint range.
"""

import copy
import hashlib
import json
import math
import shutil
import sys
from datetime import datetime, timezone
from pathlib import Path

import FreeCAD as App
import Mesh
import Part

sys.path.insert(0, r"D:/AI_JIXIEBI/go_m8010_arm_gui")
import build_robot_arm_gripper_connector_revision_v15 as base
import build_robot_arm_axis_roll_camera_ear_direct_v15 as v1512


SOURCE_ROOT = Path(
    r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15/"
    r"夹爪单独反转180_连接件不转_相机铰链耳直装_v15"
)
SOURCE_FCSTD = SOURCE_ROOT / "机械臂完整装配_六轴_夹爪单独反转180_连接件不转_相机铰链耳直装_v15.FCStd"
SOURCE_AXES = Path(r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15/V15_真实关节轴定义.json")
SOURCE_VIDEO = Path(
    r"C:/Users/91592/xwechat_files/wxid_h3dw5kd9queg22_8478/msg/video/"
    r"2026-08/20954c166d1bc421cfe79299d02cfc75.mp4"
)
SOURCE_CONTACT_SHEET = Path(
    r"D:/AI_JIXIEBI/go_m8010_arm_gui/tmp/physical_motion_video_20954/contact_sheet_2s.png"
)

OUTPUT_ROOT = Path(
    r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15/"
    r"V15_13_整机深度复核_相机上置机械零位"
)
OUTPUT_FCSTD = OUTPUT_ROOT / "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
OUTPUT_AXES = OUTPUT_ROOT / "V15_13_真实关节轴与相机上置零位.json"
OUTPUT_TOOL_STEP = OUTPUT_ROOT / "J6末端_相机上置机械零位_v15_13.step"
OUTPUT_TOOL_STL = OUTPUT_ROOT / "J6末端_相机上置机械零位_v15_13.stl"
OUTPUT_QA = OUTPUT_ROOT / "QA_V15_13_相机上置机械零位_几何复核.json"
OUTPUT_README = OUTPUT_ROOT / "README_V15_13_相机上置机械零位.md"
EVIDENCE_DIR = OUTPUT_ROOT / "evidence"

EXPECTED_SOURCE_SHA256 = "0A4EFA4668B8E5FAC613FDDA8F2D0BC1D70745A40F254800ACBBEF15125B9220"
EXPECTED_VIDEO_SHA256 = "9716C03D6DE3D36FB3A2E4E7BF8B6AB124A95F1610298C351F7DC6334CA774FD"
REVISION = "V15.13-camera-up-j6-mechanical-zero-full-range"
J6_ZERO_RECLOCK_DEG = 180.0
PLACEMENT_TOLERANCE_MM = 1.0e-8
PLACEMENT_TOLERANCE_DEG = 1.0e-8


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def vector(value: App.Vector) -> list[float]:
    return [float(value.x), float(value.y), float(value.z)]


def placement_dict(value: App.Placement) -> dict:
    quaternion = value.Rotation.Q
    return {
        "base_mm": vector(value.Base),
        "quaternion_xyzw": [
            float(quaternion[0]),
            float(quaternion[1]),
            float(quaternion[2]),
            float(quaternion[3]),
        ],
    }


def placement_error(first: App.Placement, second: App.Placement) -> dict:
    translation = float((first.Base - second.Base).Length)
    delta = first.Rotation.inverted().multiply(second.Rotation)
    rotation = abs(math.degrees(float(delta.Angle)))
    rotation = min(rotation, abs(360.0 - rotation))
    return {"translation_mm": translation, "rotation_deg": rotation}


def rotation_about(origin: App.Vector, axis: App.Vector, degrees: float) -> App.Placement:
    direction = App.Vector(axis)
    direction.normalize()
    rotation = App.Rotation(direction, degrees)
    return App.Placement(origin - rotation.multVec(origin), rotation)


def ensure_property(obj, type_id: str, name: str, group: str) -> None:
    if name not in obj.PropertiesList:
        obj.addProperty(type_id, name, group)


def set_text(obj, name: str, value: str, group: str = "V15.13") -> None:
    ensure_property(obj, "App::PropertyString", name, group)
    setattr(obj, name, str(value))


def set_bool(obj, name: str, value: bool, group: str = "V15.13") -> None:
    ensure_property(obj, "App::PropertyBool", name, group)
    setattr(obj, name, bool(value))


def set_angle(obj, name: str, value: float, group: str = "V15.13") -> None:
    ensure_property(obj, "App::PropertyAngle", name, group)
    setattr(obj, name, float(value))


def set_vector(obj, name: str, value: App.Vector, group: str = "V15.13") -> None:
    ensure_property(obj, "App::PropertyVector", name, group)
    setattr(obj, name, App.Vector(value))


def set_placement(obj, name: str, value: App.Placement, group: str = "V15.13") -> None:
    ensure_property(obj, "App::PropertyPlacement", name, group)
    setattr(obj, name, App.Placement(value))


def frame_placement(frame: dict) -> App.Placement:
    matrix = App.Matrix()
    x = frame["x_axis_world_at_zero"]
    y = frame["y_axis_world_at_zero"]
    z = frame["z_axis_world_at_zero"]
    matrix.A11, matrix.A12, matrix.A13 = x[0], y[0], z[0]
    matrix.A21, matrix.A22, matrix.A23 = x[1], y[1], z[1]
    matrix.A31, matrix.A32, matrix.A33 = x[2], y[2], z[2]
    return App.Placement(App.Vector(*frame["origin_world_mm"]), App.Rotation(matrix))


def rotation_matrix(placement: App.Placement) -> list[list[float]]:
    matrix = placement.Rotation.toMatrix()
    return [
        [float(matrix.A11), float(matrix.A12), float(matrix.A13)],
        [float(matrix.A21), float(matrix.A22), float(matrix.A23)],
        [float(matrix.A31), float(matrix.A32), float(matrix.A33)],
    ]


def matrix_to_rpy(matrix: list[list[float]]) -> list[float]:
    sy = math.sqrt(matrix[0][0] ** 2 + matrix[1][0] ** 2)
    if sy >= 1.0e-12:
        return [
            math.atan2(matrix[2][1], matrix[2][2]),
            math.atan2(-matrix[2][0], sy),
            math.atan2(matrix[1][0], matrix[0][0]),
        ]
    return [
        math.atan2(-matrix[1][2], matrix[1][1]),
        math.atan2(-matrix[2][0], sy),
        0.0,
    ]


def world_shape(obj) -> Part.Shape:
    result = obj.Shape.copy()
    result.Placement = obj.getGlobalPlacement()
    return result


def shape_bounds(shape: Part.Shape) -> list[float]:
    box = shape.BoundBox
    return [
        float(box.XMin),
        float(box.YMin),
        float(box.ZMin),
        float(box.XMax),
        float(box.YMax),
        float(box.ZMax),
    ]


def camera_conservative_proxy(camera_obj) -> tuple[Part.Shape, list[dict]]:
    source = camera_obj.Shape.copy()
    source.Placement = App.Placement()
    boxes = []
    rows = []
    for index, solid in enumerate(source.Solids, 1):
        bound = solid.BoundBox
        box = Part.makeBox(
            bound.XLength,
            bound.YLength,
            bound.ZLength,
            App.Vector(bound.XMin, bound.YMin, bound.ZMin),
        )
        boxes.append(box)
        rows.append(
            {
                "source_solid_index": index,
                "source_bound_mm": [
                    float(bound.XMin),
                    float(bound.YMin),
                    float(bound.ZMin),
                    float(bound.XMax),
                    float(bound.YMax),
                    float(bound.ZMax),
                ],
            }
        )
    if len(boxes) != 2:
        raise RuntimeError(f"expected two camera STEP solids, got {len(boxes)}")
    return Part.makeCompound(boxes), rows


def update_axis_contract(axis_data: dict, video_hash: str) -> dict:
    result = copy.deepcopy(axis_data)
    result["revision"] = REVISION
    result["generated_at_utc"] = datetime.now(timezone.utc).isoformat()
    result["derived_from_axis_revision"] = axis_data.get("revision")
    result["j6_zero_recalibration"] = {
        "policy": "camera-up physical pose is J6=0 deg",
        "legacy_v15_12_clock_offset_deg": J6_ZERO_RECLOCK_DEG,
        "position_range_deg": [-180.0, 180.0],
        "connector_relative_to_output_flange_changed": False,
        "camera_relative_to_connector_changed": False,
        "gripper_relative_to_connector_changed": False,
        "video_sha256": video_hash,
    }

    frames = result["frames_at_mechanical_zero"]
    old_link6 = frame_placement(frames["link6"])
    axis = old_link6.Rotation.multVec(App.Vector(0, 0, 1))
    axis.normalize()
    reclock = rotation_about(old_link6.Base, axis, J6_ZERO_RECLOCK_DEG)
    new_link6 = reclock.multiply(old_link6)
    matrix = rotation_matrix(new_link6)
    frames["link6"].update(
        {
            "origin_world_mm": vector(new_link6.Base),
            "x_axis_world_at_zero": [matrix[0][0], matrix[1][0], matrix[2][0]],
            "y_axis_world_at_zero": [matrix[0][1], matrix[1][1], matrix[2][1]],
            "z_axis_world_at_zero": [matrix[0][2], matrix[1][2], matrix[2][2]],
            "zero_note": "V15.13 camera-up physical J6 output clock; J6 angle property remains 0 deg",
        }
    )

    j6 = next(row for row in result["joints"] if row["name"] == "J6")
    parent = frame_placement(frames["link5"])
    relative = parent.inverse().multiply(new_link6)
    relative_matrix = rotation_matrix(relative)
    relative_rpy = matrix_to_rpy(relative_matrix)
    relative_q = relative.Rotation.Q
    relative_pose = j6["parent_to_child_at_mechanical_zero"]
    relative_pose.update(
        {
            "rotation_matrix_row_major": relative_matrix,
            "rpy_rad_urdf_fixed_axis": relative_rpy,
            "quaternion_xyzw": [
                float(relative_q[0]),
                float(relative_q[1]),
                float(relative_q[2]),
                float(relative_q[3]),
            ],
            "quaternion_wxyz_mujoco": [
                float(relative_q[3]),
                float(relative_q[0]),
                float(relative_q[1]),
                float(relative_q[2]),
            ],
            "semantics": "pose of camera-up link6 frame in link5 at J6 mechanical zero",
        }
    )
    j6["ros2_urdf"]["origin_rpy_rad"] = relative_rpy
    j6["mujoco_mjcf"]["body_quat_wxyz_in_parent"] = relative_pose[
        "quaternion_wxyz_mujoco"
    ]
    j6["mujoco_mjcf"]["joint_ref_rad"] = 0.0
    j6["mechanical_zero_deg"] = 0.0
    j6["zero_definition"] = (
        "USER_CONFIRMED_PHYSICAL_VIDEO: complete J6 output rigid body clocked so Gemini camera is above; "
        "this pose is software mechanical zero"
    )
    j6["hardware_encoder_zero_status"] = (
        "MECHANICAL_REFERENCE_DEFINED; electrical encoder offset must be written and verified at low torque"
    )
    j6["legacy_v15_12_clock_offset_deg"] = J6_ZERO_RECLOCK_DEG
    return result


def build() -> None:
    for required in (SOURCE_FCSTD, SOURCE_AXES, SOURCE_VIDEO):
        if not required.is_file():
            raise FileNotFoundError(required)
    source_hash_before = sha256(SOURCE_FCSTD)
    video_hash = sha256(SOURCE_VIDEO)
    if source_hash_before != EXPECTED_SOURCE_SHA256:
        raise RuntimeError(f"unexpected V15.12 source hash: {source_hash_before}")
    if video_hash != EXPECTED_VIDEO_SHA256:
        raise RuntimeError(f"unexpected physical motion video hash: {video_hash}")

    OUTPUT_ROOT.mkdir(parents=True, exist_ok=True)
    EVIDENCE_DIR.mkdir(parents=True, exist_ok=True)
    axes = json.loads(SOURCE_AXES.read_text(encoding="utf-8"))
    updated_axes = update_axis_contract(axes, video_hash)

    doc = App.openDocument(str(SOURCE_FCSTD))
    if doc is None:
        raise RuntimeError("cannot open V15.12 source")
    try:
        r6 = doc.getObject("J6_Driven_Rigid")
        tool = doc.getObject("J6_Gripper_Assembly")
        j6 = doc.getObject("J6_Revolute")
        camera = doc.getObject("Gemini_Pro_Camera_STEP_Display")
        connector = doc.getObject("J6_Gripper_Connector_STEP_Display")
        output_rotor = doc.getObject("J6_DM_G6220_Output_Rotor_STEP_Display")
        fixed_frame = doc.getObject("Gripper_Fixed_Frame_STEP_Display")
        controller = doc.getObject("Gripper_Servo_Controller")
        if None in (r6, tool, j6, camera, connector, output_rotor, fixed_frame, controller):
            raise RuntimeError("V15.12 is missing required J6 terminal objects")
        for index in range(1, 7):
            joint = doc.getObject(f"J{index}_Revolute")
            if joint is None or abs(float(joint.Angle)) > 1.0e-9:
                raise RuntimeError(f"source is not saved at J{index}=0 deg")

        origin = App.Placement(j6.Placement1).Base
        axis = App.Placement(j6.Placement1).Rotation.multVec(App.Vector(0, 0, 1))
        axis.normalize()
        reclock = rotation_about(origin, axis, J6_ZERO_RECLOCK_DEG)

        before = {
            "output": App.Placement(output_rotor.getGlobalPlacement()),
            "connector": App.Placement(connector.getGlobalPlacement()),
            "camera": App.Placement(camera.getGlobalPlacement()),
            "fixed_frame": App.Placement(fixed_frame.getGlobalPlacement()),
        }
        relative_before = {
            "connector_to_output": before["output"].inverse().multiply(before["connector"]),
            "camera_to_connector": before["connector"].inverse().multiply(before["camera"]),
            "fixed_frame_to_connector": before["connector"].inverse().multiply(before["fixed_frame"]),
        }
        source_camera_world_bounds = shape_bounds(world_shape(camera))

        rotated_names = []
        for obj in tuple(r6.Group):
            if obj is tool:
                continue
            if "Placement" in obj.PropertiesList:
                obj.Placement = reclock.multiply(App.Placement(obj.Placement))
                rotated_names.append(obj.Name)
        for obj in tuple(tool.Group):
            if "Placement" not in obj.PropertiesList:
                continue
            obj.Placement = reclock.multiply(App.Placement(obj.Placement))
            if "ReferencePlacement" in obj.PropertiesList:
                obj.ReferencePlacement = App.Placement(obj.Placement)
            rotated_names.append(obj.Name)

        for name in (
            "ConnectorSourceToWorldAtJ6Zero",
            "GripperSourceToWorldAtJ6Zero",
            "GripperSourceToR6AtZero",
            "CameraSourceToWorldAtJ6Zero",
        ):
            if name in tool.PropertiesList:
                setattr(tool, name, reclock.multiply(App.Placement(getattr(tool, name))))
        if "GripperSourceToR6AtZero" in controller.PropertiesList:
            controller.GripperSourceToR6AtZero = reclock.multiply(
                App.Placement(controller.GripperSourceToR6AtZero)
            )

        camera_proxy_shape, camera_proxy_rows = camera_conservative_proxy(camera)
        existing_proxy = doc.getObject("Gemini_Pro_Camera_Collision_Proxy")
        if existing_proxy is not None:
            doc.removeObject(existing_proxy.Name)
        camera_proxy = tool.newObject("Part::Feature", "Gemini_Pro_Camera_Collision_Proxy")
        camera_proxy.Label = "Gemini Pro 相机保守碰撞代理 / 按STEP两个实体分别包络"
        camera_proxy.Shape = camera_proxy_shape
        camera_proxy.Placement = App.Placement(camera.Placement)
        set_bool(camera_proxy, "CollisionEnabled", True, "Collision")
        set_bool(camera_proxy, "MassEnabled", False, "Physics")
        set_bool(camera_proxy, "Conservative", True, "Collision")
        set_text(camera_proxy, "Construction", "two per-solid source-frame AABB boxes", "Collision")
        set_placement(camera_proxy, "ReferencePlacement", camera_proxy.Placement, "Kinematics")
        try:
            camera_proxy.ViewObject.Visibility = False
            camera_proxy.ViewObject.ShapeColor = (0.96, 0.36, 0.10)
            camera_proxy.ViewObject.Transparency = 72
        except Exception:
            pass
        if "CollisionEnabled" in camera.PropertiesList:
            camera.CollisionEnabled = False
        set_text(camera, "CollisionRepresentation", camera_proxy.Name, "Collision")

        tool.Label = "J6末端：相机上置机械零位 / 连接件与夹爪相对装配保持V15.12"
        set_text(
            tool,
            "J6ZeroPolicy",
            "complete output rigid body +180 deg from V15.12 clock; saved as J6=0 deg; relative interfaces unchanged",
            "V15.13 Mechanical Zero",
        )
        set_angle(tool, "LegacyClockOffset", J6_ZERO_RECLOCK_DEG, "V15.13 Mechanical Zero")
        set_bool(tool, "CameraUpAtJ6Zero", True, "V15.13 Mechanical Zero")

        j6.Angle = 0.0
        set_angle(j6, "V15MechanicalZero", 0.0, "V15 Joint Axis")
        set_bool(j6, "V15CameraUpMechanicalZero", True, "V15.13 Mechanical Zero")
        set_angle(j6, "V15LegacyClockOffset", J6_ZERO_RECLOCK_DEG, "V15.13 Mechanical Zero")
        set_text(
            j6,
            "V15ZeroCalibrationPolicy",
            "camera-up physical output pose equals software J6=0 deg; limits remain -180..+180 deg",
            "V15.13 Mechanical Zero",
        )

        frame_obj = doc.getObject("V15_Frame_link6")
        new_frame = updated_axes["frames_at_mechanical_zero"]["link6"]
        if frame_obj is not None:
            set_vector(frame_obj, "OriginWorldAtZero", App.Vector(*new_frame["origin_world_mm"]), "V15 Engineering Frame")
            set_vector(frame_obj, "XAxisWorldAtZero", App.Vector(*new_frame["x_axis_world_at_zero"]), "V15 Engineering Frame")
            set_vector(frame_obj, "YAxisWorldAtZero", App.Vector(*new_frame["y_axis_world_at_zero"]), "V15 Engineering Frame")
            set_vector(frame_obj, "ZAxisWorldAtZero", App.Vector(*new_frame["z_axis_world_at_zero"]), "V15 Engineering Frame")
            set_text(frame_obj, "ZeroPolicy", "camera-up J6 output clock", "V15.13 Mechanical Zero")

        record = doc.getObject("V15_13_Camera_Up_J6_Zero_Record")
        if record is None:
            record = doc.addObject("App::FeaturePython", "V15_13_Camera_Up_J6_Zero_Record")
        record.Label = "V15.13 相机上置 J6 机械零位与整机复核记录"
        set_text(record, "Revision", REVISION)
        set_text(record, "SourceV1512FCStd", str(SOURCE_FCSTD), "Evidence")
        set_text(record, "SourceV1512SHA256", source_hash_before, "Evidence")
        set_text(record, "PhysicalMotionVideo", str(SOURCE_VIDEO), "Evidence")
        set_text(record, "PhysicalMotionVideoSHA256", video_hash, "Evidence")
        set_angle(record, "LegacyClockOffset", J6_ZERO_RECLOCK_DEG, "Mechanical Zero")
        set_angle(record, "SavedJ6Angle", 0.0, "Mechanical Zero")
        set_angle(record, "J6LimitMin", -180.0, "Limits")
        set_angle(record, "J6LimitMax", 180.0, "Limits")
        set_bool(record, "ConnectorRelativeOutputUnchanged", True, "Assembly")
        set_bool(record, "GripperRelativeConnectorUnchanged", True, "Assembly")
        set_bool(record, "CameraRelativeConnectorUnchanged", True, "Assembly")
        set_text(record, "CameraCollisionProxy", camera_proxy.Name, "Collision")
        set_text(
            record,
            "RuntimeGuard",
            "j123456_camera_physical_collision_guard_v15_13.py",
            "Collision",
        )

        registry = doc.getObject("V15_Joint_Axis_Registry")
        if registry is not None:
            set_text(
                registry,
                "ZeroPolicy",
                "J1-J5 retain measured V15 zero; J6 zero is recalibrated so the complete output rigid body has camera above",
                "V15 Registry",
            )
            set_text(registry, "Revision", REVISION, "V15 Registry")

        doc.recompute()
        after = {
            "output": App.Placement(output_rotor.getGlobalPlacement()),
            "connector": App.Placement(connector.getGlobalPlacement()),
            "camera": App.Placement(camera.getGlobalPlacement()),
            "fixed_frame": App.Placement(fixed_frame.getGlobalPlacement()),
        }
        relative_after = {
            "connector_to_output": after["output"].inverse().multiply(after["connector"]),
            "camera_to_connector": after["connector"].inverse().multiply(after["camera"]),
            "fixed_frame_to_connector": after["connector"].inverse().multiply(after["fixed_frame"]),
        }
        relative_errors = {
            key: placement_error(relative_before[key], relative_after[key])
            for key in relative_before
        }
        for key, error in relative_errors.items():
            if (
                error["translation_mm"] > PLACEMENT_TOLERANCE_MM
                or error["rotation_deg"] > PLACEMENT_TOLERANCE_DEG
            ):
                raise RuntimeError(f"J6 rigid relative interface changed: {key}: {error}")

        camera_bounds = shape_bounds(world_shape(camera))
        if camera_bounds[2] <= origin.z:
            raise RuntimeError(
                f"camera is not wholly above J6 axis at new zero: bounds={camera_bounds}, origin={vector(origin)}"
            )
        if source_camera_world_bounds[5] >= origin.z:
            raise RuntimeError("V15.12 evidence pose was expected to keep camera below J6 axis")
        if not (
            bool(j6.EnableAngleMin)
            and bool(j6.EnableAngleMax)
            and abs(float(j6.AngleMin) + 180.0) <= 1.0e-9
            and abs(float(j6.AngleMax) - 180.0) <= 1.0e-9
            and abs(float(j6.Angle)) <= 1.0e-9
        ):
            raise RuntimeError("J6 zero/range contract was not preserved")

        qa = {
            "status": "PASS",
            "revision": REVISION,
            "generated_at_utc": datetime.now(timezone.utc).isoformat(),
            "sources": {
                "v15_12_fcstd": str(SOURCE_FCSTD),
                "v15_12_sha256_before": source_hash_before,
                "physical_motion_video": str(SOURCE_VIDEO),
                "physical_motion_video_sha256": video_hash,
            },
            "j6_mechanical_zero": {
                "saved_angle_deg": float(j6.Angle),
                "legacy_v15_12_clock_offset_deg": J6_ZERO_RECLOCK_DEG,
                "camera_up_at_zero": True,
                "range_deg": [float(j6.AngleMin), float(j6.AngleMax)],
                "range_rad": [-math.pi, math.pi],
                "measured_origin_world_mm": vector(origin),
                "measured_axis_world": vector(axis),
                "camera_world_bounds_before_mm": source_camera_world_bounds,
                "camera_world_bounds_after_mm": camera_bounds,
                "camera_wholly_above_axis": True,
            },
            "rigid_interface_invariance": {
                "relative_placement_errors": relative_errors,
                "connector_relative_output_changed": False,
                "camera_relative_connector_changed": False,
                "gripper_fixed_frame_relative_connector_changed": False,
                "rotated_physical_object_count": len(rotated_names),
                "rotated_physical_objects": sorted(rotated_names),
            },
            "camera_collision_proxy": {
                "object": camera_proxy.Name,
                "method": "two source-frame AABB boxes, one for each supplied camera STEP solid",
                "conservative": True,
                "source_solid_bounds": camera_proxy_rows,
                "display_shape_collision_disabled": True,
            },
            "axis_contract": {
                "j6_origin_unchanged": True,
                "j6_axis_unchanged": True,
                "link6_zero_frame_xy_reclock_deg": J6_ZERO_RECLOCK_DEG,
                "ros2_mujoco_contract": str(OUTPUT_AXES),
            },
            "pending_separate_audits": [
                "full-chain delta collision sampling",
                "runtime swept guard regression",
                "rigid-link local asset export and independent hash/bounds QA",
            ],
        }

        doc.Label = "机械臂 V15.13 / 相机上置 J6 机械零位 / 整机深度复核"
        doc.recompute()
        doc.saveAs(str(OUTPUT_FCSTD))
        export_objects = [obj for obj in base.tool_export_objects(tool) if obj is not camera_proxy]
        if camera not in export_objects:
            export_objects.append(camera)
        Part.export(export_objects, str(OUTPUT_TOOL_STEP))
        Mesh.export(export_objects, str(OUTPUT_TOOL_STL))
        doc.save()
    finally:
        App.closeDocument(doc.Name)

    source_hash_after = sha256(SOURCE_FCSTD)
    if source_hash_after != source_hash_before:
        raise RuntimeError("immutable V15.12 source changed")
    qa["sources"].update(
        {
            "v15_12_sha256_after": source_hash_after,
            "source_immutable": True,
        }
    )
    qa["outputs"] = {
        "fcstd": str(OUTPUT_FCSTD),
        "fcstd_sha256": sha256(OUTPUT_FCSTD),
        "axis_contract_json": str(OUTPUT_AXES),
        "tool_step": str(OUTPUT_TOOL_STEP),
        "tool_stl": str(OUTPUT_TOOL_STL),
    }
    OUTPUT_AXES.write_text(
        json.dumps(updated_axes, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    OUTPUT_QA.write_text(json.dumps(qa, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    shutil.copy2(SOURCE_VIDEO, EVIDENCE_DIR / SOURCE_VIDEO.name)
    if SOURCE_CONTACT_SHEET.is_file():
        shutil.copy2(SOURCE_CONTACT_SHEET, EVIDENCE_DIR / SOURCE_CONTACT_SHEET.name)
    OUTPUT_README.write_text(
        f"""# V15.13 相机上置 J6 机械零位

主装配：`{OUTPUT_FCSTD.name}`

本版以用户提供的实物活动视频为装配方向证据。V15.12 的 DM-G6220 输出法兰、蓝色连接件、反转夹爪和 Gemini 相机之间的相对装配全部保持不变；完整 J6 输出刚体仅绕已测量的真实 J6 轴换钟向 180°，并把相机位于上方的姿态重新定义为 `J6=0°`。

J6 仍为有限转动关节，范围 `-180° .. +180°`，不是把保存角度停在 180° 边界。ROS2 与 MuJoCo 的 link6 零位坐标系已同步换钟向，详见 `{OUTPUT_AXES.name}`。

相机碰撞采用两个保守包络，分别包住相机 STEP 中的两个实体；显示 STEP 不重复参与碰撞。完整整机碰撞扫描、连续守卫和刚性 Link 导出由同目录后续 QA 文件给出。
""",
        encoding="utf-8",
    )
    print(
        json.dumps(
            {
                "status": "PASS",
                "revision": REVISION,
                "fcstd": str(OUTPUT_FCSTD),
                "fcstd_sha256": qa["outputs"]["fcstd_sha256"],
                "j6_saved_angle_deg": 0.0,
                "j6_range_deg": [-180.0, 180.0],
                "camera_up": True,
                "relative_interfaces_unchanged": True,
            },
            ensure_ascii=False,
            indent=2,
        )
    )


if __name__ == "__main__":
    build()

from __future__ import annotations

"""Build reproducible rigid-link assets from the measured-axis V15 assembly.

This is deliberately a non-reparenting export.  The accepted V15 assembly is
opened read-only in spirit, new local-frame reference objects are added to a
derived FCStd copy, and every mesh is exported in its owning link frame.

The three complete GO-M8010-6 display objects at J1/J2 are not welded into a
single parent link.  They are split by the audited V4 STEP solid-index rule:

    output rotor: solids 24-30, 32, 34
    neutral B6808 display: solid 31 (visual only, no collision/mass)
    stator: all remaining solids

No screw, washer, bearing or bracket becomes an independent URDF link.
"""

import csv
import hashlib
import json
import math
from datetime import datetime, timezone
from pathlib import Path

import FreeCAD as App
import Mesh
import MeshPart
import Part


WORKSPACE = Path(__file__).resolve().parent
SOURCE_ROOT = Path(r"D:\AI_JIXIEBI\模型\机械臂完整装配_真实关节轴_v15")
SOURCE_FCSTD = SOURCE_ROOT / "机械臂完整装配_六轴_真实关节轴_v15.FCStd"
SOURCE_AXES_JSON = SOURCE_ROOT / "V15_真实关节轴定义.json"

OUTPUT_ROOT = SOURCE_ROOT / "rigid_links_v15"
VISUAL_DIR = OUTPUT_ROOT / "visual"
COLLISION_DIR = OUTPUT_ROOT / "collision"
GRIPPER_INTERNAL_DIR = OUTPUT_ROOT / "gripper_internal_zero_reference"
ROS2_DIR = OUTPUT_ROOT / "ros2"
MUJOCO_DIR = OUTPUT_ROOT / "mujoco"
OUTPUT_FCSTD = SOURCE_ROOT / "机械臂完整装配_六轴_刚性Link拆分_v15.FCStd"
MANIFEST_JSON = OUTPUT_ROOT / "rigid_link_manifest.json"
MEMBERSHIP_CSV = OUTPUT_ROOT / "rigid_link_membership.csv"
QA_JSON = OUTPUT_ROOT / "qa_rigid_link_export.json"
README_MD = OUTPUT_ROOT / "README.md"

OUTPUT_SOLID_INDICES = {24, 25, 26, 27, 28, 29, 30, 32, 34}
NEUTRAL_SOLID_INDICES = {31}
OUTPUT_COLLISION_SOLID_INDICES = OUTPUT_SOLID_INDICES - {32}
EXPECTED_GO_M8010_SOLID_COUNT = 34

LINEAR_DEFLECTION_VISUAL_MM = 0.20
LINEAR_DEFLECTION_COLLISION_MM = 0.35
ANGULAR_DEFLECTION_RAD = 0.20
MESH_SCALE_TO_M = [0.001, 0.001, 0.001]


def sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest().upper()


def numbered(prefix: str, count: int) -> list[str]:
    return [f"{prefix}{index:02d}" for index in range(1, count + 1)]


def placement_from_frame(frame: dict) -> App.Placement:
    x = frame["x_axis_world_at_zero"]
    y = frame["y_axis_world_at_zero"]
    z = frame["z_axis_world_at_zero"]
    origin = frame["origin_world_mm"]
    matrix = App.Matrix()
    matrix.A11, matrix.A12, matrix.A13 = x[0], y[0], z[0]
    matrix.A21, matrix.A22, matrix.A23 = x[1], y[1], z[1]
    matrix.A31, matrix.A32, matrix.A33 = x[2], y[2], z[2]
    rotation = App.Rotation(matrix)
    return App.Placement(App.Vector(*origin), rotation)


def rotation_matrix_from_frame(frame: dict) -> list[list[float]]:
    x = frame["x_axis_world_at_zero"]
    y = frame["y_axis_world_at_zero"]
    z = frame["z_axis_world_at_zero"]
    return [
        [x[0], y[0], z[0]],
        [x[1], y[1], z[1]],
        [x[2], y[2], z[2]],
    ]


def matrix_to_rpy(matrix: list[list[float]]) -> list[float]:
    sy = math.sqrt(matrix[0][0] ** 2 + matrix[1][0] ** 2)
    singular = sy < 1.0e-12
    if not singular:
        roll = math.atan2(matrix[2][1], matrix[2][2])
        pitch = math.atan2(-matrix[2][0], sy)
        yaw = math.atan2(matrix[1][0], matrix[0][0])
    else:
        roll = math.atan2(-matrix[1][2], matrix[1][1])
        pitch = math.atan2(-matrix[2][0], sy)
        yaw = 0.0
    return [roll, pitch, yaw]


def relative_placement(frame_world: App.Placement, object_world: App.Placement) -> App.Placement:
    return frame_world.inverse().multiply(object_world)


def local_part_shape(obj, frame_world: App.Placement) -> Part.Shape:
    shape = obj.Shape.copy()
    shape.Placement = relative_placement(frame_world, obj.getGlobalPlacement())
    return shape


def local_mesh(obj, frame_world: App.Placement) -> Mesh.Mesh:
    mesh = Mesh.Mesh(obj.Mesh)
    mesh.Placement = relative_placement(frame_world, obj.getGlobalPlacement())
    return mesh


def motor_component_shape(
    motor_obj,
    component: str,
    frame_world: App.Placement,
) -> Part.Shape:
    source = motor_obj.Shape.copy()
    source.Placement = App.Placement()
    solids = list(source.Solids)
    if len(solids) != EXPECTED_GO_M8010_SOLID_COUNT:
        raise RuntimeError(
            f"{motor_obj.Name}: expected {EXPECTED_GO_M8010_SOLID_COUNT} STEP solids, "
            f"found {len(solids)}"
        )
    if component == "output":
        indices = OUTPUT_SOLID_INDICES
    elif component == "output_collision":
        indices = OUTPUT_COLLISION_SOLID_INDICES
    elif component == "neutral":
        indices = NEUTRAL_SOLID_INDICES
    elif component == "stator":
        indices = set(range(1, len(solids) + 1)) - OUTPUT_SOLID_INDICES - NEUTRAL_SOLID_INDICES
    else:
        raise ValueError(f"unknown motor component: {component}")
    selected = [solid.copy() for index, solid in enumerate(solids, 1) if index in indices]
    if len(selected) != len(indices):
        raise RuntimeError(f"{motor_obj.Name}: incomplete {component} split")
    compound = Part.makeCompound(selected)
    compound.Placement = relative_placement(frame_world, motor_obj.getGlobalPlacement())
    return compound


def part_shape_to_mesh(shape: Part.Shape, linear_deflection_mm: float) -> Mesh.Mesh:
    return MeshPart.meshFromShape(
        Shape=shape,
        LinearDeflection=linear_deflection_mm,
        AngularDeflection=ANGULAR_DEFLECTION_RAD,
        Relative=False,
    )


def build_combined_mesh(doc, specs: list[dict], frame_world: App.Placement, deflection: float) -> Mesh.Mesh:
    result = Mesh.Mesh()
    for spec in specs:
        if spec["kind"] == "object":
            obj = doc.getObject(spec["object"])
            if obj is None:
                raise RuntimeError(f"missing source object: {spec['object']}")
            if hasattr(obj, "Mesh"):
                piece = local_mesh(obj, frame_world)
            elif hasattr(obj, "Shape") and not obj.Shape.isNull():
                piece = part_shape_to_mesh(local_part_shape(obj, frame_world), deflection)
            else:
                raise RuntimeError(f"{obj.Name} has no exportable geometry")
        elif spec["kind"] == "motor_component":
            motor = doc.getObject(spec["object"])
            if motor is None:
                raise RuntimeError(f"missing source motor: {spec['object']}")
            piece = part_shape_to_mesh(
                motor_component_shape(motor, spec["component"], frame_world),
                deflection,
            )
        else:
            raise ValueError(spec["kind"])
        result.addMesh(piece)
    if result.CountFacets == 0:
        raise RuntimeError("empty combined mesh")
    return result


def build_collision_compound(doc, specs: list[dict], frame_world: App.Placement) -> Part.Shape:
    shapes: list[Part.Shape] = []
    for spec in specs:
        if spec["kind"] == "object":
            obj = doc.getObject(spec["object"])
            if obj is None or not hasattr(obj, "Shape") or obj.Shape.isNull():
                raise RuntimeError(f"missing collision BRep: {spec['object']}")
            shapes.append(local_part_shape(obj, frame_world))
        elif spec["kind"] == "motor_component":
            motor = doc.getObject(spec["object"])
            if motor is None:
                raise RuntimeError(f"missing source motor: {spec['object']}")
            shapes.append(motor_component_shape(motor, spec["component"], frame_world))
        else:
            raise ValueError(spec["kind"])
    compound = Part.makeCompound(shapes)
    if compound.isNull():
        raise RuntimeError("empty collision compound")
    return compound


def object_spec(name: str) -> dict:
    return {"kind": "object", "object": name, "token": name}


def motor_spec(source_object: str, actuator: str, component: str) -> dict:
    return {
        "kind": "motor_component",
        "object": source_object,
        "component": component,
        "actuator": actuator,
        "token": f"derived:{actuator}_GO_M8010_{component}",
    }


def specs_from_names(names: list[str]) -> list[dict]:
    return [object_spec(name) for name in names]


BASE_VISUAL = specs_from_names(["Support_Foot", "Upper_Base"]) + [
    motor_spec("Joint_Motor_GO_M8010_6", "J1", "stator")
]
BASE_COLLISION = specs_from_names(["Support_Foot", "Upper_Base", "J1_Fixed_Collision_Proxy"]) + [
    motor_spec("Joint_Motor_GO_M8010_6", "J1", "stator")
]

LINK1_HARDWARE = (
    numbered("M4_Screw_", 6)
    + numbered("J2_Left_M4x30_Screw_", 6)
    + numbered("J2_Left_M4_Washer_", 6)
    + numbered("J2_Right_M4x30_Screw_", 6)
    + numbered("J2_Right_M4_Washer_", 6)
)
LINK1_VISUAL = specs_from_names(
    ["J1_Axial_Compensation_Spacer", "V6_Shoulder_Official_M4"] + LINK1_HARDWARE
) + [
    motor_spec("Joint_Motor_GO_M8010_6", "J1", "output"),
    motor_spec("Joint_Motor_GO_M8010_6", "J1", "neutral"),
    motor_spec("J2_Left_Joint_Motor_GO_M8010_6", "J2A", "stator"),
    motor_spec("J2_Right_Joint_Motor_GO_M8010_6", "J2B", "stator"),
]
LINK1_COLLISION = specs_from_names(
    ["J1_Moving_Collision_Proxy", "J1_Output_Rotor_Proxy"]
)

LINK2_HARDWARE = (
    numbered("UpperArm_M35_Bottom_", 3)
    + numbered("UpperArm_M35_Top_", 3)
    + numbered("J3_Output_M4_Screw_", 6)
)
LINK2_VISUAL = specs_from_names(
    [
        "UpperArm_A_SleeveSide_PrintPart",
        "UpperArm_B_Distal_PrintPart",
        "J3_Output_Rotor_STEP_Display",
        "J3_B6808_Bearing_Neutral_STEP_Display",
    ]
    + LINK2_HARDWARE
) + [
    motor_spec("J2_Left_Joint_Motor_GO_M8010_6", "J2A", "output"),
    motor_spec("J2_Left_Joint_Motor_GO_M8010_6", "J2A", "neutral"),
    motor_spec("J2_Right_Joint_Motor_GO_M8010_6", "J2B", "output"),
    motor_spec("J2_Right_Joint_Motor_GO_M8010_6", "J2B", "neutral"),
]
LINK2_COLLISION = specs_from_names(
    ["UpperArm_Full_Collision_Proxy", "J3_Output_Flange_Proxy"]
) + [
    motor_spec("J2_Left_Joint_Motor_GO_M8010_6", "J2A", "output_collision"),
    motor_spec("J2_Right_Joint_Motor_GO_M8010_6", "J2B", "output_collision"),
]

LINK3_HARDWARE = numbered("Forearm_J3_M35_Screw_", 6) + numbered(
    "Forearm_J4_M35_Screw_", 6
)
LINK3_VISUAL = specs_from_names(
    [
        "J3_Motor_Stator_STEP_Display",
        "Forearm_v3_HighDetail_Display",
        "J4_Motor_Stator_STEP_Display",
    ]
    + LINK3_HARDWARE
)
LINK3_COLLISION = specs_from_names(
    [
        "J3_Motor_Stator_Collision_Proxy",
        "Forearm_Collision_Proxy",
        "J4_Motor_Stator_Collision_Proxy",
    ]
)

LINK4_HARDWARE = numbered("J4_Output_M4_Screw_", 6) + numbered(
    "Wrist_Prelink_J5_M35_Screw_", 6
)
LINK4_VISUAL = specs_from_names(
    [
        "J4_Output_Rotor_STEP_Display",
        "J4_B6808_Bearing_Neutral_STEP_Display",
        "Wrist_Prelink_v1_HighDetail_Display",
        "J5_Motor_Stator_STEP_Display",
    ]
    + LINK4_HARDWARE
)
LINK4_COLLISION = specs_from_names(
    [
        "J4_Output_Flange_Collision_Proxy",
        "Wrist_Prelink_Collision_Proxy",
        "J5_Motor_Stator_Collision_Proxy",
    ]
)

LINK5_HARDWARE = numbered("Adapter_DM_G6220_M4x12_Screw_", 3)
LINK5_VISUAL = specs_from_names(
    [
        "J5_Output_Rotor_STEP_Display",
        "J5_B6808_Bearing_Neutral_STEP_Display",
        "J5_to_Damiao_J6_Adapter_STL_Display",
        "J6_DM_G6220_Stator_STEP_Display",
    ]
    + LINK5_HARDWARE
)
LINK5_COLLISION = specs_from_names(
    [
        "J5_Output_Flange_Collision_Proxy",
        "J5_to_Damiao_J6_Adapter_Collision_Proxy",
        "J6_DM_G6220_Stator_Collision_Proxy",
    ]
)

LINK6_VISUAL = specs_from_names(["J6_DM_G6220_Output_Rotor_STEP_Display"])
LINK6_COLLISION = specs_from_names(["J6_DM_G6220_Output_Rotor_Collision_Proxy"])

GRIPPER_STATIC_HARDWARE = (
    [f"Gripper_Hardware_{index:02d}" for index in (3, 4, 7, 8, 9, 10, 15, 16)]
    + numbered("DM_G6220_Connector_M4x8_Screw_", 6)
    + numbered("Gripper_Clevis_M3x16_Screw_", 2)
)
GRIPPER_VISUAL = specs_from_names(
    [
        "J6_Gripper_Connector_STEP_Display",
        "Gripper_Servo_STEP_Display",
        "Gripper_Fixed_Frame_STEP_Display",
    ]
    + GRIPPER_STATIC_HARDWARE
)
GRIPPER_COLLISION = specs_from_names(
    [
        "J6_Gripper_Connector_Collision_Proxy",
        "Gripper_Fixed_Frame_Collision_Proxy",
        "Gripper_Servo_Collision_Proxy",
    ]
)

GRIPPER_INTERNAL_ROLE_NAMES: dict[str, list[str]] = {
    "left_outer": ["Gripper_Left_Outer_Link"],
    "right_outer": ["Gripper_Right_Outer_Link"],
    "left_drive": ["Gripper_Left_Drive_Link"],
    "right_drive": ["Gripper_Right_Drive_Link"],
    "left_finger": [
        "Gripper_Left_Finger",
        *[f"Gripper_Hardware_{index:02d}" for index in (2, 5, 11, 13, 14)],
    ],
    "right_finger": [
        "Gripper_Right_Finger",
        *[f"Gripper_Hardware_{index:02d}" for index in (1, 6, 12, 17, 18, 19)],
    ],
}

LINK_SPECS: dict[str, dict] = {
    "base_link": {
        "source_group": "J1_Fixed_Rigid",
        "visual": BASE_VISUAL,
        "collision": BASE_COLLISION,
    },
    "link1": {
        "source_group": "J1_Moving_Rigid",
        "visual": LINK1_VISUAL,
        "collision": LINK1_COLLISION,
    },
    "link2": {
        "source_group": "J2_Driven_Reference_Rigid",
        "visual": LINK2_VISUAL,
        "collision": LINK2_COLLISION,
    },
    "link3": {
        "source_group": "J3_Driven_Rigid",
        "visual": LINK3_VISUAL,
        "collision": LINK3_COLLISION,
    },
    "link4": {
        "source_group": "J4_Driven_Rigid",
        "visual": LINK4_VISUAL,
        "collision": LINK4_COLLISION,
    },
    "link5": {
        "source_group": "J5_Driven_Rigid",
        "visual": LINK5_VISUAL,
        "collision": LINK5_COLLISION,
    },
    "link6": {
        "source_group": "J6_Driven_Rigid",
        "visual": LINK6_VISUAL,
        "collision": LINK6_COLLISION,
    },
    "gripper": {
        "source_group": "J6_Gripper_Assembly/static",
        "visual": GRIPPER_VISUAL,
        "collision": GRIPPER_COLLISION,
    },
}


def add_text(obj, name: str, group: str, value: str) -> None:
    obj.addProperty("App::PropertyString", name, group)
    setattr(obj, name, value)


def add_string_list(obj, name: str, group: str, values: list[str]) -> None:
    obj.addProperty("App::PropertyStringList", name, group)
    setattr(obj, name, values)


def add_float_list(obj, name: str, group: str, values: list[float]) -> None:
    obj.addProperty("App::PropertyFloatList", name, group)
    setattr(obj, name, values)


def spec_tokens(specs: list[dict]) -> list[str]:
    return [spec["token"] for spec in specs]


def source_object_names(specs: list[dict]) -> list[str]:
    return [spec["object"] for spec in specs if spec["kind"] == "object"]


def spec_rows(link: str, role: str, specs: list[dict]) -> list[dict]:
    rows = []
    for spec in specs:
        rows.append(
            {
                "owner": link,
                "geometry_role": role,
                "token": spec["token"],
                "source_object": spec["object"],
                "derivation": spec.get("component", "direct_object"),
                "urdf_link_count": 1,
            }
        )
    return rows


def mesh_bounds(mesh: Mesh.Mesh) -> dict:
    box = mesh.BoundBox
    return {
        "min_mm": [box.XMin, box.YMin, box.ZMin],
        "max_mm": [box.XMax, box.YMax, box.ZMax],
        "size_mm": [box.XLength, box.YLength, box.ZLength],
    }


def shape_bounds(shape: Part.Shape) -> dict:
    # BoundBox can reuse a stale triangulation cache and, for imported
    # multi-solid shapes with nested Placement, can effectively count the
    # top-level transform twice.  OCCT's optimal box is geometry-based and is
    # stable across FCStd save/reload.
    box = shape.optimalBoundingBox(False, False)
    return {
        "min_mm": [box.XMin, box.YMin, box.ZMin],
        "max_mm": [box.XMax, box.YMax, box.ZMax],
        "size_mm": [box.XLength, box.YLength, box.ZLength],
    }


def classify_unexported(doc, used_direct_names: set[str], split_sources: set[str]) -> list[dict]:
    group_names = [
        "J1_Fixed_Rigid",
        "J1_Moving_Rigid",
        "J2_Driven_Reference_Rigid",
        "J3_Driven_Rigid",
        "J4_Driven_Rigid",
        "J5_Driven_Rigid",
        "J6_Driven_Rigid",
        "J6_Gripper_Assembly",
    ]
    rows: list[dict] = []
    for group_name in group_names:
        group = doc.getObject(group_name)
        if group is None:
            raise RuntimeError(f"missing source group: {group_name}")
        for obj in group.Group:
            if obj.Name in used_direct_names:
                continue
            if obj.Name in split_sources:
                reason = "complete_motor_source_replaced_by_audited_BRep_solid_split"
            elif obj.TypeId == "App::Part":
                reason = "nested_assembly_classified_separately"
            elif not hasattr(obj, "Shape") and not hasattr(obj, "Mesh"):
                reason = "metadata_controller_or_joint_frame_no_exportable_geometry"
            else:
                reason = "reference_axis_envelope_witness_or_redundant_engineering_proxy"
            rows.append({"source_group": group_name, "object": obj.Name, "reason": reason})
    return rows


def validate_unique_visual_ownership() -> dict:
    direct_owner: dict[str, str] = {}
    duplicate_direct: list[dict] = []
    derived_owner: dict[str, str] = {}
    duplicate_derived: list[dict] = []
    for link, data in LINK_SPECS.items():
        for spec in data["visual"]:
            if spec["kind"] == "object":
                previous = direct_owner.setdefault(spec["object"], link)
                if previous != link:
                    duplicate_direct.append(
                        {"object": spec["object"], "first": previous, "second": link}
                    )
            else:
                previous = derived_owner.setdefault(spec["token"], link)
                if previous != link:
                    duplicate_derived.append(
                        {"component": spec["token"], "first": previous, "second": link}
                    )
    for role, names in GRIPPER_INTERNAL_ROLE_NAMES.items():
        owner = f"gripper_internal:{role}"
        for name in names:
            previous = direct_owner.setdefault(name, owner)
            if previous != owner:
                duplicate_direct.append({"object": name, "first": previous, "second": owner})
    if duplicate_direct or duplicate_derived:
        raise RuntimeError(
            f"duplicate visual rigid ownership: direct={duplicate_direct}, derived={duplicate_derived}"
        )
    return {
        "direct_object_owner": direct_owner,
        "derived_component_owner": derived_owner,
        "duplicate_direct": duplicate_direct,
        "duplicate_derived": duplicate_derived,
    }


def build_joint_tree(axis_data: dict) -> list[dict]:
    joints_by_name = {joint["name"]: joint for joint in axis_data["joints"]}
    source_keys = ["J1", "J2A", "J3", "J4", "J5", "J6"]
    output: list[dict] = []
    for source_key in source_keys:
        source = joints_by_name[source_key]
        name = "J2" if source_key == "J2A" else source_key
        record = {
            "name": name,
            "physical_actuators": ["J2A", "J2B"] if name == "J2" else [name],
            "parent_link": source["parent_link"],
            "child_link": source["child_link"],
            "joint_type_ros2": source["ros2_urdf"]["joint_type"],
            "axis_xyz_in_joint_frame": source["ros2_urdf"]["axis_xyz_in_joint_frame"],
            "origin_xyz_m": source["ros2_urdf"]["origin_xyz_m"],
            "origin_rpy_rad": source["ros2_urdf"]["origin_rpy_rad"],
            "position_limit_rad": source["ros2_urdf"]["position_limit_rad"],
            "mechanical_zero_deg": source["mechanical_zero_deg"],
            "velocity_limit_rad_s": source["ros2_urdf"]["velocity_limit_rad_s"],
            "effort_limit_nm": source["ros2_urdf"]["effort_limit_nm"],
            "mujoco": source["mujoco_mjcf"],
        }
        output.append(record)
    j6 = next(item for item in output if item["name"] == "J6")
    lower = j6["position_limit_rad"]["lower"]
    upper = j6["position_limit_rad"]["upper"]
    if abs(lower + math.pi) > 1.0e-12 or abs(upper - math.pi) > 1.0e-12:
        raise RuntimeError(f"J6 is not ±pi: {lower}, {upper}")
    output.append(
        {
            "name": "J6_to_gripper",
            "physical_actuators": [],
            "parent_link": "link6",
            "child_link": "gripper",
            "joint_type_ros2": "fixed",
            "axis_xyz_in_joint_frame": None,
            "origin_xyz_m": [0.0, 0.0, 0.0],
            "origin_rpy_rad": [0.0, 0.0, 0.0],
            "position_limit_rad": None,
            "mechanical_zero_deg": 0.0,
            "velocity_limit_rad_s": None,
            "effort_limit_nm": None,
            "mujoco": {
                "body_pos_m_in_parent": [0.0, 0.0, 0.0],
                "body_quat_wxyz_in_parent": [1.0, 0.0, 0.0, 0.0],
                "joint_type": "fixed_body_nesting",
            },
        }
    )
    return output


def write_membership_csv(rows: list[dict]) -> None:
    with MEMBERSHIP_CSV.open("w", encoding="utf-8-sig", newline="") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "owner",
                "geometry_role",
                "token",
                "source_object",
                "derivation",
                "urdf_link_count",
            ],
        )
        writer.writeheader()
        writer.writerows(rows)


def write_ros2_notes(joint_tree: list[dict]) -> None:
    payload = {
        "schema": "go-m8010-arm-ros2-rigid-link-staging/1.0",
        "mesh_unit": "millimetre (STL is unitless)",
        "mesh_scale_xyz_to_m": MESH_SCALE_TO_M,
        "links": ["base_link", "link1", "link2", "link3", "link4", "link5", "link6", "gripper"],
        "joints": joint_tree,
        "status": "KINEMATIC_AND_MESH_FRAMES_READY; FINAL URDF BLOCKED ONLY BY VERIFIED EFFORT/VELOCITY/INERTIAL DATA",
        "do_not_guess": ["effort limits", "velocity limits", "mass", "center of mass", "inertia tensor", "tool0/TCP"],
    }
    (ROS2_DIR / "rigid_link_tree_staging.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def write_mujoco_notes(joint_tree: list[dict]) -> None:
    payload = {
        "schema": "go-m8010-arm-mujoco-rigid-link-staging/1.0",
        "mesh_scale_xyz_to_m": MESH_SCALE_TO_M,
        "body_order": ["base_link", "link1", "link2", "link3", "link4", "link5", "link6", "gripper"],
        "joints": joint_tree,
        "J6": {"type": "hinge", "limited": True, "range_rad": [-math.pi, math.pi]},
        "status": "BODY/JOIN/MESH TRANSFORMS READY; FINAL MJCF REQUIRES INERTIALS AND DESIGNED-CONTACT EXCLUSIONS",
        "do_not_guess": ["mass/inertia", "actuator limits", "contact exclusions", "tool0/TCP"],
    }
    (MUJOCO_DIR / "rigid_body_tree_staging.json").write_text(
        json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )


def write_readme() -> None:
    content = """# V15 刚性 Link 拆分

本目录不是“整机一个 STL”。主机械臂按以下刚体链导出：

`world -> base_link -> J1 -> link1 -> J2 -> link2 -> J3 -> link3 -> J4 -> link4 -> J5 -> link5 -> J6 -> link6 -> fixed -> gripper`

- `visual/*.stl` 与 `collision/*.stl` 均在各自 Link 局部坐标系中，STL 数值单位为 mm。
- ROS 2 / MuJoCo 使用时 mesh scale 为 `0.001 0.001 0.001`。
- 每颗螺丝、垫圈和固定支架已经并入随动刚体，没有生成独立 Link。
- J1/J2 的完整 GO-M8010 STEP 已按工程中审计过的 BRep 固体编号拆成定子、输出转子和中性 B6808 显示件，未按外观猜测。
- `gripper.stl` 只包含与连接件、舵机壳体和固定框架相对静止的静态部分。
- 夹爪六个会相对运动的刚体位于 `gripper_internal_zero_reference/`；这些文件共用 `gripper` 零位坐标，仅用于后续建立夹爪内部关节，不得重新焊成一个刚体。
- `gripper_complete_zero_reference.stl` 只是零位总览，不能作为单个 URDF/MuJoCo 刚体。
- 质量、质心、惯量、速度/力矩上限、MuJoCo 设计接触排除和 tool0/TCP 尚无实测数据，未填写猜测值。
"""
    README_MD.write_text(content, encoding="utf-8")


def build() -> None:
    for required in (SOURCE_FCSTD, SOURCE_AXES_JSON):
        if not required.exists():
            raise FileNotFoundError(required)
    source_hash_before = sha256(SOURCE_FCSTD)
    axis_hash = sha256(SOURCE_AXES_JSON)
    axis_data = json.loads(SOURCE_AXES_JSON.read_text(encoding="utf-8"))
    accepted_axis_revisions = {
        "V15.3-measured-axis-joint-limits-and-j6-tool-interface",
        "V15.13-camera-up-j6-mechanical-zero-full-range",
    }
    if axis_data.get("revision") not in accepted_axis_revisions:
        raise RuntimeError(f"unexpected V15 axis revision: {axis_data.get('revision')}")

    for directory in (
        OUTPUT_ROOT,
        VISUAL_DIR,
        COLLISION_DIR,
        GRIPPER_INTERNAL_DIR,
        ROS2_DIR,
        MUJOCO_DIR,
    ):
        directory.mkdir(parents=True, exist_ok=True)

    ownership = validate_unique_visual_ownership()
    frames = axis_data["frames_at_mechanical_zero"]
    frame_placements = {
        name: placement_from_frame(frames[name])
        for name in ("base_link", "link1", "link2", "link3", "link4", "link5", "link6")
    }
    frame_placements["gripper"] = frame_placements["link6"]

    doc = App.openDocument(str(SOURCE_FCSTD))
    if doc is None:
        raise RuntimeError("could not open V15 source FCStd")
    doc.Label = "机械臂 V15.4 刚性 Link 拆分 / ROS2 + MuJoCo 准备"

    existing = doc.getObject("V15_Rigid_Link_Export_Registry")
    if existing is not None:
        raise RuntimeError(
            "source V15 unexpectedly already contains V15_Rigid_Link_Export_Registry; refusing ambiguous rebuild"
        )
    registry = doc.addObject("App::Part", "V15_Rigid_Link_Export_Registry")
    registry.Label = "V15.4 刚性 Link 导出注册表（不改原装配父子关系）"
    add_text(registry, "Topology", "RigidLinks", "world/base_link/J1/link1/J2/link2/J3/link3/J4/link4/J5/link5/J6/link6/fixed/gripper")
    add_text(registry, "RigidRule", "RigidLinks", "No relative motion => one Link; no fastener becomes a Link")
    add_text(registry, "MeshUnits", "Export", "millimetres; use scale 0.001 in ROS2 and MuJoCo")
    add_text(registry, "InertialStatus", "Physics", "PENDING PHYSICAL MASS/COM/INERTIA; NOT GUESSED")
    add_text(registry, "J6Limit", "Kinematics", "revolute [-pi,+pi] rad / [-180,+180] deg")

    link_results: dict[str, dict] = {}
    membership_rows: list[dict] = []
    for link_name, data in LINK_SPECS.items():
        frame_world = frame_placements[link_name]
        visual_mesh = build_combined_mesh(
            doc,
            data["visual"],
            frame_world,
            LINEAR_DEFLECTION_VISUAL_MM,
        )
        collision_shape = build_collision_compound(doc, data["collision"], frame_world)

        visual_path = VISUAL_DIR / f"{link_name}.stl"
        collision_path = COLLISION_DIR / f"{link_name}.stl"
        collision_step_path = COLLISION_DIR / f"{link_name}.step"

        link_group = registry.newObject("App::Part", f"V15_RigidLink_{link_name}")
        link_group.Label = f"刚性 Link：{link_name} / 局部坐标"
        visual_obj = link_group.newObject("Mesh::Feature", f"V15_{link_name}_VisualMesh_Local")
        visual_obj.Label = f"{link_name} visual / local / mm"
        visual_obj.Mesh = visual_mesh
        if visual_obj.ViewObject is not None:
            visual_obj.ViewObject.ShapeColor = (0.72, 0.74, 0.78)
        collision_obj = link_group.newObject("Part::Feature", f"V15_{link_name}_Collision_Local")
        collision_obj.Label = f"{link_name} collision / local / mm"
        collision_obj.Shape = collision_shape
        if collision_obj.ViewObject is not None:
            collision_obj.ViewObject.ShapeColor = (0.92, 0.22, 0.16)
            collision_obj.ViewObject.Transparency = 75
            collision_obj.ViewObject.Visibility = False
        # Force FreeCAD to canonicalize nested TopoShape placements before
        # tessellation and bounds capture.  Without this recompute, a copied
        # compound can retain a stale source-shape bounding-box cache even
        # though its serialized BRep is correct.
        doc.recompute()
        collision_shape = collision_obj.Shape.copy()
        collision_mesh = part_shape_to_mesh(
            collision_shape, LINEAR_DEFLECTION_COLLISION_MM
        )
        visual_mesh.write(str(visual_path))
        collision_mesh.write(str(collision_path))
        Part.export([collision_obj], str(collision_step_path))

        record = link_group.newObject("App::FeaturePython", f"V15_{link_name}_RigidLink_Record")
        record.Label = f"{link_name} 刚体成员与导出契约"
        add_text(record, "LinkName", "RigidLink", link_name)
        add_text(record, "SourceGroup", "RigidLink", data["source_group"])
        add_string_list(record, "VisualMembers", "RigidLink", spec_tokens(data["visual"]))
        add_string_list(record, "CollisionMembers", "RigidLink", spec_tokens(data["collision"]))
        add_float_list(record, "MeshScaleToMeters", "Export", MESH_SCALE_TO_M)
        add_text(record, "VisualMesh", "Export", str(visual_path))
        add_text(record, "CollisionMesh", "Export", str(collision_path))
        add_text(record, "CollisionSTEP", "Export", str(collision_step_path))
        add_text(record, "InertialStatus", "Physics", "PENDING PHYSICAL MEASUREMENT; NOT GUESSED")
        frame_source = frames["link6"] if link_name == "gripper" else frames[link_name]
        add_float_list(record, "FrameOriginWorldMM", "Frame", frame_source["origin_world_mm"])
        add_float_list(record, "FrameXAxisWorld", "Frame", frame_source["x_axis_world_at_zero"])
        add_float_list(record, "FrameYAxisWorld", "Frame", frame_source["y_axis_world_at_zero"])
        add_float_list(record, "FrameZAxisWorld", "Frame", frame_source["z_axis_world_at_zero"])

        membership_rows.extend(spec_rows(link_name, "visual", data["visual"]))
        membership_rows.extend(spec_rows(link_name, "collision", data["collision"]))
        link_results[link_name] = {
            "source_group": data["source_group"],
            "frame_world_at_zero": {
                "origin_mm": frame_source["origin_world_mm"],
                "rotation_matrix_row_major": rotation_matrix_from_frame(frame_source),
                "rpy_rad_urdf_fixed_axis": matrix_to_rpy(rotation_matrix_from_frame(frame_source)),
            },
            "visual_members": spec_tokens(data["visual"]),
            "collision_members": spec_tokens(data["collision"]),
            "visual_mesh": str(visual_path),
            "collision_mesh": str(collision_path),
            "collision_step": str(collision_step_path),
            "mesh_scale_xyz_to_m": MESH_SCALE_TO_M,
            "visual_facets": visual_mesh.CountFacets,
            "collision_facets": collision_mesh.CountFacets,
            "visual_bounds_local": mesh_bounds(visual_mesh),
            "collision_bounds_local": shape_bounds(collision_shape),
            "collision_mesh_bounds_local": mesh_bounds(collision_mesh),
        }

    gripper_internal_results: dict[str, dict] = {}
    gripper_frame = frame_placements["gripper"]
    complete_zero_mesh = Mesh.Mesh()
    complete_zero_mesh.addMesh(
        build_combined_mesh(doc, GRIPPER_VISUAL, gripper_frame, LINEAR_DEFLECTION_VISUAL_MM)
    )
    for role, names in GRIPPER_INTERNAL_ROLE_NAMES.items():
        specs = specs_from_names(names)
        role_mesh = build_combined_mesh(
            doc, specs, gripper_frame, LINEAR_DEFLECTION_VISUAL_MM
        )
        role_collision_shape = build_collision_compound(
            doc, [object_spec(names[0])], gripper_frame
        )
        role_collision_mesh = part_shape_to_mesh(
            role_collision_shape, LINEAR_DEFLECTION_COLLISION_MM
        )
        visual_path = GRIPPER_INTERNAL_DIR / f"{role}_visual.stl"
        collision_path = GRIPPER_INTERNAL_DIR / f"{role}_collision.stl"
        role_mesh.write(str(visual_path))
        role_collision_mesh.write(str(collision_path))
        complete_zero_mesh.addMesh(role_mesh)
        membership_rows.extend(spec_rows(f"gripper_internal:{role}", "visual", specs))
        membership_rows.append(
            {
                "owner": f"gripper_internal:{role}",
                "geometry_role": "collision",
                "token": names[0],
                "source_object": names[0],
                "derivation": "direct_object_without_small_hardware",
                "urdf_link_count": 1,
            }
        )
        gripper_internal_results[role] = {
            "members": names,
            "visual_mesh_zero_reference_in_gripper_frame": str(visual_path),
            "collision_mesh_zero_reference_in_gripper_frame": str(collision_path),
            "visual_facets": role_mesh.CountFacets,
            "collision_facets": role_collision_mesh.CountFacets,
            "status": "RIGID MEMBERSHIP DEFINED; INTERNAL JOINT/PIVOT FRAME EXPORT IS A SEPARATE GRIPPER-KINEMATICS STEP",
        }
    complete_zero_path = GRIPPER_INTERNAL_DIR / "gripper_complete_zero_reference.stl"
    complete_zero_mesh.write(str(complete_zero_path))

    split_sources = {
        "Joint_Motor_GO_M8010_6",
        "J2_Left_Joint_Motor_GO_M8010_6",
        "J2_Right_Joint_Motor_GO_M8010_6",
    }
    used_direct_names = set(ownership["direct_object_owner"])
    for data in LINK_SPECS.values():
        used_direct_names.update(source_object_names(data["collision"]))
    for names in GRIPPER_INTERNAL_ROLE_NAMES.values():
        used_direct_names.update(names)
    unexported = classify_unexported(doc, used_direct_names, split_sources)

    joint_tree = build_joint_tree(axis_data)
    base_frame = frames["base_link"]
    world_to_base = {
        "name": "world_to_base_link",
        "type": "fixed",
        "parent_link": "world",
        "child_link": "base_link",
        "origin_xyz_m": [value * 0.001 for value in base_frame["origin_world_mm"]],
        "origin_rpy_rad": matrix_to_rpy(rotation_matrix_from_frame(base_frame)),
    }

    doc.recompute()
    doc.saveAs(str(OUTPUT_FCSTD))
    App.closeDocument(doc.Name)

    # Re-open the serialized FCStd before the final collision export.  FreeCAD
    # persists the correct BRep, but copied nested compounds may carry stale
    # pre-serialization tessellation/bounding caches.  ROS/MuJoCo assets must
    # come from the canonical reloaded BRep, not from those caches.
    canonical_doc = App.openDocument(str(OUTPUT_FCSTD))
    if canonical_doc is None:
        raise RuntimeError("could not reopen derived FCStd for canonical collision export")
    for link_name in LINK_SPECS:
        result = link_results[link_name]
        collision_obj = canonical_doc.getObject(f"V15_{link_name}_Collision_Local")
        if collision_obj is None or collision_obj.Shape.isNull():
            raise RuntimeError(f"missing canonical collision object for {link_name}")
        canonical_shape = collision_obj.Shape.copy()
        if not canonical_shape.isValid():
            raise RuntimeError(f"invalid canonical collision BRep for {link_name}")
        canonical_mesh = part_shape_to_mesh(
            canonical_shape, LINEAR_DEFLECTION_COLLISION_MM
        )
        collision_path = Path(result["collision_mesh"])
        collision_step_path = Path(result["collision_step"])
        canonical_mesh.write(str(collision_path))
        Part.export([collision_obj], str(collision_step_path))
        result["collision_facets"] = canonical_mesh.CountFacets
        result["collision_bounds_local"] = shape_bounds(canonical_shape)
        result["collision_mesh_bounds_local"] = mesh_bounds(canonical_mesh)
    App.closeDocument(canonical_doc.Name)

    write_membership_csv(membership_rows)
    write_ros2_notes(joint_tree)
    write_mujoco_notes(joint_tree)
    write_readme()

    source_hash_after = sha256(SOURCE_FCSTD)
    if source_hash_after != source_hash_before:
        raise RuntimeError("source V15 FCStd changed during rigid-link export")

    all_asset_paths = []
    for result in link_results.values():
        all_asset_paths.extend(
            [Path(result["visual_mesh"]), Path(result["collision_mesh"]), Path(result["collision_step"])]
        )
    for result in gripper_internal_results.values():
        all_asset_paths.extend(
            [
                Path(result["visual_mesh_zero_reference_in_gripper_frame"]),
                Path(result["collision_mesh_zero_reference_in_gripper_frame"]),
            ]
        )
    all_asset_paths.append(complete_zero_path)
    missing_or_empty = [str(path) for path in all_asset_paths if not path.exists() or path.stat().st_size == 0]
    if missing_or_empty:
        raise RuntimeError(f"missing or empty exports: {missing_or_empty}")

    j6_record = next(item for item in joint_tree if item["name"] == "J6")
    qa = {
        "status": "PASS",
        "source_v15_immutable": source_hash_before == source_hash_after,
        "source_v15_sha256_before": source_hash_before,
        "source_v15_sha256_after": source_hash_after,
        "source_axes_json_sha256": axis_hash,
        "output_fcstd": str(OUTPUT_FCSTD),
        "output_fcstd_sha256": sha256(OUTPUT_FCSTD),
        "link_count_excluding_world": len(LINK_SPECS),
        "main_chain": ["base_link", "link1", "link2", "link3", "link4", "link5", "link6", "gripper"],
        "all_main_link_assets_nonempty": not missing_or_empty,
        "unique_visual_direct_object_ownership": not ownership["duplicate_direct"],
        "unique_visual_derived_component_ownership": not ownership["duplicate_derived"],
        "J6_type": j6_record["joint_type_ros2"],
        "J6_lower_rad": j6_record["position_limit_rad"]["lower"],
        "J6_upper_rad": j6_record["position_limit_rad"]["upper"],
        "J6_plus_minus_180_verified": (
            abs(j6_record["position_limit_rad"]["lower"] + math.pi) <= 1.0e-12
            and abs(j6_record["position_limit_rad"]["upper"] - math.pi) <= 1.0e-12
        ),
        "fasteners_are_members_not_links": True,
        "full_go_m8010_objects_not_exported_unsplit": True,
        "go_m8010_split_rule": {
            "output_visual_solid_indices": sorted(OUTPUT_SOLID_INDICES),
            "neutral_visual_solid_indices": sorted(NEUTRAL_SOLID_INDICES),
            "stator_rule": "all remaining solids",
            "output_collision_indices": sorted(OUTPUT_COLLISION_SOLID_INDICES),
            "basis": "V4 independently QA'd BRep solid split; negative-volume solid 32 excluded from collision only",
        },
        "gripper_static_not_contaminated_by_moving_roles": all(
            name not in source_object_names(GRIPPER_VISUAL)
            for names in GRIPPER_INTERNAL_ROLE_NAMES.values()
            for name in names
        ),
        "gripper_internal_roles": sorted(GRIPPER_INTERNAL_ROLE_NAMES),
        "unexported_classified_count": len(unexported),
    }

    manifest = {
        "schema": "go-m8010-arm-v15-rigid-links/1.0",
        "revision": "V15.4-rigid-link-export-from-V15.3-measured-axes",
        "generated_at_utc": datetime.now(timezone.utc).isoformat(),
        "source": {
            "fcstd": str(SOURCE_FCSTD),
            "fcstd_sha256": source_hash_before,
            "measured_axes_json": str(SOURCE_AXES_JSON),
            "measured_axes_json_sha256": axis_hash,
        },
        "derived_fcstd": {
            "path": str(OUTPUT_FCSTD),
            "sha256": qa["output_fcstd_sha256"],
            "non_reparenting": True,
        },
        "rigid_link_rule": "All parts with zero relative motion are one Link; fasteners are members, never independent links.",
        "main_topology": {
            "world_joint": world_to_base,
            "joints": joint_tree,
            "tree": "world -> base_link -> J1 -> link1 -> J2 -> link2 -> J3 -> link3 -> J4 -> link4 -> J5 -> link5 -> J6 -> link6 -> fixed -> gripper",
        },
        "mesh_contract": {
            "stl_numeric_unit": "mm",
            "ros2_mesh_scale_xyz": MESH_SCALE_TO_M,
            "mujoco_mesh_scale_xyz": MESH_SCALE_TO_M,
            "visual_linear_deflection_mm": LINEAR_DEFLECTION_VISUAL_MM,
            "collision_linear_deflection_mm": LINEAR_DEFLECTION_COLLISION_MM,
        },
        "links": link_results,
        "gripper_internal_rigid_roles": gripper_internal_results,
        "gripper_internal_policy": {
            "gripper_main_link_contains_only_static_members": True,
            "moving_roles_are_not_merged_into_gripper": True,
            "complete_zero_reference_mesh": str(complete_zero_path),
            "complete_zero_reference_is_not_a_valid_single_link": True,
        },
        "go_m8010_split": qa["go_m8010_split_rule"],
        "unexported_or_reference_objects": unexported,
        "physics_status": {
            "mass_center_of_mass_inertia": "PENDING PHYSICAL MEASUREMENT; NOT GUESSED",
            "velocity_effort_limits": "PENDING VERIFIED MOTOR/REDUCER DATA; NOT GUESSED",
            "mujoco_contact_exclusions": "PENDING TRANSFER FROM V14 DESIGNED-CONTACT AUDIT",
            "tool0_tcp": "NOT DEFINED; REQUIRES PHYSICAL TASK-FRAME MEASUREMENT",
        },
        "known_preserved_interface_residuals_mm": {
            "DM_stator_3hole_max_axis_error": 0.0021585046047673,
            "connector_6hole_max_axis_error": 0.1897206562832246,
            "clevis_max_cross_axis_error": 0.4485390535734353,
            "clevis_radial_clearance_remaining": 0.0514609464265647,
        },
        "artifacts": {
            "membership_csv": str(MEMBERSHIP_CSV),
            "qa_json": str(QA_JSON),
            "readme": str(README_MD),
            "ros2_staging": str(ROS2_DIR / "rigid_link_tree_staging.json"),
            "mujoco_staging": str(MUJOCO_DIR / "rigid_body_tree_staging.json"),
        },
    }
    MANIFEST_JSON.write_text(
        json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    QA_JSON.write_text(json.dumps(qa, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(qa, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    build()

from __future__ import annotations

"""Runtime J1..J6 + gripper swept-collision guard for the V14 assembly.

The validated V13 guard remains the owner of the six robot revolute joints.
This module patches that runtime namespace so the connector, fixed gripper
frame, servo body (when ``Gripper_Servo_Controller`` carries a Shape), four
links and two fingers participate in every upstream/J6 collision sample.

The gripper closure is an independent symmetric parallelogram four-bar motion.
``Gripper_Servo_Controller.ClosureAngle`` uses the supplied STEP pose as
``q=0 deg`` and positive q closes the fingers.  API edits, property edits and
manual component drags are continuously swept or rejected and rolled back to
the last exact clear placement.  This module never saves the FCStd document.

Expected V14 hierarchy::

    J6_Driven_Rigid
      `- J6_Gripper_Assembly
           |- J6_Gripper_Connector_Collision_Proxy
           |- Gripper_Fixed_Frame_Collision_Proxy
           |- Gripper_Servo_Collision_Proxy
           |- Gripper_Left_Outer_Link
           |- Gripper_Right_Outer_Link
           |- Gripper_Left_Drive_Link
           |- Gripper_Right_Drive_Link
           |- Gripper_Left_Finger
           |- Gripper_Right_Finger
           `- Gripper_Servo_Controller (ClosureAngle)

Every imported STEP Part::Feature must expose its own ``SourcePlacement`` and
its q=0 final-local ``ReferencePlacement``.  These are deliberately distinct:
the motion equation is ``G * M_raw(q) * SourcePlacement`` where the common
``G`` (GripperSourceToR6AtZero) is recovered from
``ReferencePlacement * inverse(SourcePlacement)`` and cross-checked across the
gripper components.  Treating every STEP child as if it shared one Placement
breaks the pivots and is explicitly rejected.
"""

import itertools
import json
import math
from pathlib import Path

import FreeCAD as App


ROOT = Path(__file__).resolve().parent
WORKSPACE = Path(r"D:/AI_JIXIEBI/go_m8010_arm_gui")
V13_GUARD = ROOT / "j123456_physical_collision_guard_v13.py"
if not V13_GUARD.is_file():
    V13_GUARD = WORKSPACE / "j123456_physical_collision_guard_v13.py"

V14_FCSTD_DESCRIPTION = "包含 J6_Gripper_Assembly 的 V14 机械臂装配 FCStd"
R6_NAME = "J6_Driven_Rigid"
ASSEMBLY_NAME = "J6_Gripper_Assembly"
CONNECTOR_NAME = "J6_Gripper_Connector_Collision_Proxy"
FIXED_FRAME_NAME = "Gripper_Fixed_Frame_Collision_Proxy"
SERVO_PROXY_NAME = "Gripper_Servo_Collision_Proxy"
LEFT_OUTER_NAME = "Gripper_Left_Outer_Link"
RIGHT_OUTER_NAME = "Gripper_Right_Outer_Link"
LEFT_DRIVE_NAME = "Gripper_Left_Drive_Link"
RIGHT_DRIVE_NAME = "Gripper_Right_Drive_Link"
LEFT_FINGER_NAME = "Gripper_Left_Finger"
RIGHT_FINGER_NAME = "Gripper_Right_Finger"
CONTROLLER_NAME = "Gripper_Servo_Controller"
OBSERVER_ATTRIBUTE = "_J123456_GRIPPER_COLLISION_OBSERVER_V14"

GRIPPER_STATIC_BODY_NAMES = (CONNECTOR_NAME, FIXED_FRAME_NAME, SERVO_PROXY_NAME)
GRIPPER_MOVING_BODY_NAMES = (
    LEFT_OUTER_NAME,
    RIGHT_OUTER_NAME,
    LEFT_DRIVE_NAME,
    RIGHT_DRIVE_NAME,
    LEFT_FINGER_NAME,
    RIGHT_FINGER_NAME,
)
GRIPPER_REQUIRED_SHAPE_NAMES = GRIPPER_STATIC_BODY_NAMES + GRIPPER_MOVING_BODY_NAMES
GRIPPER_REQUIRED_NAMES = GRIPPER_REQUIRED_SHAPE_NAMES + (CONTROLLER_NAME,)

EXTERNAL_OBSTACLE_NAMES = (
    "J1_Fixed_Collision_Proxy",
    "J1_Moving_Collision_Proxy",
    "UpperArm_Full_Collision_Proxy",
    "J3_Output_Flange_Proxy",
    "J3_Motor_Stator_Collision_Proxy",
    "Forearm_Collision_Proxy",
    "J4_Motor_Stator_Collision_Proxy",
    "J4_Output_Flange_Collision_Proxy",
    "Wrist_Prelink_Collision_Proxy",
    "J5_Motor_Stator_Collision_Proxy",
    "J5_Output_Flange_Collision_Proxy",
    "J5_to_Damiao_J6_Adapter_Collision_Proxy",
    "J6_DM_G6220_Stator_Collision_Proxy",
    "J6_DM_G6220_Output_Rotor_Collision_Proxy",
)

J6_STATOR_NAME = "J6_DM_G6220_Stator_Collision_Proxy"
J6_ROTOR_NAME = "J6_DM_G6220_Output_Rotor_Collision_Proxy"

CLOSURE_MIN_DEG = -27.5
CLOSURE_MAX_DEG = 56.57
GEOMETRIC_CLOSURE_MIN_DEG = -28.47567453
GEOMETRIC_CLOSURE_MAX_DEG = 69.60209898
MAX_CLOSURE_SWEEP_STEP_DEG = 0.25
MAX_J6_SWEEP_STEP_DEG = 0.25
PENETRATION_VOLUME_TOLERANCE_MM3 = 1.0e-3
CONTACT_DISTANCE_TOLERANCE_MM = 1.0e-7
CLEARANCE_CERTIFICATE_MARGIN_MM = 0.05
ANGLE_EPS_DEG = 1.0e-9
PLACEMENT_TRANSLATION_TOLERANCE_MM = 1.0e-5
PLACEMENT_ROTATION_TOLERANCE_DEG = 1.0e-5

# Source STEP geometry, millimetres.  A pivot's Y coordinate is arbitrary for
# a line parallel to local +Y, so zero is used consistently.
SOURCE_PIVOTS_XZ_MM = {
    LEFT_OUTER_NAME: (86.51826267983768, -21.44495037849011),
    RIGHT_OUTER_NAME: (151.92504537363186, -21.444950378490113),
    LEFT_DRIVE_NAME: (106.72165402673477, -28.79838345999186),
    RIGHT_DRIVE_NAME: (131.72165402673477, -28.79838345999186),
}
LINK_Q_SIGNS = {
    # Source STEP motion fit: q>0 closes.  A +Y rotation decreases the
    # right-hand crank angle in source XZ, while the mirrored left mechanism
    # must use the opposite sign.  These signs keep both distal pin pairs
    # coincident with the translating finger modules through the full stroke.
    LEFT_OUTER_NAME: -1.0,
    RIGHT_OUTER_NAME: +1.0,
    LEFT_DRIVE_NAME: -1.0,
    RIGHT_DRIVE_NAME: +1.0,
}
RIGHT_CRANK_ZERO_DEG = -31.430540502622556
LINK_LENGTH_MM = 35.0

SAME_RIGID_EXCLUDED_PAIRS = {
    frozenset((CONNECTOR_NAME, FIXED_FRAME_NAME)),
    frozenset((CONNECTOR_NAME, SERVO_PROXY_NAME)),
    frozenset((FIXED_FRAME_NAME, SERVO_PROXY_NAME)),
}

DESIGN_JOINT_EXCLUDED_PAIRS = {
    # Ground revolute pins.
    frozenset((FIXED_FRAME_NAME, LEFT_OUTER_NAME)),
    frozenset((FIXED_FRAME_NAME, RIGHT_OUTER_NAME)),
    frozenset((FIXED_FRAME_NAME, LEFT_DRIVE_NAME)),
    frozenset((FIXED_FRAME_NAME, RIGHT_DRIVE_NAME)),
    # Finger-side revolute pins.
    frozenset((LEFT_OUTER_NAME, LEFT_FINGER_NAME)),
    frozenset((LEFT_DRIVE_NAME, LEFT_FINGER_NAME)),
    frozenset((RIGHT_OUTER_NAME, RIGHT_FINGER_NAME)),
    frozenset((RIGHT_DRIVE_NAME, RIGHT_FINGER_NAME)),
    # Intentional 1:1 central gear mesh and servo output interfaces.
    frozenset((LEFT_DRIVE_NAME, RIGHT_DRIVE_NAME)),
    frozenset((SERVO_PROXY_NAME, LEFT_DRIVE_NAME)),
    frozenset((SERVO_PROXY_NAME, RIGHT_DRIVE_NAME)),
}

# These fixed gripper members and the output rotor form one R6 rigid body.
ROTOR_SAME_RIGID_NAMES = set(GRIPPER_STATIC_BODY_NAMES)


if not V13_GUARD.is_file():
    raise RuntimeError("缺少 V13 J1..J6 物理碰撞保护基底: " + str(V13_GUARD))

_v13_source = V13_GUARD.read_text(encoding="utf-8")
_v13 = {
    "__file__": str(V13_GUARD),
    "__name__": "_j123456_physical_collision_guard_v13_base_for_v14",
}
exec(compile(_v13_source, str(V13_GUARD), "exec"), _v13)

_v12 = _v13["_v12"]
_base_install = _v13["install"]
_base_collision_result = _v13["collision_result"]
_base_rotor_collision_rows = _v13["_rotor_collision_rows"]
_base_kinematic_radii_mm = _v12["kinematic_radii_mm"]
_base_j1_radius_mm = _v12["_j1_radius_mm"]
world_shape = _v13["world_shape"]
_pair_result = _v13["_pair_result"]
_placement_error = _v13["_placement_error"]
_world_joint_axis = _v12["_world_joint_axis"]
_joint_frame = _v12["_joint_frame"]
_radial_bound = _v12["_radial_bound"]
_sphere_bound = _v12["_sphere_bound"]
_chain_objects = _v12["_chain_objects"]

_BUSY = False
_OBSTACLE_SHAPE_CACHE = {}
_INTERNAL_COLLISION_CACHE = {}
_PAIR_CLEARANCE_CACHE = {}
_LOCAL_BOUND_CACHE = {}


def ensure_property(obj, kind: str, name: str, group: str = "Runtime Collision Guard V14"):
    if name not in obj.PropertiesList:
        obj.addProperty(kind, name, group)
    return name


def _group_descendant_names(container) -> set[str]:
    found = set()
    stack = list(getattr(container, "Group", ()))
    while stack:
        obj = stack.pop()
        if obj is None or obj.Name in found:
            continue
        found.add(obj.Name)
        stack.extend(list(getattr(obj, "Group", ())))
    return found


def _has_usable_shape(obj) -> bool:
    if obj is None or not hasattr(obj, "Shape"):
        return False
    try:
        return not obj.Shape.isNull() and float(obj.Shape.Volume) > 0.0
    except Exception:
        return False


def active_v14_document():
    doc = _v13["active_v13_document"]()
    required = (ASSEMBLY_NAME, *GRIPPER_REQUIRED_NAMES)
    missing = [name for name in required if doc.getObject(name) is None]
    if missing:
        raise RuntimeError(
            "请先打开 " + V14_FCSTD_DESCRIPTION + "\n缺失对象: " + ", ".join(missing)
        )

    r6 = doc.getObject(R6_NAME)
    assembly = doc.getObject(ASSEMBLY_NAME)
    if assembly not in tuple(getattr(r6, "Group", ())):
        raise RuntimeError(ASSEMBLY_NAME + " 必须直接属于 " + R6_NAME)
    descendants = _group_descendant_names(assembly)
    outside = [name for name in GRIPPER_REQUIRED_NAMES if name not in descendants]
    if outside:
        raise RuntimeError("以下夹爪对象不在 J6_Gripper_Assembly 内: " + ", ".join(outside))

    invalid = []
    for name in GRIPPER_REQUIRED_SHAPE_NAMES:
        obj = doc.getObject(name)
        try:
            valid = _has_usable_shape(obj) and bool(obj.Shape.isValid())
        except Exception:
            valid = False
        if not valid:
            invalid.append(name)
    if invalid:
        raise RuntimeError("V14夹爪碰撞体必须是有效正体积BRep: " + ", ".join(invalid))

    controller = doc.getObject(CONTROLLER_NAME)
    if "ClosureAngle" not in controller.PropertiesList:
        raise RuntimeError(CONTROLLER_NAME + " 缺少 App::PropertyAngle ClosureAngle")
    value = float(controller.ClosureAngle)
    if not math.isfinite(value):
        raise RuntimeError("ClosureAngle 必须是有限角度")
    return doc


def _placement_signature(placement) -> tuple[float, ...]:
    q = placement.Rotation.Q
    return (
        float(placement.Base.x),
        float(placement.Base.y),
        float(placement.Base.z),
        float(q[0]),
        float(q[1]),
        float(q[2]),
        float(q[3]),
    )


def _shape_signature(shape) -> tuple[float, ...]:
    box = shape.BoundBox
    return (
        float(shape.Volume),
        float(box.XMin),
        float(box.YMin),
        float(box.ZMin),
        float(box.XMax),
        float(box.YMax),
        float(box.ZMax),
    )


def _intrinsic_shape_bound(obj):
    shape = obj.Shape.copy()
    shape.Placement = App.Placement()
    signature = _shape_signature(shape)
    key = (obj.Document.Name, obj.Name)
    cached = _LOCAL_BOUND_CACHE.get(key)
    if cached is not None and cached[0] == signature:
        return cached[1], cached[2], signature
    box = shape.BoundBox
    center = App.Vector(
        0.5 * (box.XMin + box.XMax),
        0.5 * (box.YMin + box.YMax),
        0.5 * (box.ZMin + box.ZMax),
    )
    radius = 0.0
    for x in (box.XMin, box.XMax):
        for y in (box.YMin, box.YMax):
            for z in (box.ZMin, box.ZMax):
                radius = max(radius, (App.Vector(x, y, z) - center).Length)
    _LOCAL_BOUND_CACHE[key] = (signature, center, radius)
    return center, radius, signature


def _rigid_motion_bound(obj, previous, current):
    center, radius, _signature = _intrinsic_shape_bound(obj)
    center_travel = (current.multVec(center) - previous.multVec(center)).Length
    _translation, rotation_deg = _placement_error(previous, current)
    rotation_travel = 2.0 * radius * math.sin(math.radians(rotation_deg) * 0.5)
    return center_travel + rotation_travel


def _guard_pair_result(
    doc,
    first_name,
    first_shape,
    second_name,
    second_shape,
    compute_clearance=False,
):
    """AABB + certified swept clearance + exact distance/BRep penetration.

    A positive exact distance at one sample is a lower-bound certificate for
    subsequent samples until the maximum possible point travel of both rigid
    bodies consumes that clearance.  This preserves continuous protection but
    avoids repeating multi-second OCC distance solves every 0.25 degree.
    """

    boxes_overlap, bound_gap = _v13["_aabb_gap"](
        first_shape.BoundBox, second_shape.BoundBox
    )
    if not boxes_overlap:
        return {
            "overlap_mm3": 0.0,
            "clearance_mm": bound_gap if compute_clearance else None,
            "collision": False,
            "broad_phase": "AABB_SEPARATED",
        }

    first_obj = doc.getObject(first_name)
    second_obj = doc.getObject(second_name)
    first_placement = first_obj.getGlobalPlacement()
    second_placement = second_obj.getGlobalPlacement()
    first_center, first_radius, first_signature = _intrinsic_shape_bound(first_obj)
    second_center, second_radius, second_signature = _intrinsic_shape_bound(second_obj)
    del first_center, first_radius, second_center, second_radius
    cache_key = (doc.Name, first_name, second_name)
    cached = _PAIR_CLEARANCE_CACHE.get(cache_key)
    if not compute_clearance and cached is not None:
        same_shapes = cached[0] == first_signature and cached[1] == second_signature
        if same_shapes:
            travel = _rigid_motion_bound(first_obj, cached[2], first_placement)
            travel += _rigid_motion_bound(second_obj, cached[3], second_placement)
            certified = float(cached[4]) - travel
            if certified > CLEARANCE_CERTIFICATE_MARGIN_MM:
                return {
                    "overlap_mm3": 0.0,
                    "clearance_mm": certified,
                    "collision": False,
                    "broad_phase": "CONTINUOUS_CLEARANCE_CERTIFICATE",
                }

    distance = float(first_shape.distToShape(second_shape)[0])
    _PAIR_CLEARANCE_CACHE[cache_key] = (
        first_signature,
        second_signature,
        App.Placement(first_placement),
        App.Placement(second_placement),
        distance,
    )
    if distance > CONTACT_DISTANCE_TOLERANCE_MM:
        return {
            "overlap_mm3": 0.0,
            "clearance_mm": distance if compute_clearance else None,
            "collision": False,
            "broad_phase": "AABB_OVERLAP_EXACT_DISTANCE_CLEAR",
        }

    # The repaired STL-derived connector can make OCC boolean common() stall.
    # At zero exact distance it is safer to stop at contact; its seated stator
    # pair is excluded separately and all other connector contacts are external.
    if CONNECTOR_NAME in (first_name, second_name):
        return {
            "overlap_mm3": None,
            "clearance_mm": 0.0,
            "collision": True,
            "broad_phase": "CONNECTOR_CONTACT_CONSERVATIVE_STOP",
        }

    common = first_shape.common(second_shape)
    overlap = 0.0 if common.isNull() else float(common.Volume)
    collision = overlap > PENETRATION_VOLUME_TOLERANCE_MM3
    return {
        "overlap_mm3": overlap,
        "clearance_mm": 0.0,
        "collision": collision,
        "broad_phase": "AABB_OVERLAP_EXACT_DISTANCE_AND_BREP",
    }


def _cached_obstacle_shape(doc, name: str):
    obj = doc.getObject(name)
    if obj is None or not _has_usable_shape(obj):
        return None
    placement = obj.getGlobalPlacement()
    signature = (_placement_signature(placement), _shape_signature(obj.Shape))
    key = (doc.Name, name)
    cached = _OBSTACLE_SHAPE_CACHE.get(key)
    if cached is None or cached[0] != signature:
        cached = (signature, world_shape(obj))
        _OBSTACLE_SHAPE_CACHE[key] = cached
    return cached[1]


def _collision_body_names(doc) -> tuple[str, ...]:
    return tuple(GRIPPER_REQUIRED_SHAPE_NAMES)


def _internal_signature(doc, names) -> tuple:
    result = []
    for name in names:
        obj = doc.getObject(name)
        result.append((name, _placement_signature(obj.Placement), _shape_signature(obj.Shape)))
    return tuple(result)


def _internal_collision_rows(doc, shapes, compute_clearance=False) -> tuple[dict, bool]:
    names = tuple(shapes)
    signature = _internal_signature(doc, names)
    cached = _INTERNAL_COLLISION_CACHE.get(doc.Name)
    if not compute_clearance and cached is not None and cached[0] == signature:
        return dict(cached[1]), bool(cached[2])

    rows = {}
    collision = False
    for first_name, second_name in itertools.combinations(names, 2):
        pair = frozenset((first_name, second_name))
        if pair in SAME_RIGID_EXCLUDED_PAIRS or pair in DESIGN_JOINT_EXCLUDED_PAIRS:
            continue
        row = _guard_pair_result(
            doc,
            first_name,
            shapes[first_name],
            second_name,
            shapes[second_name],
            compute_clearance,
        )
        key = f"{first_name} vs {second_name}"
        rows[key] = row
        collision = collision or bool(row["collision"])

    if not compute_clearance:
        _INTERNAL_COLLISION_CACHE[doc.Name] = (signature, dict(rows), collision)
    return rows, collision


def _external_pair_excluded(body_name: str, obstacle_name: str) -> bool:
    if obstacle_name == J6_ROTOR_NAME and body_name in ROTOR_SAME_RIGID_NAMES:
        return True
    # Audited by the V14 builder as the designed J6 seat/support relation.  An
    # OCC distance or common() on the repaired STL-derived connector stalls.
    if body_name == CONNECTOR_NAME and obstacle_name == J6_STATOR_NAME:
        return True
    return False


def _gripper_collision_rows(
    doc,
    compute_clearance=False,
    body_filter=None,
    include_internal=True,
) -> tuple[dict, bool]:
    names = _collision_body_names(doc)
    selected = set(names if body_filter is None else body_filter)
    shapes = {name: world_shape(doc.getObject(name)) for name in names}
    rows = {}
    collision = False

    for body_name in names:
        if body_name not in selected:
            continue
        body_shape = shapes[body_name]
        for obstacle_name in EXTERNAL_OBSTACLE_NAMES:
            if _external_pair_excluded(body_name, obstacle_name):
                continue
            obstacle = _cached_obstacle_shape(doc, obstacle_name)
            if obstacle is None:
                continue
            row = _guard_pair_result(
                doc,
                body_name,
                body_shape,
                obstacle_name,
                obstacle,
                compute_clearance,
            )
            key = f"{body_name} vs {obstacle_name}"
            rows[key] = row
            collision = collision or bool(row["collision"])

    if include_internal:
        internal, internal_hit = _internal_collision_rows(
            doc, shapes, compute_clearance=compute_clearance
        )
        rows.update(internal)
        collision = collision or internal_hit
    return rows, collision


def collision_result(doc=None, compute_clearance=True):
    """Evaluate the inherited J1..J6 matrix plus all V14 gripper pairs."""

    doc = doc or active_v14_document()
    result = dict(_base_collision_result(doc, compute_clearance=compute_clearance))
    rows = dict(result["pairs"])
    added, added_hit = _gripper_collision_rows(
        doc, compute_clearance=compute_clearance, include_internal=True
    )
    rows.update(added)
    exclusions = list(result.get("same_rigid_exclusions_v13", ()))
    exclusions.extend(
        [
            "DM-G6220 output rotor / connector / fixed gripper frame / servo housing belong to the fixed R6 tool body",
            "connector clevis and fixed-frame tongue are one bolted gripper base",
            "connector vs J6 stator is the V14 builder-audited designed seat/support pair",
        ]
    )
    design_exclusions = [
        "fixed frame vs each ground-pivot link",
        "each finger vs its two adjacent parallelogram links",
        "left/right drive gear intentional mesh",
        "servo output vs the two central drive members",
    ]
    result.update(
        {
            "collision": bool(result["collision"] or added_hit),
            "scope": "J1..J6 complete chain + DM-G6220 + V14 adaptive gripper",
            "pairs": rows,
            "v14_gripper_added_pair_count": len(added),
            "gripper_closure_default_limits_deg": [CLOSURE_MIN_DEG, CLOSURE_MAX_DEG],
            "gripper_closure_geometric_limits_deg": [
                GEOMETRIC_CLOSURE_MIN_DEG,
                GEOMETRIC_CLOSURE_MAX_DEG,
            ],
            "same_rigid_exclusions_v14": exclusions,
            "design_joint_exclusions_v14": design_exclusions,
            "servo_collision_shape_present": _has_usable_shape(doc.getObject(SERVO_PROXY_NAME)),
        }
    )
    return result


def _j6_endpoint_collision_rows(doc, compute_clearance=False) -> tuple[dict, bool]:
    """V13 J6 fast path extended with the complete attached gripper."""

    rows, collision = _base_rotor_collision_rows(
        doc, compute_clearance=compute_clearance
    )
    rows = dict(rows)
    added, added_hit = _gripper_collision_rows(
        doc, compute_clearance=compute_clearance, include_internal=True
    )
    rows.update(added)
    return rows, bool(collision or added_hit)


def _endpoint_world_shapes(doc):
    names = (J6_STATOR_NAME, J6_ROTOR_NAME, *_collision_body_names(doc))
    return [world_shape(doc.getObject(name)) for name in names if doc.getObject(name) is not None]


def kinematic_radii_mm(doc=None):
    """Extend V12's adaptive J2..J5 travel radii through the gripper tips."""

    doc = doc or active_v14_document()
    result = dict(_base_kinematic_radii_mm(doc))
    moving, r2, r3, r4, _r5, j2, j3, j4, j5 = _chain_objects(doc)
    o2, a2 = _world_joint_axis(moving, j2)
    o3, a3 = _world_joint_axis(r2, j3)
    o4, a4 = _world_joint_axis(r3, j4)
    o5, a5 = _world_joint_axis(r4, j5)
    endpoint = _endpoint_world_shapes(doc)
    if endpoint:
        result["j5_mm"] = max(
            result["j5_mm"], *(_radial_bound(shape, o5, a5) for shape in endpoint)
        )
        result["j4_mm"] = max(
            result["j4_mm"], *(_sphere_bound(shape, o4) for shape in endpoint)
        )
        result["j3_mm"] = max(
            result["j3_mm"], *(_sphere_bound(shape, o3) for shape in endpoint)
        )
        result["j2_mm"] = max(
            result["j2_mm"], *(_sphere_bound(shape, o2) for shape in endpoint)
        )
    return result


def _j1_radius_mm(doc):
    result = float(_base_j1_radius_mm(doc))
    joint = doc.getObject(_v12["J1_JOINT_NAME"])
    origin, _axis, _zero = _joint_frame(joint)
    endpoint = _endpoint_world_shapes(doc)
    if endpoint:
        result = max(result, *(_sphere_bound(shape, origin) for shape in endpoint))
    return result


def _placement_property(owner, names):
    for name in names:
        if owner is None or name not in getattr(owner, "PropertiesList", ()): 
            continue
        value = getattr(owner, name)
        if isinstance(value, App.Placement):
            return App.Placement(value)
        if value is not None and hasattr(value, "Placement"):
            return App.Placement(value.Placement)
    return None


def _source_and_reference_placement(obj):
    source = _placement_property(obj, ("SourcePlacement",))
    reference = _placement_property(
        obj,
        ("ReferencePlacement", "KinematicZeroPlacement", "ClosureZeroPlacement"),
    )
    if source is None or reference is None:
        raise RuntimeError(
            obj.Name + " 必须同时提供 SourcePlacement 与 q=0 ReferencePlacement"
        )
    return source, reference


def _common_source_to_parent_placement(doc):
    explicit_names = (
        "GripperSourceToR6AtZero",
        "SourceToAssemblyPlacement",
        "KinematicFramePlacement",
    )
    explicit = None
    explicit_owner = None
    for owner_name in (ASSEMBLY_NAME, CONTROLLER_NAME, FIXED_FRAME_NAME):
        value = _placement_property(doc.getObject(owner_name), explicit_names)
        if value is not None:
            explicit = value
            explicit_owner = owner_name
            break

    candidates = []
    for name in (FIXED_FRAME_NAME, SERVO_PROXY_NAME, *GRIPPER_MOVING_BODY_NAMES):
        obj = doc.getObject(name)
        if obj is None:
            continue
        source, reference = _source_and_reference_placement(obj)
        candidates.append((name, reference.multiply(source.inverse())))
    if not candidates:
        raise RuntimeError("无法从夹爪对象恢复 GripperSourceToR6AtZero")

    common = App.Placement(explicit if explicit is not None else candidates[0][1])
    failures = []
    for name, candidate in candidates:
        translation, rotation = _placement_error(candidate, common)
        if (
            translation > PLACEMENT_TRANSLATION_TOLERANCE_MM
            or rotation > PLACEMENT_ROTATION_TOLERANCE_DEG
        ):
            failures.append(
                f"{name}: G translation error={translation:.9g} mm, "
                f"rotation error={rotation:.9g} deg"
            )
    if failures:
        raise RuntimeError(
            "各STEP子件的 ReferencePlacement * SourcePlacement^-1 不一致:\n"
            + "\n".join(failures)
        )
    origin = explicit_owner or (candidates[0][0] + " derived G")
    return common, origin


def _rotation_about_source_pivot(pivot_xz, angle_deg):
    center = App.Vector(float(pivot_xz[0]), 0.0, float(pivot_xz[1]))
    axis = App.Vector(0.0, 1.0, 0.0)
    rotation = App.Rotation(axis, float(angle_deg))
    base = center - rotation.multVec(center)
    return App.Placement(base, rotation)


def _right_finger_translation_source(q_deg):
    theta = math.radians(RIGHT_CRANK_ZERO_DEG - float(q_deg))
    theta0 = math.radians(RIGHT_CRANK_ZERO_DEG)
    dx = LINK_LENGTH_MM * (math.cos(theta) - math.cos(theta0))
    dz = LINK_LENGTH_MM * (math.sin(theta) - math.sin(theta0))
    return App.Vector(dx, 0.0, dz)


def _finger_translation_source(name, q_deg):
    source = _right_finger_translation_source(q_deg)
    if name == LEFT_FINGER_NAME:
        source.x = -source.x
    return source


def _bbox_contains(box, point, margin=3.0):
    return (
        box.XMin - margin <= point.x <= box.XMax + margin
        and box.YMin - margin <= point.y <= box.YMax + margin
        and box.ZMin - margin <= point.z <= box.ZMax + margin
    )


def _line_intersects_bbox(origin, direction, box, margin=3.0):
    """Slab test for an infinite hinge axis against a component-local AABB."""

    t_min = -float("inf")
    t_max = float("inf")
    bounds = (
        (box.XMin - margin, box.XMax + margin, origin.x, direction.x),
        (box.YMin - margin, box.YMax + margin, origin.y, direction.y),
        (box.ZMin - margin, box.ZMax + margin, origin.z, direction.z),
    )
    for lower, upper, value, slope in bounds:
        if abs(slope) <= 1.0e-12:
            if value < lower or value > upper:
                return False
            continue
        first = (lower - value) / slope
        second = (upper - value) / slope
        if first > second:
            first, second = second, first
        t_min = max(t_min, first)
        t_max = min(t_max, second)
        if t_min > t_max:
            return False
    return True


class GripperKinematicState:
    def __init__(self, doc):
        self.doc_name = doc.Name
        self.source_to_parent, self.source_frame_origin = _common_source_to_parent_placement(doc)
        self.source_placements = {}
        self.reference_placements = {}
        self.static_placements = {
            ASSEMBLY_NAME: App.Placement(doc.getObject(ASSEMBLY_NAME).Placement),
            CONNECTOR_NAME: App.Placement(doc.getObject(CONNECTOR_NAME).Placement),
            FIXED_FRAME_NAME: App.Placement(doc.getObject(FIXED_FRAME_NAME).Placement),
            SERVO_PROXY_NAME: App.Placement(doc.getObject(SERVO_PROXY_NAME).Placement),
        }
        for name in GRIPPER_MOVING_BODY_NAMES:
            obj = doc.getObject(name)
            source, reference = _source_and_reference_placement(obj)
            self.source_placements[name] = source
            self.reference_placements[name] = reference
        self._validate_pivots(doc)
        self.last_clear_snapshot = _closure_snapshot(doc)

    def _validate_pivots(self, doc):
        errors = []
        for name, pivot_xz in SOURCE_PIVOTS_XZ_MM.items():
            obj = doc.getObject(name)
            pivot_source = App.Vector(float(pivot_xz[0]), 0.0, float(pivot_xz[1]))
            source_inverse = self.source_placements[name].inverse()
            pivot_in_shape = source_inverse.multVec(pivot_source)
            axis_in_shape = source_inverse.Rotation.multVec(App.Vector(0.0, 1.0, 0.0))
            local_shape = obj.Shape.copy()
            # Part::Feature.Shape.Placement mirrors obj.Placement after an
            # FCStd reload.  Reset it before testing component-local geometry.
            local_shape.Placement = App.Placement()
            if not _line_intersects_bbox(
                pivot_in_shape, axis_in_shape, local_shape.BoundBox
            ):
                errors.append(
                    f"{name}: source +Y hinge through ({pivot_in_shape.x:.3f},"
                    f" {pivot_in_shape.y:.3f}, {pivot_in_shape.z:.3f}) misses local BBox"
                )
        if errors:
            raise RuntimeError(
                "夹爪STEP SourcePlacement与源铰点不兼容。\n" + "\n".join(errors)
            )

    def expected_placements(self, q_deg):
        result = {}
        for name in GRIPPER_MOVING_BODY_NAMES:
            source = self.source_placements[name]
            if name in LINK_Q_SIGNS:
                delta = _rotation_about_source_pivot(
                    SOURCE_PIVOTS_XZ_MM[name],
                    LINK_Q_SIGNS[name] * float(q_deg),
                )
                result[name] = self.source_to_parent.multiply(delta).multiply(source)
            else:
                translation = App.Placement(
                    _finger_translation_source(name, q_deg), App.Rotation()
                )
                result[name] = self.source_to_parent.multiply(translation).multiply(source)
        return result


def _closure_snapshot(doc):
    return {
        "placements": {
            name: App.Placement(doc.getObject(name).Placement)
            for name in GRIPPER_MOVING_BODY_NAMES
        },
        "closure_deg": float(doc.getObject(CONTROLLER_NAME).ClosureAngle),
    }


def _restore_closure_snapshot(doc, snapshot):
    for name, placement in snapshot["placements"].items():
        doc.getObject(name).Placement = App.Placement(placement)
    doc.getObject(CONTROLLER_NAME).ClosureAngle = float(snapshot["closure_deg"])


def _verify_closure_snapshot(doc, snapshot):
    failures = []
    for name, expected in snapshot["placements"].items():
        translation, rotation = _placement_error(doc.getObject(name).Placement, expected)
        if (
            translation > PLACEMENT_TRANSLATION_TOLERANCE_MM
            or rotation > PLACEMENT_ROTATION_TOLERANCE_DEG
        ):
            failures.append(
                f"{name}: translation={translation:.9g} mm rotation={rotation:.9g} deg"
            )
    actual_q = float(doc.getObject(CONTROLLER_NAME).ClosureAngle)
    if abs(actual_q - float(snapshot["closure_deg"])) > ANGLE_EPS_DEG:
        failures.append(
            f"ClosureAngle expected {snapshot['closure_deg']:.9g}, got {actual_q:.9g}"
        )
    if failures:
        raise RuntimeError("夹爪碰撞回滚验证失败:\n" + "\n".join(failures))


def _apply_closure_unchecked(doc, state, q_deg):
    placements = state.expected_placements(float(q_deg))
    for name, placement in placements.items():
        doc.getObject(name).Placement = placement
    doc.getObject(CONTROLLER_NAME).ClosureAngle = float(q_deg)


def _runtime_state(doc):
    observer = getattr(App, OBSERVER_ATTRIBUTE, None)
    if observer is not None and getattr(observer, "doc_name", None) == doc.Name:
        return observer.state
    return GripperKinematicState(doc)


def _ensure_controller_runtime_properties(controller):
    ensure_property(controller, "App::PropertyBool", "GuardInstalled")
    ensure_property(controller, "App::PropertyString", "GuardStatus")
    ensure_property(controller, "App::PropertyBool", "GuardCollision")
    ensure_property(controller, "App::PropertyString", "GuardReport")
    ensure_property(controller, "App::PropertyAngle", "LastClearClosureAngle")
    ensure_property(controller, "App::PropertyAngle", "GuardDefaultMinimum")
    ensure_property(controller, "App::PropertyAngle", "GuardDefaultMaximum")


def _set_controller_status(controller, status, collision, report, last_clear=None):
    _ensure_controller_runtime_properties(controller)
    controller.GuardInstalled = True
    controller.GuardStatus = str(status)
    controller.GuardCollision = bool(collision)
    controller.GuardReport = json.dumps(report, ensure_ascii=False, sort_keys=True)
    controller.GuardDefaultMinimum = CLOSURE_MIN_DEG
    controller.GuardDefaultMaximum = CLOSURE_MAX_DEG
    if last_clear is not None:
        controller.LastClearClosureAngle = float(last_clear)


def current_j123456_gripper_state(doc=None):
    doc = doc or active_v14_document()
    result = dict(_v13["current_j123456_angles"](doc))
    q = float(doc.getObject(CONTROLLER_NAME).ClosureAngle)
    result["gripper_q_deg"] = q
    result["closure_deg"] = q
    return result


def _colliding_pairs(rows):
    return [name for name, row in rows.items() if bool(row.get("collision"))]


def set_gripper_closure_angle_v14(angle_deg, sweep_step_deg=None):
    """Continuously sweep the symmetric gripper q coordinate with rollback."""

    global _BUSY
    doc = active_v14_document()
    controller = doc.getObject(CONTROLLER_NAME)
    requested = float(angle_deg)
    if not math.isfinite(requested):
        raise ValueError("夹爪目标角必须是有限数")
    if requested < CLOSURE_MIN_DEG - ANGLE_EPS_DEG or requested > CLOSURE_MAX_DEG + ANGLE_EPS_DEG:
        raise ValueError(
            f"夹爪目标角 {requested:.3f}° 超出默认物理限位 "
            f"{CLOSURE_MIN_DEG:.2f}..{CLOSURE_MAX_DEG:.2f}°"
        )

    state = _runtime_state(doc)
    start = float(controller.ClosureAngle)
    step = MAX_CLOSURE_SWEEP_STEP_DEG
    if sweep_step_deg is not None:
        candidate = abs(float(sweep_step_deg))
        if not math.isfinite(candidate) or candidate <= 0.0:
            raise ValueError("sweep_step_deg 必须是有限正数")
        step = min(candidate, MAX_CLOSURE_SWEEP_STEP_DEG)
    samples = max(1, int(math.ceil(abs(requested - start) / step)))
    snapshot = _closure_snapshot(doc)

    # The inherited J1..J6 observers keep the upstream chain at its last clear
    # pose.  Re-running the complete legacy matrix here would duplicate the
    # multi-minute V13 audit at every servo edit, so closure commands only
    # recheck the newly attached gripper at their start and every q sample.
    start_rows, start_hit = _gripper_collision_rows(
        doc, compute_clearance=False, include_internal=True
    )
    if start_hit:
        pairs = _colliding_pairs(start_rows)
        _set_controller_status(
            controller,
            "REJECTED / START POSE COLLIDING",
            True,
            {"start_deg": start, "pairs": pairs},
            last_clear=snapshot["closure_deg"],
        )
        raise RuntimeError("夹爪命令起点已有碰撞，拒绝运动: " + ", ".join(pairs))

    first_collision = None
    _BUSY = True
    try:
        for index in range(1, samples + 1):
            value = start + (requested - start) * index / samples
            _apply_closure_unchecked(doc, state, value)
            rows, hit = _gripper_collision_rows(
                doc,
                compute_clearance=False,
                body_filter=GRIPPER_MOVING_BODY_NAMES,
                include_internal=True,
            )
            if hit:
                first_collision = {
                    "sample": index,
                    "sample_count": samples,
                    "closure_deg": value,
                    "pairs": _colliding_pairs(rows),
                }
                raise RuntimeError(
                    "夹爪连续扫掠碰撞: " + json.dumps(first_collision, ensure_ascii=False)
                )
    except Exception:
        _restore_closure_snapshot(doc, snapshot)
        doc.recompute()
        _verify_closure_snapshot(doc, snapshot)
        state.last_clear_snapshot = _closure_snapshot(doc)
        _set_controller_status(
            controller,
            "ROLLED BACK / COLLISION OR ERROR",
            first_collision is not None,
            first_collision or {"error": "closure sweep evaluation failed"},
            last_clear=snapshot["closure_deg"],
        )
        raise
    finally:
        _BUSY = False

    # Endpoint verification is intentionally repeated after the swept samples.
    final_rows, final_hit = _gripper_collision_rows(
        doc,
        compute_clearance=False,
        body_filter=GRIPPER_MOVING_BODY_NAMES,
        include_internal=True,
    )
    if final_hit:
        _BUSY = True
        try:
            _restore_closure_snapshot(doc, snapshot)
            doc.recompute()
            _verify_closure_snapshot(doc, snapshot)
        finally:
            _BUSY = False
        pairs = _colliding_pairs(final_rows)
        _set_controller_status(
            controller,
            "ROLLED BACK / ENDPOINT COLLISION",
            True,
            {"target_deg": requested, "pairs": pairs},
            last_clear=snapshot["closure_deg"],
        )
        raise RuntimeError("夹爪终点碰撞并已回滚: " + ", ".join(pairs))

    doc.recompute()
    accepted = _closure_snapshot(doc)
    state.last_clear_snapshot = accepted
    observer = getattr(App, OBSERVER_ATTRIBUTE, None)
    if observer is not None and getattr(observer, "doc_name", None) == doc.Name:
        observer.state.last_clear_snapshot = accepted
    report = {
        "status": "APPLIED_CLEAR",
        "start_deg": start,
        "target_deg": requested,
        "samples": samples,
        "max_step_deg": abs(requested - start) / samples if samples else 0.0,
    }
    _set_controller_status(
        controller,
        "APPLIED CLEAR",
        False,
        report,
        last_clear=requested,
    )
    report["state"] = current_j123456_gripper_state(doc)
    report["collision"] = False
    return report


class V14GripperObserver:
    def __init__(self, doc, state):
        self.doc_name = doc.Name
        self.state = state

    def slotChangedObject(self, obj, prop):
        global _BUSY
        if _BUSY or obj.Document is None or obj.Document.Name != self.doc_name:
            return
        doc = obj.Document
        try:
            if obj.Name == CONTROLLER_NAME and prop == "ClosureAngle":
                proposed = float(obj.ClosureAngle)
                _BUSY = True
                try:
                    _restore_closure_snapshot(doc, self.state.last_clear_snapshot)
                finally:
                    _BUSY = False
                try:
                    set_gripper_closure_angle_v14(proposed)
                except Exception as error:
                    App.Console.PrintError(
                        "V14夹爪ClosureAngle编辑被拒绝并已回滚: " + str(error) + "\n"
                    )
            elif obj.Name in GRIPPER_MOVING_BODY_NAMES and prop == "Placement":
                # A dragged child is not an independent DOF.  Restore the last
                # legal four-bar state instead of accepting a broken linkage.
                _BUSY = True
                try:
                    _restore_closure_snapshot(doc, self.state.last_clear_snapshot)
                    doc.recompute()
                finally:
                    _BUSY = False
                App.Console.PrintError(
                    "V14夹爪运动件不允许独立拖动；已恢复到最后无碰撞ClosureAngle。\n"
                )
            elif obj.Name in self.state.static_placements and prop == "Placement":
                _BUSY = True
                try:
                    obj.Placement = App.Placement(self.state.static_placements[obj.Name])
                    doc.recompute()
                finally:
                    _BUSY = False
                App.Console.PrintError(
                    "V14夹爪固定接口不允许脱离J6独立拖动；已恢复。\n"
                )
        except Exception as error:
            App.Console.PrintError("V14夹爪观察器错误: " + str(error) + "\n")


def _remove_existing_observer():
    previous = getattr(App, OBSERVER_ATTRIBUTE, None)
    if previous is not None:
        try:
            App.removeDocumentObserver(previous)
        except Exception:
            pass
        try:
            delattr(App, OBSERVER_ATTRIBUTE)
        except Exception:
            pass


def _decorate_joint_result(result, doc):
    if isinstance(result, dict):
        result = dict(result)
        result["state_v14"] = current_j123456_gripper_state(doc)
    return result


def _shortest_angle_error(actual_deg, requested_deg):
    return (float(actual_deg) - float(requested_deg) + 180.0) % 360.0 - 180.0


def _upstream_rigid_names():
    return (
        _v12["J1_MOVING_NAME"],
        _v12["J2_RIGID_NAME"],
        _v12["J3_RIGID_NAME"],
        _v12["J4_RIGID_NAME"],
        _v12["J5_RIGID_NAME"],
        _v13["R6_NAME"],
    )


def _upstream_joint_names():
    return (
        _v12["J1_JOINT_NAME"],
        _v12["J2_JOINT_NAME"],
        _v12["J3_JOINT_NAME"],
        _v12["J4_JOINT_NAME"],
        _v12["J5_JOINT_NAME"],
        _v13["J6_NAME"],
    )


def _upstream_snapshot(doc):
    return {
        "placements": {
            name: App.Placement(doc.getObject(name).Placement)
            for name in _upstream_rigid_names()
        },
        "angle_properties": {
            name: float(doc.getObject(name).Angle)
            for name in _upstream_joint_names()
        },
        "actual_angles": dict(_v13["current_j123456_angles"](doc)),
        "closure_snapshot": _closure_snapshot(doc),
    }


def _verify_upstream_snapshot(doc, snapshot):
    placement_errors = {
        name: _placement_error(doc.getObject(name).Placement, expected)
        for name, expected in snapshot["placements"].items()
    }
    angle_property_errors = {
        name: abs(float(doc.getObject(name).Angle) - expected)
        for name, expected in snapshot["angle_properties"].items()
    }
    actual = _v13["current_j123456_angles"](doc)
    actual_angle_errors = {
        key: abs(_shortest_angle_error(actual[key], expected))
        for key, expected in snapshot["actual_angles"].items()
    }
    closure_error = abs(
        float(doc.getObject(CONTROLLER_NAME).ClosureAngle)
        - float(snapshot["closure_snapshot"]["closure_deg"])
    )
    closure_placement_errors = {
        name: _placement_error(doc.getObject(name).Placement, expected)
        for name, expected in snapshot["closure_snapshot"]["placements"].items()
    }
    verified = (
        max(error[0] for error in placement_errors.values())
        <= PLACEMENT_TRANSLATION_TOLERANCE_MM
        and max(error[1] for error in placement_errors.values())
        <= PLACEMENT_ROTATION_TOLERANCE_DEG
        and max(angle_property_errors.values()) <= ANGLE_EPS_DEG
        and max(actual_angle_errors.values())
        <= float(_v12["APPLIED_ANGLE_TOLERANCE_DEG"])
        and closure_error <= ANGLE_EPS_DEG
        and max(error[0] for error in closure_placement_errors.values())
        <= PLACEMENT_TRANSLATION_TOLERANCE_MM
        and max(error[1] for error in closure_placement_errors.values())
        <= PLACEMENT_ROTATION_TOLERANCE_DEG
    )
    return {
        "verified": bool(verified),
        "placement_errors": {
            name: {"translation_mm": error[0], "rotation_deg": error[1]}
            for name, error in placement_errors.items()
        },
        "joint_angle_property_errors_deg": angle_property_errors,
        "actual_angle_errors_deg": actual_angle_errors,
        "closure_angle_error_deg": closure_error,
        "closure_local_placement_errors": {
            name: {"translation_mm": error[0], "rotation_deg": error[1]}
            for name, error in closure_placement_errors.items()
        },
    }


def _restore_upstream_snapshot(doc, snapshot):
    # No recompute: the imported Assembly MbD solver can converge to a nearby
    # numerical pose instead of the commanded revolute value.  The guard owns
    # these six rigid placements and restores them exactly and atomically.
    for name, value in snapshot["angle_properties"].items():
        doc.getObject(name).Angle = float(value)
    for name, placement in snapshot["placements"].items():
        doc.getObject(name).Placement = App.Placement(placement)
    _restore_closure_snapshot(doc, snapshot["closure_snapshot"])
    verification = _verify_upstream_snapshot(doc, snapshot)
    if not verification["verified"]:
        raise RuntimeError(
            "V14显式全链原子回滚验证失败: "
            + json.dumps(verification, ensure_ascii=False)
        )
    return verification


def _apply_j12345_explicit_unchecked(doc, angles):
    moving, r2, r3, r4, r5, j2, j3, j4, j5 = _chain_objects(doc)
    j1 = doc.getObject(_v12["J1_JOINT_NAME"])
    j6 = doc.getObject(_v13["J6_NAME"])
    if j1 is None or j6 is None:
        raise RuntimeError("V14显式全链缺少J1或J6关节")
    joints = (j1, j2, j3, j4, j5)
    keys = ("j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg")
    for joint, key in zip(joints, keys):
        joint.Angle = float(angles[key])
    relative = _v12["relative_placement"]
    moving.Placement = relative(j1, angles["j1_deg"])
    r2.Placement = moving.Placement.multiply(relative(j2, angles["j2_deg"]))
    r3.Placement = r2.Placement.multiply(relative(j3, angles["j3_deg"]))
    r4.Placement = r3.Placement.multiply(relative(j4, angles["j4_deg"]))
    r5.Placement = r4.Placement.multiply(relative(j5, angles["j5_deg"]))
    _v13["_propagate_r6"](doc, float(j6.Angle))


def _verify_j12345_explicit_target(doc, target):
    actual = dict(_v13["current_j123456_angles"](doc))
    angle_errors = {
        key: abs(_shortest_angle_error(actual[key], target[key]))
        for key in ("j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg")
    }
    property_errors = {
        joint_name: abs(
            float(doc.getObject(joint_name).Angle) - float(target[key])
        )
        for joint_name, key in zip(
            _upstream_joint_names()[:5],
            ("j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg"),
        )
    }
    moving, r2, r3, r4, r5, j2, j3, j4, j5 = _chain_objects(doc)
    j1 = doc.getObject(_v12["J1_JOINT_NAME"])
    j6 = doc.getObject(_v13["J6_NAME"])
    relative = _v12["relative_placement"]
    expected = {_v12["J1_MOVING_NAME"]: relative(j1, target["j1_deg"])}
    expected[_v12["J2_RIGID_NAME"]] = expected[
        _v12["J1_MOVING_NAME"]
    ].multiply(relative(j2, target["j2_deg"]))
    expected[_v12["J3_RIGID_NAME"]] = expected[
        _v12["J2_RIGID_NAME"]
    ].multiply(relative(j3, target["j3_deg"]))
    expected[_v12["J4_RIGID_NAME"]] = expected[
        _v12["J3_RIGID_NAME"]
    ].multiply(relative(j4, target["j4_deg"]))
    expected[_v12["J5_RIGID_NAME"]] = expected[
        _v12["J4_RIGID_NAME"]
    ].multiply(relative(j5, target["j5_deg"]))
    expected[_v13["R6_NAME"]] = expected[_v12["J5_RIGID_NAME"]].multiply(
        relative(j6, float(j6.Angle))
    )
    objects = {
        _v12["J1_MOVING_NAME"]: moving,
        _v12["J2_RIGID_NAME"]: r2,
        _v12["J3_RIGID_NAME"]: r3,
        _v12["J4_RIGID_NAME"]: r4,
        _v12["J5_RIGID_NAME"]: r5,
        _v13["R6_NAME"]: doc.getObject(_v13["R6_NAME"]),
    }
    placement_errors = {
        name: _placement_error(objects[name].Placement, placement)
        for name, placement in expected.items()
    }
    passed = (
        max(angle_errors.values())
        <= float(_v12["APPLIED_ANGLE_TOLERANCE_DEG"])
        and max(property_errors.values()) <= ANGLE_EPS_DEG
        and max(error[0] for error in placement_errors.values())
        <= PLACEMENT_TRANSLATION_TOLERANCE_MM
        and max(error[1] for error in placement_errors.values())
        <= PLACEMENT_ROTATION_TOLERANCE_DEG
    )
    result = {
        "pass": bool(passed),
        "requested": dict(target),
        "actual": actual,
        "canonical_angle_errors_deg": angle_errors,
        "joint_angle_property_errors_deg": property_errors,
        "placement_errors": {
            name: {"translation_mm": error[0], "rotation_deg": error[1]}
            for name, error in placement_errors.items()
        },
    }
    if not passed:
        raise RuntimeError(
            "V14显式全链终点与请求不一致: "
            + json.dumps(result, ensure_ascii=False)
        )
    return result


def _refresh_all_runtime_seeds(doc):
    _v12["_refresh_observers"](doc)
    j6_observer = getattr(App, _v13["OBSERVER_ATTRIBUTE"], None)
    if j6_observer is not None and getattr(j6_observer, "doc_name", None) == doc.Name:
        j6_observer.last_clear_r6 = App.Placement(
            doc.getObject(_v13["R6_NAME"]).Placement
        )
        j6_observer.last_clear_j6 = float(doc.getObject(_v13["J6_NAME"]).Angle)
    gripper_observer = getattr(App, OBSERVER_ATTRIBUTE, None)
    if gripper_observer is not None and getattr(gripper_observer, "doc_name", None) == doc.Name:
        gripper_observer.state.last_clear_snapshot = _closure_snapshot(doc)


def _active_collision_pair_names(result):
    return [
        name for name, row in result.get("pairs", {}).items() if row.get("collision")
    ]


def _upstream_runtime_pair_specs():
    """Certified V7/V12/V13/V14 pairs with their owning rigid groups.

    Group 0 is fixed; groups 1..6 are the rigid bodies downstream of J1..J6.
    A Jk command can only change pairs separated by the k boundary.  Pairs on
    the same side preserve their exact previously accepted relative placement
    and must not be needlessly re-solved at every 0.25-degree sub-step.
    """

    fixed = _v12["FIXED_PROXY_NAME"]
    shoulder = _v12["SHOULDER_PROXY_NAME"]
    arm_motion = "UpperArm_Motion_Collision_Proxy"
    upperarm = _v12["UPPERARM_FULL_PROXY_NAME"]
    j3_output = _v12["J3_OUTPUT_PROXY_NAME"]
    j3_stator = _v12["J3_STATOR_PROXY_NAME"]
    forearm = _v12["FOREARM_PROXY_NAME"]
    j4_stator = _v12["J4_STATOR_PROXY_NAME"]
    j4_output = _v12["J4_OUTPUT_PROXY_NAME"]
    wrist = _v12["WRIST_PROXY_NAME"]
    j5_stator = _v12["J5_STATOR_PROXY_NAME"]
    j5_output = _v12["J5_OUTPUT_PROXY_NAME"]
    adapter = _v12["ADAPTER_PROXY_NAME"]
    j6_stator = _v13["STATOR_NAME"]
    j6_rotor = _v13["ROTOR_NAME"]

    groups = {
        fixed: 0,
        shoulder: 1,
        arm_motion: 2,
        upperarm: 2,
        j3_output: 2,
        j3_stator: 3,
        forearm: 3,
        j4_stator: 3,
        j4_output: 4,
        wrist: 4,
        j5_stator: 4,
        j5_output: 5,
        adapter: 5,
        j6_stator: 5,
        j6_rotor: 6,
    }
    groups.update({name: 6 for name in GRIPPER_REQUIRED_SHAPE_NAMES})

    specs = []

    def add(key, first, second):
        if first == second:
            return
        specs.append((str(key), first, second, groups[first], groups[second]))

    # Validated V7 J1..J4 matrix.
    add("moving_vs_base", arm_motion, fixed)
    add("moving_vs_shoulder", arm_motion, shoulder)
    for prefix, body in (
        ("j3_stator", j3_stator),
        ("forearm", forearm),
        ("j4_stator", j4_stator),
    ):
        for suffix, obstacle in (
            ("base", fixed),
            ("shoulder", shoulder),
            ("upperarm", upperarm),
            ("j3_output", j3_output),
        ):
            add(f"{prefix}_vs_{suffix}", body, obstacle)
    for suffix, obstacle in (
        ("base", fixed),
        ("shoulder", shoulder),
        ("upperarm", upperarm),
        ("j3_output", j3_output),
        ("j3_stator", j3_stator),
        ("forearm", forearm),
        ("j4_stator", j4_stator),
    ):
        add(f"j4_output_vs_{suffix}", j4_output, obstacle)
        add(f"wrist_vs_{suffix}", wrist, obstacle)

    # Validated V12 J5 extension matrix.
    for suffix, obstacle in (
        ("base", fixed),
        ("shoulder", shoulder),
        ("upperarm", upperarm),
        ("j3_output", j3_output),
        ("j3_stator", j3_stator),
        ("forearm", forearm),
        ("j4_stator", j4_stator),
    ):
        add(f"j5_stator_vs_{suffix}", j5_stator, obstacle)
    for prefix, body in (("j5_output", j5_output), ("adapter", adapter)):
        for suffix, obstacle in (
            ("base", fixed),
            ("shoulder", shoulder),
            ("upperarm", upperarm),
            ("j3_output", j3_output),
            ("j3_stator", j3_stator),
            ("forearm", forearm),
            ("j4_stator", j4_stator),
            ("j4_output", j4_output),
            ("wrist", wrist),
            ("j5_stator", j5_stator),
        ):
            add(f"{prefix}_vs_{suffix}", body, obstacle)

    # Validated V13 DM-G6220 extension matrix.
    for obstacle in _v13["STATOR_OBSTACLES"]:
        add(f"{j6_stator} vs {obstacle}", j6_stator, obstacle)
    for obstacle in _v13["ROTOR_OBSTACLES"]:
        add(f"{j6_rotor} vs {obstacle}", j6_rotor, obstacle)

    # V14 gripper bodies against every certified upstream obstacle. Internal
    # gripper pairs and R6 same-rigid pairs are invariant during J1..J5 motion.
    for body in GRIPPER_REQUIRED_SHAPE_NAMES:
        for obstacle in EXTERNAL_OBSTACLE_NAMES:
            if _external_pair_excluded(body, obstacle):
                continue
            add(f"{body} vs {obstacle}", body, obstacle)

    unique = {}
    for row in specs:
        unique[(row[1], row[2])] = row
    return tuple(unique.values())


def _relative_group_motion_changes(first_group, second_group, changed_joints):
    lower, upper = sorted((int(first_group), int(second_group)))
    return any(lower < int(joint) <= upper for joint in changed_joints)


def _relevant_upstream_pair_specs(doc, changed_joints):
    rows = []
    missing = []
    for row in _upstream_runtime_pair_specs():
        key, first, second, first_group, second_group = row
        if not _relative_group_motion_changes(
            first_group, second_group, changed_joints
        ):
            continue
        if doc.getObject(first) is None or doc.getObject(second) is None:
            missing.append(first if doc.getObject(first) is None else second)
            continue
        rows.append(row)
    if missing:
        raise RuntimeError(
            "V14相关碰撞矩阵缺少代理: " + ", ".join(sorted(set(missing)))
        )
    return tuple(rows)


def _relevant_upstream_collision_result(
    doc, pair_specs, changed_joints, compute_clearance=False
):
    shape_cache = {}

    def shape(name):
        if name not in shape_cache:
            shape_cache[name] = world_shape(doc.getObject(name))
        return shape_cache[name]

    rows = {}
    collision = False
    for key, first, second, first_group, second_group in pair_specs:
        row = _guard_pair_result(
            doc,
            first,
            shape(first),
            second,
            shape(second),
            compute_clearance=compute_clearance,
        )
        row = dict(row)
        row["rigid_groups"] = [int(first_group), int(second_group)]
        rows[key] = row
        collision = collision or bool(row["collision"])
    return {
        "collision": bool(collision),
        "scope": (
            "certified motion-relevant V7/V12/V13/V14 pairs only; invariant "
            "pairs inherit the last accepted exact relative placement"
        ),
        "changed_joints": [f"J{index}" for index in sorted(changed_joints)],
        "pair_count": len(rows),
        "pairs": rows,
        "compute_clearance": bool(compute_clearance),
    }


def _run_explicit_j12345_command(doc, start, target, plan, command_label):
    global _BUSY
    snapshot = _upstream_snapshot(doc)
    changed_joints = {
        index
        for index, key in enumerate(
            ("j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg"), start=1
        )
        if abs(_shortest_angle_error(target[key], start[key])) > ANGLE_EPS_DEG
    }
    pair_specs = _relevant_upstream_pair_specs(doc, changed_joints)
    plan = dict(plan)
    plan["changed_joints"] = [f"J{index}" for index in sorted(changed_joints)]
    plan["runtime_relevant_pair_count"] = len(pair_specs)
    plan["invariant_pair_policy"] = (
        "retain prior accepted exact relative placement; do not re-evaluate"
    )
    # The current state is an observer-maintained last-clear seed. Evaluate
    # only the pairs whose relative transform can change, with exact clearance
    # at the seed so subsequent samples can use conservative motion bounds.
    start_result = _relevant_upstream_collision_result(
        doc, pair_specs, changed_joints, compute_clearance=True
    )
    if start_result["collision"]:
        pairs = _active_collision_pair_names(start_result)
        raise RuntimeError(
            command_label + "起始位姿已有碰撞，拒绝运动: " + ", ".join(pairs)
        )

    inherited_observers, inherited_states = _v12["_observer_states"]()
    previous_v13_busy = bool(_v13.get("_BUSY", False))
    previous_v14_busy = bool(_BUSY)
    _v13["_BUSY"] = True
    _BUSY = True
    last_result = start_result
    first_collision = None
    rollback = None
    try:
        count = int(plan["sample_count"])
        for index in range(1, count + 1):
            fraction = index / count
            sample = {
                key: float(start[key])
                + (float(target[key]) - float(start[key])) * fraction
                for key in ("j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg")
            }
            _apply_j12345_explicit_unchecked(doc, sample)
            last_result = _relevant_upstream_collision_result(
                doc, pair_specs, changed_joints, compute_clearance=False
            )
            if last_result["collision"]:
                first_collision = {
                    "sample_index": index,
                    "sample_count": count,
                    "angles": sample,
                    "active_pairs": _active_collision_pair_names(last_result),
                }
                raise RuntimeError(
                    command_label
                    + "路径碰撞: "
                    + json.dumps(first_collision, ensure_ascii=False)
                )

        endpoint_collision = _relevant_upstream_collision_result(
            doc, pair_specs, changed_joints, compute_clearance=True
        )
        if endpoint_collision["collision"]:
            first_collision = {
                "sample_index": int(plan["sample_count"]),
                "sample_count": int(plan["sample_count"]),
                "angles": dict(target),
                "active_pairs": _active_collision_pair_names(endpoint_collision),
                "endpoint_recheck": True,
            }
            raise RuntimeError(
                command_label
                + "终点精确清除量复核碰撞: "
                + ", ".join(first_collision["active_pairs"])
            )
        last_result = endpoint_collision
        verification = _verify_j12345_explicit_target(doc, target)
        _refresh_all_runtime_seeds(doc)
    except Exception:
        rollback = _restore_upstream_snapshot(doc, snapshot)
        _refresh_all_runtime_seeds(doc)
        try:
            _v12["_set_status"](
                doc,
                "V14_EXPLICIT_ERROR_AND_REVERTED",
                snapshot["actual_angles"],
                last_result,
                plan,
                first_collision=first_collision,
                rollback=rollback,
            )
        except Exception:
            pass
        raise
    finally:
        _BUSY = previous_v14_busy
        _v13["_BUSY"] = previous_v13_busy
        _v12["_restore_observer_states"](inherited_observers, inherited_states)

    applied = dict(_v13["current_j123456_angles"](doc))
    _v12["_set_status"](
        doc,
        "V14_EXPLICIT_CLEAR",
        applied,
        last_result,
        plan,
    )
    report = {
        "status": "APPLIED_CLEAR",
        "command": command_label,
        "kinematic_mode": "explicit J1->R2->R3->R4->R5->R6 / no MbD recompute",
        "document_recompute_performed": False,
        "start": dict(start),
        "target": dict(target),
        "applied": applied,
        "endpoint_verification": verification,
        "sweep": dict(plan),
        "collision": False,
        "state_v14": current_j123456_gripper_state(doc),
    }
    controller = doc.getObject(CONTROLLER_NAME)
    _set_controller_status(
        controller,
        "UPSTREAM JOINT APPLIED CLEAR / EXPLICIT HIERARCHY",
        False,
        {
            "command": command_label,
            "target": dict(target),
            "samples": int(plan["sample_count"]),
            "max_step_deg": float(plan["actual_max_joint_step_deg"]),
        },
        last_clear=float(controller.ClosureAngle),
    )
    return report


def set_j1_angle_v14(angle_deg, sweep_step_deg=None):
    doc = active_v14_document()
    requested = float(angle_deg)
    joint = doc.getObject(_v12["J1_JOINT_NAME"])
    _v12["_validate_joint_limit"](joint, requested, "J1")
    start = dict(_v13["current_j123456_angles"](doc))
    target = {
        key: float(start[key])
        for key in ("j1_deg", "j2_deg", "j3_deg", "j4_deg", "j5_deg")
    }
    target["j1_deg"] = requested
    plan = dict(
        _v12["_j1_sweep_plan"](
            doc, float(start["j1_deg"]), requested, sweep_step_deg
        )
    )
    return _run_explicit_j12345_command(doc, start, target, plan, "J1")


def set_j2_angle_v14(angle_deg, sweep_step_deg=None):
    return set_j2_j3_j4_j5_angles_v14(
        j2_deg=angle_deg, sweep_step_deg=sweep_step_deg
    )


def set_j3_angle_v14(angle_deg, sweep_step_deg=None):
    return set_j2_j3_j4_j5_angles_v14(
        j3_deg=angle_deg, sweep_step_deg=sweep_step_deg
    )


def set_j4_angle_v14(angle_deg, sweep_step_deg=None):
    return set_j2_j3_j4_j5_angles_v14(
        j4_deg=angle_deg, sweep_step_deg=sweep_step_deg
    )


def set_j5_angle_v14(angle_deg, sweep_step_deg=None):
    return set_j2_j3_j4_j5_angles_v14(
        j5_deg=angle_deg, sweep_step_deg=sweep_step_deg
    )


def _verify_j6_explicit_target(doc, requested):
    r5 = doc.getObject(_v13["R5_NAME"])
    r6 = doc.getObject(_v13["R6_NAME"])
    joint = doc.getObject(_v13["J6_NAME"])
    actual = float(_v13["current_j6_angle"](doc))
    angle_error = abs(_shortest_angle_error(actual, requested))
    property_error = abs(float(joint.Angle) - float(requested))
    expected = r5.Placement.multiply(
        _v12["relative_placement"](joint, float(requested))
    )
    placement_error = _placement_error(r6.Placement, expected)
    passed = (
        angle_error <= float(_v12["APPLIED_ANGLE_TOLERANCE_DEG"])
        and property_error <= ANGLE_EPS_DEG
        and placement_error[0] <= PLACEMENT_TRANSLATION_TOLERANCE_MM
        and placement_error[1] <= PLACEMENT_ROTATION_TOLERANCE_DEG
    )
    result = {
        "pass": bool(passed),
        "requested_deg": float(requested),
        "actual_deg": actual,
        "canonical_angle_error_deg": angle_error,
        "joint_angle_property_error_deg": property_error,
        "placement_error": {
            "translation_mm": placement_error[0],
            "rotation_deg": placement_error[1],
        },
    }
    if not passed:
        raise RuntimeError(
            "V14显式J6终点与请求不一致: "
            + json.dumps(result, ensure_ascii=False)
        )
    return result


def _verified_j6_last_clear_seed(doc):
    observer = getattr(App, _v13["OBSERVER_ATTRIBUTE"], None)
    if observer is None or getattr(observer, "doc_name", None) != doc.Name:
        raise RuntimeError("V14 J6命令缺少已安装的last-clear观察器")
    r6 = doc.getObject(_v13["R6_NAME"])
    joint = doc.getObject(_v13["J6_NAME"])
    placement_error = _placement_error(r6.Placement, observer.last_clear_r6)
    property_error = abs(float(joint.Angle) - float(observer.last_clear_j6))
    actual_error = abs(
        _shortest_angle_error(
            _v13["current_j6_angle"](doc), observer.last_clear_j6
        )
    )
    passed = (
        placement_error[0] <= PLACEMENT_TRANSLATION_TOLERANCE_MM
        and placement_error[1] <= PLACEMENT_ROTATION_TOLERANCE_DEG
        and property_error <= ANGLE_EPS_DEG
        and actual_error <= float(_v12["APPLIED_ANGLE_TOLERANCE_DEG"])
    )
    result = {
        "pass": bool(passed),
        "source": "V13 observer last_clear_r6 / last_clear_j6",
        "placement_error": {
            "translation_mm": placement_error[0],
            "rotation_deg": placement_error[1],
        },
        "joint_angle_property_error_deg": property_error,
        "actual_angle_error_deg": actual_error,
    }
    if not passed:
        raise RuntimeError(
            "J6当前状态不等于观察器last-clear种子，拒绝运动: "
            + json.dumps(result, ensure_ascii=False)
        )
    return result


def set_j6_angle_v14(angle_deg, sweep_step_deg=None):
    global _BUSY
    doc = active_v14_document()
    requested = float(angle_deg)
    if not math.isfinite(requested):
        raise ValueError("J6目标角必须是有限数")
    if (
        requested < float(_v13["J6_MIN_DEG"]) - ANGLE_EPS_DEG
        or requested > float(_v13["J6_MAX_DEG"]) + ANGLE_EPS_DEG
    ):
        raise ValueError(
            f"J6目标角 {requested:.3f}° 超出 "
            f"{float(_v13['J6_MIN_DEG']):.1f}..{float(_v13['J6_MAX_DEG']):.1f}°"
        )
    start = float(_v13["current_j6_angle"](doc))
    step = MAX_J6_SWEEP_STEP_DEG
    if sweep_step_deg is not None:
        candidate = abs(float(sweep_step_deg))
        if not math.isfinite(candidate) or candidate <= 0.0:
            raise ValueError("J6 sweep_step_deg 必须是有限正数")
        step = min(step, candidate)
    samples = max(1, int(math.ceil(abs(requested - start) / step)))
    snapshot = _upstream_snapshot(doc)
    changed_joints = {6}
    pair_specs = _relevant_upstream_pair_specs(doc, changed_joints)
    start_seed = _verified_j6_last_clear_seed(doc)
    start_result = {
        "collision": False,
        "scope": "verified observer last-clear start seed",
        "pairs": {},
        "prevalidated_start_seed": True,
        "verification": start_seed,
    }

    inherited_observers, inherited_states = _v12["_observer_states"]()
    previous_v13_busy = bool(_v13.get("_BUSY", False))
    previous_v14_busy = bool(_BUSY)
    _v13["_BUSY"] = True
    _BUSY = True
    last_result = start_result
    first_collision = None
    try:
        for index in range(1, samples + 1):
            value = start + (requested - start) * index / samples
            _v13["_propagate_r6"](doc, value)
            last_result = _relevant_upstream_collision_result(
                doc, pair_specs, changed_joints, compute_clearance=False
            )
            if last_result["collision"]:
                first_collision = {
                    "sample_index": index,
                    "sample_count": samples,
                    "angle_deg": value,
                    "active_pairs": _active_collision_pair_names(last_result),
                }
                raise RuntimeError(
                    "J6路径碰撞: "
                    + json.dumps(first_collision, ensure_ascii=False)
                )
        endpoint_result = _relevant_upstream_collision_result(
            doc, pair_specs, changed_joints, compute_clearance=True
        )
        if endpoint_result["collision"]:
            first_collision = {
                "sample_index": samples,
                "sample_count": samples,
                "angle_deg": requested,
                "active_pairs": _active_collision_pair_names(endpoint_result),
                "endpoint_recheck": True,
            }
            raise RuntimeError(
                "J6终点精确相关对碰撞: "
                + ", ".join(first_collision["active_pairs"])
            )
        last_result = endpoint_result
        verification = _verify_j6_explicit_target(doc, requested)
        _refresh_all_runtime_seeds(doc)
    except Exception:
        rollback = _restore_upstream_snapshot(doc, snapshot)
        _refresh_all_runtime_seeds(doc)
        status = doc.getObject(_v13["STATUS_NAME"])
        if status is not None:
            status.RuntimeGuard = "V14 EXPLICIT J6 ERROR AND REVERTED"
            status.RuntimeAngles = json.dumps(
                _v13["current_j123456_angles"](doc), ensure_ascii=False
            )
            status.RuntimeCollision = bool(first_collision is not None)
        raise
    finally:
        _BUSY = previous_v14_busy
        _v13["_BUSY"] = previous_v13_busy
        _v12["_restore_observer_states"](inherited_observers, inherited_states)

    state_now = current_j123456_gripper_state(doc)
    status = doc.getObject(_v13["STATUS_NAME"])
    if status is not None:
        status.RuntimeGuard = "V14 EXPLICIT J6 CLEAR / NO MbD RECOMPUTE"
        status.RuntimeAngles = json.dumps(state_now, ensure_ascii=False)
        status.RuntimeCollision = False
    controller = doc.getObject(CONTROLLER_NAME)
    _set_controller_status(
        controller,
        "J6 APPLIED CLEAR / EXPLICIT HIERARCHY",
        False,
        {
            "target_deg": requested,
            "samples": samples,
            "max_step_deg": abs(requested - start) / samples,
            "runtime_relevant_pair_count": len(pair_specs),
        },
        last_clear=float(controller.ClosureAngle),
    )
    return {
        "status": "APPLIED_CLEAR",
        "kinematic_mode": "explicit R5->R6 / no MbD recompute",
        "document_recompute_performed": False,
        "start_deg": start,
        "target_deg": requested,
        "samples": samples,
        "actual_max_step_deg": abs(requested - start) / samples,
        "runtime_relevant_pair_count": len(pair_specs),
        "endpoint_verification": verification,
        "start_seed_verification": start_seed,
        "collision": False,
        "state_v14": state_now,
    }


def set_j2_j3_j4_j5_angles_v14(
    j2_deg=None,
    j3_deg=None,
    j4_deg=None,
    j5_deg=None,
    sweep_step_deg=None,
):
    doc = active_v14_document()
    moving, r2, r3, r4, r5, j2, j3, j4, j5 = _chain_objects(doc)
    start = dict(_v13["current_j123456_angles"](doc))
    requested = (j2_deg, j3_deg, j4_deg, j5_deg)
    serial_keys = ("j2_deg", "j3_deg", "j4_deg", "j5_deg")
    serial_target = {
        key: float(start[key]) if value is None else float(value)
        for key, value in zip(serial_keys, requested)
    }
    _v12["_validate_target_limits"]((j2, j3, j4, j5), serial_target)
    target = {
        "j1_deg": float(start["j1_deg"]),
        **serial_target,
    }
    plan = dict(
        _v12["_sweep_plan"](
            doc,
            {key: float(start[key]) for key in serial_keys},
            serial_target,
            sweep_step_deg,
        )
    )
    return _run_explicit_j12345_command(
        doc, start, target, plan, "J2/J3/J4/J5"
    )


def _patch_inherited_guard():
    # All inherited observers/setters perform global lookups in their exec
    # namespaces, so replacing these entries extends both API and GUI edits.
    _v13["collision_result"] = collision_result
    _v13["_rotor_collision_rows"] = _j6_endpoint_collision_rows
    _v13["MAX_J6_SWEEP_STEP_DEG"] = min(
        float(_v13["MAX_J6_SWEEP_STEP_DEG"]), MAX_J6_SWEEP_STEP_DEG
    )
    _v12["kinematic_radii_mm"] = kinematic_radii_mm
    _v12["_j1_radius_mm"] = _j1_radius_mm
    _v12["MAX_JOINT_STEP_DEG"] = min(float(_v12["MAX_JOINT_STEP_DEG"]), 0.25)
    # V12 observers resolve these names dynamically in their exec namespace.
    # Patching them routes both GUI Angle edits and public V14 APIs through the
    # explicit no-MbD hierarchy instead of the drifting recompute path.
    _v12["set_j1_angle_v11"] = set_j1_angle_v14
    _v12["set_j2_j3_j4_j5_angles_v11"] = set_j2_j3_j4_j5_angles_v14
    _v13["set_j6_angle_v13"] = set_j6_angle_v14


_patch_inherited_guard()


def install():
    global _BUSY
    doc = active_v14_document()
    _BUSY = True
    try:
        base_result = _base_install()
    finally:
        _BUSY = False

    state = GripperKinematicState(doc)
    # The inherited observers are already present after V13 installation.
    # FreeCAD Assembly may emit tiny R2..R6 placement changes while the saved
    # gripper pose is normalized/recomputed.  Suppress those callbacks here;
    # otherwise each callback recursively launches another complete endpoint
    # collision pass during install.
    inherited_observers, inherited_busy_states = _v12["_observer_states"]()
    inherited_v13_busy = bool(_v13.get("_BUSY", False))
    _v13["_BUSY"] = True
    _BUSY = True
    try:
        # Normalize the saved pose to the controller's q through the exact
        # four-bar equations before accepting it as the rollback seed.
        _apply_closure_unchecked(
            doc, state, float(doc.getObject(CONTROLLER_NAME).ClosureAngle)
        )
        doc.recompute()
    finally:
        _BUSY = False
        _v13["_BUSY"] = inherited_v13_busy
        _v12["_restore_observer_states"](
            inherited_observers, inherited_busy_states
        )

    # The recomputed normalized pose is now the inherited observers' legal
    # seed; refresh their snapshots without launching a motion command.
    _v12["_refresh_observers"](doc)
    inherited_j6_observer = getattr(App, _v13["OBSERVER_ATTRIBUTE"], None)
    if inherited_j6_observer is not None:
        inherited_j6_observer.last_clear_r6 = App.Placement(
            doc.getObject(_v13["R6_NAME"]).Placement
        )
        inherited_j6_observer.last_clear_j6 = float(
            doc.getObject(_v13["J6_NAME"]).Angle
        )

    # V13's saved J1..J6 pose is prevalidated by its builder and inherited
    # guard installation.  Validate only the V14-added pairs here; the public
    # collision_result_v14 API still performs the complete explicit matrix.
    initial_rows, initial_hit = _gripper_collision_rows(
        doc, compute_clearance=False, include_internal=True
    )
    initial = {
        "collision": bool(initial_hit),
        "scope": "prevalidated V13 saved pose + V14 gripper added pairs",
        "pairs": initial_rows,
        "v14_gripper_added_pair_count": len(initial_rows),
        "prevalidated_upstream_zero_pose": True,
    }
    controller = doc.getObject(CONTROLLER_NAME)
    if initial["collision"]:
        pairs = _colliding_pairs(initial["pairs"])
        _set_controller_status(
            controller,
            "INSTALL REJECTED / SAVED POSE COLLIDING",
            True,
            {"pairs": pairs},
            last_clear=float(controller.ClosureAngle),
        )
        raise RuntimeError("V14保存姿态存在碰撞，守卫未接受: " + ", ".join(pairs))

    state.last_clear_snapshot = _closure_snapshot(doc)
    _remove_existing_observer()
    observer = V14GripperObserver(doc, state)
    App.addDocumentObserver(observer)
    setattr(App, OBSERVER_ATTRIBUTE, observer)

    App.set_j1_angle_v14 = set_j1_angle_v14
    App.set_j2_angle_v14 = set_j2_angle_v14
    App.set_j3_angle_v14 = set_j3_angle_v14
    App.set_j4_angle_v14 = set_j4_angle_v14
    App.set_j5_angle_v14 = set_j5_angle_v14
    App.set_j6_angle_v14 = set_j6_angle_v14
    App.set_j2_j3_j4_j5_angles_v14 = set_j2_j3_j4_j5_angles_v14
    App.set_gripper_closure_angle_v14 = set_gripper_closure_angle_v14
    App.set_gripper_closure_v14 = set_gripper_closure_angle_v14
    App.current_j123456_gripper_state_v14 = current_j123456_gripper_state
    App.collision_result_v14 = collision_result

    state_now = current_j123456_gripper_state(doc)
    _set_controller_status(
        controller,
        "INSTALLED / J1..J6 + GRIPPER SWEPT BREP ROLLBACK / NO AUTOSAVE",
        False,
        {
            "state": state_now,
            "source_frame": state.source_frame_origin,
            "servo_collision_shape_present": _has_usable_shape(doc.getObject(SERVO_PROXY_NAME)),
            "default_limits_deg": [CLOSURE_MIN_DEG, CLOSURE_MAX_DEG],
        },
        last_clear=float(controller.ClosureAngle),
    )

    status = doc.getObject(_v13["STATUS_NAME"])
    if status is not None:
        status.RuntimeGuard = (
            "INSTALLED V14 / J1..J6 + gripper closure swept BRep rollback / no FCStd autosave"
        )
        status.RuntimeAngles = json.dumps(state_now, ensure_ascii=False)
        status.RuntimeCollision = False

    if not _has_usable_shape(doc.getObject(SERVO_PROXY_NAME)):
        App.Console.PrintWarning(
            "V14警告: Gripper_Servo_Collision_Proxy没有有效Shape；舵机外壳未参与碰撞。\n"
        )
    App.Console.PrintMessage(
        "V14 J1..J6与夹爪物理碰撞保护已启动；ClosureAngle连续扫掠、非法拖动回滚，且不会自动保存FCStd。\n"
    )
    return {
        "base_v13": base_result,
        "observer": observer,
        "collision": initial,
        "state": state_now,
        "api": {
            "joints": [f"App.set_j{i}_angle_v14" for i in range(1, 7)],
            "closure": "App.set_gripper_closure_angle_v14",
            "state": "App.current_j123456_gripper_state_v14",
            "collision": "App.collision_result_v14",
        },
    }


if __name__ == "__main__":
    install()

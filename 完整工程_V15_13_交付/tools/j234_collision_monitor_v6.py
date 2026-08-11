from __future__ import annotations

"""Runtime swept-collision guard for the V6 J2/J3/J4 serial chain.

This module is deliberately not embedded in or saved back to the FCStd file.
The startup macro loads it into the current FreeCAD session.  The rigid-body
boundary is:

    J2_Driven_Reference_Rigid
        upper arm + J3 output rotor
    ---------------- J3 revolute ----------------
    J3_Driven_Rigid
        J3 stator + forearm + J4 stator + twelve M3.5 fasteners
    ---------------- J4 revolute ----------------
    J4_Driven_Rigid
        J4 output rotor/flange (future wrist belongs here)

Only different-rigid-body collision pairs are evaluated.  The J3 and J4
stator/output pairs remain enabled: their zero-pose proxies touch but have no
penetration volume, so normal joint contact is accepted without suppressing
the complete adjacent-body pair.
"""

import json
import math

import FreeCAD as App


ASSEMBLY_NAME = "RobotBaseShoulderAssembly"
J1_MOVING_NAME = "J1_Moving_Rigid"
J2_RIGID_NAME = "J2_Driven_Reference_Rigid"
J3_RIGID_NAME = "J3_Driven_Rigid"
J4_RIGID_NAME = "J4_Driven_Rigid"
J2_JOINT_NAME = "J2_Revolute"
J3_JOINT_NAME = "J3_Revolute"
J4_JOINT_NAME = "J4_Revolute"

FIXED_PROXY_NAME = "J1_Fixed_Collision_Proxy"
SHOULDER_PROXY_NAME = "J1_Moving_Collision_Proxy"
UPPERARM_MOTION_PROXY_NAME = "UpperArm_Motion_Collision_Proxy"
UPPERARM_FULL_PROXY_NAME = "UpperArm_Full_Collision_Proxy"
J3_OUTPUT_PROXY_NAME = "J3_Output_Flange_Proxy"
J3_STATOR_PROXY_NAME = "J3_Motor_Stator_Collision_Proxy"
FOREARM_PROXY_NAME = "Forearm_Collision_Proxy"
J4_STATOR_PROXY_NAME = "J4_Motor_Stator_Collision_Proxy"
J4_OUTPUT_PROXY_NAME = "J4_Output_Flange_Collision_Proxy"

STATUS_NAME = "V6_J4_Physical_Test_Status"
PENETRATION_VOLUME_TOLERANCE_MM3 = 1.0e-3
MAX_JOINT_STEP_DEG = 0.5
MAX_DESCENDANT_TRAVEL_MM = 1.0
MAX_SWEEP_SAMPLES = 20000
ANGLE_EPS_DEG = 1.0e-9
PLACEMENT_TRANSLATION_TOLERANCE_MM = 1.0e-4
PLACEMENT_ROTATION_TOLERANCE_DEG = 1.0e-4


def active_v6_document():
    doc = App.ActiveDocument
    required = (
        J2_RIGID_NAME,
        J3_RIGID_NAME,
        J4_RIGID_NAME,
        J2_JOINT_NAME,
        J3_JOINT_NAME,
        J4_JOINT_NAME,
        J4_STATOR_PROXY_NAME,
        J4_OUTPUT_PROXY_NAME,
    )
    if doc is None or any(doc.getObject(name) is None for name in required):
        raise RuntimeError("请先打开 机械臂底座肩部大臂小臂末端电机装配_v6.FCStd")
    return doc


def world_shape(obj):
    """Return one correctly placed world-space Part shape."""

    shape = obj.Shape.copy()
    if not shape.isNull():
        shape.Placement = obj.getGlobalPlacement()
    return shape


def _overlap_and_distance(first, second, compute_clearance=True):
    if not first.BoundBox.intersect(second.BoundBox):
        distance = float(first.distToShape(second)[0]) if compute_clearance else None
        return 0.0, distance

    common = first.common((second,), 1.0e-7)
    overlap = 0.0 if common.isNull() else abs(float(common.Volume))
    if overlap > PENETRATION_VOLUME_TOLERANCE_MM3:
        distance = 0.0
    else:
        distance = float(first.distToShape(second)[0]) if compute_clearance else None
    return overlap, distance


def _joint_frame(joint):
    origin = App.Vector(joint.Offset1.Base)
    axis = joint.Offset1.Rotation.multVec(App.Vector(0, 0, 1))
    zero = joint.Offset1.Rotation.multVec(App.Vector(1, 0, 0))
    axis.normalize()
    zero.normalize()
    return origin, axis, zero


def relative_placement(joint, angle_deg: float) -> App.Placement:
    origin, axis, _ = _joint_frame(joint)
    rotation = App.Rotation(axis, float(angle_deg))
    return App.Placement(origin - rotation.multVec(origin), rotation)


def _relative_angle(parent, child, joint) -> float:
    relative = parent.Placement.inverse().multiply(child.Placement)
    _, axis, zero = _joint_frame(joint)
    actual = relative.Rotation.multVec(zero)
    sine = axis.dot(zero.cross(actual))
    cosine = zero.dot(actual)
    return math.degrees(math.atan2(sine, cosine))


def _chain_objects(doc):
    moving = doc.getObject(J1_MOVING_NAME)
    r2 = doc.getObject(J2_RIGID_NAME)
    r3 = doc.getObject(J3_RIGID_NAME)
    r4 = doc.getObject(J4_RIGID_NAME)
    j2 = doc.getObject(J2_JOINT_NAME)
    j3 = doc.getObject(J3_JOINT_NAME)
    j4 = doc.getObject(J4_JOINT_NAME)
    objects = (moving, r2, r3, r4, j2, j3, j4)
    if any(obj is None for obj in objects):
        raise RuntimeError("V6 J2/J3/J4 刚体或关节缺失")
    return objects


def current_angles(doc=None):
    doc = doc or active_v6_document()
    moving, r2, r3, r4, j2, j3, j4 = _chain_objects(doc)
    return {
        "j2_deg": _relative_angle(moving, r2, j2),
        "j3_deg": _relative_angle(r2, r3, j3),
        "j4_deg": _relative_angle(r3, r4, j4),
    }


def apply_angles_unchecked(doc, j2_deg: float, j3_deg: float, j4_deg: float) -> None:
    moving, r2, r3, r4, j2, j3, j4 = _chain_objects(doc)
    r2.Placement = moving.Placement.multiply(relative_placement(j2, j2_deg))
    r3.Placement = r2.Placement.multiply(relative_placement(j3, j3_deg))
    r4.Placement = r3.Placement.multiply(relative_placement(j4, j4_deg))
    doc.recompute()


def collision_result(doc=None, compute_clearance=True):
    """Evaluate all external rigid-body pairs in the V6 J2/J3/J4 scope."""

    doc = doc or active_v6_document()
    names = (
        FIXED_PROXY_NAME,
        SHOULDER_PROXY_NAME,
        UPPERARM_MOTION_PROXY_NAME,
        UPPERARM_FULL_PROXY_NAME,
        J3_OUTPUT_PROXY_NAME,
        J3_STATOR_PROXY_NAME,
        FOREARM_PROXY_NAME,
        J4_STATOR_PROXY_NAME,
        J4_OUTPUT_PROXY_NAME,
    )
    objects = {name: doc.getObject(name) for name in names}
    missing = [name for name, obj in objects.items() if obj is None]
    if missing:
        raise RuntimeError("V6 碰撞代理缺失: " + ", ".join(missing))

    fixed = world_shape(objects[FIXED_PROXY_NAME])
    shoulder = world_shape(objects[SHOULDER_PROXY_NAME])
    arm_motion = world_shape(objects[UPPERARM_MOTION_PROXY_NAME])
    arm_full = world_shape(objects[UPPERARM_FULL_PROXY_NAME])
    j3_output = world_shape(objects[J3_OUTPUT_PROXY_NAME])
    j3_stator = world_shape(objects[J3_STATOR_PROXY_NAME])
    forearm = world_shape(objects[FOREARM_PROXY_NAME])
    j4_stator = world_shape(objects[J4_STATOR_PROXY_NAME])
    j4_output = world_shape(objects[J4_OUTPUT_PROXY_NAME])

    pairs = {
        # R2 against the environment/R1.
        "moving_vs_base": (arm_motion, fixed),
        "moving_vs_shoulder": (arm_motion, shoulder),
        # R3 components against environment/R1/R2.
        "j3_stator_vs_base": (j3_stator, fixed),
        "j3_stator_vs_shoulder": (j3_stator, shoulder),
        "j3_stator_vs_upperarm": (j3_stator, arm_full),
        "j3_stator_vs_j3_output": (j3_stator, j3_output),
        "forearm_vs_base": (forearm, fixed),
        "forearm_vs_shoulder": (forearm, shoulder),
        "forearm_vs_upperarm": (forearm, arm_full),
        "forearm_vs_j3_output": (forearm, j3_output),
        "j4_stator_vs_base": (j4_stator, fixed),
        "j4_stator_vs_shoulder": (j4_stator, shoulder),
        "j4_stator_vs_upperarm": (j4_stator, arm_full),
        "j4_stator_vs_j3_output": (j4_stator, j3_output),
        # R4 against environment/R1/R2/R3.  The last pair is deliberately
        # active: zero-pose tangency is accepted, penetration is not.
        "j4_output_vs_base": (j4_output, fixed),
        "j4_output_vs_shoulder": (j4_output, shoulder),
        "j4_output_vs_upperarm": (j4_output, arm_full),
        "j4_output_vs_j3_output": (j4_output, j3_output),
        "j4_output_vs_j3_stator": (j4_output, j3_stator),
        "j4_output_vs_forearm": (j4_output, forearm),
        "j4_output_vs_j4_stator": (j4_output, j4_stator),
    }

    rows = {}
    collision = False
    for name, (first, second) in pairs.items():
        overlap, distance = _overlap_and_distance(first, second, compute_clearance)
        colliding = overlap > PENETRATION_VOLUME_TOLERANCE_MM3
        rows[name] = {
            "overlap_mm3": overlap,
            "clearance_mm": 0.0 if colliding else distance,
            "collision": colliding,
        }
        collision = collision or colliding

    return {
        "collision": collision,
        "scope": "J1 environment + J2/J3/J4 serial chain; wrist geometry absent",
        "penetration_volume_tolerance_mm3": PENETRATION_VOLUME_TOLERANCE_MM3,
        "same_rigid_exclusions": [
            "J3 stator vs forearm",
            "J3 stator/forearm vs J4 stator",
            "J3 rigid components vs their twelve M3.5 fasteners",
            "J4 output rotor vs neutral bearing display/future output fasteners",
        ],
        "adjacent_joint_pairs_checked": [
            "j3_stator_vs_j3_output",
            "j4_output_vs_j4_stator",
        ],
        "adjacent_contact_policy": (
            "surface tangency/common volume zero is allowed; no whole-pair suppression"
        ),
        "pairs": rows,
    }


def _world_joint_axis(parent, joint):
    parent_world = parent.getGlobalPlacement()
    origin, axis, _ = _joint_frame(joint)
    world_origin = parent_world.multVec(origin)
    world_axis = parent_world.Rotation.multVec(axis)
    world_axis.normalize()
    return world_origin, world_axis


def _boundbox_corners(bound_box):
    for x in (bound_box.XMin, bound_box.XMax):
        for y in (bound_box.YMin, bound_box.YMax):
            for z in (bound_box.ZMin, bound_box.ZMax):
                yield App.Vector(x, y, z)


def _radial_bound(shape, origin, axis):
    maximum = 0.0
    for point in _boundbox_corners(shape.BoundBox):
        delta = point - origin
        radial = delta - axis * delta.dot(axis)
        maximum = max(maximum, radial.Length)
    return maximum


def _axis_offset(first_origin, first_axis, second_origin):
    delta = second_origin - first_origin
    return (delta - first_axis * delta.dot(first_axis)).Length


def kinematic_radii_mm(doc=None):
    """Return conservative downstream radial bounds for swept sampling."""

    doc = doc or active_v6_document()
    moving, r2, r3, _r4, j2, j3, j4 = _chain_objects(doc)
    o2, a2 = _world_joint_axis(moving, j2)
    o3, a3 = _world_joint_axis(r2, j3)
    o4, a4 = _world_joint_axis(r3, j4)

    shape = lambda name: world_shape(doc.getObject(name))
    upperarm = shape(UPPERARM_FULL_PROXY_NAME)
    j3_output = shape(J3_OUTPUT_PROXY_NAME)
    j3_stator = shape(J3_STATOR_PROXY_NAME)
    forearm = shape(FOREARM_PROXY_NAME)
    j4_stator = shape(J4_STATOR_PROXY_NAME)
    j4_output = shape(J4_OUTPUT_PROXY_NAME)

    r4 = max(
        _radial_bound(j4_output, o4, a4),
        1.0,
    )
    j4_envelope = max(
        _radial_bound(j4_stator, o4, a4),
        _radial_bound(j4_output, o4, a4),
    )
    r3 = max(
        _radial_bound(j3_stator, o3, a3),
        _radial_bound(forearm, o3, a3),
        _axis_offset(o3, a3, o4) + j4_envelope,
    )
    r2 = max(
        _radial_bound(upperarm, o2, a2),
        _radial_bound(j3_output, o2, a2),
        _axis_offset(o2, a2, o3) + r3,
    )
    return {"j2_mm": r2, "j3_mm": r3, "j4_mm": r4}


def _sweep_plan(doc, start, target, requested_step_deg):
    radii = kinematic_radii_mm(doc)
    deltas = {
        key: target[key] - start[key]
        for key in ("j2_deg", "j3_deg", "j4_deg")
    }
    maximum_delta = max(abs(value) for value in deltas.values())
    travel_bound = sum(
        radii[radius_key] * abs(math.radians(deltas[angle_key]))
        for radius_key, angle_key in (
            ("j2_mm", "j2_deg"),
            ("j3_mm", "j3_deg"),
            ("j4_mm", "j4_deg"),
        )
    )
    travel_samples = int(math.ceil(travel_bound / MAX_DESCENDANT_TRAVEL_MM))
    angular_samples = int(math.ceil(maximum_delta / MAX_JOINT_STEP_DEG))

    requested = None
    requested_samples = 0
    if requested_step_deg is not None:
        requested = abs(float(requested_step_deg))
        if not math.isfinite(requested) or requested <= 0.0:
            raise ValueError("sweep_step_deg 必须是有限正数")
        requested_samples = int(math.ceil(maximum_delta / requested))

    sample_count = max(1, travel_samples, angular_samples, requested_samples)
    if sample_count > MAX_SWEEP_SAMPLES:
        raise RuntimeError(
            f"安全扫掠需要 {sample_count} 个样本，超过上限 {MAX_SWEEP_SAMPLES}，已拒绝移动"
        )

    radius_sum = sum(radii.values())
    adaptive_equal_step = min(
        MAX_JOINT_STEP_DEG,
        math.degrees(MAX_DESCENDANT_TRAVEL_MM / radius_sum),
    )
    actual_max_step = maximum_delta / sample_count if maximum_delta else 0.0
    return {
        "sample_count": sample_count,
        "deltas_deg": deltas,
        "radii_mm": radii,
        "travel_bound_mm": travel_bound,
        "travel_sample_count": travel_samples,
        "angular_sample_count": angular_samples,
        "requested_sample_count": requested_samples,
        "requested_step_deg": requested,
        "adaptive_equal_axis_step_deg": adaptive_equal_step,
        "actual_max_joint_step_deg": actual_max_step,
        "requested_coarser_than_applied": (
            requested is not None and requested > actual_max_step + 1.0e-12
        ),
    }


def _sample_path(start, target, count):
    for index in range(1, count + 1):
        fraction = index / count
        yield index, {
            key: start[key] + (target[key] - start[key]) * fraction
            for key in ("j2_deg", "j3_deg", "j4_deg")
        }


def _snapshot_pose(doc):
    _moving, r2, r3, r4, j2, j3, j4 = _chain_objects(doc)
    return {
        "placements": {
            J2_RIGID_NAME: App.Placement(r2.Placement),
            J3_RIGID_NAME: App.Placement(r3.Placement),
            J4_RIGID_NAME: App.Placement(r4.Placement),
        },
        "angle_properties": {
            J2_JOINT_NAME: float(j2.Angle),
            J3_JOINT_NAME: float(j3.Angle),
            J4_JOINT_NAME: float(j4.Angle),
        },
        "actual_angles": current_angles(doc),
    }


def _placement_error(first, second):
    delta = first.inverse().multiply(second)
    return delta.Base.Length, abs(math.degrees(float(delta.Rotation.Angle)))


def _restore_snapshot(doc, snapshot):
    _moving, r2, r3, r4, j2, j3, j4 = _chain_objects(doc)
    for obj, name in (
        (r2, J2_RIGID_NAME),
        (r3, J3_RIGID_NAME),
        (r4, J4_RIGID_NAME),
    ):
        obj.Placement = App.Placement(snapshot["placements"][name])
    for joint, name in (
        (j2, J2_JOINT_NAME),
        (j3, J3_JOINT_NAME),
        (j4, J4_JOINT_NAME),
    ):
        joint.Angle = float(snapshot["angle_properties"][name])
    doc.recompute()

    translation_errors = []
    rotation_errors = []
    for obj, name in (
        (r2, J2_RIGID_NAME),
        (r3, J3_RIGID_NAME),
        (r4, J4_RIGID_NAME),
    ):
        translation, rotation = _placement_error(
            snapshot["placements"][name], obj.Placement
        )
        translation_errors.append(translation)
        rotation_errors.append(rotation)
    angle_errors = [
        abs(float(joint.Angle) - snapshot["angle_properties"][name])
        for joint, name in (
            (j2, J2_JOINT_NAME),
            (j3, J3_JOINT_NAME),
            (j4, J4_JOINT_NAME),
        )
    ]
    maximum_translation = max(translation_errors)
    maximum_rotation = max(rotation_errors)
    maximum_angle = max(angle_errors)
    return {
        "verified": (
            maximum_translation <= 1.0e-8
            and maximum_rotation <= 1.0e-8
            and maximum_angle <= 1.0e-9
        ),
        "max_translation_error_mm": maximum_translation,
        "max_rotation_error_deg": maximum_rotation,
        "max_angle_property_error_deg": maximum_angle,
    }


def _set_status(
    doc,
    state,
    angles,
    result,
    plan=None,
    first_collision=None,
    rollback=None,
):
    status = doc.getObject(STATUS_NAME)
    if status is None:
        return
    definitions = (
        ("RuntimeState", "App::PropertyString"),
        ("RuntimeJ2Angle", "App::PropertyAngle"),
        ("RuntimeJ3Angle", "App::PropertyAngle"),
        ("RuntimeJ4Angle", "App::PropertyAngle"),
        ("RuntimeSweepStep", "App::PropertyAngle"),
        ("RuntimeSweepSamples", "App::PropertyInteger"),
        ("RuntimeCollisionJSON", "App::PropertyString"),
        ("RuntimeFirstCollisionJSON", "App::PropertyString"),
        ("RuntimeRollbackVerified", "App::PropertyBool"),
    )
    for name, property_type in definitions:
        if name not in status.PropertiesList:
            status.addProperty(property_type, name, "Runtime Collision Guard")
    status.RuntimeState = state
    status.RuntimeJ2Angle = float(angles["j2_deg"])
    status.RuntimeJ3Angle = float(angles["j3_deg"])
    status.RuntimeJ4Angle = float(angles["j4_deg"])
    if plan is not None:
        status.RuntimeSweepStep = float(plan["actual_max_joint_step_deg"])
        status.RuntimeSweepSamples = int(plan["sample_count"])
    status.RuntimeCollisionJSON = json.dumps(result, ensure_ascii=False)
    status.RuntimeFirstCollisionJSON = json.dumps(
        first_collision or {}, ensure_ascii=False
    )
    status.RuntimeRollbackVerified = bool(
        rollback is not None and rollback.get("verified", False)
    )


def _validate_target_limits(joints, target):
    for joint, key, label in (
        (joints[0], "j2_deg", "J2"),
        (joints[1], "j3_deg", "J3"),
        (joints[2], "j4_deg", "J4"),
    ):
        value = target[key]
        if not math.isfinite(value):
            raise ValueError(f"{label}目标角必须是有限数值")
        lower_enabled = (
            "EnableAngleMin" not in joint.PropertiesList or bool(joint.EnableAngleMin)
        )
        upper_enabled = (
            "EnableAngleMax" not in joint.PropertiesList or bool(joint.EnableAngleMax)
        )
        lower = float(joint.AngleMin)
        upper = float(joint.AngleMax)
        if lower_enabled and value < lower - ANGLE_EPS_DEG:
            raise ValueError(f"{label}目标角 {value:.3f}° 低于下限 {lower:.3f}°")
        if upper_enabled and value > upper + ANGLE_EPS_DEG:
            raise ValueError(f"{label}目标角 {value:.3f}° 高于上限 {upper:.3f}°")


def set_j2_j3_j4_angles_v6(
    j2_deg: float | None = None,
    j3_deg: float | None = None,
    j4_deg: float | None = None,
    sweep_step_deg: float | None = None,
):
    """Apply one atomic J2/J3/J4 command with swept collision checking."""

    doc = active_v6_document()
    _moving, _r2, _r3, _r4, j2, j3, j4 = _chain_objects(doc)
    start = current_angles(doc)
    target = {
        "j2_deg": start["j2_deg"] if j2_deg is None else float(j2_deg),
        "j3_deg": start["j3_deg"] if j3_deg is None else float(j3_deg),
        "j4_deg": start["j4_deg"] if j4_deg is None else float(j4_deg),
    }
    _validate_target_limits((j2, j3, j4), target)
    plan = _sweep_plan(doc, start, target, sweep_step_deg)
    snapshot = _snapshot_pose(doc)

    observer = getattr(App, "_J234_COLLISION_OBSERVER_V6", None)
    previous_busy = getattr(observer, "_busy", None) if observer is not None else None
    if observer is not None:
        observer._busy = True

    restored = False
    last_result = collision_result(doc, compute_clearance=False)
    if last_result["collision"]:
        _set_status(doc, "START_POSE_COLLIDING", start, last_result, plan)
        if observer is not None and previous_busy is not None:
            observer._busy = previous_busy
        active = [
            name for name, row in last_result["pairs"].items() if row["collision"]
        ]
        raise RuntimeError("起始位姿已碰撞，已拒绝移动: " + ", ".join(active))

    try:
        for index, sample in _sample_path(start, target, plan["sample_count"]):
            apply_angles_unchecked(
                doc, sample["j2_deg"], sample["j3_deg"], sample["j4_deg"]
            )
            result = collision_result(doc, compute_clearance=False)
            if result["collision"]:
                first_collision = {
                    "sample_index": index,
                    "sample_count": plan["sample_count"],
                    "angles": sample,
                    "active_pairs": [
                        name
                        for name, row in result["pairs"].items()
                        if row["collision"]
                    ],
                }
                rollback = _restore_snapshot(doc, snapshot)
                restored = True
                _set_status(
                    doc,
                    "BLOCKED_AND_REVERTED",
                    snapshot["actual_angles"],
                    result,
                    plan,
                    first_collision,
                    rollback,
                )
                raise RuntimeError(
                    "J2/J3/J4路径在 "
                    f"({sample['j2_deg']:.3f}°, {sample['j3_deg']:.3f}°, "
                    f"{sample['j4_deg']:.3f}°) 发生碰撞，已原子回滚: "
                    + ", ".join(first_collision["active_pairs"])
                )
            last_result = result

        j2.Angle = target["j2_deg"]
        j3.Angle = target["j3_deg"]
        j4.Angle = target["j4_deg"]
        doc.recompute()
        applied = current_angles(doc)
        last_result = collision_result(doc, compute_clearance=True)
        if last_result["collision"]:
            raise RuntimeError("终点清除量复核发现碰撞")
        _set_status(doc, "CLEAR", applied, last_result, plan)
    except Exception:
        if not restored:
            rollback = _restore_snapshot(doc, snapshot)
            restored = True
            _set_status(
                doc,
                "ERROR_AND_REVERTED",
                snapshot["actual_angles"],
                last_result,
                plan,
                rollback=rollback,
            )
        raise
    finally:
        if observer is not None and previous_busy is not None:
            observer._busy = previous_busy

    output = {
        "status": "PASS",
        "start": start,
        "target": target,
        "applied": current_angles(doc),
        "sweep": plan,
        "collision": False,
        "collision_scope": last_result["scope"],
        "result": last_result,
    }
    App.Console.PrintMessage(json.dumps(output, ensure_ascii=False) + "\n")
    return output


def set_j2_angle_v6(angle_deg: float, sweep_step_deg: float | None = None):
    return set_j2_j3_j4_angles_v6(
        j2_deg=angle_deg, sweep_step_deg=sweep_step_deg
    )


def set_j3_angle_v6(angle_deg: float, sweep_step_deg: float | None = None):
    return set_j2_j3_j4_angles_v6(
        j3_deg=angle_deg, sweep_step_deg=sweep_step_deg
    )


def set_j4_angle_v6(angle_deg: float, sweep_step_deg: float | None = None):
    return set_j2_j3_j4_angles_v6(
        j4_deg=angle_deg, sweep_step_deg=sweep_step_deg
    )


# Compatibility wrappers keep J4 at its current relative angle and ensure that
# R4 follows J2/J3.  They intentionally replace the old V5 App functions.
def set_j2_j3_angles_v5(
    j2_deg: float | None = None,
    j3_deg: float | None = None,
    sweep_step_deg: float | None = None,
):
    return set_j2_j3_j4_angles_v6(
        j2_deg=j2_deg, j3_deg=j3_deg, sweep_step_deg=sweep_step_deg
    )


def set_j2_angle_v5(angle_deg: float, sweep_step_deg: float | None = None):
    return set_j2_angle_v6(angle_deg, sweep_step_deg)


def set_j3_angle_v5(angle_deg: float, sweep_step_deg: float | None = None):
    return set_j3_angle_v6(angle_deg, sweep_step_deg)


class J234CollisionObserver:
    def __init__(self, doc):
        self.doc_name = doc.Name
        self._busy = False
        self._capture(doc)

    def _capture(self, doc):
        self.last_snapshot = _snapshot_pose(doc)
        self.last_angles = dict(self.last_snapshot["actual_angles"])

    def _placement_target(self, doc, obj):
        moving, r2, r3, r4, j2, j3, j4 = _chain_objects(doc)
        mapping = {
            J2_RIGID_NAME: (moving, r2, j2, "j2_deg"),
            J3_RIGID_NAME: (r2, r3, j3, "j3_deg"),
            J4_RIGID_NAME: (r3, r4, j4, "j4_deg"),
        }
        parent, child, joint, key = mapping[obj.Name]
        angle = _relative_angle(parent, child, joint)
        expected = parent.Placement.multiply(relative_placement(joint, angle))
        translation, rotation = _placement_error(expected, child.Placement)
        if (
            translation > PLACEMENT_TRANSLATION_TOLERANCE_MM
            or rotation > PLACEMENT_ROTATION_TOLERANCE_DEG
        ):
            raise RuntimeError(
                f"{obj.Name}编辑包含非关节纯转动: "
                f"平移残差 {translation:.6g} mm, 转动残差 {rotation:.6g}°"
            )
        target = dict(self.last_angles)
        target[key] = angle
        return target

    def slotChangedObject(self, obj, prop):
        if self._busy or obj.Document is None or obj.Document.Name != self.doc_name:
            return
        watched_angle = obj.Name in (
            J2_JOINT_NAME,
            J3_JOINT_NAME,
            J4_JOINT_NAME,
        ) and prop == "Angle"
        watched_placement = obj.Name in (
            J2_RIGID_NAME,
            J3_RIGID_NAME,
            J4_RIGID_NAME,
        ) and prop == "Placement"
        if not watched_angle and not watched_placement:
            return

        self._busy = True
        try:
            doc = obj.Document
            if watched_angle:
                target = dict(self.last_angles)
                key = {
                    J2_JOINT_NAME: "j2_deg",
                    J3_JOINT_NAME: "j3_deg",
                    J4_JOINT_NAME: "j4_deg",
                }[obj.Name]
                target[key] = float(obj.Angle)
            else:
                target = self._placement_target(doc, obj)

            # The edited property/placement is already changed when the
            # observer is called.  Restore the last valid chain first, then
            # sweep atomically to the requested target.
            _restore_snapshot(doc, self.last_snapshot)
            set_j2_j3_j4_angles_v6(
                target["j2_deg"], target["j3_deg"], target["j4_deg"]
            )
            self._capture(doc)
        except Exception as exc:
            try:
                rollback = _restore_snapshot(obj.Document, self.last_snapshot)
                result = collision_result(obj.Document, compute_clearance=False)
                _set_status(
                    obj.Document,
                    "GUI_EDIT_BLOCKED_AND_REVERTED",
                    self.last_angles,
                    result,
                    rollback=rollback,
                )
            except Exception as rollback_exc:
                App.Console.PrintError(str(rollback_exc) + "\n")
            App.Console.PrintError(str(exc) + "\n")
        finally:
            self._busy = False


def _remove_observer_attribute(attribute_name):
    observer = getattr(App, attribute_name, None)
    if observer is not None:
        try:
            App.removeDocumentObserver(observer)
        except Exception:
            pass
        try:
            delattr(App, attribute_name)
        except Exception:
            pass


def install_j234_collision_monitor_v6():
    doc = active_v6_document()
    # Old V5 callbacks move only R2/R3 and would leave J4 behind.
    _remove_observer_attribute("_J23_COLLISION_OBSERVER_V5")
    _remove_observer_attribute("_J234_COLLISION_OBSERVER_V6")

    observer = J234CollisionObserver(doc)
    App.addDocumentObserver(observer)
    App._J234_COLLISION_OBSERVER_V6 = observer

    App.set_j2_j3_j4_angles_v6 = set_j2_j3_j4_angles_v6
    App.set_j2_angle_v6 = set_j2_angle_v6
    App.set_j3_angle_v6 = set_j3_angle_v6
    App.set_j4_angle_v6 = set_j4_angle_v6
    App.set_j2_j3_angles_v5 = set_j2_j3_angles_v5
    App.set_j2_angle_v5 = set_j2_angle_v5
    App.set_j3_angle_v5 = set_j3_angle_v5

    result = collision_result(doc, compute_clearance=True)
    angles = current_angles(doc)
    plan = _sweep_plan(doc, angles, angles, None)
    _set_status(
        doc,
        "CLEAR" if not result["collision"] else "COLLISION",
        angles,
        result,
        plan,
    )
    App.Console.PrintMessage(
        "V6 J2/J3/J4扫掠碰撞监视已启动（未保存FCStd）。"
        f"外部碰撞对 {len(result['pairs'])} 组，J4定转子pair已启用。"
        f"等角最坏自适应步长 {plan['adaptive_equal_axis_step_deg']:.4f}°，"
        f"单关节硬上限 {MAX_JOINT_STEP_DEG:.1f}°。"
        "命令：App.set_j4_angle_v6(deg) 或 "
        "App.set_j2_j3_j4_angles_v6(j2,j3,j4)。\n"
    )
    return observer


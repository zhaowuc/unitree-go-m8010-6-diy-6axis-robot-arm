from __future__ import annotations

"""V12 FreeCAD GUI swept-collision guard for the complete J1..J5 chain.

The V10 collision matrix is reused for the validated J1..J4 bodies.  This
module adds the terminal J5 stator/output plus Damiao-adapter collision pairs and owns the complete
R2 -> R3 -> R4 -> R5 kinematic propagation, snapshots, atomic rollback and GUI
observers.  Nothing in this module saves the FCStd document.
"""

import json
import math
from pathlib import Path

import FreeCAD as App


ROOT = Path(__file__).resolve().parent
V10_GUARD = ROOT / "j1234_physical_collision_guard_v10.py"
V12_FCSTD_NAME = "机械臂底座肩部大臂小臂腕前连接件末端J5与达妙转接件装配_v12.FCStd"

ASSEMBLY_NAME = "RobotBaseShoulderAssembly"
J1_MOVING_NAME = "J1_Moving_Rigid"
J2_RIGID_NAME = "J2_Driven_Reference_Rigid"
J3_RIGID_NAME = "J3_Driven_Rigid"
J4_RIGID_NAME = "J4_Driven_Rigid"
J5_RIGID_NAME = "J5_Driven_Rigid"
J1_JOINT_NAME = "J1_Revolute"
J2_JOINT_NAME = "J2_Revolute"
J3_JOINT_NAME = "J3_Revolute"
J4_JOINT_NAME = "J4_Revolute"
J5_JOINT_NAME = "J5_Revolute"

FIXED_PROXY_NAME = "J1_Fixed_Collision_Proxy"
SHOULDER_PROXY_NAME = "J1_Moving_Collision_Proxy"
UPPERARM_FULL_PROXY_NAME = "UpperArm_Full_Collision_Proxy"
J3_OUTPUT_PROXY_NAME = "J3_Output_Flange_Proxy"
J3_STATOR_PROXY_NAME = "J3_Motor_Stator_Collision_Proxy"
FOREARM_PROXY_NAME = "Forearm_Collision_Proxy"
J4_STATOR_PROXY_NAME = "J4_Motor_Stator_Collision_Proxy"
J4_OUTPUT_PROXY_NAME = "J4_Output_Flange_Collision_Proxy"
WRIST_PROXY_NAME = "Wrist_Prelink_Collision_Proxy"
J5_STATOR_PROXY_NAME = "J5_Motor_Stator_Collision_Proxy"
J5_OUTPUT_PROXY_NAME = "J5_Output_Flange_Collision_Proxy"
ADAPTER_PROXY_NAME = "J5_to_Damiao_J6_Adapter_Collision_Proxy"

STATUS_NAME = "V12_J5_Damiao_Adapter_Physical_Test_Status"
SERIAL_OBSERVER_ATTRIBUTE = "_J2345_PHYSICAL_COLLISION_OBSERVER_V12"
J1_OBSERVER_ATTRIBUTE = "_J1_FULL_CHAIN_COLLISION_OBSERVER_V12"

PENETRATION_VOLUME_TOLERANCE_MM3 = 1.0e-3
MAX_JOINT_STEP_DEG = 0.5
MAX_DESCENDANT_TRAVEL_MM = 1.0
MAX_SWEEP_SAMPLES = 20000
ANGLE_EPS_DEG = 1.0e-9
APPLIED_ANGLE_TOLERANCE_DEG = 1.0e-7
PLACEMENT_TRANSLATION_TOLERANCE_MM = 1.0e-4
PLACEMENT_ROTATION_TOLERANCE_DEG = 1.0e-4
J4_RUNTIME_MIN_DEG = -116.0
J4_RUNTIME_MAX_DEG = 159.0
J5_RUNTIME_MIN_DEG = -70.6
J5_RUNTIME_MAX_DEG = 151.2


if not V10_GUARD.is_file():
    raise RuntimeError("缺少V10物理碰撞保护基底: " + str(V10_GUARD))

_v10_source = V10_GUARD.read_text(encoding="utf-8")
_v10 = {
    "__file__": str(V10_GUARD),
    "__name__": "_j1234_physical_collision_guard_v10_base_for_v11",
}
exec(compile(_v10_source, str(V10_GUARD), "exec"), _v10)
_v9 = _v10["namespace"]
_v8 = _v9["_v8"]
_v7 = _v8["_v7"]
_base = _v7["_base"]
_collision_result_v10 = _v10["collision_result_v10"]

world_shape = _base["world_shape"]
_overlap_and_distance = _base["_overlap_and_distance"]
_joint_frame = _base["_joint_frame"]
relative_placement = _base["relative_placement"]
_relative_angle = _base["_relative_angle"]
_world_joint_axis = _base["_world_joint_axis"]
_radial_bound = _base["_radial_bound"]
_base_placement_error = _base["_placement_error"]


def _placement_error(first, second):
    translation, rotation = _base_placement_error(first, second)
    # FreeCAD may encode an equivalent one-degree delta as 359 degrees,
    # depending on quaternion sign.  Use the shortest SO(3) distance.
    rotation = abs(((float(rotation) + 180.0) % 360.0) - 180.0)
    return float(translation), rotation


def active_v12_document():
    doc = App.ActiveDocument
    required = (
        ASSEMBLY_NAME,
        J1_MOVING_NAME,
        J2_RIGID_NAME,
        J3_RIGID_NAME,
        J4_RIGID_NAME,
        J5_RIGID_NAME,
        J1_JOINT_NAME,
        J2_JOINT_NAME,
        J3_JOINT_NAME,
        J4_JOINT_NAME,
        J5_JOINT_NAME,
        FIXED_PROXY_NAME,
        SHOULDER_PROXY_NAME,
        UPPERARM_FULL_PROXY_NAME,
        J3_OUTPUT_PROXY_NAME,
        J3_STATOR_PROXY_NAME,
        FOREARM_PROXY_NAME,
        J4_STATOR_PROXY_NAME,
        J4_OUTPUT_PROXY_NAME,
        WRIST_PROXY_NAME,
        J5_STATOR_PROXY_NAME,
        J5_OUTPUT_PROXY_NAME,
        ADAPTER_PROXY_NAME,
        STATUS_NAME,
        *(f"Wrist_Prelink_J5_M35_Screw_{index:02d}" for index in range(1, 7)),
    )
    if doc is None or any(doc.getObject(name) is None for name in required):
        missing = [] if doc is None else [
            name for name in required if doc.getObject(name) is None
        ]
        suffix = "" if not missing else "\n缺失对象: " + ", ".join(missing)
        raise RuntimeError("请先打开 " + V12_FCSTD_NAME + suffix)
    adapter = doc.getObject(ADAPTER_PROXY_NAME)
    r5 = doc.getObject(J5_RIGID_NAME)
    if adapter not in tuple(r5.Group):
        raise RuntimeError("V12转接件碰撞代理必须属于 J5_Driven_Rigid")
    if adapter.Shape.isNull() or not adapter.Shape.isValid() or len(adapter.Shape.Solids) != 1 or adapter.Shape.Volume <= 0.0:
        raise RuntimeError("V12转接件碰撞代理必须是有效的单一BRep实体")
    return doc


def _shape_objects(doc):
    names = (
        FIXED_PROXY_NAME,
        SHOULDER_PROXY_NAME,
        UPPERARM_FULL_PROXY_NAME,
        J3_OUTPUT_PROXY_NAME,
        J3_STATOR_PROXY_NAME,
        FOREARM_PROXY_NAME,
        J4_STATOR_PROXY_NAME,
        J4_OUTPUT_PROXY_NAME,
        WRIST_PROXY_NAME,
        J5_STATOR_PROXY_NAME,
        J5_OUTPUT_PROXY_NAME,
        ADAPTER_PROXY_NAME,
    )
    objects = {name: doc.getObject(name) for name in names}
    missing = [name for name, obj in objects.items() if obj is None]
    if missing:
        raise RuntimeError("V12碰撞代理缺失: " + ", ".join(missing))
    return objects


def collision_result(doc=None, compute_clearance=True):
    """Evaluate V10 pairs plus every external pair introduced by J5."""

    doc = doc or active_v12_document()
    result = dict(_collision_result_v10(doc, compute_clearance=compute_clearance))
    rows = dict(result["pairs"])
    objects = _shape_objects(doc)
    shape = lambda name: world_shape(objects[name])

    fixed = shape(FIXED_PROXY_NAME)
    shoulder = shape(SHOULDER_PROXY_NAME)
    upperarm = shape(UPPERARM_FULL_PROXY_NAME)
    j3_output = shape(J3_OUTPUT_PROXY_NAME)
    j3_stator = shape(J3_STATOR_PROXY_NAME)
    forearm = shape(FOREARM_PROXY_NAME)
    j4_stator = shape(J4_STATOR_PROXY_NAME)
    j4_output = shape(J4_OUTPUT_PROXY_NAME)
    wrist = shape(WRIST_PROXY_NAME)
    j5_stator = shape(J5_STATOR_PROXY_NAME)
    j5_output = shape(J5_OUTPUT_PROXY_NAME)
    adapter = shape(ADAPTER_PROXY_NAME)

    new_pairs = {
        # J5 stator belongs to R4.  Its seated wrist/J4-output interfaces are
        # same-rigid exclusions; all older rigid bodies remain active.
        "j5_stator_vs_base": (j5_stator, fixed),
        "j5_stator_vs_shoulder": (j5_stator, shoulder),
        "j5_stator_vs_upperarm": (j5_stator, upperarm),
        "j5_stator_vs_j3_output": (j5_stator, j3_output),
        "j5_stator_vs_j3_stator": (j5_stator, j3_stator),
        "j5_stator_vs_forearm": (j5_stator, forearm),
        "j5_stator_vs_j4_stator": (j5_stator, j4_stator),
        # J5 output belongs to R5 and is checked against every older rigid
        # body, including its adjacent stator.  Zero-volume tangency is legal.
        "j5_output_vs_base": (j5_output, fixed),
        "j5_output_vs_shoulder": (j5_output, shoulder),
        "j5_output_vs_upperarm": (j5_output, upperarm),
        "j5_output_vs_j3_output": (j5_output, j3_output),
        "j5_output_vs_j3_stator": (j5_output, j3_stator),
        "j5_output_vs_forearm": (j5_output, forearm),
        "j5_output_vs_j4_stator": (j5_output, j4_stator),
        "j5_output_vs_j4_output": (j5_output, j4_output),
        "j5_output_vs_wrist": (j5_output, wrist),
        "j5_output_vs_j5_stator": (j5_output, j5_stator),
        # The orthogonal adapter is in R5.  Its seated J5-output interface is
        # same-rigid and excluded; every older rigid body remains active.
        "adapter_vs_base": (adapter, fixed),
        "adapter_vs_shoulder": (adapter, shoulder),
        "adapter_vs_upperarm": (adapter, upperarm),
        "adapter_vs_j3_output": (adapter, j3_output),
        "adapter_vs_j3_stator": (adapter, j3_stator),
        "adapter_vs_forearm": (adapter, forearm),
        "adapter_vs_j4_stator": (adapter, j4_stator),
        "adapter_vs_j4_output": (adapter, j4_output),
        "adapter_vs_wrist": (adapter, wrist),
        "adapter_vs_j5_stator": (adapter, j5_stator),
    }

    collision = bool(result["collision"])
    for name, (first, second) in new_pairs.items():
        overlap, distance = _overlap_and_distance(
            first, second, compute_clearance=compute_clearance
        )
        colliding = overlap > PENETRATION_VOLUME_TOLERANCE_MM3
        rows[name] = {
            "overlap_mm3": overlap,
            "clearance_mm": 0.0 if colliding else distance,
            "collision": colliding,
        }
        collision = collision or colliding

    exclusions = list(result.get("same_rigid_exclusions", []))
    exclusions.extend(
        [
            "J4 output/wrist pre-link vs J5 stator and six terminal M3.5 fasteners",
            "J5 output rotor vs neutral bearing display/future output fasteners",
            "J5-to-Damiao adapter vs J5 output rotor seated interface (same R5 rigid body)",
        ]
    )
    adjacent = list(result.get("adjacent_joint_pairs_checked", []))
    adjacent.append("j5_output_vs_j5_stator")
    adjacent.append("adapter_vs_j5_stator")
    result.update(
        {
            "collision": collision,
            "scope": "J1 environment + complete J2/J3/J4/J5 serial chain + J5-to-Damiao adapter",
            "penetration_volume_tolerance_mm3": PENETRATION_VOLUME_TOLERANCE_MM3,
            "same_rigid_exclusions": exclusions,
            "adjacent_joint_pairs_checked": adjacent,
            "pairs": rows,
        }
    )
    return result


def _chain_objects(doc):
    moving = doc.getObject(J1_MOVING_NAME)
    r2 = doc.getObject(J2_RIGID_NAME)
    r3 = doc.getObject(J3_RIGID_NAME)
    r4 = doc.getObject(J4_RIGID_NAME)
    r5 = doc.getObject(J5_RIGID_NAME)
    j2 = doc.getObject(J2_JOINT_NAME)
    j3 = doc.getObject(J3_JOINT_NAME)
    j4 = doc.getObject(J4_JOINT_NAME)
    j5 = doc.getObject(J5_JOINT_NAME)
    objects = (moving, r2, r3, r4, r5, j2, j3, j4, j5)
    if any(obj is None for obj in objects):
        raise RuntimeError("V12 J2/J3/J4/J5刚体或关节缺失")
    return objects


def current_angles(doc=None):
    doc = doc or active_v12_document()
    moving, r2, r3, r4, r5, j2, j3, j4, j5 = _chain_objects(doc)
    return {
        "j2_deg": _relative_angle(moving, r2, j2),
        "j3_deg": _relative_angle(r2, r3, j3),
        "j4_deg": _relative_angle(r3, r4, j4),
        "j5_deg": _relative_angle(r4, r5, j5),
    }


def _j1_angle_from_placement(joint, placement, reference_deg):
    _origin, axis, zero = _joint_frame(joint)
    actual = placement.Rotation.multVec(zero)
    sine = axis.dot(zero.cross(actual))
    cosine = zero.dot(actual)
    wrapped = math.degrees(math.atan2(sine, cosine))
    # Preserve a continuous turn count by selecting the equivalent angle
    # nearest the last Angle property/snapshot instead of forcing [-180,180].
    return wrapped + 360.0 * round((float(reference_deg) - wrapped) / 360.0)


def current_j1_angle(doc=None):
    doc = doc or active_v12_document()
    moving = doc.getObject(J1_MOVING_NAME)
    joint = doc.getObject(J1_JOINT_NAME)
    return _j1_angle_from_placement(joint, moving.Placement, float(joint.Angle))


def current_j12345_angles(doc=None):
    doc = doc or active_v12_document()
    result = {"j1_deg": current_j1_angle(doc)}
    result.update(current_angles(doc))
    return result


def apply_angles_unchecked(doc, j2_deg, j3_deg, j4_deg, j5_deg):
    moving, r2, r3, r4, r5, j2, j3, j4, j5 = _chain_objects(doc)
    r2.Placement = moving.Placement.multiply(relative_placement(j2, j2_deg))
    r3.Placement = r2.Placement.multiply(relative_placement(j3, j3_deg))
    r4.Placement = r3.Placement.multiply(relative_placement(j4, j4_deg))
    r5.Placement = r4.Placement.multiply(relative_placement(j5, j5_deg))
    doc.recompute()


def _apply_j12345_unchecked(doc, j1_deg, j2_deg, j3_deg, j4_deg, j5_deg):
    moving, r2, r3, r4, r5, j2, j3, j4, j5 = _chain_objects(doc)
    j1 = doc.getObject(J1_JOINT_NAME)
    moving.Placement = relative_placement(j1, j1_deg)
    r2.Placement = moving.Placement.multiply(relative_placement(j2, j2_deg))
    r3.Placement = r2.Placement.multiply(relative_placement(j3, j3_deg))
    r4.Placement = r3.Placement.multiply(relative_placement(j4, j4_deg))
    r5.Placement = r4.Placement.multiply(relative_placement(j5, j5_deg))
    doc.recompute()


def _boundbox_corners(bound_box):
    for x in (bound_box.XMin, bound_box.XMax):
        for y in (bound_box.YMin, bound_box.YMax):
            for z in (bound_box.ZMin, bound_box.ZMax):
                yield App.Vector(x, y, z)


def _sphere_bound(shape, origin):
    return max((point - origin).Length for point in _boundbox_corners(shape.BoundBox))


def kinematic_radii_mm(doc=None):
    """Conservative downstream radii including R4's J5 stator and R5."""

    doc = doc or active_v12_document()
    moving, r2, r3, r4, _r5, j2, j3, j4, j5 = _chain_objects(doc)
    o2, a2 = _world_joint_axis(moving, j2)
    o3, a3 = _world_joint_axis(r2, j3)
    o4, a4 = _world_joint_axis(r3, j4)
    o5, a5 = _world_joint_axis(r4, j5)
    objects = _shape_objects(doc)
    shape = lambda name: world_shape(objects[name])

    upperarm = shape(UPPERARM_FULL_PROXY_NAME)
    j3_output = shape(J3_OUTPUT_PROXY_NAME)
    j3_stator = shape(J3_STATOR_PROXY_NAME)
    forearm = shape(FOREARM_PROXY_NAME)
    j4_stator = shape(J4_STATOR_PROXY_NAME)
    j4_output = shape(J4_OUTPUT_PROXY_NAME)
    wrist = shape(WRIST_PROXY_NAME)
    j5_stator = shape(J5_STATOR_PROXY_NAME)
    j5_output = shape(J5_OUTPUT_PROXY_NAME)
    adapter = shape(ADAPTER_PROXY_NAME)

    r5 = max(
        _radial_bound(j5_output, o5, a5),
        _radial_bound(adapter, o5, a5),
        1.0,
    )
    r4 = max(
        _radial_bound(j4_output, o4, a4),
        _radial_bound(wrist, o4, a4),
        _radial_bound(j5_stator, o4, a4),
        _sphere_bound(j5_output, o4),
        _sphere_bound(adapter, o4),
        1.0,
    )
    r3_bound = max(
        _radial_bound(j3_stator, o3, a3),
        _radial_bound(forearm, o3, a3),
        _radial_bound(j4_stator, o3, a3),
        *(
            _sphere_bound(item, o3)
            for item in (j4_output, wrist, j5_stator, j5_output, adapter)
        ),
        1.0,
    )
    r2_bound = max(
        _radial_bound(upperarm, o2, a2),
        _radial_bound(j3_output, o2, a2),
        *(
            _sphere_bound(item, o2)
            for item in (j3_stator, forearm, j4_stator, j4_output, wrist, j5_stator, j5_output, adapter)
        ),
        1.0,
    )
    return {"j2_mm": r2_bound, "j3_mm": r3_bound, "j4_mm": r4, "j5_mm": r5}


ANGLE_KEYS = ("j2_deg", "j3_deg", "j4_deg", "j5_deg")
RADIUS_KEYS = ("j2_mm", "j3_mm", "j4_mm", "j5_mm")


def _sweep_plan(doc, start, target, requested_step_deg):
    radii = kinematic_radii_mm(doc)
    deltas = {key: target[key] - start[key] for key in ANGLE_KEYS}
    maximum_delta = max(abs(value) for value in deltas.values())
    travel_bound = sum(
        radii[radius_key] * abs(math.radians(deltas[angle_key]))
        for radius_key, angle_key in zip(RADIUS_KEYS, ANGLE_KEYS)
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
    actual_step = maximum_delta / sample_count if maximum_delta else 0.0
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
        "actual_max_joint_step_deg": actual_step,
        "requested_coarser_than_applied": (
            requested is not None and requested > actual_step + 1.0e-12
        ),
    }


def _sample_path(start, target, count):
    for index in range(1, count + 1):
        fraction = index / count
        yield index, {
            key: start[key] + (target[key] - start[key]) * fraction
            for key in ANGLE_KEYS
        }


def _serial_snapshot(doc):
    _moving, r2, r3, r4, r5, j2, j3, j4, j5 = _chain_objects(doc)
    return {
        "placements": {
            J2_RIGID_NAME: App.Placement(r2.Placement),
            J3_RIGID_NAME: App.Placement(r3.Placement),
            J4_RIGID_NAME: App.Placement(r4.Placement),
            J5_RIGID_NAME: App.Placement(r5.Placement),
        },
        "angle_properties": {
            J2_JOINT_NAME: float(j2.Angle),
            J3_JOINT_NAME: float(j3.Angle),
            J4_JOINT_NAME: float(j4.Angle),
            J5_JOINT_NAME: float(j5.Angle),
        },
        "actual_angles": current_angles(doc),
    }


def _restore_snapshot_payload(doc, snapshot, placement_names):
    for name in placement_names:
        doc.getObject(name).Placement = App.Placement(snapshot["placements"][name])
    for name, value in snapshot["angle_properties"].items():
        doc.getObject(name).Angle = float(value)
    doc.recompute()
    placement_errors = {
        name: _placement_error(snapshot["placements"][name], doc.getObject(name).Placement)
        for name in placement_names
    }
    angle_errors = {
        name: abs(float(doc.getObject(name).Angle) - value)
        for name, value in snapshot["angle_properties"].items()
    }
    verified = (
        max(value[0] for value in placement_errors.values()) <= 1.0e-8
        and max(value[1] for value in placement_errors.values()) <= 1.0e-8
        and max(angle_errors.values()) <= 1.0e-9
    )
    return {
        "verified": verified,
        "placement_errors": {
            name: {"translation_mm": value[0], "rotation_deg": value[1]}
            for name, value in placement_errors.items()
        },
        "angle_property_errors_deg": angle_errors,
    }


def _restore_serial_snapshot(doc, snapshot):
    return _restore_snapshot_payload(
        doc,
        snapshot,
        (J2_RIGID_NAME, J3_RIGID_NAME, J4_RIGID_NAME, J5_RIGID_NAME),
    )


def _full_snapshot(doc):
    snapshot = _serial_snapshot(doc)
    moving = doc.getObject(J1_MOVING_NAME)
    j1 = doc.getObject(J1_JOINT_NAME)
    snapshot["placements"] = {
        J1_MOVING_NAME: App.Placement(moving.Placement),
        **snapshot["placements"],
    }
    snapshot["angle_properties"] = {
        J1_JOINT_NAME: float(j1.Angle),
        **snapshot["angle_properties"],
    }
    snapshot["actual_angles"] = current_j12345_angles(doc)
    return snapshot


def _restore_full_snapshot(doc, snapshot):
    return _restore_snapshot_payload(
        doc,
        snapshot,
        (
            J1_MOVING_NAME,
            J2_RIGID_NAME,
            J3_RIGID_NAME,
            J4_RIGID_NAME,
            J5_RIGID_NAME,
        ),
    )


def _require_verified_rollback(rollback, context):
    if not rollback.get("verified", False):
        raise RuntimeError(
            context
            + "回滚验证失败: "
            + json.dumps(rollback, ensure_ascii=False)
        )


def _verify_serial_target(doc, target):
    applied = current_angles(doc)
    angle_errors = {
        key: abs(applied[key] - target[key]) for key in ANGLE_KEYS
    }
    moving, r2, r3, r4, r5, j2, j3, j4, j5 = _chain_objects(doc)
    expected = {
        J2_RIGID_NAME: moving.Placement.multiply(relative_placement(j2, target["j2_deg"])),
    }
    expected[J3_RIGID_NAME] = expected[J2_RIGID_NAME].multiply(
        relative_placement(j3, target["j3_deg"])
    )
    expected[J4_RIGID_NAME] = expected[J3_RIGID_NAME].multiply(
        relative_placement(j4, target["j4_deg"])
    )
    expected[J5_RIGID_NAME] = expected[J4_RIGID_NAME].multiply(
        relative_placement(j5, target["j5_deg"])
    )
    actual_objects = {
        J2_RIGID_NAME: r2,
        J3_RIGID_NAME: r3,
        J4_RIGID_NAME: r4,
        J5_RIGID_NAME: r5,
    }
    placement_errors = {
        name: _placement_error(placement, actual_objects[name].Placement)
        for name, placement in expected.items()
    }
    if (
        max(angle_errors.values()) > APPLIED_ANGLE_TOLERANCE_DEG
        or max(value[0] for value in placement_errors.values())
        > PLACEMENT_TRANSLATION_TOLERANCE_MM
        or max(value[1] for value in placement_errors.values())
        > PLACEMENT_ROTATION_TOLERANCE_DEG
    ):
        raise RuntimeError(
            "终点与请求角或理论刚体链不一致: "
            + json.dumps(
                {
                    "angle_errors_deg": angle_errors,
                    "placement_errors": placement_errors,
                },
                ensure_ascii=False,
            )
        )
    return applied


def _verify_full_target(doc, target):
    applied = current_j12345_angles(doc)
    angle_errors = {
        key: abs(applied[key] - target[key])
        for key in ("j1_deg", *ANGLE_KEYS)
    }
    moving = doc.getObject(J1_MOVING_NAME)
    j1 = doc.getObject(J1_JOINT_NAME)
    moving_error = _placement_error(
        relative_placement(j1, target["j1_deg"]), moving.Placement
    )
    _verify_serial_target(doc, {key: target[key] for key in ANGLE_KEYS})
    if (
        max(angle_errors.values()) > APPLIED_ANGLE_TOLERANCE_DEG
        or moving_error[0] > PLACEMENT_TRANSLATION_TOLERANCE_MM
        or moving_error[1] > PLACEMENT_ROTATION_TOLERANCE_DEG
    ):
        raise RuntimeError(
            "J1全链终点与请求不一致: "
            + json.dumps(
                {
                    "angle_errors_deg": angle_errors,
                    "j1_moving_placement_error": moving_error,
                },
                ensure_ascii=False,
            )
        )
    return applied


def _ensure_status_properties(status):
    definitions = (
        ("RuntimeState", "App::PropertyString"),
        ("RuntimeJ1Angle", "App::PropertyAngle"),
        ("RuntimeJ2Angle", "App::PropertyAngle"),
        ("RuntimeJ3Angle", "App::PropertyAngle"),
        ("RuntimeJ4Angle", "App::PropertyAngle"),
        ("RuntimeJ5Angle", "App::PropertyAngle"),
        ("RuntimeSweepStep", "App::PropertyAngle"),
        ("RuntimeSweepSamples", "App::PropertyInteger"),
        ("RuntimeCollisionJSON", "App::PropertyString"),
        ("RuntimeFirstCollisionJSON", "App::PropertyString"),
        ("RuntimeRollbackVerified", "App::PropertyBool"),
    )
    for name, property_type in definitions:
        if name not in status.PropertiesList:
            status.addProperty(property_type, name, "Runtime Collision Guard V12")


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
    _ensure_status_properties(status)
    status.RuntimeState = state
    status.RuntimeJ1Angle = float(angles.get("j1_deg", current_j1_angle(doc)))
    status.RuntimeJ2Angle = float(angles["j2_deg"])
    status.RuntimeJ3Angle = float(angles["j3_deg"])
    status.RuntimeJ4Angle = float(angles["j4_deg"])
    status.RuntimeJ5Angle = float(angles["j5_deg"])
    if plan is not None:
        status.RuntimeSweepStep = float(plan.get("actual_max_joint_step_deg", 0.0))
        status.RuntimeSweepSamples = int(plan.get("sample_count", 1))
    status.RuntimeCollisionJSON = json.dumps(result, ensure_ascii=False)
    status.RuntimeFirstCollisionJSON = json.dumps(
        first_collision or {}, ensure_ascii=False
    )
    status.RuntimeRollbackVerified = bool(
        rollback is not None and rollback.get("verified", False)
    )


def _validate_joint_limit(joint, value, label):
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


def _validate_target_limits(joints, target):
    for joint, key, label in zip(joints, ANGLE_KEYS, ("J2", "J3", "J4", "J5")):
        _validate_joint_limit(joint, target[key], label)
    if target["j4_deg"] < J4_RUNTIME_MIN_DEG - ANGLE_EPS_DEG:
        raise ValueError(
            f"J4目标角 {target['j4_deg']:.3f}° 低于V12高精度碰撞下限 "
            f"{J4_RUNTIME_MIN_DEG:.3f}°"
        )
    if target["j4_deg"] > J4_RUNTIME_MAX_DEG + ANGLE_EPS_DEG:
        raise ValueError(
            f"J4目标角 {target['j4_deg']:.3f}° 高于V12高精度碰撞上限 "
            f"{J4_RUNTIME_MAX_DEG:.3f}°"
        )
    if target["j5_deg"] < J5_RUNTIME_MIN_DEG - ANGLE_EPS_DEG:
        raise ValueError(
            f"J5目标角 {target['j5_deg']:.3f}° 低于V12转接件碰撞下限 "
            f"{J5_RUNTIME_MIN_DEG:.3f}°"
        )
    if target["j5_deg"] > J5_RUNTIME_MAX_DEG + ANGLE_EPS_DEG:
        raise ValueError(
            f"J5目标角 {target['j5_deg']:.3f}° 高于V12转接件碰撞上限 "
            f"{J5_RUNTIME_MAX_DEG:.3f}°"
        )


def _observer_states():
    observers = [
        getattr(App, SERIAL_OBSERVER_ATTRIBUTE, None),
        getattr(App, J1_OBSERVER_ATTRIBUTE, None),
    ]
    states = [getattr(observer, "_busy", None) if observer else None for observer in observers]
    for observer in observers:
        if observer is not None:
            observer._busy = True
    return observers, states


def _restore_observer_states(observers, states):
    for observer, state in zip(observers, states):
        if observer is not None and state is not None:
            observer._busy = state


def _refresh_observers(doc):
    for attribute in (SERIAL_OBSERVER_ATTRIBUTE, J1_OBSERVER_ATTRIBUTE):
        observer = getattr(App, attribute, None)
        if observer is not None and hasattr(observer, "_capture"):
            observer._capture(doc)


def set_j2_j3_j4_j5_angles_v11(
    j2_deg=None,
    j3_deg=None,
    j4_deg=None,
    j5_deg=None,
    sweep_step_deg=None,
):
    """Apply one atomic J2/J3/J4/J5 command with swept collision checking."""

    doc = active_v12_document()
    _moving, _r2, _r3, _r4, _r5, j2, j3, j4, j5 = _chain_objects(doc)
    start = current_angles(doc)
    requested = (j2_deg, j3_deg, j4_deg, j5_deg)
    target = {
        key: start[key] if value is None else float(value)
        for key, value in zip(ANGLE_KEYS, requested)
    }
    _validate_target_limits((j2, j3, j4, j5), target)
    plan = _sweep_plan(doc, start, target, sweep_step_deg)
    snapshot = _serial_snapshot(doc)
    observers, observer_states = _observer_states()
    restored = False
    last_result = collision_result(doc, compute_clearance=False)
    if last_result["collision"]:
        try:
            _set_status(doc, "START_POSE_COLLIDING", start, last_result, plan)
            active = [
                name for name, row in last_result["pairs"].items() if row["collision"]
            ]
        finally:
            _restore_observer_states(observers, observer_states)
        raise RuntimeError("起始位姿已碰撞，已拒绝移动: " + ", ".join(active))
    try:
        for index, sample in _sample_path(start, target, plan["sample_count"]):
            apply_angles_unchecked(
                doc,
                sample["j2_deg"],
                sample["j3_deg"],
                sample["j4_deg"],
                sample["j5_deg"],
            )
            result = collision_result(doc, compute_clearance=False)
            if result["collision"]:
                first = {
                    "sample_index": index,
                    "sample_count": plan["sample_count"],
                    "angles": sample,
                    "active_pairs": [
                        name for name, row in result["pairs"].items() if row["collision"]
                    ],
                }
                rollback = _restore_serial_snapshot(doc, snapshot)
                _require_verified_rollback(rollback, "J2/J3/J4/J5碰撞后")
                restored = True
                _set_status(
                    doc,
                    "BLOCKED_AND_REVERTED",
                    snapshot["actual_angles"],
                    result,
                    plan,
                    first,
                    rollback,
                )
                raise RuntimeError(
                    "J2/J3/J4/J5路径碰撞，已原子回滚: "
                    + ", ".join(first["active_pairs"])
                )
            last_result = result

        for joint, key in zip((j2, j3, j4, j5), ANGLE_KEYS):
            joint.Angle = target[key]
        doc.recompute()
        applied = _verify_serial_target(doc, target)
        last_result = collision_result(doc, compute_clearance=True)
        if last_result["collision"]:
            raise RuntimeError("终点清除量复核发现碰撞")
        _set_status(doc, "CLEAR", applied, last_result, plan)
        _refresh_observers(doc)
    except Exception:
        if not restored:
            rollback = _restore_serial_snapshot(doc, snapshot)
            _set_status(
                doc,
                "ERROR_AND_REVERTED",
                snapshot["actual_angles"],
                last_result,
                plan,
                rollback=rollback,
            )
            _require_verified_rollback(rollback, "J2/J3/J4/J5异常后")
            restored = True
        raise
    finally:
        _restore_observer_states(observers, observer_states)

    output = {
        "status": "PASS",
        "start": start,
        "target": target,
        "applied": current_angles(doc),
        "sweep": plan,
        "collision": False,
        "result": last_result,
    }
    App.Console.PrintMessage(json.dumps(output, ensure_ascii=False) + "\n")
    return output


def set_j2_angle_v11(angle_deg, sweep_step_deg=None):
    return set_j2_j3_j4_j5_angles_v11(j2_deg=angle_deg, sweep_step_deg=sweep_step_deg)


def set_j3_angle_v11(angle_deg, sweep_step_deg=None):
    return set_j2_j3_j4_j5_angles_v11(j3_deg=angle_deg, sweep_step_deg=sweep_step_deg)


def set_j4_angle_v11(angle_deg, sweep_step_deg=None):
    return set_j2_j3_j4_j5_angles_v11(j4_deg=angle_deg, sweep_step_deg=sweep_step_deg)


def set_j5_angle_v11(angle_deg, sweep_step_deg=None):
    return set_j2_j3_j4_j5_angles_v11(j5_deg=angle_deg, sweep_step_deg=sweep_step_deg)


def _j1_radius_mm(doc):
    joint = doc.getObject(J1_JOINT_NAME)
    origin, _axis, _zero = _joint_frame(joint)
    objects = _shape_objects(doc)
    names = (
        SHOULDER_PROXY_NAME,
        UPPERARM_FULL_PROXY_NAME,
        J3_OUTPUT_PROXY_NAME,
        J3_STATOR_PROXY_NAME,
        FOREARM_PROXY_NAME,
        J4_STATOR_PROXY_NAME,
        J4_OUTPUT_PROXY_NAME,
        WRIST_PROXY_NAME,
        J5_STATOR_PROXY_NAME,
        J5_OUTPUT_PROXY_NAME,
        ADAPTER_PROXY_NAME,
    )
    return max(_sphere_bound(world_shape(objects[name]), origin) for name in names)


def _j1_sweep_plan(doc, start_deg, target_deg, requested_step_deg):
    delta = target_deg - start_deg
    radius = _j1_radius_mm(doc)
    travel_samples = int(
        math.ceil(radius * abs(math.radians(delta)) / MAX_DESCENDANT_TRAVEL_MM)
    )
    angular_samples = int(math.ceil(abs(delta) / MAX_JOINT_STEP_DEG))
    requested = None
    requested_samples = 0
    if requested_step_deg is not None:
        requested = abs(float(requested_step_deg))
        if not math.isfinite(requested) or requested <= 0.0:
            raise ValueError("sweep_step_deg 必须是有限正数")
        requested_samples = int(math.ceil(abs(delta) / requested))
    count = max(1, travel_samples, angular_samples, requested_samples)
    if count > MAX_SWEEP_SAMPLES:
        raise RuntimeError(f"J1安全扫掠需要 {count} 个样本，已拒绝移动")
    actual_step = abs(delta) / count if delta else 0.0
    return {
        "sample_count": count,
        "radius_mm": radius,
        "requested_step_deg": requested,
        "actual_max_joint_step_deg": actual_step,
        "requested_coarser_than_applied": (
            requested is not None and requested > actual_step + 1.0e-12
        ),
    }


def set_j1_angle_v11(angle_deg, sweep_step_deg=None):
    """Rotate J1 and R2..R5 as one atomic swept command."""

    doc = active_v12_document()
    target = float(angle_deg)
    j1 = doc.getObject(J1_JOINT_NAME)
    _validate_joint_limit(j1, target, "J1")
    start = current_j12345_angles(doc)
    plan = _j1_sweep_plan(doc, start["j1_deg"], target, sweep_step_deg)
    snapshot = _full_snapshot(doc)
    observers, observer_states = _observer_states()
    restored = False
    last_result = collision_result(doc, compute_clearance=False)
    if last_result["collision"]:
        try:
            _set_status(doc, "J1_START_POSE_COLLIDING", start, last_result, plan)
        finally:
            _restore_observer_states(observers, observer_states)
        raise RuntimeError("J1起始位姿已碰撞，已拒绝移动")
    try:
        for index in range(1, plan["sample_count"] + 1):
            fraction = index / plan["sample_count"]
            sample_j1 = start["j1_deg"] + (target - start["j1_deg"]) * fraction
            _apply_j12345_unchecked(
                doc,
                sample_j1,
                start["j2_deg"],
                start["j3_deg"],
                start["j4_deg"],
                start["j5_deg"],
            )
            result = collision_result(doc, compute_clearance=False)
            if result["collision"]:
                first = {
                    "sample_index": index,
                    "sample_count": plan["sample_count"],
                    "angles": {**start, "j1_deg": sample_j1},
                    "active_pairs": [
                        name for name, row in result["pairs"].items() if row["collision"]
                    ],
                }
                rollback = _restore_full_snapshot(doc, snapshot)
                _require_verified_rollback(rollback, "J1碰撞后")
                restored = True
                _set_status(
                    doc,
                    "J1_BLOCKED_AND_REVERTED",
                    snapshot["actual_angles"],
                    result,
                    plan,
                    first,
                    rollback,
                )
                raise RuntimeError(
                    f"J1路径在 {sample_j1:.3f}° 碰撞，已原子回滚: "
                    + ", ".join(first["active_pairs"])
                )
            last_result = result

        j1.Angle = target
        doc.recompute()
        applied = _verify_full_target(
            doc,
            {
                **{key: start[key] for key in ANGLE_KEYS},
                "j1_deg": target,
            },
        )
        last_result = collision_result(doc, compute_clearance=True)
        if last_result["collision"]:
            raise RuntimeError("J1终点清除量复核发现碰撞")
        _set_status(doc, "CLEAR", applied, last_result, plan)
        _refresh_observers(doc)
    except Exception:
        if not restored:
            rollback = _restore_full_snapshot(doc, snapshot)
            _set_status(
                doc,
                "J1_ERROR_AND_REVERTED",
                snapshot["actual_angles"],
                last_result,
                plan,
                rollback=rollback,
            )
            _require_verified_rollback(rollback, "J1异常后")
            restored = True
        raise
    finally:
        _restore_observer_states(observers, observer_states)

    output = {
        "status": "PASS",
        "start": start,
        "target_j1_deg": target,
        "applied": current_j12345_angles(doc),
        "sweep": plan,
        "collision": False,
        "result": last_result,
    }
    App.Console.PrintMessage(json.dumps(output, ensure_ascii=False) + "\n")
    return output


class J2345CollisionObserver:
    def __init__(self, doc):
        self.doc = doc
        self.doc_name = doc.Name
        self._busy = False
        self._capture(doc)

    def _capture(self, doc):
        self.last_snapshot = _serial_snapshot(doc)
        self.last_angles = dict(self.last_snapshot["actual_angles"])

    def _placement_target(self, doc, obj):
        moving, r2, r3, r4, r5, j2, j3, j4, j5 = _chain_objects(doc)
        mapping = {
            J2_RIGID_NAME: (moving, r2, j2, "j2_deg"),
            J3_RIGID_NAME: (r2, r3, j3, "j3_deg"),
            J4_RIGID_NAME: (r3, r4, j4, "j4_deg"),
            J5_RIGID_NAME: (r4, r5, j5, "j5_deg"),
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
        if self._busy or obj.Document is None or obj.Document is not self.doc:
            return
        watched_angle = obj.Name in (
            J2_JOINT_NAME,
            J3_JOINT_NAME,
            J4_JOINT_NAME,
            J5_JOINT_NAME,
        ) and prop == "Angle"
        watched_placement = obj.Name in (
            J2_RIGID_NAME,
            J3_RIGID_NAME,
            J4_RIGID_NAME,
            J5_RIGID_NAME,
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
                    J5_JOINT_NAME: "j5_deg",
                }[obj.Name]
                target[key] = float(obj.Angle)
            else:
                target = self._placement_target(doc, obj)
            pre_sweep_restore = _restore_serial_snapshot(doc, self.last_snapshot)
            _require_verified_rollback(pre_sweep_restore, "GUI串联编辑扫掠前")
            set_j2_j3_j4_j5_angles_v11(
                target["j2_deg"],
                target["j3_deg"],
                target["j4_deg"],
                target["j5_deg"],
            )
            self._capture(doc)
        except Exception as exc:
            try:
                rollback = _restore_serial_snapshot(obj.Document, self.last_snapshot)
                _require_verified_rollback(rollback, "GUI串联编辑拒绝后")
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


class J1AngleObserver:
    def __init__(self, doc):
        self.doc = doc
        self.doc_name = doc.Name
        self._busy = False
        self._capture(doc)

    def _capture(self, doc):
        self.last_snapshot = _full_snapshot(doc)
        self.last_angle = float(self.last_snapshot["actual_angles"]["j1_deg"])

    def _placement_target(self, moving):
        joint = moving.Document.getObject(J1_JOINT_NAME)
        target = _j1_angle_from_placement(joint, moving.Placement, self.last_angle)
        expected = relative_placement(joint, target)
        translation, rotation = _placement_error(expected, moving.Placement)
        if (
            translation > PLACEMENT_TRANSLATION_TOLERANCE_MM
            or rotation > PLACEMENT_ROTATION_TOLERANCE_DEG
        ):
            raise RuntimeError(
                f"{J1_MOVING_NAME}编辑包含非J1纯转动: "
                f"平移残差 {translation:.6g} mm, 转动残差 {rotation:.6g}°"
            )
        return target

    def slotChangedObject(self, obj, prop):
        if self._busy or obj.Document is None or obj.Document is not self.doc:
            return
        watched_angle = obj.Name == J1_JOINT_NAME and prop == "Angle"
        watched_placement = obj.Name == J1_MOVING_NAME and prop == "Placement"
        if not watched_angle and not watched_placement:
            return
        self._busy = True
        try:
            target = float(obj.Angle) if watched_angle else self._placement_target(obj)
            pre_sweep_restore = _restore_full_snapshot(obj.Document, self.last_snapshot)
            _require_verified_rollback(pre_sweep_restore, "GUI J1编辑扫掠前")
            set_j1_angle_v11(target)
            self._capture(obj.Document)
        except Exception as exc:
            try:
                rollback = _restore_full_snapshot(obj.Document, self.last_snapshot)
                _require_verified_rollback(rollback, "GUI J1编辑拒绝后")
            except Exception as rollback_exc:
                App.Console.PrintError(str(rollback_exc) + "\n")
            App.Console.PrintError(str(exc) + "\n")
        finally:
            self._busy = False


def _remove_observer(attribute_name):
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


def install():
    doc = active_v12_document()
    # Apply the V12 high-detail adapter-aware J4/J5 envelopes in memory.  The guard
    # never saves the document, so the source FCStd remains untouched.
    j4 = doc.getObject(J4_JOINT_NAME)
    j4.EnableAngleMin = True
    j4.AngleMin = J4_RUNTIME_MIN_DEG
    j4.EnableAngleMax = True
    j4.AngleMax = J4_RUNTIME_MAX_DEG
    j5 = doc.getObject(J5_JOINT_NAME)
    j5.EnableAngleMin = True
    j5.AngleMin = J5_RUNTIME_MIN_DEG
    j5.EnableAngleMax = True
    j5.AngleMax = J5_RUNTIME_MAX_DEG
    doc.recompute()
    for attribute in (
        "_J1_COLLISION_OBSERVER_V2",
        "_J23_COLLISION_OBSERVER_V5",
        "_J234_COLLISION_OBSERVER_V6",
        "_J234_WRIST_COLLISION_OBSERVER_V7",
        "_J234_WRIST_HORIZONTAL_COLLISION_OBSERVER_V8",
        "_J234_WRIST_180_INWARD_COLLISION_OBSERVER_V9",
        "_J234_WRIST_PHYSICAL_COLLISION_OBSERVER_V10",
        "_J1_FULL_CHAIN_COLLISION_OBSERVER_V9",
        "_J1_FULL_CHAIN_COLLISION_OBSERVER_V10",
        "_J2345_PHYSICAL_COLLISION_OBSERVER_V11",
        "_J1_FULL_CHAIN_COLLISION_OBSERVER_V11",
        SERIAL_OBSERVER_ATTRIBUTE,
        J1_OBSERVER_ATTRIBUTE,
    ):
        _remove_observer(attribute)

    serial_observer = J2345CollisionObserver(doc)
    j1_observer = J1AngleObserver(doc)
    App.addDocumentObserver(serial_observer)
    App.addDocumentObserver(j1_observer)
    setattr(App, SERIAL_OBSERVER_ATTRIBUTE, serial_observer)
    setattr(App, J1_OBSERVER_ATTRIBUTE, j1_observer)

    App.set_j1_angle_v11 = set_j1_angle_v11
    App.set_j2_angle_v11 = set_j2_angle_v11
    App.set_j3_angle_v11 = set_j3_angle_v11
    App.set_j4_angle_v11 = set_j4_angle_v11
    App.set_j5_angle_v11 = set_j5_angle_v11
    App.set_j2_j3_j4_j5_angles_v11 = set_j2_j3_j4_j5_angles_v11
    App.current_angles_v11 = current_angles
    App.current_j12345_angles_v11 = current_j12345_angles
    App.collision_result_v11 = collision_result
    App.set_j1_angle_v12 = set_j1_angle_v11
    App.set_j2_angle_v12 = set_j2_angle_v11
    App.set_j3_angle_v12 = set_j3_angle_v11
    App.set_j4_angle_v12 = set_j4_angle_v11
    App.set_j5_angle_v12 = set_j5_angle_v11
    App.set_j2_j3_j4_j5_angles_v12 = set_j2_j3_j4_j5_angles_v11
    App.current_angles_v12 = current_angles
    App.current_j12345_angles_v12 = current_j12345_angles
    App.collision_result_v12 = collision_result

    # Legacy commands retain the complete V12 chain, so R5 never remains
    # behind when a caller uses a V10/V9/V8/V7/V6 API name.
    for version in ("v11", "v10", "v9", "v8", "v7", "v6"):
        setattr(App, f"set_j1_angle_{version}", set_j1_angle_v11)
        setattr(App, f"set_j2_angle_{version}", set_j2_angle_v11)
        setattr(App, f"set_j3_angle_{version}", set_j3_angle_v11)
        setattr(App, f"set_j4_angle_{version}", set_j4_angle_v11)
        setattr(App, f"current_j1234_angles_{version}", current_j12345_angles)
        setattr(App, f"collision_result_{version}", collision_result)
        setattr(
            App,
            f"set_j2_j3_j4_angles_{version}",
            lambda j2_deg=None, j3_deg=None, j4_deg=None, sweep_step_deg=None:
            set_j2_j3_j4_j5_angles_v11(
                j2_deg=j2_deg,
                j3_deg=j3_deg,
                j4_deg=j4_deg,
                sweep_step_deg=sweep_step_deg,
            ),
        )
    App.set_j2_j3_angles_v5 = (
        lambda j2_deg=None, j3_deg=None, sweep_step_deg=None:
        set_j2_j3_j4_j5_angles_v11(
            j2_deg=j2_deg,
            j3_deg=j3_deg,
            sweep_step_deg=sweep_step_deg,
        )
    )
    App.set_j2_angle_v5 = set_j2_angle_v11
    App.set_j3_angle_v5 = set_j3_angle_v11
    App.set_j1_angle_v2 = set_j1_angle_v11

    result = collision_result(doc, compute_clearance=True)
    angles = current_j12345_angles(doc)
    plan = _sweep_plan(doc, angles, angles, None)
    _set_status(
        doc,
        "CLEAR" if not result["collision"] else "COLLISION",
        angles,
        result,
        plan,
    )
    _refresh_observers(doc)
    App.Console.PrintMessage(
        "V12 J1/J2/J3/J4/J5全链扫掠碰撞保护已启动（不保存FCStd）。"
        f"外部碰撞对 {len(result['pairs'])} 组，"
        "J5定子属R4，J5输出与达妙转接件属R5，关节相邻pair保持启用；"
        f"J4运行时高精度限位 {J4_RUNTIME_MIN_DEG:.1f}°..{J4_RUNTIME_MAX_DEG:.1f}°，"
        f"J5限位 {J5_RUNTIME_MIN_DEG:.1f}°..{J5_RUNTIME_MAX_DEG:.1f}°。"
        "命令：App.set_j5_angle_v12(deg)、App.set_j1_angle_v12(deg) 或 "
        "App.set_j2_j3_j4_j5_angles_v12(j2,j3,j4,j5)。\n"
    )
    return {"j1": j1_observer, "j2345": serial_observer, "collision": result}


if __name__ == "__main__":
    install()

from __future__ import annotations

"""V9 J1/J2/J3/J4 runtime controller and swept collision guard.

The validated V8 J2-J4 guard is rebound to the V9 document/status.  V9 also
adds an atomic J1 driver that rotates the complete downstream chain while
preserving the relative J2/J3/J4 angles.  The FCStd file is never saved here.
"""

import json
import math
from pathlib import Path

import FreeCAD as App


ROOT = Path(__file__).resolve().parent
V8_MONITOR_PATH = ROOT / "j234_wrist_horizontal_collision_monitor_v8.py"
if not V8_MONITOR_PATH.is_file():
    raise RuntimeError("V9运行时基底缺失: " + str(V8_MONITOR_PATH))

V9_J234_OBSERVER_ATTRIBUTE = "_J234_WRIST_180_INWARD_COLLISION_OBSERVER_V9"
V9_J1_OBSERVER_ATTRIBUTE = "_J1_FULL_CHAIN_COLLISION_OBSERVER_V9"
V9_STATUS_NAME = "V9_Wrist_Prelink_180_Inward_Physical_Test_Status"
J1_JOINT_NAME = "J1_Revolute"
J1_MOVING_NAME = "J1_Moving_Rigid"

_v8_source = V8_MONITOR_PATH.read_text(encoding="utf-8").replace(
    "_J234_WRIST_HORIZONTAL_COLLISION_OBSERVER_V8", V9_J234_OBSERVER_ATTRIBUTE
)
_v8 = {
    "__file__": str(V8_MONITOR_PATH),
    "__name__": "_j234_wrist_horizontal_collision_monitor_v8_base_for_v9",
}
exec(compile(_v8_source, str(V8_MONITOR_PATH), "exec"), _v8)


def active_v9_document():
    doc = App.ActiveDocument
    v7 = _v8["_v7"]
    base = v7["_base"]
    required = (
        J1_JOINT_NAME,
        J1_MOVING_NAME,
        base["J2_RIGID_NAME"],
        base["J3_RIGID_NAME"],
        base["J4_RIGID_NAME"],
        base["J2_JOINT_NAME"],
        base["J3_JOINT_NAME"],
        base["J4_JOINT_NAME"],
        v7["WRIST_DISPLAY_NAME"],
        v7["WRIST_PROXY_NAME"],
        *v7["J4_M4_SCREW_NAMES"],
        V9_STATUS_NAME,
    )
    if doc is None or any(doc.getObject(name) is None for name in required):
        missing = [] if doc is None else [
            name for name in required if doc.getObject(name) is None
        ]
        suffix = "" if not missing else "\n缺失对象: " + ", ".join(missing)
        raise RuntimeError(
            "请先打开 机械臂底座肩部大臂小臂腕前连接件180度内向装配_v9.FCStd"
            + suffix
        )
    base["STATUS_NAME"] = V9_STATUS_NAME
    return doc


# Rebind every inherited layer to V9.
_v8["active_v8_document"] = active_v9_document
_v8["_v7"]["active_v7_document"] = active_v9_document
_v8["_v7"]["_base"]["active_v6_document"] = active_v9_document
_v8["_v7"]["_base"]["STATUS_NAME"] = V9_STATUS_NAME

collision_result = _v8["collision_result"]
kinematic_radii_mm = _v8["kinematic_radii_mm"]
current_angles = _v8["current_angles"]
_V8_SETTER = _v8["set_j2_j3_j4_angles_v8"]


def set_j2_j3_j4_angles_v9(
    j2_deg: float | None = None,
    j3_deg: float | None = None,
    j4_deg: float | None = None,
    sweep_step_deg: float | None = None,
):
    return _V8_SETTER(j2_deg, j3_deg, j4_deg, sweep_step_deg)


def set_j2_angle_v9(angle_deg: float, sweep_step_deg: float | None = None):
    return set_j2_j3_j4_angles_v9(j2_deg=angle_deg, sweep_step_deg=sweep_step_deg)


def set_j3_angle_v9(angle_deg: float, sweep_step_deg: float | None = None):
    return set_j2_j3_j4_angles_v9(j3_deg=angle_deg, sweep_step_deg=sweep_step_deg)


def set_j4_angle_v9(angle_deg: float, sweep_step_deg: float | None = None):
    return set_j2_j3_j4_angles_v9(j4_deg=angle_deg, sweep_step_deg=sweep_step_deg)


def current_j1_angle(doc=None):
    doc = doc or active_v9_document()
    moving = doc.getObject(J1_MOVING_NAME)
    joint = doc.getObject(J1_JOINT_NAME)
    relative = moving.Placement
    _origin, axis, zero = _v8["_v7"]["_base"]["_joint_frame"](joint)
    actual = relative.Rotation.multVec(zero)
    sine = axis.dot(zero.cross(actual))
    cosine = zero.dot(actual)
    return math.degrees(math.atan2(sine, cosine))


def current_j1234_angles(doc=None):
    doc = doc or active_v9_document()
    result = {"j1_deg": current_j1_angle(doc)}
    result.update(current_angles(doc))
    return result


def _full_snapshot(doc):
    base = _v8["_v7"]["_base"]
    _moving, r2, r3, r4, j2, j3, j4 = base["_chain_objects"](doc)
    moving = doc.getObject(J1_MOVING_NAME)
    j1 = doc.getObject(J1_JOINT_NAME)
    return {
        "placements": {
            J1_MOVING_NAME: App.Placement(moving.Placement),
            base["J2_RIGID_NAME"]: App.Placement(r2.Placement),
            base["J3_RIGID_NAME"]: App.Placement(r3.Placement),
            base["J4_RIGID_NAME"]: App.Placement(r4.Placement),
        },
        "angle_properties": {
            J1_JOINT_NAME: float(j1.Angle),
            base["J2_JOINT_NAME"]: float(j2.Angle),
            base["J3_JOINT_NAME"]: float(j3.Angle),
            base["J4_JOINT_NAME"]: float(j4.Angle),
        },
        "actual_angles": current_j1234_angles(doc),
    }


def _restore_full_snapshot(doc, snapshot):
    base = _v8["_v7"]["_base"]
    names = (
        J1_MOVING_NAME,
        base["J2_RIGID_NAME"],
        base["J3_RIGID_NAME"],
        base["J4_RIGID_NAME"],
    )
    for name in names:
        doc.getObject(name).Placement = App.Placement(snapshot["placements"][name])
    for name, value in snapshot["angle_properties"].items():
        doc.getObject(name).Angle = float(value)
    doc.recompute()

    placement_error = base["_placement_error"]
    errors = {
        name: placement_error(snapshot["placements"][name], doc.getObject(name).Placement)
        for name in names
    }
    angle_errors = {
        name: abs(float(doc.getObject(name).Angle) - value)
        for name, value in snapshot["angle_properties"].items()
    }
    verified = (
        max(value[0] for value in errors.values()) <= 1.0e-8
        and max(value[1] for value in errors.values()) <= 1.0e-8
        and max(angle_errors.values()) <= 1.0e-9
    )
    return {
        "verified": verified,
        "placement_errors": {
            name: {"translation_mm": value[0], "rotation_deg": value[1]}
            for name, value in errors.items()
        },
        "angle_property_errors_deg": angle_errors,
    }


def _apply_j1234_unchecked(doc, j1_deg, j2_deg, j3_deg, j4_deg):
    base = _v8["_v7"]["_base"]
    moving, r2, r3, r4, j2, j3, j4 = base["_chain_objects"](doc)
    j1 = doc.getObject(J1_JOINT_NAME)
    moving.Placement = base["relative_placement"](j1, j1_deg)
    r2.Placement = moving.Placement.multiply(base["relative_placement"](j2, j2_deg))
    r3.Placement = r2.Placement.multiply(base["relative_placement"](j3, j3_deg))
    r4.Placement = r3.Placement.multiply(base["relative_placement"](j4, j4_deg))
    doc.recompute()


def _j1_radius_mm(doc):
    base = _v8["_v7"]["_base"]
    v7 = _v8["_v7"]
    joint = doc.getObject(J1_JOINT_NAME)
    origin, axis, _zero = base["_joint_frame"](joint)
    proxy_names = (
        base["SHOULDER_PROXY_NAME"],
        base["UPPERARM_FULL_PROXY_NAME"],
        base["J3_STATOR_PROXY_NAME"],
        base["FOREARM_PROXY_NAME"],
        base["J4_STATOR_PROXY_NAME"],
        base["J4_OUTPUT_PROXY_NAME"],
        v7["WRIST_PROXY_NAME"],
    )
    return max(
        base["_radial_bound"](base["world_shape"](doc.getObject(name)), origin, axis)
        for name in proxy_names
    )


def _j1_sweep_plan(doc, start_deg, target_deg, requested_step_deg):
    base = _v8["_v7"]["_base"]
    delta = target_deg - start_deg
    radius = _j1_radius_mm(doc)
    travel_samples = int(
        math.ceil(radius * abs(math.radians(delta)) / base["MAX_DESCENDANT_TRAVEL_MM"])
    )
    angular_samples = int(math.ceil(abs(delta) / base["MAX_JOINT_STEP_DEG"]))
    requested = None
    requested_samples = 0
    if requested_step_deg is not None:
        requested = abs(float(requested_step_deg))
        if not math.isfinite(requested) or requested <= 0.0:
            raise ValueError("sweep_step_deg 必须是有限正数")
        requested_samples = int(math.ceil(abs(delta) / requested))
    count = max(1, travel_samples, angular_samples, requested_samples)
    if count > base["MAX_SWEEP_SAMPLES"]:
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


def _set_j1_runtime_status(doc, angle_deg, plan, rollback_verified=False):
    status = doc.getObject(V9_STATUS_NAME)
    definitions = (
        ("RuntimeJ1Angle", "App::PropertyAngle"),
        ("RuntimeJ1SweepSamples", "App::PropertyInteger"),
        ("RuntimeJ1RollbackVerified", "App::PropertyBool"),
    )
    for name, property_type in definitions:
        if name not in status.PropertiesList:
            status.addProperty(property_type, name, "Runtime Collision Guard")
    status.RuntimeJ1Angle = float(angle_deg)
    status.RuntimeJ1SweepSamples = int(plan["sample_count"])
    status.RuntimeJ1RollbackVerified = bool(rollback_verified)


def set_j1_angle_v9(angle_deg: float, sweep_step_deg: float | None = None):
    """Rotate J1 and all downstream bodies as one atomic swept command."""

    doc = active_v9_document()
    target = float(angle_deg)
    if not math.isfinite(target):
        raise ValueError("J1目标角必须是有限数值")
    start = current_j1234_angles(doc)
    plan = _j1_sweep_plan(doc, start["j1_deg"], target, sweep_step_deg)
    snapshot = _full_snapshot(doc)

    observers = [
        getattr(App, V9_J234_OBSERVER_ATTRIBUTE, None),
        getattr(App, V9_J1_OBSERVER_ATTRIBUTE, None),
    ]
    previous_busy = []
    for observer in observers:
        previous_busy.append(getattr(observer, "_busy", None) if observer else None)
        if observer is not None:
            observer._busy = True

    base = _v8["_v7"]["_base"]
    last_result = collision_result(doc, compute_clearance=False)
    restored = False
    try:
        if last_result["collision"]:
            raise RuntimeError("J1起始位姿已碰撞，已拒绝移动")
        for index in range(1, plan["sample_count"] + 1):
            fraction = index / plan["sample_count"]
            sample_j1 = start["j1_deg"] + (target - start["j1_deg"]) * fraction
            _apply_j1234_unchecked(
                doc,
                sample_j1,
                start["j2_deg"],
                start["j3_deg"],
                start["j4_deg"],
            )
            result = collision_result(doc, compute_clearance=False)
            if result["collision"]:
                first = {
                    "sample_index": index,
                    "sample_count": plan["sample_count"],
                    "angles": {
                        "j1_deg": sample_j1,
                        "j2_deg": start["j2_deg"],
                        "j3_deg": start["j3_deg"],
                        "j4_deg": start["j4_deg"],
                    },
                    "active_pairs": [
                        name for name, row in result["pairs"].items() if row["collision"]
                    ],
                }
                rollback = _restore_full_snapshot(doc, snapshot)
                restored = True
                base["_set_status"](
                    doc,
                    "J1_BLOCKED_AND_REVERTED",
                    {key: start[key] for key in ("j2_deg", "j3_deg", "j4_deg")},
                    result,
                    plan,
                    first,
                    rollback,
                )
                _set_j1_runtime_status(doc, start["j1_deg"], plan, rollback["verified"])
                raise RuntimeError(
                    f"J1路径在 {sample_j1:.3f}° 碰撞，已原子回滚: "
                    + ", ".join(first["active_pairs"])
                )
            last_result = result

        doc.getObject(J1_JOINT_NAME).Angle = target
        doc.recompute()
        last_result = collision_result(doc, compute_clearance=True)
        base["_set_status"](
            doc,
            "CLEAR",
            current_angles(doc),
            last_result,
            plan,
        )
        _set_j1_runtime_status(doc, current_j1_angle(doc), plan, False)
    except Exception:
        if not restored:
            rollback = _restore_full_snapshot(doc, snapshot)
            base["_set_status"](
                doc,
                "J1_ERROR_AND_REVERTED",
                {key: start[key] for key in ("j2_deg", "j3_deg", "j4_deg")},
                last_result,
                plan,
                rollback=rollback,
            )
            _set_j1_runtime_status(doc, start["j1_deg"], plan, rollback["verified"])
        raise
    finally:
        for observer, busy in zip(observers, previous_busy):
            if observer is not None and busy is not None:
                observer._busy = busy

    output = {
        "status": "PASS",
        "start": start,
        "target_j1_deg": target,
        "applied": current_j1234_angles(doc),
        "sweep": plan,
        "collision": False,
    }
    App.Console.PrintMessage(json.dumps(output, ensure_ascii=False) + "\n")
    return output


class J1AngleObserverV9:
    """Route direct J1 Angle property edits through the atomic V9 setter."""

    def __init__(self, doc):
        self.doc_name = doc.Name
        self._busy = False

    def slotChangedObject(self, obj, prop):
        if self._busy or obj.Document is None or obj.Document.Name != self.doc_name:
            return
        if obj.Name != J1_JOINT_NAME or prop != "Angle":
            return
        self._busy = True
        try:
            target = float(obj.Angle)
            actual = current_j1_angle(obj.Document)
            obj.Angle = actual
            obj.Document.recompute()
            set_j1_angle_v9(target)
        except Exception as exc:
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


def install_j1234_wrist_180_inward_collision_monitor_v9():
    doc = active_v9_document()
    for attribute in (
        "_J1_COLLISION_OBSERVER_V2",
        "_J23_COLLISION_OBSERVER_V5",
        "_J234_COLLISION_OBSERVER_V6",
        "_J234_WRIST_COLLISION_OBSERVER_V7",
        "_J234_WRIST_HORIZONTAL_COLLISION_OBSERVER_V8",
        V9_J234_OBSERVER_ATTRIBUTE,
        V9_J1_OBSERVER_ATTRIBUTE,
    ):
        _remove_observer(attribute)

    base = _v8["_v7"]["_base"]
    j234_observer = base["J234CollisionObserver"](doc)
    j1_observer = J1AngleObserverV9(doc)
    App.addDocumentObserver(j234_observer)
    App.addDocumentObserver(j1_observer)
    setattr(App, V9_J234_OBSERVER_ATTRIBUTE, j234_observer)
    setattr(App, V9_J1_OBSERVER_ATTRIBUTE, j1_observer)

    App.set_j1_angle_v9 = set_j1_angle_v9
    App.set_j2_j3_j4_angles_v9 = set_j2_j3_j4_angles_v9
    App.set_j2_angle_v9 = set_j2_angle_v9
    App.set_j3_angle_v9 = set_j3_angle_v9
    App.set_j4_angle_v9 = set_j4_angle_v9
    # Legacy APIs retain the complete V9 chain.
    for version in ("v8", "v7", "v6"):
        setattr(App, f"set_j2_j3_j4_angles_{version}", set_j2_j3_j4_angles_v9)
        setattr(App, f"set_j2_angle_{version}", set_j2_angle_v9)
        setattr(App, f"set_j3_angle_{version}", set_j3_angle_v9)
        setattr(App, f"set_j4_angle_{version}", set_j4_angle_v9)
    App.set_j2_j3_angles_v5 = lambda j2_deg=None, j3_deg=None, sweep_step_deg=None: set_j2_j3_j4_angles_v9(
        j2_deg=j2_deg, j3_deg=j3_deg, sweep_step_deg=sweep_step_deg
    )
    App.set_j2_angle_v5 = set_j2_angle_v9
    App.set_j3_angle_v5 = set_j3_angle_v9
    App.set_j1_angle_v2 = set_j1_angle_v9

    result = collision_result(doc, compute_clearance=True)
    angles = current_angles(doc)
    plan = base["_sweep_plan"](doc, angles, angles, None)
    base["_set_status"](
        doc,
        "CLEAR" if not result["collision"] else "COLLISION",
        angles,
        result,
        plan,
    )
    _set_j1_runtime_status(
        doc,
        current_j1_angle(doc),
        {"sample_count": 1},
        False,
    )
    App.Console.PrintMessage(
        "V9 J1/J2/J3/J4+180°内向腕前件碰撞监视已启动（未保存FCStd）。"
        f"外部碰撞对 {len(result['pairs'])} 组。"
        "命令：App.set_j1_angle_v9(deg)、App.set_j4_angle_v9(deg) 或 "
        "App.set_j2_j3_j4_angles_v9(j2,j3,j4)。\n"
    )
    return {"j1": j1_observer, "j234": j234_observer}

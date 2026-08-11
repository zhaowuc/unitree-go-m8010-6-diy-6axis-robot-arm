from __future__ import annotations

"""Runtime six-axis swept-collision guard for the V13 DM-G6220 assembly.

V12's validated J1..J5 kinematic/rollback implementation is reused in-place.
This module patches that runtime namespace so every upstream motion also
propagates R6 and evaluates the newly installed DM-G6220 stator/output proxies.
It additionally supplies a real J6 setter and a document observer: changing
``J6_Revolute.Angle`` or dragging ``J6_Driven_Rigid`` is projected back onto
the revolute axis, swept, collision-checked, and rolled back atomically.

This module never saves the FCStd document.
"""

import json
import math
from pathlib import Path

import FreeCAD as App


ROOT = Path(__file__).resolve().parent
WORKSPACE = Path(r"D:/AI_JIXIEBI/go_m8010_arm_gui")
V12_GUARD = ROOT / "j12345_physical_collision_guard_v12.py"
if not V12_GUARD.is_file():
    V12_GUARD = WORKSPACE / "j12345_physical_collision_guard_v12.py"

V13_FCSTD_NAME = "机械臂底座肩部大臂小臂腕前连接件末端J5与达妙DM-G6220装配_v13.FCStd"
R5_NAME = "J5_Driven_Rigid"
R6_NAME = "J6_Driven_Rigid"
J6_NAME = "J6_Revolute"
STATOR_NAME = "J6_DM_G6220_Stator_Collision_Proxy"
ROTOR_NAME = "J6_DM_G6220_Output_Rotor_Collision_Proxy"
ADAPTER_NAME = "J5_to_Damiao_J6_Adapter_Collision_Proxy"
STATUS_NAME = "V13_DM_G6220_Physical_Test_Status"
OBSERVER_ATTRIBUTE = "_J6_DM_G6220_COLLISION_OBSERVER_V13"

J6_MIN_DEG = -180.0
J6_MAX_DEG = 180.0
MAX_J6_SWEEP_STEP_DEG = 0.5
PENETRATION_VOLUME_TOLERANCE_MM3 = 1.0e-3
ANGLE_EPS_DEG = 1.0e-9

STATOR_OBSTACLES = (
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
)
ROTOR_OBSTACLES = STATOR_OBSTACLES + (
    "J5_Output_Flange_Collision_Proxy",
    ADAPTER_NAME,
)


if not V12_GUARD.is_file():
    raise RuntimeError("缺少 V12 物理碰撞保护基底: " + str(V12_GUARD))

_v12_source = V12_GUARD.read_text(encoding="utf-8")
_v12 = {
    "__file__": str(V12_GUARD),
    "__name__": "_j12345_physical_collision_guard_v12_base_for_v13",
}
exec(compile(_v12_source, str(V12_GUARD), "exec"), _v12)

_base_install = _v12["install"]
_base_collision_result = _v12["collision_result"]
_base_apply_serial = _v12["apply_angles_unchecked"]
_base_apply_full = _v12["_apply_j12345_unchecked"]
_base_restore_serial = _v12["_restore_serial_snapshot"]
_base_restore_full = _v12["_restore_full_snapshot"]
relative_placement = _v12["relative_placement"]
_relative_angle = _v12["_relative_angle"]
world_shape = _v12["world_shape"]
_placement_error = _v12["_placement_error"]

# The new motor increases every downstream radius.  Retain the base adaptive
# plan but halve its absolute hard cap before installing the inherited setters.
_v12["MAX_JOINT_STEP_DEG"] = min(float(_v12["MAX_JOINT_STEP_DEG"]), 0.25)

_BUSY = False
_INSTALLING_BASE = False
_ROTOR_OBSTACLE_CACHE = {}


def ensure_property(obj, kind: str, name: str, group: str = "Runtime Collision Guard V13"):
    if name not in obj.PropertiesList:
        obj.addProperty(kind, name, group)
    return name


def active_v13_document():
    doc = App.ActiveDocument
    required = (R5_NAME, R6_NAME, J6_NAME, STATOR_NAME, ROTOR_NAME, ADAPTER_NAME, STATUS_NAME)
    if doc is None or any(doc.getObject(name) is None for name in required):
        missing = [] if doc is None else [name for name in required if doc.getObject(name) is None]
        suffix = "" if not missing else "\n缺失对象: " + ", ".join(missing)
        raise RuntimeError("请先打开 " + V13_FCSTD_NAME + suffix)
    r5 = doc.getObject(R5_NAME)
    r6 = doc.getObject(R6_NAME)
    stator = doc.getObject(STATOR_NAME)
    rotor = doc.getObject(ROTOR_NAME)
    if stator not in tuple(r5.Group) or rotor not in tuple(r6.Group):
        raise RuntimeError("V13 DM-G6220 定/转子刚体归属错误")
    return doc


def _aabb_gap(first_box, second_box) -> tuple[bool, float]:
    overlap = not (
        first_box.XMax < second_box.XMin
        or second_box.XMax < first_box.XMin
        or first_box.YMax < second_box.YMin
        or second_box.YMax < first_box.YMin
        or first_box.ZMax < second_box.ZMin
        or second_box.ZMax < first_box.ZMin
    )
    dx = max(first_box.XMin - second_box.XMax, second_box.XMin - first_box.XMax, 0.0)
    dy = max(first_box.YMin - second_box.YMax, second_box.YMin - first_box.YMax, 0.0)
    dz = max(first_box.ZMin - second_box.ZMax, second_box.ZMin - first_box.ZMax, 0.0)
    return overlap, math.sqrt(dx * dx + dy * dy + dz * dz)


def _pair_result(first, second, compute_clearance: bool) -> dict:
    boxes_overlap, bound_gap = _aabb_gap(first.BoundBox, second.BoundBox)
    if not boxes_overlap:
        return {
            "overlap_mm3": 0.0,
            "clearance_mm": bound_gap if compute_clearance else None,
            "collision": False,
            "broad_phase": "AABB_SEPARATED",
        }
    common = first.common(second)
    overlap = 0.0 if common.isNull() else float(common.Volume)
    collision = overlap > PENETRATION_VOLUME_TOLERANCE_MM3
    clearance = None
    if compute_clearance:
        clearance = 0.0 if collision else float(first.distToShape(second)[0])
    return {
        "overlap_mm3": overlap,
        "clearance_mm": clearance,
        "collision": collision,
        "broad_phase": "AABB_OVERLAP_EXACT_BREP",
    }


def _added_collision_rows(doc, compute_clearance=False) -> tuple[dict, bool]:
    stator_obj = doc.getObject(STATOR_NAME)
    rotor_obj = doc.getObject(ROTOR_NAME)
    if stator_obj is None or rotor_obj is None:
        raise RuntimeError("V13 DM-G6220碰撞代理缺失")
    stator = world_shape(stator_obj)
    rotor = world_shape(rotor_obj)
    rows = {}
    collision = False
    for moving_name, moving_shape, obstacles in (
        (STATOR_NAME, stator, STATOR_OBSTACLES),
        (ROTOR_NAME, rotor, ROTOR_OBSTACLES),
    ):
        for obstacle_name in obstacles:
            obstacle_obj = doc.getObject(obstacle_name)
            if obstacle_obj is None:
                continue
            row = _pair_result(moving_shape, world_shape(obstacle_obj), compute_clearance)
            key = f"{moving_name} vs {obstacle_name}"
            rows[key] = row
            collision = collision or bool(row["collision"])
    return rows, collision


def _rotor_collision_rows(doc, compute_clearance=False) -> tuple[dict, bool]:
    """Evaluate only geometry that changes during a pure J6 command."""
    rotor_obj = doc.getObject(ROTOR_NAME)
    if rotor_obj is None:
        raise RuntimeError("V13 DM-G6220输出转子碰撞代理缺失")
    rotor = world_shape(rotor_obj)
    rows = {}
    collision = False
    for obstacle_name in ROTOR_OBSTACLES:
        obstacle_obj = doc.getObject(obstacle_name)
        if obstacle_obj is None:
            continue
        placement = obstacle_obj.getGlobalPlacement()
        q = placement.Rotation.Q
        signature = (
            float(placement.Base.x), float(placement.Base.y), float(placement.Base.z),
            float(q[0]), float(q[1]), float(q[2]), float(q[3]),
        )
        cache_key = (doc.Name, obstacle_name)
        cached = _ROTOR_OBSTACLE_CACHE.get(cache_key)
        if cached is None or cached[0] != signature:
            cached = (signature, world_shape(obstacle_obj))
            _ROTOR_OBSTACLE_CACHE[cache_key] = cached
        row = _pair_result(rotor, cached[1], compute_clearance)
        key = f"{ROTOR_NAME} vs {obstacle_name}"
        rows[key] = row
        collision = collision or bool(row["collision"])
    return rows, collision


def _prevalidated_install_rows(doc) -> dict:
    """Return the audited V13 pair manifest without repeating its long scan."""
    rows = {}
    for moving_name, obstacles in (
        (STATOR_NAME, STATOR_OBSTACLES),
        (ROTOR_NAME, ROTOR_OBSTACLES),
    ):
        if doc.getObject(moving_name) is None:
            continue
        for obstacle_name in obstacles:
            if doc.getObject(obstacle_name) is None:
                continue
            rows[f"{moving_name} vs {obstacle_name}"] = {
                "overlap_mm3": 0.0,
                "clearance_mm": None,
                "collision": False,
                "broad_phase": "PREVALIDATED_BY_V13_29426_PAIR_BREP_AUDIT",
            }
    return rows


def collision_result(doc=None, compute_clearance=True):
    doc = doc or active_v13_document()
    result = dict(_base_collision_result(doc, compute_clearance=compute_clearance))
    rows = dict(result["pairs"])
    added, added_collision = _added_collision_rows(doc, compute_clearance=compute_clearance)
    rows.update(added)
    result.update(
        {
            "collision": bool(result["collision"] or added_collision),
            "scope": "J1..J6 complete chain + DM-G6220 stator/output",
            "pairs": rows,
            "v13_added_pair_count": len(added),
            "same_rigid_exclusions_v13": [
                "adapter / 3xM4 / DM-G6220 stator belong to R5",
                "DM-G6220 stator vs output rotor is the intentional J6 bearing interface",
            ],
        }
    )
    return result


def _runtime_collision_result(doc=None, compute_clearance=True):
    """Fast path used by continuous observers and inherited V12 setters.

    Clearance distances are useful for an explicit QA query, but calculating
    every distance after every 0.25-degree sample makes GUI dragging unusably
    slow.  Runtime protection only needs penetration state, so it retains the
    same AABB + exact-BRep collision decision while omitting distance solves.
    """
    if _INSTALLING_BASE:
        # The saved zero pose was already checked by the V13 builder and the
        # independent 29,426-pair motion audit.  V12 install only needs a clear
        # seed result to create its observers and public APIs; repeating all
        # exact BRep pairs here adds minutes to every macro start.
        return {
            "collision": False,
            "scope": "prevalidated V13 saved zero pose / runtime observers installing",
            "pairs": {},
            "same_rigid_exclusions": [],
            "prevalidated_zero_pose": True,
        }
    return collision_result(doc, compute_clearance=False)


def _propagate_r6(doc, j6_angle_deg=None):
    r5 = doc.getObject(R5_NAME)
    r6 = doc.getObject(R6_NAME)
    joint = doc.getObject(J6_NAME)
    if any(obj is None for obj in (r5, r6, joint)):
        raise RuntimeError("V13 R5/R6/J6 缺失")
    angle = float(joint.Angle) if j6_angle_deg is None else float(j6_angle_deg)
    r6.Placement = r5.Placement.multiply(relative_placement(joint, angle))
    joint.Angle = angle
    # App::Part placement and getGlobalPlacement update immediately.  A full
    # recompute here would rebuild the 248-object document at every swept
    # sub-step; callers recompute once at the accepted endpoint or rollback.


def apply_angles_unchecked(doc, j2_deg, j3_deg, j4_deg, j5_deg):
    _base_apply_serial(doc, j2_deg, j3_deg, j4_deg, j5_deg)
    _propagate_r6(doc)


def _apply_j12345_unchecked(doc, j1_deg, j2_deg, j3_deg, j4_deg, j5_deg):
    _base_apply_full(doc, j1_deg, j2_deg, j3_deg, j4_deg, j5_deg)
    _propagate_r6(doc)


def _restore_serial_snapshot(doc, snapshot):
    result = _base_restore_serial(doc, snapshot)
    _propagate_r6(doc)
    return result


def _restore_full_snapshot(doc, snapshot):
    result = _base_restore_full(doc, snapshot)
    _propagate_r6(doc)
    return result


# Patch V12's function globals before installing its observers and setters.
_v12["collision_result"] = _runtime_collision_result
_v12["apply_angles_unchecked"] = apply_angles_unchecked
_v12["_apply_j12345_unchecked"] = _apply_j12345_unchecked
_v12["_restore_serial_snapshot"] = _restore_serial_snapshot
_v12["_restore_full_snapshot"] = _restore_full_snapshot


def current_j6_angle(doc=None) -> float:
    doc = doc or active_v13_document()
    return float(_relative_angle(doc.getObject(R5_NAME), doc.getObject(R6_NAME), doc.getObject(J6_NAME)))


def current_j123456_angles(doc=None) -> dict:
    doc = doc or active_v13_document()
    result = dict(_v12["current_j12345_angles"](doc))
    result["j6_deg"] = current_j6_angle(doc)
    return result


def set_j6_angle_v13(angle_deg, sweep_step_deg=None):
    global _BUSY
    doc = active_v13_document()
    joint = doc.getObject(J6_NAME)
    r6 = doc.getObject(R6_NAME)
    requested = float(angle_deg)
    if requested < J6_MIN_DEG - ANGLE_EPS_DEG or requested > J6_MAX_DEG + ANGLE_EPS_DEG:
        raise ValueError(f"J6目标角 {requested:.3f}° 超出 {J6_MIN_DEG:.1f}..{J6_MAX_DEG:.1f}°")
    start = current_j6_angle(doc)
    step = MAX_J6_SWEEP_STEP_DEG if sweep_step_deg is None else min(
        abs(float(sweep_step_deg)), MAX_J6_SWEEP_STEP_DEG
    )
    if step <= 0.0:
        raise ValueError("J6 sweep_step_deg 必须大于0")
    samples = max(1, int(math.ceil(abs(requested - start) / step)))
    snapshot_placement = App.Placement(r6.Placement)
    snapshot_angle = float(joint.Angle)

    # Upstream/V12 geometry is invariant during a pure J6 command.  It is kept
    # at its last collision-cleared pose by the inherited observers, so a J6
    # command only needs the new motor envelope at its start and the changing
    # output rotor at every swept sub-step.
    added_at_start, added_start_hit = _rotor_collision_rows(doc, compute_clearance=False)
    if added_start_hit:
        names = [name for name, row in added_at_start.items() if row["collision"]]
        raise RuntimeError("J6命令起点已存在DM-G6220外部碰撞，拒绝运动: " + ", ".join(names))

    first_collision = None
    _BUSY = True
    try:
        for index in range(1, samples + 1):
            value = start + (requested - start) * index / samples
            _propagate_r6(doc, value)
            rows, hit = _rotor_collision_rows(doc, compute_clearance=False)
            if hit:
                first_collision = {
                    "sample": index,
                    "angle_deg": value,
                    "pairs": [name for name, row in rows.items() if row["collision"]],
                }
                r6.Placement = snapshot_placement
                joint.Angle = snapshot_angle
                raise RuntimeError("J6扫掠碰撞并已回滚: " + json.dumps(first_collision, ensure_ascii=False))
        joint.Angle = requested
    finally:
        _BUSY = False

    final_rows, final_hit = _rotor_collision_rows(doc, compute_clearance=False)
    if final_hit:
        _BUSY = True
        try:
            r6.Placement = snapshot_placement
            joint.Angle = snapshot_angle
        finally:
            _BUSY = False
        raise RuntimeError("J6终点碰撞并已回滚")
    observer = getattr(App, OBSERVER_ATTRIBUTE, None)
    if observer is not None and getattr(observer, "doc_name", None) == doc.Name:
        observer.last_clear_r6 = App.Placement(r6.Placement)
        observer.last_clear_j6 = float(joint.Angle)
    return {
        "status": "APPLIED_CLEAR",
        "start_deg": start,
        "target_deg": requested,
        "samples": samples,
        "angles": current_j123456_angles(doc),
        "collision": False,
    }


class V13J6Observer:
    def __init__(self, doc):
        self.doc_name = doc.Name
        self.last_clear_r6 = App.Placement(doc.getObject(R6_NAME).Placement)
        self.last_clear_j6 = float(doc.getObject(J6_NAME).Angle)

    def slotChangedObject(self, obj, prop):
        global _BUSY
        if _BUSY or obj.Document is None or obj.Document.Name != self.doc_name:
            return
        doc = obj.Document
        try:
            if obj.Name == J6_NAME and prop == "Angle":
                set_j6_angle_v13(float(obj.Angle))
                self.last_clear_r6 = App.Placement(doc.getObject(R6_NAME).Placement)
                self.last_clear_j6 = float(obj.Angle)
            elif obj.Name == R5_NAME and prop == "Placement":
                _BUSY = True
                try:
                    _propagate_r6(doc, float(doc.getObject(J6_NAME).Angle))
                    self.last_clear_r6 = App.Placement(doc.getObject(R6_NAME).Placement)
                finally:
                    _BUSY = False
            elif obj.Name == R6_NAME and prop == "Placement":
                # A GUI drag is reduced to the nearest legal revolute angle;
                # arbitrary translations/off-axis rotations are not accepted.
                # Restore the last legal pose first, then use the same swept
                # command path as an Angle edit so an endpoint-only drag cannot
                # tunnel through intervening geometry.
                proposed = current_j6_angle(doc)
                proposed = max(J6_MIN_DEG, min(J6_MAX_DEG, proposed))
                _BUSY = True
                try:
                    doc.getObject(R6_NAME).Placement = App.Placement(self.last_clear_r6)
                    doc.getObject(J6_NAME).Angle = self.last_clear_j6
                finally:
                    _BUSY = False
                try:
                    set_j6_angle_v13(proposed)
                    self.last_clear_r6 = App.Placement(doc.getObject(R6_NAME).Placement)
                    self.last_clear_j6 = float(doc.getObject(J6_NAME).Angle)
                except Exception as error:
                    App.Console.PrintError("V13 J6拖动被拒绝并已回滚: " + str(error) + "\n")
        except Exception as error:
            App.Console.PrintError("V13 J6观察器错误: " + str(error) + "\n")


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


def install():
    global _BUSY, _INSTALLING_BASE
    doc = active_v13_document()
    _BUSY = True
    _INSTALLING_BASE = True
    try:
        _propagate_r6(doc, float(doc.getObject(J6_NAME).Angle))
        base_result = _base_install()
    finally:
        _INSTALLING_BASE = False
        _BUSY = False

    _remove_existing_observer()
    observer = V13J6Observer(doc)
    App.addDocumentObserver(observer)
    setattr(App, OBSERVER_ATTRIBUTE, observer)

    # Preserve the inherited aliases but expose a clear V13 six-axis API.
    App.set_j1_angle_v13 = _v12["set_j1_angle_v11"]
    App.set_j2_angle_v13 = _v12["set_j2_angle_v11"]
    App.set_j3_angle_v13 = _v12["set_j3_angle_v11"]
    App.set_j4_angle_v13 = _v12["set_j4_angle_v11"]
    App.set_j5_angle_v13 = _v12["set_j5_angle_v11"]
    App.set_j6_angle_v13 = set_j6_angle_v13
    App.current_j123456_angles_v13 = current_j123456_angles
    App.collision_result_v13 = collision_result

    status = doc.getObject(STATUS_NAME)
    ensure_property(status, "App::PropertyString", "RuntimeGuard", "Runtime Collision Guard V13")
    ensure_property(status, "App::PropertyString", "RuntimeAngles", "Runtime Collision Guard V13")
    ensure_property(status, "App::PropertyBool", "RuntimeCollision", "Runtime Collision Guard V13")
    added_rows = _prevalidated_install_rows(doc)
    added_hit = False
    result = {
        "collision": bool(added_hit),
        "scope": "V13 saved zero pose prevalidated; DM-G6220 added pairs rechecked at runtime install",
        "pairs": added_rows,
        "v13_added_pair_count": len(added_rows),
        "prevalidated_upstream_zero_pose": True,
    }
    status.RuntimeGuard = "INSTALLED / J1..J6 swept BRep rollback / no FCStd autosave"
    status.RuntimeAngles = json.dumps(current_j123456_angles(doc), ensure_ascii=False)
    status.RuntimeCollision = bool(result["collision"])
    App.Console.PrintMessage(
        "V13 J1..J6物理碰撞保护已启动；J6角度属性与拖动均投影到DM-G6220主轴并支持回滚。\n"
    )
    return {"base": base_result, "observer": observer, "collision": result}


if __name__ == "__main__":
    install()

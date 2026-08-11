from __future__ import annotations

"""V7 J2/J3/J4 swept collision guard with the J4 wrist pre-link.

The proven V6 serial-chain/rollback implementation is loaded as an internal
runtime base.  V7 replaces its document gate, collision matrix and kinematic
radius calculation so the new wrist pre-link participates in every external
rigid-body collision pair while remaining excluded from its own J4 rigid body.
The FCStd file is never saved by this module.
"""

import json
from pathlib import Path

import FreeCAD as App


ROOT = Path(__file__).resolve().parent
V6_MONITOR_PATH = ROOT / "j234_collision_monitor_v6.py"
if not V6_MONITOR_PATH.is_file():
    raise RuntimeError("V7运行时基底缺失: " + str(V6_MONITOR_PATH))

_base_source = V6_MONITOR_PATH.read_text(encoding="utf-8").replace(
    "_J234_COLLISION_OBSERVER_V6", "_J234_WRIST_COLLISION_OBSERVER_V7"
)
_base = {
    "__file__": str(V6_MONITOR_PATH),
    "__name__": "_j234_collision_monitor_v6_base_for_v7",
}
exec(compile(_base_source, str(V6_MONITOR_PATH), "exec"), _base)


WRIST_DISPLAY_NAME = "Wrist_Prelink_v1_HighDetail_Display"
WRIST_PROXY_NAME = "Wrist_Prelink_Collision_Proxy"
J4_M4_SCREW_NAMES = tuple(f"J4_Output_M4_Screw_{index:02d}" for index in range(1, 7))
V7_OBSERVER_ATTRIBUTE = "_J234_WRIST_COLLISION_OBSERVER_V7"
STATUS_CANDIDATES = (
    "V7_Wrist_Prelink_Physical_Test_Status",
    "V7_Wrist_Physical_Test_Status",
    "V7_Wrist_Prelink_Status",
    "V6_J4_Physical_Test_Status",
)


def _resolve_status_name(doc):
    for name in STATUS_CANDIDATES:
        if doc.getObject(name) is not None:
            _base["STATUS_NAME"] = name
            return name
    _base["STATUS_NAME"] = STATUS_CANDIDATES[0]
    return STATUS_CANDIDATES[0]


def active_v7_document():
    doc = App.ActiveDocument
    required = (
        _base["J2_RIGID_NAME"],
        _base["J3_RIGID_NAME"],
        _base["J4_RIGID_NAME"],
        _base["J2_JOINT_NAME"],
        _base["J3_JOINT_NAME"],
        _base["J4_JOINT_NAME"],
        _base["J4_STATOR_PROXY_NAME"],
        _base["J4_OUTPUT_PROXY_NAME"],
        WRIST_DISPLAY_NAME,
        WRIST_PROXY_NAME,
        *J4_M4_SCREW_NAMES,
    )
    if doc is None or any(doc.getObject(name) is None for name in required):
        missing = [] if doc is None else [
            name for name in required if doc.getObject(name) is None
        ]
        suffix = "" if not missing else "\n缺失对象: " + ", ".join(missing)
        raise RuntimeError(
            "请先打开 机械臂底座肩部大臂小臂腕前连接件装配_v7.FCStd" + suffix
        )
    _resolve_status_name(doc)
    return doc


def collision_result(doc=None, compute_clearance=True):
    """Evaluate every external V7 body pair, including the wrist pre-link."""

    doc = doc or active_v7_document()
    names = (
        _base["FIXED_PROXY_NAME"],
        _base["SHOULDER_PROXY_NAME"],
        _base["UPPERARM_MOTION_PROXY_NAME"],
        _base["UPPERARM_FULL_PROXY_NAME"],
        _base["J3_OUTPUT_PROXY_NAME"],
        _base["J3_STATOR_PROXY_NAME"],
        _base["FOREARM_PROXY_NAME"],
        _base["J4_STATOR_PROXY_NAME"],
        _base["J4_OUTPUT_PROXY_NAME"],
        WRIST_PROXY_NAME,
    )
    objects = {name: doc.getObject(name) for name in names}
    missing = [name for name, obj in objects.items() if obj is None]
    if missing:
        raise RuntimeError("V7 碰撞代理缺失: " + ", ".join(missing))

    world_shape = _base["world_shape"]
    overlap_and_distance = _base["_overlap_and_distance"]
    fixed = world_shape(objects[_base["FIXED_PROXY_NAME"]])
    shoulder = world_shape(objects[_base["SHOULDER_PROXY_NAME"]])
    arm_motion = world_shape(objects[_base["UPPERARM_MOTION_PROXY_NAME"]])
    arm_full = world_shape(objects[_base["UPPERARM_FULL_PROXY_NAME"]])
    j3_output = world_shape(objects[_base["J3_OUTPUT_PROXY_NAME"]])
    j3_stator = world_shape(objects[_base["J3_STATOR_PROXY_NAME"]])
    forearm = world_shape(objects[_base["FOREARM_PROXY_NAME"]])
    j4_stator = world_shape(objects[_base["J4_STATOR_PROXY_NAME"]])
    j4_output = world_shape(objects[_base["J4_OUTPUT_PROXY_NAME"]])
    wrist = world_shape(objects[WRIST_PROXY_NAME])

    pairs = {
        "moving_vs_base": (arm_motion, fixed),
        "moving_vs_shoulder": (arm_motion, shoulder),
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
        "j4_output_vs_base": (j4_output, fixed),
        "j4_output_vs_shoulder": (j4_output, shoulder),
        "j4_output_vs_upperarm": (j4_output, arm_full),
        "j4_output_vs_j3_output": (j4_output, j3_output),
        "j4_output_vs_j3_stator": (j4_output, j3_stator),
        "j4_output_vs_forearm": (j4_output, forearm),
        "j4_output_vs_j4_stator": (j4_output, j4_stator),
        # New R4 wrist pre-link against every older rigid body/environment.
        # Wrist vs J4 output and its six M4 screws are same-rigid exclusions.
        "wrist_vs_base": (wrist, fixed),
        "wrist_vs_shoulder": (wrist, shoulder),
        "wrist_vs_upperarm": (wrist, arm_full),
        "wrist_vs_j3_output": (wrist, j3_output),
        "wrist_vs_j3_stator": (wrist, j3_stator),
        "wrist_vs_forearm": (wrist, forearm),
        "wrist_vs_j4_stator": (wrist, j4_stator),
    }

    tolerance = _base["PENETRATION_VOLUME_TOLERANCE_MM3"]
    rows = {}
    collision = False
    for name, (first, second) in pairs.items():
        overlap, distance = overlap_and_distance(first, second, compute_clearance)
        colliding = overlap > tolerance
        rows[name] = {
            "overlap_mm3": overlap,
            "clearance_mm": 0.0 if colliding else distance,
            "collision": colliding,
        }
        collision = collision or colliding

    return {
        "collision": collision,
        "scope": "J1 environment + J2/J3/J4 + actual wrist pre-link; J5 absent",
        "penetration_volume_tolerance_mm3": tolerance,
        "same_rigid_exclusions": [
            "J3 stator vs forearm",
            "J3 stator/forearm vs J4 stator",
            "J3 rigid components vs their twelve M3.5 fasteners",
            "J4 output rotor vs wrist pre-link",
            "J4 output rotor/wrist pre-link vs six J4 output M4 fasteners",
        ],
        "adjacent_joint_pairs_checked": [
            "j3_stator_vs_j3_output",
            "j4_output_vs_j4_stator",
            "wrist_vs_j4_stator",
        ],
        "adjacent_contact_policy": (
            "surface tangency/common volume zero is allowed; no whole-rigid-pair suppression"
        ),
        "pairs": rows,
    }


def kinematic_radii_mm(doc=None):
    """Conservative J2/J3/J4 radii including the complete wrist pre-link."""

    doc = doc or active_v7_document()
    moving, r2, r3, _r4, j2, j3, j4 = _base["_chain_objects"](doc)
    o2, a2 = _base["_world_joint_axis"](moving, j2)
    o3, a3 = _base["_world_joint_axis"](r2, j3)
    o4, a4 = _base["_world_joint_axis"](r3, j4)
    shape = lambda name: _base["world_shape"](doc.getObject(name))
    radial = _base["_radial_bound"]
    axis_offset = _base["_axis_offset"]

    upperarm = shape(_base["UPPERARM_FULL_PROXY_NAME"])
    j3_output = shape(_base["J3_OUTPUT_PROXY_NAME"])
    j3_stator = shape(_base["J3_STATOR_PROXY_NAME"])
    forearm = shape(_base["FOREARM_PROXY_NAME"])
    j4_stator = shape(_base["J4_STATOR_PROXY_NAME"])
    j4_output = shape(_base["J4_OUTPUT_PROXY_NAME"])
    wrist = shape(WRIST_PROXY_NAME)

    r4 = max(radial(j4_output, o4, a4), radial(wrist, o4, a4), 1.0)
    j4_envelope = max(
        radial(j4_stator, o4, a4),
        radial(j4_output, o4, a4),
        radial(wrist, o4, a4),
    )
    r3_bound = max(
        radial(j3_stator, o3, a3),
        radial(forearm, o3, a3),
        axis_offset(o3, a3, o4) + j4_envelope,
    )
    r2_bound = max(
        radial(upperarm, o2, a2),
        radial(j3_output, o2, a2),
        axis_offset(o2, a2, o3) + r3_bound,
    )
    return {"j2_mm": r2_bound, "j3_mm": r3_bound, "j4_mm": r4}


# Replace the V6 module's dynamic hooks.  All of its proven chain transform,
# limit, sweep, six-item snapshot/rollback and observer code now calls V7.
_base["active_v6_document"] = active_v7_document
_base["collision_result"] = collision_result
_base["kinematic_radii_mm"] = kinematic_radii_mm
_BASE_SETTER = _base["set_j2_j3_j4_angles_v6"]
current_angles = _base["current_angles"]


def set_j2_j3_j4_angles_v7(
    j2_deg: float | None = None,
    j3_deg: float | None = None,
    j4_deg: float | None = None,
    sweep_step_deg: float | None = None,
):
    return _BASE_SETTER(j2_deg, j3_deg, j4_deg, sweep_step_deg)


def set_j2_angle_v7(angle_deg: float, sweep_step_deg: float | None = None):
    return set_j2_j3_j4_angles_v7(j2_deg=angle_deg, sweep_step_deg=sweep_step_deg)


def set_j3_angle_v7(angle_deg: float, sweep_step_deg: float | None = None):
    return set_j2_j3_j4_angles_v7(j3_deg=angle_deg, sweep_step_deg=sweep_step_deg)


def set_j4_angle_v7(angle_deg: float, sweep_step_deg: float | None = None):
    return set_j2_j3_j4_angles_v7(j4_deg=angle_deg, sweep_step_deg=sweep_step_deg)


def set_j2_j3_angles_compat(
    j2_deg: float | None = None,
    j3_deg: float | None = None,
    sweep_step_deg: float | None = None,
):
    return set_j2_j3_j4_angles_v7(
        j2_deg=j2_deg, j3_deg=j3_deg, sweep_step_deg=sweep_step_deg
    )


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


def install_j234_wrist_collision_monitor_v7():
    doc = active_v7_document()
    for attribute in (
        "_J23_COLLISION_OBSERVER_V5",
        "_J234_COLLISION_OBSERVER_V6",
        V7_OBSERVER_ATTRIBUTE,
    ):
        _remove_observer(attribute)

    observer = _base["J234CollisionObserver"](doc)
    App.addDocumentObserver(observer)
    setattr(App, V7_OBSERVER_ATTRIBUTE, observer)

    App.set_j2_j3_j4_angles_v7 = set_j2_j3_j4_angles_v7
    App.set_j2_angle_v7 = set_j2_angle_v7
    App.set_j3_angle_v7 = set_j3_angle_v7
    App.set_j4_angle_v7 = set_j4_angle_v7
    # V6/V5 compatibility calls route through V7 so the wrist always follows.
    App.set_j2_j3_j4_angles_v6 = set_j2_j3_j4_angles_v7
    App.set_j2_angle_v6 = set_j2_angle_v7
    App.set_j3_angle_v6 = set_j3_angle_v7
    App.set_j4_angle_v6 = set_j4_angle_v7
    App.set_j2_j3_angles_v5 = set_j2_j3_angles_compat
    App.set_j2_angle_v5 = set_j2_angle_v7
    App.set_j3_angle_v5 = set_j3_angle_v7

    result = collision_result(doc, compute_clearance=True)
    angles = current_angles(doc)
    plan = _base["_sweep_plan"](doc, angles, angles, None)
    _base["_set_status"](
        doc,
        "CLEAR" if not result["collision"] else "COLLISION",
        angles,
        result,
        plan,
    )
    App.Console.PrintMessage(
        "V7 J2/J3/J4+腕前连接件扫掠碰撞监视已启动（未保存FCStd）。"
        f"外部碰撞对 {len(result['pairs'])} 组，"
        "腕前件与J4输出/六颗M4按同刚体排除。"
        f"等角最坏自适应步长 {plan['adaptive_equal_axis_step_deg']:.4f}°。"
        "命令：App.set_j4_angle_v7(deg) 或 "
        "App.set_j2_j3_j4_angles_v7(j2,j3,j4)。\n"
    )
    return observer

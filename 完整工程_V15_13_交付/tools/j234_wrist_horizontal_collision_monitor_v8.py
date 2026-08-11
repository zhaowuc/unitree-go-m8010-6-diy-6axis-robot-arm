from __future__ import annotations

"""V8 runtime guard for the horizontal inward wrist-prelink assembly.

V8 retains the V7 rigid-body membership and collision matrix.  This thin
revision layer reuses the validated V7 swept solver/observer while binding it
to the V8 document and status object.  No function in this module saves the
FCStd document.
"""

from pathlib import Path

import FreeCAD as App


ROOT = Path(__file__).resolve().parent
V7_MONITOR_PATH = ROOT / "j234_wrist_collision_monitor_v7.py"
if not V7_MONITOR_PATH.is_file():
    raise RuntimeError("V8运行时基底缺失: " + str(V7_MONITOR_PATH))

V8_OBSERVER_ATTRIBUTE = "_J234_WRIST_HORIZONTAL_COLLISION_OBSERVER_V8"
V8_STATUS_NAME = "V8_Wrist_Prelink_Horizontal_Physical_Test_Status"

_v7_source = V7_MONITOR_PATH.read_text(encoding="utf-8").replace(
    "_J234_WRIST_COLLISION_OBSERVER_V7", V8_OBSERVER_ATTRIBUTE
)
_v7 = {
    "__file__": str(V7_MONITOR_PATH),
    "__name__": "_j234_wrist_collision_monitor_v7_base_for_v8",
}
exec(compile(_v7_source, str(V7_MONITOR_PATH), "exec"), _v7)


def active_v8_document():
    doc = App.ActiveDocument
    base = _v7["_base"]
    required = (
        base["J2_RIGID_NAME"],
        base["J3_RIGID_NAME"],
        base["J4_RIGID_NAME"],
        base["J2_JOINT_NAME"],
        base["J3_JOINT_NAME"],
        base["J4_JOINT_NAME"],
        base["J4_STATOR_PROXY_NAME"],
        base["J4_OUTPUT_PROXY_NAME"],
        _v7["WRIST_DISPLAY_NAME"],
        _v7["WRIST_PROXY_NAME"],
        *_v7["J4_M4_SCREW_NAMES"],
        V8_STATUS_NAME,
    )
    if doc is None or any(doc.getObject(name) is None for name in required):
        missing = [] if doc is None else [
            name for name in required if doc.getObject(name) is None
        ]
        suffix = "" if not missing else "\n缺失对象: " + ", ".join(missing)
        raise RuntimeError(
            "请先打开 机械臂底座肩部大臂小臂腕前连接件水平装配_v8.FCStd"
            + suffix
        )
    base["STATUS_NAME"] = V8_STATUS_NAME
    return doc


# Rebind both the V7 extension layer and its V6 serial-chain core.
_v7["active_v7_document"] = active_v8_document
_v7["_base"]["active_v6_document"] = active_v8_document
_v7["_base"]["STATUS_NAME"] = V8_STATUS_NAME

collision_result = _v7["collision_result"]
kinematic_radii_mm = _v7["kinematic_radii_mm"]
current_angles = _v7["current_angles"]
_V7_SETTER = _v7["set_j2_j3_j4_angles_v7"]


def set_j2_j3_j4_angles_v8(
    j2_deg: float | None = None,
    j3_deg: float | None = None,
    j4_deg: float | None = None,
    sweep_step_deg: float | None = None,
):
    return _V7_SETTER(j2_deg, j3_deg, j4_deg, sweep_step_deg)


def set_j2_angle_v8(angle_deg: float, sweep_step_deg: float | None = None):
    return set_j2_j3_j4_angles_v8(j2_deg=angle_deg, sweep_step_deg=sweep_step_deg)


def set_j3_angle_v8(angle_deg: float, sweep_step_deg: float | None = None):
    return set_j2_j3_j4_angles_v8(j3_deg=angle_deg, sweep_step_deg=sweep_step_deg)


def set_j4_angle_v8(angle_deg: float, sweep_step_deg: float | None = None):
    return set_j2_j3_j4_angles_v8(j4_deg=angle_deg, sweep_step_deg=sweep_step_deg)


def set_j2_j3_angles_compat(
    j2_deg: float | None = None,
    j3_deg: float | None = None,
    sweep_step_deg: float | None = None,
):
    return set_j2_j3_j4_angles_v8(
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


def install_j234_wrist_horizontal_collision_monitor_v8():
    doc = active_v8_document()
    for attribute in (
        "_J23_COLLISION_OBSERVER_V5",
        "_J234_COLLISION_OBSERVER_V6",
        "_J234_WRIST_COLLISION_OBSERVER_V7",
        V8_OBSERVER_ATTRIBUTE,
    ):
        _remove_observer(attribute)

    base = _v7["_base"]
    observer = base["J234CollisionObserver"](doc)
    App.addDocumentObserver(observer)
    setattr(App, V8_OBSERVER_ATTRIBUTE, observer)

    App.set_j2_j3_j4_angles_v8 = set_j2_j3_j4_angles_v8
    App.set_j2_angle_v8 = set_j2_angle_v8
    App.set_j3_angle_v8 = set_j3_angle_v8
    App.set_j4_angle_v8 = set_j4_angle_v8
    # All legacy calls route through the V8 chain so R4 always follows.
    App.set_j2_j3_j4_angles_v7 = set_j2_j3_j4_angles_v8
    App.set_j2_angle_v7 = set_j2_angle_v8
    App.set_j3_angle_v7 = set_j3_angle_v8
    App.set_j4_angle_v7 = set_j4_angle_v8
    App.set_j2_j3_j4_angles_v6 = set_j2_j3_j4_angles_v8
    App.set_j2_angle_v6 = set_j2_angle_v8
    App.set_j3_angle_v6 = set_j3_angle_v8
    App.set_j4_angle_v6 = set_j4_angle_v8
    App.set_j2_j3_angles_v5 = set_j2_j3_angles_compat
    App.set_j2_angle_v5 = set_j2_angle_v8
    App.set_j3_angle_v5 = set_j3_angle_v8

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
    App.Console.PrintMessage(
        "V8 J2/J3/J4+水平内向腕前件扫掠碰撞监视已启动（未保存FCStd）。"
        f"外部碰撞对 {len(result['pairs'])} 组，"
        f"等角最坏自适应步长 {plan['adaptive_equal_axis_step_deg']:.4f}°。"
        "命令：App.set_j4_angle_v8(deg) 或 "
        "App.set_j2_j3_j4_angles_v8(j2,j3,j4)。\n"
    )
    return observer


from __future__ import annotations

"""V10 FreeCAD GUI swept-collision guard.

This adapter reuses the validated V9 J1/J2/J3/J4 swept BRep monitor against the
objects and placements in the active V10 document.  V10's native J4 limits are
already narrowed to the high-detail physical envelope (-116..+170 degrees), so
the inherited target validation blocks the measured J4 contact band before a
sweep starts.  The remaining BRep monitor protects coupled J1/J2/J3/J4 paths
and atomically restores the last clear pose when penetration is detected.
"""

from pathlib import Path

import FreeCAD as App


ROOT = Path(__file__).resolve().parent
V9_MONITOR = ROOT / "j1234_wrist_180_inward_collision_monitor_v9.py"
V10_STATUS = "V10_Wrist_Prelink_Blue_Axis_Roll_Physical_Test_Status"
V10_J234_OBSERVER = "_J234_WRIST_PHYSICAL_COLLISION_OBSERVER_V10"
V10_J1_OBSERVER = "_J1_FULL_CHAIN_COLLISION_OBSERVER_V10"


if not V9_MONITOR.is_file():
    raise RuntimeError("V10运行时碰撞监视基底缺失: " + str(V9_MONITOR))


source = V9_MONITOR.read_text(encoding="utf-8")
replacements = {
    "V9_Wrist_Prelink_180_Inward_Physical_Test_Status": V10_STATUS,
    "_J234_WRIST_180_INWARD_COLLISION_OBSERVER_V9": V10_J234_OBSERVER,
    "_J1_FULL_CHAIN_COLLISION_OBSERVER_V9": V10_J1_OBSERVER,
    "active_v9_document": "active_v10_document",
    "current_j1_angle": "current_j1_angle_v10",
    "current_j1234_angles": "current_j1234_angles_v10",
    "set_j2_j3_j4_angles_v9": "set_j2_j3_j4_angles_v10",
    "set_j2_angle_v9": "set_j2_angle_v10",
    "set_j3_angle_v9": "set_j3_angle_v10",
    "set_j4_angle_v9": "set_j4_angle_v10",
    "set_j1_angle_v9": "set_j1_angle_v10",
    "J1AngleObserverV9": "J1AngleObserverV10",
    "install_j1234_wrist_180_inward_collision_monitor_v9": (
        "install_j1234_physical_collision_guard_v10"
    ),
    "机械臂底座肩部大臂小臂腕前连接件180度内向装配_v9.FCStd": (
        "机械臂底座肩部大臂小臂腕前连接件蓝线水平翻转内置装配_v10.FCStd"
    ),
    "V9 J1/J2/J3/J4+180°内向腕前件碰撞监视": (
        "V10 J1/J2/J3/J4+高精度J4实体限位碰撞监视"
    ),
}
for old, new in replacements.items():
    source = source.replace(old, new)

namespace = {
    "__file__": str(V9_MONITOR),
    "__name__": "_j1234_physical_collision_guard_v10_runtime",
}
exec(compile(source, str(V9_MONITOR), "exec"), namespace)


install_j1234_physical_collision_guard_v10 = namespace[
    "install_j1234_physical_collision_guard_v10"
]
set_j1_angle_v10 = namespace["set_j1_angle_v10"]
set_j2_angle_v10 = namespace["set_j2_angle_v10"]
set_j3_angle_v10 = namespace["set_j3_angle_v10"]
set_j4_angle_v10 = namespace["set_j4_angle_v10"]
set_j2_j3_j4_angles_v10 = namespace["set_j2_j3_j4_angles_v10"]
current_j1234_angles_v10 = namespace["current_j1234_angles_v10"]
collision_result_v10 = namespace["collision_result"]


def install() -> dict:
    result = install_j1234_physical_collision_guard_v10()
    App.set_j1_angle_v10 = set_j1_angle_v10
    App.set_j2_angle_v10 = set_j2_angle_v10
    App.set_j3_angle_v10 = set_j3_angle_v10
    App.set_j4_angle_v10 = set_j4_angle_v10
    App.set_j2_j3_j4_angles_v10 = set_j2_j3_j4_angles_v10
    App.Console.PrintMessage(
        "V10物理保护：J4原生硬限位 -116°～+170°；"
        "J1/J2/J3/J4路径采用扫掠碰撞检测并在碰撞时回滚。\n"
    )
    return result


if __name__ == "__main__":
    install()

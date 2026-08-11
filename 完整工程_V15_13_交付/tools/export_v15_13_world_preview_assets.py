from __future__ import annotations

"""Export V15.13 terminal objects in their final world placements for rendering."""

from pathlib import Path

import FreeCAD as App
import Mesh


ROOT = Path(
    r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15/"
    r"V15_13_整机深度复核_相机上置机械零位"
)
FCSTD = ROOT / "机械臂完整装配_六轴_相机上置机械零位_深度复核_v15_13.FCStd"
OUTPUT = ROOT / "三视图资产_world"

STATIC_HARDWARE = [f"Gripper_Hardware_{index:02d}" for index in (3, 4, 7, 8, 9, 10, 15, 16)]
MOVING_LINKS = (
    "Gripper_Left_Outer_Link",
    "Gripper_Right_Outer_Link",
    "Gripper_Left_Drive_Link",
    "Gripper_Right_Drive_Link",
)
FINGERS_AND_HARDWARE = (
    "Gripper_Left_Finger",
    "Gripper_Right_Finger",
    *[f"Gripper_Hardware_{index:02d}" for index in (1, 2, 5, 6, 11, 12, 13, 14, 17, 18, 19)],
)
M4_SCREWS = tuple(f"DM_G6220_Connector_M4x14_Screw_{index:02d}" for index in range(1, 7))
PRINTED_SCREWS = tuple(
    f"Gripper_Clevis_3DPrinted_M3x25_Screw_{index:02d}" for index in range(1, 3)
)


def objects(doc, names):
    result = []
    for name in names:
        obj = doc.getObject(name)
        if obj is None or not hasattr(obj, "Shape") or obj.Shape.isNull():
            raise RuntimeError(f"missing preview geometry: {name}")
        result.append(obj)
    return result


def export_layer(doc, filename: str, names):
    target = OUTPUT / filename
    selected = objects(doc, names)
    Mesh.export(selected, str(target))
    if not target.is_file() or target.stat().st_size == 0:
        raise RuntimeError(f"empty preview asset: {target}")
    return selected


def main():
    if not FCSTD.is_file():
        raise FileNotFoundError(FCSTD)
    OUTPUT.mkdir(parents=True, exist_ok=True)
    doc = App.openDocument(str(FCSTD))
    try:
        doc.recompute()
        layers = {
            "01_连接件_world.stl": ("J6_Gripper_Connector_STEP_Display",),
            "02_夹爪固定框架与舵机_world.stl": (
                "Gripper_Servo_STEP_Display",
                "Gripper_Fixed_Frame_STEP_Display",
                *STATIC_HARDWARE,
            ),
            "03_夹爪四连杆_world.stl": MOVING_LINKS,
            "04_夹爪手指与随动紧固件_world.stl": FINGERS_AND_HARDWARE,
            "05_DM连接六颗M4x14_world.stl": M4_SCREWS,
            "06_夹爪两颗打印M3x25_world.stl": PRINTED_SCREWS,
            "07_Gemini相机自带铰链耳_world.stl": ("Gemini_Pro_Camera_STEP_Display",),
        }
        complete = []
        for filename, names in layers.items():
            complete.extend(export_layer(doc, filename, names))
        target = OUTPUT / "08_J6输出刚体相机上置机械零位完整_world.stl"
        Mesh.export(complete, str(target))
        if not target.is_file() or target.stat().st_size == 0:
            raise RuntimeError(f"empty complete preview asset: {target}")
        for filename in (*layers, target.name):
            path = OUTPUT / filename
            print(f"{path}\t{path.stat().st_size}", flush=True)
    finally:
        App.closeDocument(doc.Name)


if __name__ == "__main__":
    main()

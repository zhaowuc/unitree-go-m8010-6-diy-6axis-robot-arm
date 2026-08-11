from __future__ import annotations

"""Render V15.13 using world-frame terminal assets and the proven upstream renderer."""

import importlib.util
import json
from pathlib import Path

import numpy as np
import vtk


WORKSPACE = Path(r"D:/AI_JIXIEBI/go_m8010_arm_gui")
ROOT = Path(
    r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15/"
    r"V15_13_整机深度复核_相机上置机械零位"
)
ASSETS = ROOT / "三视图资产_world"
OUTPUT_DIR = ROOT / "装配预览图"
V13_RENDER = WORKSPACE / "render_robot_arm_dm_g6220_v13.py"
AXES_JSON = ROOT / "V15_13_真实关节轴与相机上置零位.json"

CONNECTOR = ASSETS / "01_连接件_world.stl"
STATIC = ASSETS / "02_夹爪固定框架与舵机_world.stl"
LINKS = ASSETS / "03_夹爪四连杆_world.stl"
FINGERS = ASSETS / "04_夹爪手指与随动紧固件_world.stl"
M4 = ASSETS / "05_DM连接六颗M4x14_world.stl"
PRINTED_M3 = ASSETS / "06_夹爪两颗打印M3x25_world.stl"
CAMERA = ASSETS / "07_Gemini相机自带铰链耳_world.stl"

TRIVIEW = OUTPUT_DIR / "V15_13_整机三视图.png"
TERMINAL = OUTPUT_DIR / "V15_13_相机上置末端四视图.png"
ISOMETRIC = OUTPUT_DIR / "V15_13_整机等轴测.png"
README = OUTPUT_DIR / "README_装配预览.md"

WORLD_Z = np.array([0.0, 0.0, 1.0], dtype=float)
COLORS = {
    "connector": "#295BC7",
    "static": "#222831",
    "links": "#161B22",
    "fingers": "#14AFC4",
    "m4": "#D9E1EA",
    "printed": "#F0B429",
    "camera": "#20242B",
    "axis": "#80C800",
    "title": "#12263A",
    "blue": "#1769AA",
    "green": "#16845B",
}


def load_v13():
    spec = importlib.util.spec_from_file_location("_v15_13_v13_renderer", V13_RENDER)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"cannot load renderer: {V13_RENDER}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


v13 = load_v13()
v3 = v13.v3
v11 = v13.v11
axes = json.loads(AXES_JSON.read_text(encoding="utf-8"))
frame = axes["frames_at_mechanical_zero"]["link6"]
J6_ORIGIN = np.asarray(frame["origin_world_mm"], dtype=float)
J6_X = np.asarray(frame["x_axis_world_at_zero"], dtype=float)
J6_Y = np.asarray(frame["y_axis_world_at_zero"], dtype=float)
J6_Z = np.asarray(frame["z_axis_world_at_zero"], dtype=float)
for axis in (J6_X, J6_Y, J6_Z):
    axis /= np.linalg.norm(axis)

arm_longitudinal = np.asarray(v13.OUTPUT, dtype=float).copy()
arm_longitudinal[2] = 0.0
arm_longitudinal /= np.linalg.norm(arm_longitudinal)
front_direction = np.cross(WORLD_Z, arm_longitudinal)
front_direction /= np.linalg.norm(front_direction)
right_direction = arm_longitudinal


def add_tool(renderer, edges=True):
    v11.add_actor(renderer, CONNECTOR, COLORS["connector"], 1.0, edges)
    v11.add_actor(renderer, STATIC, COLORS["static"], 0.98, edges)
    v11.add_actor(renderer, LINKS, COLORS["links"], 1.0, edges)
    v11.add_actor(renderer, FINGERS, COLORS["fingers"], 1.0, edges)
    v11.add_actor(renderer, M4, COLORS["m4"], 1.0, edges)
    v11.add_actor(renderer, PRINTED_M3, COLORS["printed"], 1.0, edges)
    v11.add_actor(renderer, CAMERA, COLORS["camera"], 0.98, edges)


def add_complete(renderer, edges=True):
    v13.add_complete(renderer, edges=edges)
    add_tool(renderer, edges=edges)


def add_terminal_only(renderer, edges=True):
    v13.add_motor(renderer, edges=edges)
    add_tool(renderer, edges=edges)


def manual_camera(renderer, direction, up, focus, scale):
    direction = np.asarray(direction, dtype=float)
    direction /= np.linalg.norm(direction)
    up = np.asarray(up, dtype=float)
    up /= np.linalg.norm(up)
    camera = renderer.GetActiveCamera()
    camera.SetFocalPoint(*focus)
    camera.SetPosition(*(focus + direction * 700.0))
    camera.SetViewUp(*up)
    camera.ParallelProjectionOn()
    camera.SetParallelScale(scale)
    renderer.ResetCameraClippingRange()


def add_header(renderer, title, subtitle, detail=None):
    v3.add_text(renderer, title, (0.03, 0.935), 24, COLORS["title"])
    v3.add_text(renderer, subtitle, (0.03, 0.888), 14, COLORS["blue"])
    if detail:
        v3.add_text(renderer, detail, (0.03, 0.848), 13, COLORS["green"])


def render_terminal():
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(5000, 1500)
    focus = J6_ORIGIN + J6_Z * 62.0
    views = (
        ("CAMERA FRONT / 相机镜头侧", J6_Z, -J6_X),
        ("REAR / 相机背面侧", -J6_Z, -J6_X),
        ("HINGE SIDE / 铰链安装侧", J6_Y, J6_Z),
        ("ISOMETRIC / 末端总览", J6_Z + 0.8 * J6_Y + 0.65 * J6_X, J6_Z),
    )
    for index, (title, direction, up) in enumerate(views):
        renderer = v3.renderer()
        renderer.SetViewport(index / 4.0, 0.0, (index + 1) / 4.0, 1.0)
        window.AddRenderer(renderer)
        add_terminal_only(renderer, edges=True)
        manual_camera(renderer, direction, up, focus, 135.0)
        add_header(
            renderer,
            title,
            "V15.13 / J6=0 deg / CAMERA UP",
            "connector relative interface unchanged / no added adapter",
        )
    v3.save_window(window, TERMINAL)


def render_full_triview():
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(5400, 1600)
    views = (
        ("FRONT VIEW / 正视图", front_direction, WORLD_Z),
        ("RIGHT VIEW / 右视图", right_direction, WORLD_Z),
        ("TOP VIEW / 俯视图", WORLD_Z, front_direction),
    )
    for index, (title, direction, up) in enumerate(views):
        renderer = v3.renderer()
        renderer.SetViewport(index / 3.0, 0.0, (index + 1) / 3.0, 1.0)
        window.AddRenderer(renderer)
        add_complete(renderer, edges=True)
        v3.camera_for(renderer, direction, up, parallel_scale=420.0)
        add_header(
            renderer,
            title,
            "V15.13 ZERO POSE / SAME SCALE",
            "J6=0 deg camera-up mechanical zero / connector interface unchanged",
        )
    v3.save_window(window, TRIVIEW)


def render_isometric():
    window = vtk.vtkRenderWindow()
    window.SetOffScreenRendering(1)
    window.SetSize(2400, 1800)
    renderer = v3.renderer()
    window.AddRenderer(renderer)
    add_complete(renderer, edges=True)
    direction = front_direction * 0.75 + right_direction * 0.58 + WORLD_Z * 0.44
    v3.camera_for(renderer, direction, WORLD_Z, parallel_scale=410.0)
    add_header(
        renderer,
        "V15.13 COMPLETE ARM / 整机等轴测",
        "J6=0 deg / CAMERA UP / GRIPPER REVERSED",
        "world-frame verified render / no transition part added",
    )
    v3.save_window(window, ISOMETRIC)


def main():
    required = (
        V13_RENDER,
        AXES_JSON,
        CONNECTOR,
        STATIC,
        LINKS,
        FINGERS,
        M4,
        PRINTED_M3,
        CAMERA,
    )
    missing = [str(path) for path in required if not path.is_file() or path.stat().st_size == 0]
    if missing:
        raise FileNotFoundError("missing rendering input: " + ", ".join(missing))
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)
    render_terminal()
    render_full_triview()
    render_isometric()
    README.write_text(
        "# V15.13 装配预览\n\n"
        "三张预览均由最终 FCStd 的世界位姿生成；刚性 Link 局部坐标导出仅用于 ROS2/MuJoCo，"
        "不再直接作为整机世界坐标渲染。J6=0° 时相机在上，连接件相对 DM-G6220 输出法兰的"
        "安装关系未改变，夹爪保持反转状态，未增加转接片。\n",
        encoding="utf-8",
    )
    for path in (TERMINAL, TRIVIEW, ISOMETRIC, README):
        print(f"{path}\t{path.stat().st_size}", flush=True)


if __name__ == "__main__":
    main()

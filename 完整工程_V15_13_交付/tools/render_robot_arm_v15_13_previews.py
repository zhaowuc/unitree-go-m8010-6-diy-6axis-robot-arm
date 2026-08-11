from __future__ import annotations

"""Render V15.13 full-arm and camera-up terminal previews from link assets."""

import json
from pathlib import Path

import numpy as np
import vtk


ROOT = Path(
    r"D:/AI_JIXIEBI/模型/机械臂完整装配_真实关节轴_v15/"
    r"V15_13_整机深度复核_相机上置机械零位"
)
ASSETS = ROOT / "rigid_links_v15_13"
AXES = ROOT / "V15_13_真实关节轴与相机上置零位.json"
OUTPUT = ROOT / "装配预览图"
TRIVIEW = OUTPUT / "V15_13_整机三视图.png"
TERMINAL = OUTPUT / "V15_13_相机上置末端四视图.png"
ISOMETRIC = OUTPUT / "V15_13_整机等轴测.png"

COLORS = {
    "base_link": (0.28, 0.31, 0.36),
    "link1": (0.72, 0.12, 0.16),
    "link2": (0.76, 0.16, 0.20),
    "link3": (0.78, 0.19, 0.24),
    "link4": (0.82, 0.22, 0.27),
    "link5": (0.18, 0.21, 0.25),
    "link6": (0.12, 0.15, 0.19),
    "gripper": (0.04, 0.58, 0.68),
}


def frame_matrix(frame: dict) -> vtk.vtkMatrix4x4:
    matrix = vtk.vtkMatrix4x4()
    x = frame["x_axis_world_at_zero"]
    y = frame["y_axis_world_at_zero"]
    z = frame["z_axis_world_at_zero"]
    origin = frame["origin_world_mm"]
    values = (
        (x[0], y[0], z[0], origin[0]),
        (x[1], y[1], z[1], origin[1]),
        (x[2], y[2], z[2], origin[2]),
        (0.0, 0.0, 0.0, 1.0),
    )
    for row in range(4):
        for column in range(4):
            matrix.SetElement(row, column, float(values[row][column]))
    return matrix


def read_world_poly(path: Path, matrix: vtk.vtkMatrix4x4) -> vtk.vtkPolyData:
    reader = vtk.vtkSTLReader()
    reader.SetFileName(str(path))
    reader.Update()
    transform = vtk.vtkTransform()
    transform.SetMatrix(matrix)
    filt = vtk.vtkTransformPolyDataFilter()
    filt.SetInputData(reader.GetOutput())
    filt.SetTransform(transform)
    filt.Update()
    output = vtk.vtkPolyData()
    output.DeepCopy(filt.GetOutput())
    return output


def actor(poly: vtk.vtkPolyData, color, edges=True):
    mapper = vtk.vtkPolyDataMapper()
    mapper.SetInputData(poly)
    result = vtk.vtkActor()
    result.SetMapper(mapper)
    result.GetProperty().SetColor(*color)
    result.GetProperty().SetInterpolationToPhong()
    if edges:
        result.GetProperty().EdgeVisibilityOn()
        result.GetProperty().SetEdgeColor(0.05, 0.06, 0.08)
        result.GetProperty().SetLineWidth(0.45)
    return result


def renderer():
    result = vtk.vtkRenderer()
    result.SetBackground(0.965, 0.972, 0.982)
    result.SetBackground2(0.86, 0.89, 0.93)
    result.GradientBackgroundOn()
    result.SetUseDepthPeeling(True)
    return result


def add_text(target, text, position, size=24, color=(0.07, 0.11, 0.16)):
    item = vtk.vtkTextActor()
    item.SetInput(text)
    item.SetPosition(*position)
    item.GetTextProperty().SetFontSize(size)
    item.GetTextProperty().SetColor(*color)
    item.GetTextProperty().SetBold(True)
    target.AddActor2D(item)


def save(window, path: Path):
    window.SetOffScreenRendering(1)
    window.Render()
    image = vtk.vtkWindowToImageFilter()
    image.SetInput(window)
    image.SetScale(1)
    image.SetInputBufferTypeToRGBA()
    image.ReadFrontBufferOff()
    image.Update()
    writer = vtk.vtkPNGWriter()
    writer.SetFileName(str(path))
    writer.SetInputConnection(image.GetOutputPort())
    writer.Write()


def set_camera(target, focus, direction, up, scale):
    direction = np.asarray(direction, dtype=float)
    direction /= np.linalg.norm(direction)
    up = np.asarray(up, dtype=float)
    up /= np.linalg.norm(up)
    camera = target.GetActiveCamera()
    camera.SetFocalPoint(*focus)
    camera.SetPosition(*(focus + direction * 1100.0))
    camera.SetViewUp(*up)
    camera.ParallelProjectionOn()
    camera.SetParallelScale(float(scale))
    target.ResetCameraClippingRange()


def add_scene(target, polys, terminal_only=False):
    names = ("link5", "link6", "gripper") if terminal_only else tuple(polys)
    for name in names:
        target.AddActor(actor(polys[name], COLORS[name], edges=True))


def build_polys(axis_data):
    frames = axis_data["frames_at_mechanical_zero"]
    result = {}
    for name in ("base_link", "link1", "link2", "link3", "link4", "link5", "link6"):
        result[name] = read_world_poly(
            ASSETS / "visual" / f"{name}.stl", frame_matrix(frames[name])
        )
    result["gripper"] = read_world_poly(
        ASSETS / "gripper_internal_zero_reference" / "gripper_complete_zero_reference.stl",
        frame_matrix(frames["link6"]),
    )
    return result


def render_triview(polys, axis_data):
    window = vtk.vtkRenderWindow()
    window.SetSize(4800, 1500)
    views = (
        ("FRONT / 正视图", (0.0, -1.0, 0.0), (0.0, 0.0, 1.0)),
        ("RIGHT / 右视图", (1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),
        ("TOP / 俯视图", (0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),
    )
    focus = np.array([70.0, 280.0, 260.0])
    for index, (title, direction, up) in enumerate(views):
        target = renderer()
        target.SetViewport(index / 3.0, 0.0, (index + 1) / 3.0, 1.0)
        window.AddRenderer(target)
        add_scene(target, polys)
        set_camera(target, focus, direction, up, 390.0)
        add_text(target, title, (28, 1415), 26)
        add_text(target, "V15.13 / J6=0 deg / CAMERA UP", (28, 1368), 17, (0.08, 0.35, 0.52))
    save(window, TRIVIEW)


def render_terminal(polys, axis_data):
    frame = axis_data["frames_at_mechanical_zero"]["link6"]
    origin = np.asarray(frame["origin_world_mm"], dtype=float)
    x = np.asarray(frame["x_axis_world_at_zero"], dtype=float)
    y = np.asarray(frame["y_axis_world_at_zero"], dtype=float)
    z = np.asarray(frame["z_axis_world_at_zero"], dtype=float)
    focus = origin + z * 48.0
    views = (
        ("CAMERA FRONT / 相机正面", z, -x),
        ("REAR / 末端背面", -z, -x),
        ("HINGE SIDE / 铰链侧", y, z),
        ("ISOMETRIC / 等轴测", z + 0.72 * y + 0.55 * x, z),
    )
    window = vtk.vtkRenderWindow()
    window.SetSize(5000, 1450)
    for index, (title, direction, up) in enumerate(views):
        target = renderer()
        target.SetViewport(index / 4.0, 0.0, (index + 1) / 4.0, 1.0)
        window.AddRenderer(target)
        add_scene(target, polys, terminal_only=True)
        set_camera(target, focus, direction, up, 150.0)
        add_text(target, title, (24, 1368), 22)
        add_text(target, "connector unchanged / complete J6 output zero reclocked", (24, 1326), 13, (0.08, 0.35, 0.52))
    save(window, TERMINAL)


def render_isometric(polys):
    window = vtk.vtkRenderWindow()
    window.SetSize(2200, 1800)
    target = renderer()
    window.AddRenderer(target)
    add_scene(target, polys)
    set_camera(target, np.array([70.0, 300.0, 250.0]), (1.0, -1.25, 0.72), (0.0, 0.0, 1.0), 410.0)
    add_text(target, "GO-M8010 ARM / V15.13", (50, 1715), 34)
    add_text(target, "camera-up J6 mechanical zero / collision-audited rigid links", (50, 1660), 20, (0.08, 0.35, 0.52))
    save(window, ISOMETRIC)


def main():
    OUTPUT.mkdir(parents=True, exist_ok=True)
    data = json.loads(AXES.read_text(encoding="utf-8"))
    polys = build_polys(data)
    render_triview(polys, data)
    render_terminal(polys, data)
    render_isometric(polys)
    print(json.dumps({"status": "PASS", "outputs": [str(TRIVIEW), str(TERMINAL), str(ISOMETRIC)]}, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()

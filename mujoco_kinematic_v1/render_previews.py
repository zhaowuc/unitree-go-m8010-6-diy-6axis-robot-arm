from __future__ import annotations

"""Render verified zero-pose views with MuJoCo's own renderer."""

import os
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")

import mujoco
from PIL import Image, ImageDraw


ROOT = Path(__file__).resolve().parent
MODEL_XML = ROOT / "go_m8010_arm_v15_13_kinematic.xml"
OUTPUT = ROOT / "previews"


VIEWS = {
    "isometric": (135.0, -18.0, 1.35, (-0.14, 0.0, 0.30)),
    "front": (180.0, 0.0, 1.35, (-0.14, 0.0, 0.30)),
    "side": (90.0, 0.0, 1.35, (-0.14, 0.0, 0.30)),
    "top": (180.0, -90.0, 1.40, (-0.14, 0.0, 0.25)),
}


def render_view(renderer, data, name, settings, show_collision=False) -> Path:
    azimuth, elevation, distance, lookat = settings
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.azimuth = azimuth
    camera.elevation = elevation
    camera.distance = distance
    camera.lookat[:] = lookat
    option = mujoco.MjvOption()
    option.geomgroup[:] = 1
    if not show_collision:
        option.geomgroup[3] = 0
    renderer.update_scene(data, camera=camera, scene_option=option)
    pixels = renderer.render()
    target = OUTPUT / (f"zero_{name}_collision.png" if show_collision else f"zero_{name}.png")
    Image.fromarray(pixels).save(target)
    return target


def main() -> None:
    OUTPUT.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)
    renderer = mujoco.Renderer(model, height=720, width=960)
    paths = [render_view(renderer, data, name, settings) for name, settings in VIEWS.items()]
    render_view(renderer, data, "isometric", VIEWS["isometric"], show_collision=True)
    renderer.close()

    thumbs = [Image.open(path).convert("RGB") for path in paths]
    canvas = Image.new("RGB", (1920, 1440), (242, 244, 247))
    draw = ImageDraw.Draw(canvas)
    for index, (name, image) in enumerate(zip(VIEWS, thumbs)):
        x = (index % 2) * 960
        y = (index // 2) * 720
        canvas.paste(image, (x, y))
        draw.rectangle((x + 14, y + 14, x + 170, y + 48), fill=(20, 24, 31))
        draw.text((x + 24, y + 22), name.upper(), fill=(255, 255, 255))
    composite = OUTPUT / "V15_13_MuJoCo_zero_four_views.png"
    canvas.save(composite)
    print(composite)


if __name__ == "__main__":
    main()

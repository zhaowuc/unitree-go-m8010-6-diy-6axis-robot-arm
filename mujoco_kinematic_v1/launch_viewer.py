from __future__ import annotations

"""Open the V15.13 kinematic model in the MuJoCo interactive viewer."""

import time
from pathlib import Path

import mujoco
import mujoco.viewer


ROOT = Path(__file__).resolve().parent
MODEL_XML = ROOT / "go_m8010_arm_v15_13_kinematic.xml"


def main() -> None:
    model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    data = mujoco.MjData(model)
    data.qpos[:] = 0.0
    mujoco.mj_forward(model, data)
    with mujoco.viewer.launch_passive(model, data, show_left_ui=True, show_right_ui=True) as viewer:
        viewer.cam.lookat[:] = (0.0, 0.0, 0.33)
        viewer.cam.distance = 1.15
        viewer.cam.azimuth = 135.0
        viewer.cam.elevation = -18.0
        while viewer.is_running():
            # Kinematic version: no mj_step and no unverified dynamics.
            mujoco.mj_forward(model, data)
            viewer.sync()
            time.sleep(1.0 / 60.0)


if __name__ == "__main__":
    main()

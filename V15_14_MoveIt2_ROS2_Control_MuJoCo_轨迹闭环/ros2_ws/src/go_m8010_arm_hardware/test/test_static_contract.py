import ast
from pathlib import Path
import unittest


PACKAGE = Path(__file__).resolve().parents[1] / "go_m8010_arm_hardware"


class StaticContractTest(unittest.TestCase):
    def test_state_node_has_no_command_or_motion_api(self):
        source = (PACKAGE / "whole_arm_state_node.py").read_text(encoding="utf-8")
        forbidden = ("create_client", "create_service", "ActionServer", "SerialPort", "commandJoint", "enterFoc")
        for token in forbidden:
            self.assertNotIn(token, source)
        ast.parse(source)

    def test_mirror_is_direct_qpos_and_forward(self):
        source = (PACKAGE / "mujoco_mirror_node.py").read_text(encoding="utf-8")
        self.assertIn("self.data.qpos[self.qpos_addresses] = target", source)
        self.assertIn("self.mujoco.mj_forward", source)
        for token in ("mj_step", "ctrl[", "PID"):
            self.assertNotIn(token, source)


if __name__ == "__main__":
    unittest.main()

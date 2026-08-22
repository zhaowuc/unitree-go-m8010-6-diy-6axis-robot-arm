import ast
from pathlib import Path
import unittest


PACKAGE = Path(__file__).resolve().parents[1] / "go_m8010_arm_hardware"
REPOSITORY = next(
    parent for parent in Path(__file__).resolve().parents
    if (parent / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp").is_file()
)


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

    def test_go_worker_enforces_per_motor_active_mask_across_full_position_path(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            'go-m8010-gui-command/1.1',
            "candidate.active_joint_mask[i]",
            "if (!command.active_joint_mask[index])",
            "command.active_joint_mask[static_cast<std::size_t>(motor.joint_index)]",
            'controller_mode_by_motor[motor.name]',
        ):
            self.assertIn(token, source)

    def test_state_node_prefers_per_motor_controller_mode(self):
        source = (PACKAGE / "whole_arm_state_node.py").read_text(encoding="utf-8")
        self.assertIn('payload.get("controller_mode_by_motor", {})', source)
        self.assertIn("controller_mode_by_motor.get(sample.motor, controller_mode)", source)


if __name__ == "__main__":
    unittest.main()

#!/usr/bin/env python3

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[2]


class GoAuxSessionPermitContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.controller = (ROOT / "tools/hardware/v15_30a_gui_go_controller.cpp").read_text(
            encoding="utf-8"
        )
        cls.supervisor = (ROOT / "start_arm_gui.sh").read_text(encoding="utf-8")
        cls.state_model = (
            ROOT
            / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws/src"
            / "go_m8010_arm_hardware/go_m8010_arm_hardware/state_model.py"
        ).read_text(encoding="utf-8")
        cls.state_node = (
            ROOT
            / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws/src"
            / "go_m8010_arm_hardware/go_m8010_arm_hardware/whole_arm_state_node.py"
        ).read_text(encoding="utf-8")

    def test_brake_capture_never_loads_prior_session_branch_inputs(self) -> None:
        start = self.controller.index("std::string actual_zero_sha256;")
        load = self.controller.index("load_persistent_zero(options.zero_file, motors)", start)
        guard = self.controller.rindex("if (!options.brake_only)", start, load)
        serial_open = self.controller.index("SerialPort serial(", load)
        self.assertLess(guard, load)
        self.assertLess(load, serial_open)
        self.assertIn("BRAKE_ONLY_CAPABILITY=YES", self.controller)

    def test_aux_permit_is_consumed_before_serial_and_spent_before_udp_bind(self) -> None:
        run = self.controller[self.controller.index("int run(const Options& options)") :]
        consume = run.index("consume_j2_launch_permit(j2_launch_permit)")
        serial_open = run.index("SerialPort serial(")
        verify = run.index("GO_AUX_STARTUP_BRAKE_RECHECK_FAILED")
        spend = run.index("spend_j2_launch_permit(j2_launch_permit)", verify)
        bind = run.index("command_socket = open_command_socket(definition)", spend)
        self.assertLess(consume, serial_open)
        self.assertLess(spend, bind)
        for token in (
            "go-m8010-go-aux-power-session-reference/1.0",
            "go-m8010-go-aux-power-session-launch-permit/1.0",
            "GO_AUX_STARTUP_SESSION_VERIFIED",
            "GO_AUX_LAUNCH_PERMIT_STATE",
            "samples < 500",
        ):
            self.assertIn(token, self.controller)

    def test_supervisor_prebuild_and_signed_launch_are_fail_closed(self) -> None:
        for token in (
            "--prebuild-only",
            "PREBUILD_ONLY=PASS",
            "SERIAL_OPENED=NO",
            "UDP_OPENED=NO",
            "ROS_LAUNCHED=NO",
            "/tmp/v15_30a_gui_prebuild.lock",
            "/tmp/v15_30a_gui_j1.lock",
            "/tmp/v15_30a_gui_j2.lock",
            "/tmp/v15_30a_gui_j345.lock",
            "/tmp/v15_30a_gui_j6.lock",
            "clear_go_aux_session_inputs",
            "v15_30a_validate_go_aux_session_bundle.py",
            "--audit-go-aux-session-bundle",
            "GO_AUX_SESSION_BUNDLE_AUDIT=PASS",
            "go_aux_session_reference_path:=\"$GO_AUX_SESSION_REFERENCE\"",
            "--go-aux-session-launch-permit-file \"$GO_AUX_J1_LAUNCH_PERMIT\"",
            "--go-aux-session-launch-permit-file \"$GO_AUX_J345_LAUNCH_PERMIT\"",
            "完整签名启动拒绝故障域降级",
        ):
            self.assertIn(token, self.supervisor)
        lock = self.supervisor.index(
            'flock -n "$LEGACY_FEEDBACK_LOCK_FD" || fail "旧的全臂反馈进程'
        )
        cleanup_trap = self.supervisor.index("trap cleanup EXIT")
        self.assertLess(lock, cleanup_trap)
        prebuild_exit = self.supervisor.index("PREBUILD_ONLY=PASS")
        ros_launch = self.supervisor.index("exec ros2 launch")
        self.assertLess(prebuild_exit, ros_launch)
        self.assertIn('if [[ "$HARDWARE_SESSION_STARTED" -eq 1 ]]', self.supervisor)

    def test_state_pipeline_requires_current_power_aux_reference(self) -> None:
        for token in (
            'GO_AUX_SESSION_MOTORS = frozenset({"J1", "J3", "J4", "J5"})',
            "reference_blocked_motors",
            "go_aux_session_references",
            "GO-AUX logical positions intentionally come from the hash-bound",
        ):
            self.assertIn(token, self.state_model)
        for token in (
            "load_go_aux_session_reference",
            "recovery_hints_used_for_go_aux",
            "initial_pose_sha256",
            "sample_count < 500",
            "go_aux_session_reference_sha256",
        ):
            self.assertIn(token, self.state_node)


if __name__ == "__main__":
    unittest.main()

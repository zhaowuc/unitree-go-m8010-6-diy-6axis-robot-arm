"""无需 ROS2、Qt 或硬件即可验证 GUI 的反馈准入逻辑。"""

import ast
from pathlib import Path


SOURCE = (
    Path(__file__).resolve().parents[1]
    / "go_m8010_arm_gui"
    / "main_window.py"
)


def load_function(name):
    tree = ast.parse(SOURCE.read_text(encoding="utf-8"))
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )
    module = ast.Module(body=[function], type_ignores=[])
    namespace = {}
    exec(compile(ast.fix_missing_locations(module), str(SOURCE), "exec"), namespace)
    return namespace[name]


def test_partial_joint_health_allows_isolated_motion_actions():
    ready = load_function("control_feedback_ready")
    assert ready(True, True, [False, False, False, True, False, False])


def test_motion_action_still_requires_fresh_streams_and_one_healthy_joint():
    ready = load_function("control_feedback_ready")
    assert not ready(False, True, [True, False, False, False, False, False])
    assert not ready(True, False, [True, False, False, False, False, False])
    assert not ready(True, True, [False] * 6)


def test_complete_pose_actions_require_all_six_healthy():
    ready = load_function("control_feedback_ready")
    assert not ready(True, True, [True, True, True, True, True, False], require_all=True)
    assert ready(True, True, [True] * 6, require_all=True)


def test_active_mask_excludes_j2_and_disconnected_joints():
    mask = load_function("effective_active_joint_mask")
    assert mask(
        [True] * 6,
        [True, True, False, True, False, True],
        "hold",
    ) == [True, False, False, True, False, True]


def test_brake_always_clears_active_mask():
    mask = load_function("effective_active_joint_mask")
    assert mask([True] * 6, [True] * 6, "brake") == [False] * 6


def test_connection_loss_clears_latched_bit_and_recovery_does_not_restore_it():
    clear = load_function("clear_mask_on_connection_loss")
    dropped = clear(
        [False, False, False, True, False, False],
        [True] * 6,
        [True, True, True, False, True, True],
    )
    assert dropped == [False] * 6
    recovered = clear(dropped, [True, True, True, False, True, True], [True] * 6)
    assert recovered == [False] * 6


def test_initially_disconnected_joint_cannot_be_prearmed_for_recovery():
    mask = load_function("effective_active_joint_mask")
    connected_at_action = [True, True, True, False, True, True]
    requested = list(connected_at_action)
    assert mask(requested, connected_at_action, "hold")[3] is False
    assert mask(requested, [True] * 6, "hold")[3] is False
    assert "self.requested_active_joint_mask = [True] * 6" not in SOURCE.read_text(
        encoding="utf-8"
    )


def test_target_edit_is_pending_until_execute_authorizes_new_epoch():
    source = SOURCE.read_text(encoding="utf-8")
    assert "self.pending_target_joint_mask[index] = (" in source
    assert "self._authorize_active_joints(list(self.pending_target_joint_mask))" in source
    assert "self.activation_epoch += 1" in source
    assert "time.monotonic_ns()" in source


def test_execute_consumes_selection_and_connection_loss_clears_pending_bit():
    source = SOURCE.read_text(encoding="utf-8")
    execute_start = source.index("    def _execute_target(self) -> None:")
    execute_end = source.index("    def _stop(self) -> None:", execute_start)
    execute_source = source[execute_start:execute_end]
    assert "self.pending_target_joint_mask = [False] * 6" in execute_source

    apply_start = source.index("    def _apply_connection_state(")
    apply_end = source.index("    def _authorize_active_joints(", apply_start)
    apply_source = source[apply_start:apply_end]
    assert "self.pending_target_joint_mask = clear_mask_on_connection_loss(" in apply_source


def test_command_protocol_requires_versioned_per_joint_mask():
    source = SOURCE.read_text(encoding="utf-8")
    assert '"schema": "go-m8010-gui-command/1.1"' in source
    assert '"active_joint_mask": active_joint_mask' in source
    assert '"activation_epoch": activation_epoch' in source


def test_ros_operation_is_skipped_after_shutdown():
    run = load_function("run_ros_context_operation")
    calls = []
    assert not run(lambda: False, lambda: calls.append("called"))
    assert calls == []


def test_shutdown_race_converts_ros_exception_to_clean_exit():
    run = load_function("run_ros_context_operation")
    states = iter((True, False))

    def fail_during_shutdown():
        raise RuntimeError("ROS context is not valid")

    assert not run(lambda: next(states), fail_during_shutdown)


def test_normal_runtime_exception_is_not_swallowed():
    run = load_function("run_ros_context_operation")

    def fail_while_running():
        raise RuntimeError("unexpected GUI failure")

    try:
        run(lambda: True, fail_while_running)
    except RuntimeError as error:
        assert str(error) == "unexpected GUI failure"
    else:
        raise AssertionError("运行时异常被错误吞掉")


def test_shutdown_after_successful_ros_operation_requests_exit():
    run = load_function("run_ros_context_operation")
    states = iter((True, False))
    calls = []
    assert not run(lambda: next(states), lambda: calls.append("called"))
    assert calls == ["called"]

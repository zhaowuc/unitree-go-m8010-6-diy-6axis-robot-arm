"""Offscreen Qt checks: no ROS, MuJoCo, transport, or real hardware command."""

import math
import os
import time
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest

pytest.importorskip("PySide6")
from PySide6.QtWidgets import QApplication, QWidget

from go_m8010_arm_gui import action_group_dialog as ui
from go_m8010_arm_gui.action_groups import ActionStep, PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG
from go_m8010_arm_gui.state_machine import ModeMachine, ArrivalTracker
from go_m8010_arm_gui.workflow_contract import WorkflowState
from test_main_window_logic import hardware_state, load_function, load_main_window_method


APP = QApplication.instance() or QApplication([])


class Window(QWidget):
    def __init__(self, directory):
        super().__init__()
        self.calls = []
        self.config = {"控制": {"最大速度_度每秒": 3.0}}
        self.arrival = ArrivalTracker(math.radians(0.1), 0.5, 90,
                                      per_joint_tolerance_rad=(math.radians(0.1),) * 5 + (math.radians(0.08),))
        self.machine = ModeMachine()
        self.node = SimpleNamespace(log_directory=Path(directory))
        self.session_pose_deg = [0, 90, -14.4, 13.49, 47.94, 0]
        self.absolute_limits = PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG
        limits_rad = tuple((math.radians(lo - anchor), math.radians(hi - anchor))
                           for (lo, hi), anchor in zip(self.absolute_limits, self.session_pose_deg))
        self.actual = [0.0] * 6
        self.command_targets = self.actual[:]
        self.candidate_targets = self.actual[:]
        self.workflow_contract = WorkflowState.initialize(self.actual, limits_rad, "a" * 64)
        self.hardware_mode = "hold"
        self.queued_pose_target = None
        self.pending_collision_execute_sequence = None
        self.preview_requested_by_operator = False
        self.collision_preview_state = "idle"
        self.approved = False
        self.held = True
        self.binding = ("session", "b" * 32, "c" * 64)
        self.action_group_dialog = ui.ActionGroupDialog(self)

    def _action_group_health(self, binding=None, _rejected=None):
        if binding is not None and binding != self.binding:
            raise RuntimeError("session changed")
        return {"binding": self.binding, "rejected_commands": 0}

    def _action_group_hold_ready(self, _target):
        return self.held

    def _require_control_feedback(self, *_args, **_kwargs):
        return True

    def _refresh_virtual_editability(self):
        pass

    def _show_targets_on_virtual(self):
        pass

    def _clear_candidate_approval(self):
        self.approved = False
        self.collision_preview_state = "idle"

    def _start_virtual_preview(self):
        self.calls.append("preview")
        self.preview_requested_by_operator = True
        self.collision_preview_state = "planning"

    def _preview_approval_matches_candidate(self):
        return self.approved

    def _execute_target(self):
        self.calls.append("execute")
        self.queued_pose_target = self.candidate_targets[:]
        self.collision_preview_state = "execute_proof"

    def _stop(self):
        self.calls.append("stop")
        self.queued_pose_target = None


def test_capture_model_angles_and_reject_pwm_group_before_hardware(tmp_path):
    window = Window(tmp_path)
    dialog = window.action_group_dialog
    dialog.capture(False)
    assert dialog.group().steps[0].target_deg == tuple(window.session_pose_deg)
    dialog.table.item(0, 8).setText("close")
    dialog.start()
    assert dialog.runner is None
    assert "控制板尚未接入" in dialog.status.text()
    assert window.calls == []
    assert window.config["控制"]["最大速度_度每秒"] == 3
    window.close()


def test_real_adapter_waits_for_preview_and_measured_hold_then_restores_settings(tmp_path, monkeypatch):
    clock = [100.0]
    monkeypatch.setattr(ui, "time", SimpleNamespace(monotonic=lambda: clock[0]))
    window = Window(tmp_path)
    dialog = window.action_group_dialog
    strict_arrival = window.arrival
    target = window.session_pose_deg[:]
    target[0] = 2
    dialog.append_step(ActionStep(tuple(target), speed_deg_s=1))
    dialog.start()
    assert window.calls == ["preview"]
    assert dialog.blocks_manual
    assert window.config["控制"]["最大速度_度每秒"] == 1
    assert window.arrival.tolerance_rad == math.radians(0.25)
    assert window.candidate_targets == pytest.approx([math.radians(2)] + [0] * 5)
    window.approved = True
    dialog.tick()
    assert window.calls == ["preview", "execute"]
    window.collision_preview_state = "complete"
    window.queued_pose_target = None
    window.held = False
    dialog.tick()
    assert dialog.runner.state == "moving"
    window.held = True
    window.actual = window.candidate_targets[:]
    dialog.tick()
    clock[0] += 0.5
    dialog.tick()
    dialog.tick()
    assert dialog.runner.state == "complete"
    assert window.arrival is strict_arrival
    assert window.config["控制"]["最大速度_度每秒"] == 3
    assert '"measured_arrival"' in dialog.log_path.read_text(encoding="utf-8")
    window.close()


def test_pause_replans_and_bound_session_change_stops_without_resume(tmp_path):
    window = Window(tmp_path)
    dialog = window.action_group_dialog
    target = window.session_pose_deg[:]
    target[1] += 1
    dialog.append_step(ActionStep(tuple(target)))
    dialog.start()
    dialog.pause()
    assert window.calls == ["preview", "stop"]
    window.held = False
    dialog.tick()
    assert not dialog.resume_button.isEnabled()
    dialog.resume()
    assert dialog.runner.state == "paused"
    assert window.calls == ["preview", "stop"]
    window.held = True
    dialog.tick()
    assert dialog.resume_button.isEnabled()
    dialog.resume()
    assert window.calls == ["preview", "stop", "preview"]
    window.binding = ("new-session", *window.binding[1:])
    dialog.check_health()
    assert dialog.runner.state == "failed"
    dialog.resume()
    assert window.calls == ["preview", "stop", "preview", "stop"]
    assert not dialog.active
    window.close()


def test_manual_entry_points_refuse_action_group_ownership():
    blocked = SimpleNamespace(action_group_dialog=SimpleNamespace(blocks_manual=True))
    for name in ("_hold_current", "_execute_target", "_load_acceptance_target",
                 "_return_initial_pose", "_start_virtual_preview", "_drag_mode"):
        load_main_window_method(name)(blocked)


def test_main_health_rejects_binding_router_feedback_and_authority_changes():
    health = load_main_window_method("_action_group_health", {
        name: load_function(name, {"ROUTER_REJECTION_WARNING_WINDOW_MS": 5000.0})
        for name in ("hardware_state_source_is_fresh", "classify_logical_joint_observations",
                     "command_router_status_text", "receipt_is_fresh")
    })
    checks = SimpleNamespace(feedback_fresh=True, communication_pass=True,
                             thermal_pass=True, gravity_pass=True)
    hardware = hardware_state()
    window = SimpleNamespace(
        have_first_state=True, command_stream_suspended=False, hardware_mode="hold",
        requested_active_joint_mask=[True] * 6, session_pose_sha256="d" * 64,
        session_id=hardware["session_id"], state_instance_id=hardware["state_instance_id"],
        config={"控制": {"命令租约_秒": 0.5}}, _current_preview_checks=lambda _: checks,
        node=SimpleNamespace(
            latest_hardware=hardware, control_streams_fresh=lambda _: True,
            control_status_fresh=lambda _: True, latest_acceptance_status={},
            last_acceptance_status_receipt=0.0,
            latest_control_status={"received": True, "rejected_commands": 0,
                                   "last_mode": "hold", "last_command_age_ms": 10},
        ),
    )
    baseline = health(window)
    hardware["persistent_zero_sha256"] = "e" * 64
    with pytest.raises(RuntimeError, match="软件零位"):
        health(window, **baseline)
    hardware["persistent_zero_sha256"] = baseline["binding"][2]
    window.node.latest_control_status["rejected_commands"] = 1
    with pytest.raises(RuntimeError, match="命令路由"):
        health(window, **baseline)
    window.node.latest_control_status["rejected_commands"] = 0
    hardware["per_motor"]["J2B"]["fresh"] = False
    with pytest.raises(RuntimeError, match="七电机"):
        health(window, **baseline)
    hardware["per_motor"]["J2B"]["fresh"] = True
    checks.gravity_pass = False
    with pytest.raises(RuntimeError, match="authority"):
        health(window, **baseline)
    checks.gravity_pass = True
    window.node.latest_acceptance_status = {"result": "RUNNING", "gravity_ladder": {"active": True}}
    window.node.last_acceptance_status_receipt = time.monotonic()
    with pytest.raises(RuntimeError, match="独立验收"):
        health(window, **baseline)
    window.node.last_acceptance_status_receipt = 0.0
    assert health(window, **baseline) == baseline
    hardware["source_monotonic_ns"] = time.monotonic_ns() - 5_000_000_000
    with pytest.raises(RuntimeError, match="过期"):
        health(window, **baseline)

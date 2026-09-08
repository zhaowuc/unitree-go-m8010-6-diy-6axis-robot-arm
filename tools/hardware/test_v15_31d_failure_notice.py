"""Failure cleanup order and a real, control-free Qt result notice; no hardware."""
import json
import os
from pathlib import Path
from types import SimpleNamespace

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import v15_31d_demo_session as launcher


def test_native_fault_notice_runs_only_after_cleanup_and_report(tmp_path, monkeypatch):
    session, scripts = tmp_path / "session", tmp_path / "scripts"
    (session / "run").mkdir(parents=True)
    (session / "evidence").mkdir()
    (session / "run/j1_controller.log").write_text(
        "ASSISTED_TEACH_EXIT_HOLD reason=VELOCITY_LIMIT\n"
        "ASSISTED_TEACH_RESTRICTED_HOLD joint=J1\n"
        "ASSISTED_TEACH_STOP_ERROR_LIMIT bus=j1\nFINAL_BRAKE=PASS\n", encoding="utf-8")
    (session / "evidence/j1_action_group_demo.json").write_text(json.dumps({
        "status": "FAIL", "failure": "feedback, authority, session or command-router health changed"}), encoding="utf-8")
    order = []
    def stage(command, *_):
        if Path(command[1]).name == "start_bounded_j1_demo.sh":
            order.append("runner-exited")
            raise RuntimeError("exit 1")
    def cleanup(*_):
        order.append("cleanup-complete")
        return {"controller_terminal_confirmed": dict.fromkeys(("J1", "J2", "J345", "J6"), True), "stop_errors": []}
    def popen(command, **kwargs):
        assert order == ["runner-exited", "cleanup-complete"]
        assert command[-2] == "--failure-notice"
        summary = json.loads(Path(command[-1]).read_text(encoding="utf-8"))
        assert summary["native_failures"][0]["reason"] == "ASSISTED_TEACH_STOP_ERROR_LIMIT"
        assert len(summary["native_failures"]) == 1
        assert kwargs["start_new_session"] and kwargs["close_fds"]
        order.append("notice-spawned")
        return SimpleNamespace(pid=1)
    monkeypatch.setenv("GO_ASSISTED_TEACH", "0")
    monkeypatch.setenv("GO_TEACH_OBSERVE", "0")
    monkeypatch.setattr(launcher, "idle_demo_cores", lambda: [])
    monkeypatch.setattr(launcher, "run_stage", stage)
    monkeypatch.setattr(launcher, "cleanup", cleanup)
    monkeypatch.setattr(launcher.signal, "signal", lambda *_: None)
    monkeypatch.setattr(launcher.subprocess, "check_output", lambda *_args, **_kwargs: "")
    monkeypatch.setattr(launcher.subprocess, "Popen", popen)
    args = SimpleNamespace(assisted_teach=True, teach_observe=False, bash="bash", power_cycled=False,
        cycles=1, supported_near_vertical_recovery=False, excursion_deg=1.0, speed_deg_s=1.0,
        symmetric=False, return_center=False)
    assert launcher.execute(args, {}, {"maximum_demo_seconds": 600}, session, scripts, "test") == 1
    assert order == ["runner-exited", "cleanup-complete", "notice-spawned"]

    # Exercise actual QMessageBox creation/exec, inspect it, then close it.
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication, QMessageBox
    app = QApplication.instance() or QApplication([])
    shown = []
    def close_notice():
        box = app.activeModalWidget()
        shown.append((box.text(), box.standardButtons(), isinstance(box, QMessageBox)))
        box.accept()
    QTimer.singleShot(0, close_notice)
    assert launcher.show_failure_notice(scripts / "session_result.json") == 0
    assert shown[0][2] and "J1" in shown[0][0] and "ASSISTED_TEACH_STOP_ERROR_LIMIT" in shown[0][0]
    assert "最终制动／禁用均已确认" in shown[0][0] and "不发送控制命令" in shown[0][0]
    assert shown[0][1] == QMessageBox.Close

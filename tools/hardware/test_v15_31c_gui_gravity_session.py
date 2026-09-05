import sys
import json
import signal
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parent))
import v15_31c_gui_gravity_session as session
from v15_31c_gui_gravity_session import gui_hold_ready


def test_hold_requires_all_seven_motors_and_fresh_router_acceptance():
    motors = dict.fromkeys(("J1", "J2A", "J2B", "J3", "J4", "J5", "J6"), "hold")
    hardware = {"controller_mode_by_motor": motors}
    router = {"last_mode": "hold", "last_command_age_ms": 1}
    assert gui_hold_ready(hardware, router)
    assert not gui_hold_ready(hardware, {**router, "last_command_age_ms": 251})
    assert not gui_hold_ready(hardware, {**router, "last_mode": "brake"})
    motors["J4"] = "unknown"
    assert not gui_hold_ready(hardware, router)


@pytest.mark.parametrize("comparison_failure", [None, "COMPARISON_WITH_FF_POSE_DIFFERS_FROM_WITHOUT_FF"])
def test_session_reuses_comparison_target_and_waits_for_runner(monkeypatch, comparison_failure):
    """Exercise the coordinator without ROS/hardware, including noisy HOLD feedback."""
    clock = [0.0]
    controls, output, callbacks, handlers = [], [], {}, {}
    hardware = {
        "session_id": "session", "state_instance_id": "state",
        "position_rad": [0.0] * 6,
        "controller_mode_by_motor": dict.fromkeys(("J1", "J2A", "J2B", "J3", "J4", "J5", "J6"), "hold"),
    }
    gravity = {"empirical_validation": {"stage_index": 0, "stage_complete": True}}
    runner = {"comparison": {"complete_conditions": []}, "gravity_ladder": {"complete": False}}
    position_wait_spins = [0]

    def publish(message):
        payload = json.loads(message.data)
        action = payload.get("action")
        if not action:
            return
        controls.append(payload)
        if action == "START_J2_COMPARISON":
            runner["comparison"]["active_condition"] = payload["condition"]
            if payload["condition"] == "WITH_FF":
                runner["failure"] = comparison_failure
        elif action == "STOP_J2_COMPARISON":
            runner["comparison"]["complete_conditions"].append(runner["comparison"]["active_condition"])
            hardware["position_rad"][1] += 0.00002
        elif action == "START_POSITION":
            runner["position"] = {"started": True, "awaiting_gui_command": False}

    def scale(request):
        level = request.parameters[0].value.double_value
        gravity["empirical_validation"]["stage_index"] = int(level * 4)
        runner["gravity_ladder"]["complete"] = level == 1.0
        return SimpleNamespace(done=lambda: True, result=lambda: SimpleNamespace(results=[SimpleNamespace(successful=True)]))

    def spin_once(_node, timeout_sec):
        clock[0] += 0.1
        if "position" in runner:
            position_wait_spins[0] += 1
            runner["position"]["awaiting_gui_command"] = position_wait_spins[0] >= 3
        states = (hardware, {"last_mode": "hold", "last_command_age_ms": 1}, gravity, runner)
        for callback, state in zip(callbacks.values(), states):
            callback(SimpleNamespace(data=json.dumps(state)))

    def record_output(message, **_kwargs):
        output.append(message)
        if message.startswith("GUI_PREVIEW_READY"):
            assert runner["position"]["awaiting_gui_command"] is True
            handlers[signal.SIGTERM]()

    node = SimpleNamespace(
        create_publisher=lambda *_args: SimpleNamespace(publish=publish),
        create_subscription=lambda _type, topic, callback, _qos: callbacks.update({topic: callback}),
        create_client=lambda *_args: SimpleNamespace(wait_for_service=lambda **_kwargs: True, call_async=scale),
        destroy_node=lambda: None,
    )
    monkeypatch.setitem(sys.modules, "rclpy", SimpleNamespace(init=lambda: None, shutdown=lambda: None,
                        create_node=lambda _name: node, spin_once=spin_once))
    monkeypatch.setitem(sys.modules, "rcl_interfaces.msg", SimpleNamespace(
        Parameter=SimpleNamespace, ParameterType=SimpleNamespace(PARAMETER_DOUBLE=3), ParameterValue=SimpleNamespace))
    monkeypatch.setitem(sys.modules, "rcl_interfaces.srv", SimpleNamespace(SetParameters=SimpleNamespace(Request=SimpleNamespace)))
    monkeypatch.setitem(sys.modules, "std_msgs.msg", SimpleNamespace(String=SimpleNamespace))
    monkeypatch.setattr(session, "time", SimpleNamespace(monotonic=lambda: clock[0], time=lambda: 0))
    monkeypatch.setattr(session.signal, "signal", lambda signum, handler: handlers.update({signum: handler}))
    monkeypatch.setattr(session, "print", record_output, raising=False)
    binding = SimpleNamespace(session_id="session", state_instance_id="state", envelope_id="env",
        envelope_sha256="e" * 64, anchor_sha256="a" * 64, ensure_not_expired=lambda: None,
        expires_at_utc=SimpleNamespace(timestamp=lambda: 10000))
    if comparison_failure:
        with pytest.raises(RuntimeError, match=comparison_failure):
            session.run(binding)
        assert not any(line.startswith("GUI_PREVIEW_READY") for line in output)
    else:
        session.run(binding)
        assert any(line.startswith("GUI_PREVIEW_READY") for line in output)
    comparisons = [item for item in controls if item["action"] == "START_J2_COMPARISON"]
    assert [item["condition"] for item in comparisons] == ["WITHOUT_FF", "WITH_FF"]
    assert comparisons[0]["target_j2_rad"] == comparisons[1]["target_j2_rad"] == 0.0
    assert hardware["position_rad"][1] > 0.0
    assert controls[-1]["action"] == "STOP_AND_BRAKE"

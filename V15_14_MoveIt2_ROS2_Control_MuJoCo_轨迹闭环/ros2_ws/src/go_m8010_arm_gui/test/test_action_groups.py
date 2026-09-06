from copy import deepcopy

import pytest

from go_m8010_arm_gui.action_groups import ActionGroup, ActionGroupRunner, ActionStep


MODEL = "a" * 64


def test_action_file_round_trip_and_rejects_invalid_input(tmp_path):
    group = ActionGroup("抓放演示", (ActionStep((0,) * 6, gripper="set", opening_percent=35),), MODEL)
    path = tmp_path / "演示.json"
    group.save(path)
    assert "抓放演示" in path.read_text(encoding="utf-8")
    assert ActionGroup.load(path, expected_model_sha256=MODEL) == group
    for field, value in (("speed_deg_s", True), ("speed_deg_s", 0),
                         ("speed_deg_s", 3.1), ("dwell_s", float("nan")),
                         ("gripper_wait_s", 31), ("opening_percent", 101),
                         ("target_deg", [0] * 5), ("target_deg", [181] + [0] * 5),
                         ("target_deg", [True] + [0] * 5), ("gripper", "J6")):
        data = deepcopy(group.to_dict())
        data["steps"][0][field] = value
        with pytest.raises(ValueError):
            ActionGroup.from_dict(data)
    with pytest.raises(ValueError):
        ActionGroup.load(path, expected_model_sha256="b" * 64)
    data = group.to_dict()
    data["coordinate_frame"] = "session_relative_deg"
    with pytest.raises(ValueError):
        ActionGroup.from_dict(data)
    path.write_text('{"name":"a","name":"b"}', encoding="utf-8")
    with pytest.raises(ValueError, match="duplicate"):
        ActionGroup.load(path)


def test_failed_atomic_replace_preserves_saved_action_group(tmp_path, monkeypatch):
    import go_m8010_arm_gui.action_groups as module

    group = ActionGroup("原动作组", (ActionStep((0,) * 6),), MODEL)
    path = tmp_path / "actions.json"
    group.save(path)
    original = path.read_bytes()

    def fail_replace(*_):
        raise OSError("disk unavailable")

    monkeypatch.setattr(module.os, "replace", fail_replace)
    with pytest.raises(OSError):
        ActionGroup("修改", group.steps, MODEL).save(path)
    assert path.read_bytes() == original
    assert list(tmp_path.iterdir()) == [path]


def harness(steps, **kwargs):
    clock = [0.0]
    calls = []
    status = ["running"]
    health = [None]

    def check_health():
        if health[0]:
            raise RuntimeError(health[0])

    runner = ActionGroupRunner(
        ActionGroup("演示", tuple(steps), MODEL),
        lambda step: calls.append(("move", step.target_deg)),
        lambda: status[0], lambda: calls.append(("stop",)),
        lambda step: calls.append(("gripper", step.gripper)) or "simulated command sent",
        check_health, now=lambda: clock[0], **kwargs,
    )
    return runner, clock, calls, status, health


def test_move_then_gripper_then_dwell_then_next_move():
    runner, clock, calls, status, _ = harness([
        ActionStep((0,) * 6, gripper="close", gripper_wait_s=2, dwell_s=3),
        ActionStep((1,) * 6),
    ])
    runner.start()
    runner.tick()
    assert calls == [("move", (0.0,) * 6)]
    status[0] = "complete"
    runner.tick()
    assert runner.state == "gripper_wait"
    assert calls[-1] == ("gripper", "close")
    clock[0] = 1.9
    runner.tick()
    assert runner.state == "gripper_wait"
    clock[0] = 2
    runner.tick()
    assert runner.state == "dwell"
    clock[0] = 5
    runner.tick()
    assert calls[-1] == ("move", (1.0,) * 6)
    runner.tick()
    runner.tick()
    assert runner.state == "complete"
    assert runner.result["completed_steps"] == 2


def test_move_pause_stops_and_resume_replans_same_step():
    runner, clock, calls, _, _ = harness([ActionStep((0,) * 6)], motion_timeout_s=10)
    runner.start()
    clock[0] = 3
    runner.pause()
    assert calls[-1] == ("stop",)
    clock[0] = 100
    runner.tick()
    runner.resume()
    assert [call[0] for call in calls] == ["move", "stop", "move"]
    assert runner.index == 0
    clock[0] = 107
    runner.tick()
    assert runner.state == "failed"
    assert calls[-1] == ("stop",)


@pytest.mark.parametrize("phase", ["gripper_wait", "dwell"])
def test_wait_pause_preserves_time_and_never_repeats_gripper(phase):
    runner, clock, calls, status, _ = harness([
        ActionStep((0,) * 6, gripper="open", gripper_wait_s=4, dwell_s=4),
    ])
    runner.start()
    status[0] = "complete"
    runner.tick()
    if phase == "dwell":
        clock[0] = 4
        runner.tick()
    clock[0] += 1
    runner.pause()
    clock[0] += 100
    runner.tick()
    runner.resume()
    clock[0] += 2.9
    runner.tick()
    assert runner.state == phase
    clock[0] += 0.1
    runner.tick()
    assert runner.state != phase
    assert sum(call[0] == "gripper" for call in calls) == 1


def test_stop_and_paused_health_fault_never_advance_or_recover():
    for fault in (False, True):
        runner, clock, calls, status, health = harness([
            ActionStep((0,) * 6, gripper="close"), ActionStep((1,) * 6),
        ])
        runner.start()
        if fault:
            runner.pause()
            health[0] = "feedback stale"
            runner.tick()
            assert runner.state == "failed"
        else:
            runner.stop()
            assert runner.state == "stopped"
        status[0] = "complete"
        clock[0] = 100
        health[0] = None
        runner.resume()
        runner.tick()
        assert runner.index == 0
        assert not any(call[0] == "gripper" for call in calls)
        assert calls[-1] == ("stop",)
        with pytest.raises(RuntimeError):
            runner.start()


def test_gripper_dispatch_error_stops_without_next_step_or_retry():
    runner, _, calls, status, _ = harness([
        ActionStep((0,) * 6, gripper="close"), ActionStep((1,) * 6),
    ])

    def reject_gripper(step):
        calls.append(("gripper_attempt", step.gripper))
        raise RuntimeError("gripper transport disconnected")

    runner._gripper_command = reject_gripper
    runner.start()
    status[0] = "complete"
    runner.tick()
    runner.resume()
    runner.tick()
    assert runner.state == "failed"
    assert runner.index == 0
    assert [call[0] for call in calls] == ["move", "gripper_attempt", "stop"]
    assert "disconnected" in runner.result["detail"]

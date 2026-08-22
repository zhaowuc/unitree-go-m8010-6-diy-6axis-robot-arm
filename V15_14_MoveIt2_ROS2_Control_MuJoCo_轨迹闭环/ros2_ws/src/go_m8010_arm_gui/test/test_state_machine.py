import math

from go_m8010_arm_gui.state_machine import ArmMode, ArrivalTracker, ModeMachine


def test_modes_are_mutually_exclusive():
    machine = ModeMachine()
    machine.sim_to_real()
    assert machine.mode is ArmMode.SIM_TO_REAL
    machine.drag()
    assert machine.mode is ArmMode.DRAG and not machine.torque_enabled
    machine.position()
    assert machine.mode is ArmMode.POSITION and machine.torque_enabled
    machine.stop()
    assert machine.mode is ArmMode.BRAKE and not machine.torque_enabled


def test_arrival_requires_continuous_half_second():
    tracker = ArrivalTracker(math.radians(0.5), 0.5, 2.0)
    tracker.start(0.0)
    connected = [True] * 6
    assert tracker.update(0.0, [0.0] * 6, connected) == ["运动中"] * 6
    assert tracker.update(0.49, [0.0] * 6, connected) == ["运动中"] * 6
    assert tracker.update(0.50, [0.0] * 6, connected) == ["已到位"] * 6
    assert tracker.update(0.60, [0.0] * 6, connected) == ["保持中"] * 6


def test_timeout_never_fakes_arrival():
    tracker = ArrivalTracker(math.radians(0.5), 0.5, 1.0)
    tracker.start(0.0)
    states = tracker.update(1.1, [math.radians(4.4)] * 6, [True] * 6)
    assert states == ["未到位"] * 6

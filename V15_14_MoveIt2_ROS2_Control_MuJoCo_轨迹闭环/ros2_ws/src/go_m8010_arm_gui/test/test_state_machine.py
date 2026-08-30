import math

from go_m8010_arm_gui.state_machine import ArmMode, ArrivalTracker, ModeMachine


def test_modes_and_fixed_hold_after_arrival_policy_are_independent():
    machine = ModeMachine()
    assert machine.fixed_hold_after_arrival is True
    machine.set_fixed_hold_after_arrival(False)
    machine.sim_to_real()
    assert machine.mode is ArmMode.SIM_TO_REAL
    machine.drag()
    assert machine.mode is ArmMode.DRAG
    machine.position()
    assert machine.mode is ArmMode.POSITION
    machine.hold()
    assert machine.mode is ArmMode.HOLD
    machine.stop()
    assert machine.mode is ArmMode.BRAKE
    assert machine.fixed_hold_after_arrival is False
    machine.set_fixed_hold_after_arrival(True)
    assert machine.fixed_hold_after_arrival is True


def test_fixed_hold_after_arrival_policy_requires_strict_boolean():
    machine = ModeMachine()
    for invalid in (0, 1, "true", None):
        try:
            machine.set_fixed_hold_after_arrival(invalid)
        except TypeError:
            pass
        else:
            raise AssertionError(f"non-boolean policy was accepted: {invalid!r}")


def test_arrival_requires_continuous_half_second():
    tracker = ArrivalTracker(math.radians(0.5), 0.5, 2.0)
    tracker.start(0.0)
    connected = [True] * 6
    assert tracker.update(0.0, [0.0] * 6, connected) == ["运动中"] * 6
    assert tracker.update(0.49, [0.0] * 6, connected) == ["运动中"] * 6
    assert tracker.update(0.50, [0.0] * 6, connected) == ["已到位"] * 6
    assert tracker.update(0.60, [0.0] * 6, connected) == ["保持中"] * 6


def test_arrival_uses_stricter_j6_tolerance_without_changing_j1_to_j5():
    tracker = ArrivalTracker(
        math.radians(0.5),
        0.5,
        2.0,
        per_joint_tolerance_rad=tuple(
            [math.radians(0.5)] * 5 + [math.radians(0.08)]
        ),
    )
    tracker.start(0.0)
    connected = [True] * 6
    stationary_small_step = [math.radians(0.2)] * 6
    assert tracker.update(0.0, stationary_small_step, connected) == ["运动中"] * 6
    assert tracker.update(0.5, stationary_small_step, connected) == [
        "已到位", "已到位", "已到位", "已到位", "已到位", "运动中",
    ]

    near_target = [0.0] * 5 + [math.radians(0.07)]
    assert tracker.update(0.6, near_target, connected)[5] == "运动中"
    assert tracker.update(1.1, near_target, connected)[5] == "已到位"


def test_per_joint_arrival_tolerance_requires_six_values():
    try:
        ArrivalTracker(0.1, 0.5, 2.0, per_joint_tolerance_rad=(0.1,) * 5)
    except ValueError:
        pass
    else:
        raise AssertionError("accepted an incomplete per-joint tolerance vector")


def test_timeout_never_fakes_arrival():
    tracker = ArrivalTracker(math.radians(0.5), 0.5, 1.0)
    tracker.start(0.0)
    states = tracker.update(1.1, [math.radians(4.4)] * 6, [True] * 6)
    assert states == ["未到位"] * 6

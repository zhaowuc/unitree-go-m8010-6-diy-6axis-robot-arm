import pytest

from go_m8010_arm_hardware.thermal_manager import (
    MotorThermalManager,
    ThermalFleetManager,
    ThermalLimits,
    ThermalState,
    thermal_state_from_controller_metadata,
)


def make_limits(**overrides):
    values = {
        "normal_below_c": 45.0,
        "warning_below_c": 50.0,
        "derating_start_c": 55.0,
        "thermal_stop_c": 60.0,
        "rearm_below_c": 55.0,
        "cooldown_seconds": 30.0,
        "slope_window_seconds": 120.0,
    }
    values.update(overrides)
    return ThermalLimits(**values)


def test_temperature_bands_and_derating_are_explicit():
    manager = MotorThermalManager(make_limits())
    assert manager.observe(
        now_s=0.0, temperature_c=44, continuity_valid=True,
        activation_epoch=1,
    ).state is ThermalState.NORMAL
    assert manager.observe(
        now_s=1.0, temperature_c=50, continuity_valid=True,
        activation_epoch=1,
    ).state is ThermalState.WARNING
    derating = manager.observe(
        now_s=2.0, temperature_c=56, continuity_valid=True,
        activation_epoch=1,
    )
    assert derating.state is ThermalState.DERATING
    assert 0.0 < derating.derating_factor < 1.0


def test_single_60_then_59_never_automatically_resumes():
    manager = MotorThermalManager(make_limits(cooldown_seconds=30.0))
    trip = manager.observe(
        now_s=0.0, temperature_c=60, continuity_valid=True,
        activation_epoch=10,
    )
    assert trip.state is ThermalState.THERMAL_STOP
    assert trip.minimum_rearm_epoch == 11
    still_stopped = manager.observe(
        now_s=1.0, temperature_c=59, continuity_valid=True,
        activation_epoch=10,
    )
    assert still_stopped.state is ThermalState.THERMAL_STOP
    assert not still_stopped.motion_allowed

    cooling = manager.observe(
        now_s=2.0, temperature_c=54, continuity_valid=True,
        activation_epoch=10,
    )
    assert cooling.state is ThermalState.COOLDOWN
    ready = manager.observe(
        now_s=32.0, temperature_c=54, continuity_valid=True,
        activation_epoch=10,
    )
    assert ready.state is ThermalState.WAIT_OPERATOR_CONFIRM
    assert not manager.confirm_rearm(now_s=32.0, activation_epoch=10)
    assert manager.confirm_rearm(now_s=32.0, activation_epoch=11)
    assert manager.state is ThermalState.WARNING


def test_transport_loss_does_not_clear_thermal_latch():
    manager = MotorThermalManager(make_limits())
    manager.observe(
        now_s=0.0, temperature_c=61, continuity_valid=True,
        activation_epoch=2,
    )
    offline = manager.observe(
        now_s=1.0, temperature_c=None, continuity_valid=False,
        activation_epoch=2,
    )
    assert offline.state is ThermalState.OFFLINE
    assert offline.fault_latched
    assert not offline.motion_allowed


def test_j2_trip_latches_both_motors_and_requires_pair_cooldown():
    fleet = ThermalFleetManager(make_limits(cooldown_seconds=1.0))
    fleet.observe_motor(
        "J2A", now_s=0.0, temperature_c=60, continuity_valid=True,
        activation_epoch=7,
    )
    assert fleet.motors["J2A"].fault_latched
    assert fleet.motors["J2B"].fault_latched
    fleet.observe_motor(
        "J2A", now_s=1.0, temperature_c=54, continuity_valid=True,
        activation_epoch=7,
    )
    fleet.observe_motor(
        "J2B", now_s=1.0, temperature_c=54, continuity_valid=True,
        activation_epoch=7,
    )
    fleet.observe_motor(
        "J2A", now_s=2.0, temperature_c=54, continuity_valid=True,
        activation_epoch=7,
    )
    fleet.observe_motor(
        "J2B", now_s=2.0, temperature_c=54, continuity_valid=True,
        activation_epoch=7,
    )
    assert fleet.confirm_domain_rearm("J2", now_s=2.0, activation_epoch=8)
    assert not fleet.motors["J2A"].fault_latched
    assert not fleet.motors["J2B"].fault_latched


def test_temperature_slope_uses_recent_window():
    manager = MotorThermalManager(make_limits(slope_window_seconds=120.0))
    manager.observe(
        now_s=0.0, temperature_c=40, continuity_valid=True,
        activation_epoch=1,
    )
    snapshot = manager.observe(
        now_s=60.0, temperature_c=41, continuity_valid=True,
        activation_epoch=1,
    )
    assert snapshot.slope_c_per_min == pytest.approx(1.0)


@pytest.mark.parametrize(
    ("temperature", "latched", "ready", "expected"),
    (
        (None, False, False, ThermalState.OFFLINE),
        (44, False, False, ThermalState.NORMAL),
        (45, False, False, ThermalState.WARNING),
        (55, False, False, ThermalState.DERATING),
        (60, False, False, ThermalState.THERMAL_STOP),
        (59, True, False, ThermalState.THERMAL_STOP),
        (54, True, False, ThermalState.COOLDOWN),
        (54, True, True, ThermalState.WAIT_OPERATOR_CONFIRM),
    ),
)
def test_controller_metadata_maps_to_shared_thermal_state(
    temperature, latched, ready, expected,
):
    assert thermal_state_from_controller_metadata(
        temperature,
        fault_latched=latched,
        cooldown_ready=ready,
        limits=make_limits(),
    ) is expected

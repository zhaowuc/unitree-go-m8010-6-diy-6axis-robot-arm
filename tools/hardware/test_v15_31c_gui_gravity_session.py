import sys
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
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

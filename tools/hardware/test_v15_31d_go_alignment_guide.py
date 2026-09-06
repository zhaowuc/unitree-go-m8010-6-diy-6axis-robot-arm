"""Offline: python tools/hardware/test_v15_31d_go_alignment_guide.py."""
import math
from v15_31d_go_alignment_guide import estimate, packet_values, snapshot, MOTORS


def test_diagnostic_only():
    record = {"session_reference_raw_rad": 1.0, "logical_position_rad": 0.0,
              "gear_ratio": 6.329999923706055, "sign": -1}
    raw = 1.0-record["gear_ratio"]*math.radians(2.2115)
    angle, selected = estimate(raw, record)
    assert abs(angle-2.2115) < 1e-10  # Outside startup window remains visible.
    moved, same = estimate(raw-record["gear_ratio"]*math.radians(60), record, selected)
    assert same == selected and abs(moved-62.2115) < 1e-9  # Never silently select another turn.
    for value in (math.nan, math.inf, True):
        try:
            estimate(value, record)
        except ValueError:
            pass
        else:
            raise AssertionError("accepted non-finite/boolean raw value")
    stamp = 1_000_000_000
    latest = {name: (1.6, stamp, stamp) for name in MOTORS}
    assert snapshot(latest, stamp)["within_original_window"]  # 1.5 is not required.
    latest["J4"] = (2.067, stamp, stamp)
    assert not snapshot(latest, stamp)["within_original_window"]
    latest["J4"] = (0.0, stamp, stamp)
    latest["J2B"] = (1.0, stamp, stamp)
    assert not snapshot(latest, stamp)["within_original_window"]  # Independent A/B difference.
    assert len(snapshot(latest, stamp+250_000_001)["stale"]) == 6
    packet = {"schema": "go-m8010-motor-feedback/1.0", "source_monotonic_ns": stamp,
              "controller_mode": "brake", "controller_mode_by_motor": {"J1": "brake"},
              "domain_fault": False, "j2_sync_fault": False, "lease_safe_hold": False,
              "samples": [{"motor": "J1", "communication_ok": True, "merror": 0,
                           "temperature_c": 30, "position_rad": raw, "velocity_rad_s": 0,
                           "unwrapped_raw_position_rad": raw}]}
    values, branches = packet_values(packet, ("J1",), {"J1": record}, {}, stamp)
    assert abs(values["J1"]-2.2115) < 1e-10
    packet["controller_mode"] = "hold"
    try:
        packet_values(packet, ("J1",), {"J1": record}, branches, stamp)
    except ValueError:
        pass
    else:
        raise AssertionError("accepted non-BRAKE feedback")


if __name__ == "__main__":
    test_diagnostic_only()
    print("GO_ALIGNMENT_GUIDE_OFFLINE=PASS")

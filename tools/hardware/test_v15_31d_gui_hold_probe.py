"""Runnable offline check: python tools/hardware/test_v15_31d_gui_hold_probe.py."""

from contextlib import redirect_stdout
from io import StringIO
from pathlib import Path
import sys
from types import SimpleNamespace

sys.path.insert(0, str(Path(__file__).resolve().parent))
from v15_31d_gui_hold_probe import HoldProbe, MOTORS, empirical_binding_matches, main


def test_current_pose_hold_probe():
    class Window:
        def __init__(self):
            self.actual = [0.0] * 6
            self.command_targets = [0.0] * 6
            self.hardware_mode = "brake"
            self.command_stream_suspended = True
            self.hold_calls = self.brake_calls = 0

        def _hold_current(self):
            self.hold_calls += 1
            self.command_targets = list(self.actual)
            self.hardware_mode, self.command_stream_suspended = "hold", False

        def _tick(self):
            pass

        def _emergency_brake(self, *, support_confirmed=False):
            assert support_confirmed is True
            self.brake_calls += 1
            self.hardware_mode = "brake"

    def create():
        clock, window, override = [1.0], Window(), {}

        def sample():
            return dict({
                "source_monotonic_ns": int(clock[0] * 1e9),
                "identity": ("session", "state", "gravity", "anchor", "envelope"),
                "feedback_fresh": True, "healthy": True, "zero_ff_authority": True,
                "router_ready": True, "router_rejected_commands": 0,
                "stationary_hold_ready": True,
                "modes": dict.fromkeys(MOTORS, window.hardware_mode),
                "j6_drive_state": 1 if window.hardware_mode == "hold" else 0,
                "j6_raw_sequence": int(clock[0] * 100),
                "router_hold_fresh": window.hardware_mode == "hold",
                "actual_rad": list(window.actual),
                "velocity_rad_s": [0.0] * 6,
            }, **override)

        probe = HoldProbe(window, sample, now=lambda: clock[0])

        def tick(seconds=0.1):
            clock[0] = round(clock[0] + seconds, 6)
            probe.tick()

        return probe, window, override, tick

    probe, window, override, tick = create()
    tick()
    frozen = list(window.command_targets)
    window.actual[4] = 0.002
    for _ in range(110):
        tick()
    assert probe.result()["status"] == "PASS"
    assert probe.result()["hold_source_coverage_s"] >= 10.0
    assert probe.result()["hold_distinct_sample_count"] >= 100
    assert window.hold_calls == window.brake_calls == 1
    assert window.command_targets == frozen
    assert any(sample.get("error_deg", [0] * 6)[4] != 0 for sample in probe.samples)

    for fault in ({"zero_ff_authority": False}, {"healthy": False},
                  {"stationary_hold_ready": False}, {"router_rejected_commands": 1},
                  {"identity": ("different",)}, {"j6_drive_state": 0}, {"j6_drive_state": True}):
        probe, window, override, tick = create()
        tick()
        tick()
        override.update(fault)
        tick()
        assert probe.stage == "terminal" and window.brake_calls == 1
        override.clear()
        for _ in range(4):
            tick()
        assert probe.result()["status"] == "FAIL" and probe.terminal_confirmed

    probe, window, override, tick = create()
    tick()
    tick()
    window.command_targets[0] = 1.0
    tick()
    assert probe.failure == "frozen HOLD target changed" and window.hold_calls == 1

    probe, window, override, tick = create()
    tick()
    tick()
    window.actual[0] = 0.01
    tick()
    assert probe.failure == "demo HOLD target error exceeded 0.25 degrees"

    probe, window, override, tick = create()
    override["healthy"] = False
    tick(20.0)
    assert probe.stage == "terminal" and window.hold_calls == 0
    override["j6_drive_state"] = None
    tick(3.0)
    assert probe.done and not probe.terminal_confirmed and probe.result()["status"] == "FAIL"

    probe, window, override, tick = create()
    tick()
    tick()
    tick(0.3)
    assert probe.failure == "HOLD feedback sample gap exceeded 250 ms"

    probe, window, override, tick = create()
    tick()
    override["stationary_hold_ready"] = False
    tick()
    assert probe.stage == "readiness" and probe.hold_since is None
    override.clear()
    tick()
    assert probe.stage == "hold"
    probe.stop("operator stop")
    override["j6_raw_sequence"] = 42
    for _ in range(4):
        tick()
    assert not probe.terminal_confirmed
    override.clear()
    for _ in range(3):
        tick()
    assert probe.done and probe.terminal_confirmed and window.brake_calls == 1

    probe, window, override, tick = create()
    tick()
    tick()
    first_source = probe.first_hold_source_ns
    for _ in range(99):
        tick()
    override["source_monotonic_ns"] = first_source + 9_960_000_000
    tick()
    assert probe.stage == "hold" and not probe.hold_window_pass
    override["source_monotonic_ns"] = first_source + 10_010_000_000
    tick(0.05)
    assert probe.stage == "terminal" and probe.hold_window_pass

    imports_before = set(sys.modules)
    with redirect_stdout(StringIO()) as output:
        assert main([]) == 0
    assert "OFFLINE_DESCRIPTION_ONLY" in output.getvalue()
    assert not any(name.startswith(("rclpy", "PySide6")) for name in set(sys.modules) - imports_before)

    binding = SimpleNamespace(session_id="session", state_instance_id="state",
                              envelope_id="envelope", envelope_sha256="a" * 64,
                              anchor_sha256="b" * 64)
    gravity = {"session_id": "session", "state_instance_id": "state",
               "empirical_validation": {"envelope_id": "envelope",
                                        "envelope_sha256": "a" * 64,
                                        "anchor_sha256": "b" * 64}}
    assert empirical_binding_matches(gravity, binding)
    for field in ("envelope_id", "envelope_sha256", "anchor_sha256"):
        original = gravity["empirical_validation"][field]
        gravity["empirical_validation"][field] = "other"
        assert not empirical_binding_matches(gravity, binding)
        gravity["empirical_validation"][field] = original


if __name__ == "__main__":
    test_current_pose_hold_probe()
    print("GUI_HOLD_PROBE_OFFLINE=PASS")

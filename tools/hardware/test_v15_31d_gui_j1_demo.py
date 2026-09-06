"""Run without ROS or hardware: python tools/hardware/test_v15_31d_gui_j1_demo.py."""
from contextlib import redirect_stdout
from io import StringIO
import ast
import hashlib
import json
import math
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace

from v15_31d_gui_j1_demo import J1Demo, LEVELS, MOTORS, check_recipe, main
from v15_31b_acceptance_signal import SignalPayloadSequence

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
    "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws/src/go_m8010_arm_hardware"))
from go_m8010_arm_hardware.empirical_validation_envelope import validate_stage_confirmation
sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
    "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws/src/go_m8010_arm_gui"))
from go_m8010_arm_gui.action_groups import ActionGroup, ActionStep, PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG


def harness(cycles=1):
    clock, commands, state = [0.0], [], {"index": 0, "entered": 0.0, "confirmed": -1.0}
    override = {}
    binding = SimpleNamespace(envelope_id="envelope", envelope_sha256="a" * 64, sha256="a" * 64,
                              session_id="session", state_instance_id="state")
    sequence = SignalPayloadSequence(binding, source_instance_id="c" * 32,
                                     monotonic_ns=lambda: int(clock[0] * 1e9))
    window = SimpleNamespace(actual=[0.0] * 6, command_targets=[0.0] * 6,
                             session_pose_deg=[0.0] * 6, hardware_mode="brake", command_stream_suspended=True)
    window._tick = lambda: None
    def hold():
        commands.append("hold")
        window.command_targets = window.actual[:]
        window.hardware_mode, window.command_stream_suspended = "hold", False
    def brake(*, support_confirmed):
        assert support_confirmed is True
        commands.append("brake")
        window.hardware_mode = "brake"
    window._hold_current, window._emergency_brake = hold, brake
    def observe():
        complete = clock[0] - state["entered"] >= 0.5
        return {"source_monotonic_ns": int(clock[0] * 1e9), "healthy": True,
                "thermal_ready": True, "authority": True, "zero_ff_authority": state["index"] == 0,
                "identity": ("session", "state", "gravity", "anchor", "envelope"),
                "router_ready": True, "router_rejected_commands": 0,
                "router_hold_fresh": window.hardware_mode == "hold",
                "stationary_hold_ready": window.hardware_mode == "hold",
                "modes": dict.fromkeys(MOTORS, window.hardware_mode),
                "j6_drive_state": 0 if window.hardware_mode == "brake" else 1,
                "j6_raw_sequence": int(clock[0] * 1000), "feedback_fresh": True,
                "actual_rad": window.actual[:], "velocity_rad_s": [0.0] * 6,
                "stage_index": state["index"], "stage_level": LEVELS[state["index"]],
                "stage_complete": complete,
                "position_authorized": state["index"] == 4 and complete and state["confirmed"] >= state["entered"] + 0.5,
                **override}
    def confirm(level):
        # Use the production validator: a current-rung confirmation would fail
        # here even if the simulated status/RPC callbacks otherwise looked green.
        validate_stage_confirmation(sequence.confirmation(level), envelope=binding,
            target_scale=LEVELS[min(state["index"] + 1, 4)], now_monotonic_ns=int(clock[0] * 1e9))
        commands.append(("confirm", level))
        if level == 1.0:
            state["confirmed"] = clock[0]
    def scale(level):
        assert ("confirm", level) in commands
        commands.append(("scale", level))
        state["index"], state["entered"] = int(level * 4), clock[0]
        return SimpleNamespace(done=lambda: True,
                               result=lambda: SimpleNamespace(results=[SimpleNamespace(successful=True)]))
    def start_group(origin):
        assert observe()["position_authorized"]
        commands.append("action_group")
        return SimpleNamespace(runner=SimpleNamespace(state="moving", detail="waiting", events=[]))
    demo = J1Demo(window, observe, confirm, scale, start_group, cycles=cycles, now=lambda: clock[0])
    def tick(seconds=0.1):
        clock[0] = round(clock[0] + seconds, 6)
        demo.tick()
    return demo, commands, override, tick


def test_bounded_j1_action_group():
    for displacement, final_drift in ((1.0, False), (0.02, False), (1.0, True)):
        demo, commands, _, tick = harness()
        for _ in range(120):
            tick()
            if demo.stage == "action_group":
                break
        assert [item[1] for item in commands if isinstance(item, tuple) and item[0] == "scale"] == list(LEVELS[1:])
        assert commands.count("hold") == commands.count("action_group") == 1
        demo.dialog.runner.events = [
            {"event": "measured_arrival", "index": 0, "actual_model_deg": [displacement] + [0.0] * 5},
            {"event": "measured_arrival", "index": 1, "actual_model_deg": [0.0] * 6},
        ]
        demo.dialog.runner.state = "complete"
        if final_drift:
            demo.window.actual[0] = math.radians(0.3)
        for _ in range(5):
            tick()
        assert demo.done and demo.terminal.terminal_confirmed
        assert demo.result()["status"] == ("PASS" if displacement == 1.0 and not final_drift else "FAIL")
        assert commands.count("brake") == 1

    for fault in ("authority", "healthy", "thermal_ready"):
        demo, commands, override, tick = harness()
        tick()
        tick()
        override[fault] = False
        for _ in range(5):
            tick()
        assert demo.result()["status"] == "FAIL" and "action_group" not in commands
        assert commands.count("brake") == 1

    demo, commands, override, tick = harness()
    tick()
    override["j6_drive_state"] = None
    tick()
    assert demo.stage == "engaging" and not demo.done
    override.clear()
    tick()
    assert demo.stage == "ladder"
    demo.stop("end offline scenario")

    demo, commands, override, tick = harness()
    tick()
    tick()
    override["stage_complete"] = False
    tick(175.0)
    tick(2.0)
    tick(3.0)
    assert demo.done and demo.result()["status"] == "FAIL"
    assert "action_group" not in commands and commands.count("brake") == 1

    original = (0.0,) * 6
    recipe = SimpleNamespace(segments=[SimpleNamespace(start_rad=original,
        target_rad=(math.radians(1), math.radians(0.02), 0, 0, 0, 0), sha256="test")])
    assert check_recipe(recipe, original)[0]["moving_joints"] == ["J1", "J2"]
    recipe.segments[0].target_rad = (math.radians(1), math.radians(0.26), 0, 0, 0, 0)
    try:
        check_recipe(recipe, original)
    except RuntimeError:
        pass
    else:
        raise AssertionError("accepted an oversized secondary correction")
    with redirect_stdout(StringIO()) as output:
        assert main([]) == 0
    assert "OFFLINE_DESCRIPTION_ONLY" in output.getvalue()


def test_three_cycles_reuse_file_and_keep_completed_cycle_when_second_fails():
    # Execute the actual disk load/table-reset callback with a tiny UI stand-in.
    # This catches a changed file or accumulating rows without importing Qt/ROS.
    source = Path(__file__).with_name("v15_31d_gui_j1_demo.py")
    run_live = next(node for node in ast.parse(source.read_text(encoding="utf-8")).body
                    if isinstance(node, ast.FunctionDef) and node.name == "run_live")
    start = next(node for node in run_live.body if isinstance(node, ast.FunctionDef) and node.name == "start_group")
    for fault in (None, "authority", "file_changed"):
        with tempfile.TemporaryDirectory() as directory:
            demo, commands, override, tick = harness(cycles=3)
            rows, row_counts = [], []
            dialog = SimpleNamespace(name=SimpleNamespace(setText=lambda _: None),
                table=SimpleNamespace(setRowCount=lambda count: rows.clear()), append_step=rows.append)
            def start_dialog():
                commands.append("action_group")
                row_counts.append(len(rows))
                dialog.log_path = Path(directory) / f"cycle-{len(row_counts)}.jsonl"
                dialog.runner = SimpleNamespace(state="moving", detail="waiting", events=[])
            dialog.start = start_dialog
            demo.window.action_group_dialog = dialog
            demo.window.workflow_contract = SimpleNamespace(model_sha256="a" * 64)
            demo.window.absolute_limits = PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG
            namespace = dict(window=demo.window, node=SimpleNamespace(log_directory=Path(directory)),
                             demo=demo, ActionGroup=ActionGroup, ActionStep=ActionStep, math=math, hashlib=hashlib)
            exec(compile(ast.Module(body=[start], type_ignores=[]), str(source), "exec"), namespace)
            demo.start_group = namespace["start_group"]
            for _ in range(120):
                tick()
                if demo.stage == "action_group":
                    break
            original = demo.origin
            path = Path(demo.action_group_file["path"])
            original_bytes = path.read_bytes()
            for cycle in range(1, 4):
                if fault == "authority" and cycle == 2:
                    override["authority"] = False
                    tick()
                    override.clear()
                    break
                assert len(rows) == 2 and rows[0].target_deg[0] == 1 and rows[1].target_deg[0] == 0
                assert path.read_bytes() == original_bytes
                demo.window.actual[0] = math.radians(0.08)
                dialog.runner.events = [
                    {"event": "measured_arrival", "index": 0, "actual_model_deg": [1.1] + [0.0] * 5},
                    {"event": "measured_arrival", "index": 1, "actual_model_deg": [0.08] + [0.0] * 5},
                ]
                dialog.runner.state = "complete"
                if fault == "file_changed":
                    path.write_bytes(original_bytes + b"\n")
                tick()
                if fault == "file_changed":
                    break
            for _ in range(4):
                tick()
            result = demo.result()
            assert demo.origin == original and commands.count("hold") == commands.count("brake") == 1
            assert result["requested_cycles"] == 3 and result["maximum_seconds"] == 270
            assert result["completed_cycles"] == (3 if fault is None else 1)
            assert [item["status"] for item in result["cycle_results"]] == (["PASS"] * 3 if fault is None else ["PASS", "FAIL"])
            assert result["status"] == ("PASS" if fault is None else "FAIL")
            assert result["cycle_results"][0]["out_deg"] == 1.1
            assert len(result["cycle_results"][0]["action_group_events"]) == 2
            assert row_counts == ([2, 2, 2] if fault is None else [2, 2] if fault == "authority" else [2])
            assert result["action_group_file"]["sha256"] == hashlib.sha256(original_bytes).hexdigest()
    with redirect_stdout(StringIO()) as output:
        assert main(["--cycles", "3"]) == 0
    assert json.loads(output.getvalue())["maximum_seconds"] == 270


if __name__ == "__main__":
    test_bounded_j1_action_group()
    test_three_cycles_reuse_file_and_keep_completed_cycle_when_second_fails()
    print("GUI_J1_DEMO_OFFLINE=PASS")

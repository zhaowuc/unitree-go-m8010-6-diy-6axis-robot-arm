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

from v15_31d_gui_j1_demo import J1Demo, LEVELS, MOTORS, check_recipe, main, maximum_demo_seconds
from v15_31b_acceptance_signal import SignalPayloadSequence

sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
    "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws/src/go_m8010_arm_hardware"))
from go_m8010_arm_hardware.empirical_validation_envelope import validate_stage_confirmation
sys.path.insert(0, str(Path(__file__).resolve().parents[2] /
    "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws/src/go_m8010_arm_gui"))
from go_m8010_arm_gui.action_groups import ActionGroup, ActionStep, PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG


def harness(cycles=1, demo_type=J1Demo, **options):
    clock, commands, state = [0.0], [], {"index": 0, "entered": 0.0, "confirmed": -1.0}
    override = {}
    binding = SimpleNamespace(envelope_id="envelope", envelope_sha256="a" * 64, sha256="a" * 64,
                              session_id="session", state_instance_id="state")
    sequence = SignalPayloadSequence(binding, source_instance_id="c" * 32,
                                     monotonic_ns=lambda: int(clock[0] * 1e9))
    window = SimpleNamespace(node=SimpleNamespace(), actual=[0.0] * 6, command_targets=[0.0] * 6,
                             session_pose_deg=[0.0] * 6, activation_epoch=0,
                             hardware_mode="brake", command_stream_suspended=True)
    window._tick = lambda: None
    window.teach_toolbar = SimpleNamespace(actions=lambda: [], addWidget=lambda *_: None, setEnabled=lambda *_: None)
    def hold():
        commands.append("hold")
        window.activation_epoch += 1
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
                "source_age_ms": 0.0,
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
    demo = demo_type(window, observe, confirm, scale, start_group, cycles=cycles, now=lambda: clock[0], **options)
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
    tick()
    override["j6_drive_state"] = None
    tick()
    assert demo.stage == "engaging" and not demo.done
    override.clear()
    tick()
    assert demo.stage == "engaging"
    tick(1.01)
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


def test_engaging_waits_for_one_continuous_second_after_startup_transient():
    demo, commands, override, tick = harness()
    tick()
    tick()
    override["velocity_rad_s"] = [math.radians(0.22)] + [0.0] * 5
    tick()
    assert demo.stage == "engaging"
    demo.window.actual[0] = math.radians(0.1007)
    override.update(stationary_hold_ready=False,
                    velocity_rad_s=[math.radians(0.2559)] + [0.0] * 5)
    tick(0.3)
    assert demo.stage == "engaging" and demo.engaging_ready_since is None
    assert demo.failure is None and commands.count("hold") == 1
    override.update(stationary_hold_ready=True, velocity_rad_s=[0.0] * 6)
    tick()
    for _ in range(9):
        tick()
    assert demo.stage == "engaging"  # Earlier brief readiness did not count.
    tick(0.11)
    assert demo.stage == "ladder" and demo.failure is None
    assert demo.origin == (0.0,) * 6 and commands.count("hold") == 1
    demo.stop("end offline startup replay")


def test_initial_hold_waits_for_advancing_fresh_paired_brake_but_active_fault_stops():
    demo, commands, override, tick = harness()
    for age in (193.0, 233.0, 263.0):
        override.update(source_age_ms=age, j6_drive_state=None)
        tick()
    assert "hold" not in commands
    override.clear()
    override["j6_drive_state"] = None
    tick()
    assert "hold" not in commands
    override.update(j6_drive_state=0, source_monotonic_ns=420_000_000, j6_raw_sequence=420)
    tick(0.02)
    override["source_age_ms"] = 20.0
    tick(0.02)
    assert "hold" not in commands
    override.update(source_monotonic_ns=440_000_000, j6_raw_sequence=440)
    tick(0.02)
    assert commands.count("hold") == 1
    override.clear()
    override["healthy"] = False
    tick()
    assert demo.stage == "terminal" and commands.count("brake") == 1


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


def test_ten_symmetric_cycles_keep_midpoint_file_and_return_center():
    source = Path(__file__).with_name("v15_31d_gui_j1_demo.py")
    run_live = next(node for node in ast.parse(source.read_text(encoding="utf-8")).body
                    if isinstance(node, ast.FunctionDef) and node.name == "run_live")
    callbacks = [node for node in run_live.body if isinstance(node, ast.FunctionDef)
                 and node.name in {"start_group", "start_center"}]
    for fault in (None, "negative_endpoint", "center"):
        with tempfile.TemporaryDirectory() as directory:
            demo, commands, _, tick = harness(cycles=10, excursion_deg=10, symmetric=True,
                                              speed_deg_s=3, return_center=True)
            original_deg = [0.37, 0.02, -0.03, 0.04, -0.05, 0.06]
            demo.window.actual = [math.radians(value) for value in original_deg]
            rows, endpoints = [], []
            dialog = SimpleNamespace(name=SimpleNamespace(setText=lambda _: None),
                table=SimpleNamespace(setRowCount=lambda _: rows.clear()), append_step=rows.append)
            def start_dialog():
                endpoints.extend([step.target_deg[0] - original_deg[0] for step in rows])
                dialog.log_path = Path(directory) / f"move-{len(endpoints)}.jsonl"
                dialog.runner = SimpleNamespace(state="moving", detail="waiting", events=[])
            dialog.start = start_dialog
            demo.window.action_group_dialog = dialog
            demo.window.workflow_contract = SimpleNamespace(model_sha256="a" * 64)
            demo.window.absolute_limits = PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG
            namespace = dict(window=demo.window, node=SimpleNamespace(log_directory=Path(directory)),
                demo=demo, ActionGroup=ActionGroup, ActionStep=ActionStep, math=math, hashlib=hashlib)
            exec(compile(ast.Module(body=callbacks, type_ignores=[]), str(source), "exec"), namespace)
            demo.start_group, demo.start_center = namespace["start_group"], namespace["start_center"]
            for _ in range(120):
                tick()
                if demo.stage == "action_group":
                    break
            origin = demo.origin
            path = Path(demo.action_group_file["path"])
            original_bytes = path.read_bytes()
            for cycle in range(10):
                assert len(rows) == 2 and all(step.speed_deg_s == 3 for step in rows)
                assert all(step.target_deg[1:] == tuple(original_deg[1:]) for step in rows)
                assert path.read_bytes() == original_bytes and demo.origin == origin
                positive, negative = original_deg[:], original_deg[:]
                positive[0] += 10.1
                negative[0] -= 9.74 if fault == "negative_endpoint" and cycle == 1 else 9.9
                demo.window.actual[0] = math.radians(original_deg[0] - 9.9)
                dialog.runner.events = [
                    {"event": "measured_arrival", "index": index, "actual_model_deg": endpoint}
                    for index, endpoint in enumerate((positive, negative))]
                dialog.runner.state = "complete"
                tick()
                if demo.stage == "terminal":
                    break
            if fault != "negative_endpoint":
                assert demo.stage == "return_center" and len(rows) == 1
                assert all(abs(a-b) < 1e-12 for a,b in zip(rows[0].target_deg, original_deg))
                assert demo.result()["completed_cycles"] == 10 and not demo.success
                centered = original_deg[:]
                centered[0] += 0.26 if fault == "center" else 0.1
                demo.window.actual = [math.radians(value) for value in centered]
                dialog.runner.events = [{"event": "measured_arrival", "index": 0, "actual_model_deg": centered}]
                dialog.runner.state = "complete"
                tick()
            for _ in range(4):
                tick()
            result = demo.result()
            assert demo.origin == origin and commands.count("hold") == commands.count("brake") == 1
            assert result["status"] == ("PASS" if fault is None else "FAIL")
            assert result["maximum_seconds"] == 1500 and path.read_bytes() == original_bytes
            if fault != "negative_endpoint":
                assert all(abs(a-b) < 1e-12 for a,b in zip(endpoints, [10, -10] * 10 + [0]))
                assert len(endpoints) == 21 and len(result["cycle_results"]) == 10
                assert result["center_return"]["status"] == ("PASS" if fault is None else "FAIL")
            else:
                assert result["completed_cycles"] == 1 and result["center_return"] is None
    with redirect_stdout(StringIO()) as output:
        assert main(["--cycles", "10", "--excursion-deg", "10", "--symmetric", "--speed-deg-s", "3", "--return-center"]) == 0
    assert json.loads(output.getvalue())["maximum_seconds"] == 1500
    assert [maximum_demo_seconds(cycles) for cycles in (1, 2, 3)] == [180, 225, 270]
    for cycles, excursion in ((11, 1), (1, 10.01), (1, float("nan"))):
        try:
            maximum_demo_seconds(cycles, excursion)
        except ValueError:
            pass
        else:
            raise AssertionError("accepted invalid cycles/excursion")


def test_initial_recovery_uses_original_file_and_checks_all_axes_before_hold():
    from v15_31f_gui_hand_guidance import GuidanceDemo
    from test_v15_31f_gui_hand_guidance import Widget
    from go_m8010_arm_gui.workflow_contract import generate_segmented_quintic_recipe
    source = Path(__file__).with_name("v15_31d_gui_j1_demo.py")
    run_live = next(node for node in ast.parse(source.read_text(encoding="utf-8")).body
                    if isinstance(node, ast.FunctionDef) and node.name == "run_live")
    start = next(node for node in run_live.body if isinstance(node, ast.FunctionDef) and node.name == "start_recovery")
    read = next(node for node in run_live.body if isinstance(node, ast.FunctionDef) and node.name == "read_initial_reference")
    gui_source = Path(__file__).resolve().parents[2] / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws/src/go_m8010_arm_gui/go_m8010_arm_gui/main_window.py"
    helpers = [node for node in ast.parse(gui_source.read_text(encoding="utf-8")).body
               if isinstance(node, ast.FunctionDef) and node.name in {"optional_sha256_valid", "initial_pose_binding_valid"}]
    validation = {"hashlib": hashlib}
    exec(compile(ast.Module(body=helpers, type_ignores=[]), str(gui_source), "exec"), validation)
    for fault in (None, "health", "j6_too_far", "interactive_guidance"):
        with tempfile.TemporaryDirectory() as directory:
            interactive = fault == "interactive_guidance"
            options = {"interactive_teach": True, "demo_type": GuidanceDemo,
                       "gui": SimpleNamespace(QPushButton=Widget, QLabel=Widget)} if interactive else {}
            demo, commands, override, tick = harness(**options)
            enabled = []
            demo.window.centralWidget = lambda: SimpleNamespace(setEnabled=enabled.append)
            override["assisted_teach_authorized"] = True
            demo.recover_initial_first = True
            demo.window.actual = [math.radians(value) for value in (1.629, -0.06, -0.055, 2.067, -1.554, 0.64)]
            if fault == "j6_too_far":
                demo.window.actual[5] = math.radians(5.01)
            path = Path(directory) / "initial_pose.json"
            initial = {"有效": True, "会话标识": "persistent:" + "e" * 16,
                       "关节位置_弧度": {f"J{i}": 0.0 for i in range(1, 7)}}
            data = json.dumps(initial).encode()
            path.write_bytes(data)
            hardware = {"reference": "PERSISTENT_SOFTWARE_ZERO_V1", "persistent_zero_sha256": "e" * 64,
                        "initial_pose_sha256": hashlib.sha256(data).hexdigest()}
            rows = []
            dialog = SimpleNamespace(name=SimpleNamespace(setText=lambda _: None),
                table=SimpleNamespace(setRowCount=lambda _: rows.clear()), append_step=rows.append,
                log_path=Path(directory) / "recovery.jsonl")
            def start_dialog():
                commands.append("recover_initial")
                dialog.runner = SimpleNamespace(state="moving", detail="recovering", events=[], index=0)
                demo.events.append({"event": "checked_recovery_recipe_before_submit", "plan_token_id": "b" * 64,
                                    "segments": [f"J{i}" for i in range(1, 7)]})
            dialog.start = start_dialog
            demo.window.action_group_dialog = dialog
            demo.window.workflow_contract = SimpleNamespace(model_sha256="a" * 64)
            namespace = dict(window=demo.window, node=SimpleNamespace(initial_pose_path=path, latest_hardware=hardware),
                demo=demo, ActionGroup=ActionGroup, ActionStep=ActionStep, math=math, hashlib=hashlib, json=json,
                gui=SimpleNamespace(initial_pose_binding_valid=validation["initial_pose_binding_valid"]))
            if interactive:
                class BaseDialog(SimpleNamespace):
                    def start(self):
                        start_dialog()
                        demo.window.workflow_contract.q_plan_trajectory = generate_segmented_quintic_recipe(
                            demo.origin, (0.0,) * 6, ((-math.pi, math.pi),) * 6,
                            maximum_velocity_rad_s=math.radians(1), maximum_acceleration_rad_s2=math.radians(15),
                            maximum_segment_delta_rad=math.radians(5))
                        self._move_status()
                    def _move_status(self):
                        return "running"
                dialog_node = next(item for item in run_live.body if isinstance(item, ast.ClassDef) and item.name == "DemoDialog")
                namespace.update(ActionGroupDialog=BaseDialog, hand_guidance=True, check_recipe=check_recipe)
                exec(compile(ast.Module(body=[dialog_node], type_ignores=[]), str(source), "exec"), namespace)
                dialog = namespace["DemoDialog"](**{key: value for key, value in vars(dialog).items() if key != "start"},
                                                  window=demo.window, submitted=False)
                demo.window.action_group_dialog = dialog
                demo.window._preview_approval_matches_candidate = lambda: True
                demo.window.workflow_contract.current_plan_token = SimpleNamespace(token_id="b" * 64)
            exec(compile(ast.Module(body=[read, start], type_ignores=[]), str(source), "exec"), namespace)
            demo.start_recovery = namespace["start_recovery"]
            def validate_start(sample):
                hardware["position_rad"] = list(sample["actual_rad"])
                hardware["source_monotonic_ns"] = sample["source_monotonic_ns"]
                namespace["read_initial_reference"](sample)
            demo.validate_recovery_start = validate_start
            for _ in range(120):
                tick()
                if demo.stage == "recover_initial" or demo.done:
                    break
            if fault == "j6_too_far":
                assert commands.count("hold") == 0 and "recover_initial" not in commands
                assert "J6" in demo.failure and demo.recovery_result["status"] == "FAIL"
                assert path.read_bytes() == data
                continue
            assert len(rows) == 1 and rows[0].target_deg == (0.0,) * 6
            assert demo.recovery_target == (0.0,) * 6 and "action_group" not in commands
            assert not demo.interactive_ready and not enabled
            if fault == "health":
                override["healthy"] = False
                tick()
                override.clear()
                for _ in range(4):
                    tick()
                assert demo.recovery_result["status"] == "FAIL" and "action_group" not in commands
            else:
                demo.window.actual = [math.radians(value) for value in (0.04, 0.02, -0.03, 0.12, -0.06, 0.04)]
                dialog.runner.events = [{"event": "measured_arrival", "index": 0,
                                         "actual_model_deg": [0.04, 0.02, -0.03, 0.12, -0.06, 0.04]}]
                dialog.runner.state = "complete"
                tick()
                assert demo.stage == "recovery_hold" and commands.count("hold") == 2
                assert demo.initial_hold_target[3] == math.radians(2.067)
                assert demo.origin[3] == math.radians(0.12)
                override["j6_drive_state"] = None
                tick()
                assert "action_group" not in commands
                override.clear()
                override["assisted_teach_authorized"] = True
                tick()
                assert demo.recovery_result["status"] == "PASS"
                if interactive:
                    assert demo.interactive_ready and demo.stage == "interactive_teach" and enabled == [True]
                    assert "action_group" not in commands and commands.count("recover_initial") == 1
                    assert demo.guidance_phase == "calibrating" and demo.bias is None and demo.core is None
                    assert demo.origin == tuple(demo.window.actual) and demo.origin != demo.initial_hold_target
                    assert demo.reference([0.0] * 6)["origin_rad"] == list(demo.origin)
                else:
                    assert commands.count("action_group") == 1
                    demo.stop("end offline recovery scenario")
            assert path.read_bytes() == data
    from go_m8010_arm_gui.workflow_contract import generate_segmented_quintic_recipe
    origin = tuple(math.radians(value) for value in (1.629, -0.06, -0.055, 2.067, -1.554, 0.64))
    recipe = generate_segmented_quintic_recipe(origin, (0,) * 6, ((-math.pi, math.pi),) * 6,
        maximum_velocity_rad_s=math.radians(1), maximum_acceleration_rad_s2=math.radians(15),
        maximum_segment_delta_rad=math.radians(5))
    assert [item["moving_joints"] for item in check_recipe(recipe, origin, recover_initial=True)] == [[f"J{i}"] for i in range(1, 7)]
    recipe = SimpleNamespace(segments=[SimpleNamespace(start_rad=(0,) * 6,
        target_rad=(math.radians(5.01), 0, 0, 0, 0, 0), profile=SimpleNamespace(duration_s=10), sha256="c" * 64)])
    try:
        check_recipe(recipe, origin, recover_initial=True)
    except RuntimeError:
        pass
    else:
        raise AssertionError("recovery allowed an oversized J1 correction")


if __name__ == "__main__":
    test_bounded_j1_action_group()
    test_engaging_waits_for_one_continuous_second_after_startup_transient()
    test_initial_hold_waits_for_advancing_fresh_paired_brake_but_active_fault_stops()
    test_three_cycles_reuse_file_and_keep_completed_cycle_when_second_fails()
    test_ten_symmetric_cycles_keep_midpoint_file_and_return_center()
    test_initial_recovery_uses_original_file_and_checks_all_axes_before_hold()
    print("GUI_J1_DEMO_OFFLINE=PASS")


def test_interactive_teach_bootstrap_never_runs_motion_or_refreshes_hands_off_during_contact():
    for authorized in (False, True):
        demo, commands, override, tick = harness(interactive_teach=True)
        enabled = []
        demo.window.centralWidget = lambda: SimpleNamespace(setEnabled=enabled.append)
        override["assisted_teach_authorized"] = authorized
        for _ in range(150):
            tick()
            if demo.interactive_ready or demo.done:
                break
        assert "action_group" not in commands
        assert demo.interactive_ready is authorized
        if authorized:
            assert enabled == [True] and demo.maximum_seconds == 600
            confirmations = [c for c in commands if isinstance(c, tuple) and c[0] == "confirm"]
            demo.window.hardware_mode = "teach"
            demo.window.actual[0] = math.radians(2)
            for _ in range(10):
                tick()
            assert confirmations == [c for c in commands if isinstance(c, tuple) and c[0] == "confirm"]
            assert not demo.done and demo.stage == "interactive_teach"
            demo.stop()
            for _ in range(5):
                tick()
            assert demo.result()["status"] == "PASS"
            assert demo.result()["scope"].startswith("BOOTSTRAP_AND_TERMINAL")
        else:
            assert demo.result()["status"] == "FAIL" and not enabled


def test_requested_teach_observation_is_one_bounded_activation_without_position_moves():
    demo, commands, override, tick = harness(interactive_teach=True, teach_observe=True)
    demo.window.centralWidget = lambda: SimpleNamespace(setEnabled=lambda _: None)
    override["assisted_teach_authorized"] = True
    demo.window.teach_joint = None
    demo.window.teach_joint_selector = SimpleNamespace(setCurrentIndex=lambda i: commands.append(("select", i)))
    demo.window.teach_button = SimpleNamespace(setDown=lambda value: commands.append(("down", value)))
    def start():
        commands.append("teach_start")
        demo.window.teach_joint, demo.window.hardware_mode = 0, "teach"
    def release(reason):
        commands.append("teach_release")
        demo.window.teach_joint, demo.window.hardware_mode = None, "hold"
    demo.window._start_assisted_teach, demo.window._release_assisted_teach = start, release
    for _ in range(150):
        tick()
        if demo.interactive_ready:
            break
    tick()
    assert commands.count("teach_start") == 1 and demo.window.hardware_mode == "teach"
    tick(19.9)
    assert "teach_release" not in commands
    tick(0.2)
    assert commands.count("teach_release") == 1 and demo.window.hardware_mode == "hold"
    tick(10)
    assert commands.count("teach_start") == 1 and demo.teach_observation_finished
    assert "action_group" not in commands and demo.window.command_targets == [0.0] * 6

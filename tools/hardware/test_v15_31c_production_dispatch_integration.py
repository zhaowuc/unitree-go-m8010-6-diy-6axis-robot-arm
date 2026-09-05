"""Production dispatch contract, with real model proofs and no transport.

Ubuntu arm-gui venv after sourcing ROS Humble:
  ROS_DOMAIN_ID=177 QT_QPA_PLATFORM=offscreen MUJOCO_GL=egl python -m pytest \
    tools/hardware/test_v15_31c_production_dispatch_integration.py -q

No ROS node, publisher, socket, worker or serial device is constructed. Only
clock/encoder/temperature feedback, completed ladder prerequisites and message
transport are simulated. Planning, collision, load/thermal proof, serialization,
router gates and acceptance progression use the production implementations.
"""

from __future__ import annotations

from dataclasses import replace
from datetime import datetime, timedelta, timezone
import json
import math
import os
from pathlib import Path
import socket
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest
import yaml

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ.setdefault("MUJOCO_GL", "egl")
import rclpy  # Required: a missing production dependency must fail this gate.
import mujoco

ROOT = Path(__file__).resolve().parents[2]
WORKSPACE = next(ROOT.glob("V15_14*/ros2_ws"))
for package in ("go_m8010_arm_gui", "go_m8010_arm_hardware"):
    sys.path.insert(0, str(WORKSPACE / "src" / package))
sys.path.insert(0, str(ROOT / "tools/hardware/j6_dm_g6220"))

from go_m8010_arm_gui import command_router as router
from go_m8010_arm_gui import main_window as gui
from go_m8010_arm_gui.workflow_contract import (
    WorkflowState, trajectory_command_descriptor, trajectory_plan_manifest,
    trajectory_sample_index_at,
)
from go_m8010_arm_hardware import gravity_model as gm
from go_m8010_arm_hardware import mujoco_mirror_node as mirror
from go_m8010_arm_hardware import planned_path_feasibility as path
from go_m8010_arm_hardware.empirical_validation_envelope import (
    SOFTWARE_GRAVITY_ROTOR_LIMIT_NM,
)

import test_v15_31b_active_acceptance_runner as feedback
import v15_30a_gui_j6_controller as j6


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def test_real_model_proofs_accept_all_axis_out_and_back_with_encoder_noise(monkeypatch):
    def reject_transport(*args, **kwargs):
        raise AssertionError("production dispatch integration must never open a socket")

    monkeypatch.setattr(socket, "socket", reject_transport)
    packet_binary = os.environ.get("M8010_PACKET_AUDIT_BINARY", "")
    assert Path(packet_binary).is_file(), "set M8010_PACKET_AUDIT_BINARY to the compiled offline production packet probe"
    clock = [time.monotonic_ns()]
    monkeypatch.setattr(time, "monotonic_ns", lambda: clock[0])
    bound = replace(
        feedback.binding(),
        envelope_id="v15-31b-empirical-" + "1" * 20,
        state_instance_id="2" * 32,
        expires_at_utc=datetime.now(timezone.utc) + timedelta(minutes=60),
    )
    run = feedback.runner_mod.ActiveAcceptanceRunner(bound)
    monkeypatch.setattr(j6, "EXPECTED_GRAVITY_AUTHORITY_BINDING",
        j6.ExpectedGravityAuthorityBinding("EMPIRICAL_VALIDATION_ENVELOPE",
            bound.envelope_id, bound.envelope_sha256, bound.anchor_sha256,
            bound.session_id, bound.state_instance_id))
    j6_replay = j6.make_command_source_replay_state()
    model_path = WORKSPACE.parent / "mujoco_v15_14/go_m8010_arm_v15_14_kinematic.xml"
    pose_deg = [0.0, 90.0, -14.40, 13.49, 47.94, 0.0]
    pose = [math.radians(value) for value in pose_deg]
    anchor = gm.GravityModelAnchorV2.from_mapping({
        "schema": gm.GRAVITY_ANCHOR_SCHEMA,
        "model_sha256": gm.PRODUCTION_MODEL_SHA256,
        "session_id": bound.session_id,
        "state_instance_id": bound.state_instance_id,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "gear_ratio": gm.GO_GEAR_RATIO,
        "motor_direction_sign": dict(gm.FROZEN_MOTOR_SIGNS),
        "motor_raw_reference_rad": dict.fromkeys(gm.MOTOR_NAMES, 0.0),
        "motor_encoder_branch": dict.fromkeys(gm.MOTOR_NAMES, 0),
        "logical_joint_reference_rad": [0.0] * 6,
        "model_absolute_joint_rad": pose,
    })
    evaluator = gm.StaticGravityEvaluator(model_path)
    model = mujoco.MjModel.from_xml_path(str(model_path))
    model.opt.disableflags |= int(mujoco.mjtDisableBit.mjDSBL_FILTERPARENT)
    guard, guard_sha, contract_sha = mirror.load_verified_kinematic_guard(
        model_path, model, mujoco
    )
    engine = mirror.CollisionGuardEngine(
        guard, pose,
        absolute_joint_limits_deg=mirror.PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG,
        model_sha256=gm.PRODUCTION_MODEL_SHA256,
        guard_sha256=guard_sha, contract_sha256=contract_sha,
    )
    config = yaml.safe_load((WORKSPACE / "src/go_m8010_arm_gui/config/arm_gui.yaml").read_text(encoding="utf-8"))
    limits = tuple(zip(router.MODEL_COMMAND_LOWER_RAD, router.MODEL_COMMAND_UPPER_RAD))
    gravity_gate = router.GravityAuthorityGate()
    collision_gate = router.CollisionGuardProofGate()
    manifest_gate = router.PlanManifestGate()
    replay_gate = router.CommandReplayGuard()
    sequence = 0
    command_sequence = 0
    envelope = None
    node = SimpleNamespace(
        command_source_instance_id="3" * 32,
        command_publisher=RecordingPublisher(), target_publisher=RecordingPublisher(),
        control_streams_fresh=lambda *args: True,
        gravity_status_fresh=lambda *args: True,
        publish_collision_request=lambda request: None,
        thermal_limits=SimpleNamespace(thermal_stop_c=60.0),
    )

    def check_j6(normalized, segment=None):
        parsed = j6.parse_command(router.payload_for_domain(normalized, "J6"), clock[0])
        pending = j6.validate_empirical_command_authority(parsed, clock[0], j6_replay)
        j6.commit_empirical_command_authority(j6_replay, pending)
        expected_mode = "position" if normalized["moving_joint_mask"][5] else "hold"
        assert parsed["mode"] == expected_mode
        if expected_mode == "position":
            trajectory = parsed["trajectory"]
            for index in (0, trajectory["interval_count"] // 2, trajectory["interval_count"]):
                stamp = trajectory["execute_at_monotonic_ns"] + trajectory["duration_ns"] // trajectory["interval_count"] * index
                q, dq, actual_index, _ = j6.quintic_reference_at(
                    trajectory["start_rad"][5], trajectory["target_rad"][5],
                    trajectory["duration_ns"], trajectory["interval_count"],
                    trajectory["execute_at_monotonic_ns"], stamp)
                assert actual_index == index
                assert q == pytest.approx(segment.samples[index].q_rad[5], abs=1e-12)
                assert dq == pytest.approx(segment.samples[index].dq_rad_s[5], abs=1e-12)

    def check_go(normalized, domain, segment=None):
        packet_check = subprocess.run([packet_binary], input=json.dumps({
            "expected_binding": {
                "authority_class": "EMPIRICAL_VALIDATION_ENVELOPE",
                "empirical_envelope_id": bound.envelope_id,
                "empirical_envelope_sha256": bound.envelope_sha256,
                "anchor_sha256": bound.anchor_sha256,
                "session_id": bound.session_id,
                "state_instance_id": bound.state_instance_id,
            },
            "received_ns": clock[0],
            "packet": router.payload_for_domain(normalized, domain).decode("utf-8"),
        }), text=True, capture_output=True, timeout=10)
        assert packet_check.returncode == 0, packet_check.stderr
        consumed = json.loads(packet_check.stdout)
        assert consumed["serial_opened"] is False
        assert consumed["mode"] == ("hold" if segment is None else "position")
        if segment is not None:
            assert consumed["trajectory_sha256"] == segment.sha256
        for sampled in consumed.get("samples", []):
            reference = segment.samples[sampled["index"]]
            assert sampled["q_rad"] == pytest.approx(reference.q_rad, abs=1e-12)
            assert sampled["dq_rad_s"] == pytest.approx(reference.dq_rad_s, abs=1e-12)

    def observe(positions, *, stage=4, position_authorized=True, moving_joint=None, command=None, segment=None):
        nonlocal sequence
        sequence += 1
        now = clock[0]
        modes = dict.fromkeys(gm.MOTOR_NAMES, "hold")
        moving_motors = feedback.runner_mod.MOTOR_BY_JOINT.get(moving_joint, ())
        for motor in moving_motors:
            modes[motor] = "position"
        state = feedback.state(bound, now, sequence, positions=positions, modes=modes)
        state.update({
            "joint_names": [f"joint{i}" for i in range(1, 7)],
            "reference": "PERSISTENT_SOFTWARE_ZERO_V1",
            "persistent_zero_sha256": "4" * 64, "initial_pose_sha256": "5" * 64,
            "controller_fault_by_motor": dict.fromkeys(gm.MOTOR_NAMES, False),
            "lease_safe_hold_by_motor": dict.fromkeys(gm.MOTOR_NAMES, False),
        })
        scale = stage * 0.25
        gravity = evaluator.evaluate(anchor.model_q_from_actual(positions))
        raw_ff = gm.gravity_joint_to_rotor_commands(gravity, gravity_scale=scale)
        logical_ff = gm.gravity_joint_to_logical_rotor_feedforward(gravity, gravity_scale=scale)
        for name, motor in state["per_motor"].items():
            motor.update({
                "reference_captured": True,
                "thermal_config_sha256": path.THERMAL_CONFIG_SHA256,
                "feedback_receipt_monotonic_ns": now,
                "last_valid_feedback_monotonic_ns": now,
                "gravity_feedforward_rotor_nm": raw_ff[name] if name != "J6" else None,
            })
        if command is not None and moving_joint is not None:
            descriptor = command["trajectory"]
            index = trajectory_sample_index_at(now,
                execute_at_monotonic_ns=descriptor["execute_at_monotonic_ns"],
                duration_ns=descriptor["duration_ns"], interval_count=descriptor["interval_count"])
            sample = segment.samples[index]
            state["velocity_rad_s"] = list(sample.dq_rad_s)
            for motor in moving_motors:
                state["per_motor"][motor].update({
                    "dq_joint_rad_s": sample.dq_rad_s[int(moving_joint[1:]) - 1],
                    "trajectory_plan_token_id": command["plan_token_id"],
                    "trajectory_sha256": descriptor["trajectory_sha256"],
                    "trajectory_state": "PREPARED" if now < descriptor["execute_at_monotonic_ns"] else "COMPLETE" if index == descriptor["interval_count"] else "RUNNING",
                    "trajectory_sample_index": index,
                    "trajectory_interval_count": descriptor["interval_count"],
                })
        temperature_ok, margin, continuous, blocker = path.temperature_observation(
            state, session_id=bound.session_id, state_instance_id=bound.state_instance_id,
            derating_start_c=40.0,
        )
        proof = path.build_planned_path_proof(envelope,
            source_instance_id="6" * 32, sequence=sequence, source_monotonic_ns=now,
            model_sha256=gm.PRODUCTION_MODEL_SHA256, gravity_config_sha256=gm.GRAVITY_CONFIG_SHA256,
            thermal_config_sha256=path.THERMAL_CONFIG_SHA256,
            continuous_config_authoritative=False, continuous_hardware_authoritative=continuous,
            continuous_rotor_limits_nm=None, short_peak_rotor_limits_nm=None,
            temperature_limits_authoritative=temperature_ok, minimum_thermal_margin_c=margin,
            temperature_blocker=blocker, empirical_validation_authoritative=stage == 4 and position_authorized,
            empirical_rotor_limits_nm=SOFTWARE_GRAVITY_ROTOR_LIMIT_NM,
            empirical_envelope_id=bound.envelope_id, empirical_envelope_sha256=bound.envelope_sha256)
        # Simulated empirical ladder/feedback metadata; the PASS proof above is
        # generated only by the real model, full sample evaluator and thermal check.
        status = feedback.gravity(bound, now, scale=scale, hardware_sequence=sequence, hardware_source_ns=now,
            empirical_phase=None if position_authorized else "GRAVITY_LADDER")
        status.update({
            "source": "whole_arm_gravity_node", "source_instance_id": "6" * 32,
            "sequence": sequence, "production_model_hash_match": True,
            "q_actual_sha256": gui.collision_target_sha256(positions), "last_update_age_s": 0.0,
            "gravity_joint_nm": list(gravity), "feedforward_nm": list(logical_ff),
            "pose_feasibility": "PASS" if all(abs(raw_ff[name]) <= SOFTWARE_GRAVITY_ROTOR_LIMIT_NM[name] for name in gm.MOTOR_NAMES) else "FAIL",
            "blocker": None, "hardware_enable_requested": True,
            "torque_authority_class": "EMPIRICAL_VALIDATION_ENVELOPE",
            "actuation_interface_present": True, "planned_trajectory_feasibility": proof,
        })
        status["empirical_validation"].update({
            "maximum_position_segment_seconds": 15.0, "maximum_abs_position_segment_deg": 5.0,
            "maximum_cumulative_position_trajectory_seconds": 600.0,
        })
        assert gravity_gate.observe_status(status, now_ns=now)
        node.latest_hardware, node.latest_gravity_status = state, status
        assert run.observe_confirmation(feedback.confirmation(bound, now, sequence, target=scale), now_ns=now)
        run.observe_gravity_status(status, now_ns=now)
        run.observe_hardware_state(state, now_ns=now)
        assert run.failure is None, run.failure
        return state, proof

    # The 100% HOLD remains unarmed until a separate same-stage POSITION
    # confirmation, exactly as the production coordinator/ladder publishes it.
    for stage, authorized in [(stage, False) for stage in range(5)] + [(4, True)]:
        clock[0] += 50_000_000
        observe([0.0] * 6, stage=stage, position_authorized=authorized)
        command_sequence += 1
        gui.ArmGuiNode.publish_command(node, command_sequence, "hold", [0.0] * 6, [0.0] * 6,
            [True] * 6, [False] * 6, 1, config)
        command = json.loads(node.command_publisher.messages[-1].data)
        normalized, _ = router.validate_command(json.dumps(command), now_ns=clock[0])
        replay_gate.check(normalized, now_ns=clock[0])
        collision_gate.authorize(normalized, now_ns=clock[0])
        manifest_gate.authorize(normalized, now_ns=clock[0])
        gravity_gate.authorize(normalized, now_ns=clock[0])
        replay_gate.commit(normalized, now_ns=clock[0])
        check_j6(normalized)
        run.observe_gui_command(command, now_ns=clock[0])
    # This integration starts after successful physical ladder/comparison;
    # it deliberately does not pretend simulated feedback proves either one.
    run.gravity_ladder_complete = True
    run.comparison_complete = {"WITHOUT_FF", "WITH_FF"}
    run.start_position(now_ns=clock[0])
    for _ in range(12):
        clock[0] += 50_000_000
        observe([0.0] * 6)
    assert run.expected_joint == "J1" and run.expected_phase == "PLUS_5"

    for joint, phase in ((joint, phase) for joint in feedback.runner_mod.POSITION_ORDER
                         for phase in feedback.runner_mod.POSITION_PHASES[1:]):
        assert run.expected_joint == joint and run.expected_phase == phase and run.awaiting_position_command
        joint_index = feedback.runner_mod.JOINT_NAMES.index(joint)
        moving_mask = [index == joint_index for index in range(6)]
        position = run.status(now_ns=clock[0])["position"]
        start = position["expected_start_vector_rad"]
        target = position["expected_target_vector_rad"]
        actual = [value + (index + 1) * 1e-5 for index, value in enumerate(start)]
        clock[0] += 50_000_000
        observe(actual)
        workflow = WorkflowState.initialize(actual, limits, gui.PRODUCTION_MODEL_SHA256,
            session_id=bound.session_id, state_instance_id=bound.state_instance_id,
            gravity_config_sha256=gm.GRAVITY_CONFIG_SHA256)
        built = gui.build_virtual_preview_plan({
            "generation": 1, "workflow": workflow, "actual_rad": actual,
            "start_rad": start, "target_rad": target, "limits_rad": limits,
            "maximum_velocity_rad_s": math.radians(config["控制"]["最大速度_度每秒"]),
            "maximum_acceleration_rad_s2": math.radians(config["控制"]["最大加速度_度每二次方秒"]),
            "maximum_segment_delta_rad": math.radians(gui.COLLISION_EXECUTE_SEGMENT_MAX_DEG),
            "maximum_sample_period_s": 0.01, "source_instance_id": node.command_source_instance_id,
            "request_sequence": sequence, "session_id": bound.session_id,
            "state_instance_id": bound.state_instance_id, "base_candidate_revision": workflow.candidate_revision,
        })
        recipe = built["trajectory"]
        assert len(recipe.segments) == 1
        segment = recipe.segments[0]
        assert recipe.sha256 != segment.sha256
        assert built["workflow"].q_actual == tuple(actual)
        request = path.parse_planned_path_request(json.loads(built["serialized_request"]), now_monotonic_ns=clock[0])
        envelope = path.evaluate_path_load_envelope(request, anchor=anchor, evaluator=evaluator,
            j6_joint_to_rotor_scale=None)
        clock[0] += 1_000_000
        _, load_proof = observe(actual)
        assert load_proof["result"] == "PASS", load_proof
        assert load_proof["trajectory_sha256"] == recipe.sha256
        window = SimpleNamespace(
            node=node, direction=gui.ArmMode.SIM_TO_REAL, session_id=bound.session_id,
            state_instance_id=bound.state_instance_id, session_pose_sha256=gui.session_pose_sha256(pose),
            command_stream_suspended=False, hardware_mode="hold", command_targets=list(start),
            targets=list(target), pending_target_joint_mask=moving_mask,
            requested_active_joint_mask=[True] * 6, edit_limits=[tuple(map(math.degrees, pair)) for pair in limits],
            active_trajectory_segment=segment, collision_request_sequence=sequence, collision_requests={},
            workflow_contract=built["workflow"], preview_animation_complete=True,
            preview_collision_segment_sha256=[segment.sha256],
        )
        collision_request = gui.MainWindow._new_collision_request(window, "execute")
        assert collision_request is not None
        parsed_collision = mirror.parse_collision_guard_request(collision_request, now_ns=clock[0],
            hardware_state=node.latest_hardware, expected_session_pose_sha256=window.session_pose_sha256)
        collision_proof = engine.evaluate(parsed_collision)
        assert collision_proof["safe"], collision_proof
        assert gui.collision_guard_result_matches(collision_proof, collision_request)
        window.preview_collision_safe = collision_proof["safe"]
        checks = gui.MainWindow._current_preview_checks(window, clock[0] / 1e9)
        assert checks.complete_success, checks
        workflow = built["workflow"].accept_successful_preview(recipe, checks,
            created_monotonic_ns=clock[0], nonce=f"{joint}:{phase}", actual_tolerance_rad=gui.PLAN_ACTUAL_DRIFT_TOLERANCE_RAD)
        token = workflow.current_plan_token.token_id
        workflow = workflow.explicit_real_submit(token, actual_tolerance_rad=gui.PLAN_ACTUAL_DRIFT_TOLERANCE_RAD)
        descriptor = trajectory_command_descriptor(segment, plan_token_id=token,
            execute_at_monotonic_ns=clock[0] + 250_000_000, segment_index=0, segment_count=1)
        command_sequence += 1
        gui.ArmGuiNode.publish_command(node, command_sequence, "position", list(target), list(target),
            [True] * 6, moving_mask, command_sequence, config,
            collision_proof, descriptor, trajectory_plan_manifest(recipe))
        command = json.loads(node.command_publisher.messages[-1].data)
        assert collision_gate.observe_result(collision_proof, now_ns=clock[0])
        normalized, _ = router.validate_command(json.dumps(command), now_ns=clock[0])
        replay_gate.check(normalized, now_ns=clock[0])
        collision_gate.authorize(normalized, now_ns=clock[0])
        manifest_gate.authorize(normalized, now_ns=clock[0])
        gravity_gate.authorize(normalized, now_ns=clock[0])
        replay_gate.commit(normalized, now_ns=clock[0])
        check_j6(normalized, segment)
        go_domain = "J1" if joint == "J6" else feedback.runner_mod.DOMAIN_BY_JOINT[joint]
        check_go(normalized, go_domain, None if joint == "J6" else segment)
        run.observe_gui_command(command, now_ns=clock[0])
        assert run.failure is None, run.failure
        assert run.active_segment is not None
        run.observe_router_status({"schema": feedback.runner_mod.ROUTER_STATUS_SCHEMA,
            "last_mode": normalized["mode"], "last_moving_joint_mask": normalized["moving_joint_mask"],
            "last_command_age_ms": 0.0}, now_ns=clock[0])
        for _ in range(100):
            clock[0] += 50_000_000
            descriptor = command["trajectory"]
            index = trajectory_sample_index_at(clock[0],
                execute_at_monotonic_ns=descriptor["execute_at_monotonic_ns"],
                duration_ns=descriptor["duration_ns"], interval_count=descriptor["interval_count"])
            command_sequence += 1
            run.observe_gui_command({**command, "sequence": command_sequence,
                "source_monotonic_ns": clock[0]}, now_ns=clock[0])
            observe(list(segment.samples[index].q_rad), moving_joint=joint, command=command, segment=segment)
            if run.active_segment is None:
                break
        assert run.active_segment is None, "runner did not acknowledge sampled trajectory and endpoint dwell"
        assert run.status(now_ns=clock[0])["position"]["last_completed_trajectory_sha256"] == segment.sha256
        clock[0] += 50_000_000
        # The real GUI's arrival transition retains the endpoint and epoch;
        # never turn an observed noisy encoder value into the new HOLD target.
        command_sequence += 1
        gui.ArmGuiNode.publish_command(node, command_sequence, "hold",
            list(command["targets_rad"]), list(target), [True] * 6, [False] * 6,
            command["activation_epoch"], config)
        hold = json.loads(node.command_publisher.messages[-1].data)
        assert hold["targets_rad"] == list(segment.target_rad)
        assert hold["activation_epoch"] == command["activation_epoch"]
        normalized, _ = router.validate_command(json.dumps(hold), now_ns=clock[0])
        replay_gate.check(normalized, now_ns=clock[0])
        collision_gate.authorize(normalized, now_ns=clock[0])
        manifest_gate.authorize(normalized, now_ns=clock[0])
        gravity_gate.authorize(normalized, now_ns=clock[0])
        replay_gate.commit(normalized, now_ns=clock[0])
        check_j6(normalized)
        check_go(normalized, go_domain)
        run.observe_gui_command(hold, now_ns=clock[0])
        observe(list(target))
        if phase == feedback.runner_mod.POSITION_PHASES[-1] and not run.position_complete:
            for _ in range(12):
                clock[0] += 50_000_000
                observe(list(target))
    assert run.failure is None
    assert run.position_complete
    assert sum(json.loads(message.data)["mode"] == "position"
               for message in node.command_publisher.messages) == len(feedback.runner_mod.POSITION_ORDER) * (len(feedback.runner_mod.POSITION_PHASES) - 1)

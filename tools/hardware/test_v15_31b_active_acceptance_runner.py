from __future__ import annotations

import importlib.util
import math
import subprocess
import sys
from datetime import datetime, timedelta, timezone
from pathlib import Path

import pytest


SCRIPT = Path(__file__).with_name("v15_31b_active_acceptance_runner.py")
SPEC = importlib.util.spec_from_file_location("v15_31b_active_acceptance_runner", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
runner_mod = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = runner_mod
SPEC.loader.exec_module(runner_mod)


def git(repo: Path, *arguments: str) -> str:
    completed = subprocess.run(
        ["git", *arguments],
        cwd=str(repo),
        stdin=subprocess.DEVNULL,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        check=True,
        text=True,
        encoding="utf-8",
    )
    return completed.stdout.strip()


def synced_repo_with_supporting_evidence(tmp_path: Path):
    repo = tmp_path / "repo"
    remote = tmp_path / "remote.git"
    repo.mkdir()
    git(repo, "init", "-b", runner_mod.EXPECTED_BRANCH)
    git(repo, "config", "user.name", "V15.31B Offline Test")
    git(repo, "config", "user.email", "v15-31b@example.invalid")
    (repo / "README.txt").write_text("fixture\n", encoding="utf-8")
    git(repo, "add", "README.txt")
    git(repo, "commit", "-m", "fixture")
    remote.mkdir()
    git(remote, "init", "--bare")
    git(repo, "remote", "add", "origin", str(remote))
    git(repo, "push", "-u", "origin", runner_mod.EXPECTED_BRANCH)
    output = repo / "evidence"
    output.mkdir()
    for filename in runner_mod.SUPPORTING_ARTIFACT_NAMES:
        runner_mod._atomic_json(output / filename, {"result": "PASS"})
    provenance = runner_mod.capture_run_git_provenance(repo, output)
    return repo, output, provenance


def binding():
    return runner_mod.EvidenceBinding(
        envelope_id="v15-31b-empirical-0123456789abcdef",
        envelope_sha256="a" * 64,
        session_id="persistent:session:j2session:abc:goauxsession:def",
        state_instance_id="state-instance-1",
        anchor_sha256="b" * 64,
        expires_at_utc=datetime.now(timezone.utc) + timedelta(hours=2),
        maximum_position_seconds=600.0,
        maximum_segment_seconds=15.0,
        maximum_segment_displacement_deg=5.0,
        maximum_stage_temperature_rise_c=2.0,
    )


def confirmation(bound, now_ns: int, sequence: int = 1, target: float = 1.0):
    return {
        "schema": runner_mod.CONFIRMATION_SCHEMA,
        "source_instance_id": "1" * 32,
        "sequence": sequence,
        "source_monotonic_ns": now_ns,
        "envelope_id": bound.envelope_id,
        "envelope_sha256": bound.envelope_sha256,
        "session_id": bound.session_id,
        "state_instance_id": bound.state_instance_id,
        "target_gravity_scale": target,
        "operator_stop_ready": True,
        "j2_j3_support_reliable": True,
        "clearance_confirmed": True,
        "no_person_contact": True,
    }


def gravity(
    bound,
    now_ns: int,
    *,
    scale: float = 1.0,
    target: float | None = None,
    requested: float | None = None,
    stage_index: int | None = None,
    stage_complete: bool = True,
    empirical_phase: str | None = None,
    trajectory_sha=None,
    hardware_sequence: int | None = None,
    hardware_source_ns: int | None = None,
):
    target = scale if target is None else target
    requested = target if requested is None else requested
    stage_index = (
        runner_mod.GRAVITY_LADDER_LEVELS.index(target)
        if stage_index is None else stage_index
    )
    proof = None
    if trajectory_sha is not None:
        proof = {
            "schema": "go-m8010-empirical-planned-load-thermal-feasibility/1.0",
            "result": "PASS",
            "load_feasibility": "PASS",
            "thermal_feasibility": "PASS",
            "trajectory_sha256": trajectory_sha,
            "session_id": bound.session_id,
            "state_instance_id": bound.state_instance_id,
            "model_sha256": runner_mod.PRODUCTION_MODEL_SHA256,
            "gravity_config_sha256": runner_mod.GRAVITY_CONFIG_SHA256,
            "thermal_config_sha256": runner_mod.THERMAL_CONFIG_SHA256,
            "empirical_validation_authoritative": True,
            "empirical_envelope_id": bound.envelope_id,
            "empirical_envelope_sha256": bound.envelope_sha256,
        }
    return {
        "schema": runner_mod.GRAVITY_STATUS_SCHEMA,
        "source_instance_id": "2" * 32,
        "sequence": max(1, now_ns // 1_000_000),
        "source_monotonic_ns": now_ns,
        "model_sha256": runner_mod.PRODUCTION_MODEL_SHA256,
        "gravity_config_sha256": runner_mod.GRAVITY_CONFIG_SHA256,
        "anchor_valid": True,
        "session_id": bound.session_id,
        "state_instance_id": bound.state_instance_id,
        "finite_bounded": True,
        "feedforward_nm": [0.0, 0.2 * scale, 0.0, 0.0, 0.0, 0.0],
        "gravity_scale": scale,
        "gravity_scale_target": target,
        "gravity_scale_requested": requested,
        "hardware_state_sequence": hardware_sequence,
        "hardware_state_source_monotonic_ns": hardware_source_ns,
        "continuous_rotor_limits_authoritative": False,
        "empirical_validation_authoritative": True,
        "hardware_tff_enabled": scale > 0.0,
        "empirical_validation": {
            "authority_class": "EMPIRICAL_VALIDATION_ENVELOPE",
            "rating_classification": "NOT_OFFICIAL_CONTINUOUS_RATING",
            "envelope_id": bound.envelope_id,
            "envelope_sha256": bound.envelope_sha256,
            "anchor_sha256": bound.anchor_sha256,
            "expires_at_utc": bound.expires_at_utc.isoformat().replace("+00:00", "Z"),
            "stage_index": stage_index,
            "stage_level": target,
            "stage_complete": stage_complete,
            "phase": (
                empirical_phase
                if empirical_phase is not None
                else "POSITION_VALIDATION" if target == 1.0 else "GRAVITY_LADDER"
            ),
            "position_validation_authorized": (
                target == 1.0
                and stage_complete
                and empirical_phase in {None, "POSITION_VALIDATION"}
            ),
            "invalidated": False,
            "continuous_operation_authorized": False,
            "official_continuous_rating_claimed": False,
        },
        "planned_trajectory_feasibility": proof,
    }


def state(
    bound,
    now_ns: int,
    sequence: int,
    *,
    positions=None,
    modes=None,
    trajectory_joint=None,
    trajectory_sha=None,
    plan_token=None,
    temperature_a=30.0,
    temperature_b=30.0,
    slope_a=0.0,
    slope_b=0.0,
    gravity_scale=1.0,
    j2_pd_abs=None,
    j2_feedback_abs=None,
    saturation=False,
    j6_raw_identity=None,
):
    positions = [0.0] * 6 if positions is None else list(positions)
    modes = {motor: "hold" for motor in runner_mod.MOTOR_NAMES} if modes is None else dict(modes)
    motor_joint_index = {
        "J1": 0, "J2A": 1, "J2B": 1, "J3": 2,
        "J4": 3, "J5": 4, "J6": 5,
    }
    per_motor = {}
    for motor in runner_mod.MOTOR_NAMES:
        ff = 0.0
        cmd = 0.1
        feedback = 0.2
        if motor == "J2A" and gravity_scale > 0.0:
            ff, cmd, feedback = -0.20, -0.30, -0.25
        elif motor == "J2B" and gravity_scale > 0.0:
            ff, cmd, feedback = 0.20, 0.30, 0.25
        if motor in {"J2A", "J2B"}:
            sign = -1.0 if motor == "J2A" else 1.0
            if j2_pd_abs is not None:
                cmd = ff + sign * float(j2_pd_abs)
            if j2_feedback_abs is not None:
                feedback = sign * float(j2_feedback_abs)
        if motor == "J6":
            ff = None
            cmd = None
            feedback = None
        item = {
            "communication_ok": True,
            "fresh": True,
            "merror": 0,
            "controller_metadata_status": "OBSERVED",
            "thermal_metadata_status": "OBSERVED",
            "thermal_state": "NORMAL",
            "thermal_fault_latched": False,
            "no_progress_metadata_status": "OBSERVED",
            "load_limit_no_progress": False,
            "temperature_c": (
                temperature_a if motor == "J2A"
                else temperature_b if motor == "J2B" else 30.0
            ),
            "q_joint_rad": positions[motor_joint_index[motor]],
            "dq_joint_rad_s": 0.0,
            "trajectory_plan_token_id": None,
            "trajectory_sha256": None,
            "trajectory_state": "INACTIVE",
            "tau_cmd_rotor_nm": cmd,
            "tau_feedback_rotor_nm": feedback,
            "gravity_feedforward_rotor_nm": ff,
            "feedback_source_monotonic_ns": now_ns,
        }
        if trajectory_joint and motor in runner_mod.MOTOR_BY_JOINT[trajectory_joint]:
            item.update({
                "trajectory_plan_token_id": plan_token,
                "trajectory_sha256": trajectory_sha,
                "trajectory_state": "ACTIVE",
            })
        per_motor[motor] = item
    return {
        "schema": runner_mod.HARDWARE_STATE_SCHEMA,
        "session_id": bound.session_id,
        "state_instance_id": bound.state_instance_id,
        "source_monotonic_ns": now_ns,
        "sequence": sequence,
        "healthy": True,
        "telemetry_healthy": True,
        "safety_metadata_ready": True,
        "j2_sync_fault": False,
        "position_rad": positions,
        "velocity_rad_s": [0.0] * 6,
        "j2_e_sync_rad": 0.0,
        "j6_raw_feedback_identity": j6_raw_identity,
        "per_motor": per_motor,
        "controller_mode_by_motor": modes,
        "controller_fault_observed_by_motor": {
            motor: False for motor in runner_mod.MOTOR_NAMES
        },
        "control_available_by_domain": {
            domain: True for domain in ("J1", "J2", "J345", "J6")
        },
        "worker_supervisor_instance_id": "supervisor-one",
        "worker_supervisor_pid": 4242,
        "thermal_status_by_motor": {
            motor: {
                "slope_c_per_min": (
                    slope_a if motor == "J2A"
                    else slope_b if motor == "J2B" else 0.0
                )
            }
            for motor in runner_mod.MOTOR_NAMES
        },
        "no_progress_status_by_motor": {
            motor: {"software_saturation_observed": (
                bool(saturation) if motor in {"J2A", "J2B"} else False
            )}
            for motor in runner_mod.MOTOR_NAMES
        },
    }


def position_command(bound, run, now_ns: int, counter: int, positions):
    joint = run.expected_joint
    phase = run.expected_phase
    assert joint is not None and phase is not None
    index = runner_mod.JOINT_NAMES.index(joint)
    target = list(run.center_rad)
    target[index] += math.radians(
        runner_mod.PHASE_OFFSET_DEG[run.position_phase_index]
    )
    trajectory_sha = f"{counter:064x}"
    plan_token = f"{counter + 10_000:064x}"
    moving = [False] * 6
    moving[index] = True
    proof = {
        "schema": "go-m8010-collision-guard-result/1.0",
        "safe": True,
        "session_id": bound.session_id,
        "state_instance_id": bound.state_instance_id,
        "moving_joint_mask": moving,
        "model_sha256": runner_mod.PRODUCTION_MODEL_SHA256,
    }
    command = {
        "schema": runner_mod.GUI_COMMAND_SCHEMA,
        "source_instance_id": "3" * 32,
        "source_monotonic_ns": now_ns,
        "sequence": counter,
        "mode": "position",
        "targets_rad": target,
        "active_joint_mask": [True] * 6,
        "moving_joint_mask": moving,
        "plan_token_id": plan_token,
        "trajectory": {
            "schema": "go-m8010-quintic-command/1.0",
            "trajectory_sha256": trajectory_sha,
            "profile": "quintic-rest-to-rest-v1",
            "start_rad": list(positions),
            "target_rad": target,
            "duration_ns": 2_000_000_000,
            "interval_count": 200,
            "execute_at_monotonic_ns": now_ns + 250_000_000,
            "segment_index": 0,
            "segment_count": 1,
        },
        "plan_manifest": {
            "schema": "go-m8010-plan-manifest/1.0",
            "recipe_sha256": f"{counter + 20_000:064x}",
            "segment_sha256": [trajectory_sha],
        },
        "collision_guard_proof": proof,
    }
    return command, target, trajectory_sha, plan_token


def post_command(
    bound, run, now_ns, counter, positions, moving_index, manifest_shas,
):
    post = run.post_execution
    assert post is not None
    target = list(post.commanded_rad)
    target[moving_index] = post.target_rad[moving_index]
    trajectory_sha = f"{counter + 30_000:064x}"
    plan_token = f"{counter + 40_000:064x}"
    recipe_sha = "9" * 64
    moving = [False] * 6
    moving[moving_index] = True
    proof = {
        "schema": "go-m8010-collision-guard-result/1.0",
        "safe": True,
        "session_id": bound.session_id,
        "state_instance_id": bound.state_instance_id,
        "moving_joint_mask": moving,
        "model_sha256": runner_mod.PRODUCTION_MODEL_SHA256,
    }
    return {
        "schema": runner_mod.GUI_COMMAND_SCHEMA,
        "source_instance_id": "4" * 32,
        "source_monotonic_ns": now_ns,
        "sequence": counter,
        "mode": "position",
        "targets_rad": target,
        "active_joint_mask": [True] * 6,
        "moving_joint_mask": moving,
        "plan_token_id": plan_token,
        "trajectory": {
            "schema": "go-m8010-quintic-command/1.0",
            "trajectory_sha256": trajectory_sha,
            "profile": "quintic-rest-to-rest-v1",
            "start_rad": list(positions),
            "target_rad": target,
            "duration_ns": 2_000_000_000,
            "interval_count": 200,
            "execute_at_monotonic_ns": now_ns + 250_000_000,
            "segment_index": moving_index,
            "segment_count": 6,
        },
        "plan_manifest": {
            "schema": "go-m8010-plan-manifest/1.0",
            "recipe_sha256": recipe_sha,
            "segment_sha256": list(manifest_shas),
        },
        "collision_guard_proof": proof,
    }, target, trajectory_sha, plan_token


def execute_post_position(run, kind, target, now, seq, positions):
    bound = run.binding
    run.start_post_position_execution(kind, target, now_ns=now)
    counter = {
        "MULTI_JOINT": 1,
        "THERMAL_SETUP": 101,
        "RESTORE_INITIAL": 201,
    }[kind]
    changed = [
        index for index, (start, end) in enumerate(zip(positions, target))
        if abs(start - end) > math.radians(0.01)
    ]
    manifest_shas = [
        f"{counter + offset + 30_000:064x}"
        for offset in range(len(changed))
    ]
    for index in changed:
        now += 1
        command, next_positions, sha, token = post_command(
            bound, run, now, counter, positions, index, manifest_shas
        )
        refresh_gravity(run, now, sha)
        run.observe_gui_command(command, now_ns=now)
        run.observe_router_status({
            "schema": runner_mod.ROUTER_STATUS_SCHEMA,
            "last_mode": "position",
            "last_moving_joint_mask": command["moving_joint_mask"],
            "last_command_age_ms": 1.0,
        }, now_ns=now)
        positions = list(next_positions)
        modes = {motor: "hold" for motor in runner_mod.MOTOR_NAMES}
        joint = runner_mod.JOINT_NAMES[index]
        for motor in runner_mod.MOTOR_BY_JOINT[joint]:
            modes[motor] = "position"
        for _ in range(6):
            now += 100_000_000
            seq += 1
            refresh_gravity(run, now, sha)
            run.observe_hardware_state(state(
                bound, now, seq, positions=positions, modes=modes,
                trajectory_joint=joint, trajectory_sha=sha, plan_token=token,
            ), now_ns=now)
            assert run.failure is None
        counter += 1
    for _ in range(6):
        now += 100_000_000
        seq += 1
        refresh_gravity(run, now)
        run.observe_hardware_state(
            state(bound, now, seq, positions=positions), now_ns=now
        )
        assert run.failure is None
    assert kind in run.completed_post_execution
    return now, seq, positions


def refresh_gravity(run, now_ns: int, trajectory_sha=None, scale=1.0):
    previous = run.gravity_worker_echo_source_last
    sequence = 1 if previous is None else previous[0] + 1
    source_ns = (
        max(1, now_ns - 100_000)
        if previous is None
        else max(previous[1] + 1, now_ns - 100_000)
    )
    assert source_ns <= now_ns
    value = gravity(
        run.binding,
        now_ns,
        scale=scale,
        trajectory_sha=trajectory_sha,
    )
    value["sequence"] = sequence
    value["source_monotonic_ns"] = source_ns
    run.observe_gravity_status(
        value,
        now_ns=now_ns,
    )


def test_default_is_offline_dry_run_and_never_claims_a_motion_publisher(capsys):
    assert runner_mod.main([]) == 0
    output = capsys.readouterr().out
    assert '"mode": "OFFLINE_DRY_RUN"' in output
    assert '"publishes_position": false' in output
    assert runner_mod.dry_run_plan()["opens_can"] is False
    assert runner_mod.dry_run_plan()["opens_serial"] is False
    source = SCRIPT.read_text(encoding="utf-8")
    assert 'create_publisher(\n                String, "/whole_arm/gui_command"' in source
    assert "import socket" not in source


def test_pre_workflow_control_startup_transient_is_not_permanently_latched():
    run = runner_mod.ActiveAcceptanceRunner(binding())
    transient = state(run.binding, 1_000_000_000, 1)
    transient["control_available_by_domain"]["J1"] = False
    run.observe_hardware_state(transient, now_ns=1_000_000_000)
    assert run.failure is None
    assert run.latest_hardware is None

    healthy = state(run.binding, 1_100_000_000, 2)
    run.observe_hardware_state(healthy, now_ns=1_100_000_000)
    assert run.latest_hardware is not None

    run.gravity_ladder_active = True
    active_failure = state(run.binding, 1_200_000_000, 3)
    active_failure["control_available_by_domain"]["J1"] = False
    run.observe_hardware_state(active_failure, now_ns=1_200_000_000)
    assert run.failure is not None
    assert run.failure.reason == "HARDWARE_WORKER_CONTROL_UNAVAILABLE"


def test_worker_feedforward_echo_requires_causal_match_within_300ms():
    expected = runner_mod._expected_go_worker_feedforward
    match = runner_mod._latest_go_worker_feedforward_echo_match
    zero_node = (0.0,) * 6
    ramp_node = (0.0, 0.1, 0.0, 0.0, 0.0, 0.0)
    history = [
        (1_000_000_000, 1, zero_node, expected(zero_node)),
        (1_100_000_000, 2, ramp_node, expected(ramp_node)),
    ]
    zero_echo = expected(zero_node)
    ramp_echo = expected(ramp_node)
    sources = {motor: 1_300_000_000 for motor in runner_mod.GO_MOTOR_NAMES}
    assert match(
        history, zero_echo, echo_source_ns_by_motor=sources
    ) == history[0]
    sources = {motor: 1_400_000_000 for motor in runner_mod.GO_MOTOR_NAMES}
    assert match(
        history, ramp_echo, echo_source_ns_by_motor=sources
    ) == history[1]
    sources = {motor: 1_300_000_001 for motor in runner_mod.GO_MOTOR_NAMES}
    assert match(
        history, zero_echo, echo_source_ns_by_motor=sources
    ) is None

    # Receipt time cannot make an echo causal when even one worker source
    # predates the node publication.
    sources = {motor: 1_200_000_000 for motor in runner_mod.GO_MOTOR_NAMES}
    sources["J5"] = 1_099_999_999
    assert match(
        history, ramp_echo, echo_source_ns_by_motor=sources
    ) is None


def test_worker_echo_wait_is_only_the_first_300ms_of_ramp():
    pending = runner_mod._worker_echo_propagation_pending
    assert pending(None, "RAMP", 0)
    assert pending(None, "RAMP", 300_000_000)
    assert pending(None, "RAMP", 300_000_001)
    assert pending(None, "HOLD", 1)
    assert not pending(None, "HOLD", 300_000_001)
    assert not pending(object(), "RAMP", 1)


def test_missing_worker_temperature_slope_is_derived_from_runner_samples():
    run = runner_mod.ActiveAcceptanceRunner(binding())
    metadata = {"J2A": {"slope_c_per_min": None}}
    assert run._optional_metadata_number(
        metadata, "J2A", "slope_c_per_min"
    ) is None
    rows = [{"monotonic_ns": 1_000_000_000, "temperature_c": 30.0}]
    assert run._derived_temperature_slope(
        rows, 61_000_000_000, 31.0, "temperature_c"
    ) == 1.0


def test_sticky_zero_and_terminal_jump_cannot_erase_echo_failure():
    expected = runner_mod._expected_go_worker_feedforward
    match = runner_mod._latest_go_worker_feedforward_echo_match
    history = []
    sticky_zero = expected((0.0,) * 6)
    failed = False
    for index in range(6):
        now_ns = 2_000_000_000 + index * 100_000_000
        node = (0.0, index * 0.02, 0.0, 0.0, 0.0, 0.0)
        history.append((now_ns, index + 1, node, expected(node)))
        sources = {motor: now_ns for motor in runner_mod.GO_MOTOR_NAMES}
        if match(
            history,
            sticky_zero,
            echo_source_ns_by_motor=sources,
        ) is None:
            failed = True
    assert failed is True  # old zero exceeded the explicit 300 ms bound

    # A final-value jump can match the newest publication, but the real
    # runner's fail() latch (represented here by `failed`) is irreversible.
    terminal = (0.0, 0.25, 0.0, 0.0, 0.0, 0.0)
    terminal_ns = 2_600_000_000
    history.append((terminal_ns, 7, terminal, expected(terminal)))
    sources = {motor: terminal_ns for motor in runner_mod.GO_MOTOR_NAMES}
    assert match(
        history,
        expected(terminal),
        echo_source_ns_by_motor=sources,
    ) == history[-1]
    assert failed is True


def test_interstage_confirmation_wait_is_continuous_hold_and_bounded_30s():
    run = runner_mod.ActiveAcceptanceRunner(binding())
    bound = run.binding
    started_ns = 3_000_000_000
    assert run.observe_confirmation(
        confirmation(bound, started_ns, target=0.0), now_ns=started_ns
    )
    run.gravity_ladder_active = True
    run.gravity_ladder_center_rad = (0.0,) * 6
    run.gravity_ladder_completed_levels = [0.0]
    zero_node = (0.0,) * 6
    expected_zero = runner_mod._expected_go_worker_feedforward(zero_node)

    def capture(sample_ns: int, sequence: int, *, modes=None) -> None:
        hardware = state(
            bound,
            sample_ns,
            sequence,
            modes=modes,
            gravity_scale=0.0,
        )
        run.latest_hardware = hardware
        run.latest_hardware_received_ns = sample_ns
        run.gravity_worker_echo_history = [(
            sample_ns - 1_000_000,
            10_000 + sequence,
            zero_node,
            expected_zero,
        )]
        status = gravity(
            bound,
            sample_ns,
            scale=0.0,
            stage_index=0,
            stage_complete=True,
            hardware_sequence=sequence,
            hardware_source_ns=sample_ns,
        )
        status["sequence"] = sequence
        status["source_monotonic_ns"] = sample_ns
        run._capture_gravity_ladder(status, sample_ns)

    for index in range(301):
        capture(started_ns + index * 100_000_000, index + 1)
    assert run.gravity_ladder_phase == "AWAIT_INTERSTAGE_CONFIRMATION"
    assert run.failure is None
    assert run.gravity_ladder_rows[-1]["phase"] == (
        "AWAIT_INTERSTAGE_CONFIRMATION"
    )
    assert run.gravity_ladder_rows[-1]["phase_elapsed_s"] == 30.0

    over_limit_ns = started_ns + 30_100_000_000
    assert run.observe_confirmation(
        confirmation(bound, over_limit_ns - 1, sequence=2, target=0.25),
        now_ns=over_limit_ns,
    )
    with pytest.raises(
        runner_mod.AcceptanceError,
        match="GRAVITY_LADDER_INTERSTAGE_CONFIRMATION_EXCEEDED_30_SECONDS",
    ):
        capture(over_limit_ns, 302)

    unsafe = runner_mod.ActiveAcceptanceRunner(binding())
    assert unsafe.observe_confirmation(
        confirmation(unsafe.binding, started_ns, target=0.0),
        now_ns=started_ns,
    )
    unsafe.gravity_ladder_active = True
    unsafe.gravity_ladder_center_rad = (0.0,) * 6
    unsafe.gravity_ladder_completed_levels = [0.0]
    run = unsafe
    unsafe_modes = {
        motor: "hold" for motor in runner_mod.MOTOR_NAMES
    }
    unsafe_modes["J6"] = "position"
    with pytest.raises(
        runner_mod.AcceptanceError,
        match="GRAVITY_LADDER_NOT_WHOLE_ARM_CONTINUOUS_HOLD",
    ):
        capture(started_ns, 1, modes=unsafe_modes)


def test_gravity_hardware_pair_cache_handles_callback_reordering_exactly():
    bound = binding()

    def configured_run():
        run = runner_mod.ActiveAcceptanceRunner(bound)
        run.gravity_ladder_active = True
        captured = []

        def capture(gravity_value, receipt_ns, *, state, hardware_receipt_ns):
            captured.append((
                gravity_value["hardware_state_sequence"],
                state["sequence"],
                receipt_ns,
                hardware_receipt_ns,
            ))

        run._capture_gravity_ladder = capture
        return run, captured

    base_ns = 4_000_000_000
    run, captured = configured_run()
    run.observe_hardware_state(
        state(bound, base_ns, 1, gravity_scale=0.0), now_ns=base_ns
    )
    run.observe_hardware_state(
        state(bound, base_ns + 1_000_000, 2, gravity_scale=0.0),
        now_ns=base_ns + 1_000_000,
    )
    delayed = gravity(
        bound,
        base_ns + 2_000_000,
        scale=0.0,
        hardware_sequence=1,
        hardware_source_ns=base_ns,
    )
    run.observe_gravity_status(delayed, now_ns=base_ns + 2_000_000)
    assert captured == [(1, 1, base_ns + 2_000_000, base_ns)]

    run, captured = configured_run()
    early = gravity(
        bound,
        base_ns + 1_000_000,
        scale=0.0,
        hardware_sequence=1,
        hardware_source_ns=base_ns,
    )
    run.observe_gravity_status(early, now_ns=base_ns + 1_000_000)
    assert captured == []
    assert len(run.pending_gravity_hardware_pairs) == 1
    run.observe_hardware_state(
        state(bound, base_ns, 1, gravity_scale=0.0),
        now_ns=base_ns + 2_000_000,
    )
    assert captured == [(
        1, 1, base_ns + 1_000_000, base_ns + 2_000_000,
    )]
    assert run.pending_gravity_hardware_pairs == []


def test_gravity_hardware_pair_conflict_and_missing_timeout_fail_closed():
    bound = binding()
    base_ns = 5_000_000_000
    run = runner_mod.ActiveAcceptanceRunner(bound)
    run.gravity_ladder_active = True
    run.observe_hardware_state(
        state(bound, base_ns, 1, gravity_scale=0.0), now_ns=base_ns
    )
    conflicting = gravity(
        bound,
        base_ns + 2_000_000,
        scale=0.0,
        hardware_sequence=1,
        hardware_source_ns=base_ns + 1,
    )
    run.observe_gravity_status(
        conflicting, now_ns=base_ns + 2_000_000
    )
    assert run.failure is not None
    assert run.failure.reason == "GRAVITY_LADDER_HARDWARE_PAIR_CONFLICT"

    run = runner_mod.ActiveAcceptanceRunner(bound)
    run.gravity_ladder_active = True
    assert run.observe_confirmation(
        confirmation(bound, base_ns, target=0.0), now_ns=base_ns
    )
    missing = gravity(
        bound,
        base_ns + 1_000_000,
        scale=0.0,
        hardware_sequence=99,
        hardware_source_ns=base_ns,
    )
    run.observe_gravity_status(missing, now_ns=base_ns + 1_000_000)
    assert run.failure is None
    run.tick(
        now_ns=(
            base_ns
            + 1_000_000
            + runner_mod.MAXIMUM_GRAVITY_HARDWARE_PAIR_WAIT_NS
            + 1
        )
    )
    assert run.failure is not None
    assert (
        run.failure.reason
        == "GRAVITY_LADDER_HARDWARE_PAIR_MISSING_TIMEOUT"
    )


def test_startup_gate_is_exact_and_confirmation_is_bound_fresh_and_nonreplayed():
    with pytest.raises(runner_mod.AcceptanceError):
        runner_mod.validate_physical_startup_gate({})
    runner_mod.validate_physical_startup_gate({
        runner_mod.PHYSICAL_CONFIRMATION_ENV: runner_mod.PHYSICAL_CONFIRMATION_GATE
    })
    bound = binding()
    run = runner_mod.ActiveAcceptanceRunner(bound)
    now = 10_000_000_000
    assert run.observe_confirmation(confirmation(bound, now), now_ns=now)
    assert not run.observe_confirmation(confirmation(bound, now), now_ns=now + 1)
    stale = confirmation(bound, now + 1, 2)
    assert not run.observe_confirmation(stale, now_ns=now + 31_000_000_001)


def test_binding_loader_rejects_non_authoritative_contract_and_accepts_exact():
    bound = binding()
    envelope = {
        "schema": runner_mod.ENVELOPE_SCHEMA,
        "single_use": True,
        "authority_class": "EMPIRICAL_VALIDATION_ENVELOPE",
        "rating_classification": "NOT_OFFICIAL_CONTINUOUS_RATING",
        "envelope_id": bound.envelope_id,
        "expires_at_utc": bound.expires_at_utc.isoformat().replace("+00:00", "Z"),
        "binding": {
            "session_id": bound.session_id,
            "state_instance_id": bound.state_instance_id,
            "anchor_sha256": bound.anchor_sha256,
            "model_sha256": runner_mod.PRODUCTION_MODEL_SHA256,
            "gravity_config_sha256": runner_mod.GRAVITY_CONFIG_SHA256,
            "thermal_config_sha256": runner_mod.THERMAL_CONFIG_SHA256,
            "all_fields_must_match_runtime": True,
        },
        "position_validation": {
            "enabled": True,
            "unlock_requires_completed_gravity_ladder": True,
            "single_joint_first_required": True,
            "maximum_moving_joints_before_single_joint_pass": 1,
            "no_progress_watchdog_required_every_cycle": True,
            "temperature_required_every_cycle": True,
            "continuous_operation_authorized": False,
            runner_mod.POSITION_BUDGET_FIELD: 600.0,
            "maximum_segment_seconds": 15.0,
            "maximum_abs_segment_displacement_deg": 5.0,
            "endpoint_error_limit_deg": 0.5,
            "endpoint_dwell_seconds": 0.5,
        },
        "live_gates": {
            "temperature": {
                "realtime_valid_required": True,
                "hard_stop_c": 60.0,
                "rapid_rise_aborts_stage": True,
                "maximum_rise_within_one_stage_c": 2.0,
            },
            "operator_stop": {
                "must_remain_available": True,
                "confirmation_maximum_age_seconds": 30.0,
            },
            "physical_support": {"j2_j3_reliable_support_required": True},
            "j2_sync": {"warning_above_deg": 0.25, "hard_above_deg": 0.5},
        },
    }
    anchor = {
        "schema": runner_mod.ANCHOR_VALIDATION_SCHEMA,
        "result": "PASS",
        "session_id": bound.session_id,
        "state_instance_id": bound.state_instance_id,
        "runtime_anchor_sha256": bound.anchor_sha256,
        "checks": {
            "session_match": True,
            "state_instance_match": True,
            "model_hash_match": True,
            "motor_zero_modified": False,
            "rid_modified": False,
            "flash_or_eeprom_written": False,
        },
    }
    loaded = runner_mod.EvidenceBinding.from_documents(envelope, anchor, "a" * 64)
    assert loaded == bound
    envelope["position_validation"]["maximum_segment_seconds"] = 16.0
    with pytest.raises(runner_mod.AcceptanceError):
        runner_mod.EvidenceBinding.from_documents(envelope, anchor, "a" * 64)


def test_comparison_gravity_ladder_position_order_is_enforced():
    run = runner_mod.ActiveAcceptanceRunner(binding())
    bound = run.binding
    now = 19_000_000_000
    assert run.observe_confirmation(confirmation(bound, now), now_ns=now)
    refresh_gravity(run, now, scale=0.0)
    run.observe_hardware_state(
        state(bound, now, 1, gravity_scale=0.0), now_ns=now
    )

    with pytest.raises(
        runner_mod.AcceptanceError,
        match="GRAVITY_LADDER_REQUIRES_COMPLETED_WITHOUT_FF_COMPARISON",
    ):
        run.start_gravity_ladder(now_ns=now)

    run.comparison_complete = {"WITHOUT_FF"}
    run.start_gravity_ladder(now_ns=now)
    assert run.gravity_ladder_active

    run.gravity_ladder_active = False
    run.gravity_ladder_complete = True
    refresh_gravity(run, now + 1, scale=1.0)
    with pytest.raises(
        runner_mod.AcceptanceError,
        match="POSITION_REQUIRES_COMPLETED_WITHOUT_AND_WITH_FF_COMPARISON",
    ):
        run.start_position(now_ns=now + 1)

    run.comparison_complete.add("WITH_FF")
    run.start_position(now_ns=now + 1)
    assert run.position_started_ns == now + 1


def execute_full_position(run):
    bound = run.binding
    now = 20_000_000_000
    seq = 1
    positions = [0.0] * 6
    assert run.observe_confirmation(confirmation(bound, now), now_ns=now)
    refresh_gravity(run, now)
    run.observe_hardware_state(state(bound, now, seq, positions=positions), now_ns=now)
    run.gravity_ladder_complete = True
    run.comparison_complete = {"WITHOUT_FF", "WITH_FF"}
    run.start_position(now_ns=now)
    for _ in range(6):
        now += 100_000_000
        seq += 1
        refresh_gravity(run, now)
        run.observe_hardware_state(
            state(bound, now, seq, positions=positions), now_ns=now
        )
    command_counter = 1
    while not run.position_complete:
        assert run.awaiting_position_command
        now += 10_000_000
        command, target, trajectory_sha, plan_token = position_command(
            bound, run, now, command_counter, positions
        )
        refresh_gravity(run, now, trajectory_sha)
        run.observe_gui_command(command, now_ns=now)
        run.observe_router_status({
            "schema": runner_mod.ROUTER_STATUS_SCHEMA,
            "last_mode": "position",
            "last_moving_joint_mask": command["moving_joint_mask"],
            "last_command_age_ms": 1.0,
        }, now_ns=now)
        positions = list(target)
        moving_modes = {motor: "hold" for motor in runner_mod.MOTOR_NAMES}
        for motor in runner_mod.MOTOR_BY_JOINT[run.expected_joint]:
            moving_modes[motor] = "position"
        for _ in range(6):
            now += 100_000_000
            seq += 1
            refresh_gravity(run, now, trajectory_sha)
            run.observe_hardware_state(state(
                bound, now, seq, positions=positions, modes=moving_modes,
                trajectory_joint=run.expected_joint,
                trajectory_sha=trajectory_sha,
                plan_token=plan_token,
            ), now_ns=now)
        if run.position_complete:
            break
        if run.expected_phase == "CENTER_START":
            for _ in range(6):
                now += 100_000_000
                seq += 1
                refresh_gravity(run, now)
                run.observe_hardware_state(state(
                    bound, now, seq, positions=positions
                ), now_ns=now)
        command_counter += 1
    return now, seq, positions


def test_complete_position_sequence_is_gui_router_bound_and_validator_compatible(tmp_path):
    run = runner_mod.ActiveAcceptanceRunner(binding())
    now, _seq, _positions = execute_full_position(run)
    assert run.failure is None
    assert run.position_complete
    assert [row["joint"] for row in run.position_rows] == [
        joint for joint in runner_mod.POSITION_ORDER for _ in range(5)
    ]
    assert [row["phase"] for row in run.position_rows[:5]] == list(
        runner_mod.POSITION_PHASES
    )
    assert run.position_trajectory_budget_ns == 24 * 2_000_000_000
    for row in run.position_rows:
        if row["phase"] == "CENTER_START":
            assert row["trajectory_duration_ns"] == "CENTER_OBSERVATION"
            assert (
                row["trajectory_execute_at_monotonic_ns"]
                == "CENTER_OBSERVATION"
            )
        else:
            assert 0 < row["trajectory_duration_ns"] <= 15_000_000_000
            assert row["trajectory_execute_at_monotonic_ns"] > 0
    # This fixture validates only the position CSV.  The comparison-complete
    # state above is the required start gate, not a substitute for real
    # comparison rows in a full evidence bundle.
    run.comparison_complete.clear()
    written = run.write_evidence(tmp_path)
    assert [path.name for path in written] == ["position_validation.csv"]
    validator_path = SCRIPT.parents[1] / "validate_v15_31b_ft_evidence.py"
    validator_spec = importlib.util.spec_from_file_location("v15_31b_validator", validator_path)
    validator = importlib.util.module_from_spec(validator_spec)
    assert validator_spec.loader is not None
    validator_spec.loader.exec_module(validator)
    expected_binding = {
        "envelope_id": run.binding.envelope_id,
        "envelope_sha256": run.binding.envelope_sha256,
        "session_id": run.binding.session_id,
        "state_instance_id": run.binding.state_instance_id,
        "anchor_sha256": run.binding.anchor_sha256,
    }
    status, metrics = validator._validate_position(
        tmp_path / "position_validation.csv", expected_binding,
    )
    assert status == "PASS"
    assert metrics["maximum_error_deg"] == {joint: 0.0 for joint in runner_mod.JOINT_NAMES}


def test_wrong_joint_or_long_segment_fails_closed_and_requests_router_brake():
    brakes = []
    run = runner_mod.ActiveAcceptanceRunner(binding(), brake_callback=brakes.append)
    bound = run.binding
    now = 30_000_000_000
    assert run.observe_confirmation(confirmation(bound, now), now_ns=now)
    refresh_gravity(run, now)
    run.observe_hardware_state(state(bound, now, 1), now_ns=now)
    run.gravity_ladder_complete = True
    run.comparison_complete = {"WITHOUT_FF", "WITH_FF"}
    run.start_position(now_ns=now)
    for seq in range(2, 8):
        now += 100_000_000
        refresh_gravity(run, now)
        run.observe_hardware_state(state(bound, now, seq), now_ns=now)
    command, _target, sha, _token = position_command(bound, run, now + 1, 1, [0.0] * 6)
    command["trajectory"]["duration_ns"] = 15_000_000_001
    refresh_gravity(run, now + 1, sha)
    run.observe_gui_command(command, now_ns=now + 1)
    assert run.failure.reason == "POSITION_SEGMENT_EXCEEDS_15_SECONDS"
    assert brakes and brakes[0].related_domains == ("J1",)
    assert brakes[0].brake_scope == "ALL_DOMAINS_FAIL_CLOSED_SUPERSET"


def test_confirmation_expiry_aborts_an_active_run():
    brakes = []
    run = runner_mod.ActiveAcceptanceRunner(binding(), brake_callback=brakes.append)
    bound = run.binding
    now = 40_000_000_000
    assert run.observe_confirmation(confirmation(bound, now), now_ns=now)
    refresh_gravity(run, now)
    run.observe_hardware_state(state(bound, now, 1), now_ns=now)
    run.gravity_ladder_complete = True
    run.comparison_complete = {"WITHOUT_FF", "WITH_FF"}
    run.start_position(now_ns=now)
    run.latest_hardware_received_ns = now + 31_000_000_000
    run.latest_gravity_received_ns = now + 31_000_000_000
    run.tick(now_ns=now + 31_000_000_000)
    assert run.failure.reason == "OPERATOR_CONFIRMATION_STALE"
    assert brakes


def test_confirmation_trace_retains_only_valid_original_events_with_receipt_time():
    run = runner_mod.ActiveAcceptanceRunner(binding())
    now = 49_000_000_000
    event = confirmation(run.binding, now, sequence=7, target=0.5)
    original = dict(event)
    assert run.observe_confirmation(event, now_ns=now + 123)
    event["operator_stop_ready"] = False
    assert run.operator_confirmation_trace == [{
        **original,
        "observer_receipt_monotonic_ns": now + 123,
    }]
    assert not run.observe_confirmation(original, now_ns=now + 124)
    assert len(run.operator_confirmation_trace) == 1


def test_position_budget_counts_repeated_sha_as_a_different_new_segment():
    run = runner_mod.ActiveAcceptanceRunner(binding())
    run.position_trajectory_budget_ns = 599_000_000_000
    repeated_sha = "f" * 64
    run._budgeted_trajectory_sha256.add(repeated_sha)
    bound = run.binding
    now = 50_000_000_000
    assert run.observe_confirmation(confirmation(bound, now), now_ns=now)
    refresh_gravity(run, now)
    run.observe_hardware_state(state(bound, now, 1), now_ns=now)
    run.gravity_ladder_complete = True
    run.comparison_complete = {"WITHOUT_FF", "WITH_FF"}
    run.start_position(now_ns=now)
    for seq in range(2, 8):
        now += 100_000_000
        refresh_gravity(run, now)
        run.observe_hardware_state(state(bound, now, seq), now_ns=now)
    command, _target, _sha, _token = position_command(
        bound, run, now + 1, 1, [0.0] * 6
    )
    command["trajectory"]["trajectory_sha256"] = repeated_sha
    command["plan_manifest"]["segment_sha256"] = [repeated_sha]
    refresh_gravity(run, now + 1, repeated_sha)
    run.observe_gui_command(command, now_ns=now + 1)
    assert run.failure.reason == "POSITION_TRAJECTORY_BUDGET_EXCEEDED_600_SECONDS"


def test_endpoint_dwell_resets_across_a_telemetry_gap_over_100ms():
    run = runner_mod.ActiveAcceptanceRunner(binding())
    bound = run.binding
    now = 55_000_000_000
    seq = 1
    assert run.observe_confirmation(confirmation(bound, now), now_ns=now)
    refresh_gravity(run, now)
    run.observe_hardware_state(state(bound, now, seq), now_ns=now)
    run.gravity_ladder_complete = True
    run.comparison_complete = {"WITHOUT_FF", "WITH_FF"}
    run.start_position(now_ns=now)
    now += 1
    seq += 1
    refresh_gravity(run, now)
    run.observe_hardware_state(state(bound, now, seq), now_ns=now)
    now += 500_000_000
    seq += 1
    refresh_gravity(run, now)
    run.observe_hardware_state(state(bound, now, seq), now_ns=now)
    assert run.position_rows == []
    for _ in range(5):
        now += 100_000_000
        seq += 1
        refresh_gravity(run, now)
        run.observe_hardware_state(state(bound, now, seq), now_ns=now)
    assert len(run.position_rows) == 1


def test_position_rejects_endpoint_that_did_not_actually_move_five_degrees():
    run = runner_mod.ActiveAcceptanceRunner(binding())
    bound = run.binding
    now = 56_000_000_000
    sequence = 1
    positions = [0.0] * 6
    assert run.observe_confirmation(confirmation(bound, now), now_ns=now)
    refresh_gravity(run, now)
    run.observe_hardware_state(
        state(bound, now, sequence, positions=positions), now_ns=now
    )
    run.gravity_ladder_complete = True
    run.comparison_complete = {"WITHOUT_FF", "WITH_FF"}
    run.start_position(now_ns=now)
    for _ in range(6):
        now += 100_000_000
        sequence += 1
        refresh_gravity(run, now)
        run.observe_hardware_state(
            state(bound, now, sequence, positions=positions), now_ns=now
        )
    assert run.expected_phase == "PLUS_5"
    position_status = run.status(now_ns=now)["position"]
    assert position_status["expected_joint"] == "J1"
    assert position_status["expected_phase"] == "PLUS_5"
    assert position_status["expected_target_rad"] == pytest.approx(
        math.radians(5.0)
    )
    assert position_status["expected_target_deg"] == pytest.approx(5.0)
    assert position_status["expected_target_vector_rad"] == pytest.approx(
        [math.radians(5.0), 0.0, 0.0, 0.0, 0.0, 0.0]
    )
    now += 1
    command, target, trajectory_sha, plan_token = position_command(
        bound, run, now, 1, positions
    )
    refresh_gravity(run, now, trajectory_sha)
    run.observe_gui_command(command, now_ns=now)
    run.observe_router_status({
        "schema": runner_mod.ROUTER_STATUS_SCHEMA,
        "last_mode": "position",
        "last_moving_joint_mask": command["moving_joint_mask"],
        "last_command_age_ms": 1.0,
    }, now_ns=now)
    target[0] = math.radians(4.5001)
    modes = {motor: "hold" for motor in runner_mod.MOTOR_NAMES}
    modes["J1"] = "position"
    for _ in range(6):
        now += 100_000_000
        sequence += 1
        refresh_gravity(run, now, trajectory_sha)
        run.observe_hardware_state(state(
            bound,
            now,
            sequence,
            positions=target,
            modes=modes,
            trajectory_joint="J1",
            trajectory_sha=trajectory_sha,
            plan_token=plan_token,
        ), now_ns=now)
    assert run.failure is not None
    assert run.failure.reason == "POSITION_PLUS_ACTUAL_MOTION_BELOW_5_DEG"


def test_hardware_state_rejects_new_sequence_with_replayed_source_time():
    run = runner_mod.ActiveAcceptanceRunner(binding())
    bound = run.binding
    now = 56_900_000_000
    run.observe_hardware_state(state(bound, now, 1), now_ns=now)
    replayed_source = state(bound, now, 2)
    run.observe_hardware_state(replayed_source, now_ns=now + 1)
    assert run.failure is not None
    assert run.failure.reason == "HARDWARE_STATE_REPLAYED"


def test_comparison_gap_and_two_row_thermal_shortcut_fail_closed():
    brakes = []
    run = runner_mod.ActiveAcceptanceRunner(
        binding(), brake_callback=brakes.append
    )
    bound = run.binding
    now = 57_000_000_000
    assert run.observe_confirmation(confirmation(bound, now), now_ns=now)
    refresh_gravity(run, now, scale=0.0)
    run.observe_hardware_state(
        state(bound, now, 1, gravity_scale=0.0), now_ns=now
    )
    run.start_comparison("WITHOUT_FF", 0.0, now_ns=now)
    for sequence, offset in ((2, 1), (3, 200_000_001)):
        sample_now = now + offset
        refresh_gravity(run, sample_now, scale=0.0)
        run.observe_hardware_state(
            state(bound, sample_now, sequence, gravity_scale=0.0),
            now_ns=sample_now,
        )
    assert run.failure is not None
    assert run.failure.reason == "COMPARISON_SAMPLE_GAP_EXCEEDED_100MS"
    assert brakes

    fake_rows = [
        {"monotonic_ns": 1, "elapsed_s": 0.0},
        {"monotonic_ns": 300_000_000_001, "elapsed_s": 300.0},
    ]
    with pytest.raises(
        runner_mod.AcceptanceError,
        match="THERMAL_CONTINUOUS_SAMPLE_COUNT_INSUFFICIENT",
    ):
        runner_mod.ActiveAcceptanceRunner._validate_thermal_rows(5, fake_rows)


def run_comparisons(run, now, seq):
    bound = run.binding
    refresh_gravity(run, now, scale=0.0)
    baseline_positions = [0.0, -math.radians(0.4), 0.0, 0.0, 0.0, 0.0]
    improved_positions = [0.0, -math.radians(0.1), 0.0, 0.0, 0.0, 0.0]
    run.observe_hardware_state(state(
        bound, now, seq, positions=baseline_positions, gravity_scale=0.0,
        j2_pd_abs=0.4, j2_feedback_abs=0.5, saturation=True,
    ), now_ns=now)
    run.start_comparison("WITHOUT_FF", 0.0, now_ns=now)
    offsets = (
        1, 100_000_001, 200_000_001, 300_000_001,
        400_000_001, 500_000_001, 500_000_002,
    )
    for offset in offsets:
        seq += 1
        sample_now = now + offset
        refresh_gravity(run, sample_now, scale=0.0)
        run.observe_hardware_state(state(
            bound, sample_now, seq, positions=baseline_positions,
            gravity_scale=0.0, j2_pd_abs=0.4,
            j2_feedback_abs=0.5, saturation=True,
        ), now_ns=sample_now)
    now += offsets[-1]
    run.stop_comparison(now_ns=now)
    now += 1
    refresh_gravity(run, now, scale=1.0)
    run.gravity_ladder_complete = True
    run.start_comparison("WITH_FF", 0.0, now_ns=now)
    for offset in offsets:
        seq += 1
        sample_now = now + offset
        refresh_gravity(run, sample_now, scale=1.0)
        run.observe_hardware_state(state(
            bound, sample_now, seq, positions=improved_positions,
            gravity_scale=1.0, j2_pd_abs=0.03,
            j2_feedback_abs=0.15, saturation=False,
        ), now_ns=sample_now)
    now += offsets[-1]
    run.stop_comparison(now_ns=now)
    return now, seq


def test_comparison_and_full_thermal_state_machine_emit_validator_compatible_evidence(tmp_path):
    run = runner_mod.ActiveAcceptanceRunner(binding())
    bound = run.binding
    now = 60_000_000_000
    seq = 1
    confirmation_seq = 1
    assert run.observe_confirmation(
        confirmation(bound, now, confirmation_seq), now_ns=now
    )
    now, seq = run_comparisons(run, now, seq)
    assert run.comparison_complete == {"WITHOUT_FF", "WITH_FF"}
    run.position_complete = True
    run.position_started_ns = now
    run.center_rad = (0.0,) * 6
    run.post_workflow_phase = "AWAIT_MULTI_JOINT"
    multi_target = [math.radians(1.0), math.radians(1.0), 0.0, 0.0, 0.0, 0.0]
    now, seq, positions = execute_post_position(
        run, "MULTI_JOINT", multi_target, now + 1, seq,
        [0.0, -math.radians(0.1), 0.0, 0.0, 0.0, 0.0],
    )
    thermal_target = [0.0, math.radians(2.0), 0.0, 0.0, 0.0, 0.0]
    now, seq, positions = execute_post_position(
        run, "THERMAL_SETUP", thermal_target, now + 1, seq, positions,
    )
    for stage in (5, 15, 30):
        now += 1
        refresh_gravity(run, now)
        seq += 1
        run.observe_hardware_state(
            state(bound, now, seq, positions=positions), now_ns=now
        )
        run.start_thermal_stage(stage, positions[1], now_ns=now)
        seconds = int(runner_mod.THERMAL_STAGE_SECONDS[stage])
        for elapsed in range(0, seconds + 1):
            sample_now = now + elapsed * 1_000_000_000 + 1
            if sample_now - run.confirmation["source_monotonic_ns"] >= 10_000_000_000:
                confirmation_seq += 1
                assert run.observe_confirmation(
                    confirmation(bound, sample_now, confirmation_seq),
                    now_ns=sample_now,
                )
            refresh_gravity(run, sample_now)
            seq += 1
            fraction = elapsed / max(1, seconds)
            run.observe_hardware_state(state(
                bound, sample_now, seq, positions=positions,
                temperature_a=30.0 + 0.5 * fraction,
                temperature_b=30.0 + 0.4 * fraction,
                slope_a={5: 0.4, 15: 0.25, 30: 0.1}[stage],
                slope_b={5: 0.35, 15: 0.2, 30: 0.08}[stage],
            ), now_ns=sample_now)
            assert run.failure is None
        now += seconds * 1_000_000_000 + 1
        assert run.thermal_pending_review == stage
        run.approve_thermal_stage(stage, now_ns=now)
    summary = run.finalize_thermal_summary(
        classification="CONTROL_EXCESS_TORQUE_SOLVED"
    )
    assert summary["result"] == "PASS"
    # This test isolates the thermal collector; the full position artifact is
    # exercised independently above.
    run.position_complete = False
    written = run.write_evidence(tmp_path)
    assert {path.name for path in written} == {
        "j2_control_comparison.csv",
        "thermal_5min.csv",
        "thermal_15min.csv",
        "thermal_30min.csv",
        "thermal_summary.json",
    }
    validator_path = SCRIPT.parents[1] / "validate_v15_31b_ft_evidence.py"
    validator_spec = importlib.util.spec_from_file_location("v15_31b_validator_thermal", validator_path)
    validator = importlib.util.module_from_spec(validator_spec)
    assert validator_spec.loader is not None
    validator_spec.loader.exec_module(validator)
    expected_binding = {
        "envelope_id": bound.envelope_id,
        "envelope_sha256": bound.envelope_sha256,
        "session_id": bound.session_id,
        "state_instance_id": bound.state_instance_id,
        "anchor_sha256": bound.anchor_sha256,
    }
    comparison_status, _ = validator._validate_j2_comparison(
        tmp_path / "j2_control_comparison.csv", expected_binding,
    )
    assert comparison_status == "PASS"
    for stage in (5, 15, 30):
        status_value, _ = validator._validate_thermal_csv(
            tmp_path / f"thermal_{stage}min.csv", stage, expected_binding,
        )
        assert status_value == "PASS"


def test_thermal_rise_aborts_current_run_instead_of_continuing_to_30min():
    brakes = []
    run = runner_mod.ActiveAcceptanceRunner(binding(), brake_callback=brakes.append)
    bound = run.binding
    now = 70_000_000_000
    assert run.observe_confirmation(confirmation(bound, now), now_ns=now)
    refresh_gravity(run, now)
    run.observe_hardware_state(state(bound, now, 1), now_ns=now)
    run.position_complete = True
    run.position_started_ns = now
    run.center_rad = (0.0,) * 6
    run.post_workflow_phase = "AWAIT_MULTI_JOINT"
    now, seq, positions = execute_post_position(
        run, "MULTI_JOINT",
        [math.radians(1.0), math.radians(1.0), 0.0, 0.0, 0.0, 0.0],
        now + 1, 1, [0.0] * 6,
    )
    now, seq, positions = execute_post_position(
        run, "THERMAL_SETUP",
        [0.0, math.radians(2.0), 0.0, 0.0, 0.0, 0.0],
        now + 1, seq, positions,
    )
    run.start_thermal_stage(5, positions[1], now_ns=now)
    for sequence, (offset, temperature) in enumerate(
        ((1, 30.0), (1_000_000_001, 32.1)), start=seq + 1
    ):
        sample_now = now + offset
        refresh_gravity(run, sample_now)
        run.observe_hardware_state(state(
            bound, sample_now, sequence, positions=positions,
            temperature_a=temperature, temperature_b=temperature,
        ), now_ns=sample_now)
    assert run.failure.reason == "THERMAL_STAGE_RAPID_OR_EXCESSIVE_RISE"
    assert run.thermal_stage is None
    assert brakes


def test_safe_five_minute_fundamental_limit_short_circuits_to_restore_and_final(
    tmp_path,
):
    brakes = []
    _repo, seal_dir, provenance = synced_repo_with_supporting_evidence(
        tmp_path
    )
    run = runner_mod.ActiveAcceptanceRunner(
        binding(),
        brake_callback=brakes.append,
        run_git_provenance=provenance,
    )
    bound = run.binding
    now = 75_000_000_000
    seq = 1
    confirmation_seq = 1
    assert run.observe_confirmation(
        confirmation(bound, now, confirmation_seq), now_ns=now
    )
    now, seq = run_comparisons(run, now, seq)
    run.position_complete = True
    run.position_started_ns = now
    run.center_rad = (0.0,) * 6
    run.post_workflow_phase = "AWAIT_MULTI_JOINT"
    with pytest.raises(runner_mod.AcceptanceError):
        run.start_post_position_execution(
            "THERMAL_SETUP",
            [0.0, math.radians(2.0), 0.0, 0.0, 0.0, 0.0],
            now_ns=now,
        )
    now, seq, positions = execute_post_position(
        run,
        "MULTI_JOINT",
        [math.radians(1.0), math.radians(1.0), 0.0, 0.0, 0.0, 0.0],
        now + 1,
        seq,
        [0.0, -math.radians(0.1), 0.0, 0.0, 0.0, 0.0],
    )
    with pytest.raises(runner_mod.AcceptanceError):
        run.start_post_position_execution(
            "RESTORE_INITIAL", [0.0] * 6, now_ns=now
        )
    now, seq, positions = execute_post_position(
        run,
        "THERMAL_SETUP",
        [0.0, math.radians(2.0), 0.0, 0.0, 0.0, 0.0],
        now + 1,
        seq,
        positions,
    )
    for record in run.completed_post_execution["MULTI_JOINT"]["segments"]:
        assert 0 < record["trajectory_duration_ns"] <= 15_000_000_000
        assert record["trajectory_execute_at_monotonic_ns"] > 0
    run.start_thermal_stage(5, positions[1], now_ns=now)
    stage_started = now
    for elapsed in range(301):
        sample_now = stage_started + elapsed * 1_000_000_000 + 1
        if sample_now - run.confirmation["source_monotonic_ns"] >= 10_000_000_000:
            confirmation_seq += 1
            assert run.observe_confirmation(
                confirmation(bound, sample_now, confirmation_seq),
                now_ns=sample_now,
            )
        refresh_gravity(run, sample_now)
        seq += 1
        fraction = elapsed / 300.0
        run.observe_hardware_state(state(
            bound,
            sample_now,
            seq,
            positions=positions,
            temperature_a=30.0 + 1.3 * fraction,
            temperature_b=30.0 + 1.2 * fraction,
            slope_a=0.30,
            slope_b=0.28,
        ), now_ns=sample_now)
        assert run.failure is None
    now = stage_started + 300_000_000_001
    run.approve_thermal_stage(5, now_ns=now)
    required_reduction = (
        0.4
        * runner_mod.GO_GEAR_RATIO
        * runner_mod.MECHANICAL_UNLOAD_FRACTION
    )
    lever_arm_m = 0.25
    counterweight_mass_kg = required_reduction / (
        runner_mod.STANDARD_GRAVITY_M_S2 * lever_arm_m
    )
    recommendation = {
        "schema": "V15.31B-mechanical-counterbalance-recommendation-v1",
        "required_j2_torque_reduction_nm": required_reduction,
        "counterweight": {
            "mass_kg": counterweight_mass_kg,
            "lever_arm_m": lever_arm_m,
            "gravity_m_s2": runner_mod.STANDARD_GRAVITY_M_S2,
            "estimated_torque_nm": required_reduction,
        },
    }
    summary = run.finalize_thermal_summary(
        classification="FUNDAMENTAL_CONTINUOUS_LOAD_LIMIT",
        mechanical_recommendation=recommendation,
        now_ns=now,
    )
    assert summary["result"] == "HARDWARE_COUNTERBALANCE_REQUIRED"
    assert summary["thermal_stage_disposition"] == {
        "5": "PASS_OBSERVED",
        "15": "NOT_RUN_EARLY_FUNDAMENTAL_LIMIT",
        "30": "NOT_RUN_EARLY_FUNDAMENTAL_LIMIT",
    }
    bound_map = {
        "envelope_id": bound.envelope_id,
        "envelope_sha256": bound.envelope_sha256,
        "session_id": bound.session_id,
        "state_instance_id": bound.state_instance_id,
        "anchor_sha256": bound.anchor_sha256,
    }
    run.record_multi_joint_attestation({
        "binding": bound_map,
        "planned_preview": "PASS",
        "preview_before_real": "PASS",
        "planned_twin": "PASS",
        "actual_twin": "PASS",
        "explicit_user_submit": True,
        "actual_twin_source": "REAL_ENCODER",
    })
    now, seq, positions = execute_post_position(
        run, "RESTORE_INITIAL", [0.0] * 6, now + 1, seq, positions,
    )
    run.record_restore_attestation({
        "binding": bound_map,
        "planned_preview": "PASS",
        "explicit_user_submit": True,
        "actual_twin_source": "REAL_ENCODER",
    })
    assert run.multi_joint_document is not None
    assert run.multi_joint_document["thermal_setup"]["result"] == "PASS"
    assert len(
        run.multi_joint_document["thermal_setup"]["final_dwell_trace"]
    ) >= runner_mod.MINIMUM_HALF_SECOND_SAMPLE_COUNT
    run.request_final_brake(now_ns=now)
    modes = {motor: "brake" for motor in runner_mod.MOTOR_NAMES}
    for index in range(1, 7):
        sample_now = now + index * 100_000_000
        run.observe_motor_feedback_raw({
            "schema": "go-m8010-motor-feedback/1.0",
            "source_instance_id": "5" * 32,
            "sequence": index,
            "session_id": bound.session_id,
            "state_instance_id": bound.state_instance_id,
            "source_monotonic_ns": sample_now,
            "samples": [{
                "motor": "J6", "communication_ok": True, "merror": 0,
            }],
            "drive_state": 0,
            "controller_mode": "brake",
            "controller_mode_by_motor": {"J6": "brake"},
        }, now_ns=sample_now)
        refresh_gravity(run, sample_now)
        seq += 1
        run.observe_hardware_state(
            state(
                bound,
                sample_now,
                seq,
                positions=positions,
                modes=modes,
                j6_raw_identity={
                    "source_instance_id": "5" * 32,
                    "sequence": index,
                    "source_monotonic_ns": sample_now,
                    "session_id": bound.session_id,
                    "state_instance_id": bound.state_instance_id,
                },
            ),
            now_ns=sample_now,
        )
    assert run.final_brake_observed
    assert len(run.final_brake_pairs) == 6
    assert run.post_workflow_phase == "COMPLETE"
    assert not any(
        event["mode"] in {"brake", "drag"}
        and not event["final_brake_requested"]
        for event in run.success_path_mode_events
    )
    assert not any(
        not event["final_brake_requested"]
        and (
            event["j6_control_available"] is not True
            or any(
                mode not in {"hold", "position"}
                for mode in event["modes"].values()
            )
        )
        for event in run.success_path_hardware_events
    )
    assert brakes[-1].reason == "FINAL_ACCEPTANCE_COMPLETE"

    gui_fields = {
        "single_slider_group", "drag_controls_planned_only",
        "planned_mujoco_animation", "actual_twin_encoder_only",
        "preview_before_real", "explicit_real_submit", "execution_status",
        "progress_percent", "estimated_remaining", "heartbeat",
        "temperature_table", "online_status_7_motors", "temperature_colors",
        "position_target_error",
    }
    run.record_gui_attestation({
        "binding": bound_map,
        "checks": {field: "PASS" for field in gui_fields},
    })
    run.position_rows = []
    for joint in runner_mod.JOINT_NAMES:
        for phase in runner_mod.POSITION_PHASES:
            row = {field: "" for field in runner_mod.POSITION_FIELDS}
            row.update({
                "joint": joint,
                "phase": phase,
                "error_deg": 0.0,
                "j2_e_sync_deg": 0.0,
                "status": "PASS",
            })
            run.position_rows.append(row)
    scale_row = {field: "" for field in runner_mod.GRAVITY_SCALE_FIELDS}
    scale_row.update({"status": "PASS", "j2_e_sync_deg": 0.0})
    run.gravity_ladder_complete = True
    run.gravity_ladder_rows = [scale_row]
    final_document = run.seal_evidence_bundle(
        evidence_directory=seal_dir,
    )
    report = {
        item["name"]: item["value"]
        for item in final_document["report_fields"]
    }
    assert final_document["result"] == "HARDWARE_COUNTERBALANCE_REQUIRED"
    assert report["PRIMARY BLOCKER"] == "HARDWARE_COUNTERBALANCE_REQUIRED"
    assert report["READY_FOR_NEXT_STAGE"] is False
    assert report["implementation commit"] == provenance.run_base_commit
    assert report["evidence commit"] == provenance.run_base_commit
    assert final_document["binding"]["run_git_provenance"][
        "ahead_count"
    ] == 0
    confirmation_trace = final_document["binding"][
        "operator_confirmation_trace"
    ]
    assert confirmation_trace == run.operator_confirmation_trace
    assert confirmation_trace
    confirmation_trace_document = {
        "schema": "V15.31B-operator-confirmation-trace-v1",
        "samples": confirmation_trace,
    }
    assert final_document["binding"][
        "operator_confirmation_trace_sha256"
    ] == runner_mod._document_sha256(confirmation_trace_document)
    assert all(
        set(sample)
        == {
            "schema", "source_instance_id", "sequence",
            "source_monotonic_ns", "envelope_id", "envelope_sha256",
            "session_id", "state_instance_id", "target_gravity_scale",
            "operator_stop_ready", "j2_j3_support_reliable",
            "clearance_confirmed", "no_person_contact",
            "observer_receipt_monotonic_ns",
        }
        and sample["operator_stop_ready"] is True
        and sample["j2_j3_support_reliable"] is True
        and sample["clearance_confirmed"] is True
        and sample["no_person_contact"] is True
        for sample in confirmation_trace
    )
    assert len(report["final brake"]["paired_trace"]) == 6
    assert run.bundle_sealed and run.shutdown_requested
    assert run.status(now_ns=sample_now)["result"] == "COMPLETE"
    assert "NOT_RUN_EARLY_FUNDAMENTAL_LIMIT" in (
        seal_dir / "thermal_15min.csv"
    ).read_text(encoding="utf-8")
    assert {path.name for path in seal_dir.iterdir()} == set(
        runner_mod.EXACT_BUNDLE_FILES
    )


def test_attestation_interfaces_require_hash_bound_gui_and_multi_joint_pass():
    run = runner_mod.ActiveAcceptanceRunner(binding())
    now, seq, positions = execute_full_position(run)
    now, seq, positions = execute_post_position(
        run,
        "MULTI_JOINT",
        [math.radians(1.0), math.radians(1.0), 0.0, 0.0, 0.0, 0.0],
        now + 1,
        seq,
        positions,
    )
    bound_map = {
        "envelope_id": run.binding.envelope_id,
        "envelope_sha256": run.binding.envelope_sha256,
        "session_id": run.binding.session_id,
        "state_instance_id": run.binding.state_instance_id,
        "anchor_sha256": run.binding.anchor_sha256,
    }
    multi = {
        "schema": "V15.31B-multi-joint-validation-v1",
        "result": "PASS",
        "binding": bound_map,
        "planned_preview": "PASS",
        "preview_before_real": "PASS",
        "planned_twin": "PASS",
        "actual_twin": "PASS",
        "explicit_user_submit": True,
        "actual_twin_source": "REAL_ENCODER",
    }
    run.record_multi_joint_attestation(multi)
    gui_fields = {
        "single_slider_group", "drag_controls_planned_only",
        "planned_mujoco_animation", "actual_twin_encoder_only",
        "preview_before_real", "explicit_real_submit", "execution_status",
        "progress_percent", "estimated_remaining", "heartbeat",
        "temperature_table", "online_status_7_motors", "temperature_colors",
        "position_target_error",
    }
    run.record_gui_attestation({
        "schema": "V15.31B-gui-powered-validation-v1",
        "result": "PASS",
        "binding": bound_map,
        "checks": {field: "PASS" for field in gui_fields},
    })
    assert run.multi_joint_document is None
    assert run.gui_document is not None
    bad = dict(multi)
    bad["start_deg"] = [0.0] * 6
    with pytest.raises(runner_mod.AcceptanceError):
        run.record_multi_joint_attestation(bad)
    wrong_binding = dict(multi)
    wrong_binding["binding"] = {
        **bound_map, "anchor_sha256": "0" * 64,
    }
    with pytest.raises(runner_mod.AcceptanceError):
        run.record_multi_joint_attestation(wrong_binding)


def test_worker_supervisor_identity_change_is_a_fail_closed_session_takeover():
    brakes = []
    run = runner_mod.ActiveAcceptanceRunner(binding(), brake_callback=brakes.append)
    bound = run.binding
    now = 80_000_000_000
    run.observe_hardware_state(state(bound, now, 1), now_ns=now)
    changed = state(bound, now + 1, 2)
    changed["worker_supervisor_pid"] = 5252
    run.observe_hardware_state(changed, now_ns=now + 1)
    assert run.failure.reason == "WORKER_SUPERVISOR_RESTART_OR_SESSION_TAKEOVER"
    assert brakes


def test_state_session_or_instance_change_is_rejected_without_takeover():
    run = runner_mod.ActiveAcceptanceRunner(binding())
    bound = run.binding
    now = 85_000_000_000
    run.observe_hardware_state(state(bound, now, 1), now_ns=now)
    changed = state(bound, now + 1, 2)
    changed["state_instance_id"] = "replacement-state-instance"
    run.observe_hardware_state(changed, now_ns=now + 1)
    assert run.failure.reason == "HARDWARE_STATE_BINDING_MISMATCH"


def test_exact_gui_refresh_is_allowed_but_mutated_refresh_brakes():
    brakes = []
    run = runner_mod.ActiveAcceptanceRunner(binding(), brake_callback=brakes.append)
    bound = run.binding
    now = 87_000_000_000
    assert run.observe_confirmation(confirmation(bound, now), now_ns=now)
    refresh_gravity(run, now)
    run.observe_hardware_state(state(bound, now, 1), now_ns=now)
    run.gravity_ladder_complete = True
    run.comparison_complete = {"WITHOUT_FF", "WITH_FF"}
    run.start_position(now_ns=now)
    for seq in range(2, 8):
        now += 100_000_000
        refresh_gravity(run, now)
        run.observe_hardware_state(state(bound, now, seq), now_ns=now)
    now += 1
    command, _target, sha, _token = position_command(
        bound, run, now, 1, [0.0] * 6
    )
    refresh_gravity(run, now, sha)
    run.observe_gui_command(command, now_ns=now)
    assert run.failure is None
    refresh = {**command, "sequence": 2, "source_monotonic_ns": now + 1}
    run.observe_gui_command(refresh, now_ns=now + 1)
    assert run.failure is None
    assert run.position_trajectory_budget_ns == 2_000_000_000
    mutated = {
        **refresh,
        "source_monotonic_ns": now + 2,
        "trajectory": {**refresh["trajectory"], "duration_ns": 3_000_000_000},
    }
    run.observe_gui_command(mutated, now_ns=now + 2)
    assert run.failure.reason == "POSITION_REFRESH_MUTATED_IMMUTABLE_DESCRIPTOR"
    assert brakes


def test_final_brake_requires_observed_go_brake_and_j6_disabled():
    brakes = []
    run = runner_mod.ActiveAcceptanceRunner(binding(), brake_callback=brakes.append)
    bound = run.binding
    run.position_complete = True
    run.position_started_ns = 90_000_000_000
    run.center_rad = (0.0,) * 6
    run.thermal_summary = {
        "result": "PASS", "classification": "CONTROL_EXCESS_TORQUE_SOLVED",
    }
    run.completed_post_order = [
        "MULTI_JOINT", "THERMAL_SETUP", "RESTORE_INITIAL",
    ]
    run.completed_post_execution["RESTORE_INITIAL"] = {"result": "PASS"}
    run.post_workflow_phase = "AWAIT_FINAL_BRAKE"
    now = 90_000_000_000
    assert run.observe_confirmation(confirmation(bound, now), now_ns=now)
    refresh_gravity(run, now)
    run.observe_hardware_state(state(bound, now, 1), now_ns=now)
    run.request_final_brake(now_ns=now)
    assert brakes and brakes[0].reason == "FINAL_ACCEPTANCE_COMPLETE"
    modes = {motor: "brake" for motor in runner_mod.MOTOR_NAMES}
    for index in range(1, 7):
        sample_now = now + index * 100_000_000
        run.observe_motor_feedback_raw({
            "schema": "go-m8010-motor-feedback/1.0",
            "source_instance_id": "5" * 32,
            "sequence": index,
            "session_id": bound.session_id,
            "state_instance_id": bound.state_instance_id,
            "source_monotonic_ns": sample_now,
            "samples": [{
                "motor": "J6", "communication_ok": True, "merror": 0,
            }],
            "drive_state": 0,
            "controller_mode": "brake",
            "controller_mode_by_motor": {"J6": "brake"},
        }, now_ns=sample_now)
        refresh_gravity(run, sample_now)
        run.observe_hardware_state(
            state(
                bound,
                sample_now,
                index + 1,
                modes=modes,
                j6_raw_identity={
                    "source_instance_id": "5" * 32,
                    "sequence": index,
                    "source_monotonic_ns": sample_now,
                    "session_id": bound.session_id,
                    "state_instance_id": bound.state_instance_id,
                },
            ),
            now_ns=sample_now,
        )
        if index < 6:
            assert not run.final_brake_observed
    assert run.final_brake_observed
    assert run.final_brake_pair_count == 6
    assert run.final_brake_dwell_seconds >= 0.5
    assert runner_mod._valid_sha256(run.final_brake_hardware_state_sha256)
    assert all(
        pair["j6_raw_source_instance_id"] == "5" * 32
        and pair["j6_raw_sequence"] == index
        and pair["j6_raw_session_id"] == bound.session_id
        and pair["j6_raw_state_instance_id"] == bound.state_instance_id
        and pair["position_rad"] == [0.0] * 6
        and max(pair["position_error_from_session_center_deg"]) == 0.0
        for index, pair in enumerate(run.final_brake_pairs, start=1)
    )


def test_final_brake_rejects_cross_session_or_replayed_j6_raw():
    for mutation, expected_reason in (
        ({"session_id": "cross-session"}, "J6_DISABLED_RAW_BINDING_MISMATCH"),
        ({"sequence": 1, "source_monotonic_ns": 90_100_000_000},
         "J6_DISABLED_RAW_SAMPLE_NOT_INCREASING"),
    ):
        run = runner_mod.ActiveAcceptanceRunner(binding())
        bound = run.binding
        run.final_brake_requested = True
        now = 90_100_000_000
        baseline = {
            "schema": "go-m8010-motor-feedback/1.0",
            "source_instance_id": "5" * 32,
            "sequence": 1,
            "session_id": bound.session_id,
            "state_instance_id": bound.state_instance_id,
            "source_monotonic_ns": now,
            "samples": [{
                "motor": "J6", "communication_ok": True, "merror": 0,
            }],
            "drive_state": 0,
            "controller_mode": "brake",
            "controller_mode_by_motor": {"J6": "brake"},
        }
        if expected_reason == "J6_DISABLED_RAW_SAMPLE_NOT_INCREASING":
            run.observe_motor_feedback_raw(baseline, now_ns=now)
            assert run.failure is None
            value = {**baseline, **mutation}
            run.observe_motor_feedback_raw(value, now_ns=now)
        else:
            value = {**baseline, **mutation}
            run.observe_motor_feedback_raw(value, now_ns=now)
        assert run.failure is not None
        assert run.failure.reason == expected_reason


def test_final_brake_requires_exact_aggregated_j6_raw_identity():
    run = runner_mod.ActiveAcceptanceRunner(binding())
    bound = run.binding
    run.center_rad = (0.0,) * 6
    run.final_brake_requested = True
    now = 90_200_000_000
    raw = {
        "schema": "go-m8010-motor-feedback/1.0",
        "source_instance_id": "5" * 32,
        "sequence": 1,
        "session_id": bound.session_id,
        "state_instance_id": bound.state_instance_id,
        "source_monotonic_ns": now,
        "samples": [{
            "motor": "J6", "communication_ok": True, "merror": 0,
        }],
        "drive_state": 0,
        "controller_mode": "brake",
        "controller_mode_by_motor": {"J6": "brake"},
    }
    run.observe_motor_feedback_raw(raw, now_ns=now)
    modes = {motor: "brake" for motor in runner_mod.MOTOR_NAMES}
    wrong_identity = {
        "source_instance_id": "6" * 32,
        "sequence": 1,
        "source_monotonic_ns": now,
        "session_id": bound.session_id,
        "state_instance_id": bound.state_instance_id,
    }
    run.observe_hardware_state(
        state(
            bound,
            now,
            1,
            modes=modes,
            j6_raw_identity=wrong_identity,
        ),
        now_ns=now,
    )
    assert run.failure is not None
    assert (
        run.failure.reason
        == "FINAL_BRAKE_J6_RAW_AGGREGATOR_IDENTITY_MISMATCH"
    )


def test_manifest_seals_exactly_thirteen_files_into_exact_fourteen(tmp_path):
    for filename in runner_mod.HASHED_ARTIFACTS:
        (tmp_path / filename).write_text(f"{filename}\n", encoding="utf-8")
    manifest = runner_mod.finalize_sha256_manifest(tmp_path)
    assert manifest.name == "SHA256SUMS"
    assert {path.name for path in tmp_path.iterdir()} == set(
        runner_mod.EXACT_BUNDLE_FILES
    )
    lines = manifest.read_text(encoding="ascii").splitlines()
    assert len(lines) == 13
    assert [line.split("  ", 1)[1] for line in lines] == list(
        runner_mod.HASHED_ARTIFACTS
    )


def test_git_provenance_is_real_synced_head_and_rejects_dirty_tree(tmp_path):
    repo, output, provenance = synced_repo_with_supporting_evidence(tmp_path)
    assert provenance.branch == runner_mod.EXPECTED_BRANCH
    assert provenance.run_base_commit == git(repo, "rev-parse", "HEAD")
    assert provenance.upstream_commit == provenance.run_base_commit
    assert provenance.ahead_count == provenance.behind_count == 0
    assert set(provenance.allowed_untracked_paths) == {
        (output / name).relative_to(repo).as_posix()
        for name in runner_mod.SUPPORTING_ARTIFACT_NAMES
    }
    (repo / "README.txt").write_text("dirty\n", encoding="utf-8")
    with pytest.raises(
        runner_mod.AcceptanceError,
        match="GIT_PROVENANCE_TRACKED_OR_UNEXPECTED_CHANGE_PRESENT",
    ):
        runner_mod.capture_run_git_provenance(repo, output)


def test_mechanical_recommendation_rejects_arbitrary_tiny_reduction():
    run = runner_mod.ActiveAcceptanceRunner(binding())
    rows = [{
        "j2a_gravity_ff_rotor_nm": -0.2,
        "j2b_gravity_ff_rotor_nm": 0.2,
    }] * 6
    tiny = 1.0e-6
    value = {
        "schema": "V15.31B-mechanical-counterbalance-recommendation-v1",
        "required_j2_torque_reduction_nm": tiny,
        "gas_spring": {
            "force_n": tiny,
            "lever_arm_m": 1.0,
            "estimated_torque_nm": tiny,
        },
    }
    with pytest.raises(
        runner_mod.AcceptanceError,
        match="CLASSIFICATION_B_REDUCTION_NOT_RUNNER_DERIVED_20_PERCENT_UNLOAD",
    ):
        run._validate_mechanical_recommendation(value, rows)

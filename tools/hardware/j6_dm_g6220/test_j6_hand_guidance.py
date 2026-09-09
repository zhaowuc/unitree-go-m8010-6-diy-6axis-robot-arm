from copy import deepcopy
from dataclasses import replace
from datetime import datetime, timezone, timedelta
from pathlib import Path
import ast
import json
import math
import pytest

from j6_hand_guidance import (COMMAND_SCHEMA, GUIDANCE_SCHEMA, DEADMAN_NS,
    GuidanceReferenceRejected, validate_guidance_shape, stage_guidance_candidate,
    runtime_guidance_command, guidance_feedback_velocity_is_safe)


def packet(epoch=2, source=1_000_000_000, mode="teach"):
    return {"schema": COMMAND_SCHEMA, "mode": mode, "sequence": 1,
        "source_instance_id": "a" * 32, "source_monotonic_ns": source,
        "activation_epoch": epoch, "active_joint_mask": [mode != "brake"] * 6,
        "moving_joint_mask": [mode == "teach"] * 6,
        "targets_rad": [0.0] * 6, "feedforward_nm": [0.0] * 6,
        "maximum_velocity_rad_s": math.radians(30), "maximum_acceleration_rad_s2": math.radians(10),
        "hand_guidance": {"schema": GUIDANCE_SCHEMA, "origin_rad": [0.0] * 6,
            "velocity_rad_s": [0.0] * 6, "maximum_velocity_deg_s": 30.0,
            "maximum_excursion_deg": 10.0, "maximum_reference_error_deg": 2.0,
            "freeze_reference": False}}


def test_j6_guidance_bounded_references_deadman_frozen_hold_and_exact_owned_ack():
    from test_v15_30a_gui_profile import load_parse_command, load_mode_functions
    parse = load_parse_command()
    parse.__globals__["validate_guidance_shape"] = validate_guidance_shape
    context = {"torque_qualified": True, "healthy_foc": True, "actual_rad": 0.0,
               "actual_velocity_rad_s": 0.0}
    previous = packet(epoch=1, source=990_000_000, mode="hold")
    first = parse(json.dumps(packet()).encode(), 1_000_000_001)
    assert first["maximum_velocity_rad_s"] == math.radians(30)
    accepted, state = stage_guidance_candidate(first, None, context, previous, 1_000_000_001)
    # Admission cancellation is the same fixed-target HOLD whether this
    # domain accepted the initial teach frame or is still on ordinary HOLD.
    ordinary_hold = deepcopy(previous)
    ordinary_hold["schema"] = "go-m8010-gui-command/1.2"
    ordinary_hold.pop("hand_guidance")
    abort = packet(epoch=3, source=1_005_000_000, mode="hold")
    for admission_state, prior_command in ((None, ordinary_hold), (state, accepted)):
        _, aborted = stage_guidance_candidate(abort, admission_state, context, prior_command, 1_005_000_001)
        assert not aborted.active and aborted.targets == tuple(ordinary_hold["targets_rad"])
        assert aborted.accepted_epoch == 3
        bad_abort = deepcopy(abort)
        bad_abort["targets_rad"][5] += .001  # Still within 2 deg, but never the held target.
        with pytest.raises(ValueError, match="HOLD"):
            stage_guidance_candidate(bad_abort, admission_state, context, prior_command, 1_005_000_001)
    with pytest.raises(GuidanceReferenceRejected, match="PRIOR_EXACT_HOLD"):
        stage_guidance_candidate(abort, None, context, None, 1_005_000_001)
    displaced = dict(context, actual_rad=math.radians(2.1))
    _, retained_abort = stage_guidance_candidate(abort, None, displaced, ordinary_hold, 1_005_000_001)
    assert retained_abort.targets == tuple(ordinary_hold["targets_rad"]) and not retained_abort.active
    # Cancel never moves H0 to the displaced actual position or shifts origin.
    for field in ("targets_rad", "origin_rad"):
        changed = deepcopy(abort)
        (changed if field == "targets_rad" else changed["hand_guidance"])[field][5] = displaced["actual_rad"]
        with pytest.raises(GuidanceReferenceRejected):
            stage_guidance_candidate(changed, None, displaced, ordinary_hold, 1_005_000_001)
    for invalid_context, reason in ((dict(displaced, healthy_foc=False), "HEALTHY"),
            (dict(displaced, torque_qualified=False), "QUALIFIED"),
            (dict(displaced, actual_velocity_rad_s=math.radians(30.1)), "FEEDBACK_VELOCITY"),
            (dict(displaced, actual_velocity_rad_s=math.nan), "FEEDBACK_VELOCITY")):
        with pytest.raises(ValueError, match=reason):
            stage_guidance_candidate(abort, None, invalid_context, ordinary_hold, 1_005_000_001)
    with pytest.raises(GuidanceReferenceRejected, match="ACTUAL_ERROR"):
        stage_guidance_candidate(first, None, displaced, ordinary_hold, 1_005_000_001)
    wrong_first_origin = deepcopy(first)
    wrong_first_origin["hand_guidance"]["origin_rad"][5] = .001
    with pytest.raises(GuidanceReferenceRejected, match="FIRST_ORIGIN"):
        stage_guidance_candidate(wrong_first_origin, None, context, ordinary_hold, 1_005_000_001)
    changed_origin = deepcopy(abort)
    changed_origin["hand_guidance"]["origin_rad"][5] = .001
    with pytest.raises(GuidanceReferenceRejected, match="SESSION_ORIGIN"):
        stage_guidance_candidate(changed_origin, aborted, context, abort, 1_005_000_001)
    changed_origin["mode"] = "teach"
    changed_origin["moving_joint_mask"] = [True]*6
    changed_origin["activation_epoch"] = 4
    with pytest.raises(GuidanceReferenceRejected, match="SESSION_ORIGIN"):
        stage_guidance_candidate(changed_origin, aborted, context, abort, 1_005_000_001)
    wrong_units = packet(); wrong_units["hand_guidance"]["maximum_velocity_deg_s"] = math.radians(30)
    with pytest.raises(ValueError, match="UNITS"):
        parse(json.dumps(wrong_units).encode(), 1_000_000_001)
    with pytest.raises(ValueError, match="QUALIFIED"):
        stage_guidance_candidate(first, None, dict(context, torque_qualified=False), previous, 1_000_000_001)
    second = packet(source=1_010_000_000); second["targets_rad"][5] = .002
    second = parse(json.dumps(second).encode(), 1_010_000_001)
    accepted, state = stage_guidance_candidate(second, state, context, accepted, 1_010_000_001)
    release = packet(source=1_020_000_000)
    release["hand_guidance"]["freeze_reference"] = True
    release["targets_rad"][5] = 1.0  # Freeze never accepts a new reference from this packet.
    frozen_command, released = stage_guidance_candidate(release, state, context, accepted, 1_020_000_001)
    assert released.paused_reason == "GUI_RELEASE" and released.targets == state.targets
    assert frozen_command["targets_rad"] == list(state.targets)
    queued = packet(source=1_030_000_000)
    queued["targets_rad"][5] = .03
    queued["hand_guidance"]["velocity_rad_s"][5] = .1
    frozen_command, queued_state = stage_guidance_candidate(queued, released, context, frozen_command, 1_030_000_001)
    assert queued_state.targets == state.targets and queued_state.started_ns == state.started_ns
    assert queued_state.paused_reason == "GUI_RELEASE" and frozen_command["hand_guidance"]["velocity_rad_s"] == [0.0]*6
    _, later = runtime_guidance_command(frozen_command, queued_state, state.started_ns + DEADMAN_NS)
    assert later.paused_reason == "GUI_RELEASE"
    release_ack = packet(epoch=3, source=1_040_000_000, mode="hold")
    release_ack["targets_rad"] = list(state.targets)
    _, release_idle = stage_guidance_candidate(release_ack, queued_state, context, frozen_command, 1_040_000_001)
    assert not release_idle.active and not release_idle.paused and release_idle.accepted_epoch == 3
    with pytest.raises(ValueError, match="FIRST_REFERENCE"):
        stage_guidance_candidate(release, None, context, previous, 1_020_000_001)
    for invalid in (None, 0, 1):
        invalid_freeze = deepcopy(release)
        invalid_freeze["hand_guidance"]["freeze_reference"] = invalid
        with pytest.raises(ValueError, match="FREEZE_INVALID"):
            validate_guidance_shape(invalid_freeze)
    invalid_freeze = deepcopy(release)
    invalid_freeze["hand_guidance"]["velocity_rad_s"][0] = .001
    with pytest.raises(ValueError, match="FREEZE_VELOCITY"):
        validate_guidance_shape(invalid_freeze)
    unchanged = state
    fast = packet(source=1_020_000_000); fast["targets_rad"][5] = .02
    with pytest.raises(GuidanceReferenceRejected, match="STEP"):
        stage_guidance_candidate(fast, state, context, accepted, 1_020_000_001)
    far = packet(source=1_210_000_000); far["targets_rad"][5] = .05
    with pytest.raises(GuidanceReferenceRejected, match="ACTUAL"):
        stage_guidance_candidate(far, state, context, accepted, 1_210_000_001)
    outside = packet(source=1_410_000_000); outside["targets_rad"][5] = math.radians(10.1)
    with pytest.raises(GuidanceReferenceRejected, match="EXCURSION"):
        stage_guidance_candidate(outside, state, dict(context, actual_rad=outside['targets_rad'][5]), accepted, 1_410_000_001)
    assert state == unchanged
    frozen = packet(source=1_220_000_000)
    frozen['targets_rad'][5] = state.targets[5]
    displaced_context = dict(context, actual_rad=.10, actual_velocity_rad_s=0.0)
    renewed, retained = stage_guidance_candidate(frozen, state, displaced_context, accepted, 1_220_000_001)
    assert retained.targets[5] == state.targets[5] and retained.started_ns == state.started_ns
    assert retained.source_ns == frozen['source_monotonic_ns']
    moving_at_same_target = deepcopy(frozen)
    moving_at_same_target['hand_guidance']['velocity_rad_s'][5] = .01
    with pytest.raises(GuidanceReferenceRejected, match='ACTUAL'):
        stage_guidance_candidate(moving_at_same_target, state, displaced_context, accepted, 1_220_000_001)
    with pytest.raises(ValueError, match='FEEDBACK_VELOCITY'):
        stage_guidance_candidate(frozen, state, dict(displaced_context, actual_velocity_rad_s=math.radians(30.1)), accepted, 1_220_000_001)
    deadline = state.started_ns + DEADMAN_NS
    paused_command, paused = runtime_guidance_command(accepted, state, deadline)
    assert paused.paused and paused.active and paused.targets == state.targets and paused.paused_reason == "DEADMAN_TIMEOUT"
    assert paused_command["received_at"] == accepted["received_at"]  # no invented lease refresh
    functions = load_mode_functions()
    assert functions["effective_command_mode"](paused_command, 0, paused_command["received_at"]) == "hold"
    assert not functions["command_lease_is_fresh"](paused_command, paused_command["received_at"] + .501)
    heartbeat = packet(source=deadline + 10_000_000); heartbeat["targets_rad"][5] = 1.0
    normalized, after = stage_guidance_candidate(heartbeat, paused, context, accepted, deadline + 10_000_001)
    assert normalized["targets_rad"][5] == state.targets[5]
    assert after.started_ns == state.started_ns and after.paused
    assert normalized["hand_guidance"]["velocity_rad_s"] == [0.0] * 6
    ack = packet(epoch=3, source=deadline + 20_000_000, mode="hold")
    ack["targets_rad"] = list(state.targets)
    ack["targets_rad"][0] = .03  # Other domains may have accepted a different final frame.
    wrong_ack = deepcopy(ack); wrong_ack["targets_rad"][5] += .001
    with pytest.raises(ValueError, match="ACK"):
        stage_guidance_candidate(wrong_ack, after, context, normalized, deadline + 20_000_001)
    ack, idle = stage_guidance_candidate(ack, after, context, normalized, deadline + 20_000_001)
    assert not idle.active and not idle.paused and idle.accepted_epoch == 3
    assert idle.targets[5] == state.targets[5]
    restart = packet(epoch=4, source=deadline + 30_000_000)
    restart["targets_rad"] = list(idle.targets)
    _, restarted = stage_guidance_candidate(restart, idle, context, ack, deadline + 30_000_001)
    assert restarted.active and restarted.started_ns == deadline + 30_000_001
    # Repeat after a real previous HOLD has moved beyond two degrees from
    # the unchanged session anchor. Only entry target-to-actual uses 2 deg.
    offset_ack = deepcopy(ack)
    offset_ack["targets_rad"][5] = math.radians(3.0)
    offset_idle = replace(idle, targets=tuple(offset_ack["targets_rad"]))
    offset_press = deepcopy(restart)
    offset_press["targets_rad"] = list(offset_idle.targets)
    offset_context = dict(context, actual_rad=math.radians(3.0))
    _, repeated = stage_guidance_candidate(offset_press, offset_idle, offset_context, offset_ack, deadline + 30_000_001)
    assert repeated.active and repeated.origin == state.origin
    assert repeated.targets == offset_idle.targets and repeated.started_ns == deadline + 30_000_001
    with pytest.raises(GuidanceReferenceRejected, match="FIRST_ORIGIN"):
        stage_guidance_candidate(offset_press, None, offset_context, offset_ack, deadline + 30_000_001)
    changed_entry = deepcopy(offset_press)
    changed_entry["targets_rad"][5] += .001
    with pytest.raises(GuidanceReferenceRejected, match="ENTRY_REQUIRES_EXACT_HOLD"):
        stage_guidance_candidate(changed_entry, offset_idle, offset_context, offset_ack, deadline + 30_000_001)
    moving_entry = deepcopy(offset_press)
    moving_entry["hand_guidance"]["velocity_rad_s"][0] = .001
    with pytest.raises(GuidanceReferenceRejected, match="ZERO_VELOCITY"):
        stage_guidance_candidate(moving_entry, offset_idle, offset_context, offset_ack, deadline + 30_000_001)
    with pytest.raises(GuidanceReferenceRejected, match="ACTUAL_ERROR"):
        stage_guidance_candidate(offset_press, offset_idle, dict(context, actual_rad=0.0), offset_ack, deadline + 30_000_001)
    assert not offset_idle.active and offset_idle.targets == tuple(offset_ack["targets_rad"])
    brake = packet(mode="brake"); brake["hand_guidance"] = "broken metadata cannot prevent brake"
    parsed_brake = parse(json.dumps(brake).encode(), 1_000_000_001)
    _, cleared = stage_guidance_candidate(parsed_brake, state, None, accepted, deadline)
    assert cleared is None
    assert guidance_feedback_velocity_is_safe(math.radians(30))
    assert not guidance_feedback_velocity_is_safe(math.radians(30.1))
    assert not guidance_feedback_velocity_is_safe(math.nan)


def test_real_j6_authority_accepts_guided_27_for_teach_and_hold_only_after_ladder():
    from test_v15_30a_gui_profile import load_command_channel_functions
    namespace = load_command_channel_functions()
    namespace.update(datetime=datetime, timezone=timezone,
        gravity_authority_maximum_age_ns=lambda *_: 250_000_000,
        enforce_gravity_authority_startup_binding=lambda *_: None)
    source = Path(__file__).with_name("v15_30a_gui_j6_controller.py")
    node = next(n for n in ast.parse(source.read_text(encoding="utf-8")).body
                if isinstance(n, ast.FunctionDef) and n.name == "validate_empirical_command_authority")
    exec(compile(ast.fix_missing_locations(ast.Module(body=[node], type_ignores=[])), str(source), "exec"), namespace)
    validate = namespace["validate_empirical_command_authority"]
    command = packet()
    command["gravity_authority"] = {
        "schema": "go-m8010-gravity-command-authority/1.1", "source_instance_id": "b" * 32,
        "sequence": 1, "source_monotonic_ns": 1_000_000_000,
        "model_sha256": "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9",
        "gravity_config_sha256": "307469b8384fd35547327ba1d5f80aa440e6b9663406ab7c9d9469bea263335d",
        "session_id": "session", "state_instance_id": "c" * 32,
        "gravity_scale": 1.0, "gravity_scale_target": 1.0, "feedforward_nm": [0.0] * 6,
        "authority_class": "EMPIRICAL_VALIDATION_ENVELOPE", "rating_classification": "NOT_OFFICIAL_CONTINUOUS_RATING",
        "empirical_envelope_id": "v15-31b-empirical-01234567890123456789",
        "empirical_envelope_sha256": "d" * 64, "anchor_sha256": "e" * 64,
        "empirical_envelope_expires_at_utc": (datetime.now(timezone.utc) + timedelta(hours=1)).isoformat().replace('+00:00','Z'),
        "empirical_envelope_deadline_monotonic_ns": 601_000_000_000,
        "empirical_stage_index": 4, "empirical_position_validation_authorized": True,
        "empirical_maximum_position_segment_seconds": 15.0, "empirical_maximum_abs_position_segment_deg": 5.0,
        "empirical_assisted_teach_authorized": True, "empirical_maximum_teach_excursion_deg": 10.0,
        "empirical_maximum_teach_seconds": 600.0, "empirical_maximum_teach_velocity_deg_s": 30.0,
        "empirical_allowed_teach_joints": [f'J{i}' for i in range(1,7)],
    }
    binding = (command['gravity_authority']['empirical_envelope_id'], 'd'*64, 'e'*64, 'session', 'c'*32, 4)
    state = {'empirical_authority_binding': binding}
    assert validate(command, 1_000_000_001, state) == binding
    command['mode'] = 'hold'; command['moving_joint_mask'] = [False]*6
    assert validate(command, 1_000_000_001, state) == binding
    with pytest.raises(ValueError, match='stage zero'):
        validate(command, 1_000_000_001, {})
    wrong = deepcopy(command); wrong['gravity_authority']['empirical_maximum_teach_seconds'] = 30.0
    with pytest.raises(Exception, match='SCOPE'):
        validate(wrong, 1_000_000_001, state)
    legacy = deepcopy(command); legacy['schema'] = 'go-m8010-gui-command/1.2'
    with pytest.raises(Exception, match='incomplete'):
        validate(legacy, 1_000_000_001, state)

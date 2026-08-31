import ast
import hashlib
import importlib.util
import io
import json
import math
import os
import re
import stat
import sys
import tempfile
import time
from contextlib import redirect_stdout
from pathlib import Path
from types import SimpleNamespace
from datetime import datetime, timezone


MODULE_DIRECTORY = Path(__file__).resolve().parent
CONTROLLER = MODULE_DIRECTORY / "v15_30a_gui_j6_controller.py"
STOP_TOOL = MODULE_DIRECTORY.parent / "v15_30a_gui_stop.py"
sys.path.insert(0, str(MODULE_DIRECTORY))
SPEC = importlib.util.spec_from_file_location(
    "v15_30a_profile", MODULE_DIRECTORY / "v15_30a_profile.py"
)
assert SPEC is not None and SPEC.loader is not None
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def load_feedback_handoff_functions():
    tree = ast.parse(CONTROLLER.read_text(encoding="utf-8"))
    names = {
        "valid_lower_sha256",
        "_load_json_without_duplicate_keys",
        "_secure_regular_file_bytes",
        "_fsync_directory",
        "claim_feedback_handoff",
        "consume_feedback_handoff",
    }
    functions = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {
        "Any": object,
        "ExpectedFeedbackIdentityBinding": object,
        "Path": Path,
        "hashlib": hashlib,
        "json": json,
        "os": os,
        "re": re,
        "stat": stat,
        "time": time,
        "FEEDBACK_HANDOFF_SCHEMA": "go-m8010-j6-feedback-handoff/1.0",
        "RAW_CAPTURE_SCHEMA": (
            "go-m8010-j6-disabled-raw-capture-statistics/1.0"
        ),
        "MAXIMUM_FEEDBACK_SEQUENCE": (1 << 63) - 1,
        "MAXIMUM_HANDOFF_BYTES": 64 * 1024,
        "MAXIMUM_RAW_CAPTURE_BYTES": 16 * 1024 * 1024,
    }
    exec(
        compile(
            ast.fix_missing_locations(ast.Module(body=functions, type_ignores=[])),
            str(CONTROLLER),
            "exec",
        ),
        namespace,
    )
    return namespace


def load_thermal_config_function():
    tree = ast.parse(CONTROLLER.read_text(encoding="utf-8"))
    function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "load_thermal_limits"
    )
    namespace = {
        "Path": Path,
        "hashlib": __import__("hashlib"),
        "math": math,
        "THERMAL_CONFIG_SHA256": (
            "1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467"
        ),
        "J6_FEEDBACK_SOURCE_INSTANCE_ID": "9" * 32,
        "J6_FEEDBACK_SEQUENCE": 0,
        "EXPECTED_GRAVITY_AUTHORITY_BINDING": SimpleNamespace(
            session_id=(
                "persistent:0123456789abcdef:"
                "j2session:1111111111111111:"
                "goauxsession:2222222222222222"
            ),
            state_instance_id="8" * 32,
        ),
    }
    exec(
        compile(
            ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])),
            str(CONTROLLER),
            "exec",
        ),
        namespace,
    )
    return namespace["load_thermal_limits"]


def load_parse_command():
    tree = ast.parse(CONTROLLER.read_text(encoding="utf-8"))
    names = {"valid_lower_sha256", "parse_command"}
    functions = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {
        "json": json,
        "math": math,
        "time": __import__("time"),
        "MODEL_COMMAND_LOWER": -math.pi,
        "MODEL_COMMAND_UPPER": math.pi,
        "VMAX_LIMIT": math.radians(5.0),
        "AMAX_LIMIT": math.radians(20.0),
        "COMMAND_SOURCE_MAX_AGE_NS": 250_000_000,
        "QUINTIC_COMMAND_SCHEMA": "go-m8010-quintic-command/1.0",
        "QUINTIC_PROFILE": "quintic-rest-to-rest-v1",
        "QUINTIC_MAX_INTERVALS": 1_000_000,
        "QUINTIC_MAX_SAMPLE_PERIOD_NS": 10_000_000,
    }
    exec(
        compile(ast.fix_missing_locations(ast.Module(body=functions, type_ignores=[])),
                str(CONTROLLER), "exec"),
        namespace,
    )
    return namespace["parse_command"]


def load_mode_functions():
    tree = ast.parse(CONTROLLER.read_text(encoding="utf-8"))
    names = {
        "accept_hold_target",
        "capture_lease_safe_hold_target",
        "command_source_for_lease_capture",
        "command_lease_is_fresh",
        "command_rejection_reason",
        "command_requests_j6_active",
        "command_requests_j6_fixed_hold",
        "command_uses_empirical_gravity_authority",
        "command_epoch_is_acceptable",
        "command_epoch_was_rejected",
        "enabled_feedback_is_healthy",
        "effective_command_mode",
        "external_command_can_resume_from_safe_hold",
        "external_command_can_safely_resume_from_safe_hold",
        "external_command_explicitly_releases_safe_hold",
        "fault_dominant_mode",
        "fixed_hold_profile_state",
        "fixed_hold_velocity_limit",
        "flush_command_rejection_reports",
        "hold_command_entry_is_safe",
        "hold_command_entry_is_authorized",
        "hold_transition_reuses_position_target",
        "rejected_hold_fallback",
        "make_command_rejection_state",
        "minimum_epoch_after_interarrival_lease",
        "next_prior_external_hold_confirmation",
        "observe_valid_command_epoch",
        "position_command_starts_new_profile",
        "position_command_target_is_authorized",
        "record_command_rejection",
        "rejected_takeover_epoch_at_lease_boundary",
        "safe_pre_enable_mode",
        "update_active_deadline_miss_count",
    }
    functions = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {
        "json": json,
        "math": math,
        "time": __import__("time"),
        "LEASE_S": 0.5,
        "REJECTION_LOG_INTERVAL_S": 5.0,
        "FEEDBACK_MAX_AGE_S": 0.15,
        "FEEDBACK_HARD_LOWER": -math.pi - math.radians(0.5),
        "FEEDBACK_HARD_UPPER": math.pi + math.radians(0.5),
        "HOLD_ENTRY_TARGET_LIMIT": math.radians(5.0),
        "EMPIRICAL_INITIAL_HOLD_ENTRY_TARGET_LIMIT": math.radians(0.25),
        "RESTORE_VELOCITY_LIMIT": math.radians(1.0),
        "ACTIVE_DEADLINE_CYCLE_LIMIT_S": 0.02,
        "FAULT_STATES": {8, 9, 0xA, 0xB, 0xC, 0xD, 0xE},
        "ACTIVE_THERMAL_LIMITS": {"thermal_stop_c": 60.0},
    }
    exec(
        compile(ast.fix_missing_locations(ast.Module(body=functions, type_ignores=[])),
                str(CONTROLLER), "exec"),
        namespace,
    )
    return namespace


def load_empirical_authority_functions():
    tree = ast.parse(CONTROLLER.read_text(encoding="utf-8"))
    names = {
        "command_uses_empirical_gravity_authority",
        "empirical_gravity_authority_is_current",
    }
    functions = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {
        "time": time,
        "datetime": datetime,
        "timezone": timezone,
    }
    exec(
        compile(
            ast.fix_missing_locations(ast.Module(body=functions, type_ignores=[])),
            str(CONTROLLER),
            "exec",
        ),
        namespace,
    )
    return namespace


def load_gravity_startup_binding_functions():
    tree = ast.parse(CONTROLLER.read_text(encoding="utf-8"))
    names = {
        "valid_lower_sha256",
        "validate_gravity_authority_startup_binding",
        "validate_feedback_startup_binding",
        "enforce_gravity_authority_startup_binding",
    }
    definitions = [
        node for node in tree.body
        if (
            isinstance(node, ast.ClassDef)
            and node.name in {
                "ExpectedGravityAuthorityBinding",
                "ExpectedFeedbackIdentityBinding",
                "GravityAuthorityStartupBindingError",
            }
        )
        or (
            isinstance(node, ast.FunctionDef)
            and node.name in names
        )
    ]
    namespace = {
        "NamedTuple": __import__("typing").NamedTuple,
        "argparse": __import__("argparse"),
        "re": __import__("re"),
    }
    exec(
        compile(
            ast.fix_missing_locations(
                ast.Module(body=definitions, type_ignores=[])
            ),
            str(CONTROLLER),
            "exec",
        ),
        namespace,
    )
    return namespace


def load_command_channel_functions():
    tree = ast.parse(CONTROLLER.read_text(encoding="utf-8"))
    names = {
        "parse_command",
        "command_lease_is_fresh",
        "command_rejection_reason",
        "command_requests_j6_active",
        "emit_command_rejection_reports",
        "flush_command_rejection_reports",
        "make_command_rejection_state",
        "make_command_source_replay_state",
        "make_command_receive_events",
        "command_source_takeover_is_blocked",
        "command_source_takeover_epoch_is_fresh",
        "command_source_is_newer",
        "commit_command_source",
        "commit_empirical_command_authority",
        "spend_active_empirical_command_authority",
        "commit_position_execution_binding",
        "observe_valid_command_epoch",
        "command_epoch_is_acceptable",
        "minimum_epoch_after_interarrival_lease",
        "record_command_rejection",
        "receive_latest",
        "valid_lower_sha256",
        "command_is_v13_quintic_position",
        "position_execution_contract",
        "validate_position_execution_transition",
    }
    functions = [
        node for node in tree.body
        if (
            isinstance(node, ast.FunctionDef)
            and node.name in names
        )
        or (
            isinstance(node, ast.ClassDef)
            and node.name == "GravityAuthorityStartupBindingError"
        )
    ]
    namespace = {
        "json": json,
        "math": math,
        "socket": __import__("socket"),
        "time": __import__("time"),
        "LEASE_S": 0.5,
        "REJECTION_LOG_INTERVAL_S": 5.0,
        "COMMAND_PACKET_BUDGET": 128,
        "COMMAND_SOURCE_MAX_AGE_NS": 250_000_000,
        "COMMAND_SOURCE_TAKEOVER_LEASE_NS": 500_000_000,
        "COMMAND_SOURCE_REPLAY_LIMIT": 32,
        "MODEL_COMMAND_LOWER": -math.pi,
        "MODEL_COMMAND_UPPER": math.pi,
        "VMAX_LIMIT": math.radians(5.0),
        "AMAX_LIMIT": math.radians(20.0),
        "QUINTIC_COMMAND_SCHEMA": "go-m8010-quintic-command/1.0",
        "QUINTIC_PROFILE": "quintic-rest-to-rest-v1",
        "QUINTIC_MAX_INTERVALS": 1_000_000,
        "QUINTIC_MAX_SAMPLE_PERIOD_NS": 10_000_000,
    }
    exec(
        compile(ast.fix_missing_locations(ast.Module(body=functions, type_ignores=[])),
                str(CONTROLLER), "exec"),
        namespace,
    )
    return namespace


def load_interlock_functions():
    tree = ast.parse(CONTROLLER.read_text(encoding="utf-8"))
    names = {
        "saturating_next_activation_epoch",
        "make_thermal_interlock_state",
        "reset_thermal_cooldown_evidence",
        "latch_thermal_interlock",
        "observe_raw_temperature_thermal_trip",
        "observe_thermal_cooldown_frame",
        "make_no_progress_watchdog_state",
        "reset_no_progress_observation",
        "observe_no_progress_watchdog",
        "observe_explicit_interlock_release",
        "request_interlock_rearm_for_next_cycle",
        "apply_pending_interlock_rearm_at_cycle_start",
        "temperature_window_statistics",
        "command_requests_j6_active",
    }
    functions = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {
        "math": math,
        "statistics": __import__("statistics"),
        "ARRIVAL_TOLERANCE": math.radians(0.08),
        "TARGET_TIMEOUT_S": 90.0,
        "NO_PROGRESS_MINIMUM_IMPROVEMENT": math.radians(0.05),
        "NO_PROGRESS_MINIMUM_QUALIFYING_FRAMES": 3,
    }
    exec(
        compile(ast.fix_missing_locations(ast.Module(body=functions, type_ignores=[])),
                str(CONTROLLER), "exec"),
        namespace,
    )
    return namespace


def load_quintic_controller_functions():
    tree = ast.parse(CONTROLLER.read_text(encoding="utf-8"))
    names = {
        "command_is_v13_quintic_position",
        "v13_position_posvel_reference",
        "trajectory_feedback_status",
    }
    functions = [
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    namespace = {
        "quintic_reference_at": MODULE.quintic_reference_at,
        "quintic_posvel_speed_limit": MODULE.quintic_posvel_speed_limit,
        "RESTORE_VELOCITY_LIMIT": math.radians(1.0),
    }
    exec(
        compile(ast.fix_missing_locations(ast.Module(body=functions, type_ignores=[])),
                str(CONTROLLER), "exec"),
        namespace,
    )
    return namespace


def load_send_feedback():
    tree = ast.parse(CONTROLLER.read_text(encoding="utf-8"))
    function = next(
        node for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == "send_feedback"
    )
    namespace = {
        "json": json,
        "socket": __import__("socket"),
        "time": __import__("time"),
        "FAULT_STATES": {8, 9, 0xA, 0xB, 0xC, 0xD, 0xE},
        "NO_PROGRESS_WATCHDOG_AUTHORITY": (
            "J6_TARGET_TIMEOUT_POSITION_ERROR_V1"
        ),
        "THERMAL_CONFIG_SHA256": (
            "1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467"
        ),
        "J6_FEEDBACK_SOURCE_INSTANCE_ID": "9" * 32,
        "J6_FEEDBACK_SEQUENCE": 0,
        "EXPECTED_GRAVITY_AUTHORITY_BINDING": SimpleNamespace(
            session_id=(
                "persistent:0123456789abcdef:"
                "j2session:1111111111111111:"
                "goauxsession:2222222222222222"
            ),
            state_instance_id="8" * 32,
        ),
        "EXPECTED_FEEDBACK_IDENTITY_BINDING": SimpleNamespace(
            session_id=(
                "persistent:0123456789abcdef:"
                "j2session:1111111111111111:"
                "goauxsession:2222222222222222"
            ),
            state_instance_id="8" * 32,
        ),
    }
    exec(
        compile(ast.fix_missing_locations(ast.Module(body=[function], type_ignores=[])),
                str(CONTROLLER), "exec"),
        namespace,
    )
    return namespace["send_feedback"]


def command_payload(
    schema="go-m8010-gui-command/1.2",
    mask=None,
    *,
    source_instance_id="0123456789abcdef0123456789abcdef",
    source_monotonic_ns=None,
    sequence=1,
):
    if source_monotonic_ns is None:
        source_monotonic_ns = time.monotonic_ns()
    return json.dumps({
        "schema": schema,
        "mode": "position",
        "targets_rad": [0.0] * 6,
        "active_joint_mask": [False] * 5 + [True] if mask is None else mask,
        "moving_joint_mask": [False] * 5 + [True],
        "activation_epoch": 1,
        "maximum_velocity_rad_s": math.radians(5.0),
        "maximum_acceleration_rad_s2": math.radians(20.0),
        "source_instance_id": source_instance_id,
        "source_monotonic_ns": source_monotonic_ns,
        "sequence": sequence,
    }).encode()


def quintic_command_document(
    *,
    received_monotonic_ns=None,
    execute_at_monotonic_ns=None,
    sequence=1,
    activation_epoch=7,
    start_rad=0.0,
    target_rad=0.01,
):
    if received_monotonic_ns is None:
        received_monotonic_ns = time.monotonic_ns()
    if execute_at_monotonic_ns is None:
        execute_at_monotonic_ns = received_monotonic_ns + 100_000_000
    document = json.loads(command_payload(
        schema="go-m8010-gui-command/1.3",
        source_monotonic_ns=received_monotonic_ns,
        sequence=sequence,
    ))
    document["activation_epoch"] = activation_epoch
    document["targets_rad"][5] = target_rad
    document["plan_token_id"] = "a" * 64
    start = [0.0] * 6
    target = [0.0] * 6
    start[5] = start_rad
    target[5] = target_rad
    document["trajectory"] = {
        "schema": "go-m8010-quintic-command/1.0",
        "trajectory_sha256": "b" * 64,
        "profile": "quintic-rest-to-rest-v1",
        "start_rad": start,
        "target_rad": target,
        "duration_ns": 1_000_000_000,
        "interval_count": 100,
        "execute_at_monotonic_ns": execute_at_monotonic_ns,
        "segment_index": 0,
        "segment_count": 1,
    }
    return document


def test_profile_converges_without_velocity_or_acceleration_jump():
    q_command = 0.0
    dq_command = 0.0
    vmax = math.radians(5.0)
    amax = math.radians(20.0)
    target = math.radians(10.0)
    reached_exactly = False
    for _ in range(3000):
        previous_velocity = dq_command
        q_command, dq_command = MODULE.update_profile(
            q_command, dq_command, target, vmax, amax, 0.01
        )
        assert abs(dq_command) <= vmax + 1e-12
        assert abs(dq_command - previous_velocity) <= amax * 0.01 + 1e-12
        if q_command == target and dq_command == 0.0:
            reached_exactly = True
            break
    assert reached_exactly


def test_j6_thermal_config_has_fixed_hash_and_no_external_yaml_dependency():
    thermal_config = (
        CONTROLLER.resolve().parents[3]
        / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
        / "ros2_ws"
        / "src"
        / "go_m8010_arm_hardware"
        / "config"
        / "thermal_limits.yaml"
    )
    limits = load_thermal_config_function()(thermal_config)
    assert limits["normal_below_c"] == 45.0
    assert limits["warning_below_c"] == 50.0
    assert limits["derating_start_c"] == 55.0
    assert limits["thermal_stop_c"] == 60.0
    assert limits["rearm_below_c"] == 55.0
    assert limits["cooldown_seconds"] == 30.0
    assert limits["slope_window_seconds"] == 120.0
    source = CONTROLLER.read_text(encoding="utf-8")
    assert "import yaml" not in source
    assert "yaml.safe_load" not in source


def test_j6_thermal_derating_scales_speed_acceleration_and_hold_authority():
    factor = MODULE.thermal_derating_factor
    limits = MODULE.thermal_derated_posvel_limits
    assert factor(54.999, 55.0, 60.0) == 1.0
    assert factor(55.0, 55.0, 60.0) == 1.0
    assert math.isclose(factor(57.5, 55.0, 60.0), 0.5)
    assert math.isclose(factor(59.0, 55.0, 60.0), 0.2)
    assert math.isclose(factor(59.9, 55.0, 60.0), 0.1)
    assert factor(60.0, 55.0, 60.0) == 0.0
    assert limits(1.0, 2.0, 0.2, 0.5) == (0.5, 1.0, 0.1)
    assert limits(1.0, 2.0, 0.2, 0.0) == (0.0, 0.0, 0.0)

    source = CONTROLLER.read_text(encoding="utf-8")
    assert "J6_V13_THERMAL_DERATING_TRAJECTORY_ABORT" in source
    assert '"EXACT_TRAJECTORY_DERATING_ABORT"' in source
    assert "speed_limit *= thermal_factor" not in source
    assert "thermal_derated_posvel_limits(" in source
    assert "fixed_hold_velocity_limit() * thermal_factor" in source


def test_j6_single_60c_frame_latches_until_cooldown_release_and_higher_epoch():
    functions = load_interlock_functions()
    state = functions["make_thermal_interlock_state"]()
    trip = functions["observe_raw_temperature_thermal_trip"]
    cooldown = functions["observe_thermal_cooldown_frame"]
    release = functions["observe_explicit_interlock_release"]
    request = functions["request_interlock_rearm_for_next_cycle"]
    apply = functions["apply_pending_interlock_rearm_at_cycle_start"]

    assert trip(state, 60.0, 60.0, 7, 0, 0)
    assert state["fault_latched"] is True
    assert state["minimum_rearm_epoch"] == 8
    assert not release(state, True, cooldown_required=True)
    assert not cooldown(state, True, 0.0, 30.0, 3)
    assert not cooldown(state, True, 15.0, 30.0, 3)
    assert cooldown(state, True, 30.0, 30.0, 3)
    assert state["cooldown_ready"] is True
    assert release(state, True, cooldown_required=True)

    stale_epoch = {
        "mode": "position",
        "active_joint_mask": [False] * 5 + [True],
        "activation_epoch": 7,
    }
    higher_epoch = dict(stale_epoch, activation_epoch=8)
    assert not request(
        state, stale_epoch, True, True, 0, cooldown_required=True
    )
    assert request(
        state, higher_epoch, True, True, 0, cooldown_required=True
    )
    assert state["fault_latched"] is True
    assert state["rearm_pending_next_cycle"] is True
    assert apply(state)
    assert state["fault_latched"] is False
    assert state["rearm_pending_next_cycle"] is False


def test_j6_exact_trajectory_derating_abort_uses_thermal_latch():
    functions = load_interlock_functions()
    state = functions["make_thermal_interlock_state"]()
    assert functions["latch_thermal_interlock"](
        state,
        "EXACT_TRAJECTORY_DERATING_ABORT",
        11,
        0,
        11,
    )
    assert state["fault_latched"] is True
    assert state["trip_reason"] == "EXACT_TRAJECTORY_DERATING_ABORT"
    assert state["minimum_rearm_epoch"] == 12


def test_j6_thermal_cooldown_gap_or_hot_sample_revokes_release_evidence():
    functions = load_interlock_functions()
    state = functions["make_thermal_interlock_state"]()
    functions["observe_raw_temperature_thermal_trip"](
        state, 61.0, 60.0, 2, 0, 0
    )
    cooldown = functions["observe_thermal_cooldown_frame"]
    assert not cooldown(state, True, 0.0, 1.0, 2)
    assert cooldown(state, True, 1.0, 1.0, 2)
    assert functions["observe_explicit_interlock_release"](
        state, True, cooldown_required=True
    )
    assert not cooldown(state, False, 1.1, 1.0, 2)
    assert state["cooldown_frames"] == 0
    assert state["cooldown_ready"] is False
    assert state["release_observed"] is False


def test_j6_no_progress_improvement_resets_window_then_timeout_latches():
    functions = load_interlock_functions()
    state = functions["make_no_progress_watchdog_state"]()
    observe = functions["observe_no_progress_watchdog"]
    assert not observe(
        state, True, 0.20, 0.0, 5, 0, 0,
        timeout_seconds=10.0, minimum_improvement_rad=0.05,
        minimum_frames=3,
    )
    assert not observe(
        state, True, 0.14, 5.0, 5, 0, 0,
        timeout_seconds=10.0, minimum_improvement_rad=0.05,
        minimum_frames=3,
    )
    assert state["window_started_at"] == 5.0
    assert not observe(
        state, True, 0.14, 14.9, 5, 0, 0,
        timeout_seconds=10.0, minimum_improvement_rad=0.05,
        minimum_frames=3,
    )
    assert observe(
        state, True, 0.14, 15.0, 5, 0, 0,
        timeout_seconds=10.0, minimum_improvement_rad=0.05,
        minimum_frames=3,
    )
    assert state["fault_latched"] is True
    assert state["trip_activation_epoch"] == 5
    assert state["minimum_rearm_epoch"] == 6
    source = CONTROLLER.read_text(encoding="utf-8")
    new_profile = source.split(
        "if position_command_starts_new_profile(", 1
    )[1].split("            else:", 1)[0]
    assert "reset_no_progress_observation(no_progress_watchdog)" in new_profile


def test_j6_no_progress_rearm_requires_release_disabled_feedback_and_higher_epoch():
    functions = load_interlock_functions()
    state = functions["make_no_progress_watchdog_state"]()
    state.update({
        "fault_latched": True,
        "trip_activation_epoch": 11,
        "minimum_rearm_epoch": 12,
    })
    request = functions["request_interlock_rearm_for_next_cycle"]
    command = {
        "mode": "position",
        "active_joint_mask": [False] * 5 + [True],
        "activation_epoch": 12,
    }
    assert not request(
        state, command, True, True, 0, cooldown_required=False
    )
    assert functions["observe_explicit_interlock_release"](
        state, True, cooldown_required=False
    )
    assert not request(
        state, command, True, False, 0, cooldown_required=False
    )
    assert request(
        state, command, True, True, 0, cooldown_required=False
    )
    assert state["fault_latched"] is True
    assert functions["apply_pending_interlock_rearm_at_cycle_start"](state)
    assert state["fault_latched"] is False


def test_j6_receive_events_marks_only_a_newly_accepted_domain_release_packet():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]
    events = functions["make_command_receive_events"]()

    class FakeSocket:
        def __init__(self, packets):
            self.packets = list(packets)

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    active = command_payload(sequence=1)
    current, minimum, last_seen = receive_latest(
        FakeSocket([active]), None, 0, 0, None, None, events
    )
    assert events["domain_release_received"] is False

    brake = json.loads(command_payload(sequence=2))
    brake["mode"] = "brake"
    brake["active_joint_mask"] = [False] * 6
    brake["moving_joint_mask"] = [False] * 6
    brake["source_monotonic_ns"] = time.monotonic_ns()
    current, minimum, last_seen = receive_latest(
        FakeSocket([json.dumps(brake).encode()]),
        current,
        minimum,
        last_seen,
        None,
        None,
        events,
    )
    assert current["mode"] == "brake"
    assert events["domain_release_received"] is True


def test_j6_startup_binding_violation_immediately_revokes_cached_command():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]

    class FakeSocket:
        def __init__(self, packets):
            self.packets = list(packets)

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    current, minimum, last_seen = receive_latest(
        FakeSocket([command_payload(sequence=1)]), None, 0, 0
    )
    assert current is not None and current["mode"] == "position"

    def reject_startup_binding(_command, _received_ns, _state):
        raise functions["GravityAuthorityStartupBindingError"](
            "changed startup envelope"
        )

    functions["validate_empirical_command_authority"] = (
        reject_startup_binding
    )
    current, _minimum, _last_seen = receive_latest(
        FakeSocket([command_payload(sequence=2)]),
        current,
        minimum,
        last_seen,
        require_empirical_authority=True,
    )
    assert current is None


def test_j6_empirical_hold_brake_higher_epoch_race_stays_braked():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]
    envelope_sha256 = "a" * 64

    def validate_empirical(command, _received_ns, replay_state):
        if command["mode"] == "brake":
            return None
        if envelope_sha256 in replay_state["empirical_spent_sha256"]:
            raise ValueError("J6 empirical envelope spent")
        return (
            "v15-31b-empirical-0123456789abcdefabcd",
            envelope_sha256,
            "b" * 64,
            "session",
            "c" * 32,
            0,
        )

    functions["validate_empirical_command_authority"] = validate_empirical

    class FakeSocket:
        def __init__(self, packets):
            self.packets = list(packets)

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    source_ns = time.monotonic_ns() - 1_000_000
    active = json.loads(command_payload(
        source_monotonic_ns=source_ns, sequence=1
    ))
    active["mode"] = "hold"
    active["moving_joint_mask"] = [False] * 6
    active["activation_epoch"] = 7
    brake = dict(active)
    brake.update({
        "mode": "brake",
        "active_joint_mask": [False] * 6,
        "moving_joint_mask": [False] * 6,
        "sequence": 2,
        "source_monotonic_ns": source_ns + 1,
    })
    raced = dict(active)
    raced.update({
        "activation_epoch": 8,
        "sequence": 3,
        "source_monotonic_ns": source_ns + 2,
    })
    replay_state = functions["make_command_source_replay_state"]()
    rejection_state = functions["make_command_rejection_state"]()
    with redirect_stdout(io.StringIO()):
        current, minimum, last_seen = receive_latest(
            FakeSocket([
                json.dumps(active).encode(),
                json.dumps(brake).encode(),
                json.dumps(raced).encode(),
            ]),
            None,
            0,
            0,
            rejection_state,
            replay_state,
            require_empirical_authority=True,
        )
    assert current is not None and current["mode"] == "brake"
    assert envelope_sha256 in replay_state["empirical_spent_sha256"]
    assert replay_state["empirical_active_sha256"] is None
    assert rejection_state["total"] == 1
    assert minimum == 8 and last_seen == 7


def test_j6_empirical_startup_binding_revoke_spends_before_higher_epoch_race():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]
    envelope_sha256 = "9" * 64

    def validate_empirical(command, _received_ns, replay_state):
        sequence = command["sequence"]
        if sequence == 2:
            raise functions["GravityAuthorityStartupBindingError"](
                "changed startup envelope"
            )
        if envelope_sha256 in replay_state["empirical_spent_sha256"]:
            raise ValueError("J6 empirical envelope spent")
        return (
            "v15-31b-empirical-0123456789abcdefabcd",
            envelope_sha256,
            "8" * 64,
            "session",
            "7" * 32,
            0,
        )

    functions["validate_empirical_command_authority"] = validate_empirical

    class FakeSocket:
        def __init__(self, packets):
            self.packets = list(packets)

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    source_ns = time.monotonic_ns() - 1_000_000
    packets = []
    for sequence, epoch in ((1, 7), (2, 8), (3, 9)):
        packet = json.loads(command_payload(
            source_monotonic_ns=source_ns + sequence,
            sequence=sequence,
        ))
        packet.update({
            "mode": "hold",
            "moving_joint_mask": [False] * 6,
            "activation_epoch": epoch,
        })
        packets.append(json.dumps(packet).encode())

    replay_state = functions["make_command_source_replay_state"]()
    rejection_state = functions["make_command_rejection_state"]()
    with redirect_stdout(io.StringIO()):
        current, _minimum, _last_seen = receive_latest(
            FakeSocket(packets),
            None,
            0,
            0,
            rejection_state,
            replay_state,
            require_empirical_authority=True,
        )
    assert current is None
    assert replay_state["empirical_active_sha256"] is None
    assert envelope_sha256 in replay_state["empirical_spent_sha256"]
    assert rejection_state["total"] == 2


def test_j6_empirical_lease_expiry_spends_before_same_sha_receive():
    functions = load_command_channel_functions()
    replay_state = functions["make_command_source_replay_state"]()
    envelope_sha256 = "d" * 64
    functions["commit_empirical_command_authority"](
        replay_state,
        (
            "v15-31b-empirical-0123456789abcdefabcd",
            envelope_sha256,
            "e" * 64,
            "session",
            "f" * 32,
            0,
        ),
    )
    functions["spend_active_empirical_command_authority"](replay_state)
    assert replay_state["empirical_active_sha256"] is None
    assert envelope_sha256 in replay_state["empirical_spent_sha256"]

    source = CONTROLLER.read_text(encoding="utf-8")
    pre_receive = source.split(
        "lease_expired_before_receive = bool(", 1
    )[1].split(
        "command, minimum_activation_epoch, last_seen_activation_epoch = receive_latest(",
        1,
    )[0]
    assert "spend_active_empirical_command_authority(" in pre_receive
    assert "lease_expired_before_receive" in pre_receive
    assert "empirical_expired_before_receive" in pre_receive
    assert "fault_latched" in pre_receive
    assert 'thermal_interlock["fault_latched"]' in pre_receive
    assert 'no_progress_watchdog["fault_latched"]' in pre_receive
    assert "or STOP" in pre_receive


def test_j6_empirical_lifecycle_latch_ignores_pre_activation_and_official_brake():
    functions = load_command_channel_functions()
    replay_state = functions["make_command_source_replay_state"]()

    # Startup BRAKE and an official authority binding cannot spend or arm an
    # empirical permit that has never become active.
    functions["spend_active_empirical_command_authority"](replay_state)
    functions["commit_empirical_command_authority"](
        replay_state,
        (
            "OFFICIAL_CONTINUOUS_RATING",
            "session",
            "1" * 32,
            "",
            "",
            0,
        ),
    )
    functions["spend_active_empirical_command_authority"](replay_state)
    assert replay_state["empirical_active_sha256"] is None
    assert replay_state["empirical_spent_sha256"] == {}


def test_quintic_reference_uses_integer_grid_and_exact_endpoint_pins():
    start = -0.2
    target = 0.4
    execute_at = 10_000_000_000
    duration_ns = 1_000_000_000
    intervals = 100
    assert MODULE.quintic_sample_index(
        execute_at - 1, execute_at, duration_ns, intervals
    ) == 0
    assert MODULE.quintic_sample_index(
        execute_at + 499_999_999, execute_at, duration_ns, intervals
    ) == 49
    assert MODULE.quintic_sample_index(
        execute_at + 500_000_000, execute_at, duration_ns, intervals
    ) == 50
    assert MODULE.quintic_sample_index(
        execute_at + duration_ns, execute_at, duration_ns, intervals
    ) == intervals

    prepared = MODULE.quintic_reference_at(
        start, target, duration_ns, intervals, execute_at, execute_at - 1
    )
    midpoint = MODULE.quintic_reference_at(
        start, target, duration_ns, intervals, execute_at,
        execute_at + duration_ns // 2,
    )
    complete = MODULE.quintic_reference_at(
        start, target, duration_ns, intervals, execute_at,
        execute_at + duration_ns,
    )
    assert prepared == (start, 0.0, 0, "PREPARED")
    assert math.isclose(midpoint[0], 0.1, abs_tol=1e-15)
    assert midpoint[1] > 0.0
    assert midpoint[2:] == (50, "RUNNING")
    assert complete == (target, 0.0, intervals, "COMPLETE")


def test_quintic_posvel_cap_is_unsigned_and_never_changes_q_reference():
    vmax = math.radians(5.0)
    restore = math.radians(1.0)
    assert MODULE.quintic_posvel_speed_limit(-0.02, vmax, restore) == 0.02
    assert MODULE.quintic_posvel_speed_limit(0.0, vmax, restore) == restore
    assert MODULE.quintic_posvel_speed_limit(1.0, vmax, restore) == vmax


def test_quintic_evaluator_rejects_coerced_or_nonfinite_grid_inputs():
    invalid_calls = (
        lambda: MODULE.quintic_sample_index(True, 1, 1, 1),
        lambda: MODULE.quintic_sample_index(1, True, 1, 1),
        lambda: MODULE.quintic_sample_index(1, 1, 0, 1),
        lambda: MODULE.quintic_sample_index(1, 1, 1, 1_000_001),
        lambda: MODULE.quintic_sample(math.nan, 0.0, 1, 1, 0),
        lambda: MODULE.quintic_sample(0.0, math.inf, 1, 1, 0),
        lambda: MODULE.quintic_sample(0.0, 1.0, 1, 1, True),
        lambda: MODULE.quintic_posvel_speed_limit(-0.1, 0.0, 0.0),
        lambda: MODULE.quintic_posvel_speed_limit(-0.1, 0.2, -0.01),
    )
    for call in invalid_calls:
        try:
            call()
        except ValueError:
            pass
        else:
            raise AssertionError("quintic evaluator accepted an invalid value")


def test_posvel_speed_limit_is_positive_bounded_and_acceleration_limited():
    period = 0.01
    vmax = math.radians(5.0)
    amax = math.radians(20.0)
    restore = math.radians(1.0)
    current = 0.0
    for _ in range(100):
        previous = current
        current = MODULE.update_posvel_speed_limit(
            current,
            math.radians(-0.047),
            math.radians(0.20),
            vmax,
            amax,
            restore,
            period,
        )
        assert 0.0 < current <= vmax
        assert abs(current - previous) <= amax * period + 1e-12


def test_posvel_speed_limit_keeps_endpoint_restore_authority():
    period = 0.01
    vmax = math.radians(5.0)
    amax = math.radians(20.0)
    restore = math.radians(1.0)
    current = vmax
    for _ in range(100):
        current = MODULE.update_posvel_speed_limit(
            current, 0.25, 0.25, vmax, amax, restore, period
        )
    assert math.isclose(current, restore, abs_tol=1e-12)


def test_posvel_fixed_target_converges_and_restores_external_displacement():
    period = 0.01
    vmax = math.radians(5.0)
    amax = math.radians(20.0)
    restore = math.radians(1.0)
    target = math.radians(0.20)
    actual = math.radians(-0.047)
    speed_limit = 0.0

    def advance(actual_position, current_limit, cycle_count):
        for _ in range(cycle_count):
            current_limit = MODULE.update_posvel_speed_limit(
                current_limit,
                actual_position,
                target,
                vmax,
                amax,
                restore,
                period,
            )
            error = target - actual_position
            # A deliberately lagging pure plant: p_des remains the immutable
            # final target while the physical speed is only part of v_des.
            step = min(abs(error), current_limit * period * 0.6)
            actual_position += math.copysign(step, error) if step else 0.0
        return actual_position, current_limit

    actual, speed_limit = advance(actual, speed_limit, 300)
    assert math.isclose(actual, target, abs_tol=math.radians(0.001))
    assert speed_limit > 0.0

    actual -= math.radians(0.10)
    actual, speed_limit = advance(actual, speed_limit, 300)
    assert math.isclose(actual, target, abs_tol=math.radians(0.001))
    assert speed_limit > 0.0


def test_posvel_speed_limit_rejects_invalid_inputs():
    valid = (0.0, 0.0, 0.1, 0.2, 0.3, 0.1, 0.01)
    invalid_cases = (
        (math.nan,) + valid[1:],
        valid[:3] + (0.0,) + valid[4:],
        valid[:4] + (-0.3,) + valid[5:],
        valid[:5] + (0.0,) + valid[6:],
        valid[:-1] + (0.0,),
        valid[:5] + (0.3,) + valid[6:],
    )
    for values in invalid_cases:
        try:
            MODULE.update_posvel_speed_limit(*values)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted invalid POS_VEL inputs: {values!r}")


def test_j6_parser_requires_v12_source_and_six_boolean_mask():
    parse_command = load_parse_command()
    assert parse_command(command_payload())["active_joint_mask"][5] is True
    for payload in (
        command_payload(schema="go-m8010-gui-command/1.0"),
        command_payload(schema="go-m8010-gui-command/1.1"),
        command_payload(mask=[False] * 5),
        command_payload(mask=[False] * 5 + [1]),
    ):
        try:
            parse_command(payload)
        except ValueError:
            pass
        else:
            raise AssertionError("J6 接受了不安全的命令协议或激活掩码")


def test_j6_parser_keeps_legacy_brake_compatibility():
    parse_command = load_parse_command()
    for schema in (
        "go-m8010-gui-command/1.0",
        "go-m8010-gui-command/1.1",
        "go-m8010-gui-command/1.2",
        "go-m8010-gui-command/1.3",
    ):
        document = json.loads(command_payload(schema=schema))
        document["mode"] = "brake"
        document["active_joint_mask"] = [False] * 6
        document["source_instance_id"] = 7
        document["source_monotonic_ns"] = False
        document["sequence"] = 0
        if schema == "go-m8010-gui-command/1.0":
            document.pop("active_joint_mask")
        parsed = parse_command(json.dumps(document).encode())
        assert parsed["active_joint_mask"] == [False] * 6


def test_j6_parser_accepts_and_normalizes_v13_quintic_descriptor():
    parse_command = load_parse_command()
    received_ns = 20_000_000_000
    document = quintic_command_document(
        received_monotonic_ns=received_ns,
        execute_at_monotonic_ns=received_ns + 100_000_000,
    )
    parsed = parse_command(json.dumps(document).encode(), received_ns)
    assert parsed["schema"] == "go-m8010-gui-command/1.3"
    assert parsed["plan_token_id"] == "a" * 64
    assert parsed["trajectory"]["trajectory_sha256"] == "b" * 64
    assert parsed["trajectory"]["start_rad"][5] == 0.0
    assert parsed["trajectory"]["target_rad"] == parsed["targets_rad"]
    assert parsed["trajectory"]["duration_ns"] == 1_000_000_000
    assert parsed["trajectory"]["interval_count"] == 100


def test_j6_parser_rejects_malformed_or_unbounded_v13_quintic_fields():
    parse_command = load_parse_command()
    received_ns = 30_000_000_000
    base = quintic_command_document(
        received_monotonic_ns=received_ns,
        execute_at_monotonic_ns=received_ns + 100_000_000,
    )
    mutations = []

    def changed(mutator):
        document = json.loads(json.dumps(base))
        mutator(document)
        mutations.append(document)

    changed(lambda value: value.__setitem__("plan_token_id", "A" * 64))
    changed(lambda value: value.__setitem__("plan_token_id", "a" * 63))
    changed(lambda value: value.__setitem__("trajectory", None))
    changed(lambda value: value["trajectory"].__setitem__("extra", 1))
    changed(lambda value: value["trajectory"].__setitem__("schema", "bad"))
    changed(lambda value: value["trajectory"].__setitem__(
        "trajectory_sha256", "g" * 64
    ))
    changed(lambda value: value["trajectory"].__setitem__("profile", "linear"))
    changed(lambda value: value["trajectory"].__setitem__("start_rad", [0.0] * 5))
    changed(lambda value: value["trajectory"]["start_rad"].__setitem__(5, True))
    changed(lambda value: value["trajectory"]["target_rad"].__setitem__(5, 0.02))
    changed(lambda value: value["trajectory"]["start_rad"].__setitem__(0, 0.01))
    changed(lambda value: value.__setitem__(
        "moving_joint_mask", [True] + [False] * 4 + [True]
    ))
    changed(lambda value: value["trajectory"].__setitem__("duration_ns", True))
    changed(lambda value: value["trajectory"].__setitem__("duration_ns", 0))
    changed(lambda value: value["trajectory"].__setitem__("interval_count", 0))
    changed(lambda value: value["trajectory"].__setitem__(
        "interval_count", 1_000_001
    ))
    changed(lambda value: value["trajectory"].__setitem__("interval_count", 99))
    changed(lambda value: value["trajectory"].__setitem__(
        "execute_at_monotonic_ns", 0
    ))
    changed(lambda value: value["trajectory"].__setitem__("segment_index", -1))
    changed(lambda value: value["trajectory"].__setitem__("segment_count", 0))
    changed(lambda value: value["trajectory"].__setitem__("segment_index", 1))
    # 0.1 rad in one second exceeds the frozen 5 deg/s quintic peak.
    def excessive_velocity(value):
        value["targets_rad"][5] = 0.1
        value["trajectory"]["target_rad"][5] = 0.1
    changed(excessive_velocity)

    for document in mutations:
        try:
            parse_command(json.dumps(document).encode(), received_ns)
        except ValueError:
            pass
        else:
            raise AssertionError(f"accepted malformed v1.3 descriptor: {document!r}")


def test_j6_v13_position_reference_uses_descriptor_grid_not_final_endpoint():
    parse_command = load_parse_command()
    functions = load_quintic_controller_functions()
    reference = functions["v13_position_posvel_reference"]
    received_ns = 40_000_000_000
    execute_at_ns = received_ns + 100_000_000
    command = parse_command(
        json.dumps(quintic_command_document(
            received_monotonic_ns=received_ns,
            execute_at_monotonic_ns=execute_at_ns,
            start_rad=-0.01,
            target_rad=0.01,
        )).encode(),
        received_ns,
    )

    prepared = reference(command, execute_at_ns - 1)
    started = reference(command, execute_at_ns)
    midpoint = reference(command, execute_at_ns + 500_000_000)
    complete = reference(command, execute_at_ns + 1_000_000_000)

    assert prepared[0] == -0.01
    assert prepared[0] != command["targets_rad"][5]
    assert prepared[2:] == ("PREPARED", 0)
    assert started[0] == -0.01
    assert started[2:] == ("RUNNING", 0)
    assert math.isclose(midpoint[0], 0.0, abs_tol=1e-15)
    assert midpoint[2:] == ("RUNNING", 50)
    assert complete[0] == command["targets_rad"][5]
    assert complete[2:] == ("COMPLETE", 100)
    for _q_ref, speed_limit, _state, _index in (
        prepared, started, midpoint, complete
    ):
        assert 0.0 <= speed_limit <= command["maximum_velocity_rad_s"]


def test_j6_v13_transition_rejects_late_first_packet_and_same_epoch_mutation():
    parse_command = load_parse_command()
    validate = load_command_channel_functions()[
        "validate_position_execution_transition"
    ]
    received_ns = 50_000_000_000
    execute_at_ns = received_ns + 100_000_000
    current_document = quintic_command_document(
        received_monotonic_ns=received_ns,
        execute_at_monotonic_ns=execute_at_ns,
        sequence=1,
    )
    current = parse_command(
        json.dumps(current_document).encode(), received_ns
    )
    validate(None, current, received_ns)

    heartbeat_received_ns = execute_at_ns + 10_000_000
    heartbeat_document = quintic_command_document(
        received_monotonic_ns=heartbeat_received_ns,
        execute_at_monotonic_ns=execute_at_ns,
        sequence=2,
    )
    heartbeat = parse_command(
        json.dumps(heartbeat_document).encode(), heartbeat_received_ns
    )
    # A previously prepared, byte-equivalent descriptor may renew its lease
    # after the common start; it is not a new first packet.
    validate(current, heartbeat, heartbeat_received_ns)

    mutations = []

    def changed(mutator):
        document = json.loads(json.dumps(heartbeat_document))
        mutator(document)
        mutations.append(document)

    changed(lambda value: value.__setitem__("plan_token_id", "c" * 64))
    changed(lambda value: value["trajectory"].__setitem__(
        "trajectory_sha256", "d" * 64
    ))
    changed(lambda value: value["trajectory"].__setitem__(
        "execute_at_monotonic_ns", execute_at_ns + 1
    ))
    changed(lambda value: value["trajectory"].__setitem__(
        "segment_count", 2
    ))
    for document in mutations:
        candidate = parse_command(
            json.dumps(document).encode(), heartbeat_received_ns
        )
        try:
            validate(current, candidate, heartbeat_received_ns)
        except ValueError as error:
            assert "同一激活纪元轨迹描述符发生变化" in str(error)
        else:
            raise AssertionError("same-epoch quintic descriptor mutation accepted")

    late_received_ns = 60_000_000_000
    late_document = quintic_command_document(
        received_monotonic_ns=late_received_ns,
        execute_at_monotonic_ns=late_received_ns - 1,
    )
    late = parse_command(json.dumps(late_document).encode(), late_received_ns)
    try:
        validate(None, late, late_received_ns)
    except ValueError as error:
        assert "首包晚于执行起点" in str(error)
    else:
        raise AssertionError("late first v1.3 POSITION packet was accepted")


def test_j6_position_authority_survives_hold_for_v13_and_v12_contracts():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]

    class FakeSocket:
        def __init__(self, packet):
            self.packets = [json.dumps(packet).encode()]

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    for schema in (
        "go-m8010-gui-command/1.3",
        "go-m8010-gui-command/1.2",
    ):
        base_ns = time.monotonic_ns() - 10_000_000
        execute_at_ns = base_ns + 1_000_000_000
        source_id = (
            "11111111111111111111111111111111"
            if schema.endswith("1.3")
            else "22222222222222222222222222222222"
        )

        def position(target, sequence, source_ns):
            if schema.endswith("1.3"):
                document = quintic_command_document(
                    received_monotonic_ns=source_ns,
                    execute_at_monotonic_ns=execute_at_ns,
                    sequence=sequence,
                    activation_epoch=7,
                    target_rad=target,
                )
                document["source_instance_id"] = source_id
                return document
            document = json.loads(command_payload(
                schema=schema,
                source_instance_id=source_id,
                source_monotonic_ns=source_ns,
                sequence=sequence,
            ))
            document["activation_epoch"] = 7
            document["targets_rad"][5] = target
            return document

        accepted = position(0.01, 1, base_ns)
        hold = json.loads(command_payload(
            source_instance_id=source_id,
            source_monotonic_ns=base_ns + 1,
            sequence=2,
        ))
        hold["mode"] = "hold"
        hold["activation_epoch"] = 7
        hold["moving_joint_mask"] = [False] * 6
        hold["targets_rad"][5] = 0.01
        mutation = position(0.02, 3, base_ns + 2)
        retry_original = position(0.01, 3, base_ns + 3)

        replay_state = functions["make_command_source_replay_state"]()
        rejection_state = functions["make_command_rejection_state"]()
        current = None
        minimum = last_seen = 0
        current, minimum, last_seen = receive_latest(
            FakeSocket(accepted),
            current,
            minimum,
            last_seen,
            rejection_state,
            replay_state,
        )
        binding_key = (source_id, 7)
        frozen_contract = replay_state["position_execution_bindings"][binding_key]

        current, minimum, last_seen = receive_latest(
            FakeSocket(hold),
            current,
            minimum,
            last_seen,
            rejection_state,
            replay_state,
        )
        assert current["mode"] == "hold", schema
        assert replay_state["position_execution_bindings"][binding_key] == frozen_contract

        with redirect_stdout(io.StringIO()):
            current, minimum, last_seen = receive_latest(
                FakeSocket(mutation),
                current,
                minimum,
                last_seen,
                rejection_state,
                replay_state,
            )
        assert current["mode"] == "hold", schema
        assert rejection_state["last_reason"] == (
            "POSITION同一激活纪元轨迹描述符发生变化"
        )
        assert replay_state["sources"][source_id]["sequence"] == 2
        assert replay_state["position_execution_bindings"][binding_key] == frozen_contract

        # The rejected sequence did not poison source replay or authority; an
        # exact-contract retry with that sequence remains acceptable.
        current, minimum, last_seen = receive_latest(
            FakeSocket(retry_original),
            current,
            minimum,
            last_seen,
            rejection_state,
            replay_state,
        )
        assert current["mode"] == "position", schema
        assert current["targets_rad"][5] == 0.01
        assert replay_state["sources"][source_id]["sequence"] == 3
        assert replay_state["position_execution_bindings"][binding_key] == frozen_contract


def test_j6_late_first_v13_packet_does_not_commit_source_or_authority():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]
    replay_state = functions["make_command_source_replay_state"]()
    rejection_state = functions["make_command_rejection_state"]()
    base_ns = time.monotonic_ns() - 10_000_000

    class FakeSocket:
        def __init__(self, packet):
            self.packets = [json.dumps(packet).encode()]

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    late = quintic_command_document(
        received_monotonic_ns=base_ns,
        execute_at_monotonic_ns=base_ns - 1,
        sequence=1,
        activation_epoch=7,
    )
    with redirect_stdout(io.StringIO()):
        current, minimum, last_seen = receive_latest(
            FakeSocket(late),
            None,
            0,
            0,
            rejection_state,
            replay_state,
        )
    assert current is None
    assert (minimum, last_seen) == (0, 0)
    assert replay_state["sources"] == {}
    assert replay_state["position_execution_bindings"] == {}

    # Reuse the rejected sequence and source timestamp with a future start.
    # Acceptance proves the late packet committed neither replay nor binding.
    valid = quintic_command_document(
        received_monotonic_ns=base_ns,
        execute_at_monotonic_ns=time.monotonic_ns() + 1_000_000_000,
        sequence=1,
        activation_epoch=7,
    )
    current, minimum, last_seen = receive_latest(
        FakeSocket(valid),
        current,
        minimum,
        last_seen,
        rejection_state,
        replay_state,
    )
    assert current is not None
    assert current["mode"] == "position"
    assert len(replay_state["sources"]) == 1
    assert len(replay_state["position_execution_bindings"]) == 1


def test_j6_parser_requires_fresh_strict_source_metadata_for_every_non_brake_mode():
    parse_command = load_parse_command()
    checked_at_ns = 10_000_000_000
    source_id = "0123456789abcdef0123456789abcdef"

    for mode in ("drag", "hold", "position"):
        valid = json.loads(command_payload(
            source_instance_id=source_id,
            source_monotonic_ns=checked_at_ns - 250_000_000,
        ))
        valid["mode"] = mode
        assert parse_command(
            json.dumps(valid).encode(), checked_at_ns
        )["source_instance_id"] == source_id
        for field in ("source_instance_id", "source_monotonic_ns", "sequence"):
            missing = dict(valid)
            missing.pop(field)
            try:
                parse_command(json.dumps(missing).encode(), checked_at_ns)
            except ValueError:
                pass
            else:
                raise AssertionError(f"J6 {mode} 接受了缺失的 {field}")

    invalid_source_ids = (
        "0123456789abcdef0123456789abcde",
        "0123456789abcdef0123456789abcdef0",
        "0123456789ABCDEF0123456789ABCDEF",
        123,
    )
    invalid_timestamps = (
        True,
        float(checked_at_ns),
        0,
        checked_at_ns - 250_000_001,
        checked_at_ns + 1,
    )
    invalid_sequences = (True, 1.0, 0, 1 << 63)
    for field, invalid_values in (
        ("source_instance_id", invalid_source_ids),
        ("source_monotonic_ns", invalid_timestamps),
        ("sequence", invalid_sequences),
    ):
        for invalid_value in invalid_values:
            document = json.loads(command_payload(
                source_instance_id=source_id,
                source_monotonic_ns=checked_at_ns,
            ))
            document[field] = invalid_value
            try:
                parse_command(json.dumps(document).encode(), checked_at_ns)
            except ValueError:
                pass
            else:
                raise AssertionError(
                    f"J6 接受了非法来源字段 {field}={invalid_value!r}"
                )


def test_j6_parser_normalizes_brake_targets_outside_motion_window():
    parse_command = load_parse_command()
    document = json.loads(command_payload())
    document["mode"] = "brake"
    document["active_joint_mask"] = [False] * 6
    document["targets_rad"] = [
        math.radians(3.0), math.radians(-88.0), math.radians(155.0),
        math.radians(-32.0), math.radians(8.0), math.radians(-1.0),
    ]
    parsed = parse_command(json.dumps(document).encode())
    assert parsed["targets_rad"] == [0.0] * 6


def test_j6_parser_accepts_hold_across_mechanical_envelope():
    parse_command = load_parse_command()
    document = json.loads(command_payload())
    document["mode"] = "hold"
    document["targets_rad"] = [
        math.radians(3.0), math.radians(-88.0), math.radians(155.0),
        math.radians(-32.0), math.radians(8.0), math.radians(-100.0),
    ]
    document["moving_joint_mask"] = [False] * 6
    parsed = parse_command(json.dumps(document).encode())
    assert parsed["targets_rad"][5] == document["targets_rad"][5]
    assert parsed["moving_joint_mask"] == [False] * 6


def test_j6_parser_uses_same_full_model_range_for_moving_and_nonmoving_targets():
    parse_command = load_parse_command()
    document = json.loads(command_payload())
    document["targets_rad"][5] = math.radians(-100.0)
    document["moving_joint_mask"] = [False] * 6
    assert parse_command(json.dumps(document).encode())["targets_rad"][5] == document["targets_rad"][5]

    document["moving_joint_mask"][5] = True
    assert parse_command(json.dumps(document).encode())["targets_rad"][5] == document["targets_rad"][5]


def test_j6_parser_rejects_nonboolean_recovery_before_it_can_widen_window():
    parse_command = load_parse_command()
    for invalid_recovery in ("false", "true", 0, 1, None):
        document = json.loads(command_payload())
        document["targets_rad"][5] = math.radians(20.0)
        document["recovery"] = invalid_recovery
        try:
            parse_command(json.dumps(document).encode())
        except ValueError as error:
            assert "恢复标记必须是布尔值" in str(error)
        else:
            raise AssertionError("J6 接受了非布尔 recovery 并放宽了目标窗口")


def test_j6_recovery_flag_cannot_widen_full_model_position_range():
    parse_command = load_parse_command()
    moving = json.loads(command_payload())
    moving["recovery"] = True
    moving["targets_rad"][5] = math.radians(170.0)
    moving["moving_joint_mask"][5] = True
    parsed = parse_command(json.dumps(moving).encode())
    assert math.isclose(parsed["targets_rad"][5], math.radians(170.0))

    moving["targets_rad"][5] = math.radians(180.01)
    try:
        parse_command(json.dumps(moving).encode())
    except ValueError as error:
        assert "J6目标超出模型机械限位" in str(error)
    else:
        raise AssertionError("J6 recovery布尔值绕过了完整模型限位")

    fixed = dict(moving)
    fixed["targets_rad"] = list(moving["targets_rad"])
    fixed["moving_joint_mask"] = [False] * 6
    try:
        parse_command(json.dumps(fixed).encode())
    except ValueError as error:
        assert "J6目标超出模型机械限位" in str(error)
    else:
        raise AssertionError("J6 非移动目标绕过了完整模型限位")


def test_j6_parser_accepts_exact_full_model_endpoints_for_position_and_hold():
    parse_command = load_parse_command()
    for mode in ("position", "hold"):
        for endpoint in (-180.0, 180.0):
            document = json.loads(command_payload())
            document["mode"] = mode
            document["moving_joint_mask"] = (
                [False] * 5 + [True] if mode == "position" else [False] * 6
            )
            document["targets_rad"][5] = math.radians(endpoint)
            parsed = parse_command(json.dumps(document).encode())
            assert math.isclose(parsed["targets_rad"][5], math.radians(endpoint))


def test_j6_parser_rejects_beyond_full_model_endpoints_for_position_and_hold():
    parse_command = load_parse_command()
    for mode in ("position", "hold"):
        for endpoint in (-180.01, 180.01):
            document = json.loads(command_payload())
            document["mode"] = mode
            document["moving_joint_mask"] = (
                [False] * 5 + [True] if mode == "position" else [False] * 6
            )
            document["targets_rad"][5] = math.radians(endpoint)
            try:
                parse_command(json.dumps(document).encode())
            except ValueError as error:
                assert "J6目标超出模型机械限位" in str(error)
            else:
                raise AssertionError(f"J6 接受了越界目标 {endpoint}°")


def test_j6_parser_rejects_zero_epoch_active_command():
    parse_command = load_parse_command()
    document = json.loads(command_payload())
    document["activation_epoch"] = 0
    try:
        parse_command(json.dumps(document).encode())
    except ValueError as error:
        assert "必须大于零" in str(error)
    else:
        raise AssertionError("J6 接受了零激活纪元主动命令")


def test_j6_parser_rejects_coerced_numeric_motion_fields():
    parse_command = load_parse_command()
    for field, value in (
        ("targets_rad", "0.0"),
        ("targets_rad", True),
        ("maximum_velocity_rad_s", "0.1"),
        ("maximum_velocity_rad_s", True),
        ("maximum_acceleration_rad_s2", -0.1),
    ):
        document = json.loads(command_payload())
        if field == "targets_rad":
            document[field][5] = value
        else:
            document[field] = value
        try:
            parse_command(json.dumps(document).encode())
        except ValueError:
            pass
        else:
            raise AssertionError(f"J6 接受了强制转换的数值字段: {field}")


def test_j6_control_loop_checks_its_own_active_bit_before_enable():
    source = CONTROLLER.read_text(encoding="utf-8")
    assert 'and command["active_joint_mask"][5]' in source
    assert 'pre_enable_mode = safe_pre_enable_mode' in source
    assert source.count("safe_pre_enable_mode(") >= 4
    assert source.count("= receive_latest(") >= 4
    assert "MODEL_COMMAND_LOWER" in source
    assert "FEEDBACK_ENVELOPE_TOLERANCE = math.radians(0.5)" in source
    assert "FEEDBACK_HARD_LOWER" in source
    assert "update_posvel_speed_limit(" in source
    assert "protocol_position = reference - requested_target" in source
    assert "protocol_velocity = dq_command" in source
    assert "hold_velocity_limit" in source
    assert "reference - fixed_hold_target" in source
    assert "no_progress_position_error_rad <= ARRIVAL_TOLERANCE" in source
    assert "observe_no_progress_watchdog(" in source
    assert "J6_EXTERNAL_MOTION_OBSERVED" in source
    assert "J6_POSITION_ARRIVAL_OVERDUE" in source
    assert "ACTIVE_DEADLINE_CONSECUTIVE_LIMIT" in source


def test_j6_posvel_frame_contract_keeps_final_target_and_positive_restore_cap():
    source = CONTROLLER.read_text(encoding="utf-8")
    position_block = source.split(
        'elif mode == "position" and command is not None and enabled:', 1
    )[1].split('elif mode == "hold" and enabled:', 1)[0]
    hold_block = source.split('elif mode == "hold" and enabled:', 1)[1].split(
        "else:\n                logger.send", 1
    )[0]
    assert "q_command = requested_target" in position_block
    assert "reference - requested_target" in position_block
    assert "protocol_velocity = dq_command" in position_block
    assert "update_profile(" not in position_block
    assert "hold_velocity_limit" in hold_block
    assert "reference - fixed_hold_target" in hold_block
    assert "fixed_hold_target,\n                    0.0" not in hold_block
    assert 'command["maximum_velocity_rad_s"]' not in hold_block
    assert "fixed_hold_velocity_limit()" in hold_block
    quintic_block = position_block.split(
        "if command_is_v13_quintic_position(command):", 1
    )[1].split("                else:", 1)[0]
    assert "v13_position_posvel_reference(" in quintic_block
    assert "protocol_position = reference - q_ref" in quintic_block
    assert "protocol_velocity = speed_limit" in quintic_block
    assert "requested_target" not in quintic_block
    assert "update_posvel_speed_limit(" not in quintic_block
    assert '"GUI_POSITION_QUINTIC_REFRESH"' in quintic_block


def test_j6_rejected_packet_cannot_reduce_fixed_hold_restore_cap():
    limit = load_mode_functions()["fixed_hold_velocity_limit"]
    assert math.isclose(limit(), math.radians(1.0), abs_tol=1e-12)


def test_j6_arrival_timeout_is_not_cleared_by_stationary_point_two_degree_error():
    functions = load_interlock_functions()
    state = functions["make_no_progress_watchdog_state"]()
    observe = functions["observe_no_progress_watchdog"]
    error = math.radians(0.20)
    assert not observe(
        state, True, error, 0.0, 7, 0, 0,
        timeout_seconds=90.0, minimum_frames=3,
    )
    assert not observe(
        state, True, error, 89.999, 7, 0, 0,
        timeout_seconds=90.0, minimum_frames=3,
    )
    assert observe(
        state, True, error, 90.0, 7, 0, 0,
        timeout_seconds=90.0, minimum_frames=3,
    )
    assert state["fault_latched"] is True
    assert state["minimum_rearm_epoch"] == 8


def test_j6_external_push_after_arrival_keeps_position_authority():
    source = CONTROLLER.read_text(encoding="utf-8")
    velocity_observer = source.split(
        "rapid_motion = bool", 1
    )[1].split("current_temperature_c = float", 1)[0]
    assert "abs(decoded.velocity) > 0.7" in velocity_observer
    assert "fault_latched = True" not in velocity_observer
    assert "J6_EXTERNAL_MOTION_OBSERVED" in source
    assert "J6_EXTERNAL_MOTION_SETTLED" in source
    assert "position_started_at = None" in source


def test_j6_completed_position_lease_expiry_preserves_target_not_feedback():
    capture = load_mode_functions()["capture_lease_safe_hold_target"]
    command = {
        "mode": "position",
        "targets_rad": [0.0] * 5 + [math.radians(0.20)],
        "active_joint_mask": [False] * 5 + [True],
        "moving_joint_mask": [False] * 5 + [True],
        "activation_epoch": 7,
        "received_at": 10.0,
    }
    displaced = SimpleNamespace(
        position=1.90,
        velocity=0.20,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    captured = capture(
        command,
        True,
        None,
        displaced,
        10.59,
        2.0,
        False,
        True,
        True,
        10.60,
        completed_position_target=math.radians(0.20),
    )
    assert math.isclose(captured, math.radians(0.20), abs_tol=1e-12)
    assert not math.isclose(captured, -(displaced.position - 2.0), abs_tol=1e-12)


def test_j6_external_force_offset_during_unfinished_position_keeps_endpoint():
    capture = load_mode_functions()["capture_lease_safe_hold_target"]
    authorized_endpoint = math.radians(-35.0)
    command = {
        "mode": "position",
        "targets_rad": [0.0] * 5 + [authorized_endpoint],
        "active_joint_mask": [False] * 5 + [True],
        "moving_joint_mask": [False] * 5 + [True],
        "activation_epoch": 7,
        "received_at": 10.0,
    }
    externally_displaced = SimpleNamespace(
        # reference=2.0 would make this measured logical angle +0.80 rad,
        # deliberately far from the authorized negative endpoint.
        position=1.20,
        velocity=1.20,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    captured = capture(
        command,
        True,
        None,
        externally_displaced,
        10.59,
        2.0,
        False,
        True,
        True,
        10.60,
    )
    displaced_feedback = -(externally_displaced.position - 2.0)
    assert math.isclose(captured, authorized_endpoint, abs_tol=1e-12)
    assert not math.isclose(captured, displaced_feedback, abs_tol=1e-12)


def test_j6_lease_crossing_mid_cycle_preserves_confirmation_for_next_capture():
    functions = load_mode_functions()
    lease_fresh = functions["command_lease_is_fresh"]
    next_confirmation = functions["next_prior_external_hold_confirmation"]
    capture = functions["capture_lease_safe_hold_target"]
    target = 0.20
    command = {
        "mode": "hold",
        "targets_rad": [0.0] * 5 + [target],
        "active_joint_mask": [False] * 5 + [True],
        "moving_joint_mask": [False] * 6,
        "activation_epoch": 7,
        "received_at": 10.0,
    }

    # The command was selected and sent while fresh, then crossed the 500 ms
    # lease boundary during that same control cycle.
    assert lease_fresh(command, 10.499)
    assert not lease_fresh(command, 10.501)
    prior = next_confirmation(
        False,
        lease_safe_hold_active=False,
        enabled=True,
        fault_latched=False,
        external_active_confirmed_this_cycle=True,
    )
    # End-of-cycle staleness is not a release/fault/disable and must not erase
    # the proof needed by the immediately following lease-capture cycle.
    prior = next_confirmation(
        prior,
        lease_safe_hold_active=False,
        enabled=True,
        fault_latched=False,
        external_active_confirmed_this_cycle=False,
    )
    assert prior is True

    feedback = SimpleNamespace(
        position=1.80,
        velocity=0.0,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    captured = capture(
        command,
        prior,
        target,
        feedback,
        10.50,
        2.0,
        False,
        True,
        True,
        10.501,
    )
    assert math.isclose(captured, target, abs_tol=1e-12)
    assert next_confirmation(
        prior,
        lease_safe_hold_active=True,
        enabled=True,
        fault_latched=False,
        external_active_confirmed_this_cycle=False,
    ) is False
    assert next_confirmation(
        prior,
        lease_safe_hold_active=False,
        enabled=False,
        fault_latched=False,
        external_active_confirmed_this_cycle=False,
    ) is False
    assert next_confirmation(
        prior,
        lease_safe_hold_active=False,
        enabled=True,
        fault_latched=True,
        external_active_confirmed_this_cycle=False,
    ) is False

    source = CONTROLLER.read_text(encoding="utf-8")
    bookkeeping = source.split("            send_feedback(", 1)[1].split(
        "            previous_mode = mode", 1
    )[0]
    assert "and command_lease_fresh" in bookkeeping
    assert "command_lease_is_fresh(command)" not in bookkeeping


def test_j6_fault_dominates_rejected_target_fallback_before_active_send():
    functions = load_mode_functions()
    fallback = functions["rejected_hold_fallback"]
    dominate = functions["fault_dominant_mode"]

    # A rejected current command could otherwise restore an old target by
    # rewriting BRAKE back to HOLD after lease-safe feedback health was lost.
    fallback_target, _ = fallback(
        True,
        "hold",
        None,
        None,
        0.20,
        7,
    )
    assert fallback_target == 0.20
    assert dominate("hold", True) == "brake"
    assert dominate("position", True) == "brake"
    assert dominate("hold", False) == "hold"

    source = CONTROLLER.read_text(encoding="utf-8")
    unsafe_start = source.index("            if unsafe_active_entry:")
    final_fault_gate = source.index(
        "            mode = fault_dominant_mode(mode, fault_latched)",
        unsafe_start,
    )
    active_decision = source.index(
        '            active = mode in {"hold", "position"}',
        final_fault_gate,
    )
    active_send = source.index(
        '            if enabled and not enabled_confirmed:',
        active_decision,
    )
    assert unsafe_start < final_fault_gate < active_decision < active_send
    unsafe_definition = source[
        source.index("            unsafe_active_entry = bool("):
        unsafe_start
    ]
    assert "and not fault_latched" in unsafe_definition


def test_j6_stale_cached_feedback_blocks_new_position_before_send():
    healthy = load_mode_functions()["enabled_feedback_is_healthy"]
    feedback = SimpleNamespace(
        position=1.80,
        velocity=0.0,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    # The previous cycle ended just inside the 150 ms feedback lease; the next
    # cycle starts outside it while a new POSITION packet is available.
    assert healthy(feedback, 10.0, 2.0, False, True, True, 10.149)
    assert not healthy(feedback, 10.0, 2.0, False, True, True, 10.159)

    source = CONTROLLER.read_text(encoding="utf-8")
    pre_send_gate = source.index(
        "            # Do not apply any new target using cached feedback"
    )
    mode_selection = source.index("            mode = (", pre_send_gate)
    final_fault_gate = source.index(
        "            mode = fault_dominant_mode(mode, fault_latched)",
        mode_selection,
    )
    final_health_gate = source.rindex(
        "and not enabled_feedback_is_healthy(",
        mode_selection,
        final_fault_gate,
    )
    position_send = source.index(
        '            elif mode == "position" and command is not None and enabled:',
        final_fault_gate,
    )
    assert (
        pre_send_gate < mode_selection < final_health_gate
        < final_fault_gate < position_send
    )
    gate = source[pre_send_gate:mode_selection]
    assert "enabled_feedback_is_healthy(" in gate
    assert "fault_latched = True" in gate


def test_j6_feedback_hard_envelope_has_half_degree_endpoint_noise_tolerance():
    healthy = load_mode_functions()["enabled_feedback_is_healthy"]
    feedback = SimpleNamespace(
        position=-math.radians(180.49),
        velocity=0.0,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    assert healthy(feedback, 10.0, 0.0, False, True, True, 10.1)
    feedback.position = -math.radians(180.51)
    assert not healthy(feedback, 10.0, 0.0, False, True, True, 10.1)


def test_j6_pre_enable_receive_cannot_change_position_target_in_same_epoch():
    safe_mode = load_mode_functions()["safe_pre_enable_mode"]
    feedback = SimpleNamespace(
        position=2.0,
        velocity=0.0,
        state=0,
        mos_temp=25,
        coil_temp=26,
    )
    changed_position = {
        "mode": "position",
        "targets_rad": [0.0] * 5 + [0.70],
        "active_joint_mask": [False] * 5 + [True],
        "moving_joint_mask": [False] * 5 + [True],
        "activation_epoch": 11,
        "received_at": 10.0,
    }
    assert safe_mode(
        changed_position,
        0,
        "hold",
        11,
        feedback,
        10.0,
        2.0,
        10.1,
        last_accepted_hold_target=0.40,
    ) == "brake"
    changed_position["activation_epoch"] = 12
    assert safe_mode(
        changed_position,
        0,
        "hold",
        11,
        feedback,
        10.0,
        2.0,
        10.1,
        last_accepted_hold_target=0.40,
    ) == "position"

    source = CONTROLLER.read_text(encoding="utf-8")
    pre_enable = source.split("            if active and not enabled:", 1)[1].split(
        "            if not active and enabled:", 1
    )[0]
    assert pre_enable.count("safe_pre_enable_mode(") == 3
    assert pre_enable.count(
        "last_accepted_hold_target=last_accepted_hold_target"
    ) == 3
    assert pre_enable.count(
        "highest_rejected_active_epoch=("
    ) == 3


def test_j6_new_runtime_rejection_reasons_remain_distinct():
    reason = load_mode_functions()["command_rejection_reason"]
    for message in (
        "POSITION同一激活纪元目标发生变化",
        "活动命令激活纪元已被拒绝",
    ):
        assert reason(ValueError(message)) == message


def test_j6_rejected_epoch_lease_expiry_preserves_authoritative_target():
    capture = load_mode_functions()["capture_lease_safe_hold_target"]
    rejected_position = {
        "mode": "position",
        "targets_rad": [0.0] * 5 + [0.80],
        "active_joint_mask": [False] * 5 + [True],
        "moving_joint_mask": [False] * 5 + [True],
        "activation_epoch": 8,
        "received_at": 10.0,
    }
    displaced = SimpleNamespace(
        position=1.20,
        velocity=0.0,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    preserved = 0.20
    captured = capture(
        rejected_position,
        True,
        preserved,
        displaced,
        10.59,
        2.0,
        False,
        True,
        True,
        10.60,
        rejected_active_target=preserved,
    )
    assert math.isclose(captured, preserved, abs_tol=1e-12)
    assert not math.isclose(captured, -(displaced.position - 2.0), abs_tol=1e-12)


def test_j6_arrival_timeout_latches_safe_brake_and_keeps_worker_online():
    source = CONTROLLER.read_text(encoding="utf-8")
    timeout_block = source.split(
        "if observe_no_progress_watchdog(", 1
    )[1].split("elif decoded_events:", 1)[0]
    assert "J6_POSITION_ARRIVAL_OVERDUE" in timeout_block
    assert "action=LATCHED_SAFE_BRAKE" in timeout_block
    assert "J6_LOAD_LIMIT_NO_PROGRESS" in timeout_block
    assert 'mode = "brake"' in source
    assert "disable_and_verify(transport, logger)" in source
    assert "worker_continues_online=YES" in source


def test_j6_deadline_guard_detects_sustained_40hz_active_loop():
    update = load_mode_functions()["update_active_deadline_miss_count"]
    count = 0
    for _ in range(3):
        count = update(count, True, 0.025)
    assert count == 3
    assert update(count, True, 0.010) == 0
    assert update(2, False, 0.025) == 0


def test_j6_deadline_guard_does_not_turn_one_stall_into_three_misses():
    update = load_mode_functions()["update_active_deadline_miss_count"]
    count = update(0, True, 0.050)
    assert count == 1
    count = update(count, True, 0.010)
    assert count == 0


def test_j6_deadline_guard_uses_adjacent_loop_start_periods():
    source = CONTROLLER.read_text(encoding="utf-8")
    assert "cycle_interval = cycle_started_at - previous_cycle_started_at" in source
    assert "previous_cycle_enabled" in source
    assert "cycle_elapsed = time.monotonic() - cycle_started_at" not in source
    assert "next_tick = cycle_started_at" in source
    assert "J6_ACTIVE_LOOP_DEGRADED" in source
    assert "J6_ACTIVE_LOOP_RECOVERED" in source
    deadline_block = source.split(
        "active_deadline_miss_count = update_active_deadline_miss_count", 1
    )[1].split("previous_cycle_started_at = cycle_started_at", 1)[0]
    assert "fault_latched = True" not in deadline_block


def test_j6_unsafe_new_hold_does_not_withdraw_existing_position_authority():
    functions = load_mode_functions()
    fallback = functions["rejected_hold_fallback"]

    # The arm was held at A, then POSITION moved toward B. Rejecting an unsafe
    # HOLD C must retain B and its epoch; returning to A would reverse the arm.
    assert fallback(True, "position", 0.40, 11, -0.20, 7) == (0.40, 11)

    # Outside POSITION, the already accepted fixed HOLD remains authoritative.
    assert fallback(True, "hold", None, None, -0.20, 7) == (-0.20, 7)
    assert fallback(False, "position", 0.40, 11, -0.20, 7) == (None, None)

    # A rejected HOLD clears the old moving cap. The first cap of a subsequent
    # valid POSITION can therefore increase by at most one acceleration step.
    q_command, dq_command = functions["fixed_hold_profile_state"](0.40)
    assert (q_command, dq_command) == (0.40, 0.0)
    amax = math.radians(20.0)
    period = 0.01
    first_cap = MODULE.update_posvel_speed_limit(
        dq_command,
        actual_position=0.40,
        target_position=0.60,
        vmax=math.radians(5.0),
        amax=amax,
        restore_limit=math.radians(1.0),
        period=period,
    )
    assert first_cap <= amax * period + 1e-12


def test_j6_rejected_active_epoch_stays_blocked_after_feedback_moves_near_target():
    functions = load_mode_functions()
    rejected = functions["command_epoch_was_rejected"]
    authorized = functions["hold_command_entry_is_authorized"]
    lease_resume = functions["external_command_can_safely_resume_from_safe_hold"]
    command = {
        "mode": "hold",
        "targets_rad": [0.0] * 5 + [0.80],
        "active_joint_mask": [False] * 5 + [True],
        "moving_joint_mask": [False] * 6,
        "activation_epoch": 8,
        "received_at": 10.0,
    }
    # Reference=2 and raw position=1.2 gives logical position +0.8: external
    # motion has brought feedback exactly onto the formerly unsafe C target.
    near_rejected_target = SimpleNamespace(
        position=1.20,
        velocity=0.0,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    assert rejected(command, 8)
    assert not authorized(
        command,
        8,
        "hold",
        7,
        near_rejected_target,
        10.0,
        2.0,
        10.1,
    )
    assert not lease_resume(
        command,
        7,
        0,
        near_rejected_target,
        10.0,
        2.0,
        10.1,
        8,
    )

    # Reusing the rejected epoch for POSITION is blocked as well. A genuinely
    # higher epoch can make a fresh, safety-checked request.
    command["mode"] = "position"
    command["moving_joint_mask"][5] = True
    assert rejected(command, 8)
    assert not lease_resume(
        command,
        7,
        0,
        near_rejected_target,
        10.0,
        2.0,
        10.1,
        8,
    )
    command["activation_epoch"] = 9
    assert not rejected(command, 8)
    assert lease_resume(
        command,
        7,
        0,
        near_rejected_target,
        10.0,
        2.0,
        10.1,
        8,
    )

    command["mode"] = "brake"
    command["active_joint_mask"][5] = False
    assert not rejected(command, 8)


def test_j6_new_position_authority_restarts_unsigned_speed_cap():
    starts = load_mode_functions()["position_command_starts_new_profile"]
    command = {
        "mode": "position",
        "targets_rad": [0.0] * 5 + [-0.40],
        "active_joint_mask": [False] * 5 + [True],
        "moving_joint_mask": [False] * 5 + [True],
        "activation_epoch": 12,
    }
    assert starts(command, "position", 0.40, 11)
    assert starts(command, "hold", -0.40, 12)
    assert not starts(command, "position", -0.40, 12)

    source = CONTROLLER.read_text(encoding="utf-8")
    activation_block = source.split(
        "if position_command_starts_new_profile(", 1
    )[1].split("else:\n                position_started_at", 1)[0]
    assert "dq_command = 0.0" in activation_block
    amax = math.radians(20.0)
    period = 0.01
    first_cap = MODULE.update_posvel_speed_limit(
        0.0,
        actual_position=0.40,
        target_position=-0.40,
        vmax=math.radians(5.0),
        amax=amax,
        restore_limit=math.radians(1.0),
        period=period,
    )
    assert first_cap <= amax * period + 1e-12


def test_j6_position_to_hold_distinguishes_arrival_from_operator_stop():
    functions = load_mode_functions()
    reuses = functions["hold_transition_reuses_position_target"]
    safe = functions["hold_command_entry_is_safe"]
    command = {
        "mode": "hold",
        "targets_rad": [0.0] * 5 + [0.20],
        "active_joint_mask": [False] * 5 + [True],
        "moving_joint_mask": [False] * 6,
        "activation_epoch": 7,
    }
    displaced = SimpleNamespace(
        position=1.60,
        velocity=0.0,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    assert reuses(command, "position", 0.20, 7)
    assert safe(
        command,
        "position",
        None,
        displaced,
        10.0,
        2.0,
        10.1,
        last_position_target=0.20,
        last_position_epoch=7,
    )

    # An operator stop changes the HOLD target to fresh feedback in the same
    # epoch; that safe target must be accepted instead of the old endpoint.
    command["targets_rad"][5] = 0.40
    assert not reuses(command, "position", 0.20, 7)
    assert safe(
        command,
        "position",
        None,
        displaced,
        10.0,
        2.0,
        10.1,
        last_position_target=0.20,
        last_position_epoch=7,
    )
    target, epoch = functions["accept_hold_target"](
        command, "position", None, None
    )
    assert (target, epoch) == (0.40, 7)


def test_j6_effective_mode_requires_fresh_new_epoch_authorization():
    functions = load_mode_functions()
    effective = functions["effective_command_mode"]
    command = {
        "mode": "position",
        "targets_rad": [0.0] * 6,
        "active_joint_mask": [False] * 5 + [True],
        "activation_epoch": 7,
        "received_at": 10.0,
    }
    assert effective(command, 0, 10.1) == "position"
    assert effective(command, 8, 10.1) == "brake"
    assert effective(command, 0, 10.6) == "brake"
    command["active_joint_mask"][5] = False
    assert effective(command, 0, 10.1) == "brake"


def test_j6_position_packet_with_nonmoving_active_axis_is_fixed_hold():
    functions = load_mode_functions()
    effective = functions["effective_command_mode"]
    command = {
        "mode": "position",
        "targets_rad": [0.0] * 5 + [0.04],
        "active_joint_mask": [False] * 5 + [True],
        "moving_joint_mask": [False] * 6,
        "activation_epoch": 7,
        "received_at": 10.0,
    }
    assert effective(command, 0, 10.1) == "hold"
    command["moving_joint_mask"][5] = True
    assert effective(command, 0, 10.1) == "position"
    command.pop("moving_joint_mask")
    assert effective(command, 0, 10.1) == "position"


def test_j6_unfinished_position_lease_expiry_keeps_authorized_endpoint():
    functions = load_mode_functions()
    capture = functions["capture_lease_safe_hold_target"]
    authorized_endpoint = 0.40
    healthy = SimpleNamespace(
        position=1.75,
        velocity=0.02,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    command = {
        "mode": "position",
        "targets_rad": [0.0] * 5 + [authorized_endpoint],
        "active_joint_mask": [False] * 5 + [True],
        "moving_joint_mask": [False] * 5 + [True],
        "activation_epoch": 7,
        "received_at": 10.0,
    }
    target = capture(
        command,
        True,
        None,
        healthy,
        10.59,
        2.0,
        False,
        True,
        True,
        10.60,
    )
    displaced_feedback = -(healthy.position - 2.0)
    assert math.isclose(target, authorized_endpoint, abs_tol=1e-12)
    assert not math.isclose(target, displaced_feedback, abs_tol=1e-12)
    assert capture(
        command, True, None, healthy, 10.59, 2.0, True, True, True, 10.60
    ) is None
    assert capture(
        command, True, None, healthy, 10.40, 2.0, False, True, True, 10.60
    ) is None
    healthy.state = 8
    assert capture(
        command, True, None, healthy, 10.59, 2.0, False, True, True, 10.60
    ) is None


def test_j6_lease_hold_health_does_not_fail_only_for_external_push_velocity():
    healthy_check = load_mode_functions()["enabled_feedback_is_healthy"]
    pushed = SimpleNamespace(
        position=1.75,
        velocity=1.2,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    assert healthy_check(
        pushed, 10.59, 2.0, False, True, True, 10.60
    )


def test_j6_hold_lease_expiry_keeps_last_accepted_target_not_feedback():
    functions = load_mode_functions()
    capture = functions["capture_lease_safe_hold_target"]
    healthy = SimpleNamespace(
        position=1.75,
        velocity=0.02,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    command = {
        "mode": "hold",
        "targets_rad": [0.0] * 5 + [-0.40],
        "active_joint_mask": [False] * 5 + [True],
        "activation_epoch": 7,
        "received_at": 10.0,
    }
    target = capture(
        command,
        True,
        -0.30,
        healthy,
        10.59,
        2.0,
        False,
        True,
        True,
        10.60,
    )
    assert target == -0.30
    assert target != -(healthy.position - 2.0)


def test_j6_nonmoving_position_lease_expiry_keeps_fixed_target_not_feedback():
    functions = load_mode_functions()
    capture = functions["capture_lease_safe_hold_target"]
    healthy = SimpleNamespace(
        position=1.75,
        velocity=0.02,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    command = {
        "mode": "position",
        "targets_rad": [0.0] * 5 + [-0.40],
        "active_joint_mask": [False] * 5 + [True],
        "moving_joint_mask": [False] * 6,
        "activation_epoch": 7,
        "received_at": 10.0,
    }
    target = capture(
        command,
        True,
        -0.30,
        healthy,
        10.59,
        2.0,
        False,
        True,
        True,
        10.60,
    )
    assert target == -0.30
    assert target != -(healthy.position - 2.0)


def test_j6_hold_target_is_accepted_on_entry_and_frozen_within_epoch():
    accept = load_mode_functions()["accept_hold_target"]
    command = {
        "mode": "hold",
        "targets_rad": [0.0] * 5 + [0.40],
        "active_joint_mask": [False] * 5 + [True],
        "activation_epoch": 7,
    }
    target, epoch = accept(command, "position", None, None)
    assert (target, epoch) == (0.40, 7)

    command["targets_rad"][5] = 0.70
    target, epoch = accept(command, "hold", target, epoch)
    assert (target, epoch) == (0.40, 7)

    command["activation_epoch"] = 8
    target, epoch = accept(command, "hold", target, epoch)
    assert (target, epoch) == (0.70, 8)


def test_j6_nonmoving_position_target_is_accepted_once_and_frozen():
    accept = load_mode_functions()["accept_hold_target"]
    command = {
        "mode": "position",
        "targets_rad": [0.0] * 5 + [0.40],
        "active_joint_mask": [False] * 5 + [True],
        "moving_joint_mask": [False] * 6,
        "activation_epoch": 7,
    }
    target, epoch = accept(command, "position", None, None)
    assert (target, epoch) == (0.40, 7)

    command["targets_rad"][5] = 0.70
    target, epoch = accept(command, "hold", target, epoch)
    assert (target, epoch) == (0.40, 7)


def test_j6_same_epoch_position_cannot_replace_a_frozen_hold_endpoint():
    functions = load_mode_functions()
    accept = functions["accept_hold_target"]
    authorize_position = functions["position_command_target_is_authorized"]
    fallback = functions["rejected_hold_fallback"]
    reuses = functions["hold_transition_reuses_position_target"]
    command = {
        "mode": "hold",
        "targets_rad": [0.0] * 5 + [0.40],
        "active_joint_mask": [False] * 5 + [True],
        "moving_joint_mask": [False] * 6,
        "activation_epoch": 11,
    }
    target, epoch = accept(command, "position", None, None)
    assert (target, epoch) == (0.40, 11)

    # HOLD A/E11 -> POSITION B/E11 is rejected before B can be sent.  The
    # current authoritative fallback therefore remains A, so a later HOLD
    # packet can never cause a B->A snapback after real motion to B.
    command["mode"] = "position"
    command["moving_joint_mask"][5] = True
    command["targets_rad"][5] = 0.70
    assert not authorize_position(
        command,
        target,
        epoch,
        None,
        None,
    )
    assert fallback(True, "hold", None, None, target, epoch) == (0.40, 11)

    # B is available only through a new epoch.  Once POSITION B/E12 is the
    # recorded endpoint, automatic HOLD B/E12 adopts that exact target rather
    # than retaining A/E11.
    command["activation_epoch"] = 12
    assert authorize_position(command, target, epoch, None, None)
    assert not authorize_position(command, target, epoch, 0.60, 12)
    command["mode"] = "hold"
    command["moving_joint_mask"][5] = False
    assert reuses(command, "position", 0.70, 12)
    target, epoch = accept(command, "position", target, epoch)
    assert (target, epoch) == (0.70, 12)


def test_j6_new_hold_epoch_must_match_fresh_actual_but_same_epoch_is_frozen():
    functions = load_mode_functions()
    safe = functions["hold_command_entry_is_safe"]
    feedback = SimpleNamespace(
        position=2.0,
        velocity=0.0,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    command = {
        "mode": "hold",
        "targets_rad": [0.0] * 6,
        "active_joint_mask": [False] * 5 + [True],
        "activation_epoch": 8,
    }
    # reference=2.0 makes the current logical angle exactly zero.
    command["targets_rad"][5] = math.radians(5.0)
    assert safe(command, "hold", 7, feedback, 10.0, 2.0, 10.1)
    command["targets_rad"][5] = math.radians(5.01)
    assert not safe(command, "hold", 7, feedback, 10.0, 2.0, 10.1)
    # The accepted target is immutable within epoch 8; a changed packet cannot
    # alter it because accept_hold_target ignores same-epoch target fields.
    assert safe(command, "hold", 8, feedback, 10.0, 2.0, 10.1)


def test_j6_empirical_pre_position_hold_uses_quarter_degree_capture():
    safe = load_mode_functions()["hold_command_entry_is_safe"]
    feedback = SimpleNamespace(
        position=2.0,
        velocity=0.0,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    command = {
        "mode": "hold",
        "targets_rad": [0.0] * 6,
        "active_joint_mask": [False] * 5 + [True],
        "activation_epoch": 8,
        "gravity_authority": {
            "schema": "go-m8010-gravity-command-authority/1.1",
            "authority_class": "EMPIRICAL_VALIDATION_ENVELOPE",
            "empirical_position_validation_authorized": False,
        },
    }
    command["targets_rad"][5] = math.radians(0.25)
    assert safe(command, "brake", 7, feedback, 10.0, 2.0, 10.1)
    command["targets_rad"][5] = math.radians(0.251)
    assert not safe(command, "brake", 7, feedback, 10.0, 2.0, 10.1)
    command["gravity_authority"][
        "empirical_position_validation_authorized"
    ] = True
    command["targets_rad"][5] = math.radians(5.0)
    assert safe(command, "brake", 7, feedback, 10.0, 2.0, 10.1)


def test_j6_active_startup_binding_is_complete_immutable_and_fail_closed():
    functions = load_gravity_startup_binding_functions()
    validate = functions["validate_gravity_authority_startup_binding"]
    enforce = functions["enforce_gravity_authority_startup_binding"]
    arguments = SimpleNamespace(
        expected_gravity_authority_class=(
            "EMPIRICAL_VALIDATION_ENVELOPE"
        ),
        expected_empirical_envelope_id=(
            "v15-31b-empirical-0123456789abcdefabcd"
        ),
        expected_empirical_envelope_sha256="a" * 64,
        expected_gravity_anchor_sha256="b" * 64,
        expected_gravity_session_id=(
            "persistent:0123456789abcdef:"
            "j2session:1111111111111111:"
            "goauxsession:2222222222222222"
        ),
        expected_gravity_state_instance_id="c" * 32,
    )
    binding = validate(arguments)
    functions["EXPECTED_GRAVITY_AUTHORITY_BINDING"] = binding
    packet = {
        "session_id": binding.session_id,
        "state_instance_id": binding.state_instance_id,
        "empirical_envelope_id": binding.empirical_envelope_id,
        "empirical_envelope_sha256": binding.empirical_envelope_sha256,
        "anchor_sha256": binding.anchor_sha256,
    }
    enforce(packet, "EMPIRICAL_VALIDATION_ENVELOPE")

    for field in (
        "session_id",
        "state_instance_id",
        "empirical_envelope_id",
        "empirical_envelope_sha256",
        "anchor_sha256",
    ):
        changed = dict(packet)
        changed[field] = "0" * len(changed[field])
        try:
            enforce(changed, "EMPIRICAL_VALIDATION_ENVELOPE")
        except ValueError:
            pass
        else:
            raise AssertionError(f"J6 accepted changed startup pin: {field}")

    functions["EXPECTED_GRAVITY_AUTHORITY_BINDING"] = binding._replace(
        authority_class="NONE"
    )
    try:
        enforce(packet, "EMPIRICAL_VALIDATION_ENVELOPE")
    except ValueError:
        pass
    else:
        raise AssertionError("J6 accepted authority without a startup permit")

    missing_hash = SimpleNamespace(**vars(arguments))
    missing_hash.expected_empirical_envelope_sha256 = ""
    try:
        validate(missing_hash)
    except RuntimeError:
        pass
    else:
        raise AssertionError("J6 accepted an incomplete empirical startup pin")

    source = CONTROLLER.read_text(encoding="utf-8")
    assert source.count("enforce_gravity_authority_startup_binding(") >= 3
    bind_before_state = source.index(
        "validate_gravity_authority_startup_binding(args)"
    )
    persistent_state = source.index(
        "persistent_reference = load_persistent_zero(args.zero_file)"
    )
    assert bind_before_state < persistent_state


def test_j6_official_startup_binding_preserves_official_schema_path():
    functions = load_gravity_startup_binding_functions()
    validate = functions["validate_gravity_authority_startup_binding"]
    enforce = functions["enforce_gravity_authority_startup_binding"]
    arguments = SimpleNamespace(
        expected_gravity_authority_class="OFFICIAL_CONTINUOUS_RATING",
        expected_empirical_envelope_id="",
        expected_empirical_envelope_sha256="",
        expected_gravity_anchor_sha256="d" * 64,
        expected_gravity_session_id=(
            "persistent:0123456789abcdef:"
            "j2session:1111111111111111:"
            "goauxsession:2222222222222222"
        ),
        expected_gravity_state_instance_id="e" * 32,
    )
    binding = validate(arguments)
    functions["EXPECTED_GRAVITY_AUTHORITY_BINDING"] = binding
    enforce(
        {
            "session_id": binding.session_id,
            "state_instance_id": binding.state_instance_id,
        },
        "OFFICIAL_CONTINUOUS_RATING",
    )
    try:
        enforce(
            {
                "session_id": binding.session_id,
                "state_instance_id": "f" * 32,
            },
            "OFFICIAL_CONTINUOUS_RATING",
        )
    except ValueError:
        pass
    else:
        raise AssertionError("J6 accepted a changed official state binding")


def test_j6_feedback_identity_is_independent_fail_closed_and_tx_inert():
    functions = load_gravity_startup_binding_functions()
    validate_authority = functions[
        "validate_gravity_authority_startup_binding"
    ]
    validate_feedback = functions["validate_feedback_startup_binding"]
    enforce_authority = functions[
        "enforce_gravity_authority_startup_binding"
    ]
    session_id = (
        "persistent:0123456789abcdef:"
        "j2session:1111111111111111:"
        "goauxsession:2222222222222222"
    )
    none_arguments = SimpleNamespace(
        expected_gravity_authority_class="NONE",
        expected_empirical_envelope_id="",
        expected_empirical_envelope_sha256="",
        expected_gravity_anchor_sha256="",
        expected_gravity_session_id="",
        expected_gravity_state_instance_id="",
    )
    none_binding = validate_authority(none_arguments)
    feedback_arguments = SimpleNamespace(
        feedback_session_id=session_id,
        feedback_state_instance_id="a" * 32,
    )
    feedback_binding = validate_feedback(
        feedback_arguments, none_binding
    )
    assert feedback_binding.session_id == session_id
    assert feedback_binding.state_instance_id == "a" * 32
    assert none_binding.authority_class == "NONE"

    functions["EXPECTED_GRAVITY_AUTHORITY_BINDING"] = none_binding
    try:
        enforce_authority(
            {
                "session_id": session_id,
                "state_instance_id": "a" * 32,
            },
            "OFFICIAL_CONTINUOUS_RATING",
        )
    except ValueError:
        pass
    else:
        raise AssertionError("feedback identity granted active gravity authority")

    for bad_session, bad_state in (
        (session_id, ""),
        ("", "a" * 32),
        (session_id.upper(), "a" * 32),
        (session_id, "A" * 32),
        (session_id, "a" * 31),
    ):
        try:
            validate_feedback(
                SimpleNamespace(
                    feedback_session_id=bad_session,
                    feedback_state_instance_id=bad_state,
                ),
                none_binding,
            )
        except RuntimeError:
            pass
        else:
            raise AssertionError("J6 accepted an invalid feedback identity")

    active_arguments = SimpleNamespace(
        expected_gravity_authority_class="OFFICIAL_CONTINUOUS_RATING",
        expected_empirical_envelope_id="",
        expected_empirical_envelope_sha256="",
        expected_gravity_anchor_sha256="b" * 64,
        expected_gravity_session_id=session_id,
        expected_gravity_state_instance_id="c" * 32,
    )
    active_binding = validate_authority(active_arguments)
    matched = validate_feedback(
        SimpleNamespace(
            feedback_session_id=session_id,
            feedback_state_instance_id="c" * 32,
        ),
        active_binding,
    )
    assert matched.session_id == active_binding.session_id
    try:
        validate_feedback(
            SimpleNamespace(
                feedback_session_id=session_id,
                feedback_state_instance_id="d" * 32,
            ),
            active_binding,
        )
    except RuntimeError:
        pass
    else:
        raise AssertionError("J6 accepted split active/feedback identities")

    source = CONTROLLER.read_text(encoding="utf-8")
    validation = source[
        source.index("def validate_feedback_startup_binding(") :
        source.index("\ndef enforce_gravity_authority_startup_binding(")
    ]
    for forbidden_tx in (
        "send_enable(",
        "send_disable(",
        "logger.send(",
        "refresh_request(",
    ):
        assert forbidden_tx not in validation
    assert source.count("transport.send_enable()") == 1
    assert "--feedback-session-id" in source
    assert "--feedback-state-instance-id" in source
    feedback_validation = source.index(
        "EXPECTED_FEEDBACK_IDENTITY_BINDING = validate_feedback_startup_binding("
    )
    persistent_state = source.index(
        "persistent_reference = load_persistent_zero(args.zero_file)"
    )
    assert feedback_validation < persistent_state


def test_j6_feedback_handoff_is_single_use_continuous_and_pre_hardware():
    functions = load_feedback_handoff_functions()
    # Windows cannot represent the production Linux owner/mode contract.  The
    # behavioral test keeps the no-follow reader boundary while the source
    # assertions below preserve that Linux-only contract verbatim.
    functions["_secure_regular_file_bytes"] = (
        lambda path, maximum_bytes, _label: Path(path).read_bytes()
    )
    functions["_fsync_directory"] = lambda _path: None
    binding = SimpleNamespace(
        session_id=(
            "persistent:0123456789abcdef:"
            "j2session:1111111111111111:"
            "goauxsession:2222222222222222"
        ),
        state_instance_id="a" * 32,
    )
    source_id = "b" * 32
    last_sequence = 505
    last_source_ns = 1_000_000
    with tempfile.TemporaryDirectory() as directory:
        root = Path(directory).resolve()
        raw_path = root / "j6_raw.json"
        pending = root / "j6_handoff.json"
        raw_document = {
            "schema": "go-m8010-j6-disabled-raw-capture-statistics/1.0",
            "status": "PASS",
            "packet_count": 500,
            "physical_power_off_required": False,
            "terminal": {
                "required_final_disabled_frames": 5,
                "confirmed": True,
                "channel_closed": True,
            },
            "readonly_feedback_udp": {
                "session_id": binding.session_id,
                "state_instance_id": binding.state_instance_id,
                "source_instance_id": source_id,
                "published_count": last_sequence,
                "last_sequence": last_sequence,
                "last_source_monotonic_ns": last_source_ns,
                "socket_closed": True,
                "active_control_authorized": False,
                "can_tx_policy": "DISABLED_CLASSIC_CAN_REFRESH_ONLY",
            },
            "safety": {
                "active_or_hold_commands_sent": 0,
                "active_commands_sent": 0,
                "hold_commands_sent": 0,
                "forbidden_tx_attempt_count": 0,
            },
        }
        raw_path.write_text(
            json.dumps(raw_document) + "\n", encoding="utf-8"
        )
        handoff = {
            "schema": "go-m8010-j6-feedback-handoff/1.0",
            "handoff_id": f"j6-feedback-{source_id}-{last_sequence}",
            "session_id": binding.session_id,
            "state_instance_id": binding.state_instance_id,
            "source_instance_id": source_id,
            "last_sequence": last_sequence,
            "last_source_monotonic_ns": last_source_ns,
            "raw_capture": {
                "path": str(raw_path),
                "sha256": hashlib.sha256(raw_path.read_bytes()).hexdigest(),
            },
            "terminal": {
                "drive_state": 0,
                "controller_mode": "brake",
                "confirmed": True,
                "channel_closed": True,
            },
            "forwarding": {
                "destination": "127.0.0.1:15300",
                "published_count": last_sequence,
                "socket_closed": True,
            },
            "active_control_authorized": False,
            "can_tx_policy": "DISABLED_CLASSIC_CAN_REFRESH_ONLY",
            "single_use_claim_required": True,
        }
        pending.write_text(json.dumps(handoff) + "\n", encoding="utf-8")
        source, sequence = functions["consume_feedback_handoff"](
            pending, binding, 15300, now_monotonic_ns=last_source_ns + 1
        )
        assert source == source_id
        assert sequence == last_sequence
        assert not pending.exists()
        assert pending.with_name(f"{pending.name}.claimed").is_file()
        try:
            functions["consume_feedback_handoff"](
                pending, binding, 15300,
                now_monotonic_ns=last_source_ns + 2,
            )
        except (RuntimeError, OSError):
            pass
        else:
            raise AssertionError("J6 reused a consumed feedback handoff")

    source_text = CONTROLLER.read_text(encoding="utf-8")
    consumer = source_text[
        source_text.index("def consume_feedback_handoff(") :
        source_text.index("\ndef validate_gravity_authority_startup_binding(")
    ]
    for forbidden_hardware in (
        "RawCanLogger(", "exact_usb_identity(", "logger.send(",
        "send_enable(", "send_disable(",
    ):
        assert forbidden_hardware not in consumer
    assert "os.O_NOFOLLOW" in source_text
    assert "stat.S_IMODE(metadata.st_mode) != 0o600" in source_text
    assert "metadata.st_nlink != 1" in source_text
    consume_at = source_text.index(
        ") = consume_feedback_handoff("
    )
    assert consume_at < source_text.index("exact_usb_identity()", consume_at)
    assert source_text.count("transport.send_enable()") == 1
    assert 'parser.add_argument("--feedback-handoff-file"' in source_text


def test_j6_invalid_feedback_handoff_is_spent_before_validation():
    functions = load_feedback_handoff_functions()
    functions["_secure_regular_file_bytes"] = (
        lambda path, maximum_bytes, _label: Path(path).read_bytes()
    )
    functions["_fsync_directory"] = lambda _path: None
    binding = SimpleNamespace(
        session_id=(
            "persistent:0123456789abcdef:"
            "j2session:1111111111111111:"
            "goauxsession:2222222222222222"
        ),
        state_instance_id="a" * 32,
    )
    with tempfile.TemporaryDirectory() as directory:
        pending = Path(directory).resolve() / "invalid_handoff.json"
        pending.write_text('{"schema":"invalid"}\n', encoding="utf-8")
        try:
            functions["consume_feedback_handoff"](
                pending, binding, 15300, now_monotonic_ns=2_000_000
            )
        except RuntimeError:
            pass
        else:
            raise AssertionError("J6 accepted an invalid feedback handoff")
        assert not pending.exists()
        assert pending.with_name(f"{pending.name}.claimed").is_file()


def test_j6_empirical_deadline_is_checked_each_cycle_and_lease_is_not_captured():
    functions = load_empirical_authority_functions()
    current = functions["empirical_gravity_authority_is_current"]
    command = {
        "gravity_authority": {
            "schema": "go-m8010-gravity-command-authority/1.1",
            "authority_class": "EMPIRICAL_VALIDATION_ENVELOPE",
            "empirical_envelope_deadline_monotonic_ns": 2_000,
            "empirical_envelope_expires_at_utc": "2099-01-01T00:00:00Z",
        }
    }
    wall = datetime(2026, 8, 31, tzinfo=timezone.utc)
    assert current(command, now_monotonic_ns=1_999, now_utc=wall)
    assert not current(command, now_monotonic_ns=2_000, now_utc=wall)

    source = CONTROLLER.read_text(encoding="utf-8")
    lease_capture = source.split(
        "lease_capture_source = command_source_for_lease_capture(", 1
    )[1].split("if captured_target is not None:", 1)[0]
    assert "command_uses_empirical_gravity_authority" in lease_capture
    expiry_to_brake = source.split(
        "empirical_authority_expired = bool(", 1
    )[1].split("rejected_active_command = bool(", 1)[0]
    assert 'or empirical_authority_expired' in expiry_to_brake


def test_j6_pre_enable_rechecks_hold_target_after_each_receive():
    functions = load_mode_functions()
    safe_mode = functions["safe_pre_enable_mode"]
    feedback = SimpleNamespace(
        position=2.0,
        velocity=0.0,
        state=0,
        mos_temp=25,
        coil_temp=26,
    )
    command = {
        "mode": "hold",
        "targets_rad": [0.0] * 5 + [math.radians(6.0)],
        "active_joint_mask": [False] * 5 + [True],
        "activation_epoch": 8,
        "received_at": 10.0,
    }
    assert safe_mode(command, 0, "brake", None, feedback, 10.0, 2.0, 10.1) == "brake"
    command["targets_rad"][5] = math.radians(1.0)
    assert safe_mode(command, 0, "brake", None, feedback, 10.0, 2.0, 10.1) == "hold"


def test_j6_safe_hold_release_and_resume_contract():
    functions = load_mode_functions()
    releases = functions["external_command_explicitly_releases_safe_hold"]
    resumes = functions["external_command_can_resume_from_safe_hold"]
    command = {
        "mode": "position",
        "active_joint_mask": [False] * 5 + [True],
        "activation_epoch": 7,
        "received_at": 10.0,
    }
    assert not resumes(command, 7, 8, 10.1)
    command["activation_epoch"] = 8
    assert resumes(command, 7, 8, 10.1)
    command["mode"] = "drag"
    command["activation_epoch"] = 0
    assert releases(command, 10.1)
    command["mode"] = "brake"
    assert releases(command, 10.1)
    command["received_at"] = 9.0
    assert not releases(command, 10.1)


def test_j6_unsafe_higher_epoch_hold_cannot_exit_lease_safe_hold():
    functions = load_mode_functions()
    can_resume = functions["external_command_can_resume_from_safe_hold"]
    can_safely_resume = functions[
        "external_command_can_safely_resume_from_safe_hold"
    ]
    feedback = SimpleNamespace(
        position=1.0,
        velocity=0.0,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    command = {
        "mode": "hold",
        "targets_rad": [0.0] * 5 + [2.0],
        "active_joint_mask": [False] * 5 + [True],
        "moving_joint_mask": [False] * 6,
        "activation_epoch": 8,
        "received_at": 10.0,
    }
    # reference=2 makes current logical angle C=1 rad.  B=2 rad is a
    # dangerous 1-rad step even though its epoch is newer.
    assert can_resume(command, 7, 8, 10.1)
    assert not can_safely_resume(
        command, 7, 8, feedback, 10.0, 2.0, 10.1
    )
    command["targets_rad"][5] = 1.01
    assert can_safely_resume(
        command, 7, 8, feedback, 10.0, 2.0, 10.1
    )
    command["mode"] = "position"
    command["moving_joint_mask"][5] = True
    command["targets_rad"][5] = 2.0
    assert can_safely_resume(
        command, 7, 8, feedback, 10.0, 2.0, 10.1
    )


def test_j6_unsafe_hold_arriving_at_expiry_captures_previous_position_first():
    functions = load_mode_functions()
    select_source = functions["command_source_for_lease_capture"]
    rejected_epoch = functions["rejected_takeover_epoch_at_lease_boundary"]
    capture = functions["capture_lease_safe_hold_target"]
    feedback = SimpleNamespace(
        position=1.0,
        velocity=0.0,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    previous = {
        "mode": "position",
        "targets_rad": [0.0] * 5 + [-0.35],
        "active_joint_mask": [False] * 5 + [True],
        "moving_joint_mask": [False] * 5 + [True],
        "activation_epoch": 7,
        "received_at": 9.0,
    }
    unsafe = {
        "mode": "hold",
        "targets_rad": [0.0] * 5 + [2.0],
        "active_joint_mask": [False] * 5 + [True],
        "moving_joint_mask": [False] * 6,
        "activation_epoch": 8,
        "received_at": 10.0,
    }
    source = select_source(
        previous, True, unsafe, 8, feedback, 10.0, 2.0, 10.1
    )
    assert source is previous
    highest_rejected = rejected_epoch(
        previous, True, unsafe, 8, feedback, 10.0, 2.0, 10.1
    )
    assert highest_rejected == 8
    # reference=2, position=1 -> measured logical C=1.  Lease expiry must keep
    # restoring the previous authorized endpoint A=-0.35, not adopt C or the
    # rejected incoming HOLD target B=2.0.
    captured = capture(
        source, True, 0.0, feedback, 10.0, 2.0,
        False, True, True, 10.1,
    )
    assert captured == -0.35
    assert captured != -(feedback.position - 2.0)

    # Even if feedback is externally moved onto C before the next cycle, the
    # epoch rejected at the boundary remains fenced and B stays the source.
    feedback_near_unsafe = SimpleNamespace(
        position=0.0,
        velocity=0.0,
        state=1,
        mos_temp=25,
        coil_temp=26,
    )
    assert select_source(
        previous,
        True,
        unsafe,
        8,
        feedback_near_unsafe,
        10.0,
        2.0,
        10.1,
        highest_rejected,
    ) is previous
    assert not functions["external_command_can_safely_resume_from_safe_hold"](
        unsafe,
        7,
        8,
        feedback_near_unsafe,
        10.0,
        2.0,
        10.1,
        highest_rejected,
    )

    safe = dict(unsafe)
    safe["targets_rad"] = [0.0] * 5 + [1.01]
    assert select_source(
        previous, True, safe, 8, feedback, 10.0, 2.0, 10.1
    ) is safe

    release = dict(unsafe)
    release["mode"] = "brake"
    release["active_joint_mask"] = [False] * 6
    assert select_source(
        previous, True, release, 8, feedback, 10.0, 2.0, 10.1
    ) is release


def test_j6_lease_capture_replaces_any_historical_hold_fallback():
    source = CONTROLLER.read_text(encoding="utf-8")
    capture_start = source.index("                if captured_target is not None:")
    capture_end = source.index("            mode = (", capture_start)
    capture = source[capture_start:capture_end]
    assert "last_accepted_hold_target = captured_target" in capture
    assert "last_accepted_hold_epoch = lease_safe_hold_source_epoch" in capture

    resume_start = source.index(
        "                elif external_command_can_resume_from_safe_hold("
    )
    resume_end = source.index("            if not lease_safe_hold_active:", resume_start)
    resume = source[resume_start:resume_end]
    assert resume.index("external_command_can_safely_resume_from_safe_hold(") < resume.index(
        "lease_safe_hold_active = False"
    )


def test_j6_feedback_reports_lease_safe_hold_state():
    send_feedback = load_send_feedback()

    class FakeSocket:
        def __init__(self):
            self.sent = []

        def sendto(self, payload, destination):
            self.sent.append((payload, destination))

    fake_socket = FakeSocket()
    decoded = SimpleNamespace(
        position=1.0,
        velocity=0.0,
        mos_temp=25,
        coil_temp=26,
        state=1,
    )
    rejection_state = {
        "total": 3,
        "last_reason": "JSON格式无效",
        "by_reason": {"JSON格式无效": 3},
        "pending_by_reason": {"JSON格式无效": 2},
    }
    send_feedback(
        fake_socket,
        15300,
        decoded,
        True,
        "hold",
        False,
        True,
        rejection_state,
        last_valid_feedback_monotonic_ns=123456789,
    )
    payload = json.loads(fake_socket.sent[0][0])
    assert payload["controller_mode"] == "hold"
    assert payload["source_instance_id"] == "9" * 32
    assert payload["sequence"] == 1
    assert payload["session_id"].startswith("persistent:")
    assert payload["state_instance_id"] == "8" * 32
    assert payload["lease_safe_hold"] is True
    assert payload["rejected_commands"] == 3
    assert payload["last_rejection_reason"] == "JSON格式无效"
    assert payload["suppressed_rejection_logs_pending"] == {"JSON格式无效": 2}
    assert payload["trajectory_plan_token_id"] == ""
    assert payload["trajectory_sha256"] == ""
    assert payload["trajectory_state"] == "INACTIVE"
    assert payload["trajectory_sample_index"] == 0
    assert payload["trajectory_interval_count"] == 0
    assert payload["samples"][0]["trajectory_state"] == "INACTIVE"
    assert payload["samples"][0]["trajectory_plan_token_id"] == ""
    assert payload["samples"][0]["tau_cmd_rotor_nm"] is None
    assert payload["samples"][0]["tau_feedback_rotor_nm"] is None
    assert payload["samples"][0]["tau_joint_estimated_nm"] is None
    assert payload["tau_j2_logical_total_nm"] is None
    assert (
        payload["samples"][0]["last_valid_feedback_monotonic_ns"]
        == 123456789
    )


def test_j6_feedback_reports_thermal_and_no_progress_latches_consistently():
    send_feedback = load_send_feedback()

    class FakeSocket:
        def __init__(self):
            self.sent = []

        def sendto(self, payload, destination):
            self.sent.append((payload, destination))

    socket = FakeSocket()
    decoded = SimpleNamespace(
        position=0.1,
        velocity=0.0,
        mos_temp=54,
        coil_temp=56,
        state=0,
    )
    thermal = {
        "thermal_state": "DERATING",
        "thermal_derating_factor": 0.8,
        "thermal_raw_temperature_c": 56.0,
        "thermal_window_median_c": 55.0,
        "thermal_slope_c_per_min": 0.25,
        "thermal_fault_latched": True,
        "thermal_cooldown_ready": False,
        "thermal_release_observed": False,
        "thermal_rearm_pending_next_cycle": False,
        "thermal_cooldown_valid_brake_frames": 12,
        "thermal_trip_activation_epoch": 7,
        "thermal_minimum_rearm_epoch": 8,
    }
    no_progress = {
        "no_progress_fault": True,
        "no_progress_release_observed": True,
        "no_progress_rearm_pending_next_cycle": False,
        "no_progress_watchdog_qualifying_frames": 9001,
        "no_progress_observation_valid": False,
        "no_progress_position_error_rad": 0.0,
        "no_progress_trip_position_error_rad": 0.2,
        "load_limit_watchdog_authority": (
            "J6_TARGET_TIMEOUT_POSITION_ERROR_V1"
        ),
        "no_progress_trip_activation_epoch": 7,
        "no_progress_minimum_rearm_epoch": 8,
    }
    send_feedback(
        socket, 15300, decoded, True, "brake", False, False,
        None, None, thermal, no_progress, 987654321,
    )
    payload = json.loads(socket.sent[0][0])
    assert payload["domain_fault"] is True
    assert payload["thermal_fault_latched"] is True
    assert payload["no_progress_fault"] is True
    assert payload["load_limit_no_progress"] is True
    assert payload["thermal_derating_factor"] == 0.8
    assert payload["no_progress_minimum_rearm_epoch"] == 8
    assert payload["samples"][0]["thermal_fault_latched"] is True
    assert payload["samples"][0]["load_limit_no_progress"] is True
    assert payload["samples"][0]["last_valid_feedback_monotonic_ns"] == 987654321


def test_j6_feedback_echoes_v13_token_hash_state_index_and_grid_size():
    parse_command = load_parse_command()
    feedback_status = load_quintic_controller_functions()[
        "trajectory_feedback_status"
    ]
    send_feedback = load_send_feedback()
    received_ns = 70_000_000_000
    execute_at_ns = received_ns + 100_000_000
    command = parse_command(
        json.dumps(quintic_command_document(
            received_monotonic_ns=received_ns,
            execute_at_monotonic_ns=execute_at_ns,
        )).encode(),
        received_ns,
    )

    prepared = feedback_status(command, execute_at_ns - 1)
    running = feedback_status(command, execute_at_ns + 500_000_000)
    complete = feedback_status(command, execute_at_ns + 1_000_000_000)
    assert prepared["trajectory_state"] == "PREPARED"
    assert prepared["trajectory_sample_index"] == 0
    assert running["trajectory_state"] == "RUNNING"
    assert running["trajectory_sample_index"] == 50
    assert complete["trajectory_state"] == "COMPLETE"
    assert complete["trajectory_sample_index"] == 100
    for status in (prepared, running, complete):
        assert status["trajectory_plan_token_id"] == "a" * 64
        assert status["trajectory_sha256"] == "b" * 64
        assert status["trajectory_interval_count"] == 100

    class FakeSocket:
        def __init__(self):
            self.sent = []

        def sendto(self, payload, destination):
            self.sent.append((payload, destination))

    fake_socket = FakeSocket()
    send_feedback(
        fake_socket,
        15300,
        None,
        False,
        "position",
        False,
        False,
        None,
        running,
    )
    payload = json.loads(fake_socket.sent[0][0])
    assert payload["trajectory_plan_token_id"] == "a" * 64
    assert payload["trajectory_sha256"] == "b" * 64
    assert payload["trajectory_state"] == "RUNNING"
    assert payload["trajectory_sample_index"] == 50
    assert payload["trajectory_interval_count"] == 100
    assert payload["samples"][0]["trajectory_plan_token_id"] == "a" * 64
    assert payload["samples"][0]["trajectory_sha256"] == "b" * 64
    assert payload["samples"][0]["trajectory_state"] == "RUNNING"
    assert payload["samples"][0]["trajectory_sample_index"] == 50
    assert payload["samples"][0]["trajectory_interval_count"] == 100


def test_j6_control_loop_refreshes_captured_safe_hold_without_reprofiling():
    source = CONTROLLER.read_text(encoding="utf-8")
    assert "q_command = captured_target" in source
    assert "reference - fixed_hold_target" in source
    assert "q_command, dq_command = fixed_hold_profile_state(" in source
    assert '"GUI_LEASE_SAFE_HOLD_REFRESH"' in source
    assert '"hold"\n                if lease_safe_hold_active' in source


def test_j6_brake_packet_revokes_same_epoch_even_if_followed_by_active_packet():
    functions = load_mode_functions()
    observe = functions["observe_valid_command_epoch"]
    effective = functions["effective_command_mode"]
    active = {
        "mode": "position",
        "active_joint_mask": [False] * 5 + [True],
        "activation_epoch": 7,
        "received_at": 10.0,
    }
    brake = {
        "mode": "brake",
        "active_joint_mask": [False] * 6,
        "activation_epoch": 0,
        "received_at": 10.0,
    }
    minimum, last_seen = observe(active, 0, 0)
    minimum, last_seen = observe(brake, minimum, last_seen)
    assert (minimum, last_seen) == (8, 7)
    assert effective(active, minimum, 10.1) == "brake"
    active["activation_epoch"] = 8
    assert effective(active, minimum, 10.1) == "position"


def test_j6_drag_or_domain_deselect_fences_same_epoch_backlog():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]
    active_document = json.loads(command_payload())
    active_document["activation_epoch"] = 7
    active = json.dumps(active_document).encode()

    class FakeSocket:
        def __init__(self, packets):
            self.packets = list(packets)

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    for release_mode, release_mask in (
        ("drag", [False] * 5 + [True]),
        ("hold", [False] * 6),
    ):
        release = dict(active_document)
        release["mode"] = release_mode
        release["active_joint_mask"] = release_mask
        current, minimum, last_seen = receive_latest(
            FakeSocket([json.dumps(release).encode(), active]), None, 0, 0
        )
        assert current["mode"] == release_mode
        assert current["active_joint_mask"] == release_mask
        assert minimum == 8
        assert last_seen == 7


def test_j6_rejection_logging_counts_every_packet_and_aggregates_by_reason():
    functions = load_mode_functions()
    make_state = functions["make_command_rejection_state"]
    record = functions["record_command_rejection"]
    flush = functions["flush_command_rejection_reports"]
    state = make_state()

    first = record(state, ValueError("J6目标超出模型机械限位"), now=0.0)
    assert len(first) == 1 and first[0]["initial"]
    for index in range(100):
        assert record(
            state, ValueError("J6目标超出模型机械限位"), now=0.001 * (index + 1)
        ) == []
    assert state["total"] == 101
    assert state["last_reason"] == "J6目标超出模型机械限位"
    assert state["by_reason"] == {"J6目标超出模型机械限位": 101}

    summary = flush(state, now=5.0)
    assert len(summary) == 1
    assert summary[0]["suppressed_since_last"] == 100
    assert summary[0]["reason_count"] == 101
    assert summary[0]["rejected_total"] == 101


def test_j6_receive_latest_does_not_print_every_invalid_packet():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]
    state = functions["make_command_rejection_state"]()

    class FakeSocket:
        def __init__(self):
            self.packets = [b"{"] * 100

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    output = io.StringIO()
    with redirect_stdout(output):
        current, minimum, last_seen = receive_latest(
            FakeSocket(), None, 0, 0, state
        )
    assert (current, minimum, last_seen) == (None, 0, 0)
    assert state["total"] == 100
    assert state["last_reason"] == "JSON格式无效"
    assert len(output.getvalue().strip().splitlines()) == 1


def test_j6_receive_latest_has_a_finite_per_cycle_packet_budget():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]
    state = functions["make_command_rejection_state"]()

    class FakeSocket:
        def __init__(self):
            self.packets = [b"{"] * 200

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    fake_socket = FakeSocket()
    with redirect_stdout(io.StringIO()):
        current, minimum, last_seen = receive_latest(
            fake_socket, None, 0, 0, state
        )
    assert (current, minimum, last_seen) == (None, 0, 0)
    assert state["total"] == 128
    assert len(fake_socket.packets) == 72


def test_j6_out_of_range_rejection_preserves_existing_position_or_hold_without_brake():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]

    class FakeSocket:
        def __init__(self, packets):
            self.packets = list(packets)

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    for mode in ("position", "hold"):
        source_id = "abcdef0123456789abcdef0123456789"
        accepted = json.loads(command_payload(
            source_instance_id=source_id,
            source_monotonic_ns=time.monotonic_ns(),
            sequence=1,
        ))
        accepted["mode"] = mode
        accepted["targets_rad"][5] = math.radians(100.0)
        accepted["moving_joint_mask"] = (
            [False] * 5 + [True] if mode == "position" else [False] * 6
        )
        rejected = dict(accepted)
        rejected["targets_rad"] = list(accepted["targets_rad"])
        rejected["targets_rad"][5] = math.radians(180.01)
        rejected["sequence"] = 2
        rejected["source_monotonic_ns"] = time.monotonic_ns()
        rejection_state = functions["make_command_rejection_state"]()
        replay_state = functions["make_command_source_replay_state"]()
        with redirect_stdout(io.StringIO()):
            current, _minimum, _last_seen = receive_latest(
                FakeSocket([
                    json.dumps(accepted).encode(),
                    json.dumps(rejected).encode(),
                ]),
                None,
                0,
                0,
                rejection_state,
                replay_state,
            )
        assert current is not None
        assert current["mode"] == mode
        assert current["mode"] != "brake"
        assert math.isclose(current["targets_rad"][5], math.radians(100.0))
        assert rejection_state["last_reason"] == "J6目标超出模型机械限位"


def test_j6_rejects_older_active_epoch_but_always_accepts_brake():
    functions = load_mode_functions()
    acceptable = functions["command_epoch_is_acceptable"]
    command = {
        "mode": "position",
        "active_joint_mask": [False] * 5 + [True],
        "activation_epoch": 100,
    }
    assert not acceptable(command, 101)
    command["activation_epoch"] = 101
    assert acceptable(command, 101)
    command["mode"] = "brake"
    command["activation_epoch"] = 0
    assert acceptable(command, 101)
    command["mode"] = "drag"
    assert acceptable(command, 101, 102)


def test_j6_interarrival_gap_revokes_same_epoch_before_timestamp_overwrite():
    functions = load_mode_functions()
    observe_gap = functions["minimum_epoch_after_interarrival_lease"]
    active = {
        "mode": "position",
        "active_joint_mask": [False] * 5 + [True],
        "activation_epoch": 21,
        "received_at": 10.0,
    }
    assert observe_gap(active, 0, 10.49) == 0
    assert observe_gap(active, 0, 10.501) == 22
    active["mode"] = "brake"
    assert observe_gap(active, 0, 11.0) == 0


def test_j6_same_epoch_packet_after_lease_gap_cannot_replace_stale_source():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]
    stale = json.loads(command_payload())
    stale["activation_epoch"] = 21
    stale["received_at"] = 0.0
    replay = dict(stale)
    replay.pop("received_at")

    class FakeSocket:
        def __init__(self, packets):
            self.packets = list(packets)

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    current, minimum, last_seen = receive_latest(
        FakeSocket([json.dumps(replay).encode()]), stale, 0, 21
    )
    assert current is stale
    assert minimum == 22
    assert last_seen == 21


def test_j6_receive_rejects_duplicate_or_reverse_source_sequence_and_timestamp():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]
    replay_state = functions["make_command_source_replay_state"]()
    rejection_state = functions["make_command_rejection_state"]()
    source_id = "0123456789abcdef0123456789abcdef"
    base_ns = time.monotonic_ns() - 10_000_000

    def packet(sequence, source_ns):
        return command_payload(
            source_instance_id=source_id,
            source_monotonic_ns=source_ns,
            sequence=sequence,
        )

    packets = [
        packet(2, base_ns),
        packet(2, base_ns + 1),
        packet(1, base_ns + 2),
        packet(3, base_ns),
        packet(3, base_ns + 3),
    ]

    class FakeSocket:
        def __init__(self, queued):
            self.packets = list(queued)

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    with redirect_stdout(io.StringIO()):
        current, minimum, last_seen = receive_latest(
            FakeSocket(packets),
            None,
            0,
            0,
            rejection_state,
            replay_state,
        )
    assert current["sequence"] == 3
    assert current["source_monotonic_ns"] == base_ns + 3
    assert (minimum, last_seen) == (0, 1)
    assert rejection_state["total"] == 3
    assert replay_state["sources"][source_id] == {
        "sequence": 3,
        "source_monotonic_ns": base_ns + 3,
    }


def test_j6_receive_locks_live_source_and_allows_new_source_after_lease():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]
    replay_state = functions["make_command_source_replay_state"]()
    rejection_state = functions["make_command_rejection_state"]()
    first_source = "0123456789abcdef0123456789abcdef"
    second_source = "fedcba9876543210fedcba9876543210"

    class FakeSocket:
        def __init__(self, packet):
            self.packets = [packet]

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    first = command_payload(source_instance_id=first_source)
    current, minimum, last_seen = receive_latest(
        FakeSocket(first), None, 0, 0, rejection_state, replay_state
    )
    blocked = command_payload(source_instance_id=second_source)
    with redirect_stdout(io.StringIO()):
        current, minimum, last_seen = receive_latest(
            FakeSocket(blocked),
            current,
            minimum,
            last_seen,
            rejection_state,
            replay_state,
        )
    assert current["source_instance_id"] == first_source
    assert rejection_state["last_reason"] == "当前命令来源租约仍有效"

    current["received_at"] = time.monotonic() - 0.501
    replay_state[
        "active_source_last_received_monotonic_ns"
    ] = time.monotonic_ns() - 500_000_001
    takeover = json.loads(command_payload(
        source_instance_id=second_source,
        sequence=1,
    ))
    takeover["activation_epoch"] = 2
    current, minimum, last_seen = receive_latest(
        FakeSocket(json.dumps(takeover).encode()),
        current,
        minimum,
        last_seen,
        rejection_state,
        replay_state,
    )
    assert current["source_instance_id"] == second_source
    assert current["activation_epoch"] == 2
    assert (minimum, last_seen) == (2, 2)


def test_j6_rejected_command_does_not_poison_source_replay_state():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]
    replay_state = functions["make_command_source_replay_state"]()
    rejection_state = functions["make_command_rejection_state"]()
    source_id = "fedcba9876543210fedcba9876543210"
    base_ns = time.monotonic_ns() - 10_000_000

    bad_target = json.loads(command_payload(
        source_instance_id=source_id,
        source_monotonic_ns=base_ns + 3,
        sequence=99,
    ))
    bad_target["targets_rad"][5] = math.radians(180.1)
    bad_epoch = json.loads(command_payload(
        source_instance_id=source_id,
        source_monotonic_ns=base_ns + 2,
        sequence=98,
    ))
    bad_epoch["activation_epoch"] = 0
    valid = command_payload(
        source_instance_id=source_id,
        source_monotonic_ns=base_ns + 1,
        sequence=1,
    )

    class FakeSocket:
        def __init__(self):
            self.packets = [
                json.dumps(bad_target).encode(),
                json.dumps(bad_epoch).encode(),
                valid,
            ]

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    with redirect_stdout(io.StringIO()):
        current, minimum, last_seen = receive_latest(
            FakeSocket(), None, 0, 0, rejection_state, replay_state
        )
    assert current["sequence"] == 1
    assert (minimum, last_seen) == (0, 1)
    assert rejection_state["total"] == 2
    assert replay_state["sources"][source_id]["sequence"] == 1
    assert replay_state["sources"][source_id]["source_monotonic_ns"] == base_ns + 1


def test_j6_source_replay_state_is_bounded_to_32_recent_sources():
    functions = load_command_channel_functions()
    replay_state = functions["make_command_source_replay_state"]()
    is_newer = functions["command_source_is_newer"]
    commit = functions["commit_command_source"]
    for index in range(33):
        received_ns = (index + 1) * 500_000_001
        command = {
            "mode": "position",
            "source_instance_id": f"{index:032x}",
            "source_monotonic_ns": received_ns,
            "sequence": 1,
        }
        assert is_newer(command, replay_state, received_ns)
        commit(replay_state, command, received_ns)
    assert len(replay_state["sources"]) == 32
    assert f"{0:032x}" not in replay_state["sources"]
    assert f"{1:032x}" in replay_state["sources"]
    assert replay_state["active_source_instance_id"] == f"{32:032x}"


def test_j6_position_execution_authority_is_bounded_to_32_recent_bindings():
    functions = load_command_channel_functions()
    replay_state = functions["make_command_source_replay_state"]()
    validate = functions["validate_position_execution_transition"]
    commit = functions["commit_position_execution_binding"]

    for index in range(33):
        command = {
            "schema": "go-m8010-gui-command/1.2",
            "mode": "position",
            "targets_rad": [0.0] * 5 + [0.01],
            "active_joint_mask": [False] * 5 + [True],
            "moving_joint_mask": [False] * 5 + [True],
            "activation_epoch": 1,
            "maximum_velocity_rad_s": math.radians(5.0),
            "maximum_acceleration_rad_s2": math.radians(20.0),
            "source_instance_id": f"{index:032x}",
        }
        pending = validate(None, command, index, replay_state)
        commit(replay_state, pending)

    bindings = replay_state["position_execution_bindings"]
    assert len(bindings) == 32
    assert (f"{0:032x}", 1) not in bindings
    assert (f"{1:032x}", 1) in bindings
    assert (f"{32:032x}", 1) in bindings


def test_j6_active_source_blocks_takeover_for_500ms_then_allows_it():
    functions = load_command_channel_functions()
    replay_state = functions["make_command_source_replay_state"]()
    is_newer = functions["command_source_is_newer"]
    commit = functions["commit_command_source"]
    first = {
        "mode": "position",
        "source_instance_id": "0123456789abcdef0123456789abcdef",
        "source_monotonic_ns": 1_000_000_000,
        "sequence": 1,
    }
    second = {
        "mode": "position",
        "source_instance_id": "fedcba9876543210fedcba9876543210",
        "source_monotonic_ns": 1_500_000_001,
        "sequence": 1,
    }
    assert is_newer(first, replay_state, 1_000_000_000)
    commit(replay_state, first, 1_000_000_000)
    assert not is_newer(second, replay_state, 1_500_000_000)
    assert replay_state["active_source_instance_id"] == first["source_instance_id"]
    assert is_newer(second, replay_state, 1_500_000_001)
    commit(replay_state, second, 1_500_000_001)
    assert replay_state["active_source_instance_id"] == second["source_instance_id"]


def test_j6_new_source_takeover_requires_strictly_higher_epoch():
    functions = load_command_channel_functions()
    fresh = functions["command_source_takeover_epoch_is_fresh"]
    replay_state = functions["make_command_source_replay_state"]()
    replay_state["active_source_instance_id"] = "0" * 32
    command = {
        "mode": "position",
        "active_joint_mask": [False] * 5 + [True],
        "source_instance_id": "1" * 32,
        "activation_epoch": 7,
    }
    assert not fresh(command, replay_state, 7)
    command["activation_epoch"] = 8
    assert fresh(command, replay_state, 7)
    command["source_instance_id"] = "0" * 32
    command["activation_epoch"] = 7
    assert fresh(command, replay_state, 7)


def test_j6_brake_bypasses_and_does_not_mutate_source_replay_state():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]
    replay_state = functions["make_command_source_replay_state"]()
    source_id = "0123456789abcdef0123456789abcdef"
    replay_state["sources"][source_id] = {
        "sequence": 7,
        "source_monotonic_ns": 123456,
    }
    replay_state["active_source_instance_id"] = source_id
    replay_state["active_source_last_received_monotonic_ns"] = 456789
    binding_key = (source_id, 7)
    replay_state["position_execution_bindings"][binding_key] = (
        "frozen-position-contract",
    )
    state_before_brake = {
        "sources": {source_id: dict(replay_state["sources"][source_id])},
        "position_execution_bindings": {
            binding_key: ("frozen-position-contract",)
        },
        "active_source_instance_id": source_id,
        "active_source_last_received_monotonic_ns": 456789,
        "empirical_authority_binding": None,
        "empirical_active_sha256": None,
        "empirical_spent_sha256": {},
    }
    brake = json.loads(command_payload(schema="go-m8010-gui-command/1.0"))
    brake["mode"] = "brake"
    brake.pop("active_joint_mask")
    brake["source_instance_id"] = source_id
    brake["source_monotonic_ns"] = False
    brake["sequence"] = -1

    class FakeSocket:
        def __init__(self):
            self.packets = [json.dumps(brake).encode()]

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    current, _minimum, _last_seen = receive_latest(
        FakeSocket(), None, 0, 0, None, replay_state
    )
    assert current["mode"] == "brake"
    assert replay_state == state_before_brake


def test_j6_drains_large_backlog_and_hard_stop_fences_unseen_active_packet():
    functions = load_command_channel_functions()
    receive_latest = functions["receive_latest"]
    active = command_payload()
    brake_document = json.loads(command_payload())
    brake_document["mode"] = "brake"
    brake_document["active_joint_mask"] = [False] * 6
    brake_document["activation_epoch"] = (1 << 63) - 1
    brake = json.dumps(brake_document).encode()

    class FakeSocket:
        def __init__(self, packets):
            self.packets = list(packets)

        def recv(self, _size):
            if not self.packets:
                raise BlockingIOError
            return self.packets.pop(0)

    fake_socket = FakeSocket([active] * 100 + [brake, active])
    current, minimum, last_seen = receive_latest(fake_socket, None, 0, 0)
    assert fake_socket.packets == []
    assert current["mode"] == "brake"
    assert minimum == 1 << 63
    assert last_seen == (1 << 63) - 1


def test_external_stop_uses_session_terminal_activation_epoch_fence():
    source = STOP_TOOL.read_text(encoding="utf-8")
    assert '"activation_epoch": (1 << 63) - 1' in source

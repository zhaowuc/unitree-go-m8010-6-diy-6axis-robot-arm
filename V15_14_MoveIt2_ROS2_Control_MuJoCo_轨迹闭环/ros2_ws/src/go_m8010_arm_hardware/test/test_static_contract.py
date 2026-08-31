import ast
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


PACKAGE = Path(__file__).resolve().parents[1] / "go_m8010_arm_hardware"
REPOSITORY = next(
    parent for parent in Path(__file__).resolve().parents
    if (parent / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp").is_file()
)


class StaticContractTest(unittest.TestCase):
    def test_state_node_has_no_command_or_motion_api(self):
        source = (PACKAGE / "whole_arm_state_node.py").read_text(encoding="utf-8")
        forbidden = ("create_client", "create_service", "ActionServer", "SerialPort", "commandJoint", "enterFoc")
        for token in forbidden:
            self.assertNotIn(token, source)
        ast.parse(source)

    def test_mirror_is_direct_qpos_and_forward(self):
        source = (PACKAGE / "mujoco_mirror_node.py").read_text(encoding="utf-8")
        self.assertIn("self.data.qpos[self.qpos_addresses] = target", source)
        self.assertIn("self.mujoco.mj_forward", source)
        for token in ("mj_step", "ctrl[", "PID"):
            self.assertNotIn(token, source)

    def test_gui_component_exit_does_not_cascade_to_all_hardware_domains(self):
        launch_file = (
            REPOSITORY
            / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
            / "ros2_ws"
            / "src"
            / "go_m8010_arm_gui"
            / "launch"
            / "arm_gui.launch.py"
        )
        source = launch_file.read_text(encoding="utf-8")
        self.assertIn("RegisterEventHandler(OnProcessExit", source)
        self.assertIn("LogInfo", source)
        self.assertNotIn("LogError", source)
        self.assertNotIn("EmitEvent(event=Shutdown", source)

    def test_go_worker_enforces_per_motor_active_mask_across_full_position_path(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            'go-m8010-gui-command/1.1',
            "candidate.active_joint_mask[i]",
            "if (!control_command.active_joint_mask[index] ||",
            "control_command.active_joint_mask[",
            'controller_mode_by_motor[motor.name]',
        ):
            self.assertIn(token, source)

    def test_j2_serial_is_brake_qualified_before_any_feedback_is_published(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            "kJ2StartupPrimeMaximumInvalidPrefixPairs = 3",
            "kJ2StartupPrimeRequiredHealthyPairs = 5",
            "kJ2StartupPrimeMaximumElapsedNs = 500000000ULL",
            "is_cold_no_reply_feedback",
            "send_recv_returned",
            "send_recv_threw",
            "classify_j2_startup_prime_pair",
            "prime_j2_serial_before_feedback",
            "J2_STARTUP_BRAKE_PRIME_INVALID_AFTER_HEALTHY",
            "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_TOO_LONG",
            "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_SIGNATURE",
            "J2_STARTUP_BRAKE_PRIME_NOT_STATIONARY",
            "J2_STARTUP_BRAKE_PRIME_STATE=",
            "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS=",
            "J2_STARTUP_BRAKE_PRIME_HEALTHY_PAIRS=",
        ):
            self.assertIn(token, source)
        prime_call = source.index(
            "prime_j2_serial_before_feedback(serial, motors, tx_audit)"
        )
        feedback_socket = source.index(
            "const int feedback_socket = ::socket", prime_call
        )
        main_feedback_send = source.index(
            "(void)::sendto(feedback_socket", feedback_socket
        )
        self.assertLess(prime_call, feedback_socket)
        self.assertLess(feedback_socket, main_feedback_send)

    def test_go_worker_thermal_stop_is_latched_and_requires_two_phase_rearm(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            "constexpr char kThermalConfigSha256[]",
            "ThermalPolicy g_thermal_policy",
            "parse_thermal_policy_yaml",
            "read_secure_owned_policy_file",
            '"THERMAL_CONFIG_EXPECTED_SHA256_MISMATCH"',
            '"THERMAL_CONFIG_SHA256_MISMATCH"',
            "g_thermal_policy = parse_thermal_policy_yaml",
            "g_thermal_policy.thermal_stop_c",
            "g_thermal_policy.derating_start_c",
            "g_thermal_policy.cooldown_seconds",
            "kThermalCooldownConsecutiveFrames = 500",
            "observe_raw_temperature_thermal_trip",
            "all_domain_motors_thermal_cooldown_qualified",
            "observe_thermal_cooldown_frame",
            "observe_explicit_thermal_release",
            "request_thermal_rearm_for_next_cycle",
            "apply_pending_thermal_rearm_at_cycle_start",
            "thermal_derating_factor_for_raw_temperature",
            "thermal_derating_self_test();",
            "thermally_derated_command.vmax *= thermal_derating_factor",
            "thermally_derated_command.amax *= thermal_derating_factor",
            "thermally_derated_command.kp[joint] *= thermal_derating_factor",
            "thermally_derated_command.kd[joint] *= thermal_derating_factor",
            "if (!control_command.quintic.present)",
            "dq_command[index] *= thermal_derating_factor",
            '"EXACT_TRAJECTORY_DERATING_ABORT"',
            '"thermal_derating_factor"',
            "activation_epoch > state.trip_activation_epoch",
            "activation_epoch >= state.minimum_rearm_epoch",
            "activation_epoch > highest_rejected_active_epoch",
            "thermal_interlock_self_test();",
            '"THERMAL_TRIP"',
            '"THERMAL_COOLDOWN_READY"',
            '"THERMAL_RELEASE_OBSERVED"',
            '"THERMAL_REARM_PENDING_NEXT_CYCLE"',
            '"thermal_fault", thermal_interlock.fault_latched',
            '"thermal_minimum_rearm_epoch"',
            '"thermal_cooldown_valid_brake_frames"',
            '"THERMAL_59_AUTO_RECOVERY_SELF_TEST_FAILED"',
            '"THERMAL_FIVE_SECOND_COOLDOWN_SELF_TEST_FAILED"',
            '"THERMAL_30_SECOND_COOLDOWN_SELF_TEST_FAILED"',
            '"THERMAL_NEXT_CYCLE_REARM_SELF_TEST_FAILED"',
        ):
            self.assertIn(token, source)
        thermal_clear = source.split(
            "bool apply_pending_thermal_rearm_at_cycle_start", 1
        )[1].split("void reset_no_progress_observation", 1)[0]
        self.assertEqual(thermal_clear.count("state.fault_latched = false;"), 1)

        cooldown_domain = source.split(
            "bool all_domain_motors_thermal_cooldown_qualified", 1
        )[1].split("void publish_thermal_latch_to_motors", 1)[0]
        for token in (
            "motor.last_frame_valid && motor.valid",
            "motor.merror == 0",
            "motor.returned_mode == kBrakeMode",
            "motor.temperature < g_thermal_policy.rearm_below_c",
        ):
            self.assertIn(token, cooldown_domain)
        release_policy = source.split(
            "bool observe_explicit_thermal_release", 1
        )[1].split("bool request_thermal_rearm_for_next_cycle", 1)[0]
        self.assertIn("!state.cooldown_ready", release_policy)

        runtime = source.split("int run(const Options& options)", 1)[1]
        trip = runtime.split(
            "const bool newly_latched = observe_raw_temperature_thermal_trip", 1
        )[1].split("if (feedback.continuity_valid)", 1)[0]
        for token in (
            "feedback.data.temp >= g_thermal_policy.thermal_stop_c",
            "publish_thermal_latch_to_motors(motors, true)",
            "motor_domain_brake_this_cycle = true",
            'effective_mode = "brake"',
            "position_tracking.fill(false)",
            "reset_bounded_hold_integral(state)",
        ):
            self.assertIn(token, trip)
        self.assertIn("!thermal_interlock.fault_latched", runtime)

        thermal_config_gate = runtime.index(
            "options.expected_thermal_config_sha256 != kThermalConfigSha256"
        )
        thermal_config_read = runtime.index(
            "read_secure_owned_policy_file", thermal_config_gate
        )
        bus_definition = runtime.index(
            "const BusDefinition definition", thermal_config_read
        )
        stable_port_access = runtime.index("::access(definition.port", bus_definition)
        self.assertLess(thermal_config_gate, thermal_config_read)
        self.assertLess(thermal_config_read, bus_definition)
        self.assertLess(bus_definition, stable_port_access)

        exact_abort = runtime.split(
            "const bool exact_trajectory_thermal_derating_region", 1
        )[1].split("std::array<double, 6> q_command", 1)[0]
        self.assertIn("control_command.quintic.present", exact_abort)
        self.assertIn("g_thermal_policy.derating_start_c", exact_abort)
        self.assertIn('"EXACT_TRAJECTORY_DERATING_ABORT"', exact_abort)

        rearm_request = runtime.index("request_thermal_rearm_for_next_cycle(")
        feedback_publish = runtime.index("const std::string payload = feedback_payload(")
        self.assertLess(rearm_request, feedback_publish)
        self.assertIn(
            "The latch deliberately remains set for this complete decision cycle",
            runtime[rearm_request:feedback_publish],
        )

        continuity = source.split("result.continuity_valid =", 1)[1].split(
            "result.actuation_safe =", 1
        )[0]
        self.assertNotIn("data.temp", continuity)
        self.assertNotIn("data.merror", continuity)
        actuation = source.split("result.actuation_safe =", 1)[1].split(";", 1)[0]
        self.assertIn("data.merror", actuation)
        self.assertIn("data.temp", actuation)

    def test_go_gui_socket_rejects_unauthenticated_recovery_commands(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        parser = source.split("void parse_command", 1)[1].split(
            "std::uint64_t minimum_epoch_after_lease", 1
        )[0]
        self.assertIn('throw std::runtime_error("COMMAND_RECOVERY_UNAUTHORIZED")', parser)
        self.assertIn("candidate.recovery = false", parser)
        self.assertIn('"RECOVERY_COMMAND_AUTHORIZATION_SELF_TEST_FAILED"', source)

    def test_go_worker_lease_expiry_uses_health_gated_fixed_hold(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            "capture_lease_safe_hold_command",
            "prior_external_hold_confirmed",
            "selected_owned_motors_confirmed_foc",
            "external_command_can_resume_from_safe_hold",
            "command.activation_epoch > source_epoch",
            "lease_expired_before_receive",
            "explicit_release_packet_received",
            '"lease_safe_hold", lease_safe_hold',
            '"LEASE_SAFE_HOLD_ENTER"',
            '"LEASE_SAFE_HOLD_ABORT"',
            "control_authority_available",
            'if (effective_mode == "drag")',
            '"EXPLICIT_DRAG_MODE=BRAKE',
        ):
            self.assertIn(token, source)
        self.assertNotIn('if (effective_mode == "drag" && active_allowed)', source)

    def test_hold_target_is_not_recaptured_from_external_displacement(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        capture = source.split("bool capture_lease_safe_hold_command", 1)[1].split(
            "bool external_command_explicitly_releases_safe_hold", 1
        )[0]
        for token in (
            "candidate.targets[1] = source.targets[1]",
            "candidate.targets[joint] = source.targets[joint]",
            "joint_uses_fixed_hold_target",
            'q_command[joint] = control_command.targets[joint]',
            "use_j2_hold_protection_limits",
            "should_apply_velocity_guard",
            '"LEASE_SAFE_HOLD_EXTERNAL_FORCE_SELF_TEST_FAILED"',
        ):
            self.assertIn(token, source)
        self.assertNotIn("candidate.targets[1] = 0.5 * (q_a + q_b)", capture)
        self.assertNotIn("candidate.targets[joint] = logical", capture)

    def test_fixed_hold_capture_is_close_once_and_immutable_within_epoch(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            "kFixedHoldCaptureWindow = 2.0",
            "kFixedHoldFeedbackFreshSeconds = 0.10",
            "validate_and_observe_fixed_hold_targets",
            "COMMAND_FIXED_HOLD_CAPTURE_FEEDBACK_UNHEALTHY",
            "COMMAND_FIXED_HOLD_CAPTURE_WINDOW",
            "COMMAND_FIXED_HOLD_TARGET_CHANGED_SAME_EPOCH",
            "COMMAND_POSITION_TARGET_CHANGED_SAME_EPOCH",
            "highest_rejected_active_epoch",
            "observe_rejected_active_epoch",
            '"FIXED_HOLD_INITIAL_CAPTURE_SELF_TEST_FAILED"',
            '"FIXED_HOLD_STALE_FEEDBACK_SELF_TEST_FAILED"',
            '"FIXED_HOLD_IMMUTABLE_SELF_TEST_FAILED"',
            '"FIXED_HOLD_TOGGLE_BYPASS_SELF_TEST_FAILED"',
            '"FIXED_HOLD_CAPTURE_WINDOW_SELF_TEST_FAILED"',
            '"REJECTED_ACTIVE_EPOCH_HOLD_SELF_TEST_FAILED"',
            '"REJECTED_ACTIVE_EPOCH_POSITION_SELF_TEST_FAILED"',
            '"REJECTED_ACTIVE_EPOCH_LEASE_SELF_TEST_FAILED"',
            '"REJECTED_ACTIVE_HIGHER_EPOCH_SELF_TEST_FAILED"',
            '"POSITION_TARGET_IMMUTABLE_SELF_TEST_FAILED"',
            '"POSITION_TO_HOLD_PREMATURE_SELF_TEST_FAILED"',
            '"POSITION_TO_HOLD_TARGET_REUSE_SELF_TEST_FAILED"',
            '"MANUAL_STOP_CAPTURE_SELF_TEST_FAILED"',
        ):
            self.assertIn(token, source)

    def test_velocity_observer_never_withdraws_position_authority(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            "moving_velocity_guard_tripped",
            'if (speed_degrees > 20.0)',
            'if (speed_degrees > 25.0)',
            'if (speed_degrees >= 40.0 || motor.fast_speed_count >= 2 ||',
            'return effective_mode == "position" &&',
            'command.moving_joint_mask[joint] && !command.recovery &&',
            '!position_endpoint_phase;',
            'position_arrived_once[index]',
            'position_endpoint_reached[joint] ||',
            "position_profile_at_authorized_endpoint",
            "j2_wire_q_command",
            'use_j2_hold_protection_limits(',
            '"EXTERNAL_MOTION_OBSERVED"',
            '"EXTERNAL_MOTION_SETTLED"',
            "MOVING_POSITION_VELOCITY_POLICY=TRANSITION_LOG_ONLY_NO_AUTHORITY_WITHDRAWAL",
            '"MOVING_VELOCITY_GUARD_SELF_TEST_FAILED"',
            '"J2_HOLD_PROTECTION_SELF_TEST_FAILED"',
            '"J2_PROFILE_ENDPOINT_HOLD_PROTECTION_SELF_TEST_FAILED"',
            '"J2_SYNC_SINGLE_PAIR_HARD_SELF_TEST_FAILED"',
            '"J2_SYNC_EXPLICIT_REARM_SELF_TEST_FAILED"',
            '"LEASE_SAFE_HOLD_WARNING_SYNC_SELF_TEST_FAILED"',
        ):
            self.assertIn(token, source)
        observer = source.split("bool apply_velocity_guards", 1)[1].split(
            "struct GuiCommand", 1
        )[0]
        self.assertNotIn("fault_latched", observer)

    def test_arrival_timeout_latches_domain_brake_and_reuses_safe_rearm(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            "position_arrival_overdue",
            '"POSITION_ARRIVAL_OVERDUE"',
            "position_arrival_timeout_detected",
            "position_arrival_timeout_latched",
            "latch_position_safety_watchdog",
            '"POSITION_ARRIVAL_TIMEOUT"',
            "POSITION_ARRIVAL_TIMEOUT_POLICY=",
            "LATCHED_DOMAIN_BRAKE_EXPLICIT_RELEASE_HIGHER_EPOCH_NEXT_CYCLE",
            'std::cerr << "POSITION_SAFETY_LATCH"',
        ):
            self.assertIn(token, source)
        self.assertNotIn('latch_domain_fault("J2_ARRIVAL_TIMEOUT")', source)
        self.assertNotIn('latch_domain_fault("ARRIVAL_TIMEOUT")', source)

    def test_unfinished_and_completed_position_lease_hold_preserve_endpoint(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        capture = source.split("bool capture_lease_safe_hold_command", 1)[1].split(
            "bool external_command_explicitly_releases_safe_hold", 1
        )[0]
        for token in (
            "const std::array<bool, 6>* position_arrived_once",
            "(void)position_arrived_once",
            "candidate.targets[1] = source.targets[1]",
            "candidate.targets[joint] = source.targets[joint]",
            "captured, &position_arrived_once",
            '"LEASE_SAFE_HOLD_CAPTURE_SELF_TEST_FAILED"',
            '"LEASE_SAFE_HOLD_COMPLETED_POSITION_TARGET_SELF_TEST_FAILED"',
            "!position_arrived_once[1]",
            "position_arrived_once[index] ||",
        ):
            self.assertIn(token, source)
        self.assertNotIn("q_a + q_b", capture.split("candidate.targets[1] =", 1)[1])
        self.assertNotIn("candidate.targets[joint] = logical", capture)

        unfinished_selftest = source.split(
            '"LEASE_SAFE_HOLD_CAPTURE_SELF_TEST_FAILED"', 1
        )[0].rsplit("if (", 1)[1]
        completed_selftest = source.split(
            "GuiCommand completed_position_capture", 1
        )[1].split(
            '"LEASE_SAFE_HOLD_COMPLETED_POSITION_TARGET_SELF_TEST_FAILED"', 1
        )[0]
        self.assertIn("lease_capture.targets[1] - 0.04", unfinished_selftest)
        self.assertIn(
            "completed_position_capture.targets[1] - 0.04",
            completed_selftest,
        )

    def test_position_moving_mask_separates_trajectory_and_support_axes(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            "moving_joint_mask",
            "COMMAND_MOVING_MASK_NOT_ACTIVE_SUBSET",
            "validate_command_for_owned_domain",
            "COMMAND_TARGET_MODEL_ENVELOPE",
            "within_model_command_envelope",
            "control_command.moving_joint_mask[index]",
            '"COMMAND_MIXED_LEASE_CAPTURE_SELF_TEST_FAILED"',
        ):
            self.assertIn(token, source)

    def test_go_worker_v13_quintic_authority_is_strict_and_time_indexed(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            '"go-m8010-gui-command/1.3"',
            '"go-m8010-quintic-command/1.0"',
            '"quintic-rest-to-rest-v1"',
            "struct QuinticTrajectoryDescriptor",
            "kMaximumQuinticIntervalCount = 1000000ULL",
            "kMaximumQuinticSamplePeriodNs = 10000000ULL",
            "trajectory.size() != kRequiredFields.size()",
            "result.duration_ns % result.interval_count != 0U",
            "COMMAND_QUINTIC_SAMPLE_GRID_INVALID",
            "COMMAND_QUINTIC_TARGETS_MISMATCH",
            "COMMAND_QUINTIC_NONMOVING_AXIS_CHANGED",
            "COMMAND_QUINTIC_MOVING_JOINT_COUNT_INVALID",
            "COMMAND_QUINTIC_MOVING_DISPLACEMENT_ZERO",
            "COMMAND_QUINTIC_ENDPOINT_STEP_INVALID",
            "COMMAND_QUINTIC_MOVING_MASK_MISSING",
            "COMMAND_QUINTIC_PEAK_VELOCITY_LIMIT",
            "COMMAND_QUINTIC_PEAK_ACCELERATION_LIMIT",
            "COMMAND_QUINTIC_START_MODEL_ENVELOPE",
            "COMMAND_QUINTIC_FIRST_PACKET_TOO_LATE",
            "validate_first_quintic_start_against_feedback",
            "COMMAND_QUINTIC_START_FEEDBACK_UNHEALTHY",
            "COMMAND_QUINTIC_START_FEEDBACK_MISMATCH",
            "COMMAND_QUINTIC_DESCRIPTOR_CHANGED_SAME_EPOCH",
            "same_quintic_trajectory_descriptor",
            "quintic_sample_clock",
            "unsigned __int128",
            "floor((now-execute_at)*N/duration_ns)",
            "apply_quintic_reference",
            "q_command[index] = command.quintic.target_rad[index]",
            '"trajectory_plan_token_id"',
            '"trajectory_sha256"',
            '"trajectory_state"',
            '"trajectory_sample_index"',
            '"trajectory_interval_count"',
            'const char* state = "PREPARED"',
            '"RUNNING"',
            '"COMPLETE"',
            'reported_trajectory_state = "INACTIVE"',
            '"COMMAND_QUINTIC_STRICT_FIELDS_SELF_TEST_FAILED"',
            '"COMMAND_QUINTIC_START_ENVELOPE_SELF_TEST_FAILED"',
            '"COMMAND_QUINTIC_MOVING_MASK_SELF_TEST_FAILED"',
            '"COMMAND_QUINTIC_INTEGER_GRID_SELF_TEST_FAILED"',
            '"COMMAND_QUINTIC_TEN_MS_GRID_SELF_TEST_FAILED"',
            '"COMMAND_QUINTIC_ENDPOINT_STEP_SELF_TEST_FAILED"',
            '"COMMAND_QUINTIC_ZERO_DISPLACEMENT_SELF_TEST_FAILED"',
            '"COMMAND_QUINTIC_START_MISMATCH_SELF_TEST_FAILED"',
            '"COMMAND_QUINTIC_START_HEALTH_SELF_TEST_FAILED"',
            '"COMMAND_QUINTIC_LATE_FIRST_PACKET_SELF_TEST_FAILED"',
            '"COMMAND_QUINTIC_IMMUTABLE_SELF_TEST_FAILED"',
            '"COMMAND_QUINTIC_INTEGER_SAMPLE_SELF_TEST_FAILED"',
            '"COMMAND_QUINTIC_EXACT_ENDPOINT_SELF_TEST_FAILED"',
        ):
            self.assertIn(token, source)

        start_feedback_guard = source.split(
            "void validate_first_quintic_start_against_feedback", 2
        )[2].split("bool maximum_moving_owned_position_error", 1)[0]
        self.assertIn("healthy_logical_position_for_joint(", start_feedback_guard)
        self.assertIn("kFixedHoldCaptureWindow + 1e-12", start_feedback_guard)

        parser = source.split("void parse_command", 1)[1].split(
            "std::uint64_t minimum_epoch_after_lease", 1
        )[0]
        self.assertIn("candidate.quintic = QuinticTrajectoryDescriptor{}", parser)
        self.assertIn(
            'if (schema == "go-m8010-gui-command/1.3" && mode == "position")',
            parser,
        )

        runtime = source.split("int run(const Options& options)", 1)[1]
        measured_replan = runtime.split(
            "const bool active_transition", 1
        )[1].split("if (is_position_holding_mode(effective_mode))", 1)[0]
        self.assertIn("!control_command.quintic.present", measured_replan)
        planned = runtime.split("std::set<int> planned", 1)[1].split(
            "std::array<bool, 6> profile_endpoint_phase", 1
        )[0]
        self.assertIn("if (control_command.quintic.present)", planned)
        self.assertIn("apply_quintic_reference(", planned)
        self.assertIn("else {\n            update_profile(", planned)
        self.assertIn(
            'std::string(trajectory_sample.state) != "PREPARED"', runtime
        )
        feedback_sample = source.split("samples.push_back({", 1)[1].split(
            "controller_mode_by_motor[motor.name]", 1
        )[0]
        for field in (
            '"trajectory_plan_token_id"',
            '"trajectory_sha256"',
            '"trajectory_state"',
            '"trajectory_sample_index"',
            '"trajectory_interval_count"',
        ):
            self.assertIn(field, feedback_sample)

    def test_release_commands_atomically_fence_same_epoch_backlog(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            "command_releases_owned_domain",
            "command.activation_epoch >= safety.minimum_activation_epoch",
            "safety.last_seen_activation_epoch + 1U",
            "result.domain_release_received",
            '"COMMAND_RELEASE_BACKLOG_SELF_TEST_FAILED"',
            '"COMMAND_DOMAIN_DESELECT_FENCE_SELF_TEST_FAILED"',
            '"COMMAND_DRAG_FENCE_SELF_TEST_FAILED"',
        ):
            self.assertIn(token, source)

    def test_all_buses_require_five_consecutive_bad_feedback_frames(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            "motor_feedback_requires_domain_brake",
            "owned_domain_feedback_requires_brake",
            "motor.reference_ready && motor.valid",
            "motor_domain_brake_this_cycle = true",
            "record_domain_brake_event",
            '"OWNED_DOMAIN_BRAKE_BEGIN"',
            '"OWNED_DOMAIN_BRAKE_FINAL"',
            "observe_feedback_frame_validity",
            "invalid_feedback_limit_for_bus",
            '"DOMAIN_TRANSIENT_FEEDBACK_HOLD_SELF_TEST_FAILED"',
            '"DOMAIN_SUSTAINED_FEEDBACK_BRAKE_SELF_TEST_FAILED"',
        ):
            self.assertIn(token, source)
        threshold = source.split("int invalid_feedback_limit_for_bus", 1)[1].split(
            "bool observe_feedback_frame_validity", 1
        )[0]
        self.assertIn("(void)bus", threshold)
        self.assertIn("return 5;", threshold)
        self.assertNotIn('bus == "', threshold)
        self.assertIn("for (int frame = 1; frame < 5; ++frame)", source)
        self.assertIn(
            "observe_feedback_frame_validity(lease_motors[0], false, 5)",
            source,
        )
        self.assertNotIn('"DOMAIN_SINGLE_FRAME_BRAKE_SELF_TEST_FAILED"', source)

    def test_command_receive_budget_and_rejections_are_bounded(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            "kCommandPacketBudget = 32U",
            "kCommandRejectSummarySeconds = 5.0",
            "packet_index < kCommandPacketBudget",
            '"GUI_COMMAND_REJECTED_BEGIN"',
            '"GUI_COMMAND_REJECTED_SUMMARY"',
            '"GUI_COMMAND_REJECTED_FINAL"',
            '"GUI_COMMAND_RX_BUDGET_SUMMARY"',
        ):
            self.assertIn(token, source)
        self.assertNotIn('std::cerr << "GUI_COMMAND_REJECTED="', source)

    def test_active_deadline_is_transition_logged_without_withdrawing_authority(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            "kActiveDeadlineConsecutiveLimit = 3",
            "next_active_deadline_miss_count",
            "next = loop_started",
            '"ACTIVE_LOOP_DEGRADED"',
            '"ACTIVE_LOOP_RECOVERED"',
            "ACTIVE_DEADLINE_POLICY=TRANSITION_LOG_ONLY_NO_AUTHORITY_WITHDRAWAL",
            '"DEADLINE_SINGLE_MISS_SELF_TEST_FAILED"',
            '"DEADLINE_CONSECUTIVE_SELF_TEST_FAILED"',
        ):
            self.assertIn(token, source)
        self.assertNotIn(
            'latch_domain_fault("ACTIVE_LOOP_DEADLINE_CONSECUTIVE")', source
        )

    def test_go_worker_diagnostic_episodes_are_transition_logged_and_rate_limited(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            "diagnostic_log_interval_elapsed",
            "pending_transition_count",
            '"J2_INVALID_FEEDBACK_BEGIN"',
            '"J2_INVALID_FEEDBACK_SUMMARY"',
            '"J2_INVALID_FEEDBACK_FINAL"',
            '"OWNED_DOMAIN_BRAKE_SUMMARY"',
            '"ACTIVE_COMMAND_BLOCKED_BEGIN"',
            '"ACTIVE_COMMAND_BLOCKED_SUMMARY"',
            '"ACTIVE_COMMAND_BLOCKED_FINAL"',
            '" episode_count="',
            '" total_count="',
            '"DIAGNOSTIC_RATE_LIMIT_SELF_TEST_FAILED"',
        ):
            self.assertIn(token, source)
        self.assertNotIn('std::cerr << "J2_INVALID_FEEDBACK"', source)
        self.assertNotIn('std::cerr << "ACTIVE_COMMAND_BLOCKED"', source)
        self.assertNotIn('"J2_INVALID_FEEDBACK_END"', source)
        self.assertNotIn('"OWNED_DOMAIN_BRAKE_THIS_CYCLE"', source)
        self.assertNotIn('"ACTIVE_COMMAND_BLOCKED_END"', source)
        self.assertIn("flush_active_command_blocked_summary", source)

    def test_state_node_prefers_per_motor_controller_mode(self):
        source = (PACKAGE / "whole_arm_state_node.py").read_text(encoding="utf-8")
        model = (PACKAGE / "state_model.py").read_text(encoding="utf-8")
        self.assertIn('payload.get("controller_mode_by_motor", {})', model)
        self.assertIn("set(modes_by_motor) != motors", model)
        self.assertIn("mode = modes_by_motor[motor]", model)
        self.assertIn('lease_value = payload.get("lease_safe_hold")', model)
        self.assertIn("parse_controller_feedback_metadata", source)
        self.assertIn('snapshot["lease_safe_hold_by_motor"]', source)

    def test_j2_sync_warning_is_live_and_only_updated_by_j2_payloads(self):
        source = (PACKAGE / "whole_arm_state_node.py").read_text(encoding="utf-8")
        self.assertIn("def update_j2_sync_fault", source)
        self.assertIn("in J2_MOTOR_NAMES for sample in samples", source)
        self.assertIn('return bool(payload.get("j2_sync_fault", False))', source)
        self.assertNotIn(
            'if bool(payload.get("j2_sync_fault", False)):\n                self.j2_sync_fault = True',
            source,
        )

    def test_state_node_aggregates_repeated_warning_lines(self):
        source = (PACKAGE / "whole_arm_state_node.py").read_text(encoding="utf-8")
        self.assertIn("WARNING_LOG_INTERVAL_NS = 5_000_000_000", source)
        self.assertIn("def warn_rate_limited", source)
        self.assertIn('f"raw_feedback_rejected:{reason}"', source)
        self.assertIn("def feedback_rejection_reason", source)
        self.assertIn("except UnicodeDecodeError as exc:", source)
        self.assertIn('"snapshot_unavailable"', source)
        self.assertIn("suppressed_since_last=", source)
        self.assertNotIn(
            'self.get_logger().warning(f"rejected raw feedback payload: {exc}")',
            source,
        )

    def test_state_node_commits_feedback_and_safety_metadata_atomically(self):
        source = (PACKAGE / "whole_arm_state_node.py").read_text(encoding="utf-8")
        self.assertIn(
            'payload.get("schema") != "go-m8010-motor-feedback/1.0"',
            source,
        )
        self.assertIn("self.model.update_batch(samples)", source)
        self.assertIn("parse_controller_feedback_metadata", source)
        self.assertIn('if "j2_sync_fault" not in payload:', source)
        self.assertLess(
            source.index("self.model.update_batch(samples)"),
            source.index("self.j2_sync_fault = next_j2_sync_fault"),
        )
        self.assertLess(
            source.index("self.model.update_batch(samples)"),
            source.index("self.controller_feedback = next_controller_feedback"),
        )
        self.assertIn('snapshot["schema"] = "go-m8010-hardware-state/1.1"', source)
        self.assertIn('snapshot["state_instance_id"] = self.state_instance_id', source)
        self.assertIn('snapshot["source_monotonic_ns"] = now_monotonic_ns', source)
        self.assertIn("validate_j6_feedback_identity", source)
        self.assertIn("previous=self.j6_feedback_identity", source)
        self.assertIn(
            "expected_state_instance_id=self.state_instance_id", source
        )
        self.assertIn(
            'snapshot["j6_raw_feedback_identity"]', source
        )
        self.assertLess(
            source.index("self.model.update_batch(samples)"),
            source.index(
                "self.j6_feedback_identity = next_j6_feedback_identity"
            ),
        )
        self.assertIn("controller_metadata_for_hardware_state", source)
        self.assertIn('motor_state.get("fresh") is True', source)

    def test_mock_feedback_implements_the_strict_safety_contract(self):
        source = (PACKAGE / "mock_feedback_node.py").read_text(encoding="utf-8")
        for token in (
            '"controller_mode": "brake"',
            '"domain_fault": False',
            '"j2_sync_fault": False',
            '"lease_safe_hold": False',
        ):
            self.assertIn(token, source)

    def test_go_gui_worker_freezes_full_model_ranges_and_j2_control_guards(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        for token in (
            "kModelCommandLower",
            "-155.6 * kPi / 180.0",
            "-129.49 * kPi / 180.0",
            "-118.54 * kPi / 180.0",
            "kModelCommandUpper",
            "+184.4 * kPi / 180.0",
            "+145.51 * kPi / 180.0",
            "+103.26 * kPi / 180.0",
            "kFeedbackEnvelopeTolerance = 0.5",
            "kTargetTimeoutSeconds = 90.0",
            '"MODEL_COMMAND_FULL_ENDPOINTS_SELF_TEST_FAILED"',
            '"FEEDBACK_ENDPOINT_TOLERANCE_SELF_TEST_FAILED"',
            '"REJECTED_TARGET_PRESERVES_POSITION_AUTHORITY_SELF_TEST_FAILED"',
            '"REJECTED_TARGET_PRESERVES_HOLD_AUTHORITY_SELF_TEST_FAILED"',
            "kJ2SyncWarningLimit = 0.25",
            "kJ2SyncLimit = 0.5",
            '"J2_SYNC_WARNING"',
            '"J2_SYNC_HARD_LATCHED"',
            '" action=BOTH_BRAKE"',
            '"J2_SYNC_REARM_PENDING_NEXT_CYCLE"',
            "apply_pending_j2_sync_rearm_at_cycle_start",
            "kHoldIntegralRotorHardNm",
            "kJ2IntegralRotorHardNm = kHoldIntegralRotorHardNm[1]",
            "kJ2PredictedRotorWorkNm = 1.75",
            "kJ2RotorTorqueFeedbackHardNm = 1.80",
            "kJ2MaximumAcceleration = 15.0",
            "kJ2KpRampSeconds = 0.50",
            "kJ2MovingKpLimit = 3.00",
            "kJ2MovingKdLimit = 0.30",
            "update_bounded_hold_integral",
            "govern_single_motor_reference",
            '"HOLD_TORQUE_CAPACITY_SELF_TEST_FAILED"',
            '"HOLD_TOTAL_TORQUE_BUDGET_SELF_TEST_FAILED"',
            '"HOLD_INTEGRAL_PRESERVE_SELF_TEST_FAILED"',
            '"AUX_WIRE_GOVERNOR_SELF_TEST_FAILED"',
            '"J2_LOADED_MOTION_ENVELOPE_SELF_TEST_FAILED"',
            '"LEASE_SAFE_HOLD_STICKY_TRANSIENT_SELF_TEST_FAILED"',
            '"J2_SYNC_UNAVAILABLE_RESETS_REARM_SELF_TEST_FAILED"',
            '"J2_SYNC_NONFINITE_HARD_SELF_TEST_FAILED"',
            "motor.last_frame_valid && motor.valid",
            "govern_j2_reference",
            "intersect_absolute_affine_constraint",
            '"J2_EVENT_REPLAY_GOVERNOR_SELF_TEST_FAILED"',
            '"J2_EXTERNAL_DISPLACEMENT_GOVERNOR_SELF_TEST_FAILED"',
            '"J2_GOVERNOR_LIMITED"',
            '"J2_GOVERNOR_RECOVERED"',
            "j2_integral_wire_nm",
            "within_mechanical_feedback_envelope",
            "position_endpoint_reached",
        ):
            self.assertIn(token, source)
        for forbidden in (
            "kJ2SyncImmediateLimit",
            "kJ2SyncTripConsecutiveFrames",
            '"J2_SYNC_TRANSIENT_KEEPING_FOC"',
        ):
            self.assertNotIn(forbidden, source)
        observer = source.split("bool observe_j2_sync_error", 1)[1].split(
            "bool observe_explicit_j2_sync_release", 1
        )[0]
        self.assertIn("magnitude > kJ2SyncWarningLimit", observer)
        self.assertIn("magnitude > kJ2SyncLimit", observer)
        runtime = source.split("const double sync_error = q_a - q_b;", 1)[1].split(
            "const double q_common", 1
        )[0]
        self.assertLess(
            runtime.index("observe_j2_sync_error"),
            runtime.index("for (auto& motor : motors) transact_and_commit_brake(motor);"),
        )
        self.assertIn('effective_mode = "brake";', runtime)
        self.assertIn("j2_pair_ready = false;", runtime)
        self.assertIn(
            '} else if (options.bus == "j2" && motors.size() == 2U) {',
            source,
        )
        self.assertNotIn("kTargetLimit =", source)
        self.assertNotIn("kJ2TargetLimit =", source)
        self.assertNotIn('latch_domain_fault("J2_FEEDFORWARD_BASE_INFEASIBLE")', source)
        self.assertNotIn('latch_domain_fault("J2_PD_PREDICTION_LIMIT")', source)

    def test_software_saturation_feeds_latched_no_progress_watchdog(self):
        source = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        saturation = source.split(
            "const bool torque_feedback_saturated", 1
        )[1].split("if (any_foc_sent)", 1)[0]
        for token in (
            "any_foc_sent",
            "software_saturation_observed_this_cycle",
            '"J2_TORQUE_FEEDBACK_SOFTWARE_SATURATION_OBSERVED"',
            '"J2_TORQUE_FEEDBACK_SOFTWARE_SATURATION_RECOVERED"',
            "kLoadLimitWatchdogAuthority",
            "j2_torque_feedback_saturated_reported = torque_feedback_saturated",
        ):
            self.assertIn(token, saturation)
        # One saturated frame remains an observation; only the persistent
        # error/no-improvement watchdog below may latch and brake.
        for forbidden in (
            "fault_latched",
            "latch_domain_fault",
            "motor_domain_brake_this_cycle",
            "kBrakeMode",
        ):
            self.assertNotIn(forbidden, saturation)

        for token in (
            '"SOFTWARE_GUARD_NOT_CONTINUOUS_RATING"',
            "kNoProgressMinimumPositionError = 2.0",
            "kNoProgressMinimumImprovement = 0.25",
            "kNoProgressWindowSeconds = 3.0",
            "kNoProgressMinimumQualifyingFrames = 100",
            "struct NoProgressWatchdogState",
            "latch_position_safety_watchdog",
            "observe_no_progress_watchdog",
            "maximum_moving_owned_position_error",
            "software_saturation_joint_mask",
            "!saturated_joint_mask[joint]",
            "software_saturation_observed",
            "position_error_rad >= kNoProgressMinimumPositionError",
            "state.window_baseline_error_rad - position_error_rad",
            "elapsed_seconds < kNoProgressWindowSeconds",
            "observe_explicit_no_progress_release",
            "request_no_progress_rearm_for_next_cycle",
            "apply_pending_no_progress_rearm_at_cycle_start",
            "activation_epoch > state.trip_activation_epoch",
            "activation_epoch >= state.minimum_rearm_epoch",
            "activation_epoch > highest_rejected_active_epoch",
            "all_domain_motors_valid_brake_for_rearm",
            "no_progress_watchdog_self_test();",
            '"NO_PROGRESS_SHORT_WINDOW_SELF_TEST_FAILED"',
            '"NO_PROGRESS_IMPROVEMENT_RESET_SELF_TEST_FAILED"',
            '"NO_PROGRESS_GAP_RESET_SELF_TEST_FAILED"',
            '"NO_PROGRESS_NEXT_CYCLE_REARM_SELF_TEST_FAILED"',
            '"POSITION_ARRIVAL_TIMEOUT_LATCH_SELF_TEST_FAILED"',
            '"POSITION_ARRIVAL_TIMEOUT"',
            '"EXACT_TRAJECTORY_LOAD_GOVERNOR_ABORT"',
            '"EXACT_TRAJECTORY_LOAD_GOVERNOR_ABORT_SELF_TEST_FAILED"',
            '"V13_LOAD_GOVERNOR_TRAJECTORY_ABORT"',
            "LATCHED_DOMAIN_BRAKE_EXPLICIT_RELEASE_HIGHER_EPOCH_NEXT_CYCLE",
        ):
            self.assertIn(token, source)

        self.assertEqual(source.count("governed.alpha != 1.0"), 2)
        self.assertNotIn("governed.alpha < 1.0 - 1e-9", source)
        self.assertEqual(source.count("exact_recipe && governed.alpha == 1.0"), 4)

        runtime = source.split("int run(const Options& options)", 1)[1]
        exact_abort = runtime.index(
            '"EXACT_TRAJECTORY_LOAD_GOVERNOR_ABORT"'
        )
        motor_send_loop = runtime.index("std::size_t motor_runtime_index", exact_abort)
        self.assertLess(exact_abort, motor_send_loop)
        trip = runtime.split(
            "const bool no_progress_tripped_this_cycle", 1
        )[1].split("bool lease_safe_hold_aborted_this_cycle", 1)[0]
        for token in (
            "motor_domain_brake_reason = no_progress_watchdog.trip_reason",
            'effective_mode = "brake"',
            "position_tracking.fill(false)",
            "position_tracking_epoch.fill(0U)",
            "reset_bounded_hold_integral(state)",
            "hold_integral_wire_nm.fill(0.0)",
            'std::cerr << "POSITION_SAFETY_LATCH"',
            "transact_and_commit_brake(motor)",
        ):
            self.assertIn(token, trip)
        brake_helper = runtime.split(
            "auto transact_and_commit_brake", 1
        )[1].split("if (motor_domain_brake_this_cycle)", 1)[0]
        self.assertIn("make_command(", brake_helper)
        self.assertIn("kBrakeMode", brake_helper)
        self.assertNotIn("latch_domain_fault", trip)

        pending = runtime.index("request_no_progress_rearm_for_next_cycle(")
        publish = runtime.index("const std::string payload = feedback_payload(")
        self.assertLess(pending, publish)
        self.assertIn(
            "decision cycle stays in domain BRAKE",
            runtime[pending:publish],
        )
        payload = source.split("std::string feedback_payload", 1)[1].split(
            "bool send_terminal_brake", 1
        )[0]
        for token in (
            '"tau_cmd_rotor_nm", motor.last_tau_cmd_rotor_nm',
            '"tau_feedback_rotor_nm", motor.last_tau',
            '"tau_joint_estimated_nm", motor.sign * motor.last_tau * kGear',
            '"tau_j2_logical_total_nm", tau_j2_logical_total_nm',
            '"last_valid_feedback_monotonic_ns"',
            '"load_limit_no_progress"',
            '"no_progress_fault"',
            '"no_progress_release_observed"',
            '"no_progress_minimum_rearm_epoch"',
            '"position_safety_trip_reason"',
            '"software_saturation_observed"',
            '"load_limit_watchdog_authority"',
        ):
            self.assertIn(token, payload)
        self.assertNotIn('{"torque",', payload)
        command_window = runtime.split(
            "const double tau = send_mode == kFocMode", 1
        )[1].split("const Feedback feedback = transact(", 1)[0]
        self.assertIn(
            "motor.last_tau_cmd_rotor_nm = send_mode == kFocMode ? tau : 0.0",
            command_window,
        )
        self.assertIn("motor.last_tau_cmd_rotor_nm = 0.0", brake_helper)

    def test_torque_runtime_fields_and_last_valid_feedback_are_end_to_end(self):
        controller = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        j6 = (
            REPOSITORY
            / "tools"
            / "hardware"
            / "j6_dm_g6220"
            / "v15_30a_gui_j6_controller.py"
        ).read_text(encoding="utf-8")
        state_model = (PACKAGE / "state_model.py").read_text(encoding="utf-8")
        gui = (
            REPOSITORY
            / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
            / "ros2_ws"
            / "src"
            / "go_m8010_arm_gui"
            / "go_m8010_arm_gui"
            / "main_window.py"
        ).read_text(encoding="utf-8")
        exact_fields = (
            "tau_cmd_rotor_nm",
            "tau_feedback_rotor_nm",
            "tau_joint_estimated_nm",
            "tau_j2_logical_total_nm",
        )
        for field in exact_fields:
            self.assertIn(field, controller)
            self.assertIn(field, j6)
            self.assertIn(field, state_model)
        self.assertIn('"tau_cmd_rotor_nm": None', j6)
        self.assertIn('"tau_feedback_rotor_nm": None', j6)
        self.assertIn('"tau_joint_estimated_nm": None', j6)
        self.assertIn('"tau_j2_logical_total_nm": None', j6)
        self.assertIn("last_valid_feedback_monotonic_ns", controller)
        self.assertIn("last_valid_feedback_monotonic_ns", j6)
        self.assertIn("last_valid_feedback_monotonic_ns", state_model)
        self.assertIn("last_valid_feedback_monotonic_ns", gui)
        self.assertIn('number("tau_joint_estimated_nm")', gui)
        self.assertNotIn('number("estimated_joint_torque_nm")', gui)

    def test_j2_power_session_reference_is_fail_closed_across_processes(self):
        controller = (
            REPOSITORY / "tools" / "hardware" / "v15_30a_gui_go_controller.cpp"
        ).read_text(encoding="utf-8")
        state_model = (PACKAGE / "state_model.py").read_text(encoding="utf-8")
        state_node = (PACKAGE / "whole_arm_state_node.py").read_text(encoding="utf-8")
        supervisor = (REPOSITORY / "start_arm_gui.sh").read_text(encoding="utf-8")
        launch = (
            REPOSITORY
            / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
            / "ros2_ws"
            / "src"
            / "go_m8010_arm_gui"
            / "launch"
            / "arm_gui.launch.py"
        ).read_text(encoding="utf-8")
        for token in (
            "J2_SESSION_REFERENCE_GATE_MISSING",
            "load_j2_session_reference",
            "J2_SESSION_REFERENCE_STARTUP_MISMATCH",
            "kJ2SessionStartupTolerance",
            "session_reference_configured",
        ):
            self.assertIn(token, controller)
        self.assertIn("kPersistentPhaseMismatchLimit", controller)
        self.assertNotIn("kPi + 1e-12", controller)
        self.assertIn("reference_blocked_motors", state_model)
        self.assertIn("J2_SESSION_STARTUP_TOLERANCE_RAD", state_model)
        self.assertIn("j2_session_reference_path", state_node)
        self.assertIn("parent_persistent_zero_sha256", state_node)
        self.assertIn('snapshot["initial_pose_sha256"]', state_node)
        self.assertIn("J2_SESSION_REFERENCE_FILE", supervisor)
        self.assertIn("--expected-zero-sha256", supervisor)
        self.assertIn("j2_session_reference_path", launch)

    def test_supervisor_preflights_single_use_j2_launch_permit_before_ros(self):
        supervisor = (REPOSITORY / "start_arm_gui.sh").read_text(encoding="utf-8")
        for token in (
            'J2_SESSION_LAUNCH_PERMIT="${J2_SESSION_LAUNCH_PERMIT_FILE:-}"',
            'J2_POWER_SESSION_ID="${J2_POWER_SESSION_ID:-}"',
            "go-m8010-j2-power-session-launch-permit/1.0",
            'f"/run/user/{os.geteuid()}/go-m8010/anchors"',
            'stat.S_IMODE(status.st_mode) != 0o600',
            'status.st_nlink != 1',
            'stat.S_IMODE(status.st_mode) != 0o700',
            'path_lexists(expected_inflight) or path_lexists(expected_spent)',
            'now_boottime_ns >= expires_boottime_ns',
            "MAX_TTL_NS = 30_000_000_000",
            "MAX_CAPTURE_TO_PERMIT_NS = 300_000_000_000",
            'permit.get("power_session_id") != expected_power_session_id',
            'permit_reference.get("sha256") != anchor_sha256',
            'permit_id != expected_permit_id',
            'raw_worker_path != str(worker_path)',
            'motor_ids != [0, 1]',
            'serial_signs != EXPECTED_SIGNS',
            'recheck.get("minimum_brake_frames")',
            '--j2-session-launch-permit-file "$J2_SESSION_LAUNCH_PERMIT"',
            '--expected-j2-session-reference-sha256 "$J2_SESSION_REFERENCE_SHA256"',
            '--expected-j2-power-session-id "$J2_POWER_SESSION_ID"',
            '--expected-worker-sha256 "$J2_EXPECTED_WORKER_SHA256"',
            "clear_j2_session_inputs",
            "mark_domain_unavailable J2 \"$J2_GATE_FAILURE\"",
            "initial_pose_read_only:=true",
        ):
            self.assertIn(token, supervisor)

        compile_call = supervisor.index('if ! g++ -std=c++17')
        offline_self_test = supervisor.index(
            'if [[ -z "$GO_BUILD_FAILURE" ]] && ! run_go_offline_self_test'
        )
        gate_call = supervisor.index(
            'J2_GATE_OUTPUT="$(validate_j2_session_launch_gate'
        )
        bundle_audit = supervisor.index('--audit-j2-session-bundle --bus j2')
        ros_launch = supervisor.index("exec ros2 launch")
        first_worker = supervisor.index("start_worker J1")
        self.assertLess(compile_call, offline_self_test)
        self.assertLess(offline_self_test, gate_call)
        self.assertLess(gate_call, bundle_audit)
        self.assertLess(bundle_audit, ros_launch)
        self.assertLess(bundle_audit, first_worker)
        self.assertLess(gate_call, supervisor.index("python3 -m venv"))
        self.assertLess(gate_call, supervisor.index(" -m pip install"))
        self.assertLess(gate_call, supervisor.index(" -m colcon build"))

        launch_block = supervisor[ros_launch : supervisor.index(") &", ros_launch)]
        self.assertIn('j2_session_reference_path:="$J2_SESSION_REFERENCE"', launch_block)
        self.assertNotIn("J2_SESSION_LAUNCH_PERMIT", launch_block)
        self.assertNotIn("J2_POWER_SESSION_ID", launch_block)

        gate_failure_block = supervisor[
            supervisor.index('if [[ -n "$J2_GATE_FAILURE" ]]') :
            supervisor.index(
                '  GO_AUX_GATE_FAILURE=""'
            )
        ]
        self.assertIn('mark_domain_unavailable J2 "$J2_GATE_FAILURE"', gate_failure_block)
        self.assertIn("clear_j2_session_inputs", gate_failure_block)
        self.assertNotIn("fail ", gate_failure_block)

    def test_supervisor_reuses_only_manifest_proven_current_go_build(self):
        supervisor = (REPOSITORY / "start_arm_gui.sh").read_text(encoding="utf-8")
        for token in (
            'GO_BUILD_MANIFEST="$GO_BINARY.build.json"',
            "basic_j2_permit_worker_matches",
            "canonical_worker != canonical_go",
            "worker_path_text != str(canonical_worker)",
            "binary_sha256 != worker_sha256",
            "go-m8010-go-controller-build/1.0",
            'secure_manifest=True',
            'status.st_uid != os.geteuid()',
            'stat.S_IMODE(status.st_mode) != 0o600',
            'status.st_nlink != 1',
            'manifest_source.get("path") != str(canonical_source)',
            'manifest_source.get("sha256") != source_sha256',
            'raise ValueError("build manifest controller source mismatch")',
            'raise ValueError("build manifest schema mismatch")',
            'manifest_binary.get("sha256") != binary_sha256',
            'manifest_sdk.get("sha256") != sdk_sha256',
            'manifest.get("sdk_headers") != sdk_header_bindings',
            'manifest.get("compile_profile") != expected_profile',
            '"passed": True',
            '"serial_opened": False',
            "publish_go_build_manifest",
            "self_test = subprocess.run(",
            'or "SERIAL_OPENED=NO" not in self_test_lines',
            "or digest(binary_path) != binary_sha256",
            "os.replace(temporary_path, canonical_manifest)",
            "os.fchmod(stream.fileno(), 0o600)",
            "GO_REUSED_FROM_J2_PERMIT=1",
            "run_go_offline_self_test",
            "grep -Fxq 'SERIAL_OPENED=NO'",
            "J2_SESSION_BUNDLE_AUDIT=PASS",
            'J2_SIGNED_LAUNCH_ATTEMPT=1',
            "签发 permit 后禁止编译",
            "本次禁止安装依赖",
            "本次禁止运行 colcon",
        ):
            self.assertIn(token, supervisor)
        cache_call = supervisor.index(
            'basic_j2_permit_worker_matches "$J2_SESSION_LAUNCH_PERMIT"'
        )
        compile_call = supervisor.index('if ! g++ -std=c++17')
        self_test_call = supervisor.index(
            'if [[ -z "$GO_BUILD_FAILURE" ]] && ! run_go_offline_self_test'
        )
        manifest_publish = supervisor.index(
            '! publish_go_build_manifest "$GO_BUILD_INPUT_FINGERPRINT"; then'
        )
        gate_call = supervisor.index(
            'J2_GATE_OUTPUT="$(validate_j2_session_launch_gate'
        )
        self.assertLess(cache_call, compile_call)
        self.assertLess(compile_call, self_test_call)
        self.assertLess(self_test_call, manifest_publish)
        self.assertLess(manifest_publish, gate_call)
        self.assertLess(self_test_call, gate_call)

        signed_build_start = supervisor.rindex(
            'if [[ "$J2_SIGNED_LAUNCH_ATTEMPT" -eq 1 ]]; then', 0, cache_call
        )
        unsigned_build_start = supervisor.index(
            "else\n    printf '预构建阶段：正在严格编译", signed_build_start
        )
        self.assertNotIn(
            "g++ -std=c++17",
            supervisor[signed_build_start:unsigned_build_start],
        )
        self.assertIn(
            "g++ -std=c++17",
            supervisor[unsigned_build_start:self_test_call],
        )

        ros_setup = supervisor.index("source /opt/ros/humble/setup.bash")
        signed_colcon_start = supervisor.index(
            'if [[ "$J2_SIGNED_LAUNCH_ATTEMPT" -eq 1 ]]; then', ros_setup
        )
        unsigned_colcon_start = supervisor.index(
            "else\n  printf '预构建阶段：正在构建两个最小 ROS2 包", signed_colcon_start
        )
        self.assertNotIn(
            " -m colcon build",
            supervisor[signed_colcon_start:unsigned_colcon_start],
        )

    def test_supervisor_manifest_binds_sdk_headers_and_freezes_build_inputs(self):
        supervisor = (REPOSITORY / "start_arm_gui.sh").read_text(encoding="utf-8")
        for token in (
            '"$GO_SDK_INCLUDE/unitreeMotor/unitreeMotor.h"',
            '"$GO_SDK_INCLUDE/unitreeMotor/include/motor_msg_GO-M8010-6.h"',
            '"$GO_SDK_INCLUDE/unitreeMotor/include/motor_msg_A1B1.h"',
            '"$GO_SDK_INCLUDE/serialPort/SerialPort.h"',
            '"$GO_SDK_INCLUDE/serialPort/include/errorClass.h"',
            '"$GO_SDK_INCLUDE/IOPort/IOPort.h"',
            '[[ "${#capture_args[@]}" -eq 9 ]] || return 1',
            '"schema": "go-m8010-go-build-input-fingerprint/1.0"',
            '"sha256": digest(header_path)',
            'manifest.get("sdk_headers") != sdk_header_bindings',
            'raise ValueError("build manifest SDK header path/SHA mismatch")',
            '"sdk_headers": expected_fingerprint["sdk_headers"]',
            "capture_go_build_inputs()",
            "verify_go_build_inputs_unchanged()",
            'GO_BUILD_INPUT_FINGERPRINT="$(capture_go_build_inputs',
            'elif ! verify_go_build_inputs_unchanged "$GO_BUILD_INPUT_FINGERPRINT"',
            'build_input_fingerprint(canonical_headers) != expected_fingerprint',
            'raise ValueError("build inputs changed during the manifest offline self-test")',
            'raise ValueError("build inputs or binary changed before manifest publication")',
        ):
            self.assertIn(token, supervisor)

        capture = supervisor.index(
            'GO_BUILD_INPUT_FINGERPRINT="$(capture_go_build_inputs'
        )
        compile_call = supervisor.index("elif ! g++ -std=c++17", capture)
        post_compile_check = supervisor.index(
            'elif ! verify_go_build_inputs_unchanged "$GO_BUILD_INPUT_FINGERPRINT"',
            compile_call,
        )
        self_test_call = supervisor.index(
            'if [[ -z "$GO_BUILD_FAILURE" ]] && ! run_go_offline_self_test',
            post_compile_check,
        )
        post_self_test_check = supervisor.index(
            '! verify_go_build_inputs_unchanged "$GO_BUILD_INPUT_FINGERPRINT"; then',
            self_test_call,
        )
        manifest_publish = supervisor.index(
            '! publish_go_build_manifest "$GO_BUILD_INPUT_FINGERPRINT"; then',
            post_self_test_check,
        )
        self.assertLess(capture, compile_call)
        self.assertLess(compile_call, post_compile_check)
        self.assertLess(post_compile_check, self_test_call)
        self.assertLess(self_test_call, post_self_test_check)
        self.assertLess(post_self_test_check, manifest_publish)

        capture_function = supervisor[
            supervisor.index("capture_go_build_inputs() {") :
            supervisor.index("verify_go_build_inputs_unchanged() {")
        ]
        python_start = capture_function.index("<<'PY'\n") + len("<<'PY'\n")
        fingerprint_program = capture_function[
            python_start : capture_function.index("\nPY", python_start)
        ]
        header_relatives = (
            "unitreeMotor/unitreeMotor.h",
            "unitreeMotor/include/motor_msg_GO-M8010-6.h",
            "unitreeMotor/include/motor_msg_A1B1.h",
            "serialPort/SerialPort.h",
            "serialPort/include/errorClass.h",
            "IOPort/IOPort.h",
        )
        with tempfile.TemporaryDirectory() as temporary_directory:
            root = Path(temporary_directory)
            source_path = root / "controller.cpp"
            library_path = root / "libUnitreeMotorSDK_Linux64.so"
            include_path = root / "include"
            source_path.write_text("int main() { return 0; }\n", encoding="utf-8")
            library_path.write_bytes(b"sdk-library")
            header_paths = []
            for index, relative in enumerate(header_relatives):
                header_path = include_path / relative
                header_path.parent.mkdir(parents=True, exist_ok=True)
                header_path.write_text(f"header-{index}\n", encoding="utf-8")
                header_paths.append(header_path)

            command = [
                sys.executable,
                "-",
                str(source_path),
                str(include_path),
                str(library_path),
                *(str(path) for path in header_paths),
            ]
            before = subprocess.run(
                command,
                input=fingerprint_program,
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(before.returncode, 0, before.stderr)
            before_fingerprint = json.loads(before.stdout)

            changed_header = header_paths[2]
            changed_header.write_text("changed-header-content\n", encoding="utf-8")
            after = subprocess.run(
                command,
                input=fingerprint_program,
                text=True,
                capture_output=True,
                check=False,
            )
            self.assertEqual(after.returncode, 0, after.stderr)
            after_fingerprint = json.loads(after.stdout)
            self.assertNotEqual(before_fingerprint, after_fingerprint)
            self.assertNotEqual(
                before_fingerprint["sdk_headers"][2]["sha256"],
                after_fingerprint["sdk_headers"][2]["sha256"],
            )


if __name__ == "__main__":
    unittest.main()

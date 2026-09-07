import math
import time
import unittest

from go_m8010_arm_hardware.state_model import (
    GEAR_RATIO,
    JOINT_NAMES,
    MOTOR_NAMES,
    MirrorSessionReferenceV1,
    MotorFeedback,
    PositionSpanVelocityObserver,
    WORKER_COMMAND_PORT_BY_DOMAIN,
    WORKER_CONTROL_DOMAINS,
    WORKER_SUPERVISOR_STATUS_SCHEMA,
    WorkerSupervisorStatusError,
    parse_controller_feedback_metadata,
    parse_feedback_payload,
    unavailable_worker_control_status,
    validate_j6_feedback_identity,
    validate_worker_supervisor_status,
)


class PositionSpanVelocityObserverTest(unittest.TestCase):
    def test_stationary_encoder_span_rejects_vendor_dq_noise(self):
        observer = PositionSpanVelocityObserver()
        start = 1_000_000_000
        jitter = (0.0, 0.0001, -0.0001, 0.00005, 0.0)
        result = None
        for index, value in enumerate(jitter):
            result = observer.update(
                start + index * 100_000_000,
                [value, -value, value, -value, value, -value],
            )
        self.assertIsNotNone(result)
        assert result is not None
        self.assertTrue(all(abs(value) < math.radians(0.25) for value in result))

    def test_monotonic_motion_preserves_direction_and_rate(self):
        start = 2_000_000_000
        for direction in (1.0, -1.0):
            observer = PositionSpanVelocityObserver()
            result = None
            rate = direction * math.radians(1.0)
            for index in range(5):
                elapsed_s = index * 0.1
                result = observer.update(
                    start + index * 100_000_000,
                    [rate * elapsed_s] * 6,
                )
            self.assertIsNotNone(result)
            assert result is not None
            for value in result:
                self.assertAlmostEqual(value, rate, places=10)

    def test_subwindow_is_fail_closed_and_order_is_strict(self):
        observer = PositionSpanVelocityObserver()
        self.assertIsNone(observer.update(3_000_000_000, [0.0] * 6))
        self.assertIsNone(observer.update(3_300_000_000, [0.0] * 6))
        with self.assertRaises(ValueError):
            observer.update(3_300_000_000, [0.0] * 6)
        self.assertIsNone(observer.update(4_000_000_000, [0.0] * 6))


class WorkerSupervisorStatusTest(unittest.TestCase):
    @staticmethod
    def status(now_ns=5_000_000_000):
        return {
            "schema": WORKER_SUPERVISOR_STATUS_SCHEMA,
            "supervisor_instance_id": "20260830T120000-1234",
            "supervisor_pid": 1234,
            "sequence": 9,
            "source_monotonic_ns": now_ns - 10_000_000,
            "domains": {
                domain: {
                    "worker_pid": 2000 + index,
                    "command_port": WORKER_COMMAND_PORT_BY_DOMAIN[domain],
                    "process_alive": True,
                    "udp_owner_confirmed": True,
                    "control_available": True,
                    "reason": "ready",
                }
                for index, domain in enumerate(WORKER_CONTROL_DOMAINS)
            },
        }

    def test_dead_j2_is_distinct_from_fresh_motor_telemetry(self):
        now_ns = 5_000_000_000
        status = self.status(now_ns)
        status["domains"]["J2"].update({
            "process_alive": False,
            "udp_owner_confirmed": False,
            "control_available": False,
            "reason": "process_exited",
        })

        result = validate_worker_supervisor_status(status, now_ns, 1_000_000_000)

        self.assertFalse(result["control_available_by_domain"]["J2"])
        self.assertEqual(
            result["control_reason_by_domain"]["J2"], "process_exited"
        )
        self.assertTrue(result["control_available_by_domain"]["J1"])

    def test_status_age_and_availability_claim_fail_closed(self):
        now_ns = 5_000_000_000
        stale = self.status(now_ns)
        stale["source_monotonic_ns"] = now_ns - 1_000_000_001
        with self.assertRaisesRegex(WorkerSupervisorStatusError, "stale"):
            validate_worker_supervisor_status(stale, now_ns, 1_000_000_000)

        forged = self.status(now_ns)
        forged["domains"]["J2"]["process_alive"] = False
        with self.assertRaisesRegex(WorkerSupervisorStatusError, "invalid"):
            validate_worker_supervisor_status(forged, now_ns, 1_000_000_000)

    def test_missing_supervisor_shape_is_explicitly_all_domain_unavailable(self):
        result = unavailable_worker_control_status("supervisor_status_missing")
        self.assertEqual(set(result["control_available_by_domain"]), set(
            WORKER_CONTROL_DOMAINS
        ))
        self.assertFalse(any(result["control_available_by_domain"].values()))
        self.assertEqual(
            set(result["control_reason_by_domain"].values()),
            {"supervisor_status_missing"},
        )


class ControllerFeedbackMetadataTest(unittest.TestCase):
    @staticmethod
    def payload(motors=("J1",), now=None):
        stamp = time.monotonic_ns() if now is None else now
        logical_feedforward = [0.1, 0.4, -0.2, 0.1, -0.05, 0.0]
        sample_feedforward = {
            "J1": logical_feedforward[0],
            "J2A": -logical_feedforward[1],
            "J2B": logical_feedforward[1],
            "J3": logical_feedforward[2],
            "J4": -logical_feedforward[3],
            "J5": logical_feedforward[4],
        }
        return {
            "schema": "go-m8010-motor-feedback/1.0",
            "source_monotonic_ns": stamp,
            **(
                {
                    "source_instance_id": "9" * 32,
                    "sequence": 1,
                    "session_id": (
                        "persistent:0123456789abcdef:"
                        "j2session:1111111111111111:"
                        "goauxsession:2222222222222222"
                    ),
                    "state_instance_id": "8" * 32,
                }
                if set(motors) == {"J6"} else {}
            ),
            "samples": [
                {
                    "motor": motor,
                    "position_rad": 0.0,
                    "velocity_rad_s": 0.0,
                    "temperature_c": 45.0,
                    "communication_ok": True,
                    "merror": 0,
                    "tau_cmd_rotor_nm": None if motor == "J6" else 0.0,
                    "tau_feedback_rotor_nm": None if motor == "J6" else 0.0,
                    "tau_joint_estimated_nm": None if motor == "J6" else 0.0,
                    "last_valid_feedback_monotonic_ns": stamp,
                    "thermal_fault_latched": False,
                    "load_limit_no_progress": False,
                    **(
                        {"gravity_feedforward_rotor_nm": sample_feedforward[motor]}
                        if motor != "J6" else {}
                    ),
                }
                for motor in motors
            ],
            "controller_mode": "hold",
            "tau_j2_logical_total_nm": (
                0.0 if set(motors) == {"J2A", "J2B"} else None
            ),
            "controller_mode_by_motor": {
                motor: "hold" for motor in motors
            },
            "domain_fault": False,
            "lease_safe_hold": False,
            "thermal_fault_latched": False,
            "thermal_cooldown_ready": False,
            "thermal_release_observed": False,
            "thermal_rearm_pending": False,
            "thermal_rearm_pending_next_cycle": False,
            "thermal_cooldown_valid_brake_frames": 0,
            "thermal_trip_activation_epoch": 0,
            "thermal_minimum_rearm_epoch": 0,
            "thermal_state": "WARNING",
            "thermal_state_by_motor": {
                motor: "WARNING" for motor in motors
            },
            "thermal_trip_reason": "",
            "thermal_config_sha256": (
                "1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467"
            ),
            "no_progress_fault": False,
            "load_limit_fault": False,
            "load_limit_no_progress": False,
            "no_progress_release_observed": False,
            "no_progress_rearm_pending_next_cycle": False,
            "no_progress_watchdog_qualifying_frames": 0,
            "no_progress_observation_valid": False,
            "no_progress_position_error_rad": 0.0,
            "no_progress_trip_position_error_rad": 0.0,
            "position_safety_trip_reason": "",
            "software_saturation_observed": False,
            "load_limit_watchdog_authority": (
                "SOFTWARE_GUARD_NOT_CONTINUOUS_RATING"
            ),
            "no_progress_trip_activation_epoch": 0,
            "no_progress_minimum_rearm_epoch": 0,
            "gravity_authority_present": True,
            "gravity_policy_identity_status": "ECHOED",
            "gravity_source_instance_id": "1" * 32,
            "gravity_session_id": "vertical-session",
            "gravity_state_instance_id": "2" * 32,
            "gravity_model_sha256": (
                "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
            ),
            "gravity_config_sha256": (
                "307469b8384fd35547327ba1d5f80aa440e6b9663406ab7c9d9469bea263335d"
            ),
            "gravity_continuous_rotor_limits_authoritative": True,
            "gravity_scale": 0.5,
            "gravity_scale_target": 0.5,
            "feedforward_nm": logical_feedforward,
        }

    def test_j6_raw_identity_is_session_bound_and_strictly_increasing(self):
        now = 5_000_000_000
        payload = self.payload(("J6",), now)
        first = validate_j6_feedback_identity(
            payload,
            now,
            expected_session_id=payload["session_id"],
            expected_state_instance_id=payload["state_instance_id"],
        )
        replay = dict(payload)
        with self.assertRaisesRegex(ValueError, "replayed"):
            validate_j6_feedback_identity(replay, now, previous=first)
        changed = dict(payload)
        changed["sequence"] = 2
        changed["source_monotonic_ns"] = now + 1
        changed["source_instance_id"] = "7" * 32
        with self.assertRaisesRegex(ValueError, "changed"):
            validate_j6_feedback_identity(
                changed, now + 1, previous=first
            )

    def parse(self, payload):
        receipt = payload["source_monotonic_ns"] + 1
        samples = parse_feedback_payload(payload, receipt)
        return parse_controller_feedback_metadata(payload, samples)

    def test_go_metadata_is_atomic_machine_readable_and_not_a_rating(self):
        result = self.parse(self.payload())
        record = result["J1"]
        self.assertEqual(record["controller_mode"], "hold")
        self.assertFalse(record["domain_fault"])
        self.assertEqual(record["thermal"]["metadata_status"], "OBSERVED")
        self.assertFalse(record["thermal"]["fault_latched"])
        self.assertEqual(
            record["no_progress"]["metadata_status"], "OBSERVED"
        )
        self.assertFalse(
            record["no_progress"]["continuous_rating_authoritative"]
        )
        self.assertEqual(record["no_progress"]["trip_reason"], "")
        self.assertEqual(record["gravity"]["metadata_status"], "OBSERVED")
        self.assertEqual(record["gravity"]["policy_identity_status"], "ECHOED")
        self.assertEqual(
            record["gravity"]["source_instance_id"], "1" * 32
        )
        self.assertEqual(record["gravity"]["applied_rotor_nm"], 0.1)

    def test_teach_feedback_preserves_selected_motors_and_safety_metadata(self):
        for motors, selected in (
            (("J1",), {"J1"}),
            (("J2A", "J2B"), {"J2A", "J2B"}),
            (("J3", "J4", "J5"), {"J3"}),
            (("J3", "J4", "J5"), {"J4"}),
            (("J3", "J4", "J5"), {"J5"}),
        ):
            payload = self.payload(motors)
            payload["controller_mode"] = "teach"
            payload["controller_mode_by_motor"] = {
                motor: "teach" if motor in selected else "hold" for motor in motors
            }
            result = self.parse(payload)
            for motor in motors:
                self.assertEqual(result[motor]["controller_mode"],
                                 "teach" if motor in selected else "hold")
                self.assertFalse(result[motor]["domain_fault"])
                self.assertFalse(result[motor]["lease_safe_hold"])
                self.assertFalse(result[motor]["thermal"]["fault_latched"])
            payload["domain_fault"] = True
            self.assertTrue(all(record["domain_fault"] for record in self.parse(payload).values()))
            with self.assertRaisesRegex(ValueError, "stale"):
                parse_feedback_payload(payload, payload["source_monotonic_ns"] + 100_000_001)
            payload["controller_mode_by_motor"][motors[0]] = "invalid"
            with self.assertRaisesRegex(ValueError, "controller mode"):
                self.parse(payload)

    def test_j2_physical_feedforward_signs_are_checked(self):
        payload = self.payload(("J2A", "J2B"))
        result = self.parse(payload)
        self.assertEqual(result["J2A"]["gravity"]["applied_rotor_nm"], -0.4)
        self.assertEqual(result["J2B"]["gravity"]["applied_rotor_nm"], 0.4)
        payload["samples"][0]["gravity_feedforward_rotor_nm"] = 0.4
        with self.assertRaisesRegex(ValueError, "gravity feedforward disagrees"):
            self.parse(payload)

    def test_legacy_optional_groups_are_unknown_not_healthy_defaults(self):
        now = time.monotonic_ns()
        payload = {
            "schema": "go-m8010-motor-feedback/1.0",
            "source_monotonic_ns": now,
            "samples": [{
                "motor": "J1",
                "position_rad": 0.0,
                "velocity_rad_s": 0.0,
                "temperature_c": 25.0,
                "communication_ok": True,
                "merror": 0,
                "tau_cmd_rotor_nm": 0.0,
                "tau_feedback_rotor_nm": 0.0,
                "tau_joint_estimated_nm": 0.0,
                "last_valid_feedback_monotonic_ns": now,
            }],
            "tau_j2_logical_total_nm": None,
            "controller_mode": "brake",
            "controller_mode_by_motor": {"J1": "brake"},
            "domain_fault": False,
        }
        record = self.parse(payload)["J1"]
        self.assertEqual(record["thermal"]["metadata_status"], "UNKNOWN")
        self.assertIsNone(record["thermal"]["fault_latched"])
        self.assertEqual(record["no_progress"]["metadata_status"], "UNKNOWN")
        self.assertIsNone(record["no_progress"]["fault_latched"])
        self.assertEqual(record["gravity"]["metadata_status"], "UNKNOWN")
        self.assertIsNone(record["gravity"]["authority_present"])
        self.assertIsNone(record["lease_safe_hold"])

    def test_partial_or_contradictory_safety_groups_are_rejected(self):
        mutations = (
            lambda value: value.pop("thermal_cooldown_ready"),
            lambda value: value.__setitem__("load_limit_fault", True),
            lambda value: value.__setitem__(
                "position_safety_trip_reason", "POSITION_ARRIVAL_TIMEOUT"
            ),
            lambda value: value.__setitem__(
                "load_limit_watchdog_authority", "CONTINUOUS_RATING"
            ),
            lambda value: value.__setitem__("gravity_scale_target", 0.4),
            lambda value: value["samples"][0].__setitem__(
                "thermal_fault_latched", True
            ),
        )
        for mutation in mutations:
            payload = self.payload()
            mutation(payload)
            with self.assertRaises(ValueError):
                self.parse(payload)

    def test_position_arrival_timeout_reason_is_atomic_with_latch(self):
        for reason in (
            "POSITION_ARRIVAL_TIMEOUT",
            "EXACT_TRAJECTORY_LOAD_GOVERNOR_ABORT",
        ):
            payload = self.payload()
            payload.update({
                "no_progress_fault": True,
                "load_limit_fault": True,
                "load_limit_no_progress": True,
                "no_progress_trip_position_error_rad": 0.25,
                "position_safety_trip_reason": reason,
                "no_progress_trip_activation_epoch": 7,
                "no_progress_minimum_rearm_epoch": 8,
            })
            for sample in payload["samples"]:
                sample["load_limit_no_progress"] = True
            record = self.parse(payload)["J1"]["no_progress"]
            self.assertTrue(record["fault_latched"])
            self.assertEqual(record["trip_reason"], reason)
            self.assertEqual(record["trip_position_error_rad"], 0.25)

    def test_j6_extended_thermal_metadata_is_strict(self):
        payload = self.payload(("J6",))
        for field in (
            "gravity_authority_present",
            "gravity_policy_identity_status",
            "gravity_source_instance_id",
            "gravity_session_id",
            "gravity_state_instance_id",
            "gravity_model_sha256",
            "gravity_config_sha256",
            "gravity_continuous_rotor_limits_authoritative",
            "gravity_scale",
            "gravity_scale_target",
            "feedforward_nm",
        ):
            payload.pop(field)
        payload.update({
            "thermal_state": "DERATING",
            "thermal_state_by_motor": {"J6": "DERATING"},
            "thermal_derating_factor": 0.6,
            "thermal_raw_temperature_c": 57.0,
            "thermal_window_median_c": 56.0,
            "thermal_slope_c_per_min": 0.5,
            "load_limit_watchdog_authority": (
                "J6_TARGET_TIMEOUT_POSITION_ERROR_V1"
            ),
        })
        payload["samples"][0]["temperature_c"] = 57.0
        result = self.parse(payload)["J6"]
        self.assertEqual(result["thermal"]["reported_state"], "DERATING")
        self.assertEqual(result["thermal"]["derating_factor"], 0.6)
        self.assertEqual(result["gravity"]["metadata_status"], "UNKNOWN")
        payload["thermal_derating_factor"] = 1.1
        with self.assertRaisesRegex(ValueError, "outside"):
            self.parse(payload)

    def test_j345_thermal_state_map_preserves_cross_temperature_states(self):
        payload = self.payload(("J3", "J4", "J5"))
        temperatures = {"J3": 44.0, "J4": 55.0, "J5": 57.0}
        states = {"J3": "NORMAL", "J4": "DERATING", "J5": "DERATING"}
        for sample in payload["samples"]:
            sample["temperature_c"] = temperatures[sample["motor"]]
        payload.update({
            "thermal_state": "DERATING",
            "thermal_state_by_motor": states,
            "thermal_raw_temperature_c": 57.0,
            "thermal_derating_factor": 0.6,
        })
        result = self.parse(payload)
        self.assertEqual(
            {name: result[name]["thermal"]["reported_state"] for name in states},
            states,
        )
        for name in states:
            self.assertEqual(
                result[name]["thermal"]["domain_reported_state"], "DERATING"
            )

    def test_thermal_state_map_hash_and_trip_reason_are_strict(self):
        mutations = (
            lambda value: value["thermal_state_by_motor"].pop("J5"),
            lambda value: value["thermal_state_by_motor"].update(J6="NORMAL"),
            lambda value: value["thermal_state_by_motor"].update(J4="HOT"),
            lambda value: value.__setitem__("thermal_config_sha256", "0" * 64),
            lambda value: value.__setitem__(
                "thermal_trip_reason", "RAW_TEMPERATURE_LIMIT"
            ),
            lambda value: value.__setitem__("thermal_trip_reason", "UNKNOWN"),
        )
        for mutation in mutations:
            payload = self.payload(("J3", "J4", "J5"))
            mutation(payload)
            with self.assertRaises(ValueError):
                self.parse(payload)

        payload = self.payload(("J3", "J4", "J5"))
        payload.update({
            "thermal_fault_latched": True,
            "thermal_state": "THERMAL_STOP",
            "thermal_state_by_motor": {
                name: "THERMAL_STOP" for name in ("J3", "J4", "J5")
            },
            "thermal_trip_reason": "EXACT_TRAJECTORY_DERATING_ABORT",
            "thermal_trip_activation_epoch": 4,
            "thermal_minimum_rearm_epoch": 5,
        })
        for sample in payload["samples"]:
            sample["thermal_fault_latched"] = True
        result = self.parse(payload)
        self.assertEqual(
            result["J4"]["thermal"]["trip_reason"],
            "EXACT_TRAJECTORY_DERATING_ABORT",
        )

    def test_no_progress_authority_is_bound_to_producer(self):
        go_record = self.parse(self.payload())["J1"]["no_progress"]
        self.assertEqual(
            go_record["watchdog_authority"],
            "SOFTWARE_GUARD_NOT_CONTINUOUS_RATING",
        )
        self.assertFalse(go_record["continuous_rating_authoritative"])

        j6_payload = self.payload(("J6",))
        j6_payload["load_limit_watchdog_authority"] = (
            "J6_TARGET_TIMEOUT_POSITION_ERROR_V1"
        )
        j6_record = self.parse(j6_payload)["J6"]["no_progress"]
        self.assertEqual(
            j6_record["watchdog_authority"],
            "J6_TARGET_TIMEOUT_POSITION_ERROR_V1",
        )
        self.assertFalse(j6_record["continuous_rating_authoritative"])

        j6_payload["load_limit_watchdog_authority"] = "UNKNOWN"
        with self.assertRaisesRegex(ValueError, "authority"):
            self.parse(j6_payload)

    def test_gravity_policy_identity_is_strict_and_absent_authority_is_zero(self):
        payload = self.payload()
        payload["gravity_model_sha256"] = "3" * 64
        with self.assertRaisesRegex(ValueError, "identity"):
            self.parse(payload)

        payload = self.payload()
        payload.update({
            "gravity_authority_present": False,
            "gravity_policy_identity_status": "NO_AUTHORITY",
            "gravity_source_instance_id": "",
            "gravity_session_id": "",
            "gravity_state_instance_id": "",
            "gravity_model_sha256": "",
            "gravity_config_sha256": "",
            "gravity_continuous_rotor_limits_authoritative": False,
            "gravity_scale": 0.0,
            "gravity_scale_target": 0.0,
            "feedforward_nm": [0.0] * 6,
        })
        payload["samples"][0]["gravity_feedforward_rotor_nm"] = 0.0
        result = self.parse(payload)["J1"]["gravity"]
        self.assertEqual(result["policy_identity_status"], "NO_AUTHORITY")
        self.assertFalse(result["authority_present"])

        payload["feedforward_nm"][0] = 0.01
        with self.assertRaisesRegex(ValueError, "absent gravity"):
            self.parse(payload)

    def test_empirical_zero_hold_feedback_metadata_is_accepted(self):
        payload = self.payload()
        payload.update({
            "gravity_continuous_rotor_limits_authoritative": False,
            "gravity_scale": 0.0,
            "gravity_scale_target": 0.0,
            "feedforward_nm": [0.0] * 6,
            "gravity_authority_class": "EMPIRICAL_VALIDATION_ENVELOPE",
            "gravity_rating_classification": "NOT_OFFICIAL_CONTINUOUS_RATING",
            "gravity_empirical_envelope_id": "v15-31b-empirical-" + "1" * 20,
            "gravity_empirical_envelope_sha256": "2" * 64,
            "gravity_empirical_envelope_expires_at_utc": "2099-01-01T00:00:00Z",
            "gravity_anchor_sha256": "3" * 64,
            "gravity_empirical_stage_index": 0,
            "gravity_empirical_position_validation_authorized": False,
        })
        for sample in payload["samples"]:
            sample["gravity_feedforward_rotor_nm"] = 0.0
        result = self.parse(payload)
        self.assertTrue(all(
            record["gravity"]["authority_present"] for record in result.values()
        ))


class StateModelTest(unittest.TestCase):
    def make_feedback(self, motor, position, velocity=0.0, now=None):
        now = time.monotonic_ns() if now is None else now
        return MotorFeedback(motor, position, velocity, 31.0, 0, True, now, now)

    def capture(self, model, positions, now):
        for offset in range(model.capture_samples):
            for motor in MOTOR_NAMES:
                model.update(self.make_feedback(motor, positions[motor], now=now + offset))
        self.assertTrue(model.try_capture(now + model.capture_samples))

    def test_seven_motors_become_six_lowercase_joints(self):
        now = time.monotonic_ns()
        model = MirrorSessionReferenceV1(capture_samples=3)
        base = {name: float(index + 1) for index, name in enumerate(MOTOR_NAMES)}
        self.capture(model, base, now)
        deltas = {"J1": 0.1, "J2A": -0.2, "J2B": 0.2, "J3": 0.3, "J4": -0.4, "J5": 0.5, "J6": -0.6}
        # Feed realistic incremental frames; a single rotor jump over pi is
        # directionally ambiguous by definition.
        for step in range(1, 11):
            for motor in MOTOR_NAMES:
                scale = GEAR_RATIO if motor != "J6" else 1.0
                model.update(self.make_feedback(
                    motor,
                    base[motor] + scale * deltas[motor] * step / 10.0,
                    now=now + 10 + step,
                ))
        state = model.snapshot(now + 11)
        self.assertEqual(tuple(state["joint_names"]), JOINT_NAMES)
        expected = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6]
        for actual, wanted in zip(state["position_rad"], expected):
            self.assertAlmostEqual(actual, wanted, places=10)
        self.assertAlmostEqual(state["j2_e_sync_rad"], 0.0, places=10)

    def test_j2_sync_error_is_preserved(self):
        now = time.monotonic_ns()
        model = MirrorSessionReferenceV1(capture_samples=3)
        base = {name: 0.1 * (index + 1) for index, name in enumerate(MOTOR_NAMES)}
        self.capture(model, base, now)
        model.update(self.make_feedback("J2A", base["J2A"] - GEAR_RATIO * 0.08, now=now + 10))
        model.update(self.make_feedback("J2B", base["J2B"] + GEAR_RATIO * 0.06, now=now + 10))
        state = model.snapshot(now + 11)
        self.assertAlmostEqual(state["position_rad"][1], 0.07, places=10)
        self.assertAlmostEqual(state["j2_e_sync_rad"], 0.02, places=10)

    def test_wrap_is_unwrapped_before_session_delta(self):
        now = time.monotonic_ns()
        model = MirrorSessionReferenceV1(capture_samples=3)
        base = {name: 0.0 for name in MOTOR_NAMES}
        base["J1"] = math.pi - 0.01
        self.capture(model, base, now)
        model.update(self.make_feedback("J1", -math.pi + 0.02, now=now + 10))
        state = model.snapshot(now + 11)
        self.assertAlmostEqual(state["position_rad"][0], 0.03 / GEAR_RATIO, places=10)

    def test_persistent_zero_survives_a_new_process_start(self):
        now = time.monotonic_ns()
        zero = {name: float(index + 1) for index, name in enumerate(MOTOR_NAMES)}
        model = MirrorSessionReferenceV1(
            capture_samples=3,
            persistent_references=zero,
            j2_session_references={"J2A": zero["J2A"], "J2B": zero["J2B"]},
            j2_session_hints={"J2A": 0.0, "J2B": 0.0},
            go_aux_session_references={
                name: zero[name] for name in ("J1", "J3", "J4", "J5")
            },
            go_aux_session_hints={
                name: 0.0 for name in ("J1", "J3", "J4", "J5")
            },
        )
        for motor in MOTOR_NAMES:
            model.update(self.make_feedback(motor, zero[motor], now=now))
        self.assertTrue(model.ready)
        state = model.snapshot(now + 1)
        self.assertEqual(state["reference"], "PERSISTENT_SOFTWARE_ZERO_V1")
        self.assertEqual(state["position_rad"], [0.0] * 6)

    def test_persistent_wrapped_reference_aligns_to_nearest_turn(self):
        now = time.monotonic_ns()
        zero = {name: 0.0 for name in MOTOR_NAMES}
        model = MirrorSessionReferenceV1(
            capture_samples=3,
            persistent_references=zero,
            j2_session_references={"J2A": 0.0, "J2B": 0.0},
            j2_session_hints={"J2A": 0.0, "J2B": 0.0},
            go_aux_session_references={
                name: zero[name] for name in ("J1", "J3", "J4", "J5")
            },
            go_aux_session_hints={
                name: 0.0 for name in ("J1", "J3", "J4", "J5")
            },
        )
        for motor in MOTOR_NAMES:
            position = 2.0 * math.pi + 0.1 if motor in {"J1", "J6"} else 0.0
            model.update(self.make_feedback(motor, position, now=now))
        state = model.snapshot(now + 1)
        self.assertAlmostEqual(state["position_rad"][0], 0.1 / GEAR_RATIO)
        self.assertAlmostEqual(state["position_rad"][5], -0.1)

    def test_recovery_hints_restore_geared_multi_turn_branch(self):
        now = time.monotonic_ns()
        zero = {name: 0.2 * (index + 1) for index, name in enumerate(MOTOR_NAMES)}
        hints = {
            "J1": 0.05,
            "J2A": -1.53,
            "J2B": -1.535,
            "J3": 2.71,
            "J4": -0.56,
            "J5": 0.14,
        }
        signs = {"J1": 1, "J2A": -1, "J2B": 1, "J3": 1, "J4": -1, "J5": 1}
        model = MirrorSessionReferenceV1(
            capture_samples=3,
            persistent_references=zero,
            recovery_hints=hints,
            j2_session_references={
                "J2A": zero["J2A"],
                "J2B": zero["J2B"],
            },
            j2_session_hints={
                "J2A": hints["J2A"],
                "J2B": hints["J2B"],
            },
            go_aux_session_references={
                name: zero[name] for name in ("J1", "J3", "J4", "J5")
            },
            go_aux_session_hints={
                name: hints[name] for name in ("J1", "J3", "J4", "J5")
            },
        )
        for index, motor in enumerate(MOTOR_NAMES):
            position = zero[motor]
            if motor in hints:
                position += 2.0 * math.pi * (index - 2)
                position += signs[motor] * GEAR_RATIO * hints[motor]
            model.update(self.make_feedback(motor, position, now=now))
        state = model.snapshot(now + 1)
        expected_j2 = 0.5 * (hints["J2A"] + hints["J2B"])
        expected = [hints["J1"], expected_j2, hints["J3"], hints["J4"], hints["J5"], 0.0]
        for actual, wanted in zip(state["position_rad"], expected):
            self.assertAlmostEqual(actual, wanted, places=10)

    def test_j2_previous_power_raw_is_blocked_without_session_reference(self):
        now = time.monotonic_ns()
        zero = {name: 0.0 for name in MOTOR_NAMES}
        zero.update({"J2A": -4.596924286444928, "J2B": 12.4254104489523})
        model = MirrorSessionReferenceV1(
            capture_samples=3,
            persistent_references=zero,
            recovery_hints={"J2A": 0.0, "J2B": 0.0},
        )
        model.update_batch([
            self.make_feedback("J2A", 5.04623, now=now),
            self.make_feedback("J2B", 2.76539, now=now),
        ])
        self.assertNotIn("J2A", model.references)
        self.assertNotIn("J2B", model.references)
        self.assertEqual(len(model.history["J2A"]), 0)
        self.assertEqual(len(model.history["J2B"]), 0)

    def test_go_aux_previous_power_raw_is_blocked_without_session_reference(self):
        now = time.monotonic_ns()
        zero = {name: 0.0 for name in MOTOR_NAMES}
        model = MirrorSessionReferenceV1(
            capture_samples=3,
            persistent_references=zero,
            recovery_hints={"J3": -0.2882325, "J4": 0.3028612},
            j2_session_references={"J2A": 0.0, "J2B": 0.0},
            j2_session_hints={"J2A": 0.0, "J2B": 0.0},
        )
        for motor, raw in {"J1": 1.0, "J3": 2.0, "J4": -2.0, "J5": 0.5}.items():
            model.update(self.make_feedback(motor, raw, now=now))
            self.assertNotIn(motor, model.references)
            self.assertEqual(len(model.history[motor]), 0)

    def test_go_aux_session_reference_overrides_stale_recovery_hints(self):
        now = time.monotonic_ns()
        zero = {name: 0.0 for name in MOTOR_NAMES}
        session = {"J1": 1.0, "J3": 2.0, "J4": -2.0, "J5": 0.5}
        model = MirrorSessionReferenceV1(
            capture_samples=3,
            persistent_references=zero,
            recovery_hints={"J3": -0.2882325, "J4": 0.3028612},
            j2_session_references={"J2A": 0.0, "J2B": 0.0},
            j2_session_hints={"J2A": 0.0, "J2B": 0.0},
            go_aux_session_references=session,
            go_aux_session_hints={name: 0.0 for name in session},
        )
        for motor, raw in session.items():
            model.update(self.make_feedback(motor, raw, now=now))
        snapshot = model.snapshot_available(now + 1)
        for motor in session:
            self.assertIn(motor, snapshot["available_motors"])
            self.assertAlmostEqual(snapshot["per_motor"][motor]["q_joint_rad"], 0.0)

    def test_real_j2_session_reference_removes_the_26_degree_false_branch(self):
        now = time.monotonic_ns()
        zero = {name: 0.0 for name in MOTOR_NAMES}
        zero.update({"J2A": -4.596924286444928, "J2B": 12.4254104489523})
        startup = {"J2A": 5.04623, "J2B": 2.76539}
        model = MirrorSessionReferenceV1(
            capture_samples=3,
            persistent_references=zero,
            recovery_hints={"J2A": 0.0, "J2B": 0.0},
            j2_session_references=startup,
            j2_session_hints={"J2A": 0.0, "J2B": 0.0},
            go_aux_session_references={
                name: zero[name] for name in ("J1", "J3", "J4", "J5")
            },
            go_aux_session_hints={
                name: 0.0 for name in ("J1", "J3", "J4", "J5")
            },
        )
        for motor in MOTOR_NAMES:
            position = startup[motor] if motor in startup else zero[motor]
            model.update(self.make_feedback(motor, position, now=now))
        state = model.snapshot(now + 1)
        self.assertAlmostEqual(state["position_rad"][1], 0.0, places=12)
        self.assertAlmostEqual(state["j2_e_sync_rad"], 0.0, places=12)

        displacement = math.radians(1.0)
        model.update_batch([
            self.make_feedback(
                "J2A", startup["J2A"] - GEAR_RATIO * displacement, now=now + 2
            ),
            self.make_feedback(
                "J2B", startup["J2B"] + GEAR_RATIO * displacement, now=now + 2
            ),
        ])
        state = model.snapshot(now + 3)
        self.assertAlmostEqual(state["position_rad"][1], displacement, places=10)

    def test_j2_session_reference_rejects_startup_far_from_trusted_hint(self):
        now = time.monotonic_ns()
        zero = {name: 0.0 for name in MOTOR_NAMES}
        model = MirrorSessionReferenceV1(
            capture_samples=3,
            persistent_references=zero,
            j2_session_references={"J2A": 5.04623, "J2B": 2.76539},
            j2_session_hints={"J2A": 0.0, "J2B": 0.0},
        )
        offset = GEAR_RATIO * math.radians(3.0)
        with self.assertRaisesRegex(ValueError, "startup mismatch"):
            model.update_batch([
                self.make_feedback("J2A", 5.04623 - offset, now=now),
                self.make_feedback("J2B", 2.76539 + offset, now=now),
            ])
        self.assertNotIn("J2A", model.references)
        self.assertNotIn("J2B", model.references)

    def test_payload_contract(self):
        now = time.monotonic_ns()
        samples = list(parse_feedback_payload({
            "source_monotonic_ns": now,
            "samples": [{
                "motor": "j1", "position_rad": 1.2,
                "velocity_rad_s": 0.0, "temperature_c": 25.0,
                "communication_ok": True, "merror": 0,
                "tau_cmd_rotor_nm": 0.25,
                "tau_feedback_rotor_nm": -3.4453125,
                "tau_joint_estimated_nm": -3.4453125 * GEAR_RATIO,
                "last_valid_feedback_monotonic_ns": now,
                "trajectory_plan_token_id": "a" * 64,
                "trajectory_sha256": "b" * 64,
                "trajectory_state": "RUNNING",
                "trajectory_sample_index": 5,
                "trajectory_interval_count": 20,
            }],
            "tau_j2_logical_total_nm": None,
        }, now))
        self.assertEqual(samples[0].motor, "J1")
        self.assertEqual(samples[0].receipt_monotonic_ns, now)
        self.assertEqual(samples[0].tau_cmd_rotor_nm, 0.25)
        self.assertEqual(samples[0].tau_feedback_rotor_nm, -3.4453125)
        self.assertAlmostEqual(
            samples[0].tau_joint_estimated_nm,
            -3.4453125 * GEAR_RATIO,
        )
        self.assertEqual(samples[0].last_valid_feedback_monotonic_ns, now)
        self.assertEqual(samples[0].trajectory_plan_token_id, "a" * 64)
        self.assertEqual(samples[0].trajectory_sha256, "b" * 64)
        self.assertEqual(samples[0].trajectory_state, "RUNNING")
        self.assertEqual(samples[0].trajectory_sample_index, 5)
        self.assertEqual(samples[0].trajectory_interval_count, 20)

    def test_torque_contract_validates_j2_total_j6_nulls_and_required_fields(self):
        now = time.monotonic_ns()

        def j2_sample(motor, tau_feedback):
            sign = -1 if motor == "J2A" else 1
            return {
                "motor": motor,
                "position_rad": 0.0,
                "velocity_rad_s": 0.0,
                "temperature_c": 25.0,
                "communication_ok": True,
                "merror": 0,
                "tau_cmd_rotor_nm": 0.1 * sign,
                "tau_feedback_rotor_nm": tau_feedback,
                "tau_joint_estimated_nm": sign * tau_feedback * GEAR_RATIO,
                "last_valid_feedback_monotonic_ns": now,
            }

        j2_payload = {
            "source_monotonic_ns": now,
            "samples": [j2_sample("J2A", -0.2), j2_sample("J2B", 0.2)],
            "tau_j2_logical_total_nm": 0.4 * GEAR_RATIO,
        }
        pair = parse_feedback_payload(j2_payload, now)
        self.assertAlmostEqual(
            sum(sample.tau_joint_estimated_nm for sample in pair),
            0.4 * GEAR_RATIO,
        )
        j2_payload["tau_j2_logical_total_nm"] = 0.0
        with self.assertRaisesRegex(ValueError, "disagrees"):
            parse_feedback_payload(j2_payload, now)

        j6_sample = {
            "motor": "J6",
            "position_rad": 0.0,
            "velocity_rad_s": 0.0,
            "temperature_c": 25.0,
            "communication_ok": True,
            "merror": 0,
            "tau_cmd_rotor_nm": None,
            "tau_feedback_rotor_nm": None,
            "tau_joint_estimated_nm": None,
            "last_valid_feedback_monotonic_ns": now,
        }
        j6_payload = {
            "source_monotonic_ns": now,
            "source_instance_id": "9" * 32,
            "sequence": 1,
            "session_id": (
                "persistent:0123456789abcdef:"
                "j2session:1111111111111111:"
                "goauxsession:2222222222222222"
            ),
            "state_instance_id": "8" * 32,
            "samples": [j6_sample],
            "tau_j2_logical_total_nm": None,
        }
        self.assertEqual(parse_feedback_payload(j6_payload, now)[0].motor, "J6")
        j6_sample["tau_cmd_rotor_nm"] = 0.0
        with self.assertRaisesRegex(ValueError, "POS_VEL"):
            parse_feedback_payload(j6_payload, now)

        required_payload = {
            "source_monotonic_ns": now,
            "samples": [j2_sample("J1", 0.0)],
            "tau_j2_logical_total_nm": None,
        }
        for field in (
            "tau_cmd_rotor_nm",
            "tau_feedback_rotor_nm",
            "tau_joint_estimated_nm",
            "last_valid_feedback_monotonic_ns",
        ):
            missing = {
                **required_payload,
                "samples": [dict(required_payload["samples"][0])],
            }
            missing["samples"][0].pop(field)
            with self.assertRaisesRegex(ValueError, "required"):
                parse_feedback_payload(missing, now)
        missing_total = dict(required_payload)
        missing_total.pop("tau_j2_logical_total_nm")
        with self.assertRaisesRegex(ValueError, "tau_j2_logical_total_nm is required"):
            parse_feedback_payload(missing_total, now)

    def test_payload_contract_rejects_duplicate_nonfinite_and_coerced_health(self):
        now = time.monotonic_ns()
        invalid_payloads = (
            {"source_monotonic_ns": now, "samples": [
                {"motor": "J1", "position_rad": 0.0,
                 "velocity_rad_s": 0.0, "temperature_c": 25.0,
                 "communication_ok": True, "merror": 0},
                {"motor": "J1", "position_rad": 0.1,
                 "velocity_rad_s": 0.0, "temperature_c": 25.0,
                 "communication_ok": True, "merror": 0},
            ]},
            {"source_monotonic_ns": now, "samples": [{
                "motor": "J1", "position_rad": float("nan"),
                "velocity_rad_s": 0.0, "temperature_c": 25.0,
                "communication_ok": True, "merror": 0,
            }]},
            {"source_monotonic_ns": now, "samples": [{
                "motor": "J1", "position_rad": 0.0,
                "velocity_rad_s": 0.0, "temperature_c": 25.0,
                "communication_ok": "false",
                "merror": 0,
            }]},
            {"source_monotonic_ns": now, "samples": [{
                "motor": "J1", "position_rad": 0.0,
                "velocity_rad_s": 0.0, "temperature_c": 25.0,
                "communication_ok": True,
            }]},
            {"source_monotonic_ns": now, "samples": [{
                "motor": "J1", "position_rad": 0.0,
                "velocity_rad_s": 0.0, "temperature_c": 25.0,
                "communication_ok": True, "merror": 0,
                "tau_feedback_rotor_nm": float("nan"),
            }]},
            {"source_monotonic_ns": now, "samples": [{
                "motor": "J1", "position_rad": 0.0,
                "velocity_rad_s": 0.0, "temperature_c": 25.0,
                "communication_ok": True, "merror": 0,
                "trajectory_state": "RUNNING",
                "trajectory_sample_index": 1,
                "trajectory_interval_count": 10,
            }]},
            {"source_monotonic_ns": now, "samples": [{
                "motor": "J1", "position_rad": 0.0,
                "velocity_rad_s": 0.0, "temperature_c": 25.0,
                "communication_ok": True, "merror": 0,
                "trajectory_plan_token_id": "a" * 64,
                "trajectory_sha256": "b" * 64,
                "trajectory_state": "RUNNING",
                "trajectory_sample_index": 11,
                "trajectory_interval_count": 10,
            }]},
        )
        for payload in invalid_payloads:
            with self.assertRaises(ValueError):
                list(parse_feedback_payload(payload, now))

    def test_payload_rejects_stale_replayed_and_coerced_source_fields(self):
        now = time.monotonic_ns()
        base = {
            "source_monotonic_ns": now,
            "samples": [{
                "motor": "J1", "position_rad": 0.0,
                "velocity_rad_s": 0.0, "temperature_c": 25.0,
                "communication_ok": True, "merror": 0,
            }],
        }
        invalid = []
        for source in (str(now), True, now + 1, now - 100_000_001):
            payload = {
                "source_monotonic_ns": source,
                "samples": [dict(base["samples"][0])],
            }
            invalid.append(payload)
        for field, value in (
            ("position_rad", "0.0"),
            ("velocity_rad_s", False),
            ("temperature_c", "25"),
        ):
            sample = dict(base["samples"][0])
            sample[field] = value
            invalid.append({"source_monotonic_ns": now, "samples": [sample]})
        for payload in invalid:
            with self.assertRaises(ValueError):
                list(parse_feedback_payload(payload, now))

    def test_payload_requires_a_complete_fault_domain_motor_set(self):
        now = time.monotonic_ns()

        def sample(motor):
            sign = -1.0 if motor == "J2A" else 1.0
            return {
                "motor": motor, "position_rad": 0.0,
                "velocity_rad_s": 0.0, "temperature_c": 25.0,
                "communication_ok": True, "merror": 0,
                "tau_cmd_rotor_nm": 0.0,
                "tau_feedback_rotor_nm": sign,
                "tau_joint_estimated_nm": sign * (
                    -1.0 if motor == "J2A" else 1.0
                ) * GEAR_RATIO,
                "last_valid_feedback_monotonic_ns": now,
            }

        with self.assertRaises(ValueError):
            list(parse_feedback_payload({
                "source_monotonic_ns": now,
                "samples": [sample("J2A")],
                "tau_j2_logical_total_nm": None,
            }, now))
        pair = list(parse_feedback_payload({
            "source_monotonic_ns": now,
            "samples": [sample("J2A"), sample("J2B")],
            "tau_j2_logical_total_nm": 2.0 * GEAR_RATIO,
        }, now))
        self.assertEqual({item.motor for item in pair}, {"J2A", "J2B"})

    def test_model_rejects_per_motor_source_replay_without_state_change(self):
        now = time.monotonic_ns()
        model = MirrorSessionReferenceV1(capture_samples=3)
        accepted = self.make_feedback("J1", 0.25, now=now)
        model.update(accepted)
        with self.assertRaises(ValueError):
            model.update(MotorFeedback(
                "J1", 1.25, 0.0, 25.0, 0, True, now, now + 1,
            ))
        self.assertEqual(model.latest["J1"], accepted)
        self.assertAlmostEqual(model.latest_unwrapped["J1"], 0.25)

    def test_feedback_batch_is_atomic_when_later_sample_is_invalid(self):
        now = time.monotonic_ns()
        model = MirrorSessionReferenceV1(capture_samples=3)
        first = self.make_feedback("J2A", 0.25, now=now)
        second = self.make_feedback("J2B", float("nan"), now=now)
        with self.assertRaises(ValueError):
            model.update_batch([first, second])
        self.assertNotIn("J2A", model.latest)
        self.assertIsNone(model.unwrappers["J2A"].previous)

    def test_partial_reference_does_not_block_other_joints(self):
        now = time.monotonic_ns()
        model = MirrorSessionReferenceV1(capture_samples=3, freshness_s=1.0)
        for offset in range(3):
            model.update(self.make_feedback("J4", 1.0, now=now + offset))
        self.assertTrue(model.try_capture_available(now + 10))
        state = model.snapshot_available(now + 10)
        self.assertEqual(state["available_motors"], ["J4"])
        self.assertEqual(state["position_rad"], [0.0] * 6)
        self.assertTrue(state["per_motor"]["J4"]["communication_ok"])
        self.assertFalse(state["per_motor"]["J2A"]["communication_ok"])


if __name__ == "__main__":
    unittest.main()

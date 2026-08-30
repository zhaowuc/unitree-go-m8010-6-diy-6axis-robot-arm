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
    parse_feedback_payload,
    unavailable_worker_control_status,
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
        deltas = {"J1": 0.1, "J2A": -0.2, "J2B": 0.2, "J3": 0.3, "J4": 0.4, "J5": 0.5, "J6": -0.6}
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
        signs = {"J1": 1, "J2A": -1, "J2B": 1, "J3": 1, "J4": 1, "J5": 1}
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
            }],
        }, now))
        self.assertEqual(samples[0].motor, "J1")
        self.assertEqual(samples[0].receipt_monotonic_ns, now)

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
            return {
                "motor": motor, "position_rad": 0.0,
                "velocity_rad_s": 0.0, "temperature_c": 25.0,
                "communication_ok": True, "merror": 0,
            }

        with self.assertRaises(ValueError):
            list(parse_feedback_payload({
                "source_monotonic_ns": now,
                "samples": [sample("J2A")],
            }, now))
        pair = list(parse_feedback_payload({
            "source_monotonic_ns": now,
            "samples": [sample("J2A"), sample("J2B")],
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

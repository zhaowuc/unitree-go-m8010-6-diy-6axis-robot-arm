import math
import time
import unittest

from go_m8010_arm_hardware.state_model import (
    GEAR_RATIO,
    JOINT_NAMES,
    MOTOR_NAMES,
    MirrorSessionReferenceV1,
    MotorFeedback,
    parse_feedback_payload,
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

    def test_payload_contract(self):
        now = time.monotonic_ns()
        samples = list(parse_feedback_payload({"samples": [{"motor": "j1", "position_rad": 1.2}]}, now))
        self.assertEqual(samples[0].motor, "J1")
        self.assertEqual(samples[0].receipt_monotonic_ns, now)


if __name__ == "__main__":
    unittest.main()

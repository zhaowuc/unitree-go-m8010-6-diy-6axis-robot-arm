import copy
from dataclasses import replace
import math
import pytest
from hand_guidance_feedback import matched_observation, stationary_bias


def test_paired_gravity_motor_units_frozen_bias_and_stale_rejection():
    now = 10_000_000_000
    names = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
    hardware = {"session_id": "s", "state_instance_id": "i", "sequence": 4,
        "source_monotonic_ns": now - 10_000_000, "healthy": True, "safety_metadata_ready": True,
        "j2_sync_fault": False, "j2_e_sync_rad": 0.0, "position_rad": [0.0] * 6,
        "velocity_rad_s": [0.0] * 6, "controller_mode_by_motor": dict.fromkeys(names, "hold"),
        "per_motor": {name: {"fresh": True, "communication_ok": True, "merror": 0,
            "temperature_c": 32.0, "feedback_source_monotonic_ns": now - 20_000_000,
            "tau_joint_estimated_nm": value} for name, value in zip(names, (0, 1, 2, 4, -0.5, 0.1, None))}}
    hardware["per_motor"]["J6"].update(joint_motor_torque_estimated_nm=-0.02,
        motor_torque_qualification_sha256="a" * 64, motor_torque_source_monotonic_ns=now-20_000_000,
        motor_torque_estimate_metadata_status="OBSERVED")
    gravity = {"session_id": "s", "state_instance_id": "i", "source_monotonic_ns": now,
        "hardware_state_sequence": 4, "hardware_state_source_monotonic_ns": hardware["source_monotonic_ns"],
        "anchor_valid": True, "production_model_hash_match": True, "joint_state_crosscheck": True,
        "finite_bounded": True, "pose_feasibility": "PASS", "gravity_joint_nm": [0, 3.1, 4, -0.5, 0.1, 0.03]}
    assert matched_observation([], gravity, now_ns=now) is None
    sample = matched_observation([hardware], gravity, now_ns=now)
    assert sample.motor_torque_nm == (0, 3, 4, -0.5, 0.1, -0.02)
    assert sample.residual_nm == pytest.approx((0, 0.1, 0, 0, 0, 0.05))
    samples = [replace(sample, source_monotonic_ns=now + i * 20_000_000) for i in range(51)]
    bias, noise = stationary_bias(samples, operator_hands_off=True)
    assert bias == sample.residual_nm and noise == (0.0,) * 6
    pushed = copy.deepcopy(hardware)
    pushed["per_motor"]["J2A"]["tau_joint_estimated_nm"] -= 0.8
    assert matched_observation([pushed], gravity, now_ns=now).residual_nm[1] - bias[1] == pytest.approx(0.8)
    with pytest.raises(ValueError, match="HANDS_OFF"):
        stationary_bias(samples, operator_hands_off=False)
    with pytest.raises(ValueError, match="BASELINE_POSE_MOVED"):
        stationary_bias(samples[:-1] + [replace(samples[-1], q=(0, math.radians(0.2), 0, 0, 0, 0))], operator_hands_off=True)
    invalid = copy.deepcopy(hardware)
    invalid["per_motor"]["J6"]["motor_torque_estimate_metadata_status"] = "STALE"
    with pytest.raises(ValueError, match="J6_TORQUE_OBSERVATION_UNQUALIFIED"):
        matched_observation([invalid], gravity, now_ns=now)
    with pytest.raises(ValueError, match="STALE"):
        matched_observation([hardware], gravity, now_ns=now + 101_000_000)
    with pytest.raises(ValueError, match="SOURCE_MISMATCH"):
        matched_observation([hardware], {**gravity, "hardware_state_source_monotonic_ns": now-1}, now_ns=now)

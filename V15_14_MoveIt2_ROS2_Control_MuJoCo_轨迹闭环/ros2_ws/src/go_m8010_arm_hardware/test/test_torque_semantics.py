import math

import pytest

from go_m8010_arm_hardware.torque_semantics import (
    GO_GEAR_RATIO,
    decode_go_tau_feedback,
    encode_go_tau_q8,
    j2_rotor_feedback_to_logical_total,
    joint_torque_to_rotor_torque,
    rotor_torque_to_joint_torque,
    split_j2_joint_torque,
)


@pytest.mark.parametrize(
    ("literal_nm", "expected_count", "decoded_nm"),
    (
        (-129.0, -32765, -127.98828125),
        (-0.05, -12, -0.046875),
        (-0.00390625, -1, -0.00390625),
        (0.0, 0, 0.0),
        (0.00390625, 1, 0.00390625),
        (0.05, 12, 0.046875),
        (129.0, 32765, 127.98828125),
    ),
)
def test_go_q8_matches_frozen_sdk_golden_cases(
    literal_nm, expected_count, decoded_nm,
):
    count = encode_go_tau_q8(literal_nm)
    assert count == expected_count
    assert decode_go_tau_feedback(count) == decoded_nm


def test_joint_rotor_round_trip_includes_direction_and_one_reducer_conversion():
    for sign in (-1, +1):
        rotor_nm = joint_torque_to_rotor_torque(
            12.5, direction_sign=sign,
        )
        assert rotor_torque_to_joint_torque(
            rotor_nm, direction_sign=sign,
        ) == pytest.approx(12.5)


def test_j2_split_is_equal_opposite_and_reconstructs_total_joint_torque():
    split = split_j2_joint_torque(8.0)
    expected_per_rotor = 8.0 / (2.0 * GO_GEAR_RATIO)
    assert split.tau_cmd_j2a_rotor_nm == pytest.approx(-expected_per_rotor)
    assert split.tau_cmd_j2b_rotor_nm == pytest.approx(+expected_per_rotor)
    assert j2_rotor_feedback_to_logical_total(
        split.tau_cmd_j2a_rotor_nm,
        split.tau_cmd_j2b_rotor_nm,
    ) == pytest.approx(8.0)


@pytest.mark.parametrize(
    "call",
    (
        lambda: encode_go_tau_q8(math.nan),
        lambda: decode_go_tau_feedback(32768),
        lambda: joint_torque_to_rotor_torque(1.0, direction_sign=0),
        lambda: joint_torque_to_rotor_torque(
            1.0, direction_sign=1, gear_ratio=0.0,
        ),
        lambda: joint_torque_to_rotor_torque(
            1.0, direction_sign=1, load_share=1.1,
        ),
    ),
)
def test_invalid_torque_inputs_fail_closed(call):
    with pytest.raises(ValueError):
        call()

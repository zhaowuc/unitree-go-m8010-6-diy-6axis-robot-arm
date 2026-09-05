import json
import math
from pathlib import Path
from types import SimpleNamespace

import pytest

from go_m8010_arm_hardware.gravity_model import (
    GRAVITY_ANCHOR_SCHEMA,
    PRODUCTION_MODEL_SHA256,
    GravityAnchorError,
    GravityFeedforwardController,
    GravityModelAnchorV2,
    StaticGravityEvaluator,
    dynamics_only_xml,
    GravityScaleRamp,
    RotorTorqueSlewLimiter,
    gravity_joint_to_rotor_commands,
    gravity_joint_to_logical_rotor_feedforward,
    normalize_named_joint_positions,
)
from go_m8010_arm_hardware.torque_semantics import GO_GEAR_RATIO


def anchor_document():
    return {
        "schema": GRAVITY_ANCHOR_SCHEMA,
        "model_sha256": PRODUCTION_MODEL_SHA256,
        "session_id": "persistent:test-session",
        "state_instance_id": "0123456789abcdef0123456789abcdef",
        "created_utc": "2026-08-30T12:00:00Z",
        "gear_ratio": GO_GEAR_RATIO,
        "motor_direction_sign": {
            "J1": 1, "J2A": -1, "J2B": 1, "J3": 1,
            "J4": -1, "J5": 1, "J6": -1,
        },
        "motor_raw_reference_rad": {
            name: float(index) for index, name in enumerate(
                ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
            )
        },
        "motor_encoder_branch": {
            name: 0 for name in
            ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
        },
        "logical_joint_reference_rad": [0.1] * 6,
        "model_absolute_joint_rad": [0.2] * 6,
    }


def test_anchor_is_session_bound_and_maps_relative_motion_without_zero_write(tmp_path):
    document = anchor_document()
    path = tmp_path / "gravity_model_anchor_v2.json"
    path.write_text(json.dumps(document), encoding="utf-8")
    anchor = GravityModelAnchorV2.from_path(path)
    assert anchor.matches_runtime(
        document["session_id"], document["state_instance_id"]
    )
    assert not anchor.matches_runtime("persistent:new", document["state_instance_id"])
    assert anchor.model_q_from_actual([0.15] * 6) == pytest.approx([0.25] * 6)


@pytest.mark.parametrize(
    ("field", "replacement"),
    (
        ("model_sha256", "0" * 64),
        ("gear_ratio", 6.0),
        ("created_utc", "2026-08-30T12:00:00"),
        ("motor_encoder_branch", {"J1": 0}),
    ),
)
def test_anchor_mismatch_fails_closed(field, replacement):
    document = anchor_document()
    document[field] = replacement
    with pytest.raises(GravityAnchorError):
        GravityModelAnchorV2.from_mapping(document)


def test_gravity_mapping_splits_j2_and_leaves_j6_posvel():
    commands = gravity_joint_to_rotor_commands(
        [6.33, 12.66, -6.33, 0.0, 3.165, 99.0], gravity_scale=0.5,
        gear_ratio=6.33,
    )
    assert commands == pytest.approx({
        "J1": 0.5,
        "J2A": -0.5,
        "J2B": 0.5,
        "J3": -0.5,
        "J4": 0.0,
        "J5": 0.25,
        "J6": 0.0,
    })


def test_j4_feedback_command_and_torque_share_one_direction():
    from go_m8010_arm_hardware.state_model import MOTOR_SPECS

    spec = MOTOR_SPECS["J4"]
    assert spec.sign == -1
    raw_delta = -spec.gear_ratio * math.radians(5)
    logical_delta = spec.sign * raw_delta / spec.gear_ratio
    assert logical_delta == pytest.approx(math.radians(5))
    assert spec.sign * spec.gear_ratio * logical_delta == pytest.approx(raw_delta)
    joint_torque = [0, 0, 0, 6.33, 0, 0]
    rotor = gravity_joint_to_rotor_commands(joint_torque, gravity_scale=1, gear_ratio=6.33)
    logical = gravity_joint_to_logical_rotor_feedforward(joint_torque, gravity_scale=1, gear_ratio=6.33)
    assert logical[3] == 1
    assert rotor["J4"] == spec.sign * logical[3] == -1


def test_gravity_mapping_rejects_nonfinite_or_out_of_range_scale():
    with pytest.raises(ValueError):
        gravity_joint_to_rotor_commands([0.0] * 6, gravity_scale=1.01)
    with pytest.raises(GravityAnchorError):
        gravity_joint_to_rotor_commands(
            [0.0, 0.0, math.nan, 0.0, 0.0, 0.0], gravity_scale=0.0,
        )


def test_logical_rotor_feedforward_uses_half_j2_and_zero_j6():
    assert gravity_joint_to_logical_rotor_feedforward(
        [6.33, 12.66, -6.33, 0.0, 3.165, 100.0],
        gravity_scale=0.5,
        gear_ratio=6.33,
    ) == pytest.approx((0.5, 0.5, -0.5, 0.0, 0.25, 0.0))


def test_feedforward_controller_defaults_zero_and_ramps_two_layers():
    controller = GravityFeedforwardController(
        ramp_seconds=2.0,
        maximum_slew_nm_per_s=1.0,
    )
    gravity = [6.33, 12.66, 6.33, 0.0, 0.0, 0.0]
    scale, logical, physical = controller.step(
        gravity, enabled=False, target_scale=0.25, now_s=10.0,
    )
    assert scale == 0.0
    assert logical == (0.0,) * 6
    assert not any(physical.values())
    # First enabled sample establishes both clocks and remains exactly zero.
    scale, logical, _physical = controller.step(
        gravity, enabled=True, target_scale=0.25, now_s=10.1,
    )
    assert scale == 0.0
    assert logical == (0.0,) * 6
    scale, logical, physical = controller.step(
        gravity, enabled=True, target_scale=0.25, now_s=10.6,
    )
    assert scale == pytest.approx(0.25)
    assert logical == pytest.approx((0.25, 0.25, 0.25, 0.0, 0.0, 0.0))
    assert physical["J2A"] == pytest.approx(-0.25)
    assert physical["J2B"] == pytest.approx(0.25)
    # Invalid scale rungs and disable/re-enable jumps are fail-closed.
    with pytest.raises(ValueError, match="0/25/50/75/100"):
        controller.step(
            gravity, enabled=True, target_scale=0.3, now_s=10.7,
        )
    controller.force_zero(10.8)
    assert controller.step(
        gravity, enabled=False, target_scale=0.0, now_s=10.9,
    )[1] == (0.0,) * 6


def test_static_evaluator_owns_data_and_reads_qfrc_bias(tmp_path, monkeypatch):
    model_path = tmp_path / "model.xml"
    model_path.write_text("<mujoco/>", encoding="utf-8")
    import go_m8010_arm_hardware.gravity_model as module
    monkeypatch.setattr(module, "sha256_file", lambda _path: PRODUCTION_MODEL_SHA256)

    class Vector(list):
        def __setitem__(self, key, value):
            if isinstance(key, slice):
                super().__setitem__(key, list(value))
            else:
                super().__setitem__(key, value)

    class FakeModel:
        nq = 6
        nv = 6
        opt = SimpleNamespace(gravity=Vector([0.0, 0.0, 0.0]))

    class FakeMjModel:
        @staticmethod
        def from_xml_string(_xml):
            return FakeModel()

    class FakeData:
        def __init__(self, _model):
            self.qpos = Vector([0.0] * 6)
            self.qvel = Vector([1.0] * 6)
            self.qacc = Vector([1.0] * 6)
            self.qfrc_bias = Vector([0.0] * 6)
            self.qfrc_inverse = Vector([0.0] * 6)

    def fake_forward(_model, data):
        assert data.qvel == [0.0] * 6
        assert data.qacc == [0.0] * 6
        data.qfrc_bias[:] = [value * 2.0 for value in data.qpos]

    def fake_inverse(_model, data):
        data.qfrc_inverse[:] = [
            position + velocity + acceleration
            for position, velocity, acceleration in zip(
                data.qpos, data.qvel, data.qacc
            )
        ]

    fake_mujoco = SimpleNamespace(
        MjModel=FakeMjModel,
        MjData=FakeData,
        mj_forward=fake_forward,
        mj_inverse=fake_inverse,
    )
    evaluator = StaticGravityEvaluator(model_path, fake_mujoco)
    assert evaluator.evaluate([1, 2, 3, 4, 5, 6]) == (2, 4, 6, 8, 10, 12)
    assert evaluator.model.opt.gravity == [0.0, 0.0, -9.81]
    assert evaluator.evaluate_inverse(
        [1, 2, 3, 4, 5, 6],
        [0.1] * 6,
        [0.2] * 6,
    ) == pytest.approx((1.3, 2.3, 3.3, 4.3, 5.3, 6.3))


def test_dynamics_only_xml_preserves_inertial_chain_without_mesh_payload(tmp_path):
    source = tmp_path / "model.xml"
    source.write_text(
        """<mujoco><size njmax="4000" nconmax="2000"/>
        <visual/><asset><mesh name="large" file="large.stl"/></asset>
        <contact><pair geom1="a" geom2="b"/></contact>
        <worldbody><geom name="ground" type="plane" size="1 1 1"/>
        <body name="link"><joint name="J1"/><inertial mass="1"
        pos="0 0 0" diaginertia="1 1 1"/><geom name="a"
        type="mesh" mesh="large"/><camera name="view"/></body></worldbody>
        </mujoco>""",
        encoding="utf-8",
    )
    derived = dynamics_only_xml(source)
    assert "<joint name=\"J1\"" in derived
    assert "<inertial" in derived
    for forbidden in ("<asset", "<mesh", "<geom", "<contact", "<visual", "<size"):
        assert forbidden not in derived


def test_gravity_scale_ramp_is_bounded_and_can_fail_closed_immediately():
    ramp = GravityScaleRamp(ramp_seconds=2.0)
    ramp.set_target(1.0)
    assert ramp.step(10.0) == 0.0
    assert ramp.step(10.5) == pytest.approx(0.25)
    assert ramp.step(11.0) == pytest.approx(0.50)
    ramp.force_zero(11.1)
    assert ramp.current_scale == 0.0
    assert ramp.target_scale == 0.0
    ramp.set_target(0.5)
    assert ramp.step(11.2) == pytest.approx(0.05)


def test_gravity_scale_ramp_rejects_time_reversal():
    ramp = GravityScaleRamp(ramp_seconds=2.0)
    ramp.step(2.0)
    with pytest.raises(ValueError, match="backwards"):
        ramp.step(1.0)


def test_empirical_fixed_stage_ramp_takes_two_seconds_per_25_percent():
    ramp = GravityScaleRamp(
        ramp_seconds=2.0,
        fixed_transition_duration=True,
    )
    now = 10.0
    assert ramp.step(now) == 0.0
    for target in (0.25, 0.50, 0.75, 1.0):
        start = ramp.current_scale
        ramp.set_target(target)
        assert ramp.step(now) == pytest.approx(start)
        for tick in range(1, 200):
            value = ramp.step(now + tick * 0.01)
            assert start <= value < target
        now += 2.0
        assert ramp.step(now) == pytest.approx(target)


def test_rotor_torque_slew_is_per_motor_and_named_in_rotor_nm():
    limiter = RotorTorqueSlewLimiter(maximum_slew_nm_per_s=2.0)
    target = {
        "J1": 2.0,
        "J2A": -2.0,
        "J2B": 2.0,
        "J3": 1.0,
        "J4": 0.0,
        "J5": -1.0,
        "J6": 0.0,
    }
    assert set(limiter.step(target, 0.0)) == set(target)
    first = limiter.step(target, 0.25)
    assert first["J1"] == pytest.approx(0.5)
    assert first["J2A"] == pytest.approx(-0.5)
    assert first["J4"] == 0.0
    assert limiter.force_zero(0.30) == {name: 0.0 for name in target}


def test_named_joint_state_is_order_independent_and_strict():
    assert normalize_named_joint_positions(
        ["J6", "J4", "J2", "J1", "J5", "J3"],
        [6, 4, 2, 1, 5, 3],
    ) == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    assert normalize_named_joint_positions(
        ["joint6", "joint4", "joint2", "joint1", "joint5", "joint3"],
        [6, 4, 2, 1, 5, 3],
    ) == (1.0, 2.0, 3.0, 4.0, 5.0, 6.0)
    with pytest.raises(ValueError, match="J1..J6"):
        normalize_named_joint_positions(["J1"], [1.0])
    with pytest.raises(ValueError, match="duplicate"):
        normalize_named_joint_positions(
            ["J1", "J1", "J2", "J3", "J4", "J5", "J6"],
            [0.0] * 7,
        )
    with pytest.raises(ValueError, match="duplicate"):
        normalize_named_joint_positions(
            ["J1", "joint1", "J2", "J3", "J4", "J5", "J6"],
            [0.0] * 7,
        )

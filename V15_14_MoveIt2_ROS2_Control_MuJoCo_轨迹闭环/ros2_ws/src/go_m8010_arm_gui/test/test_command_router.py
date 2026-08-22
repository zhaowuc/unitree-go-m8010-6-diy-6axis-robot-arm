import json
import math

import pytest

from go_m8010_arm_gui.command_router import payload_for_domain, validate_command


def command(mode="position", target=0.0, active_joint_mask=None):
    if active_joint_mask is None:
        active_joint_mask = [False] * 6 if mode == "brake" else [True] * 6
    return json.dumps({
        "schema": "go-m8010-gui-command/1.1",
        "sequence": 1,
        "mode": mode,
        "targets_rad": [target] * 6,
        "active_joint_mask": active_joint_mask,
        "activation_epoch": 1,
        "maximum_velocity_rad_s": math.radians(5.0),
        "maximum_acceleration_rad_s2": math.radians(20.0),
    })


def test_valid_six_joint_command():
    value, payload = validate_command(command())
    assert value["mode"] == "position" and len(value["targets_rad"]) == 6
    assert json.loads(payload)["schema"] == "go-m8010-gui-command/1.1"


@pytest.mark.parametrize("mode", ["drag", "hold", "position"])
def test_j2_active_control_is_always_forwarded_as_brake(mode):
    value, _payload = validate_command(command(mode=mode))
    assert json.loads(payload_for_domain(value, "J2"))["mode"] == "brake"
    for domain in ("J1", "J345", "J6"):
        assert json.loads(payload_for_domain(value, domain))["mode"] == mode


def test_j2_brake_request_remains_brake():
    value, _payload = validate_command(command(mode="brake"))
    assert json.loads(payload_for_domain(value, "J2"))["mode"] == "brake"


def test_j4_position_isolated_to_j345_domain_and_one_joint_mask():
    value, _payload = validate_command(command(active_joint_mask=[False, False, False, True, False, False]))
    for domain in ("J1", "J2", "J6"):
        payload = json.loads(payload_for_domain(value, domain))
        assert payload["mode"] == "brake"
        assert payload["active_joint_mask"] == [False] * 6
    j345 = json.loads(payload_for_domain(value, "J345"))
    assert j345["mode"] == "position"
    assert j345["active_joint_mask"] == [False, False, False, True, False, False]


def test_legacy_schema_is_rejected_fail_closed():
    document = json.loads(command())
    document["schema"] = "go-m8010-gui-command/1.0"
    with pytest.raises(ValueError, match="旧版协议仅允许制动"):
        validate_command(json.dumps(document))


def test_legacy_brake_remains_available_for_emergency_compatibility():
    document = json.loads(command(mode="brake"))
    document["schema"] = "go-m8010-gui-command/1.0"
    document.pop("active_joint_mask")
    value, payload = validate_command(json.dumps(document))
    assert value["mode"] == "brake"
    assert value["active_joint_mask"] == [False] * 6
    assert json.loads(payload)["schema"] == "go-m8010-gui-command/1.1"


@pytest.mark.parametrize("mask", [None, [True] * 5, [0, False, False, False, False, False]])
def test_active_joint_mask_must_be_six_strict_booleans(mask):
    document = json.loads(command())
    if mask is None:
        document.pop("active_joint_mask")
    else:
        document["active_joint_mask"] = mask
    with pytest.raises(ValueError, match="关节激活掩码"):
        validate_command(json.dumps(document))


def test_brake_command_with_active_joint_is_rejected():
    with pytest.raises(ValueError, match="制动命令不得携带激活关节"):
        validate_command(command(mode="brake", active_joint_mask=[True] + [False] * 5))


@pytest.mark.parametrize("epoch", [-1, True, 1.5, "1", 1 << 63])
def test_activation_epoch_must_be_bounded_nonnegative_integer(epoch):
    document = json.loads(command())
    document["activation_epoch"] = epoch
    with pytest.raises(ValueError, match="激活纪元"):
        validate_command(json.dumps(document))


def test_active_command_requires_nonzero_activation_epoch():
    document = json.loads(command())
    document["activation_epoch"] = 0
    with pytest.raises(ValueError, match="必须大于零"):
        validate_command(json.dumps(document))


def test_unknown_domain_is_rejected():
    value, _payload = validate_command(command())
    with pytest.raises(ValueError, match="未知硬件故障域"):
        payload_for_domain(value, "UNKNOWN")


@pytest.mark.parametrize("mode", ["real_to_sim", "sim_to_real", "torque"])
def test_non_hardware_modes_rejected(mode):
    with pytest.raises(ValueError):
        validate_command(command(mode=mode))


def test_out_of_envelope_rejected():
    with pytest.raises(ValueError):
        validate_command(command(target=math.radians(10.01)))


@pytest.mark.parametrize("field,value", [
    ("maximum_velocity_rad_s", float("nan")),
    ("maximum_velocity_rad_s", float("inf")),
    ("maximum_acceleration_rad_s2", 0.0),
])
def test_invalid_profile_rejected(field, value):
    document = json.loads(command())
    document[field] = value
    with pytest.raises(ValueError):
        validate_command(json.dumps(document))

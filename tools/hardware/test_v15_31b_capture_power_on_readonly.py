from __future__ import annotations

import importlib.util
import json
import math
from pathlib import Path

import pytest


SCRIPT = Path(__file__).with_name("v15_31b_capture_power_on_readonly.py")
SPEC = importlib.util.spec_from_file_location("v15_31b_power_capture", SCRIPT)
assert SPEC is not None and SPEC.loader is not None
capture = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(capture)


SESSION = "boot-session"
INSTANCE = "3" * 32


def hardware_state(sequence: int, source_ns: int) -> dict:
    positions = [0.001, -0.002, 0.003, -0.004, 0.005, -0.006]
    motor_logical = {
        "J1": positions[0],
        "J2A": positions[1] - math.radians(0.05),
        "J2B": positions[1] + math.radians(0.05),
        "J3": positions[2],
        "J4": positions[3],
        "J5": positions[4],
        "J6": positions[5],
    }
    raw = {
        "J1": 0.1,
        "J2A": 2.0 * math.pi + 0.2,
        "J2B": -2.0 * math.pi - 0.3,
        "J3": 4.0 * math.pi + 0.4,
        "J4": -4.0 * math.pi - 0.5,
        "J5": 0.6,
        "J6": -0.7,
    }
    per_motor = {}
    for name in capture.MOTOR_NAMES:
        is_j6 = name == "J6"
        per_motor[name] = {
            "raw_position_rad": raw[name],
            "q_joint_rad": motor_logical[name],
            "dq_joint_rad_s": 0.0,
            "tau_cmd_rotor_nm": None if is_j6 else 0.0,
            "tau_feedback_rotor_nm": None if is_j6 else 0.1,
            "tau_joint_estimated_nm": None if is_j6 else 0.2,
            "temperature_c": 31.0,
            "merror": 0,
            "communication_ok": True,
            "fresh": True,
            "age_ms": 5.0,
        }
    return {
        "schema": capture.HARDWARE_STATE_SCHEMA,
        "sequence": sequence,
        "source_monotonic_ns": source_ns,
        "session_id": SESSION,
        "state_instance_id": INSTANCE,
        "position_rad": positions,
        "velocity_rad_s": [0.0] * 6,
        "velocity_source": "POSITION_SPAN_0P40S",
        "j2_e_sync_rad": -math.radians(0.10),
        "j2_sync_fault": False,
        "controller_mode_by_motor": {
            name: "brake" for name in capture.MOTOR_NAMES
        },
        "per_motor": per_motor,
    }


def recorder() -> object:
    return capture.PowerOnReadOnlyRecorder(
        model_absolute_joint_rad=[0.0] * 6,
        operator_confirmed_at_utc="2026-08-31T12:00:00Z",
        minimum_duration_ns=capture.MINIMUM_DURATION_NS,
        minimum_valid_frames=2,
    )


def add_ten_second_window(value) -> None:
    for index in range(41):
        value.add_document(
            hardware_state(index + 1, 1_000_000_000 + index * 250_000_000)
        )


def test_builds_anchor_compatible_and_evidence_validator_compatible_document() -> None:
    value = recorder()
    add_ten_second_window(value)
    document = value.build_document()
    assert document["schema"] == capture.POWER_ON_SCHEMA
    assert document["status"] == "PASS"
    assert document["duration_s"] == 10.0
    assert document["valid_frame_count"] == 41
    assert document["session_id"] == SESSION
    assert document["state_instance_id"] == INSTANCE
    assert set(document["per_motor"]) == set(capture.MOTOR_NAMES)
    assert document["per_motor"]["J6"]["tau_feedback_rotor_nm"] is None
    assert document["per_motor"]["J2A"]["max_abs_tau_cmd_rotor_nm"] == 0.0
    assert document["j2"]["max_abs_e_sync_deg"] == pytest.approx(0.10)
    assert document["safety"]["active_command_count"] == 0
    assert len(document["samples"]) == 41
    first_sample = document["samples"][0]
    assert first_sample["per_motor"]["J2A"]["tau_cmd_rotor_nm"] == 0.0
    assert first_sample["per_motor"]["J2A"]["tau_feedback_rotor_nm"] == 0.1
    assert first_sample["per_motor"]["J6"]["tau_cmd_rotor_nm"] is None
    assert first_sample["per_motor"]["J6"]["tau_feedback_rotor_nm"] is None


@pytest.mark.parametrize(
    "mutation, reason",
    (
        (lambda value: value["controller_mode_by_motor"].update(J3="position"), "active/unknown"),
        (lambda value: value["per_motor"]["J2A"].update(tau_cmd_rotor_nm=0.01), "nonzero active"),
        (lambda value: value["per_motor"]["J5"].update(merror=1), "merror"),
        (lambda value: value["per_motor"]["J6"].update(communication_ok=False), "communication"),
        (lambda value: value.update(j2_e_sync_rad=math.radians(0.251)), "synchronization warning"),
        (lambda value: value.update(velocity_rad_s=[math.radians(0.251)] + [0.0] * 5), "stationary"),
        (lambda value: value["per_motor"]["J4"].update(temperature_c=60.0), "temperature"),
    ),
)
def test_rejects_any_unsafe_readonly_frame(mutation, reason: str) -> None:
    value = recorder()
    sample = hardware_state(1, 1_000_000_000)
    mutation(sample)
    with pytest.raises(capture.ReadOnlyCaptureError, match=reason):
        value.add_document(sample)


def test_accepts_stationary_position_observer_with_noisy_vendor_dq() -> None:
    value = recorder()
    sample = hardware_state(1, 1_000_000_000)
    sample["per_motor"]["J1"]["dq_joint_rad_s"] = math.radians(4.0)
    value.add_document(sample)
    assert value.velocity_values["J1"] == [pytest.approx(math.radians(4.0))]


def test_rejects_session_change_replay_and_gap() -> None:
    value = recorder()
    value.add_document(hardware_state(1, 1_000_000_000))
    changed = hardware_state(2, 1_020_000_000)
    changed["state_instance_id"] = "4" * 32
    with pytest.raises(capture.ReadOnlyCaptureError, match="session/state"):
        value.add_document(changed)

    value = recorder()
    value.add_document(hardware_state(1, 1_000_000_000))
    with pytest.raises(capture.ReadOnlyCaptureError, match="sequence"):
        value.add_document(hardware_state(1, 1_020_000_000))

    value = recorder()
    value.add_document(hardware_state(1, 1_000_000_000))
    with pytest.raises(capture.ReadOnlyCaptureError, match="gap"):
        value.add_document(hardware_state(2, 1_250_000_001))


def test_refuses_incomplete_capture() -> None:
    value = recorder()
    value.add_document(hardware_state(1, 1_000_000_000))
    with pytest.raises(capture.ReadOnlyCaptureError, match="incomplete"):
        value.build_document()


def test_atomic_output_is_no_overwrite(tmp_path: Path) -> None:
    output = tmp_path / "evidence" / "power_on_readonly.json"
    data = capture.json_bytes({"status": "PASS"})
    capture.atomic_write_new(output, data)
    assert output.read_bytes() == data
    with pytest.raises(capture.ReadOnlyCaptureError, match="overwrite"):
        capture.atomic_write_new(output, b"different")
    assert output.read_bytes() == data


def test_preflight_requires_frozen_model_and_exact_gate(tmp_path: Path) -> None:
    model = tmp_path / "model.xml"
    model.write_text("not production", encoding="utf-8")
    args = capture.parse_args([
        "--execute-readonly",
        "--confirm", capture.READONLY_GATE,
        "--model-path", str(model),
        "--model-absolute-q-rad", "0", "0", "0", "0", "0", "0",
        "--operator-confirmed-at-utc", "2026-08-31T12:00:00Z",
        "--output", str(tmp_path / "out.json"),
    ])
    with pytest.raises(capture.ReadOnlyCaptureError, match="model SHA"):
        capture.validate_preflight(args)


def test_json_has_no_nonfinite_values() -> None:
    value = recorder()
    add_ten_second_window(value)
    encoded = capture.json_bytes(value.build_document())
    decoded = json.loads(encoded)
    assert decoded["status"] == "PASS"

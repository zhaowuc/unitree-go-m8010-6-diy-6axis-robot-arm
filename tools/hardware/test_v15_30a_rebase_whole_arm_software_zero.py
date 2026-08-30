from __future__ import annotations

import copy
import hashlib
import importlib.util
import json
import os
from pathlib import Path

import pytest


MODULE_PATH = Path(__file__).with_name("v15_30a_rebase_whole_arm_software_zero.py")
SPEC = importlib.util.spec_from_file_location("whole_arm_rebase", MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
rebase = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(rebase)


BOOT_ID = "a5542fa1-f39c-42c3-b466-a47b0e50aac9"
POSE_BINDING_ID = "b" * 64
OLD_SHA = "a" * 64
WORKER_SHA = "c" * 64
BASE_BOOTTIME_NS = 1_000_000_000_000


def identity(kind: str, seconds: int) -> dict:
    prefixes = {
        "j2": ("j2-brake-raw", "j2-power-session"),
        "go_aux": ("go-aux-brake-raw", "go-aux-power-session"),
        "j6": ("j6-disabled-raw", "j6-power-session"),
    }
    capture_prefix, power_prefix = prefixes[kind]
    timestamp = f"20260828T1200{seconds:02d}000001Z"
    token = f"{seconds + 1:032x}"
    return {
        "capture_id": f"{capture_prefix}-{timestamp}-{token[:16]}",
        "power_session_id": f"{power_prefix}-{timestamp}-{token}",
        "recorded_at_utc": f"2026-08-28T12:00:{seconds:02d}.000001Z",
        "host_boot_id": BOOT_ID,
        "recorded_boottime_ns": BASE_BOOTTIME_NS + seconds * 1_000_000_000,
        "pose_binding_id": POSE_BINDING_ID,
        "status": "PASS",
        "hardware_accessed": True,
        "physical_power_state_during_capture": "24V_ON",
        "physical_power_off_required": False,
    }


def terminal_proof(bus: str, count: int) -> dict[str, str]:
    completed_cycles = count + 10
    tx_attempt_total = count + 10
    if bus == "j2":
        invalid_prefix_pairs = 3
        healthy_pairs = 5
        attempted_pairs = invalid_prefix_pairs + healthy_pairs
        prime_tx_attempt_count = 2 * attempted_pairs
        completed_cycles = count
        tx_attempt_total = prime_tx_attempt_count + 2 * count + 40
    proof = {
        "TERMINAL_PATH": "NORMAL",
        "GUI_GO_CONTROLLER_BUS": bus,
        "EXECUTION_POLICY": "BRAKE_ONLY",
        "COMMAND_RX_ENABLED": "NO",
        "FOC_TX_ATTEMPT_COUNT": "0",
        "FOC_SERIAL_SEND_CALL_COUNT": "0",
        "OTHER_MODE_TX_ATTEMPT_COUNT": "0",
        "BRAKE_ONLY_GUARD_BLOCK_COUNT": "0",
        "BRAKE_ONLY_AUDIT": "PASS",
        "FINAL_MODE": "BRAKE",
        "FINAL_BRAKE": "PASS",
        "MOTOR_INTERNAL_ZERO_WRITE": "NO",
        "COMPLETED_CYCLES": str(completed_cycles),
        "TX_ATTEMPT_TOTAL": str(tx_attempt_total),
        "BRAKE_TX_ATTEMPT_COUNT": str(tx_attempt_total),
        "SERIAL_SEND_CALL_COUNT": str(tx_attempt_total),
    }
    if bus == "j2":
        proof.update(
            {
                "J2_SESSION_REFERENCE_CONFIGURED": "NO",
                "J2_STARTUP_BRAKE_PRIME_STATE": "PASS",
                "J2_STARTUP_BRAKE_PRIME_MAX_INVALID_PREFIX_PAIRS": "3",
                "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS": str(
                    invalid_prefix_pairs
                ),
                "J2_STARTUP_BRAKE_PRIME_REQUIRED_HEALTHY_PAIRS": "5",
                "J2_STARTUP_BRAKE_PRIME_HEALTHY_PAIRS": str(healthy_pairs),
                "J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS": str(attempted_pairs),
                "J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT": str(
                    prime_tx_attempt_count
                ),
                "J2_STARTUP_BRAKE_PRIME_ELAPSED_NS": "180000000",
                "FINAL_BRAKE_TX_ATTEMPT_COUNT": "40",
            }
        )
    return proof


def go_statistics(mean: float) -> dict:
    return {
        "mean": mean,
        "minimum": mean - 0.0002,
        "maximum": mean + 0.0002,
        "span": 0.0004,
        "standard_deviation": 0.0001,
    }


def go_motor(mean: float, count: int = 500) -> dict:
    return {
        "sample_count": count,
        "unwrapped_raw_position_rad": go_statistics(mean),
    }


def j6_motor(mean: float) -> dict:
    raw = {**go_statistics(mean), "median": mean}
    return {
        "sample_count": 500,
        "raw_position_rad": raw,
        "velocity_rad_s": {"minimum": 0.0, "maximum": 0.0, "mean": 0.0},
        "mos_temperature_c": {"minimum": 30.0, "maximum": 31.0},
        "coil_temperature_c": {"minimum": 30.0, "maximum": 31.0},
        "state_values": [0],
        "state_names": ["DISABLED"],
    }


def j2_capture() -> dict:
    count = 500
    return {
        "schema": rebase.J2_CAPTURE_SCHEMA,
        **identity("j2", 0),
        "packet_count": count,
        "source_coverage_s": 4.99,
        "safety": {
            "execution_policy": "BRAKE_ONLY",
            "all_controller_modes": ["brake"],
            "command_rx_enabled": False,
            "j2_session_reference_configured": False,
            "foc_tx_attempt_count": 0,
            "foc_serial_send_call_count": 0,
            "other_mode_tx_attempt_count": 0,
            "brake_only_guard_block_count": 0,
            "active_or_hold_commands_sent": 0,
            "active_commands_sent": 0,
            "hold_commands_sent": 0,
            "communication_failure_packets": 0,
            "merror_nonzero_packets": 0,
            "motor_internal_zero_modified": False,
            "motor_internal_zero_write_count": 0,
            "rid_written": False,
            "rid_write_count": 0,
            "flash_or_eeprom_written": False,
            "flash_or_eeprom_write_count": 0,
            "startup_brake_prime_passed": True,
            "startup_invalid_prefix_pairs": 3,
        },
        "motors": {"J2A": go_motor(5.1), "J2B": go_motor(2.2)},
        "physical_confirmation": {
            "confirmation_gate": rebase.J2_PHYSICAL_GATE,
            "support_reliable": True,
            "j2_vertical_initialization_pose": True,
            "arm_not_moved": True,
            "arm_stationary": True,
            "not_at_mechanical_limit": True,
        },
        "raw_safety_summary": {
            "branch_inputs_loaded": False,
            "persistent_zero_loaded": False,
            "recovery_hint_loaded": False,
            "integer_turn_branch_inferred": False,
            "udp_send_calls": 0,
            "controller_fault_packets": 0,
            "startup_brake_prime": {
                "state": "PASS",
                "maximum_invalid_prefix_pairs": 3,
                "invalid_prefix_pairs": 3,
                "required_healthy_pairs": 5,
                "healthy_pairs": 5,
                "attempted_pairs": 8,
                "tx_attempt_count": 16,
                "elapsed_ns": 180_000_000,
                "feedback_published_during_prime": False,
                "window_reopens_after_first_healthy_pair": False,
            },
            "phase_tx_accounting": {
                "completed_cycles": count,
                "startup_prime_tx_attempt_count": 16,
                "control_loop_tx_attempt_count": 2 * count,
                "final_brake_tx_attempt_count": 40,
                "aggregate_tx_attempt_count": 16 + 2 * count + 40,
            },
            "temperature_min_c": {"J2A": 30.0, "J2B": 30.0},
            "temperature_max_c": {"J2A": 31.0, "J2B": 31.0},
            "worker": {
                "path": "/tmp/go-worker",
                "sha256": WORKER_SHA,
                "returncode": 0,
                "stop_escalation": "none",
                "forced_kill": False,
                "terminal_proof": terminal_proof("j2", count),
                "stdout": [],
                "stderr": [],
            },
        },
    }


def set_j2_invalid_prefix_pairs(document: dict, invalid_prefix_pairs: int) -> None:
    healthy_pairs = 5
    attempted_pairs = invalid_prefix_pairs + healthy_pairs
    prime_tx_attempt_count = 2 * attempted_pairs
    packet_count = document["packet_count"]
    aggregate_tx_attempt_count = prime_tx_attempt_count + 2 * packet_count + 40
    document["safety"]["startup_invalid_prefix_pairs"] = invalid_prefix_pairs
    prime = document["raw_safety_summary"]["startup_brake_prime"]
    prime["invalid_prefix_pairs"] = invalid_prefix_pairs
    prime["attempted_pairs"] = attempted_pairs
    prime["tx_attempt_count"] = prime_tx_attempt_count
    phase = document["raw_safety_summary"]["phase_tx_accounting"]
    phase["startup_prime_tx_attempt_count"] = prime_tx_attempt_count
    phase["aggregate_tx_attempt_count"] = aggregate_tx_attempt_count
    proof = document["raw_safety_summary"]["worker"]["terminal_proof"]
    proof["J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS"] = str(
        invalid_prefix_pairs
    )
    proof["J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS"] = str(attempted_pairs)
    proof["J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT"] = str(
        prime_tx_attempt_count
    )
    for field in (
        "TX_ATTEMPT_TOTAL",
        "BRAKE_TX_ATTEMPT_COUNT",
        "SERIAL_SEND_CALL_COUNT",
    ):
        proof[field] = str(aggregate_tx_attempt_count)


def set_j2_terminal_totals(document: dict, value: str) -> None:
    proof = document["raw_safety_summary"]["worker"]["terminal_proof"]
    for field in (
        "TX_ATTEMPT_TOTAL",
        "BRAKE_TX_ATTEMPT_COUNT",
        "SERIAL_SEND_CALL_COUNT",
    ):
        proof[field] = value


def go_aux_capture() -> dict:
    bus_motors = {"j1": ("J1",), "j345": ("J3", "J4", "J5")}
    means = {"J1": 1.1, "J3": 3.3, "J4": 4.4, "J5": 5.5}
    domains = {}
    for bus, names in bus_motors.items():
        domains[bus] = {
            "packet_count": 500,
            "source_coverage_s": 4.99,
            "local_receive_coverage_s": 4.99,
            "motor_names": list(names),
            "temperature_min_c": {name: 30.0 for name in names},
            "temperature_max_c": {name: 31.0 for name in names},
            "feedback_source_endpoint": f"127.0.0.1:{15310 if bus == 'j1' else 15313}",
            "terminal_proof": terminal_proof(bus, 500),
        }
    return {
        "schema": rebase.GO_AUX_CAPTURE_SCHEMA,
        **identity("go_aux", 10),
        "worker": {"path": "/tmp/go-worker", "sha256": WORKER_SHA},
        "domains": domains,
        "motors": {name: go_motor(mean) for name, mean in means.items()},
        "safety": {
            "execution_policy": "BRAKE_ONLY",
            "all_controller_modes": ["brake"],
            "command_rx_enabled": False,
            "foc_tx_attempt_count": 0,
            "foc_serial_send_call_count": 0,
            "other_mode_tx_attempt_count": 0,
            "active_or_hold_commands_sent": 0,
            "communication_failure_packets": 0,
            "merror_nonzero_packets": 0,
            "motor_internal_zero_modified": False,
            "rid_written": False,
            "flash_or_eeprom_written": False,
        },
        "physical_confirmation": {
            "confirmation_gate": rebase.GO_AUX_PHYSICAL_GATE,
            "support_reliable": True,
            "whole_arm_vertical_initialization_pose": True,
            "arm_not_moved": True,
            "arm_stationary": True,
            "not_at_mechanical_limit": True,
        },
        "raw_safety_summary": {
            "persistent_zero_loaded": False,
            "recovery_hint_loaded": False,
            "integer_turn_branch_inferred": False,
            "udp_send_calls": 0,
            "domain_worker_stdout": {"j1": [], "j345": []},
            "domain_worker_stderr": {"j1": [], "j345": []},
        },
    }


def j6_capture() -> dict:
    audit = {
        "total_tx_attempt_count": 505,
        "refresh_request_count": 505,
        "rid_read_request_count": 0,
        "rid_write_count": 0,
        "flash_or_eeprom_write_count": 0,
        "motor_internal_zero_write_count": 0,
        "set_zero_command_count": 0,
        "fc_enable_count": 0,
        "fd_disable_count": 0,
        "position_velocity_torque_command_count": 0,
        "other_tx_attempt_count": 0,
        "forbidden_tx_attempt_count": 0,
    }
    return {
        "schema": rebase.J6_CAPTURE_SCHEMA,
        **identity("j6", 20),
        "packet_count": 500,
        "source_coverage_s": 4.99,
        "safety": {
            "execution_policy": "DISABLED_REFRESH_ONLY",
            **audit,
            "observed_tx_audit": copy.deepcopy(audit),
            "active_or_hold_commands_sent": 0,
            "active_commands_sent": 0,
            "hold_commands_sent": 0,
            "motor_internal_zero_modified": False,
            "motor_internal_zero_write_count": 0,
            "rid_written": False,
            "rid_write_count": 0,
            "flash_or_eeprom_written": False,
            "flash_or_eeprom_write_count": 0,
        },
        "motors": {"J6": j6_motor(-2.0)},
        "terminal": {
            "final_disabled_frames": 5,
            "required_final_disabled_frames": 5,
            "states": [0, 0, 0, 0, 0],
            "state_names": ["DISABLED"] * 5,
            "confirmed": True,
            "channel_closed": True,
        },
        "physical_confirmation": {
            "confirmation_gate": rebase.J6_PHYSICAL_GATE,
            "j6_24v_on": True,
            "support_reliable": True,
            "whole_arm_vertical_initialization_pose": True,
            "arm_stationary": True,
            "not_at_mechanical_limit": True,
        },
        "usb_identity": copy.deepcopy(rebase.EXPECTED_J6_USB),
        "raw_safety_summary": {
            "local_capture_duration_s": 4.99,
            "error_callback_count": 0,
            "unknown_rx_callback_count": 0,
            "expected_usb": copy.deepcopy(rebase.EXPECTED_J6_USB),
            "expected_can_channel": 0,
            "expected_motor_id": 1,
            "expected_master_id": 0,
            "branch_inputs_loaded": False,
            "persistent_zero_loaded": False,
            "recovery_hint_loaded": False,
            "integer_turn_branch_inferred": False,
        },
        "failures": [],
    }


def captures() -> tuple[dict, dict, dict]:
    return j2_capture(), go_aux_capture(), j6_capture()


def production_documents() -> tuple[dict, dict, dict]:
    zero = {
        "schema": rebase.ZERO_SCHEMA,
        "created_at_utc": "2026-08-25T07:30:21+00:00",
        "reference_name": rebase.REFERENCE_NAME,
        "motors": {
            name: {
                "raw_position_rad": float(index),
                "sample_count": 100,
                "sample_span_rad": 0.0005,
            }
            for index, name in enumerate(rebase.EXPECTED_MOTORS, 1)
        },
        "mapping": {"gear_ratio": rebase.GEAR_RATIO, "signs": dict(rebase.ALL_SIGNS)},
        "writes": {
            "motor_internal_zero_modified": False,
            "rid_written": False,
            "flash_or_eeprom_written": False,
        },
        "terminal_state": {"go_motors": "BRAKE_CONFIRMED", "J6": "DISABLED_CONFIRMED"},
        "last_software_zero_rebase": {
            "schema": "go-m8010-software-zero-rebase-record/1.0",
            "recorded_at_utc": "2026-08-27T10:41:24+00:00",
            "scope": ["J2A", "J2B"],
        },
    }
    hints = {
        "schema": rebase.HINT_SCHEMA,
        "reference_name": rebase.REFERENCE_NAME,
        "source": "OLD",
        "motors": {
            name: {"logical_position_rad": 0.25}
            for name in rebase.GO_HINT_MOTORS
        },
        "updated_at_utc": "2026-08-27T10:41:24+00:00",
    }
    initial = {
        "有效": True,
        "会话标识": f"persistent:{OLD_SHA[:16]}",
        "时间戳": "2026-08-27T10:41:24+00:00",
        "语义": "OLD",
        "电机内部零位": "未修改",
        "CAD零位": "PENDING",
        "ROS零位": rebase.REFERENCE_NAME,
        "关节位置_弧度": {f"J{index}": 0.25 for index in range(1, 7)},
    }
    return zero, hints, initial


def valid_bundle(current_boottime_ns: int | None = None):
    j2, aux, j6 = captures()
    return rebase.validate_capture_bundle(
        j2,
        aux,
        j6,
        host_boot_id=BOOT_ID,
        pose_binding_id=POSE_BINDING_ID,
        current_boottime_ns=current_boottime_ns,
    )


def test_complete_real_schema_bundle_is_accepted() -> None:
    measurements, identities = valid_bundle(BASE_BOOTTIME_NS + 30_000_000_000)
    assert set(measurements) == set(rebase.EXPECTED_MOTORS)
    assert measurements["J3"]["raw_position_rad"] == 3.3
    assert measurements["J6"]["raw_position_rad"] == -2.0
    assert len({record["capture_id"] for record in identities.values()}) == 3


def test_j2_capture_always_passes_shared_anchor_contract(monkeypatch) -> None:
    document = j2_capture()
    original = rebase.j2_anchor_contract.validate_capture
    observed: list[dict] = []

    def recording_validator(candidate: dict) -> dict:
        observed.append(candidate)
        return original(candidate)

    monkeypatch.setattr(
        rebase.j2_anchor_contract, "validate_capture", recording_validator
    )
    rebase.validate_j2_capture(
        document,
        host_boot_id=BOOT_ID,
        pose_binding_id=POSE_BINDING_ID,
    )
    assert observed == [document]


@pytest.mark.parametrize("invalid_prefix_pairs", [0, 3])
def test_j2_startup_prime_boundary_and_phase_accounting_are_accepted(
    invalid_prefix_pairs: int,
) -> None:
    document = j2_capture()
    set_j2_invalid_prefix_pairs(document, invalid_prefix_pairs)
    measurements, identity_record = rebase.validate_j2_capture(
        document,
        host_boot_id=BOOT_ID,
        pose_binding_id=POSE_BINDING_ID,
    )
    assert set(measurements) == {"J2A", "J2B"}
    assert identity_record["capture_id"] == document["capture_id"]


@pytest.mark.parametrize("elapsed_ns", [1, 500_000_000])
def test_j2_startup_prime_elapsed_boundaries_are_accepted(elapsed_ns: int) -> None:
    document = j2_capture()
    document["raw_safety_summary"]["startup_brake_prime"][
        "elapsed_ns"
    ] = elapsed_ns
    document["raw_safety_summary"]["worker"]["terminal_proof"][
        "J2_STARTUP_BRAKE_PRIME_ELAPSED_NS"
    ] = str(elapsed_ns)
    rebase.validate_j2_capture(
        document,
        host_boot_id=BOOT_ID,
        pose_binding_id=POSE_BINDING_ID,
    )


@pytest.mark.parametrize(
    "mutator",
    [
        pytest.param(
            lambda document: document["safety"].__setitem__(
                "startup_brake_prime_passed", False
            ),
            id="safety-prime-pass-false",
        ),
        pytest.param(
            lambda document: document["safety"].__setitem__(
                "startup_invalid_prefix_pairs", 2
            ),
            id="safety-prefix-disagrees",
        ),
        pytest.param(
            lambda document: document["safety"].__setitem__(
                "startup_invalid_prefix_pairs", True
            ),
            id="safety-prefix-bool",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"].pop(
                "startup_brake_prime"
            ),
            id="prime-missing",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"].pop(
                "phase_tx_accounting"
            ),
            id="phase-accounting-missing",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"][
                "startup_brake_prime"
            ].__setitem__("maximum_invalid_prefix_pairs", 2),
            id="prime-maximum-not-three",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"][
                "startup_brake_prime"
            ].__setitem__("required_healthy_pairs", 4),
            id="prime-required-not-five",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"][
                "startup_brake_prime"
            ].__setitem__("invalid_prefix_pairs", -1),
            id="prime-prefix-negative",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"][
                "startup_brake_prime"
            ].__setitem__("invalid_prefix_pairs", 4),
            id="prime-prefix-over-three",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"][
                "startup_brake_prime"
            ].__setitem__("healthy_pairs", 4),
            id="prime-healthy-not-five",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"][
                "startup_brake_prime"
            ].__setitem__("attempted_pairs", 9),
            id="prime-attempted-not-sum",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"][
                "startup_brake_prime"
            ].__setitem__("tx_attempt_count", 15),
            id="prime-tx-not-twice-attempted",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"][
                "startup_brake_prime"
            ].__setitem__("elapsed_ns", 0),
            id="prime-elapsed-zero",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"][
                "startup_brake_prime"
            ].__setitem__("elapsed_ns", 500_000_001),
            id="prime-elapsed-over-limit",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"][
                "startup_brake_prime"
            ].__setitem__("feedback_published_during_prime", True),
            id="prime-feedback-published",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"][
                "startup_brake_prime"
            ].__setitem__("window_reopens_after_first_healthy_pair", True),
            id="prime-window-reopens",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"]["worker"][
                "terminal_proof"
            ].__setitem__("J2_STARTUP_BRAKE_PRIME_STATE", "FAIL"),
            id="terminal-prime-state-fail",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"]["worker"][
                "terminal_proof"
            ].__setitem__("J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS", "03"),
            id="terminal-prefix-not-canonical",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"]["worker"][
                "terminal_proof"
            ].__setitem__("J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS", "9"),
            id="terminal-attempted-disagrees",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"]["worker"][
                "terminal_proof"
            ].__setitem__("COMPLETED_CYCLES", "501"),
            id="terminal-cycles-not-packet-count",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"]["worker"][
                "terminal_proof"
            ].__setitem__("FINAL_BRAKE_TX_ATTEMPT_COUNT", "39"),
            id="terminal-final-not-forty",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"][
                "phase_tx_accounting"
            ].__setitem__("completed_cycles", 501),
            id="phase-cycles-not-packet-count",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"][
                "phase_tx_accounting"
            ].__setitem__("startup_prime_tx_attempt_count", 15),
            id="phase-prime-tx-disagrees",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"][
                "phase_tx_accounting"
            ].__setitem__("control_loop_tx_attempt_count", 999),
            id="phase-control-not-twice-packets",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"][
                "phase_tx_accounting"
            ].__setitem__("final_brake_tx_attempt_count", 39),
            id="phase-final-not-forty",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"][
                "phase_tx_accounting"
            ].__setitem__("aggregate_tx_attempt_count", 1055),
            id="phase-aggregate-not-strict-sum",
        ),
        pytest.param(
            lambda document: set_j2_terminal_totals(document, "1055"),
            id="terminal-equal-totals-not-phase-aggregate",
        ),
        pytest.param(
            lambda document: document["raw_safety_summary"]["worker"][
                "terminal_proof"
            ].__setitem__("SERIAL_SEND_CALL_COUNT", "1055"),
            id="terminal-three-totals-disagree",
        ),
    ],
)
def test_j2_startup_prime_and_phase_accounting_are_strictly_cross_bound(
    mutator,
) -> None:
    document = j2_capture()
    mutator(document)
    with pytest.raises(ValueError):
        rebase.validate_j2_capture(
            document,
            host_boot_id=BOOT_ID,
            pose_binding_id=POSE_BINDING_ID,
        )


def test_build_rebases_every_motor_and_binds_pose_evidence() -> None:
    zero, hints, initial = production_documents()
    measurements, identities = valid_bundle()
    capture_ids = {name: record["capture_id"] for name, record in identities.items()}
    updated_zero, updated_hints, updated_initial, new_sha = rebase.build_documents(
        zero,
        hints,
        initial,
        original_sha256=OLD_SHA,
        measurements=measurements,
        capture_hashes={"j2": "1" * 64, "go_aux": "2" * 64, "j6": "3" * 64},
        capture_ids=capture_ids,
        pose_binding_id=POSE_BINDING_ID,
        recorded_at_utc="2026-08-28T12:00:30+00:00",
        host_boot_id=BOOT_ID,
    )
    assert updated_zero["motors"]["J1"]["raw_position_rad"] == 1.1
    assert updated_zero["motors"]["J2A"]["raw_position_rad"] == 5.1
    assert updated_zero["motors"]["J6"]["raw_position_rad"] == -2.0
    assert all(
        record["logical_position_rad"] == 0.0
        for record in updated_hints["motors"].values()
    )
    assert updated_initial["关节位置_弧度"] == {
        f"J{index}": 0.0 for index in range(1, 7)
    }
    assert updated_initial["会话标识"] == f"persistent:{new_sha[:16]}"
    assert updated_zero["software_zero_rebase_history"][0]["scope"] == ["J2A", "J2B"]
    last = updated_zero["software_zero_rebase_history"][-1]
    assert last["pose_binding_id"] == POSE_BINDING_ID
    assert last["capture_id"] == capture_ids
    assert updated_zero["writes"] == zero["writes"]


@pytest.mark.parametrize(
    "mutator",
    [
        lambda j2, aux, j6: j2["safety"].pop("foc_tx_attempt_count"),
        lambda j2, aux, j6: j6["safety"].__setitem__(
            "position_velocity_torque_command_count", 1
        ),
        lambda j2, aux, j6: j2["physical_confirmation"].__setitem__(
            "arm_not_moved", False
        ),
        lambda j2, aux, j6: j2.__setitem__("hardware_accessed", False),
        lambda j2, aux, j6: j2.__setitem__(
            "physical_power_state_during_capture", "24V_OFF"
        ),
        lambda j2, aux, j6: j2["motors"]["J2A"][
            "unwrapped_raw_position_rad"
        ].pop("minimum"),
        lambda j2, aux, j6: aux["motors"]["J1"][
            "unwrapped_raw_position_rad"
        ].__setitem__("span", 0.0),
        lambda j2, aux, j6: aux["safety"].__setitem__(
            "all_controller_modes", ["hold"]
        ),
        lambda j2, aux, j6: j6["terminal"].__setitem__("confirmed", False),
        lambda j2, aux, j6: j6["terminal"].__setitem__("states", [0, 0, 1, 0, 0]),
        lambda j2, aux, j6: j6["safety"]["observed_tx_audit"].__setitem__(
            "refresh_request_count", 504
        ),
        lambda j2, aux, j6: aux["worker"].__setitem__("sha256", WORKER_SHA.upper()),
        lambda j2, aux, j6: aux["domains"]["j345"].__setitem__(
            "motor_names", ["J4", "J3", "J5"]
        ),
        lambda j2, aux, j6: j6["raw_safety_summary"].__setitem__(
            "error_callback_count", 1
        ),
    ],
)
def test_incomplete_or_false_safety_evidence_is_rejected(mutator) -> None:
    j2, aux, j6 = captures()
    mutator(j2, aux, j6)
    with pytest.raises(ValueError):
        rebase.validate_capture_bundle(
            j2,
            aux,
            j6,
            host_boot_id=BOOT_ID,
            pose_binding_id=POSE_BINDING_ID,
        )


def test_pose_binding_must_match_all_three_captures() -> None:
    j2, aux, j6 = captures()
    aux["pose_binding_id"] = "d" * 64
    with pytest.raises(ValueError, match="pose_binding_id"):
        rebase.validate_capture_bundle(
            j2,
            aux,
            j6,
            host_boot_id=BOOT_ID,
            pose_binding_id=POSE_BINDING_ID,
        )


def test_capture_bundle_span_and_apply_freshness_are_enforced() -> None:
    j2, aux, j6 = captures()
    j6["recorded_boottime_ns"] = BASE_BOOTTIME_NS + 121_000_000_000
    with pytest.raises(ValueError, match="120-second"):
        rebase.validate_capture_bundle(
            j2,
            aux,
            j6,
            host_boot_id=BOOT_ID,
            pose_binding_id=POSE_BINDING_ID,
        )
    j2, aux, j6 = captures()
    with pytest.raises(ValueError, match="300-second"):
        rebase.validate_capture_bundle(
            j2,
            aux,
            j6,
            host_boot_id=BOOT_ID,
            pose_binding_id=POSE_BINDING_ID,
            current_boottime_ns=BASE_BOOTTIME_NS + 301_000_000_000,
        )


def test_capture_identity_format_and_lowercase_pose_are_enforced() -> None:
    j2, aux, j6 = captures()
    j2["capture_id"] = "j2-wrong"
    with pytest.raises(ValueError, match="capture_id"):
        rebase.validate_capture_bundle(
            j2,
            aux,
            j6,
            host_boot_id=BOOT_ID,
            pose_binding_id=POSE_BINDING_ID,
        )
    with pytest.raises(ValueError, match="lowercase"):
        rebase.validate_capture_bundle(
            *captures(),
            host_boot_id=BOOT_ID,
            pose_binding_id=POSE_BINDING_ID.upper(),
        )


def test_initial_pose_must_be_bound_to_current_zero_hash() -> None:
    _, _, initial = production_documents()
    initial["会话标识"] = "persistent:" + "e" * 16
    with pytest.raises(ValueError, match="different persistent zero"):
        rebase.validate_initial_pose(initial, OLD_SHA)


def write_production_files(root: Path) -> tuple[dict[str, Path], dict[Path, bytes]]:
    names = {
        "zero": "persistent_software_zero.json",
        "sidecar": "persistent_software_zero.json.sha256",
        "hints": "recovery_branch_hints.json",
        "initial": "initial_pose.json",
    }
    payloads = {
        "zero": b"old-zero\n",
        "sidecar": b"old-sidecar\n",
        "hints": b"old-hints\n",
        "initial": b"old-initial\n",
    }
    paths = {name: root / filename for name, filename in names.items()}
    originals = {}
    for name, path in paths.items():
        path.write_bytes(payloads[name])
        os.chmod(path, 0o600)
        originals[path] = payloads[name]
    return paths, originals


def call_apply(paths, originals, backup_dir: Path) -> None:
    rebase.apply_documents(
        zero_path=paths["zero"],
        sidecar_path=paths["sidecar"],
        hint_path=paths["hints"],
        initial_path=paths["initial"],
        backup_dir=backup_dir,
        zero_data=b"new-zero\n",
        sidecar_data=b"new-sidecar\n",
        hint_data=b"new-hints\n",
        initial_data=b"new-initial\n",
        original_data=originals,
    )


def test_transaction_commits_all_files_and_durable_backup(tmp_path: Path) -> None:
    paths, originals = write_production_files(tmp_path)
    backup = tmp_path / "backup"
    call_apply(paths, originals, backup)
    assert paths["zero"].read_bytes() == b"new-zero\n"
    assert paths["sidecar"].read_bytes() == b"new-sidecar\n"
    assert paths["hints"].read_bytes() == b"new-hints\n"
    assert paths["initial"].read_bytes() == b"new-initial\n"
    for path, data in originals.items():
        assert (backup / path.name).read_bytes() == data


@pytest.mark.parametrize("failure_index", [1, 2, 3, 4])
def test_every_partial_replace_failure_rolls_back_verified_originals(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch, failure_index: int
) -> None:
    paths, originals = write_production_files(tmp_path)
    real_replace = os.replace
    calls = 0

    def fail_once(source, destination):
        nonlocal calls
        calls += 1
        if calls == failure_index:
            raise OSError(f"injected replace failure {failure_index}")
        return real_replace(source, destination)

    monkeypatch.setattr(rebase.os, "replace", fail_once)
    with pytest.raises(OSError, match="injected replace failure"):
        call_apply(paths, originals, tmp_path / "backup")
    for path, data in originals.items():
        assert path.read_bytes() == data
        assert (tmp_path / "backup" / path.name).read_bytes() == data


def test_stale_production_snapshot_is_rejected_before_backup(tmp_path: Path) -> None:
    paths, originals = write_production_files(tmp_path)
    paths["hints"].write_bytes(b"concurrent-new-hints\n")
    os.chmod(paths["hints"], 0o600)
    with pytest.raises(RuntimeError, match="pre-backup"):
        call_apply(paths, originals, tmp_path / "backup")
    assert not (tmp_path / "backup").exists()


def test_json_hash_is_computed_from_the_exact_parsed_bytes(tmp_path: Path) -> None:
    path = tmp_path / "capture.json"
    data = b'{"schema":"x","value":1}\n'
    path.write_bytes(data)
    document, parsed_bytes, digest = rebase.read_json(path)
    assert document == {"schema": "x", "value": 1}
    assert parsed_bytes == data
    assert digest == hashlib.sha256(data).hexdigest()

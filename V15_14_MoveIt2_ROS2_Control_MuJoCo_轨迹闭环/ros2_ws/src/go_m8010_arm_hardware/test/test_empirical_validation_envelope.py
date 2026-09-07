from __future__ import annotations

import hashlib
import json
import sys
from datetime import datetime, timezone
from pathlib import Path

import pytest


REPO = Path(__file__).resolve().parents[5]
TOOLS = REPO / "tools" / "hardware"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from go_m8010_arm_hardware.empirical_validation_envelope import (  # noqa: E402
    CONFIRMATION_SCHEMA,
    EmpiricalEnvelopeError,
    EmpiricalStageGate,
    EmpiricalValidationEnvelope,
    live_hardware_blocker,
    select_runtime_torque_authority,
)
from v15_31b_create_empirical_validation_envelope import (  # noqa: E402
    build_envelope,
    json_bytes,
)


MOTORS = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")


def _envelope_file(tmp_path: Path, now: datetime, *, assisted_teach=False) -> tuple[Path, str]:
    document = build_envelope(
        session_id="session-31b",
        state_instance_id="state-31b",
        anchor_sha256="a" * 64,
        evidence_hashes={
            "power_on_readonly.json": "1" * 64,
            "model_session_anchor_validation.json": "2" * 64,
            "gravity_readonly_validation.json": "3" * 64,
        },
        maximum_predicted_rotor_nm={name: 0.0 for name in MOTORS},
        thermal_policy={
            "derating_start_c": 55.0,
            "thermal_stop_c": 60.0,
            "threshold_authority": "PROJECT_POLICY_NOT_VENDOR_CONTINUOUS_RATING",
        },
        hold_seconds=5.0,
        lifetime_seconds=300,
        created_at=now,
        assisted_teach=assisted_teach,
    )
    data = json_bytes(document)
    path = tmp_path / "envelope.json"
    path.write_bytes(data)
    return path, hashlib.sha256(data).hexdigest()


@pytest.mark.parametrize("field,value", (("precision_contract_id", None),
    ("endpoint_error_limit_deg", 0.5), ("minimum_actual_displacement_deg", 5.0),
    ("nominal_command_displacement_deg", 4.8)))
def test_runtime_rejects_legacy_or_mixed_precision_contract(tmp_path, field, value):
    now = datetime.now(timezone.utc)
    path, _ = _envelope_file(tmp_path, now)
    document = json.loads(path.read_text(encoding="utf-8"))
    document["position_validation"][field] = value
    data = json_bytes(document)
    path.write_bytes(data)
    with pytest.raises(EmpiricalEnvelopeError, match="POSITION_VALIDATION_BOUNDS_INVALID"):
        EmpiricalValidationEnvelope.from_path(path, hashlib.sha256(data).hexdigest(),
            now_utc=now, now_monotonic_ns=1_000_000_000)


def _hardware(now_ns: int, *, mode: str = "hold") -> dict:
    return {
        "schema": "go-m8010-hardware-state/1.1",
        "session_id": "session-31b",
        "state_instance_id": "state-31b",
        "source_monotonic_ns": now_ns,
        "healthy": True,
        "safety_metadata_ready": True,
        "j2_sync_fault": False,
        "j2_e_sync_rad": 0.0,
        "velocity_rad_s": [0.0] * 6,
        "controller_mode_by_motor": {name: mode for name in MOTORS},
        "per_motor": {
            name: {
                "fresh": True,
                "communication_ok": True,
                "merror": 0,
                "age_ms": 1.0,
                "temperature_c": 40.0,
                "thermal_metadata_status": "OBSERVED",
                "thermal_fault_latched": False,
                "no_progress_metadata_status": "OBSERVED",
                "load_limit_no_progress": False,
            }
            for name in MOTORS
        },
    }


def _confirmation(envelope, now_ns: int, sequence: int, target: float) -> dict:
    return {
        "schema": CONFIRMATION_SCHEMA,
        "source_instance_id": "4" * 32,
        "sequence": sequence,
        "source_monotonic_ns": now_ns,
        "envelope_id": envelope.envelope_id,
        "envelope_sha256": envelope.sha256,
        "session_id": envelope.session_id,
        "state_instance_id": envelope.state_instance_id,
        "target_gravity_scale": target,
        "operator_stop_ready": True,
        "j2_j3_support_reliable": True,
        "clearance_confirmed": True,
        "no_person_contact": True,
    }


def _step(gate, now, now_ns, requested, applied, mode="hold"):
    return gate.step(
        requested_scale=requested,
        applied_scale=applied,
        hardware_state=_hardware(now_ns, mode=mode),
        session_id="session-31b",
        state_instance_id="state-31b",
        anchor_sha256="a" * 64,
        hardware_enable_requested=True,
        now_monotonic_ns=now_ns,
        now_utc=now,
    )


def test_hash_pinned_envelope_is_not_a_rating_and_claim_is_single_use(tmp_path):
    now = datetime.now(timezone.utc)
    path, digest = _envelope_file(tmp_path, now)
    envelope = EmpiricalValidationEnvelope.from_path(
        path, digest, now_utc=now
    )
    assert envelope.maximum_total_active_seconds == 153.5
    assert envelope.maximum_abs_position_segment_deg == 5.0
    claim_dir = tmp_path / "claims"
    assert envelope.claim_single_use(claim_dir).is_file()
    with pytest.raises(EmpiricalEnvelopeError, match="ALREADY_CLAIMED"):
        envelope.claim_single_use(claim_dir)

    value = json.loads(path.read_text(encoding="utf-8"))
    value["rating_classification"] = "OFFICIAL_CONTINUOUS_RATING"
    tampered = tmp_path / "tampered.json"
    tampered.write_text(json.dumps(value), encoding="utf-8")
    tampered_digest = hashlib.sha256(tampered.read_bytes()).hexdigest()
    with pytest.raises(EmpiricalEnvelopeError, match="RATING_CLASSIFICATION"):
        EmpiricalValidationEnvelope.from_path(
            tampered, tampered_digest, now_utc=now
        )


def test_stage_gate_requires_zero_hold_strict_ladder_and_fresh_position_reconfirm(tmp_path):
    now = datetime.now(timezone.utc)
    path, digest = _envelope_file(tmp_path, now)
    envelope = EmpiricalValidationEnvelope.from_path(
        path, digest, now_utc=now, now_monotonic_ns=1_000_000_000
    )
    gate = EmpiricalStageGate(envelope)
    ns = 1_000_000_000

    # Zero authority is available to establish HOLD, but the rung does not
    # complete while controllers are still in BRAKE.
    assert _step(gate, now, ns, 0.0, 0.0, mode="brake") is True
    assert gate.stage_complete is False
    ns += 1_000_000
    assert _step(gate, now, ns, 0.0, 0.0) is True
    ns += 5_000_000_000
    assert _step(gate, now, ns, 0.0, 0.0) is True
    assert gate.stage_complete is True

    for sequence, target in enumerate((0.25, 0.50, 0.75, 1.0), start=1):
        ns += 1_000_000
        assert gate.observe_confirmation(
            _confirmation(envelope, ns, sequence, target),
            now_monotonic_ns=ns,
        )
        assert _step(gate, now, ns, target, target - 0.25) is True
        ns += 2_000_000_000
        assert _step(gate, now, ns, target, target) is True
        ns += 5_000_000_000
        assert _step(gate, now, ns, target, target) is True
        assert gate.stage_complete is True

    assert gate.status()["position_validation_authorized"] is False
    ns += 1_000_000
    assert gate.observe_confirmation(
        _confirmation(envelope, ns, 5, 1.0), now_monotonic_ns=ns
    )
    assert _step(gate, now, ns, 1.0, 1.0) is True
    assert gate.status()["position_validation_authorized"] is True

    # The operator-stop/support reconfirmation is a live 30 s gate, not a
    # one-time checkbox for the whole position phase.
    ns += 30_000_000_001
    assert _step(gate, now, ns, 1.0, 1.0) is False
    assert gate.invalidated is True
    assert "CONFIRMATION_EXPIRED" in gate.blocker


def test_stage_jump_or_live_fault_invalidates_without_retry(tmp_path):
    now = datetime.now(timezone.utc)
    path, digest = _envelope_file(tmp_path, now)
    envelope = EmpiricalValidationEnvelope.from_path(
        path, digest, now_utc=now, now_monotonic_ns=1_000_000_000
    )
    gate = EmpiricalStageGate(envelope)
    ns = 1_000_000_000
    assert _step(gate, now, ns, 0.50, 0.0) is False
    assert gate.invalidated is True
    assert _step(gate, now, ns + 1, 0.0, 0.0) is False


def test_stage_zero_active_fault_is_terminal_and_entry_temperature_is_strict(
    tmp_path,
):
    now = datetime.now(timezone.utc)
    path, digest = _envelope_file(tmp_path, now)
    envelope = EmpiricalValidationEnvelope.from_path(
        path, digest, now_utc=now, now_monotonic_ns=1_000_000_000
    )
    gate = EmpiricalStageGate(envelope)
    ns = 1_000_000_000
    assert _step(gate, now, ns, 0.0, 0.0) is True
    assert gate.active_started_ns == ns
    broken = _hardware(ns + 1)
    broken["per_motor"]["J2A"]["communication_ok"] = False
    assert gate.step(
        requested_scale=0.0,
        applied_scale=0.0,
        hardware_state=broken,
        session_id="session-31b",
        state_instance_id="state-31b",
        anchor_sha256="a" * 64,
        hardware_enable_requested=True,
        now_monotonic_ns=ns + 1,
        now_utc=now,
    ) is False
    assert gate.invalidated is True

    for temperature, entry_blocked in ((54.9, False), (55.0, True), (59.0, True)):
        hardware = _hardware(ns)
        for sample in hardware["per_motor"].values():
            sample["temperature_c"] = temperature
        blocker, _ = live_hardware_blocker(
            hardware,
            session_id="session-31b",
            state_instance_id="state-31b",
            now_monotonic_ns=ns,
            require_current_position_hold=True,
            require_entry_temperature=True,
        )
        assert bool(blocker) is entry_blocked
        runtime_blocker, _ = live_hardware_blocker(
            hardware,
            session_id="session-31b",
            state_instance_id="state-31b",
            now_monotonic_ns=ns,
            require_current_position_hold=True,
            require_entry_temperature=False,
        )
        assert runtime_blocker == ""


@pytest.mark.parametrize("lost_mode", ["brake", "position"])
def test_stage_zero_hold_loss_invalidates_and_cannot_retake_j6(
    tmp_path, lost_mode,
):
    now = datetime.now(timezone.utc)
    path, digest = _envelope_file(tmp_path, now)
    envelope = EmpiricalValidationEnvelope.from_path(
        path, digest, now_utc=now, now_monotonic_ns=1_000_000_000
    )
    gate = EmpiricalStageGate(envelope)
    ns = 1_000_000_000
    assert _step(gate, now, ns, 0.0, 0.0, mode="hold") is True
    assert gate.active_started_ns == ns

    # Once the first all-domain current-position HOLD spends this permit,
    # BRAKE or any non-HOLD mode is terminal.  A later HOLD packet (including
    # a second J6 enable/takeover attempt) cannot reuse the same envelope.
    ns += 1_000_000
    assert _step(gate, now, ns, 0.0, 0.0, mode=lost_mode) is False
    assert gate.invalidated is True
    assert gate.blocker == "EMPIRICAL_ZERO_CURRENT_POSITION_HOLD_LOST"
    assert _step(gate, now, ns + 1, 0.0, 0.0, mode="hold") is False
    assert gate.invalidated is True


def test_total_active_and_monotonic_deadlines_fail_closed(tmp_path):
    now = datetime.now(timezone.utc)
    path, digest = _envelope_file(tmp_path, now)
    envelope = EmpiricalValidationEnvelope.from_path(
        path, digest, now_utc=now, now_monotonic_ns=1_000_000_000
    )
    gate = EmpiricalStageGate(envelope)
    ns = 1_000_000_000
    assert _step(gate, now, ns, 0.0, 0.0) is True
    expired_active_ns = ns + int(
        (envelope.maximum_total_active_seconds + 0.01) * 1.0e9
    )
    assert _step(gate, now, expired_active_ns, 0.0, 0.0) is False
    assert "MAXIMUM_ACTIVE_TIME" in gate.blocker


def test_four_bounded_human_reconfirmations_fit_single_worker_session(
    tmp_path,
):
    now = datetime.now(timezone.utc)
    path, digest = _envelope_file(tmp_path, now)
    envelope = EmpiricalValidationEnvelope.from_path(
        path, digest, now_utc=now, now_monotonic_ns=1_000_000_000
    )
    gate = EmpiricalStageGate(envelope)
    ns = 1_000_000_000
    assert _step(gate, now, ns, 0.0, 0.0)
    ns += 5_000_000_000
    assert _step(gate, now, ns, 0.0, 0.0)
    assert gate.stage_complete

    previous = 0.0
    for sequence, target in enumerate((0.25, 0.5, 0.75, 1.0), 1):
        # Each explicit support/stop reconfirmation gets nearly its full 30 s
        # live window without restarting any GO worker or re-enabling J6.
        ns += 29_900_000_000
        assert _step(gate, now, ns, previous, previous)
        assert gate.observe_confirmation(
            _confirmation(envelope, ns, sequence, target),
            now_monotonic_ns=ns,
        )
        assert _step(gate, now, ns, target, previous)
        ns += 2_000_000_000
        assert _step(gate, now, ns, target, target)
        ns += 5_000_000_000
        assert _step(gate, now, ns, target, target)
        assert gate.stage_complete
        previous = target

    assert gate.stage_index == 4
    assert gate.invalidated is False
    assert (ns - gate.active_started_ns) * 1.0e-9 == pytest.approx(152.6)

    # A wall-clock rollback cannot extend the monotonic permit.
    assert envelope.runtime_blocker(
        session_id="session-31b",
        state_instance_id="state-31b",
        anchor_sha256="a" * 64,
        now_utc=now,
        now_monotonic_ns=envelope.monotonic_deadline_ns,
    ) == "EMPIRICAL_ENVELOPE_MONOTONIC_DEADLINE_EXPIRED"


def test_official_and_empirical_authority_selection_are_distinct():
    assert select_runtime_torque_authority(
        official_continuous_authoritative=True,
        empirical_authoritative=False,
        empirical_envelope_configured=False,
    ) == "OFFICIAL_CONTINUOUS_RATING"
    assert select_runtime_torque_authority(
        official_continuous_authoritative=False,
        empirical_authoritative=True,
        empirical_envelope_configured=True,
    ) == "EMPIRICAL_VALIDATION_ENVELOPE"
    assert select_runtime_torque_authority(
        official_continuous_authoritative=True,
        empirical_authoritative=True,
        empirical_envelope_configured=True,
    ) == "BLOCKED_AMBIGUOUS_AUTHORITY"


@pytest.mark.parametrize("opt_in", [False, True])
def test_assisted_teach_requires_explicit_envelope_full_ladder_and_live_confirmation(tmp_path, opt_in):
    now = datetime.now(timezone.utc)
    path, digest = _envelope_file(tmp_path, now, assisted_teach=opt_in)
    assert ("assisted_teach" in json.loads(path.read_bytes())) is opt_in
    envelope = EmpiricalValidationEnvelope.from_path(
        path, digest, now_utc=now, now_monotonic_ns=1_000_000_000)
    gate = EmpiricalStageGate(envelope)
    ns = 1_000_000_000
    assert not gate.status()["assisted_teach_authorized"]
    assert _step(gate, now, ns, 0.0, 0.0, mode="brake")
    assert not gate.status()["assisted_teach_authorized"]
    ns += 1_000_000
    assert _step(gate, now, ns, 0.0, 0.0)
    ns += 5_000_000_000
    assert _step(gate, now, ns, 0.0, 0.0)
    for sequence, target in enumerate((0.25, 0.5, 0.75, 1.0), 1):
        ns += 1_000_000
        assert gate.observe_confirmation(_confirmation(envelope, ns, sequence, target), now_monotonic_ns=ns)
        assert _step(gate, now, ns, target, target - 0.25)
        ns += 2_000_000_000
        assert _step(gate, now, ns, target, target)
        ns += 5_000_000_000
        assert _step(gate, now, ns, target, target)
        assert not gate.status()["assisted_teach_authorized"]
    ns += 1_000_000
    assert gate.observe_confirmation(_confirmation(envelope, ns, 5, 1.0), now_monotonic_ns=ns)
    assert _step(gate, now, ns, 1.0, 1.0)
    assert gate.status()["position_validation_authorized"]
    assert gate.status()["assisted_teach_authorized"] is opt_in
    assert gate.status()["maximum_teach_seconds"] == (30.0 if opt_in else None)
    hardware = _hardware(ns + 1)
    hardware["controller_mode_by_motor"].update(J2A="teach", J2B="teach")
    assert gate.step(requested_scale=1.0, applied_scale=1.0,
        hardware_state=hardware, session_id=envelope.session_id,
        state_instance_id=envelope.state_instance_id, anchor_sha256=envelope.anchor_sha256,
        hardware_enable_requested=True, now_monotonic_ns=ns + 1, now_utc=now) is opt_in
    if opt_in:
        ns += 30_000_000_001
        # The separately opted-in manual session does not fabricate a new
        # hands-free attestation while the operator is touching the arm.
        assert _step(gate, now, ns, 1.0, 1.0)
        assert gate.status()["assisted_teach_authorized"]
        assert not _step(gate, now, envelope.monotonic_deadline_ns, 1.0, 1.0)
        assert gate.invalidated and not gate.status()["assisted_teach_authorized"]


@pytest.mark.parametrize("field,value", [
    ("maximum_press_seconds", 30.01), ("maximum_excursion_from_press_deg", 5.01),
    ("maximum_velocity_deg_s", 5.01), ("allowed_after_scale", True),
    ("maximum_selected_joints", True), ("allowed_joints", ["J1", "J6"]),
    ("j6_fixed_hold_required", False), ("unexpected", True),
])
def test_assisted_teach_rejects_widened_or_malformed_opt_in(tmp_path, field, value):
    now = datetime.now(timezone.utc)
    path, _ = _envelope_file(tmp_path, now, assisted_teach=True)
    document = json.loads(path.read_bytes())
    document["assisted_teach"][field] = value
    data = json_bytes(document)
    path.write_bytes(data)
    with pytest.raises(EmpiricalEnvelopeError, match="ASSISTED_TEACH"):
        EmpiricalValidationEnvelope.from_path(path, hashlib.sha256(data).hexdigest(),
            now_utc=now, now_monotonic_ns=1_000_000_000)


@pytest.mark.parametrize("taught,allowed", [
    (("J1",), True), (("J2A", "J2B"), True), (("J5",), True),
    (("J6",), False), (("J2A",), False), (("J1", "J3"), False),
])
def test_assisted_teach_keeps_one_joint_other_holds_and_velocity_gate(taught, allowed):
    ns = 1_000_000_000
    hardware = _hardware(ns)
    hardware["controller_mode_by_motor"].update({name: "teach" for name in taught})
    kwargs = dict(session_id="session-31b", state_instance_id="state-31b",
        now_monotonic_ns=ns, require_current_position_hold=True)
    blocker, _ = live_hardware_blocker(hardware, allow_assisted_teach=True, **kwargs)
    assert (not blocker) is allowed
    assert live_hardware_blocker(hardware, **kwargs)[0]
    if allowed:
        hardware["velocity_rad_s"][0] = 0.087267
        assert live_hardware_blocker(hardware, allow_assisted_teach=True, **kwargs)[0] == "EMPIRICAL_ABNORMAL_VELOCITY"

from __future__ import annotations

import hashlib
import ast
import copy
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path
from types import SimpleNamespace

import pytest


REPO = Path(__file__).resolve().parents[5]
TOOLS = REPO / "tools" / "hardware"
if str(TOOLS) not in sys.path:
    sys.path.insert(0, str(TOOLS))

from go_m8010_arm_hardware.empirical_validation_envelope import (  # noqa: E402
    CONFIRMATION_SCHEMA,
    HAND_GUIDANCE_CONFIRMATION_SCHEMA,
    EmpiricalEnvelopeError,
    EmpiricalStageGate,
    EmpiricalValidationEnvelope,
    live_hardware_blocker,
    select_runtime_torque_authority,
    validate_stage_confirmation,
)
from v15_31b_create_empirical_validation_envelope import (  # noqa: E402
    build_envelope,
    json_bytes,
)


MOTORS = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")


def _envelope_file(tmp_path: Path, now: datetime, *, assisted_teach=False, hand_guidance=False) -> tuple[Path, str]:
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
        hand_guidance=hand_guidance,
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
    value = {
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
    if envelope.hand_guidance_enabled:
        value.pop("j2_j3_support_reliable")
        value.update(schema=HAND_GUIDANCE_CONFIRMATION_SCHEMA, base_fixed=True,
                     external_arm_support=False, established_position_hold=True)
    return value


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


def test_hand_guidance_full_ladder_then_multiple_joints_and_position_hold(tmp_path):
    now = datetime.now(timezone.utc)
    path, digest = _envelope_file(tmp_path, now, hand_guidance=True)
    envelope = EmpiricalValidationEnvelope.from_path(path, digest, now_utc=now, now_monotonic_ns=1_000_000_000)
    assert envelope.hand_guidance_enabled and not envelope.assisted_teach_enabled
    gate = EmpiricalStageGate(envelope)
    ns = 1_000_000_000
    assert not gate.status()["hand_guidance_authorized"]
    zero = _confirmation(envelope, ns, 1, 0.0)
    zero["established_position_hold"] = False
    validate_stage_confirmation(zero, envelope=envelope, target_scale=0.0, now_monotonic_ns=ns)
    zero["target_gravity_scale"] = 0.25
    with pytest.raises(EmpiricalEnvelopeError, match="POSITION_HOLD"):
        validate_stage_confirmation(zero, envelope=envelope, target_scale=0.25, now_monotonic_ns=ns)
    assert _step(gate, now, ns, 0.0, 0.0, mode="brake")
    assert not gate.stage_complete and not gate.status()["hand_guidance_authorized"]
    ns += 1_000_000
    assert _step(gate, now, ns, 0.0, 0.0)
    ns += 5_000_000_000
    assert _step(gate, now, ns, 0.0, 0.0)
    wrong = _confirmation(envelope, ns, 1, 0.25)
    wrong["external_arm_support"] = True
    assert not gate.observe_confirmation(wrong, now_monotonic_ns=ns)
    for sequence, target in enumerate((0.25, 0.5, 0.75, 1.0), 1):
        ns += 1_000_000
        assert gate.observe_confirmation(_confirmation(envelope, ns, sequence, target), now_monotonic_ns=ns)
        assert _step(gate, now, ns, target, target - 0.25)
        ns += 2_000_000_000
        assert _step(gate, now, ns, target, target)
        ns += 5_000_000_000
        assert _step(gate, now, ns, target, target)
        assert not gate.status()["hand_guidance_authorized"]
    ns += 1_000_000
    assert gate.observe_confirmation(_confirmation(envelope, ns, 5, 1.0), now_monotonic_ns=ns)
    assert _step(gate, now, ns, 1.0, 1.0)
    status = gate.status()
    assert status["hand_guidance_authorized"] and status["assisted_teach_authorized"]
    assert (status["maximum_teach_excursion_deg"], status["maximum_teach_velocity_deg_s"], status["maximum_teach_seconds"]) == (10.0, 30.0, 600.0)
    assert status["allowed_teach_joints"] == ["J1", "J2", "J3", "J4", "J5", "J6"]
    hardware = _hardware(ns + 1, mode="teach")
    hardware["velocity_rad_s"] = [math.radians(30)] * 6
    assert gate.step(requested_scale=1.0, applied_scale=1.0, hardware_state=hardware,
        session_id=envelope.session_id, state_instance_id=envelope.state_instance_id,
        anchor_sha256=envelope.anchor_sha256, hardware_enable_requested=True,
        now_monotonic_ns=ns + 1, now_utc=now)
    ns += 31_000_000_000
    assert _step(gate, now, ns, 1.0, 1.0)
    assert gate.status()["hand_guidance_authorized"]
    assert not _step(gate, now, envelope.monotonic_deadline_ns, 1.0, 1.0)


def test_qualified_guidance_warnings_retain_return_authority_without_masking_hard_faults(tmp_path):
    now = datetime.now(timezone.utc)

    def completed(profile="guidance", final_confirmation=True):
        directory = tmp_path / (profile + str(final_confirmation))
        directory.mkdir(exist_ok=True)
        path, digest = _envelope_file(directory, now, hand_guidance=profile == "guidance",
                                      assisted_teach=profile == "assisted")
        envelope = EmpiricalValidationEnvelope.from_path(path, digest, now_utc=now, now_monotonic_ns=1_000_000_000)
        gate = EmpiricalStageGate(envelope)
        ns = 1_000_000_000
        assert _step(gate, now, ns, 0.0, 0.0)
        ns += 5_000_000_000
        assert _step(gate, now, ns, 0.0, 0.0)
        for sequence, target in enumerate((.25, .5, .75, 1.0), 1):
            ns += 1_000_000
            assert gate.observe_confirmation(_confirmation(envelope, ns, sequence, target), now_monotonic_ns=ns)
            assert _step(gate, now, ns, target, target)
            ns += 5_000_000_000
            assert _step(gate, now, ns, target, target)
        if final_confirmation:
            ns += 1_000_000
            assert gate.observe_confirmation(_confirmation(envelope, ns, 5, 1.0), now_monotonic_ns=ns)
            assert _step(gate, now, ns, 1.0, 1.0)
        return gate, ns

    base, ns = completed()
    ns += 1_000_000
    hardware = _hardware(ns)
    hardware["j2_e_sync_rad"] = -math.radians(.2506)
    hardware["per_motor"]["J2A"]["temperature_c"] = 43.0  # Original stage baseline remains 40 C.
    expected_warnings = ["EMPIRICAL_J2_SYNC_WARNING", "EMPIRICAL_J2A_STAGE_TEMPERATURE_RISE"]

    def step(gate, value, checked_ns=ns):
        return gate.step(requested_scale=1.0, applied_scale=1.0, hardware_state=value,
            session_id=gate.envelope.session_id, state_instance_id=gate.envelope.state_instance_id,
            anchor_sha256=gate.envelope.anchor_sha256, hardware_enable_requested=True,
            now_monotonic_ns=checked_ns, now_utc=now)

    gate = copy.deepcopy(base)
    deadline, temperature_baseline = gate.envelope.monotonic_deadline_ns, dict(gate.stage_start_temperature_c)
    assert step(gate, hardware)
    status = gate.status()
    assert status["return_only"] and status["motion_warnings"] == expected_warnings
    assert status["position_validation_authorized"] and status["stage_complete"]
    assert not status["assisted_teach_authorized"] and not status["hand_guidance_authorized"]
    assert not gate.invalidated and status["blocker"] is None
    assert gate.stage_start_temperature_c == temperature_baseline and gate.envelope.monotonic_deadline_ns == deadline
    clear = _hardware(ns + 1)
    clear["j2_e_sync_rad"] = math.radians(.25)
    clear["per_motor"]["J2A"]["temperature_c"] = 42.0
    assert step(gate, clear, ns + 1)
    assert not gate.status()["return_only"] and gate.status()["motion_warnings"] == []
    assert gate.status()["hand_guidance_authorized"]
    assert gate.stage_start_temperature_c == temperature_baseline and gate.envelope.monotonic_deadline_ns == deadline
    boundary = copy.deepcopy(hardware)
    boundary["j2_e_sync_rad"] = math.radians(.5)
    assert step(copy.deepcopy(base), boundary)

    for mutation in (
        lambda h: h.update(j2_e_sync_rad=math.nan),
        lambda h: h.pop("j2_e_sync_rad"),
        lambda h: h.update(j2_e_sync_rad=math.radians(.5001)),
        lambda h: h.update(j2_sync_fault=True),
        lambda h: h.update(healthy=False),
        lambda h: h.update(source_monotonic_ns=ns-250_000_001),
        lambda h: h.update(session_id="different-session"),
        lambda h: h["per_motor"]["J6"].update(communication_ok=False),
        lambda h: h["per_motor"]["J5"].update(temperature_c=55.0),
        lambda h: h["per_motor"]["J5"].update(temperature_c=60.0),
        lambda h: h["per_motor"]["J6"].update(thermal_fault_latched=True),
        lambda h: h["per_motor"]["J5"].update(load_limit_no_progress=True),
        lambda h: h["velocity_rad_s"].__setitem__(1, math.radians(30.01)),
    ):
        hard = copy.deepcopy(hardware)
        mutation(hard)
        candidate = copy.deepcopy(base)
        assert not step(candidate, hard) and candidate.invalidated
        assert not candidate.status()["return_only"] and not candidate.status()["position_validation_authorized"]
    expired = copy.deepcopy(base)
    assert not step(expired, _hardware(deadline), deadline) and expired.invalidated
    for profile, confirmed in (("ordinary", True), ("assisted", True), ("guidance", False)):
        original, original_ns = completed(profile, confirmed)
        warning = _hardware(original_ns + 1)
        warning["j2_e_sync_rad"] = math.radians(.2506)
        assert not step(original, warning, original_ns + 1) and original.invalidated
        assert not original.status()["return_only"]

    # Exercise the real node's final status clamping without ROS/device calls.
    node_path = Path(__file__).resolve().parents[1] / "go_m8010_arm_hardware/whole_arm_gravity_node.py"
    tree = ast.parse(node_path.read_text(encoding="utf-8"))
    assignments = [item for item in ast.walk(tree) if isinstance(item, ast.Assign)
        and len(item.targets) == 1 and isinstance(item.targets[0], ast.Subscript)
        and isinstance(item.targets[0].value, ast.Name) and item.targets[0].value.id == "empirical_status"
        and isinstance(item.targets[0].slice, ast.Constant)
        and item.targets[0].slice.value in {"assisted_teach_authorized", "hand_guidance_authorized"}]
    assert len(assignments) == 2
    for initial_status, hard_authority in ((status, True), (gate.status(), False)):
        namespace = dict(empirical_status=dict(initial_status), hardware_authority_ready=hard_authority,
            selected_torque_authority="EMPIRICAL_VALIDATION_ENVELOPE", EMPIRICAL_AUTHORITY_CLASS="EMPIRICAL_VALIDATION_ENVELOPE",
            self=SimpleNamespace(current_gravity_scale=1.0), gravity_scale_target=1.0)
        exec(compile(ast.Module(body=assignments, type_ignores=[]), str(node_path), "exec"), namespace)
        assert not namespace["empirical_status"]["assisted_teach_authorized"]
        assert not namespace["empirical_status"]["hand_guidance_authorized"]


def test_hand_guidance_scope_and_false_support_claims_are_rejected(tmp_path):
    now = datetime.now(timezone.utc)
    for section, field, value in (
        ("hand_guidance", "maximum_selected_joints", True),
        ("hand_guidance", "maximum_velocity_deg_s", 30.01),
        ("hand_guidance", "maximum_press_seconds", 601),
        ("hand_guidance", "maximum_excursion_from_press_deg", 10.01),
        ("hand_guidance", "reference_lead_deg", 2.01),
        ("hand_guidance", "normal_exit_action", "BRAKE"),
        ("hand_guidance", "unexpected", True),
        ("physical_support", "external_arm_support", True),
        ("physical_support", "established_position_hold_required", False),
    ):
        path, _ = _envelope_file(tmp_path, now, hand_guidance=True)
        document = json.loads(path.read_bytes())
        target = document[section] if section == "hand_guidance" else document["live_gates"][section]
        target[field] = value
        data = json_bytes(document)
        path.write_bytes(data)
        with pytest.raises(EmpiricalEnvelopeError, match="HAND_GUIDANCE"):
            EmpiricalValidationEnvelope.from_path(path, hashlib.sha256(data).hexdigest(), now_utc=now)
    path, _ = _envelope_file(tmp_path, now, hand_guidance=True)
    document = json.loads(path.read_bytes())
    document["assisted_teach"] = {}
    data = json_bytes(document)
    path.write_bytes(data)
    with pytest.raises(EmpiricalEnvelopeError, match="MUTUALLY_EXCLUSIVE"):
        EmpiricalValidationEnvelope.from_path(path, hashlib.sha256(data).hexdigest(), now_utc=now)


def test_hand_guidance_live_gate_allows_paired_multijoint_and_keeps_hard_checks():
    ns = 1_000_000_000
    kwargs = dict(session_id="session-31b", state_instance_id="state-31b", now_monotonic_ns=ns,
                  require_current_position_hold=True, allow_hand_guidance=True)
    for taught, allowed in (((), True), (("J6",), True), (("J1", "J2A", "J2B", "J3", "J6"), True),
                            (("J2A", "J3"), False), (("J2B",), False)):
        hardware = _hardware(ns)
        hardware["velocity_rad_s"] = [math.radians(30)] * 6
        hardware["controller_mode_by_motor"].update({name: "teach" for name in taught})
        assert (not live_hardware_blocker(hardware, **kwargs)[0]) is allowed
    for mutation in (
        lambda h: h["velocity_rad_s"].__setitem__(2, math.radians(30.01)),
        lambda h: h["controller_mode_by_motor"].update(J1="teach", J3="position"),
        lambda h: h["per_motor"]["J6"].update(fresh=False),
        lambda h: h.update(j2_e_sync_rad=math.radians(0.251)),
        lambda h: h["per_motor"]["J3"].update(temperature_c=60.0),
        lambda h: h.update(source_monotonic_ns=ns - 250_000_001),
    ):
        hardware = _hardware(ns)
        mutation(hardware)
        assert live_hardware_blocker(hardware, **kwargs)[0]


@pytest.mark.parametrize("field,value", [
    ("maximum_press_seconds", 30.01), ("maximum_excursion_from_press_deg", 5.01),
    ("maximum_velocity_deg_s", 5.01), ("allowed_after_scale", True),
    ("maximum_selected_joints", True), ("allowed_joints", ["J1", "J6"]),
    ("stopping_hold_seconds", 1.01), ("max_stopping_error_deg", 2.01),
    ("soft_limit_action", "ignore_speed"),
    ("low_speed_stopping_error_action", "ignore_all_errors"),
    ("restricted_hold_rearm_error_deg", 0.5),
    ("restricted_hold_rearm_velocity_deg_s", 5.0),
    ("restricted_hold_minimum_stable_seconds", 0.1),
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


def test_native_stopping_hold_grace_is_one_joint_one_second_and_two_degrees():
    ns = 2_000_000_000
    hardware = _hardware(ns)
    hardware["position_rad"] = [math.radians(-1.08)] + [0.0] * 5
    hardware["velocity_rad_s"][0] = math.radians(-8.6)
    hardware.update(assisted_teach_exit_hold_validated=True,
        assisted_teach_exit_hold_source_monotonic_ns=ns,
        assisted_teach_exit_hold={
            "schema": "go-m8010-teach-exit-hold/1.0", "joint_index": 0,
            "press_activation_epoch": 123, "started_monotonic_ns": ns,
            "deadline_monotonic_ns": ns + 1_000_000_000,
            "reason": "VELOCITY_LIMIT", "targets_rad": list(hardware["position_rad"]),
            "initial_velocity_rad_s": math.radians(-8.6),
        })
    kwargs = dict(session_id="session-31b", state_instance_id="state-31b",
        now_monotonic_ns=ns, require_current_position_hold=True)
    assert not live_hardware_blocker(hardware, allow_assisted_teach=True, **kwargs)[0]
    assert live_hardware_blocker(hardware, **kwargs)[0] == "EMPIRICAL_ABNORMAL_VELOCITY"
    hardware["assisted_teach_exit_hold"]["restricted"] = True
    assert live_hardware_blocker(hardware, allow_assisted_teach=True, **kwargs)[0] == "EMPIRICAL_ABNORMAL_VELOCITY"
    hardware["velocity_rad_s"][0] = math.radians(1.6)
    hardware["position_rad"][0] += math.radians(2.01)
    assert not live_hardware_blocker(hardware, allow_assisted_teach=True, **kwargs)[0]
    hardware["position_rad"][0] = hardware["assisted_teach_exit_hold"]["targets_rad"][0]
    hardware["velocity_rad_s"][0] = math.radians(8.6)
    hardware["assisted_teach_exit_hold"].pop("restricted")
    hardware["velocity_rad_s"][1] = math.radians(5.01)
    assert live_hardware_blocker(hardware, allow_assisted_teach=True, **kwargs)[0] == "EMPIRICAL_ABNORMAL_VELOCITY"
    hardware["velocity_rad_s"][1] = 0.0
    hardware["position_rad"][0] += math.radians(2.01)
    assert live_hardware_blocker(hardware, allow_assisted_teach=True, **kwargs)[0] == "EMPIRICAL_ABNORMAL_VELOCITY"
    hardware["position_rad"][0] = hardware["assisted_teach_exit_hold"]["targets_rad"][0]
    hardware["per_motor"]["J3"]["communication_ok"] = False
    assert live_hardware_blocker(hardware, allow_assisted_teach=True, **kwargs)[0] == "EMPIRICAL_J3_FEEDBACK_INVALID"
    hardware["per_motor"]["J3"]["communication_ok"] = True
    hardware["assisted_teach_exit_hold_validated"] = False
    assert live_hardware_blocker(hardware, allow_assisted_teach=True, **kwargs)[0] == "EMPIRICAL_ASSISTED_TEACH_EXIT_PROOF_INVALID"
    hardware["assisted_teach_exit_hold_validated"] = True
    hardware["velocity_rad_s"][0] = float("nan")
    assert live_hardware_blocker(hardware, allow_assisted_teach=True, **kwargs)[0] == "EMPIRICAL_ABNORMAL_VELOCITY"
    hardware["velocity_rad_s"][0] = math.radians(8.6)
    expired = ns + 1_000_000_000
    hardware["source_monotonic_ns"] = expired
    hardware["assisted_teach_exit_hold_source_monotonic_ns"] = expired
    kwargs["now_monotonic_ns"] = expired
    assert live_hardware_blocker(hardware, allow_assisted_teach=True, **kwargs)[0] == "EMPIRICAL_ABNORMAL_VELOCITY"
    # Expired metadata is still valid history. Healthy ordinary HOLD remains
    # authorized and may ACK its exact native target without renewing grace.
    hardware["velocity_rad_s"][0] = 0.0
    assert not live_hardware_blocker(hardware, allow_assisted_teach=True, **kwargs)[0]
    hardware["source_monotonic_ns"] = ns
    assert live_hardware_blocker(hardware, allow_assisted_teach=True, **kwargs)[0] == "EMPIRICAL_HARDWARE_STATE_STALE"

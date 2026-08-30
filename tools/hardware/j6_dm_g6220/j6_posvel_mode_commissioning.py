#!/usr/bin/env python3
"""Operator-gated, volatile J6 RID10 POS_VEL commissioning.

The only parameter write implemented here is one session-local RID10 write from
MIT/1 to POS_VEL/2. No Flash/EEPROM save, zero write, ID write, or automatic MIT
rollback is implemented. Every path that opens CAN ends by sending FD at least
three times and confirming at least five DISABLED frames.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import secrets
import statistics
import struct
import time
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any, Callable, ContextManager

from j6_raw_can_diagnostic import (
    Blocked,
    RawCanLogger,
    add_evidence,
    capture_refresh_samples,
    exact_usb_identity,
    read_parameter,
    state_text,
    timestamp_token,
    write_raw_csv,
)


SOURCE_HEAD = "349fa982726e73cb799113f96cbb462e98fc85ca"
WORK_BRANCH = "agent/v15-21d-j6-posvel-enable-hold"
MODE_GATE = "J6_ALLOW_RUNTIME_CTRL_MODE_SWITCH_TO_POS_VEL=YES"
POWER_CYCLE_ATTESTATION = "J6_24V_POWER_CYCLED_SINCE_PRIOR_SESSION=YES"
MOTOR_ID = 1
CTRL_MODE_MIT = 1
CTRL_MODE_POS_VEL = 2
RID_CTRL_MODE = 10
DEVICE_LOCK_PATH = Path("/tmp/v15_30a_gui_j6.lock")
REPO_ROOT = Path(__file__).resolve().parents[3]
CANONICAL_LEDGER_PATH = (
    REPO_ROOT / ".runtime/v15_30a_gui/j6_posvel_commissioning_state.json"
)
PREFLIGHT_TTL_SECONDS = 300
SESSION_TOKEN_BYTES = 32
FINAL_DISABLE_SEND_COUNT = 3
FINAL_DISABLED_CONFIRM_COUNT = 5
ALREADY_POSVEL_BASELINE_COUNT = 100
EXPECTED_MASTER_ID = 0
EXPECTED_PMAX_RAD = 12.5
MAX_SAFE_TEMPERATURE_C = 60.0
MAXIMUM_RAW_SPAN_RAD = math.radians(0.20)
MAXIMUM_TAIL_DRIFT_RAD = math.radians(0.10)
STATIONARY_TAIL_COUNT = 10
FINAL_STATUS_COMPLETED = "COMPLETED"
FINAL_STATUS_FAILED = "FAILED_REQUIRES_POWER_CYCLE"
BLOCKED_NO_MODE_WRITE = "BLOCKED_NO_MODE_WRITE"
PREFLIGHT_READY = "PREFLIGHT_READY"
COMMIT_IN_PROGRESS = "COMMIT_IN_PROGRESS"
RESULT_SWITCHED_TO_POSVEL = (
    "SESSION_LOCAL_RID10_POS_VEL_AND_FINAL_DISABLED_PASS"
)
RESULT_ALREADY_POSVEL = (
    "SESSION_LOCAL_RID10_ALREADY_POS_VEL_AND_FINAL_DISABLED_PASS"
)
PHYSICAL_POWER_OFF_ACTION = (
    "DISCONNECT_J6_24V_KEEP_MECHANICAL_SUPPORT_AND_DO_NOT_RETRY_UNTIL_A_NEW_PREFLIGHT"
)


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _iso_utc(value: datetime) -> str:
    if value.tzinfo is None:
        raise ValueError("UTC timestamp must be timezone-aware")
    return value.astimezone(timezone.utc).isoformat()


def _parse_utc(value: Any) -> datetime:
    if not isinstance(value, str):
        raise Blocked("commissioning timestamp is missing")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError as exc:
        raise Blocked("commissioning timestamp is invalid") from exc
    if parsed.tzinfo is None:
        raise Blocked("commissioning timestamp is not timezone-aware")
    return parsed.astimezone(timezone.utc)


def atomic_write_json(path: Path, value: dict) -> None:
    """Durably replace one inventory without exposing a partial JSON file."""

    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{secrets.token_hex(8)}.tmp")
    descriptor = None
    try:
        descriptor = os.open(
            temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600
        )
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            descriptor = None
            json.dump(value, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
        try:
            os.chmod(path, 0o600)
        except OSError:
            pass
        try:
            directory_flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
            directory_fd = os.open(path.parent, directory_flags)
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
        except OSError:
            # Some filesystems (and Windows) do not support directory fsync.
            pass
    finally:
        if descriptor is not None:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


@contextmanager
def acquire_device_lock() -> Any:
    """Use the exact same non-blocking interlock as the production GUI worker."""

    try:
        import fcntl
    except ImportError as exc:  # pragma: no cover - production is Ubuntu only.
        raise Blocked("J6 commissioning lock requires POSIX fcntl") from exc

    lock_fd = os.open(
        DEVICE_LOCK_PATH,
        os.O_RDWR | os.O_CREAT | os.O_CLOEXEC,
        0o600,
    )
    try:
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except (BlockingIOError, OSError) as exc:
            raise Blocked(
                f"J6 device lock is already held: {DEVICE_LOCK_PATH}"
            ) from exc
        yield
    finally:
        try:
            fcntl.flock(lock_fd, fcntl.LOCK_UN)
        except OSError:
            pass
        os.close(lock_fd)


def _record_raw_evidence(inventory: dict, path: Path, logger: Any) -> None:
    write_raw_csv(path, logger, logger.snapshot())
    add_evidence(inventory, path)


@dataclass(frozen=True)
class RuntimeDependencies:
    """Injectable boundary; tests replace every hardware- or clock-facing item."""

    logger_factory: Callable[[], Any] = RawCanLogger
    identity_reader: Callable[[], dict] = exact_usb_identity
    parameter_reader: Callable[[Any, int], float] = read_parameter
    sample_capturer: Callable[[Any, int, str], tuple[list, int]] = capture_refresh_samples
    sleep: Callable[[float], None] = time.sleep
    now_utc: Callable[[], datetime] = _utc_now
    token_factory: Callable[[], str] = lambda: secrets.token_hex(SESSION_TOKEN_BYTES)
    lock_factory: Callable[[], ContextManager[Any]] = acquire_device_lock
    evidence_recorder: Callable[[dict, Path, Any], None] = _record_raw_evidence
    # Production must use the canonical accident ledger. Tests may explicitly
    # disable only this path check while every hardware dependency is mocked.
    enforce_canonical_inventory: bool = True


DEFAULT_DEPENDENCIES = RuntimeDependencies()


def base_inventory(token: str, created_at: datetime) -> dict:
    expires_at = created_at + timedelta(seconds=PREFLIGHT_TTL_SECONDS)
    return {
        "schema_version": "2.0",
        "task": "J6 volatile MIT-to-POS_VEL commissioning",
        "source_head": SOURCE_HEAD,
        "work_branch": WORK_BRANCH,
        "created_at_utc": _iso_utc(created_at),
        "status": "NEW",
        "commissioning_session": {
            "token": token,
            "created_at_utc": _iso_utc(created_at),
            "expires_at_utc": _iso_utc(expires_at),
            "ttl_seconds": PREFLIGHT_TTL_SECONDS,
            "token_consumed": False,
        },
        "safety": {
            "motor_state_required": "DISABLED",
            "flash_eeprom_save_used": False,
            "set_zero_used": False,
            "id_write_used": False,
            "other_rid_write_used": False,
            "automatic_mit_rollback_used": False,
            "final_fd_send_minimum": FINAL_DISABLE_SEND_COUNT,
            "final_disabled_feedback_minimum": FINAL_DISABLED_CONFIRM_COUNT,
        },
        "persistence_semantics": {
            "explicit_save_command_implemented": False,
            "explicit_save_command_sent": False,
            "flash_or_eeprom_vendor_guarantee_claimed": False,
            "prior_observation": "V15_21F_POWER_CYCLE_RETURNED_RID10_FROM_2_TO_MIT_1",
            "classification": "SESSION_LOCAL_BASED_ON_PRIOR_POWER_CYCLE_OBSERVATION",
            "future_startup_requires_rid10_readback": True,
        },
        "runtime_mode_write": {
            "operator_gate": "AWAITING",
            "rid": RID_CTRL_MODE,
            "from": CTRL_MODE_MIT,
            "to": CTRL_MODE_POS_VEL,
            "rid10_write_attempt_count": 0,
            "rid10_write_count": 0,
            "flash_eeprom_save_used": False,
            "automatic_mit_rollback_used": False,
        },
        "evidence": [],
    }


def readonly_motor_state(logger: Any, deps: RuntimeDependencies) -> dict:
    master_id = int(deps.parameter_reader(logger, 7))
    logger.master_id = master_id
    ctrl_mode = int(deps.parameter_reader(logger, 10))
    return {
        "master_id_rid7": master_id,
        "esc_id_rid8": int(deps.parameter_reader(logger, 8)),
        "ctrl_mode_rid10": ctrl_mode,
        "ctrl_mode_name": {
            CTRL_MODE_MIT: "MIT",
            CTRL_MODE_POS_VEL: "POS_VEL",
        }.get(ctrl_mode, f"UNKNOWN_{ctrl_mode}"),
        "pmax_rid21": deps.parameter_reader(logger, 21),
        "allowed_normal_feedback_can_ids": sorted(logger.allowed_feedback_ids()),
    }


def rid10_write_payload(value: int) -> bytes:
    if value != CTRL_MODE_POS_VEL:
        raise ValueError("commissioning only permits volatile RID10 POS_VEL/2")
    return bytes((MOTOR_ID, 0, 0x55, RID_CTRL_MODE)) + struct.pack("<I", value)


def _require_disabled(values: list, minimum: int, label: str) -> None:
    if len(values) < minimum:
        raise Blocked(f"{label} returned {len(values)}/{minimum} feedback frames")
    states = [item[1].state for item in values[:minimum]]
    if any(state != 0 for state in states):
        raise Blocked(f"{label} was not continuously DISABLED: {states}")
    for index, (_event, feedback) in enumerate(values[:minimum]):
        finite_values = (
            feedback.position,
            feedback.velocity,
            feedback.mos_temp,
            feedback.coil_temp,
        )
        if not all(math.isfinite(float(value)) for value in finite_values):
            raise Blocked(f"{label} frame {index} contains a non-finite value")
        if not 0.0 <= float(feedback.mos_temp) < MAX_SAFE_TEMPERATURE_C:
            raise Blocked(
                f"{label} frame {index} MOS temperature is outside [0, 60) C"
            )
        if not 0.0 <= float(feedback.coil_temp) < MAX_SAFE_TEMPERATURE_C:
            raise Blocked(
                f"{label} frame {index} coil temperature is outside [0, 60) C"
            )


def final_disable_and_confirm(
    logger: Any, deps: RuntimeDependencies, label: str
) -> dict:
    """Best-effort FD x3 followed by a strict five-frame DISABLED proof."""

    result = {
        "fd_send_attempts": 0,
        "fd_send_successes": 0,
        "required_fd_sends": FINAL_DISABLE_SEND_COUNT,
        "feedback_frames": 0,
        "required_disabled_frames": FINAL_DISABLED_CONFIRM_COUNT,
        "states": [],
        "confirmed": False,
        "errors": [],
    }
    fd_payload = b"\xFF" * 7 + b"\xFD"
    for index in range(FINAL_DISABLE_SEND_COUNT):
        result["fd_send_attempts"] += 1
        try:
            logger.send(MOTOR_ID, fd_payload, f"{label}_FD_{index + 1}")
            result["fd_send_successes"] += 1
        except BaseException as exc:  # Keep attempting all three safety frames.
            result["errors"].append(f"FD_{index + 1}:{exc}")
        if index + 1 < FINAL_DISABLE_SEND_COUNT:
            try:
                deps.sleep(0.02)
            except BaseException as exc:
                result["errors"].append(f"FD_DELAY_{index + 1}:{exc}")
    try:
        # Do not drain: capture_refresh_samples isolates with its own start index,
        # while retaining the complete pre-failure history for raw evidence.
        values, _ = deps.sample_capturer(
            logger, FINAL_DISABLED_CONFIRM_COUNT, f"{label}_DISABLED_CONFIRM"
        )
        states = [item[1].state for item in values[:FINAL_DISABLED_CONFIRM_COUNT]]
        result["feedback_frames"] = len(states)
        result["states"] = states
        _require_disabled(values, FINAL_DISABLED_CONFIRM_COUNT, label)
    except BaseException as exc:
        result["errors"].append(f"DISABLED_CONFIRM:{exc}")
    result["confirmed"] = bool(
        result["fd_send_successes"] >= FINAL_DISABLE_SEND_COUNT
        and result["feedback_frames"] >= FINAL_DISABLED_CONFIRM_COUNT
        and all(state == 0 for state in result["states"])
        and not result["errors"]
    )
    return result


def stats(values: list) -> dict:
    positions = [item[1].position for item in values]
    velocities = [item[1].velocity for item in values]
    return {
        "valid_frames": len(values),
        "total_required": len(values),
        "q_median_rad": statistics.median(positions),
        "q_mean_rad": statistics.fmean(positions),
        "q_std_rad": statistics.pstdev(positions),
        "q_min_rad": min(positions),
        "q_max_rad": max(positions),
        "velocity_min_rad_s": min(velocities),
        "velocity_max_rad_s": max(velocities),
        "mos_temperature_min_c": min(item[1].mos_temp for item in values),
        "mos_temperature_max_c": max(item[1].mos_temp for item in values),
        "coil_temperature_min_c": min(item[1].coil_temp for item in values),
        "coil_temperature_max_c": max(item[1].coil_temp for item in values),
        "states": sorted({state_text(item[1].state) for item in values}),
    }


def _close_logger(logger: Any, safeguard: dict) -> None:
    try:
        logger.close()
    except BaseException as exc:
        safeguard.setdefault("errors", []).append(f"LOGGER_CLOSE:{exc}")
        safeguard["confirmed"] = False


def _mark_failed(
    inventory: dict,
    phase: str,
    reason: str,
    deps: RuntimeDependencies,
    safeguard: dict | None,
) -> None:
    inventory["status"] = FINAL_STATUS_FAILED
    inventory["failed_at_utc"] = _iso_utc(deps.now_utc())
    inventory["failure"] = {"phase": phase, "reason": reason}
    inventory["final_disable"] = safeguard or {
        "required": False,
        "reason": "TRANSPORT_WAS_NOT_OPENED",
        "confirmed": False,
    }
    inventory["physical_action_required"] = PHYSICAL_POWER_OFF_ACTION
    inventory["safety"]["automatic_mit_rollback_used"] = False


def _mark_blocked_no_write(
    inventory: dict,
    phase: str,
    reason: str,
    deps: RuntimeDependencies,
    safeguard: dict | None,
) -> None:
    inventory["status"] = BLOCKED_NO_MODE_WRITE
    inventory["blocked_at_utc"] = _iso_utc(deps.now_utc())
    inventory["blocker"] = {"phase": phase, "reason": reason}
    inventory["final_disable"] = safeguard or {
        "required": False,
        "reason": "TRANSPORT_WAS_NOT_OPENED",
        "confirmed": False,
    }
    inventory["physical_action_required"] = "NONE"
    inventory["retry_action"] = "FIX_BLOCKER_AND_RUN_A_NEW_PREFLIGHT"
    inventory["runtime_mode_write"]["rid10_write_attempt_count"] = 0
    inventory["runtime_mode_write"]["rid10_write_count"] = 0


def _emit(inventory: dict) -> None:
    if inventory.get("status") == FINAL_STATUS_FAILED:
        print(
            "J6_COMMISSIONING=FAILED_REQUIRES_POWER_CYCLE\n"
            f"PHYSICAL_ACTION_REQUIRED={PHYSICAL_POWER_OFF_ACTION}",
            flush=True,
        )
    elif inventory.get("status") == BLOCKED_NO_MODE_WRITE:
        print(
            "J6_COMMISSIONING=BLOCKED_NO_MODE_WRITE\n"
            "MODE_WRITE_ATTEMPTED=NO\n"
            "PHYSICAL_POWER_CYCLE_REQUIRED=NO",
            flush=True,
        )
    print(json.dumps(inventory, ensure_ascii=False, indent=2), flush=True)


def _preflight_recovery_context(
    args: argparse.Namespace, deps: RuntimeDependencies
) -> dict | None:
    """Refuse to overwrite a consumed/ambiguous session without a power cycle."""

    if not args.inventory.exists():
        return None
    try:
        prior = json.loads(args.inventory.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise Blocked(
            "existing commissioning inventory is unreadable; use a new inventory "
            "path or preserve it for investigation"
        ) from exc
    if not isinstance(prior, dict):
        raise Blocked("existing commissioning inventory is not a JSON object")

    status = prior.get("status")
    if prior.get("schema_version") == "2.0" and status == PREFLIGHT_READY:
        try:
            _require_fresh_session(prior, deps.now_utc())
        except Blocked as exc:
            if str(exc) == "commissioning preflight expired":
                return {
                    "prior_status": status,
                    "reason": "EXPIRED_UNUSED_PREFLIGHT_REPLACED",
                    "power_cycle_attestation": "NOT_REQUIRED_NO_MODE_WRITE",
                }
            raise
        raise Blocked(
            "an unexpired PREFLIGHT_READY session already exists; use its token "
            "or wait for expiry"
        )

    write = prior.get("runtime_mode_write", {})
    session = prior.get("commissioning_session", {})
    safety = prior.get("safety", {})
    no_write_safety_fields = (
        "flash_eeprom_save_used",
        "set_zero_used",
        "id_write_used",
        "other_rid_write_used",
        "automatic_mit_rollback_used",
    )
    if (
        prior.get("schema_version") == "2.0"
        and status == BLOCKED_NO_MODE_WRITE
        and write.get("rid10_write_attempt_count") == 0
        and write.get("rid10_write_count") == 0
        and write.get("rid10_write_intent_durable") is not True
        and session.get("token_consumed") is False
        and all(safety.get(field) is False for field in no_write_safety_fields)
    ):
        return {
            "prior_status": status,
            "reason": "PRIOR_SESSION_PROVED_NO_MODE_WRITE",
            "power_cycle_attestation": "NOT_REQUIRED_NO_MODE_WRITE",
        }

    baseline = prior.get("posvel_disabled_baseline", {})
    final_disable = prior.get("final_disable", {})
    readonly_mode = prior.get("ctrl_mode_readonly_verification", {})
    if (
        prior.get("schema_version") == "2.0"
        and status == FINAL_STATUS_COMPLETED
        and prior.get("result") == RESULT_ALREADY_POSVEL
        and write.get("rid10_write_attempt_count") == 0
        and write.get("rid10_write_count") == 0
        and session.get("token_consumed") is False
        and session.get("token_authority") == "NONE_ALREADY_POS_VEL"
        and baseline.get("status") == "PASS"
        and baseline.get("valid_frames") == ALREADY_POSVEL_BASELINE_COUNT
        and readonly_mode.get("rid") == RID_CTRL_MODE
        and readonly_mode.get("after_disabled_baseline") == CTRL_MODE_POS_VEL
        and readonly_mode.get("write_performed") is False
        and prior.get("physical_action_required") == "NONE"
        and len(prior.get("evidence", [])) == 1
        and final_disable.get("fd_send_attempts") == FINAL_DISABLE_SEND_COUNT
        and final_disable.get("fd_send_successes") == FINAL_DISABLE_SEND_COUNT
        and final_disable.get("feedback_frames", 0)
        >= FINAL_DISABLED_CONFIRM_COUNT
        and final_disable.get("states") == [0] * FINAL_DISABLED_CONFIRM_COUNT
        and final_disable.get("errors") == []
        and final_disable.get("confirmed") is True
        and all(safety.get(field) is False for field in no_write_safety_fields)
    ):
        return {
            "prior_status": status,
            "prior_result": RESULT_ALREADY_POSVEL,
            "reason": "PRIOR_COMPLETED_SESSION_PROVED_NO_MODE_WRITE",
            "power_cycle_attestation": "NOT_REQUIRED_NO_MODE_WRITE",
        }

    supplied = getattr(args, "power_cycle_attestation", "")
    if supplied != POWER_CYCLE_ATTESTATION:
        raise Blocked(
            "existing commissioning state may include a consumed RID10 write; "
            f"after a real J6 24V power cycle require --power-cycle-attestation "
            f"'{POWER_CYCLE_ATTESTATION}'"
        )
    return {
        "prior_status": status,
        "prior_schema_version": prior.get("schema_version"),
        "reason": "OPERATOR_ATTESTED_PHYSICAL_POWER_CYCLE",
        "power_cycle_attestation": "PASS",
        "attestation_value": POWER_CYCLE_ATTESTATION,
    }


def _save_raw_evidence(
    inventory: dict,
    args: argparse.Namespace,
    logger: Any,
    deps: RuntimeDependencies,
    phase: str,
) -> str | None:
    path = args.output_dir / f"j6_posvel_{phase}_{timestamp_token()}.csv"
    try:
        deps.evidence_recorder(inventory, path, logger)
        inventory.setdefault("raw_evidence", {})[phase] = str(path)
        return None
    except BaseException as exc:
        inventory.setdefault("raw_evidence_errors", []).append(
            {"phase": phase, "reason": str(exc)}
        )
        return str(exc)


def _require_canonical_inventory(
    args: argparse.Namespace, deps: RuntimeDependencies
) -> None:
    if not deps.enforce_canonical_inventory:
        return
    requested = args.inventory.resolve(strict=False)
    canonical = CANONICAL_LEDGER_PATH.resolve(strict=False)
    if requested != canonical:
        raise Blocked(
            "production commissioning inventory is fixed and cannot be changed: "
            f"{canonical}"
        )


def preflight(
    args: argparse.Namespace, deps: RuntimeDependencies = DEFAULT_DEPENDENCIES
) -> int:
    _require_canonical_inventory(args, deps)
    created_at = deps.now_utc()
    token = deps.token_factory()
    if not isinstance(token, str) or len(token) < SESSION_TOKEN_BYTES * 2:
        raise RuntimeError("session token generator returned insufficient entropy")
    inventory = base_inventory(token, created_at)
    logger = None
    safeguard = None
    failure = None
    evidence_failure = None
    already_posvel = False
    with deps.lock_factory():
        recovery_context = _preflight_recovery_context(args, deps)
        if recovery_context is not None:
            inventory["prior_session_recovery"] = recovery_context
        try:
            identity = deps.identity_reader()
            logger = deps.logger_factory()
            deps.sleep(0.1)
            motor = readonly_motor_state(logger, deps)
            values, _ = deps.sample_capturer(logger, 5, "INITIAL_READ_ONLY")
            if motor["master_id_rid7"] != EXPECTED_MASTER_ID:
                raise Blocked(
                    f"RID7 changed: {motor['master_id_rid7']}, expected 0"
                )
            if motor["esc_id_rid8"] != MOTOR_ID:
                raise Blocked(f"RID8 changed: {motor['esc_id_rid8']}")
            pmax = float(motor["pmax_rid21"])
            if not math.isfinite(pmax) or not math.isclose(
                pmax, EXPECTED_PMAX_RAD, rel_tol=0.0, abs_tol=1.0e-6
            ):
                raise Blocked(
                    f"RID21 PMAX changed: {pmax}, expected {EXPECTED_PMAX_RAD}"
                )
            if motor["ctrl_mode_rid10"] not in {
                CTRL_MODE_MIT,
                CTRL_MODE_POS_VEL,
            }:
                raise Blocked(
                    "preflight requires RID10 MIT/1 or POS_VEL/2, observed "
                    f"{motor['ctrl_mode_rid10']}"
                )
            _require_disabled(values, 5, "initial feedback")
            inventory["device_identity"] = identity
            inventory["rid_snapshot"] = motor
            initial_q = statistics.median(item[1].position for item in values)
            inventory["initial_readonly"] = {
                "valid_frames": len(values),
                "states": [state_text(item[1].state) for item in values],
                "q_median_rad": initial_q,
            }
            if motor["ctrl_mode_rid10"] == CTRL_MODE_POS_VEL:
                already_posvel = True
                baseline_values, _ = deps.sample_capturer(
                    logger,
                    ALREADY_POSVEL_BASELINE_COUNT,
                    "POSVEL_ALREADY_DISABLED_BASELINE",
                )
                _require_disabled(
                    baseline_values,
                    ALREADY_POSVEL_BASELINE_COUNT,
                    "already-POS_VEL DISABLED baseline",
                )
                baseline = stats(baseline_values)
                q_after = baseline["q_median_rad"]
                delta_deg = math.degrees(q_after - initial_q)
                positions = [item[1].position for item in baseline_values]
                span_rad = baseline["q_max_rad"] - baseline["q_min_rad"]
                tail_median = statistics.median(
                    positions[-min(STATIONARY_TAIL_COUNT, len(positions)) :]
                )
                tail_drift_rad = tail_median - q_after
                if not all(math.isfinite(value) for value in (initial_q, q_after, delta_deg)):
                    raise Blocked("already-POS_VEL read-only baseline is non-finite")
                stationary = bool(
                    abs(delta_deg) <= 0.2
                    and span_rad <= MAXIMUM_RAW_SPAN_RAD
                    and abs(tail_drift_rad) <= MAXIMUM_TAIL_DRIFT_RAD
                )
                baseline.update(
                    {
                        "status": "PASS" if stationary else "FAIL",
                        "mode_switch_performed": False,
                        "q_initial_readonly_median_rad": initial_q,
                        "readonly_observation_delta_rad": q_after - initial_q,
                        "readonly_observation_delta_deg": delta_deg,
                        "readonly_span_rad": span_rad,
                        "readonly_span_deg": math.degrees(span_rad),
                        "tail_median_rad": tail_median,
                        "tail_drift_rad": tail_drift_rad,
                        "tail_drift_deg": math.degrees(tail_drift_rad),
                        "session_reference_semantics": (
                            "SESSION_LOCAL_ONLY_NOT_ZERO_OR_READY"
                        ),
                    }
                )
                inventory["posvel_disabled_baseline"] = baseline
                inventory["j6_posvel_session_reference_rad"] = q_after
                mode_after_baseline = int(
                    deps.parameter_reader(logger, RID_CTRL_MODE)
                )
                inventory["ctrl_mode_readonly_verification"] = {
                    "rid": RID_CTRL_MODE,
                    "initial": CTRL_MODE_POS_VEL,
                    "after_disabled_baseline": mode_after_baseline,
                    "write_performed": False,
                }
                if mode_after_baseline != CTRL_MODE_POS_VEL:
                    raise Blocked(
                        "RID10 changed during already-POS_VEL read-only baseline: "
                        f"{mode_after_baseline}, expected 2"
                    )
                write = inventory["runtime_mode_write"]
                write["operator_gate"] = "NOT_REQUIRED_ALREADY_POS_VEL"
                write["from"] = CTRL_MODE_POS_VEL
                write["required"] = False
                write["disposition"] = "SKIPPED_ALREADY_POS_VEL"
                session = inventory["commissioning_session"]
                session["token_authority"] = "NONE_ALREADY_POS_VEL"
                persistence = inventory["persistence_semantics"]
                persistence["current_observation"] = (
                    "RID10_ALREADY_POS_VEL_AT_PREFLIGHT"
                )
                persistence["classification"] = (
                    "CURRENT_READBACK_ONLY_PERSISTENCE_MECHANISM_NOT_INFERRED"
                )
                if abs(delta_deg) > 0.2:
                    raise Blocked(
                        "already-POS_VEL read-only baseline displacement "
                        "exceeded 0.2 deg"
                    )
                if span_rad > MAXIMUM_RAW_SPAN_RAD:
                    raise Blocked(
                        "already-POS_VEL read-only baseline span exceeded 0.2 deg"
                    )
                if abs(tail_drift_rad) > MAXIMUM_TAIL_DRIFT_RAD:
                    raise Blocked(
                        "already-POS_VEL read-only baseline tail drift exceeded 0.1 deg"
                    )
        except BaseException as exc:
            failure = exc
        finally:
            if logger is not None:
                safeguard = final_disable_and_confirm(logger, deps, "PREFLIGHT_FINAL")
                evidence_failure = _save_raw_evidence(
                    inventory, args, logger, deps, "preflight"
                )
                _close_logger(logger, safeguard)
    if safeguard is not None:
        inventory["final_disable"] = safeguard
        if not safeguard["confirmed"] and failure is None:
            failure = Blocked("preflight final DISABLED state could not be confirmed")
    if evidence_failure is not None and failure is None:
        failure = Blocked(f"preflight raw evidence save failed: {evidence_failure}")
    if failure is not None:
        if safeguard is not None and not safeguard["confirmed"]:
            _mark_failed(inventory, "preflight", str(failure), deps, safeguard)
        else:
            _mark_blocked_no_write(
                inventory, "preflight", str(failure), deps, safeguard
            )
        atomic_write_json(args.inventory, inventory)
        _emit(inventory)
        return 4
    if already_posvel:
        inventory["status"] = FINAL_STATUS_COMPLETED
        inventory["completed_at_utc"] = _iso_utc(deps.now_utc())
        inventory["result"] = RESULT_ALREADY_POSVEL
        inventory["physical_action_required"] = "NONE"
        atomic_write_json(args.inventory, inventory)
        _emit(inventory)
        return 0
    inventory["status"] = PREFLIGHT_READY
    inventory["preflight_ready_at_utc"] = _iso_utc(deps.now_utc())
    atomic_write_json(args.inventory, inventory)
    _emit(inventory)
    return 0


def _load_inventory(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise Blocked(f"cannot load commissioning inventory: {exc}") from exc
    if not isinstance(value, dict) or value.get("schema_version") != "2.0":
        raise Blocked("commissioning inventory schema is not 2.0")
    return value


def _require_matching_token(inventory: dict, supplied: str) -> None:
    recorded = inventory.get("commissioning_session", {}).get("token")
    if not isinstance(supplied, str) or not supplied:
        raise Blocked("commit requires explicit --session-token")
    if not isinstance(recorded, str) or not secrets.compare_digest(recorded, supplied):
        raise Blocked("commissioning session token mismatch")


def _require_fresh_session(inventory: dict, now: datetime) -> None:
    session = inventory.get("commissioning_session", {})
    created_at = _parse_utc(session.get("created_at_utc"))
    expires_at = _parse_utc(session.get("expires_at_utc"))
    if session.get("ttl_seconds") != PREFLIGHT_TTL_SECONDS:
        raise Blocked("commissioning TTL contract changed")
    lifetime = (expires_at - created_at).total_seconds()
    if not math.isclose(lifetime, PREFLIGHT_TTL_SECONDS, abs_tol=1e-6):
        raise Blocked("commissioning expiry was modified")
    if now < created_at - timedelta(seconds=5):
        raise Blocked("system clock is earlier than preflight creation time")
    if now > expires_at:
        raise Blocked("commissioning preflight expired")


def _consume_preflight(
    inventory: dict, args: argparse.Namespace, deps: RuntimeDependencies
) -> None:
    inventory["status"] = COMMIT_IN_PROGRESS
    inventory["commit_started_at_utc"] = _iso_utc(deps.now_utc())
    inventory["commissioning_session"]["token_consumed"] = True
    inventory["runtime_mode_write"]["operator_gate"] = "PASS"
    # This is a durable write intent, recorded before the only 0x55 frame. A
    # crash after this point can never replay the token or issue a second write.
    inventory["runtime_mode_write"]["rid10_write_attempt_count"] = 1
    inventory["runtime_mode_write"]["rid10_write_intent_durable"] = True
    atomic_write_json(args.inventory, inventory)


def switch_and_baseline(
    args: argparse.Namespace, deps: RuntimeDependencies = DEFAULT_DEPENDENCIES
) -> int:
    _require_canonical_inventory(args, deps)
    if args.operator_gate != MODE_GATE:
        raise Blocked(f"require --operator-gate '{MODE_GATE}'")
    initial_inventory = _load_inventory(args.inventory)
    _require_matching_token(initial_inventory, args.session_token)

    with deps.lock_factory():
        inventory = _load_inventory(args.inventory)
        _require_matching_token(inventory, args.session_token)
        if inventory.get("status") == COMMIT_IN_PROGRESS:
            _mark_failed(
                inventory,
                "commit-recovery",
                "previous commit stopped after atomic token consumption; RID10 write state is ambiguous",
                deps,
                None,
            )
            atomic_write_json(args.inventory, inventory)
            _emit(inventory)
            return 4
        if inventory.get("status") == FINAL_STATUS_FAILED:
            raise Blocked(
                "canonical ledger requires a real J6 24V power cycle; then run "
                "a new preflight with the exact power-cycle attestation gate"
            )
        if inventory.get("status") != PREFLIGHT_READY:
            raise Blocked(
                f"inventory status does not authorize RID10 write: {inventory.get('status')}"
            )
        try:
            _require_fresh_session(inventory, deps.now_utc())
        except Blocked as exc:
            _mark_blocked_no_write(
                inventory, "commit-authorization", str(exc), deps, None
            )
            atomic_write_json(args.inventory, inventory)
            _emit(inventory)
            return 4

        logger = None
        safeguard = None
        failure = None
        evidence_failure = None
        write_attempted = False
        write_confirmed = False
        try:
            current_identity = deps.identity_reader()
            if current_identity != inventory.get("device_identity"):
                raise Blocked("exact USB device identity changed after preflight")
            logger = deps.logger_factory()
            deps.sleep(0.1)
            motor = readonly_motor_state(logger, deps)
            if motor != inventory.get("rid_snapshot"):
                raise Blocked("RID snapshot changed after preflight")
            if motor["esc_id_rid8"] != MOTOR_ID:
                raise Blocked(f"RID8 changed: {motor['esc_id_rid8']}")
            if motor["ctrl_mode_rid10"] != CTRL_MODE_MIT:
                raise Blocked(
                    f"expected initial MIT/1, observed {motor['ctrl_mode_rid10']}"
                )
            before_values, _ = deps.sample_capturer(
                logger, 20, "PRE_SWITCH_DISABLED"
            )
            _require_disabled(before_values, 20, "pre-switch feedback")
            q_before = statistics.median(item[1].position for item in before_values)

            # The TTL and mode must still be valid at the actual write boundary,
            # not merely before USB/RID/feedback prechecks began.
            _require_fresh_session(inventory, deps.now_utc())
            pre_write_mode = int(deps.parameter_reader(logger, RID_CTRL_MODE))
            if pre_write_mode != CTRL_MODE_MIT:
                raise Blocked(
                    f"RID10 changed at write boundary: {pre_write_mode}, expected MIT/1"
                )

            # Durably consume the token before the sole RID10 write.
            _consume_preflight(inventory, args, deps)
            write_attempted = True
            logger.send(
                0x7FF,
                rid10_write_payload(CTRL_MODE_POS_VEL),
                "WRITE_RID10_POS_VEL_ONCE",
            )
            write_confirmed = True
            inventory["runtime_mode_write"]["rid10_write_count"] = 1
            inventory["runtime_mode_write"]["write_sent_at_utc"] = _iso_utc(
                deps.now_utc()
            )
            atomic_write_json(args.inventory, inventory)

            readback = int(deps.parameter_reader(logger, RID_CTRL_MODE))
            inventory["ctrl_mode_after_write"] = readback
            if readback != CTRL_MODE_POS_VEL:
                raise Blocked(f"RID10 readback is {readback}, expected 2")

            baseline_values, _ = deps.sample_capturer(
                logger, 100, "POSVEL_DISABLED_BASELINE"
            )
            _require_disabled(baseline_values, 100, "POS_VEL DISABLED baseline")
            baseline = stats(baseline_values)
            q_after = baseline["q_median_rad"]
            delta_deg = math.degrees(q_after - q_before)
            baseline.update(
                {
                    "status": "PASS" if abs(delta_deg) <= 0.2 else "FAIL",
                    "q_before_switch_median_rad": q_before,
                    "mode_switch_only_delta_rad": q_after - q_before,
                    "mode_switch_only_delta_deg": delta_deg,
                    "session_reference_semantics": "SESSION_LOCAL_ONLY_NOT_ZERO_OR_READY",
                }
            )
            inventory["posvel_disabled_baseline"] = baseline
            inventory["j6_posvel_session_reference_rad"] = q_after
            if abs(delta_deg) > 0.2:
                raise Blocked("mode-switch-only displacement exceeded 0.2 deg")
        except BaseException as exc:
            failure = exc
        finally:
            if logger is not None:
                safeguard = final_disable_and_confirm(logger, deps, "COMMIT_FINAL")
                evidence_failure = _save_raw_evidence(
                    inventory, args, logger, deps, "commit"
                )
                _close_logger(logger, safeguard)

        inventory["runtime_mode_write"]["rid10_write_attempt_count"] = int(
            write_attempted
        )
        inventory["runtime_mode_write"]["rid10_write_count"] = int(write_confirmed)
        inventory["final_disable"] = safeguard or {
            "required": False,
            "reason": "TRANSPORT_WAS_NOT_OPENED",
            "confirmed": False,
        }
        if safeguard is not None and not safeguard["confirmed"] and failure is None:
            failure = Blocked("final FD/DISABLED confirmation failed")
        if evidence_failure is not None and failure is None:
            failure = Blocked(f"commit raw evidence save failed: {evidence_failure}")
        if failure is not None:
            if write_attempted or (safeguard is not None and not safeguard["confirmed"]):
                _mark_failed(inventory, "commit", str(failure), deps, safeguard)
            else:
                _mark_blocked_no_write(
                    inventory, "commit", str(failure), deps, safeguard
                )
            atomic_write_json(args.inventory, inventory)
            _emit(inventory)
            return 4

        inventory["status"] = FINAL_STATUS_COMPLETED
        inventory["completed_at_utc"] = _iso_utc(deps.now_utc())
        inventory["result"] = RESULT_SWITCHED_TO_POSVEL
        inventory["physical_action_required"] = "NONE"
        atomic_write_json(args.inventory, inventory)
        _emit(inventory)
        return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("preflight", "switch-and-baseline"))
    parser.add_argument("--operator-gate", default="")
    parser.add_argument(
        "--session-token",
        default="",
        help="unpredictable token printed and stored by the immediately preceding preflight",
    )
    parser.add_argument(
        "--power-cycle-attestation",
        default="",
        help=(
            "exact physical-power-cycle gate required before replacing a prior "
            "COMMIT_IN_PROGRESS/FAILED/COMPLETED inventory"
        ),
    )
    parser.add_argument(
        "--output-dir", type=Path, default=Path("hardware/v15_21d")
    )
    parser.add_argument(
        "--inventory",
        type=Path,
        default=CANONICAL_LEDGER_PATH,
        help="fixed canonical ledger; production rejects every other path",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    try:
        if args.phase == "preflight":
            return preflight(args)
        return switch_and_baseline(args)
    except Blocked as exc:
        print(f"BLOCKED: {exc}", flush=True)
        return 4
    except BaseException as exc:
        # An unexpected software/storage interruption may happen after a durable
        # COMMIT_IN_PROGRESS record. Conservatively require physical power-off;
        # the canonical ledger prevents a path-changing retry.
        print(
            "J6_COMMISSIONING=UNHANDLED_FAILURE\n"
            f"ERROR={type(exc).__name__}:{exc}\n"
            f"PHYSICAL_ACTION_REQUIRED={PHYSICAL_POWER_OFF_ACTION}",
            flush=True,
        )
        return 5


if __name__ == "__main__":
    raise SystemExit(main())

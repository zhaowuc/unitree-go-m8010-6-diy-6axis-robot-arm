#!/usr/bin/env python3
"""Create immutable J1/J3/J4/J5 power-session phase references and permits.

This is an offline issuer.  It reads a hash-pinned BRAKE-only capture plus the
existing persistent zero, recovery hints, and read-only initial pose.  It never
opens a motor transport and never changes any calibration input.  In apply
mode it publishes one immutable anchor and two independent, 30-second,
single-use permits: one for the J1 worker and one for the J3/J4/J5 worker.
"""

from __future__ import annotations

import argparse
import hashlib
import importlib.util
import json
import math
from datetime import datetime, timedelta, timezone
from pathlib import Path
from typing import Any


_BASE_PATH = Path(__file__).with_name(
    "v15_30a_create_j2_vertical_session_phase_anchor.py"
)
_SPEC = importlib.util.spec_from_file_location("v15_30a_j2_anchor_base", _BASE_PATH)
assert _SPEC is not None and _SPEC.loader is not None
base = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(base)


ANCHOR_SCHEMA = "go-m8010-go-aux-power-session-reference/1.0"
PERMIT_SCHEMA = "go-m8010-go-aux-power-session-launch-permit/1.0"
CAPTURE_SCHEMA = "go-m8010-go-aux-brake-raw-capture-statistics/1.0"
RESULT_SCHEMA = "go-m8010-go-aux-vertical-session-phase-anchor-result/1.0"
APPLY_GATE = "V15_30A_CREATE_GO_AUX_VERTICAL_SESSION_PHASE_ANCHOR=YES"
PHYSICAL_GATE = (
    "WHOLE_ARM_VERTICAL=YES;SUPPORT_RELIABLE=YES;ARM_STATIONARY=YES;"
    "NOT_AT_MECHANICAL_LIMIT=YES"
)
MOTOR_SIGNS = {"J1": 1, "J3": 1, "J4": -1, "J5": 1}
MOTOR_TO_JOINT = {"J1": "J1", "J3": "J3", "J4": "J4", "J5": "J5"}
BUS_MOTORS = {"j1": ("J1",), "j345": ("J3", "J4", "J5")}
BUS_SERIAL = {
    "j1": "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if03-port0",
    "j345": "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if02-port0",
}
BUS_IDS = {"j1": [0], "j345": [3, 4, 5]}
MAX_RAW_SPAN_RAD = base.GEAR_RATIO * math.radians(0.20)
MIN_SAMPLES = 500
MIN_COVERAGE_S = 4.0


def _sha256(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def _validate_capture(document: dict[str, Any]) -> dict[str, Any]:
    if document.get("schema") != CAPTURE_SCHEMA or document.get("status") != "PASS":
        raise base.AnchorValidationError("GO-AUX raw capture schema/status mismatch")
    if document.get("physical_power_state_during_capture") != "24V_ON":
        raise base.AnchorValidationError("GO-AUX capture is not a 24V-on capture")
    capture_id = base.nonempty_text(document.get("capture_id"), "capture id")
    power_session_id = base.nonempty_text(
        document.get("power_session_id"), "power session id"
    )
    recorded_at = base.parse_utc_timestamp(
        document.get("recorded_at_utc"), "capture recorded_at_utc"
    )
    boot_id = base.normalized_host_boot_id(document.get("host_boot_id"))
    boottime_ns = base.positive_integer(
        document.get("recorded_boottime_ns"), "capture CLOCK_BOOTTIME"
    )
    safety = document.get("safety")
    exact = {
        "execution_policy": "BRAKE_ONLY",
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
    }
    if not isinstance(safety, dict) or any(
        safety.get(name) != value for name, value in exact.items()
    ) or safety.get("all_controller_modes") != ["brake"]:
        raise base.AnchorValidationError("GO-AUX capture is not proven BRAKE-only")
    worker = document.get("worker")
    if not isinstance(worker, dict):
        raise base.AnchorValidationError("GO-AUX worker binding is missing")
    worker_path = base.nonempty_text(worker.get("path"), "worker path")
    worker_sha = base.normalized_sha256(worker.get("sha256"), "worker")
    domains = document.get("domains")
    motors = document.get("motors")
    if not isinstance(domains, dict) or set(domains) != set(BUS_MOTORS):
        raise base.AnchorValidationError("GO-AUX capture domain set mismatch")
    if not isinstance(motors, dict) or set(motors) != set(MOTOR_SIGNS):
        raise base.AnchorValidationError("GO-AUX capture motor set mismatch")
    normalized_motors: dict[str, Any] = {}
    normalized_domains: dict[str, Any] = {}
    for bus, names in BUS_MOTORS.items():
        domain = domains[bus]
        if not isinstance(domain, dict):
            raise base.AnchorValidationError(f"{bus} capture domain is invalid")
        count = base.positive_integer(domain.get("packet_count"), f"{bus} packets")
        coverage = base.finite_number(domain.get("source_coverage_s"), f"{bus} coverage")
        if count < MIN_SAMPLES or coverage < MIN_COVERAGE_S:
            raise base.AnchorValidationError(f"{bus} capture is too short")
        if domain.get("motor_names") != list(names):
            raise base.AnchorValidationError(f"{bus} capture motor order mismatch")
        normalized_domains[bus] = {
            "packet_count": count,
            "source_coverage_s": coverage,
            "motor_names": list(names),
        }
        for name in names:
            record = motors[name]
            if not isinstance(record, dict) or record.get("sample_count") != count:
                raise base.AnchorValidationError(f"{name} sample count mismatch")
            raw = record.get("unwrapped_raw_position_rad")
            if not isinstance(raw, dict):
                raise base.AnchorValidationError(f"{name} raw statistics missing")
            mean = base.finite_number(raw.get("mean"), f"{name} mean")
            minimum = base.finite_number(raw.get("minimum"), f"{name} minimum")
            maximum = base.finite_number(raw.get("maximum"), f"{name} maximum")
            span = base.finite_number(raw.get("span"), f"{name} span")
            standard_deviation = base.finite_number(
                raw.get("standard_deviation"), f"{name} standard deviation"
            )
            if (
                any(abs(value) > base.MAX_PROTOCOL_POSITION_RAD for value in (mean, minimum, maximum))
                or not minimum <= mean <= maximum
                or span < 0.0
                or span > MAX_RAW_SPAN_RAD
                or not math.isclose(span, maximum - minimum, rel_tol=1e-9, abs_tol=1e-12)
                or not 0.0 <= standard_deviation <= span + 1e-12
            ):
                raise base.AnchorValidationError(f"{name} raw statistics invalid")
            normalized_motors[name] = {
                "sample_count": count,
                "unwrapped_raw_position_rad": {
                    "mean": mean,
                    "minimum": minimum,
                    "maximum": maximum,
                    "span": span,
                    "standard_deviation": standard_deviation,
                },
                "single_turn_phase_rad": base.normalized_single_turn_phase(mean),
            }
    return {
        "capture_id": capture_id,
        "power_session_id": power_session_id,
        "recorded_at_utc": recorded_at.isoformat().replace("+00:00", "Z"),
        "host_boot_id": boot_id,
        "recorded_boottime_ns": boottime_ns,
        "worker": {"path": worker_path, "sha256": worker_sha},
        "domains": normalized_domains,
        "motors": normalized_motors,
    }


def _operator_confirmation(args: argparse.Namespace, capture: dict[str, Any]) -> dict[str, Any]:
    if args.physical_confirmation != PHYSICAL_GATE:
        raise base.AnchorValidationError("complete whole-arm physical gate is missing")
    power_session_id = base.nonempty_text(
        args.operator_power_session_id, "operator power session id"
    )
    if power_session_id != capture["power_session_id"]:
        raise base.AnchorValidationError("operator and capture power sessions differ")
    return {
        "evidence_id": base.nonempty_text(args.operator_evidence_id, "operator evidence id"),
        "confirmed_at_utc": base.parse_utc_timestamp(
            args.operator_confirmed_at_utc, "operator confirmation time"
        ).isoformat().replace("+00:00", "Z"),
        "power_session_id": power_session_id,
        "vertical_initialization_pose": True,
        "support_reliable": True,
        "arm_not_moved": True,
        "not_at_mechanical_limit": True,
        "confirmation_gate": PHYSICAL_GATE,
    }


def _build_anchor(
    *, zero: dict[str, Any], initial_pose: dict[str, Any], fingerprints: dict[str, str],
    sidecar_sha: str, capture_path: Path, capture: dict[str, Any],
    confirmation: dict[str, Any],
) -> dict[str, Any]:
    material = {
        "schema": ANCHOR_SCHEMA,
        "parent": fingerprints["zero"],
        "capture": fingerprints["capture"],
        "capture_id": capture["capture_id"],
        "operator": confirmation["evidence_id"],
    }
    anchor_id = "go-aux-vertical-session-phase-v1-" + _sha256(
        base.canonical_bytes(material)
    )[:16]
    positions = initial_pose["关节位置_弧度"]
    motors = {}
    for name, sign in MOTOR_SIGNS.items():
        raw = capture["motors"][name]
        logical = base.finite_number(positions[MOTOR_TO_JOINT[name]], f"initial pose {name}")
        motors[name] = {
            "session_reference_raw_rad": raw["unwrapped_raw_position_rad"]["mean"],
            "logical_position_rad": logical,
            "logical_position_source": "HASH_BOUND_READ_ONLY_INITIAL_POSE",
            "sample_count": raw["sample_count"],
            "sample_span_raw_rad": raw["unwrapped_raw_position_rad"]["span"],
            "sign": sign,
            "gear_ratio": base.GEAR_RATIO,
            "single_turn_phase_rad": raw["single_turn_phase_rad"],
        }
    return {
        "schema": ANCHOR_SCHEMA,
        "anchor_version": 1,
        "anchor_id": anchor_id,
        "parent_persistent_zero_sha256": fingerprints["zero"],
        "reference_name": base.REFERENCE_NAME,
        "source_evidence": {"path": str(capture_path), "sha256": fingerprints["capture"]},
        "operator_confirmation": confirmation,
        "writes": {
            "motor_internal_zero_modified": False,
            "rid_written": False,
            "flash_or_eeprom_written": False,
        },
        "control_authority": {
            "is_software_zero": False,
            "authorizes_active_control": False,
            "authorizes_motor_internal_write": False,
        },
        "motors": motors,
        "parent_persistent_software_zero": {
            "schema": base.ZERO_SCHEMA,
            "reference_name": base.REFERENCE_NAME,
            "json_sha256": fingerprints["zero"],
            "checksum_sidecar_sha256": sidecar_sha,
            "GO_aux_raw_position_rad": {
                name: float(zero["motors"][name]["raw_position_rad"])
                for name in MOTOR_SIGNS
            },
        },
        "preserved_inputs": {
            "recovery_branch_hints_sha256": fingerprints["hints"],
            "initial_pose_sha256": fingerprints["initial_pose"],
            "overwritten_or_deleted": False,
            "recovery_hints_used_for_go_aux": False,
        },
        "raw_capture": {
            "source_schema": CAPTURE_SCHEMA,
            "source_file_sha256": fingerprints["capture"],
            **capture,
        },
        "derivation": {
            "logical_position_source": "HASH_BOUND_READ_ONLY_INITIAL_POSE",
            "integer_turn_branch_inferred": False,
            "persistent_zero_rebased": False,
            "recovery_hints_updated": False,
            "initial_pose_updated": False,
        },
        "offline_tool_safety": {
            "hardware_accessed_by_this_tool": False,
            "existing_calibration_file_modified": False,
        },
    }


def _permit_id(anchor: dict[str, Any], anchor_sha: str, bus: str) -> str:
    return _sha256(base.canonical_bytes({
        "schema": PERMIT_SCHEMA,
        "bus": bus,
        "anchor_id": anchor["anchor_id"],
        "anchor_sha256": anchor_sha,
        "parent_persistent_zero_sha256": anchor["parent_persistent_zero_sha256"],
        "power_session_id": anchor["raw_capture"]["power_session_id"],
        "host_boot_id": anchor["raw_capture"]["host_boot_id"],
        "worker_sha256": anchor["raw_capture"]["worker"]["sha256"],
    }))


def _build_permit(
    *, anchor: dict[str, Any], anchor_path: Path, anchor_sha: str, bus: str,
    pending_directory: Path, issued_at: datetime, boot_id: str, issued_boot_ns: int,
) -> tuple[dict[str, Any], dict[str, Path]]:
    if boot_id != anchor["raw_capture"]["host_boot_id"]:
        raise base.AnchorValidationError("capture host boot differs from current host")
    capture_boot_ns = anchor["raw_capture"]["recorded_boottime_ns"]
    age = issued_boot_ns - capture_boot_ns
    if age < 0 or age > base.MAX_CAPTURE_TO_PERMIT_NS:
        raise base.AnchorValidationError("capture is outside the 300-second permit window")
    permit_id = _permit_id(anchor, anchor_sha, bus)
    paths = base.launch_permit_paths(pending_directory, permit_id)
    expiry = issued_at + timedelta(seconds=base.LAUNCH_PERMIT_TTL_SECONDS)
    return {
        "schema": PERMIT_SCHEMA,
        "permit_version": 1,
        "permit_id": permit_id,
        "scope": bus,
        "single_use": True,
        "power_session_id": anchor["raw_capture"]["power_session_id"],
        "host_boot_id": boot_id,
        "issued_at_utc": issued_at.isoformat().replace("+00:00", "Z"),
        "expires_at_utc": expiry.isoformat().replace("+00:00", "Z"),
        "issued_unix_s": int(issued_at.timestamp()),
        "expires_unix_s": int(expiry.timestamp()),
        "issued_boottime_ns": issued_boot_ns,
        "expires_boottime_ns": issued_boot_ns + base.LAUNCH_PERMIT_TTL_NS,
        "ttl_seconds": base.LAUNCH_PERMIT_TTL_SECONDS,
        "parent_persistent_zero_sha256": anchor["parent_persistent_zero_sha256"],
        "source_capture": {
            "host_boot_id": boot_id,
            "recorded_boottime_ns": capture_boot_ns,
        },
        "session_reference": {"path": str(anchor_path), "sha256": anchor_sha},
        "worker": anchor["raw_capture"]["worker"],
        "pending_path": str(paths["pending"]),
        "inflight_path": str(paths["inflight"]),
        "spent_path": str(paths["spent"]),
        "serial": {
            "stable_by_id": BUS_SERIAL[bus],
            "bus": bus,
            "motor_ids": BUS_IDS[bus],
            "gear_ratio": base.GEAR_RATIO,
            "signs": {name: MOTOR_SIGNS[name] for name in BUS_MOTORS[bus]},
        },
        "startup_recheck": {
            "minimum_brake_frames": base.STARTUP_RECHECK_MINIMUM_BRAKE_FRAMES,
            "max_raw_phase_delta_rad": base.STARTUP_RECHECK_MAX_RAW_PHASE_DELTA_RAD,
            "max_raw_span_rad": base.STARTUP_RECHECK_MAX_RAW_SPAN_RAD,
        },
    }, paths


def run(
    args: argparse.Namespace, *, now_utc: datetime | None = None,
    host_boot_id: str | None = None, issued_boottime_ns: int | None = None,
) -> dict[str, Any]:
    defer_launch_permit = bool(
        getattr(args, "defer_launch_permit", False)
    )
    publish_deferred_launch_permit = bool(
        getattr(args, "publish_deferred_launch_permit", False)
    )
    if defer_launch_permit and publish_deferred_launch_permit:
        raise base.AnchorValidationError(
            "defer and publish-deferred launch-permit modes are mutually exclusive"
        )
    if (defer_launch_permit or publish_deferred_launch_permit) and not args.apply:
        raise base.AnchorValidationError(
            "deferred launch-permit modes require --apply"
        )
    inputs = base._resolve_distinct_inputs([
        args.zero_file, args.zero_sha256_file, args.recovery_hint_file,
        args.initial_pose_file, args.capture_statistics_file,
    ])
    zero_path, sidecar_path, hints_path, initial_path, capture_path = inputs
    zero, zero_data = base.read_json(zero_path)
    hints, hints_data = base.read_json(hints_path)
    initial, initial_data = base.read_json(initial_path)
    capture_document, capture_data = base.read_json(capture_path)
    sidecar_data = sidecar_path.read_bytes()
    fingerprints = {
        "zero": _sha256(zero_data),
        "sidecar": _sha256(sidecar_data),
        "hints": _sha256(hints_data),
        "initial_pose": _sha256(initial_data),
        "capture": _sha256(capture_data),
    }
    expected = {
        "zero": base.normalized_sha256(args.expected_parent_zero_sha256, "zero"),
        "hints": base.normalized_sha256(args.expected_recovery_hint_sha256, "hints"),
        "initial_pose": base.normalized_sha256(args.expected_initial_pose_sha256, "initial pose"),
        "capture": base.normalized_sha256(args.expected_capture_sha256, "capture"),
    }
    if any(fingerprints[name] != value for name, value in expected.items()):
        raise base.AnchorValidationError("a pinned input SHA-256 does not match")
    if base.parse_sidecar(sidecar_data, zero_path.name) != fingerprints["zero"]:
        raise base.AnchorValidationError("software-zero sidecar mismatch")
    base.validate_zero(zero)
    base.validate_hints(hints)
    base.validate_initial_pose(initial, fingerprints["zero"])
    capture = _validate_capture(capture_document)
    confirmation = _operator_confirmation(args, capture)
    anchor = _build_anchor(
        zero=zero, initial_pose=initial, fingerprints=fingerprints,
        sidecar_sha=fingerprints["sidecar"], capture_path=capture_path,
        capture=capture, confirmation=confirmation,
    )
    preserved_source_path = base.preserve_reference_if_requested(anchor, args, _validate_capture)
    timestamp = base.parse_utc_timestamp(
        capture["recorded_at_utc"], "capture time"
    ).strftime("%Y%m%dT%H%M%SZ")
    anchor_path = args.anchor_directory.resolve() / (
        f"go_aux_power_session_reference_v1_{timestamp}_"
        f"{anchor['anchor_id'].rsplit('-', 1)[-1]}.json"
    )
    anchor_data = base.json_bytes(anchor)
    anchor_sha = _sha256(anchor_data)
    issued_at = base.normalized_issued_at_utc(now_utc)
    boot_id = base.current_host_boot_id() if host_boot_id is None else base.normalized_host_boot_id(host_boot_id)
    boot_ns = base.current_boottime_ns() if issued_boottime_ns is None else base.positive_integer(issued_boottime_ns, "issued CLOCK_BOOTTIME")
    permits: dict[str, dict[str, Any]] = {}
    paths_by_bus: dict[str, dict[str, Path]] = {}
    for bus in BUS_MOTORS:
        permits[bus], paths_by_bus[bus] = _build_permit(
            anchor=anchor, anchor_path=anchor_path, anchor_sha=anchor_sha,
            bus=bus, pending_directory=args.launch_permit_directory.resolve(),
            issued_at=issued_at, boot_id=boot_id, issued_boot_ns=boot_ns,
        )
    output_paths = [anchor_path]
    for paths in paths_by_bus.values():
        output_paths.extend(paths.values())
    protected = set(inputs)
    if preserved_source_path is not None:
        protected.add(preserved_source_path)
    if any(path.resolve() in protected for path in output_paths):
        raise base.AnchorValidationError("output aliases protected input")
    if len({path.resolve() for path in output_paths}) != len(output_paths):
        raise base.AnchorValidationError("output paths are not distinct")
    if args.apply:
        base.validate_preserved_session_reference(anchor)
        if args.confirm != APPLY_GATE:
            raise base.AnchorValidationError(f"apply requires --confirm {APPLY_GATE}")
        permit_output_paths = [
            path
            for paths in paths_by_bus.values()
            for path in paths.values()
        ]
        if publish_deferred_launch_permit:
            if (
                not anchor_path.is_file()
                or anchor_path.is_symlink()
                or anchor_path.read_bytes() != anchor_data
            ):
                raise base.AnchorValidationError(
                    "deferred launch-permit publication requires the exact immutable anchor"
                )
            base.preflight_new_outputs(permit_output_paths)
        else:
            base.preflight_new_outputs(output_paths)
        before = {name: base.sha256_file(path) for name, path in zip(
            ("zero", "sidecar", "hints", "initial_pose", "capture"), inputs
        )}
        if before != fingerprints:
            raise base.AnchorValidationError("protected input changed before publication")
        directories = {anchor_path.parent}
        for paths in paths_by_bus.values():
            directories.update(path.parent for path in paths.values())
        for directory in directories:
            base.ensure_private_directory(directory)
        if publish_deferred_launch_permit:
            if anchor_path.read_bytes() != anchor_data:
                raise base.AnchorValidationError(
                    "immutable anchor changed before deferred launch-permit publication"
                )
        else:
            base.preflight_new_outputs(output_paths)
            base.atomic_write_new(anchor_path, anchor_data)
        if not defer_launch_permit:
            base.preflight_new_outputs(permit_output_paths)
            for bus in BUS_MOTORS:
                base.atomic_write_new(
                    paths_by_bus[bus]["pending"], base.json_bytes(permits[bus])
                )
        after = {name: base.sha256_file(path) for name, path in zip(
            ("zero", "sidecar", "hints", "initial_pose", "capture"), inputs
        )}
        if after != fingerprints:
            raise base.AnchorValidationError("protected input changed during publication")
        base.validate_preserved_session_reference(anchor)
    return {
        "schema": RESULT_SCHEMA,
        "applied": bool(args.apply),
        "anchor_path": str(anchor_path),
        "anchor_id": anchor["anchor_id"],
        "anchor_sha256": anchor_sha,
        "permit_paths": {
            bus: str(paths_by_bus[bus]["pending"]) for bus in BUS_MOTORS
        },
        "permit_ids": {bus: permits[bus]["permit_id"] for bus in BUS_MOTORS},
        "anchor_published": bool(args.apply) and not publish_deferred_launch_permit,
        "launch_permits_published": bool(args.apply) and not defer_launch_permit,
        "deferred_launch_permit": defer_launch_permit,
        "power_session_id": capture["power_session_id"],
        "parent_persistent_zero_sha256": fingerprints["zero"],
        "protected_inputs_unchanged": True,
        "hardware_accessed": False,
        "active_control_authorized": False,
        "anchor": None if args.apply else anchor,
        "launch_permits": None if args.apply else permits,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zero-file", type=Path, required=True)
    parser.add_argument("--zero-sha256-file", type=Path, required=True)
    parser.add_argument("--expected-parent-zero-sha256", required=True)
    parser.add_argument("--recovery-hint-file", type=Path, required=True)
    parser.add_argument("--expected-recovery-hint-sha256", required=True)
    parser.add_argument("--initial-pose-file", type=Path, required=True)
    parser.add_argument("--expected-initial-pose-sha256", required=True)
    parser.add_argument("--capture-statistics-file", type=Path, required=True)
    parser.add_argument("--expected-capture-sha256", required=True)
    parser.add_argument("--preserve-reference-file", type=Path)
    parser.add_argument("--expected-preserve-reference-sha256")
    parser.add_argument("--operator-evidence-id", required=True)
    parser.add_argument("--operator-confirmed-at-utc", required=True)
    parser.add_argument("--operator-power-session-id", required=True)
    parser.add_argument("--physical-confirmation", required=True)
    parser.add_argument("--anchor-directory", type=Path, required=True)
    parser.add_argument("--launch-permit-directory", type=Path, required=True)
    parser.add_argument("--apply", action="store_true")
    deferred_group = parser.add_mutually_exclusive_group()
    deferred_group.add_argument("--defer-launch-permit", action="store_true")
    deferred_group.add_argument(
        "--publish-deferred-launch-permit", action="store_true"
    )
    parser.add_argument("--confirm", default="")
    return parser.parse_args(argv)


def main() -> int:
    try:
        result = run(parse_args())
    except Exception as exc:
        print(json.dumps({
            "schema": RESULT_SCHEMA,
            "status": "BLOCKED",
            "reason": str(exc),
            "hardware_accessed": False,
            "active_control_authorized": False,
        }, ensure_ascii=False, indent=2), flush=True)
        return 4
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

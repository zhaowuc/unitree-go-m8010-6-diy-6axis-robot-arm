#!/usr/bin/env python3
"""Offline fail-closed validator for a GO-AUX anchor and two launch permits."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import stat
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Any


ANCHOR_SCHEMA = "go-m8010-go-aux-power-session-reference/1.0"
PERMIT_SCHEMA = "go-m8010-go-aux-power-session-launch-permit/1.0"
CAPTURE_SCHEMA = "go-m8010-go-aux-brake-raw-capture-statistics/1.0"
MOTOR_SIGNS = {"J1": 1, "J3": 1, "J4": 1, "J5": 1}
BUS_MOTORS = {"j1": ("J1",), "j345": ("J3", "J4", "J5")}
BUS_SERIAL = {
    "j1": "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if03-port0",
    "j345": "/dev/serial/by-id/usb-FTDI_USB__-__Serial_Converter_FTASQA6F-if02-port0",
}
BUS_IDS = {"j1": [0], "j345": [3, 4, 5]}
MAX_TTL_NS = 30_000_000_000
MAX_CAPTURE_AGE_NS = 300_000_000_000
GEAR_RATIO = 6.329999923706055


@dataclass(frozen=True)
class StableFile:
    path: Path
    data: bytes
    sha256: str


def secure_file(
    path: Path,
    label: str,
    maximum: int = 4 * 1024 * 1024,
    *,
    executable: bool = False,
) -> StableFile:
    """Open once with O_NOFOLLOW and prove identity/content stayed stable."""

    canonical = path.resolve(strict=True)
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        before = os.fstat(descriptor)
        private_mode_ok = stat.S_IMODE(before.st_mode) == 0o600
        executable_mode_ok = bool(stat.S_IMODE(before.st_mode) & 0o100)
        if (
            not stat.S_ISREG(before.st_mode)
            or before.st_uid != os.geteuid()
            or before.st_nlink != 1
            or not 0 < before.st_size <= maximum
            or (executable and not executable_mode_ok)
            or (not executable and not private_mode_ok)
        ):
            raise ValueError(f"{label} is not an acceptable owner-controlled regular file")
        chunks = []
        remaining = before.st_size
        while remaining:
            chunk = os.read(descriptor, min(remaining, 1024 * 1024))
            if not chunk:
                raise ValueError(f"{label} changed or ended during read")
            chunks.append(chunk)
            remaining -= len(chunk)
        after = os.fstat(descriptor)
        identity_before = (
            before.st_dev, before.st_ino, before.st_mode, before.st_uid,
            before.st_nlink, before.st_size, before.st_mtime_ns, before.st_ctime_ns,
        )
        identity_after = (
            after.st_dev, after.st_ino, after.st_mode, after.st_uid,
            after.st_nlink, after.st_size, after.st_mtime_ns, after.st_ctime_ns,
        )
        if identity_before != identity_after:
            raise ValueError(f"{label} changed while being read")
        path_status = os.stat(canonical, follow_symlinks=False)
        if (
            stat.S_ISLNK(path_status.st_mode)
            or (path_status.st_dev, path_status.st_ino) != (after.st_dev, after.st_ino)
        ):
            raise ValueError(f"{label} path identity changed while being read")
    finally:
        os.close(descriptor)
    data = b"".join(chunks)
    return StableFile(canonical, data, hashlib.sha256(data).hexdigest())


def load(file: StableFile, label: str) -> dict[str, Any]:
    value = json.loads(file.data.decode("utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{label} is not a JSON object")
    return value


def current_boottime_ns() -> int:
    if not hasattr(time, "CLOCK_BOOTTIME"):
        raise ValueError("CLOCK_BOOTTIME unavailable")
    return time.clock_gettime_ns(time.CLOCK_BOOTTIME)


def permit_identity(anchor: dict[str, Any], anchor_sha: str, bus: str) -> str:
    material = {
        "schema": PERMIT_SCHEMA,
        "bus": bus,
        "anchor_id": anchor["anchor_id"],
        "anchor_sha256": anchor_sha,
        "parent_persistent_zero_sha256": anchor["parent_persistent_zero_sha256"],
        "power_session_id": anchor["raw_capture"]["power_session_id"],
        "host_boot_id": anchor["raw_capture"]["host_boot_id"],
        "worker_sha256": anchor["raw_capture"]["worker"]["sha256"],
    }
    data = json.dumps(
        material, ensure_ascii=False, sort_keys=True, separators=(",", ":")
    ).encode("utf-8")
    return hashlib.sha256(data).hexdigest()


def validate(args: argparse.Namespace) -> list[str]:
    anchor_file = secure_file(args.anchor, "GO-AUX anchor")
    permit_files = {
        "j1": secure_file(args.j1_permit, "J1 permit", 131072),
        "j345": secure_file(args.j345_permit, "J345 permit", 131072),
    }
    zero_file = secure_file(args.zero, "persistent zero")
    hints_file = secure_file(args.hints, "recovery hints")
    initial_file = secure_file(args.initial_pose, "initial pose")
    worker_file = secure_file(
        args.worker, "GO worker", 64 * 1024 * 1024, executable=True
    )
    anchor_path = anchor_file.path
    permit_paths = {bus: file.path for bus, file in permit_files.items()}
    worker_path = worker_file.path
    anchor = load(anchor_file, "GO-AUX anchor")
    zero_sha = zero_file.sha256
    hints_sha = hints_file.sha256
    initial_sha = initial_file.sha256
    worker_sha = worker_file.sha256
    anchor_sha = anchor_file.sha256
    if anchor.get("schema") != ANCHOR_SCHEMA or anchor.get("anchor_version") != 1:
        raise ValueError("GO-AUX anchor schema/version mismatch")
    if anchor.get("reference_name") != "PERSISTENT_SOFTWARE_ZERO_V1":
        raise ValueError("GO-AUX anchor reference name mismatch")
    if anchor.get("parent_persistent_zero_sha256") != zero_sha:
        raise ValueError("GO-AUX anchor parent zero mismatch")
    preserved = anchor.get("preserved_inputs")
    if not isinstance(preserved, dict) or (
        preserved.get("recovery_branch_hints_sha256") != hints_sha
        or preserved.get("initial_pose_sha256") != initial_sha
        or preserved.get("overwritten_or_deleted") is not False
        or preserved.get("recovery_hints_used_for_go_aux") is not False
    ):
        raise ValueError("GO-AUX protected-input binding mismatch")
    initial = load(initial_file, "initial pose")
    if initial.get("有效") is not True or initial.get("会话标识") != f"persistent:{zero_sha[:16]}":
        raise ValueError("initial pose parent/validity mismatch")
    positions = initial.get("关节位置_弧度")
    if not isinstance(positions, dict):
        raise ValueError("initial pose positions missing")
    motors = anchor.get("motors")
    if not isinstance(motors, dict) or set(motors) != set(MOTOR_SIGNS):
        raise ValueError("GO-AUX anchor motor set mismatch")
    for name, sign in MOTOR_SIGNS.items():
        record = motors[name]
        logical = float(record.get("logical_position_rad"))
        reference = float(record.get("session_reference_raw_rad"))
        span = float(record.get("sample_span_raw_rad"))
        if (
            not all(math.isfinite(value) for value in (logical, reference, span))
            or not math.isclose(logical, float(positions[name]), rel_tol=0.0, abs_tol=1e-9)
            or record.get("logical_position_source") != "HASH_BOUND_READ_ONLY_INITIAL_POSE"
            or record.get("sign") != sign
            or not math.isclose(float(record.get("gear_ratio")), GEAR_RATIO, rel_tol=0.0, abs_tol=1e-6)
            or type(record.get("sample_count")) is not int
            or record.get("sample_count") < 500
            or not 0.0 <= span <= GEAR_RATIO * math.radians(0.20)
        ):
            raise ValueError(f"GO-AUX anchor {name} value mismatch")
    evidence = anchor.get("source_evidence")
    evidence_file = secure_file(
        Path(str(evidence.get("path", ""))), "raw evidence"
    )
    evidence_path = evidence_file.path
    if str(evidence_path) != evidence.get("path") or evidence_file.sha256 != evidence.get("sha256"):
        raise ValueError("raw evidence path/hash mismatch")
    capture = load(evidence_file, "raw evidence")
    if (
        capture.get("schema") != CAPTURE_SCHEMA
        or capture.get("status") != "PASS"
        or capture.get("hardware_accessed") is not True
        or capture.get("physical_power_state_during_capture") != "24V_ON"
    ):
        raise ValueError("raw evidence status/schema mismatch")
    safety = capture.get("safety")
    if not isinstance(safety, dict) or (
        safety.get("execution_policy") != "BRAKE_ONLY"
        or safety.get("all_controller_modes") != ["brake"]
        or safety.get("command_rx_enabled") is not False
        or any(safety.get(field) != 0 for field in (
            "foc_tx_attempt_count", "foc_serial_send_call_count",
            "other_mode_tx_attempt_count", "active_or_hold_commands_sent",
            "communication_failure_packets", "merror_nonzero_packets",
        ))
        or any(safety.get(field) is not False for field in (
            "motor_internal_zero_modified", "rid_written", "flash_or_eeprom_written",
        ))
    ):
        raise ValueError("raw evidence BRAKE-only proof mismatch")
    raw_capture = anchor.get("raw_capture")
    expected_raw_capture = {
        "source_schema": CAPTURE_SCHEMA,
        "source_file_sha256": evidence_file.sha256,
        "capture_id": capture.get("capture_id"),
        "power_session_id": capture.get("power_session_id"),
        "recorded_at_utc": capture.get("recorded_at_utc"),
        "host_boot_id": capture.get("host_boot_id"),
        "recorded_boottime_ns": capture.get("recorded_boottime_ns"),
        "worker": capture.get("worker"),
        "domains": {
            bus: {
                "packet_count": capture["domains"][bus]["packet_count"],
                "source_coverage_s": capture["domains"][bus]["source_coverage_s"],
                "motor_names": list(BUS_MOTORS[bus]),
            }
            for bus in BUS_MOTORS
        },
        "motors": {
            name: {
                "sample_count": capture["motors"][name]["sample_count"],
                "unwrapped_raw_position_rad": capture["motors"][name][
                    "unwrapped_raw_position_rad"
                ],
                "single_turn_phase_rad": (
                    float(capture["motors"][name]["unwrapped_raw_position_rad"]["mean"])
                    + math.pi
                ) % (2.0 * math.pi) - math.pi,
            }
            for name in MOTOR_SIGNS
        },
    }
    if not isinstance(raw_capture, dict) or (
        raw_capture != expected_raw_capture
        or raw_capture.get("power_session_id") != args.power_session_id
        or raw_capture.get("worker", {}).get("path") != str(worker_path)
        or raw_capture.get("worker", {}).get("sha256") != worker_sha
    ):
        raise ValueError("normalized raw capture binding mismatch")
    current_boot = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="ascii").strip()
    if raw_capture.get("host_boot_id") != current_boot:
        raise ValueError("GO-AUX capture belongs to another host boot")
    confirmation = anchor.get("operator_confirmation")
    if not isinstance(confirmation, dict) or confirmation.get("power_session_id") != args.power_session_id or any(
        confirmation.get(field) is not True for field in (
            "vertical_initialization_pose", "support_reliable",
            "arm_not_moved", "not_at_mechanical_limit",
        )
    ):
        raise ValueError("operator confirmation binding is incomplete")
    if any(anchor.get("writes", {}).get(field) is not False for field in (
        "motor_internal_zero_modified", "rid_written", "flash_or_eeprom_written",
    )) or any(anchor.get("control_authority", {}).get(field) is not False for field in (
        "is_software_zero", "authorizes_active_control", "authorizes_motor_internal_write",
    )):
        raise ValueError("GO-AUX anchor records forbidden authority/write")
    lifecycle = Path(f"/run/user/{os.geteuid()}/go-m8010/anchors")
    now = current_boottime_ns()
    for bus, permit_path in permit_paths.items():
        permit = load(permit_files[bus], f"{bus} permit")
        permit_id = permit_identity(anchor, anchor_sha, bus)
        expected_pending = lifecycle / "pending" / f"{permit_id}.json"
        expected_inflight = lifecycle / "inflight" / f"{permit_id}.json"
        expected_spent = lifecycle / "spent" / f"{permit_id}.json"
        if (
            permit.get("schema") != PERMIT_SCHEMA
            or permit.get("permit_version") != 1
            or permit.get("scope") != bus
            or permit.get("single_use") is not True
            or permit.get("permit_id") != permit_id
            or permit_path != expected_pending
            or permit.get("pending_path") != str(expected_pending)
            or permit.get("inflight_path") != str(expected_inflight)
            or permit.get("spent_path") != str(expected_spent)
            or expected_inflight.exists()
            or expected_spent.exists()
            or permit.get("power_session_id") != args.power_session_id
            or permit.get("host_boot_id") != current_boot
            or permit.get("parent_persistent_zero_sha256") != zero_sha
            or permit.get("session_reference") != {"path": str(anchor_path), "sha256": anchor_sha}
            or permit.get("worker") != {"path": str(worker_path), "sha256": worker_sha}
        ):
            raise ValueError(f"{bus} permit binding/lifecycle mismatch")
        issued = int(permit.get("issued_boottime_ns"))
        expires = int(permit.get("expires_boottime_ns"))
        source_capture = permit.get("source_capture")
        if source_capture != {
            "host_boot_id": raw_capture["host_boot_id"],
            "recorded_boottime_ns": raw_capture["recorded_boottime_ns"],
        }:
            raise ValueError(f"{bus} permit source-capture binding mismatch")
        captured = int(source_capture["recorded_boottime_ns"])
        serial_scope = permit.get("serial")
        expected_signs = {name: MOTOR_SIGNS[name] for name in BUS_MOTORS[bus]}
        if serial_scope != {
            "stable_by_id": BUS_SERIAL[bus],
            "bus": bus,
            "motor_ids": BUS_IDS[bus],
            "gear_ratio": GEAR_RATIO,
            "signs": expected_signs,
        }:
            raise ValueError(f"{bus} permit serial/motor scope mismatch")
        recheck = permit.get("startup_recheck")
        if not isinstance(recheck, dict) or (
            type(recheck.get("minimum_brake_frames")) is not int
            or not 50 <= recheck["minimum_brake_frames"] <= 150
            or not 0.0 < float(recheck.get("max_raw_phase_delta_rad", 0.0))
                <= GEAR_RATIO * math.radians(0.25) + 1e-12
            or not 0.0 < float(recheck.get("max_raw_span_rad", 0.0))
                <= GEAR_RATIO * math.radians(0.20) + 1e-12
        ):
            raise ValueError(f"{bus} permit startup recheck mismatch")
        ttl_seconds = permit.get("ttl_seconds")
        issued_unix_s = permit.get("issued_unix_s")
        expires_unix_s = permit.get("expires_unix_s")
        if not (
            captured <= issued <= now < expires
            and 0 < expires - issued <= MAX_TTL_NS
            and type(ttl_seconds) is int
            and 0 < ttl_seconds <= 30
            and expires - issued == ttl_seconds * 1_000_000_000
            and type(issued_unix_s) is int
            and type(expires_unix_s) is int
            and expires_unix_s - issued_unix_s == ttl_seconds
            and issued - captured <= MAX_CAPTURE_AGE_NS
            and now - captured <= MAX_CAPTURE_AGE_NS
        ):
            raise ValueError(f"{bus} permit is expired/future/over-age")
    return [
        str(anchor_path), anchor_sha, str(permit_paths["j1"]),
        str(permit_paths["j345"]), args.power_session_id, worker_sha,
    ]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--anchor", type=Path, required=True)
    parser.add_argument("--j1-permit", type=Path, required=True)
    parser.add_argument("--j345-permit", type=Path, required=True)
    parser.add_argument("--power-session-id", required=True)
    parser.add_argument("--zero", type=Path, required=True)
    parser.add_argument("--hints", type=Path, required=True)
    parser.add_argument("--initial-pose", type=Path, required=True)
    parser.add_argument("--worker", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    try:
        fields = validate(parse_args())
    except Exception as exc:
        print(f"GO_AUX_SESSION_BUNDLE_VALIDATION=BLOCKED\nREASON={exc}")
        return 2
    print("\n".join(fields))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

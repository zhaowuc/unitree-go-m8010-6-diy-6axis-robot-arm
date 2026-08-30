#!/usr/bin/env python3
"""Fail-closed, offline-only J2 persistent software-zero rebasing.

This tool never opens a motor device.  It updates only the two J2 entries in
the persistent software-zero document and resets the two J2 recovery hints to
zero.  Apply mode is hash-gated, confirmation-gated, backed up, and uses
same-directory atomic replacements.  The checksum sidecar is replaced last so
an interrupted update makes the production supervisor fail closed.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import shutil
import tempfile
from datetime import datetime
from pathlib import Path
from typing import Any


APPLY_GATE = "V15_30A_REBASE_J2_SOFTWARE_ZERO=YES"
ZERO_SCHEMA = "go-m8010-persistent-software-zero/1.0"
HINT_SCHEMA = "go-m8010-recovery-branch-hints/1.0"
REFERENCE_NAME = "PERSISTENT_SOFTWARE_ZERO_V1"
EXPECTED_MOTORS = {"J1", "J2A", "J2B", "J3", "J4", "J5", "J6"}
J2_SIGNS = {"J2A": -1, "J2B": 1}
GEAR_RATIO = 6.329999923706055
MAX_PROTOCOL_POSITION_RAD = 12.5
MAX_CAPTURE_SPAN_RAW_RAD = GEAR_RATIO * math.radians(0.25)


def sha256_bytes(data: bytes) -> str:
    return hashlib.sha256(data).hexdigest()


def json_bytes(document: dict[str, Any]) -> bytes:
    return (
        json.dumps(document, ensure_ascii=False, indent=2, sort_keys=False) + "\n"
    ).encode("utf-8")


def read_json(path: Path) -> tuple[dict[str, Any], bytes]:
    data = path.read_bytes()
    document = json.loads(data)
    if not isinstance(document, dict):
        raise ValueError(f"{path.name} must contain a JSON object")
    return document, data


def validate_timestamp(value: str) -> None:
    parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ValueError("--recorded-at-utc must include a UTC offset")
    if parsed.utcoffset().total_seconds() != 0:
        raise ValueError("--recorded-at-utc must be UTC")


def validate_zero(document: dict[str, Any]) -> None:
    if document.get("schema") != ZERO_SCHEMA:
        raise ValueError("persistent software-zero schema mismatch")
    if document.get("reference_name") != REFERENCE_NAME:
        raise ValueError("persistent software-zero reference mismatch")
    motors = document.get("motors")
    if not isinstance(motors, dict) or set(motors) != EXPECTED_MOTORS:
        raise ValueError("persistent software-zero must contain exactly seven motors")
    for name, value in motors.items():
        if not isinstance(value, dict):
            raise ValueError(f"invalid motor record: {name}")
        raw = value.get("raw_position_rad")
        if type(raw) not in {int, float} or not math.isfinite(float(raw)):
            raise ValueError(f"non-finite raw reference: {name}")
    writes = document.get("writes")
    if not isinstance(writes, dict) or any(
        writes.get(name) is not False
        for name in (
            "motor_internal_zero_modified",
            "rid_written",
            "flash_or_eeprom_written",
        )
    ):
        raise ValueError("persistent software-zero records a forbidden write")
    mapping = document.get("mapping")
    if not isinstance(mapping, dict):
        raise ValueError("persistent software-zero mapping is missing")
    ratio = mapping.get("gear_ratio")
    if type(ratio) not in {int, float} or not math.isclose(
        float(ratio), GEAR_RATIO, rel_tol=0.0, abs_tol=1e-9
    ):
        raise ValueError("unexpected gear ratio")
    signs = mapping.get("signs")
    if not isinstance(signs, dict) or any(
        signs.get(name) != sign for name, sign in J2_SIGNS.items()
    ):
        raise ValueError("unexpected J2 motor signs")


def validate_hints(document: dict[str, Any]) -> None:
    if document.get("schema") != HINT_SCHEMA:
        raise ValueError("recovery-hint schema mismatch")
    if document.get("reference_name") != REFERENCE_NAME:
        raise ValueError("recovery-hint reference mismatch")
    motors = document.get("motors")
    if not isinstance(motors, dict) or not set(motors).issubset(EXPECTED_MOTORS):
        raise ValueError("recovery hints contain an unknown motor")
    if not set(J2_SIGNS).issubset(motors):
        raise ValueError("recovery hints are missing J2 motors")
    for name, value in motors.items():
        if not isinstance(value, dict):
            raise ValueError(f"invalid recovery-hint record: {name}")
        logical = value.get("logical_position_rad")
        if type(logical) not in {int, float} or not math.isfinite(float(logical)):
            raise ValueError(f"non-finite recovery hint: {name}")


def validate_initial_pose(document: dict[str, Any], current_sha256: str) -> None:
    if document.get("有效") is not True:
        raise ValueError("initial pose is not marked valid")
    expected_session = f"persistent:{current_sha256[:16]}"
    if document.get("会话标识") != expected_session:
        raise ValueError("initial pose is bound to a different software-zero hash")
    positions = document.get("关节位置_弧度")
    expected = {f"J{index}" for index in range(1, 7)}
    if not isinstance(positions, dict) or set(positions) != expected:
        raise ValueError("initial pose must contain exactly six joints")
    if not all(
        type(value) in {int, float} and math.isfinite(float(value))
        for value in positions.values()
    ):
        raise ValueError("initial pose contains a non-finite position")


def validate_measurement(raw: float, span: float, sample_count: int, name: str) -> None:
    if not math.isfinite(raw) or abs(raw) > MAX_PROTOCOL_POSITION_RAD:
        raise ValueError(f"{name} raw position is outside the protocol range")
    if not math.isfinite(span) or not 0.0 <= span <= MAX_CAPTURE_SPAN_RAW_RAD:
        raise ValueError(f"{name} capture span is unsafe")
    if sample_count < 50:
        raise ValueError("at least 50 samples are required")


def parse_sidecar(path: Path, expected_basename: str) -> str:
    fields = path.read_text(encoding="utf-8").strip().split()
    if len(fields) != 2 or fields[1] != expected_basename:
        raise ValueError("persistent software-zero checksum sidecar is malformed")
    value = fields[0].lower()
    if len(value) != 64 or any(character not in "0123456789abcdef" for character in value):
        raise ValueError("persistent software-zero checksum is malformed")
    return value


def build_documents(
    zero: dict[str, Any],
    hints: dict[str, Any],
    *,
    original_sha256: str,
    j2a_raw: float,
    j2b_raw: float,
    j2a_span: float,
    j2b_span: float,
    sample_count: int,
    recorded_at_utc: str,
    source: str,
) -> tuple[dict[str, Any], dict[str, Any]]:
    validate_timestamp(recorded_at_utc)
    validate_measurement(j2a_raw, j2a_span, sample_count, "J2A")
    validate_measurement(j2b_raw, j2b_span, sample_count, "J2B")
    updated_zero = copy.deepcopy(zero)
    updated_hints = copy.deepcopy(hints)
    measurements = {
        "J2A": {
            "old_raw_position_rad": float(zero["motors"]["J2A"]["raw_position_rad"]),
            "new_raw_position_rad": j2a_raw,
            "sample_count": sample_count,
            "sample_span_rad": j2a_span,
        },
        "J2B": {
            "old_raw_position_rad": float(zero["motors"]["J2B"]["raw_position_rad"]),
            "new_raw_position_rad": j2b_raw,
            "sample_count": sample_count,
            "sample_span_rad": j2b_span,
        },
    }
    for name, raw, span in (
        ("J2A", j2a_raw, j2a_span),
        ("J2B", j2b_raw, j2b_span),
    ):
        updated_zero["motors"][name]["raw_position_rad"] = raw
        updated_zero["motors"][name]["sample_count"] = sample_count
        updated_zero["motors"][name]["sample_span_rad"] = span
        updated_hints["motors"][name]["logical_position_rad"] = 0.0
    updated_zero["last_software_zero_rebase"] = {
        "schema": "go-m8010-software-zero-rebase-record/1.0",
        "recorded_at_utc": recorded_at_utc,
        "source": source,
        "scope": ["J2A", "J2B"],
        "original_persistent_zero_sha256": original_sha256,
        "measurements": measurements,
        "motor_internal_zero_modified": False,
        "rid_written": False,
        "flash_or_eeprom_written": False,
    }
    updated_hints["source"] = source
    updated_hints["updated_at_utc"] = recorded_at_utc
    validate_zero(updated_zero)
    validate_hints(updated_hints)
    return updated_zero, updated_hints


def build_initial_pose(
    initial_pose: dict[str, Any], *, new_sha256: str, recorded_at_utc: str
) -> dict[str, Any]:
    updated = copy.deepcopy(initial_pose)
    updated["会话标识"] = f"persistent:{new_sha256[:16]}"
    updated["时间戳"] = recorded_at_utc
    return updated


def stage_file(path: Path, data: bytes, mode: int) -> Path:
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.chmod(temporary, mode)
        return temporary
    except Exception:
        temporary.unlink(missing_ok=True)
        raise


def fsync_directory(path: Path) -> None:
    # Windows does not permit opening a directory with os.open().  Production
    # apply runs on Linux; keeping this a no-op on Windows lets the pure
    # transactional tests exercise the same staging and backup logic.
    if os.name == "nt":
        return
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def apply_documents(
    *,
    zero_path: Path,
    sidecar_path: Path,
    hint_path: Path,
    initial_pose_path: Path,
    backup_dir: Path,
    zero_data: bytes,
    hint_data: bytes,
    initial_pose_data: bytes,
    sidecar_data: bytes,
) -> None:
    if backup_dir.exists():
        raise ValueError("backup directory already exists")
    backup_dir.mkdir(parents=True, exist_ok=False)
    for source in (zero_path, sidecar_path, hint_path, initial_pose_path):
        shutil.copy2(source, backup_dir / source.name)
    fsync_directory(backup_dir)
    staged: list[tuple[Path, Path]] = []
    try:
        for path, data in (
            (zero_path, zero_data),
            (hint_path, hint_data),
            (initial_pose_path, initial_pose_data),
            (sidecar_path, sidecar_data),
        ):
            mode = path.stat().st_mode & 0o777
            staged.append((stage_file(path, data, mode), path))
        # The checksum is the commit marker and is deliberately replaced last.
        for temporary, destination in staged:
            os.replace(temporary, destination)
        fsync_directory(zero_path.parent)
    finally:
        for temporary, _ in staged:
            temporary.unlink(missing_ok=True)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--zero-file", type=Path, required=True)
    parser.add_argument("--zero-sha256-file", type=Path, required=True)
    parser.add_argument("--recovery-hint-file", type=Path, required=True)
    parser.add_argument("--initial-pose-file", type=Path, required=True)
    parser.add_argument("--expected-current-sha256", required=True)
    parser.add_argument("--j2a-raw-position-rad", type=float, required=True)
    parser.add_argument("--j2b-raw-position-rad", type=float, required=True)
    parser.add_argument("--j2a-sample-span-rad", type=float, required=True)
    parser.add_argument("--j2b-sample-span-rad", type=float, required=True)
    parser.add_argument("--sample-count", type=int, required=True)
    parser.add_argument("--recorded-at-utc", required=True)
    parser.add_argument("--source", required=True)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--backup-dir", type=Path)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    zero_path = args.zero_file.resolve(strict=True)
    sidecar_path = args.zero_sha256_file.resolve(strict=True)
    hint_path = args.recovery_hint_file.resolve(strict=True)
    initial_pose_path = args.initial_pose_file.resolve(strict=True)
    zero, zero_original_data = read_json(zero_path)
    hints, _ = read_json(hint_path)
    initial_pose, _ = read_json(initial_pose_path)
    validate_zero(zero)
    validate_hints(hints)
    actual_sha256 = sha256_bytes(zero_original_data)
    expected_sha256 = args.expected_current_sha256.lower()
    if actual_sha256 != expected_sha256:
        raise SystemExit("expected current persistent software-zero hash does not match")
    if parse_sidecar(sidecar_path, zero_path.name) != actual_sha256:
        raise SystemExit("persistent software-zero checksum verification failed")
    validate_initial_pose(initial_pose, actual_sha256)
    updated_zero, updated_hints = build_documents(
        zero,
        hints,
        original_sha256=actual_sha256,
        j2a_raw=args.j2a_raw_position_rad,
        j2b_raw=args.j2b_raw_position_rad,
        j2a_span=args.j2a_sample_span_rad,
        j2b_span=args.j2b_sample_span_rad,
        sample_count=args.sample_count,
        recorded_at_utc=args.recorded_at_utc,
        source=args.source,
    )
    zero_data = json_bytes(updated_zero)
    hint_data = json_bytes(updated_hints)
    new_sha256 = sha256_bytes(zero_data)
    updated_initial_pose = build_initial_pose(
        initial_pose,
        new_sha256=new_sha256,
        recorded_at_utc=args.recorded_at_utc,
    )
    initial_pose_data = json_bytes(updated_initial_pose)
    sidecar_data = f"{new_sha256}  {zero_path.name}\n".encode("ascii")
    if args.apply:
        if args.confirm != APPLY_GATE:
            raise SystemExit("apply confirmation gate missing")
        if args.backup_dir is None:
            raise SystemExit("--backup-dir is required in apply mode")
        apply_documents(
            zero_path=zero_path,
            sidecar_path=sidecar_path,
            hint_path=hint_path,
            initial_pose_path=initial_pose_path,
            backup_dir=args.backup_dir.resolve(),
            zero_data=zero_data,
            hint_data=hint_data,
            initial_pose_data=initial_pose_data,
            sidecar_data=sidecar_data,
        )
        written_zero, written_data = read_json(zero_path)
        written_hints, _ = read_json(hint_path)
        written_initial_pose, _ = read_json(initial_pose_path)
        validate_zero(written_zero)
        validate_hints(written_hints)
        validate_initial_pose(written_initial_pose, new_sha256)
        if sha256_bytes(written_data) != new_sha256:
            raise RuntimeError("post-write persistent software-zero hash mismatch")
        if parse_sidecar(sidecar_path, zero_path.name) != new_sha256:
            raise RuntimeError("post-write checksum sidecar mismatch")
    summary = {
        "schema": "go-m8010-j2-software-zero-rebase-result/1.0",
        "applied": bool(args.apply),
        "hardware_opened": False,
        "motor_internal_zero_modified": False,
        "rid_written": False,
        "flash_or_eeprom_written": False,
        "old_persistent_zero_sha256": actual_sha256,
        "new_persistent_zero_sha256": new_sha256,
        "J2A_raw_position_rad": args.j2a_raw_position_rad,
        "J2B_raw_position_rad": args.j2b_raw_position_rad,
        "J2_recovery_hints_rad": {"J2A": 0.0, "J2B": 0.0},
        "backup_dir": str(args.backup_dir.resolve()) if args.apply else None,
    }
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

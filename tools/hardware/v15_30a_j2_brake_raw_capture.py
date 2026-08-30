#!/usr/bin/env python3
"""Capture J2 raw feedback under an independently proven BRAKE-only worker.

The capture intentionally does not load a persistent software zero, recovery
hint, or integer-turn branch.  It listens only on the loopback feedback port,
starts the hash-pinned production worker with ``--brake-only``, and publishes
one no-overwrite JSON evidence file.  A PASS file is directly consumable by
``v15_30a_create_j2_vertical_session_phase_anchor.py``.

This program never sends a UDP command and never authorizes FOC, HOLD, or
position control.  It does not modify a motor-internal zero, RID, flash, or
EEPROM.
"""

from __future__ import annotations

import argparse
import json
import math
import os
import socket
import subprocess
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any

try:
    from tools.hardware import v15_30a_j2_brake_feedback_verify as brake_verify
except ModuleNotFoundError as exc:
    if exc.name not in {"tools", "tools.hardware"}:
        raise
    import v15_30a_j2_brake_feedback_verify as brake_verify


CAPTURE_SCHEMA = "go-m8010-j2-brake-raw-capture-statistics/1.0"
CONFIRM_GATE = "V15_30A_J2_BRAKE_RAW_CAPTURE=YES"
PHYSICAL_GATE = (
    "24V_ON=YES;SUPPORT_RELIABLE=YES;J2_VERTICAL_INITIALIZATION_POSE=YES;"
    "ARM_NOT_MOVED=YES;ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES"
)
MOTOR_NAMES = ("J2A", "J2B")
MINIMUM_PACKETS = 500
TARGET_PACKETS = 500
MINIMUM_COVERAGE_S = 4.0
# GO-M8010-6 encodes position as a signed int32 Q15 value.  Unlike the
# DM-G6220 PMAX envelope, a valid GO multi-turn reading is not limited to
# +/-12.5 rad; the representable magnitude is 2**31 / 2**15 = 65536 rad.
MAXIMUM_RAW_POSITION_RAD = float(1 << 16)
MAXIMUM_RAW_SPAN_RAD = brake_verify.GEAR_RATIO * math.radians(0.2)
MAXIMUM_TEMPERATURE_C = brake_verify.MAX_TEMPERATURE_C
MAXIMUM_FEEDBACK_AGE_NS = brake_verify.MAX_FEEDBACK_AGE_NS
MAXIMUM_FEEDBACK_GAP_NS = brake_verify.MAX_FEEDBACK_GAP_NS
HOST_BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")
RUNTIME_HARDWARE_ACCESSED = False


class CaptureError(RuntimeError):
    """Raised before hardware access or for invalid captured evidence."""


def validate_pose_binding_id(value: Any) -> str:
    """Accept only an exact, caller-supplied lowercase SHA-256 token."""

    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise ValueError(
            "pose binding id must be exactly 64 lowercase hexadecimal characters"
        )
    return value


def finite_number(value: Any, label: str) -> float:
    if type(value) not in {int, float} or not math.isfinite(float(value)):
        raise CaptureError(f"non-finite {label}")
    return float(value)


def utc_text(value: datetime) -> str:
    return value.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")


def validate_host_boot_id(value: Any) -> str:
    if not isinstance(value, str):
        raise CaptureError("host boot id must be a lowercase canonical UUID")
    try:
        parsed = uuid.UUID(value)
    except (AttributeError, ValueError) as exc:
        raise CaptureError("host boot id must be a lowercase canonical UUID") from exc
    if str(parsed) != value or parsed.int == 0:
        raise CaptureError("host boot id must be a lowercase canonical UUID")
    return value


def validate_recorded_boottime_ns(value: Any) -> int:
    if type(value) is not int or value <= 0:
        raise CaptureError("recorded CLOCK_BOOTTIME must be a positive integer")
    return value


def read_host_boot_id(path: Path = HOST_BOOT_ID_PATH) -> str:
    try:
        value = path.read_text(encoding="ascii").strip()
    except (OSError, UnicodeError) as exc:
        raise CaptureError(f"host boot id is unavailable: {exc}") from exc
    return validate_host_boot_id(value)


def read_recorded_boottime_ns(
    *,
    clock_gettime_ns=None,
    clock_id=None,
) -> int:
    reader = getattr(time, "clock_gettime_ns", None) if clock_gettime_ns is None else clock_gettime_ns
    boot_clock = getattr(time, "CLOCK_BOOTTIME", None) if clock_id is None else clock_id
    if not callable(reader) or type(boot_clock) is not int:
        raise CaptureError("CLOCK_BOOTTIME is unavailable on this host")
    try:
        value = reader(boot_clock)
    except (OSError, ValueError) as exc:
        raise CaptureError(f"CLOCK_BOOTTIME read failed: {exc}") from exc
    return validate_recorded_boottime_ns(value)


def publish_json_no_overwrite(path: Path, document: dict[str, Any]) -> None:
    """Atomically publish one immutable evidence name without replacement."""

    destination = path.resolve()
    destination.parent.mkdir(parents=True, exist_ok=True)
    data = (json.dumps(document, ensure_ascii=False, indent=2) + "\n").encode(
        "utf-8"
    )
    temporary = destination.with_name(
        f".{destination.name}.{uuid.uuid4().hex}.tmp"
    )
    descriptor = os.open(
        temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600
    )
    published = False
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, destination)
        published = True
        if os.name != "nt":
            directory_fd = os.open(
                destination.parent,
                os.O_RDONLY | getattr(os, "O_DIRECTORY", 0),
            )
            try:
                os.fsync(directory_fd)
            finally:
                os.close(directory_fd)
    finally:
        temporary.unlink(missing_ok=True)
    if not published:
        raise CaptureError("raw capture evidence was not published")


def new_capture_identity(
    *,
    now: datetime | None = None,
    token: str | None = None,
    host_boot_id: str | None = None,
    recorded_boottime_ns: int | None = None,
) -> dict[str, Any]:
    instant = datetime.now(timezone.utc) if now is None else now
    if instant.tzinfo is None or instant.utcoffset() is None:
        raise CaptureError("capture identity time must be timezone-aware")
    nonce = uuid.uuid4().hex if token is None else token.lower()
    if len(nonce) != 32 or any(character not in "0123456789abcdef" for character in nonce):
        raise CaptureError("capture identity token must be 32 lowercase hex characters")
    timestamp = instant.astimezone(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    boot_id = (
        read_host_boot_id()
        if host_boot_id is None
        else validate_host_boot_id(host_boot_id)
    )
    boottime_ns = (
        read_recorded_boottime_ns()
        if recorded_boottime_ns is None
        else validate_recorded_boottime_ns(recorded_boottime_ns)
    )
    return {
        "capture_id": f"j2-brake-raw-{timestamp}-{nonce[:16]}",
        "power_session_id": f"j2-power-session-{timestamp}-{nonce}",
        "recorded_at_utc": utc_text(instant),
        "host_boot_id": boot_id,
        "recorded_boottime_ns": boottime_ns,
    }


def validate_gates(confirm: str, physical_confirmation: str) -> None:
    if confirm != CONFIRM_GATE:
        raise CaptureError(f"require --confirm {CONFIRM_GATE}")
    if physical_confirmation != PHYSICAL_GATE:
        raise CaptureError(
            "require complete 24V-on/vertical/support/not-moved/stationary/"
            "not-at-limit physical gate"
        )


def build_worker_command(args: argparse.Namespace) -> list[str]:
    """Build a worker command with no command socket or old reference inputs."""
    return [
        str(args.worker),
        "--execute",
        "--brake-only",
        "--confirm",
        brake_verify.WORKER_GATE,
        "--bus",
        "j2",
        "--feedback-port",
        str(args.feedback_port),
    ]


def raw_statistics(values: list[float], name: str) -> dict[str, float]:
    if len(values) < MINIMUM_PACKETS:
        raise CaptureError(f"{name} has fewer than {MINIMUM_PACKETS} raw samples")
    mean = math.fsum(values) / len(values)
    minimum = min(values)
    maximum = max(values)
    span = maximum - minimum
    variance = math.fsum((value - mean) ** 2 for value in values) / len(values)
    standard_deviation = math.sqrt(max(variance, 0.0))
    if any(
        not math.isfinite(value)
        for value in (mean, minimum, maximum, span, standard_deviation)
    ):
        raise CaptureError(f"{name} raw statistics are non-finite")
    # This is the live MotorData.q protocol envelope, not the persistent
    # software-reference representation (which may include added 2*pi turns).
    if any(
        abs(value) > MAXIMUM_RAW_POSITION_RAD
        for value in (mean, minimum, maximum)
    ):
        raise CaptureError(f"{name} raw position exceeds capture protocol range")
    if span > MAXIMUM_RAW_SPAN_RAD:
        raise CaptureError(f"{name} raw span {span:.9f} rad is not stationary")
    return {
        "mean": mean,
        "minimum": minimum,
        "maximum": maximum,
        "span": span,
        "standard_deviation": standard_deviation,
    }


def analyze_raw_packets(
    packets: list[dict[str, Any]],
    received_monotonic_ns: list[int],
    source_addresses: list[tuple[str, int]],
) -> dict[str, Any]:
    if len(packets) < MINIMUM_PACKETS:
        raise CaptureError(
            f"feedback packet count {len(packets)} is below {MINIMUM_PACKETS}"
        )
    if len(received_monotonic_ns) != len(packets) or len(source_addresses) != len(
        packets
    ):
        raise CaptureError("feedback capture metadata length mismatch")
    if any(type(stamp) is not int or stamp <= 0 for stamp in received_monotonic_ns):
        raise CaptureError("local receive timestamp invalid")
    if any(
        right <= left
        for left, right in zip(received_monotonic_ns, received_monotonic_ns[1:])
    ):
        raise CaptureError("local receive timestamps are not strictly increasing")

    local_coverage_s = (
        received_monotonic_ns[-1] - received_monotonic_ns[0]
    ) / 1.0e9
    if local_coverage_s < MINIMUM_COVERAGE_S:
        raise CaptureError(
            f"local feedback coverage {local_coverage_s:.3f}s is too short"
        )
    local_gaps_ns = [
        right - left
        for left, right in zip(received_monotonic_ns, received_monotonic_ns[1:])
    ]
    maximum_local_gap_ns = max(local_gaps_ns)
    if maximum_local_gap_ns > MAXIMUM_FEEDBACK_GAP_NS:
        raise CaptureError(
            f"local feedback gap {maximum_local_gap_ns / 1.0e9:.3f}s is too long"
        )

    first_source = source_addresses[0]
    if (
        first_source[0] != "127.0.0.1"
        or type(first_source[1]) is not int
        or not 1 <= first_source[1] <= 65535
        or any(address != first_source for address in source_addresses)
    ):
        raise CaptureError("feedback UDP source endpoint changed or is invalid")

    source_stamps: list[int] = []
    feedback_ages_ns: list[int] = []
    raw_by_motor: dict[str, list[float]] = {name: [] for name in MOTOR_NAMES}
    temperatures: dict[str, list[float]] = {name: [] for name in MOTOR_NAMES}

    for packet_index, packet in enumerate(packets):
        if packet.get("schema") != "go-m8010-motor-feedback/1.0":
            raise CaptureError(f"packet {packet_index} schema mismatch")
        source_stamp = packet.get("source_monotonic_ns")
        if type(source_stamp) is not int or source_stamp <= 0:
            raise CaptureError(f"packet {packet_index} timestamp invalid")
        source_stamps.append(source_stamp)
        age_ns = received_monotonic_ns[packet_index] - source_stamp
        if age_ns < 0 or age_ns > MAXIMUM_FEEDBACK_AGE_NS:
            raise CaptureError(
                f"packet {packet_index} source timestamp is stale or future-dated"
            )
        feedback_ages_ns.append(age_ns)

        if packet.get("controller_mode") != "brake":
            raise CaptureError(f"packet {packet_index} left BRAKE")
        motor_modes = packet.get("controller_mode_by_motor")
        if not isinstance(motor_modes, dict) or any(
            motor_modes.get(name) != "brake" for name in MOTOR_NAMES
        ):
            raise CaptureError(f"packet {packet_index} motor mode was not BRAKE")
        if any(
            packet.get(field) is not False
            for field in ("j2_sync_fault", "domain_fault", "lease_safe_hold")
        ):
            raise CaptureError(f"packet {packet_index} reported a controller fault")

        samples = packet.get("samples")
        if not isinstance(samples, list):
            raise CaptureError(f"packet {packet_index} samples missing")
        by_name = {
            sample.get("motor"): sample
            for sample in samples
            if isinstance(sample, dict) and sample.get("motor") in MOTOR_NAMES
        }
        if len(samples) != len(MOTOR_NAMES) or set(by_name) != set(MOTOR_NAMES):
            raise CaptureError(f"packet {packet_index} J2 sample set mismatch")

        for name in MOTOR_NAMES:
            sample = by_name[name]
            if sample.get("communication_ok") is not True:
                raise CaptureError(f"packet {packet_index} {name} communication failed")
            if sample.get("merror") != 0:
                raise CaptureError(f"packet {packet_index} {name} merror is nonzero")
            raw = finite_number(
                sample.get("unwrapped_raw_position_rad"), f"{name} raw position"
            )
            temperature = finite_number(
                sample.get("temperature_c"), f"{name} temperature"
            )
            if not 0.0 <= temperature < MAXIMUM_TEMPERATURE_C:
                raise CaptureError(f"packet {packet_index} {name} temperature unsafe")
            raw_by_motor[name].append(raw)
            temperatures[name].append(temperature)

    if any(right <= left for left, right in zip(source_stamps, source_stamps[1:])):
        raise CaptureError("source timestamps are not strictly increasing")
    source_coverage_s = (source_stamps[-1] - source_stamps[0]) / 1.0e9
    if source_coverage_s < MINIMUM_COVERAGE_S:
        raise CaptureError(
            f"feedback source coverage {source_coverage_s:.3f}s is too short"
        )
    source_gaps_ns = [
        right - left for left, right in zip(source_stamps, source_stamps[1:])
    ]
    maximum_source_gap_ns = max(source_gaps_ns)
    if maximum_source_gap_ns > MAXIMUM_FEEDBACK_GAP_NS:
        raise CaptureError(
            f"source feedback gap {maximum_source_gap_ns / 1.0e9:.3f}s is too long"
        )

    motors = {
        name: {
            "sample_count": len(raw_by_motor[name]),
            "unwrapped_raw_position_rad": raw_statistics(raw_by_motor[name], name),
        }
        for name in MOTOR_NAMES
    }
    return {
        "packet_count": len(packets),
        "source_coverage_s": source_coverage_s,
        "local_receive_coverage_s": local_coverage_s,
        "maximum_source_gap_s": maximum_source_gap_ns / 1.0e9,
        "maximum_local_gap_s": maximum_local_gap_ns / 1.0e9,
        "feedback_age_min_s": min(feedback_ages_ns) / 1.0e9,
        "feedback_age_max_s": max(feedback_ages_ns) / 1.0e9,
        "feedback_source_endpoint": f"{first_source[0]}:{first_source[1]}",
        "all_controller_modes": ["brake"],
        "communication_failure_packets": 0,
        "merror_nonzero_packets": 0,
        "controller_fault_packets": 0,
        "motors": motors,
        "temperature_min_c": {
            name: min(values) for name, values in temperatures.items()
        },
        "temperature_max_c": {
            name: max(values) for name, values in temperatures.items()
        },
    }


def build_capture_document(
    *,
    pose_binding_id: str,
    identity: dict[str, Any],
    statistics: dict[str, Any],
    worker_path: Path,
    worker_sha256: str,
    worker_returncode: int,
    stop_escalation: str,
    terminal_proof: dict[str, str],
    stdout: str,
    stderr: str,
    feedback_port: int,
) -> dict[str, Any]:
    pose_binding_id = validate_pose_binding_id(pose_binding_id)
    if worker_returncode != 0:
        raise CaptureError("cannot build PASS capture from a failed worker")
    if stop_escalation != "none":
        raise CaptureError("cannot build PASS capture after worker stop escalation")
    if terminal_proof.get("EXECUTION_POLICY") != "BRAKE_ONLY":
        raise CaptureError("worker did not prove BRAKE_ONLY execution")
    if terminal_proof.get("COMMAND_RX_ENABLED") != "NO":
        raise CaptureError("worker command reception was enabled")
    if terminal_proof.get("J2_SESSION_REFERENCE_CONFIGURED") != "NO":
        raise CaptureError("worker unexpectedly loaded a J2 session reference")
    try:
        phase_tx_accounting = brake_verify.validate_worker_cycle_accounting(
            terminal_proof, statistics["packet_count"]
        )
    except brake_verify.VerificationError as exc:
        raise CaptureError(str(exc)) from exc
    startup_prime = {
        "state": terminal_proof["J2_STARTUP_BRAKE_PRIME_STATE"],
        "maximum_invalid_prefix_pairs": int(
            terminal_proof[
                "J2_STARTUP_BRAKE_PRIME_MAX_INVALID_PREFIX_PAIRS"
            ]
        ),
        "invalid_prefix_pairs": int(
            terminal_proof["J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS"]
        ),
        "required_healthy_pairs": int(
            terminal_proof[
                "J2_STARTUP_BRAKE_PRIME_REQUIRED_HEALTHY_PAIRS"
            ]
        ),
        "healthy_pairs": int(
            terminal_proof["J2_STARTUP_BRAKE_PRIME_HEALTHY_PAIRS"]
        ),
        "attempted_pairs": int(
            terminal_proof["J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS"]
        ),
        "tx_attempt_count": int(
            terminal_proof["J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT"]
        ),
        "elapsed_ns": int(
            terminal_proof["J2_STARTUP_BRAKE_PRIME_ELAPSED_NS"]
        ),
        "feedback_published_during_prime": False,
        "window_reopens_after_first_healthy_pair": False,
    }

    safety = {
        "execution_policy": "BRAKE_ONLY",
        "all_controller_modes": statistics["all_controller_modes"],
        "command_rx_enabled": False,
        "j2_session_reference_configured": False,
        "foc_tx_attempt_count": int(terminal_proof["FOC_TX_ATTEMPT_COUNT"]),
        "foc_serial_send_call_count": int(
            terminal_proof["FOC_SERIAL_SEND_CALL_COUNT"]
        ),
        "other_mode_tx_attempt_count": int(
            terminal_proof["OTHER_MODE_TX_ATTEMPT_COUNT"]
        ),
        "brake_only_guard_block_count": int(
            terminal_proof["BRAKE_ONLY_GUARD_BLOCK_COUNT"]
        ),
        "active_or_hold_commands_sent": 0,
        "active_commands_sent": 0,
        "hold_commands_sent": 0,
        "communication_failure_packets": statistics[
            "communication_failure_packets"
        ],
        "merror_nonzero_packets": statistics["merror_nonzero_packets"],
        "motor_internal_zero_modified": False,
        "motor_internal_zero_write_count": 0,
        "rid_written": False,
        "rid_write_count": 0,
        "flash_or_eeprom_written": False,
        "flash_or_eeprom_write_count": 0,
        "startup_brake_prime_passed": True,
        "startup_invalid_prefix_pairs": startup_prime[
            "invalid_prefix_pairs"
        ],
    }
    return {
        "schema": CAPTURE_SCHEMA,
        "pose_binding_id": pose_binding_id,
        "capture_id": identity["capture_id"],
        "power_session_id": identity["power_session_id"],
        "recorded_at_utc": identity["recorded_at_utc"],
        "host_boot_id": identity["host_boot_id"],
        "recorded_boottime_ns": identity["recorded_boottime_ns"],
        "status": "PASS",
        "hardware_accessed": True,
        "physical_power_state_during_capture": "24V_ON",
        "packet_count": statistics["packet_count"],
        "source_coverage_s": statistics["source_coverage_s"],
        "safety": safety,
        "motors": statistics["motors"],
        "physical_confirmation": {
            "confirmation_gate": PHYSICAL_GATE,
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
            "udp_listener_bind": f"127.0.0.1:{feedback_port}",
            "udp_send_calls": 0,
            "feedback_source_endpoint": statistics["feedback_source_endpoint"],
            "local_receive_coverage_s": statistics["local_receive_coverage_s"],
            "maximum_source_gap_s": statistics["maximum_source_gap_s"],
            "maximum_local_gap_s": statistics["maximum_local_gap_s"],
            "feedback_age_min_s": statistics["feedback_age_min_s"],
            "feedback_age_max_s": statistics["feedback_age_max_s"],
            "controller_fault_packets": statistics["controller_fault_packets"],
            "startup_brake_prime": startup_prime,
            "phase_tx_accounting": phase_tx_accounting,
            "temperature_min_c": statistics["temperature_min_c"],
            "temperature_max_c": statistics["temperature_max_c"],
            "worker": {
                "path": str(worker_path),
                "sha256": worker_sha256,
                "returncode": worker_returncode,
                "stop_escalation": stop_escalation,
                "forced_kill": False,
                "terminal_proof": terminal_proof,
                "stdout": stdout.splitlines(),
                "stderr": stderr.splitlines(),
            },
        },
        "physical_power_off_required": False,
    }


def failure_document(
    *,
    pose_binding_id: str,
    identity: dict[str, Any],
    worker_path: Path,
    worker_sha256: str,
    process: subprocess.Popen[str] | None,
    stop_escalation: str,
    stdout: str,
    stderr: str,
    feedback_port: int,
    packet_count: int,
    failures: list[str],
) -> dict[str, Any]:
    pose_binding_id = validate_pose_binding_id(pose_binding_id)
    return {
        "schema": CAPTURE_SCHEMA,
        "pose_binding_id": pose_binding_id,
        "capture_id": identity["capture_id"],
        "power_session_id": identity["power_session_id"],
        "recorded_at_utc": identity["recorded_at_utc"],
        "host_boot_id": identity["host_boot_id"],
        "recorded_boottime_ns": identity["recorded_boottime_ns"],
        "status": "FAIL",
        "hardware_accessed": process is not None,
        "physical_power_state_during_capture": "24V_ON",
        "packet_count": packet_count,
        "failures": failures,
        "raw_safety_summary": {
            "branch_inputs_loaded": False,
            "persistent_zero_loaded": False,
            "recovery_hint_loaded": False,
            "integer_turn_branch_inferred": False,
            "udp_listener_bind": f"127.0.0.1:{feedback_port}",
            "udp_send_calls": 0,
            "worker": {
                "path": str(worker_path),
                "sha256": worker_sha256,
                "returncode": None if process is None else process.returncode,
                "stop_escalation": stop_escalation,
                "forced_kill": stop_escalation == "SIGKILL",
                "stdout": stdout.splitlines(),
                "stderr": stderr.splitlines(),
            },
        },
        "physical_power_off_required": True,
    }


def run(args: argparse.Namespace) -> int:
    global RUNTIME_HARDWARE_ACCESSED
    RUNTIME_HARDWARE_ACCESSED = False
    pose_binding_id = validate_pose_binding_id(args.pose_binding_id)
    validate_gates(args.confirm, args.physical_confirmation)
    try:
        worker = args.worker.resolve(strict=True)
    except OSError as exc:
        raise CaptureError(f"worker path is unavailable: {exc}") from exc
    try:
        expected_worker_sha256 = brake_verify._normalized_sha256(
            args.expected_worker_sha256, "expected worker"
        )
    except brake_verify.VerificationError as exc:
        raise CaptureError(str(exc)) from exc
    worker_sha256 = brake_verify.sha256_file(worker)
    if worker_sha256 != expected_worker_sha256:
        raise CaptureError(f"worker SHA-256 mismatch: {worker_sha256}")
    output = args.output.resolve()
    brake_verify.prepare_evidence_destination(output)
    identity = new_capture_identity()

    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 0)
    listener.bind(("127.0.0.1", args.feedback_port))
    listener.settimeout(0.20)
    command_args = argparse.Namespace(worker=worker, feedback_port=args.feedback_port)
    command = build_worker_command(command_args)
    process: subprocess.Popen[str] | None = None
    packets: list[dict[str, Any]] = []
    received_monotonic_ns: list[int] = []
    source_addresses: list[tuple[str, int]] = []
    collection_error: BaseException | None = None
    stdout = stderr = ""
    stop_escalation = "none"
    started_at = time.monotonic()
    try:
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
        RUNTIME_HARDWARE_ACCESSED = True
        deadline = started_at + args.maximum_runtime_s
        while len(packets) < args.target_packets and time.monotonic() < deadline:
            if process.poll() is not None:
                break
            try:
                payload, address = listener.recvfrom(65535)
            except socket.timeout:
                continue
            received_at_ns = time.monotonic_ns()
            if address[0] != "127.0.0.1":
                raise CaptureError("feedback arrived from a non-loopback source")
            try:
                document = json.loads(payload)
            except json.JSONDecodeError as exc:
                raise CaptureError("feedback JSON decode failed") from exc
            if not isinstance(document, dict):
                raise CaptureError("feedback packet is not a JSON object")
            packets.append(document)
            received_monotonic_ns.append(received_at_ns)
            source_addresses.append((address[0], int(address[1])))
    except BaseException as exc:
        collection_error = exc
    finally:
        listener.close()
        if process is not None:
            stdout, stderr, stop_escalation = brake_verify.stop_worker(process)

    failures: list[str] = []
    if collection_error is not None:
        failures.append(f"{type(collection_error).__name__}:{collection_error}")
    if process is None or process.returncode != 0:
        failures.append("worker did not exit successfully")
    if stop_escalation != "none":
        failures.append(
            f"worker stop escalated to {stop_escalation}; physical power-off is required"
        )

    terminal_proof: dict[str, str] | None = None
    try:
        terminal_proof = brake_verify.parse_worker_terminal_proof(stdout)
    except brake_verify.VerificationError as exc:
        failures.append(str(exc))

    statistics: dict[str, Any] | None = None
    try:
        statistics = analyze_raw_packets(
            packets, received_monotonic_ns, source_addresses
        )
    except CaptureError as exc:
        failures.append(str(exc))

    if not failures and process is not None and terminal_proof is not None and statistics:
        result = build_capture_document(
            pose_binding_id=pose_binding_id,
            identity=identity,
            statistics=statistics,
            worker_path=worker,
            worker_sha256=worker_sha256,
            worker_returncode=process.returncode,
            stop_escalation=stop_escalation,
            terminal_proof=terminal_proof,
            stdout=stdout,
            stderr=stderr,
            feedback_port=args.feedback_port,
        )
        returncode = 0
    else:
        result = failure_document(
            pose_binding_id=pose_binding_id,
            identity=identity,
            worker_path=worker,
            worker_sha256=worker_sha256,
            process=process,
            stop_escalation=stop_escalation,
            stdout=stdout,
            stderr=stderr,
            feedback_port=args.feedback_port,
            packet_count=len(packets),
            failures=failures,
        )
        returncode = 2
    publish_json_no_overwrite(output, result)
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return returncode


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", type=Path, required=True)
    parser.add_argument("--expected-worker-sha256", required=True)
    parser.add_argument("--feedback-port", type=int, default=15300)
    parser.add_argument("--target-packets", type=int, default=TARGET_PACKETS)
    parser.add_argument("--maximum-runtime-s", type=float, default=8.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--confirm", required=True)
    parser.add_argument("--physical-confirmation", required=True)
    parser.add_argument(
        "--pose-binding-id", type=validate_pose_binding_id, required=True
    )
    args = parser.parse_args(argv)
    if not 1024 <= args.feedback_port <= 65535:
        parser.error("feedback port out of range")
    if not MINIMUM_PACKETS <= args.target_packets <= 1500:
        parser.error(
            f"target packets must be in [{MINIMUM_PACKETS}, 1500]"
        )
    if not 5.0 <= args.maximum_runtime_s <= 15.0:
        parser.error("maximum runtime must be in [5, 15] seconds")
    return args


def main() -> int:
    args: argparse.Namespace | None = None
    try:
        args = parse_args()
        return run(args)
    except Exception as exc:
        hardware_accessed = RUNTIME_HARDWARE_ACCESSED
        document: dict[str, Any] = {
            "schema": "go-m8010-j2-brake-raw-capture-result/1.0",
            "status": "BLOCKED",
            "reason": str(exc),
            "hardware_accessed": hardware_accessed,
            "active_control_authorized": False,
            "physical_power_off_required": hardware_accessed,
        }
        if args is not None:
            pose_binding_id = getattr(args, "pose_binding_id", None)
            try:
                document["pose_binding_id"] = validate_pose_binding_id(
                    pose_binding_id
                )
            except ValueError:
                pass
        print(
            json.dumps(document, ensure_ascii=False, indent=2),
            flush=True,
        )
        return 4


if __name__ == "__main__":
    raise SystemExit(main())

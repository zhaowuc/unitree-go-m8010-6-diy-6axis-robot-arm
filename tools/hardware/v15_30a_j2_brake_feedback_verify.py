#!/usr/bin/env python3
"""Run the production J2 worker with no command source and audit BRAKE feedback.

This tool never sends a UDP command.  It binds only the worker feedback port,
starts the hash-pinned production GO worker in its default BRAKE state, captures
feedback, then requests SIGINT and requires the worker's terminal BRAKE proof.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import signal
import socket
import subprocess
import time
from pathlib import Path
from typing import Any


GEAR_RATIO = 6.329999923706055
MOTOR_SIGNS = {"J2A": -1, "J2B": 1}
CONFIRM_GATE = "J2_BRAKE_ONLY_FEEDBACK_VERIFY=YES"
WORKER_GATE = "V15_30A_GUI_GO_CONTROL_AUTHORIZED=YES"
MINIMUM_PACKETS = 450
TARGET_PACKETS = 500
MINIMUM_COVERAGE_S = 4.0
MAX_COMMON_ABS_DEG = 0.25
MAX_SYNC_ABS_DEG = 0.5
MAX_POSITION_SPAN_DEG = 0.2
MAX_TEMPERATURE_C = 60.0
MAX_FEEDBACK_AGE_NS = 250_000_000
MAX_FEEDBACK_GAP_NS = 250_000_000


class VerificationError(RuntimeError):
    pass


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _normalized_sha256(value: str, label: str) -> str:
    normalized = value.lower()
    if len(normalized) != 64 or any(
        character not in "0123456789abcdef" for character in normalized
    ):
        raise VerificationError(f"invalid {label} SHA-256")
    return normalized


def load_configuration(
    zero_path: Path,
    hint_path: Path,
    expected_zero_sha256: str,
    expected_hint_sha256: str,
) -> tuple[dict, dict, dict]:
    sidecar = Path(str(zero_path) + ".sha256")
    try:
        zero_bytes = zero_path.read_bytes()
        hint_bytes = hint_path.read_bytes()
        sidecar_fields = sidecar.read_text(encoding="utf-8").split()
        zero = json.loads(zero_bytes)
        hints = json.loads(hint_bytes)
    except (OSError, json.JSONDecodeError) as exc:
        raise VerificationError(f"configuration load failed: {exc}") from exc
    zero_sha256 = hashlib.sha256(zero_bytes).hexdigest()
    hint_sha256 = hashlib.sha256(hint_bytes).hexdigest()
    expected_zero_sha256 = _normalized_sha256(
        expected_zero_sha256, "expected persistent zero"
    )
    expected_hint_sha256 = _normalized_sha256(
        expected_hint_sha256, "expected recovery hint"
    )
    if zero_sha256 != expected_zero_sha256:
        raise VerificationError("persistent zero does not match the pinned SHA-256")
    if hint_sha256 != expected_hint_sha256:
        raise VerificationError("recovery hint does not match the pinned SHA-256")
    if len(sidecar_fields) != 2 or sidecar_fields[1] != zero_path.name:
        raise VerificationError("persistent zero sidecar format mismatch")
    if zero_sha256 != sidecar_fields[0].lower():
        raise VerificationError("persistent zero checksum mismatch")
    if (
        zero.get("schema") != "go-m8010-persistent-software-zero/1.0"
        or zero.get("reference_name") != "PERSISTENT_SOFTWARE_ZERO_V1"
    ):
        raise VerificationError("persistent zero schema mismatch")
    if (
        hints.get("schema") != "go-m8010-recovery-branch-hints/1.0"
        or hints.get("reference_name") != "PERSISTENT_SOFTWARE_ZERO_V1"
    ):
        raise VerificationError("recovery hint schema mismatch")
    for name in MOTOR_SIGNS:
        try:
            persistent = float(zero["motors"][name]["raw_position_rad"])
            hint = float(hints["motors"][name]["logical_position_rad"])
        except (KeyError, TypeError, ValueError) as exc:
            raise VerificationError(f"invalid {name} reference configuration") from exc
        if not math.isfinite(persistent) or hint != 0.0:
            raise VerificationError(f"{name} is not configured for the new zero branch")
    return zero, hints, {
        "persistent_zero_path": str(zero_path),
        "persistent_zero_sha256": zero_sha256,
        "persistent_zero_sidecar_path": str(sidecar),
        "recovery_hint_path": str(hint_path),
        "recovery_hint_sha256": hint_sha256,
        "reference_name": "PERSISTENT_SOFTWARE_ZERO_V1",
    }


def _finite_number(value: Any, label: str) -> float:
    if type(value) not in {int, float} or not math.isfinite(float(value)):
        raise VerificationError(f"non-finite {label}")
    return float(value)


def analyze_packets(
    packets: list[dict],
    received_monotonic_ns: list[int],
    source_addresses: list[tuple[str, int]],
    zero: dict,
    hints: dict,
) -> dict:
    if len(packets) < MINIMUM_PACKETS:
        raise VerificationError(
            f"feedback packet count {len(packets)} is below {MINIMUM_PACKETS}"
        )
    if len(received_monotonic_ns) != len(packets) or len(source_addresses) != len(packets):
        raise VerificationError("feedback capture metadata length mismatch")
    if any(type(stamp) is not int or stamp <= 0 for stamp in received_monotonic_ns):
        raise VerificationError("local receive timestamp invalid")
    if any(
        right <= left
        for left, right in zip(received_monotonic_ns, received_monotonic_ns[1:])
    ):
        raise VerificationError("local receive timestamps are not strictly increasing")
    local_coverage_s = (
        received_monotonic_ns[-1] - received_monotonic_ns[0]
    ) / 1.0e9
    if local_coverage_s < MINIMUM_COVERAGE_S:
        raise VerificationError(f"local feedback coverage {local_coverage_s:.3f}s is too short")
    maximum_local_gap_ns = max(
        right - left
        for left, right in zip(received_monotonic_ns, received_monotonic_ns[1:])
    )
    if maximum_local_gap_ns > MAX_FEEDBACK_GAP_NS:
        raise VerificationError(
            f"local feedback gap {maximum_local_gap_ns / 1.0e9:.3f}s is too long"
        )
    first_source = source_addresses[0]
    if (
        first_source[0] != "127.0.0.1"
        or type(first_source[1]) is not int
        or not 1 <= first_source[1] <= 65535
        or any(address != first_source for address in source_addresses)
    ):
        raise VerificationError("feedback UDP source endpoint changed or is invalid")
    stamps: list[int] = []
    feedback_ages_ns: list[int] = []
    q_by_motor = {name: [] for name in MOTOR_SIGNS}
    temperatures = {name: [] for name in MOTOR_SIGNS}
    selected_references: dict[str, float] = {}
    first_unwrapped: dict[str, float] = {}

    for packet_index, packet in enumerate(packets):
        if packet.get("schema") != "go-m8010-motor-feedback/1.0":
            raise VerificationError(f"packet {packet_index} schema mismatch")
        stamp = packet.get("source_monotonic_ns")
        if type(stamp) is not int or stamp <= 0:
            raise VerificationError(f"packet {packet_index} timestamp invalid")
        stamps.append(stamp)
        age_ns = received_monotonic_ns[packet_index] - stamp
        if age_ns < 0 or age_ns > MAX_FEEDBACK_AGE_NS:
            raise VerificationError(
                f"packet {packet_index} source timestamp is stale or future-dated"
            )
        feedback_ages_ns.append(age_ns)
        if packet.get("controller_mode") != "brake":
            raise VerificationError(f"packet {packet_index} left BRAKE")
        modes = packet.get("controller_mode_by_motor")
        if not isinstance(modes, dict) or any(
            modes.get(name) != "brake" for name in MOTOR_SIGNS
        ):
            raise VerificationError(f"packet {packet_index} motor mode was not BRAKE")
        if any(
            packet.get(field) is not False
            for field in ("j2_sync_fault", "domain_fault", "lease_safe_hold")
        ):
            raise VerificationError(f"packet {packet_index} reported a controller fault")
        samples = packet.get("samples")
        if not isinstance(samples, list):
            raise VerificationError(f"packet {packet_index} samples missing")
        by_name = {
            sample.get("motor"): sample
            for sample in samples
            if isinstance(sample, dict) and sample.get("motor") in MOTOR_SIGNS
        }
        if len(samples) != len(MOTOR_SIGNS) or set(by_name) != set(MOTOR_SIGNS):
            raise VerificationError(f"packet {packet_index} J2 sample set mismatch")

        for name, sign in MOTOR_SIGNS.items():
            sample = by_name[name]
            if sample.get("communication_ok") is not True:
                raise VerificationError(f"packet {packet_index} {name} communication failed")
            if sample.get("merror") != 0:
                raise VerificationError(f"packet {packet_index} {name} merror is nonzero")
            if sample.get("recovery_hint_configured") is not True:
                raise VerificationError(f"packet {packet_index} {name} hint is absent")
            hint = _finite_number(
                sample.get("recovery_hint_logical_position_rad"), f"{name} hint"
            )
            configured_hint = float(hints["motors"][name]["logical_position_rad"])
            if hint != 0.0 or configured_hint != 0.0:
                raise VerificationError(f"packet {packet_index} {name} hint is not zero")
            temperature = _finite_number(sample.get("temperature_c"), f"{name} temperature")
            if not 0.0 <= temperature < MAX_TEMPERATURE_C:
                raise VerificationError(f"packet {packet_index} {name} temperature unsafe")
            unwrapped = _finite_number(
                sample.get("unwrapped_raw_position_rad"), f"{name} unwrapped position"
            )
            reference = _finite_number(
                sample.get("software_zero_reference_raw_rad"), f"{name} reference"
            )
            if name not in selected_references:
                selected_references[name] = reference
                first_unwrapped[name] = unwrapped
            elif not math.isclose(
                reference, selected_references[name], rel_tol=0.0, abs_tol=1e-12
            ):
                raise VerificationError(f"packet {packet_index} {name} reference changed")
            q_by_motor[name].append(sign * (unwrapped - reference) / GEAR_RATIO)
            temperatures[name].append(temperature)

    if any(right <= left for left, right in zip(stamps, stamps[1:])):
        raise VerificationError("source timestamps are not strictly increasing")
    coverage_s = (stamps[-1] - stamps[0]) / 1.0e9
    if coverage_s < MINIMUM_COVERAGE_S:
        raise VerificationError(f"feedback coverage {coverage_s:.3f}s is too short")

    for name in MOTOR_SIGNS:
        persistent = float(zero["motors"][name]["raw_position_rad"])
        expected_reference = persistent + round(
            (first_unwrapped[name] - persistent) / (2.0 * math.pi)
        ) * (2.0 * math.pi)
        if not math.isclose(
            selected_references[name], expected_reference,
            rel_tol=0.0, abs_tol=1e-9,
        ):
            raise VerificationError(f"{name} selected the wrong persistent branch")

    common = [
        0.5 * (a + b)
        for a, b in zip(q_by_motor["J2A"], q_by_motor["J2B"])
    ]
    sync = [
        a - b
        for a, b in zip(q_by_motor["J2A"], q_by_motor["J2B"])
    ]
    deg = 180.0 / math.pi
    common_mean_deg = sum(common) / len(common) * deg
    common_span_deg = (max(common) - min(common)) * deg
    sync_max_abs_deg = max(abs(value) for value in sync) * deg
    motor_span_deg = {
        name: (max(values) - min(values)) * deg
        for name, values in q_by_motor.items()
    }
    if abs(common_mean_deg) > MAX_COMMON_ABS_DEG:
        raise VerificationError(f"J2 common zero error {common_mean_deg:.6f} deg")
    if common_span_deg > MAX_POSITION_SPAN_DEG:
        raise VerificationError(f"J2 common position span {common_span_deg:.6f} deg")
    for name, span_deg in motor_span_deg.items():
        if span_deg > MAX_POSITION_SPAN_DEG:
            raise VerificationError(f"{name} position span {span_deg:.6f} deg")
    if sync_max_abs_deg > MAX_SYNC_ABS_DEG:
        raise VerificationError(f"J2 sync error {sync_max_abs_deg:.6f} deg")

    return {
        "packet_count": len(packets),
        "source_coverage_s": coverage_s,
        "local_receive_coverage_s": local_coverage_s,
        "maximum_local_gap_s": maximum_local_gap_ns / 1.0e9,
        "feedback_age_min_s": min(feedback_ages_ns) / 1.0e9,
        "feedback_age_max_s": max(feedback_ages_ns) / 1.0e9,
        "feedback_source_endpoint": f"{first_source[0]}:{first_source[1]}",
        "all_controller_modes": ["brake"],
        "active_or_hold_udp_commands_sent": 0,
        "j2_common_mean_deg": common_mean_deg,
        "j2_common_min_deg": min(common) * deg,
        "j2_common_max_deg": max(common) * deg,
        "j2_common_span_deg": common_span_deg,
        "j2_sync_mean_deg": sum(sync) / len(sync) * deg,
        "j2_sync_max_abs_deg": sync_max_abs_deg,
        "J2A_mean_deg": sum(q_by_motor["J2A"]) / len(packets) * deg,
        "J2B_mean_deg": sum(q_by_motor["J2B"]) / len(packets) * deg,
        "J2A_span_deg": motor_span_deg["J2A"],
        "J2B_span_deg": motor_span_deg["J2B"],
        "selected_reference_raw_rad": selected_references,
        "temperature_min_c": {
            name: min(values) for name, values in temperatures.items()
        },
        "temperature_max_c": {
            name: max(values) for name, values in temperatures.items()
        },
        "merror_nonzero_packets": 0,
        "communication_failure_packets": 0,
    }


def atomic_write_json(path: Path, value: dict) -> None:
    if path.exists():
        raise VerificationError(f"refuse to overwrite evidence: {path}")
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.{os.getpid()}.tmp")
    descriptor = os.open(temporary, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as stream:
            json.dump(value, stream, ensure_ascii=False, indent=2)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def prepare_evidence_destination(path: Path) -> None:
    if path.exists():
        raise VerificationError(f"refuse to overwrite evidence: {path}")
    probe = path.parent / f".{path.name}.{os.getpid()}.write-probe"
    descriptor: int | None = None
    try:
        path.parent.mkdir(parents=True, exist_ok=True)
        descriptor = os.open(probe, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        os.write(descriptor, b"evidence-write-preflight\n")
        os.fsync(descriptor)
    except OSError as exc:
        raise VerificationError(f"evidence destination is not writable: {exc}") from exc
    finally:
        if descriptor is not None:
            os.close(descriptor)
        try:
            probe.unlink()
        except FileNotFoundError:
            pass


def stop_worker(process: subprocess.Popen[str]) -> tuple[str, str, str]:
    escalation = "none"
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
    try:
        stdout, stderr = process.communicate(timeout=5.0)
    except subprocess.TimeoutExpired:
        escalation = "SIGTERM"
        process.terminate()
        try:
            stdout, stderr = process.communicate(timeout=3.0)
        except subprocess.TimeoutExpired:
            escalation = "SIGKILL"
            process.kill()
            stdout, stderr = process.communicate(timeout=2.0)
    return stdout, stderr, escalation


def build_worker_command(args: argparse.Namespace) -> list[str]:
    return [
        str(args.worker),
        "--execute",
        "--brake-only",
        "--confirm",
        WORKER_GATE,
        "--bus",
        "j2",
        "--feedback-port",
        str(args.feedback_port),
        "--thermal-config",
        str(args.thermal_config),
        "--expected-thermal-config-sha256",
        args.expected_thermal_config_sha256,
        "--zero-file",
        str(args.zero_file),
        "--recovery-hint-file",
        str(args.recovery_hint_file),
    ]


def parse_worker_terminal_proof(stdout: str) -> dict[str, str]:
    required = {
        "TERMINAL_PATH": "NORMAL",
        "GUI_GO_CONTROLLER_BUS": "j2",
        "EXECUTION_POLICY": "BRAKE_ONLY",
        "COMMAND_RX_ENABLED": "NO",
        "FOC_TX_ATTEMPT_COUNT": "0",
        "OTHER_MODE_TX_ATTEMPT_COUNT": "0",
        "BRAKE_ONLY_GUARD_BLOCK_COUNT": "0",
        "FOC_SERIAL_SEND_CALL_COUNT": "0",
        "BRAKE_ONLY_AUDIT": "PASS",
        "FINAL_MODE": "BRAKE",
        "FINAL_BRAKE": "PASS",
        "MOTOR_INTERNAL_ZERO_WRITE": "NO",
        "J2_STARTUP_BRAKE_PRIME_STATE": "PASS",
        "J2_STARTUP_BRAKE_PRIME_MAX_INVALID_PREFIX_PAIRS": "3",
        "J2_STARTUP_BRAKE_PRIME_REQUIRED_HEALTHY_PAIRS": "5",
    }
    positive_numeric = {
        "COMPLETED_CYCLES",
        "TX_ATTEMPT_TOTAL",
        "BRAKE_TX_ATTEMPT_COUNT",
        "SERIAL_SEND_CALL_COUNT",
        "J2_STARTUP_BRAKE_PRIME_HEALTHY_PAIRS",
        "J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS",
        "J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT",
        "J2_STARTUP_BRAKE_PRIME_ELAPSED_NS",
        "FINAL_BRAKE_TX_ATTEMPT_COUNT",
    }
    nonnegative_numeric = {"J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS"}
    optional = {"J2_SESSION_REFERENCE_CONFIGURED"}
    recognized = set(required) | positive_numeric | nonnegative_numeric | optional
    proof: dict[str, str] = {}
    for raw_line in stdout.splitlines():
        line = raw_line.strip()
        if "=" not in line:
            continue
        key, value = line.split("=", 1)
        if key not in recognized:
            continue
        if key in proof:
            raise VerificationError(f"duplicate worker terminal proof key: {key}")
        proof[key] = value
    missing = (
        set(required) | positive_numeric | nonnegative_numeric
    ) - set(proof)
    if missing:
        raise VerificationError(
            "worker terminal proof is missing: " + ",".join(sorted(missing))
        )
    for key, expected in required.items():
        if proof[key] != expected:
            raise VerificationError(
                f"worker terminal proof {key}={proof[key]!r}, expected {expected!r}"
            )
    parsed_numeric: dict[str, int] = {}
    for key in positive_numeric | nonnegative_numeric:
        try:
            value = int(proof[key], 10)
        except ValueError as exc:
            raise VerificationError(f"worker terminal proof {key} is not an integer") from exc
        if proof[key] != str(value):
            raise VerificationError(
                f"worker terminal proof {key} is not canonical decimal"
            )
        if key in positive_numeric and value <= 0:
            raise VerificationError(f"worker terminal proof {key} is not positive")
        if key in nonnegative_numeric and value < 0:
            raise VerificationError(
                f"worker terminal proof {key} is not nonnegative"
            )
        parsed_numeric[key] = value
    if not (
        parsed_numeric["TX_ATTEMPT_TOTAL"]
        == parsed_numeric["BRAKE_TX_ATTEMPT_COUNT"]
        == parsed_numeric["SERIAL_SEND_CALL_COUNT"]
    ):
        raise VerificationError("worker BRAKE-only transmit counters disagree")
    invalid_prefix_pairs = parsed_numeric[
        "J2_STARTUP_BRAKE_PRIME_INVALID_PREFIX_PAIRS"
    ]
    healthy_pairs = parsed_numeric["J2_STARTUP_BRAKE_PRIME_HEALTHY_PAIRS"]
    attempted_pairs = parsed_numeric["J2_STARTUP_BRAKE_PRIME_ATTEMPTED_PAIRS"]
    elapsed_ns = parsed_numeric["J2_STARTUP_BRAKE_PRIME_ELAPSED_NS"]
    if invalid_prefix_pairs > 3:
        raise VerificationError(
            "worker J2 startup BRAKE prime invalid prefix is too long"
        )
    if healthy_pairs != 5:
        raise VerificationError(
            "worker J2 startup BRAKE prime healthy qualification is incomplete"
        )
    if attempted_pairs != invalid_prefix_pairs + healthy_pairs:
        raise VerificationError(
            "worker J2 startup BRAKE prime pair counters disagree"
        )
    if parsed_numeric["J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT"] != (
        2 * attempted_pairs
    ):
        raise VerificationError(
            "worker J2 startup BRAKE prime transmit count disagrees"
        )
    if elapsed_ns > 500_000_000:
        raise VerificationError(
            "worker J2 startup BRAKE prime exceeded its time bound"
        )
    if parsed_numeric["FINAL_BRAKE_TX_ATTEMPT_COUNT"] != 40:
        raise VerificationError(
            "worker J2 final BRAKE transmit count is not exactly 40"
        )
    return proof


def validate_worker_cycle_accounting(
    proof: dict[str, str], packet_count: int
) -> dict[str, int]:
    if type(packet_count) is not int or packet_count <= 0:
        raise VerificationError("captured packet count is not positive")
    completed_cycles = int(proof["COMPLETED_CYCLES"], 10)
    if completed_cycles != packet_count:
        raise VerificationError(
            "worker completed-cycle proof does not exactly match captured packets"
        )
    prime_tx = int(proof["J2_STARTUP_BRAKE_PRIME_TX_ATTEMPT_COUNT"], 10)
    final_tx = int(proof["FINAL_BRAKE_TX_ATTEMPT_COUNT"], 10)
    expected_total = prime_tx + 2 * completed_cycles + final_tx
    actual_total = int(proof["TX_ATTEMPT_TOTAL"], 10)
    if actual_total != expected_total:
        raise VerificationError(
            "worker aggregate transmit count does not equal prime + cycles + final BRAKE"
        )
    return {
        "completed_cycles": completed_cycles,
        "startup_prime_tx_attempt_count": prime_tx,
        "control_loop_tx_attempt_count": 2 * completed_cycles,
        "final_brake_tx_attempt_count": final_tx,
        "aggregate_tx_attempt_count": actual_total,
    }


def run(args: argparse.Namespace) -> int:
    if args.confirm != CONFIRM_GATE:
        raise VerificationError(f"require --confirm {CONFIRM_GATE}")
    worker_sha = sha256_file(args.worker)
    expected_worker_sha = _normalized_sha256(
        args.expected_worker_sha256, "expected worker"
    )
    if worker_sha != expected_worker_sha:
        raise VerificationError(f"worker SHA-256 mismatch: {worker_sha}")
    zero, hints, configuration = load_configuration(
        args.zero_file,
        args.recovery_hint_file,
        args.expected_zero_sha256,
        args.expected_hint_sha256,
    )
    prepare_evidence_destination(args.output)

    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 0)
    listener.bind(("127.0.0.1", args.feedback_port))
    listener.settimeout(0.20)
    command = build_worker_command(args)
    process: subprocess.Popen[str] | None = None
    packets: list[dict] = []
    received_monotonic_ns: list[int] = []
    source_addresses: list[tuple[str, int]] = []
    collection_error: BaseException | None = None
    started_at = time.monotonic()
    stdout = stderr = ""
    stop_escalation = "none"
    try:
        process = subprocess.Popen(
            command,
            stdin=subprocess.DEVNULL,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
        )
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
                raise VerificationError("feedback arrived from a non-loopback source")
            try:
                document = json.loads(payload)
            except json.JSONDecodeError as exc:
                raise VerificationError("feedback JSON decode failed") from exc
            if not isinstance(document, dict):
                raise VerificationError("feedback packet is not a JSON object")
            packets.append(document)
            received_monotonic_ns.append(received_at_ns)
            source_addresses.append((address[0], int(address[1])))
    except BaseException as exc:
        collection_error = exc
    finally:
        listener.close()
        if process is not None:
            stdout, stderr, stop_escalation = stop_worker(process)

    result: dict[str, Any] = {
        "schema": "go-m8010-j2-brake-feedback-verification/1.0",
        "status": "FAIL",
        "worker": {
            "path": str(args.worker),
            "sha256": worker_sha,
            "returncode": None if process is None else process.returncode,
            "stop_escalation": stop_escalation,
            "forced_kill": stop_escalation == "SIGKILL",
            "stdout": stdout.splitlines(),
            "stderr": stderr.splitlines(),
        },
        "configuration": configuration,
        "safety": {
            "udp_listener_bind": f"127.0.0.1:{args.feedback_port}",
            "udp_send_calls": 0,
            "worker_default_mode": "BRAKE",
            "worker_execution_policy_requested": "BRAKE_ONLY",
            "foc_or_position_command_source_started": False,
            "motor_internal_zero_modified": False,
        },
        "physical_power_off_required": True,
    }
    failures = []
    if collection_error is not None:
        failures.append(f"{type(collection_error).__name__}:{collection_error}")
    if process is None or process.returncode != 0:
        failures.append("worker did not exit successfully")
    if stop_escalation != "none":
        failures.append(
            f"worker stop escalated to {stop_escalation}; physical power-off is required"
        )
    terminal_proof = None
    try:
        terminal_proof = parse_worker_terminal_proof(stdout)
        result["worker"]["terminal_proof"] = terminal_proof
    except VerificationError as exc:
        failures.append(str(exc))
    statistics = None
    try:
        statistics = analyze_packets(
            packets,
            received_monotonic_ns,
            source_addresses,
            zero,
            hints,
        )
        result["statistics"] = statistics
    except VerificationError as exc:
        failures.append(str(exc))
    if terminal_proof is not None and statistics is not None:
        try:
            result["worker"]["phase_tx_accounting"] = (
                validate_worker_cycle_accounting(
                    terminal_proof, statistics["packet_count"]
                )
            )
        except VerificationError as exc:
            failures.append(str(exc))
    try:
        post_zero_sha256 = sha256_file(args.zero_file)
        post_hint_sha256 = sha256_file(args.recovery_hint_file)
    except OSError as exc:
        failures.append(f"configuration post-run hash failed: {exc}")
    else:
        result["configuration"]["post_run_persistent_zero_sha256"] = post_zero_sha256
        result["configuration"]["post_run_recovery_hint_sha256"] = post_hint_sha256
        if post_zero_sha256 != configuration["persistent_zero_sha256"]:
            failures.append("persistent zero changed during verification")
        if post_hint_sha256 != configuration["recovery_hint_sha256"]:
            failures.append("recovery hint changed during verification")
    result["failures"] = failures
    if not failures:
        result["status"] = "PASS"
        result["physical_power_off_required"] = False
    atomic_write_json(args.output, result)
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0 if result["status"] == "PASS" else 2


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", type=Path, required=True)
    parser.add_argument("--expected-worker-sha256", required=True)
    parser.add_argument("--thermal-config", type=Path, required=True)
    parser.add_argument("--expected-thermal-config-sha256", required=True)
    parser.add_argument("--zero-file", type=Path, required=True)
    parser.add_argument("--expected-zero-sha256", required=True)
    parser.add_argument("--recovery-hint-file", type=Path, required=True)
    parser.add_argument("--expected-hint-sha256", required=True)
    parser.add_argument("--feedback-port", type=int, default=15300)
    parser.add_argument("--target-packets", type=int, default=TARGET_PACKETS)
    parser.add_argument("--maximum-runtime-s", type=float, default=8.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--confirm", required=True)
    args = parser.parse_args()
    if not 1024 <= args.feedback_port <= 65535:
        parser.error("feedback port out of range")
    if args.target_packets < MINIMUM_PACKETS:
        parser.error(f"target packets must be >= {MINIMUM_PACKETS}")
    if not 5.0 <= args.maximum_runtime_s <= 15.0:
        parser.error("maximum runtime must be in [5, 15] seconds")
    return args


def main() -> int:
    try:
        return run(parse_args())
    except VerificationError as exc:
        print(f"J2_BRAKE_FEEDBACK_VERIFY=BLOCKED\nREASON={exc}", flush=True)
        return 4


if __name__ == "__main__":
    raise SystemExit(main())

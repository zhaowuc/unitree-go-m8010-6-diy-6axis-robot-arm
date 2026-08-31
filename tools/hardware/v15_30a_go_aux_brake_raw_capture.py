#!/usr/bin/env python3
"""Capture J1 and J3/J4/J5 raw phase under concurrent BRAKE-only workers.

No persistent zero, recovery hint, command socket, HOLD, FOC, or active target is
used.  The physical buses use distinct loopback feedback ports and must
independently provide at least 500 stationary BRAKE frames over at least four
seconds.  The output is one immutable evidence file consumed by the GO-AUX
session-anchor issuer.
"""

from __future__ import annotations

import argparse
import concurrent.futures
import hashlib
import importlib.util
import json
import math
import socket
import subprocess
import time
import uuid
from datetime import datetime, timezone
from pathlib import Path
from typing import Any


_RAW_PATH = Path(__file__).with_name("v15_30a_j2_brake_raw_capture.py")
_SPEC = importlib.util.spec_from_file_location("v15_30a_j2_raw_base", _RAW_PATH)
assert _SPEC is not None and _SPEC.loader is not None
raw_base = importlib.util.module_from_spec(_SPEC)
_SPEC.loader.exec_module(raw_base)


CAPTURE_SCHEMA = "go-m8010-go-aux-brake-raw-capture-statistics/1.0"
RESULT_SCHEMA = "go-m8010-go-aux-brake-raw-capture-result/1.0"
CONFIRM_GATE = "V15_30A_GO_AUX_BRAKE_RAW_CAPTURE=YES"
PHYSICAL_GATE = (
    "24V_ON=YES;SUPPORT_RELIABLE=YES;WHOLE_ARM_VERTICAL_INITIALIZATION_POSE=YES;"
    "ARM_NOT_MOVED=YES;ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES"
)
BUS_MOTORS = {"j1": ("J1",), "j345": ("J3", "J4", "J5")}
TARGET_PACKETS = 500
MIN_COVERAGE_S = 4.0
DIAGNOSTIC_GAP_NS = 100_000_000
MAX_GAP_NS = raw_base.MAXIMUM_FEEDBACK_GAP_NS
MAX_FEEDBACK_AGE_NS = raw_base.MAXIMUM_FEEDBACK_AGE_NS
MAX_TEMPERATURE_C = 60.0
GEAR_RATIO = 6.329999923706055
MAX_RAW_SPAN_RAD = GEAR_RATIO * math.radians(0.20)
RUNTIME_HARDWARE_ACCESSED = False


class CaptureError(RuntimeError):
    pass


class DomainWorkerError(CaptureError):
    """Preserve fail-closed worker evidence for one physical GO bus."""

    def __init__(self, reason: str, diagnostic: dict[str, Any]) -> None:
        super().__init__(reason)
        self.diagnostic = diagnostic


def _finite(value: Any, label: str) -> float:
    if type(value) not in {int, float} or not math.isfinite(float(value)):
        raise CaptureError(f"{label} is not finite")
    return float(value)


def _stats(values: list[float], name: str) -> dict[str, float]:
    if len(values) < TARGET_PACKETS:
        raise CaptureError(f"{name} has too few raw samples")
    mean = math.fsum(values) / len(values)
    minimum = min(values)
    maximum = max(values)
    span = maximum - minimum
    deviation = math.sqrt(
        max(math.fsum((value - mean) ** 2 for value in values) / len(values), 0.0)
    )
    if span > MAX_RAW_SPAN_RAD:
        raise CaptureError(f"{name} raw span is not stationary")
    if any(
        abs(value) > raw_base.MAXIMUM_RAW_POSITION_RAD
        for value in (mean, minimum, maximum)
    ):
        raise CaptureError(f"{name} raw position exceeds protocol range")
    return {
        "mean": mean,
        "minimum": minimum,
        "maximum": maximum,
        "span": span,
        "standard_deviation": deviation,
    }


def _maximum_gap(stamps: list[int]) -> tuple[int, int]:
    if len(stamps) < 2:
        return 0, -1
    gaps = [right - left for left, right in zip(stamps, stamps[1:])]
    maximum = max(gaps)
    return maximum, gaps.index(maximum) + 1


def _timing_summary(
    receive_ns: list[int], source_stamps: list[int]
) -> dict[str, Any]:
    maximum_local_gap_ns, local_right_index = _maximum_gap(receive_ns)
    maximum_source_gap_ns, source_right_index = _maximum_gap(source_stamps)
    local_gaps = [
        right - left for left, right in zip(receive_ns, receive_ns[1:])
    ]
    source_gaps = [
        right - left for left, right in zip(source_stamps, source_stamps[1:])
    ]
    feedback_ages = [
        received - source
        for received, source in zip(receive_ns, source_stamps)
    ]
    maximum_feedback_age_ns = max(feedback_ages)
    local_warning_count = sum(gap > DIAGNOSTIC_GAP_NS for gap in local_gaps)
    source_warning_count = sum(gap > DIAGNOSTIC_GAP_NS for gap in source_gaps)
    return {
        "diagnostic_gap_threshold_ns": DIAGNOSTIC_GAP_NS,
        "hard_gap_limit_ns": MAX_GAP_NS,
        "hard_feedback_age_limit_ns": MAX_FEEDBACK_AGE_NS,
        "maximum_local_gap_ns": maximum_local_gap_ns,
        "maximum_local_gap_s": maximum_local_gap_ns / 1.0e9,
        "maximum_local_gap_right_packet_index": local_right_index,
        "local_gap_over_diagnostic_count": local_warning_count,
        "maximum_source_gap_ns": maximum_source_gap_ns,
        "maximum_source_gap_s": maximum_source_gap_ns / 1.0e9,
        "maximum_source_gap_right_packet_index": source_right_index,
        "source_gap_over_diagnostic_count": source_warning_count,
        "feedback_age_min_ns": min(feedback_ages),
        "feedback_age_max_ns": maximum_feedback_age_ns,
        "feedback_age_max_packet_index": feedback_ages.index(
            maximum_feedback_age_ns
        ),
        "feedback_age_min_s": min(feedback_ages) / 1.0e9,
        "feedback_age_max_s": max(feedback_ages) / 1.0e9,
        "collector_scheduler_jitter_candidate": bool(
            maximum_local_gap_ns > DIAGNOSTIC_GAP_NS
            and maximum_source_gap_ns <= DIAGNOSTIC_GAP_NS
        ),
        "source_stall_candidate": bool(
            maximum_source_gap_ns > DIAGNOSTIC_GAP_NS
        ),
    }


def _health_diagnostic(
    bus: str, packets: list[Any]
) -> dict[str, Any]:
    names = BUS_MOTORS[bus]
    unhealthy: list[dict[str, Any]] = []
    unhealthy_count = 0
    for index, packet in enumerate(packets):
        failed_motors = []
        issues = []
        if not isinstance(packet, dict):
            unhealthy_count += 1
            if len(unhealthy) < 20:
                unhealthy.append(
                    {
                        "packet_index": index,
                        "failed_motors": [],
                        "issues": ["PACKET_STRUCTURE"],
                        "controller_mode": None,
                        "domain_fault": None,
                        "j2_sync_fault": None,
                        "lease_safe_hold": None,
                    }
                )
            continue
        if packet.get("schema") != "go-m8010-motor-feedback/1.0":
            issues.append("SCHEMA")
        if packet.get("controller_mode") != "brake":
            issues.append("CONTROLLER_MODE")
        modes = packet.get("controller_mode_by_motor")
        if not isinstance(modes, dict) or set(modes) != set(names) or any(
            modes.get(name) != "brake" for name in names
        ):
            issues.append("MOTOR_MODE")
        samples = packet.get("samples")
        if not isinstance(samples, list) or any(
            not isinstance(sample, dict) for sample in samples
        ):
            issues.append("SAMPLE_STRUCTURE")
        else:
            sample_names = tuple(sample.get("motor") for sample in samples)
            if sample_names != names:
                issues.append("SAMPLE_SET_OR_ORDER")
            for sample in samples:
                name = sample.get("motor")
                if name in names and (
                    sample.get("communication_ok") is not True
                    or type(sample.get("merror")) is not int
                    or sample.get("merror") != 0
                ):
                    failed_motors.append(name)
                if name in names:
                    temperature = sample.get("temperature_c")
                    raw = sample.get("unwrapped_raw_position_rad")
                    if (
                        type(temperature) not in {int, float}
                        or not math.isfinite(float(temperature))
                        or not 0.0 <= float(temperature) < MAX_TEMPERATURE_C
                    ):
                        issues.append(f"{name}_TEMPERATURE")
                    if (
                        type(raw) not in {int, float}
                        or not math.isfinite(float(raw))
                    ):
                        issues.append(f"{name}_RAW_POSITION")
        controller_fault = any(
            packet.get(field) is not False
            for field in ("j2_sync_fault", "domain_fault", "lease_safe_hold")
        )
        if controller_fault:
            issues.append("CONTROLLER_FAULT")
        if failed_motors:
            issues.append("MOTOR_COMMUNICATION_OR_MERROR")
        if issues:
            unhealthy_count += 1
            if len(unhealthy) < 20:
                unhealthy.append(
                    {
                        "packet_index": index,
                        "failed_motors": failed_motors,
                        "issues": sorted(set(issues)),
                        "controller_mode": packet.get("controller_mode"),
                        "domain_fault": packet.get("domain_fault"),
                        "j2_sync_fault": packet.get("j2_sync_fault"),
                        "lease_safe_hold": packet.get("lease_safe_hold"),
                    }
                )
    return {
        "unhealthy_packet_count": unhealthy_count,
        "first_unhealthy_packets": unhealthy,
    }


def _domain_validation_diagnostic(
    *, bus: str, packets: list[Any], receive_ns: list[int],
    target_packets: int, process: subprocess.Popen[str], escalation: str,
    stdout: str, stderr: str, terminal_proof: dict[str, str] | None,
    reason: str,
) -> dict[str, Any]:
    diagnostic: dict[str, Any] = {
        "bus": bus,
        "reason": reason,
        "packet_count_before_stop": len(packets),
        "target_packets": target_packets,
        "worker_returncode": process.returncode,
        "stop_escalation": escalation,
        "forced_kill": escalation == "SIGKILL",
        "terminal_proof": terminal_proof,
        "stdout": stdout.splitlines(),
        "stderr": stderr.splitlines(),
        "packet_health": _health_diagnostic(bus, packets),
    }
    source_stamps = [
        packet.get("source_monotonic_ns")
        if isinstance(packet, dict) else None
        for packet in packets
    ]
    if (
        len(receive_ns) == len(packets)
        and len(packets) >= 2
        and all(type(value) is int for value in receive_ns)
        and all(type(value) is int for value in source_stamps)
    ):
        diagnostic["timing"] = _timing_summary(receive_ns, source_stamps)
    return diagnostic


def analyze_domain_packets(
    *, bus: str, packets: list[Any], receive_ns: list[int],
    endpoints: list[tuple[str, int]], target_packets: int,
) -> dict[str, Any]:
    names = BUS_MOTORS[bus]
    if len(packets) < target_packets:
        raise CaptureError(f"{bus} captured only {len(packets)} packets")
    if len(receive_ns) != len(packets) or len(endpoints) != len(packets):
        raise CaptureError(f"{bus} capture metadata length mismatch")
    if any(type(stamp) is not int or stamp <= 0 for stamp in receive_ns):
        raise CaptureError(f"{bus} local receive timestamp is invalid")
    if any(right <= left for left, right in zip(receive_ns, receive_ns[1:])):
        raise CaptureError(f"{bus} receive timestamps are not increasing")
    if (
        len(set(endpoints)) != 1
        or endpoints[0][0] != "127.0.0.1"
        or type(endpoints[0][1]) is not int
        or not 1 <= endpoints[0][1] <= 65535
    ):
        raise CaptureError(f"{bus} feedback source endpoint changed")

    local_coverage = (receive_ns[-1] - receive_ns[0]) / 1.0e9
    source_stamps: list[int] = []
    raw: dict[str, list[float]] = {name: [] for name in names}
    temperatures: dict[str, list[float]] = {name: [] for name in names}
    for index, (packet, received) in enumerate(zip(packets, receive_ns)):
        if not isinstance(packet, dict):
            raise CaptureError(f"{bus} packet {index} is not an object")
        if packet.get("schema") != "go-m8010-motor-feedback/1.0":
            raise CaptureError(f"{bus} packet {index} schema mismatch")
        stamp = packet.get("source_monotonic_ns")
        if type(stamp) is not int or stamp <= 0:
            raise CaptureError(f"{bus} packet {index} timestamp is invalid")
        age_ns = received - stamp
        if age_ns < 0:
            raise CaptureError(f"{bus} packet {index} timestamp is future-dated")
        source_stamps.append(stamp)
        if packet.get("controller_mode") != "brake" or any(
            packet.get(field) is not False
            for field in ("j2_sync_fault", "domain_fault", "lease_safe_hold")
        ):
            raise CaptureError(f"{bus} packet {index} left safe BRAKE state")
        modes = packet.get("controller_mode_by_motor")
        if not isinstance(modes, dict) or set(modes) != set(names) or any(
            modes.get(name) != "brake" for name in names
        ):
            raise CaptureError(f"{bus} packet {index} motor mode mismatch")
        samples = packet.get("samples")
        if not isinstance(samples, list) or any(
            not isinstance(sample, dict) for sample in samples
        ):
            raise CaptureError(f"{bus} packet {index} samples are invalid")
        sample_names = tuple(
            sample.get("motor") for sample in samples
        )
        if sample_names != names:
            raise CaptureError(f"{bus} packet {index} motor set/order mismatch")
        for sample in samples:
            name = sample["motor"]
            if sample.get("communication_ok") is not True:
                raise CaptureError(f"{bus} packet {index} {name} communication failed")
            if type(sample.get("merror")) is not int or sample.get("merror") != 0:
                raise CaptureError(f"{bus} packet {index} {name} merror is nonzero")
            temperature = _finite(sample.get("temperature_c"), f"{name} temperature")
            if not 0.0 <= temperature < MAX_TEMPERATURE_C:
                raise CaptureError(f"{name} temperature is unsafe")
            raw[name].append(
                _finite(sample.get("unwrapped_raw_position_rad"), f"{name} raw")
            )
            temperatures[name].append(temperature)

    if local_coverage < MIN_COVERAGE_S:
        raise CaptureError(f"{bus} local coverage is too short")
    if any(right <= left for left, right in zip(source_stamps, source_stamps[1:])):
        raise CaptureError(f"{bus} source timestamps are not increasing")
    source_coverage = (source_stamps[-1] - source_stamps[0]) / 1.0e9
    if source_coverage < MIN_COVERAGE_S:
        raise CaptureError(f"{bus} source coverage is too short")

    timing = _timing_summary(receive_ns, source_stamps)
    if timing["feedback_age_max_ns"] > MAX_FEEDBACK_AGE_NS:
        raise CaptureError(
            f"{bus} feedback age {timing['feedback_age_max_ns'] / 1.0e6:.3f} ms "
            f"exceeds {MAX_FEEDBACK_AGE_NS / 1.0e6:.3f} ms"
        )
    if timing["maximum_local_gap_ns"] > MAX_GAP_NS:
        raise CaptureError(
            f"{bus} local feedback gap "
            f"{timing['maximum_local_gap_ns'] / 1.0e6:.3f} ms after packet "
            f"{timing['maximum_local_gap_right_packet_index'] - 1} exceeds "
            f"{MAX_GAP_NS / 1.0e6:.3f} ms"
        )
    if timing["maximum_source_gap_ns"] > MAX_GAP_NS:
        raise CaptureError(
            f"{bus} source feedback gap "
            f"{timing['maximum_source_gap_ns'] / 1.0e6:.3f} ms after packet "
            f"{timing['maximum_source_gap_right_packet_index'] - 1} exceeds "
            f"{MAX_GAP_NS / 1.0e6:.3f} ms"
        )

    return {
        "packet_count": len(packets),
        "source_coverage_s": source_coverage,
        "local_receive_coverage_s": local_coverage,
        "motor_names": list(names),
        "motors": {
            name: {
                "sample_count": len(raw[name]),
                "unwrapped_raw_position_rad": _stats(raw[name], name),
            }
            for name in names
        },
        "temperature_min_c": {name: min(temperatures[name]) for name in names},
        "temperature_max_c": {name: max(temperatures[name]) for name in names},
        "feedback_source_endpoint": f"{endpoints[0][0]}:{endpoints[0][1]}",
        "timing": timing,
    }


def _parse_terminal(stdout: str, bus: str) -> dict[str, str]:
    required = {
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
    }
    numeric = {"COMPLETED_CYCLES", "TX_ATTEMPT_TOTAL", "BRAKE_TX_ATTEMPT_COUNT", "SERIAL_SEND_CALL_COUNT"}
    proof: dict[str, str] = {}
    for line in stdout.splitlines():
        if "=" not in line:
            continue
        key, value = line.strip().split("=", 1)
        if key in required or key in numeric:
            if key in proof:
                raise CaptureError(f"{bus} duplicate terminal field {key}")
            proof[key] = value
    missing = (set(required) | numeric) - set(proof)
    if missing:
        raise CaptureError(f"{bus} terminal proof missing {sorted(missing)}")
    for key, expected in required.items():
        if proof[key] != expected:
            raise CaptureError(f"{bus} terminal {key} mismatch")
    for key in numeric:
        if not proof[key].isdigit() or int(proof[key]) < 0:
            raise CaptureError(f"{bus} terminal {key} is invalid")
    return proof


def _capture_domain(
    *, worker: Path, feedback_port: int, bus: str, target_packets: int,
    maximum_runtime_s: float, thermal_config: Path,
    expected_thermal_config_sha256: str,
) -> dict[str, Any]:
    global RUNTIME_HARDWARE_ACCESSED
    names = BUS_MOTORS[bus]
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 0)
    listener.bind(("127.0.0.1", feedback_port))
    listener.settimeout(0.20)
    command = [
        str(worker), "--execute", "--brake-only", "--confirm",
        raw_base.brake_verify.WORKER_GATE, "--bus", bus,
        "--feedback-port", str(feedback_port),
        "--thermal-config", str(thermal_config),
        "--expected-thermal-config-sha256", expected_thermal_config_sha256,
    ]
    process: subprocess.Popen[str] | None = None
    packets: list[Any] = []
    receive_ns: list[int] = []
    endpoints: list[tuple[str, int]] = []
    stdout = stderr = ""
    escalation = "none"
    capture_failure: Exception | None = None
    try:
        process = subprocess.Popen(
            command, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, text=True,
        )
        RUNTIME_HARDWARE_ACCESSED = True
        deadline = time.monotonic() + maximum_runtime_s
        while len(packets) < target_packets and time.monotonic() < deadline:
            if process.poll() is not None:
                break
            try:
                payload, endpoint = listener.recvfrom(65535)
            except socket.timeout:
                continue
            received = time.monotonic_ns()
            document = json.loads(payload)
            packets.append(document)
            receive_ns.append(received)
            endpoints.append((endpoint[0], int(endpoint[1])))
            if not isinstance(document, dict):
                raise CaptureError(f"{bus} feedback is not an object")
            samples = document.get("samples")
            if not isinstance(samples, list) or any(
                not isinstance(sample, dict) for sample in samples
            ):
                raise CaptureError(f"{bus} feedback samples are invalid")
            sample_names = tuple(
                sample.get("motor") for sample in samples
            )
            if sample_names != names:
                raise CaptureError(f"{bus} feedback motor set/order mismatch")
    except Exception as exc:
        capture_failure = exc
    finally:
        listener.close()
        if process is not None:
            stdout, stderr, escalation = raw_base.brake_verify.stop_worker(process)
    proof: dict[str, str] | None = None
    if process is not None:
        try:
            proof = _parse_terminal(stdout, bus)
        except CaptureError:
            pass
    if capture_failure is not None:
        reason = f"{bus} feedback collection failed: {capture_failure}"
        if process is None:
            diagnostic = {
                "bus": bus,
                "reason": reason,
                "packet_count_before_stop": len(packets),
                "target_packets": target_packets,
                "worker_returncode": None,
                "stop_escalation": escalation,
                "forced_kill": escalation == "SIGKILL",
                "terminal_proof": None,
                "stdout": stdout.splitlines(),
                "stderr": stderr.splitlines(),
                "packet_health": _health_diagnostic(bus, packets),
            }
        else:
            diagnostic = _domain_validation_diagnostic(
                bus=bus,
                packets=packets,
                receive_ns=receive_ns,
                target_packets=target_packets,
                process=process,
                escalation=escalation,
                stdout=stdout,
                stderr=stderr,
                terminal_proof=proof,
                reason=reason,
            )
        raise DomainWorkerError(reason, diagnostic) from capture_failure
    if process is None or process.returncode != 0 or escalation != "none":
        reasons = []
        if process is None:
            reasons.append("worker_not_started")
        elif process.returncode != 0:
            reasons.append(f"returncode={process.returncode}")
        if escalation != "none":
            reasons.append(f"stop_escalation={escalation}")
        reason = f"{bus} BRAKE worker did not stop normally ({', '.join(reasons)})"
        if process is None:
            diagnostic = {
                "bus": bus,
                "reason": reason,
                "packet_count_before_stop": len(packets),
                "target_packets": target_packets,
                "worker_returncode": None,
                "stop_escalation": escalation,
                "forced_kill": escalation == "SIGKILL",
                "terminal_proof": None,
                "stdout": stdout.splitlines(),
                "stderr": stderr.splitlines(),
                "packet_health": _health_diagnostic(bus, packets),
            }
        else:
            diagnostic = _domain_validation_diagnostic(
                bus=bus,
                packets=packets,
                receive_ns=receive_ns,
                target_packets=target_packets,
                process=process,
                escalation=escalation,
                stdout=stdout,
                stderr=stderr,
                terminal_proof=proof,
                reason=reason,
            )
        raise DomainWorkerError(reason, diagnostic)
    try:
        if proof is None:
            proof = _parse_terminal(stdout, bus)
        result = analyze_domain_packets(
            bus=bus,
            packets=packets,
            receive_ns=receive_ns,
            endpoints=endpoints,
            target_packets=target_packets,
        )
    except CaptureError as exc:
        raise DomainWorkerError(
            str(exc),
            _domain_validation_diagnostic(
                bus=bus,
                packets=packets,
                receive_ns=receive_ns,
                target_packets=target_packets,
                process=process,
                escalation=escalation,
                stdout=stdout,
                stderr=stderr,
                terminal_proof=proof,
                reason=str(exc),
            ),
        ) from exc
    result.update(
        {
            "terminal_proof": proof,
            "stdout": stdout.splitlines(),
            "stderr": stderr.splitlines(),
        }
    )
    return result


def run(args: argparse.Namespace) -> int:
    pose_binding_id = raw_base.validate_pose_binding_id(args.pose_binding_id)
    if args.confirm != CONFIRM_GATE or args.physical_confirmation != PHYSICAL_GATE:
        raise CaptureError("complete apply/physical gates are required")
    worker = args.worker.resolve(strict=True)
    worker_sha = hashlib.sha256(worker.read_bytes()).hexdigest()
    expected_sha = raw_base.brake_verify._normalized_sha256(
        args.expected_worker_sha256, "worker"
    )
    if worker_sha != expected_sha:
        raise CaptureError("worker SHA-256 mismatch")
    raw_base.brake_verify.prepare_evidence_destination(args.output.resolve())
    instant = datetime.now(timezone.utc)
    token = uuid.uuid4().hex
    boot_id = raw_base.read_host_boot_id()
    recorded_boot_ns = raw_base.read_recorded_boottime_ns()
    domains = {}
    motors = {}
    port_by_bus = {
        "j1": int(getattr(args, "j1_feedback_port", None) or args.feedback_port),
        "j345": int(
            getattr(args, "j345_feedback_port", None) or (args.feedback_port + 1)
        ),
    }
    if len(set(port_by_bus.values())) != 2 or any(
        not 1024 <= port <= 65535 for port in port_by_bus.values()
    ):
        raise CaptureError("J1/J345 feedback ports must be distinct valid ports")
    with concurrent.futures.ThreadPoolExecutor(
        max_workers=2, thread_name_prefix="go-aux-brake-capture"
    ) as executor:
        futures = {
            bus: executor.submit(
                _capture_domain,
                worker=worker,
                feedback_port=port_by_bus[bus],
                bus=bus,
                target_packets=args.target_packets,
                maximum_runtime_s=args.maximum_runtime_s,
                thermal_config=args.thermal_config,
                expected_thermal_config_sha256=args.expected_thermal_config_sha256,
            )
            for bus in ("j1", "j345")
        }
        for bus in ("j1", "j345"):
            domains[bus] = futures[bus].result()
            motors.update(domains[bus]["motors"])
    timestamp = instant.strftime("%Y%m%dT%H%M%S%fZ")
    document = {
        "schema": CAPTURE_SCHEMA,
        "pose_binding_id": pose_binding_id,
        "capture_id": f"go-aux-brake-raw-{timestamp}-{token[:16]}",
        "power_session_id": f"go-aux-power-session-{timestamp}-{token}",
        "recorded_at_utc": instant.isoformat().replace("+00:00", "Z"),
        "host_boot_id": boot_id,
        "recorded_boottime_ns": recorded_boot_ns,
        "status": "PASS",
        "hardware_accessed": True,
        "physical_power_state_during_capture": "24V_ON",
        "worker": {"path": str(worker), "sha256": worker_sha},
        "domains": {
            bus: {
                key: value for key, value in domains[bus].items()
                if key not in {"motors", "stdout", "stderr"}
            }
            for bus in domains
        },
        "motors": motors,
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
            "confirmation_gate": PHYSICAL_GATE,
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
            "domain_worker_stdout": {
                bus: domains[bus]["stdout"] for bus in domains
            },
            "domain_worker_stderr": {
                bus: domains[bus]["stderr"] for bus in domains
            },
        },
        "physical_power_off_required": False,
    }
    raw_base.publish_json_no_overwrite(args.output.resolve(), document)
    print(json.dumps(document, ensure_ascii=False, indent=2), flush=True)
    return 0


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--worker", type=Path, required=True)
    parser.add_argument("--expected-worker-sha256", required=True)
    parser.add_argument("--thermal-config", type=Path, required=True)
    parser.add_argument("--expected-thermal-config-sha256", required=True)
    parser.add_argument("--feedback-port", type=int, default=15300)
    parser.add_argument("--j1-feedback-port", type=int)
    parser.add_argument("--j345-feedback-port", type=int)
    parser.add_argument("--target-packets", type=int, default=TARGET_PACKETS)
    parser.add_argument("--maximum-runtime-s", type=float, default=8.0)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--confirm", required=True)
    parser.add_argument("--physical-confirmation", required=True)
    parser.add_argument(
        "--pose-binding-id",
        type=raw_base.validate_pose_binding_id,
        required=True,
    )
    args = parser.parse_args(argv)
    if not 1024 <= args.feedback_port <= 65535:
        parser.error("feedback port out of range")
    selected_ports = (
        args.j1_feedback_port or args.feedback_port,
        args.j345_feedback_port or (args.feedback_port + 1),
    )
    if any(not 1024 <= port <= 65535 for port in selected_ports):
        parser.error("J1/J345 feedback port out of range")
    if selected_ports[0] == selected_ports[1]:
        parser.error("J1/J345 feedback ports must be distinct")
    if not 500 <= args.target_packets <= 1500:
        parser.error("target packets must be in [500, 1500]")
    if not 5.0 <= args.maximum_runtime_s <= 15.0:
        parser.error("maximum runtime must be in [5, 15] seconds")
    return args


def main() -> int:
    args: argparse.Namespace | None = None
    try:
        args = parse_args()
        return run(args)
    except Exception as exc:
        document: dict[str, Any] = {
            "schema": RESULT_SCHEMA,
            "status": "BLOCKED",
            "reason": str(exc),
            "hardware_accessed": RUNTIME_HARDWARE_ACCESSED,
            "active_control_authorized": False,
            "physical_power_off_required": RUNTIME_HARDWARE_ACCESSED,
        }
        if isinstance(exc, DomainWorkerError):
            document["domain_worker_diagnostic"] = exc.diagnostic
        if args is not None:
            pose_binding_id = getattr(args, "pose_binding_id", None)
            try:
                document["pose_binding_id"] = raw_base.validate_pose_binding_id(
                    pose_binding_id
                )
            except ValueError:
                pass
        print(json.dumps(document, ensure_ascii=False, indent=2), flush=True)
        return 4


if __name__ == "__main__":
    raise SystemExit(main())

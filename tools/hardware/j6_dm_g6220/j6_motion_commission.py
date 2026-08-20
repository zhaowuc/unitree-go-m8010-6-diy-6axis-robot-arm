#!/usr/bin/env python3
"""Operator-gated V15.21B J6 MIT small-motion commissioning.

Phases are intentionally separate:

``prepare`` is strictly read-only and captures the session reference.
``sign-probe`` requires both motion gates, performs HOLD and one +3 degree
protocol-positive round trip, then disables and stops for operator review.
``bidirectional`` additionally requires the operator's ROS-direction answer,
rechecks the complete identity, and performs the one authorized +/-5 degree
route.

No phase implements parameter writes, mode switching, ID changes, set-zero,
POS_VEL, VEL, gain tuning, automatic retry, or motion beyond the frozen route.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import platform
import statistics
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Optional, Sequence

from dm_g6220_motion_transport import DmG6220MitMotionTransport
from dm_g6220_transport import (
    CAN_BITRATE,
    CTRL_MODE_MIT,
    DmCanSdkReadOnlyTransport,
    Feedback,
    TransportError,
    state_text,
)
from j6_readonly_probe import discover_one_motor


SOURCE_HEAD = "b9a604eaee7d02837db9234e30fe1ccd434a6f15"
WORK_BRANCH = "agent/v15-21b-j6-dmg6220-motion"
EXPECTED_USB_VID = "34b7"
EXPECTED_USB_PID = "6877"
EXPECTED_USB_SERIAL = "EEE8D71AB573449FCAFE7B39BD222C75"
EXPECTED_MOTOR_ID = 1
EXPECTED_CAN_CHANNEL = 0
EXPECTED_CTRL_MODE = CTRL_MODE_MIT

CLEARANCE_GATE = "J6_PHYSICAL_CLEARANCE_PLUS_MINUS_10DEG=YES"
EMERGENCY_GATE = "J6_EMERGENCY_DISABLE_READY=YES"
DIRECTION_YES = "J6_PROTOCOL_POSITIVE_IS_ROS_POSITIVE=YES"
DIRECTION_NO = "J6_PROTOCOL_POSITIVE_IS_ROS_POSITIVE=NO"
SIGN_OBSERVATION_GATE = "J6_SIGN_PROBE_OBSERVATION_SAFE=YES"

KP = 10.0
KD = 2.0
TORQUE_FF = 0.0
TX_RATE_HZ = 100.0
TX_PERIOD_S = 1.0 / TX_RATE_HZ
MAX_VELOCITY_RAD_S = math.radians(10.0)
MAX_ACCELERATION_RAD_S2 = math.radians(30.0)

WATCHDOG_S = 0.150
FEEDBACK_ENVELOPE_RAD = math.radians(8.0)
COMMAND_ENVELOPE_RAD = math.radians(6.0)
VELOCITY_GUARD_RAD_S = 0.7
TEMPERATURE_GUARD_C = 60
PREFLIGHT_SESSION_TOLERANCE_RAD = math.radians(2.0)
HOLD_TOLERANCE_RAD = math.radians(2.0)


class Blocked(RuntimeError):
    """A pre-motion gate or identity requirement was not satisfied."""


class MotionFailure(RuntimeError):
    """An active-motion safety or acceptance requirement failed."""


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat()


def timestamp_token() -> str:
    return datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_text(path: Path) -> Optional[str]:
    try:
        value = path.read_text(encoding="utf-8", errors="replace").strip()
        return value or None
    except OSError:
        return None


def exact_usb_identity() -> dict:
    matches: List[dict] = []
    for entry in sorted(Path("/sys/bus/usb/devices").glob("*")):
        vid = read_text(entry / "idVendor")
        pid = read_text(entry / "idProduct")
        serial = read_text(entry / "serial")
        if vid == EXPECTED_USB_VID and pid == EXPECTED_USB_PID:
            matches.append({
                "sysfs": str(entry),
                "vid": vid,
                "pid": pid,
                "serial": serial,
                "manufacturer": read_text(entry / "manufacturer"),
                "product": read_text(entry / "product"),
                "busnum": read_text(entry / "busnum"),
                "devnum": read_text(entry / "devnum"),
            })
    if len(matches) != 1:
        raise Blocked(f"expected exactly one USB {EXPECTED_USB_VID}:{EXPECTED_USB_PID}, found {len(matches)}")
    identity = matches[0]
    if identity["serial"] != EXPECTED_USB_SERIAL:
        raise Blocked(
            f"DM adapter serial mismatch: expected {EXPECTED_USB_SERIAL}, "
            f"observed {identity['serial']}"
        )
    return identity


def read_only_identity_and_state(session_reference: Optional[float] = None) -> dict:
    usb = exact_usb_identity()
    transport = None
    try:
        transport = DmCanSdkReadOnlyTransport(
            channel=EXPECTED_CAN_CHANNEL, can_bitrate=CAN_BITRATE
        )
        motor = discover_one_motor(transport, range(1, 33), timeout=0.12)
        if int(motor["motor_id"]) != EXPECTED_MOTOR_ID:
            raise Blocked(f"motor ID changed: {motor['motor_id']}")
        if int(motor["ctrl_mode"]) != EXPECTED_CTRL_MODE:
            raise Blocked(
                f"CTRL_MODE changed: {motor['ctrl_mode_name']} ({motor['ctrl_mode']}); "
                "mode write is forbidden"
            )
        feedback = transport.refresh_feedback(EXPECTED_MOTOR_ID, timeout=0.30)
        if feedback is None:
            raise Blocked("initial feedback timed out")
        if feedback.state != 0:
            raise Blocked(f"initial motor state is not DISABLED: {state_text(feedback.state)}")
        if session_reference is not None:
            delta = feedback.position - session_reference
            if abs(delta) > PREFLIGHT_SESSION_TOLERANCE_RAD:
                raise Blocked(
                    f"current q differs from session reference by {math.degrees(delta):.6f} deg"
                )
        return {
            "usb": usb,
            "adapter_count": 1,
            "can_channel": EXPECTED_CAN_CHANNEL,
            "can_bitrate": CAN_BITRATE,
            "canfd": False,
            "motor": motor,
            "feedback": feedback_to_dict(feedback),
            "checked_at_utc": utc_now(),
        }
    finally:
        if transport is not None:
            transport.close()


def feedback_to_dict(feedback: Feedback) -> dict:
    return {
        "can_id": feedback.can_id,
        "state": feedback.state,
        "state_name": state_text(feedback.state),
        "position_rad": feedback.position,
        "velocity_rad_s": feedback.velocity,
        "torque_protocol": feedback.torque,
        "mos_temperature_c": feedback.mos_temp,
        "coil_temperature_c": feedback.coil_temp,
    }


def capture_session(output_dir: Path) -> tuple[dict, Path]:
    usb = exact_usb_identity()
    transport = None
    try:
        transport = DmCanSdkReadOnlyTransport(
            channel=EXPECTED_CAN_CHANNEL, can_bitrate=CAN_BITRATE
        )
        motor = discover_one_motor(transport, range(1, 33), timeout=0.12)
        if int(motor["motor_id"]) != EXPECTED_MOTOR_ID:
            raise Blocked(f"motor ID changed: {motor['motor_id']}")
        if int(motor["ctrl_mode"]) != EXPECTED_CTRL_MODE:
            raise Blocked(
                f"CTRL_MODE changed: {motor['ctrl_mode_name']} ({motor['ctrl_mode']})"
            )
        initial = transport.refresh_feedback(EXPECTED_MOTOR_ID, timeout=0.30)
        if initial is None or initial.state != 0:
            observed = "TIMEOUT" if initial is None else state_text(initial.state)
            raise Blocked(f"session capture requires DISABLED, observed {observed}")

        transport.clear_pending_feedback()
        request_times: List[float] = []
        received: List[tuple[float, Feedback]] = []
        start = time.monotonic()
        for index in range(100):
            planned = start + index * TX_PERIOD_S
            remain = planned - time.monotonic()
            if remain > 0:
                time.sleep(remain)
            request_times.append(time.monotonic())
            transport.send_refresh_request(EXPECTED_MOTOR_ID)
            received.extend(transport.drain_feedback(EXPECTED_MOTOR_ID))
        deadline = time.monotonic() + 0.5
        while len(received) < 100 and time.monotonic() < deadline:
            received.extend(transport.drain_feedback(EXPECTED_MOTOR_ID))
            if len(received) < 100:
                time.sleep(0.001)
        received.extend(transport.drain_feedback(EXPECTED_MOTOR_ID))
        valid = received[:100]
        if len(valid) < 100:
            raise Blocked(f"session capture returned only {len(valid)}/100 valid samples")
        if any(item[1].state != 0 for item in valid):
            states = sorted({state_text(item[1].state) for item in valid})
            raise Blocked(f"session capture saw non-DISABLED state(s): {states}")

        output_dir.mkdir(parents=True, exist_ok=True)
        csv_path = output_dir / f"j6_session_capture_{timestamp_token()}.csv"
        fields = [
            "sample", "request_monotonic_s", "callback_monotonic_s", "rtt_ms",
            "can_id", "state", "state_name", "position_rad", "velocity_rad_s",
            "torque_protocol", "mos_temperature_c", "coil_temperature_c",
        ]
        with csv_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            for index, (callback_time, feedback) in enumerate(valid):
                request_time = request_times[index]
                writer.writerow({
                    "sample": index,
                    "request_monotonic_s": f"{request_time:.9f}",
                    "callback_monotonic_s": f"{callback_time:.9f}",
                    "rtt_ms": f"{max(0.0, callback_time - request_time) * 1000.0:.6f}",
                    **feedback_to_dict(feedback),
                })

        positions = [item[1].position for item in valid]
        summary = {
            "sample_count": len(positions),
            "target_rate_hz": TX_RATE_HZ,
            "q_session_median_rad": statistics.median(positions),
            "q_session_mean_rad": statistics.fmean(positions),
            "q_session_std_rad": statistics.pstdev(positions),
            "q_session_min_rad": min(positions),
            "q_session_max_rad": max(positions),
            "semantics": "SESSION_LOCAL_ONLY",
            "motor_state_distribution": {"DISABLED": len(positions)},
            "csv_path": str(csv_path),
            "csv_sha256": sha256_file(csv_path),
        }
        preflight = {
            "usb": usb,
            "adapter_count": 1,
            "can_channel": EXPECTED_CAN_CHANNEL,
            "can_bitrate": CAN_BITRATE,
            "canfd": False,
            "motor": motor,
            "initial_feedback": feedback_to_dict(initial),
        }
        return {"preflight": preflight, "session": summary}, csv_path
    finally:
        if transport is not None:
            transport.close()


@dataclass(frozen=True)
class CommandSample:
    label: str
    q_rad: float
    dq_rad_s: float


@dataclass(frozen=True)
class TrapezoidProfile:
    start: float
    target: float
    acceleration: float
    peak_velocity: float
    accel_time: float
    cruise_time: float
    duration: float

    @classmethod
    def create(
        cls, start: float, target: float, max_velocity: float, max_acceleration: float
    ) -> "TrapezoidProfile":
        distance = abs(target - start)
        if distance == 0.0:
            return cls(start, target, max_acceleration, 0.0, 0.0, 0.0, 0.0)
        accel_time = max_velocity / max_acceleration
        accel_decel_distance = max_acceleration * accel_time * accel_time
        if distance <= accel_decel_distance:
            accel_time = math.sqrt(distance / max_acceleration)
            peak_velocity = max_acceleration * accel_time
            cruise_time = 0.0
        else:
            peak_velocity = max_velocity
            cruise_distance = distance - accel_decel_distance
            cruise_time = cruise_distance / peak_velocity
        return cls(
            start=start,
            target=target,
            acceleration=max_acceleration,
            peak_velocity=peak_velocity,
            accel_time=accel_time,
            cruise_time=cruise_time,
            duration=2.0 * accel_time + cruise_time,
        )

    def sample(self, elapsed: float) -> tuple[float, float]:
        if self.duration == 0.0 or elapsed >= self.duration:
            return self.target, 0.0
        elapsed = max(0.0, elapsed)
        direction = 1.0 if self.target >= self.start else -1.0
        if elapsed < self.accel_time:
            travelled = 0.5 * self.acceleration * elapsed * elapsed
            velocity = self.acceleration * elapsed
        elif elapsed < self.accel_time + self.cruise_time:
            accel_distance = 0.5 * self.acceleration * self.accel_time**2
            cruise_elapsed = elapsed - self.accel_time
            travelled = accel_distance + self.peak_velocity * cruise_elapsed
            velocity = self.peak_velocity
        else:
            decel_elapsed = elapsed - self.accel_time - self.cruise_time
            accel_distance = 0.5 * self.acceleration * self.accel_time**2
            cruise_distance = self.peak_velocity * self.cruise_time
            travelled = (
                accel_distance
                + cruise_distance
                + self.peak_velocity * decel_elapsed
                - 0.5 * self.acceleration * decel_elapsed**2
            )
            velocity = max(0.0, self.peak_velocity - self.acceleration * decel_elapsed)
        return self.start + direction * travelled, direction * velocity


def hold_commands(label: str, target: float, duration_s: float) -> List[CommandSample]:
    count = max(1, int(round(duration_s * TX_RATE_HZ)))
    return [CommandSample(label, target, 0.0) for _ in range(count)]


def move_commands(label: str, start: float, target: float) -> List[CommandSample]:
    profile = TrapezoidProfile.create(
        start, target, MAX_VELOCITY_RAD_S, MAX_ACCELERATION_RAD_S2
    )
    count = max(1, math.ceil(profile.duration * TX_RATE_HZ))
    commands: List[CommandSample] = []
    for index in range(1, count + 1):
        elapsed = min(index * TX_PERIOD_S, profile.duration)
        q_rad, dq_rad_s = profile.sample(elapsed)
        commands.append(CommandSample(label, q_rad, dq_rad_s))
    commands[-1] = CommandSample(label, target, 0.0)
    return commands


TELEMETRY_FIELDS = [
    "cycle", "label", "timestamp_utc", "tx_monotonic_s", "q_cmd_protocol_rad",
    "dq_cmd_protocol_rad_s", "q_feedback_protocol_rad", "q_relative_protocol_rad",
    "q_relative_ros_rad", "velocity_feedback_rad_s", "torque_feedback_protocol",
    "motor_state", "motor_state_name", "mos_temperature_c", "coil_temperature_c",
    "latest_feedback_timestamp", "feedback_age_ms", "callback_gap_ms",
    "tx_period_ms", "tx_jitter_ms",
]


class MotionLoop:
    def __init__(
        self,
        transport: DmG6220MitMotionTransport,
        session_reference: float,
        protocol_to_ros_sign: Optional[int],
    ):
        self.transport = transport
        self.session_reference = session_reference
        self.protocol_to_ros_sign = protocol_to_ros_sign
        self.enabled_at = time.monotonic()
        self.next_tx = self.enabled_at
        self.last_tx: Optional[float] = None
        self.latest_feedback: Optional[Feedback] = None
        self.latest_feedback_time = self.enabled_at
        self.last_callback_time: Optional[float] = None
        self.latest_callback_gap_ms: Optional[float] = None
        self.records: List[dict] = []
        self.tx_times: List[float] = []
        self.abs_jitter_ms: List[float] = []
        self.callback_gaps_ms: List[float] = []
        self.feedback_ages_ms: List[float] = []
        self.cycle = 0

    def _ingest_feedback(self) -> None:
        for item in self.transport.drain_feedback():
            if self.last_callback_time is not None:
                gap_ms = max(0.0, item.callback_monotonic_s - self.last_callback_time) * 1000.0
                self.latest_callback_gap_ms = gap_ms
                self.callback_gaps_ms.append(gap_ms)
            self.last_callback_time = item.callback_monotonic_s
            self.latest_feedback_time = item.callback_monotonic_s
            self.latest_feedback = item.feedback

    def _guard_violation(self, command: CommandSample, feedback_age_ms: float) -> Optional[str]:
        relative_cmd = command.q_rad - self.session_reference
        if abs(relative_cmd) > COMMAND_ENVELOPE_RAD + 1e-12:
            return f"command envelope exceeded: {math.degrees(relative_cmd):.6f} deg"
        if feedback_age_ms > WATCHDOG_S * 1000.0:
            return f"feedback watchdog: age={feedback_age_ms:.6f} ms"
        feedback = self.latest_feedback
        if feedback is None:
            return None
        if feedback.state != 1:
            return f"motor state is not ENABLED: {state_text(feedback.state)}"
        relative_feedback = feedback.position - self.session_reference
        if abs(relative_feedback) > FEEDBACK_ENVELOPE_RAD:
            return f"feedback envelope exceeded: {math.degrees(relative_feedback):.6f} deg"
        if abs(feedback.velocity) > VELOCITY_GUARD_RAD_S:
            return f"velocity guard exceeded: {feedback.velocity:.9f} rad/s"
        if feedback.mos_temp >= TEMPERATURE_GUARD_C:
            return f"MOS temperature guard: {feedback.mos_temp} C"
        if feedback.coil_temp >= TEMPERATURE_GUARD_C:
            return f"coil temperature guard: {feedback.coil_temp} C"
        return None

    def run(self, commands: Sequence[CommandSample], csv_path: Path) -> List[dict]:
        csv_path.parent.mkdir(parents=True, exist_ok=True)
        run_records: List[dict] = []
        with csv_path.open("w", encoding="utf-8", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=TELEMETRY_FIELDS)
            writer.writeheader()
            for command in commands:
                remain = self.next_tx - time.monotonic()
                if remain > 0:
                    time.sleep(remain)
                tx_time = time.monotonic()
                tx_period_ms = None if self.last_tx is None else (tx_time - self.last_tx) * 1000.0
                tx_jitter_ms = (tx_time - self.next_tx) * 1000.0
                self.transport.send_mit_command(
                    command.q_rad, command.dq_rad_s, KP, KD, TORQUE_FF
                )
                self._ingest_feedback()
                now = time.monotonic()
                feedback_age_ms = max(0.0, now - self.latest_feedback_time) * 1000.0
                self.feedback_ages_ms.append(feedback_age_ms)
                self.tx_times.append(tx_time)
                self.abs_jitter_ms.append(abs(tx_jitter_ms))
                feedback = self.latest_feedback
                q_relative_protocol = None
                q_relative_ros = None
                if feedback is not None:
                    q_relative_protocol = feedback.position - self.session_reference
                    if self.protocol_to_ros_sign is not None:
                        q_relative_ros = self.protocol_to_ros_sign * q_relative_protocol
                row = {
                    "cycle": self.cycle,
                    "label": command.label,
                    "timestamp_utc": utc_now(),
                    "tx_monotonic_s": f"{tx_time:.9f}",
                    "q_cmd_protocol_rad": command.q_rad,
                    "dq_cmd_protocol_rad_s": command.dq_rad_s,
                    "q_feedback_protocol_rad": "" if feedback is None else feedback.position,
                    "q_relative_protocol_rad": "" if q_relative_protocol is None else q_relative_protocol,
                    "q_relative_ros_rad": "" if q_relative_ros is None else q_relative_ros,
                    "velocity_feedback_rad_s": "" if feedback is None else feedback.velocity,
                    "torque_feedback_protocol": "" if feedback is None else feedback.torque,
                    "motor_state": "" if feedback is None else feedback.state,
                    "motor_state_name": "" if feedback is None else state_text(feedback.state),
                    "mos_temperature_c": "" if feedback is None else feedback.mos_temp,
                    "coil_temperature_c": "" if feedback is None else feedback.coil_temp,
                    "latest_feedback_timestamp": "" if feedback is None else f"{self.latest_feedback_time:.9f}",
                    "feedback_age_ms": feedback_age_ms,
                    "callback_gap_ms": "" if self.latest_callback_gap_ms is None else self.latest_callback_gap_ms,
                    "tx_period_ms": "" if tx_period_ms is None else tx_period_ms,
                    "tx_jitter_ms": tx_jitter_ms,
                }
                writer.writerow(row)
                if self.cycle % 10 == 0:
                    stream.flush()
                record = dict(row)
                record["_feedback"] = feedback
                run_records.append(record)
                self.records.append(record)
                violation = self._guard_violation(command, feedback_age_ms)
                self.last_tx = tx_time
                self.next_tx += TX_PERIOD_S
                self.cycle += 1
                if violation is not None:
                    raise MotionFailure(violation)
        return run_records

    def metrics(self) -> dict:
        feedback = [record["_feedback"] for record in self.records if record["_feedback"]]
        if len(self.tx_times) >= 2:
            actual_rate = (len(self.tx_times) - 1) / (self.tx_times[-1] - self.tx_times[0])
        else:
            actual_rate = None
        return {
            "target_tx_rate_hz": TX_RATE_HZ,
            "actual_tx_rate_hz": actual_rate,
            "maximum_tx_jitter_ms": max(self.abs_jitter_ms, default=None),
            "maximum_callback_gap_ms": max(self.callback_gaps_ms, default=None),
            "maximum_feedback_age_ms": max(self.feedback_ages_ms, default=None),
            "maximum_abs_feedback_velocity_rad_s": max(
                (abs(item.velocity) for item in feedback), default=None
            ),
            "torque_protocol": range_dict([item.torque for item in feedback]),
            "mos_temperature_c": range_dict([float(item.mos_temp) for item in feedback]),
            "coil_temperature_c": range_dict([float(item.coil_temp) for item in feedback]),
            "fault_states_observed": sorted({
                state_text(item.state) for item in feedback if item.state not in (0, 1)
            }),
        }


def range_dict(values: Iterable[float]) -> Optional[dict]:
    values = list(values)
    return None if not values else {"min": min(values), "max": max(values)}


def feedback_positions(records: Sequence[dict], label: str) -> List[float]:
    return [
        record["_feedback"].position
        for record in records
        if record["label"] == label and record["_feedback"] is not None
    ]


def tail_median(values: Sequence[float], count: int = 20) -> float:
    if not values:
        raise MotionFailure("no valid feedback available for endpoint acceptance")
    return statistics.median(values[-min(count, len(values)):])


def verify_five_disabled() -> dict:
    transport = None
    try:
        time.sleep(0.10)
        transport = DmCanSdkReadOnlyTransport(
            channel=EXPECTED_CAN_CHANNEL, can_bitrate=CAN_BITRATE
        )
        transport.clear_pending_feedback()
        received: List[tuple[float, Feedback]] = []
        start = time.monotonic()
        for index in range(5):
            planned = start + index * TX_PERIOD_S
            remain = planned - time.monotonic()
            if remain > 0:
                time.sleep(remain)
            transport.send_refresh_request(EXPECTED_MOTOR_ID)
            received.extend(transport.drain_feedback(EXPECTED_MOTOR_ID))
        deadline = time.monotonic() + 0.5
        while len(received) < 5 and time.monotonic() < deadline:
            received.extend(transport.drain_feedback(EXPECTED_MOTOR_ID))
            if len(received) < 5:
                time.sleep(0.001)
        received.extend(transport.drain_feedback(EXPECTED_MOTOR_ID))
        first_five = [item[1] for item in received[:5]]
        passed = len(first_five) == 5 and all(item.state == 0 for item in first_five)
        return {
            "status": "PASS" if passed else "FAIL",
            "valid_frames": len(first_five),
            "states": [state_text(item.state) for item in first_five],
            "feedback": [feedback_to_dict(item) for item in first_five],
        }
    except Exception as exc:
        return {"status": "FAIL", "error": repr(exc), "valid_frames": 0, "states": []}
    finally:
        if transport is not None:
            transport.close()


def require_motion_gates(args: argparse.Namespace) -> None:
    if args.operator_clearance != CLEARANCE_GATE:
        raise Blocked(f"require --operator-clearance '{CLEARANCE_GATE}'")
    if args.operator_emergency != EMERGENCY_GATE:
        raise Blocked(f"require --operator-emergency '{EMERGENCY_GATE}'")


def base_inventory() -> dict:
    return {
        "schema_version": "1.0",
        "task": "V15.21B J6 DM-G6220 MIT small-motion commissioning",
        "source_head": SOURCE_HEAD,
        "work_branch": WORK_BRANCH,
        "host": platform.node(),
        "platform": platform.platform(),
        "created_at_utc": utc_now(),
        "status": "NEW",
        "control": {
            "mode": "MIT",
            "kp": KP,
            "kd": KD,
            "torque_ff": TORQUE_FF,
            "target_tx_rate_hz": TX_RATE_HZ,
            "max_velocity_deg_s": math.degrees(MAX_VELOCITY_RAD_S),
            "max_acceleration_deg_s2": math.degrees(MAX_ACCELERATION_RAD_S2),
        },
        "temporary_guards": {
            "feedback_watchdog_ms": WATCHDOG_S * 1000.0,
            "feedback_envelope_deg": math.degrees(FEEDBACK_ENVELOPE_RAD),
            "command_envelope_deg": math.degrees(COMMAND_ENVELOPE_RAD),
            "velocity_rad_s": VELOCITY_GUARD_RAD_S,
            "mos_temperature_c": TEMPERATURE_GUARD_C,
            "coil_temperature_c": TEMPERATURE_GUARD_C,
            "classification": "V15_21B_TEMPORARY_COMMISSIONING_GUARDS",
        },
        "safety": {
            "ctrl_mode_modified": False,
            "parameter_write_used": False,
            "set_zero_used": False,
            "id_write_used": False,
            "ready_pose_modified": False,
            "j1_authority_modified": False,
            "j345_authority_modified": False,
            "simulation_authority_modified": False,
            "operator_abort": False,
            "watchdog_trips": 0,
            "emergency_disable_events": [],
        },
        "calibration": {
            "protocol_to_ros_sign": "PENDING",
            "cad_zero": "PENDING",
            "ros_zero": "PENDING",
            "motor_internal_zero": "UNCHANGED",
            "physical_limits": "PENDING",
            "ready_reference": "PENDING",
        },
        "authority": {
            "communication": "PASS",
            "feedback": "PASS",
            "local_motion": "NOT_GRANTED",
            "full_joint_motion": False,
            "physical_limits": False,
            "ready_pose": False,
            "full_commissioning": False,
        },
        "evidence": [],
    }


def add_evidence(inventory: dict, path: Path) -> None:
    if not path.exists():
        return
    entry = {"path": str(path), "sha256": sha256_file(path)}
    existing = [item for item in inventory["evidence"] if item.get("path") != str(path)]
    inventory["evidence"] = existing + [entry]


def load_inventory(path: Path) -> dict:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except Exception as exc:
        raise Blocked(f"cannot load session inventory {path}: {exc}") from exc
    session = value.get("session", {})
    if session.get("semantics") != "SESSION_LOCAL_ONLY":
        raise Blocked("session reference authority is missing or has wrong semantics")
    if not isinstance(session.get("q_session_median_rad"), (int, float)):
        raise Blocked("session median is missing")
    return value


def active_phase_common(
    inventory: dict,
    output_dir: Path,
    phase: str,
    protocol_to_ros_sign: Optional[int],
) -> tuple[dict, Optional[MotionLoop], List[Path], Dict[str, List[dict]]]:
    session = float(inventory["session"]["q_session_median_rad"])
    preflight = read_only_identity_and_state(session_reference=session)
    inventory[f"{phase}_preflight"] = preflight
    transport: Optional[DmG6220MitMotionTransport] = None
    loop: Optional[MotionLoop] = None
    paths: List[Path] = []
    records: Dict[str, List[dict]] = {}
    result = {
        "status": "FAIL",
        "started_at_utc": utc_now(),
        "enable_attempted": False,
        "final_fd_disable_sent": False,
        "final_disabled_verification": {"status": "NOT_RUN"},
    }
    failure_reason: Optional[str] = None
    try:
        transport = DmG6220MitMotionTransport(
            motor_id=EXPECTED_MOTOR_ID,
            channel=EXPECTED_CAN_CHANNEL,
            can_bitrate=CAN_BITRATE,
        )
        transport.clear_feedback()
        result["enable_attempted"] = True
        transport.send_enable()
        loop = MotionLoop(transport, session, protocol_to_ros_sign)

        hold_path = output_dir / f"j6_hold_current_{timestamp_token()}.csv"
        hold = hold_commands("HOLD_CURRENT", session, 0.75)
        paths.append(hold_path)
        hold_records = loop.run(hold, hold_path)
        records["hold"] = hold_records
        hold_positions = feedback_positions(hold_records, "HOLD_CURRENT")
        if not hold_positions:
            raise MotionFailure("HOLD returned no valid feedback")
        max_hold_error = max(abs(value - session) for value in hold_positions)
        result["hold_current"] = {
            "status": "PASS" if max_hold_error <= HOLD_TOLERANCE_RAD else "FAIL",
            "maximum_abs_error_rad": max_hold_error,
            "maximum_abs_error_deg": math.degrees(max_hold_error),
        }
        if max_hold_error > HOLD_TOLERANCE_RAD:
            raise MotionFailure(
                f"HOLD current exceeded 2 deg: {math.degrees(max_hold_error):.6f} deg"
            )

        if phase == "sign_probe":
            sign_path = output_dir / f"j6_sign_probe_{timestamp_token()}.csv"
            plus3 = session + math.radians(3.0)
            commands = (
                move_commands("SIGN_MOVE_PLUS3", session, plus3)
                + hold_commands("SIGN_POS_HOLD", plus3, 0.5)
                + move_commands("SIGN_RETURN", plus3, session)
                + hold_commands("SIGN_RETURN_HOLD", session, 0.5)
            )
            paths.append(sign_path)
            phase_records = loop.run(commands, sign_path)
            records["sign_probe"] = phase_records
        elif phase == "bidirectional":
            motion_path = output_dir / f"j6_bidirectional_5deg_{timestamp_token()}.csv"
            plus5 = session + math.radians(5.0)
            minus5 = session - math.radians(5.0)
            commands = (
                move_commands("PLUS5_MOVE", session, plus5)
                + hold_commands("PLUS5_HOLD", plus5, 0.4)
                + move_commands("PLUS5_RETURN", plus5, session)
                + hold_commands("CENTER_HOLD", session, 0.4)
                + move_commands("MINUS5_MOVE", session, minus5)
                + hold_commands("MINUS5_HOLD", minus5, 0.4)
                + move_commands("FINAL_RETURN", minus5, session)
                + hold_commands("FINAL_HOLD", session, 0.4)
            )
            paths.append(motion_path)
            phase_records = loop.run(commands, motion_path)
            records["bidirectional"] = phase_records
        else:
            raise ValueError(f"unsupported active phase {phase}")
        result["status"] = "AUTOMATED_GUARDS_PASS"
    except KeyboardInterrupt:
        inventory["safety"]["operator_abort"] = True
        failure_reason = "operator abort / KeyboardInterrupt"
    except (MotionFailure, TransportError, RuntimeError, ValueError) as exc:
        failure_reason = str(exc)
        if "watchdog" in failure_reason.lower():
            inventory["safety"]["watchdog_trips"] += 1
    except Exception as exc:
        failure_reason = f"software exception: {exc!r}"
    finally:
        if failure_reason and result["enable_attempted"]:
            inventory["safety"]["emergency_disable_events"].append({
                "phase": phase,
                "reason": failure_reason,
                "at_utc": utc_now(),
            })
        if transport is not None:
            try:
                transport.send_disable()
                result["final_fd_disable_sent"] = True
            except Exception as exc:
                result["disable_send_error"] = repr(exc)
                failure_reason = failure_reason or f"FD disable send failed: {exc!r}"
            transport.close()
        verification = verify_five_disabled()
        result["final_disabled_verification"] = verification
        if verification.get("status") != "PASS":
            failure_reason = failure_reason or "final five-frame DISABLED verification failed"
        if loop is not None:
            result["telemetry_metrics"] = loop.metrics()
        result["completed_at_utc"] = utc_now()
        if failure_reason:
            if result["enable_attempted"]:
                result["status"] = "FAIL"
                result["failure_reason"] = failure_reason
            else:
                result["status"] = "BLOCKED"
                result["block_reason"] = failure_reason
    return result, loop, paths, records


def evaluate_sign_probe(inventory: dict, result: dict, records: Dict[str, List[dict]]) -> None:
    if result.get("status") == "BLOCKED":
        result["sign_probe"] = {"status": "NOT_RUN"}
        return
    if result.get("status") != "AUTOMATED_GUARDS_PASS":
        result["sign_probe"] = {"status": "FAIL"}
        return
    session = float(inventory["session"]["q_session_median_rad"])
    sign_records = records["sign_probe"]
    positive = feedback_positions(sign_records, "SIGN_POS_HOLD")
    returned = feedback_positions(sign_records, "SIGN_RETURN_HOLD")
    actual_delta = tail_median(positive) - session
    return_error = tail_median(returned) - session
    endpoint_error = actual_delta - math.radians(3.0)
    passed = (
        actual_delta > 0.0
        and abs(endpoint_error) <= math.radians(1.5)
        and abs(return_error) <= math.radians(1.0)
    )
    result["sign_probe"] = {
        "status": "AUTOMATED_PASS_AWAITING_OPERATOR_CONFIRMATION" if passed else "FAIL",
        "actual_protocol_delta_rad": actual_delta,
        "actual_protocol_delta_deg": math.degrees(actual_delta),
        "endpoint_error_deg": math.degrees(endpoint_error),
        "return_error_deg": math.degrees(return_error),
        "protocol_positive_peak_delta_rad": max(
            record["_feedback"].position - session
            for record in sign_records
            if record["_feedback"] is not None
        ),
    }
    if not passed:
        result["status"] = "FAIL"
        result["failure_reason"] = "sign probe endpoint/direction/return acceptance failed"
        inventory["safety"]["emergency_disable_events"].append({
            "phase": "sign_probe_acceptance",
            "reason": result["failure_reason"],
            "at_utc": utc_now(),
        })


def evaluate_bidirectional(inventory: dict, result: dict, records: Dict[str, List[dict]]) -> None:
    if result.get("status") == "BLOCKED":
        result["bidirectional"] = {"status": "NOT_RUN"}
        return
    if result.get("status") != "AUTOMATED_GUARDS_PASS":
        result["bidirectional"] = {"status": "FAIL"}
        return
    session = float(inventory["session"]["q_session_median_rad"])
    motion = records["bidirectional"]
    plus_values = feedback_positions(motion, "PLUS5_HOLD")
    minus_values = feedback_positions(motion, "MINUS5_HOLD")
    final_values = feedback_positions(motion, "FINAL_HOLD")
    plus_endpoint = tail_median(plus_values) - session
    minus_endpoint = tail_median(minus_values) - session
    final_error = tail_median(final_values) - session
    all_relative = [
        record["_feedback"].position - session
        for record in motion
        if record["_feedback"] is not None
    ]
    plus_error = plus_endpoint - math.radians(5.0)
    minus_error = minus_endpoint + math.radians(5.0)
    plus_pass = plus_endpoint > 0.0 and abs(plus_error) <= math.radians(1.5)
    minus_pass = minus_endpoint < 0.0 and abs(minus_error) <= math.radians(1.5)
    final_pass = abs(final_error) <= math.radians(1.0)
    passed = plus_pass and minus_pass and final_pass
    result["bidirectional"] = {
        "status": "PASS" if passed else "FAIL",
        "plus5_status": "PASS" if plus_pass else "FAIL",
        "plus5_endpoint_delta_deg": math.degrees(plus_endpoint),
        "plus5_endpoint_error_deg": math.degrees(plus_error),
        "plus5_actual_peak_deg": math.degrees(max(all_relative)),
        "minus5_status": "PASS" if minus_pass else "FAIL",
        "minus5_endpoint_delta_deg": math.degrees(minus_endpoint),
        "minus5_endpoint_error_deg": math.degrees(minus_error),
        "minus5_actual_peak_deg": math.degrees(min(all_relative)),
        "final_return_error_deg": math.degrees(final_error),
    }
    if not passed:
        result["status"] = "FAIL"
        result["failure_reason"] = "bidirectional endpoint or final-return acceptance failed"
        inventory["safety"]["emergency_disable_events"].append({
            "phase": "bidirectional_acceptance",
            "reason": result["failure_reason"],
            "at_utc": utc_now(),
        })


def prepare(args: argparse.Namespace) -> int:
    inventory = base_inventory()
    try:
        captured, csv_path = capture_session(args.output_dir)
        inventory.update(captured)
        inventory["status"] = "PREPARED_AWAITING_OPERATOR_MOTION_GATE"
        add_evidence(inventory, csv_path)
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, ensure_ascii=False, indent=2))
        return 0
    except Exception as exc:
        inventory["status"] = "BLOCKED"
        inventory["block_reason"] = str(exc)
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, ensure_ascii=False, indent=2), file=sys.stderr)
        return 4


def sign_probe(args: argparse.Namespace) -> int:
    inventory = load_inventory(args.inventory)
    try:
        require_motion_gates(args)
    except Blocked as exc:
        print(f"BLOCKED: {exc}", file=sys.stderr)
        return 4
    try:
        result, _loop, paths, records = active_phase_common(
            inventory, args.output_dir, "sign_probe", protocol_to_ros_sign=None
        )
    except Exception as exc:
        inventory["status"] = "BLOCKED"
        inventory["block_reason"] = str(exc)
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, ensure_ascii=False, indent=2), file=sys.stderr)
        return 4
    evaluate_sign_probe(inventory, result, records)
    inventory["sign_probe"] = result
    for path in paths:
        add_evidence(inventory, path)
    if result.get("status") == "BLOCKED":
        inventory["status"] = "BLOCKED"
        inventory["authority"]["local_motion"] = "NOT_GRANTED"
        exit_code = 4
    elif result.get("status") == "FAIL":
        inventory["status"] = "FAIL"
        inventory["authority"]["local_motion"] = "NOT_GRANTED"
        exit_code = 2
    else:
        inventory["status"] = "SIGN_PROBE_PASS_AWAITING_OPERATOR_DIRECTION_CONFIRMATION"
        exit_code = 0
    write_json(args.inventory, inventory)
    print(json.dumps(inventory, ensure_ascii=False, indent=2))
    return exit_code


def bidirectional(args: argparse.Namespace) -> int:
    inventory = load_inventory(args.inventory)
    try:
        require_motion_gates(args)
        if inventory.get("sign_probe", {}).get("sign_probe", {}).get("status") != (
            "AUTOMATED_PASS_AWAITING_OPERATOR_CONFIRMATION"
        ):
            raise Blocked("sign probe has not passed automated acceptance")
        if args.direction_confirmation == DIRECTION_YES:
            sign = 1
            answer = "YES"
        elif args.direction_confirmation == DIRECTION_NO:
            sign = -1
            answer = "NO"
        else:
            raise Blocked(
                f"require --direction-confirmation '{DIRECTION_YES}' or '{DIRECTION_NO}'"
            )
        if args.sign_observation != SIGN_OBSERVATION_GATE:
            raise Blocked(f"require --sign-observation '{SIGN_OBSERVATION_GATE}'")
    except Blocked as exc:
        print(f"BLOCKED: {exc}", file=sys.stderr)
        return 4

    inventory["direction_confirmation"] = {
        "operator_answer": answer,
        "authority": [
            "OPERATOR_PHYSICAL_DIRECTION_CONFIRMATION",
            "URDF_MUJOCO_AXIS_REVIEW",
        ],
        "protocol_to_ros_sign": sign,
        "confirmed_at_utc": utc_now(),
    }
    inventory["calibration"]["protocol_to_ros_sign"] = sign
    try:
        result, _loop, paths, records = active_phase_common(
            inventory, args.output_dir, "bidirectional", protocol_to_ros_sign=sign
        )
    except Exception as exc:
        inventory["status"] = "BLOCKED"
        inventory["block_reason"] = str(exc)
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, ensure_ascii=False, indent=2), file=sys.stderr)
        return 4
    evaluate_bidirectional(inventory, result, records)
    inventory["bidirectional"] = result
    for path in paths:
        add_evidence(inventory, path)
    passed = (
        result.get("status") == "AUTOMATED_GUARDS_PASS"
        and result.get("hold_current", {}).get("status") == "PASS"
        and result.get("bidirectional", {}).get("status") == "PASS"
        and result.get("final_disabled_verification", {}).get("status") == "PASS"
        and inventory["safety"]["watchdog_trips"] == 0
        and not result.get("telemetry_metrics", {}).get("fault_states_observed")
        and not inventory["safety"]["operator_abort"]
    )
    if result.get("status") == "BLOCKED":
        inventory["status"] = "BLOCKED"
        inventory["authority"]["local_motion"] = "NOT_GRANTED"
        exit_code = 4
    elif passed:
        inventory["status"] = "PASS_PENDING_OPERATOR_OBSERVATION_CONFIRMATION"
        # The operator must still confirm no collision, abnormal sound, or
        # sustained vibration before config authority may be frozen.
        exit_code = 0
    else:
        inventory["status"] = "FAIL"
        inventory["authority"]["local_motion"] = "NOT_GRANTED"
        exit_code = 2
    write_json(args.inventory, inventory)
    print(json.dumps(inventory, ensure_ascii=False, indent=2))
    return exit_code


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("prepare", "sign-probe", "bidirectional"))
    parser.add_argument("--output-dir", type=Path, default=Path("hardware/v15_21b"))
    parser.add_argument(
        "--inventory",
        type=Path,
        default=Path("hardware/v15_21b/j6_motion_inventory.json"),
    )
    parser.add_argument("--operator-clearance", default="")
    parser.add_argument("--operator-emergency", default="")
    parser.add_argument("--direction-confirmation", default="")
    parser.add_argument("--sign-observation", default="")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.phase == "prepare":
        return prepare(args)
    if args.phase == "sign-probe":
        return sign_probe(args)
    return bidirectional(args)


if __name__ == "__main__":
    raise SystemExit(main())

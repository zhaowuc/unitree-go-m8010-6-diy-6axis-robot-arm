#!/usr/bin/env python3
"""V15.21E gated POS_VEL local-motion commissioning for J6 only."""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics
import struct
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

from dm_g6220_posvel_transport import DmG6220PosVelTransport
from j6_raw_can_diagnostic import (
    Blocked,
    DecodedFeedback,
    RawCanLogger,
    RawEvent,
    add_evidence,
    capture_refresh_samples,
    exact_usb_identity,
    read_parameter,
    state_text,
    strict_decode,
    timestamp_token,
    utc_now,
    write_json,
    write_position_csv,
    write_raw_csv,
)


SOURCE_HEAD = "eb07deabed72f131a64f8dafd3543c7898c4e1d8"
WORK_BRANCH = "agent/v15-21e-j6-posvel-local-motion"
MOTOR_ID = 1
CTRL_MODE_MIT = 1
CTRL_MODE_POS_VEL = 2
MODE_GATE = "J6_ALLOW_RUNTIME_CTRL_MODE_SWITCH_TO_POS_VEL=YES"
MOTION_CLEARANCE_GATE = "J6_POSVEL_LOCAL_MOTION_CLEARANCE_10DEG=YES"
MOTION_STOP_GATE = "J6_POSVEL_EMERGENCY_STOP_READY=YES"
SIGN_YES = "J6_PROTOCOL_POSITIVE_IS_ROS_POSITIVE=YES"
SIGN_NO = "J6_PROTOCOL_POSITIVE_IS_ROS_POSITIVE=NO"
SAFE_OBSERVATION = "J6_POSVEL_BIDIRECTIONAL_OBSERVATION_SAFE=YES"
REPEAT_SIGN_GATE = "J6_REPEAT_SAME_PLUS_3DEG_SIGN_PROBE=YES"

TX_HZ = 100.0
TX_PERIOD_S = 1.0 / TX_HZ
VELOCITY_LIMIT_RAD_S = math.radians(10.0)
COMMAND_ENVELOPE_RAD = math.radians(6.0)
FEEDBACK_ENVELOPE_RAD = math.radians(8.0)
VELOCITY_GUARD_RAD_S = 0.7
TEMPERATURE_GUARD_C = 60
FEEDBACK_AGE_LIMIT_S = 0.150
FAULT_STATES = {8, 9, 0xA, 0xB, 0xC, 0xD, 0xE}


class SafetyFailure(RuntimeError):
    """Fail-closed active-motion acceptance failure."""


@dataclass
class TelemetrySample:
    timestamp_monotonic_s: float
    phase: str
    tx_target_position_rad: float
    tx_velocity_limit_rad_s: float
    tx_can_id: int
    tx_payload_hex: str
    feedback_position_rad: Optional[float]
    feedback_velocity_rad_s: Optional[float]
    feedback_torque_protocol: Optional[float]
    feedback_state: Optional[int]
    feedback_state_name: str
    mos_temperature_c: Optional[int]
    coil_temperature_c: Optional[int]
    feedback_age_ms: Optional[float]
    tx_period_ms: Optional[float]
    tx_jitter_ms: float


TELEMETRY_FIELDS = [
    "sample", "timestamp_monotonic_s", "phase", "tx_target_position_rad",
    "tx_velocity_limit_rad_s", "tx_can_id_hex", "tx_payload_hex",
    "feedback_position_rad", "feedback_velocity_rad_s",
    "feedback_torque_protocol", "feedback_state", "feedback_state_name",
    "mos_temperature_c", "coil_temperature_c", "feedback_age_ms",
    "tx_period_ms", "tx_jitter_ms",
]


def write_telemetry_csv(path: Path, samples: list[TelemetrySample]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=TELEMETRY_FIELDS)
        writer.writeheader()
        for index, item in enumerate(samples):
            writer.writerow({
                "sample": index,
                "timestamp_monotonic_s": f"{item.timestamp_monotonic_s:.9f}",
                "phase": item.phase,
                "tx_target_position_rad": f"{item.tx_target_position_rad:.15g}",
                "tx_velocity_limit_rad_s": f"{item.tx_velocity_limit_rad_s:.15g}",
                "tx_can_id_hex": f"0x{item.tx_can_id:X}",
                "tx_payload_hex": item.tx_payload_hex,
                "feedback_position_rad": "" if item.feedback_position_rad is None else f"{item.feedback_position_rad:.15g}",
                "feedback_velocity_rad_s": "" if item.feedback_velocity_rad_s is None else f"{item.feedback_velocity_rad_s:.15g}",
                "feedback_torque_protocol": "" if item.feedback_torque_protocol is None else f"{item.feedback_torque_protocol:.15g}",
                "feedback_state": "" if item.feedback_state is None else item.feedback_state,
                "feedback_state_name": item.feedback_state_name,
                "mos_temperature_c": "" if item.mos_temperature_c is None else item.mos_temperature_c,
                "coil_temperature_c": "" if item.coil_temperature_c is None else item.coil_temperature_c,
                "feedback_age_ms": "" if item.feedback_age_ms is None else f"{item.feedback_age_ms:.6f}",
                "tx_period_ms": "" if item.tx_period_ms is None else f"{item.tx_period_ms:.6f}",
                "tx_jitter_ms": f"{item.tx_jitter_ms:.6f}",
            })


def base_inventory() -> dict:
    return {
        "schema_version": "1.0",
        "task": "V15.21E J6 POS_VEL local motion commissioning",
        "source_head": SOURCE_HEAD,
        "work_branch": WORK_BRANCH,
        "created_at_utc": utc_now(),
        "status": "NEW",
        "frozen_authority": {
            "v15_21a_communication_feedback": "PASS",
            "v15_21b_mit_hold": "FAIL",
            "v15_21c_root_cause": "REAL_ENABLE_TRANSIENT",
            "v15_21d_posvel_enable_hold": "PASS",
        },
        "command_contract": {
            "mode": "POS_VEL",
            "can_id": 0x101,
            "payload": "<ff absolute_position_rad positive_velocity_limit_rad_s",
            "tx_refresh_target_hz": TX_HZ,
            "velocity_limit_deg_s": 10.0,
            "velocity_limit_rad_s": VELOCITY_LIMIT_RAD_S,
        },
        "guards": {
            "command_envelope_deg": 6.0,
            "feedback_envelope_deg": 8.0,
            "velocity_abs_rad_s": VELOCITY_GUARD_RAD_S,
            "temperature_c_exclusive_upper": TEMPERATURE_GUARD_C,
            "feedback_age_ms": FEEDBACK_AGE_LIMIT_S * 1000.0,
        },
        "safety": {
            "mit_retested": False,
            "set_zero_used": False,
            "parameter_write_other_than_optional_rid10": False,
            "flash_eeprom_save_used": False,
            "ready_pose_modified": False,
            "j1_j345_simulation_modified": False,
        },
        "authority": {
            "j6_posvel_local_motion": "NOT_ESTABLISHED",
            "j6_bidirectional_5deg": "NOT_ESTABLISHED",
            "j6_local_motion_authority": "NOT_GRANTED",
            "j6_full_range": "NOT_TESTED",
            "j6_physical_limits": "PENDING",
            "j6_cad_zero": "PENDING",
            "j6_ros_zero": "PENDING",
            "j6_ready_reference": "PENDING",
            "j6_full_commissioning": "NOT_COMPLETE",
        },
        "evidence": [],
    }


def readonly_motor(logger: RawCanLogger) -> dict:
    master_id = int(read_parameter(logger, 7))
    logger.master_id = master_id
    return {
        "master_id": master_id,
        "esc_id": int(read_parameter(logger, 8)),
        "ctrl_mode": int(read_parameter(logger, 10)),
        "pmax": read_parameter(logger, 21),
        "allowed_normal_feedback_can_ids": sorted(logger.allowed_feedback_ids()),
    }


def session_stats(values) -> dict:
    positions = [item[1].position for item in values]
    return {
        "valid_frames": len(values),
        "total_required": 100,
        "q_median_rad": statistics.median(positions),
        "q_mean_rad": statistics.fmean(positions),
        "q_std_rad": statistics.pstdev(positions),
        "q_min_rad": min(positions),
        "q_max_rad": max(positions),
        "semantics": "SESSION_LOCAL_ONLY",
    }


def preflight(args: argparse.Namespace) -> int:
    inventory = base_inventory()
    args.output_dir.mkdir(parents=True, exist_ok=True)
    inventory["usb"] = exact_usb_identity()
    logger = RawCanLogger()
    try:
        time.sleep(0.1)
        motor = readonly_motor(logger)
        inventory["motor"] = motor
        inventory["ctrl_mode_initial"] = motor["ctrl_mode"]
        if motor["esc_id"] != MOTOR_ID:
            raise Blocked(f"RID8 ESC ID changed: {motor['esc_id']}")
        initial, _ = capture_refresh_samples(logger, 5, "INITIAL_DISABLED")
        if any(item[1].state != 0 for item in initial):
            raise Blocked("motor is not DISABLED")
        if motor["ctrl_mode"] == CTRL_MODE_MIT:
            inventory["runtime_mode_write_required"] = True
            inventory["runtime_mode_write_count"] = 0
            inventory["status"] = "AWAITING_RUNTIME_MODE_WRITE_GATE"
            write_json(args.inventory, inventory)
            print(json.dumps(inventory, indent=2))
            return 3
        if motor["ctrl_mode"] != CTRL_MODE_POS_VEL:
            raise Blocked(f"unsupported RID10 value: {motor['ctrl_mode']}")
        inventory["runtime_mode_write_required"] = False
        inventory["runtime_mode_write_count"] = 0
        values, _ = capture_refresh_samples(logger, 100, "SESSION_V15_21E")
        if len(values) != 100 or any(item[1].state != 0 for item in values):
            raise Blocked("session capture acceptance failed")
        session = session_stats(values)
        inventory["session"] = session
        path = args.output_dir / f"j6_session_capture_{timestamp_token()}.csv"
        write_position_csv(path, values)
        add_evidence(inventory, path)
        inventory["status"] = "SESSION_PASS_AWAITING_LOCAL_MOTION_GATE"
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, indent=2))
        return 0
    except Blocked as exc:
        inventory["status"] = "BLOCKED"
        inventory["block_reason"] = str(exc)
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, indent=2))
        return 4
    finally:
        logger.close()


def switch_mode(args: argparse.Namespace) -> int:
    if args.operator_mode_gate != MODE_GATE:
        raise Blocked(f"require --operator-mode-gate '{MODE_GATE}'")
    inventory = json.loads(args.inventory.read_text(encoding="utf-8"))
    if inventory.get("status") != "AWAITING_RUNTIME_MODE_WRITE_GATE":
        raise Blocked("inventory does not authorize RID10 write")
    logger = RawCanLogger()
    try:
        exact_usb_identity()
        motor = readonly_motor(logger)
        if motor["ctrl_mode"] != CTRL_MODE_MIT:
            raise Blocked(f"RID10 changed before authorized write: {motor['ctrl_mode']}")
        initial, _ = capture_refresh_samples(logger, 5, "PRE_MODE_WRITE_DISABLED")
        if any(item[1].state != 0 for item in initial):
            raise Blocked("motor is not DISABLED before RID10 write")
        payload = bytes((MOTOR_ID, 0, 0x55, 10)) + struct.pack("<I", CTRL_MODE_POS_VEL)
        logger.send(0x7FF, payload, "WRITE_RID10_POS_VEL_ONCE")
        inventory["runtime_mode_write_count"] = 1
        readback = int(read_parameter(logger, 10))
        inventory["ctrl_mode_after_write"] = readback
        if readback != CTRL_MODE_POS_VEL:
            raise Blocked(f"RID10 readback failed: {readback}")
        inventory["status"] = "MODE_SWITCHED_RERUN_PREFLIGHT"
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, indent=2))
        return 0
    finally:
        logger.close()


class MotionHarness:
    def __init__(self, transport: DmG6220PosVelTransport, q_session: float) -> None:
        self.transport = transport
        self.logger = transport._raw_logger_for_commissioning()
        self.q_session = q_session
        self.raw_cursor = len(self.logger.snapshot())
        self.latest_event: Optional[RawEvent] = None
        self.latest: Optional[DecodedFeedback] = None
        self.enabled_seen = False
        self.feedback_history: list[tuple[RawEvent, DecodedFeedback]] = []
        self.telemetry: list[TelemetrySample] = []
        self.tx_times: list[float] = []
        self.tx_jitters_ms: list[float] = []
        self.next_tx_scheduled: Optional[float] = None
        self.unexpected_rx_count = 0
        self.watchdog_trips = 0
        self.fault_states: set[str] = set()
        self.active_start: Optional[float] = None

    def _next_schedule(self) -> float:
        if self.next_tx_scheduled is None:
            self.next_tx_scheduled = time.monotonic()
        scheduled = self.next_tx_scheduled
        self.next_tx_scheduled += TX_PERIOD_S
        remaining = scheduled - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        return scheduled

    def _process_feedback(self) -> None:
        snapshot = self.logger.snapshot()
        new_events = snapshot[self.raw_cursor:]
        self.raw_cursor = len(snapshot)
        for event in new_events:
            classification = self.logger.classify(event)
            if event.event_kind == "RX_CALLBACK" and classification == "UNKNOWN":
                self.unexpected_rx_count += 1
                raise SafetyFailure("unexpected RX frame")
            if classification != "NORMAL_FEEDBACK":
                continue
            decoded = strict_decode(event.payload)
            self.feedback_history.append((event, decoded))
            if decoded.state in FAULT_STATES:
                name = state_text(decoded.state)
                self.fault_states.add(name)
                raise SafetyFailure(f"fault state: {name}")
            if abs(decoded.position - self.q_session) > FEEDBACK_ENVELOPE_RAD:
                raise SafetyFailure("feedback envelope exceeded +/-8 deg")
            if abs(decoded.velocity) > VELOCITY_GUARD_RAD_S:
                raise SafetyFailure("feedback velocity exceeded 0.7 rad/s")
            if decoded.mos_temp >= TEMPERATURE_GUARD_C or decoded.coil_temp >= TEMPERATURE_GUARD_C:
                raise SafetyFailure("temperature guard tripped")
            if decoded.state == 1:
                self.enabled_seen = True
                self.latest_event = event
                self.latest = decoded
            elif self.enabled_seen and decoded.state != 1:
                raise SafetyFailure(f"unexpected active state: {state_text(decoded.state)}")

    def _check_feedback_age(self, now: float) -> None:
        if self.active_start is None:
            return
        reference = self.latest_event.event_monotonic_s if self.latest_event else self.active_start
        if now - reference > FEEDBACK_AGE_LIMIT_S:
            self.watchdog_trips += 1
            raise SafetyFailure("feedback age exceeded 150 ms")

    def _send_tick(self, phase: str, target: float, velocity: float, scheduled: float) -> None:
        if abs(target - self.q_session) > COMMAND_ENVELOPE_RAD:
            raise SafetyFailure("command envelope exceeded +/-6 deg")
        now = time.monotonic()
        tx_period_ms = None if not self.tx_times else (now - self.tx_times[-1]) * 1000.0
        jitter_ms = (now - scheduled) * 1000.0
        sent_at, payload = self.transport.send_pos_vel_command(target, velocity, phase)
        self.tx_times.append(sent_at)
        self.tx_jitters_ms.append(abs(jitter_ms))
        self._process_feedback()
        self._check_feedback_age(time.monotonic())
        age_ms = None
        if self.latest_event is not None:
            age_ms = (time.monotonic() - self.latest_event.event_monotonic_s) * 1000.0
        feedback = self.latest
        self.telemetry.append(TelemetrySample(
            timestamp_monotonic_s=sent_at,
            phase=phase,
            tx_target_position_rad=target,
            tx_velocity_limit_rad_s=velocity,
            tx_can_id=0x101,
            tx_payload_hex=payload.hex().upper(),
            feedback_position_rad=None if feedback is None else feedback.position,
            feedback_velocity_rad_s=None if feedback is None else feedback.velocity,
            feedback_torque_protocol=None if feedback is None else feedback.torque,
            feedback_state=None if feedback is None else feedback.state,
            feedback_state_name="" if feedback is None else state_text(feedback.state),
            mos_temperature_c=None if feedback is None else feedback.mos_temp,
            coil_temperature_c=None if feedback is None else feedback.coil_temp,
            feedback_age_ms=age_ms,
            tx_period_ms=tx_period_ms,
            tx_jitter_ms=jitter_ms,
        ))

    def run_duration(self, phase: str, target: float, velocity: float, duration_s: float) -> tuple[float, float]:
        start = time.monotonic()
        deadline = start + duration_s
        history_start = len(self.feedback_history)
        while time.monotonic() < deadline:
            if self.next_tx_scheduled is not None and self.next_tx_scheduled >= deadline:
                break
            scheduled = self._next_schedule()
            self._send_tick(phase, target, velocity, scheduled)
        self._process_feedback()
        values = [item[1].position for item in self.feedback_history[history_start:] if item[1].state == 1]
        if not values:
            raise SafetyFailure(f"no enabled feedback during {phase}")
        return statistics.median(values), time.monotonic() - start

    def run_motion(
        self,
        phase: str,
        target: float,
        timeout_s: float,
        relaxed_timeout_deg: Optional[float] = None,
    ) -> tuple[float, float]:
        start = time.monotonic()
        deadline = start + timeout_s
        while time.monotonic() < deadline:
            if self.next_tx_scheduled is not None and self.next_tx_scheduled >= deadline:
                break
            scheduled = self._next_schedule()
            self._send_tick(phase, target, VELOCITY_LIMIT_RAD_S, scheduled)
            if self.latest is not None and abs(self.latest.position - target) <= math.radians(1.0):
                return self.latest.position, time.monotonic() - start
        self._process_feedback()
        if (
            relaxed_timeout_deg is not None
            and self.latest is not None
            and abs(self.latest.position - target) <= math.radians(relaxed_timeout_deg)
        ):
            return self.latest.position, time.monotonic() - start
        raise SafetyFailure(f"endpoint timeout: {phase}")

    def metrics(self) -> dict:
        feedback = [item[1] for item in self.feedback_history]
        periods = [
            (current - previous) for previous, current in zip(self.tx_times, self.tx_times[1:])
        ]
        duration = self.tx_times[-1] - self.tx_times[0] if len(self.tx_times) > 1 else 0.0
        actual_rate = (len(self.tx_times) - 1) / duration if duration > 0 else 0.0
        ages = [item.feedback_age_ms for item in self.telemetry if item.feedback_age_ms is not None]
        return {
            "tx_samples": len(self.tx_times),
            "actual_tx_rate_hz": actual_rate,
            "mean_tx_period_ms": statistics.fmean(periods) * 1000.0 if periods else None,
            "max_tx_jitter_ms": max(self.tx_jitters_ms, default=0.0),
            "maximum_feedback_age_ms": max(ages, default=0.0),
            "maximum_abs_feedback_velocity_rad_s": max((abs(item.velocity) for item in feedback), default=0.0),
            "torque_protocol_min": min((item.torque for item in feedback), default=None),
            "torque_protocol_max": max((item.torque for item in feedback), default=None),
            "mos_temperature_min_c": min((item.mos_temp for item in feedback), default=None),
            "mos_temperature_max_c": max((item.mos_temp for item in feedback), default=None),
            "coil_temperature_min_c": min((item.coil_temp for item in feedback), default=None),
            "coil_temperature_max_c": max((item.coil_temp for item in feedback), default=None),
            "watchdog_trips": self.watchdog_trips,
            "fault_states": sorted(self.fault_states),
            "unexpected_rx_count": self.unexpected_rx_count,
        }


def require_motion_gates(args: argparse.Namespace) -> None:
    if args.operator_clearance != MOTION_CLEARANCE_GATE:
        raise Blocked(f"require --operator-clearance '{MOTION_CLEARANCE_GATE}'")
    if args.operator_stop != MOTION_STOP_GATE:
        raise Blocked(f"require --operator-stop '{MOTION_STOP_GATE}'")


def active_round(
    args: argparse.Namespace,
    kind: str,
    q_session: float,
    inventory: dict,
) -> int:
    exact_usb_identity()
    master_id = int(inventory["motor"]["master_id"])
    transport = DmG6220PosVelTransport(master_id)
    logger = transport._raw_logger_for_commissioning()
    harness = MotionHarness(transport, q_session)
    failure: Optional[str] = None
    final_fd_sent = False
    final_disabled = False
    result: dict = {"status": "RUNNING", "q_session_rad": q_session}
    try:
        time.sleep(0.1)
        mode = int(read_parameter(logger, 10))
        if mode != CTRL_MODE_POS_VEL:
            raise Blocked(f"RID10 is {mode}, expected POS_VEL/2")
        initial, _ = capture_refresh_samples(logger, 5, f"{kind.upper()}_PRE_ENABLE_DISABLED")
        if any(item[1].state != 0 for item in initial):
            raise Blocked("motor is not DISABLED before active round")
        current_median = statistics.median(item[1].position for item in initial)
        if abs(current_median - q_session) > math.radians(2.0):
            raise Blocked("current position differs from session by more than 2 deg")

        transport.send_pos_vel_command(q_session, 0.0, f"{kind.upper()}_PRELOAD_DISABLED")
        fc_at = transport.send_enable()
        harness.active_start = fc_at
        transport.send_pos_vel_command(q_session, 0.0, f"{kind.upper()}_HOLD_IMMEDIATE")
        hold_median, _ = harness.run_duration("INITIAL_HOLD", q_session, 0.0, 0.20)
        if abs(hold_median - q_session) > math.radians(1.0):
            raise SafetyFailure("initial HOLD position error exceeded 1 deg")
        result["initial_hold"] = "PASS"

        if kind == "sign_probe":
            target = q_session + math.radians(3.0)
            result["target_absolute_rad"] = target
            harness.run_motion("PLUS_3_MOTION", target, 0.80, relaxed_timeout_deg=1.5)
            endpoint, _ = harness.run_duration("PLUS_3_HOLD", target, 0.0, 0.40)
            delta_deg = math.degrees(endpoint - q_session)
            error_deg = abs(math.degrees(endpoint - target))
            if delta_deg <= 0 or error_deg > 1.5:
                raise SafetyFailure("+3 deg endpoint acceptance failed")
            result.update({
                "plus_3deg": "PASS",
                "endpoint_median_rad": endpoint,
                "endpoint_actual_delta_deg": delta_deg,
                "endpoint_error_deg": error_deg,
            })
            harness.run_motion("PLUS_3_RETURN", q_session, 0.80, relaxed_timeout_deg=1.0)
            returned, _ = harness.run_duration("PLUS_3_RETURN_HOLD", q_session, 0.0, 0.40)
            return_error = abs(math.degrees(returned - q_session))
            if return_error > 1.0:
                raise SafetyFailure("+3 deg return error exceeded 1 deg")
            result["return_error_deg"] = return_error
        else:
            plus_target = q_session + math.radians(5.0)
            minus_target = q_session - math.radians(5.0)
            harness.run_motion("PLUS_5_MOTION", plus_target, 1.0)
            plus_endpoint, _ = harness.run_duration("PLUS_5_HOLD", plus_target, 0.0, 0.40)
            plus_delta = math.degrees(plus_endpoint - q_session)
            plus_error = abs(math.degrees(plus_endpoint - plus_target))
            if plus_delta <= 0 or plus_error > 1.5:
                raise SafetyFailure("+5 deg endpoint acceptance failed")
            harness.run_motion("FIRST_CENTER_RETURN", q_session, 1.0)
            first_center, _ = harness.run_duration("FIRST_CENTER_HOLD", q_session, 0.0, 0.30)
            first_center_error = abs(math.degrees(first_center - q_session))
            if first_center_error > 1.0:
                raise SafetyFailure("first center return error exceeded 1 deg")
            harness.run_motion("MINUS_5_MOTION", minus_target, 1.0)
            minus_endpoint, _ = harness.run_duration("MINUS_5_HOLD", minus_target, 0.0, 0.40)
            minus_delta = math.degrees(minus_endpoint - q_session)
            minus_error = abs(math.degrees(minus_endpoint - minus_target))
            if minus_delta >= 0 or minus_error > 1.5:
                raise SafetyFailure("-5 deg endpoint acceptance failed")
            harness.run_motion("FINAL_CENTER_RETURN", q_session, 1.0)
            final_center, _ = harness.run_duration("FINAL_CENTER_HOLD", q_session, 0.0, 0.40)
            final_error = abs(math.degrees(final_center - q_session))
            if final_error > 1.0:
                raise SafetyFailure("final center return error exceeded 1 deg")
            result.update({
                "second_hold": "PASS",
                "plus_5deg": "PASS",
                "plus_5_endpoint_median_rad": plus_endpoint,
                "plus_5_endpoint_delta_deg": plus_delta,
                "plus_5_endpoint_error_deg": plus_error,
                "first_center_return_error_deg": first_center_error,
                "minus_5deg": "PASS",
                "minus_5_endpoint_median_rad": minus_endpoint,
                "minus_5_endpoint_delta_deg": minus_delta,
                "minus_5_endpoint_error_deg": minus_error,
                "final_return_error_deg": final_error,
            })
    except KeyboardInterrupt:
        failure = "KeyboardInterrupt: operator abort"
    except Exception as exc:
        failure = f"{type(exc).__name__}: {exc}"
    finally:
        try:
            transport.send_disable("FD_DISABLE_FINAL_1")
            final_fd_sent = True
            time.sleep(0.020)
            transport.send_disable("FD_DISABLE_FINAL_2")
            time.sleep(0.020)
            transport.send_disable("FD_DISABLE_FINAL_3")
            time.sleep(0.315)
            final_values, _ = capture_refresh_samples(logger, 5, f"{kind.upper()}_FINAL_DISABLED")
            final_disabled = len(final_values) == 5 and all(item[1].state == 0 for item in final_values)
        except Exception as exc:
            failure = failure or f"final disable verification: {exc}"
        try:
            result["ctrl_mode_final"] = int(read_parameter(logger, 10))
        except Exception:
            result["ctrl_mode_final"] = "UNKNOWN"
        raw_events = logger.snapshot()
        token = timestamp_token()
        telemetry_path = args.output_dir / f"j6_posvel_{kind}_{token}.csv"
        raw_path = args.output_dir / f"j6_posvel_{kind}_raw_{token}.csv"
        write_telemetry_csv(telemetry_path, harness.telemetry)
        write_raw_csv(raw_path, logger, raw_events)
        add_evidence(inventory, telemetry_path)
        add_evidence(inventory, raw_path)
        result.update(harness.metrics())
        result["final_fd_sent"] = final_fd_sent
        result["final_five_frame_disabled"] = "PASS" if final_disabled else "FAIL"
        result["telemetry_csv_path"] = str(telemetry_path)
        result["raw_csv_path"] = str(raw_path)
        if failure or not final_disabled:
            result["status"] = "FAIL"
            result["failure_reason"] = failure or "final DISABLED verification failed"
        else:
            result["status"] = "PASS"
        inventory[kind] = result
        transport.close()

    if result["status"] != "PASS":
        inventory["status"] = "FAIL"
        inventory["final_task_result"] = "FAIL"
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, indent=2))
        return 2
    if kind == "sign_probe":
        inventory["status"] = "SIGN_PROBE_PASS_AWAITING_OPERATOR_DIRECTION"
    else:
        inventory["status"] = "BIDIRECTIONAL_PASS_AWAITING_OPERATOR_SAFE_OBSERVATION"
    write_json(args.inventory, inventory)
    print(json.dumps(inventory, indent=2))
    return 0


def run_sign_probe(args: argparse.Namespace) -> int:
    require_motion_gates(args)
    inventory = json.loads(args.inventory.read_text(encoding="utf-8"))
    if inventory.get("status") != "SESSION_PASS_AWAITING_LOCAL_MOTION_GATE":
        raise Blocked(f"inventory does not authorize sign probe: {inventory.get('status')}")
    inventory["operator_motion_gate"] = "PASS"
    return active_round(args, "sign_probe", float(inventory["session"]["q_median_rad"]), inventory)


def repeat_sign_probe(args: argparse.Namespace) -> int:
    if args.operator_repeat_sign != REPEAT_SIGN_GATE:
        raise Blocked(f"require --operator-repeat-sign '{REPEAT_SIGN_GATE}'")
    inventory = json.loads(args.inventory.read_text(encoding="utf-8"))
    if inventory.get("status") != "SIGN_PROBE_PASS_AWAITING_OPERATOR_DIRECTION":
        raise Blocked("inventory is not awaiting the first sign observation")
    if inventory.get("sign_probe_attempts"):
        raise Blocked("the single observation repeat has already been used")
    exact_usb_identity()
    logger = RawCanLogger()
    try:
        time.sleep(0.1)
        motor = readonly_motor(logger)
        if motor["ctrl_mode"] != CTRL_MODE_POS_VEL or motor["esc_id"] != MOTOR_ID:
            raise Blocked("identity or POS_VEL mode changed")
        values, _ = capture_refresh_samples(logger, 100, "SIGN_REPEAT_SESSION")
        if any(item[1].state != 0 for item in values):
            raise Blocked("motor is not DISABLED before sign observation repeat")
        repeat_session = session_stats(values)
        original_q = float(inventory["session"]["q_median_rad"])
        difference_deg = math.degrees(repeat_session["q_median_rad"] - original_q)
        repeat_session["difference_from_original_session_deg"] = difference_deg
        if abs(difference_deg) > 2.0:
            raise Blocked("repeat session differs from original by more than 2 deg")
        path = args.output_dir / f"j6_session_capture_sign_repeat_{timestamp_token()}.csv"
        write_position_csv(path, values)
        add_evidence(inventory, path)
        inventory["sign_probe_repeat_session"] = repeat_session
    finally:
        logger.close()
    previous = inventory.get("sign_probe")
    if previous:
        previous["operator_observation"] = "NOT_OBSERVED_CLEARLY"
        inventory.setdefault("sign_probe_attempts", []).append(previous)
    inventory["explicit_same_plus_3deg_observation_repeat"] = True
    return active_round(
        args,
        "sign_probe",
        float(inventory["sign_probe_repeat_session"]["q_median_rad"]),
        inventory,
    )


def record_sign(args: argparse.Namespace) -> int:
    inventory = json.loads(args.inventory.read_text(encoding="utf-8"))
    if inventory.get("status") != "SIGN_PROBE_PASS_AWAITING_OPERATOR_DIRECTION":
        raise Blocked("inventory is not awaiting sign confirmation")
    if args.operator_sign not in (SIGN_YES, SIGN_NO):
        raise Blocked("operator sign answer must be the exact YES or NO token")
    answer = "YES" if args.operator_sign == SIGN_YES else "NO"
    sign = 1 if answer == "YES" else -1
    inventory["operator_ros_direction_answer"] = answer
    inventory["protocol_to_ros_sign"] = sign
    inventory["sign_authority"] = [
        "OPERATOR_PHYSICAL_DIRECTION_CONFIRMATION",
        "URDF_MUJOCO_AXIS_REVIEW",
    ]
    inventory["status"] = "SIGN_ESTABLISHED_PREPARE_BIDIRECTIONAL"
    write_json(args.inventory, inventory)
    print(json.dumps(inventory, indent=2))
    return 0


def prepare_bidirectional(args: argparse.Namespace) -> int:
    inventory = json.loads(args.inventory.read_text(encoding="utf-8"))
    args.output_dir.mkdir(parents=True, exist_ok=True)
    if inventory.get("status") != "SIGN_ESTABLISHED_PREPARE_BIDIRECTIONAL":
        raise Blocked("sign authority is not established")
    exact_usb_identity()
    logger = RawCanLogger()
    try:
        time.sleep(0.1)
        motor = readonly_motor(logger)
        if motor["ctrl_mode"] != CTRL_MODE_POS_VEL or motor["esc_id"] != MOTOR_ID:
            raise Blocked("identity or POS_VEL mode changed")
        values, _ = capture_refresh_samples(logger, 100, "BIDIRECTIONAL_SESSION")
        if any(item[1].state != 0 for item in values):
            raise Blocked("motor is not DISABLED before bidirectional round")
        second = session_stats(values)
        first_q = float(inventory["session"]["q_median_rad"])
        delta_deg = math.degrees(second["q_median_rad"] - first_q)
        second["difference_from_sign_session_deg"] = delta_deg
        if abs(delta_deg) > 2.0:
            raise Blocked("new session differs by more than 2 deg")
        path = args.output_dir / f"j6_session_capture_bidirectional_{timestamp_token()}.csv"
        write_position_csv(path, values)
        add_evidence(inventory, path)
        inventory["bidirectional_session"] = second
        inventory["status"] = "BIDIRECTIONAL_SESSION_PASS"
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, indent=2))
        return 0
    finally:
        logger.close()


def run_bidirectional(args: argparse.Namespace) -> int:
    inventory = json.loads(args.inventory.read_text(encoding="utf-8"))
    if inventory.get("status") != "BIDIRECTIONAL_SESSION_PASS":
        raise Blocked("bidirectional session is not authorized")
    return active_round(
        args,
        "bidirectional_5deg",
        float(inventory["bidirectional_session"]["q_median_rad"]),
        inventory,
    )


def finalize(args: argparse.Namespace) -> int:
    inventory = json.loads(args.inventory.read_text(encoding="utf-8"))
    if inventory.get("status") != "BIDIRECTIONAL_PASS_AWAITING_OPERATOR_SAFE_OBSERVATION":
        raise Blocked("inventory is not awaiting safe observation")
    if args.operator_safe != SAFE_OBSERVATION:
        inventory["operator_safe_observation"] = "NO"
        inventory["status"] = "FAIL"
        inventory["final_task_result"] = "FAIL"
        write_json(args.inventory, inventory)
        return 2
    inventory["operator_safe_observation"] = "YES"
    inventory["status"] = "PASS"
    inventory["final_task_result"] = "PASS"
    inventory["authority"].update({
        "j6_posvel_local_motion": "PASS",
        "j6_bidirectional_5deg": "PASS",
        "j6_local_motion_authority": "PASS",
    })
    inventory["ctrl_mode_final"] = inventory["bidirectional_5deg"]["ctrl_mode_final"]
    inventory["ctrl_mode_persistence"] = "NOT_TESTED"
    inventory["completed_at_utc"] = utc_now()
    write_json(args.inventory, inventory)
    print(json.dumps(inventory, indent=2))
    return 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "phase",
        choices=(
            "preflight", "switch-mode", "sign-probe", "record-sign",
            "repeat-sign-probe", "prepare-bidirectional", "bidirectional", "finalize",
        ),
    )
    parser.add_argument("--operator-mode-gate", default="")
    parser.add_argument("--operator-clearance", default="")
    parser.add_argument("--operator-stop", default="")
    parser.add_argument("--operator-sign", default="")
    parser.add_argument("--operator-repeat-sign", default="")
    parser.add_argument("--operator-safe", default="")
    parser.add_argument("--output-dir", type=Path, default=Path("hardware/v15_21e"))
    parser.add_argument(
        "--inventory",
        type=Path,
        default=Path("hardware/v15_21e/j6_posvel_local_motion_inventory.json"),
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    dispatch = {
        "preflight": preflight,
        "switch-mode": switch_mode,
        "sign-probe": run_sign_probe,
        "repeat-sign-probe": repeat_sign_probe,
        "record-sign": record_sign,
        "prepare-bidirectional": prepare_bidirectional,
        "bidirectional": run_bidirectional,
        "finalize": finalize,
    }
    try:
        return dispatch[args.phase](args)
    except Blocked as exc:
        print(f"BLOCKED: {exc}")
        return 4


if __name__ == "__main__":
    raise SystemExit(main())

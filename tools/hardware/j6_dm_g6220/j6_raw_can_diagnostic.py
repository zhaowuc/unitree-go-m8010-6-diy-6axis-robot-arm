#!/usr/bin/env python3
"""V15.21C raw-CAN diagnosis for the J6 DM-G6220 enable transient.

This is a diagnosis tool, not a motion commissioning tool.  The only active
phase is an operator-gated current-position preload plus a hard-deadline
micro-enable.  It implements no trajectory, gain tuning, mode/parameter/ID
write, set-zero, sign probe, or bidirectional motion path.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import statistics
import struct
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Iterable, Optional

from dm_g6220_motion_transport import pack_mit_command
from dm_g6220_transport import (
    CAN_BITRATE,
    CTRL_MODE_MIT,
    PMAX_PROTOCOL_RAD,
    TMAX_PROTOCOL,
    VMAX_PROTOCOL_RAD_S,
    _decode_feedback_payload,
    state_text,
)


SOURCE_HEAD = "1583a0f84d35e8b0fcf0849b5f737a1fc5369dd5"
WORK_BRANCH = "agent/v15-21c-j6-enable-transient-diagnosis"
EXPECTED_USB_VID = "34b7"
EXPECTED_USB_PID = "6877"
EXPECTED_USB_SERIAL = "EEE8D71AB573449FCAFE7B39BD222C75"
EXPECTED_CHANNEL = 0
EXPECTED_MOTOR_ID = 1
EXPECTED_CTRL_MODE = CTRL_MODE_MIT
TX_PERIOD_S = 0.010
KP = 10.0
KD = 2.0
TFF = 0.0
MAX_ENABLE_WINDOW_MS = 15.0
PLANNED_FIRST_FD_MS = 12.0

MICRO_CLEARANCE_GATE = "J6_DIAGNOSTIC_MICRO_ENABLE_CLEARANCE_10DEG=YES"
MICRO_STOP_GATE = "J6_DIAGNOSTIC_EMERGENCY_STOP_READY=YES"

VALID_STATES = {0, 1, 8, 9, 0xA, 0xB, 0xC, 0xD, 0xE}
PARAMETER_RIDS_UINT32 = set(range(7, 11)) | set(range(13, 17)) | {35, 36}


class Blocked(RuntimeError):
    """A read-only identity, state, or operator gate was not satisfied."""


@dataclass(frozen=True)
class RawEvent:
    event_monotonic_s: float
    event_kind: str
    sdk_timestamp: int
    direction: int
    channel: int
    can_id: int
    dlc: int
    data_length: int
    extended: bool
    canfd: bool
    rtr: bool
    brs: bool
    ack: bool
    esi: bool
    payload: bytes
    tx_label: str = ""


@dataclass(frozen=True)
class DecodedFeedback:
    state: int
    position: float
    velocity: float
    torque: float
    mos_temp: int
    coil_temp: int


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


def read_sysfs(path: Path) -> Optional[str]:
    try:
        value = path.read_text(encoding="utf-8", errors="replace").strip()
        return value or None
    except OSError:
        return None


def exact_usb_identity() -> dict:
    matches = []
    for entry in sorted(Path("/sys/bus/usb/devices").glob("*")):
        vid = read_sysfs(entry / "idVendor")
        pid = read_sysfs(entry / "idProduct")
        if vid == EXPECTED_USB_VID and pid == EXPECTED_USB_PID:
            matches.append({
                "sysfs": str(entry),
                "vid": vid,
                "pid": pid,
                "serial": read_sysfs(entry / "serial"),
                "manufacturer": read_sysfs(entry / "manufacturer"),
                "product": read_sysfs(entry / "product"),
                "busnum": read_sysfs(entry / "busnum"),
                "devnum": read_sysfs(entry / "devnum"),
            })
    if len(matches) != 1:
        raise Blocked(f"expected one DM adapter, observed {len(matches)}")
    if matches[0]["serial"] != EXPECTED_USB_SERIAL:
        raise Blocked(f"DM adapter serial changed: {matches[0]['serial']}")
    return matches[0]


def uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    return low + value / ((1 << bits) - 1) * (high - low)


def strict_decode(payload: bytes) -> DecodedFeedback:
    if len(payload) != 8:
        raise ValueError("normal feedback must be exactly 8 bytes")
    position_u = (payload[1] << 8) | payload[2]
    velocity_u = (payload[3] << 4) | (payload[4] >> 4)
    torque_u = ((payload[4] & 0x0F) << 8) | payload[5]
    return DecodedFeedback(
        state=(payload[0] >> 4) & 0x0F,
        position=uint_to_float(position_u, -PMAX_PROTOCOL_RAD, PMAX_PROTOCOL_RAD, 16),
        velocity=uint_to_float(velocity_u, -VMAX_PROTOCOL_RAD_S, VMAX_PROTOCOL_RAD_S, 12),
        torque=uint_to_float(torque_u, -TMAX_PROTOCOL, TMAX_PROTOCOL, 12),
        mos_temp=payload[6],
        coil_temp=payload[7],
    )


class RawCanLogger:
    """Capture SDK receive, sent, and error callbacks without queue clearing."""

    def __init__(self) -> None:
        from dmcan import DmCanContext
        from j6_sdk_notification_latency import runtime_context_type

        self.channel = EXPECTED_CHANNEL
        self.motor_id = EXPECTED_MOTOR_ID
        self.master_id: Optional[int] = None
        self._events: list[RawEvent] = []
        self._lock = threading.Lock()
        self.context = None
        self.device = None
        self._context_type = runtime_context_type(DmCanContext)
        self._closed = False
        self._last_send_delivered = False
        self._communication_interlock_pending = False
        self._reconnect_attempt_count = 0
        self._reconnect_success_count = 0
        self._reconnect_delay_s = 0.10
        self._next_reconnect_at = 0.0
        self._open_exact_adapter(initial=True)

    @property
    def last_send_delivered(self) -> bool:
        return self._last_send_delivered

    @property
    def reconnect_attempt_count(self) -> int:
        return self._reconnect_attempt_count

    @property
    def reconnect_success_count(self) -> int:
        return self._reconnect_success_count

    def consume_communication_interlock(self) -> bool:
        pending = self._communication_interlock_pending or self.device is None
        self._communication_interlock_pending = False
        return pending

    def _dispose_adapter(self) -> None:
        device = self.device
        self.device = None
        if device is not None:
            try:
                device.enable_channel(self.channel, False)
            except Exception:
                pass
            try:
                device.close()
            except Exception:
                pass
        context = self.context
        self.context = None
        if context is not None:
            try:
                context._ctx = None
            except Exception:
                pass

    def _open_exact_adapter(self, *, initial: bool) -> bool:
        if self._closed:
            return False
        now = time.monotonic()
        if not initial and now < self._next_reconnect_at:
            return False
        if not initial:
            self._reconnect_attempt_count += 1
        self._dispose_adapter()
        try:
            exact_usb_identity()
            context = self._context_type()
            count = int(context.find_devices())
            if count != 1:
                context._ctx = None
                raise Blocked(
                    f"expected exactly one DM adapter, observed {count}"
                )
            device = context.get_device(0)
            if not device.open():
                context._ctx = None
                raise Blocked("DM adapter open failed")
            device.enable_channel(self.channel, True)
            info = device.get_channel_baudrate(self.channel)
            if (
                info is None
                or bool(info.canfd)
                or int(info.can_baudrate) != CAN_BITRATE
            ):
                try:
                    device.enable_channel(self.channel, False)
                    device.close()
                finally:
                    context._ctx = None
                raise Blocked(
                    "adapter is not channel 0 Classic CAN at 1 Mbps"
                )
            self.context = context
            self.device = device
            device.hook_recv_callback(self._on_receive)
            device.hook_sent_callback(self._on_sent)
            device.hook_err_callback(self._on_error)
        except Exception:
            self._dispose_adapter()
            if initial:
                raise
            self._next_reconnect_at = now + self._reconnect_delay_s
            self._reconnect_delay_s = min(self._reconnect_delay_s * 2.0, 2.0)
            return False
        if not initial:
            self._reconnect_success_count += 1
            self._communication_interlock_pending = True
            self._reconnect_delay_s = 0.10
            self._next_reconnect_at = 0.0
            self._send_recovery_disable_prime()
        return True

    def _send_recovery_disable_prime(self) -> None:
        """On a new adapter epoch, transmit only DISABLE/refresh frames."""

        if self.device is None:
            return
        disable = b"\xFF" * 7 + b"\xFD"
        refresh = refresh_request()
        try:
            for _index in range(3):
                if not self.device.send_can(
                    self.channel, self.motor_id, 8, disable,
                    False, False, False, False,
                ):
                    raise RuntimeError("recovery DISABLE send failed")
            for _index in range(5):
                if not self.device.send_can(
                    self.channel, 0x7FF, 8, refresh,
                    False, False, False, False,
                ):
                    raise RuntimeError("recovery refresh send failed")
        except Exception:
            self._dispose_adapter()

    @staticmethod
    def dlc_bytes(dlc: int) -> int:
        return dlc if dlc <= 8 else {
            9: 12, 10: 16, 11: 20, 12: 24, 13: 32, 14: 48, 15: 64,
        }.get(dlc, 64)

    def _capture(self, event_kind: str, frame) -> None:
        try:
            head = frame.head
            dlc = int(head.dlc)
            length = self.dlc_bytes(dlc)
            event = RawEvent(
                event_monotonic_s=time.monotonic(),
                event_kind=event_kind,
                sdk_timestamp=int(head.timestamp),
                direction=int(head.dir),
                channel=int(head.channel),
                can_id=int(head.can_id),
                dlc=dlc,
                data_length=length,
                extended=bool(head.ext),
                canfd=bool(head.canfd),
                rtr=bool(head.rtr),
                brs=bool(head.brs),
                ack=bool(head.ack),
                esi=bool(head.esi),
                payload=bytes(frame.payload[:length]),
            )
            with self._lock:
                self._events.append(event)
        except Exception:
            return

    def _on_receive(self, _device, frame) -> None:
        self._capture("RX_CALLBACK", frame)

    def _on_sent(self, _device, frame) -> None:
        self._capture("SENT_CALLBACK", frame)

    def _on_error(self, _device, frame) -> None:
        self._capture("ERROR_CALLBACK", frame)

    def send(self, can_id: int, payload: bytes, label: str) -> float:
        if len(payload) != 8:
            raise ValueError("diagnostic sends only Classic CAN DLC 8")
        sent_at = time.monotonic()
        event = RawEvent(
            event_monotonic_s=sent_at,
            event_kind="TX_REQUEST",
            sdk_timestamp=0,
            direction=1,
            channel=self.channel,
            can_id=can_id,
            dlc=8,
            data_length=8,
            extended=False,
            canfd=False,
            rtr=False,
            brs=False,
            ack=False,
            esi=False,
            payload=payload,
            tx_label=label,
        )
        with self._lock:
            self._events.append(event)
        self._last_send_delivered = False
        if self.device is None:
            self._communication_interlock_pending = True
            self._open_exact_adapter(initial=False)
            return sent_at
        try:
            ok = self.device.send_can(
                self.channel, can_id, len(payload), payload,
                False, False, False, False,
            )
        except Exception:
            ok = False
        if not ok:
            self._communication_interlock_pending = True
            self._dispose_adapter()
            self._next_reconnect_at = 0.0
            self._open_exact_adapter(initial=False)
            # Never replay the interrupted request on a newly opened module.
            # The recovery path above emits only DISABLE/refresh frames.
            return sent_at
        self._last_send_delivered = True
        return sent_at

    def snapshot(self) -> list[RawEvent]:
        with self._lock:
            return list(self._events)

    def drain_events(self) -> tuple[RawEvent, ...]:
        """Atomically return and clear queued events for bounded long-running workers."""
        with self._lock:
            events = tuple(self._events)
            self._events.clear()
            return events

    def close(self) -> None:
        self._closed = True
        self._dispose_adapter()

    def allowed_feedback_ids(self) -> set[int]:
        ids = {self.motor_id}
        if self.master_id is not None:
            ids.add(self.master_id)
            if self.master_id == 0:
                ids.add(0)
        return ids

    def classify(self, event: RawEvent) -> str:
        if event.event_kind != "RX_CALLBACK" or event.direction != 0:
            return "UNKNOWN"
        if event.channel != self.channel or event.extended or event.canfd or event.rtr:
            return "UNKNOWN"
        payload = event.payload
        if len(payload) == 8 and int.from_bytes(payload[0:2], "little") == self.motor_id:
            if payload[2] in (0x33, 0x55):
                return "PARAM_RESPONSE"
        if (
            len(payload) == 8
            and event.can_id in self.allowed_feedback_ids()
            and (payload[0] & 0x0F) == (self.motor_id & 0x0F)
            and ((payload[0] >> 4) & 0x0F) in VALID_STATES
        ):
            return "NORMAL_FEEDBACK"
        return "UNKNOWN"


def parameter_request(rid: int) -> bytes:
    return bytes((EXPECTED_MOTOR_ID, 0, 0x33, rid & 0xFF, 0, 0, 0, 0))


def refresh_request() -> bytes:
    return bytes((EXPECTED_MOTOR_ID, 0, 0xCC, 0, 0, 0, 0, 0))


def parameter_value(payload: bytes, rid: int) -> float:
    if rid in PARAMETER_RIDS_UINT32:
        return float(int.from_bytes(payload[4:8], "little"))
    return float(struct.unpack("<f", payload[4:8])[0])


def read_parameter(logger: RawCanLogger, rid: int, timeout_s: float = 0.35) -> float:
    before = len(logger.snapshot())
    logger.send(0x7FF, parameter_request(rid), f"READ_RID_{rid}")
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        for event in logger.snapshot()[before:]:
            if (
                event.event_kind == "RX_CALLBACK"
                and event.direction == 0
                and len(event.payload) == 8
                and int.from_bytes(event.payload[0:2], "little") == EXPECTED_MOTOR_ID
                and event.payload[2] == 0x33
                and event.payload[3] == rid
            ):
                return parameter_value(event.payload, rid)
        time.sleep(0.001)
    raise Blocked(f"RID {rid} read timed out")


def normal_feedback(logger: RawCanLogger, events: Iterable[RawEvent]) -> list[tuple[RawEvent, DecodedFeedback]]:
    out = []
    for event in events:
        if logger.classify(event) == "NORMAL_FEEDBACK":
            out.append((event, strict_decode(event.payload)))
    return out


def wait_for_normal_count(
    logger: RawCanLogger,
    start_index: int,
    count: int,
    timeout_s: float,
) -> list[tuple[RawEvent, DecodedFeedback]]:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        values = normal_feedback(logger, logger.snapshot()[start_index:])
        if len(values) >= count:
            return values[:count]
        time.sleep(0.001)
    values = normal_feedback(logger, logger.snapshot()[start_index:])
    raise Blocked(f"normal feedback returned {len(values)}/{count} frames")


def capture_refresh_samples(
    logger: RawCanLogger,
    count: int,
    label: str,
) -> tuple[list[tuple[RawEvent, DecodedFeedback]], int]:
    start_index = len(logger.snapshot())
    start = time.monotonic()
    for index in range(count):
        target = start + index * TX_PERIOD_S
        remaining = target - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        logger.send(0x7FF, refresh_request(), f"{label}_{index:03d}")
    values = wait_for_normal_count(logger, start_index, count, 1.0)
    return values, start_index


CSV_FIELDS = [
    "sequence", "event_monotonic_s", "event_kind", "sdk_timestamp", "dir",
    "channel", "can_id_hex", "can_id_dec", "dlc", "data_length", "extended",
    "canfd", "rtr", "brs", "ack", "esi", "payload_hex", "classification",
    "tx_label", "state", "state_name", "position_rad", "velocity_rad_s",
    "torque_protocol", "mos_temperature_c", "coil_temperature_c",
]


def write_raw_csv(path: Path, logger: RawCanLogger, events: list[RawEvent]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
        writer.writeheader()
        for sequence, event in enumerate(events):
            classification = logger.classify(event)
            decoded = strict_decode(event.payload) if classification == "NORMAL_FEEDBACK" else None
            writer.writerow({
                "sequence": sequence,
                "event_monotonic_s": f"{event.event_monotonic_s:.9f}",
                "event_kind": event.event_kind,
                "sdk_timestamp": event.sdk_timestamp,
                "dir": event.direction,
                "channel": event.channel,
                "can_id_hex": f"0x{event.can_id:X}",
                "can_id_dec": event.can_id,
                "dlc": event.dlc,
                "data_length": event.data_length,
                "extended": int(event.extended),
                "canfd": int(event.canfd),
                "rtr": int(event.rtr),
                "brs": int(event.brs),
                "ack": int(event.ack),
                "esi": int(event.esi),
                "payload_hex": event.payload.hex().upper(),
                "classification": classification,
                "tx_label": event.tx_label,
                "state": "" if decoded is None else decoded.state,
                "state_name": "" if decoded is None else state_text(decoded.state),
                "position_rad": "" if decoded is None else f"{decoded.position:.15g}",
                "velocity_rad_s": "" if decoded is None else f"{decoded.velocity:.15g}",
                "torque_protocol": "" if decoded is None else f"{decoded.torque:.15g}",
                "mos_temperature_c": "" if decoded is None else decoded.mos_temp,
                "coil_temperature_c": "" if decoded is None else decoded.coil_temp,
            })


def write_position_csv(
    path: Path,
    values: list[tuple[RawEvent, DecodedFeedback]],
) -> None:
    fields = [
        "sample", "callback_monotonic_s", "can_id_hex", "payload_hex", "state",
        "state_name", "position_rad", "velocity_rad_s", "torque_protocol",
        "mos_temperature_c", "coil_temperature_c",
    ]
    with path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        for index, (event, decoded) in enumerate(values):
            writer.writerow({
                "sample": index,
                "callback_monotonic_s": f"{event.event_monotonic_s:.9f}",
                "can_id_hex": f"0x{event.can_id:X}",
                "payload_hex": event.payload.hex().upper(),
                "state": decoded.state,
                "state_name": state_text(decoded.state),
                "position_rad": f"{decoded.position:.15g}",
                "velocity_rad_s": f"{decoded.velocity:.15g}",
                "torque_protocol": f"{decoded.torque:.15g}",
                "mos_temperature_c": decoded.mos_temp,
                "coil_temperature_c": decoded.coil_temp,
            })


def base_inventory() -> dict:
    return {
        "schema_version": "1.0",
        "task": "V15.21C J6 enable transient root-cause diagnosis",
        "source_head": SOURCE_HEAD,
        "work_branch": WORK_BRANCH,
        "created_at_utc": utc_now(),
        "status": "NEW",
        "windows_reference_correction": {
            "windows_reference_motion": "OPERATOR_VERIFIED_PASS",
            "default_reference_motion_mode": "POS_VEL",
            "switch_mode_default_enabled": True,
            "windows_mit_kp10_kd2": "NOT_VERIFIED",
        },
        "static_audit": {
            "sdk_recv_and_sent_callbacks_are_separate": True,
            "installed_python_sdk_dir_zero_is_rx": True,
            "installed_python_sdk_dir_one_is_tx": True,
            "v15_21b_registered_callback": "RECV_ONLY",
            "v15_21b_decoder_bug_found": True,
            "decoder_bug": "CAN ID, exact DLC, and response type were not constrained",
            "decoder_bug_proven_as_v15_21b_root_cause": False,
            "official_motor_sdk_normal_feedback_mapping": "SlaveID_or_MasterID; CAN_ID_0 fallback uses payload low nibble",
            "parameter_response_signature": "payload[0:2]=SlaveID and payload[2] in {0x33,0x55}",
            "enable_reply_semantics": "official enable path reads the ordinary motor-status receive path; no distinct acknowledgement format established",
        },
        "safety": {
            "kp_kd_modified": False,
            "ctrl_mode_modified": False,
            "sign_probe_run": False,
            "bidirectional_5deg_run": False,
            "set_zero_used": False,
            "parameter_write_used": False,
            "id_write_used": False,
            "ready_pose_modified": False,
            "j1_j345_simulation_modified": False,
            "local_motion_authority": "NOT_GRANTED",
        },
        "evidence": [],
    }


def add_evidence(inventory: dict, path: Path) -> None:
    entry = {"path": str(path), "sha256": sha256_file(path)}
    inventory["evidence"] = [
        item for item in inventory.get("evidence", []) if item.get("path") != str(path)
    ] + [entry]


def hardware_readonly_preflight(logger: RawCanLogger) -> dict:
    rid7 = int(read_parameter(logger, 7))
    logger.master_id = rid7
    rid8 = int(read_parameter(logger, 8))
    rid10 = int(read_parameter(logger, 10))
    rid21 = read_parameter(logger, 21)
    if rid8 != EXPECTED_MOTOR_ID:
        raise Blocked(f"RID8 ESC_ID changed: {rid8}")
    if rid10 != EXPECTED_CTRL_MODE:
        raise Blocked(f"RID10 CTRL_MODE changed: {rid10}")
    if not math.isclose(rid21, PMAX_PROTOCOL_RAD, rel_tol=0.0, abs_tol=1e-5):
        raise Blocked(f"RID21 PMAX changed: {rid21}")
    return {
        "motor_id": EXPECTED_MOTOR_ID,
        "master_id_rid7": rid7,
        "esc_id_rid8": rid8,
        "ctrl_mode_rid10": rid10,
        "pmax_rid21": rid21,
        "allowed_normal_feedback_can_ids": sorted(logger.allowed_feedback_ids()),
    }


def baseline(args: argparse.Namespace) -> int:
    inventory = base_inventory()
    inventory["usb"] = exact_usb_identity()
    logger: Optional[RawCanLogger] = None
    try:
        logger = RawCanLogger()
        time.sleep(0.100)
        inventory["motor"] = hardware_readonly_preflight(logger)
        initial, _ = capture_refresh_samples(logger, 1, "INITIAL_DISABLED")
        if initial[0][1].state != 0:
            raise Blocked(f"initial motor state is {state_text(initial[0][1].state)}")
        values, baseline_start = capture_refresh_samples(logger, 100, "DISABLED_BASELINE")
        if any(item[1].state != 0 for item in values):
            raise Blocked("DISABLED baseline contained a non-DISABLED feedback state")

        events = logger.snapshot()
        csv_path = args.output_dir / f"j6_raw_disabled_baseline_{timestamp_token()}.csv"
        write_raw_csv(csv_path, logger, events)
        disagreements = 0
        for event, decoded in values:
            old = _decode_feedback_payload(event.can_id, event.payload, EXPECTED_MOTOR_ID)
            if old is None or any((
                old.state != decoded.state,
                old.position != decoded.position,
                old.velocity != decoded.velocity,
                old.torque != decoded.torque,
                old.mos_temp != decoded.mos_temp,
                old.coil_temp != decoded.coil_temp,
            )):
                disagreements += 1
        positions = [item[1].position for item in values]
        observed_ids = sorted({item[0].can_id for item in values})
        observed_dlcs = sorted({item[0].data_length for item in values})
        classifications: dict[str, int] = {}
        for event in events[baseline_start:]:
            key = logger.classify(event)
            classifications[key] = classifications.get(key, 0) + 1
        inventory["disabled_baseline"] = {
            "status": "PASS",
            "valid_frames": len(values),
            "state": "DISABLED",
            "normal_feedback_observed_can_ids": observed_ids,
            "normal_feedback_observed_dlc_bytes": observed_dlcs,
            "strict_vs_v15_21a_decoder_disagreements": disagreements,
            "q_median_rad": statistics.median(positions),
            "q_mean_rad": statistics.fmean(positions),
            "q_std_rad": statistics.pstdev(positions),
            "q_min_rad": min(positions),
            "q_max_rad": max(positions),
            "classifications_after_baseline_start": classifications,
            "csv_path": str(csv_path),
        }
        if len(values) != 100 or disagreements != 0:
            raise Blocked("DISABLED raw baseline acceptance failed")
        inventory["status"] = "DISABLED_BASELINE_PASS_AWAITING_OPERATOR_MICRO_ENABLE_GATE"
        add_evidence(inventory, csv_path)
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, ensure_ascii=False, indent=2))
        return 0
    except Blocked as exc:
        inventory["status"] = "DIAGNOSIS_INCONCLUSIVE"
        inventory["block_reason"] = str(exc)
        if logger is not None:
            path = args.output_dir / f"j6_raw_disabled_baseline_failed_{timestamp_token()}.csv"
            write_raw_csv(path, logger, logger.snapshot())
            add_evidence(inventory, path)
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, ensure_ascii=False, indent=2))
        return 4
    finally:
        if logger is not None:
            logger.close()


def require_micro_gates(args: argparse.Namespace) -> None:
    if args.operator_clearance != MICRO_CLEARANCE_GATE:
        raise Blocked(f"require --operator-clearance '{MICRO_CLEARANCE_GATE}'")
    if args.operator_stop != MICRO_STOP_GATE:
        raise Blocked(f"require --operator-stop '{MICRO_STOP_GATE}'")


def busy_wait_until(target: float) -> None:
    while time.monotonic() < target:
        pass


def micro_enable(args: argparse.Namespace) -> int:
    require_micro_gates(args)
    inventory = json.loads(args.inventory.read_text(encoding="utf-8"))
    initial_status = inventory.get("status")
    first_attempt = initial_status == "DISABLED_BASELINE_PASS_AWAITING_OPERATOR_MICRO_ENABLE_GATE"
    explicit_repeat = (
        initial_status == "AWAITING_OPERATOR_PHYSICAL_OBSERVATION"
        and args.operator_repeat_authorized
    )
    if not (first_attempt or explicit_repeat):
        raise Blocked(f"baseline status does not authorize micro-enable: {inventory.get('status')}")
    if explicit_repeat:
        previous = inventory.get("micro_enable")
        if previous:
            inventory.setdefault("micro_enable_attempts", []).append(previous)
    logger: Optional[RawCanLogger] = None
    first_fd_sent = False
    try:
        exact_usb_identity()
        logger = RawCanLogger()
        time.sleep(0.100)
        motor = hardware_readonly_preflight(logger)
        session_values, _session_start = capture_refresh_samples(logger, 100, "SESSION")
        if any(item[1].state != 0 for item in session_values):
            raise Blocked("session capture was not entirely DISABLED")
        session_positions = [item[1].position for item in session_values]
        q_session = statistics.median(session_positions)
        hold_payload = pack_mit_command(q_session, 0.0, KP, KD, TFF)

        logger.send(EXPECTED_MOTOR_ID, hold_payload, "PRELOAD_MIT_DISABLED")
        fc_at = logger.send(EXPECTED_MOTOR_ID, b"\xFF" * 7 + b"\xFC", "FC_ENABLE")
        logger.send(EXPECTED_MOTOR_ID, hold_payload, "MIT_HOLD_IMMEDIATE")
        busy_wait_until(fc_at + 0.010)
        logger.send(EXPECTED_MOTOR_ID, hold_payload, "MIT_HOLD_10MS")
        busy_wait_until(fc_at + PLANNED_FIRST_FD_MS / 1000.0)
        first_fd_at = logger.send(EXPECTED_MOTOR_ID, b"\xFF" * 7 + b"\xFD", "FD_DISABLE_HARD_DEADLINE")
        first_fd_sent = True
        enabled_window_ms = (first_fd_at - fc_at) * 1000.0
        time.sleep(0.020)
        logger.send(EXPECTED_MOTOR_ID, b"\xFF" * 7 + b"\xFD", "FD_DISABLE_REPEAT_2")
        time.sleep(0.020)
        logger.send(EXPECTED_MOTOR_ID, b"\xFF" * 7 + b"\xFD", "FD_DISABLE_REPEAT_3")
        remaining_log = fc_at + 0.315 - time.monotonic()
        if remaining_log > 0:
            time.sleep(remaining_log)

        final_values, _ = capture_refresh_samples(logger, 5, "FINAL_DISABLED_VERIFY")
        final_disabled = len(final_values) == 5 and all(item[1].state == 0 for item in final_values)
        post_values, _ = capture_refresh_samples(logger, 50, "POST_DISABLE_POSITION")
        if any(item[1].state != 0 for item in post_values):
            final_disabled = False

        all_events = logger.snapshot()
        raw_path = args.output_dir / f"j6_micro_enable_raw_{timestamp_token()}.csv"
        post_path = args.output_dir / f"j6_post_disable_position_{timestamp_token()}.csv"
        write_raw_csv(raw_path, logger, all_events)
        write_position_csv(post_path, post_values)

        active_feedback = [
            (event, decoded) for event, decoded in normal_feedback(logger, all_events)
            if event.event_monotonic_s >= fc_at and decoded.state == 1
        ]
        first_active = active_feedback[0] if active_feedback else None
        post_positions = [item[1].position for item in post_values]
        q_post = statistics.median(post_positions)
        inventory["micro_enable"] = {
            "status": "AWAITING_OPERATOR_PHYSICAL_OBSERVATION",
            "attempt_number": len(inventory.get("micro_enable_attempts", [])) + 1,
            "explicit_operator_repeat_after_missed_observation": explicit_repeat,
            "operator_gate": "PASS",
            "motor": motor,
            "session": {
                "valid_frames": len(session_values),
                "q_session_median_rad": q_session,
                "q_session_std_rad": statistics.pstdev(session_positions),
                "semantics": "SESSION_LOCAL_ONLY",
            },
            "preload_mit_sent_while_disabled": True,
            "fc_enable_sent": True,
            "first_fd_disable_sent": first_fd_sent,
            "actual_enabled_window_ms": enabled_window_ms,
            "maximum_allowed_enabled_window_ms": MAX_ENABLE_WINDOW_MS,
            "enabled_window_acceptance": "PASS" if enabled_window_ms <= MAX_ENABLE_WINDOW_MS else "FAIL",
            "final_five_frame_disabled": "PASS" if final_disabled else "FAIL",
            "first_active_feedback": None if first_active is None else {
                "callback_monotonic_s": first_active[0].event_monotonic_s,
                "sdk_timestamp": first_active[0].sdk_timestamp,
                "can_id": first_active[0].can_id,
                "dlc": first_active[0].data_length,
                "payload_hex": first_active[0].payload.hex().upper(),
                "classification": logger.classify(first_active[0]),
                "state": first_active[1].state,
                "position_rad": first_active[1].position,
                "velocity_rad_s": first_active[1].velocity,
                "torque_protocol": first_active[1].torque,
                "mos_temperature_c": first_active[1].mos_temp,
                "coil_temperature_c": first_active[1].coil_temp,
            },
            "active_normal_feedback_count": len(active_feedback),
            "q_post_median_rad": q_post,
            "q_post_minus_q_session_rad": q_post - q_session,
            "q_post_minus_q_session_deg": math.degrees(q_post - q_session),
            "operator_visible_jerk": "PENDING",
            "raw_csv_path": str(raw_path),
            "post_csv_path": str(post_path),
        }
        add_evidence(inventory, raw_path)
        add_evidence(inventory, post_path)
        if not final_disabled or enabled_window_ms > MAX_ENABLE_WINDOW_MS:
            inventory["status"] = "DIAGNOSIS_INCONCLUSIVE"
        else:
            inventory["status"] = "AWAITING_OPERATOR_PHYSICAL_OBSERVATION"
        write_json(args.inventory, inventory)
        print(json.dumps(inventory, ensure_ascii=False, indent=2))
        return 0 if inventory["status"] == "AWAITING_OPERATOR_PHYSICAL_OBSERVATION" else 2
    finally:
        if logger is not None:
            if not first_fd_sent:
                try:
                    for index in range(3):
                        logger.send(
                            EXPECTED_MOTOR_ID,
                            b"\xFF" * 7 + b"\xFD",
                            f"FD_DISABLE_FINALLY_{index + 1}",
                        )
                        if index < 2:
                            time.sleep(0.020)
                except Exception:
                    pass
            logger.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("phase", choices=("baseline", "micro-enable"))
    parser.add_argument("--output-dir", type=Path, default=Path("hardware/v15_21c"))
    parser.add_argument(
        "--inventory",
        type=Path,
        default=Path("hardware/v15_21c/j6_enable_transient_diagnosis.json"),
    )
    parser.add_argument("--operator-clearance", default="")
    parser.add_argument("--operator-stop", default="")
    parser.add_argument("--operator-repeat-authorized", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.phase == "baseline":
        return baseline(args)
    try:
        return micro_enable(args)
    except Blocked as exc:
        print(f"BLOCKED: {exc}")
        return 4


if __name__ == "__main__":
    raise SystemExit(main())

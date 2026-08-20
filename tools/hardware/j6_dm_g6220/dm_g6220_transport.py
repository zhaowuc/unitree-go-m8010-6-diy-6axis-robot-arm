#!/usr/bin/env python3
"""Minimal DM-G6220 transport for V15.21A read-only commissioning.

The protocol details are preserved from the operator-verified Windows source.
This module intentionally has no enable, motion-control, mode-switch, parameter-
write, ID-write, or set-zero API.  The only state-changing operation exposed is
``send_safety_disable`` (0xFD), which the probe may use only after feedback has
shown that the motor is enabled.
"""

from __future__ import annotations

import math
import struct
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Deque, List, Optional, Tuple


PMAX_PROTOCOL_RAD = 12.5
VMAX_PROTOCOL_RAD_S = 45.0
TMAX_PROTOCOL = 10.0
SERIAL_BRIDGE_BAUD = 921_600
CAN_BITRATE = 1_000_000
CTRL_MODE_MIT = 1
CTRL_MODE_POS_VEL = 2
CTRL_MODE_VEL = 3


class TransportError(RuntimeError):
    """A transport could not be opened or used safely."""


@dataclass(frozen=True)
class ParameterResponse:
    requested_id: int
    slave_id: int
    rid: int
    value: float
    wrapper_can_id: int


@dataclass(frozen=True)
class Feedback:
    can_id: int
    state: int
    position: float
    velocity: float
    torque: float
    mos_temp: int
    coil_temp: int


def ctrl_mode_text(value: int) -> str:
    return {
        CTRL_MODE_MIT: "MIT",
        CTRL_MODE_POS_VEL: "POS_VEL",
        CTRL_MODE_VEL: "VEL",
    }.get(value, f"UNKNOWN_{value}")


def state_text(value: int) -> str:
    return {
        0: "DISABLED",
        1: "ENABLED",
        8: "OVER_VOLTAGE",
        9: "UNDER_VOLTAGE",
        0xA: "OVER_CURRENT",
        0xB: "MOS_OVER_TEMP",
        0xC: "COIL_OVER_TEMP",
        0xD: "LOST_COMM",
        0xE: "OVERLOAD",
    }.get(value, f"UNKNOWN_0x{value:X}")


def _uint_to_float(value: int, low: float, high: float, bits: int) -> float:
    return low + (value / ((1 << bits) - 1)) * (high - low)


def _decode_feedback_payload(can_id: int, data: bytes, motor_id: int) -> Optional[Feedback]:
    if len(data) < 6 or (data[0] & 0x0F) != (motor_id & 0x0F):
        return None
    position_u = (data[1] << 8) | data[2]
    velocity_u = (data[3] << 4) | (data[4] >> 4)
    torque_u = ((data[4] & 0x0F) << 8) | data[5]
    feedback = Feedback(
        can_id=can_id,
        state=(data[0] >> 4) & 0x0F,
        position=_uint_to_float(position_u, -PMAX_PROTOCOL_RAD, PMAX_PROTOCOL_RAD, 16),
        velocity=_uint_to_float(velocity_u, -VMAX_PROTOCOL_RAD_S, VMAX_PROTOCOL_RAD_S, 12),
        torque=_uint_to_float(torque_u, -TMAX_PROTOCOL, TMAX_PROTOCOL, 12),
        mos_temp=data[6] if len(data) > 6 else 0,
        coil_temp=data[7] if len(data) > 7 else 0,
    )
    numeric = (feedback.position, feedback.velocity, feedback.torque)
    return feedback if all(math.isfinite(value) for value in numeric) else None


def _decode_parameter_payload(
    wrapper_can_id: int, data: bytes, requested_id: int, rid: int
) -> Optional[ParameterResponse]:
    if len(data) != 8 or data[2] != 0x33 or data[3] != (rid & 0xFF):
        return None
    slave_id = int.from_bytes(data[0:2], "little")
    if slave_id != requested_id:
        return None
    if 7 <= rid <= 10 or 13 <= rid <= 16 or 35 <= rid <= 36:
        value = float(int.from_bytes(data[4:8], "little"))
    else:
        value = float(struct.unpack("<f", data[4:8])[0])
    return ParameterResponse(requested_id, slave_id, rid, value, wrapper_can_id)


def _parameter_request(motor_id: int, rid: int) -> bytes:
    return bytes((motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0x33, rid & 0xFF, 0, 0, 0, 0))


def _refresh_request(motor_id: int) -> bytes:
    return bytes((motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0))


class SerialBridgeReadOnlyTransport:
    """Old official 921600-baud USB-CAN serial bridge (29-byte/16-byte framing)."""

    kind = "official_serial_bridge"

    def __init__(self, device: str, timeout: float = 0.03):
        import serial

        self.device = device
        self.channel = 0
        self.can_bitrate = CAN_BITRATE
        self.serial_baud = SERIAL_BRIDGE_BAUD
        self._serial = serial.Serial(device, SERIAL_BRIDGE_BAUD, timeout=timeout)
        self._rx_remainder = b""
        self._tx_template = bytearray(
            (0x55, 0xAA, 0x1E, 0x03, 0x01, 0x00, 0x00, 0x00,
             0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00)
        )

    def close(self) -> None:
        if self._serial.is_open:
            self._serial.close()

    def _send_can(self, can_id: int, data: bytes) -> None:
        if len(data) != 8:
            raise ValueError("Classic CAN data must be exactly 8 bytes")
        frame = bytearray(self._tx_template)
        frame[13] = can_id & 0xFF
        frame[14] = (can_id >> 8) & 0xFF
        frame[21:29] = data
        written = self._serial.write(frame)
        self._serial.flush()
        if written != len(frame):
            raise TransportError(f"short serial write: {written}/{len(frame)}")

    def _extract_packets(self) -> List[bytes]:
        data = self._rx_remainder + self._serial.read_all()
        packets: List[bytes] = []
        index = 0
        while index <= len(data) - 16:
            if data[index] == 0xAA and data[index + 15] == 0x55:
                packets.append(bytes(data[index:index + 16]))
                index += 16
            else:
                index += 1
        self._rx_remainder = data[index:]
        return packets

    def _clear_input(self) -> None:
        self._rx_remainder = b""
        self._serial.reset_input_buffer()

    def read_parameter(self, motor_id: int, rid: int, timeout: float = 0.12) -> Optional[ParameterResponse]:
        self._clear_input()
        self._send_can(0x7FF, _parameter_request(motor_id, rid))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for packet in self._extract_packets():
                if len(packet) != 16 or packet[0] != 0xAA or packet[1] != 0x11 or packet[15] != 0x55:
                    continue
                wrapper_can_id = int.from_bytes(packet[3:7], "little")
                response = _decode_parameter_payload(wrapper_can_id, packet[7:15], motor_id, rid)
                if response is not None:
                    return response
            time.sleep(0.001)
        return None

    def refresh_feedback(self, motor_id: int, timeout: float = 0.08) -> Optional[Feedback]:
        self._clear_input()
        self.send_refresh_request(motor_id)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            feedback = self.drain_feedback(motor_id)
            if feedback:
                return feedback[0][1]
            time.sleep(0.001)
        return None

    def clear_pending_feedback(self) -> None:
        self._clear_input()

    def send_refresh_request(self, motor_id: int) -> None:
        self._send_can(0x7FF, _refresh_request(motor_id))

    def drain_feedback(self, motor_id: int) -> List[Tuple[float, Feedback]]:
        received_at = time.monotonic()
        out: List[Tuple[float, Feedback]] = []
        for packet in self._extract_packets():
            if len(packet) != 16 or packet[0] != 0xAA or packet[1] != 0x11 or packet[15] != 0x55:
                continue
            wrapper_can_id = int.from_bytes(packet[3:7], "little")
            feedback = _decode_feedback_payload(wrapper_can_id, packet[7:15], motor_id)
            if feedback is not None:
                out.append((received_at, feedback))
        return out

    def send_safety_disable(self, motor_id: int) -> None:
        payload = b"\xFF" * 7 + b"\xFD"
        for _ in range(3):
            self._send_can(motor_id, payload)
            time.sleep(0.02)


class DmCanSdkReadOnlyTransport:
    """Official dmcan SDK backend for DM-USB2FDCAN/USB2CANFD adapters."""

    kind = "dmcan_sdk"

    def __init__(self, channel: int = 0, can_bitrate: int = CAN_BITRATE):
        try:
            from dmcan import DmCanContext
        except Exception as exc:
            raise TransportError("official dmcan SDK is not importable") from exc

        self.device = "DM-USB2FDCAN"
        self.channel = channel
        self.can_bitrate = can_bitrate
        self._rx: Deque[Tuple[float, int, bytes, bool, bool]] = deque(maxlen=2048)
        self._rx_lock = threading.Lock()
        self.context = DmCanContext()
        count = int(self.context.find_devices())
        if count != 1:
            self.context._ctx = None
            raise TransportError(f"expected exactly one DM adapter, found {count}")
        self._device = self.context.get_device(0)
        if not self._device.open():
            self.context._ctx = None
            raise TransportError("DM adapter open failed")
        self._device.enable_channel(channel, True)
        info = self._device.get_channel_baudrate(channel)
        if info is None:
            self.close()
            raise TransportError("cannot read DM adapter channel configuration")
        if bool(info.canfd) or int(info.can_baudrate) != can_bitrate:
            observed = f"canfd={bool(info.canfd)}, bitrate={int(info.can_baudrate)}"
            self.close()
            raise TransportError(
                "read-only transport refuses to rewrite adapter configuration; "
                f"expected Classic CAN at {can_bitrate}, observed {observed}"
            )
        self._device.hook_recv_callback(self._on_receive)

    @staticmethod
    def _dlc_bytes(dlc: int) -> int:
        return dlc if dlc <= 8 else {9: 12, 10: 16, 11: 20, 12: 24, 13: 32, 14: 48, 15: 64}.get(dlc, 64)

    def _on_receive(self, _device, frame) -> None:
        try:
            if int(frame.head.dir) != 0 or int(frame.head.channel) != self.channel:
                return
            length = self._dlc_bytes(int(frame.head.dlc))
            item = (
                time.monotonic(),
                int(frame.head.can_id),
                bytes(frame.payload[:length]),
                bool(frame.head.ext),
                bool(frame.head.canfd),
            )
            with self._rx_lock:
                self._rx.append(item)
        except Exception:
            return

    def _clear_rx(self) -> None:
        with self._rx_lock:
            self._rx.clear()

    def _pop_rx(self) -> List[Tuple[float, int, bytes, bool, bool]]:
        with self._rx_lock:
            frames = list(self._rx)
            self._rx.clear()
        return frames

    def _send_can(self, can_id: int, data: bytes) -> None:
        ok = self._device.send_can(self.channel, can_id, len(data), data, False, False, False, False)
        if not ok:
            raise TransportError(f"Classic CAN send failed for ID 0x{can_id:X}")

    def read_parameter(self, motor_id: int, rid: int, timeout: float = 0.12) -> Optional[ParameterResponse]:
        self._clear_rx()
        self._send_can(0x7FF, _parameter_request(motor_id, rid))
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for _received_at, can_id, data, is_extended, is_fd in self._pop_rx():
                if is_extended or is_fd:
                    continue
                response = _decode_parameter_payload(can_id, data, motor_id, rid)
                if response is not None:
                    return response
            time.sleep(0.001)
        return None

    def refresh_feedback(self, motor_id: int, timeout: float = 0.08) -> Optional[Feedback]:
        self._clear_rx()
        self.send_refresh_request(motor_id)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            feedback = self.drain_feedback(motor_id)
            if feedback:
                return feedback[0][1]
            time.sleep(0.001)
        return None

    def clear_pending_feedback(self) -> None:
        self._clear_rx()

    def send_refresh_request(self, motor_id: int) -> None:
        self._send_can(0x7FF, _refresh_request(motor_id))

    def drain_feedback(self, motor_id: int) -> List[Tuple[float, Feedback]]:
        out: List[Tuple[float, Feedback]] = []
        for received_at, can_id, data, is_extended, is_fd in self._pop_rx():
            if is_extended or is_fd:
                continue
            feedback = _decode_feedback_payload(can_id, data, motor_id)
            if feedback is not None:
                out.append((received_at, feedback))
        return out

    def send_safety_disable(self, motor_id: int) -> None:
        payload = b"\xFF" * 7 + b"\xFD"
        for _ in range(3):
            self._send_can(motor_id, payload)
            time.sleep(0.02)

    def close(self) -> None:
        try:
            self._device.enable_channel(self.channel, False)
        except Exception:
            pass
        try:
            self._device.close()
        except Exception:
            pass
        try:
            self.context._ctx = None
        except Exception:
            pass


def create_transport(kind: str, device: str = "", channel: int = 0):
    if kind == "dmcan_sdk":
        return DmCanSdkReadOnlyTransport(channel=channel, can_bitrate=CAN_BITRATE)
    if kind == "serial_bridge":
        if not device:
            raise TransportError("--device is required for serial_bridge")
        is_unitree = "FTASQA6F" in device
        try:
            from serial.tools import list_ports

            requested = Path(device).resolve()
            for item in list_ports.comports():
                if Path(item.device).resolve() != requested:
                    continue
                serial_number = (item.serial_number or "").upper()
                is_unitree = (
                    item.vid == 0x0403
                    and item.pid == 0x6011
                    and serial_number == "FTASQA6F"
                )
                break
        except Exception:
            pass
        if is_unitree:
            raise TransportError("refusing known Unitree FT4232H path")
        return SerialBridgeReadOnlyTransport(device)
    raise TransportError(f"unsupported verified transport: {kind}")

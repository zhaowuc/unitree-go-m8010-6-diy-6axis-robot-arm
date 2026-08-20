#!/usr/bin/env python3
"""Narrow DM-G6220 MIT motion transport for V15.21B.

This module is deliberately separate from the frozen V15.21A read-only
transport.  Its public motor operations are limited to FC enable, FD disable,
MIT command transmission, and asynchronous feedback draining.  It implements
no parameter write, mode switch, ID change, set-zero, POS_VEL, or VEL path.
"""

from __future__ import annotations

import math
import threading
import time
from collections import deque
from dataclasses import dataclass
from typing import Deque, List, Tuple

from dm_g6220_transport import (
    CAN_BITRATE,
    Feedback,
    PMAX_PROTOCOL_RAD,
    TMAX_PROTOCOL,
    TransportError,
    VMAX_PROTOCOL_RAD_S,
    _decode_feedback_payload,
)


KP_PROTOCOL_MAX = 500.0
KD_PROTOCOL_MAX = 5.0


def _clamp(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _float_to_uint(value: float, low: float, high: float, bits: int) -> int:
    """Byte-exact quantizer used by the operator-verified Windows source."""
    value = _clamp(value, low, high)
    return int((value - low) * ((1 << bits) - 1) / (high - low))


def pack_mit_command(
    position: float,
    velocity: float,
    kp: float,
    kd: float,
    torque_ff: float,
) -> bytes:
    """Pack the verified DM 16/12/12/12/12-bit MIT command layout."""
    values = (position, velocity, kp, kd, torque_ff)
    if not all(math.isfinite(value) for value in values):
        raise ValueError("MIT command values must be finite")
    if not -PMAX_PROTOCOL_RAD <= position <= PMAX_PROTOCOL_RAD:
        raise ValueError("position is outside the DM protocol encoding range")
    if not -VMAX_PROTOCOL_RAD_S <= velocity <= VMAX_PROTOCOL_RAD_S:
        raise ValueError("velocity is outside the DM protocol encoding range")
    if not 0.0 <= kp <= KP_PROTOCOL_MAX:
        raise ValueError("Kp is outside the DM protocol encoding range")
    if not 0.0 <= kd <= KD_PROTOCOL_MAX:
        raise ValueError("Kd is outside the DM protocol encoding range")
    if not -TMAX_PROTOCOL <= torque_ff <= TMAX_PROTOCOL:
        raise ValueError("torque_ff is outside the DM protocol encoding range")

    q_u = _float_to_uint(position, -PMAX_PROTOCOL_RAD, PMAX_PROTOCOL_RAD, 16)
    dq_u = _float_to_uint(velocity, -VMAX_PROTOCOL_RAD_S, VMAX_PROTOCOL_RAD_S, 12)
    kp_u = _float_to_uint(kp, 0.0, KP_PROTOCOL_MAX, 12)
    kd_u = _float_to_uint(kd, 0.0, KD_PROTOCOL_MAX, 12)
    tau_u = _float_to_uint(torque_ff, -TMAX_PROTOCOL, TMAX_PROTOCOL, 12)
    return bytes((
        (q_u >> 8) & 0xFF,
        q_u & 0xFF,
        dq_u >> 4,
        ((dq_u & 0x0F) << 4) | ((kp_u >> 8) & 0x0F),
        kp_u & 0xFF,
        kd_u >> 4,
        ((kd_u & 0x0F) << 4) | ((tau_u >> 8) & 0x0F),
        tau_u & 0xFF,
    ))


@dataclass(frozen=True)
class TimedFeedback:
    callback_monotonic_s: float
    feedback: Feedback


class DmG6220MitMotionTransport:
    """Fail-closed native dmcan transport with a deliberately tiny API."""

    def __init__(self, motor_id: int, channel: int = 0, can_bitrate: int = CAN_BITRATE):
        if motor_id <= 0:
            raise ValueError("motor_id must be positive")
        if channel != 0:
            raise TransportError("V15.21B is frozen to DM CAN channel 0")
        if can_bitrate != CAN_BITRATE:
            raise TransportError("V15.21B is frozen to Classic CAN at 1 Mbps")
        try:
            from dmcan import DmCanContext
        except Exception as exc:
            raise TransportError("official dmcan SDK is not importable") from exc

        self.motor_id = motor_id
        self.channel = channel
        self.can_bitrate = can_bitrate
        self.device_name = "DM-USB2FDCAN"
        self._rx: Deque[Tuple[float, int, bytes, bool, bool]] = deque(maxlen=4096)
        self._rx_lock = threading.Lock()
        self.context = DmCanContext()
        self._device = None
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
                "motion transport refuses to rewrite adapter configuration; "
                f"expected Classic CAN at {can_bitrate}, observed {observed}"
            )
        self._device.hook_recv_callback(self._on_receive)

    @staticmethod
    def _dlc_bytes(dlc: int) -> int:
        if dlc <= 8:
            return dlc
        return {9: 12, 10: 16, 11: 20, 12: 24, 13: 32, 14: 48, 15: 64}.get(dlc, 64)

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

    def _send_classic_can(self, can_id: int, data: bytes) -> None:
        if len(data) != 8:
            raise ValueError("Classic CAN payload must be exactly 8 bytes")
        if self._device is None:
            raise TransportError("DM adapter is closed")
        sent = self._device.send_can(
            self.channel, can_id, len(data), data, False, False, False, False
        )
        if not sent:
            raise TransportError(f"Classic CAN send failed for ID 0x{can_id:X}")

    def clear_feedback(self) -> None:
        with self._rx_lock:
            self._rx.clear()

    def drain_feedback(self) -> List[TimedFeedback]:
        with self._rx_lock:
            raw = list(self._rx)
            self._rx.clear()
        out: List[TimedFeedback] = []
        for received_at, can_id, data, is_extended, is_fd in raw:
            if is_extended or is_fd:
                continue
            feedback = _decode_feedback_payload(can_id, data, self.motor_id)
            if feedback is not None:
                out.append(TimedFeedback(received_at, feedback))
        return out

    def send_enable(self) -> None:
        # A single verified FC command avoids leaving the motor enabled without
        # an immediate hold target.  V15.21B does not auto-retry enable.
        self._send_classic_can(self.motor_id, b"\xFF" * 7 + b"\xFC")

    def send_disable(self) -> None:
        # Three FD frames are one fail-closed action, matching the known-good
        # reference.  They are not a motion retry.
        payload = b"\xFF" * 7 + b"\xFD"
        for index in range(3):
            self._send_classic_can(self.motor_id, payload)
            if index < 2:
                time.sleep(0.02)

    def send_mit_command(
        self,
        position: float,
        velocity: float,
        kp: float,
        kd: float,
        torque_ff: float,
    ) -> None:
        payload = pack_mit_command(position, velocity, kp, kd, torque_ff)
        self._send_classic_can(self.motor_id, payload)

    def close(self) -> None:
        if self._device is not None:
            try:
                self._device.enable_channel(self.channel, False)
            except Exception:
                pass
            try:
                self._device.close()
            except Exception:
                pass
            self._device = None
        try:
            self.context._ctx = None
        except Exception:
            pass

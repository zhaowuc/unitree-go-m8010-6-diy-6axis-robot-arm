#!/usr/bin/env python3
"""Restricted POS_VEL transport for the J6 DM-G6220.

The public API intentionally contains no MIT command, parameter write,
mode-switch, set-zero, or ID-changing path.
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

from j6_raw_can_diagnostic import (
    DecodedFeedback,
    RawCanLogger,
    RawEvent,
    strict_decode,
)


MOTOR_ID = 1
POS_VEL_COMMAND_CAN_ID = 0x100 + MOTOR_ID


@dataclass(frozen=True)
class StrictFeedback:
    event: RawEvent
    decoded: DecodedFeedback


@dataclass(frozen=True)
class DrainResult:
    raw_events: tuple[RawEvent, ...]
    normal_feedback: tuple[StrictFeedback, ...]


class DmG6220PosVelTransport:
    """Small allowlisted transport for POS_VEL enable/hold diagnosis."""

    def __init__(self, master_id: int, logger: RawCanLogger | None = None) -> None:
        self._logger = RawCanLogger() if logger is None else logger
        self._logger.master_id = int(master_id)

    def send_enable(self) -> float:
        return self._logger.send(MOTOR_ID, b"\xFF" * 7 + b"\xFC", "FC_ENABLE")

    def send_disable(self, label: str = "FD_DISABLE") -> float:
        return self._logger.send(MOTOR_ID, b"\xFF" * 7 + b"\xFD", label)

    def send_pos_vel_command(
        self,
        position_rad: float,
        velocity_rad_s: float,
        label: str,
    ) -> tuple[float, bytes]:
        payload = struct.pack("<ff", float(position_rad), float(velocity_rad_s))
        sent_at = self._logger.send(POS_VEL_COMMAND_CAN_ID, payload, label)
        return sent_at, payload

    def strict_feedback_drain(self, start_index: int = 0) -> DrainResult:
        raw = tuple(self._logger.snapshot()[start_index:])
        normal = tuple(
            StrictFeedback(event=event, decoded=strict_decode(event.payload))
            for event in raw
            if self._logger.classify(event) == "NORMAL_FEEDBACK"
        )
        return DrainResult(raw_events=raw, normal_feedback=normal)

    def _raw_logger_for_commissioning(self) -> RawCanLogger:
        """Private bridge used only by the gated V15.21D evidence harness."""
        return self._logger

    def close(self) -> None:
        self._logger.close()

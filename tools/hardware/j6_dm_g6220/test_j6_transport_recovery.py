from __future__ import annotations

from types import SimpleNamespace
import threading

from dm_g6220_posvel_transport import DmG6220PosVelTransport
from j6_raw_can_diagnostic import RawCanLogger


class FakeDevice:
    def __init__(self, result: bool) -> None:
        self.result = result
        self.sent = []
        self.closed = False

    def send_can(self, *args):
        self.sent.append(args)
        return self.result

    def enable_channel(self, *_args) -> None:
        return None

    def close(self) -> None:
        self.closed = True


def make_logger(device: FakeDevice | None) -> RawCanLogger:
    logger = RawCanLogger.__new__(RawCanLogger)
    logger.channel = 0
    logger.motor_id = 1
    logger.master_id = 0
    logger._events = []
    logger._lock = threading.Lock()
    logger.context = SimpleNamespace(_ctx=object())
    logger.device = device
    logger._closed = False
    logger._last_send_delivered = False
    logger._communication_interlock_pending = False
    logger._reconnect_attempt_count = 0
    logger._reconnect_success_count = 0
    logger._reconnect_delay_s = 0.10
    logger._next_reconnect_at = 0.0
    logger._open_exact_adapter = lambda *, initial: False
    return logger


def test_failed_active_send_is_not_replayed_and_latches_interlock() -> None:
    device = FakeDevice(False)
    logger = make_logger(device)

    logger.send(0x101, b"\x00" * 8, "ACTIVE_POSITION")

    assert len(device.sent) == 1
    assert device.closed is True
    assert logger.last_send_delivered is False
    assert logger.consume_communication_interlock() is True
    assert logger.consume_communication_interlock() is True  # still unavailable


def test_successful_send_does_not_latch_recovery() -> None:
    device = FakeDevice(True)
    logger = make_logger(device)

    logger.send(0x7FF, b"\x00" * 8, "DISABLED_REFRESH")

    assert len(device.sent) == 1
    assert logger.last_send_delivered is True
    assert logger.consume_communication_interlock() is False


def test_transport_surfaces_adapter_epoch_without_replaying_command() -> None:
    logger = make_logger(None)
    transport = DmG6220PosVelTransport(master_id=0, logger=logger)

    transport.send_pos_vel_command(1.0, 0.2, "POSITION")

    assert transport.consume_communication_interlock() is True
    assert transport.reconnect_attempt_count == 0
    assert transport.reconnect_success_count == 0


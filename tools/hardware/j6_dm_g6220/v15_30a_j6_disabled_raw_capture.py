#!/usr/bin/env python3
"""Capture a J6 software-zero candidate using DISABLED refresh feedback only.

The hardware path is deliberately smaller than the normal J6 worker.  It may
send only the fixed Classic-CAN refresh request and accepts only normal J6
feedback.  It has no RID read/write, mode switch, FC/FD, set-zero, flash, or
position/velocity/torque command path.

The resulting immutable JSON is statistics evidence, not permission to rebase
the software zero.  A separate offline, hash-gated transaction must consume a
PASS document before any reference file is changed.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import math
import os
import re
import secrets
import socket
import statistics
import time
import uuid
from contextlib import contextmanager
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Callable, ContextManager

try:
    import fcntl
except ImportError:  # pragma: no cover - production is Linux; tests inject a lock.
    fcntl = None

from j6_raw_can_diagnostic import (
    Blocked,
    EXPECTED_CHANNEL,
    EXPECTED_MOTOR_ID,
    EXPECTED_USB_PID,
    EXPECTED_USB_SERIAL,
    EXPECTED_USB_VID,
    PMAX_PROTOCOL_RAD,
    RawCanLogger,
    exact_usb_identity,
    normal_feedback,
    refresh_request,
    state_text,
    TX_PERIOD_S,
)


CAPTURE_SCHEMA = "go-m8010-j6-disabled-raw-capture-statistics/1.0"
FEEDBACK_HANDOFF_SCHEMA = "go-m8010-j6-feedback-handoff/1.0"
CONFIRM_GATE = "V15_30A_J6_DISABLED_RAW_CAPTURE=YES"
PHYSICAL_GATE = (
    "J6_24V_ON=YES;SUPPORT_RELIABLE=YES;"
    "WHOLE_ARM_VERTICAL_INITIALIZATION_POSE=YES;ARM_STATIONARY=YES;"
    "NOT_AT_MECHANICAL_LIMIT=YES"
)
DEVICE_LOCK_PATH = Path("/tmp/v15_30a_gui_j6.lock")
HOST_BOOT_ID_PATH = Path("/proc/sys/kernel/random/boot_id")
EXPECTED_MASTER_ID = 0
SAMPLE_COUNT = 500
FINAL_DISABLED_FRAME_COUNT = 5
MINIMUM_SOURCE_COVERAGE_S = 4.0
MAXIMUM_RAW_SPAN_RAD = math.radians(0.20)
MAXIMUM_TAIL_DRIFT_RAD = math.radians(0.10)
MAXIMUM_TEMPERATURE_C = 60.0
REFRESH_FEEDBACK_TIMEOUT_S = 0.25
RUNTIME_HARDWARE_ACCESSED = False


class CaptureError(RuntimeError):
    """The refresh-only capture or its immutable evidence was rejected."""


@dataclass(frozen=True)
class FeedbackUdpBinding:
    port: int
    session_id: str
    state_instance_id: str


def validate_feedback_udp_binding(
    port: Any,
    session_id: Any,
    state_instance_id: Any,
) -> FeedbackUdpBinding | None:
    """Validate an all-or-nothing loopback feedback identity."""

    if port is None:
        if (
            (session_id is not None and session_id != "")
            or (state_instance_id is not None and state_instance_id != "")
        ):
            raise CaptureError(
                "feedback identity requires --feedback-port"
            )
        return None
    if type(port) is not int or not 1 <= port <= 65535:
        raise CaptureError("feedback port must be an integer in [1, 65535]")
    if (
        not isinstance(session_id, str)
        or re.fullmatch(
            r"persistent:[0-9a-f]{16}:j2session:[0-9a-f]{16}:"
            r"goauxsession:[0-9a-f]{16}",
            session_id,
        )
        is None
    ):
        raise CaptureError("feedback session id is invalid")
    if (
        not isinstance(state_instance_id, str)
        or len(state_instance_id) != 32
        or any(
            character not in "0123456789abcdef"
            for character in state_instance_id
        )
    ):
        raise CaptureError(
            "feedback state instance id must be 32 lowercase hexadecimal characters"
        )
    return FeedbackUdpBinding(port, session_id, state_instance_id)


def validate_feedback_source_instance_id(value: Any) -> str:
    if (
        not isinstance(value, str)
        or len(value) != 32
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise CaptureError(
            "feedback source instance id must be 32 lowercase hexadecimal characters"
        )
    return value


def create_feedback_socket() -> socket.socket:
    return socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


class DisabledFeedbackPublisher:
    """Mirror validated DISABLED samples to the local state aggregator."""

    def __init__(
        self,
        binding: FeedbackUdpBinding,
        udp_socket: Any,
        source_instance_id: str,
        monotonic_ns: Callable[[], int],
    ) -> None:
        validate_feedback_source_instance_id(source_instance_id)
        self.binding = binding
        self.socket = udp_socket
        self.source_instance_id = source_instance_id
        self.monotonic_ns = monotonic_ns
        self.sequence = 0
        self.last_source_monotonic_ns = 0
        self.published_count = 0
        self.closed = False

    def publish(self, event: Any, feedback: Any) -> None:
        """Publish only one already-observed, continuously DISABLED frame."""

        validate_feedback_samples(
            [(event, feedback)],
            minimum_count=1,
            label="J6 loopback telemetry",
        )
        source_ns = self.monotonic_ns()
        if type(source_ns) is not int or source_ns <= self.last_source_monotonic_ns:
            raise CaptureError(
                "feedback source monotonic clock did not strictly advance"
            )
        feedback_event_ns = int(
            finite(
                getattr(event, "event_monotonic_s", None),
                "J6 loopback feedback timestamp",
            )
            * 1.0e9
        )
        if not 0 < feedback_event_ns <= source_ns:
            raise CaptureError(
                "J6 loopback feedback timestamp is ahead of publication"
            )
        self.sequence += 1
        if self.sequence > (1 << 63) - 1:
            raise CaptureError("feedback sequence exhausted")
        payload = {
            "schema": "go-m8010-motor-feedback/1.0",
            "source_instance_id": self.source_instance_id,
            "sequence": self.sequence,
            "session_id": self.binding.session_id,
            "state_instance_id": self.binding.state_instance_id,
            "source_monotonic_ns": source_ns,
            "samples": [{
                "motor": "J6",
                "position_rad": finite(
                    feedback.position, "J6 loopback position"
                ),
                "velocity_rad_s": finite(
                    feedback.velocity, "J6 loopback velocity"
                ),
                "temperature_c": max(
                    finite(feedback.mos_temp, "J6 loopback MOS temperature"),
                    finite(feedback.coil_temp, "J6 loopback coil temperature"),
                ),
                "merror": 0,
                "communication_ok": True,
                "tau_cmd_rotor_nm": None,
                "tau_feedback_rotor_nm": None,
                "tau_joint_estimated_nm": None,
                "last_valid_feedback_monotonic_ns": feedback_event_ns,
                "trajectory_plan_token_id": "",
                "trajectory_sha256": "",
                "trajectory_state": "INACTIVE",
                "trajectory_sample_index": 0,
                "trajectory_interval_count": 0,
            }],
            "controller_mode": "brake",
            "controller_mode_by_motor": {"J6": "brake"},
            "domain_fault": False,
            "lease_safe_hold": False,
            "drive_state": 0,
            "tau_j2_logical_total_nm": None,
        }
        encoded = json.dumps(payload, separators=(",", ":")).encode("utf-8")
        self.socket.sendto(encoded, ("127.0.0.1", self.binding.port))
        self.last_source_monotonic_ns = source_ns
        self.published_count += 1

    def close(self) -> None:
        if not self.closed:
            self.socket.close()
            self.closed = True


def utc_now() -> datetime:
    return datetime.now(timezone.utc)


def utc_text(value: datetime) -> str:
    if value.tzinfo is None or value.utcoffset() is None:
        raise CaptureError("capture timestamp must be timezone-aware")
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


def validate_pose_binding_id(value: Any) -> str:
    """Require the exact immutable-pose digest spelling used by the caller."""
    if (
        not isinstance(value, str)
        or len(value) != 64
        or any(character not in "0123456789abcdef" for character in value)
    ):
        raise CaptureError(
            "pose binding id must be exactly 64 lowercase hexadecimal characters"
        )
    return value


def pose_binding_id_argument(value: str) -> str:
    try:
        return validate_pose_binding_id(value)
    except CaptureError as exc:
        raise argparse.ArgumentTypeError(str(exc)) from exc


def read_host_boot_id(path: Path = HOST_BOOT_ID_PATH) -> str:
    try:
        value = path.read_text(encoding="ascii").strip()
    except (OSError, UnicodeError) as exc:
        raise CaptureError(f"host boot id is unavailable: {exc}") from exc
    return validate_host_boot_id(value)


def read_recorded_boottime_ns() -> int:
    reader = getattr(time, "clock_gettime_ns", None)
    clock_id = getattr(time, "CLOCK_BOOTTIME", None)
    if not callable(reader) or type(clock_id) is not int:
        raise CaptureError("CLOCK_BOOTTIME is unavailable")
    value = reader(clock_id)
    if type(value) is not int or value <= 0:
        raise CaptureError("CLOCK_BOOTTIME is invalid")
    return value


def fsync_directory(path: Path) -> None:
    if os.name == "nt":
        return
    descriptor = os.open(path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def publish_json_no_overwrite(path: Path, document: dict[str, Any]) -> None:
    """Atomically publish one evidence pathname without ever replacing it."""

    destination = Path(os.path.abspath(path))
    destination.parent.mkdir(parents=True, exist_ok=True)
    if destination.exists() or destination.is_symlink():
        raise CaptureError(f"capture output already exists: {destination}")
    data = (json.dumps(document, ensure_ascii=False, indent=2) + "\n").encode(
        "utf-8"
    )
    temporary = destination.with_name(
        f".{destination.name}.{uuid.uuid4().hex}.tmp"
    )
    descriptor = os.open(
        temporary,
        os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_CLOEXEC", 0),
        0o600,
    )
    try:
        with os.fdopen(descriptor, "wb", closefd=True) as stream:
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        # A same-directory hard link is an atomic no-replace publication: link
        # fails with EEXIST if any writer won the destination race.
        os.link(temporary, destination)
        fsync_directory(destination.parent)
    finally:
        temporary.unlink(missing_ok=True)


@contextmanager
def acquire_device_lock() -> Any:
    if fcntl is None:
        raise CaptureError("fcntl is required for the production J6 lock")
    descriptor = os.open(
        DEVICE_LOCK_PATH,
        os.O_RDWR | os.O_CREAT | getattr(os, "O_CLOEXEC", 0),
        0o600,
    )
    try:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            raise CaptureError("J6 worker/diagnostic already owns the device lock") from exc
        yield
    finally:
        try:
            fcntl.flock(descriptor, fcntl.LOCK_UN)
        except OSError:
            pass
        os.close(descriptor)


def empty_command_audit() -> dict[str, int | bool]:
    return {
        "total_tx_attempt_count": 0,
        "refresh_request_count": 0,
        "rid_read_request_count": 0,
        "rid_write_count": 0,
        "flash_or_eeprom_write_count": 0,
        "motor_internal_zero_write_count": 0,
        "set_zero_command_count": 0,
        "fc_enable_count": 0,
        "fd_disable_count": 0,
        "position_velocity_torque_command_count": 0,
        "other_tx_attempt_count": 0,
        "forbidden_tx_attempt_count": 0,
        "only_disabled_refresh_requests": True,
    }


def classify_transmit(can_id: int, payload: bytes) -> str:
    if can_id == 0x7FF and payload == refresh_request():
        return "refresh"
    if len(payload) == 8 and payload == b"\xFF" * 7 + b"\xFC":
        return "fc_enable"
    if len(payload) == 8 and payload == b"\xFF" * 7 + b"\xFD":
        return "fd_disable"
    if len(payload) == 8 and payload == b"\xFF" * 7 + b"\xFE":
        return "set_zero"
    if can_id == 0x7FF and len(payload) == 8 and payload[2] == 0x33:
        return "rid_read"
    if can_id == 0x7FF and len(payload) == 8 and payload[2] == 0x55:
        return "rid_write"
    if can_id == EXPECTED_MOTOR_ID:
        return "position_velocity_torque"
    return "other"


def increment_audit(audit: dict[str, int | bool], category: str) -> None:
    audit["total_tx_attempt_count"] = int(audit["total_tx_attempt_count"]) + 1
    field = {
        "refresh": "refresh_request_count",
        "rid_read": "rid_read_request_count",
        "rid_write": "rid_write_count",
        "fc_enable": "fc_enable_count",
        "fd_disable": "fd_disable_count",
        "set_zero": "set_zero_command_count",
        "position_velocity_torque": "position_velocity_torque_command_count",
        "other": "other_tx_attempt_count",
    }[category]
    audit[field] = int(audit[field]) + 1
    if category == "set_zero":
        audit["motor_internal_zero_write_count"] = (
            int(audit["motor_internal_zero_write_count"]) + 1
        )
    if category != "refresh":
        audit["forbidden_tx_attempt_count"] = (
            int(audit["forbidden_tx_attempt_count"]) + 1
        )
        audit["only_disabled_refresh_requests"] = False


class RefreshOnlyLogger:
    """Capability wrapper that rejects every outbound frame except refresh."""

    def __init__(self, logger: Any) -> None:
        self._logger = logger
        self._logger.master_id = EXPECTED_MASTER_ID
        self.command_audit = empty_command_audit()
        self.close_attempted = False
        self.closed = False

    @property
    def channel(self) -> int:
        return int(self._logger.channel)

    @property
    def motor_id(self) -> int:
        return int(self._logger.motor_id)

    @property
    def master_id(self) -> int:
        return int(self._logger.master_id)

    def send(self, can_id: int, payload: bytes, label: str) -> float:
        category = classify_transmit(can_id, payload)
        increment_audit(self.command_audit, category)
        if category != "refresh":
            raise CaptureError(
                f"refresh-only guard rejected {category}: {label} ID=0x{can_id:X}"
            )
        return self._logger.send(can_id, payload, label)

    def snapshot(self) -> list[Any]:
        return self._logger.snapshot()

    def classify(self, event: Any) -> str:
        return self._logger.classify(event)

    def close(self) -> None:
        if not self.closed:
            self.close_attempted = True
            self._logger.close()
            self.closed = True


class FailSafeRawCanLogger(RawCanLogger):
    """Close a partially initialized adapter if its constructor raises."""

    def __init__(self) -> None:
        try:
            super().__init__()
        except BaseException:
            # RawCanLogger.close() tolerates attributes that were not reached
            # during construction, so this covers open/enable/hook failures.
            super().close()
            raise


def capture_disabled_refresh_samples(
    logger: RefreshOnlyLogger,
    count: int,
    label: str,
    sample_callback: Callable[[Any, Any], None] | None = None,
) -> tuple[list[tuple[Any, Any]], int]:
    """Pace refreshes at 100 Hz and fail before the next TX if state is active."""

    start_index = len(logger.snapshot())
    start = time.monotonic()
    values: list[tuple[Any, Any]] = []
    for index in range(count):
        target = start + index * TX_PERIOD_S
        remaining = target - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)
        receive_start_index = len(logger.snapshot())
        sent_at = logger.send(0x7FF, refresh_request(), f"{label}_{index:03d}")
        deadline = time.monotonic() + REFRESH_FEEDBACK_TIMEOUT_S
        while True:
            candidates = [
                value
                for value in normal_feedback(
                    logger,
                    logger.snapshot()[receive_start_index:],
                )
                if value[0].event_monotonic_s >= sent_at
            ]
            if candidates:
                selected = candidates[0]
                # Stop before another refresh if this frame is not safe.
                validate_feedback_samples(
                    [selected],
                    minimum_count=1,
                    label=f"{label} frame {index}",
                )
                if sample_callback is not None:
                    sample_callback(*selected)
                values.append(selected)
                break
            if time.monotonic() >= deadline:
                raise CaptureError(
                    f"{label} frame {index} normal feedback timed out"
                )
            time.sleep(0.001)
    return values, start_index


def observed_transmit_audit(events: list[Any]) -> dict[str, int | bool]:
    audit = empty_command_audit()
    for event in events:
        if getattr(event, "event_kind", None) != "TX_REQUEST":
            continue
        increment_audit(
            audit,
            classify_transmit(int(event.can_id), bytes(event.payload)),
        )
    return audit


def finite(value: Any, label: str) -> float:
    if type(value) not in {int, float} or not math.isfinite(float(value)):
        raise CaptureError(f"non-finite {label}")
    return float(value)


def descriptive_position_statistics(values: list[float]) -> dict[str, float]:
    if not values:
        raise CaptureError("J6 has no finite raw position samples")
    mean = math.fsum(values) / len(values)
    median = statistics.median(values)
    minimum = min(values)
    maximum = max(values)
    span = maximum - minimum
    variance = math.fsum((value - mean) ** 2 for value in values) / len(values)
    standard_deviation = math.sqrt(max(0.0, variance))
    result = {
        "mean": mean,
        "median": median,
        "minimum": minimum,
        "maximum": maximum,
        "span": span,
        "standard_deviation": standard_deviation,
    }
    if not all(math.isfinite(value) for value in result.values()):
        raise CaptureError("J6 raw position statistics are non-finite")
    return result


def raw_position_statistics(
    values: list[float], minimum_count: int = SAMPLE_COUNT
) -> dict[str, float]:
    if len(values) < minimum_count:
        raise CaptureError(f"J6 has fewer than {minimum_count} raw samples")
    result = descriptive_position_statistics(values)
    if any(abs(value) > PMAX_PROTOCOL_RAD for value in values):
        raise CaptureError("J6 raw position exceeds the protocol envelope")
    if result["span"] > MAXIMUM_RAW_SPAN_RAD:
        raise CaptureError(
            f"J6 raw span {result['span']:.9f} rad is not stationary"
        )
    tail = values[-min(10, len(values)):]
    if abs(statistics.median(tail) - result["median"]) > MAXIMUM_TAIL_DRIFT_RAD:
        raise CaptureError("J6 raw position tail drift is not stationary")
    return result


def validate_usb_identity(identity: Any) -> dict[str, Any]:
    if not isinstance(identity, dict):
        raise CaptureError("J6 USB identity is not an object")
    expected = {
        "vid": EXPECTED_USB_VID,
        "pid": EXPECTED_USB_PID,
        "serial": EXPECTED_USB_SERIAL,
    }
    if any(identity.get(key) != value for key, value in expected.items()):
        raise CaptureError("J6 USB VID/PID/serial changed")
    return copy.deepcopy(identity)


def validate_feedback_samples(
    values: list[tuple[Any, Any]],
    *,
    minimum_count: int,
    label: str,
) -> dict[str, Any]:
    if len(values) < minimum_count:
        raise CaptureError(
            f"{label} returned {len(values)}/{minimum_count} valid frames"
        )
    selected = values[:minimum_count]
    event_times: list[float] = []
    positions: list[float] = []
    velocities: list[float] = []
    torques: list[float] = []
    mos_temperatures: list[float] = []
    coil_temperatures: list[float] = []
    states: list[int] = []
    for index, (event, feedback) in enumerate(selected):
        event_time = finite(
            getattr(event, "event_monotonic_s", None),
            f"{label} frame {index} timestamp",
        )
        state = getattr(feedback, "state", None)
        if type(state) is not int:
            raise CaptureError(f"{label} frame {index} state is invalid")
        event_times.append(event_time)
        states.append(state)
        positions.append(finite(feedback.position, f"{label} frame {index} position"))
        velocities.append(finite(feedback.velocity, f"{label} frame {index} velocity"))
        torques.append(finite(feedback.torque, f"{label} frame {index} torque"))
        mos_temperatures.append(
            finite(feedback.mos_temp, f"{label} frame {index} MOS temperature")
        )
        coil_temperatures.append(
            finite(feedback.coil_temp, f"{label} frame {index} coil temperature")
        )
    if any(right < left for left, right in zip(event_times, event_times[1:])):
        raise CaptureError(f"{label} feedback timestamps moved backwards")
    if any(state != 0 for state in states):
        names = sorted({state_text(state) for state in states})
        raise CaptureError(f"{label} was not continuously DISABLED: {names}")
    temperatures = mos_temperatures + coil_temperatures
    if any(not 0.0 <= value < MAXIMUM_TEMPERATURE_C for value in temperatures):
        raise CaptureError(f"{label} temperature is outside [0, 60) C")
    return {
        "count": len(selected),
        "event_times": event_times,
        "positions": positions,
        "velocities": velocities,
        "torques": torques,
        "mos_temperatures": mos_temperatures,
        "coil_temperatures": coil_temperatures,
        "states": states,
    }


@dataclass(frozen=True)
class RuntimeDependencies:
    logger_factory: Callable[[], Any] = FailSafeRawCanLogger
    identity_reader: Callable[[], dict[str, Any]] = exact_usb_identity
    sample_capturer: Callable[..., tuple[list[tuple[Any, Any]], int]] = (
        capture_disabled_refresh_samples
    )
    monotonic: Callable[[], float] = time.monotonic
    now_utc: Callable[[], datetime] = utc_now
    host_boot_id_reader: Callable[[], str] = read_host_boot_id
    boottime_ns_reader: Callable[[], int] = read_recorded_boottime_ns
    token_factory: Callable[[], str] = lambda: uuid.uuid4().hex
    lock_factory: Callable[[], ContextManager[Any]] = acquire_device_lock
    feedback_socket_factory: Callable[[], Any] = create_feedback_socket
    feedback_source_instance_id_factory: Callable[[], str] = (
        lambda: secrets.token_hex(16)
    )
    monotonic_ns: Callable[[], int] = time.monotonic_ns


DEFAULT_DEPENDENCIES = RuntimeDependencies()


def new_capture_identity(deps: RuntimeDependencies) -> dict[str, Any]:
    instant = deps.now_utc()
    token = deps.token_factory().lower()
    if len(token) != 32 or any(character not in "0123456789abcdef" for character in token):
        raise CaptureError("capture token must be 32 lowercase hexadecimal characters")
    timestamp = instant.astimezone(timezone.utc).strftime("%Y%m%dT%H%M%S%fZ")
    boottime_ns = deps.boottime_ns_reader()
    if type(boottime_ns) is not int or boottime_ns <= 0:
        raise CaptureError("recorded CLOCK_BOOTTIME is invalid")
    return {
        "capture_id": f"j6-disabled-raw-{timestamp}-{token[:16]}",
        "power_session_id": f"j6-power-session-{timestamp}-{token}",
        "recorded_at_utc": utc_text(instant),
        "host_boot_id": validate_host_boot_id(deps.host_boot_id_reader()),
        "recorded_boottime_ns": boottime_ns,
    }


def base_document(
    identity: dict[str, Any], pose_binding_id: str
) -> dict[str, Any]:
    return {
        "schema": CAPTURE_SCHEMA,
        **identity,
        "pose_binding_id": validate_pose_binding_id(pose_binding_id),
        "status": "FAIL",
        "hardware_accessed": False,
        "physical_power_state_during_capture": "24V_ON",
        "packet_count": 0,
        "source_coverage_s": 0.0,
        "safety": {
            "execution_policy": "DISABLED_REFRESH_ONLY",
            **empty_command_audit(),
            "observed_tx_audit": empty_command_audit(),
            "active_or_hold_commands_sent": 0,
            "active_commands_sent": 0,
            "hold_commands_sent": 0,
            "motor_internal_zero_modified": False,
            "motor_internal_zero_write_count": 0,
            "rid_written": False,
            "rid_write_count": 0,
            "flash_or_eeprom_written": False,
            "flash_or_eeprom_write_count": 0,
        },
        "motors": {
            "J6": {
                "sample_count": 0,
                "raw_position_rad": None,
            }
        },
        "terminal": {
            "final_disabled_frames": 0,
            "required_final_disabled_frames": FINAL_DISABLED_FRAME_COUNT,
            "states": [],
            "state_names": [],
            "confirmed": False,
            "channel_closed": False,
        },
        "failures": [],
        "physical_power_off_required": False,
    }


def feedback_handoff_document(
    raw_output: Path,
    raw_sha256: str,
    document: dict[str, Any],
    binding: FeedbackUdpBinding,
    publisher: DisabledFeedbackPublisher,
) -> dict[str, Any]:
    """Bind the one-shot producer continuation to exact PASS evidence."""

    expected_published = int(document["packet_count"]) + int(
        document["terminal"]["required_final_disabled_frames"]
    )
    if (
        document.get("status") != "PASS"
        or document.get("physical_power_off_required") is not False
        or document["terminal"].get("confirmed") is not True
        or document["terminal"].get("channel_closed") is not True
        or publisher.closed is not True
        or publisher.published_count != expected_published
        or publisher.sequence != expected_published
        or publisher.last_source_monotonic_ns <= 0
        or len(raw_sha256) != 64
        or any(
            character not in "0123456789abcdef"
            for character in raw_sha256
        )
    ):
        raise CaptureError("J6 feedback handoff requires exact closed PASS evidence")
    return {
        "schema": FEEDBACK_HANDOFF_SCHEMA,
        "handoff_id": (
            f"j6-feedback-{publisher.source_instance_id}-"
            f"{publisher.sequence}"
        ),
        "session_id": binding.session_id,
        "state_instance_id": binding.state_instance_id,
        "source_instance_id": publisher.source_instance_id,
        "last_sequence": publisher.sequence,
        "last_source_monotonic_ns": publisher.last_source_monotonic_ns,
        "raw_capture": {
            "path": str(raw_output),
            "sha256": raw_sha256,
        },
        "terminal": {
            "drive_state": 0,
            "controller_mode": "brake",
            "confirmed": True,
            "channel_closed": True,
        },
        "forwarding": {
            "destination": f"127.0.0.1:{binding.port}",
            "published_count": publisher.published_count,
            "socket_closed": publisher.closed,
        },
        "active_control_authorized": False,
        "can_tx_policy": "DISABLED_CLASSIC_CAN_REFRESH_ONLY",
        "single_use_claim_required": True,
    }


def validate_gates(confirm: str, physical_confirmation: str) -> None:
    if confirm != CONFIRM_GATE:
        raise CaptureError(f"require --confirm {CONFIRM_GATE}")
    if physical_confirmation != PHYSICAL_GATE:
        raise CaptureError(f"require --physical-confirmation '{PHYSICAL_GATE}'")


def prepare_output(path: Path) -> Path:
    output = Path(os.path.abspath(path))
    if output.exists() or output.is_symlink():
        raise CaptureError(f"capture output already exists: {output}")
    return output


def prepare_feedback_handoff_output(
    path: Any, raw_output: Path, forwarding_enabled: bool,
) -> Path | None:
    if not forwarding_enabled:
        if path is not None and path != "":
            raise CaptureError(
                "feedback handoff output requires --feedback-port"
            )
        return None
    if path is None or path == "":
        raise CaptureError(
            "forwarded J6 feedback requires --feedback-handoff-output"
        )
    result = prepare_output(Path(path))
    if result == raw_output:
        raise CaptureError("feedback handoff output must differ from raw output")
    return result


def _recover_feedback(logger: RefreshOnlyLogger, start_index: int) -> list[tuple[Any, Any]]:
    try:
        return normal_feedback(logger, logger.snapshot()[start_index:])
    except Exception:
        return []


def capture_with_optional_feedback(
    deps: RuntimeDependencies,
    logger: RefreshOnlyLogger,
    count: int,
    label: str,
    publisher: DisabledFeedbackPublisher | None,
) -> tuple[list[tuple[Any, Any]], int]:
    """Preserve the legacy three-argument path when forwarding is disabled."""

    if publisher is None:
        return deps.sample_capturer(logger, count, label)
    return deps.sample_capturer(
        logger, count, label, publisher.publish
    )


def run(
    args: argparse.Namespace,
    deps: RuntimeDependencies = DEFAULT_DEPENDENCIES,
) -> int:
    global RUNTIME_HARDWARE_ACCESSED
    RUNTIME_HARDWARE_ACCESSED = False
    pose_binding_id = validate_pose_binding_id(
        getattr(args, "pose_binding_id", None)
    )
    sample_count = int(getattr(args, "sample_count", SAMPLE_COUNT))
    minimum_source_coverage_s = float(
        getattr(args, "minimum_source_coverage_s", MINIMUM_SOURCE_COVERAGE_S)
    )
    if not 500 <= sample_count <= 6000:
        raise CaptureError("J6 sample count must be in [500, 6000]")
    if not 4.0 <= minimum_source_coverage_s <= 60.0:
        raise CaptureError("J6 minimum source coverage must be in [4, 60] seconds")
    feedback_binding = validate_feedback_udp_binding(
        getattr(args, "feedback_port", None),
        getattr(args, "feedback_session_id", None),
        getattr(args, "feedback_state_instance_id", None),
    )
    validate_gates(args.confirm, args.physical_confirmation)
    output = prepare_output(args.output)
    feedback_handoff_output = prepare_feedback_handoff_output(
        getattr(args, "feedback_handoff_output", None),
        output,
        feedback_binding is not None,
    )
    identity = new_capture_identity(deps)
    document = base_document(identity, pose_binding_id)
    feedback_source_instance_id = (
        None
        if feedback_binding is None
        else validate_feedback_source_instance_id(
            deps.feedback_source_instance_id_factory()
        )
    )
    feedback_publisher = (
        None
        if feedback_binding is None
        else DisabledFeedbackPublisher(
            feedback_binding,
            deps.feedback_socket_factory(),
            feedback_source_instance_id,
            deps.monotonic_ns,
        )
    )
    document["physical_confirmation"] = {
        "confirmation_gate": PHYSICAL_GATE,
        "j6_24v_on": True,
        "support_reliable": True,
        "whole_arm_vertical_initialization_pose": True,
        "arm_stationary": True,
        "not_at_mechanical_limit": True,
    }

    usb_identity: dict[str, Any] | None = None
    guarded: RefreshOnlyLogger | None = None
    main_values: list[tuple[Any, Any]] = []
    final_values: list[tuple[Any, Any]] = []
    main_start_index = 0
    final_start_index = 0
    capture_started_at: float | None = None
    capture_finished_at: float | None = None
    failure: BaseException | None = None
    snapshot: list[Any] = []

    try:
        with deps.lock_factory():
            usb_identity = validate_usb_identity(deps.identity_reader())
            # The factory itself opens/enables the USB-CAN channel, so mark the
            # access conservatively before entering it.
            RUNTIME_HARDWARE_ACCESSED = True
            document["hardware_accessed"] = True
            raw_logger = deps.logger_factory()
            guarded = RefreshOnlyLogger(raw_logger)
            if (
                guarded.channel != EXPECTED_CHANNEL
                or guarded.motor_id != EXPECTED_MOTOR_ID
                or guarded.master_id != EXPECTED_MASTER_ID
            ):
                raise CaptureError("J6 channel/motor/master identity changed")
            try:
                main_start_index = len(guarded.snapshot())
                capture_started_at = deps.monotonic()
                main_values, _ = capture_with_optional_feedback(
                    deps,
                    guarded,
                    sample_count,
                    "J6_DISABLED_ZERO_CAPTURE",
                    feedback_publisher,
                )
                capture_finished_at = deps.monotonic()
                final_start_index = len(guarded.snapshot())
                final_values, _ = capture_with_optional_feedback(
                    deps,
                    guarded,
                    FINAL_DISABLED_FRAME_COUNT,
                    "J6_FINAL_DISABLED",
                    feedback_publisher,
                )
            except BaseException as exc:
                failure = exc
                if not main_values:
                    main_values = _recover_feedback(guarded, main_start_index)
                elif not final_values:
                    final_values = _recover_feedback(guarded, final_start_index)
            finally:
                if capture_started_at is not None and capture_finished_at is None:
                    capture_finished_at = deps.monotonic()
                snapshot = guarded.snapshot()
                guarded.close()
    except BaseException as exc:
        if failure is None:
            failure = exc
        if guarded is not None:
            try:
                snapshot = guarded.snapshot()
            except Exception:
                pass
            try:
                guarded.close()
            except Exception as close_exc:
                document["failures"].append(
                    f"CHANNEL_CLOSE:{type(close_exc).__name__}:{close_exc}"
                )

    if feedback_publisher is not None:
        try:
            feedback_publisher.close()
        except BaseException as exc:
            if failure is None:
                failure = exc
            document["failures"].append(
                f"FEEDBACK_SOCKET_CLOSE:{type(exc).__name__}:{exc}"
            )
        document["readonly_feedback_udp"] = {
            "schema": "go-m8010-readonly-feedback-forwarding/1.0",
            "destination": f"127.0.0.1:{feedback_binding.port}",
            "session_id": feedback_binding.session_id,
            "state_instance_id": feedback_binding.state_instance_id,
            "source_instance_id": feedback_publisher.source_instance_id,
            "published_count": feedback_publisher.published_count,
            "last_sequence": feedback_publisher.sequence,
            "last_source_monotonic_ns": (
                feedback_publisher.last_source_monotonic_ns
            ),
            "socket_closed": feedback_publisher.closed,
            "active_control_authorized": False,
            "can_tx_policy": "DISABLED_CLASSIC_CAN_REFRESH_ONLY",
            "handoff_output": str(feedback_handoff_output),
        }

    document["usb_identity"] = usb_identity
    document["terminal"]["channel_closed"] = bool(
        guarded is not None and guarded.closed
    )
    attempted_audit = (
        empty_command_audit()
        if guarded is None
        else copy.deepcopy(guarded.command_audit)
    )
    observed_audit = observed_transmit_audit(snapshot)
    document["safety"].update(attempted_audit)
    document["safety"]["observed_tx_audit"] = observed_audit

    try:
        main = validate_feedback_samples(
            main_values,
            minimum_count=sample_count,
            label="main capture",
        )
        final = validate_feedback_samples(
            final_values,
            minimum_count=FINAL_DISABLED_FRAME_COUNT,
            label="final DISABLED proof",
        )
        source_coverage_s = main["event_times"][-1] - main["event_times"][0]
        wall_coverage_s = (
            0.0
            if capture_started_at is None or capture_finished_at is None
            else capture_finished_at - capture_started_at
        )
        if source_coverage_s < minimum_source_coverage_s:
            raise CaptureError(
                f"J6 feedback source coverage {source_coverage_s:.6f}s is too short"
            )
        if wall_coverage_s < minimum_source_coverage_s:
            raise CaptureError(
                f"J6 capture wall duration {wall_coverage_s:.6f}s is too short"
            )
        raw_stats = raw_position_statistics(
            main["positions"], minimum_count=sample_count
        )
        if any(
            int(attempted_audit[field]) != 0
            for field in (
                "rid_read_request_count",
                "rid_write_count",
                "flash_or_eeprom_write_count",
                "motor_internal_zero_write_count",
                "set_zero_command_count",
                "fc_enable_count",
                "fd_disable_count",
                "position_velocity_torque_command_count",
                "other_tx_attempt_count",
                "forbidden_tx_attempt_count",
            )
        ):
            raise CaptureError("J6 attempted command audit is not refresh-only")
        expected_refreshes = sample_count + FINAL_DISABLED_FRAME_COUNT
        if int(attempted_audit["refresh_request_count"]) != expected_refreshes:
            raise CaptureError("J6 attempted refresh count is incomplete")
        if observed_audit != attempted_audit:
            raise CaptureError("J6 observed TX audit differs from guarded attempts")
        error_callbacks = sum(
            getattr(event, "event_kind", None) == "ERROR_CALLBACK"
            for event in snapshot
        )
        unknown_rx_callbacks = sum(
            getattr(event, "event_kind", None) == "RX_CALLBACK"
            and guarded is not None
            and guarded.classify(event) != "NORMAL_FEEDBACK"
            for event in snapshot
        )
        if error_callbacks or unknown_rx_callbacks:
            raise CaptureError(
                "J6 capture observed an error callback or unknown RX frame"
            )

        document.update({
            "status": "PASS",
            "packet_count": sample_count,
            "source_coverage_s": source_coverage_s,
            "motors": {
                "J6": {
                    "sample_count": sample_count,
                    "raw_position_rad": raw_stats,
                    "velocity_rad_s": {
                        "minimum": min(main["velocities"]),
                        "maximum": max(main["velocities"]),
                        "mean": math.fsum(main["velocities"]) / sample_count,
                    },
                    "mos_temperature_c": {
                        "minimum": min(main["mos_temperatures"]),
                        "maximum": max(main["mos_temperatures"]),
                    },
                    "coil_temperature_c": {
                        "minimum": min(main["coil_temperatures"]),
                        "maximum": max(main["coil_temperatures"]),
                    },
                    "state_values": [0],
                    "state_names": [state_text(0)],
                }
            },
            "terminal": {
                "final_disabled_frames": FINAL_DISABLED_FRAME_COUNT,
                "required_final_disabled_frames": FINAL_DISABLED_FRAME_COUNT,
                "states": final["states"],
                "state_names": [state_text(value) for value in final["states"]],
                "confirmed": True,
                "channel_closed": bool(guarded and guarded.closed),
            },
            "raw_safety_summary": {
                "local_capture_duration_s": wall_coverage_s,
                "error_callback_count": 0,
                "unknown_rx_callback_count": 0,
                "expected_usb": {
                    "vid": EXPECTED_USB_VID,
                    "pid": EXPECTED_USB_PID,
                    "serial": EXPECTED_USB_SERIAL,
                },
                "expected_can_channel": EXPECTED_CHANNEL,
                "expected_motor_id": EXPECTED_MOTOR_ID,
                "expected_master_id": EXPECTED_MASTER_ID,
                "branch_inputs_loaded": False,
                "persistent_zero_loaded": False,
                "recovery_hint_loaded": False,
                "integer_turn_branch_inferred": False,
            },
            "failures": [],
        })
        if failure is not None:
            raise failure
    except BaseException as exc:
        if failure is None:
            failure = exc
        document["status"] = "FAIL"
        document["packet_count"] = min(len(main_values), sample_count)
        if main_values:
            try:
                event_times = [
                    finite(event.event_monotonic_s, "failure evidence timestamp")
                    for event, _ in main_values[:sample_count]
                ]
                if len(event_times) >= 2:
                    document["source_coverage_s"] = max(
                        0.0, event_times[-1] - event_times[0]
                    )
            except Exception:
                pass
        document["motors"]["J6"]["sample_count"] = min(
            len(main_values), sample_count
        )
        finite_positions = []
        for _, feedback in main_values[:sample_count]:
            value = getattr(feedback, "position", None)
            if type(value) in {int, float} and math.isfinite(float(value)):
                finite_positions.append(float(value))
        if finite_positions:
            document["motors"]["J6"]["raw_position_rad"] = (
                descriptive_position_statistics(finite_positions)
            )
            document["motors"]["J6"]["finite_position_sample_count"] = len(
                finite_positions
            )
        final_states = [
            getattr(feedback, "state", None)
            for _, feedback in final_values[:FINAL_DISABLED_FRAME_COUNT]
        ]
        final_disabled_frames = sum(state == 0 for state in final_states)
        document["terminal"].update({
            "final_disabled_frames": final_disabled_frames,
            "states": final_states,
            "state_names": [
                state_text(feedback.state)
                for _, feedback in final_values[:FINAL_DISABLED_FRAME_COUNT]
                if type(getattr(feedback, "state", None)) is int
            ],
            "confirmed": bool(
                len(final_states) >= FINAL_DISABLED_FRAME_COUNT
                and final_disabled_frames >= FINAL_DISABLED_FRAME_COUNT
            ),
            "channel_closed": bool(guarded and guarded.closed),
        })
        document["failures"].append(
            f"{type(failure).__name__}:{failure}"
        )

    observed_active_state = any(
        type(getattr(feedback, "state", None)) is int and feedback.state != 0
        for _, feedback in (main_values + final_values)
    )
    terminal_unproven = bool(
        document["hardware_accessed"]
        and (
            not document["terminal"]["confirmed"]
            or not document["terminal"]["channel_closed"]
        )
    )
    document["physical_power_off_required"] = bool(
        document["status"] != "PASS"
        and document["hardware_accessed"]
        and (observed_active_state or terminal_unproven)
    )
    if document["physical_power_off_required"]:
        document["physical_power_off_reason"] = (
            "J6 did not retain a fully proven DISABLED-and-closed terminal state"
        )

    document["completed_at_utc"] = utc_text(deps.now_utc())
    publish_json_no_overwrite(output, document)
    if feedback_handoff_output is not None and document["status"] == "PASS":
        assert feedback_binding is not None
        assert feedback_publisher is not None
        raw_sha256 = hashlib.sha256(output.read_bytes()).hexdigest()
        handoff = feedback_handoff_document(
            output,
            raw_sha256,
            document,
            feedback_binding,
            feedback_publisher,
        )
        publish_json_no_overwrite(feedback_handoff_output, handoff)
    print(json.dumps(document, ensure_ascii=False, indent=2), flush=True)
    return 0 if document["status"] == "PASS" else 2


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument(
        "--pose-binding-id",
        type=pose_binding_id_argument,
        required=True,
        help="exact 64-character lowercase hexadecimal pose binding digest",
    )
    parser.add_argument("--confirm", required=True)
    parser.add_argument("--physical-confirmation", required=True)
    parser.add_argument(
        "--sample-count",
        type=int,
        default=SAMPLE_COUNT,
        help="DISABLED refresh samples; V15.31B uses 1100 for >=10 s",
    )
    parser.add_argument(
        "--minimum-source-coverage-s",
        type=float,
        default=MINIMUM_SOURCE_COVERAGE_S,
    )
    parser.add_argument(
        "--feedback-port",
        type=int,
        help="optional localhost whole-arm state UDP port",
    )
    parser.add_argument("--feedback-session-id", default="")
    parser.add_argument("--feedback-state-instance-id", default="")
    parser.add_argument("--feedback-handoff-output", type=Path)
    args = parser.parse_args(argv)
    if not 500 <= args.sample_count <= 6000:
        parser.error("sample count must be in [500, 6000]")
    if not 4.0 <= args.minimum_source_coverage_s <= 60.0:
        parser.error("minimum source coverage must be in [4, 60] seconds")
    try:
        validate_feedback_udp_binding(
            args.feedback_port,
            args.feedback_session_id,
            args.feedback_state_instance_id,
        )
    except CaptureError as exc:
        parser.error(str(exc))
    try:
        prepare_feedback_handoff_output(
            args.feedback_handoff_output,
            Path(os.path.abspath(args.output)),
            args.feedback_port is not None,
        )
    except CaptureError as exc:
        parser.error(str(exc))
    return args


def main() -> int:
    args: argparse.Namespace | None = None
    try:
        args = parse_args()
        return run(args)
    except (CaptureError, Blocked, OSError, ValueError) as exc:
        print(
            json.dumps(
                {
                    "schema": CAPTURE_SCHEMA,
                    "status": "BLOCKED",
                    "pose_binding_id": (
                        None if args is None else getattr(args, "pose_binding_id", None)
                    ),
                    "reason": str(exc),
                    "hardware_accessed": RUNTIME_HARDWARE_ACCESSED,
                    "active_control_authorized": False,
                    "physical_power_off_required": RUNTIME_HARDWARE_ACCESSED,
                },
                ensure_ascii=False,
                indent=2,
            ),
            flush=True,
        )
        return 4


if __name__ == "__main__":
    raise SystemExit(main())

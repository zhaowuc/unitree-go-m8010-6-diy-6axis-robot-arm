"""ROS 2 state source: seven physical feedback streams to six logical joints.

The node deliberately exposes no command subscriber, trajectory action,
controller API, serial port, CAN transport, HOLD, FOC, or BRAKE operation.
The process which already owns each hardware bus publishes the raw feedback
contract on ``/whole_arm/motor_feedback_raw``.
"""

from __future__ import annotations

import csv
import hashlib
import json
import math
import os
import secrets
import signal
import socket
import stat
import statistics
import threading
import time
from pathlib import Path
from typing import Iterable, Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .state_model import (
    GEAR_RATIO,
    JOINT_NAMES,
    MOTOR_NAMES,
    MirrorSessionReferenceV1,
    PositionSpanVelocityObserver,
    WorkerSupervisorStatusError,
    parse_feedback_payload,
    unavailable_worker_control_status,
    validate_worker_supervisor_status,
)


J2_MOTOR_NAMES = frozenset({"J2A", "J2B"})
WARNING_LOG_INTERVAL_NS = 5_000_000_000
CONTROLLER_MODES = frozenset({"brake", "drag", "hold", "position", "unknown"})
WORKER_SUPERVISOR_STATUS_FILENAME = "worker_supervisor_status.json"
WORKER_SUPERVISOR_STATUS_MAX_BYTES = 16_384


def read_worker_supervisor_status_file(
    path: Path, now_monotonic_ns: int, maximum_age_ns: int,
) -> dict:
    """Read one owner-only, non-symlink supervisor heartbeat atomically."""

    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0) | getattr(os, "O_NOFOLLOW", 0)
    descriptor = os.open(path, flags)
    try:
        metadata = os.fstat(descriptor)
        if (
            not stat.S_ISREG(metadata.st_mode)
            or metadata.st_size <= 0
            or metadata.st_size > WORKER_SUPERVISOR_STATUS_MAX_BYTES
            or metadata.st_mode & (stat.S_IWGRP | stat.S_IWOTH)
            or (
                hasattr(os, "geteuid")
                and metadata.st_uid != os.geteuid()
            )
        ):
            raise WorkerSupervisorStatusError("supervisor_status_invalid")
        chunks = []
        remaining = WORKER_SUPERVISOR_STATUS_MAX_BYTES + 1
        while remaining > 0:
            chunk = os.read(descriptor, remaining)
            if not chunk:
                break
            chunks.append(chunk)
            remaining -= len(chunk)
        data = b"".join(chunks)
        if not data or len(data) > WORKER_SUPERVISOR_STATUS_MAX_BYTES:
            raise WorkerSupervisorStatusError("supervisor_status_invalid")
    finally:
        os.close(descriptor)
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise WorkerSupervisorStatusError("supervisor_status_invalid") from exc
    return validate_worker_supervisor_status(
        value, now_monotonic_ns, maximum_age_ns
    )


def feedback_rejection_reason(error: Exception) -> str:
    """Map arbitrary parser failures to bounded warning categories."""

    if isinstance(error, UnicodeDecodeError):
        return "utf8_decode"
    if isinstance(error, json.JSONDecodeError):
        return "json_decode"
    if isinstance(error, KeyError):
        return "missing_field"
    if isinstance(error, (TypeError, ValueError, OverflowError)):
        return "invalid_field"
    return "invalid_payload"


def update_j2_sync_fault(previous: bool, samples: Iterable, payload: dict) -> bool:
    """Update the live J2 interlock only from the J2 worker's payload.

    Every GO fault-domain worker uses the same payload schema and non-J2
    workers report ``j2_sync_fault=false``.  Letting those packets clear the
    flag would hide a real J2 interlock; latching every historical true value,
    on the other hand, leaves the GUI warning stuck forever.  The motor list is
    the authoritative domain identity already validated by the parser.
    """

    if any(getattr(sample, "motor", None) in J2_MOTOR_NAMES for sample in samples):
        return bool(payload.get("j2_sync_fault", False))
    return previous


def load_persistent_zero(path: Path) -> tuple[dict[str, float], str]:
    data = path.read_bytes()
    document = json.loads(data)
    if document.get("schema") != "go-m8010-persistent-software-zero/1.0":
        raise ValueError("persistent software zero schema mismatch")
    if document.get("reference_name") != "PERSISTENT_SOFTWARE_ZERO_V1":
        raise ValueError("persistent software zero reference name mismatch")
    writes = document.get("writes", {})
    if any(
        writes.get(name) is not False
        for name in (
            "motor_internal_zero_modified",
            "rid_written",
            "flash_or_eeprom_written",
        )
    ):
        raise ValueError("persistent software zero records a forbidden motor write")
    motors = document.get("motors")
    if not isinstance(motors, dict) or set(motors) != set(MOTOR_NAMES):
        raise ValueError("persistent software zero must contain exactly seven motors")
    references = {
        name: float(motors[name]["raw_position_rad"]) for name in MOTOR_NAMES
    }
    if not all(math.isfinite(value) for value in references.values()):
        raise ValueError("persistent software zero contains a non-finite reference")
    return references, hashlib.sha256(data).hexdigest()


def load_recovery_hints(path: Path) -> dict[str, float]:
    document = json.loads(path.read_text(encoding="utf-8"))
    if document.get("schema") != "go-m8010-recovery-branch-hints/1.0":
        raise ValueError("recovery hint schema mismatch")
    if document.get("reference_name") != "PERSISTENT_SOFTWARE_ZERO_V1":
        raise ValueError("recovery hint reference name mismatch")
    motors = document.get("motors")
    if not isinstance(motors, dict) or not set(motors).issubset(set(MOTOR_NAMES)):
        raise ValueError("recovery hints contain an unknown motor")
    hints = {
        name: float(value["logical_position_rad"]) for name, value in motors.items()
    }
    if not all(math.isfinite(value) for value in hints.values()):
        raise ValueError("recovery hints contain a non-finite position")
    return hints


def load_initial_pose(
    path: Path, persistent_zero_sha256: str
) -> tuple[dict[str, float], str]:
    data = path.read_bytes()
    document = json.loads(data)
    if document.get("有效") is not True:
        raise ValueError("initial pose is not marked valid")
    if document.get("会话标识") != f"persistent:{persistent_zero_sha256[:16]}":
        raise ValueError("initial pose persistent parent mismatch")
    values = document.get("关节位置_弧度")
    expected = {f"J{index}" for index in range(1, 7)}
    if not isinstance(values, dict) or set(values) != expected:
        raise ValueError("initial pose joint set mismatch")
    positions = {name: float(value) for name, value in values.items()}
    if not all(math.isfinite(value) for value in positions.values()):
        raise ValueError("initial pose contains a non-finite position")
    return positions, hashlib.sha256(data).hexdigest()


def load_j2_session_reference(
    path: Path, persistent_zero_sha256: str
) -> tuple[dict[str, float], dict[str, float], str]:
    data = path.read_bytes()
    document = json.loads(data)
    if document.get("schema") != "go-m8010-j2-power-session-reference/1.0":
        raise ValueError("J2 session reference schema mismatch")
    if document.get("reference_name") != "PERSISTENT_SOFTWARE_ZERO_V1":
        raise ValueError("J2 session reference name mismatch")
    if document.get("parent_persistent_zero_sha256") != persistent_zero_sha256:
        raise ValueError("J2 session reference persistent parent mismatch")
    evidence = document.get("source_evidence")
    if not isinstance(evidence, dict):
        raise ValueError("J2 session reference evidence is missing")
    evidence_path = Path(str(evidence.get("path", ""))).resolve()
    evidence_sha256 = str(evidence.get("sha256", ""))
    if len(evidence_sha256) != 64 or any(
        character not in "0123456789abcdef" for character in evidence_sha256
    ):
        raise ValueError("J2 session reference evidence hash is invalid")
    if not evidence_path.is_file() or hashlib.sha256(
        evidence_path.read_bytes()
    ).hexdigest() != evidence_sha256:
        raise ValueError("J2 session reference evidence hash mismatch")
    confirmation = document.get("operator_confirmation")
    if not isinstance(confirmation, dict) or any(
        confirmation.get(field) is not True
        for field in (
            "vertical_initialization_pose",
            "support_reliable",
            "arm_not_moved",
            "not_at_mechanical_limit",
        )
    ):
        raise ValueError("J2 session reference physical confirmation is incomplete")
    writes = document.get("writes")
    if not isinstance(writes, dict) or any(
        writes.get(field) is not False
        for field in (
            "motor_internal_zero_modified",
            "rid_written",
            "flash_or_eeprom_written",
        )
    ):
        raise ValueError("J2 session reference records a forbidden write")
    authority = document.get("control_authority")
    if not isinstance(authority, dict) or any(
        authority.get(field) is not False
        for field in (
            "is_software_zero",
            "authorizes_active_control",
            "authorizes_motor_internal_write",
        )
    ):
        raise ValueError("J2 session reference scope is invalid")
    motors = document.get("motors")
    expected_signs = {"J2A": -1, "J2B": +1}
    if not isinstance(motors, dict) or set(motors) != set(expected_signs):
        raise ValueError("J2 session reference must contain exactly J2A and J2B")
    references: dict[str, float] = {}
    hints: dict[str, float] = {}
    for name, sign in expected_signs.items():
        record = motors.get(name)
        if not isinstance(record, dict):
            raise ValueError(f"J2 session reference {name} record is invalid")
        reference = float(record.get("session_reference_raw_rad"))
        logical = float(record.get("logical_position_rad"))
        span = float(record.get("sample_span_raw_rad"))
        sample_count = record.get("sample_count")
        record_sign = record.get("sign")
        gear_ratio = float(record.get("gear_ratio"))
        if not all(math.isfinite(value) for value in (reference, logical, span, gear_ratio)):
            raise ValueError(f"J2 session reference {name} contains non-finite data")
        if (
            abs(logical) > 1.0e-9
            or span < 0.0
            or span / GEAR_RATIO > math.radians(0.20)
            or type(sample_count) is not int
            or sample_count < 450
            or type(record_sign) is not int
            or record_sign != sign
            or not math.isclose(gear_ratio, GEAR_RATIO, rel_tol=0.0, abs_tol=1.0e-6)
        ):
            raise ValueError(f"J2 session reference {name} value is invalid")
        references[name] = reference
        hints[name] = logical
    return references, hints, hashlib.sha256(data).hexdigest()


def load_go_aux_session_reference(
    path: Path,
    persistent_zero_sha256: str,
    recovery_hint_sha256: str,
    initial_pose_sha256: str,
    initial_pose: dict[str, float],
) -> tuple[dict[str, float], dict[str, float], str]:
    data = path.read_bytes()
    document = json.loads(data)
    if document.get("schema") != "go-m8010-go-aux-power-session-reference/1.0":
        raise ValueError("GO-AUX session reference schema mismatch")
    if document.get("reference_name") != "PERSISTENT_SOFTWARE_ZERO_V1":
        raise ValueError("GO-AUX session reference name mismatch")
    if document.get("parent_persistent_zero_sha256") != persistent_zero_sha256:
        raise ValueError("GO-AUX session reference persistent parent mismatch")
    preserved = document.get("preserved_inputs")
    if not isinstance(preserved, dict) or (
        preserved.get("recovery_branch_hints_sha256") != recovery_hint_sha256
        or preserved.get("initial_pose_sha256") != initial_pose_sha256
        or preserved.get("overwritten_or_deleted") is not False
        or preserved.get("recovery_hints_used_for_go_aux") is not False
    ):
        raise ValueError("GO-AUX session preserved-input binding mismatch")
    evidence = document.get("source_evidence")
    if not isinstance(evidence, dict):
        raise ValueError("GO-AUX session evidence is missing")
    evidence_path = Path(str(evidence.get("path", ""))).resolve()
    evidence_sha256 = str(evidence.get("sha256", ""))
    if len(evidence_sha256) != 64 or any(
        character not in "0123456789abcdef" for character in evidence_sha256
    ):
        raise ValueError("GO-AUX session evidence hash is invalid")
    if not evidence_path.is_file() or hashlib.sha256(
        evidence_path.read_bytes()
    ).hexdigest() != evidence_sha256:
        raise ValueError("GO-AUX session evidence hash mismatch")
    confirmation = document.get("operator_confirmation")
    if not isinstance(confirmation, dict) or any(
        confirmation.get(field) is not True
        for field in (
            "vertical_initialization_pose",
            "support_reliable",
            "arm_not_moved",
            "not_at_mechanical_limit",
        )
    ):
        raise ValueError("GO-AUX session physical confirmation is incomplete")
    writes = document.get("writes")
    if not isinstance(writes, dict) or any(
        writes.get(field) is not False
        for field in (
            "motor_internal_zero_modified",
            "rid_written",
            "flash_or_eeprom_written",
        )
    ):
        raise ValueError("GO-AUX session reference records a forbidden write")
    authority = document.get("control_authority")
    if not isinstance(authority, dict) or any(
        authority.get(field) is not False
        for field in (
            "is_software_zero",
            "authorizes_active_control",
            "authorizes_motor_internal_write",
        )
    ):
        raise ValueError("GO-AUX session reference scope is invalid")
    expected_signs = {"J1": +1, "J3": +1, "J4": +1, "J5": +1}
    motors = document.get("motors")
    if not isinstance(motors, dict) or set(motors) != set(expected_signs):
        raise ValueError("GO-AUX session reference motor set mismatch")
    references: dict[str, float] = {}
    logical_positions: dict[str, float] = {}
    for name, sign in expected_signs.items():
        record = motors.get(name)
        if not isinstance(record, dict):
            raise ValueError(f"GO-AUX session reference {name} record is invalid")
        reference = float(record.get("session_reference_raw_rad"))
        logical = float(record.get("logical_position_rad"))
        span = float(record.get("sample_span_raw_rad"))
        sample_count = record.get("sample_count")
        gear_ratio = float(record.get("gear_ratio"))
        if not all(
            math.isfinite(value) for value in (reference, logical, span, gear_ratio)
        ) or (
            not math.isclose(logical, initial_pose[name], rel_tol=0.0, abs_tol=1.0e-9)
            or span < 0.0
            or span / GEAR_RATIO > math.radians(0.20)
            or type(sample_count) is not int
            or sample_count < 500
            or type(record.get("sign")) is not int
            or record.get("sign") != sign
            or not math.isclose(gear_ratio, GEAR_RATIO, rel_tol=0.0, abs_tol=1.0e-6)
        ):
            raise ValueError(f"GO-AUX session reference {name} value is invalid")
        references[name] = reference
        logical_positions[name] = logical
    return references, logical_positions, hashlib.sha256(data).hexdigest()


class CsvCapture:
    def __init__(self, path: Path, fieldnames: Iterable[str]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = path.open("w", encoding="utf-8", newline="")
        self.writer = csv.DictWriter(self.stream, fieldnames=list(fieldnames))
        self.writer.writeheader()
        self.rows = 0

    def write(self, row: dict) -> None:
        self.writer.writerow(row)
        self.rows += 1
        if self.rows % 50 == 0:
            self.stream.flush()

    def close(self) -> None:
        if not self.stream.closed:
            self.stream.flush()
            self.stream.close()


class WholeArmStateNode(Node):
    def __init__(self) -> None:
        super().__init__("whole_arm_state_node")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("monitor_rate_hz", 2.0)
        self.declare_parameter("feedback_topic", "/whole_arm/motor_feedback_raw")
        self.declare_parameter("udp_bind", "127.0.0.1")
        self.declare_parameter("udp_port", 15300)
        self.declare_parameter("capture_samples", 50)
        self.declare_parameter("feedback_freshness_s", 0.10)
        self.declare_parameter("capture_max_span_deg", 0.20)
        self.declare_parameter("capture_max_tail_drift_deg", 0.10)
        self.declare_parameter("evidence_directory", "logs/arm_gui")
        self.declare_parameter("persistent_zero_path", "")
        self.declare_parameter("recovery_hint_path", "")
        self.declare_parameter("initial_pose_path", "")
        self.declare_parameter("j2_session_reference_path", "")
        self.declare_parameter("go_aux_session_reference_path", "")
        self.declare_parameter("worker_supervisor_freshness_s", 1.0)

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        monitor_rate_hz = float(self.get_parameter("monitor_rate_hz").value)
        if rate_hz < 50.0 or monitor_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be >=50 and monitor_rate_hz >0")
        zero_path_text = str(self.get_parameter("persistent_zero_path").value)
        persistent_references = None
        self.persistent_zero_sha256: Optional[str] = None
        if zero_path_text:
            persistent_references, self.persistent_zero_sha256 = load_persistent_zero(
                Path(zero_path_text).resolve()
            )
        hint_path_text = str(self.get_parameter("recovery_hint_path").value)
        recovery_hints = (
            load_recovery_hints(Path(hint_path_text).resolve())
            if hint_path_text
            else None
        )
        recovery_hint_sha256 = (
            hashlib.sha256(Path(hint_path_text).resolve().read_bytes()).hexdigest()
            if hint_path_text
            else None
        )
        initial_pose_path_text = str(
            self.get_parameter("initial_pose_path").value
        )
        initial_pose = None
        self.initial_pose_sha256: Optional[str] = None
        if initial_pose_path_text:
            if self.persistent_zero_sha256 is None:
                raise ValueError("initial pose requires a persistent parent")
            initial_pose, self.initial_pose_sha256 = load_initial_pose(
                Path(initial_pose_path_text).resolve(),
                self.persistent_zero_sha256,
            )
        session_path_text = str(
            self.get_parameter("j2_session_reference_path").value
        )
        j2_session_references = None
        j2_session_hints = None
        self.j2_session_reference_sha256: Optional[str] = None
        if session_path_text:
            if self.persistent_zero_sha256 is None:
                raise ValueError("J2 session reference requires a persistent parent")
            (
                j2_session_references,
                j2_session_hints,
                self.j2_session_reference_sha256,
            ) = load_j2_session_reference(
                Path(session_path_text).resolve(), self.persistent_zero_sha256
            )
        aux_session_path_text = str(
            self.get_parameter("go_aux_session_reference_path").value
        )
        go_aux_session_references = None
        go_aux_session_hints = None
        self.go_aux_session_reference_sha256: Optional[str] = None
        if aux_session_path_text:
            if (
                self.persistent_zero_sha256 is None
                or recovery_hint_sha256 is None
                or self.initial_pose_sha256 is None
                or initial_pose is None
            ):
                raise ValueError(
                    "GO-AUX session reference requires persistent zero, hints, and initial pose"
                )
            (
                go_aux_session_references,
                go_aux_session_hints,
                self.go_aux_session_reference_sha256,
            ) = load_go_aux_session_reference(
                Path(aux_session_path_text).resolve(),
                self.persistent_zero_sha256,
                recovery_hint_sha256,
                self.initial_pose_sha256,
                initial_pose,
            )
        self.model = MirrorSessionReferenceV1(
            capture_samples=int(self.get_parameter("capture_samples").value),
            freshness_s=float(self.get_parameter("feedback_freshness_s").value),
            capture_max_span_joint_rad=math.radians(
                float(self.get_parameter("capture_max_span_deg").value)
            ),
            capture_max_tail_drift_joint_rad=math.radians(
                float(self.get_parameter("capture_max_tail_drift_deg").value)
            ),
            persistent_references=persistent_references,
            recovery_hints=recovery_hints,
            j2_session_references=j2_session_references,
            j2_session_hints=j2_session_hints,
            go_aux_session_references=go_aux_session_references,
            go_aux_session_hints=go_aux_session_hints,
        )
        self.velocity_observer = PositionSpanVelocityObserver()
        evidence = Path(str(self.get_parameter("evidence_directory").value)).resolve()
        worker_supervisor_freshness_s = float(
            self.get_parameter("worker_supervisor_freshness_s").value
        )
        if not 0.25 <= worker_supervisor_freshness_s <= 5.0:
            raise ValueError(
                "worker_supervisor_freshness_s must be between 0.25 and 5.0"
            )
        self.worker_supervisor_status_path = (
            evidence / WORKER_SUPERVISOR_STATUS_FILENAME
        )
        self.worker_supervisor_freshness_ns = int(
            worker_supervisor_freshness_s * 1.0e9
        )
        common_fields = [
            "sequence", "ros_time_ns", "monotonic_ns", "healthy", "j2_e_sync_rad",
            *[f"{name}_position_rad" for name in JOINT_NAMES],
            *[f"{name}_velocity_rad_s" for name in JOINT_NAMES],
            *[f"{name}_temperature_c" for name in MOTOR_NAMES],
            *[f"{name}_merror" for name in MOTOR_NAMES],
            *[f"{name}_communication_ok" for name in MOTOR_NAMES],
            *[f"{name}_age_ms" for name in MOTOR_NAMES],
        ]
        self.whole_capture = CsvCapture(evidence / "whole_arm_state_capture.csv", common_fields)
        self.joint_capture = CsvCapture(
            evidence / "joint_states_capture.csv",
            ["sequence", "ros_time_ns", *[f"{name}_position_rad" for name in JOINT_NAMES],
             *[f"{name}_velocity_rad_s" for name in JOINT_NAMES]],
        )
        feedback_topic = str(self.get_parameter("feedback_topic").value)
        self.subscription = self.create_subscription(String, feedback_topic, self.on_feedback, 50)
        self.udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_socket.setblocking(False)
        self.udp_socket.bind((
            str(self.get_parameter("udp_bind").value),
            int(self.get_parameter("udp_port").value),
        ))
        self.udp_timer = self.create_timer(0.002, self.poll_udp)
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.status_publisher = self.create_publisher(String, "/whole_arm/hardware_state", 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_state)
        self.monitor_timer = self.create_timer(1.0 / monitor_rate_hz, self.monitor)
        self.sequence = 0
        self.state_instance_id = secrets.token_hex(16)
        self.invalid_payload_count = 0
        self.publish_intervals_ms: list[float] = []
        self.source_latencies_ms: list[float] = []
        self.last_publish_ns: Optional[int] = None
        self.last_snapshot: Optional[dict] = None
        self.controller_modes: dict[str, str] = {}
        self.controller_faults: dict[str, bool] = {}
        self.controller_lease_safe_hold: dict[str, bool] = {}
        self.j2_sync_fault = False
        self.reference_announced = False
        self.session_id: Optional[str] = None
        self.warning_last_logged_ns: dict[str, int] = {}
        self.warning_suppressed: dict[str, int] = {}
        self.get_logger().info(
            f"state-only aggregator listening on {feedback_topic}; no actuator command API exists"
        )

    def warn_rate_limited(self, key: str, text: str) -> None:
        """Keep exact warning counts without writing the same line every frame."""
        now_ns = time.monotonic_ns()
        last_ns = self.warning_last_logged_ns.get(key)
        if last_ns is None:
            self.warning_last_logged_ns[key] = now_ns
            self.warning_suppressed[key] = 0
            self.get_logger().warning(text)
            return
        if now_ns - last_ns < WARNING_LOG_INTERVAL_NS:
            self.warning_suppressed[key] = self.warning_suppressed.get(key, 0) + 1
            return
        suppressed = self.warning_suppressed.get(key, 0)
        self.warning_last_logged_ns[key] = now_ns
        self.warning_suppressed[key] = 0
        self.get_logger().warning(
            f"{text}; suppressed_since_last={suppressed}"
        )

    def record_invalid_payload(self, error: Exception) -> None:
        self.invalid_payload_count += 1
        reason = feedback_rejection_reason(error)
        self.warn_rate_limited(
            f"raw_feedback_rejected:{reason}",
            f"rejected raw feedback payload: reason={reason}; "
            f"rejected_total={self.invalid_payload_count}",
        )

    def worker_control_status(self, now_monotonic_ns: int) -> dict:
        """Return independently supervised command-receiver availability."""

        try:
            status = read_worker_supervisor_status_file(
                self.worker_supervisor_status_path,
                now_monotonic_ns,
                self.worker_supervisor_freshness_ns,
            )
        except FileNotFoundError:
            status = unavailable_worker_control_status(
                "supervisor_status_missing"
            )
        except WorkerSupervisorStatusError as exc:
            status = unavailable_worker_control_status(exc.reason)
        except OSError:
            status = unavailable_worker_control_status(
                "supervisor_status_invalid"
            )
        for domain, available in status["control_available_by_domain"].items():
            if available:
                continue
            reason = status["control_reason_by_domain"][domain]
            self.warn_rate_limited(
                f"worker_control_unavailable:{domain}:{reason}",
                f"command receiver unavailable: domain={domain}; reason={reason}",
            )
        return status

    def on_feedback(self, message: String) -> None:
        self.accept_payload(message.data, time.monotonic_ns())

    def poll_udp(self) -> None:
        for _ in range(100):
            try:
                data, _address = self.udp_socket.recvfrom(65535)
            except BlockingIOError:
                return
            try:
                text = data.decode("utf-8")
            except UnicodeDecodeError as exc:
                self.record_invalid_payload(exc)
                continue
            self.accept_payload(text, time.monotonic_ns())

    def accept_payload(self, text: str, receipt_ns: int) -> None:
        try:
            payload = json.loads(text)
            if not isinstance(payload, dict):
                raise ValueError("feedback payload must be an object")
            if payload.get("schema") != "go-m8010-motor-feedback/1.0":
                raise ValueError("feedback payload schema mismatch")
            samples = list(parse_feedback_payload(payload, receipt_ns))
            controller_mode = str(payload.get("controller_mode", "unknown"))
            if controller_mode not in CONTROLLER_MODES:
                raise ValueError("controller mode is invalid")
            controller_mode_by_motor = payload.get("controller_mode_by_motor", {})
            if not isinstance(controller_mode_by_motor, dict):
                raise ValueError("controller_mode_by_motor must be an object")
            if "domain_fault" not in payload:
                raise ValueError("domain_fault is required")
            controller_fault = payload["domain_fault"]
            if type(controller_fault) is not bool:
                raise ValueError("domain_fault must be boolean")
            lease_safe_hold = payload.get("lease_safe_hold", False)
            if type(lease_safe_hold) is not bool:
                raise ValueError("lease_safe_hold must be boolean")
            if any(sample.motor in J2_MOTOR_NAMES for sample in samples):
                if "j2_sync_fault" not in payload:
                    raise ValueError("j2_sync_fault is required for J2 feedback")
                if type(payload["j2_sync_fault"]) is not bool:
                    raise ValueError("j2_sync_fault must be boolean")
            mode_updates = {}
            for sample in samples:
                motor_mode = str(
                    controller_mode_by_motor.get(sample.motor, controller_mode)
                )
                if motor_mode not in CONTROLLER_MODES:
                    raise ValueError("controller mode is invalid")
                mode_updates[sample.motor] = motor_mode
            next_j2_sync_fault = update_j2_sync_fault(
                self.j2_sync_fault, samples, payload
            )
            # The model and safety metadata form one accepted datagram. The
            # transactional model update must finish before any confirmation
            # or J2 interlock flag becomes externally visible.
            self.model.update_batch(samples)
            self.j2_sync_fault = next_j2_sync_fault
            for sample in samples:
                self.controller_modes[sample.motor] = mode_updates[sample.motor]
                self.controller_faults[sample.motor] = controller_fault
                self.controller_lease_safe_hold[sample.motor] = lease_safe_hold
        except Exception as exc:
            self.record_invalid_payload(exc)

    def publish_state(self) -> None:
        now_monotonic_ns = time.monotonic_ns()
        if not self.model.try_capture_available(now_monotonic_ns):
            return
        if not self.reference_announced:
            try:
                boot_id = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip()
            except OSError:
                boot_id = "boot-id-unavailable"
            if self.persistent_zero_sha256:
                j2_suffix = (
                    f":j2session:{self.j2_session_reference_sha256[:16]}"
                    if self.j2_session_reference_sha256
                    else ":j2session:blocked"
                )
                aux_suffix = (
                    f":goauxsession:{self.go_aux_session_reference_sha256[:16]}"
                    if self.go_aux_session_reference_sha256
                    else ":goauxsession:blocked"
                )
                self.session_id = (
                    f"persistent:{self.persistent_zero_sha256[:16]}"
                    f"{j2_suffix}{aux_suffix}"
                )
                self.get_logger().info(
                    "PERSISTENT_SOFTWARE_ZERO_V1 已从磁盘加载；"
                    "J2 及 J1/J3/J4/J5 仅在父哈希绑定的本次上电会话参考"
                    "存在时可用；电机内部零位与RID未改写"
                )
            else:
                self.session_id = f"{boot_id}:{self.model.captured_monotonic_ns}"
                self.get_logger().info(
                    "SESSION_REFERENCE_V1 已从当前可用电机独立捕获；缺失关节不会阻止界面启动；"
                    "CAD_ZERO=PENDING ROS_ZERO=PENDING"
                )
            self.reference_announced = True
        try:
            snapshot = self.model.snapshot_available(now_monotonic_ns)
        except RuntimeError as exc:
            self.warn_rate_limited("snapshot_unavailable", str(exc))
            return
        observed_velocity = self.velocity_observer.update(
            now_monotonic_ns, snapshot["position_rad"]
        )
        if observed_velocity is None:
            snapshot["velocity_source"] = "RAW_MOTOR_WARMUP"
        else:
            snapshot["velocity_rad_s"] = observed_velocity
            snapshot["velocity_source"] = "POSITION_SPAN_0P40S"

        stamp = self.get_clock().now().to_msg()
        ros_time_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = list(JOINT_NAMES)
        joint_state.position = list(snapshot["position_rad"])
        joint_state.velocity = list(snapshot["velocity_rad_s"])
        self.publisher.publish(joint_state)

        if self.last_publish_ns is not None:
            self.publish_intervals_ms.append((now_monotonic_ns - self.last_publish_ns) / 1.0e6)
            self.publish_intervals_ms = self.publish_intervals_ms[-5000:]
        self.last_publish_ns = now_monotonic_ns
        self.sequence += 1
        row = {
            "sequence": self.sequence,
            "ros_time_ns": ros_time_ns,
            "monotonic_ns": now_monotonic_ns,
            "healthy": int(snapshot["healthy"]),
            "j2_e_sync_rad": snapshot["j2_e_sync_rad"],
        }
        joint_row = {"sequence": self.sequence, "ros_time_ns": ros_time_ns}
        for index, name in enumerate(JOINT_NAMES):
            row[f"{name}_position_rad"] = snapshot["position_rad"][index]
            row[f"{name}_velocity_rad_s"] = snapshot["velocity_rad_s"][index]
            joint_row[f"{name}_position_rad"] = snapshot["position_rad"][index]
            joint_row[f"{name}_velocity_rad_s"] = snapshot["velocity_rad_s"][index]
        for name in MOTOR_NAMES:
            motor = snapshot["per_motor"][name]
            row[f"{name}_temperature_c"] = motor["temperature_c"]
            row[f"{name}_merror"] = motor["merror"]
            row[f"{name}_communication_ok"] = int(motor["communication_ok"])
            row[f"{name}_age_ms"] = motor["age_ms"]
            if motor["source_latency_ms"] is not None:
                self.source_latencies_ms.append(float(motor["source_latency_ms"]))
        self.source_latencies_ms = self.source_latencies_ms[-35000:]
        self.whole_capture.write(row)
        self.joint_capture.write(joint_row)

        snapshot["schema"] = "go-m8010-hardware-state/1.1"
        snapshot["sequence"] = self.sequence
        snapshot["state_instance_id"] = self.state_instance_id
        snapshot["source_monotonic_ns"] = now_monotonic_ns
        snapshot["session_id"] = self.session_id
        snapshot["invalid_payload_count"] = self.invalid_payload_count
        snapshot["cad_zero"] = "PENDING"
        snapshot["ros_zero"] = (
            "PERSISTENT_SOFTWARE_ZERO_V1"
            if self.persistent_zero_sha256
            else "PENDING"
        )
        snapshot["persistent_zero_sha256"] = self.persistent_zero_sha256
        snapshot["initial_pose_sha256"] = self.initial_pose_sha256
        snapshot["j2_session_reference_sha256"] = (
            self.j2_session_reference_sha256
        )
        snapshot["go_aux_session_reference_sha256"] = (
            self.go_aux_session_reference_sha256
        )
        snapshot["j2_active_motion_used"] = False
        snapshot["zero_gravity"] = "NOT_IMPLEMENTED"
        snapshot["controller_mode_by_motor"] = {
            name: (
                self.controller_modes.get(name, "unknown")
                if snapshot["per_motor"][name]["fresh"]
                else "unknown"
            )
            for name in MOTOR_NAMES
        }
        snapshot["controller_fault_by_motor"] = {
            name: bool(self.controller_faults.get(name, False))
            for name in MOTOR_NAMES
        }
        snapshot["lease_safe_hold_by_motor"] = {
            name: bool(
                self.controller_lease_safe_hold.get(name, False)
                and self.controller_modes.get(name) == "hold"
                and snapshot["per_motor"][name]["fresh"]
                and snapshot["per_motor"][name]["communication_ok"]
                and snapshot["per_motor"][name]["merror"] == 0
                and not self.controller_faults.get(name, False)
            )
            for name in MOTOR_NAMES
        }
        snapshot["j2_sync_fault"] = self.j2_sync_fault
        # Feedback freshness and controller_mode describe only observed
        # telemetry.  PID/UDP-owner command-receiver liveness is a separate,
        # fail-closed supervisor contract and must never be inferred from them.
        snapshot.update(self.worker_control_status(now_monotonic_ns))
        snapshot["timing"] = self.timing_summary()
        self.status_publisher.publish(String(data=json.dumps(snapshot, ensure_ascii=False)))
        self.last_snapshot = snapshot

    def timing_summary(self) -> dict:
        intervals = self.publish_intervals_ms
        latencies = self.source_latencies_ms
        median_rate = None
        if intervals:
            median_interval = statistics.median(intervals)
            median_rate = 1000.0 / median_interval if median_interval > 0.0 else None
        return {
            "median_publish_rate_hz": median_rate,
            "p95_source_latency_ms": percentile(latencies, 0.95),
        }

    def monitor(self) -> None:
        if self.last_snapshot is None:
            counts = {name: len(self.model.history[name]) for name in MOTOR_NAMES}
            self.get_logger().info(f"等待七路新鲜反馈以捕获会话参考: {counts}")
            return
        snapshot = self.last_snapshot
        degrees = [math.degrees(value) for value in snapshot["position_rad"]]
        health = "OK" if snapshot["healthy"] else "DEGRADED"
        temperatures = ",".join(
            f"{name}:{snapshot['per_motor'][name]['temperature_c']:.0f}C"
            for name in MOTOR_NAMES
        )
        errors = ",".join(
            f"{name}:{snapshot['per_motor'][name]['merror']}" for name in MOTOR_NAMES
        )
        self.get_logger().info(
            " ".join(f"J{i + 1}={value:+.2f}deg" for i, value in enumerate(degrees))
            + f" J2_e_sync={math.degrees(snapshot['j2_e_sync_rad']):+.3f}deg"
            + f" COMM={health} TEMP=[{temperatures}] MERROR=[{errors}]"
        )

    def destroy_node(self) -> bool:
        self.udp_socket.close()
        self.whole_capture.close()
        self.joint_capture.close()
        return super().destroy_node()


def percentile(values: list[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def install_shutdown_handlers(stop_requested: threading.Event) -> dict[int, object]:
    """Keep ROS/resource teardown out of asynchronous signal handlers."""

    previous_handlers = {}

    def request_stop(_signum, _frame) -> None:
        stop_requested.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, request_stop)
    return previous_handlers


def restore_shutdown_handlers(previous_handlers: dict[int, object]) -> None:
    for signum, handler in previous_handlers.items():
        signal.signal(signum, handler)


def main(args=None) -> None:
    stop_requested = threading.Event()
    previous_handlers = install_shutdown_handlers(stop_requested)
    initialized = False
    node: Optional[WholeArmStateNode] = None
    try:
        # Do not let rclpy invalidate the context asynchronously while timers,
        # UDP and CSV resources are still owned by this node.  The main thread
        # performs the complete teardown below after observing the stop flag.
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        if stop_requested.is_set():
            return
        node = WholeArmStateNode()
        while not stop_requested.is_set() and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
    except ExternalShutdownException:
        pass
    except Exception:
        # Humble may raise its private _rclpy.RCLError after a signal has
        # already invalidated the context. Preserve every real runtime error.
        if rclpy.ok():
            raise
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            try:
                if initialized and rclpy.ok():
                    rclpy.shutdown()
            finally:
                restore_shutdown_handlers(previous_handlers)


if __name__ == "__main__":
    main()

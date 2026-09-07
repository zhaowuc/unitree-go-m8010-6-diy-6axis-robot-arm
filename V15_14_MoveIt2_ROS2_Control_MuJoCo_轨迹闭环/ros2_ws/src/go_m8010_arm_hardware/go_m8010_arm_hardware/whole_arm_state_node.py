"""ROS 2 state source: seven physical feedback streams to six logical joints.

The node deliberately exposes no command subscriber, trajectory action,
controller API, serial port, CAN transport, HOLD, FOC, or BRAKE operation.
The bus owners send raw feedback over UDP. Accepted packets are published
unchanged on ``/whole_arm/motor_feedback_raw`` for independent observers.
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
from collections import deque
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
    parse_controller_feedback_metadata,
    parse_feedback_payload,
    unavailable_worker_control_status,
    validate_j6_feedback_identity,
    validate_worker_supervisor_status,
    validate_preserved_session_reference,
    supported_near_vertical_recovery,
)
from .thermal_manager import (
    THERMAL_CONFIG_SHA256,
    ThermalLimits,
    load_thermal_limits,
    thermal_state_from_controller_metadata,
)


J2_MOTOR_NAMES = frozenset({"J2A", "J2B"})
WARNING_LOG_INTERVAL_NS = 5_000_000_000
CONTROLLER_MODES = frozenset({"brake", "drag", "hold", "position", "teach", "unknown"})
WORKER_SUPERVISOR_STATUS_FILENAME = "worker_supervisor_status.json"
WORKER_SUPERVISOR_STATUS_MAX_BYTES = 16_384


def validated_state_instance_id(configured: str) -> str:
    """Return one strict launch-pinned state identity or a random default."""

    if configured == "":
        return secrets.token_hex(16)
    if len(configured) != 32 or any(
        character not in "0123456789abcdef" for character in configured
    ):
        raise ValueError(
            "state_instance_id must be exactly 32 lowercase hexadecimal characters"
        )
    return configured


def complete_persistent_session_id(
    persistent_zero_sha256: Optional[str],
    j2_session_reference_sha256: Optional[str],
    go_aux_session_reference_sha256: Optional[str],
) -> Optional[str]:
    """Build the immutable session identity when every reference is loaded."""

    if not all(
        (
            persistent_zero_sha256,
            j2_session_reference_sha256,
            go_aux_session_reference_sha256,
        )
    ):
        return None
    return (
        f"persistent:{persistent_zero_sha256[:16]}"
        f":j2session:{j2_session_reference_sha256[:16]}"
        f":goauxsession:{go_aux_session_reference_sha256[:16]}"
    )


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


def controller_metadata_for_hardware_state(
    record: Optional[dict], motor_state: dict, thermal_limits: ThermalLimits,
) -> dict:
    """Pair safety metadata with exactly the fresh encoder sample it described."""

    current = bool(
        isinstance(record, dict)
        and motor_state.get("fresh") is True
        and record.get("source_monotonic_ns")
        == motor_state.get("feedback_source_monotonic_ns")
        and record.get("receipt_monotonic_ns")
        == motor_state.get("feedback_receipt_monotonic_ns")
    )
    if not current:
        return {
            "metadata_status": "UNKNOWN",
            "controller_mode": "unknown",
            "domain_fault": None,
            "lease_safe_hold": None,
            "brake_observed": None,
            "thermal": {
                "metadata_status": "UNKNOWN",
                "state": "UNKNOWN",
                "fault_latched": None,
                "cooldown_ready": None,
                "release_observed": None,
                "rearm_pending_next_cycle": None,
                "cooldown_valid_brake_frames": None,
                "trip_activation_epoch": None,
                "minimum_rearm_epoch": None,
                "domain_reported_state": None,
                "reported_state_by_motor": None,
                "reported_state": None,
                "reported_state_consistent": None,
                "trip_reason": None,
                "thermal_config_sha256": None,
                "derating_factor": None,
                "raw_temperature_c": None,
                "window_median_c": None,
                "slope_c_per_min": None,
            },
            "no_progress": {
                "metadata_status": "UNKNOWN",
                "fault_latched": None,
                "release_observed": None,
                "rearm_pending_next_cycle": None,
                "qualifying_frames": None,
                "observation_valid": None,
                "position_error_rad": None,
                "trip_position_error_rad": None,
                "software_saturation_observed": None,
                "watchdog_authority": None,
                "continuous_rating_authoritative": False,
                "trip_activation_epoch": None,
                "minimum_rearm_epoch": None,
            },
            "gravity": {
                "metadata_status": "UNKNOWN",
                "authority_present": None,
                "scale": None,
                "scale_target": None,
                "feedforward_nm": None,
                "applied_rotor_nm": None,
            },
        }

    assert record is not None
    thermal = dict(record["thermal"])
    if thermal["metadata_status"] == "OBSERVED":
        temperature = motor_state.get("temperature_c")
        if (
            type(temperature) in {int, float}
            and math.isfinite(float(temperature))
            and float(temperature) < 0.0
        ):
            derived_thermal_state = "OFFLINE"
        else:
            derived_thermal_state = thermal_state_from_controller_metadata(
                motor_state.get("temperature_c"),
                fault_latched=thermal["fault_latched"],
                cooldown_ready=thermal["cooldown_ready"],
                limits=thermal_limits,
            ).value
        reported_state = thermal.get("reported_state")
        reported_state_consistent = (
            None if reported_state is None
            else reported_state == derived_thermal_state
        )
        # A contradictory worker display state is not promoted to a healthy
        # shared state.  The raw latch/temperature remain available to diagnose
        # the producer, while motion authorization stays fail-closed.
        thermal_state = (
            "UNKNOWN"
            if reported_state_consistent is False
            else derived_thermal_state
        )
    else:
        temperature = motor_state.get("temperature_c")
        thermal_state = (
            "THERMAL_STOP"
            if type(temperature) in {int, float}
            and math.isfinite(float(temperature))
            and float(temperature) >= thermal_limits.thermal_stop_c
            else "UNKNOWN"
        )
        reported_state_consistent = None
    thermal.update({
        "state": thermal_state,
        "reported_state_consistent": reported_state_consistent,
    })

    mode = record["controller_mode"]
    communication_ok = bool(
        motor_state.get("communication_ok") is True
        and motor_state.get("merror") == 0
    )
    return {
        "metadata_status": "OBSERVED",
        "controller_mode": mode,
        "domain_fault": record["domain_fault"],
        "lease_safe_hold": record["lease_safe_hold"],
        "brake_observed": bool(communication_ok and mode == "brake"),
        "thermal": thermal,
        "no_progress": dict(record["no_progress"]),
        "gravity": {
            **record["gravity"],
            "feedforward_nm": (
                None
                if record["gravity"]["feedforward_nm"] is None
                else list(record["gravity"]["feedforward_nm"])
            ),
        },
    }


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
    path: Path, persistent_zero_sha256: str, *,
    runtime_startup_hints: Optional[dict[str, float]] = None,
) -> tuple[dict[str, float], dict[str, float], str]:
    data = path.read_bytes()
    document = json.loads(data)
    measured_startup = validate_preserved_session_reference(document)
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
    if runtime_startup_hints is not None and supported_near_vertical_recovery(document):
        runtime_startup_hints.update(measured_startup)
    return references, hints, hashlib.sha256(data).hexdigest()


def load_go_aux_session_reference(
    path: Path,
    persistent_zero_sha256: str,
    recovery_hint_sha256: str,
    initial_pose_sha256: str,
    initial_pose: dict[str, float],
    *, runtime_startup_hints: Optional[dict[str, float]] = None,
) -> tuple[dict[str, float], dict[str, float], str]:
    data = path.read_bytes()
    document = json.loads(data)
    measured_startup = validate_preserved_session_reference(document)
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
    expected_signs = {"J1": +1, "J3": +1, "J4": -1, "J5": +1}
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
    if runtime_startup_hints is not None and supported_near_vertical_recovery(document):
        runtime_startup_hints.update(measured_startup)
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
        self.declare_parameter("thermal_config_path", "")
        self.declare_parameter("worker_supervisor_freshness_s", 1.0)
        self.declare_parameter("state_instance_id", "")

        state_instance_id = validated_state_instance_id(
            str(self.get_parameter("state_instance_id").value)
        )
        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        monitor_rate_hz = float(self.get_parameter("monitor_rate_hz").value)
        if rate_hz < 50.0 or monitor_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be >=50 and monitor_rate_hz >0")
        thermal_config_text = str(
            self.get_parameter("thermal_config_path").value
        )
        if not thermal_config_text:
            raise ValueError("thermal_config_path is required")
        self.thermal_limits = load_thermal_limits(
            Path(thermal_config_text).resolve()
        )
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
        runtime_startup_hints: dict[str, float] = {}
        self.j2_session_reference_sha256: Optional[str] = None
        if session_path_text:
            if self.persistent_zero_sha256 is None:
                raise ValueError("J2 session reference requires a persistent parent")
            (
                j2_session_references,
                j2_session_hints,
                self.j2_session_reference_sha256,
            ) = load_j2_session_reference(
                Path(session_path_text).resolve(), self.persistent_zero_sha256,
                runtime_startup_hints=runtime_startup_hints,
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
                runtime_startup_hints=runtime_startup_hints,
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
            runtime_startup_hints=runtime_startup_hints,
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
        self.raw_feedback_publisher = self.create_publisher(String, feedback_topic, 100)
        # ponytail: bound exact, one-use self echoes to 512 packets; replace
        # with publisher-GID filtering when the installed rclpy exposes it.
        self.pending_raw_echoes = deque(maxlen=512)
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
        self.state_instance_id = state_instance_id
        self.invalid_payload_count = 0
        self.publish_intervals_ms: list[float] = []
        self.source_latencies_ms: list[float] = []
        self.last_publish_ns: Optional[int] = None
        self.last_snapshot: Optional[dict] = None
        self.controller_modes: dict[str, str] = {}
        self.controller_faults: dict[str, bool] = {}
        self.controller_lease_safe_hold: dict[str, bool] = {}
        self.controller_thermal: dict[str, dict] = {}
        # One immutable-by-convention record per latest accepted motor sample.
        # A complete copy is swapped only after the model accepts the same
        # datagram, so publishers never pair new safety metadata with old q.
        self.controller_feedback: dict[str, dict] = {}
        self.j2_sync_fault = False
        self.reference_announced = False
        # With the complete disk-backed reference set, the session identity is
        # known before the first motor frame.  This lets a separately started
        # read-only J6 producer bind its very first feedback datagram without
        # weakening the state node's strict identity check.
        self.session_id = complete_persistent_session_id(
            self.persistent_zero_sha256,
            self.j2_session_reference_sha256,
            self.go_aux_session_reference_sha256,
        )
        self.j6_feedback_identity: Optional[dict[str, object]] = None
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
        if message.data in self.pending_raw_echoes:
            self.pending_raw_echoes.remove(message.data)
            return
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
            if self.accept_payload(text, time.monotonic_ns()):
                message = String()
                message.data = text
                self.pending_raw_echoes.append(text)
                try:
                    self.raw_feedback_publisher.publish(message)
                except Exception as exc:
                    self.pending_raw_echoes.remove(text)
                    self.warn_rate_limited("raw_feedback_publish_failed", str(exc))

    def accept_payload(self, text: str, receipt_ns: int) -> bool:
        try:
            payload = json.loads(text)
            if not isinstance(payload, dict):
                raise ValueError("feedback payload must be an object")
            if payload.get("schema") != "go-m8010-motor-feedback/1.0":
                raise ValueError("feedback payload schema mismatch")
            samples = list(parse_feedback_payload(payload, receipt_ns))
            next_j6_feedback_identity = self.j6_feedback_identity
            if len(samples) == 1 and samples[0].motor == "J6":
                next_j6_feedback_identity = validate_j6_feedback_identity(
                    payload,
                    receipt_ns,
                    previous=self.j6_feedback_identity,
                    expected_session_id=self.session_id,
                    expected_state_instance_id=self.state_instance_id,
                )
            metadata_updates = parse_controller_feedback_metadata(
                payload, samples
            )
            if any(sample.motor in J2_MOTOR_NAMES for sample in samples):
                if "j2_sync_fault" not in payload:
                    raise ValueError("j2_sync_fault is required for J2 feedback")
                if type(payload["j2_sync_fault"]) is not bool:
                    raise ValueError("j2_sync_fault must be boolean")
            next_j2_sync_fault = update_j2_sync_fault(
                self.j2_sync_fault, samples, payload
            )
            next_controller_feedback = dict(self.controller_feedback)
            next_controller_feedback.update(metadata_updates)
            next_modes = dict(self.controller_modes)
            next_faults = dict(self.controller_faults)
            next_lease_holds = dict(self.controller_lease_safe_hold)
            next_thermal = dict(self.controller_thermal)
            for motor, metadata in metadata_updates.items():
                next_modes[motor] = metadata["controller_mode"]
                next_faults[motor] = metadata["domain_fault"]
                next_lease_holds[motor] = metadata["lease_safe_hold"] is True
                next_thermal[motor] = metadata["thermal"]
            # The model and safety metadata form one accepted datagram. The
            # transactional model update must finish before the complete
            # replacement maps become externally visible.
            self.model.update_batch(samples)
            self.j2_sync_fault = next_j2_sync_fault
            self.controller_feedback = next_controller_feedback
            self.controller_modes = next_modes
            self.controller_faults = next_faults
            self.controller_lease_safe_hold = next_lease_holds
            self.controller_thermal = next_thermal
            self.j6_feedback_identity = next_j6_feedback_identity
            return True
        except Exception as exc:
            self.record_invalid_payload(exc)
            return False

    def publish_state(self) -> None:
        now_monotonic_ns = time.monotonic_ns()
        if not self.model.try_capture_available(now_monotonic_ns):
            return
        if not self.reference_announced:
            try:
                boot_id = Path("/proc/sys/kernel/random/boot_id").read_text(encoding="utf-8").strip()
            except OSError:
                boot_id = "boot-id-unavailable"
            if self.session_id is not None:
                self.get_logger().info(
                    "PERSISTENT_SOFTWARE_ZERO_V1 已从磁盘加载；"
                    "J2 及 J1/J3/J4/J5 已绑定完整的本次上电会话参考；"
                    "电机内部零位与RID未改写"
                )
            elif self.persistent_zero_sha256:
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
        snapshot["j6_raw_feedback_identity"] = (
            None
            if self.j6_feedback_identity is None
            else dict(self.j6_feedback_identity)
        )
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
        feedback_records = self.controller_feedback
        controller_metadata = {
            name: controller_metadata_for_hardware_state(
                feedback_records.get(name), snapshot["per_motor"][name],
                self.thermal_limits,
            )
            for name in MOTOR_NAMES
        }
        for name in MOTOR_NAMES:
            metadata = controller_metadata[name]
            thermal = metadata["thermal"]
            no_progress = metadata["no_progress"]
            gravity = metadata["gravity"]
            snapshot["per_motor"][name].update({
                "controller_metadata_status": metadata["metadata_status"],
                "thermal_metadata_status": thermal["metadata_status"],
                "thermal_state": thermal["state"],
                "thermal_fault_latched": thermal["fault_latched"],
                "thermal_cooldown_ready": thermal["cooldown_ready"],
                "thermal_release_observed": thermal["release_observed"],
                "thermal_rearm_pending_next_cycle": thermal[
                    "rearm_pending_next_cycle"
                ],
                "thermal_cooldown_valid_brake_frames": thermal[
                    "cooldown_valid_brake_frames"
                ],
                "thermal_trip_activation_epoch": thermal[
                    "trip_activation_epoch"
                ],
                "thermal_minimum_rearm_epoch": thermal[
                    "minimum_rearm_epoch"
                ],
                "thermal_domain_reported_state": thermal[
                    "domain_reported_state"
                ],
                "thermal_reported_state": thermal["reported_state"],
                "thermal_reported_state_consistent": thermal[
                    "reported_state_consistent"
                ],
                "thermal_trip_reason": thermal["trip_reason"],
                "thermal_config_sha256": thermal[
                    "thermal_config_sha256"
                ],
                "thermal_derating_factor": thermal["derating_factor"],
                "thermal_raw_temperature_c": thermal["raw_temperature_c"],
                "thermal_window_median_c": thermal["window_median_c"],
                "thermal_slope_c_per_min": thermal["slope_c_per_min"],
                "no_progress_metadata_status": no_progress[
                    "metadata_status"
                ],
                "load_limit_no_progress": no_progress["fault_latched"],
                "no_progress_release_observed": no_progress[
                    "release_observed"
                ],
                "no_progress_rearm_pending_next_cycle": no_progress[
                    "rearm_pending_next_cycle"
                ],
                "no_progress_watchdog_qualifying_frames": no_progress[
                    "qualifying_frames"
                ],
                "no_progress_observation_valid": no_progress[
                    "observation_valid"
                ],
                "no_progress_position_error_rad": no_progress[
                    "position_error_rad"
                ],
                "no_progress_trip_position_error_rad": no_progress[
                    "trip_position_error_rad"
                ],
                "load_limit_watchdog_authority": no_progress[
                    "watchdog_authority"
                ],
                "continuous_rating_authoritative": no_progress[
                    "continuous_rating_authoritative"
                ],
                "no_progress_trip_activation_epoch": no_progress[
                    "trip_activation_epoch"
                ],
                "no_progress_minimum_rearm_epoch": no_progress[
                    "minimum_rearm_epoch"
                ],
                "gravity_metadata_status": gravity["metadata_status"],
                "gravity_authority_present": gravity["authority_present"],
                "gravity_scale": gravity["scale"],
                "gravity_scale_target": gravity["scale_target"],
                "gravity_feedforward_nm": gravity["feedforward_nm"],
                "gravity_feedforward_rotor_nm": gravity[
                    "applied_rotor_nm"
                ],
            })
        snapshot["thermal_state_by_motor"] = {
            name: snapshot["per_motor"][name]["thermal_state"]
            for name in MOTOR_NAMES
        }
        snapshot["thermal_limits"] = self.thermal_limits.as_mapping()
        snapshot["thermal_config_sha256"] = THERMAL_CONFIG_SHA256
        snapshot["thermal_fault_latched_by_motor"] = {
            name: snapshot["per_motor"][name]["thermal_fault_latched"]
            for name in MOTOR_NAMES
        }
        snapshot["thermal_status_by_motor"] = {
            name: dict(controller_metadata[name]["thermal"])
            for name in MOTOR_NAMES
        }
        snapshot["no_progress_status_by_motor"] = {
            name: dict(controller_metadata[name]["no_progress"])
            for name in MOTOR_NAMES
        }
        snapshot["load_limit_no_progress_by_motor"] = {
            name: controller_metadata[name]["no_progress"]["fault_latched"]
            for name in MOTOR_NAMES
        }
        snapshot["gravity_policy_by_motor"] = {
            name: {
                **controller_metadata[name]["gravity"],
                "feedforward_nm": (
                    None
                    if controller_metadata[name]["gravity"]["feedforward_nm"]
                    is None
                    else list(
                        controller_metadata[name]["gravity"]["feedforward_nm"]
                    )
                ),
            }
            for name in MOTOR_NAMES
        }
        snapshot["gravity_feedforward_rotor_nm_by_motor"] = {
            name: controller_metadata[name]["gravity"]["applied_rotor_nm"]
            for name in MOTOR_NAMES
        }
        go_gravity_feedback_observed = all(
            controller_metadata[name]["gravity"]["metadata_status"]
            == "OBSERVED"
            for name in MOTOR_NAMES
            if name != "J6"
        )
        snapshot["gravity_feedback_status"] = (
            "OBSERVED" if go_gravity_feedback_observed else "UNKNOWN"
        )
        # Legacy field retained, but no longer claims that the implemented
        # worker feedback path is absent.
        snapshot["zero_gravity"] = (
            "WORKER_APPLIED_FEEDBACK"
            if go_gravity_feedback_observed else "UNKNOWN"
        )
        snapshot["trajectory_status_by_motor"] = {
            name: {
                "plan_token_id": snapshot["per_motor"][name][
                    "trajectory_plan_token_id"
                ],
                "trajectory_sha256": snapshot["per_motor"][name][
                    "trajectory_sha256"
                ],
                "state": snapshot["per_motor"][name]["trajectory_state"],
                "sample_index": snapshot["per_motor"][name][
                    "trajectory_sample_index"
                ],
                "interval_count": snapshot["per_motor"][name][
                    "trajectory_interval_count"
                ],
            }
            for name in MOTOR_NAMES
        }
        snapshot["controller_mode_by_motor"] = {
            name: controller_metadata[name]["controller_mode"]
            for name in MOTOR_NAMES
        }
        snapshot["controller_fault_by_motor"] = {
            # Compatibility bool for GUI/1.1.  The parallel observation map is
            # authoritative about UNKNOWN; callers must not treat False as a
            # complete observation when metadata_status is UNKNOWN.
            name: controller_metadata[name]["domain_fault"] is True
            for name in MOTOR_NAMES
        }
        snapshot["controller_fault_observed_by_motor"] = {
            name: controller_metadata[name]["domain_fault"]
            for name in MOTOR_NAMES
        }
        snapshot["controller_metadata_status_by_motor"] = {
            name: controller_metadata[name]["metadata_status"]
            for name in MOTOR_NAMES
        }
        snapshot["brake_observed_by_motor"] = {
            name: controller_metadata[name]["brake_observed"]
            for name in MOTOR_NAMES
        }
        snapshot["lease_safe_hold_by_motor"] = {
            name: bool(
                controller_metadata[name]["lease_safe_hold"] is True
                and controller_metadata[name]["controller_mode"] == "hold"
                and snapshot["per_motor"][name]["communication_ok"]
                and snapshot["per_motor"][name]["merror"] == 0
                and controller_metadata[name]["domain_fault"] is False
            )
            for name in MOTOR_NAMES
        }
        snapshot["telemetry_healthy"] = snapshot["healthy"]
        snapshot["safety_metadata_ready"] = all(
            controller_metadata[name]["metadata_status"] == "OBSERVED"
            and controller_metadata[name]["domain_fault"] is False
            and controller_metadata[name]["thermal"]["metadata_status"]
            == "OBSERVED"
            and controller_metadata[name]["thermal"]["state"]
            in {"NORMAL", "WARNING"}
            and controller_metadata[name]["no_progress"]["metadata_status"]
            == "OBSERVED"
            and controller_metadata[name]["no_progress"]["fault_latched"]
            is False
            and (
                name == "J6"
                or controller_metadata[name]["gravity"]["metadata_status"]
                == "OBSERVED"
            )
            for name in MOTOR_NAMES
        )
        snapshot["healthy"] = bool(
            snapshot["telemetry_healthy"]
            and snapshot["safety_metadata_ready"]
            and not self.j2_sync_fault
        )
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

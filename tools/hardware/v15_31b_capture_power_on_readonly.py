#!/usr/bin/env python3
"""Capture the V15.31B power-on READ ONLY qualification from ROS telemetry.

This process has no motor transport and no command publisher.  It subscribes
only to ``/whole_arm/hardware_state`` and atomically creates one evidence file
after a continuous, same-session BRAKE/DISABLED window.  Any active mode,
non-zero GO torque command, communication break, merror, unsafe temperature,
motion, J2 synchronization warning, replay, or telemetry gap rejects the run.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import stat
import statistics
import tempfile
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Mapping, Sequence


POWER_ON_SCHEMA = "go-m8010-v15-31b-power-on-readonly/1.0"
HARDWARE_STATE_SCHEMA = "go-m8010-hardware-state/1.1"
PRODUCTION_MODEL_SHA256 = (
    "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
)
GO_GEAR_RATIO = 6.329999923706055
MOTOR_NAMES = ("J1", "J2A", "J2B", "J3", "J4", "J5", "J6")
JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
MOTOR_DIRECTION_SIGN = {
    "J1": +1,
    "J2A": -1,
    "J2B": +1,
    "J3": +1,
    "J4": +1,
    "J5": +1,
    "J6": -1,
}
MOTOR_GEAR_RATIO = {
    name: (1.0 if name == "J6" else GO_GEAR_RATIO) for name in MOTOR_NAMES
}
SAFE_MODES = {
    name: ({"brake", "disabled"} if name == "J6" else {"brake"})
    for name in MOTOR_NAMES
}
READONLY_GATE = (
    "24V_ON=YES;SUPPORT_RELIABLE=YES;MODEL_POSE_ALIGNED=YES;"
    "ARM_STATIONARY=YES;NOT_AT_MECHANICAL_LIMIT=YES;"
    "OPERATOR_STOP_READY=YES"
)
MINIMUM_DURATION_NS = 10_000_000_000
MINIMUM_VALID_FRAMES = 100
MAXIMUM_FRAME_GAP_NS = 250_000_000
MAXIMUM_FEEDBACK_AGE_MS = 250.0
MAXIMUM_STATIONARY_VELOCITY_RAD_S = math.radians(0.25)
MAXIMUM_J2_SYNC_RAD = math.radians(0.25)
THERMAL_STOP_C = 60.0
MAXIMUM_OUTPUT_BYTES = 64 * 1024 * 1024


class ReadOnlyCaptureError(ValueError):
    """Telemetry cannot establish a continuous powered read-only window."""


def finite_number(value: Any, label: str) -> float:
    if type(value) not in {int, float} or not math.isfinite(float(value)):
        raise ReadOnlyCaptureError(f"{label} must be finite")
    return float(value)


def exact_mapping(value: Any, names: Sequence[str], label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping) or set(value) != set(names):
        raise ReadOnlyCaptureError(f"{label} must contain {list(names)} exactly")
    return value


def finite_vector(value: Any, length: int, label: str) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise ReadOnlyCaptureError(f"{label} must contain exactly {length} values")
    return [finite_number(item, f"{label}[{index}]") for index, item in enumerate(value)]


def numeric_summary(values: Sequence[float]) -> dict[str, float]:
    if not values:
        raise ReadOnlyCaptureError("cannot summarize an empty telemetry series")
    checked = [finite_number(value, "telemetry value") for value in values]
    return {
        "minimum": min(checked),
        "maximum": max(checked),
        "mean": math.fsum(checked) / len(checked),
        "median": statistics.median(checked),
    }


def unwrap_series(values: Sequence[float]) -> list[float]:
    if not values:
        raise ReadOnlyCaptureError("cannot unwrap an empty position series")
    result = [finite_number(values[0], "raw position")]
    previous_raw = result[0]
    for item in values[1:]:
        raw = finite_number(item, "raw position")
        result.append(result[-1] + math.remainder(raw - previous_raw, 2.0 * math.pi))
        previous_raw = raw
    return result


def encoder_branch(name: str, raw_reference: float) -> int:
    if name == "J6":
        return 0
    normalized = (raw_reference + math.pi) % (2.0 * math.pi) - math.pi
    return round((raw_reference - normalized) / (2.0 * math.pi))


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


class PowerOnReadOnlyRecorder:
    """Fail-closed accumulator for hardware-state JSON documents."""

    def __init__(
        self,
        *,
        model_absolute_joint_rad: Sequence[float],
        operator_confirmed_at_utc: str,
        minimum_duration_ns: int = MINIMUM_DURATION_NS,
        minimum_valid_frames: int = MINIMUM_VALID_FRAMES,
    ) -> None:
        self.model_absolute_joint_rad = finite_vector(
            list(model_absolute_joint_rad), 6, "model_absolute_joint_rad"
        )
        self.operator_confirmed_at_utc = operator_confirmed_at_utc
        self.minimum_duration_ns = int(minimum_duration_ns)
        self.minimum_valid_frames = int(minimum_valid_frames)
        if self.minimum_duration_ns < MINIMUM_DURATION_NS:
            raise ReadOnlyCaptureError("capture duration cannot be shorter than 10 seconds")
        if self.minimum_valid_frames < 2:
            raise ReadOnlyCaptureError("minimum valid frame count must be at least two")
        self.samples: list[dict[str, Any]] = []
        self.session_id: str | None = None
        self.state_instance_id: str | None = None
        self.previous_sequence = 0
        self.previous_source_ns = 0
        self.raw_values = {name: [] for name in MOTOR_NAMES}
        self.logical_values = {name: [] for name in MOTOR_NAMES}
        self.velocity_values = {name: [] for name in MOTOR_NAMES}
        self.temperature_values = {name: [] for name in MOTOR_NAMES}
        self.feedback_age_values = {name: [] for name in MOTOR_NAMES}
        self.merror_values = {name: [] for name in MOTOR_NAMES}
        self.mode_values = {name: [] for name in MOTOR_NAMES}
        self.tau_command_values = {name: [] for name in MOTOR_NAMES}
        self.tau_feedback_values = {name: [] for name in MOTOR_NAMES}
        self.tau_joint_values = {name: [] for name in MOTOR_NAMES}
        self.j2_logical_values: list[float] = []
        self.j2_sync_values: list[float] = []

    @property
    def duration_ns(self) -> int:
        if len(self.samples) < 2:
            return 0
        return int(self.samples[-1]["source_monotonic_ns"]) - int(
            self.samples[0]["source_monotonic_ns"]
        )

    @property
    def complete(self) -> bool:
        return (
            self.duration_ns >= self.minimum_duration_ns
            and len(self.samples) >= self.minimum_valid_frames
        )

    def add_document(self, document: Mapping[str, Any]) -> None:
        index = len(self.samples)
        label = f"hardware_state[{index}]"
        if document.get("schema") != HARDWARE_STATE_SCHEMA:
            raise ReadOnlyCaptureError(f"{label} schema mismatch")
        session = document.get("session_id")
        instance = document.get("state_instance_id")
        if not isinstance(session, str) or not session:
            raise ReadOnlyCaptureError(f"{label} session_id missing")
        if (
            not isinstance(instance, str)
            or len(instance) != 32
            or any(character not in "0123456789abcdef" for character in instance)
        ):
            raise ReadOnlyCaptureError(f"{label} state_instance_id invalid")
        if self.session_id is None:
            self.session_id, self.state_instance_id = session, instance
        if session != self.session_id or instance != self.state_instance_id:
            raise ReadOnlyCaptureError("hardware state session/state instance changed")

        sequence = document.get("sequence")
        source_ns = document.get("source_monotonic_ns")
        if type(sequence) is not int or sequence <= self.previous_sequence:
            raise ReadOnlyCaptureError(f"{label} sequence is replayed or invalid")
        if type(source_ns) is not int or source_ns <= self.previous_source_ns:
            raise ReadOnlyCaptureError(f"{label} monotonic timestamp is replayed or invalid")
        if self.previous_source_ns and source_ns - self.previous_source_ns > MAXIMUM_FRAME_GAP_NS:
            raise ReadOnlyCaptureError(f"{label} telemetry gap exceeds 250 ms")

        positions = finite_vector(document.get("position_rad"), 6, f"{label}.position_rad")
        velocities = finite_vector(document.get("velocity_rad_s"), 6, f"{label}.velocity_rad_s")
        if document.get("velocity_source") != "POSITION_SPAN_0P40S":
            raise ReadOnlyCaptureError(
                f"{label} mechanical velocity observer is not ready"
            )
        if max(abs(value) for value in velocities) > MAXIMUM_STATIONARY_VELOCITY_RAD_S:
            raise ReadOnlyCaptureError(
                f"{label} mechanism exceeds stationary bound"
            )
        per_motor = exact_mapping(document.get("per_motor"), MOTOR_NAMES, f"{label}.per_motor")
        modes = exact_mapping(
            document.get("controller_mode_by_motor"), MOTOR_NAMES,
            f"{label}.controller_mode_by_motor",
        )
        if document.get("j2_sync_fault") is not False:
            raise ReadOnlyCaptureError(f"{label} J2 synchronization fault is active")
        sync = finite_number(document.get("j2_e_sync_rad"), f"{label}.j2_e_sync_rad")
        if abs(sync) > MAXIMUM_J2_SYNC_RAD:
            raise ReadOnlyCaptureError(f"{label} J2 synchronization warning threshold exceeded")

        stripped_motors: dict[str, dict[str, Any]] = {}
        for motor_index, name in enumerate(MOTOR_NAMES):
            record = per_motor[name]
            if not isinstance(record, Mapping):
                raise ReadOnlyCaptureError(f"{label}.{name} must be an object")
            mode = str(modes[name]).strip().lower()
            if mode not in SAFE_MODES[name]:
                raise ReadOnlyCaptureError(f"{label}.{name} active/unknown mode observed: {mode}")
            if record.get("communication_ok") is not True or record.get("fresh") is not True:
                raise ReadOnlyCaptureError(f"{label}.{name} communication is not fresh")
            if type(record.get("merror")) is not int or record.get("merror") != 0:
                raise ReadOnlyCaptureError(f"{label}.{name} merror is nonzero")
            age_ms = finite_number(record.get("age_ms"), f"{label}.{name}.age_ms")
            if not 0.0 <= age_ms <= MAXIMUM_FEEDBACK_AGE_MS:
                raise ReadOnlyCaptureError(f"{label}.{name} feedback age exceeds 250 ms")
            raw = finite_number(record.get("raw_position_rad"), f"{label}.{name}.raw_position_rad")
            logical = finite_number(record.get("q_joint_rad"), f"{label}.{name}.q_joint_rad")
            velocity = finite_number(record.get("dq_joint_rad_s"), f"{label}.{name}.dq_joint_rad_s")
            temperature = finite_number(record.get("temperature_c"), f"{label}.{name}.temperature_c")
            if not 0.0 <= temperature < THERMAL_STOP_C:
                raise ReadOnlyCaptureError(f"{label}.{name} temperature is outside read-only bounds")

            tau_command = record.get("tau_cmd_rotor_nm")
            tau_feedback = record.get("tau_feedback_rotor_nm")
            tau_joint = record.get("tau_joint_estimated_nm")
            if name == "J6":
                if any(value is not None for value in (tau_command, tau_feedback, tau_joint)):
                    raise ReadOnlyCaptureError(f"{label}.J6 manufactured a POS_VEL torque value")
            else:
                tau_command = finite_number(tau_command, f"{label}.{name}.tau_cmd_rotor_nm")
                tau_feedback = finite_number(tau_feedback, f"{label}.{name}.tau_feedback_rotor_nm")
                tau_joint = finite_number(tau_joint, f"{label}.{name}.tau_joint_estimated_nm")
                if abs(tau_command) > 1.0e-12:
                    raise ReadOnlyCaptureError(f"{label}.{name} nonzero active torque command")

            self.raw_values[name].append(raw)
            self.logical_values[name].append(logical)
            self.velocity_values[name].append(velocity)
            self.temperature_values[name].append(temperature)
            self.feedback_age_values[name].append(age_ms)
            self.merror_values[name].append(int(record["merror"]))
            self.mode_values[name].append(mode)
            if name != "J6":
                self.tau_command_values[name].append(float(tau_command))
                self.tau_feedback_values[name].append(float(tau_feedback))
                self.tau_joint_values[name].append(float(tau_joint))
            stripped_motors[name] = {
                "raw_position_rad": raw,
                "q_joint_rad": logical,
                "dq_joint_rad_s": velocity,
                "age_ms": age_ms,
                "temperature_c": temperature,
                "merror": int(record["merror"]),
                "communication_ok": True,
                "fresh": True,
                "tau_cmd_rotor_nm": (
                    None if name == "J6" else float(tau_command)
                ),
                "tau_feedback_rotor_nm": (
                    None if name == "J6" else float(tau_feedback)
                ),
                "tau_joint_estimated_nm": (
                    None if name == "J6" else float(tau_joint)
                ),
            }

        j2_logical = 0.5 * (
            self.logical_values["J2A"][-1] + self.logical_values["J2B"][-1]
        )
        self.j2_logical_values.append(j2_logical)
        self.j2_sync_values.append(sync)
        self.samples.append({
            "schema": HARDWARE_STATE_SCHEMA,
            "sequence": sequence,
            "source_monotonic_ns": source_ns,
            "session_id": session,
            "state_instance_id": instance,
            "position_rad": positions,
            "velocity_rad_s": velocities,
            "velocity_source": "POSITION_SPAN_0P40S",
            "j2_e_sync_rad": sync,
            "j2_sync_fault": False,
            "controller_mode_by_motor": {name: str(modes[name]).strip().lower() for name in MOTOR_NAMES},
            "per_motor": stripped_motors,
        })
        self.previous_sequence = sequence
        self.previous_source_ns = source_ns

    def build_document(self) -> dict[str, Any]:
        if not self.complete or self.session_id is None or self.state_instance_id is None:
            raise ReadOnlyCaptureError("continuous read-only qualification is incomplete")
        raw_reference: dict[str, float] = {}
        branches: dict[str, int] = {}
        logical_reference = []
        per_motor: dict[str, dict[str, Any]] = {}
        for name in MOTOR_NAMES:
            raw_series = (
                list(self.raw_values[name])
                if name == "J6"
                else unwrap_series(self.raw_values[name])
            )
            reference = statistics.median(raw_series)
            raw_reference[name] = reference
            branches[name] = encoder_branch(name, reference)
            summary: dict[str, Any] = {
                "online": True,
                "max_feedback_age_ms": max(self.feedback_age_values[name]),
                "raw_position_rad": numeric_summary(raw_series),
                "logical_position_rad": numeric_summary(self.logical_values[name]),
                "max_abs_velocity_rad_s": max(abs(value) for value in self.velocity_values[name]),
                "max_temperature_c": max(self.temperature_values[name]),
                "abnormal_velocity_detected": False,
                "max_abs_merror": max(abs(value) for value in self.merror_values[name]),
                "communication_interruptions": 0,
                "observed_modes": sorted(set(self.mode_values[name])),
            }
            if name == "J6":
                summary.update({
                    "tau_cmd_rotor_nm": None,
                    "tau_feedback_rotor_nm": None,
                    "tau_joint_estimated_nm": None,
                })
            else:
                summary.update({
                    "tau_feedback_rotor_nm": numeric_summary(self.tau_feedback_values[name]),
                    "tau_joint_estimated_nm": numeric_summary(self.tau_joint_values[name]),
                    "max_abs_tau_cmd_rotor_nm": max(abs(value) for value in self.tau_command_values[name]),
                })
            per_motor[name] = summary
        for name in JOINT_NAMES:
            if name == "J2":
                logical_reference.append(statistics.median(self.j2_logical_values))
            else:
                logical_reference.append(statistics.median(self.logical_values[name]))

        return {
            "schema": POWER_ON_SCHEMA,
            "schema_version": "V15.31B-power-on-readonly-v1",
            "status": "PASS",
            "recorded_at_utc": utc_now(),
            "duration_s": self.duration_ns * 1.0e-9,
            "valid_frame_count": len(self.samples),
            "model_sha256": PRODUCTION_MODEL_SHA256,
            "session_id": self.session_id,
            "state_instance_id": self.state_instance_id,
            "motor_direction_sign": dict(MOTOR_DIRECTION_SIGN),
            "motor_gear_ratio": dict(MOTOR_GEAR_RATIO),
            "motor_raw_reference_rad": raw_reference,
            "motor_encoder_branch": branches,
            "logical_joint_reference_rad": logical_reference,
            "model_absolute_joint_rad": list(self.model_absolute_joint_rad),
            "per_motor": per_motor,
            "j2": {
                "logical_position_rad": numeric_summary(self.j2_logical_values),
                "max_abs_e_sync_deg": math.degrees(max(abs(value) for value in self.j2_sync_values)),
                "j2a_logical_torque_contribution_nm": numeric_summary(self.tau_joint_values["J2A"]),
                "j2b_logical_torque_contribution_nm": numeric_summary(self.tau_joint_values["J2B"]),
            },
            "samples": list(self.samples),
            "operator_confirmation": {
                "confirmed_at_utc": self.operator_confirmed_at_utc,
                "mechanism_stationary": True,
                "model_pose_aligned": True,
                "support_reliable": True,
                "not_at_mechanical_limit": True,
                "operator_stop_ready": True,
            },
            "writes": {
                "motor_internal_zero_modified": False,
                "rid_written": False,
                "flash_or_eeprom_written": False,
            },
            "safety": {
                "position_enabled": False,
                "motor_internal_zero_modified": False,
                "flash_written": False,
                "eeprom_written": False,
                "active_command_count": 0,
            },
        }


def json_bytes(document: Mapping[str, Any]) -> bytes:
    data = (json.dumps(document, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
    if len(data) > MAXIMUM_OUTPUT_BYTES:
        raise ReadOnlyCaptureError("power-on read-only evidence exceeds 64 MiB")
    return data


def atomic_write_new(path: Path, data: bytes) -> None:
    output = Path(os.path.abspath(path))
    if output.exists() or output.is_symlink():
        raise ReadOnlyCaptureError(f"refuse to overwrite evidence: {output}")
    output.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    if output.parent.is_symlink() or not output.parent.is_dir():
        raise ReadOnlyCaptureError("evidence output directory is unsafe")
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{output.name}.", suffix=".tmp", dir=output.parent
    )
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            if hasattr(os, "fchmod"):
                os.fchmod(stream.fileno(), stat.S_IRUSR | stat.S_IWUSR)
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        try:
            os.link(temporary, output)
        except FileExistsError as exc:
            raise ReadOnlyCaptureError(f"refuse to overwrite evidence: {output}") from exc
    finally:
        temporary.unlink(missing_ok=True)


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-readonly", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--topic", default="/whole_arm/hardware_state")
    parser.add_argument("--model-path", type=Path, required=True)
    parser.add_argument("--model-absolute-q-rad", type=float, nargs=6, required=True)
    parser.add_argument("--operator-confirmed-at-utc", required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--timeout-s", type=float, default=30.0)
    args = parser.parse_args(argv)
    if args.timeout_s < 12.0 or args.timeout_s > 120.0:
        parser.error("timeout must be in [12, 120] seconds")
    return args


def validate_preflight(args: argparse.Namespace) -> None:
    model = args.model_path.resolve(strict=True)
    if not model.is_file() or hashlib.sha256(model.read_bytes()).hexdigest() != PRODUCTION_MODEL_SHA256:
        raise ReadOnlyCaptureError("frozen production model SHA-256 mismatch")
    try:
        confirmed = datetime.fromisoformat(
            args.operator_confirmed_at_utc.replace("Z", "+00:00")
        )
    except ValueError as exc:
        raise ReadOnlyCaptureError("operator confirmation timestamp is invalid") from exc
    if confirmed.tzinfo is None or confirmed.utcoffset() is None:
        raise ReadOnlyCaptureError("operator confirmation timestamp requires timezone")
    if not args.execute_readonly:
        raise ReadOnlyCaptureError("capture requires --execute-readonly")
    if args.confirm != READONLY_GATE:
        raise ReadOnlyCaptureError(f"capture requires --confirm '{READONLY_GATE}'")


def capture_ros(args: argparse.Namespace) -> dict[str, Any]:
    validate_preflight(args)
    try:
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String
    except ImportError as exc:
        raise ReadOnlyCaptureError("ROS 2 Python environment is unavailable") from exc

    recorder = PowerOnReadOnlyRecorder(
        model_absolute_joint_rad=args.model_absolute_q_rad,
        operator_confirmed_at_utc=args.operator_confirmed_at_utc,
    )
    error: list[Exception] = []

    class CaptureNode(Node):
        def __init__(self) -> None:
            super().__init__("v15_31b_power_on_readonly_capture")
            self.subscription = self.create_subscription(String, args.topic, self.on_state, 100)

        def on_state(self, message: Any) -> None:
            if recorder.complete or error:
                return
            try:
                value = json.loads(message.data)
                if not isinstance(value, dict):
                    raise ReadOnlyCaptureError("hardware state root must be an object")
                recorder.add_document(value)
            except Exception as exc:  # callback must transfer failure to main loop
                error.append(exc)

    rclpy.init(args=None)
    node = CaptureNode()
    deadline = time.monotonic() + args.timeout_s
    try:
        while rclpy.ok() and not recorder.complete and not error and time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    if error:
        raise ReadOnlyCaptureError(str(error[0]))
    if not recorder.complete:
        raise ReadOnlyCaptureError("timed out before a continuous 10-second read-only window")
    document = recorder.build_document()
    atomic_write_new(args.output, json_bytes(document))
    return document


def main(argv: Sequence[str] | None = None) -> int:
    try:
        args = parse_args(argv)
        document = capture_ros(args)
    except (ReadOnlyCaptureError, OSError, ValueError) as exc:
        print(json.dumps({
            "schema": POWER_ON_SCHEMA,
            "status": "BLOCKED",
            "reason": str(exc),
            "hardware_command_interface_present": False,
            "position_enabled": False,
            "motor_internal_zero_modified": False,
            "flash_or_eeprom_written": False,
        }, ensure_ascii=False), flush=True)
        return 4
    print(json.dumps({
        "schema": POWER_ON_SCHEMA,
        "status": "PASS",
        "duration_s": document["duration_s"],
        "valid_frame_count": document["valid_frame_count"],
        "session_id": document["session_id"],
        "state_instance_id": document["state_instance_id"],
        "output": str(args.output.resolve()),
    }, ensure_ascii=False), flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

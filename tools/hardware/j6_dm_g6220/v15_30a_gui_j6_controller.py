#!/usr/bin/env python3
"""J6 独立 POS_VEL GUI worker：本机UDP命令、100 Hz、健康租约安全保持。"""

from __future__ import annotations

import argparse
import ctypes
import fcntl
import hashlib
import json
import math
import os
import re
import secrets
import signal
import socket
import stat
import statistics
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, NamedTuple

from dm_g6220_posvel_transport import DmG6220PosVelTransport
from j6_raw_can_diagnostic import (
    RawCanLogger,
    exact_usb_identity,
    read_parameter,
    refresh_request,
    strict_decode,
)
from v15_30a_profile import (
    quintic_posvel_speed_limit,
    quintic_reference_at,
    thermal_derated_posvel_limits,
    thermal_derating_factor,
    update_posvel_speed_limit,
)


GATE = "V15_30A_GUI_J6_CONTROL_AUTHORIZED=YES"
MOTOR_ID = 1
CTRL_MODE_POS_VEL = 2
PERIOD = 0.01
LEASE_S = 0.5
EMPIRICAL_INITIAL_HOLD_ENTRY_TARGET_LIMIT = math.radians(0.25)
FEEDBACK_MAX_AGE_S = 0.15
REJECTION_LOG_INTERVAL_S = 5.0
COMMAND_PACKET_BUDGET = 128
COMMAND_SOURCE_MAX_AGE_NS = 250_000_000
COMMAND_SOURCE_TAKEOVER_LEASE_NS = 500_000_000
COMMAND_SOURCE_REPLAY_LIMIT = 32
ACTIVE_DEADLINE_CONSECUTIVE_LIMIT = 3
ACTIVE_DEADLINE_CYCLE_LIMIT_S = 0.02
HOLD_ENTRY_TARGET_LIMIT = math.radians(5.0)
MODEL_COMMAND_LOWER = math.radians(-180.0)
MODEL_COMMAND_UPPER = math.radians(180.0)
# Command endpoints remain the complete model range.  Feedback hard-fault
# detection gets a separate half-degree tolerance for encoder quantization and
# endpoint trajectory noise; that tolerance never widens an accepted target.
FEEDBACK_ENVELOPE_TOLERANCE = math.radians(0.5)
FEEDBACK_HARD_LOWER = MODEL_COMMAND_LOWER - FEEDBACK_ENVELOPE_TOLERANCE
FEEDBACK_HARD_UPPER = MODEL_COMMAND_UPPER + FEEDBACK_ENVELOPE_TOLERANCE
VMAX_LIMIT = math.radians(5.0)
AMAX_LIMIT = math.radians(20.0)
RESTORE_VELOCITY_LIMIT = math.radians(1.0)
QUINTIC_COMMAND_SCHEMA = "go-m8010-quintic-command/1.0"
QUINTIC_PROFILE = "quintic-rest-to-rest-v1"
QUINTIC_MAX_INTERVALS = 1_000_000
QUINTIC_MAX_SAMPLE_PERIOD_NS = 10_000_000
# The 16-bit G6220 position feedback is about 0.02186 deg/LSB.  Keep the
# tolerance above three LSBs, but below the commissioned 0.20 deg small step so
# a stationary motor cannot be mistaken for an arrived one.
ARRIVAL_TOLERANCE = math.radians(0.08)
TARGET_TIMEOUT_S = 90.0
NO_PROGRESS_MINIMUM_IMPROVEMENT = math.radians(0.05)
NO_PROGRESS_MINIMUM_QUALIFYING_FRAMES = 3
NO_PROGRESS_WATCHDOG_AUTHORITY = "J6_TARGET_TIMEOUT_POSITION_ERROR_V1"
THERMAL_CONFIG_SHA256 = (
    "1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467"
)
THERMAL_MINIMUM_ACTIVE_FACTOR = 0.1
ACTIVE_THERMAL_LIMITS: dict[str, float] = {}
DEFAULT_THERMAL_CONFIG = (
    Path(__file__).resolve().parents[3]
    / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环"
    / "ros2_ws"
    / "src"
    / "go_m8010_arm_hardware"
    / "config"
    / "thermal_limits.yaml"
)
FAULT_STATES = {8, 9, 0xA, 0xB, 0xC, 0xD, 0xE}
STOP = False


class ExpectedGravityAuthorityBinding(NamedTuple):
    authority_class: str
    empirical_envelope_id: str
    empirical_envelope_sha256: str
    anchor_sha256: str
    session_id: str
    state_instance_id: str


class ExpectedFeedbackIdentityBinding(NamedTuple):
    session_id: str
    state_instance_id: str


class GravityAuthorityStartupBindingError(ValueError):
    """An active packet conflicts with the immutable launcher authority."""


# Active run() replaces this immutable fail-closed sentinel before opening any
# persistent state or hardware.  A command validator can therefore never gain
# gravity authority merely because launcher binding arguments were omitted.
EXPECTED_GRAVITY_AUTHORITY_BINDING = ExpectedGravityAuthorityBinding(
    authority_class="NONE",
    empirical_envelope_id="",
    empirical_envelope_sha256="",
    anchor_sha256="",
    session_id="",
    state_instance_id="",
)
EXPECTED_FEEDBACK_IDENTITY_BINDING = ExpectedFeedbackIdentityBinding(
    session_id="",
    state_instance_id="",
)
J6_FEEDBACK_SOURCE_INSTANCE_ID = secrets.token_hex(16)
J6_FEEDBACK_SEQUENCE = 0
FEEDBACK_HANDOFF_SCHEMA = "go-m8010-j6-feedback-handoff/1.0"
RAW_CAPTURE_SCHEMA = "go-m8010-j6-disabled-raw-capture-statistics/1.0"
MAXIMUM_FEEDBACK_SEQUENCE = (1 << 63) - 1
MAXIMUM_HANDOFF_BYTES = 64 * 1024
MAXIMUM_RAW_CAPTURE_BYTES = 16 * 1024 * 1024


def stop_handler(_signum, _frame) -> None:
    global STOP
    STOP = True


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--command-port", type=int, default=15311)
    parser.add_argument("--feedback-port", type=int, default=15300)
    parser.add_argument("--zero-file", type=Path)
    parser.add_argument(
        "--thermal-config", type=Path, default=DEFAULT_THERMAL_CONFIG
    )
    parser.add_argument("--expected-gravity-authority-class", default="")
    parser.add_argument("--expected-empirical-envelope-id", default="")
    parser.add_argument("--expected-empirical-envelope-sha256", default="")
    parser.add_argument("--expected-gravity-anchor-sha256", default="")
    parser.add_argument("--expected-gravity-session-id", default="")
    parser.add_argument("--expected-gravity-state-instance-id", default="")
    parser.add_argument("--feedback-session-id", default="")
    parser.add_argument("--feedback-state-instance-id", default="")
    parser.add_argument("--feedback-handoff-file", type=Path)
    return parser.parse_args()


def load_thermal_limits(path: Path) -> dict:
    """Load the shared V15.31A thresholds before any hardware is opened."""

    raw = path.read_bytes()
    actual_sha256 = hashlib.sha256(raw).hexdigest()
    if actual_sha256 != THERMAL_CONFIG_SHA256:
        raise RuntimeError("J6热管理配置SHA256不匹配")
    try:
        text = raw.decode("utf-8")
    except UnicodeDecodeError as exc:
        raise RuntimeError("J6热管理配置不是有效UTF-8") from exc
    scalars: dict[str, str] = {}
    for original_line in text.splitlines():
        line = original_line.split("#", 1)[0].strip()
        if not line:
            continue
        key, separator, encoded = line.partition(":")
        key = key.strip()
        encoded = encoded.strip()
        if separator != ":" or not key or not encoded or key in scalars:
            raise RuntimeError("J6热管理配置格式不匹配")
        scalars[key] = encoded
    required = {
        "normal_below_c",
        "warning_below_c",
        "derating_start_c",
        "thermal_stop_c",
        "rearm_below_c",
        "cooldown_seconds",
        "slope_window_seconds",
    }
    if (
        scalars.get("schema") != "go-m8010-thermal-limits/1.0"
        or not required.issubset(scalars)
    ):
        raise RuntimeError("J6热管理配置格式不匹配")
    limits = {}
    for name in required:
        try:
            value = float(scalars[name])
        except ValueError as exc:
            raise RuntimeError(f"J6热管理配置{name}无效") from exc
        if not math.isfinite(value):
            raise RuntimeError(f"J6热管理配置{name}无效")
        limits[name] = value
    if not (
        limits["normal_below_c"]
        < limits["warning_below_c"]
        < limits["derating_start_c"]
        < limits["thermal_stop_c"]
        and limits["normal_below_c"]
        < limits["rearm_below_c"]
        < limits["thermal_stop_c"]
        and limits["cooldown_seconds"] > 0.0
        and limits["slope_window_seconds"] > 0.0
    ):
        raise RuntimeError("J6热管理配置阈值顺序无效")
    limits["threshold_authority"] = scalars.get("threshold_authority", "")
    limits["config_sha256"] = actual_sha256
    return limits


def load_persistent_zero(path: Path | None) -> float | None:
    if path is None:
        return None
    document = json.loads(path.read_text(encoding="utf-8"))
    if (
        document.get("schema") != "go-m8010-persistent-software-zero/1.0"
        or document.get("reference_name") != "PERSISTENT_SOFTWARE_ZERO_V1"
    ):
        raise RuntimeError("J6持久软件零位文件格式不匹配")
    writes = document.get("writes", {})
    if any(
        writes.get(name) is not False
        for name in (
            "motor_internal_zero_modified",
            "rid_written",
            "flash_or_eeprom_written",
        )
    ):
        raise RuntimeError("J6持久软件零位记录了禁止写入")
    reference = float(document["motors"]["J6"]["raw_position_rad"])
    if not math.isfinite(reference):
        raise RuntimeError("J6持久软件零位不是有限数")
    return reference


def valid_lower_sha256(value: object) -> bool:
    return bool(
        type(value) is str
        and len(value) == 64
        and all(character in "0123456789abcdef" for character in value)
    )


def _load_json_without_duplicate_keys(raw: bytes, label: str) -> dict[str, Any]:
    if not raw:
        raise RuntimeError(f"{label}为空")

    def reject_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
        result: dict[str, Any] = {}
        for key, value in pairs:
            if key in result:
                raise RuntimeError(f"{label}包含重复字段")
            result[key] = value
        return result

    try:
        document = json.loads(
            raw.decode("utf-8"), object_pairs_hook=reject_duplicates
        )
    except (UnicodeError, json.JSONDecodeError) as exc:
        raise RuntimeError(f"{label}不是有效UTF-8 JSON") from exc
    if type(document) is not dict:
        raise RuntimeError(f"{label}根节点无效")
    return document


def _secure_regular_file_bytes(
    path: Path, maximum_bytes: int, label: str,
) -> bytes:
    """Read one Linux-owned immutable handoff input without following links."""

    absolute = Path(os.path.abspath(path))
    try:
        metadata = os.lstat(absolute)
    except OSError as exc:
        raise RuntimeError(f"{label}不可用") from exc
    owner = getattr(os, "geteuid", lambda: metadata.st_uid)()
    if (
        not stat.S_ISREG(metadata.st_mode)
        or metadata.st_uid != owner
        or metadata.st_nlink != 1
        or stat.S_IMODE(metadata.st_mode) != 0o600
        or metadata.st_size <= 0
        or metadata.st_size > maximum_bytes
    ):
        raise RuntimeError(f"{label}文件安全属性无效")
    flags = os.O_RDONLY | getattr(os, "O_CLOEXEC", 0)
    if hasattr(os, "O_NOFOLLOW"):
        flags |= os.O_NOFOLLOW
    descriptor = os.open(absolute, flags)
    try:
        opened = os.fstat(descriptor)
        if (
            opened.st_dev != metadata.st_dev
            or opened.st_ino != metadata.st_ino
            or opened.st_size != metadata.st_size
        ):
            raise RuntimeError(f"{label}读取期间被替换")
        raw = os.read(descriptor, maximum_bytes + 1)
        if len(raw) != metadata.st_size:
            raise RuntimeError(f"{label}读取长度不匹配")
        return raw
    finally:
        os.close(descriptor)


def _fsync_directory(path: Path) -> None:
    descriptor = os.open(
        path, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0)
    )
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)


def claim_feedback_handoff(path: Path) -> Path:
    """Irreversibly claim a single-use handoff before interpreting it."""

    pending = Path(os.path.abspath(path))
    # The metadata read deliberately happens before the claim.  No JSON field
    # is trusted until the pending name has been consumed.
    try:
        before = os.lstat(pending)
    except OSError as exc:
        raise RuntimeError("J6反馈接管文件不可用") from exc
    _secure_regular_file_bytes(pending, MAXIMUM_HANDOFF_BYTES, "J6反馈接管文件")
    try:
        after_read = os.lstat(pending)
    except OSError as exc:
        raise RuntimeError("J6反馈接管文件读取后消失") from exc
    if (
        before.st_dev != after_read.st_dev
        or before.st_ino != after_read.st_ino
        or before.st_size != after_read.st_size
    ):
        raise RuntimeError("J6反馈接管文件读取期间被替换")
    claimed = pending.with_name(f"{pending.name}.claimed")
    if claimed.exists() or claimed.is_symlink():
        raise RuntimeError("J6反馈接管文件已使用")
    try:
        os.link(pending, claimed, follow_symlinks=False)
    except OSError as exc:
        raise RuntimeError("J6反馈接管文件无法原子认领") from exc
    try:
        linked = os.lstat(claimed)
        current = os.lstat(pending)
        if (
            linked.st_dev != after_read.st_dev
            or linked.st_ino != after_read.st_ino
            or current.st_dev != after_read.st_dev
            or current.st_ino != after_read.st_ino
        ):
            raise RuntimeError("J6反馈接管文件认领期间被替换")
        pending.unlink()
        _fsync_directory(pending.parent)
    except BaseException as exc:
        # Never remove the claimed name here.  Even an interrupted or invalid
        # attempt remains visibly spent and cannot be retried against hardware.
        raise RuntimeError("J6反馈接管文件认领未完整落盘") from exc
    return claimed


def consume_feedback_handoff(
    path: Path | None,
    binding: ExpectedFeedbackIdentityBinding,
    feedback_port: int,
    now_monotonic_ns: int | None = None,
) -> tuple[str, int]:
    """Claim and validate the exact raw-DISABLED feedback continuation."""

    if path is None:
        raise RuntimeError("J6主动worker缺少单次反馈接管文件")
    if not binding.session_id or not binding.state_instance_id:
        raise RuntimeError("J6反馈接管要求完整会话身份")
    if type(feedback_port) is not int or not 1 <= feedback_port <= 65535:
        raise RuntimeError("J6反馈端口无效")
    claimed = claim_feedback_handoff(path)
    raw_handoff = _secure_regular_file_bytes(
        claimed, MAXIMUM_HANDOFF_BYTES, "已认领J6反馈接管文件"
    )
    handoff = _load_json_without_duplicate_keys(raw_handoff, "J6反馈接管文件")
    expected_root = {
        "schema", "handoff_id", "session_id", "state_instance_id",
        "source_instance_id", "last_sequence", "last_source_monotonic_ns",
        "raw_capture", "terminal", "forwarding",
        "active_control_authorized", "can_tx_policy",
        "single_use_claim_required",
    }
    if set(handoff) != expected_root:
        raise RuntimeError("J6反馈接管字段集合无效")
    source = handoff.get("source_instance_id")
    sequence = handoff.get("last_sequence")
    source_ns = handoff.get("last_source_monotonic_ns")
    if (
        handoff.get("schema") != FEEDBACK_HANDOFF_SCHEMA
        or handoff.get("session_id") != binding.session_id
        or handoff.get("state_instance_id") != binding.state_instance_id
        or type(source) is not str
        or re.fullmatch(r"[0-9a-f]{32}", source) is None
        or type(sequence) is not int
        or not 0 < sequence < MAXIMUM_FEEDBACK_SEQUENCE
        or type(source_ns) is not int
        or source_ns <= 0
        or handoff.get("handoff_id") != f"j6-feedback-{source}-{sequence}"
        or handoff.get("active_control_authorized") is not False
        or handoff.get("can_tx_policy") != "DISABLED_CLASSIC_CAN_REFRESH_ONLY"
        or handoff.get("single_use_claim_required") is not True
    ):
        raise RuntimeError("J6反馈接管身份或序列无效")
    current_ns = time.monotonic_ns() if now_monotonic_ns is None else now_monotonic_ns
    if type(current_ns) is not int or current_ns <= source_ns:
        raise RuntimeError("J6反馈接管单调时钟连续性无效")

    terminal = handoff.get("terminal")
    forwarding = handoff.get("forwarding")
    raw_reference = handoff.get("raw_capture")
    if (
        type(terminal) is not dict
        or set(terminal) != {
            "drive_state", "controller_mode", "confirmed", "channel_closed"
        }
        or terminal.get("drive_state") != 0
        or terminal.get("controller_mode") != "brake"
        or terminal.get("confirmed") is not True
        or terminal.get("channel_closed") is not True
        or type(forwarding) is not dict
        or set(forwarding) != {
            "destination", "published_count", "socket_closed"
        }
        or forwarding.get("destination") != f"127.0.0.1:{feedback_port}"
        or forwarding.get("published_count") != sequence
        or forwarding.get("socket_closed") is not True
        or type(raw_reference) is not dict
        or set(raw_reference) != {"path", "sha256"}
        or not valid_lower_sha256(raw_reference.get("sha256"))
    ):
        raise RuntimeError("J6反馈接管终态或转发绑定无效")

    raw_path_text = raw_reference.get("path")
    if type(raw_path_text) is not str:
        raise RuntimeError("J6反馈接管原始证据路径无效")
    raw_path = Path(os.path.abspath(raw_path_text))
    if raw_path_text != str(raw_path):
        raise RuntimeError("J6反馈接管原始证据必须使用绝对路径")
    raw_capture_bytes = _secure_regular_file_bytes(
        raw_path, MAXIMUM_RAW_CAPTURE_BYTES, "J6原始DISABLED证据"
    )
    if hashlib.sha256(raw_capture_bytes).hexdigest() != raw_reference["sha256"]:
        raise RuntimeError("J6原始DISABLED证据SHA256不匹配")
    raw_capture = _load_json_without_duplicate_keys(
        raw_capture_bytes, "J6原始DISABLED证据"
    )
    raw_terminal = raw_capture.get("terminal")
    raw_forwarding = raw_capture.get("readonly_feedback_udp")
    raw_safety = raw_capture.get("safety")
    if (
        raw_capture.get("schema") != RAW_CAPTURE_SCHEMA
        or raw_capture.get("status") != "PASS"
        or raw_capture.get("physical_power_off_required") is not False
        or type(raw_terminal) is not dict
        or raw_terminal.get("confirmed") is not True
        or raw_terminal.get("channel_closed") is not True
        or type(raw_forwarding) is not dict
        or raw_forwarding.get("session_id") != binding.session_id
        or raw_forwarding.get("state_instance_id") != binding.state_instance_id
        or raw_forwarding.get("source_instance_id") != source
        or raw_forwarding.get("published_count") != sequence
        or raw_forwarding.get("last_sequence") != sequence
        or raw_forwarding.get("last_source_monotonic_ns") != source_ns
        or raw_forwarding.get("socket_closed") is not True
        or raw_forwarding.get("active_control_authorized") is not False
        or raw_forwarding.get("can_tx_policy")
        != "DISABLED_CLASSIC_CAN_REFRESH_ONLY"
        or type(raw_safety) is not dict
        or raw_safety.get("active_or_hold_commands_sent") != 0
        or raw_safety.get("active_commands_sent") != 0
        or raw_safety.get("hold_commands_sent") != 0
        or raw_safety.get("forbidden_tx_attempt_count") != 0
    ):
        raise RuntimeError("J6原始DISABLED证据不满足接管条件")
    required_final = raw_terminal.get("required_final_disabled_frames")
    packet_count = raw_capture.get("packet_count")
    if (
        type(required_final) is not int
        or required_final <= 0
        or type(packet_count) is not int
        or packet_count < 500
        or sequence != packet_count + required_final
    ):
        raise RuntimeError("J6原始DISABLED证据计数不连续")
    return source, sequence


def validate_gravity_authority_startup_binding(
    args: argparse.Namespace,
) -> ExpectedGravityAuthorityBinding:
    """Freeze the launcher's model-session authority before hardware access."""

    authority_class = args.expected_gravity_authority_class
    envelope_id = args.expected_empirical_envelope_id
    envelope_sha256 = args.expected_empirical_envelope_sha256
    anchor_sha256 = args.expected_gravity_anchor_sha256
    session_id = args.expected_gravity_session_id
    state_instance_id = args.expected_gravity_state_instance_id
    allowed = {
        "NONE",
        "OFFICIAL_CONTINUOUS_RATING",
        "EMPIRICAL_VALIDATION_ENVELOPE",
    }
    if authority_class not in allowed:
        raise RuntimeError("J6 gravity startup authority class is missing")
    if authority_class == "NONE":
        if any(
            (
                envelope_id,
                envelope_sha256,
                anchor_sha256,
                session_id,
                state_instance_id,
            )
        ):
            raise RuntimeError("J6 NONE gravity startup binding is invalid")
    else:
        if (
            not valid_lower_sha256(anchor_sha256)
            or re.fullmatch(
                r"persistent:[0-9a-f]{16}:j2session:[0-9a-f]{16}:"
                r"goauxsession:[0-9a-f]{16}",
                session_id,
            )
            is None
            or len(state_instance_id) != 32
            or any(
                character not in "0123456789abcdef"
                for character in state_instance_id
            )
        ):
            raise RuntimeError("J6 gravity startup session binding is invalid")
        if authority_class == "EMPIRICAL_VALIDATION_ENVELOPE":
            if (
                len(envelope_id) != 38
                or not envelope_id.startswith("v15-31b-empirical-")
                or any(
                    character not in "0123456789abcdef"
                    for character in envelope_id[18:]
                )
                or not valid_lower_sha256(envelope_sha256)
            ):
                raise RuntimeError(
                    "J6 empirical gravity startup binding is invalid"
                )
        elif envelope_id or envelope_sha256:
            raise RuntimeError(
                "J6 official gravity startup cannot bind empirical fields"
            )
    return ExpectedGravityAuthorityBinding(
        authority_class=authority_class,
        empirical_envelope_id=envelope_id,
        empirical_envelope_sha256=envelope_sha256,
        anchor_sha256=anchor_sha256,
        session_id=session_id,
        state_instance_id=state_instance_id,
    )


def validate_feedback_startup_binding(
    args: argparse.Namespace,
    authority_binding: ExpectedGravityAuthorityBinding,
) -> ExpectedFeedbackIdentityBinding:
    """Freeze feedback identity without creating active gravity authority."""

    session_id = args.feedback_session_id
    state_instance_id = args.feedback_state_instance_id
    fields_present = (bool(session_id), bool(state_instance_id))
    if fields_present == (True, False) or fields_present == (False, True):
        raise RuntimeError("J6 feedback startup identity is incomplete")
    if fields_present == (True, True) and (
        re.fullmatch(
            r"persistent:[0-9a-f]{16}:j2session:[0-9a-f]{16}:"
            r"goauxsession:[0-9a-f]{16}",
            session_id,
        )
        is None
        or len(state_instance_id) != 32
        or any(
            character not in "0123456789abcdef"
            for character in state_instance_id
        )
    ):
        raise RuntimeError("J6 feedback startup identity is invalid")
    if authority_binding.authority_class != "NONE" and (
        session_id != authority_binding.session_id
        or state_instance_id != authority_binding.state_instance_id
    ):
        raise RuntimeError(
            "J6 active gravity and feedback startup identities disagree"
        )
    return ExpectedFeedbackIdentityBinding(
        session_id=session_id,
        state_instance_id=state_instance_id,
    )


def enforce_gravity_authority_startup_binding(
    authority: dict, authority_class: str
) -> None:
    """Reject every packet that differs from the immutable launch binding."""

    expected = EXPECTED_GRAVITY_AUTHORITY_BINDING
    if expected.authority_class == "NONE":
        raise GravityAuthorityStartupBindingError(
            "J6 gravity authority was not authorized at startup"
        )
    if authority_class != expected.authority_class:
        raise GravityAuthorityStartupBindingError(
            "J6 gravity authority class changed after startup"
        )
    if (
        authority.get("session_id") != expected.session_id
        or authority.get("state_instance_id") != expected.state_instance_id
    ):
        raise GravityAuthorityStartupBindingError(
            "J6 gravity session changed after startup"
        )
    if authority_class == "EMPIRICAL_VALIDATION_ENVELOPE" and (
        authority.get("empirical_envelope_id")
        != expected.empirical_envelope_id
        or authority.get("empirical_envelope_sha256")
        != expected.empirical_envelope_sha256
        or authority.get("anchor_sha256") != expected.anchor_sha256
    ):
        raise GravityAuthorityStartupBindingError(
            "J6 empirical envelope changed after startup"
        )


def parse_command(
    payload: bytes, received_monotonic_ns: int | None = None
) -> dict:
    value = json.loads(payload)
    if not isinstance(value, dict):
        raise ValueError("命令字段类型或数值无效")
    schema = value.get("schema")
    if schema not in {
        "go-m8010-gui-command/1.0",
        "go-m8010-gui-command/1.1",
        "go-m8010-gui-command/1.2",
        "go-m8010-gui-command/1.3",
    }:
        raise ValueError("命令格式不匹配")
    if value.get("mode") not in {"brake", "drag", "hold", "position"}:
        raise ValueError("模式不允许")
    if schema in {
        "go-m8010-gui-command/1.0",
        "go-m8010-gui-command/1.1",
    } and value["mode"] != "brake":
        raise ValueError("旧版协议仅允许制动")
    if (
        schema == "go-m8010-gui-command/1.3"
        and value["mode"] not in {"position", "brake"}
    ):
        raise ValueError("1.3协议仅允许轨迹位置命令或制动")
    if received_monotonic_ns is None:
        received_monotonic_ns = time.monotonic_ns()
    if type(received_monotonic_ns) is not int or received_monotonic_ns <= 0:
        raise ValueError("命令接收时钟无效")
    if value["mode"] != "brake":
        source_instance_id = value.get("source_instance_id")
        if (
            type(source_instance_id) is not str
            or len(source_instance_id) != 32
            or any(character not in "0123456789abcdef" for character in source_instance_id)
        ):
            raise ValueError("命令来源实例无效")
        source_monotonic_ns = value.get("source_monotonic_ns")
        if type(source_monotonic_ns) is not int or source_monotonic_ns <= 0:
            raise ValueError("命令来源时钟无效")
        source_age_ns = received_monotonic_ns - source_monotonic_ns
        if not 0 <= source_age_ns <= COMMAND_SOURCE_MAX_AGE_NS:
            raise ValueError("命令来源已过期或来自未来")
        sequence = value.get("sequence")
        if type(sequence) is not int or not 1 <= sequence <= (1 << 63) - 1:
            raise ValueError("命令来源序列无效")
    raw_targets = value.get("targets_rad")
    if (
        not isinstance(raw_targets, list)
        or len(raw_targets) != 6
        or not all(type(item) in {int, float} for item in raw_targets)
    ):
        raise ValueError("目标必须是六个有限数")
    targets = [float(item) for item in raw_targets]
    if not all(math.isfinite(item) for item in targets):
        raise ValueError("目标必须是六个有限数")
    recovery = value.get("recovery", False)
    if type(recovery) is not bool:
        raise ValueError("恢复标记必须是布尔值")
    # J6 has no signed recovery permit chain.  Keep the schema field for
    # compatibility, but never let this untrusted boolean widen model limits.
    active_joint_mask = value.get("active_joint_mask")
    if schema == "go-m8010-gui-command/1.0":
        active_joint_mask = [False] * 6
        value["active_joint_mask"] = active_joint_mask
    elif (
        not isinstance(active_joint_mask, list)
        or len(active_joint_mask) != 6
        or not all(type(item) is bool for item in active_joint_mask)
    ):
        raise ValueError("关节激活掩码必须是六个布尔值")
    if value["mode"] == "brake" and any(active_joint_mask):
        raise ValueError("制动命令不得携带激活关节")
    if value["mode"] in {"brake", "drag"}:
        moving_joint_mask = [False] * 6
    else:
        supplied_moving_mask = value.get("moving_joint_mask")
        if "moving_joint_mask" not in value:
            moving_joint_mask = (
                list(active_joint_mask)
                if value["mode"] == "position"
                else [False] * 6
            )
        elif (
            not isinstance(supplied_moving_mask, list)
            or len(supplied_moving_mask) != 6
            or not all(type(item) is bool for item in supplied_moving_mask)
        ):
            raise ValueError("关节移动掩码必须是六个布尔值")
        else:
            moving_joint_mask = list(supplied_moving_mask)
        if value["mode"] == "hold":
            moving_joint_mask = [False] * 6
        if any(
            moving and not active
            for moving, active in zip(moving_joint_mask, active_joint_mask)
        ):
            raise ValueError("移动关节必须同时激活")
    value["moving_joint_mask"] = moving_joint_mask
    if value["mode"] in {"brake", "drag"}:
        targets = [0.0] * 6
    elif active_joint_mask[5]:
        j6_target = targets[5]
        if not MODEL_COMMAND_LOWER <= j6_target <= MODEL_COMMAND_UPPER:
            raise ValueError("J6目标超出模型机械限位")
    value["targets_rad"] = targets
    activation_epoch = value.get("activation_epoch", 0)
    if schema == "go-m8010-gui-command/1.0":
        activation_epoch = 0
    elif (
        type(activation_epoch) is not int
        or not 0 <= activation_epoch <= (1 << 63) - 1
    ):
        raise ValueError("激活纪元必须是非负整数")
    if (
        value["mode"] in {"hold", "position"}
        and active_joint_mask[5]
        and activation_epoch == 0
    ):
        raise ValueError("主动命令的激活纪元必须大于零")
    value["activation_epoch"] = activation_epoch
    vmax = value.get("maximum_velocity_rad_s")
    amax = value.get("maximum_acceleration_rad_s2")
    if type(vmax) not in {int, float} or type(amax) not in {int, float}:
        raise ValueError("速度或加速度无效")
    vmax = float(vmax)
    amax = float(amax)
    if not math.isfinite(vmax) or not math.isfinite(amax) or vmax <= 0.0 or amax <= 0.0:
        raise ValueError("速度或加速度无效")
    value["maximum_velocity_rad_s"] = min(vmax, VMAX_LIMIT)
    value["maximum_acceleration_rad_s2"] = min(amax, AMAX_LIMIT)
    if schema == "go-m8010-gui-command/1.3" and value["mode"] == "position":
        plan_token_id = value.get("plan_token_id")
        if not valid_lower_sha256(plan_token_id):
            raise ValueError("POSITION quintic计划令牌无效")
        trajectory = value.get("trajectory")
        required_trajectory_fields = {
            "schema",
            "trajectory_sha256",
            "profile",
            "start_rad",
            "target_rad",
            "duration_ns",
            "interval_count",
            "execute_at_monotonic_ns",
            "segment_index",
            "segment_count",
        }
        if (
            not isinstance(trajectory, dict)
            or set(trajectory) != required_trajectory_fields
        ):
            raise ValueError("POSITION quintic描述符无效")
        if trajectory.get("schema") != QUINTIC_COMMAND_SCHEMA:
            raise ValueError("POSITION quintic描述符格式不匹配")
        trajectory_sha256 = trajectory.get("trajectory_sha256")
        if not valid_lower_sha256(trajectory_sha256):
            raise ValueError("POSITION quintic轨迹哈希无效")
        if trajectory.get("profile") != QUINTIC_PROFILE:
            raise ValueError("POSITION quintic轨迹类型不匹配")
        raw_start = trajectory.get("start_rad")
        raw_target = trajectory.get("target_rad")
        for name, vector in (("起点", raw_start), ("终点", raw_target)):
            if (
                not isinstance(vector, list)
                or len(vector) != 6
                or not all(type(item) in {int, float} for item in vector)
                or not all(math.isfinite(float(item)) for item in vector)
            ):
                raise ValueError(f"POSITION quintic{name}必须是六个有限数")
        start = [float(item) for item in raw_start]
        trajectory_target = [float(item) for item in raw_target]
        if trajectory_target != targets:
            raise ValueError("POSITION quintic终点与命令目标不匹配")
        if not active_joint_mask[5] or moving_joint_mask != [False] * 5 + [True]:
            raise ValueError("POSITION quintic J6必须是唯一移动关节")
        if any(
            index != 5 and start[index] != trajectory_target[index]
            for index in range(6)
        ):
            raise ValueError("POSITION quintic非移动关节起终点不匹配")
        if not (
            MODEL_COMMAND_LOWER <= start[5] <= MODEL_COMMAND_UPPER
            and MODEL_COMMAND_LOWER
            <= trajectory_target[5]
            <= MODEL_COMMAND_UPPER
        ):
            raise ValueError("POSITION quintic J6起终点超出模型机械限位")
        if start[5] == trajectory_target[5]:
            raise ValueError("POSITION quintic移动关节位移必须非零")
        duration_ns = trajectory.get("duration_ns")
        interval_count = trajectory.get("interval_count")
        execute_at_monotonic_ns = trajectory.get("execute_at_monotonic_ns")
        segment_index = trajectory.get("segment_index")
        segment_count = trajectory.get("segment_count")
        if type(duration_ns) is not int or duration_ns <= 0:
            raise ValueError("POSITION quintic时长无效")
        if (
            type(interval_count) is not int
            or not 1 <= interval_count <= QUINTIC_MAX_INTERVALS
            or duration_ns > interval_count * QUINTIC_MAX_SAMPLE_PERIOD_NS
        ):
            raise ValueError("POSITION quintic采样网格无效")
        if (
            type(execute_at_monotonic_ns) is not int
            or execute_at_monotonic_ns <= 0
        ):
            raise ValueError("POSITION quintic执行时钟无效")
        if (
            type(segment_index) is not int
            or type(segment_count) is not int
            or segment_index < 0
            or segment_count <= 0
            or segment_index >= segment_count
        ):
            raise ValueError("POSITION quintic分段索引无效")
        duration_s = duration_ns * 1.0e-9
        displacement = abs(trajectory_target[5] - start[5])
        peak_velocity = (15.0 / 8.0) * displacement / duration_s
        peak_acceleration = (
            10.0 / math.sqrt(3.0)
        ) * displacement / (duration_s * duration_s)
        if (
            peak_velocity > value["maximum_velocity_rad_s"] + 1.0e-12
            or peak_acceleration
            > value["maximum_acceleration_rad_s2"] + 1.0e-12
        ):
            raise ValueError("POSITION quintic轨迹超过速度或加速度上限")
        value["plan_token_id"] = plan_token_id
        value["trajectory"] = {
            "schema": QUINTIC_COMMAND_SCHEMA,
            "trajectory_sha256": trajectory_sha256,
            "profile": QUINTIC_PROFILE,
            "start_rad": start,
            "target_rad": trajectory_target,
            "duration_ns": duration_ns,
            "interval_count": interval_count,
            "execute_at_monotonic_ns": execute_at_monotonic_ns,
            "segment_index": segment_index,
            "segment_count": segment_count,
        }
    value["received_at"] = received_monotonic_ns / 1_000_000_000.0
    return value


def command_is_v13_quintic_position(command: dict | None) -> bool:
    return bool(
        command is not None
        and command.get("schema") == "go-m8010-gui-command/1.3"
        and command.get("mode") == "position"
        and isinstance(command.get("moving_joint_mask"), list)
        and command["moving_joint_mask"][5] is True
        and isinstance(command.get("trajectory"), dict)
    )


def validate_empirical_command_authority(
    command: dict, received_monotonic_ns: int, replay_state: dict
) -> tuple | None:
    """Independently require the Router's short-lived empirical proof."""

    if command.get("mode") in {"brake", "drag"} or not command.get(
        "active_joint_mask", [False] * 6
    )[5]:
        return None
    authority = command.get("gravity_authority")
    common = {
        "schema", "source_instance_id", "sequence", "source_monotonic_ns",
        "model_sha256", "gravity_config_sha256", "session_id",
        "state_instance_id", "gravity_scale", "gravity_scale_target",
        "feedforward_nm",
    }
    if (
        isinstance(authority, dict)
        and authority.get("schema")
        == "go-m8010-gravity-command-authority/1.0"
    ):
        if set(authority) != common:
            raise GravityAuthorityStartupBindingError(
                "J6 official continuous authority is incomplete"
            )
        source_ns = authority.get("source_monotonic_ns")
        feedforward = authority.get("feedforward_nm")
        if (
            authority.get("model_sha256")
            != "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
            or authority.get("gravity_config_sha256")
            != "307469b8384fd35547327ba1d5f80aa440e6b9663406ab7c9d9469bea263335d"
            or type(source_ns) is not int
            or not 0 < source_ns <= received_monotonic_ns
            or received_monotonic_ns - source_ns > COMMAND_SOURCE_MAX_AGE_NS
            or not isinstance(authority.get("session_id"), str)
            or not authority["session_id"]
            or not isinstance(authority.get("state_instance_id"), str)
            or not authority["state_instance_id"]
            or not isinstance(feedforward, list)
            or len(feedforward) != 6
            or any(type(item) not in {int, float} or not math.isfinite(float(item)) for item in feedforward)
            or abs(float(feedforward[5])) > 1.0e-12
        ):
            raise ValueError("J6 official continuous authority is invalid")
        enforce_gravity_authority_startup_binding(
            authority, "OFFICIAL_CONTINUOUS_RATING"
        )
        return (
            "OFFICIAL_CONTINUOUS_RATING",
            authority["session_id"],
            authority["state_instance_id"],
            "", "", 0,
        )
    required = {
        "schema", "source_instance_id", "sequence", "source_monotonic_ns",
        "model_sha256", "gravity_config_sha256", "session_id",
        "state_instance_id", "gravity_scale", "gravity_scale_target",
        "feedforward_nm", "authority_class", "rating_classification",
        "empirical_envelope_id", "empirical_envelope_sha256",
        "empirical_envelope_expires_at_utc",
        "empirical_envelope_deadline_monotonic_ns", "anchor_sha256",
        "empirical_stage_index", "empirical_position_validation_authorized",
        "empirical_maximum_position_segment_seconds",
        "empirical_maximum_abs_position_segment_deg",
    }
    if not isinstance(authority, dict) or set(authority) != required:
        raise GravityAuthorityStartupBindingError(
            "J6 empirical authority is missing or incomplete"
        )
    if (
        authority.get("schema")
        != "go-m8010-gravity-command-authority/1.1"
        or authority.get("authority_class")
        != "EMPIRICAL_VALIDATION_ENVELOPE"
        or authority.get("rating_classification")
        != "NOT_OFFICIAL_CONTINUOUS_RATING"
        or authority.get("model_sha256")
        != "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
        or authority.get("gravity_config_sha256")
        != "307469b8384fd35547327ba1d5f80aa440e6b9663406ab7c9d9469bea263335d"
    ):
        raise ValueError("J6 empirical authority identity is invalid")
    source_ns = authority.get("source_monotonic_ns")
    sequence = authority.get("sequence")
    if (
        type(source_ns) is not int
        or not 0 < source_ns <= received_monotonic_ns
        or received_monotonic_ns - source_ns > COMMAND_SOURCE_MAX_AGE_NS
        or type(sequence) is not int
        or sequence <= 0
    ):
        raise ValueError("J6 empirical authority is stale")
    envelope_id = authority.get("empirical_envelope_id")
    if (
        not isinstance(envelope_id, str)
        or len(envelope_id) != 38
        or not envelope_id.startswith("v15-31b-empirical-")
        or any(character not in "0123456789abcdef" for character in envelope_id[18:])
        or not valid_lower_sha256(authority.get("empirical_envelope_sha256"))
        or not valid_lower_sha256(authority.get("anchor_sha256"))
        or not isinstance(authority.get("session_id"), str)
        or not authority["session_id"]
        or not isinstance(authority.get("state_instance_id"), str)
        or not authority["state_instance_id"]
        or authority.get("empirical_envelope_sha256")
        in replay_state.get("empirical_spent_sha256", {})
    ):
        raise ValueError("J6 empirical authority binding is invalid")
    expires_text = authority.get("empirical_envelope_expires_at_utc")
    deadline_ns = authority.get("empirical_envelope_deadline_monotonic_ns")
    try:
        if not isinstance(expires_text, str) or not expires_text.endswith("Z"):
            raise ValueError
        expires = datetime.fromisoformat(
            expires_text[:-1] + "+00:00"
        ).astimezone(timezone.utc)
    except ValueError as exc:
        raise ValueError("J6 empirical authority expiry is invalid") from exc
    if (
        datetime.now(timezone.utc) >= expires
        or type(deadline_ns) is not int
        or not received_monotonic_ns < deadline_ns
        <= received_monotonic_ns + 4_200_000_000_000
    ):
        raise ValueError("J6 empirical authority expired")
    stage = authority.get("empirical_stage_index")
    target = authority.get("gravity_scale_target")
    scale = authority.get("gravity_scale")
    levels = (0.0, 0.25, 0.50, 0.75, 1.0)
    if (
        type(stage) is not int
        or not 0 <= stage < len(levels)
        or type(target) not in {int, float}
        or type(scale) not in {int, float}
        or not math.isfinite(float(target))
        or not math.isfinite(float(scale))
        or float(target) != levels[stage]
        or not 0.0 <= float(scale) <= 1.0
    ):
        raise ValueError("J6 empirical authority stage is invalid")
    feedforward = authority.get("feedforward_nm")
    if (
        not isinstance(feedforward, list)
        or len(feedforward) != 6
        or any(type(value) not in {int, float} or not math.isfinite(float(value)) for value in feedforward)
        or abs(float(feedforward[5])) > 1.0e-12
    ):
        raise ValueError("J6 empirical feedforward is invalid")
    position_authorized = authority.get(
        "empirical_position_validation_authorized"
    )
    if (
        type(position_authorized) is not bool
        or authority.get("empirical_maximum_position_segment_seconds") != 15.0
        or authority.get("empirical_maximum_abs_position_segment_deg") != 5.0
        or (position_authorized and stage != 4)
        or (command.get("mode") == "position" and not position_authorized)
    ):
        raise ValueError("J6 empirical POSITION authority is invalid")
    if command.get("mode") == "position":
        trajectory = command.get("trajectory")
        if (
            not isinstance(trajectory, dict)
            or trajectory.get("duration_ns", 0) > 15_000_000_000
            or abs(
                float(trajectory["target_rad"][5])
                - float(trajectory["start_rad"][5])
            ) > math.radians(5.0) + 1.0e-12
        ):
            raise ValueError("J6 empirical POSITION segment exceeds bounds")
    previous = replay_state.get("empirical_authority_binding")
    binding = (
        envelope_id,
        authority["empirical_envelope_sha256"],
        authority["anchor_sha256"],
        authority["session_id"],
        authority["state_instance_id"],
        stage,
    )
    if previous is None:
        if stage != 0:
            raise ValueError("J6 empirical authority must start at stage zero")
    elif (
        binding[:5] != previous[:5]
        or stage < previous[5]
        or stage > previous[5] + 1
    ):
        raise ValueError("J6 empirical authority stage sequence is invalid")
    enforce_gravity_authority_startup_binding(
        authority, "EMPIRICAL_VALIDATION_ENVELOPE"
    )
    return binding


def commit_empirical_command_authority(
    replay_state: dict, pending_binding: tuple | None
) -> None:
    if pending_binding is not None:
        replay_state["empirical_authority_binding"] = pending_binding
        # Empirical bindings begin with the envelope id.  Official authority
        # bindings begin with the authority-class token and must never arm the
        # empirical lifecycle latch.
        if (
            isinstance(pending_binding[0], str)
            and pending_binding[0].startswith("v15-31b-empirical-")
        ):
            replay_state["empirical_active_sha256"] = pending_binding[1]


def spend_active_empirical_command_authority(replay_state: dict) -> None:
    """Permanently fence a permit once its active lifecycle reaches BRAKE."""

    spent_sha256 = replay_state.get("empirical_active_sha256")
    if spent_sha256 is None:
        return
    spent = replay_state.setdefault("empirical_spent_sha256", {})
    spent.pop(spent_sha256, None)
    spent[spent_sha256] = True
    while len(spent) > COMMAND_SOURCE_REPLAY_LIMIT:
        del spent[next(iter(spent))]
    replay_state["empirical_active_sha256"] = None


def command_uses_empirical_gravity_authority(
    command: dict | None,
) -> bool:
    authority = None if command is None else command.get("gravity_authority")
    return bool(
        isinstance(authority, dict)
        and authority.get("schema")
        == "go-m8010-gravity-command-authority/1.1"
        and authority.get("authority_class")
        == "EMPIRICAL_VALIDATION_ENVELOPE"
    )


def empirical_gravity_authority_is_current(
    command: dict | None,
    *,
    now_monotonic_ns: int | None = None,
    now_utc: datetime | None = None,
) -> bool:
    """Check both frozen monotonic deadline and UTC diagnostic expiry."""

    if not command_uses_empirical_gravity_authority(command):
        return True
    authority = command["gravity_authority"]
    checked_ns = (
        time.monotonic_ns()
        if now_monotonic_ns is None
        else now_monotonic_ns
    )
    checked_utc = datetime.now(timezone.utc) if now_utc is None else now_utc
    deadline_ns = authority.get("empirical_envelope_deadline_monotonic_ns")
    expires_text = authority.get("empirical_envelope_expires_at_utc")
    try:
        expires = datetime.fromisoformat(
            expires_text[:-1] + "+00:00"
        ).astimezone(timezone.utc)
    except (AttributeError, ValueError):
        return False
    return bool(
        type(checked_ns) is int
        and type(deadline_ns) is int
        and checked_ns < deadline_ns
        and checked_utc.tzinfo is not None
        and checked_utc.astimezone(timezone.utc) < expires
    )


def position_execution_contract(command: dict) -> tuple:
    """Freeze every field that could alter one accepted POSITION reference."""

    base = (
        command.get("schema"),
        tuple(command.get("targets_rad", ())),
        tuple(command.get("active_joint_mask", ())),
        tuple(command.get("moving_joint_mask", ())),
        command.get("maximum_velocity_rad_s"),
        command.get("maximum_acceleration_rad_s2"),
    )
    if not command_is_v13_quintic_position(command):
        return base
    trajectory = command["trajectory"]
    return base + (
        command["plan_token_id"],
        trajectory["schema"],
        trajectory["trajectory_sha256"],
        trajectory["profile"],
        tuple(trajectory["start_rad"]),
        tuple(trajectory["target_rad"]),
        trajectory["duration_ns"],
        trajectory["interval_count"],
        trajectory["execute_at_monotonic_ns"],
        trajectory["segment_index"],
        trajectory["segment_count"],
    )


def validate_position_execution_transition(
    current: dict | None,
    candidate: dict,
    received_monotonic_ns: int,
    source_replay_state: dict | None = None,
) -> tuple | None:
    """Validate one POSITION authority transition without committing it.

    A source/activation-epoch pair owns one immutable POSITION execution
    contract.  The optional replay state keeps that authority across HOLD and
    BRAKE packets; returning a pending binding keeps rejected packets from
    mutating the authority table.
    """

    candidate_is_quintic = command_is_v13_quintic_position(candidate)
    candidate_is_position = bool(
        candidate.get("mode") == "position"
        and isinstance(candidate.get("moving_joint_mask"), list)
        and candidate["moving_joint_mask"][5] is True
    )
    if not candidate_is_position:
        return None

    binding_key = (
        candidate.get("source_instance_id"),
        candidate.get("activation_epoch"),
    )
    candidate_contract = position_execution_contract(candidate)
    if source_replay_state is not None:
        bindings = source_replay_state.get("position_execution_bindings", {})
        if binding_key in bindings:
            if bindings[binding_key] != candidate_contract:
                raise ValueError("POSITION同一激活纪元轨迹描述符发生变化")
            return binding_key, candidate_contract

    current_is_position = bool(
        current is not None
        and current.get("mode") == "position"
        and isinstance(current.get("moving_joint_mask"), list)
        and current["moving_joint_mask"][5] is True
    )
    same_epoch_position = bool(
        current_is_position
        and candidate.get("mode") == "position"
        and isinstance(candidate.get("moving_joint_mask"), list)
        and candidate["moving_joint_mask"][5] is True
        and current.get("source_instance_id")
        == candidate.get("source_instance_id")
        and current.get("activation_epoch") == candidate.get("activation_epoch")
    )
    if source_replay_state is None and same_epoch_position:
        if position_execution_contract(current) != candidate_contract:
            raise ValueError("POSITION同一激活纪元轨迹描述符发生变化")
        return None
    if (
        candidate_is_quintic
        and received_monotonic_ns
        >= candidate["trajectory"]["execute_at_monotonic_ns"]
    ):
        raise ValueError("POSITION quintic首包晚于执行起点")
    if source_replay_state is not None:
        return binding_key, candidate_contract
    return None


def v13_position_posvel_reference(
    command: dict,
    now_monotonic_ns: int,
    restore_velocity_limit: float = RESTORE_VELOCITY_LIMIT,
) -> tuple[float, float, str, int]:
    """Return descriptor-derived J6 p_des reference and unsigned speed cap."""

    if not command_is_v13_quintic_position(command):
        raise ValueError("command is not a J6 command/1.3 quintic POSITION")
    trajectory = command["trajectory"]
    q_ref, dq_ref, sample_index, trajectory_state = quintic_reference_at(
        trajectory["start_rad"][5],
        trajectory["target_rad"][5],
        trajectory["duration_ns"],
        trajectory["interval_count"],
        trajectory["execute_at_monotonic_ns"],
        now_monotonic_ns,
    )
    speed_limit = quintic_posvel_speed_limit(
        dq_ref,
        command["maximum_velocity_rad_s"],
        min(restore_velocity_limit, command["maximum_velocity_rad_s"]),
    )
    return q_ref, speed_limit, trajectory_state, sample_index


def trajectory_feedback_status(
    command: dict | None,
    now_monotonic_ns: int,
    *,
    state_override: str | None = None,
    sample_index_override: int | None = None,
) -> dict:
    """Build the optional feedback echo without granting any authority."""

    if not command_is_v13_quintic_position(command):
        return {
            "trajectory_plan_token_id": "",
            "trajectory_sha256": "",
            "trajectory_state": "INACTIVE",
            "trajectory_sample_index": 0,
            "trajectory_interval_count": 0,
        }
    trajectory = command["trajectory"]
    _q_ref, _dq_ref, sample_index, state = quintic_reference_at(
        trajectory["start_rad"][5],
        trajectory["target_rad"][5],
        trajectory["duration_ns"],
        trajectory["interval_count"],
        trajectory["execute_at_monotonic_ns"],
        now_monotonic_ns,
    )
    if state_override is not None and state_override not in {
        "PREPARED",
        "RUNNING",
        "COMPLETE",
        "INACTIVE",
    }:
        raise ValueError("trajectory feedback state is invalid")
    if (
        sample_index_override is not None
        and (
            type(sample_index_override) is not int
            or not 0 <= sample_index_override <= trajectory["interval_count"]
        )
    ):
        raise ValueError("trajectory feedback sample index is invalid")
    return {
        "trajectory_plan_token_id": command["plan_token_id"],
        "trajectory_sha256": trajectory["trajectory_sha256"],
        "trajectory_state": state if state_override is None else state_override,
        "trajectory_sample_index": (
            sample_index
            if sample_index_override is None
            else sample_index_override
        ),
        "trajectory_interval_count": trajectory["interval_count"],
    }


def command_requests_j6_active(command: dict | None) -> bool:
    return bool(
        command is not None
        and command["mode"] in {"hold", "position"}
        and command["active_joint_mask"][5]
    )


def command_requests_j6_fixed_hold(command: dict | None) -> bool:
    if not command_requests_j6_active(command):
        return False
    if command["mode"] == "hold":
        return True
    moving_mask = command.get("moving_joint_mask")
    # A legacy position packet without the field means moving=active.
    return bool(
        command["mode"] == "position"
        and moving_mask is not None
        and not moving_mask[5]
    )


def command_epoch_was_rejected(
    command: dict | None, highest_rejected_active_epoch: int
) -> bool:
    """Latch an unsafe active epoch so feedback motion cannot authorize it later."""

    return bool(
        command_requests_j6_active(command)
        and command["activation_epoch"] <= highest_rejected_active_epoch
    )


def effective_command_mode(
    command: dict | None, minimum_activation_epoch: int, now: float | None = None
) -> str:
    if not command_lease_is_fresh(command, now):
        return "brake"
    if not command_requests_j6_active(command):
        return "brake"
    if command["activation_epoch"] < minimum_activation_epoch:
        return "brake"
    return "hold" if command_requests_j6_fixed_hold(command) else command["mode"]


def command_lease_is_fresh(
    command: dict | None, now: float | None = None
) -> bool:
    if command is None:
        return False
    checked_at = time.monotonic() if now is None else now
    return 0.0 <= checked_at - command["received_at"] <= LEASE_S


def enabled_feedback_is_healthy(
    latest,
    latest_at: float | None,
    reference: float,
    fault_latched: bool,
    enabled: bool,
    enabled_confirmed: bool,
    now: float | None = None,
) -> bool:
    if (
        latest is None
        or latest_at is None
        or fault_latched
        or not enabled
        or not enabled_confirmed
    ):
        return False
    checked_at = time.monotonic() if now is None else now
    feedback_age = checked_at - latest_at
    logical = -(latest.position - reference)
    return bool(
        0.0 <= feedback_age <= FEEDBACK_MAX_AGE_S
        and latest.state == 1
        and math.isfinite(latest.position)
        and math.isfinite(latest.velocity)
        and FEEDBACK_HARD_LOWER <= logical <= FEEDBACK_HARD_UPPER
        and 0 <= latest.mos_temp < ACTIVE_THERMAL_LIMITS["thermal_stop_c"]
        and 0 <= latest.coil_temp < ACTIVE_THERMAL_LIMITS["thermal_stop_c"]
    )


def accept_hold_target(
    command: dict | None,
    previous_mode: str,
    last_accepted_target: float | None,
    last_accepted_epoch: int | None,
) -> tuple[float | None, int | None]:
    """Accept one fixed target per activation epoch, independent of mode toggles."""

    if not command_requests_j6_fixed_hold(command):
        return last_accepted_target, last_accepted_epoch
    if (
        last_accepted_epoch is None
        or command["activation_epoch"] > last_accepted_epoch
    ):
        return command["targets_rad"][5], command["activation_epoch"]
    # Keep the established target even if POSITION/moving, BRAKE, or DRAG was
    # observed in between.  A same/lower epoch can never re-anchor fixed HOLD.
    return last_accepted_target, last_accepted_epoch


def hold_transition_reuses_position_target(
    command: dict | None,
    previous_mode: str,
    last_position_target: float | None,
    last_position_epoch: int | None,
) -> bool:
    return bool(
        command_requests_j6_fixed_hold(command)
        and previous_mode == "position"
        and last_position_target is not None
        and last_position_epoch is not None
        and command["activation_epoch"] == last_position_epoch
        and abs(command["targets_rad"][5] - last_position_target) <= 1e-9
    )


def rejected_hold_fallback(
    enabled: bool,
    previous_mode: str,
    last_position_target: float | None,
    last_position_epoch: int | None,
    last_accepted_hold_target: float | None,
    last_accepted_hold_epoch: int | None,
) -> tuple[float | None, int | None]:
    """Preserve the currently authoritative target after rejecting a HOLD step."""

    if not enabled:
        return None, None
    # A POSITION target is newer than any HOLD target retained from before the
    # trajectory.  Falling back to that older HOLD target would reverse the arm
    # after rejecting an unsafe command.
    if (
        previous_mode == "position"
        and last_position_target is not None
        and last_position_epoch is not None
    ):
        return last_position_target, last_position_epoch
    if (
        last_accepted_hold_target is not None
        and last_accepted_hold_epoch is not None
    ):
        return last_accepted_hold_target, last_accepted_hold_epoch
    return None, None


def fixed_hold_profile_state(target: float) -> tuple[float, float]:
    """Enter a fixed target with no trajectory speed carried across modes."""

    return target, 0.0


def fixed_hold_velocity_limit() -> float:
    """Use the commissioned restore cap, independent of unaccepted packets."""

    return RESTORE_VELOCITY_LIMIT


def position_command_starts_new_profile(
    command: dict,
    previous_mode: str,
    last_position_target: float | None,
    last_position_epoch: int | None,
) -> bool:
    return bool(
        previous_mode != "position"
        or last_position_target is None
        or last_position_epoch is None
        or command["activation_epoch"] != last_position_epoch
        or abs(command["targets_rad"][5] - last_position_target) > 1e-9
    )


def position_command_target_is_authorized(
    command: dict | None,
    last_accepted_hold_target: float | None,
    last_accepted_hold_epoch: int | None,
    last_position_target: float | None,
    last_position_epoch: int | None,
) -> bool:
    """Require a new epoch before changing an active POSITION endpoint.

    Mode/moving-mask refreshes may reuse an epoch only while retaining the
    exact target already authorized for that epoch.  Otherwise a HOLD A/E
    packet followed by POSITION B/E could move to B and later let frozen HOLD
    bookkeeping snap the motor back to A.
    """

    if (
        command is None
        or command.get("mode") != "position"
        or command_requests_j6_fixed_hold(command)
    ):
        return True
    target = command["targets_rad"][5]
    epoch = command["activation_epoch"]
    if (
        last_accepted_hold_target is not None
        and last_accepted_hold_epoch == epoch
        and abs(target - last_accepted_hold_target) > 1e-9
    ):
        return False
    if (
        last_position_target is not None
        and last_position_epoch == epoch
        and abs(target - last_position_target) > 1e-9
    ):
        return False
    return True


def fault_dominant_mode(mode: str, fault_latched: bool) -> str:
    """A hard health fault dominates every target-preserving fallback."""

    return "brake" if fault_latched else mode


def saturating_next_activation_epoch(epoch: int) -> int:
    maximum = (1 << 63) - 1
    if type(epoch) is not int or not 0 <= epoch <= maximum:
        raise ValueError("activation epoch is outside the command domain")
    return epoch if epoch == maximum else epoch + 1


def make_thermal_interlock_state() -> dict:
    return {
        "fault_latched": False,
        "release_observed": False,
        "cooldown_ready": False,
        "rearm_pending_next_cycle": False,
        "cooldown_frames": 0,
        "cooldown_started_at": None,
        "trip_reason": "",
        "trip_activation_epoch": 0,
        "minimum_rearm_epoch": 0,
    }


def reset_thermal_cooldown_evidence(state: dict) -> None:
    state["cooldown_ready"] = False
    state["cooldown_frames"] = 0
    state["cooldown_started_at"] = None


def latch_thermal_interlock(
    state: dict,
    trip_reason: str,
    active_epoch: int,
    current_minimum_epoch: int,
    highest_rejected_active_epoch: int,
) -> bool:
    """Latch one thermal policy edge without terminating the worker."""

    if trip_reason not in {
        "RAW_TEMPERATURE_LIMIT",
        "EXACT_TRAJECTORY_DERATING_ABORT",
    }:
        raise ValueError("J6热锁存原因无效")
    newly_latched = not state["fault_latched"]
    state["fault_latched"] = True
    state["release_observed"] = False
    state["rearm_pending_next_cycle"] = False
    reset_thermal_cooldown_evidence(state)
    if newly_latched:
        state["trip_reason"] = trip_reason
    state["trip_activation_epoch"] = max(
        state["trip_activation_epoch"], active_epoch
    )
    state["minimum_rearm_epoch"] = max(
        state["minimum_rearm_epoch"],
        current_minimum_epoch,
        saturating_next_activation_epoch(active_epoch),
        saturating_next_activation_epoch(highest_rejected_active_epoch),
    )
    return newly_latched


def observe_raw_temperature_thermal_trip(
    state: dict,
    raw_temperature_c: float,
    thermal_stop_c: float,
    active_epoch: int,
    current_minimum_epoch: int,
    highest_rejected_active_epoch: int,
) -> bool:
    """Latch one raw sample at/above stop without terminating the worker."""

    if not math.isfinite(raw_temperature_c) or raw_temperature_c < thermal_stop_c:
        return False
    return latch_thermal_interlock(
        state,
        "RAW_TEMPERATURE_LIMIT",
        active_epoch,
        current_minimum_epoch,
        highest_rejected_active_epoch,
    )


def observe_thermal_cooldown_frame(
    state: dict,
    valid_disabled_below_rearm: bool,
    observed_at: float,
    cooldown_seconds: float,
    minimum_frames: int,
) -> bool:
    """Require a continuous cool DISABLED window; any gap resets evidence."""

    if not state["fault_latched"]:
        reset_thermal_cooldown_evidence(state)
        return False
    if (
        not valid_disabled_below_rearm
        or not math.isfinite(observed_at)
        or cooldown_seconds <= 0.0
        or type(minimum_frames) is not int
        or minimum_frames <= 0
    ):
        reset_thermal_cooldown_evidence(state)
        state["release_observed"] = False
        state["rearm_pending_next_cycle"] = False
        return False
    if state["cooldown_frames"] == 0:
        state["cooldown_started_at"] = observed_at
    if (
        state["cooldown_started_at"] is None
        or observed_at < state["cooldown_started_at"]
    ):
        reset_thermal_cooldown_evidence(state)
        state["cooldown_started_at"] = observed_at
    state["cooldown_frames"] += 1
    elapsed = observed_at - state["cooldown_started_at"]
    was_ready = state["cooldown_ready"]
    state["cooldown_ready"] = bool(
        state["cooldown_frames"] >= minimum_frames
        and elapsed >= cooldown_seconds
    )
    return state["cooldown_ready"] and not was_ready


def make_no_progress_watchdog_state() -> dict:
    return {
        "fault_latched": False,
        "release_observed": False,
        "rearm_pending_next_cycle": False,
        "qualifying_frames": 0,
        "window_started_at": None,
        "window_baseline_error_rad": 0.0,
        "trip_position_error_rad": 0.0,
        "trip_activation_epoch": 0,
        "minimum_rearm_epoch": 0,
    }


def reset_no_progress_observation(state: dict) -> None:
    state["qualifying_frames"] = 0
    state["window_started_at"] = None
    state["window_baseline_error_rad"] = 0.0


def observe_no_progress_watchdog(
    state: dict,
    observation_valid: bool,
    position_error_rad: float,
    observed_at: float,
    active_epoch: int,
    current_minimum_epoch: int,
    highest_rejected_active_epoch: int,
    *,
    timeout_seconds: float = TARGET_TIMEOUT_S,
    minimum_improvement_rad: float = NO_PROGRESS_MINIMUM_IMPROVEMENT,
    minimum_frames: int = NO_PROGRESS_MINIMUM_QUALIFYING_FRAMES,
) -> bool:
    """Latch a large endpoint error that fails to improve before timeout."""

    if state["fault_latched"]:
        return False
    qualifying = bool(
        observation_valid
        and math.isfinite(position_error_rad)
        and position_error_rad > ARRIVAL_TOLERANCE
        and math.isfinite(observed_at)
    )
    if not qualifying:
        reset_no_progress_observation(state)
        return False
    if (
        timeout_seconds <= 0.0
        or minimum_improvement_rad <= 0.0
        or type(minimum_frames) is not int
        or minimum_frames <= 0
    ):
        raise ValueError("no-progress watchdog limits are invalid")
    if (
        state["qualifying_frames"] == 0
        or state["window_started_at"] is None
        or observed_at < state["window_started_at"]
    ):
        state["qualifying_frames"] = 1
        state["window_started_at"] = observed_at
        state["window_baseline_error_rad"] = position_error_rad
        return False
    if (
        state["window_baseline_error_rad"] - position_error_rad
        >= minimum_improvement_rad
    ):
        state["qualifying_frames"] = 1
        state["window_started_at"] = observed_at
        state["window_baseline_error_rad"] = position_error_rad
        return False
    state["qualifying_frames"] += 1
    elapsed = observed_at - state["window_started_at"]
    if (
        state["qualifying_frames"] < minimum_frames
        or elapsed < timeout_seconds
    ):
        return False
    state["fault_latched"] = True
    state["release_observed"] = False
    state["rearm_pending_next_cycle"] = False
    state["trip_position_error_rad"] = position_error_rad
    state["trip_activation_epoch"] = max(
        state["trip_activation_epoch"], active_epoch
    )
    state["minimum_rearm_epoch"] = max(
        state["minimum_rearm_epoch"],
        current_minimum_epoch,
        saturating_next_activation_epoch(active_epoch),
        saturating_next_activation_epoch(highest_rejected_active_epoch),
    )
    return True


def observe_explicit_interlock_release(
    state: dict,
    explicit_release_packet_received: bool,
    *,
    cooldown_required: bool,
) -> bool:
    """Record an operator/domain release; a cached command is insufficient."""

    if (
        not state["fault_latched"]
        or not explicit_release_packet_received
        or (cooldown_required and not state["cooldown_ready"])
    ):
        return False
    newly_observed = not state["release_observed"]
    state["release_observed"] = True
    return newly_observed


def request_interlock_rearm_for_next_cycle(
    state: dict,
    command: dict | None,
    command_lease_fresh: bool,
    valid_disabled_feedback: bool,
    highest_rejected_active_epoch: int,
    *,
    cooldown_required: bool,
) -> bool:
    """Accept only a fresh higher-epoch active command after explicit release."""

    if not (
        state["fault_latched"]
        and state["release_observed"]
        and (not cooldown_required or state["cooldown_ready"])
        and command_lease_fresh
        and command_requests_j6_active(command)
        and valid_disabled_feedback
        and command["activation_epoch"] > state["trip_activation_epoch"]
        and command["activation_epoch"] >= state["minimum_rearm_epoch"]
        and command["activation_epoch"] > highest_rejected_active_epoch
    ):
        return False
    state["rearm_pending_next_cycle"] = True
    return True


def apply_pending_interlock_rearm_at_cycle_start(state: dict) -> bool:
    """Keep the complete decision cycle braked, then clear at the next edge."""

    if not state["fault_latched"] or not state["rearm_pending_next_cycle"]:
        return False
    state["fault_latched"] = False
    state["release_observed"] = False
    state["rearm_pending_next_cycle"] = False
    if "trip_reason" in state:
        state["trip_reason"] = ""
    if "cooldown_ready" in state:
        reset_thermal_cooldown_evidence(state)
    else:
        reset_no_progress_observation(state)
    return True


def temperature_window_statistics(
    samples: list[tuple[float, float]],
    temperature_c: float,
    observed_at: float,
    window_seconds: float,
) -> tuple[float, float | None]:
    """Update a bounded time window and return median plus degC/min slope."""

    if not (
        math.isfinite(temperature_c)
        and math.isfinite(observed_at)
        and window_seconds > 0.0
    ):
        raise ValueError("temperature window sample is invalid")
    samples.append((observed_at, temperature_c))
    cutoff = observed_at - window_seconds
    while len(samples) > 1 and samples[0][0] < cutoff:
        del samples[0]
    median_c = statistics.median(value for _stamp, value in samples)
    elapsed = samples[-1][0] - samples[0][0]
    slope = None
    if elapsed > 0.0:
        slope = (samples[-1][1] - samples[0][1]) * 60.0 / elapsed
    return float(median_c), slope


def hold_command_entry_is_safe(
    command: dict | None,
    previous_mode: str,
    last_accepted_epoch: int | None,
    latest,
    latest_at: float | None,
    reference: float,
    now: float | None = None,
    last_position_target: float | None = None,
    last_position_epoch: int | None = None,
) -> bool:
    """Bound a newly authorized HOLD step without ever re-capturing its target."""

    if not command_requests_j6_fixed_hold(command):
        return True
    if hold_transition_reuses_position_target(
        command,
        previous_mode,
        last_position_target,
        last_position_epoch,
    ):
        # GUI automatic arrival converts POSITION to HOLD without changing the
        # exact target or epoch. External displacement must not make that
        # transition unsafe or cause the target to be re-sampled.
        return True
    if (
        last_accepted_epoch is not None
        and command["activation_epoch"] <= last_accepted_epoch
    ):
        # accept_hold_target freezes the first target for an epoch across all
        # intervening modes.  Same/lower-epoch target fields are ignored, so
        # they do not constitute a newly authorized HOLD step.
        return True
    checked_at = time.monotonic() if now is None else now
    if (
        latest is None
        or latest_at is None
        or not 0.0 <= checked_at - latest_at <= FEEDBACK_MAX_AGE_S
        or latest.state not in {0, 1}
        or not math.isfinite(latest.position)
        or not math.isfinite(latest.velocity)
        or latest.state in FAULT_STATES
        or not 0 <= latest.mos_temp < ACTIVE_THERMAL_LIMITS["thermal_stop_c"]
        or not 0 <= latest.coil_temp < ACTIVE_THERMAL_LIMITS["thermal_stop_c"]
    ):
        return False
    logical = -(latest.position - reference)
    authority = command.get("gravity_authority")
    entry_limit = (
        EMPIRICAL_INITIAL_HOLD_ENTRY_TARGET_LIMIT
        if command_uses_empirical_gravity_authority(command)
        and authority.get("empirical_position_validation_authorized") is False
        else HOLD_ENTRY_TARGET_LIMIT
    )
    return bool(
        FEEDBACK_HARD_LOWER <= logical <= FEEDBACK_HARD_UPPER
        and abs(command["targets_rad"][5] - logical)
        <= entry_limit + 1e-12
    )


def hold_command_entry_is_authorized(
    command: dict | None,
    highest_rejected_active_epoch: int,
    previous_mode: str,
    last_accepted_epoch: int | None,
    latest,
    latest_at: float | None,
    reference: float,
    now: float | None = None,
    last_position_target: float | None = None,
    last_position_epoch: int | None = None,
) -> bool:
    if command_epoch_was_rejected(command, highest_rejected_active_epoch):
        return False
    return hold_command_entry_is_safe(
        command,
        previous_mode,
        last_accepted_epoch,
        latest,
        latest_at,
        reference,
        now,
        last_position_target=last_position_target,
        last_position_epoch=last_position_epoch,
    )


def safe_pre_enable_mode(
    command: dict | None,
    minimum_activation_epoch: int,
    previous_mode: str,
    last_accepted_hold_epoch: int | None,
    latest,
    latest_at: float | None,
    reference: float,
    now: float | None = None,
    *,
    last_accepted_hold_target: float | None = None,
    last_position_target: float | None = None,
    last_position_epoch: int | None = None,
    highest_rejected_active_epoch: int = 0,
) -> str:
    mode = effective_command_mode(command, minimum_activation_epoch, now)
    if command_epoch_was_rejected(command, highest_rejected_active_epoch):
        return "brake"
    if mode == "position" and not position_command_target_is_authorized(
        command,
        last_accepted_hold_target,
        last_accepted_hold_epoch,
        last_position_target,
        last_position_epoch,
    ):
        return "brake"
    if mode == "hold" and not hold_command_entry_is_safe(
        command,
        previous_mode,
        last_accepted_hold_epoch,
        latest,
        latest_at,
        reference,
        now,
        last_position_target=last_position_target,
        last_position_epoch=last_position_epoch,
    ):
        return "brake"
    return mode


def capture_lease_safe_hold_target(
    command: dict | None,
    prior_external_hold_confirmed: bool,
    last_accepted_hold_target: float | None,
    latest,
    latest_at: float | None,
    reference: float,
    fault_latched: bool,
    enabled: bool,
    enabled_confirmed: bool,
    now: float | None = None,
    completed_position_target: float | None = None,
    rejected_active_target: float | None = None,
) -> float | None:
    checked_at = time.monotonic() if now is None else now
    if (
        not prior_external_hold_confirmed
        or not command_requests_j6_active(command)
        or command_lease_is_fresh(command, checked_at)
        or not enabled_feedback_is_healthy(
            latest,
            latest_at,
            reference,
            fault_latched,
            enabled,
            enabled_confirmed,
            checked_at,
        )
    ):
        return None
    if rejected_active_target is not None:
        # A rejected epoch may change packet mode, but it cannot make lease
        # expiry re-anchor the servo to externally displaced feedback.
        return rejected_active_target
    if command_requests_j6_fixed_hold(command):
        return last_accepted_hold_target
    if completed_position_target is not None:
        return completed_position_target
    # Lease expiry during an unfinished POSITION must keep restoring toward
    # the exact authorized endpoint.  Adopting measured feedback here would
    # silently turn any gravity/external-force displacement into a new target.
    if command is not None and command.get("mode") == "position":
        return float(command["targets_rad"][5])
    return last_accepted_hold_target


def next_prior_external_hold_confirmation(
    prior_confirmed: bool,
    *,
    lease_safe_hold_active: bool,
    enabled: bool,
    fault_latched: bool,
    external_active_confirmed_this_cycle: bool,
) -> bool:
    """Keep confirmed authority across a lease boundary inside one cycle.

    A command can be fresh when selected and sent near the start of a 10 ms
    control cycle, then become stale before end-of-cycle bookkeeping.  That
    wall-clock crossing is exactly when the next cycle must be allowed to
    transfer the already-confirmed target into lease-safe HOLD.  Therefore the
    confirmation is sticky until a successful transfer, an actual disable, or
    a hard fault; end-of-cycle lease age alone must never clear it.
    """

    if lease_safe_hold_active or not enabled or fault_latched:
        return False
    return bool(prior_confirmed or external_active_confirmed_this_cycle)


def external_command_explicitly_releases_safe_hold(
    command: dict | None, now: float | None = None
) -> bool:
    return bool(
        command_lease_is_fresh(command, now)
        and not command_requests_j6_active(command)
    )


def external_command_can_resume_from_safe_hold(
    command: dict | None,
    source_activation_epoch: int,
    minimum_activation_epoch: int,
    now: float | None = None,
) -> bool:
    return bool(
        command_lease_is_fresh(command, now)
        and command_requests_j6_active(command)
        and command["activation_epoch"] > source_activation_epoch
        and command["activation_epoch"] >= minimum_activation_epoch
    )


def external_command_can_safely_resume_from_safe_hold(
    command: dict | None,
    source_activation_epoch: int,
    minimum_activation_epoch: int,
    latest,
    latest_at: float | None,
    reference: float,
    now: float | None = None,
    highest_rejected_active_epoch: int = 0,
) -> bool:
    """Let a higher epoch replace lease HOLD only after entry validation."""

    checked_at = time.monotonic() if now is None else now
    if not external_command_can_resume_from_safe_hold(
        command,
        source_activation_epoch,
        minimum_activation_epoch,
        checked_at,
    ):
        return False
    if command_epoch_was_rejected(command, highest_rejected_active_epoch):
        return False
    mode = effective_command_mode(command, minimum_activation_epoch, checked_at)
    if mode == "position":
        return True
    return bool(
        mode == "hold"
        and hold_command_entry_is_safe(
            command,
            "hold",
            source_activation_epoch,
            latest,
            latest_at,
            reference,
            checked_at,
        )
    )


def command_source_for_lease_capture(
    previous_command: dict | None,
    previous_lease_expired_before_receive: bool,
    current_command: dict | None,
    minimum_activation_epoch: int,
    latest,
    latest_at: float | None,
    reference: float,
    now: float | None = None,
    highest_rejected_active_epoch: int = 0,
) -> dict | None:
    """Close the receive-at-expiry gap without delaying a safe takeover."""

    checked_at = time.monotonic() if now is None else now
    if (
        previous_lease_expired_before_receive
        and previous_command is not None
        and not external_command_explicitly_releases_safe_hold(
            current_command, checked_at
        )
        and not external_command_can_safely_resume_from_safe_hold(
            current_command,
            previous_command["activation_epoch"],
            minimum_activation_epoch,
            latest,
            latest_at,
            reference,
            checked_at,
            highest_rejected_active_epoch,
        )
    ):
        return previous_command
    return current_command


def rejected_takeover_epoch_at_lease_boundary(
    previous_command: dict | None,
    previous_lease_expired_before_receive: bool,
    current_command: dict | None,
    minimum_activation_epoch: int,
    latest,
    latest_at: float | None,
    reference: float,
    now: float | None = None,
    highest_rejected_active_epoch: int = 0,
) -> int | None:
    """Fence an unsafe higher-epoch takeover in the receive-at-expiry gap."""

    checked_at = time.monotonic() if now is None else now
    if (
        not previous_lease_expired_before_receive
        or previous_command is None
        or not external_command_can_resume_from_safe_hold(
            current_command,
            previous_command["activation_epoch"],
            minimum_activation_epoch,
            checked_at,
        )
        or external_command_can_safely_resume_from_safe_hold(
            current_command,
            previous_command["activation_epoch"],
            minimum_activation_epoch,
            latest,
            latest_at,
            reference,
            checked_at,
            highest_rejected_active_epoch,
        )
    ):
        return None
    return current_command["activation_epoch"]


def update_active_deadline_miss_count(
    previous_count: int,
    enabled: bool,
    cycle_interval_s: float,
) -> int:
    """Count slow active periods, including sleep and scheduler overrun.

    Adjacent loop-start intervals include work, sleep and scheduler latency,
    so a sustained 40 Hz loop cannot look healthy merely because its Python
    work finished in under 20 ms.
    """

    if not enabled:
        return 0
    if cycle_interval_s > ACTIVE_DEADLINE_CYCLE_LIMIT_S:
        return previous_count + 1
    return 0


def observe_valid_command_epoch(
    command: dict, minimum_activation_epoch: int, last_seen_activation_epoch: int
) -> tuple[int, int]:
    last_seen_activation_epoch = max(
        last_seen_activation_epoch, command["activation_epoch"]
    )
    if not command_requests_j6_active(command):
        minimum_activation_epoch = max(
            minimum_activation_epoch, last_seen_activation_epoch + 1
        )
    return minimum_activation_epoch, last_seen_activation_epoch


def command_epoch_is_acceptable(
    command: dict,
    last_seen_activation_epoch: int,
    minimum_activation_epoch: int = 0,
) -> bool:
    return bool(
        not command_requests_j6_active(command)
        or (
            command["activation_epoch"] >= last_seen_activation_epoch
            and command["activation_epoch"] >= minimum_activation_epoch
        )
    )


def make_command_rejection_state() -> dict:
    return {
        "total": 0,
        "last_reason": None,
        "by_reason": {},
        "last_logged_at": {},
        "pending_by_reason": {},
    }


def command_rejection_reason(error: Exception) -> str:
    if isinstance(error, json.JSONDecodeError):
        return "JSON格式无效"
    message = str(error).strip()
    if message in {
        "命令格式不匹配",
        "模式不允许",
        "旧版协议仅允许制动",
        "1.3协议仅允许轨迹位置命令或制动",
        "命令接收时钟无效",
        "命令来源实例无效",
        "命令来源时钟无效",
        "命令来源已过期或来自未来",
        "命令来源序列无效",
        "命令来源序列或时钟发生回放",
        "当前命令来源租约仍有效",
        "新命令来源必须使用更高激活纪元",
        "目标必须是六个有限数",
        "J6目标超出模型机械限位",
        "关节激活掩码必须是六个布尔值",
        "关节移动掩码必须是六个布尔值",
        "移动关节必须同时激活",
        "制动命令不得携带激活关节",
        "激活纪元必须是非负整数",
        "主动命令的激活纪元必须大于零",
        "速度或加速度无效",
        "POSITION quintic计划令牌无效",
        "POSITION quintic描述符无效",
        "POSITION quintic描述符格式不匹配",
        "POSITION quintic轨迹哈希无效",
        "POSITION quintic轨迹类型不匹配",
        "POSITION quintic起点必须是六个有限数",
        "POSITION quintic终点必须是六个有限数",
        "POSITION quintic终点与命令目标不匹配",
        "POSITION quintic J6必须是唯一移动关节",
        "POSITION quintic非移动关节起终点不匹配",
        "POSITION quintic J6起终点超出模型机械限位",
        "POSITION quintic移动关节位移必须非零",
        "POSITION quintic时长无效",
        "POSITION quintic采样网格无效",
        "POSITION quintic执行时钟无效",
        "POSITION quintic分段索引无效",
        "POSITION quintic轨迹超过速度或加速度上限",
        "POSITION同一激活纪元轨迹描述符发生变化",
        "POSITION quintic首包晚于执行起点",
        "主动命令激活纪元发生回放",
        "HOLD新目标超出当前反馈捕获窗口",
        "POSITION同一激活纪元目标发生变化",
        "活动命令激活纪元已被拒绝",
        "租约HOLD拒绝不安全的新目标",
    }:
        return message
    if isinstance(error, KeyError):
        return "命令缺少必需字段"
    if isinstance(error, (TypeError, ValueError, OverflowError)):
        return "命令字段类型或数值无效"
    return type(error).__name__


def flush_command_rejection_reports(
    state: dict,
    now: float | None = None,
    reasons: tuple[str, ...] | None = None,
) -> list[dict]:
    checked_at = time.monotonic() if now is None else now
    reports = []
    selected = tuple(state["by_reason"]) if reasons is None else reasons
    for reason in selected:
        pending = state["pending_by_reason"].get(reason, 0)
        last_logged_at = state["last_logged_at"].get(reason)
        if (
            pending == 0
            or last_logged_at is None
            or checked_at - last_logged_at < REJECTION_LOG_INTERVAL_S
        ):
            continue
        reports.append({
            "reason": reason,
            "reason_count": state["by_reason"][reason],
            "rejected_total": state["total"],
            "suppressed_since_last": pending,
            "initial": False,
        })
        state["pending_by_reason"][reason] = 0
        state["last_logged_at"][reason] = checked_at
    return reports


def record_command_rejection(
    state: dict, error: Exception, now: float | None = None
) -> list[dict]:
    checked_at = time.monotonic() if now is None else now
    reason = command_rejection_reason(error)
    state["total"] += 1
    state["last_reason"] = reason
    state["by_reason"][reason] = state["by_reason"].get(reason, 0) + 1
    if reason not in state["last_logged_at"]:
        state["last_logged_at"][reason] = checked_at
        state["pending_by_reason"][reason] = 0
        return [{
            "reason": reason,
            "reason_count": state["by_reason"][reason],
            "rejected_total": state["total"],
            "suppressed_since_last": 0,
            "initial": True,
        }]
    state["pending_by_reason"][reason] += 1
    return flush_command_rejection_reports(state, checked_at, (reason,))


def emit_command_rejection_reports(reports: list[dict]) -> None:
    for report in reports:
        if report["initial"]:
            print(
                f"拒绝GUI命令：{report['reason']}; "
                f"reason_count={report['reason_count']}; "
                f"rejected_total={report['rejected_total']}",
                flush=True,
            )
        else:
            print(
                f"拒绝GUI命令汇总：reason={report['reason']}; "
                f"suppressed={report['suppressed_since_last']}; "
                f"reason_count={report['reason_count']}; "
                f"rejected_total={report['rejected_total']}",
                flush=True,
            )


def minimum_epoch_after_interarrival_lease(
    current: dict | None, minimum_activation_epoch: int, received_at: float
) -> int:
    if (
        command_requests_j6_active(current)
        and current is not None
        and received_at - current["received_at"] > LEASE_S
    ):
        return max(minimum_activation_epoch, current["activation_epoch"] + 1)
    return minimum_activation_epoch


def make_command_source_replay_state() -> dict:
    return {
        "sources": {},
        "position_execution_bindings": {},
        "active_source_instance_id": None,
        "active_source_last_received_monotonic_ns": None,
        "empirical_authority_binding": None,
        "empirical_active_sha256": None,
        "empirical_spent_sha256": {},
    }


def make_command_receive_events() -> dict:
    return {"domain_release_received": False}


def command_source_takeover_is_blocked(
    command: dict,
    replay_state: dict,
    received_monotonic_ns: int,
) -> bool:
    if command["mode"] == "brake":
        return False
    active_source_instance_id = replay_state["active_source_instance_id"]
    active_source_last_received_ns = replay_state[
        "active_source_last_received_monotonic_ns"
    ]
    return bool(
        active_source_instance_id is not None
        and command["source_instance_id"] != active_source_instance_id
        and active_source_last_received_ns is not None
        and received_monotonic_ns - active_source_last_received_ns
        <= COMMAND_SOURCE_TAKEOVER_LEASE_NS
    )


def command_source_is_newer(
    command: dict,
    replay_state: dict,
    received_monotonic_ns: int,
) -> bool:
    if command["mode"] == "brake":
        return True
    source_instance_id = command["source_instance_id"]
    if command_source_takeover_is_blocked(
        command, replay_state, received_monotonic_ns
    ):
        return False
    previous = replay_state["sources"].get(source_instance_id)
    return bool(
        previous is None
        or (
            command["sequence"] > previous["sequence"]
            and command["source_monotonic_ns"] > previous["source_monotonic_ns"]
        )
    )


def command_source_takeover_epoch_is_fresh(
    command: dict,
    replay_state: dict,
    last_seen_activation_epoch: int,
) -> bool:
    """A different source must advance, never reuse, active authority."""

    if not command_requests_j6_active(command):
        return True
    active_source = replay_state.get("active_source_instance_id")
    return bool(
        active_source is None
        or command["source_instance_id"] == active_source
        or command["activation_epoch"] > last_seen_activation_epoch
    )


def commit_command_source(
    replay_state: dict,
    command: dict,
    received_monotonic_ns: int,
) -> None:
    if command["mode"] == "brake":
        return
    sources = replay_state["sources"]
    source_instance_id = command["source_instance_id"]
    # Reinsert an existing source so bounded eviction follows recent use.
    sources.pop(source_instance_id, None)
    sources[source_instance_id] = {
        "sequence": command["sequence"],
        "source_monotonic_ns": command["source_monotonic_ns"],
    }
    while len(sources) > COMMAND_SOURCE_REPLAY_LIMIT:
        del sources[next(iter(sources))]
    replay_state["active_source_instance_id"] = source_instance_id
    replay_state[
        "active_source_last_received_monotonic_ns"
    ] = received_monotonic_ns


def commit_position_execution_binding(
    replay_state: dict, pending_binding: tuple | None
) -> None:
    """Commit a fully validated POSITION binding with bounded LRU retention."""

    if pending_binding is None:
        return
    binding_key, execution_contract = pending_binding
    bindings = replay_state.setdefault("position_execution_bindings", {})
    # Reinsert an accepted heartbeat so eviction follows recent authority use.
    bindings.pop(binding_key, None)
    bindings[binding_key] = execution_contract
    while len(bindings) > COMMAND_SOURCE_REPLAY_LIMIT:
        del bindings[next(iter(bindings))]


def receive_latest(
    sock: socket.socket,
    current: dict | None,
    minimum_activation_epoch: int,
    last_seen_activation_epoch: int,
    rejection_state: dict | None = None,
    source_replay_state: dict | None = None,
    receive_events: dict | None = None,
    require_empirical_authority: bool = False,
) -> tuple[dict | None, int, int]:
    if rejection_state is None:
        rejection_state = make_command_rejection_state()
    if source_replay_state is None:
        source_replay_state = make_command_source_replay_state()
    emit_command_rejection_reports(
        flush_command_rejection_reports(rejection_state)
    )
    for _ in range(COMMAND_PACKET_BUDGET):
        try:
            payload = sock.recv(8192)
        except BlockingIOError:
            return current, minimum_activation_epoch, last_seen_activation_epoch
        try:
            received_monotonic_ns = time.monotonic_ns()
            minimum_activation_epoch = minimum_epoch_after_interarrival_lease(
                current,
                minimum_activation_epoch,
                received_monotonic_ns / 1_000_000_000.0,
            )
            candidate = parse_command(payload, received_monotonic_ns)
            pending_empirical_binding = None
            if require_empirical_authority:
                pending_empirical_binding = validate_empirical_command_authority(
                    candidate, received_monotonic_ns, source_replay_state
                )
            if not command_epoch_is_acceptable(
                candidate,
                last_seen_activation_epoch,
                minimum_activation_epoch,
            ):
                raise ValueError("主动命令激活纪元发生回放")
            if not command_source_is_newer(
                candidate, source_replay_state, received_monotonic_ns
            ):
                if command_source_takeover_is_blocked(
                    candidate,
                    source_replay_state,
                    received_monotonic_ns,
                ):
                    raise ValueError("当前命令来源租约仍有效")
                raise ValueError("命令来源序列或时钟发生回放")
            if not command_source_takeover_epoch_is_fresh(
                candidate,
                source_replay_state,
                last_seen_activation_epoch,
            ):
                raise ValueError("新命令来源必须使用更高激活纪元")
            pending_position_binding = validate_position_execution_transition(
                current,
                candidate,
                received_monotonic_ns,
                source_replay_state,
            )
            if (
                require_empirical_authority
                and not command_requests_j6_active(candidate)
                and source_replay_state.get("empirical_active_sha256")
                is not None
            ):
                spend_active_empirical_command_authority(
                    source_replay_state
                )
            minimum_activation_epoch, last_seen_activation_epoch = (
                observe_valid_command_epoch(
                    candidate,
                    minimum_activation_epoch,
                    last_seen_activation_epoch,
                )
            )
            commit_command_source(
                source_replay_state, candidate, received_monotonic_ns
            )
            commit_position_execution_binding(
                source_replay_state, pending_position_binding
            )
            if require_empirical_authority:
                commit_empirical_command_authority(
                    source_replay_state, pending_empirical_binding
                )
            if receive_events is not None and not command_requests_j6_active(
                candidate
            ):
                receive_events["domain_release_received"] = True
            current = candidate
        except GravityAuthorityStartupBindingError as exc:
            # A packet that omits or changes the launch-bound proof is not a
            # harmless candidate typo.  Revoke the cached active command now;
            # the control loop therefore emits only DISABLED/BRAKE until a
            # fully matching command is accepted again.
            # Revoking an already-active empirical command is a lifecycle
            # release.  Spend synchronously here, before another datagram can
            # present the same envelope at a higher activation epoch.
            spend_active_empirical_command_authority(source_replay_state)
            current = None
            emit_command_rejection_reports(
                record_command_rejection(rejection_state, exc)
            )
        except Exception as exc:
            emit_command_rejection_reports(
                record_command_rejection(rejection_state, exc)
            )
    return current, minimum_activation_epoch, last_seen_activation_epoch


def drain_feedback_batch(logger: RawCanLogger):
    decoded_events = []
    fatal_event = False
    for event in logger.drain_events():
        classification = logger.classify(event)
        if classification == "NORMAL_FEEDBACK":
            decoded_events.append((strict_decode(event.payload), event.event_monotonic_s))
        elif event.event_kind == "ERROR_CALLBACK" or (
            event.event_kind == "RX_CALLBACK" and classification == "UNKNOWN"
        ):
            fatal_event = True
    return decoded_events, fatal_event


def send_feedback(
    sock: socket.socket, port: int, decoded, communication_ok: bool,
    mode: str, fault_latched: bool, lease_safe_hold: bool,
    rejection_state: dict | None = None,
    trajectory_status: dict | None = None,
    thermal_status: dict | None = None,
    no_progress_status: dict | None = None,
    last_valid_feedback_monotonic_ns: int | None = None,
) -> None:
    global J6_FEEDBACK_SEQUENCE
    J6_FEEDBACK_SEQUENCE += 1
    if J6_FEEDBACK_SEQUENCE > (1 << 63) - 1:
        raise RuntimeError("J6 feedback sequence exhausted")
    feedback_source_ns = time.monotonic_ns()
    binding = EXPECTED_FEEDBACK_IDENTITY_BINDING
    if decoded is None:
        position = velocity = 0.0
        mos = coil = 0
        state = -1
    else:
        position = decoded.position
        velocity = decoded.velocity
        mos = decoded.mos_temp
        coil = decoded.coil_temp
        state = decoded.state
    if trajectory_status is None:
        trajectory_status = {
            "trajectory_plan_token_id": "",
            "trajectory_sha256": "",
            "trajectory_state": "INACTIVE",
            "trajectory_sample_index": 0,
            "trajectory_interval_count": 0,
        }
    normalized_trajectory_status = {
        "trajectory_plan_token_id": trajectory_status.get(
            "trajectory_plan_token_id", ""
        ),
        "trajectory_sha256": trajectory_status.get("trajectory_sha256", ""),
        "trajectory_state": trajectory_status.get(
            "trajectory_state", "INACTIVE"
        ),
        "trajectory_sample_index": trajectory_status.get(
            "trajectory_sample_index", 0
        ),
        "trajectory_interval_count": trajectory_status.get(
            "trajectory_interval_count", 0
        ),
    }
    if thermal_status is None:
        thermal_status = {}
    normalized_thermal_status = {
        "thermal_state": str(thermal_status.get("thermal_state", "UNKNOWN")),
        "thermal_derating_factor": float(
            thermal_status.get("thermal_derating_factor", 1.0)
        ),
        "thermal_raw_temperature_c": float(
            thermal_status.get("thermal_raw_temperature_c", max(mos, coil))
        ),
        "thermal_window_median_c": float(
            thermal_status.get("thermal_window_median_c", max(mos, coil))
        ),
        "thermal_slope_c_per_min": thermal_status.get(
            "thermal_slope_c_per_min"
        ),
        "thermal_fault_latched": bool(
            thermal_status.get("thermal_fault_latched", False)
        ),
        "thermal_cooldown_ready": bool(
            thermal_status.get("thermal_cooldown_ready", False)
        ),
        "thermal_release_observed": bool(
            thermal_status.get("thermal_release_observed", False)
        ),
        "thermal_rearm_pending_next_cycle": bool(
            thermal_status.get("thermal_rearm_pending_next_cycle", False)
        ),
        "thermal_cooldown_valid_brake_frames": int(
            thermal_status.get("thermal_cooldown_valid_brake_frames", 0)
        ),
        "thermal_trip_activation_epoch": int(
            thermal_status.get("thermal_trip_activation_epoch", 0)
        ),
        "thermal_minimum_rearm_epoch": int(
            thermal_status.get("thermal_minimum_rearm_epoch", 0)
        ),
        "thermal_trip_reason": str(
            thermal_status.get(
                "thermal_trip_reason",
                "RAW_TEMPERATURE_LIMIT"
                if thermal_status.get("thermal_fault_latched", False)
                else "",
            )
        ),
        "thermal_config_sha256": str(
            thermal_status.get(
                "thermal_config_sha256", THERMAL_CONFIG_SHA256
            )
        ),
    }
    if no_progress_status is None:
        no_progress_status = {}
    normalized_no_progress_status = {
        "no_progress_fault": bool(
            no_progress_status.get("no_progress_fault", False)
        ),
        "load_limit_fault": bool(
            no_progress_status.get("no_progress_fault", False)
        ),
        "load_limit_no_progress": bool(
            no_progress_status.get("no_progress_fault", False)
        ),
        "no_progress_release_observed": bool(
            no_progress_status.get("no_progress_release_observed", False)
        ),
        "no_progress_rearm_pending_next_cycle": bool(
            no_progress_status.get(
                "no_progress_rearm_pending_next_cycle", False
            )
        ),
        "no_progress_watchdog_qualifying_frames": int(
            no_progress_status.get(
                "no_progress_watchdog_qualifying_frames", 0
            )
        ),
        "no_progress_observation_valid": bool(
            no_progress_status.get("no_progress_observation_valid", False)
        ),
        "no_progress_position_error_rad": float(
            no_progress_status.get("no_progress_position_error_rad", 0.0)
        ),
        "no_progress_trip_position_error_rad": float(
            no_progress_status.get(
                "no_progress_trip_position_error_rad", 0.0
            )
        ),
        "software_saturation_observed": False,
        "load_limit_watchdog_authority": str(
            no_progress_status.get(
                "load_limit_watchdog_authority",
                NO_PROGRESS_WATCHDOG_AUTHORITY,
            )
        ),
        "no_progress_trip_activation_epoch": int(
            no_progress_status.get("no_progress_trip_activation_epoch", 0)
        ),
        "no_progress_minimum_rearm_epoch": int(
            no_progress_status.get("no_progress_minimum_rearm_epoch", 0)
        ),
        "position_safety_trip_reason": str(
            no_progress_status.get(
                "position_safety_trip_reason",
                "POSITION_ARRIVAL_TIMEOUT"
                if no_progress_status.get("no_progress_fault", False)
                else "",
            )
        ),
    }
    interlock_latched = bool(
        normalized_thermal_status["thermal_fault_latched"]
        or normalized_no_progress_status["no_progress_fault"]
    )
    returned_controller_mode = (
        "unknown"
        if not communication_ok or state in FAULT_STATES
        else "brake"
        if mode == "brake" and state == 0
        else mode
        if mode in {"hold", "position"} and state == 1
        else "unknown"
    )
    payload = {
        "schema": "go-m8010-motor-feedback/1.0",
        "source_instance_id": J6_FEEDBACK_SOURCE_INSTANCE_ID,
        "sequence": J6_FEEDBACK_SEQUENCE,
        "session_id": binding.session_id,
        "state_instance_id": binding.state_instance_id,
        "source_monotonic_ns": feedback_source_ns,
        "samples": [{
            "motor": "J6",
            "position_rad": position,
            "velocity_rad_s": velocity,
            # POS_VEL mode exposes no authoritative torque command or torque
            # feedback channel.  Explicit nulls keep the three physical
            # torque semantics distinct instead of manufacturing estimates.
            "tau_cmd_rotor_nm": None,
            "tau_feedback_rotor_nm": None,
            "tau_joint_estimated_nm": None,
            "last_valid_feedback_monotonic_ns": (
                last_valid_feedback_monotonic_ns
            ),
            "temperature_c": max(mos, coil),
            "merror": 0 if communication_ok and state not in FAULT_STATES else state,
            "communication_ok": communication_ok and state not in FAULT_STATES,
            "thermal_fault_latched": normalized_thermal_status[
                "thermal_fault_latched"
            ],
            "load_limit_no_progress": normalized_no_progress_status[
                "no_progress_fault"
            ],
            **normalized_trajectory_status,
        }],
        "controller_mode": mode,
        "tau_j2_logical_total_nm": None,
        "controller_mode_by_motor": {"J6": returned_controller_mode},
        "domain_fault": fault_latched or interlock_latched,
        "drive_state": state,
        "lease_safe_hold": lease_safe_hold,
        "rejected_commands": 0 if rejection_state is None else rejection_state["total"],
        "last_rejection_reason": (
            None if rejection_state is None else rejection_state["last_reason"]
        ),
        "rejected_commands_by_reason": (
            {} if rejection_state is None
            else dict(sorted(rejection_state["by_reason"].items()))
        ),
        "suppressed_rejection_logs_pending": (
            {} if rejection_state is None
            else {
                reason: count
                for reason, count in sorted(
                    rejection_state["pending_by_reason"].items()
                )
                if count > 0
            }
        ),
    }
    payload.update(normalized_trajectory_status)
    payload.update(normalized_thermal_status)
    payload["thermal_state_by_motor"] = {
        "J6": normalized_thermal_status["thermal_state"]
    }
    payload.update(normalized_no_progress_status)
    sock.sendto(json.dumps(payload, separators=(",", ":")).encode(), ("127.0.0.1", port))


def disable_and_verify(transport: DmG6220PosVelTransport, logger: RawCanLogger) -> bool:
    for index in range(3):
        transport.send_disable(f"GUI_FD_DISABLE_{index + 1}")
        time.sleep(0.02)
    logger.drain_events()
    states = []
    for index in range(50):
        logger.send(0x7FF, refresh_request(), f"GUI_FINAL_DISABLED_{index}")
        time.sleep(PERIOD)
        decoded_events, fatal_event = drain_feedback_batch(logger)
        if fatal_event:
            return False
        states.extend(decoded.state for decoded, _ in decoded_events)
        if len(states) >= 5 and all(state == 0 for state in states[-5:]):
            return True
    return False


def run(args: argparse.Namespace) -> int:
    global ACTIVE_THERMAL_LIMITS
    global EXPECTED_GRAVITY_AUTHORITY_BINDING
    global EXPECTED_FEEDBACK_IDENTITY_BINDING
    global J6_FEEDBACK_SOURCE_INSTANCE_ID
    global J6_FEEDBACK_SEQUENCE
    thermal_limits = load_thermal_limits(args.thermal_config)
    ACTIVE_THERMAL_LIMITS = thermal_limits
    if not args.execute:
        print(
            "DRY_RUN=YES\nCAN_OPENED=NO\nDEFAULT_STATE=DISABLED\n"
            "CONTROL_LOOP_HZ=100\n"
            "COMMAND_TARGET_LIMIT_DEG=[-180,180]\n"
            "FEEDBACK_ENVELOPE_TOLERANCE_DEG=0.5\n"
            "POSITION_ARRIVAL_TIMEOUT_SECONDS=90\n"
            f"THERMAL_DERATE_START_C={thermal_limits['derating_start_c']}\n"
            f"THERMAL_STOP_C={thermal_limits['thermal_stop_c']}\n"
            f"THERMAL_REARM_BELOW_C={thermal_limits['rearm_below_c']}\n"
            f"THERMAL_COOLDOWN_SECONDS={thermal_limits['cooldown_seconds']}"
        )
        return 0
    if args.confirm != GATE:
        raise RuntimeError("缺少J6主动控制授权门")
    EXPECTED_GRAVITY_AUTHORITY_BINDING = (
        validate_gravity_authority_startup_binding(args)
    )
    EXPECTED_FEEDBACK_IDENTITY_BINDING = validate_feedback_startup_binding(
        args, EXPECTED_GRAVITY_AUTHORITY_BINDING
    )
    (
        J6_FEEDBACK_SOURCE_INSTANCE_ID,
        J6_FEEDBACK_SEQUENCE,
    ) = consume_feedback_handoff(
        args.feedback_handoff_file,
        EXPECTED_FEEDBACK_IDENTITY_BINDING,
        args.feedback_port,
    )
    persistent_reference = load_persistent_zero(args.zero_file)
    libc = ctypes.CDLL(None)
    if libc.prctl(1, signal.SIGTERM) != 0 or os.getppid() == 1:
        raise RuntimeError("父进程死亡保护设置失败")
    exact_usb_identity()
    lock_path = Path("/tmp/v15_30a_gui_j6.lock")
    lock_fd = os.open(lock_path, os.O_RDWR | os.O_CREAT | os.O_CLOEXEC, 0o600)
    fcntl.flock(lock_fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
    logger = RawCanLogger()
    transport = None
    command_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    command_socket.bind(("127.0.0.1", args.command_port))
    command_socket.setblocking(False)
    feedback_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    final_disabled = False
    try:
        master_id = int(read_parameter(logger, 7))
        logger.master_id = master_id
        if int(read_parameter(logger, 8)) != MOTOR_ID:
            raise RuntimeError("J6电机ID不匹配")
        if int(read_parameter(logger, 10)) != CTRL_MODE_POS_VEL:
            raise RuntimeError("J6当前不是已验证的POS_VEL模式；本任务禁止自动改写")
        transport = DmG6220PosVelTransport(master_id, logger=logger)
        capture = []
        latest = None
        latest_at = None
        start = time.monotonic()
        for index in range(400):
            scheduled = start + index * PERIOD
            if scheduled > time.monotonic():
                time.sleep(scheduled - time.monotonic())
            logger.send(0x7FF, refresh_request(), f"GUI_SESSION_CAPTURE_{index}")
            time.sleep(PERIOD * 0.8)
            decoded_events, fatal_event = drain_feedback_batch(logger)
            if fatal_event:
                raise RuntimeError("J6会话捕获期间收到异常CAN事件")
            for decoded, event_at in decoded_events:
                if decoded.state != 0:
                    raise RuntimeError("J6会话捕获窗口不是连续DISABLED")
                capture.append(decoded.position)
                latest, latest_at = decoded, event_at
            if len(capture) >= 100:
                capture = capture[-100:]
                break
        if len(capture) < 100:
            raise RuntimeError("J6会话参考反馈不足或启动时未处于DISABLED")
        captured_reference = statistics.median(capture)
        if (math.degrees(max(capture) - min(capture)) > 0.20 or
                math.degrees(abs(statistics.median(capture[-10:]) - captured_reference)) > 0.10):
            raise RuntimeError("J6会话参考不静止")
        reference = (
            captured_reference
            if persistent_reference is None
            else persistent_reference
            + round((captured_reference - persistent_reference) / (2.0 * math.pi))
            * (2.0 * math.pi)
        )

        command = None
        enabled = False
        enabled_confirmed = False
        fault_latched = False
        communication_fault_latched = False
        non_communication_fault_latched = False
        communication_recovery_frames = 0
        communication_recovery_episode_count = 0
        q_command = 0.0
        dq_command = 0.0
        previous_mode = "brake"
        enabled_at = None
        position_started_at = None
        position_arrival_overdue = False
        last_position_target = None
        last_position_epoch = None
        minimum_activation_epoch = 0
        last_seen_activation_epoch = 0
        lease_safe_hold_active = False
        lease_safe_hold_target = None
        lease_safe_hold_source_epoch = 0
        prior_external_hold_confirmed = False
        last_accepted_hold_target = None
        last_accepted_hold_epoch = None
        command_rejection_state = make_command_rejection_state()
        command_source_replay_state = make_command_source_replay_state()
        command_receive_events = make_command_receive_events()
        thermal_interlock = make_thermal_interlock_state()
        no_progress_watchdog = make_no_progress_watchdog_state()
        temperature_samples = []
        current_temperature_c = float(max(latest.mos_temp, latest.coil_temp))
        temperature_median_c = current_temperature_c
        temperature_slope_c_per_min = None
        thermal_factor = thermal_derating_factor(
            current_temperature_c,
            thermal_limits["derating_start_c"],
            thermal_limits["thermal_stop_c"],
            THERMAL_MINIMUM_ACTIVE_FACTOR,
        )
        no_progress_observation_valid = False
        no_progress_position_error_rad = 0.0
        highest_rejected_active_epoch = 0
        last_unsafe_active_signature = None
        last_unsafe_lease_resume_signature = None
        next_tick = time.monotonic()
        active_deadline_miss_count = 0
        active_deadline_degraded = False
        rapid_motion_degraded = False
        previous_cycle_started_at = None
        previous_cycle_enabled = False
        cycles = 0

        def latch_communication_loss(reason: str) -> None:
            nonlocal command
            nonlocal communication_fault_latched
            nonlocal communication_recovery_episode_count
            nonlocal communication_recovery_frames
            nonlocal dq_command
            nonlocal enabled
            nonlocal enabled_at
            nonlocal enabled_confirmed
            nonlocal final_disabled
            nonlocal fault_latched
            nonlocal highest_rejected_active_epoch
            nonlocal last_accepted_hold_epoch
            nonlocal last_accepted_hold_target
            nonlocal last_position_epoch
            nonlocal last_position_target
            nonlocal lease_safe_hold_active
            nonlocal lease_safe_hold_target
            nonlocal minimum_activation_epoch
            nonlocal position_arrival_overdue
            nonlocal position_started_at
            nonlocal prior_external_hold_confirmed
            nonlocal q_command

            if not communication_fault_latched:
                lost_epoch = max(
                    last_seen_activation_epoch,
                    0 if command is None else command["activation_epoch"],
                )
                highest_rejected_active_epoch = max(
                    highest_rejected_active_epoch, lost_epoch
                )
                minimum_activation_epoch = max(
                    minimum_activation_epoch,
                    saturating_next_activation_epoch(lost_epoch),
                )
                communication_recovery_episode_count += 1
                print(
                    "J6_COMMUNICATION_RECOVERY_BEGIN "
                    f"episode={communication_recovery_episode_count} "
                    f"reason={reason} "
                    f"minimum_activation_epoch={minimum_activation_epoch} "
                    "old_command_discarded=YES repreview_required=YES",
                    flush=True,
                )
            communication_fault_latched = True
            communication_recovery_frames = 0
            fault_latched = True
            final_disabled = False
            for disable_index in range(3):
                try:
                    transport.send_disable(
                        f"GUI_COMM_RECOVERY_DISABLE_{disable_index + 1}"
                    )
                except Exception:
                    pass
            spend_active_empirical_command_authority(
                command_source_replay_state
            )
            command = None
            enabled = False
            enabled_confirmed = False
            enabled_at = None
            lease_safe_hold_active = False
            lease_safe_hold_target = None
            prior_external_hold_confirmed = False
            position_started_at = None
            position_arrival_overdue = False
            last_position_target = None
            last_position_epoch = None
            last_accepted_hold_target = None
            last_accepted_hold_epoch = None
            q_command = 0.0
            dq_command = 0.0

        def consume_transport_interlock(label: str) -> bool:
            if not transport.consume_communication_interlock():
                return False
            latch_communication_loss(label)
            return True

        def latch_enabled_feedback_failure(reason: str) -> None:
            nonlocal fault_latched
            nonlocal non_communication_fault_latched
            checked_at = time.monotonic()
            transport_like = bool(
                latest is None
                or latest_at is None
                or checked_at - latest_at > FEEDBACK_MAX_AGE_S
                or (latest is not None and latest.state == 0)
            )
            if transport_like:
                latch_communication_loss(reason)
            else:
                non_communication_fault_latched = True
                fault_latched = True

        while not STOP:
            cycle_started_at = time.monotonic()
            thermal_rearmed_this_cycle = (
                apply_pending_interlock_rearm_at_cycle_start(
                    thermal_interlock
                )
            )
            no_progress_rearmed_this_cycle = (
                apply_pending_interlock_rearm_at_cycle_start(
                    no_progress_watchdog
                )
            )
            if thermal_rearmed_this_cycle:
                print(
                    "J6_THERMAL_REARM_APPLIED "
                    f"minimum_epoch={thermal_interlock['minimum_rearm_epoch']}",
                    flush=True,
                )
            if no_progress_rearmed_this_cycle:
                print(
                    "J6_LOAD_LIMIT_NO_PROGRESS_REARM_APPLIED "
                    f"minimum_epoch={no_progress_watchdog['minimum_rearm_epoch']}",
                    flush=True,
                )
            command_receive_events["domain_release_received"] = False
            if previous_cycle_started_at is not None:
                cycle_interval = cycle_started_at - previous_cycle_started_at
                active_deadline_miss_count = update_active_deadline_miss_count(
                    active_deadline_miss_count,
                    previous_cycle_enabled,
                    cycle_interval,
                )
                if cycle_interval > ACTIVE_DEADLINE_CYCLE_LIMIT_S:
                    # Drop the obsolete schedule instead of executing a burst
                    # of catch-up frames that would hide sustained low rate.
                    next_tick = cycle_started_at
                if (
                    active_deadline_miss_count
                    >= ACTIVE_DEADLINE_CONSECUTIVE_LIMIT
                    and not active_deadline_degraded
                ):
                    print(
                        "J6_ACTIVE_LOOP_DEGRADED "
                        f"consecutive_slow_cycles={active_deadline_miss_count}",
                        flush=True,
                    )
                    active_deadline_degraded = True
                elif (
                    active_deadline_degraded
                    and active_deadline_miss_count == 0
                ):
                    print("J6_ACTIVE_LOOP_RECOVERED", flush=True)
                    active_deadline_degraded = False
            previous_cycle_started_at = cycle_started_at
            command_before_receive = command
            lease_expired_before_receive = bool(
                command_requests_j6_active(command_before_receive)
                and not command_lease_is_fresh(
                    command_before_receive, cycle_started_at
                )
            )
            empirical_expired_before_receive = bool(
                command_uses_empirical_gravity_authority(
                    command_before_receive
                )
                and not empirical_gravity_authority_is_current(
                    command_before_receive
                )
            )
            if (
                lease_expired_before_receive
                or empirical_expired_before_receive
                or fault_latched
                or thermal_interlock["fault_latched"]
                or no_progress_watchdog["fault_latched"]
                or STOP
            ):
                spend_active_empirical_command_authority(
                    command_source_replay_state
                )
            command, minimum_activation_epoch, last_seen_activation_epoch = receive_latest(
                command_socket,
                command,
                minimum_activation_epoch,
                last_seen_activation_epoch,
                command_rejection_state,
                command_source_replay_state,
                command_receive_events,
                True,
            )
            if communication_fault_latched and command is not None:
                rejected_epoch = command["activation_epoch"]
                highest_rejected_active_epoch = max(
                    highest_rejected_active_epoch, rejected_epoch
                )
                minimum_activation_epoch = max(
                    minimum_activation_epoch,
                    saturating_next_activation_epoch(rejected_epoch),
                )
                command = None
            now = time.monotonic()
            command_lease_fresh = command_lease_is_fresh(command, now)
            empirical_authority_expired = bool(
                command_uses_empirical_gravity_authority(command)
                and not empirical_gravity_authority_is_current(command)
            )
            if (
                command_requests_j6_active(command)
                and command is not None
                and not command_lease_fresh
            ):
                minimum_activation_epoch = max(
                    minimum_activation_epoch, command["activation_epoch"] + 1
                )
            boundary_rejected_epoch = rejected_takeover_epoch_at_lease_boundary(
                command_before_receive,
                lease_expired_before_receive,
                command,
                minimum_activation_epoch,
                latest,
                latest_at,
                reference,
                now,
                highest_rejected_active_epoch,
            )
            if boundary_rejected_epoch is not None:
                highest_rejected_active_epoch = max(
                    highest_rejected_active_epoch,
                    boundary_rejected_epoch,
                )
            # Do not apply any new target using cached feedback that is already
            # known stale at this cycle's command boundary.  The previous
            # implementation detected this only after sending one active
            # POS_VEL frame.  Unconfirmed enable handshakes retain their
            # dedicated 200 ms confirmation path below.
            if (
                enabled
                and enabled_confirmed
                and not enabled_feedback_is_healthy(
                    latest,
                    latest_at,
                    reference,
                    fault_latched,
                    enabled,
                    enabled_confirmed,
                    now,
                )
            ):
                latch_enabled_feedback_failure("ACTIVE_FEEDBACK_UNHEALTHY")
            if lease_safe_hold_active:
                if external_command_explicitly_releases_safe_hold(command, now):
                    print(
                        "J6_LEASE_SAFE_HOLD_EXIT="
                        f"EXPLICIT_{command['mode'].upper()} "
                        f"source_epoch={lease_safe_hold_source_epoch}",
                        flush=True,
                    )
                    lease_safe_hold_active = False
                    lease_safe_hold_target = None
                    last_unsafe_lease_resume_signature = None
                elif not enabled_feedback_is_healthy(
                    latest,
                    latest_at,
                    reference,
                    fault_latched,
                    enabled,
                    enabled_confirmed,
                    now,
                ):
                    lease_safe_hold_active = False
                    lease_safe_hold_target = None
                    latch_enabled_feedback_failure(
                        "LEASE_SAFE_HOLD_FEEDBACK_UNHEALTHY"
                    )
                    last_unsafe_lease_resume_signature = None
                elif external_command_can_resume_from_safe_hold(
                    command,
                    lease_safe_hold_source_epoch,
                    minimum_activation_epoch,
                    now,
                ):
                    if external_command_can_safely_resume_from_safe_hold(
                        command,
                        lease_safe_hold_source_epoch,
                        minimum_activation_epoch,
                        latest,
                        latest_at,
                        reference,
                        now,
                        highest_rejected_active_epoch,
                    ):
                        print(
                            "J6_LEASE_SAFE_HOLD_EXIT=HIGHER_ACTIVATION_EPOCH "
                            f"source_epoch={lease_safe_hold_source_epoch} "
                            f"command_epoch={command['activation_epoch']}",
                            flush=True,
                        )
                        lease_safe_hold_active = False
                        lease_safe_hold_target = None
                        last_unsafe_lease_resume_signature = None
                    else:
                        highest_rejected_active_epoch = max(
                            highest_rejected_active_epoch,
                            command["activation_epoch"],
                        )
                        signature = (
                            command["activation_epoch"],
                            command["targets_rad"][5],
                        )
                        if signature != last_unsafe_lease_resume_signature:
                            emit_command_rejection_reports(record_command_rejection(
                                command_rejection_state,
                                ValueError("租约HOLD拒绝不安全的新目标"),
                            ))
                            last_unsafe_lease_resume_signature = signature
                else:
                    last_unsafe_lease_resume_signature = None
            else:
                last_unsafe_lease_resume_signature = None
            if not lease_safe_hold_active:
                lease_capture_source = command_source_for_lease_capture(
                    command_before_receive,
                    lease_expired_before_receive,
                    command,
                    minimum_activation_epoch,
                    latest,
                    latest_at,
                    reference,
                    now,
                    highest_rejected_active_epoch,
                )
                captured_target = (
                    None
                    if command_uses_empirical_gravity_authority(
                        lease_capture_source
                    )
                    else capture_lease_safe_hold_target(
                        lease_capture_source,
                        prior_external_hold_confirmed,
                        last_accepted_hold_target,
                        latest,
                        latest_at,
                        reference,
                        fault_latched
                        or thermal_interlock["fault_latched"]
                        or no_progress_watchdog["fault_latched"],
                        enabled,
                        enabled_confirmed,
                        now,
                        completed_position_target=(
                            last_position_target
                            if position_started_at is None
                            else None
                        ),
                        rejected_active_target=(
                            last_accepted_hold_target
                            if command_epoch_was_rejected(
                                lease_capture_source,
                                highest_rejected_active_epoch,
                            )
                            else None
                        ),
                    )
                )
                if captured_target is not None:
                    lease_safe_hold_active = True
                    lease_safe_hold_target = captured_target
                    lease_safe_hold_source_epoch = lease_capture_source[
                        "activation_epoch"
                    ]
                    # The lease target is the only safe fallback from here.
                    # Never retain an older HOLD target across a moving
                    # POSITION lease capture.
                    last_accepted_hold_target = captured_target
                    last_accepted_hold_epoch = lease_safe_hold_source_epoch
                    q_command = captured_target
                    dq_command = 0.0
                    print(
                        "J6_LEASE_SAFE_HOLD_ENTER "
                        f"source_mode={lease_capture_source['mode']} "
                        f"source_epoch={lease_safe_hold_source_epoch} "
                        f"logical_target_rad={lease_safe_hold_target}",
                        flush=True,
                    )
            mode = (
                "hold"
                if lease_safe_hold_active
                else effective_command_mode(command, minimum_activation_epoch, now)
            )
            if (
                fault_latched
                or thermal_interlock["fault_latched"]
                or no_progress_watchdog["fault_latched"]
                or empirical_authority_expired
                or STOP
            ):
                mode = "brake"
            if mode == "brake":
                spend_active_empirical_command_authority(
                    command_source_replay_state
                )
            rejected_active_command = bool(
                not lease_safe_hold_active
                and not fault_latched
                and command_epoch_was_rejected(
                    command, highest_rejected_active_epoch
                )
            )
            unauthorized_position_target = bool(
                not lease_safe_hold_active
                and not fault_latched
                and mode == "position"
                and not position_command_target_is_authorized(
                    command,
                    last_accepted_hold_target,
                    last_accepted_hold_epoch,
                    last_position_target,
                    last_position_epoch,
                )
            )
            unsafe_active_entry = bool(
                not lease_safe_hold_active
                and not fault_latched
                and (
                    rejected_active_command
                    or unauthorized_position_target
                    or (
                        mode == "hold"
                        and not hold_command_entry_is_authorized(
                            command,
                            highest_rejected_active_epoch,
                            previous_mode,
                            last_accepted_hold_epoch,
                            latest,
                            latest_at,
                            reference,
                            now,
                            last_position_target=last_position_target,
                            last_position_epoch=last_position_epoch,
                        )
                    )
                )
            )
            if unsafe_active_entry:
                highest_rejected_active_epoch = max(
                    highest_rejected_active_epoch,
                    command["activation_epoch"],
                )
                signature = (
                    command["activation_epoch"], command["targets_rad"][5]
                )
                if signature != last_unsafe_active_signature:
                    rejection = (
                        "POSITION同一激活纪元目标发生变化"
                        if unauthorized_position_target
                        else "活动命令激活纪元已被拒绝"
                        if rejected_active_command
                        else "HOLD新目标超出当前反馈捕获窗口"
                    )
                    emit_command_rejection_reports(record_command_rejection(
                        command_rejection_state,
                        ValueError(rejection),
                    ))
                    last_unsafe_active_signature = signature
                # Reject the new target without withdrawing an already-active
                # position servo. The previously accepted target remains the
                # only target eligible for HOLD/lease-HOLD.
                fallback_target, fallback_epoch = rejected_hold_fallback(
                    enabled,
                    previous_mode,
                    last_position_target,
                    last_position_epoch,
                    last_accepted_hold_target,
                    last_accepted_hold_epoch,
                )
                if fallback_target is not None:
                    last_accepted_hold_target = fallback_target
                    last_accepted_hold_epoch = fallback_epoch
                    q_command, dq_command = fixed_hold_profile_state(
                        fallback_target
                    )
                    mode = "hold"
                else:
                    mode = "brake"
            else:
                last_unsafe_active_signature = None
            if (
                not lease_safe_hold_active
                and mode == "hold"
                and not unsafe_active_entry
            ):
                last_accepted_hold_target, last_accepted_hold_epoch = (
                    accept_hold_target(
                        command,
                        previous_mode,
                        last_accepted_hold_target,
                        last_accepted_hold_epoch,
                    )
                )
                if enabled_confirmed:
                    q_command, dq_command = fixed_hold_profile_state(
                        last_accepted_hold_target
                    )
            if (
                mode in {"hold", "position"}
                and enabled
                and enabled_confirmed
                and not enabled_feedback_is_healthy(
                    latest,
                    latest_at,
                    reference,
                    fault_latched,
                    enabled,
                    enabled_confirmed,
                    time.monotonic(),
                )
            ):
                latch_enabled_feedback_failure("COMMAND_BOUNDARY_FEEDBACK_UNHEALTHY")
            # A hard health failure is terminal for active output in this
            # cycle.  Apply it after every rejected-target fallback so no HOLD
            # rewrite can emit one additional POS_VEL frame.
            mode = fault_dominant_mode(mode, fault_latched)
            if (
                thermal_interlock["fault_latched"]
                or no_progress_watchdog["fault_latched"]
                or empirical_authority_expired
            ):
                mode = "brake"
            if (
                mode == "position"
                and command_is_v13_quintic_position(command)
                and current_temperature_c
                >= thermal_limits["derating_start_c"]
            ):
                newly_latched = latch_thermal_interlock(
                    thermal_interlock,
                    "EXACT_TRAJECTORY_DERATING_ABORT",
                    command["activation_epoch"],
                    minimum_activation_epoch,
                    highest_rejected_active_epoch,
                )
                minimum_activation_epoch = max(
                    minimum_activation_epoch,
                    thermal_interlock["minimum_rearm_epoch"],
                )
                mode = "brake"
                if newly_latched:
                    print(
                        "J6_V13_THERMAL_DERATING_TRAJECTORY_ABORT "
                        f"temperature_c={current_temperature_c} "
                        f"activation_epoch={command['activation_epoch']} "
                        "policy=REPREVIEW_REQUIRED",
                        flush=True,
                    )
            active = mode in {"hold", "position"}
            if (
                active
                and mode == "position"
                and command_is_v13_quintic_position(command)
                and not enabled_confirmed
                and time.monotonic_ns()
                >= command["trajectory"]["execute_at_monotonic_ns"]
            ):
                # Never join a deterministic trajectory after its common start.
                # A later cycle must not fast-forward from an unconfirmed enable
                # handshake into the middle of the preflighted reference.
                non_communication_fault_latched = True
                fault_latched = True
                mode = "brake"
                active = False
            if mode == "position":
                requested_target = command["targets_rad"][5]
                if position_command_starts_new_profile(
                    command,
                    previous_mode,
                    last_position_target,
                    last_position_epoch,
                ):
                    position_started_at = time.monotonic()
                    position_arrival_overdue = False
                    last_position_target = requested_target
                    last_position_epoch = command["activation_epoch"]
                    reset_no_progress_observation(no_progress_watchdog)
                    # DM POS_VEL uses an unsigned speed cap; never apply an old
                    # high cap immediately to a newly authorized direction.
                    dq_command = 0.0
            else:
                position_started_at = None
                position_arrival_overdue = False
                last_position_target = None
                last_position_epoch = None
            if active and not enabled:
                refresh_deadline = time.monotonic() + FEEDBACK_MAX_AGE_S
                while (not STOP and (latest_at is None or
                       time.monotonic() - latest_at > FEEDBACK_MAX_AGE_S) and
                       time.monotonic() < refresh_deadline):
                    command, minimum_activation_epoch, last_seen_activation_epoch = receive_latest(
                        command_socket,
                        command,
                        minimum_activation_epoch,
                        last_seen_activation_epoch,
                        command_rejection_state,
                        command_source_replay_state,
                        command_receive_events,
                        True,
                    )
                    if safe_pre_enable_mode(
                        command,
                        minimum_activation_epoch,
                        previous_mode,
                        last_accepted_hold_epoch,
                        latest,
                        latest_at,
                        reference,
                        last_accepted_hold_target=last_accepted_hold_target,
                        last_position_target=last_position_target,
                        last_position_epoch=last_position_epoch,
                        highest_rejected_active_epoch=(
                            highest_rejected_active_epoch
                        ),
                    ) not in {"hold", "position"}:
                        break
                    logger.send(0x7FF, refresh_request(), "GUI_PRE_ENABLE_FRESHNESS")
                    if consume_transport_interlock("PRE_ENABLE_REFRESH"):
                        break
                    time.sleep(PERIOD)
                    decoded_events, fatal_event = drain_feedback_batch(logger)
                    if fatal_event:
                        latch_communication_loss("PRE_ENABLE_CAN_ERROR")
                    for refreshed, refreshed_at in decoded_events:
                        latest, latest_at = refreshed, refreshed_at
                command, minimum_activation_epoch, last_seen_activation_epoch = receive_latest(
                    command_socket,
                    command,
                    minimum_activation_epoch,
                    last_seen_activation_epoch,
                    command_rejection_state,
                    command_source_replay_state,
                    command_receive_events,
                    True,
                )
                pre_enable_mode = safe_pre_enable_mode(
                    command,
                    minimum_activation_epoch,
                    previous_mode,
                    last_accepted_hold_epoch,
                    latest,
                    latest_at,
                    reference,
                    last_accepted_hold_target=last_accepted_hold_target,
                    last_position_target=last_position_target,
                    last_position_epoch=last_position_epoch,
                    highest_rejected_active_epoch=(
                        highest_rejected_active_epoch
                    ),
                )
                if STOP or pre_enable_mode not in {"hold", "position"}:
                    mode = "brake"
                    active = False
                elif (fault_latched or latest is None or latest_at is None or latest.state != 0 or
                        time.monotonic() - latest_at > FEEDBACK_MAX_AGE_S):
                    if latest is None or latest_at is None or (
                        time.monotonic() - latest_at > FEEDBACK_MAX_AGE_S
                    ):
                        latch_communication_loss("PRE_ENABLE_FEEDBACK_UNAVAILABLE")
                    else:
                        non_communication_fault_latched = True
                        fault_latched = True
                    mode = "brake"
                    active = False
                else:
                    mode = pre_enable_mode
                    q_command = -(latest.position - reference)
                    dq_command = 0.0
                    transport.send_pos_vel_command(latest.position, 0.0, "GUI_PRELOAD_CURRENT_DISABLED")
                    if consume_transport_interlock("PRELOAD_CURRENT_DISABLED"):
                        continue
                    command, minimum_activation_epoch, last_seen_activation_epoch = receive_latest(
                        command_socket,
                        command,
                        minimum_activation_epoch,
                        last_seen_activation_epoch,
                        command_rejection_state,
                        command_source_replay_state,
                        command_receive_events,
                        True,
                    )
                    pre_enable_mode = safe_pre_enable_mode(
                        command,
                        minimum_activation_epoch,
                        previous_mode,
                        last_accepted_hold_epoch,
                        latest,
                        latest_at,
                        reference,
                        last_accepted_hold_target=last_accepted_hold_target,
                        last_position_target=last_position_target,
                        last_position_epoch=last_position_epoch,
                        highest_rejected_active_epoch=(
                            highest_rejected_active_epoch
                        ),
                    )
                    if STOP or pre_enable_mode not in {"hold", "position"}:
                        mode = "brake"
                        active = False
                    else:
                        mode = pre_enable_mode
                        final_disabled = False
                        transport.send_enable()
                        if consume_transport_interlock("ENABLE"):
                            continue
                        transport.send_pos_vel_command(latest.position, 0.0, "GUI_HOLD_IMMEDIATE")
                        if consume_transport_interlock("HOLD_IMMEDIATE"):
                            continue
                        enabled = True
                        enabled_confirmed = False
                        enabled_at = time.monotonic()
                        next_tick = time.monotonic()
            if not active and enabled:
                final_disabled = disable_and_verify(transport, logger)
                if not final_disabled:
                    latch_communication_loss("DISABLE_UNCONFIRMED")
                    continue
                enabled = False
                enabled_confirmed = False
                enabled_at = None
                next_tick = time.monotonic()

            cycle_trajectory_status = trajectory_feedback_status(
                command, time.monotonic_ns()
            )
            if enabled and not enabled_confirmed:
                transport.send_pos_vel_command(
                    reference - q_command, 0.0, "GUI_ENABLE_CONFIRM_HOLD"
                )
                if consume_transport_interlock("ENABLE_CONFIRM_HOLD"):
                    continue
            elif mode == "position" and command is not None and enabled:
                if command_is_v13_quintic_position(command):
                    q_ref, speed_limit, trajectory_state, sample_index = (
                        v13_position_posvel_reference(
                            command, time.monotonic_ns()
                        )
                    )
                    q_command = q_ref
                    # q_ref/speed_limit are the immutable preview samples.
                    # The policy above aborts and latches DISABLED before this
                    # branch whenever the thermal derating region is entered;
                    # independently scaling speed here would break q/dq
                    # equivalence with the signed PLAN_TOKEN.
                    dq_command = speed_limit
                    protocol_position = reference - q_ref
                    protocol_velocity = speed_limit
                    cycle_trajectory_status = trajectory_feedback_status(
                        command,
                        time.monotonic_ns(),
                        state_override=trajectory_state,
                        sample_index_override=sample_index,
                    )
                    transport.send_pos_vel_command(
                        protocol_position,
                        protocol_velocity,
                        "GUI_POSITION_QUINTIC_REFRESH",
                    )
                    if consume_transport_interlock("POSITION_QUINTIC"):
                        continue
                else:
                    requested_target = command["targets_rad"][5]
                    actual_logical = (
                        q_command
                        if latest is None
                        else -(latest.position - reference)
                    )
                    restore_limit = min(
                        RESTORE_VELOCITY_LIMIT,
                        command["maximum_velocity_rad_s"],
                    )
                    thermal_vmax, thermal_amax, thermal_restore_limit = (
                        thermal_derated_posvel_limits(
                            command["maximum_velocity_rad_s"],
                            command["maximum_acceleration_rad_s2"],
                            restore_limit,
                            thermal_factor,
                        )
                    )
                    dq_command = update_posvel_speed_limit(
                        dq_command,
                        actual_logical,
                        requested_target,
                        thermal_vmax,
                        thermal_amax,
                        thermal_restore_limit,
                        PERIOD,
                    )
                    q_command = requested_target
                    protocol_position = reference - requested_target
                    protocol_velocity = dq_command
                    transport.send_pos_vel_command(
                        protocol_position,
                        protocol_velocity,
                        "GUI_POSITION_REFRESH",
                    )
                    if consume_transport_interlock("POSITION"):
                        continue
            elif mode == "hold" and enabled:
                fixed_hold_target = (
                    lease_safe_hold_target
                    if lease_safe_hold_active
                    else last_accepted_hold_target
                )
                if fixed_hold_target is None:
                    raise RuntimeError("J6 HOLD目标尚未建立")
                hold_velocity_limit = (
                    fixed_hold_velocity_limit() * thermal_factor
                )
                transport.send_pos_vel_command(
                    reference - fixed_hold_target,
                    hold_velocity_limit,
                    "GUI_LEASE_SAFE_HOLD_REFRESH"
                    if lease_safe_hold_active
                    else "GUI_HOLD_REFRESH",
                )
                if consume_transport_interlock("HOLD_REFRESH"):
                    continue
            else:
                if communication_fault_latched:
                    transport.send_disable("GUI_COMM_RECOVERY_DISABLE_REFRESH")
                    if consume_transport_interlock(
                        "COMM_RECOVERY_DISABLE_REFRESH"
                    ):
                        continue
                logger.send(0x7FF, refresh_request(), "GUI_DISABLED_REFRESH")
                if consume_transport_interlock("DISABLED_REFRESH"):
                    continue

            decoded_events, fatal_event = drain_feedback_batch(logger)
            if fatal_event:
                latch_communication_loss("CAN_ERROR_EVENT")
            if decoded_events:
                latest, latest_at = decoded_events[-1]
                decoded = latest
                logical = -(decoded.position - reference)
                # Finite external motion must create restoring position error,
                # not withdraw drive authority. Drive faults, temperature,
                # feedback freshness and the mechanical envelope remain hard
                # disable conditions.
                if (
                    not FEEDBACK_HARD_LOWER
                    <= logical
                    <= FEEDBACK_HARD_UPPER
                    or not math.isfinite(decoded.velocity)
                ):
                    non_communication_fault_latched = True
                    fault_latched = True
                rapid_motion = bool(enabled and abs(decoded.velocity) > 0.7)
                if rapid_motion and not rapid_motion_degraded:
                    print(
                        "J6_EXTERNAL_MOTION_OBSERVED "
                        f"velocity_rad_s={decoded.velocity}",
                        flush=True,
                    )
                    rapid_motion_degraded = True
                elif rapid_motion_degraded and not rapid_motion:
                    print("J6_EXTERNAL_MOTION_SETTLED", flush=True)
                    rapid_motion_degraded = False
                current_temperature_c = float(
                    max(decoded.mos_temp, decoded.coil_temp)
                )
                temperature_median_c, temperature_slope_c_per_min = (
                    temperature_window_statistics(
                        temperature_samples,
                        current_temperature_c,
                        latest_at,
                        thermal_limits["slope_window_seconds"],
                    )
                )
                thermal_factor = thermal_derating_factor(
                    current_temperature_c,
                    thermal_limits["derating_start_c"],
                    thermal_limits["thermal_stop_c"],
                    THERMAL_MINIMUM_ACTIVE_FACTOR,
                )
                active_epoch = (
                    command["activation_epoch"]
                    if command_requests_j6_active(command)
                    else 0
                )
                thermal_tripped_this_cycle = observe_raw_temperature_thermal_trip(
                    thermal_interlock,
                    current_temperature_c,
                    thermal_limits["thermal_stop_c"],
                    active_epoch,
                    minimum_activation_epoch,
                    highest_rejected_active_epoch,
                )
                if thermal_interlock["fault_latched"]:
                    minimum_activation_epoch = max(
                        minimum_activation_epoch,
                        thermal_interlock["minimum_rearm_epoch"],
                    )
                if thermal_tripped_this_cycle:
                    print(
                        "J6_THERMAL_STOP_LATCHED "
                        f"temperature_c={current_temperature_c} "
                        f"activation_epoch={active_epoch} "
                        "worker_continues_online=YES",
                        flush=True,
                    )
                if current_temperature_c < 0.0 or decoded.state in FAULT_STATES:
                    non_communication_fault_latched = True
                    fault_latched = True
                if (enabled and decoded.state != 1 and enabled_at is not None and
                        time.monotonic() - enabled_at > 0.2):
                    if decoded.state == 0:
                        latch_communication_loss("DRIVE_RESET_TO_DISABLED")
                    else:
                        non_communication_fault_latched = True
                        fault_latched = True
                if (enabled and decoded.state == 1 and enabled_at is not None and
                        latest_at is not None and latest_at >= enabled_at and
                        not enabled_confirmed):
                    enabled_confirmed = True
                if not enabled and decoded.state != 0:
                    non_communication_fault_latched = True
                    fault_latched = True
            fresh = latest is not None and latest_at is not None and time.monotonic() - latest_at <= FEEDBACK_MAX_AGE_S
            if enabled and not fresh:
                latch_communication_loss("ACTIVE_FEEDBACK_STALE")
            if communication_fault_latched:
                recovery_sample_safe = bool(
                    decoded_events
                    and not fatal_event
                    and latest is not None
                    and latest.state == 0
                    and fresh
                    and math.isfinite(latest.position)
                    and math.isfinite(latest.velocity)
                    and abs(latest.velocity) <= 0.05
                    and FEEDBACK_HARD_LOWER
                    <= -(latest.position - reference)
                    <= FEEDBACK_HARD_UPPER
                    and 0
                    <= latest.mos_temp
                    < thermal_limits["thermal_stop_c"]
                    and 0
                    <= latest.coil_temp
                    < thermal_limits["thermal_stop_c"]
                )
                if recovery_sample_safe:
                    communication_recovery_frames += 1
                else:
                    communication_recovery_frames = 0
                if communication_recovery_frames >= 5:
                    communication_fault_latched = False
                    communication_recovery_frames = 0
                    fault_latched = non_communication_fault_latched
                    final_disabled = True
                    print(
                        "J6_COMMUNICATION_RECOVERY_READY "
                        f"episode={communication_recovery_episode_count} "
                        "stable_disabled_frames=5 "
                        f"adapter_reconnect_attempts={transport.reconnect_attempt_count} "
                        f"adapter_reconnect_successes={transport.reconnect_success_count} "
                        "old_command_discarded=YES repreview_required=YES",
                        flush=True,
                    )
            if thermal_interlock["fault_latched"]:
                cooldown_qualified = bool(
                    decoded_events
                    and not fatal_event
                    and latest is not None
                    and latest.state == 0
                    and fresh
                    and current_temperature_c
                    < thermal_limits["rearm_below_c"]
                    and math.isfinite(latest.position)
                    and math.isfinite(latest.velocity)
                )
                if decoded_events or not fresh:
                    if observe_thermal_cooldown_frame(
                        thermal_interlock,
                        cooldown_qualified,
                        time.monotonic(),
                        thermal_limits["cooldown_seconds"],
                        max(
                            1,
                            math.ceil(
                                thermal_limits["cooldown_seconds"] / PERIOD
                            ),
                        ),
                    ):
                        print(
                            "J6_THERMAL_COOLDOWN_READY "
                            f"valid_brake_frames={thermal_interlock['cooldown_frames']}",
                            flush=True,
                        )
            no_progress_observation_valid = bool(
                decoded_events
                and
                enabled
                and mode == "position"
                and latest is not None
                and last_position_target is not None
                and enabled_confirmed
                and fresh
                and not fault_latched
                and not thermal_interlock["fault_latched"]
                and not no_progress_watchdog["fault_latched"]
                and (
                    not command_is_v13_quintic_position(command)
                    or time.monotonic_ns()
                    >= command["trajectory"]["execute_at_monotonic_ns"]
                )
            )
            no_progress_position_error_rad = 0.0
            if no_progress_observation_valid:
                no_progress_position_error_rad = abs(
                    (-(latest.position - reference)) - last_position_target
                )
                if no_progress_position_error_rad <= ARRIVAL_TOLERANCE:
                    position_started_at = None
                if observe_no_progress_watchdog(
                    no_progress_watchdog,
                    True,
                    no_progress_position_error_rad,
                    time.monotonic(),
                    command["activation_epoch"],
                    minimum_activation_epoch,
                    highest_rejected_active_epoch,
                ):
                    minimum_activation_epoch = max(
                        minimum_activation_epoch,
                        no_progress_watchdog["minimum_rearm_epoch"],
                    )
                    print(
                        "J6_POSITION_ARRIVAL_OVERDUE "
                        f"error_rad={no_progress_position_error_rad} "
                        "action=LATCHED_SAFE_BRAKE",
                        flush=True,
                    )
                    print(
                        "J6_LOAD_LIMIT_NO_PROGRESS "
                        f"activation_epoch={command['activation_epoch']} "
                        f"minimum_rearm_epoch={no_progress_watchdog['minimum_rearm_epoch']} "
                        f"authority={NO_PROGRESS_WATCHDOG_AUTHORITY}",
                        flush=True,
                    )
            elif decoded_events:
                observe_no_progress_watchdog(
                    no_progress_watchdog,
                    False,
                    0.0,
                    time.monotonic(),
                    0,
                    minimum_activation_epoch,
                    highest_rejected_active_epoch,
                )
            software_interlock_latched = bool(
                thermal_interlock["fault_latched"]
                or no_progress_watchdog["fault_latched"]
            )
            if fault_latched or software_interlock_latched:
                lease_safe_hold_active = False
                lease_safe_hold_target = None
                prior_external_hold_confirmed = False
                mode = "brake"
            if (
                (fault_latched or software_interlock_latched)
                and (enabled or (latest is not None and latest.state != 0))
            ):
                final_disabled = disable_and_verify(transport, logger)
                if not final_disabled:
                    latch_communication_loss("LATCHED_DISABLE_UNCONFIRMED")
                    continue
                enabled = False
                enabled_confirmed = False
                enabled_at = None
                next_tick = time.monotonic()
            explicit_release_packet_received = bool(
                command_receive_events["domain_release_received"]
            )
            if observe_explicit_interlock_release(
                thermal_interlock,
                explicit_release_packet_received,
                cooldown_required=True,
            ):
                print("J6_THERMAL_OPERATOR_RELEASE_OBSERVED", flush=True)
            if observe_explicit_interlock_release(
                no_progress_watchdog,
                explicit_release_packet_received,
                cooldown_required=False,
            ):
                print(
                    "J6_LOAD_LIMIT_NO_PROGRESS_OPERATOR_RELEASE_OBSERVED",
                    flush=True,
                )
            command_lease_fresh = command_lease_is_fresh(
                command, time.monotonic()
            )
            valid_disabled_feedback = bool(
                not fault_latched
                and fresh
                and latest is not None
                and latest.state == 0
                and math.isfinite(latest.position)
                and math.isfinite(latest.velocity)
            )
            if request_interlock_rearm_for_next_cycle(
                thermal_interlock,
                command,
                command_lease_fresh,
                valid_disabled_feedback,
                highest_rejected_active_epoch,
                cooldown_required=True,
            ):
                print(
                    "J6_THERMAL_REARM_PENDING_NEXT_CYCLE "
                    f"activation_epoch={command['activation_epoch']}",
                    flush=True,
                )
            if request_interlock_rearm_for_next_cycle(
                no_progress_watchdog,
                command,
                command_lease_fresh,
                valid_disabled_feedback,
                highest_rejected_active_epoch,
                cooldown_required=False,
            ):
                print(
                    "J6_LOAD_LIMIT_NO_PROGRESS_REARM_PENDING_NEXT_CYCLE "
                    f"activation_epoch={command['activation_epoch']}",
                    flush=True,
                )
            if (
                command_is_v13_quintic_position(command)
                and (fault_latched or software_interlock_latched or mode != "position")
            ):
                cycle_trajectory_status = trajectory_feedback_status(
                    command,
                    time.monotonic_ns(),
                    state_override="INACTIVE",
                )
            if thermal_interlock["fault_latched"]:
                thermal_state = (
                    "WAIT_OPERATOR_CONFIRM"
                    if thermal_interlock["cooldown_ready"]
                    else "COOLDOWN"
                    if current_temperature_c
                    < thermal_limits["rearm_below_c"]
                    else "THERMAL_STOP"
                )
            elif current_temperature_c >= thermal_limits["derating_start_c"]:
                thermal_state = "DERATING"
            elif current_temperature_c >= thermal_limits["normal_below_c"]:
                thermal_state = "WARNING"
            else:
                thermal_state = "NORMAL"
            thermal_status = {
                "thermal_state": thermal_state,
                "thermal_derating_factor": (
                    0.0
                    if thermal_interlock["fault_latched"]
                    else thermal_factor
                ),
                "thermal_raw_temperature_c": current_temperature_c,
                "thermal_window_median_c": temperature_median_c,
                "thermal_slope_c_per_min": temperature_slope_c_per_min,
                "thermal_fault_latched": thermal_interlock["fault_latched"],
                "thermal_cooldown_ready": thermal_interlock["cooldown_ready"],
                "thermal_release_observed": thermal_interlock["release_observed"],
                "thermal_rearm_pending_next_cycle": thermal_interlock[
                    "rearm_pending_next_cycle"
                ],
                "thermal_cooldown_valid_brake_frames": thermal_interlock[
                    "cooldown_frames"
                ],
                "thermal_trip_activation_epoch": thermal_interlock[
                    "trip_activation_epoch"
                ],
                "thermal_minimum_rearm_epoch": thermal_interlock[
                    "minimum_rearm_epoch"
                ],
                "thermal_trip_reason": thermal_interlock["trip_reason"],
                "thermal_config_sha256": thermal_limits["config_sha256"],
            }
            no_progress_status = {
                "no_progress_fault": no_progress_watchdog["fault_latched"],
                "no_progress_release_observed": no_progress_watchdog[
                    "release_observed"
                ],
                "no_progress_rearm_pending_next_cycle": no_progress_watchdog[
                    "rearm_pending_next_cycle"
                ],
                "no_progress_watchdog_qualifying_frames": no_progress_watchdog[
                    "qualifying_frames"
                ],
                "no_progress_observation_valid": no_progress_observation_valid,
                "no_progress_position_error_rad": no_progress_position_error_rad,
                "no_progress_trip_position_error_rad": no_progress_watchdog[
                    "trip_position_error_rad"
                ],
                "load_limit_watchdog_authority": NO_PROGRESS_WATCHDOG_AUTHORITY,
                "no_progress_trip_activation_epoch": no_progress_watchdog[
                    "trip_activation_epoch"
                ],
                "no_progress_minimum_rearm_epoch": no_progress_watchdog[
                    "minimum_rearm_epoch"
                ],
            }
            send_feedback(
                feedback_socket, args.feedback_port, latest, fresh,
                "brake" if fault_latched or software_interlock_latched else
                "hold" if enabled and not enabled_confirmed else mode,
                fault_latched,
                lease_safe_hold_active,
                command_rejection_state,
                cycle_trajectory_status,
                thermal_status,
                no_progress_status,
                (
                    None
                    if latest_at is None
                    else int(latest_at * 1_000_000_000)
                ),
            )
            # Use the lease decision made at the cycle boundary where this
            # external target was selected.  Re-reading the clock here can
            # turn a 0.499 s fresh command into a 0.501 s stale command after
            # it was already sent, erasing the proof needed for next-cycle
            # lease-safe HOLD and causing an unintended DISABLE.
            external_active_confirmed_this_cycle = bool(
                not lease_safe_hold_active
                and command_lease_fresh
                and mode in {"hold", "position"}
                and enabled
                and enabled_confirmed
                and fresh
                and not fault_latched
            )
            prior_external_hold_confirmed = next_prior_external_hold_confirmation(
                prior_external_hold_confirmed,
                lease_safe_hold_active=lease_safe_hold_active,
                enabled=enabled,
                fault_latched=fault_latched,
                external_active_confirmed_this_cycle=(
                    external_active_confirmed_this_cycle
                ),
            )
            previous_mode = mode
            cycles += 1
            previous_cycle_enabled = enabled
            next_tick += PERIOD
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            else:
                # Never burst through obsolete deadlines; the next adjacent
                # loop-start interval remains a truthful control-rate sample.
                next_tick = time.monotonic()
        final_disabled = disable_and_verify(transport, logger)
        print(f"J6_GUI_COMPLETED_CYCLES={cycles}\nJ6_FINAL_DISABLED={'PASS' if final_disabled else 'FAIL'}")
        return 0 if final_disabled else 2
    finally:
        if transport is not None:
            try:
                final_disabled = disable_and_verify(transport, logger) or final_disabled
            except Exception:
                pass
        if transport is not None:
            transport.close()
        else:
            logger.close()
        command_socket.close()
        feedback_socket.close()
        fcntl.flock(lock_fd, fcntl.LOCK_UN)
        os.close(lock_fd)


def main() -> int:
    args = parse_args()
    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    try:
        return run(args)
    except Exception as exc:
        print(f"J6_GUI_CONTROLLER=BLOCKED\n原因={exc}", flush=True)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

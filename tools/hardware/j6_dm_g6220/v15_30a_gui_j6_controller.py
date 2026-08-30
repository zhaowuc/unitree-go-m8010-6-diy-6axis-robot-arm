#!/usr/bin/env python3
"""J6 独立 POS_VEL GUI worker：本机UDP命令、100 Hz、健康租约安全保持。"""

from __future__ import annotations

import argparse
import ctypes
import fcntl
import json
import math
import os
import signal
import socket
import statistics
import time
from pathlib import Path

from dm_g6220_posvel_transport import DmG6220PosVelTransport
from j6_raw_can_diagnostic import (
    RawCanLogger,
    exact_usb_identity,
    read_parameter,
    refresh_request,
    strict_decode,
)
from v15_30a_profile import update_posvel_speed_limit


GATE = "V15_30A_GUI_J6_CONTROL_AUTHORIZED=YES"
MOTOR_ID = 1
CTRL_MODE_POS_VEL = 2
PERIOD = 0.01
LEASE_S = 0.5
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
# The 16-bit G6220 position feedback is about 0.02186 deg/LSB.  Keep the
# tolerance above three LSBs, but below the commissioned 0.20 deg small step so
# a stationary motor cannot be mistaken for an arrived one.
ARRIVAL_TOLERANCE = math.radians(0.08)
TARGET_TIMEOUT_S = 90.0
FAULT_STATES = {8, 9, 0xA, 0xB, 0xC, 0xD, 0xE}
STOP = False


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
    return parser.parse_args()


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
    }:
        raise ValueError("命令格式不匹配")
    if value.get("mode") not in {"brake", "drag", "hold", "position"}:
        raise ValueError("模式不允许")
    if schema != "go-m8010-gui-command/1.2" and value["mode"] != "brake":
        raise ValueError("旧版协议仅允许制动")
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
    value["received_at"] = received_monotonic_ns / 1_000_000_000.0
    return value


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
        and 0 <= latest.mos_temp < 60
        and 0 <= latest.coil_temp < 60
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
        or not 0 <= latest.mos_temp < 60
        or not 0 <= latest.coil_temp < 60
    ):
        return False
    logical = -(latest.position - reference)
    return bool(
        FEEDBACK_HARD_LOWER <= logical <= FEEDBACK_HARD_UPPER
        and abs(command["targets_rad"][5] - logical)
        <= HOLD_ENTRY_TARGET_LIMIT + 1e-12
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
        "命令接收时钟无效",
        "命令来源实例无效",
        "命令来源时钟无效",
        "命令来源已过期或来自未来",
        "命令来源序列无效",
        "命令来源序列或时钟发生回放",
        "当前命令来源租约仍有效",
        "目标必须是六个有限数",
        "J6目标超出模型机械限位",
        "关节激活掩码必须是六个布尔值",
        "关节移动掩码必须是六个布尔值",
        "移动关节必须同时激活",
        "制动命令不得携带激活关节",
        "激活纪元必须是非负整数",
        "主动命令的激活纪元必须大于零",
        "速度或加速度无效",
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
        "active_source_instance_id": None,
        "active_source_last_received_monotonic_ns": None,
    }


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


def receive_latest(
    sock: socket.socket,
    current: dict | None,
    minimum_activation_epoch: int,
    last_seen_activation_epoch: int,
    rejection_state: dict | None = None,
    source_replay_state: dict | None = None,
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
            current = candidate
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
) -> None:
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
    payload = {
        "schema": "go-m8010-motor-feedback/1.0",
        "source_monotonic_ns": time.monotonic_ns(),
        "samples": [{
            "motor": "J6",
            "position_rad": position,
            "velocity_rad_s": velocity,
            "temperature_c": max(mos, coil),
            "merror": 0 if communication_ok and state not in FAULT_STATES else state,
            "communication_ok": communication_ok and state not in FAULT_STATES,
        }],
        "controller_mode": mode,
        "controller_mode_by_motor": {"J6": mode},
        "domain_fault": fault_latched,
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
    if not args.execute:
        print(
            "DRY_RUN=YES\nCAN_OPENED=NO\nDEFAULT_STATE=DISABLED\n"
            "CONTROL_LOOP_HZ=100\n"
            "COMMAND_TARGET_LIMIT_DEG=[-180,180]\n"
            "FEEDBACK_ENVELOPE_TOLERANCE_DEG=0.5\n"
            "POSITION_ARRIVAL_TIMEOUT_SECONDS=90"
        )
        return 0
    if args.confirm != GATE:
        raise RuntimeError("缺少J6主动控制授权门")
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
        while not STOP:
            cycle_started_at = time.monotonic()
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
            command, minimum_activation_epoch, last_seen_activation_epoch = receive_latest(
                command_socket,
                command,
                minimum_activation_epoch,
                last_seen_activation_epoch,
                command_rejection_state,
                command_source_replay_state,
            )
            now = time.monotonic()
            command_lease_fresh = command_lease_is_fresh(command, now)
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
                fault_latched = True
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
                    fault_latched = True
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
                captured_target = capture_lease_safe_hold_target(
                    lease_capture_source,
                    prior_external_hold_confirmed,
                    last_accepted_hold_target,
                    latest,
                    latest_at,
                    reference,
                    fault_latched,
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
            if fault_latched:
                mode = "brake"
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
                fault_latched = True
            # A hard health failure is terminal for active output in this
            # cycle.  Apply it after every rejected-target fallback so no HOLD
            # rewrite can emit one additional POS_VEL frame.
            mode = fault_dominant_mode(mode, fault_latched)
            active = mode in {"hold", "position"}
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
                    time.sleep(PERIOD)
                    decoded_events, fatal_event = drain_feedback_batch(logger)
                    if fatal_event:
                        fault_latched = True
                    for refreshed, refreshed_at in decoded_events:
                        latest, latest_at = refreshed, refreshed_at
                command, minimum_activation_epoch, last_seen_activation_epoch = receive_latest(
                    command_socket,
                    command,
                    minimum_activation_epoch,
                    last_seen_activation_epoch,
                    command_rejection_state,
                    command_source_replay_state,
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
                    fault_latched = True
                    mode = "brake"
                    active = False
                else:
                    mode = pre_enable_mode
                    q_command = -(latest.position - reference)
                    dq_command = 0.0
                    transport.send_pos_vel_command(latest.position, 0.0, "GUI_PRELOAD_CURRENT_DISABLED")
                    command, minimum_activation_epoch, last_seen_activation_epoch = receive_latest(
                        command_socket,
                        command,
                        minimum_activation_epoch,
                        last_seen_activation_epoch,
                        command_rejection_state,
                        command_source_replay_state,
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
                        transport.send_pos_vel_command(latest.position, 0.0, "GUI_HOLD_IMMEDIATE")
                        enabled = True
                        enabled_confirmed = False
                        enabled_at = time.monotonic()
                        next_tick = time.monotonic()
            if not active and enabled:
                final_disabled = disable_and_verify(transport, logger)
                if not final_disabled:
                    raise RuntimeError("J6未能确认DISABLED终态")
                enabled = False
                enabled_confirmed = False
                enabled_at = None
                next_tick = time.monotonic()

            if enabled and not enabled_confirmed:
                transport.send_pos_vel_command(
                    reference - q_command, 0.0, "GUI_ENABLE_CONFIRM_HOLD"
                )
            elif mode == "position" and command is not None and enabled:
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
                dq_command = update_posvel_speed_limit(
                    dq_command,
                    actual_logical,
                    requested_target,
                    command["maximum_velocity_rad_s"],
                    command["maximum_acceleration_rad_s2"],
                    restore_limit,
                    PERIOD,
                )
                q_command = requested_target
                protocol_position = reference - requested_target
                protocol_velocity = dq_command
                transport.send_pos_vel_command(protocol_position, protocol_velocity, "GUI_POSITION_REFRESH")
            elif mode == "hold" and enabled:
                fixed_hold_target = (
                    lease_safe_hold_target
                    if lease_safe_hold_active
                    else last_accepted_hold_target
                )
                if fixed_hold_target is None:
                    raise RuntimeError("J6 HOLD目标尚未建立")
                hold_velocity_limit = fixed_hold_velocity_limit()
                transport.send_pos_vel_command(
                    reference - fixed_hold_target,
                    hold_velocity_limit,
                    "GUI_LEASE_SAFE_HOLD_REFRESH"
                    if lease_safe_hold_active
                    else "GUI_HOLD_REFRESH",
                )
            else:
                logger.send(0x7FF, refresh_request(), "GUI_DISABLED_REFRESH")

            decoded_events, fatal_event = drain_feedback_batch(logger)
            if fatal_event:
                fault_latched = True
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
                if decoded.mos_temp >= 60 or decoded.coil_temp >= 60 or decoded.state in FAULT_STATES:
                    fault_latched = True
                if (enabled and decoded.state != 1 and enabled_at is not None and
                        time.monotonic() - enabled_at > 0.2):
                    fault_latched = True
                if (enabled and decoded.state == 1 and enabled_at is not None and
                        latest_at is not None and latest_at >= enabled_at and
                        not enabled_confirmed):
                    enabled_confirmed = True
                if not enabled and decoded.state != 0:
                    fault_latched = True
            fresh = latest is not None and latest_at is not None and time.monotonic() - latest_at <= FEEDBACK_MAX_AGE_S
            if enabled and not fresh:
                fault_latched = True
            if (
                enabled
                and mode == "position"
                and position_started_at is not None
                and latest is not None
                and last_position_target is not None
            ):
                position_error = abs(
                    (-(latest.position - reference)) - last_position_target
                )
                if position_error <= ARRIVAL_TOLERANCE:
                    # A later external displacement must be restored to the
                    # immutable target, not reclassified as an arrival timeout.
                    if position_arrival_overdue:
                        print("J6_POSITION_ARRIVAL_RECOVERED", flush=True)
                    position_arrival_overdue = False
                    position_started_at = None
                elif (
                    time.monotonic() - position_started_at >= TARGET_TIMEOUT_S
                    and not position_arrival_overdue
                ):
                    print(
                        "J6_POSITION_ARRIVAL_OVERDUE "
                        f"error_rad={position_error}",
                        flush=True,
                    )
                    position_arrival_overdue = True
            if fault_latched:
                lease_safe_hold_active = False
                lease_safe_hold_target = None
            if fault_latched and (enabled or (latest is not None and latest.state != 0)):
                final_disabled = disable_and_verify(transport, logger)
                if not final_disabled:
                    raise RuntimeError("J6故障后未能确认DISABLED终态")
                enabled = False
                enabled_confirmed = False
                enabled_at = None
                next_tick = time.monotonic()
            send_feedback(
                feedback_socket, args.feedback_port, latest, fresh,
                "brake" if fault_latched else
                "hold" if enabled and not enabled_confirmed else mode,
                fault_latched,
                lease_safe_hold_active,
                command_rejection_state,
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

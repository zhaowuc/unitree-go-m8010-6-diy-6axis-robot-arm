#!/usr/bin/env python3
"""J6 独立 POS_VEL GUI worker：本机UDP命令、100 Hz、失联FD。"""

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
from v15_30a_profile import update_profile


GATE = "V15_30A_GUI_J6_CONTROL_AUTHORIZED=YES"
MOTOR_ID = 1
CTRL_MODE_POS_VEL = 2
PERIOD = 0.01
LEASE_S = 0.5
FEEDBACK_MAX_AGE_S = 0.15
TARGET_LIMIT = math.radians(10.0)
FEEDBACK_LIMIT = math.radians(12.0)
VMAX_LIMIT = math.radians(5.0)
AMAX_LIMIT = math.radians(20.0)
ARRIVAL_TOLERANCE = math.radians(0.5)
TARGET_TIMEOUT_S = 15.0
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
    return parser.parse_args()


def parse_command(payload: bytes) -> dict:
    value = json.loads(payload)
    schema = value.get("schema")
    if schema not in {"go-m8010-gui-command/1.0", "go-m8010-gui-command/1.1"}:
        raise ValueError("命令格式不匹配")
    if value.get("mode") not in {"brake", "drag", "hold", "position"}:
        raise ValueError("模式不允许")
    if schema == "go-m8010-gui-command/1.0" and value["mode"] != "brake":
        raise ValueError("旧版协议仅允许制动")
    targets = [float(item) for item in value["targets_rad"]]
    if len(targets) != 6 or not all(math.isfinite(item) and abs(item) <= TARGET_LIMIT for item in targets):
        raise ValueError("目标超出范围")
    value["targets_rad"] = targets
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
    vmax = abs(float(value["maximum_velocity_rad_s"]))
    amax = abs(float(value["maximum_acceleration_rad_s2"]))
    if not math.isfinite(vmax) or not math.isfinite(amax) or vmax <= 0.0 or amax <= 0.0:
        raise ValueError("速度或加速度无效")
    value["maximum_velocity_rad_s"] = min(vmax, VMAX_LIMIT)
    value["maximum_acceleration_rad_s2"] = min(amax, AMAX_LIMIT)
    value["received_at"] = time.monotonic()
    return value


def command_requests_j6_active(command: dict | None) -> bool:
    return bool(
        command is not None
        and command["mode"] in {"hold", "position"}
        and command["active_joint_mask"][5]
    )


def effective_command_mode(
    command: dict | None, minimum_activation_epoch: int, now: float | None = None
) -> str:
    checked_at = time.monotonic() if now is None else now
    if command is None or checked_at - command["received_at"] > LEASE_S:
        return "brake"
    if not command_requests_j6_active(command):
        return "brake"
    if command["activation_epoch"] < minimum_activation_epoch:
        return "brake"
    return command["mode"]


def observe_valid_command_epoch(
    command: dict, minimum_activation_epoch: int, last_seen_activation_epoch: int
) -> tuple[int, int]:
    last_seen_activation_epoch = max(
        last_seen_activation_epoch, command["activation_epoch"]
    )
    if command["mode"] == "brake":
        minimum_activation_epoch = max(
            minimum_activation_epoch, last_seen_activation_epoch + 1
        )
    return minimum_activation_epoch, last_seen_activation_epoch


def command_epoch_is_acceptable(
    command: dict, last_seen_activation_epoch: int
) -> bool:
    return bool(
        command["mode"] == "brake"
        or command["activation_epoch"] >= last_seen_activation_epoch
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


def receive_latest(
    sock: socket.socket,
    current: dict | None,
    minimum_activation_epoch: int,
    last_seen_activation_epoch: int,
) -> tuple[dict | None, int, int]:
    while True:
        try:
            payload = sock.recv(8192)
        except BlockingIOError:
            return current, minimum_activation_epoch, last_seen_activation_epoch
        try:
            minimum_activation_epoch = minimum_epoch_after_interarrival_lease(
                current, minimum_activation_epoch, time.monotonic()
            )
            candidate = parse_command(payload)
            if not command_epoch_is_acceptable(
                candidate, last_seen_activation_epoch
            ):
                raise ValueError("主动命令激活纪元发生回放")
            minimum_activation_epoch, last_seen_activation_epoch = (
                observe_valid_command_epoch(
                    candidate,
                    minimum_activation_epoch,
                    last_seen_activation_epoch,
                )
            )
            current = candidate
        except Exception as exc:
            print(f"拒绝GUI命令：{exc}", flush=True)
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
    mode: str, fault_latched: bool
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
        print("DRY_RUN=YES\nCAN_OPENED=NO\nDEFAULT_STATE=DISABLED\nCONTROL_LOOP_HZ=100")
        return 0
    if args.confirm != GATE:
        raise RuntimeError("缺少J6主动控制授权门")
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
        reference = statistics.median(capture)
        if (math.degrees(max(capture) - min(capture)) > 0.20 or
                math.degrees(abs(statistics.median(capture[-10:]) - reference)) > 0.10):
            raise RuntimeError("J6会话参考不静止")

        command = None
        enabled = False
        enabled_confirmed = False
        fault_latched = False
        q_command = 0.0
        dq_command = 0.0
        previous_mode = "brake"
        enabled_at = None
        position_started_at = None
        last_position_target = None
        minimum_activation_epoch = 0
        last_seen_activation_epoch = 0
        next_tick = time.monotonic()
        cycles = 0
        while not STOP:
            command, minimum_activation_epoch, last_seen_activation_epoch = receive_latest(
                command_socket,
                command,
                minimum_activation_epoch,
                last_seen_activation_epoch,
            )
            now = time.monotonic()
            if (
                command_requests_j6_active(command)
                and command is not None
                and now - command["received_at"] > LEASE_S
            ):
                minimum_activation_epoch = max(
                    minimum_activation_epoch, command["activation_epoch"] + 1
                )
            mode = effective_command_mode(command, minimum_activation_epoch, now)
            if fault_latched:
                mode = "brake"
            active = mode in {"hold", "position"}
            if mode == "position":
                requested_target = command["targets_rad"][5]
                if (previous_mode != "position" or last_position_target is None or
                        abs(requested_target - last_position_target) > 1e-9):
                    position_started_at = time.monotonic()
                    last_position_target = requested_target
            else:
                position_started_at = None
                last_position_target = None
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
                    )
                    if effective_command_mode(command, minimum_activation_epoch) not in {"hold", "position"}:
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
                )
                pre_enable_mode = effective_command_mode(command, minimum_activation_epoch)
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
                    )
                    pre_enable_mode = effective_command_mode(command, minimum_activation_epoch)
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
                q_command, dq_command = update_profile(
                    q_command, dq_command, command["targets_rad"][5],
                    command["maximum_velocity_rad_s"],
                    command["maximum_acceleration_rad_s2"],
                    PERIOD,
                )
                protocol_position = reference - q_command
                protocol_velocity = max(0.0, abs(dq_command))
                transport.send_pos_vel_command(protocol_position, protocol_velocity, "GUI_POSITION_REFRESH")
            elif mode == "hold" and enabled:
                transport.send_pos_vel_command(
                    reference - q_command, 0.0, "GUI_HOLD_REFRESH"
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
                if abs(logical) > FEEDBACK_LIMIT or abs(decoded.velocity) > 0.7:
                    fault_latched = True
                if decoded.mos_temp >= 60 or decoded.coil_temp >= 60 or decoded.state in FAULT_STATES:
                    fault_latched = True
                if (enabled and decoded.state != 1 and enabled_at is not None and
                        time.monotonic() - enabled_at > 0.2):
                    fault_latched = True
                if (enabled and decoded.state == 1 and enabled_at is not None and
                        latest_at is not None and latest_at >= enabled_at and
                        not enabled_confirmed):
                    enabled_confirmed = True
                    position_started_at = time.monotonic() if mode == "position" else None
                if not enabled and decoded.state != 0:
                    fault_latched = True
            fresh = latest is not None and latest_at is not None and time.monotonic() - latest_at <= FEEDBACK_MAX_AGE_S
            if enabled and not fresh:
                fault_latched = True
            if (enabled and mode == "position" and position_started_at is not None and
                    time.monotonic() - position_started_at >= TARGET_TIMEOUT_S and
                    latest is not None and last_position_target is not None and
                    abs((-(latest.position - reference)) - last_position_target) > ARRIVAL_TOLERANCE):
                fault_latched = True
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
                fault_latched
            )
            previous_mode = mode
            cycles += 1
            next_tick += PERIOD
            delay = next_tick - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            elif delay < -0.02:
                fault_latched = True
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

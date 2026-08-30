#!/usr/bin/env python3
"""Move all six logical joints to persistent software zero, then hold."""

from __future__ import annotations

import argparse
import json
import math
import socket
import time

import mujoco
import numpy as np
import rclpy

from v15_30a_j2_auto_recovery import (
    KD,
    KP,
    PORTS,
    StateReader,
    gravity_feedforward,
)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--confirm", required=True)
    parser.add_argument("--speed-deg-s", type=float, default=3.0)
    parser.add_argument("--hold-current-only", action="store_true")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.confirm != "WHOLE_ARM_ZERO_AND_RIGID_HOLD=YES":
        raise SystemExit("confirmation gate missing")

    rclpy.init()
    node = StateReader()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    model = mujoco.MjModel.from_xml_path(args.model)
    model.opt.gravity[:] = [0.0, 0.0, -9.81]
    data = mujoco.MjData(model)
    qpos_adr, dof_adr = [], []
    for name in ("J1", "J2", "J3", "J4", "J5", "J6"):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qpos_adr.append(int(model.jnt_qposadr[jid]))
        dof_adr.append(int(model.jnt_dofadr[jid]))
    qpos_adr = np.asarray(qpos_adr, dtype=int)
    dof_adr = np.asarray(dof_adr, dtype=int)
    epoch = max(1, time.monotonic_ns() & ((1 << 63) - 1))
    sequence = 0
    go_motor_names = ("J1", "J2A", "J2B", "J3", "J4", "J5")

    def go_domains_healthy(state: dict) -> bool:
        motors = state.get("per_motor", {})
        faults = state.get("controller_fault_by_motor", {})
        return all(
            bool(motors.get(name, {}).get("communication_ok"))
            and not bool(faults.get(name, False))
            for name in go_motor_names
        )

    def send(mode: str, targets, feedforward, active: bool = True) -> None:
        nonlocal sequence
        sequence += 1
        payload = json.dumps({
            "schema": "go-m8010-gui-command/1.1",
            "sequence": sequence,
            "mode": mode,
            "targets_rad": list(targets),
            "active_joint_mask": (
                [active, active, active, active, active, False]
                if args.hold_current_only else [active] * 6
            ),
            "activation_epoch": epoch,
            "maximum_velocity_rad_s": math.radians(args.speed_deg_s),
            "maximum_acceleration_rad_s2": math.radians(8.0),
            "kp": KP,
            "kd": KD,
            "feedforward_nm": list(feedforward),
            "recovery": active,
        }, separators=(",", ":")).encode()
        command_ports = PORTS[:-1] if args.hold_current_only else PORTS
        for port in command_ports:
            sock.sendto(payload, ("127.0.0.1", port))

    def brake() -> None:
        for _ in range(30):
            send("brake", [0.0] * 6, [0.0] * 6, active=False)
            time.sleep(0.01)

    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            modes = node.state.get("controller_mode_by_motor", {}) if node.state else {}
            go_modes_ready = all(
                modes.get(name) in ({"brake", "hold"} if args.hold_current_only else {"brake"})
                for name in ("J1", "J2A", "J2B", "J3", "J4", "J5")
            )
            baseline_healthy = (
                go_domains_healthy(node.state)
                if node.state and args.hold_current_only
                else bool(node.state and node.state.get("healthy"))
            )
            if node.state and baseline_healthy and go_modes_ready:
                break
        else:
            raise RuntimeError("HEALTHY_BRAKE_BASELINE_MISSING")

        initial = np.asarray(node.state["position_rad"], dtype=float)
        targets = np.zeros(6, dtype=float)
        stage = "gravity_ramp"
        stage_started = time.monotonic()
        stable_since = None
        last_print = 0.0

        while True:
            rclpy.spin_once(node, timeout_sec=0.002)
            now = time.monotonic()
            if node.state is None or now - node.received_at > 0.20:
                raise RuntimeError("STATE_STALE")
            state = node.state
            runtime_healthy = (
                go_domains_healthy(state)
                if args.hold_current_only else bool(state.get("healthy"))
            )
            if not runtime_healthy:
                raise RuntimeError("HARDWARE_NOT_HEALTHY")
            faults = state.get("controller_fault_by_motor", {})
            fault_names = go_motor_names if args.hold_current_only else tuple(faults)
            if any(bool(faults.get(name, False)) for name in fault_names):
                raise RuntimeError(f"CONTROLLER_FAULT:{faults}")
            actual = np.asarray(state["position_rad"], dtype=float)
            sync = abs(float(state["j2_e_sync_rad"]))
            if sync >= math.radians(0.5):
                raise RuntimeError(f"J2_SYNC_LIMIT:{math.degrees(sync):.3f}")

            feedforward = gravity_feedforward(
                model, data, qpos_adr, dof_adr, actual
            )
            if args.hold_current_only and abs(feedforward[1]) > 1e-6:
                feedforward[1] = float(np.clip(
                    feedforward[1] + math.copysign(0.20, feedforward[1]),
                    -1.15,
                    1.15,
                ))
            if stage == "gravity_ramp":
                ramp_seconds = 0.01 if args.hold_current_only else 2.0
                scale = min(1.0, (now - stage_started) / ramp_seconds)
                send("hold", initial, np.asarray(feedforward) * scale)
                if scale >= 1.0:
                    stage = "hold_current" if args.hold_current_only else "run"
                    stage_started = now
            elif stage == "run":
                j2_error = -actual[1]
                if abs(j2_error) > math.radians(0.20):
                    feedforward[1] = float(np.clip(
                        feedforward[1] + math.copysign(0.25, j2_error),
                        -1.15,
                        1.15,
                    ))
                send("position", targets, feedforward)
                if np.max(np.abs(np.degrees(actual))) <= 0.25:
                    stable_since = now if stable_since is None else stable_since
                    if now - stable_since >= 1.0:
                        stage = "hold_zero"
                else:
                    stable_since = None
                if now - stage_started > 30.0:
                    raise RuntimeError(
                        f"ZERO_TIMEOUT:{np.degrees(actual).tolist()}"
                    )
            else:
                send("hold", initial if args.hold_current_only else targets, feedforward)

            if now - last_print >= 0.5:
                print(json.dumps({
                    "stage": stage,
                    "position_deg": np.degrees(actual).tolist(),
                    "j2_sync_deg": math.degrees(sync),
                    "j2_ff_each_nm": feedforward[1],
                    "j3_ff_nm": feedforward[2],
                }, ensure_ascii=False), flush=True)
                last_print = now
            time.sleep(0.005)
    except BaseException as exc:
        print(f"FAIL_CLOSED:{type(exc).__name__}:{exc}", flush=True)
        brake()
        return 2
    finally:
        sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

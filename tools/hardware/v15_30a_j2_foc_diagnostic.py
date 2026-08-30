#!/usr/bin/env python3
"""Short, fail-closed J2 FOC/feedback diagnostic at the captured pose."""

from __future__ import annotations

import argparse
import json
import math
import socket
import time

import numpy as np
import rclpy

from v15_30a_j2_auto_recovery import KD, KP, StateReader


J2_PORT = 15312
ACTIVE_J2_ONLY = [False, True, False, False, False, False]


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--confirm", required=True)
    parser.add_argument("--step-seconds", type=float, default=0.45)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.confirm != "J2_FOC_DIAGNOSTIC=YES":
        raise SystemExit("confirmation gate missing")

    rclpy.init()
    node = StateReader()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    epoch = max(1, time.monotonic_ns() & ((1 << 63) - 1))
    sequence = 0

    def send(mode: str, targets, torque_each_nm: float, active: bool) -> None:
        nonlocal sequence
        sequence += 1
        feedforward = [0.0] * 6
        feedforward[1] = torque_each_nm
        payload = json.dumps({
            "schema": "go-m8010-gui-command/1.1",
            "sequence": sequence,
            "mode": mode,
            "targets_rad": list(targets),
            "active_joint_mask": ACTIVE_J2_ONLY if active else [False] * 6,
            "activation_epoch": epoch,
            "maximum_velocity_rad_s": math.radians(0.5),
            "maximum_acceleration_rad_s2": math.radians(1.0),
            "kp": KP,
            "kd": KD,
            "feedforward_nm": feedforward,
            "recovery": bool(active),
        }, separators=(",", ":")).encode()
        sock.sendto(payload, ("127.0.0.1", J2_PORT))

    def brake(targets) -> None:
        for _ in range(30):
            send("brake", targets, 0.0, False)
            time.sleep(0.01)

    initial = np.zeros(6)
    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            state = node.state
            if state is None:
                continue
            modes = state.get("controller_mode_by_motor", {})
            faults = state.get("controller_fault_by_motor", {})
            if (modes.get("J2A") == "brake" and modes.get("J2B") == "brake"
                    and not faults.get("J2A", True) and not faults.get("J2B", True)):
                break
        else:
            raise RuntimeError("J2_HEALTHY_BRAKE_BASELINE_MISSING")

        initial = np.asarray(state["position_rad"], dtype=float)
        initial_j2 = float(initial[1])
        levels = (0.0, 0.10, 0.20, 0.30, 0.40, 0.50, 0.60, 0.70)
        for level in levels:
            started = time.monotonic()
            while time.monotonic() - started < args.step_seconds:
                send("hold", initial, level, True)
                rclpy.spin_once(node, timeout_sec=0.01)
                now = time.monotonic()
                if node.state is None or now - node.received_at > 0.20:
                    raise RuntimeError("STATE_STALE")
                state = node.state
                faults = state.get("controller_fault_by_motor", {})
                if faults.get("J2A", False) or faults.get("J2B", False):
                    raise RuntimeError(f"J2_CONTROLLER_FAULT:{faults}")
                sync = abs(float(state.get("j2_e_sync_rad", math.inf)))
                if sync >= math.radians(0.5):
                    raise RuntimeError(f"J2_SYNC_LIMIT:{math.degrees(sync):.4f}")
                drift = abs(float(state["position_rad"][1]) - initial_j2)
                if drift >= math.radians(0.75):
                    raise RuntimeError(f"J2_DRIFT_LIMIT:{math.degrees(drift):.4f}")
            modes = state.get("controller_mode_by_motor", {})
            print(json.dumps({
                "torque_each_nm": level,
                "j2_deg": math.degrees(float(state["position_rad"][1])),
                "drift_deg": math.degrees(float(state["position_rad"][1]) - initial_j2),
                "sync_deg": math.degrees(float(state["j2_e_sync_rad"])),
                "J2A_mode": modes.get("J2A"),
                "J2B_mode": modes.get("J2B"),
            }, ensure_ascii=False), flush=True)

        brake(initial)
        print("J2_FOC_DIAGNOSTIC_COMPLETE", flush=True)
        return 0
    except Exception as exc:
        print(f"J2_FOC_DIAGNOSTIC_ABORT:{exc}", flush=True)
        brake(initial)
        return 2
    finally:
        node.destroy_node()
        rclpy.shutdown()
        sock.close()


if __name__ == "__main__":
    raise SystemExit(main())

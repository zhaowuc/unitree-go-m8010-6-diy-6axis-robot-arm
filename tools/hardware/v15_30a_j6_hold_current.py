#!/usr/bin/env python3
"""Keep J6 enabled at its captured current software angle."""

import json
import socket
import time

import rclpy

from v15_30a_j2_auto_recovery import StateReader


def main() -> int:
    rclpy.init()
    node = StateReader()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    epoch = max(1, time.monotonic_ns() & ((1 << 63) - 1))
    sequence = 0
    try:
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.state and node.state.get("healthy"):
                break
        else:
            raise RuntimeError("HEALTHY_STATE_MISSING")
        target = float(node.state["position_rad"][5])
        while True:
            rclpy.spin_once(node, timeout_sec=0.002)
            if node.state is None or time.monotonic() - node.received_at > 0.20:
                raise RuntimeError("STATE_STALE")
            fault = node.state.get("controller_fault_by_motor", {}).get("J6", False)
            if fault:
                raise RuntimeError("J6_CONTROLLER_FAULT")
            sequence += 1
            payload = json.dumps({
                "schema": "go-m8010-gui-command/1.1",
                "sequence": sequence,
                "mode": "hold",
                "targets_rad": [0.0, 0.0, 0.0, 0.0, 0.0, target],
                "active_joint_mask": [False, False, False, False, False, True],
                "activation_epoch": epoch,
                "maximum_velocity_rad_s": 0.05,
                "maximum_acceleration_rad_s2": 0.10,
                "kp": [0.0] * 6,
                "kd": [0.0] * 6,
                "feedforward_nm": [0.0] * 6,
                "recovery": True,
            }, separators=(",", ":")).encode()
            sock.sendto(payload, ("127.0.0.1", 15311))
            time.sleep(0.005)
    except BaseException as exc:
        print(f"J6_HOLD_STOPPED:{type(exc).__name__}:{exc}", flush=True)
        return 2
    finally:
        sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

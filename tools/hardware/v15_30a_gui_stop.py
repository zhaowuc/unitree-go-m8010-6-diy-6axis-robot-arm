#!/usr/bin/env python3
"""不依赖 ROS2 的 V15.30A GUI 紧急制动广播器。"""

from __future__ import annotations

import argparse
import json
import math
import socket
import time


PORTS = (15310, 15312, 15313, 15311)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repeat", type=int, default=50)
    parser.add_argument("--interval", type=float, default=0.02)
    args = parser.parse_args()
    if not 1 <= args.repeat <= 500 or not 0.001 <= args.interval <= 1.0:
        raise SystemExit("重复次数或间隔超出安全范围")
    document = {
        "schema": "go-m8010-gui-command/1.1",
        "sequence": 0,
        "mode": "brake",
        "targets_rad": [0.0] * 6,
        "active_joint_mask": [False] * 6,
        "activation_epoch": (1 << 63) - 1,
        "maximum_velocity_rad_s": math.radians(5.0),
        "maximum_acceleration_rad_s2": math.radians(20.0),
        "kp": [0.0] * 6,
        "kd": [0.0] * 6,
    }
    legacy_document = dict(document)
    legacy_document["schema"] = "go-m8010-gui-command/1.0"
    legacy_document.pop("active_joint_mask")
    legacy_document.pop("activation_epoch")
    payloads = [
        json.dumps(legacy_document, separators=(",", ":")).encode("utf-8"),
        json.dumps(document, separators=(",", ":")).encode("utf-8"),
    ]
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as output:
        for _ in range(args.repeat):
            for port in PORTS:
                for payload in payloads:
                    output.sendto(payload, ("127.0.0.1", port))
            time.sleep(args.interval)
    print(f"已向四个物理故障域广播兼容制动命令 {args.repeat} 轮")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

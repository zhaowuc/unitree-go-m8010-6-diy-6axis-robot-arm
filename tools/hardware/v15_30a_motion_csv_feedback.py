#!/usr/bin/env python3
"""Publish live, measured motion-runner CSV feedback to the V15.30A UDP state input."""

from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import time
from pathlib import Path


FORMATS = {
    "J1": {
        "position": "q_raw",
        "velocity": "dq_sdk",
        "temperature": "temp_c",
        "merror": "merror",
        "valid": "feedback_valid",
    },
    "J4": {
        "position": "q_feedback_motor_rad",
        "velocity": "dq_feedback_motor_rad_s",
        "temperature": "temperature_c",
        "merror": "merror",
        "valid": "correct",
    },
    "J5": {
        "position": "q_feedback_motor_rad",
        "velocity": "dq_feedback_motor_rad_s",
        "temperature": "temperature_c",
        "merror": "merror",
        "valid": "correct",
    },
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--joint", choices=tuple(FORMATS), required=True)
    parser.add_argument("--csv", type=Path, required=True)
    parser.add_argument("--udp-port", type=int, default=15300)
    parser.add_argument("--wait-timeout", type=float, default=30.0)
    return parser.parse_args()


def finite(row: dict[str, str], key: str) -> float:
    value = float(row[key])
    if not math.isfinite(value):
        raise ValueError(f"non-finite {key}")
    return value


def main() -> int:
    args = parse_args()
    deadline = time.monotonic() + args.wait_timeout
    while not args.csv.exists():
        if time.monotonic() >= deadline:
            raise TimeoutError(f"CSV did not appear: {args.csv}")
        time.sleep(0.01)

    fields = FORMATS[args.joint]
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    published = 0
    with args.csv.open("r", encoding="utf-8", newline="") as stream:
        header_line = stream.readline()
        if not header_line:
            raise RuntimeError("CSV header is missing")
        fieldnames = next(csv.reader([header_line]))
        while True:
            line = stream.readline()
            if not line:
                time.sleep(0.005)
                continue
            values = next(csv.reader([line]))
            if len(values) != len(fieldnames):
                continue
            row = dict(zip(fieldnames, values))
            try:
                valid = row[fields["valid"]] in {"1", "true", "True"}
                sample = {
                    "motor": args.joint,
                    "position_rad": finite(row, fields["position"]),
                    "velocity_rad_s": finite(row, fields["velocity"]),
                    "temperature_c": finite(row, fields["temperature"]),
                    "merror": int(row[fields["merror"]]),
                    "communication_ok": valid,
                }
            except (KeyError, ValueError):
                continue
            payload = {
                "schema": "go-m8010-motor-feedback/1.0",
                "source_monotonic_ns": time.monotonic_ns(),
                "samples": [sample],
            }
            sock.sendto(json.dumps(payload, separators=(",", ":")).encode(),
                        ("127.0.0.1", args.udp_port))
            published += 1
            if published % 100 == 0:
                print(f"MIRROR_FEEDBACK_ROWS_PUBLISHED={published}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())

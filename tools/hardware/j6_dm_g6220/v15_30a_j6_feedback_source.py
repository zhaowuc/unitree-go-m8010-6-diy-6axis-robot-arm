#!/usr/bin/env python3
"""J6 disabled-state feedback source for the V15.30A loopback contract.

No enable, mode switch, motion, parameter write, ID write, set-zero, or safety
disable is exposed. The source aborts unless J6 is already DISABLED.
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import socket
import time
from pathlib import Path

from dm_g6220_transport import create_transport, state_text


GATE = "J6_DISABLED_STATE_FEEDBACK_ONLY=YES"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--confirm", default="")
    parser.add_argument("--seconds", type=float, default=30.0)
    parser.add_argument("--hz", type=float, default=50.0)
    parser.add_argument("--udp-host", default="127.0.0.1")
    parser.add_argument("--udp-port", type=int, default=15300)
    parser.add_argument("--output", type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if not args.execute:
        print("DRY_RUN=YES\nHARDWARE_OPENED=NO\nMOTION_API=NO")
        return 0
    if args.confirm != GATE:
        raise RuntimeError("operator gate missing")
    if not math.isfinite(args.seconds) or not 1.0 <= args.seconds <= 3600.0:
        raise ValueError("seconds must be in [1,3600]")
    if not math.isfinite(args.hz) or not 50.0 <= args.hz <= 100.0:
        raise ValueError("hz must be in [50,100]")

    args.output.parent.mkdir(parents=True, exist_ok=True)
    transport = create_transport("dmcan_sdk", channel=0)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    feedback_frames = 0
    published_updates = 0
    empty_cycles = 0
    period = 1.0 / args.hz
    rows = int(round(args.seconds * args.hz))
    try:
        initial = transport.refresh_feedback(1, timeout=0.25)
        if initial is None:
            raise RuntimeError("initial J6 feedback timeout")
        if initial.state != 0:
            raise RuntimeError(f"J6 must already be DISABLED, got {state_text(initial.state)}")
        with args.output.open("w", encoding="utf-8", newline="") as stream:
            fields = [
                "cycle", "source_monotonic_ns", "batch_size", "valid", "state", "state_name",
                "position_rad", "velocity_rad_s", "torque_protocol",
                "mos_temperature_c", "coil_temperature_c",
            ]
            writer = csv.DictWriter(stream, fieldnames=fields)
            writer.writeheader()
            start = time.monotonic()
            last_valid_time = start
            transport.clear_pending_feedback()
            for cycle in range(rows):
                deadline = start + cycle * period
                remaining = deadline - time.monotonic()
                if remaining > 0.0:
                    time.sleep(remaining)
                # The official USB backend delivers RX callbacks in batches.
                # Never clear its queue per request; drain each completed batch
                # and publish its newest physical sample.
                batch = transport.drain_feedback(1)
                feedback = batch[-1][1] if batch else None
                source_ns = time.monotonic_ns()
                good = feedback is not None and feedback.state == 0
                if good:
                    feedback_frames += len(batch)
                    published_updates += 1
                    last_valid_time = time.monotonic()
                    payload = {
                        "schema": "go-m8010-motor-feedback/1.0",
                        "source_monotonic_ns": source_ns,
                        "samples": [{
                            "motor": "J6",
                            "position_rad": feedback.position,
                            "velocity_rad_s": feedback.velocity,
                            "temperature_c": max(feedback.mos_temp, feedback.coil_temp),
                            "merror": 0,
                            "communication_ok": True,
                        }],
                    }
                    udp.sendto(json.dumps(payload, separators=(",", ":")).encode(), (args.udp_host, args.udp_port))
                else:
                    empty_cycles += 1
                writer.writerow({
                    "cycle": cycle,
                    "source_monotonic_ns": source_ns,
                    "batch_size": len(batch),
                    "valid": int(good),
                    "state": "" if feedback is None else feedback.state,
                    "state_name": "TIMEOUT" if feedback is None else state_text(feedback.state),
                    "position_rad": "" if feedback is None else feedback.position,
                    "velocity_rad_s": "" if feedback is None else feedback.velocity,
                    "torque_protocol": "" if feedback is None else feedback.torque,
                    "mos_temperature_c": "" if feedback is None else feedback.mos_temp,
                    "coil_temperature_c": "" if feedback is None else feedback.coil_temp,
                })
                if cycle % 50 == 0:
                    stream.flush()
                if feedback is not None and feedback.state != 0:
                    raise RuntimeError(f"J6 left DISABLED state: {state_text(feedback.state)}")
                if time.monotonic() - last_valid_time > 0.5:
                    raise RuntimeError("J6 produced no valid feedback batch for 0.5 seconds")
                transport.send_refresh_request(1)
            # Preserve any final callback batch in the source count.
            time.sleep(0.05)
            final_batch = transport.drain_feedback(1)
            if any(item[1].state != 0 for item in final_batch):
                raise RuntimeError("J6 final feedback was not DISABLED")
            feedback_frames += len(final_batch)
    finally:
        udp.close()
        transport.close()
    frame_rate = feedback_frames / max(args.seconds, 1.0)
    print(f"J6_STATE_FEEDBACK_REQUESTS={rows}")
    print(f"J6_STATE_FEEDBACK_FRAMES={feedback_frames}")
    print(f"J6_STATE_FEEDBACK_FRAME_RATE_HZ={frame_rate:.9f}")
    print(f"J6_STATE_FEEDBACK_PUBLISHED_UPDATES={published_updates}")
    print(f"J6_STATE_FEEDBACK_EMPTY_DRAIN_CYCLES={empty_cycles}")
    print("J6_ACTIVE_MOTION_USED=NO")
    passed = feedback_frames >= int(rows * 0.99) and published_updates > 0
    print("J6_STATE_FEEDBACK_RESULT=" + ("PASS" if passed else "FAIL"))
    return 0 if passed else 2


if __name__ == "__main__":
    raise SystemExit(main())

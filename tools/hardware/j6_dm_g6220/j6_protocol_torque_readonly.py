#!/usr/bin/env python3
"""Explicit read-only J6 parameter/torque capture. No FC, FD or RID writes."""
from __future__ import annotations

import argparse
from contextlib import ExitStack
from dataclasses import asdict
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import statistics
import time

from j6_protocol_torque import QUERY_RIDS, RID_NAMES, qualify_readback, readback_sha256, validate_readonly_request


def capture(output: Path, sample_count: int) -> dict:
    # Imports stay inside the explicitly requested hardware-read action.
    from j6_raw_can_diagnostic import RawCanLogger, read_parameter, capture_refresh_samples
    from j6_posvel_mode_commissioning import acquire_device_lock

    class ReadOnlyLogger(RawCanLogger):
        def _open_exact_adapter(self, *, initial):
            if not initial:
                raise RuntimeError("READONLY_QUERY_NO_RECONNECT_OR_DISABLE")
            return super()._open_exact_adapter(initial=True)

        def send(self, can_id, payload, label):
            validate_readonly_request(can_id, payload)
            return super().send(can_id, payload, label)

    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    raw_path = output.with_suffix(".raw.json")
    if output.exists() or raw_path.exists():
        raise ValueError("REFUSE_TO_OVERWRITE_J6_QUERY_EVIDENCE")
    result = {"schema": "go-m8010-j6-protocol-torque-readonly/1.0", "status": "FAIL",
        "created_at_utc": datetime.now(timezone.utc).isoformat(),
        "created_monotonic_ns": time.monotonic_ns(),
        "host_boot_id": Path("/proc/sys/kernel/random/boot_id").read_text().strip(),
        "motor_control_commands_sent": 0, "parameter_writes_sent": 0,
        "mode_changed": False, "motor_enabled": False, "readback": {}, "samples": []}
    resources = ExitStack()
    resources.enter_context(acquire_device_lock())
    logger = None
    try:
        logger = ReadOnlyLogger()
        for rid in QUERY_RIDS:
            value = read_parameter(logger, rid)
            result["readback"][str(rid)] = int(value) if rid in (7, 8, 10) else value
            if rid == 7:
                logger.master_id = int(value)
        result["rid_names"] = {str(k): v for k, v in RID_NAMES.items()}
        digest = readback_sha256(result["readback"])
        result["readback_sha256"] = digest
        try:
            result["qualification"] = qualify_readback(result["readback"], expected_sha256=digest)
        except ValueError as exc:
            result["qualification_error"] = str(exc)
        values, _ = capture_refresh_samples(logger, sample_count, "READONLY_TORQUE")
        result["samples"] = [{"source_monotonic_ns": int(event.event_monotonic_s * 1e9),
            "can_id": event.can_id, "payload_hex": event.payload.hex(),
            "drive_state": sample.state, "position_rad": sample.position,
            "velocity_rad_s": sample.velocity, "protocol_torque": sample.torque,
            "mos_temperature_c": sample.mos_temp, "coil_temperature_c": sample.coil_temp}
            for event, sample in values]
        if any(sample.state != 0 for _, sample in values):
            raise ValueError("J6_NOT_DISABLED_READONLY_QUERY_DID_NOT_CHANGE_MODE")
        torques = [sample.torque for _, sample in values]
        result["summary"] = {"valid_frames": len(values), "all_disabled": True,
            "sample_coverage_s": values[-1][0].event_monotonic_s - values[0][0].event_monotonic_s,
            "protocol_torque_min": min(torques), "protocol_torque_max": max(torques),
            "protocol_torque_median": statistics.median(torques),
            "scope": "DISABLED feedback only; does not validate force sensitivity while actively holding."}
        result["status"] = "PASS"
    except Exception as exc:
        result["failure"] = f"{type(exc).__name__}: {exc}"
    finally:
        if logger is not None:
            raw = [{**asdict(event), "payload": event.payload.hex()} for event in logger.snapshot()]
            raw_bytes = (json.dumps(raw, ensure_ascii=False, indent=2) + "\n").encode("utf-8")
            with raw_path.open("xb") as stream:
                stream.write(raw_bytes)
            result["raw_evidence"] = {"path": str(raw_path), "sha256": hashlib.sha256(raw_bytes).hexdigest()}
            logger.close()
        resources.close()
    with output.open("x", encoding="utf-8") as stream:
        json.dump(result, stream, ensure_ascii=False, indent=2)
        stream.write("\n")
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute-readonly", action="store_true")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples", type=int, default=100)
    args = parser.parse_args(argv)
    if not 10 <= args.samples <= 500:
        parser.error("--samples must be 10..500")
    if not args.execute_readonly:
        print(json.dumps({"mode": "DRY_RUN", "hardware_accessed": False,
            "rids": list(QUERY_RIDS), "samples": args.samples,
            "allowed_CAN_commands": ["READ_RID", "REFRESH_STATUS"]}))
        return 0
    try:
        result = capture(args.output, args.samples)
    except (OSError, ValueError, RuntimeError) as exc:
        print(json.dumps({"status": "BLOCKED", "failure": f"{type(exc).__name__}: {exc}",
                          "motor_control_commands_sent": 0, "parameter_writes_sent": 0}))
        return 2
    print(json.dumps({k: result.get(k) for k in ["status", "failure", "readback", "readback_sha256", "qualification", "qualification_error", "summary"]}, ensure_ascii=False))
    return 0 if result["status"] == "PASS" else 2


if __name__ == "__main__":
    raise SystemExit(main())

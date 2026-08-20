#!/usr/bin/env python3
"""Fail-closed J6 DM-G6220 discovery and 100 Hz feedback validation.

The default ``inventory`` action performs no bus traffic.  ``validate`` is
available only with an exact operator connection confirmation and an explicit
verified transport.  No motion, enable, mode switch, parameter write, ID write,
or set-zero command is implemented by this program.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import importlib.util
import importlib.metadata
import json
import math
import os
import platform
import statistics
import sys
import time
from collections import Counter
from datetime import datetime, timezone
from pathlib import Path
from typing import Dict, Iterable, List, Optional

from dm_g6220_transport import (
    CAN_BITRATE,
    PMAX_PROTOCOL_RAD,
    TMAX_PROTOCOL,
    VMAX_PROTOCOL_RAD_S,
    Feedback,
    TransportError,
    create_transport,
    ctrl_mode_text,
    state_text,
)


CONNECTION_GATE = "J6_DMG6220_POWER_AND_CAN_CONNECTED=YES"
REFERENCE_ZIP_SHA256 = "b6329bb0c3f1dea74a856b460e360ac86515e02a50820d72ffc62d69395ded89"


def _read_text(path: Path) -> Optional[str]:
    try:
        value = path.read_text(encoding="utf-8", errors="replace").strip()
        return value or None
    except OSError:
        return None


def usb_inventory() -> List[dict]:
    devices: List[dict] = []
    root = Path("/sys/bus/usb/devices")
    if not root.is_dir():
        return devices
    for entry in sorted(root.iterdir()):
        vendor = _read_text(entry / "idVendor")
        product_id = _read_text(entry / "idProduct")
        if vendor is None or product_id is None:
            continue
        devices.append({
            "sysfs": str(entry),
            "vid_pid": f"{vendor}:{product_id}",
            "manufacturer": _read_text(entry / "manufacturer"),
            "product": _read_text(entry / "product"),
            "serial": _read_text(entry / "serial"),
            "busnum": _read_text(entry / "busnum"),
            "devnum": _read_text(entry / "devnum"),
        })
    return devices


def serial_inventory() -> List[dict]:
    try:
        from serial.tools import list_ports
    except Exception:
        return []
    return [{
        "device": item.device,
        "description": item.description,
        "hwid": item.hwid,
        "vid_pid": None if item.vid is None or item.pid is None else f"{item.vid:04x}:{item.pid:04x}",
        "serial": item.serial_number,
        "location": item.location,
    } for item in list_ports.comports() if item.vid is not None or item.pid is not None]


def socketcan_inventory() -> List[dict]:
    interfaces: List[dict] = []
    for entry in sorted(Path("/sys/class/net").glob("*")):
        interface_type = _read_text(entry / "type")
        if interface_type == "280":
            interfaces.append({"interface": entry.name, "arphrd_type": 280})
    return interfaces


def base_inventory() -> dict:
    try:
        dmcan_sdk_version = importlib.metadata.version("dmcan-sdk")
    except importlib.metadata.PackageNotFoundError:
        dmcan_sdk_version = None
    return {
        "schema_version": "1.0",
        "task": "V15.21A J6 DM-G6220 read-only bring-up",
        "status": "INVENTORY_ONLY",
        "physical_gate": "NOT_CONFIRMED",
        "captured_at_utc": datetime.now(timezone.utc).isoformat(),
        "host": platform.node(),
        "platform": platform.platform(),
        "python": sys.version.split()[0],
        "reference_zip_sha256": REFERENCE_ZIP_SHA256,
        "python_modules": {
            "pyserial": importlib.util.find_spec("serial") is not None,
            "dmcan": importlib.util.find_spec("dmcan") is not None,
            "dmcan_sdk_version": dmcan_sdk_version,
        },
        "usb_devices": usb_inventory(),
        "serial_ports": serial_inventory(),
        "socketcan_interfaces": socketcan_inventory(),
        "safety": {
            "active_enable_used": False,
            "motor_motion_used": False,
            "parameter_write_used": False,
            "set_zero_used": False,
            "switch_mode_used": False,
            "id_write_used": False,
            "ready_pose_modified": False,
            "j1_authority_modified": False,
            "j345_authority_modified": False,
            "simulation_authority_modified": False,
        },
    }


def write_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def discover_one_motor(transport, ids: Iterable[int], timeout: float) -> dict:
    found: Dict[int, dict] = {}
    for requested_id in ids:
        mode = transport.read_parameter(requested_id, 10, timeout=timeout)
        pmax = transport.read_parameter(requested_id, 21, timeout=timeout)
        responses = [item for item in (mode, pmax) if item is not None]
        for response in responses:
            record = found.setdefault(response.slave_id, {
                "motor_id": response.slave_id,
                "requested_ids": [],
                "ctrl_mode": None,
                "pmax": None,
                "wrapper_can_ids": [],
            })
            if requested_id not in record["requested_ids"]:
                record["requested_ids"].append(requested_id)
            if response.wrapper_can_id not in record["wrapper_can_ids"]:
                record["wrapper_can_ids"].append(response.wrapper_can_id)
            if response.rid == 10:
                record["ctrl_mode"] = int(response.value)
            elif response.rid == 21:
                record["pmax"] = response.value

    if not found:
        raise RuntimeError("NOT_DETECTED: no RID response from IDs 1..32")
    if len(found) != 1:
        raise RuntimeError(f"MULTIPLE_DM_MOTORS: exact slave IDs={sorted(found)}")
    motor = next(iter(found.values()))
    if motor["ctrl_mode"] is None:
        raise RuntimeError("RID10 CTRL_MODE was not returned")
    if motor["pmax"] is None:
        raise RuntimeError("RID21 PMAX was not returned")
    motor["ctrl_mode_name"] = ctrl_mode_text(motor["ctrl_mode"])
    return motor


def feedback_dict(feedback: Feedback) -> dict:
    return {
        "can_id": feedback.can_id,
        "state": feedback.state,
        "state_name": state_text(feedback.state),
        "position_rad": feedback.position,
        "velocity_rad_s": feedback.velocity,
        "torque_protocol": feedback.torque,
        "mos_temperature_c": feedback.mos_temp,
        "coil_temperature_c": feedback.coil_temp,
    }


def ensure_disabled(transport, motor_id: int, initial: Feedback) -> tuple[Feedback, bool]:
    if initial.state == 0:
        return initial, False
    if initial.state != 1:
        raise RuntimeError(f"unsafe initial motor state: {state_text(initial.state)}")
    transport.send_safety_disable(motor_id)
    confirmation = transport.refresh_feedback(motor_id, timeout=0.25)
    if confirmation is None or confirmation.state != 0:
        state = "TIMEOUT" if confirmation is None else state_text(confirmation.state)
        raise RuntimeError(f"FD safety disable not confirmed: {state}")
    return confirmation, True


def _range(values: List[float]) -> dict:
    return {"min": min(values), "max": max(values)}


def run_feedback100(transport, motor_id: int, csv_path: Path, cycles: int = 500, hz: float = 100.0) -> dict:
    period = 1.0 / hz
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    request_times: List[float] = []
    received: List[tuple[float, Feedback]] = []
    transport.clear_pending_feedback()
    start = time.monotonic()

    for cycle in range(cycles):
        planned = start + cycle * period
        remain = planned - time.monotonic()
        if remain > 0:
            time.sleep(remain)
        request_time = time.monotonic()
        transport.send_refresh_request(motor_id)
        request_times.append(request_time)
        received.extend(transport.drain_feedback(motor_id))

    # DM_Device SDK may deliver USB RX callbacks in batches. Keep the request
    # cadence at 100 Hz, then allow a bounded drain interval and pair replies in
    # FIFO order. Clearing an entire callback batch and retaining one frame
    # would incorrectly report about 10% validity.
    drain_deadline = time.monotonic() + 0.5
    while len(received) < cycles and time.monotonic() < drain_deadline:
        received.extend(transport.drain_feedback(motor_id))
        if len(received) < cycles:
            time.sleep(0.001)
    received.extend(transport.drain_feedback(motor_id))

    matched = min(cycles, len(received))
    valid_feedback = [item[1] for item in received[:matched]]
    valid_rtt_ms = [
        max(0.0, (received[index][0] - request_times[index]) * 1000.0)
        for index in range(matched)
    ]
    rows: List[dict] = []
    for cycle, request_time in enumerate(request_times):
        feedback = received[cycle][1] if cycle < matched else None
        receive_time = received[cycle][0] if cycle < matched else None
        row = {
            "cycle": cycle,
            "request_monotonic_s": f"{request_time:.9f}",
            "receive_monotonic_s": "" if receive_time is None else f"{receive_time:.9f}",
            "rtt_ms": "" if receive_time is None else f"{max(0.0, (receive_time - request_time) * 1000.0):.6f}",
            "valid": int(feedback is not None),
            "can_id": "",
            "state": "",
            "state_name": "",
            "position_rad": "",
            "velocity_rad_s": "",
            "torque_protocol": "",
            "mos_temperature_c": "",
            "coil_temperature_c": "",
        }
        if feedback is not None:
            row.update(feedback_dict(feedback))
        rows.append(row)

    send_elapsed = request_times[-1] - request_times[0] + period
    fieldnames = list(rows[0])
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)

    valid = matched
    valid_rate = valid / cycles
    max_consecutive_timeout = cycles - valid
    states = Counter(state_text(item.state) for item in valid_feedback)
    positions = [item.position for item in valid_feedback]
    velocities = [item.velocity for item in valid_feedback]
    torques = [item.torque for item in valid_feedback]
    mos_temps = [item.mos_temp for item in valid_feedback]
    coil_temps = [item.coil_temp for item in valid_feedback]
    sorted_rtt = sorted(valid_rtt_ms)
    p95_index = max(0, math.ceil(0.95 * len(sorted_rtt)) - 1) if sorted_rtt else 0
    if valid >= 2:
        feedback_elapsed = received[valid - 1][0] - received[0][0]
        actual_hz = (valid - 1) / feedback_elapsed if feedback_elapsed > 0 else None
    else:
        feedback_elapsed = None
        actual_hz = None

    summary = {
        "target_hz": hz,
        "planned_frames": cycles,
        "valid_frames": valid,
        "valid_rate": valid_rate,
        "send_elapsed_s": send_elapsed,
        "feedback_elapsed_s": feedback_elapsed,
        "actual_hz": actual_hz,
        "mean_rtt_ms": statistics.fmean(valid_rtt_ms) if valid_rtt_ms else None,
        "p95_rtt_ms": sorted_rtt[p95_index] if sorted_rtt else None,
        "max_rtt_ms": max(valid_rtt_ms) if valid_rtt_ms else None,
        "max_consecutive_timeout": max_consecutive_timeout,
        "extra_unmatched_feedback": max(0, len(received) - cycles),
        "state_distribution": dict(states),
        "position_rad": None if not positions else {**_range(positions), "span": max(positions) - min(positions)},
        "velocity_rad_s": None if not velocities else _range(velocities),
        "torque_protocol": None if not torques else _range(torques),
        "mos_temperature_c": None if not mos_temps else _range(mos_temps),
        "coil_temperature_c": None if not coil_temps else _range(coil_temps),
        "csv_path": str(csv_path),
        "csv_sha256": sha256_file(csv_path),
    }
    finite_position = bool(positions) and all(math.isfinite(value) and abs(value) <= PMAX_PROTOCOL_RAD for value in positions)
    reasonable_temperature = bool(mos_temps and coil_temps) and all(
        -40 <= value <= 150 for value in mos_temps + coil_temps
    )
    reasonable_rate = actual_hz is not None and 90.0 <= actual_hz <= 110.0
    summary["status"] = "PASS" if (
        valid_rate >= 0.99
        and max_consecutive_timeout < 5
        and set(states) == {"DISABLED"}
        and finite_position
        and reasonable_temperature
        and reasonable_rate
    ) else "FAIL"
    return summary


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action", choices=("inventory", "validate"), nargs="?", default="inventory")
    parser.add_argument("--transport", choices=("dmcan_sdk", "serial_bridge"))
    parser.add_argument("--device", default="")
    parser.add_argument("--can-channel", type=int, default=0)
    parser.add_argument("--operator-confirmed", default="")
    parser.add_argument("--output-dir", type=Path, default=Path("hardware/v15_21a"))
    parser.add_argument("--inventory-output", type=Path)
    parser.add_argument("--cycles", type=int, default=500)
    parser.add_argument("--hz", type=float, default=100.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    inventory = base_inventory()
    if args.action == "inventory":
        if args.inventory_output:
            write_json(args.inventory_output, inventory)
        print(json.dumps(inventory, ensure_ascii=False, indent=2))
        return 0

    if args.operator_confirmed != CONNECTION_GATE:
        print(f"BLOCKED: require --operator-confirmed '{CONNECTION_GATE}'", file=sys.stderr)
        return 4
    if args.transport is None:
        print("BLOCKED: an actually detected --transport is required", file=sys.stderr)
        return 4
    if args.cycles != 500 or not math.isclose(args.hz, 100.0):
        print("BLOCKED: V15.21A validation is fixed at 500 cycles / 100 Hz", file=sys.stderr)
        return 4

    transport = None
    try:
        inventory["physical_gate"] = "OPERATOR_CONFIRMED"
        transport = create_transport(args.transport, args.device, args.can_channel)
        preferred_ids = [1] + list(range(2, 33))
        motor = discover_one_motor(transport, preferred_ids, timeout=0.12)
        motor_id = int(motor["motor_id"])
        initial = transport.refresh_feedback(motor_id, timeout=0.25)
        if initial is None:
            raise RuntimeError("initial refresh/status feedback timed out")
        disabled_feedback, disable_used = ensure_disabled(transport, motor_id, initial)

        timestamp = datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ")
        csv_path = args.output_dir / f"j6_feedback100_{timestamp}.csv"
        validation = run_feedback100(transport, motor_id, csv_path, cycles=args.cycles, hz=args.hz)
        inventory.update({
            "status": validation["status"],
            "transport": {
                "type": transport.kind,
                "device": getattr(transport, "device", args.device),
                "can_channel": transport.channel,
                "can_bitrate": CAN_BITRATE,
            },
            "motor": motor,
            "initial_feedback": feedback_dict(initial),
            "pre_validation_feedback": feedback_dict(disabled_feedback),
            "safety_disable_used": disable_used,
            "feedback100": validation,
            "calibration": {
                "sign": "PENDING",
                "cad_zero": "PENDING",
                "ros_zero": "PENDING",
                "motor_internal_zero": "UNCHANGED",
                "physical_limits": "PENDING",
                "ready_reference": "PENDING",
            },
            "protocol_mapping": {
                "classification": "PROTOCOL_ENCODING_RANGE",
                "position_rad": PMAX_PROTOCOL_RAD,
                "velocity_rad_s": VMAX_PROTOCOL_RAD_S,
                "torque": TMAX_PROTOCOL,
            },
        })
        inventory["safety"]["safety_fd_disable_used"] = disable_used
        output = args.inventory_output or args.output_dir / "j6_dmg6220_readonly_inventory.json"
        write_json(output, inventory)
        print(json.dumps(inventory, ensure_ascii=False, indent=2))
        return 0 if validation["status"] == "PASS" else 2
    except (RuntimeError, TransportError) as exc:
        inventory["status"] = "BLOCKED"
        inventory["blocker"] = str(exc)
        print(json.dumps(inventory, ensure_ascii=False, indent=2), file=sys.stderr)
        return 3
    finally:
        if transport is not None:
            transport.close()


if __name__ == "__main__":
    raise SystemExit(main())

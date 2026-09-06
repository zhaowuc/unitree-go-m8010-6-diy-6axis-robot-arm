#!/usr/bin/env python3
"""Raw BRAKE-only GO alignment guide; estimates are never calibration or permits."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import signal
import socket
import subprocess
import time

import v15_30a_j2_brake_raw_capture as raw_capture
import v15_30a_go_aux_brake_raw_capture as aux_capture
from v15_31d_demo_session import ROOT, PINNED

BUSES = {"j1": ("J1",), "j2": ("J2A", "J2B"), "j345": ("J3", "J4", "J5")}
MOTORS = tuple(name for names in BUSES.values() for name in names)
THERMAL_SHA = "1926264805858f62fffc9360ef0c9d4d7f8a7e232e171105450769d493ff5467"
FRESH_NS = 250_000_000


def finite(value):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError("non-finite diagnostic value")
    return float(value)


def estimate(raw, record, selected=None):
    """Choose a nearby branch once; never enforce/alter the real startup gate."""
    raw = finite(raw)
    reference, hint = finite(record["session_reference_raw_rad"]), finite(record["logical_position_rad"])
    sign, gear = record["sign"], finite(record["gear_ratio"])
    if sign not in (-1, 1) or type(sign) is not int or gear <= 0 or abs(raw) > 65536:
        raise ValueError("invalid raw/reference geometry")
    if selected is None:
        selected = reference + round((raw - sign * gear * hint - reference) / math.tau) * math.tau
    return math.degrees(sign * (raw - selected) / gear), selected


def packet_values(packet, names, refs, selected, now_ns):
    stamp = packet.get("source_monotonic_ns")
    if (packet.get("schema") != "go-m8010-motor-feedback/1.0" or type(stamp) is not int
            or not 0 < stamp <= now_ns or now_ns - stamp > FRESH_NS
            or packet.get("controller_mode") != "brake"
            or any(packet.get(key) is not False for key in ("domain_fault", "j2_sync_fault", "lease_safe_hold"))):
        raise ValueError("feedback is stale or not healthy BRAKE")
    samples = packet.get("samples")
    if not isinstance(samples, list) or tuple(item.get("motor") for item in samples) != names:
        raise ValueError("unexpected motor set")
    values, branches = {}, dict(selected)
    for item in samples:
        name = item["motor"]
        if (item.get("communication_ok") is not True or type(item.get("merror")) is not int
                or item["merror"] != 0 or packet.get("controller_mode_by_motor", {}).get(name) != "brake"):
            raise ValueError("invalid motor feedback")
        if not 0 <= finite(item["temperature_c"]) < 60:
            raise ValueError("temperature outside existing guard")
        finite(item["position_rad"])
        finite(item["velocity_rad_s"])
        values[name], branches[name] = estimate(item["unwrapped_raw_position_rad"], refs[name], branches.get(name))
    return values, branches


def snapshot(latest, now_ns):
    values = {name: (item[0] if item and 0 <= now_ns-item[1] <= FRESH_NS
                    and 0 <= now_ns-item[2] <= FRESH_NS else None)
              for name in MOTORS for item in (latest.get(name),)}
    sync = values["J2A"] - values["J2B"] if values["J2A"] is not None and values["J2B"] is not None else None
    ready = all(value is not None and abs(value) <= 2 for value in values.values()) and sync is not None and abs(sync) < 0.5
    return {"estimated_deg": values, "j2_sync_deg": sync, "within_original_window": ready,
            "stale": [name for name, value in values.items() if value is None]}


def run(args, refs):
    from PySide6.QtCore import QTimer
    from PySide6.QtWidgets import QApplication, QLabel, QPushButton, QVBoxLayout, QWidget
    app = QApplication([])
    window = QWidget()
    window.setWindowTitle("GO原始BRAKE回位诊断：不是校准或运行许可")
    layout = QVBoxLayout(window)
    layout.addWidget(QLabel("只读监视原 ±2° 启动窗口；不自动驱动，不改变零位。回正由主控制流程执行。\n"
                           "相邻圈估计仅适用于已确认的原竖直附近；完成后仍由原采样工具复核。"))
    labels = {name: QLabel(f"{name}：等待反馈") for name in MOTORS}
    for label in labels.values():
        layout.addWidget(label)
    status = QLabel()
    layout.addWidget(status)
    complete = QPushButton("已松手，完成回位")
    complete.setEnabled(False)
    close = QPushButton("关闭（Esc）")
    close.setShortcut("Esc")
    close.clicked.connect(window.close)
    layout.addWidget(complete)
    layout.addWidget(close)
    latest, branches, endpoints, stamps, processes, handles = {}, {}, {}, {}, {}, []
    result = {"diagnostic_only": True, "operator_hands_off": False, "reason": "closed", "terminal": {}}
    listener = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    listener.bind(("127.0.0.1", args.feedback_port))
    listener.setblocking(False)
    started = time.monotonic()
    printed = [0.0]
    started_ns = {}

    def finish():
        current = snapshot(latest, time.monotonic_ns())
        if current["within_original_window"]:
            result.update(current, operator_hands_off=True, reason="operator_finished")
            window.close()
    complete.clicked.connect(finish)

    def tick():
        nonlocal branches
        for _ in range(400):
            try:
                payload, address = listener.recvfrom(65535)
            except BlockingIOError:
                break
            bus = next((key for key, endpoint in endpoints.items() if endpoint == address), None)
            try:
                packet = json.loads(payload)
                names = tuple(item["motor"] for item in packet["samples"])
                candidate = next(key for key, names_for_bus in BUSES.items() if names_for_bus == names)
                if address[0] != "127.0.0.1" or (bus is not None and bus != candidate):
                    raise ValueError("feedback endpoint mismatch")
                bus = candidate
                if bus in endpoints and endpoints[bus] != address:
                    raise ValueError("feedback source changed")
                now_ns = time.monotonic_ns()
                values, next_branches = packet_values(packet, BUSES[bus], refs, branches, now_ns)
                stamp = packet["source_monotonic_ns"]
                if stamp < started_ns[bus] or stamp <= stamps.get(bus, 0):
                    raise ValueError("old or replayed feedback")
                endpoints[bus], stamps[bus], branches = address, stamp, next_branches
                latest.update({name: (value, stamp, now_ns) for name, value in values.items()})
            except (ValueError, KeyError, TypeError, AttributeError, StopIteration):
                if bus is not None:
                    for name in BUSES[bus]:
                        latest.pop(name, None)
        current = snapshot(latest, time.monotonic_ns())
        for name, value in current["estimated_deg"].items():
            text = "陈旧/无有效反馈" if value is None else f"{value:+.3f}° · " + ("超出原±2°窗口" if abs(value)>2 else "原窗口内")
            labels[name].setText(f"{name}：{text}")
            color = "gray" if value is None else "red" if abs(value)>2 else "green" if abs(value)<=1.5 else "blue"
            labels[name].setStyleSheet(f"font-size:24px;color:{color}")
        status.setText(f"J2A−J2B：{current['j2_sync_deg']}°；剩余 {max(0, int(args.seconds-(time.monotonic()-started)))} 秒")
        complete.setEnabled(current["within_original_window"])
        if time.monotonic() - printed[0] >= 1:
            print(json.dumps(current, ensure_ascii=False), flush=True)
            printed[0] = time.monotonic()
        if time.monotonic()-started >= args.seconds or any(p.poll() is not None for p in processes.values()):
            result["reason"] = "timeout_or_worker_exit"
            window.close()

    try:
        for bus in BUSES:
            command = raw_capture.build_worker_command(args)
            command[command.index("--bus")+1] = bus
            out = (args.output_dir/f"{bus}.stdout.log").open("x")
            err = (args.output_dir/f"{bus}.stderr.log").open("x")
            handles.extend((out, err))
            started_ns[bus] = time.monotonic_ns()
            processes[bus] = subprocess.Popen(command, stdout=out, stderr=err)
        timer = QTimer(window)
        timer.timeout.connect(tick)
        timer.start(50)
        for sig in (signal.SIGINT, signal.SIGTERM):
            signal.signal(sig, lambda *_: window.close())
        window.show()
        app.exec()
    finally:
        for process in processes.values():
            if process.poll() is None:
                process.terminate()
        for bus, process in processes.items():
            try:
                _, _, escalation = raw_capture.brake_verify.stop_worker(process)
                stdout = (args.output_dir/f"{bus}.stdout.log").read_text(errors="replace")
                proof = (raw_capture.brake_verify.parse_worker_terminal_proof(stdout) if bus == "j2"
                         else aux_capture._parse_terminal(stdout, bus))
                result["terminal"][bus] = {"confirmed": process.returncode == 0 and escalation == "none", "proof": proof}
            except Exception as error:
                result["terminal"][bus] = {"confirmed": False, "error": str(error)}
        for handle in handles:
            handle.close()
        listener.close()
        if not result["operator_hands_off"]:
            result.update(snapshot(latest, time.monotonic_ns()))
        result["all_workers_brake_confirmed"] = len(result["terminal"]) == 3 and all(item["confirmed"] for item in result["terminal"].values())
        (args.output_dir/"result.json").write_text(json.dumps(result, ensure_ascii=False, indent=2)+"\n", encoding="utf-8")
        print(json.dumps(result, ensure_ascii=False), flush=True)
    return 0 if result["all_workers_brake_confirmed"] else 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    ws = ROOT/"V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws"
    parser.add_argument("--worker", type=Path, default=ws/"build/v15_30a_gui_tools/v15_30a_gui_go_controller")
    parser.add_argument("--expected-worker-sha256")
    parser.add_argument("--thermal-config", type=Path, default=ws/"src/go_m8010_arm_hardware/config/thermal_limits.yaml")
    parser.add_argument("--feedback-port", type=int, default=15300)
    parser.add_argument("--seconds", type=float, default=300)
    parser.add_argument("--output-dir", type=Path)
    args = parser.parse_args()
    if not 0 < args.seconds <= 300 or not 1024 <= args.feedback_port <= 65535:
        parser.error("seconds must be within (0,300], feedback port within [1024,65535]")
    if not args.execute:
        print("OFFLINE_DESCRIPTION_ONLY: 6 GO motors, BRAKE-only diagnostic estimates; no calibration/permit or hardware access")
        return 0
    if args.output_dir is None or args.output_dir.exists() or not args.expected_worker_sha256:
        parser.error("execute requires new --output-dir and --expected-worker-sha256")
    if hashlib.sha256(args.worker.read_bytes()).hexdigest() != args.expected_worker_sha256:
        parser.error("worker SHA256 mismatch")
    args.expected_thermal_config_sha256 = THERMAL_SHA
    if hashlib.sha256(args.thermal_config.read_bytes()).hexdigest() != THERMAL_SHA:
        parser.error("thermal config SHA256 mismatch")
    refs = {}
    for name, digest in PINNED.items():
        if "/j2_anchors/" in name or "/go_anchors/" in name:
            data = (ROOT/name).read_bytes()
            if hashlib.sha256(data).hexdigest() != digest:
                parser.error(f"preserved reference SHA256 mismatch: {name}")
            refs.update(json.loads(data)["motors"])
    args.output_dir.mkdir(parents=True, mode=0o700)
    return run(args, refs)


if __name__ == "__main__":
    raise SystemExit(main())

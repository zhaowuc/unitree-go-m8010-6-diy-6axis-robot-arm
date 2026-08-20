#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""DM-G6220 基础位置控制 GUI。

使用达妙 USB-CAN 的串口桥协议（921600 baud）。界面只保留：
连接/扫描、输入角度发送、读取/记忆当前位置、复位到记忆位置、
设置当前位置为电机零点、使能和停止（失能）。

位置记忆保存到本地 JSON，仅保存目标/反馈角度；电机断电后若机械轴
被移动，仍需要回零或外部绝对位置基准，不能把本文件当作多圈编码器。
"""

from __future__ import annotations

import importlib.util
import json
import math
import re
import serial
import sys
import threading
import time
import tkinter as tk
from pathlib import Path
from tkinter import messagebox, ttk
from typing import Optional

from serial.tools import list_ports


BASE_DIR = Path(__file__).resolve().parent
MEMORY_FILE = BASE_DIR / "dm_g6220_position_memory.json"
DRIVER_FILE = BASE_DIR / "dm_g6220_run_100_cycle.py"


def load_driver_module():
    # dataclasses（驱动模块中的 Feedback）要求模块已经注册到 sys.modules。
    # 这也避免 GUI 启动后出现“通信模块加载失败”的隐蔽问题。
    module_name = "dm_g6220_transport"
    spec = importlib.util.spec_from_file_location(module_name, DRIVER_FILE)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"无法加载通信模块: {DRIVER_FILE}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    try:
        spec.loader.exec_module(module)
    except Exception:
        sys.modules.pop(module_name, None)
        raise
    return module


class DMG6220MemoryGUI:
    def __init__(self, root: tk.Tk):
        self.root = root
        self.root.title("DM-G6220 基础位置控制")
        self.root.geometry("720x520")
        self.root.minsize(680, 480)
        self.root.protocol("WM_DELETE_WINDOW", self._on_close)

        self.driver = None
        self.ctl = None
        self.motor_id = tk.StringVar(value="1")
        self.port = tk.StringVar()
        self.baud = tk.StringVar(value="921600")
        self.angle_deg = tk.StringVar(value="90")
        self.speed = tk.StringVar(value="0.50")
        self.auto_mode = tk.BooleanVar(value=True)
        self.connection_text = tk.StringVar(value="未连接")
        self.mode_text = tk.StringVar(value="未知")
        self.current_text = tk.StringVar(value="未知")
        self.memory_text = tk.StringVar(value="未记忆")
        self.target_text = tk.StringVar(value="-")
        self.transport_kind = ""
        self.busy = False
        self._closing = False
        self._known_usb_signature = ()
        self.lock = threading.RLock()
        self.memory: Optional[dict] = self._load_memory()

        try:
            self.driver = load_driver_module()
        except Exception as exc:
            self.driver = None
            self._driver_error = str(exc)
        else:
            self._driver_error = ""

        self._build_ui()
        self._refresh_ports()
        self.root.after(2000, self._watch_ports)
        self._update_memory_label()
        if self._driver_error:
            self._log(f"通信模块加载失败: {self._driver_error}")
        else:
            self._log("就绪：将自动扫描 USB-CAN 串口；也可手动点击“自动扫描连接”。")
            self.root.after(800, self._auto_scan_connect)

    # ---------------- UI ----------------
    def _build_ui(self):
        pad = {"padx": 10, "pady": 7}
        conn = ttk.LabelFrame(self.root, text="通信", padding=10)
        conn.pack(fill=tk.X, **pad)

        ttk.Label(conn, text="端口").grid(row=0, column=0, sticky=tk.W)
        self.port_box = ttk.Combobox(conn, textvariable=self.port, width=13, state="readonly")
        self.port_box.grid(row=0, column=1, sticky=tk.W, padx=(6, 6))
        ttk.Button(conn, text="刷新", command=self._refresh_ports).grid(row=0, column=2, padx=3)
        ttk.Button(conn, text="扫描电机", command=self._scan_ports).grid(row=0, column=3, padx=3)
        ttk.Label(conn, text="电机 ID").grid(row=0, column=4, padx=(14, 3), sticky=tk.W)
        ttk.Entry(conn, textvariable=self.motor_id, width=7).grid(row=0, column=5, sticky=tk.W)
        ttk.Label(conn, text="波特率").grid(row=0, column=6, padx=(14, 3), sticky=tk.W)
        ttk.Entry(conn, textvariable=self.baud, width=9).grid(row=0, column=7, sticky=tk.W)
        self.connect_btn = ttk.Button(conn, text="连接", command=self._connect)
        self.connect_btn.grid(row=0, column=8, padx=(14, 3))
        self.disconnect_btn = ttk.Button(conn, text="断开", command=self._disconnect, state=tk.DISABLED)
        self.disconnect_btn.grid(row=0, column=9)
        self.auto_connect_btn = ttk.Button(conn, text="自动扫描连接", command=self._auto_scan_connect)
        self.auto_connect_btn.grid(row=1, column=0, columnspan=3, pady=(9, 0), sticky=tk.W)
        self.serial_diag_btn = ttk.Button(conn, text="串口上电日志诊断", command=self._serial_diagnostic)
        self.serial_diag_btn.grid(row=1, column=3, columnspan=3, padx=(8, 3), pady=(9, 0), sticky=tk.W)
        ttk.Label(conn, text="自动扫描/诊断均不发送运动命令。").grid(
            row=1, column=6, columnspan=4, padx=8, pady=(9, 0), sticky=tk.W
        )

        status = ttk.LabelFrame(self.root, text="状态", padding=10)
        status.pack(fill=tk.X, **pad)
        ttk.Label(status, text="连接").grid(row=0, column=0, sticky=tk.W)
        ttk.Label(status, textvariable=self.connection_text).grid(row=0, column=1, sticky=tk.W, padx=6)
        ttk.Label(status, text="控制模式").grid(row=0, column=2, sticky=tk.W, padx=(24, 3))
        ttk.Label(status, textvariable=self.mode_text).grid(row=0, column=3, sticky=tk.W, padx=6)
        ttk.Label(status, text="当前位置").grid(row=1, column=0, sticky=tk.W, pady=(8, 0))
        ttk.Label(status, textvariable=self.current_text).grid(row=1, column=1, columnspan=3, sticky=tk.W, padx=6, pady=(8, 0))
        ttk.Label(status, text="已记忆位置").grid(row=2, column=0, sticky=tk.W, pady=(8, 0))
        ttk.Label(status, textvariable=self.memory_text).grid(row=2, column=1, columnspan=3, sticky=tk.W, padx=6, pady=(8, 0))

        motion = ttk.LabelFrame(self.root, text="基础位置操作", padding=10)
        motion.pack(fill=tk.X, **pad)
        ttk.Label(motion, text="目标角度 (°)").grid(row=0, column=0, sticky=tk.W)
        ttk.Entry(motion, textvariable=self.angle_deg, width=12).grid(row=0, column=1, sticky=tk.W, padx=6)
        ttk.Label(motion, text="速度上限 (rad/s)").grid(row=0, column=2, sticky=tk.W, padx=(22, 3))
        ttk.Entry(motion, textvariable=self.speed, width=10).grid(row=0, column=3, sticky=tk.W, padx=6)
        ttk.Checkbutton(motion, text="发送前自动切到位置模式", variable=self.auto_mode).grid(
            row=1, column=0, columnspan=3, sticky=tk.W, pady=(8, 0)
        )

        self.send_btn = ttk.Button(motion, text="发送角度", command=self._send_angle)
        self.send_btn.grid(row=2, column=0, padx=(0, 6), pady=(14, 0), sticky=tk.W)
        self.read_btn = ttk.Button(motion, text="读取当前位置", command=self._read_position)
        self.read_btn.grid(row=2, column=1, padx=6, pady=(14, 0), sticky=tk.W)
        self.remember_btn = ttk.Button(motion, text="记忆当前位置", command=self._remember_position)
        self.remember_btn.grid(row=2, column=2, padx=6, pady=(14, 0), sticky=tk.W)
        self.reset_btn = ttk.Button(motion, text="复位到记忆位置", command=self._reset_to_memory)
        self.reset_btn.grid(row=2, column=3, padx=6, pady=(14, 0), sticky=tk.W)
        self.zero_btn = ttk.Button(motion, text="当前位置设为零点", command=self._set_zero, state=tk.DISABLED)
        self.zero_btn.grid(row=3, column=0, padx=(0, 6), pady=(10, 0), sticky=tk.W)
        self.enable_btn = ttk.Button(motion, text="使能电机", command=self._enable, state=tk.DISABLED)
        self.enable_btn.grid(row=3, column=1, padx=6, pady=(10, 0), sticky=tk.W)
        self.stop_btn = ttk.Button(motion, text="停止/失能", command=self._disable, state=tk.DISABLED)
        self.stop_btn.grid(row=3, column=2, padx=6, pady=(10, 0), sticky=tk.W)
        ttk.Label(motion, text="角度范围：±716°；默认速度 0.50 rad/s").grid(
            row=3, column=3, padx=6, pady=(10, 0), sticky=tk.W
        )

        log_frame = ttk.LabelFrame(self.root, text="日志", padding=8)
        log_frame.pack(fill=tk.BOTH, expand=True, **pad)
        self.log = tk.Text(log_frame, height=8, state=tk.DISABLED, wrap=tk.WORD)
        self.log.pack(fill=tk.BOTH, expand=True)

    def _log(self, text: str):
        def append():
            self.log.config(state=tk.NORMAL)
            self.log.insert(tk.END, f"[{time.strftime('%H:%M:%S')}] {text}\n")
            self.log.see(tk.END)
            self.log.config(state=tk.DISABLED)
        self.root.after(0, append)

    def _set_status(self, text: str):
        self.root.after(0, lambda: self.connection_text.set(text))

    def _refresh_ports(self):
        infos = list(list_ports.comports())
        ports = [p.device for p in infos]
        usb_infos = [p for p in infos if p.vid is not None or p.pid is not None]
        self.port_box["values"] = ports
        # 模块热插拔后旧 COM 号会失效；优先选择真正的 USB 串口，跳过蓝牙 SPP。
        if ports and self.port.get() not in ports:
            self.port.set(usb_infos[0].device if usb_infos else ports[0])
        if ports:
            self._log("检测到串口: " + ", ".join(ports))
        else:
            self.port.set("")
            self._log("未检测到串口。")
        self._known_usb_signature = tuple(
            (p.device, p.vid, p.pid, p.serial_number or "") for p in usb_infos
        )

    def _watch_ports(self):
        """监视 USB 模块更换；发现新模块时更新端口并自动重新扫描。"""
        if self._closing:
            return
        infos = list(list_ports.comports())
        usb_infos = [p for p in infos if p.vid is not None or p.pid is not None]
        signature = tuple((p.device, p.vid, p.pid, p.serial_number or "") for p in usb_infos)
        if signature != self._known_usb_signature:
            old = self._known_usb_signature
            self._known_usb_signature = signature
            ports = [p.device for p in infos]
            self.port_box["values"] = ports
            if self.port.get() not in ports:
                self.port.set(usb_infos[0].device if usb_infos else (ports[0] if ports else ""))
            if usb_infos:
                details = ", ".join(
                    f"{p.device} VID:PID={p.vid:04X}:{p.pid:04X} SN={p.serial_number or '-'}"
                    for p in usb_infos
                )
                self._log(f"检测到 USB 模块变化：{details}")
                if self.ctl is None and not self.busy:
                    self.root.after(250, self._auto_scan_connect)
            elif old:
                self._log("USB 模块已拔出。")
                if self.ctl is not None:
                    self._disconnect()
        self.root.after(2000, self._watch_ports)

    # ---------------- persistence ----------------
    def _load_memory(self) -> Optional[dict]:
        try:
            data = json.loads(MEMORY_FILE.read_text(encoding="utf-8"))
            if isinstance(data, dict) and isinstance(data.get("position_rad"), (int, float)):
                return data
        except (OSError, ValueError, TypeError):
            pass
        return None

    def _update_memory_label(self):
        if self.memory is None:
            self.memory_text.set("未记忆")
            return
        rad = float(self.memory["position_rad"])
        self.memory_text.set(f"{math.degrees(rad):.2f}° ({rad:.5f} rad)")

    def _save_memory(self, position_rad: float, source: str):
        self.memory = {
            "position_rad": float(position_rad),
            "position_deg": math.degrees(float(position_rad)),
            "source": source,
            "saved_at": time.strftime("%Y-%m-%d %H:%M:%S"),
        }
        MEMORY_FILE.write_text(json.dumps(self.memory, ensure_ascii=False, indent=2), encoding="utf-8")
        self.root.after(0, self._update_memory_label)

    # ---------------- parsing / transport ----------------
    def _parse_motor_id(self) -> int:
        text = self.motor_id.get().strip().lower()
        value = int(text, 16) if text.startswith("0x") else int(text, 10)
        if not 1 <= value <= 0x7FF:
            raise ValueError("电机 ID 必须在 1~0x7FF")
        return value

    def _parse_target(self) -> tuple[int, float, float]:
        motor_id = self._parse_motor_id()
        deg = float(self.angle_deg.get().strip())
        speed = float(self.speed.get().strip())
        rad = math.radians(deg)
        if not math.isfinite(rad) or abs(rad) > 12.5:
            raise ValueError("目标角度超出 DM-G6220 默认 ±12.5 rad 范围（约 ±716°）")
        if not math.isfinite(speed) or not 0.05 <= speed <= 5.0:
            raise ValueError("速度必须在 0.05~5.0 rad/s")
        return motor_id, rad, speed

    def _require_connected(self):
        if self.ctl is None:
            raise RuntimeError("请先连接 USB-CAN 串口桥")
        return self.ctl

    def _refresh_feedback(self, ctl, motor_id: int):
        # 官方 refresh_motor_status：0x7FF + [slave_lo, slave_hi, 0xCC, ...]
        data = bytes((motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0xCC, 0, 0, 0, 0, 0))
        ctl._send_data(0x7FF, data)
        replies = ctl.receive(motor_id, timeout=0.45)
        return replies[-1] if replies else None

    def _set_current(self, feedback):
        if feedback is None:
            self.root.after(0, lambda: self.current_text.set("未收到反馈"))
            return
        text = f"{math.degrees(feedback.position):.2f}° ({feedback.position:.5f} rad), 状态={self.driver.status_text(feedback.state)}"
        self.root.after(0, lambda: self.current_text.set(text))

    # ---------------- connection / scan ----------------
    def _scan_ports(self):
        if self.busy:
            return
        self.busy = True
        self._log("开始只读扫描电机端口，不发送运动命令。")

        def worker():
            try:
                if self.driver is None:
                    raise RuntimeError(self._driver_error)
                mid = self._parse_motor_id()
                baud = int(self.baud.get().strip())
                found = []
                for info in list_ports.comports():
                    try:
                        probe = self.driver.OfficialSerialMotorControl(info.device, baudrate=baud, timeout=0.08)
                        mode = probe.read_param(mid, 10, timeout=0.35)
                        probe.close()
                        if mode is not None:
                            found.append((info.device, mode))
                            self._log(f"{info.device}: 收到电机 ID={mid} 应答，CTRL_MODE={int(mode)}")
                        else:
                            self._log(f"{info.device}: 无目标电机应答")
                    except Exception as exc:
                        self._log(f"{info.device}: 跳过（{exc}）")
                if found:
                    self.root.after(0, lambda: self.port.set(found[0][0]))
                    self._log(f"已选择在线端口 {found[0][0]}")
                else:
                    self._log("没有扫描到目标电机；未发送任何运动命令。")
            except Exception as exc:
                self._log(f"扫描失败: {exc}")
            finally:
                self.busy = False

        threading.Thread(target=worker, daemon=True).start()

    def _auto_scan_connect(self):
        """扫描 USB 串口及常见电机 ID，发现应答后自动建立连接。"""
        if self.busy or self.ctl is not None:
            if self.ctl is not None:
                self._log("当前已经连接电机，无需重复自动扫描。")
            return
        if self.driver is None:
            self._log(f"通信模块加载失败: {self._driver_error}")
            return
        self.busy = True
        self.auto_connect_btn.config(state=tk.DISABLED)
        self._log("开始自动扫描：跳过蓝牙端口，依次查询电机 ID 1~32。")

        def worker():
            found = None
            try:
                selected_baud = int(self.baud.get().strip())
                baud_candidates = []
                for candidate in (selected_baud, 921600, 115200, 460800, 1000000, 230400):
                    if candidate not in baud_candidates:
                        baud_candidates.append(candidate)
                preferred = None
                try:
                    preferred = self._parse_motor_id()
                except (TypeError, ValueError):
                    pass
                ids = list(range(1, 33))
                if preferred is not None:
                    ids = [preferred] + [mid for mid in ids if mid != preferred]

                infos = list(list_ports.comports())
                usb_infos = [
                    info for info in infos
                    if not (info.vid is None and info.pid is None)
                ]
                if not usb_infos:
                    self._log("未发现 USB 串口；当前仅有蓝牙/虚拟串口。")

                # 新款/切换后的达妙模块由官方 dm-device SDK 原生访问，即使它
                # 同时枚举出 COM 口，也不能再使用旧的 55-AA 虚拟串口封装。
                native_probe = None
                try:
                    self._log("优先探测达妙官方 dm-device USB-CAN 后端。")
                    native_probe = self.driver.DmDeviceMotorControl(
                        channel=0, can_baudrate=1_000_000
                    )
                    for mid in ids:
                        mode = native_probe.read_param(mid, 10, timeout=0.15)
                        if mode is not None:
                            display_port = usb_infos[0].device if usb_infos else "DM USB-CAN"
                            # 保留这次已经打开的原生连接。官方 Windows SDK 在同一
                            # 进程内关闭后立即重开可能等待 libusb 线程，复用连接更稳。
                            found = (
                                "dm-device", display_port, mid, int(mode),
                                1_000_000, native_probe,
                            )
                            native_probe = None
                            self._log(
                                f"官方 USB-CAN: 找到电机 ID={mid}，CTRL_MODE={int(mode)}"
                            )
                            break
                except Exception as exc:
                    self._log(f"官方原生后端未找到可用电机，回退串口扫描（{exc}）")
                finally:
                    if native_probe is not None:
                        try:
                            native_probe.close()
                        except Exception:
                            pass

                for info in ([] if found is not None else usb_infos):
                    self._log(f"探测 {info.device} ({info.description or 'USB 串口'})")
                    for baud in baud_candidates:
                        probe = None
                        try:
                            probe = self.driver.OfficialSerialMotorControl(
                                info.device, baudrate=baud, timeout=0.06
                            )
                            for mid in ids:
                                mode = probe.read_param(mid, 10, timeout=0.10)
                                if mode is not None:
                                    found = ("serial", info.device, mid, int(mode), baud, None)
                                    self._log(
                                        f"{info.device}: 找到电机 ID={mid}，波特率={baud}，CTRL_MODE={int(mode)}"
                                    )
                                    break
                            if found is not None:
                                break
                            self._log(f"{info.device} @ {baud}: 未收到 ID 1~32 的电机应答")
                        except Exception as exc:
                            self._log(f"{info.device} @ {baud}: 跳过（{exc}）")
                        finally:
                            if probe is not None:
                                try:
                                    probe.close()
                                except Exception:
                                    pass
                    if found is not None:
                        break
            except Exception as exc:
                self._log(f"自动扫描失败: {exc}")
            finally:
                self.busy = False
                self.root.after(0, lambda: self.auto_connect_btn.config(state=tk.NORMAL))
                if found is not None:
                    self.root.after(0, lambda result=found: self._connect_found(result))
                else:
                    self._log("未找到电机应答。请确认电机电源、CANH/CANL、串口桥和电机 ID。")

        threading.Thread(target=worker, daemon=True).start()

    def _serial_diagnostic(self):
        """监听官方 3-pin UART 的上电打印，辅助确认串口线序和 CAN_ID。"""
        if self.busy:
            self._log("上一项扫描/诊断仍在执行，请稍候。")
            return
        port = self.port.get().strip()
        if not port:
            ports = [p.device for p in list_ports.comports() if p.vid is not None or p.pid is not None]
            if ports:
                port = ports[0]
                self.port.set(port)
            else:
                self._log("没有可用 USB 串口，无法进行串口诊断。")
                return
        self.busy = True
        self.serial_diag_btn.config(state=tk.DISABLED)
        self._log(f"开始监听 {port} 的官方 3-pin 调试串口（921600 8N1）。")
        self._log("请在监听期间给电机完全断电后重新上电；官方固件会在上电时打印 CAN_ID 和控制模式。")

        def worker():
            raw = bytearray()
            try:
                with serial.Serial(port=port, baudrate=921600, timeout=0.10) as link:
                    link.reset_input_buffer()
                    deadline = time.monotonic() + 5.0
                    while time.monotonic() < deadline:
                        chunk = link.read_all()
                        if chunk:
                            raw.extend(chunk)
                        time.sleep(0.02)
                if raw:
                    text = raw.decode("utf-8", errors="replace")
                    compact = " ".join(text.split())
                    self._log(f"收到串口数据 {len(raw)} 字节：{compact[:500]}")
                    self._log(f"串口 HEX（前 96 字节）：{raw[:96].hex(' ')}")
                    id_match = re.search(r"(?:CAN[_ ]?ID|Slave[_ ]?ID)\s*[:=：]?\s*(0x[0-9a-fA-F]+|\d+)", text, re.I)
                    if id_match:
                        raw_id = id_match.group(1)
                        motor_id = int(raw_id, 16) if raw_id.lower().startswith("0x") else int(raw_id)
                        if 1 <= motor_id <= 0x7FF:
                            self.root.after(0, lambda value=str(motor_id): self.motor_id.set(value))
                            self._log(f"从上电日志解析到 CAN_ID={motor_id}，已填入电机 ID。")
                    mode_match = re.search(r"(?:Control\s*Mode|控制模式)\s*[:=：]?\s*([^\r\n]+)", text, re.I)
                    if mode_match:
                        self._log(f"上电日志中的当前模式：{mode_match.group(1).strip()}")
                else:
                    self._log("5 秒内没有收到串口数据。请确认 3-pin 线序：RX↔TX、TX↔RX、GND↔GND，并在监听期间重新给电机上电。")
            except Exception as exc:
                self._log(f"串口诊断失败: {exc}")
            finally:
                self.busy = False
                self.root.after(0, lambda: self.serial_diag_btn.config(state=tk.NORMAL))

        threading.Thread(target=worker, daemon=True).start()

    def _connect_found(self, result):
        """在 GUI 线程中使用自动扫描结果建立连接。"""
        transport, port, mid, mode, link_rate, opened_connection = result
        if self.ctl is not None:
            if opened_connection is not None:
                try:
                    opened_connection.close()
                except Exception:
                    pass
            return
        try:
            self.port.set(port)
            self.motor_id.set(str(mid))
            if transport == "dm-device":
                self.ctl = opened_connection
                if self.ctl is None:
                    raise RuntimeError("原生 USB-CAN 连接已失效")
                self.transport_kind = "dm-device"
                self.connection_text.set(f"已连接官方 USB-CAN（{port}）")
            else:
                self.ctl = self.driver.OfficialSerialMotorControl(
                    port, baudrate=link_rate, timeout=0.15
                )
                self.transport_kind = "serial"
                self.connection_text.set(f"已连接串口桥 {port}")
            self.mode_text.set(str(mode))
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            for button in (
                self.send_btn, self.read_btn, self.remember_btn, self.reset_btn,
                self.zero_btn, self.enable_btn, self.stop_btn,
            ):
                button.config(state=tk.NORMAL)
            self._log(
                f"自动连接成功：后端={transport}，端口={port}，链路速率={link_rate}，"
                f"电机 ID={mid}，CTRL_MODE={mode}"
            )
            self._read_position()
        except Exception as exc:
            if opened_connection is not None:
                try:
                    opened_connection.close()
                except Exception:
                    pass
            self.ctl = None
            self.transport_kind = ""
            self._log(f"自动连接失败: {exc}")
            messagebox.showerror("自动连接失败", str(exc))

    def _connect(self):
        if self.busy:
            return
        if self.driver is None:
            messagebox.showerror("通信模块", self._driver_error)
            return
        port = self.port.get().strip()
        if not port:
            messagebox.showwarning("端口", "请选择或扫描 COM 端口。")
            return
        try:
            baud = int(self.baud.get().strip())
            mid = self._parse_motor_id()
            self.ctl = self.driver.OfficialSerialMotorControl(port, baudrate=baud, timeout=0.15)
            self.transport_kind = "serial"
            mode = self.ctl.read_param(mid, 10, timeout=0.5)
            self.connection_text.set(f"已连接 {port}")
            self.mode_text.set(str(int(mode)) if mode is not None else "未知")
            self.connect_btn.config(state=tk.DISABLED)
            self.disconnect_btn.config(state=tk.NORMAL)
            for button in (self.send_btn, self.read_btn, self.remember_btn, self.reset_btn, self.zero_btn, self.enable_btn, self.stop_btn):
                button.config(state=tk.NORMAL)
            self._log(f"已连接 {port} @ {baud}，电机 ID={mid}，CTRL_MODE={mode}")
            self._read_position()
        except Exception as exc:
            self.ctl = None
            messagebox.showerror("连接失败", str(exc))
            self._log(f"连接失败: {exc}")

    def _disconnect(self):
        with self.lock:
            ctl = self.ctl
            self.ctl = None
            self.transport_kind = ""
        if ctl is not None:
            try:
                ctl.disable(self._parse_motor_id())
            except Exception:
                pass
            ctl.close()
        self.connection_text.set("未连接")
        self.mode_text.set("未知")
        self.connect_btn.config(state=tk.NORMAL)
        self.disconnect_btn.config(state=tk.DISABLED)
        for button in (self.send_btn, self.read_btn, self.remember_btn, self.reset_btn, self.zero_btn, self.enable_btn, self.stop_btn):
            button.config(state=tk.DISABLED)
        self._log("已断开，电机已尝试失能。")

    # ---------------- actions ----------------
    def _run_action(self, title: str, action):
        if self.busy:
            self._log("上一操作仍在执行，请稍候。")
            return
        self.busy = True

        def worker():
            try:
                with self.lock:
                    action()
            except Exception as exc:
                self._log(f"{title}失败: {exc}")
                self.root.after(0, lambda: messagebox.showerror(title, str(exc)))
            finally:
                self.busy = False

        threading.Thread(target=worker, daemon=True).start()

    def _ensure_position_mode(self, ctl, motor_id: int):
        if not self.auto_mode.get():
            return
        if ctl.switch_mode(motor_id, self.driver.CTRL_MODE_POS_VEL, timeout=0.8):
            self.root.after(0, lambda: self.mode_text.set("2（位置速度）"))
            self._log("已切换为位置速度模式（未保存其它参数）。")
        else:
            raise RuntimeError("切换位置模式未收到确认，已阻止运动")

    def _send_angle(self):
        def action():
            ctl = self._require_connected()
            mid, rad, speed = self._parse_target()
            self._ensure_position_mode(ctl, mid)
            ctl.enable(mid, self.driver.CTRL_MODE_POS_VEL, legacy=True)
            replies = ctl.control(mid, self.driver.CTRL_MODE_POS_VEL, rad, speed, 0, 0, 0)
            self.target_text.set(f"{math.degrees(rad):.2f}°")
            if replies:
                self._set_current(replies[-1])
            self._log(f"已发送目标 {math.degrees(rad):.2f}°，速度上限 {speed:.2f} rad/s")
        self._run_action("发送角度", action)

    def _read_position(self):
        def action():
            ctl = self._require_connected()
            feedback = self._refresh_feedback(ctl, self._parse_motor_id())
            self._set_current(feedback)
            if feedback:
                self._log(f"当前位置 {math.degrees(feedback.position):.2f}° ({feedback.position:.5f} rad)")
            else:
                self._log("未收到当前位置反馈。")
        self._run_action("读取位置", action)

    def _remember_position(self):
        def action():
            ctl = self._require_connected()
            feedback = self._refresh_feedback(ctl, self._parse_motor_id())
            if feedback is None:
                raise RuntimeError("未收到反馈，不能记忆当前位置")
            self._set_current(feedback)
            self._save_memory(feedback.position, "CAN feedback")
            self._log(f"已记忆当前位置 {math.degrees(feedback.position):.2f}°")
        self._run_action("记忆位置", action)

    def _reset_to_memory(self):
        if self.memory is None:
            messagebox.showwarning("复位", "尚未记忆位置，请先点击“记忆当前位置”。")
            return
        self.angle_deg.set(f"{math.degrees(float(self.memory['position_rad'])):.2f}")
        self._send_angle()

    def _set_zero(self):
        def action():
            ctl = self._require_connected()
            mid = self._parse_motor_id()
            ctl.disable(mid)
            ctl.set_zero(mid)
            self._log("已发送“保存位置零点”命令。建议在失能状态确认机械位置后再继续运动。")
            feedback = self._refresh_feedback(ctl, mid)
            self._set_current(feedback)
        self._run_action("设置零点", action)

    def _enable(self):
        def action():
            ctl = self._require_connected()
            mid = self._parse_motor_id()
            ctl.enable(mid, self.driver.CTRL_MODE_POS_VEL, legacy=True)
            self._log("已发送使能命令；尚未发送运动目标。")
        self._run_action("使能", action)

    def _disable(self):
        def action():
            ctl = self._require_connected()
            ctl.disable(self._parse_motor_id())
            self._log("已发送失能命令。")
        self._run_action("停止/失能", action)

    def _on_close(self):
        self._closing = True
        try:
            self._disconnect()
        finally:
            self.root.destroy()


def main():
    root = tk.Tk()
    DMG6220MemoryGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()

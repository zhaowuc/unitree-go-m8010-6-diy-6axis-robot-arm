#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DM-G6220 直接控制脚本。

本文件按达妙公开 DM_CAN.py 的串口封装改写：USB-CAN 适配器不是
python-can 的裸 CAN serial backend，而是 921600 baud 的固定 USB 帧：
发送 29 字节 55 AA ...，接收 16 字节 AA ... 55。

默认启动后自动选择端口、读取 CTRL_MODE、发送官方 FC 使能帧，然后执行
正向 100° / 反向 100° 一轮。实际电机 ID 默认 1，可用 --motor-id 修改。
"""

from __future__ import annotations

import argparse
import os
import math
import struct
import sys
import threading
import time
from collections import deque
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Optional, Tuple

import serial
from serial.tools import list_ports


# DM_CAN.py 中 DMG6220 的官方映射限制（DM_Motor_Type.DMG6220 = 11）。
PMAX_DEFAULT = 12.5
VMAX_DEFAULT = 45.0
TMAX_DEFAULT = 10.0
SERIAL_BAUD_DEFAULT = 921600

CTRL_MODE_MIT = 1
CTRL_MODE_POS_VEL = 2
CTRL_MODE_VEL = 3


def clampf(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def float_to_uint(x: float, lo: float, hi: float, bits: int) -> int:
    x = clampf(x, lo, hi)
    return int((x - lo) * ((1 << bits) - 1) / (hi - lo))


def uint_to_float(x: int, lo: float, hi: float, bits: int) -> float:
    return lo + (x / ((1 << bits) - 1)) * (hi - lo)


def pack_mit(position: float, velocity: float, kp: float, kd: float, torque: float) -> bytes:
    """与官方 DM_CAN.py controlMIT 完全相同的 16/12/12/12 位域布局。"""
    q = float_to_uint(position, -PMAX_DEFAULT, PMAX_DEFAULT, 16)
    dq = float_to_uint(velocity, -VMAX_DEFAULT, VMAX_DEFAULT, 12)
    kp_u = float_to_uint(kp, 0.0, 500.0, 12)
    kd_u = float_to_uint(kd, 0.0, 5.0, 12)
    tau = float_to_uint(torque, -TMAX_DEFAULT, TMAX_DEFAULT, 12)
    return bytes((
        (q >> 8) & 0xFF,
        q & 0xFF,
        dq >> 4,
        ((dq & 0x0F) << 4) | ((kp_u >> 8) & 0x0F),
        kp_u & 0xFF,
        kd_u >> 4,
        ((kd_u & 0x0F) << 4) | ((tau >> 8) & 0x0F),
        tau & 0xFF,
    ))


def pack_pos_vel(position: float, velocity: float) -> bytes:
    """官方 control_Pos_Vel：p_des、v_des 各一个 little-endian float32。"""
    return struct.pack("<ff", float(position), float(velocity))


def pack_vel(velocity: float) -> bytes:
    return struct.pack("<f", float(velocity)) + b"\x00" * 4


def status_text(state: int) -> str:
    return {
        0: "DISABLED",
        1: "ENABLED",
        8: "OVER_VOLTAGE",
        9: "UNDER_VOLTAGE",
        0xA: "OVER_CURRENT",
        0xB: "MOS_OVER_TEMP",
        0xC: "COIL_OVER_TEMP",
        0xD: "LOST_COMM",
        0xE: "OVERLOAD",
    }.get(state, f"0x{state:X}")


@dataclass
class Feedback:
    can_id: int
    state: int
    position: float
    velocity: float
    torque: float
    mos_temp: int = 0
    coil_temp: int = 0


def decode_feedback(packet: bytes, motor_id: int) -> Optional[Feedback]:
    """按官方 DM_CAN.py __process_packet 解码 16 字节反馈封装。"""
    if len(packet) != 16 or packet[0] != 0xAA or packet[15] != 0x55 or packet[1] != 0x11:
        return None
    can_id = int.from_bytes(packet[3:7], "little")
    data = packet[7:15]
    # 新固件通常返回 CAN ID=电机 ID；旧/部分适配器返回 CAN ID=0，
    # 此时 data[0] 的低半字节是 Master ID，位置字段从 data[1] 开始。
    if can_id == motor_id:
        d = data
        state = d[0] & 0x0F
        pos_u = (d[1] << 8) | d[2]
        vel_u = (d[3] << 4) | (d[4] >> 4)
        tau_u = ((d[4] & 0x0F) << 8) | d[5]
    elif can_id == 0:
        d = data
        if (d[0] & 0x0F) != (motor_id & 0x0F):
            return None
        state = (d[0] >> 4) & 0x0F
        pos_u = (d[1] << 8) | d[2]
        vel_u = (d[3] << 4) | (d[4] >> 4)
        tau_u = ((d[4] & 0x0F) << 8) | d[5]
    else:
        # 参数/反馈回复可能使用 Master ID 作为封装 CAN ID；
        # 官方库把 Master ID 也加入 motors_map，因此这里不能只接受 Slave ID。
        d = data
        state = d[0] & 0x0F
        pos_u = (d[1] << 8) | d[2]
        vel_u = (d[3] << 4) | (d[4] >> 4)
        tau_u = ((d[4] & 0x0F) << 8) | d[5]
    return Feedback(
        can_id=can_id,
        state=state,
        position=uint_to_float(pos_u, -PMAX_DEFAULT, PMAX_DEFAULT, 16),
        velocity=uint_to_float(vel_u, -VMAX_DEFAULT, VMAX_DEFAULT, 12),
        torque=uint_to_float(tau_u, -TMAX_DEFAULT, TMAX_DEFAULT, 12),
        mos_temp=data[6],
        coil_temp=data[7],
    )


class OfficialSerialMotorControl:
    """达妙官方 DM_CAN.py 的串口封装核心，保留原始帧格式。"""

    def __init__(self, port: str, baudrate: int = SERIAL_BAUD_DEFAULT, timeout: float = 0.15):
        self.port = port
        self.serial = serial.Serial(port=port, baudrate=baudrate, timeout=timeout)
        # 与官方 send_data_frame 一致：55 AA 1E 03 01 00 00 00 0A 00 00 00 00
        # [CAN ID little-endian 两字节] + 6 个保留字节 + 8 字节 CAN data + 55。
        self.send_data_frame = bytearray(
            (0x55, 0xAA, 0x1E, 0x03, 0x01, 0x00, 0x00, 0x00,
             0x0A, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x08, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
             0x00, 0x00, 0x00, 0x00, 0x00)
        )
        self._rx_remainder = b""

    def close(self) -> None:
        if self.serial.is_open:
            self.serial.close()

    def _send_data(self, can_id: int, data: bytes) -> None:
        if len(data) != 8:
            raise ValueError("CAN data 必须正好 8 字节")
        frame = bytearray(self.send_data_frame)
        frame[13] = can_id & 0xFF
        frame[14] = (can_id >> 8) & 0xFF
        frame[21:29] = data
        self.serial.write(frame)
        self.serial.flush()

    def _extract_packets(self) -> List[bytes]:
        data = self._rx_remainder + self.serial.read_all()
        frames: List[bytes] = []
        i = 0
        while i <= len(data) - 16:
            if data[i] == 0xAA and data[i + 15] == 0x55:
                frames.append(bytes(data[i:i + 16]))
                i += 16
            else:
                i += 1
        self._rx_remainder = data[i:]
        return frames

    def receive(self, motor_id: int, timeout: float = 0.25) -> List[Feedback]:
        deadline = time.monotonic() + timeout
        out: List[Feedback] = []
        while time.monotonic() < deadline:
            for packet in self._extract_packets():
                f = decode_feedback(packet, motor_id)
                if f is not None:
                    out.append(f)
            if out:
                break
            time.sleep(0.005)
        return out

    def send_command(self, motor_id: int, cmd: int, repeats: int = 3) -> None:
        data = b"\xFF" * 7 + bytes((cmd & 0xFF,))
        for _ in range(max(1, repeats)):
            self._send_data(motor_id, data)
            time.sleep(0.02)

    def enable(self, motor_id: int, mode: int, legacy: bool = True) -> None:
        # 新固件官方 enable：ID=motor_id，末字节 FC。
        self.send_command(motor_id, 0xFC, repeats=3)
        # 官方兼容旧固件：((ControlMode-1)<<2)+SlaveID。
        # 默认保留，解决“FC 已发但旧固件仍不使能”的情况。
        if legacy:
            legacy_id = ((int(mode) - 1) << 2) + motor_id
            if legacy_id != motor_id:
                self.send_command(legacy_id, 0xFC, repeats=2)

    def disable(self, motor_id: int) -> None:
        self.send_command(motor_id, 0xFD, repeats=3)

    def set_zero(self, motor_id: int) -> None:
        self.send_command(motor_id, 0xFE, repeats=3)

    def read_param(self, motor_id: int, rid: int, timeout: float = 0.5) -> Optional[float]:
        # 官方 __read_RID_param：向 0x7FF 发送 [slave_lo, slave_hi, 0x33, RID, 0,0,0,0]。
        data = bytes((motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0x33, rid & 0xFF, 0, 0, 0, 0))
        self._send_data(0x7FF, data)
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for packet in self._extract_packets():
                if len(packet) != 16 or packet[0] != 0xAA or packet[15] != 0x55 or packet[1] != 0x11:
                    continue
                can_id = int.from_bytes(packet[3:7], "little")
                d = packet[7:15]
                # 回复 CAN ID 可能是 Slave ID、Master ID 或 0；以数据中的
                # slave_id 字段为准，和官方 __process_set_param_packet 一致。
                if int.from_bytes(d[0:2], "little") != motor_id:
                    continue
                if d[2] != 0x33 or d[3] != (rid & 0xFF):
                    continue
                # RID 7-10,13-16,35-36 是 uint32，其余是 float。
                if 7 <= rid <= 10 or 13 <= rid <= 16 or 35 <= rid <= 36:
                    return float(int.from_bytes(d[4:8], "little"))
                return float(struct.unpack("<f", d[4:8])[0])
            time.sleep(0.005)
        return None

    def write_param(self, motor_id: int, rid: int, value: float | int) -> None:
        if 7 <= rid <= 10 or 13 <= rid <= 16 or 35 <= rid <= 36:
            raw = struct.pack("<I", int(value))
        else:
            raw = struct.pack("<f", float(value))
        data = bytes((motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0x55, rid & 0xFF)) + raw
        self._send_data(0x7FF, data)

    def switch_mode(self, motor_id: int, mode: int, timeout: float = 0.8) -> bool:
        self.write_param(motor_id, 10, mode)
        return self.read_param(motor_id, 10, timeout=timeout) == float(mode)

    def control(self, motor_id: int, mode: int, position: float, speed: float, kp: float, kd: float, torque: float, feedback_timeout: float = 0.15) -> List[Feedback]:
        if mode == CTRL_MODE_MIT:
            can_id, payload = motor_id, pack_mit(position, speed, kp, kd, torque)
        elif mode == CTRL_MODE_POS_VEL:
            can_id, payload = 0x100 + motor_id, pack_pos_vel(position, speed)
        elif mode == CTRL_MODE_VEL:
            can_id, payload = 0x200 + motor_id, pack_vel(speed)
        else:
            raise ValueError(f"不支持控制模式: {mode}")
        self._send_data(can_id, payload)
        # 达妙是“一发一收”，控制帧后读取反馈。
        return self.receive(motor_id, timeout=feedback_timeout)


class DmDeviceMotorControl:
    """达妙官方 dm-device-sdk 的 USB2FDCAN/USB2CANFD 传输后端。"""

    def __init__(self, channel: int = 0, can_baudrate: int = 1_000_000):
        try:
            import dmcan
            from dmcan import DmCanContext
        except Exception as exc:
            raise RuntimeError("未安装达妙 dmcan SDK（dmcan_sdk）") from exc

        self.channel = channel
        self._rx = deque(maxlen=512)
        self._rx_lock = threading.Lock()
        self._dll_dir_handle = None
        dll_dir = Path(dmcan.__file__).resolve().parent / "dlls"
        if dll_dir.exists():
            # 官方 Python 包的 DLL 依赖 libusb/libstdc++；Windows 下需要把
            # SDK 的 dlls 目录加入搜索路径后再创建 DmCanContext。
            os.environ["PATH"] = str(dll_dir) + os.pathsep + os.environ.get("PATH", "")
            if hasattr(os, "add_dll_directory"):
                self._dll_dir_handle = os.add_dll_directory(str(dll_dir))

        self.context = DmCanContext()
        count = self.context.find_devices()
        if count <= 0:
            self.context._ctx = None
            raise RuntimeError("达妙 dm-device SDK 未发现 USB2FDCAN 设备")
        self.device = self.context.get_device(0)
        if not self.device.open():
            self.context._ctx = None
            raise RuntimeError("达妙 USB2FDCAN 打开失败")
        self.device.enable_channel(channel, True)
        info = self.device.get_channel_baudrate(channel)
        if info is not None:
            # DM-G6220 使用 Classic CAN 1 Mbps；只在配置不一致时改当前会话，
            # 不写入电机永久参数。
            if bool(info.canfd) or int(info.can_baudrate) != can_baudrate:
                info.canfd = False
                info.can_baudrate = can_baudrate
                if not self.device.set_channel_baudrate(channel, info):
                    raise RuntimeError("USB2FDCAN 设置 Classic CAN 1 Mbps 失败")
        self.device.hook_recv_callback(self._on_recv)
        print(f"[INFO] 达妙 dm-device SDK 已打开：设备索引=0，CAN 通道={channel}，速率={can_baudrate}")

    @staticmethod
    def _dlc_bytes(dlc: int) -> int:
        return dlc if dlc <= 8 else {9: 12, 10: 16, 11: 20, 12: 24, 13: 32, 14: 48, 15: 64}.get(dlc, 64)

    def _on_recv(self, _device, frame) -> None:
        try:
            # SDK 的 dir=0 是 RX，dir=1 是本机 TX 回显；只解析 RX。
            if int(frame.head.dir) != 0 or int(frame.head.channel) != self.channel:
                return
            length = self._dlc_bytes(int(frame.head.dlc))
            payload = bytes(frame.payload[:length])
            with self._rx_lock:
                self._rx.append((int(frame.head.can_id), payload, bool(frame.head.ext), bool(frame.head.canfd)))
        except Exception:
            return

    def _pop_frames(self) -> List[Tuple[int, bytes, bool, bool]]:
        with self._rx_lock:
            frames = list(self._rx)
            self._rx.clear()
        return frames

    def _send_data(self, can_id: int, data: bytes) -> bool:
        return bool(self.device.send_can(self.channel, can_id, len(data), data, False, False, False, False))

    def receive(self, motor_id: int, timeout: float = 0.25) -> List[Feedback]:
        deadline = time.monotonic() + timeout
        out: List[Feedback] = []
        while time.monotonic() < deadline:
            for can_id, data, is_ext, is_fd in self._pop_frames():
                if is_ext or is_fd or len(data) < 6:
                    continue
                # DM-CAN 反馈：首字节高 4 位状态、低 4 位电机 ID。
                if (data[0] & 0x0F) != (motor_id & 0x0F):
                    continue
                pos_u = (data[1] << 8) | data[2]
                vel_u = (data[3] << 4) | (data[4] >> 4)
                tau_u = ((data[4] & 0x0F) << 8) | data[5]
                out.append(Feedback(
                    can_id=can_id,
                    state=(data[0] >> 4) & 0x0F,
                    position=uint_to_float(pos_u, -PMAX_DEFAULT, PMAX_DEFAULT, 16),
                    velocity=uint_to_float(vel_u, -VMAX_DEFAULT, VMAX_DEFAULT, 12),
                    torque=uint_to_float(tau_u, -TMAX_DEFAULT, TMAX_DEFAULT, 12),
                    mos_temp=data[6] if len(data) > 6 else 0,
                    coil_temp=data[7] if len(data) > 7 else 0,
                ))
            if out:
                break
            time.sleep(0.002)
        return out

    def send_command(self, motor_id: int, cmd: int, repeats: int = 3) -> None:
        data = b"\xFF" * 7 + bytes((cmd & 0xFF,))
        for _ in range(max(1, repeats)):
            if not self._send_data(motor_id, data):
                raise RuntimeError(f"CAN 帧发送失败: id=0x{motor_id:X}")
            time.sleep(0.02)

    def enable(self, motor_id: int, mode: int, legacy: bool = True) -> None:
        self.send_command(motor_id, 0xFC, repeats=3)
        if legacy:
            legacy_id = ((int(mode) - 1) << 2) + motor_id
            if legacy_id != motor_id:
                self.send_command(legacy_id, 0xFC, repeats=2)

    def disable(self, motor_id: int) -> None:
        self.send_command(motor_id, 0xFD, repeats=3)

    def read_param(self, motor_id: int, rid: int, timeout: float = 0.5) -> Optional[float]:
        with self._rx_lock:
            self._rx.clear()
        request = bytes((motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0x33, rid & 0xFF, 0, 0, 0, 0))
        if not self._send_data(0x7FF, request):
            return None
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for _can_id, data, is_ext, is_fd in self._pop_frames():
                if is_ext or is_fd or len(data) != 8:
                    continue
                if int.from_bytes(data[0:2], "little") != motor_id or data[2] != 0x33 or data[3] != (rid & 0xFF):
                    continue
                if 7 <= rid <= 10 or 13 <= rid <= 16 or 35 <= rid <= 36:
                    return float(int.from_bytes(data[4:8], "little"))
                return float(struct.unpack("<f", data[4:8])[0])
            time.sleep(0.002)
        return None

    def write_param(self, motor_id: int, rid: int, value: float | int) -> None:
        if 7 <= rid <= 10 or 13 <= rid <= 16 or 35 <= rid <= 36:
            raw = struct.pack("<I", int(value))
        else:
            raw = struct.pack("<f", float(value))
        data = bytes((motor_id & 0xFF, (motor_id >> 8) & 0xFF, 0x55, rid & 0xFF)) + raw
        if not self._send_data(0x7FF, data):
            raise RuntimeError("CAN 参数写入帧发送失败")

    def switch_mode(self, motor_id: int, mode: int, timeout: float = 0.8) -> bool:
        self.write_param(motor_id, 10, mode)
        return self.read_param(motor_id, 10, timeout=timeout) == float(mode)

    def control(self, motor_id: int, mode: int, position: float, speed: float, kp: float, kd: float, torque: float, feedback_timeout: float = 0.15) -> List[Feedback]:
        if mode == CTRL_MODE_MIT:
            can_id, payload = motor_id, pack_mit(position, speed, kp, kd, torque)
        elif mode == CTRL_MODE_POS_VEL:
            can_id, payload = 0x100 + motor_id, pack_pos_vel(position, speed)
        elif mode == CTRL_MODE_VEL:
            can_id, payload = 0x200 + motor_id, pack_vel(speed)
        else:
            raise ValueError(f"不支持控制模式: {mode}")
        if not self._send_data(can_id, payload):
            raise RuntimeError(f"控制帧发送失败: id=0x{can_id:X}")
        return self.receive(motor_id, timeout=feedback_timeout)

    def close(self) -> None:
        try:
            self.device.enable_channel(self.channel, False)
        except Exception:
            pass
        try:
            self.device.close()
        except Exception:
            pass
        # 官方 SDK 1.1.0 在 Windows 上 context.destroy 偶发等待 libusb 线程；
        # 设备已关闭后令析构器跳过重复 destroy，避免脚本退出卡住。
        try:
            self.context._ctx = None
        except Exception:
            pass


def port_candidates() -> List[str]:
    return [p.device for p in list_ports.comports()]


def is_unitree_quad_rs485(port: str) -> bool:
    """识别本机已确认的 FT4232H 四路 Unitree RS485 设备。"""
    for info in list_ports.comports():
        if info.device != port:
            continue
        serial_number = (info.serial_number or "").upper()
        return info.vid == 0x0403 and info.pid == 0x6011 and serial_number.startswith("FTA9GL5L")
    return False


def scan_ports(motor_id: int, baudrate: int, probe_all: bool = False) -> List[Tuple[int, str, str]]:
    """只读查询 CTRL_MODE/PMAX，用于判断哪个 USB 模块在线。"""
    result: List[Tuple[int, str, str]] = []
    ports = port_candidates()
    if not ports:
        return result
    print(f"[SCAN] 发现串口: {', '.join(ports)}")
    for port in ports:
        port_info = next((item for item in list_ports.comports() if item.device == port), None)
        # 蓝牙 SPP 端口不是电机总线，部分 Windows 蓝牙驱动的 read_all()
        # 会长时间阻塞；直接标记并跳过，避免扫描卡住。
        if port_info is not None and port_info.vid is None and port_info.pid is None:
            detail = "Bluetooth SPP，不是 USB-CAN，已跳过"
            result.append((-100, port, detail))
            print(f"[SCAN] {port}: {detail}")
            continue
        if is_unitree_quad_rs485(port) and not probe_all:
            detail = "FT4232H 四路 Unitree RS485；不是 CAN/达妙 USB-CAN，已跳过运动协议探测"
            result.append((-50, port, detail))
            print(f"[SCAN] {port}: {detail}")
            continue
        try:
            ctl = OfficialSerialMotorControl(port, baudrate=baudrate, timeout=0.08)
            mode = ctl.read_param(motor_id, 10, timeout=0.35)
            pmax = ctl.read_param(motor_id, 21, timeout=0.35)
            ctl.close()
            if mode is not None or pmax is not None:
                detail = f"在线: CTRL_MODE={int(mode) if mode is not None else '?'} PMAX={pmax if pmax is not None else '?'}"
                result.append((100, port, detail))
                print(f"[SCAN] {port}: {detail}")
            else:
                result.append((0, port, "可打开但未收到目标电机应答"))
                print(f"[SCAN] {port}: 未收到 ID={motor_id} 的应答")
        except Exception as exc:
            result.append((-100, port, f"打开/查询失败: {exc}"))
            print(f"[SCAN] {port}: 失败: {exc}")
    result.sort(key=lambda item: item[0], reverse=True)
    return result


def parse_int(text: str) -> int:
    text = text.strip().lower()
    return int(text, 16) if text.startswith("0x") else int(text, 10)


def parse_mode(text: str) -> Optional[int]:
    text = text.lower()
    if text == "auto":
        return None
    return {"mit": CTRL_MODE_MIT, "pos_vel": CTRL_MODE_POS_VEL, "vel": CTRL_MODE_VEL}[text]


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="按达妙官方 DM_CAN.py 串口封装直接控制 DM-G6220")
    p.add_argument("--motor-id", default="1", help="电机 Slave/CAN ID，十进制或 0x 十六进制")
    p.add_argument("--port", "--channel", dest="port", default="", help="COM 端口；留空自动扫描")
    p.add_argument("--transport", choices=("auto", "dm-device", "serial"), default="auto", help="auto 优先使用达妙 dm-device SDK")
    p.add_argument("--can-channel", type=int, default=0, help="USB2FDCAN 的 SDK 通道，单路模块通常为 0")
    p.add_argument("--baudrate", type=int, default=SERIAL_BAUD_DEFAULT)
    p.add_argument("--mode", choices=("auto", "mit", "pos_vel", "vel"), default="pos_vel")
    p.add_argument("--switch-mode", dest="switch_mode", action="store_true", default=True, help="控制前临时切换为 --mode（默认开启，不保存到 flash）")
    p.add_argument("--no-switch-mode", dest="switch_mode", action="store_false", help="不自动切换电机控制模式")
    p.add_argument("--no-legacy-enable", action="store_true", help="不发送旧固件兼容使能帧")
    p.add_argument("--repeat", type=int, default=1, help="往返轮数；0 表示持续")
    p.add_argument("--amplitude", type=float, default=100.0, help="往复角度（度）")
    p.add_argument("--speed", type=float, default=0.5, help="速度上限 rad/s；POS_VEL 为 v_des，MIT 仅用于计时")
    p.add_argument("--leg-time-limit", type=float, default=0.0,
                   help="单段转动最长时间（秒）；>0 时自动提高 POS_VEL 速度以满足该上限")
    p.add_argument("--kp", type=float, default=10.0)
    p.add_argument("--kd", type=float, default=2.0)
    p.add_argument("--torque", type=float, default=0.0)
    p.add_argument("--dwell", type=float, default=0.25)
    p.add_argument("--disable-end", action="store_true")
    p.add_argument("--scan-only", action="store_true", help="只读扫描端口，不使能、不运动")
    p.add_argument("--probe-all", action="store_true", help="强制向所有串口发送一次官方只读查询（含疑似 Unitree RS485）")
    p.add_argument("--force-unitree-port", action="store_true", help="强行向疑似 Unitree RS485 端口发帧（不建议；默认阻止）")
    return p.parse_args()


def run(args: argparse.Namespace) -> int:
    motor_id = parse_int(args.motor_id)
    requested_mode = parse_mode(args.mode)
    ctl = None
    selected_port = ""
    transport_name = ""

    # DM-USB2FDCAN 的 CAN 通道走 WinUSB，不使用 COM9；auto 优先调用达妙
    # 官方 dm-device SDK。只有 SDK 不可用时才回退到旧式串口桥。
    if args.transport in ("auto", "dm-device"):
        try:
            ctl = DmDeviceMotorControl(channel=args.can_channel, can_baudrate=1_000_000)
            transport_name = "dm-device"
            selected_port = f"DM-USB2FDCAN(channel={args.can_channel})"
            if args.scan_only:
                mode = ctl.read_param(motor_id, 10, timeout=0.8)
                pmax = ctl.read_param(motor_id, 21, timeout=0.8)
                print(f"[SCAN] {selected_port}: motor_id={motor_id} CTRL_MODE={mode} PMAX={pmax}")
                # 官方 SDK 1.1.0 注册回调后，Windows libusb 接收线程偶发不退出；
                # 扫描不发送运动帧，直接结束本工具进程，让 Windows 回收
                # WinUSB 句柄，避免 device.close() 阻塞并占住下一次控制。
                sys.stdout.flush()
                os._exit(0 if mode is not None else 2)
        except Exception as exc:
            if ctl is not None:
                try:
                    ctl.close()
                except Exception:
                    pass
            ctl = None
            if args.transport == "dm-device":
                print(f"[ERROR] 达妙 dm-device SDK 打开失败: {exc}")
                return 2
            print(f"[WARN] dm-device SDK 不可用，回退串口桥: {exc}")

    if ctl is None:
        if args.port:
            selected_port = args.port
            if args.scan_only:
                probe = scan_ports(motor_id, args.baudrate, probe_all=args.probe_all)
                selected = [item for item in probe if item[1].upper() == selected_port.upper()]
                return 0 if selected and selected[0][0] > 0 else 2
        else:
            candidates = scan_ports(motor_id, args.baudrate, probe_all=args.probe_all)
            online = [item for item in candidates if item[0] > 0]
            if args.scan_only:
                return 0 if online else 2
            if not online:
                print("[ERROR] 没有端口收到目标电机应答；未发送运动命令。")
                return 2
            selected_port = online[0][1]
            print(f"[INFO] 自动选择在线串口桥: {selected_port}")

        if is_unitree_quad_rs485(selected_port) and not args.force_unitree_port:
            print(f"[ERROR] {selected_port} 是 Unitree FT4232H RS485，不是 DM-G6220 CAN 适配器；已阻止发送运动帧。")
            print("[ACTION] 请接入达妙 USB-CAN/DM-MC02（CAN_H/CAN_L，1 Mbps）后重试。")
            return 3
        ctl = OfficialSerialMotorControl(selected_port, baudrate=args.baudrate)
        transport_name = "serial-bridge"

    effective_mode = requested_mode
    run_ok = False
    try:
        if effective_mode is None:
            mode_value = ctl.read_param(motor_id, 10, timeout=0.6)
            if mode_value is not None and int(mode_value) in (1, 2, 3):
                effective_mode = int(mode_value)
                print(f"[INFO] 读取到电机 CTRL_MODE={effective_mode}")
            else:
                effective_mode = CTRL_MODE_MIT
                print("[WARN] 未读到 CTRL_MODE，按官方默认 MIT 发送")

        if args.switch_mode and requested_mode is not None:
            if ctl.switch_mode(motor_id, effective_mode):
                print(f"[INFO] 临时切换 CTRL_MODE={effective_mode} 成功")
            else:
                raise RuntimeError(f"临时切换 CTRL_MODE={effective_mode} 未收到确认，已停止运动")

        print(f"[INFO] 传输={transport_name} 端口={selected_port} motor_id={motor_id} mode={effective_mode}")
        print("[INFO] 按官方协议发送 FC 使能")
        ctl.enable(motor_id, effective_mode, legacy=not args.no_legacy_enable)
        feedback = ctl.receive(motor_id, timeout=0.5)
        if feedback:
            f = feedback[-1]
            print(f"[FB] state={status_text(f.state)} pos={f.position:.4f} vel={f.velocity:.4f} tau={f.torque:.4f}")
            # 首段运动时间从使能后的实际位置计算，避免电机上次停在非零角度
            # 时仍按零点估算，导致第一段提前结束。
            initial_position = f.position
        else:
            print("[WARN] 使能后没有收到反馈；仍发送一次控制帧以验证链路")
            initial_position = 0.0

        amplitude = math.radians(args.amplitude)
        if amplitude <= 0:
            raise ValueError("--amplitude 必须为正数")
        # 官方位置-速度模式的 p_des 是“绝对位置”（单位 rad），不是
        # 相对角度。若直接在 +pi 与 -pi 之间切换，反向段会跨越 2*pi
        # （360°）。这里以使能后的实际位置为基准，每一段只增加/减少
        # 一个 amplitude，确保用户要求的“每次转动”是相对位移。
        direction = 1.0
        cycle = 0
        previous_target = initial_position
        control_period = 0.02  # 官方例程建议 1~2 ms；PC USB 调试先用 50 Hz
        while args.repeat == 0 or cycle < args.repeat:
            for index in range(2):
                # 每一段从上一段的绝对目标位置偏移 ±amplitude；这样
                # “正转 180° / 反转 180°”两段各自都是 180°，不会在
                # +180° 与 -180°绝对坐标之间误走 360°。
                target = previous_target + direction * amplitude
                # MIT/POS_VEL 都需要持续刷新目标；只发一帧会触发通信超时，
                # 也是之前“只正转、反转看不到”的主要原因。
                motion_distance = abs(target - previous_target)
                requested_speed = max(args.speed, 0.1)
                settle_window = max(args.dwell, 0.0)
                leg_speed = requested_speed
                if args.leg_time_limit > 0:
                    # 把 dwell 也计入单段总时长，确保“每次转动”不会超过
                    # 用户给定的上限；速度只在必要时提高，不降低用户设定值。
                    available_motion = max(args.leg_time_limit - settle_window, 0.1)
                    leg_speed = max(requested_speed, motion_distance / available_motion)
                    motion_window = min(motion_distance / leg_speed, available_motion)
                else:
                    motion_window = motion_distance / leg_speed
                move_window = motion_window + settle_window
                deadline = time.monotonic() + move_window
                frame_count = 0
                last_feedback: Optional[Feedback] = None
                while time.monotonic() < deadline:
                    tick = time.monotonic()
                    elapsed = move_window - max(0.0, deadline - time.monotonic())
                    # 位置速度模式的最后 dwell 段将速度上限置零，只保持目标，
                    # 防止到位后继续以非零速度指令造成抽动。MIT 模式始终用
                    # dq=0，由 Kd 提供阻尼；args.speed 只用于估算运动窗口。
                    if effective_mode == CTRL_MODE_POS_VEL:
                        command_speed = leg_speed if elapsed < motion_window else 0.0
                    elif effective_mode == CTRL_MODE_MIT:
                        command_speed = 0.0
                    else:
                        command_speed = leg_speed
                    feedback = ctl.control(
                        motor_id, effective_mode, target, command_speed, args.kp, args.kd, args.torque,
                        feedback_timeout=min(0.012, control_period * 0.6),
                    )
                    frame_count += 1
                    if feedback:
                        last_feedback = feedback[-1]
                    remain = control_period - (time.monotonic() - tick)
                    if remain > 0:
                        time.sleep(remain)
                print(f"[CMD] 往返第 {cycle + 1} 轮 {'正转' if index == 0 else '反转'} "
                      f"target={math.degrees(target):.1f}° ({target:.4f} rad) "
                      f"leg_speed={leg_speed:.3f}rad/s duration={move_window:.2f}s "
                      f"continuous_frames={frame_count} "
                      f"frame_id=0x{(motor_id if effective_mode == 1 else (0x100 + motor_id if effective_mode == 2 else 0x200 + motor_id)):03X}")
                if last_feedback:
                    print(f"[FB] state={status_text(last_feedback.state)} pos={last_feedback.position:.4f} "
                          f"vel={last_feedback.velocity:.4f} tau={last_feedback.torque:.4f}")
                else:
                    print("[WARN] 控制帧已连续写入，但没有收到反馈")
                previous_target = target
                direction *= -1.0
            cycle += 1
        run_ok = True
    except KeyboardInterrupt:
        print("[INFO] 收到 Ctrl+C，停止往返")
        run_ok = True
    except Exception as exc:
        print(f"[ERROR] 电机控制失败: {exc}")
    finally:
        try:
            if args.disable_end:
                ctl.disable(motor_id)
                print("[INFO] 已发送 FD 失能")
        except Exception as exc:
            print(f"[WARN] 发送 FD 失能失败: {exc}")
            run_ok = False
        if transport_name == "dm-device":
            # dm-device SDK 注册接收回调后，device.close() 可能等待 libusb
            # 后台线程很久；FD 已发送后直接结束本工具进程，Windows 会回收
            # WinUSB 句柄，避免用户看到脚本“再次执行但不退出”。
            sys.stdout.flush()
            sys.stderr.flush()
            os._exit(0 if run_ok else 2)
        try:
            ctl.close()
        except Exception as exc:
            print(f"[WARN] 关闭传输失败: {exc}")
    return 0 if run_ok else 2


if __name__ == "__main__":
    raise SystemExit(run(parse_args()))

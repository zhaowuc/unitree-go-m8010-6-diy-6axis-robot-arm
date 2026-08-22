"""V15.30A-FT 六自由度机械臂全中文 PySide6 主窗口。"""

from __future__ import annotations

import csv
import json
import math
import sys
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
import yaml

from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QCloseEvent, QFont
from PySide6.QtWidgets import (
    QApplication, QDoubleSpinBox, QGridLayout, QGroupBox, QHBoxLayout,
    QLabel, QMainWindow, QMessageBox, QPushButton, QSlider, QVBoxLayout,
    QWidget,
)

from .state_machine import ArmMode, ArrivalTracker, MODE_TEXT, ModeMachine


JOINT_NAMES = tuple(f"joint{index}" for index in range(1, 7))
JOINT_LABELS = tuple(f"关节{index} J{index}" for index in range(1, 7))
DEG = 180.0 / math.pi
RAD = math.pi / 180.0
CONTROL_STREAM_TIMEOUT_S = 0.5
MUJOCO_STREAM_TIMEOUT_S = 0.5
TARGET_SELECTION_DEADBAND_RAD = math.radians(0.01)


def receipt_is_fresh(receipt: float, now: float, timeout_s: float) -> bool:
    age = now - receipt
    return receipt > 0.0 and 0.0 <= age <= timeout_s


def mujoco_status_text(status: Optional[dict], fresh: bool) -> str:
    if status is None:
        return "等待"
    if not fresh:
        return "数据过期"
    result = status.get("direct_qpos_numeric_result")
    if result == "PASS":
        return "正常"
    if result == "NOT_EVALUATED":
        return "未验证"
    return "异常"


def control_feedback_ready(
    have_first_state: bool,
    streams_fresh: bool,
    connected: list[bool],
    require_all: bool = False,
) -> bool:
    """Return whether an action has enough fresh, healthy joint feedback.

    Normal motion actions are isolated per joint by the hardware workers, so one
    healthy logical joint is sufficient.  Operations that save or command a
    complete six-joint pose explicitly opt into the all-joints requirement.
    """
    if not have_first_state or not streams_fresh or not connected:
        return False
    return all(connected) if require_all else any(connected)


def effective_active_joint_mask(
    requested: list[bool], connected: list[bool], mode: str
) -> list[bool]:
    """Return the per-joint activation mask sent to the hardware workers.

    V15.30A keeps J2 active control fail-closed.  A disconnected joint is also
    removed from every active command before publication.  Brake never carries
    an active bit.
    """
    if len(requested) != 6 or len(connected) != 6:
        raise ValueError("关节激活掩码必须包含六项")
    if mode == "brake":
        return [False] * 6
    return [
        bool(requested[index] and connected[index] and index != 1)
        for index in range(6)
    ]


def clear_mask_on_connection_loss(
    requested: list[bool], previous_connected: list[bool], connected: list[bool]
) -> list[bool]:
    """Latch an active request off after feedback health is lost.

    A later feedback recovery must never re-enable a joint without a fresh
    operator action.
    """
    if len(requested) != 6 or len(previous_connected) != 6 or len(connected) != 6:
        raise ValueError("关节连接状态必须包含六项")
    return [
        bool(requested[index] and not (
            previous_connected[index] and not connected[index]
        ))
        for index in range(6)
    ]


def run_ros_context_operation(context_ok, operation) -> bool:
    """Run one ROS operation, distinguishing shutdown from runtime failure.

    A ROS shutdown can race with either the preflight check or the operation
    itself.  Only an exception accompanied by an invalid context is treated as
    normal shutdown; exceptions while the context remains valid are re-raised.
    """
    if not context_ok():
        return False
    try:
        operation()
    except Exception:
        if context_ok():
            raise
        return False
    return context_ok()


@dataclass
class VirtualWidgets:
    slider: QSlider
    value: QLabel
    target: QDoubleSpinBox


@dataclass
class RealWidgets:
    slider: QSlider
    actual: QLabel
    target: QLabel
    error: QLabel
    state: QLabel


class ArmGuiNode(Node):
    def __init__(self) -> None:
        super().__init__("arm_control_gui")
        self.declare_parameter("config_path", "")
        self.declare_parameter("joint_limits_path", "")
        self.declare_parameter("initial_pose_path", "")
        self.declare_parameter("log_directory", "logs/arm_gui")
        self.config_path = Path(str(self.get_parameter("config_path").value)).resolve()
        self.joint_limits_path = Path(
            str(self.get_parameter("joint_limits_path").value)
        ).resolve()
        self.initial_pose_path = Path(str(self.get_parameter("initial_pose_path").value)).resolve()
        self.log_directory = Path(str(self.get_parameter("log_directory").value)).resolve()
        self.command_publisher = self.create_publisher(String, "/whole_arm/gui_command", 10)
        self.target_publisher = self.create_publisher(Float64MultiArray, "/whole_arm/gui_targets", 10)
        self.mode_publisher = self.create_publisher(String, "/whole_arm/gui_mode", 10)
        self.latest_joint_state: Optional[JointState] = None
        self.latest_hardware: Optional[dict] = None
        self.latest_mujoco: Optional[dict] = None
        self.last_joint_receipt = 0.0
        self.last_hardware_receipt = 0.0
        self.last_mujoco_receipt = 0.0
        self.create_subscription(JointState, "/joint_states", self._joint_callback, 10)
        self.create_subscription(String, "/whole_arm/hardware_state", self._hardware_callback, 10)
        self.create_subscription(String, "/whole_arm/mujoco_mirror_status", self._mujoco_callback, 10)

    def _joint_callback(self, message: JointState) -> None:
        if tuple(message.name) == JOINT_NAMES and len(message.position) == 6:
            self.latest_joint_state = message
            self.last_joint_receipt = time.monotonic()

    def _hardware_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                return
            self.latest_hardware = value
            self.last_hardware_receipt = time.monotonic()
        except json.JSONDecodeError:
            pass

    def _mujoco_callback(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            if not isinstance(value, dict):
                return
            self.latest_mujoco = value
            self.last_mujoco_receipt = time.monotonic()
        except json.JSONDecodeError:
            pass

    def control_streams_fresh(self, now: Optional[float] = None) -> bool:
        checked_at = time.monotonic() if now is None else now
        return receipt_is_fresh(
            self.last_joint_receipt, checked_at, CONTROL_STREAM_TIMEOUT_S
        ) and receipt_is_fresh(
            self.last_hardware_receipt, checked_at, CONTROL_STREAM_TIMEOUT_S
        )

    def mujoco_stream_fresh(self, now: Optional[float] = None) -> bool:
        checked_at = time.monotonic() if now is None else now
        return receipt_is_fresh(
            self.last_mujoco_receipt, checked_at, MUJOCO_STREAM_TIMEOUT_S
        )

    def publish_command(
        self, sequence: int, mode: str, command_targets: list[float],
        virtual_targets: list[float], active_joint_mask: list[bool],
        activation_epoch: int, config: dict
    ) -> None:
        control = config["控制"]
        payload = {
            "schema": "go-m8010-gui-command/1.1",
            "sequence": sequence,
            "mode": mode,
            "targets_rad": command_targets,
            "active_joint_mask": active_joint_mask,
            "activation_epoch": activation_epoch,
            "maximum_velocity_rad_s": float(control["最大速度_度每秒"]) * RAD,
            "maximum_acceleration_rad_s2": float(control["最大加速度_度每二次方秒"]) * RAD,
            "kp": [float(config["关节"][f"J{i + 1}"].get("Kp", 0.0)) for i in range(6)],
            "kd": [float(config["关节"][f"J{i + 1}"].get("Kd", 0.0)) for i in range(6)],
        }
        self.command_publisher.publish(String(data=json.dumps(payload, ensure_ascii=False)))
        self.target_publisher.publish(Float64MultiArray(data=virtual_targets))


class MainWindow(QMainWindow):
    def __init__(self, node: ArmGuiNode) -> None:
        super().__init__()
        self.node = node
        self.config = yaml.safe_load(node.config_path.read_text(encoding="utf-8"))
        limits_document = yaml.safe_load(node.joint_limits_path.read_text(encoding="utf-8"))
        if limits_document.get("单位") != "度" or limits_document.get("参考") != "SESSION_REFERENCE_V1":
            raise ValueError("关节限位文件语义不匹配")
        self.limits = []
        for index in range(6):
            bounds = [float(value) for value in limits_document[f"J{index + 1}"]]
            if len(bounds) != 2 or not all(math.isfinite(value) for value in bounds) or bounds[0] >= bounds[1]:
                raise ValueError("关节限位文件无效")
            self.limits.append((bounds[0], bounds[1]))
        control = self.config["控制"]
        self.machine = ModeMachine()
        self.direction = ArmMode.REAL_TO_SIM
        self.hardware_mode = "brake"
        self.targets = [0.0] * 6
        self.command_targets = [0.0] * 6
        self.requested_active_joint_mask = [False] * 6
        self.pending_target_joint_mask = [False] * 6
        self.activation_epoch = max(1, time.monotonic_ns() & ((1 << 63) - 1))
        self.actual = [0.0] * 6
        self.connected = [False] * 6
        self.faulted = [False] * 6
        self.have_first_state = False
        self.session_id: Optional[str] = None
        self.command_sequence = 0
        self.last_log_at = 0.0
        self.arrival = ArrivalTracker(
            tolerance_rad=float(control["到位容差_度"]) * RAD,
            dwell_s=float(control["到位持续_秒"]),
            timeout_s=float(control["目标超时_秒"]),
        )
        self.virtual_widgets: list[VirtualWidgets] = []
        self.real_widgets: list[RealWidgets] = []
        self.setWindowTitle("六自由度机械臂控制系统")
        self.resize(1220, 860)
        self._build_ui()
        self._open_log()

        refresh_ms = round(1000.0 / float(self.config["界面"]["刷新频率_赫兹"]))
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._tick)
        self.timer.start(refresh_ms)
        self.statusBar().showMessage("正在等待六关节编码器；当前为会话相对角度")

    def _build_ui(self) -> None:
        root = QWidget()
        outer = QVBoxLayout(root)
        title = QLabel("六自由度机械臂控制系统")
        title.setAlignment(Qt.AlignCenter)
        title.setFont(QFont("Sans Serif", 18, QFont.Bold))
        subtitle = QLabel("当前为会话相对角度　｜　计算机辅助设计零位：待定　｜　机器人系统零位：待定")
        subtitle.setAlignment(Qt.AlignCenter)
        outer.addWidget(title)
        outer.addWidget(subtitle)

        columns = QHBoxLayout()
        columns.addWidget(self._virtual_panel())
        columns.addWidget(self._real_panel())
        outer.addLayout(columns, 1)
        outer.addWidget(self._control_panel())

        self.summary = QLabel("硬件：正在连接　｜　ROS2：等待数据　｜　MuJoCo：正在启动")
        self.summary.setAlignment(Qt.AlignCenter)
        self.summary.setStyleSheet("padding: 8px; background: #263238; color: white;")
        outer.addWidget(self.summary)
        self.setCentralWidget(root)
        self.setStyleSheet(
            "QGroupBox { font-weight: bold; border: 1px solid #78909c; margin-top: 8px; padding-top: 8px; }"
            "QPushButton { min-height: 32px; padding: 3px 10px; }"
            "QLabel { min-height: 20px; }"
        )
        self._set_virtual_editable(False)

    def _slider(self, index: int) -> QSlider:
        slider = QSlider(Qt.Horizontal)
        lower, upper = self.limits[index]
        slider.setRange(round(lower * 100.0), round(upper * 100.0))
        slider.setSingleStep(1)
        return slider

    def _virtual_panel(self) -> QGroupBox:
        box = QGroupBox("虚拟机械臂（目标／仿真）")
        layout = QGridLayout(box)
        layout.addWidget(QLabel("关节"), 0, 0)
        layout.addWidget(QLabel("滑条"), 0, 1)
        layout.addWidget(QLabel("角度"), 0, 2)
        layout.addWidget(QLabel("精确输入"), 0, 3)
        for index, label_text in enumerate(JOINT_LABELS):
            slider = self._slider(index)
            value = QLabel("+0.00°")
            spin = QDoubleSpinBox()
            spin.setRange(*self.limits[index])
            spin.setDecimals(2)
            spin.setSingleStep(0.1)
            slider.valueChanged.connect(lambda raw, i=index: self._virtual_slider_changed(i, raw))
            spin.valueChanged.connect(lambda degrees, i=index: self._virtual_spin_changed(i, degrees))
            row = index + 1
            layout.addWidget(QLabel(label_text), row, 0)
            layout.addWidget(slider, row, 1)
            layout.addWidget(value, row, 2)
            layout.addWidget(spin, row, 3)
            self.virtual_widgets.append(VirtualWidgets(slider, value, spin))
        return box

    def _real_panel(self) -> QGroupBox:
        box = QGroupBox("现实机械臂（编码器反馈）")
        layout = QGridLayout(box)
        for column, text in enumerate(("关节", "实际滑条", "实际角度", "目标角度", "位置误差", "状态")):
            layout.addWidget(QLabel(text), 0, column)
        for index, label_text in enumerate(JOINT_LABELS):
            slider = self._slider(index)
            slider.setEnabled(False)
            widgets = RealWidgets(slider, QLabel("+0.00°"), QLabel("+0.00°"), QLabel("+0.00°"), QLabel("未连接"))
            row = index + 1
            layout.addWidget(QLabel(label_text), row, 0)
            for column, widget in enumerate((widgets.slider, widgets.actual, widgets.target, widgets.error, widgets.state), 1):
                layout.addWidget(widget, row, column)
            self.real_widgets.append(widgets)
        return box

    def _button(self, text: str, callback, object_name: str = "") -> QPushButton:
        button = QPushButton(text)
        if object_name:
            button.setObjectName(object_name)
        button.clicked.connect(callback)
        return button

    def _control_panel(self) -> QGroupBox:
        box = QGroupBox("控制模式与安全操作")
        layout = QGridLayout(box)
        buttons = [
            ("现实驱动虚拟", self._real_to_sim), ("虚拟驱动现实", self._sim_to_real),
            ("位置模式", self._position_mode), ("力矩开启", self._torque_on),
            ("力矩关闭", self._torque_off), ("可拖动模式", self._drag_mode),
            ("保持当前位置", self._hold_current), ("设置初始化姿态", self._save_initial_pose),
            ("回到初始化姿态", self._return_initial_pose), ("执行目标姿态", self._execute_target),
        ]
        for index, (text, callback) in enumerate(buttons):
            layout.addWidget(self._button(text, callback), index // 5, index % 5)
        stop = self._button("停止并制动", self._stop)
        stop.setStyleSheet("background: #b71c1c; color: white; font-weight: bold; min-height: 46px;")
        layout.addWidget(stop, 2, 0, 1, 5)
        self.mode_label = QLabel("当前模式：现实驱动虚拟　｜　驱动状态：已制动")
        self.mode_label.setAlignment(Qt.AlignCenter)
        layout.addWidget(self.mode_label, 3, 0, 1, 5)
        return box

    def _set_virtual_editable(self, enabled: bool) -> None:
        for widgets in self.virtual_widgets:
            widgets.slider.setEnabled(enabled)
            widgets.target.setEnabled(enabled)

    def _virtual_slider_changed(self, index: int, raw: int) -> None:
        if self.direction is not ArmMode.SIM_TO_REAL:
            return
        degrees = raw / 100.0
        widgets = self.virtual_widgets[index]
        widgets.value.setText(f"{degrees:+.2f}°")
        if abs(widgets.target.value() - degrees) > 0.004:
            widgets.target.blockSignals(True)
            widgets.target.setValue(degrees)
            widgets.target.blockSignals(False)
        self.targets[index] = degrees * RAD
        self.pending_target_joint_mask[index] = (
            self.connected[index]
            and abs(self.targets[index] - self.actual[index]) > TARGET_SELECTION_DEADBAND_RAD
        )

    def _virtual_spin_changed(self, index: int, degrees: float) -> None:
        if self.direction is not ArmMode.SIM_TO_REAL:
            return
        widgets = self.virtual_widgets[index]
        raw = round(degrees * 100.0)
        if widgets.slider.value() != raw:
            widgets.slider.blockSignals(True)
            widgets.slider.setValue(raw)
            widgets.slider.blockSignals(False)
        widgets.value.setText(f"{degrees:+.2f}°")
        self.targets[index] = degrees * RAD
        self.pending_target_joint_mask[index] = (
            self.connected[index]
            and abs(self.targets[index] - self.actual[index]) > TARGET_SELECTION_DEADBAND_RAD
        )

    def _real_to_sim(self) -> None:
        self.direction = ArmMode.REAL_TO_SIM
        self.machine.real_to_sim()
        self.hardware_mode = "brake"
        self.requested_active_joint_mask = [False] * 6
        self.pending_target_joint_mask = [False] * 6
        self.targets = list(self.actual)
        self.command_targets = list(self.actual)
        self._set_virtual_editable(False)
        self._update_mode_label()

    def _sim_to_real(self) -> None:
        self.direction = ArmMode.SIM_TO_REAL
        self.machine.sim_to_real()
        self.targets = list(self.actual)
        self.command_targets = list(self.actual)
        self.requested_active_joint_mask = [False] * 6
        self.pending_target_joint_mask = [False] * 6
        self._set_virtual_editable(True)
        self._show_targets_on_virtual()
        self._update_mode_label()

    def _position_mode(self) -> None:
        if not self._require_control_feedback(
            "关节与硬件状态流需保持新鲜，且至少一个关节健康连接，当前不能进入位置模式。"
        ):
            return
        self.machine.position()
        self.hardware_mode = "position"
        self.targets = list(self.actual)
        self.command_targets = list(self.actual)
        self.requested_active_joint_mask = [False] * 6
        self.pending_target_joint_mask = [False] * 6
        self._update_mode_label(
            "位置模式已准备；健康关节可执行，未连接／故障关节保持底层制动"
        )

    def _torque_on(self) -> None:
        if not self._require_control_feedback(
            "关节与硬件状态流需保持新鲜，且至少一个关节健康连接，当前不能开启主动保持。"
        ):
            return
        self.targets = list(self.actual)
        self.command_targets = list(self.actual)
        self.pending_target_joint_mask = [False] * 6
        self._authorize_active_joints(self.connected)
        self.machine.torque_on()
        self.hardware_mode = "hold"
        self._update_mode_label("健康关节主动保持；未连接／故障关节保持底层制动")

    def _torque_off(self) -> bool:
        now = time.monotonic()
        if not self.node.control_streams_fresh(now):
            self._update_connected(now)
            self._fail_closed_for_stale_feedback()
            self._message("关节或硬件状态数据不新鲜，已保持制动。")
            return False
        self.direction = ArmMode.REAL_TO_SIM
        self.machine.drag()
        self.hardware_mode = "drag"
        self.pending_target_joint_mask = [False] * 6
        self._authorize_active_joints(self.connected)
        self._set_virtual_editable(False)
        self._update_mode_label("驱动力已关闭，24V／主电源仍可能接通")
        return True

    def _drag_mode(self) -> None:
        dialog = QMessageBox(self)
        dialog.setWindowTitle("可拖动模式安全提示")
        dialog.setText(
            "当前为无重力补偿拖动模式。\nJ2／J3 等重载关节可能因机械臂自重下落，\n"
            "请可靠支撑机械臂后操作。"
        )
        confirm = dialog.addButton("确认进入", QMessageBox.ButtonRole.AcceptRole)
        dialog.addButton("取消", QMessageBox.ButtonRole.RejectRole)
        dialog.exec()
        if dialog.clickedButton() is not confirm:
            return
        if self._torque_off():
            self._update_mode_label("可拖动模式：请持续可靠支撑机械臂")

    def _hold_current(self) -> None:
        if not self._require_control_feedback(
            "关节与硬件状态流需保持新鲜，且至少一个关节健康连接，当前不能保持。"
        ):
            return
        self.targets = list(self.actual)
        self.command_targets = list(self.actual)
        self.pending_target_joint_mask = [False] * 6
        self._authorize_active_joints(self.connected)
        self.machine.hold()
        self.hardware_mode = "hold"
        self.arrival.start(time.monotonic())
        self._update_mode_label("健康关节正在保持；未连接／故障关节保持底层制动")

    def _execute_target(self) -> None:
        if not self._require_control_feedback(
            "关节与硬件状态流需保持新鲜，且至少一个关节健康连接，当前不能执行新目标。"
        ):
            return
        if self.direction is not ArmMode.SIM_TO_REAL:
            self._message("请先选择“虚拟驱动现实”。")
            return
        self.machine.position()
        self.hardware_mode = "position"
        self.command_targets = list(self.targets)
        self._authorize_active_joints(list(self.pending_target_joint_mask))
        self.pending_target_joint_mask = [False] * 6
        self.arrival.start(time.monotonic())
        self._update_mode_label("健康关节正在执行；未连接／故障关节保持底层制动")

    def _stop(self) -> None:
        self.machine.stop()
        self.hardware_mode = "brake"
        self.requested_active_joint_mask = [False] * 6
        self.pending_target_joint_mask = [False] * 6
        self._update_mode_label("已停止全部轨迹并发送制动／安全关闭")
        self._publish_command()

    def _save_initial_pose(self) -> None:
        if not self._require_control_feedback(
            "设置完整六轴初始化姿态要求六个关节均具有新鲜状态且健康连接。",
            require_all=True,
        ):
            return
        if not self.session_id:
            self._message("尚未建立有效会话参考。")
            return
        document = {
            "有效": True,
            "会话标识": self.session_id,
            "时间戳": datetime.now(timezone.utc).isoformat(),
            "语义": "用户初始化姿态；仅为当前会话相对角度",
            "电机内部零位": "未修改",
            "CAD零位": "PENDING",
            "ROS零位": "PENDING",
            "关节位置_弧度": {f"J{i + 1}": self.actual[i] for i in range(6)},
        }
        self.node.initial_pose_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = self.node.initial_pose_path.with_suffix(
            self.node.initial_pose_path.suffix + ".tmp"
        )
        temporary_path.write_text(
            json.dumps(document, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        temporary_path.replace(self.node.initial_pose_path)
        self._message("初始化姿态已保存。")

    def _return_initial_pose(self) -> None:
        if not self._require_control_feedback(
            "返回完整六轴初始化姿态要求六个关节均具有新鲜状态且健康连接。",
            require_all=True,
        ):
            return
        try:
            document = json.loads(self.node.initial_pose_path.read_text(encoding="utf-8"))
            if not document.get("有效") or document.get("会话标识") != self.session_id:
                raise ValueError
            positions = document["关节位置_弧度"]
            self.targets = [float(positions[f"J{i + 1}"]) for i in range(6)]
            if not all(
                math.isfinite(value) and self.limits[index][0] * RAD <= value <= self.limits[index][1] * RAD
                for index, value in enumerate(self.targets)
            ):
                raise ValueError
        except (OSError, KeyError, TypeError, ValueError, json.JSONDecodeError):
            self._message("初始化姿态需要重新设置。")
            return
        self.direction = ArmMode.SIM_TO_REAL
        self.pending_target_joint_mask = [
            connected and abs(target - actual) > TARGET_SELECTION_DEADBAND_RAD
            for target, actual, connected in zip(
                self.targets, self.actual, self.connected
            )
        ]
        self._set_virtual_editable(True)
        self._show_targets_on_virtual()
        self._execute_target()

    def _message(self, text: str) -> None:
        dialog = QMessageBox(self)
        dialog.setWindowTitle("机械臂控制提示")
        dialog.setText(text)
        dialog.addButton("知道了", QMessageBox.ButtonRole.AcceptRole)
        dialog.exec()

    def _require_control_feedback(self, message: str, require_all: bool = False) -> bool:
        now = time.monotonic()
        streams_fresh = self.node.control_streams_fresh(now)
        self._update_connected(now)
        if control_feedback_ready(
            self.have_first_state, streams_fresh, self.connected, require_all
        ):
            return True
        if not streams_fresh:
            self._fail_closed_for_stale_feedback()
        self._message(message)
        return False

    def _fail_closed_for_stale_feedback(self) -> None:
        if self.hardware_mode not in {"drag", "hold", "position"}:
            return
        self.machine.stop()
        self.hardware_mode = "brake"
        self.direction = ArmMode.REAL_TO_SIM
        self.targets = list(self.actual)
        self.command_targets = list(self.actual)
        self.requested_active_joint_mask = [False] * 6
        self.pending_target_joint_mask = [False] * 6
        self._set_virtual_editable(False)
        self._show_targets_on_virtual()
        self._update_mode_label("关节或硬件状态数据已过期，已请求制动")
        self._publish_command()

    def _show_targets_on_virtual(self) -> None:
        for index, target in enumerate(self.targets):
            degrees = target * DEG
            widgets = self.virtual_widgets[index]
            widgets.slider.blockSignals(True)
            widgets.target.blockSignals(True)
            widgets.slider.setValue(round(degrees * 100.0))
            widgets.target.setValue(degrees)
            widgets.slider.blockSignals(False)
            widgets.target.blockSignals(False)
            widgets.value.setText(f"{degrees:+.2f}°")

    def _update_mode_label(self, extra: str = "") -> None:
        direction_text = "现实驱动虚拟" if self.direction is ArmMode.REAL_TO_SIM else "虚拟驱动现实"
        drive_text = {"brake": "制动请求", "drag": "驱动力关闭请求", "hold": "保持请求", "position": "位置控制请求"}[self.hardware_mode]
        text = f"当前模式：{direction_text}　｜　驱动状态：{drive_text}"
        if extra:
            text += f"　｜　{extra}"
        self.mode_label.setText(text)

    def _tick(self) -> None:
        if not run_ros_context_operation(
            self._ros_context_ok,
            lambda: rclpy.spin_once(self.node, timeout_sec=0.0),
        ):
            self._close_for_ros_shutdown()
            return
        now = time.monotonic()
        message = self.node.latest_joint_state
        if message is not None:
            self.actual = [float(value) for value in message.position]
            if not self.have_first_state:
                self.targets = list(self.actual)
                self.command_targets = list(self.actual)
                self.have_first_state = True
        self._update_connected(now)
        if not self.node.control_streams_fresh(now):
            self._fail_closed_for_stale_feedback()
        self._refresh_joint_widgets()
        if not run_ros_context_operation(self._ros_context_ok, self._publish_command):
            self._close_for_ros_shutdown()
            return
        self._refresh_summary()
        self._write_log()

    def _ros_context_ok(self) -> bool:
        return rclpy.ok(context=self.node.context)

    def _close_for_ros_shutdown(self) -> None:
        self.timer.stop()
        self.close()

    def _update_connected(self, now: Optional[float] = None) -> None:
        checked_at = time.monotonic() if now is None else now
        if not self.node.control_streams_fresh(checked_at):
            self._apply_connection_state([False] * 6, [False] * 6)
            return
        hardware = self.node.latest_hardware
        if not hardware or "per_motor" not in hardware:
            self._apply_connection_state([False] * 6, [False] * 6)
            return
        motors = hardware["per_motor"]
        incoming_session = hardware.get("session_id")
        if incoming_session and self.session_id and incoming_session != self.session_id:
            self.targets = list(self.actual)
            self.command_targets = list(self.actual)
            self.requested_active_joint_mask = [False] * 6
            self.pending_target_joint_mask = [False] * 6
            self.hardware_mode = "brake"
        if incoming_session:
            self.session_id = str(incoming_session)
        good = lambda name: bool(
            motors.get(name, {}).get("communication_ok", False)
            and motors.get(name, {}).get("fresh", False)
            and motors.get(name, {}).get("merror", -1) == 0
            and motors.get(name, {}).get("reference_captured", False)
            and not hardware.get("controller_fault_by_motor", {}).get(name, False)
        )
        bad = lambda name: bool(
            motors.get(name, {}).get("reference_captured", False)
            and not good(name)
        )
        self._apply_connection_state(
            [good("J1"), good("J2A") and good("J2B"), good("J3"), good("J4"), good("J5"), good("J6")],
            [bad("J1"), bad("J2A") or bad("J2B"), bad("J3"), bad("J4"), bad("J5"), bad("J6")],
        )

    def _apply_connection_state(
        self, connected: list[bool], faulted: list[bool]
    ) -> None:
        self.requested_active_joint_mask = clear_mask_on_connection_loss(
            self.requested_active_joint_mask, self.connected, connected
        )
        self.pending_target_joint_mask = clear_mask_on_connection_loss(
            self.pending_target_joint_mask, self.connected, connected
        )
        self.connected = connected
        self.faulted = faulted

    def _authorize_active_joints(self, requested: list[bool]) -> None:
        if self.activation_epoch >= (1 << 63) - 1:
            raise RuntimeError("激活纪元已耗尽，必须安全重启完整控制会话")
        self.requested_active_joint_mask = [
            bool(requested[index] and self.connected[index])
            for index in range(6)
        ]
        self.activation_epoch += 1

    def _refresh_joint_widgets(self) -> None:
        now = time.monotonic()
        errors = [target - actual for target, actual in zip(self.command_targets, self.actual)]
        active_joint_mask = effective_active_joint_mask(
            self.requested_active_joint_mask, self.connected, self.hardware_mode
        )
        if self.hardware_mode in {"position", "hold"}:
            tracking_connected = [
                connected and active
                for connected, active in zip(self.connected, active_joint_mask)
            ]
            states = self.arrival.update(now, errors, tracking_connected)
            for index, connected in enumerate(self.connected):
                if connected and not active_joint_mask[index]:
                    states[index] = "已制动"
        elif self.hardware_mode == "brake":
            states = ["已制动" if item else ("故障" if self.faulted[index] else "未连接")
                      for index, item in enumerate(self.connected)]
        elif self.hardware_mode == "drag":
            states = [
                "待机" if item and active_joint_mask[index]
                else "已制动" if item
                else ("故障" if self.faulted[index] else "未连接")
                for index, item in enumerate(self.connected)
            ]
        else:
            states = ["待机"] * 6
        hardware = self.node.latest_hardware or {}
        for index, faulted in enumerate(self.faulted):
            if faulted:
                states[index] = "故障"
        if hardware.get("j2_sync_fault", False) or abs(float(hardware.get("j2_e_sync_rad", 0.0))) > RAD:
            states[1] = "同步异常"
        for index in range(6):
            actual_deg = self.actual[index] * DEG
            target_deg = self.command_targets[index] * DEG
            error_deg = errors[index] * DEG
            real = self.real_widgets[index]
            real.slider.setValue(round(actual_deg * 100.0))
            real.actual.setText(f"{actual_deg:+.2f}°")
            real.target.setText(f"{target_deg:+.2f}°")
            real.error.setText(f"{error_deg:+.2f}°")
            real.state.setText(states[index])
        if self.direction is ArmMode.REAL_TO_SIM:
            self.targets = list(self.actual)
            self.command_targets = list(self.actual)
            self._show_targets_on_virtual()

    def _publish_command(self) -> None:
        self.command_sequence += 1
        active_joint_mask = effective_active_joint_mask(
            self.requested_active_joint_mask, self.connected, self.hardware_mode
        )
        self.node.publish_command(
            self.command_sequence, self.hardware_mode,
            self.command_targets, self.targets, active_joint_mask,
            self.activation_epoch, self.config
        )
        self.node.mode_publisher.publish(String(data=json.dumps({
            "direction": self.direction.value,
            "hardware_mode": self.hardware_mode,
            "active_joint_mask": active_joint_mask,
            "activation_epoch": self.activation_epoch,
        })))

    def _refresh_summary(self) -> None:
        now = time.monotonic()
        ros_ok = self.node.control_streams_fresh(now)
        hardware_text = "已连接" if all(self.connected) else ("部分连接" if any(self.connected) else "未连接")
        mujoco = self.node.latest_mujoco
        mujoco_text = mujoco_status_text(mujoco, self.node.mujoco_stream_fresh(now))
        sync_text = "等待"
        if ros_ok and self.node.latest_hardware:
            sync_deg = float(self.node.latest_hardware.get("j2_e_sync_rad", 0.0)) * DEG
            sync_text = f"{sync_deg:+.3f}°"
        actual_modes = set(
            (self.node.latest_hardware or {}).get("controller_mode_by_motor", {}).values()
        ) if ros_ok else set()
        mode_text = {
            "brake": "已制动", "drag": "可拖动", "hold": "保持中",
            "position": "位置控制", "unknown": "未知",
        }
        confirmed = "、".join(sorted(mode_text.get(item, "未知") for item in actual_modes)) or "等待确认"
        measured_rate = (
            (self.node.latest_hardware or {}).get("timing", {}).get("median_publish_rate_hz")
            if ros_ok else None
        )
        rate_text = "等待" if measured_rate is None else f"{float(measured_rate):.1f}赫兹"
        joint_states = [widgets.state.text() for widgets in self.real_widgets]
        if any(state == "未到位" for state in joint_states):
            overall = "部分关节未到位"
        elif any(state in {"故障", "同步异常"} for state in joint_states):
            overall = "存在关节故障"
        elif joint_states and all(state in {"已到位", "保持中"} for state in joint_states):
            overall = "全部关节已到位"
        else:
            overall = "状态监视中"
        self.summary.setText(
            f"硬件：{hardware_text}　｜　ROS2：{'正常' if ros_ok else '等待数据'}　｜　MuJoCo：{mujoco_text}　｜　"
            f"实测更新频率：{rate_text}　｜　控制器确认：{confirmed}　｜　"
            f"J2双电机同步误差：{sync_text}　｜　整体：{overall}　｜　"
            "J2最终负载跟踪待解决"
        )

    def _open_log(self) -> None:
        self.node.log_directory.mkdir(parents=True, exist_ok=True)
        token = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_stream = (self.node.log_directory / f"arm_gui_{token}.csv").open("w", encoding="utf-8", newline="")
        fields = ["timestamp", "mode"]
        for index in range(1, 7):
            fields.extend((f"J{index}_target_rad", f"J{index}_actual_rad"))
        fields.extend(("J2_sync_error_rad", "fault_state"))
        self.log_writer = csv.DictWriter(self.log_stream, fieldnames=fields)
        self.log_writer.writeheader()

    def _write_log(self) -> None:
        now = time.monotonic()
        frequency = float(self.config["界面"]["日志频率_赫兹"])
        if now - self.last_log_at < 1.0 / frequency:
            return
        self.last_log_at = now
        row = {"timestamp": datetime.now(timezone.utc).isoformat(), "mode": self.hardware_mode}
        for index in range(6):
            row[f"J{index + 1}_target_rad"] = self.command_targets[index]
            row[f"J{index + 1}_actual_rad"] = self.actual[index]
        hardware = self.node.latest_hardware or {}
        row["J2_sync_error_rad"] = hardware.get("j2_e_sync_rad", "")
        row["fault_state"] = "" if all(self.connected) else "部分关节未连接"
        self.log_writer.writerow(row)
        self.log_stream.flush()

    def closeEvent(self, event: QCloseEvent) -> None:
        self.hardware_mode = "brake"
        self.requested_active_joint_mask = [False] * 6
        self.pending_target_joint_mask = [False] * 6

        def publish_brake_and_spin() -> None:
            self._publish_command()
            rclpy.spin_once(self.node, timeout_sec=0.01)

        for _ in range(5):
            if not run_ros_context_operation(
                self._ros_context_ok, publish_brake_and_spin
            ):
                break
        self.log_stream.flush()
        self.log_stream.close()
        event.accept()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArmGuiNode()
    app = QApplication(sys.argv)
    app.setApplicationDisplayName("六自由度机械臂控制系统")
    window = MainWindow(node)
    window.show()
    exit_code = app.exec()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    raise SystemExit(exit_code)


if __name__ == "__main__":
    main()

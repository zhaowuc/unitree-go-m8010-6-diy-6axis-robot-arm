"""Editable action groups using the existing real-arm preview/submit workflow."""

from __future__ import annotations

import json
import math
import time
from datetime import datetime, timezone

from PySide6.QtCore import Qt
from PySide6.QtWidgets import (
    QAbstractItemView, QDialog, QFileDialog, QHBoxLayout, QLabel, QLineEdit,
    QPushButton, QTableWidget, QTableWidgetItem, QVBoxLayout,
)

from .action_groups import ActionGroup, ActionGroupRunner, ActionStep
from .state_machine import ArmMode, ArrivalTracker


class ActionGroupDialog(QDialog):
    def __init__(self, window) -> None:
        super().__init__(window)
        self.window = window
        self.runner = None
        self.internal_command = False
        self.saved_settings = None
        self.log_stream = None
        self.log_path = None
        self.logged_events = 0
        self.binding = None
        self.rejected_commands = None
        self.target_rad = ()
        self.submitted = False
        self.no_motion = False
        self.inside_since = None
        self.setWindowTitle("动作组：保存、加载与六轴顺序执行")
        self.setModal(False)
        self.resize(1160, 560)
        layout = QVBoxLayout(self)
        self.name = QLineEdit("新动作组")
        self.name.setMaxLength(128)
        self.name.setPlaceholderText("动作组名称")
        layout.addWidget(self.name)
        notice = QLabel(
            "目标均为模型绝对角度。演示到位：六轴误差 ≤0.25°、静止 HOLD 连续0.5秒；"
            "原严格验收不变。\n夹爪已知为 PWM，控制板尚未接入；可编辑夹爪步骤，"
            "含夹爪动作的组暂不能下发现实。J6 是腕部关节。"
        )
        notice.setWordWrap(True)
        layout.addWidget(notice)
        self.table = QTableWidget(0, 11)
        self.table.setHorizontalHeaderLabels([
            "J1绝对°", "J2绝对°", "J3绝对°", "J4绝对°", "J5绝对°", "J6绝对°",
            "速度°/s", "停留s", "夹爪 none/open/close/set", "开度%（set）", "夹爪等待s",
        ])
        self.table.horizontalHeader().setDefaultSectionSize(85)
        self.table.setColumnWidth(8, 190)
        self.table.setSelectionBehavior(QAbstractItemView.SelectRows)
        self.table.setAlternatingRowColors(True)
        layout.addWidget(self.table, 1)
        self.edit_buttons = []
        edits = QHBoxLayout()
        for label, callback in (
            ("添加虚拟目标", lambda: self.capture(False)),
            ("捕获当前实姿", lambda: self.capture(True)),
            ("删除选中步骤", self.delete_rows),
            ("保存 JSON", self.save_file), ("加载 JSON", self.load_file),
        ):
            button = QPushButton(label)
            button.clicked.connect(lambda _checked=False, call=callback: self._edit(call))
            edits.addWidget(button)
            self.edit_buttons.append(button)
        layout.addLayout(edits)
        controls = QHBoxLayout()
        self.run_button = QPushButton("执行动作组到现实")
        self.pause_button = QPushButton("暂停")
        self.resume_button = QPushButton("恢复")
        self.stop_button = QPushButton("停止动作组并保持")
        for button, callback in (
            (self.run_button, self.start), (self.pause_button, self.pause),
            (self.resume_button, self.resume), (self.stop_button, self.stop),
        ):
            button.clicked.connect(callback)
            controls.addWidget(button)
        layout.addLayout(controls)
        self.status = QLabel("尚未执行。可先添加姿态并保存。")
        self.status.setWordWrap(True)
        self.status.setTextInteractionFlags(Qt.TextSelectableByMouse)
        layout.addWidget(self.status)
        self._refresh_controls()

    @property
    def active(self) -> bool:
        return self.runner is not None and self.runner.active

    @property
    def blocks_manual(self) -> bool:
        return self.active and not self.internal_command

    def _invoke(self, callback):
        previous = self.internal_command
        self.internal_command = True
        try:
            return callback()
        finally:
            self.internal_command = previous

    def _edit(self, callback) -> None:
        if self.active:
            return
        try:
            callback()
        except (OSError, ValueError, TypeError, OverflowError) as error:
            self.status.setText(f"未完成：{error}")

    def capture(self, actual: bool) -> None:
        window = self.window
        if actual and not window._require_control_feedback(
            "捕获实姿需要六轴新鲜健康编码器反馈。", require_all=True,
        ):
            raise ValueError("当前实姿反馈不完整或过期，未添加步骤")
        relative = window.actual if actual else window.candidate_targets
        self.append_step(ActionStep(tuple(
            math.degrees(angle) + anchor
            for angle, anchor in zip(relative, window.session_pose_deg)
        )))

    def append_step(self, step: ActionStep) -> None:
        row = self.table.rowCount()
        self.table.insertRow(row)
        values = [*step.target_deg, step.speed_deg_s, step.dwell_s, step.gripper,
                  "" if step.opening_percent is None else step.opening_percent,
                  step.gripper_wait_s]
        for column, value in enumerate(values):
            text = str(value)
            self.table.setItem(row, column, QTableWidgetItem(text))

    def group(self) -> ActionGroup:
        steps = []
        for row in range(self.table.rowCount()):
            values = [self.table.item(row, col).text().strip() for col in range(11)]
            try:
                steps.append(ActionStep(
                    tuple(float(value) for value in values[:6]),
                    float(values[6]), float(values[7]), values[8],
                    None if not values[9] else float(values[9]), float(values[10]),
                ))
            except (ValueError, TypeError, OverflowError) as error:
                raise ValueError(f"第 {row + 1} 步：{error}") from error
        return ActionGroup(self.name.text(), tuple(steps),
                           self.window.workflow_contract.model_sha256)

    def delete_rows(self) -> None:
        for row in sorted({index.row() for index in self.table.selectedIndexes()}, reverse=True):
            self.table.removeRow(row)

    def save_file(self) -> None:
        group = self.group()
        path, _ = QFileDialog.getSaveFileName(self, "保存动作组", "动作组.json", "JSON (*.json)")
        if path:
            group.save(path)
            self.status.setText("已保存：" + path)

    def load_file(self) -> None:
        path, _ = QFileDialog.getOpenFileName(self, "加载动作组", "", "JSON (*.json)")
        if path:
            group = ActionGroup.load(
                path, expected_model_sha256=self.window.workflow_contract.model_sha256,
                joint_limits_deg=self.window.absolute_limits,
            )
            self.table.setRowCount(0)
            self.name.setText(group.name)
            for step in group.steps:
                self.append_step(step)
            self.status.setText("已加载：" + path)

    def _health(self) -> None:
        self.window._action_group_health(self.binding, self.rejected_commands)
        waiting = self.runner is not None and (
            self.runner.state in {"dwell", "gripper_wait"}
            or (self.runner.state == "paused" and self.runner._paused_state in {"dwell", "gripper_wait"})
        )
        if waiting:
            if not self.window._action_group_hold_ready(self.target_rad):
                raise RuntimeError("动作组停止：停留期间六轴不再静止保持目标")

    def start(self) -> None:
        if self.active:
            return
        self.runner = None
        window = self.window
        try:
            group = self.group()
            if any(step.gripper != "none" for step in group.steps):
                raise ValueError("夹爪为 PWM，但控制板尚未接入；含夹爪动作的组未下发现实")
            baseline = window._action_group_health()
            if (window.queued_pose_target is not None or window.pending_collision_execute_sequence is not None
                    or window.hardware_mode != "hold"
                    or not window._action_group_hold_ready(window.command_targets)):
                raise RuntimeError("请先结束当前轨迹并建立六轴健康、静止 HOLD")
            self.binding = baseline["binding"]
            self.rejected_commands = baseline["rejected_commands"]
            self.saved_settings = (
                window.config["控制"]["最大速度_度每秒"], window.arrival,
                window.machine.fixed_hold_after_arrival,
            )
            window.arrival = ArrivalTracker(math.radians(0.25), 0.5,
                                            window.arrival.timeout_s,
                                            per_joint_tolerance_rad=(math.radians(0.25),) * 6)
            window.machine.set_fixed_hold_after_arrival(True)
            window.node.log_directory.mkdir(parents=True, exist_ok=True)
            stamp = datetime.now(timezone.utc).strftime("%Y%m%d_%H%M%S_%f")
            self.log_path = window.node.log_directory / f"action_group_{stamp}.jsonl"
            self.log_stream = self.log_path.open("x", encoding="utf-8")
            self.log_stream.write(json.dumps({
                "event": "run_start", "mode": "real_hardware", "group": group.to_dict(),
                "binding": self.binding, "demo_tolerance_deg": 0.25, "steady_dwell_s": 0.5,
                "gripper_available": False,
            }, ensure_ascii=False, allow_nan=False) + "\n")
            self.log_stream.flush()
            self.logged_events = 0
            self.runner = ActionGroupRunner(
                group, self._move_start, self._move_status, self._stop_motion,
                self._gripper_command, self._health,
                motion_timeout_s=1800.0,
            )
            self.runner.start()
        except Exception as error:
            self.status.setText(f"动作组未启动：{error}")
            self._restore()
            return
        self.tick()

    def _move_start(self, step: ActionStep) -> None:
        window = self.window
        self.target_rad = tuple(math.radians(angle - anchor)
                                for angle, anchor in zip(step.target_deg, window.session_pose_deg))
        self.inside_since = None
        self.no_motion = all(abs(target - locked) <= math.radians(0.01)
                             for target, locked in zip(self.target_rad, window.command_targets))
        self.submitted = self.no_motion
        window.config["控制"]["最大速度_度每秒"] = min(self.saved_settings[0], step.speed_deg_s)
        if self.no_motion:
            return
        if not window._action_group_hold_ready(window.command_targets):
            raise RuntimeError("下一动作开始前尚未确认六轴静止 HOLD")
        window.direction = ArmMode.SIM_TO_REAL
        window.candidate_targets = list(self.target_rad)
        window.targets = list(self.target_rad)
        window.candidate_joint_mask = [abs(target - locked) > math.radians(0.01)
                                       for target, locked in zip(self.target_rad, window.command_targets)]
        window.workflow_contract = window.workflow_contract.change_plan_target(self.target_rad)
        window._clear_candidate_approval()
        window._show_targets_on_virtual()
        self._invoke(window._start_virtual_preview)
        if not window.preview_requested_by_operator:
            raise RuntimeError("本步预演未启动，未取得现实运动授权")

    def _move_status(self) -> str:
        window = self.window
        if not self.no_motion and window.collision_preview_state in {
            "unsafe", "stale", "blocked", "timeout", "idle",
        }:
            raise RuntimeError("本步预演或执行已失效：" + window.collision_preview_state)
        if not self.submitted:
            if window._preview_approval_matches_candidate():
                self._invoke(window._execute_target)
                if window.queued_pose_target is None:
                    raise RuntimeError("已有预演，但本步现实下发被现有控制链拒绝")
                self.submitted = True
            return "running"
        if not self.no_motion and (
            window.collision_preview_state != "complete" or window.queued_pose_target is not None
        ):
            return "running"
        now = time.monotonic()
        if not window._action_group_hold_ready(self.target_rad):
            self.inside_since = None
            return "running"
        if self.inside_since is None:
            self.inside_since = now
        if now - self.inside_since < 0.5:
            return "running"
        self.runner._event("measured_arrival", actual_model_deg=[
            math.degrees(value) + anchor for value, anchor in zip(window.actual, window.session_pose_deg)
        ], error_deg=[math.degrees(target - actual)
                      for target, actual in zip(self.target_rad, window.actual)])
        return "complete"

    def _stop_motion(self) -> None:
        self._invoke(self.window._stop)

    @staticmethod
    def _gripper_command(_step) -> str:
        raise RuntimeError("PWM 夹爪控制板未接入，未发送夹爪命令")

    def check_health(self) -> None:
        if self.active:
            try:
                self._health()
            except Exception as error:
                self.runner._fail(error)
                self.tick()

    def tick(self) -> None:
        if self.runner is not None:
            self.runner.tick()
            try:
                if self.log_stream is not None:
                    for event in self.runner.events[self.logged_events:]:
                        self.log_stream.write(json.dumps(event, ensure_ascii=False, allow_nan=False) + "\n")
                    self.logged_events = len(self.runner.events)
                    if not self.active:
                        self.log_stream.write(json.dumps({"event": "run_result", **self.runner.result},
                                                        ensure_ascii=False) + "\n")
                    self.log_stream.flush()
            except OSError as error:
                if self.active:
                    self.runner._fail(RuntimeError(f"动作组日志写入失败：{error}"))
            self.status.setText(
                f"{self.runner.state} · {min(self.runner.index + 1, len(self.runner.group.steps))}"
                f"/{len(self.runner.group.steps)} · {self.runner.detail}\n日志：{self.log_path}"
            )
            if not self.active:
                self._restore()
        self._refresh_controls()

    def _refresh_controls(self) -> None:
        active = self.active
        self.name.setEnabled(not active)
        self.table.setEditTriggers(QAbstractItemView.NoEditTriggers if active
                                   else QAbstractItemView.DoubleClicked | QAbstractItemView.EditKeyPressed)
        for button in self.edit_buttons:
            button.setEnabled(not active)
        self.run_button.setEnabled(not active)
        paused = active and self.runner.state == "paused"
        self.pause_button.setEnabled(active and not paused)
        resume_ready = paused and (
            self.runner._paused_state != "moving"
            or self.window._action_group_hold_ready(self.window.command_targets)
        )
        self.resume_button.setEnabled(resume_ready)
        self.stop_button.setEnabled(active)
        for button in getattr(self.window, "action_group_manual_buttons", []):
            if button is not getattr(self.window, "acceptance_target_button", None):
                button.setEnabled(not active)
        self.window._refresh_virtual_editability()

    def pause(self) -> None:
        if self.active:
            self.runner.pause()
            self.tick()

    def resume(self) -> None:
        if self.active:
            self.check_health()
            if not self.active:
                return
            if (self.runner.state == "paused" and self.runner._paused_state == "moving"
                    and not self.window._action_group_hold_ready(self.window.command_targets)):
                self.runner.detail = "已暂停，等待七电机确认静止 HOLD 后可恢复"
                self.tick()
                return
            self.runner.resume()
            self.tick()

    def stop(self) -> None:
        if self.active:
            self.runner.stop()
            self.tick()

    def _restore(self) -> None:
        if self.saved_settings is not None:
            speed, arrival, fixed_hold = self.saved_settings
            self.window.config["控制"]["最大速度_度每秒"] = speed
            self.window.arrival = arrival
            arrival.start(time.monotonic())
            self.window.machine.set_fixed_hold_after_arrival(fixed_hold)
            self.saved_settings = None
        if self.log_stream is not None:
            stream = self.log_stream
            self.log_stream = None
            try:
                stream.close()
            except OSError as error:
                self.status.setText(self.status.text() + f"\n日志关闭失败：{error}")

    def reject(self) -> None:
        self.stop()
        super().reject()

    def closeEvent(self, event) -> None:
        self.stop()
        super().closeEvent(event)

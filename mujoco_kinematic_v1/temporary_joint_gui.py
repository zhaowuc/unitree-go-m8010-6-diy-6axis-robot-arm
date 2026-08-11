from __future__ import annotations

"""Temporary fail-closed J1..J6 slider GUI for the V15.13 MuJoCo model."""

import argparse
import json
import math
import sys
from datetime import datetime, timezone
from pathlib import Path

import mujoco
import mujoco.viewer
import numpy as np
from PySide6.QtCore import Qt, QTimer
from PySide6.QtGui import QCloseEvent, QFont
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QGridLayout,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QListWidget,
    QMainWindow,
    QPushButton,
    QSlider,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from kinematic_guard import KinematicGuard


ROOT = Path(__file__).resolve().parent
MODEL_XML = ROOT / "go_m8010_arm_v15_13_kinematic.xml"
AXIS_JSON = ROOT / "V15_13_真实关节轴与相机上置零位.json"
DEFAULT_ZERO_CONFIG = ROOT / "gui_default_zero.json"
JOINTS = ("J1", "J2", "J3", "J4", "J5", "J6")
SOURCE_JOINT = {"J1": "J1", "J2": "J2A", "J3": "J3", "J4": "J4", "J5": "J5", "J6": "J6"}
DISPLAY_LIMITS = {
    "J1": (-180.0, 180.0),
    "J2": (-170.0, 170.0),
    "J3": (-170.0, 170.0),
    "J4": (-116.0, 159.0),
    "J5": (-70.6, 151.2),
    "J6": (-180.0, 180.0),
}
SCALE = 10


def load_default_zero_deg() -> np.ndarray:
    if not DEFAULT_ZERO_CONFIG.exists():
        return np.zeros(6, dtype=float)
    payload = json.loads(DEFAULT_ZERO_CONFIG.read_text(encoding="utf-8"))
    angles = payload["cad_absolute_deg"]
    return np.asarray([float(angles[name]) for name in JOINTS], dtype=float)


class JointControlWindow(QMainWindow):
    def __init__(
        self,
        model: mujoco.MjModel,
        data: mujoco.MjData,
        viewer,
        startup_zero_deg: np.ndarray,
    ) -> None:
        super().__init__()
        self.model = model
        self.data = data
        self.viewer = viewer
        self.guard = KinematicGuard(MODEL_XML, model=model, data=data)
        self.joint_ids = {
            name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name) for name in JOINTS
        }
        self.current_abs_deg = np.asarray(startup_zero_deg, dtype=float).copy()
        self.temporary_zero_deg = np.asarray(startup_zero_deg, dtype=float).copy()
        self.sliders: dict[str, QSlider] = {}
        self.requested_labels: dict[str, QLabel] = {}
        self.estopped = False
        self.updating_sliders = False

        axes = json.loads(AXIS_JSON.read_text(encoding="utf-8"))
        self.source_rows = {row["name"]: row for row in axes["joints"]}
        self.setWindowTitle("V15.13 MuJoCo 临时关节控制 / 零位姿态确认")
        self.resize(980, 940)
        self._build_ui()
        self._set_slider_ranges_and_values(np.zeros(6))
        self.zero_posture.setText(
            "GUI启动0对应 CAD 姿态："
            + ", ".join(
                f"{name}={value:.3f}°" for name, value in zip(JOINTS, self.temporary_zero_deg)
            )
        )
        self._set_status("SAFE / 已加载几何精校后的 GUI 启动零位", "safe")
        self._refresh_angle_table()
        self._log("已载入实测关节轴、机械限位、自碰撞矩阵与物理地面。")
        self._log(
            "已恢复 GUI 启动0："
            + ", ".join(
                f"{name}={value:.3f}°" for name, value in zip(JOINTS, self.temporary_zero_deg)
            )
        )

        self.sync_timer = QTimer(self)
        self.sync_timer.timeout.connect(self._sync_viewer)
        self.sync_timer.start(16)

    def _build_ui(self) -> None:
        root = QWidget()
        outer = QVBoxLayout(root)
        outer.setContentsMargins(18, 16, 18, 16)
        outer.setSpacing(12)

        title_row = QHBoxLayout()
        title_block = QVBoxLayout()
        title = QLabel("MuJoCo 空载运动学控制台")
        title.setObjectName("title")
        subtitle = QLabel("V15.13 · 实测轴心/正方向 · 物理地面 · 扫掠碰撞保护")
        subtitle.setObjectName("subtitle")
        title_block.addWidget(title)
        title_block.addWidget(subtitle)
        title_row.addLayout(title_block, 1)
        self.estop_button = QPushButton("急停 / 冻结")
        self.estop_button.setObjectName("estop")
        self.estop_button.clicked.connect(self._toggle_estop)
        title_row.addWidget(self.estop_button)
        outer.addLayout(title_row)

        self.status = QLabel()
        self.status.setMinimumHeight(44)
        self.status.setAlignment(Qt.AlignmentFlag.AlignCenter)
        outer.addWidget(self.status)

        note = QLabel(
            "操作：拖动滑条后松开即执行。显示的“CAD绝对角”就是 MuJoCo qpos。"
            "“设当前为临时0”只改变本次GUI偏置，不修改 XML、URDF 或实机编码器零位。"
        )
        note.setWordWrap(True)
        note.setObjectName("note")
        outer.addWidget(note)

        sliders_frame = QFrame()
        sliders_frame.setObjectName("panel")
        slider_grid = QGridLayout(sliders_frame)
        slider_grid.setColumnStretch(1, 1)
        slider_grid.addWidget(QLabel("关节"), 0, 0)
        slider_grid.addWidget(QLabel("临时0相对角（拖动）"), 0, 1)
        slider_grid.addWidget(QLabel("请求值"), 0, 2)
        slider_grid.addWidget(QLabel("轴 / 机械范围"), 0, 3)
        for row_index, name in enumerate(JOINTS, 1):
            name_label = QLabel(name)
            name_label.setObjectName("jointName")
            slider = QSlider(Qt.Orientation.Horizontal)
            slider.setSingleStep(1)
            slider.setPageStep(10)
            slider.setTracking(True)
            slider.valueChanged.connect(lambda _value, n=name: self._preview_requested(n))
            slider.sliderReleased.connect(self._apply_requested_pose)
            requested = QLabel("0.0°")
            requested.setMinimumWidth(72)
            requested.setAlignment(Qt.AlignmentFlag.AlignRight | Qt.AlignmentFlag.AlignVCenter)
            source = self.source_rows[SOURCE_JOINT[name]]
            axis = np.asarray(source["mujoco_mjcf"]["joint_axis_in_child_body"], dtype=float)
            axis[np.abs(axis) < 5.0e-9] = 0.0
            axis_text = "[" + ", ".join(f"{value:.0f}" for value in axis) + "]"
            if name == "J1":
                range_text = f"axis {axis_text} · 连续（GUI ±180°）"
            else:
                lower, upper = DISPLAY_LIMITS[name]
                range_text = f"axis {axis_text} · {lower:g}° … {upper:g}°"
            range_label = QLabel(range_text)
            range_label.setObjectName("muted")
            slider_grid.addWidget(name_label, row_index, 0)
            slider_grid.addWidget(slider, row_index, 1)
            slider_grid.addWidget(requested, row_index, 2)
            slider_grid.addWidget(range_label, row_index, 3)
            self.sliders[name] = slider
            self.requested_labels[name] = requested
        outer.addWidget(sliders_frame)

        button_row = QHBoxLayout()
        apply_button = QPushButton("应用滑条目标")
        apply_button.clicked.connect(self._apply_requested_pose)
        zero_button = QPushButton("保存当前姿态为启动 0")
        zero_button.setObjectName("primary")
        zero_button.clicked.connect(self._set_temporary_zero)
        clear_button = QPushButton("清除临时偏置（姿态不动）")
        clear_button.clicked.connect(self._clear_temporary_zero)
        cad_zero_button = QPushButton("回 CAD 机械零位")
        cad_zero_button.clicked.connect(self._return_cad_zero)
        button_row.addWidget(apply_button)
        button_row.addWidget(zero_button)
        button_row.addWidget(clear_button)
        button_row.addWidget(cad_zero_button)
        outer.addLayout(button_row)

        self.zero_posture = QLabel("临时0对应 CAD 姿态：J1=0.0°, J2=0.0°, J3=0.0°, J4=0.0°, J5=0.0°, J6=0.0°")
        self.zero_posture.setObjectName("zeroPosture")
        self.zero_posture.setWordWrap(True)
        outer.addWidget(self.zero_posture)

        self.table = QTableWidget(6, 5)
        self.table.setHorizontalHeaderLabels(
            ["关节", "MuJoCo / CAD绝对角", "临时0相对角", "机械位置限位", "当前状态"]
        )
        self.table.verticalHeader().setVisible(False)
        self.table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeMode.Stretch)
        self.table.setEditTriggers(QTableWidget.EditTrigger.NoEditTriggers)
        self.table.setSelectionMode(QTableWidget.SelectionMode.NoSelection)
        self.table.setMaximumHeight(230)
        outer.addWidget(self.table)

        log_title = QLabel("运行日志")
        log_title.setObjectName("sectionTitle")
        outer.addWidget(log_title)
        self.log = QListWidget()
        self.log.setMinimumHeight(115)
        outer.addWidget(self.log)

        footer = QLabel(
            "碰撞策略：机械硬限位与动态碰撞域分离；地面或自碰撞姿态即使处于机械角度范围内，也会被拒绝。"
        )
        footer.setObjectName("muted")
        footer.setWordWrap(True)
        outer.addWidget(footer)

        self.setCentralWidget(root)
        self.setStyleSheet(
            """
            QWidget { background: #11151c; color: #e8edf4; font-family: 'Noto Sans CJK SC'; font-size: 13px; }
            QLabel#title { font-size: 24px; font-weight: 700; color: #f8fafc; }
            QLabel#subtitle, QLabel#muted { color: #8e9bad; }
            QLabel#note { background: #17202b; border: 1px solid #273547; border-radius: 8px; padding: 10px; color: #b8c7da; }
            QLabel#jointName { font-size: 15px; font-weight: 700; color: #76d5ff; }
            QLabel#zeroPosture { background: #18241f; border: 1px solid #2d6a4f; border-radius: 8px; padding: 9px; color: #b7f7d0; }
            QLabel#sectionTitle { font-size: 14px; font-weight: 700; }
            QFrame#panel { background: #151b24; border: 1px solid #27313f; border-radius: 10px; }
            QPushButton { background: #263241; border: 1px solid #3b4a5d; border-radius: 7px; padding: 9px 12px; }
            QPushButton:hover { background: #324256; }
            QPushButton#primary { background: #0c6e9f; border-color: #1597d0; font-weight: 700; }
            QPushButton#estop { background: #8f1d2c; border-color: #d23a4f; font-weight: 700; min-width: 130px; }
            QSlider::groove:horizontal { height: 6px; background: #2c3746; border-radius: 3px; }
            QSlider::handle:horizontal { width: 18px; margin: -6px 0; border-radius: 9px; background: #4cc9f0; }
            QTableWidget, QListWidget { background: #0d1117; border: 1px solid #263241; gridline-color: #263241; }
            QHeaderView::section { background: #1c2531; color: #b9c7d8; padding: 7px; border: none; }
            """
        )

    def _set_status(self, text: str, kind: str) -> None:
        styles = {
            "safe": "background:#123c2e;color:#b7f7d0;border:1px solid #2e8b65;border-radius:8px;font-weight:700;",
            "warn": "background:#493615;color:#ffd98a;border:1px solid #9b6a19;border-radius:8px;font-weight:700;",
            "stop": "background:#541b25;color:#ffc1ca;border:1px solid #c13b51;border-radius:8px;font-weight:700;",
        }
        self.status.setText(text)
        self.status.setStyleSheet(styles[kind])

    def _log(self, text: str) -> None:
        self.log.insertItem(0, text)
        while self.log.count() > 80:
            self.log.takeItem(self.log.count() - 1)

    def _absolute_from_sliders(self) -> np.ndarray:
        relative = np.array([self.sliders[name].value() / SCALE for name in JOINTS], dtype=float)
        return self.temporary_zero_deg + relative

    def _set_slider_ranges_and_values(self, relative_values: np.ndarray) -> None:
        self.updating_sliders = True
        try:
            for index, name in enumerate(JOINTS):
                if name == "J1":
                    lower, upper = DISPLAY_LIMITS[name]
                else:
                    absolute_lower, absolute_upper = DISPLAY_LIMITS[name]
                    lower = absolute_lower - self.temporary_zero_deg[index]
                    upper = absolute_upper - self.temporary_zero_deg[index]
                slider = self.sliders[name]
                slider.setMinimum(int(round(lower * SCALE)))
                slider.setMaximum(int(round(upper * SCALE)))
                slider.setValue(int(round(relative_values[index] * SCALE)))
                self.requested_labels[name].setText(f"{slider.value() / SCALE:.1f}°")
        finally:
            self.updating_sliders = False

    def _preview_requested(self, name: str) -> None:
        value = self.sliders[name].value() / SCALE
        self.requested_labels[name].setText(f"{value:.1f}°")

    def _apply_target(self, target_abs: np.ndarray, source: str) -> bool:
        if self.estopped:
            self._set_status("E-STOP / 已冻结，目标未执行", "stop")
            self._log("急停状态：拒绝执行目标。")
            return False
        result = self.guard.check_swept_deg(self.current_abs_deg, target_abs, max_step_deg=0.5)
        if not result["safe"]:
            self.guard._set_deg(self.current_abs_deg)
            relative = self.current_abs_deg - self.temporary_zero_deg
            self._set_slider_ranges_and_values(relative)
            reason = result["reason"]
            fraction = result.get("interpolation_fraction", 0.0)
            self._set_status(f"REJECTED / {reason} · 路径 {fraction * 100:.1f}% 处被拦截", "stop")
            pairs = [" ↔ ".join(row["proxy_pair"]) for row in result.get("contacts", [])[:3]]
            detail = "；".join(pairs) if pairs else "超出机械位置限位"
            self._log(f"拒绝 {source}：{reason}；{detail}")
            self._refresh_angle_table(rejected=True)
            return False
        self.current_abs_deg = np.asarray(target_abs, dtype=float)
        self.guard._set_deg(self.current_abs_deg)
        self._set_status("SAFE / 目标已执行，无活动碰撞", "safe")
        self._log(
            source + "：" + ", ".join(f"{name}={value:.1f}°" for name, value in zip(JOINTS, self.current_abs_deg))
        )
        self._refresh_angle_table()
        if self.viewer is not None and self.viewer.is_running():
            self.viewer.sync()
        return True

    def _apply_requested_pose(self) -> None:
        if self.updating_sliders:
            return
        self._apply_target(self._absolute_from_sliders(), "滑条目标")

    def _set_temporary_zero(self) -> None:
        self.temporary_zero_deg = self.current_abs_deg.copy()
        payload = {
            "schema": "go-m8010-arm-v15.13-gui-default-zero/1.0",
            "saved_at_utc": datetime.now(timezone.utc).isoformat(),
            "semantics": "GUI startup posture and slider-relative zero only; CAD/MJCF/URDF/hardware encoder zeros are unchanged.",
            "cad_absolute_deg": {
                name: float(value) for name, value in zip(JOINTS, self.temporary_zero_deg)
            },
        }
        DEFAULT_ZERO_CONFIG.write_text(
            json.dumps(payload, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )
        self._set_slider_ranges_and_values(np.zeros(6))
        self.zero_posture.setText(
            "GUI启动0对应 CAD 姿态："
            + ", ".join(f"{name}={value:.3f}°" for name, value in zip(JOINTS, self.temporary_zero_deg))
        )
        self._set_status("DEFAULT ZERO / 当前安全姿态已保存为启动 0°", "warn")
        self._log("GUI启动0已保存；模型文件和实机编码器零位均未修改。")
        self._refresh_angle_table()

    def _clear_temporary_zero(self) -> None:
        self.temporary_zero_deg[:] = 0.0
        self._set_slider_ranges_and_values(self.current_abs_deg)
        self.zero_posture.setText(
            "临时0已清除；当前采用 CAD 机械零位：J1=0.0°, J2=0.0°, J3=0.0°, J4=0.0°, J5=0.0°, J6=0.0°"
        )
        self._set_status("SAFE / 临时偏置已清除，姿态未移动", "safe")
        self._log("临时0偏置已清除；机械臂姿态保持不变。")
        self._refresh_angle_table()

    def _return_cad_zero(self) -> None:
        target = np.zeros(6, dtype=float)
        if self._apply_target(target, "回 CAD 机械零位"):
            self.temporary_zero_deg[:] = 0.0
            self._set_slider_ranges_and_values(np.zeros(6))
            self.zero_posture.setText(
                "当前采用 CAD 机械零位：J1=0.0°, J2=0.0°, J3=0.0°, J4=0.0°, J5=0.0°, J6=0.0°（J6 相机在上）"
            )
            self._refresh_angle_table()

    def _toggle_estop(self) -> None:
        self.estopped = not self.estopped
        if self.estopped:
            self.estop_button.setText("解除急停")
            self._set_status("E-STOP / 关节目标已冻结", "stop")
            self._log("急停已触发：保持当前 qpos，不执行新目标。")
        else:
            self.estop_button.setText("急停 / 冻结")
            self._set_status("SAFE / 急停已解除", "safe")
            self._log("急停已解除。")

    def _refresh_angle_table(self, rejected: bool = False) -> None:
        relative = self.current_abs_deg - self.temporary_zero_deg
        for row, name in enumerate(JOINTS):
            if name == "J1":
                limit_text = "continuous"
            else:
                lower, upper = DISPLAY_LIMITS[name]
                limit_text = f"{lower:g}° … {upper:g}°"
            values = [
                name,
                f"{self.current_abs_deg[row]:.3f}°",
                f"{relative[row]:.2f}°",
                limit_text,
                "保持上一安全姿态" if rejected else "SAFE",
            ]
            for column, value in enumerate(values):
                item = QTableWidgetItem(value)
                item.setTextAlignment(Qt.AlignmentFlag.AlignCenter)
                self.table.setItem(row, column, item)

    def _sync_viewer(self) -> None:
        if self.viewer is None:
            return
        if not self.viewer.is_running():
            self.close()
            return
        self.viewer.sync()

    def closeEvent(self, event: QCloseEvent) -> None:
        self.sync_timer.stop()
        if self.viewer is not None:
            self.viewer.close()
        event.accept()


def self_test() -> int:
    model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    data = mujoco.MjData(model)
    guard = KinematicGuard(MODEL_XML, model=model, data=data)
    zero = guard.check_pose_deg([0.0] * 6)
    outside = guard.check_pose_deg([0.0, 171.0, 0.0, 0.0, 0.0, 0.0])
    floor = guard.check_pose_deg([0.0, -90.0, 0.0, 0.0, 0.0, 0.0])
    passed = zero["safe"] and outside["reason"] == "position_limit" and floor["reason"] == "ground_collision"
    print(
        json.dumps(
            {
                "status": "PASS" if passed else "FAIL",
                "shared_model": guard.model is model,
                "shared_data": guard.data is data,
                "zero": zero["reason"],
                "outside_J2": outside["reason"],
                "J2_minus_90": floor["reason"],
            },
            ensure_ascii=False,
        )
    )
    return 0 if passed else 2


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        return self_test()

    model = mujoco.MjModel.from_xml_path(str(MODEL_XML))
    data = mujoco.MjData(model)
    startup_zero_deg = load_default_zero_deg()
    startup_guard = KinematicGuard(MODEL_XML, model=model, data=data)
    startup_result = startup_guard.check_pose_deg(startup_zero_deg)
    if not startup_result["safe"]:
        raise RuntimeError(
            "Configured GUI default zero is not safe: "
            + json.dumps(startup_result, ensure_ascii=False)
        )
    app = QApplication(sys.argv)
    app.setFont(QFont("Noto Sans CJK SC", 10))
    viewer = mujoco.viewer.launch_passive(
        model,
        data,
        show_left_ui=False,
        show_right_ui=False,
    )
    viewer.cam.lookat[:] = (0.0, 0.0, 0.30)
    viewer.cam.distance = 1.18
    viewer.cam.azimuth = 135.0
    viewer.cam.elevation = -18.0
    viewer.opt.geomgroup[:] = 0
    viewer.opt.geomgroup[0] = 1  # physical grid ground
    viewer.opt.geomgroup[2] = 1  # audited visual STL chunks
    viewer.opt.sitegroup[:] = 0
    window = JointControlWindow(model, data, viewer, startup_zero_deg)
    available = app.primaryScreen().availableGeometry()
    width = min(970, max(760, available.width() // 2 - 24))
    height = min(1180, available.height() - 30)
    window.resize(width, height)
    window.move(available.right() - width - 10, available.top() + 10)
    window.show()
    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())

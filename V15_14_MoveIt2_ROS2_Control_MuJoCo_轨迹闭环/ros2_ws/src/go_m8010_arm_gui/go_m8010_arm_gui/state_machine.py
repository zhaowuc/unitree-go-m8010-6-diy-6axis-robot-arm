"""与 Qt 无关的互斥模式和到位判定。"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from typing import Optional


class ArmMode(str, Enum):
    BRAKE = "brake"
    REAL_TO_SIM = "real_to_sim"
    SIM_TO_REAL = "sim_to_real"
    POSITION = "position"
    DRAG = "drag"
    HOLD = "hold"


MODE_TEXT = {
    ArmMode.BRAKE: "停止／制动",
    ArmMode.REAL_TO_SIM: "现实驱动虚拟",
    ArmMode.SIM_TO_REAL: "虚拟驱动现实",
    ArmMode.POSITION: "位置控制",
    ArmMode.DRAG: "可拖动",
    ArmMode.HOLD: "保持当前位置",
}


@dataclass
class ArrivalTracker:
    tolerance_rad: float
    dwell_s: float
    timeout_s: float
    started_at: Optional[float] = None
    inside_since: list[Optional[float]] = field(default_factory=lambda: [None] * 6)
    reached_once: list[bool] = field(default_factory=lambda: [False] * 6)

    def start(self, now: float) -> None:
        self.started_at = now
        self.inside_since = [None] * 6
        self.reached_once = [False] * 6

    def update(self, now: float, errors: list[float], connected: list[bool]) -> list[str]:
        result: list[str] = []
        for index, error in enumerate(errors):
            if not connected[index]:
                result.append("未连接")
                self.inside_since[index] = None
                continue
            if abs(error) <= self.tolerance_rad:
                if self.inside_since[index] is None:
                    self.inside_since[index] = now
                if now - self.inside_since[index] >= self.dwell_s:
                    result.append("保持中" if self.reached_once[index] else "已到位")
                    self.reached_once[index] = True
                else:
                    result.append("运动中")
            else:
                self.inside_since[index] = None
                timed_out = self.started_at is not None and now - self.started_at >= self.timeout_s
                result.append("未到位" if timed_out else "运动中")
        return result


class ModeMachine:
    def __init__(self) -> None:
        self.mode = ArmMode.REAL_TO_SIM
        self.torque_enabled = False

    def stop(self) -> None:
        self.mode = ArmMode.BRAKE
        self.torque_enabled = False

    def drag(self) -> None:
        self.mode = ArmMode.DRAG
        self.torque_enabled = False

    def real_to_sim(self) -> None:
        self.mode = ArmMode.REAL_TO_SIM

    def sim_to_real(self) -> None:
        self.mode = ArmMode.SIM_TO_REAL

    def torque_on(self) -> None:
        self.mode = ArmMode.HOLD
        self.torque_enabled = True

    def hold(self) -> None:
        self.mode = ArmMode.HOLD
        self.torque_enabled = True

    def position(self) -> None:
        self.mode = ArmMode.POSITION
        self.torque_enabled = True

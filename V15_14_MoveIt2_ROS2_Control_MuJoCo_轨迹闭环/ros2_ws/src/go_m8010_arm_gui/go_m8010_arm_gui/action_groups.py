"""Validated model-angle action files and a Qt/ROS-independent sequencer.

The caller owns preview authority, actual arrival checks, and hardware transport.
A gripper callback acknowledges only a sent command, never a successful grasp.
"""

from __future__ import annotations

import json
import os
import tempfile
import time
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Callable

from .workflow_contract import (
    ContractViolation,
    _strict_finite_float,
    _validated_sha256,
    finite_joint_vector,
    validated_joint_limits,
)


ACTION_GROUP_SCHEMA = "go-m8010-action-group/1.0"
PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG = (
    (-180.0, 180.0), (-170.0, 170.0), (-170.0, 170.0),
    (-116.0, 159.0), (-70.6, 151.2), (-180.0, 180.0),
)


@dataclass(frozen=True)
class ActionStep:
    target_deg: tuple[float, ...]
    speed_deg_s: float = 1.0
    dwell_s: float = 0.0
    gripper: str = "none"
    opening_percent: float | None = None
    gripper_wait_s: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(self, "target_deg", finite_joint_vector(
            self.target_deg, "target_deg",
        ))
        for name, maximum in (("speed_deg_s", 3.0), ("dwell_s", 600.0),
                              ("gripper_wait_s", 30.0)):
            value = _strict_finite_float(getattr(self, name), name)
            if not 0.0 <= value <= maximum or (name == "speed_deg_s" and value == 0):
                raise ContractViolation(f"{name} is outside its permitted range")
            object.__setattr__(self, name, value)
        if not isinstance(self.gripper, str) or self.gripper not in (
            "none", "open", "close", "set",
        ):
            raise ContractViolation("gripper must be none, open, close, or set")
        if self.gripper == "set":
            opening = _strict_finite_float(self.opening_percent, "opening_percent")
            if not 0.0 <= opening <= 100.0:
                raise ContractViolation("opening_percent must be within [0, 100]")
            object.__setattr__(self, "opening_percent", opening)
        elif self.opening_percent is not None:
            raise ContractViolation("opening_percent is only valid for gripper=set")
        self.validate_limits()

    def validate_limits(self, joint_limits_deg=PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG) -> None:
        for index, (angle, bounds) in enumerate(zip(
            self.target_deg, validated_joint_limits(joint_limits_deg),
        )):
            if not bounds[0] <= angle <= bounds[1]:
                raise ContractViolation(f"target_deg J{index + 1} exceeds model limits")


@dataclass(frozen=True)
class ActionGroup:
    name: str
    steps: tuple[ActionStep, ...]
    model_sha256: str
    schema: str = ACTION_GROUP_SCHEMA
    coordinate_frame: str = "model_absolute_deg"

    def __post_init__(self) -> None:
        if not isinstance(self.name, str) or not self.name.strip() or len(self.name) > 128:
            raise ContractViolation("action group name must contain 1 to 128 characters")
        if self.schema != ACTION_GROUP_SCHEMA:
            raise ContractViolation("unsupported action group schema")
        if self.coordinate_frame != "model_absolute_deg":
            raise ContractViolation("action targets must use model_absolute_deg")
        _validated_sha256(self.model_sha256, "model_sha256")
        if not isinstance(self.steps, (tuple, list)) or not 1 <= len(self.steps) <= 1000:
            raise ContractViolation("action group must contain 1 to 1000 steps")
        if not all(isinstance(step, ActionStep) for step in self.steps):
            raise ContractViolation("steps must contain ActionStep values")
        object.__setattr__(self, "steps", tuple(self.steps))

    def to_dict(self) -> dict:
        return {
            "schema": self.schema, "name": self.name,
            "coordinate_frame": self.coordinate_frame,
            "model_sha256": self.model_sha256,
            "steps": [dict(asdict(step), target_deg=list(step.target_deg))
                      for step in self.steps],
        }

    @classmethod
    def from_dict(cls, data: object, *,
                  joint_limits_deg=PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG,
                  expected_model_sha256: str | None = None) -> "ActionGroup":
        fields = {"schema", "name", "coordinate_frame", "model_sha256", "steps"}
        if not isinstance(data, dict) or set(data) != fields:
            raise ContractViolation("action group fields do not match its schema")
        if not isinstance(data["steps"], list):
            raise ContractViolation("steps must be a JSON array")
        if expected_model_sha256 is not None:
            _validated_sha256(expected_model_sha256, "expected_model_sha256")
            if data["model_sha256"] != expected_model_sha256:
                raise ContractViolation("action group model does not match current model")
        step_fields = set(ActionStep.__dataclass_fields__)
        steps = []
        for item in data["steps"]:
            if not isinstance(item, dict) or set(item) != step_fields:
                raise ContractViolation("action step fields do not match its schema")
            if not isinstance(item["target_deg"], list):
                raise ContractViolation("target_deg must be a JSON array")
            step = ActionStep(**item)
            step.validate_limits(joint_limits_deg)
            steps.append(step)
        return cls(data["name"], tuple(steps), data["model_sha256"],
                   data["schema"], data["coordinate_frame"])

    @classmethod
    def load(cls, path: str | Path, **validation) -> "ActionGroup":
        def unique_object(pairs):
            result = {}
            for key, value in pairs:
                if key in result:
                    raise ContractViolation(f"duplicate JSON field: {key}")
                result[key] = value
            return result

        with Path(path).open(encoding="utf-8") as stream:
            data = json.load(stream, object_pairs_hook=unique_object)
        return cls.from_dict(data, **validation)

    def save(self, path: str | Path) -> None:
        """Replace only after a complete UTF-8 file has been flushed to disk."""
        destination = Path(path)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(
                mode="w", encoding="utf-8", newline="\n", dir=destination.parent,
                prefix=f".{destination.name}.", suffix=".tmp", delete=False,
            ) as stream:
                temporary = Path(stream.name)
                json.dump(self.to_dict(), stream, ensure_ascii=False, allow_nan=False, indent=2)
                stream.write("\n")
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, destination)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)


class ActionGroupRunner:
    """One run; terminal faults require a new runner and explicit start.

    ``move_start`` starts the caller's preview/submit workflow; ``move_status``
    returns complete only after measured arrival, never merely after dispatch.
    Pausing a move withdraws the old motion and resumes by replanning that step.
    Waiting pauses retain their remaining time and never repeat a gripper command.
    """

    def __init__(self, group: ActionGroup,
                 move_start: Callable[[ActionStep], object],
                 move_status: Callable[[], str], stop_motion: Callable[[], object],
                 gripper_command: Callable[[ActionStep], str],
                 health_check: Callable[[], None], *,
                 now: Callable[[], float] = time.monotonic,
                 motion_timeout_s: float = 900.0) -> None:
        if not isinstance(group, ActionGroup):
            raise ContractViolation("group must be an ActionGroup")
        timeout = _strict_finite_float(motion_timeout_s, "motion_timeout_s")
        if timeout <= 0:
            raise ContractViolation("motion_timeout_s must be positive")
        self.group = group
        self._move_start = move_start
        self._move_status = move_status
        self._stop_motion = stop_motion
        self._gripper_command = gripper_command
        self._health_check = health_check
        self._now = now
        self.motion_timeout_s = timeout
        self.state = "idle"
        self.detail = "动作组未开始"
        self.index = 0
        self.result: dict | None = None
        self.events: list[dict] = []
        self._deadline = 0.0
        self._remaining = 0.0
        self._paused_state = ""

    @property
    def active(self) -> bool:
        return self.state in {"moving", "gripper_wait", "dwell", "paused"}

    def _event(self, kind: str, **values) -> None:
        self.events.append(dict(event=kind, at_monotonic_s=self._now(),
                                index=self.index, **values))

    def _finish(self, state: str, detail: str) -> None:
        self.state, self.detail = state, detail
        self._event(state, detail=detail)
        self.result = dict(state=state, detail=detail, completed_steps=self.index,
                           total_steps=len(self.group.steps))

    def _fail(self, error: Exception) -> None:
        detail = str(error)
        try:
            self._stop_motion()
        except Exception as stop_error:
            detail += f"；停止命令失败：{stop_error}"
        self._finish("failed", detail)

    def _begin_move(self, timeout_s: float) -> None:
        step = self.group.steps[self.index]
        self.state, self.detail = "moving", f"执行第 {self.index + 1} 步：预演并等待实际到位"
        self._deadline = self._now() + timeout_s
        self._event("move_start", target_deg=list(step.target_deg))
        self._move_start(step)

    def start(self) -> None:
        if self.state != "idle":
            raise RuntimeError("an action group runner can only be started once")
        try:
            self._health_check()
            self._begin_move(self.motion_timeout_s)
        except Exception as error:
            self._fail(error)

    def tick(self) -> None:
        if not self.active:
            return
        try:
            self._health_check()
            if self.state == "paused":
                return
            now = self._now()
            step = self.group.steps[self.index]
            if self.state == "moving":
                if now >= self._deadline:
                    raise TimeoutError(f"第 {self.index + 1} 步运动超时，未确认实际到位")
                status = self._move_status()
                if status == "running":
                    return
                if status != "complete":
                    raise RuntimeError(f"invalid move status: {status!r}")
                self._event("move_complete")
                if step.gripper != "none":
                    receipt = self._gripper_command(step)
                    if not isinstance(receipt, str) or not receipt:
                        raise RuntimeError("夹爪命令未返回发送结果")
                    self._event("gripper_command_sent", command=step.gripper,
                                opening_percent=step.opening_percent, receipt=receipt)
                    self.state = "gripper_wait"
                    self.detail = f"夹爪命令已发送：{receipt}；等待不代表已抓住物体"
                    self._deadline = self._now() + step.gripper_wait_s
                    return
                self._begin_dwell(step)
            elif now >= self._deadline:
                if self.state == "gripper_wait":
                    self._begin_dwell(step)
                else:
                    self._event("step_complete")
                    self.index += 1
                    if self.index == len(self.group.steps):
                        self._finish("complete", "动作组完成；夹爪仅记录命令发送，未验证夹持")
                    else:
                        self._begin_move(self.motion_timeout_s)
        except Exception as error:
            self._fail(error)

    def _begin_dwell(self, step: ActionStep) -> None:
        self.state, self.detail = "dwell", f"第 {self.index + 1} 步停留"
        self._deadline = self._now() + step.dwell_s
        self._event("dwell_start", dwell_s=step.dwell_s)

    def pause(self) -> None:
        if not self.active or self.state == "paused":
            return
        try:
            self._health_check()
            self._paused_state = self.state
            self._remaining = max(0.0, self._deadline - self._now())
            if self.state == "moving":
                self._stop_motion()
            self.state, self.detail = "paused", "动作组已暂停"
            self._event("pause", phase=self._paused_state)
        except Exception as error:
            self._fail(error)

    def resume(self) -> None:
        if self.state != "paused":
            return
        try:
            self._health_check()
            self._event("resume", phase=self._paused_state)
            if self._paused_state == "moving":
                if self._remaining <= 0:
                    raise TimeoutError("暂停前运动已超时")
                self._begin_move(self._remaining)
            else:
                self.state = self._paused_state
                self.detail = "恢复剩余等待；不重发夹爪命令"
                self._deadline = self._now() + self._remaining
        except Exception as error:
            self._fail(error)

    def stop(self) -> None:
        try:
            self._stop_motion()
        except Exception as error:
            self._finish("failed", f"停止命令失败：{error}")
            return
        if self.state != "failed":
            self._finish("stopped", "动作组已停止；未发送额外夹爪动作")

"""Whole-arm outer admittance inside the existing GUI/Router/worker chain.

The existing bootstrap is reused. Normal end freezes the accepted references,
waits for all-domain HOLD, then uses the existing previewed position recipe to
return to the session's initial vertical pose before releasing drive authority.
"""
from collections import deque
from dataclasses import replace
import math
import time

from hand_guidance import GuidanceProfile, HandGuidance
from hand_guidance_feedback import (MOTOR_GROUPS, GuidanceMeasurementUnavailable,
                                    matched_observation, stationary_bias)
from v15_31d_gui_j1_demo import J1Demo


def accepted_guidance_targets(hardware, epoch, now_ns):
    if (not isinstance(hardware, dict) or hardware.get("healthy") is not True
            or type(hardware.get("source_monotonic_ns")) is not int
            or not 0 <= now_ns - hardware["source_monotonic_ns"] <= 250_000_000):
        return None
    targets = []
    for names in MOTOR_GROUPS:
        values = []
        for name in names:
            item = hardware.get("per_motor", {}).get(name, {})
            target = item.get("accepted_guidance_target_rad")
            if (item.get("accepted_guidance_metadata_status") != "OBSERVED"
                    or item.get("accepted_guidance_activation_epoch") != epoch
                    or type(target) not in (int, float) or not math.isfinite(target)):
                return None
            values.append(float(target))
        if any(value != values[0] for value in values):
            return None
        targets.append(values[0])
    return targets


class GuidanceDemo(J1Demo):
    def __init__(self, *args, gui, shadow_only=False, guide_speed_deg_s=30.0, **kwargs):
        super().__init__(*args, **kwargs)
        self.gui, self.shadow_only = gui, shadow_only
        self.profile = GuidanceProfile(speed_deg_s=guide_speed_deg_s)
        self.guidance_phase = "calibrating"
        self.guidance_trace, self.guidance_events = [], []
        self.baseline = deque(maxlen=150)
        self.window.node.guidance_hardware_history = deque(maxlen=32)
        self.bias = self.filtered_residual = self.core = None
        self.input_wait_since = None
        self.last_trace_at = 0.0
        self.ending = False
        self.return_verified = False
        self.hold_target = None
        self.freeze_reference = None
        self.hold_requested = False
        self.guide_epoch = None
        self.guidance_fault = None
        self.pending_release = False
        self.return_quiet_since = None
        self.shadow_started_at = None
        toolbar = self.window.teach_toolbar
        for action in toolbar.actions():
            action.setVisible(False)
        self.drag_button = gui.QPushButton("按住：整臂柔顺拖动" if not shadow_only else "按住：仅观察手力与拟输出")
        self.drag_button.setEnabled(False)
        self.drag_button.pressed.connect(self.start_guidance)
        self.drag_button.released.connect(self.release_guidance)
        toolbar.addWidget(self.drag_button)
        self.record_button = gui.QPushButton("记录当前姿态")
        self.record_button.clicked.connect(self.record_pose)
        toolbar.addWidget(self.record_button)
        self.status = gui.QLabel("准备承重保持，手请离开机械臂；完成后再拖动。")
        self.status.setWordWrap(True)
        toolbar.addWidget(self.status)

    def reference(self, velocities, *, freeze=False):
        return {"schema": "go-m8010-hand-guidance-reference/1.0", "origin_rad": list(self.origin),
            "velocity_rad_s": list(velocities), "freeze_reference": freeze, "maximum_velocity_deg_s": 30.0,
            "maximum_excursion_deg": 10.0, "maximum_reference_error_deg": 2.0}

    def start_guidance(self):
        if self.guidance_phase != "ready" or self.ending or self.bias is None:
            return
        try:
            self.window._action_group_health()
            if not self.window._action_group_hold_ready(self.window.command_targets):
                self.status.setText("等待六轴静止保持。")
                return
            if any(abs(q-origin) > math.radians(10.0) for q,origin in zip(self.window.command_targets, self.origin)):
                raise RuntimeError("当前姿态不在本轮手导范围内，请先回到起始姿态")
            if (self.window.node.latest_gravity_status or {}).get("empirical_validation", {}).get("hand_guidance_authorized") is not True:
                raise RuntimeError("当前会话尚未开放整臂柔顺")
            self.filtered_residual = [0.0] * 6
            self.input_wait_since = None
            self.pending_release = False
            self.guidance_phase, self.guidance_fault = "engaging_guidance", None
            if not self.shadow_only:
                self.window.hand_guidance_reference = self.reference([0.0] * 6)
                self.window.hardware_mode = "teach"
                self.window.moving_joint_mask = [True] * 6
                self.window._authorize_active_joints([True] * 6)
                self.guide_epoch = self.window.activation_epoch
                self.window._set_virtual_editable(False)
                for button in self.window.action_group_manual_buttons:
                    button.setEnabled(False)
            else:
                self.core = HandGuidance(self.profile, self.window.command_targets, self.now())
                self.last_observation_at = self.now()
                self.guidance_phase = "guiding"
            self.guidance_events.append({"event": "guidance_started", "at_monotonic_s": self.now(),
                "shadow_only": self.shadow_only, "epoch": self.guide_epoch})
        except (ValueError, RuntimeError) as error:
            self.status.setText("未进入柔顺：" + str(error))

    def release_guidance(self):
        if self.guidance_phase not in {"guiding", "engaging_guidance"}:
            return
        if self.guidance_phase == "engaging_guidance":
            # No admittance reference moves during admission. A quick click
            # waits for the unchanged initial target ACK, then freezes it.
            self.pending_release = True
            return
        self.guidance_events.append({"event": "guidance_release", "at_monotonic_s": self.now()})
        self.drag_button.setEnabled(False)
        if self.shadow_only:
            self.guidance_phase = "ready"
            self.drag_button.setEnabled(not self.ending)
            return
        self.guidance_phase = "freezing"
        self.freeze_reference = None
        self.freeze_sent_ns = int(self.now() * 1e9)
        self.window.hand_guidance_reference = self.reference([0.0] * 6, freeze=True)
        self.status.setText("正在保持各轴最后接受的目标。")

    def record_pose(self):
        if self.guidance_phase != "ready" or not self.window._action_group_hold_ready(self.window.command_targets):
            self.status.setText("先松开按钮，等六轴停住后记录。")
            return
        self.window.teach_record_target = tuple(self.window.command_targets)
        self.window._record_teach_point()

    def stop(self, reason=None):
        if not self.interactive_ready or self.return_verified:
            return super().stop(reason)
        self.ending = True
        self.drag_button.setEnabled(False)
        if reason:
            self.guidance_fault = reason
        self.release_guidance()
        self.status.setText("结束柔顺并保持；请松手，静止后将回到本次起始竖直姿态。")

    def _freeze_and_hold(self):
        hardware = self.window.node.latest_hardware or {}
        target = accepted_guidance_targets(hardware, self.guide_epoch, time.monotonic_ns())
        if target is None:
            return
        if any(hardware["per_motor"][name].get("guidance_paused_reason") not in {"GUI_RELEASE", "DEADMAN_TIMEOUT"}
               or hardware["per_motor"][name]["feedback_source_monotonic_ns"] <= self.freeze_sent_ns
               for names in MOTOR_GROUPS for name in names):
            return
        self.freeze_reference = target
        self.window.command_targets = list(target)
        self.window.targets = self.window.candidate_targets = list(target)
        self.window.hardware_mode = "hold"
        self.window.machine.hold()
        self.window.moving_joint_mask = [False] * 6
        self.window._authorize_active_joints([True] * 6)
        self.hold_target = list(target)
        self.guidance_phase = "hold_barrier"

    def _begin_return(self, sample):
        if (not sample["stationary_hold_ready"] or not sample["router_hold_fresh"]
                or sample.get("j6_drive_state") != 1):
            return
        if self.bias is not None:
            observation = matched_observation(self.window.node.guidance_hardware_history,
                self.window.node.latest_gravity_status, now_ns=time.monotonic_ns())
            if observation is None or any(abs(value-bias) > threshold for value,bias,threshold in
                    zip(observation.residual_nm, self.bias, self.profile.engage_torque_nm)):
                self.return_quiet_since = None
                self.status.setText("保持当前姿态，等待松手后回位。")
                return
            self.return_quiet_since = self.now() if self.return_quiet_since is None else self.return_quiet_since
            if self.now() - self.return_quiet_since < 0.5:
                return
        if max(abs(a-b) for a,b in zip(sample["actual_rad"], self.origin)) <= math.radians(0.25):
            self.return_verified = True
            super().stop(self.guidance_fault)
            return
        self.window.hand_guidance_reference = None
        self.window._set_virtual_editable(True)
        self.stage, self.guidance_phase = "return_center", "returning"
        self.dialog = self.start_center(self.origin)
        self.status.setText("正在按已预演轨迹回到起始竖直姿态，完成后才停止驱动。")

    def check_return_recipe(self, recipe):
        if (self.guidance_phase != "returning"
                or any(abs(a-b) > 1e-10 for a,b in zip(recipe.target_rad, self.origin))):
            raise RuntimeError("guidance session only authorizes return to its initial pose")
        result = []
        for segment in recipe.segments:
            moving = [i for i,(a,b) in enumerate(zip(segment.start_rad, segment.target_rad)) if a != b]
            if len(moving) != 1 or segment.profile.duration_s > 15:
                raise RuntimeError("guidance return must use the existing bounded single-axis recipes")
            if any(abs(a-b) > math.radians(5.0) + 1e-12 for a,b in zip(segment.start_rad, segment.target_rad)):
                raise RuntimeError("guidance return segment exceeds five degrees")
            if any(abs(q-origin) > math.radians(10.25) for pose in (segment.start_rad, segment.target_rad)
                   for q,origin in zip(pose, self.origin)):
                raise RuntimeError("guidance return left the demonstrated pose region")
            result.append({"moving_joints": [f"J{i+1}" for i in moving], "trajectory_sha256": segment.sha256})
        return result

    def _pause_input(self, now, reason, diagnostics=None):
        if self.guidance_phase != "guiding":
            self.status.setText("等待新鲜力矩反馈：" + reason)
            return
        if self.input_wait_since is None:
            self.input_wait_since = now
            self.guidance_events.append({"event": "input_wait", "reason": reason,
                "diagnostics": diagnostics, "at_monotonic_s": now})
        output = self.core.pause(now)
        self.filtered_residual = [0.0] * 6
        self.last_observation_at = now
        if not self.shadow_only:
            self.window.command_targets = list(output.q_ref)
            self.window.targets = self.window.candidate_targets = list(output.q_ref)
            self.window.hand_guidance_reference = self.reference(output.dq_ref)
        self.status.setText("保持当前目标，等待新鲜力矩反馈。")
        if now - self.input_wait_since >= self.profile.max_feedback_age_s:
            self.guidance_fault = "GUIDANCE_INPUT_GAP:" + reason
            self.guidance_events.append({"event": "guidance_fault", "reason": self.guidance_fault,
                "diagnostics": diagnostics, "at_monotonic_s": now})
            print("HAND_GUIDANCE_FAULT=" + self.guidance_fault, flush=True)
            self.release_guidance()
            if self.shadow_only:
                self.stop(self.guidance_fault)

    def tick(self):
        if not self.interactive_ready or self.terminal is not None:
            return super().tick()
        now = self.now()
        try:
            sample = self.observe()
            self.samples.append({**sample, "stage": "hand_guidance", "guidance_phase": self.guidance_phase})
            if self.window.hardware_mode in {"brake", "drag"}:
                self.window.hand_guidance_reference = None
                super().stop("operator explicitly withdrew drive authority")
                return
            if (not sample["healthy"] or not sample["feedback_fresh"] or not sample["authority"]):
                # A genuine loss of actuator/feedback authority cannot be
                # repaired by inventing a held pose or extending an old lease.
                self.return_verified = False
                super().stop("hand guidance lost hardware/feedback/gravity authority")
                return
            if now - self.started >= 550 and not self.ending:
                self.stop()
            if self.shadow_started_at is not None and now - self.shadow_started_at >= 15.0 and not self.ending:
                if not self.guidance_trace:
                    self.guidance_fault = self.guidance_fault or "shadow observation produced no reference samples"
                elif max(abs(q-start) for row in self.guidance_trace
                         for q,start in zip(row["q_reference"], self.origin)) > math.radians(0.25):
                    self.guidance_fault = "unforced shadow reference drift exceeded 0.25 degrees"
                self.stop()
            if self.guidance_phase in {"guiding", "engaging_guidance"} and (
                    self.shadow_started_at is None and (not self.drag_button.isDown() or not self.window.isActiveWindow())):
                self.release_guidance()
            if self.guidance_phase == "returning":
                runner = self.dialog.runner
                if runner is None or runner.state in {"failed", "stopped"}:
                    self.guidance_phase = "return_blocked"
                    self.guidance_fault = "return recipe failed; retain current HOLD and require operator action"
                    self.status.setText("回位未完成，保持当前姿态；查看具体轨迹错误后处理。")
                    return
                if runner.state == "complete" and sample["stationary_hold_ready"] and sample["router_hold_fresh"]:
                    if max(abs(a-b) for a,b in zip(sample["actual_rad"], self.origin)) <= math.radians(0.25):
                        self.return_verified = True
                        self.guidance_events.append({"event": "initial_vertical_return_verified", "actual_rad": sample["actual_rad"]})
                        super().stop(self.guidance_fault)
                return
            if self.guidance_phase == "freezing":
                self._freeze_and_hold()
                return
            if self.guidance_phase == "hold_barrier":
                accepted = accepted_guidance_targets(self.window.node.latest_hardware,
                    self.window.activation_epoch, time.monotonic_ns())
                if accepted == self.hold_target and self.window._action_group_hold_ready(self.hold_target):
                    self.guidance_phase = "ready"
                    self.window.teach_record_target = tuple(self.hold_target)
                    self.drag_button.setEnabled(not self.ending)
                    self.status.setText("六轴保持已确认，可以再次施力拖动或记录姿态。")
                return
            if self.guidance_phase == "engaging_guidance":
                accepted = accepted_guidance_targets(self.window.node.latest_hardware, self.guide_epoch, time.monotonic_ns())
                if accepted == list(self.window.command_targets) and all(mode == "teach" for mode in sample["modes"].values()):
                    self.core = HandGuidance(self.profile, self.window.command_targets, now)
                    self.last_observation_at = now
                    self.guidance_phase = "guiding"
                    self.guidance_events.append({"event": "guidance_hardware_ack", "at_monotonic_s": now, "epoch": self.guide_epoch})
                    if self.pending_release:
                        self.release_guidance()
                return
            if self.ending and self.guidance_phase in {"ready", "calibrating"}:
                self._begin_return(sample)
                return
            diagnostics = None
            reason = "MATCHING_SNAPSHOT_PENDING"
            try:
                observation = matched_observation(self.window.node.guidance_hardware_history,
                    self.window.node.latest_gravity_status, now_ns=time.monotonic_ns())
            except GuidanceMeasurementUnavailable as error:
                observation, reason, diagnostics = None, error.reason, error.diagnostics
            if observation is None:
                self._pause_input(now, reason, diagnostics)
                return
            if self.input_wait_since is not None:
                if now - self.input_wait_since >= self.profile.max_feedback_age_s:
                    self._pause_input(now, "RECOVERY_AFTER_INPUT_GAP_LIMIT")
                    return
                self.guidance_events.append({"event": "input_resumed", "wait_s": now-self.input_wait_since,
                    "at_monotonic_s": now})
                self.core.pause(now)
                self.last_observation_at = now
                self.input_wait_since = None
                return
            if self.bias is None:
                self.baseline.append(observation)
                try:
                    self.bias, noise = stationary_bias(self.baseline, operator_hands_off=True)
                except ValueError:
                    return
                self.profile = replace(self.profile,
                    engage_torque_nm=tuple(max(a, 4*b) for a,b in zip(self.profile.engage_torque_nm, noise)))
                self.guidance_phase = "ready"
                self.drag_button.setEnabled(True)
                self.guidance_events.append({"event": "stationary_bias", "bias_nm": self.bias, "noise_mad_nm": noise})
                self.status.setText("已就绪：按住按钮后手推移动，撤去手力停住；松开按钮保持。")
                print(f"HAND_GUIDANCE_READY shadow_only={self.shadow_only} reference_speed_deg_s={self.profile.speed_deg_s:g}", flush=True)
                if self.shadow_only:
                    self.shadow_started_at = now
                    self.core = HandGuidance(self.profile, self.window.command_targets, now)
                    self.filtered_residual, self.last_observation_at = [0.0] * 6, now
                    self.guidance_phase = "guiding"
                    self.drag_button.setEnabled(False)
                    self.status.setText("15秒静置观察：请勿施力，实际目标保持不变。")
                    return
            if self.guidance_phase != "guiding":
                return
            self.window._action_group_health(allowed_modes=("hold", "teach"))
            raw_residual = [value-bias for value,bias in zip(observation.residual_nm, self.bias)]
            dt = now - getattr(self, "last_observation_at", now-0.02)
            alpha = max(0.0, min(1.0, dt / (0.05 + dt)))
            self.filtered_residual = [old + alpha*(new-old) for old,new in zip(self.filtered_residual, raw_residual)]
            output = self.core.update(now, observation.q, observation.dq, self.filtered_residual,
                                      source_time_s=observation.source_monotonic_ns * 1e-9)
            self.last_observation_at = now
            if output.fault:
                self.guidance_fault = output.fault
                self.guidance_events.append({"event": "guidance_fault", "reason": output.fault, "at_monotonic_s": now})
                print("HAND_GUIDANCE_FAULT=" + output.fault, flush=True)
                self.release_guidance()
                self.status.setText("手导暂停并保持：" + output.fault)
                if self.shadow_only:
                    self.stop(output.fault)
                return
            if any(abs(target-origin) > math.radians(10.0) for target,origin in zip(output.q_ref, self.origin)):
                self.release_guidance()
                self.status.setText("达到本轮拖动范围，已请求保持。")
                return
            if not self.shadow_only:
                self.window.command_targets = list(output.q_ref)
                self.window.targets = self.window.candidate_targets = list(output.q_ref)
                self.window.hand_guidance_reference = self.reference(output.dq_ref)
            if now - self.last_trace_at >= 0.05:
                self.guidance_trace.append({"at_monotonic_s": now, "source_monotonic_ns": observation.source_monotonic_ns,
                    "q_actual": observation.q, "q_reference": output.q_ref, "dq_reference": output.dq_ref,
                    "external_torque_estimate_nm": tuple(self.filtered_residual), "state": output.state,
                    "shadow_only": self.shadow_only})
                self.last_trace_at = now
            label = {"guiding": "随你的施力移动", "settling": "撤力减速中", "holding": "已停住，再施力可继续"}[output.state]
            self.status.setText(("仅观察拟输出：" if self.shadow_only else "整臂柔顺：") + label)
        except GuidanceMeasurementUnavailable as error:
            self._pause_input(now, error.reason, error.diagnostics)
        except (ValueError, RuntimeError) as error:
            if self.guidance_phase == "guiding":
                self.guidance_fault = str(error)
                self.guidance_events.append({"event": "guidance_fault", "reason": str(error), "at_monotonic_s": now,
                    "gravity_status": self.window.node.latest_gravity_status,
                    "j6_feedback": (self.window.node.latest_hardware or {}).get("per_motor", {}).get("J6")})
                print("HAND_GUIDANCE_FAULT=" + str(error), flush=True)
            self.release_guidance()
            self.status.setText("等待或保持：" + str(error))
            if self.shadow_only and self.guidance_fault:
                self.stop(self.guidance_fault)
        finally:
            if getattr(self.window, "hand_guidance_reference", None) is not None and self.terminal is None:
                # The generated reference and its source clock share the same
                # update time; GUI rendering cannot shorten the accepted dt.
                self.window.hand_guidance_source_monotonic_ns = int(now * 1e9)
                self.window._publish_command(guidance_owner=True)

    def result(self):
        result = super().result()
        result.update(schema="go-m8010-hand-guidance-session/1.0",
            scope="BOOTSTRAP_AND_VERIFIED_RETURN_AND_TERMINAL; manual effect requires measured guidance trace",
            shadow_only=self.shadow_only, guidance_events=self.guidance_events,
            guidance_trace=self.guidance_trace, guidance_fault=self.guidance_fault,
            initial_vertical_return_verified=self.return_verified)
        result["guidance_input_qualified"] = self.bias is not None
        if not self.return_verified or self.guidance_fault or self.bias is None:
            result["status"] = "FAIL"
        return result

#!/usr/bin/env python3
"""Bounded J1 action-group example using the production GUI.

Other axes retain their original final targets; existing sequential correction
segments are allowed only within 0.25 degrees and are recorded. No gains,
geometry, preview gates, or motor transport are replaced. --execute requires a
fresh empirical envelope: the final BRAKE spends it even after success. One to
ten cycles reuse one saved action file and the same initial HOLD target.
--symmetric visits +excursion and -excursion about that fixed midpoint;
--return-center adds a separate final move to the midpoint, not another cycle.
--recover-initial-first optionally returns all axes to the hash-bound original initial
pose before capturing the shared demonstration HOLD; no calibration is changed.
"""
from __future__ import annotations

import argparse
from collections import deque
import hashlib
import json
import math
from pathlib import Path
import signal
import sys
import time

from v15_31d_gui_hold_probe import HoldProbe, MOTORS, empirical_binding_matches, observe_gui

LEVELS = (0.0, 0.25, 0.5, 0.75, 1.0)
MAXIMUM_SECONDS = 180.0


def maximum_demo_seconds(cycles, excursion_deg=1.0, symmetric=False):
    if type(cycles) is not int or not 1 <= cycles <= 10:
        raise ValueError("cycles must be an integer from 1 to 10")
    if (type(excursion_deg) not in {int, float} or not math.isfinite(excursion_deg)
            or not 0 < excursion_deg <= 10):
        raise ValueError("excursion_deg must be finite and greater than 0 through 10")
    if type(symmetric) is not bool:
        raise ValueError("symmetric must be boolean")
    return 1500.0 if cycles > 3 or excursion_deg != 1.0 or symmetric else MAXIMUM_SECONDS + 45.0 * (cycles - 1)


def initial_brake_ready(sample):
    age = sample.get("source_age_ms")
    modes = sample.get("modes")
    return bool(sample.get("healthy") and sample.get("thermal_ready") and sample.get("authority")
        and sample.get("zero_ff_authority") and sample.get("router_ready")
        and type(age) in {int, float} and math.isfinite(age) and 0 <= age < 100
        and isinstance(modes, dict) and set(modes) == set(MOTORS)
        and all(mode == "brake" for mode in modes.values())
        and type(sample.get("j6_drive_state")) is int and sample["j6_drive_state"] == 0
        and type(sample.get("source_monotonic_ns")) is int and sample["source_monotonic_ns"] > 0
        and type(sample.get("j6_raw_sequence")) is int and sample["j6_raw_sequence"] > 0)


def check_recipe(recipe, origin, *, recover_initial=False):
    """Keep the five secondary corrections bounded without changing the recipe."""
    result = []
    for segment in recipe.segments:
        moving = [i for i, (a, b) in enumerate(zip(segment.start_rad, segment.target_rad)) if a != b]
        for i in range(6) if recover_initial else range(1, 6):
            limit = 5.0 if recover_initial else 0.25
            if max(abs(segment.start_rad[i] - origin[i]), abs(segment.target_rad[i] - origin[i]),
                   abs(segment.target_rad[i] - segment.start_rad[i])) > math.radians(limit) + 1e-12:
                raise RuntimeError(f"J{i + 1} correction exceeds {limit:g} degrees")
        if recover_initial and segment.profile.duration_s > 15.0:
            raise RuntimeError("initial-pose recovery segment exceeds 15 seconds")
        result.append({"moving_joints": [f"J{i + 1}" for i in moving],
                       "start_rad": list(segment.start_rad), "target_rad": list(segment.target_rad),
                       "trajectory_sha256": segment.sha256})
    return result


class J1Demo:
    def __init__(self, window, observe, confirm, set_scale, start_group, *, cycles=1,
                 excursion_deg=1.0, symmetric=False, speed_deg_s=1.0, return_center=False, start_center=None,
                 recover_initial_first=False, start_recovery=None, validate_recovery_start=None,
                 interactive_teach=False, now=time.monotonic):
        self.interactive_teach = interactive_teach
        self.interactive_ready = False
        self.maximum_seconds = 600.0 if interactive_teach else maximum_demo_seconds(cycles, excursion_deg, symmetric)
        if (type(speed_deg_s) not in {int, float} or not math.isfinite(speed_deg_s)
                or not 0 < speed_deg_s <= 3):
            raise ValueError("speed_deg_s must be finite and greater than 0 through 3")
        self.excursion_deg, self.symmetric, self.speed_deg_s = excursion_deg, symmetric, speed_deg_s
        self.return_center, self.start_center, self.center_return = return_center, start_center, None
        self.window, self.observe, self.confirm = window, observe, confirm
        self.set_scale, self.start_group, self.now = set_scale, start_group, now
        self.started = now()
        self.stage, self.failure, self.origin, self.identity = "readiness", None, None, None
        self.rejected = None
        self.initial_ready_sample = None
        self.pending_level, self.pending_since, self.future = None, 0.0, None
        self.last_confirmation = -float("inf")
        self.dialog, self.terminal = None, None
        self.action_group_file = None
        self.recover_initial_first, self.start_recovery = recover_initial_first, start_recovery
        self.validate_recovery_start = validate_recovery_start
        self.recovery_target = self.recovery_result = self.recovery_started = None
        self.initial_hold_target = None
        self.requested_cycles, self.cycle_number = cycles, 1
        self.cycle_results = []
        self.success = self.done = False
        self.samples, self.events = [], []

    def stop(self, reason=None):
        if self.terminal is not None:
            return
        record_failure = reason is not None and self.stage == "action_group"
        center_failure = reason is not None and self.stage == "return_center"
        recovery_failure = reason is not None and (self.stage in {"recover_initial", "recovery_hold"}
            or (self.recover_initial_first and self.recovery_result is None))
        self.failure = reason
        self.stage = "terminal"
        self.terminal = HoldProbe(self.window, self.observe, now=self.now)
        self.terminal.identity = self.identity
        self.terminal.failure = reason
        self.terminal._terminal()
        if record_failure:
            self._record_cycle("FAIL", failure=reason)
        if recovery_failure:
            self._record_recovery("FAIL", failure=reason)
        if center_failure:
            self._record_center("FAIL", failure=reason)

    def _record_center(self, status, **details):
        runner = self.dialog.runner if self.dialog is not None else None
        self.center_return = {"status": status, "target_rad": list(self.origin), **details,
            "runner_result": getattr(runner, "result", None),
            "action_group_events": list(runner.events) if runner is not None else [],
            "action_group_log": str(getattr(self.dialog, "log_path", "")),
            "recipe_events": [event for event in self.events
                              if event.get("event") == "checked_center_recipe_before_submit"]}

    def _record_recovery(self, status, **details):
        runner = self.dialog.runner if self.dialog is not None else None
        if self.recovery_result is None:
            self.recovery_result = {}
        self.recovery_result.update({"status": status, **details})
        if runner is not None:
            self.recovery_result.update({"runner_result": getattr(runner, "result", None),
                "action_group_events": list(runner.events),
                "action_group_log": str(getattr(self.dialog, "log_path", ""))})
        self.recovery_result["recipe_events"] = [event for event in self.events
            if event.get("event") == "checked_recovery_recipe_before_submit"]

    def _record_cycle(self, status, **details):
        runner = self.dialog.runner if self.dialog is not None else None
        log_path = getattr(self.dialog, "log_path", None)
        self.cycle_results.append({"cycle": self.cycle_number, "status": status,
            "out_deg": None, "return_error_deg": None, **details,
            "action_group_log": str(log_path) if log_path is not None else None,
            "runner_result": getattr(runner, "result", None),
            "action_group_events": list(runner.events) if runner is not None else [],
            "recipe_events": [event for event in self.events
                              if event.get("event") == "checked_recipe_before_submit"
                              and event.get("cycle") == self.cycle_number]})

    def tick(self):
        if self.done:
            return
        if self.terminal is not None:
            self.terminal.tick()
            self.done = self.terminal.done
            return
        try:
            now, sample = self.now(), self.observe()
            self.samples.append({**sample, "stage": self.stage,
                                 "cycle": self.cycle_number if self.stage == "action_group" else None})
            if now - self.started >= self.maximum_seconds - 3:
                if self.interactive_teach and self.interactive_ready:
                    self.stop()
                    return
                raise RuntimeError(f"{self.maximum_seconds - 3:g}-second active deadline reached")
            if self.stage in {"recover_initial", "recovery_hold"} and now - self.recovery_started >= 45:
                raise RuntimeError("initial-pose recovery exceeded its 45-second deadline")
            if self.origin is None:
                if now - self.started >= 20:
                    raise RuntimeError("current-pose HOLD readiness timed out")
                if not initial_brake_ready(sample):
                    self.initial_ready_sample = None
                    return
                previous = self.initial_ready_sample
                self.initial_ready_sample = sample
                if (previous is None or sample["identity"] != previous["identity"]
                        or sample["source_monotonic_ns"] <= previous["source_monotonic_ns"]
                        or sample["j6_raw_sequence"] <= previous["j6_raw_sequence"]):
                    return
                self.window._tick()
                sample = self.observe()
                if not initial_brake_ready(sample) or sample["identity"] != self.initial_ready_sample["identity"]:
                    self.initial_ready_sample = None
                    return
                if self.recover_initial_first:
                    self.samples.append({**sample, "stage": "recovery_preflight", "cycle": None})
                    self.validate_recovery_start(sample)
                self.window._hold_current()
                if self.window.hardware_mode != "hold" or self.window.command_stream_suspended:
                    raise RuntimeError("GUI rejected initial HOLD")
                self.origin = tuple(self.window.command_targets)
                self.initial_hold_target = self.origin
                self.identity, self.rejected = sample["identity"], sample["router_rejected_commands"]
                self.stage = "engaging"
                self.events.append({"event": "frozen_initial_hold", "target_rad": list(self.origin)})
                return
            if (not sample["healthy"] or not sample["thermal_ready"] or not sample["authority"] or sample["identity"] != self.identity
                    or sample["router_rejected_commands"] != self.rejected):
                raise RuntimeError("feedback, authority, session or command-router health changed")
            if self.stage == "interactive_teach":
                # Manual contact is expected here: never refresh the calibration
                # ladder's no_person_contact attestation during operator teaching.
                if self.window.hardware_mode in {"brake", "drag"} or self.window.command_stream_suspended:
                    self.stop("operator left the armed teaching session")
                return
            level = (
                self.pending_level
                if self.pending_level is not None and sample.get("stage_level") < self.pending_level
                else 1.0 if sample.get("stage_index") == 4 and sample.get("stage_complete") else None
            )
            if level is not None and now - self.last_confirmation >= 1.0:
                if self.confirm(level) is not False:
                    self.last_confirmation = now
            if self.stage == "engaging":
                if (sample["stationary_hold_ready"] and sample["router_hold_fresh"]
                        and type(sample["j6_drive_state"]) is int and sample["j6_drive_state"] == 1):
                    self.stage = "ladder"
                elif now - self.started >= 20:
                    raise RuntimeError("seven-motor stationary HOLD engagement timed out")
                return
            if self.stage == "ladder":
                if (tuple(self.window.command_targets) != self.origin or not sample["stationary_hold_ready"]
                        or not sample["router_hold_fresh"]
                        or max(abs(math.degrees(a - b)) for a, b in zip(sample["actual_rad"], self.origin)) > 0.25):
                    raise RuntimeError("ladder lost frozen-target stationary HOLD within 0.25 degrees")
                if sample["j6_drive_state"] is None:
                    return
                if type(sample["j6_drive_state"]) is not int or sample["j6_drive_state"] != 1:
                    raise RuntimeError("paired J6 raw feedback does not confirm enabled HOLD")
                if self.pending_level is not None:
                    if self.future is None and now - self.pending_since >= 0.2:
                        self.future = self.set_scale(self.pending_level)
                    if self.future is not None and self.future.done():
                        if not self.future.result().results or not all(item.successful for item in self.future.result().results):
                            raise RuntimeError("gravity parameter service rejected next level")
                        if sample["stage_level"] == self.pending_level:
                            self.events.append({"event": "stage_entered", "level": self.pending_level})
                            self.pending_level, self.future = None, None
                    if now - self.pending_since >= 18:
                        raise RuntimeError("gravity stage transition timed out")
                elif sample["stage_complete"]:
                    index = sample["stage_index"]
                    if index < 4:
                        self.pending_level, self.pending_since = LEVELS[index + 1], now
                        self.confirm(self.pending_level)
                        self.last_confirmation = now
                    elif sample["position_authorized"]:
                        self.window._tick()
                        if self.interactive_teach:
                            if not sample.get("assisted_teach_authorized"):
                                raise RuntimeError("explicit single-axis teaching authority is unavailable")
                            self.stage, self.interactive_ready = "interactive_teach", True
                            self.window.centralWidget().setEnabled(True)
                            self.events.append({"event": "interactive_teach_ready", "at_monotonic_s": now})
                        elif self.recover_initial_first:
                            self.stage, self.recovery_started = "recover_initial", now
                            self.recovery_result = {"status": "RUNNING", "initial_hold_target_rad": list(self.origin)}
                            self.dialog = self.start_recovery(self.origin)
                        else:
                            self.stage = "action_group"
                            self.dialog = self.start_group(self.origin)
                return
            if self.stage == "recovery_hold":
                if (sample["stationary_hold_ready"] and sample["router_hold_fresh"]
                        and type(sample["j6_drive_state"]) is int and sample["j6_drive_state"] == 1
                        and max(abs(math.degrees(a-b)) for a,b in zip(sample["actual_rad"], self.origin)) <= 0.25):
                    self._record_recovery("PASS", action_group_origin_rad=list(self.origin))
                    self.stage, self.dialog = "action_group", None
                    self.dialog = self.start_group(self.origin)
                return
            runner = self.dialog.runner
            if runner is None or runner.state in {"failed", "stopped"}:
                raise RuntimeError("action group failed: " + (runner.detail if runner else "not started"))
            if runner.state == "complete":
                target = list(self.recovery_target if self.stage == "recover_initial" else self.origin)
                if self.stage == "action_group" and self.symmetric:
                    target[0] -= math.radians(self.excursion_deg)
                if (not sample["stationary_hold_ready"] or not sample["router_hold_fresh"]
                        or max(abs(math.degrees(a-b)) for a,b in zip(sample["actual_rad"], target)) > 0.25):
                    raise RuntimeError("final fresh HOLD pose no longer satisfies the return bound")
                if sample["j6_drive_state"] is None:
                    return
                if type(sample["j6_drive_state"]) is not int or sample["j6_drive_state"] != 1:
                    raise RuntimeError("final paired J6 raw feedback does not confirm enabled HOLD")
                arrivals = [item for item in runner.events if item["event"] == "measured_arrival"]
                if self.stage == "recover_initial":
                    if len(arrivals) != 1:
                        raise RuntimeError("one measured initial-pose recovery endpoint is required")
                    if (any(abs(a-b) > math.radians(0.25) for a,b in zip(self.initial_hold_target, target))
                            and not any(event.get("event") == "checked_recovery_recipe_before_submit" for event in self.events)):
                        raise RuntimeError("initial-pose recovery has no checked plan token")
                    self._record_recovery("HOLD_RECAPTURE_REQUESTED", measured_arrival=arrivals[0],
                        actual_rad=list(sample["actual_rad"]),
                        error_deg=[math.degrees(b-a) for a,b in zip(sample["actual_rad"], target)])
                    self.window._tick()
                    epoch = self.window.activation_epoch
                    self.window._hold_current()
                    if (self.window.hardware_mode != "hold" or self.window.command_stream_suspended
                            or self.window.activation_epoch <= epoch
                            or tuple(self.window.command_targets) != tuple(self.window.actual)):
                        raise RuntimeError("GUI did not capture the recovered current-pose HOLD")
                    self.origin = tuple(self.window.command_targets)
                    self.stage = "recovery_hold"
                    return
                if self.stage == "return_center":
                    if len(arrivals) != 1:
                        raise RuntimeError("one measured center-return endpoint is required")
                    center_model = [math.degrees(a) + b for a, b in zip(self.origin, self.window.session_pose_deg)]
                    errors = [a-b for a,b in zip(arrivals[0]["actual_model_deg"], center_model)]
                    if max(abs(error) for error in errors) > 0.25:
                        raise RuntimeError("measured center return failed demo bounds")
                    self._record_center("PASS", measured_arrival=arrivals[0], actual_rad=list(sample["actual_rad"]),
                                        error_deg=errors)
                    self.success = True
                    self.stop()
                    return
                if len(arrivals) != 2:
                    raise RuntimeError("two measured endpoints are required")
                initial = math.degrees(self.origin[0]) + self.window.session_pose_deg[0]
                displacement = arrivals[0]["actual_model_deg"][0] - initial
                returned = arrivals[1]["actual_model_deg"][0] - initial
                out_error = displacement - self.excursion_deg
                return_error = returned + self.excursion_deg if self.symmetric else returned
                if abs(out_error) > 0.25 or abs(return_error) > 0.25:
                    raise RuntimeError("measured J1 out/back displacement failed demo bounds")
                self.events.append({"event": "measured_j1_out_and_back", "cycle": self.cycle_number,
                                    "out_deg": displacement, "return_deg": returned,
                                    "out_error_deg": out_error, "return_error_deg": return_error})
                self._record_cycle("PASS", out_deg=displacement, return_deg=returned,
                                   out_error_deg=out_error, return_error_deg=return_error)
                if self.cycle_number == self.requested_cycles:
                    if self.return_center:
                        self.stage, self.dialog = "return_center", None
                        self.window._tick()
                        self.dialog = self.start_center(self.origin)
                    else:
                        self.success = True
                        self.stop()
                else:
                    self.cycle_number += 1
                    self.dialog = None
                    self.window._tick()
                    self.dialog = self.start_group(self.origin)
        except Exception as error:
            self.stop(str(error))

    def result(self):
        terminal_ok = self.terminal is not None and self.terminal.terminal_confirmed
        failure = self.failure or (self.terminal.failure if self.terminal else "terminal not observed")
        if self.interactive_teach:
            return {"schema": "go-m8010-interactive-teach-session/1.0",
                    "status": "PASS" if self.interactive_ready and terminal_ok and not failure else "FAIL",
                    "scope": "BOOTSTRAP_AND_TERMINAL; manual teaching evidence is recorded separately",
                    "failure": failure, "interactive_ready": self.interactive_ready,
                    "elapsed_s": self.now() - self.started, "maximum_seconds": self.maximum_seconds,
                    "terminal_brake_and_j6_disabled_confirmed": terminal_ok,
                    "initial_hold_target_rad": self.initial_hold_target,
                    "events": self.events, "samples": self.samples,
                    "teach_events": list(getattr(self.window, "teach_events", [])),
                    "terminal_samples": self.terminal.samples if self.terminal else []}
        completed = sum(result["status"] == "PASS" for result in self.cycle_results)
        recovery_ok = not self.recover_initial_first or (self.recovery_result or {}).get("status") == "PASS"
        center_ok = not self.return_center or (self.center_return or {}).get("status") == "PASS"
        return {"schema": "go-m8010-j1-action-group-demo/1.0",
                "status": "PASS" if self.success and completed == self.requested_cycles and terminal_ok and recovery_ok and center_ok and not failure else "FAIL",
                "scope": "J1_SYMMETRIC_ENDPOINTS" if self.symmetric else "J1_OUT_AND_BACK",
                "strict_precision_qualification": "NOT_RUN_OR_MODIFIED", "failure": failure,
                "terminal_brake_and_j6_disabled_confirmed": terminal_ok,
                "elapsed_s": self.now() - self.started, "initial_hold_target_rad": self.initial_hold_target,
                "action_group_origin_rad": self.origin, "recovery_result": self.recovery_result,
                "recover_initial_first": self.recover_initial_first,
                "excursion_deg": self.excursion_deg, "symmetric": self.symmetric, "speed_deg_s": self.speed_deg_s,
                "return_center": self.return_center, "center_return": self.center_return,
                "action_group_file": self.action_group_file,
                "requested_cycles": self.requested_cycles, "completed_cycles": completed,
                "maximum_seconds": self.maximum_seconds, "cycle_results": self.cycle_results,
                "events": self.events, "samples": self.samples,
                "terminal_samples": self.terminal.samples if self.terminal else [],
                "action_group_events": self.dialog.runner.events if self.dialog and self.dialog.runner else []}


def run_live(ros_args, binding, cycles=1, recover_initial_first=False, *,
             excursion_deg=1.0, symmetric=False, speed_deg_s=1.0, return_center=False, interactive_teach=False):
    root = Path(__file__).resolve().parents[2]
    sys.path.insert(0, str(root / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环/ros2_ws/src/go_m8010_arm_gui"))
    from go_m8010_arm_gui import main_window as gui
    from go_m8010_arm_gui.action_group_dialog import ActionGroupDialog
    from go_m8010_arm_gui.action_groups import ActionGroup, ActionStep
    from v15_31b_acceptance_signal import CONFIRMATION_TOPIC, SignalPayloadSequence
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters

    gui.rclpy.init(args=ros_args)
    app = gui.QApplication([sys.argv[0]])
    app.setQuitOnLastWindowClosed(False)
    node = gui.ArmGuiNode()
    demo = None

    class Window(gui.MainWindow):
        def closeEvent(self, event):
            if demo is not None and not demo.done:
                demo.stop(None if interactive_teach else "operator closed demo window")
                event.ignore()
            else:
                super().closeEvent(event)

    class DemoDialog(ActionGroupDialog):
        def _move_status(self):
            if not self.submitted and self.window._preview_approval_matches_candidate():
                recipe = self.window.workflow_contract.q_plan_trajectory
                recovering = demo.stage == "recover_initial"
                segments = check_recipe(recipe, demo.origin, recover_initial=recovering)
                center = demo.stage == "return_center"
                demo.events.append({"event": "checked_recovery_recipe_before_submit" if recovering else "checked_center_recipe_before_submit" if center else "checked_recipe_before_submit",
                                    "cycle": None if center else 0 if recovering else demo.cycle_number, "step": self.runner.index,
                                    "plan_token_id": self.window.workflow_contract.current_plan_token.token_id,
                                    "start_error_from_initial_deg": [math.degrees(a-b) for a,b in zip(recipe.start_rad, demo.origin)],
                                    "segments": segments})
            return super()._move_status()

    window, raw = Window(node), deque(maxlen=64)
    window.action_group_dialog = DemoDialog(window)
    def receive_raw(message):
        try:
            value = json.loads(message.data)
            if (value.get("schema") == "go-m8010-motor-feedback/1.0" and len(value.get("samples", [])) == 1
                    and value["samples"][0].get("motor") == "J6"):
                raw.append(value)
        except (ValueError, TypeError, AttributeError):
            pass
    subscription = node.create_subscription(gui.String, "/whole_arm/motor_feedback_raw", receive_raw, 100)
    publisher = node.create_publisher(gui.String, CONFIRMATION_TOPIC, 10)
    sequence = SignalPayloadSequence(binding)
    client = node.create_client(SetParameters, "/whole_arm_gravity_node/set_parameters")

    def observe():
        sample = observe_gui(window, gui, raw, binding)
        if interactive_teach:
            sample.update(command_targets_rad=list(window.command_targets),
                          teach_joint=getattr(window, "teach_joint", None))
        gravity = node.latest_gravity_status or {}
        empirical = gravity.get("empirical_validation", {})
        sample.update({"authority": bool(node.gravity_status_fresh() and empirical_binding_matches(gravity, binding)
            and gui.gravity_status_authorizes_hardware(gravity, session_id=binding.session_id,
                state_instance_id=binding.state_instance_id, now_monotonic_ns=time.monotonic_ns())),
            "thermal_ready": window._current_preview_checks(time.monotonic()).thermal_pass,
            "stage_index": empirical.get("stage_index"), "stage_level": empirical.get("stage_level"),
            "stage_complete": empirical.get("stage_complete") is True,
            "position_authorized": empirical.get("position_validation_authorized") is True,
            "assisted_teach_authorized": empirical.get("assisted_teach_authorized") is True})
        acceptance = node.latest_acceptance_status or {}
        if (gui.receipt_is_fresh(node.last_acceptance_status_receipt, time.monotonic(), 1.0)
                and acceptance.get("result") in {"RUNNING", "FINALIZING"}):
            raise RuntimeError("independent acceptance runner is active")
        return sample

    def confirm(level):
        binding.ensure_not_expired()
        hardware = node.latest_hardware
        if (not gui.hardware_state_contract_valid(hardware)
                or not gui.hardware_state_source_is_fresh(hardware, time.monotonic_ns())):
            return False
        if (hardware["session_id"], hardware["state_instance_id"]) != (binding.session_id, binding.state_instance_id):
            raise RuntimeError("empirical envelope does not match the current hardware session")
        if not node.gravity_status_fresh():
            return False
        if not empirical_binding_matches(node.latest_gravity_status or {}, binding):
            raise RuntimeError("live gravity envelope/anchor differs from the supplied binding")
        publisher.publish(gui.String(data=json.dumps(sequence.confirmation(level))))
        return True

    def set_scale(level):
        if not client.service_is_ready():
            raise RuntimeError("gravity parameter service unavailable")
        return client.call_async(SetParameters.Request(parameters=[Parameter(name="gravity_scale_target",
            value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=level))]))

    def start_group(origin):
        dialog = window.action_group_dialog
        path = node.log_directory / "j1_demo_action_group.json"
        if demo.action_group_file is None:
            baseline = [math.degrees(a) + b for a, b in zip(origin, window.session_pose_deg)]
            target = baseline[:]
            target[0] += demo.excursion_deg
            if demo.symmetric:
                baseline[0] -= demo.excursion_deg
            group = ActionGroup(f"J1 {'±' if demo.symmetric else ''}{demo.excursion_deg:g}°往返动作组示例", (
                ActionStep(tuple(target), speed_deg_s=demo.speed_deg_s, dwell_s=0.5),
                ActionStep(tuple(baseline), speed_deg_s=demo.speed_deg_s, dwell_s=0.5),
            ), window.workflow_contract.model_sha256)
            if path.exists():
                raise RuntimeError("action-group artifact already exists; use a fresh run directory")
            group.save(path)
            demo.action_group_file = {"path": str(path.resolve()), "sha256": hashlib.sha256(path.read_bytes()).hexdigest()}
        if str(path.resolve()) != demo.action_group_file["path"] or hashlib.sha256(path.read_bytes()).hexdigest() != demo.action_group_file["sha256"]:
            raise RuntimeError("saved action-group file changed between cycles")
        loaded = ActionGroup.load(path, expected_model_sha256=window.workflow_contract.model_sha256,
                                  joint_limits_deg=window.absolute_limits)
        if hashlib.sha256(path.read_bytes()).hexdigest() != demo.action_group_file["sha256"]:
            raise RuntimeError("saved action-group file changed while loading")
        dialog.table.setRowCount(0)
        dialog.name.setText(loaded.name)
        for step in loaded.steps:
            dialog.append_step(step)
        dialog.start()
        return dialog

    def start_center(origin):
        group = ActionGroup("J1 回固定中点", (ActionStep(tuple(
            math.degrees(a) + b for a,b in zip(origin, window.session_pose_deg)),
            speed_deg_s=demo.speed_deg_s, dwell_s=0.5),), window.workflow_contract.model_sha256)
        dialog = window.action_group_dialog
        dialog.table.setRowCount(0)
        dialog.name.setText(group.name)
        for step in group.steps:
            dialog.append_step(step)
        dialog.start()
        return dialog

    def read_initial_reference(sample=None):
        path = node.initial_pose_path
        data = path.read_bytes()
        initial = json.loads(data)
        hardware = node.latest_hardware
        if not gui.initial_pose_binding_valid(initial, data, hardware):
            raise RuntimeError("original initial-pose file does not match the current hardware binding")
        reference = [initial["关节位置_弧度"][f"J{i}"] for i in range(1, 7)]
        if any(type(value) not in {int, float} or not math.isfinite(value) for value in reference):
            raise RuntimeError("original six-axis reference is not finite")
        if sample is not None:
            if (sample["source_monotonic_ns"] != hardware.get("source_monotonic_ns")
                    or sample["actual_rad"] != hardware.get("position_rad")):
                raise RuntimeError("recovery pre-HOLD snapshot changed")
            for index, (actual, original) in enumerate(zip(sample["actual_rad"], reference)):
                if abs(actual - original) > math.radians(5):
                    raise RuntimeError(f"recovery pre-HOLD J{index + 1} is more than 5 degrees from the original reference")
        return reference, {"path": str(path.resolve()), "sha256": hashlib.sha256(data).hexdigest()}

    def start_recovery(origin):
        target, source = read_initial_reference()
        if any(abs(a-b) > math.radians(5) for a,b in zip(target, origin)):
            raise RuntimeError("initial-pose recovery target is more than 5 degrees away")
        demo.recovery_target = tuple(target)
        demo.recovery_result.update({"initial_pose_file": source, "target_rad": target,
                                     "original_reference_rad": list(target)})
        group = ActionGroup("回原初始化姿态（不改校准）", (ActionStep(tuple(
            math.degrees(a) + b for a,b in zip(target, window.session_pose_deg)), speed_deg_s=1.0, dwell_s=0.5),),
            window.workflow_contract.model_sha256)
        dialog = window.action_group_dialog
        dialog.table.setRowCount(0)
        dialog.name.setText(group.name)
        for step in group.steps:
            dialog.append_step(step)
        dialog.start()
        return dialog

    demo = J1Demo(window, observe, confirm, set_scale, start_group, cycles=cycles,
                  excursion_deg=excursion_deg, symmetric=symmetric, speed_deg_s=speed_deg_s,
                  return_center=return_center, start_center=start_center,
                  recover_initial_first=recover_initial_first, start_recovery=start_recovery,
                  validate_recovery_start=read_initial_reference, interactive_teach=interactive_teach)
    timer = gui.QTimer(window)
    def tick():
        try:
            if gui.rclpy.ok(context=node.context):
                for _ in range(4):
                    gui.pump_ros_callbacks(node, gui.rclpy.spin_once)
        except Exception as error:
            demo.stop(f"ROS callback failure: {error}")
        demo.tick()
        if demo.done:
            timer.stop()
            app.quit()
    timer.timeout.connect(tick)
    timer.start(20)
    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, lambda *_: demo.stop("operator/process stop requested"))
    window.centralWidget().setEnabled(False)
    stop = window.addToolBar("验证停止").addAction("停止往返示例并制动（Esc）")
    stop.setShortcut("Esc")
    stop.triggered.connect(lambda *_: demo.stop(None if interactive_teach else "operator pressed stop"))
    window.setWindowTitle(f"J1 {'±' if symmetric else ''}{excursion_deg:g}°往返动作组示例 × {cycles}轮；逐轴小修正会记录；结束自动制动")
    if interactive_teach:
        window.setWindowTitle("选轴辅助示教：J1–J5 单轴可拖动，其余保持；松键锁定；10分钟自动结束")
        stop.setText("结束示教并制动（Esc）")
    window.show()
    try:
        app.exec()
    except BaseException as error:
        demo.stop(f"GUI event loop exited: {error}")
    finally:
        if not demo.done:
            demo.stop("event loop ended before terminal verification")
            while not demo.done:
                try:
                    window._tick()
                except Exception:
                    pass
                demo.tick()
                time.sleep(0.01)
        result = demo.result()
        result["binding"] = {"session_id": binding.session_id, "state_instance_id": binding.state_instance_id,
            "envelope_sha256": binding.envelope_sha256, "anchor_sha256": binding.anchor_sha256}
        result["gui_control_config"] = window.config["控制"]
        result["action_group_log"] = str(window.action_group_dialog.log_path) if window.action_group_dialog else None
        window.timer.stop()
        window.close()
        node.destroy_subscription(subscription)
        node.destroy_node()
        if gui.rclpy.ok():
            gui.rclpy.shutdown()
    return result


def main(argv=None):
    values = list(sys.argv[1:] if argv is None else argv)
    split = values.index("--ros-args") if "--ros-args" in values else len(values)
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--cycles", type=int, choices=range(1, 11), default=1)
    parser.add_argument("--excursion-deg", type=float, default=1.0)
    parser.add_argument("--symmetric", action="store_true")
    parser.add_argument("--speed-deg-s", type=float, default=1.0)
    parser.add_argument("--return-center", action="store_true")
    parser.add_argument("--recover-initial-first", action="store_true")
    parser.add_argument("--interactive-teach", action="store_true")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--envelope", type=Path)
    parser.add_argument("--anchor-validation", type=Path)
    parser.add_argument("--expected-envelope-sha256")
    args = parser.parse_args(values[:split])
    try:
        maximum_seconds = 600.0 if args.interactive_teach else maximum_demo_seconds(args.cycles, args.excursion_deg, args.symmetric)
        if not math.isfinite(args.speed_deg_s) or not 0 < args.speed_deg_s <= 3:
            raise ValueError("speed_deg_s must be finite and greater than 0 through 3")
    except ValueError as error:
        parser.error(str(error))
    if not args.execute:
        print(json.dumps({"mode": "OFFLINE_DESCRIPTION_ONLY", "hardware_accessed": False,
                          "cycles": args.cycles, "recover_initial_first": args.recover_initial_first,
                          "excursion_deg": args.excursion_deg, "symmetric": args.symmetric,
                          "speed_deg_s": args.speed_deg_s, "return_center": args.return_center,
                          "maximum_seconds": maximum_seconds, "gravity_levels": LEVELS,
                          "demo": "J1 fixed-origin endpoints; secondary corrections <=0.25 degrees"}))
        return 0
    if (not all((args.output, args.envelope, args.anchor_validation, args.expected_envelope_sha256))
            or args.output.exists() or split == len(values)):
        parser.error("--execute requires new --output, bound envelope/anchor/SHA and existing GUI --ros-args")
    from v15_31b_active_acceptance_runner import EvidenceBinding
    binding = EvidenceBinding.from_paths(args.envelope, args.anchor_validation, args.expected_envelope_sha256)
    binding.ensure_not_expired()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("x+", encoding="utf-8") as stream:
        stream.write('{"status":"INCOMPLETE"}\n')
        stream.flush()
        result = run_live(values[split:], binding, args.cycles, args.recover_initial_first,
                          excursion_deg=args.excursion_deg, symmetric=args.symmetric,
                          speed_deg_s=args.speed_deg_s, return_center=args.return_center, interactive_teach=args.interactive_teach)
        stream.seek(0)
        json.dump(result, stream, ensure_ascii=False, allow_nan=False, indent=2)
        stream.write("\n")
        stream.truncate()
    print(json.dumps({"status": result["status"], "failure": result["failure"], "report": str(args.output.resolve())}, ensure_ascii=False))
    return 0 if result["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())

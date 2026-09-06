#!/usr/bin/env python3
"""Fast-forward an action group with synthetic feedback; never access hardware.

Run: python tools/demo_action_group.py --check-controls --output result.json
The example demonstrates sequencing only, not a physical pick/place prescription.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(
    ROOT / "V15_14_MoveIt2_ROS2_Control_MuJoCo_轨迹闭环" / "ros2_ws" / "src"
    / "go_m8010_arm_gui"
))

from go_m8010_arm_gui.action_groups import (  # noqa: E402
    ActionGroup, ActionGroupRunner, PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG,
)
from go_m8010_arm_gui.workflow_contract import (  # noqa: E402
    generate_segmented_quintic_recipe,
)


MODEL_SHA256 = "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"
INITIAL_DEG = (0.0, 90.0, -14.40, 13.49, 47.94, 0.0)
LIMITS_RAD = tuple(tuple(map(math.radians, pair))
                   for pair in PRODUCTION_ABSOLUTE_JOINT_LIMITS_DEG)


class SyntheticBackend:
    """Exact recipe samples substituted for encoders, with a virtual clock."""

    def __init__(self):
        self.time_s = 0.0
        self.q_rad = tuple(map(math.radians, INITIAL_DEG))
        self.recipe = None
        self.started_s = 0.0
        self.sample_times = []
        self.trajectories = []
        self.gripper_commands = []

    def move_start(self, step):
        target_rad = tuple(map(math.radians, step.target_deg))
        self.started_s = self.time_s
        self.recipe = None
        if target_rad == self.q_rad:
            return
        self.recipe = generate_segmented_quintic_recipe(
            self.q_rad, target_rad, LIMITS_RAD,
            maximum_velocity_rad_s=math.radians(step.speed_deg_s),
            maximum_acceleration_rad_s2=math.radians(15.0),
            maximum_segment_delta_rad=math.radians(30.0),
        )
        self.sample_times = [sample.time_s for sample in self.recipe.samples]
        self.trajectories.append({
            "at_simulation_s": self.time_s,
            "recipe_sha256": self.recipe.sha256,
            "start_deg": list(map(math.degrees, self.q_rad)),
            "target_deg": list(step.target_deg),
            "duration_s": self.recipe.profile.duration_s,
            "sample_count": len(self.recipe.samples),
            "moving_joints_by_segment": [
                [f"J{i + 1}" for i, (start, end) in enumerate(zip(
                    segment.start_rad, segment.target_rad,
                )) if start != end]
                for segment in self.recipe.segments
            ],
        })

    def advance(self, seconds=0.01):
        self.time_s = round(self.time_s + seconds, 9)
        if self.recipe is not None:
            index = bisect_right(self.sample_times, self.time_s - self.started_s) - 1
            self.q_rad = self.recipe.samples[max(0, index)].q_rad

    def move_status(self):
        if self.recipe is None or self.time_s - self.started_s >= self.recipe.profile.duration_s:
            return "complete"
        return "running"

    def stop_motion(self):
        self.recipe = None

    def gripper_command(self, step):
        self.gripper_commands.append({
            "at_simulation_s": self.time_s, "command": step.gripper,
            "opening_percent": step.opening_percent,
            "wait_s": step.gripper_wait_s,
            "transport": "SYNTHETIC_NO_DEVICE", "physical_grasp_verified": False,
        })
        return "SOFTWARE_SIMULATION_ONLY: synthetic receipt; no physical gripper controller"


def simulate(group: ActionGroup, *, control_check: str | None = None) -> dict:
    backend = SyntheticBackend()
    runner = ActionGroupRunner(
        group, backend.move_start, backend.move_status, backend.stop_motion,
        backend.gripper_command, lambda: None, now=lambda: backend.time_s,
    )
    runner.start()
    checked = False
    # Virtual time is bounded by the runner's per-step timeout and wait limits.
    limit_s = len(group.steps) * (runner.motion_timeout_s + 630.0) + 3.0
    while runner.active and backend.time_s <= limit_s:
        backend.advance()
        runner.tick()
        if (control_check and not checked and runner.state == "moving"
                and backend.recipe is not None and backend.time_s - backend.started_s >= 0.1):
            if control_check == "pause_resume":
                runner.pause()
                assert runner.state == "paused"
                position, commands = backend.q_rad, len(backend.gripper_commands)
                backend.advance(2.0)
                runner.tick()
                assert runner.state == "paused" and backend.q_rad == position
                assert len(backend.gripper_commands) == commands
                runner.resume()
                assert runner.state == "moving"
            else:
                runner.stop()
                position, commands, events = backend.q_rad, len(backend.gripper_commands), len(runner.events)
                backend.advance(2.0)
                runner.tick()
                assert runner.state == "stopped" and backend.q_rad == position
                assert len(backend.gripper_commands) == commands and len(runner.events) == events
            checked = True
    if runner.active:
        raise RuntimeError("software simulation exceeded its bounded virtual time")
    expected_state = "stopped" if control_check == "stop" else "complete"
    return {
        "result": runner.result, "simulation_duration_s": backend.time_s,
        "final_synthetic_deg": list(map(math.degrees, backend.q_rad)),
        "events": runner.events, "trajectories": backend.trajectories,
        "gripper_commands": backend.gripper_commands,
        "control_check": control_check,
        "control_check_result": (
            "PASS" if checked and runner.state == expected_state
            else "FAIL" if checked else "NOT_EXERCISED"
        ) if control_check else None,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action_group", nargs="?", type=Path,
                        default=ROOT / "examples" / "action_groups" / "pick_place_demo.json")
    parser.add_argument("--output", type=Path, help="write the software-only JSON report")
    parser.add_argument("--check-controls", action="store_true",
                        help="also exercise pause/resume and stop on synthetic motion")
    args = parser.parse_args()
    if args.output and args.output.resolve() == args.action_group.resolve():
        parser.error("output must not overwrite the input action group")
    try:
        group = ActionGroup.load(args.action_group, expected_model_sha256=MODEL_SHA256)
        report = {
            "schema": "go-m8010-action-group-software-demo/1.0",
            "validation_scope": "SOFTWARE_SIMULATION_ONLY",
            "feedback_source": "SYNTHETIC_QUINTIC_SAMPLES",
            "collision_proof": "NOT_PERFORMED", "physical_grasp_verified": False,
            "hardware_accessed": False,
            "physical_gripper": "PWM servo; controller not installed; ESP32-P4 planned",
            "notice": "仅验证软件步序和等待；没有真实反馈、碰撞证明或实机抓放结果。",
            "action_group": str(args.action_group.resolve()),
            "name": group.name, "model_sha256": group.model_sha256,
            "initial_synthetic_deg": list(INITIAL_DEG),
            "run": simulate(group),
        }
        if args.check_controls:
            report["control_checks"] = {
                check: simulate(group, control_check=check)
                for check in ("pause_resume", "stop")
            }
        encoded = json.dumps(report, ensure_ascii=False, allow_nan=False, indent=2) + "\n"
        if args.output:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            args.output.write_text(encoded, encoding="utf-8")
            print(json.dumps({
                "validation_scope": report["validation_scope"],
                "result": report["run"]["result"],
                "control_checks": {
                    name: check["control_check_result"]
                    for name, check in report.get("control_checks", {}).items()
                },
                "report": str(args.output.resolve()),
            }, ensure_ascii=False))
        else:
            print(encoded, end="")
        checks = report.get("control_checks", {}).values()
        return 0 if (report["run"]["result"]["state"] == "complete"
                     and all(check["control_check_result"] == "PASS" for check in checks)) else 1
    except (OSError, ValueError, RuntimeError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())

#!/usr/bin/env python3
"""One-shot, fail-closed J2 recovery with whole-arm gravity hold."""

from __future__ import annotations

import argparse
import json
import math
import socket
import time

import mujoco
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


GEAR = 6.329999923706055
PORTS = (15310, 15312, 15313, 15311)
KP = [0.50, 1.00, 0.60, 0.50, 0.50, 0.0]
KD = [0.05, 0.10, 0.05, 0.05, 0.05, 0.0]
OFFSET = np.radians([0.0, 90.0, 0.0, 0.0, 0.0, 0.0])


class StateReader(Node):
    def __init__(self) -> None:
        super().__init__("v15_30a_j2_auto_recovery")
        self.state = None
        self.received_at = 0.0
        self.create_subscription(String, "/whole_arm/hardware_state", self._on_state, 20)

    def _on_state(self, message: String) -> None:
        self.state = json.loads(message.data)
        self.received_at = time.monotonic()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--confirm", required=True)
    parser.add_argument("--validation-step-deg", type=float, default=0.5)
    parser.add_argument("--run-speed-deg-s", type=float, default=2.5)
    parser.add_argument("--motion-boost-nm", type=float, default=0.08)
    return parser.parse_args()


def gravity_feedforward(model, data, qpos_adr, dof_adr, position):
    data.qpos[qpos_adr] = OFFSET + np.asarray(position, dtype=float)
    data.qvel[:] = 0.0
    mujoco.mj_forward(model, data)
    tau = np.asarray(data.qfrc_bias)[dof_adr]
    # J2 must overcome the modeled load immediately.  The other geared joints
    # are already stationary under substantial static friction; applying their
    # full theoretical gravity torque caused a measurable over-compensation.
    # Keep them at their captured position with PD. J2 receives its full
    # posture-dependent common feedforward. Empirical bracketing showed that
    # zero J3 feedforward sags and 50% over-compensates by nearly the same
    # amount. The initial 25% estimate still sagged slowly during sustained J2
    # motion, so use the measured 30% balance correction.
    # MotorCmd values are rotor-side in the vendor SDK.  Split the logical J2
    # torque across the coupled pair, then convert output torque to rotor torque.
    result = [
        0.0,
        float(tau[1] / (2.0 * GEAR)),
        0.30 * float(tau[2] / GEAR),
        0.0,
        0.0,
        0.0,
    ]
    limits = [0.20, 1.75, 1.00, 0.40, 0.20, 0.0]
    if any(abs(value) > limit + 1e-9 for value, limit in zip(result, limits)):
        raise RuntimeError(f"GRAVITY_FEEDFORWARD_LIMIT:{result}")
    return result


def main() -> int:
    args = parse_args()
    if args.confirm != "J2_AUTO_RECOVERY_WITH_GRAVITY_HOLD=YES":
        raise SystemExit("confirmation gate missing")
    rclpy.init()
    node = StateReader()
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    model = mujoco.MjModel.from_xml_path(args.model)
    model.opt.gravity[:] = [0.0, 0.0, -9.81]
    data = mujoco.MjData(model)
    qpos_adr, dof_adr = [], []
    for name in ("J1", "J2", "J3", "J4", "J5", "J6"):
        jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)
        qpos_adr.append(int(model.jnt_qposadr[jid]))
        dof_adr.append(int(model.jnt_dofadr[jid]))
    qpos_adr = np.asarray(qpos_adr, dtype=int)
    dof_adr = np.asarray(dof_adr, dtype=int)
    epoch = max(1, time.monotonic_ns() & ((1 << 63) - 1))
    sequence = 0

    def send(mode, targets, feedforward, vmax_deg_s, amax_deg_s2, active=True):
        nonlocal sequence
        sequence += 1
        payload = json.dumps({
            "schema": "go-m8010-gui-command/1.1",
            "sequence": sequence,
            "mode": mode,
            "targets_rad": list(targets),
            "active_joint_mask": (
                [False, True, True, True, True, False]
                if active else [False] * 6
            ),
            "activation_epoch": epoch,
            "maximum_velocity_rad_s": math.radians(vmax_deg_s),
            "maximum_acceleration_rad_s2": math.radians(amax_deg_s2),
            "kp": KP,
            "kd": KD,
            "feedforward_nm": list(feedforward),
            "recovery": bool(active),
        }, separators=(",", ":")).encode()
        for port in PORTS:
            sock.sendto(payload, ("127.0.0.1", port))

    def brake():
        for _ in range(30):
            send("brake", [0.0] * 6, [0.0] * 6, 0.5, 2.0, active=False)
            time.sleep(0.01)

    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.state and node.state.get("healthy") and all(
                value == "brake" for value in node.state["controller_mode_by_motor"].values()
            ):
                break
        else:
            raise RuntimeError("HEALTHY_BRAKE_BASELINE_MISSING")

        initial = np.asarray(node.state["position_rad"], dtype=float)
        if not (-math.radians(260.0) <= initial[1] < -math.radians(0.5)):
            raise RuntimeError(f"J2_RECOVERY_DIRECTION_INVALID:{math.degrees(initial[1])}")
        targets = initial.copy()
        validation_target = min(0.0, initial[1] + math.radians(args.validation_step_deg))
        stage = "gravity_ramp"
        stage_started = time.monotonic()
        stable_since = None
        last_print = 0.0
        reached_validation = False

        while True:
            rclpy.spin_once(node, timeout_sec=0.002)
            now = time.monotonic()
            if node.state is None or now - node.received_at > 0.20:
                raise RuntimeError("STATE_STALE")
            state = node.state
            actual = np.asarray(state["position_rad"], dtype=float)
            sync = abs(float(state["j2_e_sync_rad"]))
            faults = state.get("controller_fault_by_motor", {})
            if any(bool(value) for value in faults.values()):
                raise RuntimeError(f"CONTROLLER_FAULT:{faults}")
            if sync >= math.radians(0.5):
                raise RuntimeError(f"J2_SYNC_LIMIT:{math.degrees(sync):.3f}")
            drift = np.abs(np.degrees(actual - initial))
            if any(drift[index] > 0.75 for index in (0, 2, 3, 4, 5)):
                raise RuntimeError(f"NON_J2_DRIFT:{drift.tolist()}")
            feedforward = gravity_feedforward(model, data, qpos_adr, dof_adr, actual)
            base_j2_feedforward = feedforward[1]

            # The model feedforward balances gravity but deliberately contains
            # no friction term.  Add a small bounded common-mode boost only
            # while J2 is commanded upward, then taper it out before arrival so
            # the zero hold is not biased.  Both J2 motors receive the same
            # logical contribution through their opposite motor signs.
            motion_target = validation_target if stage == "validate" else 0.0
            motion_error_deg = math.degrees(motion_target - actual[1])
            if stage in ("validate", "run") and motion_error_deg > 0.10:
                boost_scale = min(1.0, (motion_error_deg - 0.10) / 0.20)
                boost_limit = (
                    args.motion_boost_nm if stage == "validate"
                    else min(args.motion_boost_nm, 0.60)
                )
                feedforward[1] = min(
                    1.15, feedforward[1] + boost_limit * boost_scale
                )

            if stage == "gravity_ramp":
                scale = min(1.0, (now - stage_started) / 2.0)
                send("hold", initial, np.asarray(feedforward) * scale, 0.5, 2.0)
                if scale >= 1.0:
                    stage = "validate"
                    stage_started = now
                    targets[1] = validation_target
            elif stage == "validate":
                send("position", targets, feedforward, 0.5, 2.0)
                validated_motion = actual[1] - initial[1]
                required_motion = math.radians(
                    min(0.20, max(0.05, args.validation_step_deg * 0.40))
                )
                if validated_motion >= required_motion and sync <= math.radians(0.4):
                    stable_since = now if stable_since is None else stable_since
                    if now - stable_since >= 0.5:
                        reached_validation = True
                        stage = "run"
                        stage_started = now
                        targets[1] = 0.0
                else:
                    stable_since = None
                if now - stage_started > 8.0:
                    raise RuntimeError(
                        f"VALIDATION_TIMEOUT:q2={math.degrees(actual[1]):.3f}"
                    )
            elif stage == "run":
                send("position", targets, feedforward, args.run_speed_deg_s, 5.0)
                if actual[1] < initial[1] - math.radians(0.30):
                    raise RuntimeError("J2_WRONG_DIRECTION")
                if abs(actual[1]) <= math.radians(0.25):
                    stable_since = now if stable_since is None else stable_since
                    if now - stable_since >= 1.0:
                        stage = "hold_zero"
                else:
                    stable_since = None
            else:
                send("hold", targets, feedforward, 0.5, 2.0)

            if now - last_print >= 0.5:
                print(json.dumps({
                    "stage": stage,
                    "j2_deg": math.degrees(actual[1]),
                    "j2_sync_deg": math.degrees(sync),
                    "max_non_j2_drift_deg": max(drift[index] for index in (0, 2, 3, 4, 5)),
                    "j2_ff_each_nm": feedforward[1],
                    "j2_motion_boost_nm": max(0.0, feedforward[1] - base_j2_feedforward),
                    "j3_ff_nm": feedforward[2],
                    "validation_passed": reached_validation,
                }, ensure_ascii=False), flush=True)
                last_print = now
            time.sleep(0.005)
    except BaseException as exc:
        print(f"FAIL_CLOSED:{type(exc).__name__}:{exc}", flush=True)
        brake()
        return 2
    finally:
        sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

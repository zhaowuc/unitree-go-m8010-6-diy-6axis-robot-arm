#!/usr/bin/env python3
"""Independent J1-J5 current-pose hold with fault-domain isolation."""

from __future__ import annotations

import argparse
import json
import math
import signal
import socket
import time

import mujoco
import numpy as np
import rclpy
from std_msgs.msg import String

from v15_30a_j2_auto_recovery import StateReader, gravity_feedforward


RIGID_KP = [1.50, 3.00, 2.00, 2.00, 1.50, 0.0]
RIGID_KD = [0.15, 0.30, 0.15, 0.15, 0.12, 0.0]
INTEGRAL_KI = np.asarray([2.0, 3.0, 4.0, 3.0, 2.0, 0.0])
INTEGRAL_LIMIT = np.asarray([0.15, 0.50, 0.40, 0.25, 0.15, 0.0])
FEEDFORWARD_LIMIT = np.asarray([0.20, 1.75, 1.00, 0.40, 0.20, 0.0])
GUI_LOWER_RAD = np.radians([-180.0, -260.0, -170.0, -116.0, -70.6, -180.0])
GUI_UPPER_RAD = np.radians([180.0, 80.0, 170.0, 159.0, 151.2, 180.0])


DOMAINS = {
    "j1": {"port": 15310, "motors": ("J1",), "mask": (True, False, False, False, False, False)},
    "j2": {"port": 15312, "motors": ("J2A", "J2B"), "mask": (False, True, False, False, False, False)},
    "j345": {"port": 15313, "motors": ("J3", "J4", "J5"), "mask": (False, False, True, True, True, False)},
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--model", required=True)
    parser.add_argument("--confirm", required=True)
    parser.add_argument("--target-deg")
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    if args.confirm != "RIGID_CURRENT_HOLD_J1_J5=YES":
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

    stop_requested = False

    def request_stop(_signum, _frame) -> None:
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    epoch = max(1, time.monotonic_ns() & ((1 << 63) - 1))
    sequence = 0
    gui_request = None

    def on_gui_command(message: String) -> None:
        nonlocal gui_request
        try:
            value = json.loads(message.data)
            targets = np.asarray(value["targets_rad"], dtype=float)
            mask = value.get("active_joint_mask", [False] * 6)
            mode = str(value.get("mode", "brake"))
            if targets.shape != (6,) or not np.all(np.isfinite(targets)):
                return
            if np.any(targets < GUI_LOWER_RAD) or np.any(targets > GUI_UPPER_RAD):
                return
            if len(mask) != 6 or not all(type(item) is bool for item in mask):
                return
            gui_request = {
                "sequence": int(value.get("sequence", 0)),
                "mode": mode,
                "targets": targets,
                "mask": list(mask),
            }
        except Exception:
            return

    node.create_subscription(String, "/whole_arm/gui_command", on_gui_command, 20)

    def send(domain: str, mode: str, targets, feedforward, active: bool) -> None:
        nonlocal sequence
        sequence += 1
        mask = list(DOMAINS[domain]["mask"]) if active else [False] * 6
        brake = mode == "brake"
        wire_targets = np.zeros(6, dtype=float) if brake else np.asarray(targets, dtype=float)
        wire_kp = [0.0] * 6 if brake else RIGID_KP
        wire_kd = [0.0] * 6 if brake else RIGID_KD
        wire_feedforward = np.zeros(6, dtype=float) if brake else np.asarray(feedforward, dtype=float)
        payload = json.dumps({
            "schema": "go-m8010-gui-command/1.1",
            "sequence": sequence,
            "mode": mode,
            # BRAKE has no position authority.  Keeping stale measured targets
            # here previously made an inactive domain fail the worker's motion
            # envelope before it could reach the fail-closed BRAKE path.
            "targets_rad": wire_targets.tolist(),
            "active_joint_mask": mask,
            # Advance the epoch with every refresh.  If Linux scheduling ever
            # creates a >0.5 s lease gap, the motor controller fences the old
            # epoch; a constant epoch can then leave that domain in BRAKE
            # forever despite healthy feedback.  A monotonic refresh epoch
            # automatically reacquires the same position target.
            "activation_epoch": epoch + sequence,
            "maximum_velocity_rad_s": math.radians(5.0),
            "maximum_acceleration_rad_s2": math.radians(15.0),
            "kp": wire_kp,
            "kd": wire_kd,
            "feedforward_nm": wire_feedforward.tolist(),
            "recovery": active,
        }, separators=(",", ":")).encode()
        sock.sendto(payload, ("127.0.0.1", DOMAINS[domain]["port"]))

    def domain_is_healthy(state, domain_name: str) -> bool:
        domain = DOMAINS[domain_name]
        faults = state.get("controller_fault_by_motor", {})
        per_motor = state.get("per_motor", {})
        healthy = all(
            bool(per_motor.get(name, {}).get("fresh"))
            and int(per_motor.get(name, {}).get("merror", -1)) == 0
            and bool(per_motor.get(name, {}).get("reference_captured"))
            and not bool(faults.get(name, True))
            for name in domain["motors"]
        )
        if domain_name == "j2":
            healthy = healthy and abs(
                float(state.get("j2_e_sync_rad", math.inf))
            ) < math.radians(0.5)
        return healthy

    try:
        deadline = time.monotonic() + 8.0
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.state is None:
                continue
            # J1/J2 are the gravity-critical domains.  Do not withhold their
            # position leases merely because the independent J3-J5 serial bus
            # is offline; that previously made J2 fall back to BRAKE.
            if domain_is_healthy(node.state, "j1") and domain_is_healthy(
                node.state, "j2"
            ):
                break
        else:
            raise RuntimeError("J1_J2_HEALTHY_BASELINE_MISSING")

        captured = np.asarray(node.state["position_rad"], dtype=float)
        if args.target_deg:
            requested = [float(value) for value in args.target_deg.split(",")]
            if len(requested) != 6:
                raise RuntimeError("TARGET_DEG_SIZE_INVALID")
            target = np.radians(requested)
        else:
            target = captured.copy()
        last_actual = captured.copy()
        active_domains = {
            name: domain_is_healthy(node.state, name) for name in DOMAINS
        }
        health_changed_at = {name: time.monotonic() for name in DOMAINS}
        last_health = dict(active_domains)
        started = time.monotonic()
        previous_loop = started
        integral_nm = np.zeros(6, dtype=float)
        last_gui_sequence = -1
        # A freshly opened GUI publishes its idle BRAKE state immediately.
        # When an explicit recovery target was requested, that startup frame
        # must not overwrite the requested zero pose with the current pose.
        last_gui_mode = "brake" if args.target_deg else None
        last_print = 0.0
        stale_reported = False
        print(json.dumps({
            "event": "RIGID_CURRENT_HOLD_CAPTURED",
            "target_deg": np.degrees(target).tolist(),
        }, ensure_ascii=False), flush=True)

        while not stop_requested:
            rclpy.spin_once(node, timeout_sec=0.001)
            now = time.monotonic()
            dt = min(0.05, max(0.0, now - previous_loop))
            previous_loop = now
            state_fresh = node.state is not None and now - node.received_at <= 0.25
            if state_fresh:
                stale_reported = False
                state = node.state
                last_actual = np.asarray(state["position_rad"], dtype=float)
                faults = state.get("controller_fault_by_motor", {})
                per_motor = state.get("per_motor", {})
                for domain_name, domain in DOMAINS.items():
                    healthy = domain_is_healthy(state, domain_name)
                    if healthy != last_health[domain_name]:
                        last_health[domain_name] = healthy
                        health_changed_at[domain_name] = now
                    stable_for = now - health_changed_at[domain_name]
                    # A brief missing aggregate frame must not remove torque.
                    # Persistent loss still isolates only the affected bus.
                    if not healthy and active_domains[domain_name] and stable_for >= 0.35:
                        active_domains[domain_name] = False
                        print(json.dumps({
                            "event": "DOMAIN_ISOLATED",
                            "domain": domain_name,
                            "faults": faults,
                        }, ensure_ascii=False), flush=True)
                    elif healthy and not active_domains[domain_name] and stable_for >= 0.20:
                        active_domains[domain_name] = True
                        for index, selected in enumerate(domain["mask"]):
                            if selected:
                                integral_nm[index] = 0.0
                        print(json.dumps({
                            "event": "DOMAIN_REACQUIRED",
                            "domain": domain_name,
                        }, ensure_ascii=False), flush=True)
                if gui_request is not None and gui_request["sequence"] != last_gui_sequence:
                    request = gui_request
                    last_gui_sequence = request["sequence"]
                    mode_changed = request["mode"] != last_gui_mode
                    last_gui_mode = request["mode"]
                    previous_target = target.copy()
                    if request["mode"] == "position":
                        for index in range(5):
                            if request["mask"][index]:
                                target[index] = request["targets"][index]
                    elif request["mode"] in {"brake", "hold"} and mode_changed:
                        target[:5] = last_actual[:5]
                    changed = np.abs(target - previous_target) > 1e-9
                    integral_nm[changed] = 0.0
                    if np.any(changed):
                        print(json.dumps({
                            "event": "GUI_TARGET_ACCEPTED",
                            "mode": request["mode"],
                            "target_deg": np.degrees(target).tolist(),
                        }, ensure_ascii=False), flush=True)
            elif not stale_reported:
                print("STATE_AGGREGATOR_STALE_KEEPING_LAST_HOLD", flush=True)
                stale_reported = True

            feedforward = gravity_feedforward(
                model, data, qpos_adr, dof_adr, last_actual
            )
            if state_fresh:
                error = target - last_actual
                outside_deadband = np.abs(error) > math.radians(0.03)
                integral_nm += INTEGRAL_KI * error * dt * outside_deadband
                integral_nm = np.clip(
                    integral_nm, -INTEGRAL_LIMIT, INTEGRAL_LIMIT
                )
            ramp = min(1.0, (now - started) / 0.25)
            feedforward = np.asarray(feedforward, dtype=float) * ramp + integral_nm
            feedforward = np.clip(
                feedforward, -FEEDFORWARD_LIMIT, FEEDFORWARD_LIMIT
            )
            for domain_name in DOMAINS:
                if active_domains[domain_name]:
                    send(domain_name, "position", target, feedforward, True)
                else:
                    send(domain_name, "brake", target, np.zeros(6), False)

            if now - last_print >= 0.5:
                print(json.dumps({
                    "event": "RIGID_CURRENT_HOLD_ACTIVE",
                    "position_deg": np.degrees(last_actual).tolist(),
                    "error_deg": np.degrees(last_actual - target).tolist(),
                    "j2_sync_deg": math.degrees(float(node.state["j2_e_sync_rad"])) if state_fresh else None,
                    "j2_ff_each_nm": float(feedforward[1]),
                    "j3_ff_nm": float(feedforward[2]),
                    "integral_nm": integral_nm.tolist(),
                    "active_domains": active_domains,
                }, ensure_ascii=False), flush=True)
                last_print = now
            time.sleep(0.006)

        print("RIGID_CURRENT_HOLD_STOP_REQUESTED", flush=True)
        return 0
    except Exception as exc:
        # Do not broadcast a whole-arm BRAKE here. Controllers retain their
        # independent leases and fault isolation; the supervisor is expected
        # to be restarted immediately if this process fails.
        print(f"RIGID_CURRENT_HOLD_ABORT:{type(exc).__name__}:{exc}", flush=True)
        return 2
    finally:
        sock.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())

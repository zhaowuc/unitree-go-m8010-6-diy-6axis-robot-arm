"""Direct-qpos MuJoCo mirror for session-relative measured JointState data."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import statistics
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .state_model import JOINT_NAMES


MUJOCO_JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def parse_pose_degrees(value: str) -> np.ndarray:
    fields = [field.strip() for field in value.split(",")]
    if len(fields) != 6:
        raise ValueError("session_pose_deg must contain exactly six comma-separated values")
    pose = np.radians(np.asarray([float(field) for field in fields], dtype=float))
    if not np.all(np.isfinite(pose)):
        raise ValueError("session pose contains non-finite values")
    return pose


class WholeArmMujocoMirror(Node):
    def __init__(self) -> None:
        super().__init__("whole_arm_mujoco_mirror")
        self.declare_parameter("model_path", "")
        self.declare_parameter("session_pose_deg", "0,0,0,0,0,0")
        self.declare_parameter("pose_matched", False)
        self.declare_parameter("use_viewer", True)
        self.declare_parameter("evidence_directory", "hardware/v15_30a_ft")

        if not bool(self.get_parameter("pose_matched").value):
            raise RuntimeError(
                "operator visual pose match is required: set pose_matched:=true and session_pose_deg"
            )
        model_path = Path(str(self.get_parameter("model_path").value)).expanduser().resolve()
        if not model_path.is_file():
            raise FileNotFoundError(f"frozen production MuJoCo model not found: {model_path}")
        self.session_pose = parse_pose_degrees(str(self.get_parameter("session_pose_deg").value))

        import mujoco

        self.mujoco = mujoco
        self.model_path = model_path
        self.model_hash = sha256_file(model_path)
        self.model = mujoco.MjModel.from_xml_path(str(model_path))
        self.data = mujoco.MjData(self.model)
        qpos_addresses = []
        qvel_addresses = []
        for name in MUJOCO_JOINT_NAMES:
            joint_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if joint_id < 0:
                raise RuntimeError(f"MuJoCo model is missing joint {name}")
            qpos_addresses.append(int(self.model.jnt_qposadr[joint_id]))
            qvel_addresses.append(int(self.model.jnt_dofadr[joint_id]))
        self.qpos_addresses = np.asarray(qpos_addresses, dtype=int)
        self.qvel_addresses = np.asarray(qvel_addresses, dtype=int)
        self.data.qpos[self.qpos_addresses] = self.session_pose
        self.data.qvel[self.qvel_addresses] = 0.0
        mujoco.mj_forward(self.model, self.data)

        evidence = Path(str(self.get_parameter("evidence_directory").value)).resolve()
        evidence.mkdir(parents=True, exist_ok=True)
        self.validation_path = evidence / "mirror_validation.json"
        capture_path = evidence / "mujoco_mirror_capture.csv"
        self.capture_stream = capture_path.open("w", encoding="utf-8", newline="")
        fieldnames = ["sequence", "ros_stamp_ns", "receipt_monotonic_ns", "latency_ms", "max_error_rad"]
        fieldnames += [f"{name}_target_qpos_rad" for name in JOINT_NAMES]
        fieldnames += [f"{name}_actual_qpos_rad" for name in JOINT_NAMES]
        self.capture_writer = csv.DictWriter(self.capture_stream, fieldnames=fieldnames)
        self.capture_writer.writeheader()

        self.subscription = self.create_subscription(JointState, "/joint_states", self.on_joint_state, 1)
        self.status_publisher = self.create_publisher(String, "/whole_arm/mujoco_mirror_status", 10)
        self.sequence = 0
        self.rejected_messages = 0
        self.latencies_ms: list[float] = []
        self.errors_rad: list[float] = []
        self.last_target: Optional[np.ndarray] = None
        self.get_logger().info(
            "MUJOCO_SESSION_POSE_MATCHED=YES; direct qpos mirror active; "
            f"model_sha256={self.model_hash}"
        )

    def on_joint_state(self, message: JointState) -> None:
        receipt_ns = time.monotonic_ns()
        try:
            if tuple(message.name) != JOINT_NAMES:
                raise ValueError(f"joint names/order must be exactly {JOINT_NAMES}")
            if len(message.position) != 6 or len(message.velocity) not in (0, 6):
                raise ValueError("JointState requires six positions and zero or six velocities")
            relative = np.asarray(message.position, dtype=float)
            velocity = np.zeros(6, dtype=float) if not message.velocity else np.asarray(message.velocity, dtype=float)
            if not np.all(np.isfinite(relative)) or not np.all(np.isfinite(velocity)):
                raise ValueError("JointState contains non-finite values")
            target = self.session_pose + relative
            self.data.qpos[self.qpos_addresses] = target
            self.data.qvel[self.qvel_addresses] = velocity
            self.mujoco.mj_forward(self.model, self.data)
            actual = self.data.qpos[self.qpos_addresses].copy()
            error = actual - target
            max_error = float(np.max(np.abs(error)))
            self.errors_rad.append(max_error)
            self.errors_rad = self.errors_rad[-5000:]
            self.last_target = target
            self.sequence += 1

            ros_stamp_ns = int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)
            now_ros_ns = self.get_clock().now().nanoseconds
            latency_ms = max(0.0, (now_ros_ns - ros_stamp_ns) / 1.0e6) if ros_stamp_ns else 0.0
            self.latencies_ms.append(latency_ms)
            self.latencies_ms = self.latencies_ms[-5000:]
            row = {
                "sequence": self.sequence,
                "ros_stamp_ns": ros_stamp_ns,
                "receipt_monotonic_ns": receipt_ns,
                "latency_ms": latency_ms,
                "max_error_rad": max_error,
            }
            for index, name in enumerate(JOINT_NAMES):
                row[f"{name}_target_qpos_rad"] = target[index]
                row[f"{name}_actual_qpos_rad"] = actual[index]
            self.capture_writer.writerow(row)
            if self.sequence % 50 == 0:
                self.capture_stream.flush()
                self.write_validation()
            self.status_publisher.publish(String(data=json.dumps(self.validation(), ensure_ascii=False)))
        except Exception as exc:
            self.rejected_messages += 1
            self.get_logger().warning(f"rejected /joint_states: {exc}")

    def validation(self) -> dict:
        return {
            "schema": "go-m8010-v15.30a-direct-qpos-mirror/1.0",
            "method": "DIRECT_QPOS_MIRROR",
            "model_path": str(self.model_path),
            "model_sha256": self.model_hash,
            "session_reference": "MIRROR_SESSION_REFERENCE_V1",
            "mujoco_session_pose_matched": "YES",
            "session_pose_rad": self.session_pose.tolist(),
            "joint_state_messages": self.sequence,
            "rejected_messages": self.rejected_messages,
            "median_latency_ms": statistics.median(self.latencies_ms) if self.latencies_ms else None,
            "p95_latency_ms": percentile(self.latencies_ms, 0.95),
            "maximum_mirror_error_rad": max(self.errors_rad) if self.errors_rad else None,
            "direct_qpos_numeric_result": (
                "PASS" if self.errors_rad and max(self.errors_rad) <= 1.0e-12 else "NOT_EVALUATED"
            ),
            "cad_zero": "PENDING",
            "ros_zero": "PENDING",
            "pid_used": False,
            "actuator_tracking_used": False,
            "trajectory_simulation_used": False,
        }

    def write_validation(self) -> None:
        self.validation_path.write_text(
            json.dumps(self.validation(), ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
        )

    def destroy_node(self) -> bool:
        self.write_validation()
        if not self.capture_stream.closed:
            self.capture_stream.flush()
            self.capture_stream.close()
        return super().destroy_node()


def percentile(values: list[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    return ordered[max(0, math.ceil(fraction * len(ordered)) - 1)]


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[WholeArmMujocoMirror] = None
    viewer = None
    try:
        node = WholeArmMujocoMirror()
        if bool(node.get_parameter("use_viewer").value):
            import mujoco.viewer

            viewer = mujoco.viewer.launch_passive(node.model, node.data)
            while rclpy.ok() and viewer.is_running():
                rclpy.spin_once(node, timeout_sec=0.01)
                viewer.sync()
        else:
            rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        if viewer is not None:
            viewer.close()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

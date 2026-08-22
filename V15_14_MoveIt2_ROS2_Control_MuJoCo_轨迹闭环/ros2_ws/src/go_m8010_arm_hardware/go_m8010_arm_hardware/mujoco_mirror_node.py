"""Direct-qpos MuJoCo mirror for session-relative measured JointState data."""

from __future__ import annotations

import csv
import hashlib
import json
import math
import signal
import statistics
import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String

from .state_model import JOINT_NAMES


MUJOCO_JOINT_NAMES = ("J1", "J2", "J3", "J4", "J5", "J6")
PRODUCTION_MODEL_SHA256 = "5ea615cff88d3594fa12812fc9e4c738fb84c7993159feaf45b364d86a58f9c9"


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
        self.declare_parameter("session_relative_baseline", False)
        self.declare_parameter("numeric_test_only", False)
        self.declare_parameter("use_viewer", True)
        self.declare_parameter("evidence_directory", "logs/arm_gui")

        self.pose_matched = bool(self.get_parameter("pose_matched").value)
        self.session_relative_baseline = bool(
            self.get_parameter("session_relative_baseline").value
        )
        self.numeric_test_only = bool(self.get_parameter("numeric_test_only").value)
        if not self.pose_matched and not self.session_relative_baseline and not self.numeric_test_only:
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
        if self.model_hash != PRODUCTION_MODEL_SHA256:
            raise RuntimeError(
                f"冻结生产MuJoCo模型哈希不匹配: {self.model_hash}"
            )
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
        self.gui_target: Optional[np.ndarray] = None
        self.gui_direction = "real_to_sim"
        self.gui_hardware_mode = "brake"
        self.target_subscription = self.create_subscription(
            Float64MultiArray, "/whole_arm/gui_targets", self.on_gui_target, 10
        )
        self.mode_subscription = self.create_subscription(
            String, "/whole_arm/gui_mode", self.on_gui_mode, 10
        )
        self.get_logger().info(
            ("MUJOCO_SESSION_POSE_MATCHED=YES" if self.pose_matched else
             "MUJOCO_SESSION_REFERENCE=SESSION_RELATIVE_BASELINE" if self.session_relative_baseline else
             "MUJOCO_SESSION_POSE_MATCHED=NO_NUMERIC_TEST_ONLY")
            + "; direct qpos mirror active; "
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
            if self.gui_direction == "sim_to_real" and self.gui_target is not None:
                relative = self.gui_target.copy()
                velocity = np.zeros(6, dtype=float)
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

    def on_gui_target(self, message: Float64MultiArray) -> None:
        try:
            target = np.asarray(message.data, dtype=float)
            if target.shape != (6,) or not np.all(np.isfinite(target)):
                raise ValueError("GUI目标必须是六个有限关节角")
            if np.any(np.abs(target) > math.radians(10.0) + 1.0e-12):
                raise ValueError("GUI目标超出会话参考正负十度")
            self.gui_target = target
        except Exception as exc:
            self.get_logger().warning(f"拒绝GUI仿真目标：{exc}")

    def on_gui_mode(self, message: String) -> None:
        try:
            value = json.loads(message.data)
            direction = value.get("direction")
            if direction not in {"real_to_sim", "sim_to_real"}:
                raise ValueError("方向模式不允许")
            self.gui_direction = direction
            self.gui_hardware_mode = str(value.get("hardware_mode", "brake"))
        except Exception as exc:
            self.get_logger().warning(f"拒绝GUI模式：{exc}")

    def validation(self) -> dict:
        return {
            "schema": "go-m8010-v15.30a-direct-qpos-mirror/1.0",
            "method": "DIRECT_QPOS_MIRROR",
            "model_path": str(self.model_path),
            "model_sha256": self.model_hash,
            "session_reference": "SESSION_REFERENCE_V1",
            "mujoco_session_pose_matched": "YES" if self.pose_matched else "NO",
            "mujoco_pose_semantics": (
                "VISUALLY_MATCHED" if self.pose_matched else
                "SESSION_RELATIVE_BASELINE" if self.session_relative_baseline else
                "NUMERIC_TEST_ONLY"
            ),
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
            "display_source": (
                "GUI_TARGET" if self.gui_direction == "sim_to_real" else "REAL_ENCODER"
            ),
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


def install_shutdown_handlers(stop_requested: threading.Event) -> dict[int, object]:
    """Keep ROS and GLFW teardown out of asynchronous signal handlers."""

    previous_handlers = {}

    def request_stop(_signum, _frame) -> None:
        stop_requested.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        previous_handlers[signum] = signal.getsignal(signum)
        signal.signal(signum, request_stop)
    return previous_handlers


def restore_shutdown_handlers(previous_handlers: dict[int, object]) -> None:
    for signum, handler in previous_handlers.items():
        signal.signal(signum, handler)


def close_viewer_and_wait(viewer, timeout_sec: float = 2.0) -> None:
    """Close the passive viewer and wait for its native UI thread to release data.

    MuJoCo's Handle.close() only raises the native exit request.  The render
    thread can still hold the model and data for a short period afterwards, so
    returning from Python immediately can race interpreter teardown and crash.
    """
    viewer.close()
    simulate_reference = getattr(viewer, "_sim", None)
    if not callable(simulate_reference):
        return
    deadline = time.monotonic() + timeout_sec
    while simulate_reference() is not None:
        if time.monotonic() >= deadline:
            raise RuntimeError("MuJoCo passive viewer did not close within timeout")
        time.sleep(0.01)


def main(args=None) -> None:
    stop_requested = threading.Event()
    previous_handlers = install_shutdown_handlers(stop_requested)
    initialized = False
    node: Optional[WholeArmMujocoMirror] = None
    viewer = None
    try:
        # rclpy's default handler asynchronously shuts down its context while
        # MuJoCo's passive GLFW thread is still alive.  Own SIGINT/SIGTERM here
        # and perform every native teardown step serially on this main thread.
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        initialized = True
        if stop_requested.is_set():
            return
        node = WholeArmMujocoMirror()
        if bool(node.get_parameter("use_viewer").value):
            import mujoco.viewer

            viewer = mujoco.viewer.launch_passive(node.model, node.data)
            while not stop_requested.is_set() and rclpy.ok() and viewer.is_running():
                rclpy.spin_once(node, timeout_sec=0.01)
                viewer.sync()
        else:
            while not stop_requested.is_set() and rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.05)
    except ExternalShutdownException:
        pass
    finally:
        try:
            if viewer is not None:
                closing_viewer = viewer
                viewer = None
                close_viewer_and_wait(closing_viewer)
                del closing_viewer
        finally:
            try:
                if node is not None:
                    node.destroy_node()
            finally:
                try:
                    if initialized and rclpy.ok():
                        rclpy.shutdown()
                finally:
                    restore_shutdown_handlers(previous_handlers)


if __name__ == "__main__":
    main()

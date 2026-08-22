"""ROS 2 state source: seven physical feedback streams to six logical joints.

The node deliberately exposes no command subscriber, trajectory action,
controller API, serial port, CAN transport, HOLD, FOC, or BRAKE operation.
The process which already owns each hardware bus publishes the raw feedback
contract on ``/whole_arm/motor_feedback_raw``.
"""

from __future__ import annotations

import csv
import json
import math
import statistics
import time
from pathlib import Path
from typing import Iterable, Optional

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String

from .state_model import JOINT_NAMES, MOTOR_NAMES, MirrorSessionReferenceV1, parse_feedback_payload


class CsvCapture:
    def __init__(self, path: Path, fieldnames: Iterable[str]) -> None:
        path.parent.mkdir(parents=True, exist_ok=True)
        self.stream = path.open("w", encoding="utf-8", newline="")
        self.writer = csv.DictWriter(self.stream, fieldnames=list(fieldnames))
        self.writer.writeheader()
        self.rows = 0

    def write(self, row: dict) -> None:
        self.writer.writerow(row)
        self.rows += 1
        if self.rows % 50 == 0:
            self.stream.flush()

    def close(self) -> None:
        if not self.stream.closed:
            self.stream.flush()
            self.stream.close()


class WholeArmStateNode(Node):
    def __init__(self) -> None:
        super().__init__("whole_arm_state_node")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("monitor_rate_hz", 2.0)
        self.declare_parameter("feedback_topic", "/whole_arm/motor_feedback_raw")
        self.declare_parameter("capture_samples", 25)
        self.declare_parameter("feedback_freshness_s", 0.10)
        self.declare_parameter("capture_max_span_deg", 0.25)
        self.declare_parameter("evidence_directory", "hardware/v15_30a_ft")

        rate_hz = float(self.get_parameter("publish_rate_hz").value)
        monitor_rate_hz = float(self.get_parameter("monitor_rate_hz").value)
        if rate_hz < 50.0 or monitor_rate_hz <= 0.0:
            raise ValueError("publish_rate_hz must be >=50 and monitor_rate_hz >0")
        self.model = MirrorSessionReferenceV1(
            capture_samples=int(self.get_parameter("capture_samples").value),
            freshness_s=float(self.get_parameter("feedback_freshness_s").value),
            capture_max_span_joint_rad=math.radians(
                float(self.get_parameter("capture_max_span_deg").value)
            ),
        )
        evidence = Path(str(self.get_parameter("evidence_directory").value)).resolve()
        common_fields = [
            "sequence", "ros_time_ns", "monotonic_ns", "healthy", "j2_e_sync_rad",
            *[f"{name}_position_rad" for name in JOINT_NAMES],
            *[f"{name}_velocity_rad_s" for name in JOINT_NAMES],
            *[f"{name}_temperature_c" for name in MOTOR_NAMES],
            *[f"{name}_merror" for name in MOTOR_NAMES],
            *[f"{name}_communication_ok" for name in MOTOR_NAMES],
            *[f"{name}_age_ms" for name in MOTOR_NAMES],
        ]
        self.whole_capture = CsvCapture(evidence / "whole_arm_state_capture.csv", common_fields)
        self.joint_capture = CsvCapture(
            evidence / "joint_states_capture.csv",
            ["sequence", "ros_time_ns", *[f"{name}_position_rad" for name in JOINT_NAMES],
             *[f"{name}_velocity_rad_s" for name in JOINT_NAMES]],
        )
        feedback_topic = str(self.get_parameter("feedback_topic").value)
        self.subscription = self.create_subscription(String, feedback_topic, self.on_feedback, 50)
        self.publisher = self.create_publisher(JointState, "/joint_states", 10)
        self.status_publisher = self.create_publisher(String, "/whole_arm/hardware_state", 10)
        self.timer = self.create_timer(1.0 / rate_hz, self.publish_state)
        self.monitor_timer = self.create_timer(1.0 / monitor_rate_hz, self.monitor)
        self.sequence = 0
        self.invalid_payload_count = 0
        self.publish_intervals_ms: list[float] = []
        self.source_latencies_ms: list[float] = []
        self.last_publish_ns: Optional[int] = None
        self.last_snapshot: Optional[dict] = None
        self.reference_announced = False
        self.get_logger().info(
            f"state-only aggregator listening on {feedback_topic}; no actuator command API exists"
        )

    def on_feedback(self, message: String) -> None:
        receipt_ns = time.monotonic_ns()
        try:
            payload = json.loads(message.data)
            for sample in parse_feedback_payload(payload, receipt_ns):
                self.model.update(sample)
        except Exception as exc:
            self.invalid_payload_count += 1
            self.get_logger().warning(f"rejected raw feedback payload: {exc}")

    def publish_state(self) -> None:
        now_monotonic_ns = time.monotonic_ns()
        if not self.model.try_capture(now_monotonic_ns):
            return
        if not self.reference_announced:
            self.get_logger().info(
                "MIRROR_SESSION_REFERENCE_V1 captured from all seven fresh motor sources; "
                "CAD_ZERO=PENDING ROS_ZERO=PENDING"
            )
            self.reference_announced = True
        try:
            snapshot = self.model.snapshot(now_monotonic_ns)
        except RuntimeError as exc:
            self.get_logger().warning(str(exc))
            return

        stamp = self.get_clock().now().to_msg()
        ros_time_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = list(JOINT_NAMES)
        joint_state.position = list(snapshot["position_rad"])
        joint_state.velocity = list(snapshot["velocity_rad_s"])
        self.publisher.publish(joint_state)

        if self.last_publish_ns is not None:
            self.publish_intervals_ms.append((now_monotonic_ns - self.last_publish_ns) / 1.0e6)
            self.publish_intervals_ms = self.publish_intervals_ms[-5000:]
        self.last_publish_ns = now_monotonic_ns
        self.sequence += 1
        row = {
            "sequence": self.sequence,
            "ros_time_ns": ros_time_ns,
            "monotonic_ns": now_monotonic_ns,
            "healthy": int(snapshot["healthy"]),
            "j2_e_sync_rad": snapshot["j2_e_sync_rad"],
        }
        joint_row = {"sequence": self.sequence, "ros_time_ns": ros_time_ns}
        for index, name in enumerate(JOINT_NAMES):
            row[f"{name}_position_rad"] = snapshot["position_rad"][index]
            row[f"{name}_velocity_rad_s"] = snapshot["velocity_rad_s"][index]
            joint_row[f"{name}_position_rad"] = snapshot["position_rad"][index]
            joint_row[f"{name}_velocity_rad_s"] = snapshot["velocity_rad_s"][index]
        for name in MOTOR_NAMES:
            motor = snapshot["per_motor"][name]
            row[f"{name}_temperature_c"] = motor["temperature_c"]
            row[f"{name}_merror"] = motor["merror"]
            row[f"{name}_communication_ok"] = int(motor["communication_ok"])
            row[f"{name}_age_ms"] = motor["age_ms"]
            self.source_latencies_ms.append(motor["source_latency_ms"])
        self.source_latencies_ms = self.source_latencies_ms[-35000:]
        self.whole_capture.write(row)
        self.joint_capture.write(joint_row)

        snapshot["sequence"] = self.sequence
        snapshot["invalid_payload_count"] = self.invalid_payload_count
        snapshot["cad_zero"] = "PENDING"
        snapshot["ros_zero"] = "PENDING"
        snapshot["j2_active_motion_used"] = False
        snapshot["zero_gravity"] = "NOT_IMPLEMENTED"
        snapshot["timing"] = self.timing_summary()
        self.status_publisher.publish(String(data=json.dumps(snapshot, ensure_ascii=False)))
        self.last_snapshot = snapshot

    def timing_summary(self) -> dict:
        intervals = self.publish_intervals_ms
        latencies = self.source_latencies_ms
        median_rate = None
        if intervals:
            median_interval = statistics.median(intervals)
            median_rate = 1000.0 / median_interval if median_interval > 0.0 else None
        return {
            "median_publish_rate_hz": median_rate,
            "p95_source_latency_ms": percentile(latencies, 0.95),
        }

    def monitor(self) -> None:
        if self.last_snapshot is None:
            counts = {name: len(self.model.history[name]) for name in MOTOR_NAMES}
            self.get_logger().info(f"等待七路新鲜反馈以捕获会话参考: {counts}")
            return
        snapshot = self.last_snapshot
        degrees = [math.degrees(value) for value in snapshot["position_rad"]]
        health = "OK" if snapshot["healthy"] else "DEGRADED"
        temperatures = ",".join(
            f"{name}:{snapshot['per_motor'][name]['temperature_c']:.0f}C"
            for name in MOTOR_NAMES
        )
        errors = ",".join(
            f"{name}:{snapshot['per_motor'][name]['merror']}" for name in MOTOR_NAMES
        )
        self.get_logger().info(
            " ".join(f"J{i + 1}={value:+.2f}deg" for i, value in enumerate(degrees))
            + f" J2_e_sync={math.degrees(snapshot['j2_e_sync_rad']):+.3f}deg"
            + f" COMM={health} TEMP=[{temperatures}] MERROR=[{errors}]"
        )

    def destroy_node(self) -> bool:
        self.whole_capture.close()
        self.joint_capture.close()
        return super().destroy_node()


def percentile(values: list[float], fraction: float) -> Optional[float]:
    if not values:
        return None
    ordered = sorted(values)
    index = max(0, math.ceil(fraction * len(ordered)) - 1)
    return ordered[index]


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[WholeArmStateNode] = None
    try:
        node = WholeArmStateNode()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

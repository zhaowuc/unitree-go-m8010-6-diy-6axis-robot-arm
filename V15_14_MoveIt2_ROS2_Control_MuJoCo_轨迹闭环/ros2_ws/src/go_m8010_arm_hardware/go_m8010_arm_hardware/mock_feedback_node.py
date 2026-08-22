"""Deterministic no-hardware feedback source for integration verification."""

from __future__ import annotations

import json
import math
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .state_model import GEAR_RATIO


class MockFeedbackNode(Node):
    def __init__(self) -> None:
        super().__init__("mock_motor_feedback")
        self.publisher = self.create_publisher(String, "/whole_arm/motor_feedback_raw", 50)
        self.started = time.monotonic()
        self.timer = self.create_timer(0.01, self.publish_feedback)

    def publish_feedback(self) -> None:
        now = time.monotonic()
        elapsed = now - self.started
        # Stay still long enough for the reference, then visibly move one axis.
        phase = max(0.0, elapsed - 1.0)
        q = 0.0 if phase == 0.0 else math.radians(7.0) * math.sin(2.0 * math.pi * 0.1 * phase)
        dq = 0.0 if phase == 0.0 else math.radians(7.0) * 2.0 * math.pi * 0.1 * math.cos(2.0 * math.pi * 0.1 * phase)
        stamp = time.monotonic_ns()
        samples = [
            {"motor": "J1", "position_rad": 1.0 + GEAR_RATIO * q, "velocity_rad_s": GEAR_RATIO * dq},
            {"motor": "J2A", "position_rad": 2.0, "velocity_rad_s": 0.0},
            {"motor": "J2B", "position_rad": 3.0, "velocity_rad_s": 0.0},
            {"motor": "J3", "position_rad": 4.0, "velocity_rad_s": 0.0},
            {"motor": "J4", "position_rad": 5.0, "velocity_rad_s": 0.0},
            {"motor": "J5", "position_rad": 6.0, "velocity_rad_s": 0.0},
            {"motor": "J6", "position_rad": 0.5, "velocity_rad_s": 0.0},
        ]
        for sample in samples:
            sample.update({"temperature_c": 30.0, "merror": 0, "communication_ok": True})
        payload = {"schema": "go-m8010-motor-feedback/1.0", "source_monotonic_ns": stamp, "samples": samples}
        self.publisher.publish(String(data=json.dumps(payload)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MockFeedbackNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

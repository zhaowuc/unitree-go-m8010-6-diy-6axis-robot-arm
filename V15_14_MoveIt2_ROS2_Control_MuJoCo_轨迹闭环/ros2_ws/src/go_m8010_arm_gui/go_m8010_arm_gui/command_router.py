"""把 ROS2 GUI 命令严格校验后转发给本机独立硬件控制进程。"""

from __future__ import annotations

import json
import math
import socket
import time
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


ALLOWED_MODES = {"brake", "drag", "hold", "position"}
TARGET_LIMIT_RAD = math.radians(10.0)
J2_ACTIVE_CONTROL_BLOCKED = True
DOMAIN_JOINT_INDICES = {
    "J1": frozenset({0}),
    "J2": frozenset({1}),
    "J345": frozenset({2, 3, 4}),
    "J6": frozenset({5}),
}


def validate_command(text: str) -> tuple[dict, bytes]:
    value = json.loads(text)
    schema = value.get("schema")
    if schema not in {"go-m8010-gui-command/1.0", "go-m8010-gui-command/1.1"}:
        raise ValueError("命令格式不匹配")
    mode = value.get("mode")
    if mode not in ALLOWED_MODES:
        raise ValueError("控制模式不允许")
    if schema == "go-m8010-gui-command/1.0" and mode != "brake":
        raise ValueError("旧版协议仅允许制动")
    targets = value.get("targets_rad")
    if not isinstance(targets, list) or len(targets) != 6:
        raise ValueError("必须包含六个逻辑关节目标")
    targets = [float(item) for item in targets]
    if not all(math.isfinite(item) and abs(item) <= TARGET_LIMIT_RAD + 1e-12 for item in targets):
        raise ValueError("关节目标超出会话参考正负十度")
    active_joint_mask = value.get("active_joint_mask")
    if schema == "go-m8010-gui-command/1.0":
        active_joint_mask = [False] * 6
    elif (
        not isinstance(active_joint_mask, list)
        or len(active_joint_mask) != 6
        or not all(type(item) is bool for item in active_joint_mask)
    ):
        raise ValueError("关节激活掩码必须是六个布尔值")
    if mode == "brake" and any(active_joint_mask):
        raise ValueError("制动命令不得携带激活关节")
    activation_epoch = value.get("activation_epoch", 0)
    if (
        schema == "go-m8010-gui-command/1.1"
        and (
            type(activation_epoch) is not int
            or not 0 <= activation_epoch <= (1 << 63) - 1
        )
    ):
        raise ValueError("激活纪元必须是非负整数")
    if schema == "go-m8010-gui-command/1.0":
        activation_epoch = 0
    if mode in {"drag", "hold", "position"} and any(active_joint_mask) and activation_epoch == 0:
        raise ValueError("主动命令的激活纪元必须大于零")
    kp = [float(item) for item in value.get("kp", [0.5, 1.0, 0.6, 0.5, 0.5, 0.0])]
    kd = [float(item) for item in value.get("kd", [0.05] * 5 + [0.0])]
    if len(kp) != 6 or len(kd) != 6 or not all(math.isfinite(item) and item >= 0.0 for item in kp + kd):
        raise ValueError("关节增益格式不正确")
    kp_limits = [0.5, 1.0, 0.6, 0.5, 0.5, 0.0]
    kd_limits = [0.05, 0.05, 0.05, 0.05, 0.05, 0.0]
    if any(item > limit + 1e-12 for item, limit in zip(kp, kp_limits)) or any(
        item > limit + 1e-12 for item, limit in zip(kd, kd_limits)
    ):
        raise ValueError("关节增益超过冻结上限")
    maximum_velocity = abs(float(value.get("maximum_velocity_rad_s", math.radians(5.0))))
    maximum_acceleration = abs(float(value.get("maximum_acceleration_rad_s2", math.radians(20.0))))
    if (not math.isfinite(maximum_velocity) or maximum_velocity <= 0.0 or
            not math.isfinite(maximum_acceleration) or maximum_acceleration <= 0.0):
        raise ValueError("速度或加速度必须为正的有限数")
    normalized = {
        "schema": "go-m8010-gui-command/1.1",
        "sequence": int(value.get("sequence", 0)),
        "source_monotonic_ns": time.monotonic_ns(),
        "mode": mode,
        "targets_rad": targets,
        "active_joint_mask": list(active_joint_mask),
        "activation_epoch": activation_epoch,
        "maximum_velocity_rad_s": min(maximum_velocity, math.radians(5.0)),
        "maximum_acceleration_rad_s2": min(maximum_acceleration, math.radians(20.0)),
        "kp": kp,
        "kd": kd,
    }
    return normalized, json.dumps(normalized, separators=(",", ":")).encode("utf-8")


def payload_for_domain(normalized: dict, domain: str) -> bytes:
    """Keep the unresolved J2 active-control domain fail-closed.

    J2 feedback and synchronization monitoring remain live, but drag, hold and
    position requests are never forwarded to the J2 worker in V15.30A.  This
    lets the independently validated joints move without implicitly granting
    J2 active-motion authority.
    """

    if domain not in DOMAIN_JOINT_INDICES:
        raise ValueError("未知硬件故障域")
    domain_command = normalized
    active_mode = normalized["mode"] in {"drag", "hold", "position"}
    domain_selected = any(
        normalized["active_joint_mask"][index]
        for index in DOMAIN_JOINT_INDICES[domain]
    )
    if (
        active_mode
        and (
            not domain_selected
            or (J2_ACTIVE_CONTROL_BLOCKED and domain == "J2")
        )
    ):
        domain_command = dict(normalized)
        domain_command["mode"] = "brake"
        domain_command["active_joint_mask"] = [False] * 6
    return json.dumps(domain_command, separators=(",", ":")).encode("utf-8")


class CommandRouter(Node):
    def __init__(self) -> None:
        super().__init__("arm_gui_command_router")
        self.declare_parameter("go_udp_port", 15310)
        self.declare_parameter("j2_udp_port", 15312)
        self.declare_parameter("j345_udp_port", 15313)
        self.declare_parameter("j6_udp_port", 15311)
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.destinations = [
            ("J1", ("127.0.0.1", int(self.get_parameter("go_udp_port").value))),
            ("J2", ("127.0.0.1", int(self.get_parameter("j2_udp_port").value))),
            ("J345", ("127.0.0.1", int(self.get_parameter("j345_udp_port").value))),
            ("J6", ("127.0.0.1", int(self.get_parameter("j6_udp_port").value))),
        ]
        self.publisher = self.create_publisher(String, "/whole_arm/control_status", 10)
        self.subscription = self.create_subscription(
            String, "/whole_arm/gui_command", self.on_command, 10
        )
        self.last_command: Optional[dict] = None
        self.rejected = 0
        self.timer = self.create_timer(0.5, self.publish_status)
        self.get_logger().info("GUI命令路由已启动，仅允许本机UDP目标")

    def on_command(self, message: String) -> None:
        try:
            normalized, _payload = validate_command(message.data)
            for domain, destination in self.destinations:
                self.socket.sendto(payload_for_domain(normalized, domain), destination)
            self.last_command = normalized
        except Exception as exc:
            self.rejected += 1
            self.get_logger().warning(f"拒绝GUI命令：{exc}")

    def publish_status(self) -> None:
        age_ms = None
        if self.last_command is not None:
            age_ms = (time.monotonic_ns() - self.last_command["source_monotonic_ns"]) / 1.0e6
        self.publisher.publish(String(data=json.dumps({
            "schema": "go-m8010-command-router-status/1.0",
            "received": self.last_command is not None,
            "last_mode": None if self.last_command is None else self.last_command["mode"],
            "j2_active_control_blocked": J2_ACTIVE_CONTROL_BLOCKED,
            "j2_forwarded_mode": None if self.last_command is None else "brake",
            "last_active_joint_mask": None if self.last_command is None else
                self.last_command["active_joint_mask"],
            "last_activation_epoch": None if self.last_command is None else
                self.last_command["activation_epoch"],
            "last_command_age_ms": age_ms,
            "rejected_commands": self.rejected,
        }, ensure_ascii=False)))

    def destroy_node(self) -> bool:
        self.socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[CommandRouter] = None
    try:
        node = CommandRouter()
        rclpy.spin(node)
    except ExternalShutdownException:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

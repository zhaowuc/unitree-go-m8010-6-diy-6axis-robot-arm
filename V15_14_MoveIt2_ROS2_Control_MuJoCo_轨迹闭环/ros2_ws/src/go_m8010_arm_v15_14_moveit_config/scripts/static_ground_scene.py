#!/usr/bin/env python3
"""Install the MuJoCo ground plane equivalent into the MoveIt planning scene."""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject, PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node
from shape_msgs.msg import SolidPrimitive


GROUND_ID = "v15_14_mujoco_ground"
GROUND_TOP_Z_M = -0.09
GROUND_THICKNESS_M = 0.02
GROUND_SIZE_XY_M = 4.0


class StaticGroundScene(Node):
    def __init__(self) -> None:
        super().__init__("v15_14_static_ground_scene")
        self.client = self.create_client(ApplyPlanningScene, "/apply_planning_scene")

    def apply(self) -> None:
        if not self.client.wait_for_service(timeout_sec=60.0):
            raise RuntimeError("/apply_planning_scene unavailable")

        ground = CollisionObject()
        ground.header.frame_id = "world"
        ground.id = GROUND_ID
        ground.operation = CollisionObject.ADD
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [
            GROUND_SIZE_XY_M,
            GROUND_SIZE_XY_M,
            GROUND_THICKNESS_M,
        ]
        pose = Pose()
        pose.position.z = GROUND_TOP_Z_M - GROUND_THICKNESS_M / 2.0
        pose.orientation.w = 1.0
        ground.primitives = [primitive]
        ground.primitive_poses = [pose]

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects = [ground]
        future = self.client.call_async(ApplyPlanningScene.Request(scene=scene))
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        response = future.result()
        if response is None or not response.success:
            raise RuntimeError("MoveIt rejected the V15.14 ground scene")
        self.get_logger().info(
            "installed v15_14_mujoco_ground: world box 4x4x0.02 m, top z=-0.09 m"
        )


def main() -> None:
    rclpy.init()
    node = StaticGroundScene()
    try:
        node.apply()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

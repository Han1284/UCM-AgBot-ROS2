#!/usr/bin/env python3

"""Install Gazebo plant proxies into MoveIt's persistent PlanningScene."""

import rclpy
from moveit_msgs.msg import PlanningScene
from moveit_msgs.srv import ApplyPlanningScene
from rclpy.node import Node

from leaf_manipulation_sim.plant_collision_geometry import (
    plant_collision_objects,
)


class PlanningSceneInitializer(Node):
    def __init__(self):
        super().__init__('leaf_planning_scene_initializer')
        self.client = self.create_client(
            ApplyPlanningScene,
            '/apply_planning_scene',
        )

    def apply(self):
        if not self.client.wait_for_service(timeout_sec=20.0):
            self.get_logger().error(
                'MoveIt /apply_planning_scene service was not available')
            return False

        request = ApplyPlanningScene.Request()
        request.scene = PlanningScene()
        request.scene.is_diff = True
        request.scene.world.collision_objects = plant_collision_objects()
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=20.0)
        if not future.done() or future.result() is None:
            self.get_logger().error('PlanningScene update timed out')
            return False
        if not future.result().success:
            self.get_logger().error('MoveIt rejected the PlanningScene update')
            return False

        self.get_logger().info(
            'Installed pot and 20 leaf collision proxies in MoveIt')
        return True


def main(args=None):
    rclpy.init(args=args)
    node = PlanningSceneInitializer()
    try:
        node.apply()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

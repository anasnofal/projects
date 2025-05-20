#!/usr/bin/env python3
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped

class GoToBinsNode(Node):

    def __init__(self):
        super().__init__('go_to_bins')

        # 1) Nav2 action client
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # 2) Static list of your bin positions (x, y, yaw)
        self.targets = [
            {'name': 'first_2015_trash_can_0', 'x': 0.900, 'y': 2.1, 'theta': 0.0},
            {'name': 'first_2015_trash_can_1', 'x': -1.400, 'y': -0.0450, 'theta': 0.0},
            {'name': 'first_2015_trash_can_2', 'x': 1.100, 'y': 4.000, 'theta': 0.0},
        ]

        # 3) Start sending goals
        self.send_goals()

    def _make_quaternion(self, yaw: float):
        """Return a (x,y,z,w) quaternion for a yaw-only rotation."""
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def send_goals(self):
        for t in self.targets:
            # Build the goal message
            goal_msg = NavigateToPose.Goal()
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = t['x']
            pose.pose.position.y = t['y']
            qx, qy, qz, qw = self._make_quaternion(t['theta'])
            pose.pose.orientation.x = qx
            pose.pose.orientation.y = qy
            pose.pose.orientation.z = qz
            pose.pose.orientation.w = qw
            goal_msg.pose = pose

            # Wait for Nav2 server, send goal
            self._nav_client.wait_for_server()
            self.get_logger().info(
                f"Sending robot to '{t['name']}' at ({t['x']:.2f}, {t['y']:.2f})"
            )
            self._nav_client.send_goal_async(goal_msg)

            # Simple pause to let the robot move (replace later with real result handling)
            time.sleep(10.0)

def main(args=None):
    rclpy.init(args=args)
    node = GoToBinsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

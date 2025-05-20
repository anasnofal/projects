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
_trash_can_2'
]


        # 4) Query and build target list
        self.targets = self._fetch_bin_positions()

        # 5) Once we have targets, start sending goals
        self.send_goals()

    def _fetch_bin_positions(self):
        """Call /gazebo/get_model_state for each bin name and return list of dicts."""
        targets = []
        for name in self.bin_names:
            req = GetModelState.Request()
            req.model_name = name
            req.relative_entity_name = 'world'
            future = self._state_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            if res is None or not res.success:
                self.get_logger().warn(f"Couldn't get state for '{name}'. Skipping.")
                continue

            x = res.pose.position.x
            y = res.pose.position.y
            # Face “forward” along +x by default
            theta = 0.0
            self.get_logger().info(f"Found '{name}' at x={x:.2f}, y={y:.2f}")
            targets.append({'name': name, 'x': x, 'y': y, 'theta': theta})
        return targets

    def _make_quaternion(self, yaw: float):
        """Return (x, y, z, w) quaternion for a yaw-only rotation."""
        half = yaw * 0.5
        return (0.0, 0.0, math.sin(half), math.cos(half))

    def send_goals(self):
        for t in self.targets:
            # Build the goal
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

            # Send it
            self._nav_client.wait_for_server()
            self.get_logger().info(f"Sending robot to '{t['name']}' at ({t['x']:.2f},{t['y']:.2f})")
            self._nav_client.send_goal_async(goal_msg)

            # Wait for 10s before next goal (replace with real result handling later)
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


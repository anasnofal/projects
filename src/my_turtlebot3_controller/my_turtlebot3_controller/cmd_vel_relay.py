#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from gazebo_msgs.srv import GetModelState
from rclpy.action import ActionClient
from tf_transformations import quaternion_from_euler
import time

class GoToBinsNode(Node):

    def __init__(self):
        super().__init__('go_to_bins')

        # Nav2 action client
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        # Gazebo service client for model states
        self.get_state_cli = self.create_client(
            GetModelState, '/gazebo/get_model_state')
        while not self.get_state_cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /gazebo/get_model_state service...')

        # List your bin model names here
        self.bin_names = ['trash_bin_1', 'trash_bin_2']
        self.targets = []

        # Query each bin for its (x,y) in the world frame
        for name in self.bin_names:
            req = GetModelState.Request()
            req.model_name = name
            req.relative_entity_name = 'world'
            future = self.get_state_cli.call_async(req)
            rclpy.spin_until_future_complete(self, future)
            res = future.result()
            if res.success:
                x = res.pose.position.x
                y = res.pose.position.y
                theta = 0.0
                self.targets.append({'x': x, 'y': y, 'theta': theta})
                self.get_logger().info(f'Found {name} at x={x:.2f}, y={y:.2f}')
            else:
                self.get_logger().error(f'Failed to get state for {name}')

        # Once we have real coords, send the goals
        self.send_goals()

    def send_goals(self):
        for t in self.targets:
            # Build the NavigateToPose goal
            goal_msg = NavigateToPose.Goal()
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = t['x']
            pose.pose.position.y = t['y']
            # Convert yaw (theta) to quaternion
            q = quaternion_from_euler(0, 0, t['theta'])
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]
            goal_msg.pose = pose

            self.client.wait_for_server()
            self.get_logger().info(f'Sending goal to {t}')
            self.client.send_goal_async(goal_msg)

            # Simple fixed wait; you can replace with proper result callbacks
            self.get_logger().info('Waiting 10 seconds for the robot to navigate…')
            time.sleep(10)

def main(args=None):
    rclpy.init(args=args)
    node = GoToBinsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler # For converting yaw to quaternion
import math
import time

class GoToBinsNode(Node):
    def __init__(self):
        super().__init__('GoToBinsNode')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose') # Standard Nav2 action server
        self.get_logger().info("Coordinate Navigator Node Initialized.")
        self.goal_handle = None
        self.goal_done_status = None # To store final status

    def wait_for_action_server(self, timeout_sec=5.0):
        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        if not self._action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('/navigate_to_pose action server not available after waiting.')
            return False
        self.get_logger().info('/navigate_to_pose action server is available.')
        return True

    def send_navigation_goal(self, x, y, yaw_degrees):
        if not self.wait_for_action_server():
            return False

        self.goal_done_status = None # Reset status for new goal

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map' # Goals are in the map frame

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0  # Assuming 2D navigation

        yaw_radians = math.radians(float(yaw_degrees))
        q = quaternion_from_euler(0, 0, yaw_radians)  # Roll, Pitch, Yaw
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        self.get_logger().info(f"Sending goal: X={x}, Y={y}, Yaw={yaw_degrees}°")
        
        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        
        send_goal_future.add_done_callback(self.goal_response_callback)
        return True

    def goal_response_callback(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().error('Goal rejected by Nav2 server.')
            self.goal_done_status = rclpy.action.GoalStatus.STATUS_REJECTED
            return

        self.get_logger().info('Goal accepted by Nav2 server. Waiting for result...')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status = future.result().status
        # result = future.result().result # NavigateToPose.Result is empty
        self.goal_done_status = status

        if status == rclpy.action.GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation Succeeded!')
        elif status == rclpy.action.GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Navigation Aborted by Nav2.')
        elif status == rclpy.action.GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Navigation Canceled.')
        else:
            self.get_logger().info(f'Navigation finished with status: {status}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f} m")
        # You can log other feedback like feedback.current_pose, feedback.navigation_time etc.

    def is_goal_done(self):
        return self.goal_done_status is not None

def main(args=None):
    rclpy.init(args=args)
    navigator_node = GoToBinsNode()

    # --- DEFINE YOUR TARGET COORDINATES AND YAW (in degrees) HERE ---
    # Replace with coordinates you determined in Step 1 from your map
    target_x = 1.5
    target_y = -1.0
    target_yaw_degrees = 0.0
    # --------------------------------------------------------------

    if navigator_node.send_navigation_goal(target_x, target_y, target_yaw_degrees):
        while rclpy.ok() and not navigator_node.is_goal_done():
            rclpy.spin_once(navigator_node, timeout_sec=0.1)
        
        if navigator_node.is_goal_done():
            navigator_node.get_logger().info("Goal processing sequence complete.")
    else:
        navigator_node.get_logger().error("Failed to send goal.")

    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
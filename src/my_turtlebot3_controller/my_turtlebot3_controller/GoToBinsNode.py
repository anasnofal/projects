#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
import math
import time

from action_msgs.msg import GoalStatus as ActionGoalStatus 

class GoToBinsNode(Node):
    def __init__(self):
        super().__init__('GoToBinsNode')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose') 
        self.get_logger().info("Coordinate Navigator Node Initialized.")
        self.goal_handle = None
        self.goal_done_status = None 
        #self.current_goal_active = False
        #self.goal_subscriber = self.create_subscription()

    def wait_for_action_server(self, timeout_sec=5.0):
        self.get_logger().info('Waiting for /navigate_to_pose action server...')
        if not self._action_client.wait_for_server(timeout_sec=timeout_sec):
            self.get_logger().error('/navigate_to_pose action server not available after waiting.')
            return False
        self.get_logger().info('/navigate_to_pose action server is available.')
        return True

    def send_navigation_goal(self, x, y, yaw_degrees):
        if not self.wait_for_action_server():
            self.goal_done_status = -1 
            return False

        self.goal_done_status = None 

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.header.frame_id = 'map' 

        goal_msg.pose.pose.position.x = float(x)
        goal_msg.pose.pose.position.y = float(y)
        goal_msg.pose.pose.position.z = 0.0  

        yaw_radians = math.radians(float(yaw_degrees))
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = math.sin(yaw_radians/2.0)
        goal_msg.pose.pose.orientation.w = math.cos(yaw_radians/2.0)

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
            self.goal_done_status = ActionGoalStatus.STATUS_REJECTED
            return

        self.get_logger().info('Goal accepted by Nav2 server. Waiting for result...')
        result_future = self.goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        status_code = future.result().status 
        self.goal_done_status = status_code 

        # Compare with the imported ActionGoalStatus constants
        if status_code == ActionGoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation Succeeded!')
        elif status_code == ActionGoalStatus.STATUS_ABORTED:
            self.get_logger().error(f'Navigation Aborted by Nav2. Status Code: {status_code}')
        elif status_code == ActionGoalStatus.STATUS_CANCELED:
            self.get_logger().warn(f'Navigation Canceled. Status Code: {status_code}')
        else: # This will catch other statuses like REJECTED (if it passed acceptance), UNKNOWN, etc.
            self.get_logger().info(f'Navigation finished with status code: {status_code}')

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        # Example: log only if significant distance remains, or at intervals
        # if feedback.distance_remaining > 0.1:
        #    self.get_logger().info(f"Distance remaining: {feedback.distance_remaining:.2f} m")
        pass

    def is_goal_done(self):
        return self.goal_done_status is not None

def main(args=None):
    rclpy.init(args=args)
    navigator_node = GoToBinsNode()

    target_x = 0.25
    target_y = -0.4
    target_yaw_degrees = 0.0

    if navigator_node.send_navigation_goal(target_x, target_y, target_yaw_degrees):
        # Spin until the goal is done or Ctrl-C
        while rclpy.ok() and not navigator_node.is_goal_done():
            rclpy.spin_once(navigator_node, timeout_sec=0.1)
        
        if navigator_node.is_goal_done(): # Check one last time after loop exits
            navigator_node.get_logger().info(f"Goal processing sequence complete with status: {navigator_node.goal_done_status}")
    else:
        navigator_node.get_logger().error("Failed to send goal initially (action server not available).")

    navigator_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
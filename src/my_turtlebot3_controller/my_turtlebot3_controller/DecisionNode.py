#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Quaternion
import math
import time

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # Publisher to send navigation goals to NavigationExecutorNode
        self.goal_publisher = self.create_publisher(PoseStamped, '/dispatch_nav_goal', 10)

        # Subscriber to bin fill levels (from BinSensorMockNode)
        self.fill_level_subscriber = self.create_subscription(
            Float32MultiArray,
            '/bin_fill_levels',
            self.fill_level_callback,
            10)
        
        # Subscriber to get status FROM NavigationExecutorNode
        self.nav_status_subscriber = self.create_subscription(
            String, 
            '/navigation_executor_status',
            self.navigation_status_callback,
            10)
        
        # Publisher to command BinSensorMockNode to reset fill level
        self.reset_bin_publisher = self.create_publisher(String, '/set_bin_level_zero', 10)

        # --- Configuration ---
        self.bins_data = {
            0: {'name': 'bin_0', 'location': (0.25, -0.4, 0.0),  'fill': 0.0},
            1: {'name': 'bin_1', 'location': (1.0,  1.5, 90.0), 'fill': 0.0},
            2: {'name': 'bin_2', 'location': (-1.0, 0.8, -90.0), 'fill': 0.0},
        }
        self.fill_threshold = 75.0 
        self.required_stop_duration_sec = 10.0 # The wait time at the bin
        
        self.current_task_phase = "IDLE" # IDLE, WAITING_FOR_NAV_SUCCESS, WAITING_AT_BIN
        self.dispatched_bin_index = None
        
        self.at_bin_wait_timer = None 

        self.decision_timer = self.create_timer(5.0, self.make_decision_cycle) # main timer 

        self.get_logger().info("Decision Node Initialized. Bin data:")
        for i, data in self.bins_data.items():
            self.get_logger().info(f"  {data['name']} (Index {i}): Loc {data['location']}, Fill {data['fill']}%")

    def fill_level_callback(self, msg: Float32MultiArray):
        for i, level in enumerate(msg.data):
            if i in self.bins_data:
                self.bins_data[i]['fill'] = level

    def navigation_status_callback(self, msg: String):
        nav_status = msg.data
        self.get_logger().info(f"Received Nav Executor Status: '{nav_status}' (Current Phase: '{self.current_task_phase}')")

        if self.current_task_phase == "WAITING_FOR_NAV_SUCCESS":
            if nav_status == "SUCCEEDED_AT_POSE":
                self.get_logger().info(f"Navigation to bin {self.dispatched_bin_index} succeeded. Starting {self.required_stop_duration_sec}s wait.")
                self.current_task_phase = "WAITING_AT_BIN"
                if self.at_bin_wait_timer is not None: 
                    self.at_bin_wait_timer.cancel()
                self.at_bin_wait_timer = self.create_timer(self.required_stop_duration_sec, self.ten_second_wait_is_over_callback)

            elif nav_status in ["FAILED_NAVIGATION", "ABORTED_NAVIGATION", "CANCELED_NAVIGATION", "REJECTED", "IDLE"]:
                self.get_logger().warn(f"Navigation task for bin {self.dispatched_bin_index} did not succeed (Status: '{nav_status}'). Resetting task.")
                self.reset_task_state()

    def ten_second_wait_is_over_callback(self):
        """This function is called only once by the timer after the 10s wait."""
        self.get_logger().info(f"{self.required_stop_duration_sec}s wait at bin {self.dispatched_bin_index} is complete. Commanding bin empty.")
        
        if self.at_bin_wait_timer and not self.at_bin_wait_timer.is_canceled():
            self.at_bin_wait_timer.cancel()
        self.at_bin_wait_timer = None

        if self.dispatched_bin_index is not None:
            bin_id_str = self.bins_data[self.dispatched_bin_index]['name']
            reset_cmd = String()
            reset_cmd.data = bin_id_str
            self.reset_bin_publisher.publish(reset_cmd)
            self.get_logger().info(f"Reset command sent for bin '{bin_id_str}'. Task fully complete.")
        
        # Reset state to be ready for the next decision cycle
        self.reset_task_state()

    def make_decision_cycle(self):
        if self.current_task_phase != "IDLE":
            self.get_logger().debug(f"Decision cycle: Robot busy in phase '{self.current_task_phase}'. Waiting...")
            return

        target_id_to_dispatch = None
        max_fill = -1.0

        for id_key, data in self.bins_data.items():
            if data['fill'] >= self.fill_threshold:
                if data['fill'] > max_fill:
                    max_fill = data['fill']
                    target_id_to_dispatch = id_key
        
        if target_id_to_dispatch is not None:
            self.dispatched_bin_index = target_id_to_dispatch
            
            x, y, yaw_deg = self.bins_data[self.dispatched_bin_index]['location']
            
            goal_pose_msg = PoseStamped()
            goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
            goal_pose_msg.header.frame_id = 'map'
            goal_pose_msg.pose.position.x = float(x)
            goal_pose_msg.pose.position.y = float(y)
            goal_pose_msg.pose.position.z = 0.0

            yaw_rad = math.radians(float(yaw_deg))
            goal_pose_msg.pose.orientation.x = 0.0
            goal_pose_msg.pose.orientation.y = 0.0
            goal_pose_msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
            goal_pose_msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
            
            self.goal_publisher.publish(goal_pose_msg)
            self.current_task_phase = "WAITING_FOR_NAV_SUCCESS" 
            self.get_logger().info(f"Decision: Dispatched robot to {self.bins_data[self.dispatched_bin_index]['name']} (Index: {self.dispatched_bin_index})")

    def reset_task_state(self):
        self.get_logger().info("Resetting DecisionNode task state to IDLE.")
        self.current_task_phase = "IDLE"
        self.dispatched_bin_index = None

def main(args=None):
    rclpy.init(args=args)
    decision_node = DecisionNode()
    try:
        rclpy.spin(decision_node)
    except KeyboardInterrupt:
        decision_node.get_logger().info("Decision node shutting down.")
    finally:
        if decision_node.decision_timer and not decision_node.decision_timer.is_canceled():
            decision_node.decision_timer.cancel()
        if decision_node.at_bin_wait_timer and not decision_node.at_bin_wait_timer.is_canceled():
            decision_node.at_bin_wait_timer.cancel()
        
        decision_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
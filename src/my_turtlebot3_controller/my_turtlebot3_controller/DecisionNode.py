#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Float32MultiArray, String # For fill levels and reset command
from geometry_msgs.msg import PoseStamped, Point as PosePoint, Quaternion # For goals and pose checking
import math
import time # For initial sleep or delays

# You WON'T need tf_transformations here if you do manual quaternion calculation below

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # Publisher to send navigation goals TO NavigationExecutorNode
        self.goal_publisher = self.create_publisher(PoseStamped, '/dispatch_nav_goal', 10)

        # Subscriber to bin fill levels (from BinSensorMockNode)
        self.fill_level_subscriber = self.create_subscription(
            Float32MultiArray, # Example: data = [fill_level_bin0, fill_level_bin1, ...]
            '/bin_fill_levels',
            self.fill_level_callback,
            10)
        
        # Subscriber to get status FROM NavigationExecutorNode
        self.nav_status_subscriber = self.create_subscription(
            String, 
            '/navigation_executor_status',
            self.navigation_status_callback,
            10)

        # Subscriber to get current robot pose for proximity/stop checks
        self.robot_pose_subscriber = self.create_subscription(
            PoseStamped, # Assuming AMCL or similar provides PoseStamped in map frame
            '/amcl_pose', # Or your localized robot pose topic
            self.robot_pose_callback,
            10)
        self.current_robot_pose = None # Store current PoseStamped

        # Publisher to command BinSensorMockNode to reset fill level
        self.reset_bin_publisher = self.create_publisher(String, '/set_bin_level_zero', 10)

        # Bin Data: bin_index: {'name': str, 'location': (x,y,yaw_deg), 'fill': float, 'is_targeted': bool}
        # IMPORTANT: Populate with YOUR actual bin map coordinates!
        self.bins_data = {
            0: {'name': 'Bin_0', 'location': (0.25, -0.4, 0.0),  'fill': 0.0, 'is_targeted': False},
            1: {'name': 'Bin_1', 'location': (1.0,  1.5, 90.0), 'fill': 0.0, 'is_targeted': False},
            2: {'name': 'Bin_2', 'location': (-1.0, 0.8, -90.0), 'fill': 0.0, 'is_targeted': False},
        }
        
        self.fill_threshold = 75.0 # Dispatch if fill >= 75%
        
        self.current_task_phase = "IDLE" # IDLE, DISPATCHED_WAITING_FOR_NAV_SUCCESS, AT_BIN_VERIFYING_STOP, AT_BIN_TIMING_STOP
        self.dispatched_bin_index = None
        self.dispatched_bin_target_pose_stamped = None # Store the PoseStamped sent to NavExecutor

        self.at_bin_stop_start_time = None
        self.proximity_check_radius = 0.15 # FR6: 0.1m, giving slight buffer
        self.required_stop_duration_sec = 10.0

        self.fr6_check_timer = None # Timer for FR6 logic after arrival

        self.decision_timer = self.create_timer(5.0, self.make_decision_cycle) # Check every 5 seconds
        self.get_logger().info("Decision Node Initialized. Bin data:")
        for i, data in self.bins_data.items():
            self.get_logger().info(f"  {data['name']} (Index {i}): Loc {data['location']}, Fill {data['fill']}%")


    def fill_level_callback(self, msg: Float32MultiArray):
        # self.get_logger().info(f"Received fill levels: {msg.data}")
        for i, level in enumerate(msg.data):
            if i in self.bins_data:
                self.bins_data[i]['fill'] = level
        # Optional: Trigger a decision cycle immediately after fill levels update
        # self.make_decision_cycle() 

    def robot_pose_callback(self, msg: PoseStamped):
        self.current_robot_pose = msg

    def navigation_status_callback(self, msg: String):
        nav_status = msg.data
        self.get_logger().info(f"Received Nav Executor Status: {nav_status}")

        if self.current_task_phase == "DISPATCHED_WAITING_FOR_NAV_SUCCESS":
            if nav_status == "SUCCEEDED_AT_POSE":
                self.get_logger().info(f"Navigation to bin index {self.dispatched_bin_index} succeeded. Starting FR6 checks.")
                self.current_task_phase = "AT_BIN_VERIFYING_STOP"
                # Start a periodic check for proximity and stop
                if self.fr6_check_timer is not None: self.fr6_check_timer.cancel()
                self.fr6_check_timer = self.create_timer(0.5, self.fr6_proximity_and_stop_check) # Check every 0.5s
            elif nav_status in ["FAILED_NAVIGATION", "ABORTED_NAVIGATION", "CANCELED_NAVIGATION", "REJECTED", "IDLE_SERVER_UNAVAILABLE"]:
                self.get_logger().warn(f"Navigation to bin index {self.dispatched_bin_index} failed or was problematic. Resetting task.")
                if self.dispatched_bin_index is not None:
                    self.bins_data[self.dispatched_bin_index]['is_targeted'] = False # Allow retry
                self.reset_task_state()
        
        # If NavExecutor becomes IDLE for any other reason while we thought it was busy on our task
        elif nav_status == "IDLE" and self.current_task_phase != "IDLE":
             self.get_logger().warn(f"Nav Executor became IDLE unexpectedly during phase {self.current_task_phase}. Resetting task.")
             if self.dispatched_bin_index is not None and self.dispatched_bin_index in self.bins_data:
                self.bins_data[self.dispatched_bin_index]['is_targeted'] = False
             self.reset_task_state()


    def make_decision_cycle(self):
        if self.current_task_phase != "IDLE":
            # self.get_logger().info(f"Decision cycle: Robot busy with task phase '{self.current_task_phase}'. Waiting...")
            return

        target_id_to_dispatch = None
        max_fill = -1.0

        for id_key, data in self.bins_data.items():
            if not data['is_targeted'] and data['fill'] >= self.fill_threshold:
                if data['fill'] > max_fill:
                    max_fill = data['fill']
                    target_id_to_dispatch = id_key
        
        if target_id_to_dispatch is not None:
            self.dispatched_bin_index = target_id_to_dispatch
            self.bins_data[self.dispatched_bin_index]['is_targeted'] = True
            
            x, y, yaw_deg = self.bins_data[self.dispatched_bin_index]['location']
            
            goal_pose_msg = PoseStamped()
            goal_pose_msg.header.stamp = self.get_clock().now().to_msg()
            goal_pose_msg.header.frame_id = 'map' # Crucial: goals must be in map frame
            goal_pose_msg.pose.position.x = float(x)
            goal_pose_msg.pose.position.y = float(y)
            goal_pose_msg.pose.position.z = 0.0 # Assuming 2D navigation

            yaw_rad = math.radians(float(yaw_deg))
            goal_pose_msg.pose.orientation.x = 0.0
            goal_pose_msg.pose.orientation.y = 0.0
            goal_pose_msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
            goal_pose_msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
            
            self.dispatched_bin_target_pose_stamped = goal_pose_msg # Store for proximity check

            self.goal_publisher.publish(goal_pose_msg)
            self.current_task_phase = "DISPATCHED_WAITING_FOR_NAV_SUCCESS"
            self.get_logger().info(f"Decision: Dispatched robot to {self.bins_data[self.dispatched_bin_index]['name']} (Index: {self.dispatched_bin_index})")
        # else:
            # self.get_logger().info("Decision cycle: No bins require servicing.")

    def fr6_proximity_and_stop_check(self):
        if self.current_task_phase != "AT_BIN_VERIFYING_STOP":
            if self.fr6_check_timer: self.fr6_check_timer.cancel() # Stop timer if state changed
            return

        if self.current_robot_pose is None or self.dispatched_bin_target_pose_stamped is None:
            self.get_logger().warn("FR6 Check: Missing robot pose or target pose.")
            return

        # 1. Check Proximity
        dist = math.sqrt(
            (self.current_robot_pose.pose.position.x - self.dispatched_bin_target_pose_stamped.pose.position.x)**2 +
            (self.current_robot_pose.pose.position.y - self.dispatched_bin_target_pose_stamped.pose.position.y)**2
        )

        if dist <= self.proximity_check_radius:
            # 2. Check if stopped (simple check: pose hasn't changed significantly)
            # This is a basic stop check. A more robust one would use /odom velocities.
            is_stopped_now = False
            if self.last_pose_for_stop_check and self.last_pose_time_for_stop_check:
                pos_diff = math.sqrt(
                    (self.current_robot_pose.pose.position.x - self.last_pose_for_stop_check.pose.position.x)**2 +
                    (self.current_robot_pose.pose.position.y - self.last_pose_for_stop_check.pose.position.y)**2
                )
                # For orientation, you'd compare quaternions or euler angles
                time_diff = (self.get_clock().now() - self.last_pose_time_for_stop_check).nanoseconds / 1e9
                
                if time_diff > 0 and pos_diff / time_diff < 0.01: # Velocity < 0.01 m/s
                    is_stopped_now = True
            
            self.last_pose_for_stop_check = self.current_robot_pose
            self.last_pose_time_for_stop_check = self.get_clock().now()

            if is_stopped_now:
                if self.time_entered_stopped_state is None:
                    self.time_entered_stopped_state = self.get_clock().now()
                    self.get_logger().info(f"FR6: Robot stopped within proximity of bin {self.dispatched_bin_index}. Starting 10s countdown.")
                
                time_stopped_so_far = (self.get_clock().now() - self.time_entered_stopped_state).nanoseconds / 1e9
                self.get_logger().info(f"FR6: Been stopped for {time_stopped_so_far:.1f}s / {self.required_stop_duration_sec}s")

                if time_stopped_so_far >= self.required_stop_duration_sec:
                    self.get_logger().info(f"FR6: Robot stopped at bin {self.dispatched_bin_index} for {self.required_stop_duration_sec}s. Commanding reset.")
                    if self.fr6_check_timer: self.fr6_check_timer.cancel() # Stop this check timer
                    
                    reset_cmd = String()
                    reset_cmd.data = str(self.dispatched_bin_index) # Send bin index as string for simplicity
                    self.reset_bin_publisher.publish(reset_cmd)
                    
                    self.bins_data[self.dispatched_bin_index]['is_targeted'] = False # Allow re-targeting in future
                    self.reset_task_state() # Resets phase to IDLE, clears dispatched_bin_index
                    self.get_logger().info(f"FR6: Bin {self.dispatched_bin_index} emptying process initiated.")
            else: # Still moving or just arrived
                self.time_entered_stopped_state = None # Reset stop timer if it moves
                self.get_logger().info(f"FR6: Robot at bin {self.dispatched_bin_index}, but still moving or just arrived. Verifying stop...")
        else:
            self.get_logger().warn(f"FR6: Robot is {dist:.2f}m away from bin {self.dispatched_bin_index} (threshold {self.proximity_check_radius}m). Aborting FR6 interaction.")
            if self.fr6_check_timer: self.fr6_check_timer.cancel()
            self.bins_data[self.dispatched_bin_index]['is_targeted'] = False # Allow retry
            self.reset_task_state()


    def reset_task_state(self):
        self.get_logger().info("Resetting task state to IDLE.")
        self.current_task_phase = "IDLE"
        self.dispatched_bin_index = None
        self.dispatched_bin_target_pose_stamped = None
        self.at_bin_stop_start_time = None
        self.last_pose_for_stop_check = None
        self.last_pose_time_for_stop_check = None
        self.time_entered_stopped_state = None
        if self.fr6_check_timer and not self.fr6_check_timer.canceled:
            self.fr6_check_timer.cancel()
        self.fr6_check_timer = None


def main(args=None):
    rclpy.init(args=args)
    decision_node = DecisionNode()
    try:
        rclpy.spin(decision_node)
    except KeyboardInterrupt:
        decision_node.get_logger().info("Decision node shutting down.")
    finally:
        if decision_node.fr6_check_timer is not None: # Ensure timer is cancelled on shutdown
            decision_node.fr6_check_timer.cancel()
        decision_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
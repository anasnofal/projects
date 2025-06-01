#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from std_msgs.msg import Float32MultiArray, String # For fill levels and reset command
from geometry_msgs.msg import PoseStamped, Point as PosePoint, Quaternion # For goals and pose checking
import math
# import time # Not strictly needed for main logic due to timers

class DecisionNode(Node):
    def __init__(self):
        super().__init__('decision_node')

        # Publisher to send navigation goals TO NavigationExecutorNode
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

        # Subscriber to get current robot pose for proximity/stop checks
        self.robot_pose_subscriber = self.create_subscription(
            PoseStamped, 
            '/amcl_pose', # Or your localized robot pose topic in map frame
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
        
        self.current_task_phase = "IDLE" # IDLE, DISPATCHED_WAITING_FOR_NAV_SUCCESS, AT_BIN_VERIFYING_STOP
        self.dispatched_bin_index = None
        self.dispatched_bin_target_pose_stamped = None # Store the PoseStamped sent for proximity check

        self.at_bin_stop_start_time = None # When the robot was confirmed stopped at the bin
        self.proximity_check_radius = 0.15 # meters (FR6 wants 0.1m, giving slight buffer for Nav2 goal tolerance)
        self.required_stop_duration_sec = 10.0
        self.min_duration_to_be_considered_stopped = 1.5 # seconds robot must be relatively still

        self.fr6_check_timer = None # Timer for FR6 logic after arrival (proximity and stop checks)
        self.last_pose_for_stop_check = None
        self.last_pose_time_for_stop_check = None
        self.time_entered_stopped_state = None


        self.decision_timer = self.create_timer(5.0, self.make_decision_cycle) # Check every 5 seconds
        self.get_logger().info("Decision Node Initialized. Bin data:")
        for i, data in self.bins_data.items(): # Using .items() to iterate through dict
            self.get_logger().info(f"  {data['name']} (Index {i}): Loc {data['location']}, Fill {data['fill']}%")


    def fill_level_callback(self, msg: Float32MultiArray):
        # self.get_logger().debug(f"Received fill levels: {msg.data}")
        for i, level in enumerate(msg.data):
            if i in self.bins_data:
                self.bins_data[i]['fill'] = level
        # Optional: Trigger a decision cycle immediately after fill levels update if not busy
        # if self.current_task_phase == "IDLE":
        #    self.make_decision_cycle()


    def robot_pose_callback(self, msg: PoseStamped):
        self.current_robot_pose = msg
        # If we are in AT_BIN_VERIFYING_STOP, this new pose helps fr6_proximity_and_stop_check
        # No direct action here, the timer callback will use self.current_robot_pose


    def navigation_status_callback(self, msg: String):
        nav_status = msg.data
        self.get_logger().info(f"Received Nav Executor Status: '{nav_status}' (Current DecisionNode Phase: '{self.current_task_phase}')")

        if self.current_task_phase == "DISPATCHED_WAITING_FOR_NAV_SUCCESS":
            if nav_status == "SUCCEEDED_AT_POSE":
                self.get_logger().info(f"Navigation to bin index {self.dispatched_bin_index} SUCCEEDED. Starting FR6 checks.")
                self.current_task_phase = "AT_BIN_VERIFYING_STOP"
                self.last_pose_for_stop_check = None # Reset for new arrival stop check
                self.time_entered_stopped_state = None # Reset
                if self.fr6_check_timer is not None and not self.fr6_check_timer.is_canceled():
                    self.fr6_check_timer.cancel()
                self.fr6_check_timer = self.create_timer(0.5, self.fr6_proximity_and_stop_check) # Check every 0.5s
            
            elif nav_status in ["FAILED_NAVIGATION", "ABORTED_NAVIGATION", "CANCELED_NAVIGATION", "REJECTED", "IDLE_SERVER_UNAVAILABLE"]:
                self.get_logger().warn(f"Navigation to bin index {self.dispatched_bin_index} FAILED/PROBLEM (Status: '{nav_status}'). Resetting task.")
                if self.dispatched_bin_index is not None and self.dispatched_bin_index in self.bins_data:
                    self.bins_data[self.dispatched_bin_index]['is_targeted'] = False
                self.reset_task_state()
            
            elif nav_status == "IDLE" and self.dispatched_bin_index is not None:
                # This means NavExecutor went IDLE after dispatch but before SUCCEEDED_AT_POSE
                self.get_logger().warn(f"Nav Executor went IDLE before navigation to bin {self.dispatched_bin_index} could succeed. Resetting task.")
                if self.dispatched_bin_index in self.bins_data:
                    self.bins_data[self.dispatched_bin_index]['is_targeted'] = False
                self.reset_task_state()

        elif self.current_task_phase in ["AT_BIN_VERIFYING_STOP"]:
            if nav_status == "IDLE":
                self.get_logger().info(f"Nav Executor is now IDLE. DecisionNode continues FR6 for bin {self.dispatched_bin_index}.")
            # If Nav Executor starts NAVIGATING to a new goal while we are in FR6, it's an issue,
            # but our make_decision_cycle should prevent sending new goals while current_task_phase isn't IDLE.

    def make_decision_cycle(self):
        if self.current_task_phase != "IDLE":
            # self.get_logger().info(f"Decision cycle: Robot busy with task phase '{self.current_task_phase}'. Waiting...")
            return

        target_id_to_dispatch = None
        max_fill = -1.0

        for id_key, data in self.bins_data.items(): # Use .items() for dictionaries
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
            goal_pose_msg.header.frame_id = 'map' 
            goal_pose_msg.pose.position.x = float(x)
            goal_pose_msg.pose.position.y = float(y)
            goal_pose_msg.pose.position.z = 0.0

            yaw_rad = math.radians(float(yaw_deg))
            goal_pose_msg.pose.orientation.x = 0.0 # Manual quaternion for yaw-only
            goal_pose_msg.pose.orientation.y = 0.0
            goal_pose_msg.pose.orientation.z = math.sin(yaw_rad / 2.0)
            goal_pose_msg.pose.orientation.w = math.cos(yaw_rad / 2.0)
            
            self.dispatched_bin_target_pose_stamped = goal_pose_msg # Store for proximity check

            self.goal_publisher.publish(goal_pose_msg)
            self.current_task_phase = "DISPATCHED_WAITING_FOR_NAV_SUCCESS"
            self.get_logger().info(f"Decision: Dispatched robot to {self.bins_data[self.dispatched_bin_index]['name']} (Index: {self.dispatched_bin_index})")
        # else:
            # self.get_logger().debug("Decision cycle: No bins meet dispatch criteria.")

    def fr6_proximity_and_stop_check(self):
        if self.current_task_phase != "AT_BIN_VERIFYING_STOP":
            # If state changed (e.g. task reset by other means), ensure timer is stopped.
            if self.fr6_check_timer and not self.fr6_check_timer.is_canceled():
                 self.fr6_check_timer.cancel()
            return

        if self.current_robot_pose is None or self.dispatched_bin_target_pose_stamped is None:
            self.get_logger().warn("FR6 Check: Missing robot pose or target pose for dispatched bin.")
            return

        # 1. Check Proximity
        dist = math.sqrt(
            (self.current_robot_pose.pose.position.x - self.dispatched_bin_target_pose_stamped.pose.position.x)**2 +
            (self.current_robot_pose.pose.position.y - self.dispatched_bin_target_pose_stamped.pose.position.y)**2
        )

        if dist <= self.proximity_check_radius:
            # 2. Check if stopped (robot pose hasn't changed significantly for a short duration)
            is_considered_stopped = False
            current_time = self.get_clock().now()
            if self.last_pose_for_stop_check and self.last_pose_time_for_stop_check:
                pos_diff = math.sqrt(
                    (self.current_robot_pose.pose.position.x - self.last_pose_for_stop_check.pose.position.x)**2 +
                    (self.current_robot_pose.pose.position.y - self.last_pose_for_stop_check.pose.position.y)**2
                )
                time_diff_secs = (current_time - self.last_pose_time_for_stop_check).nanoseconds / 1e9
                
                if time_diff_secs > 0.1: # Avoid division by zero and ensure some time has passed
                    velocity_estimate = pos_diff / time_diff_secs
                    if velocity_estimate < 0.01: # Threshold for being "stopped"
                        if self.time_entered_stopped_state is None: # First time we detect stop
                            self.time_entered_stopped_state = current_time
                            self.get_logger().info(f"FR6: Robot confirmed stopped within proximity of bin {self.dispatched_bin_index}. Starting {self.required_stop_duration_sec}s countdown.")
                        is_considered_stopped = True
                    else: # Robot is moving
                        self.time_entered_stopped_state = None # Reset if it moves
                        self.get_logger().info(f"FR6: Robot at bin {self.dispatched_bin_index}, but moving (est. vel: {velocity_estimate:.3f} m/s). Resetting stop timer.")
                else: # Not enough time passed for reliable velocity check yet
                     pass # Keep last_pose_for_stop_check as is, wait for more time_diff
            else: # First check or after movement
                self.get_logger().info(f"FR6: Robot at bin {self.dispatched_bin_index}. Initializing stop check variables.")


            self.last_pose_for_stop_check = self.current_robot_pose
            self.last_pose_time_for_stop_check = current_time

            if is_considered_stopped and self.time_entered_stopped_state is not None:
                time_stopped_so_far = (current_time - self.time_entered_stopped_state).nanoseconds / 1e9
                self.get_logger().info(f"FR6: Been stopped for {time_stopped_so_far:.1f}s / {self.required_stop_duration_sec}s")

                if time_stopped_so_far >= self.required_stop_duration_sec:
                    self.get_logger().info(f"FR6: Robot stopped at bin {self.dispatched_bin_index} for required duration. Commanding reset.")
                    if self.fr6_check_timer and not self.fr6_check_timer.is_canceled():
                         self.fr6_check_timer.cancel() 
                    self.fr6_check_timer = None
                    
                    reset_cmd = String()
                    # Assuming bin IDs used by BinSensorNode are "bin_
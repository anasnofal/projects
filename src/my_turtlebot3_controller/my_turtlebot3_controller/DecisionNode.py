#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration # For time calculations

from std_msgs.msg import Float32MultiArray, String
from geometry_msgs.msg import PoseStamped, Point as PosePoint, Quaternion
import math
# No tf_transformations needed here as we do manual quaternion calculation

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
            '/amcl_pose', # Localized robot pose in map frame
            self.robot_pose_callback,
            10)
        
        # Publisher to command BinSensorMockNode to reset fill level
        self.reset_bin_publisher = self.create_publisher(String, '/set_bin_level_zero', 10)

        # --- Configuration ---
        # Bin Data: bin_index: {'name': str, 'location': (x,y,yaw_deg), 'fill': float, 'is_targeted': bool}
        # !!! IMPORTANT: Populate with YOUR actual bin map coordinates and names !!!
        self.bins_data = {
            0: {'name': 'bin_0', 'location': (0.25, -0.4, 0.0),  'fill': 0.0, 'is_targeted': False},
            1: {'name': 'bin_1', 'location': (1.0,  1.5, 90.0), 'fill': 0.0, 'is_targeted': False},
            2: {'name': 'bin_2', 'location': (-1.0, 0.8, -90.0), 'fill': 0.0, 'is_targeted': False},
        }
        self.fill_threshold = 45.0 
        
        # --- State Variables ---
        self.current_robot_pose = None # Stores current PoseStamped from /amcl_pose
        self.current_task_phase = "IDLE" # IDLE, DISPATCHED_WAITING_FOR_NAV_SUCCESS, AT_BIN_VERIFYING_STOP, AT_BIN_TIMING_STOP
        self.dispatched_bin_index = None
        self.dispatched_bin_target_pose_stamped = None # Store the PoseStamped sent to NavExecutor

        # --- FR6 Specific State ---
        self.at_bin_stop_start_time = None # When the 10s stop at bin starts
        self.proximity_check_radius = 0.15 # meters (FR6 target is 0.1m)
        self.required_stop_duration_sec = 10.0
        
        self.fr6_check_timer = None # Timer for FR6 logic (proximity and stop checks)
        self.last_pose_for_stop_check = None # For checking if robot is stationary
        self.last_pose_time_for_stop_check = None # Timestamp for last_pose_for_stop_check
        self.time_entered_stopped_state = None # When robot is confirmed stopped within proximity

        # --- Timers ---
        self.decision_timer = self.create_timer(5.0, self.make_decision_cycle) # Main decision loop

        self.get_logger().info("Decision Node Initialized. Bin data:")
        for i, data in self.bins_data.items():
            self.get_logger().info(f"  {data['name']} (Index {i}): Loc {data['location']}, Fill {data['fill']}%")

    def fill_level_callback(self, msg: Float32MultiArray):
        # self.get_logger().debug(f"Received fill levels: {msg.data}") # DEBUG to avoid spam
        for i, level in enumerate(msg.data):
            if i in self.bins_data:
                self.bins_data[i]['fill'] = level
        # Not triggering make_decision_cycle here; let the timer handle it to avoid rapid re-evaluation.

    def robot_pose_callback(self, msg: PoseStamped):
        self.current_robot_pose = msg
        # self.get_logger().debug(f"DecisionNode received robot_pose: x={msg.pose.position.x:.2f}") # DEBUG

    def navigation_status_callback(self, msg: String):
        nav_status = msg.data
        self.get_logger().info(f"Received Nav Executor Status: '{nav_status}' (Current DecisionNode Phase: '{self.current_task_phase}')")

        if self.current_task_phase == "DISPATCHED_WAITING_FOR_NAV_SUCCESS":
            if nav_status == "SUCCEEDED_AT_POSE":
                self.get_logger().info(f"Navigation to bin index {self.dispatched_bin_index} SUCCEEDED (according to NavExecutor). Starting FR6 checks.")
                self.current_task_phase = "AT_BIN_VERIFYING_STOP"
                self.last_pose_for_stop_check = None # Reset for new arrival sequence
                self.time_entered_stopped_state = None # Reset
                if self.fr6_check_timer is not None and not self.fr6_check_timer.is_canceled():
                    self.fr6_check_timer.cancel()
                self.fr6_check_timer = self.create_timer(0.5, self.fr6_proximity_and_stop_check) # Check every 0.5s
            
            elif nav_status in ["FAILED_NAVIGATION", "ABORTED_NAVIGATION", "CANCELED_NAVIGATION", "REJECTED", "IDLE_SERVER_UNAVAILABLE", "IDLE"]:
                self.get_logger().warn(f"Navigation to bin index {self.dispatched_bin_index} failed or was problematic (NavExecutor Status: '{nav_status}'). Resetting task.")
                if self.dispatched_bin_index is not None and self.dispatched_bin_index in self.bins_data:
                    self.bins_data[self.dispatched_bin_index]['is_targeted'] = False
                self.reset_task_state()
        
        elif self.current_task_phase in ["AT_BIN_VERIFYING_STOP", "AT_BIN_TIMING_STOP"]:
            if nav_status == "IDLE":
                self.get_logger().info(f"Nav Executor is now IDLE. DecisionNode continues FR6 for bin {self.dispatched_bin_index}.")
            elif nav_status not in ["NAVIGATING", "SUCCEEDED_AT_POSE"]: # Should not get these specific ones again if already in FR6
                self.get_logger().warn(f"Nav Executor sent unexpected status '{nav_status}' while DecisionNode is in FR6 phase for bin {self.dispatched_bin_index}.")

    def make_decision_cycle(self):
        # self.get_logger().debug(f"Make_decision_cycle. Current phase: {self.current_task_phase}")
        if self.current_task_phase != "IDLE":
            return # Robot/Decision process is busy with a task

        target_id_to_dispatch = None
        max_fill = -1.0

        for id_key, data in self.bins_data.items():
            if not data['is_targeted'] and data['fill'] >= self.fill_threshold:
                if data['fill'] > max_fill:
                    max_fill = data['fill']
                    target_id_to_dispatch = id_key
        
        if target_id_to_dispatch is not None:
            self.dispatched_bin_index = target_id_to_dispatch
            self.bins_data[self.dispatched_bin_index]['is_targeted'] = True # Mark as targeted
            
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
            
            self.dispatched_bin_target_pose_stamped = goal_pose_msg # Store for proximity check

            self.goal_publisher.publish(goal_pose_msg)
            self.current_task_phase = "DISPATCHED_WAITING_FOR_NAV_SUCCESS"
            self.get_logger().info(f"Decision: Dispatched robot to {self.bins_data[self.dispatched_bin_index]['name']} (Index: {self.dispatched_bin_index})")
        # else:
            # self.get_logger().debug("Decision cycle: No bins meet dispatch criteria or robot busy.")


    def fr6_proximity_and_stop_check(self):
        # This timer callback handles the FR6 logic after Nav2 reports success
        current_time_fr6_check = self.get_clock().now()
        self.get_logger().debug(f"FR6_TIMER_TICK ({current_time_fr6_check.nanoseconds/1e9:.3f}): Current phase '{self.current_task_phase}'")

        if self.current_task_phase not in ["AT_BIN_VERIFYING_STOP", "AT_BIN_TIMING_STOP"]: # Added AT_BIN_TIMING_STOP
            self.get_logger().info(f"FR6 Check: Not in an FR6 active phase (current: {self.current_task_phase}). Cancelling timer if active.")
            if self.fr6_check_timer and not self.fr6_check_timer.is_canceled():
                 self.fr6_check_timer.cancel()
            self.fr6_check_timer = None
            # It's possible reset_task_state() was already called, if not, call it to be safe
            if self.current_task_phase != "IDLE": # Avoid calling if already reset
                self.reset_task_state()
            return

        if self.current_robot_pose is None:
            self.get_logger().warn("FR6 Check: Robot pose not yet available from /amcl_pose in this tick. Waiting for update...")
            return 

        if self.dispatched_bin_target_pose_stamped is None:
            self.get_logger().error("FR6 Check: Dispatched bin target pose is None! Logic error. Resetting task.")
            self.reset_task_state() 
            return
        
        # --- Proximity Check ---
        dist = math.sqrt(
            (self.current_robot_pose.pose.position.x - self.dispatched_bin_target_pose_stamped.pose.position.x)**2 +
            (self.current_robot_pose.pose.position.y - self.dispatched_bin_target_pose_stamped.pose.position.y)**2
        )
        self.get_logger().debug(f"FR6: Current distance to target bin '{self.dispatched_bin_index}': {dist:.2f}m")

        if dist <= self.proximity_check_radius:
            # --- Stop Verification & Timing ---
            is_stopped_criteria_met = False # Has been stopped for required confirmation duration
            current_time = self.get_clock().now()

            if self.last_pose_for_stop_check and self.last_pose_time_for_stop_check:
                pos_diff = math.sqrt(
                    (self.current_robot_pose.pose.position.x - self.last_pose_for_stop_check.pose.position.x)**2 +
                    (self.current_robot_pose.pose.position.y - self.last_pose_for_stop_check.pose.position.y)**2
                )
                # TODO: Add orientation change check for a more robust "stopped"
                time_diff_dt = current_time - self.last_pose_time_for_stop_check
                time_diff_sec = time_diff_dt.nanoseconds / 1e9
                
                # Considered "stopped" if very little movement over the check interval
                if time_diff_sec > (self.velocity_check_interval * 0.5) and (pos_diff / time_diff_sec) < 0.01: # Effectively velocity < 0.01 m/s
                    if self.time_entered_stopped_state is None: # First time we detect it's stopped
                        self.time_entered_stopped_state = current_time
                        self.get_logger().info(f"FR6: Robot appears stopped within proximity of bin {self.dispatched_bin_index}. Starting {self.required_stop_duration_sec}s countdown.")
                        self.current_task_phase = "AT_BIN_TIMING_STOP" # Transition to timing phase
                    
                    time_stopped_so_far_dt = current_time - self.time_entered_stopped_state
                    time_stopped_so_far_sec = time_stopped_so_far_dt.nanoseconds / 1e9
                    self.get_logger().info(f"FR6: Been confirmed stopped for {time_stopped_so_far_sec:.1f}s / {self.required_stop_duration_sec}s")

                    if time_stopped_so_far_sec >= self.required_stop_duration_sec:
                        is_stopped_criteria_met = True
                else: # Robot is moving
                    if self.time_entered_stopped_state is not None:
                         self.get_logger().info(f"FR6: Robot moved while timing stop for bin {self.dispatched_bin_index}. Resetting stop timer.")
                    self.time_entered_stopped_state = None # Reset stop timer if it moves
                    self.current_task_phase = "AT_BIN_VERIFYING_STOP" # Go back to verifying stop
                    
            self.last_pose_for_stop_check = self.current_robot_pose
            self.last_pose_time_for_stop_check = current_time

            if is_stopped_criteria_met: # Met proximity, stopped, and waited 10s
                self.get_logger().info(f"FR6: Robot confirmed stopped at bin {self.dispatched_bin_index} for {self.required_stop_duration_sec}s. Commanding reset.")
                if self.fr6_check_timer and not self.fr6_check_timer.is_canceled(): 
                    self.fr6_check_timer.cancel()
                self.fr6_check_timer = None 
                
                reset_cmd = String()
                bin_id_str = self.bins_data[self.dispatched_bin_index]['name'] # Use name or construct "bin_0"
                reset_cmd.data = bin_id_str 
                self.reset_bin_publisher.publish(reset_cmd)
                
                self.bins_data[self.dispatched_bin_index]['is_targeted'] = False 
                self.reset_task_state() 
                self.get_logger().info(f"FR6: Bin {bin_id_str} emptying process initiated. Task complete.")
        else: # Not within proximity
            # If we were timing a stop but robot moved out of proximity
            if self.current_task_phase == "AT_BIN_TIMING_STOP":
                self.get_logger().warn(f"FR6: Robot moved out of proximity for bin {self.dispatched_bin_index} while timing stop. Resetting task.")
                if self.fr6_check_timer and not self.fr6_check_timer.is_canceled():
                    self.fr6_check_timer.cancel()
                self.fr6_check_timer = None
                if self.dispatched_bin_index is not None and self.dispatched_bin_index in self.bins_data:
                     self.bins_data[self.dispatched_bin_index]['is_targeted'] = False
                self.reset_task_state()
            else:
                # Still in AT_BIN_VERIFYING_STOP but not close enough yet. Keep checking.
                self.get_logger().debug(f"FR6: Robot {dist:.2f}m away from bin {self.dispatched_bin_index}. Waiting to get closer/confirm stop.")
                self.time_entered_stopped_state = None # Not stopped in proximity yet

    def reset_task_state(self):
        self.get_logger().info("Resetting DecisionNode task state to IDLE.")
        self.current_task_phase = "IDLE"
        # Do not reset 'is_targeted' here unconditionally. Only if the task truly failed before FR6 completion.
        # If FR6 completed successfully, 'is_targeted' was already set to False.
        # If FR6 failed (e.g. moved away), it's also set to False in fr6_proximity_and_stop_check.
        
        self.dispatched_bin_index = None
        self.dispatched_bin_target_pose_stamped = None
        self.at_bin_stop_start_time = None # This was used for the 10s timer if فر6_check_timer was one-shot
        self.last_pose_for_stop_check = None
        self.last_pose_time_for_stop_check = None
        self.time_entered_stopped_state = None # Reset this crucial state

        if self.fr6_check_timer and not self.fr6_check_timer.is_canceled():
            self.fr6_check_timer.cancel()
        self.fr6_check_timer = None


def main(args=None):
    rclpy.init(args=args)
    decision_node = DecisionNode()
    try:
        rclpy.spin(decision_node)
    except KeyboardInterrupt:
        decision_node.get_logger().info("Decision node shutting down due to KeyboardInterrupt.")
    finally:
        decision_node.get_logger().info("Cleaning up DecisionNode resources...")
        if hasattr(decision_node, 'fr6_check_timer') and decision_node.fr6_check_timer is not None and not decision_node.fr6_check_timer.is_canceled():
            decision_node.get_logger().info("Cancelling active fr6_check_timer.")
            decision_node.fr6_check_timer.cancel()
        decision_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
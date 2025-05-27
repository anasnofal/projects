#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult # For Nav2
from geometry_msgs.msg import PoseStamped
from tf_transformations import quaternion_from_euler # For converting yaw to quaternion
import math
import time

class NavigatorCommanderNode(Node):
    def __init__(self):
        super().__init__('navigator_commander_node')
        self.navigator = BasicNavigator()
        self.get_logger().info("Navigator Commander Node Initialized.")
        
        # It's good practice to make sure Nav2 is up and ready.
        # The navigator can handle bringing up Nav2 servers if they are not active
        # For a simple goal sending, we assume Nav2 is already launched and active.
        # You can add navigator.waitUntilNav2Active() if needed,
        # or navigator.lifecycleStartup() if you want this node to manage Nav2's lifecycle.
        # For this example, we'll assume Nav2 is fully up from another launch file.

    def go_to_pose(self, x, y, yaw_degrees):
        self.get_logger().info("Setting goal pose...")
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.navigator.get_clock().now().to_msg() # Use navigator's clock

        goal_pose.pose.position.x = float(x)
        goal_pose.pose.position.y = float(y)
        goal_pose.pose.position.z = 0.0

        yaw_radians = math.radians(float(yaw_degrees))
        q = quaternion_from_euler(0, 0, yaw_radians)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]

        self.get_logger().info(f"Attempting to navigate to: X={x}, Y={y}, Yaw={yaw_degrees}°")
        
        # Go to the pose
        self.navigator.goToPose(goal_pose)

        # Wait until the task is complete
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            if feedback: # feedback can be None if no new feedback is available
                self.get_logger().info(
                    f"Distance remaining: {feedback.distance_remaining:.2f} m, "
                    f"Time elapsed: {feedback.navigation_time.sec}s"
                )
            # Add a small delay to avoid busy-waiting, rclpy.spin_once is not needed here
            # as BasicNavigator handles its own spinning for service/action calls.
            time.sleep(0.1)


        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
            return True
        elif result == TaskResult.CANCELED:
            self.get_logger().warn('Goal was canceled!')
            return False
        elif result == TaskResult.FAILED:
            self.get_logger().error('Goal failed!')
            return False
        else:
            self.get_logger().info(f'Goal has an invalid return status: {result}')
            return False

    def set_initial_pose(self, x, y, yaw_degrees):
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.navigator.get_clock().now().to_msg()
        initial_pose.pose.position.x = float(x)
        initial_pose.pose.position.y = float(y)
        yaw_radians = math.radians(float(yaw_degrees))
        q = quaternion_from_euler(0, 0, yaw_radians)
        initial_pose.pose.orientation.x = q[0]
        initial_pose.pose.orientation.y = q[1]
        initial_pose.pose.orientation.z = q[2]
        initial_pose.pose.orientation.w = q[3]
        
        self.navigator.setInitialPose(initial_pose)
        self.get_logger().info("Initial pose set.")
        # You might want to wait for AMCL to fully converge after setting initial pose
        self.navigator.waitUntilNav2Active() # Good check after setting pose
        self.get_logger().info("Waiting for AMCL to stabilize after initial pose...")
        time.sleep(5) # Give AMCL some time

def main(args=None):
    rclpy.init(args=args)
    navigator_node = NavigatorCommanderNode()

    # --- IMPORTANT: SET INITIAL POSE IF ROBOT ISN'T LOCALIZED ---
    # If your robot is not yet localized by AMCL, or if you want to ensure it starts from a known pose.
    # Replace these with your robot's actual starting pose in the map.
    # If AMCL is already running and localized, you might be able to skip this.
    # For simulation, often starts at (0,0,0) if map origin is same as world origin.
    initial_x = 0.0
    initial_y = 0.0
    initial_yaw_degrees = 0.0
    # navigator_node.set_initial_pose(initial_x, initial_y, initial_yaw_degrees)
    # -----------------------------------------------------------------

    # --- DEFINE YOUR TARGET COORDINATES AND YAW (in degrees) HERE ---
    target_x = 1.0
    target_y = -0.5
    target_yaw_degrees = 90.0
    # --------------------------------------------------------------

    try:
        # Ensure Nav2 is active. If you don't want this node to manage lifecycle,
        # make sure Nav2 is fully launched and active before running this script.
        # For simplicity, we'll assume it's active. If you want to be robust:
        # navigator_node.navigator.waitUntilNav2Active()

        success = navigator_node.go_to_pose(target_x, target_y, target_yaw_degrees)
        if success:
            navigator_node.get_logger().info("Navigation task finished successfully.")
        else:
            navigator_node.get_logger().info("Navigation task did not succeed.")

    except KeyboardInterrupt:
        navigator_node.get_logger().info("Keyboard interrupt, shutting down tasks.")
        # navigator_node.navigator.cancelTask() # Example of cancelling
    except Exception as e:
        navigator_node.get_logger().error(f"An error occurred: {e}")
    finally:
        # BasicNavigator doesn't require explicit destruction like an action client in the node
        # but shutting down rclpy and destroying the node is good.
        # navigator_node.navigator.lifecycleShutdown() # If this node was managing lifecycle
        navigator_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
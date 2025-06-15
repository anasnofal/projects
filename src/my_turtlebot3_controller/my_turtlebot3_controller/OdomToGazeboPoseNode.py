#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from gazebo_msgs.srv import SetEntityState
from geometry_msgs.msg import Pose

class OdomToGazeboPoseNode(Node):
    """
    This node subscribes to a robot's odometry topic and uses Gazebo's
    /set_entity_state service to update the pose of a corresponding model
    in the simulation. This creates a high-fidelity  digital twin
    that mirrors the measured movement of a real robot.
    """
    def __init__(self):
        super().__init__('odom_to_gazebo_pose_node')
        
        self.declare_parameter('gazebo_model_name', 'turtlebot3_burger') 
        self.gazebo_model_name = self.get_parameter('gazebo_model_name').get_parameter_value().string_value

        self.declare_parameter('odom_topic', '/odom')
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value


        self.odom_subscription = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10) 

        # Client for the Gazebo service to set model/entity state
        self.set_state_client = self.create_client(SetEntityState, '/set_entity_state')
        while not self.set_state_client.wait_for_service(timeout_sec=2.0):
            self.get_logger().info("Gazebo's /set_entity_state service not available, waiting...")

        self.get_logger().info(f"OdomToGazeboPoseNode Initialized.")
        self.get_logger().info(f"Mirroring odometry from '{self.odom_topic}' to Gazebo model '{self.gazebo_model_name}'.")

    def odom_callback(self, msg: Odometry):
        """
        This callback is triggered for every new Odometry message.
        It calls the Gazebo service to set the simulated model's pose.
        """
        
        request = SetEntityState.Request()
        
        request.state.name = self.gazebo_model_name
        request.state.pose = msg.pose.pose 
        
        # request.state.twist = msg.twist.twist
        
        request.state.reference_frame = 'odom' 

        future = self.set_state_client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    node = OdomToGazeboPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("OdomToGazeboPoseNode stopped cleanly")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()

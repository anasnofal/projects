#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelayNode(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay_node')
        
        # Subscriber to the original /cmd_vel topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',         # Topic to subscribe to
            self.listener_callback,
            10)                 # QoS profile depth
        self.subscription  # prevent unused variable warning

        # Publisher to the new /my_tb3/cmd_vel topic
        self.publisher = self.create_publisher(
            Twist,
            '/my_tb3/cmd_vel',  # Topic to publish to
            10)                 # QoS profile depth

        self.get_logger().info('CmdVelRelayNode_PANA_DSP_PROJECT_FR1_Initialise started. Relaying /cmd_vel -> /my_tb3/cmd_vel')

    def listener_callback(self, msg):
        # This function is called every time a message is received on /cmd_vel
        # self.get_logger().info(f'Relaying Twist: Linear X: {msg.linear.x}, Angular Z: {msg.angular.z}') # Uncomment for verbose logging
        self.publisher.publish(msg) # Simply republish the received message

def main(args=None):
    rclpy.init(args=args)

    cmd_vel_relay_node = CmdVelRelayNode()

    try:
        rclpy.spin(cmd_vel_relay_node)
    except KeyboardInterrupt:
        cmd_vel_relay_node.get_logger().info('CmdVelRelayNode stopped cleanly')
    finally:
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node object)
        cmd_vel_relay_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
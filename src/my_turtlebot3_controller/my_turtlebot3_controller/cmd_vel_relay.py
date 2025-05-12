# cmd_vel_relay.py

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.get_logger().info('Starting cmd_vel relay node...')

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',  # here the  Gazebo cmd_vel
            self.cmd_vel_callback,
            10
        )

        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',  # Same topic name, here it should be changed to real robot cmd_vel
            10
        )

    def cmd_vel_callback(self, msg):
        self.get_logger().debug(f'Relaying cmd_vel: {msg}')
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

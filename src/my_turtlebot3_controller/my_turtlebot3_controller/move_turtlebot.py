# my_turtlebot3_controller/move_turtlebot.py
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        self.sub = self.create_subscription(Twist, '/gazebo/cmd_vel', self.callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def callback(self, msg):
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = CmdVelRelay()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

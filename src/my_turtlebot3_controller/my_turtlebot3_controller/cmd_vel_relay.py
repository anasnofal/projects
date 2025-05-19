import rclpy
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
import time

class GoToBinsNode(Node):

    def __init__(self):
        super().__init__('go_to_bins')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.targets = [
            # Replace with coordinates near your trash bins
            {'x': 2.0, 'y': 1.5, 'theta': 0.0},
            {'x': 4.0, 'y': 3.2, 'theta': 1.57}
        ]
        self.send_goals()

    def send_goals(self):
        for target in self.targets:
            goal_msg = NavigateToPose.Goal()
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.pose.position.x = target['x']
            pose.pose.position.y = target['y']
            pose.pose.orientation.z = 1.0  # Simplified for straight orientation
            goal_msg.pose = pose

            self.client.wait_for_server()
            self.get_logger().info(f'Sending goal: {target}')
            self.client.send_goal_async(goal_msg)
            time.sleep(10)  # Give time to navigate

def main(args=None):
    rclpy.init(args=args)
    node = GoToBinsNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

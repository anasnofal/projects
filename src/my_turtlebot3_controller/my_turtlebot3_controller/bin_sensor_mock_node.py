#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32MultiArray, MultiArrayDimension, MultiArrayLayout
import random
import time # For initial sleep, though timers are used for periodic tasks

class BinSensorMockNode(Node):
    def __init__(self):
        super().__init__('bin_sensor_mock_node')

        # --- Configuration Parameters ---
        self.declare_parameter('num_bins', 3)
        self.declare_parameter('initial_fill_levels', [10.0, 45.0, 75.0]) # Fill %
        self.declare_parameter('bin_ids_prefix', 'bin_') # e.g. bin_0, bin_1
        self.declare_parameter('max_capacity', 100.0)
        self.declare_parameter('fill_increase_min_per_update', 0.1) # Min % increase
        self.declare_parameter('fill_increase_max_per_update', 1.0) # Max % increase
        
        self.declare_parameter('update_fill_levels_interval_sec', 1.0) # How often to "add trash"
        self.declare_parameter('publish_states_interval_sec', 2.0) # How often to publish all bin states

        # Get parameters
        self.num_bins = self.get_parameter('num_bins').get_parameter_value().integer_value
        initial_fills_param = self.get_parameter('initial_fill_levels').get_parameter_value().double_array_value
        self.bin_ids_prefix = self.get_parameter('bin_ids_prefix').get_parameter_value().string_value
        self.max_capacity = self.get_parameter('max_capacity').get_parameter_value().double_value
        self.fill_increase_min = self.get_parameter('fill_increase_min_per_update').get_parameter_value().double_value
        self.fill_increase_max = self.get_parameter('fill_increase_max_per_update').get_parameter_value().double_value
        update_interval = self.get_parameter('update_fill_levels_interval_sec').get_parameter_value().double_value
        publish_interval = self.get_parameter('publish_states_interval_sec').get_parameter_value().double_value

        # --- Internal State ---
        # self.bins will be a list of dictionaries. The index in this list
        # will correspond to the index in the Float32MultiArray.data field.
        self.bins = []
        for i in range(self.num_bins):
            initial_fill = initial_fills_param[i] if i < len(initial_fills_param) else 0.0
            self.bins.append({
                'id': f'{self.bin_ids_prefix}{i}', # String ID, e.g., "bin_0", "bin_1"
                'fill_level': float(initial_fill), # Numeric fill level
            })

        # --- ROS Communications ---
        # Publisher for all bin states using Float32MultiArray
        self.bin_states_publisher = self.create_publisher(
            Float32MultiArray, 
            '/bin_fill_levels', 
            10)
        
        # Subscriber to reset a specific bin's fill level
        self.reset_bin_subscriber = self.create_subscription(
            String, # Expects message.data to be the string bin_id (e.g., "bin_0")
            '/set_bin_level_zero',
            self.reset_bin_callback,
            10)

        # Timer to periodically add trash
        self.update_timer = self.create_timer(update_interval, self.update_fill_levels_tick)
        
        # Timer to periodically publish all bin states
        self.publish_timer = self.create_timer(publish_interval, self.publish_all_bin_states_tick)

        self.get_logger().info(f"Bin Sensor Mock Node Initialized with {self.num_bins} bins.")
        self.log_bin_states("Initial")
        self.publish_all_bin_states_tick() # Publish initial state immediately

    def update_fill_levels_tick(self):
        """Periodically adds a random amount of trash to each bin."""
        changed = False
        for bin_data in self.bins: # Iterates through the list of bin dictionaries
            if bin_data['fill_level'] < self.max_capacity:
                increase = random.uniform(self.fill_increase_min, self.fill_increase_max)
                bin_data['fill_level'] += increase
                if bin_data['fill_level'] > self.max_capacity:
                    bin_data['fill_level'] = self.max_capacity
                changed = True
        
        if changed:
            # self.log_bin_states("Updated internally") # Can be verbose
            pass # Actual publishing is handled by publish_all_bin_states_tick

    def publish_all_bin_states_tick(self):
        """Constructs and publishes the Float32MultiArray message."""
        fill_levels_data = [bin_data['fill_level'] for bin_data in self.bins]

        msg = Float32MultiArray()
        
        # Optional: Setup layout for better introspection, though not strictly needed if subscriber knows order
        msg.layout = MultiArrayLayout()
        msg.layout.dim = [MultiArrayDimension()]
        msg.layout.dim[0].label = "bins"
        msg.layout.dim[0].size = self.num_bins
        msg.layout.dim[0].stride = self.num_bins # For a 1D array, stride can be size
        msg.layout.data_offset = 0

        msg.data = fill_levels_data
        
        self.bin_states_publisher.publish(msg)
        log_msg_data = ", ".join([f"{self.bins[i]['id']}: {fill_levels_data[i]:.1f}%" for i in range(len(fill_levels_data))])
        self.get_logger().debug(f"Published /bin_fill_levels: [{log_msg_data}]")


    def reset_bin_callback(self, msg: String):
        bin_id_to_reset = msg.data # This is the string ID like "bin_0"
        found = False
        for i, bin_data in enumerate(self.bins):
            if bin_data['id'] == bin_id_to_reset:
                if bin_data['fill_level'] > 0.0:
                    self.get_logger().info(f"Bin '{bin_id_to_reset}' (index {i}) current fill: {bin_data['fill_level']:.1f}%. Resetting to 0%.")
                    bin_data['fill_level'] = 0.0
                else:
                    self.get_logger().info(f"Bin '{bin_id_to_reset}' (index {i}) already at 0% or being reset.")
                found = True
                break
        
        if not found:
            self.get_logger().warn(f"Received reset command for unknown bin ID: '{bin_id_to_reset}'")
        
        self.publish_all_bin_states_tick() # Publish updated states immediately

    def log_bin_states(self, prefix="Current"):
        states_str = ", ".join([f"{b['id']}: {b['fill_level']:.1f}%" for b in self.bins])
        self.get_logger().info(f"{prefix} bin states: [{states_str}]")


def main(args=None):
    rclpy.init(args=args)
    bin_sensor_node = BinSensorMockNode()
    try:
        rclpy.spin(bin_sensor_node)
    except KeyboardInterrupt:
        bin_sensor_node.get_logger().info("Bin Sensor Mock Node shutting down.")
    finally:
        bin_sensor_node.destroy_node()
        if rclpy.ok(): # Check if shutdown was already called
            rclpy.shutdown()

if __name__ == '__main__':
    main()
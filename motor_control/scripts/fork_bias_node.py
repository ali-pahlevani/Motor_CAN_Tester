#!/usr/bin/env python3
import os
import yaml
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class ForkBiasNode(Node):
    def __init__(self):
        super().__init__('fork_bias_node')
        
        # Get the YAML file path from the motor_control package
        try:
            package_share_dir = get_package_share_directory('motor_control')
            self.yaml_path = os.path.join(package_share_dir, 'encoder', 'rel_to_abs_encode.yaml')
        except Exception as e:
            self.get_logger().error(f'Failed to find motor_control package: {e}')
            rclpy.shutdown()
            return
        
        # Load initial height_bias from YAML
        self.height_bias = 0.0
        self.load_bias()
        
        # Current raw position storage
        self.current_raw_position = 0.0
        
        # Subscriber to /joint_states
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        # Publisher to /forks_abs_height (Float64, only position)
        self.publisher = self.create_publisher(Float64, '/forks_abs_height', 10)
        
        # Timer to periodically save the current absolute height as bias (every 5 seconds)
        self.timer = self.create_timer(5.0, self.save_bias)

    def load_bias(self):
        try:
            with open(self.yaml_path, 'r') as file:
                data = yaml.safe_load(file)
                # Handle if it's a list like [{'height_bias': value}] or dict {'height_bias': value}
                if isinstance(data, list) and data:
                    self.height_bias = data[0].get('height_bias', 0.0)
                elif isinstance(data, dict):
                    self.height_bias = data.get('height_bias', 0.0)
                else:
                    raise ValueError('Invalid YAML format')
            self.get_logger().info(f'Loaded height_bias: {self.height_bias}')
        except Exception as e:
            self.get_logger().warn(f'Failed to load YAML: {e}. Using default 0.0')
            self.height_bias = 0.0

    def save_bias(self):
        # Calculate current absolute height
        current_abs = self.current_raw_position + self.height_bias
        data = {'height_bias': current_abs}
        try:
            with open(self.yaml_path, 'w') as file:
                yaml.dump(data, file)
            self.get_logger().info(f'Saved height_bias (current abs height): {current_abs}')
        except Exception as e:
            self.get_logger().error(f'Failed to save YAML: {e}')

    def joint_state_callback(self, msg):
        try:
            # Find the index of 'forks_joint'
            index = msg.name.index('forks_joint')
            self.current_raw_position = msg.position[index]
            
            # Calculate absolute height
            abs_height = self.current_raw_position + self.height_bias
            
            # Publish to /forks_abs_height
            abs_msg = Float64()
            abs_msg.data = abs_height
            self.publisher.publish(abs_msg)
        except ValueError:
            self.get_logger().warn('forks_joint not found in /joint_states')

def main(args=None):
    rclpy.init(args=args)
    node = ForkBiasNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Save bias on shutdown
        node.save_bias()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
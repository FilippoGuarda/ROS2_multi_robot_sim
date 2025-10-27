#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String
import json

class FleetPositionAggregator(Node):
    def __init__(self):
        super().__init__('fleet_position_aggregator')
        
        self.declare_parameter('robot_namespaces', ['robot1', 'robot2'])
        self.declare_parameter('use_sim_time', True)
        
        self.robot_names = self.get_parameter('robot_namespaces').get_parameter_value().string_array_value
        
        self.positions = {}
        

        self.pose_subscribers = {}
        for name in self.robot_names:
            self.pose_subscribers[name] = self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{name}/amcl_pose',
                lambda msg, n=name: self.pose_callback(msg, n),
                10
            )
            self.get_logger().info(f'Subscribed to /{name}/amcl_pose')
        
        self.publisher = self.create_publisher(String, '/fleet/robot_positions', 10)
        self.timer = self.create_timer(1.0, self.publish_positions)
        
        self.get_logger().info('Fleet Position Aggregator initialized')
    
    def pose_callback(self, msg, robot_name):
        self.positions[robot_name] = {
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            }
        }
    
    def publish_positions(self):
        if self.positions:
            positions_json = json.dumps(self.positions)
            self.publisher.publish(String(data=positions_json))
            self.get_logger().debug(f'Published positions: {positions_json}')

def main(args=None):
    rclpy.init(args=args)
    node = FleetPositionAggregator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
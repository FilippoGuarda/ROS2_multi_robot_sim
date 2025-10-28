#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from smrta_messages.msg import RobotPosition, FleetRobotPositions
import math
import json

class FleetPositionAggregator(Node):
    def __init__(self):
        super().__init__('fleet_position_aggregator')
        
        self.declare_parameter('robot_namespaces', ['robot1', 'robot2'])
        self.declare_parameter('graph_nodes_file', '')  # NEW: graph node positions file
        
        self.robot_names = self.get_parameter('robot_namespaces').get_parameter_value().string_array_value
        graph_nodes_file = self.get_parameter('graph_nodes_file').get_parameter_value().string_value
        
        self.positions = {}
        self.pose_subscribers = {}
        self.graph_nodes = {}
        
        # Load graph node positions (mapping from node_id to (x, y) coordinates)
        if graph_nodes_file:
            try:
                with open(graph_nodes_file, 'r') as f:
                    self.graph_nodes = json.load(f)
                self.get_logger().info(f'Loaded {len(self.graph_nodes)} graph nodes from {graph_nodes_file}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load graph nodes file: {e}')
        
        for name in self.robot_names:
            self.pose_subscribers[name] = self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{name}/amcl_pose',
                lambda msg, n=name: self.pose_callback(msg, n),
                10
            )
            self.get_logger().info(f'Subscribed to /{name}/amcl_pose')
        
        # Publish using custom message
        self.publisher = self.create_publisher(FleetRobotPositions, '/fleet/robot_positions', 10)
        self.timer = self.create_timer(1.0, self.publish_positions)
        
        self.get_logger().info('Fleet Position Aggregator initialized')
    
    def find_closest_graph_node(self, x, y):
        """Find the closest graph node to given (x, y) coordinates"""
        if not self.graph_nodes:
            return 0  # Default node if no graph loaded
        
        min_dist = float('inf')
        closest_node = 0
        
        for node_id_str, node_pos in self.graph_nodes.items():
            node_id = int(node_id_str)
            dx = x - node_pos['x']
            dy = y - node_pos['y']
            dist = math.sqrt(dx*dx + dy*dy)
            
            if dist < min_dist:
                min_dist = dist
                closest_node = node_id
        
        return closest_node
    
    def pose_callback(self, msg, robot_name):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        
        # Find closest graph node
        graph_node_id = self.find_closest_graph_node(x, y)
        
        self.positions[robot_name] = {
            'x': x,
            'y': y,
            'z': z,
            'graph_node_id': graph_node_id,  # NEW: include graph node mapping
            'orientation': {
                'x': msg.pose.pose.orientation.x,
                'y': msg.pose.pose.orientation.y,
                'z': msg.pose.pose.orientation.z,
                'w': msg.pose.pose.orientation.w
            }
        }
    
    def publish_positions(self):
        if self.positions:
            fleet_msg = FleetRobotPositions()
            
            for robot_id, pos_data in self.positions.items():
                robot_pos = RobotPosition()
                robot_pos.robot_id = robot_id
                robot_pos.x = pos_data['x']
                robot_pos.y = pos_data['y']
                robot_pos.z = pos_data['z']
                robot_pos.graph_node_id = pos_data['graph_node_id']
                robot_pos.orientation_x = pos_data['orientation']['x']
                robot_pos.orientation_y = pos_data['orientation']['y']
                robot_pos.orientation_z = pos_data['orientation']['z']
                robot_pos.orientation_w = pos_data['orientation']['w']
                
                fleet_msg.positions.append(robot_pos)
            
            self.publisher.publish(fleet_msg)
            self.get_logger().debug(f'Published {len(fleet_msg.positions)} robot positions')

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
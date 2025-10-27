#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String
import json
from smrta_messages.msg import Assignment, FleetAssignments, RobotPosition, Task, FleetTasks, FleetRobotPositions

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        
        self.declare_parameter('robot_id', 'robot1')
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('task_poses_file', '')
        
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        task_poses_file = self.get_parameter('task_poses_file').get_parameter_value().string_value
        
        self.current_tasks = []
        self.current_task_index = 0
        self.executing = False
        self.task_poses_map = {}
        
        if task_poses_file:
            try:
                with open(task_poses_file, 'r') as f:
                    self.task_poses_map = json.load(f)
                self.get_logger().info(f'Loaded task poses from {task_poses_file}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load task poses file: {e}')
        
        self.assignment_sub = self.create_subscription(
            String,
            '/smrta/task_assignments',
            self.assignment_callback,
            10
        )
        
        self.nav_action_client = ActionClient(
            self,
            NavigateToPose,
            f'/{self.robot_id}/navigate_to_pose'
        )
        
        self.get_logger().info(f'Robot Controller initialized for {self.robot_id}')
        self.get_logger().info(f'Waiting for navigation action server at /{self.robot_id}/navigate_to_pose')
    
    def assignment_callback(self, msg):
        try:
            assignments = json.loads(msg.data)
            
            if self.robot_id in assignments:
                new_tasks = assignments[self.robot_id]
                
                if new_tasks != self.current_tasks:
                    self.current_tasks = new_tasks
                    self.current_task_index = 0
                    self.get_logger().info(f'Received new task assignments: {self.current_tasks}')
                    
                    if not self.executing and self.current_tasks:
                        self.send_next_goal()
        
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse task assignments: {e}')
    
    def send_next_goal(self):
        if self.current_task_index >= len(self.current_tasks):
            self.get_logger().info('All assigned tasks completed.')
            self.executing = False
            return
        
        task_id = self.current_tasks[self.current_task_index]
        task_pose = self.lookup_task_pose(task_id)
        
        if task_pose is None:
            self.get_logger().error(f'Could not find pose for task {task_id}, skipping')
            self.current_task_index += 1
            self.executing = False
            self.send_next_goal()
            return
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = task_pose
        
        self.get_logger().info(f'Sending navigation goal for task {task_id}')
        self.get_logger().info(f'Target: x={task_pose.pose.position.x:.2f}, y={task_pose.pose.position.y:.2f}')
        
        self.executing = True
        
        if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            self.executing = False
            return
        
        self._send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def lookup_task_pose(self, task_id):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        if str(task_id) in self.task_poses_map:
            task_data = self.task_poses_map[str(task_id)]
            pose.pose.position.x = task_data.get('x', 0.0)
            pose.pose.position.y = task_data.get('y', 0.0)
            pose.pose.position.z = task_data.get('z', 0.0)
            pose.pose.orientation.x = task_data.get('qx', 0.0)
            pose.pose.orientation.y = task_data.get('qy', 0.0)
            pose.pose.orientation.z = task_data.get('qz', 0.0)
            pose.pose.orientation.w = task_data.get('qw', 1.0)
            return pose
        
        self.get_logger().warn(f'No pose mapping for task {task_id}, using default coordinates')
        pose.pose.position.x = float(task_id) * 2.0
        pose.pose.position.y = float(task_id) * 1.5
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        
        return pose
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected by action server')
            self.executing = False
            self.current_task_index += 1
            self.send_next_goal()
            return
        
        self.get_logger().info('Navigation goal accepted')
        
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_result_callback)
    
    def nav_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(
                f'Navigation succeeded for task {self.current_tasks[self.current_task_index]}'
            )
            self.current_task_index += 1
            self.executing = False
            self.send_next_goal()
        else:
            self.get_logger().error(
                f'Navigation failed with status {status} for task {self.current_tasks[self.current_task_index]}'
            )
            self.current_task_index += 1
            self.executing = False
            self.send_next_goal()
    
    def nav_feedback_callback(self, feedback_msg):
        pass

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from smrta_messages.msg import FleetAssignments, FleetTasks, Task
import json
import os


class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')
        
        # Parameters
        self.declare_parameter('robot_id', 'robot1')
        self.declare_parameter('task_poses_map_file', '/home/workspace/src/multi_robot_sim/config/task_poses_map.json')
        self.declare_parameter('graph_nodes_map_file', '/home/workspace/src/multi_robot_sim/config/graph_nodes_map.json')
        
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        task_poses_file = self.get_parameter('task_poses_map_file').get_parameter_value().string_value
        graph_nodes_file = self.get_parameter('graph_nodes_map_file').get_parameter_value().string_value
        
        # Task execution state
        self.current_tasks = []
        self.current_task_index = 0
        self.current_task_phase = 'start'  # 'start' or 'end'
        self.executing = False
        
        # Data stores
        self.task_poses_map = {}  # task_id (str) -> {x,y,z,qx,qy,qz,qw,original_node_id}
        self.graph_nodes_map = {}  # node_id (int) -> {x,y}
        self.task_start_nodes = {}  # task_id (str) -> start_node (int)
        self.task_end_nodes = {}  # task_id (str) -> end_node (int)
        
        # Load task poses map (optional, for specific task locations)
        if task_poses_file and os.path.exists(task_poses_file):
            try:
                with open(task_poses_file, 'r') as f:
                    self.task_poses_map = json.load(f)
                self.get_logger().info(f'Loaded task poses map: {task_poses_file}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load task_poses_map.json: {e}')
        else:
            self.get_logger().warn('task_poses_map.json not found or not provided')
        
        # Load graph nodes map (for node_id -> coordinates)
        if graph_nodes_file and os.path.exists(graph_nodes_file):
            try:
                with open(graph_nodes_file, 'r') as f:
                    raw_nodes = json.load(f)
                for k, v in raw_nodes.items():
                    self.graph_nodes_map[int(k)] = {'x': float(v['x']), 'y': float(v['y'])}
                self.get_logger().info(f'Loaded graph nodes map with {len(self.graph_nodes_map)} nodes')
            except Exception as e:
                self.get_logger().warn(f'Failed to load graph_nodes_map.json: {e}')
        else:
            self.get_logger().warn('graph_nodes_map.json not found or not provided')
        
        # Subscriptions
        self.assignment_sub = self.create_subscription(
            FleetAssignments,
            '/smrta/task_assignments',
            self.assignment_callback,
            10
        )
        
        self.tasks_sub = self.create_subscription(
            FleetTasks,
            '/fleet/tasks',
            self.tasks_cache_callback,
            10
        )
        
        # Navigation action client
        self.nav_action_client = ActionClient(
            self,
            NavigateToPose,
            f'/{self.robot_id}/navigate_to_pose'
        )
        
        self.get_logger().info(f'Robot Controller initialized for {self.robot_id}')
        self.get_logger().info('Subscribing to: /smrta/task_assignments (FleetAssignments)')
        self.get_logger().info('Subscribing to: /fleet/tasks (FleetTasks)')
        self.get_logger().info(f'Waiting for navigation action server at /{self.robot_id}/navigate_to_pose')
    
    def tasks_cache_callback(self, msg: FleetTasks):
        """Cache both start and end nodes for each task"""
        for t in msg.tasks:
            task_id_str = str(t.task_id)
            self.task_start_nodes[task_id_str] = int(t.start_node)
            self.task_end_nodes[task_id_str] = int(t.end_node)
            self.get_logger().debug(
                f"Cached task {task_id_str}: start={t.start_node}, end={t.end_node}"
            )
    
    def assignment_callback(self, msg: FleetAssignments):
        """Handle new task assignments from SMRTA"""
        for assignment in msg.assignments:
            if assignment.robot_id == self.robot_id:
                new_tasks = list(assignment.task_ids)
                if new_tasks != self.current_tasks:
                    self.current_tasks = new_tasks
                    self.current_task_index = 0
                    self.current_task_phase = 'start'
                    self.get_logger().info(f'Received new task assignments: {self.current_tasks}')
                    if not self.executing and self.current_tasks:
                        self.send_next_goal()
                break
    
    def send_next_goal(self):
        """Send the next navigation goal (either start or end node of current task)"""
        # Check if all tasks completed
        if self.current_task_index >= len(self.current_tasks):
            self.get_logger().info('All assigned tasks completed.')
            self.executing = False
            return
        
        task_id = self.current_tasks[self.current_task_index]
        
        # Determine which node to navigate to
        if self.current_task_phase == 'start':
            goal_pose = self.resolve_node_pose(task_id, 'start')
            phase_name = "START (pickup)"
        else:  # 'end'
            goal_pose = self.resolve_node_pose(task_id, 'end')
            phase_name = "END (dropoff)"
        
        if goal_pose is None:
            self.get_logger().error(
                f'Could not resolve {phase_name} pose for task {task_id}, skipping'
            )
            # Skip to next task
            self.current_task_index += 1
            self.current_task_phase = 'start'
            self.executing = False
            self.send_next_goal()
            return
        
        # Create navigation goal
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose
        
        self.get_logger().info(
            f'Sending navigation goal for task {task_id} - {phase_name}'
        )
        self.get_logger().info(
            f'Target: x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}'
        )
        
        self.executing = True
        
        # Wait for action server
        if not self.nav_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Navigation action server not available')
            self.executing = False
            return
        
        # Send goal
        self._send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def resolve_node_pose(self, task_id, phase):
        """
        Resolve a PoseStamped for a given task_id and phase ('start' or 'end').
        
        Priority:
        1) task_poses_map[task_id] (if available)
        2) graph_nodes_map[node_id] where node_id is start_node or end_node
        
        Args:
            task_id: Task identifier (string)
            phase: 'start' or 'end'
        
        Returns:
            PoseStamped or None
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        
        tid = str(task_id)
        
        # Determine which node to use
        if phase == 'start':
            node_id = self.task_start_nodes.get(tid, None)
        else:  # 'end'
            node_id = self.task_end_nodes.get(tid, None)
        
        if node_id is None:
            self.get_logger().error(
                f"No {phase} node found for task {task_id}"
            )
            return None
        
        # Option 1: Use task_poses_map if available (specific poses per task)
        # This would require task_poses_map to have entries like "task_2_start", "task_2_end"
        pose_key = f"{tid}_{phase}"
        if pose_key in self.task_poses_map:
            data = self.task_poses_map[pose_key]
            pose.pose.position.x = float(data.get('x', 0.0))
            pose.pose.position.y = float(data.get('y', 0.0))
            pose.pose.position.z = float(data.get('z', 0.0))
            pose.pose.orientation.x = float(data.get('qx', 0.0))
            pose.pose.orientation.y = float(data.get('qy', 0.0))
            pose.pose.orientation.z = float(data.get('qz', 0.0))
            pose.pose.orientation.w = float(data.get('qw', 1.0))
            self.get_logger().debug(f"Using task_poses_map for {pose_key}")
            return pose
        
        # Option 2: Use graph_nodes_map with node_id
        if node_id in self.graph_nodes_map:
            node_xy = self.graph_nodes_map[node_id]
            pose.pose.position.x = float(node_xy['x'])
            pose.pose.position.y = float(node_xy['y'])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            self.get_logger().debug(
                f"Using graph_nodes_map: node {node_id} -> ({node_xy['x']:.2f}, {node_xy['y']:.2f})"
            )
            return pose
        
        # Failed to resolve
        self.get_logger().error(
            f"Could not resolve pose for task {task_id}, {phase} node {node_id}"
        )
        return None
    
    def goal_response_callback(self, future):
        """Handle navigation goal acceptance/rejection"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Navigation goal rejected by action server')
            self.executing = False
            # Move to next phase or task
            if self.current_task_phase == 'start':
                # Skip to end phase
                self.current_task_phase = 'end'
            else:
                # Move to next task
                self.current_task_index += 1
                self.current_task_phase = 'start'
            self.send_next_goal()
            return
        
        self.get_logger().info('Navigation goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.nav_result_callback)
    
    def nav_result_callback(self, future):
        """Handle navigation result"""
        status = future.result().status
        task_id = self.current_tasks[self.current_task_index]
        phase = "START" if self.current_task_phase == 'start' else "END"
        
        if status == 4:  # SUCCEEDED
            self.get_logger().info(
                f'Navigation succeeded for task {task_id} - {phase}'
            )
        else:
            self.get_logger().error(
                f'Navigation failed with status {status} for task {task_id} - {phase}'
            )
        
        # Progress to next phase or task
        if self.current_task_phase == 'start':
            # Move to end phase of same task
            self.current_task_phase = 'end'
            self.get_logger().info(f'Task {task_id}: START complete, moving to END phase')
        else:
            # Move to next task
            self.current_task_index += 1
            self.current_task_phase = 'start'
            self.get_logger().info(f'Task {task_id}: END complete, task finished')
        
        self.executing = False
        self.send_next_goal()
    
    def nav_feedback_callback(self, feedback_msg):
        """Handle navigation feedback (optional)"""
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

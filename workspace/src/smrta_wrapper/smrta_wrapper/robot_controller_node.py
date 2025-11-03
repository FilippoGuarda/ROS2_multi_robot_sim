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

        self.declare_parameter('robot_id', 'robot1')
        self.declare_parameter('task_poses_map_file', '/home/workspace/src/multi_robot_sim/config/task_poses_map.json')
        self.declare_parameter('graph_nodes_map_file', '/home/workspace/src/multi_robot_sim/config/graph_nodes_map.json')

        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value
        task_poses_file = self.get_parameter('task_poses_map_file').get_parameter_value().string_value
        graph_nodes_file = self.get_parameter('graph_nodes_map_file').get_parameter_value().string_value

        self.current_tasks = []
        self.current_task_index = 0
        self.executing = False
        self.task_poses_map = {}          # task_id (str) -> {x,y,z,qx,qy,qz,qw,original_node_id}
        self.graph_nodes_map = {}         # node_id (str/int) -> {x,y}
        self.task_end_nodes = {}          # task_id (str) -> end_node (int)

        if task_poses_file and os.path.exists(task_poses_file):
            try:
                with open(task_poses_file, 'r') as f:
                    self.task_poses_map = json.load(f)
                self.get_logger().info(f'Loaded task poses map: {task_poses_file}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load task_poses_map.json: {e}')
        else:
            self.get_logger().warn('task_poses_map.json not found or not provided')

        # Load graph nodes (map coordinates) for fallback (end_node → pose)
        if graph_nodes_file and os.path.exists(graph_nodes_file):
            try:
                with open(graph_nodes_file, 'r') as f:
                    raw_nodes = json.load(f)
                for k, v in raw_nodes.items():
                    self.graph_nodes_map[int(k)] = {'x': float(v['x']), 'y': float(v['y'])}
                self.get_logger().info(f'Loaded graph nodes map: {graph_nodes_file}')
            except Exception as e:
                self.get_logger().warn(f'Failed to load graph_nodes_map.json: {e}')
        else:
            self.get_logger().warn('graph_nodes_map.json not found or not provided')

        # Subscriptions
        # 1) Assignments from SMRTA
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

        self.nav_action_client = ActionClient(
            self,
            NavigateToPose,
            f'/{self.robot_id}/navigate_to_pose'
        )

        self.get_logger().info(f'Robot Controller initialized for {self.robot_id}')
        self.get_logger().info('Subscribing to: /smrta/task_assignments (FleetAssignments)')
        self.get_logger().info(f'Waiting for navigation action server at /{self.robot_id}/navigate_to_pose')

    def tasks_cache_callback(self, msg: FleetTasks):
        for t in msg.tasks:
            self.task_end_nodes[str(t.task_id)] = int(t.end_node)

    def assignment_callback(self, msg: FleetAssignments):
        # Find robot's assignment list
        for assignment in msg.assignments:
            if assignment.robot_id == self.robot_id:
                new_tasks = list(assignment.task_ids)
                if new_tasks != self.current_tasks:
                    self.current_tasks = new_tasks
                    self.current_task_index = 0
                    self.get_logger().info(f'Received new task assignments: {self.current_tasks}')
                    if not self.executing and self.current_tasks:
                        self.send_next_goal()
                break

    def send_next_goal(self):
        if self.current_task_index >= len(self.current_tasks):
            self.get_logger().info('All assigned tasks completed.')
            self.executing = False
            return

        task_id = self.current_tasks[self.current_task_index]
        goal_pose = self.resolve_task_pose(task_id)

        if goal_pose is None:
            self.get_logger().error(f'Could not resolve pose for task {task_id}, skipping')
            self.current_task_index += 1
            self.executing = False
            self.send_next_goal()
            return

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = goal_pose

        self.get_logger().info(f'Sending navigation goal for task {task_id}')
        self.get_logger().info(f'Target: x={goal_pose.pose.position.x:.2f}, y={goal_pose.pose.position.y:.2f}')

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

    def resolve_task_pose(self, task_id):
        """
        Resolve a PoseStamped in 'map' frame for a given task_id.
        Priority:
          1) task_poses_map[task_id]  (already in map coordinates)
          2) end_node → graph_nodes_map[end_node] (fallback via /fleet/tasks cache)
        """
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        tid = str(task_id)

        # 1) Preferred: task_poses_map.json keyed by task_id
        if tid in self.task_poses_map:
            data = self.task_poses_map[tid]
            pose.pose.position.x = float(data.get('x', 0.0))
            pose.pose.position.y = float(data.get('y', 0.0))
            pose.pose.position.z = float(data.get('z', 0.0))
            pose.pose.orientation.x = float(data.get('qx', 0.0))
            pose.pose.orientation.y = float(data.get('qy', 0.0))
            pose.pose.orientation.z = float(data.get('qz', 0.0))
            pose.pose.orientation.w = float(data.get('qw', 1.0))
            return pose

        # 2) Fallback: use end_node → graph_nodes_map.json
        end_node = self.task_end_nodes.get(tid, None)
        if end_node is not None and end_node in self.graph_nodes_map:
            node_xy = self.graph_nodes_map[end_node]
            pose.pose.position.x = float(node_xy['x'])
            pose.pose.position.y = float(node_xy['y'])
            pose.pose.position.z = 0.0
            pose.pose.orientation.w = 1.0
            return pose

        # Failed to resolve
        return None

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
        status = future.result().status
        if status == 4:  # SUCCEEDED
            self.get_logger().info(
                f'Navigation succeeded for task {self.current_tasks[self.current_task_index]}'
            )
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

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from smrta_messages.msg import FleetTasks, Task
import sys

class TaskPublisherCLI(Node):
    def __init__(self):
        super().__init__('task_publisher_cli')
        
        self.publisher = self.create_publisher(FleetTasks, '/fleet/tasks', 10)
        self.get_logger().info('Task Publisher CLI initialized')
        self.get_logger().info('Publishing to: /fleet/tasks (FleetTasks)')
    
    def publish_tasks(self, tasks_data):
        """
        Publish a list of tasks
        
        Args:
            tasks_data: List of dicts, each containing:
                - task_id: int or string
                - start_node: int (graph node ID)
                - end_node: int (graph node ID)
                - deadline: int (optional, 0 or negative means no deadline)
        """
        fleet_tasks = FleetTasks()
        
        for task_data in tasks_data:
            task_msg = Task()
            task_msg.task_id = str(task_data['task_id'])
            task_msg.start_node = int(task_data['start_node'])
            task_msg.end_node = int(task_data['end_node'])
            task_msg.deadline = int(task_data.get('deadline', 0))
            
            fleet_tasks.tasks.append(task_msg)
        
        self.publisher.publish(fleet_tasks)
        self.get_logger().info(f'Published {len(fleet_tasks.tasks)} tasks')
        
        for task in fleet_tasks.tasks:
            deadline_str = f", deadline={task.deadline}" if task.deadline > 0 else ""
            self.get_logger().info(f'  Task {task.task_id}: {task.start_node} -> {task.end_node}{deadline_str}')

def main(args=None):
    rclpy.init(args=args)
    node = TaskPublisherCLI()
    
    # Example tasks - modify as needed
    # Task format: {task_id, start_node, end_node, deadline (optional)}
    example_tasks = [
        {'task_id': '1', 'start_node': 0, 'end_node': 5, 'deadline': 100},
        {'task_id': '2', 'start_node': 1, 'end_node': 7, 'deadline': 150},
        {'task_id': '3', 'start_node': 2, 'end_node': 9, 'deadline': 0},  # No deadline
    ]
    
    # Check if tasks provided via command line
    if len(sys.argv) > 1:
        # Parse command line arguments
        # Format: task_id,start,end,deadline task_id,start,end,deadline ...
        tasks = []
        for arg in sys.argv[1:]:
            parts = arg.split(',')
            if len(parts) >= 3:
                task = {
                    'task_id': parts[0],
                    'start_node': int(parts[1]),
                    'end_node': int(parts[2]),
                    'deadline': int(parts[3]) if len(parts) > 3 else 0
                }
                tasks.append(task)
        
        if tasks:
            node.publish_tasks(tasks)
        else:
            node.get_logger().error('Invalid task format. Use: task_id,start,end[,deadline]')
    else:
        # Publish example tasks
        node.get_logger().info('No tasks provided via CLI, publishing example tasks')
        node.get_logger().info('Usage: ros2 run smrta_wrapper task_publisher_cli task_id,start,end[,deadline] ...')
        node.publish_tasks(example_tasks)
    
    # Give time for message to be sent
    rclpy.spin_once(node, timeout_sec=1.0)
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
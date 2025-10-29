#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from smrta_messages.msg import FleetRobotPositions, FleetTasks, FleetAssignments, Assignment
import json
import os

# Import SMrTA
try:
    from MRTASolver.objects import Robot, Task
    from MRTASolver.MRTASolver import MRTASolver
    from MRTASolver.run_realistic_setting import load_weighted_graph, dictionary_to_matrix
    SMRTA_AVAILABLE = True
except ImportError:
    SMRTA_AVAILABLE = False

class SMrTaTaskAssignmentNode(Node):
    def __init__(self):
        super().__init__('smrta_task_assignment_node')
        
        # Declare parameters
        self.declare_parameter('graph_file', '/home/workspace/src/multi_robot_sim/config/weighted_graph_p3.pkl')
        self.declare_parameter('capacity', 2)
        self.declare_parameter('solver_name', 'z3')
        self.declare_parameter('theory', 'QF_UFLIA')
        self.declare_parameter('allocation_period', 5.0)
        
        # Get parameters
        self.graph_file = self.get_parameter('graph_file').get_parameter_value().string_value
        self.capacity = self.get_parameter('capacity').get_parameter_value().integer_value
        self.solver_name = self.get_parameter('solver_name').get_parameter_value().string_value
        self.theory = self.get_parameter('theory').get_parameter_value().string_value
        self.allocation_period = self.get_parameter('allocation_period').get_parameter_value().double_value
        
        # Internal state
        self.robot_states = {}
        self.tasks = []
        self.graph = None
        self.room_count = None
        
        # Check if SMrTA is available
        if not SMRTA_AVAILABLE:
            self.get_logger().error('SMrTA solver not available. Please install MRTASolver package.')
            return
        
        # Subscriptions - CORRECTED to use custom messages
        self.position_sub = self.create_subscription(
            FleetRobotPositions,
            '/fleet/robot_positions',
            self.positions_callback,
            10
        )
        
        self.task_sub = self.create_subscription(
            FleetTasks,
            '/fleet/tasks',
            self.tasks_callback,
            10
        )
        
        # Publisher - CORRECTED to use custom message
        self.assignment_pub = self.create_publisher(FleetAssignments, '/smrta/task_assignments', 10)
        
        # Timer
        self.timer = self.create_timer(self.allocation_period, self.run_smrta)
        
        self.get_logger().info(f'SMrTA Task Assignment Node initialized')
        self.get_logger().info(f'Graph file: {self.graph_file}')
        self.get_logger().info(f'Solver: {self.solver_name}, Theory: {self.theory}')
        self.get_logger().info(f'Subscribing to: /fleet/robot_positions (FleetRobotPositions)')
        self.get_logger().info(f'Subscribing to: /fleet/tasks (FleetTasks)')
        self.get_logger().info(f'Publishing to: /smrta/task_assignments (FleetAssignments)')
    
    def positions_callback(self, msg):
        """Callback for FleetRobotPositions message"""
        self.robot_states = {}
        for robot_pos in msg.positions:
            self.robot_states[robot_pos.robot_id] = {
                'x': robot_pos.x,
                'y': robot_pos.y,
                'z': robot_pos.z,
                'graph_node_id': robot_pos.graph_node_id,  # CORRECTED: now available
                'orientation': {
                    'x': robot_pos.orientation_x,
                    'y': robot_pos.orientation_y,
                    'z': robot_pos.orientation_z,
                    'w': robot_pos.orientation_w
                }
            }
        self.get_logger().debug(f"Received positions for {len(self.robot_states)} robots")
    
    def tasks_callback(self, msg):
        """Callback for FleetTasks message"""
        self.tasks = []
        for task_msg in msg.tasks:
            task_dict = {
                'id': task_msg.task_id,
                'start': task_msg.start_node,
                'end': task_msg.end_node,
                'deadline': task_msg.deadline if task_msg.deadline > 0 else None
            }
            self.tasks.append(task_dict)
        self.get_logger().info(f"Received {len(self.tasks)} tasks")
    
    def run_smrta(self):
        if not self.robot_states:
            self.get_logger().debug("Waiting for robot positions...")
            return
        
        if not self.tasks:
            self.get_logger().debug("Waiting for tasks...")
            return
        
        if not SMRTA_AVAILABLE:
            return
        
        try:
            # Load graph if needed
            if self.graph is None or self.room_count is None:
                if not os.path.exists(self.graph_file):
                    self.get_logger().error(f'Graph file not found: {self.graph_file}')
                    return
                
                self.get_logger().info(f'Loading graph from {self.graph_file}')
                room_dictionary = load_weighted_graph(self.graph_file)
                self.room_count, self.graph = dictionary_to_matrix(room_dictionary)
                self.get_logger().info(f'Graph loaded: {self.room_count} rooms')
            
            # Prepare robots - CORRECTED: use graph_node_id instead of room_id
            robots = []
            for robot_id, pose in self.robot_states.items():
                position = pose.get('graph_node_id', 0)
                robots.append(Robot(robot_id, position))
            
            # Prepare tasks
            task_objects = []
            for t in self.tasks:
                task_obj = Task(
                    t['id'],
                    t['start'],
                    t['end'],
                    t.get('deadline', None)
                )
                task_objects.append(task_obj)
            
            tasks_stream = [(task_objects, 0)]

            aps_list = list(range(self.room_count))
            num_aps = aps_list[-1] if aps_list else 0
            
            # Run solver
            self.get_logger().info('Running SMrTA solver...')
            solver = MRTASolver(
                solver_name=self.solver_name,
                theory=self.theory,
                agents=robots,
                tasks_stream=tasks_stream,
                room_graph=self.graph,
                capacity=self.capacity,
                aps_list=aps_list,
                num_aps=num_aps,
                debug=False
            )
            
            solution = solver.extract_model(solver.s)
            
            if solution:
                self.publish_assignments(solution, robots)
            else:
                self.get_logger().warn('No solution found by SMrTA solver')
        
        except Exception as e:
            self.get_logger().error(f'Error running SMrTA: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def publish_assignments(self, solution, robots):
        """Publish task assignments using custom message"""
        try:
            fleet_assignments = FleetAssignments()
            
            if 'agt' in solution:
                for idx, robot_data in enumerate(solution['agt']):
                    if idx < len(robots):
                        robot_id = robots[idx].id
                        assigned_task_ids = robot_data.get('id', [])
                        
                        assignment = Assignment()
                        assignment.robot_id = robot_id
                
                        assignment.task_ids = [str(task_id) for task_id in assigned_task_ids]
                        
                        fleet_assignments.assignments.append(assignment)
            
            self.assignment_pub.publish(fleet_assignments)
            self.get_logger().info(f"Published assignments for {len(fleet_assignments.assignments)} robots")
            
            # Log the assignments
            for assignment in fleet_assignments.assignments:
                self.get_logger().info(f"  {assignment.robot_id}: {assignment.task_ids}")
        
        except Exception as e:
            self.get_logger().error(f'Error publishing assignments: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = SMrTaTaskAssignmentNode()
    
    if SMRTA_AVAILABLE:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
            pass
        finally:
            node.destroy_node()
            rclpy.shutdown()
    else:
        node.get_logger().error('Cannot start node without SMrTA solver')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
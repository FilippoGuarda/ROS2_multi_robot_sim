#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
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
        self.declare_parameter('graph_file', 'weighted_graph_p3.pkl')
        self.declare_parameter('capacity', 2)
        self.declare_parameter('num_aps', 5)
        self.declare_parameter('solver_name', 'z3')
        self.declare_parameter('theory', 'QF_UFLIA')
        self.declare_parameter('use_sim_time', True)
        self.declare_parameter('allocation_period', 5.0)
        
        # Get parameters
        self.graph_file = self.get_parameter('graph_file').get_parameter_value().string_value
        self.capacity = self.get_parameter('capacity').get_parameter_value().integer_value
        self.num_aps = self.get_parameter('num_aps').get_parameter_value().integer_value
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
        
        # Subscriptions
        self.position_sub = self.create_subscription(
            String, 
            '/fleet/robot_positions', 
            self.positions_callback, 
            10
        )
        
        self.task_sub = self.create_subscription(
            String, 
            '/fleet/tasks', 
            self.tasks_callback, 
            10
        )
        
        # Publisher
        self.assignment_pub = self.create_publisher(String, '/smrta/task_assignments', 10)
        
        # Timer
        self.timer = self.create_timer(self.allocation_period, self.run_smrta)
        
        self.get_logger().info(f'SMrTA Task Assignment Node initialized')
        self.get_logger().info(f'Graph file: {self.graph_file}')
        self.get_logger().info(f'Capacity: {self.capacity}, Num APs: {self.num_aps}')
        self.get_logger().info(f'Solver: {self.solver_name}, Theory: {self.theory}')
    
    def positions_callback(self, msg):
        try:
            self.robot_states = json.loads(msg.data)
            self.get_logger().debug(f"Received robot positions: {list(self.robot_states.keys())}")
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse robot positions: {e}')
    
    def tasks_callback(self, msg):
        try:
            task_dicts = json.loads(msg.data)
            self.tasks = task_dicts
            self.get_logger().info(f"Received {len(self.tasks)} tasks")
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse tasks: {e}')
    
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
            
            # Prepare robots
            robots = []
            for robot_id, pose in self.robot_states.items():
                position = pose.get('room_id', 0)
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
            
            # Run solver
            self.get_logger().info('Running SMrTA solver...')
            solver = MRTASolver(
                solver_name=self.solver_name,
                theory=self.theory,
                agents=robots,
                tasks_stream=tasks_stream,
                room_graph=self.graph,
                capacity=self.capacity,
                num_aps=self.num_aps,
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
        try:
            assignments = {}
            
            if 'agt' in solution:
                for idx, robot_data in enumerate(solution['agt']):
                    if idx < len(robots):
                        robot_id = robots[idx].id
                        assigned_tasks = robot_data.get('id', [])
                        assignments[robot_id] = assigned_tasks
            
            assignment_msg = json.dumps(assignments)
            self.assignment_pub.publish(String(data=assignment_msg))
            self.get_logger().info(f"Published assignments: {assignments}")
        
        except Exception as e:
            self.get_logger().error(f'Error publishing assignments: {e}')

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
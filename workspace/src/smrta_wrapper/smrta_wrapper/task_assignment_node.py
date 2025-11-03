#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from smrta_messages.msg import FleetRobotPositions, FleetTasks, FleetAssignments, Assignment
import json
import os
import signal

# Import SMrTA
try:
    from MRTASolver.objects import Robot, Task
    from MRTASolver.SolverInterface import Result
    from MRTASolver.MRTASolver import MRTASolver
    from MRTASolver.run_realistic_setting import load_weighted_graph, dictionary_to_matrix
    SMRTA_AVAILABLE = True
except ImportError:
    SMRTA_AVAILABLE = False

class SMrTaTaskAssignmentNode(Node):
    def __init__(self):
        super().__init__('smrta_task_assignment_node')
        self._shutdown = False
        signal.signal(signal.SIGINT, self._signal_handler)

        self.declare_parameter('graph_file', '/home/workspace/src/multi_robot_sim/config/weighted_graph.pkl')
        self.declare_parameter('node_mapping_file', '/home/workspace/src/multi_robot_sim/config/node_mapping.json')
        self.declare_parameter('capacity', 2)
        self.declare_parameter('solver_name', 'z3')
        self.declare_parameter('theory', 'QF_UFLIA')
        self.declare_parameter('allocation_period', 5.0)
        
        self.graph_file = self.get_parameter('graph_file').get_parameter_value().string_value
        self.node_mapping_file = self.get_parameter('node_mapping_file').get_parameter_value().string_value
        self.capacity = self.get_parameter('capacity').get_parameter_value().integer_value
        self.solver_name = self.get_parameter('solver_name').get_parameter_value().string_value
        self.theory = self.get_parameter('theory').get_parameter_value().string_value
        self.allocation_period = self.get_parameter('allocation_period').get_parameter_value().double_value

        self.robot_states = {}
        self.tasks = []
        self.graph = None
        self.room_count = None

        self.original_to_seq = {}  # original_id → sequential_id
        self.seq_to_original = {}  # sequential_id → original_id

        if not SMRTA_AVAILABLE:
            self.get_logger().error('SMrTA solver not available. Please install MRTASolver package.')
            return

        self.load_node_mappings()

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

        self.assignment_pub = self.create_publisher(FleetAssignments, '/smrta/task_assignments', 10)

        self.timer = self.create_timer(self.allocation_period, self.run_smrta)
        
        self.get_logger().info(f'SMrTA Task Assignment Node initialized')
        self.get_logger().info(f'Graph file: {self.graph_file}')
        self.get_logger().info(f'Node mapping file: {self.node_mapping_file}')
        self.get_logger().info(f'Solver: {self.solver_name}, Theory: {self.theory}')
        self.get_logger().info(f'Subscribing to: /fleet/robot_positions (FleetRobotPositions)')
        self.get_logger().info(f'Subscribing to: /fleet/tasks (FleetTasks)')
        self.get_logger().info(f'Publishing to: /smrta/task_assignments (FleetAssignments)')
    
    def load_node_mappings(self):
        """Load node ID mappings for converting between original and sequential IDs."""
        if not os.path.exists(self.node_mapping_file):
            self.get_logger().warn(f'Node mapping file not found: {self.node_mapping_file}')
            self.get_logger().warn('Assuming sequential node IDs (0, 1, 2, ...)')
            return
        
        try:
            with open(self.node_mapping_file, 'r') as f:
                mapping = json.load(f)
            
            # Convert string keys back to integers
            self.original_to_seq = {int(k): v for k, v in mapping['original_to_sequential'].items()}
            self.seq_to_original = {int(k): v for k, v in mapping['sequential_to_original'].items()}
            
            self.get_logger().info(f'Loaded node mappings: {len(self.original_to_seq)} nodes')
            self.get_logger().info(f'  Original ID range: {min(self.original_to_seq.keys())} to {max(self.original_to_seq.keys())}')
            self.get_logger().info(f'  Sequential ID range: 0 to {len(self.original_to_seq) - 1}')
        
        except Exception as e:
            self.get_logger().error(f'Failed to load node mappings: {e}')
    
    def _convert_original_to_seq(self, original_id):
        """Convert original node ID to sequential ID for SMRTA."""
        if not self.original_to_seq:

            return original_id
        
        if original_id not in self.original_to_seq:
            self.get_logger().warn(f'Node {original_id} not in mapping, using as-is')
            return original_id
        
        return self.original_to_seq[original_id]
    
    def _convert_seq_to_original(self, seq_id):
        """Convert sequential ID from SMRTA back to original node ID."""
        if not self.seq_to_original:

            return seq_id
        
        if seq_id not in self.seq_to_original:
            self.get_logger().warn(f'Sequential ID {seq_id} not in mapping, using as-is')
            return seq_id
        
        return self.seq_to_original[seq_id]
    
    def positions_callback(self, msg):
        """Callback for FleetRobotPositions message"""
        self.robot_states = {}
        for robot_pos in msg.positions:
            self.robot_states[robot_pos.robot_id] = {
                'x': robot_pos.x,
                'y': robot_pos.y,
                'z': robot_pos.z,
                'graph_node_id': robot_pos.graph_node_id,  # Original ID
                'orientation': {
                    'x': robot_pos.orientation_x,
                    'y': robot_pos.orientation_y,
                    'z': robot_pos.orientation_z,
                    'w': robot_pos.orientation_w
                }
            }
        self.get_logger().debug(f"Received positions for {len(self.robot_states)} robots")
    
    def _signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully"""
        self.get_logger().info('Shutdown requested')
        self._shutdown = True
    
    def tasks_callback(self, msg):
        """Callback for FleetTasks message"""
        self.tasks = []
        for task_msg in msg.tasks:
            task_dict = {
                'id': task_msg.task_id,
                'start': task_msg.start_node,  # Original ID
                'end': task_msg.end_node,      # Original ID
                'deadline': task_msg.deadline if task_msg.deadline > 0 else None
            }
            self.tasks.append(task_dict)
        self.get_logger().info(f"Received {len(self.tasks)} tasks")
    
    def run_smrta(self):
        if self._shutdown:
            return
        
        if not self.robot_states:
            self.get_logger().debug("Waiting for robot positions...")
            return
        
        if not self.tasks:
            self.get_logger().debug("Waiting for tasks...")
            return
        
        if not SMRTA_AVAILABLE:
            return
        
        solution = None
        
        try:
            # Load graph if needed
            if self.graph is None or self.room_count is None:
                if not os.path.exists(self.graph_file):
                    self.get_logger().error(f'Graph file not found: {self.graph_file}')
                    return
                
                self.get_logger().info(f'Loading graph from {self.graph_file}')
                room_dictionary = load_weighted_graph(self.graph_file)
                self.room_count, self.graph = dictionary_to_matrix(room_dictionary)
                self.get_logger().info(f'Graph loaded: {self.room_count} rooms (sequential IDs)')
            
            # Prepare robots - CONVERT from original to sequential IDs
            robots = []
            for robot_id, pose in self.robot_states.items():
                original_position = pose.get('graph_node_id', 0)
                # Convert original ID to sequential ID
                seq_position = self._convert_original_to_seq(original_position)
                
                if seq_position >= self.room_count:
                    self.get_logger().error(
                        f"Robot {robot_id}: sequential position {seq_position} >= graph size {self.room_count}"
                    )
                    continue
                
                robots.append(Robot(robot_id, seq_position))
                self.get_logger().debug(f"Robot {robot_id}: original node {original_position} → sequential {seq_position}")
            

            task_objects = []
            for t in self.tasks:
                original_start = t['start']
                original_end = t['end']
                

                seq_start = self._convert_original_to_seq(original_start)
                seq_end = self._convert_original_to_seq(original_end)
                
                if seq_start >= self.room_count or seq_end >= self.room_count:
                    self.get_logger().error(
                        f"Task {t['id']}: converted to {seq_start} → {seq_end}, "
                        f"exceeds graph size {self.room_count}"
                    )
                    continue
                
                task_obj = Task(
                    t['id'],
                    seq_start,
                    seq_end,
                    t.get('deadline', None)
                )
                task_objects.append(task_obj)
                self.get_logger().debug(
                    f"Task {t['id']}: original {original_start} → {original_end}, "
                    f"sequential {seq_start} → {seq_end}"
                )
            
            if not robots or not task_objects:
                self.get_logger().error('No valid robots or tasks after conversion')
                return
            
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
            
            # Handle results
            if solver.s.res == Result.sat:
                self.get_logger().info('Solver found a solution (sat)')
                solution = solver.extract_model(solver.s)
                if solution:
                    self.publish_assignments(solution, robots)
                    self.tasks = []
                    self.get_logger().info('Tasks assigned and cleared')
                else:
                    self.get_logger().warn('Solver returned sat but failed to extract solution')
            
            elif solver.s.res == Result.unsat:
                self.get_logger().warn('No solution exists for current tasks (unsat)')
                self.get_logger().warn(f'  Robots: {len(robots)}, Tasks: {len(task_objects)}, Nodes: {self.room_count}')
                self.get_logger().warn('  Check robot positions and task nodes are valid')
            
            elif solver.s.res == Result.unknown:
                self.get_logger().warn('Solver timeout or interrupted (unknown)')
            
            else:
                self.get_logger().error(f'Unexpected solver result: {solver.s.res}')
        
        except Exception as e:
            self.get_logger().error(f'Error running SMrTA: {e}')
            import traceback
            self.get_logger().error(traceback.format_exc())
    
    def publish_assignments(self, solution, robots):
        """Publish task assignments"""
        try:
            fleet_assignments = FleetAssignments()
            
            if 'agt' in solution:
                for idx, robot_data in enumerate(solution['agt']):
                    if idx < len(robots):
                        robot_id = robots[idx].id
                        
                        # Extract unique tasks from schedule
                        schedule = robot_data.get('id', [])
                        unique_tasks = []
                        seen = set()
                        for task_id in schedule:
                            if task_id != 0 and task_id not in seen:
                                unique_tasks.append(task_id)
                                seen.add(task_id)
                        
                        if unique_tasks:
                            assignment = Assignment()
                            assignment.robot_id = str(robot_id)
                            assignment.task_ids = [str(task_id) for task_id in unique_tasks]
                            fleet_assignments.assignments.append(assignment)
            
            self.assignment_pub.publish(fleet_assignments)
            self.get_logger().info(f"Published assignments for {len(fleet_assignments.assignments)} robots")
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
        except:
            node.get_logger().info('Keyboard interrupt received')
        finally:
            node.destroy_node()
            rclpy.shutdown()

    else:
        node.get_logger().error('Cannot start node without SMrTA solver')
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
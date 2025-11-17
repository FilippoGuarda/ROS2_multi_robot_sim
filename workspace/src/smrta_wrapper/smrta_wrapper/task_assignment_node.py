#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from smrta_messages.msg import FleetRobotPositions, FleetTasks, FleetAssignments, Assignment
import json
import os
import signal
import math

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

        # Declare parameters
        self.declare_parameter('graph_file', '/home/workspace/src/multi_robot_sim/config/weighted_graph.pkl')
        self.declare_parameter('node_mapping_file', '/home/workspace/src/multi_robot_sim/config/node_mapping.json')
        self.declare_parameter('capacity', 1)
        self.declare_parameter('solver_name', 'z3')
        self.declare_parameter('theory', 'QF_UFLIA')
        self.declare_parameter('allocation_period', 5.0)

        # Get parameters
        self.graph_file = self.get_parameter('graph_file').get_parameter_value().string_value
        self.node_mapping_file = self.get_parameter('node_mapping_file').get_parameter_value().string_value
        self.capacity = self.get_parameter('capacity').get_parameter_value().integer_value
        self.solver_name = self.get_parameter('solver_name').get_parameter_value().string_value
        self.theory = self.get_parameter('theory').get_parameter_value().string_value
        self.allocation_period = self.get_parameter('allocation_period').get_parameter_value().double_value

        # State variables
        self.robot_states = {}
        self.tasks = []
        self.graph = None
        self.node_id = None
        self.original_to_seq = {}  # original_id → sequential_id
        self.seq_to_original = {}  # sequential_id → original_id
        self.node_coordinates_map = None

        if not SMRTA_AVAILABLE:
            self.get_logger().error('SMrTA solver not available. Please install MRTASolver package.')
            return

        # Load node mappings
        self.load_node_mappings()

        # Create subscribers
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

        # Create publisher
        self.assignment_pub = self.create_publisher(FleetAssignments, '/smrta/task_assignments', 10)

        # Create timer for periodic task assignment
        self.timer = self.create_timer(self.allocation_period, self.run_smrta)

        # Log initialization
        self.get_logger().info('SMrTA Task Assignment Node initialized')
        self.get_logger().info(f'Graph file: {self.graph_file}')
        self.get_logger().info(f'Node mapping file: {self.node_mapping_file}')
        self.get_logger().info(f'Solver: {self.solver_name}, Theory: {self.theory}')
        self.get_logger().info('Subscribing to: /fleet/robot_positions (FleetRobotPositions)')
        self.get_logger().info('Subscribing to: /fleet/tasks (FleetTasks)')
        self.get_logger().info('Publishing to: /smrta/task_assignments (FleetAssignments)')

    def _signal_handler(self, signum, frame):
        self.get_logger().info('Shutdown requested')
        self._shutdown = True

    def load_node_mappings(self):
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
        if not self.original_to_seq:
            return original_id
        if original_id not in self.original_to_seq:
            self.get_logger().warn(f'Node {original_id} not in mapping, using as-is')
            return original_id
        return self.original_to_seq[original_id]

    def _convert_seq_to_original(self, seq_id):
        if not self.seq_to_original:
            return seq_id
        if seq_id not in self.seq_to_original:
            self.get_logger().warn(f'Sequential ID {seq_id} not in mapping, using as-is')
            return seq_id
        return self.seq_to_original[seq_id]

    def _load_node_coordinates(self):
        try:
            # Default location
            coord_file = '/home/workspace/src/multi_robot_sim/config/graph_nodes_map.json'

            if not os.path.exists(coord_file):
                self.get_logger().error(f"Node coordinates file not found: {coord_file}")
                return None

            with open(coord_file, 'r') as f:
                node_map = json.load(f)

            self.get_logger().info(f"Loaded {len(node_map)} node coordinates from {coord_file}")
            return node_map
        except json.JSONDecodeError as e:
            self.get_logger().error(f"Error parsing JSON from {coord_file}: {e}")
            return None
        except Exception as e:
            self.get_logger().error(f"Error loading node coordinates: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return None

    def find_closest_node(self, robot_pose):
        if self.graph is None or self.node_id is None:
            self.get_logger().error("Graph not loaded, returning default node 0")
            return 0

        # FIRST PRIORITY: If robot has a valid explicitly assigned node, use it
        graph_node = robot_pose.get('graph_node_id', None)
        if graph_node is not None:
            # graph_node is in original numbering, convert to sequential
            if 0 <= graph_node < len(self.seq_to_original):
                try:
                    seq_node = self.original_to_seq.get(graph_node)
                    if seq_node is not None and 0 <= seq_node < self.node_id:
                        self.get_logger().debug(
                            f"Using explicitly assigned node: "
                            f"original={graph_node} → sequential={seq_node}"
                        )
                        return seq_node
                except Exception as e:
                    self.get_logger().warn(
                        f"Error converting node {graph_node}: {e}, using fallback"
                    )

        # SECOND PRIORITY: Find closest node by coordinates
        robot_x = robot_pose.get('x', None)
        robot_y = robot_pose.get('y', None)

        if robot_x is None or robot_y is None:
            self.get_logger().warn(
                f"Robot position missing x={robot_x}, y={robot_y}, "
                f"no graph_node_id provided, returning node 0"
            )
            return 0

        try:
            # Load graph_nodes_map.json to get node coordinates
            if self.node_coordinates_map is None:
                self.node_coordinates_map = self._load_node_coordinates()
                if not self.node_coordinates_map:
                    self.get_logger().error(
                        "Could not load node coordinates, returning default node 0"
                    )
                    return 0

            # Find the closest node by Euclidean distance
            closest_node_orig = None
            min_distance = float('inf')

            for node_id_str, coords in self.node_coordinates_map.items():
                node_id = int(node_id_str)
                # Only consider nodes that exist in the graph
                if node_id not in self.original_to_seq:
                    continue

                # Calculate Euclidean distance
                dx = coords['x'] - robot_x
                dy = coords['y'] - robot_y
                distance = (dx**2 + dy**2) ** 0.5

                if distance < min_distance:
                    min_distance = distance
                    closest_node_orig = node_id

            if closest_node_orig is None:
                self.get_logger().error(
                    "No valid nodes found in coordinate map, returning default node 0"
                )
                return 0

            # Convert original node ID to sequential
            seq_node = self.original_to_seq[closest_node_orig]

            if not (0 <= seq_node < self.node_id):
                self.get_logger().error(
                    f"Closest node {closest_node_orig} converted to invalid "
                    f"sequential ID {seq_node}, returning node 0"
                )
                return 0

            self.get_logger().info(
                f"Robot at ({robot_x:.2f}, {robot_y:.2f}): closest node is "
                f"{closest_node_orig} (seq={seq_node}) at distance {min_distance:.2f}m"
            )
            return seq_node

        except Exception as e:
            self.get_logger().error(
                f"Error finding closest node: {e}, returning default node 0"
            )
            import traceback
            self.get_logger().error(traceback.format_exc())
            return 0

    def positions_callback(self, msg):
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

    def tasks_callback(self, msg):
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
            if self.graph is None or self.node_id is None:
                if not os.path.exists(self.graph_file):
                    self.get_logger().error(f'Graph file not found: {self.graph_file}')
                    return

                self.get_logger().info(f'Loading graph from {self.graph_file}')
                node_dictionary = load_weighted_graph(self.graph_file)
                self.node_id, self.graph = dictionary_to_matrix(node_dictionary)
                self.get_logger().info(f'Graph loaded: {self.node_id} nodes (sequential IDs)')

            # Create Robot objects with home nodes
            robots = []
            for robot_id, robot_pose in self.robot_states.items():
                home_node_seq = self.find_closest_node(robot_pose)
                if home_node_seq >= self.node_id:
                    self.get_logger().error(
                        f"Robot {robot_id} home node {home_node_seq} >= graph size {self.node_id}"
                    )
                    continue
                robots.append(Robot(robot_id, home_node_seq))
                self.get_logger().debug(f"Robot {robot_id} home node (sequential): {home_node_seq}")

            # Prepare tasks as Task objects with sequential IDs
            task_objects = []
            for t in self.tasks:
                seq_start = self._convert_original_to_seq(t['start'])
                seq_end = self._convert_original_to_seq(t['end'])

                if seq_start >= self.node_id or seq_end >= self.node_id:
                    self.get_logger().error(
                        f"Task {t['id']}: converted to {seq_start} → {seq_end}, "
                        f"exceeds graph size {self.node_id}"
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
                    f"Task {t['id']}: original {t['start']} → {t['end']}, "
                    f"sequential {seq_start} → {seq_end}"
                )

            if not robots or not task_objects:
                self.get_logger().error('No valid robots or tasks after conversion')
                return

            # Prepare task stream
            tasks_stream = [(task_objects, 0)]
            num_tasks = len(task_objects)
            num_robots = len(robots)

            # FIXED: Calculate minimum required action points using correct formula
            # This matches MRTASolver.get_min_dps()
            min_aps = math.ceil(num_tasks / num_robots) * 2 + 1

            # Create action points list from available nodes
            aps_list = list(range(self.node_id))
            num_aps = aps_list[-1]

            # Validate that we have enough nodes
            if num_aps < min_aps:
                self.get_logger().error(
                    f"Graph only has {num_aps} nodes but needs at least {min_aps} "
                    f"decision points for {num_tasks} tasks and {num_robots} robots"
                )
                return

            self.get_logger().info(
                f'Running SMrTA solver with {num_tasks} tasks, {num_robots} robots, '
                f'min_aps {min_aps}'
            )
            self.get_logger().info(f'Using aps_list: {aps_list} and num_aps: {num_aps}')

            # FIXED: Pass num_aps (not min_aps) and add aps_list parameter
            solver = MRTASolver(
                solver_name=self.solver_name,
                theory=self.theory,
                agents=robots,
                tasks_stream=tasks_stream,
                room_graph=self.graph,
                capacity=self.capacity,
                num_aps=num_aps,        # FIXED: was min_aps
                aps_list=aps_list,      # FIXED: added this parameter
                incremental=True,
                debug=False
            )

            # Handle solver results
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
                self.get_logger().warn(
                    f'  Robots: {num_robots}, Tasks: {num_tasks}, Nodes: {self.node_id}'
                )
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
        try:
            fleet_assignments = FleetAssignments()
            num_agents = len(robots)
            assigned_tasks_overall = set()

            if 'agt' not in solution:
                self.get_logger().warn("No agent assignments found in solver solution")
                return

            for idx, robot_data in enumerate(solution['agt']):
                if idx >= len(robots):
                    self.get_logger().warn(
                        f"Solver solution includes unexpected robot index {idx}"
                    )
                    continue

                robot_id = robots[idx].id
                schedule = robot_data.get('id', [])
                robot_tasks = []
                seen_task_idxs = set()

                self.get_logger().debug(
                    f"Processing assignments for robot {robot_id} with schedule: {schedule}"
                )

                for action_id in schedule:
                    # Skip home positions (action IDs 0 to num_agents-1)
                    if action_id < num_agents:
                        continue

                    # Convert action to task index
                    task_action_offset = action_id - num_agents
                    task_idx = task_action_offset // 2

                    # Validate task index
                    if task_idx >= len(self.tasks):
                        self.get_logger().warn(
                            f"Task index {task_idx} out of range (tasks: {len(self.tasks)})"
                        )
                        continue

                    # FIXED: Removed early continue that was preventing dropoff processing

                    task = self.tasks[task_idx]
                    task_id = task['id']

                    # Handle both pickup (even) and dropoff (odd) actions
                    if task_action_offset % 2 == 0:
                        # Pickup action - add task to robot's list only once
                        if task_idx not in seen_task_idxs:
                            robot_tasks.append({
                                'task_id': task_id,
                                'start': task['start'],
                                'end': task['end']
                            })
                            seen_task_idxs.add(task_idx)
                            assigned_tasks_overall.add(task_id)
                            self.get_logger().info(
                                f"Robot {robot_id} assigned to task {task_id} "
                                f"(start: {task['start']} → end: {task['end']}, "
                                f"action_id {action_id})"
                            )
                    else:
                        # Dropoff action - now properly processed
                        self.get_logger().debug(
                            f"Robot {robot_id} dropoff for task {task_id} "
                            f"(action_id {action_id})"
                        )

                # Create assignment message
                if robot_tasks:
                    assignment = Assignment()
                    assignment.robot_id = str(robot_id)
                    assignment.task_ids = [str(t['task_id']) for t in robot_tasks]

                    fleet_assignments.assignments.append(assignment)

                    self.get_logger().info(
                        f"Robot {robot_id}: assigned {len(robot_tasks)} task(s)"
                    )
                else:
                    self.get_logger().info(
                        f"Robot {robot_id} has no assigned tasks in this solution"
                    )

            # Publish assignments
            self.assignment_pub.publish(fleet_assignments)
            self.get_logger().info(
                f"Published assignments for {len(fleet_assignments.assignments)} robots; "
                f"total assigned tasks: {len(assigned_tasks_overall)}"
            )

        except Exception as e:
            self.get_logger().error(f"Error publishing assignments: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    rclpy.init(args=args)
    node = SMrTaTaskAssignmentNode()

    if SMRTA_AVAILABLE:
        try:
            rclpy.spin(node)
        except KeyboardInterrupt:
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
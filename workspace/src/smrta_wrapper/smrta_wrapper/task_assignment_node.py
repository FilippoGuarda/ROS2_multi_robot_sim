import rclpy
from rclpy.node import Node

# Example: custom messages (replace with your defined message types)
from std_msgs.msg import String

# Import SMrTa
from MRTASolver.objects import Robot, Task
from MRTASolver.MRTASolver import MRTASolver
from MRTASolver.run_realistic_setting import load_weighted_graph, dictionary_to_matrix

import json

class SMrTaTaskAssignmentNode(Node):
    def __init__(self):
        super().__init__('smrta_task_assignment_node')

        # Parameters (could be set via ROS2 params or launch file)
        self.graph_file = 'weighted_graph_p3.pkl'
        self.capacity = 2
        self.num_aps = 5

        # Internal state
        self.robot_states = {}  # {robot_id: position}
        self.tasks = []         # [Task]
        self.graph = None
        self.room_count = None

        # Subscriptions
        self.create_subscription(String, '/fleet/robot_positions', self.positions_callback, 10)
        self.create_subscription(String, '/fleet/tasks', self.tasks_callback, 10)

        # Publisher: assignments to fleet
        self.assignment_pub = self.create_publisher(String, '/smrta/task_assignments', 10)

        # Timer to periodically run SMrTa
        self.timer = self.create_timer(5.0, self.run_smrta)

    def positions_callback(self, msg):
        # Expecting JSON: {"robot1": 1, "robot2": 2, ...}
        self.robot_states = json.loads(msg.data)
        self.get_logger().info(f"Received robot positions: {self.robot_states}")

    def tasks_callback(self, msg):
        # Expecting JSON: [{"id": 1, "start": 2, "end": 5, "deadline": 100}, ...]
        task_dicts = json.loads(msg.data)
        self.tasks = [Task(t['id'], t['start'], t['end'], t.get('deadline')) for t in task_dicts]
        self.get_logger().info(f"Received tasks: {self.tasks}")

    def run_smrta(self):
        # Only proceed if we have robot positions, tasks, and graph
        if not self.robot_states or not self.tasks:
            self.get_logger().info("Waiting for robot positions and tasks...")
            return

        if self.graph is None or self.room_count is None:
            room_dictionary = load_weighted_graph(self.graph_file)
            self.room_count, self.graph = dictionary_to_matrix(room_dictionary)

        # Prepare robots and tasks for SMrTa
        robots = [Robot(i, pos) for i, pos in self.robot_states.items()]
        tasks_stream = [(self.tasks, 0)]  # Just one batch for simplicity

        # Create and run solver
        solver = MRTASolver(
            solver_name='z3',
            theory='QF_UFLIA',
            agents=robots,
            tasks_stream=tasks_stream,
            room_graph=self.graph,
            capacity=self.capacity,
            num_aps=self.num_aps,
            debug=True
        )

        solution = solver.extract_model(solver.s)
        self.publish_assignments(solution)

    def publish_assignments(self, solution):
        # Serialize assignments for publishing.
        # Example: {robot_id: [task_ids], ...}
        assignments = {}
        for robot_id, robot_data in enumerate(solution['agt']):
            assignments[str(robot_id)] = robot_data['id']
        assignment_msg = json.dumps(assignments)
        self.assignment_pub.publish(String(data=assignment_msg))
        self.get_logger().info(f"Published assignments: {assignment_msg}")

def main(args=None):
    rclpy.init(args=args)
    node = SMrTaTaskAssignmentNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
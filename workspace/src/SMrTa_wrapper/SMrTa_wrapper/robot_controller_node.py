import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose

# Import your custom message types
from SMrTA_wrapper.msg import FleetAssignments, Assignment, Task

from geometry_msgs.msg import PoseStamped

class RobotControllerNode(Node):
    def __init__(self):
        super().__init__('robot_controller_node')

        # Parameters
        self.declare_parameter('robot_id', 'robot1')
        self.robot_id = self.get_parameter('robot_id').get_parameter_value().string_value

        # Internal state
        self.current_tasks = []
        self.current_task_index = 0
        self.executing = False

        # Subscribe to assignments for the fleet
        self.assignment_sub = self.create_subscription(
            FleetAssignments,
            '/smrta/task_assignments',
            self.assignment_callback,
            10
        )

        # Action client for navigation
        self.nav_action_client = ActionClient(
            self,
            NavigateToPose,
            f'/{self.robot_id}/navigate_to_pose'
        )

        # Optionally: publisher for task completion feedback
        # self.task_feedback_pub = self.create_publisher(...)

    def assignment_callback(self, msg):
        # Find this robot's assignment
        for assignment in msg.assignments:
            if assignment.robot_id == self.robot_id:
                self.current_tasks = assignment.assigned_task_ids
                self.current_task_index = 0
                self.get_logger().info(f"Received assignments: {self.current_tasks}")
                if not self.executing and self.current_tasks:
                    self.send_next_goal()
                break

    def send_next_goal(self):
        if self.current_task_index >= len(self.current_tasks):
            self.get_logger().info("All assigned tasks completed.")
            self.executing = False
            return

        task_id = self.current_tasks[self.current_task_index]
        # Here you would retrieve the task details (start/end coords)
        # For demo, suppose you have a mapping from task_id to pose
        task_pose = self.lookup_task_pose(task_id)

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = task_pose

        self.get_logger().info(f"Sending navigation goal for task {task_id}")
        self.executing = True
        self.nav_action_client.wait_for_server()
        self._send_goal_future = self.nav_action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.nav_feedback_callback
        )
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def lookup_task_pose(self, task_id):
        # Implement: convert task_id to actual PoseStamped for nav2
        # Could be from a parameter server, a shared map, or a topic
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        # Example coordinates
        pose.pose.position.x = 1.0 * task_id
        pose.pose.position.y = 2.0
        pose.pose.orientation.w = 1.0
        return pose

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self.executing = False
            return
        goal_handle.get_result_async().add_done_callback(self.nav_result_callback)

    def nav_result_callback(self, future):
        result = future.result().result
        if result.error_code == 0:
            self.get_logger().info(f"Navigation succeeded for task {self.current_tasks[self.current_task_index]}")
            # Optionally, publish completion feedback here
            self.current_task_index += 1
            self.executing = False
            self.send_next_goal()
        else:
            self.get_logger().error(f"Navigation failed with code {result.error_code}")
            # Optionally, handle failure (re-try, report, etc.)
            self.executing = False

    def nav_feedback_callback(self, feedback_msg):
        # Optionally, handle feedback (progress updates)
        pass

def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
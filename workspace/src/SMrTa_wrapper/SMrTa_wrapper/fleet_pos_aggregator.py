import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import String

class FleetPositionAggregator(Node):
    def __init__(self):
        super().__init__('fleet_position_aggregator')
        # List of robot names (could be parameterized)
        self.robot_names = ['robot1', 'robot2']  # Extend as needed
        self.positions = {}

        # Subscribers for each robot's pose
        for name in self.robot_names:
            self.create_subscription(
                PoseWithCovarianceStamped,
                f'/{name}/amcl_pose',
                lambda msg, n=name: self.pose_callback(msg, n),
                10
            )

        # Publisher for fleet positions
        self.publisher = self.create_publisher(String, '/fleet/robot_positions', 10)
        self.timer = self.create_timer(1.0, self.publish_positions)

    def pose_callback(self, msg, robot_name):
        self.positions[robot_name] = msg.pose.pose

    def publish_positions(self):
        # This is a placeholder; use a custom message for real implementation
        positions_str = ', '.join(
            f'{name}:({pose.position.x},{pose.position.y})'
            for name, pose in self.positions.items()
        )
        self.publisher.publish(String(data=positions_str))

def main(args=None):
    rclpy.init(args=args)
    node = FleetPositionAggregator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
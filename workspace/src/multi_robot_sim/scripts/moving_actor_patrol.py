
#!/usr/bin/env python3
"""Waypoint patrol node for human actors driven via ActorVelocitySubscriberPlugin.

Reads a humans waypoint YAML (actorN: list of [x, y]) and drives each active
actor through its cyclic waypoint list by publishing geometry_msgs/Twist on
/actorN/cmd_vel. Feedback comes from the nav_msgs/Odometry published by
ActorStatePublisherPlugin on /gazebo/humanN.

Actors that are commented out in the world (no /gazebo/humanN topic) are
skipped silently.

Parameters:
    waypoint_file    Path to YAML mapping actorN -> [[x, y], ...]
    linear_speed     Walking speed in m/s
    goal_tolerance   Waypoint switch distance in m
    kp_yaw           Proportional gain on heading error
    max_yaw_rate     Angular velocity clamp in rad/s
    yaw_offset       Added to the odom yaw before computing heading error.
                     Set to 1.5708 or -1.5708 if actors turn but never walk
                     (or walk sideways).
"""
import math

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


class MovingActorPatrol(Node):

    def __init__(self):
        super().__init__('moving_actor_patrol')

        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('linear_speed', 0.8)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('kp_yaw', 2.0)
        self.declare_parameter('max_yaw_rate', 1.0)
        self.declare_parameter('yaw_offset', 0.0)

        wp_file = self.get_parameter('waypoint_file').value
        if not wp_file:
            raise ValueError('waypoint_file parameter is required')
        with open(wp_file, 'r') as f:
            config = yaml.safe_load(f)
        if not config:
            raise ValueError(f'No actors found in {wp_file}')

        self.v = float(self.get_parameter('linear_speed').value)
        self.tol = float(self.get_parameter('goal_tolerance').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.max_yaw = float(self.get_parameter('max_yaw_rate').value)
        self.yaw_offset = float(self.get_parameter('yaw_offset').value)

        self.actors = {}
        for name, wps in config.items():
            if not isinstance(name, str) or not name.startswith('actor') or not wps:
                continue
            waypoints = [(float(w[0]), float(w[1])) for w in wps]
            odom_topic = f"/gazebo/human{name[len('actor'):]}"
            cmd_topic = f'/{name}/cmd_vel'
            state = {
                'waypoints': waypoints,
                'wp_idx': 0,
                'x': None,
                'y': None,
                'yaw': None,
                'pub': self.create_publisher(Twist, cmd_topic, 10),
            }
            self.create_subscription(
                Odometry, odom_topic,
                lambda msg, s=state: self.on_odom(msg, s), 10)
            self.actors[name] = state
            self.get_logger().info(
                f'{name}: {len(waypoints)} waypoints, cmd {cmd_topic}, '
                f'feedback {odom_topic}')

        self.create_timer(0.05, self.control_step)

    def on_odom(self, msg: Odometry, state: dict):
        if state['x'] is None:
            self.get_logger().info('Actor alive, starting patrol')
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        state['x'] = p.x
        state['y'] = p.y
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        state['yaw'] = math.atan2(siny, cosy)

    def control_step(self):
        for state in self.actors.values():
            if state['yaw'] is None:
                continue

            yaw = state['yaw'] + self.yaw_offset
            waypoints = state['waypoints']
            tx, ty = waypoints[state['wp_idx']]
            dx = tx - state['x']
            dy = ty - state['y']
            dist = math.hypot(dx, dy)

            cmd = Twist()
            if dist < self.tol:
                state['wp_idx'] = (state['wp_idx'] + 1) % len(waypoints)
            else:
                target_yaw = math.atan2(dy, dx)
                err = math.atan2(math.sin(target_yaw - yaw),
                                 math.cos(target_yaw - yaw))
                cmd.angular.z = max(-self.max_yaw,
                                    min(self.max_yaw, self.kp_yaw * err))
                cmd.linear.x = self.v * max(0.0, math.cos(err))
            state['pub'].publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MovingActorPatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
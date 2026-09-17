#!/usr/bin/env python3
"""Waypoint patrol node for a gazebo_ros_planar_move-driven moving obstacle.

Subscribes to /gazebo/model_states for ground-truth pose and publishes
geometry_msgs/Twist on the obstacle's cmd_vel topic to drive it through a
cyclic list of waypoints.

Parameters:
    model_name       Gazebo model name of the obstacle (as named in the world SDF)
    cmd_vel_topic    Full topic of the planar move plugin input
    states_topic     ModelStates topic from gazebo_ros_state
    linear_speed     Forward speed in m/s
    goal_tolerance   Waypoint switch distance in m
    kp_yaw           Proportional gain on heading error
    max_yaw_rate     Angular velocity clamp in rad/s
    waypoints        Flat list [x1, y1, x2, y2, ...] cycled in order
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates


class MovingObstaclePatrol(Node):

    def __init__(self):
        super().__init__('moving_obstacle_patrol')

        self.declare_parameter('model_name', 'moving_obstacle_1')
        self.declare_parameter('cmd_vel_topic', '/moving_obstacle_1/cmd_vel')
        self.declare_parameter('states_topic', '/gazebo/model_states')
        self.declare_parameter('linear_speed', 0.4)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('kp_yaw', 2.0)
        self.declare_parameter('max_yaw_rate', 1.0)
        self.declare_parameter(
            'waypoints',
            [-4.0, 11.0, 4.0, 11.0, 4.0, -11.0, -4.0, -11.0])

        self.model_name = self.get_parameter('model_name').value
        cmd_topic = self.get_parameter('cmd_vel_topic').value
        states_topic = self.get_parameter('states_topic').value
        self.v = float(self.get_parameter('linear_speed').value)
        self.tol = float(self.get_parameter('goal_tolerance').value)
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.max_yaw = float(self.get_parameter('max_yaw_rate').value)
        flat = self.get_parameter('waypoints').value
        if not flat or len(flat) % 2 != 0:
            raise ValueError('waypoints must be a flat list [x1, y1, x2, y2, ...]')
        self.waypoints = [(float(flat[i]), float(flat[i + 1]))
                          for i in range(0, len(flat), 2)]

        self.wp_idx = 0
        self.x = None
        self.y = None
        self.yaw = 0.0

        self.pub = self.create_publisher(Twist, cmd_topic, 10)
        self.create_subscription(ModelStates, states_topic, self.on_states, 10)
        self.create_timer(0.05, self.control_step)

        self.get_logger().info(
            f'Patrolling {self.model_name} through {len(self.waypoints)} waypoints')

    def on_states(self, msg: ModelStates):
        try:
            i = msg.name.index(self.model_name)
        except ValueError:
            return
        p = msg.pose[i].position
        q = msg.pose[i].orientation
        self.x = p.x
        self.y = p.y
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        self.yaw = math.atan2(siny, cosy)

    def control_step(self):
        if self.x is None:
            return

        tx, ty = self.waypoints[self.wp_idx]
        dx = tx - self.x
        dy = ty - self.y
        dist = math.hypot(dx, dy)

        cmd = Twist()
        if dist < self.tol:
            self.wp_idx = (self.wp_idx + 1) % len(self.waypoints)
        else:
            target_yaw = math.atan2(dy, dx)
            err = math.atan2(math.sin(target_yaw - self.yaw),
                             math.cos(target_yaw - self.yaw))
            cmd.angular.z = max(-self.max_yaw,
                                min(self.max_yaw, self.kp_yaw * err))
            # Cosine ramp instead of a hard gate: always creeps forward when
            # roughly aligned, so the obstacle never spins in place waiting
            # for |err| < 0.5 rad.
            cmd.linear.x = self.v * max(0.0, math.cos(err))
        self.pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = MovingObstaclePatrol()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
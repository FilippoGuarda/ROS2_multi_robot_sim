#!/usr/bin/env python3
"""Spawn vertical cylinders and patrol them along waypoint routes.

Each cylinder is spawned at the first [x, y] waypoint from the YAML file.
The SDF contains no world translation; the spawn service applies the initial
pose exactly once through SpawnEntity.initial_pose.

The models remain dynamic and collidable, while gravity, contact friction,
and velocity damping are disabled. Motion is commanded through the Gazebo ROS
planar-move plugin.
"""

import math
import subprocess

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, Twist
from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SpawnEntity


SDF_TEMPLATE = """<?xml version="1.0"?>
<sdf version="1.6">0
  <model name="{name}">
    <static>false</static>
    <self_collide>false</self_collide>
    <allow_auto_disable>false</allow_auto_disable>
    <gravity>false</gravity>

    <!-- The world pose is supplied exactly once through SpawnEntity.initial_pose. -->
    <pose>0 0 0 0 0 0</pose>

    <link name="link">
      <gravity>false</gravity>
      <kinematic>false</kinematic>

      <inertial>
        <mass>{mass}</mass>
        <inertia>
          <ixx>{ixx}</ixx>
          <iyy>{iyy}</iyy>
          <izz>{izz}</izz>
          <ixy>0</ixy>
          <ixz>0</ixz>
          <iyz>0</iyz>
        </inertia>
      </inertial>

      <collision name="collision">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>

        <surface>
          <friction>
            <ode>
              <mu>0.0</mu>
              <mu2>0.0</mu2>
              <slip1>1.0</slip1>
              <slip2>1.0</slip2>
            </ode>
          </friction>
          <contact>
            <ode>
              <kp>1000000.0</kp>
              <kd>1.0</kd>
              <max_vel>0.0</max_vel>
              <min_depth>0.001</min_depth>
            </ode>
          </contact>
        </surface>
      </collision>

      <visual name="visual">
        <geometry>
          <cylinder>
            <radius>{radius}</radius>
            <length>{height}</length>
          </cylinder>
        </geometry>
        <material>
          <ambient>1.0 0.5 0.0 1</ambient>
          <diffuse>1.0 0.5 0.0 1</diffuse>
        </material>
      </visual>

      <velocity_decay>
        <linear>0.0</linear>
        <angular>0.0</angular>
      </velocity_decay>
    </link>

    <plugin name="{name}_planar_move"
            filename="libgazebo_ros_planar_move.so">
      <ros>
        <namespace>/{name}</namespace>
      </ros>
      <update_rate>50</update_rate>
      <publish_odom>true</publish_odom>
      <publish_odom_tf>false</publish_odom_tf>
      <covariance_x>0.0001</covariance_x>
      <covariance_y>0.0001</covariance_y>
      <covariance_yaw>0.01</covariance_yaw>
    </plugin>
  </model>
</sdf>"""


def cylinder_inertia(mass, radius, height):
    """Return inertia tensor terms for a solid cylinder aligned with z."""
    ixx = mass * (3.0 * radius * radius + height * height) / 12.0
    izz = 0.5 * mass * radius * radius
    return ixx, ixx, izz


class MovingObstacleSpawner(Node):
    def __init__(self):
        super().__init__('moving_obstacle_spawner')

        self.declare_parameter('waypoint_file', '')
        self.declare_parameter('prefix', 'moving_obstacle')
        self.declare_parameter('radius', 0.25)
        self.declare_parameter('height', 1.0)
        self.declare_parameter('mass', 20.0)
        self.declare_parameter('linear_speed', 0.4)
        self.declare_parameter('goal_tolerance', 0.3)
        self.declare_parameter('kp_yaw', 2.0)
        self.declare_parameter('max_yaw_rate', 1.0)

        waypoint_file = self.get_parameter('waypoint_file').value
        if not waypoint_file:
            raise ValueError('waypoint_file parameter is required')

        with open(waypoint_file, 'r', encoding='utf-8') as file:
            config = yaml.safe_load(file)

        if not isinstance(config, dict) or not config:
            raise ValueError(f'No entries found in {waypoint_file}')

        self.prefix = str(self.get_parameter('prefix').value)
        self.radius = float(self.get_parameter('radius').value)
        self.height = float(self.get_parameter('height').value)
        self.mass = float(self.get_parameter('mass').value)
        self.speed = float(self.get_parameter('linear_speed').value)
        self.goal_tolerance = float(
            self.get_parameter('goal_tolerance').value
        )
        self.kp_yaw = float(self.get_parameter('kp_yaw').value)
        self.max_yaw_rate = float(
            self.get_parameter('max_yaw_rate').value
        )

        if self.radius <= 0.0:
            raise ValueError('radius must be greater than zero')
        if self.height <= 0.0:
            raise ValueError('height must be greater than zero')
        if self.mass <= 0.0:
            raise ValueError('mass must be greater than zero')

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.obstacles = {}

        for key, waypoint_list in config.items():
            if not waypoint_list:
                continue
            if not isinstance(waypoint_list, list):
                raise ValueError(
                    f'Waypoints for {key} must be a list of [x, y] pairs'
                )

            waypoints = []
            for waypoint in waypoint_list:
                if not isinstance(waypoint, (list, tuple)) or len(waypoint) < 2:
                    raise ValueError(
                        f'Invalid waypoint for {key}: {waypoint!r}'
                    )
                waypoints.append(
                    (float(waypoint[0]), float(waypoint[1]))
                )

            name = f'{self.prefix}_{key}'
            self.obstacles[name] = {
                'waypoints': waypoints,
                'wp_idx': 0,
                'x': None,
                'y': None,
                'yaw': None,
                'pub': self.create_publisher(
                    Twist,
                    f'/{name}/cmd_vel',
                    10,
                ),
            }
            self.get_logger().info(
                f'{name}: {len(waypoints)} waypoints; '
                f'initial target=({waypoints[0][0]:.3f}, '
                f'{waypoints[0][1]:.3f})'
            )

        if not self.obstacles:
            raise ValueError('No valid obstacle waypoint lists found')

        self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.on_states,
            10,
        )
        self.create_timer(0.05, self.control_step)
        self.create_timer(5.0, self.disable_gravity_all)

    def disable_gravity_all(self):
        """Reapply the gravity setting for dynamically spawned models."""
        for name in self.obstacles:
            try:
                subprocess.run(
                    [
                        'gz',
                        'model',
                        '-m',
                        name,
                        '--gravity-mode',
                        'false',
                    ],
                    capture_output=True,
                    timeout=2.0,
                    check=False,
                )
            except Exception as exc:
                self.get_logger().debug(
                    f'Could not disable gravity for {name}: {exc}'
                )

    def spawn_all(self):
        if not self.spawn_client.wait_for_service(timeout_sec=60.0):
            self.get_logger().error(
                '/spawn_entity unavailable. Is gzserver running with '
                'libgazebo_ros_factory.so?'
            )
            return

        ixx, iyy, izz = cylinder_inertia(
            self.mass,
            self.radius,
            self.height,
        )

        for name, state in self.obstacles.items():
            sx, sy = state['waypoints'][0]

            # The cylinder is centered at z=height/2, placing its bottom at z=0.
            sz = self.height / 2.0 + 0.05

            self.get_logger().info(
                f'{name} initial YAML waypoint: '
                f'x={sx:.3f}, y={sy:.3f}, z={sz:.3f}'
            )

            # Do not put x, y, or z in the SDF. The spawn service applies the
            # world pose exactly once using request.initial_pose.
            sdf = SDF_TEMPLATE.format(
                name=name,
                radius=self.radius,
                height=self.height,
                mass=self.mass,
                ixx=ixx,
                iyy=iyy,
                izz=izz,
            )

            request = SpawnEntity.Request()
            request.name = name
            request.xml = sdf
            request.reference_frame = 'world'

            request.initial_pose = Pose()
            request.initial_pose.position.x = sx
            request.initial_pose.position.y = sy
            request.initial_pose.position.z = sz
            request.initial_pose.orientation.w = 1.0

            future = self.spawn_client.call_async(request)
            rclpy.spin_until_future_complete(self, future)
            response = future.result()

            if response is not None and response.success:
                self.get_logger().info(
                    f'Spawned {name} at YAML waypoint '
                    f'({sx:.3f}, {sy:.3f}, {sz:.3f})'
                )
                self.disable_gravity_all()
            else:
                status = response.status_message if response else 'no response'
                self.get_logger().error(
                    f'Failed to spawn {name}: {status}'
                )

    def on_states(self, message: ModelStates):
        for index, name in enumerate(message.name):
            state = self.obstacles.get(name)
            if state is None:
                continue

            pose = message.pose[index]
            state['x'] = pose.position.x
            state['y'] = pose.position.y

            q = pose.orientation
            sin_yaw = 2.0 * (q.w * q.z + q.x * q.y)
            cos_yaw = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            state['yaw'] = math.atan2(sin_yaw, cos_yaw)

    def control_step(self):
        for name, state in self.obstacles.items():
            if state['x'] is None or state['y'] is None:
                continue
            if state['yaw'] is None:
                continue

            tx, ty = state['waypoints'][state['wp_idx']]
            dx = tx - state['x']
            dy = ty - state['y']
            distance = math.hypot(dx, dy)

            if distance < self.goal_tolerance:
                state['wp_idx'] = (
                    state['wp_idx'] + 1
                ) % len(state['waypoints'])
                tx, ty = state['waypoints'][state['wp_idx']]
                dx = tx - state['x']
                dy = ty - state['y']

            target_yaw = math.atan2(dy, dx)
            yaw_error = math.atan2(
                math.sin(target_yaw - state['yaw']),
                math.cos(target_yaw - state['yaw']),
            )

            command = Twist()
            command.angular.z = max(
                -self.max_yaw_rate,
                min(self.max_yaw_rate, self.kp_yaw * yaw_error),
            )
            command.linear.x = self.speed * max(
                0.0,
                math.cos(yaw_error),
            )
            state['pub'].publish(command)


def main(args=None):
    rclpy.init(args=args)
    node = MovingObstacleSpawner()

    try:
        node.spawn_all()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
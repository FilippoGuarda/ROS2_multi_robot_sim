"""
Copyright 2025 Filippo Guarda
Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at
    http://www.apache.org/licenses/LICENSE-2.0
Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.




WARNING: The namespacing in ROS2 is particularly finnicky, especially around
the nav2 stack, before trying to rewrite parts of this launch file refer to:
https://github.com/ros-navigation/navigation2/issues/2796
https://github.com/ros-navigation/navigation2/blob/galactic/nav2_bringup/bringup/launch/bringup_launch.py#L80-L82

"""


import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    multi_robot_sim_dir = get_package_share_directory('multi_robot_sim')
    
    # LaunchConfigurations - keep as Substitution objects
    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    initial_pose_x = LaunchConfiguration('initial_pose_x')
    initial_pose_y = LaunchConfiguration('initial_pose_y')
    initial_pose_yaw = LaunchConfiguration('initial_pose_yaw')
    other_robot_namespaces = LaunchConfiguration('other_robot_namespaces')
    
    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='robot1', description='Top-level namespace')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Use simulation clock')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(multi_robot_sim_dir, 'config', 'nav2_params.yaml'),
        description='Path to params file')
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart', default_value='true', description='Auto start nav2 stack')
    
    declare_initial_pose_x_cmd = DeclareLaunchArgument(
        'initial_pose_x', default_value='0.0')
    declare_initial_pose_y_cmd = DeclareLaunchArgument(
        'initial_pose_y', default_value='0.0')
    declare_initial_pose_yaw_cmd = DeclareLaunchArgument(
        'initial_pose_yaw', default_value='0.0')
    
    declare_other_robots_cmd = DeclareLaunchArgument(
        'other_robot_namespaces', default_value='[]', 
        description='JSON list of other robots')
    
    # RewrittenYaml created with LaunchConfiguration objects
    param_substitutions = {
        'use_sim_time': use_sim_time,
        'robot_base_frame': [namespace, '/base_footprint'],
        'odom_frame_id': [namespace, '/odom'],
        'base_frame_id': [namespace, '/base_footprint'],
        'global_frame': [namespace, '/odom'],
        'topic': ['/', namespace, '/scan'],
        'costmap_topic': [namespace, '/local_costmap/costmap_raw'],
        'footprint_topic': [namespace, '/local_costmap/published_footprint'],
        'robot_namespace': namespace
    }
    
    configured_params = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions,
        convert_types=True)
    
    # Create a different param substitution set since the global costmap
    # uses map as the global frame
    param_substitutions_global_cost = param_substitutions.copy()
    param_substitutions_global_cost['global_frame'] = 'map'
    configured_params_global_cost = RewrittenYaml(
        source_file=params_file,
        root_key=namespace,
        param_rewrites=param_substitutions_global_cost,
        convert_types=True)


    remappings = [('map', '/map')]

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            configured_params,
            {
                'set_initial_pose': True,
                'initial_pose.x': initial_pose_x,
                'initial_pose.y': initial_pose_y,
                'initial_pose.yaw': initial_pose_yaw
            }
        ],
        remappings=remappings)
    
    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params_global_cost],
        remappings=remappings)
    
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params_global_cost],
        remappings=remappings)
    
    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params_global_cost],
        remappings=remappings)
    
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'autostart': autostart,
            'node_names': [
                'amcl',
                'controller_server',
                'planner_server',
                'behavior_server',
                'bt_navigator'
            ]
        }])
    
    # OpaqueFunction ONLY for controller_server (needs dynamic tracked_agents)
    def create_controller_server(context):
        other_robots_json = context.launch_configurations.get('other_robot_namespaces', '[]')
        
        try:
            other_namespaces = json.loads(other_robots_json)
        except (json.JSONDecodeError, TypeError):
            other_namespaces = []
        
        tracked_agents = [f"{ns}/base_footprint" for ns in other_namespaces]
        
        return [Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[
                configured_params,
                {
                    'FollowPath.InteractionAwareCritic.plugin': "mppi_multi_robot_critic/InteractionAwareCritic",
                    'FollowPath.InteractionAwareCritic.tracked_agents': tracked_agents
                }
            ],
            remappings=remappings)]
    
    ld = LaunchDescription()
    
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_initial_pose_x_cmd)
    ld.add_action(declare_initial_pose_y_cmd)
    ld.add_action(declare_initial_pose_yaw_cmd)
    ld.add_action(declare_other_robots_cmd)
    
    nav2_bringup_group = GroupAction([
        PushRosNamespace(namespace=namespace),
        
        # Add all the nodes that need to be namespaced
        amcl_node,
        OpaqueFunction(function=create_controller_server),
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        lifecycle_manager_node
    ])
    
    ld.add_action(nav2_bringup_group)
    
    return ld

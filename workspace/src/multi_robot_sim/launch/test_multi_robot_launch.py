import os
import sys
import argparse
import json

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.conditions import IfCondition

ROBOT_POSITIONS = []

def parse_arguments(argv):
    """Parse launch arguments"""
    args = []
    for arg in argv:
        if ":=" in arg:
            parsed_arg = arg.split(':')
            if not parsed_arg[0] == 'use_sim_time' and not parsed_arg[0] == 'autostart':
                args.append(f"-{parsed_arg[0]}")
                args.append(parsed_arg[1][1:])
    return args

def get_scenario_file_from_arguments(arguments):
    """Parse scenario file from command line arguments"""
    parser = argparse.ArgumentParser(description='Start multi-robot hospital simulation')
    parser.add_argument('-input_file', type=str, help='Scenario file', required=True)
    args = parser.parse_args(arguments)
    return args.input_file


def get_robot_positions(file):
    """Load robot starting positions from scenario file"""
    with open(file, 'r') as f:
        scenario_setup = json.load(f) 
    positions = []
    for robot in scenario_setup["agents"].values():
        print(f"Robot: {robot}")
        position_dict = {
            'x': float(robot["start"][0]),
            'y': float(robot["start"][1]),
            # Handle cases where yaw might not be included in the JSON.
            'yaw': float(robot["start"]) if len(robot["start"]) > 2 else 0.0
        }
        
        # Append the correctly formatted dictionary to the list.
        positions.append(position_dict)

    return positions

def generate_robot_launches(context):
    """Generate launch includes for each robot"""
    global ROBOT_POSITIONS
    
    namespaces_str = context.launch_configurations.get('namespaces', '')
    namespaces = [ns.strip() for ns in namespaces_str.split(',') if ns.strip()]
    
    multi_robot_sim_dir = get_package_share_directory('multi_robot_sim')
    
    robot_launches = []
    for i, namespace in enumerate(namespaces):
        # Get initial pose for this robot (default to origin if not available)
        
        try:
            initial_pose = ROBOT_POSITIONS[i]
        except (IndexError, KeyError):
            error_msg = f"Missing position input for robot {i} in namespace '{namespace}'. Use input_file:=</positions/file/path.json> at launch"
            print(f"[ERROR] {error_msg}")
            sys.exit(1) 

        robot_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(multi_robot_sim_dir, 'launch', 'single_robot_nav_launch.py')
            ),
            launch_arguments={
                'namespace': namespace,
                'use_sim_time': LaunchConfiguration('use_sim_time').perform(context),
                'params_file': LaunchConfiguration('params_file').perform(context),
                'autostart': LaunchConfiguration('autostart').perform(context),
                'initial_pose_x': str(initial_pose['x']),
                'initial_pose_y': str(initial_pose['y']),
                'initial_pose_yaw': str(initial_pose['yaw']),
            }.items()
        )
        robot_launches.append(robot_launch)
    
    return robot_launches

def generate_launch_description():
    global ROBOT_POSITIONS
    
    # Parse scenario file if provided
    arguments = parse_arguments(sys.argv)
    robot_names = ["robot1", "robot2", "robot3", "robot4", "robot5", "robot6"]  # Default
    
    if arguments:
        try:
            scenario_file = get_scenario_file_from_arguments(arguments)
            ROBOT_POSITIONS = get_robot_positions(scenario_file)
            robot_names = [f'robot{i + 1}' for i in range(len(ROBOT_POSITIONS))]
            print(f"Loaded {len(ROBOT_POSITIONS)} robot positions from scenario file")
        except Exception as e:
            print(f"Could not parse scenario file: {e}, using default robot names and positions")
            ROBOT_POSITIONS = []
    
    # Package directories
    multi_robot_sim_dir = get_package_share_directory('multi_robot_sim')
    multi_robot_sim_config_dir = os.path.join(multi_robot_sim_dir, 'configs')
    mrgc_pkg_dir = get_package_share_directory('multi_robot_costmap_plugin')
    
    # Launch configuration variables
    map_yaml_file = LaunchConfiguration('map')
    use_sim_time = LaunchConfiguration('use_sim_time')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    rviz_config_file = LaunchConfiguration('rviz_config_file')
    use_rviz = LaunchConfiguration('use_rviz')
    use_simulator = LaunchConfiguration('use_simulator')
    headless = LaunchConfiguration('headless')
    mrgc_config_file = LaunchConfiguration('mrgc_config_file')
    
    # Declare launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespaces',
        default_value=','.join(robot_names),
        description='Comma-separated list of robot namespaces'
    )
    
    declare_map_yaml_cmd = DeclareLaunchArgument(
        'map',
        default_value=os.path.join(multi_robot_sim_dir, 'worlds', 'map_aws', 'my_map.yaml'),
        description='Full path to map file to load'
    )
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='True',
        description='Use simulation (Gazebo) clock if true'
    )
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(multi_robot_sim_config_dir, 'nav2_params.yaml'),
        description='Full path to the ROS2 parameters file to use for all launched nodes'
    )
    
    declare_autostart_cmd = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically startup the nav2 stack'
    )
    
    declare_rviz_config_file_cmd = DeclareLaunchArgument(
        'rviz_config_file',
        default_value=os.path.join(multi_robot_sim_config_dir, 'shared_costmap_rviz.rviz'),
        description='Full path to the RVIZ config file to use'
    )
    
    declare_use_rviz_cmd = DeclareLaunchArgument(
        'use_rviz',
        default_value='True',
        description='Whether to start RVIZ'
    )
    
    declare_use_simulator_cmd = DeclareLaunchArgument(
        'use_simulator',
        default_value='True',
        description='Whether to start Gazebo simulator'
    )
    
    declare_headless_cmd = DeclareLaunchArgument(
        'headless',
        default_value='False',
        description='Run Gazebo in headless mode'
    )

    declare_mrgc_config_file_cmd = DeclareLaunchArgument(
        'mrgc_config_file',
        default_value=os.path.join(mrgc_pkg_dir, 'config', 'multi_robot_costmap.yaml'),
        description='Path to config file'
    )
    
    # Set environment variable for TurtleBot3
    env_cmd = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle')
    
    # Common nodes - Map server (single, not namespaced)
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'yaml_filename': map_yaml_file
        }]
    )

    fusion_node = Node(
        package='multi_robot_costmap_plugin',
        executable='global_costmap_fusion_node',
        name='global_costmap_fusion',
        output='screen',
        parameters=[
            LaunchConfiguration('mrgc_config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        remappings=[
        ]
    )
    
    # Lifecycle manager for map server
    map_lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'autostart': ParameterValue(autostart, value_type=bool),
            'node_names': ['map_server']
        }]
    )
    
    # RViz
    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        condition=IfCondition(use_rviz)
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_rviz_config_file_cmd)
    ld.add_action(declare_use_rviz_cmd)
    ld.add_action(declare_use_simulator_cmd)
    ld.add_action(declare_headless_cmd)
    ld.add_action(declare_mrgc_config_file_cmd)
    
    # Add environment variable
    ld.add_action(env_cmd)
    
    # Add common nodes
    ld.add_action(map_server_node)
    ld.add_action(map_lifecycle_manager)
    # ld.add_action(fusion_node)
    
    # Add individual robot launches
    ld.add_action(OpaqueFunction(function=generate_robot_launches))
    
    # Add RViz
    ld.add_action(rviz_cmd)
    
    return ld

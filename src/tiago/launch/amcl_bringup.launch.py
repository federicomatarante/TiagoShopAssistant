import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    tiago_package_dir = get_package_share_directory('tiago') # Your package name
    map_file_path = os.path.join(tiago_package_dir, 'maps', 'my_map.yaml') # Your map path
    amcl_params_path = os.path.join(tiago_package_dir,'config', 'amcl_params.yaml') # Path to your AMCL params

    use_sim_time = LaunchConfiguration('use_sim_time')
    
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')

    # Node for map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file_path}])

    # Node for map_server lifecycle manager (required by Nav2)
    lifecycle_manager_map_server = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map_server',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['map_server']}])

    # Node for AMCL - ADD EXPLICIT REMAPPING
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[amcl_params_path, {'use_sim_time': use_sim_time}],
        remappings=[('scan', 'scan_raw')]  # ADD THIS LINE
    )

    # Node for amcl lifecycle manager
    lifecycle_manager_amcl = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_amcl',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time},
                    {'autostart': True},
                    {'node_names': ['amcl']}])

    return LaunchDescription([
        declare_use_sim_time_cmd,
        map_server_node,
        lifecycle_manager_map_server,
        amcl_node,
        lifecycle_manager_amcl
    ])
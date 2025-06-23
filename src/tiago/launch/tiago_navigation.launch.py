import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    # Get the share directory for your tiago package
    tiago_share_dir = get_package_share_directory('tiago')
    
    # Get the share directory for the nav2_bringup package
    nav2_bringup_share_dir = get_package_share_directory('nav2_bringup')

    # --- Declare the path to your map file ---
    # IMPORTANT: This assumes your map file is in 'tiago/maps/my_map.yaml'
    map_file_path = os.path.join(tiago_share_dir, 'maps', 'my_map.yaml')

    # --- AMCL (Localization) ---
    # Include the localization_launch.py from nav2_bringup
    amcl_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share_dir, 'launch', 'localization_launch.py')
        ),
        launch_arguments={'map': map_file_path}.items()
    )

    # --- RViz ---
    # Start RViz with a configuration file
    rviz_config_file = os.path.join(tiago_share_dir, 'rviz', 'nav2_config.rviz') # Assuming you have a config file here
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        # arguments=['-d', rviz_config_file],
        output='screen'
    )

    return LaunchDescription([
        amcl_launch,
        rviz_node
    ])
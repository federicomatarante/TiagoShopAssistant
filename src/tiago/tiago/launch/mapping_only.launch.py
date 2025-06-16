import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Robot State Publisher (for the mapping robot)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'use_sim_time': True,
            'robot_description': open(os.path.expanduser('~/TiagoShopAssistant/src/tiago/tiago/models/Robot/model.sdf')).read()
        }]
    )

    # Spawn the mapping robot
    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', os.path.expanduser('~/TiagoShopAssistant/src/tiago/tiago/models/Robot/model.sdf'),
            '-entity', 'mapping_robot',
            '-x', '0', '-y', '0', '-z', '0.1'
        ],
        parameters=[{'use_sim_time': True}]
    )

    # SLAM Toolbox
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch'),
            '/online_async_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': 'true'
        }.items()
    )

    return LaunchDescription([
        robot_state_publisher,
        spawn_robot,
        slam_toolbox,
    ])
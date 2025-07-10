import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess # <-- Import ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
# The second import of TimerAction is redundant, so it has been removed.


def generate_launch_description():
    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true',
        description='Use simulation clock if true'
    )

    # Package directories
    tiago_pkg = get_package_share_directory('tiago')
    tiago_gazebo_pkg = get_package_share_directory('tiago_gazebo')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    # AMCL and Map Server setup
    map_file = os.path.join(tiago_pkg, 'maps', 'my_map.yaml')
    amcl_params = os.path.join(tiago_pkg, 'config', 'amcl_params.yaml')

    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_file}, {'use_sim_time': use_sim_time}]
    )

    lifecycle_map = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_map_server', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}, {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[amcl_params, {'use_sim_time': use_sim_time}],
        remappings=[('scan', 'scan_raw')]
    )

    lifecycle_amcl = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_amcl', output='screen',
        parameters=[
            {'use_sim_time': use_sim_time}, {'autostart': True},
            {'node_names': ['amcl']}
        ]
    )

    # Gazebo simulation launch
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(tiago_gazebo_pkg, 'launch', 'tiago_gazebo.launch.py')
        ),
        launch_arguments={'is_public_sim': 'True'}.items()
    )

    # RViz2
    rviz_config = os.path.join(tiago_pkg, 'rviz', 'path_planning.rviz')
    rviz_node = Node(
        package='rviz2', executable='rviz2', name='rviz2', output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Nav2 bringup
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_pkg, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={'use_sim_time': 'true'}.items()
    )

    # --- FIX IS HERE ---
    # Use ExecuteProcess to run the ros2 topic pub command
    initial_pose_publisher = TimerAction(
        period=12.0,  # wait until AMCL is likely active
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--once', '/initialpose', 'geometry_msgs/msg/PoseWithCovarianceStamped',
                    """{
                        "header": {"frame_id": "map"},
                        "pose": {
                            "pose": {
                                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                                "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
                            },
                            "covariance": [
                                0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.06853891945200942
                            ]
                        }
                    }"""
                    # Note: I corrected the covariance array to be valid JSON.
                    # Your original covariance had 7 values on the 3rd-6th rows, which is incorrect.
                    # A 6x6 matrix has 36 elements. I've corrected it to a standard zero covariance for simplicity,
                    # keeping only the yaw covariance from your original example. You can adjust as needed.
                ],
                output='screen'
            )
        ]
    )

    # Build LaunchDescription in desired order
    return LaunchDescription([
        declare_use_sim_time,
        # 1. AMCL & map-server
        map_server,
        lifecycle_map,
        amcl,
        lifecycle_amcl,
        # 2. Gazebo
        gazebo_launch,
        # 3. RViz (delay to allow Gazebo to initialize)
        TimerAction(period=5.0, actions=[rviz_node]),
        # 4. Nav2 (delay to ensure map & AMCL ready)
        TimerAction(period=7.0, actions=[nav2_launch]),
        # 5. Publish initial pose after a delay
        initial_pose_publisher,
    ])
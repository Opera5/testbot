from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    testbot_dir = get_package_share_directory('testbot')

    # Launch configuration
    map_file = LaunchConfiguration('map')

    return LaunchDescription([
        # Declare map parameter
        DeclareLaunchArgument(
            'map',
            default_value=os.path.join(testbot_dir, 'maps', 'testhus_map.yaml'),
            description='Full path to map yaml file'),

        # Launch Gazebo with your robot
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(testbot_dir, 'launch', '4w_rsp.launch.py')
            ),
            launch_arguments={'use_sim_time': 'true'}.items(),
        ),

        # Map server
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='mapserver',
            output='screen',
            parameters=[{'yaml_filename': map_file,
                         'use_sim_time': True}]
        ),

        # AMCL
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
                        {'use_sim_time': True}]
        ),

        # Lifecycle Manager
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{'use_sim_time': True},
                        {'autostart': True},
                        {'node_names': ['mapserver', 'amcl']}]
        ),

        # RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(testbot_dir, 'rviz', 'amcl.rviz')],
            parameters=[{'use_sim_time': True}]
        ),
    ])

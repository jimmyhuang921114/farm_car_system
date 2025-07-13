from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='obs_filiter',
            executable='realsense_read_node',
            name='realsense_read_node',
            output='screen'
        ),
        Node(
            package='obs_filiter',
            executable='plane_node',
            name='plane_node',
            output='screen'
        ),
        Node(
            package='obs_filiter',
            executable='obstacle_node',
            name='obstacle_node',
            output='screen'
        ),
        Node(
            package='obs_filiter',
            executable='xz_visualizer_node',
            name='xz_visualizer_node',
            output='screen'
        ),
    ])

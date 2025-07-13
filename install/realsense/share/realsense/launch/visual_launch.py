from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense',
            executable='color_range',
            name='color_range_node',
            output='screen'
        ),
        Node(
            package='realsense',
            executable='depth_calculate',
            name='depth_calculate_node',
            output='screen'
        ),
        Node(
            package='realsense',
            executable='paddleorc',
            name='paddleorc_node',
            output='screen'
        ),
        Node(
            package='realsense',
            executable='realsense_read',
            name='realsense_read_node',
            output='screen'
        )
    ])

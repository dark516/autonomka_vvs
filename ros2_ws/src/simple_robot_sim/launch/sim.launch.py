from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='simple_robot_sim',
            executable='simulator_node',
            name='simulator',
            output='screen',
            parameters=[{
                'field_size': 10.0,
                'window_size': 1000,
                'publish_rate': 30.0
            }]
        )
        # data_publisher_node больше не нужен
    ])

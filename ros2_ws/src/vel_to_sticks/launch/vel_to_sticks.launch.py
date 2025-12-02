from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='vel_to_sticks',
            executable='vel_to_sticks',
            name='vel_to_sticks',
            parameters=[{
                'serial_port': '/dev/ttyACM0',
                'baud_rate': 115200,
                'send_frequency': 50.0
            }],
            output='screen'
        )
    ])

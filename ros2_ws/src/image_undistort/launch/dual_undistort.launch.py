import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    # Undistortion node for camera_1
    undistort_camera_1 = Node(
        package='image_undistort',
        executable='undistort_node',
        name='undistort_camera_1',
        parameters=[{
            'input_topic': '/camera_1/image_raw',
            'output_topic': '/camera_1/image_undistorted',
            'calibration_file': '/home/sverk/stilex_110/src/image_undistort/calibration/camera_1_calibration.npz',  # Update with your actual path
            'output_width': 1920,
            'output_height': 1080
        }],
        output='screen'
    )
    
    # Undistortion node for camera_2
    undistort_camera_2 = Node(
        package='image_undistort',
        executable='undistort_node',
        name='undistort_camera_2',
        parameters=[{
            'input_topic': '/camera_2/image_raw',
            'output_topic': '/camera_2/image_undistorted',
            'calibration_file': '/home/sverk/stilex_110/src/image_undistort/calibration/camera_2_calibration.npz',  # Update with your actual path
            'output_width': 1920,
            'output_height': 1080
        }],
        output='screen'
    )
    
    ld.add_action(undistort_camera_1)
    ld.add_action(undistort_camera_2)
    
    return ld
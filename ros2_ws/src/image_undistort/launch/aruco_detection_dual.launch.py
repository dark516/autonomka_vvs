import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    package_share = get_package_share_directory('image_undistort')
    
    # ArUco detector for camera_1
    aruco_detector_1 = Node(
        package='image_undistort',
        executable='aruco_detector_single',
        name='aruco_detector_camera_1',
        parameters=[{
            'camera_name': 'camera_1',
            'input_image_topic': '/camera_1/image_undistorted',
            'output_pose_topic': '/camera_1/aruco_poses',
            'debug_image_topic': '/camera_1/aruco_debug',
            'marker_size': 0.05,  # 5cm markers - adjust to your actual marker size
            'calibration_file': os.path.join(package_share, 'calibration', 'camera_1_calibration.npz'),
        }],
        output='screen'
    )
    
    # ArUco detector for camera_2
    aruco_detector_2 = Node(
        package='image_undistort',
        executable='aruco_detector_single',
        name='aruco_detector_camera_2',
        parameters=[{
            'camera_name': 'camera_2',
            'input_image_topic': '/camera_2/image_undistorted',
            'output_pose_topic': '/camera_2/aruco_poses',
            'debug_image_topic': '/camera_2/aruco_debug',
            'marker_size': 0.05,  # 5cm markers - adjust to your actual marker size
            'calibration_file': os.path.join(package_share, 'calibration', 'camera_2_calibration.npz'),
        }],
        output='screen'
    )
    
    ld.add_action(aruco_detector_1)
    ld.add_action(aruco_detector_2)
    
    return ld
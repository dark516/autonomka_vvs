import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    ld = LaunchDescription()
    
    package_share = get_package_share_directory('image_undistort')
    
    # Static transforms
    tf_camera1 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera1_tf_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_1_optical_frame']
    )
    
    tf_camera2 = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera2_tf_publisher',
        arguments=['0.5', '0', '0', '0', '0', '0', 'map', 'camera_2_optical_frame']
    )
    
    # Undistortion nodes
    undistort_camera_1 = Node(
        package='image_undistort',
        executable='undistort_node',
        name='undistort_camera_1',
        parameters=[{
            'input_topic': '/camera_1/image_raw',
            'output_topic': '/camera_1/image_undistorted',
            'calibration_file': os.path.join(package_share, 'calibration', 'camera_1_calibration.npz'),
            'output_width': 1920,
            'output_height': 1080
        }]
    )
    
    undistort_camera_2 = Node(
        package='image_undistort',
        executable='undistort_node',
        name='undistort_camera_2',
        parameters=[{
            'input_topic': '/camera_2/image_raw',
            'output_topic': '/camera_2/image_undistorted',
            'calibration_file': os.path.join(package_share, 'calibration', 'camera_2_calibration.npz'),
            'output_width': 1920,
            'output_height': 1080
        }]
    )
    
    # ArUco detection nodes
    aruco_detector_1 = Node(
        package='image_undistort',
        executable='aruco_detector_single',
        name='aruco_detector_camera_1',
        parameters=[{
            'camera_name': 'camera_1',
            'input_image_topic': '/camera_1/image_raw',
            'output_pose_topic': '/camera_1/aruco_poses',
            'debug_image_topic': '/camera_1/aruco_debug',
            'marker_size': 0.05,
            'aruco_to_find': -1  # Detect all markers
        }]
    )
    
    aruco_detector_2 = Node(
        package='image_undistort',
        executable='aruco_detector_single',
        name='aruco_detector_camera_2',
        parameters=[{
            'camera_name': 'camera_2',
            'input_image_topic': '/camera_2/image_undistorted',
            'output_pose_topic': '/camera_2/aruco_poses',
            'debug_image_topic': '/camera_2/aruco_debug',
            'marker_size': 0.05,
            'aruco_to_find': -1  # Detect all markers
        }]
    )
    
    ld.add_action(tf_camera1)
    ld.add_action(tf_camera2)
    ld.add_action(undistort_camera_1)
    ld.add_action(undistort_camera_2)
    ld.add_action(aruco_detector_1)
    ld.add_action(aruco_detector_2)
    
    return ld

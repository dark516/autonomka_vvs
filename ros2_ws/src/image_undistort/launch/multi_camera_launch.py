#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    # Get package share directory
    package_share = get_package_share_directory('image_undistort')
    
    # ========== CONFIGURABLE PARAMETERS ==========
    # Camera 1 parameters
    camera_1_name = 'camera_1'
    camera_1_device_id = '/dev/video5'
    camera_1_calibration_file = os.path.join(package_share, 'calibration', 'camera_1_calibration.npz')
    image_width = 1920
    image_height = 1080
    
    # Camera 2 parameters
    camera_2_name = 'camera_2'
    camera_2_device_id = '/dev/video6'
    camera_2_calibration_file = os.path.join(package_share, 'calibration', 'camera_2_calibration.npz')
    # =============================================
    
    # ========== CAMERA 1 NODES ==========
    
    # 1. Static transform publisher for camera_1
    camera_1_static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_1_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_1_optical_frame'],
        output='screen'
    )
    
    # 2. Combined USB Camera + Undistortion node for camera_1
    camera_1_combined_node = Node(
        package='image_undistort',  # Update with your package name
        executable='usb_cam_undistort_node',
        name='camera_1_combined',
        parameters=[{
            'camera_name': camera_1_name,
            'video_device': camera_1_device_id,
            'output_topic': '/camera_1/image_undistorted/compressed',
            'camera_info_topic': '/camera_1/camera_info',
            'image_width': image_width,
            'image_height': image_height,
            'framerate': 60.0,
            'brightness': 100,
            'auto_white_balance': 0,
            'white_balance': 3950,
            'autoexposure': 0,
            'exposure': 750,
            'autofocus': 0,
            'output_width': image_width,
            'output_height': image_height,
            'encode_format': 'jpeg',
            'jpeg_quality': 80,
             'K' : [314.000000, 0.0, 945.000000, 0.0, 307.000000, 557.000000, 0.0, 0.0, 1.0],
             'D' : [0.000069, 0.000000, 0.001000, 0.000001]
        }],
        output='screen'
    )
    
    # 3. Robot ArUco detector for camera_1
    camera_1_robot_aruco_detector = Node(
        package='image_undistort',
        executable='aruco_detector_single',
        name='robot_aruco_detector_camera_1',
        parameters=[{
            'camera_name': 'camera_1',
            'input_image_topic': '/camera_1/image_undistorted/compressed',
            'output_topic': '/robot/pose/camera_1',
            'aruco_id': 14,
            'message_type': 'pose',
            'marker_name': 'ROBOT',
            'center_dot_color': 'yellow',
            'encode_format': 'jpeg',
            'jpeg_quality': 80,
            'center_x': 891,
            'center_y': 776
        }],
        output='screen'
    )
    
    # 4. Enemy ArUco detector for camera_1
    camera_1_enemy_aruco_detector = Node(
        package='image_undistort',
        executable='aruco_detector_single',
        name='enemy_aruco_detector_camera_1',
        parameters=[{
            'camera_name': 'camera_1',
            'input_image_topic': '/camera_1/image_undistorted/compressed',
            'output_topic': '/enemy/point/camera_1',
            'aruco_id': 47,
            'message_type': 'point_stamped',
            'marker_name': 'ENEMY',
            'center_dot_color': 'cyan',
            'encode_format': 'jpeg',
            'jpeg_quality': 80,
            'center_x': 891,
            'center_y': 776
        }],
        output='screen'
    )

    morph_node_camera_1 = Node(
        package='image_undistort',
        executable='morph_detector',
        name='morph_detection_camera_1',
        parameters=[{
    'input_image_topic': '/camera_1/image_undistorted/compressed',
    'output_topic_1': '/morph/point_1/camera_1',
    'output_topic_2': '/morph/point_2/camera_1',
    'output_topic_debug': '/morph/debug_image_rviz/camera_1',
    'center_x': 891 // 3,
    'center_y': 776 // 3
        }],
        output='screen'
    )
    
    # ========== CAMERA 2 NODES ==========
    
    # 1. Static transform publisher for camera_2
    camera_2_static_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_2_static_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'camera_2_optical_frame'],
        output='screen'
    )
    
    # 2. Combined USB Camera + Undistortion node for camera_2
    camera_2_combined_node = Node(
        package='image_undistort',  # Update with your package name
        executable='usb_cam_undistort_node',
        name='camera_2_combined',
        parameters=[{
            'camera_name': camera_2_name,
            'video_device': camera_2_device_id,
            'output_topic': '/camera_2/image_undistorted/compressed',
            'camera_info_topic': '/camera_2/camera_info',
            'image_width': image_width,
            'image_height': image_height,
            'framerate': 60.0,
            'brightness': 128,
            'auto_white_balance': 0,
            'white_balance': 3950,
            'autoexposure': 0,
            'exposure': 250,
            'autofocus': 0,
            'output_width': image_width,
            'output_height': image_height,
            'encode_format': 'jpeg',
            'jpeg_quality': 80,
            'K' : [492.000000, 0.0, 884.000000, 0.0, 396.000000, 459.000000, 0.0, 0.0, 1.0],
            'D' : [0.000006, 0.000000, 0.000059, 0.000597]
        }],
        output='screen'
    )
    
    # 3. Robot ArUco detector for camera_2
    camera_2_robot_aruco_detector = Node(
        package='image_undistort',
        executable='aruco_detector_single',
        name='robot_aruco_detector_camera_2',
        parameters=[{
            'camera_name': 'camera_2',
            'input_image_topic': '/camera_2/image_undistorted/compressed',
            'output_topic': '/robot/pose/camera_2',
            'aruco_id': 14,
            'message_type': 'pose',
            'marker_name': 'ROBOT',
            'center_dot_color': 'yellow',
            'encode_format': 'jpeg',
            'jpeg_quality': 80,
            'center_x': 936,
            'center_y': 344
        }],
        output='screen'
    )
    
    # 4. Enemy ArUco detector for camera_2
    camera_2_enemy_aruco_detector = Node(
        package='image_undistort',
        executable='aruco_detector_single',
        name='enemy_aruco_detector_camera_2',
        parameters=[{
            'camera_name': 'camera_2',
            'input_image_topic': '/camera_2/image_undistorted/compressed',
            'output_topic': '/enemy/point/camera_2',
            'aruco_id': 47,
            'message_type': 'point_stamped',
            'marker_name': 'ENEMY',
            'center_dot_color': 'cyan',
            'encode_format': 'jpeg',
            'jpeg_quality': 80,
            'center_x': 936,
            'center_y': 344
        }],
        output='screen'
    )
    
    morph_node_camera_2 = Node(
        package='image_undistort',
        executable='morph_detector',
        name='morph_detection_camera_2',
        parameters=[{
    'input_image_topic': '/camera_2/image_undistorted/compressed',
    'output_topic_1': '/morph/point_1/camera_2',
    'output_topic_2': '/morph/point_2/camera_2',
    'output_topic_debug': '/morph/debug_image_rviz/camera_2',
    'center_x': 936 // 3,
    'center_y': 344 // 3
        }],
        output='screen'
    )
    
    # Add all camera_1 nodes to launch description
    ld.add_action(camera_1_static_transform)
    ld.add_action(camera_1_combined_node)
    ld.add_action(camera_1_robot_aruco_detector)
    ld.add_action(camera_1_enemy_aruco_detector)
    ld.add_action(morph_node_camera_1)
    
    # Add all camera_2 nodes to launch description
    ld.add_action(camera_2_static_transform)
    ld.add_action(camera_2_combined_node)
    ld.add_action(camera_2_robot_aruco_detector)
    ld.add_action(camera_2_enemy_aruco_detector)
    ld.add_action(morph_node_camera_2)
    
    return ld

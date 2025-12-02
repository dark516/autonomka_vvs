from setuptools import setup
import os
from glob import glob

package_name = 'image_undistort'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include all launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/calibration', glob('calibration/*.npz')),
        ('share/' + package_name + '/launch', ['launch/multi_camera_launch.py']),
        (os.path.join('share', package_name, 'launch'), 
         glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Varin Georgiy',
    maintainer_email='todo@example.com',
    description='Image processing package with undistortion and ArUco detection',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'undistort_node = image_undistort.undistort_node:main',
            'usb_cam_undistort_node = image_undistort.usb_cam_undistort_node:main',
            'aruco_detector_single = image_undistort.aruco_detector_single:main',
            'debug_viewer = image_undistort.debug_viewer:main',
            'aruco_viz_node = image_undistort.aruco_viz_node:main',
            'pose_debug_node = image_undistort.pose_debug_node:main',
            'morph_detector = image_undistort.morph_detector_node:main',
            'detector = image_undistort.detector:main'
        ],
    },
)

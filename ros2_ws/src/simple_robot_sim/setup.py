from setuptools import setup
import os
from glob import glob

package_name = 'simple_robot_sim'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your-email@example.com',
    description='2D симулятор боевых роботов',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'simulator_node = simple_robot_sim.simulator_node:main',
            'data_publisher_node = simple_robot_sim.data_publisher_node:main',
            'test_cmd_vel_publisher = simple_robot_sim.test_cmd_vel_publisher',
        ],
    },
)

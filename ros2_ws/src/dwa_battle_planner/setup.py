from setuptools import find_packages, setup

package_name = 'dwa_battle_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kulagin Alexander',
    maintainer_email='sashakulagin2007@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dwa_planner_node = dwa_battle_planner.dwa_planner_node:main',
            'robot_pose_test = dwa_battle_planner.robot_pose_test:main'
        ],
    },
)

from setuptools import find_packages, setup

package_name = 'uav_path_planning'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch',
        ['launch/path_planning_collision_avoidance.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mikelap',
    maintainer_email='mi.tamvak@gmail.com',
    description='A*-based collision avoidance stack for UAVs (ROS 2 + PX4).',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'goal_projector = uav_path_planning.goal_projector_node:main',
            'astar_planner = uav_path_planning.astar_planner_node:main',
            'obstacle_detector = uav_path_planning.obstacle_detector_node:main',
            'command_node = uav_path_planning.command_node:main',
        ],
    },
)

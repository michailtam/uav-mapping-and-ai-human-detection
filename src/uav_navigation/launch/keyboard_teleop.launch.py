#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    
    # Configuration
    package_name = 'px4_offboard' 
    ros_distro = 'jazzy'         

    # Path to your ROS 2 setup file
    ros_setup_path = f'/opt/ros/{ros_distro}/setup.bash'

    # Launch the main offboard controller node
    controller_node = Node(
        package=package_name,
        executable='velocity_offboard_control.py',
        name='velocity_offboard_control',
        output='screen',
        emulate_tty=True,  # This ensures Python's print() statements are shown
    )
    
    # Launch the custom teleop keyboard node in a new gnome-terminal
    teleop_node_process = ExecuteProcess(
        cmd=[
            'gnome-terminal',
            '--',
            'bash', '-c',
            # Command to run in the new terminal:
            f'source {ros_setup_path} && '
            f'echo "Terminal for Keyboard Teleop (Press Space to Arm)" && '
            f'ros2 run {package_name} teleop_keyboard.py; '
            'exec bash' # Keeps the terminal open after the script exits
        ],
        output='screen',
        shell=False
    )

    return LaunchDescription([
        controller_node,
        teleop_node_process
    ])
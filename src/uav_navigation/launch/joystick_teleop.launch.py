#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    joy_config_yaml_file = os.path.join(get_package_share_directory("uav_navigation"), "config", "joy_config.yaml")

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time", 
        default_value="True",
        description="Use simulated time"
    )

    # controller_node = Node(
    #     package='uav_navigation',
    #     executable='velocity_offboard_control.py',
    #     name='velocity_offboard_control',
    #     output='screen',
    #     emulate_tty=True,  # This ensures Python's print() statements are shown
    # )

    joystick_teleop_node = Node(
        package='uav_navigation',
        executable='teleop_joystick.py',
        name='teleop_joystick',
        output='screen',
        emulate_tty=True,  # This ensures Python's print() statements are shown
    )

    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        parameters=[
            joy_config_yaml_file,
            {"use_sim_time": LaunchConfiguration("use_sim_time")}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        # controller_node,
        joy_node,
        joystick_teleop_node,
    ])
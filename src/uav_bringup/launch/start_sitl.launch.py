import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription
from launch_ros.actions import Node
from launch.event_handlers import OnShutdown, OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package directories
    pkg_share_uav_gazebo_sim = get_package_share_directory("uav_gazebo_sim")

    # Start the XRCE-DDS agent to establish communication between px4 and ros2
    px4_agent_proc = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        shell=False
    )

    # Stop the XRCE-DDS agent before shutdown
    kill_agent_on_shutdown = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[ExecuteProcess(
                cmd=['pkill', '-f', 'MicroXRCEAgent'], 
                output='screen', 
                shell=False)]
        )
    )

    # Execute the simulation with px4 flight controller
    px4_autopilot_proc = ExecuteProcess(
        cmd=['./external/PX4-Autopilot/build/px4_sitl_default/bin/px4'],
        output='screen',
        shell=True
    )

    # Load ros2_control controllers
    gazebo_pkg_include_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share_uav_gazebo_sim, 
                'launch', 
                'spawn_uav.launch.py']),
            ))
    
    ld = LaunchDescription()
    ld.add_action(px4_agent_proc)
    ld.add_action(kill_agent_on_shutdown)
    ld.add_action(px4_autopilot_proc)
    ld.add_action(gazebo_pkg_include_sim)
    return ld
import os
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package directories
    pkg_share_uav_gazebo_sim = get_package_share_directory("uav_gazebo_sim")

    # Start Gazebo bridge (includes reading joint_states)
    ros_gz_bridge_incl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share_uav_gazebo_sim, 'launch', 'start_ros_gz_bridge.launch.py'])
            )
        )
    
    ld = LaunchDescription()
    ld.add_action(ros_gz_bridge_incl)
    return ld
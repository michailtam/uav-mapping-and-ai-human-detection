from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package directories
    pkg_share_uav_navigation = get_package_share_directory("uav_navigation")
    
    # Start the launch file for SLAM
    slam_incl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share_uav_navigation, 'launch', 'slam.launch.py'])
            )
        )
    
    # Start the launch file for navigation
    navigation_incl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share_uav_navigation, 'launch', 'navigate.launch.py'])
            )
        )
    
    ld = LaunchDescription()
    ld.add_action(slam_incl)
    ld.add_action(navigation_incl)
    return ld
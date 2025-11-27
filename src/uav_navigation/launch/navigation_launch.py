from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.substitutions import FindPackageShare

from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    # Package and file paths
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])

    ld = LaunchDescription()
    return ld
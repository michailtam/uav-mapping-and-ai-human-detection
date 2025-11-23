import os
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_uav_navigation = get_package_share_directory('uav_navigation')

    nav2_params_file = os.path.join(
        get_package_share_directory('uav_navigation'), 'config', 'nav2_params.yaml')
    
    # Path to the Nav2 navigation launch file.
    nav2_launch = PathJoinSubstitution([pkg_uav_navigation, 'launch', 'navigation_launch.py'])

    use_sim_time = LaunchConfiguration('use_sim_time')

    use_sim_time_arg = DeclareLaunchArgument(
        name='use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    # Include the full Nav2 navigation stack (global planner, controller, BT navigator and so on).
    nav2_launch_incl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(nav2_launch),
        launch_arguments=[
            ('use_sim_time', use_sim_time),
            ('params_file', nav2_params_file)])
    
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(nav2_launch_incl)
    return ld

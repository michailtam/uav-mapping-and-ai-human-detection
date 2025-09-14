from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package and file paths
    pkg_share_bringup = get_package_share_directory('uav_bringup')

    # Launch argument to launch gazebo optionally
    with_gazebo_arg = DeclareLaunchArgument(
        'with_gazebo',
        default_value='true',
        description='Launch Gazebo simulation'
    )
    with_gazebo = LaunchConfiguration('with_gazebo')

    # Display the UAV and optionally in RViz
    display_uav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share_bringup,
                'launch',
                'display.launch.py'])))
    
    # Spawn the UAV in gazebo simulation
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share_bringup,
                'launch',
                'spawn_uav.launch.py'])),
        condition=IfCondition(with_gazebo)
    )

    ld = LaunchDescription()
    ld.add_action(with_gazebo_arg)
    ld.add_action(display_uav)
    ld.add_action(gazebo_sim)

    return ld
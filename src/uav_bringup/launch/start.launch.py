from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package and file paths
    pkg_share_description = get_package_share_directory('uav_description')
    pkg_share_gazebo_sim = get_package_share_directory('uav_gazebo_sim')
    pkg_share_ros2_control = get_package_share_directory('uav_ros2_control')
    pkg_share_perception = get_package_share_directory('uav_perception')

    # Declare launch arguments

    # controller_delay = LaunchConfiguration('controller_delay')
    controller_delay = LaunchConfiguration('controller_delay')
    controller_delay_arg = DeclareLaunchArgument(
        name="controller_delay",
        default_value='10.0',   # Delay depending on the chosen world loading time.
        description='Delay which determines that Gazebo has loaded, before loading the controllers')

    # Launch argument to launch gazebo optionally
    with_gazebo = LaunchConfiguration('with_gazebo')
    with_gazebo_arg = DeclareLaunchArgument(
        'with_gazebo',
        default_value='true',
        description='Launch Gazebo simulation'
    )

    # Display the UAV and optionally in RViz
    launch_display_uav = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share_description,
                'launch',
                'display.launch.py'])))
    
    # Spawn the UAV in gazebo simulation
    launch_gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share_gazebo_sim,
                'launch',
                'spawn_uav.launch.py'])),
        condition=IfCondition(with_gazebo)
    )

    # Launch joint state broadcaster
    launch_controllers = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share_ros2_control,
                'launch',
                'start_controllers.launch.py'])))
    
    # Launch controllers with a short delay after Gazebo to ensure the simulation is fully ready
    delayed_launch_controllers = TimerAction(
        period=controller_delay,
        actions=[launch_controllers]
    )

    # # Start mapping process using RTAB-Map 
    # rtabmap = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource(
    #         PathJoinSubstitution([
    #             pkg_share_perception,
    #             'launch',
    #             'rtabmap_mapping.launch.py'])),
    # )

    ld = LaunchDescription()
    ld.add_action(with_gazebo_arg)
    ld.add_action(controller_delay_arg)
    ld.add_action(launch_display_uav)
    ld.add_action(launch_gazebo_sim)
    ld.add_action(delayed_launch_controllers)
    # ld.add_action(rtabmap)

    return ld
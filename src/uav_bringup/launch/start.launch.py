from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package and file paths
    pkg_share_description = get_package_share_directory('uav_description')
    pkg_share_gazebo_sim = get_package_share_directory('uav_gazebo_sim')
    pkg_share_perception = get_package_share_directory('uav_perception')

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
                pkg_share_description,
                'launch',
                'display.launch.py'])))
    
    # Spawn the UAV in gazebo simulation
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share_gazebo_sim,
                'launch',
                'spawn_uav.launch.py'])),
        condition=IfCondition(with_gazebo)
    )

    # Start mapping process using RTAB-Map 
    rtabmap = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                pkg_share_perception,
                'launch',
                'rtabmap_mapping.launch.py'])),
    )

    # Launch joint state broadcaster
    # start_joint_state_broadcaster_cmd = ExecuteProcess(
    #     cmd=['ros2', 'control', 'load_controller', '--set-state', 'active',
    #         'joint_state_broadcaster'],
    #         output='screen')

    # # Launch the joint state broadcaster after spawning the robot
    # load_joint_state_broadcaster_cmd = RegisterEventHandler(
    #     event_handler=OnProcessExit(
    #     target_action=spawn ,
    #     on_exit=[start_joint_state_broadcaster_cmd],))

    # # Delay the start by 7 seconds using TimerAction
    # delayed_rqt_traj_controller = TimerAction(
    #     period=10.0,  # Delay in seconds
    #     actions=[rqt_traj_controller]
    # )

    ld = LaunchDescription()
    ld.add_action(with_gazebo_arg)
    ld.add_action(display_uav)
    ld.add_action(gazebo_sim)
    ld.add_action(rtabmap)

    return ld
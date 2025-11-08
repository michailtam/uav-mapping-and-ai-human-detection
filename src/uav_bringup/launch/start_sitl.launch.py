from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler, IncludeLaunchDescription, SetEnvironmentVariable
from launch.event_handlers import OnShutdown
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package directories
    pkg_share_uav_description = get_package_share_directory("uav_description")

    # Publish robot_description and /joint_states BEFORE gazebo and px4 starts
    robotdescription_incl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share_uav_description, 'launch', 'display.launch.py'])
            )
        )

    # Start the XRCE-DDS agent to establish communication between px4 and ros2
    px4_agent_proc = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        shell=False
    )

    # Stop the XRCE-DDS agent before shutdown
    kill_agent_on_shutdown_proc = RegisterEventHandler(
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
    
    ld = LaunchDescription()
    ld.add_action(px4_agent_proc)
    ld.add_action(kill_agent_on_shutdown_proc)
    ld.add_action(px4_autopilot_proc)
    ld.add_action(robotdescription_incl)
    return ld
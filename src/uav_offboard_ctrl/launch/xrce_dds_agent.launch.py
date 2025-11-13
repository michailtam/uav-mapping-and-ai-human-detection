from launch import LaunchDescription
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnShutdown


def generate_launch_description():

    # Start the XRCE-DDS agent to establish communication between px4 and ros2
    agent_proc = ExecuteProcess(
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

    ld = LaunchDescription()
    ld.add_action(agent_proc)
    ld.add_action(kill_agent_on_shutdown_proc)
    return ld
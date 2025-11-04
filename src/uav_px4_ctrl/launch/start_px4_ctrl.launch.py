import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess, RegisterEventHandler
from launch_ros.actions import Node
from launch.event_handlers import OnShutdown
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package and file paths
    # pkg_share_uav_px4_ctrl = get_package_share_directory('uav_px4_ctrl')

    px4_process = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        shell=True,
    )

    kill_agent_on_shutdown = RegisterEventHandler(
        event_handler=OnShutdown(
            on_shutdown=[ExecuteProcess(
                cmd=['pkill', '-f', 'MicroXRCEAgent'], shell=True)]
        )
    )

    # px4_ctrl_node = Node(
    #     package='uav_px4_ctrl',
    #     executable='px4_ctrl',
    #     name='px4_ctrl_node',
    #     parameters=[{
    #       'use_sim_time': True
    #     }],
    #     output='screen'
    # )

    ld = LaunchDescription()
    ld.add_action(px4_process)
    ld.add_action(kill_agent_on_shutdown)
    # ld.add_action(px4_ctrl_node)
    return ld
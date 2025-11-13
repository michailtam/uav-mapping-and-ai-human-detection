from launch import LaunchDescription
from launch.actions import ExecuteProcess


def generate_launch_description():

    # Execute the simulation with px4 flight controller
    px4_autopilot_proc = ExecuteProcess(
        cmd=['./external/PX4-Autopilot/build/px4_sitl_default/bin/px4'],
        output='screen',
        shell=True
    )

    ld = LaunchDescription()
    ld.add_action(px4_autopilot_proc)
    return ld
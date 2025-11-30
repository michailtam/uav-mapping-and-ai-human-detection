from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    # Start the FCU control
    px4_ctrl_node = Node(
        package='uav_offboard_ctrl',
        executable='px4_ctrl',
        name='px4_ctrl',
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(px4_ctrl_node)

    return ld

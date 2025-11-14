from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='uav_path_planning',
            executable='goal_projector',
            output='screen'
        ),
        Node(
            package='uav_path_planning',
            executable='astar_planner',
            output='screen'
        ),
        Node(
            package='uav_path_planning',
            executable='obstacle_detector',
            output='screen'
        ),
        Node(
            package='uav_path_planning',
            executable='command_node',
            output='screen'
        ),
    ])

import os
import xacro
import subprocess
from launch_ros.actions import Node
from launch import LaunchDescription
from launch. actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Package and file paths
    share_dir = get_package_share_directory('uav_description')
    rviz_config_file = os.path.join(share_dir, 'rviz', 'rviz_px4_props_conf.rviz')

    # Create the robot_description from xacro file
    xacro_file = os.path.join(share_dir, 'urdf', 'uav.urdf.xacro') 
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()

    # Declare launch arguments and default parameters
    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='false'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value='true'
    )

    # Create the robot-state-publisher node
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
                'use_sim_time': use_sim_time, 
                'robot_description': robot_urdf
            }])

    # Create RViz node
    rviz_node = Node(
        condition=UnlessCondition(gui),
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(gui_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(robot_state_publisher_node)
    ld.add_action(rviz_node)

    return ld

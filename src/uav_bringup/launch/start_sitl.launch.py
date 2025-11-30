from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package directories
    pkg_share_uav_description = get_package_share_directory("uav_description")
    pkg_share_uav_offboard_ctrl = get_package_share_directory("uav_offboard_ctrl")
    
    # Include the launch of PX4-Autopilot
    px4_autopilot_incl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share_uav_offboard_ctrl, 'launch', 'px4_autopilot.launch.py'])
            )
        )
    
    # Include the launch of robot_description and /joint_states BEFORE gazebo and px4 starts
    robotdescription_incl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share_uav_description, 'launch', 'display.launch.py'])
            )
        )
    
    ld = LaunchDescription()
    ld.add_action(robotdescription_incl)
    ld.add_action(px4_autopilot_incl)
    return ld
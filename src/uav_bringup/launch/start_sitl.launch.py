from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    
    ld = LaunchDescription()
    return ld
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_prefix
import os

def generate_launch_description():
    prefix = get_package_prefix('uav_bringup')
    script_path = os.path.join(prefix, 'lib', 'uav_bringup', 'run_x650_gz.sh')

    return LaunchDescription([
        ExecuteProcess(
            cmd=[script_path],
            output='screen',
            shell=False
        )
    ])
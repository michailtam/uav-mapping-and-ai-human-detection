import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    
    pkg_share_uav_navigation = get_package_share_directory('uav_navigation')
    nav2_params = os.path.join(pkg_share_uav_navigation, "config", "nav2_no_map_params.yaml")
    bringup_dir = get_package_share_directory('nav2_bringup')
    
    configured_params = RewrittenYaml(
        source_file=nav2_params, root_key="", param_rewrites="", convert_types=True
    )

    navigation2_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time": "True",
            "params_file": configured_params,
            "autostart": "True",
        }.items(),
    )

    rviz_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", 'rviz_launch.py')),
    )

    # Create the launch description and populate
    ld = LaunchDescription()
    ld.add_action(navigation2_cmd)
    ld.add_action(rviz_cmd)

    return ld

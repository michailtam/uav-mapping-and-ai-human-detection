import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    config_rviz = os.path.join(
        get_package_share_directory('rtabmap_demos'), 'config', 'demo_robot_mapping.rviz'
    )

    # # Declare launch arguments (and keep the actions)
    # rtabmap_viz_arg = DeclareLaunchArgument(
    #     'rtabmap_viz',
    #     default_value='false',
    #     description='Launch RTAB-Map UI (optional).'
    # )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RVIZ (optional).'
    )

    localization_arg = DeclareLaunchArgument(
        'localization',
        default_value='false',
        description='Launch in localization mode.'
    )

    rviz_cfg_arg = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=config_rviz,
        description='Configuration path of rviz2.'
    )

    parameters = {
        'frame_id': 'base_footprint',
        'subscribe_rgbd': True,
        'subscribe_scan': True,
        'approx_sync': True,
        'map_negative_poses_ignored': True,
        'subscribe_odom_info': True,
        'OdomF2M/MaxSize': '1000',
        'GFTT/MinDistance': '10',
        'GFTT/QualityLevel': '0.00001',
    }

    localization = LaunchConfiguration('localization')

    # RTAB-Map SLAM mode
    rtabmap_node = Node(
        condition=UnlessCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[parameters],
        arguments=['-d']
    )

    # RTAB-Map Localization mode
    rtabmap_loc_node = Node(
        condition=IfCondition(localization),
        package='rtabmap_slam',
        executable='rtabmap',
        output='screen',
        parameters=[parameters, {
            'Mem/IncrementalMemory': 'False',
            'Mem/InitWMWithAllNodes': 'True'
        }],
    )

    # Visualization
    rtabmap_viz_node = Node(
        package='rtabmap_viz',
        executable='rtabmap_viz',
        output='screen',
        condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
        parameters=[parameters],
    )

    # rviz_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     output='screen',
    #     condition=IfCondition(LaunchConfiguration('rviz')),
    #     arguments=['-d', LaunchConfiguration('rviz_cfg')]
    # )

    # Build LaunchDescription with actions only (no LaunchConfiguration directly)
    ld = LaunchDescription()
    ld.add_action(rtabmap_viz_arg)
    ld.add_action(rviz_arg)
    ld.add_action(localization_arg)
    ld.add_action(rviz_cfg_arg)
    ld.add_action(rtabmap_node)
    ld.add_action(rtabmap_loc_node)
    ld.add_action(rtabmap_viz_node)
    # ld.add_action(rviz_node)

    return ld

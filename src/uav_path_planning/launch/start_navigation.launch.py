import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package directories
    pkg_share_uav_path_planning = get_package_share_directory("uav_path_planning")
    pkg_share_uav_mapping = get_package_share_directory("uav_mapping")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_rtabmap = LaunchConfiguration("use_rtabmap")
    use_rviz = LaunchConfiguration("use_rviz")

    # Declare the launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    use_rtabmap_arg = DeclareLaunchArgument(
        "use_rtabmap",
        default_value="true"
    )

    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true"
    )
    
    # Start the Localization process
    localization = IncludeLaunchDescription(
        os.path.join(
            pkg_share_uav_path_planning,
            "launch",
            "localization.launch.py"
        ),
        condition=UnlessCondition(use_rtabmap)
    )

    # Include the mapping using RTAB-Map
    online_mapping_incl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share_uav_mapping, 'launch', 'rtabmap.launch.py'])
            ),
            condition=IfCondition(use_rtabmap)
        )

    # Save the map to hard disk when SLAM is desired
    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name="map_saver_server",
        output="screen",
        parameters=[
            {"save_map_timeout": 5.0},
            {"use_sim_time": use_sim_time},
            {"free_thresh_default": 0.196},
            {"occupied_thresh_default": 0.65}
        ],
        condition=IfCondition(use_rtabmap)
    )

    # Lifecycle manager for map_saver_server
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": ["map_saver_server"]},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
        condition=IfCondition(use_rtabmap)
    )

    # Improve the Localization with an Extended Kalman Filter (EKF) 
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(pkg_share_uav_path_planning, "config", "ekf.yaml")],
    )

    # Launch rviz
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(
                pkg_share_uav_path_planning,
                "rviz",
                "slam.rviz"
            )
        ],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        condition=IfCondition(use_rviz)
    )
    
    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(use_rtabmap_arg)
    ld.add_action(use_rviz_arg)
    ld.add_action(localization)
    ld.add_action(online_mapping_incl)
    ld.add_action(ekf)
    ld.add_action(nav2_map_saver)
    ld.add_action(nav2_lifecycle_manager)
    ld.add_action(rviz)

    return ld
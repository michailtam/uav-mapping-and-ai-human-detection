import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_config = LaunchConfiguration("slam_config")

    # Declare the launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    slam_config_arg = DeclareLaunchArgument(
        "slam_config",
        default_value=os.path.join(
            get_package_share_directory("uav_navigation"), "config", "slam_toolbox.yaml"),
        description="Full path to slam yaml file to load"
    )

    # Launch SLAM
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name='slam_toolbox',
        output="screen",
        parameters=[
            slam_config,
            {"use_sim_time": use_sim_time},
        ]
    )

    # Save the map to hard disk when SLAM is desired
    nav2_map_saver = Node(
        package="nav2_map_server",
        executable="map_saver_server",
        name='map_saver_server',
        output="screen",
        parameters=[
            {"save_map_timeout": 25.0},
            {"use_sim_time": use_sim_time},
            {"free_thresh_default": 0.25},   # 0.196
            {"occupied_thresh_default": 0.60}
        ]
    )

    # Manages node states 
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        output="screen",
        parameters=[
            {"node_names": ["map_saver_server", "slam_toolbox"]},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"bond_timeout": 0.0}
        ]
    )

    # Improve the Localization with an Extended Kalman Filter (Sensor Fusion) 
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("uav_navigation"), "config", "ekf.yaml"),
                    {"use_sim_time": use_sim_time}
        ]
    )

    # Static transform for base_link to laser/camera frame
    static_tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'x650_0/base_footprint', 'base_link'],
        parameters=[{"use_sim_time": use_sim_time}]
    )
    
    return LaunchDescription([
        use_sim_time_arg,
        slam_config_arg,
        slam,
        ekf,
        nav2_map_saver,
        nav2_lifecycle_manager,
        static_tf_base_footprint_to_base_link
    ])
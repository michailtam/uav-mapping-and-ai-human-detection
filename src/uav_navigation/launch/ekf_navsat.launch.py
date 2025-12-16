import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import launch.actions


def generate_launch_description():

    pkg_share_uav_navigation = get_package_share_directory('uav_navigation')
    rl_params_file = os.path.join(pkg_share_uav_navigation, "config", "ekf_navsat.yaml")

    use_sim_time = LaunchConfiguration("use_sim_time")

    # Declare the launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    launch.actions.DeclareLaunchArgument("output_final_position", default_value="false")
    launch.actions.DeclareLaunchArgument("output_location", default_value="~/dual_ekf_navsat_example_debug.txt")
    
    ekf_filer_node_odom = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_odom",
        output="screen",
        parameters=[rl_params_file, {"use_sim_time": True}, {"use_sim_time": use_sim_time}],
        remappings=[("odometry/filtered", "odometry/local")]
    )

    ekf_filter_node_map = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node_map",
        output="screen",
        parameters=[rl_params_file, {"use_sim_time": True}],
        remappings=[("odometry/filtered", "odometry/global")]
    )

    # Static transform for base_link to laser/camera frame
    static_tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_base_to_laser',
        arguments=['0', '0', '0', '0', '0', '0', 'x650_0/base_footprint', 'base_link'],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    navsat_transform = Node(
        package="robot_localization",
        executable="navsat_transform_node",
        name="navsat_transform",
        output="screen",
        parameters=[rl_params_file, {"use_sim_time": True}],
        remappings=[
            ("imu/data", "imu/data"),
            ("gps/fix", "gps/fix"),
            ("gps/filtered", "gps/filtered"),
            ("odometry/gps", "odometry/gps"),
            ("odometry/filtered", "odometry/global"),
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        ekf_filer_node_odom,
        ekf_filter_node_map,
        navsat_transform,
        static_tf_base_footprint_to_base_link
    ])
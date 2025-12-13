import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    pkg_share_uav_navigation = get_package_share_directory("uav_navigation")

    use_sim_time = LaunchConfiguration("use_sim_time")
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    # Nodes that gets managed by the lifecycle manager
    lifecycle_nodes = ["controller_server", "planner_server"]

    # Executes planned path following by generating velocity commands (necessary for PX4)
    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name='controller_server',
        output="screen",
        parameters=[
            os.path.join(pkg_share_uav_navigation, "config", "controller_server.yaml"),
            {"use_sim_time": use_sim_time}
        ]
    )
    
    # Executes the planner for collision-free paths in 2D space from start to goal
    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            os.path.join(pkg_share_uav_navigation, "config", "planner_server.yaml"),
            {"use_sim_time": use_sim_time}
        ]
    )

    # # Executes the high-level navigation coordinator using behavior trees
    # nav2_bt_navigator = Node(
    #     package="nav2_bt_navigator",
    #     executable="bt_navigator",
    #     name="bt_navigator",
    #     output="screen",
    #     parameters=[
    #         os.path.join(pkg_share_uav_navigation, "config", "bt_navigator.yaml"),
    #         {"use_sim_time": use_sim_time}
    #     ]
    # )

    # Manages node states 
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True},
            {"bond_timeout": 20.0}
        ]
    )

    return LaunchDescription([
        use_sim_time_arg,
        nav2_controller_server,
        nav2_planner_server,
        # nav2_bt_navigator,
        nav2_lifecycle_manager,
    ])
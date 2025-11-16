import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    lifecycle_nodes = ["controller_server", "planner_server", "smoother_server", "bt_navigator", "behavior_server"]
    pkg_share_uav_path_planning = get_package_share_directory("uav_path_planning")

    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true"
    )

    # Start the server for the controller
    nav2_controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        output="screen",
        parameters=[
            os.path.join(
                pkg_share_uav_path_planning,
                "config",
                "controller_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )
    
    # Start the planner to calculate the path
    nav2_planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        output="screen",
        parameters=[
            os.path.join(
                pkg_share_uav_path_planning,
                "config",
                "planner_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    # Start the smoother which smooths the path to the destination
    nav2_smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        output="screen",
        parameters=[
            os.path.join(
                pkg_share_uav_path_planning,
                "config",
                "smoother_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    # Start the nav2 navigator for executing the navigation process
    nav2_bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        output="screen",
        parameters=[
            os.path.join(
                pkg_share_uav_path_planning,
                "config",
                "bt_navigator.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    # Start the behavior of the robot when it cannot execute the navigation
    nav2_behaviours = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        output="screen",
        parameters=[
            os.path.join(
                pkg_share_uav_path_planning,
                "config",
                "behavior_server.yaml"),
            {"use_sim_time": use_sim_time}
        ],
    )

    # Start the finate state machine for navigation 
    nav2_lifecycle_manager = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time": use_sim_time},
            {"autostart": True}
        ],
    )

    ld = LaunchDescription()
    ld.add_action(use_sim_time_arg)
    ld.add_action(nav2_controller_server)
    ld.add_action(nav2_planner_server)
    ld.add_action(nav2_smoother_server)
    ld.add_action(nav2_bt_navigator)
    ld.add_action(nav2_behaviours)
    ld.add_action(nav2_lifecycle_manager)

    return ld
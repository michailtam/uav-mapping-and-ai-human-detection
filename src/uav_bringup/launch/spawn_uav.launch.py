import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration, Command
from launch_ros.actions import Node
from os.path import join
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package and file paths
    pkg_share_gazebo = get_package_share_directory('uav_gazebo_sim')
    pkg_ros_gz_rbot = get_package_share_directory('uav_description')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    ros_gz_bridge_config = os.path.join(pkg_ros_gz_rbot, 'config', 'ros_gz_bridge_gazebo.yaml')

    # Set the launch configuration for the world
    world_name = LaunchConfiguration('world_name')

    # Set the pose configuration variables
    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    roll = LaunchConfiguration('roll')
    pitch = LaunchConfiguration('pitch')
    yaw = LaunchConfiguration('yaw')

    # Set default launch arguments
    declare_world_arg = DeclareLaunchArgument(
        name="world_name",
        default_value='empty',
        description='The world file to use')

    declare_x_pos_arg = DeclareLaunchArgument(
        name='x',
        default_value='0.0',
        description='x-position')

    declare_y_pos_arg = DeclareLaunchArgument(
        name='y',
        default_value='3.0',
        description='y-position')
    
    declare_z_pos_arg = DeclareLaunchArgument(
        name='z',
        default_value='0.0',
        description='z-position')

    declare_roll_arg = DeclareLaunchArgument(
        name='roll',
        default_value='0.0',
        description='roll angle of initial orientation in radians')

    declare_pitch_arg = DeclareLaunchArgument(
        name='pitch',
        default_value='0.0',
        description='pitch angle of initial orientation in radians')

    declare_yaw_arg = DeclareLaunchArgument(
        name='yaw',
        default_value='-1.57',
        description='yaw angle of initial orientation in radians')

    ros_gz_bridge_config = PathJoinSubstitution([
        pkg_share_gazebo, 
        'config',
        'ros_gz_bridge.yaml'])
    
    # Gazebo simulator launch
    world_path = Command(['echo ', pkg_share_gazebo, '/worlds/', world_name, '.world'])

    # Start Gazebo Sim
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments=[
            ('gz_args', ['-r -v 4 ', world_path])
        ]
    )

    # Spawn Robot in Gazebo   
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            "-topic", "/robot_description",
            "-name", "uav",
            "-allow_renaming", "true",
            '-x', x,
            '-y', y,
            '-z', z,
            '-R', roll,
            '-P', pitch,
            '-Y', yaw,
            '-allow_renaming', 'true'
        ],            
        parameters=[{'use_sim_time': True}],
        output='screen',
    )

    # Bridge ROS topics and Gazebo messages for establishing communication
    gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
          'config_file': ros_gz_bridge_config,
          'use_sim_time': True
        }],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(declare_world_arg)
    ld.add_action(declare_x_pos_arg)
    ld.add_action(declare_y_pos_arg)
    ld.add_action(declare_z_pos_arg)
    ld.add_action(declare_roll_arg)
    ld.add_action(declare_pitch_arg)
    ld.add_action(declare_yaw_arg)
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_entity)
    ld.add_action(gazebo_ros_bridge_cmd)
    
    return ld
import os
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.substitutions import PathJoinSubstitution, TextSubstitution, EnvironmentVariable
from launch_ros.actions import Node
from os.path import join
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # # IMPORTANT: Adding the environment variable must be done at first
    # pkg_share_gz = os.path.join(os.getcwd(), "Tools", "simulation", "gz", "models")
    # set_gazebo_model_path = SetEnvironmentVariable(
    #    name='GZ_SIM_RESOURCE_PATH',
    #    value=[
    #        EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value=''),
    #        TextSubstitution(text=':' + pkg_share_gz)
    #    ])

    # Package and file paths
    pkg_share_uav_gazebo_sim = get_package_share_directory('uav_gazebo_sim')
    
    # Set default launch arguments
    ros_gz_bridge_config = PathJoinSubstitution([
        pkg_share_uav_gazebo_sim, 
        'config',
        'ros_gz_bridge.yaml'])

    # Bridge ROS topics and Gazebo messages for establishing communication
    gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gazebo_bridge',
        parameters=[{
          'config_file': ros_gz_bridge_config,
          'use_sim_time': True
        }],
        output='screen'
    )

    # The bridge for cameras
    gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=[
          '/camera/depth_image',
          '/camera/image'
        ],
        remappings=[
          ('/camera/depth_image', '/camera/depth/image_rect_raw'),
          ('/camera/image', '/camera/color/image_raw'),
        ],
      )

    ld = LaunchDescription()
    ld.add_action(gazebo_ros_bridge_cmd)
    ld.add_action(gazebo_ros_image_bridge_cmd)
    
    return ld
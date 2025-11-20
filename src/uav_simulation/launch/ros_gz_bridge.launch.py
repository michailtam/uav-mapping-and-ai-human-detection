import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package and file paths
    pkg_share_uav_simulation = get_package_share_directory('uav_simulation')
    
    # Set default launch arguments
    ros_gz_bridge_config = PathJoinSubstitution([
      pkg_share_uav_simulation, 
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
    
    # This node is necessary to create a static transform between the base_link and x650_0/base_link/laser_sensor link.
    # Otherwise, the lidar sensor data don't get published and therefore shown to RViz.
    static_laser_tf = Node(
      package="tf2_ros",
      executable="static_transform_publisher",
      name="laser_static_tf",
      arguments=["0", "0", "0", "0", "0", "0",
                "base_link", "x650_0/base_link/laser_sensor"])
    
    # The bridge for cameras
    gazebo_ros_image_bridge_cmd = Node(
      package='ros_gz_image',
      executable='image_bridge',
      arguments=[
        'camera/depth_image',
        'camera/image'
      ],
      remappings=[
        ('camera/image', '/camera/color/image_raw'),
        ('camera/depth_image', '/camera/depth/image_rect_raw')
      ],
      output='screen'
    )

    # RGB processor: /camera/color/image_raw -> /image_rgb
    image_tools_rgb_node = Node(
        package='uav_vision',
        executable='image_tools',
        name='camera_image_tools',
        output='screen',
        parameters=[{"use_sim_time": True}],
    )
    
    ld = LaunchDescription()
    ld.add_action(gazebo_ros_bridge_cmd)
    ld.add_action(gazebo_ros_image_bridge_cmd)
    ld.add_action(static_laser_tf)
    ld.add_action(image_tools_rgb_node)

    return ld
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
    pkg_share_uav_simulation = get_package_share_directory('uav_simulation')
    # pkg_share_uav_vision = get_package_share_directory('uav_vision')
    
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

    # Rotate RGB image: /camera/color/image_raw -> /camera/color/image_upright
    rgb_rotate_node = Node(
        package="uav_vision",
        executable="image_tools",
        name="camera_rgb_rotate",
        output="screen",
        remappings=[
            ("image", "/camera/color/image_raw"),
            ("image_rotated", "/camera/color/image_upright"),
        ]
    )

    # Rotate depth image: /camera/depth/image_rect_raw -> /camera/depth/image_rect_upright
    depth_rotate_node = Node(
        package="uav_vision",
        executable="image_tools",
        name="camera_depth_rotate",
        output="screen",
        remappings=[
            ("image", "/camera/depth/image_rect_raw"),
            ("image_rotated", "/camera/depth/image_rect_upright"),
        ]
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_ros_bridge_cmd)
    ld.add_action(gazebo_ros_image_bridge_cmd)
    ld.add_action(static_laser_tf)
    ld.add_action(rgb_rotate_node)
    ld.add_action(depth_rotate_node)

    return ld
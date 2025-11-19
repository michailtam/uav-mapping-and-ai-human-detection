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
      )
    
    # This node is necessary to create a static transform between the base_link and x650_0/base_link/laser_sensor link.
    # Otherwise, the lidar sensor data don't get published and therefore shown to RViz.
    static_laser_tf = Node(
      package="tf2_ros",
      executable="static_transform_publisher",
      name="laser_static_tf",
      arguments=["0", "0", "0", "0", "0", "0",
                "base_link", "x650_0/base_link/laser_sensor"],
    )

    # This node rotates an incoming image by a fixed angle.
    # It's used, to correct the upside-down RGB-D camera image without modifying the SDF model. 
    # The rotated image will be republished under /camera/image_upright.
    image_rotate_node = Node(
        package="image_rotate",
        executable="image_rotate",
        name="camera_image_rotate",
        output="screen",
        remappings=[
            ("image", "/camera/image_raw"),   # INPUT topics from camera
            ("camera_info", "/camera/camera_info"), 
            ("image_rotated", "/camera/image_upright")  # OUTPUT rotated image topic
        ],
        parameters=[{"target_rotation": 3.14159}])  # 180 degrees rotation

    ld = LaunchDescription()
    ld.add_action(gazebo_ros_bridge_cmd)
    ld.add_action(gazebo_ros_image_bridge_cmd)
    ld.add_action(static_laser_tf)
    ld.add_action(image_rotate_node)

    return ld
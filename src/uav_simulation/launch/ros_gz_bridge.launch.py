import os
from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    # Package and file paths
    pkg_share_uav_simulation = get_package_share_directory('uav_simulation')
    pkg_share_uav_mapping = get_package_share_directory('uav_mapping')
    
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
    
    # This node is necessary to create a static transform between the base_footprint and x650_0/base_link/laser_sensor link.
    # Otherwise, the lidar sensor data don't get published and therefore shown to RViz.
    static_laser_tf = Node(
      package="tf2_ros",
      executable="static_transform_publisher",
      name="laser_static_tf",
      parameters=[{'use_sim_time': True}],
      arguments=["0", "0", "0", "0", "0", "0",
                "x650_0/base_footprint", "x650_0/base_link/laser_sensor"])
    
    # Static transform for camera: base_footprint -> x650_0/base_link/rgbd_cam
    # Connects the main base_footprint (from odom/Gazebo) to the camera frame where data is published
    # Camera pose from SDF: xyz="0.13758 -0.00058142 -0.0090308" rpy="-3.1377 -0.0036204 -0.013619"
    static_camera_tf = Node(
      package="tf2_ros",
      executable="static_transform_publisher",
      name="camera_static_tf",
      parameters=[{'use_sim_time': True}],
      arguments=["0.13758", "-0.00058142", "-0.0090308",
                  "-3.1377", "-0.0036204", "-0.013619",
                "x650_0/base_footprint", "x650_0/base_link/rgbd_cam"])
    
    # Static transform for IMU: base_footprint -> x650_0/base_link/imu_sensor
    # IMU is at the center of the drone (same as base_footprint)
    static_imu_tf = Node(
      package="tf2_ros",
      executable="static_transform_publisher",
      name="imu_static_tf",
      parameters=[{'use_sim_time': True}],
      arguments=["0", "0", "0", "0", "0", "0",
                "x650_0/base_footprint", "x650_0/base_link/imu_sensor"])
    
    # Image processor: rotates and republishes camera images
    
    image_tools_rgb_node = Node(
      package='uav_vision',
      executable='image_tools',
      name='camera_image_tools',
      output='screen',
      parameters=[{"use_sim_time": True}],
    )
    
    # Convert odometry message to TF transform
    # Publishes dynamic TF: x650_0/odom -> x650_0/base_footprint
    odom_to_tf_node = Node(
      package='odom_to_tf_ros2',
      executable='odom_to_tf',
      name='odom_to_tf',
      output='screen',
      parameters=[{'use_sim_time': True}],
      remappings=[('odom/perfect', '/odom')])


    # Include the launch of Visual Slam for 3D mapping
    vslam_incl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_share_uav_mapping, 'launch', 'vslam.launch.py'])
        )
    )
    
    ld = LaunchDescription()
    ld.add_action(gazebo_ros_bridge_cmd)
    ld.add_action(static_laser_tf)
    ld.add_action(static_camera_tf)
    ld.add_action(static_imu_tf)
    ld.add_action(odom_to_tf_node)
    ld.add_action(image_tools_rgb_node)
    ld.add_action(vslam_incl)

    return ld
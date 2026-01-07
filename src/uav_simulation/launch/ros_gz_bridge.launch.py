from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    
    # Convert odometry message to TF transform
    # Publishes dynamic TF: /odom -> /base_link
    odom_to_tf_node = Node(
      package='odom_to_tf_ros2',
      executable='odom_to_tf',
      name='odom_to_tf',
      output='screen',
      parameters=[{'use_sim_time': True}],
      remappings=[('odom/perfect', '/odom')])
    
    # Propeller joint states publisher
    offboard_ctrl_node = Node(
      package='uav_offboard_ctrl',
      executable='px4_prop_js_publisher',
      name='px4_prop_js_publisher',
      output='screen',
      parameters=[{'use_sim_time': True}])
    
    ld = LaunchDescription()
    ld.add_action(gazebo_ros_bridge_cmd)
    ld.add_action(odom_to_tf_node)
    ld.add_action(offboard_ctrl_node)

    return ld
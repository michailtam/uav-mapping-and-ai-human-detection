import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    # Package and file paths
    pkg_uav_mapping_share_dir = get_package_share_directory('uav_mapping')
    rviz_config_file = os.path.join(pkg_uav_mapping_share_dir, 'rviz', 'mapping_config.rviz')
    
    # Declare launch arguments and default parameters
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    use_rviz = LaunchConfiguration('use_rviz')

    use_sim_time_decl = DeclareLaunchArgument(
        'use_sim_time', 
        default_value='true')
    
    use_rviz_decl = DeclareLaunchArgument(
        name='use_rviz',
        default_value='true')

    # RTAB-Map Launch
    rtabmap_launch_path = os.path.join(
        get_package_share_directory('rtabmap_launch'), 'launch', 'rtabmap.launch.py')

    rtabmap_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_path),
        launch_arguments={
            'rgb_topic': '/camera/image',
            'depth_topic': '/camera/depth/image',
            'camera_info_topic': '/camera/camera_info',
            'subscribe_scan': 'true',
            'scan_topic': '/scan',
            'approx_sync': 'true',
            'frame_id': 'base_link',
            
            # --- ODOMETRY SETTINGS ---
            'visual_odometry': 'false', # Rely on Gazebo Odom
            'odom_topic': '/odom',
            
            'use_sim_time': use_sim_time,
            'rtabmap_args': '--RGBD/NeighborLinkRefining true --Reg/Strategy 1 -d',
            'qos': '1'
        }.items()
    )

    # Static Transform: Footprint -> Base Link
    tf_base_footprint_to_base = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_footprint_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0', 'x650_0/base_footprint', 'base_link'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # Static Transform: Base Link -> Camera Frame
    tf_base_to_cam = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_camera',
        # x y z yaw pitch roll parent child
        arguments=['0.1', '0', '0', '-1.57', '0', '-1.57', 'base_link', 'x650_0/base_link/rgbd_cam'],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    rviz_node = Node(
        condition=IfCondition(use_rviz),
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config_file],
        parameters=[{"use_sim_time": use_sim_time}]
    )

    return LaunchDescription([
        use_sim_time_decl,
        use_rviz_decl,
        rtabmap_stack,
        tf_base_footprint_to_base,
        tf_base_to_cam,
        rviz_node
    ])
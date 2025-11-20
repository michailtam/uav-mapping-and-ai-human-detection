# NOTE: This code is based on this example: https://github.com/matlabbe/rtabmap_drone_example/tree/ros2

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):

    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params_file = os.path.join(
        get_package_share_directory('uav_navigation'), 
        'config',
        'nav2_params.yaml')

    use_sim_time = LaunchConfiguration("use_sim_time")

    vslam_params ={
        'frame_id':'base_link',                     
        'guess_frame_id':'base_link_stabilized',    
        'approx_sync': False,
        'use_sim_time': use_sim_time,
        'subscribe_rgbd':True,
        'subscribe_odom_info':True,
        'wait_imu_to_init': True,
        'wait_for_transform': 0.5,
        'use_action_for_goal':True,
        # RTAB-Map's parameters should be strings
        'Optimizer/GravitySigma': '0.1',
        'Vis/FeatureType': '10',
        'Kp/DetectorStrategy': '10',
        'Grid/MapFrameProjection': 'true',
        'NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '1.15' ,
        'Grid/MaxObstacleHeight': '1.75',
        'RGBD/StartAtOrigin': 'true'
    }
    
    vslam_remappings=[('imu', '/imu/data'),
                      ('map', '/map'), # nav2 is expecting a static map on topic /map
                      ('navigate_to_pose', '/navigate_to_pose'), # nav2 action server
                      # For humble: https://github.com/ros2/ros2/issues/1312
                      ('navigate_to_pose/_action/feedback', '/navigate_to_pose/_action/feedback'),
                      ('navigate_to_pose/_action/status', '/navigate_to_pose/_action/status'),
                      ('navigate_to_pose/_action/cancel_goal', '/navigate_to_pose/_action/cancel_goal'),
                      ('navigate_to_pose/_action/get_result', '/navigate_to_pose/_action/get_result'),
                      ('navigate_to_pose/_action/send_goal', '/navigate_to_pose/_action/send_goal')]
    
    nav2_launch = PathJoinSubstitution(
        [pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
    
    # Compute imu orientation
    imu_orientation_node = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        parameters=[{
            'use_mag':False,
            'world_frame':'enu',
            'publish_tf':False,
            'use_sim_time': use_sim_time}])
    
    imu_to_tf_node = Node(
        package='rtabmap_util', executable='imu_to_tf', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'fixed_frame_id':'base_link_stabilized',
            'base_frame_id':'base_link'}])
        
    # VSLAM nodes
    vslam_node = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        namespace='rtabmap',
        parameters=[vslam_params],
        remappings=[('rgb/image', '/camera/rgb/image'),
                    ('rgb/camera_info', '/camera/rgb/camera_info'),
                    ('depth/image', '/camera/depth/image')])

    rtabmap_odom_node = Node(
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        namespace='rtabmap',
        parameters=[vslam_params, {'odom_frame_id': 'odom'}],
        remappings=vslam_remappings,
        arguments=["--ros-args", "--log-level", 'warn'])

    # SLAM mode
    rtabmap_slam_node = Node(
        condition=UnlessCondition(LaunchConfiguration('localization')),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        namespace='rtabmap',
        parameters=[vslam_params],
        remappings=vslam_remappings,
        arguments=['-d']) # This will delete the previous database (~/.ros/rtabmap.db)
            
    # Localization mode
    rtabmap_loc_node = Node(
        condition=IfCondition(LaunchConfiguration('localization')),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        namespace='rtabmap',
        parameters=[vslam_params,
                    {'Mem/IncrementalMemory': 'False',
                        'Mem/InitWMWithAllNodes': 'True'}],
        remappings=vslam_remappings)
            
    rtabmap_viz_node = Node(
        condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        namespace='rtabmap',
        parameters=[vslam_params],
        remappings=vslam_remappings)
        
    ros_odom_to_vehcl = Node(
        package='uav_navigation', executable='ros_odometry_to_vehicle_odometry', output='screen',
        parameters=[{'map_frame_id': 'map',
                    'use_sim_time': use_sim_time}],
        remappings=[('odom', '/rtabmap/odom')])

    rtabmap_util_node = Node(
        package='rtabmap_util', executable='point_cloud_xyz', output='screen',
        parameters=[{'decimation': 2,
                        'max_depth': 3.0,
                        'voxel_size': 0.02,
                        'use_sim_time': use_sim_time}],
        remappings=[('depth/image', '/camera/depth/image'),
                    ('depth/camera_info', '/camera/rgb/camera_info'),
                    ('cloud', '/camera/cloud')])
    
    # rtabmap_costmap_node = Node(
    #     package='rtabmap_costmap_plugins', executable='voxel_marker', output='screen',
    #     namespace="local_costmap",
    #     parameters=[{'use_sim_time': use_sim_time}])

    nav2_launch_incl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('params_file', nav2_params_file)
        ]
    )

    # IMPORTANT: OpaqueFunction expects a list of actions, not a LaunchDescription.
    return [
        imu_orientation_node,
        imu_to_tf_node,
        vslam_node,
        rtabmap_odom_node,
        rtabmap_slam_node,
        rtabmap_loc_node,
        rtabmap_viz_node,
        ros_odom_to_vehcl,
        rtabmap_util_node,
        # rtabmap_costmap_node,
        nav2_launch_incl
    ]
    
def generate_launch_description():
    
    return LaunchDescription([
        DeclareLaunchArgument(
            name='use_sim_time', 
            default_value='true',
            description='Enable use_sime_time to true'
        ),
        DeclareLaunchArgument(
            name='rtabmap_viz', 
            default_value='false',
            description='Launch rtabmap_viz'
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='false', 
            description='Launch in localization mode.'),
        
        OpaqueFunction(function=launch_setup)
    ])
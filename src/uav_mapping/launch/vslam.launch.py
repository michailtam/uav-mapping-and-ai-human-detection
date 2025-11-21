# NOTE: This code is based on this example: https://github.com/matlabbe/rtabmap_drone_example/tree/ros2

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


"""
Funtion to setup vSLAM
"""
def launch_setup(context, *args, **kwargs):

    # Package and file paths
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    nav2_params_file = os.path.join(
        get_package_share_directory('uav_navigation'), 'config', 'nav2_params.yaml')

    use_sim_time = LaunchConfiguration("use_sim_time")

    vslam_params = {
        'frame_id':'base_link',                     # Robot body frame used for odometry and TF tree (child frame of odom)
        'guess_frame_id':'base_link_stabilized',    # Frame providing motion prediction (IMU-based stabilization for better VO)
        'approx_sync': False,                       # Require exact RGB/depth timestamp sync (better accuracy, strict timing)
        'use_sim_time': use_sim_time,               # Use Gazebo simulated clock instead of system time
        'subscribe_rgbd': True,                     # Enable RGB-D input (combined RGB + depth image subscription)
        'subscribe_odom_info': True,                # Subscribe to odom info (motion guesses and covariance for better matching)
        'wait_imu_to_init': True,                   # Wait until IMU provides stable gravity direction before starting SLAM
        'wait_for_transform': 0.5,                  # Max TF wait time (sec) for transforms between camera & base_link
        'use_action_for_goal': True,                # Enable Nav2 NavigateToPose action integration (RTAB-Map sends goals)
        
        # RTAB-Map's internal SLAM parameters (must be strings)
        'Optimizer/GravitySigma': '0.1',            # Strength of gravity prior (IMU) for graph optimization (low = strong trust)
        'Vis/FeatureType': '10',                    # Feature type = ORB (10) for visual feature extraction
        'Kp/DetectorStrategy': '10',                # Keypoint detection strategy = ORB (10), consistent with Vis/FeatureType
        'Grid/MapFrameProjection': 'true',          # Project 3D map into 2D grid map (required for Nav2 costmaps)
        'NormalsSegmentation': 'false',             # Disable plane/normal-based segmentation (saves CPU)
        'Grid/MaxGroundHeight': '1.15',             # Max height considered "ground" for obstacle filtering
        'Grid/MaxObstacleHeight': '1.75',           # Max obstacle height included in 2D occupancy grid
        'RGBD/StartAtOrigin': 'true'                # Start SLAM map at (0,0,0) instead of waiting for odom alignment
    }
    
    # Remap RTAB-Map topics to match UAV sensor inputs and Nav2 action interfaces.
    vslam_remappings=[('imu', '/imu/data'),         # IMU data used for motion prediction and gravity alignment
                      ('map', '/map'),                              # Nav2 requires the global map on /map
                      ('navigate_to_pose', '/navigate_to_pose'),    # Nav2 NavigateToPose action server
                      # For humble: https://github.com/ros2/ros2/issues/1312
                      ('navigate_to_pose/_action/feedback', '/navigate_to_pose/_action/feedback'),
                      ('navigate_to_pose/_action/status', '/navigate_to_pose/_action/status'),
                      ('navigate_to_pose/_action/cancel_goal', '/navigate_to_pose/_action/cancel_goal'),
                      ('navigate_to_pose/_action/get_result', '/navigate_to_pose/_action/get_result'),
                      ('navigate_to_pose/_action/send_goal', '/navigate_to_pose/_action/send_goal')]
    
    # Path to the Nav2 navigation launch file (global planner, controller, BT navigator).
    nav2_launch = PathJoinSubstitution([pkg_nav2_bringup, 'launch', 'navigation_launch.py'])
    
    # Filter raw IMU data and compute orientation (roll/pitch/yaw) using the Madgwick filter.
    imu_orientation_node = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        parameters=[{
            'use_mag':False,
            'world_frame':'enu',
            'publish_tf':False,
            'use_sim_time': use_sim_time}])
    
    # Publish a TF transform from base_link_stabilized (IMU) to base_link to support gravity-aligned SLAM.
    imu_to_tf_node = Node(
        package='rtabmap_util', executable='imu_to_tf', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'fixed_frame_id':'base_link_stabilized',
            'base_frame_id':'base_link'}])
        
    # Synchronize RGB + Depth images into unified RGBD messages for the RTAB-Map pipeline.
    vslam_node = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        namespace='rtabmap',
        parameters=[vslam_params],
        remappings=[('rgb/image', '/camera/rgb/image'),
                    ('rgb/camera_info', '/camera/rgb/camera_info'),
                    ('depth/image', '/camera/depth/image')])

    # Compute visual odometry (VO) from RGBD data: Publishes /rtabmap/odom for SLAM + navigation.
    rtabmap_odom_node = Node(
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        namespace='rtabmap',
        parameters=[vslam_params, {'odom_frame_id': 'odom'}],
        remappings=vslam_remappings,
        arguments=["--ros-args", "--log-level", 'warn'])

    # Full SLAM mode: Build the map, perform loop closures, update /map to /odom transform.
    rtabmap_slam_node = Node(
        condition=UnlessCondition(LaunchConfiguration('localization')),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        namespace='rtabmap',
        parameters=[vslam_params],
        remappings=vslam_remappings,
        arguments=['-d'])       # This will delete the previous database (~/.ros/rtabmap.db)
            
    # Localization-only mode: Disable map growth, use existing map for pose estimation.
    rtabmap_loc_node = Node(
        condition=IfCondition(LaunchConfiguration('localization')),
        package='rtabmap_slam', executable='rtabmap', output='screen',
        namespace='rtabmap',
        parameters=[vslam_params,
                    {'Mem/IncrementalMemory': 'False',
                     'Mem/InitWMWithAllNodes': 'True'}],
        remappings=vslam_remappings)
            
    # RTAB-Map visualization GUI (optional): Display map, graph, keyframes and point clouds.
    rtabmap_viz_node = Node(
        condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
        package='rtabmap_viz', executable='rtabmap_viz', output='screen',
        namespace='rtabmap',
        parameters=[vslam_params],
        remappings=vslam_remappings)
        
    # Convert RTAB-Map odometry into PX4 VehicleOdometry messages (optional for PX4 control).
    ros_odom_to_vehcl = Node(
        package='uav_navigation', executable='ros_odometry_to_vehicle_odometry', output='screen',
        parameters=[{'map_frame_id': 'map',
                    'use_sim_time': use_sim_time}],
        remappings=[('odom', '/rtabmap/odom')])

    # Generate a clean XYZ point cloud from depth + camera info: Use this in RViz or Nav2 costmap.
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

    # Optional: include the full Nav2 navigation stack (global planner, controller, BT navigator).
    nav2_launch_incl = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([nav2_launch]),
        launch_arguments=[
            ('use_sim_time', 'true'),
            ('params_file', nav2_params_file)])

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
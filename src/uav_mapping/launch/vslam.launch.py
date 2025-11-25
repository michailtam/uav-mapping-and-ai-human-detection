# NOTE: This code is based on this example: https://github.com/matlabbe/rtabmap_drone_example/tree/ros2

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


"""
Function to setup vSLAM
"""
def launch_setup(context, *args, **kwargs):

    # Package and file paths
    use_sim_time = LaunchConfiguration("use_sim_time")

    vslam_params = {
        # Use prefixed frame name from Gazebo
        'frame_id':'x650_0/base_footprint',                     # Robot body frame used for odometry and TF tree (child frame of odom)
        'guess_frame_id':'x650_0/base_footprint_stabilized',    # Frame providing motion prediction (IMU-based stabilization for better VO)
        'approx_sync': True,                        # Require exact RGB/depth timestamp sync (better accuracy, strict timing)
        'use_sim_time': use_sim_time,               # Use Gazebo simulated clock instead of system time
        'publish_tf': True,                         # Publish TF transforms (map->odom and odom->base_link)
        'subscribe_rgbd': True,                     # Enable RGB-D input (combined RGB + depth image subscription)
        'subscribe_odom_info': True,                # Subscribe to odom info (motion guesses and covariance for better matching)
        'wait_imu_to_init': True,                   # Wait until IMU provides stable gravity direction before starting SLAM
        'wait_for_transform': 0.5,                  # Max TF wait time (sec) for transforms between camera & base_link
        'use_action_for_goal': False,               # Do NOT use Nav2 NavigateToPose via RTAB-Map (Nav2 handles goals itself)
        'odom_frame_id': 'x650_0/odom',                    # odometry frame name
        'map_frame_id': 'map',                      # global map frame

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
    vslam_remappings=[('imu', '/imu/data')]
    
    # Filter raw IMU data and compute orientation (roll/pitch/yaw) using the Madgwick filter.
    imu_orientation_node = Node(
        package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
        parameters=[{
            'use_mag':False,
            'world_frame':'enu',
            'publish_tf':False,
            'use_sim_time': use_sim_time}])
    
    # Publish a TF transform from base_footprint_stabilized (IMU) to base_footprint to support gravity-aligned SLAM.
    imu_to_tf_node = Node(
        package='rtabmap_util', executable='imu_to_tf', output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'fixed_frame_id':'x650_0/base_footprint_stabilized',
            'base_frame_id':'x650_0/base_footprint'}])
        
    # Synchronize RGB + Depth images into unified RGBD messages for the RTAB-Map pipeline.
    vslam_node = Node(
        package='rtabmap_sync', executable='rgbd_sync', output='screen',
        namespace='rtabmap',
        parameters=[vslam_params],
        remappings = [
                        ("rgb/image", "/camera/image_optical"),
                        ("depth/image", "/camera/depth_image_optical"),
                        ("rgb/camera_info", "/camera/depth/camera_info_optical")])

    # Compute visual odometry (VO) from RGBD data: Publishes /rtabmap/odom for SLAM + navigation.
    rtabmap_odom_node = Node(
        package='rtabmap_odom', executable='rgbd_odometry', output='screen',
        namespace='rtabmap',
        parameters=[vslam_params, {'odom_frame_id': 'x650_0/odom'}],
        remappings=vslam_remappings,
        arguments=["--ros-args", "--log-level", 'warn'])

    # Full SLAM mode: Build the map, perform loop closures, update /map to /odom transform.
    # Always run SLAM when this launch is started so mapping occurs every flight.
    # Do not delete the previous database by default so maps are incremental across runs.
    rtabmap_slam_node = Node(
        package='rtabmap_slam', executable='rtabmap', output='screen',
        namespace='rtabmap',
        parameters=[vslam_params],
        remappings=vslam_remappings)
            
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
        parameters=[{'map_frame_id': 'odom',
                    'use_sim_time': use_sim_time}],
        remappings=[('odom', '/rtabmap/odom')])

    # Generate a clean XYZ point cloud from depth + camera info: Use this in RViz or Nav2 costmap.
    rtabmap_util_node = Node(
        package='rtabmap_util', executable='point_cloud_xyz', output='screen',
        parameters=[{'decimation': 2,
                        # increased for outdoor mapping; set to sensor max range
                        'max_depth': 20.0,
                        # coarser voxels reduce CPU and still give useful cloud
                        'voxel_size': 0.05,
                        'use_sim_time': use_sim_time}],
        remappings=[('depth/image', '/camera/depth_image_optical'),
                    ('depth/camera_info', '/camera/depth/camera_info_optical'),
                    ('cloud', '/camera/cloud')])

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
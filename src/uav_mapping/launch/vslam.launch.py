from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    
    use_sim_time = LaunchConfiguration("use_sim_time")

    vslam_params ={
        'frame_id':'base_link',
        'odom_frame_id': 'x650_0/odom',
        'guess_frame_id':'base_link_stabilized',
        'approx_sync': False,
        'use_sim_time': use_sim_time,
        'subscribe_rgbd':True,
        'subscribe_odom_info':False,
        'wait_imu_to_init': True,
        'wait_for_transform': 0.5,
        'use_action_for_goal':False, 
        
        # RTAB-Map's parameters should be strings
        'Optimizer/GravitySigma': '0.1',
        'Vis/FeatureType': '10',
        'Kp/DetectorStrategy': '10',

        # Grid parameters are useful for the 2D map result in RViz
        'Grid/MapFrameProjection': 'true',
        'NormalsSegmentation': 'false',
        'Grid/MaxGroundHeight': '1.15' ,
        'Grid/MaxObstacleHeight': '1.75',
        'RGBD/StartAtOrigin': 'true'
    }
    
    vslam_remappings=[('/imu/data', '/imu/data_raw'),  # Required for SLAM stability
                      ('/map', '/map'),  # This is where the 2D occupancy grid is published for visualization
                      ('/camera/image','/camera/rgb/image'),
                      ('/camera/camera_info','/camera/rgb/camera_info'),
                      ('/camera/depth_image','/camera/depth/image'),
                    # ('navigate_to_pose', '/navigate_to_pose'), # nav2 action server
                      # For humble: https://github.com/ros2/ros2/issues/1312
                    # ('navigate_to_pose/_action/feedback', '/navigate_to_pose/_action/feedback'),
                    # ('navigate_to_pose/_action/status', '/navigate_to_pose/_action/status'),
                    # ('navigate_to_pose/_action/cancel_goal', '/navigate_to_pose/_action/cancel_goal'),
                    # ('navigate_to_pose/_action/get_result', '/navigate_to_pose/_action/get_result'),
                    # ('navigate_to_pose/_action/send_goal', '/navigate_to_pose/_action/send_goal'),
                      ]
    
    pkg_uav_navigation_share_dir = get_package_share_directory('uav_navigation')
    nav2_params_file = os.path.join(pkg_uav_navigation_share_dir, 'config','nav2_params.yaml')

    pkg_nav2_bringup_share_dir = get_package_share_directory('nav2_bringup')
    nav2_launch = PathJoinSubstitution([pkg_nav2_bringup_share_dir, 'launch', 'navigation_launch.py'])
    
    rviz_config_file = os.path.join(
        get_package_share_directory('uav_mapping'), 'rviz', 'slam_config.rviz')
    
    return [

        # compute imu orientation
        Node(
            package='imu_filter_madgwick', executable='imu_filter_madgwick_node', output='screen',
            parameters=[{
              'use_mag':False,
              'world_frame':'enu',
              'publish_tf':False,
              'use_sim_time': use_sim_time}]),

        Node(
            package='rtabmap_util', executable='imu_to_tf', output='screen',
            parameters=[{
              'use_sim_time': use_sim_time,
              'fixed_frame_id':'base_link_stabilized',
              'base_frame_id':'base_link'}]),
        
        # VSLAM nodes:
        Node(
            package='rtabmap_sync', executable='rgbd_sync', output='screen',
            namespace='rtabmap',
            parameters=[vslam_params],
            remappings=[('rgb/image', '/camera/rgb/image'),             
                        ('rgb/camera_info', '/camera/rgb/camera_info'),     
                        ('depth/image', '/camera/depth/image')]),       

        Node(
            package='rtabmap_odom', executable='rgbd_odometry', output='screen',
            namespace='rtabmap',
            parameters=[vslam_params],
            remappings=vslam_remappings,
            arguments=["--ros-args", "--log-level", 'warn']),

        # SLAM mode:
        Node(
            condition=UnlessCondition(LaunchConfiguration('localization')),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            namespace='rtabmap',
            parameters=[vslam_params],
            remappings=vslam_remappings,
            arguments=['-d']), # This will delete the previous database (~/.ros/rtabmap.db)
            
        # Localization mode:
        Node(
            condition=IfCondition(LaunchConfiguration('localization')),
            package='rtabmap_slam', executable='rtabmap', output='screen',
            namespace='rtabmap',
            parameters=[vslam_params,
                        {'Mem/IncrementalMemory': 'False',
                         'Mem/InitWMWithAllNodes': 'True'}],
            remappings=vslam_remappings),
            
        Node(
            condition=IfCondition(LaunchConfiguration('rtabmap_viz')),
            package='rtabmap_viz', executable='rtabmap_viz', output='screen',
            namespace='rtabmap',
            parameters=[vslam_params],
            remappings=vslam_remappings),
        
        Node(
            condition=IfCondition(LaunchConfiguration('rviz')),
            package='rviz2', executable='rviz2', output='screen',
            arguments=["-d", rviz_config_file],
            remappings=vslam_remappings),

        # # Publish corresponding TF to main components, based on sdf
        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     arguments=[
        #         '0.12', '0.03', '0.242',          # x y z
        #         '-1.570796327', '0', '-1.570796327',  # roll pitch yaw
        #         'base_link', 'camera_link'        # frame-id child-frame-id
        #     ]),

        # Node(
        #     package='tf2_ros', executable='static_transform_publisher', output='screen',
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     arguments=[
        #         '0', '0', '0.14',            # x y z
        #         '0', '0', '0',               # roll pitch yaw
        #         'base_link', 'middle'        # frame-id child-frame-id
        #     ],
        # ),
        
        Node(
            package='uav_navigation', executable='odometry_to_vehicle_odometry', output='screen',   # TODO:
            parameters=[{'map_frame_id': 'map',
                        'use_sim_time': use_sim_time}],
            remappings=[('odom', '/rtabmap/odom')]
        ),
        
        # Node(package='joy', executable='joy_node', output='screen',
        #     parameters=[{'autorepeat_rate': 20.0,
        #                  'use_sim_time': use_sim_time}]
        # ),

        # Node(package='teleop_twist_joy', executable='teleop_node', output='screen',
        #     parameters=[joy_config_file_path,
        #                 {'use_sim_time': use_sim_time}]
        # ),

        Node(
            package='rtabmap_util', executable='point_cloud_xyz', output='screen',
            parameters=[{'decimation': 2,
                         'max_depth': 3.0,
                         'voxel_size': 0.02,
                         'use_sim_time': use_sim_time}],
            remappings=[('depth/image', '/camera/depth/image'),
                        ('depth/camera_info', '/camera/rgb/camera_info'),
                        ('cloud', '/camera/cloud')]),

        # Node(
        #     package='rtabmap_costmap_plugins', executable='voxel_marker', output='screen',
        #     namespace="local_costmap",
        #     parameters=[{'use_sim_time': use_sim_time}]),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_link_to_footprint',
            arguments=['0', '0', '0', '0', '0', '0', 'x650_0/base_footprint', 'base_link']
        ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([nav2_launch]),
        #     launch_arguments=[
        #         ('use_sim_time', 'true'),
        #         ('params_file', nav2_params_file)
        #     ]
        # )
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
            name='rviz', 
            default_value='true',
            description='Launch rviz'
        ),
        DeclareLaunchArgument(
            'localization',
            default_value='false', 
            description='Launch in localization mode.'),
        
        OpaqueFunction(function=launch_setup)
    ])
import rclpy
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node
from rclpy.node import Node as RclNode


def launch_px4_prop_js_pub(context, *args, **kwargs):
    # Use a tiny rclpy node inside the launch to wait for the PX4 topic /fmu/out/actuator_motors.
    rclpy.init(args=None)
    tmp_node = RclNode('wait_for_px4_actuator_topic')
    target_topic = '/fmu/out/actuator_motors'
    tmp_node.get_logger().info(f'Waiting for topic: {target_topic}')

    found = False
    while not found:
        topics = tmp_node.get_topic_names_and_types()
        found = any(name == target_topic for name, _ in topics)
        if not found:
            rclpy.spin_once(tmp_node, timeout_sec=0.5)

    tmp_node.get_logger().info(
        f'PX4 actuator topic detected -> launching px4_prop_js_publisher and px4_ctrl'
    )
    tmp_node.destroy_node()
    rclpy.shutdown()

    # Start publishing joint states delayed from PX4 (not from Gazebo)
    # Note: This is required for RViz to display propellers and rotating on place. 
    px4_prop_joint_states_pub_node = Node(
        package='uav_offboard_ctrl',
        executable='px4_prop_js_publisher',  # Be sure this matches CMakeLists setting
        name='px4_prop_js_publisher',
        parameters=[{
            'use_sim_time': True
        }],
        output='screen'
    )

    # IMPORTANT: OpaqueFunction must return the actions, not a LaunchDescription
    return [
        px4_prop_joint_states_pub_node,
    ]


def generate_launch_description():
    ld = LaunchDescription()
    
    # Add the OpaqueFunction that waits for PX4 and launches the two nodes
    ld.add_action(OpaqueFunction(function=launch_px4_prop_js_pub))

    return ld

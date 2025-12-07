#!/usr/bin/env python3
import rclpy
import tf_transformations
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class Nav2SimpleCommander(Node):

    def __init__(self):
        super().__init__("nav2_simple_commander")
        
        pose_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        self._nav = BasicNavigator()

        self._ini_pose_clbk = self.create_subscription(PoseStamped, "/start_pose_from_px4", self.initial_pose_clbk, pose_qos)
        self._goal_pose_clbk = self.create_subscription(PoseStamped, "/goal_pose_from_px4", self.goal_pose_clbk, pose_qos)

        self._nav.waitUntilNav2Active()

        self._start_pose = None
        self._goal_pose = None

    def initial_pose_clbk(self, msg: PoseStamped):
        '''
        # Callback method to set the initial pose
        :type msg: PoseStamped
        '''
        self.get_logger().info(f"Initial pose received: {msg.pose.position}")

        # Transform the Euler angels to Quaternions
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)
        
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self._nav.get_clock().now().to_msg()
        initial_pose.pose.position.x = msg.pose.position.x
        initial_pose.pose.position.y = msg.pose.position.y
        initial_pose.pose.position.z = msg.pose.position.z
        initial_pose.pose.orientation.x = q_x
        initial_pose.pose.orientation.y = q_y
        initial_pose.pose.orientation.z = q_z
        initial_pose.pose.orientation.w = q_w
        
        self._nav.setInitialPose(initial_pose)  # Send the initial pose to the navigation stack

    def goal_pose_clbk(self, msg: PoseStamped):
        '''
        # Callback method to set the goal pose
        :type msg: PoseStamped
        '''
        self.get_logger().info(f"Goal pose received: {msg.pose.position}")

        # Transform the Euler angels to Quaternions
        q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w)
        
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self._nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = msg.pose.position.x
        goal_pose.pose.position.y = msg.pose.position.y
        goal_pose.pose.position.z = msg.pose.position.z
        goal_pose.pose.orientation.x = q_x
        goal_pose.pose.orientation.y = q_y
        goal_pose.pose.orientation.z = q_z
        goal_pose.pose.orientation.w = q_w

        self._nav.goToPose()    # Send the goal pose to the navigation stack

        # Waits until the goal position is reached and exits
        while not self._nav.isTaskComplete():
            feedback = self._nav.getFeedback()     # Get the current position of the robot
            self.get_logger().info(f"Current position: {feedback}")
        
        result = self._nav.getResult()
        self.get_logger().info(f"Result: {result}")


def main(args=None):
    rclpy.init(args=args)
    node = Nav2SimpleCommander()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
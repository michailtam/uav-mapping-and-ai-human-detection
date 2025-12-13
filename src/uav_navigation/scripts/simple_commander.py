#!/usr/bin/env python3
import rclpy
import tf_transformations
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from uav_navigation.srv import SetNavData


class Nav2SimpleCommander(Node):

    def __init__(self):
        super().__init__("nav2_simple_commander_server")

        self._nav = BasicNavigator()
        # self.get_logger().warn("wait until Nav2 is active")
        # self._nav.waitUntilNav2Active(localizer='slam_toolbox')

        # Start the server to handle start and goal pose from PX4
        self._nav2_data_server_ = self.create_service(SetNavData, "srv_nav_data", self.callback_nav_data)
        self.get_logger().warn("Nav2 simple commander server started and waiting for requests")

    def callback_nav_data(self, request: SetNavData.Request, response: SetNavData.Response):
        
        self._start = request.start
        self._goal = request.goal
        
        start_x = request.start.pose.position.x
        start_y = request.start.pose.position.y
        start_z = request.start.pose.position.z
        goal_x = request.goal.pose.position.x
        goal_y = request.goal.pose.position.y
        goal_z = request.goal.pose.position.z

        self.get_logger().info(f"Received start pose: {start_x} {start_y} {start_z} and goal pose: {goal_x} {goal_y} {goal_z}")
        
        response.accepted = True
        return response
        

def main(args=None):
    rclpy.init(args=args)
    node = Nav2SimpleCommander()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
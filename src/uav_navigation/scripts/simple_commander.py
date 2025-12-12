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
        super().__init__("nav2_simple_commander")

        self._nav = BasicNavigator()
        self._nav.waitUntilNav2Active(localizer='slam_toolbox')

        # Start the client to request start and goal pose from PX4
        self._nav2_nav_data_client_ = self.create_client(SetNavData, "srv_nav_data")
        
    def call_nav_data(self, code):
        # Wait until server has the data
        while not self._nav2_nav_data_client_.wait_for_service():
            self.get_logger().warn("Waiting for set_nav_data server...")

        request = SetNavData.Request()
        request.code = code

        future = self._nav2_nav_data_client_.call_async(request)
        future.add_done_callback(self.callback_nav_data)

    def callback_nav_data(self, future):
        response = future.result()
        self.get_logger().info("Got start pose:"+str(response.start)+" and goal pose:"+str(response.goal))

        # self._nav.setInitialPose(initial_pose)  # Send the initial pose to the navigation stack

        # self._nav.goToPose()    # Send the goal pose to the navigation stack

        # # Waits until the goal position is reached and exits
        # while not self._nav.isTaskComplete():
        #     feedback = self._nav.getFeedback()     # Get the current position of the robot
        #     self.get_logger().info(f"Current position: {feedback}")
        
        # result = self._nav.getResult()
        # self.get_logger().info(f"Result: {result}")
        

def main(args=None):
    rclpy.init(args=args)
    node = Nav2SimpleCommander()
    node.call_nav_data(1)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from rclpy.node import Node


class Nav2SimpleCommander(Node):

    def __init__(self):
        super().__init__("nav2_simple_commander_server")

        self._nav2 = BasicNavigator()
        # self.get_logger().warn("wait until Nav2 is active")
        # self._nav.waitUntilNav2Active(localizer='slam_toolbox')

        # Start the server to handle start and goal pose from PX4
        self.nav2_home_pose_client_ = self.create_service(SetInitialPose, "set_home_pose", self.set_initial_pose_cbk)
        self.nav2_goal_pose_client_ = self.create_service(SetGoalPose, "set_goal_pose", self.set_goal_pose_cbk)
        self.get_logger().warn("Nav2 simple commander server started and waiting for requests")

    def set_initial_pose_cbk(self, request: SetInitialPose.Request, response: SetInitialPose.Response):
    
        # Safe the received data
        self.home_ = request.home
        self.get_logger().info(f"Received home pose: {self.home_}")

        response.accepted = True
        self._nav2.setInitialPose(self.home_)
        
        self.get_logger().info("Initial pose set (i.e. home pose))")
        return response
    
    def set_goal_pose_cbk(self, request: SetInitialPose.Request, response: SetInitialPose.Response):
    
        # Safe the received data
        self.goal_ = request.goal
        self.get_logger().info(f"Received goal pose: {self.home_}")

        response.accepted = True
        self._nav2.goToPose(self.goal_)

        # Publish odometry data until the navigation has finished
        while not self._nav2.isTaskComplete():
            feedback = self._nav2.getFeedback()     # The current position of the robot
            self.get_logger().info(f"The current position is: {feedback}")

        return response


def main(args=None):
    rclpy.init(args=args)
    node = Nav2SimpleCommander()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
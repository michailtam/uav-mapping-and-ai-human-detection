#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped


class CommandNode(Node):
    """
    CommandNode
    -----------
    Skeleton of the Command / FSM node described in the thesis.

    In the full system, this node:
      - supervises PX4 state and mission progress,
      - switches between nominal mission and collision avoidance (Offboard),
      - forwards A* paths to the low-level controller.

    Include:
      - subscriptions to /obstacle_detected and /astar_failed,
      - a simple one-shot publication of a test /goal_pose to trigger planning.
    """

    def __init__(self):
        super().__init__('command_node')

        # Subscribe to avoidance-related flags.
        self.obstacle_sub = self.create_subscription(Bool, '/obstacle_detected', self.obstacle_cb, 10)
        self.astar_failed_sub = self.create_subscription(Bool, '/astar_failed', self.astar_failed_cb, 10)

        # Publisher for the local goal used by GoalProjector and A*.
        self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        # Simple mechanism to send a single test goal at startup.
        self.goal_sent = False
        self.timer = self.create_timer(2.0, self.timer_cb)

        self.get_logger().info('CommandNode (FSM skeleton) started.')

    #### Periodic goal sender (for testing) ####

    def timer_cb(self):
        """
        After a short delay, publish one test goal.

        In the real system, goals would come from the mission
        (e.g. converted from GPS waypoints), not from here.
        """
        if self.goal_sent: 
            return

        goal = PoseStamped()
        goal.header.frame_id = 'odom'  # Should match costmap frame.
        goal.header.stamp = self.get_clock().now().to_msg()

        # Simple test goal straight ahead in x.
        goal.pose.position.x = 5.0
        goal.pose.position.y = 0.0
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)
        self.get_logger().info('Published test goal to /goal_pose: x=5.0, y=0.0')
        self.goal_sent = True

    #### Flag callbacks ####

    def obstacle_cb(self, msg: Bool):
        """
        Receive obstacle detection flag.

        In the complete FSM, this would trigger transitions from
        "Mission" to "Avoidance" mode as described in the PX4 integration.
        """
        self.get_logger().info(f'Obstacle detected: {msg.data}')

    def astar_failed_cb(self, msg: Bool):
        """
        Receive A* failure flag.

        In the full system, this would handle recovery or fallback behaviours
        (e.g. hover, emergency braking, or Bug-like strategies).
        """
        self.get_logger().info(f'A* failed: {msg.data}')


def main(args=None):
    rclpy.init(args=args)
    node = CommandNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

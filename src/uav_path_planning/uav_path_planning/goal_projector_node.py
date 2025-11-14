#!/usr/bin/env python3
import math
from typing import Optional, Tuple

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry


class GoalProjector(Node):
    """
    Project the global goal into the local costmap.

    This node limits the (possibly far) mission goal to a reachable intermediate goal inside the local 
    A*-based collision avoidance costmap, along the current direction of motion.
    """

    def __init__(self):
        super().__init__('goal_projector')

        # How often to recompute the projected goal (Hz).
        self.declare_parameter('publish_rate', 5.0)

        # Angular search limits around the straight line to the goal (deg).
        self.declare_parameter('max_angle_deg', 30.0)
        self.declare_parameter('angle_step_deg', 15.0)

        self.publish_rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.max_angle = math.radians(self.get_parameter('max_angle_deg').get_parameter_value().double_value)
        self.angle_step = math.radians(self.get_parameter('angle_step_deg').get_parameter_value().double_value)

        # Latest global goal, odometry and costmap.
        self._goal: Optional[PoseStamped] = None
        self._odom: Optional[Odometry] = None
        self._costmap: Optional[OccupancyGrid] = None

        # Global goal from mission planner / higher-level node.
        self.goal_sub = self.create_subscription(PoseStamped, '/goal_pose', self.goal_cb, 10)

        # UAV current pose (used as start for ray sampling).
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Local costmap used later by the A* planner.
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.costmap_cb, 10)

        # Projected goal inside the local costmap (intermediate target).
        self.projected_pub = self.create_publisher(PoseStamped, '/projected_goal', 10)

        # Periodically recompute a valid projected goal.
        self.timer = self.create_timer(1.0 / self.publish_rate, self.timer_cb)

        self.get_logger().info('GoalProjector node started.')

    #### Callbacks ####

    def goal_cb(self, msg: PoseStamped):
        """ Store latest global goal. """
        self._goal = msg
        self.get_logger().info(
            f'Received new global goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})'
        )

    def odom_cb(self, msg: Odometry):
        """ Store latest odometry. """
        self._odom = msg

    def costmap_cb(self, msg: OccupancyGrid):
        """ Store latest local costmap. """
        self._costmap = msg

    #### Main logic ####

    def timer_cb(self):
        """
        Periodic projection step.

        1. Check that odom, goal and costmap are available.
        2. Compute bearing from UAV to global goal.
        3. Sample along that ray and small angular deviations.
        4. Publish the first free/unknown cell as /projected_goal.
        """
        if self._goal is None or self._odom is None or self._costmap is None:
            return

        # Current UAV position.
        rx = self._odom.pose.pose.position.x
        ry = self._odom.pose.pose.position.y

        # Global goal position.
        gx = self._goal.pose.position.x
        gy = self._goal.pose.position.y

        dx = gx - rx
        dy = gy - ry
        dist = math.hypot(dx, dy)

        if dist < 1e-3:
            # Already at the goal.
            return

        # Base angle to the goal.
        base_angle = math.atan2(dy, dx)

        # Build list of angle deviations: 0, +step, -step, +2*step, -2*step, ...
        angles = [0.0]
        a = self.angle_step
        while a <= self.max_angle + 1e-6:
            angles.append(a)
            angles.append(-a)
            a += self.angle_step

        projected: Optional[Tuple[float, float]] = None

        # Try each angle until we find a traversable cell.
        for da in angles:
            theta = base_angle + da
            projected = self._sample_along_direction(rx, ry, theta, dist)
            if projected is not None:
                break

        if projected is None:
            # No valid projected goal in the local costmap.
            return

        px, py = projected

        # Build and publish projected goal message.
        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = self._costmap.header.frame_id
        out.pose.position.x = px
        out.pose.position.y = py
        out.pose.position.z = self._goal.pose.position.z
        out.pose.orientation.w = 1.0
        self.projected_pub.publish(out)

    #### Helpers ####

    def _sample_along_direction(
        self, rx: float, ry: float, theta: float, max_dist: float
    ) -> Optional[Tuple[float, float]]:
        """
        Sample points starting from the UAV along direction theta, up to max_dist.

        Return the first point that falls in a free or unknown cell in the costmap.
        """
        cm = self._costmap
        res = cm.info.resolution
        step = 0.5 * res

        d = res  # start slightly in front of the UAV
        while d <= max_dist:
            x = rx + d * math.cos(theta)
            y = ry + d * math.sin(theta)
            idx = self._world_to_index(x, y)
            if idx is None:
                d += step
                continue

            val = cm.data[idx]
            # Treat free (0) and unknown (-1) as traversable, like in the thesis.
            if val == 0 or val == -1:
                return x, y

            d += step

        return None

    def _world_to_index(self, x: float, y: float) -> Optional[int]:
        """
        Convert the world coordinates to a flat index in the OccupancyGrid.
        """
        cm = self._costmap
        origin_x = cm.info.origin.position.x
        origin_y = cm.info.origin.position.y
        res = cm.info.resolution
        width = cm.info.width
        height = cm.info.height

        mx = int((x - origin_x) / res)
        my = int((y - origin_y) / res)

        if mx < 0 or my < 0 or mx >= width or my >= height:
            return None

        return my * width + mx


def main(args=None):
    rclpy.init(args=args)
    node = GoalProjector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

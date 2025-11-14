#!/usr/bin/env python3
import math
from typing import Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped


class ObstacleDetector(Node):
    """
    ObstacleDetector node (Drone, Appendix B.3 of the thesis).

    Monitors a rectangular corridor in front of the UAV in the local costmap.
    Even though the drone moves in 3D, the detector operates on the XY
    projection of the motion: the costmap is assumed to represent a 2D slice
    of the environment around the current flight altitude.

    A cell triggers detection if:
        - it is inside the corridor aligned with the vector (drone -> goal),
        - 0 <= forward <= L_max,
        - |lateral| <= half_width,
        - occupancy > 0 (obstacle or inflated region).
    """

    def __init__(self):
        super().__init__('obstacle_detector')

        # Corridor half-width [m] around the line drone->goal
        self.declare_parameter('half_width', 1.5)
        # How often to check the corridor [s]
        self.declare_parameter('check_interval', 0.2)
        # Sampling step used to find L_max inside the costmap [m]
        self.declare_parameter('step_size', 0.1)

        self.half_width = self.get_parameter(
            'half_width').get_parameter_value().double_value
        self.check_interval = self.get_parameter(
            'check_interval').get_parameter_value().double_value
        self.step_size = self.get_parameter(
            'step_size').get_parameter_value().double_value

        # Latest map, goal and odometry
        self._costmap: Optional[OccupancyGrid] = None
        self._odom: Optional[Odometry] = None
        self._goal_x: Optional[float] = None
        self._goal_y: Optional[float] = None

        # Subscriptions:
        # local dynamic costmap (2D slice at current altitude)
        self.costmap_sub = self.create_subscription(
            OccupancyGrid, '/local_costmap/costmap', self.costmap_cb, 10
        )
        # UAV odometry in the same frame as the costmap (e.g. "odom")
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_cb, 10
        )
        # Global / mission goal in the same frame as the costmap
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose', self.goal_cb, 10
        )

        # Boolean flag used by the Command / Mode Manager
        self.flag_pub = self.create_publisher(Bool, '/obstacle_detected', 10)

        # Periodic check of the corridor
        self.timer = self.create_timer(self.check_interval, self.timer_cb)

        self.get_logger().info('ObstacleDetector node started.')

    #### Callbacks ####

    def costmap_cb(self, msg: OccupancyGrid):
        """Store latest local costmap."""
        self._costmap = msg

    def odom_cb(self, msg: Odometry):
        """Store latest UAV odometry."""
        self._odom = msg

    def goal_cb(self, msg: PoseStamped):
        """Store latest mission goal coordinates in the costmap frame."""
        self._goal_x = msg.pose.position.x
        self._goal_y = msg.pose.position.y

    #### Main check ####

    def timer_cb(self):
        """
        Periodic obstacle check in a corridor in front of the UAV.

        1. Get drone position (x,y) from /odom.
        2. Get goal position (x,y) from /goal_pose.
        3. Compute direction vector d and perpendicular p in the XY plane.
        4. Compute L_max: maximum forward distance within the costmap.
        5. Scan all cells; for each cell center, compute:
             forward = r · d
             lateral = r · p
           and trigger flag if inside corridor and occupancy > 0.
        """
        if self._costmap is None or self._odom is None:
            return
        if self._goal_x is None or self._goal_y is None:
            return

        cm = self._costmap

        # Drone position in costmap frame (XY plane).
        drone_x = self._odom.pose.pose.position.x
        drone_y = self._odom.pose.pose.position.y
        # Note: drone_z exists (3D motion), but the detector uses XY projection.
        # The costmap itself is assumed to represent the environment around
        # the current flight altitude.

        # Goal position in costmap frame.
        goal_x = self._goal_x
        goal_y = self._goal_y

        dx = goal_x - drone_x
        dy = goal_y - drone_y
        L = math.hypot(dx, dy)
        if L < 0.01:
            # Goal is essentially at the drone position.
            self._publish_flag(False)
            return

        # Unit direction vector from drone to goal.
        dir_x = dx / L
        dir_y = dy / L
        # Perpendicular unit vector (left/right).
        perp_x = -dir_y
        perp_y = dir_x

        # Compute maximum usable length L_max inside the costmap.
        L_max = self._compute_Lmax(drone_x, drone_y, dir_x, dir_y, L)

        if L_max <= 0.0:
            self._publish_flag(False)
            return

        # Scan all cells and check occupancy within corridor.
        has_obstacle = self._scan_corridor_for_obstacles(
            cm, drone_x, drone_y, dir_x, dir_y, perp_x, perp_y, L_max
        )

        self._publish_flag(has_obstacle)

    #### Helpers ####

    def _compute_Lmax(
        self,
        drone_x: float,
        drone_y: float,
        dir_x: float,
        dir_y: float,
        L: float,
    ) -> float:
        """
        Find L_max: maximum forward distance along dir such that
        the point stays inside the costmap. Follows the idea of Appendix B.3.
        """
        cm = self._costmap
        step = self.step_size
        L_max = L

        # Sample points along the ray (drone → goal).
        # As soon as we leave the map, we stop and set L_max to that distance.
        ell = step
        while ell <= L:
            x = drone_x + ell * dir_x
            y = drone_y + ell * dir_y
            idx = self._world_to_index(cm, x, y)
            if idx is None:
                L_max = ell
                break
            ell += step

        return L_max

    def _scan_corridor_for_obstacles(
        self,
        cm: OccupancyGrid,
        drone_x: float,
        drone_y: float,
        dir_x: float,
        dir_y: float,
        perp_x: float,
        perp_y: float,
        L_max: float,
    ) -> bool:
        """
        Scan all cells in the costmap and check if any occupied cell lies inside
        the rectangular corridor:

            0 <= forward <= L_max
            |lateral| <= half_width

        forward/lateral are computed by projecting the vector from the drone
        to the cell center onto (dir, perp).
        """
        width = cm.info.width
        height = cm.info.height
        data = cm.data

        res = cm.info.resolution
        origin_x = cm.info.origin.position.x
        origin_y = cm.info.origin.position.y

        for my in range(height):
            for mx in range(width):
                idx = my * width + mx
                val = data[idx]

                # Only consider cells with positive occupancy value:
                #   0       -> free
                #   -1      -> unknown
                #   > 0     -> obstacle or inflated region
                if val <= 0:
                    continue

                # Cell center in world coordinates (XY).
                wx = origin_x + (mx + 0.5) * res
                wy = origin_y + (my + 0.5) * res

                # Vector from drone to this cell.
                rx = wx - drone_x
                ry = wy - drone_y

                # Longitudinal and lateral components in the local frame.
                forward = rx * dir_x + ry * dir_y
                lateral = rx * perp_x + ry * perp_y

                # Check if we are inside the corridor.
                if 0.0 <= forward <= L_max and abs(lateral) <= self.half_width:
                    # Found an occupied cell in the corridor.
                    return True

        return False

    def _world_to_index(self, cm: OccupancyGrid, x: float, y: float) -> Optional[int]:
        """
        Convert world (x,y) to a flat index in the OccupancyGrid.
        Used to check whether a point lies inside the map.
        """
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

    def _publish_flag(self, has_obstacle: bool):
        """Publish the /obstacle_detected flag."""
        msg = Bool()
        msg.data = has_obstacle
        self.flag_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

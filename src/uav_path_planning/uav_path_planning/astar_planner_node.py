#!/usr/bin/env python3
import heapq
import math
from typing import Optional, Tuple, List, Dict

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from std_msgs.msg import Bool


class AStarPlanner(Node):
    """
    Local A* planner operating on the dynamic costmap computes a collision-free path between the UAV current pose (/odom)
    and the projected local goal (/projected_goal) on the 2D OccupancyGrid.
    """

    def __init__(self):
        super().__init__('astar_planner')

        # How often planning is performed (Hz).
        self.declare_parameter('plan_rate', 2.0)
        self.plan_rate = self.get_parameter('plan_rate').get_parameter_value().double_value

        # Latest data: costmap, goal and odometry.
        self._costmap: Optional[OccupancyGrid] = None
        self._goal: Optional[PoseStamped] = None
        self._odom: Optional[Odometry] = None

        # Costmap from the perception layer (Nav2-like local costmap).
        self.costmap_sub = self.create_subscription(OccupancyGrid, '/local_costmap/costmap', self.costmap_cb, 10)

        # Intermediate goal inside the local costmap (from GoalProjector).
        self.goal_sub = self.create_subscription(PoseStamped, '/projected_goal', self.goal_cb, 10)

        # UAV current pose used as start state in the grid.
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)

        # Path for RViz visualization (absolute world frame).
        self.path_pub = self.create_publisher(Path, '/astar_path_visual', 10)

        # Path for the controller, expressed as a PoseArray.
        self.posearray_pub = self.create_publisher(PoseArray, '/astar_path', 10)

        # Boolean flag for planning failure.
        self.failed_pub = self.create_publisher(Bool, '/astar_failed', 10)

        # Periodic planning loop.
        self.timer = self.create_timer(1.0 / self.plan_rate, self.timer_cb)
        self.get_logger().info('AStarPlanner node started.')

        self.declare_parameter('connectivity', 16)  # 4, 8 or 16
        self.connectivity = self.get_parameter('connectivity').get_parameter_value().integer_value


    #### Callbacks ####

    def costmap_cb(self, msg: OccupancyGrid):
        """ Store latest costmap. """
        self._costmap = msg

    def goal_cb(self, msg: PoseStamped):
        """ Store latest projected goal. """
        self._goal = msg
        self.get_logger().info(f'Received projected goal: ({msg.pose.position.x:.2f}, {msg.pose.position.y:.2f})')

    def odom_cb(self, msg: Odometry):
        """ Store latest odometry. """
        self._odom = msg

    #### Planning loop ####

    def timer_cb(self):
        """
        Main planning step:
          1. Check that all inputs are available.
          2. Convert start/goal world poses into grid indices.
          3. Run A* on the OccupancyGrid.
          4. Publish path or failure flag.
        """
        if self._costmap is None or self._goal is None or self._odom is None:
            return

        # World coordinates of start (UAV) and goal.
        sx = self._odom.pose.pose.position.x
        sy = self._odom.pose.pose.position.y
        gx = self._goal.pose.position.x
        gy = self._goal.pose.position.y

        # Convert to grid indices.
        start_idx = self._world_to_index(sx, sy)
        goal_idx = self._world_to_index(gx, gy)

        if start_idx is None or goal_idx is None:
            self._publish_failed(True) # Start or goal outside costmap bounds.
            return

        # Run A* search.
        path_indices = self._astar_search(start_idx, goal_idx)
        if path_indices is None or len(path_indices) < 2:
            self._publish_failed(True) # No valid path found.
            return

        # Planning succeeded.
        self._publish_failed(False)
        self._publish_paths(path_indices)

    #### A* core ####

    def _astar_search(self, start: int, goal: int) -> Optional[List[int]]:
        """
        A* search on a 2D grid represented as a flat array.

        Free and unknown cells (0, -1) are traversable; occupied cells (e.g. 100)
        are blocked, following the occupancy coding discussed in the thesis.
        """
        cm = self._costmap
        width = cm.info.width
        height = cm.info.height
        data = cm.data

        def neighbors(idx: int) -> List[Tuple[int, float]]:
            """ Return neighbor indices and movement cost based on grid connectivity. """
            x = idx % width
            y = idx // width
            neigh = []
            moves = []

            cardinal_moves = [(1, 0), (-1, 0), (0, 1), (0, -1)]     # 4-connected: cardinal moves only
            diagonal_moves = [(1, 1), (1, -1), (-1, 1), (-1, -1)]   # 8-connected: add diagonals

            # 16-connected: add "knight-like" moves (2,1) and (1,2)
            knight_moves = [
                (2, 1),  (2, -1),  (-2, 1),  (-2, -1),
                (1, 2),  (1, -2),  (-1, 2),  (-1, -2)]

            if self.connectivity >= 4: moves += cardinal_moves  # Only up/down/left/right
            if self.connectivity >= 8: moves += diagonal_moves  # Classic diagonal
            if self.connectivity >= 16: moves += knight_moves   # "knight" (2,1) / (1,2) moves -> smoother angles

            for dx, dy in moves:
                nx = x + dx
                ny = y + dy

                # Bounds check
                if nx < 0 or ny < 0 or nx >= width or ny >= height:
                    continue

                nidx = ny * width + nx
                val = data[nidx]

                # Walkable: free (0) or unknown (-1)
                if val == 0 or val == -1:
                    # Movement cost = Euclidean distance in grid cell units
                    cost = math.hypot(dx, dy)
                    neigh.append((nidx, cost))

            return neigh


        def heuristic(a: int, b: int) -> float:
            """ Euclidean heuristic between two cells. """
            ax = a % width
            ay = a // width
            bx = b % width
            by = b // width
            return math.hypot(ax - bx, ay - by)

        # Priority queue (min-heap) storing (f_score, index).
        open_set: List[Tuple[float, int]] = []
        heapq.heappush(open_set, (0.0, start))

        # g_score: cost from start, f_score: g + heuristic.
        came_from: Dict[int, int] = {}
        g_score: Dict[int, float] = {start: 0.0}
        f_score: Dict[int, float] = {start: heuristic(start, goal)}

        closed = set()

        # Standard A* loop.
        while open_set:
            _, current = heapq.heappop(open_set)

            # Goal reached.
            if current == goal:
                return self._reconstruct_path(came_from, current)

            if current in closed:
                continue
            closed.add(current)

            for nidx, move_cost in neighbors(current):
                tentative_g = g_score[current] + move_cost
                if nidx in g_score and tentative_g >= g_score[nidx]:
                    continue

                came_from[nidx] = current
                g_score[nidx] = tentative_g
                f_score[nidx] = tentative_g + heuristic(nidx, goal)
                heapq.heappush(open_set, (f_score[nidx], nidx))

        return None # No path found.

    def _reconstruct_path(self, came_from: Dict[int, int], current: int) -> List[int]:
        """ Backtrack from goal to start using the came_from map. """
        path = [current]
        while current in came_from:
            current = came_from[current]
            path.append(current)
        path.reverse()
        return path

    #### Path publishing ####

    def _publish_failed(self, failed: bool):
        """ Publish planning status as a Bool flag. """
        msg = Bool()
        msg.data = failed
        self.failed_pub.publish(msg)

    def _publish_paths(self, path_indices: List[int]):
        """
        Publish:
          - nav_msgs/Path in the costmap frame (for RViz),
          - PoseArray relative to the first waypoint (for the controller).
        """
        cm = self._costmap
        width = cm.info.width

        # Convert indices to world coordinates.
        points: List[Tuple[float, float]] = []
        for idx in path_indices:
            x = idx % width
            y = idx // width
            wx, wy = self._map_to_world(x, y)
            points.append((wx, wy))

        # Path for visualization.
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = cm.header.frame_id

        for wx, wy in points:
            ps = PoseStamped()
            ps.header = path_msg.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.position.z = 0.0
            ps.pose.orientation.w = 1.0
            path_msg.poses.append(ps)

        self.path_pub.publish(path_msg)

        # PoseArray for the controller (relative frame).
        pose_array = PoseArray()
        pose_array.header = path_msg.header

        if not points:
            self.posearray_pub.publish(pose_array)
            return

        base_x, base_y = points[0]

        for i, (wx, wy) in enumerate(points):
            pose = Pose()
            pose.position.x = wx - base_x
            pose.position.y = wy - base_y
            pose.position.z = 0.0

            # Heading along the path segment.
            if i < len(points) - 1:
                nx, ny = points[i + 1]
                yaw = math.atan2(ny - wy, nx - wx)
            else:
                yaw = 0.0

            qz = math.sin(yaw / 2.0)
            qw = math.cos(yaw / 2.0)
            pose.orientation.z = qz
            pose.orientation.w = qw

            pose_array.poses.append(pose)

        self.posearray_pub.publish(pose_array)

    #### Grid to world conversion ####

    def _world_to_index(self, x: float, y: float) -> Optional[int]:
        """ World (x,y) → flat cell index in the OccupancyGrid. """
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

    def _map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        """ Grid cell (mx,my) → world coordinates (wx,wy). """
        cm = self._costmap
        origin_x = cm.info.origin.position.x
        origin_y = cm.info.origin.position.y
        res = cm.info.resolution

        wx = origin_x + (mx + 0.5) * res
        wy = origin_y + (my + 0.5) * res
        return wx, wy


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlanner()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

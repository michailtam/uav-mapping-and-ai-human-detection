#!/usr/bin/env python3
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import FollowPath
from nav_msgs.msg import Path, Odometry
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from px4_msgs.msg import TrajectorySetpoint, HomePosition, PositionSetpointTriplet
from geographiclib.geodesic import Geodesic

from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSDurabilityPolicy,
    QoSLivelinessPolicy,
    HistoryPolicy,
)


def yaw_to_quaternion(yaw: float) -> Quaternion:
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def latlon_to_local(home_lat, home_lon, home_alt, target_lat, target_lon, target_alt):
    g = Geodesic.WGS84.Inverse(home_lat, home_lon, target_lat, target_lon)
    s = g["s12"]
    az = math.radians(g["azi1"])
    east = s * math.sin(az)
    north = s * math.cos(az)
    up = (0.0 if math.isnan(target_alt) else target_alt) - (0.0 if math.isnan(home_alt) else home_alt)
    return east, north, up


def make_straight_path(now_msg_time, start: PoseStamped, goal: PoseStamped, step: float, frame_id: str, max_len: float) -> Path:
    sx, sy = start.pose.position.x, start.pose.position.y
    gx, gy = goal.pose.position.x, goal.pose.position.y
    dx, dy = gx - sx, gy - sy
    dist = math.hypot(dx, dy)

    # Clamp to local horizon
    if dist > max_len and dist > 1e-6:
        scale = max_len / dist
        gx = sx + dx * scale
        gy = sy + dy * scale
        dx, dy = gx - sx, gy - sy
        dist = max_len

    yaw = math.atan2(dy, dx) if dist > 1e-6 else 0.0
    q = yaw_to_quaternion(yaw)

    n = max(1, int(dist / step))
    poses = []

    # First pose: exactly the start
    ps0 = PoseStamped()
    ps0.header.frame_id = frame_id
    ps0.header.stamp = now_msg_time
    ps0.pose.position.x = sx
    ps0.pose.position.y = sy
    ps0.pose.position.z = 0.0
    ps0.pose.orientation = q
    poses.append(ps0)

    # Intermediate + end poses
    for i in range(1, n + 1):
        t = i / n
        px = sx + dx * t
        py = sy + dy * t
        psi = PoseStamped()
        psi.header.frame_id = frame_id
        psi.header.stamp = now_msg_time
        psi.pose.position.x = px
        psi.pose.position.y = py
        psi.pose.position.z = 0.0
        psi.pose.orientation = q
        poses.append(psi)

    path = Path()
    path.header.frame_id = frame_id
    path.header.stamp = now_msg_time
    path.poses = poses
    return path


class SimpleCommander(Node):
    def __init__(self):
        super().__init__("nav2_simple_commander")

        # Parameters aligned with your YAML
        self.global_frame = "x650_0/odom"
        self.controller_id = "FollowPath"  # matches controller_plugins ID
        self.goal_checker_id = ""  # leave empty unless defined in YAML
        self.path_step = 0.25
        self.local_horizon = 30.0

        self._cbg = ReentrantCallbackGroup()

        # Controller client
        self._controller_client = ActionClient(self, FollowPath, "/follow_path", callback_group=self._cbg)

        # Path publisher
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.path_pub = self.create_publisher(Path, "/computed_path", qos)

        # PX4 velocity bridge
        self._traj_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)

        # PX4 QoS
        px4_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
        )

        # Subscriptions
        self.create_subscription(HomePosition, "/fmu/out/home_position_v1", self.home_cb, px4_qos, callback_group=self._cbg)
        self.create_subscription(PositionSetpointTriplet, "/fmu/out/position_setpoint_triplet", self.goal_cb, px4_qos, callback_group=self._cbg)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_cb, 10, callback_group=self._cbg)
        self.create_subscription(Odometry, "/odom", self.odom_cb, 10, callback_group=self._cbg)

        # State
        self.home_lat: float | None = None
        self.home_lon: float | None = None
        self.home_alt: float | None = None
        self.current_pose_: PoseStamped | None = None
        self.goal_: PoseStamped | None = None

        # Concurrency
        self._plan_thread: threading.Thread | None = None
        self._busy: bool = False
        self._lock = threading.Lock()

        self.get_logger().info(f"Commander started. Using frame '{self.global_frame}'. Waiting for PX4 home and /odom...")

    def odom_cb(self, msg: Odometry):
        ps = PoseStamped()
        ps.header.frame_id = self.global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = msg.pose.pose
        self.current_pose_ = ps

    def home_cb(self, msg: HomePosition):
        # Cache home; ignore NaNs
        if msg.valid_hpos and not (math.isnan(msg.lat) or math.isnan(msg.lon)):
            self.home_lat = msg.lat
            self.home_lon = msg.lon
        if msg.valid_alt and not math.isnan(msg.alt):
            self.home_alt = msg.alt
        if self.home_lat is not None and self.home_lon is not None:
            self.get_logger().info(f"Home lat/lon cached: ({self.home_lat:.7f}, {self.home_lon:.7f})")

    def goal_cb(self, msg: PositionSetpointTriplet):
        sp = msg.current
        # Filter invalid PX4 setpoints
        if not sp.valid or math.isnan(sp.lat) or math.isnan(sp.lon):
            self.get_logger().warn("Ignoring invalid goal setpoint from PX4")
            return
        if self.home_lat is None or self.home_lon is None:
            self.get_logger().warn("Goal received but home lat/lon not yet available. Ignoring.")
            return
        if self.current_pose_ is None:
            self.get_logger().warn("Goal received but /odom not yet available. Ignoring.")
            return

        east, north, up = latlon_to_local(
            self.home_lat,
            self.home_lon,
            self.home_alt if self.home_alt is not None else 0.0,
            sp.lat,
            sp.lon,
            sp.alt if not math.isnan(sp.alt) else 0.0,
        )
        print(f"East:{east:.3f}, North:{north:.3f}, Up:{up:.3f}")

        yaw = sp.yaw if not math.isnan(sp.yaw) else 0.0

        ps = PoseStamped()
        ps.header.frame_id = self.global_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose.position.x = east
        ps.pose.position.y = north
        ps.pose.position.z = 0.0
        ps.pose.orientation = yaw_to_quaternion(yaw)

        with self._lock:
            # Debounce near-duplicate goals
            if (
                self.goal_ is not None
                and abs(self.goal_.pose.position.x - ps.pose.position.x) < 0.05
                and abs(self.goal_.pose.position.y - ps.pose.position.y) < 0.05
            ):
                self.get_logger().debug("Duplicate goal within tolerance; ignoring.")
                return

            self.goal_ = ps
            if not self._busy:
                self._busy = True
                self.get_logger().info(f"Goal pose {ps} received. Starting local path/controller thread...")
                self._plan_thread = threading.Thread(target=self.plan_and_follow, daemon=True)
                self._plan_thread.start()
            else:
                self.get_logger().info("Updated goal while controller is running; will plan next after current run.")

    def cmd_vel_cb(self, msg: Twist):
        # Bridge velocities to PX4 (ENU -> NED consideration handled by PX4 expectations if applicable)
        ts = TrajectorySetpoint()
        ts.timestamp = self.get_clock().now().nanoseconds // 1000
        ts.velocity = [msg.linear.x, msg.linear.y, -msg.linear.z]
        ts.yaw = 0.0
        self._traj_pub.publish(ts)

    def plan_and_follow(self):
        try:
            with self._lock:
                start = self.current_pose_
                goal = self.goal_

            if start is None or goal is None:
                if start is None:
                    self.get_logger().error("Start (/odom) missing!")
                else:
                    self.get_logger().error("Goal position missing!")
                return

            # Ensure frame consistency
            start.header.frame_id = self.global_frame
            goal.header.frame_id = self.global_frame

            now_msg_time = self.get_clock().now().to_msg()
            path = make_straight_path(
                now_msg_time,
                start,
                goal,
                step=self.path_step,
                frame_id=self.global_frame,
                max_len=self.local_horizon,
            )
            if not path.poses:
                self.get_logger().error("Local path generation returned empty path")
                return

            self.path_pub.publish(path)
            self.get_logger().info(f"Straight path ready with {len(path.poses)} poses in frame '{self.global_frame}'")

            # Wait for controller
            if not self._controller_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Controller not reachable")
                return

            ctrl_goal = FollowPath.Goal()
            ctrl_goal.path = path
            ctrl_goal.controller_id = self.controller_id  # must match YAML ID
            ctrl_goal.goal_checker_id = self.goal_checker_id

            future_ctrl = self._controller_client.send_goal_async(ctrl_goal)
            rclpy.spin_until_future_complete(self, future_ctrl)
            ctrl_handle = future_ctrl.result()
            if ctrl_handle is None or not ctrl_handle.accepted:
                self.get_logger().error("Controller goal not accepted")
                return

            result_future_ctrl = ctrl_handle.get_result_async()
            self.get_logger().info("Following straight path with local obstacle avoidance...")
            rclpy.spin_until_future_complete(self, result_future_ctrl)
            ctrl_result = result_future_ctrl.result()
            self.get_logger().info(
                f"Controller finished with result: {ctrl_result.result}, error_code={getattr(ctrl_result, 'error_code', 'n/a')}"
            )
        finally:
            with self._lock:
                self._busy = False
                self._plan_thread = None


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

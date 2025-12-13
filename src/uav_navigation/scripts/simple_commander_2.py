#!/usr/bin/env python3
import threading
import math

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from nav2_msgs.action import ComputePathToPose, FollowPath
from uav_navigation.srv import SetInitialPose, SetGoalPose
from nav_msgs.msg import Path
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from px4_msgs.msg import TrajectorySetpoint

from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped

from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy


def normalize_quaternion(q: Quaternion) -> Quaternion:
    n = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
    return Quaternion(x=0.0, y=0.0, z=0.0, w=1.0) if n == 0.0 else Quaternion(x=q.x/n, y=q.y/n, z=q.z/n, w=q.w/n)


def sanitize_pose(ps: PoseStamped, frame: str) -> PoseStamped:
    ps.header.frame_id = frame
    # Force 2D planning
    ps.pose.position.z = 0.0
    ps.pose.orientation = normalize_quaternion(ps.pose.orientation)
    return ps


def make_straight_path(start: PoseStamped, goal: PoseStamped, steps: int = 50) -> Path:
    path = Path()
    path.header.frame_id = start.header.frame_id
    path.header.stamp = start.header.stamp

    sx, sy, sz = start.pose.position.x, start.pose.position.y, 0.0
    gx, gy, gz = goal.pose.position.x, goal.pose.position.y, 0.0

    for i in range(steps + 1):
        t = i / float(steps)
        p = PoseStamped()
        p.header.frame_id = path.header.frame_id
        p.header.stamp = start.header.stamp
        p.pose.position.x = sx + t * (gx - sx)
        p.pose.position.y = sy + t * (gy - sy)
        p.pose.position.z = 0.0
        # Face along the path (optional simple yaw)
        yaw = math.atan2(gy - sy, gx - sx) if (gx != sx or gy != sy) else 0.0
        p.pose.orientation = Quaternion(
            x=0.0, y=0.0,
            z=math.sin(yaw / 2.0),
            w=math.cos(yaw / 2.0)
        )
        path.poses.append(p)
    return path


class Nav2SimpleCommander(Node):
    def __init__(self):
        super().__init__("direct_nav2_simple_commander")

        # Frames
        self.global_frame = "map"
        self.base_frame = "x650_0/base_footprint"  # matches your costmap params

        # Reentrant group allows concurrent callbacks (services + action futures)
        self._cbg = ReentrantCallbackGroup()

        # TF (to fallback to current robot pose if no home is set)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self, spin_thread=True)

        # Action clients (bind to actual endpoints exposed by servers)
        self._planner_client = ActionClient(
            self, ComputePathToPose, "/compute_path_to_pose", callback_group=self._cbg
        )
        self._controller_client = ActionClient(
            self, FollowPath, "/follow_path", callback_group=self._cbg
        )

        # Services to receive home & goal
        self.create_service(SetInitialPose, "set_home_pose", self.set_initial_pose_cbk, callback_group=self._cbg)
        self.create_service(SetGoalPose, "set_goal_pose", self.set_goal_pose_cbk, callback_group=self._cbg)

        # Publishers/subscribers
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.VOLATILE
        qos.reliability = QoSReliabilityPolicy.RELIABLE

        self.path_pub = self.create_publisher(Path, "/computed_path", qos)
        self._px4_traj_pub = self.create_publisher(TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10)
        self.create_subscription(Twist, "/cmd_vel", self.cmd_vel_to_px4, 10, callback_group=self._cbg)

        # State
        self.home_: PoseStamped | None = None
        self.goal_: PoseStamped | None = None

        # Explicit planner id (matches your YAML: GridBased with Navfn)
        self.planner_id = "GridBased"

        self.get_logger().warn("Nav2 simple commander server started and waiting for requests")

    def cmd_vel_to_px4(self, msg: Twist):
        ts = TrajectorySetpoint()
        ts.timestamp = self.get_clock().now().nanoseconds // 1000  # PX4 expects microseconds
        # PX4 uses NED: x forward, y right, z down
        ts.velocity = [msg.linear.x, msg.linear.y, -msg.linear.z]
        ts.yaw = 0.0
        self._px4_traj_pub.publish(ts)

    def set_initial_pose_cbk(self, request: SetInitialPose.Request, response: SetInitialPose.Response):
        # Trust PX4 stamp/frame but sanitize for 2D planning
        self.home_ = sanitize_pose(request.home, self.global_frame)
        self.get_logger().info("Received home pose")
        response.accepted = True
        return response

    def set_goal_pose_cbk(self, request: SetGoalPose.Request, response: SetGoalPose.Response):
        # Trust PX4 stamp/frame but sanitize for 2D planning
        self.goal_ = sanitize_pose(request.goal, self.global_frame)
        self.get_logger().info("Received goal pose")
        response.accepted = True

        if self.home_ is not None:
            threading.Thread(target=self.calculate_and_follow_path, daemon=True).start()
        else:
            self.get_logger().warn("Goal received but home not set yet; waiting for home pose.")
        return response

    def lookup_robot_pose(self, timeout_sec: float = 0.5) -> PoseStamped | None:
        try:
            tf: TransformStamped = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame, rclpy.time.Time(), timeout_sec=timeout_sec
            )
            ps = PoseStamped()
            ps.header.frame_id = self.global_frame
            ps.header.stamp = tf.header.stamp
            ps.pose.position.x = tf.transform.translation.x
            ps.pose.position.y = tf.transform.translation.y
            ps.pose.position.z = 0.0
            ps.pose.orientation = normalize_quaternion(tf.transform.rotation)
            return ps
        except Exception as e:
            self.get_logger().warn(f"TF lookup {self.global_frame}->{self.base_frame} failed: {e}")
            return None

    def calculate_and_follow_path(self):
        # Pre-flight validations
        if self.goal_ is None:
            self.get_logger().error("Cannot plan: goal is None")
            return

        start = self.home_ or self.lookup_robot_pose()
        if start is None:
            self.get_logger().error("Cannot plan: no start pose (home missing and TF unavailable)")
            return

        start = sanitize_pose(start, self.global_frame)
        goal = sanitize_pose(self.goal_, self.global_frame)

        # Log start/goal for traceability
        s, g = start.pose.position, goal.pose.position
        self.get_logger().info(
            f"Planning with planner_id='{self.planner_id}', base_frame='{self.base_frame}', "
            f"Start: ({s.x:.2f},{s.y:.2f},{s.z:.2f}) Goal: ({g.x:.2f},{g.y:.2f},{g.z:.2f})"
        )

        # Bounded server wait
        if not self._planner_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Planner server not reachable within 5s")
            # Fallback: synthesize a straight path so the controller can run
            path = make_straight_path(start, goal, steps=50)
            self._run_controller_with_path(path, use_fallback=True)
            return

        # Build planner goal
        goal_msg = ComputePathToPose.Goal()
        goal_msg.start = start
        goal_msg.goal = goal
        goal_msg.planner_id = self.planner_id

        # Send planner goal
        future_path = self._planner_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future_path)
        plan_handle = future_path.result()
        if plan_handle is None or not plan_handle.accepted:
            self.get_logger().error("Planner goal was not accepted.")
            # Fallback path
            path = make_straight_path(start, goal, steps=50)
            self._run_controller_with_path(path, use_fallback=True)
            return

        # Get planner result
        result_future = plan_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        result = result_future.result()
        if result is None or result.result is None:
            self.get_logger().error("Planner result is None")
            # Fallback path
            path = make_straight_path(start, goal, steps=50)
            self._run_controller_with_path(path, use_fallback=True)
            return

        path = result.result.path
        if not path.poses:
            self.get_logger().error("Planner returned empty path. Using synthetic straight path fallback.")
            path = make_straight_path(start, goal, steps=50)

        # Publish for RViz display and proceed to controller
        self.get_logger().info(f"Path ready with {len(path.poses)} poses")
        self.path_pub.publish(path)
        self._run_controller_with_path(path)

    def _run_controller_with_path(self, path: Path, use_fallback: bool = False):
        # Controller phase
        if not self._controller_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Controller server not reachable within 5s")
            return

        ctrl_goal = FollowPath.Goal()
        ctrl_goal.path = path
        ctrl_goal.controller_id = ''
        ctrl_goal.goal_checker_id = ''

        future_ctrl = self._controller_client.send_goal_async(ctrl_goal)
        rclpy.spin_until_future_complete(self, future_ctrl)
        ctrl_handle = future_ctrl.result()
        if ctrl_handle is None or not ctrl_handle.accepted:
            self.get_logger().error("Controller goal was not accepted.")
            return

        result_future_ctrl = ctrl_handle.get_result_async()
        self.get_logger().info("Following path..." + (" (synthetic fallback)" if use_fallback else ""))
        rclpy.spin_until_future_complete(self, result_future_ctrl)
        ctrl_result = result_future_ctrl.result()
        if ctrl_result is None or ctrl_result.result is None:
            self.get_logger().error("Controller result is None")
            return
        self.get_logger().info(f"Controller finished with result: {ctrl_result.result}")


def main(args=None):
    rclpy.init(args=args)
    node = Nav2SimpleCommander()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
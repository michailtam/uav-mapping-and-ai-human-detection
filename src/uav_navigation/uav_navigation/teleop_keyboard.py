#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

import sys
import termios
import tty
import select

# Throttle increments (like a real drone)
THROTTLE_INCREMENT = 0.05  # Small increments
MIN_THROTTLE = 0.0
MAX_THROTTLE = 1.0
HOVER_THROTTLE = 0.5  # Middle point for hovering

# Movement speeds
XY_VELOCITY = 0.8  # m/s for horizontal movement
YAW_RATE = 1.5     # rad/s for yaw rotation (~86 deg/s - very visible!)

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')

        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        self.pub = self.create_publisher(Twist, '/offboard_velocity_cmd', qos)
        self.arm_pub = self.create_publisher(Bool, '/arm_message', qos)

        self.throttle = 0.0  # Current throttle level (0.0 to 1.0)
        self.vx = 0.0  # Forward/backward
        self.vy = 0.0  # Left/right
        self.yaw_rate = 0.0  # Rotation

        self.armed = False
        
        # Track which movement keys are currently being "held"
        self.key_held_time = {}  # key -> last_time_seen
        self.key_timeout = 0.15  # If key not seen for 150ms, consider released

        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("Realistic Drone Keyboard Teleop Ready!")
        self.get_logger().info("="*50)
        self.get_logger().info("SPACE       = Arm / Disarm")
        self.get_logger().info("W           = Throttle UP (incremental)")
        self.get_logger().info("S           = Throttle DOWN (incremental)")
        self.get_logger().info("Arrow UP    = Move Forward (hold key)")
        self.get_logger().info("Arrow DOWN  = Move Backward (hold key)")
        self.get_logger().info("Arrow LEFT  = Move Left (hold key)")
        self.get_logger().info("Arrow RIGHT = Move Right (hold key)")
        self.get_logger().info("A           = Yaw Left (hold key)")
        self.get_logger().info("D           = Yaw Right (hold key)")
        self.get_logger().info("="*50 + "\n")

        self.timer = self.create_timer(0.05, self.loop)  # 20 Hz

    def get_key(self):
        dr, _, _ = select.select([sys.stdin], [], [], 0)
        if dr:
            return sys.stdin.read(1)
        return None

    def loop(self):
        key = self.get_key()
        current_time = time.time()
        
        # Reset velocities
        self.vx = 0.0
        self.vy = 0.0
        self.yaw_rate = 0.0

        # Clean up old held keys
        expired = [k for k, t in self.key_held_time.items() if current_time - t > self.key_timeout]
        for k in expired:
            del self.key_held_time[k]

        if key is not None:
            if key == '\x03':  # CTRL+C
                raise KeyboardInterrupt()

            # ------------------------
            # SPACE → Toggle Arm/Disarm
            # ------------------------
            elif key == " ":
                self.armed = not self.armed
                msg = Bool()
                msg.data = self.armed
                self.arm_pub.publish(msg)
                status = "ARMED" if self.armed else "DISARMED"
                self.get_logger().info(f"*** {status} ***")
                if not self.armed:
                    self.throttle = 0.0  # Reset throttle on disarm

            # ------------------------
            # W/S → Throttle Control (Incremental)
            # ------------------------
            elif key.lower() == 'w':
                self.throttle = max(MIN_THROTTLE, self.throttle - THROTTLE_INCREMENT)
                self.get_logger().info(f"Throttle UP: {self.throttle:.2f}")

            elif key.lower() == 's':
                self.throttle = min(MAX_THROTTLE, self.throttle + THROTTLE_INCREMENT)
                self.get_logger().info(f"Throttle DOWN: {self.throttle:.2f}")

            # ------------------------
            # A/D → Yaw Control (Mark as held for continuous rotation)
            # ------------------------
            elif key.lower() == 'a':
                self.key_held_time['a'] = current_time

            elif key.lower() == 'd':
                self.key_held_time['d'] = current_time

            # ------------------------
            # Arrow Keys → XY Movement (Mark as held)
            # ------------------------
            elif key == '\x1b':  # Arrow key prefix
                seq = sys.stdin.read(2)
                if seq == "[A":  # UP arrow
                    self.key_held_time['up'] = current_time
                elif seq == "[B":  # DOWN arrow
                    self.key_held_time['down'] = current_time
                elif seq == "[D":  # RIGHT arrow
                    self.key_held_time['right'] = current_time
                elif seq == "[C":  # LEFT arrow
                    self.key_held_time['left'] = current_time

        # Apply velocities for all currently held keys
        if 'a' in self.key_held_time:
            self.yaw_rate = -YAW_RATE
            self.get_logger().info(f"YAW LEFT active: {YAW_RATE:.2f} rad/s", throttle_duration_sec=0.3)
        if 'd' in self.key_held_time:
            self.yaw_rate = YAW_RATE
            self.get_logger().info(f"YAW RIGHT active: {-YAW_RATE:.2f} rad/s", throttle_duration_sec=0.3)
        if 'up' in self.key_held_time:
            self.vx = XY_VELOCITY
        if 'down' in self.key_held_time:
            self.vx = -XY_VELOCITY
        if 'left' in self.key_held_time:
            self.vy = XY_VELOCITY
        if 'right' in self.key_held_time:
            self.vy = -XY_VELOCITY

        # Convert throttle to vertical velocity
        # Throttle 0.0 = descend, 0.5 = hover, 1.0 = max ascent
        vz = (self.throttle - HOVER_THROTTLE) * 2.0  # Map to -1.0 to +1.0 m/s

        # Publish Twist message
        twist = Twist()
        twist.linear.x = self.vx
        twist.linear.y = self.vy
        twist.linear.z = vz
        twist.angular.z = self.yaw_rate
        self.pub.publish(twist)


def main(args=None):
    settings = termios.tcgetattr(sys.stdin)
    tty.setcbreak(sys.stdin.fileno())

    rclpy.init(args=args)
    node = KeyboardTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
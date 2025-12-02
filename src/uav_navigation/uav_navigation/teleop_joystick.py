#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

# Controller axis mappings (Based on your joy_teleop config)
AXIS_LEFT_STICK_LR = 0      # Left stick Left/Right → Yaw rotation (angular-z)
AXIS_LEFT_STICK_UD = 1      # Left stick Up/Down → Throttle/Altitude (linear-z)
AXIS_RIGHT_STICK_LR = 2     # Right stick Left/Right → Strafe (linear-y)
AXIS_RIGHT_STICK_UD = 3     # Right stick Up/Down → Forward/Back (linear-x)

# Button mappings (Xbox controller default)
BUTTON_A = 0                # A button → ARM
BUTTON_X = 3                # X button → Enable auto-hover (throttle = 0.5)
BUTTON_Y = 4                # Y button → Emergency stop (disarm + zero throttle)

# Movement parameters
XY_VELOCITY_MAX = 0.8       # m/s for horizontal movement
YAW_RATE_MAX = 1.5          # rad/s for yaw rotation
HOVER_THROTTLE = 0.5        # Neutral throttle position for hovering

# Deadzone for joystick axes
AXIS_DEADZONE = 0.1
THROTTLE_DEADZONE = 0.15    # Larger deadzone for throttle stick


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')

        # QoS for publishers (to match PX4)
        qos_pub = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )

        # QoS for joy subscriber (to match joy_node's VOLATILE durability)
        qos_joy = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.vel_pub = self.create_publisher(Twist, '/offboard_velocity_cmd', qos_pub)
        self.arm_pub = self.create_publisher(Bool, '/arm_message', qos_pub)

        # Subscriber
        self.create_subscription(Joy, '/joy', self.joy_callback, qos_joy)

        # State
        self.armed = False
        self.disarm_requested = False  # Flag to stop publishing commands
        self.last_button_state = []
        self.auto_hover = False  # When enabled, throttle locks to 0.5
        
        # Store current velocities
        self.current_vx = 0.0
        self.current_vy = 0.0
        self.current_vz = 0.0
        self.current_yaw_rate = 0.0
        self.current_throttle = 0.0

        # Controller configuration
        self.declare_parameter('controller_type', 'PS4')
        self.controller_type = self.get_parameter('controller_type').value

        # Create timer to publish at constant rate
        self.create_timer(0.05, self.publish_velocity)  # 20 Hz

        self.print_instructions()

    def print_instructions(self):
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("Joystick Teleop for PX4 Drone Ready!")
        self.get_logger().info("="*60)
        self.get_logger().info(f"Controller Type: {self.controller_type.upper()}")
        self.get_logger().info("-"*60)
        self.get_logger().info("LEFT STICK LR   = Yaw (Rotate)")
        self.get_logger().info("LEFT STICK UD   = Throttle (Direct control)")
        self.get_logger().info("  • Stick DOWN  = Descend (throttle 0.0)")
        self.get_logger().info("  • Stick CENTER= Hover (throttle 0.5)")
        self.get_logger().info("  • Stick UP    = Ascend (throttle 1.0)")
        self.get_logger().info("RIGHT STICK LR  = Strafe Left/Right")
        self.get_logger().info("RIGHT STICK UD  = Move Forward/Backward")
        self.get_logger().info("")
        self.get_logger().info("A Button (⬇)    = ARM drone")
        self.get_logger().info("Y Button (⬅)    = DISARM drone")
        self.get_logger().info("="*60 + "\n")

    def apply_deadzone(self, value, deadzone=AXIS_DEADZONE):
        """Apply deadzone to joystick axis."""
        if abs(value) < deadzone:
            return 0.0
        # Scale the value so that deadzone maps to 0 and 1.0 maps to 1.0
        sign = 1.0 if value > 0 else -1.0
        scaled = (abs(value) - deadzone) / (1.0 - deadzone)
        return sign * scaled

    def map_throttle_axis(self, axis_value):
        """
        Map joystick axis to throttle (0.0 to 1.0)
        Joystick typically returns -1.0 (up) to +1.0 (down)
        We want: -1.0 → 1.0 (max up), 0.0 → 0.5 (hover), +1.0 → 0.0 (max down)
        """
        # Apply deadzone first
        if abs(axis_value) < THROTTLE_DEADZONE:
            return HOVER_THROTTLE
        
        # Map -1.0 (stick up) to 1.0 (max throttle)
        # Map +1.0 (stick down) to 0.0 (min throttle)
        throttle = (1.0 - axis_value) / 2.0
        return max(0.0, min(1.0, throttle))

    def joy_callback(self, msg: Joy):
        """Process joystick input."""
        axes = msg.axes
        buttons = msg.buttons

        # Initialize button state tracking on first message
        if not self.last_button_state:
            self.last_button_state = [0] * len(buttons)
            self.get_logger().info(f"Controller connected! Axes: {len(axes)}, Buttons: {len(buttons)}")

        # =====================================================================
        # BUTTON HANDLING (Edge Detection)
        # =====================================================================
        
        # A Button → ARM
        if self.is_button_pressed(buttons, BUTTON_A):
            if not self.armed:
                self.armed = True
                self.disarm_requested = False  # Clear disarm flag
                arm_msg = Bool()
                arm_msg.data = True
                self.arm_pub.publish(arm_msg)
                self.get_logger().info("ARMED")

        # X Button → Toggle AUTO-HOVER
        if self.is_button_pressed(buttons, BUTTON_X):
            self.auto_hover = not self.auto_hover
            status = "ENABLED (throttle locked at 0.5)" if self.auto_hover else "DISABLED"
            self.get_logger().info(f"AUTO-HOVER {status}")

        # Y Button → DISARM/ EMERGENCY STOP
        if self.is_button_pressed(buttons, BUTTON_Y):
            self.armed = False
            self.disarm_requested = True
            self.auto_hover = False
            arm_msg = Bool()
            arm_msg.data = False
            self.arm_pub.publish(arm_msg)
            self.get_logger().warn("EMERGENCY STOP - DISARMED")

        # Update button state for next iteration
        self.last_button_state = list(buttons)

        # =====================================================================
        # AXIS HANDLING
        # =====================================================================

        # RIGHT stick → Forward/Back (vx) and Left/Right strafe (vy)
        vx_raw =  axes[AXIS_RIGHT_STICK_UD] if len(axes) > AXIS_RIGHT_STICK_UD else 0.0
        vy_raw = -axes[AXIS_RIGHT_STICK_LR] if len(axes) > AXIS_RIGHT_STICK_LR else 0.0

        vx = self.apply_deadzone(vx_raw) * XY_VELOCITY_MAX
        vy = self.apply_deadzone(vy_raw) * XY_VELOCITY_MAX

        # LEFT stick Left/Right → Yaw rotation
        yaw_raw = -axes[AXIS_LEFT_STICK_LR] if len(axes) > AXIS_LEFT_STICK_LR else 0.0
        yaw_rate = self.apply_deadzone(yaw_raw) * YAW_RATE_MAX

        # LEFT stick Up/Down → Throttle (DIRECT mapping)
        if self.auto_hover:
            throttle = HOVER_THROTTLE
        elif len(axes) > AXIS_LEFT_STICK_UD:
            # Use LEFT stick vertical for throttle
            throttle_raw = axes[AXIS_LEFT_STICK_UD]
            throttle = self.map_throttle_axis(throttle_raw)
        else:
            throttle = HOVER_THROTTLE

        # Convert throttle to vertical velocity
        # Throttle 0.0 = descend, 0.5 = hover, 1.0 = max ascent
        vz = (throttle - HOVER_THROTTLE) * 2.0  # Map to -1.0 to +1.0 m/s

        # Store velocities for continuous publishing
        self.current_vx = vx
        self.current_vy = vy
        self.current_vz = vz
        self.current_yaw_rate = yaw_rate
        self.current_throttle = throttle

    def publish_velocity(self):
        """Publish velocity at constant rate."""
        # If disarm requested, stop publishing (cuts offboard heartbeat)
        if self.disarm_requested:
            return
        
        twist = Twist()
        twist.linear.x = self.current_vx
        twist.linear.y = self.current_vy
        twist.linear.z = self.current_vz
        twist.angular.z = self.current_yaw_rate
        self.vel_pub.publish(twist)

        # Debug logging (throttled to avoid spam)
        if (abs(self.current_vx) > 0.01 or abs(self.current_vy) > 0.01 or 
            abs(self.current_yaw_rate) > 0.01 or abs(self.current_vz) > 0.01):
            hover_indicator = " [AUTO-HOVER]" if self.auto_hover else ""
            self.get_logger().info(
                f"vx={self.current_vx:.2f} vy={self.current_vy:.2f} vz={self.current_vz:.2f} "
                f"yaw={self.current_yaw_rate:.2f} | throttle={self.current_throttle:.2f}{hover_indicator}",
                throttle_duration_sec=0.5
            )

    def is_button_pressed(self, buttons, button_index):
        """Detect button press (edge detection: 0 → 1 transition)."""
        if button_index >= len(buttons) or button_index >= len(self.last_button_state):
            return False
        return buttons[button_index] == 1 and self.last_button_state[button_index] == 0


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Joystick Teleop shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
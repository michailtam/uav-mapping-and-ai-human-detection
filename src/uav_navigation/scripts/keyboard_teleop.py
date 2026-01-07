#!/usr/bin/env python3

"""
This file contains functions on how to teleop a drone. When a key gets  
released the value gets set to zero.

Example: 
    When the key for forward pitch was pressed the drone flies forward, 
    and when it get's released the drone stops.
"""

import sys
import geometry_msgs.msg
import std_msgs.msg
import rclpy
from pynput.keyboard import Key, Listener
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy


if sys.platform == 'win32':
    import msvcrt
else:
    import termios
    import tty

instruction_msg = """
This node reads keys pressed by the keyboard and publishes
special commands and Twist messages to the PX4 control node.

########################## MOVEMENT ##########################
#                                                            #
#       (w): Up                                              #
#       (s): Down                                            #
#       (a): Yaw Left                                        #
#       (d): Yaw Right                                       #
#       (Up Arrow): Pitch Forward                            #
#       (Down Arrow): Pitch Backward                         #
#       (Left Arrow): Roll Left                              #
#       (Right Arrow): Roll Right                            #
#                                                            #                             
#       Press (SPACE) to arm/disarm the drone                #
#                                                            #
#       Press (R) to return to base                          #
#       Press (ESC) to quit the application                  #
#                                                            #
##############################################################
"""


def set_echo(enable):
    # Get terminal attributes
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    new_settings = termios.tcgetattr(fd)
    
    if enable:
        # Turn echo ON
        new_settings[3] = new_settings[3] | termios.ECHO
    else:
        # Turn echo OFF
        new_settings[3] = new_settings[3] & ~termios.ECHO
        
    termios.tcsetattr(fd, termios.TCSADRAIN, new_settings)


def main():

    # ------------------------------ Embedded Functions ------------------------------
    def on_press(key):
        """
        Handles when a key has been pressed
        :param key: The key that was pressed
        """
        nonlocal x, y, z, th, status, x_val, y_val, z_val, yaw_val, arm_toggle, has_takeoff
        
        # Transform the key code to char
        key_ch = getattr(key, 'char', key)

        if has_takeoff:
            if key in moveBindings.keys() or getattr(key, 'char', key) in moveBindings.keys():
                key_as_char = getattr(key, 'char', key)
                x = moveBindings[key_as_char][0]
                y = moveBindings[key_as_char][1]
                z = moveBindings[key_as_char][2]
                th = moveBindings[key_as_char][3]

                x_val = (x * speed) + x_val
                y_val = (y * speed) + y_val
                z_val = (z * speed) + z_val
                
                # Create the Twist message to send to PX4
                twist = geometry_msgs.msg.Twist()
                twist.linear.x = float(x * speed)
                twist.linear.y = float(y * speed)
                twist.linear.z = float(z * speed)
                twist.angular.x = 0.0
                twist.angular.y = 0.0
                # 1*(-1) or 1*(1) -> velocity multiplication factor
                twist.angular.z = float(th * turn)
                pub.publish(twist)

        elif key_ch == Key.space and not has_takeoff:  # Arm
            arm_toggle = not arm_toggle  # Flip the value to arm or disarm
            arm_msg = std_msgs.msg.Int32()
            arm_msg.data = arm_toggle
            teleop_cmd_pub.publish(arm_msg) # 0 or 1
            print(f"Arm/Disarm toggle: {arm_msg.data}")
            has_takeoff = True

        # Check if landing mode is desired
        if isinstance(key_ch, str) and key_ch.lower() == 'l':  # Land
            arm_msg = std_msgs.msg.Int32()
            arm_msg.data = 2
            teleop_cmd_pub.publish(arm_msg)
            print(f"Landing engaged")
            has_takeoff = False

        # Check if RTL mode is desired
        if isinstance(key_ch, str) and key_ch.lower() == 'r':  # RTL
            arm_msg = std_msgs.msg.Int32()
            arm_msg.data = 3
            teleop_cmd_pub.publish(arm_msg)
            print(f"RTL engaged")
        
        # Check for ending the program
        elif key == Key.esc:  # End/Quit
            # Stop listener
            print("Teleop operation ended!")
            return False

    def on_release(key):
        """
        Handle when a key has been released
        :param key: The key that was released
        """
        nonlocal x, y, z, th, status, x_val, y_val, z_val, yaw_val, arm_toggle

        status = 0.0
        x_val = 0.0
        y_val = 0.0
        z_val = 0.0
        yaw_val = 0.0
        x = 0.0
        y = 0.0
        z = 0.0
        th = 0.0

        twist = geometry_msgs.msg.Twist()
        twist.linear.x = x
        twist.linear.y = y
        twist.linear.z = z
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0

        pub.publish(twist)


    # -------------------------------------------------------------------------------

    rclpy.init()

    node = rclpy.create_node('keyboard_teleop')

    # Display the movement instructions and the cursor in terminal
    print(instruction_msg)
    set_echo(False) 

    qos_profile = QoSProfile(
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        history=QoSHistoryPolicy.KEEP_LAST,
        depth=10
    )

    pub = node.create_publisher(geometry_msgs.msg.Twist, '/offboard_velocity_cmd', qos_profile)
    teleop_cmd_pub = node.create_publisher(std_msgs.msg.Int32, '/teleop_command', qos_profile)
    arm_toggle = False

    speed = 5.0     # Moves 5 meters per second
    turn = 1.0      # Rotates at 1 radians per second
    x = 0.0
    y = 0.0
    z = 0.0
    th = 0.0
    status = 0.0
    x_val = 0.0
    y_val = 0.0
    z_val = 0.0
    yaw_val = 0.0

    moveBindings = {
        'w': (0, 0, 1, 0),          # Z+
        's': (0, 0, -1, 0),         # Z-
        'a': (0, 0, 0, -1),         # Yaw+
        'd': (0, 0, 0, 1),          # Yaw-
        Key.up : (1, 0, 0, 0),      # Pitch forward -> Up Arrow
        Key.down : (-1, 0, 0, 0),   # Pitch backwards -> Down Arrow
        Key.right : (0, 1, 0, 0),   # Right Arrow
        Key.left : (0, -1, 0, 0),   # Left Arrow
    }

    has_takeoff = False

    try:
        # Collect all events until released
        with Listener(on_press = on_press, on_release=on_release) as listener:   
            listener.join()
            
    except Exception as e:
        print(e)

    finally:
        set_echo(True)

        twist = geometry_msgs.msg.Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = 0.0
        pub.publish(twist)

        node.get_logger().info("Keyboard teleop shutdown")
        rclpy.shutdown()


if __name__ == '__main__':
    main()
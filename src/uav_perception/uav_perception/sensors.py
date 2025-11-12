#!/usr/bin/env python3 
import rclpy
from rclpy.node import Node


class Sensors(Node):

    def __init__(self):
        super().__init__("sensors_node")
        self.get_logger().info("Senor node test")


def main(args=None):
    rclpy.init(args=args)
    node = Sensors()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()
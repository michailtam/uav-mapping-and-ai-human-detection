#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageTools(Node):
    def __init__(self):
        # Node name will be overridden from launch so it's safe to reuse
        super().__init__("image_tools_node")

        # Create the subscriber to read the raw image data from the camera and the publisher to send
        # the edited image for further processing.
        self.bridge = CvBridge()
        self.sub_ = self.create_subscription(Image, "image", self.image_callback, 10)
        self.pub_ = self.create_publisher(Image, "image_rotated", 10)
        
    def image_callback(self, msg: Image):
        try:
            # Convert image to opencv format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")
            return

        # Apply image processing: Rotate the image 180 degrees
        rotated = cv2.rotate(cv_image, cv2.ROTATE_180)
        out_msg = self.bridge.cv2_to_imgmsg(rotated, encoding="bgr8")
        out_msg.header = msg.header  # Keep the same timestamp / frame
        self.pub_.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ImageTools()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

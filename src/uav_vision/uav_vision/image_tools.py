#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class ImageTools(Node):

    def __init__(self):
        # Node name will be overridden from launch so it's safe to reuse
        super().__init__("image_tools_node")
        
        self.bridge = CvBridge()

        # Sensor QoS (matches camera / ros_gz_image publishers)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        # Subscribers
        # RGB image from Gazebo bridge
        self.rgb_sub_ = self.create_subscription(Image, "/camera/image", self.rgb_image_callback, sensor_qos)

        # Depth image from Gazebo bridge
        self.depth_sub_ = self.create_subscription(Image, "/camera/depth_image", self.depth_image_callback, sensor_qos)

        # Publishers
        self.rgb_pub_ = self.create_publisher(Image, "/camera/rgb/image", sensor_qos)
        self.depth_pub_ = self.create_publisher(Image, "/camera/depth/image", sensor_qos)
        
        self.get_logger().info("ImageTools node started (RGB + depth)")

    def rgb_image_callback(self, msg: Image):
        self.get_logger().debug(f"Incoming image encoding: {msg.encoding}, frame_id: {msg.header.frame_id}")

        try:
            # Assume RGB-like image
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            # Rotate / flip rgb
            rotated = cv2.rotate(cv_img, cv2.ROTATE_180)  # This fix solves the image orientation problem from the camera
            out_msg = self.bridge.cv2_to_imgmsg(rotated, encoding="bgr8")
            out_msg.header = msg.header
            self.rgb_pub_.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")
        
    def depth_image_callback(self, msg: Image):
        self.get_logger().debug(f"Incoming image encoding: {msg.encoding}, frame_id: {msg.header.frame_id}")

        try:
            # Depth: keep original encoding
            cv_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            # Rotate / flip depth
            rotated = cv2.rotate(cv_img, cv2.ROTATE_180)
            out_msg = self.bridge.cv2_to_imgmsg(rotated, encoding=msg.encoding)
            out_msg.header = msg.header
            self.depth_pub_.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageTools()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

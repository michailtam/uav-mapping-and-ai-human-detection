#!/usr/bin/env python3
"""
Camera Frame Republisher Node

This node subscribes to camera topics published by Gazebo and republishes them
with the correct optical frame_id for proper visualization in RViz.

Subscribes to:
  - /camera/image (sensor_msgs/Image)
  - /camera/depth_image (sensor_msgs/Image)
  - /camera/depth/camera_info (sensor_msgs/CameraInfo)

Publishes to:
  - /camera/image_optical (sensor_msgs/Image)
  - /camera/depth_image_optical (sensor_msgs/Image)
  - /camera/depth/camera_info_optical (sensor_msgs/CameraInfo)

All republished messages have frame_id changed to: x650_0/base_link/rgbd_cam_optical
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class CameraFrameRepublisher(Node):
    def __init__(self):
        super().__init__('camera_frame_republisher')
        
        # Optical frame name
        self.optical_frame = 'x650_0/base_link/rgbd_cam_optical'
        
        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.rgb_sub = self.create_subscription(
            Image, '/camera/image', self.rgb_callback, sensor_qos)
        self.depth_sub = self.create_subscription(
            Image, '/camera/depth_image', self.depth_callback, sensor_qos)
        self.info_sub = self.create_subscription(
            CameraInfo, '/camera/depth/camera_info', self.info_callback, sensor_qos)
        
        # Publishers
        self.rgb_pub = self.create_publisher(Image, '/camera/image_optical', sensor_qos)
        self.depth_pub = self.create_publisher(Image, '/camera/depth_image_optical', sensor_qos)
        self.info_pub = self.create_publisher(CameraInfo, '/camera/depth/camera_info_optical', sensor_qos)
        
        self.get_logger().info(f'Camera frame republisher started. Optical frame: {self.optical_frame}')
    
    def rgb_callback(self, msg):
        """Republish RGB image with optical frame_id"""
        msg.header.frame_id = self.optical_frame
        self.rgb_pub.publish(msg)
    
    def depth_callback(self, msg):
        """Republish depth image with optical frame_id"""
        msg.header.frame_id = self.optical_frame
        self.depth_pub.publish(msg)
    
    def info_callback(self, msg):
        """Republish camera info with optical frame_id"""
        msg.header.frame_id = self.optical_frame
        self.info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraFrameRepublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

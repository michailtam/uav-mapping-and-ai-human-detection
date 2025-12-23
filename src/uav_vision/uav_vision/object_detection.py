#!/usr/bin/env python3
import os
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from ultralytics import YOLO # YOLO library


class ObjectDetection(Node):

    def __init__(self):
        # Node name will be overridden from launch so it's safe to reuse
        super().__init__("object_detection")
        
        package_share_directory = get_package_share_directory('uav_vision')
        model_path = os.path.join(package_share_directory, 'models', 'yolov8n.pt')
        self._model = YOLO(model_path)

        self.bridge = CvBridge()

        # Sensor QoS (matches camera / ros_gz_image publishers)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        # Subscriber
        # RGB image from Gazebo bridge
        self.rgb_sub_ = self.create_subscription(Image, "/camera/image", self.camera_listener_clbk, sensor_qos)

        # Publisher
        self.rgb_pub_ = self.create_publisher(Image, "/detection", sensor_qos)
        
    def camera_listener_clbk(self, msg: Image):
        
        try:
            # Assume RGB-like image
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            
            # Object Detection
            results = self._model.predict(image, classes=[0, 2])
            img_res = results[0].plot()
            out_msg = self.bridge.cv2_to_imgmsg(img_res, encoding="bgr8")
            out_msg.header = msg.header
            self.rgb_pub_.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

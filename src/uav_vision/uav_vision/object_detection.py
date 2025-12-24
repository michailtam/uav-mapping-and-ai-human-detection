#!/usr/bin/env python3
import os
import rclpy
import cv2
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
        self.rgb_pub_ = self.create_publisher(Image, "/ai/detection", sensor_qos)

        cv2.namedWindow("Suspect Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Suspect Detection", 800, 600)
        
    def camera_listener_clbk(self, data: Image):
        
        try:
            current_frame = self.bridge.imgmsg_to_cv2(data, desired_encoding="bgr8") # Convert ROS Image to OpenCV
            image = current_frame
            image = cv2.resize(image, (800, 600), interpolation=cv2.INTER_AREA)
            results = self._model.predict(image, classes=[0, 2], verbose=False) # Object Detection
            img = results[0].plot() # Plot bounding box on the image
            
            cv2.imshow("Suspect Detection", img) # Display in a persistent window
            cv2.waitKey(1)

            # Publish the annotated image back to ROS
            out_data = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            out_data.header = data.header
            self.rgb_pub_.publish(out_data)
            pass

        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    try:
        rclpy.init(args=args)
        node = ObjectDetection()
        rclpy.spin(node)
    finally:
        cv2.destroyAllWindows()
        node.get_logger().info("All windows destroyed") # Very important to destroy all dangling windows
        rclpy.shutdown()


if __name__ == "__main__":
    main()

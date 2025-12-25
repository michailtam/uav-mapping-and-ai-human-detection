#!/usr/bin/env python3
import os
import rclpy
import cv2
import message_filters
import numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory

from ultralytics import YOLO 


class ObjectDetection(Node):

    def __init__(self):
        super().__init__("object_detection")
        
        # Model Setup
        package_share_directory = get_package_share_directory('uav_vision')
        model_path = os.path.join(package_share_directory, 'models', 'yolov8n.pt')
        self._model = YOLO(model_path)
        self.bridge = CvBridge()

        # QoS Profile (this has to match Gazebo/PX4 standard)
        sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10)

        # Use message_filters.Subscriber to fuse both subscribers (required for the distance calculation)
        self.rgb_sub = message_filters.Subscriber(self, Image, "/camera/image", qos_profile=sensor_qos)
        self.depth_sub = message_filters.Subscriber(self, Image, "/camera/depth/image", qos_profile=sensor_qos)

        # Variables in the list must match the names defined above (self.rgb_sub, self.depth_sub)
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.rgb_sub, self.depth_sub], 
            queue_size=10, 
            slop=0.1 
        )
        self.ts.registerCallback(self.synchronized_clbk)

        # Publisher
        self.rgb_pub_ = self.create_publisher(Image, "/image/detection", sensor_qos)

        # Persistent Window setup
        cv2.namedWindow("Suspect Detection", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Suspect Detection", 800, 600)
        self.get_logger().info("Node started. Waiting for synchronized RGB and Depth frames...")
    
    def synchronized_clbk(self, rgb_msg, depth_msg):
        try:
            # Convert images
            color_image = self.bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, "32FC1") # Float32 meters

            # AI Detection (NMS applied via max_det=1)
            results = self._model.predict(color_image, classes=[0], conf=0.6, max_det=1, verbose=False)

            # Define default annotated frame (incase no detection)
            annotated_frame = color_image.copy()

            for r in results:
                # Plot YOLO boxes first
                annotated_frame = r.plot() 
                
                for box in r.boxes:
                    # Calculate center coordinates
                    x1, y1, x2, y2 = box.xyxy[0]
                    cx, cy = int((x1 + x2) / 2), int((y1 + y2) / 2)

                    # Safeguard against out-of-bounds
                    cy = min(max(cy, 2), depth_image.shape[0] - 3)
                    cx = min(max(cx, 2), depth_image.shape[1] - 3)

                    # Extract average depth from 5x5 ROI
                    depth_roi = depth_image[cy-2:cy+3, cx-2:cx+3]
                    dist = np.nanmean(depth_roi)

                    if not np.isnan(dist) and not np.isinf(dist):
                        dist_label = f"distance: {dist:.2f}m"   # Format label to look like the YOLO label
                        
                        # Use the same font settings as YOLO for matching dimensions
                        font = cv2.FONT_HERSHEY_SIMPLEX
                        font_scale = 0.6
                        thickness = 2
                        
                        # Calculate size of the distance label
                        (w, h), _ = cv2.getTextSize(dist_label, font, font_scale, thickness)
                        
                        # Position it directly above the person label
                        label_x = int(x1)
                        label_y = int(y1) - 22 # Offset to sit above the blue person label

                        # Draw background rectangle (matching the person label style)
                        cv2.rectangle(annotated_frame, (label_x, label_y - h - 5), (label_x + w, label_y + 5), 
                                      (75,75,75), -1)
                        
                        # Draw the text
                        cv2.putText(annotated_frame, dist_label, (label_x, label_y), 
                                    font, font_scale, (0, 255, 0), thickness)

            # Display and Publish the annotated image
            cv2.imshow("Suspect Detection", annotated_frame)
            cv2.waitKey(1)
            
            out_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
            out_msg.header = rgb_msg.header
            self.rgb_pub_.publish(out_msg)

        except Exception as e:
            self.get_logger().error(f"Sync Callback failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetection()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        # Final cleanup
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
#!/usr/bin/env python3

from ultralytics import YOLO
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import numpy as np

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

bridge = CvBridge()

class CameraSubscriber(Node):
    def __init__(self):
        super().__init__('camera_subscriber')

        # Load YOLO model
        self.model = YOLO('/home/mta/nav_ws/src/helios_recognition/yolov8n.pt')

        # Create message and publishers
        self.yolov8_inference = Yolov8Inference()
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)
        
        # Subscribe to image and LiDAR topics
        self.subscription = self.create_subscription(Image, 'camera/image_raw', self.camera_callback, 10)
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)

        # Store latest LiDAR data
        self.lidar_data = None
        self.camera_fov_half = 1.089 / 2  # Half of the camera's FOV in radians

    def lidar_callback(self, data):
        # Store latest LiDAR data for processing with detections
        self.lidar_data = data

    def camera_callback(self, data):
        # Convert ROS Image message to OpenCV image
        img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(img)  # Perform YOLO inference

        # Prepare inference message with header
        self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = self.get_clock().now().to_msg()

        if self.lidar_data is None:
            self.get_logger().warning("LiDAR data not yet available")
            return  # Wait for LiDAR data before processing

        # Process each detected object
        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                
                # Extract bounding box and class
                b = box.xyxy[0].to('cpu').detach().numpy().copy()
                self.inference_result.top = int(b[1])
                self.inference_result.left = int(b[0])
                self.inference_result.bottom = int(b[3])
                self.inference_result.right = int(b[2])
                c = box.cls
                self.inference_result.class_name = self.model.names[int(c)]

                # Calculate center coordinates
                center_x = int((self.inference_result.left + self.inference_result.right) / 2)
                center_y = int((self.inference_result.top + self.inference_result.bottom) / 2)
                self.inference_result.center_x = center_x
                self.inference_result.center_y = center_y

                # Calculate angle for the object's center relative to the LiDAR
                angle_to_object = np.arctan2(center_x - (img.shape[1] / 2), img.shape[1])  # Adjust this if x-axis differs

                # Check if object center is within the camera's FOV
                if -self.camera_fov_half <= angle_to_object <= self.camera_fov_half:
                    # Find the corresponding LiDAR index
                    lidar_index = int((angle_to_object - self.lidar_data.angle_min) / self.lidar_data.angle_increment)

                    # Ensure index is within range of LiDAR data
                    if 0 <= lidar_index < len(self.lidar_data.ranges):
                        distance = self.lidar_data.ranges[lidar_index]
                        # Assign LiDAR distance to detected object
                        self.inference_result.distance = distance
                        self.get_logger().info(f"Object '{self.inference_result.class_name}' at distance: {distance:.2f}m")

                # Append result to Yolov8Inference message
                self.yolov8_inference.yolov8_inference.append(self.inference_result)

        # Annotate and publish the image
        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame, encoding="bgr8")
        self.img_pub.publish(img_msg)

        # Publish inference results
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = CameraSubscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()

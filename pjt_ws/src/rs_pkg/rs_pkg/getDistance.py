import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

import cv2
import numpy as np

import pyrealsense2 as rs

from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from dobot_msgs.action import PointToPoint

import torch

class RealSenseYoloNode(Node):
    def __init__(self):
        super().__init__('get_dist_node')

        # YOLOv5 model_load
        self.yolo_model = torch.hub.load('ultralytics/yolov5', 'custom', path='/home/edurobot/ssafy_ws/src/ssafy_pjt/yolov5_model/best.pt')

        # RealSense camera setup
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
        config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        self.pipeline.start(config)

        self.detection_publisher = self.create_publisher(String, 'detection_results', 10)
        self.image_publisher = self.create_publisher(Image, 'detection_image', 10)

        self.bridge = CvBridge()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        # Get RealSense frames
        frames = self.pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if not color_frame or not depth_frame:
            return

        # Convert frames to numpy arrays
        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())

        # YOLOv5 inference
        results = self.yolo_model(color_image)

        self.detection_result = String()

        # Draw bounding boxes and calculate distance
        for result in results.xyxy[0]:
            x1, y1, x2, y2, confidence, class_id = map(int, result[:6])

            # Calculate the object's center point
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            # Get the depth at the center point
            depth_value = depth_image[center_y, center_x] * 0.001  # Convert mm to meters

            # Get object label
            label = self.yolo_model.names[class_id]
            self.detection_result.data = f'{label}, distance: {depth_value:.2f}m'

            # Draw bounding box
            cv2.rectangle(color_image, (x1, y1), (x2, y2), (0, 255, 0), 2)

            # Draw center point
            cv2.circle(color_image, (center_x, center_y), 5, (0, 0, 255), -1)  # Red dot

            # Display label and distance
            cv2.putText(color_image, f'{label} {depth_value:.2f}m', (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        self.detection_publisher.publish(self.detection_result)
        ros_image_message = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        self.image_publisher.publish(ros_image_message)

    def destroy_node(self):
        self.pipeline.stop()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = RealSenseYoloNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()

if __name__ == '__main__':
    main()

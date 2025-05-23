import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import torch

class ConveyorYoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # YOLOv5 모델 로드 (경로는 상황에 맞게 수정)
        self.yolo_model = torch.hub.load('ultralytics/yolov5', 'custom', path='pjt_pkg/src/yolo_pkg/model/best.pt')

        # 이미지 구독
        self.image_sub = self.create_subscription(
            Image,
            '/conv_space_image',
            self.image_callback,
            10
        )

        self.detection_publisher = self.create_publisher(String, 'detection_results', 10)
        self.image_publisher = self.create_publisher(Image, 'detection_image', 10)

        self.bridge = CvBridge()

    def get_color_name(self, hsv_color):
        h, s, v = hsv_color
        if 20 < h < 60 and s < 80 and 120 < v < 255:
            return 'white'
        elif ((h < 10 or h > 160) and s > 100 and v > 60):
            return 'red'
        elif 80 < h < 150 and s > 80 and v > 40:
            return 'blue'
        return 'unknown'

    def get_color_bgr(self, color_name):
        if color_name == 'white':
            return (255, 255, 255)
        elif color_name == 'red':
            return (0, 0, 255)
        elif color_name == 'blue':
            return (255, 0, 0)
        return (0, 255, 0)  # Default to green for unknown colors

    def get_center_color(self, image):
        height, width = image.shape[:2]
        center_y, center_x = height // 2, width // 2
        sample_size = min(width, height) // 4
        start_x = max(0, center_x - sample_size // 2)
        end_x = min(width, center_x + sample_size // 2)
        start_y = max(0, center_y - sample_size // 2)
        end_y = min(height, center_y + sample_size // 2)
        center_region = image[start_y:end_y, start_x:end_x]
        hsv_region = cv2.cvtColor(center_region, cv2.COLOR_BGR2HSV)
        average_color = np.mean(hsv_region, axis=(0, 1))
        return average_color

    def image_callback(self, msg):
        # ROS Image -> OpenCV Image
        color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # YOLOv5 inference
        results = self.yolo_model(color_image)

        detection_result = String()

        # Draw bounding boxes
        for result in results.xyxy[0]:
            x1, y1, x2, y2, confidence, class_id = map(int, result[:6])
            object_roi = color_image[y1:y2, x1:x2]
            if object_roi.size == 0:
                continue  # Skip invalid ROI
            center_color = self.get_center_color(object_roi)
            color_name = self.get_color_name(center_color)
            color_bgr = self.get_color_bgr(color_name)
            label = self.yolo_model.names[class_id]
            detection_result.data = label + ' ' + color_name

            cv2.rectangle(color_image, (x1, y1), (x2, y2), color_bgr, 2)
            cv2.putText(
                color_image, f'{label}-{color_name}', (x1, y1-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2
            )

        self.detection_publisher.publish(detection_result)
        ros_image_message = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
        self.image_publisher.publish(ros_image_message)

def main(args=None):
    rclpy.init(args=args)
    node = ConveyorYoloNode()
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

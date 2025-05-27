import rclpy
from rclpy.node import Node

import cv2
import numpy as np

from std_msgs.msg import String
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import torch

import time

H = 0
S = 0
V = 0
count = 0

package_name = 'yolo_pkg'



class ConveyorYoloNode(Node):
    def __init__(self):
        super().__init__('yolo_node')

        # YOLOv5 모델 로드 (경로는 상황에 맞게 수정)
        model_path = '/home/ssafy/SSAFY-Smart_Factory/ros2_ws/src/yolo_pkg/model/best.pt'
        self.yolo_model = torch.hub.load('ultralytics/yolov5', 'custom', path=model_path)

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
        print(f"[HSV 평균값] H: {h:.2f}, S: {s:.2f}, V: {v:.2f}", end=" - ")

        # BLUE: H값이 100~120, S가 높고 V는 중간 이상
        if 100 < h < 120 and s > 230 and v > 50:
            print("blue")
            return 'blue'
        # WHITE: S가 매우 낮고 V가 높음
        elif s < 30 and v > 100:
            print("white")
            return 'white'
        print("red")
        return 'red'



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
            detection_result.data = color_name

            cv2.rectangle(color_image, (x1, y1), (x2, y2), color_bgr, 2)
            cv2.putText(
                color_image, f'{label}-{color_name}', (x1, y1-10),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, color_bgr, 2
            )
            break

        cv2.imshow('object detection', color_image)
        cv2.waitKey(1)
        if detection_result.data != '':
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
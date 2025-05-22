import rclpy
from rclpy.node import Node
import cv2
from sensor_msgs.msg import Image
from std_msgs.msg import String
import torch # type: ignore
from cv_bridge import CvBridge


class RealSenseYoloNode(Node):
    def __init__(self):
        super().__init__('rs_yolo_node')

        # publisher
        self.result_publisher = self.create_publisher(String, 'detection_results', 10) # yolov5_results라는 토픽으로 날리기
        self.image_publisher = self.create_publisher(Image, 'yolov5_image_with_boxes',10) 
        
        # subscriber
        self.image_subscriber = self.create_subscription(Image, '/camera/camera/color/image_raw',
                                                          self.image_callback, 10)
        # YOLO
        self.model = torch.hub.load('ultralytics/yolov5','yolov5s',pretrained=True)

        # OpenCV
        self.bridge = CvBridge()
        
    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # cv 계열 이미지로 파싱
        except Exception as e:
            self.get_logger().error(f"Failed to convert: {e}")
            return
        
        results = self.model(cv_image)
        detections = results.pandas().xyxy[0]

        detection_data = []

        for _, row in detections.iterrows():
            label = row['name']
            confidence = row['confidence']
            xmin, ymin, xmax, ymax = int(row['xmin']), int(row['ymin']), int(row['xmax']), int(row['ymax'])

            detection_info = f"Label: {label}, Confidence: {confidence:.2f}, BBox: ({xmin},{ymin},{xmax},{ymax})"
            detection_data.append(detection_info)

            cv2.rectangle(cv_image, (xmin, ymin), (xmax, ymax), (0, 255, 0), 2)
            cv2.putText(cv_image, f"{label} {confidence:.2f}", (xmin, ymin-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        result_message = String()
        result_message.data = '/n'.join(detection_data)
        self.result_publisher.publish(result_message)
        self.get_logger().info("Published YOLOv5 detection results")

        try:
            boxed_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
            self.image_publisher.publish(boxed_image_msg)
            self.get_logger().info("Publisherd image with bounding boxes.")

        except:
            self.get_logger().error(f"Failed to publish image: {e}")



def main(args=None):
    rclpy.init(args=None)
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
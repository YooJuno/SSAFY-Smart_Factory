import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import cv2
import threading

from flask import Flask, Response

app = Flask(__name__)
viewer_node = None  # RealSenseViewer 인스턴스 저장용


class RealSenseViewer(Node):

    def __init__(self):
        super().__init__('realsense_viewer')

        self.bridge = CvBridge()
        self.latest_frame = None
        self.lock = threading.Lock()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',  # ← 여기 수정됨!
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            cv_image = cv2.resize(cv_image, (cv_image.shape[1]//2, cv_image.shape[0]//2))
            with self.lock:
                self.latest_frame = cv_image.copy()
            cv2.imshow("RealSense Color Stream", cv_image)
            cv2.waitKey(1)
        except Exception as e:
            self.get_logger().error(f"CV conversion failed: {e}")

    def get_jpeg_image(self):
        with self.lock:
            if self.latest_frame is None:
                return None
            success, jpeg = cv2.imencode('.jpg', self.latest_frame)
            if success:
                return jpeg.tobytes()
            return None


@app.route('/camera/image')
def get_image():
    global viewer_node
    if viewer_node is None:
        return Response("ROS2 node not initialized", status=500)

    image = viewer_node.get_jpeg_image()
    if image:
        return Response(response=image, content_type='image/jpeg')
    else:
        return Response("No image available", status=404)


def ros2_thread_main():
    global viewer_node
    rclpy.init()
    viewer_node = RealSenseViewer()
    rclpy.spin(viewer_node)
    viewer_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    ros_thread = threading.Thread(target=ros2_thread_main, daemon=True)
    ros_thread.start()

    app.run(host='0.0.0.0', port=8000)

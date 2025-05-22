import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np


class ImageTransformer(Node):
    def __init__(self):
        super().__init__('image_transformer')
        self.bridge = CvBridge()

        # 이미지 구독
        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10
        )

        # 좌표 퍼블리셔
        self.publisher = self.create_publisher(Point, '/target_pos', 10)

        self.points = []
        self.original_image = None
        self.latest_frame = None
        self.transform_matrix = None
        self.warped_size = (290 * 4, 210 * 4)  # (width, height)
        self.padding = 100  # 검은 여백 (pixels)

        cv2.namedWindow('RealSense Image')
        cv2.setMouseCallback('RealSense Image', self.mouse_callback_original)
        cv2.namedWindow('Transformed')
        cv2.setMouseCallback('Transformed', self.mouse_callback_transformed)

    def mouse_callback_original(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(self.points) < 4:
            self.points.append([x, y])
            self.get_logger().info(f'Point {len(self.points)}: ({x}, {y})')

            if len(self.points) == 4:
                pts_src = np.array(self.points, dtype='float32')
                width, height = self.warped_size
                pts_dst = np.array([
                    [0, 0],
                    [width - 1, 0],
                    [width - 1, height - 1],
                    [0, height - 1]
                ], dtype='float32')

                self.transform_matrix = cv2.getPerspectiveTransform(pts_src, pts_dst)
                self.points = []

    def mouse_callback_transformed(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            x_local = x - self.padding
            y_local = y - self.padding

            real_x, real_y = self.pixel_to_real(x_local, y_local)
            self.get_logger().info(
                f'[Transformed] Pos : (y : {real_x:.2f}, x : {real_y:.2f})'
            )

            # 퍼블리시
            point = Point()
            point.x = float(real_x)
            point.y = float(real_y)
            point.z = 0.0
            self.publisher.publish(point)

    def pixel_to_real(self, x, y):
        width, height = self.warped_size
        x_real_min, x_real_max = -280, 300
        y_real_min, y_real_max = -100, 320

        real_x = x_real_max - (x / (width - 1)) * (x_real_max - x_real_min)
        real_y = y_real_max - (y / (height - 1)) * (y_real_max - y_real_min)

        return real_x, real_y

    def listener_callback(self, msg):
        try:
            image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.original_image = image.copy()
            display_image = image.copy()

            for pt in self.points:
                cv2.circle(display_image, tuple(pt), 5, (0, 255, 0), -1)

            self.latest_frame = display_image

            if self.transform_matrix is not None:
                width, height = self.warped_size
                pad = self.padding

                warped = cv2.warpPerspective(image, self.transform_matrix, (width, height))

                warped = cv2.copyMakeBorder(
                    warped, pad, pad, pad, pad,
                    borderType=cv2.BORDER_CONSTANT, value=(0, 0, 0)
                )

                # 빨간 테두리
                cv2.rectangle(warped, (pad, pad), (pad + width - 1, pad + height - 1), (0, 0, 255), 2)

                # 파란색 그리드 (가로 29칸, 세로 21칸)
                for i in range(1, 29):
                    x = pad + int(i * width / 29)
                    cv2.line(warped, (x, pad), (x, pad + height), (255, 0, 0), 1)

                for j in range(1, 21):
                    y = pad + int(j * height / 21)
                    cv2.line(warped, (pad, y), (pad + width, y), (255, 0, 0), 1)

                cv2.imshow('Transformed', warped)

        except Exception as e:
            self.get_logger().error(f"CV Bridge error: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageTransformer()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.01)

            if node.latest_frame is not None:
                cv2.imshow('RealSense Image', node.latest_frame)

            if cv2.waitKey(1) == 27:  # ESC key
                break

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

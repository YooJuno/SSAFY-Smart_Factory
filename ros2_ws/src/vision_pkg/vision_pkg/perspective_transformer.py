import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import numpy as np
import time

class ImageTransformer(Node):
    def __init__(self):
        super().__init__('perspective_transformer_node')
        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/camera/color/image_raw',
            self.listener_callback,
            10
        )

        self.prev_time = 0

        # 좌표 퍼블리셔
        self.publisher_point = self.create_publisher(Point, '/target_pos', 10)
        self.publisher_conv_image = self.create_publisher(Image, '/conv_space_image', 10)
        self.publisher_coord_image = self.create_publisher(Image, '/coord_space_image', 10)

        self.points_coordinate = []

        self.original_image = None
        self.latest_frame = None

        self.transform_matrix_coordinate = None

        self.warped_size_coordinate = (290 * 4, 210 * 4)  # (width, height)

        self.pix_x, self.pix_y = self.warped_size_coordinate[0]/2, self.warped_size_coordinate[1]/2

        cv2.namedWindow('RealSense Image')
        cv2.setMouseCallback('RealSense Image', self.mouse_callback_original)
        
        cv2.namedWindow('Transformed_Coordinate')
        cv2.setMouseCallback('Transformed_Coordinate', self.mouse_callback_transformed_coordinate)

        cv2.namedWindow('Transformed_Conveyor')

    def mouse_callback_original(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN and len(self.points_coordinate) < 4:
            self.points_coordinate.append([x, y])
            self.get_logger().info(f'Point {len(self.points_coordinate)}: ({x}, {y})')

            if len(self.points_coordinate) == 4:
                width, height = self.warped_size_coordinate
                pts_src = np.array(self.points_coordinate, dtype='float32')
                pts_dst = np.array([
                    [0, 0],
                    [width - 1, 0],
                    [width - 1, height - 1],
                    [0, height - 1]
                ], dtype='float32')

                self.transform_matrix_coordinate = cv2.getPerspectiveTransform(pts_src, pts_dst)
                self.points_coordinate = []

    def mouse_callback_transformed_coordinate(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pix_x, self.pix_y = x, y
            self.x, self.y = self.pixel_to_real(x, y)
            self.get_logger().info(
                f'[Transformed] Pos : (y : {self.x:.2f}, x : {self.y:.2f})'
            )

            point = Point()
            point.x = float(self.x)
            point.y = float(self.y)
            point.z = 0.0
            self.publisher_point.publish(point)

    def pixel_to_real(self, x, y):
        width, height = self.warped_size_coordinate
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

            for pt in self.points_coordinate:
                cv2.circle(display_image, tuple(pt), 5, (0, 255, 0), -1)

            self.latest_frame = display_image

            if self.transform_matrix_coordinate is not None:
                width, height = self.warped_size_coordinate

                warped_coordinate = cv2.warpPerspective(image, self.transform_matrix_coordinate, (width, height))

                warped_conveyor = warped_coordinate.copy()[55:340, 40:700]

                # 파란색 그리드 (가로 29칸, 세로 21칸)
                for i in range(1, 29):
                    x = int(i * width / 29)
                    cv2.line(warped_coordinate, (x, 0), (x, height), (200, 50, 0), 1)

                for j in range(1, 21):
                    y = int(j * height / 21)
                    cv2.line(warped_coordinate, (0, y), (width, y), (200, 50, 0), 1)

                cv2.circle(warped_coordinate, (int(self.pix_x), int(self.pix_y)), 3, (0, 0, 255), 3)

                cv2.imshow('Transformed_Coordinate', warped_coordinate)
                cv2.imshow('Transformed_Conveyor', warped_conveyor)
                
                # 1초에 한 장 씩 pub
                cur_time = time.time()
                if cur_time - self.prev_time >= 1.0:
                    conv_image = self.bridge.cv2_to_imgmsg(warped_conveyor, encoding='bgr8')
                    self.publisher_conv_image.publish(conv_image)

                    self.prev_time = cur_time
                    
                resized_wrape_coord_image = cv2.resize(warped_coordinate, (warped_coordinate.shape[1] // 2, warped_coordinate.shape[0] // 2))
                coord_image = self.bridge.cv2_to_imgmsg(resized_wrape_coord_image, encoding='bgr8')
                self.publisher_coord_image.publish(coord_image)

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

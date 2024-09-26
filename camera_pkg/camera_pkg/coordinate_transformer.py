import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import cv2
import numpy as np
import ros2_numpy
from std_msgs.msg import Header

class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')
        self.publisher_ = self.create_publisher(PointStamped, 'ball_coordinates', 10)
        self.bridge = CvBridge()
        self.lower_green = np.array([35, 100, 100])
        self.upper_green = np.array([85, 255, 255])

        self.color_image = None
        self.depth_image = None
        self.depth_intrinsics = None

        # Custom transformation parameters
        source_point = np.array([238.28991449860578, 19.19139225783736, 518.0])
        target_point = np.array([0.18176895380020142, 0.6987220644950867, 0.06288695335388184])
        self.translation = self.calculate_translation(source_point, target_point)

        # Subscribe to camera topics
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.color_info_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info', self.depth_info_callback, 10)

    def calculate_translation(self, source_point, target_point):
        return target_point - source_point

    def apply_custom_transform(self, x, y, z):
        # Convert the point to a numpy array
        point = np.array([x, y, z])
        # Apply the translation
        transformed_point = point + self.translation
        return transformed_point

    def image_callback(self, msg):
        self.color_image = ros2_numpy.numpify(msg)
        self.detect_ball()

    def depth_callback(self, msg):
        self.depth_image = ros2_numpy.numpify(msg)

    def color_info_callback(self, msg):
        # Process camera color info if needed
        pass

    def depth_info_callback(self, msg):
        self.depth_intrinsics = {
            'width': msg.width,
            'height': msg.height,
            'ppx': msg.k[2],
            'ppy': msg.k[5],
            'fx': msg.k[0],
            'fy': msg.k[4]
        }

    def detect_ball(self):
        if self.color_image is None:
            return

        hsv_image = cv2.cvtColor(self.color_image, cv2.COLOR_BGR2HSV)
        mask_green = cv2.inRange(hsv_image, self.lower_green, self.upper_green)
        contours_green, _ = cv2.findContours(mask_green, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        detected_balls = 0
        published_balls = 0

        for contour in contours_green:
            area = cv2.contourArea(contour)
            if area > 500:
                epsilon = 0.02 * cv2.arcLength(contour, True)
                approx_polygon = cv2.approxPolyDP(contour, epsilon, True)
                M = cv2.moments(approx_polygon)
                if M['m00'] != 0:
                    cX = int(M['m10'] / M['m00'])
                    cY = int(M['m01'] / M['m00'])
                    center = (cX, cY)
                    detected_balls += 1

                    cv2.drawContours(self.color_image, [approx_polygon], -1, (0, 255, 0), 2)
                    cv2.circle(self.color_image, center, 5, (0, 0, 255), -1)
                    cv2.putText(self.color_image, "Center", center, cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    if self.depth_image is not None and self.depth_intrinsics is not None:
                        if 0 <= center[1] < self.depth_image.shape[0] and 0 <= center[0] < self.depth_image.shape[1]:
                            depth_value = self.depth_image[center[1], center[0]]
                            if depth_value > 0:
                                x = (center[0] - self.depth_intrinsics['ppx']) * depth_value / self.depth_intrinsics['fx']
                                y = (center[1] - self.depth_intrinsics['ppy']) * depth_value / self.depth_intrinsics['fy']
                                z = float(depth_value)  # Ensure depth is a float

                                # Log the depth and coordinates before transformation
                                self.get_logger().info(f"Depth value: {depth_value}")
                                self.get_logger().info(f"Calculated coordinates: x={x}, y={y}, z={z}")

                                # Apply custom transformation
                                transformed_point = self.apply_custom_transform(x, y, z)

                                point_stamped = PointStamped()
                                point_stamped.header.stamp = self.get_clock().now().to_msg()
                                point_stamped.header.frame_id = 'base_link'
                                point_stamped.point.x = transformed_point[0]
                                point_stamped.point.y = transformed_point[1]
                                point_stamped.point.z = transformed_point[2]

                                self.publisher_.publish(point_stamped)
                                published_balls += 1
                                self.get_logger().info(f"Published transformed ball coordinates: x={transformed_point[0]}, y={transformed_point[1]}, z={transformed_point[2]}")

        self.get_logger().info(f"Detected balls: {detected_balls}, Published balls: {published_balls}")

        # Display image
        cv2.imshow('RealSense', self.color_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = BallDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()

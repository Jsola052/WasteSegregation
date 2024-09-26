import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PointStamped
import cv2
import numpy as np
import ros2_numpy
from std_msgs.msg import Header
from ur_interface.srv import TransformPoint  

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

        # Create a service client for transforming the coordinates
        self.client = self.create_client(TransformPoint, 'transform_point')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for transform_point service...')

        # Subscribe to camera topics
        self.create_subscription(Image, '/camera/camera/color/image_raw', self.image_callback, 10)
        self.create_subscription(Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info', self.color_info_callback, 10)
        self.create_subscription(CameraInfo, '/camera/camera/depth/camera_info', self.depth_info_callback, 10)

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

                                # Log the depth and coordinates before service call
                                self.get_logger().info(f"Depth value: {depth_value}")
                                self.get_logger().info(f"Calculated coordinates: x={x}, y={y}, z={z}")

                                # Call the transform_point service
                                transformed_point = self.call_transform_service(x, y, z)

                                # If the service call was successful, publish the transformed point
                                if transformed_point is not None:
                                    point_stamped = PointStamped()
                                    point_stamped.header.stamp = self.get_clock().now().to_msg()
                                    point_stamped.header.frame_id = 'base_link'
                                    point_stamped.point.x = transformed_point.point.x
                                    point_stamped.point.y = transformed_point.point.y
                                    point_stamped.point.z = transformed_point.point.z

                                    self.publisher_.publish(point_stamped)
                                    published_balls += 1
                                    self.get_logger().info(f"Published transformed ball coordinates: x={transformed_point.point.x}, y={transformed_point.point.y}, z={transformed_point.point.z}")

        self.get_logger().info(f"Detected balls: {detected_balls}, Published balls: {published_balls}")


    def call_transform_service(self, x, y, z):
        # Create request
        self.get_logger().info(f"Starting Service Call...")
        request = TransformPoint.Request()
        point_stamped = PointStamped()
        point_stamped.header.stamp = self.get_clock().now().to_msg()
        point_stamped.header.frame_id = 'camera_depth_optical_frame'
        point_stamped.point.x = x
        point_stamped.point.y = y
        point_stamped.point.z = z
        request.input_point = point_stamped

        # Call service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            return future.result().transformed_point
        else:
            self.get_logger().error('Service call failed')
            return None

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

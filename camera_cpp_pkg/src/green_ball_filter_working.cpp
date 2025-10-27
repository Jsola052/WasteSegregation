#!/usr/bin/env python3
import cv2
import numpy as np
import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import String
from cv_bridge import CvBridge
import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

# ---------------------------
# AUTO LABELER CLASS (HSV-BASED DETECTOR)
# ---------------------------
class AutoLabeler:
    def __init__(self):
        # Define HSV color ranges for cap detection.
        self.color_ranges = {
            'red': [
                {'lower': np.array([0, 100, 100]), 'upper': np.array([10, 255, 255])},
                {'lower': np.array([160, 100, 100]), 'upper': np.array([179, 255, 255])}
            ],
            'blue': [
                {'lower': np.array([100, 100, 100]), 'upper': np.array([130, 255, 255])}
            ],
            'white': [
                {'lower': np.array([0, 0, 200]), 'upper': np.array([180, 30, 255])}
            ],
            # Black is not processed in this detection.
            'black': [
                {'lower': np.array([0, 0, 0]), 'upper': np.array([180, 255, 30])}
            ]
        }
        # Map colors to class IDs.
        self.class_map = {
            'red': 0,
            'blue': 1,
            'white': 2,
            'black': 3
        }

    def detect_caps(self, image):
        """
        Detect vial caps in the provided BGR image based on color.
        Returns a list of detections where each detection is a tuple:
           (class_id, x_norm, y_norm, w_norm, h_norm)
        The x_norm, y_norm values are normalized coordinates (0 to 1).
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        height, width = image.shape[:2]
        detections = []
        min_cap_area = 500
        max_cap_area = 5000

        for color, ranges in self.color_ranges.items():
            # Skip black in detection.
            if color == 'black':
                continue

            # Create an empty mask for each color.
            mask = np.zeros((height, width), dtype=np.uint8)
            for range_dict in ranges:
                color_mask = cv2.inRange(hsv, range_dict['lower'], range_dict['upper'])
                mask = cv2.bitwise_or(mask, color_mask)

            # Remove noise.
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if min_cap_area < area < max_cap_area:
                    x, y, w, h = cv2.boundingRect(contour)
                    perimeter = cv2.arcLength(contour, True)
                    circularity = 4 * np.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
                    # Enforce a near-circular criterion.
                    if circularity > 0.6:
                        x_center = (x + w / 2) / width
                        y_center = (y + h / 2) / height
                        w_norm = w / width
                        h_norm = h / height
                        class_id = self.class_map[color]
                        detections.append((class_id, x_center, y_center, w_norm, h_norm))
        return detections

# ---------------------------
# REAL-TIME VIAL DETECTION NODE
# ---------------------------
class VialDetectionRealTimeNode(Node):
    def __init__(self):
        super().__init__('vial_detection_real_time_node')
        self.get_logger().info('Vial Detection Real-Time Node started.')

        # Subscribers for the required topics.
        self.color_sub = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.color_callback,
            10
        )
        self.depth_sub = self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.depth_callback,
            10
        )
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.camera_info_callback,
            10
        )

        # Publisher for the final arranged vial positions.
        self.positions_pub = self.create_publisher(String, "vial_positions", 10)

        # For converting between ROS Image messages and OpenCV images.
        self.bridge = CvBridge()
        # Instance of the vial detector.
        self.labeler = AutoLabeler()

        # TF2 for coordinate transforms.
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Storage for the latest depth image and camera info.
        self.latest_depth = None
        self.camera_info = None

        # Arrangement parameters for target positions (predefined in the robot's base frame)
        # These are absolute positions in "base_link" where the UR16 should place the vials.
        self.red_target_start = (0.2, 0.8, 0.0)
        self.wb_target_start = (0.2, 0.6, 0.0)
        self.target_spacing = 0.1

    def camera_info_callback(self, msg):
        self.camera_info = msg
        # Log the received camera info once.
        self.get_logger().info("Camera info received.")

    def depth_callback(self, msg):
        try:
            # Convert depth image to a NumPy array.
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            self.latest_depth = depth_image
        except Exception as e:
            self.get_logger().error(f"Depth conversion failed: {e}")

    def color_callback(self, msg):
        if self.camera_info is None or self.latest_depth is None:
            self.get_logger().warn("Waiting for camera info and depth image...")
            return

        try:
            cv_color = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:
            self.get_logger().error(f"CV bridge conversion failed: {e}")
            return

        # Get image dimensions.
        height, width, _ = cv_color.shape

        # Detect vial caps using the labeler.
        detections = self.labeler.detect_caps(cv_color)
        if not detections:
            self.get_logger().info("No vials detected in current frame.")
            return

        # Prepare lists for grouping detections by color.
        red_detections = [d for d in detections if d[0] == self.labeler.class_map['red']]
        blue_detections = [d for d in detections if d[0] == self.labeler.class_map['blue']]
        white_detections = [d for d in detections if d[0] == self.labeler.class_map['white']]

        # Sort each group by x-coordinate (normalized).
        red_detections.sort(key=lambda d: d[1])
        white_detections.sort(key=lambda d: d[1])
        blue_detections.sort(key=lambda d: d[1])

        # Combine detections in the desired order: red first, then white, then blue.
        ordered_detections = red_detections + white_detections + blue_detections

        output_data = []
        # Counters for target arrangement.
        red_count = 0
        wb_count = 0

        for det in ordered_detections:
            class_id, x_norm, y_norm, _, _ = det
            # Convert normalized coordinates to pixel indices.
            u = int(x_norm * width)
            v = int(y_norm * height)
            # Make sure the pixel is within the depth image bounds.
            if u < 0 or v < 0 or u >= width or v >= height:
                continue
            # Retrieve depth at the given pixel.
            depth = self.latest_depth[v, u]
            # Some depth cameras use 0 to indicate invalid measurements.
            if depth <= 0.0:
                self.get_logger().warn("Invalid depth value encountered; skipping detection.")
                continue

            # Use camera intrinsics from the CameraInfo message.
            fx = self.camera_info.k[0]
            fy = self.camera_info.k[4]
            cx = self.camera_info.k[2]
            cy = self.camera_info.k[5]

            # Convert pixel (u, v) and depth to a 3D point in the camera frame.
            x_camera = (u - cx) * depth / fx
            y_camera = (v - cy) * depth / fy
            z_camera = depth

            # Create a PointStamped in the camera frame.
            point_cam = PointStamped()
            point_cam.header.stamp = self.get_clock().now().to_msg()
            point_cam.header.frame_id = "camera_link"
            point_cam.point.x = x_camera
            point_cam.point.y = y_camera
            point_cam.point.z = z_camera

            # Transform the point from the camera frame to the robot's base frame.
            try:
                transform = self.tf_buffer.lookup_transform(
                    "base_link",      # target frame
                    "camera_link",    # source frame
                    rclpy.time.Time(),  # latest available transform
                    rclpy.duration.Duration(seconds=1.0)
                )
                point_world = tf2_geometry_msgs.do_transform_point(point_cam, transform)
            except Exception as e:
                self.get_logger().error(f"TF2 transform failed: {e}")
                continue

            detected_pose = {
                "x": point_world.point.x,
                "y": point_world.point.y,
                "z": point_world.point.z
            }

            # Assign target arrangement based on color.
            if class_id == self.labeler.class_map['red']:
                target_x = self.red_target_start[0] + red_count * self.target_spacing
                target_y = self.red_target_start[1]
                red_count += 1
                color = "red"
            elif class_id == self.labeler.class_map['white']:
                target_x = self.wb_target_start[0] + wb_count * self.target_spacing
                target_y = self.wb_target_start[1]
                wb_count += 1
                color = "white"
            elif class_id == self.labeler.class_map['blue']:
                target_x = self.wb_target_start[0] + wb_count * self.target_spacing
                target_y = self.wb_target_start[1]
                wb_count += 1
                color = "blue"
            else:
                target_x, target_y = 0.0, 0.0
                color = "unknown"

            target_pose = {"x": target_x, "y": target_y, "z": 0.0}
            output_data.append({
                "color": color,
                "detected_pose": detected_pose,
                "target_pose": target_pose
            })

        # Publish the detection and arrangement data as a JSON string.
        msg_out = String()
        msg_out.data = json.dumps(output_data)
        self.positions_pub.publish(msg_out)
        self.get_logger().info(f"Published vial positions: {msg_out.data}")

def main(args=None):
    rclpy.init(args=args)
    node = VialDetectionRealTimeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Vial Detection Real-Time Node.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
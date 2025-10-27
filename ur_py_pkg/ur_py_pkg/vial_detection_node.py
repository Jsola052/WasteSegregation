#!/usr/bin/env python3
import os
import cv2
import numpy as np
import random
import yaml
import time
import json

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

class AutoLabeler:
    def __init__(self):
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
            'black': [
                {'lower': np.array([0, 0, 0]), 'upper': np.array([180, 255, 30])}
            ]
        }
        self.class_map = {'red': 0, 'blue': 1, 'white': 2, 'black': 3}

    def detect_caps(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        height, width = image.shape[:2]
        caps = []
        min_cap_area = 500
        max_cap_area = 5000
        for color, ranges in self.color_ranges.items():
            if color == 'black':
                continue
            mask = np.zeros(image.shape[:2], dtype=np.uint8)
            for range_dict in ranges:
                color_mask = cv2.inRange(hsv, range_dict['lower'], range_dict['upper'])
                mask = cv2.bitwise_or(mask, color_mask)
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
                    if circularity > 0.6:
                        x_center = (x + w/2) / width
                        y_center = (y + h/2) / height
                        w_norm = w / width
                        h_norm = h / height
                        class_id = self.class_map[color]
                        caps.append((class_id, x_center, y_center, w_norm, h_norm))
        return caps

class VialDetectionRealTimeNode(Node):
    def __init__(self):
        super().__init__('vial_detection_realtime_node')
        self.get_logger().info("Vial Detection Real-Time Node started.")
        self.publisher_ = self.create_publisher(String, 'vial_positions', 10)
        self.image_subscriber = self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.labeler = AutoLabeler()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.TABLE_HEIGHT = 0.1
        self.camera_scale = 1.0
        self.red_target_start = (0.2, 0.8, self.TABLE_HEIGHT)
        self.wb_target_start = (0.2, 0.6, self.TABLE_HEIGHT)
        self.target_spacing = 0.1

    def convert_to_world(self, x_norm, y_norm):
        x_camera = (x_norm - 0.5) * self.camera_scale
        y_camera = (y_norm - 0.5) * self.camera_scale
        z_camera = self.TABLE_HEIGHT
        point_cam = PointStamped()
        point_cam.header.stamp = self.get_clock().now().to_msg()  # Use current time
        point_cam.header.frame_id = "camera_link"
        point_cam.point.x = x_camera
        point_cam.point.y = y_camera
        point_cam.point.z = z_camera
        try:
            # Use latest available transform by passing an empty Time() object.
            transform = self.tf_buffer.lookup_transform(
                "base_link",
                "camera_link",
                rclpy.time.Time(),  # This means "latest transform"
                rclpy.duration.Duration(seconds=1.0)
            )
            point_world = tf2_geometry_msgs.do_transform_point(point_cam, transform)
            return point_world.point.x, point_world.point.y, point_world.point.z
        except Exception as e:
            self.get_logger().error(f"TF2 transform failed: {e}")
            return 0.0, 0.0, 0.0

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f"CV bridge conversion failed: {e}")
            return
        detections = self.labeler.detect_caps(cv_image)
        red_detections = [d for d in detections if d[0] == self.labeler.class_map['red']]
        blue_detections = [d for d in detections if d[0] == self.labeler.class_map['blue']]
        white_detections = [d for d in detections if d[0] == self.labeler.class_map['white']]
        red_detections.sort(key=lambda d: d[1])
        blue_detections.sort(key=lambda d: d[1])
        white_detections.sort(key=lambda d: d[1])
        ordered_detections = red_detections + white_detections + blue_detections
        output_data = []
        red_count = 0
        wb_count = 0
        for det in ordered_detections:
            class_id, x_center, y_center, w_norm, h_norm = det
            detected_x, detected_y, detected_z = self.convert_to_world(x_center, y_center)
            detected_pose = {"x": detected_x, "y": detected_y, "z": detected_z}
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
            target_pose = {"x": target_x, "y": target_y, "z": self.TABLE_HEIGHT}
            output_data.append({
                "color": color,
                "detected_pose": detected_pose,
                "target_pose": target_pose
            })
        msg_out = String()
        msg_out.data = json.dumps(output_data)
        self.publisher_.publish(msg_out)
        self.get_logger().info(f"Published vial positions: {msg_out.data}")

def main(args=None):
    rclpy.init(args=args)
    node = VialDetectionRealTimeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

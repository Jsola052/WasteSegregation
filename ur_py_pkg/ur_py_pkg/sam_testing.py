import os
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import String, Header
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray
from cv_bridge import CvBridge
import numpy as np
import cv2
import tf2_ros
import tf2_geometry_msgs
from rtde_control import RTDEControlInterface
import open3d as o3d
from ultralytics import YOLO
from message_filters import Subscriber, ApproximateTimeSynchronizer
from sensor_msgs_py import point_cloud2
from builtin_interfaces.msg import Time
from torch.serialization import add_safe_globals
from ultralytics.nn.tasks import SegmentationModel, DetectionModel, ClassificationModel
import torch
import torch.nn as nn
# === SAM imports ===
from segment_anything import sam_model_registry, SamPredictor


class DetectAndPickNode(Node):
    def __init__(self):
        super().__init__('bottles_demo_node')
        self.bridge = CvBridge()
        try:
            from ultralytics.nn.tasks import SegmentationModel, DetectionModel, ClassificationModel
            extra = [SegmentationModel, DetectionModel, ClassificationModel]
        except Exception:
            extra = []

        # === YOLO model ===
        model_path = os.path.expanduser('~/Downloads/weights2.pt')
        add_safe_globals([nn.Sequential, nn.ModuleList, nn.ModuleDict] + extra)        
        self.model = YOLO(model_path)
        self.model.to("cpu")
        print(f"[INFO] YOLO model loaded from {model_path}")

        # === SAM model ===
        sam_checkpoint = os.path.expanduser("/home/robotics/sam_vit_b_01ec64.pth")
        sam_model = sam_model_registry["vit_b"](checkpoint=sam_checkpoint)
        self.sam_predictor = SamPredictor(sam_model)
        print(f"[INFO] SAM model loaded from {sam_checkpoint}")

        # === TF and robot control ===
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.rtde_control = RTDEControlInterface("172.16.3.15")
        self.tool_pub = self.create_publisher(String, "ur_tools", 10)
        self.cloud_pub = self.create_publisher(PointCloud2, "/detected_objects_cloud", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/detected_object_markers", 10)

        # Camera intrinsics
        self.fx = self.fy = self.cx = self.cy = None
        self.intrinsics_logged = False
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.camera_info_callback, 10)

        # Sync color + depth
        self.color_sub = Subscriber(self, Image, "/camera/camera/color/image_raw")
        self.depth_sub = Subscriber(self, Image, "/camera/camera/depth/image_rect_raw")
        self.ts = ApproximateTimeSynchronizer([self.color_sub, self.depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.synced_callback)

        # TCP and orientation
        self.rtde_control.setTcp([0.000, 0.0, 0.336, 0.0, 0.0, 0.0])
        self.orientation = (0.002, 0.004)
        self.home = [-0.47988, 0.17318, 0.738, 0.002, 0.004, -1.569]

        # Class-specific drop-off poses
        self.class_locations = {
            "can": [[-0.607, -0.534, 0.739, 0.003, 0.002, -1.570], [-0.607, -0.534, 0.906, 0.003, 0.002, -1.570]],
            "bottle": [[-0.607, -0.534, 0.739, 0.003, 0.002, -1.570], [-0.607, -0.534, 0.906, 0.003, 0.002, -1.570]],
            "glove": [[-0.433, -0.203, 0.747, 0.003, 0.028, -1.570],
                      [-0.137, -0.356, 0.753, 0.026, -0.022, -1.570],
                      [-0.137, -0.356, 1.122, 0.026, -0.022, -1.570]],
            "glasses": [[-0.300, 0.300, 0.800, 0.003, 0.002, -1.570],
                        [-0.300, 0.300, 1.200, 0.003, 0.002, -1.570]],
            "screwdriver": [[-0.142, 0.497, 0.750, 0.003, 0.002, -1.570],
                            [-0.142, 0.497, 1.357, 0.003, 0.002, -1.570]]
        }

        # Visualization / saving
        self.marker_id_counter = 0
        self.save_dir = Path.home() / "test_images/test3"
        self.save_dir.mkdir(parents=True, exist_ok=True)
        self.image_counter = 1
        self.processing = False
        self.subscription_active = True

        print("[INFO] Node initialized. Waiting for synchronized frames...")

    # === Camera intrinsics ===
    def camera_info_callback(self, msg: CameraInfo):
        self.fx, self.fy = msg.k[0], msg.k[4]
        self.cx, self.cy = msg.k[2], msg.k[5]
        if not self.intrinsics_logged:
            print("[INFO] Camera intrinsics received.")
            self.intrinsics_logged = True

    # === SAM-based centroid/orientation ===
    def get_centroid_from_sam(self, image, box):
        """
        Use SAM to refine object mask inside YOLO box and compute centroid + orientation.
        """
        self.sam_predictor.set_image(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        input_box = np.array([box], dtype=np.float32)

        masks, _, _ = self.sam_predictor.predict(
            point_coords=None,
            point_labels=None,
            box=input_box,
            multimask_output=False,
        )
        mask = masks[0].astype(np.uint8)

        # Compute centroid + orientation
        moments = cv2.moments(mask)
        if moments["m00"] == 0:
            return None, None

        cx = int(moments["m10"] / moments["m00"])
        cy = int(moments["m01"] / moments["m00"])
        mu20 = moments["mu20"] / moments["m00"]
        mu02 = moments["mu02"] / moments["m00"]
        mu11 = moments["mu11"] / moments["m00"]
        theta = 0.5 * np.arctan2(2 * mu11, mu20 - mu02)

        return (cx, cy), theta

    # === Marker handling ===
    def clear_markers(self):
        delete_array = MarkerArray()
        for i in range(self.marker_id_counter):
            for ns in ["detected_spheres", "orientation_vectors"]:
                m = Marker()
                m.header.frame_id = "base_link"
                m.header.stamp = self.get_clock().now().to_msg()
                m.ns = ns
                m.id = i
                m.action = Marker.DELETE
                delete_array.markers.append(m)
        self.marker_pub.publish(delete_array)
        self.marker_id_counter = 0

    def publish_marker_array(self, points):
        marker_array = MarkerArray()
        class_colors = {
            "can": (0.0, 1.0, 0.0),
            "bottle": (0.0, 0.5, 1.0),
            "glove": (1.0, 0.5, 0.0),
            "glasses": (0.6, 0.0, 1.0),
            "screwdriver": (1.0, 0.0, 0.0),
        }

        for i, (x, y, z, theta, label) in enumerate(points):
            color = class_colors.get(label, (1.0, 1.0, 1.0))

            sphere_marker = Marker()
            sphere_marker.header.frame_id = "base_link"
            sphere_marker.header.stamp = self.get_clock().now().to_msg()
            sphere_marker.ns = "detected_spheres"
            sphere_marker.id = i * 3
            sphere_marker.type = Marker.SPHERE
            sphere_marker.action = Marker.ADD
            sphere_marker.pose.position.x = -x
            sphere_marker.pose.position.y = -y
            sphere_marker.pose.position.z = z
            sphere_marker.pose.orientation.w = 1.0
            sphere_marker.scale.x = 0.05
            sphere_marker.scale.y = 0.05
            sphere_marker.scale.z = 0.05
            sphere_marker.color.r = color[0]
            sphere_marker.color.g = color[1]
            sphere_marker.color.b = color[2]
            sphere_marker.color.a = 1.0
            marker_array.markers.append(sphere_marker)

            arrow_marker = Marker()
            arrow_marker.header.frame_id = "base_link"
            arrow_marker.header.stamp = self.get_clock().now().to_msg()
            arrow_marker.ns = "orientation_vectors"
            arrow_marker.id = i * 3 + 1
            arrow_marker.type = Marker.ARROW
            arrow_marker.action = Marker.ADD
            arrow_marker.pose.position.x = -x
            arrow_marker.pose.position.y = -y
            arrow_marker.pose.position.z = z
            arrow_marker.pose.orientation.x = 0.0
            arrow_marker.pose.orientation.y = 0.0
            arrow_marker.pose.orientation.z = np.sin(theta / 2.0)
            arrow_marker.pose.orientation.w = np.cos(theta / 2.0)
            arrow_marker.scale.x = 0.1
            arrow_marker.scale.y = 0.01
            arrow_marker.scale.z = 0.01
            arrow_marker.color.r = color[0]
            arrow_marker.color.g = color[1]
            arrow_marker.color.b = color[2]
            arrow_marker.color.a = 1.0
            marker_array.markers.append(arrow_marker)

        self.marker_id_counter = len(points)
        self.marker_pub.publish(marker_array)

    # === Main callback ===
    def synced_callback(self, color_msg: Image, depth_msg: Image):
        if not self.subscription_active or self.processing or None in (self.fx, self.fy, self.cx, self.cy):
            return
        self.processing = True
        try:
            raw_color = self.bridge.imgmsg_to_cv2(color_msg, "bgr8")
            raw_depth = self.bridge.imgmsg_to_cv2(depth_msg, "passthrough")
            color_image = raw_color.copy()
            depth_for_open3d = raw_depth.copy()

            results = self.model.predict(source=color_image, verbose=False)[0]
            if len(results.boxes) == 0:
                print("[INFO] No objects detected.")
                self.processing = False
                return

            rz = -1.569
            color_h, color_w = color_image.shape[:2]
            depth_h, depth_w = depth_for_open3d.shape
            scale_x = depth_w / color_w
            scale_y = depth_h / color_h
            detected_points = []

            frame_best_label = None
            frame_best_coords = None
            frame_best_conf = 0.5

            for box in results.boxes:
                x1, y1, x2, y2 = [int(coord.item()) for coord in box.xyxy[0]]
                conf = float(box.conf[0])
                label = results.names[int(box.cls[0])].lower()

                if conf < 0.75:
                    continue

                centroid, orientation = self.get_centroid_from_sam(color_image, [x1, y1, x2, y2])
                if centroid is None or orientation is None:
                    continue

                cx, cy = centroid
                u = int(cx * scale_x)
                v = int(cy * scale_y)

                if not (0 <= u < depth_w and 0 <= v < depth_h):
                    continue

                depth = depth_for_open3d[v, u]
                if depth == 0:
                    continue

                depth = depth / 1000.0
                X = (u - self.cx * scale_x) * depth / (self.fx * scale_x)
                Y = (v - self.cy * scale_y) * depth / (self.fy * scale_y)
                Z = depth

                pt = PointStamped()
                pt.header.frame_id = "camera_color_optical_frame"
                pt.header.stamp = self.get_clock().now().to_msg()
                pt.point.x = float(X)
                pt.point.y = float(Y)
                pt.point.z = float(Z)

                try:
                    xf = self.tf_buffer.lookup_transform("base_link", pt.header.frame_id, Time(),
                                                         timeout=rclpy.duration.Duration(seconds=1))
                    pb = tf2_geometry_msgs.do_transform_point(pt, xf)
                    x, y, z = -pb.point.x, -pb.point.y, pb.point.z
                    detected_points.append((x, y, z, rz, label))

                    if label in self.class_locations and conf > frame_best_conf:
                        if z < 0.960 or z > 0.980:
                            z = 0.997
                        if not (-0.810 <= x <= -0.541) or not (-0.180 <= y <= 0.420):
                            continue
                        frame_best_label = label
                        frame_best_coords = (x, y, z)
                        frame_best_conf = conf

                except Exception as e:
                    self.get_logger().warn(f"TF transform failed: {repr(e)}")
                    continue

            if detected_points:
                self.publish_marker_array(detected_points)

            if frame_best_coords is None:
                print("[INFO] No valid object for picking in this frame.")
                return

            self.subscription_active = False
            time.sleep(2)
            self.clear_markers()

            x, y, z = frame_best_coords
            rx, ry = self.orientation
            vel, acc = 0.1, 0.1

            self.rtde_control.moveL(self.home, vel, acc)
            time.sleep(2)
            self.tool_pub.publish(String(data="tool on"))
            self.rtde_control.moveL([x, y, z - 0.1, rx, ry, rz], vel, acc)
            self.rtde_control.moveL([x, y, z, rx, ry, rz], vel, acc)
            time.sleep(2)
            self.clear_markers()
            self.rtde_control.moveL([x, y, z - 0.1, rx, ry, rz], vel, acc)
            self.rtde_control.moveL(self.home, vel, acc)
            time.sleep(2)

            for pose in self.class_locations[frame_best_label]:
                self.rtde_control.moveL(pose, vel, acc)
            self.tool_pub.publish(String(data="tool off"))
            time.sleep(1)
            for pose in reversed(self.class_locations[frame_best_label][:-1]):
                self.rtde_control.moveL(pose, vel, acc)

            self.rtde_control.moveL(self.home, vel, acc)
            time.sleep(2)
            print("[INFO] Pick-and-place complete.")
            time.sleep(1)

        finally:
            self.processing = False
            time.sleep(1)
            self.subscription_active = True

    def o3d_to_ros_cloud(self, cloud, frame_id="camera_color_optical_frame"):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = frame_id

        points = np.asarray(cloud.points)
        colors = (np.asarray(cloud.colors) * 255).astype(np.uint8)

        fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='r', offset=12, datatype=PointField.UINT8, count=1),
            PointField(name='g', offset=13, datatype=PointField.UINT8, count=1),
            PointField(name='b', offset=14, datatype=PointField.UINT8, count=1),
        ]

        cloud_data = [tuple(p) + tuple(c) for p, c in zip(points, colors)]
        return point_cloud2.create_cloud(header, fields, cloud_data)


def main(args=None):
    rclpy.init(args=args)
    node = DetectAndPickNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("[INFO] Node interrupted.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

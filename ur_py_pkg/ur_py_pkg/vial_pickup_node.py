#!/usr/bin/env python3
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class VialPositionSubscriber(Node):
    def __init__(self):
        super().__init__('vial_position_subscriber')
        self.subscription = self.create_subscription(
            String,
            'vial_positions',
            self.listener_callback,
            10
        )

    def listener_callback(self, msg):
        try:
            data = json.loads(msg.data)
        except Exception as e:
            self.get_logger().error(f"Failed to parse JSON: {e}")
            return

        # Group detections by color.
        grouped = {}
        for detection in data:
            color = detection.get("color", "unknown")
            if color not in grouped:
                grouped[color] = []
            grouped[color].append(detection)

        # Build a list of formatted strings with numbering for each color.
        output_list = []
        for color, detections in grouped.items():
            for i, det in enumerate(detections, start=1):
                name = f"vial {i} {color}"
                detected_pose = det.get("detected_pose", {})
                target_pose = det.get("target_pose", {})
                info = f"{name}: Detected Pose: {detected_pose}, Target Pose: {target_pose}"
                output_list.append(info)
                self.get_logger().info(info)

def main(args=None):
    rclpy.init(args=args)
    node = VialPositionSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

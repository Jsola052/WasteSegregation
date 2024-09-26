import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from ur_interface.srv import TransformPoint
import tf2_ros
import tf2_geometry_msgs

class TransformPointService(Node):
    def __init__(self):
        super().__init__('transform_point_service')
        self.srv = self.create_service(TransformPoint, 'transform_point', self.handle_transform_point)
        
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def handle_transform_point(self, request, response):
        try:
            point_in_camera_frame = PointStamped()
            point_in_camera_frame.header.stamp = self.get_clock().now().to_msg()
            point_in_camera_frame.header.frame_id = request.input_point.header.frame_id  
            point_in_camera_frame.point = request.input_point.point  
            transform = self.tf_buffer.lookup_transform('base_link',  
                                                        point_in_camera_frame.header.frame_id,  
                                                        rclpy.time.Time())  

            transformed_point = tf2_geometry_msgs.do_transform_point(point_in_camera_frame, transform)

            response.transformed_point = transformed_point
            self.get_logger().info(f"Transformed point: {transformed_point.point.x}, {transformed_point.point.y}, {transformed_point.point.z}")

        except Exception as e:
            self.get_logger().error(f"Error occurred while transforming point: {str(e)}")
            response.success = False
            return response

        response.success = True
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TransformPointService()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

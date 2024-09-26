import rclpy
from rclpy.node import Node
from pyfirmata import Arduino
from std_msgs.msg import String
from tool_changer_interface.srv import ToolChanger

class ToolChangerServerNode(Node):

    def __init__(self):
        super().__init__("tool_changer_server_node")
        self.server = self.create_service(ToolChanger, "tool_changer", self.callback_tool_changer)
        self.get_logger().info("Tool Changer server has been started.")
        self.publisher = self.create_publisher(String, "tool_changer_status", 10)
        self.success = self.create_publisher(String, "tool_changer_success", 10)
        self.board = Arduino('/dev/arduino')
        self.relay_pin_number = 7
        self.relay_pin = self.board.get_pin(f'd:{self.relay_pin_number}:o')

    def callback_tool_changer(self, request, response):
        self.relay_pin.write(request.a)
        response.success = True
        self.publish_tool_changer_status(request)
        self.get_logger().info("Successful tool change")
        self.publish_tool_changer_success(response)
        self.get_logger().info("Tool changer status has been updated")
        return response

    def publish_tool_changer_status(self, request):
        msg = String()
        if request.a == 0:
            msg.data = "Tool Changer is locked"
        elif(request.a == 1):
            msg.data = "Tool Changer is unlocked"
        self.publisher.publish(msg)
        
    def publish_tool_changer_success(self, response):
        msg = String()
        if response.success:
            msg.data = "Success"
        else:
            msg.data = "Fail"
        self.success.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ToolChangerServerNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()

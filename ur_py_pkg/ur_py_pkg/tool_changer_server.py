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
        self.tool_activation_pin = 7
        self.tool_relay_pin = self.board.get_pin(f'd:{self.tool_activation_pin}:o')
        self.tool_changer_activation_pin = 8
        self.tool_changer_relay_pin = self.board.get_pin(f'd:{self.tool_changer_activation_pin}:o')

    def callback_tool_changer(self, request, response):
        if 0 <= request.a < 2: 
            self.tool_changer_relay_pin.write(request.a)
            response.success = True
            self.publish_tool_changer_status(request)
            self.get_logger().info("Successful tool change")
            self.publish_tool_changer_success(response)
            self.get_logger().info("Tool changer status has been updated")
        elif request.a == 2: 
            self.tool_relay_pin.write(0)
            response.success = True
            self.publish_tool_changer_status(request)
            self.get_logger().info("Successful tool activation")
            self.publish_tool_changer_success(response)
            self.get_logger().info("Tool status has been updated")
        elif request.a == 3: 
            self.tool_relay_pin.write(1)
            response.success = True
            self.publish_tool_changer_status(request)
            self.get_logger().info("Successful tool activation")
            self.publish_tool_changer_success(response)
            self.get_logger().info("Tool status has been updated")
            #test comment
        return response
    

    def publish_tool_changer_status(self, request):
        msg = String()
        if request.a == 0:
            msg.data = "Tool Changer is locked"
        elif(request.a == 1):
            msg.data = "Tool Changer is unlocked"
        elif(request.a == 2):
            msg.data = "Tool Off"
        elif(request.a == 3):
            msg.data = "Tool On"
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

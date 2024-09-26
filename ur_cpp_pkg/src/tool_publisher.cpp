#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class GetToolPublisherNode : public rclcpp::Node // modify name
{
public:
    GetToolPublisherNode() : Node("GetToolPublisherNode") // modify name
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("ur_tools", 10);
        // timer_ = this->create_wall_timer(std::chrono::milliseconds(500),
        //                                  std::bind(&GetToolPublisherNode::publishTool, this));
        // RCLCPP_INFO(this->get_logger(), "get tool publisher has been started.");
        publishTool();
    }

private:
    void publishTool()
    {
        auto msg = std_msgs::msg::String();
        msg.data = std::string("get gripper");
        publisher_->publish(msg);
        msg.data = std::string("return gripper");
        publisher_->publish(msg);
    }
    std::string robot_name_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GetToolPublisherNode>(); // modify name
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

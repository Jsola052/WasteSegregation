#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class ToolChangerSuccessSubscriberNode : public rclcpp::Node
{
public:
    ToolChangerSuccessSubscriberNode() : Node("tool_changer_subscriber")
    {
        subscriber_ = this->create_subscription<std_msgs::msg::String>(
            "tool_changer_success", 10,
            std::bind(&ToolChangerSuccessSubscriberNode::callbackToolChanger, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(), "tool changer success subscriber has been started.");
    }

private:
    void callbackToolChanger(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ToolChangerSuccessSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

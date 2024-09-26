#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "tool_changer_interface/srv/tool_changer.hpp"

using String = std_msgs::msg::String;
using namespace std::placeholders;
using namespace std::chrono_literals;

class GetToolSubscriberNode : public rclcpp::Node {
public:
    GetToolSubscriberNode()
        : Node("get_tool"), success(false)
    {
        // Creating client for tool changer service
        client_ = create_client<tool_changer_interface::srv::ToolChanger>("tool_changer");
        while (!client_->wait_for_service(std::chrono::seconds(1)))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for the service to appear...");
        }

        // Creating subscriber for get_tool topic
        subscriber_ = create_subscription<String>(
            "ur_tools", 10,
            std::bind(&GetToolSubscriberNode::callbackGetTool, this, _1));

        // Creating subscriber for tool_changer_success topic
        subscriber_success_ = create_subscription<String>(
            "tool_changer_success", 10,
            std::bind(&GetToolSubscriberNode::callbackSuccess, this, _1));

        RCLCPP_INFO(this->get_logger(), "Get tool subscriber has been started.");
    }

private:
    void send_request(int64_t a)
    {
        auto request = std::make_shared<tool_changer_interface::srv::ToolChanger::Request>();
        request->a = a;

        // Send asynchronous request
        auto future = client_->async_send_request(request,
            std::bind(&GetToolSubscriberNode::handle_response, this, _1));
    }

    void handle_response(rclcpp::Client<tool_changer_interface::srv::ToolChanger>::SharedFuture future)
    {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Response received.");

        // Wait until success is true
        while (!success)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for tool change to complete...");
            std::this_thread::sleep_for(1s);
        }

        RCLCPP_INFO(this->get_logger(), "Tool change completed successfully.");
    }

    // void home()
    // {
    //     robot_.moveJ({-1.57, -1.57, -1.57, -1.57, 1.57, 0}, 0.6, 0.6);
    // }

    void callbackGetTool(const String::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "%s", msg->data.c_str());
        success = false; // Reset the success flag for each new tool command

        // if (msg->data == "get gripper")
        // {
        //     getGripper();
        // }
        // else if (msg->data == "return gripper")
        // {
        //     returnGripper();
        // }
        // else if (msg->data == "get suction cup")
        // {
        //     getSuctionCup();
        // }
        // else if (msg->data == "return suction cup")
        // {
        //     returnSuctionCup();
        // }
        // else if (msg->data == "home")
        // {
        //     home();
        // } 
        if (msg->data == "tool on")
        {
            tool_on();
        }
        else if (msg->data == "tool off")
        {
            tool_off();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "No valid message received. Check status.");
        }
    }

    void tool_on(){
        send_request(1);
    }
    void tool_off(){
        send_request(0);
    }

    // void getGripper()
    // {
    //     home();
    //     robot_.setTcp({0, 0, 0, 0, 0, 0});
    //     robot_.moveJ({-1.57, -1.57, -1.57, -1.57, 1.57, 1.57}, 0.6, 0.6);
    //     sleep(1.5);
    //     send_request(1);
    //     sleep(1.5);
    //     home();
    //     sleep(1.5);
    //     send_request(0);
    //     sleep(1.5);
    // }
    
    // void returnGripper()
    // {
    //     home();
    //     robot_.setTcp({0, 0, 0, 0, 0, 0});
    //     robot_.moveJ({-1.57, -1.57, -1.57, -1.57, 1.57, 1.57}, 0.6, 0.6);
    //     sleep(1.5);
    //     send_request(1);
    //     sleep(1.5);
    //     home();
    //     sleep(1.5);
    //     send_request(0);
    //     sleep(1.5);
    // }

    //  void getSuctionCup()
    // {
    //     home();
    //     robot_.setTcp({0, 0, 0, 0, 0, 0});
    //     home();
    // }

    // void returnSuctionCup()
    // {
    //     home();
    //     robot_.setTcp({0, 0, 0, 0, 0, 0});
    //     home();
    // }

    void callbackSuccess(const String::SharedPtr msg)
    {
        if (msg->data == "Success")
        {
            RCLCPP_INFO(this->get_logger(), "Received Success message.");
            success = true; // Set success flag when "Success" message is received
        }
    }

    // RTDEControlInterface &robot_;
    rclcpp::Subscription<String>::SharedPtr subscriber_;
    rclcpp::Subscription<String>::SharedPtr subscriber_success_;
    rclcpp::Client<tool_changer_interface::srv::ToolChanger>::SharedPtr client_;

    bool success;
};

int main(int argc, char **argv)
{
    // Initialize ROS
    rclcpp::init(argc, argv);

    // Create RTDEControlInterface object for robot control
    // RTDEControlInterface rtde_control("172.16.3.114");

    auto node = std::make_shared<GetToolSubscriberNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}

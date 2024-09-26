#include <memory>
#include <vector>
#include <iostream>
#include "rclcpp/rclcpp.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include <ur_rtde/rtde_control_interface.h>
#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using namespace ur_rtde;

class BallPickupDemoNode : public rclcpp::Node {
public:
    BallPickupDemoNode(RTDEControlInterface &rtde_control)
    : Node("ball_pickup_demo"), 
      robot_(rtde_control), 
      tf_buffer_(this->get_clock()), 
      tf_listener_(tf_buffer_)
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/green_ball_position", 10,
            std::bind(&BallPickupDemoNode::topic_callback, this, std::placeholders::_1));
        
        publisher_ = this->create_publisher<std_msgs::msg::String>("ur_tools", 10);

        timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&BallPickupDemoNode::pick_up_balls, this));
    }

private:
    void topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // Transform point to base_link frame
        try {
            geometry_msgs::msg::PointStamped transformed_point;
            tf_buffer_.transform(*msg, transformed_point, "base_link", tf2::durationFromSec(1.0));
            
            // Apply filtering logic
            double x = transformed_point.point.x * -1000;  // Convert to mm
            double y = transformed_point.point.y * -1000;  // Convert to mm
            double z = transformed_point.point.z * 1000;  // Convert to mm
            
            // Check x and y value ranges
            if (x >= -340 && x <= 10 && y >= -750 && y <= -340) {
                // Ensure z value is at least 28 mm
                if (z < 38) {
                    transformed_point.point.z = 0.041; // Set z to 28 mm
                }
                // Add the point to the list for pickup
                points_.push_back(transformed_point.point);
            } else {
                RCLCPP_INFO(this->get_logger(), "Point discarded due to range: x: %.2f, y: %.2f, z: %.2f", x, y, z);
            }

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Could not transform point: %s", ex.what());
        }
    }

    void publish_message(const std::string &action)
    {
        auto msg = std_msgs::msg::String();
        msg.data = action;
        publisher_->publish(msg);
    }

    void home()
    {
        robot_.moveJ({-1.57, -1.57, -1.57, -1.57, 1.57, 0.0}, 0.6, 0.6);
    }

    void log_target_position(const std::vector<double>& target_position) {
        std::ostringstream oss;
        oss << "[";
        for (size_t i = 0; i < target_position.size(); ++i) {
            oss << target_position[i];
            if (i < target_position.size() - 1) {
                oss << ", ";
            }
        }
        oss << "]";
        RCLCPP_INFO(this->get_logger(), "Target position: %s", oss.str().c_str());
    }

    void pick_up_balls()
    {
        if (points_.empty()) {
            RCLCPP_INFO(this->get_logger(), "No points to pick up.");
            return;
        }

        robot_.setTcp({0.0, 0.0, 0.142, 0, 0, 0});
        RCLCPP_INFO(this->get_logger(), "Pick-up started.");

        for (const auto &point : points_) {
            std::vector<double> target_position = {-point.x, -point.y, point.z, 0, 3.14, 0};
            log_target_position(target_position);

            // Uncomment to perform actual pickup operations
            home();
            publish_message("tool on");
            robot_.moveL(target_position, 0.1, 0.1);
            sleep(0.8);
            home();
            robot_.moveL({-0.613, -0.479, 0.311, 0.001, 3.145, -0.001}, 0.2, 0.2);
            sleep(0.8);
            publish_message("tool off");
            sleep(0.8);
            home();
        }

        points_.clear();
        RCLCPP_INFO(this->get_logger(), "Pick-up finished.");
    }

    RTDEControlInterface &robot_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<geometry_msgs::msg::Point> points_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    RTDEControlInterface rtde_control("172.16.3.114");
    rtde_control.moveJ({-1.57, -1.57, -1.57, -1.57, 1.57, 0}, 0.6, 0.6);
    auto node = std::make_shared<BallPickupDemoNode>(rtde_control);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

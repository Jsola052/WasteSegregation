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
      tf_buffer_(this->get_clock(), tf2::Duration(std::chrono::seconds(10))),  // Extended TF buffer duration
      tf_listener_(tf_buffer_)
    {
        publisher_ = this->create_publisher<std_msgs::msg::String>("ur_tools", 10);
        publish_message("Standby");
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/green_ball_position", 10,
            std::bind(&BallPickupDemoNode::topic_callback, this, std::placeholders::_1));
    }

private:
    void topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        // Check if the transform is available
        if (!tf_buffer_.canTransform("base_link", msg->header.frame_id, tf2::TimePointZero, tf2::durationFromSec(1.0))) {
            RCLCPP_WARN(this->get_logger(), "Transform not available yet for frame: %s", msg->header.frame_id.c_str());
            return;
        }

        // Transform point to base_link frame
        try {
            geometry_msgs::msg::PointStamped transformed_point;

            // Use a 1-second timeout instead of tf2::TimePointZero
            tf_buffer_.transform(*msg, transformed_point, "base_link", tf2::durationFromSec(1.0));

            double x = transformed_point.point.x * -1000;  // Convert to mm
            double y = transformed_point.point.y * -1000;  // Convert to mm
            double z = transformed_point.point.z * 1000;   // Convert to mm

            if (x <= -531 && x >= -733 && y >= -110 && y <= 355) {
                if (z <= 925 || z >= 935) {
                    transformed_point.point.z = 0.93086;  // Set z to 930 mm
                }

                points_.push_back(transformed_point.point);
                pick_up_balls();
            } else {
                RCLCPP_INFO(this->get_logger(), "Point discarded due to range: x: %.2f, y: %.2f, z: %.2f", x, y, z);
            }

        } catch (const tf2::TransformException &ex) {
            RCLCPP_WARN(this->get_logger(), "Transform exception: %s", ex.what());
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
        robot_.moveJ({3.14, -1.84, -2.18, -0.68, -1.57, 0.0}, 0.5, 0.5);
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
        publish_message("Operation");

        robot_.setTcp({0.0, 0.0, 0.140, 0, 0, 0});
        RCLCPP_INFO(this->get_logger(), "Pick-up started.");
        
        for (const auto &point : points_) {
            std::vector<double> target_position = {-point.x, -point.y, point.z, 0.005, 0.002, -1.569};
            log_target_position(target_position);

            home();
            publish_message("tool on");
            robot_.moveL(target_position, 0.1, 0.1);
            sleep(0.8);
            home();
            robot_.moveJ({0, -1.84, -2.18, -0.68, -1.57, 0.0}, 0.5, 0.5);
            robot_.moveJ({0, -1.61, -1.697, -1.40, -1.57, 0.0}, 0.7, 0.7);
            sleep(0.8);
            publish_message("tool off");
            sleep(0.8);
            robot_.moveJ({0, -1.84, -2.18, -0.68, -1.57, 0.0}, 0.7, 0.7);
            home();
        }

        points_.clear();
        RCLCPP_INFO(this->get_logger(), "Pick-up finished.");
        publish_message("Standby");
    }

    RTDEControlInterface &robot_;
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    std::vector<geometry_msgs::msg::Point> points_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    RTDEControlInterface rtde_control("172.16.3.15");
    rtde_control.moveJ({3.14, -1.84, -2.18, -0.68, -1.57, 0.0}, 0.6, 0.6);
    auto node = std::make_shared<BallPickupDemoNode>(rtde_control);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

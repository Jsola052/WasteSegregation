#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <ur_rtde/robotiq_gripper.h>
#include <thread>
#include <chrono>
#include <memory>
#include <sstream>

using std::placeholders::_1;
using namespace ur_rtde;

class GripperControllerNode : public rclcpp::Node
{
public:
  GripperControllerNode()
      : Node("gripper_controller_node")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing gripper...");
    gripper_ = std::make_shared<RobotiqGripper>("127.0.0.1", 63352, true);

    try
    {
      gripper_->connect();

      if (!gripper_->isActive())
      {
        RCLCPP_INFO(this->get_logger(), "Activating gripper...");
        gripper_->activate();
        std::this_thread::sleep_for(std::chrono::seconds(1));
      }

      gripper_->setSpeed(0.5);
      gripper_->setForce(0.5);
    }
    catch (const std::exception &e)
    {
      RCLCPP_ERROR(this->get_logger(), "Failed to initialize gripper: %s", e.what());
      return;
    }

    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "/gripper_move", 10, std::bind(&GripperControllerNode::topic_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    std::istringstream ss(msg->data);
    std::string command;
    ss >> command;

    if (command == "open")
    {
      gripper_->open(1.0, speed_, RobotiqGripper::WAIT_FINISHED);
      RCLCPP_INFO(this->get_logger(), "Gripper opened.");
    }
    else if (command == "close")
    {
      gripper_->close(1.0, speed_, RobotiqGripper::WAIT_FINISHED);
      RCLCPP_INFO(this->get_logger(), "Gripper closed.");
    }
    else if (command == "set_force")
    {
      double force_val;
      if (ss >> force_val && force_val >= 0.0 && force_val <= 1.0)
      {
        force_ = force_val;
        gripper_->setForce(force_);
        RCLCPP_INFO(this->get_logger(), "Force set to %.2f", force_);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Invalid force value. Use range 0.0–1.0");
      }
    }
    else if (command == "set_speed")
    {
      double speed_val;
      if (ss >> speed_val && speed_val >= 0.0 && speed_val <= 1.0)
      {
        speed_ = speed_val;
        gripper_->setSpeed(speed_);
        RCLCPP_INFO(this->get_logger(), "Speed set to %.2f", speed_);
      }
      else
      {
        RCLCPP_WARN(this->get_logger(), "Invalid speed value. Use range 0.0–1.0");
      }
    }
    else if (command == "status")
    {
      int status = gripper_->objectDetectionStatus();
      printStatus(status);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Unknown command: '%s'", msg->data.c_str());
    }
  }

  void printStatus(int Status)
  {
    switch (Status)
    {
    case RobotiqGripper::MOVING:
      RCLCPP_INFO(this->get_logger(), "Status: moving");
      break;
    case RobotiqGripper::STOPPED_OUTER_OBJECT:
      RCLCPP_INFO(this->get_logger(), "Status: outer object detected");
      break;
    case RobotiqGripper::STOPPED_INNER_OBJECT:
      RCLCPP_INFO(this->get_logger(), "Status: inner object detected");
      break;
    case RobotiqGripper::AT_DEST:
      RCLCPP_INFO(this->get_logger(), "Status: at destination");
      break;
    default:
      RCLCPP_INFO(this->get_logger(), "Status: unknown (%d)", Status);
      break;
    }
  }

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  std::shared_ptr<RobotiqGripper> gripper_;
  double force_ = 0.5;
  double speed_ = 0.5;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperControllerNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

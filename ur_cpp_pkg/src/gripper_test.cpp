#include <rclcpp/rclcpp.hpp>
#include <ur_rtde/robotiq_gripper.h>
#include <chrono>
#include <thread>

using namespace std::chrono_literals;
using namespace ur_rtde;

class GripperTestNode : public rclcpp::Node
{
public:
  GripperTestNode()
  : Node("gripper_test_node")
  {
    RCLCPP_INFO(this->get_logger(), "Starting gripper test...");

    gripper_ = std::make_shared<RobotiqGripper>("127.0.0.1", 63352, true);

    gripper_->connect();  


    if (!gripper_->isActive()) {
      gripper_->emergencyRelease(RobotiqGripper::OPEN);
    }

    RCLCPP_INFO(this->get_logger(), "Fault status: 0x%X", gripper_->faultStatus());

    gripper_->activate();
    std::this_thread::sleep_for(2s);

    gripper_->setUnit(RobotiqGripper::POSITION, RobotiqGripper::UNIT_NORMALIZED);
    gripper_->setForce(0.0);
    gripper_->setSpeed(0.5);

    testMotion();
  }

private:
  std::shared_ptr<RobotiqGripper> gripper_;

  void printStatus(int status)
  {
    std::string result;
    switch (status) {
      case RobotiqGripper::MOVING:
        result = "moving";
        break;
      case RobotiqGripper::STOPPED_OUTER_OBJECT:
        result = "outer object detected";
        break;
      case RobotiqGripper::STOPPED_INNER_OBJECT:
        result = "inner object detected";
        break;
      case RobotiqGripper::AT_DEST:
        result = "at destination";
        break;
      default:
        result = "unknown";
    }
    RCLCPP_INFO(this->get_logger(), "Status: %s", result.c_str());
  }

  void testMotion()
  {
    int status = gripper_->move(1, 1, 0, RobotiqGripper::WAIT_FINISHED);
    printStatus(status);

    status = gripper_->move(0, 1, 0, RobotiqGripper::WAIT_FINISHED);
    printStatus(status);

    gripper_->setUnit(RobotiqGripper::POSITION, RobotiqGripper::UNIT_MM);
    gripper_->setPositionRange_mm(50);
    gripper_->move(50);
    printStatus(gripper_->waitForMotionComplete());

    gripper_->move(10);
    printStatus(gripper_->waitForMotionComplete());

    RCLCPP_INFO(this->get_logger(), "Opening gripper...");
    status = gripper_->open();
    printStatus(gripper_->waitForMotionComplete());

    gripper_->close(0.02, 0, RobotiqGripper::START_MOVE);
    printStatus(gripper_->waitForMotionComplete());

    status = gripper_->open(1.0, 0.0, RobotiqGripper::WAIT_FINISHED);
    printStatus(status);

    gripper_->disconnect();
    RCLCPP_INFO(this->get_logger(), "Gripper test complete. Disconnected.");
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<GripperTestNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

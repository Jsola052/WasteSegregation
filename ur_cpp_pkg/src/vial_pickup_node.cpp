#include <memory>
#include <vector>
#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>
#include <nlohmann/json.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using json = nlohmann::json;
using namespace ur_rtde;

struct VialPose {
  geometry_msgs::msg::Point detected;
  geometry_msgs::msg::Point target;
};

class VialPickupNode : public rclcpp::Node {
public:
  VialPickupNode(RTDEControlInterface &rtde_control, RTDEReceiveInterface &rtde_receive)
  : Node("vial_pickup_node"),
    robot_(rtde_control),
    robot_r_(rtde_receive),
    tf_buffer_(this->get_clock()),
    tf_listener_(tf_buffer_)
  {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "vial_positions", 10,
      std::bind(&VialPickupNode::poseCallback, this, std::placeholders::_1)
    );

    publisher_ = this->create_publisher<std_msgs::msg::String>("ur_tools", 10);

    timer_ = this->create_wall_timer(
      std::chrono::seconds(5),
      std::bind(&VialPickupNode::processVials, this)
    );

    speed_ = 0.1;
    accel_ = 0.1;
    gripper_dio_ = 1;

    // Store home joint configuration
    home_pose_ = robot_r_.getActualQ();
  }

private:
  void poseCallback(const std_msgs::msg::String::SharedPtr msg) {
    std::vector<VialPose> tmp;
    try {
      auto arr = json::parse(msg->data);
      for (auto &d : arr) {
        geometry_msgs::msg::PointStamped in_pt, out_pt;
        in_pt.header.frame_id = "camera_link";
        {
          auto now = this->get_clock()->now();
          in_pt.header.stamp.sec = now.seconds();
          in_pt.header.stamp.nanosec = now.nanoseconds() % 1000000000u;
        }
        in_pt.point.x = d["detected_pose"]["x"];
        in_pt.point.y = d["detected_pose"]["y"];
        in_pt.point.z = d["detected_pose"]["z"];
        try {
          tf_buffer_.transform(in_pt, out_pt, "base_link", tf2::durationFromSec(1.0));
        } catch (const tf2::TransformException &ex) {
          RCLCPP_WARN(this->get_logger(), "TF transform failed: %s", ex.what());
          continue;
        }

        VialPose vp;
        vp.detected = out_pt.point;
        vp.target.x = d["target_pose"]["x"];
        vp.target.y = d["target_pose"]["y"];
        vp.target.z = d["target_pose"]["z"];
        tmp.push_back(vp);
      }
    } catch (const json::parse_error &e) {
      RCLCPP_ERROR(this->get_logger(), "JSON parse error: %s", e.what());
    }
    vials_ = std::move(tmp);
  }

  void processVials() {
    if (vials_.empty()) {
      RCLCPP_INFO(this->get_logger(), "No vials to process.");
      return;
    }

    robot_.setTcp({0, 0, 0.14, 0, 0, 0});
    RCLCPP_INFO(this->get_logger(), "Starting vial pick-and-place for %zu vial(s)", vials_.size());

    for (size_t idx = 0; idx < vials_.size(); ++idx) {
      const auto &vp = vials_[idx];
      RCLCPP_INFO(this->get_logger(), "[Vial %zu] Detected at x=%.3f, y=%.3f, z=%.3f; Target at x=%.3f, y=%.3f, z=%.3f",
                  idx+1,
                  vp.detected.x, vp.detected.y, vp.detected.z,
                  vp.target.x, vp.target.y, vp.target.z);
      pickAndPlace(vp.detected, vp.target, idx+1);
    }

    vials_.clear();
    RCLCPP_INFO(this->get_logger(), "Completed all vials.");
  }

  void pickAndPlace(const geometry_msgs::msg::Point &det,
                    const geometry_msgs::msg::Point &tgt,
                    size_t idx) {
    // Approach above detected
    std::vector<double> approach = {det.x, det.y, det.z + 0.20, 0, 0, 0};
    RCLCPP_INFO(this->get_logger(), "[Vial %zu] moveL to approach (%.3f, %.3f, %.3f)",
                idx, approach[0], approach[1], approach[2]);
    robot_.moveL(approach, speed_, accel_, true);

    // Lower to pick
    std::vector<double> pick = {det.x, det.y, det.z + 0.02, 0, 0, 0};
    RCLCPP_INFO(this->get_logger(), "[Vial %zu] moveL to pick (%.3f, %.3f, %.3f)",
                idx, pick[0], pick[1], pick[2]);
    robot_.moveL(pick, speed_/2, accel_/2, true);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Lift up
    RCLCPP_INFO(this->get_logger(), "[Vial %zu] moveL to lift (%.3f, %.3f, %.3f)",
                idx, approach[0], approach[1], approach[2]);
    robot_.moveL(approach, speed_, accel_, true);

    // Approach above target
    std::vector<double> approach_t = {tgt.x, tgt.y, tgt.z + 0.20, 0, 0, 0};
    RCLCPP_INFO(this->get_logger(), "[Vial %zu] moveL to approach target (%.3f, %.3f, %.3f)",
                idx, approach_t[0], approach_t[1], approach_t[2]);
    robot_.moveL(approach_t, speed_, accel_, true);

    // Lower to place
    std::vector<double> place = {tgt.x, tgt.y, tgt.z + 0.02, 0, 0, 0};
    RCLCPP_INFO(this->get_logger(), "[Vial %zu] moveL to place (%.3f, %.3f, %.3f)",
                idx, place[0], place[1], place[2]);
    robot_.moveL(place, speed_/2, accel_/2, true);

    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    // Retract
    RCLCPP_INFO(this->get_logger(), "[Vial %zu] moveL to retract (%.3f, %.3f, %.3f)",
                idx, approach_t[0], approach_t[1], approach_t[2]);
    robot_.moveL(approach_t, speed_, accel_, true);

    // Return home
    RCLCPP_INFO(this->get_logger(), "[Vial %zu] moveJ to home position", idx);
    robot_.moveJ(home_pose_, speed_, accel_, true);
  }

  RTDEControlInterface &robot_;
  RTDEReceiveInterface &robot_r_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::vector<VialPose> vials_;
  double speed_, accel_;
  int gripper_dio_;
  std::vector<double> home_pose_;
  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char * argv[]) {
  rclcpp::init(argc, argv);
  RTDEControlInterface rtde_control("172.16.3.15");
  RTDEReceiveInterface rtde_receive("172.16.3.15");
  auto node = std::make_shared<VialPickupNode>(rtde_control, rtde_receive);
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

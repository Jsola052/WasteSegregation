#include <memory>
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

using namespace std::chrono_literals;

class URBasicControlNode : public rclcpp::Node
{
public:
  URBasicControlNode()
  : Node("ur_basic_control_node")
  {
    // --- 1.  Retrieve robot IP from a parameter so you don’t re-compile to change it ----------
    this->declare_parameter<std::string>("robot_ip", "172.16.3.15");
    std::string robot_ip = this->get_parameter("robot_ip").as_string();

    RCLCPP_INFO(get_logger(), "Connecting to UR16e at %s with RTDE…", robot_ip.c_str());

    try
    {
      rtde_control_ = std::make_shared<ur_rtde::RTDEControlInterface>(robot_ip);

      if (!rtde_control_->isConnected())
      {
        RCLCPP_ERROR(get_logger(), "Failed to connect to UR16e.");
        rclcpp::shutdown();
        return;
      }
      RCLCPP_INFO(get_logger(), "Successfully connected to UR16e!");

      // --- 2.  Move to an initial joint position ------------------------------------------------
      std::vector<double> home = {3.12, -2.0, -2.25, -0.436,  -1.57, 0.0};

      RCLCPP_INFO(get_logger(), "Moving to initial joint position…");
      rtde_control_->moveJ(home, /*speed=*/0.5, /*acc=*/0.5);
      


      std::this_thread::sleep_for(2s);

      // --- 3.  Move to a Cartesian pose ---------------------------------------------------------
      std::vector<double> move1 = {-0.570, 0.0757, 0.738, 0.003, 0.002, -1.570};
      std::vector<double> move2 = {-0.570, 0.0757, 0.9394, 0.003, 0.002, -1.570};
      std::vector<double> move3 = {-0.570, 0.0757, 0.672, 0.003, 0.002, -1.570};
      std::vector<double> move4 = {-0.6179, -0.5536, 0.672, 0.003, 0.002, -1.570};

      RCLCPP_INFO(get_logger(), "Moving linearly to target pose…");
      rtde_control_->moveL(move1, /*speed=*/0.1, /*acc=*/0.1);
      rtde_control_->moveL(move2, /*speed=*/0.1, /*acc=*/0.1);

      std::this_thread::sleep_for(5s);

      rtde_control_->moveL(move3, /*speed=*/0.1, /*acc=*/0.1);
      rtde_control_->moveL(move4, /*speed=*/0.1, /*acc=*/0.1);

      std::this_thread::sleep_for(5s);

      rtde_control_->moveJ(home, /*speed=*/0.1, /*acc=*/0.1);

      std::this_thread::sleep_for(2s);

      RCLCPP_INFO(get_logger(), "Motion complete!");

      // --- 4.  Gracefully end the script and disconnect -----------------------------------------
      rtde_control_->stopScript();
      rtde_control_->disconnect();
    }
    catch (const std::exception &ex)
    {
      RCLCPP_ERROR(get_logger(), "Exception: %s", ex.what());
    }

    // Shutdown once the work is done so rclcpp::spin() returns immediately.
    rclcpp::shutdown();
  }

private:
  std::shared_ptr<ur_rtde::RTDEControlInterface> rtde_control_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<URBasicControlNode>();
  rclcpp::spin(node);     // returns almost immediately because the node shut itself down
  return 0;
}

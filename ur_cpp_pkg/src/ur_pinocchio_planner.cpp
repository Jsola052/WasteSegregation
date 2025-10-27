#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <ur_rtde/rtde_control_interface.h>
#include <ur_rtde/rtde_receive_interface.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>

using namespace pinocchio;

class URPinocchioPlanner : public rclcpp::Node {
public:
    URPinocchioPlanner()
    : Node("ur_pinocchio_planner_node"),
      model(),
      data(model) {

        std::string urdf_xacro = "/home/robotics/ur16_ros2_ws/src/Universal_Robots_ROS2_Description/urdf/ur.urdf.xacro";
        std::string urdf_file = "/tmp/ur16e.urdf";

        RCLCPP_INFO(this->get_logger(), "URDF (xacro) path: %s", urdf_xacro.c_str());

        std::string cmd = "xacro " + urdf_xacro +
            " ur_type:=ur16e name:=ur16e prefix:=\"\" visual:=true > " + urdf_file;

        if (system(cmd.c_str()) != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to run xacro.");
            return;
        }

        // Load model into Pinocchio
        pinocchio::urdf::buildModel(urdf_file, model);
        data = pinocchio::Data(model);

        RCLCPP_INFO(this->get_logger(), "Pinocchio model loaded: %s", model.name.c_str());

        try {
            std::string robot_ip = "172.16.3.15";
            ur_rtde::RTDEControlInterface rtde_control(robot_ip);
            ur_rtde::RTDEReceiveInterface rtde_receive(robot_ip);

            // Read current joint values from robot
            std::vector<double> current_q = rtde_receive.getActualQ();
            RCLCPP_INFO(this->get_logger(), "Current joint positions received.");

            q_init = Eigen::VectorXd::Zero(model.nq);
            for (size_t i = 0; i < current_q.size(); ++i) {
                q_init[i] = current_q[i];
            }

            // Set goal: rotate joint 1 slightly
            Eigen::VectorXd q_goal = q_init;
            q_goal[0] += 0.2;

            // Interpolate trajectory
            const int steps = 50;
            std::vector<std::vector<double>> joint_trajectory;
            for (int i = 0; i <= steps; ++i) {
                double alpha = static_cast<double>(i) / steps;
                Eigen::VectorXd q_interp = (1 - alpha) * q_init + alpha * q_goal;
                joint_trajectory.emplace_back(q_interp.data(), q_interp.data() + q_interp.size());
            }

            RCLCPP_INFO(this->get_logger(), "Printing planned joint trajectory:");
            for (const auto& joints : joint_trajectory) {
                rtde_control.servoJ(joints, 0.5, 0.5, 0.05, 0.1, 200);
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }
            
            RCLCPP_INFO(this->get_logger(), "Holding final pose before stopping servo...");
            for (int i = 0; i < 10; ++i) {
                rtde_control.servoJ(joint_trajectory.back(), 0.05, 0.1, 0.05, 0.05, 100);  // lower speed & gain
                rclcpp::sleep_for(std::chrono::milliseconds(50));
            }

            rtde_control.servoStop();
            RCLCPP_INFO(this->get_logger(), "Smooth stop complete.");

            rtde_control.stopScript();
            RCLCPP_INFO(this->get_logger(), "Trajectory printing complete.");
            

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "RTDE error: %s", e.what());
        }
    }

private:
    Model model;
    Data data;
    Eigen::VectorXd q_init;

    void printVector(const std::vector<double>& vec) {
        std::cout << "[";
        for (size_t i = 0; i < vec.size(); ++i) {
            std::cout << vec[i];
            if (i != vec.size() - 1)
                std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<URPinocchioPlanner>());
    rclcpp::shutdown();
    return 0;
}

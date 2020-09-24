
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/joint_command_handle.hpp>
#include <hardware_interface/joint_state_handle.hpp>
#include <hardware_interface/robot_hardware.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "crane_plus_control/crane_plus_driver.hpp"


class CranePlusInterface : public hardware_interface::RobotHardware
{
public:
  CranePlusInterface();
  ~CranePlusInterface();
  hardware_interface::return_type init();
  hardware_interface::return_type read();
  hardware_interface::return_type write();

private:
  std::vector<hardware_interface::JointStateHandle> joint_state_handles_;
  std::vector<hardware_interface::JointCommandHandle> joint_command_handles_;
  std::vector<hardware_interface::OperationModeHandle> joint_mode_handles_;

  std::vector<std::string> joint_names_;

  std::vector<double> pos_;
  std::vector<double> vel_;
  std::vector<double> eff_;
  std::vector<double> cmd_;
  std::vector<hardware_interface::OperationMode> op_mode_;

  std::shared_ptr<CranePlusDriver> driver_;
};

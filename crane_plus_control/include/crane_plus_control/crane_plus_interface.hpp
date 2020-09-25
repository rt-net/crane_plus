
#include <controller_interface/controller_interface.hpp>
#include <hardware_interface/joint_handle.hpp>
#include <hardware_interface/robot_hardware.hpp>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <string>

#include "crane_plus_control/crane_plus_driver.hpp"


class CranePlusInterface : public hardware_interface::RobotHardware
{

using PosHandle = hardware_interface::JointHandle;
using PosCmdHandle = hardware_interface::JointHandle;

public:
  CranePlusInterface();
  ~CranePlusInterface();
  hardware_interface::return_type init();
  hardware_interface::return_type read();
  hardware_interface::return_type write();

private:
  std::vector<hardware_interface::OperationModeHandle> joint_mode_handles_;
  std::vector<std::shared_ptr<PosHandle>> joint_pos_handles_;
  std::vector<std::shared_ptr<PosCmdHandle>> joint_pos_cmd_handles_;

  std::vector<std::string> joint_names_;
  std::vector<hardware_interface::OperationMode> op_mode_;

  std::shared_ptr<CranePlusDriver> driver_;
};

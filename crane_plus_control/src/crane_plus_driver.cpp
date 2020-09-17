
#include "crane_plus_control/crane_plus_driver.hpp"

// static const rclcpp::Logger LOGGER = rclcpp::get_logger("fake_joint_driver");

CranePlusDriver::CranePlusDriver()
{
}

CranePlusDriver::~CranePlusDriver()
{
}

hardware_interface::return_type CranePlusDriver::init()
{
  joint_names_.push_back("crane_plus_joint1");
  joint_names_.push_back("crane_plus_joint2");
  joint_names_.push_back("crane_plus_joint3");
  joint_names_.push_back("crane_plus_joint4");

  // Resize members
  pos_.resize(joint_names_.size());
  vel_.resize(joint_names_.size());
  eff_.resize(joint_names_.size());
  cmd_.resize(joint_names_.size());
  op_mode_.resize(joint_names_.size());
  joint_state_handles_.resize(joint_names_.size());
  joint_command_handles_.resize(joint_names_.size());
  joint_mode_handles_.resize(joint_names_.size());

  size_t i = 0;
  for (auto & joint_name : joint_names_) {

    hardware_interface::JointStateHandle state_handle(joint_name, &pos_[i], &vel_[i], &eff_[i]);
    joint_state_handles_[i] = state_handle;

    if (register_joint_state_handle(&joint_state_handles_[i]) != hardware_interface::return_type::OK) {
      throw std::runtime_error("unable to register " + joint_state_handles_[i].get_name());
    }

    hardware_interface::JointCommandHandle command_handle(joint_name, &cmd_[i]);
    joint_command_handles_[i] = command_handle;
    if (register_joint_command_handle(&joint_command_handles_[i]) !=
      hardware_interface::return_type::OK)
    {
      throw std::runtime_error("unable to register " + joint_command_handles_[i].get_name());
    }

    joint_mode_handles_[i] = hardware_interface::OperationModeHandle(joint_name, &op_mode_[i]);
    if (register_operation_mode_handle(&joint_mode_handles_[i]) != hardware_interface::return_type::OK)
    {
      throw std::runtime_error("unable to register " + joint_mode_handles_[i].get_name());
    }
    ++i;
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CranePlusDriver::update()
{
  pos_ = cmd_;
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CranePlusDriver::read()
{

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CranePlusDriver::write()
{
  return hardware_interface::return_type::OK;
}
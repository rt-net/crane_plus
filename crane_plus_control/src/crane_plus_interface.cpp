
#include "crane_plus_control/crane_plus_interface.hpp"

static const rclcpp::Logger LOGGER = rclcpp::get_logger("crane_plus_interface");

CranePlusInterface::CranePlusInterface()
{
}

CranePlusInterface::~CranePlusInterface()
{
  driver_->torque_enable(false);
  driver_->close_port();
}

hardware_interface::return_type CranePlusInterface::init()
{
  std::vector<uint8_t> id_list{1, 2, 3, 4, 5};
  driver_ = std::make_shared<CranePlusDriver>("/dev/ttyUSB0", 1000000, id_list);

  if(!driver_->open_port()){
    throw std::runtime_error(driver_->get_last_error_log());
  }

  joint_names_.push_back("crane_plus_joint1");
  joint_names_.push_back("crane_plus_joint2");
  joint_names_.push_back("crane_plus_joint3");
  joint_names_.push_back("crane_plus_joint4");
  joint_names_.push_back("crane_plus_joint_hand");

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

  driver_->torque_enable(true);

  read();  // set current joint positions to pos_. 
  for(size_t i = 0; i < cmd_.size(); i++){
    cmd_[i] = pos_[i];  // set current joint positions to target positions cmd_.
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CranePlusInterface::read()
{
  std::vector<double> joint_positions;
  if(!driver_->read_present_joint_positions(&joint_positions)){
    RCLCPP_ERROR(LOGGER, driver_->get_last_error_log());
    return hardware_interface::return_type::ERROR;

  }else if(pos_.size() != joint_positions.size()){
    RCLCPP_ERROR(LOGGER, "vectors size does not match. pos_:%d, joint_positions:%d",
      pos_.size(), joint_positions.size());
    return hardware_interface::return_type::ERROR;

  }else{
    for(size_t i=0; i < pos_.size(); ++i){
      pos_[i] = joint_positions[i];
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CranePlusInterface::write()
{
  if(!driver_->write_goal_joint_positions(cmd_)){
    RCLCPP_ERROR(LOGGER, driver_->get_last_error_log());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

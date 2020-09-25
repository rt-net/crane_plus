
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

  joint_names_.push_back("crane_plus_joint1");
  joint_names_.push_back("crane_plus_joint2");
  joint_names_.push_back("crane_plus_joint3");
  joint_names_.push_back("crane_plus_joint4");
  joint_names_.push_back("crane_plus_joint_hand");

  // Resize members
  op_mode_.resize(joint_names_.size());
  joint_mode_handles_.resize(joint_names_.size());

  if(!driver_->open_port()){
    throw std::runtime_error(driver_->get_last_error_log());
  }

  for (size_t i = 0; i < joint_names_.size(); i++) {
    if (register_joint(joint_names_[i], "position") != hardware_interface::return_type::OK)
      throw std::runtime_error("unable to register position " + joint_names_[i]);

    if (register_joint(joint_names_[i], "position_command") != hardware_interface::return_type::OK)
      throw std::runtime_error("unable to register position_command " + joint_names_[i]);

    if (register_joint(joint_names_[i], "velocity") != hardware_interface::return_type::OK)
      throw std::runtime_error("unable to register velocity " + joint_names_[i]);

    if (register_joint(joint_names_[i], "velocity_command") != hardware_interface::return_type::OK)
      throw std::runtime_error("unable to register velocity_command " + joint_names_[i]);

    if (register_joint(joint_names_[i], "effort") != hardware_interface::return_type::OK)
      throw std::runtime_error("unable to register effort " + joint_names_[i]);

    if (register_joint(joint_names_[i], "effort_command") != hardware_interface::return_type::OK)
      throw std::runtime_error("unable to register effort_command " + joint_names_[i]);

    joint_mode_handles_[i] = hardware_interface::OperationModeHandle(joint_names_[i], &op_mode_[i]);
    if (register_operation_mode_handle(&joint_mode_handles_[i]) != hardware_interface::return_type::OK)
    {
      throw std::runtime_error("unable to register " + joint_mode_handles_[i].get_name());
    }
  }

  // get joint handles
  for(const auto & joint_name : joint_names_){
    auto pos_handle = std::make_shared<PosHandle>(joint_name, "position");
    auto pos_cmd_handle = std::make_shared<PosCmdHandle>(joint_name, "position_command");
    get_joint_handle(*pos_handle);
    get_joint_handle(*pos_cmd_handle);
    joint_pos_handles_.push_back(pos_handle);
    joint_pos_cmd_handles_.push_back(pos_cmd_handle);
  }

  read();  // set current joint positions to position handles. 
  for(size_t i=0; i < joint_pos_cmd_handles_.size(); ++i){
    // set current joint positions to position_command handles.
    joint_pos_cmd_handles_[i]->set_value(joint_pos_handles_[i]->get_value());
  }

  driver_->torque_enable(true);

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CranePlusInterface::read()
{
  std::vector<double> joint_positions;
  if(!driver_->read_present_joint_positions(&joint_positions)){
    RCLCPP_ERROR(LOGGER, driver_->get_last_error_log());
    return hardware_interface::return_type::ERROR;

  }else if(joint_pos_handles_.size() != joint_positions.size()){
    RCLCPP_ERROR(LOGGER, "vectors size does not match. joint_pos_handles_:%d, joint_positions:%d",
      joint_pos_handles_.size(), joint_positions.size());
    return hardware_interface::return_type::ERROR;

  }else{
    for(size_t i=0; i < joint_pos_handles_.size(); ++i){
      joint_pos_handles_[i]->set_value(joint_positions[i]);
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type CranePlusInterface::write()
{
  std::vector<double> pos_commands;
  for(size_t i=0; i < joint_pos_cmd_handles_.size(); ++i){
    pos_commands.push_back(joint_pos_cmd_handles_[i]->get_value());
  }

  if(!driver_->write_goal_joint_positions(pos_commands)){
    RCLCPP_ERROR(LOGGER, driver_->get_last_error_log());
    return hardware_interface::return_type::ERROR;
  }
  return hardware_interface::return_type::OK;
}

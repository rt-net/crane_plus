// Copyright 2021 RT Corporation
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.


#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "crane_plus_control/crane_plus_hardware.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"


namespace crane_plus_control
{

CranePlusHardware::~CranePlusHardware()
{
  driver_->torque_enable(false);
  driver_->close_port();
}

CallbackReturn CranePlusHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // Get parameters from URDF
  // Initialize member variables
  std::string port_name = info_.hardware_parameters["port_name"];
  int baudrate = std::stoi(info_.hardware_parameters["baudrate"]);
  timeout_seconds_ = std::stod(info_.hardware_parameters["timeout_seconds"]);
  read_velocities_ = std::stoi(info_.hardware_parameters["read_velocities"]);
  read_loads_ = std::stoi(info_.hardware_parameters["read_loads"]);
  read_voltages_ = std::stoi(info_.hardware_parameters["read_voltages"]);
  read_temperatures_ = std::stoi(info_.hardware_parameters["read_temperatures"]);

  std::vector<uint8_t> dxl_id_list;
  for (auto joint : info_.joints) {
    if (joint.parameters["dxl_id"] != "") {
      dxl_id_list.push_back(std::stoi(joint.parameters["dxl_id"]));
    } else {
      RCLCPP_ERROR(
        rclcpp::get_logger("CranePlusHardware"),
        "Joint '%s' does not have 'dxl_id' parameter.",
        joint.name.c_str());
      return CallbackReturn::ERROR;
    }
  }

  hw_position_commands_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_position_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_velocity_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_load_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_voltage_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());
  hw_temperature_states_.resize(info_.joints.size(), std::numeric_limits<double>::quiet_NaN());

  // Open a crane_plus_driver
  driver_ = std::make_shared<CranePlusDriver>(port_name, baudrate, dxl_id_list);
  if (!driver_->open_port()) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CranePlusHardware"), driver_->get_last_error_log().c_str());
    return CallbackReturn::ERROR;
  }
  if (!driver_->torque_enable(false)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CranePlusHardware"), driver_->get_last_error_log().c_str());
    return CallbackReturn::ERROR;
  }

  // Verify that the interface required by CranePlusHardware is set in the URDF.
  for (const hardware_interface::ComponentInfo & joint : info_.joints) {
    if (joint.command_interfaces.size() != 1) {
      RCLCPP_ERROR(
        rclcpp::get_logger("CranePlusHardware"),
        "Joint '%s' has %lu command interfaces found. 1 expected.",
        joint.name.c_str(), joint.command_interfaces.size());
      return CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_ERROR(
        rclcpp::get_logger("CranePlusHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return CallbackReturn::ERROR;
    }
  }

  steady_clock_ = rclcpp::Clock(RCL_STEADY_TIME);


  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
CranePlusHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_position_states_[i])
    );

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY,
        &hw_velocity_states_[i])
    );

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "load",
        &hw_load_states_[i])
    );
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "voltage",
        &hw_voltage_states_[i])
    );
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, "temperature",
        &hw_temperature_states_[i])
    );
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
CranePlusHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (uint i = 0; i < info_.joints.size(); i++) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION,
        &hw_position_commands_[i])
    );
  }

  return command_interfaces;
}

CallbackReturn CranePlusHardware::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!driver_->torque_enable(false)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CranePlusHardware"),
      driver_->get_last_error_log().c_str());
    return CallbackReturn::ERROR;
  }
  // Set current timestamp to disable the communication timeout.
  prev_comm_timestamp_ = steady_clock_.now();
  timeout_has_printed_ = false;

  // Set current joint positions to hw_position_commands.
  read(prev_comm_timestamp_, rclcpp::Duration::from_seconds(0));
  for (uint i = 0; i < hw_position_commands_.size(); i++) {
    hw_position_commands_[i] = hw_position_states_[i];
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn CranePlusHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  driver_->torque_enable(false);

  return CallbackReturn::SUCCESS;
}

return_type CranePlusHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (communication_timeout()) {
    if (!timeout_has_printed_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("CranePlusHardware"), "Communication timeout!");
      timeout_has_printed_ = true;
    }
    return return_type::ERROR;
  }

  std::vector<double> joint_positions;
  if (!driver_->read_present_joint_positions(joint_positions)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CranePlusHardware"),
      driver_->get_last_error_log().c_str());
    return return_type::ERROR;
  } else {
    for (uint i = 0; i < hw_position_states_.size(); ++i) {
      hw_position_states_[i] = joint_positions[i];
    }
  }

  // Reading joint speeds, loads, voltages or temperatures
  // causes a decrease of the communication rate.
  if (read_velocities_) {
    std::vector<double> joint_speeds;
    if (driver_->read_present_joint_speeds(joint_speeds)) {
      for (uint i = 0; i < hw_velocity_states_.size(); ++i) {
        hw_velocity_states_[i] = joint_speeds[i];
      }
    }
  }

  if (read_loads_) {
    std::vector<double> joint_loads;
    if (driver_->read_present_joint_loads(joint_loads)) {
      for (uint i = 0; i < hw_load_states_.size(); ++i) {
        hw_load_states_[i] = joint_loads[i];
      }
    }
  }

  if (read_voltages_) {
    std::vector<double> joint_voltages;
    if (driver_->read_present_joint_voltages(joint_voltages)) {
      for (uint i = 0; i < hw_voltage_states_.size(); ++i) {
        hw_voltage_states_[i] = joint_voltages[i];
      }
    }
  }


  if (read_temperatures_) {
    std::vector<double> joint_temperatures;
    if (driver_->read_present_joint_temperatures(joint_temperatures)) {
      for (uint i = 0; i < hw_temperature_states_.size(); ++i) {
        hw_temperature_states_[i] = joint_temperatures[i];
      }
    }
  }

  prev_comm_timestamp_ = steady_clock_.now();
  return return_type::OK;
}

return_type CranePlusHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (communication_timeout()) {
    if (!timeout_has_printed_) {
      RCLCPP_ERROR(
        rclcpp::get_logger("CranePlusHardware"), "Communication timeout!");
      timeout_has_printed_ = true;
    }
    return return_type::ERROR;
  }

  if (!driver_->write_goal_joint_positions(hw_position_commands_)) {
    RCLCPP_ERROR(
      rclcpp::get_logger("CranePlusHardware"),
      driver_->get_last_error_log().c_str());
    return return_type::ERROR;
  }

  prev_comm_timestamp_ = steady_clock_.now();
  return return_type::OK;
}

bool CranePlusHardware::communication_timeout()
{
  if (steady_clock_.now().seconds() - prev_comm_timestamp_.seconds() >= timeout_seconds_) {
    return true;
  } else {
    return false;
  }
}

}  // namespace crane_plus_control

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  crane_plus_control::CranePlusHardware,
  hardware_interface::SystemInterface
)

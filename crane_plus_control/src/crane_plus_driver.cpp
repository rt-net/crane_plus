// Copyright 2020 RT Corporation
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

#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "crane_plus_control/crane_plus_driver.hpp"

constexpr double PROTOCOL_VERSION = 1.0;
constexpr int DXL_HOME_POSITION = 511;  // value range:0 ~ 1023
constexpr double DXL_MAX_POSITION = 1023.0;
constexpr double DXL_MAX_POSITION_DEGREES = 300.0;
constexpr double TO_RADIANS = (DXL_MAX_POSITION_DEGREES / DXL_MAX_POSITION) * M_PI / 180.0;
constexpr double TO_DXL_POS = 1.0 / TO_RADIANS;
constexpr double TO_SPEED_REV_PER_MIN = 0.111;
constexpr double TO_SPEED_RAD_PER_MIN = TO_SPEED_REV_PER_MIN * 2.0 * M_PI;
constexpr double TO_SPEED_RAD_PER_SEC = TO_SPEED_RAD_PER_MIN / 60.0;
constexpr double TO_LOAD_PERCENT = 0.1;
constexpr double TO_VOLTAGE = 0.1;

// Dynamixel AX-12A address table
// Ref: https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/
constexpr uint16_t ADDR_TORQUE_ENABLE = 24;
constexpr uint16_t ADDR_GOAL_POSITION = 30;
constexpr uint16_t ADDR_MOVING_SPEED = 32;
constexpr uint16_t ADDR_PRESENT_POSITION = 36;
constexpr uint16_t ADDR_PRESENT_SPEED = 38;
constexpr uint16_t ADDR_PRESENT_LOAD = 40;
constexpr uint16_t ADDR_PRESENT_VOLTAGE = 42;
constexpr uint16_t ADDR_PRESENT_TEMPERATURE = 43;

CranePlusDriver::CranePlusDriver(
  const std::string port_name, const int baudrate,
  std::vector<uint8_t> id_list)
: baudrate_(baudrate), id_list_(id_list)
{
  dxl_port_handler_ = std::shared_ptr<dynamixel::PortHandler>(
    dynamixel::PortHandler::getPortHandler(port_name.c_str()));
  dxl_packet_handler_ = std::shared_ptr<dynamixel::PacketHandler>(
    dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION));
}

CranePlusDriver::~CranePlusDriver()
{
  close_port();
}

bool CranePlusDriver::open_port(void)
{
  if (!dxl_port_handler_->openPort()) {
    last_error_log_ = std::string(__func__) + ": unable to open dynamixel port: " +
      dxl_port_handler_->getPortName();
    return false;
  }

  if (!dxl_port_handler_->setBaudRate(baudrate_)) {
    last_error_log_ = std::string(__func__) + ": unable to set baudrate" +
      std::to_string(dxl_port_handler_->getBaudRate());
    return false;
  }

  return true;
}

void CranePlusDriver::close_port(void)
{
  dxl_port_handler_->closePort();
}

std::string CranePlusDriver::get_last_error_log(void)
{
  return last_error_log_;
}

bool CranePlusDriver::torque_enable(const bool enable)
{
  bool retval = true;
  for (auto dxl_id : id_list_) {
    uint8_t dxl_error = 0;
    int dxl_result = dxl_packet_handler_->write1ByteTxRx(
      dxl_port_handler_.get(),
      dxl_id, ADDR_TORQUE_ENABLE, enable, &dxl_error);

    if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
      retval = false;
    }
  }

  return retval;
}

bool CranePlusDriver::write_goal_joint_positions(const std::vector<double> & goal_positions)
{
  if (goal_positions.size() != id_list_.size()) {
    last_error_log_ = std::string(__func__) + ": vectors size does not match: " +
      " goal_positions:" + std::to_string(goal_positions.size()) +
      ", id_list:" + std::to_string(id_list_.size());
    return false;
  }

  bool retval = true;

  for (size_t i = 0; i < goal_positions.size(); i++) {
    uint8_t dxl_error = 0;
    uint16_t goal_position = radian_to_dxl_pos(goal_positions[i]);
    auto dxl_id = id_list_[i];
    int dxl_result = dxl_packet_handler_->write2ByteTxRx(
      dxl_port_handler_.get(),
      dxl_id, ADDR_GOAL_POSITION, goal_position, &dxl_error);

    if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
      retval = false;
    }
  }

  return retval;
}

bool CranePlusDriver::write_moving_speed_rpm(const uint8_t dxl_id, const double speed_rpm)
{
  const int DXL_MAX_MOVING_SPEED = 1023;
  const double SPEED_UNIT = 0.111;  // rpm
  if (std::find(id_list_.begin(), id_list_.end(), dxl_id) == id_list_.end()) {
    last_error_log_ = std::string(__func__) + ": dxl_id: " + std::to_string(dxl_id) +
      "not found.";
    return false;
  }

  int dxl_moving_speed = speed_rpm / SPEED_UNIT;
  if (dxl_moving_speed > DXL_MAX_MOVING_SPEED) {
    dxl_moving_speed = DXL_MAX_MOVING_SPEED;
  } else if (dxl_moving_speed == 0) {
    // If moving_speed is set to 0, it means the maximum rpm of the motor is used
    // without controlling the speed.
    dxl_moving_speed = 1;
  }

  bool retval = true;

  uint8_t dxl_error = 0;
  int dxl_result = dxl_packet_handler_->write2ByteTxRx(
    dxl_port_handler_.get(),
    dxl_id, ADDR_MOVING_SPEED, dxl_moving_speed, &dxl_error);

  retval = parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error);

  return retval;
}

bool CranePlusDriver::write_moving_speed_rpm_all(const double speed_rpm)
{
  bool retval = true;

  for (auto dxl_id : id_list_) {
    if (!write_moving_speed_rpm(dxl_id, speed_rpm)) {
      retval = false;
    }
  }

  return retval;
}

bool CranePlusDriver::read_present_joint_positions(std::vector<double> & joint_positions)
{
  std::vector<uint16_t> buffer;
  bool retval = read_word_data_from_each_joints(ADDR_PRESENT_POSITION, buffer);

  for (auto data : buffer) {
    joint_positions.push_back(dxl_pos_to_radian(data));
  }

  return retval;
}

bool CranePlusDriver::read_present_joint_speeds(std::vector<double> & joint_speeds)
{
  std::vector<uint16_t> buffer;
  bool retval = read_word_data_from_each_joints(ADDR_PRESENT_SPEED, buffer);

  for (auto data : buffer) {
    joint_speeds.push_back(dxl_speed_to_rps(data));
  }

  return retval;
}

bool CranePlusDriver::read_present_joint_loads(std::vector<double> & joint_loads)
{
  std::vector<uint16_t> buffer;
  bool retval = read_word_data_from_each_joints(ADDR_PRESENT_LOAD, buffer);

  for (auto data : buffer) {
    joint_loads.push_back(dxl_load_to_percent(data));
  }

  return retval;
}

bool CranePlusDriver::read_present_joint_voltages(std::vector<double> & joint_voltages)
{
  std::vector<uint8_t> buffer;
  bool retval = read_byte_data_from_each_joints(ADDR_PRESENT_VOLTAGE, buffer);

  for (auto data : buffer) {
    joint_voltages.push_back(dxl_voltage_to_actual_voltage(data));
  }

  return retval;
}

bool CranePlusDriver::read_present_joint_temperatures(std::vector<double> & joint_temperatures)
{
  std::vector<uint8_t> buffer;
  bool retval = read_byte_data_from_each_joints(ADDR_PRESENT_TEMPERATURE, buffer);

  for (auto data : buffer) {
    joint_temperatures.push_back(data);
  }

  return retval;
}

bool CranePlusDriver::read_byte_data_from_each_joints(
  const uint16_t address,
  std::vector<uint8_t> & buffer)
{
  bool retval = true;
  for (auto dxl_id : id_list_) {
    uint8_t dxl_error = 0;
    uint8_t data = 0;
    int dxl_result = dxl_packet_handler_->read1ByteTxRx(
      dxl_port_handler_.get(),
      dxl_id, address, &data, &dxl_error);

    if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
      retval = false;
    }

    buffer.push_back(data);
  }

  return retval;
}

bool CranePlusDriver::read_word_data_from_each_joints(
  const uint16_t address,
  std::vector<uint16_t> & buffer)
{
  bool retval = true;
  for (auto dxl_id : id_list_) {
    uint8_t dxl_error = 0;
    uint16_t data = 0;
    int dxl_result = dxl_packet_handler_->read2ByteTxRx(
      dxl_port_handler_.get(),
      dxl_id, address, &data, &dxl_error);

    if (!parse_dxl_error(std::string(__func__), dxl_id, dxl_result, dxl_error)) {
      retval = false;
    }

    buffer.push_back(data);
  }

  return retval;
}

bool CranePlusDriver::parse_dxl_error(
  const std::string func_name, const uint8_t dxl_id,
  const int dxl_comm_result, const uint8_t dxl_packet_error)
{
  bool retval = true;

  if (dxl_comm_result != COMM_SUCCESS) {
    last_error_log_ = func_name + ": dxl_id: " + std::to_string(dxl_id) + " :" +
      std::string(dxl_packet_handler_->getTxRxResult(dxl_comm_result));
    retval = false;
  }

  if (dxl_packet_error != 0) {
    last_error_log_ = func_name + ": dxl_id: " + std::to_string(dxl_id) + " :" +
      std::string(dxl_packet_handler_->getRxPacketError(dxl_packet_error));
    retval = false;
  }

  return retval;
}

double CranePlusDriver::dxl_pos_to_radian(const uint16_t position)
{
  return (position - DXL_HOME_POSITION) * TO_RADIANS;
}

uint16_t CranePlusDriver::radian_to_dxl_pos(const double position)
{
  return position * TO_DXL_POS + DXL_HOME_POSITION;
}

double CranePlusDriver::dxl_speed_to_rps(const uint16_t speed)
{
  if (speed < 1024) {  // CCW, positive rotation
    return speed * TO_SPEED_RAD_PER_SEC;
  } else {  // CW, negative rotation
    return -(speed - 1024) * TO_SPEED_RAD_PER_SEC;
  }
}

double CranePlusDriver::dxl_load_to_percent(const uint16_t load)
{
  if (load < 1024) {  // CCW, positive rotation
    return load * TO_LOAD_PERCENT;
  } else {  // CW, negative rotation
    return -(load - 1024) * TO_LOAD_PERCENT;
  }
}

double CranePlusDriver::dxl_voltage_to_actual_voltage(const uint8_t voltage)
{
  return voltage * TO_VOLTAGE;
}

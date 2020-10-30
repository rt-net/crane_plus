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

// Ref: https://github.com/ros-controls/ros2_control#writing-a-demo-for-your-own-robot

#include <memory>
#include <vector>

#include "controller_manager/controller_manager.hpp"
#include "crane_plus_control/crane_plus_interface.hpp"
#include "rclcpp/rclcpp.hpp"

static constexpr double SPIN_RATE = 100;  // Hz

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
  exe->spin();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Logger
  const rclcpp::Logger logger = rclcpp::get_logger("my_robot_logger");

  // create node to receive parameters
  auto control_param_node = rclcpp::Node::make_shared("control_param_node");
  control_param_node->declare_parameter("port_name", "/dev/ttyUSB0");
  control_param_node->declare_parameter("baudrate", 1000000);
  control_param_node->declare_parameter("joint_name_list");
  control_param_node->declare_parameter("dxl_id_list");
  control_param_node->declare_parameter("timeout_seconds", 5.0);

  auto port_name = control_param_node->get_parameter("port_name").as_string();
  auto baudrate = control_param_node->get_parameter("baudrate").as_int();
  auto joint_name_list = control_param_node->get_parameter("joint_name_list").as_string_array();
  auto TIMEOUT_SECONDS = control_param_node->get_parameter("timeout_seconds").as_double();

  // TODO(ShotaAk): Use byte_array for dxl_id_list
  auto id_list = control_param_node->get_parameter("dxl_id_list").as_integer_array();
  std::vector<uint8_t> dxl_id_list;
  for (auto dxl_id : id_list) {
    dxl_id_list.push_back(dxl_id);
  }

  // create my_robot instance
  auto my_robot = std::make_shared<CranePlusInterface>();

  // initialize the robot
  if (my_robot->init(
      port_name, baudrate, dxl_id_list,
      joint_name_list) != hardware_interface::return_type::OK)
  {
    RCLCPP_ERROR(logger, "failed to initialized crane_plus hardware");
    return -1;
  }

  auto executor =
    std::make_shared<rclcpp::executors::MultiThreadedExecutor>();

  // start the controller manager with the robot hardware
  controller_manager::ControllerManager cm(my_robot, executor);
  // load the joint state controller.
  // "ros_controllers" is the resource index from where to look for controllers
  // "ros_controllers::JointStateController" is the class we want to load
  // "my_robot_joint_state_controller" is the name for the node to spawn
  cm.load_controller(
    "crane_plus_joint_state_controller",
    "joint_state_controller/JointStateController");
  // load the trajectory controller
  cm.load_controller(
    "crane_plus_arm_controller",
    "joint_trajectory_controller/JointTrajectoryController");
  cm.load_controller(
    "crane_plus_gripper_controller",
    "joint_trajectory_controller/JointTrajectoryController");

  // there is no async spinner in ROS 2, so we have to put the spin() in its own thread
  auto future_handle = std::async(std::launch::async, spin, executor);

  // we can either configure each controller individually through its services
  // or we use the controller manager to configure every loaded controller
  if (cm.configure() != controller_interface::return_type::SUCCESS) {
    RCLCPP_ERROR(logger, "at least one controller failed to configure");
    return -1;
  }
  // and activate all controller
  if (cm.activate() != controller_interface::return_type::SUCCESS) {
    RCLCPP_ERROR(logger, "at least one controller failed to activate");
    return -1;
  }

  // main loop
  rclcpp::Rate rate(SPIN_RATE);
  rclcpp::Time last_successful_time;
  while (rclcpp::ok()) {
    if (my_robot->read() == hardware_interface::return_type::OK) {
      last_successful_time = rclcpp::Clock().now();
    }

    cm.update();

    if (my_robot->write() == hardware_interface::return_type::OK) {
      last_successful_time = rclcpp::Clock().now();
    }

    // check communication timeout
    if (rclcpp::Clock().now().seconds() - last_successful_time.seconds() >= TIMEOUT_SECONDS) {
      RCLCPP_ERROR(logger, "Communication timeout in %f seconds.", TIMEOUT_SECONDS);
      break;
    }

    rate.sleep();
  }

  executor->cancel();
}

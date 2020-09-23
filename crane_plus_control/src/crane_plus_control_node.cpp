
#include <controller_manager/controller_manager.hpp>
#include <crane_plus_control/crane_plus_interface.hpp>
#include <rclcpp/rclcpp.hpp>


static constexpr double SPIN_RATE = 200;  // Hz

void spin(std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exe)
{
  exe->spin();
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  // Logger
  const rclcpp::Logger logger = rclcpp::get_logger("my_robot_logger");
  
  // create my_robot instance
  auto my_robot = std::make_shared<CranePlusDriver>();
  
  // initialize the robot
  if (my_robot->init() != hardware_interface::return_type::OK) {
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
  hardware_interface::return_type ret;
  rclcpp::Rate rate(SPIN_RATE);
  while (rclcpp::ok()) {
    ret = my_robot->read();
    if (ret != hardware_interface::return_type::OK) {
      fprintf(stderr, "read failed!\n");
    }

    cm.update();

    ret = my_robot->write();
    if (ret != hardware_interface::return_type::OK) {
      fprintf(stderr, "write failed!\n");
    }

    rate.sleep();
  }

  executor->cancel();
}

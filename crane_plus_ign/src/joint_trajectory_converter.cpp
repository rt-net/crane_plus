#include <memory>

#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <rclcpp/rclcpp.hpp>

#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "crane_plus_ign/visibility_control.h"

namespace crane_plus_ign
{
class JTrajectoryConverter : public rclcpp::Node
{
public:
  using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
  using GoalHandleFollowJointTrajectory = rclcpp_action::ServerGoalHandle<FollowJointTrajectory>;
//   using Fibonacci = action_tutorials_interfaces::action::Fibonacci;
//   using GoalHandleFibonacci = rclcpp_action::ServerGoalHandle<Fibonacci>;

  CRANE_PLUS_IGN_PUBLIC
  explicit JTrajectoryConverter(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("JointTrajectoryConverter", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
      this->get_node_base_interface(),
      this->get_node_clock_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "crane_plus_arm_controller/follow_joint_trajectory",
      std::bind(&JTrajectoryConverter::handle_goal, this, _1, _2),
      std::bind(&JTrajectoryConverter::handle_cancel, this, _1),
      std::bind(&JTrajectoryConverter::handle_accepted, this, _1));
  }

private:
  rclcpp_action::Server<FollowJointTrajectory>::SharedPtr action_server_;

  CRANE_PLUS_IGN_LOCAL
  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const FollowJointTrajectory::Goal> goal)
  {
    for(auto joint_name : goal->trajectory.joint_names){
      RCLCPP_INFO(this->get_logger(), "Received joints: %s", joint_name.c_str());
    }
    
    RCLCPP_INFO(this->get_logger(), "Received goal request with order %d", goal->trajectory.joint_names[0]);
    (void)uuid;
    // if (goal->order > 9000) {
    //   return rclcpp_action::GoalResponse::REJECT;
    // }
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  CRANE_PLUS_IGN_LOCAL
  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  CRANE_PLUS_IGN_LOCAL
  void handle_accepted(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&JTrajectoryConverter::execute, this, _1), goal_handle}.detach();
  }

  CRANE_PLUS_IGN_LOCAL
  void execute(const std::shared_ptr<GoalHandleFollowJointTrajectory> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    // rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<FollowJointTrajectory::Feedback>();
    auto result = std::make_shared<FollowJointTrajectory::Result>();

    for (auto point : goal->trajectory.points){
      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // goal_handle->publish_feedback(feedback);
      // ここでpointのtime_from_startを見てTargetJointPositionをパブリッシュしたい
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->error_code = 0;
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class JTrajectoryConverter

}  // namespace crane_plus_ign

RCLCPP_COMPONENTS_REGISTER_NODE(crane_plus_ign::JTrajectoryConverter)
# Copyright 2020 RT Corporation
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import math

import rclpy
from geometry_msgs.msg import PoseStamped

# generic ros libraries
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)
from utils import plan_and_execute, euler_to_quaternion


def to_radians(deg_angle):
    return deg_angle * math.pi / 180.0


def main(args=None):
    # ros2の初期化
    rclpy.init(args=args)

    # ロガー生成
    logger = get_logger("pick_and_place")

    # MoveItPy初期化
    crane_plus = MoveItPy(node_name="moveit_py")
    crane_plus_arm = crane_plus.get_planning_component("arm")
    crane_plus_gripper = crane_plus.get_planning_component("gripper")
    logger.info("MoveItPy instance created")

    # instantiate a RobotState instance using the current robot model
    robot_model = crane_plus.get_robot_model()
    robot_state = RobotState(robot_model)

    # planningのパラメータ設定
    # armのパラメータ設定用
    arm_plan_request_params = PlanRequestParameters(
        crane_plus,
        "ompl_rrtc",
    )
    arm_plan_request_params.max_acceleration_scaling_factor = 0.1  # Set 0.0 ~ 1.0
    arm_plan_request_params.max_velocity_scaling_factor = 0.1  # Set 0.0 ~ 1.0

    # gripperのパラメータ設定用
    gripper_plan_request_params = PlanRequestParameters(
        crane_plus,
        "ompl_rrtc",
    )
    gripper_plan_request_params.max_acceleration_scaling_factor = 0.1  # Set 0.0 ~ 1.0
    gripper_plan_request_params.max_velocity_scaling_factor = 0.1  # Set 0.0 ~ 1.0

    # gripperの開閉角度
    GRIPPER_DEFAULT = 0.0
    GRIPPER_OPEN = math.radians(-30.0)
    GRIPPER_CLOSE = math.radians(10.0)

    # armを現在の位置から"vertical"ポジションに動かす
    crane_plus_arm.set_start_state_to_current_state()
    crane_plus_arm.set_goal_state(configuration_name="vertical")
    plan_and_execute(
        crane_plus,
        crane_plus_arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # gripperを0度の位置に動かす
    robot_state.set_joint_group_positions("gripper", [GRIPPER_DEFAULT])
    crane_plus_gripper.set_start_state_to_current_state()
    crane_plus_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_plus,
        crane_plus_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )
    
    # ----- Picking Preparation -----
    # armを現在の位置から"home"ポジションに動かす
    crane_plus_arm.set_start_state_to_current_state()
    crane_plus_arm.set_goal_state(configuration_name="home")
    plan_and_execute(
        crane_plus,
        crane_plus_arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # なにか掴んでいたときのためにgripperを開く
    robot_state.set_joint_group_positions("gripper", [GRIPPER_OPEN])
    crane_plus_gripper.set_start_state_to_current_state()
    crane_plus_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_plus,
        crane_plus_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    # armが掴みに行く位置を指定して動かす
    pose_goal = PoseStamped()
    pose_goal.header.frame_id = "base_link"
    pose_goal.pose.position.x = 0.0
    pose_goal.pose.position.y = -0.09
    pose_goal.pose.position.z = 0.17
    q = euler_to_quaternion(to_radians(0), to_radians(90), to_radians(-90))
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]
    crane_plus_arm.set_start_state_to_current_state()
    crane_plus_arm.set_goal_state(
        pose_stamped_msg=pose_goal, pose_link="crane_plus_link4"
    )
    plan_and_execute(
        crane_plus,
        crane_plus_arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # armの姿勢を変える(Y軸反転)
    q = euler_to_quaternion(to_radians(0), to_radians(180), to_radians(-90))
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]
    crane_plus_arm.set_start_state_to_current_state()
    crane_plus_arm.set_goal_state(
        pose_stamped_msg=pose_goal, pose_link="crane_plus_link4"
    )
    plan_and_execute(
        crane_plus,
        crane_plus_arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # armをz軸上に動かす
    pose_goal.pose.position.z = 0.14
    crane_plus_arm.set_start_state_to_current_state()
    crane_plus_arm.set_goal_state(
        pose_stamped_msg=pose_goal, pose_link="crane_plus_link4"
    )
    plan_and_execute(
        crane_plus,
        crane_plus_arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # Grasp
    robot_state.set_joint_group_positions("gripper", [GRIPPER_CLOSE])
    crane_plus_gripper.set_start_state_to_current_state()
    crane_plus_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_plus,
        crane_plus_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )
    
    # armをz軸上に動かす
    pose_goal.pose.position.z = 0.17
    crane_plus_arm.set_start_state_to_current_state()
    crane_plus_arm.set_goal_state(
        pose_stamped_msg=pose_goal, pose_link="crane_plus_link4"
    )
    plan_and_execute(
        crane_plus,
        crane_plus_arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # ----- Placing Preparation -----
    # armを現在の位置から"home"ポジションに動かす
    crane_plus_arm.set_start_state_to_current_state()
    crane_plus_arm.set_goal_state(configuration_name="home")
    plan_and_execute(
        crane_plus,
        crane_plus_arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # armが掴みに行く位置を指定して動かす
    pose_goal.pose.position.x = 0.15
    pose_goal.pose.position.y = 0.0
    pose_goal.pose.position.z = 0.06
    q = euler_to_quaternion(to_radians(0), to_radians(90), to_radians(0))
    pose_goal.pose.orientation.x = q[0]
    pose_goal.pose.orientation.y = q[1]
    pose_goal.pose.orientation.z = q[2]
    pose_goal.pose.orientation.w = q[3]
    crane_plus_arm.set_start_state_to_current_state()
    crane_plus_arm.set_goal_state(
        pose_stamped_msg=pose_goal, pose_link="crane_plus_link4"
    )
    plan_and_execute(
        crane_plus,
        crane_plus_arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # Release
    robot_state.set_joint_group_positions("gripper", [GRIPPER_OPEN])
    crane_plus_gripper.set_start_state_to_current_state()
    crane_plus_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_plus,
        crane_plus_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )

    #  Return to home and vertical pose
    # armを現在の位置から"home"ポジションに動かす
    crane_plus_arm.set_start_state_to_current_state()
    crane_plus_arm.set_goal_state(configuration_name="home")
    plan_and_execute(
        crane_plus,
        crane_plus_arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # armを現在の位置から"vertical"ポジションに動かす
    crane_plus_arm.set_start_state_to_current_state()
    crane_plus_arm.set_goal_state(configuration_name="vertical")
    plan_and_execute(
        crane_plus,
        crane_plus_arm,
        logger,
        single_plan_parameters=arm_plan_request_params,
    )

    # gripperの開閉をデフォルトの間隔に戻す
    robot_state.set_joint_group_positions("gripper", [GRIPPER_DEFAULT])
    crane_plus_gripper.set_start_state_to_current_state()
    crane_plus_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_plus,
        crane_plus_gripper,
        logger,
        single_plan_parameters=gripper_plan_request_params,
    )
    # MoveItPyの終了
    crane_plus.shutdown()

    # rclpyの終了
    rclpy.shutdown()


if __name__ == "__main__":
    main()

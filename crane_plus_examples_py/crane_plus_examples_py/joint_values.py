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

# generic ros libraries
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)
from moveit.core.kinematic_constraints import construct_joint_constraint

from crane_plus_examples_py.utils import plan_and_execute


def main(args=None):
    # ros2の初期化
    rclpy.init(args=args)

    # ロガー生成
    logger = get_logger("joint_values")

    # MoveItPy初期化
    crane_plus = MoveItPy(node_name="moveit_py")
    crane_plus_arm = crane_plus.get_planning_component("arm")
    logger.info("MoveItPy instance created")
    
    # instantiate a a RobotState instance using the current robot model
    robot_model = crane_plus.get_robot_model()
    robot_state = RobotState(robot_model)

    plan_request_params = PlanRequestParameters(
        crane_plus,
        "ompl_rrtc",
    )

    plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0
    plan_request_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0

    # armの関節のjoint1〜4を順番に45[deg]ずつ動かす
    joint_names = ["crane_plus_joint1", "crane_plus_joint2", "crane_plus_joint3", "crane_plus_joint4"]
    target_joint_value = math.radians(45)
    for joint_name in joint_names:
        joint_values = {joint_name: target_joint_value}
        robot_state.joint_positions = joint_values
        joint_constraint = construct_joint_constraint(
            robot_state=robot_state,
            joint_model_group=crane_plus.get_robot_model().get_joint_model_group("arm"),
        )
        crane_plus_arm.set_start_state_to_current_state()
        crane_plus_arm.set_goal_state(motion_plan_constraints=[joint_constraint])
        plan_and_execute(
            crane_plus,
            crane_plus_arm,
            logger,
            single_plan_parameters=plan_request_params,
        )

    # 現在の位置から"vertical"ポジションに動かす
    crane_plus_arm.set_start_state_to_current_state()
    crane_plus_arm.set_goal_state(configuration_name="vertical")
    plan_and_execute(
        crane_plus,
        crane_plus_arm,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # MoveItPyの終了
    crane_plus.shutdown()

    # rclpyの終了
    rclpy.shutdown()


if __name__ == "__main__":
    main()

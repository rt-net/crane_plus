# Copyright 2025 RT Corporation
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

from crane_plus_examples_py.utils import plan_and_execute

from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)

import rclpy
from rclpy.logging import get_logger


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger('gripper_control')

    # instantiate MoveItPy instance and get planning component
    crane_plus = MoveItPy(node_name='gripper_control')
    logger.info('MoveItPy instance created')

    # グリッパ制御用 planning component
    gripper = crane_plus.get_planning_component('gripper')

    # instantiate a RobotState instance using the current robot model
    robot_model = crane_plus.get_robot_model()

    plan_request_params = PlanRequestParameters(
        crane_plus,
        'ompl_rrtc',
    )

    # 動作速度の調整
    plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0
    plan_request_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0

    # gripperを閉じる
    gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('gripper', [math.radians(30.0)])
    gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_plus,
        gripper,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # gripperを開く
    gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('gripper', [math.radians(-30.0)])
    gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_plus,
        gripper,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # gripperを0度にする
    gripper.set_start_state_to_current_state()
    robot_state = RobotState(robot_model)
    robot_state.set_joint_group_positions('gripper', [math.radians(0.0)])
    gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_plus,
        gripper,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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

from crane_plus_examples_py.utils import plan_and_execute

import math

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)
import rclpy
from rclpy.logging import get_logger


def main(args=None):
    # ros2の初期化
    rclpy.init(args=args)

    # ロガー生成
    logger = get_logger('gripper_control')

    # MoveItPy初期化
    crane_plus = MoveItPy(node_name='moveit_py')
    crane_plus_gripper = crane_plus.get_planning_component('gripper')
    logger.info('MoveItPy instance created')

    # instantiate a RobotState instance using the current robot model
    robot_model = crane_plus.get_robot_model()
    robot_state = RobotState(robot_model)

    # planningのパラメータ設定
    plan_request_params = PlanRequestParameters(
        crane_plus,
        'ompl_rrtc',
    )

    # 速度＆加速度のスケーリングファクタを設定
    plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0
    plan_request_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0

    # グリッパーを+30[deg]の位置に動かす
    robot_state.set_joint_group_positions('gripper', [math.radians(30)])
    crane_plus_gripper.set_start_state_to_current_state()
    crane_plus_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_plus,
        crane_plus_gripper,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # グリッパーを-30[deg]の位置に動かす
    robot_state.set_joint_group_positions('gripper', [math.radians(-30)])
    crane_plus_gripper.set_start_state_to_current_state()
    crane_plus_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_plus,
        crane_plus_gripper,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # グリッパーを0[deg]の位置に動かす
    robot_state.set_joint_group_positions('gripper', [0.0])
    crane_plus_gripper.set_start_state_to_current_state()
    crane_plus_gripper.set_goal_state(robot_state=robot_state)
    plan_and_execute(
        crane_plus,
        crane_plus_gripper,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # MoveItPyの終了
    crane_plus.shutdown()

    # rclpyの終了
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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

from crane_plus_examples_py.utils import plan_and_execute

from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)

import rclpy
from rclpy.logging import get_logger


def main(args=None):
    rclpy.init(args=args)
    logger = get_logger('pose_groupstate')

    # instantiate MoveItPy instance and get planning component
    crane_plus = MoveItPy(node_name='pose_groupstate')
    logger.info('MoveItPy instance created')

    # アーム制御用 planning component
    arm = crane_plus.get_planning_component('arm_tcp')

    plan_request_params = PlanRequestParameters(
        crane_plus,
        'ompl_rrtc',
    )

    # 動作速度の調整
    plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0
    plan_request_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0

    # SRDF内に定義されている'home'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='home')
    plan_and_execute(
        crane_plus,
        arm,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # SRDF内に定義されている'vertical'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='vertical')
    plan_and_execute(
        crane_plus,
        arm,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # SRDF内に定義されている'home'の姿勢にする
    arm.set_start_state_to_current_state()
    arm.set_goal_state(configuration_name='home')
    plan_and_execute(
        crane_plus,
        arm,
        logger,
        single_plan_parameters=plan_request_params,
    )

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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
from moveit.planning import (
    MoveItPy,
    PlanRequestParameters,
)

from utils import plan_and_execute


def to_radians(deg_angle):
    return deg_angle * math.pi / 180.0


def main(args=None):
    # ros2の初期化
    rclpy.init(args=args)

    # ロガー生成
    logger = get_logger("joint_values")

    # MoveItPy初期化
    crane_plus = MoveItPy(node_name="moveit_py")
    crane_plus_arm = crane_plus.get_planning_component("arm")
    logger.info("MoveItPy instance created")

    # 速度＆加速度のスケーリングファクタを設定
    plan_request_params = PlanRequestParameters(
        crane_plus,
        "ompl_rrtc",
    )
    plan_request_params.max_acceleration_scaling_factor(1.0)  # Set 0.0 ~ 1.0
    plan_request_params.max_velocity_scaling_factor(1.0)  # Set 0.0 ~ 1.0

    # 現在の位置から"vertical"ポジションに動かす
    crane_plus_arm.set_start_state_to_current_state()
    crane_plus_arm.set_goal_state(configuration_name="vertical")
    plan_and_execute(
        crane_plus,
        crane_plus_arm,
        logger,
        single_plan_parameters=plan_request_params,
    )
    
    # 現在の位置からarmの関節を全て順番に45度動かす
    joint_values = crane_plus_arm.get_named_target_state_values("arm")
    target_joint_value = math.radians(45.0)
    
    # joint_valuesはDict型なので、keyの数（=jointの数）だけ関節を動かす
    for key in enumerate(joint_values.keys()):
        joint_values[key] = target_joint_value
        crane_plus_arm.set_start_state_to_current_state()
        crane_plus_arm.set_goal_state(configuration_name="vertical")
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


if __name__ == "__main__":
    main()

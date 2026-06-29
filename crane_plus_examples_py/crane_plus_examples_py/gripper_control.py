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


class GripperController:
    def __init__(self):
        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.crane_plus = MoveItPy(node_name='gripper_control')
        self.gripper = self.crane_plus.get_planning_component('gripper')
        self.robot_model = self.crane_plus.get_robot_model()

        # プランニングパラメータの設定
        self.plan_request_params = PlanRequestParameters(self.crane_plus, 'ompl_rrtc')
        self.plan_request_params.max_velocity_scaling_factor = 1.0      # Set 0.0 ~ 1.0
        self.plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0

        self.logger = get_logger('gripper_control')

    def set_gripper_angle(self, angle):
        # グリッパの目標角度を設定して動作計画・実行する
        self.gripper.set_start_state_to_current_state()
        robot_state = RobotState(self.robot_model)
        robot_state.set_joint_group_positions('gripper', [angle])
        self.gripper.set_goal_state(robot_state=robot_state)
        plan_and_execute(
            self.crane_plus,
            self.gripper,
            self.logger,
            single_plan_parameters=self.plan_request_params,
        )


def main(args=None):
    rclpy.init(args=args)

    controller = GripperController()

    # グリッパを閉じる
    controller.set_gripper_angle(math.radians(30.0))

    # グリッパを開く
    controller.set_gripper_angle(math.radians(-30.0))

    # グリッパを0度にする
    controller.set_gripper_angle(math.radians(0.0))

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()

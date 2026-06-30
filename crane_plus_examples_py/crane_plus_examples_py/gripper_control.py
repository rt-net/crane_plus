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


class GripperControl:
    # グリッパの開閉角度
    GRIPPER_DEFAULT = 0.0
    GRIPPER_OPEN = math.radians(-30.0)
    GRIPPER_CLOSE = math.radians(30.0)

    def __init__(self):
        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.crane_plus = MoveItPy(node_name='gripper_control')
        self.gripper = self.crane_plus.get_planning_component('gripper')
        self.robot_model = self.crane_plus.get_robot_model()

        # プランニングの設定（動作プランナーと速度・加速度スケール）
        self.plan_request_params = PlanRequestParameters(self.crane_plus, 'ompl_rrtc')
        self.plan_request_params.max_velocity_scaling_factor = 1.0  # Set 0.0 ~ 1.0
        self.plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0

        self.logger = get_logger('gripper_control')

    def move_gripper_angle(self, angle):
        # グリッパを角度[rad]を指定して開閉する
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

    controller = GripperControl()

    # グリッパを閉じる
    controller.move_gripper_angle(GripperControl.GRIPPER_CLOSE)

    # グリッパを開く
    controller.move_gripper_angle(GripperControl.GRIPPER_OPEN)

    # グリッパを0度にする
    controller.move_gripper_angle(GripperControl.GRIPPER_DEFAULT)

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()

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


class PoseGroupstateController:
    def __init__(self):
        # MoveItPyのインスタンスを生成し、planning componentを取得
        self.crane_plus = MoveItPy(node_name='pose_groupstate')
        self.arm = self.crane_plus.get_planning_component('arm_tcp')

        # プランニングパラメータの設定
        self.plan_request_params = PlanRequestParameters(self.crane_plus, 'ompl_rrtc')
        self.plan_request_params.max_velocity_scaling_factor = 1.0      # Set 0.0 ~ 1.0
        self.plan_request_params.max_acceleration_scaling_factor = 1.0  # Set 0.0 ~ 1.0

        self.logger = get_logger('pose_groupstate')

    def move_arm_to_named_pose(self, name):
        # SRDFに定義されている名前付きの姿勢に移動する
        self.arm.set_start_state_to_current_state()
        self.arm.set_goal_state(configuration_name=name)
        plan_and_execute(
            self.crane_plus,
            self.arm,
            self.logger,
            single_plan_parameters=self.plan_request_params,
        )


def main(args=None):
    rclpy.init(args=args)

    controller = PoseGroupstateController()

    # SRDFに定義されている名前付き姿勢を順番に実行する
    controller.move_arm_to_named_pose('home')
    controller.move_arm_to_named_pose('vertical')
    controller.move_arm_to_named_pose('home')

    # Finish with error. Related Issue
    # https://github.com/moveit/moveit2/issues/2693
    rclpy.shutdown()


if __name__ == '__main__':
    main()

# Copyright 2022 RT Corporation
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

import os

from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command


class RobotDescriptionLoader():

    def __init__(self):
        self.robot_description_path = os.path.join(
            get_package_share_directory('crane_plus_description'),
            'urdf',
            'crane_plus.urdf.xacro')
        self.port_name = '/dev/ttyUSB0'
        self.use_gazebo = 'false'
        self.use_ignition = 'false'

    def load(self):
        return Command([
                'xacro ',
                self.robot_description_path,
                ' port_name:=', self.port_name,
                ' use_gazebo:=', self.use_gazebo,
                ' use_ignition:=', self.use_ignition
                ])

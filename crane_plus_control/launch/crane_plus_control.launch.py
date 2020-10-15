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

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    xacro_file = os.path.join(get_package_share_directory('crane_plus_description'),
                              'urdf', 'crane_plus.urdf.xacro')
    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toprettyxml(indent='  ')
    robot_description = {'robot_description': robot_description_config}

    crane_plus_control_node = Node(
        package='crane_plus_control',
        executable='crane_plus_control_node',
        output='screen',
        parameters=[{'controller_name': 'crane_plus_arm_controller'},
                    os.path.join(get_package_share_directory('crane_plus_moveit_config'),
                                 'config', 'crane_plus_controllers.yaml'),
                    os.path.join(get_package_share_directory('crane_plus_moveit_config'),
                                 'config', 'start_positions.yaml'),
                    robot_description])

    return LaunchDescription([crane_plus_control_node])

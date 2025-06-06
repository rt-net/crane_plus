# Copyright 2024 RT Corporation
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
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter


def generate_launch_description():
    world_file = os.path.join(
        get_package_share_directory('crane_plus_gazebo'),
        'worlds',
        'table_with_red_cube.sdf',
    )
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                get_package_share_directory('crane_plus_gazebo'),
                '/launch/crane_plus_with_table.launch.py',
            ]
        ),
        launch_arguments={'world_name': world_file}.items(),
    )

    return LaunchDescription([SetParameter(name='use_sim_time', value=True), world_launch])

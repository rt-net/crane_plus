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

from ament_index_python.packages import get_package_share_directory
from crane_plus_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    declare_port_name = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyUSB0',
        description='Set port name.'
    )

    declare_use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='false',
        description='Use camera.'
    )

    description_loader = RobotDescriptionLoader()
    description_loader.port_name = LaunchConfiguration('port_name')
    description_loader.use_camera = LaunchConfiguration('use_camera')
    description = description_loader.load()

    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('crane_plus_moveit_config'),
                '/launch/run_move_group.launch.py']),
            launch_arguments={'loaded_description': description}.items()
        )

    control_node = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('crane_plus_control'),
                '/launch/crane_plus_control.launch.py']),
            launch_arguments={'loaded_description': description}.items()
        )

    usb_cam_node = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            parameters=[
                {'video_device': '/dev/video2'},
                {'frame_id': 'camera_color_optical_frame'}
            ],
            condition=IfCondition(LaunchConfiguration('use_camera'))
        )

    return LaunchDescription([
        declare_port_name,
        declare_use_camera,
        move_group,
        control_node,
        usb_cam_node
    ])

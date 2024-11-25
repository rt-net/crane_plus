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
from crane_plus_description.robot_description_loader import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import ExecuteProcess
from launch.actions import IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.conditions import UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    declare_use_camera = DeclareLaunchArgument(
        'use_camera',
        default_value='false',
        description='Use camera.'
        )

    # PATHを追加で通さないとSTLファイルが読み込まれない
    env = {'IGN_GAZEBO_SYSTEM_PLUGIN_PATH': os.environ['LD_LIBRARY_PATH'],
           'IGN_GAZEBO_RESOURCE_PATH': os.path.dirname(
               get_package_share_directory('crane_plus_description'))}
    world_file = os.path.join(
        get_package_share_directory('crane_plus_gazebo'), 'worlds', 'table.sdf')
    gui_config = os.path.join(
        get_package_share_directory('crane_plus_gazebo'), 'gui', 'gui.config')
    # -r オプションで起動時にシミュレーションをスタートしないと、コントローラが起動しない
    ign_gazebo = ExecuteProcess(
            cmd=['ign gazebo -r', world_file, '--gui-config', gui_config],
            output='screen',
            additional_env=env,
            shell=True
        )

    gazebo_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', '/robot_description',
                   '-name', 'crane_plus',
                   '-z', '1.015',
                   '-allow_renaming', 'true'],
    )

    description_loader = RobotDescriptionLoader()
    description_loader.use_camera = LaunchConfiguration('use_camera')
    description_loader.use_gazebo = 'true'
    description_loader.use_ignition = 'true'
    description_loader.gz_control_config_package = 'crane_plus_control'
    description_loader.gz_control_config_file_path = 'config/crane_plus_controllers.yaml'
    description = description_loader.load()

    move_group = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('crane_plus_moveit_config'),
                '/launch/run_move_group.launch.py']),
            condition=UnlessCondition(LaunchConfiguration('use_camera')),
            launch_arguments={'loaded_description': description}.items()
        )

    rviz_config_file = get_package_share_directory(
        'crane_plus_examples') + '/launch/camera_example.rviz'
    move_group_camera = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                get_package_share_directory('crane_plus_moveit_config'),
                '/launch/run_move_group.launch.py']),
            condition=IfCondition(LaunchConfiguration('use_camera')),
            launch_arguments={
                'loaded_description': description,
                'rviz_config_file': rviz_config_file
            }.items()
        )

    spawn_joint_state_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner joint_state_broadcaster'],
                shell=True,
                output='screen',
            )

    spawn_arm_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner crane_plus_arm_controller'],
                shell=True,
                output='screen',
            )

    spawn_gripper_controller = ExecuteProcess(
                cmd=['ros2 run controller_manager spawner crane_plus_gripper_controller'],
                shell=True,
                output='screen',
            )

    bridge = Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock',
                           'image_raw@sensor_msgs/msg/Image[ignition.msgs.Image',
                           'camera_info@sensor_msgs/msg/CameraInfo[ignition.msgs.CameraInfo'],
                output='screen'
            )

    return LaunchDescription([
        SetParameter(name='use_sim_time', value=True),
        declare_use_camera,
        ign_gazebo,
        gazebo_spawn_entity,
        move_group,
        move_group_camera,
        spawn_joint_state_controller,
        spawn_arm_controller,
        spawn_gripper_controller,
        bridge
    ])

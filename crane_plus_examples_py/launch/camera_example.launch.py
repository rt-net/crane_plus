# Copyright 2023 RT Corporation
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
from crane_plus_description.robot_description_loader \
    import RobotDescriptionLoader
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ld = LaunchDescription()
    description_loader = RobotDescriptionLoader()
    ld.add_action(
        DeclareLaunchArgument(
            'loaded_description',
            default_value=description_loader.load(),
            description='Set robot_description text.  \
                      It is recommended to use RobotDescriptionLoader() \
                          in crane_plus_description.',
        )
    )

    moveit_config = (
        MoveItConfigsBuilder('crane_plus')
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory('crane_plus_description'),
                'urdf',
                'crane_plus.urdf.xacro',
            ),
            mappings={},
        )
        .robot_description_kinematics(
            file_path=get_package_share_directory('crane_plus_moveit_config')
            + '/config/kinematics.yaml'
        )
        # .robot_description_semantic(
        #     file_path='config/crsane_plus.srdf',
        #     mappings={'model': 'crane_plus'},
        # )
        # .joint_limits(file_path='config/joint_limits.yaml')
        .trajectory_execution(
            file_path=get_package_share_directory('crane_plus_moveit_config')
            + '/config/controllers.yaml'
        )
        # .planning_pipelines(pipelines=['ompl'])
        .moveit_cpp(
            file_path=get_package_share_directory('crane_plus_examples_py')
            + '/config/crane_plus_moveit_py_examples.yaml'
        )
        .to_moveit_configs()
    )

    moveit_config.robot_description = {
        'robot_description': LaunchConfiguration('loaded_description')
    }

    moveit_config.move_group_capabilities = {'capabilities': ''}

    declare_example_name = DeclareLaunchArgument(
        'example', default_value='color_detection',
        description=('Set an example executable name: '
                     '[aruco_detection, color_detection]')
    )

    picking_node = Node(
        name='pick_and_place_tf',
        package='crane_plus_examples_py',
        executable='pick_and_place_tf',
        output='screen',
        parameters=[
            # moveit_config.robot_description,
            # moveit_config.robot_description_semantic,
            # moveit_config.robot_description_kinematics,
            moveit_config.to_dict()
        ],
    )

    example_node = Node(
        name=[LaunchConfiguration('example'), '_node'],
        package='crane_plus_examples_py',
        executable=LaunchConfiguration('example'),
        output='screen',
        parameters=[moveit_config.to_dict()],
    )

    # ld = LaunchDescription()
    ld.add_action(example_node)
    ld.add_action(picking_node)
    ld.add_action(declare_example_name)

    return ld

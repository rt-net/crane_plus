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
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.actions import SetParameter
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    ld = LaunchDescription()
    declare_loaded_description = DeclareLaunchArgument(
        "loaded_description",
        default_value="",
        description="Set robot_description text.  \
                     It is recommended to use RobotDescriptionLoader() in crane_plus_description.",
    )

    moveit_config = (
        MoveItConfigsBuilder("crane_plus")
        .planning_scene_monitor(
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("crane_plus_description"),
                "urdf",
                "crane_plus.urdf.xacro",
            ),
            mappings={},
        )
        .robot_description_semantic(
            file_path="config/crane_plus.srdf",
            mappings={"model": "crane_plus"},
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        .trajectory_execution(
            file_path="config/controllers.yaml", moveit_manage_controllers=True
        )
        .planning_pipelines(pipelines=["ompl"])
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .moveit_cpp(
            file_path=get_package_share_directory("crane_plus_examples_py")
            + "/config/crane_plus_moveit_py_examples.yaml"
        )
        .to_moveit_configs()
    )

    moveit_config.robot_description = {
        "robot_description": LaunchConfiguration("loaded_description")
    }

    moveit_config.move_group_capabilities = {"capabilities": ""}

    declare_example_name = DeclareLaunchArgument(
        "example",
        default_value="gripper_control",
        description=(
            "Set an example executable name: "
            "[gripper_control, pose_groupstate, joint_values, pick_and_place]"
        ),
    )

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description=("Set true when using the gazebo simulator."),
    )

    use_sim_time_name = SetParameter(
        name="use_sim_time", value=LaunchConfiguration("use_sim_time")
    )

    example_node = Node(
        name=[LaunchConfiguration("example"), "_node"],
        package="crane_plus_examples",
        executable=LaunchConfiguration("example"),
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    ld.add_action(declare_loaded_description)
    ld.add_action(declare_example_name)
    ld.add_action(declare_use_sim_time)
    ld.add_action(use_sim_time_name)
    ld.add_action(example_node)

    return ld

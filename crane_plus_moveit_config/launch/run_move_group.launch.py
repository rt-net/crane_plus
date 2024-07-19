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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml
from moveit_configs_utils import MoveItConfigsBuilder

from moveit_configs_utils.launches import generate_move_group_launch
from moveit_configs_utils.launches import generate_moveit_rviz_launch

from moveit_configs_utils.launches import generate_static_virtual_joint_tfs_launch

from moveit_configs_utils.launches import generate_rsp_launch

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


# Reference: https://github.com/ros-planning/moveit2/blob/main/moveit_demo_nodes/
# run_move_group/launch/run_move_group.launch.py


def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except (
        EnvironmentError
    ):  # parent of IOError, OSError *and* WindowsError where available
        return None


def generate_launch_description():
    ld = LaunchDescription()

    declare_loaded_description = DeclareLaunchArgument(
        "loaded_description",
        default_value="",
        description="Set robot_description text.  \
                     It is recommended to use RobotDescriptionLoader() in crane_plus_description.",
    )

    ld.add_action(declare_loaded_description)

    # MoveItConfigBuilderによるパラメータ設定
    moveit_config = (
        # ロボット名の定義
        MoveItConfigsBuilder("crane_plus")
        # URDFの設定
        .robot_description(
            file_path=os.path.join(
                get_package_share_directory("crane_plus_description"),
                "urdf",
                "crane_plus.urdf.xacro",
            ),
            mappings={},
        )
        # SRDFの設定
        .robot_description_semantic(
            file_path="config/crane_plus.srdf",
            mappings={"model": "crane_plus"},
        )
        .joint_limits(file_path="config/joint_limits.yaml")
        # Planning Sceneのトピックの設定
        .planning_scene_monitor(
            publish_planning_scene=True,
            publish_geometry_updates=True,
            publish_state_updates=True,
            publish_transforms_updates=True,
            publish_robot_description=True,
            publish_robot_description_semantic=True,
        )
        # 軌道追従制御ノードの設定
        .trajectory_execution(
            file_path="config/controllers.yaml",
            moveit_manage_controllers=True
        )
        # 軌道計画のプラグイン設定
        .planning_pipelines(pipelines=["ompl"], default_planning_pipeline="ompl")
        # キネマティクスの設定
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    moveit_config.robot_description = {
        "robot_description": LaunchConfiguration("loaded_description")
    }

    moveit_config.move_group_capabilities = {"capabilities": ""}

    # Move group
    ld.add_entity(generate_move_group_launch(moveit_config))

    # RViz
    ld.add_entity(generate_moveit_rviz_launch(moveit_config))
    ld.add_action(DeclareBooleanLaunchArg("debug", default_value=False))
    ld.add_action(
        DeclareLaunchArgument(
            "rviz_config",
            default_value=get_package_share_directory("crane_plus_moveit_config")
            + "/config/moveit.rviz",
            description="Set the path to rviz configuration file.",
        )
    )
 
    # Static TF
    ld.add_action(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="static_transform_publisher",
            output="log",
            arguments=["0.0", "0.0", "0.0", "0.0", "0.0", "0.0", "world", "base_link"],
        )
    )

    # Publish TF
    ld.add_entity(generate_rsp_launch(moveit_config))

    return ld
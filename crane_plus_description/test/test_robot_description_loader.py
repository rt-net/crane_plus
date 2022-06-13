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

from crane_plus_description.robot_description_loader import RobotDescriptionLoader
from launch.launch_context import LaunchContext
import pytest


def exec_load(loader):
    # Command substitutionの実行方法はCommandのテストを参考にした
    # https://github.com/ros2/launch/blob/074cd2903ddccd61bce8f40a0f58da0b7c200481/launch/test/launch/substitutions/test_command.py#L47
    context = LaunchContext()
    return loader.load().perform(context)


def test_load_description():
    # xacroの読み込みが成功することを期待
    rdl = RobotDescriptionLoader()
    assert exec_load(rdl)


def test_change_description_path():
    # xacroのファイルパスを変更し、読み込みが失敗することを期待
    rdl = RobotDescriptionLoader()
    rdl.robot_description_path = 'hoge'
    with pytest.raises(Exception) as e:
        exec_load(rdl)
    assert e.value


def test_port_name():
    # port_nameが変更され、xacroにポート名がセットされることを期待
    rdl = RobotDescriptionLoader()
    rdl.port_name = '/dev/ttyUSB1'
    assert '/dev/ttyUSB1' in exec_load(rdl)


def test_use_gazebo():
    # use_gazeboが変更され、xacroにgazebo_ros2_controlがセットされることを期待
    rdl = RobotDescriptionLoader()
    rdl.use_gazebo = 'true'
    assert 'gazebo_ros2_control/GazeboSystem' in exec_load(rdl)


def test_use_ignition():
    # use_gazeboとuse_ignitionが変更され、xacroにign_ros2_controlがセットされることを期待
    rdl = RobotDescriptionLoader()
    rdl.use_gazebo = 'true'
    rdl.use_ignition = 'true'
    assert 'ign_ros2_control/IgnitionSystem' in exec_load(rdl)

# crane_plus_examples

このパッケージはCRANE+V2 ROS 2パッケージのサンプルコード集です。

## 実行手順

### 1. CRANE+V2本体をPCに接続する

CRANE+V2本体をPCに接続します。
接続方法は製品マニュアルを参照してください。

### 2. USBポートの接続を確認する

USBポートの接続確認や設定については`crane_plus_control`のREADMEを参照してください。

特に`latency_timer`が変更されていない場合は、
CRANE+V2の制御周期が小さくなるので注意してください。

```sh
# latency_timerの変更方法

# rootに切り替える
$ sudo sh

# --- 以下 rootで実行するコマンド ---
# echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# exit
```

### 3. move_groupとcontrollerを起動する

次のコマンドでmove_group (`crane_plus_moveit_config`)と
controller (`crane_plus_control`)を起動します。

```sh
$ ros2 launch crane_plus_examples demo.launch.py
```

### 4. サンプルプログラムを実行する

サンプルプログラムを実行します。
例えばグリッパを開閉するサンプルは次のコマンドで実行できます。

```sh
# Example
$ ros2 launch crane_plus_examples example.launch.py name:='gripper_control'
```

## Examples

`demo.launch.py`を実行している状態で各サンプルを実行できます。

- [gripper_control](#gripper_control)
- [pose_groupstate](#pose_groupstate)

---

### gripper_control

グリッパを開閉させるコード例です。

**注意：`gripper_action_controller`が`ros2_controllers`に移植されていないため、
代わりに`joint_trajectory_controller`でグリッパを操作しています**

次のコマンドを実行します。

```sh
$ ros2 launch crane_plus_examples example.launch.py name:='gripper_control'
```

<img src=https://rt-net.github.io/images/crane-plus/gripper_control.gif width=500px />

[back to example list](#examples)

---

### pose_groupstate

group_stateを使うコード例です。

SRDFファイル[crane_plus_moveit_config/config/crane_plus.srdf](../crane_plus_moveit_config/config/crane_plus.srdf)
に記載されている`home`と`vertical`の姿勢に移行します。

次のコマンドを実行します。

```sh
$ ros2 launch crane_plus_examples example.launch.py name:='pose_groupstate'
```

<img src=https://rt-net.github.io/images/crane-plus/pose_groupstate.gif width=500px />

[back to example list](#examples)

---

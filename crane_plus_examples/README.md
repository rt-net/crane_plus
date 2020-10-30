# crane_plus_examples

このパッケージはCRANE+V2 ROS 2パッケージのサンプルコード集です。

## 実行手順

### 1. CRANE+V2本体をPCに接続する

CRANE+V2本体をPCに接続します。
接続方法は製品マニュアルを参照してください。

### 2. USB通信ポートの接続を確認する

USB通信ポートの設定については`crane_plus_control`のREADMEを参照してください。

特に`latency_timer`が変更されていない場合は、
`crane_plus_control`の制御周期が遅くなるので注意してください。

```sh
# /dev/ttyUSB0を使用する場合

# rootに切り替える
$ sudo su

--- 以下rootで実行 ---
# echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
1
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
$ ros2 launch crane_plus_examples example.launch.py example:='gripper_control'
```

## Examples

`demo.launch.py`を実行している状態で各サンプルを実行できます。

- [gripper_control](#gripper_control)
- [pose_groupstate](#pose_groupstate)
- [joint_values](#joint_values)
- [pick_and_place](#pick_and_place)

実行できるサンプルの一覧は、`examples.launch.py`にオプション`-s`を付けて実行することで表示できます。

```sh
$ ros2 launch crane_plus_examples example.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

    'example':
        Set an example executable name: [gripper_control, pose_groupstate, joint_values, pick_and_place]
        (default: 'gripper_control')
```

---

### gripper_control

グリッパを開閉させるコード例です。

**注意：`gripper_action_controller`が`ros2_controllers`に移植されていないため、
代わりに`joint_trajectory_controller`でグリッパを操作しています**

次のコマンドを実行します。

```sh
$ ros2 launch crane_plus_examples example.launch.py example:='gripper_control'
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
$ ros2 launch crane_plus_examples example.launch.py example:='pose_groupstate'
```

<img src=https://rt-net.github.io/images/crane-plus/pose_groupstate.gif width=500px />

[back to example list](#examples)

---

### joint_values 

アームのジョイント角度を１つずつ変更するコード例です。

次のコマンドを実行します。

```sh
$ ros2 launch crane_plus_examples example.launch.py example:='joint_values'
```

<img src=https://rt-net.github.io/images/crane-plus/joint_values.gif width=500px />

[back to example list](#examples)

---

### pick_and_place

モノを掴む・持ち上げる・運ぶ・置くコード例です。

次のコマンドを実行します。

```sh
$ ros2 launch crane_plus_examples example.launch.py example:='pick_and_place'
```

<img src=https://rt-net.github.io/images/crane-plus/pick_and_place.gif width=500px />

[back to example list](#examples)

---

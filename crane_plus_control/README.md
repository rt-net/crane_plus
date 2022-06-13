# crane_plus_control

このパッケージは[ros2_control](https://github.com/ros-controls/ros2_control)
をベースにした、CRANE+V2 のコントローラパッケージです。

## ros2_control関連ファイル

- `crane_plus_control::CranePlusHardware (crane_plus_hardware)`
  - 本パッケージがエクスポートする[Hardware Components](https://ros-controls.github.io/control.ros.org/getting_started.html#hardware-components)です
  - CRANE+V2実機と通信します
  - [crane_plus_description/urdf/crane_plus.ros2_control.xacro](../crane_plus_description/urdf/crane_plus.ros2_control.xacro)から読み込まれます
- [launch/crane_plus_control.launch.py](./launch/crane_plus_control.launch.py)
  - [Controller Manager](https://ros-controls.github.io/control.ros.org/getting_started.html#controller-manager)とコントローラを起動するlaunchファイルです
- [config/crane_plus_controllers.yaml](./config/crane_plus_controllers.yaml)
  - Controller Managerのパラメータファイルです

## 実機のセットアップ

`crane_plus_hardware`がCRANE+V2実機と通信するために、
PCとCRANE+V2の設定が必要です。

**正しく設定できていない場合、CRANE+V2が動作しない、振動する、などの不安定な動きをするため注意してください**

### USB通信ポートの設定

`crane_plus_hardware`はUSB通信ポート（`/dev/ttyUSB*`）を経由してCRANE+V2と通信します。

次のコマンドでアクセス権限を変更します。

```sh
# /dev/ttyUSB0を使用する場合
$ sudo chmod 666 /dev/ttyUSB0
```

### latency_timerの設定

CRANE+V2を100 Hz周期で制御するためには、
USB通信ポートとサーボモータの設定を変更します。

下記のコマンドを実行してUSB通信ポートの`latency_timer`を変更します。

参考資料：https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/#usb-latency-setting

```sh
# /dev/ttyUSB0を使用する場合

# rootに切り替える
$ sudo su
```

```txt
# echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
# cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer
1
# exit
```

### Return Delay Timeの設定

CRANE+V2に搭載されているサーボモータ[Dynamixel AX-12A](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/)
には`Return Delay Time`というパラメータがあります。

デフォルトは250がセットされており、
サーボモータが`Instruction Packet`を受信してから`Status Packet`を送信するまでに`500 usec`の遅れがあります。

[Dynamixel Wizard 2](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
を使用して`Retrun Delay Time`を小さくすると、制御周期が早くなります。

![Setting Return Delay Time](https://rt-net.github.io/images/crane-plus/setting_return_delay_time.png)

## ノードの起動

`crane_plus_control.launch.py`を実行すると、`Controller Manager`ノードが起動し、
以下のコントローラが読み込まれます。

- crane_plus_joint_state_controller (`joint_state_controller/JointStateController`)
- crane_plus_arm_controller (`joint_trajectory_controller/JointTrajectoryController`)
- crane_plus_gripper_controller (`joint_trajectory_controller/JointTrajectoryController`)

ノードが起動した後、
次のコマンドでジョイント角度情報（`joint_states`）を表示できます

```sh
$ ros2 topic echo /joint_states
```

## Controller Managerのパラメータ

`Controller Manager`のパラメータは
[config/crane_plus_controllers.yaml](./config/crane_plus_controllers.yaml)
で設定しています。

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    crane_plus_arm_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    crane_plus_gripper_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    joint_state_controller:
      type: joint_state_controller/JointStateController
```

### 制御周期

`update_rate`は制御周期を設定します。

CRANE+V2に使用しているサーボモータの仕様により、
100 Hz以上の周期で制御できません。

### コントローラ

CRANE+V2の腕の制御用に`crane_plus_arm_controller`を、
グリッパの制御用に`crane_plus_gripper_controller`を設定しています。

## crane_plus_hardwareのパラメータ

`crane_plus_hardware`のパラメータは
[crane_plus_description/urdf/crane_plus.urdf.xacro](../crane_plus_description/urdf/crane_plus.urdf.xacro)
で設定しています。

```xml
<xacro:arg name="use_gazebo" default="false" />
<xacro:arg name="port_name" default="/dev/ttyUSB0" />
<xacro:arg name="baudrate" default="1000000" />
<xacro:arg name="timeout_seconds" default="5.0" />
<xacro:arg name="read_velocities" default="0" />
<xacro:arg name="read_loads" default="0" />
<xacro:arg name="read_voltages" default="0" />
<xacro:arg name="read_temperatures" default="0" />
```

### USB通信ポート

`port_name`はCRANE+V2との通信に使用するUSB通信ポートを設定します。

### ボーレート

`baudrate`はCRANE+V2に搭載したDynamixelとの通信ボーレートを設定します。

デフォルト値にはDynamixel AX-12Aの最高ボーレートである`1000000` (1 Mbps)を設定しています。

### 通信タイムアウト

`timeout_seconds`は通信タイムアウト時間（秒）を設定します。

`crane_plus_hardware`は、一定時間（デフォルト5秒間）通信に失敗し続けると、
read/write動作を停止します。
USBケーブルや電源ケーブルが抜けた場合等に有効です。

### サーボパラメータ

`read_velocities`、`read_loads`、`read_voltages`、`read_temperatures`
は、サーボの回転速度、電圧、負荷、温度を読み取るためのパラメータです。

`1`をセットすると、サーボパラメータを読み取ります。

これらのパラメータを読み取ると通信データ量が増加するため、制御周期が100 Hzより低下します。

読み取ったパラメータは`dynamic_joint_states`トピックとしてパブリッシュされます。

```sh
$ ros2 topic echo /dynamic_joint_states
```

---

[back to top](#crane_plus_control)

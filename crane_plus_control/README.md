# crane_plus_control

このパッケージはros2_controlをベースにした、CRANE+V2 のコントローラパッケージです。

## 実機のセットアップ

`crane_plus_control`はCRANE+V2実機と通信するノードのため、
PCとCRANE+V2の設定が必要です。

### USB通信ポートの設定

`crane_plus_control`はUSB通信ポート（`/dev/ttyUSB*`）を経由してCRANE+V2と通信します。

次のコマンドでアクセス権限を変更します。

```sh
# /dev/ttyUSB0を使用する場合
$ sudo chmod 666 /dev/ttyUSB0
```

### USB通信ポートの変更

`/dev/ttyUBS0`以外を使用する場合は
[crane_plus_control/config/crane_plus_controllers.yaml](./config/crane_plus_controllers.yaml)
の`port_name`を変更します。

```yaml
control_param_node:
  ros__parameters:
    port_name: /dev/ttyUSB0
    baudrate: 1000000
```

### latency_timerの設定

`crane_plus_control`は100 Hz周期で制御するように設定されていますが、
USB通信ポートとサーボモータの設定を変更しなければ100 Hzで制御できません。

下記のコマンドを実行してUSB通信ポートの`latency_timer`を変更します。

参考資料：https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/#usb-latency-setting

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

### Return Delay Timeの設定

CRANE+V2に搭載されているサーボモータ[Dynamixel AX-12A](https://emanual.robotis.com/docs/en/dxl/ax/ax-12a/)
には`Return Delay Time`というパラメータがあります。

デフォルトは250がセットされており、
DynamixelがInstruction Packetを受信してからStatus Packetを送信するまでに`500 usec`の遅れがあります。

[Dynamixel Wizard 2](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
を使用して`Retrun Delay Time`を小さくすると、制御周期が早くなります。

![Setting Return Delay Time](https://rt-net.github.io/images/crane-plus/setting_return_delay_time.png)

## ノードの起動

下記のコマンドで`crane_plus_control_node`が起動します。

```sh
$ ros2 launch crane_plus_control crane_plus_control.launch.py 
```

ノードが起動すると以下のコントローラが読み込まれます。

- crane_plus_joint_state_controller (`joint_state_controller/JointStateController`)
- crane_plus_arm_controller (`joint_trajectory_controller/JointTrajectoryController`)
- crane_plus_gripper_controller (`joint_trajectory_controller/JointTrajectoryController`)
  - **`gripper_action_controller`が`ros2_controllers`に移植されたら変更します**

ノードが起動した後、
次のコマンドでジョイント角度情報（`joint_state`）を表示できます

```sh
$ ros2 topic echo /joint_state
```


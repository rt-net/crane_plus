# crane_plus_examples

このパッケージはCRANE+ V2 ROS 2パッケージのサンプルコード集です。

## 準備（実機を使う場合）

![crane_plus](https://rt-net.github.io/images/crane-plus/CRANEV2-500x500.png)

### 1. CRANE+ V2本体をPCに接続する

CRANE+ V2本体をPCに接続します。
接続方法は製品マニュアルを参照してください。

**※CRANE+ V2本体が接触しないように、十分なスペースを確保してください。**

### 2. USB通信ポートの接続を確認する

USB通信ポートの設定については`crane_plus_control`の
[README](../crane_plus_control/README.md)
を参照してください。

**正しく設定できていない場合、CRANE+ V2が動作しない、振動する、などの不安定な動きになるので注意してください**

### 3. move_groupとcontrollerを起動する

#### 標準のCRANE+ V2を使用する場合

次のコマンドでmove_group (`crane_plus_moveit_config`)と
controller (`crane_plus_control`)を起動します。

```sh
$ ros2 launch crane_plus_examples demo.launch.py port_name:=/dev/ttyUSB0
```

#### Webカメラ搭載モデルを使用する場合

Webカメラ搭載モデルの場合は、次のコマンドを実行してください。
```video_device```は使用するWebカメラを指定してください。

```sh
$ ros2 launch crane_plus_examples demo.launch.py port_name:=/dev/ttyUSB0 use_camera:=true video_device:=/dev/video0
```

## 準備（Gazeboを使う場合）
=======
![crane_plus_ignition](https://rt-net.github.io/images/crane-plus/crane_plus_ignition.png)

### 1. move_groupとGazeboを起動する

次のコマンドでmove_group (`crane_plus_moveit_config`)とGazeboを起動します。

```sh
$ ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py
```

#### Webカメラ搭載モデルを使用する場合

Webカメラ搭載モデルの場合は、次のコマンドを実行してください。

```sh
$ ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py use_camera:=true
```

CRANE+ V2の前にArUcoマーカ付きのBoxを置いたシミュレータ環境を使用する場合は次のコマンドを実行します。
[aruco\_detection](#aruco_detection)サンプルを実行する際に使用することを想定しています。

```sh
$ ros2 launch crane_plus_gazebo crane_plus_with_aruco_cube.launch.py use_camera:=true
```

CRANE+ V2の前に赤いBoxを置いたシミュレータ環境を使用する場合は次のコマンドを実行します。
[color\_detection](#color_detection)サンプルを実行する際に使用すること想定しています。

```sh
$ ros2 launch crane_plus_gazebo crane_plus_with_red_cube.launch.py use_camera:=true
```

## 準備（Mock Componentsを使う場合）

### 1. move_groupとcontrollerを起動する

次のコマンドでmove_group (`crane_plus_moveit_config`)と
controller (`crane_plus_control`)を起動します。

```sh
$ ros2 launch crane_plus_examples demo.launch.py use_mock_components:=true
```

Mock Componentsではカメラを使ったサンプルを実行することはできません。

## サンプルプログラムを実行する

準備ができたらサンプルプログラムを実行します。
例えばグリッパを開閉するサンプルは次のコマンドで実行できます。

```sh
$ ros2 launch crane_plus_examples example.launch.py example:='gripper_control'
```

終了するときは`Ctrl+c`を入力します。

## Gazeboでサンプルプログラムを実行する場合

Gazeboでサンプルプログラムを実行する場合は`use_sim_time`オプションを付けます。

```sh
$ ros2 launch crane_plus_examples example.launch.py example:='gripper_control' use_sim_time:=true
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

## Camera Examples

Webカメラ搭載モデルのカメラを使用したサンプルコードです。

[「Webカメラ搭載モデルを使用する場合」](#Webカメラ搭載モデルを使用する場合)の手順に従って、
`demo.launch`を実行している状態で、
各サンプルを実行できます。

- [aruco\_detection](#aruco_detection)
- [color\_detection](#color_detection)

実行できるサンプルの一覧は、`camera_example.launch.py`にオプション`-s`を付けて実行することで確認できます。

```sh
$ ros2 launch crane_plus_examples camera_example.launch.py -s
Arguments (pass arguments as '<name>:=<value>'):

    'example':
        Set an example executable name: [color_detection]
        (default: 'color_detection')
```

---

### aruco_detection

モノに取り付けたArUcoマーカをカメラで検出し、マーカ位置に合わせて掴むコード例です。
マーカは[aruco_markers.pdf](./aruco_markers.pdf)をA4紙に印刷して、一辺50mmの立方体に取り付けて使用します。

検出されたマーカの位置姿勢はtfのフレームとして配信されます。
tfの`frame_id`はマーカIDごとに異なりID0のマーカの`frame_id`は`target_0`になります。
掴む対象は`target_0`に設定されています。
マーカ検出には[OpenCV](https://docs.opencv.org/4.x/d5/dae/tutorial_aruco_detection.html)を使用しています。

次のコマンドを実行します。

```bash
ros2 launch crane_plus_examples camera_example.launch.py example:='aruco_detection'
```

#### Videos
[![crane_plus_aruco_detection_demo](https://rt-net.github.io/images/crane-plus/aruco_detection.gif)](https://youtu.be/m9dus6LCocc)

[back to example list](#examples)

---

### color_detection

特定の色の物体を検出して掴むコード例です。

デフォルトでは赤い物体の位置をtfのフレームとして配信します。
tfの`frame_id`は`target_0`です。
色検出には[OpenCV](https://docs.opencv.org/4.x/db/d8e/tutorial_threshold.html)を使用しています。

次のコマンドを実行します。

```sh
ros2 launch crane_plus_examples camera_example.launch.py example:='color_detection'
```

#### Videos
[![crane_plus_color_detection_demo](https://rt-net.github.io/images/crane-plus/color_detection.gif)](https://youtu.be/Kn0eWA7sALY)

[back to example list](#examples)

---

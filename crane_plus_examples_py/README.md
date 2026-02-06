# crane_plus_examples_py

このパッケージはCRANE+ V2 ROS 2パッケージのPythonによるサンプルコード集です。

- [crane\_plus\_examples\_py](#crane_plus_examples_py)
  - [起動方法](#起動方法)
  - [サンプルプログラムを実行する](#サンプルプログラムを実行する)
    - [Gazeboでサンプルプログラムを実行する場合](#Gazeboでサンプルプログラムを実行する場合)
  - [Examples](#examples)
    - [gripper\_control](#gripper_control)
    - [pose\_groupstate](#pose_groupstate)
    - [joint\_values](#joint_values)
    - [pick\_and\_place](#pick_and_place)
  - [Camera Examples](#camera-examples)
    - [aruco\_detection](#aruco_detection)
    - [color\_detection](#color_detection)

## 起動方法
CRANE+ V2の起動方法は[crane_plus_examplesのREADME](../crane_plus_examples/README.md)を参照してください。

## サンプルプログラムを実行する
準備ができたらPythonによるサンプルプログラムを実行します。
例えばグリッパを開閉するサンプルは次のコマンドで実行できます。

```sh
$ ros2 launch crane_plus_examples_py example.launch.py example:='gripper_control'
```

終了するときは`Ctrl+c`を入力します。

## Gazeboでサンプルプログラムを実行する場合

Gazeboでサンプルプログラムを実行する場合は`use_sim_time`オプションを付けます。

```sh
$ ros2 launch crane_plus_examples_py example.launch.py example:='gripper_control' use_sim_time:=true
```

## Examples

`demo.launch.py`を実行している状態で各サンプルを実行できます。

- [gripper_control](#gripper_control)
- [pose_groupstate](#pose_groupstate)
- [joint_values](#joint_values)
- [pick_and_place](#pick_and_place)

実行できるサンプルの一覧は、`examples.launch.py`にオプション`-s`を付けて実行することで表示できます。

```sh
$ ros2 launch crane_plus_examples_py example.launch.py -s
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
$ ros2 launch crane_plus_examples_py example.launch.py example:='gripper_control'
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
$ ros2 launch crane_plus_examples_py example.launch.py example:='pose_groupstate'
```

<img src=https://rt-net.github.io/images/crane-plus/pose_groupstate.gif width=500px />

[back to example list](#examples)

---

### joint_values 

アームのジョイント角度を１つずつ変更するコード例です。

次のコマンドを実行します。

```sh
$ ros2 launch crane_plus_examples_py example.launch.py example:='joint_values'
```

<img src=https://rt-net.github.io/images/crane-plus/joint_values.gif width=500px />

[back to example list](#examples)

---

### pick_and_place

モノを掴む・持ち上げる・運ぶ・置くコード例です。

次のコマンドを実行します。

```sh
$ ros2 launch crane_plus_examples_py example.launch.py example:='pick_and_place'
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
$ ros2 launch crane_plus_examples_py camera_example.launch.py -s
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
ros2 launch crane_plus_examples_py camera_example.launch.py example:='aruco_detection'
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
ros2 launch crane_plus_examples_py camera_example.launch.py example:='color_detection'
```

#### Videos
[![crane_plus_color_detection_demo](https://rt-net.github.io/images/crane-plus/color_detection.gif)](https://youtu.be/Kn0eWA7sALY)

[back to example list](#examples)

---

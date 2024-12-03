# crane_plus_gazebo

CRANE+ V2 の[Gazebo](https://gazebosim.org/home)
シミュレーションパッケージです。

## ノードの起動

次のコマンドを実行するとGazeboが起動し、CRANE+ V2モデルとTable、Boxが表示されます。

初回起動時はTableとBoxのモデルをダウンロードするため、モデルの表示に時間がかかることがあります。

実機との接続や`crane_plus_examples/launch/demo.launch.py`の実行は必要ありません。

```sh
$ ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py
```

カメラ付きモデルを使用する場合は下記コマンドを実行します。

```sh
$ ros2 launch crane_plus_gazebo crane_plus_with_table.launch.py use_camera:=true
```

CRANE+ V2の前にArUcoマーカ付きのBoxを置いたシミュレータ環境を使用する場合は下記コマンドを実行します。

```sh
$ ros2 launch crane_plus_gazebo crane_plus_with_aruco_cube.launch.py
```

CRANE+ V2の前に赤いBoxを置いたシミュレータ環境を使用する場合は下記コマンドを実行します。

```sh
$ ros2 launch crane_plus_gazebo crane_plus_with_red_cube.launch.py
```

![crane_plus_ignition](https://rt-net.github.io/images/crane-plus/crane_plus_ignition.png)

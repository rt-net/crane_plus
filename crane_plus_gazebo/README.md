# crane_plus_gazebo

CRANE+V2 のGazeboシミュレーションパッケージです。

**現在、Gazebo上のCRANE+V2を動かすことは出来ません**

`crane_plus_gazebo`は
[gazebo_ros2_control](https://github.com/ros-simulation/gazebo_ros2_control)及び
[ros2_control](https://github.com/ros-controls/ros2_control)
の開発状況に依存しています。

## ノードの起動

次のコマンドを実行するとGazeboが起動し、CRANE+V2モデルと、Table、Cubeが表示がされます。

初回起動時はTableとCubeのモデルをダウンロードするため、モデルの表示に時間がかかることがあります。

```sh
$ ros2 launch crane_plus_gazebo crane_plus_gazebo.launch.py
```

![crane_plus_gazebo](https://rt-net.github.io/images/crane-plus/crane_plus_gazebo.png)

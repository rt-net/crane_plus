# crane_plus_ignition

CRANE+V2 の[Ignition Gazebo](https://gazebosim.org/home)
シミュレーションパッケージです。

## ノードの起動

次のコマンドを実行するとIgnition Gazeboが起動し、CRANE+V2モデルとTable、Boxが表示されます。

初回起動時はTableとBoxのモデルをダウンロードするため、モデルの表示に時間がかかることがあります。

実機との接続や`crane_plus_examples/launch/demo.launch/py`の実行は必要ありません。

```sh
$ ros2 launch crane_plus_ignitionw crane_plus_ignition.launch.py
```

![crane_plus_ignition](https://rt-net.github.io/images/crane-plus/crane_plus_ignition.png)

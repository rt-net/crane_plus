# crane_plus_gazebo

CRANE+V2 のGazeboシミュレーションパッケージです。

## ノードの起動

次のコマンドを実行するとGazeboが起動し、CRANE+V2モデルとTableが表示されます。

初回起動時はTableのモデルをダウンロードするため、モデルの表示に時間がかかることがあります。

実機との接続や`crane_plus_examples/launch/demo.launch/py`の実行は必要ありません。

```sh
$ ros2 launch crane_plus_gazebo crane_plus_gazebo.launch.py
```

![crane_plus_gazebo](https://rt-net.github.io/images/crane-plus/crane_plus_gazebo.png)

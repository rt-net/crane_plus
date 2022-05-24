# Dockerを使用する

## Dockerイメージを使用する

https://github.com/rt-net/crane_plus/pkgs/container/crane_plus
にDockerイメージ`ghcr.io/rt-net/crane_plus:$ROS_DISTRO`
をアップロードしています。
tagにはROSのディストリビューションを指定してください。

foxyディストリビューションのイメージをダウンロードする場合は次のコマンドを実行します。

```sh
$ docker pull ghcr.io/rt-net/crane_plus:foxy
```

### ノードの起動

GUIアプリケーションを起動するため
[osrf/rocker](https://github.com/osrf/rocker)
を使用します

rockerのオプションには、
ホストのネットワーク環境を使用するための`--net=host`と
[ネットワーク使用時のエラー](https://github.com/osrf/rocker/issues/13)
を回避するための`--privileged`を与えます

また、USBデバイスを使用するため`--volume /dev:/dev`で`/dev/`ディレクトリをコンテナにマウントします。

```sh
# rockerのインストール
$ sudo apt install python3-rocker

# NVIDIA GPUを使用する場合
$ rocker --nvidia --x11 --net=host --privileged \
    --volume /dev:/dev \
    -- ghcr.io/rt-net/crane_plus:$ROS_DISTRO \
    ros2 launch crane_plus_examples demo.launch.py
# Intelグラフィックスを使用する場合
$ rocker --devices /dev/dri/card0 --x11 --net=host --privileged \
    --volume /dev:/dev \
    -- ghcr.io/rt-net/crane_plus:$ROS_DISTRO \
    ros2 launch crane_plus_examples demo.launch.py
```

### パッケージの再ビルド

ホストに作成したワークスペースをDockerコンテナにマウントすることで、
ホスト環境でのファイル編集内容をコンテナ内に反映できます

```sh
# ワークスペース作成
$ mkdir -p ~/crane_ws/src
# パッケージをクローン
$ git clone https://github.com/rt-net/crane_plus.git ~/crane_ws/src/crane_plus

# パッケージをビルド
$ rocker --devices /dev/dri/card0 --x11 --net=host --privileged \
    --volume ~/crane_ws:/root/overlay_ws \
    -- ghcr.io/rt-net/crane_plus:$ROS_DISTRO \
    colcon build --symlink-install

# ノードを起動
$ rocker --devices /dev/dri/card0 --x11 --net=host --privileged \
    --volume ~/crane_ws:/root/overlay_ws /dev:/dev \
    -- ghcr.io/rt-net/crane_plus:$ROS_DISTRO \
    ros2 launch crane_plus_examples demo.launch.py
```

## Dockerイメージをビルドする

`./build_source.sh $ROS_DISTRO`を実行してイメージを作成します。

```sh
# foxyディストリビューションのイメージを作成する
$ cd crane_plus/.docker
$ ./build_source.sh foxy
...
Successfully tagged crane_plus:foxy
```

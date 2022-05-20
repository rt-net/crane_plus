# Dockerを使用する

## Dockerイメージをダウンロードする

TBD

## Dockerイメージを使用する

TBD

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
    -- crane_plus:foxy ros2 launch crane_plus_examples demo.launch.py
# Intelグラフィックスを使用する場合
$ rocker --devices /dev/dri/card0 --x11 --net=host --privileged \
    --volume /dev:/dev \
    -- crane_plus:foxy ros2 launch crane_plus_examples demo.launch.py
```

### パッケージの再ビルド

ホストに作成したワークスペースをDockerコンテナにマウントすることで、
ホスト環境でのファイル編集をコンテナ内に反映できます

```sh
# ワークスペース作成
$ mkdir -p ~/crane_ws/src
# パッケージをクローン
$ git clone https://github.com/rt-net/crane_plus.git ~/crane_ws/src/crane_plus

# パッケージをビルド
$ rocker --devices /dev/dri/card0 --x11 --net=host --privileged \
    --volume ~/crane_ws:/root/overlay_ws \
    -- crane_plus:foxy colcon build --symlink-install

# ノードを起動
$ rocker --devices /dev/dri/card0 --x11 --net=host --privileged \
    --volume ~/crane_ws:/root/overlay_ws /dev:/dev \
    -- crane_plus:foxy ros2 launch crane_plus_examples demo.launch.py

```

## Dockerイメージをビルドする

`./build_source.sh ROS_DISTRO`を実行してイメージを作成します。

```sh
# foxy向けにイメージを作成する
$ cd crane_plus/.docker
$ ./build_source.sh foxy
...
Successfully tagged crane_plus:foxy
```

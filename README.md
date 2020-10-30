# crane_plus

[![container_ci](https://github.com/rt-net/crane_plus/workflows/container_ci/badge.svg?branch=master)](https://github.com/rt-net/crane_plus/actions?query=workflow%3Acontainer_ci+branch%3Amaster)

ROS 2 package suite of CRANE+V2.

<img src=https://www.rt-shop.jp/images/RT/CRANEplusV2.png width=400px/><img src=https://rt-net.github.io/images/crane-plus/pick_and_place.gif width=400px />

## Table of Contents

- [Requirements](#requirements)
- [Installation](#installation)
  - [MoveIt 2 Source Build](#moveit-2-source-build)
  - [Crane_plus Source Build](#crane_plus-source-build)
- [Quick Start](#quick-start)
- [Packages](#packages)
- [License](#license)

## Requirements

- CRANE+V2
    - https://rt-net.jp/products/cranev2/
    - [RT Robot Shop](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_1&products_id=3626&language=ja)
- Linux OS
  - Ubuntu 20.04
- ROS
  - [Foxy Fitzroy](https://index.ros.org/doc/ros2/Installation/Foxy/)

## Installation

このパッケージで使用している[moveit2](https://github.com/ros-planning/moveit2)
パッケージはROS 2 Foxy向けにリリースされていません。
そのため、[moveit2 website](https://moveit.ros.org/install-moveit2/source/)
の手順に従って`moveit2`及び依存パッケージをインストールした後に
`crane_plus`パケージをインストールします。

### MoveIt 2 Source Build

詳細は[こちら](https://moveit.ros.org/install-moveit2/source/)
を参照してください。

まずはじめにビルドツールをインストールします。
(詳細は[こちら](https://index.ros.org/doc/ros2/Installation/Foxy/Linux-Development-Setup/#install-development-tools-and-ros-tools))

次のコマンドを実行します。

```sh
sudo apt update && sudo apt install -y \
  build-essential \
  cmake \
  git \
  libbullet-dev \
  python3-colcon-common-extensions \
  python3-flake8 \
  python3-pip \
  python3-pytest-cov \
  python3-rosdep \
  python3-setuptools \
  python3-vcstool \
  wget
# install some pip packages needed for testing
python3 -m pip install -U \
  argcomplete \
  flake8-blind-except \
  flake8-builtins \
  flake8-class-newline \
  flake8-comprehensions \
  flake8-deprecated \
  flake8-docstrings \
  flake8-import-order \
  flake8-quotes \
  pytest-repeat \
  pytest-rerunfailures \
  pytest
# install Fast-RTPS dependencies
sudo apt install --no-install-recommends -y \
  libasio-dev \
  libtinyxml2-dev
# install Cyclone DDS dependencies
sudo apt install --no-install-recommends -y \
  libcunit1-dev
```

`moveit2`用のワークスペースを作成します。

```sh
$ mkdir -p ~/moveit_ws/src
```

`moveit2`と依存パッケージをダウンロードします。

```sh
$ cd ~/moveit_ws/src
$ git clone https://github.com/ros-planning/moveit2.git -b main
$ vcs import < moveit2/moveit2.repos
$ rosdep install -r --from-paths . --ignore-src --rosdistro foxy -y
```

`moveit2`をビルドします。
パッケージ数が多いためビルド時間は長いです。

```sh
$ cd ~/moveit_ws
$ colcon build --event-handlers desktop_notification- status- --cmake-args -DCMAKE_BUILD_TYPE=Release
```

ビルドが完了したら次のコマンドを実行してパッケージを読み込みます。
このコマンドは`moveit2`及び`crane_plus`を使用する際に毎回実行します。

```sh
$ source ~/moveit_ws/install/setup.bash
```

### Crane_plus Source Build

`moveit2`とは別のワークスペースを作成します。

```sh
$ mkdir -p ~/ros2_ws/src
```

`crane_plus`と依存パッケージをダウンロードします。

```sh
$ cd ~/ros2_ws/src
$ git clone https://github.com/rt-net/crane_plus.git

# Install dependencies
# Run 'source ~/moveit_ws/install/setup.bash' before installation.
$ rosdep install -r -y -i --from-paths .
```

`crane_plus`をビルドします。

```sh
$ cd ~/ros2_ws
$ colcon build --symlink-install
```

ビルドが完了したら次のコマンドを実行してパッケージを読み込みます。
このコマンドは`crane_plus`を使用する際に毎回実行します。

```sh
$ source ~/ros2_ws/install/setup.bash
```

## Quick Start

```sh
# Connect CRANE+V2 to PC, then
$ ros2 launch crane_plus_examples demo.launch.py

# Terminal 2
$ ros2 launch crane_plus_examples example.launch.py example:='gripper_control'
```

<img src=https://rt-net.github.io/images/crane-plus/gripper_control.gif width=500px />

詳細は[crane_plus_examples](./crane_plus_examples/README.md)
を参照してください。

## Packages

- crane_plus_control
  - [README](./crane_plus_control/README.md)
  - CRANE+V2を制御するパッケージです
  - USB通信ポートの設定方法をREAMDEに記載してます
- crane_plus_description
  - [README](./crane_plus_description/README.md)
  - CRANE+V2のモデルデータ（xacro）を定義するパッケージです
- crane_plus_examples
  - [README](./crane_plus_examples/README.md)
  - CRANE+V2のサンプルコード集です
- crane_plus_gazebo
  - [README](./crane_plus_gazebo/README.md)
  - CRANE+V2のGazeboシミュレーションパッケージです
  - **現在、Gazebo上のCRANE+V2を制御することはできません**
- crane_plus_moveit_config
  - [README](./crane_plus_moveit_config/README.md)
  - CRANE+V2の`moveit2`設定ファイルです

## License

このリポジトリはApache 2.0ライセンスの元、公開されています。 
ライセンスについては[LICENSE](./LICENSE)を参照ください。

サーボモータのAX-12Aに関するCADモデルの使用については、ROBOTIS社より使用許諾を受けています。 
CRANE+V2に使用されているROBOTIS社の部品類にかかる著作権、商標権、その他の知的財産権は、ROBOTIS社に帰属します。

We have obtained permission from ROBOTIS Co., Ltd. to use CAD models relating to servo motors AX-12A. The proprietary rights relating to any components or parts manufactured by ROBOTIS and used in this product, including but not limited to copyrights, trademarks, and other intellectual property rights, shall remain vested in ROBOTIS.

## Disclaimer

本ソフトウェアはApache 2.0ライセンスで、「AS IS」（現状有姿のまま）で提供しています。本ソフトウェアに関する無償サポートはありません。

当該製品および当ソフトウェアの使用中に生じたいかなる損害も株式会社アールティでは一切の責任を負いかねます。 ユーザー自身で作成されたプログラムに適切な制限動作が備わっていない場合、本体の損傷や、本体が周囲や作業者に接触、あるいは衝突し、思わぬ重大事故が発生する危険があります。 ユーザーの責任において十分に安全に注意した上でご使用下さい。


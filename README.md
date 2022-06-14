# crane_plus

[![industrial_ci](https://github.com/rt-net/crane_plus/workflows/industrial_ci/badge.svg?branch=master)](https://github.com/rt-net/crane_plus/actions?query=workflow%3Aindustrial_ci+branch%3Amaster)

ROS 2 package suite of CRANE+V2.

<img src=https://www.rt-shop.jp/images/RT/CRANEplusV2.png width=400px/><img src=https://rt-net.github.io/images/crane-plus/pick_and_place.gif width=400px />

## Table of Contents

- [Requirements](#requirements)
- [Installation](#installation)
  - [Binary installation](#binary-installation)
  - [Source Build](#source-build)
- [Quick Start](#quick-start)
- [Packages](#packages)
- [License](#license)
- [Disclaimer](#disclaimer)

## Requirements

- CRANE+V2
  - [Product Introduction](https://rt-net.jp/products/cranev2/)
  - [Web Shop](https://www.rt-shop.jp/index.php?main_page=product_info&cPath=1348_1&products_id=3626&language=ja)
- Linux OS
  - Ubuntu 20.04
- ROS
  - [Foxy Fitzroy](https://index.ros.org/doc/ros2/Installation/Foxy/)

## Installation

### Docker images

ビルド済みのパッケージ含むDocker imageを用意してます。
詳細は[.docker/README.md](./.docker/README.md)を参照してください。

### Binary installation

TBD

### Source Build

```sh
# Setup ROS environment
$ source /opt/ros/foxy/setup.bash

# Download crane_plus repository
$ mkdir -p ~/ros2_ws/src
$ cd ~/ros2_ws/src
$ git clone https://github.com/rt-net/crane_plus.git

# Install dependencies
$ rosdep install -r -y -i --from-paths .

# Build & Install
$ cd ~/ros2_ws
$ colcon build --symlink-install
$ source ~/ros2_ws/install/setup.bash
```

## Quick Start

```sh
# Connect CRANE+V2 to PC, then
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch crane_plus_examples demo.launch.py port_name:=/dev/ttyUSB0

# Terminal 2
$ source ~/ros2_ws/install/setup.bash
$ ros2 launch crane_plus_examples example.launch.py example:='gripper_control'

# Press [Ctrl-c] to terminate.
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
- crane_plus_ignition
  - [README](./crane_plus_ignition/README.md)
  - CRANE+V2のIgnition Gazeboシミュレーションパッケージです
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


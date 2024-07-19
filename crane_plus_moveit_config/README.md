# crane_plus_moveit_config

このパッケージはCRANE+ V2のmove_group設定ファイル及びlaunchファイルを含んでいます。

[ros-planning/moveit2/moveit_demo_nodes/run_move_group](https://github.com/ros-planning/moveit2/tree/main/moveit_demo_nodes/run_move_group)
を参考にパッケージを作成しています。

## ノードの起動

`run_move_group.launch.py`を実行すると、`move_group`や`rviz`等のノードが起動します。
コントローラノードは起動しないため、
CRANE+ V2本体を動かすことはできません。(`crane_plus_examples`を参照してください。)

## configファイル

- controllers.yaml
  - `moveit_simple_controller_manager`のパラメータを設定しています
  - 設定内容は`crane_plus_control`のコントローラ名やコントローラタイプに依存します
- crane_plus.srdf
  - move_groupとして`arm`、`gripper`を設定しています
  - `arm`のgourp_stateとして`home`、`vertical`を設定しています
- kinematics.yaml
  - `arm`のkinematics_solverを設定しています
  - デフォルトの`KDLKinematicsPlugin`では軌道計画に失敗するため、`LMAKinematicsPlugin`を使用しています
- ompl_planning.yaml
  - Open Motion Planning Libraryのパラメータを設定しています
  - [ros-planning/moveit_resources/panda_moveit_config](https://github.com/ros-planning/moveit_resources/tree/master/panda_moveit_config)のパラメータを流用しています

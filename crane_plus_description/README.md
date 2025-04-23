# crane_plus_description

このパッケージはCRANE+ V2のモデルデータ(xacro)を所持しています。

## display robot model

下記のコマンドを実行して、`robot_state_publisher`、`joint_state_publisher`、`rviz2`を起動します。

CRANE+ V2のモデルが表示されるので、xacroファイルのデバッグに役立ちます。

```sh
$ ros2 launch crane_plus_description display.launch.py
```

Webカメラ搭載モデルの場合は、下記のコマンドを実行してください。

```sh
$ ros2 launch crane_plus_description display.launch.py use_camera:=true
```

![display.launch.py](https://rt-net.github.io/images/crane-plus/display_launch.png)

## configure servo angle limits

CRANE+ V2の実機を動かす場合は、
事前にサーボモータ内部の角度リミット（`CW Angle Limit`、`CCW Angle Limit`）を設定してください。

CRANE+ V2に搭載されているサーボモータはROBOTISのAX-12Aのため、
[Dynamixel Wizard 2](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
を使用して角度リミットを設定できます。

![dynamixel wizard2](https://rt-net.github.io/images/crane-plus/dynamixel_wizard2.png)

[crane_plus.urdf.xacro](./urdf/crane_plus.urdf.xacro)には、
下記のように各関節の角度リミットが定義されています。
この角度リミットに近い値をサーボモータに設定することを推奨します。

```xml
  <xacro:property name="SERVO_HOME" value="${radians(150.0)}"/>
  <xacro:property name="JOINT_EFFORT_LIMIT" value="1.5"/>
  <xacro:property name="JOINT_VELOCITY_LIMIT" value="2.0"/>
  <xacro:property name="JOINT_1_LOWER_LIMIT" value="${radians(0) - SERVO_HOME}"/>
  <xacro:property name="JOINT_1_UPPER_LIMIT" value="${radians(300) - SERVO_HOME}"/>
  <xacro:property name="JOINT_2_LOWER_LIMIT" value="${radians(45) - SERVO_HOME}"/>
  <xacro:property name="JOINT_2_UPPER_LIMIT" value="${radians(253) - SERVO_HOME}"/>
  <xacro:property name="JOINT_3_LOWER_LIMIT" value="${radians(3) - SERVO_HOME}"/>
  <xacro:property name="JOINT_3_UPPER_LIMIT" value="${radians(291) - SERVO_HOME}"/>
  <xacro:property name="JOINT_4_LOWER_LIMIT" value="${radians(44) - SERVO_HOME}"/>
  <xacro:property name="JOINT_4_UPPER_LIMIT" value="${radians(252) - SERVO_HOME}"/>
  <xacro:property name="JOINT_HAND_LOWER_LIMIT" value="${radians(109) - SERVO_HOME}"/>
  <xacro:property name="JOINT_HAND_UPPER_LIMIT" value="${radians(189) - SERVO_HOME}"/>
```

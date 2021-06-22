# crane_plus_description

このパッケージはCRANE+V2のモデルデータ(xacro)を所持しています。

## display robot model

下記のコマンドを実行して、`robot_state_publisher`、`joint_state_publisher`、`rviz2`を起動します。

CRANE+V2のモデルが表示されるので、xacroファイルのデバッグに役立ちます。

```sh
$ ros2 launch crane_plus_description display.launch.py
```

![display.launch.py](https://rt-net.github.io/images/crane-plus/display_launch.png)

## configure servo angle limits

CRANE+V2の実機を動かす場合は、
事前にサーボモータ内部の角度リミット（`CW Angle Limit`、`CCW Angle Limit`）を設定してください。

CRANE+V2に搭載されているサーボモータはROBOTISのAX-12Aのため、
[Dynamixel Wizard 2](https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_wizard2/)
を使用して角度リミットを設定できます。

![dynamixel wizard2](https://rt-net.github.io/images/crane-plus/dynamixel_wizard2.png)

[crane_plus.urdf.xacro](./urdf/crane_plus.urdf.xacro)には、
下記のように各関節の角度リミットが定義されています。
この角度リミットに近い値をサーボモータに設定することを推奨します。

```xml
  <xacro:property name="M_PI" value="3.14159"/>
  <xacro:property name="TO_RADIAN" value="${M_PI / 180.0}"/>
  <xacro:property name="SERVO_HOME" value="${TO_RADIAN * 150.0}"/>
  <xacro:property name="JOINT_VELOCITY_LIMIT" value="2.0"/>
  <xacro:property name="JOINT_1_LOWER_LIMIT" value="${0.0 * TO_RADIAN - SERVO_HOME}"/>
  <xacro:property name="JOINT_1_UPPER_LIMIT" value="${300.0 * TO_RADIAN - SERVO_HOME}"/>
  <xacro:property name="JOINT_2_LOWER_LIMIT" value="${45.45 * TO_RADIAN - SERVO_HOME}"/>
  <xacro:property name="JOINT_2_UPPER_LIMIT" value="${252.20 * TO_RADIAN - SERVO_HOME}"/>
  <xacro:property name="JOINT_3_LOWER_LIMIT" value="${3.52 * TO_RADIAN - SERVO_HOME}"/>
  <xacro:property name="JOINT_3_UPPER_LIMIT" value="${290.62 * TO_RADIAN - SERVO_HOME}"/>
  <xacro:property name="JOINT_4_LOWER_LIMIT" value="${44.57 * TO_RADIAN - SERVO_HOME}"/>
  <xacro:property name="JOINT_4_UPPER_LIMIT" value="${251.32 * TO_RADIAN - SERVO_HOME}"/>
  <xacro:property name="JOINT_HAND_LOWER_LIMIT" value="${109.38 * TO_RADIAN - SERVO_HOME}"/>
  <xacro:property name="JOINT_HAND_UPPER_LIMIT" value="${188.27 * TO_RADIAN - SERVO_HOME}"/>
```

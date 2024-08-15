import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String


# generic ros libraries
from rclpy.logging import get_logger

# moveit python library
from moveit.core.robot_state import RobotState
from moveit.planning import (
    MoveItPy,
    MultiPipelinePlanRequestParameters,
)

from config import moveit_config

def to_radians(deg_angle):
    return deg_angle * math.pi / 180.0


def main(args=None):
    # ros2の初期化
    rclpy.init(args=args)
    
    # ロガー生成
    logger = get_logger("gripper_control")
    
    # MoveItPy初期化
    crane_plus = MoveItPy(node_name="moveit_py", config_dict=moveit_config)
    crane_plus_gripper = crane_plus.get_planning_component("gripper")
    logger.info("MoveItPy instance created")
    
    
    
    
    
    
    
    # MoveItPyの終了
    crane_plus.shutdown()
    
    # rclpy.shutdown()
if __name__ == '__main__':
    main()
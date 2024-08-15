import rclpy
from rclpy.node import Node

import math

# from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import Quaternion
import tf2_ros


class ImageSubscriber(Node):
    def __init__(self):
        super().__init__("aruco_detection")
        self.image_subscription = self.create_subscription(
            Image, "image_raw", self.listener_callback, 10
        )
        self.image_subscription  # prevent unused variable warning
        
        self.camera_info_subscription = self.create_subscription(
            Image, "camera_info", self.listener_callback, 10
        )
        self.camera_info_subscription  # prevent unused variable warning
        
        self.camera_info = None

    def image_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)
    
    def camera_info_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()
    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

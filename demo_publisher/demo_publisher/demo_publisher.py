import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

import logging
import threading

import sensor_msgs.msg
import geometry_msgs.msg
from std_msgs.msg import String
import cv2
from cv_bridge import CvBridge, CvBridgeError

from demo_publisher.repeated_timer import RepeatedTimer

class DemoPublisher(Node):
    def __init__(self):
        Node.__init__(self, 'demo_publisher')

        self.logger = self.get_logger()
        output_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=5)
        
        self._message = String()

        self.bridge = CvBridge()
        self.frame = cv2.imread("robomaster_ros/demo_publisher/resource/example.jpg")
        self._image_pub = self.create_publisher(sensor_msgs.msg.Image, "/image", output_qos)

        self._image = sensor_msgs.msg.Image()
        self._string_pub = self.create_publisher(String, '/demo_string', output_qos)

        self.count = 0
        self.timer = RepeatedTimer(1.0, self.publish)
        self.timer.start()

    def publish(self):
        self._message.data = "count:"+str(self.count)
        self.logger.info(self._message.data)
        print(self._message.data)
        self._string_pub.publish(self._message)
        self.count += 1

        self._image_pub.publish(self.bridge.cv2_to_imgmsg(self.frame, "bgr8"))
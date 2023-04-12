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
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rclpy.action import ActionClient

from PyQt5.QtCore import QObject, pyqtSignal
from PyQt5.QtWidgets import QWidget

from typing import Any, Optional
import logging

import sensor_msgs.msg
import geometry_msgs.msg
import robomaster_msgs.msg
from robomaster_msgs.action import GripperControl
from std_msgs.msg import String

import enum

class GripperState(enum.Enum):
    PAUSED = 0
    OPEN = 1
    CLOSE = 2

class ConsoleNode(Node, QObject):
    printSignal = pyqtSignal(str)
    def __init__(self, executor: rclpy.executors.Executor):
        Node.__init__(self, 'console_node')
        QObject.__init__(self)

        self.logger = self.get_logger()

        output_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1)
        input_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1)

        self.move_cmd = geometry_msgs.msg.Twist()
        self.arm_cmd = geometry_msgs.msg.Point()
        self._gripper_client = ActionClient(self, GripperControl, '/gripper')

        self.arm_status = geometry_msgs.msg.PointStamped()
        self.gripper_status = robomaster_msgs.msg.GripperState()

        self._twist_pub = self.create_publisher(geometry_msgs.msg.Twist, '/cmd_vel', output_qos)
        self._arm_pub = self.create_publisher(geometry_msgs.msg.Point, '/target_arm_position', output_qos)
        self._wheel_speeds_pub = self.create_publisher(robomaster_msgs.msg.WheelSpeeds, '/cmd_wheels', output_qos)

        self._arm_sub = self.create_subscription(geometry_msgs.msg.PointStamped, '/arm_position', 
                                                 self.update_arm_status, 1)
        self._lidar_sub = self.create_subscription(sensor_msgs.msg.LaserScan, '/scan', 
                                                 self.update_laser_scan, input_qos)
        
        # reset arm
        self.arm_max_x = 0.187
        self.arm_min_x = 0.07
        self.arm_max_z = 0.15
        self.arm_min_z = 0.05

        self.arm_cmd.x = self.arm_min_x
        self.arm_cmd.z = self.arm_min_z
        self._arm_pub.publish(self.arm_cmd)

    def drive(self, linear_speed):
        self.move_cmd.linear.x = linear_speed
        self._twist_pub.publish(self.move_cmd)

    def turn(self, angular_speed):
        self.move_cmd.angular.z = angular_speed
        self._twist_pub.publish(self.move_cmd)
    
    def stop(self):
        self.move_cmd.linear.x = 0.0
        self.move_cmd.angular.z = 0.0
        self._twist_pub.publish(self.move_cmd)
    
    def lift_arm(self):
        self.arm_cmd.z += 0.01
        self.arm_cmd.z = max(self.arm_min_z, min(self.arm_max_z, self.arm_cmd.z))
        self._arm_pub.publish(self.arm_cmd)

    def lower_arm(self):
        self.arm_cmd.z -= 0.01
        self.arm_cmd.z = max(self.arm_min_z, min(self.arm_max_z, self.arm_cmd.z))
        self._arm_pub.publish(self.arm_cmd)

    def reach_arm(self):
        self.arm_cmd.x += 0.01
        self.arm_cmd.x = max(self.arm_min_x, min(self.arm_max_x, self.arm_cmd.x))
        self._arm_pub.publish(self.arm_cmd)

    def withdraw_arm(self):
        self.arm_cmd.x -= 0.01
        self.arm_cmd.x = max(self.arm_min_x, min(self.arm_max_x, self.arm_cmd.x))
        self._arm_pub.publish(self.arm_cmd)

    def enclasp_gripper(self):
        goal_msg = GripperControl.Goal()
        goal_msg.target_state = 2
        goal_msg.power = 0.5
        # self._gripper_client.wait_for_server()
        # self._gripper_client.send_goal_async(goal_msg)
        self._gripper_client.send_goal(goal_msg)


    def loose_gripper(self):
        goal_msg = GripperControl.Goal()
        goal_msg.target_state = 1
        # self._gripper_client.send_goal_async(goal_msg)
        self._gripper_client.send_goal(goal_msg)

    def stop_gripper(self):
        goal_msg = GripperControl.Goal()
        goal_msg.target_state = 0
        # self._gripper_client.send_goal_async(goal_msg)
        self._gripper_client.send_goal(goal_msg)

    def update_arm_status(self, status: geometry_msgs.msg.PointStamped):
        self.arm_status = status

    def update_laser_scan(self, status: sensor_msgs.msg.LaserScan):
        self.printSignal.emit("scan_time: "+str(status.scan_time))
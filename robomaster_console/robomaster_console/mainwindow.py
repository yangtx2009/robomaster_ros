from robomaster_console.console_node import ConsoleNode
from robomaster_console.spin_thread import SpinThread
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QWidget, QLabel, \
        QHBoxLayout, QGridLayout, QVBoxLayout, QSizePolicy, QGroupBox
from PyQt5.QtGui import QIcon, QPixmap, QTransform, QKeySequence
from PyQt5.QtCore import QSize, Qt, QEvent, QThread
import sys
import pathlib

import rclpy

import sensor_msgs.msg

class MainWindow(QMainWindow):
    def __init__(self, node: ConsoleNode, executor: rclpy.executors.Executor):
        super().__init__()
        self.setWindowTitle("Demobot Console")
        self.resize(800, 500)

        self.lateral_speed = 0.5
        self.angular_speed = 0.6
        self.count = 1

        self.node = node
        self.init_layouts()
        self.init_widgets()
        self.init_connection()

        self.thread = SpinThread(node, executor)
        self.thread.start()

        self.installEventFilter(self)
    
    def closeEvent(self, event):
        self.thread.stop()
        self.thread.wait()
        event.accept()

    def init_layouts(self):
        main_widget = QWidget(self)
        main_layout = QHBoxLayout(main_widget)
        self.setCentralWidget(main_widget)

        # navigation
        navi_groupbox = QGroupBox("Navigation")
        main_layout.addWidget(navi_groupbox, 3)
        self.navigation_layout = QGridLayout()
        navi_groupbox.setLayout(self.navigation_layout)

        arm_gripper_layout = QVBoxLayout()
        main_layout.addLayout(arm_gripper_layout)

        # arm
        arm_groupbox = QGroupBox("Arm")
        arm_gripper_layout.addWidget(arm_groupbox, 1)
        self.arm_layout1 = QVBoxLayout()
        arm_groupbox.setLayout(self.arm_layout1)
        self.arm_layout2 = QHBoxLayout()
        arm_inout_widget = QWidget()
        arm_inout_widget.setLayout(self.arm_layout2)
        self.arm_layout1.addWidget(arm_inout_widget)

        # gripper
        gripper_groupbox = QGroupBox("Gripper")
        arm_gripper_layout.addWidget(gripper_groupbox, 1)
        self.gripper_layout = QHBoxLayout()
        gripper_groupbox.setLayout(self.gripper_layout)
        arm_gripper_layout.addWidget(gripper_groupbox)

        # control_widget = QWidget(self)
        self.control_layout = QVBoxLayout()
        main_layout.addLayout(self.control_layout, 2)

        self.battery_layout = QHBoxLayout()
        self.control_layout.addLayout(self.battery_layout)

    def init_widgets(self):
        self.forward_button = QPushButton(self)
        self.forward_button.setAutoRepeat(False)
        self.forward_button.setFixedSize(110,80)
        # self.forward_button.setShortcut("W")
        forward_icon = QPixmap("robomaster_console/resource/up-arrow.png")
        self.forward_button.setIcon(QIcon(forward_icon))
        self.forward_button.setIconSize(QSize(80,80))
        self.forward_button.setFlat(True)
        self.forward_button.setToolTip("W")
        self.navigation_layout.addWidget(self.forward_button, 0, 1)

        self.backward_button = QPushButton(self)
        self.backward_button.setAutoRepeat(False)
        self.backward_button.setFixedSize(110,80)
        backward_icon = QPixmap("robomaster_console/resource/up-arrow.png")
        backward_icon = backward_icon.transformed(QTransform().rotate(180), Qt.SmoothTransformation)
        self.backward_button.setIcon(QIcon(backward_icon))
        self.backward_button.setIconSize(QSize(80,80))
        self.backward_button.setFlat(True)
        self.backward_button.setToolTip("S")
        self.navigation_layout.addWidget(self.backward_button, 2, 1)

        self.left_button = QPushButton(self)
        self.left_button.setAutoRepeat(False)
        self.left_button.setFixedSize(80,80)
        # self.left_button.setShortcut("A")
        left_icon = QPixmap("robomaster_console/resource/up-arrow.png")
        left_icon = left_icon.transformed(QTransform().rotate(270), Qt.SmoothTransformation)
        self.left_button.setIcon(QIcon(left_icon))
        self.left_button.setIconSize(QSize(80,80))
        self.left_button.setFlat(True)
        self.left_button.setToolTip("A")
        self.navigation_layout.addWidget(self.left_button,1, 0)

        self.right_button = QPushButton(self)
        self.right_button.setAutoRepeat(False)
        self.right_button.setFixedSize(80,80)
        right_icon = QPixmap("robomaster_console/resource/up-arrow.png")
        right_icon = right_icon.transformed(QTransform().rotate(90), Qt.SmoothTransformation)
        self.right_button.setIcon(QIcon(right_icon))
        self.right_button.setIconSize(QSize(80,80))
        self.right_button.setFlat(True)
        self.right_button.setToolTip("D")
        self.navigation_layout.addWidget(self.right_button,1, 2)

        # arm
        self.up_button = QPushButton(self)
        self.up_button.setAutoRepeat(False)
        self.up_button.setFixedSize(105,80)
        up_icon = QPixmap("robomaster_console/resource/up-arrow.png")
        self.up_button.setIcon(QIcon(up_icon))
        self.up_button.setIconSize(QSize(80,80))
        self.up_button.setFlat(True)
        self.up_button.setToolTip("E")
        self.arm_layout1.insertWidget(0, self.up_button, 2)

        self.down_button = QPushButton(self)
        self.down_button.setAutoRepeat(False)
        self.down_button.setFixedSize(105,80)
        down_icon = QPixmap("robomaster_console/resource/up-arrow.png")
        down_icon = down_icon.transformed(QTransform().rotate(180), Qt.SmoothTransformation)
        self.down_button.setIcon(QIcon(down_icon))
        self.down_button.setIconSize(QSize(80,80))
        self.down_button.setFlat(True)
        self.down_button.setToolTip("C")
        self.arm_layout1.insertWidget(1, self.down_button, 2)

        self.in_button = QPushButton(self)
        self.in_button.setAutoRepeat(False)
        self.in_button.setFixedSize(50,50)
        in_icon = QPixmap("robomaster_console/resource/up-arrow.png")
        in_icon = in_icon.transformed(QTransform().rotate(270), Qt.SmoothTransformation)
        self.in_button.setIcon(QIcon(in_icon))
        self.in_button.setIconSize(QSize(50,50))
        self.in_button.setFlat(True)
        self.in_button.setToolTip("F")
        self.arm_layout2.addWidget(self.in_button)

        self.out_button = QPushButton(self)
        self.out_button.setAutoRepeat(False)
        self.out_button.setFixedSize(50,50)
        out_icon = QPixmap("robomaster_console/resource/up-arrow.png")
        out_icon = out_icon.transformed(QTransform().rotate(90), Qt.SmoothTransformation)
        self.out_button.setIcon(QIcon(out_icon))
        self.out_button.setIconSize(QSize(50,50))
        self.out_button.setFlat(True)
        self.out_button.setToolTip("F")
        self.arm_layout2.addWidget(self.out_button)

        # gripper
        self.hold_button = QPushButton(self)
        self.hold_button.setAutoRepeat(False)
        self.hold_button.setFixedSize(50,50)
        hold_icon = QPixmap("robomaster_console/resource/unfold-less.png")
        hold_icon = hold_icon.transformed(QTransform().rotate(90), Qt.SmoothTransformation)
        self.hold_button.setIcon(QIcon(hold_icon))
        self.hold_button.setIconSize(QSize(50,50))
        self.hold_button.setFlat(True)
        self.hold_button.setToolTip("V")
        self.gripper_layout.addWidget(self.hold_button)

        self.release_button = QPushButton(self)
        self.release_button.setAutoRepeat(False)
        self.release_button.setFixedSize(50,50)
        release_icon = QPixmap("robomaster_console/resource/unfold-more.png")
        release_icon = release_icon.transformed(QTransform().rotate(90), Qt.SmoothTransformation)
        self.release_button.setIcon(QIcon(release_icon))
        self.release_button.setIconSize(QSize(50,50))
        self.release_button.setFlat(True)
        self.release_button.setToolTip("B")
        self.gripper_layout.addWidget(self.release_button)


        # battery status
        label = QLabel("Battery Status:", self)
        label.setMidLineWidth(50)
        self.battery_status_label = QLabel(self)
        self.battery_layout.addWidget(label)
        self.battery_layout.addWidget(self.battery_status_label)

        # set stop button
        self.stop_button = QPushButton(self)
        self.stop_button.setShortcut("Ctrl+B")
        self.stop_button.setIcon(QIcon("robomaster_console/resource/stop.png"))
        self.stop_button.setIconSize(QSize(200,200))
        self.stop_button.setFlat(True)

        vertical_policy = QSizePolicy()
        vertical_policy.setVerticalPolicy(QSizePolicy.Expanding)
        self.stop_button.setSizePolicy(vertical_policy)
        self.control_layout.addWidget(self.stop_button)

    def init_connection(self):
        # chassis
        self.forward_button.pressed.connect(self.forward)
        self.forward_button.released.connect(self.stop_lateral_movement)

        self.backward_button.pressed.connect(self.backward)
        self.backward_button.released.connect(self.stop_lateral_movement)

        self.left_button.pressed.connect(self.turn_left)
        self.left_button.released.connect(self.stop_rotation)

        self.right_button.pressed.connect(self.turn_right)
        self.right_button.released.connect(self.stop_rotation)

        # arm
        self.up_button.clicked.connect(self.lift_arm)
        self.down_button.clicked.connect(self.lower_arm)

        self.in_button.clicked.connect(self.withdraw_arm)
        self.out_button.clicked.connect(self.reach_arm)

        # gripper
        self.hold_button.pressed.connect(self.enclasp_gripper)
        self.hold_button.released.connect(self.stop_gripper)
        self.release_button.pressed.connect(self.loose_gripper)
        self.release_button.released.connect(self.stop_gripper)

        self.node.create_subscription(sensor_msgs.msg.BatteryState, '/battery', self.update_battery_status, 1)

    def eventFilter(self, source, event):
        if event.type() == QEvent.KeyPress:
            key = event.text()
            # chassis
            if key in ['d', 'D']:
                self.right_button.setDown(True)
                self.turn_right()
            elif key in ['a', 'A']:
                self.left_button.setDown(True)
                self.turn_left()
            elif key in ['w', 'W']:
                self.forward_button.setDown(True)
                self.forward()
            elif key in ['s', 'S']:
                self.backward_button.setDown(True)
                self.backward()

            # arm
            elif key in ['e', 'E']:
                self.up_button.setDown(True)
                self.lift_arm()
            elif key in ['c', 'C']:
                self.down_button.setDown(True)
                self.lower_arm()
            elif key in ['f', 'F']:
                self.in_button.setDown(True)
                self.withdraw_arm()
            elif key in ['g', 'G']:
                self.out_button.setDown(True)
                self.reach_arm()
            
            # gripper
            elif key in ['v', 'V']:
                self.hold_button.setDown(True)
                self.enclasp_gripper()
            elif key in ['b', 'B']:
                self.release_button.setDown(True)
                self.loose_gripper()

        elif event.type() == QEvent.KeyRelease:
            key = event.text()

            # chassis
            if key in ['d', 'D']:
                self.right_button.setDown(False)
                self.stop_rotation()
            elif key in ['a', 'A']:
                self.left_button.setDown(False)
                self.stop_rotation()
            elif key in ['w', 'W']:
                self.forward_button.setDown(False)
                self.stop_lateral_movement()
            elif key in ['s', 'S']:
                self.backward_button.setDown(False)
                self.stop_lateral_movement()

            # arm
            elif key in ['e', 'E']:
                self.up_button.setDown(False)
            elif key in ['c', 'C']:
                self.down_button.setDown(False)
            elif key in ['f', 'F']:
                self.in_button.setDown(False)
            elif key in ['g', 'G']:
                self.out_button.setDown(False)

            # gripper
            elif key in ['v', 'V']:
                self.hold_button.setDown(False)
                self.stop_gripper()
            elif key in ['b', 'B']:
                self.release_button.setDown(False)
                self.stop_gripper()

        return super(MainWindow, self).eventFilter(source, event)

    def forward(self):
        self.node.drive(self.lateral_speed)

    def backward(self):
        self.node.drive(-1.0*self.lateral_speed)

    def stop_lateral_movement(self):
        self.node.drive(0.0)
    
    def turn_left(self):
        self.node.turn(self.angular_speed)
    
    def turn_right(self):
        self.node.turn(-1.0*self.angular_speed)
    
    def stop_rotation(self):
        self.node.turn(0.0)

    def lift_arm(self):
        self.node.lift_arm()

    def lower_arm(self):
        self.node.lower_arm()
    
    def reach_arm(self):
        self.node.reach_arm()

    def withdraw_arm(self):
        self.node.withdraw_arm()

    def hold_arm(self):
        self.node.hold_arm()

    def enclasp_gripper(self):
        self.node.enclasp_gripper()

    def loose_gripper(self):
        self.node.loose_gripper()

    def stop_gripper(self):
        self.node.stop_gripper()

    def update_battery_status(self, status: sensor_msgs.msg.BatteryState):
        percentage = status.percentage * 100
        self.battery_status_label.setText(f"{int(percentage)}%")
from robomaster_console.mainwindow import MainWindow
from robomaster_console.console_node import ConsoleNode
import rclpy
import sys
from PyQt5.QtWidgets import QApplication, QWidget

def main(args=None):
    app = QApplication(sys.argv)

    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    node = ConsoleNode(executor=executor)

    window = MainWindow(node, executor)
    window.show()

    # spin node inside mainwindow
    app.exec()

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
from PyQt5.QtCore import QThread, QObject, pyqtSignal, QCoreApplication
import rclpy
import time

from robomaster_console.console_node import ConsoleNode

class SpinThread(QThread):
    finished = pyqtSignal()
    def __init__(self, node: ConsoleNode, executor: rclpy.executors.Executor):
        QThread.__init__(self)
        self.node = node
        self.executor = executor
        self.keep_working = True

    def stop(self):
        self.keep_working = False

    def run(self):
        while self.keep_working:
            rclpy.spin_once(self.node, executor=self.executor, timeout_sec=0.1)
            time.sleep(0.1)
            QCoreApplication.processEvents()

        print("finished")

        # self.finished.emit()
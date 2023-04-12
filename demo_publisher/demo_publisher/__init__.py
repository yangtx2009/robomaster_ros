import rclpy
import sys

from demo_publisher.demo_publisher import DemoPublisher

def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()
    node = DemoPublisher()
    executor.add_node(node)

    try: 
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
    
    rclpy.shutdown()

if __name__ == "__main__":
    main()
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class PathPlanner(Node):
    def __init__(self):
        super().__init__('path_planner')
        self.get_logger().info(f"Node 'path_planner' (class: PathPlanner) has been started.")
        # Add your publishers, subscribers, timers, etc. here
        # Example: self.timer = self.create_timer(1.0, self.timer_callback)

    # def timer_callback(self):
    #     self.get_logger().info('Timer callback triggered.')

def main(args=None):
    rclpy.init(args=args)
    node = PathPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

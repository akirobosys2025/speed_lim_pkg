#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class StatusChecker(Node):
    def __init__(self):
        super().__init__('status_checker')
        self.sub = self.create_subscription(
            String,
            '/speed_limit_status',
            self.cb,
            10
        )
        self.done = False
        self.result = False

    def cb(self, msg: String):
        if self.done:
            return

        print(msg.data)

        result = (
            'limited=True' in msg.data and 
            'lin_out=0.5' in msg.data and 
            'ang_out=1.0' in msg.data
        )

        self.done = True
        self.get_logger().info('Result received, shutting down...')

def main():
    rclpy.init()
    node = StatusChecker()

    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)
    
    rclpy.shutdown()
    sys.exit()

if __name__ == '__main__':
    main()


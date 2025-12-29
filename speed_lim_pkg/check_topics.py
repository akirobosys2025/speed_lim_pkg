#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Aki Moto aki.robosys2025@gmail.com
# SPDX-License-Indentifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

res = None

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

        ok = (msg.data == 'NORMAL')
        self.done = True
        self.get_logger().info('Result received, shutting down...')

        sys.exit(0 if ok else 1)

def main():
    rclpy.init()
    node = StatusChecker()

    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)

    rclpy.shutdown()

if __name__ == '__main__':
    main()


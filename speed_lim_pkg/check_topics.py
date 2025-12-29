#!/usr/bin/env python3
# SPDX-FileCopyrightText: 2025 Aki Moto
# SPDX-License-Identifier: BSD-3-Clause

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys

class StatusChecker(Node):
    def __init__(self):
        super().__init__('status_checker')
        self.sub = self.create_subscription(String, '/speed_limit_status', self.cb, 10)
        self.done = False

    def cb(self, msg: String):
        if self.done:
            return

        status = msg.data.strip()
        self.done = True

        print(f'{status}')

        self.get_logger().info('Result received, shutting down...')

def main():
    rclpy.init()
    node = StatusChecker()

    while rclpy.ok() and not node.done:
        rclpy.spin_once(node, timeout_sec=1.0)

    rclpy.shutdown()

if __name__ == '__main__':
    main()


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class SpeedLimit(Node):
    def __init__(self):
        super().__init__('speed_limit')

        self.declare_parameter('max_linear', 0.5)
        self.declare_parameter('max_angular', 1.0)

        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value

        self.emergency = False

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Bool, '/emergency_stop', self.emergency_callback, 10)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel_safe', 10)
        self.reason_pub = self.create_publisher(String, '/speed_limit_reason', 10)
        self.status_pub = self.create_publisher(String, '/speed_limit_status', 10)

        self.get_logger().info('Speed limiter node started')

    def emergency_callback(self, msg):
        self.emergency = msg.data

    def limit(self, value, max_value):
        return max(min(value, max_value), -max_value)

    def cmd_callback(self, msg):
        output = Twist()
        reason = String()
        status = String()

        if self.emergency:
            output.linear.x = 0.0
            output.angular.z = 0.0
            reason.data = 'Emergency stop active'
            status.data = 'EMERGENCY'
        else:
            output.linear.x = self.limit(msg.linear.x, self.max_linear)
            output.angular.z = self.limit(msg.angular.z, self.max_angular)

            if abs(msg.linear.x) > self.max_linear or abs(msg.angular.z) > self.max_angular:
                reason.data = 'Speed limited'
                status.data = 'LIMITED'
            else:
                reason.data = 'Normal operation'
                status.data = 'NORMAL'

        self.cmd_pub.publish(output)
        self.reason_pub.publish(reason)
        self.status_pub.publish(status)


def main():
    rclpy.init()
    node = SpeedLimit()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


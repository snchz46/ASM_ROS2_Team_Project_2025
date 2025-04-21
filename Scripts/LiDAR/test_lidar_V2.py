#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class SensorListener(Node):
    """
    This node subscribes to the '/scan' topic and logs the distance at the front (0 degrees).
    Instead of printing every time a message is received, it prints once per second using a timer.
    """

    def __init__(self):
        super().__init__('RPLIDAR_listener')

        # Latest LaserScan message received
        self.latest_scan = None

        # Create a subscriber to the LiDAR topic
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # QoS history depth
        )

        # Create a timer to process data every second (1 Hz)
        self.timer = self.create_timer(1.0, self.timer_callback)

    def scan_callback(self, msg):
        """
        Callback executed every time a LaserScan message is received.
        We store the latest message for processing in the timer.
        """
        self.latest_scan = msg

    def timer_callback(self):
        """
        This function is called once per second by the timer.
        It processes and prints data from the most recent scan.
        """
        if self.latest_scan and self.latest_scan.ranges:
            dist = self.latest_scan.ranges[0]
            self.get_logger().info(f"[1 Hz] Distance at angle 0: {dist:.2f} m")
        else:
            self.get_logger().warn("No valid LiDAR data received.")


def main(args=None):
    rclpy.init(args=args)
    node = SensorListener()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

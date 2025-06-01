import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class SensorListener(Node):
    """
    This node subscribes to the '/scan' topic, which publishes LaserScan data
    from a LiDAR sensor. Each time a new scan is available, it logs the distance
    at the 0th index, just as a simple example.
    """

    def __init__(self):
        super().__init__('RPLIDAR_listener')
        # Create a subscription to the '/scan' topic, expecting LaserScan messages.
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10  # QoS (quality of service) history depth.
        )
        # Prevent unused variable warning (good practice in Python).
        self.subscription  

    def scan_callback(self, msg):
        """
        Callback function that gets called whenever a new LaserScan message
        arrives. We log the reading at the 0th index as an example.
        """
        if msg.ranges:
            self.get_logger().info(f"Distance at angle 0: {msg.ranges[0]}")

def main(args=None):
    # Initialize rclpy for Python.
    rclpy.init(args=args)
    node = SensorListener()

    # Spin the node so callbacks are processed.
    rclpy.spin(node)

    # Clean up and shutdown.
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
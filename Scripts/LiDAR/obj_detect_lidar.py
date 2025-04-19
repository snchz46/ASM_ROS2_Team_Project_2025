
# Initial setup:
# 1. Make sure you the RPLIDAR A2 is running and the nodes and topics are created.
# 2. Run this script with the command:
#   python ~path/obj_detect_lidar.py

# This script is a simple obstacle avoidance node for a robot using LiDAR data.

import rclpy
# The rclpy library is the ROS 2 Python client library.
# It provides the necessary tools to create and manage ROS 2 nodes.
from rclpy.node import Node
# The Node class is the base class for creating a ROS 2 node.
from sensor_msgs.msg import LaserScan
# The LaserScan message contains the distance measurements from the LiDAR sensor.
# The ranges are the distances to obstacles at various angles.
# It provides methods for subscribing to topics, publishing messages, and logging.
# The LaserScan message is used to represent the data from the LiDAR sensor.
from geometry_msgs.msg import Twist

class ObstacleAvoidanceNode(Node):
    def __init__(self):
        super().__init__('obstacle_avoidance_node')
        
        # Sub to LiDAR (LaserScan)
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # Pub for velocity commands (Twist)
        # This is typically used to control the robot's movement.
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info("Obstacle avoidance node started.")

    def scan_callback(self, msg):
        if not msg.ranges or all(r == float('inf') for r in msg.ranges):
            self.get_logger().warn("No valid ranges received.")
            return
        
        # total number of points in the scan
        # msg.ranges is a list of distances at various angles
        num_points = len(msg.ranges)

        # 180 for centering the front of the robot
        # Assuming the robot is facing forward, we take the middle index as the "front"
        center_idx = num_points 

        # window size for the frontal range
        # This defines how many points to consider in front of the robot
        window_size = 10
        start_idx = max(0, center_idx - window_size)
        end_idx = min(num_points, center_idx + window_size)

        # front ranges are the distances in front of the robot
        # We take a slice of the ranges list to get the frontal distances
        frontal_ranges = msg.ranges[start_idx:end_idx]

        # minimum distance in the frontal ranges
        # This gives us the closest obstacle in front of the robot
        front_dist = min(frontal_ranges)

        # threshold for obstacle avoidance
        # If the closest obstacle is closer than this threshold, we need to react
        dist_threshold = 0.3     # 30 cm

        twist = Twist()

        if front_dist < dist_threshold:
            # twist or break
            # If an obstacle is detected within the threshold, we stop and turn
            twist.linear.x = 0.0
            twist.angular.z = 0.5
            self.get_logger().info(
                f"Obstcle detected at {front_dist:.2f} m! Twisting..."
            )
        else:
            # Forward motion
            # If the path is clear, we move forward
            twist.linear.x = 0.2
            twist.angular.z = 0.0
            self.get_logger().info(
                f"No obstacle ahead (min: {front_dist:.2f} m). Moving forward."
            )

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = ObstacleAvoidanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

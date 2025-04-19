# Implementing LiDAR ROS2 package (only ubuntu)
![image](https://github.com/user-attachments/assets/bb9dfcd6-6884-4252-855d-483c64913795)

Documents, packages and tools: [https://www.slamtec.com/en/Support#rplidar-a-series](https://www.slamtec.com/en/Support#rplidar-a-series)

ROS2 Package [Full guide](https://github.com/Slamtec/rplidar_ros/tree/ros2).

After following the guide above, create a new environment and execute these commands on your Ubuntu terminal:

````bash
mkdir -p ~/ros2_ws/src
````
````bash
cd ~/ros2_ws/src
````
````bash
git clone -b ros2 https://github.com/Slamtec/rplidar_ros.git
````
````bash
cd ~/ros2_ws/
````
````bash
source /opt/ros/<rosdistro>/setup.bash
````
````bash
colcon build --symlink-install
````
````bash
source ./install/setup.bash
````
````bash
sudo chmod 777 /dev/ttyUSB0
````
````bash
ros2 launch rplidar_ros view_rplidar_a2m8_launch.py
````
After these steps a RViz2 window will appear. Make sure that the topic is set to /scan and the Fixed Frame to "laser"

![Screenshot from 2025-04-13 12-32-52](https://github.com/user-attachments/assets/d55d1b3a-c6bc-4b53-975a-cf20d37777aa)


![RViz2](https://github.com/user-attachments/assets/6adccf89-6fe4-4c80-ada2-616be50cbd76)

# Retrieving sensor data from the LiDAR

## Script code to read data from the LiDAR

<details><summary>📜 Python Script </summary>
  
````python
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
````
</details>

---

⬅️ [Nodes and topics](02_nodes_topics.md) | 🔝 [Index](README.md) | ➡️ [Installation guides](04_installations.md)

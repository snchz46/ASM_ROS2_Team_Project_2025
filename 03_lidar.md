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

The following Python script demonstrates a minimal ROS 2 node that subscribes to LiDAR data published on the `/scan` topic. It listens for `LaserScan` messages and logs the distance value at angle 0 (the first index of the scan data) as a simple example.

[minimal_publisher.py](Scripts/LiDAR/test_lidar_V2.py)

![Screenshot from 2025-04-13 12-33-38](https://github.com/user-attachments/assets/07ff02bc-0f49-4b06-a879-d4ff9a1f7cb6)


## LiDAR-Based Obstacle Avoidance Node

This Python script implements a simple obstacle avoidance node for a robot using LiDAR data in ROS 2.  
The node subscribes to the `/scan` topic to receive `LaserScan` messages from the LiDAR sensor and publishes velocity commands to the `/cmd_vel` topic to control the robot's movement.

It analyzes the frontal range of the LiDAR readings, detects obstacles within a specified threshold distance (30 cm), and reacts by either stopping and turning or moving forward if the path is clear.

[obj_detect_lidar.py](Scripts/LiDAR/obj_detect_lidar.py)

https://github.com/user-attachments/assets/6586f72c-890b-40f4-ac10-3be17d219837



![Screenshot from 2025-04-13 12-34-12](https://github.com/user-attachments/assets/057add27-a50d-40a3-aca2-d6dbdd1155c0)

### Implementing LiDAR-Based Obstacle Avoidance Node on Turtlesim

[lidar_turtle.py](Scripts/LiDAR/lidar_turtle.py)

https://github.com/user-attachments/assets/9bbca0de-e839-4993-8c65-b6a2eeb36290

---

⬅️ [Nodes and topics](02_nodes_topics.md) | 🔝 [Index](README.md) | ➡️ [Stereo Camera Integration](04_stereo_cam.md)

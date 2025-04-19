# Implementing LiDAR (ubuntu)
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

---

⬅️ [Nodes and topics](02_nodes_topics.md) | 🔝 [Index](README.md) | ➡️ [Installation guides](04_installations.md)

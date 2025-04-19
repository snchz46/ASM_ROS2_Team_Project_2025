
# ASM ROS2 TEAM PROJECT 2025
## 🤖 ROS 2 Guide

Welcome to this practical and compact guide about ROS 2. Click on the sections below to navigate through each topic.

## 📚 Table of Contents

- [1️⃣ Introduction](01_intro.md)
- [2️⃣ Nodes and Topics](02_nodes_topics.md)
- [3️⃣ LiDAR Integration (Ubuntu)](03_lidar.md)
- [4️⃣ Compatibility Table](04_compatibility.md)
- [5️⃣ Installation Guides](05_install.md)
- [6️⃣ Data Sheets](06_datasheet.md)
- [7️⃣ References](07_references.md)



# ROS2 Introduction
## ROS 2: Nodes and Topics
In ROS 2, a node is a basic building block responsible for a specific, modular function within a robotic system. For example, one node might control wheel motors, while another might publish sensor data from a LiDAR. A complete robotic system is typically made up of many nodes working together, each performing a distinct role.
Nodes communicate with each other through several mechanisms: topics, services, actions, and parameters. Among these, topics are the most common and flexible method for data exchange.
A full robotic system is comprised of many nodes working together.

![Nodes-TopicandService](https://github.com/user-attachments/assets/488d8800-1916-43cd-9a26-cd6079f5d9c7)
>[Source: Understanding ROS Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)

A topic acts like a data bus that allows nodes to publish and subscribe to messages. One node can publish information on a topic, and multiple other nodes can subscribe to that topic to receive the data in real-time. Similarly, a node can publish multiple topics and subscribe to several others at the same time, enabling complex and dynamic data flows.
Topics are an essential part of the ROS 2 graph, enabling modular, distributed systems where components can be developed and run independently while still interacting seamlessly.
 
![Topic-MultiplePublisherandMultipleSubscriber](https://github.com/user-attachments/assets/3f7fdfc6-7162-45cc-96ff-816891511dc4)
>[Source: Understanding ROS Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)
>
## Example of Nodes and Topics
### Communication Between Two Devices on a Wired Local Network
This example demonstrates the communication between two devices connected through a wired local network using ROS 2. Each device runs different nodes and topics to test interoperability across the network.
On the Windows device on MATLAB, a node named NodePublisher_MATLAB is launched. It publishes messages to the topic /Hello_MATLAB, sending the string 'This is a MATLAB pub'. This setup verifies that nodes and topics created in MATLAB on Windows can be correctly published and recognized in the ROS 2 environment, both in a Windows terminal and Linux

![0dda7b462f995c3d1e234b4459595b49](https://github.com/user-attachments/assets/3e1cc6c1-4567-4534-8609-fb201829961f)

On the Linux device, several ROS 2 demos and applications are executed, including the Talker demo, the Turtlesim3 demo, and an RPLIDAR application. Each of these creates its own set of nodes and topics as part of their default operation, contributing to the network-wide ROS 2 graph.

![image](https://github.com/user-attachments/assets/ab71bd6d-27cf-4351-b3fa-53f973846e86)

## Implementing LiDAR (ubuntu)
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

## Appendix A: Tables
### Compatibility table

| ROS 2 Distribution     | Compatible Ubuntu Version          | Official ROS 2 Support | End of Support | Jetson Kits        | MATLAB         |
|------------------------|------------------------------------|------------------------|----------------|--------------------|----------------|
| Jazzy (rolling)        | Ubuntu 24.04 (in development)      | Yes (rolling release) | Rolling        | -                  | Not yet        |
| Iron Irwini            | Ubuntu 22.04 (Jammy)               | Yes                   | 2029           | Jetson Orin Nano   | Planned/Partial|
| Humble Hawksbill       | Ubuntu 22.04 (Jammy)               | Yes (LTS)             | 2027           | Jetson Orin Nano   | Yes            |
| Galactic Geochelone    | Ubuntu 20.04 (Focal)               | No                    | 2022           | Jetson Nano        | Yes            |
| Foxy Fitzroy           | Ubuntu 20.04 (Focal)               | No                    | 2023           | Jetson Nano        | Yes            |
| Dashing Diademata      | Ubuntu 18.04 (Bionic)              | No                    | 2021           | Not recommended    | Legacy         |
| Crystal Clemmys        | Ubuntu 18.04 (Bionic)              | No                    | 2020           | Not recommended    | No             |
| Bouncy Bolson          | Ubuntu 18.04 (Bionic)              | No                    | 2019           | Not recommended    | No             |
| Ardent Apalone         | Ubuntu 16.04 (Xenial)              | No                    | 2018           | Not supported      | No             |


## Appendix B: Installations
### Installing ROS2 on Ubuntu 20.04.6 on NVIDIA Jetson Nano

![image](https://github.com/user-attachments/assets/c0bb7570-e373-4814-b670-85ec30277220)

### Jetson Nano with Ubuntu 20.04 OS image

[Installation ](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image?tab=readme-ov-file#installation):

- Get a >32 GB SD card to hold the image. 
- Download the image JetsonNanoUb20_3b.img.xz (8.7 GByte!) from Sync. 
- Flash the image on the SD card with Imager.
- Flash the xz directly, not an unzipped img image. 
- Insert the SD card in your Jetson Nano. 
- Password: jetson

More details on: https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html

## Appendix C: Data Sheets
## References

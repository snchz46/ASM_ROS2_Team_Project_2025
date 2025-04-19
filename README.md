
# 🤖 ASM ROS2 TEAM PROJECT 2025
> A guide by Samuel Sanchez
##  ROS 2 Guide

Welcome to this practical and compact guide about ROS 2. Click on the sections below to navigate through each topic.

## Table of Contents

- [1️⃣ Abstract](01_abstract.md)
- [2️⃣ ROS 2 Introduction](02_ros_intro.md)
- [3️⃣ LiDAR Integration](03_lidar.md)
- [4️⃣ Stereo Camera Integration](04_stereo_cam.md)
- [5️⃣ Installation Guides](05_installations.md)


## Compatibility table

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

---

## References used on this guide

[ROS 2 Org.](https://docs.ros.org/en)

- [Open Robotics: Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

- [Open Robotics: Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)

[MATLAB Help-Center: ROS 2](https://de.mathworks.com/help/ros/ug/get-started-with-ros-2.html)

- [Get Started with ROS 2 in Simulink](https://de.mathworks.com/help/ros/ug/get-started-with-ros-2-in-simulink.html)

- [Generate a Standalone ROS Node from Simulink](https://de.mathworks.com/help/ros/ug/generate-a-standalone-ros-node-from-simulink.html)

- [Generate Code to Manually Deploy a ROS 2 Node from Simulink](https://de.mathworks.com/help/ros/ug/generate-code-to-manually-deploy-ros-2-node.html)

[Slamtec RPLIDAR A2](https://www.slamtec.com/en/Support#rplidar-a-series)

- [Slamtec RPLIDAR A2 ROS 2 Package](https://github.com/Slamtec/rplidar_ros/tree/ros2)

[NVIDIA Jetson Nano](https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit)

- [Q-engineering Installing Ubuntu on Jetson Nano](https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html) 

- [Q-engineering Further info about Jetson Nano](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image?tab=readme-ov-file#installation)

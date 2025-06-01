# Abstract
Robot Operating System 2 (ROS2) is an open-source middleware framework for developing distributed and modular robotic applications. It has native support for real-time communication, cross-platform compatibility, and easy interfacing with packages like Simulink and MATLAB.

This renders it a perfect middleware for prototyping and deployment of autonomous features in both academic and industrial environments. This research covers the integration and implementation of ROS2 nodes for fusing LiDAR and stereo camera sensors on a Jetson Orin Nano controller. 

The theoretical foundation underlying this project is part of a broader research project named Automated Driving in MIniaturized Traffic scenarios 1:14 (ADMIT14), which are intended to validate autonomous driving technology in a controlled, small-scale environment. In this study, we concentrate on creating a modular configuration, where Python Publishers transmit raw sensor data, and Python subscribers or MATLAB decode them at higher levels. This isolated configuration allows independent development, testing, and replacement of individual hardware components, showcasing the versatility that ROS2 provides.

# Introduction

Autonomous driving technologies essentially rely on the integration of various sensors and control components to perceive, interpret, and react to their environment. In complex systems such as autonomous cars, it is necessary to ensure that every subsystem can be developed, tested, and updated independently, which is crucial for scaling up and for the sake of long-term sustainability.

The ADMIT14 program, Automated Driving in MIniaturized Traffic environments scale 1:14, was started at Hochschule Esslingen with the vision of validating and testing autonomous functionality in a manageable miniature environment. Our work within this paradigm revolves around the realization of a sensor fusion architecture with ROS2, by means of LiDAR technology and stereo cameras on an NVIDIA Jetson platform.


---

🔝 [Index](README.md) | ➡️ [ROS 2 Introduction](02_ros_intro.md)

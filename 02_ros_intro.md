# ROS 2 Introduction

ROS 2 (Robot Operating System 2) is a flexible and modular open-source framework for developing robotic applications. It provides the tools, libraries, and conventions needed to build distributed systems where multiple components—called nodes—can communicate over well-defined interfaces.

Built to address the limitations of the original ROS, ROS 2 offers real-time capabilities, enhanced security, improved middleware abstraction, and native support for multi-platform systems (Linux, Windows, macOS). It is widely used in research, industrial automation, and autonomous systems.

This guide aims to introduce ROS 2 concepts and provide practical steps for installation, configuration, and development with real examples using sensors like LiDAR and integrations with MATLAB.

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
### Example of Nodes and Topics
#### Communication Between Two Devices on a Wired Local Network
This example demonstrates the communication between two devices connected through a wired local network using ROS 2. Each device runs different nodes and topics to test interoperability across the network.
On the Windows device on MATLAB, a node named NodePublisher_MATLAB is launched. It publishes messages to the topic /Hello_MATLAB, sending the string 'This is a MATLAB pub'. This setup verifies that nodes and topics created in MATLAB on Windows can be correctly published and recognized in the ROS 2 environment, both in a Windows terminal and Linux

![0dda7b462f995c3d1e234b4459595b49](https://github.com/user-attachments/assets/3e1cc6c1-4567-4534-8609-fb201829961f)

On the Linux device, several ROS 2 demos and applications are executed, including the Talker demo, the Turtlesim3 demo, and an RPLIDAR application. Each of these creates its own set of nodes and topics as part of their default operation, contributing to the network-wide ROS 2 graph.

![test](https://github.com/user-attachments/assets/da3d569a-8bac-47a9-9a8a-0589a1b8859d)


# Further Reading

[![ROS 2 Overview | MATLAB and Simulink Tutorial](https://img.youtube.com/vi/9EB0f3lVY-0/0.jpg)](https://www.youtube.com/watch?v=9EB0f3lVY-0&list=PLw9UeyR2OgE0tOF7uac1OEFiNwY7-9j5y)

[ROS 2 Org.](https://docs.ros.org/en)

- [Open Robotics: Nodes](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Topics/Understanding-ROS2-Topics.html)

- [Open Robotics: Topics](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Nodes/Understanding-ROS2-Nodes.html)

[MATLAB Help-Center: ROS 2](https://de.mathworks.com/help/ros/ug/get-started-with-ros-2.html)

- [Get Started with ROS 2 in Simulink](https://de.mathworks.com/help/ros/ug/get-started-with-ros-2-in-simulink.html)

- [Generate a Standalone ROS Node from Simulink](https://de.mathworks.com/help/ros/ug/generate-a-standalone-ros-node-from-simulink.html)

- [Generate Code to Manually Deploy a ROS 2 Node from Simulink](https://de.mathworks.com/help/ros/ug/generate-code-to-manually-deploy-ros-2-node.html)


---

⬅️ [Introduction](01_intro.md) | 🔝 [Index](README.md) | ➡️ [LiDAR Integration](03_lidar.md)

# ROS 2 Introduction

ROS2 is an open-source middleware framework that is tailored for the creation of distributed and modular robot applications. ROS2 extends the basic ROS environment with the inclusion of real-time communication, multi-platform runtimes, and enhanced security features. In the context of this project, ROS2 serves as the centerpiece of the communication system among the sensors (stereo camera and LiDAR), the processing module (Jetson Nano), and external modules (Arduino, MATLAB) with standardized topics and interfaces.

The publish-subscribe architecture allows for a loose coupling between data producers and consumers, thereby enabling easy integration and subsequent substitution of components with minimal impacts on the overall system.

One of the primary motivations for using ROS2 is because of its modularity, which promotes breaking up functionalities into independent nodes. Modularity renders both the development process and the adaptation of the system afterward straightforward. For instance, swapping a LiDAR sensor, updating the stereo camera, or changing the communication interfaces (i.e., from CAN to Ethernet) does not necessitate a redesign of the overall system. Rather, only the respective ROS2 node must be recompiled or adjusted.

Additionally, ROS2 has real-time communication, platform independence, and seamless interfacing with Simulink and MATLAB tools built in. All these make it an ideal middleware candidate for the development and deployment of autonomous capabilities in academic and industrial environments. The remainder of this paper is organized as follows: Section 2 covers the development challenges faced. Section 3 presents the system architecture. Section 4 reports the implementation details. Section 5 gives the experimental results, and Section 6 concludes the paper with future research directions.

## ROS 2 Compatibility table

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

---

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

⬅️ [Introduction](01_intro.md) | 🔝 [Index](README.md) | ➡️ [ROS 2 Installation](03_ros2_install.md)

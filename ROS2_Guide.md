
















ROS2 Introdoction
ROS 2: Nodes and Topics
In ROS 2, a node is a basic building block responsible for a specific, modular function within a robotic system. For example, one node might control wheel motors, while another might publish sensor data from a LiDAR. A complete robotic system is typically made up of many nodes working together, each performing a distinct role.
Nodes communicate with each other through several mechanisms: topics, services, actions, and parameters. Among these, topics are the most common and flexible method for data exchange.
A full robotic system is comprised of many nodes working together.

Source: 
A topic acts like a data bus that allows nodes to publish and subscribe to messages. One node can publish information on a topic, and multiple other nodes can subscribe to that topic to receive the data in real-time. Similarly, a node can publish multiple topics and subscribe to several others at the same time, enabling complex and dynamic data flows.
Topics are an essential part of the ROS 2 graph, enabling modular, distributed systems where components can be developed and run independently while still interacting seamlessly.

Source: 
Example of Nodes and Topics
Communication Between Two Devices on a Wired Local Network
This example demonstrates the communication between two devices connected through a wired local network using ROS 2. Each device runs different nodes and topics to test interoperability across the network.
On the Windows device on MATLAB, a node named NodePublisher\_MATLAB is launched. It publishes messages to the topic /Hello\_MATLAB, sending the string 'This is a MATLAB pub'. This setup verifies that nodes and topics created in MATLAB on Windows can be correctly published and recognized in the ROS 2 environment, both in a Windows terminal and Linux

On the Linux device, several ROS 2 demos and applications are executed, including the Talker demo, the Turtlesim3 demo, and an RPLIDAR application. Each of these creates its own set of nodes and topics as part of their default operation, contributing to the network-wide ROS 2 graph.


Implementing LiDAR (ubuntu)

Downloads: 
ROS2 Package Full guide: 

mkdir -p ~/ros2\_ws/src
cd ~/ros2\_ws/src
git clone -b ros2 
cd ~/ros2\_ws/
source /opt/ros//setup.bash
colcon build --symlink-install
source ./install/setup.bash
sudo chmod 777 /dev/ttyUSB0
ros2 launch rplidar\_ros view\_rplidar\_a2m8\_launch.py













Appendix A: Tables
Compatibility table ROS2




















Appendix B: Installations
Installing ROS2 on Ubuntu 20.04.6 on NVIDIA Jetson Nano
 
Jetson Nano with Ubuntu 20.04 OS image
Installation:
Get a >32 GB SD card to hold the image. 
Download the image JetsonNanoUb20\_3b.img.xz (8.7 GByte!) from . 
Flash the image on the SD card with .
Flash the xz directly, not an unzipped img image. 
Insert the SD card in your Jetson Nano. 
Password: jetson
More details on .
ROS2 Foxy Installation
After the successful installation of Ubuntu, open a terminal and time the following commands:
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add –
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb\_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-foxy-desktop
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
Installing ROS2 on Windows and macOS
Follow this tutorial, only for W10+ devices:

For macOS refer to this other tutorial:


After successfully followed all the steps on the guides, open x64 Native Tools Command Prompt for VS 2022 command line console and type:
call "C:\dev\ros2\_humble\setup.bat" 
with your own path to the setup.bat file. After that, if everything worked well, running any ros2 command will work, the environment will be set up, for example:
ros2
The call mentioned above must be run every time a new console is opened, it is recommended to create a launch file where this command Is already run, so every time the launch file is opened there is no need to call the setup file

Appendix C: Data Scheets
RPLIDAR SLAMTEC A2M8

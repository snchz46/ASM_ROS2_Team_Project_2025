# Installing ROS2 on Ubuntu 20.04.6 on NVIDIA Jetson Nano

![image](https://github.com/user-attachments/assets/c0bb7570-e373-4814-b670-85ec30277220)

## Jetson Nano with Ubuntu 20.04 OS image

[Installation ](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image?tab=readme-ov-file#installation):

- Get a >32 GB SD card to hold the image. 
- Download the image JetsonNanoUb20_3b.img.xz (8.7 GByte!) from Sync. 
- Flash the image on the SD card with Imager.
- Flash the xz directly, not an unzipped img image. 
- Insert the SD card in your Jetson Nano. 
- Password: jetson

More details on: https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html

## ROS2 Foxy Installation
After the successful installation of Ubuntu, open a terminal and time the following commands:

````bash
sudo apt update && sudo apt install curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add –
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update
sudo apt install ros-foxy-desktop
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python3-colcon-common-extensions python3-rosdep
sudo rosdep init
rosdep update
````

# Installing ROS2 on Windows and macOS

Follow this [tutorial](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html), only for W10+ devices

For macOS refer to this other [tutorial](https://docs.ros.org/en/humble/Installation/Alternatives/macOS-Development-Setup.html).

After successfully followed all the steps on the guides, open x64 Native Tools Command Prompt for VS 2022 command line console and type:
````bash
call "C:\dev\ros2_humble\setup.bat"
````

with your own path to the setup.bat file. After that, if everything worked well, running any ros2 command will work, the environment will be set up, for example:
````bash
ros2
````
The call mentioned above must be run every time a new console is opened, it is recommended to create a launch file where this command Is already run, so every time the launch file is opened there is no need to call the setup file

---

⬅️ [Stereo Camera Integration](04_stereo_cam.md) | 🔝 [Index](README.md)


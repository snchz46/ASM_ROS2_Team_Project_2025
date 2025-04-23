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
---

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

# ROS 2 Python Package Setup Guide (Ubuntu 22.04 + ROS 2 Humble)

This guide shows how to turn your stand‑alone Python scripts (e.g. LiDAR obstacle avoidance for **Turtlesim**) into a proper ROS 2 package inside a workspace called **`ros2_ws`**.


## Prerequisites

* Ubuntu 22.04 with **ROS 2 Humble** (or newer) installed.  
* Basic familiarity with the terminal and Git.

> **Tip** – All shell snippets below are copy‑ready: just click the copy button and paste into your terminal.


## 1  Create / locate the workspace

```bash
# Create workspace skeleton if it does not exist
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```

## 2  Generate a Python package

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python my_pkg_name
```

* `my_pkg_name` becomes the folder and package name.  
* ROS automatically creates `setup.py`, `package.xml`, and a module folder `my_pkg_name/`.

## 3  Add your scripts

Copy any Python nodes (e.g. `my_script.py`) into the **inner** package folder:

```bash
cp /path/to/my_script.py ~/ros2_ws/src/my_pkg_name/my_pkg_name/
```

Ensure each script:

```python
#!/usr/bin/env python3
# …your imports…

def main():
    # entry‑point code
    pass

if __name__ == "__main__":
    main()
```

Give execute permission (optional but handy):

```bash
chmod +x ~/ros2_ws/src/my_pkg_name/my_pkg_name/my_script.py
```

## 4  Expose entry‑points in `setup.py`

Open **`~/ros2_ws/src/my_pkg_name/setup.py`** and locate the `entry_points` block.  Add one line per script:

```python
entry_points={
    'console_scripts': [
        'console_call_name = my_pkg_name.my_script:main',
        # 'another_node = my_pkg_name.another_node:main',
    ],
},
```

> **Why?**  `ros2 run my_pkg_name console_call_name` will map to the `main()` function inside `my_script.py`.

## 5  Declare dependencies in `package.xml`

Edit **`~/ros2_ws/src/my_pkg_name/package.xml`** and add the runtime deps inside `<exec_depend>` (keep existing ones):

```xml
<exec_depend>import_1</exec_depend>
<exec_depend>import_2</exec_depend>
<exec_depend>import_3</exec_depend>
```

Also keep the default `<buildtool_depend>ament_python</buildtool_depend>`.

## 6  Build the workspace

```bash
cd ~/ros2_ws
colcon build --packages-select my_pkg_name --symlink-install
```

* `--symlink-install` allows live editing of Python files without rebuilding.

## 7  Source the environment

```bash
# Source ROS 2 installation
source /opt/ros/humble/setup.bash
# Source your freshly built workspace
source ~/ros2_ws/install/setup.bash
```

Add the second line to `~/.bashrc` so it loads automatically:

```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

## 8  Run your node

```bash
ros2 run my_pkg_name console_call_name
```

You should see logs from your script and, for the Turtlesim demo, the turtle will move accordingly.



---

⬅️ [Stereo Camera Integration](04_stereo_cam.md) | 🔝 [Index](README.md)


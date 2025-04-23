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
# 0  Refresh APT & essential tools
sudo apt update && sudo apt install curl gnupg lsb-release

# 1  Add ROS 2 GPG key and repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add –
sudo sh -c 'echo "deb http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
sudo apt update

# 2  Install desktop variant (∼1 GB)
sudo apt install ros-foxy-desktop

# 3  Environment setup (auto‑source on every shell)
echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc
source ~/.bashrc

# 4  Developer tools
sudo apt install python3-colcon-common-extensions python3-rosdep

sudo rosdep init
rosdep update
````

---

# ROS 2 Humble on Windows 10/11

Install VS 2022 Build Tools + CMake (if not already).

Follow the official step‑by‑step binary guide → https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html

After finishing the guide, open x64 Native Tools Command Prompt for VS 2022 and run:

````bash
call "C:\dev\ros2_humble\local_setup.bat"  # adjust to your path
````
Add that call line to a ros2_env.bat file on your desktop so you can double‑click and start a pre‑sourced shell anytime.

Verify:
````bash
ros2 --help
````
If it prints the CLI help, your environment is good to go.

---

# ROS 2 Humble on macOS (Apple Silicon & Intel)

Official doc → https://docs.ros.org/en/humble/Installation/Alternatives/macOS-Development-Setup.html

Cheat‑sheet highlights:

````bash
# Homebrew prereqs
brew install python@3.11 cmake qt @llvm dev tools

# Clone the ros2 repo list & build (takes a while!)
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
vcs import src < https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos

# Resolve dependencies
rosdep init  # first‑time only (sudo)
rosdep update
rosdep install --from-paths src --ignore-src -y --rosdistro humble

# Build
colcon build --symlink-install

# Source overlay
source install/setup.zsh  # use .bash if you prefer bash
````
Add the source line to your shell RC (.zshrc / .bash_profile) for convenience.

---

# How to create ROS2 packages

This guide shows how to turn your stand‑alone Python scripts into a proper ROS 2 package inside a workspace called **`ros2_ws`**. Only valid for Ubuntu 22.04 + ROS 2 Humble and python scripts, other versions not checked.

## 1  Create / locate the workspace

```bash
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


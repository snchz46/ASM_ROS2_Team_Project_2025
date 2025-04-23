# ROS 2 Workspace Quick‑Start


## Prerequisites  
* ROS 2 Humble (or newer) already installed on the host.  
* Python 3 and *colcon* (comes with most ROS 2 desktop installs).

---

## Ubuntu 22.04

Install extra colcon helpers (optional but handy)
```bash
sudo apt update && sudo apt install -y python3-colcon-common-extensions
```
Create workspace skeleton
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```
Build (even if src is empty, this lays out install/ & build/)
```bash
colcon build --symlink-install
```
Source the workspace for this shell
```bash
source install/setup.bash
```
(Optional) Auto‑source for every new shell
```bash
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Windows 10 / 11 (PowerShell)

Open a ROS 2 Developer Command Prompt  (or run `ros2` env script manually).
Create workspace
```bash
md C:\ros2_ws\src
cd C:\ros2_ws
```
Build
```bash
colcon build --merge-install
```
Set environment for this session
```bash
.\install\local_setup.ps1  # dot‑source
```
*(For traditional CMD use `install\local_setup.bat` instead.)*

---

## macOS (Apple Silicon & Intel)

Ensure ROS 2 environment is sourced (e.g. via /opt/ros/humble/setup.bash)
Create workspace
```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws
```
Build
```bash
colcon build --symlink-install
```
Source workspace
```bash
source install/setup.bash
```
(Option) add to ~/.zshrc or ~/.bash_profile


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

## 8  Run your package

```bash
ros2 run my_pkg_name console_call_name
```

---

⬅️ [Stereo Camera Integration](04_stereo_cam.md) | 🔝 [Index](README.md)


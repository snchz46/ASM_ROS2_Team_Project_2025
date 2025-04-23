# ROS 2 Workspace Quick‑Start

> Minimal steps to create and initialise an *empty* ROS 2 workspace on **Ubuntu 22.04**, **Windows 10/11**, or **macOS 12+**.

---

## Prerequisites  
* ROS 2 Humble (or newer) already installed on the host.  
* Python 3 and *colcon* (comes with most ROS 2 desktop installs).

---

## Ubuntu 22.04
```bash
# 1  Install extra colcon helpers (optional but handy)
sudo apt update && sudo apt install -y python3-colcon-common-extensions

# 2  Create workspace skeleton
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 3  Build (even if src is empty, this lays out install/ & build/)
colcon build --symlink-install

# 4  Source the workspace for this shell
source install/setup.bash

# 5  (Optional) Auto‑source for every new shell
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
```

---

## Windows 10 / 11 (PowerShell)
```powershell
# 1  Open a ROS 2 Developer Command Prompt  (or run `ros2` env script manually)

# 2  Create workspace
md C:\ros2_ws\src
cd C:\ros2_ws

# 3  Build
colcon build --merge-install

# 4  Set environment for this session
.\install\local_setup.ps1  # dot‑source
```
*(For traditional CMD use `install\local_setup.bat` instead.)*

---

## macOS (Apple Silicon & Intel)
```bash
# 1  Ensure ROS 2 environment is sourced (e.g. via /opt/ros/humble/setup.bash)

# 2  Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws

# 3  Build
colcon build --symlink-install

# 4  Source workspace
source install/setup.bash

# 5  (Option) add to ~/.zshrc or ~/.bash_profile
```

---

## Next Steps
* Add a **package** inside `src/` (`ros2 pkg create …`).  
* Re‑run `colcon build` whenever you add or modify packages.  
* Always source `install/setup.*` before running `ros2` commands.

Happy coding 🚀


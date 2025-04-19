
# 🤖 ASM ROS2 TEAM PROJECT 2025
##  ROS 2 Guide

Welcome to this practical and compact guide about ROS 2. Click on the sections below to navigate through each topic.

## Table of Contents

- [1️⃣ Introduction](01_intro.md)
- [2️⃣ Nodes and Topics](02_nodes_topics.md)
- [3️⃣ LiDAR Integration (Ubuntu)](03_lidar.md)
- [4️⃣ Compatibility Table](04_compatibility.md)
- [5️⃣ Installation Guides](05_install.md)
- [6️⃣ Data Sheets](06_datasheet.md)
- [7️⃣ References](07_references.md)

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



### Installing ROS2 on Ubuntu 20.04.6 on NVIDIA Jetson Nano

![image](https://github.com/user-attachments/assets/c0bb7570-e373-4814-b670-85ec30277220)

### Jetson Nano with Ubuntu 20.04 OS image

[Installation ](https://github.com/Qengineering/Jetson-Nano-Ubuntu-20-image?tab=readme-ov-file#installation):

- Get a >32 GB SD card to hold the image. 
- Download the image JetsonNanoUb20_3b.img.xz (8.7 GByte!) from Sync. 
- Flash the image on the SD card with Imager.
- Flash the xz directly, not an unzipped img image. 
- Insert the SD card in your Jetson Nano. 
- Password: jetson

More details on: https://qengineering.eu/install-ubuntu-20.04-on-jetson-nano.html

## Appendix C: Data Sheets
## References

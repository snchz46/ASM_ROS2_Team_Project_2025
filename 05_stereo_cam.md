# Implementing Stereo Camera on ROS2


![image](https://github.com/user-attachments/assets/25b26369-02e4-43c2-8a25-182d475c4171)

Documents, packages and tools: [https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera](https://www.waveshare.com/wiki/IMX219-83_Stereo_Camera)

## Publishing image data from the Stereo Camera into the ROS2 network

Two different scripts were developed, one for rapid debugging, using a regular USB webcam connected to a Windows environment and another for the targeted HW, the Waveshare IMX219 Stereo_Camera, connected on the Jetson Nano.

- [Stereo Camera Publisher](jetson_stereo_cam_pub.py)

- [Windows Camera Publisher](windows_cam_pub.py)

## Retrieving image data from the Camera via Python

- [Stereo Camera Subscriber w/ YOLO V8](jetson_yolov8_cam_sub.py)

- [Windows Camera Subscriber w/ YOLO V8](windows_yolov8_cam_sub.py)

- [Windows Camera Subscriber](windows_cam_sub.py)

Screenshot for [Windows Camera Subscriber w/ YOLO V8](windows_yolov8_cam_sub.py)

![python_image_subscriber](https://github.com/user-attachments/assets/4dfd5280-882e-4df7-80f2-9268f2333d37)

## Retrieving image data from the Camera via MATLAB

Screenshot for [MATLAB Stereo Camera Subscriber](matlab_cam_sub.m)

![matlab_image_subscriber](https://github.com/user-attachments/assets/4286b8b6-0be8-4bfb-befd-44d1431a7ab7)

---

⬅️ [ROS 2 Setup](04_ros2_setup.md) | 🔝 [Index](README.md) | ➡️ [LiDAR Implementation](06_lidar.md)

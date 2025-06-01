#!/usr/bin/env python3
#
# image_view_detect.py
#
# Subscribes to /camera/image_raw, runs YOLOv8, and shows the frame
# with boxes + labels.  Needs:
#   pip install ultralytics opencv-python rclpy sensor_msgs numpy
#
# If you have a GPU and CUDA/cuDNN installed, YOLOv8 will use it
# automatically; otherwise it’ll fall back to CPU (a bit slower).

import rclpy, cv2, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO    # ultralytics >= 8.1.0

class Viewer(Node):
    def __init__(self):
        super().__init__('viewer_detect')

        # Load the teeny-tiny YOLOv8 model (n = nano, fastest)
        self.model = YOLO('yolov8n.pt')       # first time triggers an auto-download

        # Pick a nicer palette than the default YOLO one
        self.palette = np.random.randint(0, 255, size=(80, 3), dtype=np.uint8)

        self.sub = self.create_subscription(
            Image, '/camera/image_raw', self.cb, 10)

    def cb(self, msg: Image):
        # --- ROS2 Image -> numpy (BGR) ---------------------------------------
        img = np.frombuffer(msg.data, dtype=np.uint8)\
                .reshape(msg.height, msg.width, 3)

        # --- Run detector (expects RGB) --------------------------------------
        results = self.model(img[..., ::-1], verbose=False)[0]   # first elem of list

        # --- Draw detections --------------------------------------------------
        for box, cls, conf in zip(results.boxes.xyxy,
                                  results.boxes.cls,
                                  results.boxes.conf):
            x1, y1, x2, y2 = map(int, box)
            c = int(cls)
            label = f'{self.model.names[c]} {conf:.2f}'
            color = tuple(int(v) for v in self.palette[c])

            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            (tw, th), _ = cv2.getTextSize(label,
                                          cv2.FONT_HERSHEY_SIMPLEX,
                                          0.5, 1)
            cv2.rectangle(img, (x1, y1 - th - 4),
                               (x1 + tw, y1), color, -1)
            cv2.putText(img, label,
                        (x1, y1 - 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1, cv2.LINE_AA)

        # --- Show -------------------------------------------------------------
        cv2.imshow('cam + YOLOv8', img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    rclpy.spin(Viewer())

if __name__ == '__main__':
    main()

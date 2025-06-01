#!/usr/bin/env python3
"""
stereo_viewer_detect.py

Subscribes to:
    /stereo/left/image_raw
    /stereo/right/image_raw

• Shows the two streams side-by-side.  
• Runs YOLOv8-nano on the LEFT frame.  
• Boxes + labels are painted on the left half only.

Dependencies
------------
pip install rclpy opencv-python numpy sensor_msgs ultralytics
If you have CUDA on the Jetson the model will auto-use it; otherwise falls back to CPU.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


class StereoViewerDetect(Node):
    def __init__(self) -> None:
        super().__init__('stereo_viewer_detect')

        # Latest frames
        self.left_img = None
        self.right_img = None

        # Subscribe to both eyes
        self.create_subscription(Image,
                                 '/stereo/left/image_raw',
                                 self.left_cb,  10)
        self.create_subscription(Image,
                                 '/stereo/right/image_raw',
                                 self.right_cb, 10)

        # Load the tiny YOLOv8 model once
        self.model = YOLO('yolov8n.pt')      # downloads on first run
        self.palette = np.random.randint(0, 255, size=(80, 3), dtype=np.uint8)

        # UI refresh / inference rate (Hz)
        self.timer = self.create_timer(15 ** -1, self.draw)   # ~15 Hz

    # ---------- ROS callbacks ----------
    def left_cb(self, msg: Image) -> None:
        self.left_img = self.msg_to_cv(msg)

    def right_cb(self, msg: Image) -> None:
        self.right_img = self.msg_to_cv(msg)

    # ---------- helpers ----------
    @staticmethod
    def msg_to_cv(msg: Image) -> np.ndarray:
        img = np.frombuffer(msg.data, dtype=np.uint8)
        return img.reshape(msg.height, msg.width, 3)

    def annotate(self, img: np.ndarray) -> np.ndarray:
        """Run YOLO and draw boxes on a BGR frame."""
        res = self.model(img[..., ::-1], verbose=False)[0]    # expects RGB

        for box, cls, conf in zip(res.boxes.xyxy,
                                  res.boxes.cls,
                                  res.boxes.conf):
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

        return img

    # ---------- UI loop ----------
    def draw(self) -> None:
        if self.left_img is None or self.right_img is None:
            return

        # Ensure identical crop
        h = min(self.left_img.shape[0], self.right_img.shape[0])
        w = min(self.left_img.shape[1], self.right_img.shape[1])
        l = self.left_img [:h, :w].copy()    # copy because we draw on it
        r = self.right_img[:h, :w]

        # Object detection on left image
        l = self.annotate(l)

        # Show
        cv2.imshow('Stereo + YOLO (left)', np.hstack((l, r)))
        cv2.waitKey(1)

    # ---------- cleanup ----------
    def destroy_node(self) -> None:
        cv2.destroyAllWindows()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = StereoViewerDetect()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

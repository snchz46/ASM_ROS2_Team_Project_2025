#!/usr/bin/env python3
# left_eye_viewer_detect_flip.py  ── imagen al derecho + cajas al derecho

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO


class LeftEyeViewerFlip(Node):
    def __init__(self):
        super().__init__('right_eye_viewer_detect_flip')

        self.model   = YOLO('yolov8n.pt')
        self.palette = np.random.randint(0, 255, (80, 3), dtype=np.uint8)

        self.create_subscription(Image,
                                 '/right/image_raw',
                                 self.cb,
                                 10)

        self.frame = None
        self.timer = self.create_timer(1 / 30, self.draw)   # 30 Hz

    # ---------- callback ----------
    def cb(self, msg: Image):
        img = np.frombuffer(msg.data, dtype=np.uint8)\
                .reshape(msg.height, msg.width, 3)
        self.frame = img   

    # ---------- detección ----------
    def annotate(self, img):
        res = self.model(img[..., ::-1], verbose=False)[0]   # BGR→RGB
        for xyxy, cls, conf in zip(res.boxes.xyxy,
                                   res.boxes.cls,
                                   res.boxes.conf):
            x1, y1, x2, y2 = map(int, xyxy)
            c     = int(cls)
            color = tuple(int(v) for v in self.palette[c])
            label = f'{self.model.names[c]} {conf:.2f}'

            cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
            (tw, th), _ = cv2.getTextSize(label,
                                          cv2.FONT_HERSHEY_SIMPLEX,
                                          0.5, 1)
            cv2.rectangle(img, (x1, y1 - th - 4), (x1 + tw, y1), color, -1)
            cv2.putText(img, label, (x1, y1 - 2),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (255, 255, 255), 1, cv2.LINE_AA)
        return img

    # ---------- UI loop ----------
    def draw(self):
        if self.frame is None:
            return

        img = cv2.flip(self.frame, 1)   
        img = self.annotate(img)        
        img = cv2.resize(img, None, fx=0.5, fy=0.5,
                 interpolation=cv2.INTER_AREA)

        cv2.imshow('Right eye upright + YOLO', img)
        cv2.waitKey(1)

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main():
    rclpy.init()
    node = LeftEyeViewerFlip()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

import rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np

class Viewer(Node):
    def __init__(self):
        super().__init__('viewer')
        self.create_subscription(Image, '/camera/image_raw', self.cb, 10)

    def cb(self, msg):
        img = np.frombuffer(msg.data, dtype=np.uint8)\
                .reshape(msg.height, msg.width, 3)
        cv2.imshow('cam', img); cv2.waitKey(1)

rclpy.init(); rclpy.spin(Viewer())

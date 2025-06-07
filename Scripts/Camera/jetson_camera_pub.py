#!/usr/bin/env python3
"""
Node ROS 2 minimalist: captures IMX219 via GStreamer and publishes sensor_msgs/Image
Uso:
  python3 cam_pub.py --id 0 --topic /left/image_raw --frame left_camera
  python3 cam_pub.py --id 1 --topic /right/image_raw --frame right_camera
"""
import argparse, rclpy, cv2, numpy as np
from rclpy.node import Node
from sensor_msgs.msg import Image
from builtin_interfaces.msg import Time

PIPE_TPL = ("nvarguscamerasrc sensor-id={id} ! "
            "video/x-raw(memory:NVMM),width=640,height=360,framerate=30/1 ! "
            "nvvidconv ! video/x-raw,format=BGRx ! "
            "videoconvert ! video/x-raw,format=BGR ! appsink drop=true sync=false")

class CamNode(Node):
    def __init__(self, sensor_id: int, topic: str, frame_id: str):
        super().__init__(f"cam_{sensor_id}")
        pipe = PIPE_TPL.format(id=sensor_id)
        self.cap = cv2.VideoCapture(pipe, cv2.CAP_GSTREAMER)
        if not self.cap.isOpened():
            self.get_logger().error(f"Cannot open sensor-id {sensor_id}")
            raise SystemExit
        self.pub = self.create_publisher(Image, topic, 10)
        self.frame_id = frame_id
        self.timer = self.create_timer(1/30, self.loop)  # ~30 Hz
        self.get_logger().info(f"Publishing {topic} from sensor-id {sensor_id}")

    def loop(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warning("Frame plost"); return
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.pub.publish(msg)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--id',    type=int, required=True, help='sensor-id (0 or 1)')
    parser.add_argument('--topic', type=str, required=True, help='topic ROS to pusblish')
    parser.add_argument('--frame', type=str, required=True, help='frame_id')
    args = parser.parse_args()

    rclpy.init()
    node = CamNode(args.id, args.topic, args.frame)
    rclpy.spin(node)

if __name__ == '__main__':
    main()

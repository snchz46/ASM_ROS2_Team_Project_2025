#!/usr/bin/env python3
"""
ROS 2 minimal webcam publisher for Windows.
Usage (defaults are fine for a single cam):

  # plain Python
  python test_camera_pub_W.py

  # or the ROS 2 way
  ros2 run webcam_publisher test_camera_pub_win

Extra CLI goodies if you ever plug in more cams:

  python cam_pub_win.py --id 1 --topic /second/image_raw --frame second_cam
"""
import argparse, rclpy, cv2
from rclpy.node import Node
from sensor_msgs.msg import Image

class CamNode(Node):
    def __init__(self, device_index: int, topic: str, frame_id: str):
        # device_index = int(0)   # 0,1,2... for multiple cameras
        super().__init__(f"cam_{device_index}")

        # On Windows CAP_DSHOW is the most reliable backend for USB cams.
        self.cap = cv2.VideoCapture(device_index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            self.get_logger().error(f"No camera detected {device_index}")
            raise SystemExit

        # Optional: lock resolution (comment out if you don’t care)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1280)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

        self.pub = self.create_publisher(Image, topic, 10)
        self.frame_id = frame_id
        self.timer = self.create_timer(1/30, self.loop)  # ~30 Hz
        self.get_logger().info(f"Publishing {topic} from camera device:  {device_index}")

    def loop(self):
        ok, frame = self.cap.read()
        if not ok:
            self.get_logger().warning("Frame lost"); return

        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        self.pub.publish(msg)

    def destroy_node(self):
        if self.cap.isOpened():
            self.cap.release()
        super().destroy_node()

def main():
    parser = argparse.ArgumentParser(description="Publish a single Windows webcam to ROS 2")
    parser.add_argument('--id',    type=int, default=0, help='Device index (0 = first webcam)')
    parser.add_argument('--topic', type=str, default='/camera/image_raw', help='ROS topic')
    parser.add_argument('--frame', type=str, default='webcam', help='TF frame_id')
    args = parser.parse_args()

    rclpy.init()
    node = CamNode(args.id, args.topic, args.frame)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

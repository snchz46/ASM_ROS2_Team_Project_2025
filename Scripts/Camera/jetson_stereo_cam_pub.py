#!/usr/bin/env python3
"""
ROS 2 publisher for the Jetson Nano dual-IMX219 stereo camera.

Topics:
    /stereo/left/image_raw
    /stereo/right/image_raw

Works with the stock JetPack GStreamer stack (nvarguscamerasrc).
"""

import argparse
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


def gstreamer_pipeline(sensor_id=0, w=1280, h=720, fps=30, flip=0):
    """Return a GStreamer pipeline string for nvarguscamerasrc."""
    return (
        f"nvarguscamerasrc sensor-id={sensor_id} ! "
        f"video/x-raw(memory:NVMM), width={w}, height={h}, "
        f"format=NV12, framerate={fps}/1 ! "
        f"nvvidconv flip-method={flip} ! "
        f"video/x-raw, format=BGRx ! "
        f"videoconvert ! "
        f"video/x-raw, format=BGR ! appsink drop=true"
    )


class StereoPublisher(Node):
    def __init__(self, args) -> None:
        super().__init__('stereo_publisher')

        # two image publishers
        self.left_pub = self.create_publisher(Image, 'stereo/left/image_raw', 10)
        self.right_pub = self.create_publisher(Image, 'stereo/right/image_raw', 10)

        # open both sensors
        self.left_cap = cv2.VideoCapture(
            gstreamer_pipeline(sensor_id=args.left_id, w=args.w, h=args.h,
                               fps=args.fps, flip=args.flip),
            cv2.CAP_GSTREAMER)
        self.right_cap = cv2.VideoCapture(
            gstreamer_pipeline(sensor_id=args.right_id, w=args.w, h=args.h,
                               fps=args.fps, flip=args.flip),
            cv2.CAP_GSTREAMER)

        if not (self.left_cap.isOpened() and self.right_cap.isOpened()):
            self.get_logger().error('Could not open one or both IMX219 sensors')
            raise SystemExit

        # sample period comes from the requested FPS
        self.timer = self.create_timer(1.0 / args.fps, self.cb)

        self.frame_id_left = 'stereo_left'
        self.frame_id_right = 'stereo_right'

    # ---------- helpers --------------------------------------------------
    def cv2_to_msg(self, frame, stamp, frame_id):
        msg = Image()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.height, msg.width = frame.shape[:2]
        msg.encoding = 'bgr8'
        msg.step = msg.width * 3
        msg.data = frame.tobytes()
        return msg

    # ---------- main loop -------------------------------------------------
    def cb(self):
        ok_l, frame_l = self.left_cap.read()
        ok_r, frame_r = self.right_cap.read()
        if not (ok_l and ok_r):
            self.get_logger().warning('Frame grab failed')
            return

        now = self.get_clock().now().to_msg()
        self.left_pub.publish(self.cv2_to_msg(frame_l, now, self.frame_id_left))
        self.right_pub.publish(self.cv2_to_msg(frame_r, now, self.frame_id_right))

    # ---------- cleanup ---------------------------------------------------
    def destroy_node(self) -> None:
        for cap in (self.left_cap, self.right_cap):
            if cap.isOpened():
                cap.release()
        super().destroy_node()


# -------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description='Jetson Nano IMX219 stereo ROS 2 publisher')
    parser.add_argument('--left-id',  type=int, default=0,
                        help='nvarguscamerasrc sensor-id for the left cam')
    parser.add_argument('--right-id', type=int, default=1,
                        help='nvarguscamerasrc sensor-id for the right cam')
    parser.add_argument('--w',   type=int, default=1280, help='width')
    parser.add_argument('--h',   type=int, default=720,  help='height')
    parser.add_argument('--fps', type=int, default=30,   help='frames per second')
    parser.add_argument('--flip', type=int, default=0,
                        help='nvvidconv flip-method (0-7)')
    args, _ = parser.parse_known_args()

    rclpy.init()
    node = StereoPublisher(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

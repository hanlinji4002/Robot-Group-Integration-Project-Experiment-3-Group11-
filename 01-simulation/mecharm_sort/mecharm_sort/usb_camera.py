#!/usr/bin/env python3
# 【讲解】USB 相机节点（真机用）：OpenCV 打开 /dev/videoN，按设定分辨率/帧率抓图，发布 sensor_msgs/Image（bgr8）。
# 不依赖 v4l2_camera/usb_cam 这些 ROS 包（Jetson 上没装，也不需要联网装）。相机是 Realtek 0bda:3035，MJPG 最高 2592×1944，
# 默认用 1280×720@15，够识别和 ArUco 定位用。相机不需要标定：网格定位靠画面里的 ArUco 码。
"""USB 相机节点：VideoCapture → /camera/image_raw。"""
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class UsbCamera(Node):
    def __init__(self):
        super().__init__('usb_camera')
        for name, default in [
            ('device', 0), ('width', 1280), ('height', 720), ('fps', 15.0), ('fourcc', 'MJPG'),
            ('rotate_180', False), ('frame_id', 'camera'), ('image_topic', '/camera/image_raw'),
        ]:
            self.declare_parameter(name, default)
        g = lambda n: self.get_parameter(n).value
        self.cap = cv2.VideoCapture(int(g('device')), cv2.CAP_V4L2)
        fourcc = str(g('fourcc'))
        if len(fourcc) == 4:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(g('width')))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(g('height')))
        self.cap.set(cv2.CAP_PROP_FPS, float(g('fps')))
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if not self.cap.isOpened():
            raise RuntimeError(f"打不开相机 /dev/video{g('device')}")
        self.rotate = bool(g('rotate_180'))
        self.frame_id = str(g('frame_id'))
        self.bridge = CvBridge()
        self.pub = self.create_publisher(Image, g('image_topic'), 5)
        # 【讲解】相机按自己的帧率（多为 30 fps）不停出帧，如果只按发布节奏去 read()，V4L2 队列里会积压几帧，
        # 画面就"慢半拍"。这里单开一个线程不停 read() 把队列吃空，只留最新一帧，发布定时器拿最新帧发出去
        self.latest, self.lock, self.alive = None, threading.Lock(), True
        threading.Thread(target=self._capture_loop, daemon=True).start()
        self.timer = self.create_timer(1.0 / float(g('fps')), self._tick)
        self.n, self.t0 = 0, time.time()
        w, h = self.cap.get(cv2.CAP_PROP_FRAME_WIDTH), self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
        self.get_logger().info(f"相机 /dev/video{g('device')} 已打开：{int(w)}x{int(h)} @ {g('fps')} fps → {g('image_topic')}")

    def _capture_loop(self):
        fails = 0
        while self.alive:
            ok, frame = self.cap.read()
            if not ok:
                fails += 1
                if fails % 50 == 1:
                    self.get_logger().warn('读不到相机帧')
                time.sleep(0.02)
                continue
            fails = 0
            with self.lock:
                self.latest = frame

    def _tick(self):
        with self.lock:
            frame, self.latest = self.latest, None
        if frame is None:
            return
        if self.rotate:
            frame = cv2.rotate(frame, cv2.ROTATE_180)
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        self.pub.publish(msg)
        self.n += 1
        if self.n % 300 == 0:
            self.get_logger().info(f'已发布 {self.n} 帧，平均 {self.n / (time.time() - self.t0):.1f} fps')

    def destroy_node(self):
        self.alive = False
        time.sleep(0.1)
        self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = UsbCamera()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()

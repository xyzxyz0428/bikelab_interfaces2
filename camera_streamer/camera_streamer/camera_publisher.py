#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
import array

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CompressedImage


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # 参数
        self.declare_parameter('device_id', 0)
        self.declare_parameter('frame_rate', 30.0)
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('frame_id', 'camera_frame')

        self.device_id = int(self.get_parameter('device_id').value)
        self.frame_rate = float(self.get_parameter('frame_rate').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        qos = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        # 两个发布器：raw + compressed
        self.pub_raw = self.create_publisher(Image, '/camera/image_raw', qos)
        self.pub_compressed = self.create_publisher(CompressedImage, '/camera/image_compressed', qos)

        # ---- 打开摄像头（V4L2 + MJPG）----
        self.cap = cv2.VideoCapture(self.device_id, cv2.CAP_V4L2)

        fourcc = cv2.VideoWriter_fourcc(*'MJPG')
        self.cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        self.cap.set(cv2.CAP_PROP_FPS, self.frame_rate)

        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {self.device_id}')
            raise RuntimeError('Cannot open camera')

        # 保存最新帧（BGR）
        self._lock = threading.Lock()
        self._latest = None
        self._running = True

        # 采集线程
        self._grab_cnt = 0
        self._grab_t0 = time.time()
        self._grab_thread = threading.Thread(target=self._grab_loop, daemon=True)
        self._grab_thread.start()

        # 发布定时器
        period = 1.0 / (self.frame_rate if self.frame_rate > 0 else 30.0)
        self._pub_timer = self.create_timer(period, self._publish_latest)

        self.get_logger().info(
            f'Camera publisher started (V4L2+MJPG): '
            f'/dev/video{self.device_id}, {self.width}x{self.height}@{int(self.frame_rate)}'
        )

    def _grab_loop(self):
        """单独线程：一直抓帧，不发布，只更新 self._latest。"""
        while self._running and self.cap.isOpened():
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.001)
                continue

            with self._lock:
                self._latest = frame

            # 抓帧 fps 日志
            self._grab_cnt += 1
            now = time.time()
            if now - self._grab_t0 >= 1.0:
                fps = self._grab_cnt / (now - self._grab_t0)
                self.get_logger().info(f'grab fps ~ {fps:.1f}')
                self._grab_cnt = 0
                self._grab_t0 = now

    def _publish_latest(self):
        """按固定频率发布 raw + compressed。"""
        with self._lock:
            if self._latest is None:
                return
            frame = self._latest.copy()    # BGR 图像 (H,W,3)

        # 统一时间戳，raw 和 compressed 完全同步
        stamp = self.get_clock().now().to_msg()

        # -------- raw Image --------
        raw_msg = Image()
        raw_msg.header.stamp = stamp
        raw_msg.header.frame_id = self.frame_id

        h, w, ch = frame.shape
        raw_msg.height = h
        raw_msg.width = w
        raw_msg.encoding = 'bgr8'   # OpenCV 默认 BGR
        raw_msg.is_bigendian = 0
        raw_msg.step = w * ch       # 每行多少字节
        raw_msg.data = array.array('B', frame.tobytes())

        self.pub_raw.publish(raw_msg)

        # -------- CompressedImage (jpeg) --------
        ok, buf = cv2.imencode('.jpg', frame, [int(cv2.IMWRITE_JPEG_QUALITY), 90])
        if not ok:
            return

        comp_msg = CompressedImage()
        comp_msg.header.stamp = stamp
        comp_msg.header.frame_id = self.frame_id
        comp_msg.format = 'jpeg'
        comp_msg.data = array.array('B', buf.tobytes())

        self.pub_compressed.publish(comp_msg)

    def destroy_node(self):
        self._running = False
        try:
            if hasattr(self, '_grab_thread'):
                self._grab_thread.join(timeout=1.0)
        except Exception:
            pass
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

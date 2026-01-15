#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
)
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')

        # 参数：设备 ID 和帧率
        self.declare_parameter('device_id', 0)        # 通常 /dev/video0 对应 0
        self.declare_parameter('frame_rate', 30.0)    # Hz
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)

        device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        frame_rate = self.get_parameter('frame_rate').get_parameter_value().double_value
        width = self.get_parameter('width').get_parameter_value().integer_value
        height = self.get_parameter('height').get_parameter_value().integer_value

        # 打开摄像头
        self.cap = cv2.VideoCapture(device_id, cv2.CAP_V4L2)
        if not self.cap.isOpened():
            self.get_logger().error(f'Failed to open camera device {device_id}')
            raise RuntimeError('Cannot open camera')

        # 设置分辨率（若驱动支持）
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

        # QoS：图像流一般用 BEST_EFFORT
        qos_profile = QoSProfile(
            depth=10,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )

        self.publisher_ = self.create_publisher(
            Image,
            'camera/image_raw',
            qos_profile
        )

        self.bridge = CvBridge()

        # 定时器按照帧率触发
        timer_period = 1.0 / frame_rate if frame_rate > 0.0 else 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.get_logger().info(
            f'Camera publisher started: device={device_id}, '
            f'resolution={width}x{height}, fps={frame_rate}'
        )

    def timer_callback(self):
        if not self.cap.isOpened():
            self.get_logger().error('Camera is not opened')
            return

        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warning('Failed to read frame from camera')
            return

        # OpenCV 默认 BGR8
        msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_frame'

        self.publisher_.publish(msg)

    def destroy_node(self):
        # 退出时释放摄像头
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

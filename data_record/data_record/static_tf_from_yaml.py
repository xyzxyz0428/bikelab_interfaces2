#!/usr/bin/env python3
import os
from math import sin, cos

import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


def rpy_to_quaternion(roll: float, pitch: float, yaw: float):
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qx, qy, qz, qw


class StaticTfFromYaml(Node):
    def __init__(self):
        super().__init__("static_tf_from_yaml")

        # Parameter with path to YAML file
        self.declare_parameter("tf_yaml_path", "")
        tf_yaml_path = self.get_parameter("tf_yaml_path").get_parameter_value().string_value

        if not tf_yaml_path:
            raise RuntimeError("Parameter 'tf_yaml_path' is empty.")
        if not os.path.isfile(tf_yaml_path):
            raise FileNotFoundError(f"TF YAML not found: {tf_yaml_path}")

        self.get_logger().info(f"Loading static TFs from: {tf_yaml_path}")
        self.broadcaster = StaticTransformBroadcaster(self)
        transforms = self._load_transforms_from_yaml(tf_yaml_path)
        self.broadcaster.sendTransform(transforms)

        self.get_logger().info(f"Published {len(transforms)} static transforms from: {tf_yaml_path}")

    def _load_transforms_from_yaml(self, yaml_path: str):
        with open(yaml_path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f)

        if "frames" not in data or not isinstance(data["frames"], list):
            raise RuntimeError("YAML must contain a top-level 'frames' list.")

        transforms = []
        now_msg = self.get_clock().now().to_msg()

        for i, item in enumerate(data["frames"]):
            parent = item["parent"]
            child = item["child"]
            xyz = item["xyz"]
            rpy = item["rpy"]

            if len(xyz) != 3 or len(rpy) != 3:
                raise ValueError("xyz and rpy must have length 3")

            t = TransformStamped()
            t.header.stamp = now_msg
            t.header.frame_id = str(parent)
            t.child_frame_id = str(child)

            t.transform.translation.x = float(xyz[0])
            t.transform.translation.y = float(xyz[1])
            t.transform.translation.z = float(xyz[2])

            qx, qy, qz, qw = rpy_to_quaternion(float(rpy[0]), float(rpy[1]), float(rpy[2]))
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            transforms.append(t)

        return transforms


def main(args=None):
    rclpy.init(args=args)
    node = StaticTfFromYaml()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
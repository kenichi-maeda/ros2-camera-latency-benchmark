#!/usr/bin/env python3
"""ROS relay launched by Dora.

Env vars:
- DORA_TRANSPORT: raw|compressed
- DORA_IN_TOPIC: source topic
- DORA_OUT_TOPIC: destination topic
- DORA_QOS_RELIABLE: true|false
"""

import os
from typing import Type

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image


def _to_bool(value: str | None, default: bool = False) -> bool:
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "on"}


def _msg_type(transport: str) -> Type[Image] | Type[CompressedImage]:
    return CompressedImage if transport == "compressed" else Image


class DoraRosRelay(Node):
    def __init__(self) -> None:
        super().__init__(os.getenv("DORA_ROS_NODE_NAME", "dora_ros_bridge"))

        transport = os.getenv("DORA_TRANSPORT", "raw").strip().lower()
        if transport not in {"raw", "compressed"}:
            raise ValueError(f"Unsupported DORA_TRANSPORT={transport!r}")

        default_in = "image/compressed" if transport == "compressed" else "image"
        default_out = "image_dora/compressed" if transport == "compressed" else "image_dora"
        in_topic = os.getenv("DORA_IN_TOPIC", default_in)
        out_topic = os.getenv("DORA_OUT_TOPIC", default_out)

        reliable = _to_bool(os.getenv("DORA_QOS_RELIABLE"), default=False)
        qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=(QoSReliabilityPolicy.RELIABLE if reliable else QoSReliabilityPolicy.BEST_EFFORT),
        )

        msg_type = _msg_type(transport)
        self._count = 0
        self._pub = self.create_publisher(msg_type, out_topic, qos)
        self._sub = self.create_subscription(msg_type, in_topic, self._on_msg, qos)
        self.get_logger().info(
            f"forwarding {msg_type.__name__}: {in_topic} -> {out_topic} (reliable={reliable})"
        )

    def _on_msg(self, msg: Image | CompressedImage) -> None:
        self._pub.publish(msg)
        self._count += 1
        if self._count % 100 == 0:
            self.get_logger().info(f"forwarded {self._count} messages")


def main() -> int:
    rclpy.init()
    node = DoraRosRelay()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

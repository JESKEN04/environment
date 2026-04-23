#!/usr/bin/env python3
"""将三维栅格地图以自定义JSON消息发布到ROS2。"""

from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GridMapPublisher(Node):
    def __init__(self) -> None:
        super().__init__("grid_map_publisher")
        self.publisher_ = self.create_publisher(String, "/inspection/grid_map", 10)
        self.timer = self.create_timer(1.0, self.publish_once)
        self.map_path = Path("uav_inspection/data/voxel_map_msg.json")
        self.sent = False

    def publish_once(self) -> None:
        if self.sent:
            return
        if not self.map_path.exists():
            self.get_logger().error(f"地图文件不存在: {self.map_path}")
            return
        msg = String()
        msg.data = self.map_path.read_text(encoding="utf-8")
        self.publisher_.publish(msg)
        self.get_logger().info("已发布三维栅格地图消息 /inspection/grid_map")
        self.sent = True


def main() -> None:
    rclpy.init()
    node = GridMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

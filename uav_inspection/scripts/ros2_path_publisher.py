#!/usr/bin/env python3
"""将ABC输出路径发布到ROS2。"""

from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class PathPublisher(Node):
    def __init__(self) -> None:
        super().__init__("inspection_path_publisher")
        self.publisher_ = self.create_publisher(String, "/inspection/path", 10)
        self.timer = self.create_timer(1.0, self.publish_once)
        self.path_file = Path("uav_inspection/data/inspection_path.json")
        self.sent = False

    def publish_once(self) -> None:
        if self.sent:
            return
        if not self.path_file.exists():
            self.get_logger().error(f"路径文件不存在: {self.path_file}")
            return
        msg = String()
        msg.data = self.path_file.read_text(encoding="utf-8")
        self.publisher_.publish(msg)
        self.get_logger().info("已发布路径消息 /inspection/path")
        self.sent = True


def main() -> None:
    rclpy.init()
    node = PathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

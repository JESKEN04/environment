#!/usr/bin/env python3
"""发布规划后的路径到 ROS2 /inspection/planned_path (nav_msgs/Path)。"""

import json
from pathlib import Path

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path as PathMsg
from rclpy.node import Node


class PlannedPathPublisher(Node):
    def __init__(self):
        super().__init__("planned_path_publisher")
        self.pub = self.create_publisher(PathMsg, "/inspection/planned_path", 10)
        self.timer = self.create_timer(1.0, self._tick)

        root = Path(__file__).resolve().parents[1]
        with (root / "artifacts" / "planning" / "planned_path.json").open("r", encoding="utf-8") as f:
            data = json.load(f)
        self.points = data["smooth_path"]

    def _tick(self):
        now = self.get_clock().now().to_msg()
        msg = PathMsg()
        msg.header.frame_id = "map"
        msg.header.stamp = now
        for p in self.points:
            ps = PoseStamped()
            ps.header.frame_id = "map"
            ps.header.stamp = now
            ps.pose.position.x = float(p[0])
            ps.pose.position.y = float(p[1])
            ps.pose.position.z = float(p[2])
            ps.pose.orientation.w = 1.0
            msg.poses.append(ps)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PlannedPathPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""将三维栅格地图以自定义JSON格式发布到ROS2话题。"""

import json
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class GridMapPublisher(Node):
    def __init__(self):
        super().__init__("grid_map_publisher")
        self.pub = self.create_publisher(String, "/inspection/grid_map", 10)
        self.timer = self.create_timer(1.0, self._tick)
        root = Path(__file__).resolve().parents[1]
        arr = np.load(root / "artifacts" / "grid_map" / "voxel_map.npz")["occupancy"]
        with (root / "artifacts" / "grid_map" / "map_metadata.json").open("r", encoding="utf-8") as f:
            meta = json.load(f)
        self.payload = {
            "frame_id": "map",
            "resolution": meta["resolution"],
            "bounds": meta["bounds"],
            "shape": list(arr.shape),
            "occupied_indices": np.argwhere(arr > 0).tolist(),
            "targets": meta["targets"],
        }

    def _tick(self):
        msg = String()
        msg.data = json.dumps(self.payload, ensure_ascii=False)
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = GridMapPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""ROS2节点：发布三维栅格地图与ABC规划路径。"""

from __future__ import annotations

from pathlib import Path

import numpy as np
import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Header

from uav_inspection.msg import Path3D, VoxelGrid3D


class MapPublisher(Node):
    def __init__(self):
        super().__init__('voxel_map_publisher')
        repo = Path(__file__).resolve().parents[1]
        data = np.load(repo / 'config' / 'voxel_map.npz')

        self.pub = self.create_publisher(VoxelGrid3D, '/inspection/voxel_map', 10)
        self.timer = self.create_timer(1.0, self.tick)

        self.msg = VoxelGrid3D()
        self.msg.header = Header(frame_id='map')
        self.msg.resolution = float(data['resolution'])
        b = data['bounds']
        self.msg.origin = Point(x=float(b[0]), y=float(b[2]), z=float(b[4]))
        occ = data['occupancy']
        self.msg.size_x, self.msg.size_y, self.msg.size_z = occ.shape
        self.msg.occupancy = occ.flatten().astype(np.uint8).tolist()

        for t in data['targets']:
            self.msg.inspection_targets.append(Point(x=float(t[0]), y=float(t[1]), z=float(t[2])))

    def tick(self):
        self.msg.header.stamp = self.get_clock().now().to_msg()
        self.pub.publish(self.msg)


class PathPublisher(Node):
    def __init__(self):
        super().__init__('abc_path_publisher')
        repo = Path(__file__).resolve().parents[1]
        path = np.load(repo / 'config' / 'planned_path_smooth.npy')

        self.pub = self.create_publisher(Path3D, '/inspection/planned_path', 10)
        self.timer = self.create_timer(1.0, self.tick)

        self.msg = Path3D()
        self.msg.header = Header(frame_id='map')
        for p in path:
            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = float(p[0])
            pose.pose.position.y = float(p[1])
            pose.pose.position.z = float(p[2])
            pose.pose.orientation.w = 1.0
            self.msg.waypoints.append(pose)

    def tick(self):
        stamp = self.get_clock().now().to_msg()
        self.msg.header.stamp = stamp
        for p in self.msg.waypoints:
            p.header.stamp = stamp
        self.pub.publish(self.msg)


def main():
    rclpy.init()
    n1 = MapPublisher()
    n2 = PathPublisher()
    exe = rclpy.executors.MultiThreadedExecutor()
    exe.add_node(n1)
    exe.add_node(n2)
    exe.spin()


if __name__ == '__main__':
    main()

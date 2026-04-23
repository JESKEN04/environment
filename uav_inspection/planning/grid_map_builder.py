#!/usr/bin/env python3
"""从SDF场景构建三维栅格地图（1m分辨率）并发布ROS2话题。"""

from __future__ import annotations

import json
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import numpy as np


@dataclass
class MapConfig:
    bounds: Tuple[float, float, float, float, float, float] = (-100.0, 100.0, -100.0, 100.0, 0.0, 100.0)
    resolution: float = 1.0
    inflation_radius_m: float = 0.5


@dataclass
class AABB:
    min_xyz: np.ndarray
    max_xyz: np.ndarray


class GridMapBuilder:
    def __init__(self, sdf_path: str, config: MapConfig | None = None):
        self.sdf_path = Path(sdf_path)
        self.config = config or MapConfig()
        self.occupancy = self._create_empty_grid()
        self.targets: List[Tuple[float, float, float]] = []

    def _create_empty_grid(self) -> np.ndarray:
        xmin, xmax, ymin, ymax, zmin, zmax = self.config.bounds
        size_x = int((xmax - xmin) / self.config.resolution)
        size_y = int((ymax - ymin) / self.config.resolution)
        size_z = int((zmax - zmin) / self.config.resolution)
        return np.zeros((size_x, size_y, size_z), dtype=np.uint8)

    def _world_to_grid(self, x: float, y: float, z: float) -> Tuple[int, int, int]:
        xmin, _, ymin, _, zmin, _ = self.config.bounds
        r = self.config.resolution
        return int((x - xmin) / r), int((y - ymin) / r), int((z - zmin) / r)

    @staticmethod
    def _parse_pose(elem) -> Tuple[float, float, float]:
        if elem is None or elem.text is None:
            return 0.0, 0.0, 0.0
        vals = [float(v) for v in elem.text.strip().split()]
        return vals[0], vals[1], vals[2]

    def _extract_model_aabb(self, model_elem) -> List[AABB]:
        model_pose = self._parse_pose(model_elem.find("pose"))
        model_xyz = np.array(model_pose)
        aabbs: List[AABB] = []

        for link in model_elem.findall("link"):
            link_pose = np.array(self._parse_pose(link.find("pose")))
            for collision in link.findall("collision"):
                c_pose = np.array(self._parse_pose(collision.find("pose")))
                geom = collision.find("geometry")
                if geom is None:
                    continue

                size = None
                box = geom.find("box")
                cyl = geom.find("cylinder")
                sphere = geom.find("sphere")

                if box is not None and box.find("size") is not None:
                    size = np.array([float(v) for v in box.find("size").text.split()])
                elif cyl is not None:
                    r = float(cyl.find("radius").text)
                    l = float(cyl.find("length").text)
                    size = np.array([2 * r, 2 * r, l])
                elif sphere is not None:
                    r = float(sphere.find("radius").text)
                    size = np.array([2 * r, 2 * r, 2 * r])

                if size is None:
                    continue

                center = model_xyz + link_pose + c_pose
                half = size / 2.0
                aabbs.append(AABB(center - half, center + half))

        return aabbs

    def build_from_sdf(self) -> None:
        root = ET.parse(self.sdf_path).getroot()
        world = root.find("world")
        if world is None:
            raise RuntimeError("SDF中未找到world标签")

        for model in world.findall("model"):
            name = model.attrib.get("name", "")
            if "tower" in name or "tree" in name or "wire" in name:
                for aabb in self._extract_model_aabb(model):
                    self._mark_aabb(aabb)
            if "tower" in name:
                self.targets.append(tuple(self._parse_pose(model.find("pose"))))

        self._inflate_obstacles()

    def _mark_aabb(self, aabb: AABB) -> None:
        xmin, xmax, ymin, ymax, zmin, zmax = self.config.bounds
        minx, miny, minz = np.maximum(aabb.min_xyz, [xmin, ymin, zmin])
        maxx, maxy, maxz = np.minimum(aabb.max_xyz, [xmax, ymax, zmax])

        gx0, gy0, gz0 = self._world_to_grid(minx, miny, minz)
        gx1, gy1, gz1 = self._world_to_grid(maxx, maxy, maxz)
        self.occupancy[gx0:gx1 + 1, gy0:gy1 + 1, gz0:gz1 + 1] = 1

    def _inflate_obstacles(self) -> None:
        inflation_cells = max(1, int(math.ceil(self.config.inflation_radius_m / self.config.resolution)))
        occ = self.occupancy
        inflated = occ.copy()
        idxs = np.argwhere(occ > 0)
        for x, y, z in idxs:
            x0, x1 = max(0, x - inflation_cells), min(occ.shape[0], x + inflation_cells + 1)
            y0, y1 = max(0, y - inflation_cells), min(occ.shape[1], y + inflation_cells + 1)
            z0, z1 = max(0, z - inflation_cells), min(occ.shape[2], z + inflation_cells + 1)
            inflated[x0:x1, y0:y1, z0:z1] = 1
        self.occupancy = inflated

    def save(self, output_npz: str, output_json: str) -> None:
        np.savez_compressed(
            output_npz,
            occupancy=self.occupancy,
            resolution=self.config.resolution,
            bounds=np.array(self.config.bounds),
            targets=np.array(self.targets, dtype=np.float32),
        )
        with open(output_json, "w", encoding="utf-8") as f:
            json.dump(
                {
                    "resolution": self.config.resolution,
                    "bounds": self.config.bounds,
                    "grid_shape": self.occupancy.shape,
                    "occupied_count": int(self.occupancy.sum()),
                    "targets": self.targets,
                },
                f,
                ensure_ascii=False,
                indent=2,
            )


def main():
    repo = Path(__file__).resolve().parents[1]
    sdf = repo / "worlds" / "inspection_world.sdf"
    out_dir = repo / "config"
    out_dir.mkdir(exist_ok=True)
    builder = GridMapBuilder(str(sdf), MapConfig(bounds=(-200, 200, -200, 200, 0, 100), resolution=1.0))
    builder.build_from_sdf()
    builder.save(str(out_dir / "voxel_map.npz"), str(out_dir / "voxel_map_meta.json"))
    print(f"已生成栅格地图: {out_dir / 'voxel_map.npz'}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""根据SDF/世界配置构建三维栅格地图（1m分辨率）。"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable

import numpy as np


@dataclass
class MapSpec:
    x_min: int = -100
    x_max: int = 100
    y_min: int = -100
    y_max: int = 100
    z_min: int = 0
    z_max: int = 100
    resolution: float = 1.0

    @property
    def shape(self) -> tuple[int, int, int]:
        return (
            int((self.x_max - self.x_min) / self.resolution),
            int((self.y_max - self.y_min) / self.resolution),
            int((self.z_max - self.z_min) / self.resolution),
        )


def world_to_idx(spec: MapSpec, x: float, y: float, z: float) -> tuple[int, int, int]:
    ix = int((x - spec.x_min) / spec.resolution)
    iy = int((y - spec.y_min) / spec.resolution)
    iz = int((z - spec.z_min) / spec.resolution)
    return ix, iy, iz


def fill_aabb(grid: np.ndarray, spec: MapSpec, mins: tuple[float, float, float], maxs: tuple[float, float, float]) -> None:
    ix0, iy0, iz0 = world_to_idx(spec, *mins)
    ix1, iy1, iz1 = world_to_idx(spec, *maxs)
    ix0, iy0, iz0 = max(ix0, 0), max(iy0, 0), max(iz0, 0)
    ix1, iy1, iz1 = min(ix1, grid.shape[0] - 1), min(iy1, grid.shape[1] - 1), min(iz1, grid.shape[2] - 1)
    if ix0 <= ix1 and iy0 <= iy1 and iz0 <= iz1:
        grid[ix0 : ix1 + 1, iy0 : iy1 + 1, iz0 : iz1 + 1] = True


def obstacle_aabbs(cfg: dict) -> Iterable[tuple[tuple[float, float, float], tuple[float, float, float], str]]:
    base = cfg["base_tower"]
    peak = cfg["peak_tower"]
    tower_size = 2.0

    yield (
        (base["x"] - tower_size / 2, base["y"] - tower_size / 2, base["z"]),
        (base["x"] + tower_size / 2, base["y"] + tower_size / 2, base["top_z"]),
        "tower_A",
    )
    yield (
        (peak["x"] - tower_size / 2, peak["y"] - tower_size / 2, peak["z"]),
        (peak["x"] + tower_size / 2, peak["y"] + tower_size / 2, peak["top_z"]),
        "tower_B",
    )

    # 线缆：用沿两塔顶点之间的小AABB串近似
    p0 = np.array([base["x"], base["y"], base["wire_z_L"]])
    p1 = np.array([peak["x"], peak["y"], peak["wire_z_L"]])
    for t in np.linspace(0, 1, 60):
        p = p0 * (1 - t) + p1 * t
        sag = 7.0 * np.sin(np.pi * t)
        p[2] -= sag
        r = 0.15
        yield ((p[0] - r, p[1] - r, p[2] - r), (p[0] + r, p[1] + r, p[2] + r), "wire")

    for tree in cfg.get("trees", []):
        r = 0.8
        yield (
            (tree["x"] - r, tree["y"] - r, tree["z"]),
            (tree["x"] + r, tree["y"] + r, tree["z"] + tree["h"]),
            "tree",
        )


def dilate_binary(grid: np.ndarray, radius_cells: int) -> np.ndarray:
    if radius_cells <= 0:
        return grid
    dilated = grid.copy()
    occ = np.argwhere(grid)
    for x, y, z in occ:
        x0, x1 = max(0, x - radius_cells), min(grid.shape[0], x + radius_cells + 1)
        y0, y1 = max(0, y - radius_cells), min(grid.shape[1], y + radius_cells + 1)
        z0, z1 = max(0, z - radius_cells), min(grid.shape[2], z + radius_cells + 1)
        dilated[x0:x1, y0:y1, z0:z1] = True
    return dilated


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", type=Path, default=Path("uav_inspection/worlds/world_config.json"))
    parser.add_argument("--out", type=Path, default=Path("uav_inspection/data/voxel_map.npz"))
    parser.add_argument("--msg", type=Path, default=Path("uav_inspection/data/voxel_map_msg.json"))
    parser.add_argument("--safety_margin", type=float, default=0.5)
    args = parser.parse_args()

    spec = MapSpec()
    cfg = json.loads(args.config.read_text(encoding="utf-8"))
    grid = np.zeros(spec.shape, dtype=bool)

    labels = []
    for mins, maxs, label in obstacle_aabbs(cfg):
        fill_aabb(grid, spec, mins, maxs)
        labels.append({"type": label, "mins": mins, "maxs": maxs})

    radius_cells = int(np.ceil(args.safety_margin / spec.resolution))
    inflated = dilate_binary(grid, radius_cells)

    args.out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(
        args.out,
        occupancy=inflated.astype(np.uint8),
        resolution=np.array([spec.resolution]),
        bounds=np.array([spec.x_min, spec.x_max, spec.y_min, spec.y_max, spec.z_min, spec.z_max]),
    )

    occupied_idx = np.argwhere(inflated)
    payload = {
        "header": {"frame_id": "map", "stamp": 0.0},
        "resolution": spec.resolution,
        "bounds": {
            "x": [spec.x_min, spec.x_max],
            "y": [spec.y_min, spec.y_max],
            "z": [spec.z_min, spec.z_max],
        },
        "shape": list(spec.shape),
        "occupied_voxels": occupied_idx.tolist(),
        "targets": {
            "tower_A": [cfg["base_tower"]["x"], cfg["base_tower"]["y"], cfg["base_tower"]["top_z"] + 5.0],
            "tower_B": [cfg["peak_tower"]["x"], cfg["peak_tower"]["y"], cfg["peak_tower"]["top_z"] + 5.0],
        },
        "obstacle_aabbs": labels,
    }
    args.msg.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")

    print(f"地图体素尺寸: {spec.shape}, 占用体素: {len(occupied_idx)}")
    print(f"已输出: {args.out}")
    print(f"已输出: {args.msg}")


if __name__ == "__main__":
    main()

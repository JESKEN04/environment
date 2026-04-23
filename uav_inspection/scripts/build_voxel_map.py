#!/usr/bin/env python3
"""构建巡检场景三维栅格地图（1m分辨率）并导出可视化图。"""

from __future__ import annotations

import json
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple

try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None
import numpy as np


@dataclass
class GridConfig:
    x_min: float = -100.0
    x_max: float = 100.0
    y_min: float = -100.0
    y_max: float = 100.0
    z_min: float = 0.0
    z_max: float = 100.0
    resolution: float = 1.0
    inflate_radius_m: float = 0.5


ROOT = Path(__file__).resolve().parents[1]
WORLD_CONFIG = ROOT / "worlds" / "world_config.json"
OUT_DIR = ROOT / "artifacts" / "grid_map"
OUT_DIR.mkdir(parents=True, exist_ok=True)


def _to_grid(x: float, y: float, z: float, cfg: GridConfig) -> Tuple[int, int, int]:
    return (
        int(np.floor((x - cfg.x_min) / cfg.resolution)),
        int(np.floor((y - cfg.y_min) / cfg.resolution)),
        int(np.floor((z - cfg.z_min) / cfg.resolution)),
    )


def _mark_aabb(grid: np.ndarray, mn: Tuple[float, float, float], mx: Tuple[float, float, float], cfg: GridConfig) -> None:
    gx0, gy0, gz0 = _to_grid(*mn, cfg)
    gx1, gy1, gz1 = _to_grid(*mx, cfg)
    gx0, gy0, gz0 = np.maximum([gx0, gy0, gz0], [0, 0, 0])
    gx1, gy1, gz1 = np.minimum([gx1, gy1, gz1], np.array(grid.shape) - 1)
    if gx1 >= gx0 and gy1 >= gy0 and gz1 >= gz0:
        grid[gx0 : gx1 + 1, gy0 : gy1 + 1, gz0 : gz1 + 1] = 1


def _inflate(grid: np.ndarray, inflate_cells: int) -> np.ndarray:
    if inflate_cells <= 0:
        return grid
    occ = np.argwhere(grid > 0)
    inflated = grid.copy()
    for x, y, z in occ:
        x0, x1 = max(0, x - inflate_cells), min(grid.shape[0], x + inflate_cells + 1)
        y0, y1 = max(0, y - inflate_cells), min(grid.shape[1], y + inflate_cells + 1)
        z0, z1 = max(0, z - inflate_cells), min(grid.shape[2], z + inflate_cells + 1)
        inflated[x0:x1, y0:y1, z0:z1] = 1
    return inflated


def build_grid(cfg: GridConfig) -> Dict:
    with WORLD_CONFIG.open("r", encoding="utf-8") as f:
        wc = json.load(f)

    nx = int((cfg.x_max - cfg.x_min) / cfg.resolution)
    ny = int((cfg.y_max - cfg.y_min) / cfg.resolution)
    nz = int((cfg.z_max - cfg.z_min) / cfg.resolution)
    grid = np.zeros((nx, ny, nz), dtype=np.uint8)

    # 输电塔 A/B（以包围盒近似）
    for key in ["base_tower", "peak_tower"]:
        t = wc[key]
        tower_h = t["top_z"] - t["z"]
        mn = (t["x"] - 2.0, t["y"] - 2.0, t["z"])
        mx = (t["x"] + 2.0, t["y"] + 2.0, t["z"] + tower_h)
        _mark_aabb(grid, mn, mx, cfg)

    # 树木障碍物
    for tr in wc.get("trees", []):
        mn = (tr["x"] - 0.6, tr["y"] - 0.6, tr["z"])
        mx = (tr["x"] + 0.6, tr["y"] + 0.6, tr["z"] + tr["h"])
        _mark_aabb(grid, mn, mx, cfg)

    # 电线（离散线段并膨胀为圆柱）
    bt, pt = wc["base_tower"], wc["peak_tower"]
    p0 = np.array([bt["x"], bt["y"], bt["wire_z_L"]], dtype=float)
    p1 = np.array([pt["x"], pt["y"], pt["wire_z_L"]], dtype=float)
    n_seg = 120
    sag = 5.0
    wire_r = 0.05
    for i in range(n_seg + 1):
        t = i / n_seg
        p = p0 * (1 - t) + p1 * t
        p[2] -= sag * np.sin(np.pi * t)
        r = 0.4 + wire_r
        mn = tuple(p - r)
        mx = tuple(p + r)
        _mark_aabb(grid, mn, mx, cfg)

    inflate_cells = int(np.ceil(cfg.inflate_radius_m / cfg.resolution))
    inflated = _inflate(grid, inflate_cells)

    # 巡检目标点（电线上方 5m）
    targets = []
    for i in range(0, n_seg + 1, 10):
        t = i / n_seg
        p = p0 * (1 - t) + p1 * t
        p[2] = p[2] - sag * np.sin(np.pi * t) + 5.0
        targets.append(p.tolist())

    np.savez_compressed(OUT_DIR / "voxel_map.npz", occupancy=inflated)
    meta = {
        "bounds": {
            "x": [cfg.x_min, cfg.x_max],
            "y": [cfg.y_min, cfg.y_max],
            "z": [cfg.z_min, cfg.z_max],
        },
        "resolution": cfg.resolution,
        "inflate_radius_m": cfg.inflate_radius_m,
        "targets": targets,
        "start": [wc["drones"][0]["x"], wc["drones"][0]["y"], wc["drones"][0]["z"]],
    }
    with (OUT_DIR / "map_metadata.json").open("w", encoding="utf-8") as f:
        json.dump(meta, f, indent=2, ensure_ascii=False)

    # 彩色可视化（xy投影）
    if plt is not None:
        occ_xy = np.max(inflated, axis=2).T
        fig, ax = plt.subplots(figsize=(10, 8), dpi=160)
        ax.imshow(
            occ_xy,
            origin="lower",
            extent=[cfg.x_min, cfg.x_max, cfg.y_min, cfg.y_max],
            cmap="magma",
            alpha=0.8,
        )
        tar = np.array(targets)
        ax.plot(tar[:, 0], tar[:, 1], color="#00e5ff", linewidth=2.0, label="巡检目标投影")
        ax.scatter(meta["start"][0], meta["start"][1], c="#76ff03", s=80, marker="*", label="起点")
        ax.set_title("三维栅格地图（XY投影，1m分辨率）", fontproperties="DejaVu Sans")
        ax.set_xlabel("X / m")
        ax.set_ylabel("Y / m")
        ax.legend()
        ax.grid(alpha=0.2)
        fig.tight_layout()
        fig.savefig(OUT_DIR / "grid_map_xy.png")
        plt.close(fig)

    return meta


if __name__ == "__main__":
    info = build_grid(GridConfig())
    print("[OK] 栅格地图已生成:", OUT_DIR)
    print("[INFO] 目标点数量:", len(info["targets"]))

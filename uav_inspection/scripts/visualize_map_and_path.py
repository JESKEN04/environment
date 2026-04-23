#!/usr/bin/env python3
"""离线可视化三维栅格地图与规划航迹。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np

try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None


ROOT = Path(__file__).resolve().parents[1]
MAP_DIR = ROOT / "artifacts" / "grid_map"
PLAN_DIR = ROOT / "artifacts" / "planning"
VIS_DIR = ROOT / "artifacts" / "visualization"
VIS_DIR.mkdir(parents=True, exist_ok=True)


def main() -> None:
    if plt is None:
        print("[WARN] matplotlib 未安装，无法绘制可视化图。")
        print("[TIP] 先安装依赖: sudo apt install -y python3-matplotlib")
        return

    occ = np.load(MAP_DIR / "voxel_map.npz")["occupancy"]
    with (MAP_DIR / "map_metadata.json").open("r", encoding="utf-8") as f:
        meta = json.load(f)
    with (PLAN_DIR / "planned_path.json").open("r", encoding="utf-8") as f:
        plan = json.load(f)

    raw = np.array(plan["raw_path"])
    smooth = np.array(plan["smooth_path"])

    x_min, x_max = meta["bounds"]["x"]
    y_min, y_max = meta["bounds"]["y"]
    z_min, z_max = meta["bounds"]["z"]
    res = meta["resolution"]

    # 稀疏采样占用点，避免过慢
    occ_idx = np.argwhere(occ > 0)
    if len(occ_idx) > 25000:
        occ_idx = occ_idx[:: max(1, len(occ_idx) // 25000)]

    occ_world = np.zeros((len(occ_idx), 3), dtype=float)
    occ_world[:, 0] = x_min + (occ_idx[:, 0] + 0.5) * res
    occ_world[:, 1] = y_min + (occ_idx[:, 1] + 0.5) * res
    occ_world[:, 2] = z_min + (occ_idx[:, 2] + 0.5) * res

    # 3D图
    fig = plt.figure(figsize=(12, 9), dpi=170)
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(
        occ_world[:, 0],
        occ_world[:, 1],
        occ_world[:, 2],
        s=2,
        c="#8e24aa",
        alpha=0.15,
        label="障碍物栅格",
    )
    ax.plot(raw[:, 0], raw[:, 1], raw[:, 2], "--", lw=1.3, c="#ef5350", label="原始ABC路径")
    ax.plot(smooth[:, 0], smooth[:, 1], smooth[:, 2], lw=2.5, c="#00c853", label="平滑路径")

    start = np.array(meta["start"])
    targets = np.array(meta["targets"])
    ax.scatter(start[0], start[1], start[2], c="#2979ff", s=120, marker="*", label="起点")
    ax.scatter(targets[:, 0], targets[:, 1], targets[:, 2], c="#ffb300", s=24, label="巡检目标点")

    ax.set_xlim(x_min, x_max)
    ax.set_ylim(y_min, y_max)
    ax.set_zlim(z_min, z_max)
    ax.set_xlabel("X / m")
    ax.set_ylabel("Y / m")
    ax.set_zlabel("Z / m")
    ax.set_title("三维栅格地图与规划航迹可视化")
    ax.legend(loc="upper left")
    plt.tight_layout()
    fig.savefig(VIS_DIR / "map_and_path_3d.png")
    plt.close(fig)

    # 俯视图
    fig2, ax2 = plt.subplots(figsize=(10, 8), dpi=170)
    occ_xy = np.max(occ, axis=2).T
    ax2.imshow(
        occ_xy,
        origin="lower",
        extent=[x_min, x_max, y_min, y_max],
        cmap="magma",
        alpha=0.75,
    )
    ax2.plot(raw[:, 0], raw[:, 1], "--", lw=1.5, c="#ff7043", label="原始路径")
    ax2.plot(smooth[:, 0], smooth[:, 1], lw=2.4, c="#00e676", label="平滑路径")
    ax2.scatter(start[0], start[1], c="#2979ff", s=120, marker="*", label="起点")
    ax2.scatter(targets[:, 0], targets[:, 1], c="#ffd54f", s=28, label="巡检目标")
    ax2.set_title("地图与航迹俯视图")
    ax2.set_xlabel("X / m")
    ax2.set_ylabel("Y / m")
    ax2.grid(alpha=0.2)
    ax2.legend()
    plt.tight_layout()
    fig2.savefig(VIS_DIR / "map_and_path_xy.png")
    plt.close(fig2)

    print("[OK] 可视化图已生成:")
    print(" -", VIS_DIR / "map_and_path_3d.png")
    print(" -", VIS_DIR / "map_and_path_xy.png")


if __name__ == "__main__":
    main()

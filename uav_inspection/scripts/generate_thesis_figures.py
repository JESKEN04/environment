#!/usr/bin/env python3
"""生成论文可用彩色图：栅格地图、路径、公式图。"""

from __future__ import annotations

import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def main() -> None:
    data_dir = Path("uav_inspection/data")
    fig_dir = Path("uav_inspection/figures")
    fig_dir.mkdir(parents=True, exist_ok=True)

    vox = np.load(data_dir / "voxel_map.npz")
    occ = vox["occupancy"].astype(bool)
    bounds = vox["bounds"]
    path = json.loads((data_dir / "inspection_path.json").read_text(encoding="utf-8"))
    smooth = np.array(path["smoothed_waypoints"])

    occ_idx = np.argwhere(occ)
    x = occ_idx[:, 0] + bounds[0]
    y = occ_idx[:, 1] + bounds[2]
    z = occ_idx[:, 2] + bounds[4]

    fig = plt.figure(figsize=(11, 8))
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(x[::5], y[::5], z[::5], s=1, c=z[::5], cmap="viridis", alpha=0.25, label="障碍物")
    ax.plot(smooth[:, 0], smooth[:, 1], smooth[:, 2], c="red", lw=2.5, label="ABC规划路径")
    ax.scatter(*path["inspection_targets"]["tower_A"], c="orange", s=70, label="巡检点A")
    ax.scatter(*path["inspection_targets"]["tower_B"], c="cyan", s=70, label="巡检点B")
    ax.set_title("三维栅格地图与ABC路径规划结果", fontsize=14)
    ax.set_xlabel("X / m")
    ax.set_ylabel("Y / m")
    ax.set_zlabel("Z / m")
    ax.legend()
    fig.tight_layout()
    fig.savefig(fig_dir / "grid_and_path.png", dpi=220)

    fig2 = plt.figure(figsize=(10, 5))
    fig2.patch.set_facecolor("white")
    txt = (
        "适应度函数:\n"
        r"$f = w_1L + w_2\frac{1}{D_{min}} + w_3C$" "\n\n"
        "其中: "
        r"$w_1=0.4,\;w_2=0.4,\;w_3=0.2$" "\n"
        r"$L$: 路径长度, $D_{min}$: 最近障碍距离, $C$: 曲率代价"
    )
    plt.text(0.08, 0.5, txt, fontsize=22, va="center")
    plt.axis("off")
    fig2.savefig(fig_dir / "abc_formula.png", dpi=260, bbox_inches="tight")

    print(f"已生成图片: {fig_dir / 'grid_and_path.png'}")
    print(f"已生成图片: {fig_dir / 'abc_formula.png'}")


if __name__ == "__main__":
    main()

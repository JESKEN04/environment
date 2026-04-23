#!/usr/bin/env python3
"""基于改进人工蜂群算法(ABC)的三维路径规划。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import numpy as np


def load_map(path: Path):
    data = np.load(path)
    occ = data["occupancy"].astype(bool)
    bounds = data["bounds"]
    return occ, bounds


def idx_to_world(bounds: np.ndarray, idx: np.ndarray) -> np.ndarray:
    x_min, _, y_min, _, z_min, _ = bounds
    pts = idx.astype(float)
    pts[:, 0] += x_min
    pts[:, 1] += y_min
    pts[:, 2] += z_min
    return pts


def collision_free(occ: np.ndarray, bounds: np.ndarray, points: np.ndarray) -> bool:
    x_min, x_max, y_min, y_max, z_min, z_max = bounds
    for p in points:
        x, y, z = p
        if not (x_min <= x < x_max and y_min <= y < y_max and z_min <= z < z_max):
            return False
        ix, iy, iz = int(x - x_min), int(y - y_min), int(z - z_min)
        if occ[ix, iy, iz]:
            return False
    return True


def line_sample(a: np.ndarray, b: np.ndarray, step: float = 1.0) -> np.ndarray:
    dist = np.linalg.norm(b - a)
    n = max(2, int(dist / step))
    t = np.linspace(0, 1, n)
    return a[None, :] * (1 - t[:, None]) + b[None, :] * t[:, None]


def fitness(path: np.ndarray, occ_pts: np.ndarray, w=(0.4, 0.4, 0.2)) -> float:
    seg = np.diff(path, axis=0)
    length = np.linalg.norm(seg, axis=1).sum()

    dmin = 1e9
    sample_idx = np.linspace(0, len(occ_pts) - 1, min(1200, len(occ_pts))).astype(int)
    subset = occ_pts[sample_idx]
    for p in path:
        d = np.linalg.norm(subset - p, axis=1).min()
        dmin = min(dmin, d)

    curvature = 0.0
    for i in range(1, len(path) - 1):
        v1 = path[i] - path[i - 1]
        v2 = path[i + 1] - path[i]
        n1 = np.linalg.norm(v1) + 1e-6
        n2 = np.linalg.norm(v2) + 1e-6
        cosang = np.clip(np.dot(v1, v2) / (n1 * n2), -1, 1)
        curvature += np.arccos(cosang)

    return w[0] * length + w[1] * (1.0 / max(dmin, 1e-3)) + w[2] * curvature


def bspline_like_smooth(path: np.ndarray, n_out: int = 120) -> np.ndarray:
    # 无scipy场景下的简化三次样条近似：分段C1连续插值
    t = np.linspace(0, 1, len(path))
    tout = np.linspace(0, 1, n_out)
    out = np.zeros((n_out, 3), dtype=float)
    for k in range(3):
        out[:, k] = np.interp(tout, t, path[:, k])
        out[:, k] = np.convolve(out[:, k], np.array([1, 4, 6, 4, 1]) / 16.0, mode="same")
    out[0] = path[0]
    out[-1] = path[-1]
    return out


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--map", type=Path, default=Path("uav_inspection/data/voxel_map.npz"))
    parser.add_argument("--config", type=Path, default=Path("uav_inspection/worlds/world_config.json"))
    parser.add_argument("--out", type=Path, default=Path("uav_inspection/data/inspection_path.json"))
    parser.add_argument("--iters", type=int, default=500)
    parser.add_argument("--food", type=int, default=40)
    args = parser.parse_args()

    occ, bounds = load_map(args.map)
    cfg = json.loads(args.config.read_text(encoding="utf-8"))

    np.random.seed(42)
    start = np.array([cfg["drones"][0]["x"], cfg["drones"][0]["y"], cfg["drones"][0]["z"]])
    tower_a = np.array([cfg["base_tower"]["x"], cfg["base_tower"]["y"], cfg["base_tower"]["top_z"] + 5.0])
    tower_b = np.array([cfg["peak_tower"]["x"], cfg["peak_tower"]["y"], cfg["peak_tower"]["top_z"] + 5.0])

    occ_pts = idx_to_world(bounds, np.argwhere(occ))

    # 初始化：启发式 + 随机中间控制点
    foods: list[np.ndarray] = []
    heuristic = np.vstack([start, tower_a, tower_b])
    foods.append(heuristic)
    for _ in range(args.food - 1):
        mid = (tower_a + tower_b) / 2 + np.random.uniform(-10, 10, 3)
        mid[2] = np.clip(mid[2], 15, 95)
        foods.append(np.vstack([start, tower_a, mid, tower_b]))

    best = foods[0]
    best_score = 1e18
    stagnant = np.zeros(args.food, dtype=int)

    for _ in range(args.iters):
        for i in range(args.food):
            candidate = foods[i].copy()
            if len(candidate) > 3:
                j = np.random.randint(1, len(candidate) - 1)
                candidate[j] += np.random.uniform(-2, 2, 3)
            ok = True
            for k in range(len(candidate) - 1):
                if not collision_free(occ, bounds, line_sample(candidate[k], candidate[k + 1], 1.0)):
                    ok = False
                    break
            if not ok:
                stagnant[i] += 1
                continue

            s_new = fitness(candidate, occ_pts)
            s_old = fitness(foods[i], occ_pts)
            if s_new < s_old:
                foods[i] = candidate
                stagnant[i] = 0
            else:
                stagnant[i] += 1

            if s_new < best_score:
                best_score = s_new
                best = candidate.copy()

        for i in range(args.food):
            if stagnant[i] > 30:
                foods[i] = np.vstack([
                    start,
                    tower_a,
                    (tower_a + tower_b) / 2 + np.random.uniform(-15, 15, 3),
                    tower_b,
                ])
                stagnant[i] = 0

    smooth = bspline_like_smooth(best)

    payload = {
        "algorithm": "ABC",
        "objective": "f = 0.4*L + 0.4*(1/D_min) + 0.2*C",
        "raw_waypoints": best.tolist(),
        "smoothed_waypoints": smooth.tolist(),
        "best_fitness": float(best_score),
        "inspection_targets": {"tower_A": tower_a.tolist(), "tower_B": tower_b.tolist()},
    }

    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"完成路径规划, best fitness={best_score:.3f}")
    print(f"路径已保存: {args.out}")


if __name__ == "__main__":
    main()

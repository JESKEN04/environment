#!/usr/bin/env python3
"""改进人工蜂群(ABC)三维路径规划 + B样条平滑。"""

from __future__ import annotations

import json
from pathlib import Path
from typing import List, Tuple

try:
    import matplotlib.pyplot as plt
except Exception:
    plt = None
import numpy as np

try:
    from scipy.interpolate import splprep, splev
except Exception:  # scipy可选
    splprep = None
    splev = None


ROOT = Path(__file__).resolve().parents[1]
MAP_DIR = ROOT / "artifacts" / "grid_map"
PLAN_DIR = ROOT / "artifacts" / "planning"
PLAN_DIR.mkdir(parents=True, exist_ok=True)

W1, W2, W3 = 0.4, 0.4, 0.2


def load_data():
    occ = np.load(MAP_DIR / "voxel_map.npz")["occupancy"]
    with (MAP_DIR / "map_metadata.json").open("r", encoding="utf-8") as f:
        meta = json.load(f)
    return occ, meta


def world_to_grid(p, meta):
    res = meta["resolution"]
    x = int((p[0] - meta["bounds"]["x"][0]) / res)
    y = int((p[1] - meta["bounds"]["y"][0]) / res)
    z = int((p[2] - meta["bounds"]["z"][0]) / res)
    return np.array([x, y, z], dtype=int)


def grid_to_world(g, meta):
    res = meta["resolution"]
    return np.array([
        meta["bounds"]["x"][0] + (g[0] + 0.5) * res,
        meta["bounds"]["y"][0] + (g[1] + 0.5) * res,
        meta["bounds"]["z"][0] + (g[2] + 0.5) * res,
    ])


def clamp_grid(g, occ):
    return np.clip(g, [0, 0, 0], np.array(occ.shape) - 1)


def collides(path: np.ndarray, occ: np.ndarray) -> bool:
    for p in path:
        g = clamp_grid(p.astype(int), occ)
        if occ[g[0], g[1], g[2]] > 0:
            return True
    return False


def path_length(path):
    return float(np.sum(np.linalg.norm(np.diff(path, axis=0), axis=1)))


def min_obs_dist(path: np.ndarray, occ: np.ndarray) -> float:
    occ_idx = np.argwhere(occ > 0)
    if len(occ_idx) == 0:
        return 1e3
    sample = occ_idx[:: max(1, len(occ_idx) // 3000)]
    dmin = 1e9
    for p in path:
        d = np.min(np.linalg.norm(sample - p, axis=1))
        dmin = min(dmin, d)
    return float(max(dmin, 1e-3))


def curvature(path: np.ndarray) -> float:
    if len(path) < 3:
        return 0.0
    total = 0.0
    for i in range(1, len(path) - 1):
        v1 = path[i] - path[i - 1]
        v2 = path[i + 1] - path[i]
        n1, n2 = np.linalg.norm(v1), np.linalg.norm(v2)
        if n1 < 1e-6 or n2 < 1e-6:
            continue
        c = np.clip(np.dot(v1, v2) / (n1 * n2), -1.0, 1.0)
        total += np.arccos(c)
    return float(total)


def fitness(path: np.ndarray, occ: np.ndarray) -> float:
    l = path_length(path)
    dmin = min_obs_dist(path, occ)
    c = curvature(path)
    penalty = 10000.0 if collides(path, occ) else 0.0
    return W1 * l + W2 * (1.0 / dmin) + W3 * c + penalty


def smooth_bspline(path: np.ndarray, n_out: int = 180) -> np.ndarray:
    if splprep is None or len(path) < 4:
        return path
    tck, _ = splprep([path[:, 0], path[:, 1], path[:, 2]], s=2.5, k=3)
    u = np.linspace(0.0, 1.0, n_out)
    x, y, z = splev(u, tck)
    return np.vstack([x, y, z]).T


def make_heuristic(start: np.ndarray, goals: np.ndarray) -> np.ndarray:
    # 启发式初始化：按巡检点连线构造初值
    route = [start]
    for g in goals:
        route.append(g)
    return np.array(route, dtype=float)


def abc_plan(occ: np.ndarray, start: np.ndarray, goals: np.ndarray, n_food=30, iters=500):
    rng = np.random.default_rng(2026)
    n_wp = len(goals) + 1

    heuristic = make_heuristic(start, goals)
    foods = []
    foods.append(heuristic.copy())
    for _ in range(n_food - 1):
        noise = rng.normal(0, 3.0, size=heuristic.shape)
        cand = heuristic + noise
        cand[0] = start
        foods.append(cand)

    foods = np.array(foods)
    fit = np.array([fitness(f, occ) for f in foods])
    best_curve = []

    for _ in range(iters):
        # employed bees
        for i in range(n_food):
            k = rng.integers(0, n_food)
            j = rng.integers(1, n_wp)  # 不扰动起点
            phi = rng.uniform(-1.0, 1.0)
            v = foods[i].copy()
            v[j] = v[j] + phi * (v[j] - foods[k][j])
            fv = fitness(v, occ)
            if fv < fit[i]:
                foods[i], fit[i] = v, fv

        # onlooker bees
        prob = (1.0 / (fit + 1e-9))
        prob /= np.sum(prob)
        for _ in range(n_food):
            i = rng.choice(np.arange(n_food), p=prob)
            k = rng.integers(0, n_food)
            j = rng.integers(1, n_wp)
            phi = rng.uniform(-1.0, 1.0)
            v = foods[i].copy()
            v[j] = v[j] + phi * (v[j] - foods[k][j])
            fv = fitness(v, occ)
            if fv < fit[i]:
                foods[i], fit[i] = v, fv

        # scout bee
        worst = int(np.argmax(fit))
        foods[worst] = heuristic + rng.normal(0, 8.0, size=heuristic.shape)
        foods[worst][0] = start
        fit[worst] = fitness(foods[worst], occ)

        best_curve.append(float(np.min(fit)))

    best_i = int(np.argmin(fit))
    return foods[best_i], best_curve


if __name__ == "__main__":
    occ, meta = load_data()
    start = world_to_grid(np.array(meta["start"]), meta)
    goals_world = np.array(meta["targets"][::3])
    goals = np.array([world_to_grid(g, meta) for g in goals_world])

    best_grid, history = abc_plan(occ, start, goals, n_food=30, iters=500)
    best_world = np.array([grid_to_world(p, meta) for p in best_grid])
    smooth_world = smooth_bspline(best_world, n_out=220)

    with (PLAN_DIR / "planned_path.json").open("w", encoding="utf-8") as f:
        json.dump(
            {
                "raw_path": best_world.tolist(),
                "smooth_path": smooth_world.tolist(),
                "fitness_last": history[-1],
                "weights": {"w1": W1, "w2": W2, "w3": W3},
                "iterations": 500,
            },
            f,
            indent=2,
            ensure_ascii=False,
        )

    if plt is not None:
        # 图1: 收敛曲线
        fig, ax = plt.subplots(figsize=(8, 5), dpi=160)
        ax.plot(history, color="#ff6f00", lw=2)
        ax.set_title("改进ABC算法收敛曲线")
        ax.set_xlabel("迭代次数")
        ax.set_ylabel("适应度")
        ax.grid(alpha=0.25)
        fig.tight_layout()
        fig.savefig(PLAN_DIR / "abc_convergence.png")
        plt.close(fig)

        # 图2: 路径俯视图
        fig, ax = plt.subplots(figsize=(10, 8), dpi=160)
        occ_xy = np.max(occ, axis=2).T
        ax.imshow(
            occ_xy,
            origin="lower",
            extent=[meta["bounds"]["x"][0], meta["bounds"]["x"][1], meta["bounds"]["y"][0], meta["bounds"]["y"][1]],
            cmap="cividis",
            alpha=0.7,
        )
        ax.plot(best_world[:, 0], best_world[:, 1], "--", color="#ef5350", lw=1.5, label="原始路径")
        ax.plot(smooth_world[:, 0], smooth_world[:, 1], color="#00e676", lw=2.4, label="B样条平滑路径")
        ax.scatter(meta["start"][0], meta["start"][1], c="#2979ff", s=80, marker="*", label="起点")
        ax.set_title("三维栅格约束下路径规划结果（俯视图）")
        ax.set_xlabel("X / m")
        ax.set_ylabel("Y / m")
        ax.legend()
        ax.grid(alpha=0.2)
        fig.tight_layout()
        fig.savefig(PLAN_DIR / "planned_path_xy.png")
        plt.close(fig)

    print("[OK] 路径规划完成，输出目录:", PLAN_DIR)

#!/usr/bin/env python3
"""改进人工蜂群(ABC)三维路径规划 + 平滑 + 可执行航迹导出。"""

from __future__ import annotations

import csv
import json
import math
from pathlib import Path

import numpy as np

try:
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
except Exception:
    plt = None

ROOT = Path(__file__).resolve().parents[1]
MAP_DIR = ROOT / "artifacts" / "grid_map"
PLAN_DIR = ROOT / "artifacts" / "planning"
PLAN_DIR.mkdir(parents=True, exist_ok=True)

W1, W2, W3 = 0.4, 0.4, 0.2


def _setup_plot_font() -> bool:
    if plt is None:
        return False
    names = {f.name for f in font_manager.fontManager.ttflist}
    for n in ["Noto Sans CJK SC", "Noto Sans CJK JP", "WenQuanYi Micro Hei", "SimHei", "Microsoft YaHei"]:
        if n in names:
            matplotlib.rcParams["font.sans-serif"] = [n, "DejaVu Sans"]
            matplotlib.rcParams["axes.unicode_minus"] = False
            return True
    matplotlib.rcParams["font.sans-serif"] = ["DejaVu Sans"]
    matplotlib.rcParams["axes.unicode_minus"] = False
    return False


def load_data():
    occ = np.load(MAP_DIR / "voxel_map.npz")["occupancy"]
    with (MAP_DIR / "map_metadata.json").open("r", encoding="utf-8") as f:
        meta = json.load(f)
    return occ, meta


def world_to_grid(p, meta):
    res = meta["resolution"]
    return np.array([
        int((p[0] - meta["bounds"]["x"][0]) / res),
        int((p[1] - meta["bounds"]["y"][0]) / res),
        int((p[2] - meta["bounds"]["z"][0]) / res),
    ], dtype=int)


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
    sample = occ_idx[:: max(1, len(occ_idx) // 4000)]
    dmin = 1e9
    for p in path:
        dmin = min(dmin, np.min(np.linalg.norm(sample - p, axis=1)))
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
    penalty = 10000.0 if collides(path, occ) else 0.0
    return W1 * path_length(path) + W2 * (1.0 / min_obs_dist(path, occ)) + W3 * curvature(path) + penalty


def chaikin_smooth(points: np.ndarray, iterations: int = 3) -> np.ndarray:
    out = points.copy()
    for _ in range(iterations):
        new_points = [out[0]]
        for i in range(len(out) - 1):
            p, q = out[i], out[i + 1]
            new_points.append(0.75 * p + 0.25 * q)
            new_points.append(0.25 * p + 0.75 * q)
        new_points.append(out[-1])
        out = np.array(new_points)
    return out


def make_heuristic(start: np.ndarray, goals: np.ndarray) -> np.ndarray:
    return np.array([start, *goals], dtype=float)


def abc_plan(occ: np.ndarray, start: np.ndarray, goals: np.ndarray, n_food=36, iters=500):
    rng = np.random.default_rng(2026)
    n_wp = len(goals) + 1
    heuristic = make_heuristic(start, goals)

    foods = [heuristic.copy()]
    for _ in range(n_food - 1):
        cand = heuristic + rng.normal(0, 3.0, size=heuristic.shape)
        cand[0] = start
        foods.append(cand)
    foods = np.array(foods)
    fit = np.array([fitness(f, occ) for f in foods])
    history = []

    for _ in range(iters):
        for i in range(n_food):
            k = rng.integers(0, n_food)
            j = rng.integers(1, n_wp)
            v = foods[i].copy()
            v[j] = v[j] + rng.uniform(-1.0, 1.0) * (v[j] - foods[k][j])
            fv = fitness(v, occ)
            if fv < fit[i]:
                foods[i], fit[i] = v, fv

        prob = (1.0 / (fit + 1e-9))
        prob /= np.sum(prob)
        for _ in range(n_food):
            i = rng.choice(np.arange(n_food), p=prob)
            k = rng.integers(0, n_food)
            j = rng.integers(1, n_wp)
            v = foods[i].copy()
            v[j] = v[j] + rng.uniform(-1.0, 1.0) * (v[j] - foods[k][j])
            fv = fitness(v, occ)
            if fv < fit[i]:
                foods[i], fit[i] = v, fv

        worst = int(np.argmax(fit))
        foods[worst] = heuristic + rng.normal(0, 8.0, size=heuristic.shape)
        foods[worst][0] = start
        fit[worst] = fitness(foods[worst], occ)
        history.append(float(np.min(fit)))

    return foods[int(np.argmin(fit))], history


def export_mission(path_world: np.ndarray) -> dict:
    # 用于后续 offboard 脚本直接读取
    speed_mps = 8.0
    cumulative_t = 0.0
    rows = []
    for i, p in enumerate(path_world):
        if i == 0:
            yaw = 0.0
            dt = 0.0
        else:
            d = path_world[i] - path_world[i - 1]
            dist = float(np.linalg.norm(d))
            dt = dist / max(speed_mps, 0.1)
            cumulative_t += dt
            yaw = math.atan2(d[1], d[0])
        rows.append([i, round(cumulative_t, 3), float(p[0]), float(p[1]), float(p[2]), float(yaw), 0.0])

    with (PLAN_DIR / "mission_waypoints.csv").open("w", newline="", encoding="utf-8") as f:
        wr = csv.writer(f)
        wr.writerow(["idx", "t_sec", "x", "y", "z", "yaw_rad", "hold_sec"])
        wr.writerows(rows)

    mission = {
        "frame": "map",
        "speed_mps": speed_mps,
        "total_time_sec": rows[-1][1] if rows else 0.0,
        "waypoints": [
            {"idx": int(r[0]), "t_sec": r[1], "x": r[2], "y": r[3], "z": r[4], "yaw": r[5], "hold_sec": r[6]}
            for r in rows
        ],
    }
    with (PLAN_DIR / "mission_waypoints.json").open("w", encoding="utf-8") as f:
        json.dump(mission, f, indent=2, ensure_ascii=False)
    return mission


if __name__ == "__main__":
    occ, meta = load_data()
    start = world_to_grid(np.array(meta["start"]), meta)
    goals = np.array([world_to_grid(g, meta) for g in np.array(meta["targets"][::2])])

    best_grid, history = abc_plan(occ, start, goals, n_food=36, iters=500)
    best_world = np.array([grid_to_world(p, meta) for p in best_grid])
    smooth_world = chaikin_smooth(best_world, iterations=3)

    mission = export_mission(smooth_world)
    with (PLAN_DIR / "planned_path.json").open("w", encoding="utf-8") as f:
        json.dump(
            {
                "raw_path": best_world.tolist(),
                "smooth_path": smooth_world.tolist(),
                "fitness_last": history[-1],
                "weights": {"w1": W1, "w2": W2, "w3": W3},
                "iterations": 500,
                "mission_total_time_sec": mission["total_time_sec"],
            },
            f,
            indent=2,
            ensure_ascii=False,
        )

    if plt is not None:
        cjk = _setup_plot_font()
        title1 = "改进ABC算法收敛曲线" if cjk else "Improved ABC Convergence"
        title2 = "三维栅格约束下路径规划（俯视图）" if cjk else "Path Planning under Voxel Constraints (Top View)"

        fig, ax = plt.subplots(figsize=(8, 5), dpi=180)
        ax.plot(history, color="#ff6f00", lw=2)
        ax.set_title(title1)
        ax.set_xlabel("Iteration")
        ax.set_ylabel("Fitness")
        ax.grid(alpha=0.25)
        fig.tight_layout()
        fig.savefig(PLAN_DIR / "abc_convergence.png")
        plt.close(fig)

        fig, ax = plt.subplots(figsize=(10, 8), dpi=180)
        occ_xy = np.max(occ, axis=2).T
        ax.imshow(occ_xy, origin="lower", extent=[meta["bounds"]["x"][0], meta["bounds"]["x"][1], meta["bounds"]["y"][0], meta["bounds"]["y"][1]], cmap="cividis", alpha=0.75)
        ax.plot(best_world[:, 0], best_world[:, 1], "--", color="#ef5350", lw=1.5, label="raw")
        ax.plot(smooth_world[:, 0], smooth_world[:, 1], color="#00e676", lw=2.4, label="smoothed")
        ax.scatter(meta["start"][0], meta["start"][1], c="#2979ff", s=80, marker="*", label="start")
        ax.set_title(title2)
        ax.set_xlabel("X / m")
        ax.set_ylabel("Y / m")
        ax.legend()
        ax.grid(alpha=0.2)
        fig.tight_layout()
        fig.savefig(PLAN_DIR / "planned_path_xy.png")
        plt.close(fig)

    print("[OK] 路径规划完成，输出目录:", PLAN_DIR)
    print("[INFO] mission total time (s):", mission["total_time_sec"])

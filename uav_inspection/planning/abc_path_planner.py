#!/usr/bin/env python3
"""基于人工蜂群(ABC)算法的三维航迹规划。"""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple

import numpy as np
from scipy.interpolate import splprep, splev


@dataclass
class ABCConfig:
    n_bees: int = 30
    n_points: int = 14
    iterations: int = 500
    limit: int = 40
    w1: float = 0.4
    w2: float = 0.4
    w3: float = 0.2


class ABCPathPlanner:
    def __init__(self, map_npz: str, cfg: ABCConfig | None = None):
        data = np.load(map_npz)
        self.occ = data["occupancy"]
        self.res = float(data["resolution"])
        self.bounds = data["bounds"]
        self.targets = data["targets"]
        self.cfg = cfg or ABCConfig()

    def _world_to_grid(self, p):
        xmin, _, ymin, _, zmin, _ = self.bounds
        return np.array([(p[0] - xmin) / self.res, (p[1] - ymin) / self.res, (p[2] - zmin) / self.res], dtype=np.int32)

    def _is_free(self, p) -> bool:
        g = self._world_to_grid(p)
        if np.any(g < 0) or g[0] >= self.occ.shape[0] or g[1] >= self.occ.shape[1] or g[2] >= self.occ.shape[2]:
            return False
        return self.occ[g[0], g[1], g[2]] == 0

    def _nearest_obstacle_dist(self, p) -> float:
        g = self._world_to_grid(p)
        radius = 6
        min_d = 1e9
        x0, x1 = max(0, g[0] - radius), min(self.occ.shape[0], g[0] + radius + 1)
        y0, y1 = max(0, g[1] - radius), min(self.occ.shape[1], g[1] + radius + 1)
        z0, z1 = max(0, g[2] - radius), min(self.occ.shape[2], g[2] + radius + 1)
        sub = self.occ[x0:x1, y0:y1, z0:z1]
        idxs = np.argwhere(sub > 0)
        if len(idxs) == 0:
            return 20.0
        for idx in idxs:
            obs = np.array([x0, y0, z0]) + idx
            d = np.linalg.norm((obs - g) * self.res)
            min_d = min(min_d, d)
        return float(min_d)

    def _fitness(self, path: np.ndarray) -> float:
        if any(not self._is_free(p) for p in path):
            return 1e6
        seg = np.diff(path, axis=0)
        length = np.sum(np.linalg.norm(seg, axis=1))
        dmin = min(self._nearest_obstacle_dist(p) for p in path)
        curv = 0.0
        for i in range(1, len(path) - 1):
            v1 = path[i] - path[i - 1]
            v2 = path[i + 1] - path[i]
            denom = np.linalg.norm(v1) * np.linalg.norm(v2) + 1e-6
            curv += math.acos(np.clip(np.dot(v1, v2) / denom, -1.0, 1.0))
        c = curv / max(1, len(path) - 2)
        return self.cfg.w1 * length + self.cfg.w2 * (1.0 / max(dmin, 0.2)) + self.cfg.w3 * c

    def _heuristic_seed(self, start, goal) -> np.ndarray:
        line = np.linspace(start, goal, self.cfg.n_points)
        for i in range(1, self.cfg.n_points - 1):
            if not self._is_free(line[i]):
                line[i][2] += 3.0
        return line

    def optimize(self, start: np.ndarray, goal: np.ndarray) -> Tuple[np.ndarray, float]:
        bees = []
        bees.append(self._heuristic_seed(start, goal))
        for _ in range(self.cfg.n_bees - 1):
            rand = np.linspace(start, goal, self.cfg.n_points)
            rand[1:-1] += np.random.uniform([-4, -4, -2], [4, 4, 2], (self.cfg.n_points - 2, 3))
            bees.append(rand)
        bees = np.array(bees)

        fit = np.array([self._fitness(b) for b in bees])
        trials = np.zeros(self.cfg.n_bees, dtype=np.int32)

        for _ in range(self.cfg.iterations):
            for i in range(self.cfg.n_bees):
                k = np.random.choice([x for x in range(self.cfg.n_bees) if x != i])
                phi = np.random.uniform(-1, 1, bees[i].shape)
                cand = bees[i] + phi * (bees[i] - bees[k])
                cand[0], cand[-1] = start, goal
                f = self._fitness(cand)
                if f < fit[i]:
                    bees[i], fit[i], trials[i] = cand, f, 0
                else:
                    trials[i] += 1

            p = (1 / (fit + 1e-6))
            p /= p.sum()
            for _ in range(self.cfg.n_bees):
                i = np.random.choice(self.cfg.n_bees, p=p)
                k = np.random.choice([x for x in range(self.cfg.n_bees) if x != i])
                phi = np.random.uniform(-1, 1, bees[i].shape)
                cand = bees[i] + phi * (bees[i] - bees[k])
                cand[0], cand[-1] = start, goal
                f = self._fitness(cand)
                if f < fit[i]:
                    bees[i], fit[i], trials[i] = cand, f, 0
                else:
                    trials[i] += 1

            for i in range(self.cfg.n_bees):
                if trials[i] > self.cfg.limit:
                    bees[i] = self._heuristic_seed(start, goal)
                    fit[i] = self._fitness(bees[i])
                    trials[i] = 0

        best_idx = int(np.argmin(fit))
        return bees[best_idx], float(fit[best_idx])

    @staticmethod
    def smooth(path: np.ndarray, n_samples: int = 120) -> np.ndarray:
        tck, _ = splprep([path[:, 0], path[:, 1], path[:, 2]], s=2.0, k=min(3, len(path) - 1))
        u = np.linspace(0, 1, n_samples)
        x, y, z = splev(u, tck)
        return np.vstack([x, y, z]).T


def main():
    repo = Path(__file__).resolve().parents[1]
    map_npz = repo / "config" / "voxel_map.npz"
    planner = ABCPathPlanner(str(map_npz))
    start = np.array([-146.32, -85.89, 27.3], dtype=np.float32)
    goal = np.array([-19.29, 23.57, 79.9], dtype=np.float32)
    path, cost = planner.optimize(start, goal)
    smooth = planner.smooth(path)

    out_dir = repo / "config"
    np.save(out_dir / "planned_path_raw.npy", path)
    np.save(out_dir / "planned_path_smooth.npy", smooth)
    with open(out_dir / "planned_path_meta.json", "w", encoding="utf-8") as f:
        json.dump({"cost": cost, "num_points": int(len(smooth))}, f, ensure_ascii=False, indent=2)

    print(f"规划完成，cost={cost:.3f}")


if __name__ == "__main__":
    main()

#!/usr/bin/env python3
"""最小二乘法训练RBF网络（离线脚本模板）。"""

from __future__ import annotations

import json
from pathlib import Path

import numpy as np


def rbf_features(x: np.ndarray, centers: np.ndarray, sigma: float) -> np.ndarray:
    d2 = np.sum((x[:, None, :] - centers[None, :, :]) ** 2, axis=2)
    return np.exp(-d2 / (2 * sigma**2))


def main() -> None:
    # 模板：用随机数据占位，后续替换为ULG解析后的真实误差/参数样本
    rng = np.random.default_rng(42)
    n, in_dim, out_dim, m = 2000, 6, 12, 32
    x = rng.normal(size=(n, in_dim))
    y = rng.normal(size=(n, out_dim))

    centers = x[rng.choice(n, m, replace=False)]
    sigma = float(np.mean(np.std(x, axis=0)) + 1e-6)
    phi = rbf_features(x, centers, sigma)

    # 最小二乘闭式解: W = (Phi^T Phi)^-1 Phi^T Y
    w = np.linalg.pinv(phi.T @ phi) @ phi.T @ y

    out = {
        "centers": centers.tolist(),
        "sigma": sigma,
        "weights": w.tolist(),
        "input_desc": ["ex", "ey", "ez", "evx", "evy", "evz"],
        "output_desc": [
            "kp_pos", "ki_pos", "kd_pos",
            "kp_vel", "ki_vel", "kd_vel",
            "kp_att", "ki_att", "kd_att",
            "kp_rate", "ki_rate", "kd_rate",
        ],
    }

    out_path = Path("uav_inspection/rbf_training/rbf_pid_weights.json")
    out_path.write_text(json.dumps(out, ensure_ascii=False, indent=2), encoding="utf-8")
    print(f"RBF参数已导出: {out_path}")


if __name__ == "__main__":
    main()

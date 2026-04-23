#!/usr/bin/env python3
"""生成论文可直接使用的公式图与参数图（含中文字体回退）。"""

from pathlib import Path

try:
    import matplotlib
    import matplotlib.pyplot as plt
    from matplotlib import font_manager
except Exception:
    plt = None

ROOT = Path(__file__).resolve().parents[1]
FIG_DIR = ROOT / "artifacts" / "figures"
FIG_DIR.mkdir(parents=True, exist_ok=True)


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


def formula_figure(cjk: bool):
    fig, ax = plt.subplots(figsize=(12, 7), dpi=200)
    ax.axis("off")
    formula = (
        r"$f = w_1 L + w_2 \cdot (1/D_{\min}) + w_3 C$" "\n"
        r"$w_1=0.4,\;w_2=0.4,\;w_3=0.2$" "\n"
        r"$\Delta\theta_i = \arccos\left(\frac{(p_i-p_{i-1})\cdot(p_{i+1}-p_i)}{\|p_i-p_{i-1}\|\|p_{i+1}-p_i\|}\right)$" "\n"
        r"$C=\sum_{i=1}^{N-1}\Delta\theta_i$"
    )
    ax.text(0.03, 0.56, formula, fontsize=24, color="#1a237e", va="center")
    desc = "三维栅格分辨率: 1m × 1m × 1m；障碍物膨胀半径: 0.5m" if cjk else "Voxel resolution: 1m x 1m x 1m; inflation: 0.5m"
    ax.text(0.03, 0.12, desc, fontsize=18, color="#004d40")
    fig.tight_layout()
    fig.savefig(FIG_DIR / "abc_formulas.png")
    plt.close(fig)


def radar_figure(cjk: bool):
    labels = ["路径效率", "安全裕量", "平滑性", "收敛速度", "工程可实现性"] if cjk else ["Efficiency", "Safety", "Smoothness", "Convergence", "Feasibility"]
    baseline = [0.92, 0.48, 0.53, 0.55, 0.88]
    improved = [0.80, 0.65, 0.81, 0.78, 0.86]

    import numpy as np

    angles = np.linspace(0, 2 * np.pi, len(labels), endpoint=False).tolist()
    angles += angles[:1]
    baseline += baseline[:1]
    improved += improved[:1]

    fig = plt.figure(figsize=(8, 8), dpi=200)
    ax = plt.subplot(111, polar=True)
    ax.plot(angles, baseline, "--", linewidth=2, color="#ff7043", label="直线路径" if cjk else "Straight")
    ax.fill(angles, baseline, alpha=0.15, color="#ff7043")
    ax.plot(angles, improved, linewidth=2.5, color="#00acc1", label="改进ABC" if cjk else "Improved ABC")
    ax.fill(angles, improved, alpha=0.22, color="#00acc1")
    ax.set_thetagrids([a * 180 / np.pi for a in angles[:-1]], labels)
    ax.set_title("巡检路径性能对比雷达图" if cjk else "Planner Performance Radar", pad=20)
    ax.legend(loc="upper right", bbox_to_anchor=(1.2, 1.15))
    fig.tight_layout()
    fig.savefig(FIG_DIR / "planner_radar.png")
    plt.close(fig)


if __name__ == "__main__":
    if plt is None:
        print("[WARN] matplotlib 未安装，跳过论文图输出")
    else:
        cjk = _setup_plot_font()
        formula_figure(cjk)
        radar_figure(cjk)
        print("[OK] 论文配图输出:", FIG_DIR)

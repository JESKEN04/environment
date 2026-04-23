#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

python3 - <<'PY'
import sys
try:
    import numpy
    print(f"[INFO] numpy={numpy.__version__}")
except Exception as e:
    print(f"[ERROR] numpy not available: {e}")
    sys.exit(1)
PY

python3 "$SCRIPT_DIR/build_voxel_map.py"
python3 "$SCRIPT_DIR/abc_path_planner.py"
python3 "$SCRIPT_DIR/generate_thesis_figures.py"
python3 "$SCRIPT_DIR/visualize_map_and_path.py"

echo "[OK] 第二部分(三维栅格+ABC路径规划+可视化)已完成。"
echo "[OUT] 地图:        $SCRIPT_DIR/../artifacts/grid_map/grid_map_xy.png"
echo "[OUT] 路径:        $SCRIPT_DIR/../artifacts/planning/planned_path_xy.png"
echo "[OUT] 总体可视化:  $SCRIPT_DIR/../artifacts/visualization/map_and_path_3d.png"
echo "[OUT] 飞行航点CSV: $SCRIPT_DIR/../artifacts/planning/mission_waypoints.csv"

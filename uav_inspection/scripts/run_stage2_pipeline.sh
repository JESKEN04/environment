#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

python3 "$SCRIPT_DIR/build_voxel_map.py"
python3 "$SCRIPT_DIR/abc_path_planner.py"
python3 "$SCRIPT_DIR/generate_thesis_figures.py"

echo "[OK] 第二部分(三维栅格+ABC路径规划)已完成。"

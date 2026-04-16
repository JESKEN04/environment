#!/usr/bin/env bash
# ============================================================
# start_simulation.sh
# 无人机巡检仿真一键启动脚本
# 面向巡检任务的无人机编队策略研究
# ============================================================
# Usage:
#   chmod +x start_simulation.sh
#   ./start_simulation.sh
#
# What this does:
#   Window 1: Gazebo simulation world
#   Window 2: MicroXRCE-DDS Agent (ROS2 ↔ PX4 bridge)
#   Window 3: PX4 SITL – UAV1 (Leader)
#   Window 4: PX4 SITL – UAV2 (Follower 1)
#   Window 5: PX4 SITL – UAV3 (Follower 2)
#   (QGroundControl must be started manually)
# ============================================================

set -e

# ─── User-configurable paths ────────────────────────────────
PX4_DIR="${PX4_DIR:-$HOME/PX4-Autopilot}"
ROS2_WS="${ROS2_WS:-$HOME/ros2_ws}"
WORLD_FILE="$ROS2_WS/src/uav_inspection/worlds/inspection_world.sdf"
GZ_RESOURCE="$ROS2_WS/src/uav_inspection/worlds"

# Drone spawn positions (base tower nearby flat area)
# 关键修复：
# 1) 提高初始高度，避免模型与山体网格发生初始碰撞导致“倒着/侧翻”
# 2) 统一偏航朝向（yaw=1.57），让机头朝线路方向，视觉上“正着”出现
# 3) 保持较大机间距，减少初始化阶段互扰
UAV1_POSE="-124.00,-64.00,12.0,0,0,1.57"
UAV2_POSE="-124.00,-59.00,12.0,0,0,1.57"
UAV3_POSE="-124.00,-54.00,12.0,0,0,1.57"

# ─── Terminal emulator detection ────────────────────────────
if command -v gnome-terminal &>/dev/null; then
    TERM_CMD="gnome-terminal --"
elif command -v xterm &>/dev/null; then
    TERM_CMD="xterm -e"
elif command -v konsole &>/dev/null; then
    TERM_CMD="konsole -e"
else
    echo "No supported terminal emulator found."
    echo "Please run each command below in separate terminals:"
    echo ""
    echo "  [1] gz sim -v3 $WORLD_FILE"
    echo "  [2] MicroXRCEAgent udp4 -p 8888"
    echo "  [3] cd $PX4_DIR && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"$UAV1_POSE\" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 1"
    echo "  [4] cd $PX4_DIR && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"$UAV2_POSE\" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 2"
    echo "  [5] cd $PX4_DIR && PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_GZ_MODEL_POSE=\"$UAV3_POSE\" PX4_SIM_MODEL=gz_x500 ./build/px4_sitl_default/bin/px4 -i 3"
    exit 0
fi

# ─── Source ROS2 ────────────────────────────────────────────
ROS_SETUP="/opt/ros/humble/setup.bash"
WS_SETUP="$ROS2_WS/install/setup.bash"

# ─── Verify PX4 is built ────────────────────────────────────
PX4_BIN="$PX4_DIR/build/px4_sitl_default/bin/px4"
if [ ! -f "$PX4_BIN" ]; then
    echo "ERROR: PX4 binary not found: $PX4_BIN"
    echo "Please build PX4 first: cd $PX4_DIR && make px4_sitl"
    exit 1
fi

# ─── Verify world file exists ────────────────────────────────
if [ ! -f "$WORLD_FILE" ]; then
    echo "ERROR: World file not found: $WORLD_FILE"
    echo "Please run generate_world.py first"
    exit 1
fi

echo "═══════════════════════════════════════════════════════════"
echo "  Starting UAV power-line inspection simulation"
echo "  面向巡检任务的无人机编队策略研究"
echo "═══════════════════════════════════════════════════════════"
echo "  World:  $WORLD_FILE"
echo "  PX4:    $PX4_DIR"
echo ""

# ─── [1] Gazebo ─────────────────────────────────────────────
echo "[1/5] Launching Gazebo …"
$TERM_CMD bash -c "
  export GZ_SIM_RESOURCE_PATH=$GZ_RESOURCE:$PX4_DIR/Tools/simulation/gz/models:\$GZ_SIM_RESOURCE_PATH
  export GZ_SIM_SYSTEM_PLUGIN_PATH=$PX4_DIR/build/px4_sitl_default/src/modules/simulation/gz_plugins:\$GZ_SIM_SYSTEM_PLUGIN_PATH
  source $ROS_SETUP 2>/dev/null || true
  [ -f $WS_SETUP ] && source $WS_SETUP
  echo '>>> Gazebo starting with auto-play (-r) …'
  gz sim -v3 -r '$WORLD_FILE'
  exec bash
" &
sleep 10  # Give Gazebo more time to initialize physics engine

# ─── [2] MicroXRCE-DDS Agent ────────────────────────────────
echo "[2/5] Launching MicroXRCE-DDS Agent …"
$TERM_CMD bash -c "
  echo '>>> MicroXRCE-DDS Agent (UDP port 8888)'
  MicroXRCEAgent udp4 -p 8888
  exec bash
" &
sleep 1

# ─── [3] UAV1 – Leader ──────────────────────────────────────
echo "[3/5] Launching PX4 SITL UAV1 (Leader) …"
$TERM_CMD bash -c "
  cd '$PX4_DIR'
  source ./build/px4_sitl_default/rootfs/gz_env.sh
  echo '>>> PX4 SITL – UAV1 (Leader)'
  PX4_GZ_STANDALONE=1 \
  PX4_GZ_WORLD=power_line_inspection \
  PX4_SYS_AUTOSTART=4001 \
  PX4_GZ_MODEL_POSE=\"$UAV1_POSE\" \
  PX4_SIM_MODEL=gz_x500 \
  PX4_GZ_NO_FOLLOW=1 \
  ./build/px4_sitl_default/bin/px4 -i 1
  # 禁用飞行日志：param set SDLOG_MODE 0
  exec bash
" &
sleep 5  # Wait for UAV1 EKF to converge

# ─── [4] UAV2 – Follower 1 ──────────────────────────────────
echo "[4/5] Launching PX4 SITL UAV2 (Follower 1) …"
$TERM_CMD bash -c "
  cd '$PX4_DIR'
  source ./build/px4_sitl_default/rootfs/gz_env.sh
  echo '>>> PX4 SITL – UAV2 (Follower 1)'
  PX4_GZ_STANDALONE=1 \
  PX4_GZ_WORLD=power_line_inspection \
  PX4_SYS_AUTOSTART=4001 \
  PX4_GZ_MODEL_POSE=\"$UAV2_POSE\" \
  PX4_SIM_MODEL=gz_x500 \
  PX4_GZ_NO_FOLLOW=1 \
  ./build/px4_sitl_default/bin/px4 -i 2
  exec bash
" &
sleep 5  # Wait for UAV2 EKF to converge

# ─── [5] UAV3 – Follower 2 ──────────────────────────────────
echo "[5/5] Launching PX4 SITL UAV3 (Follower 2) …"
$TERM_CMD bash -c "
  cd '$PX4_DIR'
  source ./build/px4_sitl_default/rootfs/gz_env.sh
  echo '>>> PX4 SITL – UAV3 (Follower 2)'
  PX4_GZ_STANDALONE=1 \
  PX4_GZ_WORLD=power_line_inspection \
  PX4_SYS_AUTOSTART=4001 \
  PX4_GZ_MODEL_POSE=\"$UAV3_POSE\" \
  PX4_SIM_MODEL=gz_x500 \
  PX4_GZ_NO_FOLLOW=1 \
  ./build/px4_sitl_default/bin/px4 -i 3
  exec bash
" &

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "  All processes started."
echo "  ▶ Start QGroundControl manually and it will auto-connect."
echo "  ▶ Wait 15-20 s for all UAVs to complete initialization."
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "  Drone spawn positions (near base tower, 5m behind):"
echo "    UAV1 (Leader)    : x=-124.00, y=-64.00, z=12.0, yaw=1.57"
echo "    UAV2 (Follower1) : x=-124.00, y=-59.00, z=12.0, yaw=1.57"
echo "    UAV3 (Follower2) : x=-124.00, y=-54.00, z=12.0, yaw=1.57"
echo ""
echo "  Tower positions:"
echo "    Base tower (foot): x=-116.32, y=-52.89, z_terrain=4.00m"
echo "    Peak tower (top) : x=-19.29,  y=23.57,  z_terrain=56.60m"
echo ""
echo "  Power wire heights:"
echo "    Base tower wire  : z=22.30m"
echo "    Peak tower wire  : z=74.90m"
echo ""
echo "  Inspection target: Wires at 5m above wire height"
echo "    Base section: z ≈ 27.30m"
echo "    Peak section: z ≈ 79.90m"
echo ""
echo "  清理日志文件（仿真结束后运行）："
echo "    rm -rf ~/PX4-Autopilot/build/px4_sitl_default/rootfs/[123]/log/*"
echo ""

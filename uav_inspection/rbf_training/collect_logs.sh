#!/usr/bin/env bash
set -e
N=${1:-50}
LOG_DIR=${2:-./logs}
mkdir -p "$LOG_DIR"

echo "开始采集 ${N} 组飞行日志（示例脚本，请按你的PX4启动命令补全）"
for i in $(seq 1 "$N"); do
  echo "[$i/$N] TODO: 启动一次SITL并执行标准巡检机动，结束后拷贝ulg到 $LOG_DIR/flight_$i.ulg"
  sleep 0.1
done

echo "采集流程模板已执行。"

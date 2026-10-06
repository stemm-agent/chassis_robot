#!/bin/bash
# 后台启动运行监控（与 SSH 会话解耦）；返回运行目录
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/hnh/wheeltec_ros2/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
PKG=/home/hnh/wheeltec_ros2/src/stemm_vision_mapping
RUN_DIR=${1:-$HOME/.local/share/stemm/vision_runs/run_$(date +%Y%m%d-%H%M%S)}
mkdir -p "$RUN_DIR"
setsid nohup python3 $PKG/scripts/run_monitor.py --out "$RUN_DIR" \
  > "$RUN_DIR/monitor_stdout.log" 2>&1 < /dev/null &
sleep 3
echo "$RUN_DIR"

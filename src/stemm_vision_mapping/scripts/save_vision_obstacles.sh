#!/bin/bash
# 保存"深度障碍栅格"（激光盲区障碍）为 pgm/yaml（自研保存器，QoS 固定 transient_local+reliable）
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/hnh/wheeltec_ros2/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
OUT_DIR="${1:-$HOME/.local/share/stemm/vision_maps}"
OUT="$OUT_DIR/stemm_vision_obstacles_$(date +%Y%m%d-%H%M%S)"
mkdir -p "$OUT_DIR"
ros2 run stemm_vision_mapping save_grid.py \
  --ros-args -p topic:=/vision/obstacle_grid -p out:="$OUT"
echo "（若有 rtabmap 数据库，建议改用 vision_finalize.sh，从 .db 重建闭环修正后的障碍图）"

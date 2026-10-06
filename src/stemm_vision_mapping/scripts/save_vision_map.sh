#!/bin/bash
# 导出 2D 占据栅格（给导航用）到视觉地图目录；不覆盖现有 cartographer 地图。
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/hnh/wheeltec_ros2/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4

OUT_DIR="${1:-$HOME/.local/share/stemm/vision_maps}"
STAMP=$(date +%Y%m%d-%H%M%S)
OUT="$OUT_DIR/stemm_vision_map_$STAMP"
mkdir -p "$OUT_DIR"

echo "导出 2D 地图 → $OUT.{pgm,yaml}"
ros2 run nav2_map_server map_saver_cli -f "$OUT" \
  --ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=30.0

echo
echo "校验产物:"
ls -l "$OUT".pgm "$OUT".yaml 2>/dev/null
head -8 "$OUT".yaml 2>/dev/null | sed 's/^/  /'
echo
echo "最新视觉地图数据库:"
ls -lt "$OUT_DIR"/*.db 2>/dev/null | head -3

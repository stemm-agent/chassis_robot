#!/bin/bash
# 优雅停止视觉建图：SIGINT → 等待 → SIGTERM → SIGKILL，并校验 /map 已释放。
# 用法: bash scripts/stop_vision_mapping.sh
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/hnh/wheeltec_ros2/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4

PAT='rtabmap_slam/rtabmap|rtabmap_sync/rgbd_sync|rtabmap_viz|point_cloud_xyzrgb|depth_obstacle_grid'
n=$(pgrep -f "$PAT" | wc -l)
if [ "$n" = "0" ]; then echo "没有正在运行的视觉建图节点。"; else
  echo "正在优雅停止 $n 个节点（SIGINT，让 rtabmap 正常写库）..."
  pkill -INT -f "$PAT"
  for i in $(seq 1 10); do
    sleep 1
    [ "$(pgrep -f "$PAT" | wc -l)" = "0" ] && { echo "  已退出（用时 ${i}s）"; break; }
  done
  if [ "$(pgrep -f "$PAT" | wc -l)" != "0" ]; then
    echo "  未退出 → SIGTERM"; pkill -TERM -f "$PAT"; sleep 4
  fi
  if [ "$(pgrep -f "$PAT" | wc -l)" != "0" ]; then
    echo "  仍未退出 → SIGKILL"; pkill -KILL -f "$PAT"; sleep 2
  fi
fi
echo
echo "校验：残留 $(pgrep -f "$PAT" | wc -l) 个进程；/map $(timeout 8 ros2 topic info /map 2>/dev/null | grep -oE 'Publisher count: [0-9]+')"
echo "提示：结束建图后请先运行 scripts/save_vision_map.sh 导出 2D 地图。"

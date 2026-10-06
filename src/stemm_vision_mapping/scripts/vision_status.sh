#!/bin/bash
# 视觉建图现场状态：一次调用打印全部关键指标。
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/hnh/wheeltec_ros2/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4

echo "=== 进程与算力 ==="
ps -o pid,%cpu,%mem,rss,etime,comm -p $(pgrep -d, -f 'rtabmap_slam/rtabmap|rtabmap_sync/rgbd_sync' 2>/dev/null) 2>/dev/null | sed 's/^/  /'
echo "  整机负载: $(cut -d' ' -f1-3 /proc/loadavg)"

echo "=== 同步与输入 ==="
printf "  %-24s %s\n" "/rgbd_image" "$(timeout 4 ros2 topic hz /rgbd_image 2>/dev/null | grep -m1 average | grep -oE '[0-9.]+$') Hz"
printf "  %-24s %s\n" "/scan" "$(timeout 4 ros2 topic hz /scan 2>/dev/null | grep -m1 average | grep -oE '[0-9.]+$') Hz"

echo "=== 里程计位姿（相对建图起点）==="
timeout 6 ros2 run tf2_ros tf2_echo map base_footprint 2>/dev/null | grep -m1 -A1 'Translation' | tr '\n' ' ' | sed 's/^/  /'; echo

echo "=== 地图 ==="
timeout 8 ros2 topic echo /map --once 2>/dev/null | awk '
  /width:/{w=$2} /height:/{h=$2} /resolution:/{r=$2} /x:/{if(!x){x=$2}} /y:/{if(!y){y=$2}}
  END{printf "  栅格: %sx%s @ %s m  原点: (%s, %s)\n", w, h, r, x, y}'
timeout 8 ros2 topic info /map 2>/dev/null | tr '\n' ' ' | sed 's/^/  /'; echo

echo "=== RTAB-Map 统计（节点数 / 回环 / 处理耗时）==="
timeout 8 ros2 topic echo /rtabmap/info --once 2>/dev/null | head -26 | sed 's/^/  /'
echo "  --- 来自日志：工作内存与回环 ---"
LOG=$(ls -t /tmp/vision_*.log 2>/dev/null | head -1)
[ -n "$LOG" ] && grep -oE 'local map=[0-9]+, WM=[0-9]+' "$LOG" | tail -1 | sed 's/^/  工作内存: /'
[ -n "$LOG" ] && grep -c -iE 'loop closure detected' "$LOG" | sed 's/^/  已检测回环次数: /'
echo "=== 数据库 ==="
ls -lt /home/hnh/.local/share/stemm/vision_maps/*.db 2>/dev/null | head -2 | sed 's/^/  /'

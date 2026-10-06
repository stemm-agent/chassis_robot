#!/bin/bash
# 视觉融合建图前置检查：只读，不改任何状态。
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/hnh/wheeltec_ros2/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4

FAIL=0
echo "=== 1. 输入话题 ==="
for t in /scan /odom_combined /camera/color/image_raw /camera/depth/image_raw \
         /camera/color/camera_info /camera/depth/camera_info; do
  R=$(timeout 5 ros2 topic hz $t 2>/dev/null | grep -m1 average | grep -oE '[0-9.]+$')
  if [ -n "$R" ]; then printf "  ✅ %-34s %s Hz\n" "$t" "$R"
  else printf "  ❌ %-34s 无数据\n" "$t"; FAIL=1; fi
done

echo "=== 2. TF 链 ==="
for pair in "odom_combined base_footprint" "base_footprint camera_color_optical_frame" "base_footprint laser"; do
  set -- $pair
  if timeout 6 ros2 run tf2_ros tf2_echo $1 $2 2>/dev/null | grep -qm1 Translation; then
    printf "  ✅ %s → %s\n" "$1" "$2"
  else printf "  ❌ %s → %s 查不到\n" "$1" "$2"; FAIL=1; fi
done

echo "=== 3. 冲突检查（rtabmap 要独占 map→odom 与 /map）==="
PUB=$(timeout 6 ros2 topic info /map 2>/dev/null | grep -oE 'Publisher count: [0-9]+' | grep -oE '[0-9]+')
echo "  /map 当前发布者数: ${PUB:-?}（建图前应为 0）"
if [ "${PUB:-0}" != "0" ]; then
  echo "  ❌ /map 已被占用（有 rtabmap 或 map_server 在发图）→ 直接启动会出现双发布者！"
  echo "     请先执行: bash scripts/stop_vision_mapping.sh  并确认 pgrep -af rtabmap 为空"
  FAIL=1
fi
STALE=$(pgrep -f 'rtabmap_slam/rtabmap|rtabmap_sync/rgbd_sync|point_cloud_xyzrgb' | wc -l)
echo "  残留视觉建图进程数: $STALE（应为 0）"
if [ "$STALE" != "0" ]; then
  echo "  ❌ 检测到残留进程，先执行 scripts/stop_vision_mapping.sh"; FAIL=1
fi
MAPTF=$(timeout 6 ros2 topic echo /tf --once 2>/dev/null | grep -c 'frame_id: map')
echo "  /tf 中 map 帧条目: ${MAPTF:-?}（应为 0）"

echo "=== 4. 结论 ==="
[ $FAIL -eq 0 ] && echo "  ✅ 前置条件满足，可以启动 vision_mapping.launch.py" \
                || echo "  ⚠ 有前置项未通过，请先排查（不要带着缺失启动建图）"
exit $FAIL

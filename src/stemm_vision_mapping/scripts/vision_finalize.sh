#!/bin/bash
# 实车建图收尾一键流程（DB 版）：保存激光图 → 保存实时障碍栅格 → 从 .db 重建闭环修正后的障碍图
# → 投影 → 融合 → 可信度校验（保守图）→ 高亮对照图。只处理并保存，不自动打开任何窗口。
#
# 等价的标准 ROS 2 命令行见每步注释（可直接抄出来单跑）。
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/hnh/wheeltec_ros2/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
PKG=/home/hnh/wheeltec_ros2/src/stemm_vision_mapping
MD=$HOME/.local/share/stemm/vision_maps
TS=$(date +%Y%m%d-%H%M%S)
OUT=$MD/run_$TS
mkdir -p "$OUT/export"
echo "输出目录: $OUT"

pub_count () { timeout -k 2 6 ros2 topic info "$1" 2>/dev/null | grep -oE 'Publisher count: [0-9]+' | grep -oE '[0-9]+'; }

# [1] 激光 2D 图（rtabmap 的 /map）
#     标准命令: ros2 run nav2_map_server map_saver_cli -f <out>/laser_map \
#               --ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=30.0
if [ "$(pub_count /map)" != "0" ]; then
  ros2 run nav2_map_server map_saver_cli -f "$OUT/laser_map" \
    --ros-args -p map_subscribe_transient_local:=true -p save_map_timeout:=30.0 >/dev/null 2>&1 \
    && echo "[1] 激光图已保存: $OUT/laser_map.pgm" || echo "[1] 激光图保存失败"
else
  echo "[1] /map 无发布者（建图未运行）→ 跳过，稍后用数据库/旧图"
fi

# [2] 实时累积的深度障碍栅格（自研保存器）
#     标准命令: ros2 run stemm_vision_mapping save_grid.py \
#               --ros-args -p topic:=/vision/obstacle_grid -p out:=<out>/vision_obstacles_live
if [ "$(pub_count /vision/obstacle_grid)" != "0" ]; then
  ros2 run stemm_vision_mapping save_grid.py \
    --ros-args -p topic:=/vision/obstacle_grid -p out:="$OUT/vision_obstacles_live" 2>&1 | tail -1
else
  echo "[2] /vision/obstacle_grid 无发布者 → 跳过实时栅格（DB 重建可用）"
fi

# [3] 从最新数据库导出闭环修正后的 3D 点云
#     标准命令: /opt/ros/humble/bin/rtabmap-export --cloud --output_dir <out>/export --output run <db>
DB=$(ls -t $MD/*.db 2>/dev/null | head -1)
PLY="$OUT/export/run_cloud.ply"
if [ -n "$DB" ] && [ "${SKIP_DB:-0}" != "1" ]; then
  echo "[3] 导出 3D 点云: $(basename $DB)"
  /opt/ros/humble/bin/rtabmap-export --cloud --output_dir "$OUT/export" --output run "$DB" 2>&1 | tail -1
else
  echo "[3] 跳过数据库导出"
fi

# [4] 投影 + 融合 + 可信度校验
if [ -f "$PLY" ]; then
  python3 $PKG/scripts/cloud_to_grid.py "$PLY" "$OUT/vision_obstacles_db" --res 0.05 --zmin 0.05 --zmax 1.80 --min-pts 2
  V="$OUT/vision_obstacles_db.yaml"
elif [ -f "$OUT/vision_obstacles_live.yaml" ]; then
  V="$OUT/vision_obstacles_live.yaml"; echo "[4] 用实时栅格作为视觉障碍图"
else
  V=$(ls -t $MD/stemm_vision_obstacles_fromdb_*.yaml 2>/dev/null | head -1); echo "[4] 回退到已有的视觉障碍图: $(basename ${V:-无})"
fi

L="$OUT/laser_map.yaml"
[ -f "$L" ] || L=$(ls -t $MD/stemm_vision_map_*.yaml 2>/dev/null | head -1)
if [ -f "$L" ] && [ -n "$V" ] && [ -f "$V" ]; then
  python3 $PKG/scripts/merge_maps.py "$L" "$V" "$OUT/fused"
  python3 $PKG/scripts/grid_confidence.py "$L" "$V" "$OUT/validation" --min-cells 2
  # 高亮图：背景=激光图，红=视觉新增（对比对象必须是"纯视觉障碍图"，不能用融合图）
  python3 $PKG/scripts/make_highlight.py "$L" "$OUT/validation_conservative.yaml" "$OUT/highlight.png"
else
  echo "[4] 缺少激光图或视觉障碍图，跳过融合（L=${L:-无} V=${V:-无}）"
fi

echo
echo "=== 产物（未自动打开任何窗口）==="
ls -1 "$OUT" 2>/dev/null | sed 's/^/  /'
echo
echo "推荐给 nav2 用: $OUT/validation_conservative_fused.yaml"
echo "对照高亮图    : $OUT/highlight.png   （需要时自己打开: eog <路径>）"

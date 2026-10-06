#!/bin/bash
# 视觉融合导航静态验证（不发导航目标）：rtabmap 提供 map→odom 与视觉点云，Nav2 加载视觉图层。
set +u
source /opt/ros/humble/setup.bash 2>/dev/null
source /home/hnh/wheeltec_ros2/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=0 FASTDDS_BUILTIN_TRANSPORTS=UDPv4
PKG=/home/hnh/wheeltec_ros2/src/stemm_vision_mapping
PARAMS=$PKG/config/param_four_wheel_diff_vision.yaml
LAUNCH=/home/hnh/wheeltec_ros2/install/wheeltec_robot_rrt/share/wheeltec_robot_rrt/launch/navigation/nav2_bringup_launch.py
MAPF=/home/hnh/wheeltec_ros2/install/wheeltec_robot_rrt/share/wheeltec_robot_rrt/map_data/image_map.yaml
[ -f "$LAUNCH" ] || { echo "找不到 Nav2 launch: $LAUNCH"; exit 1; }
echo "launch=$LAUNCH"; echo "params=$PARAMS"; echo "map=$MAPF"
exec ros2 launch "$LAUNCH" slam:=True params_file:=$PARAMS map:=$MAPF \
     use_sim_time:=false autostart:=True

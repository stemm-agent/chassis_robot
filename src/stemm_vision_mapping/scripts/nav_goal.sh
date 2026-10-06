#!/bin/bash
# nav_goal.sh -- send / cancel a single-point navigation goal from the command line.
#
#   bash nav_goal.sh --x 1.5 --y 0.0                 # NavigateToPose action (same as RViz "Nav2 Goal")
#   bash nav_goal.sh --x 1.5 --y 0.0 --topic         # publish to /goal_pose (same as RViz "2D Goal Pose"
#                                                    # and the voice stack's command_recognition)
#   bash nav_goal.sh --cancel                        # cancel the current NavigateToPose goal
#   bash nav_goal.sh --status                        # show action servers / clients
set +u   # ROS setup.bash references unbound vars; set -u would abort the script here
source /opt/ros/humble/setup.bash >/dev/null 2>&1
source /home/hnh/wheeltec_ros2/install/setup.bash >/dev/null 2>&1
X=1.0; Y=0.0; YAW=0.0; FRAME=map; MODE=action
while [ $# -gt 0 ]; do
  case "$1" in
    --x) X="$2"; shift 2;;
    --y) Y="$2"; shift 2;;
    --yaw) YAW="$2"; shift 2;;
    --frame) FRAME="$2"; shift 2;;
    --topic) MODE=topic; shift;;
    --cancel) MODE=cancel; shift;;
    --status) MODE=status; shift;;
    *) echo "unknown arg: $1"; exit 2;;
  esac
done
QZ=$(python3 -c "import math;print(math.sin($YAW/2))")
QW=$(python3 -c "import math;print(math.cos($YAW/2))")
case "$MODE" in
  status)
    timeout -k 2 15 ros2 action info /navigate_to_pose 2>/dev/null
    ;;
  cancel)
    timeout -k 2 15 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: $FRAME}, pose: {position: {x: $X, y: $Y}, orientation: {z: $QZ, w: $QW}}}}" >/dev/null 2>&1 &
    sleep 1; pkill -INT -f "action send_goal /navigate_to_pose" 2>/dev/null
    echo "cancel requested (Ctrl-C in the owning terminal / RViz also cancels)"
    ;;
  topic)
    timeout -k 2 10 ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped \
      "{header: {frame_id: $FRAME}, pose: {position: {x: $X, y: $Y, z: 0.0}, orientation: {z: $QZ, w: $QW}}}"
    ;;
  action)
    timeout -k 3 180 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
      "{pose: {header: {frame_id: $FRAME}, pose: {position: {x: $X, y: $Y}, orientation: {z: $QZ, w: $QW}}}}" --feedback
    ;;
esac

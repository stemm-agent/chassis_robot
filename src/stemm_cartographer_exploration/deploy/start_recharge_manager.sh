#!/usr/bin/env bash
set -eo pipefail

WORKSPACE_ROOT=${WORKSPACE_ROOT:-/home/wheeltec/wheeltec_ros2}
START_DELAY=${RECHARGE_MANAGER_START_DELAY:-5}
RECHARGE_EXECUTABLE="${WORKSPACE_ROOT}/install/auto_recharge_ros2/lib/auto_recharge_ros2/recharge_manager"

source /opt/ros/humble/setup.bash
source "${WORKSPACE_ROOT}/install/setup.bash"
cd "${WORKSPACE_ROOT}"

if [ "${START_DELAY}" != "0" ]; then
  sleep "${START_DELAY}"
fi

if [ ! -x "${RECHARGE_EXECUTABLE}" ]; then
  echo "recharge manager executable is missing: ${RECHARGE_EXECUTABLE}" >&2
  exit 127
fi

# Make systemd's MainPID the real ROS node rather than the `ros2 run` launcher.
exec "${RECHARGE_EXECUTABLE}" "$@"

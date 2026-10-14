#!/usr/bin/env bash
set -euo pipefail
MODE="${1:-d0}"
case "$MODE" in
  d0|baseline) CONFIG="delay_baseline.yaml"; ROBOT_CONFIG="fr3_rate_medium.yaml";;
  d1|250ms) CONFIG="delay_250ms.yaml"; ROBOT_CONFIG="fr3_rate_medium.yaml";;
  d2|500ms) CONFIG="delay_500ms.yaml"; ROBOT_CONFIG="fr3_rate_medium.yaml";;
  d3|750ms) CONFIG="delay_750ms.yaml"; ROBOT_CONFIG="fr3_rate_medium.yaml";;
  *) echo "用法: $0 {d0|d1|d2|d3}"; exit 2;;
esac
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"; WS="${ROOT}/ros2"
# ROS setup scripts reference variables that may be unset; keep nounset for
# the launcher itself, but disable it while sourcing the environment.
set +u
source /opt/ros/humble/setup.bash
set -u
if [[ ! -f "${WS}/install/setup.bash" ]]; then echo "请先构建 delay 工作区: cd ${WS} && colcon build --symlink-install" >&2; exit 1; fi
BASE="${GELLO_BASIC_ROOT:-${ROOT%/gello_delay}/gello_basic_launch}/gello_software/ros2"
if [[ ! -f "${BASE}/install/setup.bash" ]]; then echo "找不到 baseline 工作区: ${BASE}" >&2; exit 1; fi
set +u
source "${BASE}/install/setup.bash"
# Delay workspace must be sourced last so its publisher/config overlays baseline.
source "${WS}/install/setup.bash"
set -u
echo "启动真机 GELLO delay=${MODE}"
PIDS=""; cleanup(){ kill ${PIDS:-} 2>/dev/null || true; }; trap cleanup INT TERM EXIT
ros2 launch franka_gello_state_publisher main.launch.py config_file:="${CONFIG}" & PIDS="$!"; sleep 2
ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py robot_config_file:="${ROBOT_CONFIG}" & PIDS="$PIDS $!"; sleep 2
ros2 launch franka_gripper_manager franka_gripper_client.launch.py config_file:=example_fr3_config_franka_hand.yaml & PIDS="$PIDS $!"
wait

#!/usr/bin/env bash
set -euo pipefail
MODE="${1:-d0}"
case "$MODE" in
  d0|baseline) CONFIG="delay_baseline.yaml"; ROBOT_CONFIG="fr3_rate_medium.yaml";;
  d1|200ms) CONFIG="delay_200ms.yaml"; ROBOT_CONFIG="fr3_rate_medium.yaml";;
  d2|400ms) CONFIG="delay_400ms.yaml"; ROBOT_CONFIG="fr3_rate_medium.yaml";;
  d3|600ms) CONFIG="delay_600ms.yaml"; ROBOT_CONFIG="fr3_rate_medium.yaml";;
  *) echo "用法: $0 {d0|d1|d2|d3}"; exit 2;;
esac
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# Only one GELLO publisher may own the Dynamixel serial bus and /gello/joint_states.
LOCK_DIR="/tmp/gello_ros_control.lock"
if ! mkdir "${LOCK_DIR}" 2>/dev/null; then
  echo "检测到已有 GELLO 控制链路，正在切换到 delay=${MODE}..." >&2
  pkill -TERM -f 'ros2 launch franka_gello_state_publisher|ros2 launch franka_fr3_arm_controllers|ros2 launch franka_gripper_manager|gello_publisher' 2>/dev/null || true
  sleep 2
  pkill -KILL -f 'ros2 launch franka_gello_state_publisher|ros2 launch franka_fr3_arm_controllers|ros2 launch franka_gripper_manager|gello_publisher' 2>/dev/null || true
  rm -rf "${LOCK_DIR}"
  mkdir "${LOCK_DIR}"
fi
cleanup_lock(){ rmdir "${LOCK_DIR}" 2>/dev/null || true; }
trap cleanup_lock EXIT
WS="${ROOT}/ros2"
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
# baseline and delay contain the same ROS package name; force delay to win.
DELAY_PREFIX="${WS}/install/franka_gello_state_publisher"
DELAY_BUILD="${WS}/build/franka_gello_state_publisher"
export AMENT_PREFIX_PATH="${DELAY_PREFIX}:${AMENT_PREFIX_PATH:-}"
export PYTHONPATH="${DELAY_BUILD}:${DELAY_PREFIX}/lib/python3.10/site-packages:${PYTHONPATH:-}"
RESOLVED_PREFIX="$(ros2 pkg prefix franka_gello_state_publisher)"
if [[ "${RESOLVED_PREFIX}" != "${DELAY_PREFIX}" ]]; then
  echo "错误：delay 包解析为 ${RESOLVED_PREFIX}，期望 ${DELAY_PREFIX}" >&2
  exit 1
fi
echo "启动真机 GELLO delay=${MODE}; package=${RESOLVED_PREFIX}; config=${CONFIG}"
PIDS=""; cleanup(){ kill ${PIDS:-} 2>/dev/null || true; rmdir "${LOCK_DIR}" 2>/dev/null || true; }; trap cleanup INT TERM EXIT
ros2 launch franka_gello_state_publisher main.launch.py config_file:="${CONFIG}" & PIDS="$!"; sleep 2
ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py robot_config_file:="${ROBOT_CONFIG}" & PIDS="$PIDS $!"; sleep 2
ros2 launch franka_gripper_manager franka_gripper_client.launch.py config_file:=example_fr3_config_franka_hand.yaml & PIDS="$PIDS $!"
wait

#!/usr/bin/env bash
set -euo pipefail
case "${1:-f0}" in
  baselinef0|baseline|f0) CONFIG=force_baselinef0.yaml ;;
  f1) CONFIG=force_f1.yaml ;;
  *) echo "用法: $0 {baselinef0|f0|f1}" >&2; exit 2 ;;
esac
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
LOCK_DIR="/tmp/gello_ros_control.lock"
CONTROL_PROCS='ros2 launch franka_gello_state_publisher|ros2 launch franka_fr3_arm_controllers|ros2 launch franka_gripper_manager|gello_publisher'
# Match delay startup: always clear a previous/incomplete control chain before
# taking ownership, including a stale lock left by an interrupted terminal.
if [[ -d "${LOCK_DIR}" ]] || pgrep -f "${CONTROL_PROCS}" >/dev/null 2>&1; then
  echo "检测到已有 GELLO 控制链路，正在切换到 force..." >&2
  pkill -TERM -f "${CONTROL_PROCS}" 2>/dev/null || true
  sleep 2
  pkill -KILL -f "${CONTROL_PROCS}" 2>/dev/null || true
fi
rm -rf "${LOCK_DIR}"
mkdir "${LOCK_DIR}"
cleanup_lock() { rmdir "${LOCK_DIR}" 2>/dev/null || true; }
trap cleanup_lock EXIT
BASE="${GELLO_BASIC_ROOT:-${ROOT%/gello_force}/gello_basic_launch}/gello_software/ros2"
FRANKA_WS="${FRANKA_ROS2_WS:-${ROOT%/gello_force}/franka_ros2_ws}"
for setup in /opt/ros/humble/setup.bash "$FRANKA_WS/install/local_setup.bash" "$BASE/install/local_setup.bash" "$ROOT/ros2/install/local_setup.bash"; do
  [[ -f "$setup" ]] || { echo "缺少环境文件: $setup；请先构建工作区" >&2; exit 1; }
  set +u
  source "$setup"
  set -u
done
export PYTHONPATH="${ROOT}/ros2/python_deps${PYTHONPATH:+:${PYTHONPATH}}"
FORCE_PREFIX="${ROOT}/ros2/install/franka_gello_state_publisher"
FORCE_BUILD="${ROOT}/ros2/build/franka_gello_state_publisher"
export AMENT_PREFIX_PATH="${FORCE_PREFIX}:${AMENT_PREFIX_PATH:-}"
export PYTHONPATH="${FORCE_BUILD}:${FORCE_PREFIX}/lib/python3.10/site-packages:${PYTHONPATH:-}"
RESOLVED_FORCE_PREFIX="$(ros2 pkg prefix franka_gello_state_publisher)"
if [[ "${RESOLVED_FORCE_PREFIX}" != "${FORCE_PREFIX}" ]]; then
  echo "错误：force 包解析为 ${RESOLVED_FORCE_PREFIX}，期望 ${FORCE_PREFIX}" >&2
  exit 1
fi
# Do not import the ROS Python extension with the user's default `python3`.
# On machines with Conda enabled that may be Python 3.13, while ROS Humble's
# rclpy binary is built for Python 3.10. `ros2 launch` selects the ROS runtime
# itself and is the authoritative startup check.
PIDS=()
cleanup() { if ((${#PIDS[@]})); then kill -INT "${PIDS[@]}" 2>/dev/null || true; fi; rmdir "${LOCK_DIR}" 2>/dev/null || true; }
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM
ros2 launch franka_gello_state_publisher main.launch.py config_file:="$CONFIG" & PIDS+=("$!")
ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py robot_config_file:=fr3_rate_medium.yaml & PIDS+=("$!")
ros2 launch franka_gripper_manager franka_gripper_client.launch.py config_file:=example_fr3_config_franka_hand.yaml & PIDS+=("$!")
echo '等待反馈后保持当前位置；调用 /gello_publisher/start_trial 开始。'
wait -n "${PIDS[@]}"

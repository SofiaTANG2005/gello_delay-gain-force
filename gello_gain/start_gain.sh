#!/usr/bin/env bash
set -euo pipefail
MODE="${1:-g0}"
case "$MODE" in
  g0) CONFIG="gain_g0.yaml";;
  g1) CONFIG="gain_g1.yaml";;
  g2) CONFIG="gain_g2.yaml";;
  g3) CONFIG="gain_g3.yaml";;
  g4) CONFIG="gain_g4.yaml";;
  *) echo "用法: $0 {g0|g1|g2|g3|g4}"; exit 2;;
esac
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${ROOT}/ros2"
if [[ ! -f "${WS}/install/setup.bash" ]]; then
  echo "首次运行请先构建: cd ${WS} && colcon build --symlink-install" >&2; exit 1
fi
set +u
source /opt/ros/humble/setup.bash
source "${WS}/install/local_setup.bash"
set -u
BASE="${GELLO_BASIC_ROOT:-${ROOT%/gello_gain}/gello_basic_launch}/gello_software/ros2"
if [[ ! -f "${BASE}/install/setup.bash" ]]; then echo "找不到 baseline ROS2 工作区: ${BASE}" >&2; exit 1; fi
set +u
source "${BASE}/install/setup.bash"
source "${WS}/install/local_setup.bash"
set -u
# BASE exports another package with the same name.  colcon's unique-path helper
# leaves that older prefix first, so explicitly give this experiment's package
# precedence.
GAIN_PACKAGE_PREFIX="${WS}/install/franka_gello_state_publisher"
export AMENT_PREFIX_PATH="${GAIN_PACKAGE_PREFIX}:${AMENT_PREFIX_PATH}"
# The baseline workspace also exports the same Python package.  Put both the
# symlink-build source and installed module for this experiment first so the
# console-script entry point cannot import the baseline implementation.
GAIN_PYTHON_BUILD="${WS}/build/franka_gello_state_publisher"
GAIN_PYTHON_INSTALL="${GAIN_PACKAGE_PREFIX}/lib/python3.10/site-packages"
export PYTHONPATH="${GAIN_PYTHON_BUILD}:${GAIN_PYTHON_INSTALL}:${PYTHONPATH:-}"

# Refuse to start the real robot unless the gain package/config resolves to
# this workspace.
RESOLVED_GELLO_PREFIX="$(ros2 pkg prefix franka_gello_state_publisher)"
EXPECTED_CONFIG="${RESOLVED_GELLO_PREFIX}/share/franka_gello_state_publisher/config/${CONFIG}"
if [[ "${RESOLVED_GELLO_PREFIX}" != "${GAIN_PACKAGE_PREFIX}" || ! -f "${EXPECTED_CONFIG}" ]]; then
  echo "增益 GELLO 包解析错误: ${RESOLVED_GELLO_PREFIX}" >&2
  echo "找不到配置: ${EXPECTED_CONFIG}" >&2
  exit 1
fi
RESOLVED_GELLO_MODULE="$(python3 -c 'import importlib.util; print(importlib.util.find_spec("franka_gello_state_publisher.gello_publisher").origin)')"
if [[ "${RESOLVED_GELLO_MODULE}" != "${GAIN_PYTHON_BUILD}"/* ]]; then
  echo "增益 GELLO Python 模块解析错误: ${RESOLVED_GELLO_MODULE}" >&2
  exit 1
fi
echo "GELLO 配置: ${CONFIG}; Python 模块: ${RESOLVED_GELLO_MODULE}"
echo "启动真机 GELLO + FR3, gain=${MODE}; Ctrl-C 退出"
cleanup() { kill ${PIDS:-} 2>/dev/null || true; }
trap cleanup INT TERM EXIT
ros2 launch franka_gello_state_publisher main.launch.py config_file:="${CONFIG}" & PIDS="$!"
sleep 2
if ! kill -0 "${PIDS}" 2>/dev/null; then
  echo "GELLO 启动失败，已取消启动 FR3。" >&2
  wait "${PIDS}"
  exit 1
fi
ros2 launch franka_fr3_arm_controllers franka_fr3_arm_controllers.launch.py robot_config_file:=fr3_rate_medium.yaml & PIDS="$PIDS $!"
sleep 2
ros2 launch franka_gripper_manager franka_gripper_client.launch.py config_file:=example_fr3_config_franka_hand.yaml & PIDS="$PIDS $!"
wait

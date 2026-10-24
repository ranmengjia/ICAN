#!/usr/bin/env bash

if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then
  echo "请使用 source 加载环境："
  echo "  source scripts/setup_env.sh"
  exit 1
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
ROS_SETUP="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
BINARY_SETUP="${WORKSPACE_DIR}/binary_install/setup.bash"
WORKSPACE_SETUP="${WORKSPACE_DIR}/install/setup.bash"

if [[ ! -f "${ROS_SETUP}" ]]; then
  echo "未找到 ${ROS_SETUP}，请先运行 bash scripts/00_install_ros2_gazebo.sh" >&2
  return 1
fi

if [[ ! -f "${BINARY_SETUP}" ]]; then
  echo "未找到 ${BINARY_SETUP}，请先运行 bash scripts/01_setup_inspection_workspace.sh" >&2
  return 1
fi

if [[ ! -f "${WORKSPACE_SETUP}" ]]; then
  echo "未找到 ${WORKSPACE_SETUP}，请先运行 bash scripts/01_setup_inspection_workspace.sh" >&2
  return 1
fi

set +u
# shellcheck disable=SC1090
source "${ROS_SETUP}"
# shellcheck disable=SC1090
source "${BINARY_SETUP}"
# shellcheck disable=SC1090
source "${WORKSPACE_SETUP}"

export ICAN_INSPECTION_WS="${WORKSPACE_DIR}"

# 让 Gazebo 找到自定义材质纹理（窗口目标照片等），否则目标会渲染成灰板。
# 注意：GAZEBO_RESOURCE_PATH 一旦设置会顶替 Gazebo 默认资源目录，
# 必须同时带上 /usr/share/gazebo-11，否则核心渲染资源（shadow_caster 材质、
# RTShader 库）丢失，导致 gzclient 窗口创建失败、相机插件不推流。
GAZEBO_SIM_SHARE="${WORKSPACE_DIR}/binary_install/share/ican_inspection_sim"
if [[ -d "${GAZEBO_SIM_SHARE}" ]]; then
  export GAZEBO_RESOURCE_PATH="${GAZEBO_SIM_SHARE}:/usr/share/gazebo-11${GAZEBO_RESOURCE_PATH:+:${GAZEBO_RESOURCE_PATH}}"
fi

echo "已加载 ROS 2 Humble 和 iCAN 任务一巡检仿真工作区。"

#!/usr/bin/env bash
# myAGV Pro：底盘驱动 + 整机模型(底盘+Pro450) + Orbbec 建图 一键启动。
# 遥控只驱动底盘；机械臂在 RViz 中以收拢/零位显示，便于看清在控什么。
#
# Usage:
#   ros2 run pro450_sim pro450_myagv_slam_mapping          # 一键：底盘+整机显示+相机+建图
#   ros2 run pro450_sim pro450_myagv_slam_mapping teleop   # 另开终端遥控底盘
#   ros2 run pro450_sim pro450_myagv_slam_mapping save NAME
#   ros2 run pro450_sim pro450_myagv_slam_mapping check

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
WS="${HOME}/projects/ros2_ws"
if [[ ! -f "${WS}/install/setup.bash" ]]; then
  d="${SCRIPT_DIR}"
  WS=""
  for _ in 1 2 3 4 5 6; do
    d="$(cd "${d}/.." && pwd)"
    if [[ -f "${d}/install/setup.bash" ]]; then
      WS="${d}"
      break
    fi
  done
fi
if [[ -z "${WS}" || ! -f "${WS}/install/setup.bash" ]]; then
  echo "[slam] ERROR: cannot find ros2_ws/install/setup.bash"
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u

MAP_DIR="${WS}/maps/pro450_myagv"
mkdir -p "${MAP_DIR}"

have_pkg() { ros2 pkg prefix "$1" >/dev/null 2>&1; }
have_topic() { ros2 topic list 2>/dev/null | grep -qx "$1"; }

cmd="${1:-all}"

case "${cmd}" in
  all|mapping|"")
    echo "[slam] 一键启动: 底盘驱动 + myAGV/Pro450 整机显示 + Orbbec + SLAM"
    echo "[slam] 说明: teleop 只控底盘；臂在 RViz 以零位显示（本模式不启臂驱动）"
    if ! have_pkg agv_pro_base; then
      echo "[slam] ERROR: agv_pro_base 不可见。请在本终端执行:"
      echo "  source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash"
      exit 1
    fi
    if ! have_pkg pro450_description; then
      echo "[slam] ERROR: pro450_description 不可见"
      exit 1
    fi
    if ! have_pkg orbbec_camera; then
      echo "[slam] ERROR: orbbec_camera 不可见。请 colcon build --packages-up-to orbbec_camera"
      exit 1
    fi
    echo "[slam] 另开终端遥控:  $0 teleop"
    echo "[slam] 存图:          $0 save atrium"
    exec ros2 launch pro450_sim pro450_myagv_slam.launch.py \
      start_bringup:=true show_manipulator:=true start_camera:=true \
      enable_lidar:=false rviz:=true
    ;;

  teleop)
    exec ros2 run teleop_twist_keyboard teleop_twist_keyboard
    ;;

  save)
    name="${2:-myagv_orbbec}"
    out="${MAP_DIR}/${name}"
    if ! have_topic /map; then
      echo "[slam] ERROR: /map 未发布"
      exit 1
    fi
    echo "[slam] Saving ${out}.{pgm,yaml}"
    ros2 run nav2_map_server map_saver_cli -f "${out}" --ros-args -p save_map_timeout:=30.0
    ;;

  check)
    echo "[slam] WS=${WS}"
    echo "[slam] packages:"
    for p in agv_pro_base agv_pro_bringup pro450_description orbbec_camera rtabmap_slam slam_toolbox depthimage_to_laserscan; do
      if have_pkg "$p"; then echo "  OK  $p"; else echo "  --  $p"; fi
    done
    echo "[slam] topics:"
    for t in /odom /cmd_vel /robot_description /camera/color/image_raw /camera/depth/image_raw /scan /map; do
      if have_topic "$t"; then echo "  OK  $t"; else echo "  --  $t"; fi
    done
    ;;

  *)
    cat <<EOF
Usage: $0 [all|teleop|save NAME|check]

  all      底盘+整机(RViz含臂)+相机+建图（默认）
  teleop   键盘遥控底盘
  save n   保存地图
  check    检查依赖
EOF
    exit 1
    ;;
esac

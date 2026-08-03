#!/usr/bin/env bash
# 回字形走廊仿真移动 SLAM 三段工作流：
#   1) all     — Gazebo 环境 + 整机(底盘+臂) + 底盘 3D 相机 + SLAM + RViz(含墙)
#   2) teleop  — 键盘遥控探索
#   3) save    — 保存 /map
#
# Usage:
#   ros2 run pro450_sim pro450_myagv_atrium_slam
#   ros2 run pro450_sim pro450_myagv_atrium_slam teleop
#   ros2 run pro450_sim pro450_myagv_atrium_slam save atrium_map
#   ros2 run pro450_sim pro450_myagv_atrium_slam check

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
  echo "[atrium_slam] ERROR: cannot find ros2_ws/install/setup.bash"
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u

# Keep this self-contained demo away from other Gazebo /clock publishers.
# All subcommands use the same defaults, so teleop and save still discover the
# mapping launch.  Override only when deliberate integration needs another ID.
export ROS_DOMAIN_ID="${PRO450_ATRIUM_ROS_DOMAIN_ID:-45}"
export IGN_PARTITION="${PRO450_ATRIUM_IGN_PARTITION:-pro450_atrium_demo}"

MAP_DIR="${WS}/maps/pro450_myagv"
mkdir -p "${MAP_DIR}"

have_pkg() { ros2 pkg prefix "$1" >/dev/null 2>&1; }
have_topic() { ros2 topic list 2>/dev/null | grep -qx "$1"; }

cmd="${1:-all}"

case "${cmd}" in
  all|mapping|"")
    clock_info="$(ros2 topic info --no-daemon /clock 2>/dev/null || true)"
    if echo "${clock_info}" | grep -Eq 'Publisher count: [1-9]'; then
      echo "[atrium_slam] ERROR: domain ${ROS_DOMAIN_ID} already has a /clock publisher."
      echo "  Stop the previous atrium SLAM/patrol launch before starting another."
      exit 1
    fi
    echo "[atrium_slam] 启动: 回字走廊 Gazebo + 整机 + 底盘RGB-D SLAM + RViz墙体"
    echo "[atrium_slam] ROS_DOMAIN_ID=${ROS_DOMAIN_ID} IGN_PARTITION=${IGN_PARTITION}"
    echo "[atrium_slam] 另开终端遥控:  $0 teleop"
    echo "[atrium_slam] 存图:          $0 save atrium_map"
    if ! have_pkg pro450_sim; then
      echo "[atrium_slam] ERROR: pro450_sim 不可见，请 source install/setup.bash"
      exit 1
    fi
    exec ros2 launch pro450_sim pro450_myagv_atrium_slam.launch.py \
      gui:=true rviz:=true backend:=auto
    ;;

  teleop)
    echo "[atrium_slam] 键盘遥控底盘 (i/j/k/l/,) — 只控移动，机械臂保持零位显示"
    echo "[atrium_slam] ROS_DOMAIN_ID=${ROS_DOMAIN_ID}"
    exec ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p stamped:=false
    ;;

  save)
    name="${2:-atrium_map}"
    out="${MAP_DIR}/${name}"
    if ! have_topic /map; then
      echo "[atrium_slam] ERROR: /map 未发布，请先完成建图探索"
      exit 1
    fi
    echo "[atrium_slam] Saving ${out}.{pgm,yaml}"
    ros2 run nav2_map_server map_saver_cli -f "${out}" --ros-args -p save_map_timeout:=30.0
    echo "[atrium_slam] done: ${out}.yaml"
    ;;

  check)
    echo "[atrium_slam] WS=${WS}"
    echo "[atrium_slam] packages:"
    for p in pro450_sim pro450_description ros_gz_sim ros_gz_bridge \
             slam_toolbox depthimage_to_laserscan rtabmap_slam nav2_map_server; do
      if have_pkg "$p"; then echo "  OK  $p"; else echo "  --  $p"; fi
    done
    echo "[atrium_slam] topics:"
    for t in /odom /cmd_vel /robot_description /camera/color/image_raw \
             /camera/depth/image_raw /scan /map \
             /atrium_plant_markers /atrium_wall_markers; do
      if have_topic "$t"; then echo "  OK  $t"; else echo "  --  $t"; fi
    done
    ;;

  *)
    cat <<EOF
Usage: $0 [all|teleop|save NAME|check]

  all      回字环境 + 整机 + 底盘3D相机建图 + Gazebo/RViz（默认）
  teleop   键盘遥控探索
  save n   保存地图到 maps/pro450_myagv/n
  check    检查依赖与话题
EOF
    exit 1
    ;;
esac

#!/usr/bin/env bash
# 回字走廊一圈巡检：沿走廊行驶，每经过一盆停 5 秒（机械臂不动）。
#
# Usage:
#   ros2 run pro450_sim pro450_myagv_atrium_patrol_bringup
#   ros2 run pro450_sim pro450_myagv_atrium_patrol_bringup map /path/to/atrium_map.yaml

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
  echo "[atrium_patrol] ERROR: cannot find ros2_ws/install/setup.bash"
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u

export ROS_DOMAIN_ID="${PRO450_ATRIUM_ROS_DOMAIN_ID:-45}"
export IGN_PARTITION="${PRO450_ATRIUM_IGN_PARTITION:-pro450_atrium_demo}"

cmd="${1:-all}"
if [[ "${cmd}" == "map" && -n "${2:-}" ]]; then
  MAP="$2"
  cmd="all"
fi

case "${cmd}" in
  all|"")
    clock_info="$(ros2 topic info --no-daemon /clock 2>/dev/null || true)"
    if echo "${clock_info}" | grep -Eq 'Publisher count: [1-9]'; then
      echo "[atrium_patrol] ERROR: domain ${ROS_DOMAIN_ID} already has a /clock publisher."
      echo "  Stop the previous atrium SLAM/patrol launch before starting another."
      exit 1
    fi
    launch_args=(auto_start:=true gui:=true rviz:=true)
    if [[ -n "${MAP:-}" ]]; then
      if [[ ! -f "${MAP}" ]]; then
        echo "[atrium_patrol] ERROR: map not found: ${MAP}"
        exit 1
      fi
      launch_args+=(map:="${MAP}")
      echo "[atrium_patrol] custom map=${MAP}"
    else
      echo "[atrium_patrol] map=clean deterministic atrium map"
    fi
    echo "[atrium_patrol] Nav2: atrium_map + global/local plan; 5s plant dwells; arm idle"
    echo "[atrium_patrol] ROS_DOMAIN_ID=${ROS_DOMAIN_ID} IGN_PARTITION=${IGN_PARTITION}"
    echo "[atrium_patrol] Wait ~25s for Gazebo+Nav2, then robot moves (RViz shows /plan + /local_plan)"
    exec ros2 launch pro450_sim pro450_myagv_atrium_patrol.launch.py \
      "${launch_args[@]}"
    ;;
  *)
    cat <<EOF
Usage: $0 [all]
       $0 map /path/to/atrium_map.yaml
EOF
    exit 1
    ;;
esac

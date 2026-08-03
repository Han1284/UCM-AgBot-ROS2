#!/usr/bin/env bash
# Start Gazebo/Nav2/controllers/RViz and the eight numbered plants only.

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
  echo "[leaf_environment] ERROR: cannot find ros2_ws/install/setup.bash"
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

clock_info="$(ros2 topic info --no-daemon /clock 2>/dev/null || true)"
if echo "${clock_info}" | grep -Eq 'Publisher count: [1-9]'; then
  echo "[leaf_environment] ERROR: domain ${ROS_DOMAIN_ID} already has an environment."
  echo "  Stop the previous environment before starting another."
  exit 1
fi

echo "[leaf_environment] Starting Gazebo + Nav2 + controllers + numbered plants + RViz."
echo "[leaf_environment] Keep this terminal running, then start the mission script in terminal 2."
exec ros2 launch pro450_sim pro450_myagv_leaf_environment.launch.py \
  gui:=true rviz:=true

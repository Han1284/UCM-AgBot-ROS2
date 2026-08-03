#!/usr/bin/env bash
# Interactive mission for an already running leaf-inspection environment.

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
  echo "[leaf_inspection] ERROR: cannot find ros2_ws/install/setup.bash"
  exit 1
fi

set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
# shellcheck disable=SC1091
source "${WS}/install/setup.bash"
set -u

echo "可选植物："
echo "  1: 北侧东株    2: 北侧西株    3: 南侧东株    4: 南侧西株"
echo "  5: 东侧北株    6: 东侧南株    7: 西侧北株    8: 西侧南株"
while true; do
  read -r -p "请输入要检查的植物编号 [1-8]: " plant_id
  if [[ "${plant_id}" =~ ^[1-8]$ ]]; then
    break
  fi
  echo "输入无效，请输入 1 到 8。"
done

export ROS_DOMAIN_ID="${PRO450_ATRIUM_ROS_DOMAIN_ID:-45}"
export IGN_PARTITION="${PRO450_ATRIUM_IGN_PARTITION:-pro450_atrium_demo}"

exec 9>/tmp/pro450_myagv_leaf_mission.lock
if ! flock -n 9; then
  echo "[leaf_inspection] ERROR: another inspection mission is already running."
  exit 1
fi

clock_info="$(ros2 topic info --no-daemon /clock 2>/dev/null || true)"
if ! echo "${clock_info}" | grep -Eq 'Publisher count: [1-9]'; then
  echo "[leaf_inspection] ERROR: inspection environment is not running."
  echo "  Start pro450_myagv_leaf_environment in terminal 1 first."
  exit 1
fi

echo "[leaf_inspection] Waiting for Nav2 and arm controller actions..."
ready=false
for _ in $(seq 1 120); do
  actions="$(ros2 action list 2>/dev/null || true)"
  if echo "${actions}" | grep -qx '/navigate_to_pose' && \
     echo "${actions}" | grep -qx \
       '/arm_trajectory_controller/follow_joint_trajectory'; then
    ready=true
    break
  fi
  sleep 1
done
if [[ "${ready}" != true ]]; then
  echo "[leaf_inspection] ERROR: environment did not become ready within 120 seconds."
  exit 1
fi

echo "[leaf_inspection] plant=${plant_id}, execute=true, return_base_home=true"
exec ros2 launch pro450_sim pro450_myagv_leaf_mission.launch.py \
  plant_id:="${plant_id}"

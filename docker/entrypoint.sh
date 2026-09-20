#!/usr/bin/env bash
set -eo pipefail
umask 000

# ROS setup files reference unset variables; do not run them under `set -u`.
set +u
source /opt/ros/jazzy/setup.bash
source /edge_ws/doosan_ws/install/setup.bash
source /edge_ws/install/setup.bash
set -u

export ADAPTIVE_SCANNING_WS_ROOT="${ADAPTIVE_SCANNING_WS_ROOT:-/edge_ws}"
export ADAPTIVE_SCANNING_SIM_LOG_DIR="${ADAPTIVE_SCANNING_SIM_LOG_DIR:-/runtime}"
export SIM_LOG_DIR="${SIM_LOG_DIR:-${ADAPTIVE_SCANNING_SIM_LOG_DIR}/sim/logs}"
export SIM_RUN_DIR="${SIM_RUN_DIR:-${ADAPTIVE_SCANNING_SIM_LOG_DIR}/sim/run}"
mkdir -p "${SIM_LOG_DIR}" "${SIM_RUN_DIR}"

echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-unset}"
echo "ADAPTIVE_SCANNING_WS_ROOT=${ADAPTIVE_SCANNING_WS_ROOT}"
echo "ADAPTIVE_SCANNING_SIM_LOG_DIR=${ADAPTIVE_SCANNING_SIM_LOG_DIR}"
echo "SIM_GZ_GUI=${SIM_GZ_GUI:-false}"

# The robot container reads dataset_dir from the shared runtime volume, because
# it has no as_sim package of its own.
dataset_src="$(ros2 pkg prefix as_sim 2>/dev/null)/share/as_sim/worlds/Dataset"
if [ -d "${dataset_src}" ]; then
  mkdir -p "${ADAPTIVE_SCANNING_SIM_LOG_DIR}/sim"
  cp -rn "${dataset_src}" "${ADAPTIVE_SCANNING_SIM_LOG_DIR}/sim/" 2>/dev/null || true
fi

if [ "$#" -eq 0 ]; then
  set -- gazebo
fi
exec /opt/adaptive_scanning/sim/sim.sh "$@"

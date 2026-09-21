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

# sim.sh starts Gazebo in its own process group and then returns so that the
# host workflow can launch later stages independently. In Compose this is the
# container's only long-lived workload: keep PID 1 alive while that stage runs
# and use sim.sh's normal cleanup ladder on container shutdown.
if [ "$1" = "gazebo" ]; then
  stage_pid_file="${SIM_RUN_DIR}/gazebo.pid"
  # This container owns the gazebo stage outright, so a pid file on the shared
  # /runtime volume is always a leftover from a container that was killed
  # before its cleanup ran. Left in place it can trip start_stage's "already
  # running" branch and abort us under `set -e`.
  rm -f "${stage_pid_file}"

  /opt/adaptive_scanning/sim/sim.sh "$@"
  if [ ! -s "${stage_pid_file}" ]; then
    echo "simulation gazebo stage did not create ${stage_pid_file}" >&2
    exit 1
  fi
  # start_stage writes "<pid> <start-ticks>"; only the first field is the pid.
  read -r stage_pid _ < "${stage_pid_file}" || true
  case "${stage_pid:-}" in
    '' | *[!0-9]*)
      echo "simulation gazebo stage wrote an unusable pid file: ${stage_pid_file}" >&2
      exit 1
      ;;
  esac

  cleanup() {
    /opt/adaptive_scanning/sim/sim.sh down || true
  }
  # A stop signal is a clean shutdown; the stage dying on its own is not.
  on_signal() {
    trap - EXIT INT TERM
    cleanup
    exit 0
  }
  trap cleanup EXIT
  trap on_signal INT TERM
  while kill -0 "${stage_pid}" 2>/dev/null; do
    sleep 1
  done
  # Reached only when the stage died by itself: report failure rather than
  # letting Compose read a crashed Gazebo as a clean exit. The EXIT trap still
  # runs sim.sh down to reap whatever is left of the stage.
  echo "simulation gazebo stage exited unexpectedly; see ${SIM_LOG_DIR}/gazebo.log" >&2
  exit 1
fi

exec /opt/adaptive_scanning/sim/sim.sh "$@"

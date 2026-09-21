#!/usr/bin/env bash
# Shared helpers for the AdaptiveScanning Gazebo simulation scripts.
# Source this from the stage scripts; do not execute directly.

# --- paths -------------------------------------------------------------------
SIM_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SIM_REPO_ROOT="$(cd "${SIM_DIR}/.." && pwd)"             # .../src/adaptive_scanning_simulation
WS_ROOT="${ADAPTIVE_SCANNING_WS_ROOT:-$(cd "${SIM_REPO_ROOT}/../.." && pwd)}"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"

# Runtime/output root. All runtime artifacts (table.ply, run logs, tsdf, ...)
# live here. Default to the in-repo scan_logs/ so nothing needs /scan_logs.
RUNTIME_ROOT="${ADAPTIVE_SCANNING_SIM_LOG_DIR:-${SIM_REPO_ROOT}/scan_logs}"

SIM_LOG_DIR="${SIM_LOG_DIR:-${SIM_DIR}/logs}"
SIM_RUN_DIR="${SIM_RUN_DIR:-${SIM_DIR}/run}"              # pid files
SIM_PARAMS="${SIM_PARAMS:-${SIM_RUN_DIR}/sim_scan_params.yaml}"
SIM_DOCKER_PARAMS_HOST="${RUNTIME_ROOT}/sim/sim_scan_params.yaml"
SIM_DOCKER_PARAMS_CONTAINER="/scan_logs/sim/sim_scan_params.yaml"
TABLE_PLY="${RUNTIME_ROOT}/Calibration/table.ply"

# --- tunables (override via env) --------------------------------------------
SIM_NAME="${SIM_NAME:-dsr01}"
SIM_ROBOT_NS="${SIM_ROBOT_NS:-/dsr01}"
SIM_PART="${SIM_PART-1}"             # Dataset/<part>.stl spawned in Gazebo ("" = none)
# The stable validation path is headless. GUI/RViz are opt-in diagnostics: on
# low-resource hosts their render sinks can starve gz physics and make /clock
# non-monotonic.
SIM_GZ_GUI="${SIM_GZ_GUI:-false}"
SIM_USE_LOOKAT="${SIM_USE_LOOKAT:-true}"  # exercise camera look-at scan
SIM_OBJECT="${SIM_OBJECT:-}"         # Optional STL filename/absolute path override
SIM_OBJ_X="${SIM_OBJ_X:-0.65}"
SIM_OBJ_Y="${SIM_OBJ_Y:-0.0}"
SIM_OBJ_Z="${SIM_OBJ_Z:-0.0}"
SIM_OBJ_R="${SIM_OBJ_R:-0.0}"
SIM_OBJ_P="${SIM_OBJ_P:-0.0}"
SIM_OBJ_YAW="${SIM_OBJ_YAW:-0.0}"
SIM_OBJ_SCALE="${SIM_OBJ_SCALE:-0.001}"
SIM_OBJ_PLACE_ON_FLOOR="${SIM_OBJ_PLACE_ON_FLOOR:-true}"
SIM_OBJ_FLOOR_CLEARANCE_M="${SIM_OBJ_FLOOR_CLEARANCE_M:-0.02}"
SIM_FLOOR_Z="${SIM_FLOOR_Z:-0.0}"
SIM_BASE_FLOOR_Z_M="${SIM_BASE_FLOOR_Z_M:--0.12}"
SIM_OBJECT_POSE_SOURCE_FRAME="${SIM_OBJECT_POSE_SOURCE_FRAME:-gazebo_world}"

# --- colours / logging -------------------------------------------------------
if [ -t 1 ]; then
  C_BLUE='\033[1;34m'; C_GRN='\033[1;32m'; C_YEL='\033[1;33m'; C_RED='\033[1;31m'; C_RST='\033[0m'
else
  C_BLUE=''; C_GRN=''; C_YEL=''; C_RED=''; C_RST=''
fi
log()  { echo -e "${C_BLUE}[sim]${C_RST} $*"; }
ok()   { echo -e "${C_GRN}[sim] OK${C_RST} $*"; }
warn() { echo -e "${C_YEL}[sim] WARN${C_RST} $*"; }
err()  { echo -e "${C_RED}[sim] ERROR${C_RST} $*" >&2; }

# --- environment -------------------------------------------------------------
source_ros_base() {
  set +u
  # shellcheck disable=SC1090
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
}

source_ros() {
  source_ros_base || return 1
  set +u
  if [ -f "${WS_ROOT}/install/setup.bash" ]; then
    # shellcheck disable=SC1090
    source "${WS_ROOT}/install/setup.bash"
  else
    err "workspace not built: ${WS_ROOT}/install/setup.bash missing. Run 'sim.sh build'."
    return 1
  fi
  set -u
  # Runtime root: drives run_layout + table.ply resolution in the launch files.
  export ADAPTIVE_SCANNING_LOG_DIR="${RUNTIME_ROOT}"
  export RUN_LOG_DIR="${RUNTIME_ROOT}"
  export RUN_LOG_SKIP_PROMPT=1
  # Gazebo can find as_sim worlds/models via the package share parent (the
  # spawn launch sets GZ_SIM_RESOURCE_PATH itself, this is a belt-and-braces).
  export QT_QPA_PLATFORM="${QT_QPA_PLATFORM:-xcb}"
  mkdir -p "${SIM_LOG_DIR}" "${SIM_RUN_DIR}" "${RUNTIME_ROOT}/Calibration"
}

# Generate a sim params file from the installed scan_params.yaml:
#   * rewrite hard-coded /scan_logs paths to the runtime root
#   * force use_sim_time true on the global wildcard
gen_params() {
  local src
  src="$(ros2 pkg prefix as_edge_common 2>/dev/null)/share/as_edge_common/config/scan_params.yaml"
  if [ ! -f "${src}" ]; then
    err "installed scan_params.yaml not found at ${src}"
    return 1
  fi
  local probe_step="${ARMX_SIM_PROBE_SWEEP_STEP_M:-${SIM_PROBE_SWEEP_STEP_M:-0.19}}"
  mkdir -p "$(dirname "${SIM_PARAMS}")" "$(dirname "${SIM_DOCKER_PARAMS_HOST}")"
  # Transforms applied for simulation (real-hardware defaults in the repo are
  # left untouched):
  #   * /scan_logs -> in-repo runtime root (no privileged mount needed)
  #   * use_sim_time true so every node honours Gazebo's /clock
  #   * TSDF max_tf_wait_sec lowered: under sim time the depth image stamp runs a
  #     few ms ahead of the latest base_link<-camera TF while the arm moves (and
  #     TF lags further under planning CPU load), so the exact-time lookup the
  #     fusion node requires raises "extrapolation into the future". The node
  #     deliberately refuses a stale TF (it would corrupt the fused geometry), so
  #     these motion-time frames are dropped either way; lowering the wait from
  #     1.5s avoids blocking the callback 1.5s per dropped frame, keeping it
  #     responsive to the good frames captured while the arm is stationary. The
  #     TSDF still accumulates a dense cloud from those stationary captures.
  # Probe settle: under Gazebo the probe Cartesian sweep would otherwise finish
  # before enough stationary /base_pc frames are captured (depth stamp runs ahead
  # of TF during motion), starving the occupancy grid that gates every downstream
  # stage. A short dwell before occupancy freezes materially improves it. Real
  # hardware keeps 0.0 (packaged default); sim overrides to this value.
  local probe_settle="${ARMX_SIM_PROBE_POST_SETTLE_S:-${SIM_PROBE_POST_SETTLE_S:-0.6}}"
  # A0912 simulation tuning is intentionally applied only to generated sim
  # parameters; the real stack retains its calibrated fleet home/workspace.
  sed -e "s|/scan_logs|${RUNTIME_ROOT}|g" \
      -e "s|use_sim_time: false|use_sim_time: true|g" \
      -e "s|max_tf_wait_sec: 1.50|max_tf_wait_sec: 0.30|g" \
      -e "s|sweep_step_m: 0.2|sweep_step_m: ${probe_step}|g" \
      -e "s|post_execution_settle_s: 0.0|post_execution_settle_s: ${probe_settle}|g" \
      -e 's|scan_target_type: "pose"|scan_target_type: "joints"|g' \
      -e "s|execute_joint_full_target: false|execute_joint_full_target: true|g" \
      -e "s|scanner_offset_m: 0.25|scanner_offset_m: 0.32|g" \
      -e "s|boundary_min: \\[-0.30, 0.40, -0.12\\]|boundary_min: [-0.45, 0.35, -0.12]|g" \
      -e "s|boundary_max: \\[0.30, 0.75, 0.33\\]|boundary_max: [0.45, 1.00, 0.55]|g" \
      "${src}" > "${SIM_PARAMS}"
  sed -e "s|use_sim_time: false|use_sim_time: true|g" \
      -e "s|max_tf_wait_sec: 1.50|max_tf_wait_sec: 0.30|g" \
      -e "s|sweep_step_m: 0.2|sweep_step_m: ${probe_step}|g" \
      -e "s|post_execution_settle_s: 0.0|post_execution_settle_s: ${probe_settle}|g" \
      -e 's|scan_target_type: "pose"|scan_target_type: "joints"|g' \
      -e "s|execute_joint_full_target: false|execute_joint_full_target: true|g" \
      -e "s|scanner_offset_m: 0.25|scanner_offset_m: 0.32|g" \
      -e "s|boundary_min: \\[-0.30, 0.40, -0.12\\]|boundary_min: [-0.45, 0.35, -0.12]|g" \
      -e "s|boundary_max: \\[0.30, 0.75, 0.33\\]|boundary_max: [0.45, 1.00, 0.55]|g" \
      "${src}" > "${SIM_DOCKER_PARAMS_HOST}"
  ok "generated sim params -> ${SIM_PARAMS}"
  ok "generated Docker sim params -> ${SIM_DOCKER_PARAMS_HOST} (${SIM_DOCKER_PARAMS_CONTAINER})"
}

# Generate the synthetic table calibration cloud if missing (or FORCE=1).
gen_table() {
  local metadata_path
  metadata_path="$(dirname "${TABLE_PLY}")/metadata.json"
  if [ -f "${TABLE_PLY}" ] && [ -f "${metadata_path}" ] && [ "${FORCE:-0}" != "1" ]; then
    if table_calibration_matches_sim "${TABLE_PLY}" "${metadata_path}" "-0.12"; then
      log "table.ply present -> ${TABLE_PLY}"
      return 0
    fi
    warn "existing table calibration does not match the synthetic sim table; regenerating ${TABLE_PLY}"
  fi
  # ASCII PLY: the sim host lacks Open3D, and table_plane_from_ply.py only parses
  # binary PLY when Open3D is present (else it raises and the collision-plane node
  # dies, so move_group's static-scene wait times out). PCL's base_pc_generator
  # reads either format, so ASCII is safe for both consumers.
  python3 "${SIM_DIR}/gen_table_ply.py" --out "${TABLE_PLY}" --z -0.12 --ascii || return 1
}

table_calibration_matches_sim() {
  local ply_path="$1" metadata_path="$2" expected_z="$3"
  python3 - "$ply_path" "$metadata_path" "$expected_z" <<'PY'
import hashlib
import json
import math
import statistics
import sys
from pathlib import Path

ply_path = Path(sys.argv[1])
metadata_path = Path(sys.argv[2])
expected_z = float(sys.argv[3])

try:
    metadata = json.loads(metadata_path.read_text(encoding="utf-8"))
except Exception:
    sys.exit(1)

if metadata.get("source") != "simulation":
    sys.exit(1)

expected_sha = str(metadata.get("table_ply_sha256") or "")
actual_sha = hashlib.sha256(ply_path.read_bytes()).hexdigest()
if expected_sha != actual_sha:
    sys.exit(1)

try:
    with ply_path.open("r", encoding="ascii", errors="strict") as handle:
        vertex_count = None
        for line in handle:
            stripped = line.strip()
            if stripped.startswith("format ") and stripped != "format ascii 1.0":
                sys.exit(1)
            if stripped.startswith("element vertex "):
                vertex_count = int(stripped.split()[2])
            if stripped == "end_header":
                break
        if vertex_count is None:
            sys.exit(1)
        zs = []
        for _ in range(vertex_count):
            parts = handle.readline().split()
            if len(parts) < 3:
                sys.exit(1)
            zs.append(float(parts[2]))
except Exception:
    sys.exit(1)

if not zs:
    sys.exit(1)

median_z = statistics.median(zs)
max_abs_z_error = max(abs(z - expected_z) for z in zs)
if not math.isfinite(median_z) or abs(median_z - expected_z) > 1.0e-4:
    sys.exit(1)
if max_abs_z_error > 1.0e-4:
    sys.exit(1)

sys.exit(0)
PY
}

sim_object_expected() {
  [ -n "${SIM_OBJECT}" ] || [ -n "${SIM_PART}" ]
}

sim_object_resolved_name() {
  local resolved="${SIM_OBJECT:-${SIM_PART}}"
  if [ -n "${resolved}" ] && [[ "${resolved,,}" != *.stl ]]; then
    resolved="${resolved}.stl"
  fi
  echo "${resolved}"
}

sim_object_path() {
  local resolved
  resolved="$(sim_object_resolved_name)"
  [ -n "${resolved}" ] || return 1
  if [[ "${resolved}" = /* ]]; then
    echo "${resolved}"
    return 0
  fi
  local dataset_dir
  dataset_dir="$(ros2 pkg prefix as_sim 2>/dev/null)/share/as_sim/worlds/Dataset"
  if [ ! -d "${dataset_dir}" ]; then
    err "installed as_sim Dataset directory not found: ${dataset_dir}"
    return 1
  fi
  echo "${dataset_dir}/${resolved}"
}

validate_sim_object() {
  sim_object_expected || return 0
  local resolved path
  resolved="$(sim_object_resolved_name)"
  path="$(sim_object_path)" || return 1
  if [ ! -f "${path}" ]; then
    err "simulation dataset object missing: ${resolved} -> ${path}"
    err "set SIM_PART to an existing Dataset/<n>.stl or SIM_OBJECT to an STL filename/absolute path"
    return 1
  fi
  log "dataset object selected: ${resolved} -> ${path}"
}

# --- readiness helpers -------------------------------------------------------
# wait_for "<description>" <timeout_s> <command...>  -> polls until command succeeds
wait_for() {
  local desc="$1" timeout="$2"; shift 2
  local start now
  start="$(date +%s)"
  log "waiting for ${desc} (timeout ${timeout}s)..."
  while true; do
    if "$@" >/dev/null 2>&1; then ok "${desc}"; return 0; fi
    now="$(date +%s)"
    if [ $((now - start)) -ge "${timeout}" ]; then
      err "timed out waiting for ${desc}"
      return 1
    fi
    sleep 2
  done
}

_has_topic()  { ros2 topic list 2>/dev/null | grep -qxF "$1"; }
_has_action() { ros2 action list 2>/dev/null | grep -qxF "$1"; }
_has_node()   { ros2 node list 2>/dev/null | grep -qxF "$1"; }
_topic_has_pub() { [ "$(ros2 topic info "$1" 2>/dev/null | awk -F': ' '/Publisher count/{print $2}')" -gt 0 ] 2>/dev/null; }
_topic_pub_count() {
  local topic="$1"
  local count
  count="$(ros2 topic info "${topic}" 2>/dev/null | awk -F': ' '/Publisher count/{print $2; exit}')"
  echo "${count:-0}"
}
_no_clock_publishers() {
  [ "$(_topic_pub_count /clock)" -eq 0 ] 2>/dev/null
}
_no_stale_sim_nodes() {
  local nodes
  ros2 daemon stop >/dev/null 2>&1 || true
  nodes="$(ros2 node list 2>/dev/null)" || return 0
  ! grep -Eq '(^|/)(as_moveit_node|base_pc_generator|controller_manager|dataset_collision_scene|dsr_moveit_controller|edge_job_agent|execute_server_targets|gz_ros_control|initial_reconstruction|joint_state_broadcaster|move_group|move_to_scan|parallel_planning_and_execution_node|pointcloud_frame_relay|robot_state_publisher|ros_gz_bridge|parameter_bridge|table_plane_from_ply|waypoint_status_logger)$' <<<"${nodes}"
}
_clock_is_monotonic() {
  python3 "${SIM_DIR}/check_clock_monotonic.py" --topic /clock --duration-sec "${1:-8}" --min-samples "${2:-20}"
}
_controllers_active() {
  local output name
  output="$(ros2 control list_controllers -c "${SIM_ROBOT_NS}/controller_manager" 2>/dev/null)" || return 1
  for name in "$@"; do
    grep -Eq "^${name}[[:space:]].*[[:space:]]active([[:space:]]|$)" <<<"${output}" || return 1
  done
}
_action_servers_match() {
  ros2 run as_edge_common check_action_servers.py --robot-ns "${SIM_ROBOT_NS}" "$@"
}
_no_runtime_action_servers() {
  _action_servers_match \
    --timeout-sec 2.0 \
    --expect execute_joint=0 \
    --expect execute_waypoints=0 \
    --expect lookat_waypoints=0 \
    --expect /execute_server_targets=0
}
_robot_action_servers_ready() {
  _action_servers_match \
    --timeout-sec 10.0 \
    --expect execute_joint=1 \
    --expect execute_waypoints=1 \
    --expect lookat_waypoints=1 \
    --expect /execute_server_targets=0
}
_scan_action_servers_ready() {
  _action_servers_match \
    --timeout-sec 30.0 \
    --expect execute_joint=1 \
    --expect execute_waypoints=1 \
    --expect lookat_waypoints=1 \
    --expect /execute_server_targets=1
}
_moveit_action_server_ready() {
  _action_servers_match --timeout-sec 10.0 --expect move_action=1
}
_gz_model_exists() {
  gz model --list 2>/dev/null | sed -E 's/^[[:space:]]*-[[:space:]]*//' | grep -qxF "$1"
}

stop_process_pattern() {
  local pattern="$1"
  local label="${2:-$1}"
  if ! pgrep -f "${pattern}" >/dev/null 2>&1; then
    return 0
  fi
  warn "stopping escaped ${label} processes"
  pkill -TERM -f "${pattern}" 2>/dev/null || true
  for _ in $(seq 1 10); do
    pgrep -f "${pattern}" >/dev/null 2>&1 || return 0
    sleep 1
  done
  warn "force-killing escaped ${label} processes"
  pkill -KILL -f "${pattern}" 2>/dev/null || true
}

stop_sim_stragglers() {
  local patterns=(
    "scan_and_perceive.launch.py"
    "edge_scan.launch.py"
    "as_edge_robot_doosan_a0912 robot.launch.py"
    "as_sim gazebo.launch.py"
    "gz sim"
    "ruby.*gz sim"
    "parameter_bridge"
    "ros_gz_bridge"
    "pointcloud_frame_relay.py"
    "base_pc_generator"
    "table_plane_from_ply.py"
    "dataset_collision_scene.py"
    "robot_state_publisher"
    "move_group"
    "controller_manager"
    "as_moveit_node"
    "parallel_planning_and_execution_node"
    "move_to_scan_pose_node.py"
    "initial_reconstruction_action_server.py"
    "edge_job_agent.py"
    "ros2_tsdf_fusion.py"
    "waypoint_status_logger.py"
    "lookat_targets_executor.py"
  )
  local pattern
  for pattern in "${patterns[@]}"; do
    stop_process_pattern "${pattern}"
  done
}

# --- process management ------------------------------------------------------
# start_stage <name> <logfile> <command...>
start_stage() {
  local name="$1" logfile="$2"; shift 2
  local pidfile="${SIM_RUN_DIR}/${name}.pid"
  local previous_pid previous_start
  if [ -f "${pidfile}" ]; then
    read -r previous_pid previous_start < "${pidfile}" || true
  fi
  if [ -n "${previous_pid:-}" ] && stage_pid_matches "${previous_pid}" "${previous_start:-}"; then
    warn "${name} already running (pid ${previous_pid}). Use 'sim.sh down' first."
    return 1
  fi
  rm -f "${pidfile}"
  log "starting ${name} -> ${logfile}"
  # New process group so we can signal the whole launch tree on teardown.
  #
  # armx-e_sim.sh serializes lifecycle commands with flock on FD 9.  Shell
  # descriptors are inherited across exec, so a detached stage must explicitly
  # close it.  Otherwise Gazebo/MoveIt can retain the launcher's lifecycle lock
  # after the launcher exits, which prevents the Runner's recovery stop.
  setsid bash -c "exec 9>&-; $* >'${logfile}' 2>&1" &
  echo "$! $(stage_start_time "$!")" > "${pidfile}"
  ok "${name} started (pid $!)"
}

stage_start_time() {
  local pid="$1"
  awk '{print $22}' "/proc/${pid}/stat" 2>/dev/null || true
}

stage_pid_matches() {
  local pid="$1" expected_start="${2:-}" actual_start
  kill -0 "${pid}" 2>/dev/null || return 1
  # Legacy one-column pid files remain valid for a single transition release.
  [ -z "${expected_start}" ] && return 0
  actual_start="$(stage_start_time "${pid}")"
  [ -n "${actual_start}" ] && [ "${actual_start}" = "${expected_start}" ]
}

stop_stage() {
  local name="$1"
  local pidfile="${SIM_RUN_DIR}/${name}.pid"
  [ -f "${pidfile}" ] || return 0
  local pid started_at
  read -r pid started_at < "${pidfile}" || true
  if [ -n "${pid:-}" ] && stage_pid_matches "${pid}" "${started_at:-}"; then
    log "stopping ${name} (pgid ${pid})"
    # A bridge pause uses SIGSTOP on its scan process group.  Continue it
    # before SIGINT so ROS can run its normal shutdown handlers.
    kill -CONT -- "-${pid}" 2>/dev/null || true
    kill -INT -- "-${pid}" 2>/dev/null || kill -INT "${pid}" 2>/dev/null || true
    for _ in $(seq 1 15); do kill -0 "${pid}" 2>/dev/null || break; sleep 1; done
    kill -0 "${pid}" 2>/dev/null && kill -TERM -- "-${pid}" 2>/dev/null
    sleep 2
    kill -0 "${pid}" 2>/dev/null && kill -KILL -- "-${pid}" 2>/dev/null
  fi
  rm -f "${pidfile}"
}

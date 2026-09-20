#!/usr/bin/env bash
#
# AdaptiveScanning Gazebo simulation driver.
#
# Brings up the full self-contained scan pipeline in Gazebo (no real robot, no
# real camera, no cloud): Gazebo + robot + on-robot RGBD camera, MoveIt, and the
# scan_and_perceive orchestration (MoveToScanPose -> RunInitialProbe ->
# RunCameraScan -> MoveToScanPose), with probe/look-at planning done locally.
#
# Usage:
#   sim/sim.sh build           # colcon build the sim-relevant packages
#   sim/sim.sh prepare         # generate sim params + synthetic table.ply
#   sim/sim.sh gazebo          # stage 1: Gazebo + robot + camera + dataset object
#   sim/sim.sh moveit          # stage 2: MoveIt (move_group) for the gazebo robot
#   sim/sim.sh scan            # stage 3: run scan_and_perceive end-to-end
#   sim/sim.sh up              # prepare + all stages, gated by readiness checks
#   sim/sim.sh down            # stop everything
#   sim/sim.sh status          # ROS graph snapshot
#   sim/sim.sh logs [stage]    # tail a stage log (gazebo|moveit|scan)
#
# RViz is always disabled in the sim (low-resource hosts: RViz's ogre2 render
# sink starves the physics loop and makes gz /clock non-monotonic). Use the real
# robot bringup if you need RViz.
# Useful env overrides: SIM_PART=16  SIM_GZ_GUI=true
#                       SIM_USE_LOOKAT=false  FORCE=1 (regenerate table.ply)
set -uo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"
export SIM_MAJOR_EVENTS_LOG_PATH="${SIM_REPO_ROOT}/sim/major_events.log"

cmd_build() {
  # Prebuilt container image: the workspace is already built and only as_sim
  # has sources. Nothing to do but source the overlay.
  if [ "${SIM_SKIP_BUILD:-0}" = "1" ]; then
    source_ros || return 1
    ok "build skipped (SIM_SKIP_BUILD=1, prebuilt image)"
    return 0
  fi
  source_ros_base || return 1
  log "colcon build (symlink-install) in ${WS_ROOT}"
  ( cd "${WS_ROOT}" && colcon build --symlink-install \
      --packages-up-to as_edge_common as_edge_robot_doosan_a0912 \
                       as_edge_camera_intel_d405 as_sim ) || return 1
  source_ros || return 1
  ok "build complete"
}

cmd_prepare() {
  # In the container stack the sim-prepare service generates params and the
  # table calibration into the shared runtime volume before this starts.
  if [ "${SIM_SKIP_PREPARE:-0}" = "1" ]; then
    source_ros || return 1
    validate_sim_object || return 1
    ok "prepare skipped (SIM_SKIP_PREPARE=1, artifacts supplied externally)"
    return 0
  fi
  source_ros || return 1
  validate_sim_object || return 1
  gen_params || return 1
  gen_table  || return 1
  > "${SIM_MAJOR_EVENTS_LOG_PATH}" || true
  ok "prepared: params=${SIM_PARAMS} table=${TABLE_PLY}"
}

cmd_gazebo() {
  source_ros || return 1
  validate_sim_object || return 1
  local args=(
    name:="${SIM_NAME}" model:=a0912_scanner mode:=gazebo
    use_sim_time:=true gui:="${SIM_GZ_GUI}"
    obj_x:="${SIM_OBJ_X}" obj_y:="${SIM_OBJ_Y}" obj_z:="${SIM_OBJ_Z}"
    obj_R:="${SIM_OBJ_R}" obj_P:="${SIM_OBJ_P}" obj_Y:="${SIM_OBJ_YAW}"
    obj_scale:="${SIM_OBJ_SCALE}" obj_place_on_floor:="${SIM_OBJ_PLACE_ON_FLOOR}"
    obj_floor_clearance_m:="${SIM_OBJ_FLOOR_CLEARANCE_M}" floor_z:="${SIM_FLOOR_Z}"
  )
  [ -n "${SIM_OBJECT}" ] && args+=(object:="${SIM_OBJECT}")
  [ -n "${SIM_PART}" ] && args+=(part:="${SIM_PART}")
  # Headless Gazebo server when GUI disabled.
  case "${SIM_GZ_GUI,,}" in
    false|0|no|off) export GZ_SIM_HEADLESS=1 ;;
    *) unset GZ_SIM_HEADLESS ;;
  esac
  start_stage gazebo "${SIM_LOG_DIR}/gazebo.log" \
    "ros2 launch as_sim gazebo.launch.py ${args[*]}"
}

wait_gazebo_ready() {
  source_ros || return 1
  wait_for "/clock (sim time)"                 60 _has_topic "/clock" || return 1
  wait_for "monotonic /clock"                  30 _clock_is_monotonic 8 5 || return 1
  wait_for "${SIM_ROBOT_NS}/joint_states"      90 _has_topic "${SIM_ROBOT_NS}/joint_states" || return 1
  wait_for "depth camera info"                 90 _topic_has_pub "/camera/depth/camera_info" || return 1
  wait_for "depth image"                       60 _topic_has_pub "/camera/depth/image_rect_raw" || return 1
  if sim_object_expected; then
    wait_for "Gazebo dataset_object model"     45 _gz_model_exists "dataset_object" || return 1
  fi
  wait_for "Gazebo controllers active"         90 _controllers_active \
    joint_state_broadcaster dsr_moveit_controller || return 1
}

cmd_moveit() {
  source_ros || return 1
  wait_for "monotonic /clock before MoveIt" 30 _clock_is_monotonic 8 5 || return 1
  wait_for "no stale Robot or scan action servers" 10 _no_runtime_action_servers || return 1
  validate_sim_object || return 1
  local dataset_dir
  dataset_dir="$(ros2 pkg prefix as_sim 2>/dev/null)/share/as_sim/worlds/Dataset"
  local enable_dataset_collision="false"
  if [ -n "${SIM_OBJECT}" ] || [ -n "${SIM_PART}" ]; then
    enable_dataset_collision="true"
  fi
  local args=(
    mode:=gazebo name:="${SIM_NAME}" start_rviz:="${SIM_RVIZ:-false}"
    enable_dataset_collision:="${enable_dataset_collision}" dataset_dir:="${dataset_dir}"
    obj_x:="${SIM_OBJ_X}" obj_y:="${SIM_OBJ_Y}" obj_z:="${SIM_OBJ_Z}"
    obj_R:="${SIM_OBJ_R}" obj_P:="${SIM_OBJ_P}" obj_Y:="${SIM_OBJ_YAW}"
    obj_scale:="${SIM_OBJ_SCALE}" obj_place_on_floor:="${SIM_OBJ_PLACE_ON_FLOOR}"
    obj_floor_clearance_m:="${SIM_OBJ_FLOOR_CLEARANCE_M}" floor_z:="${SIM_FLOOR_Z}"
    object_pose_source_frame:="${SIM_OBJECT_POSE_SOURCE_FRAME}"
    base_floor_z_m:="${SIM_BASE_FLOOR_Z_M}"
  )
  [ -n "${SIM_OBJECT}" ] && args+=(object:="${SIM_OBJECT}")
  [ -n "${SIM_PART}" ] && args+=(part:="${SIM_PART}")
  start_stage moveit "${SIM_LOG_DIR}/moveit.log" \
    "ros2 launch as_edge_robot_doosan_a0912 robot.launch.py ${args[*]}"
}

wait_moveit_ready() {
  source_ros || return 1
  wait_for "move_group node"              120 _has_node "${SIM_ROBOT_NS}/move_group" || return 1
  wait_for "move_action server"           120 _has_action "${SIM_ROBOT_NS}/move_action" || return 1
  wait_for "single move_action server"      15 _moveit_action_server_ready || return 1
  wait_for "Robot-owned motion action servers" 30 _robot_action_servers_ready || return 1
  local required_objects="[table_plane, robot_base_mount, table_stand]"
  if sim_object_expected; then
    required_objects="[table_plane, robot_base_mount, table_stand, dataset_object]"
  fi
  wait_for "static planning-scene collision objects" 90 \
    ros2 run as_edge_common wait_for_planning_scene_objects.py \
      --ros-args -p robot_ns:="${SIM_ROBOT_NS}" \
      -p required_object_ids:="${required_objects}" -p timeout_sec:=3.0 || return 1
}

cmd_scan() {
  source_ros || return 1
  wait_for "monotonic /clock before scan" 30 _clock_is_monotonic 8 5 || return 1
  wait_for "Robot actions ready and no stale Common scan server" 10 _robot_action_servers_ready || return 1
  local args=(
    name:="${SIM_NAME}" robot_ns:="${SIM_ROBOT_NS}"
    use_sim_time:=true use_lookat:="${SIM_USE_LOOKAT}"
    params_file:="${SIM_PARAMS}"
  )
  start_stage scan "${SIM_LOG_DIR}/scan.log" \
    "ros2 launch as_edge_common scan_and_perceive.launch.py ${args[*]}"
}

wait_scan_ready() {
  source_ros || return 1
  wait_for "scan action server uniqueness" 45 _scan_action_servers_ready || return 1
}

cmd_hw() {
  # Sim "hardware" only: Gazebo robot + camera + MoveIt, no scan. Used when an
  # external orchestrator (armx-e bridge) launches the scan itself.
  cmd_build || return 1
  cmd_prepare || return 1
  cmd_gazebo  || return 1
  wait_gazebo_ready || { err "gazebo not ready; see ${SIM_LOG_DIR}/gazebo.log"; return 1; }
  cmd_moveit  || return 1
  wait_moveit_ready || { err "moveit not ready; see ${SIM_LOG_DIR}/moveit.log"; return 1; }
  ok "sim hardware up (Gazebo + MoveIt). No scan started."
}

cmd_up() {
  cmd_build || return 1
  cmd_prepare || return 1
  cmd_gazebo  || return 1
  wait_gazebo_ready || { err "gazebo not ready; see ${SIM_LOG_DIR}/gazebo.log"; return 1; }
  cmd_moveit  || return 1
  wait_moveit_ready || { err "moveit not ready; see ${SIM_LOG_DIR}/moveit.log"; return 1; }
  # Let controllers settle, planning scene populate.
  sleep 5
  cmd_scan || return 1
  wait_scan_ready || { err "scan action graph not ready; see ${SIM_LOG_DIR}/scan.log"; return 1; }
  ok "all stages up. Follow the scan with: sim/sim.sh logs scan"
}

cmd_down() {
  stop_stage scan
  stop_stage moveit
  stop_stage gazebo
  stop_sim_stragglers
  ok "all stages stopped"
}

cmd_status() {
  source_ros || return 1
  echo "--- nodes ---";   ros2 node list 2>/dev/null | sort
  echo "--- actions ---"; ros2 action list 2>/dev/null | sort
  echo "--- key topics ---"
  ros2 topic list 2>/dev/null | grep -E "clock|camera|base_pc|joint_states|table_plane|tsdf|scan" | sort
}

cmd_logs() {
  local stage="${1:-scan}"
  tail -n 200 -f "${SIM_LOG_DIR}/${stage}.log"
}

main() {
  local cmd="${1:-up}"; shift || true
  case "${cmd}" in
    build)   cmd_build ;;
    prepare) cmd_prepare ;;
    gazebo)  cmd_gazebo ;;
    moveit)  cmd_moveit ;;
    scan)    cmd_scan ;;
    hw)      cmd_hw ;;
    up)      cmd_up ;;
    down)    cmd_down ;;
    status)  cmd_status ;;
    logs)    cmd_logs "$@" ;;
    *) err "unknown command: ${cmd}"; sed -n '3,40p' "${BASH_SOURCE[0]}"; exit 2 ;;
  esac
}
main "$@"

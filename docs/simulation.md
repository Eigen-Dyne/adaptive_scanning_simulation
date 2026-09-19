# Simulation (Gazebo) — Self-Contained Scan

This documents how AdaptiveScanning runs end-to-end in Gazebo with **no real
robot, camera, or cloud**, the scripts that drive it, and the bugs that were
fixed to make it work. For day-to-day usage see [`../sim/README.md`](../sim/README.md).

## What runs

| Layer | Launch | Provides |
| --- | --- | --- |
| Gazebo | `as_sim/launch/gazebo.launch.py` | gz-sim world, robot spawn, gz↔ROS camera bridge, point-cloud frame relay, dataset object, gz_ros2_control + controllers |
| MoveIt | `as_edge_robot_doosan_a0912/launch/robot.launch.py mode:=gazebo` | `move_group`, table collision plane, dataset-object collision mirror |
| Scan | `as_edge_common/launch/scan_and_perceive.launch.py` | scan action servers + `edge_orchestrator` |

The robot base is welded to the world with a −90° yaw
([`a0912_scanner.urdf.xacro`](../as_edge_robot_doosan_a0912/doosan/dsr_description/xacro/a0912_scanner.urdf.xacro)),
so a Gazebo-frame object at `(0.65, 0)` maps to base-frame `(0, 0.65)` — inside the
simulation workspace `y∈[0.35,1.00]` and under the A0912 scan pose. The on-arm D405 is an
`rgbd_camera` sensor in
[`a0912_end_effector.xacro`](../as_edge_robot_doosan_a0912/doosan/dsr_description/xacro/a0912_end_effector.xacro).

Gazebo owns the visual/physical `dataset_object`. MoveIt receives a matching
collision mesh from `dataset_collision_scene.py`; in simulation the object pose
is declared in Gazebo/world coordinates and converted into `base_link` before it
is applied to the planning scene. Direct `robot.launch.py` use keeps the legacy
base-frame pose behavior unless `object_pose_source_frame:=gazebo_world` is set.

## Driver scripts

`sim/sim.sh` (subcommands `build|prepare|gazebo|moveit|scan|up|down|status|logs`)
plus `sim/lib.sh` and `sim/gen_table_ply.py`. `up` runs the three stages with
readiness gating (`/clock` presence and monotonicity, `joint_states`, camera,
controllers → `move_group` → scan action graph).

## Verified run

```
move_to_scan ✓ → run_initial_probe 8/8, 16 occupied voxels ✓
              → 27–33 look-at targets generated ✓
              → run_camera_scan: ~26–29/33 look-at waypoints executed ✓
              → TSDF saved ~23k–26k points, classified mesh, 17 red-normal targets ✓
              → final move_to_scan ✓ → orchestrator exit 0
```

Outputs land under `scan_logs/scan_<ts>/scan and perceive launch/` (look-at CSVs,
`tsdf/*.ply`, `red_normal_targets.csv`).

## Bugs found and fixed

### 1. Empty Gazebo world (critical)
`as_sim/worlds/scanner_world.world` was an empty `<world>` element. With no
system plugins gz-sim runs no physics (the robot can't be commanded), exposes no
`/world/.../create` service (the robot and dataset object can't be spawned), and
never renders the camera. **Fix:** populated the world with the Physics,
SceneBroadcaster, UserCommands and Sensors systems, plus a sun and a ground plane
(the ground at gz z=0 is the table top, matching base-frame `table_plane_z_m=-0.12`).

### 2. Camera render system declared in the spawned model
The gz `Sensors` system was declared inside the robot model
(`a0912_end_effector.xacro`), which is spawned *after* the world starts — the
render system may not register, and it double-declares once the world owns it.
**Fix:** the world now owns the `Sensors` system (loaded at world start); the
model keeps only the sensor definition.

### 3. `table.ply` prerequisite + hard-coded `/scan_logs` paths
`scan_and_perceive.launch.py` refuses to start without a table-calibration cloud,
and several params point at the privileged `/scan_logs` mount. **Fix:** the
scripts generate a params overlay with `/scan_logs` rewritten to the in-repo
`scan_logs/` runtime root, and synthesize a base-frame `table.ply` flat sheet
(`gen_table_ply.py`) — no separate calibration run or root mount needed.

### 4. TSDF frame-drop noise under sim time (benign)
While the arm moves, the depth-image timestamp runs slightly ahead of the latest
`base_link←camera` TF (and TF lags further under planning CPU load), so the
fusion node's required exact-time lookup reports "extrapolation into the future".
The node **correctly** refuses a stale transform — using one would corrupt the
fused geometry — so those motion-time frames are dropped by design. The TSDF
still accumulates a dense cloud (~23k points) from the stationary capture pauses.
**Fix:** the overlay only lowers `max_tf_wait_sec` (1.5 → 0.3) so the node fails
fast instead of blocking 1.5 s per dropped frame. This is a sim-time artifact,
not a pipeline defect; real-hardware defaults are untouched.

## ARM-X cloud path

`armx-e/scripts/armx-e_sim.sh` starts the same Gazebo/MoveIt hardware layer and
then launches the ARM-X services against either the real cloud or the optional
in-process mock AI. Object selection follows the same `SIM_PART`/`SIM_OBJECT`
contract via `ARMX_SIM_PART`/`ARMX_SIM_OBJECT`, and the wrapper forwards object
pose, scale, and base-floor settings into `sim/sim.sh`.

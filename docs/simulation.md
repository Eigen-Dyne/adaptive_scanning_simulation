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

## Auto hand-eye calibration from the UI (simulation only)

The Fleet Status **Hand-Eye Calibration → Calibrate** button uses the existing
Common calibration request and `/armx/run_handeye_calibration` Robot action.
With a simulated Robot, that action starts the headless Sentiac engine ported
from `A0912_Unified` at `0422c419`. The real-mode action retains its existing
runtime; this integration does not enable the auto engine on hardware.

Rebuild Common, Robot and Simulation, then restart the stack in simulation:

```bash
./scripts/adaptive-scanning build --with-sim common robot simulation
./scripts/adaptive-scanning up sim
```

For a native ROS workspace, rebuild packages up to `as_edge_common`,
`as_edge_robot_doosan_a0912` and `as_sim`, source the new install overlay, and
restart Gazebo and Robot/MoveIt in `mode:=gazebo` before using the UI.
The Robot component includes the engine and pinned `industrial_calibration`,
`boost_plugin_loader` and `ros_industrial_cmake_boilerplate` sources. No separate
source dependency checkout is needed. The Robot Dockerfile builds them too.

Calibration needs an idle simulation with the Robot action server, MoveIt,
controllers, `/clock`, the robot's namespaced `/dsr01/joint_states`, RGB images
and CameraInfo running.
The Simulation package bridges Gazebo `/camera/depth/image` to ROS
`/camera/color/image_raw` and owns the temporary ChArUco board. It removes the
current dataset object while calibration uses the object area, then restores
the same model and resolved pose on completion or failure.

The engine uses the Robot's evaluated model and `/dsr01` MoveIt namespace
(or the configured robot name), moves through `scanpose` and `capturepose`,
then plans and captures additional views. The simulation profile uses
`camera_depth_optical_frame`, a 5×7 board with `DICT_6X6_250`, 35 mm squares
and 25 mm markers, and requires at least 15 accepted observations.
Each run has an isolated ROS namespace and session directory. A stale completed
session, observation-only validation, missing samples or failed fit cannot
complete the UI request. The run has a 30-minute wall-time limit.

Outputs under the shared runtime root are:

- `Handeye Calibration/sentiac_sessions/<run>/session_<timestamp>/`: raw images,
  sample poses, final solution, report and residual diagnostics.
- `Handeye Calibration/handeye_result.yaml`: measured flange-to-`camera_link`
  pose, converted from the optical-frame solution, with quaternion and Euler angles.
- `Handeye Calibration/handeye_pose_samples.yaml` and `handeye_capture_status.csv`:
  the accepted capture records consumed by the Common lifecycle.
- `handeye_sim_<fixture>/`: simulation result archive, separate from the
  fixture calibration files loaded for real hardware.
- `calibration-records/handeye.json`: completed hand-eye status for the UI.

Simulated hand-eye always launches the engine, including when
`ARMX_ALLOW_SYNTHETIC_CALIBRATION=true`. Table simulation can still use its
existing synthetic path; it no longer creates or overwrites a hand-eye pose.
Hand-eye can finish before table calibration, but scan readiness still requires
the table artifacts. The measured camera result is recorded without changing
Gazebo's fixed mounting transform or applying it to a real-camera model.

Verification for this port: the engine and its dependencies build; native
solver tests, isolated ROS observation/capture-retry tests, and adapter/bridge
tests pass. The broader runtime suite has a pre-existing real preflight test
that omits the required validity period. A full Gazebo run from the UI still
needs checking in the deployed stack.

An isolated native Gazebo run was attempted in ROS domain 179. Gazebo crashed
in Ogre2's EGL/Mesa renderer before camera readiness, so that run could not
verify a completed calibration. Its diagnostic logs are in
`/tmp/handeye-gazebo-runtime/`. The test processes were stopped. The simulation
Robot launch now avoids resolving the unused real `dsr_controller2` package.

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

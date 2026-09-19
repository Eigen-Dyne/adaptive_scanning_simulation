# AdaptiveScanning — Gazebo Simulation

Run the full **self-contained** scan pipeline in Gazebo, with **no real robot,
no real camera, and no cloud/AI backend**. The robot, the on-arm Intel D405
depth camera, and the table object are all simulated; the scan
(`MoveToScanPose → RunInitialProbe → RunCameraScan → MoveToScanPose`) is driven
by `scan_and_perceive.launch.py`, with probe sweeping, occupancy, look-at target
generation, and TSDF fusion all done locally.

> This is the **self-contained** ROS path. For the ArmX-E application path using
> the same simulated hardware, follow the root [README](../README.md).

## Prerequisites

- ROS 2 Jazzy, the workspace built once (`colcon build`), Gazebo (gz-sim 8) and
  `ros_gz_*` installed (all already present on this machine).
- `doosan-robot2` checked out in the same workspace (`~/ros2_ws/src/doosan-robot2`).
- Gazebo runs headless by default. A display is only needed when explicitly
  opting into the Gazebo GUI.

## Quick start

```bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation

./sim/sim.sh build      # one-time (or after C++/launch changes)
./sim/sim.sh up         # bring up Gazebo + MoveIt + run the scan end-to-end
./sim/sim.sh logs scan  # follow the scan
./sim/sim.sh down       # stop everything
```

`up` runs the stages in order with readiness gating between them:

1. **gazebo** — `as_sim gazebo.launch.py`: Gazebo + robot + RGBD camera bridge +
   point-cloud relay + dataset object + controllers.
2. **moveit** — `as_edge_robot_doosan_a0912 robot.launch.py mode:=gazebo`:
   `move_group`, the table collision plane, and the transformed dataset-object
   collision mirror.
3. **scan** — `as_edge_common scan_and_perceive.launch.py`: scan action servers
   and the orchestrator that drives the end-to-end scan, then exits.

## Commands

| Command | Purpose |
| --- | --- |
| `sim.sh build` | `colcon build --symlink-install` for the sim-relevant packages |
| `sim.sh prepare` | Generate the sim params overlay + synthetic `table.ply` |
| `sim.sh gazebo` | Stage 1 only |
| `sim.sh moveit` | Stage 2 only |
| `sim.sh scan` | Stage 3 only |
| `sim.sh up` | prepare + all stages, gated by readiness checks |
| `sim.sh down` | Stop all stages |
| `sim.sh status` | ROS node/action/topic snapshot |
| `sim.sh logs [gazebo\|moveit\|scan]` | Tail a stage log |

## Configuration (env overrides)

| Variable | Default | Meaning |
| --- | --- | --- |
| `SIM_PART` | `1` | `as_sim/worlds/Dataset/<part>.stl` to scan (`""` = none) |
| `SIM_OBJECT` | unset | STL filename from Dataset or absolute STL path override |
| `SIM_OBJ_X/Y/Z` | `0.65/0.0/0.0` | Dataset object pose in Gazebo/world coordinates |
| `SIM_OBJ_R/P/YAW` | `0.0/0.0/0.0` | Dataset object orientation in Gazebo/world coordinates |
| `SIM_OBJ_SCALE` | `0.001` | Uniform mesh scale |
| `SIM_BASE_FLOOR_Z_M` | `-0.12` | Floor/table z used when mirroring the Gazebo object into `base_link` |
| `SIM_GZ_GUI` | `false` | Start the Gazebo GUI when set to `true` |
| `SIM_USE_LOOKAT` | `true` | Run the camera look-at scan (`false` = probe + target gen only) |
| `FORCE` | `0` | Regenerate `table.ply` even if it exists |
| `ADAPTIVE_SCANNING_SIM_LOG_DIR` | `<repo>/scan_logs` | Runtime/output root |

The stable validation path is headless: Gazebo GUI and RViz are opt-in
diagnostics because their render sinks can starve the Gazebo physics loop and
make `/clock` non-monotonic.

Example: scan object 16 with the stable headless profile:

```bash
SIM_PART=16 ./sim/sim.sh up
```

## Outputs

Per-run artifacts land under `<repo>/scan_logs/scan_<timestamp>/scan and perceive launch/`:

- `lookat_targets.csv`, `scanner_lookat_targets.csv` — generated look-at targets
- `tsdf/tsdf_cloud_*.ply`, `tsdf/tsdf_mesh_*.ply`, `tsdf/classified_mesh_*.ply`
- `tsdf/red_normal_targets.csv` — surface targets from the classified mesh

Stage console logs are in `sim/logs/{gazebo,moveit,scan}.log`.

## Files

| File | Role |
| --- | --- |
| `sim.sh` | CLI entry point |
| `lib.sh` | Shared env, params/table provisioning, readiness + process helpers |
| `gen_table_ply.py` | Synthesizes the base-frame table calibration cloud |
| `run/sim_scan_params.yaml` | Generated params overlay (do not edit; regenerated) |

## What these scripts fix to make sim work

The simulation pipeline did not run before; the following bugs were found and
fixed (see `../docs/simulation.md` for detail):

1. **Empty Gazebo world** — `as_sim/worlds/scanner_world.world` was an empty
   `<world>` element: no physics, no render/sensors system, no light/ground, and
   no spawn services. Populated it (Physics, SceneBroadcaster, UserCommands,
   Sensors, sun, ground plane).
2. **Camera render system declared in the spawned model** — moved the gz
   `Sensors` system into the world (loaded before the robot spawns) so the depth
   camera actually renders.
3. **Hard-coded `/scan_logs` paths + missing `table.ply`** — the scan launch
   refuses to start without a table calibration cloud and several params point at
   the privileged `/scan_logs` mount. The scripts generate a params overlay
   (paths rewritten to the in-repo runtime root) and a synthetic `table.ply`.
4. **TSDF frame-drop noise under sim time** — while the arm moves, the depth
   image stamp runs slightly ahead of the latest TF, so the fusion node's
   required exact-time lookup reports "extrapolation into the future". The node
   *correctly* refuses a stale transform (using one would corrupt the geometry),
   so those motion-time frames are dropped by design; the TSDF still builds a
   dense cloud from the stationary capture pauses (~23k points/run). The overlay
   only lowers `max_tf_wait_sec` so the node fails fast instead of blocking 1.5 s
   per dropped frame. This is a sim-time artifact, not a pipeline defect.

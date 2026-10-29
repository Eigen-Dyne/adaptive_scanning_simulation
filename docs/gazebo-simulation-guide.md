# Gazebo Simulation Guide

This guide is for a new operator using this device for the first time. It explains how to open a terminal, start the Gazebo simulation, stop it safely, change the scanned object, watch logs, and use the main Gazebo window controls.

This guide covers only the self-contained Gazebo simulation flow:

```text
Gazebo + simulated Doosan robot + simulated D405 camera + MoveIt + scan pipeline
```

It does not cover the ARMx-E cloud UI workflow.

## 1. Open A Terminal

Press:

```text
Ctrl + Alt + T
```

You should see a terminal prompt similar to:

```bash
nandha@nandha-Latitude-E5470:~$
```

This is where you type commands.

Useful terminal commands:

| Command | Meaning |
| --- | --- |
| `pwd` | Shows the current folder. |
| `ls` | Lists files and folders. |
| `cd folder_name` | Moves into a folder. |
| `cd ..` | Moves one folder back. |
| `clear` | Clears the terminal screen. |
| `Ctrl + C` | Stops the command currently running in that terminal. |

## 2. Load ROS

Your ROS workspace is expected to be here:

```bash
cd ~/Desktop/ros2_ws
```

Load ROS 2 Jazzy:

```bash
source /opt/ros/jazzy/setup.bash
```

Load the built workspace:

```bash
source install/setup.bash
```

If `source install/setup.bash` says the file does not exist, build the workspace first:

```bash
cd ~/Desktop/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## 3. Go To The Simulation Project

Move into the AdaptiveScanning project:

```bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation
```

This folder contains the simulation script:

```bash
./sim/sim.sh
```

## 4. Start Gazebo Simulation

From the AdaptiveScanning folder, run:

```bash
./sim/sim.sh up
```

This starts:

| Component | Purpose |
| --- | --- |
| Gazebo world | The 3D simulation environment. |
| Simulated Doosan robot | The robot arm used for scanning. |
| Simulated D405 camera | The depth camera attached to the robot. |
| MoveIt | Robot motion planning. |
| Scan pipeline | The scanning workflow. |
| Dataset object | The object being scanned. |

Gazebo can take some time to open, especially on the first run.

## 5. Stop Gazebo Simulation

To stop everything safely:

```bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation
./sim/sim.sh down
```

If the simulation is running in the same terminal and the terminal is busy, press:

```text
Ctrl + C
```

Then run:

```bash
./sim/sim.sh down
```

## 6. Check Simulation Status

Use:

```bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation
./sim/sim.sh status
```

This shows whether Gazebo, MoveIt, scan nodes, topics, and actions are running.

## 7. Watch Logs

To see Gazebo logs:

```bash
./sim/sim.sh logs gazebo
```

To see MoveIt logs:

```bash
./sim/sim.sh logs moveit
```

To see scan logs:

```bash
./sim/sim.sh logs scan
```

To exit log view:

```text
Ctrl + C
```

## 8. Change The Object Before Starting Simulation

Dataset objects are stored here:

```text
~/adaptive_scanning_ws/src/adaptive_scanning_simulation/as_sim/worlds/Dataset/
```

They are named like:

```text
0.stl
1.stl
2.stl
...
20.stl
```

To run the simulation with object `16.stl`:

```bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation
SIM_PART=16 ./sim/sim.sh up
```

To run with object `5.stl`:

```bash
SIM_PART=5 ./sim/sim.sh up
```

To use the default object:

```bash
./sim/sim.sh up
```

Before changing the object, stop the current simulation:

```bash
./sim/sim.sh down
```

Then start again with the new object:

```bash
SIM_PART=<object_number> ./sim/sim.sh up
```

Example:

```bash
SIM_PART=12 ./sim/sim.sh up
```

## 9. Change The Object While Gazebo Is Running

If Gazebo is already open and running, the safest method is to stop the current simulation and restart with the new object.

Recommended method:

1. Open a new terminal with `Ctrl + Alt + T`.
2. Go to the project:

```bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation
```

3. Stop the current simulation:

```bash
./sim/sim.sh down
```

4. Start again with the new object number:

```bash
SIM_PART=16 ./sim/sim.sh up
```

Change `16` to the object number you want.

Examples:

```bash
SIM_PART=1 ./sim/sim.sh up
SIM_PART=5 ./sim/sim.sh up
SIM_PART=10 ./sim/sim.sh up
SIM_PART=20 ./sim/sim.sh up
```

Quick object-change command:

```bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation
./sim/sim.sh down
SIM_PART=16 ./sim/sim.sh up
```

### Can I Change The Object Without Restarting?

For normal scan testing, do not change the object manually while Gazebo is running.

Gazebo allows you to delete, move, insert, or rotate objects from the GUI, but the scan pipeline expects the object, robot, camera, MoveIt, and scan parameters to start together cleanly. If you manually change the object while scanning is active, the scan result may be wrong or the robot may continue using old assumptions.

For reliable scan results, use this flow:

```text
Stop simulation -> choose object -> start simulation again
```

### Manual GUI Method

Use this only for visual checking, not for official scan runs.

1. In Gazebo, open the left panel or entity tree.
2. Find `dataset_object`.
3. Select it.
4. Delete or remove it from the world.
5. Use the `Insert` panel to insert another mesh or model.
6. Move it to the correct scan area using the move/translate tool.

Important: this manual method does not automatically update the scan script or object setup. For real scan testing, use:

```bash
SIM_PART=<object_number> ./sim/sim.sh up
```

## 10. Run Gazebo With Or Without GUI

The stable validation path runs without the Gazebo GUI:

```bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation
./sim/sim.sh up
```

To open the Gazebo GUI for visual diagnostics, opt in explicitly:

```bash
SIM_GZ_GUI=true ./sim/sim.sh up
```

## 11. Start With RViz

To also open RViz:

```bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation
SIM_RVIZ=true ./sim/sim.sh up
```

To scan object `16.stl` with RViz:

```bash
SIM_PART=16 SIM_RVIZ=true ./sim/sim.sh up
```

## 12. Gazebo Window Buttons

Common Gazebo controls:

| Control | Use |
| --- | --- |
| Play / Pause | Starts or pauses physics. If paused, the robot and sensors may not move or update. |
| Step | Moves simulation forward by one small step. Useful when paused for debugging. |
| Select | Selects objects, robot parts, lights, or models. |
| Translate / Move | Moves the selected object along X, Y, and Z axes. Use carefully because moving the object can affect scanning. |
| Rotate | Rotates the selected object around roll, pitch, and yaw axes. |
| Scale | Changes the size of the selected object. Usually avoid this for scan tests. |
| Camera View | Lets you inspect the scene visually. |
| Insert | Adds models into the world manually. Usually not needed because scripts spawn the robot and object automatically. |
| Entity Tree / Left Panel | Shows world objects such as the robot, ground plane, camera, and dataset object. |
| World / Model Properties | Shows position, rotation, collision, visual, and other properties for the selected object. |
| Reset | Resets simulation time or world state. Avoid during active scans unless you plan to restart. |

## 13. Mouse Controls In Gazebo

| Action | Mouse Control |
| --- | --- |
| Rotate camera view | Hold left mouse button and drag. |
| Pan camera view | Hold middle mouse button and drag. |
| Zoom | Use mouse wheel. |
| Select object | Left-click the object. |
| Move selected object | Choose the translate/move tool, then drag the colored arrows. |

## 14. Recommended Daily Workflow

Use this sequence for a normal run:

```bash
cd ~/Desktop/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash

cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation
./sim/sim.sh down
SIM_PART=1 ./sim/sim.sh up
```

To stop:

```bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation
./sim/sim.sh down
```

To change object:

```bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation
./sim/sim.sh down
SIM_PART=16 ./sim/sim.sh up
```

## 15. Output Files

Scan results are saved under:

```text
~/adaptive_scanning_ws/src/adaptive_scanning_simulation/scan_logs/
```

Stage logs are saved under:

```text
~/adaptive_scanning_ws/src/adaptive_scanning_simulation/sim/logs/
```

Important output files may include:

```text
lookat_targets.csv
scanner_lookat_targets.csv
tsdf_cloud_*.ply
tsdf_mesh_*.ply
classified_mesh_*.ply
red_normal_targets.csv
```

## 16. Common Problems

### ROS Package Not Found

Run:

```bash
cd ~/Desktop/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
```

### `install/setup.bash` Is Missing

Build the workspace:

```bash
cd ~/Desktop/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
```

### Gazebo Is Stuck Or Old Processes Are Running

Stop everything and check status:

```bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation
./sim/sim.sh down
./sim/sim.sh status
```

### Laptop Is Slow

Use the stable headless profile:

```bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation
./sim/sim.sh up
```

## 17. Full Copy-Paste Start Command

For normal simulation with object `1.stl`:

```bash
cd ~/Desktop/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation
./sim/sim.sh down
SIM_PART=1 ./sim/sim.sh up
```

## 18. Full Copy-Paste Stop Command

```bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation
./sim/sim.sh down
```

## 19. Full Copy-Paste Change Object Command

Replace `16` with the object number you want:

```bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation
./sim/sim.sh down
SIM_PART=16 ./sim/sim.sh up
```

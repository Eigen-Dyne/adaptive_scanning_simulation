# Adaptive Scanning Simulation

This repository owns the Gazebo simulation package and test runner for the
Adaptive Scanning stack. It is development/test tooling and is not included in
the production Common, Robot, or Sensor containers.

The simulation preserves the production ROS 2 contracts:

- Gazebo replaces the physical robot and depth camera.
- the Robot package runs MoveIt and the motion action servers in `gazebo` mode;
- the Common package runs the same scan orchestration used with real hardware;
- ArmX-E can run against the simulated ROS graph for end-to-end UI testing.

## Workspace setup

Use a clean ROS 2 Jazzy workspace. Clone every repository on the same branch:

```bash
mkdir -p ~/adaptive_scanning_ws/src
cd ~/adaptive_scanning_ws/src

git clone --branch dependency_fix \
  https://github.com/Eigen-Dyne/adaptive_scanning_simulation.git
git clone --branch dependency_fix \
  https://github.com/Robolab-Development/Dyne-vision-common.git
git clone --branch dependency_fix \
  https://github.com/Robolab-Development/Dyne-vision-robot.git
git clone --branch dependency_fix \
  https://github.com/Robolab-Development/Dyne-vision-sensor.git
git clone --branch dependency_fix \
  https://github.com/Eigen-Dyne/adaptive_scanning_interfaces.git
```

The interface repository is listed separately because it is the public ROS
message/action contract shared by Common and Robot.

Install the ROS dependencies from the workspace root. The Robot package also
requires the Jazzy Doosan ROS 2 stack described by its container and deployment
setup.

```bash
cd ~/adaptive_scanning_ws
source /opt/ros/jazzy/setup.bash
rosdep update
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy
```

## Self-contained ROS simulation

This path runs Gazebo, Robot, Common, and the scan pipeline directly in the host
ROS workspace. It does not start the ArmX-E application.

```bash
cd ~/adaptive_scanning_ws/src/adaptive_scanning_simulation

./sim/sim.sh build
SIM_PART=10 SIM_GZ_GUI=true ./sim/sim.sh up
```

Useful commands:

```bash
./sim/sim.sh status
./sim/sim.sh logs gazebo
./sim/sim.sh logs moveit
./sim/sim.sh logs scan
./sim/sim.sh down
```

The stable automated path is headless. Set `SIM_GZ_GUI=true` only when a Gazebo
window is required.

## ArmX-E application test with simulated hardware

This path starts simulated hardware from this repository and the ArmX-E
application services from the Common repository. The Common scan can run on the
host or in its production-like container while communicating with the same ROS
graph.

```bash
cd ~/adaptive_scanning_ws/src/Dyne-vision-common

ARMX_SIM_OBJECT=10.stl \
ARMX_SIM_GZ_GUI=true \
ARMX_SIM_USE_MOCK_AI=false \
ARMX_AI_UPLOAD_FALLBACK_ENABLED=true \
ARMX_SIM_START_RVIZ=true \
ARMX_BRIDGE_START_TIMEOUT_S=120 \
./armx-e/scripts/armx-e_sim.sh start
```

The launcher automatically finds `../adaptive_scanning_simulation`. For a
different checkout layout, set:

```bash
export ADAPTIVE_SCANNING_SIM_ROOT=/absolute/path/to/adaptive_scanning_simulation
```

To run the Common scan in its container instead of as a host ROS process:

```bash
export ARMX_SIM_EDGE_SCAN_BACKEND=docker_common
export ADAPTIVE_SCANNING_INTERFACES_REF=dependency_fix
```

The Robot and Gazebo processes remain host-side in this test path. The Robot
production container is intended for physical hardware; Gazebo supplies its
replacement hardware interfaces during simulation.

## Runtime stages

1. `as_sim/gazebo.launch.py` loads the world, Robot xacro, controllers, camera,
   and selected STL object.
2. `as_edge_robot_doosan_a0912/robot.launch.py mode:=gazebo` starts one MoveIt
   instance and the Robot-owned motion/calibration/telemetry interfaces.
3. Common starts the scan orchestration and consumes the same Robot actions and
   camera topics used in production.
4. With the ArmX-E launcher, the UI and application services control the scan
   job and receive its status.

Detailed runner options are documented in [sim/README.md](sim/README.md).

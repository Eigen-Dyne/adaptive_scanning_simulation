# syntax=docker/dockerfile:1.7
#
# DEV-ONLY IMAGE. Never deployed to a fleet.
#
# It exists so that the images that actually ship (robot, common, sensor) are
# the ones exercised in simulation, instead of a host colcon build of source.
#
# Build this image through the repository lifecycle command:
#
#   ./scripts/adaptive-scanning build --with-sim
#
# Its build context is components/simulation. Bake supplies the just-built
# Robot image as a named context, so it must not be built independently.
#
# Why FROM the robot image:
#   Gazebo loads gz_ros2_control *inside its own process*, and that plugin
#   resolves $(find as_edge_robot_doosan_a0912)/config/dsr_controller2_gz.yaml
#   (doosan/dsr_description/xacro/macro.gazebo.scanner.xacro). It also needs the
#   robot meshes and runs as_edge_robot_doosan_a0912/ensure_controllers.py.
#   Whichever container runs Gazebo must carry the robot package; inheriting it
#   is the only option that does not duplicate ~90 MB of Doosan assets and let
#   them drift out of sync.
#
# This container owns ONLY the `gazebo` stage of sim.sh:
#   simulation -> Gazebo, world+robot spawn, robot_state_publisher,
#                 gz_ros2_control (/dsr01/controller_manager), gz<->ROS bridge
#   robot      -> robot.launch.py mode:=gazebo  (MoveIt, action servers)
#   common     -> scan orchestration
# This matches the contract robot.launch.py already logs:
#   "expecting as_sim gazebo.launch.py to own robot_state_publisher and
#    /dsr01/controller_manager"

ARG ROBOT_IMAGE=dyne-vision-robot:local
FROM ${ROBOT_IMAGE} AS runtime

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ros_gz / gz_ros2_control are deliberately skip-keyed in the robot image's
# rosdep invocation; simulation adds them on top. The robot package is not
# rebuilt.
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-ros-gz-sim \
      ros-jazzy-ros-gz-bridge \
      ros-jazzy-ros-gz-interfaces \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-ros2controlcli \
      libegl1 libgl1 libglx-mesa0 libgles2 libgl1-mesa-dri \
      procps psmisc \
    && rm -rf /var/lib/apt/lists/*

COPY as_sim /edge_ws/src/as_sim

WORKDIR /edge_ws
RUN source /opt/ros/jazzy/setup.bash && \
    source /edge_ws/doosan_ws/install/setup.bash && \
    source /edge_ws/install/setup.bash && \
    colcon build --symlink-install --packages-select as_sim

COPY sim /opt/adaptive_scanning/sim
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# ADAPTIVE_SCANNING_WS_ROOT defuses sim/lib.sh's "${SIM_REPO_ROOT}/../.."
# workspace heuristic, which is wrong inside the image.
ENV ADAPTIVE_SCANNING_WS_ROOT=/edge_ws \
    ADAPTIVE_SCANNING_SIM_LOG_DIR=/runtime \
    SIM_SKIP_BUILD=1 \
    SIM_SKIP_PREPARE=1 \
    QT_QPA_PLATFORM=xcb

ENTRYPOINT ["/entrypoint.sh"]
CMD ["gazebo"]

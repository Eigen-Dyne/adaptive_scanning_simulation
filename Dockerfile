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
# Its build context is components/simulation. Bake supplies Robot's stable
# dependencies and current runtime as named contexts.
#
# Why share Robot's dependencies and built workspace:
#   Gazebo loads gz_ros2_control *inside its own process*, and that plugin
#   resolves $(find as_edge_robot_doosan_a0912)/config/dsr_controller2_gz.yaml
#   (doosan/dsr_description/xacro/macro.gazebo.scanner.xacro). It also needs the
#   robot meshes and runs as_edge_robot_doosan_a0912/ensure_controllers.py.
#   Whichever container runs Gazebo must carry the current robot package.
#   Copy the whole workspace to preserve colcon's source/install symlinks.
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
FROM ${ROBOT_IMAGE} AS robot-runtime
FROM robot-deps AS simulation-deps

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# Keep downloads independent of robot code, meshes and build timestamps.
# Cache archives as well as indexes for interrupted or dependency-change builds.
ARG TARGETARCH
RUN --mount=type=cache,id=armx-sim-apt-${TARGETARCH},target=/var/cache/apt,sharing=locked \
    --mount=type=cache,id=armx-sim-apt-lists-${TARGETARCH},target=/var/lib/apt/lists,sharing=locked \
    rm -f /etc/apt/apt.conf.d/docker-clean && \
    apt-get -o Acquire::Retries=3 update && \
    apt-get -o Acquire::Retries=3 install -y --no-install-recommends \
      ros-jazzy-ros-gz-sim \
      ros-jazzy-ros-gz-bridge \
      ros-jazzy-ros-gz-interfaces \
      ros-jazzy-gz-ros2-control \
      ros-jazzy-ros2controlcli \
      libegl1 libgl1 libglx-mesa0 libgles2 libgl1-mesa-dri \
      procps psmisc

FROM simulation-deps AS runtime

# Application updates invalidate these copies, never the dependency install.
COPY --from=robot-runtime /edge_ws /edge_ws

COPY as_sim/package.xml as_sim/CMakeLists.txt /edge_ws/src/as_sim/

WORKDIR /edge_ws
RUN source /opt/ros/jazzy/setup.bash && \
    source /edge_ws/doosan_ws/install/setup.bash && \
    source /edge_ws/install/setup.bash && \
    colcon build --symlink-install --packages-select as_sim --cmake-target all

# Keep the large dataset/model layer ahead of frequently edited launch/scripts.
COPY as_sim/worlds /edge_ws/src/as_sim/worlds
COPY as_sim/launch /edge_ws/src/as_sim/launch
COPY as_sim/scripts /edge_ws/src/as_sim/scripts
RUN cmake --install /edge_ws/build/as_sim

COPY sim /opt/adaptive_scanning/sim
COPY --chmod=755 docker/entrypoint.sh /entrypoint.sh
# A new build timestamp must not rerun even the small as_sim configure/install.
COPY --from=robot-runtime /opt/armx /opt/armx

# ADAPTIVE_SCANNING_WS_ROOT defuses sim/lib.sh's "${SIM_REPO_ROOT}/../.."
# workspace heuristic, which is wrong inside the image.
ENV ADAPTIVE_SCANNING_WS_ROOT=/edge_ws \
    ADAPTIVE_SCANNING_SIM_LOG_DIR=/runtime \
    SIM_SKIP_BUILD=1 \
    SIM_SKIP_PREPARE=1 \
    QT_QPA_PLATFORM=xcb \
    ROBOT_FIXTURE=zeiss

ENTRYPOINT ["/entrypoint.sh"]
CMD ["gazebo"]

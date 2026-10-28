# MoveIt 2 + MoveIt Task Constructor side of the Isaac Sim digital twin (ROS 2 Jazzy, arm64 / amd64),
# with the repo's mtc_dynamic package built in. Build context = the package source:
#   docker build -t twin-moveit -f docker/moveit/Dockerfile ros2_ws/mtc_controller_final/src
# (docker/moveit/run_mtc.sh does this, then runs launch/isaac.launch.py).
FROM ros:jazzy

ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-moveit \
      ros-jazzy-moveit-task-constructor-core \
      ros-jazzy-moveit-task-constructor-capabilities \
      ros-jazzy-moveit-task-constructor-msgs \
      ros-jazzy-moveit-resources-panda-moveit-config \
      ros-jazzy-moveit-configs-utils \
      ros-jazzy-ros2-control \
      ros-jazzy-ros2-controllers \
      ros-jazzy-generate-parameter-library \
      ros-jazzy-py-binding-tools \
      ros-jazzy-octomap-msgs \
      ros-jazzy-geometric-shapes \
      ros-jazzy-tf2-eigen \
      python3-colcon-common-extensions \
    && rm -rf /var/lib/apt/lists/*

COPY . /ws/src/mtc_dynamic
RUN . /opt/ros/jazzy/setup.sh && cd /ws \
    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF --event-handlers console_cohesion+ \
    && rm -rf build log

# Cross-container DDS: UDP only (default SHM transport fails between containers with separate /dev/shm).
# ROS_DOMAIN_ID is deliberately NOT baked in: pass the same explicit -e ROS_DOMAIN_ID as the Isaac container.
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    FASTDDS_BUILTIN_TRANSPORTS=UDPv4
RUN echo 'source /ws/install/setup.bash' >> /root/.bashrc

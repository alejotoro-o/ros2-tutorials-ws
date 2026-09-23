FROM osrf/ros:jazzy-desktop

ENV DEBIAN_FRONTEND=noninteractive

# System + build deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    wget \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-pip \
    python3-numpy \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Simulation deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-webots-ros2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-cartographer-ros \
    ros-jazzy-rtabmap-ros \
    && rm -rf /var/lib/apt/lists/*

# Webots R2025a
RUN wget -q -O /tmp/webots.deb \
    https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb \
    && apt-get update \
    && apt-get install -y /tmp/webots.deb \
    && rm /tmp/webots.deb \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -ms /bin/bash ros2
USER ros2
WORKDIR /home/ros2/ros2_ws

COPY --chown=ros2:ros2 src/ src/
COPY --chown=ros2:ros2 generate_ros_map/ generate_ros_map/

RUN . /opt/ros/jazzy/setup.sh \
    && rosdep update \
    && rosdep install --from-paths src --ignore-src -r -y \
    && rm -rf build install log \
    && colcon build --symlink-install --packages-skip rosmasterx3_sim \
    && colcon build --packages-select rosmasterx3_sim \
    && MESHES_DIR="/home/ros2/ros2_ws/install/rosmasterx3_sim/share/rosmasterx3_sim/meshes" \
    && WBT_DIR="/home/ros2/ros2_ws/install/rosmasterx3_sim/share/rosmasterx3_sim/worlds" \
    && sed -i "s|\"//.*//meshes/|\"${MESHES_DIR}/|g;s|\".*/rosmasterx3_sim/meshes/|\"${MESHES_DIR}/|g" "${WBT_DIR}/rosmasterx3.wbt" "${WBT_DIR}/multi_robot.wbt" \
    && echo "source /opt/ros/jazzy/setup.bash" >> /home/ros2/.bashrc \
    && echo "source /home/ros2/ros2_ws/install/setup.bash" >> /home/ros2/.bashrc

COPY --chown=ros2:ros2 docker-entrypoint.sh /
ENTRYPOINT ["/docker-entrypoint.sh"]
CMD ["bash"]

# Base ROS2 image (Jazzy Jalisco)
FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

# ─── Temel Araçlar ───
RUN apt-get update && apt-get install -y \
    wget gnupg \
    python3-colcon-common-extensions \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# ─── Gazebo Repo ───
RUN wget https://packages.osrfoundation.org/gazebo.gpg \
      -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] \
      http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null

# ─── Tüm Bağımlılıklar (Python + C++ Autonomy) ───
RUN apt-get update && apt-get install -y \
    ros-jazzy-ros-gz \
    python3-gz-transport13 \
    python3-gz-msgs10 \
    ros-jazzy-cv-bridge \
    ros-jazzy-image-transport \
    libopencv-dev \
    ffmpeg \
    && rm -rf /var/lib/apt/lists/*

# ─── Workspace ───
WORKDIR /ros2_ws
COPY src/ /ros2_ws/src/

RUN source /opt/ros/$ROS_DISTRO/setup.bash && colcon build --symlink-install

COPY ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "run", "siha_telemetri", "harita_node"]

# syntax=docker/dockerfile:1

# Ubuntu 22.04 (jammy) + ROS 2 Humble, ros-base variant.
# On an M2 this resolves to the arm64 image and builds natively - no --platform
# flag and no Rosetta needed. ROS Humble and Ignition Fortress both publish
# arm64 debs for jammy.
FROM ros:humble-ros-base-jammy

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
WORKDIR /root/rosmaster_ws

ARG USERNAME=alumno
ARG USER_UID=1000
ARG USER_GID=1000

RUN getent group render || groupadd -g 128 render \
    && groupadd --gid ${USER_GID} ${USERNAME} 2>/dev/null || true \
    && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash ${USERNAME} \
    && usermod -aG dialout,video,render,plugdev ${USERNAME} \
    && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
    && chmod 0440 /etc/sudoers.d/${USERNAME}
# ---------------------------------------------------------------------------
# Gazebo Ignition 
# ---------------------------------------------------------------------------
RUN apt update && apt install -y ros-${ROS_DISTRO}-ros-gz

# ---------------------------------------------------------------------------
# Yahboom dependencies
# ---------------------------------------------------------------------------
RUN apt update && apt install -y \
    build-essential \
    cmake \
    git \
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-ros-gz \
    ros-humble-teleop-twist-keyboard

# ---------------------------------------------------------------------------
# Building Yahboom
# ---------------------------------------------------------------------------
USER ${USERNAME}
WORKDIR /home/${USERNAME}/rosmaster_ws

RUN mkdir -p src && cd src \
    && git clone https://github.com/bchax/yahboom_rosmaster.git

RUN rosdep init 2>/dev/null || true
RUN rosdep update
RUN rosdep install --from-paths src --ignore-src -r -y --rosdistro humble

RUN source /opt/ros/${ROS_DISTRO}/setup.bash \
    && colcon build --symlink-install


USER root
RUN rm -rf /var/lib/apt/lists/*
USER ${USERNAME}
WORKDIR /home/${USERNAME}/rosmaster_ws

# ---------------------------------------------------------------------------
# Shell env
# ---------------------------------------------------------------------------
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc \
    && echo "source ~/rosmaster_ws/install/setup.bash" >> ~/.bashrc
 
#ENV XDG_RUNTIME_DIR=/tmp/runtime-${USERNAME}
#RUN mkdir -p /tmp/runtime-${USERNAME} && chmod 0700 /tmp/runtime-${USERNAME}
 
CMD ["bash"]

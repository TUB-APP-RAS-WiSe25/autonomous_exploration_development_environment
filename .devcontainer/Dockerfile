# Base: ROS 2 Humble + GUI tools on Ubuntu 22.04
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive \
    TZ=Etc/UTC \
    LANG=en_US.UTF-8 \
    LC_ALL=en_US.UTF-8

# Create a non-root user that matches VS Code's devcontainers defaults
ARG USERNAME=vscode
ARG USER_UID=1000
ARG USER_GID=1000

RUN apt-get update && apt-get install -y --no-install-recommends \
    sudo locales tzdata ca-certificates curl git bash-completion \
    # requirements from  https://drive.google.com/file/d/1a01RSrPFan9SV2g2BFP_8GGtnamAdGIc/view
    libusb-dev \
    ros-humble-joy \
    ros-humble-gazebo-msgs \
    ros-humble-gazebo-plugins \
    ros-humble-gazebo-ros \
    ros-humble-gazebo-ros2-control \
    ros-humble-gazebo-ros-pkgs \
    python3-colcon-common-extensions \
    ros-humble-octomap-ros \
    libgoogle-glog-dev libgflags-dev \
    mesa-utils x11-apps \
    ros-humble-rmw-cyclonedds-cpp \
    unzip \
 && rm -rf /var/lib/apt/lists/*

# Locale
RUN locale-gen en_US.UTF-8

# Create user and allow sudo
RUN groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd -s /bin/bash --uid ${USER_UID} --gid ${USER_GID} -m ${USERNAME} \
 && echo "${USERNAME} ALL=(root) NOPASSWD:ALL" >/etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# rosdep: initialize as root; update will be run by the user later
RUN rosdep init || true

# Default to Cyclone DDS (robust choice in Humble)
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# Make ROS & colcon available in all shells
RUN echo "source /opt/ros/humble/setup.bash" >> /etc/bash.bashrc

# X11 / Wayland friendly env
ENV DISPLAY=:0 \
    QT_X11_NO_MITSHM=1 \
    LIBGL_ALWAYS_INDIRECT=0

# NVIDIA hints (no effect unless runtime passes GPUs)
ENV NVIDIA_VISIBLE_DEVICES=void \
    NVIDIA_DRIVER_CAPABILITIES=none

USER ${USERNAME}
WORKDIR /workspaces

# Shell quality of life
RUN echo 'source /usr/share/bash-completion/bash_completion' >> ~/.bashrc && \
    echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'export COLCON_HOME=$HOME/.colcon' >> ~/.bashrc && \
    # Optional: run rosdep update when the container starts an interactive shell
    echo 'command -v rosdep >/dev/null || true' >> ~/.bashrc

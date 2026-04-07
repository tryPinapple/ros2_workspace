FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive

# Base + locales
RUN apt update && apt install -y \
    locales \
    curl \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && locale-gen en_US en_US.UTF-8

ENV LANG=en_US.UTF-8
ENV LC_ALL=en_US.UTF-8

# Ajouter la clé ROS (méthode moderne, pas apt-key)
RUN mkdir -p /etc/apt/keyrings && \
    curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    | gpg --dearmor -o /etc/apt/keyrings/ros-archive-keyring.gpg

# Ajouter le repo ROS 2 (jammy)
RUN echo "deb [signed-by=/etc/apt/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" \
    > /etc/apt/sources.list.d/ros2.list

# Installer ROS 2 + outils dev
RUN apt update && apt install -y \
    ros-humble-desktop \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    git \
    build-essential \
    && rm -rf /var/lib/apt/lists/*

# Init rosdep
RUN rosdep init || true && rosdep update

# Source automatique
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /workspace

CMD ["/bin/bash"]

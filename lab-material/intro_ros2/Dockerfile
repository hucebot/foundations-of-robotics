# ROS 2 Humble base
FROM ros:humble

ENV DEBIAN_FRONTEND=noninteractive
ENV COLCON_WS=/root/ros2_ws

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-desktop \
    python3-argcomplete \
    python3-evdev \
    python3-colcon-common-extensions \
    build-essential \
    git \
    terminator \
    nano \
    x11-xserver-utils \
    libx11-6 \
    libxext6 \
    libxrender1 \
    libxtst6 \
    libxi6 \
    && rm -rf /var/lib/apt/lists/*

# Convenience environment
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR ${COLCON_WS}
RUN mkdir src

CMD ["bash"]

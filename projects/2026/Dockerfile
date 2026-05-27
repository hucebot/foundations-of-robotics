FROM ros:jazzy-ros-base

SHELL ["/bin/bash", "-c"]

ENV DEBIAN_FRONTEND="noninteractive"
ENV TZ="Europe/Paris"

RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-venv \
    libgl1 \
    libglib2.0-0 \
    libx11-xcb1 \
    libxrender1 \
    libxi6 \
    libxkbcommon-x11-0 \
    libsm6 \
    libxext6 \
    libegl1

RUN python3 -m venv /opt/venv
ENV PATH="/opt/venv/bin:$PATH"

RUN pip install --upgrade pip
RUN pip install numpy PyQt6 pyqtgraph matplotlib scipy

RUN apt-get update && apt-get install -y \
    libxcb-cursor0 \
    libxcb-xinerama0 \
    libxcb-icccm4 \
    libxcb-image0 \
    libxcb-keysyms1 \
    libxcb-randr0 \
    libxcb-render-util0 \
    libxcb-xkb1 \
    libxkbcommon-x11-0 \
    libxcb-shape0 \
    terminator
    
WORKDIR /home
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

WORKDIR /home/ros2_ws

RUN colcon build
RUN echo "source /home/ros2_ws/install/local_setup.bash" >> ~/.bashrc

WORKDIR /home/ros2_ws/src
RUN git clone https://github.com/hucebot/unicycle_dynamic_feedback_linearization_deluca_oriolo_vendittelli_2000.git

WORKDIR /home/ros2_ws/
RUN rosdep install --from-paths src --ignore-src -r -y
RUN colcon build --packages-select turtle_node


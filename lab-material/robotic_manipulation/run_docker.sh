#!/usr/bin/env bash

CONTAINER_NAME=franka_humble
IMAGE_NAME=ghcr.io/fabio-amadio/panda-ros2:2026
COLCON_WS=/home/user/humble_ws

isRunning=$(docker ps -f name=${CONTAINER_NAME} | grep -c ${CONTAINER_NAME})

is_wsl() {
    grep -qi microsoft /proc/version
}

if [ "$isRunning" -eq 0 ]; then
    echo "Starting Franka Panda ROS 2 container..."

    # Allow Docker to access X server (skip in WSL)    
    if ! is_wsl; then
        xhost +local:docker >/dev/null
    fi

    # Remove stopped container if it exists
    docker rm ${CONTAINER_NAME} >/dev/null 2>&1

    docker run \
        -it \
        --name ${CONTAINER_NAME} \
        --network host \
        --ipc host \
        -e DISPLAY=${DISPLAY} \
        -e QT_X11_NO_MITSHM=1 \
        -e ROS_DOMAIN_ID=30 \
        -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
        -e SDL_AUDIODRIVER=dummy \
        -e ALSOFT_DRIVERS=null \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v /dev/shm:/dev/shm \
        -v ./franka_example_controllers:${COLCON_WS}/src/multipanda/franka_example_controllers \
        -v ./franka_simple_publishers:${COLCON_WS}/src/multipanda/franka_simple_publishers \
        --entrypoint terminator \
        ${IMAGE_NAME}

else
    echo "Franka Panda ROS 2 container already running."
    docker exec -it ${CONTAINER_NAME} terminator
fi

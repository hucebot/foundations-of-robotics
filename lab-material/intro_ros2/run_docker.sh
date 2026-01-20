#!/usr/bin/env bash

CONTAINER_NAME=intro-ros2
IMAGE_NAME=ghcr.io/fabio-amadio/intro-ros2:2026
COLCON_WS=/root/ros2_ws

isRunning=$(docker ps -f name=${CONTAINER_NAME} | grep -c ${CONTAINER_NAME})

is_wsl() {
    grep -qi microsoft /proc/version
}

if [ "$isRunning" -eq 0 ]; then
    echo "Starting ROS 2 Humble container..."

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
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v ./exercise_pub_sub:${COLCON_WS}/src/exercise_pub_sub \
        --entrypoint terminator \
        ${IMAGE_NAME}

else
    echo "ROS 2 container already running."
    docker exec -it ${CONTAINER_NAME} terminator
fi


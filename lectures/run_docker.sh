#!/usr/bin/env bash

CONTAINER_NAME=robotics-lectures
IMAGE_NAME=ghcr.io/fabio-amadio/robotics-lectures:2026
ROOT_DIR="$(pwd)"

isRunning=$(docker ps -f name=${CONTAINER_NAME} | grep -c ${CONTAINER_NAME})

is_wsl() {
    grep -qi microsoft /proc/version
}

if [ $isRunning -eq 0 ]; then
    echo "Starting Robotics-Lectures container..."

	# Allow Docker to access X server (skip in WSL)    
    if ! is_wsl; then
        xhost +local:docker >/dev/null
    fi

	# Remove stopped container if it exists
    docker rm ${CONTAINER_NAME} >/dev/null 2>&1

	docker run \
        -it \
        --name ${CONTAINER_NAME} \
		--security-opt seccomp=unconfined \
		--interactive \
		--tty \
		--net host \
		--rm \
		--env DISPLAY=$DISPLAY \
		--env ROS_DOMAIN_ID=69 \
		--privileged \
		--volume /tmp/.X11-unix:/tmp/.X11-unix \
		--volume ${ROOT_DIR}/lecture1:/home/introduction_to_robotics/lecture1 \
		--volume ${ROOT_DIR}/lecture2:/home/introduction_to_robotics/lecture2 \
		--volume ${ROOT_DIR}/lecture3:/home/introduction_to_robotics/lecture3 \
		--volume ${ROOT_DIR}/lecture4:/home/introduction_to_robotics/lecture4 \
		--volume ${ROOT_DIR}/lecture4:/home/introduction_to_robotics/lecture5 \
		${IMAGE_NAME}

else
    echo "Docker Robotics-Lectures already running."
    docker exec -it ${CONTAINER_NAME} /bin/bash
fi

# For graphics
xhost +

isRunning=`docker ps -f name=introduction_to_robotics | grep -c "introduction_to_robotics"`;
ROOT_DIR="$(pwd)"

if [ $isRunning -eq 0 ]; then
	docker rm introduction_to_robotics
	docker run \
		--security-opt seccomp=unconfined \
		--name introduction_to_robotics  \
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
		introduction_to_robotics

else
    echo "Docker already running."
    docker exec -it introduction_to_robotics /bin/bash
fi

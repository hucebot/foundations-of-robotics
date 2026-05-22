# For graphics
xhost +

isRunning=`docker ps -f name=robotics-projects-2026 | grep -c "robotics-projects-2026"`;

if [ $isRunning -eq 0 ]; then
	docker rm robotics-projects-2026
	docker run \
		--name robotics-projects-2026  \
		--interactive \
		--tty \
		--net host \
		--rm \
		--env DISPLAY=$DISPLAY \
		--privileged \
		--volume /tmp/.X11-unix:/tmp/.X11-unix \
		--volume $(pwd)/task_priority_framework_siciliano_slotine_1991:/home/projects/task_priority_framework_siciliano_slotine_1991 \
		robotics-projects-2026

else
    echo "Docker already running."
    docker exec -it robotics-projects-2026 /bin/bash
fi

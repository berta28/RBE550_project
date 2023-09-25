#! /bin/bash

DOCKER_IMAGE="rbe550_project:0.0.0"
CMD_S="bash"
xhost +
docker run \
	--rm \
	-it \
	--net=host \
	--privileged \
	-e DISPLAY=$DISPLAY \
	-v /tmp/.X11-unix:/tmp/.X11-unix \
	-v ~/src:/mnt/src \
	--name ros_dev \
	${DOCKER_IMAGE} ${CMD_S}
#
# This is a project Makefile. It is assumed the directory this Makefile resides in is a
# project subdirectory.
#

BASE_IMG=ubuntu:18.04
BASE_IMG_NVIDIA=nvidia/opengl:1.2-glvnd-runtime-ubuntu18.04
IMG_NAME=gazebo-rl-img
CONTAINER_NAME=gazebo-rl

SRC_DIR=${PWD}
USER_NAME=$$(whoami)
USER_ID=$$(id -u)
GROUP_ID=$$(id -g)

DOCKER_BUILD=docker build -t ${IMG_NAME} -f ${SRC_DIR}/Dockerfile ${SRC_DIR} \
				--network=host \
				--build-arg user_name=${USER_NAME} \
				--build-arg user_id=${USER_ID} \
				--build-arg group_id=${GROUP_ID} \
				--build-arg base_image=

DOCKER_RUN=docker run \
				-ti --rm \
				-u ${USER_ID}:${GROUP_ID} \
				-e "DISPLAY" \
				-e "QT_X11_NO_MITSHM=1" \
				-v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
				-e XAUTHORITY \
				-v ${SRC_DIR}:${SRC_DIR}:rw \
				--net=host \
				--privileged \
				--name ${CONTAINER_NAME} \
				-w ${SRC_DIR}/catkin_ws

build:
	${DOCKER_BUILD}${BASE_IMG} 

build-nvidia:
	${DOCKER_BUILD}${BASE_IMG_NVIDIA}

run:
	${DOCKER_RUN} ${IMG_NAME}

run-nvidia:
	${DOCKER_RUN} --gpus all ${IMG_NAME} bash -c " \
											source /opt/ros/melodic/setup.bash && \
											catkin_make && \
											echo 'source ${SRC_DIR}/catkin_ws/devel/setup.bash' >> ~/.bashrc && \
											bash"

connect:
	docker exec -ti ${CONTAINER_NAME} bash

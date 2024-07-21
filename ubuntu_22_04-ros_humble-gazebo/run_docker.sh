#!/bin/bash

xhost +local:docker || true

ROOT_DIR=$PWD

if [[ $1 = "--nvidia" ]] || [[ $1 = "-n" ]]
  then
    docker run --gpus all \
                -ti --rm \
                -e "DISPLAY" \
                -e "QT_X11_NO_MITSHM=1" \
                -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                -e XAUTHORITY \
                -v $ROOT_DIR/catkin_ws:/catkin_ws \
               --network=host \
               --privileged \
               --name ros2-humble-gazebo ros2-humble-gazebo-img

else

    echo "[!] If you wanna use nvidia gpu, please use script with -n or --nvidia argument"
    docker run  -ti --rm \
                -e "DISPLAY" \
                -e "QT_X11_NO_MITSHM=1" \
                -v "/tmp/.X11-unix:/tmp/.X11-unix:rw" \
                -e XAUTHORITY \
                -v $ROOT_DIR/catkin_ws:/catkin_ws \
               --network=host \
               --privileged \
               --name ros2-humble-gazebo ros2-humble-gazebo-img
fi

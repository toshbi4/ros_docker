#!/usr/bin/env bash

ROOT_DIR=$PWD
EXEC_PATH=$PWD

cd $ROOT_DIR


if [[ $1 = "--nvidia" ]] || [[ $1 = "-n" ]]
  then
    docker build -t ros2-humble-gazebo-img -f $ROOT_DIR/Dockerfile $ROOT_DIR \
                                            --network=host \
                                            --build-arg from=nvidia/opengl:1.2-glvnd-runtime-ubuntu22.04

else
    echo "[!] If you use nvidia gpu, please rebuild with -n or --nvidia argument"
    docker build -t ros2-humble-gazebo-img -f $ROOT_DIR/Dockerfile $ROOT_DIR \
                                            --network=host \
                                            --build-arg from=ubuntu:22.04
fi

cd $EXEC_PATH

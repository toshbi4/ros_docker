ARG from

FROM ${from}

RUN apt-get update && apt-get install -y apt-utils \
                                         lsb-release \
                                         mesa-utils \
                                         gnupg2 \
                                         net-tools \
                                         build-essential \
                                         wget \
                                         unzip \
                                         curl \
                                         git \
                                         mc


# Timezone Configuration

ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install ROS desktop and Gazebo
RUN DEBIAN_FRONTEND=noninteractive && \
     apt install -y software-properties-common && \
     add-apt-repository universe && \
     apt update && apt install curl -y && \
     curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \ 
     -o /usr/share/keyrings/ros-archive-keyring.gpg && \
     echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
     http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
     tee /etc/apt/sources.list.d/ros2.list > /dev/null && \
     apt-get update && \
                       apt-get install -y ros-humble-desktop-full \
                                          gazebo \
                                          ros-humble-gazebo-ros-pkgs \
                                          ros-humble-control-toolbox \
                                          ros-humble-gazebo-ros2-control \
                                          ros-humble-geographic-info \
                                          ros-humble-teleop-tools \
                                          ros-humble-joy \
                                          ros-humble-effort-controllers \
                                          ros-humble-controller-manager \
                                          ros-humble-turtlebot3-gazebo \
                                          ros-humble-turtlebot3 \
                                          ros-humble-turtlebot3-example \
                                          ros-humble-turtlebot3-node \
                                          python3-rosdep \
                                          libcanberra-gtk-module \
                                          libcanberra-gtk3-module \
                                          ros-humble-visp* && \
     rosdep init && rosdep update && \
     echo "source /opt/ros/humble/setup.bash"  >> ~/.bashrc

# Install extra libraries
RUN apt-get install -y libvisp-dev libvisp-doc 
RUN apt-get install -y python3-pip
RUN pip3 install pathlib statistics scipy
RUN apt install -y python3-colcon-core

RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
                      apt-get install -y ros-humble-tf2-sensor-msgs \
                                            ros-humble-image-proc

RUN apt-get update && apt-get install -y \
                libopencv-dev libx11-dev liblapack-dev libv4l-dev libzbar-dev libpthread-stubs0-dev libsdl2-dev libsdl-image1.2-dev \
                libeigen3-dev

RUN pip install numpy scikit-image 
RUN pip install pillow 
RUN pip install opencv-contrib-python

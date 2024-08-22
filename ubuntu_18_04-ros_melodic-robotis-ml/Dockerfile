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
                                         mc \
                                         screen


# Timezone Configuration

ENV TZ=Europe/Moscow
RUN ln -snf /usr/share/zoneinfo/$TZ /etc/localtime && echo $TZ > /etc/timezone

# Install ROS desktop and Gazebo

RUN sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' && \
    apt-key adv --keyserver 'hkp://keyserver.ubuntu.com:80' --recv-key C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update && DEBIAN_FRONTEND=noninteractive \
                      apt-get install -y ros-melodic-desktop-full \
                                         gazebo9 \
                                         ros-melodic-gazebo-ros-pkgs \
                                         ros-melodic-ros-control \
                                         ros-melodic-gazebo-ros-control \
                                         ros-melodic-geographic-info \
                                         ros-melodic-teleop-twist-keyboard \
                                         ros-melodic-joy \
                                         ros-melodic-effort-controllers \
                                         ros-melodic-controller-manager \
                                         python-rosdep \
                                         python-rosinstall \
                                         python-rosinstall-generator \
                                         python-wstool \
                                         python-catkin-tools \
                                         libcanberra-gtk-module \
                                         libcanberra-gtk3-module \
                                         ros-melodic-pid && \
                                         # ros-melodic-visp* && \
    rosdep init && rosdep update && \
    echo "source /opt/ros/melodic/setup.bash"  >> ~/.bashrc && \
    echo "source ~/catkin_ws/devel/setup.bash"  >> ~/.bashrc

# Install extra libraries
RUN apt-get install -y libvisp-dev libvisp-doc 
RUN apt-get install -y python-pip
RUN pip install pathlib statistics scipy

RUN apt-get update && DEBIAN_FRONTEND=noninteractive \
                      apt-get install -y ros-melodic-ecl-geometry \
                                            ros-melodic-tf2-sensor-msgs \
                                            ros-melodic-move-base-msgs \
                                            ros-melodic-image-proc

RUN apt-get update && apt-get install -y \
                libopencv-dev libx11-dev liblapack-dev libv4l-dev libzbar-dev libpthread-stubs0-dev libsdl-dev libsdl-image1.2-dev \
                libeigen3-dev

RUN pip install scikit-image 
RUN pip install pillow 
RUN pip install opencv-contrib-python

# ML
RUN wget https://repo.anaconda.com/archive/Anaconda2-5.2.0-Linux-x86_64.sh
RUN python -V
RUN pip install msgpack argparse
RUN pip install -U rosinstall empy defusedxml netifaces
RUN pip install keras==2.1.5

# catkin ws
RUN apt install -y ros-melodic-turtlebot3 ros-melodic-turtlebot3-msgs ros-melodic-turtlebot3-simulations
RUN mkdir -p ~/catkin_ws/src/ && cd ~/catkin_ws/src/ && git clone https://github.com/toshbi4/turtlebot3_machine_learning.git
RUN cd ~/catkin_ws/src/ && git clone -b kinetic-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
RUN /bin/bash -c 'source /opt/ros/melodic/setup.bash && cd ~/catkin_ws && catkin_make'

# RUN pip install tensorflow==1.14
RUN pip install protobuf==3.6.1
RUN pip install tensorflow==1.14.0
# RUN pip install --ignore-installed --upgrade https://storage.googleapis.com/tensorflow/linux/cpu/tensorflow-1.8.0-cp27-none-linux_x86_64.whl
RUN pip install numpy pyqtgraph
RUN pip install tensorboard

# Patch to decrease a sensor scanning density
RUN sed -i 's/>360</>24</g' /opt/ros/melodic/share/turtlebot3_description/urdf/turtlebot3_waffle.gazebo.xacro
RUN sed -i 's/>360</>24</g' /opt/ros/melodic/share/turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf

FROM nvidia/cuda:8.0-cudnn6-devel-ubuntu16.04

# ROS kinetic core
RUN apt-key adv --keyserver hkp://p80.pool.sks-keyservers.net:80 --recv-keys 421C365BD9FF1F717815A3895523BAEEB01FA116
RUN echo "deb http://packages.ros.org/ros/ubuntu xenial main" > /etc/apt/sources.list.d/ros-latest.list
RUN apt-get update && apt-get install --no-install-recommends -y \
    python-rosdep \
    python-rosinstall \
    python-vcstools \
    && rm -rf /var/lib/apt/lists/*
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
RUN rosdep init \
    && rosdep update
ENV ROS_DISTRO kinetic
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-core=1.3.1-0* \
    && rm -rf /var/lib/apt/lists/*

# ROS base
RUN apt-get update && apt-get install -y \
    ros-kinetic-ros-base=1.3.1-0* \
    && rm -rf /var/lib/apt/lists/*

# ROS robot
RUN apt-get update && apt-get install -y \
    ros-kinetic-robot=1.3.1-0* \
    && rm -rf /var/lib/apt/lists/*

# ROS desktop
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop=1.3.1-0* \
    && rm -rf /var/lib/apt/lists/*

# ROS desktop full
RUN apt-get update && apt-get install -y \
    ros-kinetic-desktop-full=1.3.1-0* \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /home/catkin_ws/src
RUN apt-get update && apt-get -y install build-essential \
    git wget nano v4l-utils

WORKDIR /home/catkin_ws/src
RUN git clone --recursive -b cuda8_0-dev https://github.com/pgigioli/darknet_ros.git
RUN git clone https://github.com/bosch-ros-pkg/usb_cam.git

RUN mkdir /home/catkin_ws/src/darknet_ros/weights
WORKDIR /home/catkin_ws/src/darknet_ros/weights
RUN wget http://pjreddie.com/media/files/tiny-yolo-voc.weights
RUN wget http://pjreddie.com/media/files/yolo-voc.weights

WORKDIR /home/catkin_ws

ENV LD_LIBRARY_PATH = $LD_LIBRARY_PATH:/usr/local/cuda/lib64

COPY ./ros_entrypoint.sh /
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]

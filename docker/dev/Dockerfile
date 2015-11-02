FROM cogrob/object_tracking_2d_ros:dep
MAINTAINER Cognitive Robotics "http://cogrob.org/"

# install build tools
RUN apt-get install -y \
	cmake \
	cmake-curses-gui

# install build libraries
RUN apt-get install -y \
	libboost-all-dev \
	libflann-dev \
	libgsl0-dev \
	libgoogle-perftools-dev

# install extra ros libraries
RUN apt-get install -y \
	libeigen3-dev \
	ros-indigo-perception-pcl \
	ros-indigo-openni2-launch \
	ros-indigo-usb-cam

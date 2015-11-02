FROM cogrob/object_tracking_2d_ros:ocv
MAINTAINER Cognitive Robotics "http://cogrob.org/"

WORKDIR /root

# install dependencies
RUN apt-get install -y \
  libboost-all-dev \
  libglew-dev \
  freeglut3-dev \
  liblapack3 \
  libopencv-dev

# clone object_tracking_2d_ros project
RUN mkdir -p catkin_ws/src \
  && cd catkin_ws/src \
  && git clone https://github.com/CognitiveRobotics/object_tracking_2d_ros.git \
  && cd object_tracking_2d_ros

# build omnimapper ros wrapper
ADD build.sh catkin_ws/src/apriltags/build.sh
RUN ./catkin_ws/src/apriltags/build.sh

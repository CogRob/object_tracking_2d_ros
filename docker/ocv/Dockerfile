FROM cogrob/object_tracking_2d_ros:dev
MAINTAINER Cognitive Robotics "http://cogrob.org/"

WORKDIR /root

# clone and build OpenCV
RUN mkdir -p workspace \
  && cd workspace \
  && git clone https://github.com/Itseez/opencv.git \
  && cd opencv \
  && git checkout 2.4.9 \
  && mkdir build \
  && cd build \
  && cmake -BUILD_opencv_nonfree="True" cmake -D CMAKE_BUILD_TYPE=RELEASE -D WITH_OPENCL=OFF -D CMAKE_INSTALL_PREFIX=/usr/local .. \
  && make -j$(nproc) \
  && make install \
  && ldconfig

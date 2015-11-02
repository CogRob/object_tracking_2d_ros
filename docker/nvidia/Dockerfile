FROM cogrob/object_tracking_2d_ros:gui

RUN apt-get install -y \
	x-window-system \
	binutils \
	mesa-utils \
	module-init-tools

ADD nvidia-driver.run /tmp/nvidia-driver.run
RUN sudo sh /tmp/nvidia-driver.run -a -N --ui=none --no-kernel-module \
	&& sudo rm /tmp/nvidia-driver.run

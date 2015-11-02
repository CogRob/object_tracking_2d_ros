#!/bin/sh

docker run \
	-it \
	--env="DISPLAY" \
	--env="QT_X11_NO_MITSHM=1" \
	--volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
	--device /dev/bus/usb:/dev/bus/usb:rwm \
	--device /dev/video0:/dev/video0:rwm \
	$@

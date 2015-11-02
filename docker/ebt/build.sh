#!/bin/bash
# set -e

source /opt/ros/indigo/setup.bash
source ~/catkin_ws/devel/setup.bash

cd ~/catkin_ws/src

catkin_init_workspace

cd ..

catkin_make || true
catkin_make || true
catkin_make

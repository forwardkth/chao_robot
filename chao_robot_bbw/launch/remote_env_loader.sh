#!/bin/bash
export ROSLAUNCH_SSH_UNKNOWN=1
export ROS_IP=BBW
export ROS_MASTER_URI=http://Jetson-TX2:11311
source /opt/ros/kinetic/setup.bash
source /root/chao_robot_bbw/devel/setup.bash
export ROS_HOSTNAME=BBW
exec "$@"
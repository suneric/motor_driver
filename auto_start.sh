#!/bin/bash
export ROS_MASTER_URI=http://ubuntu-Aurora-R7:11311
export ROS_HOSTNAME=ubuntu-desktop

source /opt/ros/noetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash

python3 /home/ubuntu/catkin_ws/src/motor_driver/scripts/motor_driver_interface.py
aplay /home/ubuntu/catkin_ws/src/motor_driver/sound/heart_beat.wav

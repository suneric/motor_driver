#!/bin/bash
export ROS_HOSTNAME=ubuntu-desktop
export ROS_MASTER_URI=http://192.168.1.7:11311

source /opt/ros/noetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash

python3 /home/ubuntu/catkin_ws/src/motor_driver/scripts/motor_driver_interface.py

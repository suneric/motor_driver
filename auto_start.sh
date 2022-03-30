#!/bin/bash
export ROS_MASTER_URI=http://192.168.1.19.11311
export ROS_IP=192.168.1.15

source /opt/ros/noetic/setup.bash
source /home/ubuntu/catkin_ws/devel/setup.bash
sudo ip link set can0 up type can bitrate 100000
python3 /home/ubuntu/catkin_ws/src/motor_driver/scripts/motor_driver_interface.py
aplay /home/ubuntu/catkin_ws/src/motor_driver/sound/heart_beat.wav

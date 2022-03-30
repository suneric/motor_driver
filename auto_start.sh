export ROS_MASTER_URI=http://192.168.1.19.11311
export ROS_IP=192.168.1.15

source ~/catkin_ws/devel/setup.bash
python3 ~/catkin_ws/src/motor_driver/scripts/motor_driver_interface.py
aplay ~/catkin_ws/src/motor_driver/sound/heart_beat.wav

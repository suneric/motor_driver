# Motor Driver
Controller for [RoboMaster M2006 P36](https://www.robomaster.com/zh-CN/products/components/general/M2006) + C610 based on Can control

## equipment
- RoboMaster M2006 P36 + C610
- Raspberry Pi 4 Model B

## Software
- Ubuntu 20.04 for Raspberry Pi
- ROS Noetic

### Installation
1. Install Ubuntu 20.04
2. Install ROS Noetic
3. Install can-util and python3-can
```
sudo apt install can-util
sudo apt install python3-can
```


### network setup
- ROS_MASTER_URI = http://192.168.1.19:11311
- ROS_IP = 192.168.1.15


### clone this repo to catkin_ws/src
```
git clone https://github.com/suneric/motor_drive.git
```

### setup
1. create a service /etc/systemd/system/rm2006-service.service
```
[Unit]
Description="RM2006 P36 ROS Start"
After=network.service

[Service]
ExecStart=/home/ubuntu/catkin_ws/src/motor_driver/auto_start.sh

[Install]
WantedBy=default.target
```
2. make scripts executable
```
sudo chmod +x auto_start.sh
sudo chmod +x scripts/motor_driver_interface.py
```
3. enable service
```
sudo systemctl enable rm2006-service
```

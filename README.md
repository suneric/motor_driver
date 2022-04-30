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
- ROS_MASTER_URI = http://ubuntu-Aurora-R7:11311
- ROS_HOSTNAME=motor_drive
- add hostname to /etc/hosts
```
127.0.1.1 motor-raspi
192.168.1.7 ubuntu-Aurora-R7
```

### clone this repo to catkin_ws/src
```
git clone https://github.com/suneric/motor_drive.git
```

### socketcan service
1. create a service /etc/systemd/system/socketcan.service
```
[Unit]
Description=SocketCAN interface can0
After=multi-user.target

[Service]
Type=oneshot
RemainAfterExit=yes
ExecStart=/sbin/ip link set can0 type can bitrate 1000000 ; /sbin/ifconfig can0 up
ExecReload=/sbin/ifconfig can0 down ; /sbin/ip link set can0 type can bitrate 1000000 ; /sbin/ifconfig can0 up
ExecStop=/sbin/ifconfig can0 down

[Install]
WantedBy=multi-user.target
```
2. enable service
```
sudo systemctl enable socketcan.service
```

### setup
1. create a service /etc/systemd/system/rm2006-service.service
```
[Unit]
Description="RM2006 P36 ROS Start"
Wants=network-online.target
After=multi-user.target network.target network-online.target socketcan.service

[Service]
Type=simple
User=ubuntu
Group=ubuntu
ExecStart=/home/ubuntu/catkin_ws/src/motor_driver/auto_start.sh

[Install]
WantedBy=multi-user.target
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

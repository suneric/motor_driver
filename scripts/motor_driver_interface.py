#!/user/bin/python3

import rospy
import time
import os
import can
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension, Int32

"""
3 robomaster M2006 P36 motors, with C610 can control
ids = [0x201, 0x202, 0x203] (Hex to Decimal [513, 514, 515])
CAN protocal
1. send -1A (-1000 = [0xFC,0x18]) to motor 513 (first 2 bytes)
, 10mA (10 = [0x00, 0x0A]) to motor 514 (second two bytes), and 100 mA (100 = [0x00, 0x64]) to motor 515 (third two bytes)
data = [0xFC,0x18,0x00,0x0A,0x00,0x64,0x00,0x00]
2. receive data
 angle: (data[0]*156+data[1])/8191*360
 speed: data[2]*256+data[3]
 torque: data[4]*256+data[5]
"""

# 1 turn for 2 mm

"""
SPEED Level (A): [-2,-1.5,-1,-0.5,0,0.5,1,1.5,2]
HEX: [F830,FA24,FC18,FE0C,0000,01F4,03E8,05DC,07D0]
"""
SPEED = []

class MotorLowLevelControl:
    def __init__(self):
        self.bus = can.interface.Bus(bustype='socketcan',channel='can0',bitrate=1000000)
        rospy.loginfo("Setting up the node")
        rospy.init_node("motor_ros_interface", anonymous=True)
        self.command_pub = rospy.Publisher('/robomotor_status', Float32MultiArray, queue_size=1)
        self.command_sub = rospy.Subscriber('/robomotor_cmd', Int32, self.motor_status_cb)
        self.speed_sub = rospy.Subscriber('/robomotor_speed', Int32, self.motor_speed_cb)
        self.motor_1 = []
        self.motor_2 = []
        self.motor_3 = []
        self.speed = [0x00,0x00]

    def motor_speed_cb(self, data):
        level = data.data
        if level > 4:
            level = 4
        if level < -4:
            level = -4
        a1 = [-4,-3,-2,-1, 0, 1, 2, 3, 4]
        a2 = [[0xF8,0x30],[0xFA,0x24],[0xFC,0x18],[0xFE,0x0C],[0x00,0x00],[0x01,0xF4],[0x03,0xE8],[0x05,0xDC],[0x07,0xD0]]
        a1.index(level)
        self.speed = a2[a1.index(level)]

    def motor_status_cb(self, data):
        id = int(data.data)
        cmd_data = [self.speed[0],self.speed[1], self.speed[0],self.speed[1], self.speed[0],self.speed[1], 0x00, 0x00]
        if id == 513:
            cmd_data = [self.speed[0],self.speed[1], 0x00, 0x00, 0x00, 0x00, 0x00, 0x00]
        elif id == 514:
            cmd_data = [0x00, 0x00, self.speed[0],self.speed[1], 0x00, 0x00, 0x00, 0x00]
        elif id == 515:
            cmd_data = [0x00, 0x00, 0x00, 0x00, self.speed[0],self.speed[1], 0x00, 0x00]
        msg = can.Message(arbitration_id=512, is_extended_id=False, data=cmd_data)
        self.bus.send(msg,timeout=0.1)

    def get_s16(self, val):
        if val < 0x8000:
            return val
        else:
            return (val-0x10000)

    def motor_status(self, msg):
        id = msg.arbitration_id
        angle = self.get_s16(msg.data[0]*256+msg.data[1]) / 8191 * 360
        rpm = self.get_s16(msg.data[2]*256+msg.data[3])
        torque = self.get_s16(msg.data[4]*256+msg.data[5])
        if id == 513:
            self.motor_1 = [id, angle, rpm, torque]
        elif id == 514:
            self.motor_2 = [id, angle, rpm, torque]
        elif id == 515:
            self.motor_3 = [id, angle, rpm, torque]

        return id, angle, rpm, torque

    def publish_status(self):
        msg = self.bus.recv(100)
        id, angle, rpm, torque = self.motor_status(msg)
        # print("motor status", id, angle, rpm, torque)
        status = Float32MultiArray(data=[id,angle,rpm,torque])
        self.command_pub.publish(status)

    def run(self):
        rate = rospy.Rate(30)
        try:
            while not rospy.is_shutdown():
                rate.sleep()
                self.publish_status()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    controller = MotorLowLevelControl()
    controller.run()

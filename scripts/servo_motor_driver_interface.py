#!/usr/bin/python3

import rospy
import time
import os
import can
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension, Int32
import numpy as np

"""
3 robomaster M2006 P36 motors, with C610 can control
ids = [0x201, 0x202, 0x203] (Hex to Decimal [513, 514, 515])
CAN protocal
1. send -1A (-1000 = [0xFC,0x18]) to motor 513 (first two bytes)
, 10mA (10 = [0x00, 0x0A]) to motor 514 (second two bytes), and 100 mA (100 = [0x00, 0x64]) to motor 515 (third two bytes)
data = [0xFC,0x18,0x00,0x0A,0x00,0x64,0x00,0x00]
2. receive data
 angle: (data[0]*156+data[1])/8191*360
 speed: data[2]*256+data[3]
 torque: data[4]*256+data[5]
"""
# 1 turn for 2 mm

def GetS16(val):
    if val < 0x8000:
        return val
    else:
        return (val-0x10000)

def DecToHex16(val):
    """
    Convert Decimal to Hex
    Return hex, high, and low
    example:
        Input: val = 513
        Return: hexs = 0201, high = 0x02, low = 0x01
    """
    OFFSET = 1 << 16
    MASK = OFFSET - 1
    hexs = '%04x' % (val + OFFSET & MASK)
    high = '0x'+hexs[0:2]
    low = '0x'+hexs[2:4]
    return hexs, high, low

"""
Motor Control
"""
class MotorControl:
    def __init__(self, id, curr_level=3):
        self.curr_level = curr_level
        self.id = id # 513-515
        self.initialize()

    def initialize(self):
        mid,h,l = DecToHex16(self.id)
        filters = [{"can_id":int(mid,16),"can_mask":0xFFFF,"extended":False}]
        self.bus = can.interface.Bus(bustype='socketcan',channel='can0',bitrate=1000000,can_filters=filters)

    def move(self,data):
        change = np.sign(data)
        val = abs(data)
        for i in range(val):
            self.send_msg(change*self.curr_level)
            rospy.sleep(0.05)
        self.send_msg(0)

    def send_msg(self,curr):
        _,hexh,hexl = DecToHex16(curr*1000)
        data = [0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        if self.id == 513:
            data = [int(hexh,16),int(hexl,16),0x00,0x00,0x00,0x00,0x00,0x00]
        elif self.id == 514:
            data = [0x00,0x00,int(hexh,16),int(hexl,16),0x00,0x00,0x00,0x00]
        elif self.id == 515:
            data = [0x00,0x00,0x00,0x00,int(hexh,16),int(hexl,16),0x00,0x00]
        msg = can.Message(arbitration_id=512, is_extended_id=False, data=data)
        self.bus.send(msg)

class ServoMotorLowLevelControl:
    def __init__(self):
        rospy.loginfo("Setting up the node")
        rospy.init_node("servo_motor_ros_interface", anonymous=True)
        self.sub1 = rospy.Subscriber('/robo1_cmd',Int32,self.motor1_cb)
        self.sub2 = rospy.Subscriber('/robo2_cmd',Int32,self.motor2_cb)
        self.sub3 = rospy.Subscriber('/robo3_cmd',Int32,self.motor3_cb)
        self.m1ctrl = MotorControl(513)
        self.m2ctrl = MotorControl(514)
        self.m3ctrl = MotorControl(515)

    def motor1_cb(self,data):
        self.m1ctrl.move(data.data)

    def motor2_cb(self,data):
        self.m2ctrl.move(data.data)

    def motor3_cb(self,data):
        self.m3ctrl.move(data.data)

if __name__ == '__main__':
    controller = ServoMotorLowLevelControl()

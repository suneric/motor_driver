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
    def __init__(self, id):
        self.id = id # 513-515
        self.total_angle = 0
        self.angle = 0
        self.offset_angle = 0
        self.last_angle = 0
        self.rpm = 0
        self.curr = 0
        self.round_cnt = 0
        self.initialize()

    def initialize(self):
        mid,h,l = DecToHex16(self.id)
        filters = [{"can_id":int(mid,16),"can_mask":0xFFFF,"extended":False}]
        self.bus = can.interface.Bus(bustype='socketcan',channel='can0',bitrate=1000000,can_filters=filters)
        msg = self.bus.recv()
        self.angle = GetS16(msg.data[0]*256+msg.data[1])
        self.offset_angle = self.angle

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

    def motor_measure(self,msg):
        self.last_angle = self.angle
        self.angle = GetS16(msg.data[0]*256+msg.data[1])
        self.rpm = GetS16(msg.data[2]*256+msg.data[3])
        self.curr = GetS16(msg.data[4]*256+msg.data[5])*5./16384.
        if self.angle-self.last_angle > 4096:
            self.round_cnt -= 1
        elif self.angle-self.last_angle < -4096:
            self.round_cnt += 1
        self.total_angle = self.round_cnt*8192+self.angle-self.offset_angle

    def motor_total_angle(self):
        res1, res2, delta = 0, 0, 0
        if self.angle < self.last_angle:
            res1 = self.angle + 8192 - self.last_angle
            res2 = self.angle - self.last_angle
        else:
            res1 = self.angle - 8192 - self.last_angle
            res2 = self.angle - self.last_angle
        if abs(res1) < abs(res2):
            delta = res1
        else:
            delta = res2
        self.total_angle += delta
        self.last_angle = self.angle

class ServoMotorLowLevelControl:
    def __init__(self):
        rospy.loginfo("Setting up the node")
        rospy.init_node("servo_motor_ros_interface", anonymous=True)
        self.cmd_sub_1 = rospy.Subscriber('/robo1_cmd',Int32, self.motor1_cb) # -,0,+
        self.cmd_sub_2 = rospy.Subscriber('/robo2_cmd',Int32, self.motor2_cb)
        self.cmd_sub_3 = rospy.Subscriber('/robo3_cmd',Int32, self.motor3_cb)
        self.m1ctrl = MotorControl(513)
        self.m2ctrl = MotorControl(514)
        self.m3ctrl = MotorControl(515)
        self.curr = 3 # 0-10 A

    def motor1_cb(self,data):
        change = np.sign(data.data)
        self.m1ctrl.send_msg(change*self.curr)

    def motor2_cb(self,data):
        change = np.sign(data.data)
        self.m2ctrl.send_msg(0)

    def motor3_cb(self,data):
        change = np.sign(data.data)
        self.m3ctrl.send_msg(change*self.curr)

    def publish_status(self):
        motors = [self.m1ctrl, self.m2ctrl, self.m3ctrl]
        for m in motors:
            status = Float32MultiArray(data=[m.id, m.angle/8192*360, m.rpm, m.curr, m.total_angle/8192*360])
            self.status_pub.publish(status)

    def run(self):
        rate = rospy.Rate(100)
        try:
            while not rospy.is_shutdown():
                self.publish_status()
                rate.sleep()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    controller = ServoMotorLowLevelControl()
    controller.run()

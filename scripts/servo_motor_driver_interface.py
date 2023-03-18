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

class ServoMotorLowLevelControl:
    def __init__(self):
        self.bus = can.interface.Bus(bustype='socketcan',channel='can0',bitrate=1000000)
        rospy.loginfo("Setting up the node")
        rospy.init_node("servo_motor_ros_interface", anonymous=True)
        self.status_pub = rospy.Publisher('/robomotor_status', Float32MultiArray, queue_size=1)
        self.speed_sub = rospy.Subscriber('/robomotor_speed', Int32, self.motor_speed_cb) # +,0,-
        self.cmd_sub_1 = rospy.Subscriber('/robo1_cmd',Int32, self.motor1_cb) # -,0,+
        self.cmd_sub_2 = rospy.Subscriber('/robo2_cmd',Int32, self.motor2_cb)
        self.cmd_sub_3 = rospy.Subscriber('/robo3_cmd',Int32, self.motor3_cb)
        self.speed = 5 # 0~10 1 for 1 A
        self.m1s = None
        self.m2s = None
        self.m3s = None

    def motor_speed_cb(self, data):
        # print(data,type(data),data.data)
        change = np.sign(data.data)
        level = self.speed + 1.0*change
        if level > 10.0:
            self.speed = 10.0
        elif level < 0.0:
            self.speed = 0.0
        else:
            self.speed = level
        # print("speed level (0-10) is {}".format(self.speed))

    def tohex16(self,val): # convert Decimal to hex
        OFFSET = 1 << 16
        MASK = OFFSET - 1
        hexs = '%04x' % (val + OFFSET & MASK)
        high = '0x'+hexs[0:2]
        low = '0x'+hexs[2:4]
        return high, low

    def adjust_angle(self, angle, change, start):
        if change > 0 and angle < start:
            angle += 360
        elif change < 0 and angle > start:
            angle -= 360

    def motor1_cb(self,data):
        start = self.m1s[0]
        change = np.sign(data.data)
        self.send_msg(change*self.speed,1)
        while abs(self.adjust_angle(self.m1s[0],change,start) - start) < 180:
            self.send_msg(change*self.speed,1)
        self.send_msg(0,1)

    def motor2_cb(self,data):
        change = np.sign(data.data)
        self.send_msg(change, self.speed, 2)

    def motor3_cb(self,data):
        change = np.sign(data.data)
        self.send_msg(change, self.speed, 3)

    def send_msg(self, val, motor_id):
        hexh,hexl = self.tohex16(val*1000)
        cmd = [0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        if motor_id == 1:
            cmd = [int(hexh,16),int(hexl,16),0x00,0x00,0x00,0x00,0x00,0x00]
        elif motor_id == 2:
            cmd = [0x00,0x00,int(hexh,16),int(hexl,16),0x00,0x00,0x00,0x00]
        elif motor_id == 3:
            cmd = [0x00,0x00,0x00,0x00,int(hexh,16),int(hexl,16),0x00,0x00]
        msg = can.Message(arbitration_id=512, is_extended_id=False, data=cmd)
        self.bus.send(msg)

    def motor_status(self, msg):
        id = msg.arbitration_id
        angle = self.get_s16(msg.data[0]*256+msg.data[1]) / 8191 * 360
        rpm = self.get_s16(msg.data[2]*256+msg.data[3])
        torque = self.get_s16(msg.data[4]*256+msg.data[5])
        if int(id) == 513:
            self.m1s = (angle, rpm, torque)
        elif int(id) == 514:
            self.m2s = (angle, rpm, torque)
        elif int(id) == 515:
            self.m3s = (angle, rpm, torque)
        return id, angle, rpm, torque

    def get_s16(self, val):
        if val < 0x8000:
            return val
        else:
            return (val-0x10000)

    def publish_status(self):
        msg = self.bus.recv(100)
        id, angle, rpm, torque = self.motor_status(msg)
        status = Float32MultiArray(data=[id, angle, rpm, torque, self.speed])
        self.status_pub.publish(status)

    def run(self):
        if self.bus == None or self.bus.recv(100) == None:
            print("no can bus or motor detected")
            return
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

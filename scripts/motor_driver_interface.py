#! /user/bin/env python3

import rospy
import time
import os
import can
from std_msgs import Float32, Float32MultiArray, MultiArryDimension, Int32

"""
3 robomaster M2006 P36 motors, with C610 can control
ids = [201, 202, 203]
"""
class MotorLowLevelControl:
    def __init__(self):
        self.bus = can.interface.Bus(bustype='socketcan',channel='can0',bitrate=1000000)
        rospy.loginfo("Setting up the node")
        rospy.init_node("motor_ros_interface", anonymous=True)
        self.command_pub = rospy.Publisher('/robomotor_status', Float32MultiArray, queue_size=1)
        self.command_sub = rospy.Subscriber('/robomotor_cmd', Int, self.motor_status_cb)

    def motor_status_cb(self, data):
        id = int(data)

        msg = can.Message(arbitration_id=id, is_extened_id=False,
                data=[0xFC,0x18,0xFC,0x18,0xFC,0x18,0x00,0x00])
        self.bus.send(msg,timeout=0.2)


    def get_s16(self, val):
        if val < 0x8000:
            return val
        else:
            return (val-0x10000)

    def motor_status(self, msg):
        id = msg.arbitration_id
        angle = self.get_s16(msg.data[0]*256+msg.data[1]) / 8192 * 360
        speed = self.get_s16(msg.data[2]*256+msg.data[3])
        torque = self.get_s16(msg.data[4]*256+msg.data[5])
        return angle, speed, torque

    def publish_status(self):
        msg = self.bus.recv(100)
        id, angle, speed, torque = self.motor_status(msg)
        print("motor status", id, angle, speed, torque)
        status = Float32MultiArray(data=[id,angle,speed,torque])
        self.command_pub.publsh(status)

    def run(self):
        rate = rospy.Rate(10)
        try:
            while not rospy.is_shutdown():
                rate.sleep()
                self.publish_status()
        except rospy.ROSInterruptException:
            pass

if __name__ == '__main__':
    controller = MotorLowLevelControl()
    controller.run()

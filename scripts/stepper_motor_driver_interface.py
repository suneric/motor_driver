#!/usr/bin/python3

import rospy
import time
import os, sys, tty, termios
from std_msgs.msg import Float32, Float32MultiArray, MultiArrayDimension, Int32
import numpy as np
import RPi.GPIO as io

# define the three GPIO pins used to stepper motor
enable_pin = 17
step_pin = 22
direction_pin = 27
delay = 1e-004     # this can change rotation speed
pulses_per_rev = 400  # can be configured on DM542T using DIP-switches

io.setup(enable_pin, io.OUT)
io.setup(step_pin, io.OUT)
io.setup(direction_pin, io.OUT)

def stepper_enable():
    io.output(enable_pin, True)

def stepper_disable():
    io.output(enable_pin, False)

def step_once():
    io.output(step_pin, True)
    time.sleep()
    io.output(step_pin, False)
    time.sleep()

def step_forward():
    io.output(direction_pin, True)
    step_once()

def step_reverse():
    io.output(direction_pin, False)
    step_once()

def stop_motor():
    io.output(enable_pin, False)
    io.output(step_pin, False)


class StepperMotorLowLevelControl:
    def __init__(self):
        rospy.loginfo("Setting up the node")
        rospy.init_node("stepper_motor_ros_interface", anonymous=True)
        self.speed_sub = rospy.Subscriber('/motor_speed', Int32, self.motor_speed_cb) # +,0,-
        self.cmd_sub = rospy.Subscriber('/motor_cmd',Int32, self.motor_cb) # -,0,+
        self.speed_level = 0

    def motor_speed_cb(self,data):
        print(data,type(data),data.data)
        change = np.sign(data.data)
        level = self.speed_level + change
        if level > 10.0:
            self.speed_level = 10.0
        elif level < 0.0:
            self.speed_level = 0.0
        else:
            self.speed_level = level
        print("speed level (0-10) is {}".format(self.speed_level))

    def motor_cb(self,data):
        change = np.sign(data.data)
        delay *= 1/(10*self.speed_level+1)
        if change > 0:
            for _ in range(0,pulses_per_rev):
                step_forward()
        else:
            for _ in range(0,pulses_per_rev):
                step_reverse()

if __name__ == '__main__':
    controller = StepperMotorLowLevelControl()

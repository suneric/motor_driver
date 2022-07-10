#!usr/bin/python3

import time, os, sys, tty, termios
import can

class ServoMotorControl:
    def __init__(self):
        self.bus = can.interface.Bus(bustype='socketcan', channel='can0', bitrate=1000000)
        self.speed = 1

    def tohex16(self,val):
        OFFSET = 1 << 16
        MASK = OFFSET - 1
        hexs = '%04x' % (val + OFFSET & MASK)
        high = '0x' + hexs[0:2]
        low = '0x' + hexs[2:4]
        return high, low

    def forward(self):
        h,l = self.tohex16(self.speed*1000)
        cmd = [int(h,16),int(l,16),int(h,16),int(l,16),int(h,16),int(l,16),0x00,0x00]
        msg = can.Message(arbitration_id=512,is_extended_id=False,data=cmd)
        self.bus.send(msg)

    def reverse(self):
        h,l = self.tohex16(-self.speed*1000)
        cmd = [int(h,16),int(l,16),int(h,16),int(l,16),int(h,16),int(l,16),0x00,0x00]
        msg = can.Message(arbitration_id=512,is_extended_id=False,data=cmd)
        self.bus.send(msg)

    def setSpeed(self,speed):
        self.speed = speed

    def stop(self):
        cmd = [0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00]
        msg = can.Message(arbitration_id=512,is_extended_id=False,data=cmd)
        self.bus.send(msg)


def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

if __name__ == '__main__':
    controller = ServoMotorControl()

    print("q/e: speed change")
    print("f/r: steering")
    print("s: stop")
    print("x: exit")

    while True:
        char = getch()
        if char == "f":
            controller.forward()
        elif char == "r":
            controller.reverse()
        elif char == "s":
            controller.stop()
        elif char == "q":
            speed = controller.speed + 1
            if speed > 10:
                speed = 10
            controller.setSpeed(speed)
            print("current speed level is ", controller.speed)
        elif char = "e":
            speed = controller.speed - 1
            if speed < 0:
                speed = 0
            controller.setSpeed(speed)
            print("current speed level is ", controller.speed)
        elif char == "x":
            print("exit")
            break

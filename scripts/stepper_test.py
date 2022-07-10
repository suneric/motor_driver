import RPi.GPIO as io
io.setmode(io.BCM)
import sys, tty, termios, time

# This block of code defines the three GPIO pins used for stepper motor
motor_enable_pin = 17
motor_direction_pin = 27
motor_step_pin = 22

delay = 1e-5 # by playing with this delay you can change the rotational speed
pulse_per_rev = 400 # this can be configured on the driver using the DIP-switches

io.setup(motor_enable_pin, io.OUT)
io.setup(motor_direction_pin, io.OUT)
io.setup(motor_step_pin, io.OUT)

def getch():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

def stepper_enable():
    io.output(motor_enable_pin, True)

def stepper_disable():
    io.output(motor_enable_pin, False)

def step_once():
    io.output(motor_step_pin, True)
    time.sleep(delay)
    io.output(motor_step_pin, False)
    time.sleep(delay)

def step_forward():
    io.output(motor_direction_pin, True)
    step_once()

def step_reverse():
    io.output(motor_direction_pin, False)
    step_once()

if __name__ == '__main__':

    io.output(motor_enable_pin, False)
    io.output(motor_step_pin, False)

    print("q/e: speed change")
    print("f/r: steering")
    print("s: stop")
    print("x: exit")

    while True:
        char=getch()
        if char == "e":
            stepper_enable()
        elif char == "d":
            stepper_disable()
        elif char == "f":
            step_forward()
        elif char == "r":
            step_reverse()
        elif char == "g":
            for x in range(0, pulse_per_rev):
                step_forward()
        elif char == "t":
            for x in range(0, pulse_per_rev):
                step_reverse()
        elif char == "x":
            print("Exit")
            break

        char = ""

    io.cleanup()

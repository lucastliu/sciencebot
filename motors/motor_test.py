import time

import RPi.GPIO as GPIO
import time
from SerialMotor import SerialMotor
from motor_constants import *


print('Start Test')

sm = SerialMotor("/dev/ttyACM1")
time.sleep(2)

#  wheel power, -1 to 1
left_power = .6
right_power = -.6

sm.set_motor(3, left_power)  # left
sm.set_motor(4, right_power)  # right
time.sleep(1)

sm.set_motor(3, 0)
sm.set_motor(4, 0)
time.sleep(.5)

sm.set_motor(3, -1*left_power)
sm.set_motor(4, -1*right_power)
time.sleep(1)


sm.set_motor(3, 0)
sm.set_motor(4, 0)

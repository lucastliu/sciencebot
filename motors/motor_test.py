# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at

import time

import RPi.GPIO as GPIO
import time
from SerialMotor import SerialMotor
from motor_constants import *


print('Start Test')

sm = SerialMotor("/dev/ttyACM1")
time.sleep(2)

#  convert desired velocity to wheel power, -1 to 1
#  hard cap top and bottom ranges
#  only one formula for now
left_power = .6
right_power = -.6


print('2')
sm.set_motor(3, left_power)  # left
sm.set_motor(4, right_power)  #right
time.sleep(1) #about 3/4 turn at .6, 2sec



sm.set_motor(3, 0)  #left
sm.set_motor(4, 0)  # right
time.sleep(.5)

sm.set_motor(3, -1*left_power)  # left
sm.set_motor(4, -1*right_power)  #right
time.sleep(1) #about 3/4 turn at .6, 2sec



sm.set_motor(3, 0)  #left
sm.set_motor(4, 0)  # right

print('3')
import time
from MotorControllerUSB import MotorControllerUSB

power = 70
x = MotorControllerUSB()
x.setSpeed(power)

#x.turnRight(90, power)
#time.sleep(1)
time.sleep(3)
x.turn(-90)
time.sleep(2)
#x.turnLeft(90, power)
#time.sleep(1)
x.turn(30)
time.sleep(2)
x.move(-1)


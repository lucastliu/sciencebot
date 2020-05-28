import time
from MotorControllerUSB import motorControllerUSB

power = 100
x = motorControllerUSB()


x.turnRight(90, power)
time.sleep(1)
x.turn(False, 90, power)
time.sleep(1)
x.turnLeft(90, power)
time.sleep(1)
x.turn(True, 90, power)
time.sleep(1)
x.move(2, power)


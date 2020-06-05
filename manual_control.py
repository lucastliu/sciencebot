import time
from motors.MotorControllerUSB import MotorControllerUSB
from imu.imu_integrated_movement import *
from camera import H7Camera

# Bot Testing Script 


# Manual Drive 
# WASD

power = 70

args = "/home/pi/pi-bno055/getbno055"

x = MotorControllerUSB()
x.setSpeed(power)
time.sleep(2)

cam = H7Camera(port_name="/dev/ttyACM0")
print("cam ready")
time.sleep(2)
cam.get_photo()

i = 0
while i  < 10:
    print(getYaw(args))
    time.sleep(1)
    i += 1

#x.turnRight(90, power)
#time.sleep(1)
time.sleep(3)
x.turn(-90)
time.sleep(2)
#x.turnLeft(90, power)
#time.sleep(1)
x.turn(30)
time.sleep(2)
x.move(-.3)


while True:
    time.sleep(1)
    z = input("Enter Command Key:\n")

    if z == 'W':
        x.move(.2)
    
    if z == 'S':
        x.move(-.2)

    if z == 'A':
        x.turn(-20)
    
    if z == 'D':
        x.move(20)

    if z == 'P':
        cam.get_photo()
        print("photo taken")
    
    if z == 'M':
        print(getYaw(args))



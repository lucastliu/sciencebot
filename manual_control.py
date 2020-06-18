import time
#from motors.SerialMotor import SerialMotor
# from .motors.MotorControllerUSB import MotorControllerUSB
# from imu.imu_integrated_movement import *
from camera.H7Camera import H7Camera

# Bot Testing Script 


# Manual Drive 
# WASD

# power = 70
# 
# args = "/home/pi/pi-bno055/getbno055"
# 
# car = MotorControllerUSB()
# car.setSpeed(power)
# time.sleep(2)
print("start")

cam = H7Camera(port_name="/dev/ttyACM3")
print("cam ready")
time.sleep(2)
cam.get_photo(name="img1.jpg")

print("one")
cam_2 = H7Camera(port_name="/dev/ttyACM2")
print("cam ready")
time.sleep(2)
cam_2.get_photo(name="img2.jpg")
print("and done")

# 
# i = 0
# while i < 3:
#     print(getYaw(args))
#     time.sleep(1)
#     i += 1
# 
# #x.turnRight(90, power)
# #time.sleep(1)
# time.sleep(3)
# car.turn(-90)
# time.sleep(2)
# #x.turnLeft(90, power)
# #time.sleep(1)
# car.turn(30)
# time.sleep(2)
# car.move(-.3)
# 
# 
# while True:
#     time.sleep(1)
#     z = input("Enter Command Key:\n")
# 
#     if z == 'W':
#         x.move(.2)
#     
#     if z == 'S':
#         x.move(-.2)
# 
#     if z == 'A':
#         car.turn(-20)
#     
#     if z == 'D':
#         car.turn(20)
# 
#     if z == 'P':
#         cam.get_photo()
#         print("photo taken")
#     
#     if z == 'M':
#         print(getYaw(args))
# 
# 

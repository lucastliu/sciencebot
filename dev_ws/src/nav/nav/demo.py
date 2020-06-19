import time
import math
from statistics import mean
from MotorControllerUSB import MotorControllerUSB
import subprocess
from subprocess import check_output
args = "/home/pi/IMU/pi-bno055/getbno055"

x = MotorControllerUSB()
degreeError = 2
maxNumChecks = 10

def getYaw(pathToIMU):
    while True:
        try:        
            yaw = subprocess.Popen([pathToIMU, "-m", "ndof"])
            yaww = check_output([pathToIMU, "-t", "eul"])
            return float(yaww.split()[1].decode('utf-8'))
        except:
            print("ERROR with IMU")

def angle_between_headings(angle_1, angle_2):
    wrapped_delta = abs(angle_1 - angle_2) % (2*math.pi)
    shortest_delta = 2*math.pi - wrapped_delta if wrapped_delta > math.pi else wrapped_delta
    sign = 1 if (angle_1 - angle_2 >= 0 and angle_1 - angle_2 <= math.pi) \
                or (angle_1 - angle_2 <= -math.pi and angle_1 - angle_2 >= -2*math.pi) else -1
    shortest_delta *= sign
    return shortest_delta


def turn_to_heading(rads):
    time.sleep(0.2)
    current_heading = getYaw(args)
    delta = math.degrees(angle_between_headings(math.radians(current_heading), rads))
    print('Current heading: ', current_heading)
    print('Delta for initial spin: ', delta)
    count = 0
    while abs(delta) > degreeError  and count < maxNumChecks:
        if(count == maxNumChecks / 2):
            x.turn(-delta + 30)
        x.turn(-delta)
        time.sleep(0.2)
        current_heading = getYaw(args)
        delta = math.degrees(angle_between_headings(math.radians(current_heading), rads))
        print('Current heading: ', current_heading)
        print('Delta for correction ', count, ' is: ', delta)
        count += 1


def movementWithTTH():    
    x.setSpeed(80)
    turn_to_heading(math.pi / 2)
    x.move(1.225)
    turn_to_heading(math.pi / 2)
    x.move(1.225)
    turn_to_heading(0)
    # facing the door
    x.move(0.8)
    turn_to_heading(-5*math.pi/180)
    x.move(0.8)
    turn_to_heading(-5*math.pi/180)
    x.move(0.8)
    turn_to_heading(-5*math.pi/180)
    x.move(0.8)
    turn_to_heading(-5*math.pi/180)
    x.move(0.5)
    turn_to_heading(-56*math.pi/180)
    x.move(0.7)
    time.sleep(5)
    x.move(-0.7)
    x.setSpeed(50)
    turn_to_heading(math.pi)
    time.sleep(1)
    x.move(-1.5)

movementWithTTH()


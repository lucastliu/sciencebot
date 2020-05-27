import subprocess
import math
from math import atan
from subprocess import check_output
from statistics import mean

args = "/home/pi/IMU/pi-bno055/getbno055"


def getYaw(pathToIMU):
    while True:
        try:
            yaw = subprocess.Popen([pathToIMU, "-m", "ndof"])
            yaww = check_output([pathToIMU, "-t", "eul"])
            return float(yaww.split()[1].decode('utf-8'))
        except:
            print("ERROR with IMU")

def queryAndProcessYaw(pathToIMU):
    results = []
    for _ in range(10):
        yaw = subprocess.Popen([pathToIMU, "-m", "ndof"])
        yaww = check_output([pathToIMU, "-t", "eul"])
        results += [float(yaww.split()[1].decode('utf-8'))]
    results.sort()
    middle_elements = results[2:8]
    return mean(middle_elements)

# print(getYaw(args))

numOfChecks = 20
x1 = 1
y1 = 1
x2 = 3
y2 = 3

def calculateRotation(initial, x1, y1, x2, y2):
    angle = atan((float(y2)-float(y1)) / (float(x2) - float(x1)))
    return 90 - math.degrees(angle)

def rotateTo(angle):
     #rotate(right, angle)
    return
    
def validateAngle(angle):
    initialDir = getYaw(args)
    #logic for initialDir + or - angle should be tested
    target = initialDir + angle
    current = getYaw(args)
    if((current >= target - 0.1*target) & (current <= target + 0.1*target)):
        print("Adjust NOT needed")
    else:
        #adjust(current, target)
        print(current)
        print(target)
        print("Adjust needed")
    return
    
def getSlope(x1, y1, x2, y2):
    return (float(y2)-float(y1))/(float(x2)-float(x1))

def generatePoints(numChecks, slope, x1, x2):
    spacing = (float(x2) - float(x1)) / numChecks
    nextX = spacing
    multiplier = 1
    allPoints = []
    currX = 0
    while currX < x2:
        currX = x1 + multiplier*spacing
        y = slope*currX
        allPoints.append(currX)
        allPoints.append(y)
        multiplier = multiplier + 1
    return allPoints


def go(allPoints, angle):
    allDist = []
    for x in range(0, len(allPoints)-3, 2):
        dist = math.sqrt(math.pow(allPoints[x+2] - allPoints[x], 2) + math.pow(allPoints[x+3] - allPoints[x+1], 2))
        allDist.append(dist)
    # for dist in allDist:
        # Forward(dist)
        # validateAngle(angle)
    return allDist

# # Step 0
# initialDir = getYaw(args)
#
# # Step 1 Given x1, y1, x2, y2
# angle = calculateRotation(initialDir, x1, y1, x2, y2)
# print(angle)
# rotateTo(angle)
# testRotate = input()
# validateAngle(angle)
#
# # Step 2
# path = getSlope(x1, y1, x2, y2)
# allPoints = generatePoints(numOfChecks, path, x1, x2)
# print(allPoints)
#
# # Step 3
# allDist = go(allPoints, angle)
# print(allDist)
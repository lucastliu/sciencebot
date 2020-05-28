import math
import time

from bots import *
from camera import H7Camera

from error_constants import *


def dock_v3():
    print('Dock_v3')
    autobot = CamBot()
    time.sleep(0.5)

    v = H7Camera(port_name="/dev/ttyACM0")
    print("cam ready") 
    # This runs
#     iii = 0
#     while iii < 6:
#         while v.get_tag_present() and v.get_z() > 20:
#             if v.get_tag_present():
#                 print(v.get_test())
#                 print(v.get_z())
#                 print(v.get_x_offset())
#         iii += 1
# -------------------------------
    L = 0
    T = math.pi/6
    while not v.get_tag_present():
        print('looking for tag')

    print("proceeding")
#     if v.get_tag_present():
#         while not v.get_trust_reading():
#             print("trust check")
#             tug.move(10)

    offset = v.get_x_offset()
    print("offset :" + str(offset))
    # turn 15 degrees towards tag
    print("turning")
    time.sleep(3)
    if offset < 0:
        autobot.turnLeft(T)
        L = 1
    else:
        autobot.turnRight(T)
        
    tag_dist = v.get_z()
    distance = distance_to_travel_for_perp_intercept(autobot, v, tag_dist, offset)

    if(distance < 25):
        distance += 10
    if(distance < 40):
        distance += 10
    if(distance > 70):
        distance = distance - 10
    
    print('distance: '+ str(distance))
    time.sleep(3)
    if(distance > 80):
        print("bad distance: "+ str(distance))
        return
    
    autobot.move(distance)
    time.sleep(2)
    
    if L:
        autobot.turnRight(T)
    else:
        autobot.turnLeft(T)
    
    #turn_to_center_on_tag_real(tug, v)
#     time.sleep(0.2)
#     off_1 = v.get_x_offset()
#     print("offset: " + str(off_1))
#     rads_off_by = (off_1 / X_OFFSET_MAX) * (fov_rad / 2)
#     print('Delta for initial spin: ', math.degrees(rads_off_by))
# 
#     if abs(rads_off_by) < 4:
#         rads_off_by *= 2
#     if rads_off_by < 0:
#         tug.turnLeft(abs(rads_off_by))
#     else:
#         tug.turnRight(abs(rads_off_by))   
    time.sleep(1)
    
    while(tag_dist > 20):
        autobot.move(10)
        tag_dist = v.get_z()
        print(tag_dist)
        
    print(tag_dist)
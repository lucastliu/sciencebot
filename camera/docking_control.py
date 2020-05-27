import math
import time

from eTaxi_Lucas import eTaxi_Lucas
from H7Camera import H7Camera

X_OFFSET_MAX = 80
CM_PER_MOVE = 20


ACCEPTABLE_TURN_ERROR_DEGREES = 2
ACCEPTABLE_TURN_ERROR = math.radians(ACCEPTABLE_TURN_ERROR_DEGREES)
MAX_NUM_TURN_ADJUSTMENTS = 20


fov_degrees = 70
fov_rad = math.radians(fov_degrees)
search_turn_mag_degrees = fov_degrees/2


def dock_v2():
    print('Dock_v1')
    tug = eTaxi_Lucas()
    time.sleep(0.5)
    
    v = H7Camera(port_name="/dev/ttyACM0")


# This runs
    iii = 0
    while iii < 6:
        while v.get_tag_present() and v.get_z() > 20:
            if v.get_tag_present():
                print(v.get_test())
                print(v.get_z())

                print(v.get_x_offset())
        iii += 1
# -------------------------------



# This does not run

    while not (v.get_trust_reading() and v.get_z() < 150):
        if v.get_tag_present():
            #print("CM1")
            #tug.move(CM_PER_MOVE)
            offset_current = v.get_x_offset()
            print(offset_current)

        else:
            print('attempting to find tag')

# ------------------------------------------

## you cannot run 2 heavy computations in a single while loop

    while not v.get_tag_present():
       print("No tag")
       
    if v.get_tag_present():
        print("Tag is present")
        offset = v.get_x_offset()
        print('offset: ', offset)
        rads_off_from_tag_heading = (offset / X_OFFSET_MAX) * (fov_rad / 2)
        #print('degree turn: ', math.degrees(rads_off_from_tag_heading))
        # turn bot so tag is symmetrically in opposite side of FOV
        if rads_off_from_tag_heading < 1000:
            if offset < 0:
                tug.turnLeft(2*abs(rads_off_from_tag_heading))
            else:
                tug.turnRight(2*abs(rads_off_from_tag_heading))
        else:
            print('crazy turn requested')

    while not (v.get_trust_reading() and v.get_z() < 150):
        time.sleep(3)
        if v.get_tag_present():
            print("CM1")
            #tug.move(CM_PER_MOVE)
            offset_current = v.get_x_offset()
            print(offset_current)
            offset_delta = -offset - offset_current
            #rads_off_from_tag_heading = (offset_delta / X_OFFSET_MAX) * (fov_rad / 2)
            #print('offset-delta: ', offset_delta)
            if rads_off_from_tag_heading < 1000:
                if offset_delta > 0:
                    tug.turnRight(abs(rads_off_from_tag_heading))
                else:
                    tug.turnLeft(abs(rads_off_from_tag_heading))
            else:
                print('crazy turn requested')
            time.sleep(1)
        else:
            print('attempting to find tag')
            if offset < 0:
                tug.turnRight(search_turn_mag_degrees)
            else:
                tug.turnLeft(search_turn_mag_degrees)
    
    time.sleep(5)
    final_z = v.get_z()
    distance = distance_to_travel_for_perp_intercept(tug, v, final_z)
    while distance > 500:
        print('bad z: ', distance)
        time.sleep(3)
        distance = distance_to_travel_for_perp_intercept(tug, v, final_z)

    print("m1")
    print('alignment to perp move dist: ', distance)
    time.sleep(5)
    #tug.move(distance)
    #tug.turn_to_heading(0)
    # tag_z_distance = tug.cameras[0].get_z()
    # print("m2")
    # tug.move(-tag_z_distance*2/3)



def dock_v3():
    print('Dock_v3')
    tug = eTaxi_Lucas()
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
        tug.turnLeft(T)
        L = 1
    else:
        tug.turnRight(T)
        
    tag_dist = v.get_z()
    distance = distance_to_travel_for_perp_intercept(tug, v, tag_dist, offset)

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
    
    tug.move(distance)
    time.sleep(2)
    
    if L:
        tug.turnRight(T)
    else:
        tug.turnLeft(T)
    
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
        tug.move(10)
        tag_dist = v.get_z()
        print(tag_dist)
        
    print(tag_dist)
    
def straight70():
    print('Straight 70')
    tug = eTaxi_Lucas()
    time.sleep(0.5)

    v = H7Camera(port_name="/dev/ttyACM0")
    print("cam ready")
    
    while not v.get_tag_present():
        print('looking for tag')
        
    tag_dist = v.get_z()
    print("final approach:"+str(tag_dist))
    time.sleep(4)
    
    while(tag_dist > 20):
        tug.move(10)
        tag_dist = v.get_z()
        print(tag_dist)
    
    print(tag_dist)
        
# all angles should be in radians
def distance_to_travel_for_perp_intercept(tug, v, z_dist, offset):
    
    while not v.get_tag_present():
        print("No Tag")
        
    if 1:
        time.sleep(0.5)
        distance_to_tag = z_dist
        print('z to tag: ', distance_to_tag)
#         offset = v.get_x_offset()
        print('Internal offset' + str(offset))
    
        theta_1 = (offset / X_OFFSET_MAX) * (fov_rad / 2)
        psi_2 = abs(tug.angle_between_headings(0, math.pi/6))
        distance_1 = distance_to_tag * math.cos(theta_1)
        print('distance_1: ', distance_1)
        if psi_2 < math.pi/2:
            distance_2 = -(distance_to_tag * math.sin(theta_1) * math.tan((math.pi/2) - psi_2))
            print('distance_2.1: ', distance_2)
        else:
            psi_1 = math.pi - psi_2
            distance_2 = (distance_to_tag * math.sin(theta_1) * math.tan((math.pi/2) - psi_1))
            print('distance_2.2: ', distance_2)

        return distance_1 + distance_2
    return 0

# straight70()


def turn_to_center_on_tag_real(bot, vision):
    time.sleep(0.2)
    off_1 = vision.get_x_offset()
    print("offset: " + str(off_1))
    rads_off_by = (off_1 / X_OFFSET_MAX) * (fov_rad / 2)
    print('Delta for initial spin: ', math.degrees(rads_off_by))
    count = 0
    while abs(rads_off_by) > ACCEPTABLE_TURN_ERROR and count < MAX_NUM_TURN_ADJUSTMENTS:
        if abs(rads_off_by) < 4:
            rads_off_by *= 2
        if rads_off_by < 0:
            bot.turnLeft(abs(rads_off_by))
        else:
            bot.turnRight(abs(rads_off_by))

        time.sleep(0.2)
        off_2 = vision.get_x_offset()
        print("offset: " + str(off_2))
        rads_off_by = (off_2 / X_OFFSET_MAX) * (fov_rad / 2)
        print('Delta for initial spin: ', math.degrees(rads_off_by))


def turn_to_center_on_tag(bot, vision):
    time.sleep(0.2)
    off_1 = vision.get_x_offset()
    print("offset: " + str(off_1))
    rads_off_by_1 = (off_1 / X_OFFSET_MAX) * (fov_rad / 2)
    print('Delta for initial spin: ', math.degrees(rads_off_by_1))

#     while abs(rads_off_by) > ACCEPTABLE_TURN_ERROR and count < MAX_NUM_TURN_ADJUSTMENTS:
    if abs(rads_off_by_1) < 4:
        rads_off_by_1 *= 2
    if rads_off_by_1 < 0:
        bot.turnLeft(rads_off_by_1)
    else:
        bot.turnRight(rads_off_by_1)

    time.sleep(0.2)
    off_2 = vision.get_x_offset()
    print("offset: " + str(off_2))
    rads_off_by_2 = (off_2 / X_OFFSET_MAX) * (fov_rad / 2)
    print('Delta for initial spin: ', math.degrees(rads_off_by_2))
    if abs(rads_off_by_2) <= ACCEPTABLE_TURN_ERROR:
        return
    if abs(rads_off_by_2) < 4:
        rads_off_by_2 *= 2
    if rads_off_by_2 < 0:
        bot.turnLeft(rads_off_by_2)
    else:
        bot.turnRight(rads_off_by_2)

    time.sleep(0.2)
    off_3 = vision.get_x_offset()
    print("offset: " + str(off_3))
    rads_off_by_3 = (off_3 / X_OFFSET_MAX) * (fov_rad / 2)
    print('Delta for initial spin: ', math.degrees(rads_off_by_3))
    if abs(rads_off_by_3) <= ACCEPTABLE_TURN_ERROR:
        return
    if abs(rads_off_by_3) < 4:
        rads_off_by_3 *= 2
    if rads_off_by_3 < 0:
        bot.turnLeft(rads_off_by_3)
    else:
        bot.turnRight(rads_off_by_3)

    time.sleep(0.2)
    off_4 = vision.get_x_offset()
    print("offset: " + str(off_4))
    rads_off_by_4 = (off_4 / X_OFFSET_MAX) * (fov_rad / 2)
    print('Delta for initial spin: ', math.degrees(rads_off_by_4))
    if abs(rads_off_by_4) <= ACCEPTABLE_TURN_ERROR:
        return
    if abs(rads_off_by_4) < 4:
        rads_off_by_4 *= 2
    if rads_off_by_4 < 0:
        bot.turnLeft(rads_off_by_4)
    else:
        bot.turnRight(rads_off_by_4)

    return
#     offset = v.get_x_offset()
#     print("offset: " + str(offset))
#     rads_off_by = (offset / X_OFFSET_MAX) * (fov_rad / 2)
#     print('Delta for correction ', count, ' is: ', rads_off_by)
#     count += 1
#     if count == MAX_NUM_TURN_ADJUSTMENTS:
#         print('HIT MAX NUMBER OF TURN ADJUSTMENTS')


# 
# tug = eTaxi_Lucas()
# # time.sleep(0.5)
# # 
# v = H7Camera(port_name="/dev/ttyACM0")
# print("cam ready")
# turn_to_center_on_tag_real(tug, v)


dock_v3()


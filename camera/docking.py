import math
import time

from eTaxi_Lucas import eTaxi_Lucas
from H7Camera import H7Camera

X_OFFSET_MAX = 80
CM_PER_MOVE = 20


TURN_SPEED = 50
MOVE_SPEED = 100


fov_degrees = 10
fov_rad = math.radians(fov_degrees)


def dock_v1():
    print('Dock_v1')
    tug = eTaxi_Lucas()
    time.sleep(0.5)
    
#     value = [tug.cameras[0].get_thread_test()].copy()
#     print('thread_test', value[0])
#     while True:
#         value = tug.cameras[0].get_thread_test()
#         print("tx1: ", value)
#         value = tug.cameras[0].get_test()
#         print("tx2: ", value)
        
    while not tug.cameras[0].get_tag_present():
        print('looking for tag')
        print('THREAD_TEST through getter: ', tug.cameras[0].get_thread_test())
    if tug.cameras[0].get_tag_present():
        print("Tag is present")
        offset = tug.cameras[0].get_x_offset()
        print('offset: ', offset)
        degrees_off_from_tag_heading = (offset/X_OFFSET_MAX)*(fov_degrees/2)
        print('degree turn: ', degrees_off_from_tag_heading)
        if offset < 0:
            tug.get_motors().turnLeft(degrees_off_from_tag_heading, TURN_SPEED)
        else:
            tug.get_motors().turnRight(degrees_off_from_tag_heading, TURN_SPEED)
        print('Bot just turned hopefully')
        while not tug.cameras[0].get_trust_reading():
            tug.move(CM_PER_MOVE)
            time.sleep(0.5)
        print('bot just moved')
        if offset < 0:
            tug.get_motors().turnRight(degrees_off_from_tag_heading, TURN_SPEED)
        else:
            tug.get_motors().turnLeft(degrees_off_from_tag_heading, TURN_SPEED)
        print('done')


dock_v1()


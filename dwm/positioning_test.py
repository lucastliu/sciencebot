import threading
import time 
from DWMTag import DWMTag
# from DWMBot import DWMBot


def main():
    tag = DWMTag(port_name="/dev/ttyACM0")
    
    x = 1
    while x < 100:
        tag.update_position()
        position = tag.get_pos()
        print('position: ', position)
        time.sleep(1)
        x = x + 1


main()
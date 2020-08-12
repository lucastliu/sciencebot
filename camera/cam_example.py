import math
import time
from H7Camera import H7Camera


"""
Basic Example of how to use H7Camera object.

Looks for AprilTag. If it is deemed close enough,
an image is taken and saved to the host device.
"""

v = H7Camera(port_name="/dev/ttyACM0")

optimal_locale = false

while(not optimal_locale):

    if not v.get_tag_present:
        print("No AprilTag Found")

    elif v.get_tag_present():
        print("Tag Found")

        if v.get_z() > 20:
            print("Tag too Far")
        else:
            v.get_photo()
            optimal_locale = true

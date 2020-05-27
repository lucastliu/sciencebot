# This example shows how to use the USB VCP class to send an image to PC on demand.
# Host Code
#
#!/usr/bin/env python2.7
import sys, serial, struct
import time


class H7Camera():
    def __init__(self, port_name="/dev/ttyACM0"):
        #Exact port name may vary
        self.port_name = port_name
        
        self.tag_present = 0
        self.x_offset = 0.0
        self.z = 999.9
        self.trust_reading = 0
        self.test = 0.0
        self.thread_test = 0

    def cam_mand(self, serialcmd):
        
        sp = serial.Serial(self.port_name, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                xonxoff=False, rtscts=False, stopbits=serial.STOPBITS_ONE, timeout=None, dsrdtr=True)
        try:
            sp.setDTR(True) # dsrdtr is ignored on Windows.
            sp.write(serialcmd.encode())
            sp.flush()
            result = struct.unpack('<L', sp.read(4))[0]
            sp.close()
            return result
        except:
            print("Serial went wrong")
            return -1

    def get_photo(self):
        serialcmd="snap"
        sp = serial.Serial(self.port_name, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
                xonxoff=False, rtscts=False, stopbits=serial.STOPBITS_ONE, timeout=None, dsrdtr=True)
        sp.setDTR(True) # dsrdtr is ignored on Windows.
        sp.write(serialcmd.encode())
        sp.flush()
        size = struct.unpack('<L', sp.read(4))[0]
        img = sp.read(size)
        sp.close()
        
        with open("img.jpg", "wb") as f:
            f.write(img)


# update calls used by eTaxi_Lucas

    def update_z(self):
        self.z = self.cam_mand("getz")

    def update_x_offset(self):
        self.x_offset = self.cam_mand("getx")

    def update_tag_present(self):
        self.tag_present = self.cam_mand("find")
        return self.tag_present

    def update_trust_reading(self):
        self.trust_reading = self.cam_mand("trst")
        return self.trust_reading
    
    def update_test(self):
        self.test = self.cam_mand("test")

    def update_thread_test(self):
        self.thread_test += 1 #only this works, arithmetic. nested function fails
        
    def update(self):
        if self.update_tag_present():
            time.sleep(0.1)
            self.update_x_offset()
            time.sleep(0.1)
            self.update_trust_reading()
            time.sleep(0.1)
            self.update_z()
            time.sleep(0.1)
        else:
            self.trust_reading = 0
        # self.update_test()
        # self.update_thread_test()
        
# get methods will be used by docking script

    def get_z(self):
        return self.cam_mand("getz")

    def get_x_offset(self):
        return self.cam_mand("getx") - 80

    def get_tag_present(self):
        return self.cam_mand("find")

    def get_trust_reading(self):
        return self.cam_mand("trst")

    def get_test(self):
        return self.cam_mand("test")
    
    def get_thread_test(self):
        return self.thread_test


# v = H7Camera(port_name="/dev/ttyACM0")
# print(v.get_tag_present())
# print(v.get_x_offset())
# while(True):
#     print(v.get_x_offset())
#  iii = 0
# while iii < 10:
#     v.update_test()
#     print(v.get_test())
#     v.update_z()
#     print(v.get_z())
#     v.update_x_offset()
#     print(v.get_x_offset())
#     iii += 1


import sys
import serial
import struct
import time


class H7Camera():
    """
    OpenMV Cam H7 Host Device Side Camera Object

    WIP
    """
    def __init__(self, port_name="/dev/ttyACM1"):
        self.port_name = port_name
        self.tag_present = 0
        self.x_offset = 0.0
        self.z = 999.9
        self.trust_reading = 0
        self.test = 0.0
        self.thread_test = 0

    def cam_mand(self, serialcmd):

        sp = serial.Serial(self.port_name,
                           baudrate=115200,
                           bytesize=serial.EIGHTBITS,
                           parity=serial.PARITY_NONE,
                           xonxoff=False, rtscts=False,
                           stopbits=serial.STOPBITS_ONE,
                           timeout=None,
                           dsrdtr=True)
        try:
            sp.setDTR(True)  # dsrdtr is ignored on Windows.
            sp.write(serialcmd.encode())
            sp.flush()
            result = struct.unpack('<L', sp.read(4))[0]
            sp.close()
            return result

        except Exception as ex:
            print("Serial Communication Error")
            return -1

    def get_photo(self, name="img.jpg"):
        serialcmd = "snap"
        sp = serial.Serial(self.port_name,
                           baudrate=115200,
                           bytesize=serial.EIGHTBITS,
                           parity=serial.PARITY_NONE,
                           xonxoff=False, rtscts=False,
                           stopbits=serial.STOPBITS_ONE,
                           timeout=None,
                           dsrdtr=True)

        sp.setDTR(True)  # dsrdtr is ignored on Windows.
        sp.write(serialcmd.encode())
        sp.flush()
        size = struct.unpack('<L', sp.read(4))[0]
        img = sp.read(size)
        sp.close()

        with open(name, "wb") as f:
            f.write(img)

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
        self.thread_test += 1

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





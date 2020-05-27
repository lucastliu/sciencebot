# This example shows how to use the USB VCP class to send an image to PC on demand.
# Host Code
#
#!/usr/bin/env python2.7
import sys, serial, struct

port = '/dev/ttyACM1'
sp = serial.Serial(port, baudrate=115200, bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            xonxoff=False, rtscts=False, stopbits=serial.STOPBITS_ONE, timeout=None, dsrdtr=True)
sp.setDTR(True) # dsrdtr is ignored on Windows.
serialcmd = "snap"
sp.write(serialcmd.encode())
sp.flush()
size = struct.unpack('<L', sp.read(4))[0]
img = sp.read(size)
sp.close()

with open("img.jpg", "wb") as f:
    f.write(img)
